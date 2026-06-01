#!/usr/bin/env python3

from dataclasses import dataclass
import math
import os
from typing import Iterable, Tuple

import pandas as pd
import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool


DEFAULT_PARQUET_PATH = os.path.join(os.getcwd(), "data", "chunk-000_ee", "file-000.parquet")


@dataclass
class ArmPose:
    position: list
    quaternion_xyzw: list
    trigger: float


def _to_float_list(values: Iterable[float], expected_size: int, label: str) -> list:
    result = [float(value) for value in values]
    if len(result) != expected_size:
        raise ValueError(f"{label} must have {expected_size} values, got {len(result)}")
    return result


def parse_arm_pose_row(row, column_prefix: str) -> Tuple[ArmPose, ArmPose, Tuple[float, float]]:
    left_values = _to_float_list(row[f"{column_prefix}.left_arm"], 8, f"{column_prefix}.left_arm")
    right_values = _to_float_list(row[f"{column_prefix}.right_arm"], 8, f"{column_prefix}.right_arm")

    left = ArmPose(left_values[0:3], left_values[3:7], left_values[7])
    right = ArmPose(right_values[0:3], right_values[3:7], right_values[7])
    return left, right, (left.trigger, right.trigger)


def quaternion_xyzw_to_matrix(quaternion_xyzw: Iterable[float]) -> list:
    x, y, z, w = _to_float_list(quaternion_xyzw, 4, "quaternion_xyzw")
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]

    x /= norm
    y /= norm
    z /= norm
    w /= norm

    return [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]


def quaternion_xyzw_to_rot_vec(quaternion_xyzw: Iterable[float]) -> list:
    x, y, z, w = _to_float_list(quaternion_xyzw, 4, "quaternion_xyzw")
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return [0.0, 0.0, 0.0]

    x /= norm
    y /= norm
    z /= norm
    w /= norm

    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w

    sin_half = math.sqrt(x * x + y * y + z * z)
    if sin_half < 1e-12:
        return [0.0, 0.0, 0.0]

    angle = 2.0 * math.atan2(sin_half, w)
    return [angle * x / sin_half, angle * y / sin_half, angle * z / sin_half]


def arm_pose_to_six_d_payload(pose: ArmPose) -> list:
    rotation = quaternion_xyzw_to_matrix(pose.quaternion_xyzw)
    return pose.position + rotation[0] + rotation[1]


def arm_pose_to_rot_vec_payload(pose: ArmPose) -> list:
    return pose.position + quaternion_xyzw_to_rot_vec(pose.quaternion_xyzw)


class EeParquetPosePlayer(Node):
    def __init__(self):
        super().__init__("ee_parquet_pose_player")

        self.parquet_path = self.declare_parameter("parquet_path", DEFAULT_PARQUET_PATH).value
        self.pose_input_mode = self.declare_parameter("pose_input_mode", "quaternion").value
        self.column_prefix = self.declare_parameter("column_prefix", "action").value
        self.loop = self.declare_parameter("loop", True).value
        self.auto_enable_teleop = self.declare_parameter("auto_enable_teleop", True).value

        replay_rate_hz = float(self.declare_parameter("replay_rate_hz", 30.0).value)
        if replay_rate_hz <= 0.0:
            raise ValueError("replay_rate_hz must be > 0")

        self.rows = pd.read_parquet(self.parquet_path).to_dict("records")
        if not self.rows:
            raise ValueError(f"No rows in parquet file: {self.parquet_path}")

        self.current_idx = 0
        self.left_pub = None
        self.right_pub = None
        self.configure_pose_publishers()
        self.trigger_pub = self.create_publisher(JointState, "/joint_states", 10)

        self.teleop_client = self.create_client(SetBool, "/toggle_teleop")
        if self.auto_enable_teleop:
            self.enable_timer = self.create_timer(0.5, self.try_enable_teleop)
        else:
            self.enable_timer = None

        self.timer = self.create_timer(1.0 / replay_rate_hz, self.publish_next)
        self.get_logger().info(
            f"EE parquet replay started: path={self.parquet_path}, rows={len(self.rows)}, "
            f"mode={self.pose_input_mode}, columns={self.column_prefix}.*"
        )

    def configure_pose_publishers(self):
        if self.pose_input_mode == "quaternion":
            self.left_pub = self.create_publisher(Pose, "/control/body/arm_left_controller/pose", 10)
            self.right_pub = self.create_publisher(Pose, "/control/body/arm_right_controller/pose", 10)
            return

        if self.pose_input_mode == "six_d":
            self.left_pub = self.create_publisher(Float64MultiArray, "/control/body/arm_left_controller/pose_6d", 10)
            self.right_pub = self.create_publisher(Float64MultiArray, "/control/body/arm_right_controller/pose_6d", 10)
            return

        if self.pose_input_mode in ("rot_vec", "axis_angle"):
            self.left_pub = self.create_publisher(Float64MultiArray, "/control/body/arm_left_controller/pose_rot_vec", 10)
            self.right_pub = self.create_publisher(Float64MultiArray, "/control/body/arm_right_controller/pose_rot_vec", 10)
            return

        raise ValueError(f"Unsupported pose_input_mode: {self.pose_input_mode}")

    def try_enable_teleop(self):
        if not self.teleop_client.service_is_ready():
            self.teleop_client.wait_for_service(timeout_sec=0.0)
            return

        request = SetBool.Request()
        request.data = True
        future = self.teleop_client.call_async(request)
        future.add_done_callback(self.on_enable_done)
        self.destroy_timer(self.enable_timer)
        self.enable_timer = None

    def on_enable_done(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"Failed to enable teleop: {exc}")
            return

        if response.success:
            self.get_logger().info("Teleop enabled for EE parquet replay")
        else:
            self.get_logger().warn(f"Teleop enable request rejected: {response.message}")

    def publish_next(self):
        if self.current_idx >= len(self.rows):
            if not self.loop:
                self.get_logger().info("EE parquet replay finished")
                self.timer.cancel()
                return
            self.current_idx = 0

        left_pose, right_pose, triggers = parse_arm_pose_row(self.rows[self.current_idx], self.column_prefix)

        if self.pose_input_mode == "quaternion":
            self.left_pub.publish(self.make_pose_msg(left_pose))
            self.right_pub.publish(self.make_pose_msg(right_pose))
        elif self.pose_input_mode == "six_d":
            self.left_pub.publish(self.make_array_msg(arm_pose_to_six_d_payload(left_pose)))
            self.right_pub.publish(self.make_array_msg(arm_pose_to_six_d_payload(right_pose)))
        else:
            self.left_pub.publish(self.make_array_msg(arm_pose_to_rot_vec_payload(left_pose)))
            self.right_pub.publish(self.make_array_msg(arm_pose_to_rot_vec_payload(right_pose)))

        self.publish_triggers(triggers)
        self.current_idx += 1

    @staticmethod
    def make_pose_msg(pose: ArmPose):
        msg = Pose()
        msg.position.x, msg.position.y, msg.position.z = pose.position
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = pose.quaternion_xyzw
        return msg

    @staticmethod
    def make_array_msg(payload: list):
        msg = Float64MultiArray()
        msg.data = payload
        return msg

    def publish_triggers(self, triggers: Tuple[float, float]):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["left_trigger", "right_trigger"]
        msg.position = [float(triggers[0]), float(triggers[1])]
        self.trigger_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EeParquetPosePlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
