#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool


class SimPosePublisher(Node):
    def __init__(self):
        super().__init__('sim_pose_publisher')

        self.pose_input_mode = self.declare_parameter('pose_input_mode', 'quaternion').value
        self.auto_enable_teleop = self.declare_parameter('auto_enable_teleop', True).value
        self.phase = 0.0

        self.left_pub = None
        self.right_pub = None
        self.configure_publishers()

        self.teleop_client = self.create_client(SetBool, '/toggle_teleop')
        if self.auto_enable_teleop:
            self.enable_timer = self.create_timer(0.5, self.try_enable_teleop)
        else:
            self.enable_timer = None

        self.timer = self.create_timer(0.02, self.publish_pose)
        self.get_logger().info(f'Sim pose publisher started with pose_input_mode={self.pose_input_mode}')

    def configure_publishers(self):
        if self.pose_input_mode == 'quaternion':
            self.left_pub = self.create_publisher(Pose, '/control/body/arm_left_controller/pose', 10)
            self.right_pub = self.create_publisher(Pose, '/control/body/arm_right_controller/pose', 10)
            return

        if self.pose_input_mode == 'six_d':
            self.left_pub = self.create_publisher(Float64MultiArray, '/control/body/arm_left_controller/pose_6d', 10)
            self.right_pub = self.create_publisher(Float64MultiArray, '/control/body/arm_right_controller/pose_6d', 10)
            return

        if self.pose_input_mode in ('rot_vec', 'axis_angle'):
            self.left_pub = self.create_publisher(Float64MultiArray, '/control/body/arm_left_controller/pose_rot_vec', 10)
            self.right_pub = self.create_publisher(Float64MultiArray, '/control/body/arm_right_controller/pose_rot_vec', 10)
            return

        self.get_logger().warn(f'Unknown pose_input_mode={self.pose_input_mode}; falling back to quaternion')
        self.pose_input_mode = 'quaternion'
        self.left_pub = self.create_publisher(Pose, '/control/body/arm_left_controller/pose', 10)
        self.right_pub = self.create_publisher(Pose, '/control/body/arm_right_controller/pose', 10)

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
            self.get_logger().warn(f'Failed to enable teleop: {exc}')
            return

        if response.success:
            self.get_logger().info('Teleop enabled for sim test')
        else:
            self.get_logger().warn(f'Teleop enable request rejected: {response.message}')

    def publish_pose(self):
        self.phase += 0.02
        sway = 0.08 * math.sin(self.phase)
        lift = 0.04 * math.sin(self.phase * 0.7)

        left_position = (0.38 + lift, 0.24 + sway, 0.16)
        right_position = (0.38 + lift, -0.24 + sway, 0.16)

        if self.pose_input_mode == 'quaternion':
            self.left_pub.publish(self.make_quaternion_pose(left_position))
            self.right_pub.publish(self.make_quaternion_pose(right_position))
        elif self.pose_input_mode == 'six_d':
            self.left_pub.publish(self.make_six_d_pose(left_position))
            self.right_pub.publish(self.make_six_d_pose(right_position))
        else:
            self.left_pub.publish(self.make_rot_vec_pose(left_position))
            self.right_pub.publish(self.make_rot_vec_pose(right_position))

    @staticmethod
    def make_quaternion_pose(position):
        msg = Pose()
        msg.position.x = position[0]
        msg.position.y = position[1]
        msg.position.z = position[2]
        msg.orientation.w = 1.0
        return msg

    @staticmethod
    def make_six_d_pose(position):
        msg = Float64MultiArray()
        msg.data = [
            position[0], position[1], position[2],
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
        ]
        return msg

    @staticmethod
    def make_rot_vec_pose(position):
        msg = Float64MultiArray()
        msg.data = [position[0], position[1], position[2], 0.0, 0.0, 0.0]
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = SimPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
