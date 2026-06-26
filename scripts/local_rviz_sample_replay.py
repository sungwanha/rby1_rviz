#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, TransformStamped
from tf2_ros import TransformBroadcaster
import math


JOINT_NAMES = [
    "head_0",
    "head_1",
    "left_arm_0",
    "left_arm_1",
    "left_arm_2",
    "left_arm_3",
    "left_arm_4",
    "left_arm_5",
    "left_arm_6",
    "left_trigger",
    "left_wheel",
    "right_arm_0",
    "right_arm_1",
    "right_arm_2",
    "right_arm_3",
    "right_arm_4",
    "right_arm_5",
    "right_arm_6",
    "right_trigger",
    "right_wheel",
    "torso_0",
    "torso_1",
    "torso_2",
    "torso_3",
    "torso_4",
    "torso_5",
]

JOINT_POSITIONS = [
    0.0,
    0.05223204582750608,
    -1.671366992955112,
    0.7227517549337721,
    -0.062262913266704814,
    -1.356726270259737,
    0.7593971890427866,
    1.8858568058667586,
    -0.8177574881178958,
    0.008791924454575106,
    0.37461089157490596,
    -1.1575517746920903,
    -0.6958957001993287,
    0.5892498626019426,
    -2.1062999070886184,
    -0.9399850772966237,
    1.7839774718393338,
    -0.0032980586939541285,
    0.006085192697768749,
    0.41085118768870427,
    1.1390946444695356e-05,
    0.0872622437306629,
    -0.19197921439674728,
    0.5846782960954715,
    0.0,
    0.0,
]

LEFT_POSE = {
    "position": (-0.14087633793805562, -0.2451823720594151, 0.23272867243300133),
    "orientation": (-0.7643914119061574, 0.06213544547413849, -0.15496346317824417, 0.6227610142740525),
}

RIGHT_POSE = {
    "position": (0.13817941868020547, 0.027697111854010602, 0.26047311648602267),
    "orientation": (0.8582431796623441, 0.04523278309966772, -0.17825341971134503, -0.47916422890000754),
}

LEFT_OFFSET_XYZ = (-0.0073, -0.0051, -0.0720)
LEFT_OFFSET_RPY_DEG = (-97.58, 1.71, 15.36)
RIGHT_OFFSET_XYZ = (-0.0073, 0.0039, -0.0720)
RIGHT_OFFSET_RPY_DEG = (98.99, 0.30, 163.89)


def pose_from_sample(sample):
    msg = Pose()
    msg.position.x, msg.position.y, msg.position.z = sample["position"]
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = sample["orientation"]
    return msg


def quaternion_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quaternion_from_axis_angle(axis, angle):
    half = angle * 0.5
    scale = math.sin(half)
    return (axis[0] * scale, axis[1] * scale, axis[2] * scale, math.cos(half))


def quaternion_from_rpy_deg(roll_deg, pitch_deg, yaw_deg):
    # Matches Eigen AngleAxis Z * Y * X composition in the C++ nodes.
    qx = quaternion_from_axis_angle((1.0, 0.0, 0.0), math.radians(roll_deg))
    qy = quaternion_from_axis_angle((0.0, 1.0, 0.0), math.radians(pitch_deg))
    qz = quaternion_from_axis_angle((0.0, 0.0, 1.0), math.radians(yaw_deg))
    return quaternion_multiply(quaternion_multiply(qz, qy), qx)


def rotate_vector(quaternion, vector):
    q_vec = (vector[0], vector[1], vector[2], 0.0)
    q_inv = (-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3])
    rotated = quaternion_multiply(quaternion_multiply(quaternion, q_vec), q_inv)
    return rotated[:3]


def apply_offset_to_pose(pose, offset_xyz, offset_rpy_deg):
    pose_q = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    offset_q = quaternion_from_rpy_deg(*offset_rpy_deg)
    offset_translation = rotate_vector(offset_q, offset_xyz)
    rotated_offset = rotate_vector(pose_q, offset_translation)

    result = Pose()
    result.position.x = pose.position.x + rotated_offset[0]
    result.position.y = pose.position.y + rotated_offset[1]
    result.position.z = pose.position.z + rotated_offset[2]
    result.orientation.x, result.orientation.y, result.orientation.z, result.orientation.w = quaternion_multiply(
        pose_q, offset_q
    )
    return result


class LocalRvizSampleReplay(Node):
    def __init__(self):
        super().__init__("local_rviz_sample_replay")
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.left_pose_pub = self.create_publisher(Pose, "/sensors/proprio/arm_left/pose", 10)
        self.right_pose_pub = self.create_publisher(Pose, "/sensors/proprio/arm_right/pose", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self.publish_sample)

    def publish_sample(self):
        now = self.get_clock().now().to_msg()

        joints = JointState()
        joints.header.stamp = now
        joints.name = JOINT_NAMES
        joints.position = JOINT_POSITIONS
        self.joint_pub.publish(joints)

        left_pose = pose_from_sample(LEFT_POSE)
        right_pose = pose_from_sample(RIGHT_POSE)
        self.left_pose_pub.publish(left_pose)
        self.right_pose_pub.publish(right_pose)
        left_ee_pose = apply_offset_to_pose(left_pose, LEFT_OFFSET_XYZ, LEFT_OFFSET_RPY_DEG)
        right_ee_pose = apply_offset_to_pose(right_pose, RIGHT_OFFSET_XYZ, RIGHT_OFFSET_RPY_DEG)

        self.tf_broadcaster.sendTransform(
            [
                self.pose_to_transform(now, left_pose, "zed2i_left_camera_optical_frame", "sample_left_quaternion_pose"),
                self.pose_to_transform(now, right_pose, "zed2i_left_camera_optical_frame", "sample_right_quaternion_pose"),
                self.pose_to_transform(now, left_ee_pose, "zed2i_left_camera_optical_frame", "sample_left_ee_reconstructed"),
                self.pose_to_transform(now, right_ee_pose, "zed2i_left_camera_optical_frame", "sample_right_ee_reconstructed"),
            ]
        )

    def pose_to_transform(self, stamp, pose, parent, child):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        transform.transform.rotation = pose.orientation
        return transform


def main():
    rclpy.init()
    node = LocalRvizSampleReplay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
