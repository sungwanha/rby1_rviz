#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import math

class JointStateVisualizer(Node):
    def __init__(self):
        super().__init__('joint_state_visualizer')
        
        # 1. 모든 관절 초기값 설정
        self.joint_map = {}
        self.init_joint_map()

        # 2. 퍼블리셔: RViz용 /joint_states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # 3. 구독자: 좌/우 팔 JointTrajectory
        self.sub_left = self.create_subscription(
            JointTrajectory,
            '/leader/master_arm_joint_left/joint_trajectory',
            self.trajectory_callback,
            10)

        self.sub_right = self.create_subscription(
            JointTrajectory,
            '/leader/master_arm_joint_right/joint_trajectory',
            self.trajectory_callback,
            10)

        # 4. 타이머: 50Hz (20ms)
        self.timer = self.create_timer(0.02, self.publish_joint_states)
        
        self.get_logger().info('Joint State Visualizer (Python) Node Started.')

    def init_joint_map(self):
        d2r = math.pi / 180.0

        # Torso 0~5
        for i in range(6):
            self.joint_map[f'torso_{i}'] = 0.0
        self.joint_map['torso_1'] = -30.0 * d2r
        self.joint_map['torso_2'] = 14.8019 * d2r
        self.joint_map['torso_3'] = 51.7269 * d2r

        # Head 0~1
        self.joint_map['head_0'] = 0.0
        self.joint_map['head_1'] = -3.4250 * d2r

        # Arm 0~6
        for i in range(7):
            self.joint_map[f'left_arm_{i}'] = 0.0
            self.joint_map[f'right_arm_{i}'] = 0.0
        self.joint_map['left_arm_3'] = -120 * d2r
        self.joint_map['left_arm_6'] = -90 * d2r
        self.joint_map['right_arm_3'] = -120 * d2r
        self.joint_map['right_arm_6'] = 90 * d2r
        
        # Triggers
        self.joint_map['left_trigger'] = 1.0
        self.joint_map['right_trigger'] = 1.0

    def trajectory_callback(self, msg):
        if not msg.points:
            return

        # 첫 번째 포인트를 현재 값으로 업데이트
        point = msg.points[0]
        for i, name in enumerate(msg.joint_names):
            if name in self.joint_map:
                self.joint_map[name] = point.positions[i]

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # 딕셔너리 데이터를 리스트로 변환하여 메시지에 삽입
        msg.name = list(self.joint_map.keys())
        msg.position = list(self.joint_map.values())
        
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()