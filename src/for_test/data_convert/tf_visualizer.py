import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
import os
import pandas as pd

DEFAULT_PARQUET_PATH = "/home/hsw/rby1_rviz/data/chunk-000/file-000.parquet"


def load_action_rows(parquet_path):
    df = pd.read_parquet(parquet_path)

    if "action" in df.columns:
        return [[float(value) for value in row] for row in df["action"]]

    required_cols = ["action.left_arm", "action.right_arm"]
    missing_cols = [col for col in required_cols if col not in df.columns]
    if missing_cols:
        raise ValueError(f"Missing action columns: {missing_cols}")

    return [
        [float(value) for value in list(left_arm) + list(right_arm)]
        for left_arm, right_arm in zip(df["action.left_arm"], df["action.right_arm"])
    ]


class ParquetPlayerNode(Node):
    def __init__(self):
        super().__init__('parquet_player_node')

        # 1. 파일 경로 설정
        self.parquet_path = os.environ.get("PARQUET_PATH", DEFAULT_PARQUET_PATH)
        self.action_data = load_action_rows(self.parquet_path)
        self.current_idx = 0
        self.total_len = len(self.action_data)

        # 2. 퍼블리셔 설정
        self.pub_pose_left = self.create_publisher(PoseStamped, '/simdd/pose_left', 10)
        self.pub_pose_right = self.create_publisher(PoseStamped, '/simdd/pose_right', 10)
        self.pub_joint_state = self.create_publisher(JointState, '/joint_states', 10)

        # 3. Teleop 활성화 서비스 자동 호출
        self.auto_enable_teleop = os.environ.get("AUTO_ENABLE_TELEOP", "1") != "0"
        if self.auto_enable_teleop:
            self.teleop_client = self.create_client(SetBool, 'toggle_teleop')
            self.enable_teleop()
        else:
            self.get_logger().info('Teleop 서비스 자동 호출을 건너뜁니다.')

        # 4. 타이머 설정 (IK 노드가 20ms 간격이므로 동일하게 50Hz로 퍼블리시)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info(f"데이터 재생 시작! 총 {self.total_len} 프레임이 50Hz로 재생됩니다.")

    def enable_teleop(self):
        while not self.teleop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('teleop 서비스를 기다리는 중...')
        req = SetBool.Request()
        req.data = True
        self.teleop_client.call_async(req)
        self.get_logger().info('Teleop 모드 자동 활성화 완료!')

    def timer_callback(self):
        if self.current_idx >= self.total_len:
            self.get_logger().info("재생이 끝났습니다. 처음부터 다시 반복합니다.")
            self.current_idx = 0

        row = self.action_data[self.current_idx]
        now = self.get_clock().now().to_msg()
        frame_id = "zed2i_left_camera_optical_frame"

        # 16차원 데이터 분해: [pos_l(3), quat_l(4), trig_l(1), pos_r(3), quat_r(4), trig_r(1)]
        pos_l, quat_l, trig_l = row[0:3], row[3:7], row[7]
        pos_r, quat_r, trig_r = row[8:11], row[11:15], row[15]

        # --- Left Pose ---
        msg_l = PoseStamped()
        msg_l.header.stamp = now
        msg_l.header.frame_id = frame_id
        msg_l.pose.position.x, msg_l.pose.position.y, msg_l.pose.position.z = pos_l[0], pos_l[1], pos_l[2]
        msg_l.pose.orientation.x, msg_l.pose.orientation.y, msg_l.pose.orientation.z, msg_l.pose.orientation.w = quat_l[0], quat_l[1], quat_l[2], quat_l[3]

        # --- Right Pose ---
        msg_r = PoseStamped()
        msg_r.header.stamp = now
        msg_r.header.frame_id = frame_id
        msg_r.pose.position.x, msg_r.pose.position.y, msg_r.pose.position.z = pos_r[0], pos_r[1], pos_r[2]
        msg_r.pose.orientation.x, msg_r.pose.orientation.y, msg_r.pose.orientation.z, msg_r.pose.orientation.w = quat_r[0], quat_r[1], quat_r[2], quat_r[3]

        # --- Trigger States ---
        msg_js = JointState()
        msg_js.header.stamp = now
        msg_js.name = ["left_trigger", "right_trigger"]
        msg_js.position = [float(trig_l), float(trig_r)]

        # 퍼블리시
        self.pub_pose_left.publish(msg_l)
        self.pub_pose_right.publish(msg_r)
        self.pub_joint_state.publish(msg_js)

        self.current_idx += 1

def main(args=None):
    rclpy.init(args=args)
    node = ParquetPlayerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
