import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
from scipy.spatial.transform import Rotation as R
import numpy as np

class SimDDRvizVisualizer(Node):
    def __init__(self):
        super().__init__('simdd_rviz_visualizer')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # 💡 OpenXR World -> ROS World(FLU) 회전 변환 행렬
        # ROS_X = -OpenXR_Z
        # ROS_Y = -OpenXR_X
        # ROS_Z = OpenXR_Y
        self.r_base = R.from_matrix(np.array([
            [ 0,  0, -1],
            [-1,  0,  0],
            [ 0,  1,  0]
        ]))
        
        # 구독 설정
        self.create_subscription(PoseStamped, '/simdd/pose_head', self.cb_h, 10)
        self.create_subscription(PoseStamped, '/simdd/pose_right', self.cb_r, 10)
        self.create_subscription(PoseStamped, '/simdd/pose_left', self.cb_l, 10)
        self.create_subscription(PoseStamped, '/simdd/pose_hmd', self.cb_hmd, 10)
        
        self.get_logger().info("SimDD RViz Visualizer 시작: OpenXR -> ROS 좌표계 변환 적용됨")

    def broadcast_tf(self, pose, child_frame, parent_frame, is_relative=False):
        """
        TF를 생성하고 전송하는 공통 함수
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        if not is_relative:
            # 1. 헤드/HMD (절대좌표): OpenXR 월드에서 ROS 월드로 변환
            # 위치(Translation) 매핑
            t.transform.translation.x = float(-pose.position.z) # ROS Front
            t.transform.translation.y = float(-pose.position.x) # ROS Left
            t.transform.translation.z = float(pose.position.y)  # ROS Up

            # 회전(Rotation) 매핑: (Base 행렬 * 원본 쿼터니언)
            r_raw = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
            quat = (self.r_base * r_raw).as_quat()
        else:
            # 2. 컨트롤러 (상대좌표): 이미 C++에서 부모(hmd_link) 기준으로 변환 완료됨
            # 값 그대로 1:1 직결
            t.transform.translation.x = float(pose.position.x)
            t.transform.translation.y = float(pose.position.y)
            t.transform.translation.z = float(pose.position.z)
            
            quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])

        self.tf_broadcaster.sendTransform(t)

    def cb_h(self, msg):
        # 헤드(카메라 광학 프레임)는 world를 기준으로 움직임 (절대 좌표)
        self.broadcast_tf(msg.pose, 'hmd_link', 'world', is_relative=False)

    def cb_hmd(self, msg):
        # HMD(글래스 프레임)는 world를 기준으로 움직임 (절대 좌표)
        self.broadcast_tf(msg.pose, 'real_hmd_link', 'world', is_relative=False)

    def cb_r(self, msg):
        # 오른쪽 컨트롤러는 카메라 광학 프레임(hmd_link)을 기준으로 움직임 (상대 좌표)
        self.broadcast_tf(msg.pose, 'controller_right', 'hmd_link', is_relative=True)

    def cb_l(self, msg):
        self.broadcast_tf(msg.pose, 'controller_left', 'hmd_link', is_relative=True)

def main():
    rclpy.init()
    node = SimDDRvizVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()