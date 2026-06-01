import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class PoseToTF(Node):
    def __init__(self):
        super().__init__('pose_to_tf_relay')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/simdd/pose',
            self.listener_callback,
            10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info('🚀 Quest Pose to TF Relay Node Started!')
        self.get_logger().info('Subscribed to: /simdd/pose')

    def listener_callback(self, msg):
        # 32개의 데이터가 들어오는지 확인 (위치 3, RPY 3, 쿼터니언 4 * 3개 + 트래킹 상태 등)
        if len(msg.data) < 32:
            return

        data = msg.data
        
        # XR (OpenXR) -> ROS 좌표계 변환
        # OpenXR: X(Right), Y(Up), Z(Backward)  -> -Z가 Forward
        # ROS:    X(Forward), Y(Left), Z(Up)
        # 변환식: ROS_X = -Quest_Z, ROS_Y = -Quest_X, ROS_Z = Quest_Y

        # 1. Map -> HMD (World fixed)
        hmd_pos = [0.0, 0.0, 0.0]
        hmd_quat = [0.0, 0.0, 0.0, 1.0] # -qz, -qx, qy, qw
        # hmd_pos = [-data[2], -data[0], data[1]]
        # hmd_quat = [-data[22], -data[20], data[21], data[23]] # -qz, -qx, qy, qw
        self.broadcast_tf('link_head_2', 'hmd', hmd_pos, hmd_quat)

        # 2. HMD -> Right Controller (Relative)
        # ROS 2 Mapping: [X, Y, Z] = [-qz, -qx, qy, qw]
        r_pos = [-data[9], -data[7], data[8]]
        rx, ry, rz, rw = -data[26], -data[24], data[25], data[27]

        # Z축(Blue) 기준 반시계방향 90도 (+90 deg CCW) 회전: Q' = Q * (0, 0, sin(45), cos(45))
        # s = 0.7071, c = 0.7071
        s, c = 0.70710678118, 0.70710678118
        r_quat = [rx*c + ry*s, ry*c - rx*s, rz*c + rw*s, rw*c - rz*s]
        self.broadcast_tf('hmd', 'right_controller', r_pos, r_quat)

        # 3. HMD -> Left Controller (Relative)
        l_pos = [-data[16], -data[14], data[15]]
        lx, ly, lz, lw = -data[30], -data[28], data[29], data[31]

        # Z축(Blue) 기준 시계방향 90도 (-90 deg CCW) 회전: Q' = Q * (0, 0, -sin(45), cos(45))
        # s = -0.7071, c = 0.7071
        s, c = -0.70710678118, 0.70710678118
        l_quat = [lx*c + ly*s, ly*c - lx*s, lz*c + lw*s, lw*c - lz*s]
        self.broadcast_tf('hmd', 'left_controller', l_pos, l_quat)



    def broadcast_tf(self, parent, child, pos, quat):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        
        # Translation
        t.transform.translation.x = float(pos[0])
        t.transform.translation.y = float(pos[1])
        t.transform.translation.z = float(pos[2])
        
        # Rotation (Quaternion)
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = PoseToTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
