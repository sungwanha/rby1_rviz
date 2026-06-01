import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation as R
import threading
import time

class SimDDRvizVisualizer(Node):
    def __init__(self):
        super().__init__('simdd_rviz_visualizer')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # 목표 치수 (사용자 설정)
        self.target_dist_z = 0.252   
        self.target_dist_x = 0.146   
        self.target_height = 0.880   

        self.current_poses = {'head': None, 'left': None, 'right': None}
        self.target_offsets_rot = {'left': R.identity(), 'right': R.identity()}
        self.target_offsets_pos = {'left': np.zeros(3), 'right': np.zeros(3)}
        
        self.step = 0 
        self.is_calibrated = False
        self.last_print_time = time.time() # 출력 간격 조절용

        self.create_subscription(PoseStamped, '/simdd/pose_head', self.cb_h, 10)
        self.create_subscription(PoseStamped, '/simdd/pose_right', self.cb_r, 10)
        self.create_subscription(PoseStamped, '/simdd/pose_left', self.cb_l, 10)

        self.get_logger().info("=== 2단계 캘리브레이션 시작 ===")
        self.get_logger().info("STEP 1: 컨트롤러를 '마주 보게' 두고 [ENTER]")
        threading.Thread(target=self.wait_for_enter, daemon=True).start()

    # ... (wait_for_enter, get_world_data, calibrate_step_1, calibrate_step_2 함수는 동일)

    def wait_for_enter(self):
        while rclpy.ok():
            input()
            if not all(self.current_poses.values()):
                self.get_logger().warn("데이터 수신 중...")
                continue

            if self.step == 0:
                self.calibrate_step_1()
                self.step = 1
                self.get_logger().info("STEP 2: 컨트롤러를 '나란히' 두고 [ENTER]")
            elif self.step == 1:
                self.calibrate_step_2()
                self.is_calibrated = True
                self.print_final_offsets()

    def get_world_data(self, side):
        h, p = self.current_poses['head'], self.current_poses[side]
        h_pos = np.array([h.position.x, -h.position.z, h.position.y])
        h_rot = R.from_euler('x', 90, degrees=True) * R.from_quat([h.orientation.x, h.orientation.y, h.orientation.z, h.orientation.w])
        c_pos_world = h_pos + h_rot.apply(np.array([p.position.x, p.position.y, p.position.z]))
        c_rot_world = h_rot * R.from_quat([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return c_pos_world, c_rot_world

    def calibrate_step_1(self):
        pos_l, rot_l_w = self.get_world_data('left')
        pos_r, rot_r_w = self.get_world_data('right')

        def get_target_rot_w(my_pos, target_pos, is_right):
            y_axis = np.array([0, 0, -1]) if is_right else np.array([0, 0, 1])
            look_vec = my_pos - target_pos
            z_axis = look_vec - np.dot(look_vec, y_axis) * y_axis
            z_axis /= np.linalg.norm(z_axis)
            x_axis = np.cross(y_axis, z_axis)
            x_axis /= np.linalg.norm(x_axis)
            return R.from_matrix(np.column_stack([x_axis, y_axis, z_axis]))

        self.target_offsets_rot['right'] = rot_r_w.inv() * get_target_rot_w(pos_r, pos_l, True)
        self.target_offsets_rot['left'] = rot_l_w.inv() * get_target_rot_w(pos_l, pos_r, False)

        current_dist_z = np.linalg.norm(pos_r - pos_l)
        diff_z = (self.target_dist_z - current_dist_z) / 2.0
        diff_y_r = self.target_height - pos_r[2]
        diff_y_l = self.target_height - pos_l[2]

        self.target_offsets_pos['right'] = np.array([0.0, diff_y_r, diff_z])
        self.target_offsets_pos['left'] = np.array([0.0, -diff_y_l, diff_z])

    def calibrate_step_2(self):
        pos_l, _ = self.get_world_data('left')
        pos_r, _ = self.get_world_data('right')
        current_dist_x = np.linalg.norm(pos_r - pos_l)
        diff_x = (current_dist_x - self.target_dist_x) / 2.0
        self.target_offsets_pos['right'][0] = diff_x
        self.target_offsets_pos['left'][0] = diff_x
        self.get_logger().info(f"교정 완료: 측정거리={current_dist_x:.3f}m, 오프셋={diff_x:.3f}m")

    def broadcast_main_tfs(self):
        now = self.get_clock().now().to_msg()
        h = self.current_poses['head']
        if not h: return

        # Head TF
        self.send_t(now, 'world', 'hmd_link', [h.position.x, -h.position.z, h.position.y], 
                   (R.from_euler('x', 90, degrees=True) * R.from_quat([h.orientation.x, h.orientation.y, h.orientation.z, h.orientation.w])).as_quat())

        target_positions_world = {}

        for side in ['left', 'right']:
            p = self.current_poses[side]
            if not p: continue
            c_name = f'controller_{side}'
            self.send_t(now, 'hmd_link', c_name, [p.position.x, p.position.y, p.position.z], [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

            if self.is_calibrated:
                # Target Pose 계산 및 TF 발행
                rot_offset = self.target_offsets_rot[side]
                pos_offset_local = rot_offset.apply(self.target_offsets_pos[side])
                self.send_t(now, c_name, f'target_{side}', pos_offset_local, rot_offset.as_quat())
                
                # 거리 계산을 위해 월드 좌표상의 Target 위치 저장
                c_pos_w, c_rot_w = self.get_world_data(side)
                # 월드상 위치 = 컨트롤러월드위치 + 컨트롤러월드회전 * 로컬오프셋
                target_pos_w = c_pos_w + c_rot_w.apply(pos_offset_local)
                target_positions_world[side] = target_pos_w

        # --- 실시간 거리 출력 로직 추가 ---
        if self.is_calibrated and 'left' in target_positions_world and 'right' in target_positions_world:
            dist = np.linalg.norm(target_positions_world['right'] - target_positions_world['left'])
            
            # 너무 자주 출력되지 않도록 0.2초마다 출력
            if time.time() - self.last_print_time > 0.2:
                # \r을 사용하여 한 줄에서 숫자가 업데이트되게 함
                print(f"\r[Live Distance] Target L <-> R: {dist:.4f} m (Target: {self.target_dist_z}m)", end="")
                self.last_print_time = time.time()

    def print_final_offsets(self):
        """최종 오프셋 값을 터미널에 깔끔하게 출력"""
        print("\n" + "="*50)
        print(" FINAL OFFSET VALUES (Rigid Body Alignment)")
        print("="*50)

        for side in ['right', 'left']:
            # Position
            p = self.target_offsets_pos[side]
            # Orientation (Euler angles in degrees: Roll, Pitch, Yaw)
            # scipy의 euler는 기본적으로 'xyz' 순서
            r_euler = self.target_offsets_rot[side].as_euler('xyz', degrees=True)
            s = side[0].upper() # 'R' or 'L'
            print(f"--- {side.upper()} SIDE ---")
            print(f"offset_{s}_x: {p[0]:.4f} m")
            print(f"offset_{s}_y: {p[1]:.4f} m")
            print(f"offset_{s}_z: {p[2]:.4f} m")
            print(f"offset_{s}_roll: {r_euler[0]:.2f}°")
            print(f"offset_{s}_pitch: {r_euler[1]:.2f}°")
            print(f"offset_{s}_yaw: {r_euler[2]:.2f}°")
            print("-" * 20)
        print("="*50)
        print("이제 컨트롤러를 움직여도 위 오프셋이 적용된 target_link가 유지됩니다.\n")

    def send_t(self, time, p, c, pos, quat):
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = time, p, c
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, pos)
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = map(float, quat)
        self.tf_broadcaster.sendTransform(t)

    def cb_h(self, msg): self.current_poses['head'] = msg.pose; self.broadcast_main_tfs()
    def cb_r(self, msg): self.current_poses['right'] = msg.pose; self.broadcast_main_tfs()
    def cb_l(self, msg): self.current_poses['left'] = msg.pose; self.broadcast_main_tfs()

def main():
    rclpy.init(); node = SimDDRvizVisualizer()
    try: rclpy.spin(node)
    except: pass
    finally: rclpy.shutdown()

if __name__ == '__main__': main()