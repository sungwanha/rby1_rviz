import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import pinocchio as pin
import numpy as np
import math
import threading
import tkinter as tk
from tkinter import ttk

class TeleopIKNode(Node):
    def __init__(self, urdf_path):
        super().__init__('teleop_ik_node')
        
        # Pinocchio 모델 로드
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.current_q = pin.neutral(self.model)
        
        self.frame_names = [f.name for f in self.model.frames]
        self.joint_names = []
        self.joint_limits = {}
        
        # 조인트 정보 추출
        for i in range(1, self.model.njoints):
            name = self.model.names[i]
            idx_q = self.model.joints[i].idx_q
            nq = self.model.joints[i].nq
            if nq == 1:
                self.joint_names.append(name)
                lower = self.model.lowerPositionLimit[idx_q]
                upper = self.model.upperPositionLimit[idx_q]
                self.joint_limits[name] = (lower, upper, idx_q)

        # ROS 2 설정
        self.tf_broadcaster = TransformBroadcaster(self)
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/simdd/pose',
            self.teleop_callback,
            10)
            
        self.timer = self.create_timer(1.0 / 60.0, self.publish_joint_states)
        
        # 제어 상태
        self.teleop_enabled = False
        self.IT_MAX = 10
        self.DT = 0.5
        self.damp = 1e-6
        
        # 고정 조인트 설정 (팔만 움직임)
        self.locked_keywords = ['torso', 'head', 'wheel']
        self.locked_v_indices = []
        for i in range(1, self.model.njoints):
            if any(k in self.model.names[i] for k in self.locked_keywords):
                idx_v = self.model.joints[i].idx_v
                for v_i in range(idx_v, idx_v + self.model.joints[i].nv):
                    self.locked_v_indices.append(v_i)

    def set_init_pose(self):
        """요청된 초기 포즈로 설정"""
        self.current_q = pin.neutral(self.model)
        init_values = {
            'left_arm_3': -1.5708,
            'left_arm_6': -1.5708,
            'right_arm_3': -1.5708,
            'right_arm_6': 1.5708
        }
        for name, val in init_values.items():
            if name in self.joint_limits:
                idx = self.joint_limits[name][2]
                self.current_q[idx] = val

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(1, self.model.njoints):
            name = self.model.names[i]
            idx_q = self.model.joints[i].idx_q
            nq = self.model.joints[i].nq
            if nq == 1:
                msg.name.append(name)
                msg.position.append(float(self.current_q[idx_q]))
        self.publisher_.publish(msg)

    def broadcast_target_tf(self, parent, child, pose_se3):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        trans = pose_se3.translation
        quat = pin.Quaternion(pose_se3.rotation)
        t.transform.translation.x = float(trans[0]); t.transform.translation.y = float(trans[1]); t.transform.translation.z = float(trans[2])
        t.transform.rotation.x = float(quat.x); t.transform.rotation.y = float(quat.y); t.transform.rotation.z = float(quat.z); t.transform.rotation.w = float(quat.w)
        self.tf_broadcaster.sendTransform(t)

    def teleop_callback(self, msg):
        if len(msg.data) < 32: return
        data = msg.data
        
        # 트래킹 상태 확인 (6: 오른손, 13: 왼손)
        # 1.0이면 트래킹 중, 0.0이면 끊김
        right_tracking = data[6] > 0.5
        left_tracking = data[13] > 0.5
        
        # 양손 모두 트래킹이 안 되면 즉시 리턴
        if not right_tracking and not left_tracking:
            return

        # OpenXR -> ROS 변환 및 상대 포즈 계산
        # 오른손 데이터 처리
        r_pos = np.array([-data[9], -data[7], data[8]])
        rx, ry, rz, rw = -data[26], -data[24], data[25], data[27]
        s_r, c_r = 0.70710678118, 0.70710678118
        rq = np.array([rx*c_r + ry*s_r, ry*c_r - rx*s_r, rz*c_r + rw*s_r, rw*c_r - rz*s_r])
        
        # 왼손 데이터 처리
        l_pos = np.array([-data[16], -data[14], data[15]])
        lx, ly, lz, lw = -data[30], -data[28], data[29], data[31]
        s_l, c_l = -0.70710678118, 0.70710678118
        lq = np.array([lx*c_l + ly*s_l, ly*c_l - lx*s_l, lz*c_l + lw*s_l, lw*c_l - lz*s_l])

        M_hmd_r = pin.SE3(pin.Quaternion(np.array(rq)).toRotationMatrix(), r_pos)
        M_hmd_l = pin.SE3(pin.Quaternion(np.array(lq)).toRotationMatrix(), l_pos)
        
        if self.teleop_enabled:
            pin.forwardKinematics(self.model, self.data, self.current_q)
            pin.updateFramePlacements(self.model, self.data)
            
            head_frame = 'link_head_2'
            if not self.model.existFrame(head_frame): head_frame = 'head_1'
            head_id = self.model.getFrameId(head_frame)
            oM_head = self.data.oMf[head_id]
            
            target_r = oM_head * M_hmd_r
            target_l = oM_head * M_hmd_l
            
            # 1. 트래킹 되는 손의 TF만 발행
            if right_tracking:
                self.broadcast_target_tf('base', 'target_right', target_r)
            if left_tracking:
                self.broadcast_target_tf('base', 'target_left', target_l)
            
            # 2. IK 실행 조건 분기
            if right_tracking and left_tracking:
                # 둘 다 트래킹 되면 기존 Dual IK
                self.solve_ik_dual('ee_right', target_r, 'ee_left', target_l)
            elif right_tracking:
                # 오른손만 되면 오른손만 IK
                self.solve_ik_single('ee_right', target_r)
            elif left_tracking:
                # 왼손만 되면 왼손만 IK
                self.solve_ik_single('ee_left', target_l)

    def solve_ik_dual(self, frame_name_r, target_pose_r, frame_name_l, target_pose_l):
        id_r = self.model.getFrameId(frame_name_r); id_l = self.model.getFrameId(frame_name_l)
        q = self.current_q.copy()
        for i in range(self.IT_MAX):
            pin.forwardKinematics(self.model, self.data, q); pin.updateFramePlacements(self.model, self.data)
            err_r = pin.log6(target_pose_r.actInv(self.data.oMf[id_r])).vector
            err_l = pin.log6(target_pose_l.actInv(self.data.oMf[id_l])).vector
            combined_err = np.concatenate([err_r, err_l])
            if np.linalg.norm(combined_err) < 1e-4: break
            J_r = pin.computeFrameJacobian(self.model, self.data, q, id_r, pin.ReferenceFrame.LOCAL)
            J_l = pin.computeFrameJacobian(self.model, self.data, q, id_l, pin.ReferenceFrame.LOCAL)
            for idx in self.locked_v_indices: J_r[:, idx] = 0.0; J_l[:, idx] = 0.0
            J = np.vstack([J_r, J_l])
            v = - J.T.dot(np.linalg.solve(J.dot(J.T) + self.damp * np.eye(12), combined_err))
            q = pin.integrate(self.model, q, v * self.DT)
            q = np.clip(q, self.model.lowerPositionLimit, self.model.upperPositionLimit)
        self.current_q = q

    def get_fk(self, frame_name):
        pin.forwardKinematics(self.model, self.data, self.current_q); pin.updateFramePlacements(self.model, self.data)
        if self.model.existFrame(frame_name): return self.data.oMf[self.model.getFrameId(frame_name)]
        return None

    def solve_ik_single(self, frame_name, target_pose):
        frame_id = self.model.getFrameId(frame_name); q = self.current_q.copy()
        for i in range(50):
            pin.forwardKinematics(self.model, self.data, q); pin.updateFramePlacements(self.model, self.data)
            err = pin.log6(target_pose.actInv(self.data.oMf[frame_id])).vector
            if np.linalg.norm(err) < 1e-4: break
            J = pin.computeFrameJacobian(self.model, self.data, q, frame_id, pin.ReferenceFrame.LOCAL)
            for idx in self.locked_v_indices: J[:, idx] = 0.0
            v = - J.T.dot(np.linalg.solve(J.dot(J.T) + self.damp * np.eye(6), err))
            q = pin.integrate(self.model, q, v * self.DT)
            q = np.clip(q, self.model.lowerPositionLimit, self.model.upperPositionLimit)
        self.current_q = q
        return True

def run_gui(node):
    root = tk.Tk(); root.title("RBY1 Teleop & Teaching Pendant"); root.geometry("1100x850")
    
    # [좌측 영역] 슬라이더 패널
    left_main = tk.Frame(root, width=400); left_main.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)
    
    # 상단 버튼 프레임 (Teleop & Init 버튼)
    btn_frame = tk.Frame(left_main); btn_frame.pack(pady=10)
    
    teleop_var = tk.BooleanVar(value=False)
    def toggle_teleop():
        node.teleop_enabled = teleop_var.get()
        btn_teleop.config(bg='green' if node.teleop_enabled else 'red', text="Teleop ON" if node.teleop_enabled else "Teleop OFF")
    
    btn_teleop = tk.Checkbutton(btn_frame, text="Teleop OFF", variable=teleop_var, command=toggle_teleop, 
                                 indicatoron=False, width=15, height=2, bg='red', font=('Arial', 10, 'bold'))
    btn_teleop.pack(side=tk.LEFT, padx=5)

    def update_sliders():
        nonlocal is_updating_sliders
        is_updating_sliders = True
        for j, s in sliders.items(): s.set(node.current_q[node.joint_limits[j][2]])
        is_updating_sliders = False

    def click_init():
        node.set_init_pose()
        update_sliders()

    btn_init = tk.Button(btn_frame, text="Init Pose", command=click_init, 
                         width=15, height=2, bg='#dddddd', font=('Arial', 10, 'bold'))
    btn_init.pack(side=tk.LEFT, padx=5)

    # 스크롤 가능한 슬라이더 영역
    container = tk.Frame(left_main); container.pack(fill=tk.BOTH, expand=True)
    canvas = tk.Canvas(container); scrollbar = ttk.Scrollbar(container, orient="vertical", command=canvas.yview)
    scrollable_frame = tk.Frame(canvas)

    def update_scroll_region(event):
        canvas.configure(scrollregion=canvas.bbox("all"))
    scrollable_frame.bind("<Configure>", update_scroll_region)
    
    canvas_window = canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
    def on_canvas_configure(event):
        canvas.itemconfig(canvas_window, width=event.width)
    canvas.bind("<Configure>", on_canvas_configure)
    canvas.configure(yscrollcommand=scrollbar.set)
    
    def on_mousewheel(event):
        if event.num == 4 or event.delta > 0: canvas.yview_scroll(-1, "units")
        elif event.num == 5 or event.delta < 0: canvas.yview_scroll(1, "units")
    canvas.bind_all("<MouseWheel>", on_mousewheel)
    canvas.bind_all("<Button-4>", on_mousewheel)
    canvas.bind_all("<Button-5>", on_mousewheel)

    canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

    sliders = {}; is_updating_sliders = False
    def on_slider_move(val, j_name):
        if not is_updating_sliders: node.current_q[node.joint_limits[j_name][2]] = float(val)

    for j_name in node.joint_names:
        l, u, _ = node.joint_limits[j_name]
        f = tk.Frame(scrollable_frame); f.pack(fill=tk.X, pady=2, padx=5)
        tk.Label(f, text=j_name, width=15, anchor='w', font=("Arial", 9)).pack(side=tk.LEFT)
        s = tk.Scale(f, from_=l, to=u, resolution=0.001, orient=tk.HORIZONTAL, length=180, command=lambda v, n=j_name: on_slider_move(v, n))
        s.set(float(node.current_q[node.joint_limits[j_name][2]])); s.pack(side=tk.RIGHT, fill=tk.X, expand=True)
        sliders[j_name] = s

    # [우측 영역] 제어 패널
    right_frame = tk.Frame(root); right_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=20, pady=10)
    frame_combo = ttk.Combobox(right_frame, values=node.frame_names, state="readonly"); frame_combo.set('ee_left'); frame_combo.pack(pady=5)
    lbl_fk = tk.Label(right_frame, text="Pose Info", font=("Consolas", 10), justify=tk.LEFT, bg="#f0f0f0", height=8, width=50); lbl_fk.pack(pady=5)

    set_frame = tk.Frame(right_frame); set_frame.pack()
    tk.Label(set_frame, text="이동(m):").grid(row=0, column=0); ent_step = tk.Entry(set_frame, width=6); ent_step.insert(0, "0.02"); ent_step.grid(row=0, column=1)
    tk.Label(set_frame, text=" 회전(deg):").grid(row=0, column=2); ent_deg = tk.Entry(set_frame, width=6); ent_deg.insert(0, "5.0"); ent_deg.grid(row=0, column=3)

    def jog(mode, axis, direction):
        frame_name = frame_combo.get(); curr = node.get_fk(frame_name)
        if not curr: return
        t_pos = curr.translation.copy(); t_rot = curr.rotation.copy()
        if mode == 'POS':
            step = float(ent_step.get())
            if axis == 'X': t_pos[0] += step * direction
            elif axis == 'Y': t_pos[1] += step * direction
            elif axis == 'Z': t_pos[2] += step * direction
        else:
            deg = math.radians(float(ent_deg.get()) * direction); axis_vec = np.array([1, 0, 0] if axis == 'X' else [0, 1, 0] if axis == 'Y' else [0, 0, 1])
            t_rot = pin.exp3(axis_vec * deg) @ t_rot
        node.solve_ik_single(frame_name, pin.SE3(t_rot, t_pos))

    def update_loop():
        if node.teleop_enabled: update_sliders()
        pose = node.get_fk(frame_combo.get())
        if pose:
            t, r = pose.translation, pose.rotation; rpy = pin.rpy.matrixToRpy(r) * 180 / math.pi
            lbl_fk.config(text=f"Frame: {frame_combo.get()}\nPos: {t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}\nRPY: {rpy[0]:.1f}, {rpy[1]:.1f}, {rpy[2]:.1f}")
        root.after(100, update_loop)

    update_loop(); root.mainloop()

if __name__ == '__main__':
    rclpy.init(); node = TeleopIKNode('/home/hsw/rby1_rviz/model/model_v1.2.urdf')
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    run_gui(node); rclpy.shutdown()
