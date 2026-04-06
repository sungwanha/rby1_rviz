import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import pinocchio as pin
import numpy as np
import math
import threading
import tkinter as tk
from tkinter import ttk, messagebox

class TeachingPendantNode(Node):
    def __init__(self, urdf_path):
        super().__init__('teaching_pendant_node')
        
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.current_q = pin.neutral(self.model)
        
        self.frame_names = [f.name for f in self.model.frames]
        self.joint_names = []
        self.joint_limits = {}
        
        for i in range(1, self.model.njoints):
            name = self.model.names[i]
            idx_q = self.model.joints[i].idx_q
            nq = self.model.joints[i].nq
            if nq == 1:
                self.joint_names.append(name)
                lower = self.model.lowerPositionLimit[idx_q]
                upper = self.model.upperPositionLimit[idx_q]
                self.joint_limits[name] = (lower, upper, idx_q)

        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(1.0 / 30.0, self.publish_joint_states)

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

    def get_fk(self, frame_name):
        pin.forwardKinematics(self.model, self.data, self.current_q)
        pin.updateFramePlacements(self.model, self.data)
        if self.model.existFrame(frame_name):
            frame_id = self.model.getFrameId(frame_name)
            return self.data.oMf[frame_id]
        return None

    def solve_ik(self, frame_name, target_pose):
        frame_id = self.model.getFrameId(frame_name)
        q = self.current_q.copy()
        
        eps = 1e-4
        IT_MAX = 500
        DT = 1e-1
        damp = 1e-12
        
        # 고정 부위 설정 (팔만 움직이도록)
        locked_keywords = ['torso', 'head', 'wheel']
        locked_v_indices = []
        for i in range(1, self.model.njoints):
            if any(k in self.model.names[i] for k in locked_keywords):
                idx_v = self.model.joints[i].idx_v
                for v_i in range(idx_v, idx_v + self.model.joints[i].nv):
                    locked_v_indices.append(v_i)

        for i in range(IT_MAX):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            dMi = target_pose.actInv(self.data.oMf[frame_id])
            err = pin.log6(dMi).vector
            if np.linalg.norm(err) < eps: break
            J = pin.computeFrameJacobian(self.model, self.data, q, frame_id, pin.ReferenceFrame.LOCAL)
            for idx in locked_v_indices: J[:, idx] = 0.0
            v = - J.T.dot(np.linalg.solve(J.dot(J.T) + damp * np.eye(6), err))
            q = pin.integrate(self.model, q, v * DT)
            q = np.clip(q, self.model.lowerPositionLimit, self.model.upperPositionLimit)
            
        self.current_q = q
        return True

def run_gui(node):
    root = tk.Tk()
    root.title("Advanced Robot Teaching Pendant")
    root.geometry("1100x800")

    # [좌측 슬라이더 생략 - 이전과 동일하게 유지]
    left_frame = tk.Frame(root, width=350); left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)
    canvas = tk.Canvas(left_frame); scrollable_frame = tk.Frame(canvas); canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    sliders = {}; is_updating_sliders = False
    def on_slider_move(val, j_name):
        if not is_updating_sliders: node.current_q[node.joint_limits[j_name][2]] = float(val)
    for j_name in node.joint_names:
        l, u, _ = node.joint_limits[j_name]
        f = tk.Frame(scrollable_frame); f.pack(fill=tk.X)
        s = tk.Scale(f, from_=l, to=u, resolution=0.01, orient=tk.HORIZONTAL, length=180, command=lambda v, n=j_name: on_slider_move(v, n))
        s.set(0.0); s.pack(side=tk.RIGHT); tk.Label(f, text=j_name).pack(side=tk.LEFT); sliders[j_name] = s
    canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")

    # [우측 컨트롤러 패널]
    right_frame = tk.Frame(root); right_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=20, pady=10)
    frame_combo = ttk.Combobox(right_frame, values=node.frame_names, state="readonly"); frame_combo.set('ee_left'); frame_combo.pack(pady=5)
    lbl_fk = tk.Label(right_frame, text="Pose Info", font=("Consolas", 10), justify=tk.LEFT, bg="#f0f0f0", height=8, width=50); lbl_fk.pack(pady=5)

    # 간격 설정
    set_frame = tk.Frame(right_frame); set_frame.pack()
    tk.Label(set_frame, text="이동(m):").grid(row=0, column=0); ent_step = tk.Entry(set_frame, width=6); ent_step.insert(0, "0.02"); ent_step.grid(row=0, column=1)
    tk.Label(set_frame, text=" 회전(deg):").grid(row=0, column=2); ent_deg = tk.Entry(set_frame, width=6); ent_deg.insert(0, "5.0"); ent_deg.grid(row=0, column=3)

    def jog(mode, axis, direction):
        frame_name = frame_combo.get()
        curr = node.get_fk(frame_name)
        if not curr: return
        
        t_pos = curr.translation.copy()
        t_rot = curr.rotation.copy()

        if mode == 'POS':
            step = float(ent_step.get())
            if axis == 'X': t_pos[0] += step * direction
            elif axis == 'Y': t_pos[1] += step * direction
            elif axis == 'Z': t_pos[2] += step * direction
        else:
            deg = math.radians(float(ent_deg.get()) * direction)
            # 회전축 벡터 생성
            axis_vec = np.array([1, 0, 0] if axis == 'X' else [0, 1, 0] if axis == 'Y' else [0, 0, 1])
            # 증분 회전 행렬 생성 (Rodrigues 공식)
            R_step = pin.exp3(axis_vec * deg)
            # 현재 회전에 증분 회전 적용 (Global 기준 회전)
            t_rot = R_step @ t_rot

        node.solve_ik(frame_name, pin.SE3(t_rot, t_pos))
        nonlocal is_updating_sliders
        is_updating_sliders = True
        for j, s in sliders.items(): s.set(node.current_q[node.joint_limits[j][2]])
        is_updating_sliders = False

    # 버튼 배치
    btn_p = tk.LabelFrame(right_frame, text=" Position Jog (위치) ", padx=10, pady=10); btn_p.pack(pady=10)
    colors = ['#ffcccc', '#ccffcc', '#ccccff']
    for i, ax in enumerate(['X', 'Y', 'Z']):
        tk.Button(btn_p, text=f"+{ax}", bg=colors[i], width=8, command=lambda a=ax: jog('POS', a, 1)).grid(row=0, column=i, padx=5)
        tk.Button(btn_p, text=f"-{ax}", bg=colors[i], width=8, command=lambda a=ax: jog('POS', a, -1)).grid(row=1, column=i, padx=5)

    btn_r = tk.LabelFrame(right_frame, text=" Rotation Jog (회전) ", padx=10, pady=10); btn_r.pack(pady=10)
    for i, ax in enumerate(['X', 'Y', 'Z']):
        name = ['Roll', 'Pitch', 'Yaw'][i]
        tk.Button(btn_r, text=f"+{name}", bg=colors[i], width=8, command=lambda a=ax: jog('ROT', a, 1)).grid(row=0, column=i, padx=5)
        tk.Button(btn_r, text=f"-{name}", bg=colors[i], width=8, command=lambda a=ax: jog('ROT', a, -1)).grid(row=1, column=i, padx=5)

    def update_loop():
        pose = node.get_fk(frame_combo.get())
        if pose:
            t, r = pose.translation, pose.rotation
            rpy = pin.rpy.matrixToRpy(r) * 180 / math.pi
            lbl_fk.config(text=f"Pos: {t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}\nRPY: {rpy[0]:.1f}, {rpy[1]:.1f}, {rpy[2]:.1f}")
        root.after(100, update_loop)

    update_loop(); root.mainloop()

if __name__ == '__main__':
    rclpy.init(); node = TeachingPendantNode('/home/hsw/rby1_rviz/model/model_v1.2.urdf')
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    run_gui(node)