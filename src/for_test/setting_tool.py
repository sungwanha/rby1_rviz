import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import pinocchio as pin
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

import tkinter as tk
from tkinter import ttk, messagebox
import threading

class KinematicsAnalysisNode(Node):
    def __init__(self):
        super().__init__('kinematics_analysis_gui_node')
        self.model_loaded = False
        self.fixed_target_link = "zed2i_right_camera_optical_frame"
        
        try:
            pkg_share_path = get_package_share_directory('rby1_teleop_cpp')
            self.urdf_path = os.path.join(pkg_share_path, 'model', 'model_v1.2.urdf')
        except Exception as e:
            self.get_logger().error(f"Failed to find package: {e}")
            return

        try:
            self.model = pin.buildModelFromUrdf(self.urdf_path)
            self.data = self.model.createData()
            self.q = pin.neutral(self.model)
            self.model_loaded = True
            self.get_logger().info("URDF model loaded successfully!")
        except Exception as e:
            self.get_logger().error(f"Failed to load URDF: {e}")
            return

        self.joint_name_to_id = {self.model.names[i]: i for i in range(1, self.model.njoints)}
        self.sub_js = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

    def joint_state_callback(self, msg):
        if not self.model_loaded: return
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_name_to_id:
                jid = self.joint_name_to_id[name]
                idx_q = self.model.joints[jid].idx_q
                self.q[idx_q] = pos

    def get_all_joint_names(self):
        if not self.model_loaded: return []
        return [self.model.names[i] for i in range(1, self.model.njoints) if self.model.joints[i].nq == 1]

    def compute_fk(self):
        if not self.model_loaded: return False, "Model is not loaded."
        frame_id = self.model.getFrameId(self.fixed_target_link)
        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)
        pose = self.data.oMf[frame_id]
        x, y, z = pose.translation
        z_axis = pose.rotation[:, 2] 
        angle_rad = np.arcsin(np.clip(z_axis[2], -1.0, 1.0))
        return True, (x, y, z, angle_rad)

    def compute_ik(self, movable_joints, target_xyz, target_angle_rad):
        if not self.model_loaded: return False, "Model is not loaded."
        frame_id = self.model.getFrameId(self.fixed_target_link)
        q_opt = self.q.copy()

        pin.forwardKinematics(self.model, self.data, q_opt)
        pin.updateFramePlacements(self.model, self.data)
        target_pose = self.data.oMf[frame_id].copy()
        target_pose.translation = np.array(target_xyz)

        z_axis = target_pose.rotation[:, 2] 
        xy_norm = np.linalg.norm(z_axis[:2])
        if xy_norm > 1e-6:
            v_target_z = np.array([
                z_axis[0] / xy_norm * np.cos(target_angle_rad),
                z_axis[1] / xy_norm * np.cos(target_angle_rad),
                np.sin(target_angle_rad)
            ])
        else:
            v_target_z = np.array([np.cos(target_angle_rad), 0.0, np.sin(target_angle_rad)])

        q_rot = pin.Quaternion.FromTwoVectors(z_axis, v_target_z)
        target_pose.rotation = q_rot.matrix() @ target_pose.rotation

        eps = 1e-4
        success = False
        for i in range(100):
            pin.forwardKinematics(self.model, self.data, q_opt)
            pin.updateFramePlacements(self.model, self.data)
            dM = self.data.oMf[frame_id].actInv(target_pose)
            err = pin.log6(dM).vector
            if np.linalg.norm(err) < eps:
                success = True
                break
            J = pin.computeFrameJacobian(self.model, self.data, q_opt, frame_id, pin.ReferenceFrame.LOCAL)
            for j in range(1, self.model.njoints):
                if self.model.names[j] not in movable_joints:
                    idx_v = self.model.joints[j].idx_v
                    J[:, idx_v : idx_v + self.model.joints[j].nv] = 0.0
            J_inv = J.T @ np.linalg.inv(J @ J.T + np.eye(6) * 0.01)
            q_opt = pin.integrate(self.model, q_opt, J_inv @ err * 0.5)
            q_opt = np.clip(q_opt, self.model.lowerPositionLimit, self.model.upperPositionLimit)

        if not success:
            return False, "수렴 불가능합니다. (범위 초과)"
        result_dict = {j: q_opt[self.model.joints[self.joint_name_to_id[j]].idx_q] for j in movable_joints}
        return True, result_dict

class KinematicsApp:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.root.title("Camera Precise Controller (Joint State Editing)")
        self.root.geometry("650x950")

        self.unit_mode = tk.StringVar(value="rad")
        self.last_unit = "rad"
        self.step_cm = tk.DoubleVar(value=1.0)
        
        self.target_vars = {
            'x': tk.StringVar(value="0.500"),
            'y': tk.StringVar(value="0.000"),
            'z': tk.StringVar(value="0.500"),
            'ang': tk.StringVar(value="0.000")
        }

        style = ttk.Style()
        style.theme_use('clam')

        # === 0. Target Status Display ===
        status_frame = ttk.LabelFrame(root, text="Current Target Status (Goal)", padding=10)
        status_frame.pack(fill="x", padx=10, pady=5)
        self.lbl_target_summary = ttk.Label(status_frame, text="Target: X 0.500 | Y 0.000 | Z 0.500 | A 0.000 rad", 
                                            font=("Courier", 11, "bold"), foreground="darkgreen")
        self.lbl_target_summary.pack()

        # === 1. Sync & Unit Setting ===
        sync_frame = ttk.Frame(root, padding=10)
        sync_frame.pack(fill="x", padx=10)
        ttk.Button(sync_frame, text="Get Current Pose (Sync Target)", command=self.sync_to_current).pack(side="left", expand=True, fill="x")
        
        unit_frame = ttk.LabelFrame(root, text="Step & Unit Setting", padding=10)
        unit_frame.pack(fill="x", padx=10, pady=5)
        ttk.Label(unit_frame, text="Step:").grid(row=0, column=0)
        ttk.Entry(unit_frame, textvariable=self.step_cm, width=5).grid(row=0, column=1, padx=5)
        ttk.Label(unit_frame, text="cm/deg").grid(row=0, column=2)
        ttk.Radiobutton(unit_frame, text="Rad", variable=self.unit_mode, value="rad", command=self.toggle_unit).grid(row=0, column=3, padx=10)
        ttk.Radiobutton(unit_frame, text="Deg", variable=self.unit_mode, value="deg", command=self.toggle_unit).grid(row=0, column=4)

        # === 2. Target Adjustment ===
        jog_frame = ttk.LabelFrame(root, text="Target Pose Adjustment", padding=10)
        jog_frame.pack(fill="x", padx=10, pady=5)

        configs = [("X Pos (m)", 'x', "m"), ("Y Pos (m)", 'y', "m"), 
                   ("Z Pos (m)", 'z', "m"), ("Angle", 'ang', "ang")]

        for i, (label, key, t) in enumerate(configs):
            ttk.Label(jog_frame, text=label, font=('Arial', 9, 'bold')).grid(row=i, column=0, sticky="w")
            ttk.Button(jog_frame, text="-", width=3, command=lambda k=key, ty=t: self.adjust(k, ty, -1)).grid(row=i, column=1, padx=2)
            ent = ttk.Entry(jog_frame, textvariable=self.target_vars[key], width=12, justify="center")
            ent.grid(row=i, column=2, padx=2)
            ent.bind("<Return>", lambda e: self.run_ik())
            ttk.Button(jog_frame, text="+", width=3, command=lambda k=key, ty=t: self.adjust(k, ty, 1)).grid(row=i, column=3, padx=2)

        ttk.Button(jog_frame, text="Apply Typed Values (Enter)", command=self.run_ik).grid(row=4, column=0, columnspan=4, pady=10)

        # === 3. Joint Selection & Real-time State ===
        sel_frame = ttk.LabelFrame(root, text="Movable Joints & Current States (Edit to override)", padding=10)
        sel_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        # Header for the scrollable area
        header_f = ttk.Frame(sel_frame)
        header_f.pack(fill="x")
        ttk.Label(header_f, text="Use?", width=5).pack(side="left")
        ttk.Label(header_f, text="Joint Name", width=25).pack(side="left")
        ttk.Label(header_f, text="Current Angle", width=15).pack(side="left")

        canvas = tk.Canvas(sel_frame, height=250)
        scrollbar = ttk.Scrollbar(sel_frame, orient="vertical", command=canvas.yview)
        self.scroll_frame = tk.Frame(canvas)
        self.scroll_frame.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=self.scroll_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        self.joint_vars = {}       # For checkboxes (BooleanVar)
        self.joint_state_vars = {} # For entry boxes (StringVar)
        self.joint_entries = {}    # To check focus
        
        default_joints = ["torso1", "torso2", "torso3", "head1"]
        
        for j in self.node.get_all_joint_names():
            row = tk.Frame(self.scroll_frame)
            row.pack(fill="x", pady=1)

            # 1. Checkbox
            is_default = any(dj in j.lower() for dj in default_joints)
            v = tk.BooleanVar(value=is_default)
            self.joint_vars[j] = v
            tk.Checkbutton(row, variable=v, width=2).pack(side="left")

            # 2. Name
            tk.Label(row, text=j, width=25, anchor="w").pack(side="left")

            # 3. State Entry (Editable)
            sv = tk.StringVar(value="0.0000")
            self.joint_state_vars[j] = sv
            ent = tk.Entry(row, textvariable=sv, width=15, justify="right")
            ent.pack(side="left", padx=5)
            self.joint_entries[j] = ent
        
        ttk.Button(sel_frame, text="Apply Joint Values to Model (Manual Override)", 
                   command=self.apply_manual_joints).pack(fill="x", pady=5)

        # === 4. Result Output ===
        self.txt_res = tk.Text(root, height=12, bg="#f4f4f4", state="disabled", font=("Courier", 10))
        self.txt_res.pack(fill="x", padx=10, pady=10)

        # Start UI Update Loop
        self.update_ui_loop()

    def update_ui_loop(self):
        """ROS의 관절 상태를 UI 필드에 주기적으로 업데이트 (사용자가 입력 중이 아닐 때만)"""
        unit = self.unit_mode.get()
        for name, sv in self.joint_state_vars.items():
            # 사용자가 해당 Entry를 클릭하고 있지 않을 때만 ROS 값 반영
            if self.root.focus_get() != self.joint_entries[name]:
                jid = self.node.joint_name_to_id[name]
                idx_q = self.node.model.joints[jid].idx_q
                val_rad = self.node.q[idx_q]
                val = np.rad2deg(val_rad) if unit == "deg" else val_rad
                sv.set(f"{val:.4f}")
        
        self.root.after(100, self.update_ui_loop)

    def apply_manual_joints(self):
        """사용자가 Entry에 입력한 값을 Node의 현재 q에 강제 적용"""
        unit = self.unit_mode.get()
        try:
            for name, sv in self.joint_state_vars.items():
                val = float(sv.get())
                val_rad = np.deg2rad(val) if unit == "deg" else val
                jid = self.node.joint_name_to_id[name]
                idx_q = self.node.model.joints[jid].idx_q
                self.node.q[idx_q] = val_rad
            messagebox.showinfo("Success", "Joint values updated in model.")
        except ValueError:
            messagebox.showerror("Error", "Invalid joint value entered.")

    def toggle_unit(self):
        new_unit = self.unit_mode.get()
        if new_unit == self.last_unit: return
        try:
            # Target Angle 변환
            current_ang = float(self.target_vars['ang'].get())
            converted = np.rad2deg(current_ang) if new_unit == "deg" else np.deg2rad(current_ang)
            self.target_vars['ang'].set(f"{converted:.4f}")
        except ValueError: pass
        self.last_unit = new_unit
        self.run_ik()

    def sync_to_current(self):
        success, res = self.node.compute_fk()
        if success:
            x, y, z, ang_rad = res
            self.target_vars['x'].set(f"{x:.4f}")
            self.target_vars['y'].set(f"{y:.4f}")
            self.target_vars['z'].set(f"{z:.4f}")
            self.target_vars['ang'].set(f"{np.rad2deg(ang_rad) if self.unit_mode.get() == 'deg' else ang_rad:.4f}")
            self.run_ik()
        else: messagebox.showerror("Error", res)

    def adjust(self, key, type, direction):
        try:
            current_val = float(self.target_vars[key].get())
            step = self.step_cm.get()
            change = (step / 100.0) * direction if type == "m" else step * direction
            self.target_vars[key].set(f"{current_val + change:.4f}")
            self.run_ik()
        except ValueError: pass

    def run_ik(self):
        try:
            tx, ty, tz = float(self.target_vars['x'].get()), float(self.target_vars['y'].get()), float(self.target_vars['z'].get())
            ta_val = float(self.target_vars['ang'].get())
            unit = self.unit_mode.get()
            ta_rad = np.deg2rad(ta_val) if unit == "deg" else ta_val
            self.lbl_target_summary.config(text=f"Target: X {tx:.3f} | Y {ty:.3f} | Z {tz:.3f} | A {ta_val:.3f} {unit}")
        except ValueError: return

        movable = [j for j, v in self.joint_vars.items() if v.get()]
        success, res = self.node.compute_ik(movable, [tx, ty, tz], ta_rad)
        
        self.txt_res.config(state="normal")
        self.txt_res.delete("1.0", tk.END)
        if success:
            out = f"✅ IK Success (Output Unit: {unit})\n" + "-"*40 + "\n"
            for k, v_rad in res.items():
                val = np.rad2deg(v_rad) if unit == "deg" else v_rad
                out += f"{k:30}: {val:10.4f}\n"
            self.txt_res.insert(tk.END, out)
        else: self.txt_res.insert(tk.END, f"❌ {res}")
        self.txt_res.config(state="disabled")

def main(args=None):
    rclpy.init(args=args)
    node = KinematicsAnalysisNode()
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    root = tk.Tk()
    app = KinematicsApp(root, node)
    root.protocol("WM_DELETE_WINDOW", lambda: [node.destroy_node(), rclpy.shutdown(), root.destroy()])
    root.mainloop()

if __name__ == '__main__':
    main()