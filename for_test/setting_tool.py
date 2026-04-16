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
        
        # 고정할 타겟 링크
        self.fixed_target_link = "zed2i_right_camera_optical_frame"
        
        # 1. Find package path dynamically
        try:
            pkg_share_path = get_package_share_directory('rby1_teleop_cpp')
            self.urdf_path = os.path.join(pkg_share_path, 'model', 'model_v1.2.urdf')
        except Exception as e:
            self.get_logger().error(f"Failed to find package: {e}")
            return

        # 2. Build Pinocchio model
        try:
            self.model = pin.buildModelFromUrdf(self.urdf_path)
            self.data = self.model.createData()
            self.q = pin.neutral(self.model)
            self.model_loaded = True
            self.get_logger().info("URDF model loaded successfully!")
        except Exception as e:
            self.get_logger().error(f"Failed to load URDF: {e}")
            return

        # 3. Joint name to ID mapping
        self.joint_name_to_id = {}
        for i in range(1, self.model.njoints):
            self.joint_name_to_id[self.model.names[i]] = i

        # 4. Subscribe to joint_states
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
        if not self.model.existFrame(self.fixed_target_link): 
            return False, f"Link does not exist: {self.fixed_target_link}"

        frame_id = self.model.getFrameId(self.fixed_target_link)
        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)
        
        pose = self.data.oMf[frame_id]
        height = pose.translation[2]
        
        z_axis = pose.rotation[:, 2] 
        deg_with_xy = np.rad2deg(np.arcsin(z_axis[2]))
        
        return True, (height, deg_with_xy)

    def compute_ik(self, movable_joints, target_h, target_deg):
        if not self.model_loaded: return False, "Model is not loaded."
        if not self.model.existFrame(self.fixed_target_link): 
            return False, f"Link does not exist: {self.fixed_target_link}"
        if not movable_joints: return False, "No joints selected for movement."

        frame_id = self.model.getFrameId(self.fixed_target_link)
        q_opt = self.q.copy()

        pin.forwardKinematics(self.model, self.data, q_opt)
        pin.updateFramePlacements(self.model, self.data)
        target_pose = self.data.oMf[frame_id].copy()
        
        target_pose.translation[2] = target_h
        
        z_axis = target_pose.rotation[:, 2] 
        xy_norm = np.linalg.norm(z_axis[:2])
        alpha_rad = np.deg2rad(target_deg)

        if xy_norm > 1e-6:
            v_target_z = np.array([
                z_axis[0] / xy_norm * np.cos(alpha_rad),
                z_axis[1] / xy_norm * np.cos(alpha_rad),
                np.sin(alpha_rad)
            ])
        else:
            v_target_z = np.array([np.cos(alpha_rad), 0.0, np.sin(alpha_rad)])

        q_rot = pin.Quaternion.FromTwoVectors(z_axis, v_target_z)
        target_pose.rotation = q_rot.matrix() @ target_pose.rotation

        for i in range(50):
            pin.forwardKinematics(self.model, self.data, q_opt)
            pin.updateFramePlacements(self.model, self.data)
            
            dM = self.data.oMf[frame_id].actInv(target_pose)
            err = pin.log6(dM).vector
            if np.linalg.norm(err) < 1e-4: break

            J = pin.computeFrameJacobian(self.model, self.data, q_opt, frame_id, pin.ReferenceFrame.LOCAL)

            for j in range(1, self.model.njoints):
                if self.model.names[j] not in movable_joints:
                    idx_v = self.model.joints[j].idx_v
                    nv = self.model.joints[j].nv
                    for v in range(nv):
                        J[:, idx_v + v] = 0.0

            J_inv = J.T @ np.linalg.inv(J @ J.T + np.eye(6) * 0.01)
            q_opt = pin.integrate(self.model, q_opt, J_inv @ err * 0.5)
            q_opt = np.clip(q_opt, self.model.lowerPositionLimit, self.model.upperPositionLimit)

        result_dict = {}
        for j_name in movable_joints:
            if j_name in self.joint_name_to_id:
                jid = self.joint_name_to_id[j_name]
                val = q_opt[self.model.joints[jid].idx_q]
                result_dict[j_name] = val
        return True, result_dict


class KinematicsApp:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.root.title("Camera Kinematics Tool")
        self.root.geometry("500x700")
        self.root.resizable(False, False)

        style = ttk.Style()
        style.theme_use('clam')

        self.joint_names = self.node.get_all_joint_names()
        self.fixed_link = self.node.fixed_target_link

        # === 1. Status Frame ===
        status_frame = ttk.LabelFrame(root, text="System Status", padding=(10, 10))
        status_frame.pack(fill="x", padx=10, pady=5)
        self.lbl_status = ttk.Label(status_frame, text="URDF Loaded & ROS Ready" if node.model_loaded else "URDF Load Failed!", 
                                    foreground="green" if node.model_loaded else "red", font=("Arial", 10, "bold"))
        self.lbl_status.pack()

        # === 2. FK Mode Frame ===
        fk_frame = ttk.LabelFrame(root, text="[FK Mode] Current Pose", padding=(10, 10))
        fk_frame.pack(fill="x", padx=10, pady=5)

        ttk.Label(fk_frame, text="Fixed Target Link:").grid(row=0, column=0, sticky="w", pady=2)
        ttk.Label(fk_frame, text=self.fixed_link, foreground="blue", font=("Arial", 9, "bold")).grid(row=0, column=1, sticky="w", pady=2, padx=5)

        ttk.Button(fk_frame, text="Compute FK", command=self.run_fk).grid(row=1, column=0, columnspan=2, pady=10)

        self.lbl_fk_result = ttk.Label(fk_frame, text="Height (Z): - m\nAngle with XY plane: - °", justify="left")
        self.lbl_fk_result.grid(row=2, column=0, columnspan=2, sticky="w")

        # === 3. IK Mode Frame ===
        ik_frame = ttk.LabelFrame(root, text="[IK Mode] Adjust Camera Pose", padding=(10, 10))
        ik_frame.pack(fill="x", padx=10, pady=5)

        ttk.Label(ik_frame, text="Target Height (Z, m):").grid(row=0, column=0, sticky="w", pady=2)
        self.ent_ik_h = ttk.Entry(ik_frame, width=25)
        self.ent_ik_h.insert(0, "0.5")
        self.ent_ik_h.grid(row=0, column=1, pady=2, padx=5)

        ttk.Label(ik_frame, text="Angle with XY Plane (°):").grid(row=1, column=0, sticky="w", pady=2)
        self.ent_ik_deg = ttk.Entry(ik_frame, width=25)
        self.ent_ik_deg.insert(0, "0.0")
        self.ent_ik_deg.grid(row=1, column=1, pady=2, padx=5)

        ttk.Label(ik_frame, text="Select Movable Joints (Check to enable):").grid(row=2, column=0, columnspan=2, sticky="w", pady=(10,0))
        
        # --- [수정된 부분] 스크롤 가능한 체크박스 영역 ---
        list_frame = tk.Frame(ik_frame, bd=2, relief="sunken", bg="white")
        list_frame.grid(row=3, column=0, columnspan=2, pady=5, sticky="we")
        
        canvas = tk.Canvas(list_frame, bg="white", height=120)
        scrollbar = ttk.Scrollbar(list_frame, orient="vertical", command=canvas.yview)
        scrollable_frame = tk.Frame(canvas, bg="white")

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # 조인트별로 체크 상태를 저장할 변수 딕셔너리
        self.joint_vars = {}
        for j in self.joint_names:
            var = tk.BooleanVar()
            # 예시: head나 torso가 이름에 들어가면 기본 체크
            if "head" in j or "torso" in j:
                var.set(True)
            self.joint_vars[j] = var
            
            # 체크박스 생성
            chk = tk.Checkbutton(scrollable_frame, text=j, variable=var, bg="white", anchor="w", justify="left")
            chk.pack(fill="x", padx=5, pady=0)
        # -----------------------------------------------

        ttk.Button(ik_frame, text="Solve IK", command=self.run_ik).grid(row=4, column=0, columnspan=2, pady=10)

        self.txt_ik_result = tk.Text(ik_frame, width=45, height=7, state="disabled", bg="#f0f0f0")
        self.txt_ik_result.grid(row=5, column=0, columnspan=2)

    def run_fk(self):
        success, res = self.node.compute_fk()
        if success:
            h, deg = res
            self.lbl_fk_result.config(text=f"✅ Result\nHeight (Z): {h:.4f} m\nAngle with XY Plane: {deg:.2f} °")
        else:
            messagebox.showerror("FK Error", res)

    def run_ik(self):
        try:
            target_h = float(self.ent_ik_h.get())
            target_deg = float(self.ent_ik_deg.get())
        except ValueError:
            messagebox.showerror("Input Error", "Height and Degree must be numbers.")
            return

        # 체크박스에서 True(체크됨)인 조인트 이름만 리스트로 뽑아냄
        movable_joints = [j_name for j_name, var in self.joint_vars.items() if var.get()]

        success, res = self.node.compute_ik(movable_joints, target_h, target_deg)
        
        self.txt_ik_result.config(state="normal")
        self.txt_ik_result.delete("1.0", tk.END)
        if success:
            result_str = "✅ Target Joint Values:\n"
            for j_name, val in res.items():
                result_str += f"- {j_name}: {val:.4f} rad\n"
            self.txt_ik_result.insert(tk.END, result_str)
        else:
            self.txt_ik_result.insert(tk.END, f"❌ Error: {res}")
        self.txt_ik_result.config(state="disabled")

def main(args=None):
    rclpy.init(args=args)
    node = KinematicsAnalysisNode()

    ros_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    ros_thread.start()

    root = tk.Tk()
    app = KinematicsApp(root, node)
    
    def on_closing():
        node.destroy_node()
        rclpy.shutdown()
        root.destroy()
        
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()

if __name__ == '__main__':
    main()