import os
import glob
import math
import numpy as np
import pandas as pd
import pinocchio as pin

# ==========================================
# 1. 설정 (경로 유지)
# ==========================================
INPUT_DIR = "/home/hsw/rby1_rviz/data/chunk-000"
OUTPUT_DIR = "/home/hsw/rby1_rviz/data/chunk-000_ee"
URDF_PATH = "./model/model_v1.2.urdf"

os.makedirs(OUTPUT_DIR, exist_ok=True)

# ==========================================
# 2. Pinocchio 모델 로드 및 오프셋 설정
# ==========================================
model = pin.buildModelFromUrdf(URDF_PATH)
data = model.createData()

q_neutral = pin.neutral(model)
D2R = math.pi / 180.0

def set_joint_val(q, joint_name, val_rad):
    if model.existJointName(joint_name):
        idx = model.joints[model.getJointId(joint_name)].idx_q
        q[idx] = val_rad

set_joint_val(q_neutral, "torso_1", -22.8 * D2R)
set_joint_val(q_neutral, "torso_3", 60.3 * D2R)

head_id = model.getFrameId("zed2i_left_camera_optical_frame")
ee_l_id = model.getFrameId("ee_left")
ee_r_id = model.getFrameId("ee_right")

# 🔥 C++과 동일한 오프셋 행렬 생성 함수 🔥
def get_offset_se3(xyz, rpy_deg):
    rpy_rad = np.array(rpy_deg) * D2R
    # Roll(x), Pitch(y), Yaw(z) 순서로 회전 행렬 생성
    R = pin.rpy.rpyToMatrix(rpy_rad[0], rpy_rad[1], rpy_rad[2])
    
    # C++의 SE3(R, 0) * SE3(I, xyz) 로직 구현
    se3_rot = pin.SE3(R, np.zeros(3))
    se3_trans = pin.SE3(np.eye(3), np.array(xyz))
    return se3_rot * se3_trans

# 오프셋 SE3 생성
T_off_l = get_offset_se3([-0.0073, -0.0051, -0.0720], [-97.58, 1.71, 15.36])
T_off_r = get_offset_se3([-0.0073, 0.0039, -0.0720], [98.99, 0.30, 163.89])

# 역행렬(Inverse) 미리 계산
T_off_l_inv = T_off_l.inverse()
T_off_r_inv = T_off_r.inverse()

# ==========================================
# 3. 변환 함수 (오프셋 보정 포함)
# ==========================================
def convert_joints_to_ee_pose(row_data):
    q = q_neutral.copy()
    
    for i in range(7):
        set_joint_val(q, f"left_arm_{i}", row_data[i])
    # 🔥 [수정됨] 1 - trigger 적용
    left_trigger = 1.0 - row_data[7]
    
    for i in range(7):
        set_joint_val(q, f"right_arm_{i}", row_data[8 + i])
    # 🔥 [수정됨] 1 - trigger 적용
    right_trigger = 1.0 - row_data[15]

    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)

    oM_head = data.oMf[head_id]
    oM_ee_l = data.oMf[ee_l_id]
    oM_ee_r = data.oMf[ee_r_id]

    # 1. 카메라 기준 상대 좌표 (로봇 EE)
    head_M_ee_l = oM_head.actInv(oM_ee_l)
    head_M_ee_r = oM_head.actInv(oM_ee_r)

    # 2. 오프셋 역연산을 통한 컨트롤러 좌표 복원
    controller_M_l = head_M_ee_l * T_off_l_inv
    controller_M_r = head_M_ee_r * T_off_r_inv

    # 3. 복원된 좌표에서 Position / Quaternion 추출
    pos_l = controller_M_l.translation
    quat_l = pin.Quaternion(controller_M_l.rotation).coeffs() 
    
    pos_r = controller_M_r.translation
    quat_r = pin.Quaternion(controller_M_r.rotation).coeffs()

    new_state = np.concatenate([
        pos_l, quat_l, [left_trigger],
        pos_r, quat_r, [right_trigger]
    ])
    
    return new_state.tolist()

# ==========================================
# 4. 일괄 변환(Batch Processing) 실행
# ==========================================
parquet_files = sorted(glob.glob(os.path.join(INPUT_DIR, "*.parquet")))

if not parquet_files:
    print(f"Error: No .parquet files found in {INPUT_DIR}")
else:
    print(f"Found {len(parquet_files)} files. Starting conversion...\n")

    for file_path in parquet_files:
        file_name = os.path.basename(file_path)
        output_path = os.path.join(OUTPUT_DIR, file_name)
        
        print(f"Processing: {file_name} ...", end=" ")
        
        df = pd.read_parquet(file_path)

        # observation.state 변환 수행
        if "observation.state" in df.columns:
            df["observation.state"] = df["observation.state"].apply(convert_joints_to_ee_pose)
            
        if "action" in df.columns:
            df["action"] = df["action"].apply(convert_joints_to_ee_pose)

        df.to_parquet(output_path, engine="pyarrow")
        print("Done!")

print("\nAll files have been successfully converted and saved to:")
print(OUTPUT_DIR)