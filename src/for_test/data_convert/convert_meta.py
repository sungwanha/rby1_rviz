import os
import json
import glob
import shutil
import numpy as np
import pandas as pd

# ==========================================
# 1. 경로 설정
# ==========================================
NEW_DATA_DIR = "/home/hsw/rby1_rviz/data/chunk-000_ee" # 우리가 변환한 데이터 폴더
OLD_META_DIR = "/home/hsw/rby1_rviz/meta"              # 온전한 원본 메타 폴더
NEW_META_DIR = "/home/hsw/rby1_rviz/meta_ee"           # 새로 만들 메타 폴더

# 기존 메타 폴더를 통째로 복사 (영상 시간, 에피소드 정보, 태그 등 완벽 보존)
if os.path.exists(NEW_META_DIR):
    shutil.rmtree(NEW_META_DIR)
shutil.copytree(OLD_META_DIR, NEW_META_DIR)
print(f"✅ 원본 메타 폴더 구조 복사 완료: {NEW_META_DIR}")

# ==========================================
# 2. 통계(Stats) 재계산
# ==========================================
print("📊 변환된 데이터에서 새로운 통계값을 계산 중...")
parquet_files = sorted(glob.glob(os.path.join(NEW_DATA_DIR, "*.parquet")))

all_actions, all_states = [], []

for file_path in parquet_files:
    df = pd.read_parquet(file_path)
    if "action" in df.columns:
        all_actions.append(np.stack(df["action"].values))
    if "observation.state" in df.columns:
        all_states.append(np.stack(df["observation.state"].values))

def compute_stats(data_list):
    if not data_list: return None
    concatenated = np.concatenate(data_list, axis=0) 
    return {
        "min": np.min(concatenated, axis=0).tolist(),
        "max": np.max(concatenated, axis=0).tolist(),
        "mean": np.mean(concatenated, axis=0).tolist(),
        "std": np.std(concatenated, axis=0, ddof=1).tolist()
    }

new_action_stats = compute_stats(all_actions)
new_state_stats = compute_stats(all_states)

# ==========================================
# 3. stats.json 업데이트
# ==========================================
stats_json_path = os.path.join(NEW_META_DIR, "stats.json")
with open(stats_json_path, "r") as f:
    stats_data = json.load(f)

if new_action_stats:
    stats_data["action"].update(new_action_stats)
if new_state_stats:
    stats_data["observation.state"].update(new_state_stats)

with open(stats_json_path, "w") as f:
    json.dump(stats_data, f, indent=4)
print("✅ stats.json 업데이트 완료")

# ==========================================
# 4. episodes 내의 parquet 파일 통계값 교체 (에피소드별 정밀 계산)
# ==========================================
print("🔍 에피소드별 개별 통계 계산 중...")
all_data_df = pd.concat([pd.read_parquet(f) for f in parquet_files], ignore_index=True)

episode_files = glob.glob(os.path.join(NEW_META_DIR, "episodes", "**", "*.parquet"), recursive=True)

for ep_file in episode_files:
    ep_df = pd.read_parquet(ep_file)
    
    for idx, row in ep_df.iterrows():
        ep_idx = row["episode_index"]
        ep_data = all_data_df[all_data_df["episode_index"] == ep_idx]
        
        if ep_data.empty:
            continue
            
        # 값의 존재 여부를 타입(list 또는 ndarray)으로 안전하게 확인
        action_min_val = row.get("stats/action/min")
        state_min_val = row.get("stats/observation.state/min")
            
        # Action 통계 업데이트 (값이 리스트/배열 형태로 존재할 때만)
        if "action" in ep_data.columns and isinstance(action_min_val, (list, np.ndarray)):
            ep_actions = np.stack(ep_data["action"].values)
            ep_df.at[idx, "stats/action/min"] = np.min(ep_actions, axis=0).tolist()
            ep_df.at[idx, "stats/action/max"] = np.max(ep_actions, axis=0).tolist()
            ep_df.at[idx, "stats/action/mean"] = np.mean(ep_actions, axis=0).tolist()
            ep_df.at[idx, "stats/action/std"] = np.std(ep_actions, axis=0, ddof=1).tolist()
            
        # Observation.state 통계 업데이트 (값이 리스트/배열 형태로 존재할 때만)
        if "observation.state" in ep_data.columns and isinstance(state_min_val, (list, np.ndarray)):
            ep_states = np.stack(ep_data["observation.state"].values)
            ep_df.at[idx, "stats/observation.state/min"] = np.min(ep_states, axis=0).tolist()
            ep_df.at[idx, "stats/observation.state/max"] = np.max(ep_states, axis=0).tolist()
            ep_df.at[idx, "stats/observation.state/mean"] = np.mean(ep_states, axis=0).tolist()
            ep_df.at[idx, "stats/observation.state/std"] = np.std(ep_states, axis=0, ddof=1).tolist()

    # 수정된 내용을 다시 덮어쓰기
    ep_df.to_parquet(ep_file, engine="pyarrow")
    print(f"✅ 에피소드 파케이 파일 개별 업데이트 완료: {os.path.basename(ep_file)}")