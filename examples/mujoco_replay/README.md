# MuJoCo에서 샘플 에피소드 재생

Ubuntu 24.04 / ROS 2 Jazzy에서 RB-Y1 A v1.2 본체와 양쪽 그리퍼를 하나의 MuJoCo 장면으로 실행합니다.
이 저장소의 IK를 통해 샘플 456프레임을 30 Hz로 한 번 재생하고, 완료 또는 오류 시 텔레옵을 끕니다.
같은 ROS 도메인에는 이 예제의 시뮬레이터 하나만 실행합니다.

## 설치

GitHub 인증 후 두 저장소를 같은 workspace에 받습니다. `rby1_ros2` 모델·설정은 시연에 사용한 revision으로 고정합니다.

```bash
mkdir -p ~/rby1_replay_ws/src
cd ~/rby1_replay_ws
gh repo clone sungwanha/rby1_rviz src/rby1_rviz
git clone --recurse-submodules https://github.com/Tommoro-AI/rby1_ros2.git src/rby1_ros2
git -C src/rby1_ros2 checkout 59d0f6825f83b2471139bbecf8ccad48d22c74b6
git -C src/rby1_ros2 submodule update --init --recursive
```

ROS 2 Jazzy가 설치된 환경에서 의존성을 설치합니다.

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions python3-venv python3-pykdl python3-yaml \
  ros-jazzy-pinocchio ros-jazzy-mujoco-ros2-control \
  ros-jazzy-mujoco-ros2-control-msgs ros-jazzy-mujoco-vendor \
  ros-jazzy-controller-manager ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller ros-jazzy-diff-drive-controller \
  ros-jazzy-robot-state-publisher ros-jazzy-xacro \
  ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-rviz2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select \
  rby1_description rby1_gripper_description rby1_bringup rby1_teleop_cpp \
  --symlink-install --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
```

기존 MuJoCo 패키지가 있어도 plugin·msgs·vendor 세 패키지를 함께 갱신합니다.
plugin이 요구하는 service 타입과 `libmujoco.so`가 오래된 라이브러리와 맞지 않는 상황을 피하기 위해서입니다.

리플레이용 Python 의존성은 별도 환경에 설치합니다. ROS Python 패키지를 함께 사용하도록 system packages를 허용합니다.

```bash
python3 -m venv --system-site-packages ~/rby1_replay_ws/.venv
source ~/rby1_replay_ws/.venv/bin/activate
python -m pip install --only-binary=:all: pandas pyarrow
```

## 실행

사용하는 터미널마다 아래 공통 환경을 적용합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/rby1_replay_ws/install/setup.bash
source ~/rby1_replay_ws/.venv/bin/activate
cd ~/rby1_replay_ws/src/rby1_rviz
export ROS_DOMAIN_ID=99
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

터미널 1에서 모델을 생성·검사한 뒤 MuJoCo와 IK를 실행합니다.
생성 파일은 이 폴더의 `generated/` 아래에만 저장되며 원본 모델은 수정하지 않습니다.

```bash
python examples/mujoco_replay/prepare.py
python examples/mujoco_replay/check_model.py
ros2 launch examples/mujoco_replay/demo.launch.py headless:=false
```

화면 없는 환경에서는 `headless:=true`로 실행합니다. GPU 초기화에 문제가 있으면 `LIBGL_ALWAYS_SOFTWARE=1`을 적용합니다.

터미널 2에서 데이터와 실행 상태를 검사한 뒤 재생합니다.

```bash
python examples/mujoco_replay/replay_episode.py --validate-only
python examples/mujoco_replay/check_live.py
python examples/mujoco_replay/replay_episode.py
```

데이터 검사는 ROS 노드를 만들지 않습니다. 실행 상태 검사는 하나의 `/clock`, 본체 24개 관절,
독립된 양쪽 trigger 실측과 14개 활성 controller를 확인합니다.
재생은 현재 pose에서 첫 프레임으로 3초간 이동한 후 15.2초짜리 샘플을 한 번 전송합니다.
끝에서 2초간 상태를 확인하고 텔레옵을 비활성화한 뒤 종료합니다.
출력 JSON의 `status: completed`, `frames_sent: 456`, `teleop_disabled: true`를 확인합니다.
이 값은 프레임 전송·실측 수신·종료 확인 결과이며 정밀 추종 또는 파지 성공을 뜻하지 않습니다.

RViz가 필요하면 터미널 3에서 실행합니다.

```bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch rby1_bringup rviz.launch.py \
  gripper_side:=both use_sim_time:=true
```

MuJoCo 종료는 터미널 1에서 Ctrl+C로 합니다. 재생 중단은 터미널 2에서 Ctrl+C로 하며, 스크립트가 텔레옵 비활성화를 시도합니다.

## 연결과 확인 범위

`demo.launch.py`가 IK의 기존 topic을 다음 MuJoCo controller로 remap합니다.

| 대상 | MuJoCo endpoint |
| --- | --- |
| 본체 실측 | `/joint_states` |
| 왼팔 / 오른팔 명령 | `/left_arm_controller/joint_trajectory`, `/right_arm_controller/joint_trajectory` |
| 왼손 / 오른손 명령 | `/left_gripper_controller/joint_trajectory`, `/right_gripper_controller/joint_trajectory` |
| 왼손 / 오른손 실측 | `/left_gripper/joint_states`, `/right_gripper/joint_states` |

샘플의 손 입력은 `Float64` trigger 토픽으로 전달하며, 가짜 `/joint_states` 실측을 발행하지 않습니다.
손 실측은 검증 스크립트가 별도로 읽습니다. IK의 손 실측 입력은 remap하지 않아 목표 trigger가 실측값으로 덮어써지는 것을 방지합니다.

초기 자세는 기존 IK의 torso·elbow·wrist 자세이며 head는 0입니다. 외부 모델·설정은 설치된 ROS package 경로로 찾습니다.
이 예제는 `mujoco_ros2_control` / `mujoco_ros2_control_msgs` 0.1.2,
`mujoco_vendor` 0.1.0 / MuJoCo 3.12.0에서 headless 재생을 검증했습니다.
다른 plugin 버전에서는 위 모델·실행 검사를 먼저 확인합니다.

오른쪽 trigger에는 원본부터 빠른 반복 변화가 있습니다. 그리퍼의 원본 감쇠 설정도 유지하므로 진동·오버슈트가 재현될 수 있습니다.
본체 MuJoCo 모델의 TCP와 이 저장소 IK의 TCP에는 35 mm 차이가 있으며, 오차 출력은 IK의 보정된 pose 기준입니다.
이 예제는 pose 리플레이와 개폐 확인용이며 물체 파지·접촉과 실물 제어는 검증하지 않았습니다.
