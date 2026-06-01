# rby1_teleop_cpp

RBY1 RViz 표시와 실제 로봇용 IK 텔레옵 노드를 함께 실행하는 ROS 2 패키지입니다.

## 실행 구조

현재 설치되는 런치 파일은 `display.launch.py` 하나입니다.

```bash
ros2 launch rby1_teleop_cpp display.launch.py
```

이 런치 파일은 다음 노드를 실행합니다.

| Node | Package / Executable | 역할 |
| --- | --- | --- |
| `robot_state_publisher` | `robot_state_publisher/robot_state_publisher` | `model/model_v1.2.urdf`를 `/robot_description`으로 게시 |
| `rviz2` | `rviz2/rviz2` | `rby1.rviz` 설정으로 RViz 실행 |
| `teleop_ik_node` | `rby1_teleop_cpp/teleop_ik_node` | 선택된 pose 입력 모드를 받아 양팔 IK를 계산하고 joint trajectory 게시 |
| `proprio_pose_node` | `rby1_teleop_cpp/proprio_pose_node` | `/joint_states`에서 ZED 기준 좌/우 팔 pose 관측값 게시 |
| `rosbridge_server` | `rosbridge_server/rosbridge_websocket` | `use_rosbridge:=true`일 때만 WebSocket bridge 실행, port `9091` |

`robot_state_publisher`는 URDF와 `/joint_states`를 받아 각 링크의 TF를 만드는 노드입니다. RViz의 RobotModel은 `/robot_description`과 TF를 기준으로 로봇을 표시하므로 RViz 확인에는 이 노드가 필요합니다.

## 텔레옵 흐름

`teleop_ik_node`는 시작 직후 바로 로봇 명령을 내리지 않고, `/toggle_teleop` 서비스가 켜졌을 때만 IK 결과를 trajectory로 게시합니다.

입력:

| Topic / Service | Type | 설명 |
| --- | --- | --- |
| `/joint_states` | `sensor_msgs/msg/JointState` | 텔레옵 비활성 상태에서 현재 로봇 자세 초기화 |
| `/toggle_teleop` | `std_srvs/srv/SetBool` | 텔레옵 활성/비활성 전환 |

pose 입력 토픽은 `teleop_ik_node`의 `pose_input_mode` 파라미터로 하나만 선택합니다. 기존 `/simdd/pose_left`, `/simdd/pose_right` 토픽은 사용하지 않습니다.

| `pose_input_mode` | Left Topic | Right Topic | Type | Data |
| --- | --- | --- | --- | --- |
| `quaternion` | `/control/body/arm_left_controller/pose` | `/control/body/arm_right_controller/pose` | `geometry_msgs/msg/Pose` | position 3 + quaternion 4 |
| `six_d` | `/control/body/arm_left_controller/pose_6d` | `/control/body/arm_right_controller/pose_6d` | `std_msgs/msg/Float64MultiArray` | `data[0:3] = [x, y, z]`, `data[3:9] = [r11, r12, r13, r21, r22, r23]` |
| `rot_vec` | `/control/body/arm_left_controller/pose_rot_vec` | `/control/body/arm_right_controller/pose_rot_vec` | `std_msgs/msg/Float64MultiArray` | `data[0:3] = [x, y, z]`, `data[3:6] = [rx, ry, rz]` |

출력:

| Topic | Type | 설명 |
| --- | --- | --- |
| `/leader/master_arm_joint_left/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | 왼팔 실제 로봇 제어 명령 |
| `/leader/master_arm_joint_right/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | 오른팔 실제 로봇 제어 명령 |
| `/joint_states` | `sensor_msgs/msg/JointState` | RViz 표시용 현재 IK 상태 |
| `/tf` | `tf2_msgs/msg/TFMessage` | `target_left`, `target_right` 시각화 TF |

관측 pose 출력:

`proprio_pose_node`는 `/joint_states`를 받아 Pinocchio FK를 계산하고, IK 입력과 같은 ZED 기준 pose를 publish합니다. 기준 프레임은 `zed2i_left_camera_optical_frame`이며, 손목 오프셋은 IK 입력과 반대로 보정합니다.

| Topic | Type | Data |
| --- | --- | --- |
| `/sensors/proprio/arm_left/pose` | `geometry_msgs/msg/Pose` | position 3 + quaternion 4 |
| `/sensors/proprio/arm_right/pose` | `geometry_msgs/msg/Pose` | position 3 + quaternion 4 |
| `/sensors/proprio/arm_left/pose_6d` | `std_msgs/msg/Float64MultiArray` | `data[0:3] = [x, y, z]`, `data[3:9] = [r11, r12, r13, r21, r22, r23]` |
| `/sensors/proprio/arm_right/pose_6d` | `std_msgs/msg/Float64MultiArray` | `data[0:3] = [x, y, z]`, `data[3:9] = [r11, r12, r13, r21, r22, r23]` |
| `/sensors/proprio/arm_left/pose_rot_vec` | `std_msgs/msg/Float64MultiArray` | `data[0:3] = [x, y, z]`, `data[3:6] = [rx, ry, rz]` |
| `/sensors/proprio/arm_right/pose_rot_vec` | `std_msgs/msg/Float64MultiArray` | `data[0:3] = [x, y, z]`, `data[3:6] = [rx, ry, rz]` |

## 사용 방법

빌드 후 환경을 source합니다.

```bash
colcon build --symlink-install
source install/setup.bash
```

런치 파일을 실행합니다.

```bash
ros2 launch rby1_teleop_cpp display.launch.py
```

`proprio_pose_node`는 기본으로 같이 실행됩니다. 필요 없으면 끌 수 있습니다.

```bash
ros2 launch rby1_teleop_cpp display.launch.py publish_proprio_poses:=false
```

기본 입력 모드는 `quaternion`입니다. 다른 모드를 쓰려면 launch 파라미터를 지정합니다.

```bash
ros2 launch rby1_teleop_cpp display.launch.py pose_input_mode:=six_d
```

```bash
ros2 launch rby1_teleop_cpp display.launch.py pose_input_mode:=rot_vec
```

rosbridge는 기본으로 실행하지 않습니다. WebSocket 클라이언트가 필요할 때만 켭니다.

```bash
ros2 launch rby1_teleop_cpp display.launch.py use_rosbridge:=true
```

## 실제 로봇 사용

실제 로봇에서 사용할 때는 데이터 리플레이나 임의 pose 테스트를 켜지 않습니다. 실제 로봇 제어기와 같은 `ROS_DOMAIN_ID`를 사용하고, 로봇의 `/joint_states`가 들어오는지 먼저 확인합니다.

```bash
export ROS_DOMAIN_ID=<robot_domain_id>
source install/setup.bash
ros2 topic echo /joint_states --once
```

텔레옵 노드와 관측 pose 노드를 실행합니다. 외부 추론/제어 노드가 quaternion pose를 보낸다면 기본 모드 그대로 실행합니다.

```bash
ros2 launch rby1_teleop_cpp display.launch.py pose_input_mode:=quaternion
```

외부 추론/제어 노드가 6D 또는 rotation vector를 보낸다면 모드를 맞춥니다.

```bash
ros2 launch rby1_teleop_cpp display.launch.py pose_input_mode:=six_d
```

```bash
ros2 launch rby1_teleop_cpp display.launch.py pose_input_mode:=rot_vec
```

실제 로봇에 명령을 내리기 전 입력 토픽이 들어오는지 확인합니다.

```bash
ros2 topic echo /control/body/arm_left_controller/pose --once
ros2 topic echo /control/body/arm_right_controller/pose --once
```

`pose_input_mode:=six_d`일 때는 `/pose_6d`, `pose_input_mode:=rot_vec`일 때는 `/pose_rot_vec` 토픽을 확인합니다.

관측 pose 출력은 다음 토픽에서 확인합니다.

```bash
ros2 topic echo /sensors/proprio/arm_left/pose --once
ros2 topic echo /sensors/proprio/arm_right/pose --once
```

입력 pose와 현재 자세가 정상임을 확인한 뒤 텔레옵을 켭니다.

```bash
ros2 service call /toggle_teleop std_srvs/srv/SetBool "{data: true}"
```

중지:

```bash
ros2 service call /toggle_teleop std_srvs/srv/SetBool "{data: false}"
```

실제 로봇 실행 시 주의사항:

- `data_replay:=true`와 `sim_test:=true`는 실제 로봇 domain에서 사용하지 않습니다.
- `use_rosbridge`는 필요 없으면 켜지 않습니다.
- `teleop_ik_node`는 텔레옵 ON 상태에서 `/leader/master_arm_joint_left/joint_trajectory`, `/leader/master_arm_joint_right/joint_trajectory`로 실제 명령을 publish합니다.
- 텔레옵 ON 전에 로봇 주변 안전 상태와 입력 pose 방향/스케일을 확인합니다.

## RViz 데이터 리플레이 테스트

실제 로봇과 같은 ROS domain에 연결된 상태에서 테스트 pose를 보내면 `/leader/master_arm_joint_*` 명령 토픽이 실제 제어기로 전달될 수 있습니다. 데이터 리플레이 테스트는 별도 `ROS_DOMAIN_ID`에서 실행합니다.

기본 리플레이 파일은 `data/chunk-000_ee/file-000.parquet`이고, 기본 컬럼은 `action.left_arm`, `action.right_arm`입니다. 각 arm 컬럼은 `[x, y, z, qx, qy, qz, qw, trigger]`로 해석합니다.

```bash
export ROS_DOMAIN_ID=99
source install/setup.bash
ros2 launch rby1_teleop_cpp display.launch.py data_replay:=true pose_input_mode:=quaternion
```

`data_replay:=true`는 `ee_parquet_pose_player.py`를 함께 실행합니다. 이 노드는 선택한 `pose_input_mode`에 맞춰 parquet의 EE pose를 `/control/body/...` 토픽으로 publish하고 `/toggle_teleop`을 자동으로 켭니다.

6D 입력 테스트:

```bash
export ROS_DOMAIN_ID=99
source install/setup.bash
ros2 launch rby1_teleop_cpp display.launch.py data_replay:=true pose_input_mode:=six_d
```

Rotation vector 입력 테스트:

```bash
export ROS_DOMAIN_ID=99
source install/setup.bash
ros2 launch rby1_teleop_cpp display.launch.py data_replay:=true pose_input_mode:=rot_vec
```

관측 pose를 재생하려면 `replay_column_prefix:=observation`을 지정합니다.

```bash
export ROS_DOMAIN_ID=99
source install/setup.bash
ros2 launch rby1_teleop_cpp display.launch.py data_replay:=true replay_column_prefix:=observation
```

다른 parquet 파일이나 재생 속도를 쓰려면 다음처럼 지정합니다.

```bash
ros2 launch rby1_teleop_cpp display.launch.py data_replay:=true parquet_path:=/absolute/path/to/file.parquet replay_rate_hz:=30.0
```

## RViz 임의 pose 테스트

`sim_test:=true`는 parquet 데이터가 아니라 작은 sin 궤적의 임의 pose를 publish합니다. 데이터 기반 검증에는 `data_replay:=true`를 사용합니다.

```bash
export ROS_DOMAIN_ID=99
source install/setup.bash
ros2 launch rby1_teleop_cpp display.launch.py sim_test:=true pose_input_mode:=quaternion
```

텔레옵을 시작합니다.

```bash
ros2 service call /toggle_teleop std_srvs/srv/SetBool "{data: true}"
```

텔레옵을 종료합니다.

```bash
ros2 service call /toggle_teleop std_srvs/srv/SetBool "{data: false}"
```

## Docker

`docker/docker-compose.yml`의 컨테이너는 host network를 사용하며 기본 ROS 설정은 다음과 같습니다.

```yaml
ROS_DOMAIN_ID: 52
RMW_IMPLEMENTATION: rmw_cyclonedds_cpp
```

개발 컨테이너는 시작 시 `COLCON_BUILD_ON_START=1`로 빌드를 수행합니다.
