# 리플레이 샘플

`episode_000.parquet`은 로컬 `data/chunk-000_ee/file-000.parquet`에서
`episode_index == 0`인 행만 추출한 파일입니다.

- 456프레임, 30 Hz, 15.2초. `frame_index`는 0부터 455까지입니다.
- 원본 컬럼과 값은 유지했습니다. 영상은 포함하지 않습니다.
- `action.left_arm`, `action.right_arm`은 각각 `[x, y, z, qx, qy, qz, qw, trigger]`입니다.
- pose는 기존 IK 입력과 같은 `zed2i_left_camera_optical_frame` 기준이며 위치 단위는 m입니다.
- `trigger`는 0~1 값입니다. 원본 오른쪽 trigger의 반복 변화도 그대로 보존했습니다.

[MuJoCo에서 재생하기](../examples/mujoco_replay/README.md)
