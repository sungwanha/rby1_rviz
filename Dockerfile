FROM osrf/ros:jazzy-desktop-full

# 필수 도구 및 Pinocchio 설치
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-jazzy-pinocchio \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# rosdep 초기화 (한 번만 수행)
RUN if [ ! -d "/etc/ros/rosdep/sources.list.d" ]; then rosdep init; fi && rosdep update

WORKDIR /app

# [추가] 터미널 접속 시 자동으로 환경 설정을 불러오도록 설정
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /app/install/setup.bash ]; then source /app/install/setup.bash; fi" >> /root/.bashrc

# 빌드 시에도 환경을 불러와서 실행하도록 수정
RUN bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

# 주의: 런타임에 볼륨이 들어오므로 여기서 빌드하지 않습니다.
# 실행 권한은 호스트에서 이미 부여되어 있어야 하지만, 안전을 위해 남겨둡니다.
ENTRYPOINT ["/app/main.sh"]