FROM ros:jazzy-ros-base AS base

ENV DEBIAN_FRONTEND=noninteractive

# 필수 도구 및 의존성 설치
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-jazzy-rviz2 \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-pinocchio \
    ros-jazzy-rosbridge-suite \
    python3-tornado \
    python3-twisted \
    python3-pymongo \
    python3-rosdep \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    # ADB 도구
    android-tools-adb \
    # OpenCV 및 Numpy
    python3-opencv \
    python3-numpy \
    python3-pandas \
    python3-pip \
    # GUI 실행을 위한 최신 라이브러리 (Noble 버전 대응)
    libgl1 \
    libglx0 \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir --break-system-packages --only-binary=:all: pyarrow

RUN apt-get update && apt-get install -y \
    ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# rosdep 초기화
RUN if [ ! -d "/etc/ros/rosdep/sources.list.d" ]; then rosdep init; fi && rosdep update

WORKDIR /app

COPY scripts/entrypoint.sh /usr/local/bin/rby1_entrypoint.sh
RUN chmod +x /usr/local/bin/rby1_entrypoint.sh

# 터미널 접속 시 자동 환경 설정
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /app/install/setup.bash ]; then source /app/install/setup.bash; fi" >> /root/.bashrc

ENTRYPOINT ["/usr/local/bin/rby1_entrypoint.sh"]
CMD ["sleep", "infinity"]

FROM base AS dev

ENV COLCON_BUILD_ON_START=1
WORKDIR /app

FROM base AS prod-build

WORKDIR /workspace/src/rby1_teleop_cpp
COPY . .
WORKDIR /workspace
RUN bash -c "source /opt/ros/jazzy/setup.bash && colcon build --merge-install"

FROM base AS prod

ENV COLCON_BUILD_ON_START=0
WORKDIR /app
COPY --from=prod-build /workspace/install /app/install
