FROM osrf/ros:jazzy-desktop-full

# 필수 도구 및 의존성 설치
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-jazzy-pinocchio \
    python3-rosdep \
    # ADB 도구
    android-tools-adb \
    # OpenCV 및 Numpy
    python3-opencv \
    python3-numpy \
    # GUI 실행을 위한 최신 라이브러리 (Noble 버전 대응)
    libgl1 \
    libglx0 \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    && rm -rf /var/lib/apt/lists/*

# rosdep 초기화
RUN if [ ! -d "/etc/ros/rosdep/sources.list.d" ]; then rosdep init; fi && rosdep update

WORKDIR /app

# 터미널 접속 시 자동 환경 설정
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# 소스 코드가 있다면 빌드 (없다면 생략 가능)
# RUN bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

ENTRYPOINT ["/app/main.sh"]