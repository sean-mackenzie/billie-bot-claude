# BillieBot dev container — ROS 2 Humble on Ubuntu 22.04 (multi-arch; arm64-native on Apple Silicon)
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# --- ROS packages the workspace depends on ---------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-rplidar-ros \
    ros-humble-behaviortree-cpp \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-foxglove-bridge \
    ros-humble-rviz2 \
    ros-humble-teleop-twist-keyboard \
    # --- billiebot_sensor_tests: rosbag2 (authoritative bench recording) and
    # diagnostic_msgs (bench status/diagnostics topics) are pulled in transitively by
    # nav2/slam_toolbox today, but are declared explicitly here since
    # billiebot_sensor_tests depends on them directly, not on navigation ---
    ros-humble-rosbag2 \
    ros-humble-diagnostic-msgs \
    # --- build & system tools ---
    python3-pip \
    python3-colcon-common-extensions \
    python3-pytest \
    alsa-utils \
    libportaudio2 \
    libusb-1.0-0 \
    i2c-tools \
    # usbutils provides lsusb, shelled out to by billiebot_sensor_tests'
    # common/preflight.py for the OAK-D and XVF3800 USB preflight (i2c-tools is
    # the I2C counterpart, used for the MLX90640's i2cdetect preflight)
    usbutils \
    sqlite3 \
    && rm -rf /var/lib/apt/lists/*

# --- Python packages used by the nodes (no requirements.txt exists in-repo) -
# numpy is pinned to the 1.x ABI that tflite-runtime and depthai are built against;
# depthai is pinned to 2.32.0.0 because the OAK-D nodes use the v2 API -- do not bump
# to 3.x without porting oakd_dog_detector.py and oakd/oakd_bench_publisher.py.
# pillow already arrives transitively via matplotlib, but is listed explicitly because
# billiebot_sensor_tests' common/preview.py depends on it directly for the JPEG-compressed
# Foxglove preview topics -- a matplotlib change must not silently break bench visualization.
RUN pip3 install --no-cache-dir \
    pyserial numpy==1.26.4 sounddevice pyusb depthai==2.32.0.0 \
    pyyaml jinja2 matplotlib pillow fastapi uvicorn markdown \
    tflite-runtime

# Data directories used by billiebot_cognition (config/cognition.yaml)
RUN mkdir -p /var/lib/billiebot/snapshots /var/lib/billiebot/reports

# CycloneDDS as the RMW (the repo's multi-machine config assumes it)
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORKDIR /ws
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc

CMD ["bash"]