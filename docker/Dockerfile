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
    sqlite3 \
    && rm -rf /var/lib/apt/lists/*

# --- Python packages used by the nodes (no requirements.txt exists in-repo) -
RUN pip3 install --no-cache-dir \
    pyserial numpy sounddevice pyusb depthai \
    pyyaml jinja2 matplotlib fastapi uvicorn markdown \
    tflite-runtime

# Data directories used by billiebot_cognition (config/cognition.yaml)
RUN mkdir -p /var/lib/billiebot/snapshots /var/lib/billiebot/reports

# CycloneDDS as the RMW (the repo's multi-machine config assumes it)
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORKDIR /ws
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc

CMD ["bash"]