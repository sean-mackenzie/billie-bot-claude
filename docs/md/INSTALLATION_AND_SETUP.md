# BillieBot — Installation & Setup Guide

**Audience:** a developer starting from a bare operating system.
**Dev host targeted by Part 1:** 2024 MacBook Pro, Apple M4 Pro (arm64), 24 GB RAM, macOS Tahoe 26.x.
**Robot targets covered by Part 2:** NVIDIA Jetson Orin Nano (JetPack 6 / Ubuntu 22.04), Raspberry Pi 5, Arduino Nano.

This guide takes you from zero installed software to (1) building and running the **entire BillieBot stack in mock mode on your Mac**, and (2) provisioning the **robot's onboard computers** for real-hardware deployment. Every fork in the road is labeled **Recommended** or **Quickest**; each major step ends with a **Verify** check.

> **Conventions used throughout**
> - The repository (`github.com/sean-mackenzie/billie-bot-claude`) is assumed to be cloned at `~/billie-bot-claude` on every machine. Adjust paths if yours differs.
> - Commands prefixed `host$` run on your Mac's terminal; `container$` inside a Docker container; `jetson$` / `pi$` on the robot computers over SSH.
> - The ROS 2 workspace inside the repo is `billiebot_ws/` and is always mounted/opened at `/ws` inside containers.

---

## Table of contents

- [0. System overview](#0-system-overview)
- [Part 1 — Development host (macOS, Apple Silicon)](#part-1--development-host-macos-apple-silicon)
  - [1.1 Base tools](#11-base-tools-xcode-clt-homebrew-apps)
  - [1.2 Get the code](#12-get-the-code)
  - [1.3 Path A (Recommended): project dev container](#13-path-a-recommended-project-dev-container)
  - [1.4 Path B (Quickest): stock ROS image](#14-path-b-quickest-stock-ros-image)
  - [1.5 Build the workspace](#15-build-the-workspace)
  - [1.6 Run the full stack in mock mode](#16-run-the-full-stack-in-mock-mode)
  - [1.7 Visualization from the Mac](#17-visualization-from-the-mac)
  - [1.8 Optional: VS Code Dev Containers](#18-optional-vs-code-dev-containers)
  - [1.9 Flash the Arduino firmware from the Mac](#19-flash-the-arduino-firmware-from-the-mac)
  - [1.10 Alternatives considered (and why not)](#110-alternatives-considered-and-why-not)
- [Part 2 — Robot deployment](#part-2--robot-deployment)
  - [2.1 Network plan (do this first)](#21-network-plan-do-this-first)
  - [2.2 Jetson Orin Nano](#22-jetson-orin-nano)
  - [2.3 Raspberry Pi](#23-raspberry-pi)
  - [2.4 Arduino Nano (on-robot)](#24-arduino-nano-on-robot)
  - [2.5 Multi-machine bringup](#25-multi-machine-bringup)
- [Part 3 — Build, first run & verification](#part-3--build-first-run--verification)
- [Appendix A — Complete dependency reference](#appendix-a--complete-dependency-reference)
- [Appendix B — ML model assets](#appendix-b--ml-model-assets)
- [Appendix C — Troubleshooting](#appendix-c--troubleshooting)
- [Appendix D — Known discrepancies & decisions](#appendix-d--known-discrepancies--decisions)

---

## 0. System overview

### 0.1 Hardware and what runs where

| Component | Role | Software that touches it |
|---|---|---|
| **Jetson Orin Nano** | Primary compute | Nav2, SLAM/AMCL, EKF, `base_bridge`, RPLidar driver, OAK-D detector, mission BT (`jetson.launch.py`) |
| **Raspberry Pi 5** | Secondary compute | Thermal, NoIR camera, audio classifier/speaker, state fusion, SQLite logger, report server (`pi.launch.py`) |
| **Arduino Nano** (ATmega328) | Motor PID @ 30 Hz, encoders, battery ADC | ROSArduinoBridge firmware, serial 57600 baud to Jetson |
| **RPLidar A1** | 2D lidar for SLAM/Nav | `rplidar_ros` (USB serial, 115200) |
| **OAK-D Lite** | On-camera YOLOv8n spatial dog detection | `depthai` Python SDK (USB 3) |
| **MLX90640** | 32×24 thermal imager | Adafruit Blinka + `adafruit-circuitpython-mlx90640` (I²C bus 1, addr 0x33) |
| **Pi Camera 3 NoIR** | Near-IR night imaging | `picamera2` (CSI) |
| **ReSpeaker XVF3800** | 4-mic array, audio classification + DoA | `sounddevice` (USB audio) + `pyusb` (DoA, VID 0x2886) |
| **MAX98357A + speaker** | Sound playback | `aplay` (ALSA, I²S DAC as card 0) |
| **L298N + DC motors + encoders** | Differential drive | Driven by the Arduino firmware |
| **Your Mac** | Development, mock-mode testing, visualization, firmware flashing | Docker (ROS 2 Humble container), Foxglove Studio, Arduino IDE |

### 0.2 Software stack at a glance

- **ROS 2 Humble** on Ubuntu 22.04 — everywhere (container on the Mac and Pi, native on the Jetson).
- **Build:** `colcon build --symlink-install` in `billiebot_ws/`.
- **DDS:** CycloneDDS with a static peers list (`billiebot_bringup/config/cyclonedds.xml`) because Wi-Fi multicast is unreliable.
- **Hardware-free development:** every launch file accepts `mock:=true`. There is no Gazebo simulation; mock mode *is* the sim story.
- **Not in the repo, installed by this guide:** the `rplidar_ros` driver, ~15 pip packages (there is no `requirements.txt`), two ML model files (YAMNet `.tflite`, YOLOv8n `.blob`), ALSA/udev/I²C system configuration, and the `/var/lib/billiebot` data directories.

### 0.3 What you need before starting

- Your Mac with admin rights, ~15 GB free disk, and a reasonable internet connection.
- For Part 2: the Jetson + a ≥64 GB microSD (A2), the Pi + a ≥32 GB microSD, a USB-C/microSD reader, and monitor/keyboard for first boot (or headless config, covered below).
- Rough time: **Part 1 ≈ 1–1.5 h** (mostly downloads), **Part 2 ≈ 2–4 h** across both boards.

---

# Part 1 — Development host (macOS, Apple Silicon)

Everything in Part 1 runs the full stack **in mock mode** — no robot hardware required. The only hardware step is §1.9 (flashing the Arduino over USB, which you can also defer to Part 2).

## 1.1 Base tools (Xcode CLT, Homebrew, apps)

**Step 1 — Xcode Command Line Tools** (provides `git`, compilers):

```bash
host$ xcode-select --install
```

Click *Install* in the dialog and wait for it to finish.

**Verify:** `git --version` prints a version.

**Step 2 — Homebrew** (package manager for the rest):

```bash
host$ /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

Follow the "Next steps" it prints (it adds `brew` to your PATH via `~/.zprofile`), then open a new terminal.

**Verify:** `brew --version` prints a version.

**Step 3 — Applications:**

```bash
host$ brew install --cask docker-desktop     # Docker Desktop for Mac
host$ brew install --cask foxglove-studio    # visualization (native macOS app)
host$ brew install --cask arduino-ide        # firmware flashing (§1.9)
```

> If `docker-desktop` isn't found in your Homebrew version, use `brew install --cask docker` (older cask name) or download from <https://www.docker.com/products/docker-desktop/>.
>
> **Alternative:** [OrbStack](https://orbstack.dev) (`brew install --cask orbstack`) is a lighter, faster Docker Desktop replacement on Apple Silicon and is drop-in compatible with every `docker` command in this guide.

Launch **Docker Desktop** once from Applications and complete its first-run setup. In *Settings → Resources*, giving it **8 GB memory / 6 CPUs** is comfortable for this project on a 24 GB machine.

**Verify:**

```bash
host$ docker run --rm hello-world
```

prints "Hello from Docker!".

## 1.2 Get the code

```bash
host$ git clone https://github.com/sean-mackenzie/billie-bot-claude.git ~/billie-bot-claude   # skip if you already have it
host$ cd ~/billie-bot-claude
host$ ls billiebot_ws/src
```

**Verify:** you see the 10 packages: `billiebot_interfaces`, `billiebot_description`, `billiebot_base`, `billiebot_navigation`, `billiebot_perception`, `billiebot_audio`, `billiebot_cognition`, `billiebot_mission`, `billiebot_bringup`, `billiebot_tests`.

## 1.3 Path A (Recommended): project dev container

**Why this path:** the robot runs ROS 2 Humble on Ubuntu 22.04 (arm64). A Docker container from the official `ros:humble` image is a byte-for-byte match of that environment, runs **natively** on the M4 (the image is multi-arch with arm64), and bakes in every dependency once so rebuilds are instant.

> ⚠️ **Do not use `osrf/ros:humble-desktop`** as the base image. As of July 2026 it is published **amd64-only**, so on Apple Silicon it would run under slow QEMU emulation. The official `ros:humble` library image is multi-arch (amd64 + arm64); this guide installs the desktop tools on top of it.

**Step 1 — Create the Dockerfile.** Save the following as `~/billie-bot-claude/docker/Dockerfile` (create the `docker/` directory; committing it to the repo is up to you):

```dockerfile
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
    # --- build & system tools ---
    python3-pip \
    python3-colcon-common-extensions \
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
```

Notes:
- `tflite-runtime` publishes arm64 Linux wheels for Python 3.10 (Ubuntu 22.04). If `pip` ever fails to find it, substitute `tensorflow` — the audio node falls back to `tf.lite` automatically (`billiebot_audio/audio_classifier.py`).
- `picamera2` and `adafruit-blinka`/`adafruit-circuitpython-mlx90640` are deliberately **not** installed here — they are Raspberry-Pi-only. The NoIR and thermal nodes fall back to mock mode on the Mac.

**Step 2 — Build the image** (~5–10 min, ~6 GB):

```bash
host$ cd ~/billie-bot-claude
host$ docker build -t billiebot-dev docker/
```

**Verify:** `docker image inspect billiebot-dev --format '{{.Architecture}}'` prints `arm64`.

**Step 3 — Start the container**, mounting the workspace and publishing the Foxglove-bridge and report-server ports:

```bash
host$ docker run -it --name billiebot-dev \
    -v "$HOME/billie-bot-claude/billiebot_ws:/ws" \
    -p 8765:8765 \
    -p 8080:8080 \
    billiebot-dev
```

You land in a `root@…:/ws#` shell. Day-to-day container management:

```bash
host$ docker start -ai billiebot-dev     # re-enter after exit/reboot
host$ docker exec -it billiebot-dev bash # open a SECOND shell (needed for tests)
```

**Verify (inside the container):**

```bash
container$ ros2 doctor --report > /tmp/doctor_report.txt
container$ grep -i "distribution name" /tmp/doctor_report.txt
```

shows `distribution name : humble` with no fatal errors. (Don't pipe `ros2 doctor --report`
directly into `head` — the report is longer than a few lines, and `head` closing the pipe
early causes a harmless but alarming-looking `BrokenPipeError` traceback. Redirecting to a
file first avoids it; check the rest of the report with `head -30 /tmp/doctor_report.txt`
if you want to skim more.)

Continue to [§1.5 Build the workspace](#15-build-the-workspace).

## 1.4 Path B (Quickest): stock ROS image

**Why this path:** zero files to create — one `docker run` plus one paste-block gets you to a running mock stack in ~15 minutes. The trade-off: dependencies live only inside the container instance, so if you delete it you re-install them (use `docker start`, not `docker run`, to come back to it).

```bash
host$ cd ~/billie-bot-claude
host$ docker run -it --name billiebot-quick \
    -v "$HOME/billie-bot-claude/billiebot_ws:/ws" \
    -p 8765:8765 -p 8080:8080 \
    ros:humble bash
```

Then paste this single block inside the container (~5 min):

```bash
apt-get update && apt-get install -y --no-install-recommends \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
  ros-humble-robot-localization ros-humble-rplidar-ros ros-humble-behaviortree-cpp \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher ros-humble-xacro \
  ros-humble-rmw-cyclonedds-cpp ros-humble-foxglove-bridge \
  python3-pip python3-colcon-common-extensions libportaudio2 alsa-utils && \
pip3 install pyserial numpy sounddevice pyusb depthai pyyaml jinja2 \
  matplotlib fastapi uvicorn markdown tflite-runtime && \
mkdir -p /var/lib/billiebot/snapshots /var/lib/billiebot/reports && \
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

**Verify:** `ros2 pkg prefix nav2_bringup` prints `/opt/ros/humble`.

Continue to §1.5. (Everything below works identically in Path A and Path B containers.)

## 1.5 Build the workspace

Inside the container:

```bash
container$ cd /ws
container$ source /opt/ros/humble/setup.bash    # already in .bashrc on Path A
container$ colcon build --symlink-install
container$ source install/setup.bash
```

Expect ~2–4 min on the M4. Warnings about `setup.py` deprecation from the `ament_python` packages are normal on Humble.

> `--symlink-install` matters: because the workspace is volume-mounted from macOS, edits you make to Python nodes and config YAMLs **on the Mac** (in your editor of choice) take effect in the container without rebuilding.

**Verify:**

```bash
container$ colcon build --symlink-install 2>&1 | tail -3   # "Summary: 10 packages finished"
container$ ros2 interface show billiebot_interfaces/msg/DogState | head -5
```

## 1.6 Run the full stack in mock mode

```bash
container$ ros2 launch billiebot_bringup 14_full_bringup.launch.py mock:=true
```

This is rung 14 of the bringup ladder — it chain-includes every subsystem (nav, perception, audio, cognition, mission) with all hardware mocked.

**Optional — supply a map so Nav2 fully activates.** Without a map, `map_server` logs
`yaml-filename parameter is empty` and Nav2's lifecycle bringup stalls at
`Activating planner_server` (the global costmap's static layer waits forever for `/map`).
The Verify topics/services below still appear, so this doesn't block the rest of Part 1 —
but navigation itself stays inactive. To run with a map:

1. Place `<name>.yaml` + `<name>.pgm` in `billiebot_ws/src/billiebot_navigation/maps/`.
   The yaml's `image:` field must exactly match the `.pgm` filename (it is resolved
   relative to the yaml's own directory).
2. Rebuild so the map is installed:
   `colcon build --symlink-install --packages-select billiebot_navigation`.
3. Launch with the `map` argument — use an **absolute path** (relative paths resolve
   against the launch process's working directory and only work by accident):

```bash
container$ ros2 launch billiebot_bringup 14_full_bringup.launch.py mock:=true \
    map:=/ws/src/billiebot_navigation/maps/my_apartment_v1.yaml
```

With a map loaded, the launch log shows
`lifecycle_manager_navigation: Managed nodes are active`, and
`ros2 topic echo /map --once --field info` (second shell) returns the map metadata.

**Verify** (in a second shell — `docker exec -it billiebot-dev bash`, then `cd /ws && source install/setup.bash`):

```bash
container$ ros2 topic list | grep -E "billie|dog|audio|scan|odom"
container$ ros2 topic echo /billie/state --once
container$ ros2 service list | grep -E "e_stop|set_mode"
```

You should see `/billie/state`, `/dog/detections_3d`, `/audio/events`, `/scan`, `/odom`, and the `/e_stop` and `/set_mode` services. For the full 22-test acceptance suite, see [Part 3](#part-3--build-first-run--verification).

## 1.7 Visualization from the Mac

**Recommended: Foxglove Studio** (native macOS app — no X11, works over Wi-Fi to the robot later).

1. In a container shell with the workspace sourced, start the bridge:

   ```bash
   container$ ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
   ```

2. Open Foxglove Studio on the Mac → *Open connection…* → *Foxglove WebSocket* → `ws://localhost:8765`.

**Verify:** the topic list populates; add a *Raw Messages* panel on `/billie/state` and watch mock updates arrive.

**Fallback: RViz2 via XQuartz** (only if you specifically need RViz):

```bash
host$ brew install --cask xquartz
```

Log out/in (or reboot), open XQuartz → *Settings → Security* → check **Allow connections from network clients**, then:

```bash
host$ xhost +localhost
host$ docker exec -it -e DISPLAY=host.docker.internal:0 -e LIBGL_ALWAYS_SOFTWARE=1 billiebot-dev \
      bash -lc "rviz2"
```

Expect software-rendered (slower) graphics; Foxglove is the better daily driver on macOS.

## 1.8 Optional: VS Code Dev Containers

If you use VS Code: install it (`brew install --cask visual-studio-code`) plus the **Dev Containers** extension, then save this as `~/billie-bot-claude/.devcontainer/devcontainer.json`:

```jsonc
{
  "name": "billiebot-dev",
  "build": { "dockerfile": "../docker/Dockerfile" },
  "workspaceMount": "source=${localWorkspaceFolder}/billiebot_ws,target=/ws,type=bind",
  "workspaceFolder": "/ws",
  "forwardPorts": [8765, 8080],
  "customizations": {
    "vscode": { "extensions": ["ms-python.python", "ms-iot.vscode-ros"] }
  }
}
```

*Command Palette → "Dev Containers: Reopen in Container"* gives you an integrated terminal, Python IntelliSense against the container's ROS environment, and the same mounts/ports as §1.3.

## 1.9 Flash the Arduino firmware from the Mac

The motor-controller firmware is the ROSArduinoBridge fork at
`reference_my_bot/diff-drive-motor-controller/arduino-nano-firmware/ROSArduinoBridge/`. One modification is **required** (per `firmware/README.md`, safety requirement SYS-PLT-5).

1. Connect the Arduino Nano over USB. Modern macOS includes the CH340 USB-serial driver; the port appears as `/dev/cu.usbserial-*`.

   **Verify:** `ls /dev/cu.usbserial*` shows a device.

2. Open Arduino IDE → *File → Open* → `~/billie-bot-claude/reference_my_bot/diff-drive-motor-controller/arduino-nano-firmware/ROSArduinoBridge/ROSArduinoBridge.ino`.

3. Edit line 117 — change the motor-watchdog timeout from 2000 ms to **500 ms**:

   ```cpp
   #define AUTO_STOP_INTERVAL 500
   ```

4. *Tools → Board* → **Arduino Nano**; *Tools → Processor* → **ATmega328P** (if upload fails with a sync error, switch to **ATmega328P (Old Bootloader)** — most Nano clones need it); *Tools → Port* → your `/dev/cu.usbserial-*`.

5. Click **Upload**.

6. **Verify:** open *Tools → Serial Monitor*, set **57600 baud** and line ending **Carriage Return** (the firmware executes a command on CR, `chr == 13` — plain "Newline" will not work), and exercise the protocol (`commands.h`):
   - send `b` → replies `57600` (baud check)
   - send `e` → replies `0 0` (encoder counts)
   - with wheels **off the ground** and motor power connected, send `m 30 30` → motors spin, then **auto-stop after ~0.5 s** — this confirms the watchdog change took.

Leave the config `#define`s as-is: the firmware is already set for `ARDUINO_ENC_COUNTER` + `L298_MOTOR_DRIVER`, and this combination needs **no external Arduino libraries**. The optional BNO055 IMU extension described in `firmware/README.md` is deferred until the A4/A5 encoder rewire (`docs/MEASURE_ME.md`); the stack runs with `use_imu: false`.

## 1.10 Alternatives considered (and why not)

| Route | Verdict |
|---|---|
| **UTM/Parallels Ubuntu 22.04 VM** | Works, native GUI for RViz, but heavier than Docker (dedicated RAM/disk), no image reproducibility, and slower file sharing with macOS. Use it only if you strongly prefer a full desktop Linux. |
| **RoboStack (conda-forge ROS on macOS)** | Runs Humble natively on Apple Silicon — impressive, but package coverage for this stack (Nav2 + slam_toolbox + BehaviorTree.CPP v4 + rplidar) is not guaranteed on `osx-arm64`, and you'd be debugging an environment the robot will never run. Not worth the divergence risk. |
| **Native macOS ROS 2 source build** | Tier-3 platform, hours of build time, frequent breakage. Avoid. |

---

# Part 2 — Robot deployment

## 2.1 Network plan (do this first)

The two robot computers discover each other via **static unicast peers** (multicast is disabled) configured in `billiebot_ws/src/billiebot_bringup/config/cyclonedds.xml`, which ships with these defaults:

| Machine | IP in shipped config |
|---|---|
| Jetson Orin Nano | `192.168.42.100` |
| Raspberry Pi | `192.168.42.101` |

1. In your router, give both boards **DHCP reservations** (either matching the shipped IPs, or your own choices).
2. If you change the IPs, edit the two `<Peer address="…"/>` lines in the **source** file above and rebuild (`colcon build --symlink-install`) on both machines — the launch files point `CYCLONEDDS_URI` at the *installed* copy of that file.
3. Two things the launch files do **not** do, so you must (both boards, `~/.bashrc`):

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # launch files set CYCLONEDDS_URI but not the RMW itself
# ROS_DOMAIN_ID defaults to 0 everywhere; only set it if your network has other ROS 2 systems:
# export ROS_DOMAIN_ID=42
```

Without `RMW_IMPLEMENTATION`, Humble silently uses Fast DDS and the CycloneDDS peers file is ignored — the classic "each machine only sees its own topics" failure.

## 2.2 Jetson Orin Nano

### 2.2.1 Flash JetPack 6

**Recommended (from your Mac):** the Orin Nano **Developer Kit** boots from microSD.

1. Download the **JetPack 6 SD card image** for Orin Nano from <https://developer.nvidia.com/embedded/jetpack>.
2. Flash it to the microSD:

   ```bash
   host$ brew install --cask balenaetcher
   ```

   Open balenaEtcher → select the image → select the SD card → Flash.
3. Insert, connect monitor/keyboard/Ethernet, power on, and complete the Ubuntu 22.04 first-boot wizard (create user, join network).

> ⚠️ **Firmware note:** Orin Nano dev kits manufactured before JetPack 6 need a one-time UEFI firmware update before a JetPack 6 SD card will boot. If the board doesn't boot the new card, follow the "firmware update" instructions on the JetPack page. NVIDIA **SDK Manager** (the full flashing tool) requires an x86-64 Ubuntu host and is *not* usable from your Mac — the SD-card route avoids it.

**Verify:** `jetson$ cat /etc/nv_tegra_release` shows an R36.x (JetPack 6) release, and `lsb_release -rs` prints `22.04`.

### 2.2.2 Install ROS 2 Humble (native)

```bash
jetson$ sudo apt update && sudo apt install -y software-properties-common curl
jetson$ sudo add-apt-repository universe -y
jetson$ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
jetson$ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
jetson$ sudo apt update
jetson$ sudo apt install -y ros-humble-ros-base ros-dev-tools
```

Then the BillieBot dependencies (same set as the Mac container, plus nothing extra):

```bash
jetson$ sudo apt install -y \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
  ros-humble-robot-localization ros-humble-rplidar-ros ros-humble-behaviortree-cpp \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher ros-humble-xacro \
  ros-humble-rmw-cyclonedds-cpp ros-humble-foxglove-bridge \
  python3-pip python3-colcon-common-extensions
jetson$ pip3 install pyserial numpy depthai pyyaml
```

**Verify:** `source /opt/ros/humble/setup.bash && ros2 pkg prefix rplidar_ros` prints `/opt/ros/humble`.

### 2.2.3 Device access (serial + OAK-D)

```bash
# Serial devices (Arduino + RPLidar):
jetson$ sudo usermod -aG dialout $USER

# OAK-D Lite udev rule (Movidius VID 03e7):
jetson$ echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' \
        | sudo tee /etc/udev/rules.d/80-movidius.rules
jetson$ sudo udevadm control --reload-rules && sudo udevadm trigger
```

Log out and back in for the group change. Plug in the Arduino, RPLidar, and OAK-D (OAK-D on a **USB 3** port with the supplied cable).

**Verify:**

```bash
jetson$ ls /dev/serial/by-id/
# expect: usb-1a86_USB_Serial-if00-port0   ← the Arduino (matches billiebot_base/config/base_driver.yaml)
jetson$ ls /dev/ttyUSB*
# the RPLidar is expected at /dev/ttyUSB1 by 01_lidar.launch.py — see the caveat below
jetson$ python3 -c "import depthai as dai; print(dai.Device.getAllAvailableDevices())"
# expect a non-empty list with the OAK-D
```

> ⚠️ **USB enumeration caveat:** `01_lidar.launch.py` hardcodes the lidar at `/dev/ttyUSB1` (115200 baud). The Arduino is addressed by its stable `/dev/serial/by-id/...` path, but plain `ttyUSB` numbering depends on plug-in order. If the lidar lands on `ttyUSB0`, either swap plug-in order or add a udev symlink rule — and note this as a candidate code fix (use a by-id path for the lidar too).

### 2.2.4 Clone, build, configure

```bash
jetson$ git clone https://github.com/sean-mackenzie/billie-bot-claude.git ~/billie-bot-claude
jetson$ cd ~/billie-bot-claude/billiebot_ws
jetson$ source /opt/ros/humble/setup.bash
jetson$ colcon build --symlink-install
jetson$ echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
jetson$ echo 'source ~/billie-bot-claude/billiebot_ws/install/setup.bash' >> ~/.bashrc
jetson$ echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
```

Install the **YOLOv8n blob** for the OAK-D ([Appendix B](#appendix-b--ml-model-assets)) and set its path in `billiebot_ws/src/billiebot_perception/config/perception.yaml` (`oakd_dog_detector → model_path`). The parameter ships empty; in real (non-mock) mode the node logs an error without it.

**Verify:** `ros2 launch billiebot_bringup jetson.launch.py mock:=true` starts Nav2, the OAK-D detector, dog locator, and mission nodes (rungs 06–08 + 13, which chain-include base/lidar/EKF/AMCL) with no crash loops. Then re-run without `mock:=true` once hardware is attached.

## 2.3 Raspberry Pi

Runs: thermal, NoIR, audio, cognition (`pi.launch.py` = rungs 09–12).

> The deployed board is a **Raspberry Pi 5** (design doc §5.1; naming previously drifted to "Pi 4" — closed as GAP-19). The steps below also work on a Pi 4 if you substitute one.

### 2.3.1 Flash the OS

**Recommended (per the system design doc §5.1): Raspberry Pi OS Lite 64-bit (Bookworm) + ROS 2 Humble in a Docker container.** Pi OS gives you first-class camera/I²S/I²C support; the container sidesteps the OS-version mismatch (Bookworm has no native Humble packages).

```bash
host$ brew install --cask raspberry-pi-imager
```

In Raspberry Pi Imager: choose *Raspberry Pi OS Lite (64-bit)*, your SD card, and use the ⚙️ pre-configuration to set hostname (`billiebot-pi`), enable SSH, and add your Wi-Fi credentials. Flash, boot, and SSH in.

### 2.3.2 Host OS configuration (I²C, audio, camera, Docker)

```bash
pi$ sudo apt update && sudo apt full-upgrade -y
pi$ sudo raspi-config nonint do_i2c 0          # enable I²C
pi$ sudo apt install -y i2c-tools alsa-utils
pi$ sudo usermod -aG i2c,audio,video,dialout $USER
```

**I²S speaker (MAX98357A):** edit `/boot/firmware/config.txt`:

```ini
dtparam=audio=off          # disable onboard audio so the DAC becomes card 0
dtoverlay=hifiberry-dac    # generic I²S DAC overlay, works for MAX98357A
```

Reboot. `speaker_node` plays through `plughw:0,0`, so the DAC must be **card 0**.

**Docker:**

```bash
pi$ curl -fsSL https://get.docker.com | sudo sh
pi$ sudo usermod -aG docker $USER
```

Log out/in.

**Verify (host OS, with sensors attached):**

```bash
pi$ i2cdetect -y 1            # expect "33" in the grid → MLX90640
pi$ arecord -l                # expect the ReSpeaker XVF3800 USB device
pi$ aplay -l                  # expect the I²S DAC as card 0
pi$ speaker-test -c 1 -t sine -f 440 -l 1   # hear a beep
pi$ rpicam-hello --list-cameras   # expect the NoIR camera module
pi$ docker run --rm hello-world
```

### 2.3.3 The Pi ROS container

Save as `~/billiebot-pi.Dockerfile` on the Pi:

```dockerfile
FROM ros:humble
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rmw-cyclonedds-cpp \
    python3-pip python3-colcon-common-extensions \
    alsa-utils libportaudio2 libusb-1.0-0 i2c-tools sqlite3 \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir \
    numpy sounddevice pyusb tflite-runtime \
    adafruit-blinka adafruit-circuitpython-mlx90640 \
    pyyaml jinja2 matplotlib fastapi uvicorn markdown
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
WORKDIR /ws
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc
CMD ["bash"]
```

Create the data directories on the **host** (bind-mounted so the SQLite DB and reports survive container rebuilds), clone, build, run:

```bash
pi$ sudo mkdir -p /var/lib/billiebot/snapshots /var/lib/billiebot/reports
pi$ sudo chown -R $USER /var/lib/billiebot
pi$ git clone https://github.com/sean-mackenzie/billie-bot-claude.git ~/billie-bot-claude
pi$ docker build -t billiebot-pi -f ~/billiebot-pi.Dockerfile .
pi$ docker run -it --name billiebot-pi \
      --privileged \
      --network host \
      -v /dev:/dev \
      -v /var/lib/billiebot:/var/lib/billiebot \
      -v "$HOME/billie-bot-claude/billiebot_ws:/ws" \
      billiebot-pi
container$ colcon build --symlink-install && source install/setup.bash
```

`--network host` (native on Linux, unlike macOS) lets CycloneDDS talk to the Jetson directly; `--privileged -v /dev:/dev` exposes I²C, USB audio, and the mic array.

Install the **YAMNet model** ([Appendix B](#appendix-b--ml-model-assets)) into `/var/lib/billiebot/models/` and set `model_path` in `billiebot_ws/src/billiebot_audio/config/audio.yaml`.

> ⚠️ **NoIR camera limitation:** `noir_cam_node` uses `picamera2`, which cannot be cleanly pip-installed inside an Ubuntu 22.04 container against Pi OS's libcamera stack. **Run the NoIR node in mock mode initially** (`pi.launch.py mock:=true` mocks it; or pass `mock:=true` only to rung 10). Real-camera options, in order of practicality: (a) build a custom container from Debian Bookworm with `python3-picamera2` + ROS from source (advanced), (b) switch the Pi to Ubuntu 22.04 Server + native Humble and fight picamera2/libcamera on Ubuntu. Treat this as a known integration task; the rest of the Pi stack (thermal, audio, cognition) runs real in the container.

**Verify:** `ros2 launch billiebot_bringup pi.launch.py mock:=true` inside the container starts thermal, NoIR, audio, and cognition nodes; `curl http://localhost:8080` from the Pi host returns the report page (uvicorn must be running — it starts with rung 12).

## 2.4 Arduino Nano (on-robot)

If you flashed in §1.9, just plug the Nano into the Jetson. Otherwise repeat §1.9 from any machine with the Arduino IDE.

**Verify (from the Jetson):**

```bash
jetson$ ls /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0   # exists → matches base_driver.yaml
jetson$ python3 - <<'EOF'
import serial, time
s = serial.Serial('/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0', 57600, timeout=2)
time.sleep(2)          # Nano resets on port open
s.write(b'b\r'); print(s.readline())   # expect b'57600'
EOF
```

## 2.5 Multi-machine bringup

1. Confirm §2.1 is done on both machines (IPs match `cyclonedds.xml`, `RMW_IMPLEMENTATION` exported, workspace rebuilt after any IP edit).
2. Start the Jetson side: `jetson$ ros2 launch billiebot_bringup jetson.launch.py`
3. Start the Pi side (inside its container): `container$ ros2 launch billiebot_bringup pi.launch.py`
4. **Verify cross-machine discovery** — on the Jetson:

   ```bash
   jetson$ ros2 topic list | grep -E "thermal|audio|billie/state"   # Pi topics visible on Jetson
   jetson$ ros2 topic echo /billie/state --once
   ```

5. **Watch from the Mac:** run the Foxglove bridge on the Jetson (`ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765`), then in Foxglove Studio on the Mac connect to `ws://<jetson-ip>:8765`. The daily report is at `http://<pi-ip>:8080`.

---

# Part 3 — Build, first run & verification

This section applies on any machine (Mac container, Jetson, Pi container). `docs/VERIFICATION.md` is the authoritative rung-by-rung checklist; this is the condensed flow.

**1. Build:**

```bash
cd <workspace>            # /ws in containers, ~/billie-bot-claude/billiebot_ws on the Jetson
colcon build --symlink-install
source install/setup.bash
```

**2. Climb the bringup ladder in mock mode.** Each rung chain-includes the ones below it, so you can jump straight to any rung. (Exception: rungs 04 SLAM and 05 AMCL are *alternatives* — each includes 01–03, but 05 does not include 04. Map with 04 first, then localize on the saved map with 05+.)

| Rung | Command (`ros2 launch billiebot_bringup …`) | Adds |
|---|---|---|
| 01 | `01_lidar.launch.py mock:=true` | RPLidar driver |
| 02 | `02_base.launch.py mock:=true` | base bridge + robot_state_publisher |
| 03 | `03_ekf.launch.py mock:=true` | robot_localization EKF |
| 04 | `04_slam.launch.py mock:=true` | slam_toolbox (mapping) |
| 05 | `05_amcl.launch.py mock:=true` | AMCL (saved-map localization) |
| 06 | `06_nav2.launch.py mock:=true` | full Nav2 stack |
| 07–10 | `07_oakd` / `08_dog_locator` / `09_thermal` / `10_noir` | perception |
| 11 | `11_audio.launch.py mock:=true` | audio classifier + speaker |
| 12 | `12_cognition.launch.py` | state fusion, logger, report server |
| 13 | `13_mission.launch.py mock:=true` | BT mission controller + action servers |
| 14 | `14_full_bringup.launch.py mock:=true` | everything |

**3. Run the acceptance suite** (22 tests; needs rung 14 running in another shell):

```bash
# Terminal 1:
ros2 launch billiebot_bringup 14_full_bringup.launch.py mock:=true
# Terminal 2:
cd <workspace> && source install/setup.bash
./src/billiebot_tests/scripts/run_all_mock_tests.sh
```

**Verify:** the script reports all TC-xx checks passing.

**4. Unit/lint tests:**

```bash
colcon test --packages-select billiebot_tests && colcon test-result --verbose
```

**5. Report server smoke test** (rung 12+ running):

```bash
curl -s http://localhost:8080 | head -5    # HTML from the FastAPI report server
```

**6. Before real-hardware driving:** work through `docs/MEASURE_ME.md` — encoder ticks/rev, wheel radius/separation, motor/encoder signs, and battery divider ratio all live in `billiebot_ws/src/billiebot_base/config/base_driver.yaml` and **must be measured on your build**, not trusted from defaults.

---

# Appendix A — Complete dependency reference

No `requirements.txt` or dependency lockfile exists in the repo; this appendix is the consolidated list (source: every `package.xml` + an import scan of all nodes).

### apt — ROS 2 packages

| Package | Needed on | Why |
|---|---|---|
| `ros-humble-ros-base` (or `ros:humble` image) | all | ROS 2 core |
| `ros-humble-navigation2`, `ros-humble-nav2-bringup` | Mac, Jetson | Nav2 (declared by `billiebot_navigation`) |
| `ros-humble-slam-toolbox` | Mac, Jetson | mapping |
| `ros-humble-robot-localization` | Mac, Jetson | EKF |
| `ros-humble-rplidar-ros` | Mac, Jetson | lidar driver — **used by `01_lidar.launch.py` but undeclared in any package.xml** |
| `ros-humble-behaviortree-cpp` | Mac, Jetson | BT.CPP **v4** (`billiebot_mission` C++ nodes include `behaviortree_cpp/...`) |
| `ros-humble-robot-state-publisher`, `ros-humble-joint-state-publisher`, `ros-humble-xacro` | Mac, Jetson | URDF publishing (`billiebot_description`) |
| `ros-humble-rmw-cyclonedds-cpp` | all | CycloneDDS RMW (multi-machine config assumes it) |
| `ros-humble-foxglove-bridge` | Mac, Jetson | visualization bridge |
| `ros-humble-rviz2`, `ros-humble-teleop-twist-keyboard` | Mac (optional) | operator tools |

### apt — system packages

| Package | Needed on | Why |
|---|---|---|
| `python3-pip`, `python3-colcon-common-extensions` | all | build tooling |
| `alsa-utils` | Pi (+Mac container for completeness) | `speaker_node` shells out to `aplay plughw:0,0` |
| `libportaudio2` | Mac, Pi | native library behind `sounddevice` |
| `libusb-1.0-0` | Mac, Pi | native library behind `pyusb` (ReSpeaker DoA) |
| `i2c-tools` | Pi | MLX90640 debugging (`i2cdetect`) |
| `sqlite3` (CLI) | Pi (optional) | inspect `/var/lib/billiebot/billie_events.db` |

### pip — Python packages

| Package | Needed on | Used by |
|---|---|---|
| `pyserial` | Mac, Jetson | `billiebot_base/base_bridge.py` (the only pip dep declared in a package.xml) |
| `numpy` | all | audio + perception |
| `sounddevice` | Mac, Pi | `audio_classifier` mic streaming |
| `tflite-runtime` (fallback: `tensorflow`) | Mac, Pi | YAMNet inference |
| `pyusb` | Mac, Pi | ReSpeaker DoA (VID 0x2886) |
| `depthai` | Mac, Jetson | OAK-D pipeline |
| `picamera2` | Pi only — see §2.3 caveat | NoIR camera |
| `adafruit-blinka`, `adafruit-circuitpython-mlx90640` | Pi | thermal sensor |
| `pyyaml`, `jinja2`, `matplotlib`, `fastapi`, `uvicorn`, `markdown` | Mac, Pi | cognition: logger, daily report, report server |

### Filesystem / OS configuration

| Item | Machine | Detail |
|---|---|---|
| `/var/lib/billiebot/{snapshots,reports}` + SQLite DB path | Pi (and Mac container) | from `billiebot_cognition/config/cognition.yaml`; create before rung 12 |
| `dialout` group | Jetson | Arduino + RPLidar serial |
| `i2c`, `audio`, `video` groups; I²C enabled; `dtparam=audio=off` + I²S DAC overlay | Pi | MLX90640, ReSpeaker, MAX98357A |
| udev rule VID `03e7` MODE 0666 | Jetson | OAK-D Lite |
| DHCP reservations + `cyclonedds.xml` peers + `RMW_IMPLEMENTATION` export | Jetson, Pi, (Mac) | §2.1 |

# Appendix B — ML model assets

Neither model ships in the repo; both `model_path` parameters default to `""`.

### YAMNet (audio classifier — Pi)

1. Download the **TFLite classification variant** of Google's YAMNet from Kaggle Models: <https://www.kaggle.com/models/google/yamnet> → *TFLite* → `classification-tflite` (free account required; the old `tfhub.dev/google/yamnet` links redirect here). Save/rename as `yamnet.tflite`.
2. Download the class map (the node loads `yamnet_class_map.csv` **from the same directory** as the model):

   ```bash
   curl -L -o yamnet_class_map.csv \
     https://raw.githubusercontent.com/tensorflow/models/master/research/audioset/yamnet/yamnet_class_map.csv
   ```

3. Place both in `/var/lib/billiebot/models/` on the Pi and set in `billiebot_ws/src/billiebot_audio/config/audio.yaml`:

   ```yaml
   audio_classifier:
     ros__parameters:
       model_path: /var/lib/billiebot/models/yamnet.tflite
   ```

### YOLOv8n blob (OAK-D — Jetson)

The detector creates a `YoloSpatialDetectionNetwork` with a **416×416** preview and filters **COCO class 16 (dog)** — so any COCO-trained YOLOv8n export works.

1. Go to Luxonis's converter at <https://tools.luxonis.com>, upload a stock `yolov8n.pt` (COCO weights from Ultralytics), set input shape **416**, target **RVC2** (OAK-D Lite), and download the resulting `.blob`.
2. Place it at e.g. `/home/<user>/billiebot/models/yolov8n_416.blob` on the Jetson and set in `billiebot_ws/src/billiebot_perception/config/perception.yaml`:

   ```yaml
   oakd_dog_detector:
     ros__parameters:
       model_path: /home/<user>/billiebot/models/yolov8n_416.blob
   ```

### Speaker sounds (optional)

`speaker_node` plays `.wav` files from the directory given by its `sounds_dir` parameter (`billiebot_audio/config/audio.yaml`, ships empty). Supply your own calm-voice clips; without them the node still runs and mock mode is unaffected.

# Appendix C — Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Docker build/run is very slow on the Mac; `docker image inspect` shows `amd64` | You based on an amd64-only image (e.g. `osrf/ros:humble-desktop`). Use `ros:humble` per §1.3 — it's arm64-native. |
| Machines can't see each other's topics | `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` not exported (launch files only set `CYCLONEDDS_URI`); or IPs in `cyclonedds.xml` don't match reality; or you edited the source XML but didn't rebuild (the launch files read the **installed** copy). |
| `ros2 topic list` differs between shells on one machine | One shell hasn't sourced `install/setup.bash`, or has a different `ROS_DOMAIN_ID`/`RMW_IMPLEMENTATION`. `ros2 daemon stop` after changing env vars. |
| `serial.SerialException: Permission denied` | User not in `dialout` (Jetson) — re-login after `usermod`. |
| Lidar fails to open port | Enumeration swap: lidar expected at `/dev/ttyUSB1` by `01_lidar.launch.py`. Check `ls -l /dev/serial/by-id/`, replug in the right order, or add a udev symlink. |
| `depthai` raises `X_LINK_DEVICE_NOT_FOUND` | Missing udev rule (§2.2.3), USB-2 cable/port, or insufficient power. Replug after `udevadm trigger`. |
| `aplay: audio open error: Device or resource busy` / wrong output | The I²S DAC must be **card 0** (`plughw:0,0` is hardcoded in `speaker_node`): set `dtparam=audio=off` and the DAC overlay, reboot, confirm with `aplay -l`. |
| YAMNet node logs "No model_path specified" | Expected until Appendix B is done; harmless in mock mode. |
| OAK-D node logs "No model_path specified" in real mode | Set the blob path (Appendix B). |
| `pip install` on the **Pi host OS** fails with `externally-managed-environment` | Pi OS Bookworm PEP-668. Prefer installing Python deps **inside the container**; on the host use `python3 -m venv` or `--break-system-packages`. |
| RViz over XQuartz shows a black/garbled window | Add `-e LIBGL_ALWAYS_SOFTWARE=1`, and enable "Allow connections from network clients" in XQuartz settings, then `xhost +localhost`. |
| Port 8080/8765 already in use on the Mac | Another container or app holds it: `docker ps` / change the `-p` mapping. |
| `colcon build` warns about `setup.py install` deprecation | Normal for `ament_python` on Humble; ignore. |

# Appendix D — Known discrepancies & decisions

| Item | Status |
|---|---|
| **Pi 4 vs Pi 5** | **Resolved (GAP-19, 2026-07-12):** the board is a **Raspberry Pi 5** (`BillieBot_System_Design.md` §5.1); all README/verification/config references are now aligned to Pi 5. Tracked in `docs/md/DISCREPANCY_RESOLUTION_PLAN.md`. |
| **Serial baud rate** | Firmware and `base_driver.yaml` use **57600**; the design doc's 115200 is superseded (already noted in the README's Design Decisions). |
| **`osrf/ros:humble-desktop` is amd64-only** | Checked against the Docker Hub API on 2026-07-05. This guide uses multi-arch `ros:humble` + apt desktop tools instead. |
| **Launch files set `CYCLONEDDS_URI` but not `RMW_IMPLEMENTATION`** | `jetson.launch.py` / `pi.launch.py` point at the CycloneDDS config, but selecting the CycloneDDS RMW is left to the environment — this guide exports it in `~/.bashrc`/Dockerfiles. Candidate code fix: add `SetEnvironmentVariable('RMW_IMPLEMENTATION', …)` to those launch files. |
| **`rplidar_ros` undeclared** | Used by `01_lidar.launch.py` but absent from every `package.xml`, so `rosdep install` won't pull it. Installed explicitly by this guide. Candidate fix: add `<exec_depend>rplidar_ros</exec_depend>` to `billiebot_bringup`. |
| **Lidar port hardcoded** | `/dev/ttyUSB1` @ 115200 in `01_lidar.launch.py`; fragile vs. USB enumeration. Candidate fix: use a `/dev/serial/by-id/...` path like the base bridge does. |
| **No IMU for MVP** | `use_imu: false` in `base_driver.yaml` until the A4/A5 encoder rewire (`docs/MEASURE_ME.md`); the BNO055 firmware extension in `firmware/README.md` is deferred accordingly. |
| **NoIR-in-container** | `picamera2` doesn't install cleanly in an Ubuntu 22.04 container on Pi OS; run rung 10 mocked until resolved (§2.3.3). |
