# BillieBot

An autonomous indoor robot built on ROS 2 that patrols an apartment, finds a miniature dachshund named Billie, classifies her behavioral state, logs events to SQLite, and generates daily activity reports.

## Overview

BillieBot is a differential-drive robot with multi-modal sensing (stereo vision, thermal, audio, NoIR) designed for observe-and-report pet monitoring. The MVP watches and logs; the architecture includes extension points for a future Behavior AI (contextual bandit / RL-lite) that can decide when to approach, retreat, speak, or dispense treats.

**Operational modes:** IDLE, PATROL, INVESTIGATE (audio-cued), TRACK/OBSERVE, RETURN, SAFE (low battery / fault)

## Hardware

| Component | Purpose |
|-----------|---------|
| Jetson Orin Nano | Primary compute (Nav2, OAK-D, mission) |
| Raspberry Pi 5 | Secondary compute (thermal, NoIR, audio, cognition) |
| Arduino Nano | Motor PID control, encoder reading, battery ADC |
| RPLidar A1 | 2D SLAM and navigation |
| OAK-D Lite | Spatial dog detection (YOLOv8n + stereo depth) |
| MLX90640 | 32x24 thermal imaging for low-light detection |
| Pi Camera 3 NoIR | Near-infrared imaging in darkness |
| ReSpeaker XVF3800 | 4-mic array for audio classification + DoA |
| L298N + DC motors | Differential drive with encoder feedback |

## Software Architecture

11 ROS 2 packages in `billiebot_ws/src/`:

| Package | Type | Description |
|---------|------|-------------|
| `billiebot_interfaces` | ament_cmake | 6 msgs, 3 srvs, 5 actions |
| `billiebot_description` | ament_cmake | URDF/xacro with 8 sensor frames |
| `billiebot_base` | ament_python | Diff-drive bridge with mock mode, battery, e-stop |
| `billiebot_navigation` | ament_cmake | EKF, SLAM toolbox, AMCL, Nav2 configs |
| `billiebot_perception` | ament_python | OAK-D detector, dog locator, thermal, NoIR |
| `billiebot_audio` | ament_python | YAMNet classifier with DoA, speaker server |
| `billiebot_cognition` | ament_python | State fusion, SQLite logger, daily report, web server |
| `billiebot_mission` | ament_cmake+py | BT nodes (C++), action servers, mission controller |
| `billiebot_bringup` | ament_cmake | 16 launch files, CycloneDDS multi-machine config |
| `billiebot_tests` | ament_python | Integration tests, mock test suite |
| `billiebot_sensor_tests` | ament_python | Per-sensor real-hardware bench tests (see [package README](billiebot_ws/src/billiebot_sensor_tests/README.md) and [bench test plan](docs/md/BILLIEBOT_SENSOR_BENCH_TEST_PLAN.md)) |

## Quick Start

```bash
# Build
cd billiebot_ws
colcon build --symlink-install
source install/setup.bash

# Run full stack in mock mode (no hardware needed)
ros2 launch billiebot_bringup 14_full_bringup.launch.py mock:=true \
    map:="$(ros2 pkg prefix billiebot_navigation)/share/billiebot_navigation/maps/my_apartment_v1.yaml"

# Run verification tests
./src/billiebot_tests/scripts/run_all_mock_tests.sh
```

`mock:=true` mocks the hardware, not the map — this launch runs `map_server` + AMCL, and the `map`
argument defaults to empty, so omitting it leaves Nav2 stuck waiting for `/map`. See
[Why Nav2 needs a map](docs/md/INSTALLATION_AND_SETUP.md#nav2-needs-a-map).

## Bringup Ladder

The system starts incrementally via numbered launch files:

| Rung | Launch File | What It Adds |
|------|------------|--------------|
| 01 | `01_lidar.launch.py` | RPLidar A1 driver |
| 02 | `02_base.launch.py` | Base bridge + robot_state_publisher |
| 03 | `03_ekf.launch.py` | EKF (robot_localization) |
| 04 | `04_slam.launch.py` | SLAM toolbox for mapping |
| 05 | `05_amcl.launch.py` | AMCL localization with saved map |
| 06 | `06_nav2.launch.py` | Full Nav2 stack |
| 07-10 | `07_oakd` / `09_thermal` / `10_noir` | Perception sensors |
| 11 | `11_audio.launch.py` | Audio classifier + speaker |
| 12 | `12_cognition.launch.py` | State fusion, logger, report server |
| 13 | `13_mission.launch.py` | Mission controller + action servers |
| 14 | `14_full_bringup.launch.py` | Everything |

All rungs support `mock:=true` for hardware-free testing. Rungs 05, 06, and 14 additionally
require `map:=<absolute path to a Nav2 map yaml>` — mock mode does not remove that need.

### Multi-Machine Deployment

```bash
# Jetson Orin Nano (map:= is required — it runs Nav2)
ros2 launch billiebot_bringup jetson.launch.py \
    map:="$(ros2 pkg prefix billiebot_navigation)/share/billiebot_navigation/maps/my_apartment_v1.yaml"

# Raspberry Pi 5
ros2 launch billiebot_bringup pi.launch.py
```

Update IPs in `billiebot_bringup/config/cyclonedds.xml`.

## Key Topics and Services

| Topic / Service | Type | Description |
|----------------|------|-------------|
| `/billie/state` | DogState | Fused behavioral state estimate |
| `/dog/detections_3d` | DogDetection3D | Spatial dog detections from OAK-D |
| `/audio/events` | AudioEvent | Classified audio with DoA |
| `/billiebot/mission_status` | MissionStatus | Current operational mode |
| `/e_stop` (srv) | EStop | Emergency motor stop |
| `/set_mode` (srv) | SetMode | Change operational mode |
| `http://localhost:8080` | HTTP | Daily report web interface |

## Documentation

- **[docs/VERIFICATION.md](docs/VERIFICATION.md)** — Full bringup ladder with expected outputs and 22 acceptance tests
- **[docs/MEASURE_ME.md](docs/MEASURE_ME.md)** — Physical values to measure on the actual robot (sensor mounts, encoder calibration, battery divider)
- **[docs/md/BILLIEBOT_SENSOR_BENCH_TEST_PLAN.md](docs/md/BILLIEBOT_SENSOR_BENCH_TEST_PLAN.md)** — Per-sensor real-hardware bench-test plan (OAK-D, MLX90640, NoIR, XVF3800), implemented by `billiebot_sensor_tests`
- **[firmware/README.md](firmware/README.md)** — Arduino watchdog timer change (2000ms to 500ms)

## Reference Code

The `reference_my_bot/` directory contains the original differential drive code that `billiebot_base` is adapted from. It is kept for reference and not built as part of the workspace.

## Design Decisions

- **No IMU for MVP:** The BNO055 requires I2C pins currently used by the right encoder. Run with `use_imu:=false` until hardware rewire (documented in MEASURE_ME.md).
- **No ros2_control:** The base bridge directly wraps the reference `diff_drive_base.py` serial protocol.
- **ObserveOnlyPolicy:** The behavior tree's `PolicyDecision` node always returns OBSERVE in the MVP. Future policies plug in via the same interface.
- **Baud rate 57600:** Matches the working firmware (design doc said 115200 but firmware uses 57600).

## License

MIT
