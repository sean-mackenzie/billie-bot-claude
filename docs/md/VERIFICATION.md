# BillieBot — Verification & Bringup Guide

## Build

```bash
cd billiebot_ws
colcon build --symlink-install
source install/setup.bash
```

## Bringup Ladder

Each rung builds on the previous. Use `mock:=true` for hardware-free testing.

**Rungs 05, 06, 14 and `jetson.launch.py` need a map** — they start `map_server` + AMCL, and `mock:=true` does not change that. The `map` argument defaults to empty and fails quietly if you omit it; see [INSTALLATION_AND_SETUP.md §1.6 "Why Nav2 needs a map"](INSTALLATION_AND_SETUP.md#nav2-needs-a-map). Set it once per shell and reuse it below:

```bash
export MAP="$(ros2 pkg prefix billiebot_navigation)/share/billiebot_navigation/maps/my_apartment_v1.yaml"
test -f "$MAP" && echo "[PASS] map YAML exists" || echo "[FAIL] map YAML missing"
```

**On hardware (skip for `mock:=true`):** run the device preflight first — `ros2 run billiebot_bringup check_devices.sh` should report `[PASS]` for the RPLidar, the Arduino, `dialout` membership, and the OAK-D. Jetson device setup is in `INSTALLATION_AND_SETUP.md` §2.2.4.

### Rung 01: Lidar
```bash
ros2 launch billiebot_bringup 01_lidar.launch.py mock:=true
```
**Verify:** `/scan` topic is publishing (`ros2 topic echo /scan --once`)
- `ros2 run billiebot_bringup verify_rung_01.sh`

#### Rung 01 on real hardware (Jetson)

Do the device setup in `INSTALLATION_AND_SETUP.md` §2.2.4 first. Drop `mock:=true` to start the real driver:

```bash
jetson$ ros2 launch billiebot_bringup 01_lidar.launch.py
```

This **blocks** — leave it running and open a **second SSH session** for the checks:

```bash
jetson$ ros2 param get /rplidar_node serial_port
# expect the /dev/serial/by-id/usb-Silicon_Labs_CP2102_... path from
# billiebot_bringup/config/lidar.yaml — NOT /dev/ttyUSB0 or /dev/ttyUSB1
jetson$ ros2 run billiebot_bringup verify_rung_01.sh
# expect [PASS] x2
```

Expect roughly **5–8 Hz** on `/scan`, not the mock's 10 Hz — the A1 spins slower in `Standard` mode. A sagging or jittery rate usually means USB power or serial contention.

> If `rplidar_node` dies with `*** buffer overflow detected ***`, the port is simply missing — that is upstream `rplidar_ros` 2.0.0 failing unhelpfully, not a clue about the cause. Run `check_devices.sh`.

#### Plug-order regression test (GAP-20)

The point of addressing devices by `/dev/serial/by-id/` is that USB enumeration order stops mattering. Prove it after any hardware change:

```bash
# Ctrl-C the launch above, then physically unplug the RPLidar and the Arduino
# and plug them back in the OPPOSITE order.
jetson$ ls -l /dev/serial/by-id/
# the two symlinks now point at swapped ../../ttyUSBn targets — that is expected
jetson$ ros2 run billiebot_bringup check_devices.sh     # still all [PASS]
jetson$ ros2 launch billiebot_bringup 01_lidar.launch.py   # still comes up
jetson$ ros2 launch billiebot_bringup 02_base.launch.py    # still comes up
```

Both rungs must work regardless of the order the cables went in. Repeat once across a full reboot.

### Rung 02: Base + Description
```bash
ros2 launch billiebot_bringup 02_base.launch.py mock:=true
```
**Verify:**
- `/odom` publishing Odometry
- `/joint_states` publishing JointState
- `/battery_state` publishing BatteryState
- `/e_stop` service available
- TF tree: `odom → base_link → chassis → (sensor frames)`
- `ros2 run billiebot_bringup verify_rung_02.sh`

### Rung 03: EKF
```bash
ros2 launch billiebot_bringup 03_ekf.launch.py mock:=true
```
**Verify:** `/odometry/filtered` publishing from `robot_localization`

### Rung 04: SLAM
```bash
ros2 launch billiebot_bringup 04_slam.launch.py mock:=true
```
**Verify:** `/map` topic publishing OccupancyGrid (requires real lidar data)

### Rung 05: AMCL Localization
```bash
ros2 launch billiebot_bringup 05_amcl.launch.py mock:=true map:="$MAP"
```
**Verify:** AMCL particle cloud on `/particle_cloud`, map→odom TF, and `lifecycle_manager_localization: Managed nodes are active`

(Drop `mock:=true` on the robot. Keep `map:=` either way.)

### Rung 06: Nav2
```bash
ros2 launch billiebot_bringup 06_nav2.launch.py mock:=true map:="$MAP"
```
**Verify:**
- `navigate_to_pose` action available
- Costmap topics publishing
- `lifecycle_manager_navigation: Managed nodes are active`
- `ros2 run billiebot_bringup verify_rung_06.sh`

### Rung 07: OAK-D Dog Detector
```bash
ros2 launch billiebot_bringup 07_oakd.launch.py mock:=true
```
**Verify:** `/dog/detections_3d` and `/dog/found` topics publishing

### Rung 08: Dog Locator
```bash
ros2 launch billiebot_bringup 08_dog_locator.launch.py
```
**Verify:** `/dog/pose_map` publishing when detections + TF available

### Rung 09: Thermal Camera
```bash
ros2 launch billiebot_bringup 09_thermal.launch.py mock:=true
```
**Verify:** `/thermal/image` (32x24 32FC1) and `/thermal/blob` publishing

### Rung 10: NoIR Camera
```bash
ros2 launch billiebot_bringup 10_noir.launch.py mock:=true
```
**Verify:** `/noir/image` publishing

### Rung 11: Audio
```bash
ros2 launch billiebot_bringup 11_audio.launch.py mock:=true
```
**Verify:** `/audio/events` publishing AudioEvent messages

### Rung 12: Cognition
```bash
ros2 launch billiebot_bringup 12_cognition.launch.py
```
**Verify:**
- `/billie/state` publishing DogState
- `/get_dog_state` service responds
- `http://localhost:8080/health` returns OK
- `ros2 run billiebot_bringup verify_rung_12.sh`

### Rung 13: Mission
```bash
ros2 launch billiebot_bringup 13_mission.launch.py mock:=true
```
**Verify:**
- `/billiebot/mission_status` publishing
- `/set_mode` service responds
- Action servers: `/approach_dog`, `/retreat`, `/dispense_treat`

### Rung 14: Full Bringup
```bash
ros2 launch billiebot_bringup 14_full_bringup.launch.py mock:=true map:="$MAP"
```
**Verify:** both `lifecycle_manager_localization` and `lifecycle_manager_navigation` log `Managed nodes are active`. Transform and dropped-message warnings during the first few seconds are normal while the mock TF chain fills in; they should stop.

## Multi-Machine Setup

**Jetson Orin Nano:**
```bash
ros2 launch billiebot_bringup jetson.launch.py map:="$MAP"
```

**Raspberry Pi 5:**
```bash
ros2 launch billiebot_bringup pi.launch.py
```

Update IPs in `billiebot_bringup/config/cyclonedds.xml` first. Add `mock:=true` on the Jetson side to bring the stack up without hardware — the `map:=` argument is still required, since mocking the hardware does not remove Nav2's need for a map.

**Verify:** `Managed nodes are active` from `lifecycle_manager_navigation` on the Jetson. If instead you see `yaml-filename parameter is empty`, `Waiting for map....`, or `Timed out waiting for transform from base_link to map` repeating, the map did not load — see [INSTALLATION_AND_SETUP.md §2.2.3](INSTALLATION_AND_SETUP.md#223-clone-build-configure) for the checklist.

## Mock Test Suite

Run all tests without hardware:
```bash
# Terminal 1: Launch full mock stack
ros2 launch billiebot_bringup 14_full_bringup.launch.py mock:=true map:="$MAP"

# Terminal 2: Run tests
./billiebot_ws/src/billiebot_tests/scripts/run_all_mock_tests.sh
```

## Acceptance Test IDs

| TC | Description | Traces To |
|----|------------|-----------|
| TC-01 | Interface definitions build | Phase 1A |
| TC-02 | URDF validity | SYS-NAV-1 |
| TC-03 | Odometry publishing | SYS-NAV-2 |
| TC-04 | Joint states | Phase 1C |
| TC-05 | Battery monitoring | SYS-PLT-2 |
| TC-06 | E-stop service | SYS-PLT-5 |
| TC-07 | Dog 3D detection | SYS-PER-1, SYS-PER-2 |
| TC-08 | Dog locator TF | SYS-PER-2 |
| TC-09 | Thermal imaging | SYS-PER-3 |
| TC-10 | Thermal blob detection | SYS-PER-3 |
| TC-11 | Audio classification | SYS-PER-4 |
| TC-12 | State fusion | SYS-STL-1 |
| TC-13 | Mission status | Phase 6 |
| TC-14 | GetDogState service | SYS-STL-1 |
| TC-15 | SetMode service | Phase 6 |
| TC-16 | Waypoint navigation | SYS-NAV-3, SYS-NAV-6 |
| TC-17 | Standoff distance | SYS-FND-3 |
| TC-18 | Speed limiting | SYS-NAV-5 |
| TC-19 | Stuck recovery | SYS-NAV-4 |
| TC-20 | SQLite logging | SYS-STL-2, SYS-STL-4 |
| TC-21 | Daily report | SYS-RPT-1 |
| TC-22 | Report server | SYS-RPT-2 |
