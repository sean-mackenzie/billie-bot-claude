# BillieBot Bringup Ladder — Systems Decomposition & Analysis

**Author role:** Senior Systems Engineer decomposition
**Sources:** Static analysis of `docs/VERIFICATION.md`, all launch files, node sources, configuration files, interface definitions, and `BillieBot_System_Design.md` (source document of `BillieBot_System_Design.md.pdf`). No code was executed or modified to produce this report.
**Date:** 2026-07-03

---

## 1. Purpose & Method

This report decomposes each of the 14 rungs of the Bringup Ladder defined in `docs/VERIFICATION.md`. For each rung it answers:

1. **Referenced files** — launch files (including transitive includes), configs, node sources, verify scripts, and external package dependencies.
2. **Key input parameters** — the parameters that materially determine behavior, with values and rationale.
3. **Runtime process architecture** — which nodes run, what each publishes/subscribes/serves, message types, publish rates, TF contributions, and mock-vs-real differences.
4. **Measurable outputs** — the concrete data and physical observables that verify correct function, with the exact commands to measure them.
5. **Requirements traceability** — which `SYS-*` requirements from the System Design Document the rung satisfies (fully, partially, or could with additions), cross-referenced to the TC-01…TC-22 acceptance-test IDs.

Method: every claim below is traced to a file in this repository. Facts that come from hardware datasheets or upstream package behavior rather than this repo (e.g., RPLidar A1 scan rate, Nav2 default action names) are marked *(inference)*.

### 1.1 System context

BillieBot is a differential-drive indoor robot whose MVP mission is **observe-and-report**: patrol an apartment, find a miniature dachshund (Billie), hold a ≥1.0 m standoff, fuse visual/thermal/audio evidence into a behavioral state, log state transitions to SQLite, and serve a daily HTTP report. The software is a ROS 2 (Humble-targeted) stack split across three processors:

| Host | Role | Nodes (per design §5.2 and the `jetson.launch.py`/`pi.launch.py` split) |
|---|---|---|
| **Jetson Orin Nano** | Real-time autonomy | `rplidar_node`, `base_bridge`, `ekf_filter_node`, `slam_toolbox`/`amcl`, Nav2 stack, `oakd_dog_detector`, `dog_locator`, mission nodes |
| **Raspberry Pi 5** | Sensing & cognition | `thermal_node`, `noir_cam_node`, `audio_classifier`, `speaker_node`, `state_fusion`, `dog_logger`, `daily_report`, `report_server` |
| **Arduino Nano** | Hard real-time I/O | Encoder counting, PID @30 Hz, L298N PWM, battery ADC, 500 ms serial-heartbeat motor cutoff (`firmware/README.md`) |

Multi-machine DDS uses CycloneDDS with multicast disabled and static unicast peers (`billiebot_bringup/config/cyclonedds.xml`: Jetson `192.168.42.100`, Pi `192.168.42.101`). The `jetson.launch.py` and `pi.launch.py` files set `CYCLONEDDS_URI` to this config and launch the host-appropriate subset of rungs — they are deployment groupings, not rungs themselves.

Every rung (except 08 and 12) accepts a `mock:=true` launch argument that substitutes hardware drivers with synthetic publishers so the entire ladder can be exercised on a development machine.

### 1.2 Launch inclusion graph

Rungs compose **transitively** — launching rung N brings up (most of) the rungs below it. Several integration defects identified in Appendix B arise directly from double-inclusion in this graph.

```
14_full_bringup ─┬─► 06_nav2 ─┬─► 05_amcl ─┬─► 01_lidar                (rplidar OR mock stub)
                 │            │            ├─► 03_ekf ──► 02_base ─┬─► description.launch.py (robot_state_publisher)
                 │            │            │        │              └─► base.launch.py (base_bridge)
                 │            │            │        └─► ekf_node (robot_localization)
                 │            │            └─► localization.launch.py (map_server + amcl + lifecycle_mgr)
                 │            └─► navigation.launch.py (controller + planner
                 │                                       + behaviors + bt_navigator + lifecycle_mgr)
                 ├─► 07_oakd            (oakd_dog_detector)
                 ├─► 08_dog_locator     (dog_locator)
                 ├─► 09_thermal         (thermal_node)
                 ├─► 10_noir            (noir_cam_node)
                 ├─► 11_audio ──► audio.launch.py (audio_classifier + speaker_node)
                 ├─► 12_cognition ──► cognition.launch.py (state_fusion + dog_logger
                 │                                          + daily_report + report_server)
                 └─► 13_mission ──► mission.launch.py (mission_controller + approach_dog_server
                                                        + retreat_server + speak_server
                                                        + dispense_treat_server)

04_slam ─┬─► 01_lidar
         ├─► 03_ekf ──► 02_base ──► (as above)
         └─► slam.launch.py (async_slam_toolbox_node)      [04 is NOT included by 05/06/14 —
                                                             mapping and localization are alternatives]
```

Note the standing duplication visible in this graph: in mock mode, `01_lidar` launches a **second `base_bridge` instance** (named `mock_lidar_stub`), and rungs 04/05/06/14 include both `01` and the `03→02` chain. (A second duplication — rung 06 starting two `ekf_filter_node`s via `navigation.launch.py` — was resolved 2026-07-17, GAP-6.) See Appendix B.

### 1.3 Per-rung template

Each rung section below follows the five-question structure: **(1) Referenced files → (2) Key parameters → (3) Runtime architecture → (4) Measurable outputs → (5) Requirements traceability.** All paths are relative to `billiebot_ws/src/` unless otherwise noted.

---

## 2. Rung 01 — Lidar

```bash
ros2 launch billiebot_bringup 01_lidar.launch.py mock:=true
```

### 2.1 Referenced files

| File | Role |
|---|---|
| `billiebot_bringup/launch/01_lidar.launch.py` | Rung entry point; selects real vs. mock via `IfCondition`/`UnlessCondition` |
| `billiebot_base/billiebot_base/base_bridge.py` | **Mock branch only** — launched as node `mock_lidar_stub` (placeholder; see §2.3) |
| `billiebot_bringup/scripts/verify_rung_01.sh` | Verification script (topic existence + `ros2 topic hz`) |
| External: `rplidar_ros` package (`rplidar_node`) | Real lidar driver *(external dependency)* |

### 2.2 Key input parameters

| Parameter | Value | Significance |
|---|---|---|
| `mock` (launch arg) | default `false` | Selects driver vs. placeholder stub |
| `serial_port` | `/dev/ttyUSB1` | RPLidar A1 USB-UART device on the Jetson. Note `base_bridge` uses a `/dev/serial/by-id/...` stable path while the lidar uses a raw `ttyUSB1` index — enumeration-order sensitive (Appendix B-9) |
| `serial_baudrate` | `115200` | A1 standard baud |
| `frame_id` | `laser_frame` | Must match the link created by `billiebot_description/urdf/lidar.xacro` (it does) |
| `angle_compensate` | `true` | Interpolates to uniform angular spacing for SLAM |
| `scan_mode` | `Standard` | A1 standard mode: ~8 kHz sample rate, ~5.5 Hz rotation *(datasheet inference)* |

### 2.3 Runtime process architecture

**Real mode** — one node:

| Node | Pub | Type | Rate |
|---|---|---|---|
| `rplidar_node` | `/scan` | `sensor_msgs/LaserScan` | ~5.5–8 Hz *(A1 hardware inference)* |

No TF is produced by this rung. `/scan` messages carry `frame_id: laser_frame`, which is **unresolvable** until rung 02 brings up `robot_state_publisher` (the `chassis → laser_frame` static transform lives in the URDF). Standalone RViz viewing requires setting the fixed frame to `laser_frame`.

**Mock mode** — one node, and this is the ladder's most significant defect:

The mock branch launches `billiebot_base`'s `base_bridge` executable under the name `mock_lidar_stub` with `mock: true`. The launch file's own comment concedes: *"In practice, a dedicated mock scan publisher would be better. For now, this is a placeholder."* `base_bridge` contains **no `/scan` publisher whatsoever**. In mock mode this rung actually produces:

| Node | Pub/Srv | Type | Rate |
|---|---|---|---|
| `mock_lidar_stub` (a `base_bridge` instance) | `/odom` | `nav_msgs/Odometry` | 30 Hz |
| | `/joint_states` | `sensor_msgs/JointState` | 30 Hz |
| | `/battery_state` | `sensor_msgs/BatteryState` | 1 Hz (constant ≈12.58 V from simulated ADC) |
| | TF `odom → base_link` | `tf2_msgs/TFMessage` | 30 Hz |
| | `/e_stop` | `billiebot_interfaces/srv/EStop` | service |
| | `/cmd_vel` (sub) | `geometry_msgs/Twist` | — |

Consequences: (a) the rung's own verify criterion (`/scan` publishing) **cannot pass in mock mode**; (b) every higher rung that includes 01 in mock mode gains a duplicate base-bridge (Appendix B-1, B-2).

### 2.4 Measurable outputs

| Output | Measurement | Expected (real) | Expected (mock) |
|---|---|---|---|
| `/scan` exists & publishes | `ros2 topic echo /scan --once`; `ros2 topic hz /scan` | LaserScan at ~5.5–8 Hz, 360° ranges 0.15–12 m | **Absent** (defect) |
| Scan geometry sane | RViz LaserScan display, fixed frame `laser_frame` | Room outline visible; motor audibly spinning (physical) | n/a |
| Script result | `./scripts/verify_rung_01.sh` | `[PASS]` ×2, exit 0 | `[FAIL] /scan not found`, exit 1 |

The verify script checks topic existence via `ros2 topic info` and then rate via `timeout 5 ros2 topic hz` — it validates *liveness*, not scan content or field ranges.

### 2.5 Requirements traceability

| Requirement | Relationship |
|---|---|
| **SYS-NAV-1** (lidar SLAM map) | **Prerequisite.** `/scan` is the sole input to `slam_toolbox` (rung 04) and AMCL (rung 05). Rung 01 alone doesn't satisfy it, but SYS-NAV-1 is unreachable without it. |
| **SYS-NAV-2, SYS-NAV-3** | Prerequisite — `/scan` drives AMCL localization and both Nav2 costmap obstacle layers (`nav2_params.yaml` `observation_sources: scan`). |
| Design §3.4 BDD-03 | The lidar is modeled under the Perception Subsystem with data allocated to Nav — rung 01 verifies the physical `lidar → jet` USB/UART item flow of IBD-01. |
| TC coverage | No dedicated TC; TC-02…TC-06 begin at rung 02. **Could** satisfy a lidar-liveness test if the mock stub were replaced with a real synthetic `/scan` publisher — recommended (Appendix B-1). |

---

## 3. Rung 02 — Base + Description

```bash
ros2 launch billiebot_bringup 02_base.launch.py mock:=true
```

### 3.1 Referenced files

| File | Role |
|---|---|
| `billiebot_bringup/launch/02_base.launch.py` | Rung entry; includes the two package launches below |
| `billiebot_description/launch/description.launch.py` | Runs `xacro` on the URDF and starts `robot_state_publisher` |
| `billiebot_description/urdf/billiebot.urdf.xacro` | Top-level robot model; includes `robot_core.xacro`, `lidar.xacro`, `oakd_lite.xacro`, `noir_camera.xacro`, `thermal.xacro`, `mic_array.xacro`, `imu.xacro`, `inertial_macros.xacro` |
| `billiebot_base/launch/base.launch.py` | Starts `base_bridge` with the config below |
| `billiebot_base/config/base_driver.yaml` | All drive/serial/battery parameters |
| `billiebot_base/billiebot_base/base_bridge.py` | Differential-drive bridge node (480 lines; contains `MockSerial`) |
| `billiebot_interfaces/srv/EStop.srv` | E-stop service definition |
| `billiebot_bringup/scripts/verify_rung_02.sh` | Verification script |
| `docs/MEASURE_ME.md` | Documents which of these parameters are placeholders pending physical measurement |
| Related (not launched): `reference_my_bot/diff-drive-motor-controller/` and `firmware/README.md` | The Arduino-side serial protocol implementation and the 500 ms watchdog change |

### 3.2 Key input parameters

From `base_driver.yaml` (calibration status per `docs/MEASURE_ME.md`):

| Parameter | Value | Significance |
|---|---|---|
| `port` / `baudrate` | `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` @ 57600 | Stable-path Arduino serial link (CH340 adapter) |
| `wheel_radius` | 0.034 m | Odometry scale — **to be measured under load** |
| `wheel_separation` | 0.298 m | Yaw-rate scale — **to be measured** |
| `encoder_ticks_per_rev` | 2000.0 | Tick→radian conversion; reference project offers 1974.7/1779.0 alternatives — **calibrate by driving 1 m** |
| `pid_rate_hz` | 30.0 | Must match Arduino PID loop rate; used in the counts-per-loop conversion |
| `cmd_timeout_sec` | 0.5 | Software deadman: zero motor command if no `/cmd_vel` for 0.5 s (complements the Arduino 500 ms heartbeat cutoff) |
| `publish_rate_hz` | 30.0 | Rate of the odom/joint/TF pipeline |
| `publish_tf` | `false` | EKF owns `odom → base_link` from rung 03 up (GAP-5 resolved 2026-07-17); rung-02-only bench work can restore the base broadcast with the `publish_tf:=true` launch arg (Appendix B-3) |
| `use_imu` | `false` | BNO055 disabled until the A4/A5 encoder rewire (`docs/MEASURE_ME.md`) |
| `battery_divider_ratio` / `battery_pin` | 6.0 / A0 | ADC→voltage: `V = adc·5/1023·6.0` |
| `battery_low_voltage` / `battery_critical_voltage` | 10.5 V / 9.9 V | 3.5 and 3.3 V/cell thresholds (SYS-PLT-2 boundary values) |
| `left/right_motor_sign`, `*_encoder_sign` | 1.0 | Polarity flips set during physical commissioning |
| Description args | `use_lidar/oakd/noir/thermal/mic:=true`, `use_imu:=false` | Gate which sensor links appear in the URDF/TF tree |

### 3.3 Runtime process architecture

Two nodes:

**`robot_state_publisher`** — runs `xacro billiebot.urdf.xacro` at launch time via a `Command` substitution:

| Interface | Direction | Type | Rate |
|---|---|---|---|
| `/robot_description` | pub (latched/transient-local) | `std_msgs/String` | once |
| `/tf_static` | pub | `tf2_msgs/TFMessage` | once — all fixed joints: `base_link→chassis`, `chassis→laser_frame`, `chassis→oakd_link→oakd_link_optical`, `chassis→noir_link→noir_link_optical`, `chassis→thermal_link→thermal_link_optical`, `chassis→mic_array` (names from the xacro files; positions are `TODO(measure)` placeholders) |
| `/joint_states` | sub | `sensor_msgs/JointState` | consumes base_bridge output to publish wheel-joint TF on `/tf` at 30 Hz |

**`base_bridge`** — the drive interface. Internal loop (`update()` on a 30 Hz timer): apply e-stop/timeout gating → send `m <left_cpl> <right_cpl>` motor command over serial → read encoders with `e` → integrate differential-drive odometry (exact midpoint-arc model) → publish. Conversion: `counts_per_loop = rad_s/(2π) · 2000 / 30`.

| Interface | Direction | Type | Rate / trigger |
|---|---|---|---|
| `/cmd_vel` | sub | `geometry_msgs/Twist` | inverse kinematics: `v ± ω·d/2 / r` → wheel targets; ignored while e-stopped |
| `/odom` | pub | `nav_msgs/Odometry` (`odom`→`base_link`, pose + twist) | 30 Hz |
| `/joint_states` | pub | `sensor_msgs/JointState` (`left_wheel_joint`, `right_wheel_joint` pos+vel) | 30 Hz |
| `/battery_state` | pub | `sensor_msgs/BatteryState` (voltage, per-cell array, health enum vs. 10.5/9.9 V thresholds) | 1 Hz |
| TF `odom → base_link` | broadcast | `TransformStamped` | 30 Hz (gated by `publish_tf`) |
| `/e_stop` | service server | `billiebot_interfaces/srv/EStop` (`engage: bool → success, message`) | engage: zeroes targets, sends `m 0 0`, blocks `/cmd_vel`; release: re-enables |

**Serial protocol** (Jetson ↔ Arduino, 57600 baud, `\r`-terminated — mirrors `reference_my_bot` ROSArduinoBridge):

| Cmd | Meaning | Response |
|---|---|---|
| `m L R` | Target wheel speeds in encoder counts/PID-loop | `OK` |
| `e` | Read cumulative encoder ticks | `<left> <right>` |
| `r` | Reset encoders | `OK` |
| `a <pin>` | Read ADC (battery divider) | `0–1023` |

**Mock vs. real:** `MockSerial` replaces `pyserial` in-process. It integrates commanded counts-per-loop into fake encoder ticks (perfect compliance — commanded velocity is instantly achieved) and answers `a` with a constant ADC of 429 → **12.58 V** battery forever. Consequence: battery-triggered SAFE-mode logic (rung 13) can never fire in mock mode.

TF tree after this rung (matches VERIFICATION.md): `odom → base_link → chassis → {laser_frame, oakd_link[_optical], noir_link[_optical], thermal_link[_optical], mic_array}` plus wheel joints.

### 3.4 Measurable outputs

| Output | Measurement | Expected |
|---|---|---|
| Odometry | `ros2 topic hz /odom` → 30 Hz; `ros2 topic echo /odom` | Pose integrates when commanded; twist echoes commanded velocity (mock: exactly) |
| Joint states | `ros2 topic hz /joint_states` → 30 Hz | Wheel positions monotonically increase while driving |
| Battery | `ros2 topic echo /battery_state` | Mock: 12.58 V, health `GOOD`; real: pack voltage within 9.9–12.6 V |
| TF | `ros2 run tf2_tools view_frames` | Single connected tree `odom→base_link→chassis→sensors` |
| Robot description | `ros2 topic echo /robot_description --once` | Valid URDF XML (TC-02) |
| E-stop | `ros2 service call /e_stop billiebot_interfaces/srv/EStop "{engage: true}"` | `success: true`; subsequent `/cmd_vel` produces zero motion / zero odom delta |
| **Physical (real)** | `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"` | Robot drives forward ~0.1 m/s (wrong direction ⇒ flip `*_sign` params per MEASURE_ME); odom `x` grows ≈ tape-measure distance |
| Closed-loop odometry sanity | Drive 1.0 m, compare `/odom` | Error budget informs `encoder_ticks_per_rev` calibration |
| Script | `./scripts/verify_rung_02.sh` | 6/6 PASS (4 topics + `/tf` + `/e_stop`), exit 0 |

### 3.5 Requirements traceability

| Requirement / TC | Relationship |
|---|---|
| **TC-02** (URDF validity → SYS-NAV-1) | **Satisfied** — `/robot_description` published from the xacro chain |
| **TC-03** (Odometry → SYS-NAV-2) | **Satisfied (mechanism)** — `/odom` at 30 Hz. The ≤0.15 m accuracy figure of SYS-NAV-2 belongs to rung 05 + a physical test |
| **TC-04** (Joint states) | **Satisfied** |
| **TC-05** (Battery monitoring → SYS-PLT-2) | **Partial** — voltage measurement, per-cell reporting, and threshold-based health enums exist. The *reaction* (enter SAFE, stop, alert) lives in rung 13's mission controller; "request pickup"/alert is unimplemented |
| **TC-06** (E-stop service → SYS-PLT-5) | **Partial** — the software e-stop path exists and stops motor commands. The **200 ms PWM-cut latency** and the **Arduino-side 500 ms heartbeat watchdog** are firmware properties (`firmware/README.md`, `AUTO_STOP_INTERVAL 500`) verifiable only on hardware with a timing rig — not by this rung's script |
| **SYS-PLT-4** (teleop) | **Enabler** — `/cmd_vel` + `/e_stop` are exactly the teleop surface; no teleop node is launched by any rung |
| **SYS-EXT-4 note** | `base_bridge` odometry feeds the EKF that the future engagement-outcome measurement ("dog approaches robot") will rely on |

---

## 4. Rung 03 — EKF

```bash
ros2 launch billiebot_bringup 03_ekf.launch.py mock:=true
```

### 4.1 Referenced files

| File | Role |
|---|---|
| `billiebot_bringup/launch/03_ekf.launch.py` | Rung entry; includes **rung 02 in its entirety**, then adds the EKF |
| `billiebot_navigation/config/ekf.yaml` | `robot_localization` configuration |
| All rung-02 files | Transitively included |
| External: `robot_localization` (`ekf_node`) | The filter implementation *(external dependency)* |

Note the rung's docstring says "EKF on top of base **+ lidar**", but the launch file includes only `02_base.launch.py` — no lidar. Correct behavior (the EKF doesn't consume `/scan`), stale comment.

### 4.2 Key input parameters

From `ekf.yaml`:

| Parameter | Value | Significance |
|---|---|---|
| `frequency` | 30.0 | Filter output rate, matched to base_bridge's 30 Hz input |
| `two_d_mode` | `true` | Planar robot: z/roll/pitch forced to zero |
| `publish_tf` | `true` | EKF broadcasts `odom → base_link` — sole owner since GAP-5 resolution (base_bridge default is now `false`; Appendix B-3) |
| `odom_frame`/`base_link_frame`/`world_frame` | `odom`/`base_link`/`odom` | Odom-frame filter (map frame left to SLAM/AMCL) |
| `odom0` | `/odom` | Sole active sensor input |
| `odom0_config` | fuses x, y, yaw, vx, vy, vyaw | Absolute pose + planar velocities from wheel odometry |
| `imu0` (commented out) | `/imu/data` | Entire IMU block disabled pending the BNO055 A4/A5 rewire — with a single sensor the "fusion" is effectively a smoother, not a multi-sensor EKF |
| `process_noise_covariance` | 15×15 diag | Standard robot_localization defaults, lightly tuned |

### 4.3 Runtime process architecture

Everything from rung 02, plus:

| Node | Interface | Direction | Type | Rate |
|---|---|---|---|---|
| `ekf_filter_node` | `/odom` | sub | `nav_msgs/Odometry` | 30 Hz in |
| | `/odometry/filtered` | pub | `nav_msgs/Odometry` | 30 Hz |
| | TF `odom → base_link` | broadcast | | 30 Hz |
| | `/set_pose` | sub | `PoseWithCovarianceStamped` | manual filter reset *(robot_localization standard interface — inference)* |
| | `/diagnostics` | pub | `diagnostic_msgs/DiagnosticArray` | periodic *(standard — inference)* |

**Data flow:** `base_bridge /odom → ekf_filter_node → /odometry/filtered`. Nav2 consumes `/odometry/filtered` as design §5.2 intends — `nav2_params.yaml` sets `odom_topic: /odometry/filtered` for both `bt_navigator` and `controller_server` (B-8 resolved 2026-07-17, GAP-4).

**TF ownership (GAP-5 resolved 2026-07-17):** the EKF is the sole `odom → base_link` broadcaster from this rung up — `base_driver.yaml` now defaults `publish_tf: false`, and base_bridge only re-enables it via the explicit `publish_tf:=true` launch arg for rung-02-only bench work. (Historically both broadcast at 30 Hz; mock agreement masked the defect, while on hardware the EKF-smoothed pose would diverge from raw integration and TF consumers would see time-interleaved jumps.)

### 4.4 Measurable outputs

| Output | Measurement | Expected |
|---|---|---|
| Filtered odometry | `ros2 topic hz /odometry/filtered` → ~30 Hz; `echo` | Pose tracks `/odom`; covariance populated (unlike `/odom`, which leaves covariance zero) |
| TF health | `ros2 run tf2_ros tf2_echo odom base_link` | Continuous transform; **watch for dual-publisher jitter** |
| Node liveness | `ros2 node list` | `ekf_filter_node` present exactly once (twice indicates rung 06's duplicate) |
| Script | `./scripts/verify_rung_03.sh` | `[PASS] /odometry/filtered exists`, exit 0 (existence only — does not verify rate or content) |

### 4.5 Requirements traceability

| Requirement | Relationship |
|---|---|
| **SYS-NAV-2** (≤0.15 m localization) | **Supporting mechanism** — the EKF provides the smoothed odom-frame state that AMCL corrects into the map frame. With the IMU disabled its contribution over raw odometry is modest; enabling `imu0` after the hardware rewire is the intended upgrade path (design §5.1: "wheel odom ⊕ BNO055") |
| **SYS-NAV-3/4** | Supporting — Nav2 controller behavior depends on odometry quality |
| Design §5.2 row `ekf_localization` | Verifies the designed node exists with the designed I/O (`/odom` in, `/odometry/filtered` + TF out); the designed `/imu/data` input is dormant |
| TC coverage | No dedicated TC; rung 03 is infrastructure for TC-16/TC-19 class tests |

---

## 5. Rung 04 — SLAM

```bash
ros2 launch billiebot_bringup 04_slam.launch.py mock:=true
```

### 5.1 Referenced files

| File | Role |
|---|---|
| `billiebot_bringup/launch/04_slam.launch.py` | Rung entry; includes `01_lidar` + `03_ekf` (which includes 02) + `slam.launch.py` |
| `billiebot_navigation/launch/slam.launch.py` | Starts `slam_toolbox` `async_slam_toolbox_node` |
| `billiebot_navigation/config/slam_toolbox_params.yaml` | Full mapper configuration (Ceres solver, loop closure, scan matching) |
| All rung 01/02/03 files | Transitively included |
| External: `slam_toolbox` *(external dependency)* | |

### 5.2 Key input parameters

From `slam_toolbox_params.yaml`:

| Parameter | Value | Significance |
|---|---|---|
| `mode` | `mapping` | Online map building (vs. localization mode) |
| `scan_topic` | `/scan` | Ties directly to rung 01's output |
| `odom_frame`/`map_frame`/`base_frame` | `odom`/`map`/`base_link` | Standard REP-105 chain |
| `resolution` | 0.05 m | Occupancy-grid cell size |
| `max_laser_range` | 12.0 m | Matches RPLidar A1 envelope |
| `map_update_interval` | 5.0 s | `/map` republish cadence |
| `minimum_travel_distance` / `_heading` | 0.5 m / 0.5 rad | New scan-node admission gates — robot must move for the map to grow |
| `transform_publish_period` | 0.02 s | `map → odom` TF at 50 Hz |
| `do_loop_closing` | `true` | With `loop_search_maximum_distance: 3.0`, apartment-scale loop closure |
| `solver_plugin` | `CeresSolver` (SPARSE_NORMAL_CHOLESKY / LM) | Pose-graph backend |

Launch args: `mock` (propagated to 01 and 03), `use_sim_time` (default false).

### 5.3 Runtime process architecture

Everything from rungs 01+02+03, plus:

| Node | Interface | Direction | Type | Rate |
|---|---|---|---|---|
| `slam_toolbox` | `/scan` | sub | `LaserScan` | ~5.5 Hz in (real) |
| | TF `odom → base_link` | consume | | needs a valid odom chain |
| | `/map` | pub | `nav_msgs/OccupancyGrid` | every 5 s (`map_update_interval`) |
| | `/map_metadata` | pub | `nav_msgs/MapMetaData` | with map |
| | TF `map → odom` | broadcast | | 50 Hz |
| | `/slam_toolbox/save_map`, `/slam_toolbox/serialize_map` etc. | services | `slam_toolbox` srvs | on demand *(standard slam_toolbox interface — inference)* |

**Composition caveats (mock):**
1. `/scan` never appears (rung 01 defect) → `slam_toolbox` idles; **no `/map` is produced**. VERIFICATION.md itself concedes this: "*requires real lidar data*". In mock mode this rung verifies only process liveness.
2. `04 → 01(mock)` launches `mock_lidar_stub` (a base_bridge) **and** `04 → 03 → 02` launches the real `base_bridge` — two instances of the same executable publishing `/odom`, `/joint_states`, `/battery_state`, and `odom→base_link` TF simultaneously, plus two `/e_stop` servers (Appendix B-2). Combined with the EKF broadcast, **three** publishers contend for `odom → base_link`.

**Real-mode data flow:** teleoperate the robot (operator publishes `/cmd_vel`, per Build Phase 2 of the design) → wheel odometry + EKF hold the `odom` frame → `slam_toolbox` scan-matches `/scan` against the growing pose graph → publishes `/map` and the `map → odom` correction.

### 5.4 Measurable outputs

| Output | Measurement | Expected |
|---|---|---|
| Map | `ros2 topic echo /map --once` (header + info); RViz Map display | Occupancy grid appears and **grows as the robot is driven**; walls crisp after loop closure (physical+data) |
| TF chain complete | `ros2 run tf2_tools view_frames` | `map → odom → base_link → …` one connected tree |
| Map persistence (SYS-NAV-1 "persist") | `ros2 run nav2_map_server map_saver_cli -f apartment` or `/slam_toolbox/save_map` | `apartment.yaml` + `.pgm` on disk — this artifact is the **input to rungs 05/06** |
| Pose-graph health | slam_toolbox console output | Scan-match responses above `link_match_minimum_response_fine: 0.1` |
| Mock ceiling | `ros2 node list` | Nodes alive, but no `/map` — expected failure mode |

### 5.5 Requirements traceability

| Requirement | Relationship |
|---|---|
| **SYS-NAV-1** ("build and persist a 2-D occupancy map using onboard lidar SLAM", Verify: **D**) | **Directly satisfied by demonstration** on real hardware — this is *the* rung for SYS-NAV-1. The Build Plan Phase 2 exit criterion ("Map complete") is this rung's acceptance |
| **SYS-NAV-2** | Enabler — produces the saved map AMCL localizes against |
| **SYS-NAV-6 / SYS-FND-1** | Enabler — patrol waypoints (`patrol_waypoints.yaml`) and room boundaries (`rooms.yaml`) are defined in the map frame this rung creates; both configs carry "update after mapping" notes |
| TC coverage | Feeds TC-16 (waypoint navigation) indirectly; no dedicated mapping TC exists — **gap**: consider a TC that asserts a saved map file with >N known-free cells |

---

## 6. Rung 05 — AMCL Localization

```bash
ros2 launch billiebot_bringup 05_amcl.launch.py map:=/path/to/map.yaml
```

(Note: VERIFICATION.md shows no `mock:=true` for this rung — it presumes a real map and real lidar.)

### 6.1 Referenced files

| File | Role |
|---|---|
| `billiebot_bringup/launch/05_amcl.launch.py` | Rung entry; includes `01_lidar` + `03_ekf`(→02) + `localization.launch.py`, forwarding `map` |
| `billiebot_navigation/launch/localization.launch.py` | `nav2_map_server` + `nav2_amcl` + `nav2_lifecycle_manager` |
| `billiebot_navigation/config/amcl_params.yaml` | Particle filter configuration |
| `billiebot_navigation/config/nav2_params.yaml` | Only its `map_server:` section is consumed here |
| User-supplied `map.yaml` + `.pgm` | The rung-04 artifact |
| External: `nav2_map_server`, `nav2_amcl`, `nav2_lifecycle_manager` *(external)* | |

### 6.2 Key input parameters

| Parameter | Value | Significance |
|---|---|---|
| `map` (launch arg) | default `''` | **Required.** With the empty default, `map_server` has no `yaml_filename` and its lifecycle *configure* fails; the lifecycle manager cannot reach *active* — the rung silently does nothing useful (Appendix B-6) |
| `min_particles` / `max_particles` | 500 / 2000 | Filter size, apartment-appropriate |
| `laser_model_type` | `likelihood_field` | Standard indoor model; `max_beams: 60` subsampling |
| `laser_min_range` / `laser_max_range` | 0.15 / 12.0 m | Matches A1 |
| `update_min_d` / `update_min_a` | 0.25 m / 0.2 rad | Motion gates for filter updates — the robot must move to converge |
| `alpha1–alpha5` | 0.2 | `DifferentialMotionModel` odometry noise |
| `transform_tolerance` | 1.0 s | Future-dating of the `map→odom` TF |
| `set_initial_pose` / `initial_pose` | `true` / (0,0,0,0) | Auto-initializes at the map origin — correct only if the robot physically starts at the mapping start point (`map_start_at_dock: true` in rung 04 makes this consistent) |
| `tf_broadcast` | `true` | AMCL owns `map → odom` |

### 6.3 Runtime process architecture

Everything from rungs 01+02+03 (real lidar assumed), plus three lifecycle-managed nodes:

| Node | Interface | Direction | Type | Rate |
|---|---|---|---|---|
| `map_server` | `/map` | pub (transient-local) | `nav_msgs/OccupancyGrid` | latched once |
| `amcl` | `/scan` | sub | `LaserScan` | ~5.5 Hz in |
| | `/initialpose` | sub | `PoseWithCovarianceStamped` | operator relocalization (RViz "2D Pose Estimate") |
| | `/particle_cloud` | pub | `nav2_msgs/ParticleCloud` | on filter update (motion-gated) |
| | `/amcl_pose` | pub | `PoseWithCovarianceStamped` | on update; `save_pose_rate: 0.5` |
| | TF `map → odom` | broadcast | | per update, future-dated 1.0 s |
| `lifecycle_manager_localization` | `autostart: true`, manages `[map_server, amcl]` | | | bond/heartbeat |

**Full TF chain now:** `map →(amcl) odom →(base_bridge ∥ EKF, duplicated) base_link →(rsp) chassis → sensors`.

**Mock composition note:** if launched with `mock:=true`, the rung 01 stub again yields no `/scan`, so AMCL never updates; and the duplicate base-bridge issue from §5.3 recurs.

### 6.4 Measurable outputs

| Output | Measurement | Expected |
|---|---|---|
| Particle cloud | RViz `ParticleCloud` display; `ros2 topic echo /particle_cloud --once` | Cloud appears at the initial pose; **visibly converges** (shrinks) as the robot drives — the canonical physical+data verification |
| Pose estimate | `ros2 topic echo /amcl_pose` | Covariance decreases with motion; pose matches the robot's true room position |
| `map → odom` TF | `ros2 run tf2_ros tf2_echo map odom` | Present and stable after convergence |
| Lifecycle | `ros2 lifecycle get /map_server /amcl` *(or lifecycle manager log)* | Both `active`; failure here almost always means a bad/missing `map` arg |
| **SYS-NAV-2 quantitative test** | Place robot at ≥3 surveyed points; compare `/amcl_pose` to tape-measured ground truth | Mean position error ≤ 0.15 m |
| Script | none exists for rung 05 (gap) | — |

### 6.5 Requirements traceability

| Requirement | Relationship |
|---|---|
| **SYS-NAV-2** (localize within saved map, ≤0.15 m mean error, Verify: **T**) | **Directly targeted** — this rung *is* the localization mechanism; the quantitative test above completes it. Build Phase 2 exit: "reloc ≤ 0.15 m (TC-NAV-2)" |
| **SYS-NAV-3/5/6, SYS-FND-*** | Prerequisite — every map-frame behavior (Nav2 goals, patrol waypoints, dog map pose, room attribution) depends on this TF |
| **SYS-STL-2** | Enabler — logged event locations (x, y, room) are only meaningful with map-frame localization |
| TC coverage | Supports TC-16; no dedicated relocalization TC in the VERIFICATION.md table — the design's TC-NAV-2 naming suggests one was intended (**traceability gap**) |

---

## 7. Rung 06 — Nav2

```bash
ros2 launch billiebot_bringup 06_nav2.launch.py mock:=true map:=/path/to/map.yaml
```

### 7.1 Referenced files

| File | Role |
|---|---|
| `billiebot_bringup/launch/06_nav2.launch.py` | Rung entry; includes `05_amcl` (full chain) + `navigation.launch.py` |
| `billiebot_navigation/launch/navigation.launch.py` | `controller_server` + `planner_server` + `behavior_server` + `bt_navigator` + `lifecycle_manager_navigation` (Nav2 only — its former second `ekf_node` was removed, GAP-6 resolved 2026-07-17) |
| `billiebot_navigation/config/nav2_params.yaml` | All Nav2 servers + both costmaps |
| `billiebot_bringup/scripts/verify_rung_06.sh` | Verification script |
| All rung 01/02/03/05 files | Transitive |
| External: `nav2_controller`, `nav2_planner`, `nav2_behaviors`, `nav2_bt_navigator`, `nav2_lifecycle_manager`, `dwb_core`, `nav2_navfn_planner`, `nav2_costmap_2d` *(external)* | |

### 7.2 Key input parameters

From `nav2_params.yaml`:

| Group | Parameter | Value | Significance |
|---|---|---|---|
| controller | `controller_frequency` | 20 Hz | `/cmd_vel` output rate while navigating |
| controller | `FollowPath` plugin | `dwb_core::DWBLocalPlanner` | Design §5.1 choice ("DWB or MPPI") |
| DWB | `max_vel_x`, `max_speed_xy` | **0.3 m/s** | **Direct encoding of SYS-NAV-5's normal-transit cap.** The 0.15 m/s near-dog limit has **no implementation** (no speed-filter costmap layer/keepout zone exists — Appendix B-7) |
| DWB | `max_vel_theta` / `acc_lim_x` | 1.0 rad/s / 2.5 m/s² | Kinodynamic envelope |
| progress_checker | `required_movement_radius`, `movement_time_allowance` | 0.5 m / 10 s | **Stuck detection** — looser than SYS-NAV-4's ">5 s no progress" wording (Appendix B-7) |
| goal checker | `xy_goal_tolerance` | 0.25 m | Waypoint arrival radius |
| planner | `GridBased` | `NavfnPlanner`, `tolerance 0.5`, `allow_unknown: true` | Global planner |
| behaviors | `behavior_plugins` | `["spin", "backup", "wait"]` @ `cycle_frequency` 10 Hz | The SYS-NAV-4 recovery primitives (launch file comments this explicitly) |
| bt_navigator | `navigators` | `navigate_to_pose`, `navigate_through_poses` | The action surface consumed by rung 13 and TC-16 |
| bt_navigator, controller_server | `odom_topic` | `/odometry/filtered` | EKF-filtered odometry per design §5.2 (B-8 resolved 2026-07-17, GAP-4; controller_server's param had been silently defaulting to raw odom) |
| local costmap | rolling 3×3 m @ `update` 5 Hz / `publish` 2 Hz, `robot_radius` 0.18 m, inflation 0.55 m | Obstacle layer from `/scan` (marking+clearing, 2.5 m obstacle range) |
| global costmap | full map @ 1 Hz, static + obstacle + inflation layers | Same `/scan` source |

### 7.3 Runtime process architecture

Everything from rung 05, plus six nodes. The action/topic surface:

| Node | Interface | Direction | Type/notes | Rate |
|---|---|---|---|---|
| `controller_server` | `/cmd_vel` | pub | `geometry_msgs/Twist` → consumed by `base_bridge` | 20 Hz while navigating |
| | `follow_path` | action server | `nav2_msgs/action/FollowPath` | |
| | `/local_costmap/costmap` | pub | `OccupancyGrid` | 2 Hz |
| `planner_server` | `compute_path_to_pose` | action server | `nav2_msgs/action/ComputePathToPose`; publishes `/plan` (Path) | on request |
| | `/global_costmap/costmap` | pub | `OccupancyGrid` | 1 Hz |
| `behavior_server` | `spin`, `backup`, `wait` | action servers | `nav2_msgs` actions — recovery primitives | 10 Hz cycle |
| `bt_navigator` | `navigate_to_pose` | **action server** | `nav2_msgs/action/NavigateToPose` — the system's primary mobility API (used by `mission_controller`, `approach_dog_server`, TC-16) | |
| | `navigate_through_poses` | action server | multi-waypoint variant → SYS-NAV-6 | |
| `ekf_filter_node` | single instance, from rung 03 | | the former duplicate launched by `navigation.launch.py` was removed 2026-07-17 (Appendix B-4, GAP-6) | 30 Hz |
| `lifecycle_manager_navigation` | manages the four Nav2 servers | | autostart | |

**Closed control loop now exists:** `navigate_to_pose` goal → bt_navigator ticks its BT → planner (global costmap/map) → controller DWB (local costmap from `/scan`, odometry) → `/cmd_vel` → `base_bridge` → serial `m` commands → wheels → encoders → `/odom` → EKF/AMCL → TF → costmaps. This is the design's IBD-01 "perception→planning loop on one machine" realized.

**Mock ceiling:** no `/scan` → costmap obstacle layers receive nothing; with no map arg the entire localization side is inactive. `verify_rung_06.sh` can still find the *action* (bt_navigator advertises it once active) but costmap topic checks will fail without a map. Effectively, rung 06 is a **real-hardware rung**; mock only proves process/action liveness.

### 7.4 Measurable outputs

| Output | Measurement | Expected |
|---|---|---|
| Action surface | `ros2 action list` | `/navigate_to_pose`, `/navigate_through_poses`, `/follow_path`, `/compute_path_to_pose`, `/spin`, `/backup`, `/wait` |
| Costmaps | `ros2 topic hz /local_costmap/costmap` (~2 Hz), `/global_costmap/costmap` (~1 Hz); RViz costmap displays | Inflated obstacles visible around lidar returns |
| **End-to-end navigation (physical)** | RViz "Nav2 Goal" or `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0}}}}"` | Robot plans (green `/plan` path), drives ≤0.3 m/s, stops within 0.25 m of goal, action returns SUCCEEDED |
| Dynamic obstacle response | Step in front of the moving robot | Local costmap marks; robot replans/stops (SYS-NAV-3 demonstration) |
| Recovery behaviors | Block the robot completely | progress checker trips after ~10 s; spin/backup recoveries execute (SYS-NAV-4 partial demonstration) |
| Speed cap | `ros2 topic echo /cmd_vel` during transit | `linear.x ≤ 0.3` always (TC-18 evidence) |
| Script | `./scripts/verify_rung_06.sh` | 3/3 PASS (action + both costmap topics) |

### 7.5 Requirements traceability

| Requirement | Relationship |
|---|---|
| **SYS-NAV-3** (collision-free paths, replanning around dynamic obstacles incl. the dog, Verify: D) | **Satisfied (mechanism + demonstration)** — planner/controller/costmaps with marking+clearing from `/scan`. Note the dog (0.15–0.25 m tall) *is* within lidar plane visibility only if the lidar height permits; the OAK-D detection path does not feed costmaps (**could**: add a speed/keepout filter fed by `/dog/pose_map`) |
| **SYS-NAV-4** (stuck detection >5 s, recoveries, 3 failures → SAFE + alert, Verify: T) | **Partial** — recovery primitives (spin/backup/wait) and a progress checker exist; the 5 s threshold is configured as 10 s/0.5 m; the "3 failed recoveries → SAFE + alert" escalation is delegated to `mission_controller` whose failure counter is never incremented (Appendix B-5). TC-19 cannot currently pass end-to-end |
| **SYS-NAV-5** (≤0.3 m/s transit; ≤0.15 m/s within 2 m of dog, Verify: T) | **Half-satisfied** — 0.3 cap encoded in DWB; near-dog slowdown unimplemented (design intended "Nav2 speed-restricted zones / speed-filter mask around detected dog") — TC-18 partial |
| **SYS-NAV-6** (configurable patrol route, Verify: D) | **Enabler** — `navigate_through_poses`/waypoint capability exists; `patrol_waypoints.yaml` defines the route, but **no launched node reads that file** (Appendix B-5). TC-16 currently has no automated driver |
| **SYS-FND-3** | Enabler — `approach_dog_server` (rung 13) rides on `navigate_to_pose` |
| Design §5.2 `nav2` row | Matches: subscribes map/scan/odometry, publishes `/cmd_vel`, serves NavigateToPose. Deviation: `FollowWaypoints` named in design vs. `navigate_through_poses` configured here |

---

## 8. Rung 07 — OAK-D Dog Detector

```bash
ros2 launch billiebot_bringup 07_oakd.launch.py mock:=true
```

### 8.1 Referenced files

| File | Role |
|---|---|
| `billiebot_bringup/launch/07_oakd.launch.py` | Rung entry — launches the detector **standalone** (no other rungs) |
| `billiebot_perception/billiebot_perception/oakd_dog_detector.py` | Node source (DepthAI pipeline + mock) |
| `billiebot_interfaces/msg/DogDetection3D.msg` | Output message |
| `billiebot_bringup/scripts/verify_rung_07.sh` | Verification script |
| `billiebot_perception/config/perception.yaml` | **Not loaded by this rung** — the launch passes only `{'mock': mock}`; the node runs on its declared defaults (which happen to equal the yaml values). Only `perception.launch.py` (unused by the ladder) loads the yaml. Same pattern for rungs 09/10 (Appendix B-10) |
| External: `depthai` Python SDK, YOLOv8n `.blob` model *(external)* | |

### 8.2 Key input parameters

| Parameter | Default | Significance |
|---|---|---|
| `mock` | `false` | Synthetic detections vs. DepthAI pipeline |
| `confidence_threshold` | 0.5 | Detection gate (design: "COCO class `dog`, tuned threshold") |
| `model_path` | `''` | Path to the YOLOv8n blob. **Empty default ⇒ in real mode the node logs an error and creates no timer — zero output.** A real deployment must set this parameter (Appendix B-11) |
| `camera_frame` | `oakd_link_optical` | Stamped into detections; matches `oakd_lite.xacro` |
| `publish_rate_hz` | 5.0 | Poll/publish rate — deliberately matches SYS-PER-1's "≥5 FPS" |

Real-pipeline internals (hard-coded): 416×416 RGB preview from the 1080p color camera; stereo depth from both 400p monos (HIGH_DENSITY preset); `YoloSpatialDetectionNetwork` with depth thresholds 100–5000 mm; COCO label filter `== 16` (dog).

### 8.3 Runtime process architecture

One node, `oakd_dog_detector`:

| Interface | Direction | Type | Rate |
|---|---|---|---|
| `/dog/detections_3d` | pub | `billiebot_interfaces/DogDetection3D` — bbox (px), confidence, `geometry_msgs/Point position` in camera optical frame (m), depth, label | ≤5 Hz (only when a dog is detected) |
| `/dog/found` | pub | `std_msgs/Bool` | 5 Hz continuous (true/false every tick) |

**Mock behavior (5 Hz timer):** each tick, 70 % probability of publishing a detection with confidence ≈0.85±0.1 at position ≈(0.05, −0.10, 2.0±0.3) m (i.e., a dog ~2 m in front of the optical axis) plus `found=true`; otherwise `found=false` only. Expected steady-state: `/dog/detections_3d` ≈3.5 Hz, `/dog/found` = 5 Hz alternating.

**Real behavior:** detections come from the OAK-D's on-device NN (design: "~15–20 FPS on RVC2, zero Jetson GPU load"), drained by the node at 5 Hz. Depth (`spatialCoordinates`, mm→m) gives the 3-D position. Latent defect: `det.xmin/xmax` from DepthAI are **normalized 0–1 floats**; `int(det.xmin)` truncates the bbox fields to 0 (Appendix B-11) — position/depth are unaffected.

No TF is consumed or produced; frame resolution happens downstream in rung 08.

### 8.4 Measurable outputs

| Output | Measurement | Expected |
|---|---|---|
| Detections | `ros2 topic echo /dog/detections_3d` | Mock: bursts at ~3.5 Hz, conf 0.75–0.95, z ≈ 1.7–2.3 m. Real: detections when a dog (or dog photo) is in frame |
| Found flag | `ros2 topic hz /dog/found` → 5 Hz | Boolean duty cycle ≈70 % true in mock |
| Rate vs. SYS-PER-1 | `ros2 topic hz /dog/detections_3d` with a continuously visible dog | ≥5 Hz sustained |
| Depth accuracy (real) | Place target at tape-measured 2.0 m | `position.z` within ±0.2 m (SYS-PER-2 figure) |
| Script | `./scripts/verify_rung_07.sh` | 2/2 topics PASS |

### 8.5 Requirements traceability

| Requirement | Relationship |
|---|---|
| **SYS-PER-1** (dog in RGB ≥5 FPS, ≥85 % recall ≤4 m, Verify: T) | **Mechanism satisfied** — 5 Hz spatial YOLO with confidence gate. The recall figure requires a field test with the actual dog (Build Phase 4 / TC-07); mock mode proves plumbing only |
| **SYS-PER-2** (3-D position ±0.2 m @ 2 m, Verify: T) | **Partial** — camera-frame 3-D position from stereo; map-frame completion is rung 08 |
| **TC-07** (Dog 3D detection → SYS-PER-1/2) | **Satisfied** at topic level by the mock suite |
| Design §5.2 `oakd_dog_detector` row | Matches pubs `/dog/detections_3d`; design also lists `/oak/rgb/preview` which is **not implemented** (no image preview topic — minor deviation, limits SYS-PLT-4 operator visualization of detections) |

---

## 9. Rung 08 — Dog Locator

```bash
ros2 launch billiebot_bringup 08_dog_locator.launch.py
```

(No `mock` argument exists — the node is pure software.)

### 9.1 Referenced files

| File | Role |
|---|---|
| `billiebot_bringup/launch/08_dog_locator.launch.py` | Rung entry — starts `dog_locator` with **no parameters** (code defaults apply) |
| `billiebot_perception/billiebot_perception/dog_locator.py` | Node source |
| `billiebot_interfaces/msg/DogDetection3D.msg` | Input message |
| Upstream dependencies at runtime: rung 07 (detections), rung 02 (URDF static TF for `oakd_link_optical`), rung 04 or 05 (`map → odom` TF) | The rung is only *meaningful* inside a larger stack |

### 9.2 Key input parameters

| Parameter | Default | Significance |
|---|---|---|
| `map_frame` | `map` | Target frame for the transform |
| `min_confidence` | 0.5 | Drops low-confidence detections before transforming |

### 9.3 Runtime process architecture

One node, `dog_locator`, purely event-driven (no timer):

| Interface | Direction | Type | Rate |
|---|---|---|---|
| `/dog/detections_3d` | sub | `DogDetection3D` | ≤5 Hz in |
| TF (`Buffer` + `TransformListener`) | consume | needs `map ← odom ← base_link ← chassis ← oakd_link_optical` complete at the detection timestamp (0.1 s timeout) | |
| `/dog/pose_map` | pub | `geometry_msgs/PoseStamped` (map frame, identity orientation) | one per accepted detection (≤5 Hz) |
| `/dog/found` | pub | `std_msgs/Bool` (always `true` on success) | with each pose |

Processing: detection → confidence gate → wrap `position` as `PointStamped` in the camera optical frame → `tf_buffer.transform(..., map)` → publish pose. On `TransformException` it logs at debug level and stays silent — hence VERIFICATION.md's phrasing "*publishing when detections + TF available*."

Two integration observations:
1. **`/dog/found` has two publishers** once rung 07 and 08 both run (07 publishes true/false at 5 Hz; 08 publishes only `true` on success). Subscribers like `mission_controller` see an interleaved stream (Appendix B-12).
2. The source imports only `Buffer/TransformListener/TransformException` from `tf2_ros`; `Buffer.transform()` on a `PointStamped` requires the type-registration side-effect of importing `tf2_geometry_msgs`, which is absent. If that import is not pulled in transitively, every callback raises an unregistered-type error that the `except TransformException` clause does **not** catch — a **plausible latent defect to check on first hardware run** (Appendix B-13).

### 9.4 Measurable outputs

| Output | Measurement | Expected |
|---|---|---|
| Map-frame pose | `ros2 topic echo /dog/pose_map` | PoseStamped in `map` frame at ≤5 Hz **only when** rungs 02+05(+07) provide TF and detections; RViz Pose display sits on the dog's map location |
| Standalone behavior | launch rung 08 alone | Node starts, logs `DogLocator started`, publishes nothing — correct (no inputs) |
| Accuracy (real) | Compare `/dog/pose_map` to the dog's surveyed position | ±0.2 m at 2 m per SYS-PER-2 |
| Consistency | Drive the robot around a static dog target | `/dog/pose_map` stays fixed in the map frame while the camera-frame detection moves — the definitive TF-correctness test |

### 9.5 Requirements traceability

| Requirement | Relationship |
|---|---|
| **SYS-PER-2** (3-D dog position **in the map frame**, Verify: T) | **Completes the requirement** begun in rung 07 — this rung is the camera→map transform stage. TC-08 ("Dog locator TF") |
| **SYS-FND-1** | Enabler — "last-known location first" seeding requires exactly this map-frame pose |
| **SYS-FND-3** | Enabler — `approach_dog_server` consumes `/dog/pose_map` to compute the standoff goal |
| **SYS-STL-1/2** | Enabler — `state_fusion` takes `/dog/pose_map` for position/room attribution; `dog_logger` writes x/y/room from it |
| Design §5.2 `dog_locator` row | Matches exactly (subs `/dog/detections_3d` + TF; pubs `/dog/pose_map`, `/dog/found`) |

---

## 10. Rung 09 — Thermal Camera

```bash
ros2 launch billiebot_bringup 09_thermal.launch.py mock:=true
```

### 10.1 Referenced files

| File | Role |
|---|---|
| `billiebot_bringup/launch/09_thermal.launch.py` | Rung entry — `thermal_node` standalone, only `mock` passed (defaults = `perception.yaml` values; yaml itself not loaded, Appendix B-10) |
| `billiebot_perception/billiebot_perception/thermal_node.py` | Node source (MLX90640 driver + blob detector + mock) |
| `billiebot_interfaces/msg/ThermalBlob.msg` | Output message |
| External: `adafruit_mlx90640` + `board` (Blinka) over I²C *(external)* | |

### 10.2 Key input parameters

| Parameter | Default | Significance |
|---|---|---|
| `mock` | `false` | Synthetic 22 °C ambient + 35 °C blob vs. real sensor |
| `publish_rate_hz` | 4.0 | Matches the sensor's configured `REFRESH_4_HZ` |
| `thermal_frame` | `thermal_link_optical` | Matches `thermal.xacro` (mounted pitched 0.2 rad down for the low dog) |
| `dog_temp_min` / `dog_temp_max` | 30.0 / 40.0 °C | **The SYS-PER-3 temperature window, literally encoded** |
| `min_blob_area` | 8 px | SYS-PER-3's "≥ N pixels" with N=8 (of 768 total) |
| `i2c_bus` / `i2c_address` | 1 / 0x33 | Pi I²C wiring (per design: thermal on the Pi, not the Jetson, avoiding I²C clock-stretch issues) |

### 10.3 Runtime process architecture

One node, `thermal_node`, on a 4 Hz timer:

| Interface | Direction | Type | Rate |
|---|---|---|---|
| `/thermal/image` | pub | `sensor_msgs/Image`, **32×24, encoding `32FC1`** (raw °C floats, step 128 B) | 4 Hz |
| `/thermal/blob` | pub | `billiebot_interfaces/ThermalBlob` — centroid (cx, cy), area, max/mean temp, `is_dog_candidate` | 4 Hz **only when** ≥8 in-window pixels exist (silent otherwise) |

Blob algorithm: threshold all pixels into [30, 40] °C → if count ≥ 8, publish centroid + stats. It is a *global* threshold count, not connected-component analysis — two warm objects merge into one "blob" (adequate for a single-dog apartment, worth knowing for verification).

**Mock:** ambient 22±0.5 °C with a circular ~45 px blob at 35±1 °C centered at (16, 12) — always exceeds the 8 px gate, so `/thermal/blob` publishes every frame with `is_dog_candidate: true`, `mean_temp ≈ 35`. (Downstream note: 35 °C > the 33 °C SLEEPING cutoff in `state_fusion`, so mock thermal biases fusion toward RESTING/ACTIVE, never SLEEPING.)

### 10.4 Measurable outputs

| Output | Measurement | Expected |
|---|---|---|
| Image stream | `ros2 topic hz /thermal/image` → 4 Hz; `echo --once` shows width 32, height 24, encoding `32FC1` — **exactly the VERIFICATION.md criterion** | RViz/Foxglove need a colormap for 32FC1; values are literal °C |
| Blob stream | `ros2 topic echo /thermal/blob` | Mock: area ≈ 45, mean ≈ 35 °C, candidate true @ 4 Hz. Real: publishes only when a warm body is in view |
| **Physical (real)** | Hold a hand / have the dog at ≤1.5 m; then remove | Blob appears with plausible temps (30–37 °C), disappears when clear; works with lights **off** — the darkness half of SYS-PER-3 |
| Range test | Dog at 1.5 m per SYS-PER-3 | area ≥ 8 px still achieved (55° FOV, 32 px across) |

### 10.5 Requirements traceability

| Requirement | Relationship |
|---|---|
| **SYS-PER-3** (warm body 30–40 °C, ≥N px, in darkness, ≤1.5 m, Verify: T) | **Directly implemented** — thresholds are literal parameters. TC-09 (thermal imaging) + TC-10 (blob detection) both land here |
| **SYS-STL-1** | Enabler — `is_dog_candidate` blobs and `mean_temp` are fusion inputs (the SLEEPING/RESTING discriminator in `state_fusion` is thermal temperature) |
| **SYS-PER-5** | Compensator — the design's night-vision gap plan is "rely on the thermal camera at night for MVP" |
| Design §5.2 `thermal_node + thermal_blob` row | Design shows two nodes; implemented as one node with two publishers — functionally equivalent |

---

## 11. Rung 10 — NoIR Camera

```bash
ros2 launch billiebot_bringup 10_noir.launch.py mock:=true
```

### 11.1 Referenced files

| File | Role |
|---|---|
| `billiebot_bringup/launch/10_noir.launch.py` | Rung entry — `noir_cam_node` standalone, only `mock` passed |
| `billiebot_perception/billiebot_perception/noir_cam_node.py` | Node source (picamera2 + mock) |
| External: `picamera2`/libcamera on the Pi *(external)* | |

### 11.2 Key input parameters

| Parameter | Default | Significance |
|---|---|---|
| `mock` | `false` | Synthetic dark-grey frames vs. CSI capture |
| `publish_rate_hz` | 5.0 | Frame rate |
| `camera_frame` | `noir_link_optical` | Matches `noir_camera.xacro` |
| `width` / `height` | 640 / 480 | RGB888 still-configuration capture size |

### 11.3 Runtime process architecture

One node, `noir_cam_node`, on a 5 Hz timer:

| Interface | Direction | Type | Rate |
|---|---|---|---|
| `/noir/image` | pub | `sensor_msgs/Image` 640×480 `rgb8` | 5 Hz (≈4.6 MB/s raw — noteworthy if ever bridged across the Wi-Fi DDS link; currently **no node subscribes to it at all**) |

**Mock:** uniform dark-grey (RGB 40,40,40) frames simulating low light. **Real:** `Picamera2` still-configuration capture loop.

This rung is a *sensor-liveness* rung only: no detector, classifier, or logger consumes `/noir/image` anywhere in the codebase. It exists to prove the CSI/night-imaging hardware path called for by SYS-PER-5.

### 11.4 Measurable outputs

| Output | Measurement | Expected |
|---|---|---|
| Image stream | `ros2 topic hz /noir/image` → 5 Hz — the sole VERIFICATION.md criterion | |
| Image content | Foxglove/rqt_image_view | Mock: flat dark grey. Real: recognizable scene; in darkness, IR-sensitive imagery (limited without an illuminator) |
| **Physical (real)** | Lights off + 850 nm source (if fitted) | Scene visible in the image where a normal camera sees black |

### 11.5 Requirements traceability

| Requirement | Relationship |
|---|---|
| **SYS-PER-5** (low-light detection via NoIR when lux < threshold, Verify: D) | **Partial at best** — imagery is published, but (a) no low-light *detector* consumes it (the requirement says *detect*, not *image*), (b) no lux thresholding/night-mode switching exists, and (c) the design's own HW gap list notes the required IR illuminator is not yet fitted. **Could** satisfy SYS-PER-5 by feeding `/noir/image` to a YOLO instance (design reserved Jetson GPU headroom for exactly this) plus the illuminator |
| TC coverage | **None** — TC-01…TC-22 contain no NoIR test; `run_all_mock_tests.sh` never checks `/noir/image` (traceability gap, Appendix A) |
| Design §5.2 `noir_cam` row | Matches (`/noir/image (night mode)`), with "night mode" logic not yet present |

---

## 12. Rung 11 — Audio

```bash
ros2 launch billiebot_bringup 11_audio.launch.py mock:=true
```

### 12.1 Referenced files

| File | Role |
|---|---|
| `billiebot_bringup/launch/11_audio.launch.py` | Rung entry; includes the package launch below |
| `billiebot_audio/launch/audio.launch.py` | Starts both audio nodes **with `audio.yaml`** (unlike the perception rungs, this rung does load its config) |
| `billiebot_audio/config/audio.yaml` | Classifier + speaker parameters |
| `billiebot_audio/billiebot_audio/audio_classifier.py` | YAMNet classifier + DoA node |
| `billiebot_audio/billiebot_audio/speaker_node.py` | `/speak` action server |
| `billiebot_interfaces/msg/AudioEvent.msg`, `billiebot_interfaces/action/Speak.action` | Interfaces |
| External: `tflite_runtime` (or TF), `sounddevice`, `pyusb`, YAMNet `yamnet.tflite` + `yamnet_class_map.csv`, ALSA `aplay` *(external)* | |

### 12.2 Key input parameters

| Node | Parameter | Value | Significance |
|---|---|---|---|
| classifier | `publish_rate_hz` | 2.0 | Classification tick (each tick records a chunk in real mode) |
| classifier | `chunk_duration_sec` / `sample_rate` | 0.975 s / 16 kHz | YAMNet's exact input format (15,600 samples) |
| classifier | `confidence_threshold` | 0.3 | Score gate on the top YAMNet class |
| classifier | `energy_threshold_db` | −30 dB | RMS pre-gate: silence is never classified |
| classifier | `model_path` | `''` | **Real mode requires the operator to supply yamnet.tflite; empty ⇒ error log, no output** (same failure pattern as rung 07) |
| speaker | `min_interval_sec` | 10.0 | **SYS-PLT-6 rate limit** — goals within 10 s of the last playback are `REJECT`ed |
| speaker | `max_volume` | 0.5 | Volume clamp (dog-welfare) |
| speaker | `sounds_dir` | `''` | WAV directory for real playback via `aplay -D plughw:0,0` |

### 12.3 Runtime process architecture

Two nodes:

**`audio_classifier`** (2 Hz timer):

| Interface | Direction | Type | Rate |
|---|---|---|---|
| `/audio/events` | pub | `billiebot_interfaces/AudioEvent` — `event_type` enum {BARK=0, WHINE=1, HOWL=2, LOUD_NOISE=3, SILENCE=4}, confidence, `doa_deg` [0,360), `yamnet_label`, `energy_db` | intermittent (event-driven) |

Real pipeline per tick: record 0.975 s mono @16 kHz (`sounddevice`) → RMS energy gate (−30 dB) → YAMNet TFLite inference → top class mapped through `YAMNET_DOG_CLASSES` (Bark/Bow-wow/Yip/Growling→BARK, Howl→HOWL, Whimper→WHINE, anything else above threshold→LOUD_NOISE) → DoA read via USB vendor control transfer to VID 0x2886 / PID 0x0018. **Risk:** that VID/PID is the ReSpeaker *4-Mic Array (XVF-3000 era)* ID; the design specifies the **XVF3800**, which may enumerate differently — DoA would silently return 0.0 (the code's fallback) (Appendix B-14).

Mock per tick (2 Hz): 15 % chance BARK (conf ≈0.75, random DoA), 5 % chance WHINE (conf ≈0.6) ⇒ expected ≈0.3 barks/s + 0.1 whines/s — a deliberately bark-heavy stream that exercises downstream BARKING fusion and stress-proxy math.

**`speaker_node`**:

| Interface | Direction | Type | Notes |
|---|---|---|---|
| `/speak` | **action server** | `billiebot_interfaces/action/Speak` (goal: `sound_id`, `volume`; feedback: `progress`; result: `success`, `message`) | Goal callback enforces the 10 s rate limit *before* acceptance; volume clamped to 0.5. Mock: simulated 1 s playback with 10 feedback ticks. Real: `aplay` subprocess (30 s timeout). Note: `volume` is computed but never applied to `aplay` (no `-v` mechanism) — real loudness control relies on ALSA mixer state (minor gap) |

### 12.4 Measurable outputs

| Output | Measurement | Expected |
|---|---|---|
| Events | `ros2 topic echo /audio/events` | Mock: BARK/WHINE events, several per 10 s, DoA uniformly random. Real: events **only** when actual barks/loud sounds occur — clap test → LOUD_NOISE; recorded Billie barks → BARK (the SYS-PER-4 recall test) |
| DoA sanity (real) | Bark/clap from a known bearing | `doa_deg` within ±15° of truth (SYS-PER-4 figure) |
| Speak action | `ros2 action send_goal /speak billiebot_interfaces/action/Speak "{sound_id: test.wav, volume: 0.3}"` | Mock: feedback 0.1→1.0 over ~1 s, success. Real: **audible sound** (physical output) |
| Rate limiting | Send a second goal within 10 s | Goal **rejected** — directly observable SYS-PLT-6 evidence |
| Action discovery | `ros2 action list` | `/speak` present |

### 12.5 Requirements traceability

| Requirement | Relationship |
|---|---|
| **SYS-PER-4** (bark/whine/howl/loud detection ≥80 % recall + DoA ±15°, Verify: T) | **Mechanism satisfied** (classification + DoA plumbing + energy gating); the recall figure needs the recorded-Billie test set (Build Phase 4), and DoA carries the XVF3800 VID/PID risk. TC-11 |
| **SYS-PLT-6** (no sudden loud sounds; speaker rate-limited, Verify: I) | **Satisfied by inspection** — 10 s minimum interval, 0.5 volume clamp, and goal-time rejection are all in `speaker_node.py`; the declared-but-unused `fade_in_sec` param shows the fade-in intent is not yet implemented (minor) |
| **SYS-FND-2** (audio DoA re-prioritizes search) | Enabler — `doa_deg` reaches `mission_controller`, which currently only logs it (rung 13, Appendix B-5) |
| **SYS-EXT-1** (Speak as a uniform action server) | **Satisfied** — `/speak` here plus the `/mission/speak` wrapper in rung 13 |
| Design §5.2 `audio_classifier`/`speaker_node` rows | Match (`/audio/events` out; `/speak` rate-limited in) |

---

## 13. Rung 12 — Cognition

```bash
ros2 launch billiebot_bringup 12_cognition.launch.py
```

(No `mock` argument — all four nodes are pure software; "mock" is meaningless for them.)

### 13.1 Referenced files

| File | Role |
|---|---|
| `billiebot_bringup/launch/12_cognition.launch.py` → `billiebot_cognition/launch/cognition.launch.py` | Starts all four cognition nodes with `cognition.yaml` (+ `rooms.yaml` path for the logger) |
| `billiebot_cognition/config/cognition.yaml` | All four nodes' parameters |
| `billiebot_cognition/config/rooms.yaml` | Map-frame room bounding boxes (placeholders pending mapping) |
| `billiebot_cognition/billiebot_cognition/state_fusion.py` | Evidence fusion → `/billie/state` |
| `billiebot_cognition/billiebot_cognition/dog_logger.py` | SQLite WAL event log |
| `billiebot_cognition/billiebot_cognition/daily_report.py` | Scheduled report generator (also `--standalone` CLI) |
| `billiebot_cognition/billiebot_cognition/report_server.py` | FastAPI HTTP server on :8080 |
| `billiebot_interfaces/msg/DogState.msg`, `srv/GetDogState.srv` | Interfaces |
| `billiebot_bringup/scripts/verify_rung_12.sh` | Verification script |
| External: `fastapi`, `uvicorn`, optional `markdown`, `jinja2`, `matplotlib`, `pyyaml` *(external, all soft-degrading except fastapi/uvicorn which disable the server)* | |

### 13.2 Key input parameters

| Node | Parameter | Value | Significance |
|---|---|---|---|
| state_fusion | `publish_rate_hz` | 2.0 | `/billie/state` cadence |
| state_fusion | `window_sec` | 10.0 | Sliding evidence window (design §5.3: "10 s sliding window") |
| state_fusion | `hysteresis_sec` | 3.0 | Minimum dwell before a state change is accepted (anti-flapping) |
| state_fusion | `bark_rate_stress_threshold` | 0.3 barks/s | Normalizer for `stress_proxy = min(1, bark_rate/0.3)` (SYS-EXT-4) |
| dog_logger | `db_path` | `/var/lib/billiebot/billie_events.db` | **System path — the node `os.makedirs` this; without pre-created, user-writable `/var/lib/billiebot` the node crashes on startup** (Appendix B-15) |
| dog_logger | `snapshot_dir` / `enable_snapshots` | `/var/lib/billiebot/snapshots` / true | Snapshot JPEGs on transitions — **currently zero-byte placeholders** (Appendix B-16) |
| daily_report | `generate_hour`/`generate_minute` | 23:55 | Design §3.6 says 23:30 — minor deviation (Appendix B-17) |
| report_server | `host`/`port` | 0.0.0.0 / 8080 | LAN-reachable (SYS-RPT-2) |

### 13.3 Runtime process architecture

Four nodes:

**`state_fusion`** — the SYS-STL-1 core. Timestamped deques of the last 10 s of evidence; 2 Hz fusion tick:

| Interface | Direction | Type |
|---|---|---|
| `/dog/detections_3d` | sub | `DogDetection3D` (visual evidence) |
| `/thermal/blob` | sub | `ThermalBlob` (thermal evidence, `is_dog_candidate` filtered) |
| `/audio/events` | sub | `AudioEvent` (bark/whine counting) |
| `/dog/pose_map` | sub | `PoseStamped` (position memory) |
| `/billie/state` | pub @ 2 Hz | `DogState` — state enum, confidence, position, room, `context[6]`, `stress_proxy`, `state_duration` |
| `/get_dog_state` | service | `GetDogState` (empty → current DogState + `dog_found`) |

Decision cascade (per 10 s window): ≥2 BARKs → **BARKING** (conf 0.5+0.1/bark, cap 0.9); else visual present → if also thermal: mean temp <33 °C & no audio → **SLEEPING**; <36 °C & no audio → **RESTING**; else **ACTIVE**; visual only → **ACTIVE** (conf ×0.8); thermal only → **RESTING** (conf 0.4); nothing → **NOT_FOUND**. A 3 s hysteresis gates all transitions. `context = [n_visual, n_thermal, n_barks, n_whines, bark_rate, stress_proxy]` — the SYS-STL-3/SYS-EXT-3 forward-compatible feature vector. Deviations from design §5.3: no bbox-motion-energy or displacement test for ACTIVE, no 120 s static requirement for SLEEPING (thermal temperature is used as the proxy instead), and `room` is only echoed from input (never computed here — always empty; room attribution actually happens in `dog_logger`).

**`dog_logger`** — subscribes `/billie/state`; writes to SQLite (WAL mode, schema `dog_events(timestamp, epoch, state, state_id, confidence, x, y, room, image_path, context_json, action='OBSERVE', outcome='', stress_proxy)`) **on every state transition plus a 60 s periodic heartbeat row**. Room is derived from position via `rooms.yaml` bounding boxes (first match wins; overlapping placeholder boxes exist — e.g. `hallway` overlaps `living_room`). Snapshot files are created but empty (no image topic subscription).

**`daily_report`** — 60 s check timer; at 23:55 queries the day's rows, aggregates per-state durations (inter-event gaps capped at 300 s), bark log, room counts; renders Jinja2 Markdown to `reports/report_YYYY-MM-DD.md` + matplotlib timeline PNG. Also runnable as `daily_report --standalone [date]` for systemd-timer use (design §5.1).

**`report_server`** — FastAPI in a daemon thread: `GET /` (HTML-rendered latest report), `/latest` (raw Markdown), `/health` (`{"status": "ok", "reports": N}`), `/reports` (list).

**Standalone rung 12** (no perception running): fusion sees no evidence → `/billie/state` publishes **NOT_FOUND at 2 Hz** — still fully verifiable at the interface level, which is exactly what `verify_rung_12.sh` checks.

### 13.4 Measurable outputs

| Output | Measurement | Expected |
|---|---|---|
| Fused state | `ros2 topic hz /billie/state` → 2 Hz; `echo` | Standalone: `state: 0` (NOT_FOUND). With rungs 07/09/11 mock feeding it: transitions among ACTIVE/RESTING/BARKING visible, confidence populated, `context[6]` non-zero, `stress_proxy` rising with mock bark bursts |
| State service | `ros2 service call /get_dog_state billiebot_interfaces/srv/GetDogState` | Returns current state + `dog_found` |
| HTTP health | `curl http://localhost:8080/health` | `{"status":"ok","reports":N}` — the VERIFICATION.md criterion |
| Report retrieval | `curl http://localhost:8080/latest`; browser to `/` | Markdown/HTML report (after ≥1 generation) |
| Persistence | `sqlite3 /var/lib/billiebot/billie_events.db 'SELECT state, COUNT(*) FROM dog_events GROUP BY 1'` | Rows accumulate: transitions + 60 s heartbeats; `PRAGMA journal_mode` returns `wal` (SYS-STL-4 evidence) |
| On-demand report | `ros2 run billiebot_cognition daily_report --standalone` | `report_<date>.md` + `timeline_<date>.png` in the reports dir — avoids waiting for 23:55 |
| Script | `./scripts/verify_rung_12.sh` | state topic + service PASS; server health PASS or WARN (fastapi treated as optional) |

### 13.5 Requirements traceability

| Requirement | Relationship |
|---|---|
| **SYS-STL-1** (fuse visual/thermal/audio → state with confidence, Verify: T) | **Satisfied (heuristic MVP)** — the state set matches {SLEEPING, RESTING, ACTIVE, BARKING, EATING\*, NOT_FOUND} (EATING defined in the message but never produced — the design marks it optional/waypoint-based). TC-12 |
| **SYS-STL-2** (log every transition with t/state/conf/location/snapshot, Verify: T) | **Partial** — all fields logged **except** the snapshot, which is a zero-byte placeholder. TC-20 |
| **SYS-STL-3** (schema forward-compatible with Behavior AI, Verify: I) | **Satisfied** — `context[]` + `stress_proxy` in `DogState.msg`; `context_json`/`action`/`outcome` columns in the DB |
| **SYS-STL-4** (survive power loss; WAL + snapshot fsync, Verify: T) | **Partial** — WAL enabled; no explicit fsync of snapshots (moot while they're empty); the actual pull-the-plug test remains to be run |
| **SYS-RPT-1** (daily summary: durations, barks, timeline, rooms, snapshots, Verify: D) | **Mostly satisfied** — durations/bark log/room counts/timeline PNG present; representative snapshots absent (blocked by B-16); generation time 23:55 vs. design's 23:30 |
| **SYS-RPT-2** (retrievable via local network, Verify: D) | **Satisfied** — HTTP on 0.0.0.0:8080. TC-22 |
| **SYS-EXT-3** ((context, action, outcome) tuples logged, Verify: I) | **Partial** — columns exist and are written, but `action` is hard-coded `'OBSERVE'` and `outcome` always empty; no engagement action ever writes its result (the rung-13 action servers don't touch the logger) |
| **SYS-EXT-4** (`stress_proxy` field, Verify: I) | **Satisfied** — bark-rate heuristic (the design's "retreat motion" term is not included) |
| **TC-14** (GetDogState) | **Satisfied** |

---

## 14. Rung 13 — Mission

```bash
ros2 launch billiebot_bringup 13_mission.launch.py mock:=true
```

### 14.1 Referenced files

| File | Role |
|---|---|
| `billiebot_bringup/launch/13_mission.launch.py` → `billiebot_mission/launch/mission.launch.py` | Starts the five mission nodes with `mission.yaml` |
| `billiebot_mission/config/mission.yaml` | Controller + action-server parameters |
| `billiebot_mission/billiebot_mission/mission_controller.py` | Mode state machine (Python) |
| `billiebot_mission/billiebot_mission/approach_dog_server.py` | `/approach_dog` action → Nav2 |
| `billiebot_mission/billiebot_mission/retreat_server.py` | `/retreat` action → raw `/cmd_vel` |
| `billiebot_mission/billiebot_mission/speak_server.py` | `/mission/speak` → forwards to `/speak` |
| `billiebot_mission/billiebot_mission/dispense_treat_server.py` | `/dispense_treat` stub (always NOT_IMPLEMENTED) |
| `billiebot_interfaces/` `MissionStatus.msg`, `SetMode.srv`, `ApproachDog/Retreat/Speak/DispenseTreat.action` | Interfaces |
| **Present but not launched:** `billiebot_mission/src/{policy_decision,battery_guard,estop_guard}_node.cpp` (+ headers) and `behavior_trees/billiebot_main.xml` | BehaviorTree.CPP nodes and the mission BT — compiled by `CMakeLists.txt` but **no launched process loads the XML or registers the nodes** (Appendix B-5) |
| Related, unconsumed: `billiebot_navigation/config/patrol_waypoints.yaml` | Named waypoint poses — no node reads this file |

### 14.2 Key input parameters

| Node | Parameter | Value | Significance |
|---|---|---|---|
| mission_controller | `patrol_waypoints` | `[living_room, kitchen, bedroom, hallway, bathroom]` | **Names only** — the poses live in `patrol_waypoints.yaml`, which nothing loads; the controller can name a waypoint in `MissionStatus` but cannot navigate to it |
| mission_controller | `battery_safe_voltage` | 10.5 V | SAFE-mode trigger (matches SYS-PLT-2's 3.5 V/cell) |
| mission_controller | `max_nav_failures` | 3 | SYS-NAV-4's "3 failed recoveries → SAFE" — but the failure counter is never incremented (no nav goals are ever sent) |
| mission_controller | `tick_rate_hz` | 2.0 | State-machine tick and `MissionStatus` rate |
| approach_dog_server | `min_standoff` | 1.0 m | **SYS-FND-3's hard floor, enforced at goal acceptance** (`REJECT` if goal < 1.0) |
| approach_dog_server | `max_speed` | 0.15 m/s | SYS-NAV-5 near-dog cap — declared, **not enforced** (never applied to Nav2) |
| retreat_server | `retreat_speed` | 0.1 m/s | Open-loop reverse speed |

### 14.3 Runtime process architecture

**`mission_controller`** (2 Hz tick):

| Interface | Direction | Type | Notes |
|---|---|---|---|
| `/billie/state` | sub | `DogState` | fused state (rung 12) |
| `/battery_state` | sub | `BatteryState` | voltage for SAFE check (defaults 12.6 V if absent) |
| `/dog/found` | sub | `Bool` | PATROL↔TRACK_OBSERVE trigger (dual-publisher stream, B-12) |
| `/audio/events` | sub | `AudioEvent` | BARK during PATROL → **log line only** ("Re-sort patrol towards DoA (SYS-FND-2)" is a comment, not code) |
| `/billiebot/mission_status` | pub @ 2 Hz | `MissionStatus` — mode, current_waypoint, dog state/conf, battery, nav_active, recovery_count, estopped | |
| `/set_mode` | service | `SetMode` (mode 0–5 → success, previous_mode) | The only way to leave IDLE |
| `navigate_to_pose` | action **client** | `NavigateToPose` | **Created, never used** — `tick()` sends no goals |

Mode logic implemented: battery < 10.5 → SAFE; `_estopped` → SAFE (but `_estopped` has no input — never wired to `/e_stop` or `MissionStatus`); nav-failures ≥ 3 → SAFE (counter frozen at 0); PATROL + dog_found → TRACK_OBSERVE; TRACK_OBSERVE + !dog_found → PATROL. Startup mode: **IDLE** — an operator `/set_mode` call to PATROL is required. INVESTIGATE and RETURN are reachable *only* via `/set_mode`, with no behavior attached.

**Action servers** (the SYS-EXT-1 engagement-primitive surface):

| Server | Action name | Behavior | Physical effect |
|---|---|---|---|
| `approach_dog_server` | `/approach_dog` | Rejects standoff < 1.0 m; takes last `/dog/pose_map`; computes goal at `(dog_x − standoff, dog_y)` (**x-axis-only geometry** — not along the robot→dog line, Appendix B-18); forwards to Nav2 `navigate_to_pose`; result reports the *commanded* standoff as `final_distance`, not measured | Robot drives toward the dog, stops ≥1 m away (needs rungs 06+08 live) |
| `retreat_server` | `/retreat` | Time-based open-loop reverse: publishes `Twist(linear.x = −0.1)` at 10 Hz for `distance/0.1` s, then zero | Robot backs up — **bypasses Nav2 entirely: no rear obstacle sensing** (Appendix B-19) |
| `speak_server` | `/mission/speak` | Thin forwarder to the speaker's `/speak` (needs rung 11); surfaces rate-limit rejections | Sound plays |
| `dispense_treat_server` | `/dispense_treat` | Always aborts with "NOT_IMPLEMENTED — treat dispenser hardware not installed" | none (stub per SYS-EXT-1/EXT-5) |

### 14.4 Measurable outputs

| Output | Measurement | Expected |
|---|---|---|
| Mission status | `ros2 topic hz /billiebot/mission_status` → 2 Hz; `echo` | `mode: 0` (IDLE) at startup; battery 12.6 (default) unless rung 02 feeds it — the VERIFICATION.md criterion |
| Mode service | `ros2 service call /set_mode billiebot_interfaces/srv/SetMode "{mode: 1}"` | `success: true, previous_mode: 0`; status now PATROL |
| Action inventory | `ros2 action list` | `/approach_dog`, `/retreat`, `/dispense_treat`, `/mission/speak` — note VERIFICATION.md lists the first three; `/mission/speak` is the fourth (its name differs from the design's `/speak`, which belongs to rung 11) |
| Standoff floor | `ros2 action send_goal /approach_dog billiebot_interfaces/action/ApproachDog "{standoff_distance: 0.5}"` | **Goal rejected** — direct SYS-FND-3 evidence |
| Treat stub | `send_goal /dispense_treat … "{quantity: 1}"` | Aborts with NOT_IMPLEMENTED — SYS-EXT-1 evidence |
| Retreat (physical, real) | `send_goal /retreat … "{retreat_distance: 0.5}"` | Robot reverses ~0.5 m at 0.1 m/s; feedback `distance_so_far` ramps |
| Mode transitions (integration) | Run with rungs 07+12 mock; set PATROL | Status oscillates PATROL↔TRACK_OBSERVE following mock `/dog/found` (flapping expected — no hysteresis in the controller) |
| SAFE trigger | (Not reachable in mock — battery constant 12.6 V.) Real: discharge below 10.5 V, or publish a synthetic low `BatteryState` | mode → SAFE with warning log |

### 14.5 Requirements traceability

| Requirement | Relationship |
|---|---|
| **SYS-EXT-1** (engagement primitives as uniform action servers, Verify: I) | **Satisfied** — all four exist with goal/feedback/result contracts, including the mandated DispenseTreat stub. TC-13/TC-15 |
| **SYS-EXT-2** (BT mission logic with `PolicyDecision` extension point, Verify: I) | **Partial** — `PolicyDecisionNode` (returns OBSERVE for `ObserveOnlyPolicy`), `BatteryGuard`, `EStopGuard`, and `billiebot_main.xml` (which encodes patrol→detect→policy→approach/retreat/speak/observe and the safety guards) all exist and compile, **but the runtime uses the Python `mission_controller` instead — the BT is dead code**. The design's satisfy relationship is documented but not demonstrated |
| **SYS-FND-3** (approach & hold 1.0–2.0 m standoff, never < 1.0 m, Verify: T) | **Partial** — the 1.0 m floor is enforced (TC-17 evidence); "hold" behavior and correct approach geometry are not implemented |
| **SYS-FND-1/2** (priority search; DoA re-prioritization) | **Not implemented** — waypoint queue, priority ordering, and DoA re-sorting are absent (DoA is logged only) |
| **SYS-NAV-4** (3 failures → SAFE + alert) | **Partial** — SAFE transition coded; counter never increments; no operator alert |
| **SYS-NAV-5** (0.15 m/s near dog) | **Declared** (`max_speed` param, action field) but unenforced |
| **SYS-NAV-6** (patrol route over waypoints, Verify: D) | **Not satisfied** — no navigation is commanded; `patrol_waypoints.yaml` unconsumed. TC-16 has no automated path |
| **SYS-PLT-2** (SAFE at 3.5 V/cell) | **Partial** — threshold + SAFE mode present; "alert, request pickup" absent |
| **SYS-PLT-5** | Adjacent — `estopped` field exists in status but is never driven by the actual `/e_stop` service state |
| **STM-01** (design mode machine) | Modes and enum match `MissionStatus.msg` exactly; transition coverage is ~half (PATROL↔TRACK_OBSERVE + SAFE entries) |

---

## 15. Rung 14 — Full Bringup

```bash
ros2 launch billiebot_bringup 14_full_bringup.launch.py mock:=true
```

### 15.1 Referenced files

`billiebot_bringup/launch/14_full_bringup.launch.py` includes, in order: `06_nav2` (→ 05 → {01, 03 → 02} + navigation) with `mock` + `map` forwarded, then `07_oakd`, `08_dog_locator`, `09_thermal`, `10_noir`, `11_audio`, `12_cognition`, `13_mission` — i.e., **every file listed in rungs 01–13**. The mock test suite `billiebot_tests/scripts/run_all_mock_tests.sh` is designed to run against this rung. The deployment variants `jetson.launch.py` (06+07+08+13 with `CYCLONEDDS_URI`) and `pi.launch.py` (09+10+11+12 with `CYCLONEDDS_URI`) split this same set across the two computers.

### 15.2 Key input parameters

Only two: `mock` (fanned out to every rung that accepts it — 08 and 12 take none) and `map` (fanned to the localization chain; default `''`, so **full bringup without a map argument leaves the entire localization/navigation side inactive** even on real hardware).

### 15.3 Runtime process architecture

Real-hardware node census (27 processes):

| Subsystem | Nodes |
|---|---|
| Drive & description | `robot_state_publisher`, `base_bridge` |
| Localization | `ekf_filter_node` **×2** (B-4), `map_server`, `amcl`, `lifecycle_manager_localization` |
| Lidar | `rplidar_node` (mock: `mock_lidar_stub` = extra base_bridge, B-1/B-2) |
| Nav2 | `controller_server`, `planner_server`, `behavior_server`, `bt_navigator`, `lifecycle_manager_navigation` |
| Perception | `oakd_dog_detector`, `dog_locator`, `thermal_node`, `noir_cam_node` |
| Audio | `audio_classifier`, `speaker_node` |
| Cognition | `state_fusion`, `dog_logger`, `daily_report`, `report_server` |
| Mission | `mission_controller`, `approach_dog_server`, `retreat_server`, `speak_server`, `dispense_treat_server` |

**The intended end-to-end data flow** (design ACT-01/ACT-02 realized):

```
/scan ──► slam/amcl ──► map→odom TF ─┐
/cmd_vel ◄── Nav2 ◄── navigate_to_pose◄─┐                    (mobility loop)
   │                                    │
base_bridge ──► /odom ──► EKF ──► TF ───┤
                                        │
OAK-D ──► /dog/detections_3d ──┬──► dog_locator ──► /dog/pose_map ─┐
                               │         (needs TF)                │
MLX90640 ──► /thermal/blob ────┤                                   │
ReSpeaker ──► /audio/events ───┴──────────► state_fusion ◄─────────┘
                                                 │
                              /billie/state (2 Hz, DogState)
                                    │                │
                             dog_logger          mission_controller ──► /billiebot/mission_status
                              (SQLite)               │ (mode machine; Nav2 client unused)
                                    │            action servers: /approach_dog /retreat
                             daily_report                         /mission/speak /dispense_treat
                                    │
                             report_server ──► http://<pi>:8080/
```

**What actually closes in mock mode:** the perception→fusion→logging→reporting chain and the mission status/mode surface are fully live; the mobility loop is dead (no `/scan`, no map). The duplicate-node pathologies (two base_bridges, two EKFs, triple `odom→base_link` TF broadcasters) are all present simultaneously. **This is the rung where Appendix B items compound.**

Message-rate budget at steady state (mock): `/odom` + `/joint_states` + TF ≈ 30 Hz ×2 instances, `/dog/found` 5 Hz, `/dog/detections_3d` ~3.5 Hz, `/thermal/image`+`/thermal/blob` 4 Hz each, `/noir/image` 5 Hz (~4.6 MB/s — the single largest bandwidth consumer, with zero subscribers), `/audio/events` ~0.4 Hz, `/billie/state` 2 Hz, `/billiebot/mission_status` 2 Hz.

### 15.4 Measurable outputs

| Output | Measurement | Expected |
|---|---|---|
| **Mock test suite** | `./billiebot_ws/src/billiebot_tests/scripts/run_all_mock_tests.sh` (terminal 2) | 17 checks / TC-01…TC-15: interface show, `/robot_description`, `/odom`, `/joint_states`, `/battery_state`, `/e_stop`, `/dog/detections_3d`, `/dog/pose_map`, `/thermal/image`, `/thermal/blob`, `/audio/events`, `/billie/state`, `/billiebot/mission_status`, `/get_dog_state`, `/set_mode` → "ALL TESTS PASSED", exit 0. **Caveat:** `/dog/pose_map` (TC-08) requires TF that mock cannot provide without a map — expect this check to be the marginal one. Note the suite checks *existence only* (`ros2 topic info`), not rates or content, and omits `/scan`, `/noir/image`, costmaps, and Nav2 actions entirely |
| End-to-end cognition | Watch `/billie/state` while mock perception runs; then `sqlite3 … 'SELECT * FROM dog_events ORDER BY id DESC LIMIT 10'`; then `curl :8080/health` | State transitions driven by synthetic detections/barks → DB rows → report pipeline: the full STK-5/STK-6 thread demonstrated without hardware |
| Full-stack real demo | Map arg + hardware: `/set_mode 1`, send `navigate_to_pose` goals | Robot physically drives; detections localize the dog on the map; logger records; report served — the Build Plan Phase 5/6 rehearsal |
| Multi-machine | `jetson.launch.py` + `pi.launch.py` with updated `cyclonedds.xml` IPs | `ros2 topic list` identical on both hosts (DDS discovery over unicast peers) |

### 15.5 Requirements traceability

Rung 14 is the **integration verification platform** rather than a requirement-holder: every SYS-* mapping from rungs 01–13 applies here in composition. Additionally:

| Requirement | Relationship |
|---|---|
| **SYS-PLT-4** (operator teleop/viz/waypoints/e-stop over Wi-Fi, Verify: D) | **Partial** — the DDS + cyclonedds.xml setup makes all topics visible to a host running RViz/Foxglove, and `/e_stop` + `/cmd_vel` are callable remotely; but no rung launches a teleop node, RViz config, or Foxglove bridge |
| **SYS-PLT-1** (60 min endurance) | Only measurable on this rung (full load) with the physical battery — no software support/test exists |
| **STK-1…STK-6** | The mock full-bringup demonstrates the complete software thread for STK-2…STK-6 (find→classify→log→report) minus real sensing; STK-1 (navigate without getting stuck) requires real hardware + map |
| Build Plan Phase 6 ("24 h soak") | This rung is the configuration under test |

---

## Appendix A — Requirements Coverage Matrix

Legend: ● satisfied by the bringup ladder (mechanism present and verifiable) · ◐ partial (mechanism present with gaps noted) · ○ enabler only / prerequisite · — no bringup coverage (needs hardware test, physical inspection, or unimplemented software).

| Requirement | Primary rung(s) | Coverage | Notes / gap |
|---|---|---|---|
| SYS-NAV-1 (SLAM map) | 04 | ● | Real lidar required; no automated map-quality check |
| SYS-NAV-2 (≤0.15 m localization) | 05 (03 supporting) | ◐ | Mechanism complete; quantitative T-test procedure not scripted |
| SYS-NAV-3 (collision-free + replan) | 06 | ● | Demonstration on hardware; dog not fed into costmaps |
| SYS-NAV-4 (stuck → recover → SAFE) | 06 + 13 | ◐ | Recoveries + progress checker exist (10 s vs. spec 5 s); mission failure counter never increments; no operator alert |
| SYS-NAV-5 (0.3 / 0.15 near dog) | 06 (13) | ◐ | 0.3 cap in DWB ●; near-dog 0.15 slowdown unimplemented — |
| SYS-NAV-6 (patrol waypoints) | 06 + 13 | ○ | `navigate_through_poses` exists; nothing consumes `patrol_waypoints.yaml`; no patrol executor |
| SYS-FND-1 (priority search) | — | — | Not implemented |
| SYS-FND-2 (DoA re-prioritization) | 11 + 13 | ◐→— | DoA produced and received; re-sorting is a log statement |
| SYS-FND-3 (1.0–2.0 m standoff) | 13 | ◐ | 1.0 m floor enforced at goal acceptance; hold + correct geometry missing |
| SYS-FND-4 (find ≤10 min, 80 %) | — | — | Acceptance trial only (Build Phase 5) |
| SYS-PER-1 (RGB dog ≥5 FPS) | 07 | ◐ | 5 Hz pipeline ●; recall test on real dog outstanding; real mode needs `model_path` |
| SYS-PER-2 (3-D position ±0.2 m) | 07 + 08 | ◐ | Full camera→map chain; accuracy test outstanding; B-13 latent TF-import risk |
| SYS-PER-3 (thermal blob) | 09 | ● | Thresholds literal (30–40 °C, ≥8 px); range/darkness demo outstanding |
| SYS-PER-4 (audio class + DoA) | 11 | ◐ | Pipeline ●; recall test set outstanding; XVF3800 VID/PID risk (B-14) |
| SYS-PER-5 (NoIR low-light detect) | 10 | ○ | Imagery only; no detector, no lux switch, no IR illuminator (design-acknowledged gap) |
| SYS-STL-1 (state fusion) | 12 | ● | Heuristic per design §5.3 (simplified: no motion-energy term) |
| SYS-STL-2 (SQLite event log) | 12 | ◐ | All fields except real snapshots (B-16) |
| SYS-STL-3 (forward-compatible schema) | 12 | ● | `context[]`, `stress_proxy`, DB columns |
| SYS-STL-4 (power-loss safety) | 12 | ◐ | WAL ●; snapshot fsync n/a; pull-plug test outstanding |
| SYS-RPT-1 (daily summary) | 12 | ◐ | Durations/barks/rooms/timeline ●; snapshots absent; 23:55 vs. 23:30 |
| SYS-RPT-2 (network retrieval) | 12 | ● | FastAPI on 0.0.0.0:8080 |
| SYS-PLT-1 (60 min endurance) | (14) | — | Physical test only |
| SYS-PLT-2 (battery SAFE @3.5 V/cell) | 02 + 13 | ◐ | Monitoring + threshold + SAFE transition ●; alert/pickup request —; untestable in mock (constant 12.6 V) |
| SYS-PLT-3 (fusing/separation) | — | — | Hardware inspection (design §4.2) |
| SYS-PLT-4 (teleop/viz/e-stop from host) | 02 + 14 | ◐ | Interfaces exist and are DDS-reachable; no teleop/viz launch provided |
| SYS-PLT-5 (e-stop 200 ms; heartbeat 500 ms) | 02 (+ firmware) | ◐ | Service path ●; timing verification requires firmware + instrumented test (`firmware/README.md` documents the 500 ms `AUTO_STOP_INTERVAL`) |
| SYS-PLT-6 (no sudden sounds; rate limit) | 11 | ● | 10 s interval + 0.5 volume clamp, inspectable and testable |
| SYS-EXT-1 (uniform action servers) | 13 | ● | ApproachDog/Retreat/Speak/DispenseTreat-stub all present |
| SYS-EXT-2 (BT + PolicyDecision point) | 13 | ◐ | BT nodes + XML exist but are not executed at runtime (B-5) |
| SYS-EXT-3 ((context, action, outcome) log) | 12 | ◐ | Columns written; action/outcome never populated by real actions |
| SYS-EXT-4 (stress_proxy) | 12 | ● | Bark-rate heuristic (retreat-motion term omitted) |
| SYS-EXT-5 (power reserve for dispenser) | — | — | Hardware provision (design §4.2 reserved rail) |

**Requirements with zero bringup-ladder coverage:** SYS-FND-1, SYS-FND-4, SYS-PLT-1, SYS-PLT-3, SYS-EXT-5 — all are either physical-inspection items or acceptance trials, **except SYS-FND-1 (search priority logic), which is a pure software gap**.

---

## Appendix B — Discrepancy & Risk Register

Findings from this decomposition, ordered by systems impact. "Latent" = predicted from source reading, not yet observed at runtime.

| # | Finding | Evidence | Impact | Suggested disposition |
|---|---|---|---|---|
| B-1 | **Mock lidar publishes no `/scan`.** Rung 01's mock branch launches `base_bridge` as `mock_lidar_stub`; the launch file's own comment calls it a placeholder | `01_lidar.launch.py` (mock Node block); `base_bridge.py` has no LaserScan publisher | Rung 01's verify criterion unfalsifiable in mock; rungs 04/05/06/14 mock have no SLAM/costmap data; `verify_rung_01.sh` fails by construction | Write a ~30-line mock scan publisher (synthetic rectangular room) — unblocks the whole mock nav chain |
| B-2 | **Duplicate base_bridge in mock composition.** Rungs 04/05/06/14 include both `01_lidar` (mock ⇒ stub base_bridge) and `03→02` (real base_bridge): two publishers on `/odom`, `/joint_states`, `/battery_state`, two `/e_stop` servers (TF flapping no longer occurs since B-3's fix — neither instance broadcasts `odom→base_link` by default) | Launch inclusion graph §1.2 | Interleaved odometry from two integrators; nondeterministic e-stop behavior | Falls out automatically if B-1 is fixed |
| B-3 | **Resolved 2026-07-17 (GAP-5).** ~~`odom→base_link` TF multi-broadcast~~ — `base_driver.yaml` now defaults `publish_tf: false` (EKF sole owner); `publish_tf:=true` launch arg restores the base broadcast for rung-02-only bench work | Both config files; `base.launch.py` / `02_base.launch.py` | (historical) raw vs. filtered pose divergence → TF consumers see jumps | Done — see `DISCREPANCY_RESOLUTION_PLAN.md` §GAP-5 |
| B-4 | **Resolved 2026-07-17 (GAP-6).** ~~Second `ekf_filter_node` in rung 06~~ — the EKF block was removed from `navigation.launch.py`; rung 03's `ekf_filter_node` is the single instance | `navigation.launch.py` | (historical) duplicate node name; two `/odometry/filtered` publishers; `odom→base_link` contention | Done — see `DISCREPANCY_RESOLUTION_PLAN.md` §GAP-6 |
| B-5 | **Mission logic is a shell.** `mission_controller` never sends Nav2 goals, never advances `_current_wp_idx`, never increments `_nav_failure_count`, never sets `_estopped` from `/e_stop`, and only logs DoA. The compiled BT (`billiebot_main.xml` + PolicyDecision/BatteryGuard/EStopGuard) — which *does* encode patrol/policy/safety — is loaded by no process | `mission_controller.py` `tick()`; `mission.launch.py` (no BT executor) | SYS-NAV-4/6, SYS-FND-1/2, SYS-EXT-2 unverifiable end-to-end; TC-16/TC-19 blocked | Decide: finish the Python controller (load `patrol_waypoints.yaml`, drive Nav2) or stand up the BT executor the design intended |
| B-6 | **Empty `map` default silently disables localization.** `map:=''` → `map_server` lifecycle configure fails; lifecycle manager can't activate; rungs 05/06/14 come up "green-ish" with no map frame | `05_amcl.launch.py` default; `localization.launch.py` | Confusing partial bringup; downstream TF-dependent nodes just stay silent | Fail loudly (launch-time assertion) or document a bundled test map |
| B-7 | **SYS-NAV-4/5 parameter drift.** Progress checker = 0.5 m/10 s vs. spec ">5 s"; near-dog 0.15 m/s speed zone absent (no speed-filter/keepout layer) | `nav2_params.yaml` | Spec-to-config mismatch discoverable only in test | Tune `movement_time_allowance`; add Nav2 speed-filter mask fed by `/dog/pose_map` |
| B-8 | **Resolved 2026-07-17 (GAP-4).** ~~bt_navigator uses raw `/odom`~~ — `odom_topic: /odometry/filtered` set for bt_navigator *and* controller_server (the latter had defaulted to raw odom) | `nav2_params.yaml` | (historical) EKF value partially bypassed | Done — see `DISCREPANCY_RESOLUTION_PLAN.md` §GAP-4 |
| B-9 | **Serial-port fragility.** Lidar on raw `/dev/ttyUSB1` while Arduino uses a by-id path — USB enumeration order can swap devices | `01_lidar.launch.py`; `base_driver.yaml` | Boot-order-dependent bringup failures on the Jetson | Use `/dev/serial/by-id/` for the lidar too, or udev rules |
| B-10 | **`perception.yaml` never loaded by the ladder.** Rungs 07/09/10 pass only `mock`; the yaml is loaded only by `perception.launch.py`, which no rung uses. Defaults currently equal the yaml, so behavior matches — until someone edits the yaml and nothing changes | Rung launch files vs. `perception.launch.py` | Silent config drift trap | Point rungs 07/09/10 at the yaml (as rung 11 already does for audio) |
| B-11 | **Real-mode perception needs unset parameters and has a bbox bug.** `model_path` defaults `''` in both `oakd_dog_detector` and `audio_classifier` → real mode = error log + zero output. Additionally DepthAI `det.xmin/ymin/xmax/ymax` are normalized floats; `int()` of them zeroes the bbox fields (position/depth unaffected) | `oakd_dog_detector.py` `init_depthai_pipeline`/`real_detect`; `audio_classifier.py` | First hardware run of rungs 07/11 will silently produce nothing; bbox fields useless for downstream motion-energy features (design §5.3 ACTIVE heuristic) | Provide model artifacts + set params in yaml; scale bbox by preview size |
| B-12 | **`/dog/found` has two publishers** (oakd_dog_detector true/false @5 Hz; dog_locator true-only per pose) | Both node sources | `mission_controller` sees an interleaved stream; PATROL↔TRACK_OBSERVE flapping in mock (no hysteresis in mission) | Single ownership (locator), or debounce in mission |
| B-13 | *(Latent)* **`dog_locator` may throw on every detection**: `tf2_geometry_msgs` is never imported, but `Buffer.transform(PointStamped, …)` requires its type registration; the resulting error is not a `TransformException` and is uncaught | `dog_locator.py` imports | If the registration isn't pulled in transitively, `/dog/pose_map` never publishes even with perfect TF — blocks SYS-PER-2, TC-08 | Add `import tf2_geometry_msgs`; verify on first integrated run |
| B-14 | *(Latent)* **DoA queries target the wrong USB ID for the specified hardware**: code uses VID 0x2886/PID 0x0018 (ReSpeaker 4-Mic, XVF-3000 family); the design BOM says **XVF3800** | `audio_classifier.py` `_get_doa` | DoA silently 0.0° → SYS-PER-4's ±15° DoA and SYS-FND-2 unverifiable | Confirm enumeration on hardware; use the XVF3800 host API per design §5.1 |
| B-15 | **`/var/lib/billiebot` permissions.** `dog_logger` (and report nodes) `os.makedirs` under `/var/lib` — crashes with `PermissionError` for a non-root user unless the directory is pre-provisioned | `dog_logger.py` `__init__` | Rung 12 fails at startup on a fresh machine | Provision dir in setup docs/systemd, or default to a user path |
| B-16 | **Snapshots are zero-byte placeholders** — `_capture_snapshot` writes `b''`; no image topic is subscribed | `dog_logger.py` | SYS-STL-2's "image snapshot reference" and SYS-RPT-1's "representative snapshots" reference empty files | Subscribe `/noir/image` or an OAK preview and JPEG-encode (also gives `/noir/image` its first consumer) |
| B-17 | **Report time 23:55 vs. design 23:30**; `daily_report` also only fires if the node is alive at that exact minute (60 s poll) | `cognition.yaml`; design §3.6 ACT-03 | Minor spec deviation; missed generation if node restarts at 23:55 | Align config; or systemd timer per design |
| B-18 | **Approach geometry is x-axis-only**: goal = `(dog_x − standoff, dog_y)` regardless of robot bearing; result reports commanded standoff as measured | `approach_dog_server.py` | Approach from the "west" only; standoff distance unverified in result | Compute along robot→dog vector; measure actual final range |
| B-19 | **`retreat_server` bypasses Nav2** — open-loop reverse `/cmd_vel` with no rear sensing (lidar sees 360° but isn't consulted) | `retreat_server.py` | Collision risk during retreat; contradicts "safety rails below the policy" intent (§5.4) | Use Nav2 backup behavior (already running in rung 06's behavior server) |
| B-20 | **Verification depth is existence-only.** All verify scripts and `run_all_mock_tests.sh` check topic/service *presence*; none assert rates, field ranges, TF correctness, or end-to-end latency; TC-16…TC-22 have no scripts at all | `scripts/*.sh`, `run_all_mock_tests.sh` | "PASS" ≠ functioning system | Add `ros2 topic hz` thresholds and content assertions; script TC-16…TC-22 |

---

## Appendix C — Acceptance-Test ↔ Rung ↔ Requirement Cross-Reference

From the TC table in `docs/VERIFICATION.md`, with the rung that produces the evidence and current automation status. `run_all_mock_tests.sh` covers **TC-01…TC-15 only**, all as existence checks.

| TC | Description | Traces to | Evidence rung | Automated? |
|---|---|---|---|---|
| TC-01 | Interface definitions build | Phase 1A | build + 14 | ✔ mock suite (`ros2 interface show DogState`) |
| TC-02 | URDF validity | SYS-NAV-1 | 02 | ✔ mock suite (`/robot_description`) |
| TC-03 | Odometry publishing | SYS-NAV-2 | 02 | ✔ mock suite |
| TC-04 | Joint states | Phase 1C | 02 | ✔ mock suite |
| TC-05 | Battery monitoring | SYS-PLT-2 | 02 | ✔ mock suite (existence; not thresholds) |
| TC-06 | E-stop service | SYS-PLT-5 | 02 | ✔ mock suite (existence; not stop latency) |
| TC-07 | Dog 3D detection | SYS-PER-1/2 | 07 | ✔ mock suite + `verify_rung_07.sh` |
| TC-08 | Dog locator TF | SYS-PER-2 | 08 | ✔ mock suite — but needs TF the mock stack can't provide (B-1/B-13) |
| TC-09 | Thermal imaging | SYS-PER-3 | 09 | ✔ mock suite |
| TC-10 | Thermal blob detection | SYS-PER-3 | 09 | ✔ mock suite |
| TC-11 | Audio classification | SYS-PER-4 | 11 | ✔ mock suite (existence; not recall/DoA) |
| TC-12 | State fusion | SYS-STL-1 | 12 | ✔ mock suite + `verify_rung_12.sh` |
| TC-13 | Mission status | Phase 6 | 13 | ✔ mock suite |
| TC-14 | GetDogState service | SYS-STL-1 | 12 | ✔ mock suite |
| TC-15 | SetMode service | Phase 6 | 13 | ✔ mock suite |
| TC-16 | Waypoint navigation | SYS-NAV-3/6 | 06 (+13) | ✘ manual only; blocked from full automation by B-5 |
| TC-17 | Standoff distance | SYS-FND-3 | 13 | ✘ manual (`action send_goal` rejection test is scriptable) |
| TC-18 | Speed limiting | SYS-NAV-5 | 06 | ✘ manual (`/cmd_vel` monitor is scriptable); near-dog half unimplemented (B-7) |
| TC-19 | Stuck recovery | SYS-NAV-4 | 06 (+13) | ✘ physical test; SAFE escalation blocked by B-5 |
| TC-20 | SQLite logging | SYS-STL-2/4 | 12 | ✘ manual (`sqlite3` query is scriptable) |
| TC-21 | Daily report | SYS-RPT-1 | 12 | ✘ manual (`daily_report --standalone` is scriptable) |
| TC-22 | Report server | SYS-RPT-2 | 12 | ◐ `verify_rung_12.sh` curls `/health` (WARN-only, non-fatal) |

**Summary judgment:** the ladder is a well-ordered dependency chain whose lower rungs (02, 03, 09, 11, 12) are genuinely verifiable today, whose middle rungs (04–06) are real-hardware-only until the mock scan gap (B-1) is closed, and whose top rungs (13–14) verify *interfaces* while the mission *behavior* they imply (patrol, search, standoff-hold, SAFE escalation) remains the largest open implementation front (B-5). The requirements architecture — action-server primitives, context-vector state messages, (context, action, outcome) logging — is faithfully carried through the code, so the Behavior-AI insertion path (SYS-EXT-*) survives decomposition intact.

---

*End of report. Generated by static analysis; items marked (inference) or (latent) should be confirmed on hardware.*



