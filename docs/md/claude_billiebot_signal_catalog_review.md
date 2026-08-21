# BillieBot Signal Catalog — Review

Companion document to `docs/architecture/billiebot_signal_catalog.csv`.

**Scope of review:** the entire `sean-mackenzie/billie-bot-claude` repository — 11 ROS 2 packages,
the Sensor Nano firmware, the reference drivetrain firmware, all launch and configuration files,
the URDF/xacro description, the bench test suite, and all Markdown documentation under `docs/md/`.

**Method:** code search and cross-referencing (publisher/subscriber/service/action/TF-broadcaster
audit, parameter and YAML sweep, firmware timing extraction, pin-assignment trace) rather than
reliance on the architecture documents. Where the documents and the code disagree, the code wins
and the disagreement is recorded.

**No production source code was modified.** The repository was cloned read-only; the only files
written are the two artefacts in `docs/architecture/`.

---

## 1. Catalog statistics

| Measure | Count |
|---|---|
| **Unique signals** (`SignalID`) | **211** |
| **Sender → receiver flow rows** | **299** |

### By category (unique signals)

| Category | Unique signals |
|---|---|
| ROS | 80 |
| Software Data | 47 |
| Digital Electrical | 32 |
| Mechanical | 18 |
| Optical | 7 |
| Power/Energy | 6 |
| Network | 5 |
| Analog Electrical | 4 |
| Environmental | 4 |
| Acoustic | 3 |
| Human/Operator | 3 |
| Thermal | 2 |

Electrical signals total **36** across the Digital Electrical and Analog Electrical categories.
Environmental sensing spans **4** entries in the Environmental category plus the **2** Thermal
radiance entries, which are categorised as Thermal because they are radiative rather than contact
measurements.

### By evidence and confidence

| Evidence type | Unique signals | | Confidence | Unique signals |
|---|---|---|---|---|
| Source Code | 112 | | High | 147 |
| Documentation | 36 | | Medium | 58 |
| Firmware | 22 | | Low | 6 |
| **Inferred** | **20** | | | |
| Configuration | 18 | | | |
| Test | 4 | | | |
| Hardware Definition | 2 | | | |

### By implementation status

| Status | Unique signals |
|---|---|
| Production | 146 |
| Test/Bench Only | 47 |
| Documentation Only | 8 |
| Planned | 6 |
| Unclear | 4 |
| Legacy | 3 |

### By boundary and SysML representation

| Direction / boundary | Unique | | Suggested SysML form | Unique |
|---|---|---|---|---|
| Internal | 175 | | Signal | 103 |
| Inbound (External-to-BillieBot) | 25 | | Physical Interaction | 48 |
| Outbound (BillieBot-to-External) | 13 | | ItemFlow | 26 |
| | | | Value/Data | 21 |
| | | | Energy Flow | 9 |
| | | | FlowProperty | 4 |

Two signals appear under more than one direction because the same logical signal crosses the
boundary differently depending on the sender: `/cmd_vel` is Internal from Nav2 and Inbound from
operator teleoperation, and `/set_mode` is Inbound from the operator and Internal from the bench
stimulus node. That is intentional and is why `Direction` is a per-flow rather than per-signal
property.

---

## 2. Major signal chains

Each stage below is a separate catalog row. The chains were deliberately not collapsed.

### Lidar → navigation
`Lidar Laser Emission` → `Lidar Range Return` → `Lidar USB-UART Link` → `/scan` →
`slam_toolbox` **or** `amcl` (never both) → `map→odom` on `/tf` → Nav2 costmaps →
`/plan` → `/cmd_vel`.
The mock branch substitutes `mock_scan` at a configured 10 Hz so AMCL can publish `map→odom`
without hardware.

### Camera → perception
`Visible Scene Radiance` / `Dog Reflected Visible Radiance` / `Stereo Scene Radiance` →
`OAK-D USB3 Device Link` → `Spatial Detection Coordinates` → `Normalised Bounding Box` →
`/dog/detections_3d` → `dog_locator` (tf2 transform) → `/dog/pose_map` → `state_fusion` and
`approach_dog_server`.
Inference runs on the OAK-D's RVC2, so the detection itself never crosses the USB link as pixels.
The `OAK-D Pipeline and Model Upload` signal captures the reverse direction: pipeline graph and
YOLOv8n blob pushed to the device at start-up.

### Thermal sensing → detection
`Dog Body Thermal IR Radiance` / `Scene Thermal IR Radiance` → `MLX90640 Thermal Frame Data`
(8 Hz sensor refresh) → `Thermal Temperature Frame Array` → `/thermal/image` (4 Hz) and
`Warm Blob Statistics` → `/thermal/blob` → `state_fusion`.
The 8 Hz vs 4 Hz split is deliberate and is one of the clearest cases in the system where sensor
sampling rate and ROS publication rate must not be conflated.

### Microphone → audio processing
`Dog Bark Acoustic Pressure` / `Ambient Acoustic Pressure` → `ReSpeaker USB Audio Stream`
(16 kHz sample rate) → `Audio Ring Buffer Waveform` → `Audio Energy Level` gate →
`YAMNet Class Scores` → `/audio/events` → `state_fusion` and `mission_controller`.
`XVF3800 Direction of Arrival Reading` joins the chain over a separate USB control transfer, read
once per published event. The reverse acoustic path is
`/mission/speak` → `/speak` → `I2S Audio Output Stream` → `Amplified Speaker Drive` →
`Speaker Acoustic Output`.

### IMU / barometer → state estimation
`Chassis Specific Force` and `Chassis Angular Rate` and `Earth Magnetic Field` →
`BNO055 Orientation Register Data` (50 Hz) → `IMU Telemetry Record` ('I') →
`Quaternion Frame Convention Transform` → `/imu/data` → `ekf_filter_node`.
**This chain terminates in bench configuration only.** `billiebot_navigation/config/ekf.yaml` has
the entire `imu0` block commented out (BLK-04), so in production the EKF fuses wheel odometry
alone. The barometer branch (`/barometer/pressure`, `/barometer/temperature`) and the magnetometer
branch (`/imu/mag`) have no live subscriber at all.

### Battery → safety logic
`Battery Pack DC Power` → `Battery Divider Sense Voltage` → ADC → `Battery ADC Count Reply`
(Motor Nano path) **or** `Battery ADC Telemetry Record` ('B', Sensor Nano path) →
`Battery Voltage Estimate` → `Battery Health Classification` → `/battery_state` →
`mission_controller` → SAFE mode → `/billiebot/mission_status`.
Two independent acquisition paths exist for the same topic; see §3.

### Navigation command → drivetrain → robot motion
`/cmd_vel` → `Target Wheel Angular Velocity` → `Wheel Velocity Command Frame` ('m L R', 30 Hz) →
`Motor Nano Command Parse` → `Motor Nano PID Duty Output` → PWM lines and enables →
`Left/Right Motor Drive Voltage and Current` → `Motor Electromagnetic Torque` →
`Wheel Shaft Torque` → `Wheel Angular Velocity (Physical)` → `Wheel Traction Force` /
`Wheel Ground Reaction Force` → `Chassis Body Motion`.

### Encoder → odometry
`Encoder Shaft Rotation` → `Encoder Channel A` / `Channel B` (four separate electrical signals) →
`Motor Nano Encoder Count` → `Encoder Count Reply` ('e' reply, 30 Hz) → `Encoder Tick Delta` →
`Wheel Angular Velocity Estimate` → `/joint_states`, and → `Integrated Odometric Pose` →
`/odom` → `ekf_filter_node` → `/odometry/filtered` → Nav2.

### Perception → cognition → report
`/dog/detections_3d` ∥ `/thermal/blob` ∥ `/audio/events` ∥ `/dog/pose_map` →
`Fused Evidence Window` → `/billie/state` (with `Behavior AI Context Vector` and
`Dog Stress Proxy`) → `dog_logger` → `Dog Event Record` → SQLite →
`Daily Activity Summary` → `Daily Report HTTP Response`.

---

## 3. Unresolved questions

These could not be settled from the repository and were **not** silently resolved by guessing.

### 3.1 The Motor Nano firmware is not in this repository

The single largest evidence gap. `firmware/README.md` documents a required change to the
ROSArduinoBridge sketch, and `reference_my_bot/` contains that sketch — but the README states the
reference directory "is kept for reference and not built as part of the workspace." Consequences:

- Encoder pin assignments (D2/D3 left, A4/A5 right), motor PWM pins (D5/D6/D9/D10) and enable pins
  (D12/D13) are catalogued at **Medium** confidence with `EvidenceType: Documentation`, because they
  are the reference pinout rather than the flashed pinout.
- **PWM carrier frequency is left blank.** It could have been guessed from ATmega328P timer defaults;
  it was not, because the repository does not state it.
- The **500 ms watchdog is catalogued as `Planned`**, not `Production`. `firmware/README.md` describes
  it as a change that must be made. Nothing in-repo evidences that it has been made, so SYS-PLT-5
  cannot be considered verified from this repository alone.
- The **firmware PID rate is not verifiable.** `base_bridge.rad_s_to_counts_per_loop()` converts rad/s
  into counts-per-loop by dividing by `pid_rate_hz` (30.0). If the flashed firmware's actual loop
  rate differs, every commanded velocity is silently scaled by the ratio.
- D13 is also the Nano's onboard LED pin. The reference pinout uses it as a motor enable. No BillieBot
  document discusses this.

### 3.2 Two battery-state publishers with different semantics

| Path | Rate | Fields |
|---|---|---|
| `base_bridge` ← Motor Nano A0 | 1 Hz | Synthesises per-cell voltages by division; maps low voltage onto `POWER_SUPPLY_HEALTH_COLD` |
| `sensor_nano_bridge` ← Sensor Nano A0 | 5 Hz | `cell_voltage` empty, health `UNKNOWN`, unmeasured fields NaN |

A third, bench-only publisher (`battery_threshold_test`) injects synthetic voltages. Which path is
production is not settled: the IMU/battery bench plan calls the Sensor Nano firmware
"production-candidate" and flags the duplication as BLK-02, but no production bringup launch file
starts `sensor_nano_bridge`. Both catalog entries share `SIG-0083` because they are the same logical
signal on the same topic; the `SentBy` and `Notes` columns distinguish them.

### 3.3 Rates that could not be determined

| Signal | Why |
|---|---|
| `/scan` (real lidar) | Scan rate follows rotor speed and `scan_mode`; neither is configured. `BRINGUP_LADDER_ANALYSIS.md` quotes a ~5.5–8 Hz range, which is not a single defensible number. Left blank. |
| PWM carrier | See §3.1 |
| Pi thermal I2C bus clock | `thermal_node` calls `board.I2C()` and never sets a clock. The declared `i2c_bus` and `i2c_address` parameters are **read into nothing** — the real-mode path ignores them. |
| I2S sample rate | `speaker_node` shells out to `aplay -D plughw:0,0`; no I2S parameter is configured anywhere. |
| Lidar rotor speed | Not exposed. |
| `/audio/events` | Genuinely event-driven and sparse. `/bench/audio_classifier/status` exists precisely so the 2 Hz cadence can be measured separately — and the bench results record that this sustained measurement was **not performed**. |
| `/thermal/blob` | Data-dependent: published only when ≥8 warm pixels are found. |

### 3.4 Ambiguous or duplicated publishers

- **`/dog/found` has two publishers** (GAP-12). `oakd_dog_detector` publishes true *and* false at 5 Hz;
  `dog_locator` publishes true only, never false on loss. `mission_controller` sees an interleaved
  stream, which `BRINGUP_LADDER_ANALYSIS.md` identifies as a cause of PATROL↔TRACK_OBSERVE flapping.
- **`/bench/noir/diagnostics` has two publishers with different payloads.** `image_quality_monitor`
  publishes computed image metrics; `noir_cam_node` publishes libcamera capture metadata when
  `publish_metadata` is enabled. Same topic name, different schemas.
- **`/cmd_vel` has two active publishers** — Nav2's `controller_server` and `retreat_server`, which
  publishes Twist directly, bypassing Nav2 and its speed limits.
- **`/tf` has up to four owners** across configurations. `odom→base_link` ownership was resolved by
  GAP-5 in favour of the EKF (`publish_tf: false` in `base_driver.yaml`), so the `base_bridge` TF
  publisher is catalogued as `Unclear` — in the shipped configuration the broadcaster object is never
  even constructed.

### 3.5 Undocumented or unverified hardware connections

- Every sensor mount origin in `billiebot.urdf.xacro` is a `TODO(measure)` placeholder, so every
  static transform value is unverified (`MEASURE_ME.md`).
- `encoder_ticks_per_rev` (2000.0), `wheel_radius` (0.034 m) and `wheel_separation` (0.298 m) are all
  uncalibrated. The reference code carried alternatives of 1974.7 and 1779.0 ticks/rev.
- `battery_divider_ratio` (6.0) is nominal until the installed resistors are measured (BLK-07);
  `adc_reference_voltage` (5.0 V) is unmeasured (BLK-08).
- The BNO055 world-frame convention is unverified on this hardware (BLK-14), which is why
  `bno055_native` is the default rather than `nwu_to_enu`.
- IMU covariances are explicitly provisional, not a characterised accuracy claim (BLK-15).
- Motor magnetic disturbance of the magnetometer is documented as a risk (BLK-13) and never measured.
- The `dog_logger` snapshot path has no image subscription, so the source of the snapshot pixels is
  not established anywhere in the repository (related to GAP-9).

### 3.6 Planned versus implemented

- **IR illuminator does not exist.** SYS-PER-5 requires low-light NoIR detection; the design document
  states plainly that the NoIR camera has no emitter. No pin, driver or code exists.
- **Treat dispenser is a stub.** `DispenseTreat` returns NOT_IMPLEMENTED; SYS-EXT-5 reserves a rail.
- **`PatrolWaypoints.action` has neither a server nor a client.** Patrol sequencing lives instead in
  `mission_controller`'s `patrol_waypoints` parameter list.
- **`BatteryStatus.msg` is built and published by nothing** (GAP-3 in `DISCREPANCY_RESOLUTION_PLAN.md`).
  It is not in the catalog as a signal, because no flow conveys it.
- **The stuck-detection logic of ACT-04 / SYS-NAV-4** (no odometric progress >5 s, three failures then
  SAFE) is not implemented in any repository node. Only the stock Nav2 recovery behaviours are
  configured.

### 3.7 Signals that may no longer be active

The three C++ behavior-tree nodes — `policy_decision_node`, `battery_guard_node`, `estop_guard_node` —
compile into `billiebot_bt_nodes`, but no launched process loads `behavior_trees/billiebot_main.xml`
or registers them. They are catalogued as `Legacy`, since SYS-EXT-2's `PolicyDecision` extension point
is architecturally live even though the code path is dead. The XML itself references roughly ten leaf
nodes with no C++ implementation.

---

## 4. Documented but not present in production code

| Signal / interface | Where documented | Status in code |
|---|---|---|
| Teleoperation `/cmd_vel` | `BillieBot_System_Design.md` §5.1, MBSE IBD-03 c1, SYS-PLT-4 | No teleop node, launch entry or dependency in `billiebot_ws` |
| `/initialpose` | MBSE IBD-03 c23 | Standard AMCL interface; nothing BillieBot-authored touches it |
| SSH admin interface `IF_SSH_Admin` | Design doc IBD-00 | Nothing configures it |
| Amplified speaker drive, MAX98357A | Design BDD-04, MBSE IBD-01 c12 | No amplifier configuration anywhere; `speaker_node` uses generic ALSA |
| Whole power-distribution tree (bus, fuses, regulators) | Design §4.2, MBSE IBD-02 | No software or firmware element measures or controls any branch except the divided sense voltage |
| Master switch / off-board charging | Design §4.2, §4.3 | Physical only; no dock or charge-contact interface |
| Reserved 5 V/9 A actuator rail | SYS-EXT-5 | Reserved only |
| `/imu/data` attributed to `base_bridge` | Design doc §5.2 ROS table | **Wrong** — `base_bridge` has never published `/imu/data`; `use_imu` is a declared-but-unused parameter. The publisher is `sensor_nano_bridge`. |
| Single Arduino MCU | Design doc BDD-02/BDD-05 | Superseded: the bench plan establishes a **two-Nano** architecture (Motor Nano + Sensor Nano). The design document has not been updated. |
| `ros2_control` + `diffdrive_arduino` | Design doc §5.1 | Not used; README records the decision to wrap the reference serial protocol directly |
| `rgb2 : Pi Camera v2` on CSI-1 | Design BDD-03 (marked optional) | No node, launch file or URDF frame |
| `mems : SPH0645 I2S mic` | Design BDD-04 (marked spare) | Nothing |

---

## 5. In production code but absent from the architecture documentation

The `BillieBot_System_Design.md` §5.2 ROS table and the MBSE IBD-03 connector table are the two
authoritative architecture views. The following signals exist in code and appear in neither.

| Signal | Interface | Note |
|---|---|---|
| Magnetic field | `/imu/mag` | Published by `sensor_nano_bridge`; no consumer |
| Barometric pressure | `/barometer/pressure` | Same |
| Barometric temperature | `/barometer/temperature` | Same |
| Battery raw ADC | `/bench/battery/adc` | The UT-BAT-01 pairing invariant depends on it |
| Sensor Nano diagnostics | `/bench/sensor_nano/diagnostics` | Carries BNO055 calibration levels that have no home in `sensor_msgs/Imu` |
| Audio classifier status | `/bench/audio_classifier/status` | The only way to measure the 2 Hz cadence |
| Audio capture diagnostics | `/bench/audio/diagnostics` | |
| OAK-D bench acquisition set | `/bench/oakd/rgb/image_raw`, `/depth/image_raw`, `/points`, both `camera_info`, `/diagnostics` | Topics the production detector deliberately does not publish |
| Compressed Foxglove previews | `/bench/oakd/**/preview/compressed`, `/bench/oakd/points_preview` | |
| Detector previews | `/oak/rgb/preview`, `/oak/depth/preview`, `/oak/rgb/annotated` | `/oak/rgb/preview` appears in the design doc; the depth and annotated variants do not |
| Thermal visualisation | `/bench/thermal/image_color`, `/bench/thermal/image_normalized` | |
| NoIR diagnostics | `/bench/noir/diagnostics` | Two publishers, see §3.4 |
| Bench rate monitor | `/bench/status` | |
| XVF3800 power policy | USB GPO read/write, LED effect | An entire enforced-and-verified device-state contract with no architectural representation |
| XVF3800 DoA control read | USB `CMD_DOA_VALUE` | The design doc mentions DoA as a value but not the control-transfer interface that fetches it |
| Nano USB auto-reset (DTR) | USB CDC DTR | Materially affects start-up behaviour and is handled asymmetrically by the two bridges |
| Sensor Nano serial record protocol | `'I'`/`'M'`/`'B'`/`'P'`/`'S'` with CRC-16/CCITT-FALSE | The design doc's serial-frame model predates it entirely |
| Sensor configuration writes | MLX90640 refresh rate, BNO055 mode, BMP280 sampling, NoIR libcamera controls, OAK-D blob upload | No architecture view models the *inbound* configuration direction of any sensor interface |

---

## 6. Notes for the Cameo/MSOSA import

- `SignalID` is stable and repeats across rows only when the flow conveys the **same logical signal**.
  Two entries share an ID when they share both `Name` and `Interface`. This is why the three
  `/cmd_vel` publishers collapse onto one ID while the five distinct transforms travelling on `/tf`
  keep separate IDs — the latter are different logical signals sharing a channel.
- `SysMLRepresentation` is a recommendation, not a constraint. Roughly half the catalog is not a UML
  `Signal`: 48 entries are physical interactions, 26 are item flows, 9 are energy flows, and 21 are
  plain value data best modelled as FlowProperties or ValueTypes on ports.
- `Direction` and `SystemBoundary` are **per-flow**, not per-signal (see §1).
- `Rate` is numeric wherever populated and blank in 183 of 299 rows. `RateBasis` is populated in
  **all** 299 rows, including every blank-rate row, so no rate is unexplained.
- Component names have been normalised: architectural blocks in title case (`OAK-D Lite`,
  `Sensor Nano`, `Left Encoder Motor`), ROS nodes in their executable names (`base_bridge`,
  `oakd_dog_detector`). No source filename is used as a `SentBy` or `ReceivedBy` value.
- The `(none)` sender/receiver on `SIG-0164` (PatrolWaypoints) is deliberate: the interface exists
  with no endpoint at either end.

### Recommended first modelling pass

1. Import `Category == Optical | Acoustic | Mechanical | Thermal | Environmental` as physical
   interactions on the SoS context IBD — these are the flows the existing architecture documents omit
   almost entirely.
2. Import `Category == ROS` filtered to `ImplementationStatus == Production` as the IBD-03 refresh;
   the `Test/Bench Only` ROS rows belong in a separate bench-configuration view.
3. Model the two Arduinos as distinct blocks before anything else. Every downstream allocation matrix
   depends on that split, and the current design document still shows one.
