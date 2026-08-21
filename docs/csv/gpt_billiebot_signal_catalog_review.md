# BillieBot System Signal Catalog — Review

**Repository:** `sean-mackenzie/billie-bot-claude`  
**Audit date:** 2026-08-20  
**Baseline branch:** `main`  
**Repository HEAD observed during audit:** `e9e22ea8220fcd69e486563a260f94b45188aacd`  
**Primary catalog:** `docs/architecture/billiebot_signal_catalog.csv`

## Scope and method

This catalog was built from a repository-wide static architecture audit centered on production ROS 2 code, launch/configuration files, custom ROS interfaces, navigation configuration, Motor Nano firmware, Sensor Nano bench firmware/bridge code, sensor test utilities, and the repository MBSE/system documentation. The audit cross-references implementation and documentation rather than treating a topic name appearing in a document as proof that the flow is active.

The catalog deliberately separates meaningful stages in end-to-end chains. For example, wheel rotation, quadrature channels, MCU counts, UART replies, host tick deltas, ROS odometry, and filtered odometry are separate entries. Likewise, physical optical/acoustic/thermal inputs are kept distinct from the digitized data and ROS messages they ultimately produce.

Unknown frequencies are intentionally blank. Serial baud rates and I2C clock frequencies are recorded in `RateBasis`, not misreported as payload `Rate` values. Architecturally important physical interactions not explicitly measured in code are included only with `Inferred` evidence and reduced confidence.

## Catalog statistics

- **Total unique signals:** 219
- **Total sender→receiver flow rows:** 266
- **ROS signals:** 81
- **Software/data signals:** 55
- **Electrical signals:** 35
- **Optical signals:** 7
- **Acoustic signals:** 3
- **Mechanical signals:** 16
- **Thermal signals:** 2
- **Environmental signals:** 4
- **Power/Energy signals:** 10
- **Network signals:** 6
- **Signals containing inferred evidence:** 36
- **Low-confidence signals:** 7

### Sender→receiver flow rows by implementation status

- Production: 187
- Test/Bench Only: 39
- Planned: 18
- Legacy: 6
- Documentation Only: 8
- Unclear: 8

## Major signal chains

### Lidar → localization/navigation

`RPLidar scanning laser → reflected laser → raw range samples → USB-UART → rplidar_node → /scan → SLAM/AMCL/Nav2 costmaps → /map and TF → Nav2 planning/control`

The RPLidar serial interface is explicitly 115200 baud. The repository does not configure one exact real-world scan publication Hz; documentation gives approximately 5.5–8 Hz, so the production `/scan` `Rate` is left blank and the range is recorded in `RateBasis`. The mock scan is explicitly 10 Hz.

### OAK-D → dog localization

`Dog/environment visible radiance → OAK-D RGB/stereo cameras → RGB + stereo depth → YOLO spatial candidate → /dog/detections_3d → dog_locator + TF → /dog/pose_map → state_fusion / approach_dog_server`

Production 3-D detections are 5 Hz. `/dog/found` has two active publishers (`oakd_dog_detector` and `dog_locator`), represented as separate sender→receiver rows under the same SignalID.

### Thermal → dog-state fusion

`Dog/environment LWIR → MLX90640 → I2C temperature array → warm-pixel mask → connected components → /thermal/image + /thermal/blob → state_fusion`

The MLX90640 sensor refresh is configured at 8 Hz while ROS thermal processing/publication is 4 Hz; those rates are intentionally not conflated.

### Audio → behavior event

`Dog bark / ambient acoustic pressure → ReSpeaker microphone array → USB PCM + USB DoA control → RMS/YAMNet → /audio/events → state_fusion + mission_controller`

The waveform is sampled at 16 kHz in approximately 0.975 s chunks. Classified events are asynchronous and rate-limited to at most 2 Hz.

### Sensor Nano bench chain

`Robot motion / magnetic field / pressure / temperature / battery-divider voltage → BNO055/BMP280/A0 → I2C/ADC → Sensor Nano records → 115200 serial → SensorNanoBridge → ROS bench topics`

Firmware rates are explicit: IMU 50 Hz, magnetometer 10 Hz, battery 5 Hz, barometer 2 Hz, firmware status 1 Hz. **These flows remain Test/Bench Only in the current repository:** rung-14 full bringup does not include SensorNanoBridge, while production `base_bridge` still publishes `/battery_state` from the Motor Nano at 1 Hz and production EKF IMU input remains disabled.

### Navigation command → drivetrain → robot motion

`/cmd_vel → base_bridge inverse kinematics → counts/PID-loop → 'm L R' UART @ 30 Hz → Motor Nano PID @ 30 Hz → D5/D6/D9/D10 PWM + D12/D13 enables → L298N motor drive → motor torque/shaft rotation → wheel torque/angular velocity → wheel-floor traction → chassis translation/yaw`

Explicit Motor Nano wiring found in firmware:
- left encoder A/B: D2/D3
- right encoder A/B: A4/A5
- right motor backward/forward: D5/D9
- left motor backward/forward: D6/D10
- right/left enables: D12/D13

Mechanical torque/traction/contact-force magnitudes are not instrumented, so those rows contain no invented numeric values and are marked inferred.

### Encoder → odometry

`Motor shaft rotation → encoder quadrature A/B → MCU pin-change ISR count → 'e' request/reply @ 30 Hz → host left/right deltas → midpoint-arc odometry → /odom @ 30 Hz → EKF → /odometry/filtered @ 30 Hz → Nav2`

### Battery → SAFE

`Battery voltage → divider → Motor Nano A0 → ADC reply @ 1 Hz → base_bridge voltage/health conversion → /battery_state → mission safety check @ 2 Hz → SAFE`

The host command deadman (0.5 s) and firmware motor watchdog (500 ms) form separate safety chains. Mission-level live e-stop propagation and navigation-failure escalation remain incomplete.

### Cognition → logging → report

`DogDetection3D + dog pose + ThermalBlob + AudioEvent → 10 s evidence window → 2 Hz classification → 3 s hysteresis → /billie/state → dog_logger → SQLite → daily_report → Markdown/PNG → HTTP :8080 → operator`

The logger's snapshot path currently refers to an empty placeholder `.jpg`; no real camera snapshot transfer is implemented.

## Important unresolved questions / gaps

1. **Sensor Nano production integration:** production-candidate firmware and extensive bench support exist, but the bridge is absent from full bringup.
2. **Potential duplicate `/battery_state` publishers:** `base_bridge` publishes at 1 Hz; SensorNanoBridge publishes at 5 Hz if launched. A production integration should select/arbitrate the authoritative source.
3. **Duplicate `/dog/found` publishers:** OAK-D detector and dog_locator both publish it.
4. **NoIR consumer:** `/noir/image` is produced at 5 Hz with no production subscriber identified.
5. **Patrol dispatch:** MissionController declares Nav2 client infrastructure, but the active patrol/waypoint dispatch chain is incomplete.
6. **Mission e-stop and nav-failure state:** `_estopped` and `_nav_failure_count` lack live production update paths.
7. **Near-dog speed restriction:** the required 0.15 m/s within 2 m speed-filter path is not implemented.
8. **Speak naming:** `/speak` and `/mission/speak` coexist.
9. **Operator alert/pickup channel:** requirements exist, but no transport/channel is implemented.
10. **Power wiring:** power-distribution flows are documentation/hardware-definition evidence and cannot be verified from source code.
11. **Physical-force quantities:** motor torque and wheel-ground forces are architecturally meaningful but not directly instrumented; no magnitudes/rates were invented.
12. **`/robot_description`:** architecture documentation describes a latched topic, but the runtime representation should be confirmed because ROS 2 commonly uses it primarily as a parameter.
13. **A4/A5 conflict history:** the legacy Motor-Nano BNO055 path conflicts with right encoder channels A4/A5; the separate Sensor Nano bench architecture appears to be the intended resolution but is not production-integrated.

## Signals/interfaces present in documentation or interface definitions but not active production behavior

- Legacy `billiebot_interfaces/msg/BatteryStatus` contract has no active publisher; production uses `sensor_msgs/msg/BatteryState`.
- `PatrolWaypoints.action` exists, but no BillieBot PatrolWaypoints action server is implemented.
- Nav2 exposes `navigate_through_poses`, but no active BillieBot patrol executor dispatches waypoint goals.
- The near-dog 0.15 m/s speed-filter/keepout path is an architecture requirement, not an implemented flow.
- The mission audio callback receives DoA-bearing AudioEvent data, but INVESTIGATE transition/reprioritization is not implemented.
- Operator SAFE/recovery alerts and battery pickup requests are required architecturally but have no implemented notification channel.
- The 850 nm NoIR illuminator and its electrical/optical flows are planned, not implemented.
- Treat-dispenser power/interface is planned; the DispenseTreat server remains a NOT_IMPLEMENTED stub.

## Signals present in current code but absent or stale in architecture documentation

- Sensor Nano bench firmware/bridge: `/imu/data`, `/imu/mag`, `/barometer/pressure`, `/barometer/temperature`, 5 Hz `/battery_state`, `/bench/battery/adc`, and `/bench/sensor_nano/diagnostics`. Rung-14 full bringup does not launch this bridge.
- Optional OAK-D bench outputs `/oak/rgb/preview`, `/oak/depth/preview`, and `/bench/oakd_detector/diagnostics` are present in current source. The older MBSE decomposition still describes the RGB preview as missing.
- Thermal test utilities add `/bench/thermal/image_color` and optional `/bench/thermal/image_normalized`.
- NoIR bench code adds `/bench/noir/diagnostics` for luminance/clipping/repeated-frame metrics.
- Audio bench code adds `/bench/audio/diagnostics` and a finite WAV capture artifact.
- Inherited Motor Nano ROSArduinoBridge generic commands (raw PWM, PID update, generic analog/digital I/O) remain in firmware but are not used by production `base_bridge`.

## Cameo/MSOSA modeling guidance

- ROS topics carrying continuous state/telemetry are generally represented as `ItemFlow`.
- ROS service/action requests/results and discrete safety/state-transition events are generally represented as `Signal`.
- Analog values local to an interface are generally represented as `FlowProperty` or `Value/Data`.
- Mechanical forces, torques, rotations, and contact interactions are marked `Physical Interaction`.
- Electrical power paths are marked `Energy Flow`.
- A single `SignalID` is reused for the same logical signal when it has multiple receivers or multiple implementation-path senders (for example real/mock `/scan`, production/bench `/battery_state`, production/bench odom→base TF).
- Blank `Rate` cells are intentional and mean no defensible single numeric Hz value was established.
- Production source code was not modified during this task.
