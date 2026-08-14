# BillieBot IMU + Battery-Sense Bench Test Plan

**Document status:** Final operator-facing plan — pending user approval  
**Date:** 2026-08-14  
**Applies to:** BillieBot two-Arduino-Nano architecture  
**Primary host:** Jetson Orin Nano  
**Sensor controller:** Arduino Nano V3 (“Sensor Nano”)  
**Motion controller:** Arduino Nano V3 (“Motor Nano”) — intentionally excluded from these bench tests  
**IMU/barometer:** DFRobot Gravity 10 DOF IMU AHRS BNO055 + BMP280, SEN0253  
**Battery:** OVONIC 3S LiPo, 11.1 V nominal, 5000 mAh, with resistive battery-sense divider  
**ROS 2:** Humble

---

## 1. Purpose

This plan verifies the new BillieBot **Sensor Nano subsystem** before full robot integration.

The approved architecture is:

```text
                              Jetson Orin Nano
                             /                 \
                            / USB               \ USB
                           /                     \
                  Motor Nano                     Sensor Nano
                  ----------                     -----------
                  Left encoder                   BNO055
                  Right encoder                  BMP280
                  Motor PWM                      Battery divider / A0
                  Motor direction
                  Motor PID/watchdog

                        ROS 2 software on Jetson
                        -----------------------
                 base_bridge           platform_sensor_bridge
                      |                         |
                 /odom, joints             /imu/data
                                           /battery_state
                                           /imu/mag
                                           /barometer/*
                                              |       |
                                              |       +--> mission_controller --> SAFE
                                              |
                                              +--> robot_localization EKF
```

The bench campaign deliberately tests the Sensor Nano separately from the Motor Nano. This provides fault isolation and prevents motor/encoder behavior from confusing IMU, I²C, ADC, serial, ROS-message, or safety-chain debugging.

The tests are intended to do two things:

1. verify the physical sensor and battery-acquisition hardware; and
2. exercise the interfaces that the production BillieBot software will ultimately consume.

This is **not** a full vehicle test and does not require the robot chassis, motors, encoders, lidar, Nav2, SLAM, cameras, Raspberry Pi, or the Motor Nano.

---

# 2. Test Summary

| Test ID | Test | Primary purpose | Nominal duration |
|---|---|---|---:|
| **UT-IMU-01** | Sensor Nano IMU/barometer acquisition | Verify BNO055/BMP280 hardware, serial protocol, rates, orientation response, and data integrity | 180 s |
| **UT-IMU-02** | ROS IMU + EKF compatibility | Verify `/imu/data` contract and acceptance by `robot_localization` | 120 s |
| **UT-BAT-01** | Battery divider acquisition and accuracy | Verify divider ratio, ADC behavior, voltage conversion, ROS publication, and accuracy versus DMM | ~150–240 s operator-paced |
| **UT-BAT-02** | Low-battery SAFE propagation | Verify real Sensor Nano battery data propagates to the production mission controller and triggers SAFE | ~90 s |

A fifth result is produced as part of UT-BAT-02:

- **UT-BAT-02B — exact software threshold boundary check at 10.5 V.**

This subtest intentionally separates the exact software inequality test from the physical PSU test, where ADC quantization and measurement uncertainty make an exact 10.500 V boundary less appropriate.

---

# 3. Acceptance Philosophy

The existing BillieBot sensor-bench suite uses the following principles, which this plan retains:

1. use minimum hardware necessary for the test;
2. keep raw/authoritative data separate from visualization;
3. use rosbag2 as the authoritative ROS time-series record;
4. save human-readable exports;
5. automatically generate `metrics.json`, `metrics.csv`, and `report.md`;
6. make provisional engineering limits configurable;
7. do not relax a hardware or timing threshold merely because Foxglove or a remote client behaves poorly;
8. preserve exact commands, repository commit, parameters, and operator-entered ground truth.

Each run shall create:

```text
<results_dir>/
├── manifest.yaml
├── console.log
├── bag/
├── exports/
├── plots/
├── metrics.json
├── metrics.csv
└── report.md
```

The top-level pass/fail value in `metrics.json` is the authoritative automated verdict. Operator observations and known blockers are recorded separately.

---

# 4. Two-Nano Architecture Rules for This Campaign

## 4.1 Motor Nano

During all four tests in this document:

- disconnect the Motor Nano from the Jetson;
- do not launch `billiebot_base/base_bridge`;
- do not connect motors;
- do not connect encoders;
- do not power the motor rail.

The Sensor Nano is the only Arduino required.

## 4.2 Sensor Nano

The Sensor Nano owns:

```text
A4 / SDA    -> SEN0253 SDA ("D")
A5 / SCL    -> SEN0253 SCL ("C")
A0          -> battery-divider midpoint
USB serial  -> Jetson
```

Its firmware shall acquire measurements only. It shall not contain:

- mission-mode logic;
- SAFE-state logic;
- motor commands;
- motor watchdogs;
- ROS-specific behavior.

The Jetson bridge owns:

- parsing;
- ROS timestamps;
- ROS frame IDs;
- SI-unit conversion;
- battery-voltage conversion/calibration;
- ROS `BatteryState`;
- ROS `Imu`;
- safety-policy interfacing.

---

# 5. Required Hardware and Tools

## 5.1 Required for UT-IMU-01 / UT-IMU-02

- Jetson Orin Nano
- Arduino Nano V3 — Sensor Nano
- USB data cable between Jetson and Sensor Nano
- DFRobot SEN0253
- 4 jumper wires or the appropriate Gravity-to-DuPont/I²C cable
- stable nonconductive test surface
- tape or paper marks indicating 0°, +90°, and -90° yaw positions
- optional right-angle block for roll/pitch positioning

The Nano is powered from the Jetson USB connection.

**Do not also feed external 5 V into the Nano 5V pin during these tests.**

## 5.2 Required for UT-BAT-01 / UT-BAT-02

Add:

- adjustable DC bench power supply capable of at least 0–13 V
- digital multimeter
- production battery-divider assembly, or the selected production resistor pair
- breadboard or secure test terminals
- jumper wires

The LiPo itself is **not required** for the controlled voltage sweep.

A real-battery spot check may be performed only after UT-BAT-01 passes.

---

# 6. Sensor Nano Wiring

## 6.1 SEN0253 to Sensor Nano

The SEN0253 accepts 3.3–5 V and exposes a Gravity I²C interface. Its default addresses are:

- BNO055: `0x28`
- BMP280: `0x76`

Wire exactly as follows:

| Sensor Nano | SEN0253 marking | Function |
|---|---|---|
| **5V** | **VCC / +** | sensor power |
| **GND** | **GND / -** | ground |
| **A5** | **C** | I²C SCL |
| **A4** | **D** | I²C SDA |

Leave these SEN0253 pins in their default state unless a later design specifically requires them:

- NBOOT
- RST
- INT
- I2C_ADDR
- PS1
- PS2
- BL_IND

### Wiring diagram

```text
Jetson USB
    |
    | USB data + 5 V
    v
+---------------------+
| Arduino Nano V3     |
|   Sensor Nano       |
|                     |
|  5V  --------------+--------------------> SEN0253 VCC
|  GND ---------------+--------------------> SEN0253 GND
|  A5  ------------------------------------> SEN0253 C / SCL
|  A4  ------------------------------------> SEN0253 D / SDA
|  A0  ---- battery divider (later tests)
+---------------------+
```

For IMU-only tests, if the battery divider is completely disconnected, install a temporary approximately 10 kΩ A0-to-GND pull-down or leave the already-installed divider attached with its battery/PSU input at 0 V. Do not leave A0 intentionally floating if battery telemetry is enabled.

---

# 7. Battery Divider Wiring

## 7.1 Divider definition

Define the divider unambiguously as:

```text
VBAT ---- R_TOP ----+---- R_BOTTOM ---- GND
                    |
                    +---- A0
```

Therefore:

```text
V_A0 / V_BAT = R_BOTTOM / (R_TOP + R_BOTTOM)

divider_ratio = V_BAT / V_A0
              = (R_TOP + R_BOTTOM) / R_BOTTOM
```

The repository currently assumes:

```text
divider_ratio = 6.0
```

The actual resistor values are not presently documented.

### Recommended default if the production divider has not yet been built

A reasonable nominal implementation is:

```text
R_TOP    = 50.0 kΩ, 1 %
R_BOTTOM = 10.0 kΩ, 1 %
```

This gives:

```text
divider_ratio = 6.0
12.6 V battery -> approximately 2.10 V at A0
```

If a divider is already assembled with different resistor values, **do not replace it solely to match these nominal values**. Measure the installed values and configure the actual ratio.

## 7.2 Bench-supply wiring

```text
                    DC BENCH SUPPLY
                    +            -
                    |            |
                    |            +-------------------------+
                    |                                      |
                    v                                      v
                 R_TOP                                   GND
                    |
                    +-------------> Arduino Nano A0
                    |
                 R_BOTTOM
                    |
                    +------------------------------------> GND
                                                         |
Jetson USB ----------------------------------------> Sensor Nano
                                                     USB power/data
```

### Exact connection table

| Connection | From | To |
|---|---|---|
| 1 | PSU positive | divider `VBAT` / top of `R_TOP` |
| 2 | divider midpoint | Sensor Nano **A0** |
| 3 | bottom of `R_BOTTOM` | Sensor Nano **GND** |
| 4 | PSU negative | Sensor Nano **GND** |
| 5 | Jetson USB | Sensor Nano USB connector |

**Never connect the bench-supply positive output to Nano 5V or VIN in this test.**

The PSU stimulates only the battery-divider input.

## 7.3 DMM measurements

At every voltage point record:

1. `V_BAT_DMM`: PSU/divider input measured directly with the DMM.
2. `V_A0_DMM`: divider midpoint to Nano GND.
3. the ROS-reported battery voltage.
4. raw ADC value.

If only one DMM is available, measure `V_BAT_DMM` first, then `V_A0_DMM`, then return the meter to `V_BAT_DMM` and confirm the supply did not move significantly.

---

# 8. Electrical Safety Gates

Before applying more than 3 V from the bench supply:

1. PSU output OFF.
2. Sensor Nano USB disconnected.
3. Measure `R_TOP`.
4. Measure `R_BOTTOM`.
5. Record both values.
6. Calculate:

```text
ratio_measured = (R_TOP + R_BOTTOM) / R_BOTTOM
V_A0_expected_at_12V6 = 12.6 / ratio_measured
```

7. Confirm the divider is approximately the intended 6:1 design.
8. Confirm `V_A0_expected_at_12V6` is approximately 2.1 V and comfortably below the Nano 5 V ADC rail.
9. Reconnect the Nano USB.
10. Set PSU to **3.0 V**.
11. Set PSU current limit to **20 mA**.
12. Turn the PSU ON.
13. Measure A0-to-GND with the DMM.
14. Confirm it matches `3.0 / ratio_measured` before proceeding.
15. Turn PSU OFF.
16. Only then perform the full sweep.

If the measured ratio is grossly different from 6.0, stop the test and correct the wiring or configuration.

---

# 9. Planned Sensor Nano Firmware

> This section is the firmware specification for the later Claude Code implementation. The firmware does not exist yet in the approved final form.

## 9.1 Proposed source location

```text
billiebot_ws/src/billiebot_sensor_tests/
└── firmware/
    └── sensor_nano/
        ├── sensor_nano.ino
        └── README.md
```

The firmware is intended to be the **production-candidate Sensor Nano firmware**, not a disposable one-off smoke-test sketch.

## 9.2 Core behavior

At boot:

1. start serial at **115200 baud**;
2. initialize I²C;
3. detect BNO055 at `0x28`;
4. detect BMP280 at `0x76`;
5. initialize the BNO055 in the selected fused-orientation mode;
6. initialize the BMP280;
7. emit a startup/status record;
8. begin non-blocking periodic acquisition.

Target rates:

| Data | Firmware rate |
|---|---:|
| BNO055 quaternion + gyro + accelerometer | **50 Hz** |
| Battery ADC | **5 Hz** |
| BMP280 pressure + temperature | **2 Hz** |
| status/health | **1 Hz** |

No `delay()`-based long blocking loop shall control scheduling. Use elapsed `micros()` / `millis()` timing.

## 9.3 IMU acceleration requirement

The firmware shall transmit the BNO055 **accelerometer/specific-force vector that includes gravity**, not the already gravity-removed BNO055 “linear acceleration” output.

This matters because the intended BillieBot `robot_localization` configuration uses:

```yaml
imu0_remove_gravitational_acceleration: true
```

ROS IMU convention expects an accelerometer at rest with +Z upward to report approximately +9.81 m/s² along +Z after the sensor-frame convention has been handled.

Using the BNO055 gravity-removed linear-acceleration vector and then asking `robot_localization` to remove gravity again would be incorrect.

## 9.4 Serial protocol

Use newline-delimited machine-readable ASCII with:

- record type;
- monotonically incrementing sequence number;
- Nano `micros()` timestamp;
- fixed field order;
- CRC-16/CCITT-FALSE checksum.

Proposed records:

### IMU

```text
I,<seq>,<t_us>,<qw>,<qx>,<qy>,<qz>,<gx>,<gy>,<gz>,<ax>,<ay>,<az>,<cal_sys>,<cal_gyr>,<cal_acc>,<cal_mag>*<crc>
```

Units:

```text
q*     unitless quaternion
g*     rad/s
a*     m/s^2, specific force including gravity
```

### Battery

```text
B,<seq>,<t_us>,<adc_mean>*<crc>
```

The Arduino sends **raw ADC**, not battery volts.

### Barometer

```text
P,<seq>,<t_us>,<pressure_pa>,<temperature_c>*<crc>
```

### Status

```text
S,<seq>,<t_us>,<bno_ok>,<bmp_ok>,<i2c_errors>,<imu_read_errors>*<crc>
```

The parser shall reject malformed or CRC-failed records and count failures.

## 9.5 Battery ADC behavior

- Arduino ADC reference: default AVcc.
- sample A0 repeatedly and average multiple samples per battery record;
- transmit the averaged raw ADC result;
- do not embed a 5.000 V assumption in firmware;
- do not embed the divider ratio in firmware;
- do not embed SAFE thresholds in firmware.

The Jetson bridge shall have parameters such as:

```yaml
battery_divider_ratio: 6.0
adc_reference_voltage: <bench-measured value>
battery_cell_count: 3
battery_low_voltage: 10.5
battery_critical_voltage: 9.9
```

A later hardware revision may improve ADC-reference accuracy. This test intentionally measures the current implementation instead of hiding error through software tuning.

## 9.6 Fault behavior

The firmware shall:

- continue running if the BMP280 fails after startup;
- flag degraded sensor health;
- attempt bounded reinitialization after repeated I²C failures;
- never hang indefinitely on an I²C transaction;
- continue battery sampling if the IMU fails;
- continue IMU sampling if the BMP280 fails;
- increment sequence numbers so dropped serial frames are detectable;
- tolerate `micros()` rollover in the Jetson-side parser.

There are no actuators on the Sensor Nano, so it does not need the motor watchdog used by the Motor Nano.

---

# 10. Planned Jetson Test Software

> These commands become runnable only after the later implementation step.

Proposed additions to `billiebot_sensor_tests`:

```text
config/
├── sensor_nano_bench.yaml
└── ekf_imu_bench.yaml

launch/
├── sensor_nano_imu_bench.launch.py
├── sensor_nano_imu_ekf_bench.launch.py
├── sensor_nano_battery_bench.launch.py
└── sensor_nano_battery_safe_bench.launch.py

billiebot_sensor_tests/
└── sensor_nano/
    ├── __init__.py
    ├── protocol.py
    ├── sensor_nano_bridge.py
    ├── imu_metrics.py
    ├── analyze_imu.py
    ├── battery_metrics.py
    ├── analyze_battery.py
    ├── battery_point_recorder.py
    └── score_battery_safe.py
```

Expected console scripts:

```text
sensor_nano_bridge
analyze_sensor_nano_imu
analyze_sensor_nano_battery
record_battery_point
score_battery_safe
```

Editing the existing `setup.py` to add these console-script entries is approved if necessary.

## 10.1 Bench ROS topics

The bench bridge shall publish production-compatible topics where the contract is itself under test:

| Topic | Type | Purpose |
|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | production IMU contract |
| `/imu/mag` | `sensor_msgs/MagneticField` | optional informational output |
| `/barometer/pressure` | `sensor_msgs/FluidPressure` | BMP280 |
| `/barometer/temperature` | `sensor_msgs/Temperature` | BMP280 |
| `/battery_state` | `sensor_msgs/BatteryState` | production battery contract |
| `/bench/battery/adc` | `std_msgs/Float32` or equivalent | raw averaged ADC |
| `/bench/sensor_nano/diagnostics` | `diagnostic_msgs/DiagnosticArray` | status, CRC/parser/I²C counters |

The bridge shall use `imu_link` as the default IMU frame.

## 10.2 ROS IMU conventions

The bridge shall ensure:

- right-handed sensor coordinates;
- angular velocity in rad/s;
- linear acceleration in m/s²;
- acceleration is specific force including gravity;
- quaternion is normalized;
- `header.frame_id = "imu_link"`;
- covariance fields are populated with configured values or zero when unknown;
- no field falsely claims valid data if the sensor does not provide it.

The bridge shall document any BNO055-axis/world-frame conversion.

---

# 11. Common Jetson Terminal Setup

Open a fresh Jetson terminal outside Docker.

Run:

```bash
source /opt/ros/humble/setup.bash
source ~/billie-bot-claude/billiebot_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI="file://$HOME/billie-bot-claude/billiebot_ws/install/billiebot_bringup/share/billiebot_bringup/config/cyclonedds.xml"
```

If `.bashrc` already performs these actions, the explicit commands above are still the reference environment.

Verify:

```bash
echo "$RMW_IMPLEMENTATION"
echo "$ROS_DOMAIN_ID"
echo "$ROS_LOCALHOST_ONLY"
echo "$CYCLONEDDS_URI"
```

Expected:

```text
rmw_cyclonedds_cpp
0
0
file:///home/sean/billie-bot-claude/billiebot_ws/install/billiebot_bringup/share/billiebot_bringup/config/cyclonedds.xml
```

Record repository state:

```bash
cd ~/billie-bot-claude
git status --short
git rev-parse HEAD
```

After the later software implementation, rebuild:

```bash
cd ~/billie-bot-claude/billiebot_ws
colcon build --packages-select billiebot_sensor_tests
source install/setup.bash
```

---

# 12. Serial-Device Preflight

For initial bench tests, connect **only the Sensor Nano**.

Run:

```bash
lsusb
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
ls -l /dev/serial/by-id/ 2>/dev/null
ls -l /dev/serial/by-path/ 2>/dev/null
dmesg -T | tail -n 40
```

Identify the Sensor Nano device.

Then:

```bash
udevadm info --query=property --name=/dev/ttyUSB0 | \
  egrep 'ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL|ID_SERIAL_SHORT|ID_PATH'
```

Replace `/dev/ttyUSB0` if the Nano enumerated differently.

Prefer `/dev/serial/by-path/...` if the CH340 device does not provide a unique serial identifier.

Set:

```bash
export SENSOR_NANO_PORT=/dev/serial/by-path/<sensor-nano-entry>
```

Verify:

```bash
readlink -f "$SENSOR_NANO_PORT"
test -r "$SENSOR_NANO_PORT" && echo "readable"
test -w "$SENSOR_NANO_PORT" && echo "writable"
```

If permission is denied:

```bash
groups
ls -l "$(readlink -f "$SENSOR_NANO_PORT")"
```

The user normally needs membership in `dialout`.

---

# 13. Firmware Build and Flash Procedure

The later implementation shall document the exact required Arduino libraries and versions.

Reference CLI procedure:

```bash
arduino-cli board list
arduino-cli core update-index
arduino-cli core install arduino:avr
```

Set the detected port on the machine performing the flash:

```bash
export SENSOR_NANO_FLASH_PORT=<detected-serial-port>
```

Compile using the source-tree firmware:

```bash
cd ~/billie-bot-claude/billiebot_ws/src/billiebot_sensor_tests/firmware/sensor_nano

arduino-cli compile \
  --fqbn arduino:avr:nano:cpu=atmega328 \
  .
```

Upload:

```bash
arduino-cli upload \
  -p "$SENSOR_NANO_FLASH_PORT" \
  --fqbn arduino:avr:nano:cpu=atmega328 \
  .
```

If a Nano clone uses the older bootloader and upload synchronization fails, retry with:

```bash
arduino-cli compile \
  --fqbn arduino:avr:nano:cpu=atmega328old \
  .

arduino-cli upload \
  -p "$SENSOR_NANO_FLASH_PORT" \
  --fqbn arduino:avr:nano:cpu=atmega328old \
  .
```

After flashing, close any serial monitor before starting ROS. Only one process may own the port.

---

# 14. UT-IMU-01 — Sensor Nano IMU/Barometer Acquisition

## 14.1 Goal

Verify:

- BNO055 is present;
- BMP280 is present;
- firmware streams correctly;
- serial framing and CRC are healthy;
- quaternion, angular velocity, and acceleration are finite and physically plausible;
- the orientation response has correct axis/sign behavior;
- output cadence is sufficient for the future EKF.

This test does **not** attempt precision navigation or magnetic-heading qualification.

## 14.2 Hardware

Use the wiring in Section 6.

Battery divider may remain unpowered at 0 V.

Keep the module away from:

- motor magnets;
- speakers;
- steel tools;
- high-current wires;
- large ferromagnetic objects.

## 14.3 Preflight

With ROS software stopped:

```bash
echo "$SENSOR_NANO_PORT"
readlink -f "$SENSOR_NANO_PORT"
```

Optional direct serial smoke test:

```bash
python3 - <<'PY'
import os, serial, time
p = os.environ["SENSOR_NANO_PORT"]
s = serial.Serial(p, 115200, timeout=1)
for _ in range(10):
    line = s.readline().decode(errors="replace").strip()
    if line:
        print(line)
s.close()
PY
```

Expected:

- at least one `S,...` status record;
- repeated `I,...` records;
- `bno_ok=1`;
- `bmp_ok=1`.

Stop here if BNO055 or BMP280 initialization fails.

## 14.4 Terminal 1 — acquisition launch

```bash
export RESULTS=~/billiebot_test_results/UT-IMU-01_$(date -u +%Y%m%dT%H%M%SZ)

ros2 launch billiebot_sensor_tests sensor_nano_imu_bench.launch.py \
  results_dir:="$RESULTS" \
  sensor_port:="$SENSOR_NANO_PORT" \
  baudrate:=115200 \
  duration_sec:=180 \
  record_bag:=true
```

## 14.5 Terminal 2 — live verification

```bash
source /opt/ros/humble/setup.bash
source ~/billie-bot-claude/billiebot_ws/install/setup.bash

ros2 topic list | egrep 'imu|barometer|sensor_nano|battery'
```

Check IMU message:

```bash
ros2 topic echo /imu/data --once
```

Check rates:

```bash
ros2 topic hz /imu/data
```

In another shell if desired:

```bash
ros2 topic hz /barometer/pressure
```

## 14.6 Manual motion sequence

Start with the board flat and motionless.

The automated recording lasts 180 s. Perform approximately this sequence:

| Approx. elapsed time | Operator action |
|---:|---|
| 0–30 s | board flat and completely stationary |
| 30–45 s | rotate approximately +90° about the board/defined X axis; hold |
| 45–60 s | return to starting orientation; hold |
| 60–75 s | rotate approximately -90° about X; hold |
| 75–90 s | return; hold |
| 90–105 s | rotate approximately +90° about Y; hold |
| 105–120 s | return; hold |
| 120–135 s | rotate approximately -90° about Y; hold |
| 135–150 s | return; hold |
| 150–165 s | rotate approximately +90° about Z/yaw; hold |
| 165–180 s | return and hold |

If the implementation includes a ground-truth marker, mark each transition/hold. The exact timestamps are more important than perfect 90° placement.

## 14.7 Analysis

After acquisition ends:

```bash
ros2 bag info "$RESULTS/bag"
```

Run:

```bash
ros2 run billiebot_sensor_tests analyze_sensor_nano_imu \
  --results-dir "$RESULTS" \
  --profile acquisition
```

Inspect:

```bash
cat "$RESULTS/metrics.json"
cat "$RESULTS/report.md"
```

## 14.8 Acceptance

### Required gates

| Criterion | Acceptance |
|---|---|
| BNO055 initialization | PASS |
| BMP280 initialization | PASS |
| IMU finite-value fraction | ≥ 99.9 % |
| Quaternion norm | 0.98–1.02 for ≥ 99 % of valid samples |
| Mean `/imu/data` rate | ≥ 45 Hz |
| Maximum IMU message gap | ≤ 0.10 s |
| Monotonic host timestamps | 100 % |
| Serial CRC/parser failure fraction | ≤ 0.1 % |
| sequence-number discontinuity fraction | ≤ 0.1 % |
| stationary acceleration magnitude | 8.5–11.2 m/s² provisional physical sanity gate |
| stationary gyro magnitude | ≤ 0.10 rad/s provisional |
| commanded hand rotations | clear correct-axis response with expected sign |
| pressure | finite and within 30,000–110,000 Pa |
| temperature | finite and plausible for bench environment |

### Informational/non-gating

- magnetometer calibration level;
- absolute magnetic heading;
- pressure-derived altitude;
- exact roll/pitch/yaw accuracy.

A magnetometer calibration value below 3 does not by itself fail this hardware-acquisition test.

---

# 15. UT-IMU-02 — ROS IMU Contract and EKF Compatibility

## 15.1 Goal

Verify that the Sensor Nano data can be represented as a ROS-standard IMU stream and consumed by `robot_localization` without:

- unit errors;
- frame errors;
- timestamp errors;
- message gaps;
- invalid quaternion behavior;
- gravity double-removal.

The current production `ekf.yaml` still has its IMU block commented out, so this test uses a **bench-only EKF YAML** that mirrors the intended configuration without changing production defaults.

## 15.2 Bench EKF configuration

The planned `ekf_imu_bench.yaml` shall use:

```yaml
frequency: 30.0
sensor_timeout: 0.1
two_d_mode: true

imu0: /imu/data
imu0_differential: false
imu0_relative: false
imu0_remove_gravitational_acceleration: true
```

It shall include the intended IMU fields appropriate for the BillieBot production design.

The launch file shall publish a bench static transform between:

```text
base_link -> imu_link
```

with identity rotation for the bench fixture unless the operator explicitly supplies a measured mounting transform.

## 15.3 Terminal 1 — launch

```bash
export RESULTS=~/billiebot_test_results/UT-IMU-02_$(date -u +%Y%m%dT%H%M%SZ)

ros2 launch billiebot_sensor_tests sensor_nano_imu_ekf_bench.launch.py \
  results_dir:="$RESULTS" \
  sensor_port:="$SENSOR_NANO_PORT" \
  baudrate:=115200 \
  duration_sec:=120 \
  record_bag:=true
```

## 15.4 Terminal 2 — verify ROS contract

```bash
ros2 topic type /imu/data
```

Expected:

```text
sensor_msgs/msg/Imu
```

Then:

```bash
ros2 topic echo /imu/data --once
```

Confirm:

```text
header.frame_id: imu_link
```

Check EKF output:

```bash
ros2 topic hz /odometry/filtered
```

and:

```bash
ros2 topic echo /odometry/filtered --once
```

## 15.5 Manual action

During the 120 s run:

| Approx. time | Action |
|---:|---|
| 0–20 s | stationary, flat |
| 20–40 s | rotate +90° yaw and hold |
| 40–60 s | return to zero and hold |
| 60–80 s | rotate -90° yaw and hold |
| 80–100 s | return to zero and hold |
| 100–120 s | stationary |

Do not translate the fixture unnecessarily.

## 15.6 Analysis

```bash
ros2 run billiebot_sensor_tests analyze_sensor_nano_imu \
  --results-dir "$RESULTS" \
  --profile ekf
```

Also inspect logs:

```bash
grep -Ei 'error|warn|transform|timeout|nan|imu' "$RESULTS/console.log"
```

## 15.7 Acceptance

| Criterion | Acceptance |
|---|---|
| `/imu/data` type | `sensor_msgs/msg/Imu` |
| frame ID | `imu_link` |
| units | rad/s and m/s² |
| acceleration convention | specific force including gravity |
| quaternion norm | 0.98–1.02 |
| IMU mean rate | ≥ 45 Hz |
| IMU max gap | ≤ 0.10 s |
| `/odometry/filtered` publishes | yes |
| EKF output mean rate | ≥ 27 Hz provisional |
| EKF output finite | 100 % |
| TF/IMU transform errors | none sustained |
| yaw response | correct sign and clearly follows hand rotation |
| return to initial yaw | returns toward starting value without gross discontinuity |

This is a **compatibility** test, not an EKF-accuracy qualification.

---

# 16. UT-BAT-01 — Battery Divider Acquisition and Accuracy

## 16.1 Goal

Verify the complete measurement path:

```text
bench PSU
   ->
production voltage divider
   ->
Sensor Nano A0
   ->
raw ADC telemetry
   ->
Jetson Sensor Nano bridge
   ->
/battery_state
```

against DMM ground truth.

The test specifically looks for:

- wrong divider ratio;
- wrong ground reference;
- ADC saturation;
- nonlinearity;
- incorrect ADC-reference assumption;
- poor publication rate;
- excessive bias around the 10.5 V SAFE threshold.

## 16.2 Preflight — power OFF

Follow Section 8.

Record:

```text
R_TOP =
R_BOTTOM =
calculated divider_ratio =
predicted A0 at 12.6 V =
```

Measure the Nano 5 V rail while USB powered:

```text
V_5V_DMM =
```

This value is important because an ADC conversion that blindly assumes exactly 5.000 V can create systematic battery-voltage error.

## 16.3 Set the bridge calibration

For the first characterization run, configure:

```text
battery_divider_ratio = measured resistor ratio
adc_reference_voltage = measured Nano 5 V rail
```

Do not tune these values against ROS output. They must come from independent physical measurements.

## 16.4 Terminal 1 — launch

```bash
export RESULTS=~/billiebot_test_results/UT-BAT-01_$(date -u +%Y%m%dT%H%M%SZ)

ros2 launch billiebot_sensor_tests sensor_nano_battery_bench.launch.py \
  results_dir:="$RESULTS" \
  sensor_port:="$SENSOR_NANO_PORT" \
  baudrate:=115200 \
  record_bag:=true
```

This test is operator-paced; the launch should remain active until Ctrl-C or until all configured points are completed.

## 16.5 Terminal 2 — live topics

```bash
ros2 topic echo /battery_state
```

In another terminal:

```bash
ros2 topic hz /battery_state
```

Optional raw ADC:

```bash
ros2 topic echo /bench/battery/adc
```

## 16.6 Voltage sweep

Use the PSU current limit established in Section 8.

At each setpoint:

1. set the PSU;
2. wait 5 s;
3. measure actual `V_BAT_DMM`;
4. measure `V_A0_DMM`;
5. record the point;
6. hold for at least 10 s before moving to the next point.

Recommended points:

```text
9.90 V
10.30 V
10.50 V
10.70 V
11.10 V
12.00 V
12.60 V
```

The 10.30/10.50/10.70 V cluster intentionally characterizes the safety-threshold region.

### Ground-truth recording command

For each point use the planned helper:

```bash
ros2 run billiebot_sensor_tests record_battery_point \
  --results-dir "$RESULTS" \
  --setpoint-v <PSU_SETPOINT> \
  --dmm-battery-v <V_BAT_DMM> \
  --dmm-a0-v <V_A0_DMM>
```

Example only:

```bash
ros2 run billiebot_sensor_tests record_battery_point \
  --results-dir "$RESULTS" \
  --setpoint-v 10.50 \
  --dmm-battery-v 10.497 \
  --dmm-a0-v 1.749
```

After the last point, Ctrl-C Terminal 1 cleanly.

## 16.7 Analysis

```bash
ros2 bag info "$RESULTS/bag"
```

Then:

```bash
ros2 run billiebot_sensor_tests analyze_sensor_nano_battery \
  --results-dir "$RESULTS"
```

Expected plots:

- ROS voltage vs DMM voltage;
- error vs DMM voltage;
- ADC count vs DMM voltage;
- divider-node voltage vs battery voltage;
- time history around each point.

Expected metrics:

- measured divider ratio;
- linear-fit slope;
- linear-fit intercept;
- RMSE;
- mean bias;
- maximum absolute error;
- error specifically at/near 10.5 V;
- publication rate;
- maximum gap;
- raw-ADC monotonicity.

## 16.8 Acceptance

| Criterion | Acceptance |
|---|---|
| divider wiring | no overvoltage or abnormal current |
| measured divider ratio | within ±2 % of configured ratio, or configuration updated to independently measured ratio |
| ADC range | 0–1023, never saturated in test range |
| ADC monotonicity | strictly/nondecreasing with increasing input, allowing only quantization noise |
| `/battery_state` mean rate | ≥ 4 Hz if configured at 5 Hz |
| maximum battery-message gap | ≤ 0.5 s |
| ROS voltage vs DMM max absolute error | ≤ 0.20 V provisional |
| ROS voltage RMSE | ≤ 0.15 V provisional |
| absolute error near 10.5 V | ≤ 0.20 V |
| gross nonlinearity | none |
| serial CRC/parser errors | ≤ 0.1 % |

If the error exceeds the limit, do **not** adjust the SAFE threshold to compensate. Correct the divider ratio, ADC-reference calibration, grounding, or hardware design.

---

# 17. UT-BAT-02 — Low-Battery SAFE Propagation

## 17.1 Goal

Verify the software path:

```text
bench PSU
  -> divider
  -> Sensor Nano
  -> /battery_state
  -> production mission_controller
  -> /billiebot/mission_status
  -> SAFE
```

The Motor Nano and `base_bridge` remain disconnected.

## 17.2 Why the motor bridge is excluded

The current `base_bridge` still owns a battery timer and publishes `/battery_state` from the motor Arduino A0 path. Running it simultaneously with the new Sensor Nano bridge would create an invalid/ambiguous test and may eventually create duplicate production publishers.

This must be migrated later.

## 17.3 Initial PSU condition

Set:

```text
10.70 V
```

Confirm with DMM.

This is intentionally above the 10.5 V SAFE threshold.

## 17.4 Terminal 1 — launch Sensor Nano + production mission controller

```bash
export RESULTS=~/billiebot_test_results/UT-BAT-02_$(date -u +%Y%m%dT%H%M%SZ)

ros2 launch billiebot_sensor_tests sensor_nano_battery_safe_bench.launch.py \
  results_dir:="$RESULTS" \
  sensor_port:="$SENSOR_NANO_PORT" \
  baudrate:=115200 \
  duration_sec:=90 \
  record_bag:=true
```

The bench launch shall start the **real** `billiebot_mission` `mission_controller` using the production mission parameters, not a reimplemented mock state machine.

## 17.5 Terminal 2 — place mission in PATROL

```bash
source /opt/ros/humble/setup.bash
source ~/billie-bot-claude/billiebot_ws/install/setup.bash

ros2 service call /set_mode billiebot_interfaces/srv/SetMode "{mode: 1}"
```

Expected:

```text
success: true
message: Mode set to PATROL
```

Verify:

```bash
ros2 topic echo /billiebot/mission_status --once
```

Expected mode:

```text
mode: 1
```

and battery voltage near 10.7 V.

## 17.6 Terminal 3 — monitor

```bash
ros2 topic echo /battery_state
```

and/or:

```bash
ros2 topic echo /billiebot/mission_status
```

## 17.7 Manual action

1. Hold 10.70 V for at least 15 s.
2. Verify the mission remains non-SAFE.
3. Record the actual DMM voltage.
4. Change the PSU to approximately **10.30 V**.
5. Start timing at the PSU step.
6. Observe `/battery_state`.
7. Observe `/billiebot/mission_status`.
8. Verify transition to mode 5 / SAFE.
9. Leave the supply at 10.30 V for at least 15 s.
10. Return the PSU to 10.70 V only after the evidence is captured.
11. Do not expect automatic exit from SAFE unless the mission design explicitly implements it.

## 17.8 Analysis

```bash
ros2 run billiebot_sensor_tests score_battery_safe \
  --results-dir "$RESULTS" \
  --high-voltage-v 10.70 \
  --low-voltage-v 10.30 \
  --safe-threshold-v 10.50
```

Metrics:

- last non-SAFE battery value;
- first below-threshold battery value;
- first SAFE timestamp;
- propagation latency from below-threshold measurement to SAFE;
- mission status publication continuity.

## 17.9 Physical-path acceptance

| Criterion | Acceptance |
|---|---|
| at ~10.70 V | mission remains non-SAFE |
| measured battery falls clearly below 10.5 V | `/battery_state` reflects it |
| SAFE transition | occurs |
| SAFE mode value | `5` |
| below-threshold-to-SAFE latency | ≤ 2.0 s provisional |
| mission-status stream | continuous |
| no unrelated Nav2/motor hardware required | yes |

---

# 18. UT-BAT-02B — Exact 10.5 V Software Boundary Check

## 18.1 Purpose

The system requirement says BillieBot shall enter SAFE at:

```text
≤ 3.5 V/cell
```

For 3S:

```text
≤ 10.5 V
```

The current mission controller uses:

```python
if self._battery_voltage < self.battery_safe_voltage:
```

which is a strict `<` comparison.

Therefore exactly 10.5 V does not currently satisfy the software condition.

The physical ADC path is not the correct way to test equality because analog uncertainty and ADC quantization obscure exact 10.500 V semantics.

## 18.2 Planned software-only check

The bench scorer shall include or launch a synthetic `BatteryState` publisher directly against the real mission controller and test:

```text
10.5001 V -> not SAFE
10.5000 V -> SAFE required by SYS-PLT-2
10.4999 V -> SAFE
```

Expected current result before the production bug is fixed:

```text
10.5000 V -> FAIL
```

This expected failure is a **production requirement discrepancy**, not a Sensor Nano hardware failure.

UT-BAT-02 shall report the hardware safety propagation separately from UT-BAT-02B so the reason is explicit.

---

# 19. Result Review Checklist

After every test:

```bash
find "$RESULTS" -maxdepth 2 -type f | sort
```

Verify at least:

```text
manifest.yaml
console.log
bag/...
exports/...
metrics.json
metrics.csv
report.md
```

Check bag:

```bash
ros2 bag info "$RESULTS/bag"
```

Read report:

```bash
cat "$RESULTS/report.md"
```

Read machine verdict:

```bash
python3 -m json.tool "$RESULTS/metrics.json"
```

Never delete a failed run until the cause is understood and a replacement run has been completed.

---

# 20. Troubleshooting

## 20.1 No serial device appears

Run:

```bash
lsusb
dmesg -T | tail -n 60
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

Check:

- cable is a data cable;
- Nano power LED;
- Jetson USB port;
- CH340/USB-serial enumeration;
- permissions.

## 20.2 Serial port exists but ROS cannot open it

Run:

```bash
lsof "$(readlink -f "$SENSOR_NANO_PORT")"
```

Close:

- Arduino Serial Monitor;
- `screen`;
- previous Python serial process;
- old ROS node.

Check `dialout` group membership.

## 20.3 Two Nano devices become ambiguous

This is a known integration risk.

The existing Motor Nano path is currently based on a generic CH340-style `/dev/serial/by-id` identifier. Two similar Nano/CH340 devices may not expose distinct serial numbers.

For bench work, connect only the Sensor Nano.

For final integration, use one of:

1. stable `/dev/serial/by-path/...` identities tied to physical Jetson USB ports;
2. udev rules creating explicit names such as:

```text
/dev/billiebot_motor
/dev/billiebot_sensors
```

3. USB-serial hardware with unique serial numbers.

Do not rely on `/dev/ttyUSB0` versus `/dev/ttyUSB1`.

## 20.4 BNO055 not detected

Expected address:

```text
0x28
```

Check:

- VCC;
- GND;
- A5 -> C/SCL;
- A4 -> D/SDA;
- no swapped C/D wires;
- I2C_ADDR pin left at default.

If an I²C scanner is incorporated in firmware diagnostics, the startup report should explicitly list detected addresses.

## 20.5 BMP280 not detected

Expected address:

```text
0x76
```

If BNO055 works but BMP280 does not, the shared I²C wiring is probably healthy; inspect BMP280 initialization/library behavior and the SEN0253 itself.

## 20.6 Quaternion looks valid but EKF behaves badly

Check in this order:

1. quaternion norm;
2. frame ID;
3. axis mapping;
4. quaternion/world-frame convention;
5. angular-velocity sign;
6. accelerometer convention;
7. static TF;
8. EKF configuration.

Do not “fix” this by arbitrarily changing covariance until the units/frame conventions are verified.

## 20.7 Stationary acceleration is near zero

This is a major warning.

The bridge/firmware may be publishing the BNO055 gravity-removed linear-acceleration vector.

For the planned EKF configuration, `/imu/data.linear_acceleration` must contain accelerometer specific force including gravity.

## 20.8 Stationary acceleration is near 9.8 m/s² but sign is wrong

Investigate the sensor-frame orientation and ROS axis conversion.

Do not invert individual axes ad hoc without documenting the complete right-handed transform.

## 20.9 IMU rate below 45 Hz

Check:

- excessive serial text length;
- blocking BNO/BMP library calls;
- serial baud;
- CRC/log-print overhead;
- repeated initialization;
- Python parser performance.

Pressure updates must not block the 50 Hz IMU schedule.

## 20.10 Battery voltage is consistently high/low

Measure independently:

1. actual `V_BAT_DMM`;
2. actual `V_A0_DMM`;
3. Nano 5 V/AVcc rail;
4. actual resistor values.

Then compare:

```text
expected_ADC = V_A0_DMM / V_REF * 1023
```

Do not compensate by moving the 10.5 V SAFE threshold.

## 20.11 Battery reading is noisy

Check:

- common ground;
- loose breadboard connections;
- divider resistor values;
- A0 wire length;
- USB noise;
- firmware ADC averaging.

A small capacitor across `R_BOTTOM` / A0-to-GND may later be considered if necessary, but it should be documented as a hardware change and followed by a retest.

## 20.12 Mission does not enter SAFE at exactly 10.5 V

This is an already-identified software issue. The current code uses `< 10.5`, while SYS-PLT-2 requires `≤ 10.5`.

Do not debug the Sensor Nano if the exact-equality subtest fails for this reason.

## 20.13 More than one `/battery_state` publisher exists

Run:

```bash
ros2 topic info /battery_state --verbose
```

During these bench tests there should be one battery publisher from the Sensor Nano bridge.

If `base_bridge` is also publishing, stop it.

---

# 21. Important Findings and Blockers Discovered During Planning

These are not reasons to abandon the bench campaign. They are explicit follow-on items that the campaign should preserve and help resolve.

## BLK-01 — Production Sensor Nano bridge does not yet exist

The current repository has no dedicated production node that owns:

```text
Sensor Nano -> /imu/data + /battery_state
```

### Bench disposition

Implement a bridge in `billiebot_sensor_tests` that obeys the intended production contract.

### Later production action

Promote/reuse the tested protocol/parser/ROS logic in a production package, likely `billiebot_base` or a dedicated platform-sensors package.

---

## BLK-02 — Current `base_bridge` still owns battery acquisition

Current `billiebot_base/base_bridge.py`:

- creates `/battery_state`;
- polls Arduino analog pin A0;
- assumes a battery divider;
- assumes a 5.0 V ADC reference.

That behavior belongs to the new Sensor Nano architecture, not the Motor Nano.

### Bench disposition

Do not launch `base_bridge` in UT-BAT-01/02.

### Later production action

Disable/remove/migrate Motor Nano battery polling before full two-Nano integration.

---

## BLK-03 — `base_driver.yaml` still describes the old one-Nano architecture

It currently contains:

```text
use_imu: false
battery_pin
battery_divider_ratio
battery thresholds
```

and comments that IMU support requires rewiring the right encoder from A4/A5.

That rewire is obsolete under the two-Nano architecture.

### Later action

Update the production configuration after the Sensor Nano architecture is implemented.

---

## BLK-04 — Production EKF does not yet consume IMU data

The current `billiebot_navigation/config/ekf.yaml` has the intended `/imu/data` block commented.

### Bench disposition

Use a separate bench-only EKF YAML.

### Later production action

After UT-IMU-02 passes, update the production EKF configuration and rerun integration tests.

---

## BLK-05 — Exact SAFE threshold inequality does not meet SYS-PLT-2

System requirement:

```text
SAFE at <= 3.5 V/cell
```

Current mission controller:

```python
battery_voltage < battery_safe_voltage
```

### Consequence

Exactly 10.5 V does not trigger SAFE.

### Later action

Change the production comparison to satisfy the requirement and add regression coverage.

---

## BLK-06 — Current battery health mapping uses `COLD` for low voltage

The current `base_bridge` maps low battery voltage to a `BatteryState` health value associated with cold temperature.

This is semantically misleading.

### Later action

The new platform sensor bridge should publish appropriate standard battery status/health semantics and allow mission logic to make voltage-threshold decisions explicitly.

---

## BLK-07 — Actual battery-divider resistor values are undocumented

The repository states a nominal divider ratio of 6.0 but does not record the physical resistor values.

### Bench disposition

Measure and record both resistors before energizing the full test voltage.

### Later action

Update hardware documentation/BOM and production calibration.

---

## BLK-08 — ADC-reference accuracy is not characterized

The old software assumes:

```text
VREF = 5.000 V
```

An Arduino Nano using AVcc as the ADC reference measures relative to the actual 5 V rail, which may not be exactly 5.000 V.

### Bench disposition

Measure Nano 5 V/AVcc with a DMM and quantify resulting error.

### Later action

Choose whether production should use:
- calibrated AVcc;
- another voltage reference;
- a calibrated scale factor;
- or a more precise ADC/reference design.

---

## BLK-09 — Two CH340 Nano devices may not have unique USB identities

The Motor Nano currently uses a generic CH340-style serial identifier. A second Nano may create ambiguous `/dev/serial/by-id` naming.

### Bench disposition

Only the Sensor Nano is connected.

### Later action

Establish deterministic `/dev/billiebot_motor` and `/dev/billiebot_sensors` naming, preferably via physical USB path/udev or unique serial hardware.

---

## BLK-10 — System design documentation still models one Arduino

The current system design allocates motor control, IMU, and battery ADC to a single Nano.

### Later action

Update the system architecture/model to include:
- Motor Control Unit;
- Platform Sensor Acquisition Unit;
- separate Jetson serial interfaces.

---

## BLK-11 — IMU URDF comments still reference the A4/A5 encoder conflict

The `imu.xacro` comment still says the right encoder must be rewired for I²C.

### Later action

Update documentation after the two-Nano design is committed.

The `imu_link` frame itself remains useful.

---

## BLK-12 — Physical IMU mounting transform is not finalized

The actual final position/orientation of the SEN0253 on BillieBot has not yet been measured.

### Bench disposition

Use an identity `base_link -> imu_link` test transform in the controlled bench fixture.

### Later action

Measure and update the real URDF transform before robot-level fusion testing.

---

## BLK-13 — BNO055 magnetic heading on the assembled robot is unqualified

Motors, speaker magnets, high-current wiring, and steel hardware can disturb the magnetometer.

### Bench disposition

Do not gate UT-IMU-01 on absolute compass heading.

### Later action

Perform an installed magnetic-environment characterization before relying strongly on absolute yaw.

---

# 22. Explicit Non-Scope

These tests do not verify:

- motor control;
- encoder counts;
- wheel odometry accuracy;
- vehicle motion;
- Nav2;
- SLAM;
- localization accuracy;
- full robot TF tree;
- full wiring harness;
- fused wheel+IMU navigation accuracy;
- long-term IMU drift;
- magnetometer accuracy throughout the apartment;
- LiPo capacity/endurance;
- LiPo cell balancing;
- state-of-charge estimation;
- battery percentage;
- power-distribution bus current capacity;
- motor electrical-noise susceptibility.

Those are later integration/system tests.

---

# 23. Recommended Test Order

Run in this order:

```text
1. Flash Sensor Nano firmware
2. Serial preflight
3. UT-IMU-01
4. Resolve any IMU hardware/protocol issues
5. UT-IMU-02
6. Resolve ROS/frame/EKF issues
7. Measure divider resistors
8. Low-voltage divider safety preflight at 3 V
9. UT-BAT-01
10. Resolve ADC/calibration issues
11. UT-BAT-02 physical SAFE test
12. UT-BAT-02B exact threshold software check
```

Do not proceed to the safety-chain test until UT-BAT-01 demonstrates that the measured voltage is trustworthy around 10.5 V.

---

# 24. Campaign Completion Criteria

This bench campaign is complete when:

1. UT-IMU-01 passes.
2. UT-IMU-02 passes using the intended `/imu/data` contract.
3. UT-BAT-01 passes the provisional ±0.20 V voltage-accuracy gate.
4. UT-BAT-02 demonstrates the physical low-voltage-to-SAFE path.
5. UT-BAT-02B either passes or clearly records the known exact-boundary production defect.
6. all test runs contain rosbag2, manifest, metrics, and report artifacts.
7. the final Sensor Nano serial protocol is documented.
8. the divider values and ADC reference measurements are recorded.
9. the Sensor Nano USB identity/path is recorded.
10. all production blockers in Section 21 have explicit disposition before full robot integration.

---

# 25. Source Basis

This plan was derived from the current BillieBot repository and the existing BillieBot sensor bench-test methodology, including:

- `billiebot_ws/src/billiebot_base/billiebot_base/base_bridge.py`
- `billiebot_ws/src/billiebot_base/config/base_driver.yaml`
- `billiebot_ws/src/billiebot_navigation/config/ekf.yaml`
- `billiebot_ws/src/billiebot_mission/billiebot_mission/mission_controller.py`
- `billiebot_ws/src/billiebot_mission/config/mission.yaml`
- `billiebot_ws/src/billiebot_description/urdf/imu.xacro`
- `docs/md/MEASURE_ME.md`
- `docs/md/BillieBot_System_Design.md`
- existing `BILLIEBOT_SENSOR_BENCH_TEST_PLAN.md`
- DFRobot SEN0253 documentation
- ROS REP-145 IMU conventions
- `robot_localization` state-estimation documentation

---

# 26. Approval Gate

**Do not implement this plan yet.**

After operator review:

- revise this document if requested;
- obtain explicit approval;
- then create a separate Claude Code Opus 5 implementation prompt.

That implementation prompt shall instruct Claude Code to:
- implement the Sensor Nano firmware;
- implement the bench ROS bridge, launch files, analyses, and tests;
- use the existing `billiebot_sensor_tests` common infrastructure where appropriate;
- add required `setup.py` console-script entries if necessary;
- preserve and report production blockers rather than silently changing unrelated production behavior unless explicitly authorized.
