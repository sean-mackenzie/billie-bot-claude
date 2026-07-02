# BillieBot — Physical Measurements & Hardware TODOs

This document lists all physical values that need to be measured/verified on the
actual robot and updated in the software configuration.

## Sensor Mount Positions

All mount positions in `billiebot_description/urdf/billiebot.urdf.xacro` are
marked with `TODO(measure)` comments. Measure from the chassis link origin
(rear-center at axle height).

| Sensor | Current Placeholder (xyz) | File | Notes |
|--------|---------------------------|------|-------|
| OAK-D Lite | `0.10325 -0.115 0.1235` | `billiebot.urdf.xacro` | Inherited from Pi Camera v2 position — OAK-D is wider |
| NoIR Camera | `0.10325 0.115 0.1235` | `billiebot.urdf.xacro` | Mirror of OAK-D — verify actual mount |
| MLX90640 thermal | `0.22 0.0 0.11` | `billiebot.urdf.xacro` | Estimate — measure actual mount + tilt angle |
| ReSpeaker mic array | `0.09 0.0 0.18` | `billiebot.urdf.xacro` | Top of chassis, centered — verify height |
| BNO055 IMU | `0.0 0.0 0.05` | `billiebot.urdf.xacro` | Approximate — measure when installed |

## Encoder Calibration

| Parameter | Current Value | File | Notes |
|-----------|---------------|------|-------|
| `encoder_ticks_per_rev` | 2000.0 | `base_driver.yaml` | Reference had alternatives: 1974.7, 1779.0. Calibrate by driving 1m straight and computing actual ticks/rev |
| `wheel_radius` | 0.034 m | `base_driver.yaml` | Measure tire under load |
| `wheel_separation` | 0.298 m | `base_driver.yaml` | Measure center-to-center of tire contact patches |

## Battery Voltage Divider

| Parameter | Current Value | File | Notes |
|-----------|---------------|------|-------|
| `battery_divider_ratio` | 6.0 | `base_driver.yaml` | Design doc specifies 1/6 divider. Verify actual resistor values: R1/(R1+R2) |
| `battery_pin` | 0 (A0) | `base_driver.yaml` | Verify which analog pin the divider is connected to |
| `battery_cell_count` | 3 | `base_driver.yaml` | 3S LiPo |
| `battery_low_voltage` | 10.5V | `base_driver.yaml` | 3.5V/cell * 3 |
| `battery_critical_voltage` | 9.9V | `base_driver.yaml` | 3.3V/cell * 3 |

## IMU Hardware Rewire (Required for use_imu:=true)

**Problem:** The right encoder uses pins A4/A5 (PC4/PC5) on the Arduino Nano.
These are also the Nano's I2C pins (SDA/SCL). The BNO055 IMU cannot coexist
with the current encoder wiring.

**Solution:**
1. Rewire right encoder from A4/A5 to D4/D7 (or other free PORTD pins)
2. Update Arduino firmware with new pin-change interrupt configuration
3. Set `use_imu:=true` in `base_driver.yaml`
4. Flash firmware with BNO055 I2C support (`'i'` command)

**Impact:** Until rewired, run with `use_imu:=false` (default). The EKF still
provides value with wheel odometry alone.

## Motor Signs

| Parameter | Notes |
|-----------|-------|
| `left_motor_sign` | Set to -1.0 if left motor spins backward when given positive command |
| `right_motor_sign` | Set to -1.0 if right motor spins backward |
| `left_encoder_sign` | Set to -1.0 if left encoder counts backward |
| `right_encoder_sign` | Set to -1.0 if right encoder counts backward |

Test with `ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}"` —
robot should drive forward. Adjust signs if it doesn't.

## Room Boundaries

Update `billiebot_cognition/config/rooms.yaml` with actual map-frame coordinates
after mapping the apartment. The current values are placeholders.

## CycloneDDS Network

Update `billiebot_bringup/config/cyclonedds.xml` with actual IP addresses of:
- Jetson Orin Nano (currently `192.168.1.100`)
- Raspberry Pi 4 (currently `192.168.1.101`)

## Patrol Waypoints

Update `billiebot_navigation/config/patrol_waypoints.yaml` with actual map-frame
poses after mapping the apartment. Run SLAM, save the map, then set waypoints
using RViz's "Publish Point" tool.
