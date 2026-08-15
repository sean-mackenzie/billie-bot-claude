# Sensor Nano firmware

Production-candidate platform-sensor acquisition firmware for the BillieBot **Sensor Nano**
— the second Arduino Nano V3 in the two-Nano architecture, carrying a DFRobot SEN0253
(BNO055 + BMP280) and the battery-sense divider.

The verification spec is
[`docs/md/BILLIEBOT_IMU_BATTERY_BENCH_TEST_PLAN.md`](../../../../../docs/md/BILLIEBOT_IMU_BATTERY_BENCH_TEST_PLAN.md).
This README covers building and flashing; that document covers the test campaign.

> The **Motor Nano** runs a different sketch (`ROSArduinoBridge`) at 57600 baud and is not
> touched by this firmware or by the bench campaign. Do not flash this sketch onto it.

---

## What this board does, and deliberately does not

Acquires and streams. It contains **no** mission logic, no SAFE-state logic, no motor
commands, and no motor watchdog — there are no actuators here.

It also contains **no calibration constants**: not the divider ratio, not the ADC reference
voltage, not the 10.5 V threshold. It emits the raw averaged ADC count and the Jetson bridge
owns every conversion. A calibration constant baked into a device that needs a reflash to
change is a calibration constant nobody will ever fix.

---

## Wiring

| Sensor Nano | SEN0253 marking | Function |
|---|---|---|
| **5V** | VCC / + | sensor power |
| **GND** | GND / − | ground |
| **A5** | **C** | I²C SCL |
| **A4** | **D** | I²C SDA |
| **A0** | — | battery-divider midpoint |

Default I²C addresses: BNO055 `0x28`, BMP280 `0x76`.

The Nano is powered from the Jetson USB connection. **Do not also feed external 5 V into the
Nano's 5V pin during these tests**, and never connect the bench supply's positive output to
5V or VIN — it drives the divider input only.

For IMU-only work with the divider disconnected, fit a temporary ~10 kΩ pull-down from A0 to
GND rather than leaving A0 floating.

---

## Toolchain and libraries

Verified working on 2026-08-14 with the versions below. `arduino-cli lib list` reproduces
the library table.

| Component | Version |
|---|---|
| `arduino-cli` | 1.5.1 |
| `arduino:avr` core | 1.8.8 |
| Adafruit BNO055 | 1.6.4 |
| Adafruit BMP280 Library | 3.0.0 |
| Adafruit BusIO | 1.17.4 |
| Adafruit Unified Sensor | 1.1.15 |

Install:

```bash
arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli lib install "Adafruit BNO055" "Adafruit BMP280 Library" \
                        "Adafruit Unified Sensor" "Adafruit BusIO"
```

`arduino:avr` 1.8.8 matters specifically for `Wire.setWireTimeout()` /
`Wire.getWireTimeoutFlag()`, which is how I²C transactions are bounded and how bus faults are
counted. Older cores lack it.

### Compile

```bash
cd ~/billie-bot-claude/billiebot_ws/src/billiebot_sensor_tests/firmware/sensor_nano

arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328 .
```

Measured on the versions above:

```text
Sketch uses 17426 bytes (56%) of program storage space. Maximum is 30720 bytes.
Global variables use 760 bytes (37%) of dynamic memory, leaving 1288 bytes for
local variables. Maximum is 2048 bytes.
```

Comfortable headroom on both. Compiles clean with `--warnings all`.

The compile is also the sketch's own regression test. A block of `static_assert`s pins the
BNO055 register addresses to the datasheet map, the little-endian two's-complement decode to
its full-scale and sign cases, and the accelerometer register to `ACC_DATA_X_LSB` rather
than the linear-acceleration registers. They cost no flash and no SRAM, and a byte-order,
sign, register or gravity-source regression fails this compile instead of reaching the bench.

### Upload

```bash
export SENSOR_NANO_FLASH_PORT=<detected-serial-port>   # arduino-cli board list

arduino-cli upload -p "$SENSOR_NANO_FLASH_PORT" \
  --fqbn arduino:avr:nano:cpu=atmega328 .
```

Many Nano clones ship the **old** bootloader; if upload synchronisation fails
(`stk500_recv(): programmer is not responding`), rebuild and upload with:

```bash
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328old .
arduino-cli upload -p "$SENSOR_NANO_FLASH_PORT" --fqbn arduino:avr:nano:cpu=atmega328old .
```

After flashing, **close every serial monitor before starting ROS** — only one process may
own the port.

---

## Serial protocol

115200 baud, 8N1, newline-delimited ASCII. The Jetson-side parser is
[`billiebot_sensor_tests/sensor_nano/protocol.py`](../../billiebot_sensor_tests/sensor_nano/protocol.py);
the two are tested against each other and must be changed together.

```text
<TYPE>,<seq>,<t_us>,<fields...>*<CRC>\n
```

| Type | Rate | Fields after `seq,t_us` |
|---|---:|---|
| `I` | 50 Hz | `qw,qx,qy,qz,gx,gy,gz,ax,ay,az,cal_sys,cal_gyr,cal_acc,cal_mag` |
| `M` | 10 Hz | `mx,my,mz` |
| `B` | 5 Hz | `adc_mean` |
| `P` | 2 Hz | `pressure_pa,temperature_c` |
| `S` | 1 Hz | `bno_ok,bmp_ok,fusion_mode,i2c_errors,imu_read_errors,bmp_errors,reinits,imu_records_dropped` |

Units on the wire: quaternion dimensionless, gyro **rad/s**, acceleration **m/s² including
gravity**, magnetometer **microtesla**, pressure Pa, temperature °C, battery **raw averaged
ADC count** (never volts).

`seq` is a single `uint32` counter shared by *all* record types, so one dropped frame of any
kind shows up as one global discontinuity on the Jetson. `t_us` is `micros()` and wraps every
~71.6 minutes; the parser unwraps it and never treats it as epoch time.

### CRC

CRC-16/CCITT-FALSE — polynomial `0x1021`, init `0xFFFF`, no input/output reflection, no final
XOR — over the payload bytes preceding the `*`, rendered as four uppercase hex digits.

The canonical check value `crc16("123456789") == 0x29B1` is asserted **at boot in this
firmware** and **in the Python test suite**, so the two implementations cannot drift apart
unnoticed. A boot-time mismatch prints `E,crc16_self_check_failed`.

### Bandwidth

Measured with realistic field widths (7-digit `seq`, 9-digit `t_us`, signed values):

| Record | Bytes | Rate | Throughput |
|---|---:|---:|---:|
| `I` | 110 | 50 Hz | 5500 B/s |
| `M` | 49 | 10 Hz | 490 B/s |
| `B` | 32 | 5 Hz | 160 B/s |
| `P` | 40 | 2 Hz | 80 B/s |
| `S` | 60 | 1 Hz | 60 B/s |
| | | **total** | **6290 B/s** |

115200 8N1 carries 11520 B/s, so the offered load is **≈55 %**. The link drains faster than
the schedule refills it, which is why the blocking `Serial.write()` in `emitRecord()` is
bounded and non-cumulative. See BLK-14 in the test plan for the margin discussion; set
`ENABLE_MAGNETOMETER 0` to reclaim ~8 % if a future record set needs it.

---

## Coordinate frames and units — read before changing anything

**Acceleration includes gravity.** The firmware sends `VECTOR_ACCELEROMETER` (specific force,
~9.8 m/s² at rest), *not* `VECTOR_LINEARACCEL`. The BillieBot `robot_localization` config
sets `imu0_remove_gravitational_acceleration: true`, so sending an already gravity-removed
vector would remove gravity twice. A stationary magnitude near zero is the signature of that
mistake.

**No axis remap, no world-frame conversion.** The BNO055 stays at its default P1 axis
placement and the quaternion is transmitted exactly as the chip fuses it. Any frame
conversion is an explicit, opt-in bridge parameter (`orientation_frame_convention`), so the
convention lives in configuration rather than buried in a sketch. The chip's fusion world
frame is not yet verified on this hardware — see BLK-14.

**Unit conversions.** The IMU path reads the BNO055 data registers directly (see *Fault
behaviour* for why); the scales are transcribed from the installed `Adafruit_BNO055.cpp`
`getQuat()`/`getVector()` bodies and datasheet §3.6.4 rather than assumed:

| Source | Raw | Firmware sends |
|---|---|---|
| `QUA_DATA_W_LSB` (`0x20`, 8 B) | scale 1/2¹⁴, dimensionless | as-is |
| `GYR_DATA_X_LSB` (`0x14`, 6 B) | **deg/s** (1 dps = 16 LSB) | × `DEG_TO_RAD` → rad/s |
| `ACC_DATA_X_LSB` (`0x08`, 6 B) | m/s² (1 m/s² = 100 LSB) | as-is |
| `getVector(VECTOR_MAGNETOMETER)` | microtesla (1 µT = 16 LSB) | as-is (bridge → tesla) |

Every one of those registers is little-endian two's-complement 16-bit; `bnoInt16()` does the
decode and `static_assert` pins its byte order and sign handling at compile time.

Those divisors are only correct because the library's `begin()` leaves `BNO055_UNIT_SEL` at
its power-on default — the write is commented out upstream. **If a future library version
starts writing `UNIT_SEL`, re-verify the gyro unit before trusting this firmware.**

### Fusion mode

`OPERATION_MODE_NDOF` (9-DoF absolute orientation) by default, so magnetometer calibration
exists as an informational metric. Change `BNO055_FUSION_MODE` to `OPERATION_MODE_IMUPLUS`
for gyro+accel-only relative heading that is immune to motor/speaker magnetics (BLK-13). The
**active** mode is read back from the chip and reported in every `S` record, so changing this
can never silently misrepresent a captured run.

---

## Fault behaviour

| Concern | Handling |
|---|---|
| I²C hang | `Wire.setWireTimeout(25000, true)` bounds every transaction and resets TWI on timeout |
| I²C fault counting | `Wire.getWireTimeoutFlag()` polled after each transaction → `i2c_errors` |
| Failed BNO055 read | every read on the IMU path is **transaction-checked** (`bnoRead()`); if the quaternion, gyro, accelerometer or calibration read fails, the **whole `I` record is dropped** and counted rather than transmitted |
| Zeroed BNO055 data that *did* read successfully | quaternion **norm gate** (0.90–1.10), as defence in depth behind the transaction check |
| Failed BMP280 read | plausibility-gated (30–110 kPa, −40–85 °C); dropped + counted |
| BMP280 dies | BNO055 and battery keep streaming |
| BNO055 dies | battery keeps streaming |
| Repeated failures | bounded re-`begin()` after 25 consecutive failures, ≥5 s apart, ≤20 attempts total |
| Schedule overrun | detected by the catch-up guard, counted in `imu_records_dropped` |

### Why the IMU path bypasses the Adafruit convenience methods

`Adafruit_BNO055::getQuat()` and `getVector()` zero their buffers, call the private
`readLen()`, and **discard the `bool` it returns**. A NACKed or timed-out I²C read therefore
surfaces as an all-zero quaternion or an all-zero vector — plausible-looking data, far worse
than a visible dropout. `read8()`, which `getCalibration()` uses, has the same defect plus a
worse consequence: it reuses one buffer for the register address and the reply, so a failed
read of `CALIB_STAT` decodes `0x35` into a fabricated `cal_gyr=3, cal_acc=1, cal_mag=1`.

No value-domain check can close that hole. A stationary gyro is legitimately near zero, and a
failed accelerometer read is exactly zero. `readLen()` is `private`, so the only way to see
the transaction result is to issue the register read in the sketch, which is what `bnoRead()`
does — mirroring `Adafruit_I2CDevice::write_then_read()` (address write with no stop, read
with one) and verifying the received byte count. The rule it enforces is:

> **failed transaction → drop the complete IMU sample → count it → never publish plausible
> zeros.**

A failed read increments `imu_read_errors` and the consecutive-failure counter that drives
recovery, clears `bno_ok`, and returns before a sequence number is consumed — so a dropped
sample shows up as a small rate deficit and a rising `imu_read_errors`, never as a stream
discontinuity or a zero vector. The quaternion norm gate is kept behind it because it catches
what a transaction check cannot: a chip that reset into `CONFIG` mode and *successfully*
returns real zeros.

The magnetometer still uses `getVector()`; its `M` record is gated on `bno_ok`, which the
50 Hz IMU path refreshes every 20 ms.

A re-`begin()` blocks for roughly a second. That is acceptable only because the peripheral is
already dead when it happens, and the cooldown is what stops the stall from repeating.

---

## Quick smoke test

With ROS stopped and nothing else holding the port:

```bash
python3 - <<'PY'
import os, serial, time
s = serial.Serial(os.environ["SENSOR_NANO_PORT"], 115200, timeout=1)
time.sleep(2.5)          # the Nano auto-resets when DTR is asserted on open
s.reset_input_buffer()
for _ in range(20):
    line = s.readline().decode(errors="replace").strip()
    if line:
        print(line)
s.close()
PY
```

Expect at least one `S,...` record with `bno_ok=1` and `bmp_ok=1`, and a steady stream of
`I,...`. Stop and fix the hardware before running any bench test if either flag is 0.

The same checks run automatically as the `sensor_nano` preflight
(`billiebot_sensor_tests/common/preflight.py`) and land in `<results_dir>/console.log`.
