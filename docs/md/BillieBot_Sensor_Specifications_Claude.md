# BillieBot — Sensor Specification Reference Sheet

**Revision:** 1.0
**Date:** 2026-08-02
**Prepared by:** Systems / Hardware Engineering
**Purpose:** Single-page reference for the specifications of all BillieBot sensors, with emphasis on measurement performance, electrical load, digital interface, and the structure/units of the data each device returns.

---

## Legend — Value Provenance

Every number in this document carries one of the following markers. **Unmarked values are manufacturer-stated** (datasheet, official product brief, or official vendor documentation).

| Marker | Meaning |
|---|---|
| *(no marker)* | Manufacturer-stated value from a datasheet or official vendor documentation |
| **[CALC]** | Derived arithmetically by the author from manufacturer-stated values. The derivation is shown. |
| **[EST]** | Estimated. Not published by the manufacturer; based on community measurement, engineering judgment, or a comparable part. **Verify on hardware before relying on it.** |
| **[CONFLICT]** | Two official sources disagree. Both values are shown. Requires bench verification. |

---

## 1. Sensor Suite at a Glance

| # | Sensor | Primary Measurand(s) | Interface | Supply | Typ. Power |
|---|---|---|---|---|---|
| 1 | Luxonis OAK-D Lite (Auto-Focus) | RGB image, stereo depth, 6-axis inertial, on-device NN inference | USB-C (USB 2/3) | 5 V (bus) | 4–5 W |
| 2 | Adafruit MLX90640 (55°), PID 4407 | Scene temperature (32×24 thermal array) | I²C (0x33) | 3–5 V in / 3.3 V core | ~60 mW **[CALC]** |
| 3 | RPi Camera Module 3 NoIR (75°), SC0873 | Visible + near-IR image | MIPI CSI-2 + I²C | 3.3 V (CSI) | ~0.8 W **[EST]** |
| 4 | Seeed reSpeaker XVF3800 USB 4-Mic Array | Audio, direction of arrival, voice activity | USB-C (UAC 2.0) + I²C | 5 V (bus) | ~1–2 W **[EST]** |
| 5 | DFRobot Gravity 10 DOF AHRS, SEN0253 | Orientation, accel, angular rate, magnetic field, pressure, temperature | I²C (0x28, 0x76) | 3.3–5 V | ~40–60 mW **[CALC]** |

### Bus Allocation Summary

- **USB:** 2 devices — OAK-D Lite and reSpeaker XVF3800. See §7.2 for a critical current-budget warning on the OAK-D Lite.
- **I²C:** 3 devices — MLX90640 (0x33), BNO055 (0x28), BMP280 (0x76). **No address conflicts.** However, see §7.3 for a bus-speed constraint that argues for splitting the MLX90640 onto its own bus.
- **MIPI CSI-2:** 1 device — Camera Module 3 NoIR. Consumes one CSI port on the host.

---

## 2. Luxonis OAK-D Lite — Auto-Focus Variant

**Part number:** OAK-D-LITE-AF (Luxonis SKU A00483)
**Architecture:** Intel Movidius Myriad X VPU (Luxonis RVC2 platform)
**Physical:** 91 × 28 × 17.5 mm; aluminium housing with Gorilla Glass front; ¼-20 tripod mount; VESA 75 mm M4 mounting holes

This is a **multi-output device**. It presents five distinct data streams to the host, covered in §2.1 through §2.5.

### 2.0 Device-Level Electrical & Environmental

| Parameter | Value | Notes |
|---|---|---|
| Supply | 5 V nominal via USB-C VBUS | 5.25 V max recommended; 3.5–5.5 V absolute |
| Max input current (abs max) | 1.5 A | Absolute maximum rating |
| Power consumption (min/typ/max) | 4 W / 5 W / 6 W | Recommended operating conditions |
| Idle power (VPU booted, no pipeline) | 2.5 W | |
| Typical operating current @ 5 V | ~0.8–1.0 A **[CALC]** | 4–5 W ÷ 5 V |
| Peak operating current @ 5 V | ~1.2 A **[CALC]** | 6 W ÷ 5 V |
| Idle current @ 5 V | ~0.5 A **[CALC]** | 2.5 W ÷ 5 V |
| Ambient operating temperature | −20 °C to +50 °C | At full VPU utilization |
| VPU thermal shutdown | 105 °C junction | DepthAI shuts the device down automatically |
| Host interface | USB-C, USB 2.0 / USB 3 | Luxonis docs cite up to 10 Gbps; datasheet cites USB 3.1 Gen1 |
| On-board EEPROM | 32 kb, I²C | Stores factory calibration |

**Power breakdown by subsystem (Luxonis):**

| Subsystem | Contribution |
|---|---|
| Base + camera streaming | 2.5 – 3 W |
| AI (neural inference) subsystem | up to 1 W |
| Stereo depth pipeline | up to 0.5 W |
| Video encoder | up to 0.5 W |

> **Design note:** Reducing pipeline FPS reduces power, because subsystems are no longer saturated.

### 2.1 Output A — RGB Color Camera

| Parameter | Value |
|---|---|
| Image sensor | Sony IMX214 |
| Nominal resolution | 13.1 MP; 4208 × 2160 active... **4208 × 3120** @ up to 60 fps (sensor-level) |
| Optical format | 1/3.06" (Luxonis docs) / 1/3.1" (datasheet) **[CONFLICT]** — negligible practical difference |
| Field of view (D / H / V) | 81° / 69° / 54° (Luxonis docs); datasheet states DFOV 81.3° |
| Shutter type | Rolling (electronic) |
| Focus | **Auto-focus, 8 cm → ∞**; software-settable to a fixed lens position via DepthAI API |
| Recommended close-range use | 8 cm – 50 cm (this is the reason to choose AF over FF) |
| IR sensitivity | No — IR cut filter fitted |
| Hardware encoding | H.264, H.265, MJPEG — 4K @ 30 fps, 1080p @ 60 fps |

**Data received by host:**
- **Uncompressed:** `ImgFrame` messages. Default color order is **NV12** (YUV 4:2:0 semi-planar, 8-bit). Also selectable: YUV420p, BGR/RGB 8-bit interleaved or planar, GRAY8. Each frame carries a monotonic sequence number and a device-synchronized timestamp.
- **Compressed:** H.264 / H.265 elementary stream or MJPEG frames as `ImgFrame` payloads.
- **Bandwidth, 1080p30 NV12:** 1920 × 1080 × 1.5 bytes × 30 fps ≈ **93 MB/s [CALC]** — plan for USB 3 if streaming uncompressed color at this rate.

### 2.2 Output B — Stereo Mono Cameras (Left / Right)

| Parameter | Value |
|---|---|
| Image sensor (×2) | OmniVision OV7251 |
| Active pixels | 640 × 480 @ up to 120 fps |
| Optical format | 1/7.5" |
| Field of view (D / H / V) | 86° / 73° / 58° (Luxonis docs); datasheet states 85.6° / 72.9° / 57.7° |
| Shutter type | **Global** — important for a moving platform; no rolling-shutter skew |
| Focus | Fixed, 6.5 cm → ∞ |
| Native output format | 8-bit or 10-bit RAW |
| IR sensitivity | No (per datasheet); note the OAK-D Lite has **no** IR dot projector and **no** IR illuminator |

**Data received by host:** `ImgFrame` messages, typically **GRAY8** (8-bit luminance), 640 × 480, one stream per camera.
**Bandwidth, both cameras @ 30 fps:** 640 × 480 × 1 byte × 30 × 2 ≈ **18.4 MB/s [CALC]**

### 2.3 Output C — Stereo Depth

| Parameter | Value |
|---|---|
| Stereo baseline | **75 mm** |
| Ideal depth range | ~80 cm – 12 m (Luxonis hardware docs) **[CONFLICT]** |
| Ideal depth range (alternate) | 40 cm – 8 m (Luxonis shop page) **[CONFLICT]** |
| MinZ, 480P + extended disparity | ~20 cm |
| MinZ, 480P standard | ~35–40 cm |
| Median depth accuracy, < 4 m | < 2 % absolute depth error |
| Median depth accuracy, 4 – 7 m | < 4 % absolute depth error |
| Median depth accuracy, 7 – 10 m | < 6 % absolute depth error |
| Max disparity throughput | 200+ fps (stereo pipeline capability) |

> **[CONFLICT] — action required:** Luxonis' own hardware documentation and store page state materially different "ideal range" figures (80 cm–12 m vs. 40 cm–8 m). The difference stems from which mono resolution and disparity mode is assumed. **Characterize the actual usable range on BillieBot in its final configuration** before fixing navigation or obstacle-avoidance thresholds.

**Data received by host:**
- **Depth map:** `ImgFrame`, 640 × 480, **uint16, 1 LSB = 1 mm** by default. A value of `0` means *invalid / no measurement*, not zero distance — filter these explicitly.
- **Disparity map:** `ImgFrame`, uint8 (or uint16 with subpixel enabled). Convert to depth via `Z = (baseline_mm × focal_length_px) / disparity`.
- **Confidence map:** optional uint8 per-pixel confidence.
- **Point cloud:** derivable on-device or host-side; XYZ in millimetres, optionally colorized from the RGB camera (RGB-depth alignment is supported).
- **Bandwidth, 640×480 depth @ 30 fps:** 640 × 480 × 2 bytes × 30 ≈ **18.4 MB/s [CALC]**

**Post-processing available on-device:** median filter, speckle filter, temporal filter, spatial filter, decimation, threshold filter, RGB-depth alignment.

### 2.4 Output D — On-Board IMU (Bosch BMI270)

> ⚠️ **CRITICAL PROCUREMENT NOTE:** Luxonis states that **all OAK-D-Lite units from the original Kickstarter campaign shipped WITHOUT an IMU** as a cost-reduction measure. Retail units have the BMI270. **Verify IMU presence on the specific unit used in BillieBot** by enumerating the device before designing this stream into the architecture.

| Parameter | Value |
|---|---|
| Device | Bosch BMI270, **6-axis** (3-axis accelerometer + 3-axis gyroscope) |
| Magnetometer | **None** |
| On-sensor fusion (quaternion / rotation vector) | **Not available** on the BMI270 path in DepthAI |
| Internal connection | SPI (VPU ↔ IMU); delivered to host over USB |

**Accelerometer:**

| Parameter | Value |
|---|---|
| Full-scale options | ±2 / ±4 / ±8 / ±16 g |
| Sensitivity | 16384 / 8192 / 4096 / 2048 LSB/g respectively |
| Noise density (typ) | 0.16 mg/√Hz |
| Native ODR range | 12.5 Hz – 1600 Hz |

**Gyroscope:**

| Parameter | Value |
|---|---|
| Full-scale options | ±125 / ±250 / ±500 / ±1000 / ±2000 °/s |
| Sensitivity | 262.144 / 131.072 / 65.536 / 32.768 / 16.384 LSB/(°/s) respectively |
| Noise density (typ, performance mode) | 0.007 °/s/√Hz |
| Native ODR range | up to 6400 Hz |

**DepthAI-exposed runtime behaviour (this is what you actually get):**

| Parameter | Value |
|---|---|
| Stable accelerometer request rates | 25 / 50 / 100 / 200 / 250 Hz |
| Stable gyroscope request rates | 25 / 50 / 100 / 200 / 250 Hz |
| Practical maximum report rate | Requests above 400 Hz top out at **~250 Hz** |
| Rate rounding | Requested rate rounds **down** to the next supported rate |

**Luxonis-characterized noise parameters (for filter tuning — units per Luxonis, treat as Allan-variance style parameters):**

| Axis | Accel noise density | Accel random walk | Accel bias stability | Gyro noise density | Gyro random walk | Gyro bias stability |
|---|---|---|---|---|---|---|
| X | 0.05003 | 3.734e-5 | 0.06865 | 0.29334 | 0.28111 | 9.19266 |
| Y | 0.05473 | 5.082e-5 | 0.11015 | 0.28519 | 0.52435 | 9.68103 |
| Z | 0.06361 | 2.845e-5 | 0.04465 | 0.30118 | 0.25036 | 6.93407 |

**Data received by host:** `IMUData` messages containing one or more `IMUPacket` entries. Each packet exposes:
- `acceleroMeter` → `.x`, `.y`, `.z` as **float32, m/s²**, plus `.sequence` and `.timestamp`
- `gyroscope` → `.x`, `.y`, `.z` as **float32, rad/s**, plus `.sequence` and `.timestamp`

**Report families (select at pipeline build time):**

| Suffix | Meaning |
|---|---|
| `_RAW` | Direct sensor output in the **sensor-native frame** |
| `_UNCALIBRATED` | Rotated into the **Luxonis RDF frame** using `imuExtrinsics`; no IMU calibration applied |
| `_CALIBRATED` | Luxonis RDF frame **and** IMU calibration parameters applied |

Batching is configurable via `setBatchReportThreshold(N)` and `setMaxBatchReports(M)` to trade host CPU load against latency.

**Bandwidth @ 200 Hz, accel + gyro:** ~6 floats + metadata ≈ 40 bytes × 200 ≈ **8 kB/s [EST]**

### 2.5 Output E — Neural Network Inference

| Parameter | Value |
|---|---|
| Total VPU compute | 4 TOPS |
| Neural inference allocation | 1.4 TOPS |
| Model support | Any architecture, after conversion to the Myriad X blob format |
| Additional CV nodes | Warp/undistort, resize, crop (ImageManip), edge detection, feature tracking, custom CV |
| Object tracking | 2D and 3D via ObjectTracker node |

**Data received by host:** Structured metadata messages, not images —
- `ImgDetections` / `SpatialImgDetections`: list of detections, each with `label` (int), `confidence` (float 0–1), normalized bounding box (`xmin`, `ymin`, `xmax`, `ymax` as floats 0–1), and for spatial detections a **`spatialCoordinates`** field giving **X, Y, Z in millimetres** in the camera frame.
- `NNData`: raw output tensors for custom models.
- `Tracklets`: tracking ID, status (NEW/TRACKED/LOST/REMOVED), ROI, and 3D spatial coordinates.

> **This is the highest-value output for BillieBot** — the device returns object identity *and* metric 3D position in one message, with no host-side depth lookup required.

---

## 3. Adafruit MLX90640 IR Thermal Camera Breakout — 55° (PID 4407)

**Sensor:** Melexis **MLX90640ESF-BAB-000-TU** (the `B` in the third option-code position denotes the 55° × 35° FOV lens)
**Breakout physical:** 25.7 × 17.7 × 16.0 mm; STEMMA QT / Qwiic connectors plus breadboard-friendly headers
**Package:** 4-lead TO-39 with integrated lens

### 3.1 Optical & Array

| Parameter | Value |
|---|---|
| Array format | **32 × 24 = 768 pixels** (32 columns wide, 24 rows tall) |
| Field of view | **55° (X / horizontal) × 35° (Y / vertical)**, typical |
| Central pointing error from normal | ±3° max |
| Instantaneous FOV per pixel (H) | ~1.72° **[CALC]** (55° ÷ 32) |
| Instantaneous FOV per pixel (V) | ~1.46° **[CALC]** (35° ÷ 24) |
| Ground sample at 1 m | ~3.0 cm × 2.5 cm per pixel **[CALC]** |
| Ground sample at 3 m | ~9.0 cm × 7.6 cm per pixel **[CALC]** |
| Defective pixels | Up to 4 permitted; flagged in on-chip EEPROM; replace by neighbour interpolation |

> **BillieBot implication:** At 3 m a standing adult (~50 cm wide) subtends only ~5–6 pixels horizontally. This sensor is a **presence/heat-signature detector**, not an imaging camera. Use it for human detection and hot-object avoidance, not for classification.

### 3.2 Measurement Performance

| Parameter | Value |
|---|---|
| Target (object) temperature range | **−40 °C to +300 °C** |
| Ambient / operating temperature range | −40 °C to +85 °C |
| Stated accuracy (Adafruit, product level) | **±2 °C** in the 0–100 °C range |
| Frame accuracy (datasheet) | ±1 °C (BAA reference case) |
| Non-uniformity, zone 1 (datasheet) | ±0.5 °C (BAA reference case) |
| Total pixel absolute accuracy example (datasheet) | ±1.5 °C for an 80 °C target in zone 1 (BAA) |
| **NETD @ 1 Hz refresh, BAB variant** | **0.25 K average**, 0.20 K minimum, σ = 0.05 K |
| NETD @ 1 Hz, BAA variant (for reference) | 0.14 K average |
| ADC resolution | **16 / 17 / 18 / 19-bit selectable; 18-bit default** |
| Emissivity | User-supplied software parameter; not stored in the device |

> **Accuracy caveats stated in the datasheet — all relevant to a mobile robot:**
> 1. Accuracy specifications apply **only under settled isothermal conditions**.
> 2. Accuracy is valid **only when the target completely fills the pixel FOV**. A person at 3 m does not fill a pixel; expect the reported temperature to be a blend of the person and the background.
> 3. IR sensors are inherently susceptible to **thermal gradients**. Avoid mounting near motors, the VPU, or other heat sources.
> 4. Noise increases at lower target temperatures and decreases at higher ones.
> 5. Corner pixels are noisier than centre pixels due to lens optical performance.

### 3.3 Timing

| Parameter | Value |
|---|---|
| Programmable refresh rate | 0.5 / 1 / 2 / 4 / 8 / 16 / 32 / 64 Hz — **default 2 Hz** |
| Practical maximum (Adafruit) | ~16 Hz (32 Hz is the theoretical limit but was not achievable in practice) |
| **Subpage structure** | The frame is split into **2 subpages**. The configured refresh rate is the **subpage** rate. |
| **Effective full-image rate** | **Half the configured refresh rate [CALC]** — e.g. 16 Hz setting → **8 full thermal images/second** |
| Reading pattern | **Chess (default, factory-calibrated, recommended)** or interleaved/TV mode |
| First valid data after power-on | 40 ms + (1 / refresh rate) — e.g. 540 ms at the 2 Hz default |
| Thermal stabilization to rated accuracy | **up to 4 minutes** |

> **Design note:** The 4-minute thermal settling time means BillieBot's thermal channel should not be trusted for absolute temperature immediately after power-up. Either warm up before relying on absolute readings, or use relative/differential detection during the first several minutes.

### 3.4 Electrical

| Parameter | Min | Typ | Max | Unit |
|---|---|---|---|---|
| Supply voltage (sensor, VDD) | 3.0 | **3.3** | 3.6 | V |
| **Supply current (IDD)** | 14 | **18** | 25 | mA |
| Power at 3.3 V, typ | — | **~59.4 mW [CALC]** | ~82.5 mW **[CALC]** | — |
| Breakout board input voltage | 3.0 | — | 5.0 | V (on-board regulator + level shifting) |
| Breakout board total current | — | ~20–25 mA **[EST]** | — | mA (sensor + regulator + pull-ups) |
| POR threshold (rising / falling) | 2.2 / — | 2.6 / 2.55 | — | V |

> Melexis recommends holding VDD to **3.3 V ± 0.1 V** for best performance. Decoupling: 100 nF SMD plus 1 µF ceramic close to VDD/VSS, with short traces on **both** rails.

### 3.5 Digital Interface

| Parameter | Value |
|---|---|
| Protocol | **I²C**, slave only |
| Default address | **0x33** (programmable, up to 127 addresses; 0x00 must be avoided) |
| Clock frequency | 400 kHz (standard/fast) up to **1 MHz (FM+ mode)** |
| Logic levels | SDA/SCL are **5 V tolerant** — can share a 5 V I²C network directly |
| SDA sink current limit | 10 mA (thermally limited; 20 mA is the I²C spec ceiling) |

### 3.6 Data Received from the Sensor

This is the most important section for BillieBot's software interface, because **the MLX90640 does not output temperature directly.**

**What is physically on the wire (raw layer):**

| Item | Address | Format |
|---|---|---|
| IR pixel data | RAM 0x0400 – 0x06FF | **768 × 16-bit signed (two's complement)** ADC counts, proportional to received IR energy — **not temperature** |
| `Ta_Vbe` | RAM 0x0700 | 16-bit signed — ambient sensor |
| `CP` subpage 0 | RAM 0x0708 | 16-bit signed — compensation pixel |
| `GAIN` | RAM 0x070A | 16-bit signed |
| `Ta_PTAT` | RAM 0x0720 | 16-bit signed — ambient sensor |
| `CP` subpage 1 | RAM 0x0728 | 16-bit signed |
| `VDD_pix` | RAM 0x072A | 16-bit signed — supply sensor |
| Calibration constants | EEPROM 0x2400 – 0x273F | ~832 words, read once after POR |
| Status register | 0x8000 | Bit 3 = "new data available in RAM" (**host must clear it**); bits 0–2 = last measured subpage |
| Control register 1 | 0x800D | Refresh rate, ADC resolution, reading pattern, subpage control |

**What the driver produces (application layer):**

The host must (a) read the EEPROM calibration once after POR, (b) read both subpages, and (c) run the datasheet's compensation chain — supply voltage → ambient temperature → gain → offset/VDD/Ta compensation → emissivity → gradient → sensitivity normalization → `To`. The result is:

> **A 768-element array of IEEE-754 floating-point values in degrees Celsius**, indexed row-major as a 32-wide × 24-tall image. Both the Adafruit CircuitPython and Arduino libraries return exactly this.

**Pixel addressing:** `Pix(i, j)` where `i` = row 1–24 and `j` = column 1–32.

**Bandwidth:** 768 floats × 4 bytes × 8 full-frames/s ≈ **24.6 kB/s [CALC]** on the application side; ~13.3 kbit per subpage read on the I²C wire **[CALC]**.

**Extended-range detail:** the compensation chain uses four object-temperature ranges with corner temperatures at −40 °C, 0 °C, CT3 (typ. 160 °C), and CT4 (typ. 320 °C), each with its own sensitivity slope `KsTo`. Standard driver libraries handle this transparently.

**"Image mode" shortcut:** If BillieBot only needs a relative thermal image (e.g. blob detection for a warm body) and not calibrated absolute temperature, the datasheet defines a reduced computation flow that skips the final `To` calculation — meaningfully lower host CPU cost.

---

## 4. Raspberry Pi Camera Module 3 NoIR — 75° (SC0873)

**Sensor:** Sony **IMX708**, back-illuminated stacked CMOS, Quad Bayer
**Variant:** Standard lens (75° DFOV), **no IR cut filter**

### 4.1 Imaging

| Parameter | Value |
|---|---|
| Resolution | **11.9 MP** |
| Pixel array | **4608 × 2592** |
| Pixel size | 1.4 µm × 1.4 µm |
| Sensor diagonal | 7.4 mm |
| Diagonal field of view | **75°** |
| Horizontal field of view | **66°** |
| Vertical field of view | **41°** |
| Focal length | 4.74 mm |
| Focal ratio | **f/1.8** |
| Focus range | **10 cm → ∞** |
| Autofocus system | **Phase Detection Autofocus (PDAF)** |
| Shutter type | Rolling **[EST]** — not stated in the product brief; IMX708 is a rolling-shutter sensor |
| **Infrared sensitive** | **Yes** — no IR cut filter fitted |
| HDR mode | Yes, up to 3 MP output |
| Defect correction | Built-in 2D Dynamic Defect Pixel Correction (DPC) |
| Re-mosaic | QBC (Quad Bayer Coding) re-mosaic function |

**Common video modes (manufacturer-listed):** 1080p50, 720p100, 480p120

> **BillieBot implication — NoIR colour behaviour:** Without the IR cut filter, the Bayer colour filter array no longer isolates R/G/B cleanly; near-IR leaks into all three channels. Daylight images will appear reddish/washed out, and **colour-based classification will be unreliable**. Under IR illumination, treat this camera as effectively monochrome. Its value to BillieBot is night-vision and low-light operation, not colour fidelity.

### 4.2 Electrical & Environmental

| Parameter | Value |
|---|---|
| Supply | 3.3 V, sourced from the host's CSI connector |
| **Current draw** | **~200–300 mA [EST]** — Raspberry Pi does **not** publish this. Estimate derives from Raspberry Pi forum measurements of Camera Module 2/3 and the widely-cited 250 mA figure for Pi camera modules. |
| **Power** | **~0.66 – 1.0 W [EST]** (0.25 A × ~3.3 V, with margin) |
| Operating temperature | **0 °C to 50 °C** — the narrowest of any sensor in the suite; see §7.4 |
| Dimensions | 25 × 24 × 11.5 mm |
| Ribbon cable | 200 mm, 15 × 1 mm FPC |
| Production lifetime | In production until at least January 2030 |

> ⚠️ **Verify current draw on the bench.** This is the only significant power number in the suite that is not manufacturer-published. It matters because the CSI connector's 3.3 V rail is fed from the host SBC's regulator, which has a finite budget shared with other peripherals.

### 4.3 Digital Interface

| Parameter | Value |
|---|---|
| Image data | **MIPI CSI-2 serial output** |
| Control | **2-wire serial (I²C)**, supports fast mode and fast-mode plus |
| Focus control | Separate 2-wire serial control of the focus mechanism |
| Software stack | `libcamera` / `picamera2` (fully supported, including autofocus). **Not** supported on Raspbian Buster or earlier. |

### 4.4 Data Received from the Sensor

| Layer | Format |
|---|---|
| **Sensor output (on the CSI wire)** | **RAW10** — 10-bit Bayer, CSI-2 packed (`SRGGB10_CSI2P`), 5 bytes per 4 pixels |
| **Post-ISP host formats** | YUV420, NV12, RGB888, BGR888, XRGB8888 — selectable per-stream in `libcamera` |
| **Application delivery** | `picamera2` returns frames as **NumPy arrays**; shape and dtype depend on the requested format (e.g. `(H, W, 3)` uint8 for RGB888) |
| **Metadata per frame** | Exposure time (µs), analogue/digital gain, colour temperature, lens position (dioptres), focus FoM, sensor timestamp (ns) |

**Bandwidth:**
- Full sensor read, 4608 × 2592 RAW10 ≈ 14.9 MB per frame **[CALC]**
- 1080p50 YUV420 = 1920 × 1080 × 1.5 × 50 ≈ **155 MB/s [CALC]** — host-side, post-ISP
- Common practice is the binned 2304 × 1296 mode for video, which halves the linear resolution and improves low-light SNR.

**Autofocus data:** `libcamera` exposes `AfState` (Idle / Scanning / Focused / Failed), `LensPosition` in reciprocal metres (dioptres), and `FocusFoM` (focus figure of merit). BillieBot can command `AfModeManual` with an explicit `LensPosition` to lock focus — recommended for a vibrating mobile platform, since continuous AF hunting wastes time and power.

---

## 5. Seeed reSpeaker XVF3800 USB 4-Mic Array

**Voice processor:** XMOS **XVF3800** (xcore.ai XU316-1024-QF60B core), 7 × 7 mm 60-pin QFN
**Audio codec:** Texas Instruments TLV320AIC3104
**Seeed SKU:** 101991441 (variant without XIAO ESP32S3)

### 5.1 Acoustic Front End

| Parameter | Value |
|---|---|
| Microphone count / type | **4 × PDM MEMS**, circular array |
| Pickup pattern | **360°** |
| Far-field pickup range | **up to 5 m** |
| AGC range | **60 dB** |
| Beamforming | Fast-tracking or fixed beamformer, **multiple simultaneous beams** |
| Acoustic Echo Cancellation | Full duplex AEC |
| Noise suppression | Dynamic / DNN-based echo and noise suppressor |
| De-reverberation | Yes |
| Voice Activity Detection | Yes |
| Direction of Arrival | Yes — see §5.4 |
| Limiter | Yes |
| ASR output | Dedicated ASR beam with configurable fixed gain |

### 5.2 Audio Data Format

| Parameter | Value |
|---|---|
| **Sample rate** | **16 kHz** |
| **Bit depth** | **32-bit** (Seeed firmware notes). Host stacks commonly negotiate `S32_LE`; ALSA `plughw` will transparently convert to `S16_LE` if requested. |
| USB class | **USB Audio Class 2.0** (high-speed USB 2.0 device) |
| Connector | USB Type-C (power + data) |

**Firmware-selectable channel maps:**

| Firmware | Channels | Channel assignment |
|---|---|---|
| `respeaker_xvf3800_usb_dfu_firmware_v2.0.x.bin` | **2** | Ch 0 = processed "Conference" audio; Ch 1 = processed "ASR" beam |
| `respeaker_xvf3800_usb_dfu_firmware_6chl_v2.0.x.bin` | **6** | Ch 0 = Conference; Ch 1 = ASR; **Ch 2–5 = raw Mic 0–3** |

> **Recommendation for BillieBot: use the 6-channel firmware.** It gives you the processed ASR beam for speech recognition *and* the four raw microphone streams, which you will need if you ever want to do your own beamforming, sound-source localization, or acoustic-event classification independent of the XMOS pipeline.

**Bandwidth [CALC]:**
- 2-channel: 16000 × 4 bytes × 2 = **128 kB/s**
- 6-channel: 16000 × 4 bytes × 6 = **384 kB/s**

**Channel re-routing:** `AUDIO_MGR_OP_L` / `AUDIO_MGR_OP_R` allow either output channel to be reassigned to a different internal source (e.g. amplified Mic 0, far-end reference).

### 5.3 Electrical

| Parameter | Value |
|---|---|
| **XVF3800 core (VDD) power, typical** | **400 mW in USB mode** (345 mW in I²S mode) |
| Core supply | 0.9 V (VDD); 3.3 V (V_DDIOL/R/T); 1.8 V (VDD_IOB18); USB PHY 1.8 V + 3.3 V |
| **Total board power** | **~1 – 2 W [EST]** — Seeed does not publish a board-level figure. Estimate = 400 mW core + codec + regulators + PDM mics + USB PHY. |
| **Total board current @ 5 V** | **~200 – 400 mA [EST]** |
| Additional load: 12 × WS2812 RGB LEDs | **up to ~0.7 A at full-white, full-brightness [EST]** (12 × ~60 mA). At the moderate brightness used for DoA indication, budget **~60–150 mA [EST]**. |
| Additional load: JST speaker amplifier | Supports **5 W** amplified speakers — if used, this dominates the board's power budget |
| Bus power | USB bus-powered; a USB 2.0 device may declare up to 500 mA |

> ⚠️ **Power budgeting action:** If BillieBot does not need the LED ring, **disable it** (`led_effect 0`, or drive GPO `X0D33` low to cut WS2812 power). This is the single largest discretionary load on this board. Similarly, hold GPO `X0D31` high to keep the speaker amplifier disabled if no speaker is fitted.

### 5.4 Control & Inference Data Interface

Control is available over **USB (EP0 vendor-specific class)** or **I²C**, via the `xvf_host` / `xvf_host.exe` utility or its underlying protocol. This is a **second, independent data path** from the audio stream.

| Command | Returns | Format |
|---|---|---|
| `AEC_AZIMUTH_VALUES` | **Direction of Arrival**, 4 values | **float32 radians** (utility also prints degrees): [fixed beam 1, fixed beam 2, free-running beam, **auto-selected beam**] |
| `AEC_SPENERGY_VALUES` | Speech energy, 4 values | Integer energy per beam, same beam ordering |
| `GPI_READ_VALUES` | 3 GPI pin states | 3 × binary (e.g. `1 0 0`) |
| `GPO_READ_VALUES` | 5 GPO pin states | 5 × binary |
| `VERSION` | Firmware version | 3 integers (e.g. `2 0 2`) |

> **The auto-selected beam azimuth is the DoA value BillieBot should consume** — it is the beam the pipeline chose for best audio, and it is what drives the LED ring indication.

**GPIO map:**

| Pin | Dir | Function |
|---|---|---|
| `X1D09` | Input | Mute button status (high = released) |
| `X1D13`, `X1D34` | Input | Floating / available |
| `X0D30` | Output | Mute LED + microphone mute control (high = muted) |
| `X0D31` | Output | Amplifier enable (**low = enabled**) |
| `X0D33` | Output | WS2812 LED power control (high = on) |
| `X0D11`, `X0D39` | Output | Floating / available |

**Fixed-beam configuration** (useful if BillieBot's microphone has a known preferred listening sector):
`AEC_FIXEDBEAMSAZIMUTH_VALUES <az1> <az2>` (radians), `AEC_FIXEDBEAMSELEVATION_VALUES <el1> <el2>`, then `AEC_FIXEDBEAMSONOFF 1`.

**Tuning parameters:** `AUDIO_MGR_REF_GAIN`, `AUDIO_MGR_MIC_GAIN`, `AUDIO_MGR_SYS_DELAY`, `PP_AGCMAXGAIN`, `AEC_ASROUTGAIN`.

### 5.5 Mechanical / Mounting Note

Seeed states the **microphone port (sound inlet) is on the back side of the board — the side with the printed Seeed Studio logo** — and must face the sound source for the acoustic algorithms to work correctly. **This is a real integration constraint for BillieBot's enclosure design** and is easy to get backwards.

---

## 6. DFRobot Gravity: 10 DOF IMU AHRS — BNO055 + BMP280 (SEN0253)

Two independent sensors on one Gravity-I²C carrier board. They are addressed separately and are covered in §6.2 (BNO055) and §6.3 (BMP280).

### 6.1 Board-Level

| Parameter | Value |
|---|---|
| Operating voltage | **3.3 – 5 V DC** |
| **Operating current (DFRobot, board)** | **5 mA** **[CONFLICT]** |
| **BNO055 alone (Bosch, NDOF @ 100 Hz)** | **up to 12.3 mA** **[CONFLICT]** |
| Interface | Gravity-I²C (4-pin) plus a broken-out header |
| Operating temperature | **−40 °C to +80 °C** |
| Dimensions | 32 × 27 mm |
| Board power @ 5 V | ~25 mW (DFRobot figure) to ~62 mW **[CALC]** (Bosch figure) |

> **[CONFLICT] — resolve on the bench.** DFRobot's 5 mA board figure is **lower than Bosch's 12.3 mA maximum for the BNO055 alone**, before accounting for the BMP280, the level shifter, or pull-ups. The 5 mA figure is likely quoted for a low-power or non-fusion mode. **For BillieBot's power budget, use ~15–20 mA @ 5 V (~75–100 mW) as the conservative planning number [EST].** In absolute terms this is negligible next to the OAK-D Lite, so the discrepancy is low-risk — but do not use 5 mA in a battery-life calculation without verifying it.

**Pinout:**

| # | Name | Function |
|---|---|---|
| 1 | VCC | Positive supply |
| 2 | GND | Ground |
| 3 | C | I²C SCL |
| 4 | D | I²C SDA |
| 5 | NBOOT | Boot mode select |
| 6 | RST | Reset |
| 7 | INT | Interrupt output |
| 8 | I2C_ADDR | BNO055 address select (0x28 / 0x29) |
| 9 | PS2 | Protocol select 2 |
| 10 | PS1 | Protocol select 1 |
| 11 | BL_IND | Bootstrap indication |

**Protocol selection (PS1 / PS2), default = 0 / 0:**

| PS1 | PS2 | Mode |
|---|---|---|
| 0 | 0 | **Standard / Fast I²C (default)** |
| 0 | 1 | HID over I²C |
| 1 | 0 | UART |
| 1 | 1 | Reserved |

**I²C addresses:** BNO055 = **0x28** (default; 0x29 selectable) · BMP280 = **0x76**

### 6.2 BNO055 — 9-Axis Absolute Orientation Sensor

Integrates a 14-bit triaxial accelerometer, a 16-bit triaxial gyroscope, a triaxial geomagnetic sensor, **and a 32-bit Cortex-M0+ running Bosch sensor-fusion firmware**. The fusion is done on-chip; the host receives orientation directly.

#### 6.2.1 Accelerometer

| Parameter | Value |
|---|---|
| Ranges | **±2 / ±4 / ±8 / ±16 g** (auto-set to ±4 g in fusion modes) |
| Resolution | **14-bit** |
| Sensitivity | **1 LSB/mg** (all ranges) |
| Sensitivity tolerance | ±1 % typ, ±4 % max @ 25 °C, ±2 g |
| Zero-g offset | ±80 mg typ, ±150 mg max over lifetime |
| Zero-g offset temp drift | ±1 mg/K typ, ±3.5 mg/K max |
| **Output noise density** | **150 µg/√Hz typ**, 190 µg/√Hz max |
| Nonlinearity | 0.5 % FS typ |
| Bandwidth options | 7.81 / 15.63 / 31.25 / **62.5 (fusion default)** / 125 / 250 / 500 / 1000 Hz |
| Cross-axis sensitivity | 1 % typ, 2 % max |
| Package alignment error | 0.5° typ, 2° max |

#### 6.2.2 Gyroscope

| Parameter | Value |
|---|---|
| Ranges | **±125 / ±250 / ±500 / ±1000 / ±2000 °/s** (auto-set to ±2000 °/s in fusion) |
| Resolution | **16-bit** |
| Sensitivity | **16.0 LSB/(°/s)**, or 900 LSB/(rad/s) |
| Zero-rate offset | ±1 °/s typ, ±3 °/s max |
| Zero-rate offset temp drift | ±0.015 °/s/K typ, ±0.03 °/s/K max |
| **Output noise** | **0.1 °/s rms typ** (BW = 47 Hz), 0.3 °/s max → 0.014 °/s/√Hz |
| Nonlinearity | ±0.05 % FS typ |
| Bandwidth options | 12 / 23 / **32 (fusion default)** / 47 / 64 / 116 / 230 / 523 Hz |
| Cross-axis sensitivity | ±1 % typ, ±3 % max |

#### 6.2.3 Magnetometer

| Parameter | Value |
|---|---|
| Range (X, Y) | **±1300 µT** typ (±1200 µT min) |
| Range (Z) | **±2500 µT** typ (±2000 µT min) |
| **Device resolution** | **0.3 µT** |
| **Heading accuracy** | **±2.5°** typ, in a 30 µT horizontal geomagnetic field @ 25 °C, **fully calibrated with ideal tilt compensation** |
| Zero-B offset (uncalibrated) | ±40 µT |
| Zero-B offset (after fusion-mode calibration) | **±2 µT** |
| Zero-B offset temp drift | ±0.23 µT/K typ, ±0.37 µT/K max |
| Output noise (regular preset) | 0.6 µT |
| Output noise (high-accuracy preset) | 0.3 µT |
| Gain error after API compensation | ±5 % typ, ±8 % max |
| Resolution (x / y / z) | 13 / 13 / 15 bits |

#### 6.2.4 Operating Modes & Output Rates

| Mode | Accel | Mag | Gyro | Fusion Output | Fused Data Rate |
|---|---|---|---|---|---|
| `ACCONLY` / `MAGONLY` / `GYROONLY` | — | — | — | none | sensor-configured |
| `ACCMAG` / `ACCGYRO` / `MAGGYRO` / `AMG` | ✓ | ✓ | ✓ | none | sensor-configured |
| `IMU` | 100 Hz | — | 100 Hz | **relative** orientation | **100 Hz** |
| `COMPASS` | 20 Hz | 20 Hz | — | absolute | **20 Hz** |
| `M4G` | 50 Hz | 50 Hz | — | relative | **50 Hz** |
| `NDOF_FMC_OFF` | 100 Hz | 20 Hz | 100 Hz | absolute | **100 Hz** |
| **`NDOF`** | **100 Hz** | **20 Hz** | **100 Hz** | **absolute** | **100 Hz** |

- `NDOF` is the full 9-DOF absolute-orientation mode with Fast Magnetometer Calibration **on**. Highest accuracy and fastest magnetometer convergence; slightly higher current than `NDOF_FMC_OFF`.
- Data rate tolerance: **±1 %** using the internal oscillator (an external 32 kHz crystal improves this).

#### 6.2.5 Electrical & Timing

| Parameter | Value |
|---|---|
| VDD (sensors) | 2.4 – 3.6 V |
| VDDIO (µC and I/O) | 1.7 – 3.6 V |
| **Total supply current, normal mode, 9DOF @ 100 Hz** | **≤ 12.3 mA** (VDD = 3 V, VDDIO = 2.5 V) |
| Total supply current, low-power mode | 0.33 – 2.72 mA |
| Total supply current, suspend mode | ≤ 0.04 mA |
| Operating temperature | −40 °C to +85 °C |
| Start-up time (off → CONFIG mode) | **400 ms** |
| POR time (reset → CONFIG mode) | **650 ms** |
| Mode switch, CONFIG → any operating mode | **7 ms** |
| Mode switch, any operating mode → CONFIG | **19 ms** |
| Mechanical shock survival | 10,000 g (≤ 200 µs) / 2,000 g (≤ 1 ms) |

#### 6.2.6 Data Received from the BNO055

All values are read from the register map as **signed integers**, and must be divided by the scaling factor below to obtain engineering units. Multi-byte reads (burst reads) are **register-shadowed**, guaranteeing LSB/MSB consistency — **single-byte reads are not, and can tear.** Always use burst reads.

| Output | Registers | Size | Rate (NDOF) | **Scaling** | Units |
|---|---|---|---|---|---|
| **Quaternion** (w, x, y, z) | 0x20 – 0x27 | 4 × int16 | 100 Hz | **1 unit = 2¹⁴ = 16384 LSB** | dimensionless |
| **Euler angles** (heading, roll, pitch) | 0x1A – 0x1F | 3 × int16 | 100 Hz | **1° = 16 LSB** or 1 rad = 900 LSB | ° or rad |
| **Accelerometer** | 0x08 – 0x0D | 3 × int16 | 100 Hz | **1 m/s² = 100 LSB**, or 1 mg = 1 LSB | m/s² or mg |
| **Linear acceleration** (gravity removed) | 0x28 – 0x2D | 3 × int16 | 100 Hz | **1 m/s² = 100 LSB** | m/s² or mg |
| **Gravity vector** | 0x2E – 0x33 | 3 × int16 | 100 Hz | **1 m/s² = 100 LSB** | m/s² or mg |
| **Gyroscope** | 0x14 – 0x19 | 3 × int16 | 100 Hz | **1 °/s = 16 LSB**, or 1 rad/s = 900 LSB | °/s or rad/s |
| **Magnetometer** | 0x0E – 0x13 | 3 × int16 | **20 Hz** | **1 µT = 16 LSB** | µT |
| **Temperature** | 0x34 | 1 × **int8** | 1 Hz | **1 °C = 1 LSB** (or 2 °F = 1 LSB) | °C or °F |
| **Calibration status** | 0x35 (`CALIB_STAT`) | 1 × uint8 | on demand | 2 bits each: system, gyro, accel, mag | 0 (uncal) – 3 (fully cal) |

**Euler angle output ranges (Android format):**

| Angle | Range |
|---|---|
| Heading / Yaw | **0° to 360°** (clockwise increases) |
| Roll | **−90° to +90°** |
| Pitch | **+180° to −180°** (clockwise **decreases**) |

Windows format is also selectable via `UNIT_SEL` bit 7, which changes the pitch sign convention. **Pick one and document it in BillieBot's ICD** — this is a classic source of sign-error bugs.

**Unit selection register (`UNIT_SEL`, 0x3B):** acceleration m/s² vs mg · angular rate dps vs rps · Euler degrees vs radians · temperature °C vs °F · Windows vs Android orientation format.

**Axis remapping:** `AXIS_MAP_CONFIG` (0x41) and `AXIS_MAP_SIGN` (0x42) let the sensor's output frame be rotated in firmware to match BillieBot's body frame regardless of how the board is physically mounted. Eight standard placements (P0–P7) are tabulated in the datasheet. **Use this rather than rotating in host software** — it keeps the fusion output already in body frame.

**Bandwidth [CALC]:** reading quaternion + linear accel + gyro + calib status ≈ 21 bytes × 100 Hz ≈ **2.1 kB/s**.

#### 6.2.7 ⚠️ Critical Application Warnings from the Bosch Datasheet

These are directly relevant to a wheeled/mobile robot and should drive BillieBot's sensor-fusion architecture:

1. **"The sensor fusion algorithm was primarily designed to track human motion. If the device is subjected to large accelerations for an extended period of time (e.g. in a vehicle cornering at high speed or braking over a long distance), the device may incorrectly interpret this large acceleration as the gravity vector."** → BillieBot's sustained accelerations during turns and braking may corrupt the roll/pitch estimate. **Test this specifically.**
2. **"The linear acceleration signal typically cannot be integrated to recover velocity, or double-integrated to recover position. The error typically becomes larger than the signal within less than 1 second"** unless other sensors compensate. → Do **not** dead-reckon from this IMU alone. Fuse with wheel odometry and/or the OAK-D Lite's visual data.
3. **Automatic background calibration cannot be disabled.** It runs continuously in all fusion modes.
4. Magnetometer distortion (from BillieBot's own motors, wiring, and battery currents) will cause the algorithm to ignore magnetometer data, at which point **heading drifts like IMU mode**. Mount the board as far from motors and high-current conductors as practical.
5. **Calibration procedure required after every power-on reset** unless a stored calibration profile is restored: accelerometer needs 6 stable orientations; gyroscope needs a few seconds stationary; magnetometer needs a figure-8 motion. Read `CALIB_STAT` to confirm. Calibration offsets/radii can be saved and re-written to skip this on subsequent boots.

### 6.3 BMP280 — Barometric Pressure & Temperature Sensor

| Parameter | Value |
|---|---|
| **Pressure range** | **300 – 1100 hPa** |
| **Relative accuracy** | **±0.12 hPa** (≈ **±1 m** altitude) |
| **Absolute accuracy** | **±1 hPa** (≈ **±8.33 m** altitude) |
| **Temperature range** | **0 °C to 65 °C** |
| **Temperature resolution** | **0.01 °C** |
| I²C address | **0x76** |
| Current draw | Not stated separately by DFRobot; **negligible vs. the BNO055 [EST]** (typically single-digit µA at 1 Hz for this Bosch part) |

**Data received from the BMP280:**
- **Raw layer:** 20-bit uncompensated pressure and 20-bit uncompensated temperature ADC values, plus a set of factory calibration coefficients in NVM. These raw values are **meaningless without compensation**.
- **Compensated layer (what the DFRobot library returns):**

| Value | Units | Notes |
|---|---|---|
| Temperature | **°C** (float) | |
| Pressure | **Pa** (float) | Note: **pascals**, not hPa — the DFRobot example prints Pa |
| Altitude | **m** (float) | **Calculated**, not measured |

> **BMP280 altitude caveat, stated explicitly by DFRobot:** *"The altitude is calculated from the temperature and pressure values collected by the on-board sensor BMP280, not the actual measured value, and there is an error."* Altitude also requires a **sea-level pressure reference** (the DFRobot example hard-codes `SEA_LEVEL_PRESSURE 1015.0f`). Absolute altitude is only as good as that reference. **Relative** altitude change (±1 m) is far more trustworthy than absolute altitude (±8.33 m).

> **BillieBot implication:** For a ground robot, the useful signal here is **relative** — detecting that BillieBot has changed floors, gone up a ramp, or been picked up. Do not use absolute barometric altitude for navigation. Also note the BMP280's **0–65 °C** temperature range is narrower than the BNO055's −40 to +85 °C.

---

## 7. System-Level Integration Notes

### 7.1 Consolidated Power Budget

| Sensor | Rail | Typ. Current | Typ. Power | Peak Power | Provenance |
|---|---|---|---|---|---|
| OAK-D Lite (AF) | 5 V USB | ~0.8 – 1.0 A **[CALC]** | **4 – 5 W** | **6 W** | Stated |
| RPi Camera Module 3 NoIR | 3.3 V CSI | ~250 mA **[EST]** | ~0.8 W **[EST]** | ~1.0 W **[EST]** | Estimated |
| reSpeaker XVF3800 (LEDs off) | 5 V USB | ~200 – 400 mA **[EST]** | ~1 – 2 W **[EST]** | ~2 W **[EST]** | Core power stated; board power estimated |
| reSpeaker XVF3800 (LED ring on) | 5 V USB | + up to ~700 mA **[EST]** | + up to ~3.5 W **[EST]** | — | Estimated |
| MLX90640 breakout | 3.3 / 5 V | 18 mA typ, 25 mA max | ~60 mW **[CALC]** | ~83 mW **[CALC]** | Stated (sensor) |
| SEN0253 (BNO055 + BMP280) | 3.3 / 5 V | ~15 mA **[EST]** | ~75 mW **[EST]** | ~100 mW **[EST]** | See §6.1 conflict |
| **TOTAL (LEDs off)** | — | — | **≈ 6 – 8 W [CALC]** | **≈ 9 – 10 W [CALC]** | — |
| **TOTAL (LEDs on)** | — | — | **≈ 9 – 11 W [CALC]** | **≈ 13 W [CALC]** | — |

> **Recommendation:** Size BillieBot's sensor power rail for **≥ 15 W** to preserve margin. The OAK-D Lite dominates — it is ~60–70 % of the sensor suite's draw on its own.

### 7.2 ⚠️ USB Current Constraint — OAK-D Lite

Luxonis states directly: *"Consumption of OAK-D Lite can be higher and will fall out of the USB2 maximum specified range of 900 mA."* On hosts with limited per-port current (Raspberry Pi 4 and similar), Luxonis supplies a **Y-adapter** that brings in a separate power feed alongside the host data connection.

**Action for BillieBot:** Either (a) power the OAK-D Lite through a Y-adapter from BillieBot's main rail, or (b) use a powered USB hub, or (c) confirm the host SBC can source ≥ 1.2 A on that single port. **Do not assume the SBC's USB port can carry it.** Undervoltage on this device manifests as intermittent enumeration failures and pipeline crashes, which are painful to diagnose.

### 7.3 ⚠️ I²C Bus Speed Conflict — MLX90640 vs. BNO055

All three I²C devices have distinct addresses (0x33, 0x28, 0x76) and *can* electrically share one bus. **But they should not.**

| Device | Max I²C clock |
|---|---|
| MLX90640 | **1 MHz (FM+)** — and it *needs* the speed |
| BNO055 | Standard / Fast I²C (**400 kHz**) |
| BMP280 | 400 kHz typical |

A shared bus must run at the slowest device's rate: **400 kHz**.

**Bandwidth analysis at 400 kHz [CALC]:**
- One MLX90640 subpage read ≈ 768 × 2 bytes + auxiliary registers ≈ 1664 bytes ≈ 13.3 kbit
- At 400 kHz with I²C overhead: ≈ **40–45 ms per subpage**
- Two subpages per full thermal image: ≈ **80–90 ms → ~11 full images/second maximum**, *with the bus doing nothing else*
- Add the BNO055 at 100 Hz (~2.1 kB/s) and the bus contention gets tight.

**Recommendation:** Put the **MLX90640 on its own I²C bus at 1 MHz**, and the BNO055 + BMP280 together on a second bus at 400 kHz. Most SBCs expose multiple I²C controllers. If only one bus is available, cap the MLX90640 at **8 Hz refresh (4 full images/second)** to leave headroom. Also note the MLX90640 datasheet's warning that **capacitive loading degrades I²C** — keep the bus short and consider stronger pull-ups.

### 7.4 Environmental Envelope — Limiting Component

| Sensor | Operating Temperature Range |
|---|---|
| MLX90640 | −40 °C to +85 °C |
| BNO055 | −40 °C to +85 °C |
| SEN0253 board | −40 °C to +80 °C |
| OAK-D Lite | **−20 °C to +50 °C** |
| **RPi Camera Module 3** | **0 °C to +50 °C** ← **limiting component** |
| BMP280 (temperature measurement range) | 0 °C to +65 °C |

**BillieBot's usable ambient envelope is 0 °C to +50 °C**, set by the Camera Module 3. Raspberry Pi additionally warns the camera should be operated in a **well-ventilated environment** and that a case should not be covered — relevant if the camera is enclosed in BillieBot's shell. The OAK-D Lite's −20/+50 °C figure applies at *full VPU utilization*; running lighter pipelines raises the tolerable ambient.

### 7.5 Aggregate Data Bandwidth **[CALC]**

| Stream | Rate |
|---|---|
| OAK-D Lite RGB, 1080p30 uncompressed NV12 | ~93 MB/s |
| OAK-D Lite RGB, 1080p30 H.265 | ~1–2 MB/s **[EST]** |
| OAK-D Lite mono ×2, 640×480 @ 30 fps | ~18.4 MB/s |
| OAK-D Lite depth, 640×480 uint16 @ 30 fps | ~18.4 MB/s |
| OAK-D Lite IMU @ 200 Hz | ~8 kB/s |
| OAK-D Lite NN detections | < 10 kB/s **[EST]** |
| RPi Camera 3, 1080p50 YUV420 | ~155 MB/s |
| reSpeaker, 6-channel | 384 kB/s |
| MLX90640 @ 8 full images/s | ~24.6 kB/s |
| SEN0253 @ 100 Hz | ~2.1 kB/s |

> **Observation:** The two cameras dominate by three orders of magnitude. **Use on-device H.265/MJPEG encoding on the OAK-D Lite and keep the Pi camera in a binned mode** unless full-resolution uncompressed frames are genuinely required. All the low-rate sensors combined (IMU, thermal, audio, AHRS) total well under 0.5 MB/s and are effectively free.

### 7.6 Complementary Coverage — Field of View Comparison

| Sensor | HFOV | VFOV | DFOV |
|---|---|---|---|
| OAK-D Lite RGB | 69° | 54° | 81° |
| OAK-D Lite mono / depth | 73° | 58° | 86° |
| RPi Camera 3 NoIR | 66° | 41° | 75° |
| MLX90640 (55°) | 55° | 35° | ~65° **[CALC]** |
| reSpeaker XVF3800 | **360°** | — | — |

The four optical sensors have broadly comparable and overlapping fields of view, with the **MLX90640 the narrowest** — if the thermal channel is to be co-registered with the RGB or depth image, the thermal FOV defines the usable common region. The microphone array is the only sensor with full 360° awareness, making its DoA output BillieBot's primary cue for **directing attention outside the optical field of view**.

---

## 8. Open Items / Bench Verification Required

| # | Item | Section | Why it matters |
|---|---|---|---|
| 1 | Confirm the OAK-D Lite unit **has an IMU** (Kickstarter units do not) | §2.4 | An entire data stream may not exist |
| 2 | Characterize actual usable stereo depth range | §2.3 | Official sources conflict (80 cm–12 m vs 40 cm–8 m) |
| 3 | Measure RPi Camera Module 3 current draw | §4.2 | Only unpublished significant power number |
| 4 | Measure reSpeaker XVF3800 board current, LEDs on and off | §5.3 | Board-level power not published |
| 5 | Measure SEN0253 current in NDOF mode | §6.1 | DFRobot (5 mA) vs Bosch (12.3 mA) conflict |
| 6 | Test BNO055 fusion under sustained cornering/braking | §6.2.7 | Datasheet warns of gravity-vector misinterpretation |
| 7 | Characterize magnetometer distortion from BillieBot's motors | §6.2.7 | Determines whether absolute heading is usable at all |
| 8 | Decide I²C bus topology (split vs shared) and verify MLX90640 throughput | §7.3 | Sets the achievable thermal frame rate |
| 9 | Verify USB power delivery path for the OAK-D Lite | §7.2 | Undervoltage causes hard-to-diagnose failures |
| 10 | Confirm reSpeaker microphone-inlet orientation in the enclosure | §5.5 | Acoustic algorithms depend on it |

---

## 9. Sources

**Luxonis OAK-D Lite**
- Luxonis hardware documentation — OAK-D Lite: https://docs.luxonis.com/hardware/products/OAK-D%20Lite
- Luxonis OAK-D Lite datasheet (Dec 2021), electrical characteristics and camera sensor characteristics
- Luxonis DepthAI IMU node documentation: https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/imu.md
- Luxonis BMI270 hardware reference: https://docs.luxonis.com/hardware/platform/sensors/imu/bmi270.md
- Luxonis store page — OAK-D Lite: https://shop.luxonis.com/products/oak-d-lite-1

**Adafruit MLX90640 (PID 4407)**
- Adafruit product page: https://www.adafruit.com/product/4407
- Melexis MLX90640 datasheet, Revision 11, 4 May 2018: https://www.melexis.com/-/media/files/documents/datasheets/mlx90640-datasheet-melexis.pdf

**Raspberry Pi Camera Module 3 NoIR (SC0873)**
- Raspberry Pi Camera Module 3 product brief, June 2024: https://datasheets.raspberrypi.com/camera/camera-module-3-product-brief.pdf
- Raspberry Pi product page: https://www.raspberrypi.com/products/camera-module-3/
- Raspberry Pi camera documentation: https://www.raspberrypi.com/documentation/accessories/camera.html
- Current-draw estimate: Raspberry Pi community forum measurements (see §4.2)

**Seeed reSpeaker XVF3800**
- Seeed Studio Wiki — Getting Started with reSpeaker XVF3800: https://wiki.seeedstudio.com/respeaker_xvf3800_introduction/
- XMOS XVF3800 datasheet v3.2.1, Key Features (power consumption): https://www.xmos.com/documentation/XM-014888-PC/html/modules/fwk_xvf/doc/datasheet/01_features.html
- XMOS XVF3800 datasheet v3.2.1, Device Operation (power supplies): https://www.xmos.com/documentation/XM-014888-PC/html/modules/fwk_xvf/doc/datasheet/06_device_operation.html
- reSpeaker XVF3800 firmware and host control repository: https://github.com/respeaker/reSpeaker_XVF3800_USB_4MIC_ARRAY

**DFRobot SEN0253**
- DFRobot Wiki — Gravity: 10 DOF IMU AHRS BNO055 + BMP280: https://wiki.dfrobot.com/sen0253/
- Bosch Sensortec BNO055 datasheet, BST-BNO055-DS000-18, Revision 1.8, October 2021: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
- DFRobot BMP280 example code and library documentation: https://wiki.dfrobot.com/sen0253/docs/23846

---

*End of document.*
