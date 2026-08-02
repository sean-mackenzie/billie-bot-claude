# BillieBot Sensor Specification Reference

**Document purpose:** Single-point engineering reference for the principal perception and state-estimation sensors used in BillieBot.  
**Prepared:** 2026-08-02  
**Scope:** OAK-D Lite (autofocus), Adafruit MLX90640 55° thermal breakout, Raspberry Pi Camera Module 3 NoIR (standard 75° lens), reSpeaker XVF3800 USB Mic Array, and DFRobot Gravity SEN0253 BNO055 + BMP280.

---

## 1. How to Interpret This Reference

Specification values use the following labels:

- **Manufacturer-stated:** Published by the device or component manufacturer.
- **Calculated:** Derived directly from published voltage/current or scaling data.
- **Engineering estimate:** A conservative integration or power-budget value where the manufacturer has not published a board-level value.
- **Application-dependent:** Strongly affected by scene, lighting, acoustic environment, calibration, mounting, configuration, host software, or selected operating mode.

### Important data-format distinction

Several of these devices do **not** inherently create image, audio, or log files. They transmit frames, samples, or register values to the host. File types such as `.jpg`, `.png`, `.dng`, `.wav`, `.mp4`, ROS 2 bag files, or NumPy arrays are created by host software.

---

## 2. At-a-Glance Integration Matrix

| Sensor | Primary BillieBot function | Physical/digital interface | Principal output | Published normal power | Recommended provisional design allocation |
|---|---|---|---|---:|---:|
| OAK-D Lite AF | RGB vision, stereo depth, edge AI, local motion sensing | USB-C; USB 2/3 | RGB/mono image frames, depth/disparity maps, NN results, 6-axis IMU reports | 2.5–3 W base + up to 2 W for AI/stereo/encoding | **7.5 W at 5 V (1.5 A)** maximum USB allocation |
| Adafruit MLX90640 55° | Low-resolution thermal imaging | I²C via STEMMA QT/Qwiic or header | 768 calibrated object-temperature values | MLX90640 IC: 20 mA typical at 3.3 V | **0.15 W** breakout allowance |
| Pi Camera Module 3 NoIR, standard | Visible + near-IR imaging/night vision | 2-lane MIPI CSI-2 ribbon | RAW10 Bayer data; ISP-processed still/video streams | Not published by Raspberry Pi | **1.0 W incremental Pi input budget** |
| reSpeaker XVF3800 | Far-field speech/bark capture, direction of arrival, VAD | USB-C UAC 2.0 or I²S firmware | 2- or 6-channel PCM audio plus control metadata | Not published by Seeed for this board | **2.5 W at 5 V (0.5 A)**, excluding meaningful speaker load |
| DFRobot SEN0253 | Orientation, angular rate, acceleration, magnetic heading, pressure, temperature, derived altitude | Gravity I²C | 16-bit register data and fused orientation products | DFRobot states 5 mA | **0.10 W** allowance; verify because BNO055 component data permit higher current |

> **Power-budget caution:** The allocations above are intentionally conservative and are not assertions of typical measured consumption. Measure current on the final BillieBot harness under representative workloads before freezing the power-tree design.

---

# 3. OAK-D Lite — Autofocus Version

## 3.1 Intended BillieBot Role

The OAK-D Lite is the primary forward-facing smart vision sensor. It can provide:

1. High-resolution color imagery.
2. Synchronized global-shutter stereo imagery.
3. On-device stereo depth/disparity.
4. On-device neural-network inference and spatial detections.
5. Six-axis acceleration and angular-rate data from the BMI270 IMU.
6. Hardware-encoded video streams.

The current retail OAK-D Lite lists an integrated BMI270. Luxonis notes that early Kickstarter-backed OAK-D Lite units omitted the IMU. Confirm the actual unit using the DepthAI device/camera/IMU enumeration API.

## 3.2 Core Hardware

| Parameter | Specification | Status / notes |
|---|---:|---|
| Processing architecture | RVC2 / Intel Movidius Myriad X | Manufacturer-stated |
| Aggregate compute | 4 TOPS; approximately 1.4 TOPS usable for AI in legacy datasheet | Manufacturer-stated |
| Host connection | USB-C, USB 2 or USB 3; up to 10 Gbit/s depending on host/cable/mode | Manufacturer-stated |
| Power input | USB VBUS, nominal 5 V | Manufacturer-stated |
| Stereo baseline | **75 mm** | Manufacturer-stated. Some rendered Luxonis pages display “75 cm”; the physical product and mechanical data are 75 mm. |
| Approximate dimensions | 91 × 28 × 17.5 mm | Manufacturer-stated |
| Approximate mass | 61 g | Manufacturer-stated in legacy product documentation |
| Operating ambient | Approximately -20 to 50 °C at full VPU utilization | Current Luxonis guidance; thermal limit is workload-dependent |

## 3.3 Center RGB Camera

| Parameter | Specification | Status / notes |
|---|---:|---|
| Sensor | Sony IMX214 | Manufacturer-stated |
| Shutter | Rolling shutter | Manufacturer-stated |
| Native active resolution | **4208 × 3120**, approximately 13 MP | Manufacturer-stated |
| Full-resolution rate | **30 fps** at 4208 × 3120 | Current Luxonis sensor-driver specification |
| Other common modes | 4056 × 3040 at 30 fps; 3840 × 2160 at 30 fps; 1920 × 1080 up to approximately 35 fps | Manufacturer-stated driver limits |
| Lens focus | Autofocus | Requested BillieBot variant |
| Published focus range | Approximately **0.08 m to infinity** | Legacy Luxonis AF optics specification |
| Field of view | 81° diagonal / 69° horizontal / 54° vertical | Manufacturer-stated |
| Pixel pitch | 1.12 µm × 1.12 µm | Manufacturer-stated |
| Sensor optical size | Approximately 1/3.06 inch | Manufacturer-stated |
| Effective focal length | Approximately 3.37 mm | Manufacturer-stated |
| F-number | f/2.2 ±5% | Manufacturer-stated |
| Lens distortion | <1% in legacy datasheet | Manufacturer-stated |
| Infrared filtering | Color module intended for visible-light imaging | Do not treat it as the primary night-vision sensor |

### RGB data received by the host

DepthAI carries camera images in an `ImgFrame` object. The configured pipeline determines the actual format.

Potential unencoded `ImgFrame` types include:

- Bayer/raw types such as `RAW8`, `RAW10`, `RAW12`, `RAW14`, or `RAW16`.
- `NV12`, `NV21`, and planar YUV formats.
- Planar or interleaved RGB/BGR, including `RGB888p`, `BGR888p`, `RGB888i`, and `BGR888i`.
- Grayscale formats where applicable.

Potential hardware-encoded outputs include:

- H.264/AVC bitstream.
- H.265/HEVC bitstream.
- MJPEG/JPEG bitstream.

The host application determines whether these streams become files such as `.h264`, `.h265`, `.mp4`, `.mjpeg`, `.jpg`, or ROS image messages.

## 3.4 Stereo Cameras

| Parameter | Specification | Status / notes |
|---|---:|---|
| Sensors | 2 × OmniVision OV7251 | Manufacturer-stated |
| Image type | Monochrome | Manufacturer-stated |
| Shutter | Global shutter | Important for robot motion and stereo correspondence |
| Native resolution | **640 × 480** per camera | Manufacturer-stated |
| Current documented module rate | Approximately 99 fps at 640 × 480; driver limits vary by mode | Current Luxonis CCM documentation |
| Legacy sensor maximum | Up to 200 fps under sensor-specific conditions | Legacy datasheet; do not assume this is available in every DepthAI pipeline |
| Focus | Fixed | Manufacturer-stated |
| Published focus range | Approximately **0.065 m to infinity** | Manufacturer-stated |
| Field of view | 86° diagonal / 73° horizontal / 58° vertical | Manufacturer-stated |
| Pixel pitch | 3.0 µm × 3.0 µm | Manufacturer-stated |
| Effective focal length | Approximately 1.3 mm | Manufacturer-stated |
| F-number | f/2.2 | Manufacturer-stated |
| Lens distortion | <1.5% in legacy datasheet | Manufacturer-stated |
| Active IR projector/illuminator | None | OAK-D Lite is passive stereo |

## 3.5 Depth Range and Accuracy

| Parameter | Specification | Status / notes |
|---|---:|---|
| Recommended/ideal operating range | Approximately **0.8 to 12 m** | Manufacturer-stated; scene dependent |
| Closest depth, extended disparity | Approximately **0.20 m** at reduced stereo height/mode | Manufacturer-stated; configuration dependent |
| Typical practical closest depth | Approximately 0.35–0.40 m in commonly used extended-disparity configurations | Configuration dependent |
| Long-range theoretical limit | Approximately 19 m in legacy calculation | Theoretical; not a robust navigation range |
| Median absolute depth error, <3 m | <2% | Luxonis median-device data for 480p/75 mm OAK-D Lite |
| Median absolute depth error, 3–6 m | <4% | Manufacturer characterization |
| Median absolute depth error, 6–8 m | <6% | Manufacturer characterization |

Depth performance degrades on blank walls, shiny surfaces, transparent objects, repetitive patterns, dark scenes, motion blur, occlusions, and poorly calibrated or mechanically disturbed stereo pairs. Unlike the OAK-D Pro, the Lite has no dot projector to add texture in low-feature scenes.

### Depth-map representation

The standard `StereoDepth.depth` output is:

- One sample per output pixel, normally matching or aligned to a selected camera frame.
- `RAW16`, represented on the host as unsigned 16-bit integers.
- Default unit: **millimetres**.
- Valid numerical range: 0–65,535 depth units.
- A value of **0 means invalid or undetermined depth**, not zero-distance contact.
- Depth can be aligned to the RGB camera; alignment can change output geometry/resolution and create invalid border regions.

Example:

```text
depth[y, x] = 1250  -> estimated range = 1,250 mm = 1.25 m
depth[y, x] = 0     -> no valid stereo depth at that pixel
```

### Disparity representation

| Mode | Output representation | Nominal values |
|---|---|---:|
| Standard integer disparity | `RAW8` | 0–95 pixels |
| Extended disparity | `RAW8` | 0–190 pixels |
| Subpixel disparity, 3 fractional bits | `RAW16` | 0–760 |
| Subpixel disparity, 4 fractional bits | `RAW16` | 0–1,520 |
| Subpixel disparity, 5 fractional bits | `RAW16` | 0–3,040 |

A larger disparity corresponds to a closer point. Depth is computed from calibrated focal length, baseline, and disparity. Subpixel mode improves long-range resolution; extended disparity improves short-range capability.

## 3.6 BMI270 Six-Axis IMU

| Parameter | Accelerometer | Gyroscope |
|---|---:|---:|
| Axes | X, Y, Z | X, Y, Z |
| Full-scale options | ±2, ±4, ±8, ±16 g | ±125, ±250, ±500, ±1,000, ±2,000 °/s |
| Native sensitivity | 16,384 / 8,192 / 4,096 / 2,048 LSB/g | 262.144 / 131.072 / 65.536 / 32.768 / 16.384 LSB/(°/s) |
| Typical native noise density | 0.16 mg/√Hz | 0.007 °/s/√Hz in performance mode |
| Silicon ODR capability | 12.5–1,600 Hz | Up to 6,400 Hz |
| Stable DepthAI request points | 25, 50, 100, 200, 250 Hz | 25, 50, 100, 200, 250 Hz |
| Practical public report-rate behavior | Requests above 400 Hz currently top out near 250 Hz | Same |
| On-sensor orientation fusion | No | No |
| Magnetometer / absolute heading | None | None |

### IMU data received by the host

DepthAI returns `IMUData` messages containing one or more timestamped `IMUPacket` reports. For the raw acceleration/gyro example, the host receives:

- Acceleration components in **m/s²**.
- Angular-velocity components in **rad/s**.
- Sequence/timestamp information.
- Potentially batched packets, depending on node configuration.

The BMI270 does **not** provide a fused quaternion or magnetic heading through the same sensor. Fusion must occur in host software, potentially with the separate BNO055/BMP280 module or wheel odometry.

## 3.7 Neural-Network and Metadata Outputs

Depending on the DepthAI pipeline, the OAK can return:

- Tensor outputs from arbitrary neural networks.
- 2D detections: class label/index, confidence, and normalized/pixel bounding boxes.
- Spatial detections: bounding box plus X/Y/Z location derived from depth.
- Tracked-object IDs and tracking status.
- Feature tracks and optical-flow-related products.
- Camera exposure, ISO/sensitivity, lens position, white-balance temperature, FPS, timestamps, and sequence numbers.

These are structured API messages, not fixed file formats.

## 3.8 Power

| Workload element | Published consumption |
|---|---:|
| Base device + camera streaming | 2.5–3.0 W |
| AI subsystem | Up to +1.0 W |
| Stereo-depth subsystem | Up to +0.5 W |
| Video encoder subsystem | Up to +0.5 W |
| Legacy standby measurement | Approximately 0.6 W |
| Legacy typical demo workload | Approximately 4 W |
| Legacy listed maximum | Approximately 4.5 W, with possible short transients |

**BillieBot design recommendation:** Reserve a USB port and 5 V rail capable of the full **1.5 A / 7.5 W** legacy input requirement. A pipeline using all cameras, stereo, AI, and encoding will generally be closer to 4–5 W than to the 2.5–3 W base value.

## 3.9 BillieBot Integration Notes

- Use a USB 3-capable port and cable for simultaneous RGB, depth, and neural outputs.
- Rigidly mount the camera; stereo calibration depends on the relative pose of the two mono cameras.
- Do not place a clear protective window immediately in front of the stereo cameras unless it is optically suitable and mechanically aligned.
- Do not use the RGB rolling-shutter image as the only fast-motion measurement source.
- For navigation, treat 0-valued depth pixels as invalid and apply confidence, spatial, and temporal filtering.
- Timestamp and transform the OAK IMU into the robot base frame before sensor fusion.

### Primary sources

- Luxonis OAK-D Lite hardware page: https://docs.luxonis.com/hardware/products/OAK-D%20Lite
- Luxonis store page: https://shop.luxonis.com/products/oak-d-lite-1
- Luxonis OAK-D Lite AF legacy datasheet mirror: https://www.mouser.com/catalog/specsheets/Luxonis_4-13-2022_OAK-D-Lite_AF%20Datasheet%204-12-22.pdf
- Luxonis IMX214 details: https://docs.luxonis.com/hardware/sensors/IMX214
- Luxonis compact-camera-module table: https://docs.luxonis.com/hardware/platform/sensors/ccms/
- Luxonis depth accuracy: https://docs.luxonis.com/hardware/platform/depth/depth-accuracy/
- DepthAI `StereoDepth`: https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/stereo_depth
- DepthAI `ImgFrame`: https://docs.luxonis.com/software-v3/depthai/depthai-components/messages/img_frame
- Luxonis BMI270: https://docs.luxonis.com/hardware/platform/sensors/imu/bmi270/
- DepthAI IMU example: https://docs.luxonis.com/software-v3/depthai/examples/imu/imu_accelerometer_gyroscope

---

# 4. Adafruit MLX90640 IR Thermal Camera Breakout — 55° Version

## 4.1 Intended BillieBot Role

The MLX90640 provides a low-resolution radiometric thermal image suitable for:

- Detecting a warm animal against a cooler background.
- Supporting Billie localization when visible-light imagery is poor.
- Estimating relative surface temperature patterns.
- Detecting warm appliances or anomalous hot regions.

It is not equivalent to a 640 × 320 or 640 × 480 thermal camera. Its native array is only 32 × 24.

## 4.2 Optical and Thermal Specifications

| Parameter | Specification | Status / notes |
|---|---:|---|
| Sensor array | **32 × 24 = 768 pixels** | Manufacturer-stated |
| Field of view | **55° horizontal × 35° vertical** | Manufacturer-stated for 55° variant |
| Angular sample spacing, approximate | 1.72°/pixel horizontal; 1.46°/pixel vertical | Calculated by FOV/pixel count; not true optical resolution |
| Object-temperature range | **-40 to +300 °C** | Manufacturer-stated |
| Commonly quoted accuracy | **±2 °C over 0–100 °C** | Adafruit summary; accuracy varies with target and ambient temperature |
| Noise-equivalent temperature difference | Approximately **0.1 K RMS at 1 Hz** | Melexis datasheet |
| Sensor operating temperature | -40 to +85 °C | Melexis datasheet |
| ADC resolution setting | Programmable 16, 17, 18, or 19 bits; 18-bit default | Manufacturer-stated |
| Refresh-rate settings | 0.5, 1, 2, 4, 8, 16, 32, or 64 Hz at sensor level | Manufacturer-stated |
| Practical Adafruit host rate | Up to approximately **16 Hz**; Adafruit reports 32 Hz was not reliably achieved | Board/library guidance |
| Frame acquisition | Two interleaved/chess subpages are combined into a complete image | Manufacturer/library behavior |

## 4.3 Interface

| Parameter | Specification |
|---|---:|
| Digital interface | I²C |
| Default 7-bit address | `0x33` |
| Sensor bus rate | 400 kHz typical, up to 1 MHz per Melexis timing |
| Breakout connectors | STEMMA QT/Qwiic plus 0.1-inch header pads |
| Breakout input voltage | 3–5 V via Adafruit regulator/level-shifting circuitry |
| Native sensor supply | Approximately 3.3 V |

The 768-pixel calibration and image transactions are relatively large for I²C. High refresh rates require a host and kernel/driver configuration that can sustain the selected bus speed.

## 4.4 Native Data Versus Temperature Output

### Critical clarification

The MLX90640 does **not** send a simple 12-bit image in which:

```text
0 -> -40 °C
4096 -> +300 °C
```

That example would be incorrect for this device.

The actual acquisition chain is:

1. Read sensor EEPROM calibration coefficients.
2. Read an 834-word working frame buffer containing pixel RAM and auxiliary values.
3. Acquire both subpages.
4. Interpret signed raw infrared values and compensation channels.
5. Correct for sensor supply, ambient temperature, per-pixel gain/offset, bad pixels, emissivity, and reflected temperature.
6. Calculate an object temperature for each of the 768 pixels.

### Common Adafruit Python/CircuitPython output

The `getFrame()` API fills a **768-element array of floating-point temperatures in degrees Celsius**. The values can be reshaped as a 24-row × 32-column image.

Conceptually:

```python
frame = [0.0] * 768
mlx.getFrame(frame)

temperature_c = frame[row * 32 + column]
```

Example:

```text
frame[327] = 31.6  -> calculated object temperature = 31.6 °C
```

The temperature is a calibrated floating-point result, not a direct linear integer code.

### Raw register representation

- Pixel and auxiliary RAM are transferred as 16-bit register words.
- Raw pixel infrared values use signed/two's-complement interpretation.
- The selected internal ADC resolution affects acquisition scaling and noise.
- Conversion to object temperature requires device-specific EEPROM coefficients and the Melexis calculation sequence.
- Emissivity assumptions materially affect the calculated temperature. Adafruit’s driver commonly uses an emissivity value of 0.95 unless changed in code.

## 4.5 Frame Rate and Motion Artifacts

A complete frame is assembled from two subpages. Rapid sensor or target motion can therefore produce a checkerboard/interleaving artifact because the two halves are acquired at slightly different times.

For BillieBot:

- **2–8 Hz** is a reasonable initial operating region for robust Raspberry Pi I²C acquisition (**engineering recommendation**, not a manufacturer limit).
- 16 Hz may be possible with proper bus configuration.
- Thermal tracking should tolerate lower spatial and temporal resolution than the OAK camera.

## 4.6 Power

| Parameter | Value | Status |
|---|---:|---|
| Native MLX90640 current, typical | 20 mA | Manufacturer-stated |
| Native MLX90640 current limits | 15 mA minimum / 25 mA maximum in datasheet table | Manufacturer-stated |
| Typical sensor-only power | 3.3 V × 20 mA = **66 mW** | Calculated |
| Maximum sensor-only power from tabulated values | 3.3 V × 25 mA = **82.5 mW** | Calculated |
| Recommended breakout power allocation | **0.15 W** | Engineering estimate including regulator, level shifting, and margin |

## 4.7 Range and Spatial Resolution

Melexis does not publish one fixed maximum target range because thermal detectability depends on:

- Target angular size.
- Target/background temperature contrast.
- Emissivity.
- Atmospheric attenuation.
- Desired confidence and processing.
- Whether the task is detection, localization, or accurate thermometry.

Approximate target footprint at range \(R\), using the total FOV:

| Range | Horizontal scene width | Vertical scene height | Approximate horizontal footprint per native pixel |
|---:|---:|---:|---:|
| 0.5 m | 0.52 m | 0.32 m | 16 mm |
| 1.0 m | 1.04 m | 0.63 m | 33 mm |
| 2.0 m | 2.08 m | 1.26 m | 65 mm |
| 3.0 m | 3.12 m | 1.89 m | 98 mm |

These are geometric estimates. A small dachshund can occupy only a few native pixels at room-scale distances, so detection should use the full warm-body blob rather than individual anatomical detail.

## 4.8 BillieBot Integration Notes

- Mount away from the Jetson, Raspberry Pi heat sinks, regulators, motors, and exhaust airflow.
- Do not cover the sensor with ordinary glass or acrylic; common visible-light windows may be opaque in the long-wave infrared band.
- Allow thermal stabilization after power-up.
- Use a configurable emissivity rather than assuming every surface is 0.95.
- Apply bad-pixel correction, temporal filtering, and blob-level tracking.
- Preserve the un-interpolated 32 × 24 values for quantitative processing; interpolation only improves visualization, not actual thermal resolution.

### Primary sources

- Adafruit product/overview guide: https://learn.adafruit.com/adafruit-mlx90640-ir-thermal-camera/overview
- Adafruit Python/CircuitPython guide: https://learn.adafruit.com/adafruit-mlx90640-ir-thermal-camera/python-circuitpython
- Adafruit CircuitPython driver: https://github.com/adafruit/Adafruit_CircuitPython_MLX90640/blob/main/adafruit_mlx90640.py
- Adafruit Jupyter example: https://learn.adafruit.com/jupyter-on-any-computer-with-circuitpython-libraries-and-mcp2221/thermal-camera
- Melexis MLX90640 datasheet: https://www.melexis.com/-/media/files/documents/datasheets/mlx90640-datasheet-melexis.pdf

---

# 5. Raspberry Pi Camera Module 3 NoIR — Standard 75° Lens

## 5.1 Intended BillieBot Role

The Camera Module 3 NoIR is the Raspberry Pi-side visible/near-infrared imager. It can provide:

- Daytime color imagery.
- Night imagery when paired with an external IR illuminator.
- High-resolution stills.
- Video for event clips and behavior review.
- Autofocus for near-to-room-scale scenes.

“NoIR” means that the normal infrared-cut filter is absent. It is not a thermal camera and does not measure temperature.

## 5.2 Optical and Sensor Specifications

| Parameter | Specification | Status / notes |
|---|---:|---|
| Sensor | Sony IMX708 | Manufacturer-stated |
| Sensor architecture | Back-illuminated stacked CMOS, Quad Bayer | Manufacturer-stated |
| Shutter | Rolling shutter | Manufacturer-stated |
| Effective resolution | **4608 × 2592**, 11.9 MP | Manufacturer-stated |
| Pixel pitch | 1.4 µm × 1.4 µm | Manufacturer-stated |
| Sensor diagonal | 7.4 mm | Manufacturer-stated |
| Standard-lens field of view | **75° diagonal / 66° horizontal / 41° vertical** | Manufacturer-stated |
| Focal length | Approximately 4.74 mm | Product brief |
| Aperture | f/1.8 | Product brief |
| Focus mechanism | Phase-detection autofocus, motorized | Manufacturer-stated |
| Focus range | Approximately **0.10 m to infinity** | Manufacturer-stated |
| HDR | Up to approximately 3 MP output | Manufacturer-stated |
| IR-cut filter | **Absent** | NoIR variant |
| Operating temperature | 0 to 50 °C | Product brief |
| Board dimensions | Approximately 25 × 24 × 11.5 mm | Manufacturer-stated |
| Ribbon cable | 200 mm, 15-pin camera-end FPC | Manufacturer-stated |

## 5.3 Frame Rates and Modes

| Mode | Published common maximum |
|---|---:|
| Full-resolution still | 4608 × 2592 |
| 1080p | 50 fps |
| 720p | 100–120 fps depending on current documentation/mode |
| 480p | Up to approximately 120 fps in product brief |
| HDR | Up to 3 MP output |

Actual available modes are exposed by `libcamera`/Picamera2 and depend on the Raspberry Pi model, software version, selected crop/binning, ISP workload, and concurrent processing.

## 5.4 Electrical and Digital Interface

| Parameter | Specification |
|---|---:|
| Image interface | 2-lane MIPI CSI-2 |
| Sensor output | **RAW10** Bayer |
| Sensor/control bus | Two-wire serial control, including focus control |
| Physical connection | Ribbon cable to Raspberry Pi CSI connector |
| Pi 5 cable note | Pi 5 uses the smaller 22-pin board connector, requiring the appropriate standard-to-mini cable |
| Separate external power | None; powered through the camera connector |

## 5.5 Data Received and Host File Formats

### Native sensor stream

The IMX708 sends:

- 4608 × 2592 maximum active pixel array.
- Bayer color-filter-array samples.
- Nominal **10-bit raw pixel values** over CSI-2.
- Exposure/gain and autofocus control/metadata through the Raspberry Pi camera stack.

The raw code is photocharge/brightness-related, not a calibrated physical radiance or temperature value.

### ISP-processed data

The Raspberry Pi image signal processor can produce:

- RGB streams.
- YUV420 streams.
- Downscaled/cropped streams.
- Autofocus and auto-exposure/white-balance-controlled imagery.

### Common still-image outputs

`rpicam-still` can create:

- JPEG (`.jpg`) — default compressed still.
- PNG (`.png`).
- BMP (`.bmp`).
- Raw RGB binary dump.
- Raw YUV420 binary dump.
- DNG (`.dng`) raw Bayer capture when `--raw` is requested.

DNG stores the sensor data and capture metadata in a standardized raw-image container. A DNG’s storage word width may be 16 bits even when the meaningful sensor sample is 10 or 12 bits.

### Common video outputs

Depending on the selected Raspberry Pi application and encoder:

- H.264 elementary stream.
- MJPEG.
- YUV420 raw stream.
- Containerized files such as MP4 when using libav/FFmpeg options.

Picamera2 can also expose NumPy arrays and multiple simultaneous streams.

## 5.6 Range and Accuracy

This camera provides no direct metric range measurement.

- **Focus range:** about 0.10 m to infinity.
- **Usable recognition range:** application-dependent, based on target size, lighting, lens FOV, motion blur, compression, and inference model.
- **Geometric accuracy:** requires intrinsic calibration and, for metric measurements, an external scale/depth source.
- **Night range:** determined primarily by IR illuminator wavelength, radiant intensity, beam angle, exposure, and target reflectivity.

NoIR images can have unusual color balance in daylight because infrared contributes to the RGB pixel response.

## 5.7 Power

Raspberry Pi does not publish a Camera Module 3 board-level normal current/power value in the product brief.

- **Published value:** Not available.
- **Engineering allocation:** Reserve **1.0 W incremental at the Raspberry Pi input** for camera operation, autofocus activity, and associated Pi camera-interface/ISP overhead.
- This is a power-tree allowance, not a claimed measured camera-module consumption.

## 5.8 BillieBot Integration Notes

- Add an eye-safe IR illuminator for darkness; the NoIR camera does not emit light.
- Avoid an illuminator placed too close to the lens if nearby dust, fur, or a protective cover can backscatter.
- Use exposure limits to control motion blur while BillieBot or Billie is moving.
- Account for rolling-shutter distortion.
- Secure the ribbon cable and avoid repeated sharp flexing.
- Use the OAK depth stream, not monocular image size alone, for primary metric ranging.

### Primary sources

- Raspberry Pi Camera Module 3 product page: https://www.raspberrypi.com/products/camera-module-3/
- Raspberry Pi camera hardware documentation: https://www.raspberrypi.com/documentation/accessories/camera.html
- Raspberry Pi camera software documentation: https://www.raspberrypi.com/documentation/computers/camera_software.html
- Camera Module 3 product brief, linked from the official product page.

---

# 6. reSpeaker XVF3800 USB Four-Microphone Array

## 6.1 Intended BillieBot Role

The XVF3800 array provides:

- Far-field speech and bark capture.
- Beamformed/processed audio for automatic speech recognition.
- Acoustic echo cancellation when BillieBot plays audio.
- Voice activity detection.
- Direction of arrival.
- Raw microphone channels when using six-channel USB firmware.
- Audio output through a 3.5 mm connection or amplified speaker connector.

## 6.2 Hardware and Acoustic Processing

| Parameter | Specification | Status / notes |
|---|---:|---|
| Main processor | XMOS XVF3800 | Manufacturer-stated |
| Microphones | 4 PDM MEMS microphones in a circular/square-coordinate array | Manufacturer-stated |
| Pickup coverage | 360° | Manufacturer-stated |
| Claimed far-field pickup distance | Up to **5 m** | Manufacturer-stated; environment dependent |
| Acoustic processing | AEC, AGC, multi-beamforming, dereverberation, dynamic noise suppression, VAD, DoA | Manufacturer-stated |
| AGC range | 60 dB | Manufacturer-stated |
| Audio codec | TLV320AIC3104 | Manufacturer-stated |
| LED array | 12 individually addressable WS2812 RGB LEDs | Manufacturer-stated |
| Mute control | Physical button and status indication | Manufacturer-stated |
| Audio outputs | 3.5 mm AUX/headphone output and JST speaker connection | Manufacturer-stated |
| Amplified-speaker support | Listed as 5 W | This is output capability, not the mic array’s idle power |
| Approximate microphone coordinates | (±0.033 m, ±0.033 m, 0) | Returned by official control command; approximately 66 mm side spacing |

The 5 m value is a product capability claim, not a guaranteed speech-recognition range. Reverberation, fan/motor noise, speaker echo, source level, microphone orientation, and room geometry strongly affect performance.

## 6.3 Interfaces and Firmware Modes

| Mode | Interface | Audio configuration |
|---|---|---|
| USB 2-channel firmware | USB Audio Class 2.0 | 16 kHz, 32-bit endpoint, 2 processed channels |
| USB 6-channel firmware | USB Audio Class 2.0 | 16 kHz, 32-bit endpoint, 2 processed + 4 raw channels |
| Standard I²S firmware | I²S | 2 processed channels, 32-bit |
| Home Assistant I²S master firmware | I²S | 48 kHz, 2 processed channels |
| Device control | USB control/Python; exposed I²C/I²S headers depending on firmware | Firmware and command dependent |
| Firmware update | USB DFU in USB firmware; I²C DFU in I²S firmware; safe mode supports both | Manufacturer-stated |

## 6.4 USB Channel Mapping

### Two-channel USB firmware

| Channel | Meaning |
|---:|---|
| 0 | Conference-processed audio |
| 1 | ASR-processed audio |

### Six-channel USB firmware

| Channel | Meaning |
|---:|---|
| 0 | Conference-processed audio |
| 1 | ASR-processed audio |
| 2 | Raw microphone 0 |
| 3 | Raw microphone 1 |
| 4 | Raw microphone 2 |
| 5 | Raw microphone 3 |

The exact DSP differences between “Conference” and “ASR” are controlled by firmware and tuning parameters. Use the ASR channel for voice-recognition tests and preserve raw channels during characterization.

## 6.5 Audio Data Representation and File Types

The USB endpoint provides PCM audio. Seeed documents the firmware endpoint as:

- 16,000 samples/s for standard USB firmware.
- 32-bit sample depth at the endpoint.
- 2 or 6 interleaved channels.

Host audio systems may request or convert the stream to another representation. Seeed examples include:

```bash
arecord -D plughw:<card>,0 -c 2 -r 16000 -f S16_LE -d 5 output.wav
```

That example creates:

- 16 kHz sample rate.
- Two channels.
- Signed 16-bit little-endian PCM.
- A `.wav` container.

Therefore, “32-bit firmware depth” and a 16-bit WAV recording are not contradictory: ALSA can expose or convert formats selected by the host.

Approximate uncompressed data rates:

| Stream | Calculation | Payload rate |
|---|---:|---:|
| 2-channel, 16 kHz, 32-bit | 2 × 16,000 × 4 bytes | 128 kB/s |
| 6-channel, 16 kHz, 32-bit | 6 × 16,000 × 4 bytes | 384 kB/s |
| 2-channel, 16 kHz, S16_LE | 2 × 16,000 × 2 bytes | 64 kB/s |

Container headers and USB transport overhead are additional.

## 6.6 Metadata and Control Data

The official Python control interface can retrieve or configure:

- Direction of arrival (`DOA_VALUE`).
- Voice activity detection.
- Beam energy.
- Firmware version.
- Microphone geometry.
- LED color/brightness/effects.
- AEC, AGC, noise suppression, channel routing, and other tuning parameters.

An example command returns:

```text
DOA_VALUE: [135]
```

This is treated by the device UI as an azimuthal direction around the array. Seeed’s introductory page does not explicitly define the coordinate zero direction and sign convention in the tabulated specification; establish these experimentally after final mounting.

## 6.7 Accuracy and Range

| Quantity | Published specification |
|---|---|
| Voice pickup distance | Up to 5 m |
| Direction-of-arrival numerical accuracy | Not published on the Seeed product page |
| VAD probability/error performance | Not published |
| Frequency response, microphone sensitivity, SNR, acoustic overload | Not published for the complete XVF3800 board on the cited introduction page |
| AEC cancellation performance | Not published as a single board-level number |

These should be verified in BillieBot-specific acceptance tests with motors, cooling fans, speaker playback, and normal apartment reverberation.

## 6.8 Power

Seeed’s current XVF3800 introductory documentation does not publish board-level current consumption.

| Item | Value |
|---|---:|
| Supply | 5 V over USB-C |
| Published array current | Not available |
| Provisional array-only design allocation | **0.5 A at 5 V = 2.5 W** |
| Speaker power | Excluded from the above allowance |
| LED effect | Can materially increase consumption depending on brightness and number of lit LEDs |

Do not reuse the older XVF3000 array’s published 170–180 mA value as though it were an XVF3800 specification.

## 6.9 BillieBot Integration Notes

- Orient the microphone inlets as directed by Seeed; the wiki specifies that the sound holes on the logo side should face the source.
- Mechanically isolate the array from chassis vibration and speaker coupling.
- Keep it away from Jetson/Pi fans and airflow turbulence.
- Use the six-channel firmware during system characterization to compare raw microphones against processed outputs.
- Characterize DoA after mounting because enclosure diffraction changes the response.
- Maintain a playback reference path for effective AEC.
- Consider disabling or dimming LEDs when minimizing power or optical distraction.

### Primary sources

- Seeed XVF3800 introduction: https://wiki.seeedstudio.com/respeaker_xvf3800_introduction/
- Seeed XVF3800 product page: https://www.seeedstudio.com/ReSpeaker-XVF3800-USB-Mic-Array-p-6488.html
- XMOS XVF3800 overview: https://www.xmos.com/xvf3800
- XMOS documentation: https://www.xmos.com/documentation/XM-014888-PC/html/modules/fwk_xvf/doc/datasheet/02_overview.html

---

# 7. DFRobot Gravity 10-DOF IMU AHRS — BNO055 + BMP280 (SEN0253)

## 7.1 Intended BillieBot Role

This module provides two independent I²C devices:

1. **BNO055:** acceleration, angular velocity, magnetic field, fused Euler angles, quaternion, linear acceleration, gravity vector, calibration status, and temperature.
2. **BMP280:** barometric pressure, sensor temperature, and host-derived altitude.

“10 DOF” is a marketing/counting convention: 3-axis acceleration + 3-axis gyro + 3-axis magnetometer + pressure/altitude channel. It does not mean ten independent orientation axes.

## 7.2 Module-Level Specifications

| Parameter | Specification | Status / notes |
|---|---:|---|
| DFRobot SKU | SEN0253 | Manufacturer-stated |
| Input voltage | 3.3–5 V DC | Manufacturer-stated |
| Interface | Gravity I²C | Manufacturer-stated |
| BNO055 address | `0x28` default; selectable `0x29` | Manufacturer-stated |
| BMP280 address | `0x76` | Manufacturer-stated |
| DFRobot operating current | 5 mA | Manufacturer-stated, but see power discrepancy below |
| Module operating temperature | -40 to +80 °C | DFRobot module specification |
| Dimensions | 32 × 27 mm | Manufacturer-stated |

## 7.3 BNO055 Raw Sensors

### Accelerometer

| Parameter | Specification |
|---|---:|
| Axes | X, Y, Z |
| Internal resolution | 14-bit sensor |
| Full-scale ranges | ±2, ±4, ±8, ±16 g |
| Output sensitivity | 1 LSB/mg, or 100 LSB per m/s² depending on selected units |
| Programmable low-pass bandwidth | Approximately 8 Hz to 1 kHz |
| Typical noise density | 150–190 µg/√Hz under listed conditions |
| Zero-g offset | Typical ±80 mg, maximum ±150 mg in component table |
| Data register representation | Three signed 16-bit little-endian values |

### Gyroscope

| Parameter | Specification |
|---|---:|
| Axes | X, Y, Z |
| Internal resolution | 16-bit |
| Full-scale ranges | ±125, ±250, ±500, ±1,000, ±2,000 °/s |
| Output sensitivity | 16 LSB/(°/s), or 900 LSB/(rad/s) |
| Programmable bandwidth | Approximately 12–523 Hz |
| Typical zero-rate offset | ±1 °/s, maximum ±3 °/s under listed conditions |
| Typical RMS noise | Approximately 0.1 °/s at 47 Hz bandwidth |
| Data register representation | Three signed 16-bit little-endian values |

### Magnetometer

| Parameter | Specification |
|---|---:|
| Axes | X, Y, Z |
| Range | ±1,300 µT X/Y; ±2,500 µT Z, typical |
| Resolution | Approximately 0.3 µT |
| Register scaling | 16 LSB/µT |
| Typical heading accuracy | ±2.5° under a 30 µT horizontal field, fully calibrated with ideal tilt compensation |
| Calibrated zero-field offset | Approximately ±2 µT typical |
| Data register representation | Three signed 16-bit little-endian values |

The heading specification assumes an ideal magnetic environment. Motors, steel fasteners, speakers, high-current conductors, batteries, and the robot chassis can create much larger errors.

## 7.4 BNO055 Fused Outputs

| Output | Register representation | Scaling / units |
|---|---|---|
| Euler heading, roll, pitch | 3 × signed 16-bit | 16 LSB/degree or 900 LSB/radian |
| Quaternion W, X, Y, Z | 4 × signed 16-bit | \(2^{14}\) LSB per unit quaternion value |
| Linear acceleration X/Y/Z | 3 × signed 16-bit | 100 LSB/(m/s²) or 1 LSB/mg |
| Gravity vector X/Y/Z | 3 × signed 16-bit | 100 LSB/(m/s²) or 1 LSB/mg |
| Raw/calibrated acceleration | 3 × signed 16-bit | Selected SI or mg units |
| Raw/calibrated gyro | 3 × signed 16-bit | Selected °/s or rad/s units |
| Raw/calibrated magnetic field | 3 × signed 16-bit | 16 LSB/µT |
| Temperature | Signed 8-bit | 1 °C/LSB |
| Calibration state | Packed status fields | 0–3 for system, gyro, accelerometer, magnetometer |

### Fusion output rates

| Fusion mode | Accel output | Magnetometer output | Gyro output | Fusion output |
|---|---:|---:|---:|---:|
| IMU | 100 Hz | N/A | 100 Hz | 100 Hz |
| COMPASS | 20 Hz | 20 Hz | N/A | 20 Hz |
| M4G | 50 Hz | 50 Hz | N/A | 50 Hz |
| NDOF_FMC_OFF | 100 Hz | 20 Hz | 100 Hz | 100 Hz |
| NDOF | 100 Hz | 20 Hz | 100 Hz | 100 Hz |

### Euler-angle ranges

| Angle | Range |
|---|---:|
| Heading/yaw | 0° to 360° |
| Roll | -90° to +90° |
| Pitch | Convention-dependent; Android and Windows modes differ in sign/range convention |

For robot software, quaternions are normally preferable to Euler angles because Euler representations have wrapping and singularity issues.

### Important BNO055 limitation

The Bosch datasheet explicitly warns that the linear-acceleration output normally cannot be integrated once for reliable velocity or twice for reliable position; bias error rapidly dominates. Use wheel odometry, visual odometry, and other constraints.

## 7.5 BMP280 Pressure and Temperature Sensor

| Parameter | Specification | Status / notes |
|---|---:|---|
| Pressure range | **300–1,100 hPa** | Manufacturer-stated |
| Approximate altitude envelope | +9,000 to -500 m | Bosch equivalent atmosphere range |
| Relative pressure accuracy | **±0.12 hPa**, approximately ±1 m | 700–900 hPa and 25–40 °C |
| Absolute pressure accuracy | **±1.0 hPa**, approximately ±8.3 m | 0–65 °C full-accuracy region |
| Absolute temperature accuracy | ±0.5 °C at 25 °C; ±1.0 °C over 0–65 °C | Bosch component specification |
| Pressure output resolution, ultra-high mode | 0.0016 hPa = 0.16 Pa | Manufacturer-stated |
| Temperature output resolution | 0.01 °C in final compensated output | DFRobot/Bosch |
| Full-band pressure RMS noise | Approximately 1.3 Pa / 11 cm in ultra-high-resolution mode | Manufacturer-stated |
| Lowest filtered pressure noise | Approximately 0.2 Pa / 1.7 cm | Manufacturer-stated |
| Temperature coefficient of pressure offset | ±1.5 Pa/K, approximately 12.6 cm/K | Manufacturer-stated |
| Full operating temperature | -40 to +85 °C | Bosch component |
| Full specified accuracy temperature | 0 to +65 °C | Bosch/DFRobot |
| Maximum basic sampling rate | Approximately 157–182 Hz at ×1 pressure and temperature oversampling | Manufacturer-stated |

### BMP280 native and calculated data

The pressure and temperature conversion registers each contain a **20-bit unsigned raw ADC value**, spread across three registers:

```text
MSB[7:0] | LSB[7:0] | XLSB[7:4]
```

The host must read factory trimming coefficients and run Bosch compensation formulas. Typical library outputs are:

- Temperature in degrees Celsius.
- Pressure in pascals or hectopascals.
- Altitude in metres, calculated from pressure and an assumed sea-level reference.

DFRobot explicitly notes that altitude is calculated rather than directly measured.

A common barometric altitude relationship is:

\[
h \approx 44330\left[1-\left(\frac{P}{P_0}\right)^{0.1903}\right]
\]

where \(P_0\) is the reference sea-level pressure. Weather changes and HVAC pressure effects can look like altitude changes indoors.

## 7.6 Current and Power Discrepancy

| Source | Stated value |
|---|---:|
| DFRobot complete-module page | 5 mA |
| Bosch BNO055 component, 9-DOF at 100 Hz | Up to 12.3 mA total supply current |
| Bosch BMP280 | µA-scale average depending on sampling; peak pressure-conversion current up to approximately 1.12 mA |

The DFRobot 5 mA module value is difficult to reconcile with Bosch’s BNO055 9-DOF maximum. Possible explanations include a different test mode, typical rather than worst-case behavior, or documentation simplification.

**BillieBot design recommendation:** Allocate **20 mA at 5 V = 0.10 W** and measure the actual SEN0253 current in the intended NDOF and pressure-sampling modes.

## 7.7 Accuracy, Calibration, and Mounting

- Perform the required BNO055 accelerometer, gyro, and magnetometer calibration.
- Save and restore calibration offsets if the software stack supports it.
- Mount the module rigidly with a documented axis transform.
- Keep it away from motors, motor wires, switching inductors, speakers, magnets, steel hardware, and high-current battery conductors.
- A magnetometer on a compact robot may not provide reliable absolute yaw without a careful magnetic survey and hard/soft-iron calibration.
- Barometric altitude is useful for trends and floor-change detection, not centimetre-accurate indoor vertical position.
- The BMP280 temperature is the sensor/PCB temperature and can be above ambient due to self-heating and nearby electronics.

### Primary sources

- DFRobot SEN0253 wiki: https://wiki.dfrobot.com/sen0253/
- Bosch BNO055 datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
- Bosch BMP280 datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf

---

# 8. Recommended BillieBot Data Products

The following normalized data products would make downstream software consistent:

| Sensor | Recommended normalized output |
|---|---|
| OAK RGB | Timestamped BGR/RGB image plus camera calibration |
| OAK mono | Timestamped `mono8` left/right images if needed |
| OAK depth | `uint16` depth image in millimetres, invalid = 0, plus camera info |
| OAK disparity | Preserve raw disparity and configuration metadata when debugging |
| OAK IMU | Acceleration in m/s² and angular velocity in rad/s with covariance and frame ID |
| MLX90640 | 24 × 32 `float32` array in °C plus emissivity, ambient estimate, and timestamp |
| Pi NoIR | Timestamped image with exposure, gain, focus position, and camera calibration |
| reSpeaker | PCM stream with explicit sample rate, sample type, channel count, and channel map |
| reSpeaker metadata | DoA, VAD, beam energy, and firmware/tuning version |
| BNO055 | Quaternion, angular velocity, linear acceleration, magnetic field, calibration state |
| BMP280 | Pressure in Pa, temperature in °C, and derived altitude with reference pressure |

---

# 9. Provisional Combined Sensor Power Budget

This table excludes the Jetson, Raspberry Pi, lidar, motor system, IR illuminator, and speakers.

| Sensor | Conservative allocation |
|---|---:|
| OAK-D Lite | 7.50 W |
| MLX90640 breakout | 0.15 W |
| Camera Module 3 / associated Pi overhead | 1.00 W |
| reSpeaker array, no meaningful speaker load | 2.50 W |
| SEN0253 | 0.10 W |
| **Total provisional allocation** | **11.25 W** |

This is deliberately above expected normal sensor-only consumption. Add separate allowances for:

- IR illumination.
- reSpeaker-driven speaker output.
- USB conversion losses and cable voltage drop.
- 5 V regulator inefficiency.
- startup/transient margin.

---

# 10. Open Verification Items for BillieBot Bring-Up

The following values should be measured or verified on the actual hardware:

1. OAK-D Lite current in the final RGB + stereo + neural-network pipeline.
2. Whether the specific OAK-D Lite unit enumerates a BMI270.
3. OAK depth error against known targets from 0.25–5 m in BillieBot’s environment.
4. MLX90640 sustained frame rate and I²C error rate on the Raspberry Pi.
5. MLX90640 temperature offset after enclosure installation and warm-up.
6. Camera Module 3 incremental Pi input power during full-resolution and 1080p operation.
7. Camera Module 3 NoIR range with the selected IR illuminator.
8. XVF3800 current with LEDs off/on and with speaker playback.
9. XVF3800 DoA zero-angle/sign convention after mounting.
10. XVF3800 speech/bark performance with fans and drive motors operating.
11. SEN0253 current in NDOF mode.
12. BNO055 magnetic heading error after installation near the motor and power system.
13. BMP280 pressure drift as the robot electronics warm up.

---

## Revision Notes

- This reference prioritizes current manufacturer documentation.
- Older OAK-D Lite electrical/mechanical figures are retained where current pages do not expose the same detail.
- Engineering estimates are explicitly labeled and should be replaced with measured BillieBot values as hardware verification proceeds.
