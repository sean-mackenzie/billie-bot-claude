# billiebot_sensor_tests

Bench-test suite for BillieBot's four sensors — OAK-D Lite, MLX90640 thermal, Pi Camera 3
NoIR, reSpeaker XVF3800 — verified stand-alone, on their target host, before vehicle
integration. Implements every test in
[`docs/md/BILLIEBOT_SENSOR_BENCH_TEST_PLAN.md`](../../docs/md/BILLIEBOT_SENSOR_BENCH_TEST_PLAN.md),
which is the authoritative functional/verification spec — this README covers how to run
the suite, not the test rationale or acceptance-criteria derivation (see that document).

## Separation from `billiebot_tests`

`billiebot_tests` is a separate, existing package: a **mock-only, stack-level** smoke test
(`ros2 topic info`/`ros2 service list` checks against the full mock bringup ladder). It has
no rosbag2 recording, no real-hardware code, and no quantitative sensor analysis.
`billiebot_sensor_tests` is the opposite: **one real sensor at a time**, on its target
host, with recorded evidence and scored acceptance criteria. Neither package depends on
or modifies the other.

## Host assignments

| Sensor | Host | Interface |
|---|---|---|
| OAK-D Lite | Jetson Orin Nano | USB 3.x |
| MLX90640 | Raspberry Pi 5 | I²C bus 1, address `0x33` |
| Pi Camera 3 NoIR | Raspberry Pi 5 | CSI-2 |
| reSpeaker XVF3800 | Raspberry Pi 5 | USB |
| Sensor Nano — DFRobot SEN0253 (BNO055 `0x28` + BMP280 `0x76`) and the battery-sense divider | Jetson Orin Nano | USB serial to an Arduino Nano V3 @ 115200 baud; SEN0253 on the Nano's own I²C (A4/A5), divider on A0 |

Run only one sensor's tests at a time. Chassis, lidar, Nav2, and multi-machine DDS are never
required.

Two notes specific to the Sensor Nano suite:

> **The Motor Nano stays disconnected for the whole Sensor Nano campaign.** Do not launch
> `billiebot_base/base_bridge`: it still publishes its own `/battery_state` from the *Motor*
> Nano's A0 (BLK-02), and two publishers on that topic makes UT-BAT-01/02 meaningless. Two
> CH340 Nanos may also not expose distinct `/dev/serial/by-id` names (BLK-09) — prefer a
> `/dev/serial/by-path/...` entry, and connect only the Sensor Nano.

> **UT-BAT-02 and UT-BAT-02B are the one place this package launches mission software.** They
> deliberately drive the *real* `billiebot_mission` `mission_controller` with the production
> `mission.yaml`, because the shipping SAFE logic is what is under test. Nav2 is still not
> required — the controller constructs its action client without waiting for a server.

## Prerequisites

- The BillieBot ROS 2 workspace built and sourced on the target host (Jetson or Pi 5).
- Hardware-specific Python libraries staged on that host (see repo-root
  `docs/md/INSTALLATION_AND_SETUP.md` and `docker/Dockerfile`): `depthai==2.32.0.0`
  (Jetson), `adafruit-circuitpython-mlx90640` + Blinka (Pi 5, thermal), `picamera2`
  (Pi 5, NoIR), `sounddevice`/`pyusb`/`tflite-runtime` (Pi 5, audio). None of these are
  required to build this package or run its non-hardware unit tests.
- Two pins apply on **both** hosts and are not optional: `numpy==1.26.4` (the
  `tflite-runtime`/`depthai` wheels are built against the NumPy 1.x ABI — 2.x breaks them
  at import) and `depthai==2.32.0.0` (`oakd_bench_publisher.py` and the production
  `oakd_dog_detector.py` use the DepthAI v2 API; 3.x is a breaking rewrite neither is
  ported to). The audio preflight records the installed NumPy version in
  `exports/preflight.json` so a drifted host is visible in the evidence.
- apt tooling the preflight step shells out to: `usbutils` for `lsusb` (Jetson — OAK-D;
  Pi 5 — XVF3800) and `i2c-tools` for `i2cdetect` (Pi 5 — MLX90640). Neither is in the
  stock `ros:humble` image.
- For DT-AUD-01/02: a staged `yamnet.tflite` + `yamnet_class_map.csv` in the same
  directory, with `model_path` set in `billiebot_audio/config/audio.yaml` or passed as a
  launch argument (`model_path:=/path/to/yamnet.tflite`) — the production node exits
  nonzero at startup if this is missing (fail-loud).
- For hardware hookup steps (I²C pinout, CSI cable orientation, USB port selection), see
  the bench test plan §2 and the per-test hardware sections — not duplicated here.

## Running a test

Preferred: the orchestrator, which runs the launch file, records a bag, and scores it in
one command:

```bash
ros2 run billiebot_sensor_tests run_sensor_test --test-id UT-OAK-01 \
  --results-dir ~/billiebot_test_results/UT-OAK-01_$(date -u +%Y%m%dT%H%M%SZ) \
  --duration-sec 60
```

`--test-id` is any of: `UT-OAK-01`, `UT-OAK-02`, `DT-OAK-01`, `UT-THM-01`, `UT-THM-02`,
`DT-THM-01`, `UT-NIR-01`, `UT-NIR-02`, `UT-AUD-01`, `DT-AUD-01`, `DT-AUD-02`, `UT-IMU-01`,
`UT-IMU-02`, `UT-BAT-01`, `UT-BAT-02`, `UT-BAT-02B`.

The Sensor Nano tests additionally take `--sensor-port` (and optionally `--baudrate`), which
are forwarded to the launch file only when set:

```bash
ros2 run billiebot_sensor_tests run_sensor_test --test-id UT-IMU-01 \
  --results-dir ~/billiebot_test_results/UT-IMU-01_$(date -u +%Y%m%dT%H%M%SZ) \
  --sensor-port "$SENSOR_NANO_PORT"
```

`UT-BAT-01` is **operator-paced**: its default `duration_sec` is `0`, so the launch streams
until you press Ctrl-C and scoring runs immediately afterwards. Record each PSU setpoint from
a second terminal while it runs (see the per-test table below).

`UT-BAT-02B` needs **no hardware at all** — no Sensor Nano, no divider, no PSU — so it can be
run anywhere the workspace builds.

Equivalent, run in two steps (useful when you want to inspect the bag or copy it to
another machine before scoring):

```bash
ros2 launch billiebot_sensor_tests oakd_unit_bench.launch.py \
  results_dir:=~/billiebot_test_results/UT-OAK-01_smoke duration_sec:=60
ros2 run billiebot_sensor_tests analyze_oakd_depth \
  --results-dir ~/billiebot_test_results/UT-OAK-01_smoke --profile stream
```

Both paths produce identical `metrics.json`/exit codes — pass/fail is always computed
from the files written to `results_dir`, never from a launch/process return code.

### Per-test launch files and profiles

| Test ID | Launch file | Analysis command |
|---|---|---|
| UT-OAK-01 | `oakd_unit_bench.launch.py` | `analyze_oakd_depth --profile stream` |
| UT-OAK-02 | `oakd_unit_bench.launch.py test_mode:=flat_target` | `analyze_oakd_depth --profile accuracy --ground-truth-m 2.0` |
| DT-OAK-01 | `oakd_detection_bench.launch.py` | `score_oakd_detector` |
| UT-THM-01 | `thermal_unit_bench.launch.py` | `analyze_thermal_frame --profile stream` |
| UT-THM-02 | `thermal_unit_bench.launch.py duration_sec:=90` | `analyze_thermal_frame --profile contrast --ref-target-c 35 --ref-background-c 22` |
| DT-THM-01 | `thermal_detection_bench.launch.py` | `score_thermal_blob` |
| UT-NIR-01 | `noir_unit_bench.launch.py` | `analyze_noir_image --profile stream` |
| UT-NIR-02 | `noir_unit_bench.launch.py duration_sec:=15` (run 4×, one per lighting condition) | `analyze_noir_image --profile quality --capture-dir <run1> --condition normal_light --capture-dir <run2> --condition dark_with_ir ...` |
| UT-AUD-01 | `audio_capture_bench.launch.py capture_label:=ambient\|speech\|impulse` (run 3×) | `analyze_audio` |
| DT-AUD-01 | `audio_classifier_bench.launch.py` | `score_audio_classifier --profile classification` |
| DT-AUD-02 | `audio_classifier_bench.launch.py` (separate session, own `--results-dir`) | `score_audio_classifier --profile doa` |
| UT-IMU-01 | `sensor_nano_imu_bench.launch.py sensor_port:=$SENSOR_NANO_PORT` | `analyze_sensor_nano_imu --profile acquisition` |
| UT-IMU-02 | `sensor_nano_imu_ekf_bench.launch.py sensor_port:=$SENSOR_NANO_PORT` | `analyze_sensor_nano_imu --profile ekf` |
| UT-BAT-01 | `sensor_nano_battery_bench.launch.py sensor_port:=$SENSOR_NANO_PORT` (operator-paced, `duration_sec:=0`; run `record_battery_point` once per PSU setpoint from a second terminal) | `analyze_sensor_nano_battery` |
| UT-BAT-02 | `sensor_nano_battery_safe_bench.launch.py sensor_port:=$SENSOR_NANO_PORT` | `score_battery_safe --profile physical` |
| UT-BAT-02B | `sensor_nano_battery_threshold_bench.launch.py` (no hardware) | `score_battery_safe --profile threshold` |

### Sensor Nano ground-truth marking

UT-IMU-01 and UT-IMU-02 start `ground_truth_marker_node`. Type `mark <label>` into the launch
terminal at each hold; the labels the analyzers expect are configured under
`sensor_nano.ut_imu_01_rotation_sequence` / `ut_imu_02_rotation_sequence` in
`config/sensor_bench.yaml` (`flat`, `x_plus_90`, `x_minus_90`, `y_plus_90`, `y_minus_90`,
`z_plus_90`; and `flat`, `yaw_plus_90`, `yaw_minus_90`, `flat_end`).

**Without those marks the commanded-rotation criterion fails rather than being skipped** —
"clear correct-axis response with expected sign" is a required gate, and a run carrying no
evidence for it has not demonstrated it.

UT-BAT-01 ground truth is entered per setpoint instead:

```bash
ros2 run billiebot_sensor_tests record_battery_point \
  --results-dir "$RESULTS" \
  --setpoint-v 10.50 --dmm-battery-v 10.497 --dmm-a0-v 1.749
```

Each call appends one row to `exports/battery_points.csv`, pairing your DMM readings with the
live `/battery_state` and `/bench/battery/adc` values sampled over the same few seconds.

### Sensor Nano firmware

The Arduino sketch, its pinned library versions, and the `arduino-cli` build/flash commands
live in [`firmware/sensor_nano/README.md`](firmware/sensor_nano/README.md). Flash the Sensor
Nano before running any UT-IMU/UT-BAT test.

Ground-truth entry (distances, orientations, conditions) is stdin-driven during the
launch: type e.g. `mark dog_present distance=1.5m orientation=front condition=normal_light`
and press Enter — this is read by `ground_truth_marker_node` and appended to
`exports/ground_truth_segments.csv` with a real timestamp.

## Foxglove connection

Every launch file supports `start_foxglove:=true` (default) and `foxglove_port:=8765`
(default). On the sensor host:

```bash
hostname -I   # find the host IP
```

On your MacBook, open Foxglove Studio → **Open connection** → `ws://<host-IP>:8765`, then
**Import layout** and select `foxglove/billiebot_sensor_bench.json` (installed to
`share/billiebot_sensor_tests/foxglove/`). One layout covers all sensors/tests — tabs for
OAK-D, Thermal, NoIR, Audio, plus a shared `/bench/status` diagnostics strip. Panels for
topics not currently published simply show no data. Foxglove is qualitative evidence only
— numeric pass/fail always comes from `metrics.json`.

## Visualization topics vs. authoritative data

**The rule:** high-rate/high-volume raw sensor topics are *authoritative data products*,
intended for local rosbag2 recording and quantitative analysis. Remote visualization uses
dedicated downsampled, colorized, compressed and/or rate-limited `/bench/.../preview` topics.
Pass/fail always comes from the raw data; a preview topic must never gate a result.

### Why raw streams overwhelm a remote Foxglove connection

`foxglove_bridge` serves a WebSocket client over Wi-Fi and enforces a `send_buffer_limit`
(10 MB by default). When a client cannot keep up, the bridge **drops messages for that client**
rather than applying back-pressure to the publisher. Local subscribers — rosbag2, the rate
monitor — are entirely unaffected, so the sensor keeps running at full rate while the remote
panel appears frozen. The OAK-D raw set alone is far past what any Wi-Fi link can carry:

| Old default Foxglove subscription | Payload | Rate | Data rate |
|---|---|---|---|
| `/bench/oakd/rgb/image_raw` 1920×1080 `bgr8` | 6.22 MB | 5 Hz | 248.8 Mbit/s |
| `/bench/oakd/depth/image_raw` 640×400 `16UC1` | 512 kB | 5 Hz | 20.5 Mbit/s |
| `/bench/oakd/points` stride 4 (16 000 pts) | 192 kB | 5 Hz | 7.7 Mbit/s |
| **total** | | | **≈ 277 Mbit/s** |

This was observed on a UT-OAK-02 run: all three topics published at ~5.0 Hz, rosbag2 recorded
~279 messages each over ~55 s (~3 GiB bag), `topic_rate_monitor.json` passed — yet Foxglove
received ~4–5 messages per topic (~0.12 Hz).

> **Poor Foxglove rate does not imply poor sensor/ROS publication rate.** They are independent
> measurements of different things.

### Diagnosing it

1. `exports/topic_rate_monitor.json` — the authoritative rate evidence. This is what the sensor
   actually published, measured on the sensor host.
2. `ros2 bag info <results_dir>/bag` — message counts and duration per topic, recorded locally.
3. Only then look at Foxglove's per-panel rate.

If (1) and (2) are healthy and Foxglove is not, it is a **transport/visualization** problem, not
a sensor or acquisition problem. Do not relax an acceptance threshold in response.

### OAK-D visualization topics

These are what the shipped Foxglove layout subscribes to. All are `sensor_msgs/CompressedImage`
(JPEG) except the point cloud, which is `sensor_msgs/PointCloud2`.

| Topic | Test | Size | Rate | Compression | Payload |
|---|---|---|---|---|---|
| `/bench/oakd/rgb/preview/compressed` | UT-OAK-01/02 | 640×360 | 5 Hz | JPEG q70 | ~37 KiB |
| `/bench/oakd/depth/preview/compressed` | UT-OAK-01/02 | 320×200 | 5 Hz | JPEG q70, colorized 0.1–5.0 m | ~4–10 KiB |
| `/bench/oakd/points_preview` | UT-OAK-01/02 | stride 16 (~1 000 pts) | 2 Hz | — | ~12 kB |
| `/bench/oakd_detector/rgb/preview/compressed` | DT-OAK-01 | 416×416 | 5 Hz | JPEG q70 | ~27 KiB |
| `/bench/oakd_detector/annotated/preview/compressed` | DT-OAK-01 | 416×416 | 5 Hz | JPEG q70 | ~27 KiB |
| `/bench/oakd_detector/depth/preview/compressed` | DT-OAK-01 | 320×200 | 5 Hz | JPEG q70, colorized | ~4 KiB |

The UT-OAK preview set totals **≈ 1.8–2.1 Mbit/s against the old ≈ 277 Mbit/s — a ~130× reduction.**

The raw topics are unchanged and still recorded, still rate-gated, still the only thing analysis
reads. To inspect them deliberately in Foxglove, add a panel by hand — but expect the bandwidth
above, and prefer replaying the bag locally instead.

**The depth preview is not valid for quantitative depth analysis.** It is 8-bit colorized and
clipped to the display range; invalid pixels (`0` mm) and out-of-range pixels are painted black.
UT-OAK-02's accuracy math always reads raw `16UC1` millimetres from the bag.

### Overriding the defaults

Previews are on by default and independently disableable. Acquisition, recording, rate gating,
and analysis are identical either way:

```bash
# turn the whole visualization path off
ros2 launch billiebot_sensor_tests oakd_unit_bench.launch.py \
  results_dir:=... start_visualization_previews:=false

# tune resolution / rate / quality
ros2 launch billiebot_sensor_tests oakd_unit_bench.launch.py results_dir:=... \
  preview_width:=960 preview_height:=540 preview_jpeg_quality:=85 \
  depth_preview_min_m:=0.3 depth_preview_max_m:=3.0 \
  points_preview_stride:=8 points_preview_rate_hz:=1.0

# uncompressed downsampled Image instead of CompressedImage (no Pillow required)
ros2 launch billiebot_sensor_tests oakd_unit_bench.launch.py \
  results_dir:=... preview_format:=raw
```

The shipped defaults are also recorded under `oakd.visualization` in `config/sensor_bench.yaml`.
That block is documentation, deliberately **not** a threshold: nothing about a preview lives in
`thresholds.required` or `thresholds.provisional`, so no visualization setting can move a
pass/fail boundary.

Other sensors' bench topics are small enough that they still go to Foxglove directly (thermal
`32FC1` is ~3 kB/frame; NoIR `rgb8` 640×480 is ~921 kB/frame). `bench_preview_node` is
sensor-agnostic and configured entirely through two JSON parameters, so migrating them is a
launch-file and layout change with no new node code.

## Result-directory format

```text
<results_dir>/
├── manifest.yaml       # test ID, host/OS/ROS distro, git commit+dirty, launch command,
│                       # ground truth, operator notes -- written at start, finalized at shutdown
├── console.log         # preflight output + folded node logs
├── bag/                # rosbag2 (sqlite3) -- the authoritative time-series record
├── exports/            # preflight.json, topic_rate_monitor.json, ground_truth_segments.csv,
│                       # WAV files (audio), representative frames
├── plots/              # matplotlib PNGs from analysis CLIs
├── metrics.json        # machine-readable pass/fail -- the authoritative verdict
├── metrics.csv         # flattened metrics, for spreadsheet import
└── report.md           # human-readable summary
```

## Pass/fail interpretation

`metrics.json`'s top-level `"pass"` boolean is authoritative. `report.md`'s per-criterion
table tags each check `required` (system requirement — always gates) or `provisional`
(engineering gate — reported, logged with a `PROVISIONAL` warning, and only gates the
result where the test plan says it should). Never treat a provisional-tier failure as
equivalent to a required-tier failure without checking `report.md`.

## Common failure classifications

When a test fails, check in this order:

1. **Hardware/wiring** — preflight (`exports/preflight.json`, `console.log`) shows the
   device didn't enumerate (`lsusb`, `i2cdetect`, `rpicam-hello --list-cameras`,
   `arecord -l`). A `returncode: -1` with a `No such file or directory` stderr is *not* an
   enumeration failure — the tool itself is missing (`usbutils` for `lsusb`, `i2c-tools`
   for `i2cdetect`); install it and re-run before concluding anything about the hardware.
2. **Host driver/library** — preflight succeeds but the node logs an import/init error
   (missing `depthai`/`picamera2`/`adafruit_mlx90640`/`sounddevice`, or a `sys.exit(1)`
   fail-loud message naming a missing model/class-map file).
3. **ROS transport/topic** — `exports/topic_rate_monitor.json` shows a required topic
   never appeared or violated its rate/gap threshold, but the node itself started cleanly.
4. **Test harness** — the bag is empty (`bag.is_empty`) or `ground_truth_segments.csv` is
   missing/malformed; re-run with the operator actually typing `mark ...` commands.
5. **Algorithm/model** — rate/topics are fine but detection/classification metrics
   (recall, false-positive fraction, bias, CNR) miss threshold. This is a real sensor/
   algorithm characterization result, not a harness bug.
6. **Configuration** — check `config/sensor_bench.yaml` for a threshold that's
   misconfigured, or a `--config-file` override pointing at the wrong file.
7. **Environmental limitation** — explicitly called out in the test plan (e.g. dark-no-IR
   for NoIR, reference-thermometer uncertainty >1°C for thermal) — the report labels these
   "characterization only" and they don't fail the run.
8. **Visualization transport** — Foxglove panels are frozen or stuttering, but
   `exports/topic_rate_monitor.json` passes and `ros2 bag info` shows the expected message
   counts. This is **not a test failure at all**: the sensor, ROS, and the recording are healthy
   and the run's evidence is intact. A Foxglove panel is subscribed to a raw high-bandwidth topic
   instead of its `/bench/.../preview` counterpart — re-import the shipped layout. See
   "Visualization topics vs. authoritative data" above.

## Which tests require live hardware vs. run from fixtures

Every launch file (`launch/*.launch.py`) and every hardware-touching node
(`oakd/oakd_bench_publisher.py`, `audio/audio_capture_node.py`, and the *production*
`thermal_node.py`/`noir_cam_node.py`/`oakd_dog_detector.py`/`audio_classifier.py` they
invoke) requires real sensor hardware. **`colcon test --packages-select
billiebot_sensor_tests` requires no hardware at all** — it exercises only pure metric/
scoring functions (`*/metrics.py`, `oakd/detection_scoring.py`, `thermal/blob_scoring.py`,
`audio/classifier_scoring.py`, `sensor_nano/protocol.py`, `sensor_nano/imu_metrics.py`,
`sensor_nano/battery_metrics.py`, `sensor_nano/safety_metrics.py`) against synthetic fixtures
in `test/fixtures/`, and construct-checks every launch file's
`generate_launch_description()` without executing any node. Each `analyze_*`/`score_*` CLI
also has a `--self-test` flag that exercises its core metric against inline synthetic data
(no bag, no hardware) — useful as a post-install smoke check.

`UT-BAT-02B` is the one **bench test** that needs no hardware: it publishes synthetic
`BatteryState` at the exact 10.5 V boundary against the real production mission controller.
It is not part of `colcon test` (it is a bench run producing a full result directory), but it
can be executed on any machine where the workspace builds.

> **UT-BAT-02B is expected to FAIL today, and that failure is the deliverable.**
> `mission_controller.py:147` compares with a strict `<` while SYS-PLT-2 requires `<=`, so
> exactly 10.5000 V does not trigger SAFE. The 10.5001 V and 10.4999 V cases pass. This is
> BLK-05, a production requirement discrepancy — not a Sensor Nano hardware fault, and not a
> defect in the test. Do not "fix" it by changing the expectation.

## How to add a new bench test

1. Add a `TestSpec` entry to `orchestrate/test_registry.py` (test ID, sensor, launch file,
   analysis module + profile).
2. If it's a new sensor/topic set: add a launch file under `launch/`, reusing
   `common/launch_helpers.py`'s shared actions (`declare_common_bench_args`,
   `manifest_bootstrap_action`, `record_bag_action`, `foxglove_bridge_action`,
   `duration_shutdown_action`, `finalize_on_shutdown_action`, and
   `replicate_production_node` if it exercises a production detector/classifier).
3. Add pure metric functions to `<sensor>/metrics.py` (or a new `<sensor>/` package) and
   an `analyze_*`/`score_*` CLI following the existing pattern: argparse → `BagReader` →
   metric functions → `common.report.write_metrics_json`/`write_metrics_csv`/
   `render_report_md` → exit code.
4. Add every new acceptance threshold to `config/sensor_bench.yaml` under the sensor's
   `thresholds.required`/`thresholds.provisional` — never as a bare literal in code.
5. Add synthetic fixtures to `test/fixtures/` and a `test/test_<sensor>_metrics.py` case
   for every new metric function.
6. Add the new topics to `foxglove/billiebot_sensor_bench.json` if useful for live
   visualization — but **never point a Foxglove panel at a high-volume raw topic**. Anything
   above roughly 1 Mbit/s over the remote link (any camera image at rate, any point cloud)
   gets a `/bench/.../preview` counterpart instead: add a `bench_preview_node` to the launch
   file with the source topic in `image_sources_json`/`depth_sources_json`, and subscribe the
   panel to the preview. The raw topic still goes in the bag and the rate monitor. See
   "Visualization topics vs. authoritative data" above for the reasoning and the numbers.

## Validation

```bash
colcon build --packages-select billiebot_sensor_tests
colcon test --packages-select billiebot_sensor_tests
colcon test-result --verbose
```
