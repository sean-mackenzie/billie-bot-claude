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

Run only one sensor's tests at a time. Chassis, lidar, Nav2, mission, and multi-machine
DDS are never required.

## Prerequisites

- The BillieBot ROS 2 workspace built and sourced on the target host (Jetson or Pi 5).
- Hardware-specific Python libraries staged on that host (see repo-root
  `docs/md/INSTALLATION_AND_SETUP.md` and `docker/Dockerfile`): `depthai` (Jetson),
  `adafruit-circuitpython-mlx90640` + Blinka (Pi 5, thermal), `picamera2` (Pi 5, NoIR),
  `sounddevice`/`pyusb`/`tflite-runtime` (Pi 5, audio). None of these are required to
  build this package or run its non-hardware unit tests.
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
`DT-THM-01`, `UT-NIR-01`, `UT-NIR-02`, `UT-AUD-01`, `DT-AUD-01`, `DT-AUD-02`.

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
   `arecord -l`).
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

## Which tests require live hardware vs. run from fixtures

Every launch file (`launch/*.launch.py`) and every hardware-touching node
(`oakd/oakd_bench_publisher.py`, `audio/audio_capture_node.py`, and the *production*
`thermal_node.py`/`noir_cam_node.py`/`oakd_dog_detector.py`/`audio_classifier.py` they
invoke) requires real sensor hardware. **`colcon test --packages-select
billiebot_sensor_tests` requires no hardware at all** — it exercises only pure metric/
scoring functions (`*/metrics.py`, `oakd/detection_scoring.py`, `thermal/blob_scoring.py`,
`audio/classifier_scoring.py`) against synthetic fixtures in `test/fixtures/`, and
construct-checks every launch file's `generate_launch_description()` without executing any
node. Each `analyze_*`/`score_*` CLI also has a `--self-test` flag that exercises its core
metric against inline synthetic data (no bag, no hardware) — useful as a post-install
smoke check.

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
   visualization.

## Validation

```bash
colcon build --packages-select billiebot_sensor_tests
colcon test --packages-select billiebot_sensor_tests
colcon test-result --verbose
```
