# BillieBot — Foxglove Studio Guide

A hands-on companion to [VERIFICATION.md](VERIFICATION.md). That document tells you *what to launch and what must be true* at each rung of the bringup ladder; this one teaches you *how to see it* in Foxglove Studio, rung by rung, twice:

- **[Part 1](#part-1-the-bringup-ladder-in-mock-mode)** — rungs 01–14 with `mock:=true` (hardware-free, in the Docker container on your Mac). This is where you learn every Foxglove skill.
- **[Part 2](#part-2-the-bringup-ladder-on-the-real-robot)** — rungs 01–14 on the real BillieBot with all sensors. Shorter: it reuses the skills from Part 1 and adds only what's different with real hardware (bridge placement, safety, sensor sanity checks, real SLAM mapping).

The instructions build sequentially — each rung assumes you can do everything from the previous rungs. If you're comfortable in RViz, start with the [translation table](#03-coming-from-rviz-a-translation-table) below; most concepts carry over one-to-one.

> **Version note:** written against the Foxglove desktop app v2.x (2026). The product was renamed from "Foxglove Studio" to just "Foxglove"; this guide uses the names interchangeably. Exact menu labels occasionally drift between releases — if a setting isn't where this guide says, look in the same panel's settings sidebar; it will be nearby.

**Contents**

- [Part 0: Foxglove fundamentals (one-time setup)](#part-0-foxglove-fundamentals-one-time-setup)
- [Part 1: The bringup ladder in mock mode](#part-1-the-bringup-ladder-in-mock-mode)
- [Part 2: The bringup ladder on the real robot](#part-2-the-bringup-ladder-on-the-real-robot)
- [FAQ: where each question is answered](#faq-where-each-question-is-answered)
- [Appendix A: Beyond the bringup ladder](#appendix-a-beyond-the-bringup-ladder)
- [Appendix B: Full OAK-D Lite streams (RGB, stereo depth, point cloud)](#appendix-b-full-oak-d-lite-streams-rgb-stereo-depth-point-cloud)
- [Appendix C: Troubleshooting](#appendix-c-troubleshooting)

---

## Part 0: Foxglove fundamentals (one-time setup)

### 0.1 Install and connect

You should already have Foxglove installed from [INSTALLATION_AND_SETUP.md](INSTALLATION_AND_SETUP.md) §1.2 (`brew install --cask foxglove-studio`). On first launch it asks you to sign in — a free account is required and takes a minute.

Foxglove never talks DDS directly. It talks a WebSocket protocol to **`foxglove_bridge`**, a ROS 2 node you run wherever the ROS graph lives (the container in mock mode; the Jetson or Pi on the real robot). This is exactly why it beats RViz on a Mac: no X11, no DDS-through-Docker gymnastics — just one TCP port (8765), which the container already publishes (`-p 8765:8765`).

**Every session in Part 1 starts the same way.** Keep two container shells open:

```bash
# Shell A — the bridge (leave running all day)
host$ docker start billiebot-dev            # if not already running
host$ docker exec -it billiebot-dev bash
container$ source /opt/ros/humble/setup.bash && source /ws/install/setup.bash
container$ ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765

# Shell B — whatever rung you're testing
host$ docker exec -it billiebot-dev bash
container$ source /opt/ros/humble/setup.bash && source /ws/install/setup.bash
container$ ros2 launch billiebot_bringup 01_lidar.launch.py mock:=true   # (example)
```

Then in Foxglove: **Open connection…** → **Foxglove WebSocket** → `ws://localhost:8765` → **Open**.

The bridge is launched separately on purpose — no rung launch file starts it, so it survives as you Ctrl-C one rung and launch the next. Foxglove reconnects and re-subscribes automatically; you don't need to touch the app between rungs.

### 0.2 A five-minute tour of the UI

With the connection open (even with no rungs launched yet):

1. **Left sidebar → Topics.** The live topic list, with message type and receive rate (Hz) per topic. This replaces `ros2 topic list` + `ros2 topic hz` for most purposes and is the first thing to check at every rung.
2. **Panels.** The workspace is a grid of panels (RViz "displays" ≈ Foxglove "panels", but panels also cover plots, images, service calls, teleop…). Add one with the **Add panel** button (or split an existing panel from the menu in its top-right corner). Drag panel edges to resize, drag title bars to rearrange.
3. **Panel settings.** Click a panel, then the **settings (gear/sliders) icon** in its top-right — settings open in the left sidebar. Everything configurable about a panel lives there.
4. **Layouts.** The arrangement + settings of all panels is a **layout** (the analog of an RViz `.rviz` config). Open the **Layouts** menu (left sidebar) → **Create new layout**, name it **`BillieBot Mock`**. Layouts auto-save as you edit; you can export/import them as JSON files (see [Appendix A.2](#a2-layouts-save-share-commit)).
5. **Problems.** The sidebar's Problems tab reports subscription/connection issues — check it whenever a panel is unexpectedly empty.

You will build up the `BillieBot Mock` layout cumulatively through Part 1 — by Rung 14 it becomes your mission-control dashboard.

### 0.3 Coming from RViz: a translation table

| RViz concept | Foxglove equivalent |
|---|---|
| Displays panel | Panels (one per visualization; the **3D panel** hosts most RViz-style displays as its topic list) |
| Fixed Frame | 3D panel settings → **Frame** → **Display frame** |
| Add display → LaserScan / Map / Path / TF / RobotModel | 3D panel settings → **Topics** → toggle the topic's visibility |
| 2D Pose Estimate tool | 3D panel toolbar → **Publish** tool, type = *Pose estimate* → publishes `/initialpose` |
| 2D Nav Goal tool | 3D panel toolbar → **Publish** tool, type = *Pose* → publishes to a configurable topic (set it to `/goal_pose`) |
| `.rviz` config file | Layout (exportable JSON) |
| `rqt_plot` | **Plot** panel |
| `rqt_image_view` | **Image** panel |
| `rqt_console` | **Log** panel |
| `rqt_service_caller` | **Call service** panel |
| `rqt_robot_steering` / `teleop_twist_keyboard` | **Teleop** panel |
| — (no RViz equivalent) | **Raw Messages**, **State Transitions**, **Gauge**, **Indicator**, **User Scripts** panels |

One habit to unlearn: RViz displays live in a single 3D view; in Foxglove only *spatial* data goes in the 3D panel — everything else (numbers, states, images, raw fields) gets its own purpose-built panel. Embrace the dashboard.

### 0.4 Message paths (you'll use these constantly)

Plot, Gauge, Indicator, State Transitions, and Raw Messages panels all take a **message path**: a topic name plus a dotted path into the message, with autocomplete. Examples you'll use in Part 1:

```
/odom.pose.pose.position.x
/battery_state.voltage
/billie/state.state
/audio/events.doa_deg
/amcl_pose.pose.covariance[0]
```

Arrays support indexing (`[0]`) and slicing (`[:]`). Type slowly and let the autocomplete guide you — it knows the full message definitions, including BillieBot's custom `billiebot_interfaces` types.

---

## Part 1: The bringup ladder in mock mode

Work through these in order with your `BillieBot Mock` layout active. Per rung: the launch command (same as VERIFICATION.md), the new Foxglove skills, numbered steps, and what success looks like. Launch commands go in **Shell B** (Ctrl-C the previous rung first, unless a rung says otherwise); the bridge keeps running in Shell A.

> **What mock mode is:** every launch file accepts `mock:=true`, which swaps hardware drivers for synthetic publishers (there is no Gazebo — mock mode *is* the sim story). The data is fake but the topics, types, rates, and TF tree are real, which is exactly what you need to learn the tooling.

### Mock Rung 01: Lidar

```bash
container$ ros2 launch billiebot_bringup 01_lidar.launch.py mock:=true
```

**New skills:** Topics sidebar, Raw Messages panel, your first 3D panel, display frames, LaserScan styling.

1. **Topics sidebar:** `/scan` appears — `sensor_msgs/LaserScan` at ~10 Hz. Get in the habit: *topic exists → type is right → rate is right*, before you visualize anything.
2. **Add a Raw Messages panel.** Set its message path to `/scan`. Expand the message: `header.frame_id: laser_frame`, `ranges` is a 360-element array, `range_min: 0.15`, `range_max: 12.0`. Raw Messages is your `ros2 topic echo` — reach for it first whenever a topic misbehaves.
3. **Add a 3D panel.** It's empty because it doesn't know which frame to render in — rung 01 publishes no TF, so the only frame in existence is the scan's own. Open the panel settings → **Frame** → set **Display frame** to `laser_frame`.
4. In the 3D panel settings → **Topics**, toggle `/scan` visible. You should see a noisy rectangle of points, roughly **5 m × 4 m**, centered on the origin.
5. **Style the scan** (settings under the `/scan` topic entry): bump **Point size** to ~5, try a **color-by** field/gradient, and try **Decay time** = 2 s (points persist and smear — useful later for watching motion; set it back to 0).
6. Mouse controls: scroll = zoom, left-drag = orbit, right-drag = pan; there's a 2D/3D toggle in the panel toolbar for a top-down view (the natural view for a planar lidar).

**Success:** a crisp rectangle at 10 Hz. That rectangle is `mock_scan.py`'s synthetic 2.5 × 2.0 m half-extent room with 1 cm Gaussian noise.

> **Caveat that matters later:** the mock scan is generated *around the lidar* and never changes as the robot moves. Remember this at Rungs 04–06.

### Mock Rung 02: Base + Description

```bash
container$ ros2 launch billiebot_bringup 02_base.launch.py mock:=true
```

**New skills:** viewing the robot model (URDF), TF, Teleop, Plot, Gauge. *(This rung answers "How do I view the robot?")*

1. **Topics sidebar:** `/odom` (~30 Hz), `/joint_states` (~30 Hz), `/battery_state` (1 Hz), `/robot_description`, `/tf`, `/tf_static`.
2. **View the robot:** in the 3D panel, set **Display frame** to `odom`. In the panel's **Topics** list, toggle `/robot_description` visible — Foxglove parses the URDF published by `robot_state_publisher` and renders BillieBot's chassis, wheels, and sensor pods (the URDF uses primitive geometry, so nothing else is needed; a URDF with meshes would need the mesh files resolvable).
3. **View TF:** in 3D panel settings → **Transforms**, toggle frames visible. Confirm the tree from VERIFICATION.md rung 02: `odom → base_link → chassis → (laser_frame, oakd_link_optical, noir_link_optical, thermal_link_optical, mic_link, wheels)`. Turn frame labels on, admire, then hide the ones you don't need (they clutter fast).
4. **Drive it — add a Teleop panel.** In its settings: topic `/cmd_vel`, forward/back speed ±0.2 m/s, angular ±0.5 rad/s (the stack's Nav2 limits are 0.3 m/s / 1.0 rad/s — stay under them out of habit). Press and hold the arrows: the robot drives around the 3D panel. This works in mock because the fake Arduino integrates commanded wheel speeds into encoder ticks, which `base_bridge` turns into `/odom` + TF. Releasing the button stops the robot within ~0.5 s (`base_bridge` zeroes the motors when `/cmd_vel` goes quiet — `cmd_timeout_sec: 0.5`).
5. **Add a Plot panel.** Series: `/odom.pose.pose.position.x` and `/odom.pose.pose.position.y`. Drive and watch the integrals accumulate. Add `/odom.twist.twist.linear.x` to see commanded-vs-achieved velocity shape.
6. **Add a Gauge panel** on `/battery_state.voltage`, min 10, max 13. Mock pins it at ≈12.6 V (a battery-dead scenario can never fire in mock — worth knowing before you trust mock soak tests).

**Success:** you can drive a rendered BillieBot around an empty odom frame and read its telemetry. Save the layout.

### Mock Rung 03: EKF

```bash
container$ ros2 launch billiebot_bringup 03_ekf.launch.py mock:=true
```

**New skill:** multi-series comparison plots.

1. Topics: `/odometry/filtered` (30 Hz) joins the party — `robot_localization`'s EKF output (currently fusing wheel odometry only; the IMU is disabled stack-wide).
2. In your Plot panel, add `/odometry/filtered.pose.pose.position.x` alongside `/odom.pose.pose.position.x`. Drive with Teleop. The traces should track almost perfectly — with a single input, the EKF is nearly a passthrough. The value of this plot is on the real robot (Rung 03, Part 2), where you'll compare them after real wheel slip; build the panel now so it's waiting.
3. Raw Messages on `/odometry/filtered`: note the populated covariance versus `/odom`'s.

### Mock Rung 04: SLAM

```bash
container$ ros2 launch billiebot_bringup 04_slam.launch.py mock:=true
```

**New skills:** OccupancyGrid display, the `map` frame, calling a service (map saving). *(First half of "How do I build a map with SLAM and save it?" — the real version is in Part 2.)*

> VERIFICATION.md says rung 04 "requires real lidar data" — that note predates the mock scan publisher (`mock_scan.py`); mock SLAM now runs. What it *builds* is another matter (see the caveat).

1. Topics: `/map` (`nav_msgs/OccupancyGrid`) and `/map_metadata` appear; slam_toolbox also starts broadcasting the `map → odom` transform.
2. 3D panel: set **Display frame** to `map` — from now on this is your default. Toggle `/map` visible. Within ~5–10 s (slam_toolbox's `map_update_interval: 5.0`) a small occupancy grid materializes: white free space, black walls — the mock rectangle, mapped.
3. Note `/map`'s rate in the Topics sidebar: ~0.2 Hz, and it can read 0 between updates. Latched/slow topics are normal — the 3D panel keeps rendering the last message.

> **Caveat — do not Teleop-drive while mock-mapping.** The mock scan is pose-independent: SLAM's scan matcher sees a room that moves with the robot while odometry says the robot is translating, and the map corrupts into smeared rectangles. In mock, map with the robot parked. (Driving *while mapping* is precisely what you *will* do on the real robot.)

4. **Save the map — add a Call service panel.** Select service `/slam_toolbox/save_map` (type `slam_toolbox/srv/SaveMap`), request:
   ```json
   { "name": { "data": "mock_room" } }
   ```
   Click **Call**. `mock_room.pgm` + `mock_room.yaml` land in the launch shell's working directory (i.e. `/ws` in the container). The CLI equivalent — which you'll prefer on the real robot — is `ros2 run nav2_map_server map_saver_cli -f mock_room`.

**Success:** a mapped rectangle in the `map` frame and a saved `.pgm`/`.yaml` pair you made without leaving Foxglove.

### Mock Rung 05: AMCL Localization

```bash
container$ ros2 launch billiebot_bringup 05_amcl.launch.py mock:=true \
    map:=/ws/src/billiebot_navigation/maps/my_apartment_v1.yaml
```

(VERIFICATION.md's rung 05 line omits `mock:=true` — on the Mac you need it, or the launch tries to open the real lidar and Arduino serial ports. The repo ships an apartment map to localize against.)

**New skills:** setting the initial pose from the 3D panel, latched topics, covariance visualization. *(This answers "How do I set the pose?")*

1. Topics: `/map` (now served by `map_server` from the YAML — the shipped apartment, not a rectangle), `/amcl_pose`, `/particle_cloud`. The 3D panel shows the apartment floor plan.
2. AMCL auto-initializes at the map origin (`set_initial_pose: true`, pose 0,0,0 in `amcl_params.yaml`) — you'll see the robot model sitting at (0, 0).
3. **Set the pose:** in the 3D panel toolbar find the **Publish** tool and set its type to **Pose estimate** (the dropdown next to the tool button; the output topic is configured in panel settings → **Publish** → pose estimate topic = `/initialpose`, the default). Click where the robot "is" on the map and **drag to set heading** before releasing — identical muscle memory to RViz's *2D Pose Estimate*.
4. Watch three things react: Raw Messages on `/amcl_pose` jumps to your pose; the robot model teleports (the `map → odom` transform snapped); `/particle_cloud` republishes.
5. **Covariance:** enable `/amcl_pose` in the 3D panel topics — Foxglove draws `PoseWithCovarianceStamped` with its covariance ellipse. Add a Plot series on `/amcl_pose.pose.covariance[0]` (x-variance) — your localization-health signal for Part 2.

> **Two honest notes.** (1) The particle cloud: Humble's AMCL publishes `nav2_msgs/ParticleCloud`, which the Foxglove 3D panel doesn't render natively (RViz needs a dedicated plugin for it too). Inspect it with Raw Messages (`/particle_cloud.particles` length between 500 and 2000); the covariance ellipse on `/amcl_pose` conveys the same health information visually. (2) Convergence: the mock scan is a rectangle and the map is an apartment — AMCL can't meaningfully converge no matter how much you drive. The exercise is the tooling; the payoff is Part 2.

### Mock Rung 06: Nav2

```bash
container$ ros2 launch billiebot_bringup 06_nav2.launch.py mock:=true \
    map:=/ws/src/billiebot_navigation/maps/my_apartment_v1.yaml
```

**New skills:** costmaps, path display, sending a navigation goal from the 3D panel, the Log panel. *(This answers "How do I set the pose goal?")*

1. Give Nav2 ~15 s to bring its lifecycle nodes up. New topics: `/global_costmap/costmap`, `/local_costmap/costmap`, `/plan`.
2. **Costmaps:** in the 3D panel, toggle both costmaps visible. They're `OccupancyGrid`s like `/map`, drawn with inflation halos around obstacles (global = whole map at ~1 Hz; local = a rolling 3 × 3 m window in the `odom` frame at ~2 Hz, fed by `/scan` — watch it travel with the robot). If layers z-fight visually, most grid topics have a color-scheme/opacity setting in the panel — lower the costmap opacity so `/map` shows through.
3. Toggle `/plan` (`nav_msgs/Path`) visible — nothing yet; there's no goal.
4. **Send a goal:** set the 3D panel's **Publish** tool type to **Pose**, and in panel settings → **Publish**, set the pose topic to **`/goal_pose`** (Foxglove's default pose topic is a ROS 1 relic — change it once, it's saved in your layout). Click-and-drag a goal a couple of metres away in free space. Nav2's `bt_navigator` subscribes to `/goal_pose` and starts a `navigate_to_pose` action on your behalf — the same plumbing RViz's *Nav2 Goal* tool uses.
5. Watch: `/plan` draws the global path; `/cmd_vel` starts streaming (add it to a Plot); the robot follows the path; the local costmap rolls along underneath.
6. **Add a Log panel.** It shows `/rosout` — bt_navigator, planner, and controller narrate goal acceptance, progress, and failures here. When navigation misbehaves, this panel is where the truth is.

> **Set expectations in mock:** localization is fictional (rectangle scan vs. apartment map), so the robot may cruise cleanly, or AMCL may jump mid-run and trigger a re-plan or a spin/backup recovery behavior. Both are instructive — watch the Log panel narrate. Clean, repeatable navigation demos belong to Part 2.
>
> **Actions caveat:** `foxglove_bridge` exposes topics, services, and parameters — **not ROS 2 actions**. Publishing `/goal_pose` sidesteps that for navigation. For direct action calls (with feedback) use the CLI:
> ```bash
> container$ ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
>     "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0}, orientation: {w: 1.0}}}}"
> ```

### Mock Rung 07: OAK-D Dog Detector

```bash
container$ ros2 launch billiebot_bringup 07_oakd.launch.py mock:=true
```

**New skills:** custom message types over the bridge, Indicator panel.

1. Topics: `/dog/detections_3d` (`billiebot_interfaces/DogDetection3D`, ~5 Hz with gaps — mock emits a detection on ~70 % of ticks) and `/dog/found` (`std_msgs/Bool`).
2. Raw Messages on `/dog/detections_3d`: bounding box (`bbox_x/y/w/h`), `confidence` ≈ 0.85, `position` (camera optical frame — z is forward), `depth` ≈ 2.0 m, `label: "dog"`. Custom messages need zero configuration — the bridge ships the message definitions and every panel understands them.
3. **Add an Indicator panel** on `/dog/found.data`: rule `= true` → green, label "DOG FOUND"; default red, "NO DOG". It'll flicker with the mock's 70 % duty cycle.
4. Plot `/dog/detections_3d.depth` — a flat ~2 m line with dropouts.

> **Where are the camera images?** There are none, by design: `oakd_dog_detector` runs YOLO **on the camera** via the depthai SDK and publishes only detections — no RGB, stereo, or point-cloud topics exist in the BillieBot stack. To see what the OAK-D sees, use the standalone driver recipe in [Appendix B](#appendix-b-full-oak-d-lite-streams-rgb-stereo-depth-point-cloud).

### Mock Rung 08: Dog Locator

```bash
# Three shells (B, C, D) — this rung composes on top of others:
container$ ros2 launch billiebot_bringup 04_slam.launch.py mock:=true      # TF: map→odom→base_link→oakd_link_optical
container$ ros2 launch billiebot_bringup 07_oakd.launch.py mock:=true      # detections
container$ ros2 launch billiebot_bringup 08_dog_locator.launch.py          # this rung (no mock arg — it's pure math)
```

**New skill:** composing rungs; PoseStamped display.

VERIFICATION.md's "publishing when detections + TF available" is the lesson: `dog_locator` transforms optical-frame detections into the `map` frame, so it needs a full TF chain up to `map` (rung 04 or 05) *and* detections (rung 07). Run all three.

1. Topic: `/dog/pose_map` (`geometry_msgs/PoseStamped`, frame `map`).
2. 3D panel (display frame `map`): toggle `/dog/pose_map` visible — an arrow ~2 m in front of the robot's camera, flickering with the mock detection duty cycle. Set a small **Decay time** on it to smooth the flicker.
3. Teleop-rotate the robot in place: the dog pose orbits with the camera (the "dog" is always 2 m ahead of the lens — it's mock data being transformed, working exactly as designed).

### Mock Rung 09: Thermal Camera

```bash
container$ ros2 launch billiebot_bringup 09_thermal.launch.py mock:=true
```

**New skills:** Image panel, colormapping float images.

1. Topics: `/thermal/image` (`sensor_msgs/Image`, **32×24, encoding `32FC1`**, 4 Hz — each pixel is a temperature in °C) and `/thermal/blob` (`billiebot_interfaces/ThermalBlob`).
2. **Add an Image panel**, topic `/thermal/image`. It may render black or flat — a `32FC1` image has no inherent brightness mapping. In the panel settings, set the **color mode to a gradient/colormap** (e.g. Turbo) and the **value min/max to 20 / 40** (°C). Now you see it: ~22 °C ambient with a ~35 °C warm blob in the center — the mock "dog".
3. It's 32×24, so it renders as large soft pixels — that's the actual MLX90640 resolution, not a rendering problem.
4. Raw Messages on `/thermal/blob`: centroid (`cx`, `cy`), `area`, `max_temp`, `mean_temp`, `is_dog_candidate`. Plot `/thermal/blob.max_temp` — flat ~35 °C in mock; on the real sensor this becomes your presence signal.

### Mock Rung 10: NoIR Camera

```bash
container$ ros2 launch billiebot_bringup 10_noir.launch.py mock:=true
```

**New skill:** viewing a streaming camera and verifying its frame rate. *(This answers "How do I view camera images at ~5 Hz?")*

1. Topic: `/noir/image` — `sensor_msgs/Image`, 640×480 `rgb8`, **5 Hz**.
2. Image panel on `/noir/image`. In mock it's a flat dark-grey frame (the mock publishes constant `[40,40,40]` pixels) — visually boring, but the pipeline, encoding, and cadence are all real.
3. **Verify the rate** in the Topics sidebar. The node is configured for 5 Hz, but in the Docker mock expect it to fall short (~1–3 Hz): the mock builds each 921 KB frame in pure Python, which is CPU-bound. Don't chase this — the real capture path is efficient, and Real Rung 10 on the Pi is the definitive 5 Hz check. Either way, a ≤5 Hz stream *looks* like a slideshow in the panel; that's the sensor config, not a Foxglove problem.
4. Bandwidth intuition: 640×480×3 bytes×5 Hz ≈ **4.6 MB/s** for this one raw topic. Trivial over localhost; consequential over the robot's Wi-Fi (see Real Rung 10). There is no compressed variant in the stack.

### Mock Rung 11: Audio

```bash
container$ ros2 launch billiebot_bringup 11_audio.launch.py mock:=true
```

**New skills:** State Transitions panel; visualizing non-visual data. *(This answers "Are there methods for viewing audio data?")*

1. Topic: `/audio/events` (`billiebot_interfaces/AudioEvent`, 2 Hz). Mock emits BARK ~15 % and WHINE ~5 % of the time, otherwise silence.
2. Raw Messages on `/audio/events`: `event_type`, `confidence`, `doa_deg` (direction of arrival, 0–360° from robot front), `yamnet_label`, `energy_db`.
3. **Add a State Transitions panel**, path `/audio/events.event_type`. It draws a timeline of value changes — bark/whine events appear as colored bands. Decoder ring: `0 BARK · 1 WHINE · 2 HOWL · 3 LOUD_NOISE · 4 SILENCE`.
4. **Gauge** on `/audio/events.doa_deg`, min 0, max 360 — the "which way was the bark" dial (random in mock; meaningful on the real mic array).
5. **Plot** `/audio/events.energy_db`.

> **The honest answer on audio:** Foxglove has **no audio playback panel** — you cannot *listen* through it. BillieBot also publishes no raw-audio topic (the mic feeds YAMNet in-process; only classified `AudioEvent`s hit ROS), so there'd be nothing to play anyway. Visualizing *derived* audio features — events, energy, DoA, labels — as you just did is the practical method, and it's genuinely more useful for a monitoring robot. If you ever add a raw PCM topic (e.g. `audio_common`), the closest Foxglove gets is an oscilloscope: a Plot panel on the sample array with `[:]` slicing. For actually listening, record a bag and play the audio back outside Foxglove.

### Mock Rung 12: Cognition

```bash
# Give state fusion something to fuse — run in parallel shells:
container$ ros2 launch billiebot_bringup 07_oakd.launch.py mock:=true
container$ ros2 launch billiebot_bringup 11_audio.launch.py mock:=true
container$ ros2 launch billiebot_bringup 12_cognition.launch.py            # (no mock arg)
```

**New skills:** the INSTALLATION_AND_SETUP §1.7 verify, State Transitions on the fused state, services with empty requests, the report server. *(This is "add a Raw Messages panel on `/billie/state` and watch mock updates arrive.")*

1. Topic: `/billie/state` (`billiebot_interfaces/DogState`, 2 Hz), fused from detections + thermal + audio over a 10 s sliding window with 3 s hysteresis.
2. **The §1.7 verify, done properly:** add a **Raw Messages** panel on `/billie/state` and watch updates arrive at 2 Hz: `state`, `confidence`, `position` (map frame), `room`, `context[6]`, `stress_proxy`, `state_duration`. This exact panel is the smoke test that the whole mock stack + bridge + Foxglove chain works.
3. **State Transitions panel** on `/billie/state.state` — the marquee use of this panel type. Decoder: `0 NOT_FOUND · 1 SLEEPING · 2 RESTING · 3 ACTIVE · 4 BARKING · 5 EATING`. With rungs 07 + 11 feeding it, watch it move between states as mock barks land; note the hysteresis (no state flapping faster than 3 s).
4. **Plot** `/billie/state.stress_proxy` and `/billie/state.confidence` together.
5. **Call service** panel: `/get_dog_state` (`billiebot_interfaces/srv/GetDogState`) with an empty request `{}` — the response echoes the current fused state.
6. Foxglove doesn't do HTTP, but your browser does: `http://localhost:8080/health` (VERIFICATION's check), then `http://localhost:8080/` for the latest daily report and `/reports` for the list (port 8080 is the other port the container publishes).

### Mock Rung 13: Mission

```bash
container$ ros2 launch billiebot_bringup 13_mission.launch.py mock:=true
```

(Richer with rung 12's shells still running — the mission controller subscribes to `/billie/state` and `/battery_state`.)

**New skill:** commanding the robot's brain from a dashboard.

1. Topic: `/billiebot/mission_status` (`billiebot_interfaces/MissionStatus`, ~2 Hz). Raw Messages: `mode`, `current_waypoint`, `dog_state`, `battery_voltage`, `nav_active`, `recovery_count`, `estopped`.
2. **State Transitions** on `/billiebot/mission_status.mode`. Decoder: `0 IDLE · 1 PATROL · 2 INVESTIGATE · 3 TRACK_OBSERVE · 4 RETURN · 5 SAFE`.
3. **Call service** `/set_mode` (`billiebot_interfaces/srv/SetMode`), request `{ "mode": 1 }` → response `success: true`, and your State Transitions band flips IDLE → PATROL. Set it back with `{ "mode": 0 }`. You are now operating the robot from Foxglove.
4. The mission action servers (`/approach_dog`, `/retreat`, `/dispense_treat`) are actions — invisible to the bridge (same caveat as Rung 06). Verify them from Shell B: `ros2 action list`, and exercise via `ros2 action send_goal`.

### Mock Rung 14: Full Bringup

```bash
container$ ros2 launch billiebot_bringup 14_full_bringup.launch.py mock:=true \
    map:=/ws/src/billiebot_navigation/maps/my_apartment_v1.yaml
```

**New skill:** assembling the mission-control layout.

One command, the whole stack (rung 14 transitively includes 06 → 05 → 01/03 → 02, plus 07–13). The Topics sidebar should list every topic you've met. Now consolidate everything you built into one dashboard — a suggested arrangement:

```
+--------------------------------------+---------------------+
|                                      | Image: /thermal     |
|   3D panel (display frame: map)      |  (Turbo, 20–40 °C)  |
|   robot + /scan + /map + costmaps    +---------------------+
|   + /plan + /amcl_pose + /dog/pose_map | Image: /noir      |
+------------------+-------------------+---------------------+
| State Trans:     | Plot: stress_proxy| Raw: /billie/state  |
| billie/state.state| + battery voltage|                     |
| mission…mode     |                   |                     |
+---------+--------+-------------------+---------------------+
| Teleop  | Gauge: doa_deg | Indicator: | Log (rosout)       |
| /cmd_vel| Gauge: battery | /dog/found |                    |
+---------+----------------+------------+--------------------+
```

Exercise the full loop from the dashboard: set a `/goal_pose`, watch `/plan` + costmaps + mission status respond, flip `/set_mode`, watch `/billie/state` transition as mock detections arrive. Then **export the layout** (Layouts menu → export as JSON) and keep it under version control — see [Appendix A.2](#a2-layouts-save-share-commit). Duplicate it as **`BillieBot Robot`** for Part 2.

---

## Part 2: The bringup ladder on the real robot

Same ladder, real hardware. This part assumes every skill from Part 1 and covers only what changes: where things run, where the bridge lives, safety, and what *real* data should look like.

### 2.0 Where everything runs, and where the bridge goes

BillieBot is multi-machine (see INSTALLATION_AND_SETUP.md §2):

| Machine | IP (shipped) | Hardware attached | Rungs launched here |
|---|---|---|---|
| Jetson Orin Nano | `192.168.1.100` | RPLidar A1, Arduino/base, OAK-D Lite | 01–08, 13 |
| Raspberry Pi | `192.168.1.101` | MLX90640, NoIR cam, ReSpeaker, speaker | 09–12 |
| Your Mac | (any, on robot Wi-Fi) | — | Foxglove only |

**Bridge strategy:** run `foxglove_bridge` on the machine whose rung you're testing, and connect Foxglove to that machine:

```bash
jetson$ ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765   # rungs 01–08, 13
pi$     ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765   # rungs 09–12
```

Foxglove → Open connection → `ws://192.168.1.100:8765` (Jetson) or `ws://192.168.1.101:8765` (Pi). Your Mac needs only WebSocket reachability to that IP over the robot's Wi-Fi (GL-SFT1200 router) — it does **not** need to be a DDS participant. That's the whole point of the bridge.

Once both machines are up with CycloneDDS peers configured (`billiebot_bringup/config/cyclonedds.xml`, multicast off, static peers; remember `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` on both boards per §2.1), **one bridge on the Jetson sees the Pi's topics too** — from then on, connect only to the Jetson and drive everything from one dashboard.

Drop `mock:=true` from every command below. Where VERIFICATION.md lists a `verify_rung_*.sh` script, still run it — Foxglove complements the scripts, it doesn't replace them.

### Real Rung 01: Lidar

```bash
jetson$ ros2 launch billiebot_bringup 01_lidar.launch.py
```

Same panels as Mock Rung 01 (`ws://192.168.1.100:8765` now). What's different — and what to check:

1. The rectangle is gone; you see **your actual room**, walls, furniture and all. Rotate to top-down 2D view.
2. **Rate check** (Topics sidebar): the RPLidar A1 in `Standard` mode spins slower than the 10 Hz mock — expect roughly 5–8 Hz, steady. A sagging or jittery rate usually means USB power or serial contention.
3. **Sanity checks:** stand in front of the robot — your legs appear as an arc at the right bearing and distance (this confirms mounting orientation and `laser_frame`). Look for `inf`/dropout sectors in Raw Messages: black/absorbent surfaces and glass legitimately eat returns; a *fixed* dead sector at all times is the robot's own frame occluding the lidar.
4. Try **Decay time** ≈ 5 s and walk around the robot — you'll paint a ghost trail through the scan. Set it back to 0.

### Real Rung 02: Base + Description

```bash
jetson$ ros2 launch billiebot_bringup 02_base.launch.py
```

Your Teleop panel now moves a physical robot. Do this in order:

1. **Wheels-off-ground first.** Robot on a block. Teleop forward: wheels spin the right direction, `/joint_states` animates the wheel joints in the 3D panel, `/odom` integrates. Verify releasing the Teleop button stops the wheels within ~0.5 s — that's `base_bridge`'s `cmd_timeout_sec` doing its job (the Arduino firmware has its own independent 500 ms auto-stop watchdog; you're verifying the software layer of the same contract).
2. **Wire up the e-stop before the robot touches the floor.** Call service panel → `/e_stop` (`billiebot_interfaces/srv/EStop`), request `{ "engage": true }`. Keep this panel top-right in your layout for the rest of the robot's life. Engage it, confirm Teleop does nothing, release with `{ "engage": false }`.
3. **On the floor:** drive gently (≤0.2 m/s). **Odometry calibration check:** Plot `/odom.pose.pose.position.x`, drive dead-ahead exactly 1.0 m by tape measure, read the plot. Off by more than a few percent → wheel radius/separation/ticks-per-rev in `billiebot_base/config/base_driver.yaml` need attention. Spin 360° by eye and check yaw returns to start.
4. Battery Gauge now reads truth. Note the resting voltage; watch it sag under drive load. (Mission-level low-battery SAFE behavior gets verified at Rung 13, not here.)

### Real Rung 03: EKF

```bash
jetson$ ros2 launch billiebot_bringup 03_ekf.launch.py
```

The comparison plot you pre-built in Part 1 earns its keep: drive a ~1 m square by Teleop, watch `/odom` vs `/odometry/filtered` x/y. With only wheel odometry fused they'll stay close; induce wheel slip (drive against a wall edge, carpet transition) and watch both drift *together* — that shared drift is exactly what SLAM/AMCL will correct from rung 04 on, and why the map frame exists.

### Real Rung 04: SLAM — build and save your apartment map

```bash
jetson$ ros2 launch billiebot_bringup 04_slam.launch.py
```

*(The second half of "How do I build a map using SLAM and then save it?" This is the payoff run.)*

1. 3D panel: display frame `map`, visible: `/map`, `/scan`, robot model, TF off (clutter). Top-down 2D view.
2. **Drive the map into existence.** Teleop at ~0.15–0.2 m/s. slam_toolbox only integrates new scans after ~0.5 m of travel or ~0.5 rad of turn (`minimum_travel_distance/heading`), and redraws `/map` every ~5 s — so drive, pause, watch the map catch up, repeat. Hug walls loosely, cover each room, avoid long straight sprints through open space.
3. **Watch scan-vs-map registration:** with both `/scan` and `/map` visible, the live scan should lie *on top of* the drawn walls. Scans peeling away from walls = drift accumulating; slow down, revisit somewhere known.
4. **Loop closure, live:** finish your circuit back where you started. When slam_toolbox recognizes the revisit, the whole map visibly *snaps* into better alignment — one of the most satisfying things you'll ever watch in Foxglove.
5. **Save the map, twice** (belt and suspenders, from a Jetson shell):
   ```bash
   jetson$ ros2 run nav2_map_server map_saver_cli -f ~/maps/my_apartment_v2       # portable .pgm + .yaml → for AMCL/Nav2
   jetson$ ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
       "{filename: '/home/jetson/maps/my_apartment_v2'}"                          # pose-graph → resume mapping later
   ```
   (Or the Call service panel on `/slam_toolbox/save_map` as in Part 1 — but the CLI lets you control the output directory.) Inspect the `.pgm`: crisp walls, no doubled/ghost walls. Ghosting → remap that area with slower turns.
6. **Install the map** where the launch files expect maps: copy the `.yaml` + `.pgm` into `billiebot_ws/src/billiebot_navigation/maps/`, rebuild (`colcon build --symlink-install`), and use it via `map:=` below (absolute path; the `.yaml`'s `image:` line references the `.pgm` relatively — keep them together).

### Real Rung 05: AMCL Localization

```bash
jetson$ ros2 launch billiebot_bringup 05_amcl.launch.py map:=/path/to/my_apartment_v2.yaml
```

Same panels as Mock Rung 05, now with signal in the loop:

1. AMCL self-initializes at the map origin — almost certainly *not* where the robot physically is. **Publish pose estimate** at the robot's true position and heading (your Part 1 muscle memory).
2. Drive a slow loop. Now that scans match the map, watch the **covariance ellipse on `/amcl_pose` shrink** as AMCL converges — and your Plot of `/amcl_pose.pose.covariance[0]` trend down. This plot is your go/no-go for autonomous navigation: don't send Nav2 goals while covariance is still fat.
3. **Kidnapped-robot drill:** pick the robot up, carry it two rooms over, put it down. Watch localization break (scan peels off the map, covariance balloons) — then rescue it with a fresh pose estimate. Knowing what *lost* looks like in the dashboard is the skill.

### Real Rung 06: Nav2 — first autonomous motion

```bash
jetson$ ros2 launch billiebot_bringup 06_nav2.launch.py map:=/path/to/my_apartment_v2.yaml
```

Pre-flight, non-negotiable: floor clear of clutter, e-stop Call-service panel armed and *tested this session*, localization converged (Rung 05), speeds capped in `nav2_params.yaml` (0.3 m/s — leave it).

1. Layout: 3D panel with `/map`, both costmaps (lower their opacity), `/plan`, `/scan`, robot; Log panel visible; Plot with `/cmd_vel.linear.x`.
2. **Send a real `/goal_pose`** (Publish-Pose tool) to a spot 2–3 m away with a clear line. Watch the sequence you know from mock — plan drawn, robot rolls, local costmap slides along — now with real obstacle inflation blooming around real furniture.
3. **Dynamic obstacle test:** step into the robot's path. Your legs appear in `/scan` → the local costmap inflates around them → DWB veers or stops, and on a blocked-long-enough path the recovery behaviors (spin/backup/wait) fire while the Log panel narrates. Step aside; it resumes.
4. Goal tolerance is 0.25 m (`xy_goal_tolerance`) — the robot stops *near* the arrow, not on it. That's configured, not broken.
5. For scripted/repeatable goals (with live feedback), use the action CLI from Rung 06 Part 1 on the Jetson; monitor in Foxglove.

### Real Rung 07: OAK-D Dog Detector

```bash
jetson$ ros2 launch billiebot_bringup 07_oakd.launch.py
```

1. Same panels as Mock Rung 07. Present a dog to the camera — a cooperative real dog, or a printed dog photo / stuffed dog for bench work (YOLO is not picky at conf ≈ 0.5; check `confidence` in Raw Messages).
2. Your Indicator flips DOG FOUND on real detections now; `/dog/detections_3d.depth` in the Plot should track tape-measure distance to the target within ~±10 % in the 1–4 m band. Walk the target left/right and watch `bbox_x` sweep.
3. Remember: still no image topics from this node (on-camera inference). To *see* the camera view while tuning detection, park this node and use [Appendix B](#appendix-b-full-oak-d-lite-streams-rgb-stereo-depth-point-cloud) — one process owns the OAK-D at a time.

### Real Rung 08: Dog Locator

```bash
# Jetson, three shells (or just proceed to rung 14 later):
jetson$ ros2 launch billiebot_bringup 05_amcl.launch.py map:=/path/to/my_apartment_v2.yaml
jetson$ ros2 launch billiebot_bringup 07_oakd.launch.py
jetson$ ros2 launch billiebot_bringup 08_dog_locator.launch.py
```

With real localization + real detections: `/dog/pose_map` is now an arrow at the dog's *actual position on your apartment map*. Walk the dog (or carry the stuffed one) around the room and watch the arrow track across the floor plan. This is the first genuinely magical dashboard moment — the whole perception-TF-localization chain, visible in one glyph.

### Real Rung 09: Thermal Camera (Pi)

```bash
pi$ ros2 launch billiebot_bringup 09_thermal.launch.py
```

Bridge on the Pi for rungs 09–12 if the Jetson stack is down (`ws://192.168.1.101:8765`) — or bring both machines up (§2.0) and keep using the Jetson bridge.

1. Same Image panel (Turbo, 20–40 °C). Wave your hand in front of the MLX90640: a ~33–36 °C blob tracks your hand at 4 Hz across the 32×24 grid.
2. Tune the colormap range to your room: min = ambient − 2 °C, max ≈ 40 °C gives the best contrast for mammal-hunting.
3. Plot `/thermal/blob.max_temp` and watch `is_dog_candidate` in Raw Messages as a warm body enters and leaves frame.

### Real Rung 10: NoIR Camera (Pi)

```bash
pi$ ros2 launch billiebot_bringup 10_noir.launch.py
```

1. Image panel on `/noir/image`: real frames at ~5 Hz — confirm the cadence in the Topics sidebar, and expect slideshow-smoothness; that's the sensor config, not lag.
2. **The Wi-Fi caveat from Part 1 is now real:** this raw topic costs ~4.6 MB/s from Pi → bridge → Mac. On a strong LAN it's fine; if frames stall while everything else flows, this topic is saturating the link — close its Image panel when you don't need it (Foxglove only subscribes to topics that panels use, so closing the panel actually frees the bandwidth).
3. NoIR party trick: kill the room lights and illuminate with an IR source — the NoIR (no IR-cut filter) keeps seeing. That's its night-monitoring job.

### Real Rung 11: Audio (Pi)

```bash
pi$ ros2 launch billiebot_bringup 11_audio.launch.py
```

1. Same panels as Mock Rung 11. Make noise: clap (LOUD_NOISE), play a dog-bark video off your phone (BARK — YAMNet is very good at this), speak (check `yamnet_label` in Raw Messages for what YAMNet actually heard).
2. **DoA is the star:** walk around the robot playing barks and watch the `doa_deg` Gauge track your bearing (0° = robot front, counterclockwise). This validates mic-array orientation — if DoA reads mirrored, the array is mounted rotated.
3. Watch `energy_db` in the Plot spike with each event, and confirm quiet-room baseline vs. event separation — that margin is what the classifier lives on.

### Real Rung 12: Cognition (Pi + Jetson)

```bash
pi$ ros2 launch billiebot_bringup 12_cognition.launch.py
```

For real fused state you want the Jetson perception rungs (07/08) up too — this is your first true multi-machine session: both boards launched with CycloneDDS peers + `RMW_IMPLEMENTATION` set, one bridge on the Jetson, and the Topics sidebar showing topics from *both* machines. (If Pi topics are missing from the Jetson bridge, it's DDS peering, not Foxglove — Appendix C.)

1. The §1.7 Raw Messages panel on `/billie/state`, now with a real dog in it: position in map coordinates, `room` naming the actual room (per `rooms.yaml`), states driven by real detections and barks.
2. State Transitions on `/billie/state.state` over a 10-minute observation of the dog is the single best "is this robot working" artifact the project produces. Screenshot it.
3. Daily report: `http://192.168.1.101:8080/` in your browser.

### Real Rung 13: Mission (Jetson)

```bash
jetson$ ros2 launch billiebot_bringup 13_mission.launch.py
```

With cognition + nav running: `/set_mode` → `{ "mode": 1 }` (PATROL) from the Call service panel, hands hovering over the e-stop panel. Watch `/billiebot/mission_status` in State Transitions, `nav_active` and `recovery_count` in Raw Messages, and the robot on the map in the 3D panel. Run the e-stop drill once while it's moving: `{ "engage": true }` → verify `estopped: true` in mission status *and* physical stop → release.

### Real Rung 14: Full system

On the real robot, "rung 14" is the multi-machine bringup (VERIFICATION.md §Multi-Machine Setup), not the single-machine `14_full_bringup.launch.py` (which assumes all hardware on one host — bench use only):

```bash
jetson$ ros2 launch billiebot_bringup jetson.launch.py map:=/path/to/my_apartment_v2.yaml
pi$     ros2 launch billiebot_bringup pi.launch.py
jetson$ ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

Connect to `ws://192.168.1.100:8765`, load the **`BillieBot Robot`** layout — every panel you built now shows the live robot. This layout *is* the operator console from the system design (Host Computer → Foxglove via router). Leave it running during a patrol and watch the day's story: map position, dog state bands, stress plot, thermal blobs, mission mode.

---

## FAQ: where each question is answered

| Question | Where |
|---|---|
| How do I view the robot? | [Mock Rung 02](#mock-rung-02-base--description) (URDF from `/robot_description` + TF in the 3D panel) |
| How do I view real lidar data? | [Mock Rung 01](#mock-rung-01-lidar) for mechanics, [Real Rung 01](#real-rung-01-lidar) for real-data checks |
| How do I view camera images streaming at ~5 Hz? | [Mock](#mock-rung-10-noir-camera) / [Real Rung 10](#real-rung-10-noir-camera-pi) (`/noir/image`), plus [Rung 09](#mock-rung-09-thermal-camera) for float-image colormaps |
| How do I view a loaded map? | [Mock Rung 05](#mock-rung-05-amcl-localization) (`map_server` + latched `/map`); built live in [Rung 04](#mock-rung-04-slam) |
| How do I set the pose? | [Mock Rung 05](#mock-rung-05-amcl-localization), step 3 (Publish → Pose estimate → `/initialpose`) |
| How do I set the pose goal? | [Mock Rung 06](#mock-rung-06-nav2), step 4 (Publish → Pose → `/goal_pose`; actions caveat there too) |
| The §1.7 Raw Messages check on `/billie/state`? | [Mock Rung 12](#mock-rung-12-cognition), step 2 |
| OAK-D Lite stereo images / point cloud? | [Rung 07](#mock-rung-07-oak-d-dog-detector) for what the stack publishes (detections only); [Appendix B](#appendix-b-full-oak-d-lite-streams-rgb-stereo-depth-point-cloud) for full streams |
| Viewing audio data? | [Mock](#mock-rung-11-audio) / [Real Rung 11](#real-rung-11-audio-pi) (and the honest-answer callout there) |
| Building and saving a SLAM map? | [Mock Rung 04](#mock-rung-04-slam) (mechanics) → [Real Rung 04](#real-rung-04-slam--build-and-save-your-apartment-map) (the real workflow) |

---

## Appendix A: Beyond the bringup ladder

### A.1 Record and replay (the debugging superpower)

Foxglove's killer feature over RViz isn't live viewing — it's that **the same panels work on recorded data**, with a scrub bar.

```bash
container$ apt-get install -y ros-humble-rosbag2-storage-mcap    # once (if not present)
container$ ros2 bag record -s mcap -a -o /ws/bags/billie_run1    # record everything
```

Reproduce a behavior (a failed nav goal, a state-fusion misfire), Ctrl-C the recording, then in Foxglove: **Open local file…** → the `.mcap` file (on the Mac: copy it out, or record into the mounted `billiebot_ws` so it's already on the host). Now scrub, pause at the failure, step message-by-message, and every panel — 3D, plots, state transitions, images — replays in sync. Your layouts work unchanged on recorded data. On the real robot, record on the Jetson and copy files off afterward; that also sidesteps all Wi-Fi bandwidth limits since nothing streams live.

This is the answer to "how do I debug something that happened at 3 am": leave a bag recording during overnight patrols.

### A.2 Layouts: save, share, commit

Layouts menu → **Export layout to file…** produces a JSON file; **Import** restores it on any machine. Suggested convention for this repo: commit them under `docs/foxglove_layouts/` (e.g. `billiebot_mock.json`, `billiebot_robot.json`) so a fresh machine gets the full operator console with one import. Layouts store panel arrangement *and* all settings — colormap ranges, publish topics, gauge bounds — so the `/goal_pose` fix from Rung 06 travels with the file.

### A.3 User Scripts: derived signals without writing a node

The **User Scripts** panel runs TypeScript on live messages and publishes the result as a virtual topic — for quick derived metrics that don't deserve a ROS node. Classic example, "distance to nearest obstacle":

```ts
import { Input } from "./types";

export const inputs = ["/scan"];
export const output = "/studio_script/nearest_obstacle";

export default function script(event: Input<"/scan">): { range_m: number } {
  const valid = event.message.ranges.filter((r) => Number.isFinite(r) && r > 0.05);
  return { range_m: valid.length ? Math.min(...valid) : NaN };
}
```

Plot `/studio_script/nearest_obstacle.range_m` and you have a proximity alarm strip-chart. Other easy wins: bark-rate-per-minute from `/audio/events`, battery percentage from voltage, XY distance robot↔dog from `/amcl_pose` + `/dog/pose_map`.

### A.4 Parameters panel

`foxglove_bridge` exposes the parameters capability, so the **Parameters** panel can read — and live-edit — node parameters. Useful: eyeball `amcl` particle counts, flip `base_bridge`'s `cmd_timeout_sec` during watchdog experiments, or check what `slam_toolbox` actually loaded. Edits are live and unsaved (they don't persist to YAML) — treat it as a tuning scratchpad, then write the winner back into the config file.

### A.5 Publish panel: arbitrary messages by hand

The **Publish** panel sends any message to any topic — the general tool behind the 3D panel's pose buttons. Uses: hand-craft an `/initialpose` with custom covariance; send a single `geometry_msgs/Twist` to `/cmd_vel` (all zeros = a poor-man's software stop); poke `/goal_pose` numerically for repeatable test goals.

### A.6 Things Foxglove will not do (so you stop looking)

- **Call ROS 2 actions** — bridge limitation; use `ros2 action send_goal` (Rungs 06/13).
- **Play audio** — no playback panel (Rung 11 callout).
- **Serve as a DDS participant** — it only ever talks to a bridge; if a topic isn't in the bridge's graph, Foxglove can't see it.
- **Persist parameter edits** — A.4 caveat.

---

## Appendix B: Full OAK-D Lite streams (RGB, stereo depth, point cloud)

The BillieBot stack deliberately never publishes OAK-D imagery — `oakd_dog_detector` runs YOLO on-camera and ships only `DogDetection3D`. When you *do* want to see through the camera (mount alignment, focus/exposure sanity, depth-quality assessment), run Luxonis's standalone ROS driver instead. **The OAK-D is single-owner: stop rung 07 first** (`Ctrl-C`), then:

```bash
jetson$ sudo apt-get install -y ros-humble-depthai-ros            # once
jetson$ ros2 launch depthai_ros_driver rgbd_pcl.launch.py         # RGB + depth + point cloud
```

In Foxglove (bridge on the Jetson as usual):

1. **RGB:** Image panel on `/oak/rgb/image_raw`.
2. **Stereo depth:** Image panel on `/oak/stereo/image_raw` — a 16-bit depth image; as with the thermal camera, give it a colormap and set value min/max to your scene's depth band (e.g. 300–5000 mm). Nearby surfaces should be smooth gradients; speckle and holes on textureless/black surfaces are normal stereo physics.
3. **Point cloud:** 3D panel → toggle `/oak/points` (`sensor_msgs/PointCloud2`). Set the 3D panel's display frame to the driver's camera frame (autocomplete offers it — the driver publishes its own TF, unconnected to BillieBot's URDF unless you bridge them). Color by RGB, orbit your desk in 3D.
4. Mind USB3 bandwidth on shared hubs, and Wi-Fi bandwidth if you stream the cloud to the Mac — point clouds are the heaviest thing in this guide. Prefer bench Ethernet or a recorded bag (A.1).

When you're done sightseeing, Ctrl-C the driver and relaunch rung 07 — detections resume.

> If the project later wants a permanent low-rate RGB preview topic from within `oakd_dog_detector` (the design docs sketch a `/oak/rgb/preview`), that's a code change to the detector node, not a Foxglove setting.

---

## Appendix C: Troubleshooting

| Symptom | Likely cause → fix |
|---|---|
| Connection refused / spinner on `ws://localhost:8765` | Bridge not running in the container, or container started without `-p 8765:8765` → check Shell A; `docker ps` to confirm the port mapping. |
| Connected, but the topic list is empty | No rung launched (bridge alone publishes nothing), or the rung shell forgot to `source /ws/install/setup.bash`. |
| Topic listed but its panel shows nothing | Check the sidebar **Problems** tab; for 3D topics, 9 times out of 10 it's the **display frame** (no TF path from the message's frame to the display frame — e.g. `map` selected before any rung publishes `map → odom`). |
| `/map` shows 0 Hz and you're worried | It's latched (transient_local) and republish is slow (SLAM ~0.2 Hz) — the panel renders the last message; 0 Hz between updates is normal. |
| `/thermal/image` renders black/blank | It's `32FC1` — set the Image panel's colormap and value min/max (Rung 09). Same treatment for OAK depth (`16UC1`, Appendix B). |
| Nav goal does nothing | The 3D panel's Publish-Pose topic is still the default, not `/goal_pose` (Rung 06 step 4); or Nav2 lifecycle isn't active — check the Log panel; or no map was passed (`map:=`) so the planner never activated. |
| Can't call `/approach_dog`, `/navigate_to_pose`… from Foxglove | They're actions; the bridge doesn't expose actions → CLI (Rung 06/13 callouts). |
| Images stutter over Wi-Fi but small topics flow | Bandwidth (raw `rgb8` ≈ 4.6 MB/s; point clouds worse) → close unused Image panels (closing actually unsubscribes), move to Ethernet, or record a bag and review offline (A.1). |
| `/noir/image` well below 5 Hz **in mock mode** | Known mock artifact: the synthetic frame is generated in pure Python and is CPU-bound in the container. The real Pi camera path is efficient — verify the true 5 Hz at Real Rung 10. |
| Jetson bridge doesn't show Pi topics | DDS peering, not Foxglove: check `cyclonedds.xml` peer IPs on **both** machines, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` exported in both launch shells (INSTALLATION §2). |
| Port 8765 already in use | Another container/app holds it (INSTALLATION Appendix C) → `docker ps`, or launch the bridge with a different `port:=` and matching URL. |
| Panel settings you set keep coming back wrong | You're editing a different layout than you think — check the active layout name; settings live in layouts. |

---

*Written for the BillieBot MVP stack (ROS 2 Humble). Companion docs: [VERIFICATION.md](VERIFICATION.md) (what must be true per rung), [INSTALLATION_AND_SETUP.md](INSTALLATION_AND_SETUP.md) (environment + §1.7 visualization setup), [BRINGUP_LADDER_ANALYSIS.md](BRINGUP_LADDER_ANALYSIS.md) (per-rung topic/type/rate deep dive).*
