# BillieBot — Discrepancy Resolution Plan

**Purpose:** A self-contained remediation guide for the 19 design-vs-implementation discrepancies (GAP-1…GAP-19) identified during the systems decomposition of 2026-07-04. Each gap gets a resolution sheet with exact file/symbol locations, a recommended fix, and closure-verification commands, so a future session or engineer can pick this document up cold and work through it.
**Companion documents:** `docs/MBSE_SYSTEM_DECOMPOSITION.md` (§5.4 gap register — the numbering used here is identical; §2.3 for the L2 requirement IDs cited below) · `docs/BRINGUP_LADDER_ANALYSIS.md` (per-rung runtime context, Appendix B defects) · `docs/VERIFICATION.md` (TC-01…22).
**Date:** 2026-07-04. All file:line references were verified against the repository on this date; re-verify line numbers if the files have since changed (symbol names are the stable anchor).

---

## 1. How to Use This Document

1. Work phases **A → B → C → D** (§3). Phase A is prerequisite to putting the robot on the floor; Phase B delivers the MVP's core promise; C and D can interleave after B.
2. Before starting a gap, read its sheet in §4 **and** re-check the cited code — a gap may have been partially closed since this was written.
3. After closing a gap, update its **Status** in the §2 summary table and in its sheet: `Open → In progress → Resolved (date, commit)` or `Waived (rationale)`.
4. Some gaps end in a **sponsor decision** rather than a code fix (GAP-1, GAP-3, and the illuminator question under GAP-2's sibling PROP-07). These are collected in §5.2 — get decisions early; they gate other work.
5. Every sheet's **Verify closure** section assumes the workspace is built (`colcon build --symlink-install; source install/setup.bash` from `billiebot_ws/`).

**Legend** — Severity: 🔴 Critical-safety · 🟠 Major-functional · 🟡 Minor-hygiene · ⚪ Doc-only. Disposition: **F** fix code · **P** parameter/config change · **M** update model/design doc · **H** hardware task. Effort: **S** ≤ 1 h · **M** half-day · **L** multi-day.

## 2. Summary Table

| GAP | One-line description | Sev | Disp | Phase | Effort | Requirements hit | Status |
|---|---|---|---|---|---|---|---|
| GAP-14 | Firmware watchdog still 2000 ms (design: 500 ms) | 🔴 | F | A | S | MOB-05, SYS-PLT-5 | Resolved (2026-07-11) |
| GAP-5 | `odom→base_link` TF broadcast by both base_bridge and EKF | 🟠 | P | A | S | NAV-07, SYS-NAV-2 | Resolved (2026-07-17) |
| GAP-6 | Second `ekf_filter_node` launched by rung 06 | 🟠 | F | A | S | NAV-07 | Resolved (2026-07-17) |
| GAP-4 | Nav2 consumes raw `/odom`; `/odometry/filtered` unused | 🟡 | P | A | S | NAV-08 | Open |
| GAP-8 | Mission never learns of e-stop (`_estopped` never set) | 🟠 | F | A | S | MSN-02 | Open |
| GAP-7 | Mission never dispatches Nav2 goals; failure counter dead | 🟠 | F | B | L | MSN-05, NAV-14, SYS-NAV-4/6, SYS-FND-1 | Open |
| GAP-10 | No patrol-waypoint executor; `patrol_waypoints.yaml` unloaded | 🟠 | F | B | M | MSN-06, MSN-14, SYS-NAV-6 | Open |
| GAP-15 | Audio-DoA response is a log line; no INVESTIGATE / re-sort | 🟠 | F | B | M | MSN-04, SYS-FND-2 | Open |
| GAP-13 | No near-dog speed restriction (≤ 0.15 m/s within 2 m) | 🟠 | F | B | M | NAV-12, SYS-NAV-5 | Open |
| GAP-11 | OAK-D `model_path` defaults to `''` — real detector silently inert | 🟠 | P | B | S | PER-01, SYS-PER-1 | Open |
| GAP-9 | Logger gaps: empty snapshots, hard-coded action, no `/events/last` | 🟠 | F | C | M | STL-12/13/14, RPT-02, SYS-EXT-3 | Open |
| GAP-12 | `/dog/found` has two publishers (and no false-on-loss from locator) | 🟡 | F | C | S | PER-04 | Open |
| GAP-3 | `BatteryStatus.msg` defined but never published | 🟡 | M | C | S | IFC-06 | Open |
| GAP-16 | Rung 01 mock launches a base_bridge stub — no mock `/scan` | 🟠 | F | D | S | NAV-04, PLT-06 | Open |
| GAP-17 | Speak action naming split (`/speak` vs `/mission/speak`) | 🟡 | F | D | S | AUD-06 | Open |
| GAP-1 | Mission is a Python state machine; designed BT is compiled but never run | 🟠 | F* | D | L | SYS-EXT-2, MSN-01/12, EXT-02 | Open (decision) |
| GAP-2 | IMU dormant — BNO055 blocked by A4/A5 encoder pin conflict | 🟠 | H | D | M | NAV-06 | Open (hardware) |
| GAP-18 | `/oak/rgb/preview` in design but not published | 🟡 | F/M | D | S | SYS-PLT-4 (minor) | Open |
| GAP-19 | Docs say Raspberry Pi 5, configs/README say Pi 4 | ⚪ | M | D | S | doc-only | Resolved (2026-07-12) |

\* GAP-1 disposition depends on a sponsor decision (see sheet).

## 3. Resolution Phases

### Phase A — Safety & correctness floor (before any hardware driving)

**GAP-14 → GAP-5 → GAP-6 → GAP-4 → GAP-8.** These five are all S-effort and remove the two classes of danger: motors that keep spinning after a control-path loss (GAP-14 — **resolved**, firmware watchdog now fires at 500 ms), and a TF/odometry stack that lies to the navigator (GAP-5/6 — **resolved 2026-07-17**, landed together: the EKF is now the sole `odom→base_link` owner and rung 06 starts exactly one `ekf_filter_node`; GAP-4 remains). GAP-8 completes the e-stop chain so the mission layer latches SAFE instead of continuing to sequence behaviors while the base is frozen. Order matters only in that GAP-5 and GAP-6 should land together (both touch who owns `odom→base_link`).

### Phase B — Close the autonomy loop (the MVP's core promise)

**GAP-7 → GAP-10 → GAP-15 → GAP-13 → GAP-11.** Today the robot can navigate when a human sends goals, and can classify the dog when it happens to see her — but nothing connects the two. GAP-7 (mission dispatches Nav2 goals and reacts to results) is the keystone; GAP-10 (waypoint config + executor) is its data source; GAP-15 (INVESTIGATE on bark DoA) and GAP-13 (near-dog slowdown) only become meaningful once GAP-7 exists. GAP-11 is independent but required before any real-hardware perception trial.

**Dependencies:** GAP-10 and GAP-15 build directly on GAP-7's goal-dispatch machinery — do GAP-7 first, and consider implementing GAP-10 inside the same work package. GAP-13's speed-filter mask needs `/dog/pose_map`, which already works.

### Phase C — Data & reporting integrity

**GAP-9 → GAP-12 → GAP-3.** These make the logged record trustworthy: real snapshots (GAP-9, which also unblocks report images per RPT-02), a single honest `/dog/found` signal (GAP-12), and one battery message contract (GAP-3). None block Phase B; do them before the 24 h acceptance soak (Build Plan Phase 6).

### Phase D — Testability, architecture & hygiene

**GAP-16 → GAP-17 → GAP-1 → GAP-2 → GAP-18 → GAP-19.** GAP-16 first: a real mock `/scan` publisher makes rungs 04/05/06 exercisable off-hardware, which de-risks everything above. GAP-1 (behavior-tree vs. state-machine decision) is deliberately late: the Phase B work can land in the existing Python controller either way, and the decision is cheaper once the required behaviors are known concretely. GAP-2 waits on the physical A4/A5 rewire (see `docs/MEASURE_ME.md`).

---

## 4. Per-Gap Resolution Sheets

All paths are relative to the repository root; `…/src/` abbreviates `billiebot_ws/src/`.

### Phase A sheets

#### GAP-14 — Firmware watchdog interval (500 ms)

**Status:** Resolved (2026-07-11, commit `850bf50`) · **Severity:** 🔴 Critical-safety · **Disposition:** F · **Effort:** S

- **What/Where:** `reference_my_bot/diff-drive-motor-controller/arduino-nano-firmware/ROSArduinoBridge/ROSArduinoBridge.ino:117` now reads `#define AUTO_STOP_INTERVAL 500`; the cutoff check is at line 344 (`if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL)`). `firmware/README.md`'s documented required change (500 ms) is now applied in the firmware source and matches.
- **Why it matters:** SYS-PLT-5 / MOB-05 require the Arduino to stop the motors within 500 ms of losing the Jetson serial heartbeat. Previously, a Jetson crash or USB disconnect could leave the robot driving blind for up to 2 s (~0.6 m at patrol speed).
- **Fix applied:**
  1. The flight firmware remained in place at `reference_my_bot/diff-drive-motor-controller/arduino-nano-firmware/ROSArduinoBridge/` rather than being copied into `firmware/` first — a minor deviation from the sheet's original "preferred" step, not a functional issue, since `firmware/README.md` still documents the change against the same file.
  2. Line 117 changed to `#define AUTO_STOP_INTERVAL 500`.
  3. Nano reflashed with the updated firmware.
- **Verify closure:** TC-29 bench test executed and passed — motors commanded via `/cmd_vel` at 10 Hz, then `base_bridge` killed (serial link dropped); motor stop measured at ≤ 500 ms. Normal driving confirmed unaffected (the 30 Hz `m` stream resets `lastMotorCommand` far faster than 500 ms).
- **Risks/notes:** None to software. Do not set below ~100 ms — serial jitter at 30 Hz command spacing (33 ms) plus retries needs headroom.

#### GAP-5 — Two broadcasters for `odom→base_link`

**Status:** Resolved (2026-07-17, PR `fix/gap-5-6-tf-single-owner`) · **Severity:** 🟠 Major-functional · **Disposition:** P · **Effort:** S

- **What/Where:** `…/src/billiebot_base/config/base_driver.yaml:28` — `publish_tf: true` (its own comment says "disable when EKF provides odom->base_link"); `…/src/billiebot_navigation/config/ekf.yaml:9` — `publish_tf: true`. The base broadcast happens in `base_bridge.py:368` (`publish_tf_odom`, gated by the `publish_tf` param declared at line 116).
- **Why it matters:** NAV-07. Two (with GAP-6, three) broadcasters fight over the same transform at 30 Hz. In mock they agree (masking the bug); on hardware the EKF-smoothed pose diverges from raw integration and TF consumers (AMCL, costmaps, dog_locator) see time-interleaved jumps.
- **Fix applied:** The EKF owns the transform whenever it runs (rungs 03+); `ekf.yaml`'s `publish_tf: true` is unchanged and now correct-by-ownership.
  1. `base_driver.yaml:28` — `publish_tf` default flipped to `false` (comment updated with the bench-override recipe).
  2. `…/src/billiebot_base/launch/base.launch.py` and `…/src/billiebot_bringup/launch/02_base.launch.py` — new `publish_tf` launch argument (default `false`), so rung-02-only bench work can restore the base broadcast with `ros2 launch billiebot_bringup 02_base.launch.py publish_tf:=true`.
  3. `base_bridge.py:158` — the `TransformBroadcaster` is now only created when `publish_tf` is true, so a disabled base_bridge no longer registers an idle `/tf` publisher (keeps the TC-24 publisher-count check meaningful).
- **Verify closure (executed 2026-07-17, mock stack in the `billiebot-dev` Docker container):** With rung 06 up (includes rung 03): `ros2 topic info /tf --verbose` lists `ekf_filter_node`, `amcl`, `robot_state_publisher`, `bt_navigator` — **no `base_bridge`**; `tf2_echo odom base_link` streams (EKF-owned). Rung 02 only, default: `publish_tf` = `False`, no `odom→base_link` (expected — no EKF at rung 02). Rung 02 only with `publish_tf:=true`: param `True`, `tf2_echo odom base_link` streams from base_bridge. `verify_rung_02.sh` passes in both rung-02 configurations.
- **Risks/notes:** Closed together with GAP-6 (same PR), as required for the "one owner" check to hold.

#### GAP-6 — EKF launched twice by rung 06

**Status:** Resolved (2026-07-17, PR `fix/gap-5-6-tf-single-owner`) · **Severity:** 🟠 Major-functional · **Disposition:** F · **Effort:** S

- **What/Where:** `…/src/billiebot_navigation/launch/navigation.launch.py:19-29` started an `ekf_filter_node` (loading `ekf.yaml`), but `…/src/billiebot_bringup/launch/06_nav2.launch.py:21-26` already includes `05_amcl.launch.py` → `03_ekf.launch.py`, which starts the same node. Result: two `ekf_filter_node` processes, duplicate `/odometry/filtered` publishers and TF broadcasts.
- **Why it matters:** NAV-07 (with GAP-5); also confusing diagnostics (`ros2 node list` shows the name collision).
- **Fix applied:** Deleted the EKF `Node(...)` block (and the unused `ekf.yaml` load) from `navigation.launch.py`, keeping it purely Nav2 as its name says; the ladder's `03_ekf` rung is the canonical EKF owner. Safe because every include of `navigation.launch.py` repo-wide (rung 06, rung 14 → 06, `jetson.launch.py` → 06) reaches it via `05_amcl` → `03_ekf`. A module docstring now warns standalone users to bring their own EKF (e.g. `ros2 launch billiebot_bringup 03_ekf.launch.py`).
- **Verify closure (executed 2026-07-17, mock stack in the `billiebot-dev` Docker container):** `ros2 launch billiebot_bringup 06_nav2.launch.py mock:=true` → `ros2 node list | grep -c ekf_filter_node` → **1**; `/odometry/filtered` present (`verify_rung_03.sh` passes) and `verify_rung_06.sh` passes (navigate_to_pose action + both costmaps up).
- **Risks/notes:** Standalone `navigation.launch.py` users must now bring their own EKF — documented in the launch file docstring.

#### GAP-4 — Nav2 uses raw `/odom` instead of `/odometry/filtered`

**Status:** Open · **Severity:** 🟡 Minor-hygiene (Major once IMU lands) · **Disposition:** P · **Effort:** S

- **What/Where:** `…/src/billiebot_navigation/config/nav2_params.yaml:8` — `bt_navigator: odom_topic: /odom`. The EKF publishes `/odometry/filtered` (ekf.yaml input `odom0: /odom`), which nothing consumes.
- **Why it matters:** NAV-08; design §5.2 wires Nav2 to the filtered estimate. With only wheel odometry fused, the two topics are nearly identical, so this is currently cosmetic — but the moment the BNO055 is enabled (GAP-2) Nav2 would be navigating on the *worse* estimate.
- **Recommended fix:** Change `nav2_params.yaml:8` to `odom_topic: /odometry/filtered`. Audit for other raw-`/odom` consumers in Nav2 config while there (the DWB critics use TF, not the topic, so this one line is expected to be the only change).
- **Verify closure:** Rung 06 up: `ros2 param get /bt_navigator odom_topic` → `/odometry/filtered`; send a `navigate_to_pose` goal and confirm normal driving.
- **Risks/notes:** None while EKF and raw odom agree. Do after GAP-6 so only one filtered publisher exists.

#### GAP-8 — Mission never learns of e-stop

**Status:** Open · **Severity:** 🟠 Major-functional · **Disposition:** F · **Effort:** S

- **What/Where:** `…/src/billiebot_mission/billiebot_mission/mission_controller.py` — `self._estopped = False` (line 64) is checked every tick (line 144: `if self._estopped: self._mode = Mode.SAFE`) but **no callback ever sets it**. The e-stop state lives privately in `base_bridge.py` (`self._estopped`, line 161, toggled by `estop_callback` line 293). `MissionStatus.estopped` (mission_controller.py:189) is therefore always `false`.
- **Why it matters:** MSN-02 / STM-01 t6: when the operator e-stops, the mission keeps sequencing PATROL/TRACK logic against a frozen base, and the operator's status display claims no e-stop is active.
- **Recommended fix:** Publish e-stop state from the owner: in `base_bridge.py`, add a latched (`TRANSIENT_LOCAL`) `std_msgs/Bool` publisher on a new topic `/e_stop_state`, published from `estop_callback` on every engage/release (and once at startup). In `mission_controller.py.__init__`, subscribe and set `self._estopped = msg.data`. This keeps `/e_stop` (the service) as the single command interface and adds a state topic — no message/interface changes needed.
  - Alternative (rejected): mission polls the service — services carry no state; would need a new query service and polling.
- **Verify closure:** Mock stack up (rungs 02 + 13): `ros2 service call /e_stop billiebot_interfaces/srv/EStop "{engage: true}"` → `ros2 topic echo /billiebot/mission_status --once` shows `estopped: true`, `mode: 5` (SAFE); release → mission stays SAFE until operator `/set_mode` (confirm this latch behavior is desired — today SAFE is sticky only until `/set_mode`, which matches STM-01 t13).
- **Risks/notes:** Update `docs/MBSE_SYSTEM_DECOMPOSITION.md` IBD-03 (add the `/e_stop_state` connector) when done.

### Phase B sheets

#### GAP-7 — Mission dispatches no Nav2 goals; failure counting dead

**Status:** Open · **Severity:** 🟠 Major-functional (keystone gap) · **Disposition:** F · **Effort:** L

- **What/Where:** `…/src/billiebot_mission/billiebot_mission/mission_controller.py` — the `ActionClient(self, NavigateToPose, 'navigate_to_pose')` is created (lines 95-97) but never used; `tick()` (141-173) only flips modes on `/dog/found`; `_nav_failure_count` (line 63) is checked against `max_nav_failures` (line 154) but never incremented; `_current_wp_idx` (line 62) is never advanced; `_nav_active` (line 98) is never set true.
- **Why it matters:** This is the difference between a robot and a sensor cart: SYS-NAV-6 (patrol), SYS-FND-1 (search), SYS-NAV-4 escalation (NAV-14), and the whole find-the-dog mission depend on it. TC-16 and TC-19 cannot pass end-to-end.
- **Recommended fix (incremental, all in `mission_controller.py`):**
  1. **Waypoint table:** load named poses (see GAP-10 — do together) into `{name: (x, y, yaw)}`.
  2. **Goal dispatch:** in `tick()`, when mode == PATROL and `not self._nav_active`, build `NavigateToPose.Goal()` (frame `map`) for `patrol_waypoints[self._current_wp_idx]`, `send_goal_async` with a result callback; set `_nav_active = True`.
  3. **Result handling:** on SUCCEEDED → advance `_current_wp_idx` (mod len), reset `_nav_failure_count = 0`, `_nav_active = False`. On ABORTED/CANCELED → `_nav_failure_count += 1`, `_nav_active = False` (Nav2's behavior_server has already run spin/backup recoveries internally before aborting — the counter counts *failed navigations*, satisfying the "3 failed recoveries → SAFE" intent of SYS-NAV-4).
  4. **Mode exits:** on entering TRACK_OBSERVE or SAFE, cancel any active goal (`goal_handle.cancel_goal_async()`).
  5. **Alert:** SAFE entry should raise an operator alert — blocked on PROP-01 (no alert channel exists); minimally, log at ERROR and set a field the report server's `/health` can expose.
- **Verify closure:** Requires a Nav2 that accepts goals — on hardware with a map, or in mock after GAP-16 + a map. Mock-level partial check: `ros2 topic echo /billiebot/mission_status` shows `nav_active: true` and `current_waypoint` advancing; force failures (no map loaded → goals abort) and confirm `recovery_count` climbs and mode → SAFE at 3. Full check = TC-16 + TC-19 on hardware.
- **Risks/notes:** Largest change in the plan — keep it a PR of its own. The `patrol_waypoints` param default (mission_controller.py:43-44) omits `bathroom` while `…/src/billiebot_mission/config/mission.yaml` includes it; the GAP-10 loader supersedes both.

#### GAP-10 — No patrol-waypoint source of truth or executor

**Status:** Open · **Severity:** 🟠 Major-functional · **Disposition:** F · **Effort:** M (fold into GAP-7)

- **What/Where:** `…/src/billiebot_navigation/config/patrol_waypoints.yaml` defines named `[x, y, yaw]` poses (living_room, kitchen, bedroom, hallway, bathroom) but **no node loads it**; `billiebot_interfaces/action/PatrolWaypoints.action` is defined but **no server implements it**; `mission_controller` keeps its own name-only list (no coordinates at all).
- **Why it matters:** MSN-06 (operator-configurable route), MSN-14, SYS-NAV-6. Names without poses cannot be navigated to.
- **Recommended fix:**
  1. Make `patrol_waypoints.yaml` the single source of truth. Load it in `mission_controller` via a `waypoints_config` parameter (same pattern as `dog_logger`'s `rooms_config` loader, `dog_logger.py:96-108` — reuse that yaml-load-with-fallback idiom), injected by `…/src/billiebot_mission/launch/mission.launch.py`.
  2. Keep the `patrol_waypoints` name-list param as the *route order* over those named poses; validate every name resolves at startup and fail loudly if not.
  3. **Defer** the `PatrolWaypoints` action server: with GAP-7's dispatch loop, a separate server is redundant for the MVP. Either implement it later as a thin wrapper for operator-triggered one-shot patrols, or delete the action definition (sponsor decision, §5.2) — don't leave it half-present.
- **Verify closure:** Launch rung 13 with a deliberately bad name in the route → startup error. With good config: `ros2 topic echo /billiebot/mission_status` shows `current_waypoint` cycling through the yaml's names (with GAP-7 in place).
- **Risks/notes:** Waypoint coordinates are placeholders until the apartment is mapped (`docs/MEASURE_ME.md` §Patrol Waypoints).

#### GAP-15 — Bark DoA triggers nothing

**Status:** Open · **Severity:** 🟠 Major-functional · **Disposition:** F · **Effort:** M

- **What/Where:** `mission_controller.py:115-121` — `_on_audio` logs `'Audio event: bark at DoA=…'` and returns; INVESTIGATE (Mode 2) is unreachable except via manual `/set_mode`.
- **Why it matters:** MSN-04 / SYS-FND-2 / STM-01 t2: the audio subsystem's DoA output (the reason the 4-mic array exists) has no behavioral consumer.
- **Recommended fix (after GAP-7):** In `_on_audio`, when mode == PATROL and `event_type in {BARK, HOWL, WHINE}`: (1) convert `doa_deg` (robot-frame) to a map-frame bearing using current pose from TF; (2) re-sort the remaining route by angular proximity of each waypoint to that bearing (the design's "re-prioritize", ACT-01 a5); (3) enter INVESTIGATE: cancel the current goal, dispatch the best-bearing waypoint, and start a timeout timer (suggest 120 s param `investigate_timeout_sec`). On `/dog/found` → TRACK_OBSERVE (STM t10); on timeout → PATROL (STM t11).
- **Verify closure:** Mock stack (audio mock emits BARK ~15 % of ticks — `audio_classifier.py` mock branch): watch `/billiebot/mission_status` — mode flips PATROL→INVESTIGATE(2) on a bark and back to PATROL(1) after the timeout. TC-27.
- **Risks/notes:** Mock barks fire often; gate INVESTIGATE re-entry with a cooldown (e.g., not more than once per 60 s) to avoid thrash — this is also a dog-welfare behavior (SYS-PLT-6 spirit).

#### GAP-13 — No near-dog speed restriction

**Status:** Open · **Severity:** 🟠 Major-functional (dog-welfare) · **Disposition:** F · **Effort:** M

- **What/Where:** `…/src/billiebot_navigation/config/nav2_params.yaml` encodes only the global cap (`max_vel_x: 0.3`, line 44; `max_speed_xy: 0.3`, line 48). No `nav2_costmap_2d::SpeedFilter`/keepout layer exists, and nothing converts `/dog/pose_map` into a filter mask. Only `approach_dog_server` caps its own goals at 0.15 m/s — ordinary patrol legs passing near the dog run at 0.3 m/s.
- **Why it matters:** NAV-12 / SYS-NAV-5 second clause; also the safety-rails-below-policy principle (MSN-13) for the future Behavior AI.
- **Recommended fix:** Nav2 speed-filter route (matches design §5.1 "speed-restricted zones"):
  1. New small node (suggest `billiebot_navigation/dog_speed_mask.py`): subscribes `/dog/pose_map`, publishes a `nav_msgs/OccupancyGrid` mask + `nav2_msgs/CostmapFilterInfo` where cells within 2.0 m of the dog encode a 0.15 m/s limit (percent-of-max encoding: 50 % of 0.3).
  2. Add `speed_filter` plugin to the **global** costmap's `filters` list in `nav2_params.yaml` (`plugin: nav2_costmap_2d::SpeedFilter`, `speed_limit_topic`, `filter_info_topic`) and set `controller_server` to subscribe to the speed-limit topic.
  3. Decay/clear the mask when the dog pose is stale (> 10 s) so the robot isn't permanently slowed by a ghost.
  - Simpler fallback (if filter plumbing fights back): a `cmd_vel` governor node between controller and base that scales Twist when TF distance robot→dog < 2 m. Less elegant (fights the planner) but 1-day-safe.
- **Verify closure:** Mock/hardware: place a (mock) dog pose 1 m ahead, send a nav goal past it, `ros2 topic echo /cmd_vel` → `linear.x ≤ 0.15` while within 2 m, returning to ≤ 0.3 beyond (TC-18 full version).
- **Risks/notes:** SpeedFilter needs the map frame — only meaningful with localization up. Tune inflation so the 2 m zone doesn't make doorway passages unplannable.

#### GAP-11 — Real detector silently inert without `model_path`

**Status:** Open · **Severity:** 🟠 Major-functional (field-failure trap) · **Disposition:** P (+ tiny F) · **Effort:** S

- **What/Where:** `…/src/billiebot_perception/billiebot_perception/oakd_dog_detector.py:24` — `declare_parameter('model_path', '')`; in real mode, line 84-87 only calls `setBlobPath` if non-empty, and on failure line 109 logs `'No model_path specified for OAK-D detector'` — **the node stays alive with no timer and publishes nothing**. Neither `…/launch/07_oakd.launch.py` (passes only `mock`) nor `perception.yaml` sets a real path.
- **Why it matters:** PER-01 / SYS-PER-1: on the actual robot the perception chain comes up "green" (node running) but blind; downstream `/dog/found` simply never fires. Worst kind of failure — silent.
- **Recommended fix:** (1) In real mode, treat empty/missing blob as **fatal**: log ERROR and exit non-zero so the launch supervisor makes the failure visible. (2) Add `model_path` to `…/src/billiebot_perception/config/perception.yaml` with the deployment path (e.g., `/opt/billiebot/models/yolov8n_coco_416x416.blob`) and make `07_oakd.launch.py` load that yaml (today it passes only `{'mock': mock}` — same fix pattern applies to rungs 09/10, which also skip `perception.yaml`).
  (3) Procure/convert the YOLOv8n blob for RVC2 (Luxonis blobconverter) and stage it at that path — hardware/asset task.
- **Verify closure:** `ros2 launch billiebot_bringup 07_oakd.launch.py` (real mode, no blob staged) → process exits with a clear error. With blob staged on hardware: `ros2 topic hz /dog/detections_3d` ≈ 5 Hz with a dog/photo in view (TC-07).
- **Risks/notes:** Keep mock mode's behavior unchanged (mock never needs the blob).

### Phase C sheets

#### GAP-9 — Logger narrower than design: snapshots, action/outcome, `/events/last`

**Status:** Open · **Severity:** 🟠 Major-functional · **Disposition:** F · **Effort:** M

- **What/Where:** `…/src/billiebot_cognition/billiebot_cognition/dog_logger.py`:
  - `_capture_snapshot` (lines 187-200) writes an **empty file** (`f.write(b'')`, line 197) with a comment admitting it's a placeholder.
  - The `action` column is hard-coded `'OBSERVE'` (line 172) and `outcome` is `''` (line 173); no subscription to engagement action results exists.
  - No `/events/last` interface (design §5.2 lists it), and no `/dog/found` subscription.
- **Why it matters:** STL-12/13/14, RPT-02, SYS-EXT-3. Daily reports have no images; the `(context, action, outcome)` dataset that is supposed to bootstrap the future bandit records a constant; operators can't query the last event.
- **Recommended fix (three independent sub-tasks):**
  1. **Snapshots:** subscribe to an image topic and JPEG-encode the latest frame on transition. Source choice: `/noir/image` is the only continuously-published image today (`sensor_msgs/Image rgb8`) — use it, and note the OAK-D preview alternative arrives with GAP-18. Keep the last frame in memory; on `is_transition` (line 142) encode with cv2/PIL, write, `os.fsync` (SYS-STL-4's "snapshot fsync"). Add a `snapshot_topic` parameter.
  2. **Action/outcome:** subscribe to the engagement action result topics (e.g., `/approach_dog/_action/status` or, cleaner, have each action server publish a small `std_msgs/String` JSON on a new `/billiebot/action_events` topic on completion — server names + results). On receipt, insert an event row with `action`/`outcome` filled. Keep `'OBSERVE'` as the default for state-transition rows.
  3. **`/events/last`:** add a service (reuse pattern of `state_fusion`'s `/get_dog_state`) or a latched topic republishing the last inserted row as JSON. Service is the design's intent; suggest `billiebot_interfaces/srv/GetLastEvent` or — to avoid a new interface — a latched `std_msgs/String` topic `/events/last`. Topic is cheaper; pick one and record it in the MBSE report's IBD-03.
- **Verify closure:** (1) Mock stack: force a state transition (mock detections toggling do this naturally), then `ls -la /var/lib/billiebot/snapshots/` → non-zero-byte JPEGs; `sqlite3 …/billie_events.db 'SELECT image_path FROM dog_events ORDER BY id DESC LIMIT 1'` points at it (TC-28). (2) Call `/approach_dog` (mock) → new row with `action='APPROACH'` and result outcome. (3) `/events/last` returns the same row.
- **Risks/notes:** `enable_snapshots` param (line 63) already exists — keep honoring it. On the real robot the logger runs on the Pi while the OAK-D is on the Jetson; `/noir/image` is Pi-local, another reason to prefer it.

#### GAP-12 — `/dog/found` dual-published, and never goes false from the locator

**Status:** Open · **Severity:** 🟡 Minor-hygiene (correctness edge) · **Disposition:** F · **Effort:** S

- **What/Where:** Publishers in both `…/src/billiebot_perception/billiebot_perception/oakd_dog_detector.py:36` (publishes every tick, true/false) and `…/dog_locator.py:34` (publishes `True` only on a successful transform, lines 60-62 — **never publishes `False`**).
- **Why it matters:** PER-04. Subscribers (mission_controller `_on_dog_found`) receive an interleaved stream from two sources with different semantics; whichever message arrives last wins. The locator's true-only stream also means "found" can stick if the detector node dies.
- **Recommended fix:** Single owner = `dog_locator` (it represents the *validated, map-frame* detection — the thing the mission actually cares about). (1) Delete the `found_pub` and its publish from `oakd_dog_detector.py` (lines 36 and the per-tick publish). (2) In `dog_locator`, make it stateful: publish `True` on successful transform, and add a timer that publishes `False` when no detection has been transformed for N seconds (suggest `found_timeout_sec: 2.0`). (3) Update `…/scripts/verify_rung_07.sh` if it checks `/dog/found` (rung 07 alone will no longer publish it — the check moves to rung 08).
- **Verify closure:** Rungs 07+08 mock: `ros2 topic info /dog/found --verbose` → exactly 1 publisher (`dog_locator`); stop the detector (`Ctrl-C` rung 07) → `/dog/found` flips to `false` within the timeout; mission drops TRACK_OBSERVE → PATROL.
- **Risks/notes:** In mock without TF (rung 08 needs rung 02's TF tree), the locator publishes nothing — run the combined stack when verifying.

#### GAP-3 — Orphan `BatteryStatus.msg`

**Status:** Open · **Severity:** 🟡 Minor-hygiene · **Disposition:** M (sponsor decision) · **Effort:** S

- **What/Where:** `…/src/billiebot_interfaces/msg/BatteryStatus.msg` (voltage, voltage_per_cell, cell_count, percentage, OK/LOW/CRITICAL status) is built by the interfaces package but published by nothing; `base_bridge.py` publishes standard `sensor_msgs/BatteryState` on `/battery_state` (line 157) with the thresholds mapped to `power_supply_health` enums (lines 332-340).
- **Why it matters:** IFC-06 — two contracts for one datum invites a future split-brain (someone subscribes to the bespoke one that never fires).
- **Options:** **(a) Delete** the message (recommended — `BatteryState`'s enums cover the need; less to maintain), or **(b) adopt** it: publish it alongside from `read_battery()` for the explicit OK/LOW/CRITICAL semantics the health-enum mapping only approximates.
- **Recommended fix:** (a): remove `msg/BatteryStatus.msg`, drop its entry from `…/src/billiebot_interfaces/CMakeLists.txt`, rebuild, and check `billiebot_tests/test/test_interfaces.py` for a reference to it (update the test if present).
- **Verify closure:** `colcon build` clean; `ros2 interface list | grep -i battery` shows only `sensor_msgs/msg/BatteryState`; interface tests pass.
- **Risks/notes:** Sponsor decision only because deleting a shipped interface is one-way; trivial either way.

### Phase D sheets

#### GAP-16 — Rung 01 mock produces no `/scan`

**Status:** Open · **Severity:** 🟠 Major-functional (test infrastructure) · **Disposition:** F · **Effort:** S

- **What/Where:** `…/src/billiebot_bringup/launch/01_lidar.launch.py:33-43` — the mock branch launches `billiebot_base`'s `base_bridge` executable named `mock_lidar_stub`, with the in-file comment "a dedicated mock scan publisher would be better. For now, this is a placeholder." `base_bridge` has no `/scan` publisher, so: rung 01's own verify criterion fails in mock, rungs 04/05/06 idle without scans, and every mock run of rung ≥ 04 spawns a **duplicate base_bridge** (second `/odom`, `/e_stop`, TF broadcaster).
- **Why it matters:** NAV-04, PLT-06. The entire hardware-free ladder above rung 03 is untestable, and the duplicate node corrupts the rungs that *are* testable.
- **Recommended fix:** Write `mock_scan_publisher.py` in `billiebot_bringup` (or `billiebot_base`): publishes `sensor_msgs/LaserScan` at ~6 Hz, `frame_id: laser_frame`, 360 samples over 2π, ranges 0.15–12 m simulating a simple rectangular room (parameterize room size). Register as an entry point; replace the mock branch in `01_lidar.launch.py` with it. This also unblocks mock SLAM (rung 04 can then actually build a toy map) and TC-23.
- **Verify closure:** `ros2 launch billiebot_bringup 01_lidar.launch.py mock:=true` → `./…/scripts/verify_rung_01.sh` passes; `ros2 launch billiebot_bringup 04_slam.launch.py mock:=true` → `ros2 node list` shows exactly one `base_bridge`; `/map` appears after simulated motion.
- **Risks/notes:** Keep the synthetic room consistent with wheel odometry (static room + moving robot needs scan generation from the robot's mock pose — acceptable v1: static scan, which still exercises topic plumbing; note the limitation in the node docstring).

#### GAP-17 — Speak action naming split

**Status:** Open · **Severity:** 🟡 Minor-hygiene · **Disposition:** F · **Effort:** S

- **What/Where:** `…/src/billiebot_audio/billiebot_audio/speaker_node.py:39` serves action `/speak`; `…/src/billiebot_mission/billiebot_mission/speak_server.py:23` serves `/mission/speak` and forwards to `/speak` (client at line 18). The unused BT XML references a `Speak` node bound to neither.
- **Why it matters:** AUD-06 / SYS-EXT-1's "uniform interface": the future policy needs one canonical name per primitive; today there are two, and the wrapper adds a hop with no added rails (`speaker_node` already enforces rate/volume limits, lines 51/64).
- **Recommended fix:** Delete the `speak_server.py` wrapper (remove from `…/src/billiebot_mission/launch/mission.launch.py` and `setup.py` entry points); `/speak` on `speaker_node` becomes the canonical engagement primitive alongside `/approach_dog`, `/retreat`, `/dispense_treat`. Rationale: the welfare rails must live in the leaf server anyway (MSN-13) — the wrapper duplicates nothing useful.
- **Verify closure:** Rungs 11+13 up: `ros2 action list | grep speak` → only `/speak`; `ros2 action send_goal /speak billiebot_interfaces/action/Speak "{sound_id: test, volume: 0.2}"` twice within 10 s → second goal rejected (rate limit intact).
- **Risks/notes:** Update MBSE report IBD-03 c13 note when done.

#### GAP-1 — State machine vs. designed behavior tree (sponsor decision)

**Status:** Open (decision needed) · **Severity:** 🟠 Major-architectural · **Disposition:** F after decision · **Effort:** L (option A) / M (option B)

- **What/Where:** Design (SYS-EXT-2) specifies mission logic as a BehaviorTree.CPP tree with a `PolicyDecision` extension point. Implemented: `mission_controller.py` (Python IntEnum state machine, no policy hook). BT assets exist but are orphaned: `…/src/billiebot_mission/behavior_trees/billiebot_main.xml` (references ~10 leaf nodes that have no C++ implementation), `src/policy_decision_node.cpp` (compiled into `billiebot_bt_nodes` lib, always returns `"OBSERVE"`, never ticked), `battery_guard_node.cpp`, `estop_guard_node.cpp`.
- **Why it matters:** SYS-EXT-2 is the Behavior-AI insertion contract. Also, half-present BT assets mislead readers into thinking the tree runs.
- **Options:**
  - **A — Commit to the BT (design intent):** implement the missing leaf nodes (NavigateToWaypoint, ReorderPatrol, AdvanceWaypoint, HoldPosition, AlertOperator, StopMotors, IsSafeMode, DecisionIs, DogDetected, HasAudioEvent), write a BT executor node that loads `billiebot_main.xml` and ticks it, and retire `mission_controller.py`. Honors SYS-EXT-2 literally; large C++ effort; duplicates the Phase B Python work unless done *instead of* it.
  - **B — Commit to the state machine (recommended):** declare `mission_controller.py` the mission architecture; port the `PolicyDecision` seam into it as a Python strategy object (`policy.decide(dog_state, stress_proxy) -> {OBSERVE, APPROACH, RETREAT, SPEAK, TREAT}`, default `ObserveOnlyPolicy`) called from the TRACK_OBSERVE branch; **delete** the orphaned BT XML/C++ (or move to a `design_reference/` folder); update the design doc + MBSE report to re-baseline SYS-EXT-2 as "mission logic shall route decisions through a PolicyDecision extension point" (implementation-neutral wording).
  - Recommendation rationale: the extension *contract* (uniform action servers + `/billie/state` context + policy seam) is what SYS-EXT-1/3/4 actually protect; the BT was a means, and Phase B lands naturally in Python.
- **Verify closure:** Option B: unit-test that a stub policy returning `APPROACH` causes an `/approach_dog` goal (mock); grep confirms no `BehaviorTree` build targets remain; MBSE report §2.2 SYS-EXT-2 row updated.
- **Risks/notes:** Decide before investing further in `mission_controller.py` beyond Phase B — the decision determines where GAP-15's logic permanently lives.

#### GAP-2 — IMU dormant (hardware pin conflict)

**Status:** Open (hardware-blocked) · **Severity:** 🟠 Major-functional (odometry quality) · **Disposition:** H then P · **Effort:** M

- **What/Where:** The BNO055 needs the Nano's I²C pins A4/A5, which the right encoder currently occupies (`docs/MEASURE_ME.md` §IMU Hardware Rewire). Consequently: `base_driver.yaml:31` `use_imu: false`; `ekf.yaml:26-36` — the entire `imu0` block commented out; `base_bridge` publishes no `/imu/data`.
- **Why it matters:** NAV-06 — with a single input the EKF is a smoother, not a fusion; yaw drift over long patrols degrades AMCL convergence and room attribution.
- **Recommended fix (ordered):** (1) Rewire right encoder A4/A5 → D4/D7 per MEASURE_ME; (2) update firmware pin-change interrupt config (PORTD) and add the `'i'` IMU read command per `firmware/README.md`; (3) extend `base_bridge` to poll `'i'` and publish `sensor_msgs/Imu` on `/imu/data`; (4) set `use_imu: true` (base_driver.yaml:31); (5) uncomment `imu0` in `ekf.yaml`; (6) confirm GAP-4 is closed so Nav2 actually benefits.
- **Verify closure:** `ros2 topic hz /imu/data` ≈ expected rate; rotate robot 360° by teleop → `/odometry/filtered` yaw returns to start within a few degrees while raw `/odom` shows drift.
- **Risks/notes:** Pure-hardware first step; schedule with the encoder-calibration bench session (MEASURE_ME). Until then this gap is *accepted*, not forgotten.

#### GAP-18 — No `/oak/rgb/preview` published

**Status:** Open · **Severity:** 🟡 Minor-hygiene · **Disposition:** F or M · **Effort:** S

- **What/Where:** Design §5.2 lists `/oak/rgb/preview` from the OAK-D node; `oakd_dog_detector.py` publishes only `/dog/detections_3d` (+ `/dog/found`, until GAP-12). The DepthAI pipeline already builds a 416×416 ColorCamera preview internally — it just isn't exported as a ROS topic.
- **Why it matters:** SYS-PLT-4 (operator visualization: "camera, detections") and it's the natural snapshot source for GAP-9 once available.
- **Recommended fix:** Add an XLinkOut for the preview stream in the DepthAI pipeline and publish `sensor_msgs/Image` (rgb8) on `/oak/rgb/preview` at a throttled rate (2–5 Hz — Wi-Fi budget). Alternatively (M): waive it and update the design doc — acceptable only if GAP-9 uses `/noir/image`.
- **Verify closure:** On hardware: `ros2 topic hz /oak/rgb/preview` ≈ configured rate; image visible in Foxglove.
- **Risks/notes:** Real-hardware only (mock mode should synthesize a frame or skip). Watch OAK-D USB power budget (design §4.1 gotcha).

#### GAP-19 — Pi 4 vs. Pi 5 naming drift

**Status:** Resolved (2026-07-12) · **Severity:** ⚪ Doc-only · **Disposition:** M · **Effort:** S

- **What/Where:** Design doc says Raspberry Pi 5; `README.md` ("Raspberry Pi 4", twice), `docs/VERIFICATION.md` ("Raspberry Pi 4:"), and a comment in `cyclonedds.xml` referenced Pi 4; several guides/MBSE diagrams hedged with "Pi 4/5". (The sheet's original mention of `base_driver.yaml` was stale — that file carries no Pi reference.)
- **Why it matters:** Whichever board is actually deployed changes the camera stack (Pi 5 dual CSI + picamera2 requirements), PD power board suitability (design §4.2 assumes Pi 5's 5 V/5 A USB-C PD), and Docker/OS choices (design §5.1).
- **Recommended fix:** Confirm the physical board, then sweep all references to the same truth: `grep -rn "Pi 4\|Pi 5\|Raspberry" README.md docs/ billiebot_ws/src/billiebot_bringup/config/ billiebot_ws/src/billiebot_base/config/` and align. If it's a Pi 4, also revisit the power budget line items in the design doc.
- **Resolution:** Board confirmed as **Raspberry Pi 5**. Swept every reference to Pi 5 — `README.md` (hardware table + launch comment), `docs/md/VERIFICATION.md`, `docs/md/MEASURE_ME.md`, `docs/md/BRINGUP_LADDER_ANALYSIS.md`, `docs/md/MBSE_SYSTEM_DECOMPOSITION.md` (table + BDD text + both mermaid subgraph labels), `docs/md/INSTALLATION_AND_SETUP.md` (target list, hardware table, §2.3 note, Appendix D), and `billiebot_ws/src/billiebot_bringup/config/cyclonedds.xml` comments. The design doc (source of truth) already read Pi 5, so its power budget is unchanged. Generic strings ("Pi Camera 3 NoIR", "Raspberry Pi-only nodes") and read-only `reference_my_bot/` left as-is.
- **Verify closure:** The grep above returns a single consistent board name (Pi 5); no active "Pi 4" board reference remains outside this historical record.
- **Risks/notes:** None; done alongside the doc sweep.

---

## 5. Cross-Reference Appendix

### 5.1 Gap ↔ requirement ↔ test mapping

| GAP | MBSE report L2 reqs | L1 reqs | Verifying TC (existing/proposed) | Ladder-analysis Appendix B |
|---|---|---|---|---|
| GAP-1 | MSN-01, MSN-12, EXT-02 | SYS-EXT-2 | TC-13 (partial) | — |
| GAP-2 | NAV-06 | SYS-NAV-2 | TC-03 (extended) | — |
| GAP-3 | IFC-06 | SYS-PLT-2 (hygiene) | TC-01 | — |
| GAP-4 | NAV-08 | SYS-NAV-2 | TC-16 | B-8 |
| GAP-5 | NAV-07 | SYS-NAV-2 | TC-24 ★ | B-3 |
| GAP-6 | NAV-07 | SYS-NAV-2 | TC-24 ★ | B-4 |
| GAP-7 | MSN-05, NAV-14 | SYS-NAV-4/6, SYS-FND-1 | TC-16, TC-19 | B-5 |
| GAP-8 | MSN-02 | SYS-PLT-5 | TC-06 (extended) | — |
| GAP-9 | STL-12/13/14, RPT-02 | SYS-STL-2, SYS-EXT-3, SYS-RPT-1 | TC-20, TC-28 ★ | — |
| GAP-10 | MSN-06, MSN-14 | SYS-NAV-6 | TC-16 | B-5 |
| GAP-11 | PER-01 | SYS-PER-1 | TC-07 | B-11 |
| GAP-12 | PER-04 | SYS-PER-2 | TC-07/08 | — |
| GAP-13 | NAV-12 | SYS-NAV-5 | TC-18 (full) | B-7 |
| GAP-14 | MOB-05 | SYS-PLT-5 | TC-29 ★ | — |
| GAP-15 | MSN-04 | SYS-FND-2 | TC-27 ★ | — |
| GAP-16 | NAV-04, PLT-06 | testability | TC-23 ★ | B-1, B-2 |
| GAP-17 | AUD-06 | SYS-EXT-1 | TC-01 (naming) | — |
| GAP-18 | — (SYS-PLT-4 minor) | SYS-PLT-4 | — | — |
| GAP-19 | — | doc-only | — | B-9 (adjacent) |

★ = proposed test defined in `MBSE_SYSTEM_DECOMPOSITION.md` §5.2 (TC-23…TC-30).

Numbering history: GAP-1…10, 14, 16, 17 match the original 17-item design-vs-code discrepancy list from the 2026-07-04 exploration; the original #11 (dual `/dog/found`) → GAP-12, #12 (`/oak/rgb/preview`) → GAP-18, #13 (audio re-sort stub) → GAP-15, #15 (Pi naming) → GAP-19; GAP-11 (model_path) and GAP-13 (near-dog speed) were promoted from sub-findings to first-class gaps.

### 5.2 Sponsor decisions needed (gate before the affected phase)

| # | Decision | Gates | Sheet |
|---|---|---|---|
| D-1 | Mission architecture: behavior tree (option A) vs. state machine + Python policy seam (option B, recommended) | Phase D (and where GAP-15 logic permanently lives) | GAP-1 |
| D-2 | `BatteryStatus.msg`: delete (recommended) vs. adopt | Phase C | GAP-3 |
| D-3 | `PatrolWaypoints.action`: implement as operator-facing wrapper vs. delete | Phase B cleanup | GAP-10 |
| D-4 | Night path: fund 850 nm IR illuminator (design gap list / MBSE PROP-07) vs. waive SYS-PER-5 to thermal-only | hardware order | (adjacent to GAP-18) |
| D-5 | Stuck-detection numbers: relax SYS-NAV-4 to 0.5 m/10 s vs. tighten Nav2 progress checker to 5 s (MBSE PROP-04) | Phase B (TC-19 pass criteria) | (parameter of GAP-7 verification) |
| D-6 | Alert channel design (MBSE PROP-01) — SAFE-mode "alert operator" has no mechanism; needed by GAP-7 step 5 and GAP-8's SAFE path | Phase B completeness | GAP-7 |

*— End of plan. Update the §2 Status column as gaps close. —*


