# Follower change log

Changes made to the follower, grouped by when the work was done.

# Green detection update — earlier session (before 3 Aug 2026)

## 1. Green detection in dim light

**Problem:** the hexapod lost the green target as soon as the room got slightly dark.

**Files:** `follower/robot_implementation.py`

- **Widened the green HSV window** (`COLOR_RANGES["green"]`): from `(35,70,60)–(85,255,255)` to
  `(35,60,50)–(90,255,255)`. The old saturation/value floors were the first thing to fail as the
  light dropped.
- **CLAHE on the V channel for dim frames** (`detect_target`): if the frame's mean brightness is
  below `DIM_FRAME_V_MEAN` (110), the value channel is contrast-equalised before thresholding, so
  the target keeps roughly the same HSV signature it has under good light.
- **Relaxed second detection pass**: if the normal mask yields no blob, detection retries with
  saturation/value floors scaled down (`RELAXED_SAT_SCALE` 0.45 / `RELAXED_VAL_SCALE` 0.40, floors
  35 / 25) via the new `relax_ranges()` helper. Good light still uses the strict mask, so no extra
  false positives there.
- **Refactor:** mask building and contour selection were pulled out into `build_color_mask()` and
  `largest_target_contour()` so both passes share the same code.
- **Occlusion check made relative** (`detect_target_occlusion`): it used a fixed `gray < 80` cutoff,
  so in a dim room everything around the target read as "dark" and the robot declared an occlusion
  even while it was seeing the target. The cutoff is now 55% of frame mean brightness, clamped to
  25–80.

# Today's session — 3 Aug 2026

## 2. Stop distance tuning (no code change)

**Question:** which parameter controls how close the hexapod gets before stopping.

**Answer:** `TARGET_AREA_MIN` (`robot_implementation.py`, default `0.05`), exposed as the
`--target-area` CLI flag in `run_follower.py`. It is the fraction of frame pixels the target blob
must cover. A bigger physical object reaches that fraction from further away, so it stops early —
**raise** the value to make it approach closer. Area scales as 1/distance², so distance scales as
1/√area: 4× the value ≈ half the stopping distance.

Related:
- `target_area_max = target_area_min + 0.15` is the back-off ("too close, reverse") threshold and
  tracks the setting automatically.
- The stop fires on **whichever comes first**, visual area or ultrasonic distance
  (`state_machine.reached_global_approach_goal`).
- `ignore_lower_frame` (default 0.25) blanks the bottom quarter of the mask, so a very close target
  loses real area — pushing `--target-area` much past ~0.2 may never trigger.

## 3. Local-follow re-search when the target is lost

**Problem:** once in `LOCAL_FOLLOW`, losing sight of the green object left the robot stuck.

**Cause:** `handle_local_follow` dropped to `LOST_TARGET` on the very first missed frame, and
`LOST_TARGET` → `GLOBAL_SEARCH`, which with no fresh superintendent measurement just calls
`global_search_stop()` and stands still.

**Files:** `follower/state_machine.py`, `follower/robot_implementation.py`

- **`local_search_scan_motion()`** (new, on both the base class and the Freenove implementation):
  forces the camera to pan 0 / tilt 0 and rotates the **body** in place, always the same direction,
  so the search combs the full 360° horizon. Rotation step `LOCAL_SEARCH_ROTATE_STEP` (now 6°),
  issued every `LOCAL_SEARCH_MOVE_INTERVAL` (0.50 s — raised from 0.35 s because the swerving was
  too aggressive). No forward/backward motion while scanning.
- **No timeout:** `LOCAL_FOLLOW` keeps rotating until it reacquires; it no longer falls out to
  `LOST_TARGET` / `GLOBAL_SEARCH` on its own.
- **Size gate so small green dots don't stop the search** — this was the real "gets stuck" cause,
  since the scan would otherwise latch onto any blob above the tiny 0.002 floor:
  - `local_detect()` now takes a `min_area` argument, applied inside `_detect()`; a detection under
    it is discarded (`action_status = "local-search-reject-small"`).
  - `local_search_area_floor()` returns `max(LOCAL_SEARCH_AREA_FLOOR 0.010,
    LOCAL_SEARCH_AREA_RATIO 0.35 × last_tracked_area)`.
  - `last_tracked_area` is a new field holding the area the target had the last time it was
    genuinely tracked, so the gate scales with the object and with distance.
  - Normal following passes `min_area=0` and is unaffected.
- **State machine:** new `local_search_active` flag; set when the scan starts (publishes
  `LOCAL_SEARCH_SCANNING`), cleared on reacquire (`LOCAL_SEARCH_REACQUIRED`) and on any transition
  out of `LOCAL_FOLLOW`. `last_seen_time` is now also refreshed on every successful local-follow and
  local-lock frame.

## 4. Known defect — the search overshoots

**Observed:** when the target is lost, the robot performs a full swing even when the object is
sitting only ~30° away. It does not stop turning promptly at the bearing where the target reappears.

**Why my implementation does this — two separate causes:**

1. **The rotation direction is arbitrary.** `local_search_scan_motion()` turns in
   `self.search_direction`, which is whatever it happened to be from an earlier global search — not
   the side the target actually exited towards. Roughly half the time it starts combing *away* from
   the target and has to travel ~330° to reach something that was 30° away.
2. **Nothing damps the turn on reacquire.** The scan issues a fixed 6° step every 0.5 s and only
   stops when a frame comes back detected. Combined with frame latency and the size gate rejecting
   the target's first partial appearances at the frame edge, the body carries past the bearing
   before the follow controller takes over.

**Suggested fix (not applied):**

- **Seed the direction from the last known bearing.** In `_detect()`, `self.target_x` already holds
  the horizontal position of the target (−1 left … +1 right) from the last good frame. Add a
  `last_tracked_x` field alongside `last_tracked_area`, and at the start of a local search set
  `self.search_direction = 1 if last_tracked_x >= 0 else -1`. That alone turns the worst case from
  ~330° into ~30° for a target that slid out of one side of the frame.
- **Slow down near the expected bearing / on partial sighting.** Two options, cheapest first:
  - Shrink the step once the accumulated rotation approaches the last known bearing (track a running
    sum of the degrees commanded since the search started).
  - Use a two-tier area gate: keep the strict `local_search_area_floor()` for *ending* the search,
    but treat any above-`LOCAL_SEARCH_AREA_FLOOR` blob as "warm" and drop the step to 2–3° so the
    body creeps onto the target instead of swinging past it.
- **Optional:** bound the comb with a lap counter (accumulated degrees ≥ 360 ⇒ one full lap) and
  reverse direction or fall back to `GLOBAL_SEARCH` after one or two laps, so a genuinely absent
  target doesn't spin forever.
