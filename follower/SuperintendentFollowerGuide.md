# Superintendent Follower Guide

This guide explains how to start a follower robot that consumes overhead camera
distance events from `Superintendent`.

The overhead camera publisher is documented in
[`DistanceDetection/MqttCameraDistanceGuide.md`](../DistanceDetection/MqttCameraDistanceGuide.md).

## Required Publisher

Start the distance detector with the same marker pair the follower will use:

```bash
python3 /Platooning-project/DistanceDetection/DistanceDetection.py \
  --mqtt-broker 10.0.7.51 \
  --mqtt-marker-pair green:pink
```

This publishes `GLOBAL_SEARCH_MARKER_DISTANCE` events to:

```text
platoon/robot/Superintendent/event
```

## Start Follower

Start the follower with the overhead marker pair assigned to this robot:

```bash
python3 /Platooning-project/run_follower.py \
  --lock-tilt \
  --robot-id Hexapod1 \
  --broker 10.0.7.51 \
  --superintendent-source-marker pink \
  --superintendent-target-marker green
```

`--superintendent-source-marker` is the overhead marker attached to this
robot.

`--superintendent-target-marker` is the overhead marker this robot should move
toward during `GLOBAL_SEARCH`.

For the example above, `Hexapod1` has the `green` overhead marker and should
move toward the `pink` overhead marker.

Both options must be passed together.

## Consumed Messages

The follower listens to:

```text
platoon/robot/+/event
```

It accepts only:

```text
robot_id = Superintendent
event = GLOBAL_SEARCH_MARKER_DISTANCE
source_marker = configured source marker
target_marker = configured target marker
```

The robot uses `distance_m`, `dx_m`, and `dy_m` from the payload.

`distance_m` is the stabilized distance between the two overhead markers.

`dx_m` and `dy_m` are the overhead world-plane vector from the source marker to
the target marker.

## State Behavior

`GLOBAL_SEARCH` first tries onboard blob detection with `global_detect(...)`.

If the blob is visible, the follower publishes `TARGET_FOUND_GLOBAL` and enters
`GLOBAL_APPROACH`.

If the blob is not visible and a fresh Superintendent measurement exists,
`GLOBAL_SEARCH` uses `dx_m` and `dy_m` for coarse left/right and forward/back
motion.

If the blob is not visible and no fresh Superintendent measurement exists,
`GLOBAL_SEARCH` falls back to the normal local search motion.

If `GLOBAL_APPROACH` loses the blob for longer than the global lost-target
timeout, it publishes `TARGET_LOST_GLOBAL` and returns to `GLOBAL_SEARCH`.
From there, a fresh close Superintendent measurement can immediately route the
robot back into `GLOBAL_VISUAL_ACQUIRE`.

## GLOBAL_VISUAL_ACQUIRE

`GLOBAL_VISUAL_ACQUIRE` is the bridge between overhead guidance and onboard
camera approach.

It is entered from `GLOBAL_SEARCH` when:

```text
Superintendent distance_m <= 0.80
```

Inside this state the robot:

1. Checks onboard `global_detect(...)` first.
2. Transitions to `GLOBAL_APPROACH` as soon as the blob is visible.
3. Sweeps the camera left/right if the blob is not visible.
4. Rotates the body slowly in place when the camera pan is near its limit.
5. Otherwise uses small Superintendent-guided creep moves.
6. Stops between rotate/creep moves to let the camera image settle.

It exits back to `GLOBAL_SEARCH` when:

```text
Superintendent measurement is stale
Superintendent distance_m > 1.10
visual acquisition takes longer than 8 seconds
```

The Superintendent does not directly trigger `GLOBAL_APPROACH`; onboard blob
detection still owns that transition.

## Quick Checklist

1. Start `DistanceDetection.py` with `--mqtt-broker` and `--mqtt-marker-pair`.
2. Start `run_follower.py` with matching Superintendent marker arguments.
3. Send or wait for the follower assignment.
4. Put the follower into `GLOBAL_SEARCH`.
5. Watch the follower state and `platoon/robot/Superintendent/event`.
