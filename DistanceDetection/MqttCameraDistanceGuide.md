# MQTT Camera Distance Guide

This guide explains how to start the overhead camera distance detector and what
MQTT messages it publishes.

## What Runs

`DistanceDetection.py`:

1. Opens the RTSP camera stream.
2. Detects configured color markers from the `COLORS` table in
   `DistanceDetection.py`.
3. Measures marker-pair distances on the calibrated floor plane.
4. Shows raw and stabilized distances in the browser video stream.
5. Publishes selected marker-pair distances continuously over MQTT as
   `Superintendent`.

The browser view is:

```text
http://<camera-host-ip>:5000/
```

On the same machine:

```text
http://localhost:5000/
```

## Calibration

Calibration is persistent.

Run calibration only when the camera moved, the camera angle changed, the
measured floor rectangle changed, the calibration file was deleted, or the
displayed distance looks wrong.

Start calibration:

```bash
python3 /Platooning-project/DistanceDetection/DistanceDetection.py --calibrate
```

Open the browser view and click the floor rectangle corners in this exact
order:

1. Top-left
2. Top-right
3. Bottom-right
4. Bottom-left

Then enter the measured side lengths and apply calibration.

The calibration is saved here by default:

```text
~/.local/share/distance_detection2/calibration.json
```

Normal startup loads the saved calibration automatically.

## Start Without MQTT

Use this when only the browser video and distance overlay is needed:

```bash
python3 /Platooning-project/DistanceDetection/DistanceDetection.py
```

By default, the overlay draws all detected marker pairs.

```text
green-pink: raw 91.00 cm | stable 82.00 cm
```

To draw only selected pairs, repeat `--display-pair`:

```bash
python3 /Platooning-project/DistanceDetection/DistanceDetection.py \
  --display-pair green:pink
```

## Start With MQTT

Use `--mqtt-marker-pair source:target` to choose what distances are published.
The option can be repeated.

Single pair example:

```bash
python3 /Platooning-project/DistanceDetection/DistanceDetection.py \
  --mqtt-broker 10.0.7.51 \
  --mqtt-marker-pair green:pink
```

Multiple pair example:

```bash
python3 /Platooning-project/DistanceDetection/DistanceDetection.py \
  --mqtt-broker 10.0.7.51 \
  --mqtt-marker-pair green:pink \
  --mqtt-marker-pair green:blue
```

The distance detector always publishes as `Superintendent`.

## Follower Usage

Follower startup and state-machine behavior are documented separately in
[`follower/SuperintendentFollowerGuide.md`](../follower/SuperintendentFollowerGuide.md).

## Published Topic

Distance events are published to:

```text
platoon/robot/Superintendent/event
```

Example:

```text
platoon/robot/Superintendent/event
```

QoS is `1`. Retain is `false`.

When multiple MQTT marker pairs are configured, each pair is published as a
separate event payload to the same topic during each publish tick.

## Successful Distance Event

Event name:

```text
GLOBAL_SEARCH_MARKER_DISTANCE
```

Example payload:

```json
{
  "type": "EVENT",
  "robot_id": "Superintendent",
  "event": "GLOBAL_SEARCH_MARKER_DISTANCE",
  "state": "MEASURING",
  "timestamp": 1784343103.09,
  "source_marker": "green",
  "target_marker": "pink",
  "distance": 82.0,
  "raw_distance": 91.0,
  "unit": "cm",
  "distance_m": 0.82,
  "raw_distance_m": 0.91,
  "dx_m": 0.34,
  "dy_m": -0.71,
  "source_world_x_m": 1.24,
  "source_world_y_m": 0.42,
  "source_frontier_distance_m": 0.42,
  "source_frontier_side": "top",
  "pixel_distance": 312.4,
  "dx_pixels": 240,
  "dy_pixels": -199,
  "calibrated": true,
  "filtered": true,
  "filter_reset": false,
  "filter_rejected_jump_count": 0,
  "filter_window": 5,
  "filter_alpha": 0.35,
  "filter_max_jump_m": 0.5,
  "filter_max_jump_rejects": 3,
  "measurement_timestamp": 1784343103.05
}
```

Use `distance_m` for robot decisions. It is the stabilized value.

Use `raw_distance_m` for debugging and tuning. It is the newest camera
measurement before filtering.

## Unavailable Distance Event

Event name:

```text
GLOBAL_SEARCH_MARKER_DISTANCE_UNAVAILABLE
```

No camera frame yet:

```json
{
  "type": "EVENT",
  "robot_id": "Superintendent",
  "event": "GLOBAL_SEARCH_MARKER_DISTANCE_UNAVAILABLE",
  "state": "MEASURING",
  "timestamp": 1784343103.09,
  "source_marker": "green",
  "target_marker": "pink",
  "error": "No frame is available yet."
}
```

Markers missing:

```json
{
  "type": "EVENT",
  "robot_id": "Superintendent",
  "event": "GLOBAL_SEARCH_MARKER_DISTANCE_UNAVAILABLE",
  "state": "MEASURING",
  "timestamp": 1784343103.09,
  "source_marker": "green",
  "target_marker": "pink",
  "available_markers": ["green"],
  "error": "Could not detect both requested markers."
}
```

## Stabilization

The detector publishes both raw and stabilized values.

`raw_distance_m` is the newest unfiltered camera measurement.

`distance_m` is the stabilized value after rolling median, exponential
smoothing, and large-jump rejection. Consecutive large jumps are treated as a
real robot/camera/marker movement and reset the filter to the new measurement,
so the stable value does not stay stuck on the old position.

Tune stabilization at startup:

```bash
python3 /Platooning-project/DistanceDetection/DistanceDetection.py \
  --mqtt-broker 10.0.7.51 \
  --mqtt-marker-pair green:pink \
  --mqtt-period 0.5 \
  --filter-window 5 \
  --filter-alpha 0.35 \
  --filter-max-jump-m 0.50 \
  --filter-max-jump-rejects 3
```

## Field Notes

`distance` is the stabilized distance in the calibration unit, such as `cm`.

`distance_m` is the stabilized distance converted to meters.

`raw_distance` is the newest unfiltered distance in the calibration unit.

`raw_distance_m` is the newest unfiltered distance converted to meters.

`filter_reset` is `true` on the sample where the stabilizer accepted repeated
large jumps as a real movement and re-anchored the stable value.

`filter_rejected_jump_count` counts consecutive large jumps that are still
being rejected as likely noise.

`dx_m` and `dy_m` are the overhead world-plane vector from `source_marker` to
`target_marker`, converted to meters.

`source_frontier_distance_m` is the source marker's nearest distance to the
calibrated floor rectangle boundary. `source_frontier_side` names that nearest
calibration side.

`pixel_distance`, `dx_pixels`, and `dy_pixels` are image-space diagnostics.

`calibrated` should be `true` for real-world distance. If it is `false`, the
camera can still detect marker positions, but real-world distance is not ready.

## Quick Checklist

1. Calibrate once if needed.
2. Start `DistanceDetection.py` with `--mqtt-broker` and at least one
   `--mqtt-marker-pair`.
3. Open `http://<camera-host-ip>:5000/` and confirm the markers are visible.
4. Watch `platoon/robot/Superintendent/event` for
   `GLOBAL_SEARCH_MARKER_DISTANCE`.
5. Start the follower with matching markers when the robot should use these
   events.
