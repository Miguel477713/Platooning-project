# Distance Detection Guide

This guide explains how `DistanceDetection.py` measures the real distance
between the green and pink objects using four blue reference markers.

## What The Program Does

`DistanceDetection.py` starts a Flask video stream, reads frames from an RTSP
camera, detects colored objects with HSV thresholds, and overlays:

- The four blue calibration markers.
- A blue calibration rectangle and its real side lengths.
- The green and pink object positions.
- A white line between the green and pink objects.
- The real green-pink distance in the configured unit.

The stream is served at:

```bash
http://<jetson-ip>:5000/
```

For the same machine running the script, use:

```bash
http://localhost:5000/
```

## High-Level Flow

1. The camera frame is captured from `RTSP_URL`.
2. The newest frame is kept and old buffered frames are dropped.
3. The frame is converted from BGR to HSV.
4. Blue markers are detected only until calibration succeeds.
5. The four blue marker centers are ordered as:
   `top-left`, `top-right`, `bottom-right`, `bottom-left`.
6. A homography matrix is calculated from the blue marker rectangle.
7. That homography is saved in memory for the rest of the runtime.
8. Green and pink objects are detected every frame.
9. The green and pink points are projected into the calibrated real-world plane.
10. The Euclidean distance between those projected points is displayed.

## Why Homography Is Used

The camera is looking at the floor from an angle, so a single pixel-to-centimeter
ratio is not reliable across the full image. Objects near the top of the image
and objects near the bottom of the image do not share the same pixel scale.

Instead, the program uses four blue reference markers to compute a perspective
transform, also called a homography. This maps image coordinates into real-world
coordinates on the floor plane.

Because of this, the program saves the homography matrix, not a simple
pixel/unit ratio.

## Calibration Behavior

At startup, the program searches for blue markers until it finds four valid
blue blobs.

Once the four markers are found:

- The homography is calculated.
- The transform is saved in memory.
- The program stops recalculating blue markers every frame.
- The same saved transform is used for the rest of the runtime.

This makes the green-pink distance more stable because temporary blue-marker
detection failures do not break measurement after calibration.

The on-screen calibration status can show:

```text
calibration: waiting for 4 blue markers - N found
calibration: saved transform from 4 blue markers
calibration: using saved transform - recalibrate if camera moved
```

If a blue marker disappears after calibration, distance measurement can continue
as long as the camera and floor reference plane have not moved.

If the camera moves, the saved homography becomes invalid. Restart the program
or recalibrate before trusting the distance values again.

## Reference Marker Dimensions

The four blue markers define a real rectangle. The program accepts four side
lengths:

- `--top`
- `--right`
- `--bottom`
- `--left`

All values use the same unit. The default unit is `cm`.

The current default side length is:

```text
100 cm
```

If your blue markers form a 50 cm by 50 cm square, start the program with:

```bash
python3 "Distance detection/DistanceDetection.py" --top 50 --right 50 --bottom 50 --left 50
```

If opposite sides are slightly different, the program prints a note and uses
the average of opposite sides internally. A homography requires rectangular
world coordinates.

## Terminal Usage

Run these commands from the project root:

```bash
cd /Platooning-project
python3 "Distance detection/DistanceDetection.py"
```

Then open:

```bash
http://localhost:5000/
```

or from another device on the same network:

```bash
http://<jetson-ip>:5000/
```

## Common Invocations

Use a 50 cm square calibration reference:

```bash
python3 "Distance detection/DistanceDetection.py" --top 50 --right 50 --bottom 50 --left 50
```

Use a 100 cm by 80 cm rectangle:

```bash
python3 "Distance detection/DistanceDetection.py" --top 100 --bottom 100 --left 80 --right 80
```

Use meters instead of centimeters:

```bash
python3 "Distance detection/DistanceDetection.py" --top 1 --right 1 --bottom 1 --left 1 --unit m
```

Enter the side lengths interactively:

```bash
python3 "Distance detection/DistanceDetection.py" --interactive
```

Use a different RTSP camera URL:

```bash
RTSP_URL="rtsp://user:password@camera-ip:554/stream1" python3 "Distance detection/DistanceDetection.py"
```

Reduce the processing width for faster processing:

```bash
PROCESS_WIDTH=960 python3 "Distance detection/DistanceDetection.py"
```

Limit processing to 15 FPS:

```bash
TARGET_FPS=15 python3 "Distance detection/DistanceDetection.py"
```

## Command-Line Options

`--top VALUE`

Real length of the top blue-marker side. Default: `100`.

`--right VALUE`

Real length of the right blue-marker side. Default: `100`.

`--bottom VALUE`

Real length of the bottom blue-marker side. Default: `100`.

`--left VALUE`

Real length of the left blue-marker side. Default: `100`.

`--unit UNIT`

Display unit for all measured distances. Default: `cm`.

`--interactive`

Prompt for all four side lengths in the terminal instead of using the defaults
or command-line values.

## Environment Variables

`RTSP_URL`

Camera stream URL. If not provided, the script uses the hard-coded default in
`DistanceDetection.py`.

`MIN_AREA`

Minimum contour area accepted as a detected blob. Default: `300`.

Increase this if noise is being detected as markers. Decrease it if valid
objects are too small and are being ignored.

`TARGET_FPS`

Maximum processing FPS. Default: `30`.

`PROCESS_WIDTH`

Optional frame width for processing. Default: `0`, which disables resizing.

For example, `PROCESS_WIDTH=960` can reduce CPU usage. Calibration and
measurement still work because all detection happens on the resized frame
consistently.

`JPEG_QUALITY`

JPEG encoding quality for the browser stream. Default: `95`.

`DISTANCE_UNIT`

Default unit label. This is used only if `--unit` is not supplied.

Example:

```bash
DISTANCE_UNIT=m python3 "Distance detection/DistanceDetection.py" --top 1 --right 1 --bottom 1 --left 1
```

## Color Detection

The program uses HSV color ranges.

Blue is reserved for calibration markers and uses several ranges to handle
different lighting conditions:

- Normal blue marker tone.
- Lighter marker tone under sunlight.
- Darker marker tone.

Pink and green are detected as the measured objects.

If the detections are unstable, use `colorPicker.py` to inspect HSV values and
then adjust the ranges in `DistanceDetection.py`.

Run:

```bash
python3 "Distance detection/colorPicker.py"
```

## What Happens If A Marker Disappears?

Before calibration:

- The program waits until four blue markers are visible.
- Green-pink distance is shown as `calibration needed`.

After calibration:

- The saved homography remains in memory.
- Blue markers are no longer required on every frame.
- Green-pink distance continues to be displayed.

After camera movement:

- The saved homography may be wrong.
- Restart the script with the blue markers visible to recalibrate.

## Troubleshooting

No video appears:

- Check that the RTSP URL is correct.
- Confirm the camera is reachable from the Jetson.
- Check terminal logs for `Could not open RTSP stream`.

Calibration never starts:

- Make sure all four blue markers are visible at startup.
- Check that the blue marker color matches `BLUE_MARKER_RANGES`.
- Lower `MIN_AREA` if the markers are detected as too small.

Wrong green-pink distance:

- Confirm the blue marker side lengths passed in the terminal match the real
  measured side lengths.
- Confirm the four blue markers are on the same floor plane as the green and
  pink objects.
- Restart calibration if the camera moved.
- Check that the detected green and pink centers correspond to the object points
  you intend to measure.

Extra blue objects are detected:

- The program uses the four largest blue blobs during startup calibration.
- Remove unrelated blue objects from the frame when starting the program.
- Tighten `BLUE_MARKER_RANGES` if other objects are still detected.

The stream has too much delay:

- Lower `TARGET_FPS`.
- Set `PROCESS_WIDTH` to a smaller value such as `960`.
- Keep the latest-frame worker enabled; it already drops old buffered frames.

## Dependencies

The script uses:

- Python 3
- OpenCV (`cv2`)
- NumPy
- Flask

If the packages are missing, install them in your Python environment:

```bash
python3 -m pip install flask numpy opencv-python
```

On Jetson systems, OpenCV may already be installed through the system image or
JetPack environment.
