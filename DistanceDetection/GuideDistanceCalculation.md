# Distance Detection Guide

This guide explains how `DistanceDetection.py` measures the real-world
distance between pink and green objects. It uses four user-selected points on
the video instead of searching for blue calibration markers.

## What The Program Does

The program reads the RTSP camera stream, detects the pink and green objects
with HSV ranges, and displays their measured distance on the video. A
homography maps image pixels onto the floor plane using a rectangle selected
by the user.

The browser stream is available at:

```text
http://<jetson-ip>:5000/
```

On the Jetson itself, use `http://localhost:5000/`.

## Start The Program

Use the existing saved calibration:

```bash
python3 /Platooning-project/DistanceDetection/DistanceDetection.py
```

Create a new calibration:

```bash
python3 /Platooning-project/DistanceDetection/DistanceDetection.py --calibrate
```

Use `--calibrate` whenever the camera, camera angle, or floor measurement
area changes. A normal launch loads the previous calibration and does not
allow it to be changed from the browser.

If no saved calibration exists, the program automatically opens calibration
mode and asks for it in the browser.

## Browser Calibration

Open the stream page and click the four corners of the measured floor
rectangle in this exact order:

1. Top-left
2. Top-right
3. Bottom-right
4. Bottom-left

Then enter the real measured lengths of the top, right, bottom, and left sides
in one consistent unit, such as `cm`, and select **Apply calibration**.

For a 100 cm by 100 cm square, enter `100` for all four sides and leave the
unit as `cm`.

The stream fills the available browser width. Select **Fullscreen video** to
expand the video further while choosing corners or observing measurements.

## Saved Calibration

After applying calibration, the four clicked points and the entered side
lengths are stored at:

```text
~/.local/share/distance_detection2/calibration.json
```

The next normal launch restores this calibration automatically. To use a
different location, set `CALIBRATION_FILE` before starting the program:

```bash
CALIBRATION_FILE=/tmp/floor-calibration.json \
python3 /Platooning-project/DistanceDetection/DistanceDetection.py --calibrate
```

## How Distance Is Calculated

The camera views the floor at an angle, so one fixed pixel-to-centimeter ratio
is not accurate across the image. The four clicked corners and their physical
side lengths define a perspective transform, or homography.

For every frame, the program:

1. Detects the largest valid pink and green blobs.
2. Finds the center point of each blob.
3. Projects both center points into the calibrated floor plane.
4. Calculates the Euclidean distance between those projected points.
5. Draws the green-pink distance and the active calibration rectangle on the
   video stream.

All four clicked points and both colored objects must be on the same flat floor
plane for the displayed measurement to be meaningful.

## Environment Variables

`RTSP_URL`

The camera stream URL. If omitted, the script uses its built-in RTSP URL.

`MIN_AREA`

Minimum contour area for a pink or green object. Default: `300`. Increase it
to ignore small color noise; decrease it when the real object is too small.

`TARGET_FPS`

Maximum processing rate. Default: `30`.

`PROCESS_WIDTH`

Optional frame width used for processing. Default: `0`, which leaves the
original size unchanged. For example, `PROCESS_WIDTH=960` reduces CPU use.

`JPEG_QUALITY`

JPEG quality for the browser video stream. Default: `95`.

`CALIBRATION_FILE`

Optional path for the saved click calibration. By default it is stored under
`~/.local/share/distance_detection2/`.

## Color Detection

Pink and green objects are detected with HSV ranges defined in
`DistanceDetection.py`. Blue markers are not used by this script.

Use `colorPicker.py` to inspect HSV values when the pink or green object is
not detected reliably:

```bash
python3 /Platooning-project/DistanceDetection/colorPicker.py
```

## Troubleshooting

No video appears:

- Verify `RTSP_URL` and network access to the camera.
- Check the terminal for `Could not open RTSP stream`.

Calibration mode does not appear:

- Start the program with `--calibrate`.
- If using a saved calibration intentionally, this is expected: the browser
  shows the stored transform and hides calibration controls.

Wrong distance:

- Run with `--calibrate` and click the floor rectangle corners again.
- Verify the click order is top-left, top-right, bottom-right, bottom-left.
- Enter the real side lengths in the same unit.
- Keep the selected rectangle and colored objects on the same floor plane.
- Recalibrate after moving or rotating the camera.

Pink or green is not detected:

- Confirm the object is large enough for `MIN_AREA`.
- Check the HSV values with `colorPicker.py`.
- Adjust the pink or green HSV ranges in `DistanceDetection.py`.
