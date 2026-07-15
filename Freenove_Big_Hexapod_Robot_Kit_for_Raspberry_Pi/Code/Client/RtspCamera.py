# -*- coding: utf-8 -*-
import argparse
import getpass
import os
import time
from urllib.parse import quote, urlencode

import cv2


COMMON_RTSP_PATHS = [
    "/Streaming/Channels/101",
    "/Streaming/Channels/102",
    "/cam/realmonitor",
    "/live",
    "/live/ch00_0",
    "/live/ch00_1",
    "/h264Preview_01_main",
    "/h264Preview_01_sub",
]


def bool_text(value):
    return "yes" if value else "no"


def build_rtsp_url(args, username, password, path=None, query=None):
    safe_user = quote(username, safe="") if username else ""
    safe_password = quote(password, safe="") if password else ""
    auth = ""
    if safe_user:
        auth = safe_user
        if safe_password:
            auth += ":" + safe_password
        auth += "@"

    selected_path = path if path is not None else args.path
    if not selected_path.startswith("/"):
        selected_path = "/" + selected_path

    query_params = {}
    if args.channel is not None:
        query_params["channel"] = args.channel
    if args.subtype is not None:
        query_params["subtype"] = args.subtype
    if args.query:
        for item in args.query:
            key, _, value = item.partition("=")
            if key:
                query_params[key] = value
    if query:
        query_params.update(query)

    url = "rtsp://{auth}{host}:{port}{path}".format(
        auth=auth,
        host=args.ip,
        port=args.port,
        path=selected_path,
    )
    if query_params:
        url += "?" + urlencode(query_params)
    return url


def redact_url(url):
    prefix = "rtsp://"
    if not url.startswith(prefix) or "@" not in url:
        return url
    auth_end = url.find("@")
    return prefix + "***:***" + url[auth_end:]


def configure_ffmpeg_options(args):
    options = {
        "rtsp_transport": args.transport,
        "stimeout": str(args.timeout_ms * 1000),
        "rw_timeout": str(args.timeout_ms * 1000),
        "max_delay": str(args.max_delay_us),
        "reorder_queue_size": str(args.reorder_queue_size),
        "buffer_size": str(args.socket_buffer),
        "fflags": "nobuffer",
        "flags": "low_delay",
        "probesize": str(args.probe_size),
        "analyzeduration": str(args.analyze_duration_us),
    }
    if args.extra_ffmpeg:
        for item in args.extra_ffmpeg:
            key, _, value = item.partition("=")
            if key and value:
                options[key] = value

    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "|".join(
        key + ";" + value for key, value in options.items()
    )
    return options


def open_capture(url, args):
    capture = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, args.frame_buffer)
    if args.width:
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    if args.height:
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if args.fps:
        capture.set(cv2.CAP_PROP_FPS, args.fps)
    return capture


def probe_paths(args, username, password):
    print("Probing common RTSP paths. This may take a few seconds per path.")
    for path in COMMON_RTSP_PATHS:
        query = None
        if path == "/cam/realmonitor":
            query = {"channel": args.channel or "1", "subtype": args.subtype or "0"}
        url = build_rtsp_url(args, username, password, path=path, query=query)
        capture = open_capture(url, args)
        ok, frame = capture.read()
        width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
        height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
        capture.release()
        print(
            "{status} {url} {size}".format(
                status="OK " if ok and frame is not None else "FAIL",
                url=redact_url(url),
                size="{}x{}".format(width, height) if width and height else "",
            )
        )


def draw_status(frame, frame_count, measured_fps, dropped_reads, url):
    text = "fps={:.1f} frames={} dropped={} q=quit s=snapshot".format(
        measured_fps,
        frame_count,
        dropped_reads,
    )
    cv2.putText(
        frame,
        text,
        (12, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        redact_url(url),
        (12, frame.shape[0] - 14),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return frame


def stream(args, username, password):
    url = args.url or build_rtsp_url(args, username, password)
    options = configure_ffmpeg_options(args)

    print("RTSP URL: " + redact_url(url))
    print("Transport: " + args.transport)
    print("Low-latency options: " + ", ".join(sorted(options.keys())))
    print("Display: " + bool_text(not args.no_display))

    capture = None
    frame_count = 0
    dropped_reads = 0
    last_stats_time = time.time()
    stats_frames = 0
    measured_fps = 0.0
    last_reconnect = 0.0

    try:
        while True:
            if capture is None or not capture.isOpened():
                capture = open_capture(url, args)
                if not capture.isOpened():
                    print("Could not open stream. Retrying in {:.1f}s...".format(args.reconnect_delay))
                    time.sleep(args.reconnect_delay)
                    continue
                print(
                    "Connected: {}x{} reported_fps={:.1f}".format(
                        int(capture.get(cv2.CAP_PROP_FRAME_WIDTH) or 0),
                        int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0),
                        capture.get(cv2.CAP_PROP_FPS) or 0.0,
                    )
                )

            ok, frame = capture.read()
            if not ok or frame is None:
                dropped_reads += 1
                now = time.time()
                if now - last_reconnect >= args.reconnect_delay:
                    print("Frame read failed. Reconnecting...")
                    capture.release()
                    capture = None
                    last_reconnect = now
                continue

            frame_count += 1
            stats_frames += 1
            now = time.time()
            elapsed = now - last_stats_time
            if elapsed >= args.stats_interval:
                measured_fps = stats_frames / elapsed
                print(
                    "stats frames={} fps={:.1f} dropped={}".format(
                        frame_count,
                        measured_fps,
                        dropped_reads,
                    )
                )
                last_stats_time = now
                stats_frames = 0

            if args.no_display:
                continue

            if args.scale != 1.0:
                frame = cv2.resize(frame, None, fx=args.scale, fy=args.scale)
            frame = draw_status(frame, frame_count, measured_fps, dropped_reads, url)
            cv2.imshow("RTSP WiFi Camera", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s"):
                filename = "rtsp_snapshot_{}.jpg".format(time.strftime("%Y%m%d_%H%M%S"))
                cv2.imwrite(filename, frame)
                print("Saved " + filename)
    except KeyboardInterrupt:
        print("Stopping stream.")
    finally:
        if capture is not None:
            capture.release()
        if not args.no_display:
            cv2.destroyAllWindows()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Connect to a WiFi camera through RTSP using OpenCV/FFmpeg."
    )
    parser.add_argument("--ip", help="Camera IP address or hostname.")
    parser.add_argument("--port", type=int, default=554, help="RTSP port. Default: 554.")
    parser.add_argument("--user", help="Camera account username.")
    parser.add_argument("--password", help="Camera password. If omitted, you will be prompted.")
    parser.add_argument(
        "--url",
        help="Full RTSP URL. If provided, --ip/--user/--password/--path are ignored for streaming.",
    )
    parser.add_argument(
        "--path",
        default="/Streaming/Channels/101",
        help="RTSP path. Try /Streaming/Channels/102 for a lower-latency substream.",
    )
    parser.add_argument("--channel", help="Optional channel query parameter for cameras that need it.")
    parser.add_argument("--subtype", help="Optional subtype query parameter: often 0=main, 1=substream.")
    parser.add_argument(
        "--query",
        action="append",
        help="Extra URL query parameter as key=value. Can be repeated.",
    )
    parser.add_argument(
        "--probe",
        action="store_true",
        help="Test common RTSP paths and report which ones return frames.",
    )
    parser.add_argument(
        "--transport",
        choices=["tcp", "udp", "udp_multicast", "http"],
        default="tcp",
        help="RTSP transport. tcp is usually smoother on WiFi; udp can reduce latency on clean networks.",
    )
    parser.add_argument("--timeout-ms", type=int, default=5000, help="Open/read timeout in milliseconds.")
    parser.add_argument("--max-delay-us", type=int, default=500000, help="FFmpeg max demux delay.")
    parser.add_argument("--reorder-queue-size", type=int, default=0, help="Use 0 for low latency.")
    parser.add_argument("--socket-buffer", type=int, default=102400, help="FFmpeg socket buffer bytes.")
    parser.add_argument("--probe-size", type=int, default=32768, help="FFmpeg probe size bytes.")
    parser.add_argument("--analyze-duration-us", type=int, default=0, help="FFmpeg analyze duration.")
    parser.add_argument(
        "--extra-ffmpeg",
        action="append",
        help="Extra OPENCV_FFMPEG_CAPTURE_OPTIONS value as key=value. Can be repeated.",
    )
    parser.add_argument("--frame-buffer", type=int, default=1, help="OpenCV capture buffer size.")
    parser.add_argument("--width", type=int, help="Requested capture width if the camera honors it.")
    parser.add_argument("--height", type=int, help="Requested capture height if the camera honors it.")
    parser.add_argument("--fps", type=int, help="Requested FPS if the camera honors it.")
    parser.add_argument("--scale", type=float, default=1.0, help="Display scale factor.")
    parser.add_argument("--no-display", action="store_true", help="Read frames and print stats without a window.")
    parser.add_argument("--stats-interval", type=float, default=2.0, help="Seconds between FPS reports.")
    parser.add_argument("--reconnect-delay", type=float, default=1.5, help="Seconds before reconnect attempts.")
    return parser.parse_args()


def main():
    args = parse_args()
    if not args.url and not args.ip:
        raise SystemExit("Use --ip or provide a full --url.")

    username = args.user or ""
    password = args.password
    if not args.url and username and password is None:
        password = getpass.getpass("Camera password: ")
    if password is None:
        password = ""

    configure_ffmpeg_options(args)
    if args.probe:
        if args.url:
            raise SystemExit("--probe builds test URLs, so use --ip instead of --url.")
        probe_paths(args, username, password)
        return

    stream(args, username, password)


if __name__ == "__main__":
    main()
