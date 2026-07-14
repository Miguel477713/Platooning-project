import curses
import time
from onvif import ONVIFCamera

CAMERA_IP = "10.0.7.10"
ONVIF_PORT = 2020
USERNAME = "oliversina134"
PASSWORD = "12345678"

SPEED = 0.5


def Main(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.clear()

    cam = ONVIFCamera(CAMERA_IP, ONVIF_PORT, USERNAME, PASSWORD)
    media = cam.create_media_service()
    ptz = cam.create_ptz_service()

    profile = media.GetProfiles()[0]

    stdscr.addstr(0, 0, "ONVIF PTZ control")
    stdscr.addstr(2, 0, "Arrow keys = pan/tilt")
    stdscr.addstr(3, 0, "+ / -      = zoom in/out")
    stdscr.addstr(4, 0, "space      = stop")
    stdscr.addstr(5, 0, "q          = quit")
    stdscr.refresh()

    def Move(x=0, y=0, zoom=0):
        req = ptz.create_type("ContinuousMove")
        req.ProfileToken = profile.token
        req.Velocity = {
            "PanTilt": {"x": x, "y": y},
            "Zoom": {"x": zoom}
        }
        ptz.ContinuousMove(req)

    def Stop():
        req = ptz.create_type("Stop")
        req.ProfileToken = profile.token
        req.PanTilt = True
        req.Zoom = True
        ptz.Stop(req)

    last_key = None

    try:
        while True:
            key = stdscr.getch()

            if key == ord("q"):
                Stop()
                break

            elif key == curses.KEY_UP:
                Move(0, SPEED, 0)
                last_key = "tilt up"

            elif key == curses.KEY_DOWN:
                Move(0, -SPEED, 0)
                last_key = "tilt down"

            elif key == curses.KEY_LEFT:
                Move(-SPEED, 0, 0)
                last_key = "pan left"

            elif key == curses.KEY_RIGHT:
                Move(SPEED, 0, 0)
                last_key = "pan right"

            elif key == ord("+"):
                Move(0, 0, SPEED)
                last_key = "zoom in"

            elif key == ord("-"):
                Move(0, 0, -SPEED)
                last_key = "zoom out"

            elif key == ord(" "):
                Stop()
                last_key = "Stop"

            if last_key:
                stdscr.addstr(7, 0, f"Last command: {last_key}      ")
                stdscr.refresh()

            time.sleep(0.05)

    finally:
        Stop()


curses.wrapper(Main)