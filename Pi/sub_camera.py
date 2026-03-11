import os
import shutil
import subprocess
import sys

import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstRtspServer", "1.0")
from gi.repository import Gst, GstRtspServer, GLib

# streaming parameters
WIDTH = 720
HEIGHT = 480
FPS = 30
BITRATE = 3000000  # bps
VFLIP = True
HFLIP = False
RTSP_DISPLAY_URL = "rtsp://<pi-ip>:8554/stream"
FIFO_PATH = "/tmp/anglerfish_cam.h264"


def _find_camera_tool() -> str:
    for name in ("rpicam-vid", "libcamera-vid"):
        if shutil.which(name):
            return name
    raise RuntimeError("Missing required camera tool: rpicam-vid/libcamera-vid")


def _prepare_fifo() -> None:
    if os.path.exists(FIFO_PATH):
        os.remove(FIFO_PATH)
    os.mkfifo(FIFO_PATH)


def _build_camera_cmd(camera_tool: str) -> list[str]:
    # Hardware H.264 on Raspberry Pi camera stack.
    cmd = [
        camera_tool,
        "--nopreview",
        "--inline",
        "--codec",
        "h264",
        "--width",
        str(WIDTH),
        "--height",
        str(HEIGHT),
        "--framerate",
        str(FPS),
        "--bitrate",
        str(BITRATE),
        "--intra",
        str(max(1, FPS)),
        "--flush",
        "-t",
        "0",
        "-o",
        FIFO_PATH,
    ]

    if VFLIP:
        cmd.append("--vflip")
    if HFLIP:
        cmd.append("--hflip")

    return cmd


class StreamFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self):
        super().__init__()
        launch = (
            f"filesrc location={FIFO_PATH} do-timestamp=true ! "
            "h264parse config-interval=1 ! "
            "rtph264pay name=pay0 pt=96 config-interval=1"
        )
        print(f"[sub_camera] RTSP pipeline: {launch}")
        self.set_launch(launch)
        self.set_shared(True)


def main() -> None:
    try:
        camera_tool = _find_camera_tool()
    except Exception as e:
        print(f"[sub_camera] ERROR: {e}")
        print("[sub_camera] Install: sudo apt install libcamera-apps rpicam-apps gir1.2-gst-rtsp-server-1.0")
        sys.exit(1)

    _prepare_fifo()
    Gst.init(None)

    cam_cmd = _build_camera_cmd(camera_tool)
    print(f"[sub_camera] Using hardware H.264 via {camera_tool}")
    print(f"[sub_camera] Connect from PC: {RTSP_DISPLAY_URL}")
    print(f"[sub_camera] Camera cmd: {' '.join(cam_cmd)}")

    cam_proc = None
    try:
        # Start RTSP server first so clients can connect immediately.
        server = GstRtspServer.RTSPServer()
        mounts = server.get_mount_points()
        mounts.add_factory("/stream", StreamFactory())
        server.attach(None)
        print("[sub_camera] RTSP server started on 0.0.0.0:8554")

        # Start camera process that writes H.264 Annex-B to FIFO.
        cam_proc = subprocess.Popen(cam_cmd)

        loop = GLib.MainLoop()
        loop.run()
    except KeyboardInterrupt:
        print("[sub_camera] Shutting down")
    finally:
        if cam_proc is not None and cam_proc.poll() is None:
            cam_proc.terminate()
            try:
                cam_proc.wait(timeout=2)
            except Exception:
                cam_proc.kill()
        if os.path.exists(FIFO_PATH):
            os.remove(FIFO_PATH)


if __name__ == "__main__":
    main()
