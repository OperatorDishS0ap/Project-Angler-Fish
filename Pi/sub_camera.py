"""RTSP server for H.264 video using GStreamer.

Replaces the previous UDP-based streaming approach. A hardware H.264
encoder (4l2h264enc) is driven directly from the Pi camera and streamed
over RTSP using gst-rtsp-server. Clients can connect to

tsp://<pi-ip>:8554/stream.

Dependencies:
    python3-gi
    gir1.2-gst-rtsp-server-1.0
    gstreamer1.0-plugins-base, gstreamer1.0-plugins-good, gstreamer1.0-plugins-bad

Usage is simple: run this script and then open the RTSP URL from a PC or
mobile device with an H.264-capable player or via OpenCV/FFmpeg.
"""

import sys

import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstRtspServer", "1.0")
from gi.repository import Gst, GstRtspServer, GObject

# initialize GStreamer
Gst.init(None)

# streaming parameters
WIDTH = 640
HEIGHT = 480
FPS = 30
BITRATE = 500000  # 500 kbps


class SensorFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self):
        super().__init__()
        pipeline = (
            f"v4l2src device=/dev/video0 ! video/x-raw,width={WIDTH},height={HEIGHT},"
            f"framerate={FPS}/1 ! videoconvert ! v4l2h264enc bitrate={BITRATE} ! "
            "rtph264pay name=pay0 pt=96"
        )
        self.set_launch(pipeline)
        self.set_shared(True)


class GstServer:
    def __init__(self):
        self.server = GstRtspServer.RTSPServer()
        mounts = self.server.get_mount_points()
        mounts.add_factory("/stream", SensorFactory())
        self.server.attach(None)
        print("[sub_camera] RTSP server started at rtsp://0.0.0.0:8554/stream")


def main():
    server = GstServer()
    loop = GObject.MainLoop()
    try:
        loop.run()
    except KeyboardInterrupt:
        print("[sub_camera] Shutting down")
        sys.exit(0)


if __name__ == "__main__":
    main()
