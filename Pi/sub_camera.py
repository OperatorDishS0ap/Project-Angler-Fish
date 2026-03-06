"""RTSP server for H.264 video using GStreamer.

Uses libcamerasrc on Raspberry Pi and serves an RTSP stream at:
rtsp://<pi-ip>:8554/stream

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
from gi.repository import Gst, GstRtspServer, GLib

# streaming parameters
WIDTH = 640
HEIGHT = 480
FPS = 20
BITRATE = 400000  # 400 kbps


class SensorFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self, pipeline: str):
        super().__init__()
        print(f"[sub_camera] Pipeline: {pipeline}")
        self.set_launch(pipeline)
        self.set_shared(True)


class GstServer:
    def __init__(self, pipeline: str):
        self.server = GstRtspServer.RTSPServer()
        mounts = self.server.get_mount_points()
        mounts.add_factory("/stream", SensorFactory(pipeline))
        self.server.attach(None)
        print("[sub_camera] RTSP server started at rtsp://0.0.0.0:8554/stream")


def _choose_pipeline() -> str:
    has_v4l2 = Gst.ElementFactory.find("v4l2h264enc") is not None

    if has_v4l2:
        print("[sub_camera] Using v4l2h264enc (hardware)")
        # Conservative hardware path: avoid v4l2convert import issues and force mmap io-mode.
        return (
            f"libcamerasrc ! video/x-raw,format=NV12,width={WIDTH},height={HEIGHT},framerate={FPS}/1 ! "
            "queue max-size-buffers=6 leaky=downstream ! "
            f"v4l2h264enc output-io-mode=mmap capture-io-mode=mmap "
            f"extra-controls=\"controls,video_bitrate={BITRATE},video_bitrate_mode=1,repeat_sequence_header=1,h264_i_frame_period={FPS};\" ! "
            "h264parse config-interval=1 ! "
            "rtph264pay name=pay0 pt=96 config-interval=1"
        )

    raise RuntimeError("No hardware H.264 encoder found (v4l2h264enc)")


def main():
    import os
    os.environ['GST_DEBUG'] = '2'  # 0=none, 1=ERROR, 2=WARNING, 3=INFO, 4=DEBUG
    Gst.init(None)
    
    print("[sub_camera] Checking GStreamer components...")
    registry = Gst.Registry.get()

    required_plugins = ['libcamera', 'video4linux2', 'rtp']
    for plugin_name in required_plugins:
        plugin = registry.find_plugin(plugin_name)
        if plugin:
            print(f"[sub_camera] ✓ {plugin_name} plugin found")
        else:
            print(f"[sub_camera] ✗ {plugin_name} plugin NOT found")

    for elem in ["libcamerasrc", "queue", "v4l2h264enc", "h264parse", "rtph264pay"]:
        factory = Gst.ElementFactory.find(elem)
        if factory:
            print(f"[sub_camera] ✓ {elem} element found")
        else:
            print(f"[sub_camera] ✗ {elem} element NOT found")

    try:
        pipeline = _choose_pipeline()
    except Exception as e:
        print(f"[sub_camera] ERROR: {e}")
        print("[sub_camera] Install/verify hardware stack: sudo apt install gstreamer1.0-plugins-good gstreamer1.0-libcamera")
        sys.exit(1)

    server = GstServer(pipeline)
    loop = GLib.MainLoop()
    try:
        loop.run()
    except KeyboardInterrupt:
        print("[sub_camera] Shutting down")
        sys.exit(0)


if __name__ == "__main__":
    main()
