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
FPS = 30
BITRATE = 500000  # 500 kbps


class SensorFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self):
        super().__init__()
        # Avoid videoconvert dependency by requesting NV12 directly.
        pipeline = (
            f"libcamerasrc ! video/x-raw,format=NV12,width={WIDTH},height={HEIGHT},"
            f"framerate={FPS}/1 ! "
            f"v4l2h264enc extra-controls=\"controls,video_bitrate={BITRATE};\" ! "
            "rtph264pay name=pay0 pt=96"
        )
        print(f"[sub_camera] Pipeline: {pipeline}")
        self.set_launch(pipeline)
        self.set_shared(True)
    
    def do_create_element(self, url):
        """Override to add error logging."""
        print(f"[sub_camera] Client requesting: {url.get_request_uri()}")
        try:
            element = super().do_create_element(url)
            if element:
                print("[sub_camera] Pipeline element created successfully")
            else:
                print("[sub_camera] ERROR: Pipeline element is None")
            return element
        except Exception as e:
            print(f"[sub_camera] ERROR creating pipeline element: {e}")
            import traceback
            traceback.print_exc()
            raise


class GstServer:
    def __init__(self):
        self.server = GstRtspServer.RTSPServer()
        mounts = self.server.get_mount_points()
        mounts.add_factory("/stream", SensorFactory())
        self.server.attach(None)
        print("[sub_camera] RTSP server started at rtsp://0.0.0.0:8554/stream")


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

    # videoconvert is an element factory (from videoconvertscale), not a plugin name.
    for elem in ["libcamerasrc", "v4l2h264enc", "rtph264pay"]:
        factory = Gst.ElementFactory.find(elem)
        if factory:
            print(f"[sub_camera] ✓ {elem} element found")
        else:
            print(f"[sub_camera] ✗ {elem} element NOT found")
    
    server = GstServer()
    loop = GLib.MainLoop()
    try:
        loop.run()
    except KeyboardInterrupt:
        print("[sub_camera] Shutting down")
        sys.exit(0)


if __name__ == "__main__":
    main()
