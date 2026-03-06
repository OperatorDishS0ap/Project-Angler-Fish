import shutil
import subprocess
import sys

# streaming parameters
WIDTH = 640
HEIGHT = 480
FPS = 20
BITRATE = 400000  # bps
RTSP_URL = "rtsp://0.0.0.0:8554/stream"
RTSP_DISPLAY_URL = "rtsp://<pi-ip>:8554/stream"


def _find_camera_tool() -> str:
    # Newer Raspberry Pi OS uses rpicam-vid; older uses libcamera-vid.
    for name in ("rpicam-vid", "libcamera-vid"):
        if shutil.which(name):
            return name
    raise RuntimeError("Missing required camera tool: rpicam-vid/libcamera-vid")


def _require_tool(name: str) -> None:
    if shutil.which(name) is None:
        raise RuntimeError(f"Missing required tool: {name}")


def _build_libcamera_cmd(camera_tool: str) -> list[str]:
    # libcamera-vid uses Pi hardware H.264 encoder.
    return [
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
        "-t",
        "0",
        "-o",
        "-",
    ]


def _build_ffmpeg_cmd() -> list[str]:
    # FFmpeg receives Annex-B H.264 and serves it via RTSP in listen mode.
    return [
        "ffmpeg",
        "-loglevel",
        "info",
        "-fflags",
        "nobuffer",
        "-flags",
        "low_delay",
        "-f",
        "h264",
        "-i",
        "-",
        "-an",
        "-c:v",
        "copy",
        "-muxdelay",
        "0.1",
        "-f",
        "rtsp",
        "-rtsp_transport",
        "tcp",
        "-listen",
        "1",
        RTSP_URL,
    ]


def main() -> None:
    try:
        camera_tool = _find_camera_tool()
        _require_tool("ffmpeg")
    except Exception as e:
        print(f"[sub_camera] ERROR: {e}")
        print("[sub_camera] Install: sudo apt install ffmpeg libcamera-apps rpicam-apps")
        sys.exit(1)

    cam_cmd = _build_libcamera_cmd(camera_tool)
    ff_cmd = _build_ffmpeg_cmd()
    print(f"[sub_camera] Using hardware H.264 via {camera_tool}")
    print(f"[sub_camera] RTSP server bind: {RTSP_URL}")
    print(f"[sub_camera] Connect from PC: {RTSP_DISPLAY_URL}")
    print(f"[sub_camera] ffmpeg cmd: {' '.join(ff_cmd)}")

    cam_proc = None
    ff_proc = None
    try:
        cam_proc = subprocess.Popen(cam_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        ff_proc = subprocess.Popen(ff_cmd, stdin=cam_proc.stdout, stderr=subprocess.PIPE, text=True)

        # Ensure only ffmpeg owns the camera stdout pipe.
        if cam_proc.stdout is not None:
            cam_proc.stdout.close()

        # Wait until one process exits.
        ff_rc = ff_proc.wait()
        if ff_proc.stderr is not None:
            ff_err = ff_proc.stderr.read().strip()
            if ff_err:
                print(f"[sub_camera] ffmpeg stderr:\n{ff_err}")
        cam_rc = cam_proc.poll()
        print(f"[sub_camera] ffmpeg exited rc={ff_rc}, libcamera rc={cam_rc}")
    except KeyboardInterrupt:
        print("[sub_camera] Shutting down")
    finally:
        for proc in (ff_proc, cam_proc):
            if proc is not None and proc.poll() is None:
                proc.terminate()
        for proc in (ff_proc, cam_proc):
            if proc is not None:
                try:
                    proc.wait(timeout=2)
                except Exception:
                    proc.kill()


if __name__ == "__main__":
    main()
