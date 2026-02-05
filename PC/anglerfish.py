import json
import os
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

import cv2
from PIL import Image, ImageTk  # pillow
import paramiko

from camera import CameraClient
from controller import XboxControllerReader, MotorUdpSender
from sensor import SensorUdpReceiver


CONFIG_PATH = os.path.join(os.path.expanduser("~"), ".anglerfish_config.json")


def load_config():
    try:
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {}


def save_config(cfg):
    try:
        with open(CONFIG_PATH, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2)
    except Exception:
        pass


class StatusBox(ttk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self.var = tk.StringVar(value="Disconnected")
        self.label = tk.Label(self, textvariable=self.var, width=18, relief="groove")
        self.label.pack(fill="x")
        self.set_connected(False)

    def set_connected(self, connected: bool, text: str | None = None):
        self.var.set(text if text is not None else ("Connected" if connected else "Disconnected"))
        self.label.configure(bg=("light green" if connected else "tomato"))

    def set_text(self, text: str):
        self.var.set(text)


class AnglerFishApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Project Angler Fish")
        self.geometry("1200x700")

        cfg = load_config()
        self.pi_ip = tk.StringVar(value=cfg.get("ip", "192.168.137.64"))
        self.pi_user = tk.StringVar(value=cfg.get("user", "pi"))
        self.pi_pass = tk.StringVar(value=cfg.get("pass", "raspberry"))
        self.save_creds = tk.BooleanVar(value=cfg.get("save", True))
        self.show_pass = tk.BooleanVar(value=False)

        self.running = False
        self.start_time = None

        self.camera_client = None
        self.sensor_rx = None
        self.motor_sender = None
        self.controller = None

        self.page1 = ttk.Frame(self)
        self.page2 = ttk.Frame(self)

        self._build_page1()
        self._build_page2()

        self.page1.pack(fill="both", expand=True)
        self._ui_loop()

    # SSH helpers
    def _ssh_connect(self):
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect(self.pi_ip.get().strip(), username=self.pi_user.get().strip(),
                       password=self.pi_pass.get(), timeout=8)
        return client

    def _ssh_run(self, client, cmd: str):
        stdin, stdout, stderr = client.exec_command(cmd)
        _ = stdout.read()
        err = stderr.read()
        return _.decode("utf-8", errors="ignore"), err.decode("utf-8", errors="ignore")

    # Page 1
    def _build_page1(self):
        f = self.page1

        ttk.Label(f, text="Project Angler Fish", font=("Segoe UI", 20, "bold")).pack(pady=12)

        form = ttk.Frame(f)
        form.pack(pady=10)

        ttk.Label(form, text="IP Address:").grid(row=0, column=0, sticky="e", padx=6, pady=6)
        ttk.Entry(form, textvariable=self.pi_ip, width=24).grid(row=0, column=1, sticky="w", padx=6, pady=6)

        ttk.Label(form, text="Username:").grid(row=1, column=0, sticky="e", padx=6, pady=6)
        ttk.Entry(form, textvariable=self.pi_user, width=24).grid(row=1, column=1, sticky="w", padx=6, pady=6)

        ttk.Label(form, text="Password:").grid(row=2, column=0, sticky="e", padx=6, pady=6)
        self.pass_entry = ttk.Entry(form, textvariable=self.pi_pass, width=24, show="*")
        self.pass_entry.grid(row=2, column=1, sticky="w", padx=6, pady=6)

        ttk.Checkbutton(form, text="Show password", variable=self.show_pass, command=self._toggle_pass).grid(
            row=3, column=1, sticky="w", padx=6, pady=2
        )
        ttk.Checkbutton(form, text="Save IP/User/Password", variable=self.save_creds).grid(
            row=4, column=1, sticky="w", padx=6, pady=2
        )

        self.status1 = StatusBox(f)
        self.status1.pack(pady=10)

        btns = ttk.Frame(f)
        btns.pack(pady=10)

        ttk.Button(btns, text="Update Submarine", command=self._on_update_sub).grid(row=0, column=0, padx=10)
        ttk.Button(btns, text="Start Submarine", command=self._on_start_sub).grid(row=0, column=1, padx=10)

        ttk.Label(f, text=f"Config file: {CONFIG_PATH}", foreground="gray").pack(pady=6)

    def _toggle_pass(self):
        self.pass_entry.configure(show="" if self.show_pass.get() else "*")

    def _maybe_save_creds(self):
        if self.save_creds.get():
            save_config({"ip": self.pi_ip.get(), "user": self.pi_user.get(), "pass": self.pi_pass.get(), "save": True})
        else:
            save_config({"ip": self.pi_ip.get(), "user": self.pi_user.get(), "save": False})

    def _on_update_sub(self):
        self._maybe_save_creds()

        def worker():
            try:
                self.status1.set_connected(True, "Connected")
                client = self._ssh_connect()
                self.status1.set_text("Updating Files")
                self._ssh_run(client, "cd ~/Project-Angler-Fish && git pull")
                time.sleep(6)
                client.close()
                self.status1.set_connected(False, "Disconnected")
            except Exception as e:
                self.status1.set_connected(False, "Disconnected")
                messagebox.showerror("Update failed", str(e))

        threading.Thread(target=worker, daemon=True).start()

    def _on_start_sub(self):
        self._maybe_save_creds()

        def worker():
            try:
                self.status1.set_connected(True, "Connected")
                client = self._ssh_connect()
                cmd = "sudo pigpiod"
                self._ssh_run(client, cmd)
                cmd = "nohup python3 /home/pi/Project-Angler-Fish/Pi/sub_launcher.py > launcher.log 2>&1 &"
                self._ssh_run(client, cmd)
                client.close()
                self.status1.set_connected(False, "Disconnected")
                time.sleep(2)
                self._start_runtime_clients()
                self._show_page2()
            except Exception as e:
                self.status1.set_connected(False, "Disconnected")
                messagebox.showerror("Start failed", str(e))

        threading.Thread(target=worker, daemon=True).start()

    # Page 2
    def _build_page2(self):
        f = self.page2

        top = ttk.Frame(f)
        top.pack(fill="x", pady=6)

        ttk.Label(top, text="Project Angler Fish - Live", font=("Segoe UI", 16, "bold")).pack(side="left", padx=10)
        self.status2 = StatusBox(top)
        self.status2.pack(side="left", padx=10)
        ttk.Button(top, text="Shutdown Submarine", command=self._on_shutdown).pack(side="right", padx=10)

        body = ttk.Frame(f)
        body.pack(fill="both", expand=True)

        # Grid layout prevents the video widget from pushing/covering the right panel.
        body.columnconfigure(0, weight=1)
        body.columnconfigure(1, weight=0)
        body.rowconfigure(0, weight=1)

        video_frame = ttk.Frame(body)
        video_frame.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
        video_frame.columnconfigure(0, weight=1)
        video_frame.rowconfigure(0, weight=1)

        self.video_label = tk.Label(video_frame, text="Waiting for video...", bg="black", fg="white")
        self.video_label.grid(row=0, column=0, sticky="nsew")

        side = ttk.Frame(body, width=320)
        side.grid(row=0, column=1, sticky="ns", padx=10, pady=10)
        side.grid_propagate(False)

        self.battery_var = tk.StringVar(value="Battery: -- V")
        self.depth_var = tk.StringVar(value="Depth: --")
        self.press_var = tk.StringVar(value="Pressure: --")
        self.temp_var = tk.StringVar(value="Temps: Pi -- C | Env -- C")
        self.timer_var = tk.StringVar(value="Timer: 00:00")

        for v in [self.battery_var, self.depth_var, self.press_var, self.temp_var, self.timer_var]:
            ttk.Label(side, textvariable=v, font=("Segoe UI", 11)).pack(anchor="w", pady=4)

        ttk.Separator(side, orient="horizontal").pack(fill="x", pady=8)

        self.throttle_var = tk.StringVar(value="throttle: 0")
        self.yaw_var = tk.StringVar(value="yaw: 0")
        self.pitch_var = tk.StringVar(value="pitch: 0")
        self.roll_var = tk.StringVar(value="roll: 0")
        self.a_flag_var = tk.StringVar(value="a_flag: 0")

        ttk.Label(side, text="Vectoring", font=("Segoe UI", 12, "bold")).pack(anchor="w", pady=4)
        for v in [self.throttle_var, self.yaw_var, self.pitch_var, self.roll_var, self.a_flag_var]:
            ttk.Label(side, textvariable=v, font=("Segoe UI", 11)).pack(anchor="w", pady=2)

        ttk.Separator(side, orient="horizontal").pack(fill="x", pady=8)

        self.armed_var = tk.StringVar(value="DISARMED")
        self.armed_label = tk.Label(side, textvariable=self.armed_var, font=("Segoe UI", 14, "bold"), 
                                     relief="sunken", pady=8)
        self.armed_label.pack(anchor="w", fill="x", pady=4)
        self._update_armed_display(False)

    def _update_armed_display(self, armed: bool):
        """Update ARMED status indicator visual."""
        if armed:
            self.armed_var.set("ARMED")
            self.armed_label.configure(bg="#00ff00", fg="black")
        else:
            self.armed_var.set("DISARMED")
            self.armed_label.configure(bg="#ff6b6b", fg="white")

    def _show_page2(self):
        self.page1.pack_forget()
        self.page2.pack(fill="both", expand=True)

    def _show_page1(self):
        self.page2.pack_forget()
        self.page1.pack(fill="both", expand=True)

    def _start_runtime_clients(self):
        self.start_time = time.time()
        self.running = True

        self.camera_client = CameraClient(self.pi_ip.get().strip(), port=8000, reconnect=True)
        self.camera_client.start()

        self.sensor_rx = SensorUdpReceiver(listen_port=9100)
        self.sensor_rx.start()

        try:
            self.controller = XboxControllerReader()
        except Exception as e:
            messagebox.showwarning("Controller", f"Controller not ready: {e}")
            self.controller = None

        self.motor_sender = MotorUdpSender(self.pi_ip.get().strip(), pi_port=9000, rate_hz=30.0)
        self.motor_sender.start()

    def _stop_runtime_clients(self):
        self.running = False
        if self.camera_client:
            self.camera_client.stop()
        if self.sensor_rx:
            self.sensor_rx.stop()
        if self.motor_sender:
            self.motor_sender.stop()

        self.camera_client = None
        self.sensor_rx = None
        self.motor_sender = None
        self.controller = None
        self.start_time = None

    def _on_shutdown(self):
        def worker():
            try:
                self.status2.set_connected(True, "Connected")
                self.status2.set_text("Terminating")
                client = self._ssh_connect()
                for name in [
                    "sub_launcher.py",
                    "sensor_controller.py", "sub_sensors.py",
                    "motor_controller.py", "sub_motors.py",
                    "camera_controller.py", "sub_camera.py",
                ]:
                    self._ssh_run(client, f"pkill -f {name} || true")
                    time.sleep(0.2)
                client.close()
                self.status2.set_connected(False, "Disconnected")
            except Exception as e:
                self.status2.set_connected(False, "Disconnected")
                messagebox.showerror("Shutdown failed", str(e))
            finally:
                self._stop_runtime_clients()
                self._show_page1()

        threading.Thread(target=worker, daemon=True).start()

    def _ui_loop(self):
        if self.page2.winfo_ismapped():
            if self.camera_client and self.camera_client.connected:
                self.status2.set_connected(True, "Connected")
            else:
                self.status2.set_connected(False, "Disconnected")

            if self.sensor_rx:
                t = self.sensor_rx.latest
                self.battery_var.set(f"Battery: {t.battery:.2f} V")
                self.depth_var.set(f"Depth: {t.depth:.2f}")
                self.press_var.set(f"Pressure: {t.pressure:.2f}")
                self.temp_var.set(f"Temps: Pi {t.temp_pi:.1f} C | Env {t.temp_env:.1f} C")

            if self.controller and self.motor_sender:
                cmd = self.controller.poll()
                self.motor_sender.set_target(cmd)
                throttlep, yawp, pitchp, rollp, a_flagp = cmd.pct()
                self.throttle_var.set(f"throttle: {throttlep:.0f}")
                self.yaw_var.set(f"yaw: {yawp:.0f}")
                self.pitch_var.set(f"pitch: {pitchp:.0f}")
                self.roll_var.set(f"roll: {rollp:.0f}")
                self.a_flag_var.set(f"a_flag: {a_flagp:.0f}")
                self._update_armed_display(bool(cmd.a_flag))
            if self.start_time:
                elapsed = int(time.time() - self.start_time)
                mm, ss = divmod(elapsed, 60)
                self.timer_var.set(f"Timer: {mm:02d}:{ss:02d}")

            if self.camera_client:
                frame = self.camera_client.get_frame()
                if frame is not None:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    img = Image.fromarray(frame)

                    w = max(1, self.video_label.winfo_width())
                    h = max(1, self.video_label.winfo_height())
                    img = img.resize((w, h))

                    imgtk = ImageTk.PhotoImage(image=img)
                    self.video_label.imgtk = imgtk
                    self.video_label.configure(image=imgtk, text="")

        self.after(33, self._ui_loop)


if __name__ == "__main__":
    app = AnglerFishApp()
    app.mainloop()
