import json
import os
import threading
import time

import cv2
import paramiko
from PySide6.QtCore import QTimer, Qt, Signal
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from camera import CameraClient
from controller import XboxControllerReader, MotorUdpSender, get_mix_limits_pct, set_mix_limits_pct
from sensor import SensorUdpReceiver


CONFIG_PATH = os.path.join(os.path.expanduser("~"), ".anglerfish_config.json")
AVOID_LOW = 1406
AVOID_HIGH = 1514
PULSE_MIN_LOW = 1000
PULSE_MAX_HIGH = 2000


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


class StatusBox(QWidget):
    def __init__(self, parent):
        super().__init__(parent)
        self.label = QLabel("Disconnected")
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setFrameShape(QFrame.Box)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.label)

        self.set_connected(False)

    def set_connected(self, connected: bool, text: str | None = None):
        self.label.setText(text if text is not None else ("Connected" if connected else "Disconnected"))
        bg = "#90EE90" if connected else "#FF6347"
        self.label.setStyleSheet(f"QLabel {{ background-color: {bg}; }}")

    def set_text(self, text: str):
        self.label.setText(text)


class TuneDialog(QDialog):
    def __init__(self, parent, initial_vals: dict, apply_callback):
        super().__init__(parent)
        self.setWindowTitle("Tune Parameters")
        self.setModal(True)
        self.resize(460, 300)

        layout = QVBoxLayout(self)
        form = QFormLayout()

        self.pulse_min_pct = QDoubleSpinBox()
        self.pulse_min_pct.setRange(0.0, 100.0)
        self.pulse_min_pct.setDecimals(1)
        self.pulse_min_pct.setSuffix(" %")

        self.pulse_min_us = QDoubleSpinBox()
        self.pulse_min_us.setRange(PULSE_MIN_LOW, AVOID_LOW)
        self.pulse_min_us.setDecimals(0)
        self.pulse_min_us.setSuffix(" us")

        self.pulse_max_pct = QDoubleSpinBox()
        self.pulse_max_pct.setRange(0.0, 100.0)
        self.pulse_max_pct.setDecimals(1)
        self.pulse_max_pct.setSuffix(" %")

        self.pulse_max_us = QDoubleSpinBox()
        self.pulse_max_us.setRange(AVOID_HIGH, PULSE_MAX_HIGH)
        self.pulse_max_us.setDecimals(0)
        self.pulse_max_us.setSuffix(" us")

        self.roll_pct = QDoubleSpinBox()
        self.roll_pct.setRange(0.0, 100.0)
        self.roll_pct.setDecimals(1)
        self.roll_pct.setSuffix(" %")

        self.pitch_pct = QDoubleSpinBox()
        self.pitch_pct.setRange(0.0, 100.0)
        self.pitch_pct.setDecimals(1)
        self.pitch_pct.setSuffix(" %")

        self.yaw_pct = QDoubleSpinBox()
        self.yaw_pct.setRange(0.0, 100.0)
        self.yaw_pct.setDecimals(1)
        self.yaw_pct.setSuffix(" %")

        form.addRow("Pi PULSE_MIN (%: 1000=100, AVOID_LOW=0):", self.pulse_min_pct)
        form.addRow("Pi PULSE_MIN (us):", self.pulse_min_us)
        form.addRow("Pi PULSE_MAX (%: AVOID_HIGH=0, 2000=100):", self.pulse_max_pct)
        form.addRow("Pi PULSE_MAX (us):", self.pulse_max_us)
        form.addRow(QLabel(""), QLabel(""))
        form.addRow("PC ROLL_MAX:", self.roll_pct)
        form.addRow("PC PITCH_MAX:", self.pitch_pct)
        form.addRow("PC YAW_MAX:", self.yaw_pct)

        layout.addLayout(form)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        apply_btn = buttons.addButton("Apply", QDialogButtonBox.ApplyRole)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        apply_btn.clicked.connect(self._on_apply_clicked)
        layout.addWidget(buttons)

        self._apply_callback = apply_callback

        self._syncing = False
        self.pulse_min_pct.valueChanged.connect(self._on_min_pct_changed)
        self.pulse_min_us.valueChanged.connect(self._on_min_us_changed)
        self.pulse_max_pct.valueChanged.connect(self._on_max_pct_changed)
        self.pulse_max_us.valueChanged.connect(self._on_max_us_changed)

        self.pulse_min_us.setValue(initial_vals["pulse_min_us"])
        self.pulse_max_us.setValue(initial_vals["pulse_max_us"])
        self.roll_pct.setValue(initial_vals["roll_pct"])
        self.pitch_pct.setValue(initial_vals["pitch_pct"])
        self.yaw_pct.setValue(initial_vals["yaw_pct"])

    def _on_min_pct_changed(self, pct: float):
        if self._syncing:
            return
        self._syncing = True
        span = AVOID_LOW - PULSE_MIN_LOW
        self.pulse_min_us.setValue(round(AVOID_LOW - ((pct / 100.0) * span)))
        self._syncing = False

    def _on_min_us_changed(self, us: float):
        if self._syncing:
            return
        self._syncing = True
        span = AVOID_LOW - PULSE_MIN_LOW
        self.pulse_min_pct.setValue(((AVOID_LOW - us) / span) * 100.0)
        self._syncing = False

    def _on_max_pct_changed(self, pct: float):
        if self._syncing:
            return
        self._syncing = True
        span = PULSE_MAX_HIGH - AVOID_HIGH
        self.pulse_max_us.setValue(round(AVOID_HIGH + ((pct / 100.0) * span)))
        self._syncing = False

    def _on_max_us_changed(self, us: float):
        if self._syncing:
            return
        self._syncing = True
        span = PULSE_MAX_HIGH - AVOID_HIGH
        self.pulse_max_pct.setValue(((us - AVOID_HIGH) / span) * 100.0)
        self._syncing = False

    def _on_apply_clicked(self):
        self._apply_callback(self.values())

    def values(self) -> dict:
        return {
            "pulse_min_us": int(self.pulse_min_us.value()),
            "pulse_max_us": int(self.pulse_max_us.value()),
            "roll_pct": float(self.roll_pct.value()),
            "pitch_pct": float(self.pitch_pct.value()),
            "yaw_pct": float(self.yaw_pct.value()),
        }


class AnglerFishApp(QMainWindow):
    status1_conn_signal = Signal(bool, str)
    status1_text_signal = Signal(str)
    status2_conn_signal = Signal(bool, str)
    status2_text_signal = Signal(str)
    error_signal = Signal(str, str)
    warning_signal = Signal(str, str)
    show_page2_signal = Signal()
    show_page1_signal = Signal()

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Project Angler Fish")
        self.resize(1200, 700)

        cfg = load_config()
        self.pi_ip = cfg.get("ip", "anglerfish.local")
        self.pi_user = cfg.get("user", "pi")
        self.pi_pass = cfg.get("pass", "raspberry")
        self.save_creds = cfg.get("save", True)

        cfg_roll = float(cfg.get("roll_pct", 30.0))
        cfg_pitch = float(cfg.get("pitch_pct", 30.0))
        cfg_yaw = float(cfg.get("yaw_pct", 30.0))
        set_mix_limits_pct(cfg_roll, cfg_pitch, cfg_yaw)

        self.running = False
        self.start_time = None

        self.tune_state = {
            "pulse_min_us": max(PULSE_MIN_LOW, min(AVOID_LOW, int(cfg.get("pulse_min_us", 1350)))),
            "pulse_max_us": max(AVOID_HIGH, min(PULSE_MAX_HIGH, int(cfg.get("pulse_max_us", 1750)))),
        }

        self.camera_client = None
        self.sensor_rx = None
        self.motor_sender = None
        self.controller = None

        self.stack = QStackedWidget()
        self.page1 = QWidget()
        self.page2 = QWidget()

        self.setCentralWidget(self.stack)
        self.stack.addWidget(self.page1)
        self.stack.addWidget(self.page2)

        self.status1_conn_signal.connect(self._set_status1_connected)
        self.status1_text_signal.connect(self._set_status1_text)
        self.status2_conn_signal.connect(self._set_status2_connected)
        self.status2_text_signal.connect(self._set_status2_text)
        self.error_signal.connect(self._show_error)
        self.warning_signal.connect(self._show_warning)
        self.show_page2_signal.connect(self._show_page2)
        self.show_page1_signal.connect(self._show_page1)

        self._build_page1()
        self._build_page2()

        self.stack.setCurrentWidget(self.page1)

        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self._ui_loop)
        self.ui_timer.start(16)

    def closeEvent(self, event):
        self._stop_runtime_clients()
        super().closeEvent(event)

    def _show_error(self, title: str, text: str):
        QMessageBox.critical(self, title, text)

    def _show_warning(self, title: str, text: str):
        QMessageBox.warning(self, title, text)

    def _set_status1_connected(self, connected: bool, text: str):
        self.status1.set_connected(connected, text)

    def _set_status1_text(self, text: str):
        self.status1.set_text(text)

    def _set_status2_connected(self, connected: bool, text: str):
        self.status2.set_connected(connected, text)

    def _set_status2_text(self, text: str):
        self.status2.set_text(text)

    # SSH helpers
    def _ssh_connect(self):
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect(self.pi_ip_input.text().strip(), username=self.pi_user_input.text().strip(), password=self.pi_pass_input.text(), timeout=8)
        return client

    def _ssh_run(self, client, cmd: str):
        stdin, stdout, stderr = client.exec_command(cmd)
        _ = stdout.read()
        err = stderr.read()
        return _.decode("utf-8", errors="ignore"), err.decode("utf-8", errors="ignore")

    # Page 1
    def _build_page1(self):
        root_layout = QVBoxLayout(self.page1)
        root_layout.setContentsMargins(24, 24, 24, 24)
        root_layout.setSpacing(10)

        title = QLabel("Project Angler Fish")
        title.setStyleSheet("font-size: 28px; font-weight: 700;")
        root_layout.addWidget(title, alignment=Qt.AlignHCenter)

        form_wrap = QWidget()
        form_layout = QFormLayout(form_wrap)

        self.pi_ip_input = QLineEdit(self.pi_ip)
        self.pi_user_input = QLineEdit(self.pi_user)
        self.pi_pass_input = QLineEdit(self.pi_pass)
        self.pi_pass_input.setEchoMode(QLineEdit.Password)

        form_layout.addRow("Host / IP Address:", self.pi_ip_input)
        form_layout.addRow("Username:", self.pi_user_input)
        form_layout.addRow("Password:", self.pi_pass_input)

        self.show_pass_check = QCheckBox("Show password")
        self.show_pass_check.toggled.connect(self._toggle_pass)
        self.save_creds_check = QCheckBox("Save IP/User/Password")
        self.save_creds_check.setChecked(self.save_creds)
        form_layout.addRow("", self.show_pass_check)
        form_layout.addRow("", self.save_creds_check)

        root_layout.addWidget(form_wrap, alignment=Qt.AlignHCenter)

        self.status1 = StatusBox(self.page1)
        self.status1.setFixedWidth(220)
        root_layout.addWidget(self.status1, alignment=Qt.AlignHCenter)

        btn_row = QHBoxLayout()
        update_btn = QPushButton("Update Submarine")
        update_btn.clicked.connect(self._on_update_sub)
        start_btn = QPushButton("Start Submarine")
        start_btn.clicked.connect(self._on_start_sub)
        btn_row.addWidget(update_btn)
        btn_row.addWidget(start_btn)
        root_layout.addLayout(btn_row)
        root_layout.addStretch()

    def _toggle_pass(self, checked: bool):
        self.pi_pass_input.setEchoMode(QLineEdit.Normal if checked else QLineEdit.Password)

    def _maybe_save_creds(self):
        cfg = load_config()
        ip = self.pi_ip_input.text()
        user = self.pi_user_input.text()
        pw = self.pi_pass_input.text()
        cfg.update({"ip": ip, "user": user})
        if self.save_creds_check.isChecked():
            cfg.update({"pass": pw, "save": True})
        else:
            cfg.pop("pass", None)
            cfg.update({"save": False})
        save_config(cfg)

    def _save_tune_config(self):
        cfg = load_config()
        roll_pct, pitch_pct, yaw_pct = get_mix_limits_pct()
        cfg.update(
            {
                "pulse_min_us": int(self.tune_state["pulse_min_us"]),
                "pulse_max_us": int(self.tune_state["pulse_max_us"]),
                "roll_pct": float(roll_pct),
                "pitch_pct": float(pitch_pct),
                "yaw_pct": float(yaw_pct),
            }
        )
        save_config(cfg)

    def _on_update_sub(self):
        self._maybe_save_creds()

        def worker():
            try:
                self.status1_conn_signal.emit(True, "Connected")
                client = self._ssh_connect()
                self.status1_text_signal.emit("Updating Files")
                self._ssh_run(client, "cd ~/Project-Angler-Fish && git pull")
                time.sleep(6)
                client.close()
                self.status1_conn_signal.emit(False, "Disconnected")
            except Exception as e:
                self.status1_conn_signal.emit(False, "Disconnected")
                self.error_signal.emit("Update failed", str(e))

        threading.Thread(target=worker, daemon=True).start()

    def _on_start_sub(self):
        self._maybe_save_creds()

        def worker():
            try:
                self.status1_conn_signal.emit(True, "Connected")
                client = self._ssh_connect()
                cmd = "sudo pigpiod"
                self._ssh_run(client, cmd)
                cmd = "nohup python3 /home/pi/Project-Angler-Fish/Pi/sub_launcher.py > launcher.log 2>&1 &"
                self._ssh_run(client, cmd)
                client.close()
                self.status1_conn_signal.emit(False, "Disconnected")
                time.sleep(2)
                self._start_runtime_clients()
                self.show_page2_signal.emit()
            except Exception as e:
                self.status1_conn_signal.emit(False, "Disconnected")
                self.error_signal.emit("Start failed", str(e))

        threading.Thread(target=worker, daemon=True).start()

    # Page 2
    def _build_page2(self):
        root_layout = QVBoxLayout(self.page2)

        top_layout = QHBoxLayout()
        top_title = QLabel("Project Angler Fish - Live")
        top_title.setStyleSheet("font-size: 22px; font-weight: 700;")
        top_layout.addWidget(top_title)

        self.status2 = StatusBox(self.page2)
        self.status2.setFixedWidth(180)
        top_layout.addWidget(self.status2)
        top_layout.addStretch()

        shutdown_btn = QPushButton("Shutdown Submarine")
        shutdown_btn.clicked.connect(self._on_shutdown)
        tune_btn = QPushButton("Tune")
        tune_btn.clicked.connect(self._on_tune)
        top_layout.addWidget(tune_btn)
        top_layout.addWidget(shutdown_btn)

        root_layout.addLayout(top_layout)

        body_layout = QGridLayout()
        body_layout.setColumnStretch(0, 1)
        body_layout.setColumnStretch(1, 0)

        self.video_label = QLabel("Waiting for video...")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("background-color: black; color: white;")
        self.video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        body_layout.addWidget(self.video_label, 0, 0)

        side = QWidget()
        side.setFixedWidth(320)
        side_layout = QVBoxLayout(side)
        side_layout.setContentsMargins(8, 8, 8, 8)
        side_layout.setSpacing(6)

        # System status block
        sys_title = QLabel("System")
        sys_title.setStyleSheet("font-weight: 700;")
        side_layout.addWidget(sys_title)

        self.timer_label = QLabel("Timer: 00:00")
        self.battery_label = QLabel("Battery: -- V")
        self.temp_label = QLabel("Pi Temp: -- C")
        self.video_fps_label = QLabel("Video FPS: --")
        for lbl in [self.timer_label, self.battery_label, self.temp_label, self.video_fps_label]:
            side_layout.addWidget(lbl)

        sep1 = QFrame()
        sep1.setFrameShape(QFrame.HLine)
        side_layout.addWidget(sep1)

        # Water/environment block
        env_title = QLabel("Environment")
        env_title.setStyleSheet("font-weight: 700;")
        side_layout.addWidget(env_title)

        self.depth_label = QLabel("Depth: --")
        self.press_label = QLabel("Pressure: --")
        self.env_temp_label = QLabel("Water Temp: -- C")
        self.enclosure_temp_label = QLabel("Enclosure Temp: -- C")
        for lbl in [self.depth_label, self.press_label, self.env_temp_label, self.enclosure_temp_label]:
            side_layout.addWidget(lbl)

        sep2 = QFrame()
        sep2.setFrameShape(QFrame.HLine)
        side_layout.addWidget(sep2)

        # Motion block
        motion_title = QLabel("Motion")
        motion_title.setStyleSheet("font-weight: 700;")
        side_layout.addWidget(motion_title)
        self.speed_label = QLabel("Speed: -- m/s")
        self.accel_label = QLabel("Accel: -- m/s^2")
        for lbl in [self.speed_label, self.accel_label]:
            side_layout.addWidget(lbl)

        sep3 = QFrame()
        sep3.setFrameShape(QFrame.HLine)
        side_layout.addWidget(sep3)

        # Thruster command block
        vector_title = QLabel("Thruster Command")
        vector_title.setStyleSheet("font-weight: 700;")
        side_layout.addWidget(vector_title)
        self.m1_label = QLabel("m1: 0")
        self.m2_label = QLabel("m2: 0")
        self.m3_label = QLabel("m3: 0")
        self.m4_label = QLabel("m4: 0")
        for lbl in [self.m1_label, self.m2_label, self.m3_label, self.m4_label]:
            side_layout.addWidget(lbl)

        sep4 = QFrame()
        sep4.setFrameShape(QFrame.HLine)
        side_layout.addWidget(sep4)

        self.armed_label = QLabel("DISARMED")
        self.armed_label.setAlignment(Qt.AlignCenter)
        self.armed_label.setMinimumHeight(36)
        self.armed_label.setFrameShape(QFrame.Panel)
        self.armed_label.setFrameShadow(QFrame.Sunken)
        side_layout.addWidget(self.armed_label)

        side_layout.addStretch()
        body_layout.addWidget(side, 0, 1)

        root_layout.addLayout(body_layout)
        self._update_armed_display(False)

    def _update_armed_display(self, armed: bool):
        if armed:
            self.armed_label.setText("ARMED")
            self.armed_label.setStyleSheet("background-color: #00ff00; color: black; font-weight: 700;")
        else:
            self.armed_label.setText("DISARMED")
            self.armed_label.setStyleSheet("background-color: #ff6b6b; color: white; font-weight: 700;")

    def _show_page2(self):
        self.stack.setCurrentWidget(self.page2)

    def _show_page1(self):
        self.stack.setCurrentWidget(self.page1)

    def _start_runtime_clients(self):
        self.start_time = time.time()
        self.running = True

        # use RTSP port and default path
        ip = self.pi_ip_input.text().strip()
        self.camera_client = CameraClient(ip, port=8554, path="stream", reconnect=True)
        self.camera_client.start()

        self.sensor_rx = SensorUdpReceiver(listen_port=9100)
        self.sensor_rx.start()

        try:
            self.controller = XboxControllerReader()
        except Exception as e:
            self.warning_signal.emit("Controller", f"Controller not ready: {e}")
            self.controller = None

        self.motor_sender = MotorUdpSender(ip, pi_port=9000, rate_hz=30.0)
        self.motor_sender.start()
        self.motor_sender.send_tuning(self.tune_state["pulse_min_us"], self.tune_state["pulse_max_us"])

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
                self.status2_conn_signal.emit(True, "Connected")
                self.status2_text_signal.emit("Terminating")
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
                self.status2_conn_signal.emit(False, "Disconnected")
            except Exception as e:
                self.status2_conn_signal.emit(False, "Disconnected")
                self.error_signal.emit("Shutdown failed", str(e))
            finally:
                self._stop_runtime_clients()
                self.show_page1_signal.emit()

        threading.Thread(target=worker, daemon=True).start()

    def _on_tune(self):
        roll_pct, pitch_pct, yaw_pct = get_mix_limits_pct()
        initial_vals = {
            "pulse_min_us": self.tune_state["pulse_min_us"],
            "pulse_max_us": self.tune_state["pulse_max_us"],
            "roll_pct": roll_pct,
            "pitch_pct": pitch_pct,
            "yaw_pct": yaw_pct,
        }
        dlg = TuneDialog(self, initial_vals, self._apply_tune_values)
        if dlg.exec() == QDialog.Accepted:
            self._apply_tune_values(dlg.values())

    def _apply_tune_values(self, vals: dict):
        pulse_min_us = max(PULSE_MIN_LOW, min(AVOID_LOW, int(vals["pulse_min_us"])))
        pulse_max_us = max(AVOID_HIGH, min(PULSE_MAX_HIGH, int(vals["pulse_max_us"])))
        if pulse_min_us >= pulse_max_us:
            self.warning_signal.emit("Tune", "PULSE_MIN must be lower than PULSE_MAX.")
            return False

        set_mix_limits_pct(vals["roll_pct"], vals["pitch_pct"], vals["yaw_pct"])
        self.tune_state["pulse_min_us"] = pulse_min_us
        self.tune_state["pulse_max_us"] = pulse_max_us
        self._save_tune_config()

        if self.motor_sender:
            self.motor_sender.send_tuning(pulse_min_us, pulse_max_us)

        self.status2_text_signal.emit(
            f"Tune applied: min={pulse_min_us} max={pulse_max_us} | "
            f"roll={vals['roll_pct']:.1f}% pitch={vals['pitch_pct']:.1f}% yaw={vals['yaw_pct']:.1f}%"
        )
        return True

    def _ui_loop(self):
        if self.stack.currentWidget() is self.page2:
            if self.camera_client and self.camera_client.connected:
                self.status2.set_connected(True, "Connected")
            else:
                self.status2.set_connected(False, "Disconnected")

            if self.sensor_rx:
                t = self.sensor_rx.latest
                self.battery_label.setText(f"Battery: {t.battery:.2f} V")
                self.depth_label.setText(f"Depth: {t.depth:.2f}")
                self.press_label.setText(f"Pressure: {t.pressure:.2f}")
                self.temp_label.setText(f"Pi Temp: {t.temp_pi:.1f} C")
                self.env_temp_label.setText(f"Water Temp: {t.temp_env:.1f} C")
                self.enclosure_temp_label.setText(f"Enclosure Temp: {t.temp_enclosure:.1f} C")
                self.speed_label.setText(f"Speed: {t.speed:.2f} m/s")
                self.accel_label.setText(f"Accel: {t.acceleration:.2f} m/s^2")

            if self.controller and self.motor_sender:
                cmd = self.controller.poll()
                self.motor_sender.set_target(cmd)
                throttlep, yawp, pitchp, rollp, _ = cmd.pct()
                self.m1_label.setText(f"m1: {throttlep:.0f}")
                self.m2_label.setText(f"m2: {yawp:.0f}")
                self.m3_label.setText(f"m3: {pitchp:.0f}")
                self.m4_label.setText(f"m4: {rollp:.0f}")
                self._update_armed_display(bool(cmd.a_flag))
            if self.start_time:
                elapsed = int(time.time() - self.start_time)
                mm, ss = divmod(elapsed, 60)
                self.timer_label.setText(f"Timer: {mm:02d}:{ss:02d}")

            if self.camera_client:
                frame = self.camera_client.get_frame()
                self.video_fps_label.setText(f"Video FPS: {self.camera_client.fps:.1f}")
                if frame is not None:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    h, w, ch = frame.shape
                    qimg = QImage(frame.data, w, h, ch * w, QImage.Format_RGB888)
                    pix = QPixmap.fromImage(qimg)
                    scaled = pix.scaled(self.video_label.size(), Qt.IgnoreAspectRatio, Qt.SmoothTransformation)
                    self.video_label.setPixmap(scaled)
                    self.video_label.setText("")


if __name__ == "__main__":
    app = QApplication([])
    win = AnglerFishApp()
    win.show()
    app.exec()
