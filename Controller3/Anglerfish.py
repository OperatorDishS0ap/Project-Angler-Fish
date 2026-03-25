import sys
import time
import json
import socket
import os
import shlex
import subprocess
import tempfile
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
try:
    import paramiko
except ImportError:
    paramiko = None

try:
    import pygame
except ImportError:
    pygame = None

try:
    import xbox360_controller
except ImportError:
    xbox360_controller = None

from PySide6.QtCore import Qt, QThread, Signal, Slot, QTimer
from PySide6.QtGui import QAction, QImage, QPixmap
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QSpinBox,
    QStatusBar,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


# ============================================================
# Data models
# ============================================================
@dataclass
class TelemetryData:
    timer_s: float = 75.0
    battery_v: float = 0.0
    pi_temp_c: float = 0.0
    video_fps: float = 0.0

    depth_m: float = 0.0
    pressure_bar: float = 0.0
    water_temp_c: float = 0.0
    enclosure_temp_c: float = 0.0

    speed_mps: float = 0.0
    accel_mps2: float = 0.0

    m1: int = 0
    m2: int = 0
    m3: int = 0
    m4: int = 0

    armed: bool = False


@dataclass
class TuningData:
    deadzone: float = 0.08
    trigger_scale: float = 1.00
    yaw_scale: float = 1.00
    strafe_scale: float = 1.00
    vertical_scale: float = 1.00
    max_command: int = 1000
    command_timeout_ms: int = 250
    camera_brightness: float = 0.0
    camera_contrast: float = 1.0
    auto_reconnect_video: bool = True
    auto_reconnect_udp: bool = True


# ============================================================
# Worker threads
# ============================================================
class VideoWorker(QThread):
    frame_ready = Signal(QImage)
    stats_ready = Signal(dict)
    status_changed = Signal(str)

    def __init__(self, rtsp_url: str, parent=None):
        super().__init__(parent)
        self.rtsp_url = rtsp_url
        self._running = True
        self._capture = None

    def stop(self):
        self._running = False

    def run(self):
        self.status_changed.emit("Opening video stream...")
        self._capture = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)

        if not self._capture.isOpened():
            self.status_changed.emit("Video stream failed to open")
            return

        self.status_changed.emit("Video connected")
        prev_time = time.time()
        fps = 0.0

        while self._running:
            ok, frame = self._capture.read()
            if not ok:
                self.status_changed.emit("Video read failed")
                break

            now = time.time()
            dt = max(now - prev_time, 1e-6)
            fps = 1.0 / dt
            prev_time = now

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            bytes_per_line = ch * w
            image = QImage(rgb.data, w, h, bytes_per_line, QImage.Format_RGB888).copy()
            self.frame_ready.emit(image)
            self.stats_ready.emit({"video_fps": fps})

        if self._capture is not None:
            self._capture.release()
        self.status_changed.emit("Video stopped")


class UdpLinkWorker(QThread):
    telemetry_received = Signal(dict)
    log_message = Signal(str)
    link_state = Signal(str)

    def __init__(self, pi_host: str, cmd_port: int, telemetry_port: int, parent=None):
        super().__init__(parent)
        self.pi_host = pi_host
        self.cmd_port = cmd_port
        self.telemetry_port = telemetry_port
        self._running = True
        self._cmd_sock: Optional[socket.socket] = None
        self._telemetry_sock: Optional[socket.socket] = None
        self._latest_command = {"m1": 0.0, "m2": 0.0, "m3": 0.0, "m4": 0.0, "arm": False}
        self._latest_tuning = asdict(TuningData())

    def stop(self):
        self._running = False

    @Slot(dict)
    def update_command(self, cmd: dict):
        if "armed" in cmd and "arm" not in cmd:
            cmd = dict(cmd)
            cmd["arm"] = bool(cmd["armed"])
        self._latest_command.update(cmd)
        if not bool(self._latest_command.get("arm", False)):
            self._latest_command["m1"] = 0.0
            self._latest_command["m2"] = 0.0
            self._latest_command["m3"] = 0.0
            self._latest_command["m4"] = 0.0

    @Slot(dict)
    def update_tuning(self, tuning: dict):
        self._latest_tuning.update(tuning)
        tune_payload = {"type": "tune"}
        if "pulse_min_us" in self._latest_tuning:
            tune_payload["pulse_min_us"] = int(self._latest_tuning["pulse_min_us"])
        if "pulse_max_us" in self._latest_tuning:
            tune_payload["pulse_max_us"] = int(self._latest_tuning["pulse_max_us"])
        if len(tune_payload) > 1:
            self.send_json(tune_payload)

    def _build_motor_command_payload(self) -> dict:
        armed = bool(self._latest_command.get("arm", self._latest_command.get("armed", False)))
        if not armed:
            return {
                "type": "command",
                "arm": False,
            }
        return {
            "type": "command",
            "m1": float(self._latest_command.get("m1", 0.0)),
            "m2": float(self._latest_command.get("m2", 0.0)),
            "m3": float(self._latest_command.get("m3", 0.0)),
            "m4": float(self._latest_command.get("m4", 0.0)),
            "arm": True,
        }

    def send_json(self, payload: dict):
        if not self._cmd_sock:
            return
        try:
            raw = json.dumps(payload).encode("utf-8")
            self._cmd_sock.sendto(raw, (self.pi_host, self.cmd_port))
        except OSError as exc:
            self.log_message.emit(f"Send error: {exc}")

    def run(self):
        try:
            self._cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._cmd_sock.setblocking(False)

            self._telemetry_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._telemetry_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._telemetry_sock.bind(("0.0.0.0", self.telemetry_port))
            self._telemetry_sock.settimeout(0.1)
            self.link_state.emit("UDP link ready")
            self.log_message.emit(
                f"Telemetry listener bound on 0.0.0.0:{self.telemetry_port}, sending commands to {self.pi_host}:{self.cmd_port}"
            )
        except OSError as exc:
            self.link_state.emit("UDP link failed")
            self.log_message.emit(f"UDP setup error: {exc}")
            return

        last_command_tx = 0.0
        while self._running:
            now = time.time()
            if now - last_command_tx >= 0.05:
                self.send_json(self._build_motor_command_payload())
                last_command_tx = now

            try:
                data, addr = self._telemetry_sock.recvfrom(8192)
                payload = json.loads(data.decode("utf-8"))
                if isinstance(payload, dict):
                    self.telemetry_received.emit(payload)
            except socket.timeout:
                pass
            except BlockingIOError:
                pass
            except json.JSONDecodeError as exc:
                self.log_message.emit(f"Telemetry JSON error: {exc}")
            except OSError as exc:
                self.log_message.emit(f"Telemetry socket error: {exc}")
                break

        for sock in (self._cmd_sock, self._telemetry_sock):
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
        self.link_state.emit("UDP link stopped")


# ============================================================
# Reusable widgets
# ============================================================
class SectionBox(QGroupBox):
    def __init__(self, title: str, parent=None):
        super().__init__(title, parent)
        self.setStyleSheet(
            """
            QGroupBox {
                font-size: 16px;
                font-weight: 700;
                border: 1px solid #606060;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 4px;
            }
            """
        )


class TelemetryPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.labels = {}

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(10)

        system = self._make_section(
            "System",
            [
                ("timer_s", "Timer", self._format_timer),
                ("pi_temp_c", "Pi Temp", lambda v: f"{v:.1f} C"),
                ("video_fps", "Video FPS", lambda v: f"{v:.1f}"),
            ],
        )
        environment = self._make_section(
            "Environment",
            [
                ("pressure_bar", "Pressure", lambda v: f"{v:.2f}"),
                ("water_temp_c", "Water Temp", lambda v: f"{v:.1f} C"),
            ],
        )
        motion = self._make_section(
            "Motion",
            [
                ("accel_mps2", "Accel", lambda v: f"{v:.2f} m/s^2"),
            ],
        )
        thrusters = self._make_section(
            "Thruster Command",
            [
                ("m1", "m1", lambda v: f"{int(v)}"),
                ("m2", "m2", lambda v: f"{int(v)}"),
                ("m3", "m3", lambda v: f"{int(v)}"),
                ("m4", "m4", lambda v: f"{int(v)}"),
            ],
        )

        self.arm_label = QLabel("DISARMED")
        self.arm_label.setAlignment(Qt.AlignCenter)
        self.arm_label.setMinimumHeight(54)
        self.arm_label.setStyleSheet(self._arm_style(False))

        root.addWidget(system)
        root.addWidget(environment)
        root.addWidget(motion)
        root.addWidget(thrusters)
        root.addWidget(self.arm_label)
        root.addStretch(1)

        self.update_telemetry(TelemetryData())

    def _make_section(self, title, rows):
        box = SectionBox(title)
        layout = QFormLayout(box)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)
        for key, label, formatter in rows:
            value_label = QLabel("0")
            value_label.setStyleSheet("font-size: 14px;")
            value_label.setProperty("formatter", formatter)
            self.labels[key] = value_label
            layout.addRow(f"{label}:", value_label)
        return box

    @staticmethod
    def _format_timer(seconds: float) -> str:
        total = max(0, int(seconds))
        mins = total // 60
        secs = total % 60
        return f"{mins:02d}:{secs:02d}"

    @staticmethod
    def _arm_style(armed: bool) -> str:
        if armed:
            return (
                "background-color: #14ff14; color: black; font-size: 24px; "
                "font-weight: 800; border-radius: 8px;"
            )
        return (
            "background-color: #5a0000; color: white; font-size: 24px; "
            "font-weight: 800; border-radius: 8px;"
        )

    def update_telemetry(self, telemetry: TelemetryData):
        data = asdict(telemetry)
        for key, label in self.labels.items():
            formatter = label.property("formatter")
            label.setText(formatter(data.get(key, 0)))
        self.arm_label.setText("ARMED" if telemetry.armed else "DISARMED")
        self.arm_label.setStyleSheet(self._arm_style(telemetry.armed))


class TuningTab(QWidget):
    tuning_changed = Signal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.controls = {}

        root = QVBoxLayout(self)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(12)

        drive_box = SectionBox("Drive Tuning")
        drive_form = QFormLayout(drive_box)
        self.controls["deadzone"] = self._double(0.00, 0.50, 0.01, 0.08)
        self.controls["trigger_scale"] = self._double(0.10, 2.00, 0.01, 1.00)
        self.controls["yaw_scale"] = self._double(0.10, 2.00, 0.01, 1.00)
        self.controls["strafe_scale"] = self._double(0.10, 2.00, 0.01, 1.00)
        self.controls["vertical_scale"] = self._double(0.10, 2.00, 0.01, 1.00)
        self.controls["max_command"] = self._spin(100, 1000, 10, 1000)
        self.controls["command_timeout_ms"] = self._spin(50, 2000, 10, 250)

        drive_form.addRow("Deadzone", self.controls["deadzone"])
        drive_form.addRow("Trigger Scale", self.controls["trigger_scale"])
        drive_form.addRow("Yaw Scale", self.controls["yaw_scale"])
        drive_form.addRow("Strafe Scale", self.controls["strafe_scale"])
        drive_form.addRow("Vertical Scale", self.controls["vertical_scale"])
        drive_form.addRow("Max Command", self.controls["max_command"])
        drive_form.addRow("Command Timeout (ms)", self.controls["command_timeout_ms"])

        camera_box = SectionBox("Camera Tuning")
        camera_form = QFormLayout(camera_box)
        self.controls["camera_brightness"] = self._double(-1.0, 1.0, 0.01, 0.0)
        self.controls["camera_contrast"] = self._double(0.0, 4.0, 0.01, 1.0)
        camera_form.addRow("Brightness", self.controls["camera_brightness"])
        camera_form.addRow("Contrast", self.controls["camera_contrast"])

        link_box = SectionBox("Link Options")
        link_form = QFormLayout(link_box)
        self.controls["auto_reconnect_video"] = QCheckBox()
        self.controls["auto_reconnect_video"].setChecked(True)
        self.controls["auto_reconnect_udp"] = QCheckBox()
        self.controls["auto_reconnect_udp"].setChecked(True)
        link_form.addRow("Auto Reconnect Video", self.controls["auto_reconnect_video"])
        link_form.addRow("Auto Reconnect UDP", self.controls["auto_reconnect_udp"])

        buttons_row = QHBoxLayout()
        self.apply_btn = QPushButton("Apply Live")
        self.reset_btn = QPushButton("Reset Defaults")
        buttons_row.addWidget(self.apply_btn)
        buttons_row.addWidget(self.reset_btn)
        buttons_row.addStretch(1)

        root.addWidget(drive_box)
        root.addWidget(camera_box)
        root.addWidget(link_box)
        root.addLayout(buttons_row)
        root.addStretch(1)

        self.apply_btn.clicked.connect(self.emit_tuning)
        self.reset_btn.clicked.connect(self.reset_defaults)

    def _double(self, minimum, maximum, step, value):
        widget = QDoubleSpinBox()
        widget.setRange(minimum, maximum)
        widget.setSingleStep(step)
        widget.setValue(value)
        widget.setDecimals(3)
        widget.setMinimumWidth(120)
        return widget

    def _spin(self, minimum, maximum, step, value):
        widget = QSpinBox()
        widget.setRange(minimum, maximum)
        widget.setSingleStep(step)
        widget.setValue(value)
        widget.setMinimumWidth(120)
        return widget

    def current_tuning(self) -> dict:
        result = {}
        for key, widget in self.controls.items():
            if isinstance(widget, (QDoubleSpinBox, QSpinBox)):
                result[key] = widget.value()
            elif isinstance(widget, QCheckBox):
                result[key] = widget.isChecked()
        return result

    @Slot()
    def emit_tuning(self):
        self.tuning_changed.emit(self.current_tuning())

    @Slot()
    def reset_defaults(self):
        defaults = TuningData()
        for key, value in asdict(defaults).items():
            widget = self.controls[key]
            if isinstance(widget, (QDoubleSpinBox, QSpinBox)):
                widget.setValue(value)
            elif isinstance(widget, QCheckBox):
                widget.setChecked(value)
        self.emit_tuning()


class LogTab(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        self.text = QTextEdit()
        self.text.setReadOnly(True)
        layout.addWidget(self.text)

    @Slot(str)
    def append_log(self, message: str):
        timestamp = time.strftime("%H:%M:%S")
        self.text.append(f"[{timestamp}] {message}")


# ============================================================
# Main window
# ============================================================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AnglerFish Control Station")
        self.resize(1400, 850)

        self.telemetry = TelemetryData()
        self.video_worker: Optional[VideoWorker] = None
        self.udp_worker: Optional[UdpLinkWorker] = None
        self.link_ready = False
        self.controller_device = None
        self.controller_active = False
        self.controller_armed = False
        self.controller_a_last_press_time = 0.0
        self.controller_missing_logged = False

        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)

        # Left: video and connection
        left = QVBoxLayout()
        left.setSpacing(10)

        self.connection_box = SectionBox("Connection")
        connection_layout = QGridLayout(self.connection_box)
        self.pi_username_edit = QLineEdit("pi")
        self.pi_password_edit = QLineEdit("")
        self.pi_password_edit.setEchoMode(QLineEdit.Password)
        self.pi_password_edit.setPlaceholderText("SSH password (optional)")
        self.show_password_check = QCheckBox("Show Password")
        self.pi_hostname_edit = QLineEdit("anglerfish.local")
        self.rtsp_path_edit = QLineEdit("rtsp://anglerfish.local:8554/cam")
        self.cmd_port_spin = QSpinBox()
        self.cmd_port_spin.setRange(1, 65535)
        self.cmd_port_spin.setValue(9000)
        self.telemetry_port_spin = QSpinBox()
        self.telemetry_port_spin.setRange(1, 65535)
        self.telemetry_port_spin.setValue(9001)

        self.connect_btn = QPushButton("Connect")
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.setEnabled(False)
        self.arm_btn = QPushButton("Arm")
        self.disarm_btn = QPushButton("Disarm")

        connection_layout.addWidget(QLabel("Pi Username"), 0, 0)
        connection_layout.addWidget(self.pi_username_edit, 0, 1)
        connection_layout.addWidget(QLabel("Pi Hostname"), 0, 2)
        connection_layout.addWidget(self.pi_hostname_edit, 0, 3)
        connection_layout.addWidget(QLabel("Pi Password"), 1, 0)
        connection_layout.addWidget(self.pi_password_edit, 1, 1, 1, 2)
        connection_layout.addWidget(self.show_password_check, 1, 3)
        connection_layout.addWidget(QLabel("RTSP URL"), 2, 0)
        connection_layout.addWidget(self.rtsp_path_edit, 2, 1, 1, 3)
        connection_layout.addWidget(QLabel("Cmd Port"), 3, 0)
        connection_layout.addWidget(self.cmd_port_spin, 3, 1)
        connection_layout.addWidget(QLabel("Telemetry Port"), 3, 2)
        connection_layout.addWidget(self.telemetry_port_spin, 3, 3)
        connection_layout.addWidget(self.connect_btn, 4, 0)

        self.disconnect_only_widget = QWidget()
        disconnect_only_layout = QHBoxLayout(self.disconnect_only_widget)
        disconnect_only_layout.setContentsMargins(0, 0, 0, 0)
        disconnect_only_layout.addStretch(1)
        disconnect_only_layout.addWidget(self.disconnect_btn)
        disconnect_only_layout.addStretch(1)
        self.disconnect_only_widget.setVisible(False)

        self.video_label = QLabel("Video not connected")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(800, 450)
        self.video_label.setStyleSheet(
            "background-color: black; color: #d0d0d0; border: 1px solid #505050; border-radius: 8px;"
        )
        self.video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.overlay_depth = QLabel(self.video_label)
        self.overlay_enclosure_temp = QLabel(self.video_label)
        self.overlay_battery = QLabel(self.video_label)
        self.overlay_speed = QLabel(self.video_label)
        for overlay in (
            self.overlay_depth,
            self.overlay_enclosure_temp,
            self.overlay_battery,
            self.overlay_speed,
        ):
            overlay.setAlignment(Qt.AlignCenter)
            overlay.setStyleSheet(
                "background: transparent; color: white; font-size: 13px; font-weight: 700;"
            )
            overlay.setAttribute(Qt.WA_TransparentForMouseEvents, True)
            overlay.show()

        left.addWidget(self.connection_box)
        left.addWidget(self.disconnect_only_widget)
        left.addWidget(self.video_label, stretch=1)

        # Right: tabs + telemetry panel
        right = QVBoxLayout()
        right.setSpacing(10)

        self.tabs = QTabWidget()
        self.telemetry_tab = QWidget()
        telemetry_tab_layout = QVBoxLayout(self.telemetry_tab)
        self.telemetry_panel = TelemetryPanel()
        arm_controls_layout = QHBoxLayout()
        arm_controls_layout.addWidget(self.arm_btn)
        arm_controls_layout.addWidget(self.disarm_btn)
        arm_controls_layout.addStretch(1)
        telemetry_tab_layout.addWidget(self.telemetry_panel)
        telemetry_tab_layout.addLayout(arm_controls_layout)

        self.tuning_tab = TuningTab()
        self.log_tab = LogTab()

        self.tabs.addTab(self.telemetry_tab, "Telemetry")
        self.tabs.addTab(self.tuning_tab, "Tuning")
        self.tabs.addTab(self.log_tab, "Log")

        right.addWidget(self.tabs)

        root.addLayout(left, stretch=3)
        root.addLayout(right, stretch=2)

        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.video_status = QLabel("Video: idle")
        self.link_status = QLabel("UDP: idle")
        self.status_bar.addPermanentWidget(self.video_status)
        self.status_bar.addPermanentWidget(self.link_status)

        self._build_menu()
        self._connect_signals()
        self._update_arm_buttons()
        self._update_video_overlay()

        # demo update timer for local UI test only
        self.demo_timer = QTimer(self)
        self.demo_timer.timeout.connect(self._demo_telemetry_tick)

        self.controller_timer = QTimer(self)
        self.controller_timer.setInterval(33)
        self.controller_timer.timeout.connect(self._controller_poll_tick)
        self._start_controller_polling()

        self._apply_dark_theme()

    @Slot(str)
    def update_rtsp_url_from_host(self, host: str):
        host = host.strip()
        if host:
            self.rtsp_path_edit.setText(f"rtsp://{host}:8554/cam")

    def _build_menu(self):
        file_menu = self.menuBar().addMenu("File")
        exit_action = QAction("Exit", self)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

        tools_menu = self.menuBar().addMenu("Tools")
        demo_action = QAction("Start Demo Telemetry", self)
        demo_action.triggered.connect(self.toggle_demo)
        tools_menu.addAction(demo_action)
        init_git_action = QAction("Initialize Git Deploy", self)
        init_git_action.triggered.connect(self.initialize_git_deploy)
        tools_menu.addAction(init_git_action)
        preflight_action = QAction("Git Deploy Preflight Check", self)
        preflight_action.triggered.connect(self.git_deploy_preflight_check)
        tools_menu.addAction(preflight_action)
        deploy_action = QAction("Deploy to Pi", self)
        deploy_action.triggered.connect(self.deploy_pi_folder)
        tools_menu.addAction(deploy_action)

    def _connect_signals(self):
        self.connect_btn.clicked.connect(self.start_links)
        self.disconnect_btn.clicked.connect(self.stop_links)
        self.arm_btn.clicked.connect(lambda: self.send_arm_state(True))
        self.disarm_btn.clicked.connect(lambda: self.send_arm_state(False))
        self.tuning_tab.tuning_changed.connect(self.on_tuning_changed)
        self.pi_hostname_edit.textChanged.connect(self.update_rtsp_url_from_host)
        self.show_password_check.toggled.connect(self.on_show_password_toggled)

    @Slot(bool)
    def on_show_password_toggled(self, checked: bool):
        self.pi_password_edit.setEchoMode(QLineEdit.Normal if checked else QLineEdit.Password)

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))

    def _apply_deadzone(self, value: float, deadzone: float) -> float:
        if abs(value) < deadzone:
            return 0.0
        return value

    def _try_initialize_controller(self) -> bool:
        if self.controller_active and self.controller_device is not None:
            return True

        if pygame is None or xbox360_controller is None:
            self.log_tab.append_log("Controller disabled: install pygame and ensure xbox360_controller.py is present")
            return False

        try:
            if not pygame.get_init():
                pygame.init()
            if not pygame.joystick.get_init():
                pygame.joystick.init()

            if pygame.joystick.get_count() == 0:
                if not self.controller_missing_logged:
                    self.log_tab.append_log("No Xbox controller detected")
                    self.controller_missing_logged = True
                return False

            deadzone = float(self.tuning_tab.current_tuning().get("deadzone", 0.08))
            self.controller_device = xbox360_controller.Controller(dead_zone=deadzone)
            self.controller_active = True
            self.controller_missing_logged = False
            self.log_tab.append_log("Xbox controller connected")
            return True
        except Exception as exc:
            self.controller_device = None
            self.controller_active = False
            self.log_tab.append_log(f"Controller init failed: {exc}")
            return False

    def _start_controller_polling(self):
        if self._try_initialize_controller():
            self.controller_timer.start()

    def _stop_controller_polling(self):
        self.controller_timer.stop()
        self.controller_device = None
        self.controller_active = False

    @Slot()
    def _controller_poll_tick(self):
        if not self._try_initialize_controller():
            return
        if self.udp_worker is None:
            return

        try:
            pygame.event.pump()
            buttons = self.controller_device.get_buttons()
            lt_x, lt_y = self.controller_device.get_left_stick()
            rt_x, _rt_y = self.controller_device.get_right_stick()
            triggers = self.controller_device.get_triggers()
            pad_up, pad_right, pad_down, pad_left = self.controller_device.get_pad()
        except Exception as exc:
            self.log_tab.append_log(f"Controller read failed: {exc}")
            self._stop_controller_polling()
            return

        if hasattr(xbox360_controller, "A") and xbox360_controller.A < len(buttons):
            if buttons[xbox360_controller.A]:
                now = time.time()
                if now - self.controller_a_last_press_time > 0.5:
                    self.controller_armed = not self.controller_armed
                    self.controller_a_last_press_time = now
                    self.log_tab.append_log("Controller ARMED" if self.controller_armed else "Controller DISARMED")

        tuning = self.tuning_tab.current_tuning()
        deadzone = float(tuning.get("deadzone", 0.08))
        trigger_scale = float(tuning.get("trigger_scale", 1.0))
        yaw_scale = float(tuning.get("yaw_scale", 1.0))
        strafe_scale = float(tuning.get("strafe_scale", 1.0))
        vertical_scale = float(tuning.get("vertical_scale", 1.0))

        lt_x = self._clamp(self._apply_deadzone(float(lt_x), deadzone) * strafe_scale, -1.0, 1.0)
        lt_y = self._clamp(self._apply_deadzone(float(lt_y), deadzone) * vertical_scale, -1.0, 1.0)
        rt_x = self._clamp(self._apply_deadzone(float(rt_x), deadzone) * yaw_scale, -1.0, 1.0)
        triggers = self._clamp(self._apply_deadzone(float(triggers), deadzone) * trigger_scale, -1.0, 1.0)

        yaw_flag = False
        pitch_flag = False
        m1 = m2 = m3 = m4 = 0.0

        if abs(triggers) > 0.05:
            m1 = triggers
            m2 = triggers
            yaw_flag = True

        yaw_step = self._clamp(0.3 * yaw_scale, 0.0, 1.0)
        if not yaw_flag:
            if abs(rt_x) > 0.05:
                m2 = rt_x
                m1 = -m2
            elif pad_left > 0:
                m2 = yaw_step
                m1 = -m2
            elif pad_right > 0:
                m2 = -yaw_step
                m1 = -m2

        pitch_step = self._clamp(0.3 * vertical_scale, 0.0, 1.0)
        if abs(lt_y) > 0.05:
            m3 = -lt_y
            m4 = m3
            pitch_flag = True
        elif pad_up > 0:
            m3 = pitch_step
            m4 = m3
            pitch_flag = True
        elif pad_down > 0:
            m3 = -pitch_step
            m4 = m3
            pitch_flag = True

        if not pitch_flag and abs(lt_x) > 0.05:
            m4 = lt_x
            m3 = -m4

        if not self.controller_armed:
            m1 = m2 = m3 = m4 = 0.0

        payload = {
            "m1": self._clamp(m1 * 100.0, -100.0, 100.0),
            "m2": self._clamp(m2 * 100.0, -100.0, 100.0),
            "m3": self._clamp(m3 * 100.0, -100.0, 100.0),
            "m4": self._clamp(m4 * 100.0, -100.0, 100.0),
            "arm": self.controller_armed,
        }
        self.udp_worker.update_command(payload)

        self.telemetry.m1 = int(payload["m1"])
        self.telemetry.m2 = int(payload["m2"])
        self.telemetry.m3 = int(payload["m3"])
        self.telemetry.m4 = int(payload["m4"])
        self.telemetry.armed = self.controller_armed
        self.telemetry_panel.update_telemetry(self.telemetry)
        self._update_arm_buttons()
        self._update_video_overlay()

    def _update_arm_buttons(self):
        self.arm_btn.setEnabled(self.link_ready and not self.telemetry.armed)
        self.disarm_btn.setEnabled(self.link_ready and self.telemetry.armed)

    def _update_video_overlay(self):
        self.overlay_depth.setText(f"Depth: {self.telemetry.depth_m:.2f} m")
        self.overlay_enclosure_temp.setText(
            f"Enclosure Temp: {self.telemetry.enclosure_temp_c:.1f} C"
        )
        self.overlay_battery.setText(f"Battery: {self.telemetry.battery_v:.2f} V")
        self.overlay_speed.setText(f"Speed: {self.telemetry.speed_mps:.2f} m/s")

        for overlay in (
            self.overlay_depth,
            self.overlay_enclosure_temp,
            self.overlay_battery,
            self.overlay_speed,
        ):
            overlay.adjustSize()

        self._position_video_overlays()

    def _position_video_overlays(self):
        margin = 12
        label_width = self.video_label.width()
        label_height = self.video_label.height()

        pixmap = self.video_label.pixmap()
        if pixmap is not None and not pixmap.isNull():
            video_width = pixmap.width()
            video_height = pixmap.height()
            video_x = max(0, (label_width - video_width) // 2)
            video_y = max(0, (label_height - video_height) // 2)
        else:
            video_width = label_width
            video_height = label_height
            video_x = 0
            video_y = 0

        depth_width = self.overlay_depth.width()
        depth_height = self.overlay_depth.height()
        enclosure_width = self.overlay_enclosure_temp.width()
        enclosure_height = self.overlay_enclosure_temp.height()
        battery_width = self.overlay_battery.width()
        battery_height = self.overlay_battery.height()
        speed_height = self.overlay_speed.height()

        self.overlay_depth.move(video_x + margin, video_y + margin)
        self.overlay_enclosure_temp.move(
            video_x + max(margin, (video_width - enclosure_width) // 2),
            video_y + margin,
        )
        self.overlay_battery.move(
            video_x + max(margin, video_width - battery_width - margin),
            video_y + margin,
        )
        self.overlay_speed.move(
            video_x + margin,
            video_y + max(margin, video_height - speed_height - margin),
        )

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._position_video_overlays()

    def _apply_dark_theme(self):
        self.setStyleSheet(
            """
            QWidget { background-color: #151515; color: #f0f0f0; font-size: 14px; }
            QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox, QTextEdit, QTabWidget::pane {
                background-color: #202020;
                border: 1px solid #4c4c4c;
                border-radius: 6px;
                padding: 4px;
            }
            QPushButton {
                background-color: #2b2b2b;
                border: 1px solid #5a5a5a;
                border-radius: 6px;
                padding: 8px 14px;
                font-weight: 600;
            }
            QPushButton:hover { background-color: #383838; }
            QPushButton:disabled { color: #888888; }
            QTabBar::tab {
                background-color: #222222;
                padding: 8px 14px;
                border: 1px solid #4a4a4a;
                border-top-left-radius: 6px;
                border-top-right-radius: 6px;
                margin-right: 2px;
            }
            QTabBar::tab:selected { background-color: #343434; }
            """
        )

    @Slot()
    def start_links(self):
        self.stop_links()

        pi_host = self.pi_hostname_edit.text().strip()
        rtsp_url = self.rtsp_path_edit.text().strip()
        cmd_port = self.cmd_port_spin.value()
        telemetry_port = self.telemetry_port_spin.value()

        self.video_worker = VideoWorker(rtsp_url)
        self.video_worker.frame_ready.connect(self.on_video_frame)
        self.video_worker.stats_ready.connect(self.on_video_stats)
        self.video_worker.status_changed.connect(self.on_video_status)
        self.video_worker.start()

        self.udp_worker = UdpLinkWorker(pi_host, cmd_port, telemetry_port)
        self.udp_worker.telemetry_received.connect(self.on_telemetry_packet)
        self.udp_worker.log_message.connect(self.log_tab.append_log)
        self.udp_worker.link_state.connect(self.on_link_status)
        self.tuning_tab.tuning_changed.connect(self.udp_worker.update_tuning)
        self.udp_worker.start()

        self.connect_btn.setEnabled(False)
        self.disconnect_btn.setEnabled(True)
        self.connection_box.setVisible(False)
        self.disconnect_only_widget.setVisible(True)
        self.link_ready = False
        self._update_arm_buttons()
        self.status_bar.showMessage("Connecting to Pi...", 3000)
        self.log_tab.append_log("Connection started")
        self._update_video_overlay()

    @Slot()
    def stop_links(self):
        if self.video_worker is not None:
            self.video_worker.stop()
            self.video_worker.wait(1500)
            self.video_worker = None

        if self.udp_worker is not None:
            self.udp_worker.stop()
            self.udp_worker.wait(1500)
            self.udp_worker = None

        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.connection_box.setVisible(True)
        self.disconnect_only_widget.setVisible(False)
        self.link_ready = False
        self._update_arm_buttons()
        self.video_status.setText("Video: idle")
        self.link_status.setText("UDP: idle")
        self.video_label.setText("Video not connected")
        self.video_label.setPixmap(QPixmap())
        self._update_video_overlay()

    @Slot(QImage)
    def on_video_frame(self, image: QImage):
        pixmap = QPixmap.fromImage(image)
        scaled = pixmap.scaled(
            self.video_label.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )
        self.video_label.setPixmap(scaled)
        self._position_video_overlays()

    @Slot(dict)
    def on_video_stats(self, stats: dict):
        self.telemetry.video_fps = float(stats.get("video_fps", self.telemetry.video_fps))
        self.telemetry_panel.update_telemetry(self.telemetry)
        self._update_video_overlay()

    @Slot(str)
    def on_video_status(self, text: str):
        self.video_status.setText(f"Video: {text}")
        self.log_tab.append_log(text)

    @Slot(str)
    def on_link_status(self, text: str):
        self.link_status.setText(f"UDP: {text}")
        if text == "UDP link ready":
            self.link_ready = True
        elif text in ("UDP link failed", "UDP link stopped"):
            self.link_ready = False
        self._update_arm_buttons()
        self.log_tab.append_log(text)

    @Slot(dict)
    def on_telemetry_packet(self, payload: dict):
        data = payload.get("telemetry", payload)
        for key in asdict(self.telemetry).keys():
            if key in data:
                setattr(self.telemetry, key, data[key])
        self.telemetry_panel.update_telemetry(self.telemetry)
        self._update_arm_buttons()
        self._update_video_overlay()

    @Slot(dict)
    def on_tuning_changed(self, tuning: dict):
        self.log_tab.append_log(f"Live tuning update: {tuning}")
        if self.udp_worker is not None:
            self.udp_worker.update_tuning(tuning)

    def send_arm_state(self, armed: bool):
        self.controller_armed = armed
        self.telemetry.armed = armed
        self.telemetry_panel.update_telemetry(self.telemetry)
        self._update_arm_buttons()
        self._update_video_overlay()
        if self.udp_worker is not None:
            self.udp_worker.update_command({"arm": armed})
        self.log_tab.append_log("ARM command sent" if armed else "DISARM command sent")

    def toggle_demo(self):
        if self.demo_timer.isActive():
            self.demo_timer.stop()
            self.log_tab.append_log("Demo telemetry stopped")
        else:
            self.demo_timer.start(500)
            self.log_tab.append_log("Demo telemetry started")

    @Slot()
    def deploy_pi_folder(self):
        local_dir = Path(__file__).resolve().parent / "PI"
        remote_dir = "/home/pi/anglerfish"
        pi_user = self.pi_username_edit.text().strip() or "pi"
        pi_host = self.pi_hostname_edit.text().strip()
        pi_password = self._get_ssh_password()

        if not local_dir.exists() or not local_dir.is_dir():
            QMessageBox.warning(
                self,
                "Deploy Failed",
                f"Local folder not found:\n{local_dir}",
            )
            return

        if not pi_host:
            QMessageBox.warning(self, "Deploy Failed", "Pi hostname is empty.")
            return

        if pi_password:
            method_note = "Files will be copied directly via SFTP (password auth)."
        else:
            method_note = "Files will be deployed via Git push/pull (SSH key auth)."

        confirmation = QMessageBox.question(
            self,
            "Deploy to Pi",
            (
                f"Deploy local PI folder to:\n{pi_user}@{pi_host}:{remote_dir}\n\n"
                f"{method_note}\n\nContinue?"
            ),
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.Yes,
        )
        if confirmation != QMessageBox.Yes:
            return

        if pi_password and paramiko is None:
            QMessageBox.critical(
                self,
                "Deploy Failed",
                "Paramiko is required for password-based deploy.\nInstall with: pip install paramiko",
            )
            return

        self.status_bar.showMessage("Deploying to Pi...")
        self.log_tab.append_log(
            f"Deploy started: {local_dir} -> {pi_user}@{pi_host}:{remote_dir}"
        )

        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            if pi_password:
                ok, message = self._deploy_via_sftp(
                    host=pi_host,
                    user=pi_user,
                    password=pi_password,
                    local_dir=local_dir,
                    remote_dir=remote_dir,
                )
            else:
                ok, message = self._deploy_via_git(
                    host=pi_host,
                    user=pi_user,
                    local_dir=local_dir,
                    remote_dir=remote_dir,
                )
        finally:
            QApplication.restoreOverrideCursor()

        if ok:
            self.status_bar.showMessage("Deploy completed", 5000)
            self.log_tab.append_log("Deploy completed successfully")
            QMessageBox.information(self, "Deploy Complete", "PI files were updated successfully.")
            return

        self.status_bar.showMessage("Deploy failed", 8000)
        self.log_tab.append_log(f"Deploy failed: {message}")
        tip = (
            "Tip: Ensure Pi Password is filled in, or set up SSH key auth and use Git deploy."
            if not pi_password
            else "Tip: Verify Pi Username, Pi Password, and that the Pi is reachable."
        )
        QMessageBox.critical(
            self,
            "Deploy Failed",
            f"Could not deploy files.\n\nReason:\n{message}\n\n{tip}",
        )

    @Slot()
    def initialize_git_deploy(self):
        local_dir = Path(__file__).resolve().parent / "PI"
        remote_working_dir = "/home/pi/anglerfish"
        remote_bare_repo = "/home/pi/anglerfish.git"
        pi_user = self.pi_username_edit.text().strip() or "pi"
        pi_host = self.pi_hostname_edit.text().strip()

        if not local_dir.exists() or not local_dir.is_dir():
            QMessageBox.warning(
                self,
                "Init Failed",
                f"Local folder not found:\n{local_dir}",
            )
            return

        if not pi_host:
            QMessageBox.warning(self, "Init Failed", "Pi hostname is empty.")
            return

        local_remote_url = f"ssh://{pi_user}@{pi_host}{remote_bare_repo}"
        confirmation = QMessageBox.question(
            self,
            "Initialize Git Deploy",
            (
                "This sets up Git deploy automatically:\n\n"
                f"1) Local repo: {local_dir}\n"
                f"2) Local origin -> {local_remote_url}\n"
                f"3) Remote bare repo: {remote_bare_repo}\n"
                f"4) Remote working tree: {remote_working_dir}\n\n"
                "Continue?"
            ),
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.Yes,
        )
        if confirmation != QMessageBox.Yes:
            return

        self.status_bar.showMessage("Initializing Git deploy setup...")
        self.log_tab.append_log("Initializing Git deploy setup")

        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            ok, message = self._initialize_git_deploy(
                host=pi_host,
                user=pi_user,
                local_dir=local_dir,
                remote_working_dir=remote_working_dir,
                remote_bare_repo=remote_bare_repo,
            )
        finally:
            QApplication.restoreOverrideCursor()

        if ok:
            self.status_bar.showMessage("Git deploy initialized", 6000)
            self.log_tab.append_log("Git deploy initialized successfully")
            QMessageBox.information(
                self,
                "Init Complete",
                "Git deploy setup is complete. You can now use Deploy PI Folder via Git.",
            )
            return

        self.status_bar.showMessage("Git deploy init failed", 8000)
        self.log_tab.append_log(f"Git deploy init failed: {message}")
        QMessageBox.critical(self, "Init Failed", f"Could not initialize Git deploy.\n\n{message}")

    @Slot()
    def git_deploy_preflight_check(self):
        local_dir = Path(__file__).resolve().parent / "PI"
        remote_working_dir = "/home/pi/anglerfish"
        remote_bare_repo = "/home/pi/anglerfish.git"
        pi_user = self.pi_username_edit.text().strip() or "pi"
        pi_host = self.pi_hostname_edit.text().strip()

        if not pi_host:
            QMessageBox.warning(self, "Preflight Failed", "Pi hostname is empty.")
            return

        self.status_bar.showMessage("Running Git deploy preflight check...")
        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            ok, lines = self._run_git_deploy_preflight(
                host=pi_host,
                user=pi_user,
                local_dir=local_dir,
                remote_working_dir=remote_working_dir,
                remote_bare_repo=remote_bare_repo,
            )
        finally:
            QApplication.restoreOverrideCursor()

        for line in lines:
            self.log_tab.append_log(f"[Preflight] {line}")

        if ok:
            self.status_bar.showMessage("Git deploy preflight passed", 6000)
            QMessageBox.information(
                self,
                "Preflight Passed",
                "Git deploy preflight passed. Deploy should work.",
            )
        else:
            self.status_bar.showMessage("Git deploy preflight failed", 8000)
            QMessageBox.warning(
                self,
                "Preflight Issues Found",
                "Git deploy preflight found issues. See Log tab for details.",
            )

    def _deploy_via_git(
        self,
        host: str,
        user: str,
        local_dir: Path,
        remote_dir: str,
    ) -> tuple[bool, str]:
        try:
            local_dir_str = str(local_dir)
            local_repo_check = subprocess.run(
                ["git", "-C", local_dir_str, "rev-parse", "--is-inside-work-tree"],
                capture_output=True,
                text=True,
                check=False,
            )
            if local_repo_check.returncode != 0:
                return False, "Local PI folder is not a Git repository"

            origin_check = subprocess.run(
                ["git", "-C", local_dir_str, "remote", "get-url", "origin"],
                capture_output=True,
                text=True,
                check=False,
            )
            if origin_check.returncode != 0:
                return False, "Local PI repository has no 'origin' remote configured"

            dirty_check = subprocess.run(
                ["git", "-C", local_dir_str, "status", "--porcelain"],
                capture_output=True,
                text=True,
                check=False,
            )
            if dirty_check.returncode != 0:
                return False, (dirty_check.stderr or dirty_check.stdout or "Failed to check local repo state").strip()
            if dirty_check.stdout.strip():
                return False, "Local PI repository has uncommitted changes; commit or stash before deploy"

            ok, output = self._run_git_push_command(
                ["git", "-C", local_dir_str, "push", "origin", "main"]
            )
            if not ok:
                return False, output

            remote_quoted = shlex.quote(remote_dir)
            ssh_prefix = self._build_ssh_prefix(user, host)

            remote_pull_command = (
                f"mkdir -p {remote_quoted} && "
                f"cd {remote_quoted} && "
                "git rev-parse --is-inside-work-tree >/dev/null 2>&1 && "
                "git pull --ff-only"
            )
            remote_pull_result = subprocess.run(
                ssh_prefix + [remote_pull_command],
                capture_output=True,
                text=True,
                check=False,
            )
            if remote_pull_result.returncode != 0:
                return False, (
                    remote_pull_result.stderr
                    or remote_pull_result.stdout
                    or "Remote git pull failed (is /home/pi/anglerfish a Git repo?)"
                ).strip()

            return True, "OK"
        except FileNotFoundError:
            return False, "Required executable not found (git or ssh)"
        except Exception as exc:
            return False, str(exc)

    def _sftp_mkdir_p(self, sftp, remote_path: str) -> None:
        """Recursively create remote directories via an open SFTP session."""
        parts = [p for p in remote_path.split("/") if p]
        current = ""
        for part in parts:
            current += "/" + part
            try:
                sftp.stat(current)
            except IOError:
                sftp.mkdir(current)

    def _deploy_via_sftp(
        self,
        host: str,
        user: str,
        password: str,
        local_dir: Path,
        remote_dir: str,
    ) -> tuple[bool, str]:
        """Copy all files from local_dir to remote_dir on the Pi using Paramiko SFTP."""
        try:
            client = paramiko.SSHClient()
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            client.connect(
                hostname=host,
                username=user,
                password=password,
                timeout=8,
                auth_timeout=8,
                banner_timeout=8,
                look_for_keys=False,
                allow_agent=False,
            )
            sftp = client.open_sftp()
            self._sftp_mkdir_p(sftp, remote_dir)

            for local_file in sorted(local_dir.rglob("*")):
                if not local_file.is_file():
                    continue
                relative = local_file.relative_to(local_dir)
                remote_path = remote_dir + "/" + str(relative).replace("\\", "/")
                remote_parent = remote_dir + "/" + str(relative.parent).replace("\\", "/")
                if str(relative.parent) != ".":
                    self._sftp_mkdir_p(sftp, remote_parent)
                sftp.put(str(local_file), remote_path)

            sftp.close()
            client.close()
            return True, "OK"
        except Exception as exc:
            error_text = str(exc)
            hint = self._classify_ssh_error(user, host, error_text)
            if hint:
                return False, f"{error_text}\nHint: {hint}"
            return False, error_text

    def _run_local_command(self, args: list[str], cwd: Optional[str] = None) -> tuple[bool, str]:
        result = subprocess.run(
            args,
            cwd=cwd,
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode != 0:
            return False, (result.stderr or result.stdout or "Command failed").strip()
        return True, (result.stdout or "").strip()

    def _run_git_push_command(self, args: list[str]) -> tuple[bool, str]:
        password = self._get_ssh_password()
        if not password:
            return self._run_local_command(args)

        askpass_path = None
        try:
            with tempfile.NamedTemporaryFile("w", delete=False, suffix=".cmd", encoding="utf-8") as askpass_file:
                askpass_file.write("@echo off\r\n")
                askpass_file.write("echo %ANGLERFISH_SSH_PASSWORD%\r\n")
                askpass_path = askpass_file.name

            env = os.environ.copy()
            env["ANGLERFISH_SSH_PASSWORD"] = password
            env["SSH_ASKPASS"] = askpass_path
            env["SSH_ASKPASS_REQUIRE"] = "force"
            env["GIT_TERMINAL_PROMPT"] = "0"
            env["DISPLAY"] = "anglerfish"
            env["GIT_SSH_COMMAND"] = (
                "ssh -o BatchMode=no "
                "-o PreferredAuthentications=publickey,password,keyboard-interactive "
                "-o StrictHostKeyChecking=accept-new "
                "-o ConnectTimeout=8"
            )

            result = subprocess.run(
                args,
                capture_output=True,
                text=True,
                check=False,
                env=env,
            )
            if result.returncode != 0:
                error_text = (result.stderr or result.stdout or "git push failed").strip()
                hint = self._classify_ssh_error(
                    self.pi_username_edit.text().strip() or "pi",
                    self.pi_hostname_edit.text().strip(),
                    error_text,
                )
                if hint:
                    return False, f"{error_text}\nHint: {hint}"
                return False, error_text
            return True, (result.stdout or "").strip()
        finally:
            if askpass_path:
                try:
                    os.remove(askpass_path)
                except OSError:
                    pass

    def _build_ssh_prefix(self, user: str, host: str) -> list[str]:
        return [
            "ssh",
            "-o",
            "BatchMode=no",
            "-o",
            "PreferredAuthentications=publickey,password,keyboard-interactive",
            "-o",
            "StrictHostKeyChecking=accept-new",
            "-o",
            "ConnectTimeout=8",
            f"{user}@{host}",
        ]

    def _get_ssh_password(self) -> str:
        return self.pi_password_edit.text()

    def _classify_ssh_error(self, user: str, host: str, raw_error: str) -> Optional[str]:
        text = (raw_error or "").lower()
        if not text:
            return None

        if "permission denied" in text:
            return (
                f"Authentication failed for {user}@{host}. Verify Pi Username and Pi Password in the UI, then test with: "
                f"ssh {user}@{host}."
            )
        if "host key verification failed" in text or "remote host identification has changed" in text:
            return (
                f"Host key mismatch for {host}. Remove/update the old key in known_hosts, then reconnect."
            )
        if "could not resolve hostname" in text or "name or service not known" in text:
            return f"Hostname {host} could not be resolved. Check Pi Hostname or local DNS/mDNS."
        if "connection timed out" in text or "operation timed out" in text:
            return f"Connection to {host} timed out. Check network, power, and SSH service on the Pi."
        if "connection refused" in text:
            return f"SSH port refused on {host}. Ensure SSH is enabled and running on the Pi."
        if "no route to host" in text or "network is unreachable" in text:
            return f"No network route to {host}. Verify both devices are on the same reachable network."
        return None

    def _run_ssh_command(self, user: str, host: str, command: str) -> tuple[bool, str]:
        password = self._get_ssh_password()
        if password:
            if paramiko is None:
                return (
                    False,
                    "SSH password was provided, but Paramiko is not installed. Install with: pip install paramiko",
                )
            try:
                client = paramiko.SSHClient()
                client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                client.connect(
                    hostname=host,
                    username=user,
                    password=password,
                    timeout=8,
                    auth_timeout=8,
                    banner_timeout=8,
                    look_for_keys=False,
                    allow_agent=False,
                )
                stdin, stdout, stderr = client.exec_command(command, timeout=25)
                _ = stdin
                out_text = stdout.read().decode("utf-8", errors="replace").strip()
                err_text = stderr.read().decode("utf-8", errors="replace").strip()
                exit_code = stdout.channel.recv_exit_status()
                client.close()
                if exit_code != 0:
                    error_text = err_text or out_text or "SSH command failed"
                    hint = self._classify_ssh_error(user, host, error_text)
                    if hint:
                        return False, f"{error_text}\nHint: {hint}"
                    return False, error_text
                return True, out_text
            except Exception as exc:
                error_text = str(exc)
                hint = self._classify_ssh_error(user, host, error_text)
                if hint:
                    return False, f"{error_text}\nHint: {hint}"
                return False, error_text

        ssh_prefix = self._build_ssh_prefix(user, host)
        result = subprocess.run(
            ssh_prefix + [command],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode != 0:
            error_text = (result.stderr or result.stdout or "SSH command failed").strip()
            hint = self._classify_ssh_error(user, host, error_text)
            if hint:
                return False, f"{error_text}\nHint: {hint}"
            return False, error_text
        return True, (result.stdout or "").strip()

    def _initialize_git_deploy(
        self,
        host: str,
        user: str,
        local_dir: Path,
        remote_working_dir: str,
        remote_bare_repo: str,
    ) -> tuple[bool, str]:
        try:
            local_dir_str = str(local_dir)

            # Check if local_dir already sits inside a parent git repo (e.g. the main project repo).
            # If the git root is not local_dir itself, we must init a fresh repo scoped to local_dir
            # rather than contaminating the parent repo's remotes.
            root_check = self._run_local_command(
                ["git", "-C", local_dir_str, "rev-parse", "--show-toplevel"]
            )
            own_repo = root_check[0] and Path(root_check[1]).resolve() == local_dir.resolve()

            if not own_repo:
                # Initialise a new git repo scoped exactly to local_dir
                ok, output = self._run_local_command(
                    ["git", "-C", local_dir_str, "init", "-b", "main"]
                )
                if not ok:
                    ok, output = self._run_local_command(["git", "-C", local_dir_str, "init"])
                    if not ok:
                        return False, f"Local git init failed: {output}"
                    ok, output = self._run_local_command(
                        ["git", "-C", local_dir_str, "checkout", "-B", "main"]
                    )
                    if not ok:
                        return False, f"Local main branch setup failed: {output}"

            ok, output = self._run_local_command(["git", "-C", local_dir_str, "checkout", "-B", "main"])
            if not ok:
                return False, f"Local main branch setup failed: {output}"

            remote_bare_quoted = shlex.quote(remote_bare_repo)
            remote_setup_bare_command = f"mkdir -p {remote_bare_quoted} && git init --bare {remote_bare_quoted}"
            ok, output = self._run_ssh_command(user, host, remote_setup_bare_command)
            if not ok:
                return False, f"Remote bare repository setup failed: {output}"

            local_remote_url = f"ssh://{user}@{host}{remote_bare_repo}"
            ok, output = self._run_local_command(["git", "-C", local_dir_str, "remote", "remove", "origin"])
            if not ok and "No such remote" not in output:
                return False, f"Failed to reset local origin: {output}"

            ok, output = self._run_local_command(
                ["git", "-C", local_dir_str, "remote", "add", "origin", local_remote_url]
            )
            if not ok:
                return False, f"Failed to set local origin: {output}"

            ok, output = self._run_local_command(["git", "-C", local_dir_str, "add", "-A"])
            if not ok:
                return False, f"Failed to stage local files: {output}"

            ok, status_output = self._run_local_command(
                ["git", "-C", local_dir_str, "status", "--porcelain"]
            )
            if not ok:
                return False, f"Failed to inspect local status: {status_output}"

            if status_output.strip():
                ok, output = self._run_local_command(
                    ["git", "-C", local_dir_str, "commit", "-m", "Initialize PI deploy"]
                )
                if not ok and "nothing to commit" not in output:
                    return False, f"Initial local commit failed: {output}"

            ok, output = self._run_git_push_command(
                ["git", "-C", local_dir_str, "push", "-u", "origin", "main"]
            )
            if not ok:
                return False, f"Initial push failed: {output}"

            remote_working_quoted = shlex.quote(remote_working_dir)
            remote_setup_working_command = (
                f"mkdir -p {remote_working_quoted} && "
                f"git -C {remote_working_quoted} init -b main >/dev/null 2>&1 || git -C {remote_working_quoted} init && "
                f"git -C {remote_working_quoted} remote remove origin >/dev/null 2>&1 || true && "
                f"git -C {remote_working_quoted} remote add origin {remote_bare_quoted} && "
                f"git -C {remote_working_quoted} fetch origin main && "
                f"git -C {remote_working_quoted} checkout -B main && "
                f"git -C {remote_working_quoted} branch --set-upstream-to=origin/main main && "
                f"git -C {remote_working_quoted} pull --ff-only origin main"
            )
            ok, output = self._run_ssh_command(user, host, remote_setup_working_command)
            if not ok:
                return False, f"Remote working tree setup failed: {output}"

            return True, "OK"
        except FileNotFoundError:
            return False, "Required executable not found (git or ssh)"
        except Exception as exc:
            return False, str(exc)

    def _run_git_deploy_preflight(
        self,
        host: str,
        user: str,
        local_dir: Path,
        remote_working_dir: str,
        remote_bare_repo: str,
    ) -> tuple[bool, list[str]]:
        lines = []
        all_ok = True
        local_dir_str = str(local_dir)

        ok, output = self._run_local_command(["git", "--version"])
        if ok:
            lines.append(f"OK local git: {output}")
        else:
            all_ok = False
            lines.append(f"FAIL local git not available: {output}")

        ok, output = self._run_local_command(["ssh", "-V"])
        if ok:
            lines.append("OK local ssh executable found")
        else:
            all_ok = False
            lines.append(f"FAIL local ssh not available: {output}")

        if local_dir.exists() and local_dir.is_dir():
            lines.append(f"OK local PI folder exists: {local_dir}")
        else:
            all_ok = False
            lines.append(f"FAIL local PI folder missing: {local_dir}")

        ok, output = self._run_local_command(
            ["git", "-C", local_dir_str, "rev-parse", "--is-inside-work-tree"]
        )
        if ok:
            lines.append("OK local PI folder is a Git repository")
        else:
            all_ok = False
            lines.append(f"FAIL local PI folder is not a Git repository: {output}")

        ok, output = self._run_local_command(
            ["git", "-C", local_dir_str, "remote", "get-url", "origin"]
        )
        if ok:
            lines.append(f"OK local origin remote: {output}")
        else:
            all_ok = False
            lines.append(f"FAIL local origin remote missing: {output}")

        ok, output = self._run_local_command(["git", "-C", local_dir_str, "status", "--porcelain"])
        if ok and not output.strip():
            lines.append("OK local working tree clean")
        elif ok:
            all_ok = False
            lines.append("FAIL local working tree has uncommitted changes")
        else:
            all_ok = False
            lines.append(f"FAIL local status check failed: {output}")

        ok, output = self._run_ssh_command(user, host, "echo connected")
        if ok:
            lines.append(f"OK SSH connectivity to {user}@{host}")
        else:
            all_ok = False
            lines.append(f"FAIL SSH connectivity to {user}@{host}: {output}")
            return all_ok, lines

        ok, output = self._run_ssh_command(user, host, "git --version")
        if ok:
            lines.append(f"OK remote git: {output}")
        else:
            all_ok = False
            lines.append(f"FAIL remote git not available: {output}")

        remote_working_quoted = shlex.quote(remote_working_dir)
        remote_bare_quoted = shlex.quote(remote_bare_repo)

        ok, output = self._run_ssh_command(
            user,
            host,
            f"git -C {remote_working_quoted} rev-parse --is-inside-work-tree",
        )
        if ok:
            lines.append(f"OK remote working repo exists: {remote_working_dir}")
        else:
            all_ok = False
            lines.append(f"FAIL remote working repo missing or invalid at {remote_working_dir}: {output}")

        ok, output = self._run_ssh_command(
            user,
            host,
            f"git -C {remote_working_quoted} remote get-url origin",
        )
        if ok:
            lines.append(f"OK remote working origin: {output}")
        else:
            all_ok = False
            lines.append(f"FAIL remote working origin missing: {output}")

        ok, output = self._run_ssh_command(
            user,
            host,
            f"git -C {remote_working_quoted} rev-parse --abbrev-ref --symbolic-full-name @{{u}}",
        )
        if ok:
            lines.append(f"OK remote upstream branch: {output}")
        else:
            all_ok = False
            lines.append(f"FAIL remote upstream branch not set: {output}")

        ok, output = self._run_ssh_command(
            user,
            host,
            f"git --git-dir {remote_bare_quoted} rev-parse --is-bare-repository",
        )
        if ok and output.strip() == "true":
            lines.append(f"OK remote bare repository exists: {remote_bare_repo}")
        elif ok:
            all_ok = False
            lines.append(f"FAIL remote bare repository check returned unexpected value: {output}")
        else:
            all_ok = False
            lines.append(f"FAIL remote bare repository missing or invalid at {remote_bare_repo}: {output}")

        if all_ok:
            lines.append("SUMMARY: preflight passed")
        else:
            lines.append("SUMMARY: preflight found issues")

        return all_ok, lines

    def _demo_telemetry_tick(self):
        self.telemetry.timer_s += 1
        self.telemetry.battery_v = max(9.5, 16.2 - (self.telemetry.timer_s / 800.0))
        self.telemetry.pi_temp_c = 44.0 + ((self.telemetry.timer_s % 7) * 0.3)
        self.telemetry.depth_m = abs(np.sin(self.telemetry.timer_s / 20.0)) * 4.5
        self.telemetry.pressure_bar = 1.0 + (self.telemetry.depth_m / 10.0)
        self.telemetry.water_temp_c = 22.5
        self.telemetry.enclosure_temp_c = 27.0 + ((self.telemetry.timer_s % 5) * 0.2)
        self.telemetry.speed_mps = abs(np.sin(self.telemetry.timer_s / 10.0)) * 1.8
        self.telemetry.accel_mps2 = abs(np.cos(self.telemetry.timer_s / 8.0)) * 0.7
        self.telemetry.m1 = int(200 * np.sin(self.telemetry.timer_s / 5.0))
        self.telemetry.m2 = int(200 * np.cos(self.telemetry.timer_s / 5.0))
        self.telemetry.m3 = int(180 * np.sin(self.telemetry.timer_s / 7.0))
        self.telemetry.m4 = int(180 * np.cos(self.telemetry.timer_s / 7.0))
        self.telemetry.armed = True
        self.telemetry_panel.update_telemetry(self.telemetry)
        self._update_arm_buttons()
        self._update_video_overlay()

    def closeEvent(self, event):
        self._stop_controller_polling()
        self.stop_links()
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
