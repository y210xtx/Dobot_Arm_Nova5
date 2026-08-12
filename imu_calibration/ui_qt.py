# -*- coding: utf-8 -*-
"""PySide6 migration of the original Tkinter Dobot demo interface."""

import csv
import json
import math
import re
import statistics
import sys
import threading
import time
from pathlib import Path

import numpy as np
from PySide6.QtCore import QThread, Qt, QTimer, Signal
from PySide6.QtGui import QBrush, QColor, QFont, QPainter, QPen
from PySide6.QtCharts import QChart, QChartView, QLineSeries, QValueAxis
from PySide6.QtWidgets import (
    QApplication, QComboBox, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QLineEdit, QListWidget, QMainWindow, QMessageBox,
    QPushButton, QSplitter, QTabWidget, QTextEdit, QVBoxLayout, QWidget,
)

from dobot_api import DobotApiDashboard, DobotApiFeedBack
from files.alarmController import alarm_controller_list
from files.alarmServo import alarm_servo_list


JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5", "J6")
COORD_NAMES = ("X", "Y", "Z", "Rx", "Ry", "Rz")
ROBOT_MODES = {
    1: "初始化", 2: "抱闸松开", 3: "保留状态",
    4: "未使能", 5: "已使能", 6: "拖拽模式",
    7: "运行中", 8: "轨迹记录中", 9: "报警状态",
    10: "暂停", 11: "点动中",
}

POSE_RECORD_FILE = Path(__file__).resolve().with_name("recorded_pose.json")
TOOL_OFFSET_FILE = Path(__file__).resolve().with_name("tool_offset_config.json")
ROTATION_LOG_DIR = Path(__file__).resolve().with_name("rotation_logs")
ROTATION_SPEED_CALIBRATION_FILE = Path(__file__).resolve().with_name(
    "rotation_speed_calibration.json"
)
STATIC_IMU_CALIBRATION_FILE = Path(__file__).resolve().with_name(
    "static_imu_calibration_samples.json"
)

MODERN_STYLE = """
QWidget {
    color: #1f2937;
    font-family: "Microsoft YaHei UI", "Segoe UI", sans-serif;
    font-size: 13px;
}
QWidget#appBackground {
    background: #f3f6fa;
}
QLabel#connectionBadge {
    color: #b42318;
    background: #fef3f2;
    border: 1px solid #fecdca;
    border-radius: 14px;
    padding: 5px 12px;
    font-weight: 600;
}
QLabel#connectionBadge[connected="true"] {
    color: #027a48;
    background: #ecfdf3;
    border-color: #abefc6;
}
QGroupBox {
    background: #ffffff;
    border: 1px solid #dfe7f1;
    border-radius: 10px;
    margin-top: 14px;
    padding: 12px 10px 8px 10px;
    font-weight: 650;
    color: #344054;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 14px;
    padding: 0 6px;
    color: #344054;
    background: #ffffff;
}
QGroupBox#compactCard {
    margin-top: 12px;
    padding: 8px 7px 5px 7px;
}
QLineEdit, QComboBox {
    min-height: 25px;
    padding: 1px 7px;
    color: #1d2939;
    background: #ffffff;
    border: 1px solid #cbd5e1;
    border-radius: 6px;
    selection-background-color: #2563eb;
}
QLineEdit:hover, QComboBox:hover {
    border-color: #94a3b8;
}
QLineEdit:focus, QComboBox:focus {
    border: 2px solid #3b82f6;
    padding: 0 6px;
}
QComboBox::drop-down {
    width: 22px;
    border: none;
}
QComboBox QAbstractItemView {
    background: #ffffff;
    border: 1px solid #d0d5dd;
    selection-background-color: #eff6ff;
    selection-color: #1d4ed8;
    outline: none;
}
QPushButton {
    min-height: 26px;
    padding: 0 7px;
    color: #ffffff;
    background: #2563eb;
    border: 1px solid #2563eb;
    border-radius: 6px;
    font-weight: 600;
}
QPushButton:hover {
    background: #1d4ed8;
    border-color: #1d4ed8;
}
QPushButton:pressed {
    background: #1e40af;
    border-color: #1e40af;
}
QPushButton:disabled {
    color: #98a2b3;
    background: #f2f4f7;
    border-color: #e4e7ec;
}
QPushButton[jog="true"] {
    min-height: 20px;
    min-width: 34px;
    padding: 0 2px;
    color: #344054;
    background: #ffffff;
    border-color: #cbd5e1;
    font-weight: 500;
}
QPushButton[jog="true"]:hover {
    color: #1d4ed8;
    background: #eff6ff;
    border-color: #93c5fd;
}
QPushButton[accent="true"] {
    background: #0f766e;
    border-color: #0f766e;
}
QPushButton[accent="true"]:hover {
    background: #0d5f59;
    border-color: #0d5f59;
}
QPushButton[warning="true"] {
    color: #ffffff;
    background: #d97706;
    border-color: #d97706;
}
QPushButton[warning="true"]:hover {
    background: #b45309;
    border-color: #b45309;
}
QPushButton[danger="true"] {
    color: #ffffff;
    background: #dc2626;
    border-color: #dc2626;
}
QPushButton[danger="true"]:hover {
    background: #b91c1c;
    border-color: #b91c1c;
}
QPushButton[jog="true"]:disabled,
QPushButton[accent="true"]:disabled,
QPushButton[warning="true"]:disabled,
QPushButton[danger="true"]:disabled {
    color: #98a2b3;
    background: #f2f4f7;
    border-color: #e4e7ec;
}
QTextEdit {
    color: #344054;
    background: #f8fafc;
    border: 1px solid #e2e8f0;
    border-radius: 7px;
    padding: 6px;
    selection-background-color: #bfdbfe;
}
QTextEdit#logText {
    font-family: "Cascadia Mono", "Consolas", monospace;
    font-size: 12px;
}
QLabel[value="true"] {
    color: #175cd3;
    font-weight: 700;
}
QLabel[ioValue="true"] {
    color: #475467;
    font-family: "Cascadia Mono", "Consolas", monospace;
    font-size: 11px;
}
QTabWidget#cardTabs::pane {
    border: 1px solid #dfe7f1;
    border-radius: 10px;
    background: #ffffff;
    top: 0;
}
QTabWidget#cardTabs QTabBar::tab {
    min-width: 86px;
    min-height: 26px;
    padding: 2px 12px;
    margin-right: 4px;
    color: #475467;
    background: #f8fafc;
    border: 1px solid #dfe7f1;
    border-bottom-color: #dfe7f1;
    border-top-left-radius: 7px;
    border-top-right-radius: 7px;
    font-weight: 600;
}
QTabWidget#cardTabs QTabBar::tab:selected {
    color: #0f766e;
    background: #ffffff;
    border-bottom-color: #ffffff;
}
QTabWidget#cardTabs QTabBar::tab:hover {
    color: #1d4ed8;
    background: #eff6ff;
}
QSplitter::handle {
    background: transparent;
}
QSplitter::handle:hover {
    background: #dbeafe;
    border-radius: 3px;
}
QScrollBar:vertical {
    width: 9px;
    margin: 2px;
    background: transparent;
}
QScrollBar::handle:vertical {
    min-height: 25px;
    background: #cbd5e1;
    border-radius: 4px;
}
QScrollBar::handle:vertical:hover {
    background: #94a3b8;
}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0;
}
"""


class FeedbackThread(QThread):
    """Receive 30004 feedback without blocking the Qt event loop."""

    feedback = Signal(object)
    connection_lost = Signal(str)

    def __init__(self, client, parent=None):
        super().__init__(parent)
        self.client = client
        self.running = True

    def stop(self):
        self.running = False
        try:
            self.client.close()
        except Exception:
            pass

    def run(self):
        while self.running:
            try:
                packet = self.client.feedBackData()
                if packet is None or len(packet) == 0:
                    continue
                if int(packet["TestValue"][0]) != 0x123456789ABCDEF:
                    continue
                self.feedback.emit({
                    "speed": float(packet["SpeedScaling"][0]),
                    "mode": int(packet["RobotMode"][0]),
                    "controller_timestamp": int(packet["TimeStamp"][0]),
                    "di": int(packet["DigitalInputs"][0]),
                    "do": int(packet["DigitalOutputs"][0]),
                    "joints": np.asarray(packet["QActual"][0], dtype=float).tolist(),
                    "pose": np.asarray(packet["ToolVectorActual"][0], dtype=float).tolist(),
                    "user": int(packet["User"][0]),
                    "tool": int(packet["Tool"][0]),
                    "angular_speed": np.asarray(
                        packet["TCPSpeedActual"][0][3:6], dtype=float
                    ).tolist(),
                    "tcp_speed": np.asarray(
                        packet["TCPSpeedActual"][0], dtype=float
                    ).tolist(),
                })
            except Exception as exc:
                if self.running:
                    self.connection_lost.emit(str(exc))
                return


class AlarmThread(QThread):
    """Read alarm details in the background because HTTP can time out."""

    result = Signal(object)

    def __init__(self, client, controller_alarms, servo_alarms, parent=None):
        super().__init__(parent)
        self.client = client
        self.controller_alarms = controller_alarms
        self.servo_alarms = servo_alarms

    def run(self):
        try:
            response = self.client.GetError("zh_cn")
            if response and response.get("errMsg"):
                self.result.emit(("new", response["errMsg"]))
                return
        except Exception:
            pass

        # The window may have been disconnected while the HTTP request waited.
        if self.isInterruptionRequested():
            return

        try:
            raw = self.client.GetErrorID()
            error_list = json.loads(raw.split("{", 1)[1].split("}", 1)[0])
            errors = []
            for error_id in (error_list[0] if error_list else []):
                errors.append((error_id, self.controller_alarms, "控制器报警"))
            for servo_errors in error_list[1:]:
                for error_id in servo_errors or []:
                    errors.append((error_id, self.servo_alarms, "伺服报警"))
            self.result.emit(("legacy", errors))
        except Exception as exc:
            self.result.emit(("failure", str(exc)))


class RotationDiagnostics:
    """Thread-safe CSV recorder and summary generator for one rotation run."""

    FIELDS = (
        "event", "pc_time_s", "controller_timestamp", "robot_mode", "axis",
        "target_speed_deg_s", "target_delta_deg", "selected_actual_deg_s",
        "speed_error_deg_s", "command_interval_ms", "command_latency_ms",
        "target_x", "target_y", "target_z", "target_rx", "target_ry", "target_rz",
        "actual_x", "actual_y", "actual_z", "actual_rx", "actual_ry", "actual_rz",
        "tcp_vx", "tcp_vy", "tcp_vz", "tcp_wx", "tcp_wy", "tcp_wz", "response",
    )

    def __init__(
        self, axis, target_speed, duration, start_pose, user_index, tool_index,
        axis_frame="euler"
    ):
        ROTATION_LOG_DIR.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        fraction = int((time.time() % 1) * 1000)
        self.path = ROTATION_LOG_DIR / f"rotation_{stamp}_{fraction:03d}.csv"
        self.summary_path = self.path.with_suffix(".summary.json")
        self.axis = axis
        self.axis_frame = axis_frame
        self.axis_index = {"Rx": 0, "Ry": 1, "Rz": 2}[axis]
        self.target_speed = float(target_speed)
        self.duration = float(duration)
        self.started_at = time.perf_counter()
        self.lock = threading.Lock()
        self.file = self.path.open("w", newline="", encoding="utf-8-sig")
        self.writer = csv.DictWriter(self.file, fieldnames=self.FIELDS)
        self.writer.writeheader()
        self.rows_since_flush = 0
        self.closed = False
        self.summary = None
        self.last_target = list(start_pose)
        self.last_target_delta = 0.0
        self.last_commanded_speed = 0.0
        self.last_axis_unit = self._axis_unit(start_pose)
        self.command_intervals = []
        self.command_latencies = []
        self.feedback_samples = []
        self.metadata = {
            "axis": axis,
            "target_speed_deg_s": self.target_speed,
            "duration_s": self.duration,
            "user": int(user_index),
            "tool": int(tool_index),
            "axis_frame": axis_frame,
            "start_pose": list(start_pose),
            "csv_file": str(self.path),
        }

    def _write(self, values):
        row = {field: "" for field in self.FIELDS}
        row.update(values)
        self.writer.writerow(row)
        self.rows_since_flush += 1
        if self.rows_since_flush >= 25:
            self.file.flush()
            self.rows_since_flush = 0

    def _axis_unit(self, pose):
        """Map the requested rotation axis into the base angular-velocity frame."""
        rx = math.radians(float(pose[3]))
        ry = math.radians(float(pose[4]))
        rz = math.radians(float(pose[5]))
        if self.axis_frame == "tool":
            # Columns of Rz(rz) * Ry(ry) * Rx(rx): the local Tool axes in base.
            if self.axis == "Rx":
                return (
                    math.cos(rz) * math.cos(ry),
                    math.sin(rz) * math.cos(ry),
                    -math.sin(ry),
                )
            if self.axis == "Ry":
                return (
                    math.cos(rz) * math.sin(ry) * math.sin(rx)
                    - math.sin(rz) * math.cos(rx),
                    math.sin(rz) * math.sin(ry) * math.sin(rx)
                    + math.cos(rz) * math.cos(rx),
                    math.cos(ry) * math.sin(rx),
                )
            return (
                math.cos(rz) * math.sin(ry) * math.cos(rx)
                + math.sin(rz) * math.sin(rx),
                math.sin(rz) * math.sin(ry) * math.cos(rx)
                - math.cos(rz) * math.sin(rx),
                math.cos(ry) * math.cos(rx),
            )
        if self.axis == "Rx":
            return (
                math.cos(rz) * math.cos(ry),
                math.sin(rz) * math.cos(ry),
                -math.sin(ry),
            )
        if self.axis == "Ry":
            return (-math.sin(rz), math.cos(rz), 0.0)
        return (0.0, 0.0, 1.0)

    def configure_motion(self, controller_v, global_speed_scale, method):
        self.metadata.update({
            "controller_v_percent": int(controller_v),
            "global_speed_scale": float(global_speed_scale),
            "control_method": str(method),
        })

    def log_command(
        self, elapsed, target, rotation_angle, commanded_speed,
        interval_ms, latency_ms, response
    ):
        with self.lock:
            if self.closed:
                return
            self.last_target = list(target)
            self.last_target_delta = float(rotation_angle)
            self.last_commanded_speed = float(commanded_speed)
            self.last_axis_unit = self._axis_unit(target)
            if interval_ms is not None:
                self.command_intervals.append(float(interval_ms))
            self.command_latencies.append(float(latency_ms))
            self._write({
                "event": "command",
                "pc_time_s": f"{elapsed:.6f}",
                "axis": self.axis,
                "target_speed_deg_s": f"{self.last_commanded_speed:.6f}",
                "target_delta_deg": f"{self.last_target_delta:.6f}",
                "command_interval_ms": "" if interval_ms is None else f"{interval_ms:.3f}",
                "command_latency_ms": f"{latency_ms:.3f}",
                **{
                    f"target_{name}": f"{value:.6f}"
                    for name, value in zip(("x", "y", "z", "rx", "ry", "rz"), target)
                },
                "response": str(response).strip(),
            })

    def log_feedback(self, state):
        elapsed = time.perf_counter() - self.started_at
        angular_speed = [float(value) for value in state["angular_speed"]]
        pose = state["pose"]
        tcp_speed = state["tcp_speed"]
        with self.lock:
            if self.closed:
                return
            # TCPSpeedActual is expressed as a physical angular-velocity vector.
            # An Euler angle rate is not generally the same as the equally named
            # vector component (for example Ry at Rz=180 deg maps to -Wy).
            axis_unit = (
                self._axis_unit(pose) if self.axis_frame == "tool" else self.last_axis_unit
            )
            selected_speed = sum(
                value * direction
                for value, direction in zip(angular_speed, axis_unit)
            )
            commanded_speed = self.last_commanded_speed
            error = selected_speed - commanded_speed
            self.feedback_samples.append(
                (elapsed, selected_speed, commanded_speed, *angular_speed)
            )
            self._write({
                "event": "feedback",
                "pc_time_s": f"{elapsed:.6f}",
                "controller_timestamp": state["controller_timestamp"],
                "robot_mode": state["mode"],
                "axis": self.axis,
                "target_speed_deg_s": f"{commanded_speed:.6f}",
                "target_delta_deg": f"{self.last_target_delta:.6f}",
                "selected_actual_deg_s": f"{selected_speed:.6f}",
                "speed_error_deg_s": f"{error:.6f}",
                **{
                    f"target_{name}": f"{value:.6f}"
                    for name, value in zip(
                        ("x", "y", "z", "rx", "ry", "rz"), self.last_target
                    )
                },
                **{
                    f"actual_{name}": f"{value:.6f}"
                    for name, value in zip(("x", "y", "z", "rx", "ry", "rz"), pose)
                },
                **{
                    f"tcp_{name}": f"{value:.6f}"
                    for name, value in zip(("vx", "vy", "vz", "wx", "wy", "wz"), tcp_speed)
                },
            })

    @staticmethod
    def _stats(values):
        if not values:
            return {"count": 0, "mean": None, "std": None, "min": None, "max": None}
        return {
            "count": len(values),
            "mean": statistics.fmean(values),
            "std": statistics.pstdev(values) if len(values) > 1 else 0.0,
            "min": min(values),
            "max": max(values),
        }

    def finish(self, reason):
        with self.lock:
            if self.closed:
                return self.summary
            # Only evaluate the constant-speed portion, excluding ramps.
            steady = [
                sample for sample in self.feedback_samples
                if sample[0] >= 0.5
                and abs(sample[2]) >= abs(self.target_speed) * 0.8
            ]
            selected = [sample[1] for sample in steady]
            other_indices = [index for index in range(3) if index != self.axis_index]
            cross_axis_rms = {}
            for index in other_indices:
                values = [sample[3 + index] for sample in steady]
                cross_axis_rms[("Rx", "Ry", "Rz")[index]] = (
                    math.sqrt(statistics.fmean(value * value for value in values)) if values else None
                )
            errors = [sample[1] - sample[2] for sample in steady]
            candidates = [
                sample[1] for sample in self.feedback_samples
                if sample[0] >= 0.3 and sample[1] * self.target_speed > 0
            ]
            plateau = []
            if candidates:
                magnitudes = sorted(abs(value) for value in candidates)
                p90 = magnitudes[min(len(magnitudes) - 1, int(len(magnitudes) * 0.9))]
                plateau = [value for value in candidates if abs(value) >= p90 * 0.8]
            summary = {
                **self.metadata,
                "finished_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "finish_reason": reason,
                "feedback": self._stats(selected),
                "speed_error": self._stats(errors),
                "plateau_feedback": self._stats(plateau),
                "cross_axis_rms_deg_s": cross_axis_rms,
                "command_interval_ms": self._stats(self.command_intervals),
                "command_latency_ms": self._stats(self.command_latencies),
                "feedback_total_count": len(self.feedback_samples),
            }
            self.file.flush()
            self.file.close()
            self.closed = True
            self.summary = summary
        self.summary_path.write_text(
            json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        return summary


class RotationSequenceDiagnostics:
    """Record planned and actual angular-velocity vectors for a blended plan."""

    FIELDS = (
        "event", "pc_time_s", "controller_timestamp", "robot_mode", "segment",
        "planned_rx_deg_s", "planned_ry_deg_s", "planned_rz_deg_s", "cp",
        "actual_wx_deg_s", "actual_wy_deg_s", "actual_wz_deg_s", "latency_ms",
        "response",
    )

    def __init__(self):
        ROTATION_LOG_DIR.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.path = ROTATION_LOG_DIR / f"rotation_sequence_{stamp}_{int((time.time() % 1) * 1000):03d}.csv"
        self.summary_path = self.path.with_suffix(".summary.json")
        self.started_at = time.perf_counter()
        self.file = self.path.open("w", newline="", encoding="utf-8-sig")
        self.writer = csv.DictWriter(self.file, fieldnames=self.FIELDS)
        self.writer.writeheader()
        self.lock = threading.Lock()
        self.closed = False
        self.current_speed = [0.0, 0.0, 0.0]
        self.current_segment = 0
        self.samples = []

    def _write(self, row):
        self.writer.writerow({field: "" for field in self.FIELDS} | row)

    def log_command(self, elapsed, segment, cp, latency_ms, response):
        with self.lock:
            if self.closed:
                return
            self.current_speed = [float(value) for value in segment["speed_vector"]]
            self.current_segment += 1
            self._write({
                "event": "command", "pc_time_s": f"{elapsed:.6f}",
                "segment": self.current_segment, "cp": cp,
                "planned_rx_deg_s": self.current_speed[0],
                "planned_ry_deg_s": self.current_speed[1],
                "planned_rz_deg_s": self.current_speed[2],
                "latency_ms": f"{latency_ms:.3f}", "response": str(response).strip(),
            })

    def log_feedback(self, state):
        with self.lock:
            if self.closed:
                return
            actual = [float(value) for value in state["angular_speed"]]
            elapsed = time.perf_counter() - self.started_at
            self.samples.append((elapsed, self.current_segment, *self.current_speed, *actual))
            self._write({
                "event": "feedback", "pc_time_s": f"{elapsed:.6f}",
                "controller_timestamp": state["controller_timestamp"], "robot_mode": state["mode"],
                "segment": self.current_segment,
                "planned_rx_deg_s": self.current_speed[0],
                "planned_ry_deg_s": self.current_speed[1],
                "planned_rz_deg_s": self.current_speed[2],
                "actual_wx_deg_s": actual[0], "actual_wy_deg_s": actual[1], "actual_wz_deg_s": actual[2],
            })

    def finish(self, reason):
        with self.lock:
            if self.closed:
                return None
            magnitudes = [math.sqrt(x * x + y * y + z * z) for _, _, x, y, z, _, _, _ in self.samples]
            summary = {
                "finish_reason": reason, "csv_file": str(self.path),
                "feedback_count": len(self.samples),
                "actual_angular_speed_norm_deg_s": RotationDiagnostics._stats(magnitudes),
            }
            self.file.flush()
            self.file.close()
            self.closed = True
        self.summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
        return summary


class RotationThread(QThread):
    """Queue controller-planned relative Tool-axis rotations for IMU calibration."""

    progress = Signal(float)
    rotation_done = Signal(str)
    rotation_error = Signal(str)

    def __init__(
        self, client, start_pose, axis, angular_speed, duration, controller_v,
        state_provider, diagnostics=None, parent=None
    ):
        super().__init__(parent)
        self.client = client
        self.start_pose = list(start_pose)
        self.axis = axis
        self.angular_speed = float(angular_speed)
        self.duration = float(duration)
        self.controller_v = int(controller_v)
        self.state_provider = state_provider
        # A finite IMU sweep should be one controller trajectory whenever possible.
        # RelMovLTool segment boundaries visibly decelerate even with cp=100, so only
        # split sweeps above 170 deg. Manual/infinite motion keeps a smaller safety
        # segment because Stop() is then the only normal termination mechanism.
        self.finite_segment_angle = 170.0
        self.manual_segment_angle = 60.0
        self.running = True
        self.diagnostics = diagnostics

    def stop(self):
        self.running = False

    def _offset(self, angle):
        values = [0.0] * 6
        values[{"Rx": 3, "Ry": 4, "Rz": 5}[self.axis]] = angle
        return values

    @staticmethod
    def _error_id(response):
        match = re.match(r"\s*(-?\d+)", str(response))
        return int(match.group(1)) if match else None

    def _send_segment(self, angle, cumulative_angle, elapsed, cp, last_send_started):
        send_started = time.perf_counter()
        interval_ms = (
            None if last_send_started is None
            else (send_started - last_send_started) * 1000.0
        )
        response = self.client.RelMovLTool(
            *self._offset(angle),
            user=-1,
            tool=-1,
            a=30,
            v=self.controller_v,
            cp=cp,
        )
        latency_ms = (time.perf_counter() - send_started) * 1000.0
        if self.diagnostics is not None:
            self.diagnostics.log_command(
                elapsed,
                self.start_pose,
                cumulative_angle,
                self.angular_speed,
                interval_ms,
                latency_ms,
                response,
            )
        error_id = self._error_id(response)
        if error_id not in (0, None):
            raise RuntimeError(f"RelMovLTool 返回错误：{response}")
        return send_started

    def _wait_for_finite_motion(self, started_at, total_angle):
        expected_seconds = total_angle / abs(self.angular_speed)
        deadline = started_at + max(12.0, expected_seconds * 4.0 + 8.0)
        seen_running = False
        while self.running and time.perf_counter() < deadline:
            elapsed = time.perf_counter() - started_at
            mode = self.state_provider()
            if mode == 7:
                seen_running = True
            if (seen_running and mode == 5) or (elapsed > 0.8 and mode == 5):
                return
            self.progress.emit(
                math.copysign(min(total_angle, elapsed * abs(self.angular_speed)), self.angular_speed)
            )
            self.msleep(50)
        if self.running:
            raise TimeoutError("等待 Tool 轴旋转完成超时，已停止继续下发")

    def _run_finite(self, started_at):
        total_angle = abs(self.angular_speed) * self.duration
        direction = 1.0 if self.angular_speed > 0 else -1.0
        segment_count = max(1, math.ceil(total_angle / self.finite_segment_angle))
        segments = [self.finite_segment_angle] * segment_count
        segments[-1] = total_angle - self.finite_segment_angle * (segment_count - 1)
        cumulative = 0.0
        last_send_started = None
        next_enqueue = started_at
        for index, amount in enumerate(segments):
            # Keep at most two nominal segments queued so long runs cannot overflow
            # the controller command queue.
            if index >= 2:
                while self.running and time.perf_counter() < next_enqueue:
                    elapsed = time.perf_counter() - started_at
                    self.progress.emit(
                        math.copysign(
                            min(total_angle, elapsed * abs(self.angular_speed)),
                            self.angular_speed,
                        )
                    )
                    self.msleep(50)
            if not self.running:
                return
            signed_amount = direction * amount
            cumulative += signed_amount
            # Queue adjacent segments with full blending; the final segment ends normally.
            cp = 100 if index < len(segments) - 1 else 0
            last_send_started = self._send_segment(
                signed_amount,
                cumulative,
                time.perf_counter() - started_at,
                cp,
                last_send_started,
            )
            if index == 1:
                next_enqueue = (
                    time.perf_counter() + self.finite_segment_angle / abs(self.angular_speed)
                )
            elif index >= 2:
                next_enqueue = time.perf_counter() + amount / abs(self.angular_speed)
        if self.running:
            self._wait_for_finite_motion(started_at, total_angle)

    def _run_manual(self, started_at):
        direction = 1.0 if self.angular_speed > 0 else -1.0
        signed_segment = direction * self.manual_segment_angle
        expected_segment_time = self.manual_segment_angle / abs(self.angular_speed)
        cumulative = 0.0
        last_send_started = None

        # Keep two controller-planned segments queued, then replenish at their
        # nominal consumption rate. This avoids PC-period interpolation jitter.
        for _ in range(2):
            cumulative += signed_segment
            last_send_started = self._send_segment(
                signed_segment,
                cumulative,
                time.perf_counter() - started_at,
                100,
                last_send_started,
            )
        next_enqueue = time.perf_counter() + expected_segment_time
        while self.running:
            now = time.perf_counter()
            if now >= next_enqueue:
                cumulative += signed_segment
                last_send_started = self._send_segment(
                    signed_segment,
                    cumulative,
                    now - started_at,
                    100,
                    last_send_started,
                )
                next_enqueue = now + expected_segment_time
            self.progress.emit(
                math.copysign((now - started_at) * abs(self.angular_speed), self.angular_speed)
            )
            self.msleep(50)

    def run(self):
        started_at = time.perf_counter()
        try:
            if self.duration > 0:
                self._run_finite(started_at)
            else:
                self._run_manual(started_at)
            message = "旋转已完成" if self.running else "旋转已手动停止"
            self.rotation_done.emit(message)
        except Exception as exc:
            self.rotation_error.emit(str(exc))


class RotationPlanThread(QThread):
    """Queue a finite IMU rotation plan as one blended controller trajectory."""

    progress = Signal(object)
    rotation_done = Signal(str)
    rotation_error = Signal(str)

    def __init__(self, client, items, state_provider, cp=100, diagnostics=None, parent=None):
        super().__init__(parent)
        self.client = client
        self.items = [dict(item) for item in items]
        self.state_provider = state_provider
        self.cp = int(cp)
        self.diagnostics = diagnostics
        self.segment_angle = 170.0
        self.running = True

    def stop(self):
        self.running = False

    @staticmethod
    def _error_id(response):
        match = re.match(r"\s*(-?\d+)", str(response))
        return int(match.group(1)) if match else None

    @staticmethod
    def _offset(axis, angle):
        values = [0.0] * 6
        values[{"Rx": 3, "Ry": 4, "Rz": 5}[axis]] = angle
        return values

    def _segments(self):
        segments = []
        for index, item in enumerate(self.items):
            if item.get("type") == "vector":
                deltas = [
                    float(value) * float(item["duration"])
                    for value in item["angular_speed_vector"]
                ]
                max_delta = max(abs(value) for value in deltas)
                count = max(1, math.ceil(max_delta / self.segment_angle))
                for _ in range(count):
                    offset = [0.0, 0.0, 0.0] + [value / count for value in deltas]
                    segments.append({
                        "item_index": index,
                        "axis": "组合",
                        "angle": math.sqrt(sum(value * value for value in offset[3:])),
                        "offset": offset,
                        "controller_v": int(item["controller_v"]),
                        "speed_vector": list(item["angular_speed_vector"]),
                    })
            else:
                total_angle = abs(item["angular_speed"]) * item["duration"]
                direction = 1.0 if item["angular_speed"] > 0 else -1.0
                count = max(1, math.ceil(total_angle / self.segment_angle))
                amounts = [self.segment_angle] * count
                amounts[-1] = total_angle - self.segment_angle * (count - 1)
                for amount in amounts:
                    segments.append({
                        "item_index": index,
                        "axis": item["axis"],
                        "angle": direction * amount,
                        "offset": self._offset(item["axis"], direction * amount),
                        "controller_v": int(item["controller_v"]),
                        "speed_vector": self._offset(item["axis"], item["angular_speed"])[3:],
                    })
        return segments

    def run(self):
        started_at = time.perf_counter()
        try:
            segments = self._segments()
            if not segments:
                self.rotation_done.emit("运动序列为空")
                return

            last_send_started = None
            for index, segment in enumerate(segments):
                if not self.running:
                    break
                send_started = time.perf_counter()
                interval_ms = (
                    None if last_send_started is None
                    else (send_started - last_send_started) * 1000.0
                )
                # Only intermediate queued points blend into their successor; the
                # last point must finish normally rather than blend past the plan.
                cp = self.cp if index < len(segments) - 1 else 0
                response = self.client.RelMovLTool(
                    *segment["offset"],
                    user=-1,
                    tool=-1,
                    a=30,
                    v=segment["controller_v"],
                    cp=cp,
                )
                latency_ms = (time.perf_counter() - send_started) * 1000.0
                error_id = self._error_id(response)
                if error_id not in (0, None):
                    raise RuntimeError(f"RelMovLTool 序列返回错误：{response}")
                if self.diagnostics is not None:
                    self.diagnostics.log_command(
                        time.perf_counter() - started_at, segment, cp, latency_ms, response
                    )
                self.progress.emit({
                    "item_index": segment["item_index"],
                    "axis": segment["axis"],
                    "angle": segment["angle"],
                    "phase": "queued",
                    "interval_ms": interval_ms,
                    "latency_ms": latency_ms,
                })
                last_send_started = send_started
                self.msleep(5)

            expected_total = sum(item["duration"] for item in self.items)
            deadline = started_at + max(12.0, expected_total * 4.0 + 8.0)
            seen_running = False
            cumulative_times = []
            total = 0.0
            for item in self.items:
                total += item["duration"]
                cumulative_times.append(total)

            while self.running and time.perf_counter() < deadline:
                elapsed = time.perf_counter() - started_at
                mode = self.state_provider()
                if mode == 7:
                    seen_running = True
                item_index = 0
                for index, end_time in enumerate(cumulative_times):
                    if elapsed <= end_time:
                        item_index = index
                        break
                else:
                    item_index = len(self.items) - 1
                item = self.items[item_index]
                item_start = 0.0 if item_index == 0 else cumulative_times[item_index - 1]
                item_elapsed = max(0.0, elapsed - item_start)
                if item.get("type") == "vector":
                    speed_norm = math.sqrt(
                        sum(value * value for value in item["angular_speed_vector"])
                    )
                    angle = min(speed_norm * item["duration"], item_elapsed * speed_norm)
                else:
                    angle = math.copysign(
                        min(
                            abs(item["angular_speed"]) * item["duration"],
                            item_elapsed * abs(item["angular_speed"]),
                        ),
                        item["angular_speed"],
                    )
                self.progress.emit({
                    "item_index": item_index,
                    "axis": item["axis"],
                    "angle": angle,
                    "phase": "running",
                })
                if (seen_running and mode == 5) or (elapsed > expected_total + 0.8 and mode == 5):
                    break
                self.msleep(50)
            if self.running and time.perf_counter() >= deadline:
                raise TimeoutError("等待 IMU 运动序列完成超时")

            message = "运动序列已完成" if self.running else "运动序列已手动停止"
            self.rotation_done.emit(message)
        except Exception as exc:
            self.rotation_error.emit(str(exc))


class JogRotationThread(QThread):
    """Controller-internal continuous Tool-axis jog for long IMU sweeps."""

    progress = Signal(float)
    rotation_done = Signal(str)
    rotation_error = Signal(str)

    def __init__(
        self, client, start_pose, axis, angular_speed, duration, user_index,
        tool_index, diagnostics=None, parent=None
    ):
        super().__init__(parent)
        self.client = client
        self.start_pose = list(start_pose)
        self.axis = axis
        self.angular_speed = float(angular_speed)
        self.duration = float(duration)
        self.user_index = int(user_index)
        self.tool_index = int(tool_index)
        self.diagnostics = diagnostics
        self.running = True

    def stop(self):
        self.running = False

    @staticmethod
    def _error_id(response):
        match = re.match(r"\s*(-?\d+)", str(response))
        return int(match.group(1)) if match else None

    def run(self):
        command = f"{self.axis}{'+' if self.angular_speed > 0 else '-'}"
        started_at = time.perf_counter()
        try:
            send_started = time.perf_counter()
            response = self.client.MoveJog(
                command, coordtype=2, user=self.user_index, tool=self.tool_index
            )
            latency_ms = (time.perf_counter() - send_started) * 1000.0
            if self.diagnostics is not None:
                target_angle = self.angular_speed * self.duration if self.duration > 0 else 0.0
                self.diagnostics.log_command(
                    0.0, self.start_pose, target_angle, self.angular_speed,
                    None, latency_ms, response
                )
            if self._error_id(response) not in (0, None):
                raise RuntimeError(f"Tool MoveJog 返回错误：{response}")

            while self.running:
                elapsed = time.perf_counter() - started_at
                if self.duration > 0 and elapsed >= self.duration:
                    break
                self.progress.emit(self.angular_speed * elapsed)
                self.msleep(25)

            stop_response = self.client.MoveJog("")
            if self._error_id(stop_response) not in (0, None):
                raise RuntimeError(f"停止 Tool MoveJog 返回错误：{stop_response}")
            message = "旋转已完成" if self.running else "旋转已手动停止"
            self.rotation_done.emit(message)
        except Exception as exc:
            try:
                self.client.MoveJog("")
            except Exception:
                pass
            self.rotation_error.emit(str(exc))


class RobotUI(QMainWindow):
    """Dobot dashboard, motion, jog, feedback, I/O and alarm interface."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Dobot 机械臂控制与 IMU 校准")
        self.resize(1280, 1000)
        self.setMinimumSize(1100, 900)

        self.client_dash = None
        self.client_feed = None
        self.feedback_thread = None
        self.alarm_thread = None
        self.rotation_thread = None
        self.rotation_diagnostics = None
        self.connected = False
        self.enabled = False
        self.latest_pose = None
        self.latest_robot_mode = None
        self.latest_user_index = 0
        self.latest_tool_index = 0
        self.latest_speed_scaling = 1.0
        self.recorded_pose = None
        self.recorded_user_index = 0
        self.recorded_tool_index = 0
        self.recorded_at = None
        self.saved_tool_offset = None
        self.rotation_running = False
        self.rotation_plan_items = []
        self.rotation_sequence_items = []
        self.rotation_sequence_index = 0
        self.rotation_sequence_stopping = False
        self.rotation_sequence_blended = False
        self.active_rotation_axis = None
        self.active_tool_index = 0
        self.alarm_requested = False
        self.command_buttons = []
        self.move_entries = {}
        self.feedback_labels = {}
        self.calibration_running = False
        self.calibration_started_at = 0.0
        self.last_chart_update = 0.0
        self.chart_time_window = 20.0
        self.chart_max_points = 400
        self.angular_series = {}
        self.angular_samples = {}
        self.latest_tool_angular_speed = None
        self.static_imu_samples = []
        self.active_jog_command = None
        self.jog_started_at = 0.0
        self.jog_angular_samples = []
        self.rotation_speed_profiles = {}
        self.controller_alarms = {item["id"]: item for item in alarm_controller_list}
        self.servo_alarms = {item["id"]: item for item in alarm_servo_list}

        self.build_ui()
        self.load_tool_offset_config()
        self.load_rotation_speed_calibration()
        self.load_recorded_pose()
        self.set_controls_enabled(False)

    def build_ui(self):
        central = QWidget()
        central.setObjectName("appBackground")
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(18, 14, 18, 16)
        root.setSpacing(10)

        top_left = QWidget()
        top_left_layout = QVBoxLayout(top_left)
        top_left_layout.setContentsMargins(0, 0, 0, 0)
        top_left_layout.setSpacing(10)
        top_left_layout.addWidget(self.build_connection_group())
        top_left_layout.addWidget(self.build_dashboard_group())
        top_left_layout.addStretch()

        log_alarm_splitter = QSplitter(Qt.Orientation.Horizontal)
        log_alarm_splitter.addWidget(self.build_log_group())
        log_alarm_splitter.addWidget(self.build_alarm_group())
        log_alarm_splitter.setSizes([570, 250])
        log_alarm_splitter.setStretchFactor(0, 1)
        log_alarm_splitter.setStretchFactor(1, 0)
        log_alarm_splitter.setHandleWidth(8)

        top_splitter = QSplitter(Qt.Orientation.Horizontal)
        top_splitter.addWidget(top_left)
        top_splitter.addWidget(log_alarm_splitter)
        top_splitter.setSizes([430, 820])
        top_splitter.setStretchFactor(0, 0)
        top_splitter.setStretchFactor(1, 1)
        top_splitter.setHandleWidth(8)
        top_splitter.setMinimumHeight(250)

        content_splitter = QSplitter(Qt.Orientation.Vertical)
        content_splitter.addWidget(top_splitter)
        content_splitter.addWidget(self.build_status_motion_tabs())
        content_splitter.addWidget(self.build_imu_calibration_group())
        content_splitter.setSizes([255, 190, 485])
        content_splitter.setStretchFactor(0, 0)
        content_splitter.setStretchFactor(1, 0)
        content_splitter.setStretchFactor(2, 1)
        content_splitter.setHandleWidth(9)
        root.addWidget(content_splitter, 1)

        self.setStyleSheet(MODERN_STYLE)

    def build_status_motion_tabs(self):
        tabs = QTabWidget()
        tabs.setObjectName("cardTabs")
        tabs.setDocumentMode(False)
        tabs.tabBar().setDrawBase(False)
        tabs.addTab(self.build_move_group(), "运动控制")
        tabs.addTab(self.build_feedback_group(), "状态反馈")
        return tabs

    def build_connection_group(self):
        group = QGroupBox("机器人连接")
        group.setObjectName("compactCard")
        group.setMinimumHeight(116)
        grid = QGridLayout(group)
        grid.setContentsMargins(9, 13, 9, 8)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(7)
        grid.addWidget(QLabel("IP 地址："), 0, 0)
        self.ip_edit = QLineEdit("192.168.5.1")
        self.ip_edit.setFixedWidth(115)
        grid.addWidget(self.ip_edit, 0, 1)
        grid.addWidget(QLabel("控制端口："), 1, 0)
        self.dashboard_port_edit = QLineEdit("29999")
        self.dashboard_port_edit.setFixedWidth(62)
        grid.addWidget(self.dashboard_port_edit, 1, 1)
        grid.addWidget(QLabel("反馈端口："), 1, 2)
        self.feedback_port_edit = QLineEdit("30004")
        self.feedback_port_edit.setFixedWidth(62)
        grid.addWidget(self.feedback_port_edit, 1, 3)
        self.connection_status_label = QLabel("●  未连接")
        self.connection_status_label.setObjectName("connectionBadge")
        self.connection_status_label.setProperty("connected", False)
        self.connection_status_label.setFixedHeight(30)
        grid.addWidget(self.connection_status_label, 0, 2, 1, 2)
        self.connect_button = QPushButton("连接")
        self.connect_button.setProperty("accent", True)
        self.connect_button.setFixedWidth(82)
        self.connect_button.clicked.connect(self.toggle_connection)
        grid.addWidget(self.connect_button, 0, 4, 2, 1)
        grid.setColumnStretch(3, 1)
        return group

    def build_dashboard_group(self):
        group = QGroupBox("机器人控制")
        group.setObjectName("compactCard")
        group.setMinimumHeight(116)
        grid = QGridLayout(group)
        grid.setContentsMargins(9, 13, 9, 8)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(7)
        self.enable_button = self.command_button("使能", self.toggle_enable)
        grid.addWidget(self.enable_button, 0, 0)
        grid.addWidget(self.command_button("清除错误", self.clear_error), 0, 2)
        grid.addWidget(QLabel("速度比例："), 0, 4)
        self.speed_edit = QLineEdit("50")
        self.speed_edit.setFixedWidth(48)
        self.speed_edit.setToolTip(
            "全局速度比例。点动实际速度 = DobotStudio 点动设置 × 此比例；"
            "不等同于一个固定的 °/s 数值。"
        )
        grid.addWidget(self.speed_edit, 0, 5)
        grid.addWidget(QLabel("%"), 0, 6)
        grid.addWidget(self.command_button("确认", self.confirm_speed), 0, 7)

        grid.addWidget(QLabel("数字输出　索引："), 1, 0, 1, 2)
        self.do_index_edit = QLineEdit("1")
        self.do_index_edit.setFixedWidth(44)
        grid.addWidget(self.do_index_edit, 1, 2)
        grid.addWidget(QLabel("状态："), 1, 3)
        self.do_status_combo = QComboBox()
        self.do_status_combo.addItems(("开启", "关闭"))
        self.do_status_combo.setFixedWidth(64)
        grid.addWidget(self.do_status_combo, 1, 4)
        grid.addWidget(self.command_button("确认", self.confirm_do), 1, 5)
        grid.setColumnStretch(8, 1)
        return group

    def build_move_group(self):
        page = QWidget()
        grid = QGridLayout(page)
        grid.setContentsMargins(10, 10, 10, 8)
        self.add_move_row(grid, 0, COORD_NAMES, ("600", "-260", "380", "170", "12", "140"))
        grid.addWidget(self.command_button("关节运动", self.move_pose_j), 0, 12)
        grid.addWidget(self.command_button("直线运动", self.move_pose_l), 0, 13)
        self.add_move_row(grid, 1, JOINT_NAMES, ("0", "-20", "-80", "30", "90", "120"))
        grid.addWidget(self.command_button("关节运动", self.move_joint_j), 1, 12)
        self.stop_motion_button = self.command_button("停止运动", self.stop_all_motion)
        self.stop_motion_button.setProperty("danger", True)
        grid.addWidget(self.stop_motion_button, 1, 13)
        grid.addWidget(self.build_pose_record_group(), 0, 14, 2, 1)
        grid.setColumnStretch(14, 1)
        return page

    def build_pose_record_group(self):
        group = QGroupBox("位姿记录与恢复")
        group.setMinimumWidth(350)
        layout = QHBoxLayout(group)
        layout.setContentsMargins(8, 7, 8, 6)
        layout.setSpacing(6)
        self.record_pose_button = self.command_button("记录当前位姿", self.record_current_pose)
        layout.addWidget(self.record_pose_button)
        self.restore_pose_button = self.command_button("恢复记录位姿", self.restore_recorded_pose)
        self.restore_pose_button.setProperty("warning", True)
        layout.addWidget(self.restore_pose_button)
        self.recorded_pose_label = QLabel("尚未记录")
        self.recorded_pose_label.setProperty("value", True)
        self.recorded_pose_label.setMinimumWidth(110)
        self.recorded_pose_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addWidget(self.recorded_pose_label, 1)
        return group

    def add_move_row(self, grid, row, names, defaults):
        for index, (name, default) in enumerate(zip(names, defaults)):
            key = f"{name}:"
            grid.addWidget(QLabel(key), row, index * 2)
            edit = QLineEdit(default)
            edit.setFixedWidth(62)
            self.move_entries[key] = edit
            grid.addWidget(edit, row, index * 2 + 1)

    def build_feedback_group(self):
        page = QWidget()
        outer = QVBoxLayout(page)
        outer.setContentsMargins(10, 10, 10, 8)
        status = QGridLayout()
        status.addWidget(QLabel("当前速度比例："), 0, 0)
        self.speed_feedback = QLabel()
        self.speed_feedback.setProperty("value", True)
        status.addWidget(self.speed_feedback, 0, 1)
        status.addWidget(QLabel("%"), 0, 2)
        status.addWidget(QLabel("机器人模式："), 1, 0)
        self.mode_feedback = QLabel()
        self.mode_feedback.setProperty("value", True)
        status.addWidget(self.mode_feedback, 1, 1, 1, 3)
        self.di_feedback = QLabel()
        self.do_feedback = QLabel()
        self.di_feedback.setProperty("ioValue", True)
        self.do_feedback.setProperty("ioValue", True)
        self.di_feedback.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.do_feedback.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        status.addWidget(QLabel("数字输入："), 0, 4)
        status.addWidget(self.di_feedback, 0, 5)
        status.addWidget(QLabel("数字输出："), 1, 4)
        status.addWidget(self.do_feedback, 1, 5)
        status.setColumnStretch(3, 1)
        status.setColumnStretch(5, 1)
        outer.addLayout(status)

        jog_row = QHBoxLayout()
        jog_row.setContentsMargins(0, 0, 0, 0)
        jog_row.setSpacing(8)
        jog_row.addWidget(self.build_horizontal_jog_group("关节点动", JOINT_NAMES), 1)
        jog_row.addWidget(self.build_horizontal_jog_group("Tool 坐标点动", COORD_NAMES), 1)
        outer.addLayout(jog_row)
        return page

    def build_horizontal_jog_group(self, title, names):
        group = QGroupBox(title)
        layout = QHBoxLayout(group)
        layout.setContentsMargins(7, 6, 7, 5)
        layout.setSpacing(5)
        for name in names:
            axis_widget = QWidget()
            axis_layout = QVBoxLayout(axis_widget)
            axis_layout.setContentsMargins(0, 0, 0, 0)
            axis_layout.setSpacing(3)

            value = QLabel(" ")
            value.setProperty("value", True)
            value.setMinimumWidth(48)
            value.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.feedback_labels[f"{name}:"] = value

            title_row = QHBoxLayout()
            title_row.setContentsMargins(0, 0, 0, 0)
            title_row.setSpacing(3)
            name_label = QLabel(f"{name}:")
            name_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            title_row.addWidget(name_label)
            title_row.addWidget(value, 1)
            axis_layout.addLayout(title_row)

            minus = self.jog_button(f"{name}-")
            plus = self.jog_button(f"{name}+")
            button_row = QHBoxLayout()
            button_row.setContentsMargins(0, 0, 0, 0)
            button_row.setSpacing(3)
            button_row.addWidget(minus)
            button_row.addWidget(plus)
            axis_layout.addLayout(button_row)
            layout.addWidget(axis_widget, 1)
        return group

    def build_log_group(self):
        group = QGroupBox("运行日志")
        layout = QVBoxLayout(group)
        self.log_text = QTextEdit()
        self.log_text.setObjectName("logText")
        self.log_text.setReadOnly(True)
        layout.addWidget(self.log_text)
        return group

    def build_alarm_group(self):
        group = QGroupBox("报警信息")
        layout = QVBoxLayout(group)
        self.error_text = QTextEdit()
        self.error_text.setReadOnly(True)
        layout.addWidget(self.error_text)
        layout.addWidget(
            self.command_button("清空", self.error_text.clear),
            alignment=Qt.AlignmentFlag.AlignRight,
        )
        return group

    def build_imu_calibration_group(self):
        group = QGroupBox("IMU 校准")
        layout = QVBoxLayout(group)
        layout.setContentsMargins(8, 8, 8, 8)

        self.imu_tabs = QTabWidget()
        self.imu_tabs.setObjectName("cardTabs")
        self.imu_tabs.setDocumentMode(False)
        self.imu_tabs.tabBar().setDrawBase(False)
        self.imu_tabs.setTabPosition(QTabWidget.TabPosition.North)

        dynamic_page = QWidget()
        dynamic_layout = QVBoxLayout(dynamic_page)
        dynamic_layout.setContentsMargins(8, 8, 8, 8)
        dynamic_layout.setSpacing(10)

        top_controls = QHBoxLayout()
        top_controls.setSpacing(10)

        controls = QWidget()
        controls.setMinimumWidth(620)
        controls_layout = QVBoxLayout(controls)
        controls_layout.setContentsMargins(0, 0, 4, 0)
        controls_layout.setSpacing(6)

        tool_group = QGroupBox("末端工具坐标系偏移")
        tool_grid = QGridLayout(tool_group)
        tool_grid.setContentsMargins(8, 8, 8, 6)
        tool_grid.setHorizontalSpacing(5)
        tool_grid.setVerticalSpacing(5)
        tool_grid.addWidget(QLabel("Tool："), 0, 0)
        self.tool_index_combo = QComboBox()
        self.tool_index_combo.addItems([str(index) for index in range(1, 10)])
        self.tool_index_combo.setFixedWidth(50)
        tool_grid.addWidget(self.tool_index_combo, 0, 1)
        self.tool_offset_edits = {}
        for column, name in enumerate(("X", "Y", "Z")):
            tool_grid.addWidget(QLabel(f"{name}："), 0, 2 + column * 2)
            edit = QLineEdit("0")
            edit.setFixedWidth(55)
            self.tool_offset_edits[name] = edit
            tool_grid.addWidget(edit, 0, 3 + column * 2)
        for column, name in enumerate(("Rx", "Ry", "Rz")):
            tool_grid.addWidget(QLabel(f"{name}："), 1, column * 2)
            edit = QLineEdit("0")
            edit.setFixedWidth(55)
            self.tool_offset_edits[name] = edit
            tool_grid.addWidget(edit, 1, column * 2 + 1)
        self.apply_tool_button = self.command_button("保存并启用", self.apply_tool_offset)
        tool_grid.addWidget(self.apply_tool_button, 1, 6, 1, 2)
        self.tool_status_label = QLabel("当前使用 Tool 0（法兰坐标系）")
        self.tool_status_label.setProperty("value", True)
        tool_grid.addWidget(self.tool_status_label, 2, 0, 1, 6)
        self.restore_tool_button = self.command_button("恢复 Tool 0", self.restore_tool_zero)
        tool_grid.addWidget(self.restore_tool_button, 2, 6, 1, 2)
        controls_layout.addWidget(tool_group)

        rotation_group = QGroupBox("IMU Tool 轴匀速旋转")
        rotation_grid = QGridLayout(rotation_group)
        rotation_grid.setContentsMargins(8, 8, 8, 6)
        rotation_grid.setHorizontalSpacing(6)
        rotation_grid.setVerticalSpacing(7)
        rotation_grid.addWidget(QLabel("旋转轴："), 0, 0)
        self.rotation_axis_combo = QComboBox()
        self.rotation_axis_combo.addItems(("Rx", "Ry", "Rz"))
        self.rotation_axis_combo.setFixedWidth(58)
        rotation_grid.addWidget(self.rotation_axis_combo, 0, 1)
        rotation_grid.addWidget(QLabel("角速度："), 0, 2)
        self.angular_speed_edit = QLineEdit("5.0")
        self.angular_speed_edit.setFixedWidth(62)
        self.angular_speed_edit.setToolTip("单位：°/s；负数表示反向旋转")
        rotation_grid.addWidget(self.angular_speed_edit, 0, 3)
        rotation_grid.addWidget(QLabel("持续时间："), 0, 4)
        self.rotation_duration_edit = QLineEdit("5.0")
        self.rotation_duration_edit.setFixedWidth(55)
        self.rotation_duration_edit.setToolTip("单位：秒；填 0 时持续运行至手动停止")
        rotation_grid.addWidget(self.rotation_duration_edit, 0, 5)
        self.rotation_button = self.command_button("开始旋转", self.toggle_continuous_rotation)
        self.rotation_button.setProperty("accent", True)
        rotation_grid.addWidget(self.rotation_button, 1, 0, 1, 2)
        self.rotation_status_label = QLabel("等待连接")
        self.rotation_status_label.setProperty("value", True)
        rotation_grid.addWidget(self.rotation_status_label, 1, 2, 1, 4)

        combined_widget = QWidget()
        combined_grid = QGridLayout(combined_widget)
        combined_grid.setContentsMargins(0, 0, 0, 0)
        combined_grid.setHorizontalSpacing(6)
        combined_grid.setVerticalSpacing(5)
        combined_grid.addWidget(QLabel("组合角速度："), 0, 0)
        self.combined_speed_edits = {}
        for index, name in enumerate(("Rx", "Ry", "Rz")):
            combined_grid.addWidget(QLabel(f"{name}："), 0, index * 2 + 1)
            edit = QLineEdit("0")
            edit.setFixedWidth(52)
            edit.setToolTip("组合旋转角速度，单位：°/s")
            self.combined_speed_edits[name] = edit
            combined_grid.addWidget(edit, 0, index * 2 + 2)
        combined_grid.addWidget(QLabel("项间 CP："), 1, 0)
        self.rotation_cp_edit = QLineEdit("100")
        self.rotation_cp_edit.setFixedWidth(42)
        self.rotation_cp_edit.setToolTip(
            "运动列表中相邻点的平滑过渡比例：0 为不平滑，100 为最大平滑"
        )
        combined_grid.addWidget(self.rotation_cp_edit, 1, 1)
        combined_grid.addWidget(QLabel("过渡："), 1, 2)
        self.rotation_transition_time_edit = QLineEdit("0.8")
        self.rotation_transition_time_edit.setFixedWidth(38)
        self.rotation_transition_time_edit.setToolTip("相邻运动项自动插值过渡总时长（秒）")
        combined_grid.addWidget(self.rotation_transition_time_edit, 1, 3)
        combined_grid.addWidget(QLabel("秒 / 段数："), 1, 4)
        self.rotation_transition_steps_edit = QLineEdit("6")
        self.rotation_transition_steps_edit.setFixedWidth(30)
        self.rotation_transition_steps_edit.setToolTip("每个相邻运动项之间自动生成的插值段数（2～20）")
        combined_grid.addWidget(self.rotation_transition_steps_edit, 1, 5)
        combined_grid.setColumnStretch(7, 1)
        rotation_grid.addWidget(combined_widget, 2, 0, 1, 6)

        self.add_combined_rotation_item_button = self.command_button(
            "添加组合项", self.add_combined_rotation_plan_item
        )
        self.add_rotation_item_button = self.command_button(
            "添加运动项", self.add_rotation_plan_item
        )
        self.remove_rotation_item_button = self.command_button(
            "删除选中", self.remove_selected_rotation_plan_item
        )
        self.clear_rotation_items_button = self.command_button(
            "清空列表", self.clear_rotation_plan_items
        )
        plan_button_widget = QWidget()
        plan_button_row = QHBoxLayout(plan_button_widget)
        plan_button_row.setContentsMargins(0, 0, 0, 0)
        plan_button_row.setSpacing(6)
        for button in (
            self.add_rotation_item_button,
            self.add_combined_rotation_item_button,
            self.remove_rotation_item_button,
            self.clear_rotation_items_button,
        ):
            plan_button_row.addWidget(button, 1)
        rotation_grid.addWidget(plan_button_widget, 3, 0, 1, 6)

        self.rotation_plan_list = QListWidget()
        self.rotation_plan_list.setFixedHeight(170)
        self.rotation_plan_list.setStyleSheet("QListWidget::item { min-height: 24px; }")
        self.rotation_plan_list.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOn)
        self.rotation_plan_list.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.rotation_plan_list.setToolTip(
            "按添加顺序依次运行；每一项完成后，下一项会从机械臂当前反馈姿态开始"
        )
        rotation_grid.addWidget(self.rotation_plan_list, 4, 0, 1, 6)
        for column in range(6):
            rotation_grid.setColumnStretch(column, 1)

        capture_row = QHBoxLayout()
        capture_row.addWidget(QLabel("角速度曲线："))
        self.imu_start_button = self.command_button("开始采集", self.toggle_imu_calibration)
        self.imu_start_button.setProperty("accent", True)
        self.imu_start_button.setMinimumHeight(32)
        capture_row.addWidget(self.imu_start_button)
        self.imu_status_label = QLabel("等待连接")
        self.imu_status_label.setProperty("value", True)
        capture_row.addWidget(self.imu_status_label)
        capture_row.addStretch()
        controls_layout.addLayout(capture_row)
        chart = QChart()
        chart.setTitle("机械臂末端在 Tool 坐标系下的旋转速度")
        chart.setTitleFont(QFont("Microsoft YaHei UI", 11, QFont.Weight.DemiBold))
        chart.setTitleBrush(QBrush(QColor("#344054")))
        chart.setBackgroundVisible(False)
        chart.setPlotAreaBackgroundVisible(True)
        chart.setPlotAreaBackgroundBrush(QBrush(QColor("#fbfdff")))
        chart.legend().setVisible(True)
        chart.legend().setLabelColor(QColor("#475467"))
        series_config = (
            ("x", "绕 Tool X 轴", "#2563eb"),
            ("y", "绕 Tool Y 轴", "#f59e0b"),
            ("z", "绕 Tool Z 轴", "#10b981"),
        )
        for key, title, color in series_config:
            series = QLineSeries()
            series.setName(title)
            series.setPen(QPen(QColor(color), 2.2))
            chart.addSeries(series)
            self.angular_series[key] = series
            self.angular_samples[key] = []

        self.chart_axis_x = QValueAxis()
        self.chart_axis_x.setTitleText("时间（秒）")
        self.chart_axis_x.setLabelFormat("%.1f")
        self.chart_axis_x.setRange(0.0, self.chart_time_window)
        self.chart_axis_y = QValueAxis()
        self.chart_axis_y.setTitleText("旋转速度（°/s）")
        self.chart_axis_y.setLabelFormat("%.2f")
        self.chart_axis_y.setRange(-1.0, 1.0)
        for axis in (self.chart_axis_x, self.chart_axis_y):
            axis.setLabelsColor(QColor("#667085"))
            axis.setTitleBrush(QBrush(QColor("#475467")))
            axis.setGridLineColor(QColor("#e8eef5"))
            axis.setLinePenColor(QColor("#cbd5e1"))
        chart.addAxis(self.chart_axis_x, Qt.AlignmentFlag.AlignBottom)
        chart.addAxis(self.chart_axis_y, Qt.AlignmentFlag.AlignLeft)
        for series in self.angular_series.values():
            series.attachAxis(self.chart_axis_x)
            series.attachAxis(self.chart_axis_y)

        chart_view = QChartView(chart)
        chart_view.setStyleSheet("background: transparent; border: none;")
        chart_view.setRenderHint(QPainter.RenderHint.Antialiasing)
        chart_view.setMinimumHeight(260)
        controls_layout.addWidget(chart_view, 1)

        rotation_group.setMinimumWidth(430)
        top_controls.addWidget(controls, 2)
        top_controls.addWidget(rotation_group, 1)
        dynamic_layout.addLayout(top_controls, 1)

        static_page = self.build_static_imu_calibration_page()
        self.imu_tabs.addTab(dynamic_page, "动态校准")
        self.imu_tabs.addTab(static_page, "静态校准")
        layout.addWidget(self.imu_tabs)
        return group

    def build_static_imu_calibration_page(self):
        page = QWidget()
        layout = QHBoxLayout(page)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(10)

        controls = QGroupBox("静态校准采集")
        controls.setFixedWidth(420)
        control_layout = QGridLayout(controls)
        control_layout.setContentsMargins(8, 8, 8, 6)
        control_layout.setHorizontalSpacing(6)
        control_layout.setVerticalSpacing(6)

        control_layout.addWidget(QLabel("采样时长："), 0, 0)
        self.static_sample_duration_edit = QLineEdit("5.0")
        self.static_sample_duration_edit.setFixedWidth(64)
        self.static_sample_duration_edit.setToolTip("单位：秒；静止放置后采集一段稳定数据")
        control_layout.addWidget(self.static_sample_duration_edit, 0, 1)
        control_layout.addWidget(QLabel("样本编号："), 0, 2)
        self.static_sample_index_label = QLabel("0")
        self.static_sample_index_label.setProperty("value", True)
        control_layout.addWidget(self.static_sample_index_label, 0, 3)

        self.static_sample_button = self.command_button(
            "记录静止样本", self.record_static_imu_sample
        )
        self.static_sample_button.setProperty("accent", True)
        control_layout.addWidget(self.static_sample_button, 1, 0, 1, 2)

        self.static_clear_button = self.command_button(
            "清空样本", self.clear_static_imu_samples
        )
        control_layout.addWidget(self.static_clear_button, 1, 2)

        self.static_save_button = self.command_button(
            "保存静态校准", self.save_static_imu_calibration
        )
        control_layout.addWidget(self.static_save_button, 1, 3)

        self.static_status_label = QLabel("等待连接")
        self.static_status_label.setProperty("value", True)
        control_layout.addWidget(self.static_status_label, 2, 0, 1, 4)

        layout.addWidget(controls)

        info_group = QGroupBox("静态样本状态")
        info_layout = QVBoxLayout(info_group)
        info_layout.setContentsMargins(8, 8, 8, 6)
        self.static_sample_text = QTextEdit()
        self.static_sample_text.setReadOnly(True)
        self.static_sample_text.setMinimumHeight(240)
        self.static_sample_text.setPlaceholderText(
            "静态校准用于记录 IMU 在不同静止姿态下的零偏/重力方向样本。"
        )
        info_layout.addWidget(self.static_sample_text)
        layout.addWidget(info_group, 1)
        return page

    def command_button(self, text, callback):
        button = QPushButton(text)
        button.clicked.connect(callback)
        self.command_buttons.append(button)
        return button

    def jog_button(self, command):
        button = QPushButton(command)
        button.setProperty("jog", True)
        button.setFixedSize(40, 22)
        button.pressed.connect(lambda value=command: self.start_jog(value))
        button.released.connect(self.stop_jog)
        self.command_buttons.append(button)
        return button

    def set_controls_enabled(self, enabled):
        for button in self.command_buttons:
            button.setEnabled(enabled)
        for name in (
            "add_combined_rotation_item_button",
            "add_rotation_item_button",
            "remove_rotation_item_button",
            "clear_rotation_items_button",
        ):
            if hasattr(self, name):
                getattr(self, name).setEnabled(True)
        if hasattr(self, "restore_pose_button"):
            self.restore_pose_button.setEnabled(enabled and self.recorded_pose is not None)

    def set_connection_status(self, connected):
        self.connection_status_label.setText("●  已连接" if connected else "●  未连接")
        self.connection_status_label.setProperty("connected", connected)
        # Re-polish so the dynamic-property selector is applied immediately.
        self.connection_status_label.style().unpolish(self.connection_status_label)
        self.connection_status_label.style().polish(self.connection_status_label)

    def append_log(self, message):
        self.log_text.append(f"[{time.strftime('%H:%M:%S')}] {message}")

    @staticmethod
    def normalize_speed_scaling(value):
        value = float(value)
        return max(0.01, min(1.0, value / 100.0 if value > 1.5 else value))

    @staticmethod
    def rotation_profile_key(tool_index, axis):
        return f"tool_{int(tool_index)}_{axis}"

    def load_rotation_speed_calibration(self):
        if not ROTATION_SPEED_CALIBRATION_FILE.exists():
            return
        try:
            data = json.loads(ROTATION_SPEED_CALIBRATION_FILE.read_text(encoding="utf-8"))
            profiles = data.get("profiles", {})
            if not isinstance(profiles, dict):
                raise ValueError("profiles 不是对象")
            self.rotation_speed_profiles = profiles
            self.append_log(f"已载入 Tool 轴速度标定：{len(profiles)} 项")
        except Exception as exc:
            self.append_log(f"Tool 轴速度标定文件读取失败，已忽略：{exc}")

    def save_rotation_speed_calibration(self):
        data = {"version": 1, "profiles": self.rotation_speed_profiles}
        temporary = ROTATION_SPEED_CALIBRATION_FILE.with_suffix(".json.tmp")
        temporary.write_text(
            json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        temporary.replace(ROTATION_SPEED_CALIBRATION_FILE)

    def update_static_sample_view(self):
        if not hasattr(self, "static_sample_text"):
            return
        self.static_sample_index_label.setText(str(len(self.static_imu_samples)))
        if not self.static_imu_samples:
            self.static_sample_text.clear()
            self.static_status_label.setText("尚未记录静态样本")
            return
        lines = []
        for index, sample in enumerate(self.static_imu_samples, 1):
            pose = sample["pose"]
            angular = sample["tool_angular_speed_deg_s"]
            lines.append(
                f"样本 {index}  {sample['recorded_at']}\n"
                f"  Tool {sample['tool']} / User {sample['user']}，采样时长 {sample['duration_s']:.2f} s\n"
                f"  位姿：X={pose[0]:.2f}, Y={pose[1]:.2f}, Z={pose[2]:.2f}, "
                f"Rx={pose[3]:.2f}, Ry={pose[4]:.2f}, Rz={pose[5]:.2f}\n"
                f"  Tool 角速度：Wx={angular[0]:+.3f}, Wy={angular[1]:+.3f}, "
                f"Wz={angular[2]:+.3f} °/s"
            )
        self.static_sample_text.setPlainText("\n\n".join(lines))
        self.static_status_label.setText(f"已记录 {len(self.static_imu_samples)} 个静态样本")

    def record_static_imu_sample(self):
        if not self.connected:
            QMessageBox.warning(self, "尚未连接", "请先连接机械臂")
            return
        if self.latest_pose is None:
            QMessageBox.warning(self, "暂无反馈", "还没有读取到机械臂当前位姿")
            return
        try:
            duration = float(self.static_sample_duration_edit.text())
            if duration <= 0:
                raise ValueError("采样时长必须大于 0")
        except ValueError as exc:
            QMessageBox.warning(self, "输入错误", str(exc))
            return

        angular = self.latest_tool_angular_speed or [0.0, 0.0, 0.0]
        sample = {
            "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "duration_s": duration,
            "user": int(self.latest_user_index),
            "tool": int(self.latest_tool_index),
            "pose": [float(value) for value in self.latest_pose],
            "tool_angular_speed_deg_s": [float(value) for value in angular],
        }
        self.static_imu_samples.append(sample)
        self.update_static_sample_view()
        self.append_log(f"已记录静态 IMU 样本 {len(self.static_imu_samples)}")

    def clear_static_imu_samples(self):
        self.static_imu_samples.clear()
        self.update_static_sample_view()
        self.append_log("已清空静态 IMU 样本")

    def save_static_imu_calibration(self):
        if not self.static_imu_samples:
            QMessageBox.warning(self, "没有样本", "请先记录至少一个静态样本")
            return
        data = {
            "version": 1,
            "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "samples": self.static_imu_samples,
        }
        temporary = STATIC_IMU_CALIBRATION_FILE.with_suffix(".json.tmp")
        try:
            temporary.write_text(
                json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            temporary.replace(STATIC_IMU_CALIBRATION_FILE)
        except Exception as exc:
            QMessageBox.critical(self, "保存失败", f"静态校准样本保存失败：\n{exc}")
            return
        self.static_status_label.setText(f"已保存到 {STATIC_IMU_CALIBRATION_FILE.name}")
        self.append_log(f"静态 IMU 样本已保存：{STATIC_IMU_CALIBRATION_FILE}")

    def controller_v_for_rotation(self, axis, angular_speed, tool_index):
        global_scale = self.normalize_speed_scaling(self.latest_speed_scaling)
        key = self.rotation_profile_key(tool_index, axis)
        profile = self.rotation_speed_profiles.get(key)
        calibrated = False
        if isinstance(profile, dict):
            rate = float(profile.get("deg_s_per_v_at_full_global", 0.0))
            if rate > 0:
                controller_v = round(abs(angular_speed) / (rate * global_scale))
                calibrated = True
            else:
                controller_v = 0
        else:
            controller_v = 0
        if not calibrated:
            # Safe first-run estimate: assume v=100 gives roughly 100 deg/s at
            # full global scaling. The measured plateau updates this automatically.
            controller_v = min(20, round(abs(angular_speed) / global_scale))
        return max(1, min(100, controller_v)), global_scale, calibrated

    def prepare_jog_rotation_speed(self, axis, angular_speed, tool_index):
        jog_key = f"jog_{self.rotation_profile_key(tool_index, axis)}"
        jog_profile = self.rotation_speed_profiles.get(jog_key, {})
        full_speed = float(jog_profile.get("full_global_speed_deg_s", 0.0) or 0.0)
        calibrated = full_speed > 0
        if full_speed <= 0:
            # Use the controller-planned Tool rotation calibration as the first
            # estimate; the first jog run then creates an independent jog profile.
            rel_profile = self.rotation_speed_profiles.get(
                self.rotation_profile_key(tool_index, axis), {}
            )
            rate = float(rel_profile.get("deg_s_per_v_at_full_global", 0.0) or 0.0)
            full_speed = rate * 100.0
        if full_speed <= 0:
            return self.normalize_speed_scaling(self.latest_speed_scaling), False, None

        requested_percent = round(abs(angular_speed) / full_speed * 100.0)
        applied_percent = max(1, min(100, requested_percent))
        result = self.run_command(
            "设置 IMU 连续点动速度比例",
            lambda: self.client_dash.SpeedFactor(applied_percent),
        )
        if self.result_error_id(result) != 0:
            raise RuntimeError(f"设置连续点动速度比例失败：{result}")
        self.speed_edit.setText(str(applied_percent))
        self.latest_speed_scaling = applied_percent / 100.0
        return applied_percent / 100.0, calibrated, full_speed

    def update_rotation_speed_calibration(self, summary):
        method = summary.get("control_method")
        if method not in ("RelMovLTool", "MoveJogTool"):
            return
        plateau = summary.get("plateau_feedback", {})
        measured = plateau.get("mean")
        count = int(plateau.get("count") or 0)
        controller_v = int(summary.get("controller_v_percent") or 0)
        global_scale = float(summary.get("global_speed_scale") or 0.0)
        if measured is None or count < 5 or global_scale <= 0:
            return
        if method == "MoveJogTool":
            full_speed = abs(float(measured)) / global_scale
            if not math.isfinite(full_speed) or full_speed <= 0:
                return
            key = f"jog_{self.rotation_profile_key(summary['tool'], summary['axis'])}"
            old = self.rotation_speed_profiles.get(key, {})
            old_speed = float(old.get("full_global_speed_deg_s", 0.0) or 0.0)
            runs = int(old.get("runs", 0) or 0)
            combined = full_speed if old_speed <= 0 else old_speed * 0.4 + full_speed * 0.6
            self.rotation_speed_profiles[key] = {
                "tool": int(summary["tool"]),
                "axis": summary["axis"],
                "full_global_speed_deg_s": combined,
                "runs": runs + 1,
                "last_measured_deg_s": abs(float(measured)),
                "last_global_speed_scale": global_scale,
                "updated_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
            }
            try:
                self.save_rotation_speed_calibration()
            except Exception as exc:
                self.append_log(f"Tool 点动速度标定保存失败：{exc}")
                return
            self.append_log(
                f"已更新 Tool {summary['tool']} {summary['axis']} 连续点动标定："
                f"实测稳定速度={abs(float(measured)):.3f} °/s"
            )
            return
        if controller_v <= 0:
            return
        measured_rate = abs(float(measured)) / (controller_v * global_scale)
        if not math.isfinite(measured_rate) or measured_rate <= 0:
            return
        key = self.rotation_profile_key(summary["tool"], summary["axis"])
        old = self.rotation_speed_profiles.get(key, {})
        old_rate = float(old.get("deg_s_per_v_at_full_global", 0.0) or 0.0)
        sample_count = int(old.get("runs", 0) or 0)
        # Give the latest physical run meaningful weight while retaining history.
        combined_rate = measured_rate if old_rate <= 0 else old_rate * 0.4 + measured_rate * 0.6
        self.rotation_speed_profiles[key] = {
            "tool": int(summary["tool"]),
            "axis": summary["axis"],
            "deg_s_per_v_at_full_global": combined_rate,
            "runs": sample_count + 1,
            "last_measured_deg_s": abs(float(measured)),
            "last_controller_v": controller_v,
            "last_global_speed_scale": global_scale,
            "updated_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        }
        try:
            self.save_rotation_speed_calibration()
        except Exception as exc:
            self.append_log(f"Tool 轴速度标定保存失败：{exc}")
            return
        self.append_log(
            f"已更新 Tool {summary['tool']} {summary['axis']} 速度标定："
            f"实测稳定速度={abs(float(measured)):.3f} °/s，下一次将自动修正 v 比例"
        )

    def toggle_connection(self):
        self.disconnect_robot() if self.connected else self.connect_robot()

    def connect_robot(self):
        try:
            ip = self.ip_edit.text().strip()
            dashboard_port = int(self.dashboard_port_edit.text())
            feedback_port = int(self.feedback_port_edit.text())
            if not ip:
                raise ValueError("IP 地址不能为空")
            self.client_dash = DobotApiDashboard(ip, dashboard_port, self.log_text)
            self.client_feed = DobotApiFeedBack(ip, feedback_port, self.log_text)
            self.feedback_thread = FeedbackThread(self.client_feed, self)
            self.feedback_thread.feedback.connect(self.update_feedback)
            self.feedback_thread.connection_lost.connect(self.on_connection_lost)
            self.feedback_thread.start()
        except Exception as exc:
            self.close_clients()
            QMessageBox.critical(self, "连接失败", f"无法连接机器人：{exc}")
            return
        self.connected = True
        self.set_connection_status(True)
        self.connect_button.setText("断开")
        self.set_controls_enabled(True)
        self.imu_status_label.setText("等待开始")
        self.rotation_status_label.setText("等待开始")
        self.static_status_label.setText("等待记录静态样本")
        self.tool_status_label.setText("当前使用 Tool 0（法兰坐标系）")
        self.append_log(f"已连接机器人：{ip}")
        self.apply_saved_tool_offset_to_robot()

    def disconnect_robot(self):
        self.shutdown_rotation()
        if self.alarm_thread is not None and self.alarm_thread.isRunning():
            self.alarm_thread.requestInterruption()
        if self.feedback_thread is not None:
            self.feedback_thread.stop()
            self.feedback_thread.wait(1500)
            self.feedback_thread = None
        self.close_clients()
        self.connected = False
        self.set_connection_status(False)
        self.enabled = False
        self.alarm_requested = False
        self.calibration_running = False
        self.latest_pose = None
        self.latest_robot_mode = None
        self.latest_user_index = 0
        self.latest_tool_index = 0
        self.latest_speed_scaling = 1.0
        self.latest_tool_angular_speed = None
        self.active_jog_command = None
        self.jog_angular_samples.clear()
        self.connect_button.setText("连接")
        self.enable_button.setText("使能")
        self.imu_start_button.setText("开始采集")
        self.imu_status_label.setText("等待连接")
        self.rotation_button.setText("开始旋转")
        self.rotation_status_label.setText("等待连接")
        self.static_status_label.setText("等待连接")
        self.tool_status_label.setText("等待连接")
        self.rotation_axis_combo.setEnabled(True)
        self.angular_speed_edit.setEnabled(True)
        self.rotation_duration_edit.setEnabled(True)
        self.active_rotation_axis = None
        self.set_controls_enabled(False)
        self.append_log("已断开机器人连接")

    def close_clients(self):
        for client in (self.client_feed, self.client_dash):
            if client is not None:
                try:
                    client.close()
                except Exception:
                    pass
        self.client_feed = None
        self.client_dash = None

    def on_connection_lost(self, message):
        if not self.connected:
            return
        self.append_log(f"反馈连接已停止：{message}")
        QMessageBox.warning(self, "连接异常", f"机器人反馈连接已停止：\n{message}")
        self.disconnect_robot()

    def run_command(self, description, command):
        if not self.connected or self.client_dash is None:
            return None
        try:
            result = command()
            self.append_log(f"{description}: {result}")
            return result
        except Exception as exc:
            self.append_log(f"{description}失败：{exc}")
            QMessageBox.critical(self, "指令错误", f"{description}失败：\n{exc}")
            return None

    def toggle_enable(self):
        if self.enabled:
            result = self.run_command("机器人下使能", self.client_dash.DisableRobot)
            if result is not None:
                self.enabled = False
                self.enable_button.setText("使能")
        else:
            result = self.run_command("机器人使能", self.client_dash.EnableRobot)
            if result is not None:
                self.enabled = True
                self.enable_button.setText("下使能")

    def clear_error(self):
        self.run_command("清除错误", self.client_dash.ClearError)

    def confirm_speed(self):
        try:
            speed = int(self.speed_edit.text())
            if not 1 <= speed <= 100:
                raise ValueError("速度比例必须在 1～100 之间")
        except ValueError as exc:
            QMessageBox.warning(self, "输入错误", str(exc))
            return
        self.run_command("设置速度比例", lambda: self.client_dash.SpeedFactor(speed))

    def confirm_do(self):
        try:
            index = int(self.do_index_edit.text())
        except ValueError:
            QMessageBox.warning(self, "输入错误", "数字输出索引必须是整数")
            return
        value = 1 if self.do_status_combo.currentText() == "开启" else 0
        self.run_command("设置数字输出", lambda: self.client_dash.DO(index, value))

    def move_values(self, names):
        return [float(self.move_entries[f"{name}:"].text()) for name in names]

    def move(self, description, names, coordinate_mode, method):
        try:
            values = self.move_values(names)
        except ValueError:
            QMessageBox.warning(self, "输入错误", "所有运动参数都必须是数字")
            return
        self.run_command(description, lambda: method(*values, coordinate_mode))

    def move_pose_j(self):
        self.move("笛卡尔位姿关节运动", COORD_NAMES, 0, self.client_dash.MovJ)

    def move_pose_l(self):
        self.move("笛卡尔位姿直线运动", COORD_NAMES, 0, self.client_dash.MovL)

    def move_joint_j(self):
        self.move("关节角度运动", JOINT_NAMES, 1, self.client_dash.MovJ)

    def start_jog(self, command):
        if command.startswith(("Rx", "Ry", "Rz")):
            self.active_jog_command = command
            self.jog_started_at = time.perf_counter()
            self.jog_angular_samples = []
        else:
            self.active_jog_command = None
            self.jog_angular_samples = []
        if command.startswith("J"):
            result = self.run_command("关节点动", lambda: self.client_dash.MoveJog(command))
        else:
            tool_index = int(self.latest_tool_index)
            user_index = int(self.latest_user_index)
            result = self.run_command(
                f"Tool 坐标点动 Tool {tool_index}",
                lambda: self.client_dash.MoveJog(
                    command, coordtype=2, user=user_index, tool=tool_index
                ),
            )
        if self.result_error_id(result) not in (0, None):
            self.active_jog_command = None
            self.jog_angular_samples = []

    def stop_jog(self):
        self.run_command("停止点动", lambda: self.client_dash.MoveJog(""))
        command = self.active_jog_command
        samples = list(self.jog_angular_samples)
        self.active_jog_command = None
        self.jog_angular_samples = []
        # Ignore the acceleration transient and report the stable portion.
        steady = [value for elapsed, value in samples if elapsed >= 0.30]
        if command and steady:
            deviation = statistics.pstdev(steady) if len(steady) > 1 else 0.0
            self.append_log(
                f"点动 {command} 实测角速度："
                f"平均={statistics.fmean(steady):+.3f} °/s，"
                f"标准差={deviation:.3f} °/s，"
                f"范围={min(steady):+.3f}～{max(steady):+.3f} °/s，"
                f"样本数={len(steady)}"
            )
        elif command:
            self.append_log(f"点动 {command} 时间过短，未取得稳定段角速度；请按住至少 0.5 秒")

    def stop_all_motion(self):
        if self.rotation_thread is not None and self.rotation_thread.isRunning():
            self.rotation_thread.stop()
            self.rotation_button.setEnabled(False)
            self.rotation_status_label.setText("正在停止...")
        result = self.run_command("停止当前运动", self.client_dash.Stop)
        if self.result_error_id(result) not in (0, None):
            QMessageBox.warning(self, "停止失败", f"停止运动指令执行失败：\n{result}")

    @staticmethod
    def result_error_id(result):
        match = re.match(r"\s*(-?\d+)", str(result)) if result is not None else None
        return int(match.group(1)) if match else None

    @staticmethod
    def validate_tool_offset_config(data):
        tool_index = int(data["tool"])
        active_tool = int(data.get("active_tool", tool_index))
        values = [float(value) for value in data["offset"]]
        if not 1 <= tool_index <= 9:
            raise ValueError("Tool 编号必须在 1～9 之间")
        if active_tool != 0 and active_tool != tool_index:
            raise ValueError("active_tool 必须为 0 或保存的 Tool 编号")
        if len(values) != 6 or not all(math.isfinite(value) for value in values):
            raise ValueError("工具坐标系偏移必须包含 6 个有效数值")
        if any(abs(value) > 1000 for value in values[:3]):
            raise ValueError("XYZ 偏移范围必须在 -1000～1000 mm 之间")
        if any(abs(value) > 180 for value in values[3:]):
            raise ValueError("Rx/Ry/Rz 偏移范围必须在 -180～180° 之间")
        return {
            "tool": tool_index,
            "active_tool": active_tool,
            "offset": values,
            "saved_at": str(data.get("saved_at", "")),
        }

    def load_tool_offset_config(self):
        if not TOOL_OFFSET_FILE.exists():
            return
        try:
            data = json.loads(TOOL_OFFSET_FILE.read_text(encoding="utf-8"))
            config = self.validate_tool_offset_config(data)
        except Exception as exc:
            self.append_log(f"工具坐标系配置读取失败，已忽略：{exc}")
            return
        self.saved_tool_offset = config
        self.tool_index_combo.setCurrentText(str(config["tool"]))
        for name, value in zip(COORD_NAMES, config["offset"]):
            self.tool_offset_edits[name].setText(f"{value:g}")
        if config["active_tool"] == 0:
            self.active_tool_index = 0
            self.tool_status_label.setText(
                f"已载入 Tool {config['tool']} 偏移；当前默认 Tool 0"
            )
        else:
            self.active_tool_index = config["tool"]
            self.tool_status_label.setText(
                f"已载入 Tool {config['tool']} 偏移，连接后自动启用"
            )
        self.append_log(
            f"已载入工具坐标系配置：Tool {config['tool']}，"
            f"active={config['active_tool']}"
        )

    def save_tool_offset_config(self, tool_index, values, active_tool):
        data = {
            "version": 1,
            "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "tool": int(tool_index),
            "active_tool": int(active_tool),
            "offset": [float(value) for value in values],
            "units": {
                "X": "mm", "Y": "mm", "Z": "mm",
                "Rx": "deg", "Ry": "deg", "Rz": "deg",
            },
        }
        temporary = TOOL_OFFSET_FILE.with_suffix(".json.tmp")
        temporary.write_text(
            json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        temporary.replace(TOOL_OFFSET_FILE)
        self.saved_tool_offset = self.validate_tool_offset_config(data)

    def apply_saved_tool_offset_to_robot(self):
        config = self.saved_tool_offset
        if not config or config["active_tool"] == 0:
            return
        tool_index = int(config["tool"])
        values = config["offset"]
        table = "{" + ",".join(f"{value:.6f}" for value in values) + "}"
        result = self.run_command(
            f"恢复上次 Tool {tool_index} 坐标系",
            lambda: self.client_dash.SetTool(tool_index, table),
        )
        if self.result_error_id(result) != 0:
            self.tool_status_label.setText(f"Tool {tool_index} 自动恢复失败")
            self.append_log(f"Tool {tool_index} 自动恢复失败：{result}")
            return
        result = self.run_command(
            f"启用上次 Tool {tool_index}", lambda: self.client_dash.Tool(tool_index)
        )
        if self.result_error_id(result) != 0:
            self.tool_status_label.setText(f"Tool {tool_index} 自动启用失败")
            self.append_log(f"Tool {tool_index} 自动启用失败：{result}")
            return
        self.active_tool_index = tool_index
        self.latest_tool_index = tool_index
        self.tool_status_label.setText(f"Tool {tool_index} 已从上次配置恢复并启用")

    def apply_tool_offset(self):
        try:
            tool_index = int(self.tool_index_combo.currentText())
            values = [float(self.tool_offset_edits[name].text()) for name in COORD_NAMES]
            if any(abs(value) > 1000 for value in values[:3]):
                raise ValueError("XYZ 偏移范围必须在 -1000～1000 mm 之间")
            if any(abs(value) > 180 for value in values[3:]):
                raise ValueError("Rx/Ry/Rz 偏移范围必须在 -180～180° 之间")
        except ValueError as exc:
            QMessageBox.warning(self, "输入错误", str(exc))
            return

        table = "{" + ",".join(f"{value:.6f}" for value in values) + "}"
        result = self.run_command(
            f"保存 Tool {tool_index} 坐标系",
            lambda: self.client_dash.SetTool(tool_index, table),
        )
        if self.result_error_id(result) != 0:
            QMessageBox.warning(self, "设置失败", f"工具坐标系保存失败：\n{result}")
            return
        result = self.run_command(
            f"启用 Tool {tool_index}", lambda: self.client_dash.Tool(tool_index)
        )
        if self.result_error_id(result) != 0:
            QMessageBox.warning(self, "设置失败", f"工具坐标系启用失败：\n{result}")
            return
        self.active_tool_index = tool_index
        self.latest_tool_index = tool_index
        try:
            self.save_tool_offset_config(tool_index, values, tool_index)
        except Exception as exc:
            QMessageBox.warning(self, "本地保存失败", f"Tool 已启用，但本地配置保存失败：\n{exc}")
            self.append_log(f"工具坐标系本地配置保存失败：{exc}")
            return
        self.tool_status_label.setText(f"Tool {tool_index} 已保存并启用")

    def restore_tool_zero(self):
        result = self.run_command("恢复法兰坐标系 Tool 0", lambda: self.client_dash.Tool(0))
        if self.result_error_id(result) != 0:
            QMessageBox.warning(self, "设置失败", f"Tool 0 启用失败：\n{result}")
            return
        self.active_tool_index = 0
        self.latest_tool_index = 0
        if self.saved_tool_offset:
            try:
                self.save_tool_offset_config(
                    self.saved_tool_offset["tool"],
                    self.saved_tool_offset["offset"],
                    0,
                )
            except Exception as exc:
                self.append_log(f"Tool 0 状态本地保存失败：{exc}")
        self.tool_status_label.setText("当前使用 Tool 0（法兰坐标系）")

    def load_recorded_pose(self):
        if not POSE_RECORD_FILE.exists():
            return
        try:
            data = json.loads(POSE_RECORD_FILE.read_text(encoding="utf-8"))
            pose = [float(value) for value in data["pose"]]
            user_index = int(data["user"])
            tool_index = int(data["tool"])
            recorded_at = str(data["recorded_at"])
            if len(pose) != 6 or not all(math.isfinite(value) for value in pose):
                raise ValueError("位姿必须包含 6 个有效数值")
            if not 0 <= user_index <= 9 or not 0 <= tool_index <= 9:
                raise ValueError("User/Tool 编号超出 0～9 范围")
        except Exception as exc:
            self.append_log(f"历史位姿文件读取失败，已忽略：{exc}")
            return

        self.recorded_pose = pose
        self.recorded_user_index = user_index
        self.recorded_tool_index = tool_index
        self.recorded_at = recorded_at
        display_time = recorded_at.replace("T", " ")
        self.recorded_pose_label.setText(
            f"已载入 {display_time}｜User {user_index} / Tool {tool_index}"
        )
        pose_text = ", ".join(
            f"{name}={value:.4f}" for name, value in zip(COORD_NAMES, pose)
        )
        self.recorded_pose_label.setToolTip(pose_text)
        self.append_log(f"已载入历史记录位姿：{pose_text}")

    def save_recorded_pose(self):
        data = {
            "version": 1,
            "recorded_at": self.recorded_at,
            "pose": self.recorded_pose,
            "user": self.recorded_user_index,
            "tool": self.recorded_tool_index,
        }
        temporary_file = POSE_RECORD_FILE.with_suffix(".json.tmp")
        try:
            temporary_file.write_text(
                json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            temporary_file.replace(POSE_RECORD_FILE)
        except Exception as exc:
            self.append_log(f"记录位姿已保留在内存，但写入文件失败：{exc}")
            QMessageBox.warning(
                self,
                "保存失败",
                f"当前位姿已记录，但无法保存到下次启动：\n{exc}",
            )
            return False
        return True

    def record_current_pose(self):
        if self.latest_pose is None:
            QMessageBox.warning(self, "暂无反馈", "尚未收到机械臂实时位姿，请稍后再试")
            return
        if self.latest_robot_mode != 5:
            mode_text = ROBOT_MODES.get(self.latest_robot_mode, str(self.latest_robot_mode))
            QMessageBox.warning(
                self, "机器人未静止", f"请在机器人已使能且空闲时记录位姿。\n当前状态：{mode_text}"
            )
            return

        self.recorded_pose = self.latest_pose.copy()
        self.recorded_user_index = self.latest_user_index
        self.recorded_tool_index = self.latest_tool_index
        self.recorded_at = time.strftime("%Y-%m-%dT%H:%M:%S")
        recorded_at = time.strftime("%H:%M:%S")
        self.recorded_pose_label.setText(
            f"已记录 {recorded_at}｜User {self.recorded_user_index} / Tool {self.recorded_tool_index}"
        )
        pose_text = ", ".join(
            f"{name}={value:.4f}" for name, value in zip(COORD_NAMES, self.recorded_pose)
        )
        self.recorded_pose_label.setToolTip(pose_text)
        self.restore_pose_button.setEnabled(True)
        saved_to_file = self.save_recorded_pose()
        self.append_log(
            f"已记录当前位姿：{pose_text}，User={self.recorded_user_index}，"
            f"Tool={self.recorded_tool_index}，持久化={'成功' if saved_to_file else '失败'}"
        )

    def restore_recorded_pose(self):
        if self.recorded_pose is None:
            QMessageBox.warning(self, "没有记录", "请先记录当前位姿")
            return
        if self.latest_robot_mode != 5:
            mode_text = ROBOT_MODES.get(self.latest_robot_mode, str(self.latest_robot_mode))
            QMessageBox.warning(
                self, "机器人未就绪", f"恢复位姿要求机器人已使能且空闲。\n当前状态：{mode_text}"
            )
            return

        pose_text = "\n".join(
            (
                f"X/Y/Z：{self.recorded_pose[0]:.3f}, {self.recorded_pose[1]:.3f}, {self.recorded_pose[2]:.3f} mm",
                f"Rx/Ry/Rz：{self.recorded_pose[3]:.3f}, {self.recorded_pose[4]:.3f}, {self.recorded_pose[5]:.3f}°",
                f"User {self.recorded_user_index} / Tool {self.recorded_tool_index}",
            )
        )
        answer = QMessageBox.question(
            self,
            "确认恢复位姿",
            f"机械臂将以 50% 速度比例运动到记录位姿：\n\n{pose_text}\n\n确认执行吗？",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if answer != QMessageBox.StandardButton.Yes:
            return

        target = self.recorded_pose.copy()
        for name, value in zip(COORD_NAMES, target):
            self.move_entries[f"{name}:"].setText(f"{value:.2f}")
        result = self.run_command(
            "恢复记录位姿",
            lambda: self.client_dash.MovJ(
                *target,
                0,
                user=self.recorded_user_index,
                tool=self.recorded_tool_index,
                v=50,
            ),
        )
        if self.result_error_id(result) != 0:
            QMessageBox.warning(self, "恢复失败", f"恢复位姿指令执行失败：\n{result}")
            return
        self.recorded_pose_label.setText("恢复指令已发送（速度比例 50%）")

    def rotation_item_from_inputs(self, allow_manual_stop):
        angular_speed = float(self.angular_speed_edit.text())
        duration = float(self.rotation_duration_edit.text())
        if not 0.1 <= abs(angular_speed) <= 60.0:
            raise ValueError("角速度绝对值必须在 0.1～60 °/s 之间")
        if allow_manual_stop:
            if not 0.0 <= duration <= 60.0:
                raise ValueError("持续时间必须在 0～60 秒之间；0 表示手动停止")
        elif not 0.1 <= duration <= 60.0:
            raise ValueError("运动项持续时间必须在 0.1～60 秒之间")
        return {
            "axis": self.rotation_axis_combo.currentText(),
            "angular_speed": angular_speed,
            "duration": duration,
        }

    @staticmethod
    def format_rotation_plan_item(item, index=None):
        prefix = "" if index is None else f"{index}. "
        if item.get("type") == "vector":
            speeds = item["angular_speed_vector"]
            speed_norm = math.sqrt(sum(value * value for value in speeds))
            total_angle = speed_norm * item["duration"]
            return (
                f"{prefix}组合  "
                f"Rx={speeds[0]:+.2f}, Ry={speeds[1]:+.2f}, Rz={speeds[2]:+.2f} °/s  "
                f"{item['duration']:.2f} s  合速度 {speed_norm:.2f} °/s  "
                f"等效角度 {total_angle:.2f}°"
            )
        total_angle = abs(item["angular_speed"]) * item["duration"]
        return (
            f"{prefix}{item['axis']}  "
            f"{item['angular_speed']:+.2f} °/s  "
            f"{item['duration']:.2f} s  "
            f"总角度 {total_angle:.2f}°"
        )

    def refresh_rotation_plan_list(self):
        selected = self.rotation_plan_list.currentRow()
        self.rotation_plan_list.clear()
        for index, item in enumerate(self.rotation_plan_items, 1):
            text = self.format_rotation_plan_item(item, index)
            self.rotation_plan_list.addItem(text)
        if self.rotation_plan_items:
            selected = min(max(selected, 0), len(self.rotation_plan_items) - 1)
            self.rotation_plan_list.setCurrentRow(selected)

    def add_rotation_plan_item(self):
        try:
            item = self.rotation_item_from_inputs(allow_manual_stop=False)
        except ValueError as exc:
            QMessageBox.warning(self, "输入错误", str(exc))
            return
        self.rotation_plan_items.append(item)
        self.refresh_rotation_plan_list()
        self.rotation_status_label.setText(f"已添加 {len(self.rotation_plan_items)} 个运动项")
        self.append_log(f"已添加 IMU 运动项：{self.format_rotation_plan_item(item)}")

    def add_combined_rotation_plan_item(self):
        try:
            speeds = [
                float(self.combined_speed_edits[name].text())
                for name in ("Rx", "Ry", "Rz")
            ]
            duration = float(self.rotation_duration_edit.text())
            speed_norm = math.sqrt(sum(value * value for value in speeds))
            if not 0.1 <= speed_norm <= 60.0:
                raise ValueError("组合角速度合速度必须在 0.1～60 °/s 之间")
            if not 0.1 <= duration <= 60.0:
                raise ValueError("组合运动项持续时间必须在 0.1～60 秒之间")
            if any(abs(value) > 60.0 for value in speeds):
                raise ValueError("单轴角速度绝对值不能超过 60 °/s")
        except ValueError as exc:
            QMessageBox.warning(self, "输入错误", str(exc))
            return
        item = {
            "type": "vector",
            "axis": "组合",
            "angular_speed_vector": speeds,
            "duration": duration,
        }
        self.rotation_plan_items.append(item)
        self.refresh_rotation_plan_list()
        self.rotation_status_label.setText(f"已添加 {len(self.rotation_plan_items)} 个运动项")
        self.append_log(f"已添加 IMU 组合运动项：{self.format_rotation_plan_item(item)}")

    def remove_selected_rotation_plan_item(self):
        row = self.rotation_plan_list.currentRow()
        if row < 0 or row >= len(self.rotation_plan_items):
            QMessageBox.information(self, "未选择", "请先在运动项列表中选择一项")
            return
        removed = self.rotation_plan_items.pop(row)
        self.refresh_rotation_plan_list()
        self.append_log(f"已删除 IMU 运动项：{self.format_rotation_plan_item(removed)}")

    def clear_rotation_plan_items(self):
        self.rotation_plan_items.clear()
        self.refresh_rotation_plan_list()
        self.rotation_status_label.setText("运动项列表已清空")
        self.append_log("已清空 IMU 运动项列表")

    def toggle_continuous_rotation(self):
        if self.rotation_running or self.rotation_sequence_items:
            self.stop_continuous_rotation()
        else:
            self.start_continuous_rotation()

    def start_continuous_rotation(self):
        if not self.connected or self.client_dash is None:
            QMessageBox.warning(self, "尚未连接", "请先连接机械臂")
            return
        if self.latest_pose is None:
            QMessageBox.warning(self, "暂无反馈", "尚未收到机械臂实时位姿，请稍后再试")
            return
        if self.latest_robot_mode != 5:
            mode_text = ROBOT_MODES.get(self.latest_robot_mode, str(self.latest_robot_mode))
            QMessageBox.warning(
                self, "机器人未就绪", f"连续旋转要求机器人处于已使能且空闲状态。\n当前状态：{mode_text}"
            )
            return
        try:
            if self.rotation_plan_items:
                items = [dict(item) for item in self.rotation_plan_items]
            else:
                items = [self.rotation_item_from_inputs(allow_manual_stop=True)]
        except ValueError as exc:
            QMessageBox.warning(self, "输入错误", str(exc))
            return

        if not self.calibration_running:
            self.toggle_imu_calibration()

        self.rotation_sequence_items = items
        self.rotation_sequence_index = 0
        self.rotation_sequence_stopping = False
        self.rotation_sequence_blended = False
        if (
            all(item["duration"] > 0 for item in items)
            and (len(items) > 1 or any(item.get("type") == "vector" for item in items))
        ):
            self.start_blended_rotation_sequence(items)
        else:
            self.start_rotation_sequence_item()

    def start_blended_rotation_sequence(self, items):
        try:
            cp = int(self.rotation_cp_edit.text())
            if not 0 <= cp <= 100:
                raise ValueError
            transition_total = float(self.rotation_transition_time_edit.text())
            transition_steps = int(self.rotation_transition_steps_edit.text())
            if not 0.05 <= transition_total <= 3.0 or not 2 <= transition_steps <= 20:
                raise ValueError
        except ValueError:
            QMessageBox.warning(self, "输入错误", "CP 为 0～100；过渡时间为 0.05～3 秒；段数为 2～20")
            return

        # Convert every item to an Rx/Ry/Rz speed vector.  At each genuine
        # direction/speed change, insert a short average-speed item and remove
        # half of that time from either neighbour.  This preserves the total
        # angular displacement while replacing an instantaneous velocity change
        # by a physically smoother transition.
        def speed_vector(item):
            if item.get("type") == "vector":
                return [float(value) for value in item["angular_speed_vector"]]
            values = [0.0, 0.0, 0.0]
            values[("Rx", "Ry", "Rz").index(item["axis"])] = float(item["angular_speed"])
            return values

        source_items = [dict(item) for item in items]
        transition_after = [None] * max(0, len(source_items) - 1)
        duration_reduction = [0.0] * len(source_items)
        transition_count = 0
        for index in range(len(source_items) - 1):
            previous = source_items[index]
            following = source_items[index + 1]
            previous_speed = speed_vector(previous)
            following_speed = speed_vector(following)
            if max(abs(a - b) for a, b in zip(previous_speed, following_speed)) < 1e-6:
                continue
            transition_duration = min(transition_total, float(previous["duration"]) * 0.4, float(following["duration"]) * 0.4)
            if transition_duration <= 0.0:
                continue
            duration_reduction[index] += transition_duration / 2
            duration_reduction[index + 1] += transition_duration / 2
            transition_after[index] = [
                {
                    "type": "vector", "axis": "自动过渡",
                    "angular_speed_vector": [a + (b - a) * step / transition_steps for a, b in zip(previous_speed, following_speed)],
                    "duration": transition_duration / transition_steps,
                    "auto_transition": True,
                }
                for step in range(1, transition_steps + 1)
            ]
            transition_count += transition_steps

        smoothed_items = []
        for index, item in enumerate(source_items):
            item["duration"] = float(item["duration"]) - duration_reduction[index]
            smoothed_items.append(item)
            if index < len(transition_after) and transition_after[index] is not None:
                smoothed_items.extend(transition_after[index])

        planned_items = []
        try:
            for item in smoothed_items:
                if item.get("type") == "vector":
                    values = []
                    calibrated_flags = []
                    global_scales = []
                    for axis, speed in zip(
                        ("Rx", "Ry", "Rz"), item["angular_speed_vector"]
                    ):
                        if abs(speed) < 1e-9:
                            continue
                        axis_v, axis_global_scale, axis_calibrated = (
                            self.controller_v_for_rotation(
                                axis, speed, self.latest_tool_index
                            )
                        )
                        values.append(axis_v)
                        global_scales.append(axis_global_scale)
                        calibrated_flags.append(axis_calibrated)
                    controller_v = max(values) if values else 1
                    global_scale = global_scales[0] if global_scales else 1.0
                    calibrated = all(calibrated_flags) if calibrated_flags else False
                else:
                    controller_v, global_scale, calibrated = self.controller_v_for_rotation(
                        item["axis"], item["angular_speed"], self.latest_tool_index
                    )
                planned = dict(item)
                planned.update({
                    "controller_v": controller_v,
                    "global_speed_scale": global_scale,
                    "calibrated": calibrated,
                })
                planned_items.append(planned)
        except Exception as exc:
            QMessageBox.critical(self, "序列准备失败", f"无法准备 IMU 运动序列：\n{exc}")
            self.rotation_sequence_items = []
            self.rotation_sequence_stopping = False
            self.rotation_sequence_blended = False
            self.restore_rotation_controls()
            return

        self.rotation_sequence_items = planned_items
        self.rotation_sequence_index = 0
        self.rotation_sequence_blended = True
        self.rotation_diagnostics = None
        self.rotation_diagnostics = RotationSequenceDiagnostics()
        self.active_rotation_axis = planned_items[0]["axis"]
        self.rotation_thread = RotationPlanThread(
            self.client_dash,
            planned_items,
            lambda: self.latest_robot_mode,
            cp=cp,
            diagnostics=self.rotation_diagnostics,
            parent=self,
        )
        self.rotation_thread.progress.connect(self.on_rotation_progress)
        self.rotation_thread.rotation_done.connect(self.on_rotation_done)
        self.rotation_thread.rotation_error.connect(self.on_rotation_error)
        self.rotation_running = True
        self.rotation_button.setText("停止旋转")
        self.rotation_status_label.setText(f"连续序列：第 1/{len(planned_items)} 项")
        for button in self.command_buttons:
            button.setEnabled(button in (self.rotation_button, self.stop_motion_button))
        self.rotation_axis_combo.setEnabled(False)
        self.angular_speed_edit.setEnabled(False)
        self.rotation_duration_edit.setEnabled(False)
        for edit in self.combined_speed_edits.values():
            edit.setEnabled(False)
        self.rotation_cp_edit.setEnabled(False)
        self.rotation_transition_time_edit.setEnabled(False)
        self.rotation_transition_steps_edit.setEnabled(False)
        self.rotation_plan_list.setEnabled(False)
        self.rotation_plan_list.setCurrentRow(0)
        self.append_log(
            f"开始 IMU 连续运动序列：共 {len(planned_items)} 项，"
            f"使用 RelMovLTool 提前排队，项间 cp={cp} 平滑衔接；"
            f"已自动插入 {transition_count} 个速度过渡段（目标 {transition_total:.2f} s / 衔接）"
        )
        for index, item in enumerate(planned_items, 1):
            if item.get("type") == "vector":
                speeds = item["angular_speed_vector"]
                speed_norm = math.sqrt(sum(value * value for value in speeds))
                self.append_log(
                    f"序列第 {index} 项：Tool {self.latest_tool_index} 组合旋转，"
                    f"Rx={speeds[0]:+.2f}, Ry={speeds[1]:+.2f}, Rz={speeds[2]:+.2f} °/s，"
                    f"持续时间={item['duration']:g} s，合速度={speed_norm:.2f} °/s，"
                    f"v={item['controller_v']}%"
                )
            else:
                total_angle = abs(item["angular_speed"]) * item["duration"]
                self.append_log(
                    f"序列第 {index} 项：Tool {self.latest_tool_index} {item['axis']}，"
                    f"目标角速度={item['angular_speed']:+.2f} °/s，"
                    f"持续时间={item['duration']:g} s，总角度={total_angle:.2f}°，"
                    f"v={item['controller_v']}%"
                )
        self.append_log("连续序列模式下不按单项生成诊断 CSV；右侧曲线仍显示实时反馈角速度")
        self.rotation_thread.start()

    def start_rotation_sequence_item(self, retry_count=0):
        if self.rotation_sequence_stopping or not self.rotation_sequence_items:
            return
        if not self.connected or self.client_dash is None:
            self.rotation_sequence_items = []
            return
        if self.latest_pose is None:
            QMessageBox.warning(self, "暂无反馈", "尚未收到机械臂实时位姿，请稍后再试")
            self.rotation_sequence_items = []
            return
        if self.latest_robot_mode != 5:
            if retry_count < 15:
                self.rotation_status_label.setText("等待机械臂空闲...")
                QTimer.singleShot(
                    200,
                    lambda: self.start_rotation_sequence_item(retry_count + 1),
                )
                return
            mode_text = ROBOT_MODES.get(self.latest_robot_mode, str(self.latest_robot_mode))
            QMessageBox.warning(
                self,
                "机器人未就绪",
                f"下一项旋转要求机器人处于已使能且空闲状态。\n当前状态：{mode_text}",
            )
            self.rotation_sequence_items = []
            self.restore_rotation_controls()
            return

        item = self.rotation_sequence_items[self.rotation_sequence_index]
        axis = item["axis"]
        angular_speed = float(item["angular_speed"])
        duration = float(item["duration"])
        self.active_rotation_axis = axis
        total_angle = abs(angular_speed) * duration if duration > 0 else math.inf
        use_jog = duration == 0 or total_angle > 170.0
        try:
            if use_jog:
                global_scale, calibrated, full_jog_speed = self.prepare_jog_rotation_speed(
                    axis, angular_speed, self.latest_tool_index
                )
                controller_v = 100
                control_method = "MoveJogTool"
            else:
                controller_v, global_scale, calibrated = self.controller_v_for_rotation(
                    axis, angular_speed, self.latest_tool_index
                )
                full_jog_speed = None
                control_method = "RelMovLTool"
            self.rotation_diagnostics = RotationDiagnostics(
                axis,
                angular_speed,
                duration,
                self.latest_pose,
                self.latest_user_index,
                self.latest_tool_index,
                axis_frame="tool",
            )
            self.rotation_diagnostics.configure_motion(
                controller_v, global_scale, control_method
            )
        except Exception as exc:
            QMessageBox.critical(self, "旋转准备失败", f"无法准备 IMU 旋转：\n{exc}")
            self.rotation_sequence_items = []
            self.rotation_sequence_stopping = False
            self.restore_rotation_controls()
            return
        if use_jog:
            self.rotation_thread = JogRotationThread(
                self.client_dash,
                self.latest_pose,
                axis,
                angular_speed,
                duration,
                self.latest_user_index,
                self.latest_tool_index,
                diagnostics=self.rotation_diagnostics,
                parent=self,
            )
        else:
            self.rotation_thread = RotationThread(
                self.client_dash,
                self.latest_pose,
                axis,
                angular_speed,
                duration,
                controller_v,
                lambda: self.latest_robot_mode,
                diagnostics=self.rotation_diagnostics,
                parent=self,
            )
        self.rotation_thread.progress.connect(self.on_rotation_progress)
        self.rotation_thread.rotation_done.connect(self.on_rotation_done)
        self.rotation_thread.rotation_error.connect(self.on_rotation_error)
        self.rotation_running = True
        self.rotation_button.setText("停止旋转")
        total_items = len(self.rotation_sequence_items)
        item_text = (
            f"第 {self.rotation_sequence_index + 1}/{total_items} 项 "
            if total_items > 1 else ""
        )
        self.rotation_status_label.setText(
            f"{item_text}{axis}：{angular_speed:+.2f} °/s"
        )
        for button in self.command_buttons:
            button.setEnabled(button in (self.rotation_button, self.stop_motion_button))
        self.rotation_axis_combo.setEnabled(False)
        self.angular_speed_edit.setEnabled(False)
        self.rotation_duration_edit.setEnabled(False)
        for edit in self.combined_speed_edits.values():
            edit.setEnabled(False)
        self.rotation_cp_edit.setEnabled(False)
        self.rotation_transition_time_edit.setEnabled(False)
        self.rotation_transition_steps_edit.setEnabled(False)
        self.rotation_plan_list.setEnabled(False)
        if total_items > 1:
            self.rotation_plan_list.setCurrentRow(self.rotation_sequence_index)
        self.append_log(
            f"开始 IMU Tool 轴旋转：{item_text}Tool {self.latest_tool_index} {axis}，"
            f"目标角速度={angular_speed:+.2f} °/s，持续时间={duration:g} s"
        )
        if use_jog:
            self.append_log(
                f"控制方式：Tool MoveJog 控制器内部连续点动，"
                f"全局速度比例={global_scale * 100:.1f}%，"
                f"速度换算={'已使用点动标定' if calibrated else '使用初始估算，结束后自动标定'}"
            )
            if full_jog_speed is not None:
                self.append_log(f"估算的全速 Tool 点动速度={full_jog_speed:.2f} °/s")
            if duration > 0:
                self.append_log(f"长角度连续轨迹：目标总角度={total_angle:.2f}°，无中间姿态分段")
        else:
            self.append_log(
                f"控制方式：RelMovLTool 单段控制器插补，v={controller_v}%，"
                f"全局速度比例={global_scale * 100:.1f}%，"
                f"速度换算={'已标定' if calibrated else '首次保守估算，结束后自动标定'}"
            )
            self.append_log(f"有限角度单段轨迹：总角度={total_angle:.2f}°")
        self.append_log(f"诊断日志：{self.rotation_diagnostics.path}")
        self.rotation_status_label.setToolTip(str(self.rotation_diagnostics.path))
        self.rotation_thread.start()

    def restore_rotation_controls(self):
        self.rotation_button.setEnabled(True)
        self.rotation_button.setText("开始旋转")
        if self.connected:
            self.set_controls_enabled(True)
        self.rotation_axis_combo.setEnabled(True)
        self.angular_speed_edit.setEnabled(True)
        self.rotation_duration_edit.setEnabled(True)
        for edit in self.combined_speed_edits.values():
            edit.setEnabled(True)
        self.rotation_cp_edit.setEnabled(True)
        self.rotation_transition_time_edit.setEnabled(True)
        self.rotation_transition_steps_edit.setEnabled(True)
        self.rotation_plan_list.setEnabled(True)

    def stop_continuous_rotation(self):
        self.rotation_sequence_stopping = True
        self.rotation_sequence_items = []
        thread_running = (
            self.rotation_thread is not None and self.rotation_thread.isRunning()
        )
        if thread_running:
            self.rotation_thread.stop()
        self.rotation_button.setEnabled(False)
        self.rotation_status_label.setText("正在停止...")
        if not thread_running:
            self.rotation_running = False
            self.rotation_sequence_stopping = False
            self.rotation_sequence_blended = False
            self.restore_rotation_controls()
            self.rotation_status_label.setText("已停止")
            return
        if isinstance(self.rotation_thread, JogRotationThread):
            return
        self.run_command("停止连续旋转", self.client_dash.Stop)

    def shutdown_rotation(self):
        self.rotation_sequence_stopping = True
        self.rotation_sequence_items = []
        self.rotation_sequence_blended = False
        if self.rotation_thread is None or not self.rotation_thread.isRunning():
            self.rotation_running = False
            self.finish_rotation_diagnostics("连接关闭")
            return
        self.rotation_thread.stop()
        if self.client_dash is not None:
            try:
                self.client_dash.Stop()
            except Exception:
                pass
        self.rotation_thread.wait(2000)
        self.rotation_running = False
        self.finish_rotation_diagnostics("连接关闭")

    def finish_rotation_diagnostics(self, reason):
        diagnostics = self.rotation_diagnostics
        if diagnostics is None:
            return
        self.rotation_diagnostics = None
        try:
            summary = diagnostics.finish(reason)
        except Exception as exc:
            self.append_log(f"旋转诊断日志结束失败：{exc}")
            return
        if not summary:
            return

        if isinstance(diagnostics, RotationSequenceDiagnostics):
            actual = summary["actual_angular_speed_norm_deg_s"]
            self.append_log(f"多点平滑诊断 CSV 已保存：{diagnostics.path}")
            self.append_log(f"多点平滑诊断摘要已保存：{diagnostics.summary_path}")
            self.append_log(
                f"反馈帧={summary['feedback_count']}；实际角速度合速度均值="
                f"{actual['mean'] if actual['mean'] is not None else '无'} °/s"
            )
            self.rotation_status_label.setToolTip(
                f"CSV：{diagnostics.path}\n摘要：{diagnostics.summary_path}"
            )
            return

        feedback = summary["feedback"]
        plateau = summary.get("plateau_feedback", {})
        displayed_feedback = plateau if plateau.get("count", 0) else feedback
        interval = summary["command_interval_ms"]
        latency = summary["command_latency_ms"]

        def number(value, digits=3):
            return "无" if value is None else f"{value:.{digits}f}"

        self.append_log(f"旋转诊断 CSV 已保存：{diagnostics.path}")
        self.append_log(f"旋转诊断摘要已保存：{diagnostics.summary_path}")
        self.append_log(
            "诊断摘要："
            f"稳定段实际均值={number(displayed_feedback['mean'])} °/s，"
            f"标准差={number(displayed_feedback['std'])} °/s，"
            f"指令周期均值={number(interval['mean'])} ms，"
            f"周期标准差={number(interval['std'])} ms，"
            f"响应延迟均值={number(latency['mean'])} ms"
        )
        self.update_rotation_speed_calibration(summary)
        self.rotation_status_label.setToolTip(
            f"CSV：{diagnostics.path}\n摘要：{diagnostics.summary_path}"
        )

    def on_rotation_progress(self, angle):
        if isinstance(angle, dict):
            item_index = int(angle.get("item_index", self.rotation_sequence_index))
            self.rotation_sequence_index = item_index
            axis = angle.get("axis", self.active_rotation_axis or self.rotation_axis_combo.currentText())
            value = float(angle.get("angle", 0.0))
            total_items = len(self.rotation_sequence_items)
            if total_items > 1:
                self.rotation_plan_list.blockSignals(True)
                self.rotation_plan_list.setCurrentRow(item_index)
                self.rotation_plan_list.blockSignals(False)
                self.rotation_status_label.setText(
                    f"第 {item_index + 1}/{total_items} 项 旋转中：{axis} {value:+.2f}°"
                )
            else:
                self.rotation_status_label.setText(f"旋转中：{axis} {value:+.2f}°")
            return
        axis = self.active_rotation_axis or self.rotation_axis_combo.currentText()
        total_items = len(self.rotation_sequence_items)
        item_text = (
            f"第 {self.rotation_sequence_index + 1}/{total_items} 项 "
            if total_items > 1 else ""
        )
        self.rotation_status_label.setText(f"{item_text}旋转中：{axis} {angle:+.2f}°")

    def on_rotation_done(self, message):
        self.finish_rotation_diagnostics(message)
        self.rotation_running = False
        self.append_log(message)
        if (
            not self.rotation_sequence_stopping
            and self.connected
            and not self.rotation_sequence_blended
            and self.rotation_sequence_index + 1 < len(self.rotation_sequence_items)
        ):
            self.rotation_sequence_index += 1
            self.rotation_status_label.setText(
                f"准备第 {self.rotation_sequence_index + 1}/{len(self.rotation_sequence_items)} 项..."
            )
            QTimer.singleShot(250, self.start_rotation_sequence_item)
            return

        completed_count = len(self.rotation_sequence_items)
        self.rotation_sequence_items = []
        self.rotation_sequence_stopping = False
        self.rotation_sequence_blended = False
        self.restore_rotation_controls()
        if self.connected:
            if completed_count > 1:
                self.rotation_status_label.setText(f"运动序列已完成，共 {completed_count} 项")
                self.append_log(f"IMU 运动序列已完成，共 {completed_count} 项")
            else:
                self.rotation_status_label.setText(message)
        else:
            self.rotation_status_label.setText("等待连接")

    def on_rotation_error(self, message):
        self.finish_rotation_diagnostics(f"旋转失败：{message}")
        self.rotation_running = False
        self.rotation_sequence_items = []
        self.rotation_sequence_stopping = False
        self.rotation_sequence_blended = False
        self.rotation_status_label.setText("旋转失败" if self.connected else "等待连接")
        if self.connected:
            self.run_command("停止连续旋转", self.client_dash.Stop)
        self.restore_rotation_controls()
        self.append_log(f"连续旋转失败：{message}")
        if self.connected:
            QMessageBox.critical(self, "连续旋转失败", message)

    def toggle_imu_calibration(self):
        if self.calibration_running:
            self.calibration_running = False
            self.imu_start_button.setText("开始采集")
            self.imu_status_label.setText("已停止")
            self.append_log("IMU 校准数据采集已停止")
            return

        if not self.connected:
            QMessageBox.warning(self, "尚未连接", "请先连接机械臂")
            return

        for key, series in self.angular_series.items():
            series.clear()
            self.angular_samples[key].clear()
        self.chart_axis_x.setRange(0.0, self.chart_time_window)
        self.chart_axis_y.setRange(-1.0, 1.0)
        self.calibration_started_at = time.perf_counter()
        self.last_chart_update = 0.0
        self.calibration_running = True
        self.imu_start_button.setText("停止采集")
        self.imu_status_label.setText("采集中")
        self.append_log("IMU 校准数据采集已开始")

    def update_angular_speed_chart(self, angular_speed):
        if not self.calibration_running:
            return

        elapsed = time.perf_counter() - self.calibration_started_at
        # Robot feedback can be very fast; 20 Hz is sufficient for UI plotting.
        if elapsed - self.last_chart_update < 0.05:
            return
        self.last_chart_update = elapsed

        for key, value in zip(("x", "y", "z"), angular_speed):
            numeric_value = float(value)
            self.angular_series[key].append(elapsed, numeric_value)
            samples = self.angular_samples[key]
            samples.append(numeric_value)
            if len(samples) > self.chart_max_points:
                del samples[0]
            excess = self.angular_series[key].count() - self.chart_max_points
            if excess > 0:
                self.angular_series[key].removePoints(0, excess)

        x_max = max(self.chart_time_window, elapsed)
        self.chart_axis_x.setRange(max(0.0, x_max - self.chart_time_window), x_max)
        max_speed = max(
            (abs(value) for samples in self.angular_samples.values() for value in samples),
            default=1.0,
        )
        y_limit = max(1.0, max_speed * 1.2)
        self.chart_axis_y.setRange(-y_limit, y_limit)

    @staticmethod
    def angular_speed_to_tool_frame(pose, angular_speed):
        rx, ry, rz = (math.radians(float(value)) for value in pose[3:6])
        tool_axes = (
            (
                math.cos(rz) * math.cos(ry),
                math.sin(rz) * math.cos(ry),
                -math.sin(ry),
            ),
            (
                math.cos(rz) * math.sin(ry) * math.sin(rx) - math.sin(rz) * math.cos(rx),
                math.sin(rz) * math.sin(ry) * math.sin(rx) + math.cos(rz) * math.cos(rx),
                math.cos(ry) * math.sin(rx),
            ),
            (
                math.cos(rz) * math.sin(ry) * math.cos(rx) + math.sin(rz) * math.sin(rx),
                math.sin(rz) * math.sin(ry) * math.cos(rx) - math.cos(rz) * math.sin(rx),
                math.cos(ry) * math.cos(rx),
            ),
        )
        return [
            sum(float(value) * direction for value, direction in zip(angular_speed, axis))
            for axis in tool_axes
        ]

    def update_feedback(self, state):
        self.latest_pose = list(state["pose"])
        self.latest_robot_mode = state["mode"]
        self.latest_user_index = state["user"]
        self.latest_tool_index = state["tool"]
        self.latest_speed_scaling = float(state["speed"])
        self.speed_feedback.setText(f"{state['speed']:.4g}")
        self.mode_feedback.setText(ROBOT_MODES.get(state["mode"], f"未知模式（{state['mode']}）"))
        self.di_feedback.setText(format(state["di"], "064b"))
        self.do_feedback.setText(format(state["do"], "064b"))
        for name, value in zip(JOINT_NAMES, state["joints"]):
            self.feedback_labels[f"{name}:"].setText(f"{value:.4f}")
        for name, value in zip(COORD_NAMES, state["pose"]):
            self.feedback_labels[f"{name}:"].setText(f"{value:.4f}")
        if self.active_jog_command is not None:
            axis_name = self.active_jog_command[:2]
            axis_index = {"Rx": 0, "Ry": 1, "Rz": 2}.get(axis_name)
            if axis_index is not None:
                elapsed = time.perf_counter() - self.jog_started_at
                value = float(state["angular_speed"][axis_index])
                self.jog_angular_samples.append((elapsed, value))
        if self.rotation_running and self.rotation_diagnostics is not None:
            try:
                self.rotation_diagnostics.log_feedback(state)
            except Exception as exc:
                self.append_log(f"旋转反馈日志写入失败：{exc}")
                self.finish_rotation_diagnostics("日志写入失败")
        tool_angular_speed = self.angular_speed_to_tool_frame(
            state["pose"], state["angular_speed"]
        )
        self.latest_tool_angular_speed = tool_angular_speed
        self.update_angular_speed_chart(tool_angular_speed)

        if state["mode"] == 9 and not self.alarm_requested:
            self.alarm_requested = True
            self.request_alarm_info()
        elif state["mode"] != 9:
            self.alarm_requested = False

    def request_alarm_info(self):
        if self.alarm_thread is not None and self.alarm_thread.isRunning():
            return
        self.alarm_thread = AlarmThread(
            self.client_dash, self.controller_alarms, self.servo_alarms, self
        )
        self.alarm_thread.result.connect(self.display_alarm_result)
        self.alarm_thread.start()

    def display_alarm_result(self, result):
        kind, payload = result
        if kind == "new":
            for error in payload:
                self.error_text.append(
                    f"时间：{error.get('date', '无')} {error.get('time', '无')}\n"
                    f"编号：{error.get('id', '无')}\n类型：{error.get('mode', '无')}\n"
                    f"等级：{error.get('level', '无')}\n"
                    f"描述：{error.get('description', '无')}\n"
                    f"解决方法：{error.get('solution', '无')}\n"
                )
        elif kind == "legacy":
            for error_id, alarm_dict, error_type in payload:
                alarm = alarm_dict.get(error_id)
                if alarm:
                    self.error_text.append(
                        f"时间：{time.strftime('%Y-%m-%d %H:%M:%S')}\n"
                        f"编号：{error_id}\n类型：{error_type}\n等级：{alarm['level']}\n"
                        f"解决方法：{alarm['zh_CN']['solution']}\n"
                    )
        else:
            self.append_log(f"报警信息读取失败：{payload}")

    def closeEvent(self, event):
        if self.connected:
            self.disconnect_robot()
        if self.alarm_thread is not None and self.alarm_thread.isRunning():
            self.alarm_thread.requestInterruption()
            self.alarm_thread.wait(6000)
        event.accept()


def run():
    app = QApplication.instance() or QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setFont(QFont("Microsoft YaHei UI", 10))
    window = RobotUI()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(run())
