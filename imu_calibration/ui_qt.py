# -*- coding: utf-8 -*-
"""PySide6 migration of the original Tkinter Dobot demo interface."""

import json
import math
import re
import sys
import time
from pathlib import Path

import numpy as np
from PySide6.QtCore import QThread, Qt, Signal
from PySide6.QtGui import QBrush, QColor, QFont, QPainter, QPen
from PySide6.QtCharts import QChart, QChartView, QLineSeries, QValueAxis
from PySide6.QtWidgets import (
    QApplication, QComboBox, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QLineEdit, QMainWindow, QMessageBox, QPushButton,
    QSplitter, QTextEdit, QVBoxLayout, QWidget,
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

MODERN_STYLE = """
QWidget {
    color: #1f2937;
    font-family: "Microsoft YaHei UI", "Segoe UI", sans-serif;
    font-size: 13px;
}
QWidget#appBackground {
    background: #f3f6fa;
}
QWidget#pageHeader {
    background: #ffffff;
    border: 1px solid #dfe7f1;
    border-radius: 12px;
}
QLabel#pageTitle {
    color: #10213a;
    font-size: 20px;
    font-weight: 700;
}
QLabel#pageSubtitle {
    color: #718096;
    font-size: 12px;
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
QPushButton[jog="true"]:disabled,
QPushButton[accent="true"]:disabled,
QPushButton[warning="true"]:disabled {
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
                    "di": int(packet["DigitalInputs"][0]),
                    "do": int(packet["DigitalOutputs"][0]),
                    "joints": np.asarray(packet["QActual"][0], dtype=float).tolist(),
                    "pose": np.asarray(packet["ToolVectorActual"][0], dtype=float).tolist(),
                    "user": int(packet["User"][0]),
                    "tool": int(packet["Tool"][0]),
                    "angular_speed": np.asarray(
                        packet["TCPSpeedActual"][0][3:6], dtype=float
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


class RotationThread(QThread):
    """Continuously update one TCP Euler component with ServoP."""

    progress = Signal(float)
    rotation_done = Signal(str)
    rotation_error = Signal(str)

    def __init__(self, client, start_pose, axis, angular_speed, duration, parent=None):
        super().__init__(parent)
        self.client = client
        self.start_pose = list(start_pose)
        self.axis_index = {"Rx": 3, "Ry": 4, "Rz": 5}[axis]
        self.angular_speed = float(angular_speed)
        self.duration = float(duration)
        self.period = 0.05
        self.running = True

    def stop(self):
        self.running = False

    @staticmethod
    def normalize_angle(angle):
        return (angle + 180.0) % 360.0 - 180.0

    def run(self):
        started_at = time.perf_counter()
        next_send = started_at
        last_progress = -1.0
        try:
            while self.running:
                now = time.perf_counter()
                elapsed = now - started_at
                duration_reached = self.duration > 0 and elapsed >= self.duration
                motion_time = self.duration if duration_reached else elapsed

                target = self.start_pose.copy()
                rotation_angle = self.angular_speed * motion_time
                target[self.axis_index] = self.normalize_angle(
                    self.start_pose[self.axis_index] + rotation_angle
                )
                response = self.client.ServoP(*target, t=self.period)
                match = re.match(r"\s*(-?\d+)", str(response))
                if match and int(match.group(1)) != 0:
                    raise RuntimeError(f"ServoP 返回错误：{response}")

                if elapsed - last_progress >= 0.2:
                    self.progress.emit(rotation_angle)
                    last_progress = elapsed

                if duration_reached:
                    break

                next_send += self.period
                sleep_seconds = next_send - time.perf_counter()
                if sleep_seconds > 0:
                    self.msleep(max(1, int(sleep_seconds * 1000)))

            message = "旋转已完成" if self.running else "旋转已手动停止"
            self.rotation_done.emit(message)
        except Exception as exc:
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
        self.connected = False
        self.enabled = False
        self.latest_pose = None
        self.latest_robot_mode = None
        self.latest_user_index = 0
        self.latest_tool_index = 0
        self.recorded_pose = None
        self.recorded_user_index = 0
        self.recorded_tool_index = 0
        self.recorded_at = None
        self.rotation_running = False
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
        self.controller_alarms = {item["id"]: item for item in alarm_controller_list}
        self.servo_alarms = {item["id"]: item for item in alarm_servo_list}

        self.build_ui()
        self.load_recorded_pose()
        self.set_controls_enabled(False)

    def build_ui(self):
        central = QWidget()
        central.setObjectName("appBackground")
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(18, 14, 18, 16)
        root.setSpacing(10)
        root.addWidget(self.build_header())

        top_controls = QHBoxLayout()
        top_controls.setSpacing(10)
        top_controls.addWidget(self.build_connection_group(), 6)
        top_controls.addWidget(self.build_dashboard_group(), 5)
        root.addLayout(top_controls)
        root.addWidget(self.build_move_group())

        feedback_splitter = QSplitter(Qt.Orientation.Horizontal)
        feedback_splitter.addWidget(self.build_feedback_group())
        feedback_splitter.addWidget(self.build_log_group())
        feedback_splitter.setSizes([800, 440])
        feedback_splitter.setStretchFactor(0, 2)
        feedback_splitter.setStretchFactor(1, 1)
        feedback_splitter.setHandleWidth(8)
        feedback_splitter.setMinimumHeight(240)

        content_splitter = QSplitter(Qt.Orientation.Vertical)
        content_splitter.addWidget(feedback_splitter)
        content_splitter.addWidget(self.build_imu_calibration_group())
        content_splitter.setSizes([260, 410])
        content_splitter.setStretchFactor(0, 0)
        content_splitter.setStretchFactor(1, 1)
        content_splitter.setHandleWidth(9)
        root.addWidget(content_splitter, 1)

        self.setStyleSheet(MODERN_STYLE)

    def build_header(self):
        header = QWidget()
        header.setObjectName("pageHeader")
        row = QHBoxLayout(header)
        row.setContentsMargins(18, 10, 18, 10)

        text_layout = QVBoxLayout()
        text_layout.setSpacing(1)
        title = QLabel("Dobot 机械臂控制与 IMU 校准")
        title.setObjectName("pageTitle")
        subtitle = QLabel("实时控制、状态监测与末端旋转速度分析")
        subtitle.setObjectName("pageSubtitle")
        text_layout.addWidget(title)
        text_layout.addWidget(subtitle)
        row.addLayout(text_layout)
        row.addStretch()

        self.connection_status_label = QLabel("●  未连接")
        self.connection_status_label.setObjectName("connectionBadge")
        self.connection_status_label.setProperty("connected", False)
        row.addWidget(self.connection_status_label)
        return header

    def build_connection_group(self):
        group = QGroupBox("机器人连接")
        group.setObjectName("compactCard")
        row = QHBoxLayout(group)
        row.setContentsMargins(9, 7, 9, 6)
        row.setSpacing(6)
        row.addWidget(QLabel("IP 地址："))
        self.ip_edit = QLineEdit("192.168.5.1")
        self.ip_edit.setFixedWidth(115)
        row.addWidget(self.ip_edit)
        row.addSpacing(7)
        row.addWidget(QLabel("控制端口："))
        self.dashboard_port_edit = QLineEdit("29999")
        self.dashboard_port_edit.setFixedWidth(62)
        row.addWidget(self.dashboard_port_edit)
        row.addSpacing(7)
        row.addWidget(QLabel("反馈端口："))
        self.feedback_port_edit = QLineEdit("30004")
        self.feedback_port_edit.setFixedWidth(62)
        row.addWidget(self.feedback_port_edit)
        row.addStretch()
        self.connect_button = QPushButton("连接")
        self.connect_button.setProperty("accent", True)
        self.connect_button.setFixedWidth(82)
        self.connect_button.clicked.connect(self.toggle_connection)
        row.addWidget(self.connect_button)
        return group

    def build_dashboard_group(self):
        group = QGroupBox("机器人控制")
        group.setObjectName("compactCard")
        grid = QGridLayout(group)
        grid.setContentsMargins(9, 7, 9, 6)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(5)
        self.enable_button = self.command_button("使能", self.toggle_enable)
        grid.addWidget(self.enable_button, 0, 0)
        grid.addWidget(self.command_button("清除错误", self.clear_error), 0, 2)
        grid.addWidget(QLabel("速度比例："), 0, 4)
        self.speed_edit = QLineEdit("50")
        self.speed_edit.setFixedWidth(48)
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
        group = QGroupBox("运动控制")
        grid = QGridLayout(group)
        self.add_move_row(grid, 0, COORD_NAMES, ("600", "-260", "380", "170", "12", "140"))
        grid.addWidget(self.command_button("关节运动", self.move_pose_j), 0, 12)
        grid.addWidget(self.command_button("直线运动", self.move_pose_l), 0, 13)
        self.add_move_row(grid, 1, JOINT_NAMES, ("0", "-20", "-80", "30", "90", "120"))
        grid.addWidget(self.command_button("关节运动", self.move_joint_j), 1, 12)
        grid.addWidget(self.build_pose_record_group(), 0, 14, 2, 1)
        grid.setColumnStretch(14, 1)
        return group

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
        group = QGroupBox("状态反馈")
        outer = QVBoxLayout(group)
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

        middle = QHBoxLayout()
        jog_widget = QWidget()
        jog_grid = QGridLayout(jog_widget)
        jog_grid.setContentsMargins(0, 0, 0, 0)
        jog_grid.setHorizontalSpacing(5)
        jog_grid.setVerticalSpacing(3)
        self.add_jog_columns(jog_grid, 0, JOINT_NAMES)
        jog_grid.setColumnMinimumWidth(4, 14)
        self.add_jog_columns(jog_grid, 5, COORD_NAMES)
        middle.addWidget(jog_widget, 4)

        error_group = QGroupBox("报警信息")
        error_layout = QVBoxLayout(error_group)
        self.error_text = QTextEdit()
        self.error_text.setReadOnly(True)
        error_layout.addWidget(self.error_text)
        error_layout.addWidget(
            self.command_button("清空", self.error_text.clear),
            alignment=Qt.AlignmentFlag.AlignRight,
        )
        middle.addWidget(error_group, 2)
        outer.addLayout(middle, 1)
        return group

    def add_jog_columns(self, grid, start_column, names):
        for row, name in enumerate(names):
            minus = self.jog_button(f"{name}-")
            plus = self.jog_button(f"{name}+")
            value = QLabel(" ")
            value.setProperty("value", True)
            value.setMinimumWidth(54)
            value.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            self.feedback_labels[f"{name}:"] = value
            grid.addWidget(minus, row, start_column)
            grid.addWidget(QLabel(f"{name}:"), row, start_column + 1)
            grid.addWidget(value, row, start_column + 2)
            grid.addWidget(plus, row, start_column + 3)

    def build_log_group(self):
        group = QGroupBox("运行日志")
        layout = QVBoxLayout(group)
        self.log_text = QTextEdit()
        self.log_text.setObjectName("logText")
        self.log_text.setReadOnly(True)
        layout.addWidget(self.log_text)
        return group

    def build_imu_calibration_group(self):
        group = QGroupBox("IMU 校准")
        layout = QHBoxLayout(group)

        controls = QWidget()
        controls.setFixedWidth(420)
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

        rotation_group = QGroupBox("指定角速度连续旋转")
        rotation_grid = QGridLayout(rotation_group)
        rotation_grid.setContentsMargins(8, 8, 8, 6)
        rotation_grid.setHorizontalSpacing(6)
        rotation_grid.setVerticalSpacing(5)
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
        controls_layout.addWidget(rotation_group)

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
        layout.addWidget(controls)

        chart = QChart()
        chart.setTitle("机械臂末端绕各轴的旋转速度")
        chart.setTitleFont(QFont("Microsoft YaHei UI", 11, QFont.Weight.DemiBold))
        chart.setTitleBrush(QBrush(QColor("#344054")))
        chart.setBackgroundVisible(False)
        chart.setPlotAreaBackgroundVisible(True)
        chart.setPlotAreaBackgroundBrush(QBrush(QColor("#fbfdff")))
        chart.legend().setVisible(True)
        chart.legend().setLabelColor(QColor("#475467"))
        series_config = (
            ("x", "绕 X 轴", "#2563eb"),
            ("y", "绕 Y 轴", "#f59e0b"),
            ("z", "绕 Z 轴", "#10b981"),
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
        chart_view.setMinimumHeight(330)
        layout.addWidget(chart_view, 1)
        return group

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
        self.tool_status_label.setText("当前使用 Tool 0（法兰坐标系）")
        self.append_log(f"已连接机器人：{ip}")

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
        self.connect_button.setText("连接")
        self.enable_button.setText("使能")
        self.imu_start_button.setText("开始采集")
        self.imu_status_label.setText("等待连接")
        self.rotation_button.setText("开始旋转")
        self.rotation_status_label.setText("等待连接")
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
        if command.startswith("J"):
            self.run_command("关节点动", lambda: self.client_dash.MoveJog(command))
        else:
            self.run_command(
                "坐标点动", lambda: self.client_dash.MoveJog(command, coordtype=1, user=0, tool=0)
            )

    def stop_jog(self):
        self.run_command("停止点动", lambda: self.client_dash.MoveJog(""))

    @staticmethod
    def result_error_id(result):
        match = re.match(r"\s*(-?\d+)", str(result)) if result is not None else None
        return int(match.group(1)) if match else None

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
        self.tool_status_label.setText(f"Tool {tool_index} 已保存并启用")

    def restore_tool_zero(self):
        result = self.run_command("恢复法兰坐标系 Tool 0", lambda: self.client_dash.Tool(0))
        if self.result_error_id(result) != 0:
            QMessageBox.warning(self, "设置失败", f"Tool 0 启用失败：\n{result}")
            return
        self.active_tool_index = 0
        self.tool_status_label.setText("当前使用 Tool 0（法兰坐标系）")

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
        recorded_at = time.strftime("%H:%M:%S")
        self.recorded_pose_label.setText(
            f"已记录 {recorded_at}｜User {self.recorded_user_index} / Tool {self.recorded_tool_index}"
        )
        pose_text = ", ".join(
            f"{name}={value:.4f}" for name, value in zip(COORD_NAMES, self.recorded_pose)
        )
        self.recorded_pose_label.setToolTip(pose_text)
        self.restore_pose_button.setEnabled(True)
        self.append_log(
            f"已记录当前位姿：{pose_text}，User={self.recorded_user_index}，Tool={self.recorded_tool_index}"
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
            f"机械臂将以 10% 速度比例运动到记录位姿：\n\n{pose_text}\n\n确认执行吗？",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if answer != QMessageBox.StandardButton.Yes:
            return

        target = self.recorded_pose.copy()
        result = self.run_command(
            "恢复记录位姿",
            lambda: self.client_dash.MovJ(
                *target,
                0,
                user=self.recorded_user_index,
                tool=self.recorded_tool_index,
                v=10,
            ),
        )
        if self.result_error_id(result) != 0:
            QMessageBox.warning(self, "恢复失败", f"恢复位姿指令执行失败：\n{result}")
            return
        self.recorded_pose_label.setText("恢复指令已发送（速度比例 10%）")

    def toggle_continuous_rotation(self):
        if self.rotation_running:
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
            angular_speed = float(self.angular_speed_edit.text())
            duration = float(self.rotation_duration_edit.text())
            if not 0.1 <= abs(angular_speed) <= 60.0:
                raise ValueError("角速度绝对值必须在 0.1～60 °/s 之间")
            if not 0.0 <= duration <= 60.0:
                raise ValueError("持续时间必须在 0～60 秒之间；0 表示手动停止")
        except ValueError as exc:
            QMessageBox.warning(self, "输入错误", str(exc))
            return

        if not self.calibration_running:
            self.toggle_imu_calibration()

        axis = self.rotation_axis_combo.currentText()
        self.active_rotation_axis = axis
        self.rotation_thread = RotationThread(
            self.client_dash,
            self.latest_pose,
            axis,
            angular_speed,
            duration,
            self,
        )
        self.rotation_thread.progress.connect(self.on_rotation_progress)
        self.rotation_thread.rotation_done.connect(self.on_rotation_done)
        self.rotation_thread.rotation_error.connect(self.on_rotation_error)
        self.rotation_running = True
        self.rotation_button.setText("停止旋转")
        self.rotation_status_label.setText(
            f"{axis}：{angular_speed:+.2f} °/s"
        )
        for button in self.command_buttons:
            button.setEnabled(button is self.rotation_button)
        self.rotation_axis_combo.setEnabled(False)
        self.angular_speed_edit.setEnabled(False)
        self.rotation_duration_edit.setEnabled(False)
        self.append_log(
            f"开始连续旋转：轴={axis}，角速度={angular_speed:+.2f} °/s，持续时间={duration:g} s"
        )
        self.rotation_thread.start()

    def stop_continuous_rotation(self):
        if self.rotation_thread is not None and self.rotation_thread.isRunning():
            self.rotation_thread.stop()
        self.rotation_button.setEnabled(False)
        self.rotation_status_label.setText("正在停止...")
        self.run_command("停止连续旋转", self.client_dash.Stop)

    def shutdown_rotation(self):
        if self.rotation_thread is None or not self.rotation_thread.isRunning():
            self.rotation_running = False
            return
        self.rotation_thread.stop()
        if self.client_dash is not None:
            try:
                self.client_dash.Stop()
            except Exception:
                pass
        self.rotation_thread.wait(2000)
        self.rotation_running = False

    def on_rotation_progress(self, angle):
        axis = self.active_rotation_axis or self.rotation_axis_combo.currentText()
        self.rotation_status_label.setText(f"旋转中：{axis} {angle:+.2f}°")

    def on_rotation_done(self, message):
        self.rotation_running = False
        self.rotation_button.setText("开始旋转")
        self.rotation_status_label.setText(message if self.connected else "等待连接")
        if self.connected:
            self.set_controls_enabled(True)
        self.rotation_axis_combo.setEnabled(True)
        self.angular_speed_edit.setEnabled(True)
        self.rotation_duration_edit.setEnabled(True)
        self.append_log(message)

    def on_rotation_error(self, message):
        self.rotation_running = False
        self.rotation_button.setText("开始旋转")
        self.rotation_status_label.setText("旋转失败" if self.connected else "等待连接")
        if self.connected:
            self.set_controls_enabled(True)
            self.run_command("停止连续旋转", self.client_dash.Stop)
        self.rotation_axis_combo.setEnabled(True)
        self.angular_speed_edit.setEnabled(True)
        self.rotation_duration_edit.setEnabled(True)
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

    def update_feedback(self, state):
        self.latest_pose = list(state["pose"])
        self.latest_robot_mode = state["mode"]
        self.latest_user_index = state["user"]
        self.latest_tool_index = state["tool"]
        self.speed_feedback.setText(f"{state['speed']:.4g}")
        self.mode_feedback.setText(ROBOT_MODES.get(state["mode"], f"未知模式（{state['mode']}）"))
        self.di_feedback.setText(format(state["di"], "064b"))
        self.do_feedback.setText(format(state["do"], "064b"))
        for name, value in zip(JOINT_NAMES, state["joints"]):
            self.feedback_labels[f"{name}:"].setText(f"{value:.4f}")
        for name, value in zip(COORD_NAMES, state["pose"]):
            self.feedback_labels[f"{name}:"].setText(f"{value:.4f}")
        self.update_angular_speed_chart(state["angular_speed"])

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
