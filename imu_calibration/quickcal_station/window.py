"""Dedicated production UI for 11-lane IMU QuickCal."""

from __future__ import annotations

import json
import math
from pathlib import Path
import time

from PySide6.QtCore import Qt, QTimer, Slot
from PySide6.QtGui import QColor, QCloseEvent, QFont
from PySide6.QtWidgets import (
    QAbstractItemView,
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QProgressBar,
    QPushButton,
    QSpinBox,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from .coordinator import QuickCalCoordinator, RunState
from .action_config import (
    DEFAULT_ACTIONS,
    RobotActionConfig,
    gravity_after_tool_rotation,
    load_action_config,
    save_action_config,
    vector_angle_deg,
)
from .glove_device import GloveDevice
from .pose_store import TaughtPose, load_legacy_safe_pose, load_pose_config, save_pose_config
from .protocol import ALL_IMU_MASK, EXPECTED_GYRO_SEGMENTS
from .robot_device import RobotDevice
from .workflow import (
    LIMITED_GYRO_ACCEL_DECEL_DEG,
    LIMITED_GYRO_CAPTURE_BOUND_DEG,
    LIMITED_GYRO_CAPTURE_S,
    LIMITED_GYRO_OUTER_DEG,
    IMU_NAMES,
    ROLL_PITCH_GYRO_RATE_DEG_S,
    YawLimits,
    expected_capture_seconds,
    expected_total_seconds,
    steps_for_limits,
)


APP_DIR = Path(__file__).resolve().parent.parent
DEFAULT_OUTPUT_DIR = APP_DIR / "quickcal_records"
RECORDED_POSE_FILE = APP_DIR / "recorded_pose.json"
POSE_CONFIG_FILE = APP_DIR / "quickcal_poses.local.json"
ACTION_CONFIG_FILE = APP_DIR / "quickcal_actions.local.json"
ROTATION_SPEED_CALIBRATION_FILE = APP_DIR / "rotation_speed_calibration.json"
TOOL_CONFIG_FILE = APP_DIR / "tool_offset_config.json"

MCAL_STATUS_LABELS = {
    1: "状态不允许",
    5: "Flash 写入失败",
    11: "质量不足",
    15: "阶段状态错误",
}

MANUAL_POSITION_TOLERANCE_MM = 2.0
MANUAL_ANGLE_TOLERANCE_DEG = 1.0
MANUAL_MOTION_START_GRACE_NS = 1_500_000_000
MANUAL_MOTION_STABLE_NS = 500_000_000
MANUAL_MOTION_TIMEOUT_NS = 60_000_000_000

GYRO_AUTO_STEPS = ("G01", "G02", "G03", "G04", "G05", "G06")
GYRO_LIMITED_STEPS = ("G01", "G02")
G01_RESTORE_RX_DEG = 90.0
G02_RESTORE_NEUTRAL_RZ_DEG = 90.0
GYRO_RATE_TOLERANCE_DEG_S = 3.0
GYRO_SPEED_STABLE_NS = 300_000_000
GYRO_TIMEOUT_S = 90.0
GYRO_MISSED_START_MARGIN_DEG = 2.0
GYRO_ENDPOINT_TOLERANCE_DEG = 2.0
GYRO_POSTPOSITION_SPEED_PERCENT = 15
GYRO_POSTPOSITION_ACCEL_PERCENT = 5
GYRO_TRANSIT_SPEED_PERCENT = 80
GYRO_RETURN_SPEED_PERCENT = 100
NEUTRAL_RETURN_SPEED_PERCENT = 60
# Fraction of the outer region already travelled, followed by the fraction of
# the calibrated jog SpeedFactor to retain.  The fitted capture window has
# already closed before this S-shaped deceleration starts.
GYRO_SMOOTH_DECEL_PROFILE = (
    (0.00, 0.65),
    (0.12, 0.50),
    (0.28, 0.35),
    (0.44, 0.25),
    (0.60, 0.18),
    (0.74, 0.12),
    (0.86, 0.07),
)
# Stop before the nominal outer endpoint, then use the slow post-position
# move for the final correction. Feedback arrives discretely while braking.
GYRO_SMOOTH_STOP_MARGIN_DEG = 1.25
# This is only a braking tolerance outside the closed ±45° capture window;
# the robot is corrected back to the nominal ±55° outer endpoint afterwards.
GYRO_DECEL_OVERSHOOT_TOLERANCE_DEG = 2.0

MAG_AUTO_STEPS = ("M01", "M02", "M03", "M04")
MAG_RATE_TOLERANCE_DEG_S = 4.0
MAG_TIMEOUT_S = 90.0
MAG_NEUTRAL_TOLERANCE_DEG = 2.0
MAG_M04_STOP_SETTLE_S = 1.0
MAG_M04_MOTION_GRACE_S = 0.5
MAG_M01_PITCH_SAFE_DEG = 75.0
MAG_M01_J6_SAFE_DEG = 75.0
MAG_M01_XYZ_TOLERANCE_MM = 2.0
MAG_M01_SEGMENT_S = 5.0
MAG_M01_GLOBAL_SPEED_FACTOR = 100
# Queue all intermediate Cartesian waypoints with full controller blending.
# The final point remains CP=0 so the robot settles at the entry TCP pose.
MAG_M01_BLEND_PERCENT = 100
MAG_YAW_SAFE_DEG = 45.0
MAG_TARGET_RATE_DEG_S = {"M02": 9.0, "M03": 9.0}
# M01 retains the entry TCP XYZ exactly as G03/G04 do. The target orientation
# is composed in the entry Tool frame as R0 * Ry(pitch) * Rz(roll). The Rz
# component is the tool-axis rotation produced by J6; composing it before the
# Cartesian command avoids over-constraining an Ry-only inverse solution and
# then invalidating that solution by overwriting J6. Each tuple is
# (Tool Ry pitch, J6-direction Tool Rz roll).
_MAG_M01_DIAGONAL_J6_DEG = MAG_M01_J6_SAFE_DEG / math.sqrt(2.0)
MAG_M01_POSE_WAYPOINTS = (
    (0.0, 0.0),
    (-MAG_M01_PITCH_SAFE_DEG, +_MAG_M01_DIAGONAL_J6_DEG),
    (0.0, +MAG_M01_J6_SAFE_DEG),
    (+MAG_M01_PITCH_SAFE_DEG, +_MAG_M01_DIAGONAL_J6_DEG),
    (0.0, 0.0),
    (-MAG_M01_PITCH_SAFE_DEG, -_MAG_M01_DIAGONAL_J6_DEG),
    (0.0, -MAG_M01_J6_SAFE_DEG),
    (+MAG_M01_PITCH_SAFE_DEG, -_MAG_M01_DIAGONAL_J6_DEG),
    (0.0, 0.0),
)
# M02/M03 use the fixture's Tool Rx axis for one-sided Yaw jog paths from
# neutral to the safe endpoint and back to neutral.
MAG_TRAJECTORIES = {
    "M01": (),
    "M02": (("Rx", True, 5.0), ("Rx", False, 5.0)),
    "M03": (("Rx", False, 5.0), ("Rx", True, 5.0)),
    "M04": (),
}
MAG_STEP_BUTTON_TEXT = {
    "M01": "自动执行 M01 固定 XYZ 俯仰/J6 组合往复",
    "M02": "自动执行 M02 Yaw 正向往返",
    "M03": "自动执行 M03 Yaw 负向往返",
    "M04": "自动执行 M04 中位静止",
}
MAG_REPORT_SENSOR_NAMES = (
    "MAG · MMC5983MA",
    "MAG · BMM350@0x14",
    "MAG · BMM350@0x15",
)


STYLE = """
QWidget { color: #1f2937; font-family: "Microsoft YaHei UI", "Segoe UI"; font-size: 12px; }
QMainWindow, QWidget#root { background: #f3f6fa; }
QGroupBox { background: #ffffff; border: 1px solid #c7d2df; border-radius: 8px;
            margin-top: 12px; padding: 10px 8px 7px 8px; font-weight: 600; }
QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 5px; color: #24324a; }
QLabel { color: #26354d; }
QLineEdit, QComboBox, QSpinBox { min-height: 27px; color: #172033; background: #ffffff;
                               border: 1px solid #aebdce; border-radius: 5px; padding: 1px 6px;
                               selection-color: #ffffff; selection-background-color: #1d4ed8; }
QLineEdit:focus, QComboBox:focus, QSpinBox:focus { border: 2px solid #2563eb; }
QLineEdit:disabled, QComboBox:disabled, QSpinBox:disabled {
    color: #526173; background: #e6ebf1; border-color: #c1ccd8;
}
QComboBox QAbstractItemView { color: #172033; background: #ffffff; border: 1px solid #94a3b8;
                             selection-color: #ffffff; selection-background-color: #2563eb; }
QPushButton { min-height: 29px; color: #ffffff; background: #2563eb; border: 1px solid #2563eb;
              border-radius: 5px; padding: 0 10px; font-weight: 600; }
QPushButton:hover { background: #1d4ed8; }
QPushButton:pressed { background: #1e40af; }
QPushButton:focus { border: 2px solid #93c5fd; }
QPushButton:disabled { color: #5d6b7d; background: #e1e7ee; border-color: #bdc9d6; }
QPushButton[danger="true"] { background: #dc2626; border-color: #dc2626; }
QPushButton[danger="true"]:hover { background: #b91c1c; }
QPushButton[success="true"] { background: #087f5b; border-color: #087f5b; }
QPushButton[success="true"]:hover { background: #066c4d; }
QPushButton[secondary="true"] { color: #344054; background: #ffffff; border-color: #9aaabd; }
QPushButton[secondary="true"]:hover { color: #172033; background: #e8eef6; border-color: #8293a8; }
QPushButton#robotEnable:checked { color: #ffffff; background: #b54708; border-color: #b54708; }
QPushButton#robotEnable:checked:hover { background: #93370d; border-color: #93370d; }
QPushButton[danger="true"]:disabled, QPushButton[success="true"]:disabled,
QPushButton[secondary="true"]:disabled, QPushButton#robotEnable:checked:disabled {
    color: #5d6b7d; background: #e1e7ee; border-color: #bdc9d6;
}
QCheckBox { color: #26354d; spacing: 7px; }
QCheckBox:disabled { color: #5d6b7d; }
QTableWidget { color: #1f2d42; background: #ffffff; alternate-background-color: #f1f5f9;
               border: 1px solid #c5d0dc; border-radius: 6px; gridline-color: #dbe3ec;
               selection-background-color: #dbeafe; selection-color: #1e3a5f; }
QTableWidget:disabled { color: #526173; background: #e8edf3; }
QHeaderView::section { background: #e5ecf4; color: #344258; padding: 6px; border: none;
                       border-right: 1px solid #cbd5e1; border-bottom: 1px solid #cbd5e1; font-weight: 600; }
QTextEdit { background: #0f172a; color: #dbeafe; border: 1px solid #1e293b; border-radius: 6px;
            font-family: "Cascadia Mono", Consolas; font-size: 11px; }
QProgressBar { min-height: 20px; color: #172033; border: 1px solid #aebdce; border-radius: 5px;
               text-align: center; background: #ffffff; font-weight: 600; }
QProgressBar::chunk { background: #5eead4; border-radius: 4px; }
QTabWidget::pane { background: #ffffff; border: 1px solid #b8c5d3; border-radius: 5px; top: -1px; }
QTabBar::tab { color: #344258; background: #e2e8f0; border: 1px solid #b8c5d3;
               padding: 7px 14px; margin-right: 2px; }
QTabBar::tab:selected { color: #ffffff; background: #315f9b; border-color: #315f9b; font-weight: 600; }
QTabBar::tab:hover:!selected { color: #172033; background: #cbd5e1; }
QSplitter::handle { background: #cbd5e1; }
QStatusBar { color: #26354d; background: #e5ebf2; }
QToolTip { color: #ffffff; background: #172033; border: 1px solid #334155; padding: 4px; }
QMessageBox { background: #f8fafc; }
QMessageBox QLabel { color: #111827; background: transparent; font-size: 13px; }
QMessageBox QPushButton { min-width: 72px; color: #ffffff; background: #2563eb;
                         border: 1px solid #1d4ed8; padding: 5px 14px; }
QMessageBox QPushButton:hover { background: #1d4ed8; }
QMessageBox QPushButton:pressed { background: #1e40af; }
QLabel[badge="true"] { border-radius: 11px; padding: 3px 9px; background: #fff1f2; color: #b42318; font-weight: 650; }
QLabel[badgeState="ok"] { background: #ecfdf3; color: #027a48; }
QLabel[badgeState="warn"] { background: #fff7ed; color: #b45309; }
QLabel#headline { color: #172b4d; font-size: 22px; font-weight: 750; }
QLabel#subheadline { color: #526173; font-size: 12px; }
QLabel#instruction { background: #e8f2ff; color: #183b75; border: 1px solid #93c5fd;
                     border-radius: 7px; padding: 9px; font-size: 13px; font-weight: 600; }
"""


class QuickCalWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("SDB QuickCal V1｜11 路 IMU 机械臂标定站")
        self.resize(1540, 940)
        self.setMinimumSize(1250, 780)
        self.glove = GloveDevice(self)
        self.robot = RobotDevice(self)
        self.coordinator = QuickCalCoordinator(self.glove, self.robot, self)
        self.output_directory = DEFAULT_OUTPUT_DIR
        self.taught_poses = self._load_taught_poses()
        self.robot_actions = self._load_robot_actions()
        self.tool_offset_config = self._load_tool_offset_config()
        self._building_limits = False
        self._building_action_config = False
        self._ui_raw_imu_frame = None
        self._ui_raw_imu_ns = 0
        self._ui_register_imu_frame = None
        self._ui_register_imu_ns = 0
        self._robot_enable_pending: bool | None = None
        self._robot_enable_timeout = QTimer(self)
        self._robot_enable_timeout.setSingleShot(True)
        self._robot_enable_timeout.setInterval(3000)
        self._robot_enable_timeout.timeout.connect(self._on_robot_enable_timeout)
        self._auto_action_step: str | None = None
        self._auto_action_started_ns = 0
        self._auto_action_deadline_ns = 0
        self._auto_action_stable_since_ns = 0
        self._auto_action_seen_motion = False
        self._full_auto_enabled = False
        self._full_auto_timer = QTimer(self)
        self._full_auto_timer.setInterval(250)
        self._full_auto_timer.timeout.connect(self._try_start_full_auto_step)
        self._full_auto_neutral_return_started_ns = 0
        self._full_auto_neutral_return_seen_motion = False
        self._gyro_x_phase = ""
        self._gyro_x_phase_started_ns = 0
        self._gyro_x_reference_pose: tuple[float, ...] | None = None
        self._gyro_x_original_speed_factor = 0
        self._gyro_x_jog_speed_factor = 0
        self._gyro_x_decel_level = -1
        self._gyro_x_jog_active = False
        self._mag_phase = ""
        self._mag_phase_started_ns = 0
        self._mag_m04_motion_since_ns = 0
        self._mag_segment_index = -1
        self._mag_pending_segment_index = -1
        self._mag_reference_pose: tuple[float, ...] | None = None
        self._mag_reference_joints: tuple[float, ...] | None = None
        self._mag_original_speed_factor = 0
        self._mag_speed_factor = 0
        self._mag_speed_source = ""
        self._mag_jog_active = False
        self._manual_absolute_pose_initialized = False
        self._manual_absolute_pose_frame: tuple[int, int] | None = None
        self._manual_motion_target: tuple[float, ...] | None = None
        self._manual_motion_frame: tuple[int, int] | None = None
        self._manual_motion_kind = ""
        self._manual_motion_started_ns = 0
        self._manual_motion_stable_since_ns = 0
        self._manual_motion_seen_motion = False

        root = QWidget()
        root.setObjectName("root")
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(8)
        layout.addLayout(self._build_header())
        layout.addWidget(self._build_top_controls())
        layout.addWidget(self._build_main_tabs(), 1)
        layout.addWidget(self._build_action_bar())
        self.setStyleSheet(STYLE)

        self._connect_signals()
        self._populate_workflow()
        self._populate_imu_table()
        self._refresh_ports()
        self._refresh_limits()
        self._update_action_controls()

        self.health_timer = QTimer(self)
        self.health_timer.setInterval(300)
        self.health_timer.timeout.connect(self._refresh_health)
        self.health_timer.start()

    def _build_header(self):
        layout = QHBoxLayout()
        titles = QVBoxLayout()
        title = QLabel("11 路 IMU 机械臂工厂标定")
        title.setObjectName("headline")
        subtitle = QLabel("QuickCal V1｜一次启动、逐步门控、全流程自动执行")
        subtitle.setObjectName("subheadline")
        titles.addWidget(title)
        titles.addWidget(subtitle)
        layout.addLayout(titles)
        layout.addStretch()
        self.robot_badge = self._badge("机械臂未连接")
        self.glove_badge = self._badge("手套未连接")
        self.imu_badge = self._badge("IMU 0/11")
        self.session_badge = self._badge("会话空闲", "warn")
        for badge in (self.robot_badge, self.glove_badge, self.imu_badge, self.session_badge):
            layout.addWidget(badge)
        return layout

    @staticmethod
    def _badge(text: str, state: str = "bad") -> QLabel:
        label = QLabel(text)
        label.setProperty("badge", True)
        label.setProperty("badgeState", state)
        return label

    @staticmethod
    def _set_badge(label: QLabel, text: str, state: str) -> None:
        label.setText(text)
        label.setProperty("badgeState", state)
        label.style().unpolish(label)
        label.style().polish(label)

    def _build_top_controls(self) -> QWidget:
        holder = QWidget()
        layout = QHBoxLayout(holder)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        robot_group = QGroupBox("Dobot Nova 5")
        robot_grid = QGridLayout(robot_group)
        self.robot_ip = QLineEdit("192.168.5.1")
        self.robot_connect = QPushButton("连接")
        self.robot_enable = QPushButton("使能机械臂")
        self.robot_enable.setObjectName("robotEnable")
        self.robot_enable.setCheckable(True)
        self.robot_clear = QPushButton("清报警")
        self.robot_stop = QPushButton("立即停止")
        self.robot_stop.setProperty("danger", True)
        self.robot_teach_safe = QPushButton("示教安全位")
        self.robot_teach_safe.setProperty("secondary", True)
        self.robot_safe = QPushButton("回安全位")
        self.robot_safe.setProperty("secondary", True)
        self.robot_teach_neutral = QPushButton("示教标定中位")
        self.robot_teach_neutral.setProperty("secondary", True)
        self.robot_neutral = QPushButton("到标定中位")
        self.robot_neutral.setProperty("secondary", True)
        self.robot_state_label = QLabel("mode=--｜TCP=--")
        robot_grid.addWidget(QLabel("IP"), 0, 0)
        robot_grid.addWidget(self.robot_ip, 0, 1, 1, 2)
        robot_grid.addWidget(self.robot_connect, 0, 3)
        robot_grid.addWidget(self.robot_enable, 1, 0)
        robot_grid.addWidget(self.robot_clear, 1, 1)
        robot_grid.addWidget(self.robot_safe, 1, 2)
        robot_grid.addWidget(self.robot_stop, 1, 3)
        robot_grid.addWidget(self.robot_teach_safe, 2, 0)
        robot_grid.addWidget(self.robot_teach_neutral, 2, 1)
        robot_grid.addWidget(self.robot_neutral, 2, 2, 1, 2)
        robot_grid.addWidget(self.robot_state_label, 3, 0, 1, 4)
        self._refresh_taught_pose_label()
        layout.addWidget(robot_group, 5)

        glove_group = QGroupBox("11 路 IMU 手套")
        glove_grid = QGridLayout(glove_group)
        self.port_combo = QComboBox()
        self.port_refresh = QPushButton("刷新")
        self.port_refresh.setProperty("secondary", True)
        self.glove_connect = QPushButton("连接")
        self.glove_query = QPushButton("查询版本")
        self.glove_version_label = QLabel("固件：--")
        glove_grid.addWidget(QLabel("USB CDC"), 0, 0)
        glove_grid.addWidget(self.port_combo, 0, 1, 1, 2)
        glove_grid.addWidget(self.port_refresh, 0, 3)
        glove_grid.addWidget(self.glove_connect, 1, 0, 1, 2)
        glove_grid.addWidget(self.glove_query, 1, 2, 1, 2)
        glove_grid.addWidget(self.glove_version_label, 2, 0, 1, 4)
        layout.addWidget(glove_group, 4)

        trace_group = QGroupBox("追溯信息")
        trace_grid = QGridLayout(trace_group)
        self.sn_edit = QLineEdit()
        self.station_edit = QLineEdit("QC-01")
        self.operator_edit = QLineEdit()
        self.output_edit = QLineEdit(str(self.output_directory))
        self.output_choose = QPushButton("选择")
        self.output_choose.setProperty("secondary", True)
        self.environment_check = QCheckBox("磁环境、夹具、线束已确认")
        trace_grid.addWidget(QLabel("产品 SN"), 0, 0)
        trace_grid.addWidget(self.sn_edit, 0, 1)
        trace_grid.addWidget(QLabel("工位"), 0, 2)
        trace_grid.addWidget(self.station_edit, 0, 3)
        trace_grid.addWidget(QLabel("操作员"), 1, 0)
        trace_grid.addWidget(self.operator_edit, 1, 1)
        trace_grid.addWidget(self.output_edit, 1, 2)
        trace_grid.addWidget(self.output_choose, 1, 3)
        trace_grid.addWidget(self.environment_check, 2, 0, 1, 4)
        layout.addWidget(trace_group, 5)
        return holder

    def _build_main_tabs(self) -> QTabWidget:
        tabs = QTabWidget()
        tabs.addTab(self._build_station_page(), "标定工站")
        tabs.addTab(self._build_limits_page(), "G01/G02 固定运动参数")
        tabs.addTab(self._build_report_page(), "最终质量报告")
        tabs.addTab(self._build_actions_page(), "机械臂动作配置")
        tabs.addTab(self._build_tool_motion_page(), "Tool 与手动运动")
        return tabs

    def _build_station_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 6, 0, 0)
        splitter = QSplitter(Qt.Orientation.Horizontal)

        self.workflow_table = QTableWidget(0, 7)
        self.workflow_table.setHorizontalHeaderLabels(("步骤", "阶段", "动作", "采集", "阶段码", "状态", "说明"))
        self.workflow_table.setAlternatingRowColors(True)
        self.workflow_table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.workflow_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.workflow_table.verticalHeader().setVisible(False)
        header = self.workflow_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(4, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(5, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(6, QHeaderView.ResizeMode.Stretch)
        splitter.addWidget(self.workflow_table)

        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(4, 0, 0, 0)
        health_group = QGroupBox("实时 11 路 IMU 健康")
        health_layout = QVBoxLayout(health_group)
        self.imu_table = QTableWidget(11, 7)
        self.imu_table.setHorizontalHeaderLabels(("IMU", "在线", "|ω|", "|a|", "Gyro LSB", "Acc LSB", "最终质量"))
        self.imu_table.setAlternatingRowColors(True)
        self.imu_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.imu_table.verticalHeader().setVisible(False)
        self.imu_table.verticalHeader().setDefaultSectionSize(28)
        self.imu_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        health_layout.addWidget(self.imu_table)
        right_layout.addWidget(health_group, 1)

        feedback_group = QGroupBox("设备数据状态与日志")
        feedback_layout = QVBoxLayout(feedback_group)
        self.health_label = QLabel("等待 Dobot 机械臂与 IMU 手套连接")
        self.health_label.setWordWrap(False)
        self.health_label.setToolTip(
            "Dobot 机械臂反馈来自机械臂实时反馈端口；"
            "type=5/8/9/11 均来自 IMU 手套串口。"
        )
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.document().setMaximumBlockCount(1200)
        feedback_layout.addWidget(self.health_label)
        feedback_layout.addWidget(self.log_text, 1)
        right_layout.addWidget(feedback_group, 1)
        splitter.addWidget(right)
        splitter.setSizes((860, 600))
        layout.addWidget(splitter)
        return page

    def _build_limits_page(self) -> QWidget:
        page = QWidget()
        layout = QHBoxLayout(page)
        input_group = QGroupBox("G01/G02 当前配置轴的固定运动参数")
        form = QFormLayout(input_group)
        self.yaw_negative = QLineEdit(f"{-LIMITED_GYRO_OUTER_DEG:g}")
        self.yaw_positive = QLineEdit(f"{LIMITED_GYRO_OUTER_DEG:g}")
        self.yaw_margin = QLineEdit(f"{LIMITED_GYRO_ACCEL_DECEL_DEG:g}")
        self.yaw_rate = QLineEdit(f"{ROLL_PITCH_GYRO_RATE_DEG_S:g}")
        self.yaw_min_capture = QLineEdit(f"{LIMITED_GYRO_CAPTURE_S:g}")
        for label, edit in (
            ("完整运动负端点（°）", self.yaw_negative),
            ("完整运动正端点（°）", self.yaw_positive),
            ("单端加/减速区（°）", self.yaw_margin),
            ("匀速角速度（°/s）", self.yaw_rate),
            ("固定有效采集（s）", self.yaw_min_capture),
        ):
            form.addRow(label, edit)
            edit.setReadOnly(True)
        layout.addWidget(input_group)
        result_group = QGroupBox("自动计算与强制联锁")
        result_layout = QVBoxLayout(result_group)
        self.limits_result = QLabel()
        self.limits_result.setWordWrap(True)
        self.limits_result.setFont(QFont("Microsoft YaHei UI", 13))
        self.timeline_summary = QLabel()
        self.timeline_summary.setWordWrap(True)
        warning = QLabel(
            "G01/G02 必须从实测夹具 X 对应的 Tool Rx 0° 中位进入，完整行程固定为 -55°～+55°；"
            "只有 -45°～+45° 的 6 秒匀速段进入拟合，加减速和回中位不采集。"
        )
        warning.setWordWrap(True)
        warning.setStyleSheet("color:#b42318;background:#fff1f2;border:1px solid #fecdd3;border-radius:6px;padding:9px;")
        result_layout.addWidget(self.limits_result)
        result_layout.addWidget(self.timeline_summary)
        result_layout.addWidget(warning)
        result_layout.addStretch()
        layout.addWidget(result_group, 2)
        return page

    def _build_report_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        self.report_summary = QLabel("尚未收到 type=7 最终报告")
        self.report_summary.setWordWrap(True)
        self.report_table = QTableWidget(len(IMU_NAMES) + len(MAG_REPORT_SENSOR_NAMES), 13)
        self.report_table.setHorizontalHeaderLabels(
            (
                "传感器",
                "Gyro",
                "Gyro拒绝原因",
                "RMS（°）",
                "窗口数",
                "最大非对角",
                "Accel",
                "Accel诊断",
                "Mag样本数",
                "Mag Span X/Y/Z",
                "Mag Offset X/Y/Z",
                "Mag Scale X/Y/Z",
                "Mag质量",
            )
        )
        self.report_table.verticalHeader().setVisible(False)
        self.report_table.verticalHeader().setDefaultSectionSize(22)
        self.report_table.setAlternatingRowColors(True)
        self.report_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        report_header = self.report_table.horizontalHeader()
        report_header.setSectionResizeMode(QHeaderView.ResizeMode.Interactive)
        report_header.setMinimumSectionSize(60)
        report_header.setStretchLastSection(False)
        for column, width in enumerate(
            (145, 80, 220, 90, 80, 105, 80, 260, 90, 155, 170, 170, 240)
        ):
            self.report_table.setColumnWidth(column, width)
        for row, name in enumerate(IMU_NAMES):
            self.report_table.setItem(row, 0, QTableWidgetItem(name))
            for column in range(1, 13):
                self.report_table.setItem(
                    row,
                    column,
                    QTableWidgetItem("—" if column >= 8 else "--"),
                )
        for slot, name in enumerate(MAG_REPORT_SENSOR_NAMES):
            row = len(IMU_NAMES) + slot
            self.report_table.setItem(row, 0, QTableWidgetItem(name))
            for column in range(1, 13):
                self.report_table.setItem(
                    row,
                    column,
                    QTableWidgetItem("—" if column < 8 else "--"),
                )
        layout.addWidget(self.report_summary)
        layout.addWidget(self.report_table, 1)
        return page

    def _build_actions_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(10)

        config_group = QGroupBox("各标定阶段机械臂旋转参数")
        config_layout = QVBoxLayout(config_group)
        config_layout.setContentsMargins(14, 16, 14, 14)
        config_layout.setSpacing(12)
        first_step = next(iter(self.robot_actions))
        config = self.robot_actions[first_step]

        header_row = QHBoxLayout()
        header_row.setSpacing(10)
        header_row.addWidget(QLabel("配置步骤"))
        self.action_step_combo = QComboBox()
        self.action_step_combo.addItems(tuple(self.robot_actions))
        self.action_step_combo.setMinimumWidth(180)
        header_row.addWidget(self.action_step_combo, 1)
        self.action_auto_enabled = QCheckBox("启用本步骤自动旋转与到位采集")
        self.action_auto_enabled.setChecked(config.enabled)
        header_row.addWidget(self.action_auto_enabled)
        self.save_action_config_button = QPushButton(f"保存 {first_step} 配置")
        self.save_action_config_button.setProperty("secondary", True)
        self.save_action_config_button.setMinimumWidth(150)
        header_row.addWidget(self.save_action_config_button)
        config_layout.addLayout(header_row)

        parameter_grid = QGridLayout()
        parameter_grid.setHorizontalSpacing(12)
        parameter_grid.setVerticalSpacing(6)
        self.action_rotation_axis = QComboBox()
        self.action_rotation_axis.addItems(("Rx", "Ry", "Rz"))
        self.action_rotation_axis.setCurrentText(config.axis)
        self.action_rotation_degrees = QDoubleSpinBox()
        self.action_rotation_degrees.setRange(-180.0, 180.0)
        self.action_rotation_degrees.setDecimals(2)
        self.action_rotation_degrees.setSingleStep(0.1)
        self.action_rotation_degrees.setValue(config.degrees)
        self.action_rotation_degrees.setSuffix(" °")
        self.action_velocity = QSpinBox()
        self.action_velocity.setRange(1, 100)
        self.action_velocity.setValue(config.velocity_percent)
        self.action_velocity.setSuffix(" %")
        self.action_timeout = QDoubleSpinBox()
        self.action_timeout.setRange(5.0, 180.0)
        self.action_timeout.setDecimals(1)
        self.action_timeout.setValue(config.timeout_s)
        self.action_timeout.setSuffix(" s")
        parameter_widgets = (
            ("Tool 旋转轴", self.action_rotation_axis),
            ("相对角度 / 有效扫转角", self.action_rotation_degrees),
            ("控制器速度比例", self.action_velocity),
            ("动作超时", self.action_timeout),
        )
        for column, (label, widget) in enumerate(parameter_widgets):
            parameter_grid.addWidget(QLabel(label), 0, column)
            parameter_grid.addWidget(widget, 1, column)
            parameter_grid.setColumnStretch(column, 1)
        config_layout.addLayout(parameter_grid)

        prediction_title = QLabel("实时姿态与动作预测")
        prediction_title.setStyleSheet("font-weight:600;color:#344054;margin-top:3px;")
        config_layout.addWidget(prediction_title)
        self.action_prediction = QLabel()
        self.action_prediction.setWordWrap(True)
        self.action_prediction.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        self.action_prediction.setMinimumHeight(90)
        self.action_prediction.setMaximumHeight(130)
        self.action_prediction.setStyleSheet(
            "background:#e8f2ff;border:1px solid #93c5fd;border-radius:6px;padding:12px;"
        )
        config_layout.addWidget(self.action_prediction)
        layout.addWidget(config_group)
        layout.addStretch(1)
        self._refresh_action_preview()
        return page

    def _build_tool_motion_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(12)

        tool_group = QGroupBox("Tool 坐标系配置")
        tool_layout = QVBoxLayout(tool_group)
        tool_layout.setContentsMargins(14, 16, 14, 14)
        tool_layout.setSpacing(10)
        tool_grid = QGridLayout()
        tool_grid.setHorizontalSpacing(10)
        tool_grid.setVerticalSpacing(6)
        self.manual_tool_index = QSpinBox()
        self.manual_tool_index.setRange(1, 9)
        self.manual_tool_index.setValue(int(self.tool_offset_config["tool"]))
        tool_grid.addWidget(QLabel("Tool 编号"), 0, 0)
        tool_grid.addWidget(self.manual_tool_index, 1, 0)
        self.manual_tool_offset_spins: dict[str, QDoubleSpinBox] = {}
        saved_offset = self.tool_offset_config["offset"]
        for column, name in enumerate(("X", "Y", "Z", "Rx", "Ry", "Rz"), 1):
            spin = QDoubleSpinBox()
            spin.setDecimals(3)
            if column <= 3:
                spin.setRange(-1000.0, 1000.0)
            else:
                spin.setRange(-180.0, 180.0)
            spin.setSingleStep(1.0)
            spin.setSuffix(" mm" if column <= 3 else " °")
            spin.setValue(float(saved_offset[column - 1]))
            self.manual_tool_offset_spins[name] = spin
            tool_grid.addWidget(QLabel(name), 0, column)
            tool_grid.addWidget(spin, 1, column)
            tool_grid.setColumnStretch(column, 1)
        tool_layout.addLayout(tool_grid)

        tool_actions = QHBoxLayout()
        self.tool_config_status = QLabel(
            f"本地配置：Tool {self.tool_offset_config['tool']}；"
            f"当前反馈 Tool {self.robot.latest_state.tool if self.robot.latest_state else '--'}"
        )
        self.tool_config_status.setWordWrap(True)
        tool_actions.addWidget(self.tool_config_status, 1)
        self.activate_tool_zero_button = QPushButton("启用 Tool 0（法兰）")
        self.activate_tool_zero_button.setProperty("secondary", True)
        self.apply_tool_config_button = QPushButton("保存并启用 Tool")
        tool_actions.addWidget(self.activate_tool_zero_button)
        tool_actions.addWidget(self.apply_tool_config_button)
        tool_layout.addLayout(tool_actions)
        layout.addWidget(tool_group)

        motion_group = QGroupBox("TCP 绝对目标")
        motion_group.setMinimumHeight(160)
        motion_layout = QVBoxLayout(motion_group)
        motion_layout.setContentsMargins(14, 14, 14, 12)
        motion_layout.setSpacing(7)

        pose_grid = QGridLayout()
        pose_grid.setHorizontalSpacing(12)
        pose_grid.setVerticalSpacing(8)
        self.manual_position_spins: dict[str, QDoubleSpinBox] = {}
        for column, name in enumerate(("X", "Y", "Z")):
            spin = QDoubleSpinBox()
            spin.setDecimals(2)
            spin.setRange(-2000.0, 2000.0)
            spin.setSingleStep(1.0)
            spin.setPrefix(f"{name}  ")
            spin.setSuffix(" mm")
            spin.setMinimumWidth(120)
            spin.setMinimumHeight(30)
            self.manual_position_spins[name] = spin
            pose_grid.addWidget(spin, 0, column)
            pose_grid.setColumnStretch(column, 1)
        self.manual_rotation_spins: dict[str, QDoubleSpinBox] = {}
        for column, name in enumerate(("Rx", "Ry", "Rz"), 3):
            spin = QDoubleSpinBox()
            spin.setDecimals(2)
            spin.setRange(-180.0, 180.0)
            spin.setSingleStep(1.0)
            spin.setPrefix(f"{name}  ")
            spin.setSuffix(" °")
            spin.setMinimumWidth(120)
            spin.setMinimumHeight(30)
            self.manual_rotation_spins[name] = spin
            pose_grid.addWidget(spin, 0, column)
            pose_grid.setColumnStretch(column, 1)
        self.execute_translation_button = QPushButton("仅移动位置")
        self.execute_rotation_button = QPushButton("仅旋转姿态")
        motion_layout.addLayout(pose_grid)

        motion_actions = QHBoxLayout()
        motion_actions.setSpacing(8)
        self.tool_motion_feedback = QLabel("等待反馈")
        self.tool_motion_feedback.setMinimumWidth(90)
        motion_actions.addWidget(self.tool_motion_feedback)
        motion_actions.addStretch(1)
        self.load_current_pose_button = QPushButton("读取当前位姿")
        self.load_current_pose_button.setProperty("secondary", True)
        motion_actions.addWidget(self.load_current_pose_button)
        motion_actions.addWidget(QLabel("速度"))
        self.manual_motion_velocity = QSpinBox()
        self.manual_motion_velocity.setRange(1, 100)
        self.manual_motion_velocity.setValue(20)
        self.manual_motion_velocity.setSuffix(" %")
        self.manual_motion_velocity.setMinimumWidth(90)
        motion_actions.addWidget(self.manual_motion_velocity)
        motion_actions.addWidget(self.execute_translation_button)
        motion_actions.addWidget(self.execute_rotation_button)
        motion_layout.addLayout(motion_actions)

        self.manual_motion_result = QLabel(
            "绝对目标：位置按钮保留当前姿态，姿态按钮保留当前位置；Tool 补偿生效。"
        )
        self.manual_motion_result.setWordWrap(False)
        self.manual_motion_result.setStyleSheet(
            "background:#f8fafc;border:1px solid #cbd5e1;border-radius:6px;padding:7px;color:#475467;"
        )
        motion_layout.addWidget(self.manual_motion_result)
        layout.addWidget(motion_group)
        layout.addStretch(1)
        return page

    def _build_action_bar(self) -> QWidget:
        group = QGroupBox("当前动作与采集控制")
        layout = QVBoxLayout(group)
        self.instruction_label = QLabel("连接机械臂和手套后开始工厂会话。")
        self.instruction_label.setObjectName("instruction")
        self.instruction_label.setWordWrap(True)
        layout.addWidget(self.instruction_label)
        row = QHBoxLayout()
        self.capture_progress = QProgressBar()
        self.capture_progress.setRange(0, 100)
        self.capture_progress.setFormat("等待开始")
        self.start_button = QPushButton("开始全自动校准")
        self.start_button.setProperty("success", True)
        self.confirm_button = QPushButton("动作条件已满足，开始采集")
        self.abort_button = QPushButton("停止并放弃")
        self.abort_button.setProperty("danger", True)
        row.addWidget(self.capture_progress, 1)
        row.addWidget(self.start_button)
        row.addWidget(self.confirm_button)
        row.addWidget(self.abort_button)
        layout.addLayout(row)
        return group

    def _connect_signals(self) -> None:
        self.robot_connect.clicked.connect(self._toggle_robot)
        self.robot_enable.clicked.connect(self._toggle_robot_enable)
        self.robot_clear.clicked.connect(self.robot.clear_error)
        self.robot_stop.clicked.connect(self.robot.stop)
        self.robot_safe.clicked.connect(self._return_safe_pose)
        self.robot_teach_safe.clicked.connect(lambda: self._teach_pose("safe"))
        self.robot_teach_neutral.clicked.connect(lambda: self._teach_pose("neutral"))
        self.robot_neutral.clicked.connect(lambda: self._return_taught_pose("neutral"))
        self.port_refresh.clicked.connect(self._refresh_ports)
        self.glove_connect.clicked.connect(self._toggle_glove)
        self.glove_query.clicked.connect(self.glove.query_version)
        self.output_choose.clicked.connect(self._choose_output)
        self.start_button.clicked.connect(self._start_session)
        self.confirm_button.clicked.connect(self._confirm_or_execute_current_action)
        self.abort_button.clicked.connect(lambda: self._confirm_abort("操作员中止"))
        self.action_step_combo.currentTextChanged.connect(self._load_selected_action_config)
        self.save_action_config_button.clicked.connect(self._save_selected_action_config)
        self.action_auto_enabled.toggled.connect(self._refresh_action_preview)
        self.action_rotation_axis.currentTextChanged.connect(self._refresh_action_preview)
        self.action_rotation_degrees.valueChanged.connect(self._refresh_action_preview)
        self.action_velocity.valueChanged.connect(self._refresh_action_preview)
        self.action_timeout.valueChanged.connect(self._refresh_action_preview)
        self.apply_tool_config_button.clicked.connect(self._apply_tool_config)
        self.activate_tool_zero_button.clicked.connect(self._activate_tool_zero)
        self.load_current_pose_button.clicked.connect(
            self._load_current_pose_into_manual_controls
        )
        self.execute_translation_button.clicked.connect(
            lambda: self._execute_manual_absolute_motion("position")
        )
        self.execute_rotation_button.clicked.connect(
            lambda: self._execute_manual_absolute_motion("orientation")
        )

        self.robot.connection_changed.connect(self._robot_connection_changed)
        self.glove.connection_changed.connect(self._glove_connection_changed)
        self.robot.state_received.connect(self._on_robot_state)
        self.glove.raw_imu_received.connect(self._on_raw_imu)
        self.glove.register_raw_imu_received.connect(self._on_register_imu)
        self.glove.version_received.connect(self._on_version)
        self.glove.mcal_report_received.connect(self._on_report)
        self.robot.log_message.connect(self._append_log)
        self.glove.log_message.connect(self._append_log)
        self.robot.error_occurred.connect(lambda message: self._show_error(message, False))
        self.glove.error_occurred.connect(lambda message: self._show_error(message, False))

        self.coordinator.state_changed.connect(self._on_run_state)
        self.coordinator.current_step_changed.connect(self._on_current_step)
        self.coordinator.step_status_changed.connect(self._on_step_status)
        self.coordinator.progress_changed.connect(self._on_progress)
        self.coordinator.status_message.connect(self._on_status_message)
        self.coordinator.finished.connect(self._on_finished)

    def _populate_workflow(self) -> None:
        limits = self._limits_from_ui(silent=True) or YawLimits()
        steps = steps_for_limits(limits)
        existing_status = getattr(self.coordinator, "step_status", ["未开始"] * len(steps))
        self.workflow_table.setRowCount(len(steps))
        for row, step in enumerate(steps):
            values = (
                step.step_id,
                step.group,
                step.name,
                f"{step.capture_s:.2f} s" if step.capture_s else "—",
                f"0x{step.stage_code:02X}" if step.stage_code is not None else "—",
                existing_status[row] if row < len(existing_status) else "未开始",
                step.start_condition,
            )
            for column, value in enumerate(values):
                self.workflow_table.setItem(row, column, QTableWidgetItem(value))
            self.workflow_table.setRowHeight(row, 27)

    def _populate_imu_table(self) -> None:
        for row, name in enumerate(IMU_NAMES):
            values = (name, "未连接", "--", "--", "--", "--", "等待")
            for column, value in enumerate(values):
                self.imu_table.setItem(row, column, QTableWidgetItem(value))
            self.imu_table.setRowHeight(row, 26)

    @Slot()
    def _refresh_ports(self) -> None:
        selected = self.port_combo.currentData()
        self.port_combo.clear()
        for port in GloveDevice.available_ports():
            text = f"{port['name']}｜{port['description']}｜{port['vid_pid']}"
            self.port_combo.addItem(text, port["name"])
        if selected:
            index = self.port_combo.findData(selected)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)

    def _toggle_glove(self) -> None:
        if self.glove.is_open:
            self.glove.close()
        else:
            name = self.port_combo.currentData()
            if not name:
                self._show_error("没有可用的 USB CDC 串口", True)
                return
            if self.glove.open(name):
                QTimer.singleShot(200, self.glove.query_version)

    def _toggle_robot(self) -> None:
        if self.robot.connected:
            self.robot.disconnect_robot()
        else:
            self.robot.connect_robot(self.robot_ip.text().strip())

    @staticmethod
    def _robot_mode_is_enabled(mode: int) -> bool:
        return mode in (5, 7, 8, 10)

    def _toggle_robot_enable(self, _checked: bool = False) -> None:
        state = self.robot.latest_state
        if not self.robot.connected or state is None:
            self._robot_enable_pending = None
            self._update_robot_enable_button()
            self._show_error("尚未收到机械臂状态反馈，不能切换使能", False)
            return
        if state.mode not in (3, 4, 5):
            self._update_robot_enable_button()
            self._show_error(f"机械臂当前 mode={state.mode}，请停止运动或清除异常后再切换使能", True)
            return

        target_enabled = not self._robot_mode_is_enabled(state.mode)
        self._robot_enable_pending = target_enabled
        self._update_robot_enable_button()
        succeeded = self.robot.enable() if target_enabled else self.robot.disable()
        if not succeeded:
            self._robot_enable_pending = None
            self._update_robot_enable_button()
            return
        if self._robot_enable_pending is not None:
            self._robot_enable_timeout.start()

    def _on_robot_enable_timeout(self) -> None:
        if self._robot_enable_pending is None:
            return
        action = "使能" if self._robot_enable_pending else "关闭使能"
        self._robot_enable_pending = None
        self._update_robot_enable_button()
        self._show_error(f"{action}命令已发送，但 3 秒内未收到对应的 RobotMode 反馈", False)

    def _update_robot_enable_button(self) -> None:
        state = self.robot.latest_state
        if self._robot_enable_pending is not None:
            target_enabled = self._robot_enable_pending
            self.robot_enable.setChecked(target_enabled)
            self.robot_enable.setText("正在使能…" if target_enabled else "正在关闭使能…")
            return
        enabled = bool(state and self._robot_mode_is_enabled(state.mode))
        self.robot_enable.setChecked(enabled)
        self.robot_enable.setText("关闭使能" if enabled else "使能机械臂")

    def _load_taught_poses(self) -> dict[str, TaughtPose]:
        try:
            poses = load_pose_config(POSE_CONFIG_FILE)
        except Exception:
            poses = {}
        if "safe" not in poses:
            legacy_safe = load_legacy_safe_pose(RECORDED_POSE_FILE)
            if legacy_safe is not None:
                poses["safe"] = legacy_safe
        return poses

    def _load_robot_actions(self) -> dict[str, RobotActionConfig]:
        try:
            return load_action_config(ACTION_CONFIG_FILE)
        except Exception:
            return dict(DEFAULT_ACTIONS)

    def _load_tool_offset_config(self) -> dict[str, object]:
        default = {"tool": 1, "active_tool": 1, "offset": [0.0] * 6}
        try:
            data = json.loads(TOOL_CONFIG_FILE.read_text(encoding="utf-8"))
            tool = int(data.get("tool", 1))
            active_tool = int(data.get("active_tool", tool))
            offset = [float(value) for value in data.get("offset", ())]
            if not 1 <= tool <= 9 or not 0 <= active_tool <= 9:
                raise ValueError("Tool 编号越界")
            if len(offset) != 6 or not all(math.isfinite(value) for value in offset):
                raise ValueError("Tool 偏置无效")
            return {"tool": tool, "active_tool": active_tool, "offset": offset}
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            return default

    def _save_tool_offset_config(
        self, tool: int, offset: tuple[float, ...], active_tool: int
    ) -> None:
        data = {
            "version": 1,
            "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "tool": int(tool),
            "active_tool": int(active_tool),
            "offset": [float(value) for value in offset],
            "units": {
                "X": "mm",
                "Y": "mm",
                "Z": "mm",
                "Rx": "deg",
                "Ry": "deg",
                "Rz": "deg",
            },
        }
        temporary = TOOL_CONFIG_FILE.with_suffix(".json.tmp")
        temporary.write_text(
            json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        temporary.replace(TOOL_CONFIG_FILE)

    def _manual_robot_ready(self, operation: str) -> bool:
        if self.coordinator.running:
            self._show_error(f"标定会话运行期间禁止{operation}", True)
            return False
        state = self.robot.latest_state
        if (
            not self.robot.connected
            or state is None
            or time.monotonic_ns() - state.received_monotonic_ns >= 1_000_000_000
        ):
            self._show_error("机械臂未连接或实时反馈已超时", True)
            return False
        if (
            state.mode != 5
            or state.linear_speed_norm > 1.0
            or state.angular_speed_norm > 0.8
        ):
            self._show_error("机械臂必须已使能、处于空闲状态并完全停止", True)
            return False
        return True

    @Slot()
    def _apply_tool_config(self) -> None:
        if not self._manual_robot_ready("修改 Tool 坐标系"):
            return
        tool = self.manual_tool_index.value()
        offset = tuple(
            self.manual_tool_offset_spins[name].value()
            for name in ("X", "Y", "Z", "Rx", "Ry", "Rz")
        )
        text = ", ".join(f"{value:+.3f}" for value in offset)
        message = (
            f"将覆盖控制器 Tool {tool} 并立即启用。\n\n"
            f"X/Y/Z/Rx/Ry/Rz = ({text})\n\n"
            "修改 Tool 定义后，原示教位姿和六面方向的物理含义可能改变。"
            "请确认这些参数对应当前夹具，且机械臂已完全停止。"
        )
        if (
            QMessageBox.question(
                self,
                "确认保存 Tool 坐标系",
                message,
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No,
            )
            != QMessageBox.StandardButton.Yes
        ):
            return
        if not self.robot.set_and_activate_tool(tool, offset):
            return
        try:
            self._save_tool_offset_config(tool, offset, tool)
        except Exception as exc:
            self._show_error(f"Tool 已在控制器中启用，但本地保存失败：{exc}", True)
            return
        self.tool_offset_config = {
            "tool": tool,
            "active_tool": tool,
            "offset": list(offset),
        }
        self.tool_config_status.setText(
            f"Tool {tool} 已保存并启用；等待实时反馈确认"
        )
        self._append_log(f"Tool {tool} 已保存并启用：({text})", "good")

    @Slot()
    def _activate_tool_zero(self) -> None:
        if not self._manual_robot_ready("切换 Tool 坐标系"):
            return
        if (
            QMessageBox.question(
                self,
                "确认启用 Tool 0",
                "将启用法兰坐标系 Tool 0。后续相对运动方向和 TCP 位置会随之改变，确认继续？",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No,
            )
            != QMessageBox.StandardButton.Yes
        ):
            return
        if not self.robot.activate_tool(0):
            return
        offset = tuple(float(value) for value in self.tool_offset_config["offset"])
        try:
            self._save_tool_offset_config(
                int(self.tool_offset_config["tool"]), offset, 0
            )
        except Exception as exc:
            self._show_error(f"Tool 0 已启用，但本地状态保存失败：{exc}", True)
            return
        self.tool_offset_config["active_tool"] = 0
        self.tool_config_status.setText("Tool 0 已启用；等待实时反馈确认")
        self._append_log("已启用 Tool 0（法兰坐标系）", "good")

    def _load_current_pose_into_manual_controls(self) -> None:
        state = self.robot.latest_state
        if state is None:
            self._show_error("尚未收到机械臂实时位姿", False)
            return
        for name, value in zip(("X", "Y", "Z"), state.pose[:3]):
            self.manual_position_spins[name].setValue(float(value))
        for name, value in zip(("Rx", "Ry", "Rz"), state.pose[3:]):
            self.manual_rotation_spins[name].setValue(float(value))
        self._manual_absolute_pose_initialized = True
        self._manual_absolute_pose_frame = (int(state.user), int(state.tool))

    def _set_manual_motion_result(self, message: str, level: str = "idle") -> None:
        colors = {
            "idle": ("#f8fafc", "#cbd5e1", "#475467"),
            "info": ("#eff6ff", "#93c5fd", "#1d4ed8"),
            "good": ("#ecfdf3", "#86efac", "#027a48"),
            "error": ("#fff1f2", "#fda4af", "#b42318"),
        }
        background, border, foreground = colors[level]
        self.manual_motion_result.setText(message)
        self.manual_motion_result.setStyleSheet(
            f"background:{background};border:1px solid {border};border-radius:6px;"
            f"padding:7px;color:{foreground};font-weight:600;"
        )

    @classmethod
    def _manual_target_errors(
        cls, target_pose, actual_pose
    ) -> tuple[float, float]:
        position_error = max(
            abs(float(target) - float(actual))
            for target, actual in zip(target_pose[:3], actual_pose[:3])
        )
        target_axes = QuickCalCoordinator._tool_axes(target_pose)
        actual_axes = QuickCalCoordinator._tool_axes(actual_pose)
        trace = sum(
            sum(a * b for a, b in zip(target_axis, actual_axis))
            for target_axis, actual_axis in zip(target_axes, actual_axes)
        )
        cosine = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
        angle_error = math.degrees(math.acos(cosine))
        return position_error, angle_error

    def _finish_manual_motion(self, message: str, level: str) -> None:
        self._manual_motion_target = None
        self._manual_motion_frame = None
        self._manual_motion_kind = ""
        self._manual_motion_started_ns = 0
        self._manual_motion_stable_since_ns = 0
        self._manual_motion_seen_motion = False
        self._set_manual_motion_result(message, level)
        self._append_log(message, level)
        self._update_action_controls()

    def _update_manual_motion_result(self, state) -> None:
        target = self._manual_motion_target
        if target is None:
            return
        now = time.monotonic_ns()
        position_error, angle_error = self._manual_target_errors(target, state.pose)
        error_detail = (
            f"位置误差 {position_error:.2f} mm，角度误差 {angle_error:.2f}°"
        )
        if (int(state.user), int(state.tool)) != self._manual_motion_frame:
            self._finish_manual_motion(
                "手动运动失败：运动期间 User/Tool 发生变化", "error"
            )
            return
        if state.mode == 9:
            self._finish_manual_motion("手动运动失败：机械臂进入报警状态", "error")
            return
        if state.mode == 11:
            self._finish_manual_motion("手动运动失败：机械臂触发碰撞检测", "error")
            return
        if not self._robot_mode_is_enabled(state.mode):
            self._finish_manual_motion(
                f"手动运动失败：机械臂已退出使能状态（mode={state.mode}）",
                "error",
            )
            return
        if now - self._manual_motion_started_ns > MANUAL_MOTION_TIMEOUT_NS:
            self._finish_manual_motion(
                f"手动运动超时：{error_detail}", "error"
            )
            return
        moving = bool(
            state.mode in (7, 8)
            or state.linear_speed_norm > 1.0
            or state.angular_speed_norm > 0.8
        )
        if moving:
            self._manual_motion_seen_motion = True
            self._manual_motion_stable_since_ns = 0
            self._set_manual_motion_result(
                f"{self._manual_motion_kind}执行中｜{error_detail}", "info"
            )
            return
        if state.mode != 5:
            self._manual_motion_stable_since_ns = 0
            self._set_manual_motion_result(
                f"等待机械臂恢复空闲（mode={state.mode}）", "info"
            )
            return
        elapsed_ns = now - self._manual_motion_started_ns
        if (
            not self._manual_motion_seen_motion
            and elapsed_ns < MANUAL_MOTION_START_GRACE_NS
        ):
            self._set_manual_motion_result("命令已接受，等待机械臂开始运动", "info")
            return
        if self._manual_motion_stable_since_ns == 0:
            self._manual_motion_stable_since_ns = now
            self._set_manual_motion_result(
                f"机械臂已停止，正在确认到位｜{error_detail}", "info"
            )
            return
        if now - self._manual_motion_stable_since_ns < MANUAL_MOTION_STABLE_NS:
            return
        if (
            position_error <= MANUAL_POSITION_TOLERANCE_MM
            and angle_error <= MANUAL_ANGLE_TOLERANCE_DEG
        ):
            self._finish_manual_motion(
                f"{self._manual_motion_kind}完成｜{error_detail}", "good"
            )
        else:
            self._finish_manual_motion(
                f"{self._manual_motion_kind}失败：机械臂已停止但未到达目标｜{error_detail}",
                "error",
            )

    def _check_manual_motion_watchdog(self, now: int) -> None:
        target = self._manual_motion_target
        if target is None:
            return
        state = self.robot.latest_state
        if (
            state is None
            or now - state.received_monotonic_ns > self.coordinator.ROBOT_FRESH_NS
        ):
            self._finish_manual_motion(
                "手动运动失败：机械臂实时反馈中断", "error"
            )
            return
        if now - self._manual_motion_started_ns > MANUAL_MOTION_TIMEOUT_NS:
            position_error, angle_error = self._manual_target_errors(
                target, state.pose
            )
            self._finish_manual_motion(
                "手动运动超时："
                f"位置误差 {position_error:.2f} mm，角度误差 {angle_error:.2f}°",
                "error",
            )

    def _execute_manual_absolute_motion(self, kind: str) -> None:
        if self._manual_motion_target is not None:
            self._show_error("上一条手动运动仍在等待结果", False)
            return
        if not self._manual_robot_ready("手动控制末端运动"):
            return
        if not self._manual_absolute_pose_initialized:
            self._show_error("请先点击“读取当前位姿”，再输入绝对目标", False)
            return
        state = self.robot.latest_state
        if kind == "position":
            target_pose = (
                *(self.manual_position_spins[name].value() for name in ("X", "Y", "Z")),
                *state.pose[3:],
            )
            changed = max(abs(target - actual) for target, actual in zip(target_pose[:3], state.pose[:3]))
        else:
            target_pose = (
                *state.pose[:3],
                *(self.manual_rotation_spins[name].value() for name in ("Rx", "Ry", "Rz")),
            )
            changed = max(
                self._angle_distance_deg(target, actual)
                for target, actual in zip(target_pose[3:], state.pose[3:])
            )
        if changed < 0.001:
            self._show_error("绝对目标与当前反馈相同，无需运动", False)
            return
        current_values = ", ".join(f"{value:+.2f}" for value in state.pose)
        target_values = ", ".join(f"{value:+.2f}" for value in target_pose)
        velocity = self.manual_motion_velocity.value()
        message = (
            f"当前 User {state.user} / Tool {state.tool}\n"
            f"当前 TCP (X, Y, Z, Rx, Ry, Rz) = ({current_values})\n"
            f"绝对目标 (X, Y, Z, Rx, Ry, Rz) = ({target_values})\n"
            f"速度比例 {velocity}%\n\n"
            "命令使用 MovL 绝对位姿。请确认整条直线路径无碰撞、线束无拉扯，"
            "人员已离开运动区域。"
        )
        if (
            QMessageBox.question(
                self,
                "确认末端绝对运动",
                message,
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No,
            )
            != QMessageBox.StandardButton.Yes
        ):
            return
        if self.robot.move_pose_l(
            target_pose, velocity, user=state.user, tool=state.tool
        ):
            self._manual_motion_target = tuple(float(value) for value in target_pose)
            self._manual_motion_frame = (int(state.user), int(state.tool))
            self._manual_motion_kind = "位置运动" if kind == "position" else "姿态运动"
            self._manual_motion_started_ns = time.monotonic_ns()
            self._manual_motion_stable_since_ns = 0
            self._manual_motion_seen_motion = False
            self._set_manual_motion_result("命令已接受，等待机械臂开始运动", "info")
            self._append_log(
                f"末端绝对运动已发送：User {state.user} / Tool {state.tool}，"
                f"target=({target_values})，v={velocity}%",
                "info",
            )
            self._update_action_controls()
        else:
            self._set_manual_motion_result("手动运动命令发送失败", "error")

    def _action_config_from_ui(self, silent: bool = False) -> RobotActionConfig | None:
        try:
            config = RobotActionConfig(
                enabled=self.action_auto_enabled.isChecked(),
                axis=self.action_rotation_axis.currentText(),
                degrees=self.action_rotation_degrees.value(),
                velocity_percent=self.action_velocity.value(),
                timeout_s=self.action_timeout.value(),
            )
            step_id = self.action_step_combo.currentText()
            if step_id in GYRO_AUTO_STEPS:
                expected_sweep = 90.0 if step_id in GYRO_LIMITED_STEPS else 150.0
                if not math.isclose(abs(config.degrees), expected_sweep, abs_tol=1e-6):
                    raise ValueError(
                        f"{step_id} 有效扫转角绝对值必须为 {expected_sweep:.0f}°；"
                        "只可通过正负号修改方向"
                    )
            return config
        except ValueError as exc:
            if not silent:
                self._show_error(str(exc), True)
            return None

    @Slot()
    def _save_selected_action_config(self) -> None:
        step_id = self.action_step_combo.currentText()
        config = self._action_config_from_ui()
        if config is None:
            return
        actions = dict(self.robot_actions)
        actions[step_id] = config
        try:
            save_action_config(ACTION_CONFIG_FILE, actions)
        except Exception as exc:
            self._show_error(f"保存 {step_id} 自动动作配置失败：{exc}", True)
            return
        self.robot_actions = actions
        self._refresh_action_preview()
        if step_id in GYRO_AUTO_STEPS:
            self._append_log(
                f"已保存 {step_id}：Tool {config.axis}，有效扫转 "
                f"{config.degrees:+.0f}°，固定 15°/s",
                "good",
            )
        else:
            self._append_log(
                f"已保存 {step_id}：Tool {config.axis} {config.degrees:+.2f}°，"
                f"速度 {config.velocity_percent}%",
                "good",
            )

    @Slot(str)
    def _load_selected_action_config(self, step_id: str) -> None:
        config = self.robot_actions.get(step_id)
        if config is None:
            return
        self._building_action_config = True
        try:
            self.action_auto_enabled.setChecked(config.enabled)
            self.action_rotation_axis.setCurrentText(config.axis)
            self.action_rotation_degrees.setValue(config.degrees)
            self.action_velocity.setValue(config.velocity_percent)
            self.action_timeout.setValue(config.timeout_s)
            self.save_action_config_button.setText(f"保存 {step_id} 配置")
            is_gyro = step_id in GYRO_AUTO_STEPS
            self.action_rotation_degrees.setToolTip(
                "G 阶段绝对值由 r024 固定，只能修改正负号来改变方向"
                if is_gyro
                else "相对于当前 Tool 坐标系的旋转角度"
            )
            fixed_hint = "G 阶段速度固定为 15°/s，由点动速度标定自动换算"
            self.action_velocity.setToolTip(fixed_hint if is_gyro else "控制器速度比例")
            self.action_timeout.setToolTip(
                "G 阶段使用自动扫转安全超时" if is_gyro else "相对旋转动作超时"
            )
            editable = self._auto_action_step is None
            self.action_velocity.setEnabled(editable and not is_gyro)
            self.action_timeout.setEnabled(editable and not is_gyro)
        finally:
            self._building_action_config = False
        self._refresh_action_preview()

    def _config_for_step(self, step_id: str) -> RobotActionConfig | None:
        if (
            hasattr(self, "action_step_combo")
            and self.action_step_combo.currentText() == step_id
        ):
            return self._action_config_from_ui(silent=True)
        return self.robot_actions.get(step_id)

    @Slot()
    def _refresh_action_preview(self, *_args) -> None:
        if self._building_action_config or not hasattr(self, "action_prediction"):
            return
        step_id = self.action_step_combo.currentText()
        config = self._action_config_from_ui(silent=True)
        if config is None:
            self.action_prediction.setText(f"{step_id} 配置无效")
            return
        if step_id in GYRO_AUTO_STEPS:
            direction = "+" if config.degrees > 0 else "-"
            outer = LIMITED_GYRO_OUTER_DEG if step_id in GYRO_LIMITED_STEPS else 85.0
            bound = LIMITED_GYRO_CAPTURE_BOUND_DEG if step_id in GYRO_LIMITED_STEPS else 75.0
            self.action_prediction.setText(
                f"{step_id} 协议阶段保持不变；机械臂映射为 Tool {config.axis}{direction}。\n"
                f"预置到 {-math.copysign(outer, config.degrees):+.1f}°，"
                f"仅在 {-math.copysign(bound, config.degrees):+.1f}° → "
                f"{math.copysign(bound, config.degrees):+.1f}° 以 "
                f"{math.copysign(ROLL_PITCH_GYRO_RATE_DEG_S, config.degrees):+.1f}°/s 采集。\n"
                "角度绝对值由 r024 锁定；修改正负号只改变扫转方向。"
            )
            return
        state = self.robot.latest_state
        if state is None:
            self.action_prediction.setText(
                f"等待机械臂反馈。当前配置：Tool {config.axis} {config.degrees:+.2f}°，"
                f"速度 {config.velocity_percent}%，超时 {config.timeout_s:.1f} s。"
            )
            return
        alignment = self.coordinator._accel_face_alignment(step_id, state.pose)
        if alignment is None:
            self.action_prediction.setText(f"无法计算 {step_id} 姿态预测")
            return
        current_angle, gravity_tool, face_name = alignment
        predicted_gravity = gravity_after_tool_rotation(
            gravity_tool, config.axis, config.degrees
        )
        target_vector = self.coordinator.ACCEL_FACE_TARGETS[step_id][1]
        predicted_angle = vector_angle_deg(predicted_gravity, target_vector)
        self.action_prediction.setText(
            f"当前 {step_id}（目标 {face_name}）偏差：{current_angle:.2f}°，重力(Tool)="
            f"({gravity_tool[0]:+.3f}, {gravity_tool[1]:+.3f}, {gravity_tool[2]:+.3f})\n"
            f"按配置旋转后的理论偏差：{predicted_angle:.2f}°，预测重力(Tool)="
            f"({predicted_gravity[0]:+.3f}, {predicted_gravity[1]:+.3f}, {predicted_gravity[2]:+.3f})\n"
            f"判定：{'预计可进入 5° 门限' if predicted_angle <= self.coordinator.ACCEL_FACE_MAX_DEG else '预计仍不能通过，请调整轴或角度'}"
        )

    @Slot()
    def _confirm_or_execute_current_action(self) -> None:
        step_id = self.coordinator.current_step.step_id
        if step_id in GYRO_AUTO_STEPS:
            self._start_gyro_x_auto_action(step_id)
            return
        if step_id in MAG_AUTO_STEPS:
            self._start_mag_auto_action(step_id)
            return
        config = self._config_for_step(step_id)
        if step_id in self.robot_actions and config is not None and config.enabled:
            self._start_accel_auto_action(step_id, config)
            return
        self.coordinator.confirm_current_action()

    def _try_start_full_auto_step(self) -> None:
        if not self._full_auto_enabled or not self.coordinator.running:
            return
        if (
            self.coordinator.state != RunState.READY
            or self._auto_action_step is not None
        ):
            return
        step_id = self.coordinator.current_step.step_id
        if step_id == "P1":
            # P1 has its own continuous two-second stillness gate in the coordinator.
            return
        if step_id in ("A01", "A02", "A03", "A04", "A05", "A06"):
            config = self._config_for_step(step_id)
            if config is None or not config.enabled:
                self._show_error(
                    f"全自动流程无法执行 {step_id}：该步骤的机械臂自动动作未启用",
                    True,
                )
                return
            self._start_accel_auto_action(step_id, config)
            return
        if step_id in GYRO_AUTO_STEPS:
            self._start_gyro_x_auto_action(step_id)
            return
        if step_id in MAG_AUTO_STEPS:
            self._start_mag_auto_action(step_id)
            return
        if step_id == "S02" and not self._update_full_auto_neutral_return():
            return
        if step_id in ("S01", "S02"):
            # Wait here rather than emitting repeated condition errors while the
            # previous robot action is returning to neutral.
            if self.coordinator._check_motion_condition(
                self.coordinator.current_step
            ):
                return
            self.coordinator.confirm_current_action()

    def _update_full_auto_neutral_return(self) -> bool:
        taught = self.taught_poses.get("neutral")
        if taught is None:
            self._show_error("全自动收尾失败：尚未示教标定中位", True)
            return False
        state = self.robot.latest_state
        now = time.monotonic_ns()
        if (
            not self.robot.connected
            or state is None
            or now - state.received_monotonic_ns >= 1_000_000_000
        ):
            self._show_error("全自动回标定中位失败：机械臂未连接或反馈超时", True)
            return False
        if (
            self._at_taught_pose(taught)
            and state.mode == 5
            and state.linear_speed_norm <= 1.0
            and state.angular_speed_norm <= 0.8
        ):
            self._full_auto_neutral_return_started_ns = 0
            self._full_auto_neutral_return_seen_motion = False
            return True
        if self._full_auto_neutral_return_started_ns == 0:
            if (
                state.mode != 5
                or state.linear_speed_norm > 1.0
                or state.angular_speed_norm > 0.8
            ):
                return False
            if not self.robot.activate_frames(taught.user, taught.tool):
                self._show_error("全自动回标定中位失败：User/Tool 启用失败", True)
                return False
            command_ok = (
                self.robot.move_joints(taught.joints, NEUTRAL_RETURN_SPEED_PERCENT)
                if taught.joints
                else self.robot.move_pose_j(
                    taught.pose,
                    NEUTRAL_RETURN_SPEED_PERCENT,
                    taught.user,
                    taught.tool,
                )
            )
            if not command_ok:
                self._show_error("全自动回标定中位命令发送失败", True)
                return False
            self._full_auto_neutral_return_started_ns = now
            self._full_auto_neutral_return_seen_motion = False
            self._append_log(
                f"S02 开始以 {NEUTRAL_RETURN_SPEED_PERCENT}% 速度自动返回示教标定中位",
                "info",
            )
            return False
        elapsed_ns = now - self._full_auto_neutral_return_started_ns
        if elapsed_ns > 90_000_000_000:
            self._show_error("全自动回标定中位等待到位超时", True)
            return False
        if state.mode in (7, 8) or state.linear_speed_norm > 1.0 or state.angular_speed_norm > 0.8:
            self._full_auto_neutral_return_seen_motion = True
            return False
        if state.mode != 5:
            return False
        if not self._full_auto_neutral_return_seen_motion and elapsed_ns < 3_000_000_000:
            return False
        self._show_error("机械臂已停止，但没有到达示教标定中位", True)
        return False

    def _start_accel_auto_action(
        self, step_id: str, config: RobotActionConfig
    ) -> None:
        if self._auto_action_step is not None:
            self._show_error("机械臂自动动作已经在执行", False)
            return
        if (
            self.coordinator.state != RunState.READY
            or self.coordinator.current_step.step_id != step_id
        ):
            self._show_error(f"只有在 {step_id} 等待动作条件时才能执行该自动旋转", False)
            return
        state = self.robot.latest_state
        if (
            not self.robot.connected
            or state is None
            or time.monotonic_ns() - state.received_monotonic_ns >= 1_000_000_000
        ):
            self._show_error("机械臂未连接或实时反馈已超时，不能执行自动动作", True)
            return
        if state.mode != 5 or state.linear_speed_norm > 1.0 or state.angular_speed_norm > 0.8:
            self._show_error(f"执行 {step_id} 自动动作前机械臂必须已使能、空闲并完全停止", True)
            return
        neutral = self.taught_poses.get("neutral")
        if neutral is None or state.user != neutral.user or state.tool != neutral.tool:
            self._show_error(
                "当前 User/Tool 与示教标定中位不一致，禁止执行相对旋转",
                True,
            )
            return
        alignment = self.coordinator._accel_face_alignment(step_id, state.pose)
        if alignment is None:
            self._show_error(f"无法计算当前 {step_id} 姿态", True)
            return
        current_angle, gravity_tool, _face_name = alignment
        predicted_gravity = gravity_after_tool_rotation(
            gravity_tool, config.axis, config.degrees
        )
        target_vector = self.coordinator.ACCEL_FACE_TARGETS[step_id][1]
        predicted_angle = vector_angle_deg(predicted_gravity, target_vector)
        if predicted_angle > self.coordinator.ACCEL_FACE_MAX_DEG:
            self._show_error(
                f"按当前配置旋转后预计偏差仍为 {predicted_angle:.2f}°，"
                f"超过 {self.coordinator.ACCEL_FACE_MAX_DEG:.1f}°，已禁止下发",
                True,
            )
            return

        if current_angle > self.coordinator.ACCEL_FACE_MAX_DEG:
            segments = 2 if abs(config.degrees) > 170.0 else 1
            message = (
                f"{step_id} 当前偏差 {current_angle:.2f}°。\n"
                f"将保持 XYZ 不变，沿当前 Tool {config.axis} 相对旋转 "
                f"{config.degrees:+.2f}°，速度 {config.velocity_percent}%。\n"
                f"控制器命令分为 {segments} 段，预计到位偏差 {predicted_angle:.2f}°。\n\n"
                "请确认整条旋转路径无碰撞、线束无拉扯，人员已离开运动区域。"
            )
            if (
                not self._full_auto_enabled
                and QMessageBox.question(
                    self, f"确认执行 {step_id} 自动旋转", message
                )
                != QMessageBox.StandardButton.Yes
            ):
                return

        now = time.monotonic_ns()
        self._auto_action_step = step_id
        self._auto_action_started_ns = now
        self._auto_action_deadline_ns = now + int(config.timeout_s * 1_000_000_000)
        self._auto_action_stable_since_ns = 0
        self._auto_action_seen_motion = current_angle <= self.coordinator.ACCEL_FACE_MAX_DEG
        self.coordinator.condition_stable_since_ns = 0
        detail = (
            f"axis={config.axis}; degrees={config.degrees:+.2f}; "
            f"velocity={config.velocity_percent}%; predicted_error={predicted_angle:.2f}deg"
        )
        self.coordinator.recorder.marker("robot_auto_move_request", step_id, detail)
        self._append_log(f"{step_id} 自动动作已准备：{detail}", "info")
        self._update_action_controls()

        if current_angle <= self.coordinator.ACCEL_FACE_MAX_DEG:
            self._append_log(f"{step_id} 当前姿态已在 5° 门限内，跳过旋转并自动等待稳定", "good")
            return
        if not self.robot.relative_tool_rotation(
            config.axis, config.degrees, config.velocity_percent
        ):
            self.robot.stop()
            self._finish_auto_action(f"{step_id} 自动旋转命令发送失败", "error")

    def _finish_auto_action(self, message: str, level: str = "info") -> None:
        if self._auto_action_step is not None:
            self.coordinator.recorder.marker(
                "robot_auto_move_end", self._auto_action_step, message
            )
        self._auto_action_step = None
        self._auto_action_started_ns = 0
        self._auto_action_deadline_ns = 0
        self._auto_action_stable_since_ns = 0
        self._auto_action_seen_motion = False
        self._mag_m04_motion_since_ns = 0
        self._append_log(message, level)
        self._update_action_controls()
        if level == "error" and self._full_auto_enabled:
            self._stop_full_auto(message, abort=True)
        elif self._full_auto_enabled:
            QTimer.singleShot(0, self._try_start_full_auto_step)

    def _check_auto_action_timeout(self, now_ns: int | None = None) -> None:
        if self._auto_action_step is None:
            return
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        if self._auto_action_deadline_ns and now_ns > self._auto_action_deadline_ns:
            step_id = self._auto_action_step
            if step_id in GYRO_AUTO_STEPS:
                self._fail_gyro_x_auto_action(f"{step_id} 自动动作超时")
                return
            if step_id in MAG_AUTO_STEPS:
                self._fail_mag_auto_action(f"{step_id} 自动磁工单动作超时")
                return
            self.robot.stop()
            self._finish_auto_action(f"{step_id} 自动旋转等待到位超时，已停止机械臂", "error")

    @staticmethod
    def _speed_factor_percent(value: float) -> int:
        value = float(value)
        percent = value if value > 1.5 else value * 100.0
        return max(1, min(100, round(percent)))

    def _gyro_motion_parameters(
        self, step_id: str
    ) -> tuple[str, float, float, float]:
        config = self._config_for_step(step_id)
        if config is None or not config.enabled:
            raise ValueError(f"{step_id} 机械臂自动动作未启用")
        expected_sweep = 90.0 if step_id in GYRO_LIMITED_STEPS else 150.0
        if not math.isclose(abs(config.degrees), expected_sweep, abs_tol=1e-6):
            raise ValueError(f"{step_id} 有效扫转角必须为 ±{expected_sweep:.0f}°")
        if step_id in GYRO_LIMITED_STEPS and not self.coordinator.limits.valid:
            raise ValueError("G01/G02 固定运动参数不是 ±55°/±45°、15°/s、6 s")
        direction = 1.0 if config.degrees > 0 else -1.0
        outer = LIMITED_GYRO_OUTER_DEG if step_id in GYRO_LIMITED_STEPS else 85.0
        capture_bound = (
            LIMITED_GYRO_CAPTURE_BOUND_DEG if step_id in GYRO_LIMITED_STEPS else 75.0
        )
        return (
            config.axis,
            -direction * outer,
            -direction * capture_bound,
            direction * ROLL_PITCH_GYRO_RATE_DEG_S,
        )

    def _gyro_decel_endpoint(self, step_id: str) -> float:
        """Return the opposite outer endpoint, outside the fitted capture window."""
        _axis, lead_in, _capture_start, _target_rate = (
            self._gyro_motion_parameters(step_id)
        )
        return -lead_in

    def _gyro_jog_speed_factor(
        self, axis: str, target_rate: float, tool: int
    ) -> tuple[int, str]:
        full_speed_deg_s = 0.0
        source = "保守估算"
        try:
            data = json.loads(ROTATION_SPEED_CALIBRATION_FILE.read_text(encoding="utf-8"))
            profiles = data.get("profiles", {})
            jog_profile = profiles.get(f"jog_tool_{int(tool)}_{axis}", {})
            full_speed_deg_s = float(
                jog_profile.get("full_global_speed_deg_s", 0.0) or 0.0
            )
            if full_speed_deg_s > 0.0:
                source = f"Tool {axis} 点动标定"
            else:
                relative_profile = profiles.get(f"tool_{int(tool)}_{axis}", {})
                rate = float(
                    relative_profile.get("deg_s_per_v_at_full_global", 0.0) or 0.0
                )
                if rate > 0.0:
                    full_speed_deg_s = rate * 100.0
                    source = f"Tool {axis} 相对运动标定"
        except (OSError, ValueError, TypeError):
            full_speed_deg_s = 0.0
        if not math.isfinite(full_speed_deg_s) or full_speed_deg_s <= 0.0:
            full_speed_deg_s = 100.0
        factor = round(abs(target_rate) / full_speed_deg_s * 100.0)
        return max(1, min(100, factor)), source

    def _relative_tool_axis_deg(
        self,
        reference_pose: tuple[float, ...],
        current_pose: tuple[float, ...],
        axis: str,
    ) -> float:
        reference_axes = self.coordinator._tool_axes(reference_pose)
        current_axes = self.coordinator._tool_axes(current_pose)
        if axis == "Rx":
            cosine = sum(a * b for a, b in zip(reference_axes[1], current_axes[1]))
            sine = sum(a * b for a, b in zip(reference_axes[2], current_axes[1]))
        elif axis == "Ry":
            cosine = sum(a * b for a, b in zip(reference_axes[0], current_axes[0]))
            sine = sum(a * b for a, b in zip(reference_axes[0], current_axes[2]))
        elif axis == "Rz":
            cosine = sum(a * b for a, b in zip(reference_axes[0], current_axes[0]))
            sine = sum(a * b for a, b in zip(reference_axes[1], current_axes[0]))
        else:
            raise ValueError(f"不支持的 Tool 相对旋转轴：{axis}")
        return math.degrees(math.atan2(sine, cosine))

    def _orientation_error_deg(
        self, reference_pose: tuple[float, ...], current_pose: tuple[float, ...]
    ) -> float:
        reference_axes = self.coordinator._tool_axes(reference_pose)
        current_axes = self.coordinator._tool_axes(current_pose)
        trace = sum(
            sum(a * b for a, b in zip(reference_axis, current_axis))
            for reference_axis, current_axis in zip(reference_axes, current_axes)
        )
        cosine = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
        return math.degrees(math.acos(cosine))

    def _start_gyro_x_auto_action(self, step_id: str) -> None:
        if step_id not in GYRO_AUTO_STEPS:
            self._show_error(f"不支持的自动陀螺步骤：{step_id}", True)
            return
        if self._auto_action_step is not None:
            self._show_error("机械臂自动动作已经在执行", False)
            return
        if (
            self.coordinator.state != RunState.READY
            or self.coordinator.current_step.step_id != step_id
        ):
            self._show_error(f"只有在 {step_id} 等待动作条件时才能执行自动扫转", False)
            return
        state = self.robot.latest_state
        if (
            not self.robot.connected
            or state is None
            or time.monotonic_ns() - state.received_monotonic_ns >= 1_000_000_000
            or state.mode != 5
        ):
            self._show_error("机械臂未连接、反馈超时或未处于已使能空闲状态", True)
            return
        neutral = self.taught_poses.get("neutral")
        if neutral is None or state.user != neutral.user or state.tool != neutral.tool:
            self._show_error(
                f"当前 User/Tool 与示教标定中位不一致，禁止执行 {step_id}", True
            )
            return
        try:
            axis, lead_in, capture_start, target_rate = self._gyro_motion_parameters(
                step_id
            )
        except (KeyError, ValueError) as exc:
            self._show_error(f"{step_id} 自动动作参数无效：{exc}", True)
            return
        decel_end = self._gyro_decel_endpoint(step_id)

        if step_id == "G01":
            alignment = self.coordinator._accel_face_alignment("A06", state.pose)
            if alignment is None or alignment[0] > self.coordinator.ACCEL_FACE_MAX_DEG:
                detail = (
                    "无法计算当前姿态"
                    if alignment is None
                    else f"当前 -Z 偏差 {alignment[0]:.2f}°"
                )
                self._show_error(f"G01 必须从 A06 完成姿态开始；{detail}", True)
                return
            after_rx = gravity_after_tool_rotation(
                alignment[1], "Rx", G01_RESTORE_RX_DEG
            )
            rx_restore_error = vector_angle_deg(after_rx, (0.0, -1.0, 0.0))
            if rx_restore_error > self.coordinator.ACCEL_FACE_MAX_DEG:
                self._show_error(
                    f"G01 基准姿态预测未通过：Rx 复原后 -Y 偏差 "
                    f"{rx_restore_error:.2f}°",
                    True,
                )
                return
            message = (
                "G01 将自动执行以下动作：\n"
                "1. Tool Rx +90°，撤销 A05/A06 的净 Rx 动作并回到 -Y；\n"
                "2. 以 -Y 姿态作为 G01/G02 的 Rx 零位，使重力与 Rx 垂直；\n"
                f"3. Tool {axis} {lead_in:+.1f}°，作为加速引入区；\n"
                f"4. Tool {axis}{'+' if target_rate > 0 else '-'} 点动，"
                f"经过 {capture_start:+.1f}°且稳定在 "
                f"{target_rate:+.1f}±{max(GYRO_RATE_TOLERANCE_DEG_S, abs(target_rate) * 0.2):.1f}°/s "
                f"时开启 {self.coordinator.current_step.capture_s:.2f} 秒采集，"
                f"预计到达 {-capture_start:+.1f}°；\n"
                f"5. 关闭采集后在减速区到达 Tool {axis} {decel_end:+.1f}°，"
                "再回到 G01/G02 共用的 -Y 基准。\n\n"
                "请确认完整路径无碰撞、线束无拉扯，人员已离开运动区域。"
            )
        else:
            if step_id == "G02":
                alignment = self.coordinator._accel_face_alignment("A04", state.pose)
                if alignment is None or alignment[0] > self.coordinator.ACCEL_FACE_MAX_DEG:
                    error = math.inf if alignment is None else alignment[0]
                    self._show_error(
                        f"G02 必须从 G01 返回的 -Y 基准开始；当前偏差 {error:.2f}°",
                        True,
                    )
                    return
            neutral_error = self._orientation_error_deg(neutral.pose, state.pose)
            neutral_tolerance = (
                2.0
                if step_id in GYRO_LIMITED_STEPS
                else self.coordinator.ACCEL_FACE_MAX_DEG
            )
            if step_id != "G02" and neutral_error > neutral_tolerance:
                self._show_error(
                    f"{step_id} 必须从上一步回到标定中位后开始；"
                    f"当前与示教中位的三维姿态偏差 {neutral_error:.2f}°，"
                    f"允许 {neutral_tolerance:.1f}°",
                    True,
                )
                return
            direction = "+" if target_rate > 0 else "-"
            if step_id in GYRO_LIMITED_STEPS:
                limits = self.coordinator.limits
                capture_end = (
                    limits.positive_safe_deg
                    if target_rate > 0
                    else limits.negative_safe_deg
                )
                range_detail = (
                    f"完整行程 {limits.negative_soft_limit_deg:+.1f}°～"
                    f"{limits.positive_soft_limit_deg:+.1f}°，匀速采集区 "
                    f"{limits.negative_safe_deg:+.1f}°～{limits.positive_safe_deg:+.1f}°"
                )
            else:
                capture_end = -capture_start
                range_detail = (
                    f"匀速采集范围约 {capture_start:+.0f}°～{capture_end:+.0f}°"
                )
            message = (
                f"{step_id} 将自动执行以下动作：\n"
                f"1. 以当前标定中位为 {axis} 零位，Tool {axis} "
                f"{lead_in:+.1f}°预置；\n"
                f"2. Tool {axis}{direction} 点动，经过 {capture_start:+.1f}°且稳定在 "
                f"{target_rate:+.1f}±{max(GYRO_RATE_TOLERANCE_DEG_S, abs(target_rate) * 0.2):.1f}°/s "
                f"时开启 {self.coordinator.current_step.capture_s:.2f} 秒采集；\n"
                f"3. {range_detail}，在 {capture_end:+.1f}°关闭采集；\n"
                f"4. 在采集窗外减速并到达 {decel_end:+.1f}°，"
                f"再回到本次 {step_id} 的标定中位。\n\n"
                "请确认完整路径无碰撞、线束无拉扯，人员已离开运动区域。"
            )
        if (
            not self._full_auto_enabled
            and QMessageBox.question(
                self, f"确认执行 {step_id} 自动扫转", message
            )
            != QMessageBox.StandardButton.Yes
        ):
            return

        now = time.monotonic_ns()
        self._auto_action_step = step_id
        self._auto_action_started_ns = now
        timeout_s = max(
            GYRO_TIMEOUT_S, self.coordinator.current_step.capture_s + 30.0
        )
        self._auto_action_deadline_ns = now + int(timeout_s * 1_000_000_000)
        self._auto_action_seen_motion = False
        self._auto_action_stable_since_ns = 0
        self._gyro_x_phase_started_ns = now
        self._gyro_x_reference_pose = None
        self._gyro_x_original_speed_factor = self._speed_factor_percent(
            state.speed_scaling
        )
        self._gyro_x_jog_speed_factor = 0
        self._gyro_x_decel_level = -1
        self._gyro_x_jog_active = False
        self._update_action_controls()
        if step_id == "G01":
            self._gyro_x_phase = "restore_rx"
            self.coordinator.recorder.marker(
                "robot_auto_move_request",
                step_id,
                f"restore=Rx+90; gravity_base=-Y; lead_in={axis}{lead_in:+.1f}; "
                f"capture={axis}{'+' if target_rate > 0 else '-'}"
                f"@{abs(target_rate):.1f}deg/s",
            )
            self._append_log(
                f"G01 开始建立 -Y 重力基准：执行 Tool Rx "
                f"+{G01_RESTORE_RX_DEG:.1f}°；"
                f"原全局速度比例 {self._gyro_x_original_speed_factor}%",
                "info",
            )
            if not self.robot.relative_tool_rotation(
                "Rx", G01_RESTORE_RX_DEG, GYRO_RETURN_SPEED_PERCENT
            ):
                self._fail_gyro_x_auto_action("G01 Rx 复原命令发送失败")
        else:
            if step_id == "G02":
                self._gyro_x_reference_pose = tuple(state.pose)
                self.coordinator.gyro_limited_reference_pose = tuple(state.pose)
            self.coordinator.recorder.marker(
                "robot_auto_move_request",
                step_id,
                f"lead_in={axis}{lead_in:+.1f}; capture={axis}"
                f"{'+' if target_rate > 0 else '-'}@{abs(target_rate):.1f}deg/s",
            )
            self._begin_gyro_x_preposition(step_id, state, now)

    def _begin_gyro_x_preposition(self, step_id: str, state, now: int) -> None:
        axis, lead_in, _capture_start, _target_rate = self._gyro_motion_parameters(
            step_id
        )
        if step_id in GYRO_LIMITED_STEPS:
            if self._gyro_x_reference_pose is None:
                reference = self.coordinator.gyro_limited_reference_pose
                if reference is None:
                    self._fail_gyro_x_auto_action(
                        f"{step_id} 缺少 G01/G02 共用的 -Y 基准姿态"
                    )
                    return
                self._gyro_x_reference_pose = tuple(reference)
            current_angle = self._relative_tool_axis_deg(
                self._gyro_x_reference_pose, state.pose, axis
            )
            command_angle = lead_in - current_angle
            reference_description = (
                f"-Y 重力基准（当前 {axis} 偏差 {current_angle:+.2f}°，已补偿）"
            )
        else:
            self._gyro_x_reference_pose = tuple(state.pose)
            command_angle = lead_in
            reference_description = "当前标定中位"
        self._gyro_x_phase = "preposition"
        self._gyro_x_phase_started_ns = now
        self._auto_action_seen_motion = False
        self._append_log(
            f"{step_id} 已使用{reference_description}作为零位，"
            f"开始 Tool {axis} {command_angle:+.2f}°运动到 {lead_in:+.1f}°加速引入位",
            "good",
        )
        if not self.robot.relative_tool_rotation(
            axis, command_angle, GYRO_TRANSIT_SPEED_PERCENT
        ):
            self._fail_gyro_x_auto_action(f"{step_id} 加速引入预置命令发送失败")

    def _restore_gyro_x_speed_factor(self) -> None:
        if self._gyro_x_original_speed_factor:
            self.robot.set_speed_factor(self._gyro_x_original_speed_factor)
            self._gyro_x_original_speed_factor = 0

    def _stop_gyro_x_jog(self, *, restore_speed: bool = True) -> None:
        if self._gyro_x_jog_active:
            self.robot.stop_tool_jog()
            self._gyro_x_jog_active = False
        if restore_speed:
            self._restore_gyro_x_speed_factor()

    def _set_gyro_x_decel_level(self, level: int) -> bool:
        if level <= self._gyro_x_decel_level:
            return True
        _threshold, multiplier = GYRO_SMOOTH_DECEL_PROFILE[level]
        speed_factor = max(1, round(self._gyro_x_jog_speed_factor * multiplier))
        if not self.robot.set_speed_factor(speed_factor):
            return False
        self._gyro_x_decel_level = level
        return True

    def _fail_gyro_x_auto_action(self, message: str) -> None:
        self._stop_gyro_x_jog()
        self.robot.stop()
        self._gyro_x_phase = ""
        self._gyro_x_reference_pose = None
        self._finish_auto_action(message, "error")
        if self.coordinator.running:
            self.coordinator.abort(message)

    def _begin_gyro_x_return(
        self, step_id: str, axis: str, angle: float, now: int
    ) -> None:
        if abs(angle) < 0.5:
            self._gyro_x_phase = "return"
            self._gyro_x_phase_started_ns = now - 800_000_000
            self._auto_action_seen_motion = True
            return
        if abs(angle) > 180.0:
            self._fail_gyro_x_auto_action(
                f"{step_id} 端点位置无法安全回中：{axis} {angle:+.2f}°"
            )
            return
        self._gyro_x_phase = "return"
        self._gyro_x_phase_started_ns = now
        self._auto_action_seen_motion = False
        self._append_log(
            f"{step_id} 已完成采集窗外减速区，当前 {axis} {angle:+.2f}°，"
            "开始回标定中位",
            "info",
        )
        if not self.robot.relative_tool_rotation(
            axis, -angle, GYRO_RETURN_SPEED_PERCENT
        ):
            self._fail_gyro_x_auto_action(f"{step_id} 回标定中位命令发送失败")

    def _update_gyro_x_auto_action(self, state) -> None:
        step_id = self._auto_action_step
        if step_id not in GYRO_AUTO_STEPS:
            return
        now = time.monotonic_ns()
        self._check_auto_action_timeout(now)
        if self._auto_action_step != step_id:
            return
        try:
            axis, lead_in, capture_start, target_rate = self._gyro_motion_parameters(
                step_id
            )
        except (KeyError, ValueError) as exc:
            self._fail_gyro_x_auto_action(f"{step_id} 自动动作参数失效：{exc}")
            return
        axis_index = {"Rx": 0, "Ry": 1, "Rz": 2}[axis]
        rate_tolerance = max(
            GYRO_RATE_TOLERANCE_DEG_S, abs(target_rate) * 0.2
        )
        phase = self._gyro_x_phase
        if phase in (
            "restore_rx",
            "restore_neutral_rz",
            "preposition",
            "postposition",
            "return",
        ):
            if state.mode in (7, 8) or state.angular_speed_norm > 0.8:
                self._auto_action_seen_motion = True
                return
            if state.mode != 5 or state.linear_speed_norm > 1.0 or state.angular_speed_norm > 0.8:
                return
            if (
                not self._auto_action_seen_motion
                and now - self._gyro_x_phase_started_ns < 800_000_000
            ):
                return
            if phase == "restore_rx":
                alignment = self.coordinator._accel_face_alignment("A04", state.pose)
                if alignment is None or alignment[0] > self.coordinator.ACCEL_FACE_MAX_DEG:
                    error = math.inf if alignment is None else alignment[0]
                    self._fail_gyro_x_auto_action(
                        f"G01 Rx +90°复原后 -Y 偏差 {error:.2f}°"
                    )
                    return
                self._gyro_x_reference_pose = tuple(state.pose)
                self.coordinator.gyro_limited_reference_pose = tuple(state.pose)
                self._append_log(
                    "G01 已恢复 -Y；保持该姿态作为 G01/G02 共用重力基准", "good"
                )
                self._begin_gyro_x_preposition(step_id, state, now)
                return
            if phase == "restore_neutral_rz":
                alignment = self.coordinator._accel_face_alignment("A02", state.pose)
                if alignment is None or alignment[0] > self.coordinator.ACCEL_FACE_MAX_DEG:
                    error = math.inf if alignment is None else alignment[0]
                    self._fail_gyro_x_auto_action(
                        f"G02 Rz +90°回全局中位后 -X 偏差 {error:.2f}°"
                    )
                    return
                neutral = self.taught_poses.get("neutral")
                if neutral is None:
                    self._fail_gyro_x_auto_action("G02 缺少示教标定中位")
                    return
                orientation_error = self._orientation_error_deg(
                    neutral.pose, state.pose
                )
                if orientation_error > 2.0:
                    self._fail_gyro_x_auto_action(
                        "G02 回全局中位后的重力方向合格，但绕重力轴方向不正确："
                        f"与示教中位三维姿态偏差 {orientation_error:.2f}°，允许 2.0°"
                    )
                    return
                self.coordinator.gyro_limited_reference_pose = None
                self._gyro_x_phase = ""
                self._gyro_x_reference_pose = None
                self._finish_auto_action(
                    "G02 已完成采集并从 -Y 基准回到全局 -X 标定中位", "good"
                )
                return
            if self._gyro_x_reference_pose is None:
                self._fail_gyro_x_auto_action(f"{step_id} 缺少标定中位参考姿态")
                return
            angle = self._relative_tool_axis_deg(
                self._gyro_x_reference_pose, state.pose, axis
            )
            if phase == "preposition":
                if abs(angle - lead_in) > 5.0:
                    self._fail_gyro_x_auto_action(
                        f"{step_id} 预置到位角度异常：目标 {lead_in:+.1f}°，"
                        f"实测 {axis} {angle:+.2f}°"
                    )
                    return
                speed_factor, source = self._gyro_jog_speed_factor(
                    axis, target_rate, state.tool
                )
                self._gyro_x_phase = "lead_in"
                self._gyro_x_phase_started_ns = now
                self._auto_action_stable_since_ns = 0
                self._append_log(
                    f"{step_id} 开始 {axis}{'+' if target_rate > 0 else '-'} 点动："
                    f"速度比例 {speed_factor}%（{source}），等待经过 "
                    f"{capture_start:+.1f}°并达到 {target_rate:+.1f}±"
                    f"{rate_tolerance:.1f}°/s",
                    "info",
                )
                if not self.robot.start_tool_jog(
                    axis,
                    target_rate > 0,
                    speed_factor,
                    user=state.user,
                    tool=state.tool,
                ):
                    self._fail_gyro_x_auto_action(f"{step_id} 连续点动命令发送失败")
                    return
                self._gyro_x_jog_speed_factor = speed_factor
                self._gyro_x_decel_level = -1
                self._gyro_x_jog_active = True
                return
            if phase == "postposition":
                decel_end = self._gyro_decel_endpoint(step_id)
                if abs(angle - decel_end) > GYRO_ENDPOINT_TOLERANCE_DEG:
                    self._fail_gyro_x_auto_action(
                        f"{step_id} 减速端点到位异常：目标 {axis} "
                        f"{decel_end:+.1f}°，实测 {angle:+.2f}°"
                    )
                    return
                self._begin_gyro_x_return(step_id, axis, angle, now)
                return
            orientation_error = self._orientation_error_deg(
                self._gyro_x_reference_pose, state.pose
            )
            if orientation_error > 2.0:
                self._fail_gyro_x_auto_action(
                    f"{step_id} 回标定中位后仍有三维姿态偏差 "
                    f"{orientation_error:.2f}°（{axis}={angle:+.2f}°）"
                )
                return
            if step_id == "G02":
                self._gyro_x_phase = "restore_neutral_rz"
                self._gyro_x_phase_started_ns = now
                self._auto_action_seen_motion = False
                self._append_log(
                    f"G02 已回到 -Y 基准，开始 Tool Rz "
                    f"+{G02_RESTORE_NEUTRAL_RZ_DEG:.1f}°回全局 -X 标定中位",
                    "good",
                )
                if not self.robot.relative_tool_rotation(
                    "Rz",
                    G02_RESTORE_NEUTRAL_RZ_DEG,
                    GYRO_RETURN_SPEED_PERCENT,
                ):
                    self._fail_gyro_x_auto_action(
                        "G02 Rz 回全局标定中位命令发送失败"
                    )
                return
            self._gyro_x_phase = ""
            self._gyro_x_reference_pose = None
            detail = (
                "G01 已完成采集并回到 G01/G02 共用 -Y 基准"
                if step_id == "G01"
                else f"{step_id} 已完成采集并回到标定中位"
            )
            self._finish_auto_action(detail, "good")
            return

        if self._gyro_x_reference_pose is None:
            self._fail_gyro_x_auto_action(f"{step_id} 缺少标定中位参考姿态")
            return
        angle = self._relative_tool_axis_deg(
            self._gyro_x_reference_pose, state.pose, axis
        )
        tool_rate = self.coordinator._tool_angular_speed(
            state.pose, state.angular_speed
        )[axis_index]
        if phase == "lead_in":
            rate_ok = (
                target_rate - rate_tolerance
                <= tool_rate
                <= target_rate + rate_tolerance
            )
            if rate_ok:
                if self._auto_action_stable_since_ns == 0:
                    self._auto_action_stable_since_ns = now
            else:
                self._auto_action_stable_since_ns = 0
            stable = (
                self._auto_action_stable_since_ns
                and now - self._auto_action_stable_since_ns >= GYRO_SPEED_STABLE_NS
            )
            reached_start = (
                angle >= capture_start if target_rate > 0 else angle <= capture_start
            )
            if reached_start and stable:
                self.coordinator.condition_stable_since_ns = now - 1_100_000_000
                if not self.coordinator.confirm_current_action():
                    self._fail_gyro_x_auto_action(
                        f"{step_id} 匀速到位后的阶段开启复检未通过"
                    )
                    return
                self._gyro_x_phase = "wait_stage_open"
                self._append_log(
                    f"{step_id} 经过 {axis} {angle:+.2f}°，实测 {tool_rate:+.2f}°/s，"
                    "正在开启采集",
                    "good",
                )
                return
            missed_threshold = (
                capture_start + GYRO_MISSED_START_MARGIN_DEG
                if target_rate > 0
                else capture_start - GYRO_MISSED_START_MARGIN_DEG
            )
            missed_start = (
                angle > missed_threshold
                if target_rate > 0
                else angle < missed_threshold
            )
            if missed_start:
                self._fail_gyro_x_auto_action(
                    f"{step_id} 已越过 {missed_threshold:+.1f}°"
                    f"仍未达到稳定速度：实测 {tool_rate:+.2f}°/s"
                )
            return
        if phase == "wait_stage_open" and self.coordinator.state == RunState.CAPTURING:
            self._gyro_x_phase = "capturing"
            return
        if phase == "capturing":
            if step_id in GYRO_LIMITED_STEPS:
                end_angle = (
                    self.coordinator.limits.positive_safe_deg
                    if target_rate > 0
                    else self.coordinator.limits.negative_safe_deg
                )
                overshoot = 2.0
            else:
                end_angle = -capture_start
                overshoot = 5.0
            beyond_end = (
                angle > end_angle + overshoot
                if target_rate > 0
                else angle < end_angle - overshoot
            )
            if beyond_end:
                self._fail_gyro_x_auto_action(
                    f"{step_id} 扫转已越过预计终点：{axis}={angle:+.2f}°，"
                    f"终点={end_angle:+.2f}°，已紧急停止"
                )
            return
        if phase == "smooth_decel":
            capture_end = (
                self.coordinator.limits.positive_safe_deg
                if step_id in GYRO_LIMITED_STEPS and target_rate > 0
                else self.coordinator.limits.negative_safe_deg
                if step_id in GYRO_LIMITED_STEPS
                else -capture_start
            )
            decel_end = self._gyro_decel_endpoint(step_id)
            direction = 1.0 if target_rate > 0 else -1.0
            region_deg = direction * (decel_end - capture_end)
            progress_deg = direction * (angle - capture_end)
            if region_deg <= 0.0:
                self._fail_gyro_x_auto_action(
                    f"{step_id} 减速区配置无效：{capture_end:+.1f}°→{decel_end:+.1f}°"
                )
                return
            if progress_deg > region_deg + GYRO_DECEL_OVERSHOOT_TOLERANCE_DEG:
                self._fail_gyro_x_auto_action(
                    f"{step_id} 丝滑减速时越过外端点：{axis}={angle:+.2f}°"
                )
                return
            progress_ratio = max(0.0, progress_deg / region_deg)
            desired_level = max(
                index
                for index, (threshold, _multiplier) in enumerate(
                    GYRO_SMOOTH_DECEL_PROFILE
                )
                if progress_ratio >= threshold
            )
            if desired_level > self._gyro_x_decel_level:
                if not self._set_gyro_x_decel_level(desired_level):
                    self._fail_gyro_x_auto_action(
                        f"{step_id} 动态降低点动速度失败，已停止机械臂"
                    )
                    return
            remaining_deg = direction * (decel_end - angle)
            if remaining_deg <= GYRO_SMOOTH_STOP_MARGIN_DEG:
                self._stop_gyro_x_jog(restore_speed=False)
                self._gyro_x_phase = "wait_jog_stop"
                self._gyro_x_phase_started_ns = now
            return
        if phase == "wait_jog_stop":
            if state.mode != 5 or state.angular_speed_norm > 0.8:
                return
            # Some controller versions reject SpeedFactor while a jog is still
            # decelerating, so restore it only after the robot is truly idle.
            self._restore_gyro_x_speed_factor()
            step_index = GYRO_AUTO_STEPS.index(step_id)
            expected_next = (
                GYRO_AUTO_STEPS[step_index + 1]
                if step_index + 1 < len(GYRO_AUTO_STEPS)
                else (
                    "S01"
                    if self.coordinator.SKIP_MAGNETIC_STAGES
                    else "M01"
                )
            )
            if (
                self.coordinator.state != RunState.READY
                or self.coordinator.current_step.step_id != expected_next
            ):
                return
            stop_angle = self._relative_tool_axis_deg(
                self._gyro_x_reference_pose, state.pose, axis
            )
            decel_end = self._gyro_decel_endpoint(step_id)
            if step_id in GYRO_LIMITED_STEPS:
                limits = self.coordinator.limits
                if not (
                    limits.negative_soft_limit_deg - GYRO_DECEL_OVERSHOOT_TOLERANCE_DEG
                    <= stop_angle
                    <= limits.positive_soft_limit_deg + GYRO_DECEL_OVERSHOOT_TOLERANCE_DEG
                ):
                    self._fail_gyro_x_auto_action(
                        f"{step_id} 减速时越过 Tool {axis} 软限位："
                        f"当前 {stop_angle:+.2f}°"
                    )
                    return
            if abs(stop_angle - decel_end) <= GYRO_ENDPOINT_TOLERANCE_DEG:
                self._begin_gyro_x_return(step_id, axis, stop_angle, now)
                return
            endpoint_move = decel_end - stop_angle
            if abs(endpoint_move) > 30.0:
                self._fail_gyro_x_auto_action(
                    f"{step_id} 减速停止位置异常：{axis} {stop_angle:+.2f}°，"
                    f"距离端点 {decel_end:+.1f}° 仍有 {endpoint_move:+.2f}°"
                )
                return
            self._gyro_x_phase = "postposition"
            self._gyro_x_phase_started_ns = now
            self._auto_action_seen_motion = False
            self._append_log(
                f"{step_id} 已在采集窗外停止于 {axis} {stop_angle:+.2f}°，"
                f"以 {GYRO_POSTPOSITION_SPEED_PERCENT}% 补偿到减速端点 "
                f"{decel_end:+.1f}°",
                "info",
            )
            if not self.robot.relative_tool_rotation(
                axis,
                endpoint_move,
                GYRO_POSTPOSITION_SPEED_PERCENT,
                GYRO_POSTPOSITION_ACCEL_PERCENT,
            ):
                self._fail_gyro_x_auto_action(
                    f"{step_id} 减速端点补偿命令发送失败"
                )

    def _start_mag_auto_action(self, step_id: str) -> None:
        if step_id not in MAG_AUTO_STEPS:
            self._show_error(f"不支持的自动磁翻转步骤：{step_id}", True)
            return
        if self._auto_action_step is not None:
            self._show_error("机械臂自动动作已经在执行", False)
            return
        if (
            self.coordinator.state != RunState.READY
            or self.coordinator.current_step.step_id != step_id
        ):
            self._show_error(f"只有在 {step_id} 等待动作条件时才能执行自动磁翻转", False)
            return
        if not self.coordinator.environment_confirmed:
            self._show_error("磁翻转前必须确认磁环境、夹具和线束状态", True)
            return
        state = self.robot.latest_state
        if (
            not self.robot.connected
            or state is None
            or time.monotonic_ns() - state.received_monotonic_ns >= 1_000_000_000
            or state.mode != 5
            or state.linear_speed_norm > 1.0
            or state.angular_speed_norm > 0.8
        ):
            self._show_error("机械臂未连接、反馈超时或未在已使能静止状态", True)
            return
        neutral = self.taught_poses.get("neutral")
        if neutral is None or state.user != neutral.user or state.tool != neutral.tool:
            self._show_error(
                f"当前 User/Tool 与示教标定中位不一致，禁止执行 {step_id}", True
            )
            return
        neutral_error = self._orientation_error_deg(neutral.pose, state.pose)
        if neutral_error > MAG_NEUTRAL_TOLERANCE_DEG:
            self._show_error(
                f"{step_id} 必须从标定中位开始；当前三维姿态偏差 "
                f"{neutral_error:.2f}°，允许 {MAG_NEUTRAL_TOLERANCE_DEG:.1f}°",
                True,
            )
            return
        trajectory = MAG_TRAJECTORIES[step_id]
        if step_id == "M01":
            planned_s = (
                len(MAG_M01_POSE_WAYPOINTS) - 1
            ) * MAG_M01_SEGMENT_S
        else:
            planned_s = sum(
                duration_s for _axis, _positive, duration_s in trajectory
            )
        if step_id == "M04":
            planned_s = self.coordinator.current_step.exit_s
        expected_s = (
            self.coordinator.current_step.exit_s
            if step_id == "M04"
            else self.coordinator.current_step.capture_s
        )
        if abs(planned_s - expected_s) > 0.01:
            self._show_error(
                f"{step_id} 自动轨迹 {planned_s:.1f} s 与工单时间 "
                f"{expected_s:.1f} s 不一致",
                True,
            )
            return
        if step_id == "M01":
            path_text = (
                "G03/G04 式固定 TCP XYZ 的 Tool Ry 俯仰 + J6 方向旋转"
                "组合轨迹（8×5.0s，Ry/Rz 均 ±75°）"
            )
        elif trajectory:
            path_text = " → ".join(
                f"{axis}{'+' if positive else '-'}({duration_s:.1f}s)"
                for axis, positive, duration_s in trajectory
            )
        else:
            path_text = "Tool Rz=0°，标定中位静止保持"
        limits = self.coordinator.limits
        if step_id == "M01":
            action_detail = (
                f"Tool Ry（Pitch）与 J6 方向 Tool Rz（Roll）同步叠加：{path_text}\n"
                "以进入 M01 时的 TCP XYZ 和 Tool 姿态为参考，先合成 Ry/Rz "
                "完整目标姿态，再使用 MovL 任务空间插补；"
                f"XYZ 始终固定，CP={MAG_M01_BLEND_PERCENT}；"
                f"Ry 俯仰限制为 ±{MAG_M01_PITCH_SAFE_DEG:.0f}°，"
                f"J6 实时关节偏移保护为 ±{MAG_M01_J6_SAFE_DEG:.0f}°。"
            )
        elif step_id == "M02":
            action_detail = (
                f"Yaw 正向单侧往返：{path_text}\n"
                f"目标速度 {MAG_TARGET_RATE_DEG_S[step_id]:.1f}°/s，"
                f"路径约 0° → +{MAG_YAW_SAFE_DEG:.0f}° → 0°。"
            )
        elif step_id == "M03":
            action_detail = (
                f"Yaw 负向单侧往返：{path_text}\n"
                f"目标速度 {MAG_TARGET_RATE_DEG_S[step_id]:.1f}°/s，"
                f"路径约 0° → -{MAG_YAW_SAFE_DEG:.0f}° → 0°。"
            )
        else:
            action_detail = (
                "确认 Tool Rz=0° 中位，机械臂静止保持 5 秒，线束无受力；"
                "本阶段只做安全收尾，不采集磁数据。"
            )
        message = (
            f"{step_id} 将按工单执行 {planned_s:.0f} 秒磁阶段：\n{action_detail}\n"
            "阶段完成后机械臂保持或回到示教标定中位。\n\n"
            "请确认完整路径无碰撞、线束无拉扯，人员已离开运动区域。"
        )
        if (
            not self._full_auto_enabled
            and QMessageBox.question(
                self, f"确认执行 {step_id} 自动磁工单动作", message
            )
            != QMessageBox.StandardButton.Yes
        ):
            return

        now = time.monotonic_ns()
        self._auto_action_step = step_id
        self._auto_action_started_ns = now
        self._auto_action_deadline_ns = now + int(MAG_TIMEOUT_S * 1_000_000_000)
        self._auto_action_stable_since_ns = 0
        self._auto_action_seen_motion = False
        self._mag_m04_motion_since_ns = 0
        self._mag_phase = "static_wait_stop" if step_id == "M04" else "wait_stage_open"
        self._mag_phase_started_ns = now
        self._mag_segment_index = -1
        self._mag_pending_segment_index = -1
        # Capture the actual entry pose and joints. M01 holds this TCP XYZ and
        # composes both rotations before issuing each Cartesian target.
        self._mag_reference_pose = tuple(state.pose)
        self._mag_reference_joints = tuple(state.joints)
        self._mag_original_speed_factor = self._speed_factor_percent(
            state.speed_scaling
        )
        self._mag_speed_factor = 0
        self._mag_speed_source = ""
        self._mag_jog_active = False
        self.coordinator.condition_stable_since_ns = 0
        self.coordinator.recorder.marker(
            "robot_auto_move_request",
            step_id,
            f"trajectory={path_text}; duration={planned_s:.1f}s; "
            f"control={'MovL-fixed-XYZ-Tool-Ry-Rz-combined' if step_id == 'M01' else 'MoveJog'}; "
            f"rate={MAG_TARGET_RATE_DEG_S.get(step_id, 0.0):.1f}deg/s; "
            f"Rx_safe={limits.negative_safe_deg:+.1f}..{limits.positive_safe_deg:+.1f}",
        )
        self._append_log(
            f"{step_id} 开始磁工单动作，原全局速度比例 "
            f"{self._mag_original_speed_factor}%：{path_text}",
            "info",
        )
        self._update_action_controls()
        if step_id == "M04":
            # M03 may have only just completed its return.  Stop once, then
            # wait for the feedback stream to prove the arm is stationary
            # before starting M04's required five-second hold timer.
            self.robot.stop()
            self._append_log("M04 等待机械臂完全停稳后开始静止 5 秒；本阶段不采集磁数据", "good")
            return
        # Open the stage while still exactly at neutral.  The first jog starts
        # from the CAPTURING notification so timed paths map to their endpoints
        # without a pre-capture lead-in angle.
        self.coordinator.condition_stable_since_ns = now
        if not self.coordinator.confirm_current_action():
            self._fail_mag_auto_action(f"{step_id} 中位阶段开启复检未通过")
        else:
            self._append_log(f"{step_id} 正在开启磁采集，ACK 后开始运动", "good")

    @staticmethod
    def _matmul3(left, right):
        return tuple(
            tuple(sum(left[row][index] * right[index][column] for index in range(3)) for column in range(3))
            for row in range(3)
        )

    def _m01_absolute_targets(
        self, reference_pose: tuple[float, ...]
    ) -> tuple[tuple[float, ...], ...]:
        """Build fixed-XYZ targets as R0 * Ry(pitch) * Rz(J6 roll)."""
        axes = self.coordinator._tool_axes(reference_pose)
        reference = tuple(
            tuple(axes[column][row] for column in range(3))
            for row in range(3)
        )
        targets = []
        for pitch_deg, j6_roll_deg in MAG_M01_POSE_WAYPOINTS[1:]:
            pitch = math.radians(pitch_deg)
            roll = math.radians(j6_roll_deg)
            ry = (
                (math.cos(pitch), 0.0, math.sin(pitch)),
                (0.0, 1.0, 0.0),
                (-math.sin(pitch), 0.0, math.cos(pitch)),
            )
            rz = (
                (math.cos(roll), -math.sin(roll), 0.0),
                (math.sin(roll), math.cos(roll), 0.0),
                (0.0, 0.0, 1.0),
            )
            matrix = self._matmul3(reference, self._matmul3(ry, rz))
            controller_ry = math.asin(
                max(-1.0, min(1.0, -matrix[2][0]))
            )
            controller_rx = math.atan2(matrix[2][1], matrix[2][2])
            controller_rz = math.atan2(matrix[1][0], matrix[0][0])
            targets.append(
                (
                    *reference_pose[:3],
                    *(
                        math.degrees(value)
                        for value in (controller_rx, controller_ry, controller_rz)
                    ),
                )
            )
        return tuple(targets)

    def _m01_fixed_frame_angles(
        self, reference_pose: tuple[float, ...], current_pose: tuple[float, ...]
    ) -> tuple[float, float]:
        """Extract (Roll/Rz, Pitch/Ry) from R0·Ry(pitch)·Rz(roll)."""
        ref_axes = self.coordinator._tool_axes(reference_pose)
        cur_axes = self.coordinator._tool_axes(current_pose)
        reference = tuple(
            tuple(ref_axes[column][row] for column in range(3))
            for row in range(3)
        )
        current = tuple(
            tuple(cur_axes[column][row] for column in range(3))
            for row in range(3)
        )
        transpose = tuple(
            tuple(reference[column][row] for column in range(3))
            for row in range(3)
        )
        relative = self._matmul3(transpose, current)
        roll_deg = math.degrees(math.atan2(relative[1][0], relative[1][1]))
        pitch_deg = math.degrees(
            math.asin(max(-1.0, min(1.0, relative[0][2])))
        )
        return roll_deg, pitch_deg

    def _start_m01_combined_trajectory(self, state) -> bool:
        if self._mag_reference_pose is None or self._mag_reference_joints is None:
            self._fail_mag_auto_action("M01 缺少初始 Tool 姿态或关节参考")
            return False
        targets = self._m01_absolute_targets(self._mag_reference_pose)
        if len(targets) != 8:
            self._fail_mag_auto_action("M01 固定 XYZ 的 Ry/J6 组合轨迹无效")
            return False
        if not self.robot.set_speed_factor(MAG_M01_GLOBAL_SPEED_FACTOR):
            self._fail_mag_auto_action("M01 无法设置 100% 全局速度比例")
            return False
        self._mag_speed_factor = MAG_M01_GLOBAL_SPEED_FACTOR
        self._mag_speed_source = "M01 固定 XYZ 的 5 秒分段"
        previous_target = self._mag_reference_pose
        for index, (target, waypoint) in enumerate(
            zip(targets, MAG_M01_POSE_WAYPOINTS[1:])
        ):
            pitch_deg, j6_roll_deg = waypoint
            segment_angle_deg = self._orientation_error_deg(
                previous_target, target
            )
            velocity = max(
                1,
                min(100, round(segment_angle_deg / MAG_M01_SEGMENT_S)),
            )
            blend = MAG_M01_BLEND_PERCENT if index < len(targets) - 1 else 0
            if not self.robot.move_pose_l(
                target,
                velocity,
                user=state.user,
                tool=state.tool,
                acceleration_percent=30,
                blend_percent=blend,
            ):
                self._fail_mag_auto_action(
                    f"M01 固定 XYZ 的 Ry/J6 组合轨迹第 "
                    f"{index + 1}/{len(targets)} 段发送失败"
                )
                return False
            detail = (
                f"segment={index + 1}/{len(targets)}; "
                f"fixed_XYZ={target[0]:.3f},{target[1]:.3f},{target[2]:.3f}; "
                f"fixed_frame_pitch/Ry={pitch_deg:+.2f}deg; "
                f"J6_direction_roll/Rz={j6_roll_deg:+.2f}deg; "
                f"duration≈{MAG_M01_SEGMENT_S:.1f}s; "
                f"orientation_delta={segment_angle_deg:.2f}deg; "
                f"v={velocity}%; cp={blend}; source={self._mag_speed_source}"
            )
            self.coordinator.recorder.marker(
                "robot_auto_move_phase", "M01", detail
            )
            self._append_log(f"M01 固定 XYZ 的 Ry/J6 组合轨迹：{detail}", "info")
            previous_target = target
        self._auto_action_seen_motion = False
        return True

    def _start_mag_segment(self, step_id: str, index: int, state) -> bool:
        trajectory = MAG_TRAJECTORIES[step_id]
        if not 0 <= index < len(trajectory):
            self._fail_mag_auto_action(f"{step_id} 磁动作段号越界：{index}")
            return False
        axis, positive, _duration_s = trajectory[index]
        if self._mag_jog_active:
            if not self.robot.stop_tool_jog():
                self._fail_mag_auto_action(f"{step_id} 切换磁动作轴时停止失败")
                return False
            self._mag_jog_active = False
            self._mag_pending_segment_index = index
            self._mag_phase = "wait_segment_stop"
            self._mag_phase_started_ns = time.monotonic_ns()
            self._append_log(
                f"{step_id} 已停止上一段点动，等待机械臂静止后切换到 "
                f"Tool {axis}{'+' if positive else '-'}",
                "info",
            )
            return True
        rate = MAG_TARGET_RATE_DEG_S[step_id]
        target_rate = rate if positive else -rate
        if self._mag_speed_factor == 0:
            self._mag_speed_factor, self._mag_speed_source = (
                self._gyro_jog_speed_factor(axis, target_rate, state.tool)
            )
            command_ok = self.robot.start_tool_jog(
                axis,
                positive,
                self._mag_speed_factor,
                user=state.user,
                tool=state.tool,
            )
        else:
            command_ok = self.robot.switch_tool_jog(
                axis,
                positive,
                user=state.user,
                tool=state.tool,
            )
        if not command_ok:
            self._fail_mag_auto_action(
                f"{step_id} Tool {axis}{'+' if positive else '-'} 点动发送失败"
            )
            return False
        self._mag_jog_active = True
        self._mag_segment_index = index
        self._mag_pending_segment_index = -1
        self._auto_action_stable_since_ns = 0
        detail = (
            f"segment={index + 1}/{len(trajectory)}; axis={axis}; "
            f"direction={'+' if positive else '-'}; target={target_rate:+.1f}deg/s; "
            f"speed_factor={self._mag_speed_factor}%; "
            f"source={self._mag_speed_source}"
        )
        self.coordinator.recorder.marker("robot_auto_move_phase", step_id, detail)
        self._append_log(f"{step_id} 磁工单动作：{detail}", "info")
        return True

    def _restore_mag_speed_factor(self) -> None:
        if self._mag_original_speed_factor:
            self.robot.set_speed_factor(self._mag_original_speed_factor)
            self._mag_original_speed_factor = 0
        self._mag_speed_factor = 0
        self._mag_speed_source = ""

    def _stop_mag_jog(self, restore_speed: bool = True) -> None:
        if self._mag_jog_active:
            self.robot.stop_tool_jog()
            self._mag_jog_active = False
        if restore_speed:
            self._restore_mag_speed_factor()

    def _fail_mag_auto_action(self, message: str) -> None:
        self._stop_mag_jog(restore_speed=False)
        self.robot.stop()
        self._restore_mag_speed_factor()
        self._mag_phase = ""
        self._mag_phase_started_ns = 0
        self._mag_m04_motion_since_ns = 0
        self._mag_segment_index = -1
        self._mag_pending_segment_index = -1
        self._mag_reference_pose = None
        self._mag_reference_joints = None
        self._finish_auto_action(message, "error")
        if self.coordinator.running:
            self.coordinator.abort(message)

    def _mag_limit_error(self, step_id: str, state) -> str:
        if self._mag_reference_pose is None:
            return "磁翻转缺少标定中位参考姿态"
        if step_id == "M01":
            tool_rz_deg, ry_deg = self._m01_fixed_frame_angles(
                self._mag_reference_pose, state.pose
            )
            xyz_error_mm = max(
                abs(float(current) - float(reference))
                for current, reference in zip(
                    state.pose[:3], self._mag_reference_pose[:3]
                )
            )
            if xyz_error_mm > MAG_M01_XYZ_TOLERANCE_MM:
                return (
                    "M01 TCP XYZ 未按 G03/G04 方式保持："
                    f"最大单轴偏差 {xyz_error_mm:.2f} mm，"
                    f"允许 {MAG_M01_XYZ_TOLERANCE_MM:.1f} mm"
                )
            if self._mag_reference_joints is None or len(state.joints) < 6:
                return "M01 缺少 J6 中位参考或实时关节反馈"
            j6_delta_deg = (
                state.joints[5] - self._mag_reference_joints[5] + 180.0
            ) % 360.0 - 180.0
            if abs(j6_delta_deg) > MAG_M01_J6_SAFE_DEG + 5.0:
                return (
                    f"J6 往复越过相对中位安全范围：当前 {j6_delta_deg:+.2f}°，"
                    f"允许 ±{MAG_M01_J6_SAFE_DEG:.1f}°"
                )
            if abs(ry_deg) > MAG_M01_PITCH_SAFE_DEG + 5.0:
                return (
                    f"Pitch 摆动越过 M01 初始 Tool 参考系 Tool Ry 安全范围：当前 {ry_deg:+.2f}°，"
                    f"允许 ±{MAG_M01_PITCH_SAFE_DEG:.1f}°"
                )
            if abs(tool_rz_deg) > MAG_M01_J6_SAFE_DEG + 10.0:
                return (
                    "J6 叠加后的 Tool Rz 姿态越过保护范围："
                    f"当前 {tool_rz_deg:+.2f}°，允许 ±{MAG_M01_J6_SAFE_DEG + 10.0:.1f}°"
                )
        elif step_id in ("M02", "M03"):
            yaw_deg = self._relative_tool_axis_deg(
                self._mag_reference_pose, state.pose, "Rx"
            )
            margin_deg = 5.0
            lower_deg, upper_deg = (
                (-margin_deg, MAG_YAW_SAFE_DEG + margin_deg)
                if step_id == "M02"
                else (-MAG_YAW_SAFE_DEG - margin_deg, margin_deg)
            )
            if not lower_deg <= yaw_deg <= upper_deg:
                return (
                    f"Yaw 单侧往返越过 {step_id} 安全范围：当前 Tool Rx "
                    f"{yaw_deg:+.2f}°，允许 {lower_deg:+.1f}°～{upper_deg:+.1f}°"
                )
        return ""

    def _update_mag_auto_action(self, state) -> None:
        step_id = self._auto_action_step
        if step_id not in MAG_AUTO_STEPS:
            return
        now = time.monotonic_ns()
        self._check_auto_action_timeout(now)
        if self._auto_action_step != step_id:
            return
        limit_error = self._mag_limit_error(step_id, state)
        if limit_error and self._mag_phase not in ("wait_jog_stop", "return"):
            self._fail_mag_auto_action(f"{step_id} {limit_error}")
            return
        phase = self._mag_phase
        if phase == "static_wait_stop":
            if state.mode in (9, 11):
                self._fail_mag_auto_action(
                    f"M04 等待停稳时机械臂进入异常模式：mode={state.mode}"
                )
                return
            if (
                state.mode != 5
                or state.linear_speed_norm > 1.0
                or state.angular_speed_norm > 0.8
            ):
                self._auto_action_stable_since_ns = 0
                return
            if self._auto_action_stable_since_ns == 0:
                self._auto_action_stable_since_ns = now
                return
            if now - self._auto_action_stable_since_ns < int(MAG_M04_STOP_SETTLE_S * 1_000_000_000):
                return
            self._mag_phase = "static_settle"
            self._mag_phase_started_ns = now
            self._append_log(
                f"M04 机械臂已连续停稳 {MAG_M04_STOP_SETTLE_S:.1f} 秒，"
                "开始正式静止 5 秒",
                "good",
            )
            return
        if phase == "static_settle":
            if (
                state.mode != 5
                or state.linear_speed_norm > 1.0
                or state.angular_speed_norm > 0.8
            ):
                if state.mode in (9, 11):
                    self._fail_mag_auto_action(
                        f"M04 中位静止时机械臂进入异常模式：mode={state.mode}"
                    )
                    return
                self._mag_phase = "static_recover"
                self._mag_phase_started_ns = now
                self._mag_m04_motion_since_ns = now
                self._auto_action_stable_since_ns = 0
                self._append_log(
                    "M04 静止计时检测到速度波动，已清零并重新等待停稳："
                    f"mode={state.mode}，线速度={state.linear_speed_norm:.2f} mm/s，"
                    f"角速度={state.angular_speed_norm:.2f}°/s",
                    "warn",
                )
                return
            hold_s = self.coordinator.current_step.exit_s
            elapsed_s = (now - self._mag_phase_started_ns) / 1_000_000_000.0
            if elapsed_s < hold_s:
                return
            self.coordinator.condition_stable_since_ns = now - 600_000_000
            self._mag_phase = "wait_stage_open"
            if not self.coordinator.confirm_current_action():
                self._fail_mag_auto_action("M04 中位静止后的阶段标记复检未通过")
            else:
                self._append_log(
                    "M04 已静止满 5 秒，正在发送不采样的阶段标记", "good"
                )
            return
        if phase == "static_recover":
            if state.mode in (9, 11):
                self._fail_mag_auto_action(
                    f"M04 恢复等待时机械臂进入异常模式：mode={state.mode}"
                )
                return
            moving = (
                state.mode != 5
                or state.linear_speed_norm > 1.0
                or state.angular_speed_norm > 0.8
            )
            if moving:
                self._auto_action_stable_since_ns = 0
                if self._mag_m04_motion_since_ns == 0:
                    self._mag_m04_motion_since_ns = now
                if now - self._mag_m04_motion_since_ns >= int(
                    MAG_M04_MOTION_GRACE_S * 1_000_000_000
                ):
                    self._fail_mag_auto_action(
                        "M04 静止窗口内检测到持续机械臂运动："
                        f"mode={state.mode}，线速度={state.linear_speed_norm:.2f} mm/s，"
                        f"角速度={state.angular_speed_norm:.2f}°/s，"
                        f"持续至少 {MAG_M04_MOTION_GRACE_S:.1f} 秒"
                    )
                return
            self._mag_m04_motion_since_ns = 0
            if self._auto_action_stable_since_ns == 0:
                self._auto_action_stable_since_ns = now
                return
            if now - self._auto_action_stable_since_ns < int(
                MAG_M04_STOP_SETTLE_S * 1_000_000_000
            ):
                return
            self._mag_phase = "static_settle"
            self._mag_phase_started_ns = now
            self._append_log(
                f"M04 已重新连续停稳 {MAG_M04_STOP_SETTLE_S:.1f} 秒，"
                "重新开始静止 5 秒",
                "good",
            )
            return
        if phase == "lead_in":
            axis, positive, _duration_s = MAG_TRAJECTORIES[step_id][
                self._mag_segment_index
            ]
            axis_index = {"Rx": 0, "Ry": 1, "Rz": 2}[axis]
            rate = MAG_TARGET_RATE_DEG_S[step_id]
            target_rate = rate if positive else -rate
            tool_rate = self.coordinator._tool_angular_speed(
                state.pose, state.angular_speed
            )[axis_index]
            rate_ok = abs(tool_rate - target_rate) <= MAG_RATE_TOLERANCE_DEG_S
            if rate_ok:
                if self._auto_action_stable_since_ns == 0:
                    self._auto_action_stable_since_ns = now
            else:
                self._auto_action_stable_since_ns = 0
            if (
                self._auto_action_stable_since_ns
                and now - self._auto_action_stable_since_ns >= GYRO_SPEED_STABLE_NS
            ):
                self.coordinator.condition_stable_since_ns = now - 400_000_000
                if not self.coordinator.confirm_current_action():
                    self._fail_mag_auto_action(
                        f"{step_id} 磁动作稳定后的阶段开启复检未通过"
                    )
                    return
                self._mag_phase = "wait_stage_open"
                self._append_log(
                    f"{step_id} Tool {axis} 实测 {tool_rate:+.2f}°/s，正在开启磁采集",
                    "good",
                )
            return
        if phase == "wait_stage_open":
            return
        if phase == "combined_capturing":
            if self.coordinator.state != RunState.CAPTURING:
                return
            elapsed_s = max(
                0.0,
                (now - self.coordinator.capture_started_ns) / 1_000_000_000.0,
            )
            moving = state.mode in (7, 8) or state.angular_speed_norm > 0.8
            if moving:
                self._auto_action_seen_motion = True
                return
            if not self._auto_action_seen_motion and elapsed_s > 2.0:
                self._fail_mag_auto_action("M01 组合轨迹发送后机械臂未开始运动")
                return
            if (
                self._auto_action_seen_motion
                and elapsed_s < self.coordinator.current_step.capture_s - 1.0
            ):
                self._fail_mag_auto_action(
                    f"M01 组合轨迹在 {elapsed_s:.1f} 秒提前停止"
                )
            return
        if phase == "wait_segment_stop":
            if now - self._mag_phase_started_ns > 3_000_000_000:
                self._fail_mag_auto_action(
                    f"{step_id} 停止上一段点动后 3 秒内未进入静止状态"
                )
                return
            if (
                state.mode != 5
                or state.linear_speed_norm > 1.0
                or state.angular_speed_norm > 0.8
            ):
                return
            index = self._mag_pending_segment_index
            self._mag_phase = "capturing"
            self._mag_phase_started_ns = now
            self._start_mag_segment(step_id, index, state)
            return
        if phase == "capturing":
            if self.coordinator.state != RunState.CAPTURING:
                return
            elapsed_s = max(
                0.0,
                (now - self.coordinator.capture_started_ns) / 1_000_000_000.0,
            )
            desired_index = len(MAG_TRAJECTORIES[step_id]) - 1
            cumulative_s = 0.0
            for index, (_axis, _positive, duration_s) in enumerate(
                MAG_TRAJECTORIES[step_id]
            ):
                cumulative_s += duration_s
                if elapsed_s < cumulative_s:
                    desired_index = index
                    break
            if desired_index != self._mag_segment_index:
                self._start_mag_segment(step_id, desired_index, state)
            return
        if phase == "static_capturing":
            return
        if phase == "wait_jog_stop":
            if state.mode != 5 or state.angular_speed_norm > 0.8:
                return
            self._restore_mag_speed_factor()
            if (
                self.coordinator.state != RunState.READY
                or self.coordinator.current_step.step_id == step_id
            ):
                return
            neutral = self.taught_poses.get("neutral")
            if neutral is None:
                self._fail_mag_auto_action(f"{step_id} 缺少示教标定中位")
                return
            orientation_error = self._orientation_error_deg(neutral.pose, state.pose)
            self._mag_phase = "return"
            self._mag_phase_started_ns = now
            self._auto_action_seen_motion = False
            if orientation_error <= MAG_NEUTRAL_TOLERANCE_DEG:
                self._finish_mag_return(step_id, orientation_error)
                return
            self._append_log(
                f"{step_id} 采集完成，当前与中位偏差 {orientation_error:.2f}°，"
                "开始回示教标定中位",
                "info",
            )
            if not self.robot.move_pose_j(
                neutral.pose,
                NEUTRAL_RETURN_SPEED_PERCENT,
                user=neutral.user,
                tool=neutral.tool,
            ):
                self._fail_mag_auto_action(f"{step_id} 回标定中位命令发送失败")
            return
        if phase == "return":
            if state.mode in (7, 8) or state.angular_speed_norm > 0.8:
                self._auto_action_seen_motion = True
                return
            if state.mode != 5 or state.linear_speed_norm > 1.0:
                return
            if (
                not self._auto_action_seen_motion
                and now - self._mag_phase_started_ns < 800_000_000
            ):
                return
            neutral = self.taught_poses.get("neutral")
            if neutral is None:
                self._fail_mag_auto_action(f"{step_id} 缺少示教标定中位")
                return
            orientation_error = self._orientation_error_deg(neutral.pose, state.pose)
            if orientation_error > MAG_NEUTRAL_TOLERANCE_DEG:
                self._fail_mag_auto_action(
                    f"{step_id} 回标定中位后姿态偏差 {orientation_error:.2f}°，"
                    f"允许 {MAG_NEUTRAL_TOLERANCE_DEG:.1f}°"
                )
                return
            self._finish_mag_return(step_id, orientation_error)

    def _finish_mag_return(self, step_id: str, orientation_error: float) -> None:
        step = next(step for step in self.coordinator.steps if step.step_id == step_id)
        duration_s = step.exit_s if step_id == "M04" else step.capture_s
        self._mag_phase = ""
        self._mag_phase_started_ns = 0
        self._mag_segment_index = -1
        self._mag_pending_segment_index = -1
        self._mag_reference_pose = None
        self._mag_reference_joints = None
        self._finish_auto_action(
            f"{step_id} 已完成 {duration_s:.0f} 秒工单动作并回到标定中位，"
            f"姿态偏差 {orientation_error:.2f}°",
            "good",
        )

    def _update_accel_auto_action(self, state) -> None:
        step_id = self._auto_action_step
        if step_id not in self.robot_actions:
            return
        now = time.monotonic_ns()
        self._check_auto_action_timeout(now)
        if self._auto_action_step is None:
            return
        elapsed_s = (now - self._auto_action_started_ns) / 1_000_000_000.0
        if state.mode in (7, 8) or state.angular_speed_norm > 0.8:
            self._auto_action_seen_motion = True
            self._auto_action_stable_since_ns = 0
            return
        if state.mode != 5 or state.linear_speed_norm > 1.0 or state.angular_speed_norm > 0.8:
            self._auto_action_stable_since_ns = 0
            return
        if not self._auto_action_seen_motion and elapsed_s < 0.8:
            return
        alignment = self.coordinator._accel_face_alignment(step_id, state.pose)
        if alignment is None:
            self._finish_auto_action(f"{step_id} 到位后无法计算姿态偏差", "error")
            return
        angle_deg, gravity_tool, face_name = alignment
        if angle_deg > self.coordinator.ACCEL_FACE_MAX_DEG:
            self._finish_auto_action(
                f"{step_id} 自动旋转已停止，但实测偏差 {angle_deg:.2f}° 仍超过 5.0°；"
                "请在第 4 页调整旋转轴或角度后重试",
                "error",
            )
            return
        if self._auto_action_stable_since_ns == 0:
            self._auto_action_stable_since_ns = now
            self._append_log(
                f"{step_id} 已到位：{self.coordinator._format_accel_alignment(face_name, angle_deg, gravity_tool)}；"
                "开始连续静止 2 秒确认",
                "good",
            )
            return
        settle_s = self.coordinator.current_step.settle_s
        if now - self._auto_action_stable_since_ns < int(settle_s * 1_000_000_000):
            return
        self.coordinator.condition_stable_since_ns = now - int(settle_s * 1_000_000_000)
        self._finish_auto_action(f"{step_id} 到位并稳定 {settle_s:.1f} 秒，正在开启采集", "good")
        if not self.coordinator.confirm_current_action():
            self._show_error(f"{step_id} 到位后阶段开启前的最终条件复检未通过", False)

    @staticmethod
    def _pose_text(taught: TaughtPose) -> str:
        tcp = ", ".join(f"{value:.3f}" for value in taught.pose)
        joints = ", ".join(f"{value:.3f}" for value in taught.joints) if taught.joints else "历史记录未包含关节角"
        return (
            f"TCP X/Y/Z/Rx/Ry/Rz：{tcp}\n"
            f"J1～J6：{joints}\n"
            f"User {taught.user} / Tool {taught.tool}\n"
            f"示教时间：{taught.recorded_at.replace('T', ' ')}"
        )

    def _refresh_taught_pose_label(self) -> None:
        pose_buttons = {
            "safe": (self.robot_teach_safe, self.robot_safe),
            "neutral": (self.robot_teach_neutral, self.robot_neutral),
        }
        for name, title in (("safe", "安全位"), ("neutral", "标定中位")):
            taught = self.taught_poses.get(name)
            if taught is None:
                tooltip = f"{title}尚未示教"
            else:
                tooltip = f"{title}\n{self._pose_text(taught)}"
            for button in pose_buttons[name]:
                button.setToolTip(tooltip)

    def _teach_pose(self, name: str) -> None:
        if self.coordinator.running:
            self._show_error("标定会话进行中禁止修改示教位", True)
            return
        state = self.robot.latest_state
        if not self.robot.connected or state is None:
            self._show_error("尚未收到机械臂实时状态，不能示教", True)
            return
        age_s = (time.monotonic_ns() - state.received_monotonic_ns) / 1_000_000_000.0
        if age_s > 1.0:
            self._show_error(f"机械臂反馈已超时 {age_s:.1f} 秒，不能示教", True)
            return
        if state.mode != 5 or state.linear_speed_norm > 1.0 or state.angular_speed_norm > 0.8:
            self._show_error("示教要求机械臂已使能、处于空闲状态并且完全静止", True)
            return

        title = "安全位" if name == "safe" else "标定中位"
        taught = TaughtPose.from_robot_state(state)
        overwrite = "\n\n该位置已有记录，将被覆盖。" if name in self.taught_poses else ""
        warning = (
            "请确认当前位置远离碰撞、线束无拉扯，且已经低速验证运动余量。"
            if name == "safe"
            else "请确认从这里绕夹具 Tool X 可达设定安全限位、Tool Y/Z 可达 ±75°，且线束无拉扯。"
        )
        message = f"将当前位置记录为{title}。\n\n{self._pose_text(taught)}\n\n{warning}{overwrite}\n\n确认记录？"
        if QMessageBox.question(self, f"示教{title}", message) != QMessageBox.StandardButton.Yes:
            return
        updated = dict(self.taught_poses)
        updated[name] = taught
        try:
            save_pose_config(POSE_CONFIG_FILE, updated)
        except Exception as exc:
            self._show_error(f"保存{title}失败：{exc}", True)
            return
        self.taught_poses = updated
        self._refresh_taught_pose_label()
        self._update_action_controls()
        self._append_log(f"已示教{title}：User {taught.user} / Tool {taught.tool}", "good")

    def _return_taught_pose(self, name: str) -> None:
        if self.coordinator.running:
            self._show_error("标定会话进行中禁止单独发送回位命令", True)
            return
        taught = self.taught_poses.get(name)
        title = "安全位" if name == "safe" else "标定中位"
        if taught is None:
            self._show_error(f"尚未示教{title}", True)
            return
        state = self.robot.latest_state
        if not self.robot.connected or state is None or state.mode != 5:
            self._show_error(f"移动到{title}要求机械臂已连接、已使能且处于空闲状态", True)
            return
        velocity_percent = NEUTRAL_RETURN_SPEED_PERCENT if name == "neutral" else 15
        message = (
            f"机械臂将先启用 User {taught.user} / Tool {taught.tool}，"
            f"再以 {velocity_percent}% 速度移动到{title}。\n\n"
            f"{self._pose_text(taught)}\n\n请确认路径无碰撞、人员已离开运动区域。"
        )
        if QMessageBox.question(self, "确认机器人运动", message) != QMessageBox.StandardButton.Yes:
            return
        if not self.robot.activate_frames(taught.user, taught.tool):
            return
        if taught.joints:
            self.robot.move_joints(taught.joints, velocity_percent)
        else:
            self.robot.move_pose_j(
                taught.pose, velocity_percent, taught.user, taught.tool
            )

    def _return_safe_pose(self) -> None:
        self._return_taught_pose("safe")

    @staticmethod
    def _angle_distance_deg(left: float, right: float) -> float:
        return abs((float(left) - float(right) + 180.0) % 360.0 - 180.0)

    def _at_taught_pose(self, taught: TaughtPose) -> bool:
        state = self.robot.latest_state
        if state is None or state.user != taught.user or state.tool != taught.tool:
            return False
        if taught.joints:
            return max(
                self._angle_distance_deg(actual, target)
                for actual, target in zip(state.joints, taught.joints)
            ) <= 1.0
        position_error = max(abs(actual - target) for actual, target in zip(state.pose[:3], taught.pose[:3]))
        angle_error = max(
            self._angle_distance_deg(actual, target)
            for actual, target in zip(state.pose[3:], taught.pose[3:])
        )
        return position_error <= 2.0 and angle_error <= 1.0

    def _choose_output(self) -> None:
        selected = QFileDialog.getExistingDirectory(self, "选择 QuickCal 记录根目录", self.output_edit.text())
        if selected:
            self.output_edit.setText(selected)

    def _limits_from_ui(self, silent: bool = False) -> YawLimits | None:
        try:
            return YawLimits(
                negative_soft_limit_deg=float(self.yaw_negative.text()),
                positive_soft_limit_deg=float(self.yaw_positive.text()),
                safety_margin_deg=float(self.yaw_margin.text()),
                rate_deg_s=float(self.yaw_rate.text()),
                minimum_capture_s=float(self.yaw_min_capture.text()),
            )
        except (ValueError, AttributeError):
            if not silent:
                self._show_error("G01/G02 配置轴限位参数必须是有效数字", True)
            return None

    @Slot()
    def _refresh_limits(self) -> None:
        if self._building_limits:
            return
        limits = self._limits_from_ui(silent=True)
        if limits is None:
            self.limits_result.setText("参数格式错误")
            self.limits_result.setStyleSheet("color:#b42318;")
            return
        result = "通过" if limits.valid else "不通过"
        self.limits_result.setText(
            f"完整运动：{limits.negative_soft_limit_deg:+.1f}°～"
            f"{limits.positive_soft_limit_deg:+.1f}°\n"
            f"加速区：-55.0°→-45.0°｜减速区：+45.0°→+55.0°\n"
            f"反向动作与上述区间对称\n"
            f"匀速采集：{limits.negative_safe_deg:+.1f}°～"
            f"{limits.positive_safe_deg:+.1f}°，固定 {LIMITED_GYRO_CAPTURE_S:.1f} s\n"
            f"路径判定：{result}"
        )
        self.limits_result.setStyleSheet("color:#087f5b;" if limits.valid else "color:#b42318;")
        configured_steps = steps_for_limits(limits)
        if self.coordinator.SKIP_MAGNETIC_STAGES:
            displayed_total_s = sum(
                step.total_s
                for step in configured_steps
                if not step.step_id.startswith("M")
            )
            displayed_capture_s = sum(
                step.capture_s
                for step in configured_steps
                if not step.step_id.startswith("M")
            )
        else:
            displayed_total_s = expected_total_seconds(limits)
            displayed_capture_s = expected_capture_seconds(limits)
        self.timeline_summary.setText(
            f"预计工单总时长：{displayed_total_s:.1f} s\n"
            f"预计有效采集时间：{displayed_capture_s:.1f} s\n"
            "阶段与采集时间来自 QuickCal 15 dps 工单与 r024-fac-magq 协议。\n"
            + (
                "r024 无磁 QuickCal：正式阶段仅 P1/A01-A06/G01-G06，G06 后回中静止并自动提交。"
                if self.coordinator.SKIP_MAGNETIC_STAGES
                else "磁标定流程已启用：17 个正式阶段，G06 后继续执行 M01-M04。"
            )
        )
        self._populate_workflow()

    def _start_session(self) -> None:
        limits = self._limits_from_ui()
        if limits is None:
            return
        unavailable_actions = [
            step_id
            for step_id in (
                "A01", "A02", "A03", "A04", "A05", "A06",
                "G01", "G02", "G03", "G04", "G05", "G06",
            )
            if (
                (config := self._config_for_step(step_id)) is None
                or not config.enabled
            )
        ]
        if unavailable_actions:
            self._show_error(
                "开始全自动校准前必须启用 A01-A06 和 G01-G06 机械臂动作："
                + "、".join(unavailable_actions),
                True,
            )
            return
        if "safe" not in self.taught_poses or "neutral" not in self.taught_poses:
            self._show_error("开始标定前必须先示教安全位和标定中位", True)
            return
        neutral = self.taught_poses["neutral"]
        if not self._at_taught_pose(neutral):
            self._show_error(
                "机械臂当前不在已示教的标定中位（容差：关节 1°）。请先点击“到标定中位”，等待机器人停止后再开始。",
                True,
            )
            return
        try:
            gyro_motion_map = {}
            for step_id in GYRO_AUTO_STEPS:
                axis, _lead_in, _capture_start, target_rate = (
                    self._gyro_motion_parameters(step_id)
                )
                gyro_motion_map[step_id] = (axis, target_rate)
            self.coordinator.configure(
                self.sn_edit.text(),
                self.station_edit.text(),
                self.operator_edit.text(),
                Path(self.output_edit.text()),
                limits,
                self.environment_check.isChecked(),
                neutral_pose=neutral.pose,
                gyro_motion_map=gyro_motion_map,
            )
        except Exception as exc:
            self._show_error(str(exc), True)
            return
        self._populate_workflow()
        self._full_auto_enabled = True
        if not self.coordinator.start_session():
            self._stop_full_auto()
            return
        self._full_auto_timer.start()
        self._append_log(
            "全自动流程已启动：后续步骤将按顺序自动执行；任一步门控或 ACK 失败都会立即停止",
            "good",
        )

    @Slot(bool, str)
    def _robot_connection_changed(self, connected: bool, detail: str) -> None:
        self.robot_connect.setText("断开" if connected else "连接")
        if not connected:
            if self._manual_motion_target is not None:
                self._finish_manual_motion(
                    "手动运动失败：机械臂连接中断", "error"
                )
            self._robot_enable_pending = None
            self._robot_enable_timeout.stop()
            self._manual_absolute_pose_initialized = False
            self._manual_absolute_pose_frame = None
        self._update_robot_enable_button()
        self._set_badge(
            self.robot_badge,
            "机械臂已连接，等待状态" if connected else "机械臂未连接",
            "warn" if connected else "bad",
        )
        self._update_action_controls()

    @Slot(bool, str)
    def _glove_connection_changed(self, connected: bool, detail: str) -> None:
        self.glove_connect.setText("断开" if connected else "连接")
        self._set_badge(self.glove_badge, "手套已连接" if connected else "手套未连接", "ok" if connected else "bad")
        self._ui_raw_imu_frame = None
        self._ui_raw_imu_ns = 0
        self._ui_register_imu_frame = None
        self._ui_register_imu_ns = 0
        for row in range(len(IMU_NAMES)):
            for column in range(2, 6):
                self.imu_table.item(row, column).setText("--")
        self._refresh_imu_online_status()
        self._update_action_controls()

    @Slot(object)
    def _on_robot_state(self, state) -> None:
        if (
            hasattr(self, "manual_position_spins")
            and (
                not self._manual_absolute_pose_initialized
                or self._manual_absolute_pose_frame != (int(state.user), int(state.tool))
            )
        ):
            self._load_current_pose_into_manual_controls()
        pose = ", ".join(f"{value:.1f}" for value in state.pose)
        state_text = f"mode={state.mode}｜TCP=({pose})｜|ω|={state.angular_speed_norm:.2f}°/s"
        alignment = self.coordinator._accel_face_alignment(
            self.coordinator.current_step.step_id, state.pose
        )
        if alignment is not None:
            angle_deg, gravity_tool, face_name = alignment
            gate = (
                "合格"
                if angle_deg <= self.coordinator.ACCEL_FACE_WARNING_DEG
                else "警告"
                if angle_deg <= self.coordinator.ACCEL_FACE_MAX_DEG
                else "禁止采集"
            )
            state_text += f"｜六面 {face_name} 偏差={angle_deg:.2f}°（{gate}）"
            self.robot_state_label.setToolTip(
                self.coordinator._format_accel_alignment(face_name, angle_deg, gravity_tool)
            )
        else:
            self.robot_state_label.setToolTip("")
        self.robot_state_label.setText(state_text)
        if hasattr(self, "tool_motion_feedback"):
            motion_state = "静止" if state.angular_speed_norm <= 0.8 else "运动中"
            self.tool_motion_feedback.setText(
                f"U{state.user} / T{state.tool}｜{motion_state}"
            )
            self.tool_motion_feedback.setToolTip(
                f"TCP=({pose})｜实时角速度 |ω|={state.angular_speed_norm:.2f}°/s"
            )
            configured_tool = int(self.tool_offset_config["tool"])
            configured_offset = ", ".join(
                f"{float(value):.1f}" for value in self.tool_offset_config["offset"]
            )
            self.tool_config_status.setText(
                f"本地 Tool {configured_tool}=({configured_offset})｜"
                f"控制器当前反馈 Tool {state.tool}"
            )
            self._update_manual_motion_result(state)
        enabled = self._robot_mode_is_enabled(state.mode)
        if self._robot_enable_pending is not None and enabled == self._robot_enable_pending:
            self._robot_enable_pending = None
            self._robot_enable_timeout.stop()
        self._update_robot_enable_button()
        if state.mode == 9:
            self._set_badge(self.robot_badge, "机械臂报警", "bad")
        elif state.mode == 11:
            self._set_badge(self.robot_badge, "机械臂碰撞", "bad")
        elif enabled:
            self._set_badge(self.robot_badge, "机械臂已使能", "ok")
        else:
            self._set_badge(self.robot_badge, "机械臂未使能", "warn")
        if self._auto_action_step in GYRO_AUTO_STEPS:
            self._update_gyro_x_auto_action(state)
        elif self._auto_action_step in MAG_AUTO_STEPS:
            self._update_mag_auto_action(state)
        else:
            self._update_accel_auto_action(state)
        self._update_action_controls()

    @Slot(object)
    def _on_raw_imu(self, frame) -> None:
        self._ui_raw_imu_frame = frame
        self._ui_raw_imu_ns = time.monotonic_ns()
        for row, sample in enumerate(frame.samples):
            gyro_norm = math.sqrt(sample.gx * sample.gx + sample.gy * sample.gy + sample.gz * sample.gz)
            accel_norm = math.sqrt(sample.ax * sample.ax + sample.ay * sample.ay + sample.az * sample.az)
            self.imu_table.item(row, 2).setText(f"{gyro_norm:.3f}")
            self.imu_table.item(row, 3).setText(f"{accel_norm:.3f}")
        self._refresh_imu_online_status()

    @Slot(object)
    def _on_register_imu(self, frame) -> None:
        self._ui_register_imu_frame = frame
        self._ui_register_imu_ns = time.monotonic_ns()
        for row, sample in enumerate(frame.samples):
            self.imu_table.item(row, 4).setText(f"{sample.gx}/{sample.gy}/{sample.gz}")
            self.imu_table.item(row, 5).setText(f"{sample.ax}/{sample.ay}/{sample.az}")
        self._refresh_imu_online_status()

    def _refresh_imu_online_status(self, now_ns: int | None = None) -> None:
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        if not self.glove.is_open:
            text, color = "未连接", "#526173"
            for row in range(len(IMU_NAMES)):
                item = self.imu_table.item(row, 1)
                item.setText(text)
                item.setForeground(QColor(color))
                item.setToolTip("USB CDC 串口尚未连接")
            self._set_badge(self.imu_badge, "IMU 未连接", "bad")
            return
        missing_streams = []
        if self._ui_raw_imu_frame is None or self._ui_raw_imu_ns == 0:
            missing_streams.append("type=9")
        if self._ui_register_imu_frame is None or self._ui_register_imu_ns == 0:
            missing_streams.append("type=11")
        if missing_streams:
            stream_text = "/".join(missing_streams)
            text, color = f"等待 {stream_text}", "#b45309"
            for row in range(len(IMU_NAMES)):
                item = self.imu_table.item(row, 1)
                item.setText(text)
                item.setForeground(QColor(color))
                item.setToolTip(f"r024 预检要求同时收到新鲜的 type=9 和 type=11；当前缺少 {stream_text}")
            self._set_badge(self.imu_badge, f"等待 {stream_text}", "warn")
            return
        raw_age_s = (now_ns - self._ui_raw_imu_ns) / 1_000_000_000.0
        register_age_s = (now_ns - self._ui_register_imu_ns) / 1_000_000_000.0
        if max(raw_age_s, register_age_s) >= self.coordinator.RAW_FRESH_NS / 1_000_000_000.0:
            for row in range(len(IMU_NAMES)):
                item = self.imu_table.item(row, 1)
                item.setText("数据超时")
                item.setForeground(QColor("#b42318"))
                item.setToolTip(
                    f"type=9 龄期 {raw_age_s:.2f}s；type=11 龄期 {register_age_s:.2f}s；允许 <0.80s"
                )
            self._set_badge(self.imu_badge, "IMU 数据超时", "bad")
            return
        raw_mask = self._ui_raw_imu_frame.presence_mask & ALL_IMU_MASK
        register_mask = self._ui_register_imu_frame.presence_mask & ALL_IMU_MASK
        presence_mask = raw_mask & register_mask
        for row in range(len(IMU_NAMES)):
            online = bool(presence_mask & (1 << row))
            item = self.imu_table.item(row, 1)
            item.setText("在线" if online else "离线")
            item.setForeground(QColor("#087f5b" if online else "#dc2626"))
            item.setToolTip(
                f"type=9 mask=0x{raw_mask:04X}, stage=0x{self._ui_raw_imu_frame.stage_id:02X}, "
                f"capture=0x{self._ui_raw_imu_frame.capture_mask:02X}；"
                f"type=11 mask=0x{register_mask:04X}, stage=0x{self._ui_register_imu_frame.stage_id:02X}, "
                f"capture=0x{self._ui_register_imu_frame.capture_mask:02X}"
            )
        online_count = presence_mask.bit_count()
        self._set_badge(self.imu_badge, f"IMU {online_count}/11", "ok" if online_count == 11 else "bad")

    @Slot(object)
    def _on_version(self, frame) -> None:
        self.glove_version_label.setText(f"固件：{frame.revision_tag}｜{frame.imu_model}｜{frame.hand_side}")
        self._append_log(
            f"固件 {frame.revision_tag}，revision={frame.revision}，features=0x{frame.features:02X}，"
            f"原始流能力={'有' if frame.factory_raw_streams else '无'}，"
            f"磁标定能力={'有' if frame.magnetic_factory else '无'}，构建 {frame.build_date} {frame.build_time}"
        )

    @staticmethod
    def _format_report_summary(report) -> str:
        gyro_ok_count = sum(
            1
            for item in report.gyro_quality
            if item.ok
            and item.reject_flags == 0
            and item.window_count >= EXPECTED_GYRO_SEGMENTS
        )
        accel_ok_count = sum(1 for item in report.accel_quality if item.ok)
        imu_count = report.imu_count

        if report.factory_pass:
            outcome = "通过"
        elif report.status == 0:
            outcome = "未通过：报告质量项不完整"
        else:
            reason = MCAL_STATUS_LABELS.get(report.status, "固件报告失败")
            outcome = f"未通过：{reason}"

        gyro_text = f"Gyro={gyro_ok_count}/{imu_count}"
        if report.calibrated_count != gyro_ok_count:
            gyro_text += f"（报告头 nCal={report.calibrated_count}）"
        accel_text = f"Accel={accel_ok_count}/{imu_count}"
        anchor = report.mag_quality.slots[0] if report.mag_quality.slots else None
        if report.mag_all_ok:
            mag_text = "Mag=通过"
        else:
            reasons = "、".join(report.mag_quality.reject_reasons) or "质量判定失败"
            mag_text = f"Mag=失败（{reasons}）"
        if anchor:
            mag_text += (
                f" samples={anchor.sample_count}，span=({anchor.span_x},{anchor.span_y},{anchor.span_z})，"
                f"offset=({anchor.offset_x},{anchor.offset_y},{anchor.offset_z})，"
                f"scale=({anchor.scale_x1000 / 1000:.3f},{anchor.scale_y1000 / 1000:.3f},"
                f"{anchor.scale_z1000 / 1000:.3f})"
            )

        if report.status == 0:
            flash_text = f"Flash 已写入（calSeq={report.flash_sequence}）"
        else:
            flash_text = f"本次 Flash 未写入｜当前 calSeq={report.flash_sequence}"
        rms_text = (
            f"平均 RMS={report.mean_rms_mdeg / 1000:.3f}°"
            if report.gyro_all_ok
            else "平均 RMS=--"
        )
        return (
            f"报告 v{report.version}｜{outcome}（status={report.status}）｜"
            f"{gyro_text}｜{accel_text}｜{mag_text}｜{flash_text}｜{rms_text}"
        )

    @Slot(object)
    def _on_report(self, report) -> None:
        self.report_summary.setText(self._format_report_summary(report))
        # ``Gyro拒绝原因`` only carries information when a gyro channel is
        # rejected.  Hiding it for an all-pass report makes the combined
        # IMU/Mag table substantially easier to scan, while keeping it
        # available automatically for every failed or incomplete report.
        all_gyro_passed = len(report.gyro_quality) >= len(IMU_NAMES) and all(
            gyro.ok
            and gyro.reject_flags == 0
            and gyro.window_count >= EXPECTED_GYRO_SEGMENTS
            for gyro in report.gyro_quality[: len(IMU_NAMES)]
        )
        self.report_table.setColumnHidden(2, all_gyro_passed)
        mag = report.mag_quality
        mag_reasons = "；".join(mag.reject_reasons)
        mag_tooltip = (
            f"seenMask=0x{mag.seen_mask:02X}；rejectFlags=0x{mag.reject_flags:02X}；"
            f"{mag_reasons or '无拒绝原因'}"
        )
        for slot_index, sensor_name in enumerate(MAG_REPORT_SENSOR_NAMES):
            row = len(IMU_NAMES) + slot_index
            slot = mag.slots[slot_index] if slot_index < len(mag.slots) else None
            seen = bool(mag.seen_mask & (1 << slot_index))
            if slot is None:
                values = ("--", "--", "--", "--", "报告未包含该槽位")
            elif slot_index > 0 and not seen:
                # Disabled BMM350 slots remain present in the fixed-size v4
                # report.  Their offset/scale bytes are not valid calibration
                # results, so do not present those placeholders as measurements.
                values = ("—", "—", "—", "—", "未启用")
            else:
                if slot_index == 0:
                    quality = "通过" if report.mag_all_ok else (mag_reasons or "失败")
                else:
                    quality = "已采样（旁路）" if seen else "未启用/未检测"
                values = (
                    str(slot.sample_count),
                    f"{slot.span_x} / {slot.span_y} / {slot.span_z}",
                    f"{slot.offset_x} / {slot.offset_y} / {slot.offset_z}",
                    f"{slot.scale_x1000 / 1000:.3f} / {slot.scale_y1000 / 1000:.3f} / {slot.scale_z1000 / 1000:.3f}",
                    quality,
                )
            self.report_table.item(row, 0).setText(sensor_name)
            for column, value in enumerate(values, 8):
                item = self.report_table.item(row, column)
                item.setText(value)
                item.setToolTip(mag_tooltip)
                if column == 12:
                    if slot_index == 0:
                        item.setForeground(
                            QColor("#087f5b" if report.mag_all_ok else "#dc2626")
                        )
                    else:
                        item.setForeground(QColor("#087f5b" if seen else "#667085"))
        for row in range(11):
            gyro = report.gyro_quality[row] if row < len(report.gyro_quality) else None
            accel = report.accel_quality[row] if row < len(report.accel_quality) else None
            gyro_passed = bool(
                gyro
                and gyro.ok
                and gyro.reject_flags == 0
                and gyro.window_count >= EXPECTED_GYRO_SEGMENTS
            )
            gyro_reasons = list(gyro.reject_reasons) if gyro else []
            if (
                gyro
                and gyro.window_count < EXPECTED_GYRO_SEGMENTS
                and "有效窗口不足" not in gyro_reasons
            ):
                gyro_reasons.append("有效窗口不足")
            values = (
                "通过" if gyro_passed else "失败",
                "、".join(gyro_reasons) if gyro_reasons else ("—" if gyro else "--"),
                f"{gyro.rms_mdeg / 1000:.3f}" if gyro else "--",
                str(gyro.window_count) if gyro else "--",
                f"{gyro.max_off_axis / 1000:.3f}" if gyro else "--",
                "通过" if accel and accel.ok else "失败",
                (
                    f"残差={accel.residual_x1000 / 1000:.3f}；"
                    f"比例={accel.max_abs_scale_error_x1000 / 1000:.3f}；"
                    f"交叉轴={accel.max_cross_axis_x1000 / 1000:.3f}；"
                    f"Bias=({accel.bias_x_mg},{accel.bias_y_mg},{accel.bias_z_mg})mg"
                    if accel
                    else "--"
                ),
            )
            for column, value in enumerate(values, 1):
                self.report_table.item(row, column).setText(value)
            passed = bool(gyro_passed and accel and accel.ok)
            self.imu_table.item(row, 6).setText("通过" if passed else "失败")
            self.imu_table.item(row, 6).setForeground(QColor("#087f5b" if passed else "#dc2626"))

    @Slot(str)
    def _on_run_state(self, state: str) -> None:
        if self._auto_action_step in GYRO_AUTO_STEPS:
            if (
                state == RunState.CAPTURING.value
                and self._gyro_x_phase == "wait_stage_open"
            ):
                self._gyro_x_phase = "capturing"
            elif state == RunState.WAIT_STAGE_CLOSE.value and self._gyro_x_phase in (
                "wait_stage_open",
                "capturing",
            ):
                # The fitted six-second window has ended. Start reducing jog
                # speed immediately so the complete move remains inside the
                # fixed ten-degree outer deceleration region while the close
                # ACK is in flight.
                self._gyro_x_phase = "smooth_decel"
                self._gyro_x_phase_started_ns = time.monotonic_ns()
                if not self._set_gyro_x_decel_level(0):
                    self._fail_gyro_x_auto_action(
                        f"{self._auto_action_step} 无法开始丝滑减速，已停止机械臂"
                    )
        elif self._auto_action_step in MAG_AUTO_STEPS:
            if (
                state == RunState.CAPTURING.value
                and self._mag_phase == "wait_stage_open"
            ):
                step_id = self._auto_action_step
                self._mag_phase = (
                    "static_capturing"
                    if step_id == "M04"
                    else "combined_capturing"
                    if step_id == "M01"
                    else "capturing"
                )
                self._mag_phase_started_ns = time.monotonic_ns()
                if step_id != "M04":
                    latest_state = self.robot.latest_state
                    if latest_state is None:
                        self._fail_mag_auto_action(
                            f"{step_id} 阶段已开启，但缺少机械臂实时状态"
                        )
                    elif step_id == "M01":
                        self._start_m01_combined_trajectory(latest_state)
                    else:
                        self._start_mag_segment(step_id, 0, latest_state)
            elif state == RunState.WAIT_STAGE_CLOSE.value and self._mag_phase in (
                "wait_stage_open",
                "capturing",
                "combined_capturing",
                "static_capturing",
                "wait_segment_stop",
            ):
                if self._auto_action_step == "M01":
                    self.robot.stop()
                else:
                    self._stop_mag_jog(restore_speed=False)
                self._mag_phase = "wait_jog_stop"
                self._mag_phase_started_ns = time.monotonic_ns()
        if self._auto_action_step is not None and state in (
            RunState.ABORTED.value,
            RunState.COMPLETE.value,
        ):
            step_id = self._auto_action_step
            if step_id in GYRO_AUTO_STEPS:
                self._stop_gyro_x_jog()
                self._gyro_x_phase = ""
                self._gyro_x_reference_pose = None
            elif step_id in MAG_AUTO_STEPS:
                self._stop_mag_jog(restore_speed=False)
                self._restore_mag_speed_factor()
                self._mag_phase = ""
                self._mag_phase_started_ns = 0
                self._mag_segment_index = -1
                self._mag_pending_segment_index = -1
                self._mag_reference_pose = None
                self._mag_reference_joints = None
            self._finish_auto_action(f"会话已结束，{step_id} 自动动作状态已清除", "error")
        badge_state = "ok" if state == RunState.COMPLETE.value else "warn" if state in (RunState.IDLE.value, RunState.READY.value) else "bad" if state == RunState.ABORTED.value else "warn"
        self._set_badge(self.session_badge, state, badge_state)
        self._update_action_controls()

    @Slot(int, object)
    def _on_current_step(self, index: int, step) -> None:
        self.workflow_table.selectRow(index)
        self.workflow_table.scrollToItem(self.workflow_table.item(index, 0))
        self.instruction_label.setText(
            f"{step.step_id}｜{step.name}\n机械臂：{step.robot_action}\n开始条件：{step.start_condition}\n通过条件：{step.pass_condition}"
        )
        action_config = self._config_for_step(step.step_id)
        self.confirm_button.setText(
            "正在自动检测 P1 静止条件"
            if step.step_id == "P1"
            else "自动复原并执行 G01"
            if step.step_id == "G01"
            else (
                f"自动执行 {step.step_id} Tool "
                f"{action_config.axis}"
                f"{'+' if action_config.degrees > 0 else '-'} 扫转"
            )
            if step.step_id in GYRO_AUTO_STEPS and action_config is not None
            else f"{step.step_id} 轴映射配置无效"
            if step.step_id in GYRO_AUTO_STEPS
            else MAG_STEP_BUTTON_TEXT[step.step_id]
            if step.step_id in MAG_AUTO_STEPS
            else "求解并写入设备"
            if step.step_id == "S01"
            else "完成归档"
            if step.step_id == "S02"
            else f"自动旋转并执行 {step.step_id}"
            if step.step_id in self.robot_actions
            and action_config is not None
            and action_config.enabled
            else "确认动作条件，执行本步"
        )
        self._update_action_controls()
        if self._full_auto_enabled:
            QTimer.singleShot(0, self._try_start_full_auto_step)

    @Slot(int, str, str)
    def _on_step_status(self, row: int, status: str, detail: str) -> None:
        item = self.workflow_table.item(row, 5)
        item.setText(status)
        color = {"完成": "#087f5b", "失败": "#dc2626", "进行中": "#b45309"}.get(status, "#475467")
        item.setForeground(QColor(color))
        if detail:
            self.workflow_table.item(row, 6).setToolTip(detail)

    @Slot(int, str)
    def _on_progress(self, value: int, text: str) -> None:
        self.capture_progress.setValue(value)
        self.capture_progress.setFormat(f"{value}%｜{text}")

    @Slot(str, str)
    def _on_status_message(self, message: str, level: str) -> None:
        self._append_log(message, level)
        if level == "error":
            self.statusBar().showMessage(message, 10000)
        else:
            self.statusBar().showMessage(message, 5000)

    @Slot(bool, str)
    def _on_finished(self, passed: bool, reason: str) -> None:
        self._stop_full_auto()
        self._update_action_controls()
        QMessageBox.information(self, "QuickCal 完成" if passed else "QuickCal 未通过", reason)

    def _update_action_controls(self) -> None:
        state = self.coordinator.state
        idle = state in (RunState.IDLE, RunState.COMPLETE, RunState.ABORTED)
        self.start_button.setEnabled(idle)
        if self._full_auto_enabled:
            self.confirm_button.setText("全自动流程运行中")
        self.confirm_button.setEnabled(
            state == RunState.READY
            and self.coordinator.current_step.step_id != "P1"
            and self._auto_action_step is None
            and not self._full_auto_enabled
        )
        self.abort_button.setEnabled(self.coordinator.running)
        connection_controls = not self.coordinator.running
        for widget in (self.robot_connect, self.glove_connect, self.port_refresh, self.output_choose):
            widget.setEnabled(connection_controls)
        robot_state = self.robot.latest_state
        robot_mode_allows_toggle = bool(robot_state and robot_state.mode in (3, 4, 5))
        self.robot_enable.setEnabled(
            self.robot.connected
            and connection_controls
            and robot_mode_allows_toggle
            and self._robot_enable_pending is None
        )
        self._update_robot_enable_button()
        self.robot_clear.setEnabled(self.robot.connected)
        self.robot_stop.setEnabled(self.robot.connected)
        robot_feedback_fresh = bool(
            robot_state
            and time.monotonic_ns() - robot_state.received_monotonic_ns < 1_000_000_000
        )
        robot_idle = bool(robot_feedback_fresh and robot_state.mode == 5)
        robot_still = bool(
            robot_idle
            and robot_state.linear_speed_norm <= 1.0
            and robot_state.angular_speed_norm <= 0.8
        )
        self.robot_teach_safe.setEnabled(self.robot.connected and connection_controls and robot_still)
        self.robot_teach_neutral.setEnabled(self.robot.connected and connection_controls and robot_still)
        self.robot_safe.setEnabled(
            self.robot.connected and connection_controls and robot_idle and "safe" in self.taught_poses
        )
        self.robot_neutral.setEnabled(
            self.robot.connected and connection_controls and robot_idle and "neutral" in self.taught_poses
        )
        action_config_enabled = self._auto_action_step is None
        for widget in (
            self.action_step_combo,
            self.action_auto_enabled,
            self.action_rotation_axis,
            self.action_rotation_degrees,
            self.action_velocity,
            self.action_timeout,
            self.save_action_config_button,
        ):
            widget.setEnabled(action_config_enabled)
        if self.action_step_combo.currentText() in GYRO_AUTO_STEPS:
            # G-stage speed and timeout are governed by the fixed r024 scan
            # controller; this page intentionally exposes only axis/direction.
            self.action_velocity.setEnabled(False)
            self.action_timeout.setEnabled(False)
        manual_motion_pending = self._manual_motion_target is not None
        manual_page_editable = (
            connection_controls
            and self._auto_action_step is None
            and not manual_motion_pending
        )
        for widget in (
            self.manual_tool_index,
            *self.manual_tool_offset_spins.values(),
            self.manual_motion_velocity,
            *self.manual_position_spins.values(),
            *self.manual_rotation_spins.values(),
        ):
            widget.setEnabled(manual_page_editable)
        manual_motion_ready = bool(
            self.robot.connected and manual_page_editable and robot_still
        )
        self.apply_tool_config_button.setEnabled(manual_motion_ready)
        self.activate_tool_zero_button.setEnabled(manual_motion_ready)
        self.load_current_pose_button.setEnabled(manual_motion_ready)
        self.execute_translation_button.setEnabled(manual_motion_ready)
        self.execute_rotation_button.setEnabled(manual_motion_ready)
        self.glove_query.setEnabled(self.glove.is_open)
        for widget in (
            self.sn_edit,
            self.station_edit,
            self.operator_edit,
            self.output_edit,
            self.environment_check,
            self.yaw_negative,
            self.yaw_positive,
            self.yaw_margin,
            self.yaw_rate,
            self.yaw_min_capture,
        ):
            widget.setEnabled(connection_controls)

    def _refresh_health(self) -> None:
        now = time.monotonic_ns()
        self._check_auto_action_timeout(now)
        self._check_manual_motion_watchdog(now)
        self._refresh_action_preview()
        self._refresh_imu_online_status(now)
        raw_age = (now - self._ui_raw_imu_ns) / 1e9 if self._ui_raw_imu_ns else math.inf
        reg_age = (now - self._ui_register_imu_ns) / 1e9 if self._ui_register_imu_ns else math.inf
        robot_age = (now - self.coordinator.latest_robot_state.received_monotonic_ns) / 1e9 if self.coordinator.latest_robot_state else math.inf
        stats = self.glove.parser.stats
        self.health_label.setText(
            f"数据流：T9 {'正常' if raw_age < 0.8 else '等待'}｜"
            f"T11 {'正常' if reg_age < 0.8 else '等待'}｜"
            f"机械臂 {'正常' if robot_age < 1.0 else '等待'}　"
            f"帧：有效 {stats.accepted_frames}｜异常 {stats.invalid_frames}｜丢序 {stats.sequence_gaps}"
        )
        self.health_label.setToolTip(
            f"type=9 最近数据：{f'{raw_age:.2f}s' if math.isfinite(raw_age) else '未收到'}\n"
            f"type=11 最近数据：{f'{reg_age:.2f}s' if math.isfinite(reg_age) else '未收到'}\n"
            f"机械臂最近反馈：{f'{robot_age:.2f}s' if math.isfinite(robot_age) else '未收到'}\n"
            f"type=5：{stats.frames_by_type.get(5, 0)}；type=8：{stats.frames_by_type.get(8, 0)}；"
            f"type=9：{stats.frames_by_type.get(9, 0)}；type=11：{stats.frames_by_type.get(11, 0)}"
        )

    def _append_log(self, message: str, level: str = "info") -> None:
        color = {"error": "#fca5a5", "good": "#86efac", "info": "#bfdbfe"}.get(level, "#dbeafe")
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.append(f'<span style="color:#94a3b8">[{timestamp}]</span> <span style="color:{color}">{message}</span>')

    def _show_error(self, message: str, modal: bool) -> None:
        self._append_log(message, "error")
        if modal and self._full_auto_enabled and self.coordinator.running:
            self._stop_full_auto(message, abort=True)
            return
        if modal:
            QMessageBox.warning(self, "操作未执行", message)

    def _stop_full_auto(self, reason: str = "", abort: bool = False) -> None:
        was_enabled = self._full_auto_enabled
        self._full_auto_enabled = False
        self._full_auto_timer.stop()
        self._full_auto_neutral_return_started_ns = 0
        self._full_auto_neutral_return_seen_motion = False
        if abort and self.coordinator.running:
            if was_enabled and reason:
                self._append_log(f"全自动流程停止：{reason}", "error")
            self.coordinator.abort(reason or "全自动流程停止")

    def _confirm_abort(self, reason: str) -> None:
        if QMessageBox.question(self, "确认中止", "将立即停止机械臂、发送 MCAL_ABORT，并保留失败记录。确认继续？") == QMessageBox.StandardButton.Yes:
            self.coordinator.abort(reason)

    def closeEvent(self, event: QCloseEvent) -> None:
        if self.coordinator.running:
            answer = QMessageBox.question(self, "标定仍在进行", "关闭程序将停止机械臂并放弃当前会话。确认关闭？")
            if answer != QMessageBox.StandardButton.Yes:
                event.ignore()
                return
            self.coordinator.abort("程序关闭")
        self.glove.close()
        self.robot.disconnect_robot()
        event.accept()


def run() -> int:
    app = QApplication.instance() or QApplication([])
    app.setApplicationName("SDB QuickCal V1")
    window = QuickCalWindow()
    window.show()
    return app.exec()
