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
from .protocol import ALL_IMU_MASK
from .robot_device import RobotDevice
from .workflow import IMU_NAMES, YawLimits, expected_capture_seconds, expected_total_seconds, steps_for_limits


APP_DIR = Path(__file__).resolve().parent.parent
DEFAULT_OUTPUT_DIR = APP_DIR / "quickcal_records"
RECORDED_POSE_FILE = APP_DIR / "recorded_pose.json"
POSE_CONFIG_FILE = APP_DIR / "quickcal_poses.local.json"
ACTION_CONFIG_FILE = APP_DIR / "quickcal_actions.local.json"
ROTATION_SPEED_CALIBRATION_FILE = APP_DIR / "rotation_speed_calibration.json"
TOOL_CONFIG_FILE = APP_DIR / "tool_offset_config.json"

G01_RESTORE_DEG = 90.0
G01_LEAD_IN_DEG = -85.0
G01_CAPTURE_START_DEG = -75.0
G01_TARGET_RATE_DEG_S = 15.0
G01_RATE_TOLERANCE_DEG_S = 3.0
G01_SPEED_STABLE_NS = 300_000_000
G01_TIMEOUT_S = 90.0


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
        self._g01_phase = ""
        self._g01_phase_started_ns = 0
        self._g01_reference_pose: tuple[float, ...] | None = None
        self._g01_original_speed_factor = 0
        self._g01_jog_active = False
        self._manual_absolute_pose_initialized = False
        self._manual_absolute_pose_frame: tuple[int, int] | None = None

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
        subtitle = QLabel("QuickCal V1｜30 s 静止 + 六面 + 六方向 15°/s + 分段三维磁翻转")
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
        self.robot_pose_label = QLabel()
        self.robot_pose_label.setWordWrap(True)
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
        robot_grid.addWidget(self.robot_pose_label, 3, 0, 1, 4)
        robot_grid.addWidget(self.robot_state_label, 4, 0, 1, 4)
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
        tabs.addTab(self._build_limits_page(), "Yaw 限位与流程参数")
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
        self.imu_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        health_layout.addWidget(self.imu_table)
        right_layout.addWidget(health_group, 3)

        feedback_group = QGroupBox("设备数据状态与日志")
        feedback_layout = QVBoxLayout(feedback_group)
        self.health_label = QLabel("等待 Dobot 机械臂与 IMU 手套连接")
        self.health_label.setWordWrap(True)
        self.health_label.setToolTip(
            "Dobot 机械臂反馈来自机械臂实时反馈端口；"
            "type=5/8/9/11 均来自 IMU 手套串口。"
        )
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.document().setMaximumBlockCount(1200)
        feedback_layout.addWidget(self.health_label)
        feedback_layout.addWidget(self.log_text, 1)
        right_layout.addWidget(feedback_group, 2)
        splitter.addWidget(right)
        splitter.setSizes((860, 600))
        layout.addWidget(splitter)
        return page

    def _build_limits_page(self) -> QWidget:
        page = QWidget()
        layout = QHBoxLayout(page)
        input_group = QGroupBox("Yaw 软限位输入（来自控制器实际配置）")
        form = QFormLayout(input_group)
        self.yaw_negative = QLineEdit("-50")
        self.yaw_positive = QLineEdit("50")
        self.yaw_margin = QLineEdit("10")
        self.yaw_rate = QLineEdit("15")
        self.yaw_min_capture = QLineEdit("2")
        for label, edit in (
            ("负向软限位（°）", self.yaw_negative),
            ("正向软限位（°）", self.yaw_positive),
            ("两端安全余量（°）", self.yaw_margin),
            ("标定角速度（°/s）", self.yaw_rate),
            ("最短有效采集（s）", self.yaw_min_capture),
        ):
            form.addRow(label, edit)
            edit.textChanged.connect(self._refresh_limits)
        layout.addWidget(input_group)
        result_group = QGroupBox("自动计算与强制联锁")
        result_layout = QVBoxLayout(result_group)
        self.limits_result = QLabel()
        self.limits_result.setWordWrap(True)
        self.limits_result.setFont(QFont("Microsoft YaHei UI", 13))
        self.timeline_summary = QLabel()
        self.timeline_summary.setWordWrap(True)
        warning = QLabel(
            "所有 Yaw 动作必须从 0° 中位进入，完成后回 0°；严禁累计转角。"
            "移动、加减速、反向和回中位均不得进入拟合采集窗口。"
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
        self.report_table = QTableWidget(11, 6)
        self.report_table.setHorizontalHeaderLabels(("IMU", "Gyro", "RMS（°）", "窗口数", "最大偏差", "Accel"))
        self.report_table.verticalHeader().setVisible(False)
        self.report_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.report_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        for row, name in enumerate(IMU_NAMES):
            self.report_table.setItem(row, 0, QTableWidgetItem(name))
            for column in range(1, 6):
                self.report_table.setItem(row, column, QTableWidgetItem("--"))
        layout.addWidget(self.report_summary)
        layout.addWidget(self.report_table)
        return page

    def _build_actions_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(10)

        config_group = QGroupBox("六面步骤自动旋转参数")
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
        self.action_velocity.setRange(1, 80)
        self.action_velocity.setValue(config.velocity_percent)
        self.action_velocity.setSuffix(" %")
        self.action_timeout = QDoubleSpinBox()
        self.action_timeout.setRange(5.0, 180.0)
        self.action_timeout.setDecimals(1)
        self.action_timeout.setValue(config.timeout_s)
        self.action_timeout.setSuffix(" s")
        parameter_widgets = (
            ("Tool 旋转轴", self.action_rotation_axis),
            ("相对旋转角度", self.action_rotation_degrees),
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

        motion_group = QGroupBox("末端绝对位姿（当前 User / Tool）")
        motion_layout = QVBoxLayout(motion_group)
        motion_layout.setContentsMargins(14, 16, 14, 14)
        motion_layout.setSpacing(10)
        motion_header = QHBoxLayout()
        self.tool_motion_feedback = QLabel("等待机械臂实时反馈")
        self.tool_motion_feedback.setWordWrap(True)
        motion_header.addWidget(self.tool_motion_feedback, 1)
        self.load_current_pose_button = QPushButton("读取当前位姿")
        self.load_current_pose_button.setProperty("secondary", True)
        motion_header.addWidget(self.load_current_pose_button)
        motion_header.addWidget(QLabel("速度比例"))
        self.manual_motion_velocity = QSpinBox()
        self.manual_motion_velocity.setRange(1, 80)
        self.manual_motion_velocity.setValue(20)
        self.manual_motion_velocity.setSuffix(" %")
        self.manual_motion_velocity.setMinimumWidth(100)
        motion_header.addWidget(self.manual_motion_velocity)
        motion_layout.addLayout(motion_header)

        translation_grid = QGridLayout()
        translation_grid.setHorizontalSpacing(10)
        translation_grid.addWidget(QLabel("绝对坐标"), 0, 0)
        self.manual_position_spins: dict[str, QDoubleSpinBox] = {}
        for column, name in enumerate(("X", "Y", "Z"), 1):
            spin = QDoubleSpinBox()
            spin.setDecimals(2)
            spin.setRange(-2000.0, 2000.0)
            spin.setSingleStep(1.0)
            spin.setSuffix(" mm")
            self.manual_position_spins[name] = spin
            translation_grid.addWidget(QLabel(name), 0, column)
            translation_grid.addWidget(spin, 1, column)
            translation_grid.setColumnStretch(column, 1)
        self.execute_translation_button = QPushButton("移动到绝对坐标")
        translation_grid.addWidget(self.execute_translation_button, 1, 4)
        motion_layout.addLayout(translation_grid)

        rotation_grid = QGridLayout()
        rotation_grid.setHorizontalSpacing(10)
        rotation_grid.addWidget(QLabel("绝对角度"), 0, 0)
        self.manual_rotation_spins: dict[str, QDoubleSpinBox] = {}
        for column, name in enumerate(("Rx", "Ry", "Rz"), 1):
            spin = QDoubleSpinBox()
            spin.setDecimals(2)
            spin.setRange(-180.0, 180.0)
            spin.setSingleStep(1.0)
            spin.setSuffix(" °")
            self.manual_rotation_spins[name] = spin
            rotation_grid.addWidget(QLabel(name), 0, column)
            rotation_grid.addWidget(spin, 1, column)
            rotation_grid.setColumnStretch(column, 1)
        self.execute_rotation_button = QPushButton("旋转到绝对角度")
        rotation_grid.addWidget(self.execute_rotation_button, 1, 4)
        motion_layout.addLayout(rotation_grid)

        note = QLabel(
            "输入值是当前 User 坐标系下的绝对 TCP 目标，当前 Tool 补偿保持生效。"
            "位置运动保留实时反馈的 Rx/Ry/Rz；角度运动保留实时反馈的 X/Y/Z。"
            "每次运动前显示完整目标位姿并再次确认；标定会话运行期间禁止手动运动。"
        )
        note.setWordWrap(True)
        note.setStyleSheet(
            "background:#f8fafc;border:1px solid #cbd5e1;border-radius:6px;padding:9px;color:#475467;"
        )
        motion_layout.addWidget(note)
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
        self.start_button = QPushButton("开始工厂会话")
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

    def _execute_manual_absolute_motion(self, kind: str) -> None:
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
            self._append_log(
                f"末端绝对运动已发送：User {state.user} / Tool {state.tool}，"
                f"target=({target_values})，v={velocity}%",
                "info",
            )

    def _action_config_from_ui(self, silent: bool = False) -> RobotActionConfig | None:
        try:
            return RobotActionConfig(
                enabled=self.action_auto_enabled.isChecked(),
                axis=self.action_rotation_axis.currentText(),
                degrees=self.action_rotation_degrees.value(),
                velocity_percent=self.action_velocity.value(),
                timeout_s=self.action_timeout.value(),
            )
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
        finally:
            self._building_action_config = False
        self._refresh_action_preview()

    def _config_for_step(self, step_id: str) -> RobotActionConfig | None:
        if self.action_step_combo.currentText() == step_id:
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
        if step_id == "G01":
            self._start_g01_auto_action()
            return
        config = self._config_for_step(step_id)
        if step_id in self.robot_actions and config is not None and config.enabled:
            self._start_accel_auto_action(step_id, config)
            return
        self.coordinator.confirm_current_action()

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
            if QMessageBox.question(self, f"确认执行 {step_id} 自动旋转", message) != QMessageBox.StandardButton.Yes:
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
        self._append_log(message, level)
        self._update_action_controls()

    def _check_auto_action_timeout(self, now_ns: int | None = None) -> None:
        if self._auto_action_step is None:
            return
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        if self._auto_action_deadline_ns and now_ns > self._auto_action_deadline_ns:
            step_id = self._auto_action_step
            if step_id == "G01":
                self._fail_g01_auto_action("G01 自动动作超时")
                return
            self.robot.stop()
            self._finish_auto_action(f"{step_id} 自动旋转等待到位超时，已停止机械臂", "error")

    @staticmethod
    def _speed_factor_percent(value: float) -> int:
        value = float(value)
        percent = value if value > 1.5 else value * 100.0
        return max(1, min(100, round(percent)))

    def _g01_jog_speed_factor(self, tool: int) -> tuple[int, str]:
        full_speed_deg_s = 0.0
        source = "保守估算"
        try:
            data = json.loads(ROTATION_SPEED_CALIBRATION_FILE.read_text(encoding="utf-8"))
            profiles = data.get("profiles", {})
            jog_profile = profiles.get(f"jog_tool_{int(tool)}_Rx", {})
            full_speed_deg_s = float(
                jog_profile.get("full_global_speed_deg_s", 0.0) or 0.0
            )
            if full_speed_deg_s > 0.0:
                source = "Tool Rx 点动标定"
            else:
                relative_profile = profiles.get(f"tool_{int(tool)}_Rx", {})
                rate = float(
                    relative_profile.get("deg_s_per_v_at_full_global", 0.0) or 0.0
                )
                if rate > 0.0:
                    full_speed_deg_s = rate * 100.0
                    source = "Tool Rx 相对运动标定"
        except (OSError, ValueError, TypeError):
            full_speed_deg_s = 0.0
        if not math.isfinite(full_speed_deg_s) or full_speed_deg_s <= 0.0:
            full_speed_deg_s = 100.0
        factor = round(G01_TARGET_RATE_DEG_S / full_speed_deg_s * 100.0)
        return max(1, min(80, factor)), source

    def _relative_tool_rx_deg(
        self, reference_pose: tuple[float, ...], current_pose: tuple[float, ...]
    ) -> float:
        reference_axes = self.coordinator._tool_axes(reference_pose)
        current_axes = self.coordinator._tool_axes(current_pose)
        cosine = sum(a * b for a, b in zip(reference_axes[1], current_axes[1]))
        sine = sum(a * b for a, b in zip(reference_axes[2], current_axes[1]))
        return math.degrees(math.atan2(sine, cosine))

    def _start_g01_auto_action(self) -> None:
        if self._auto_action_step is not None:
            self._show_error("机械臂自动动作已经在执行", False)
            return
        if (
            self.coordinator.state != RunState.READY
            or self.coordinator.current_step.step_id != "G01"
        ):
            self._show_error("只有在 G01 等待动作条件时才能执行自动扫转", False)
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
            self._show_error("当前 User/Tool 与示教标定中位不一致，禁止执行 G01", True)
            return
        alignment = self.coordinator._accel_face_alignment("A06", state.pose)
        if alignment is None or alignment[0] > self.coordinator.ACCEL_FACE_MAX_DEG:
            detail = "无法计算当前姿态" if alignment is None else f"当前 -Z 偏差 {alignment[0]:.2f}°"
            self._show_error(f"G01 必须从 A06 完成姿态开始；{detail}", True)
            return
        predicted = gravity_after_tool_rotation(
            alignment[1], "Rx", G01_RESTORE_DEG
        )
        restore_error = vector_angle_deg(predicted, (0.0, -1.0, 0.0))
        if restore_error > self.coordinator.ACCEL_FACE_MAX_DEG:
            self._show_error(
                f"Rx +90° 复原后预计 -Y 偏差 {restore_error:.2f}°，已禁止下发",
                True,
            )
            return
        message = (
            "G01 将自动执行以下动作：\n"
            "1. Tool Rx +90°，撤销 A05/A06 的净 Rx 动作并回到 -Y 基准；\n"
            "2. Tool Rx -85°，作为加速引入区；\n"
            "3. Tool Rx+ 连续点动，实测经过 -75°且稳定在 +15±3°/s 时开启 10 秒采集；\n"
            "4. 采集结束停止点动，并自动回到 G01 的 X 中位。\n\n"
            "请确认完整路径无碰撞、线束无拉扯，人员已离开运动区域。"
        )
        if QMessageBox.question(self, "确认执行 G01 自动扫转", message) != QMessageBox.StandardButton.Yes:
            return

        now = time.monotonic_ns()
        self._auto_action_step = "G01"
        self._auto_action_started_ns = now
        self._auto_action_deadline_ns = now + int(G01_TIMEOUT_S * 1_000_000_000)
        self._auto_action_seen_motion = False
        self._auto_action_stable_since_ns = 0
        self._g01_phase = "restore"
        self._g01_phase_started_ns = now
        self._g01_reference_pose = None
        self._g01_original_speed_factor = self._speed_factor_percent(state.speed_scaling)
        self._g01_jog_active = False
        self.coordinator.recorder.marker(
            "robot_auto_move_request", "G01", "restore=Rx+90; lead_in=Rx-85; capture=Rx+@15deg/s"
        )
        self._append_log(
            f"G01 开始复原：Tool Rx +{G01_RESTORE_DEG:.1f}°，原全局速度比例 "
            f"{self._g01_original_speed_factor}%",
            "info",
        )
        self._update_action_controls()
        if not self.robot.relative_tool_rotation("Rx", G01_RESTORE_DEG, 80):
            self._fail_g01_auto_action("G01 复原命令发送失败")

    def _restore_g01_speed_factor(self) -> None:
        if self._g01_original_speed_factor:
            self.robot.set_speed_factor(self._g01_original_speed_factor)
            self._g01_original_speed_factor = 0

    def _stop_g01_jog(self) -> None:
        if self._g01_jog_active:
            self.robot.stop_tool_jog()
            self._g01_jog_active = False
        self._restore_g01_speed_factor()

    def _fail_g01_auto_action(self, message: str) -> None:
        self._stop_g01_jog()
        self.robot.stop()
        self._g01_phase = ""
        self._g01_reference_pose = None
        self._finish_auto_action(message, "error")
        if self.coordinator.running:
            self.coordinator.abort(message)

    def _update_g01_auto_action(self, state) -> None:
        if self._auto_action_step != "G01":
            return
        now = time.monotonic_ns()
        self._check_auto_action_timeout(now)
        if self._auto_action_step != "G01":
            return
        phase = self._g01_phase
        if phase in ("restore", "preposition", "return"):
            if state.mode in (7, 8) or state.angular_speed_norm > 0.8:
                self._auto_action_seen_motion = True
                return
            if state.mode != 5 or state.linear_speed_norm > 1.0 or state.angular_speed_norm > 0.8:
                return
            if (
                not self._auto_action_seen_motion
                and now - self._g01_phase_started_ns < 800_000_000
            ):
                return
            if phase == "restore":
                alignment = self.coordinator._accel_face_alignment("A04", state.pose)
                if alignment is None or alignment[0] > self.coordinator.ACCEL_FACE_MAX_DEG:
                    error = math.inf if alignment is None else alignment[0]
                    self._fail_g01_auto_action(f"G01 Rx +90°复原后 -Y 偏差 {error:.2f}°")
                    return
                self._g01_reference_pose = tuple(state.pose)
                self._g01_phase = "preposition"
                self._g01_phase_started_ns = now
                self._auto_action_seen_motion = False
                self._append_log("G01 已恢复 -Y 基准，开始 Rx -85° 加速引入预置", "good")
                if not self.robot.relative_tool_rotation("Rx", G01_LEAD_IN_DEG, 80):
                    self._fail_g01_auto_action("G01 加速引入预置命令发送失败")
                return
            if self._g01_reference_pose is None:
                self._fail_g01_auto_action("G01 缺少 X 中位参考姿态")
                return
            angle = self._relative_tool_rx_deg(self._g01_reference_pose, state.pose)
            if phase == "preposition":
                if abs(angle - G01_LEAD_IN_DEG) > 5.0:
                    self._fail_g01_auto_action(
                        f"G01 预置到位角度异常：实测 Rx {angle:+.2f}°"
                    )
                    return
                speed_factor, source = self._g01_jog_speed_factor(state.tool)
                self._g01_phase = "lead_in"
                self._g01_phase_started_ns = now
                self._auto_action_stable_since_ns = 0
                self._append_log(
                    f"G01 开始 Rx+ 点动：速度比例 {speed_factor}%（{source}），"
                    "等待经过 -75°并达到 +15±3°/s",
                    "info",
                )
                if not self.robot.start_tool_jog(
                    "Rx", True, speed_factor, user=state.user, tool=state.tool
                ):
                    self._fail_g01_auto_action("G01 连续点动命令发送失败")
                    return
                self._g01_jog_active = True
                return
            if abs(angle) > 2.0:
                self._fail_g01_auto_action(
                    f"G01 回 X 中位后仍偏差 {angle:+.2f}°"
                )
                return
            self._g01_phase = ""
            self._g01_reference_pose = None
            self._finish_auto_action("G01 已完成采集并回到 X 中位", "good")
            return

        if self._g01_reference_pose is None:
            self._fail_g01_auto_action("G01 缺少 X 中位参考姿态")
            return
        angle = self._relative_tool_rx_deg(self._g01_reference_pose, state.pose)
        tool_rate = self.coordinator._tool_angular_speed(
            state.pose, state.angular_speed
        )[0]
        if phase == "lead_in":
            rate_ok = (
                G01_TARGET_RATE_DEG_S - G01_RATE_TOLERANCE_DEG_S
                <= tool_rate
                <= G01_TARGET_RATE_DEG_S + G01_RATE_TOLERANCE_DEG_S
            )
            if rate_ok:
                if self._auto_action_stable_since_ns == 0:
                    self._auto_action_stable_since_ns = now
            else:
                self._auto_action_stable_since_ns = 0
            stable = (
                self._auto_action_stable_since_ns
                and now - self._auto_action_stable_since_ns >= G01_SPEED_STABLE_NS
            )
            if angle >= G01_CAPTURE_START_DEG and stable:
                self.coordinator.condition_stable_since_ns = now - 1_100_000_000
                if not self.coordinator.confirm_current_action():
                    self._fail_g01_auto_action("G01 匀速到位后的阶段开启复检未通过")
                    return
                self._g01_phase = "wait_stage_open"
                self._append_log(
                    f"G01 经过 Rx {angle:+.2f}°，实测 {tool_rate:+.2f}°/s，正在开启采集",
                    "good",
                )
                return
            if angle > -65.0:
                self._fail_g01_auto_action(
                    f"G01 已越过 -65°仍未达到稳定速度：实测 {tool_rate:+.2f}°/s"
                )
            return
        if phase == "wait_stage_open" and self.coordinator.state == RunState.CAPTURING:
            self._g01_phase = "capturing"
            return
        if phase == "wait_jog_stop":
            if state.mode != 5 or state.angular_speed_norm > 0.8:
                return
            if (
                self.coordinator.state != RunState.READY
                or self.coordinator.current_step.step_id != "G02"
            ):
                return
            return_angle = self._relative_tool_rx_deg(
                self._g01_reference_pose, state.pose
            )
            if abs(return_angle) < 0.5:
                self._g01_phase = "return"
                self._g01_phase_started_ns = now - 800_000_000
                self._auto_action_seen_motion = True
                return
            if abs(return_angle) > 180.0:
                self._fail_g01_auto_action(
                    f"G01 停止位置无法安全回中：Rx {return_angle:+.2f}°"
                )
                return
            self._g01_phase = "return"
            self._g01_phase_started_ns = now
            self._auto_action_seen_motion = False
            self._append_log(
                f"G01 采集扫转停止于 Rx {return_angle:+.2f}°，开始回 X 中位",
                "info",
            )
            if not self.robot.relative_tool_rotation("Rx", -return_angle, 80):
                self._fail_g01_auto_action("G01 回 X 中位命令发送失败")

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
        labels = []
        tooltips = []
        for name, title in (("safe", "安全位"), ("neutral", "标定中位")):
            taught = self.taught_poses.get(name)
            if taught is None:
                labels.append(f"{title}：未示教")
                continue
            source = "历史位姿" if not taught.joints else taught.recorded_at.replace("T", " ")
            labels.append(f"{title}：已记录 {source}｜U{taught.user}/T{taught.tool}")
            tooltips.append(f"{title}\n{self._pose_text(taught)}")
        self.robot_pose_label.setText("\n".join(labels))
        self.robot_pose_label.setToolTip("\n\n".join(tooltips))

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
            else "请确认从这里绕夹具 Tool X/Y 可达 ±75°、Yaw 可达设定安全限位，且线束无拉扯。"
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
        velocity_percent = 30 if name == "neutral" else 15
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
                self._show_error("Yaw 限位参数必须是有效数字", True)
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
            f"负向安全位：{limits.negative_safe_deg:+.1f}°\n"
            f"正向安全位：{limits.positive_safe_deg:+.1f}°\n"
            f"有效扫描角度：{limits.scan_angle_deg:.1f}°\n"
            f"G05/G06 单方向有效采集：{limits.capture_s:.3f} s\n"
            f"路径判定：{result}"
        )
        self.limits_result.setStyleSheet("color:#087f5b;" if limits.valid else "color:#b42318;")
        self.timeline_summary.setText(
            f"预计工单总时长：{expected_total_seconds(limits):.1f} s\n"
            f"预计有效采集时间：{expected_capture_seconds(limits):.1f} s\n"
            "时间来自 QuickCal_V1_Robot_Control_Steps_15dps.xlsx。"
        )
        self._populate_workflow()

    def _start_session(self) -> None:
        limits = self._limits_from_ui()
        if limits is None:
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
            self.coordinator.configure(
                self.sn_edit.text(),
                self.station_edit.text(),
                self.operator_edit.text(),
                Path(self.output_edit.text()),
                limits,
                self.environment_check.isChecked(),
                neutral_pose=neutral.pose,
            )
        except Exception as exc:
            self._show_error(str(exc), True)
            return
        self._populate_workflow()
        self.coordinator.start_session()

    @Slot(bool, str)
    def _robot_connection_changed(self, connected: bool, detail: str) -> None:
        self.robot_connect.setText("断开" if connected else "连接")
        if not connected:
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
            self.tool_motion_feedback.setText(
                f"当前反馈：User {state.user} / Tool {state.tool}｜"
                f"TCP=({pose})｜|ω|={state.angular_speed_norm:.2f}°/s"
            )
            configured_tool = int(self.tool_offset_config["tool"])
            configured_offset = ", ".join(
                f"{float(value):.1f}" for value in self.tool_offset_config["offset"]
            )
            self.tool_config_status.setText(
                f"本地 Tool {configured_tool}=({configured_offset})｜"
                f"控制器当前反馈 Tool {state.tool}"
            )
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
        if self._auto_action_step == "G01":
            self._update_g01_auto_action(state)
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
        self._ui_register_imu_ns = time.monotonic_ns()
        for row, sample in enumerate(frame.samples):
            self.imu_table.item(row, 4).setText(f"{sample.gx}/{sample.gy}/{sample.gz}")
            self.imu_table.item(row, 5).setText(f"{sample.ax}/{sample.ay}/{sample.az}")

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
        if self._ui_raw_imu_frame is None or self._ui_raw_imu_ns == 0:
            text, color = "等待 type=9", "#b45309"
            for row in range(len(IMU_NAMES)):
                item = self.imu_table.item(row, 1)
                item.setText(text)
                item.setForeground(QColor(color))
                item.setToolTip("串口已打开，但尚未收到 type=9 工程量 IMU 帧；这不等同于硬件离线")
            self._set_badge(self.imu_badge, "等待 type=9", "warn")
            return
        age_s = (now_ns - self._ui_raw_imu_ns) / 1_000_000_000.0
        if age_s >= self.coordinator.RAW_FRESH_NS / 1_000_000_000.0:
            for row in range(len(IMU_NAMES)):
                item = self.imu_table.item(row, 1)
                item.setText("数据超时")
                item.setForeground(QColor("#b42318"))
                item.setToolTip(f"最后一帧 type=9 距今 {age_s:.2f} 秒")
            self._set_badge(self.imu_badge, "IMU 数据超时", "bad")
            return
        presence_mask = self._ui_raw_imu_frame.presence_mask & ALL_IMU_MASK
        for row in range(len(IMU_NAMES)):
            online = bool(presence_mask & (1 << row))
            item = self.imu_table.item(row, 1)
            item.setText("在线" if online else "离线")
            item.setForeground(QColor("#087f5b" if online else "#dc2626"))
            item.setToolTip(
                f"type=9 数据新鲜，presence_mask=0x{presence_mask:04X}，bit {row}="
                f"{1 if online else 0}"
            )
        online_count = presence_mask.bit_count()
        self._set_badge(self.imu_badge, f"IMU {online_count}/11", "ok" if online_count == 11 else "bad")

    @Slot(object)
    def _on_version(self, frame) -> None:
        self.glove_version_label.setText(f"固件：{frame.revision_tag}｜{frame.imu_model}｜{frame.hand_side}")
        self._append_log(f"固件 {frame.revision_tag}，构建 {frame.build_date} {frame.build_time}")

    @Slot(object)
    def _on_report(self, report) -> None:
        self.report_summary.setText(
            f"报告 v{report.version}｜status={report.status}｜Gyro={report.calibrated_count}/{report.imu_count}｜"
            f"Accel={'11/11' if report.accel_all_ok else '未全通过'}｜Flash seq={report.flash_sequence}｜"
            f"平均 RMS={report.mean_rms_mdeg / 1000:.3f}°"
        )
        for row in range(11):
            gyro = report.gyro_quality[row] if row < len(report.gyro_quality) else None
            accel = report.accel_quality[row] if row < len(report.accel_quality) else None
            values = (
                "通过" if gyro and gyro.ok else "失败",
                f"{gyro.rms_mdeg / 1000:.3f}" if gyro else "--",
                str(gyro.window_count) if gyro else "--",
                str(gyro.max_off_axis) if gyro else "--",
                "通过" if accel and accel.ok else "失败",
            )
            for column, value in enumerate(values, 1):
                self.report_table.item(row, column).setText(value)
            passed = bool(gyro and gyro.ok and accel and accel.ok)
            self.imu_table.item(row, 6).setText("通过" if passed else "失败")
            self.imu_table.item(row, 6).setForeground(QColor("#087f5b" if passed else "#dc2626"))

    @Slot(str)
    def _on_run_state(self, state: str) -> None:
        if self._auto_action_step == "G01":
            if state == RunState.CAPTURING.value and self._g01_phase == "wait_stage_open":
                self._g01_phase = "capturing"
            elif state == RunState.WAIT_STAGE_CLOSE.value and self._g01_phase in (
                "wait_stage_open",
                "capturing",
            ):
                self._stop_g01_jog()
                self._g01_phase = "wait_jog_stop"
                self._g01_phase_started_ns = time.monotonic_ns()
        if self._auto_action_step is not None and state in (
            RunState.ABORTED.value,
            RunState.COMPLETE.value,
        ):
            step_id = self._auto_action_step
            if step_id == "G01":
                self._stop_g01_jog()
                self._g01_phase = ""
                self._g01_reference_pose = None
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
        self._update_action_controls()
        QMessageBox.information(self, "QuickCal 完成" if passed else "QuickCal 未通过", reason)

    def _update_action_controls(self) -> None:
        state = self.coordinator.state
        idle = state in (RunState.IDLE, RunState.COMPLETE, RunState.ABORTED)
        self.start_button.setEnabled(idle)
        self.confirm_button.setEnabled(
            state == RunState.READY
            and self.coordinator.current_step.step_id != "P1"
            and self._auto_action_step is None
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
        manual_page_editable = connection_controls and self._auto_action_step is None
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
        self._refresh_action_preview()
        self._refresh_imu_online_status(now)
        parts = []
        raw_age = (now - self._ui_raw_imu_ns) / 1e9 if self._ui_raw_imu_ns else math.inf
        reg_age = (now - self._ui_register_imu_ns) / 1e9 if self._ui_register_imu_ns else math.inf
        robot_age = (now - self.coordinator.latest_robot_state.received_monotonic_ns) / 1e9 if self.coordinator.latest_robot_state else math.inf
        parts.append(
            f"IMU 工程量（type=9）：{'正常' if raw_age < 0.8 else '等待'}（{raw_age:.2f}s）"
            if math.isfinite(raw_age)
            else "IMU 工程量（type=9）：等待"
        )
        parts.append(
            f"IMU 寄存器原始数据（type=11）：{'正常' if reg_age < 0.8 else '等待'}（{reg_age:.2f}s）"
            if math.isfinite(reg_age)
            else "IMU 寄存器原始数据（type=11）：等待"
        )
        parts.append(
            f"Dobot 机械臂实时反馈：{'正常' if robot_age < 1.0 else '等待'}（{robot_age:.2f}s）"
            if math.isfinite(robot_age)
            else "Dobot 机械臂实时反馈：等待"
        )
        stats = self.glove.parser.stats
        parts.append(
            f"IMU 手套串口：有效帧={stats.accepted_frames}｜"
            f"type=5 姿态={stats.frames_by_type.get(5, 0)}｜"
            f"type=8 固件信息={stats.frames_by_type.get(8, 0)}｜"
            f"type=9 工程量={stats.frames_by_type.get(9, 0)}｜"
            f"type=11 寄存器原始数据={stats.frames_by_type.get(11, 0)}｜"
            f"无效帧={stats.invalid_frames}｜丢失序号={stats.sequence_gaps}"
        )
        self.health_label.setText("　".join(parts))

    def _append_log(self, message: str, level: str = "info") -> None:
        color = {"error": "#fca5a5", "good": "#86efac", "info": "#bfdbfe"}.get(level, "#dbeafe")
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.append(f'<span style="color:#94a3b8">[{timestamp}]</span> <span style="color:{color}">{message}</span>')

    def _show_error(self, message: str, modal: bool) -> None:
        self._append_log(message, "error")
        if modal:
            QMessageBox.warning(self, "操作未执行", message)

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
