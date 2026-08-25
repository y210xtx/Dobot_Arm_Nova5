"""Dedicated production UI for 11-lane IMU QuickCal."""

from __future__ import annotations

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
from .glove_device import GloveDevice
from .pose_store import TaughtPose, load_legacy_safe_pose, load_pose_config, save_pose_config
from .protocol import ALL_IMU_MASK
from .robot_device import RobotDevice
from .workflow import IMU_NAMES, YawLimits, expected_capture_seconds, expected_total_seconds, steps_for_limits


APP_DIR = Path(__file__).resolve().parent.parent
DEFAULT_OUTPUT_DIR = APP_DIR / "quickcal_records"
RECORDED_POSE_FILE = APP_DIR / "recorded_pose.json"
POSE_CONFIG_FILE = APP_DIR / "quickcal_poses.local.json"


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
        self._building_limits = False
        self._ui_raw_imu_frame = None
        self._ui_raw_imu_ns = 0
        self._ui_register_imu_ns = 0
        self._robot_enable_pending: bool | None = None
        self._robot_enable_timeout = QTimer(self)
        self._robot_enable_timeout.setSingleShot(True)
        self._robot_enable_timeout.setInterval(3000)
        self._robot_enable_timeout.timeout.connect(self._on_robot_enable_timeout)

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
        subtitle = QLabel("QuickCal V1｜30 s 静止 + 六面 + 六方向 30°/s + 分段三维磁翻转")
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

        feedback_group = QGroupBox("数据健康与日志")
        feedback_layout = QVBoxLayout(feedback_group)
        self.health_label = QLabel("等待设备连接")
        self.health_label.setWordWrap(True)
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
        self.yaw_rate = QLineEdit("30")
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
        self.retry_button = QPushButton("重试当前步骤")
        self.retry_button.setProperty("secondary", True)
        self.abort_button = QPushButton("停止并放弃")
        self.abort_button.setProperty("danger", True)
        row.addWidget(self.capture_progress, 1)
        row.addWidget(self.start_button)
        row.addWidget(self.confirm_button)
        row.addWidget(self.retry_button)
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
        self.confirm_button.clicked.connect(self.coordinator.confirm_current_action)
        self.retry_button.clicked.connect(self.coordinator.retry_current_step)
        self.abort_button.clicked.connect(lambda: self._confirm_abort("操作员中止"))

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
        message = (
            f"机械臂将先启用 User {taught.user} / Tool {taught.tool}，再以 15% 速度移动到{title}。\n\n"
            f"{self._pose_text(taught)}\n\n请确认路径无碰撞、人员已离开运动区域。"
        )
        if QMessageBox.question(self, "确认机器人运动", message) != QMessageBox.StandardButton.Yes:
            return
        if not self.robot.activate_frames(taught.user, taught.tool):
            return
        if taught.joints:
            self.robot.move_joints(taught.joints, 15)
        else:
            self.robot.move_pose_j(taught.pose, 15, taught.user, taught.tool)

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
            f"G05/G06 单向采集：{limits.capture_s:.3f} s\n"
            f"路径判定：{result}"
        )
        self.limits_result.setStyleSheet("color:#087f5b;" if limits.valid else "color:#b42318;")
        self.timeline_summary.setText(
            f"预计工单总时长：{expected_total_seconds(limits):.1f} s\n"
            f"预计有效采集时间：{expected_capture_seconds(limits):.1f} s\n"
            "时间来自 QuickCal_V1_Robot_Control_Steps(1).xlsx。"
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
        pose = ", ".join(f"{value:.1f}" for value in state.pose)
        self.robot_state_label.setText(f"mode={state.mode}｜TCP=({pose})｜|ω|={state.angular_speed_norm:.2f}°/s")
        enabled = self._robot_mode_is_enabled(state.mode)
        if self._robot_enable_pending is not None and enabled == self._robot_enable_pending:
            self._robot_enable_pending = None
            self._robot_enable_timeout.stop()
        self._update_robot_enable_button()
        if state.mode == 9:
            self._set_badge(self.robot_badge, "机械臂报警", "bad")
        elif enabled:
            self._set_badge(self.robot_badge, "机械臂已使能", "ok")
        else:
            self._set_badge(self.robot_badge, "机械臂未使能", "warn")
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
        badge_state = "ok" if state == RunState.COMPLETE.value else "warn" if state in (RunState.IDLE.value, RunState.READY.value) else "bad" if state in (RunState.ERROR.value, RunState.ABORTED.value) else "warn"
        self._set_badge(self.session_badge, state, badge_state)
        self._update_action_controls()

    @Slot(int, object)
    def _on_current_step(self, index: int, step) -> None:
        self.workflow_table.selectRow(index)
        self.workflow_table.scrollToItem(self.workflow_table.item(index, 0))
        self.instruction_label.setText(
            f"{step.step_id}｜{step.name}\n机械臂：{step.robot_action}\n开始条件：{step.start_condition}\n通过条件：{step.pass_condition}"
        )
        self.confirm_button.setText("求解并写入设备" if step.step_id == "S01" else "完成归档" if step.step_id == "S02" else "确认动作条件，执行本步")
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
        self.confirm_button.setEnabled(state == RunState.READY)
        self.retry_button.setEnabled(state == RunState.ERROR)
        self.abort_button.setEnabled(self.coordinator.running or state == RunState.ERROR)
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
        self._refresh_imu_online_status(now)
        parts = []
        raw_age = (now - self._ui_raw_imu_ns) / 1e9 if self._ui_raw_imu_ns else math.inf
        reg_age = (now - self._ui_register_imu_ns) / 1e9 if self._ui_register_imu_ns else math.inf
        robot_age = (now - self.coordinator.latest_robot_state.received_monotonic_ns) / 1e9 if self.coordinator.latest_robot_state else math.inf
        parts.append(f"type=9: {'正常' if raw_age < 0.8 else '等待'} ({raw_age:.2f}s)" if math.isfinite(raw_age) else "type=9: 等待")
        parts.append(f"type=11: {'正常' if reg_age < 0.8 else '等待'} ({reg_age:.2f}s)" if math.isfinite(reg_age) else "type=11: 等待")
        parts.append(f"机器人反馈: {'正常' if robot_age < 1.0 else '等待'} ({robot_age:.2f}s)" if math.isfinite(robot_age) else "机器人反馈: 等待")
        stats = self.glove.parser.stats
        type_counts = "/".join(str(stats.frames_by_type.get(frame_type, 0)) for frame_type in (5, 8, 9, 11))
        parts.append(
            f"帧={stats.accepted_frames}｜类型5/8/9/11={type_counts}｜"
            f"无效={stats.invalid_frames}｜丢序={stats.sequence_gaps}"
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
