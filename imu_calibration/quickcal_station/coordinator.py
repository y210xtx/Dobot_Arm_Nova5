"""Fail-safe coordinator for the robot-assisted QuickCal workflow."""

from __future__ import annotations

from enum import Enum
import math
from pathlib import Path
import time

from PySide6.QtCore import QObject, QTimer, Signal, Slot

from .protocol import (
    ALL_IMU_MASK,
    CMD_MCAL_ABORT,
    CMD_MCAL_BEGIN,
    CMD_MCAL_COMMIT,
    CMD_MCAL_STAGE,
    EXPECTED_FIRMWARE_REVISION,
    EXPECTED_FIRMWARE_TAG,
    EXPECTED_GYRO_SEGMENTS,
    EXPECTED_VERSION_PAYLOAD_VERSION,
)
from .session_recorder import SessionRecorder
from .workflow import (
    IMU_NAMES,
    ROLL_PITCH_GYRO_RATE_DEG_S,
    QuickCalStep,
    YawLimits,
    steps_for_limits,
)


class RunState(str, Enum):
    IDLE = "空闲"
    WAIT_BEGIN_ACK = "等待会话确认"
    READY = "等待动作条件"
    WAIT_STAGE_OPEN = "等待阶段开启"
    CAPTURING = "采集中"
    WAIT_STAGE_CLOSE = "等待阶段质检"
    WAIT_COMMIT_ACK = "等待写入确认"
    WAIT_REPORT = "等待最终报告"
    COMPLETE = "完成"
    ABORTED = "已中止"


class QuickCalCoordinator(QObject):
    state_changed = Signal(str)
    current_step_changed = Signal(int, object)
    step_status_changed = Signal(int, str, str)
    progress_changed = Signal(int, str)
    status_message = Signal(str, str)
    finished = Signal(bool, str)

    RAW_FRESH_NS = 800_000_000
    ROBOT_FRESH_NS = 1_000_000_000
    ACK_TIMEOUT_NS = 1_000_000_000
    COMMIT_TIMEOUT_NS = 2_000_000_000
    REPORT_TIMEOUT_NS = 10_000_000_000
    # Stage-open ACKs and raw frames arrive through different Qt signal queues.
    # M01 also pumps the event loop while queueing 80 MovJ targets, so a timer
    # callback can overtake the first raw frame for the new stage.  Wait for a
    # matching type=9/type=11 pair before treating the last idle-stage frame as
    # a fault; a real stream/stage mismatch still fails after this timeout.
    RAW_STAGE_SYNC_NS = 2_000_000_000
    DYNAMIC_DEVIATION_GRACE_NS = 300_000_000
    ROBOT_MODE_IDLE = 5
    ROBOT_DYNAMIC_MODES = (7, 8)
    ROBOT_CAPTURE_MODES = (ROBOT_MODE_IDLE, *ROBOT_DYNAMIC_MODES)
    ROBOT_ERROR_MODE = 9
    ROBOT_COLLISION_MODE = 11
    P1_IMU_GYRO_MAX_RAD_S = 0.10
    P1_IMU_ACCEL_DELTA_MAX_G = 0.08
    P1_IMU_ACCEL_NORM_MIN_G = 0.70
    P1_IMU_ACCEL_NORM_MAX_G = 1.30
    # The observed FlexIO/SPI fault produces a one-frame gyro spike while the
    # same lane's accelerometer becomes exactly zero. Do not confuse one such
    # corrupt snapshot with physical motion. A real P1 fault must persist for
    # several consecutive type=9 frames before it can abort the stage.
    P1_IMU_CORRUPT_ACCEL_NORM_MAX_G = 0.10
    P1_IMU_FAULT_CONFIRM_FRAMES = 3

    # Dobot base +Z points upward, so physical gravity is base -Z.  The active
    # Tool frame must be defined to match the calibration fixture axes.
    BASE_GRAVITY_UNIT = (0.0, 0.0, -1.0)
    ACCEL_FACE_WARNING_DEG = 2.0
    ACCEL_FACE_MAX_DEG = 5.0
    MAG_COVERAGE_SPEED_DEG_S = 1.0
    # MoveJog axis changes are stop-confirm-start operations.  Allow the robot
    # controller enough time to decelerate and report idle without treating the
    # deliberate transition as lost magnetic motion.
    MAG_DYNAMIC_DEVIATION_GRACE_NS = 2_000_000_000
    # r024-fac-magq workflow: G06 is followed by required M01..M04 magnetic stages.
    SKIP_MAGNETIC_STAGES = False
    ACCEL_FACE_TARGETS = {
        "A01": ("+X", (1.0, 0.0, 0.0)),
        "A02": ("-X", (-1.0, 0.0, 0.0)),
        "A03": ("+Y", (0.0, 1.0, 0.0)),
        "A04": ("-Y", (0.0, -1.0, 0.0)),
        "A05": ("+Z", (0.0, 0.0, 1.0)),
        "A06": ("-Z", (0.0, 0.0, -1.0)),
    }

    def __init__(self, glove, robot, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.glove = glove
        self.robot = robot
        self.recorder = SessionRecorder()
        self.limits = YawLimits()
        self.gyro_motion_map = {
            "G01": ("Rx", 15.0),
            "G02": ("Rx", -15.0),
            "G03": ("Ry", 15.0),
            "G04": ("Ry", -15.0),
            "G05": ("Rz", 15.0),
            "G06": ("Rz", -15.0),
        }
        self.steps = steps_for_limits(self.limits)
        self.step_status = ["未开始"] * len(self.steps)
        self.step_detail = [""] * len(self.steps)
        self.state = RunState.IDLE
        self.current_index = 0
        self.product_sn = ""
        self.station_id = ""
        self.operator = ""
        self.output_directory = Path.cwd()
        self.environment_confirmed = False
        self.neutral_pose: tuple[float, ...] | None = None
        self.gyro_limited_reference_pose: tuple[float, ...] | None = None
        self.version = None
        self.latest_raw_imu = None
        self.latest_register_imu = None
        self.latest_raw_mag = None
        self.latest_mag_pair = None
        self.latest_robot_state = None
        self.raw_imu_ns = 0
        self.register_imu_ns = 0
        self.raw_mag_ns = 0
        self.mag_pair_ns = 0
        self.valid_mag_pairs_this_step = 0
        self.invalid_mag_pairs_this_step = 0
        self.capture_started_ns = 0
        self.capture_deadline_ns = 0
        self.operation_deadline_ns = 0
        self.commit_ack_ok = False
        self.commit_ack_received = False
        self.commit_ack_frame = None
        self.pending_report = None
        self._capture_fault = ""
        self._aborting = False
        self.condition_stable_since_ns = 0
        self.dynamic_motion_lost_since_ns = 0
        self.motion_coverage: dict[str, bool] = {}
        self.p1_imu_checked_frames = 0
        self.p1_imu_motion_error = ""
        self.p1_previous_accel: tuple[tuple[float, float, float], ...] | None = None
        self.p1_imu_fault_kind = ""
        self.p1_imu_fault_count = 0
        self.p1_imu_transient_issue = False
        self.p1_imu_rejected_frames = 0

        self.tick_timer = QTimer(self)
        self.tick_timer.setInterval(100)
        self.tick_timer.timeout.connect(self._tick)
        self.tick_timer.start()

        glove.raw_imu_received.connect(self.on_raw_imu)
        glove.register_raw_imu_received.connect(self.on_register_raw_imu)
        glove.raw_mag_received.connect(self.on_raw_mag)
        glove.factory_mag_pair_received.connect(self.on_mag_pair)
        glove.version_received.connect(self.on_version)
        glove.ack_received.connect(self.on_ack)
        glove.mcal_report_received.connect(self.on_mcal_report)
        if hasattr(glove, "tx_bytes_sent"):
            glove.tx_bytes_sent.connect(self.on_glove_tx_bytes)
        if hasattr(glove, "rx_bytes_received"):
            glove.rx_bytes_received.connect(self.on_glove_rx_bytes)
        if hasattr(glove, "connection_changed"):
            glove.connection_changed.connect(self.on_glove_connection_changed)
        robot.state_received.connect(self.on_robot_state)
        robot.error_occurred.connect(self.on_robot_error)
        glove.error_occurred.connect(self.on_glove_error)

    @property
    def running(self) -> bool:
        return self.state not in (RunState.IDLE, RunState.COMPLETE, RunState.ABORTED)

    @property
    def current_step(self) -> QuickCalStep:
        return self.steps[min(self.current_index, len(self.steps) - 1)]

    def configure(
        self,
        product_sn: str,
        station_id: str,
        operator: str,
        output_directory: Path,
        limits: YawLimits,
        environment_confirmed: bool,
        neutral_pose: tuple[float, ...] | None = None,
        gyro_motion_map: dict[str, tuple[str, float]] | None = None,
    ) -> None:
        if self.running:
            raise RuntimeError("标定进行中不能修改配置")
        self.product_sn = product_sn.strip()
        self.station_id = station_id.strip()
        self.operator = operator.strip()
        self.output_directory = Path(output_directory)
        self.limits = limits
        self.steps = steps_for_limits(limits)
        self.environment_confirmed = environment_confirmed
        self.gyro_limited_reference_pose = None
        if gyro_motion_map is not None:
            expected_steps = {f"G{index:02d}" for index in range(1, 7)}
            if set(gyro_motion_map) != expected_steps:
                raise ValueError("G01-G06 机械臂轴映射必须完整")
            validated: dict[str, tuple[str, float]] = {}
            for step_id, (axis, rate) in gyro_motion_map.items():
                axis = str(axis)
                rate = float(rate)
                if axis not in ("Rx", "Ry", "Rz"):
                    raise ValueError(f"{step_id} Tool 旋转轴无效：{axis}")
                if not math.isclose(abs(rate), ROLL_PITCH_GYRO_RATE_DEG_S, abs_tol=1e-9):
                    raise ValueError(f"{step_id} 角速度必须为 ±15°/s")
                validated[step_id] = (axis, rate)
            for positive, negative in (("G01", "G02"), ("G03", "G04"), ("G05", "G06")):
                pos_axis, pos_rate = validated[positive]
                neg_axis, neg_rate = validated[negative]
                if pos_axis != neg_axis or not math.isclose(pos_rate, -neg_rate, abs_tol=1e-9):
                    raise ValueError(f"{positive}/{negative} 必须使用同一 Tool 轴且方向相反")
            self.gyro_motion_map = validated
        if neutral_pose is None:
            self.neutral_pose = None
        else:
            values = tuple(float(value) for value in neutral_pose)
            if len(values) != 6 or not all(math.isfinite(value) for value in values):
                raise ValueError("标定中位必须包含 6 个有限数值")
            self.neutral_pose = values

    def preflight_errors(self) -> list[str]:
        now = time.monotonic_ns()
        errors = []
        if not self.product_sn:
            errors.append("产品 SN 未填写")
        if not self.station_id:
            errors.append("工位编号未填写")
        if self.output_directory.exists() and not self.output_directory.is_dir():
            errors.append("记录根目录路径不是文件夹")
        if not self.glove.is_open:
            errors.append("手套串口未连接")
        if self.version is None:
            errors.append("尚未读取手套固件版本")
        else:
            if self.version.payload_version != EXPECTED_VERSION_PAYLOAD_VERSION:
                errors.append(
                    f"不支持的 Type 8 payload 版本 {self.version.payload_version}"
                )
            if not self.version.factory_intrinsic:
                errors.append("固件未声明 staged MCAL 工厂内参能力（feature 0x20）")
            if not self.version.accel_intrinsic:
                errors.append("固件未声明加速度内参能力（feature 0x40）")
            if self.version.revision != EXPECTED_FIRMWARE_REVISION:
                errors.append(
                    f"固件 revision={self.version.revision}，r024 工位要求 "
                    f"revision={EXPECTED_FIRMWARE_REVISION}"
                )
            if self.version.revision_tag != EXPECTED_FIRMWARE_TAG:
                errors.append(
                    f"固件 tag={self.version.revision_tag!r}，要求 "
                    f"{EXPECTED_FIRMWARE_TAG!r}"
                )
            if not self.version.factory_raw_streams:
                errors.append("固件未声明 r024 原始流能力（feature 0x80）")
            if not self.version.magnetic_factory:
                errors.append("固件未声明磁标定工序/质量报告能力（feature 0x10）")
        raw_error = self._raw_health_error(now)
        if raw_error:
            errors.append(raw_error)
        if not self.robot.connected:
            errors.append("机械臂未连接")
        if self.latest_robot_state is None or now - self.latest_robot_state.received_monotonic_ns > self.ROBOT_FRESH_NS:
            errors.append("机械臂反馈不新鲜")
        elif self.latest_robot_state.mode != 5:
            errors.append(f"机械臂未处于已使能空闲状态（当前 mode={self.latest_robot_state.mode}）")
        if not self.limits.valid:
            errors.append("G01/G02 固定运动参数必须为完整 ±55°、匀速 ±45°、15°/s 和 6 s")
        if self.neutral_pose is None:
            errors.append("尚未配置用于 G01/G02 旋转角度判定的标定中位")
        return errors

    def _missing_imu_names(self, presence_mask: int) -> list[str]:
        return [
            name
            for index, name in enumerate(IMU_NAMES)
            if not presence_mask & (1 << index)
        ]

    def _raw_health_error(self, now: int | None = None) -> str:
        now = time.monotonic_ns() if now is None else now
        if self.latest_raw_imu is None:
            return "尚未收到 r024 type=9 工程量原始流"
        if self.latest_register_imu is None:
            return "尚未收到 r024 type=11 寄存器原始流"
        raw_age_ns = now - self.raw_imu_ns
        register_age_ns = now - self.register_imu_ns
        if raw_age_ns > self.RAW_FRESH_NS:
            return f"type=9 原始流不新鲜（{raw_age_ns / 1e9:.2f}s）"
        if register_age_ns > self.RAW_FRESH_NS:
            return f"type=11 原始流不新鲜（{register_age_ns / 1e9:.2f}s）"
        raw_mask = self.latest_raw_imu.presence_mask & ALL_IMU_MASK
        register_mask = self.latest_register_imu.presence_mask & ALL_IMU_MASK
        if raw_mask != ALL_IMU_MASK:
            return (
                f"type=9 IMU 未全在线：mask=0x{raw_mask:04X}，缺少 "
                + "、".join(self._missing_imu_names(raw_mask))
            )
        if register_mask != ALL_IMU_MASK:
            return (
                f"type=11 IMU 未全在线：mask=0x{register_mask:04X}，缺少 "
                + "、".join(self._missing_imu_names(register_mask))
            )
        return ""

    def _raw_capture_health_error(self, now: int) -> str:
        step = self.current_step
        pending_error = ""
        for frame_type, frame, received_ns in (
            (9, self.latest_raw_imu, self.raw_imu_ns),
            (11, self.latest_register_imu, self.register_imu_ns),
        ):
            if received_ns <= self.capture_started_ns:
                pending_error = f"阶段开启后尚未收到新的 type={frame_type} 原始帧"
                break
            if frame.stage_id != step.stage_code or frame.capture_mask != step.capture_mask:
                pending_error = (
                    f"type={frame_type} 板端阶段审计不一致："
                    f"stage=0x{frame.stage_id:02X}, mask=0x{frame.capture_mask:02X}；"
                    f"期望 stage=0x{step.stage_code:02X}, mask=0x{step.capture_mask:02X}"
                )
                break
        if pending_error:
            if now - self.capture_started_ns <= self.RAW_STAGE_SYNC_NS:
                return ""
            return pending_error
        error = self._raw_health_error(now)
        if error:
            return f"采集期间{error}"
        return ""

    @Slot()
    def start_session(self) -> bool:
        if self.running:
            self.status_message.emit("工厂会话已在进行", "error")
            return False
        errors = self.preflight_errors()
        if errors:
            self.status_message.emit("；".join(errors), "error")
            return False
        now = time.monotonic_ns()
        self.gyro_limited_reference_pose = None
        raw_imu_available_at_start = bool(
            self.latest_raw_imu is not None and now - self.raw_imu_ns <= self.RAW_FRESH_NS
        )
        register_imu_available_at_start = bool(
            self.latest_register_imu is not None and now - self.register_imu_ns <= self.RAW_FRESH_NS
        )
        metadata = {
            "operator": self.operator,
            "firmware_tag": self.version.revision_tag,
            "firmware_revision": self.version.revision,
            "version_payload_version": self.version.payload_version,
            "version_features": self.version.features,
            "version_payload_hex": self.version.payload.hex(),
            "imu_model": self.version.imu_model,
            "hand_side": self.version.hand_side,
            "workflow": "QuickCal V1 Robot Control Steps",
            "workflow_source": "QuickCal_V1_Robot_Control_Steps_15dps.xlsx + r024-fac-magq type7 v4",
            "magnetic_stages_skipped": self.SKIP_MAGNETIC_STAGES,
            "magnetic_skip_reason": (
                "r024 no-magnetic QuickCal; submit directly after G06"
                if self.SKIP_MAGNETIC_STAGES
                else ""
            ),
            "roll_pitch_gyro_rate_deg_s": ROLL_PITCH_GYRO_RATE_DEG_S,
            "gyro_motion_map": {
                step_id: {"tool_axis": axis, "target_rate_deg_s": rate}
                for step_id, (axis, rate) in self.gyro_motion_map.items()
            },
            "accel_face_gate": {
                "frame": "active_tool_matches_fixture",
                "base_gravity_unit": list(self.BASE_GRAVITY_UNIT),
                "warning_deg": self.ACCEL_FACE_WARNING_DEG,
                "maximum_deg": self.ACCEL_FACE_MAX_DEG,
            },
            "neutral_pose": list(self.neutral_pose) if self.neutral_pose is not None else None,
            "diagnostic_streams": {
                "type9_available_at_start": raw_imu_available_at_start,
                "type11_available_at_start": register_imu_available_at_start,
                "required_for_calibration": True,
            },
            "g01_g02_motion_limits": {
                "applies_to": ["G01", "G02"],
                "axis": f"Tool {self.gyro_motion_map['G01'][0]}",
                "negative_soft_limit_deg": self.limits.negative_soft_limit_deg,
                "positive_soft_limit_deg": self.limits.positive_soft_limit_deg,
                "safety_margin_deg": self.limits.safety_margin_deg,
                "negative_safe_deg": self.limits.negative_safe_deg,
                "positive_safe_deg": self.limits.positive_safe_deg,
                "rate_deg_s": self.limits.rate_deg_s,
                "capture_s": self.limits.capture_s,
            },
        }
        try:
            directory = self.recorder.start(
                self.output_directory, self.product_sn, self.station_id, metadata
            )
        except Exception as exc:
            self.status_message.emit(f"无法创建会话记录：{exc}", "error")
            return False
        self.recorder.save_version(self.version)
        self.step_status = ["未开始"] * len(self.steps)
        self.step_detail = [""] * len(self.steps)
        self.current_index = 0
        self._set_step(0, "进行中", "设备自检通过")
        self._set_step(0, "完成", "连接、固件版本和机械臂状态检查通过")
        self.current_index = 1
        self.condition_stable_since_ns = 0
        self.dynamic_motion_lost_since_ns = 0
        self.commit_ack_ok = False
        self.commit_ack_received = False
        self.commit_ack_frame = None
        self.pending_report = None
        self.p1_imu_checked_frames = 0
        self.p1_imu_motion_error = ""
        self.p1_previous_accel = None
        self.p1_imu_fault_kind = ""
        self.p1_imu_fault_count = 0
        self.p1_imu_transient_issue = False
        self.p1_imu_rejected_frames = 0
        self._reset_motion_coverage()
        self.recorder.marker("session_begin", "P0", str(directory))
        self._set_state(RunState.WAIT_BEGIN_ACK)
        self.operation_deadline_ns = time.monotonic_ns() + self.ACK_TIMEOUT_NS
        if not self.glove.send_command(CMD_MCAL_BEGIN):
            self.abort("MCAL_BEGIN 发送失败")
            return False
        self.status_message.emit("等待固件确认 MCAL_BEGIN", "info")
        if self.SKIP_MAGNETIC_STAGES:
            self.status_message.emit(
                "r024 无磁 QuickCal：G06 后禁止发送 M01-M04，回中静止后自动提交",
                "good",
            )
        else:
            self.status_message.emit(
                "r024 磁标定 QuickCal：G06 后继续执行 M01-M04，提交时磁质量必须通过",
                "good",
            )
        return True

    @Slot()
    def confirm_current_action(self) -> bool:
        if self.state != RunState.READY:
            self.status_message.emit("当前状态不能开始动作采集", "error")
            return False
        step = self.current_step
        if step.step_id.startswith("M") and not self.environment_confirmed:
            self.status_message.emit("磁翻转前必须确认磁环境与夹具状态", "error")
            return False
        condition_error = self._check_motion_condition(step)
        if condition_error:
            self.status_message.emit(condition_error, "error")
            return False
        raw_error = self._raw_health_error()
        if raw_error:
            self.status_message.emit(f"原始 IMU 预检未通过：{raw_error}", "error")
            return False
        alignment = self._accel_face_alignment(step.step_id, self.latest_robot_state.pose)
        if alignment is not None:
            angle_deg, gravity_tool, face_name = alignment
            alignment_detail = self._format_accel_alignment(
                face_name, angle_deg, gravity_tool
            )
            self.recorder.marker("accel_face_alignment", step.step_id, alignment_detail)
            if angle_deg > self.ACCEL_FACE_WARNING_DEG:
                self.status_message.emit(
                    f"六面姿态警告：{alignment_detail}；允许采样，但建议调整到 "
                    f"≤{self.ACCEL_FACE_WARNING_DEG:.1f}°",
                    "warn",
                )
        if step.step_id == "S01":
            return self.commit()
        if step.step_id == "S02":
            self._set_step(self.current_index, "完成", "记录已归档")
            reason = "11 路 Gyro/Acc 标定与 Flash 回读通过，记录已归档"
            self.recorder.marker("session_complete", "S02", reason)
            self.recorder.finish(True, reason, self._step_results())
            self._set_state(RunState.COMPLETE)
            self.finished.emit(True, reason)
            return True
        if not step.sample_enabled or step.stage_code is None:
            self._complete_non_capture_step("操作员确认完成")
            return True
        self.valid_mag_pairs_this_step = 0
        self.invalid_mag_pairs_this_step = 0
        self._capture_fault = ""
        self._reset_motion_coverage()
        detail = "等待固件开启采集阶段"
        if alignment is not None:
            angle_deg, gravity_tool, face_name = alignment
            detail += "；" + self._format_accel_alignment(face_name, angle_deg, gravity_tool)
        self._set_step(self.current_index, "进行中", detail)
        self.recorder.marker("stage_open_request", step.step_id, f"stage=0x{step.stage_code:02X} mask=0x{step.capture_mask:02X}")
        self._set_state(RunState.WAIT_STAGE_OPEN)
        self.operation_deadline_ns = time.monotonic_ns() + self.ACK_TIMEOUT_NS
        return self.glove.send_command(
            CMD_MCAL_STAGE, step.stage_code, bytes((step.capture_mask,))
        )

    def commit(self) -> bool:
        if self.current_step.step_id != "S01" or self.state != RunState.READY:
            self.status_message.emit("动作清单尚未到提交步骤", "error")
            return False
        prior_statuses = self.step_status[1:self.current_index]
        if any(status not in ("完成", "跳过") for status in prior_statuses):
            self.status_message.emit("提交前仍有未完成或未明确跳过的步骤", "error")
            return False
        self._set_step(self.current_index, "进行中", "固件正在求解并写入 Flash")
        commit_detail = (
            "r024 no-magnetic QuickCal; M01-M04 forbidden/skipped"
            if self.SKIP_MAGNETIC_STAGES
            else "all workflow gates passed"
        )
        self.recorder.marker("commit_request", "S01", commit_detail)
        self.commit_ack_ok = False
        self.commit_ack_received = False
        self.commit_ack_frame = None
        self.pending_report = None
        self._set_state(RunState.WAIT_COMMIT_ACK)
        self.operation_deadline_ns = time.monotonic_ns() + self.COMMIT_TIMEOUT_NS
        return self.glove.send_command(CMD_MCAL_COMMIT)

    @Slot(str)
    def abort(self, reason: str = "操作员中止") -> None:
        if self._aborting or self.state in (RunState.IDLE, RunState.COMPLETE, RunState.ABORTED):
            return
        self._aborting = True
        self.gyro_limited_reference_pose = None
        try:
            self._set_state(RunState.ABORTED)
            try:
                self.robot.stop()
            finally:
                self.glove.send_command(CMD_MCAL_ABORT)
            if self.current_index < len(self.steps):
                self._set_step(self.current_index, "失败", reason)
            self.recorder.marker("session_abort", self.current_step.step_id, reason)
            self.recorder.finish(False, reason, self._step_results())
            self.finished.emit(False, reason)
        finally:
            self._aborting = False

    def _complete_non_capture_step(self, detail: str) -> None:
        self._set_step(self.current_index, "完成", detail)
        self.recorder.marker("step_complete", self.current_step.step_id, detail)
        self._advance()

    def _advance(self) -> None:
        self.current_index += 1
        if self.SKIP_MAGNETIC_STAGES:
            while (
                self.current_index < len(self.steps)
                and self.current_step.step_id.startswith("M")
            ):
                detail = "r024 无磁 QuickCal：协议禁止向固件发送该磁标定阶段"
                self._set_step(self.current_index, "跳过", detail)
                self.recorder.marker(
                    "stage_skipped", self.current_step.step_id, detail
                )
                self.current_index += 1
        if self.current_index >= len(self.steps):
            return
        self.condition_stable_since_ns = 0
        self.dynamic_motion_lost_since_ns = 0
        self._set_state(RunState.READY)
        self.current_step_changed.emit(self.current_index, self.current_step)
        self.progress_changed.emit(0, "等待动作条件")
        self.status_message.emit(
            f"下一步 {self.current_step.step_id}：{self.current_step.name}", "info"
        )

    def _set_state(self, state: RunState) -> None:
        self.state = state
        self.state_changed.emit(state.value)

    def _set_step(self, index: int, status: str, detail: str) -> None:
        self.step_status[index] = status
        self.step_detail[index] = detail
        self.step_status_changed.emit(index, status, detail)

    def _step_results(self) -> dict[str, str]:
        return {
            step.step_id: f"{status}: {detail}".rstrip(": ")
            for step, status, detail in zip(self.steps, self.step_status, self.step_detail)
        }

    def _check_motion_condition(self, step: QuickCalStep, *, require_settle: bool = True) -> str:
        state = self.latest_robot_state
        if state is None or time.monotonic_ns() - state.received_monotonic_ns > self.ROBOT_FRESH_NS:
            self.condition_stable_since_ns = 0
            return "机械臂反馈已超时"
        if state.mode == self.ROBOT_ERROR_MODE:
            self.condition_stable_since_ns = 0
            return "机械臂处于报警状态（mode=9）"
        if state.mode == self.ROBOT_COLLISION_MODE:
            self.condition_stable_since_ns = 0
            return "机械臂碰撞检测已触发（mode=11）"
        if state.mode not in self.ROBOT_CAPTURE_MODES:
            self.condition_stable_since_ns = 0
            return f"机械臂状态不允许采集（mode={state.mode}）"
        if step.step_id in (
            "P1", "A01", "A02", "A03", "A04", "A05", "A06", "S01", "S02"
        ):
            if (
                state.mode != self.ROBOT_MODE_IDLE
                or state.angular_speed_norm > 0.8
                or state.linear_speed_norm > 1.0
            ):
                self.condition_stable_since_ns = 0
                return (
                    f"该步骤要求机械臂静止，当前线速度 {state.linear_speed_norm:.2f} mm/s，"
                    f"角速度 {state.angular_speed_norm:.2f}°/s"
                )
        if step.step_id == "P1" and self.p1_imu_checked_frames:
            if time.monotonic_ns() - self.raw_imu_ns > self.RAW_FRESH_NS:
                self.condition_stable_since_ns = 0
                return "P1 type=9 IMU 数据流在检测期间中断"
            if self.p1_imu_motion_error:
                self.condition_stable_since_ns = 0
                return self.p1_imu_motion_error
        alignment = self._accel_face_alignment(step.step_id, state.pose)
        if alignment is not None:
            angle_deg, gravity_tool, face_name = alignment
            if angle_deg > self.ACCEL_FACE_MAX_DEG:
                self.condition_stable_since_ns = 0
                return (
                    f"{self._format_accel_alignment(face_name, angle_deg, gravity_tool)}，"
                    f"超过允许值 {self.ACCEL_FACE_MAX_DEG:.1f}°；请调整夹具姿态"
                )
        if step.step_id in ("G01", "G02") and not self.limits.valid:
            self.condition_stable_since_ns = 0
            return "G01/G02 配置轴限位计算未通过"
        if step.step_id.startswith("G"):
            gyro_error = self._gyro_motion_error(step, state)
            if gyro_error:
                self.condition_stable_since_ns = 0
                return gyro_error
        if step.step_id.startswith("M"):
            # Magnetic stages are opened at the taught neutral pose.  The
            # window starts the timed jog after the open ACK, so the work-order
            # duration maps to 0→endpoint→0 without an uncounted lead-in angle.
            if (
                state.mode != self.ROBOT_MODE_IDLE
                or state.angular_speed_norm > 0.8
                or state.linear_speed_norm > 1.0
            ):
                self.condition_stable_since_ns = 0
                return (
                    "磁阶段必须从标定中位静止开启，当前线速度 "
                    f"{state.linear_speed_norm:.2f} mm/s，角速度 "
                    f"{state.angular_speed_norm:.2f}°/s"
                )
        required_stable_s = 0.0
        if step.step_id in ("P1", "A01", "A02", "A03", "A04", "A05", "A06"):
            required_stable_s = step.settle_s
        elif step.step_id.startswith("G"):
            required_stable_s = step.settle_s
        elif step.step_id.startswith("M"):
            required_stable_s = 0.0
        elif step.step_id in ("S01", "S02"):
            required_stable_s = 0.5
        now = time.monotonic_ns()
        if require_settle and required_stable_s > 0:
            if self.condition_stable_since_ns == 0:
                self.condition_stable_since_ns = now
            stable_s = (now - self.condition_stable_since_ns) / 1_000_000_000.0
            if stable_s < required_stable_s:
                return f"动作条件已满足，请继续保持 {required_stable_s - stable_s:.1f} s"
        return ""

    def _gyro_motion_error(self, step: QuickCalStep, state) -> str:
        axis, target = self.gyro_motion_map[step.step_id]
        axis_index = {"Rx": 0, "Ry": 1, "Rz": 2}[axis]
        tool_speed = self._tool_angular_speed(state.pose, state.angular_speed)
        measured = tool_speed[axis_index]
        tolerance = max(3.0, abs(target) * 0.20)
        if abs(measured - target) > tolerance:
            return (
                f"匀速条件未满足：目标 Tool {axis}={target:+.1f}°/s，"
                f"实际 {measured:+.1f}°/s，允许偏差 ±{tolerance:.1f}°/s"
            )
        return ""

    def _reset_motion_coverage(self) -> None:
        self.motion_coverage = {
            "x_positive": False,
            "x_negative": False,
            "y_positive": False,
            "y_negative": False,
            "z_positive": False,
            "z_negative": False,
        }

    def _update_motion_coverage(self, state) -> None:
        if not self.current_step.step_id.startswith("M"):
            return
        tool_speed = self._tool_angular_speed(state.pose, state.angular_speed)
        threshold = self.MAG_COVERAGE_SPEED_DEG_S
        for axis_name in "xyz":
            value = tool_speed["xyz".index(axis_name)]
            if value >= threshold:
                self.motion_coverage[f"{axis_name}_positive"] = True
            elif value <= -threshold:
                self.motion_coverage[f"{axis_name}_negative"] = True

    def _motion_coverage_error(self, step_id: str) -> str:
        if not step_id.startswith("M"):
            return ""
        if step_id == "M04":
            return ""
        if step_id == "M01":
            required = ("z_positive", "z_negative", "y_positive", "y_negative")
            missing = [name for name in required if not self.motion_coverage[name]]
            if missing:
                return (
                    "M01 四轴叠加覆盖不足：J2/J3/J4 俯仰与 J6 往复 "
                    "必须均包含正、反向运动"
                )
            return ""
        if not (
            self.motion_coverage["x_positive"]
            and self.motion_coverage["x_negative"]
        ):
            return f"{step_id} Yaw 往返覆盖不足：Tool Rx 必须包含正、反向运动"
        return ""

    def _relative_tool_axis_deg(self, pose, axis: str) -> float | None:
        reference_pose = (
            self.gyro_limited_reference_pose
            if self.current_step.step_id in ("G01", "G02")
            and self.gyro_limited_reference_pose is not None
            else self.neutral_pose
        )
        if reference_pose is None:
            return None
        neutral_axes = self._tool_axes(reference_pose)
        current_axes = self._tool_axes(pose)
        if axis == "Rx":
            cosine = sum(a * b for a, b in zip(neutral_axes[1], current_axes[1]))
            sine = sum(a * b for a, b in zip(neutral_axes[2], current_axes[1]))
        elif axis == "Ry":
            cosine = sum(a * b for a, b in zip(neutral_axes[0], current_axes[0]))
            sine = sum(a * b for a, b in zip(neutral_axes[0], current_axes[2]))
        elif axis == "Rz":
            cosine = sum(a * b for a, b in zip(neutral_axes[0], current_axes[0]))
            sine = sum(a * b for a, b in zip(neutral_axes[1], current_axes[0]))
        else:
            return None
        return math.degrees(math.atan2(sine, cosine))

    def _relative_yaw_deg(self, pose) -> float | None:
        return self._relative_tool_axis_deg(pose, "Rz")

    def _mag_motion_error(self, state) -> str:
        tool_speed = self._tool_angular_speed(state.pose, state.angular_speed)
        step_id = self.current_step.step_id
        if step_id == "M01":
            if not any(
                abs(tool_speed[index]) >= self.MAG_COVERAGE_SPEED_DEG_S
                for index in (1, 2)
            ):
                return "M01 要求机械臂保持固定 XYZ 的 Tool Ry 与 J6 方向连续慢速往复"
        elif step_id in ("M02", "M03"):
            # The window motion is coverage-based, not a requirement to keep
            # jogging until the final millisecond.  Once both Rx directions
            # have been observed, the station may brake before neutral and
            # remain still while the fixed ten-second capture window closes.
            if (
                self.motion_coverage["x_positive"]
                and self.motion_coverage["x_negative"]
            ):
                return ""
            if abs(tool_speed[0]) < self.MAG_COVERAGE_SPEED_DEG_S:
                return f"{step_id} 要求机械臂保持 Tool Rx 慢速往返"
        return ""

    @staticmethod
    def _tool_axes(pose) -> tuple[tuple[float, float, float], ...]:
        rx, ry, rz = (math.radians(float(value)) for value in pose[3:6])
        return (
            (math.cos(rz) * math.cos(ry), math.sin(rz) * math.cos(ry), -math.sin(ry)),
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

    @classmethod
    def _tool_angular_speed(cls, pose, angular_speed) -> tuple[float, float, float]:
        axes = cls._tool_axes(pose)
        return tuple(sum(float(value) * direction for value, direction in zip(angular_speed, axis)) for axis in axes)

    @classmethod
    def _accel_face_alignment(
        cls, step_id: str, pose
    ) -> tuple[float, tuple[float, float, float], str] | None:
        target = cls.ACCEL_FACE_TARGETS.get(step_id)
        if target is None:
            return None
        face_name, target_vector = target
        axes = cls._tool_axes(pose)
        gravity_tool = tuple(
            sum(base_value * axis_value for base_value, axis_value in zip(cls.BASE_GRAVITY_UNIT, axis))
            for axis in axes
        )
        dot = sum(value * expected for value, expected in zip(gravity_tool, target_vector))
        angle_deg = math.degrees(math.acos(max(-1.0, min(1.0, dot))))
        return angle_deg, gravity_tool, face_name

    @staticmethod
    def _format_accel_alignment(face_name, angle_deg, gravity_tool) -> str:
        direction = ", ".join(f"{value:+.3f}" for value in gravity_tool)
        return f"目标 {face_name}，姿态偏差 {angle_deg:.2f}°，重力方向(Tool)=({direction})"

    def _fail_current_step(self, reason: str) -> None:
        self.abort(f"{reason}；当前固件会话已中止，请从头重新开始标定")

    @Slot()
    def _tick(self) -> None:
        now = time.monotonic_ns()
        timed_out_state = self.state
        if timed_out_state in (
            RunState.WAIT_BEGIN_ACK,
            RunState.WAIT_STAGE_OPEN,
            RunState.WAIT_STAGE_CLOSE,
            RunState.WAIT_COMMIT_ACK,
            RunState.WAIT_REPORT,
        ) and self.operation_deadline_ns and now > self.operation_deadline_ns:
            firmware_tag = getattr(self.version, "revision_tag", "未知版本")
            timeout_reason = {
                RunState.WAIT_BEGIN_ACK: (
                    f"MCAL_BEGIN 1 秒内未收到 ACK；固件 {firmware_tag} 可能不支持工厂标定协议"
                ),
                RunState.WAIT_STAGE_OPEN: (
                    f"MCAL_STAGE 开启命令 1 秒内未收到 ACK；固件 {firmware_tag} 未实现或不兼容阶段协议 0x13"
                ),
                RunState.WAIT_STAGE_CLOSE: "MCAL_STAGE 关闭命令 1 秒内未收到 ACK",
                RunState.WAIT_COMMIT_ACK: "MCAL_COMMIT 2 秒内未收到 ACK",
                RunState.WAIT_REPORT: "Commit ACK 后 10 秒内未收到 type=7 最终质量报告",
            }[timed_out_state]
            if timed_out_state in (RunState.WAIT_STAGE_OPEN, RunState.WAIT_STAGE_CLOSE):
                self._fail_current_step(timeout_reason)
            else:
                self.abort(timeout_reason)
            return
        if self.state == RunState.READY and self.current_step.step_id == "P1":
            condition_error = self._check_motion_condition(self.current_step)
            if condition_error:
                self.progress_changed.emit(0, f"P1 自动静止检测：{condition_error}")
                return
            imu_gate = (
                f"已联合核验 type=9（{self.p1_imu_checked_frames} 帧）"
                if self.p1_imu_checked_frames
                else "当前无 type=9，上位机使用机械臂静止门控，阶段末由固件质检"
            )
            self.status_message.emit(f"P1 已连续静止 {self.current_step.settle_s:.1f} 秒；{imu_gate}，自动开启采集", "good")
            self.confirm_current_action()
            return
        if (
            self.SKIP_MAGNETIC_STAGES
            and self.state == RunState.READY
            and self.current_step.step_id == "S01"
        ):
            condition_error = self._check_motion_condition(self.current_step)
            if condition_error:
                self.progress_changed.emit(
                    0, f"跳过 M01-M04，等待自动提交：{condition_error}"
                )
                return
            self.status_message.emit(
                "r024 已按协议跳过 M01-M04，机械臂静止且原始流健康，自动发送 MCAL_COMMIT",
                "good",
            )
            self.confirm_current_action()
            return
        if self.state != RunState.CAPTURING:
            return
        duration_ns = max(1, self.capture_deadline_ns - self.capture_started_ns)
        percent = min(100, int((now - self.capture_started_ns) * 100 / duration_ns))
        remaining = max(0.0, (self.capture_deadline_ns - now) / 1_000_000_000.0)
        self.progress_changed.emit(percent, f"剩余 {remaining:.1f} s")
        if not self._capture_fault:
            self._capture_fault = self._live_capture_fault(now)
        if self._capture_fault:
            self._request_stage_close()
        elif now >= self.capture_deadline_ns:
            self._request_stage_close()

    def _live_capture_fault(self, now: int) -> str:
        raw_error = self._raw_capture_health_error(now)
        if raw_error:
            return raw_error
        step_id = self.current_step.step_id
        if step_id.startswith(("G", "M")):
            state = self.latest_robot_state
            if state is None or now - state.received_monotonic_ns > self.ROBOT_FRESH_NS:
                return "动态采集期间机械臂反馈中断"
            if state.mode == self.ROBOT_COLLISION_MODE:
                return "动态采集期间机械臂碰撞检测已触发（mode=11）"
            if state.mode == self.ROBOT_ERROR_MODE:
                return "动态采集期间机械臂报警（mode=9）"
            if state.mode not in self.ROBOT_CAPTURE_MODES:
                return f"动态采集期间机械臂状态异常（mode={state.mode}）"
            gyro_rate = ROLL_PITCH_GYRO_RATE_DEG_S
            max_speed = 90.0 if step_id.startswith("M") else max(20.0, gyro_rate * 1.8)
            if state.angular_speed_norm > max_speed:
                return f"动态采集期间角速度过高：{state.angular_speed_norm:.1f}°/s"
            if step_id.startswith("G"):
                dynamic_error = self._gyro_motion_error(self.current_step, state)
                if not dynamic_error and step_id in ("G01", "G02"):
                    configured_axis = self.gyro_motion_map[step_id][0]
                    axis_deg = self._relative_tool_axis_deg(
                        state.pose, configured_axis
                    )
                    if axis_deg is None:
                        dynamic_error = "尚未配置用于 G01/G02 旋转轴判定的标定中位"
                    elif not (
                        self.limits.negative_safe_deg - 1.0
                        <= axis_deg
                        <= self.limits.positive_safe_deg + 1.0
                    ):
                        dynamic_error = (
                            f"Tool {configured_axis} 匀速采集越过安全位："
                            f"当前 {axis_deg:+.1f}°，"
                            f"范围 {self.limits.negative_safe_deg:+.1f}° 至 "
                            f"{self.limits.positive_safe_deg:+.1f}°"
                        )
            elif step_id == "M04":
                dynamic_error = self._check_motion_condition(self.current_step)
            else:
                dynamic_error = self._mag_motion_error(state)
            if dynamic_error:
                if self.dynamic_motion_lost_since_ns == 0:
                    self.dynamic_motion_lost_since_ns = now
                grace_ns = (
                    self.MAG_DYNAMIC_DEVIATION_GRACE_NS
                    if step_id.startswith("M")
                    else self.DYNAMIC_DEVIATION_GRACE_NS
                )
                if now - self.dynamic_motion_lost_since_ns > grace_ns:
                    return dynamic_error
            else:
                self.dynamic_motion_lost_since_ns = 0
        else:
            # The pre-capture settle interval is a start gate only.  During a
            # capture we still require a fresh, stationary robot and healthy P1
            # stream, but must not reinterpret a reset settle timer as a fault.
            motion_error = self._check_motion_condition(
                self.current_step, require_settle=False
            )
            if motion_error:
                return motion_error
        return ""

    def _request_stage_close(self) -> None:
        step = self.current_step
        if self.state != RunState.CAPTURING or step.stage_code is None:
            return
        if not self._capture_fault and step.step_id.startswith("M"):
            self._capture_fault = self._motion_coverage_error(step.step_id)
        if self._capture_fault:
            try:
                self.robot.stop()
            except Exception:
                pass
        self.recorder.marker("stage_close_request", step.step_id, self._capture_fault or "capture complete")
        self._set_state(RunState.WAIT_STAGE_CLOSE)
        self.operation_deadline_ns = time.monotonic_ns() + self.ACK_TIMEOUT_NS
        self.glove.send_command(CMD_MCAL_STAGE, step.stage_code, b"\x00")

    @Slot(object)
    def on_ack(self, frame) -> None:
        self.recorder.marker("ack", self.current_step.step_id, f"cmd=0x{frame.cmd:02X} status={frame.status} d0={frame.detail0} d1={frame.detail1}")
        self.status_message.emit(
            f"RX ACK cmd=0x{frame.cmd:02X} status={frame.status} detail0=0x{frame.detail0:02X} detail1=0x{frame.detail1:02X}",
            "good" if frame.status == 0 else "error",
        )
        if frame.cmd == CMD_MCAL_BEGIN and self.state == RunState.WAIT_BEGIN_ACK:
            if frame.status != 0:
                self.abort(f"固件拒绝 MCAL_BEGIN，status={frame.status}")
                return
            self._set_state(RunState.READY)
            self.current_step_changed.emit(self.current_index, self.current_step)
            self.status_message.emit("工厂会话已建立，请执行 P1 静止动作", "good")
            return
        if frame.cmd == CMD_MCAL_STAGE and self.state == RunState.WAIT_STAGE_OPEN:
            step = self.current_step
            if frame.detail0 != step.stage_code or frame.status != 0 or frame.detail1 != step.capture_mask:
                self._fail_current_step(
                    f"阶段 0x{step.stage_code:02X} 开启失败：status={frame.status}, mask=0x{frame.detail1:02X}"
                )
                return
            self.capture_started_ns = time.monotonic_ns()
            self.capture_deadline_ns = self.capture_started_ns + int(step.capture_s * 1_000_000_000)
            self.dynamic_motion_lost_since_ns = 0
            self.recorder.marker("capture_start", step.step_id, f"duration={step.capture_s:.3f}s")
            self._set_state(RunState.CAPTURING)
            self.progress_changed.emit(0, f"采集 {step.capture_s:.1f} s")
            return
        if frame.cmd == CMD_MCAL_STAGE and self.state == RunState.WAIT_STAGE_CLOSE:
            step = self.current_step
            if self._capture_fault:
                self._fail_current_step(self._capture_fault)
                return
            if frame.detail0 != step.stage_code or frame.status != 0 or frame.detail1 != 11:
                detail = (
                    f"P1 合格 IMU={frame.detail1}/11"
                    if step.step_id == "P1" and frame.status == 15
                    else f"detail1=0x{frame.detail1:02X}"
                )
                self._fail_current_step(
                    f"阶段 0x{step.stage_code:02X} 关闭失败："
                    f"status={frame.status}，{detail}"
                )
                return
            coverage_error = self._motion_coverage_error(step.step_id)
            if coverage_error:
                self._fail_current_step(coverage_error)
                return
            detail = "阶段正常关闭（固件返回值 11）；最终质量待 MCAL_COMMIT/type=7"
            if step.step_id == "P1":
                detail += (
                    f"；上位机联合核验 type=9 {self.p1_imu_checked_frames} 帧，"
                    f"剔除异常帧 {self.p1_imu_rejected_frames} 帧"
                )
            if self.valid_mag_pairs_this_step or self.invalid_mag_pairs_this_step:
                detail += (
                    f"；可选 type=12 诊断：有效 {self.valid_mag_pairs_this_step}，"
                    f"无效 {self.invalid_mag_pairs_this_step}"
                )
            self._set_step(self.current_index, "完成", detail)
            self.recorder.marker("capture_complete", step.step_id, detail)
            self._advance()
            return
        if frame.cmd == CMD_MCAL_COMMIT and self.state in (RunState.WAIT_COMMIT_ACK, RunState.WAIT_REPORT):
            self.commit_ack_received = True
            self.commit_ack_frame = frame
            self.recorder.save_commit_ack(frame)
            self.commit_ack_ok = (
                frame.status == 0
                and frame.detail0 == 11
                and frame.seq != 0
            )
            if frame.status not in (0, 5, 11):
                self.abort(
                    f"MCAL_COMMIT 被拒绝：status={frame.status}，"
                    f"detail0=0x{frame.detail0:02X}，detail1=0x{frame.detail1:02X}"
                )
                return
            self._set_state(RunState.WAIT_REPORT)
            self.operation_deadline_ns = time.monotonic_ns() + self.REPORT_TIMEOUT_NS
            if self.pending_report is not None:
                self._finish_report(self.pending_report)

    @Slot(object)
    def on_mcal_report(self, report) -> None:
        self.recorder.save_report(report)
        if self.state not in (RunState.WAIT_COMMIT_ACK, RunState.WAIT_REPORT):
            return
        self.pending_report = report
        # r024 会在 COMMIT status=5/11 后继续发送 type=7；失败 ACK 也必须
        # 进入报告解析，不能等待到超时后丢失逐 IMU 的拒绝原因。
        if self.commit_ack_received:
            self._finish_report(report)

    def _finish_report(self, report) -> None:
        passed = self.commit_ack_ok and report.factory_pass
        if not passed:
            gyro_failures = []
            for index, item in enumerate(report.gyro_quality):
                if (
                    item.ok
                    and item.reject_flags == 0
                    and item.window_count >= EXPECTED_GYRO_SEGMENTS
                ):
                    continue
                reason_items = list(item.reject_reasons)
                if (
                    item.window_count < EXPECTED_GYRO_SEGMENTS
                    and "有效窗口不足" not in reason_items
                ):
                    reason_items.append("有效窗口不足")
                if not item.ok and not reason_items:
                    reason_items.append("固件判定失败但未给出 rejectFlags")
                reasons = "、".join(reason_items)
                gyro_failures.append(
                    f"{IMU_NAMES[index]}[{reasons}; nseg={item.window_count}; "
                    f"RMS={item.rms_mdeg / 1000:.3f}°; offdiag={item.max_off_axis / 1000:.3f}]"
                )
            accel_failures = [
                f"{IMU_NAMES[index]}[residual={item.residual_x1000 / 1000:.3f}; "
                f"scale={item.max_abs_scale_error_x1000 / 1000:.3f}; "
                f"cross={item.max_cross_axis_x1000 / 1000:.3f}; "
                f"bias=({item.bias_x_mg},{item.bias_y_mg},{item.bias_z_mg})mg]"
                for index, item in enumerate(report.accel_quality)
                if not item.ok
            ]
            mag_failure = ""
            if not report.mag_all_ok:
                reasons = list(report.mag_quality.reject_reasons)
                if not reasons:
                    reasons.append("固件磁质量判定失败")
                anchor = report.mag_quality.slots[0] if report.mag_quality.slots else None
                coverage = (
                    f"samples={anchor.sample_count}; span=({anchor.span_x},{anchor.span_y},{anchor.span_z})"
                    if anchor
                    else "无 MMC5983MA 质量数据"
                )
                mag_failure = f"Mag失败：{'、'.join(reasons)}；{coverage}"
            raw_issues = self.recorder.raw_diagnostic_issues(IMU_NAMES)
            ack = self.commit_ack_frame
            ack_text = (
                f"ACK status={ack.status}, Gyro={ack.detail0}/11, "
                f"Accel/均值字段={ack.detail1}, calSeq={ack.seq}"
                if ack is not None
                else "未收到 COMMIT ACK"
            )
            details = [
                ack_text,
                f"type=7 v{report.version}, status={report.status}",
            ]
            if gyro_failures:
                details.append("Gyro失败：" + "；".join(gyro_failures))
            if accel_failures:
                details.append("Accel失败：" + "；".join(accel_failures))
            if mag_failure:
                details.append(mag_failure)
            if raw_issues:
                details.append("原始流提示：" + "；".join(raw_issues[:12]))
            self.abort(
                "参数求解/Flash 未通过｜" + "｜".join(details)
            )
            return
        detail = f"Gyro 11/11，Accel 11/11，Mag通过，Flash seq={report.flash_sequence}，平均 RMS={report.mean_rms_mdeg / 1000:.3f}°"
        self._set_step(self.current_index, "完成", detail)
        self.recorder.marker("commit_complete", "S01", detail)
        self._advance()

    @Slot(object)
    def on_raw_imu(self, frame) -> None:
        self.latest_raw_imu = frame
        self.raw_imu_ns = time.monotonic_ns()
        if self.current_step.step_id == "P1" and self.state in (
            RunState.READY,
            RunState.WAIT_STAGE_OPEN,
            RunState.CAPTURING,
        ):
            self.p1_imu_checked_frames += 1
            self.p1_imu_motion_error = self._evaluate_p1_imu_frame(frame)
            if self.p1_imu_transient_issue:
                # A suspect frame is not allowed to contribute to the two-second
                # stillness timer, but one isolated sample glitch must not abort
                # an active 30-second P1 capture either.
                self.condition_stable_since_ns = 0
            if self.p1_imu_motion_error:
                self.condition_stable_since_ns = 0
                if self.state in (RunState.WAIT_STAGE_OPEN, RunState.CAPTURING) and not self._capture_fault:
                    self._capture_fault = self.p1_imu_motion_error
        if self.state in (RunState.CAPTURING, RunState.WAIT_STAGE_CLOSE):
            self.recorder.raw_imu(self.current_step.step_id, frame)

    def _confirm_p1_imu_fault(self, kind: str, detail: str) -> str:
        if self.p1_imu_fault_kind == kind:
            self.p1_imu_fault_count += 1
        else:
            self.p1_imu_fault_kind = kind
            self.p1_imu_fault_count = 1
        self.p1_imu_transient_issue = (
            self.p1_imu_fault_count < self.P1_IMU_FAULT_CONFIRM_FRAMES
        )
        if self.p1_imu_transient_issue:
            return ""
        return (
            f"{detail}；已连续 {self.p1_imu_fault_count} 帧，"
            "排除单帧采样毛刺后确认"
        )

    def _clear_p1_imu_fault(self) -> None:
        self.p1_imu_fault_kind = ""
        self.p1_imu_fault_count = 0
        self.p1_imu_transient_issue = False

    def _evaluate_p1_imu_frame(self, frame) -> str:
        if frame.presence_mask & ALL_IMU_MASK != ALL_IMU_MASK or len(frame.samples) != 11:
            self.p1_previous_accel = None
            return self._confirm_p1_imu_fault(
                "presence",
                f"P1 IMU 在线不足：presence_mask=0x{frame.presence_mask & ALL_IMU_MASK:04X}",
            )
        accel_vectors = tuple(
            (float(sample.ax), float(sample.ay), float(sample.az))
            for sample in frame.samples
        )
        accel_norms = tuple(math.sqrt(ax * ax + ay * ay + az * az) for ax, ay, az in accel_vectors)

        # A mounted, powered IMU cannot measure 0 g on all three axes while the
        # other ten lanes measure gravity. This signature has repeatedly been
        # observed together with the false 0.274/9.697 rad/s spikes. Discard it
        # before gyro and inter-frame acceleration checks; also clear the prior
        # accel snapshot so recovery from the bad frame is not called vibration.
        corrupt_indices = tuple(
            index
            for index, value in enumerate(accel_norms)
            if value < self.P1_IMU_CORRUPT_ACCEL_NORM_MAX_G
        )
        if corrupt_indices:
            self.p1_previous_accel = None
            self.p1_imu_rejected_frames += 1
            names = ", ".join(
                IMU_NAMES[index] if index < len(IMU_NAMES) else str(index)
                for index in corrupt_indices
            )
            return self._confirm_p1_imu_fault(
                "corrupt-zero-accel",
                f"P1 IMU 原始帧异常：{names} 加速度接近 0 g",
            )

        max_gyro = max(
            math.sqrt(sample.gx * sample.gx + sample.gy * sample.gy + sample.gz * sample.gz)
            for sample in frame.samples
        )
        if max_gyro > self.P1_IMU_GYRO_MAX_RAD_S:
            self.p1_previous_accel = accel_vectors
            return self._confirm_p1_imu_fault(
                "gyro-motion",
                (
                    f"P1 IMU 检测到持续转动：最大角速度 {max_gyro:.3f} rad/s，"
                    f"限值 {self.P1_IMU_GYRO_MAX_RAD_S:.3f} rad/s"
                ),
            )
        invalid_norm = next(
            (
                value
                for value in accel_norms
                if not self.P1_IMU_ACCEL_NORM_MIN_G
                <= value
                <= self.P1_IMU_ACCEL_NORM_MAX_G
            ),
            None,
        )
        if invalid_norm is not None:
            self.p1_previous_accel = accel_vectors
            return self._confirm_p1_imu_fault(
                "accel-norm",
                (
                    f"P1 IMU 加速度幅值持续异常：{invalid_norm:.3f} g，允许 "
                    f"{self.P1_IMU_ACCEL_NORM_MIN_G:.2f}～{self.P1_IMU_ACCEL_NORM_MAX_G:.2f} g"
                ),
            )
        previous = self.p1_previous_accel
        self.p1_previous_accel = accel_vectors
        if previous is None:
            self._clear_p1_imu_fault()
            return ""
        max_delta = max(
            math.sqrt(
                (current[0] - prior[0]) ** 2
                + (current[1] - prior[1]) ** 2
                + (current[2] - prior[2]) ** 2
            )
            for current, prior in zip(accel_vectors, previous)
        )
        if max_delta > self.P1_IMU_ACCEL_DELTA_MAX_G:
            return self._confirm_p1_imu_fault(
                "accel-vibration",
                (
                    f"P1 IMU 检测到持续振动：相邻帧最大加速度变化 {max_delta:.3f} g，"
                    f"限值 {self.P1_IMU_ACCEL_DELTA_MAX_G:.3f} g"
                ),
            )
        self._clear_p1_imu_fault()
        return ""

    @Slot(object)
    def on_register_raw_imu(self, frame) -> None:
        self.latest_register_imu = frame
        self.register_imu_ns = time.monotonic_ns()
        if self.state in (RunState.CAPTURING, RunState.WAIT_STAGE_CLOSE):
            self.recorder.register_imu(self.current_step.step_id, frame)

    @Slot(object)
    def on_raw_mag(self, frame) -> None:
        self.latest_raw_mag = frame
        self.raw_mag_ns = time.monotonic_ns()
        if self.state == RunState.CAPTURING:
            self.recorder.raw_mag(self.current_step.step_id, frame)

    @Slot(object)
    def on_mag_pair(self, frame) -> None:
        self.latest_mag_pair = frame
        self.mag_pair_ns = time.monotonic_ns()
        if self.state == RunState.CAPTURING:
            self.recorder.mag_pair(self.current_step.step_id, frame)
            if frame.flags & 0x01:
                self.valid_mag_pairs_this_step += 1
            else:
                self.invalid_mag_pairs_this_step += 1

    @Slot(object)
    def on_version(self, frame) -> None:
        self.version = frame

    @Slot(object)
    def on_glove_tx_bytes(self, data: bytes) -> None:
        self.recorder.cdc_bytes("TX", data)

    @Slot(object)
    def on_glove_rx_bytes(self, data: bytes) -> None:
        self.recorder.cdc_bytes("RX", data)

    @Slot(bool, str)
    def on_glove_connection_changed(self, _connected: bool, _detail: str) -> None:
        """A version/data gate is valid only for the current serial connection."""
        self.version = None
        self.latest_raw_imu = None
        self.latest_register_imu = None
        self.latest_raw_mag = None
        self.latest_mag_pair = None
        self.raw_imu_ns = 0
        self.register_imu_ns = 0
        self.raw_mag_ns = 0
        self.mag_pair_ns = 0
    @Slot(object)
    def on_robot_state(self, state) -> None:
        self.latest_robot_state = state
        if self.recorder.active:
            self.recorder.robot_state(self.current_step.step_id, state)
        if self.state == RunState.CAPTURING:
            self._update_motion_coverage(state)
        if self.running and state.mode == self.ROBOT_ERROR_MODE:
            self.abort("机械臂进入报警状态（mode=9）")
        elif self.running and state.mode == self.ROBOT_COLLISION_MODE:
            self.abort("机械臂碰撞检测已触发（mode=11）")

    @Slot(str)
    def on_robot_error(self, message: str) -> None:
        if self.running:
            self.abort(message)

    @Slot(str)
    def on_glove_error(self, message: str) -> None:
        if self.running:
            self.abort(message)
