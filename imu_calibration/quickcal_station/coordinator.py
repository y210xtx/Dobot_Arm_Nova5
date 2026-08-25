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
)
from .session_recorder import SessionRecorder
from .workflow import QuickCalStep, YawLimits, steps_for_limits


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
    ERROR = "失败"


class QuickCalCoordinator(QObject):
    state_changed = Signal(str)
    current_step_changed = Signal(int, object)
    step_status_changed = Signal(int, str, str)
    progress_changed = Signal(int, str)
    status_message = Signal(str, str)
    finished = Signal(bool, str)

    RAW_FRESH_NS = 800_000_000
    ROBOT_FRESH_NS = 1_000_000_000
    ACK_TIMEOUT_NS = 5_000_000_000
    COMMIT_TIMEOUT_NS = 65_000_000_000

    def __init__(self, glove, robot, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.glove = glove
        self.robot = robot
        self.recorder = SessionRecorder()
        self.limits = YawLimits()
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
        self.pending_report = None
        self._capture_fault = ""
        self._aborting = False
        self.condition_stable_since_ns = 0

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
        if self.latest_raw_imu is None or now - self.raw_imu_ns > self.RAW_FRESH_NS:
            errors.append("type=9 原始 IMU 数据不新鲜")
        elif self.latest_raw_imu.presence_mask & ALL_IMU_MASK != ALL_IMU_MASK:
            errors.append(f"type=9 在线掩码不是 0x{ALL_IMU_MASK:04X}")
        if self.latest_register_imu is None or now - self.register_imu_ns > self.RAW_FRESH_NS:
            errors.append("type=11 寄存器 IMU 数据不新鲜")
        elif self.latest_register_imu.presence_mask & ALL_IMU_MASK != ALL_IMU_MASK:
            errors.append(f"type=11 在线掩码不是 0x{ALL_IMU_MASK:04X}")
        if not self.robot.connected:
            errors.append("机械臂未连接")
        if self.latest_robot_state is None or now - self.latest_robot_state.received_monotonic_ns > self.ROBOT_FRESH_NS:
            errors.append("机械臂反馈不新鲜")
        elif self.latest_robot_state.mode != 5:
            errors.append(f"机械臂未处于已使能空闲状态（当前 mode={self.latest_robot_state.mode}）")
        if not self.limits.valid:
            errors.append("Yaw 软限位、安全余量或采集时间配置不通过")
        return errors

    @Slot()
    def start_session(self) -> bool:
        if self.running:
            self.status_message.emit("工厂会话已在进行", "error")
            return False
        errors = self.preflight_errors()
        if errors:
            self.status_message.emit("；".join(errors), "error")
            return False
        metadata = {
            "operator": self.operator,
            "firmware_tag": self.version.revision_tag,
            "imu_model": self.version.imu_model,
            "hand_side": self.version.hand_side,
            "workflow": "QuickCal V1 Robot Control Steps",
            "yaw_limits": {
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
        self.step_status = ["未开始"] * len(self.steps)
        self.step_detail = [""] * len(self.steps)
        self.current_index = 0
        self._set_step(0, "进行中", "设备自检通过")
        self._set_step(0, "完成", "11 路在线，机械臂已使能")
        self.current_index = 1
        self.condition_stable_since_ns = 0
        self.commit_ack_ok = False
        self.pending_report = None
        self.recorder.marker("session_begin", "P0", str(directory))
        self._set_state(RunState.WAIT_BEGIN_ACK)
        self.operation_deadline_ns = time.monotonic_ns() + self.ACK_TIMEOUT_NS
        if not self.glove.send_command(CMD_MCAL_BEGIN):
            self.abort("MCAL_BEGIN 发送失败")
            return False
        self.status_message.emit("等待固件确认 MCAL_BEGIN", "info")
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
        if step.step_id == "M04":
            self._complete_non_capture_step("Yaw 中位与静止状态已确认")
            return True
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
        self._set_step(self.current_index, "进行中", "等待固件开启采集阶段")
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
        if any(status != "完成" for status in self.step_status[1:18]):
            self.status_message.emit("P1 至 M04 尚未全部完成", "error")
            return False
        self._set_step(self.current_index, "进行中", "固件正在求解并写入 Flash")
        self.recorder.marker("commit_request", "S01", "all workflow gates passed")
        self.commit_ack_ok = False
        self.pending_report = None
        self._set_state(RunState.WAIT_COMMIT_ACK)
        self.operation_deadline_ns = time.monotonic_ns() + self.COMMIT_TIMEOUT_NS
        return self.glove.send_command(CMD_MCAL_COMMIT)

    @Slot(str)
    def abort(self, reason: str = "操作员中止") -> None:
        if self._aborting or self.state in (RunState.IDLE, RunState.COMPLETE, RunState.ABORTED):
            return
        self._aborting = True
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

    @Slot()
    def retry_current_step(self) -> None:
        if self.state != RunState.ERROR:
            return
        self._set_step(self.current_index, "未开始", "等待重试")
        self._set_state(RunState.READY)
        self.progress_changed.emit(0, "等待动作条件")

    def _complete_non_capture_step(self, detail: str) -> None:
        self._set_step(self.current_index, "完成", detail)
        self.recorder.marker("step_complete", self.current_step.step_id, detail)
        self._advance()

    def _advance(self) -> None:
        self.current_index += 1
        if self.current_index >= len(self.steps):
            return
        self.condition_stable_since_ns = 0
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

    def _check_motion_condition(self, step: QuickCalStep) -> str:
        state = self.latest_robot_state
        if state is None or time.monotonic_ns() - state.received_monotonic_ns > self.ROBOT_FRESH_NS:
            self.condition_stable_since_ns = 0
            return "机械臂反馈已超时"
        if state.mode not in (5, 7, 11):
            self.condition_stable_since_ns = 0
            return f"机械臂状态不允许采集（mode={state.mode}）"
        if step.step_id in ("P1", "A01", "A02", "A03", "A04", "A05", "A06", "M04", "S01", "S02"):
            if state.mode != 5 or state.angular_speed_norm > 0.8 or state.linear_speed_norm > 1.0:
                self.condition_stable_since_ns = 0
                return (
                    f"该步骤要求机械臂静止，当前线速度 {state.linear_speed_norm:.2f} mm/s，"
                    f"角速度 {state.angular_speed_norm:.2f}°/s"
                )
        if step.step_id in ("G05", "G06") and not self.limits.valid:
            return "Yaw 限位计算未通过"
        if step.step_id.startswith("G"):
            axis_index = {"G01": 0, "G02": 0, "G03": 1, "G04": 1, "G05": 2, "G06": 2}[step.step_id]
            direction = 1 if step.step_id in ("G01", "G03", "G05") else -1
            tool_speed = self._tool_angular_speed(state.pose, state.angular_speed)
            measured = tool_speed[axis_index]
            target = direction * self.limits.rate_deg_s
            if abs(measured - target) > max(3.0, abs(target) * 0.20):
                self.condition_stable_since_ns = 0
                return f"匀速条件未满足：目标 {target:+.1f}°/s，实际 {measured:+.1f}°/s"
        required_stable_s = 0.0
        if step.step_id in ("P1", "A01", "A02", "A03", "A04", "A05", "A06"):
            required_stable_s = step.settle_s
        elif step.step_id.startswith("G"):
            required_stable_s = 0.5
        elif step.step_id in ("M04", "S01", "S02"):
            required_stable_s = 0.5
        now = time.monotonic_ns()
        if required_stable_s > 0:
            if self.condition_stable_since_ns == 0:
                self.condition_stable_since_ns = now
            stable_s = (now - self.condition_stable_since_ns) / 1_000_000_000.0
            if stable_s < required_stable_s:
                return f"动作条件已满足，请继续保持 {required_stable_s - stable_s:.1f} s"
        return ""

    @staticmethod
    def _tool_angular_speed(pose, angular_speed) -> tuple[float, float, float]:
        rx, ry, rz = (math.radians(float(value)) for value in pose[3:6])
        axes = (
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
        return tuple(sum(float(value) * direction for value, direction in zip(angular_speed, axis)) for axis in axes)

    def _fail_current_step(self, reason: str) -> None:
        self._set_step(self.current_index, "失败", reason)
        self.recorder.marker("step_failed", self.current_step.step_id, reason)
        self._set_state(RunState.ERROR)
        self.status_message.emit(reason, "error")

    @Slot()
    def _tick(self) -> None:
        now = time.monotonic_ns()
        if self.state in (
            RunState.WAIT_BEGIN_ACK,
            RunState.WAIT_STAGE_OPEN,
            RunState.WAIT_STAGE_CLOSE,
            RunState.WAIT_COMMIT_ACK,
            RunState.WAIT_REPORT,
        ) and self.operation_deadline_ns and now > self.operation_deadline_ns:
            self.abort(f"{self.state.value}超时")
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
            self._request_stage_close(failed=True)
        elif now >= self.capture_deadline_ns:
            self._request_stage_close(failed=False)

    def _live_capture_fault(self, now: int) -> str:
        if now - self.raw_imu_ns > self.RAW_FRESH_NS or now - self.register_imu_ns > self.RAW_FRESH_NS:
            return "采集期间 IMU 原始流中断"
        if self.latest_raw_imu.presence_mask & ALL_IMU_MASK != ALL_IMU_MASK:
            return "采集期间 type=9 出现 IMU 离线"
        if self.latest_register_imu.presence_mask & ALL_IMU_MASK != ALL_IMU_MASK:
            return "采集期间 type=11 出现 IMU 离线"
        motion_error = self._check_motion_condition(self.current_step)
        if motion_error and not self.current_step.step_id.startswith("M"):
            return motion_error
        if self.current_step.step_id.startswith("M"):
            first_pair_wait_expired = now - self.capture_started_ns > 1_500_000_000
            no_pair_this_stage = self.mag_pair_ns < self.capture_started_ns
            stale_pair = self.mag_pair_ns >= self.capture_started_ns and now - self.mag_pair_ns > 1_500_000_000
            if (no_pair_this_stage and first_pair_wait_expired) or stale_pair:
                return "磁翻转期间 SET/RESET 成对数据中断"
        return ""

    def _request_stage_close(self, failed: bool) -> None:
        step = self.current_step
        if self.state != RunState.CAPTURING or step.stage_code is None:
            return
        self.recorder.marker("stage_close_request", step.step_id, self._capture_fault or "capture complete")
        self._set_state(RunState.WAIT_STAGE_CLOSE)
        self.operation_deadline_ns = time.monotonic_ns() + self.ACK_TIMEOUT_NS
        if failed:
            self._set_step(self.current_index, "失败", self._capture_fault)
        self.glove.send_command(CMD_MCAL_STAGE, step.stage_code, b"\x00")

    @Slot(object)
    def on_ack(self, frame) -> None:
        self.recorder.marker("ack", self.current_step.step_id, f"cmd=0x{frame.cmd:02X} status={frame.status} d0={frame.detail0} d1={frame.detail1}")
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
                self._fail_current_step(
                    f"阶段 0x{step.stage_code:02X} 质量未通过：status={frame.status}，合格 IMU={frame.detail1}/11"
                )
                return
            if step.step_id.startswith("M") and self.valid_mag_pairs_this_step == 0:
                self._fail_current_step("磁翻转阶段没有有效 SET/RESET 成对数据")
                return
            detail = f"固件质量门 11/11；有效磁对 {self.valid_mag_pairs_this_step}"
            self._set_step(self.current_index, "完成", detail)
            self.recorder.marker("capture_complete", step.step_id, detail)
            self._advance()
            return
        if frame.cmd == CMD_MCAL_COMMIT and self.state in (RunState.WAIT_COMMIT_ACK, RunState.WAIT_REPORT):
            self.commit_ack_ok = frame.status == 0 and frame.detail0 == 11
            if not self.commit_ack_ok:
                self.abort(f"参数求解或 Flash 写入失败：status={frame.status}，Gyro={frame.detail0}/11")
                return
            self._set_state(RunState.WAIT_REPORT)
            self.operation_deadline_ns = time.monotonic_ns() + self.COMMIT_TIMEOUT_NS
            if self.pending_report is not None:
                self._finish_report(self.pending_report)

    @Slot(object)
    def on_mcal_report(self, report) -> None:
        self.recorder.save_report(report)
        if self.state not in (RunState.WAIT_COMMIT_ACK, RunState.WAIT_REPORT):
            return
        self.pending_report = report
        if self.commit_ack_ok:
            self._finish_report(report)

    def _finish_report(self, report) -> None:
        passed = self.commit_ack_ok and report.gyro_all_ok and report.accel_all_ok
        if not passed:
            self.abort(
                f"type=7 未通过：status={report.status}，Gyro={report.calibrated_count}/11，Accel={'11/11' if report.accel_all_ok else '未全通过'}"
            )
            return
        detail = f"Gyro 11/11，Accel 11/11，Flash seq={report.flash_sequence}，平均 RMS={report.mean_rms_mdeg / 1000:.3f}°"
        self._set_step(self.current_index, "完成", detail)
        self.recorder.marker("commit_complete", "S01", detail)
        self._advance()

    @Slot(object)
    def on_raw_imu(self, frame) -> None:
        self.latest_raw_imu = frame
        self.raw_imu_ns = time.monotonic_ns()
        if self.state == RunState.CAPTURING:
            self.recorder.raw_imu(self.current_step.step_id, frame)

    @Slot(object)
    def on_register_raw_imu(self, frame) -> None:
        self.latest_register_imu = frame
        self.register_imu_ns = time.monotonic_ns()
        if self.state == RunState.CAPTURING:
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
    def on_robot_state(self, state) -> None:
        self.latest_robot_state = state
        if self.recorder.active:
            self.recorder.robot_state(self.current_step.step_id, state)
        if self.running and state.mode == 9:
            self.abort("机械臂进入报警状态")

    @Slot(str)
    def on_robot_error(self, message: str) -> None:
        if self.running:
            self.abort(message)

    @Slot(str)
    def on_glove_error(self, message: str) -> None:
        if self.running:
            self.abort(message)
