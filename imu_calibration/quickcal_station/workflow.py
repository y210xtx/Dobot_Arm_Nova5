"""QuickCal V1 workflow transcribed from the approved robot work order."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math

from .protocol import (
    MCAL_CAPTURE_ACCEL,
    MCAL_CAPTURE_GYRO_BIAS,
    MCAL_CAPTURE_GYRO_M,
    MCAL_CAPTURE_MAG,
)


IMU_NAMES = (
    "WRIST",
    "THUMB_1",
    "THUMB_2",
    "INDEX_1",
    "INDEX_2",
    "MIDDLE_1",
    "MIDDLE_2",
    "RING_1",
    "RING_2",
    "PINKY_1",
    "PINKY_2",
)

ROLL_PITCH_GYRO_RATE_DEG_S = 15.0
LIMITED_GYRO_OUTER_DEG = 55.0
LIMITED_GYRO_CAPTURE_BOUND_DEG = 45.0
LIMITED_GYRO_ACCEL_DECEL_DEG = (
    LIMITED_GYRO_OUTER_DEG - LIMITED_GYRO_CAPTURE_BOUND_DEG
)
LIMITED_GYRO_CAPTURE_S = 6.0


@dataclass(frozen=True)
class YawLimits:
    negative_soft_limit_deg: float = -LIMITED_GYRO_OUTER_DEG
    positive_soft_limit_deg: float = LIMITED_GYRO_OUTER_DEG
    safety_margin_deg: float = LIMITED_GYRO_ACCEL_DECEL_DEG
    rate_deg_s: float = ROLL_PITCH_GYRO_RATE_DEG_S
    minimum_capture_s: float = LIMITED_GYRO_CAPTURE_S

    @property
    def negative_safe_deg(self) -> float:
        return self.negative_soft_limit_deg + self.safety_margin_deg

    @property
    def positive_safe_deg(self) -> float:
        return self.positive_soft_limit_deg - self.safety_margin_deg

    @property
    def scan_angle_deg(self) -> float:
        return max(0.0, self.positive_safe_deg - self.negative_safe_deg)

    @property
    def capture_s(self) -> float:
        return self.scan_angle_deg / self.rate_deg_s if self.rate_deg_s > 0 else 0.0

    @property
    def valid(self) -> bool:
        return (
            all(
                math.isfinite(value)
                for value in (
                    self.negative_soft_limit_deg,
                    self.positive_soft_limit_deg,
                    self.safety_margin_deg,
                    self.rate_deg_s,
                    self.minimum_capture_s,
                )
            )
            and self.negative_soft_limit_deg < 0
            and self.positive_soft_limit_deg > 0
            and self.safety_margin_deg >= 0
            and self.negative_safe_deg < 0 < self.positive_safe_deg
            and self.rate_deg_s > 0
            and self.minimum_capture_s > 0
            and self.capture_s >= self.minimum_capture_s
            and math.isclose(
                self.negative_soft_limit_deg,
                -LIMITED_GYRO_OUTER_DEG,
                abs_tol=1e-9,
            )
            and math.isclose(
                self.positive_soft_limit_deg,
                LIMITED_GYRO_OUTER_DEG,
                abs_tol=1e-9,
            )
            and math.isclose(
                self.safety_margin_deg,
                LIMITED_GYRO_ACCEL_DECEL_DEG,
                abs_tol=1e-9,
            )
            and math.isclose(
                self.rate_deg_s,
                ROLL_PITCH_GYRO_RATE_DEG_S,
                abs_tol=1e-9,
            )
            and math.isclose(
                self.capture_s,
                LIMITED_GYRO_CAPTURE_S,
                abs_tol=1e-9,
            )
        )


@dataclass(frozen=True)
class QuickCalStep:
    step_id: str
    group: str
    name: str
    robot_action: str
    start_condition: str
    move_s: float
    settle_s: float
    capture_s: float
    exit_s: float
    stage_code: int | None
    capture_mask: int
    record_data: str
    pass_condition: str
    sample_enabled: bool = True

    @property
    def total_s(self) -> float:
        return self.move_s + self.settle_s + self.capture_s + self.exit_s


QUICKCAL_STEPS = (
    QuickCalStep("P0", "准备", "设备与夹具检查", "机械臂停在安全位；夹具锁紧；确认线束安全", "Dobot 机械臂与 IMU 手套均已连接", 30, 0, 0, 0, None, 0, "在线掩码、固件、SN、工位", "11 路在线且工厂协议可用", False),
    QuickCalStep("P1", "静止基线", "30 秒静止", "机械臂保持中性姿态，夹具完全静止", "静置至少 2 秒且无振动", 0, 2, 30, 0, 0x01, MCAL_CAPTURE_GYRO_BIAS, "11 路 Gyro/Acc；磁 SET/RESET", "原始流连续且阶段 11/11 通过"),
    QuickCalStep("A01", "六面", "+X 面", "重力投影到夹具 +X；到位保持", "机器人到位", 3, 2, 3, 0, 0x10, MCAL_CAPTURE_ACCEL, "11 路 Acc/Gyro", "采样期间机械臂零角速度"),
    QuickCalStep("A02", "六面", "-X 面", "重力投影到夹具 -X；到位保持", "机器人到位", 3, 2, 3, 0, 0x11, MCAL_CAPTURE_ACCEL, "11 路 Acc/Gyro", "采样期间机械臂零角速度"),
    QuickCalStep("A03", "六面", "+Y 面", "重力投影到夹具 +Y；到位保持", "机器人到位", 3, 2, 3, 0, 0x12, MCAL_CAPTURE_ACCEL, "11 路 Acc/Gyro", "采样期间机械臂零角速度"),
    QuickCalStep("A04", "六面", "-Y 面", "重力投影到夹具 -Y；到位保持", "机器人到位", 3, 2, 3, 0, 0x13, MCAL_CAPTURE_ACCEL, "11 路 Acc/Gyro", "采样期间机械臂零角速度"),
    QuickCalStep("A05", "六面", "+Z 面", "重力投影到夹具 +Z；到位保持", "机器人到位", 3, 2, 3, 0, 0x14, MCAL_CAPTURE_ACCEL, "11 路 Acc/Gyro", "采样期间机械臂零角速度"),
    QuickCalStep("A06", "六面", "-Z 面", "重力投影到夹具 -Z；到位保持", "机器人到位", 3, 2, 3, 0, 0x15, MCAL_CAPTURE_ACCEL, "11 路 Acc/Gyro", "采样期间机械臂零角速度"),
    QuickCalStep("G01", "陀螺", "+X，15 deg/s", "沿实测夹具 X 对应的 Tool Rx 正向扫转；外端 ±55°，仅 ±45° 匀速区采集 6 秒", "已在中位；实际速度稳定", 2, 1, LIMITED_GYRO_CAPTURE_S, 2, 0x20, MCAL_CAPTURE_GYRO_M, "11 路 Gyro/Acc 原始值、实际角速度、配置轴相对角度", "采集仅覆盖 ±45°；加减速和回中位不采样"),
    QuickCalStep("G02", "陀螺", "-X，15 deg/s", "沿 G01 同一 Tool Rx 反向扫转；外端 ±55°，仅 ±45° 匀速区采集 6 秒", "已在中位；实际速度稳定", 2, 1, LIMITED_GYRO_CAPTURE_S, 2, 0x21, MCAL_CAPTURE_GYRO_M, "11 路 Gyro/Acc 原始值、实际角速度、配置轴相对角度", "采集仅覆盖 ±45°；加减速和回中位不采样"),
    QuickCalStep("G03", "陀螺", "+Y，15 deg/s", "按动作页配置的 Tool 轴/方向扫转；仅中间 ±75° 匀速平台记录", "实际速度稳定；无夹具滑动", 2, 1, 10, 2, 0x22, MCAL_CAPTURE_GYRO_M, "11 路 Gyro/Acc 原始值、实际角速度", "加速/减速/回中位未进入采集窗口"),
    QuickCalStep("G04", "陀螺", "-Y，15 deg/s", "沿 G03 同一配置 Tool 轴反向扫转；仅中间 ±75° 匀速平台记录", "实际速度稳定；无夹具滑动", 2, 1, 10, 2, 0x23, MCAL_CAPTURE_GYRO_M, "11 路 Gyro/Acc 原始值、实际角速度", "加速/减速/回中位未进入采集窗口"),
    QuickCalStep("G05", "陀螺", "+Z（Yaw），15 deg/s", "沿实测夹具 Z 对应的 Tool Rz 正向扫转；仅中间 ±75° 匀速平台记录", "实际速度稳定；无夹具滑动", 2, 1, 10, 2, 0x24, MCAL_CAPTURE_GYRO_M, "11 路 Gyro/Acc 原始值、实际角速度、配置轴相对角度", "加速/减速/回中位未进入采集窗口"),
    QuickCalStep("G06", "陀螺", "-Z（Yaw），15 deg/s", "沿 G05 同一 Tool Rz 反向扫转；仅中间 ±75° 匀速平台记录", "实际速度稳定；无夹具滑动", 2, 1, 10, 2, 0x25, MCAL_CAPTURE_GYRO_M, "11 路 Gyro/Acc 原始值、实际角速度、配置轴相对角度", "加速/减速/回中位未进入采集窗口"),
    QuickCalStep("M01", "磁标定", "固定 XYZ 的 Ry/J6 三维覆盖", "按 G03/G04 方式固定 TCP XYZ，执行 Tool Ry ±75° 俯仰并叠加 J6 方向 Tool Rz ±75° 往复", "从标定中位开始；磁环境和线束已确认", 0, 0, 40, 0, 0x30, MCAL_CAPTURE_MAG, "掌部磁传感器原始场", "40 秒内 XYZ 保持，Ry 与 J6 方向正反覆盖并回中位"),
    QuickCalStep("M02", "磁标定", "Yaw 正向单侧往返", "Tool Rx 从 0° 到 +45° 后沿原路径回到 0°", "从标定中位开始；路径无碰撞", 0, 0, 10, 0, 0x31, MCAL_CAPTURE_MAG, "掌部磁传感器原始场", "10 秒内完成正向单侧往返并回中位"),
    QuickCalStep("M03", "磁标定", "Yaw 负向单侧往返", "Tool Rx 从 0° 到 -45° 后沿原路径回到 0°", "从标定中位开始；路径无碰撞", 0, 0, 10, 0, 0x32, MCAL_CAPTURE_MAG, "掌部磁传感器原始场", "10 秒内完成负向单侧往返并回中位"),
    QuickCalStep("M04", "磁标定", "中位静止收尾", "回到 Tool Rz=0° 标定中位并静止保持 5 秒", "机械臂静止；线束无受力", 0, 0, 0, 5, 0x33, MCAL_CAPTURE_MAG, "仅记录阶段标记，不采集磁数据", "中位静止 5 秒且未向磁解算器送样"),
    QuickCalStep("S01", "提交", "求解并写入", "机械臂安全静止；禁止断电拔线", "P1、A01-A06、G01-G06、M01-M04 共 17 个正式阶段全部完成", 0, 0, 0, 60, None, 0, "type=7 v4 Gyro/Accel/Mag 报告、写入 ACK", "Gyro/Accel/Mag 和 Flash 写入全部通过", False),
    QuickCalStep("S02", "收尾", "回标定中位与归档", "机械臂回示教标定中位并保持静止", "上位机已有最终结果", 15, 0, 0, 0, None, 0, "CSV、session.json、result.json", "记录完整且机械臂已到标定中位", False),
)


def steps_for_limits(limits: YawLimits) -> tuple[QuickCalStep, ...]:
    """Return the work order with the fixed six-second G01/G02 window."""
    return tuple(
        replace(step, capture_s=LIMITED_GYRO_CAPTURE_S)
        if step.step_id in ("G01", "G02")
        else step
        for step in QUICKCAL_STEPS
    )


def expected_total_seconds(limits: YawLimits) -> float:
    return sum(step.total_s for step in steps_for_limits(limits))


def expected_capture_seconds(limits: YawLimits) -> float:
    return sum(step.capture_s for step in steps_for_limits(limits))
