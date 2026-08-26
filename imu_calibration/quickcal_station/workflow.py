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


@dataclass(frozen=True)
class YawLimits:
    negative_soft_limit_deg: float = -50.0
    positive_soft_limit_deg: float = 50.0
    safety_margin_deg: float = 10.0
    rate_deg_s: float = 15.0
    minimum_capture_s: float = 2.0

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
    QuickCalStep("G01", "陀螺", "+X，15 deg/s", "从 X 负向 75° 预置，匀速转至 X 正向 75°；仅匀速平台记录，随后回 X 中位", "实际速度稳定；无夹具滑动", 2, 1, 10, 2, 0x20, MCAL_CAPTURE_GYRO_M, "11 路 Gyro/Acc 原始值、实际角速度", "加速/减速/回中位未进入采集窗口"),
    QuickCalStep("G02", "陀螺", "-X，15 deg/s", "从 X 正向 75° 预置，匀速转至 X 负向 75°；仅匀速平台记录，随后回 X 中位", "实际速度稳定；无夹具滑动", 2, 1, 10, 2, 0x21, MCAL_CAPTURE_GYRO_M, "11 路 Gyro/Acc 原始值、实际角速度", "加速/减速/回中位未进入采集窗口"),
    QuickCalStep("G03", "陀螺", "+Y，15 deg/s", "从 Y 负向 75° 预置，匀速转至 Y 正向 75°；仅匀速平台记录，随后回 Y 中位", "实际速度稳定；无夹具滑动", 2, 1, 10, 2, 0x22, MCAL_CAPTURE_GYRO_M, "11 路 Gyro/Acc 原始值、实际角速度", "加速/减速/回中位未进入采集窗口"),
    QuickCalStep("G04", "陀螺", "-Y，15 deg/s", "从 Y 正向 75° 预置，匀速转至 Y 负向 75°；仅匀速平台记录，随后回 Y 中位", "实际速度稳定；无夹具滑动", 2, 1, 10, 2, 0x23, MCAL_CAPTURE_GYRO_M, "11 路 Gyro/Acc 原始值、实际角速度", "加速/减速/回中位未进入采集窗口"),
    QuickCalStep("G05", "陀螺", "+Z（Yaw），15 deg/s", "从 Yaw 负向安全位转至正向安全位；仅使用限位参数计算的匀速窗口，随后回 0° 中位", "Yaw 限位通过；Yaw 已在 0° 中位；实际速度稳定", 2, 1, 0, 2, 0x24, MCAL_CAPTURE_GYRO_M, "11 路 Gyro/Acc 原始值、实际角速度、Yaw 位置", "不越安全位；加减速和回中位不采样"),
    QuickCalStep("G06", "陀螺", "-Z（Yaw），15 deg/s", "从 Yaw 正向安全位转至负向安全位；仅使用限位参数计算的匀速窗口，随后回 0° 中位", "Yaw 限位通过；Yaw 已在 0° 中位；实际速度稳定", 2, 1, 0, 2, 0x25, MCAL_CAPTURE_GYRO_M, "11 路 Gyro/Acc 原始值、实际角速度、Yaw 位置", "不越安全位；加减速和回中位不采样"),
    QuickCalStep("M01", "磁翻转", "三维翻转第 1 段（15 秒）", "缓慢连续改变夹具三维姿态；不得只绕单一轴或在单一平面往复", "磁环境稳定；机械臂已进入安全的连续慢速三维翻转", 0, 0, 15, 0, 0x30, MCAL_CAPTURE_MAG, "机械臂实际姿态与角速度；固件内部掌心磁场覆盖", "阶段关闭 ACK 通过；至少覆盖两个 Tool 旋转轴"),
    QuickCalStep("M02", "磁翻转", "三维翻转第 2 段（15 秒）", "采用与上一段互补的缓慢三维翻转路径；不得只在单一平面摆动", "M01 完成；机械臂已进入互补方向的连续慢速三维翻转", 0, 0, 15, 0, 0x31, MCAL_CAPTURE_MAG, "机械臂实际姿态与角速度；固件内部掌心磁场覆盖", "阶段关闭 ACK 通过；至少覆盖两个 Tool 旋转轴"),
    QuickCalStep("M03", "磁翻转", "三维翻转第 3 段（15 秒）", "继续采用互补的缓慢三维翻转路径，扩大空间方向覆盖", "M02 完成；机械臂已进入连续慢速三维翻转", 0, 0, 15, 0, 0x32, MCAL_CAPTURE_MAG, "机械臂实际姿态与角速度；固件内部掌心磁场覆盖", "阶段关闭 ACK 通过；至少覆盖两个 Tool 旋转轴"),
    QuickCalStep("M04", "磁翻转", "三维翻转第 4 段（15 秒）", "完成最后一段互补三维翻转；保持线束和夹具安全", "M03 完成；机械臂已进入连续慢速三维翻转", 0, 0, 15, 0, 0x33, MCAL_CAPTURE_MAG, "机械臂实际姿态与角速度；固件内部掌心磁场覆盖", "阶段关闭 ACK 通过；至少覆盖两个 Tool 旋转轴"),
    QuickCalStep("S01", "提交", "求解并写入", "机械臂安全静止；禁止断电拔线", "P1 至 M04 全部完成", 0, 0, 0, 60, None, 0, "type=7 报告、写入 ACK", "11 路 Gyro/Acc 和 Flash 回读通过", False),
    QuickCalStep("S02", "收尾", "回安全位与归档", "机械臂回安全位，可安全拆卸", "上位机已有最终结果", 15, 0, 0, 0, None, 0, "CSV、session.json、result.json", "记录完整且可追溯", False),
)


def steps_for_limits(limits: YawLimits) -> tuple[QuickCalStep, ...]:
    """Return the approved work order with G05/G06 capture windows recalculated."""
    return tuple(
        replace(step, capture_s=limits.capture_s) if step.step_id in ("G05", "G06") else step
        for step in QUICKCAL_STEPS
    )


def expected_total_seconds(limits: YawLimits) -> float:
    return sum(step.total_s for step in steps_for_limits(limits))


def expected_capture_seconds(limits: YawLimits) -> float:
    return sum(step.capture_s for step in steps_for_limits(limits))
