"""Persistent, validated robot action settings for incremental automation."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Mapping


ACTION_CONFIG_VERSION = 1
VALID_TOOL_ROTATION_AXES = ("Rx", "Ry", "Rz")


@dataclass(frozen=True)
class RobotActionConfig:
    enabled: bool = True
    axis: str = "Rz"
    degrees: float = 180.0
    velocity_percent: int = 5
    timeout_s: float = 60.0

    def __post_init__(self) -> None:
        if self.axis not in VALID_TOOL_ROTATION_AXES:
            raise ValueError(f"不支持的 Tool 旋转轴：{self.axis}")
        if not math.isfinite(self.degrees) or not 0.1 <= abs(self.degrees) <= 180.0:
            raise ValueError("相对旋转角度绝对值必须在 0.1°～180°之间")
        if not 1 <= int(self.velocity_percent) <= 100:
            raise ValueError("自动标定动作速度比例必须在 1%～100% 之间")
        if not math.isfinite(self.timeout_s) or not 5.0 <= self.timeout_s <= 180.0:
            raise ValueError("动作超时必须在 5～180 秒之间")

    @classmethod
    def from_mapping(cls, data: Mapping[str, object]) -> "RobotActionConfig":
        return cls(
            enabled=bool(data.get("enabled", True)),
            axis=str(data.get("axis", "Rz")),
            degrees=float(data.get("degrees", 180.0)),
            velocity_percent=int(data.get("velocity_percent", 5)),
            timeout_s=float(data.get("timeout_s", 60.0)),
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "enabled": self.enabled,
            "axis": self.axis,
            "degrees": self.degrees,
            "velocity_percent": self.velocity_percent,
            "timeout_s": self.timeout_s,
        }


DEFAULT_ACTIONS = {
    "A01": RobotActionConfig(axis="Rz", degrees=180.0),
    "A02": RobotActionConfig(axis="Rz", degrees=-180.0),
    "A03": RobotActionConfig(axis="Rz", degrees=90.0),
    "A04": RobotActionConfig(axis="Rz", degrees=-180.0),
    "A05": RobotActionConfig(axis="Rx", degrees=90.0),
    "A06": RobotActionConfig(axis="Rx", degrees=-180.0),
    # r024 stage axes mapped from the 2026-08-27 raw-stream diagnosis.
    # For G stages ``degrees`` is the signed effective capture sweep.  The
    # short ±45°/6 s window follows the mechanically limited Tool Rx axis.
    # Six-face-derived transforms of the raw streams show Rx/Ry/Rz mapping
    # to fixture X/Y/Z respectively; sensor-local gx/gy/gz must not be used
    # directly because the eleven IMUs have different mounting rotations.
    "G01": RobotActionConfig(axis="Rx", degrees=90.0),
    "G02": RobotActionConfig(axis="Rx", degrees=-90.0),
    "G03": RobotActionConfig(axis="Ry", degrees=150.0),
    "G04": RobotActionConfig(axis="Ry", degrees=-150.0),
    "G05": RobotActionConfig(axis="Rz", degrees=150.0),
    "G06": RobotActionConfig(axis="Rz", degrees=-150.0),
}


def load_action_config(path: Path) -> dict[str, RobotActionConfig]:
    result = dict(DEFAULT_ACTIONS)
    if not path.exists():
        return result
    data = json.loads(path.read_text(encoding="utf-8"))
    if int(data.get("version", 0)) != ACTION_CONFIG_VERSION:
        raise ValueError("不支持的机械臂动作配置版本")
    for step_id, raw in data.get("actions", {}).items():
        if step_id not in DEFAULT_ACTIONS:
            continue
        result[step_id] = RobotActionConfig.from_mapping(raw)
    return result


def save_action_config(path: Path, actions: Mapping[str, RobotActionConfig]) -> None:
    unknown = set(actions) - set(DEFAULT_ACTIONS)
    if unknown:
        raise ValueError(f"尚未支持的自动动作：{', '.join(sorted(unknown))}")
    data = {
        "version": ACTION_CONFIG_VERSION,
        "actions": {
            step_id: actions[step_id].to_mapping()
            for step_id in DEFAULT_ACTIONS
            if step_id in actions
        },
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    temporary.replace(path)


def gravity_after_tool_rotation(
    gravity_tool: tuple[float, float, float], axis: str, degrees: float
) -> tuple[float, float, float]:
    """Predict R_new.T*g for a relative rotation R_new=R_old*Rot(axis)."""
    if axis not in VALID_TOOL_ROTATION_AXES:
        raise ValueError(f"不支持的 Tool 旋转轴：{axis}")
    x, y, z = (float(value) for value in gravity_tool)
    radians = math.radians(float(degrees))
    cosine = math.cos(radians)
    sine = math.sin(radians)
    if axis == "Rx":
        return x, cosine * y + sine * z, -sine * y + cosine * z
    if axis == "Ry":
        return cosine * x - sine * z, y, sine * x + cosine * z
    return cosine * x + sine * y, -sine * x + cosine * y, z


def vector_angle_deg(
    left: tuple[float, float, float], right: tuple[float, float, float]
) -> float:
    left_norm = math.sqrt(sum(value * value for value in left))
    right_norm = math.sqrt(sum(value * value for value in right))
    if left_norm <= 0.0 or right_norm <= 0.0:
        raise ValueError("方向向量长度必须大于 0")
    dot = sum(a * b for a, b in zip(left, right)) / (left_norm * right_norm)
    return math.degrees(math.acos(max(-1.0, min(1.0, dot))))
