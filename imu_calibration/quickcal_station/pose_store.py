"""Persistent, validated robot poses taught at the QuickCal station."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import json
import math
from pathlib import Path
from typing import Any, Mapping


POSE_CONFIG_VERSION = 1
POSE_NAMES = ("safe", "neutral")


@dataclass(frozen=True)
class TaughtPose:
    pose: tuple[float, ...]
    joints: tuple[float, ...]
    user: int
    tool: int
    recorded_at: str

    def __post_init__(self) -> None:
        if len(self.pose) != 6 or not all(math.isfinite(value) for value in self.pose):
            raise ValueError("TCP 位姿必须包含 6 个有效数值")
        if self.joints and (len(self.joints) != 6 or not all(math.isfinite(value) for value in self.joints)):
            raise ValueError("关节位姿必须为空或包含 6 个有效数值")
        if not 0 <= self.user <= 9 or not 0 <= self.tool <= 9:
            raise ValueError("User/Tool 编号必须在 0～9 范围内")
        if not self.recorded_at:
            raise ValueError("示教时间不能为空")

    @classmethod
    def from_robot_state(cls, state: Any) -> "TaughtPose":
        return cls(
            pose=tuple(float(value) for value in state.pose),
            joints=tuple(float(value) for value in state.joints),
            user=int(state.user),
            tool=int(state.tool),
            recorded_at=datetime.now().astimezone().isoformat(timespec="seconds"),
        )

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "TaughtPose":
        return cls(
            pose=tuple(float(value) for value in data["pose"]),
            joints=tuple(float(value) for value in data.get("joints", ())),
            user=int(data["user"]),
            tool=int(data["tool"]),
            recorded_at=str(data["recorded_at"]),
        )

    def to_mapping(self) -> dict[str, Any]:
        return {
            "recorded_at": self.recorded_at,
            "pose": list(self.pose),
            "joints": list(self.joints),
            "user": self.user,
            "tool": self.tool,
        }


def load_pose_config(path: Path) -> dict[str, TaughtPose]:
    if not path.exists():
        return {}
    data = json.loads(path.read_text(encoding="utf-8"))
    if int(data.get("version", 0)) != POSE_CONFIG_VERSION:
        raise ValueError("不支持的示教位配置版本")
    result: dict[str, TaughtPose] = {}
    raw_poses = data.get("poses", {})
    for name in POSE_NAMES:
        if name in raw_poses:
            result[name] = TaughtPose.from_mapping(raw_poses[name])
    return result


def save_pose_config(path: Path, poses: Mapping[str, TaughtPose]) -> None:
    unknown = set(poses) - set(POSE_NAMES)
    if unknown:
        raise ValueError(f"未知示教位：{', '.join(sorted(unknown))}")
    data = {
        "version": POSE_CONFIG_VERSION,
        "poses": {name: poses[name].to_mapping() for name in POSE_NAMES if name in poses},
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    temporary.replace(path)


def load_legacy_safe_pose(path: Path) -> TaughtPose | None:
    if not path.exists():
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        return TaughtPose.from_mapping(data)
    except (KeyError, TypeError, ValueError, OSError, json.JSONDecodeError):
        return None
