"""Constrained M01 joint solver for the Nova 5 calibration fixture."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Callable

import numpy as np

from .coordinator import QuickCalCoordinator


Pose = tuple[float, ...]
Joints = tuple[float, ...]
PositiveKinematics = Callable[[Joints, int, int], Pose | None]
ProgressCallback = Callable[[int, int], None]
M01_CACHE_VERSION = 1


@dataclass(frozen=True)
class M01JointTarget:
    joints: Joints
    pitch_deg: float
    roll_deg: float
    leg_index: int
    subsegment_index: int
    position_error_mm: float
    orientation_error_deg: float


@dataclass(frozen=True)
class M01JointTrajectory:
    targets: tuple[M01JointTarget, ...]
    joint_plane_axis_tool: tuple[float, float, float]
    max_position_error_mm: float
    max_orientation_error_deg: float


def load_m01_joint_trajectory_cache(
    path: Path,
    reference_pose: Pose,
    reference_joints: Joints,
    user: int,
    tool: int,
    robot_id: str,
    waypoints,
    subsegments_per_leg: int,
) -> M01JointTrajectory | None:
    """Load a trajectory only when its robot, frame and taught neutral match."""
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
        if payload.get("version") != M01_CACHE_VERSION:
            return None
        if payload.get("robot_id") != robot_id:
            return None
        if (int(payload.get("user")), int(payload.get("tool"))) != (user, tool):
            return None
        cached_pose = tuple(float(value) for value in payload["reference_pose"])
        cached_joints = tuple(float(value) for value in payload["reference_joints"])
        cached_waypoints = tuple(
            tuple(float(value) for value in point) for point in payload["waypoints"]
        )
        expected_waypoints = tuple(
            tuple(float(value) for value in point) for point in waypoints
        )
        if cached_waypoints != expected_waypoints:
            return None
        if int(payload.get("subsegments_per_leg")) != subsegments_per_leg:
            return None
        if len(cached_pose) != 6 or len(cached_joints) != 6:
            return None
        if max(abs(a - b) for a, b in zip(cached_joints, reference_joints)) > 0.05:
            return None
        if np.linalg.norm(
            np.asarray(cached_pose[:3]) - np.asarray(reference_pose[:3])
        ) > 0.5:
            return None
        pose_angle = np.linalg.norm(
            _rotation_vector_deg(
                _matrix_from_pose(cached_pose), _matrix_from_pose(reference_pose)
            )
        )
        if pose_angle > 0.2:
            return None
        expected_samples = tuple(_samples(waypoints, subsegments_per_leg))
        raw_targets = payload["targets"]
        if len(raw_targets) != len(expected_samples):
            return None
        targets = []
        for raw, expected in zip(raw_targets, expected_samples):
            joints = tuple(float(value) for value in raw["joints"])
            pitch_deg, roll_deg, leg_index, subsegment_index = expected
            if len(joints) != 6 or not np.isfinite(joints).all():
                return None
            if abs(joints[0] - reference_joints[0]) > 0.05:
                return None
            if abs(joints[4] - reference_joints[4]) > 0.05:
                return None
            if abs(joints[5] - (reference_joints[5] + roll_deg)) > 0.05:
                return None
            targets.append(
                M01JointTarget(
                    joints=joints,
                    pitch_deg=pitch_deg,
                    roll_deg=roll_deg,
                    leg_index=leg_index,
                    subsegment_index=subsegment_index,
                    position_error_mm=float(raw["position_error_mm"]),
                    orientation_error_deg=float(raw["orientation_error_deg"]),
                )
            )
        axis = tuple(float(value) for value in payload["joint_plane_axis_tool"])
        if len(axis) != 3 or not np.isfinite(axis).all():
            return None
        if max(
            abs(value - expected)
            for value, expected in zip(targets[-1].joints, reference_joints)
        ) > 0.05:
            return None
        return M01JointTrajectory(
            targets=tuple(targets),
            joint_plane_axis_tool=axis,
            max_position_error_mm=float(payload["max_position_error_mm"]),
            max_orientation_error_deg=float(payload["max_orientation_error_deg"]),
        )
    except (KeyError, TypeError, ValueError, OSError, json.JSONDecodeError):
        return None


def save_m01_joint_trajectory_cache(
    path: Path,
    trajectory: M01JointTrajectory,
    reference_pose: Pose,
    reference_joints: Joints,
    user: int,
    tool: int,
    robot_id: str,
    waypoints,
    subsegments_per_leg: int,
) -> None:
    payload = {
        "version": M01_CACHE_VERSION,
        "robot_id": robot_id,
        "user": user,
        "tool": tool,
        "reference_pose": list(reference_pose),
        "reference_joints": list(reference_joints),
        "waypoints": [list(point) for point in waypoints],
        "subsegments_per_leg": subsegments_per_leg,
        "joint_plane_axis_tool": list(trajectory.joint_plane_axis_tool),
        "max_position_error_mm": trajectory.max_position_error_mm,
        "max_orientation_error_deg": trajectory.max_orientation_error_deg,
        "targets": [
            {
                "joints": list(target.joints),
                "position_error_mm": target.position_error_mm,
                "orientation_error_deg": target.orientation_error_deg,
            }
            for target in trajectory.targets
        ],
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    temporary.replace(path)


def validate_m01_joint_trajectory(
    trajectory: M01JointTrajectory,
    reference_pose: Pose,
    reference_joints: Joints,
    user: int,
    tool: int,
    positive_kinematics: PositiveKinematics,
    position_tolerance_mm: float = 0.15,
    orientation_tolerance_deg: float = 0.20,
) -> bool:
    """Recheck cached leg endpoints against the connected controller."""
    reference_fk = positive_kinematics(reference_joints, user, tool)
    if reference_fk is None:
        return False
    reference_matrix = _matrix_from_pose(reference_fk)
    reference_xyz = np.asarray(reference_pose[:3], dtype=float)
    axis = np.asarray(trajectory.joint_plane_axis_tool, dtype=float)
    sample_indices = list(range(9, len(trajectory.targets), 10))
    if len(trajectory.targets) - 1 not in sample_indices:
        sample_indices.append(len(trajectory.targets) - 1)
    for index in sample_indices:
        target = trajectory.targets[index]
        pose = positive_kinematics(target.joints, user, tool)
        if pose is None:
            return False
        desired = _target_matrix(
            reference_matrix, axis, target.pitch_deg, target.roll_deg
        )
        position_error = np.linalg.norm(
            np.asarray(pose[:3], dtype=float) - reference_xyz
        )
        orientation_error = np.linalg.norm(
            _rotation_vector_deg(desired, _matrix_from_pose(pose))
        )
        if (
            position_error > position_tolerance_mm
            or orientation_error > orientation_tolerance_deg
        ):
            return False
    return True


def _matrix_from_pose(pose: Pose) -> np.ndarray:
    return np.asarray(QuickCalCoordinator._tool_axes(pose), dtype=float).T


def _rotation_vector_deg(reference: np.ndarray, current: np.ndarray) -> np.ndarray:
    relative = reference.T @ current
    cosine = max(-1.0, min(1.0, (float(np.trace(relative)) - 1.0) / 2.0))
    angle = math.acos(cosine)
    if angle < 1e-9:
        return np.zeros(3)
    sine = math.sin(angle)
    if abs(sine) < 1e-7:
        raise ValueError("M01 rotation-vector extraction reached 180 degrees")
    axis = np.asarray(
        (
            relative[2, 1] - relative[1, 2],
            relative[0, 2] - relative[2, 0],
            relative[1, 0] - relative[0, 1],
        ),
        dtype=float,
    ) / (2.0 * sine)
    return axis * math.degrees(angle)


def _axis_rotation(axis: np.ndarray, angle_deg: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    angle = math.radians(angle_deg)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    cross = np.asarray(((0.0, -z, y), (z, 0.0, -x), (-y, x, 0.0)))
    return cosine * np.eye(3) + (1.0 - cosine) * np.outer(axis, axis) + sine * cross


def _target_matrix(
    reference_matrix: np.ndarray,
    pitch_axis: np.ndarray,
    pitch_deg: float,
    roll_deg: float,
) -> np.ndarray:
    roll = math.radians(roll_deg)
    rz = np.asarray(
        (
            (math.cos(roll), -math.sin(roll), 0.0),
            (math.sin(roll), math.cos(roll), 0.0),
            (0.0, 0.0, 1.0),
        )
    )
    return reference_matrix @ _axis_rotation(pitch_axis, pitch_deg) @ rz


def _samples(waypoints, subsegments_per_leg: int):
    for leg_index, (start, end) in enumerate(zip(waypoints, waypoints[1:])):
        for subsegment_index in range(1, subsegments_per_leg + 1):
            ratio = subsegment_index / subsegments_per_leg
            yield (
                start[0] + (end[0] - start[0]) * ratio,
                start[1] + (end[1] - start[1]) * ratio,
                leg_index,
                subsegment_index,
            )


def solve_m01_joint_trajectory(
    reference_pose: Pose,
    reference_joints: Joints,
    user: int,
    tool: int,
    positive_kinematics: PositiveKinematics,
    waypoints,
    subsegments_per_leg: int,
    position_tolerance_mm: float = 0.1,
    orientation_tolerance_deg: float = 0.15,
    progress_callback: ProgressCallback | None = None,
) -> M01JointTrajectory:
    """Solve M01 while fixing J1/J5 and assigning roll directly to J6."""
    if len(reference_pose) != 6 or len(reference_joints) != 6:
        raise ValueError("M01 reference pose and joints must contain six values")
    reference_joints_array = np.asarray(reference_joints, dtype=float)
    reference_fk = positive_kinematics(tuple(reference_joints_array), user, tool)
    if reference_fk is None:
        raise ValueError("M01 could not calculate reference positive kinematics")
    perturbed_joints = reference_joints_array.copy()
    perturbed_joints[3] += 0.1
    perturbed_fk = positive_kinematics(tuple(perturbed_joints), user, tool)
    if perturbed_fk is None:
        raise ValueError("M01 could not identify the J2/J3/J4 joint-plane axis")
    reference_matrix = _matrix_from_pose(reference_fk)
    pitch_vector = _rotation_vector_deg(reference_matrix, _matrix_from_pose(perturbed_fk))
    pitch_norm = float(np.linalg.norm(pitch_vector))
    if pitch_norm < 0.05:
        raise ValueError("M01 joint-plane axis identification was degenerate")
    pitch_axis = pitch_vector / pitch_norm
    reference_xyz = np.asarray(reference_pose[:3], dtype=float)
    variable = reference_joints_array[1:4].copy()
    targets = []
    jacobian = None

    def evaluate(candidate: np.ndarray, pitch_deg: float, roll_deg: float, desired_matrix):
        joints = reference_joints_array.copy()
        joints[1:4] = candidate
        joints[5] = reference_joints_array[5] + roll_deg
        pose = positive_kinematics(tuple(float(value) for value in joints), user, tool)
        if pose is None:
            raise ValueError("M01 PositiveKin failed during constrained solve")
        residual = np.concatenate(
            (
                np.asarray(pose[:3], dtype=float) - reference_xyz,
                _rotation_vector_deg(desired_matrix, _matrix_from_pose(pose)),
            )
        )
        return joints, pose, residual

    samples = tuple(_samples(waypoints, subsegments_per_leg))
    for target_index, (
        pitch_deg,
        roll_deg,
        leg_index,
        subsegment_index,
    ) in enumerate(samples, 1):
        desired_matrix = _target_matrix(
            reference_matrix, pitch_axis, pitch_deg, roll_deg
        )
        if subsegment_index == 1:
            # Direction changes alter the local kinematics enough to justify a
            # fresh finite-difference Jacobian.  Inside a leg, update and reuse
            # it instead of making three extra controller calls per iteration.
            jacobian = None
        if abs(pitch_deg) < 1e-9 and abs(roll_deg) < 1e-9:
            variable = reference_joints_array[1:4].copy()
            jacobian = None
        damping = 0.05
        joints, _pose, residual = evaluate(
            variable, pitch_deg, roll_deg, desired_matrix
        )
        for _iteration in range(14):
            if (
                np.linalg.norm(residual[:3]) <= position_tolerance_mm / 2.0
                and np.linalg.norm(residual[3:]) <= orientation_tolerance_deg / 2.0
            ):
                break
            if jacobian is None:
                jacobian = np.empty((6, 3), dtype=float)
                epsilon = 0.1
                for column in range(3):
                    shifted = variable.copy()
                    shifted[column] += epsilon
                    _sj, _sp, shifted_residual = evaluate(
                        shifted, pitch_deg, roll_deg, desired_matrix
                    )
                    jacobian[:, column] = (
                        shifted_residual - residual
                    ) / epsilon
            lhs = jacobian.T @ jacobian + damping * np.eye(3)
            delta = -np.linalg.solve(lhs, jacobian.T @ residual)
            candidate = variable + np.clip(delta, -6.0, 6.0)
            candidate_joints, candidate_pose, candidate_residual = evaluate(
                candidate, pitch_deg, roll_deg, desired_matrix
            )
            if np.linalg.norm(candidate_residual) <= np.linalg.norm(residual):
                step = candidate - variable
                denominator = float(step @ step)
                if denominator > 1e-9:
                    correction = candidate_residual - residual - jacobian @ step
                    jacobian = jacobian + np.outer(correction, step) / denominator
                variable = candidate
                joints = candidate_joints
                _pose = candidate_pose
                residual = candidate_residual
                damping = max(0.001, damping / 2.0)
            else:
                jacobian = None
                damping *= 10.0
        position_error = float(np.linalg.norm(residual[:3]))
        orientation_error = float(np.linalg.norm(residual[3:]))
        if position_error > position_tolerance_mm or orientation_error > orientation_tolerance_deg:
            raise ValueError(
                f"M01 constrained solve failed at pitch={pitch_deg:+.2f}, "
                f"roll={roll_deg:+.2f}: XYZ={position_error:.3f} mm, "
                f"orientation={orientation_error:.3f} deg"
            )
        targets.append(
            M01JointTarget(
                joints=tuple(float(value) for value in joints),
                pitch_deg=pitch_deg,
                roll_deg=roll_deg,
                leg_index=leg_index,
                subsegment_index=subsegment_index,
                position_error_mm=position_error,
                orientation_error_deg=orientation_error,
            )
        )
        if progress_callback is not None:
            progress_callback(target_index, len(samples))
    return M01JointTrajectory(
        targets=tuple(targets),
        joint_plane_axis_tool=tuple(float(value) for value in pitch_axis),
        max_position_error_mm=max(target.position_error_mm for target in targets),
        max_orientation_error_deg=max(target.orientation_error_deg for target in targets),
    )
