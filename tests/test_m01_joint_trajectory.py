import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from imu_calibration.quickcal_station.m01_joint_trajectory import (
    load_m01_joint_trajectory_cache,
    save_m01_joint_trajectory_cache,
    solve_m01_joint_trajectory,
    validate_m01_joint_trajectory,
)


def matrix_to_pose(matrix, xyz):
    ry = math.asin(max(-1.0, min(1.0, -matrix[2, 0])))
    rx = math.atan2(matrix[2, 1], matrix[2, 2])
    rz = math.atan2(matrix[1, 0], matrix[0, 0])
    return (
        *xyz,
        math.degrees(rx),
        math.degrees(ry),
        math.degrees(rz),
    )


class M01JointTrajectoryTests(unittest.TestCase):
    def test_solver_fixes_j1_j5_and_overlays_j234_with_j6(self):
        reference_joints = (10.0, 20.0, 30.0, 40.0, 50.0, 60.0)
        reference_pose = (100.0, 200.0, 300.0, 0.0, 0.0, 0.0)
        fk_calls = 0

        def synthetic_fk(joints, _user, _tool):
            nonlocal fk_calls
            fk_calls += 1
            pitch = math.radians(joints[3] - reference_joints[3])
            roll = math.radians(joints[5] - reference_joints[5])
            ry = np.asarray(
                (
                    (math.cos(pitch), 0.0, math.sin(pitch)),
                    (0.0, 1.0, 0.0),
                    (-math.sin(pitch), 0.0, math.cos(pitch)),
                )
            )
            rz = np.asarray(
                (
                    (math.cos(roll), -math.sin(roll), 0.0),
                    (math.sin(roll), math.cos(roll), 0.0),
                    (0.0, 0.0, 1.0),
                )
            )
            xyz = (
                reference_pose[0] + joints[1] - reference_joints[1],
                reference_pose[1],
                reference_pose[2] + joints[2] - reference_joints[2],
            )
            return matrix_to_pose(ry @ rz, xyz)

        waypoints = ((0.0, 0.0), (-20.0, 10.0), (20.0, -10.0), (0.0, 0.0))
        progress = []
        trajectory = solve_m01_joint_trajectory(
            reference_pose,
            reference_joints,
            0,
            1,
            synthetic_fk,
            waypoints,
            2,
            progress_callback=lambda completed, total: progress.append(
                (completed, total)
            ),
        )

        self.assertEqual(len(trajectory.targets), 6)
        self.assertLess(trajectory.max_position_error_mm, 0.1)
        self.assertLess(trajectory.max_orientation_error_deg, 0.15)
        self.assertEqual(
            progress,
            [
                (index, len(trajectory.targets))
                for index in range(1, len(trajectory.targets) + 1)
            ],
        )
        for target in trajectory.targets:
            self.assertAlmostEqual(target.joints[0], reference_joints[0])
            self.assertAlmostEqual(target.joints[4], reference_joints[4])
            self.assertAlmostEqual(
                target.joints[5], reference_joints[5] + target.roll_deg
            )
        self.assertEqual(trajectory.targets[-1].joints, reference_joints)
        self.assertLess(fk_calls, 400)

        with tempfile.TemporaryDirectory() as directory:
            cache_path = Path(directory) / "m01.json"
            save_m01_joint_trajectory_cache(
                cache_path,
                trajectory,
                reference_pose,
                reference_joints,
                0,
                1,
                "test-robot",
                waypoints,
                2,
            )
            cached = load_m01_joint_trajectory_cache(
                cache_path,
                reference_pose,
                reference_joints,
                0,
                1,
                "test-robot",
                waypoints,
                2,
            )
            self.assertEqual(cached, trajectory)
            self.assertTrue(
                validate_m01_joint_trajectory(
                    cached,
                    reference_pose,
                    reference_joints,
                    0,
                    1,
                    synthetic_fk,
                )
            )
            self.assertIsNone(
                load_m01_joint_trajectory_cache(
                    cache_path,
                    reference_pose,
                    reference_joints,
                    0,
                    1,
                    "different-robot",
                    waypoints,
                    2,
                )
            )


if __name__ == "__main__":
    unittest.main()
