import json
from pathlib import Path
import tempfile
import unittest

from imu_calibration.quickcal_station.pose_store import (
    TaughtPose,
    load_legacy_safe_pose,
    load_pose_config,
    save_pose_config,
)


class PoseStoreTests(unittest.TestCase):
    def setUp(self):
        self.pose = TaughtPose(
            pose=(330.0, -98.0, 276.0, 87.0, 44.0, 91.0),
            joints=(0.0, -20.0, 80.0, 0.0, 45.0, 0.0),
            user=0,
            tool=2,
            recorded_at="2026-08-25T12:00:00+08:00",
        )

    def test_round_trip_preserves_pose_frames_and_joints(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "poses.local.json"
            save_pose_config(path, {"safe": self.pose, "neutral": self.pose})
            loaded = load_pose_config(path)
        self.assertEqual(loaded["safe"], self.pose)
        self.assertEqual(loaded["neutral"].tool, 2)

    def test_rejects_invalid_pose(self):
        with self.assertRaises(ValueError):
            TaughtPose((1.0, 2.0), (), 0, 0, "now")

    def test_loads_legacy_safe_pose_without_joint_values(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "recorded_pose.json"
            path.write_text(
                json.dumps(
                    {
                        "recorded_at": "2026-08-14T09:11:06",
                        "pose": [330.0, -98.0, 276.0, 87.0, 44.0, 91.0],
                        "user": 0,
                        "tool": 0,
                    }
                ),
                encoding="utf-8",
            )
            loaded = load_legacy_safe_pose(path)
        self.assertIsNotNone(loaded)
        self.assertEqual(loaded.joints, ())


if __name__ == "__main__":
    unittest.main()
