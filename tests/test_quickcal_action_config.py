import tempfile
from pathlib import Path
import unittest

from imu_calibration.quickcal_station.action_config import (
    DEFAULT_ACTIONS,
    RobotActionConfig,
    gravity_after_tool_rotation,
    load_action_config,
    save_action_config,
    vector_angle_deg,
)


class ActionConfigTests(unittest.TestCase):
    def test_default_a01_rz_rotation_matches_observed_gravity(self):
        observed = (-0.999, 0.042, -0.029)
        config = RobotActionConfig()
        predicted = gravity_after_tool_rotation(observed, config.axis, config.degrees)
        error_deg = vector_angle_deg(predicted, (1.0, 0.0, 0.0))

        self.assertEqual(config.axis, "Rz")
        self.assertEqual(config.degrees, 180.0)
        self.assertLess(error_deg, 5.0)
        self.assertAlmostEqual(error_deg, 2.91, places=1)

    def test_rx_rotation_cannot_flip_negative_x_to_positive_x(self):
        observed = (-0.999, 0.042, -0.029)
        predicted = gravity_after_tool_rotation(observed, "Rx", 180.0)
        self.assertGreater(vector_angle_deg(predicted, (1.0, 0.0, 0.0)), 170.0)

    def test_default_a02_returns_positive_x_to_negative_x(self):
        observed = (0.999, -0.042, -0.029)
        config = DEFAULT_ACTIONS["A02"]
        predicted = gravity_after_tool_rotation(observed, config.axis, config.degrees)
        error_deg = vector_angle_deg(predicted, (-1.0, 0.0, 0.0))

        self.assertEqual(config.axis, "Rz")
        self.assertEqual(config.degrees, -180.0)
        self.assertLess(error_deg, 5.0)

    def test_default_a03_rotates_negative_x_to_positive_y(self):
        observed = (-0.999, 0.042, -0.029)
        config = DEFAULT_ACTIONS["A03"]
        predicted = gravity_after_tool_rotation(observed, config.axis, config.degrees)
        error_deg = vector_angle_deg(predicted, (0.0, 1.0, 0.0))

        self.assertEqual(config.axis, "Rz")
        self.assertEqual(config.degrees, 90.0)
        self.assertLess(error_deg, 5.0)

    def test_default_a04_rotates_positive_y_to_negative_y(self):
        observed = (0.042, 0.999, -0.029)
        config = DEFAULT_ACTIONS["A04"]
        predicted = gravity_after_tool_rotation(observed, config.axis, config.degrees)
        error_deg = vector_angle_deg(predicted, (0.0, -1.0, 0.0))

        self.assertEqual(config.axis, "Rz")
        self.assertEqual(config.degrees, -180.0)
        self.assertLess(error_deg, 5.0)

    def test_a05_would_require_positive_rx_quarter_turn_from_negative_y(self):
        config = DEFAULT_ACTIONS["A05"]
        predicted = gravity_after_tool_rotation(
            (0.0, -1.0, 0.0), config.axis, config.degrees
        )

        self.assertEqual(config.axis, "Rx")
        self.assertEqual(config.degrees, 90.0)
        self.assertLess(vector_angle_deg(predicted, (0.0, 0.0, 1.0)), 0.001)

    def test_default_a06_rotates_positive_z_to_negative_z(self):
        config = DEFAULT_ACTIONS["A06"]
        predicted = gravity_after_tool_rotation(
            (0.0, 0.0, 1.0), config.axis, config.degrees
        )

        self.assertEqual(config.axis, "Rx")
        self.assertEqual(config.degrees, -180.0)
        self.assertLess(vector_angle_deg(predicted, (0.0, 0.0, -1.0)), 0.001)

    def test_action_config_round_trip(self):
        expected = RobotActionConfig(True, "Rz", -90.0, 4, 45.0)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "actions.local.json"
            save_action_config(path, {"A01": expected})
            loaded = load_action_config(path)
        self.assertEqual(loaded["A01"], expected)
        self.assertEqual(loaded["A02"], DEFAULT_ACTIONS["A02"])

    def test_action_config_rejects_unsafe_values(self):
        with self.assertRaises(ValueError):
            RobotActionConfig(degrees=181.0)
        with self.assertRaises(ValueError):
            RobotActionConfig(velocity_percent=81)


if __name__ == "__main__":
    unittest.main()
