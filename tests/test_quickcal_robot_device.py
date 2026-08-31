import unittest

try:
    from PySide6.QtCore import QCoreApplication

    from imu_calibration.quickcal_station.robot_device import RobotDevice
except ModuleNotFoundError as exc:
    if exc.name != "PySide6":
        raise
    QCoreApplication = None
    RobotDevice = None


class FakeDashboard:
    def __init__(self):
        self.calls = []

    def RelMovLTool(self, *offsets, **options):
        self.calls.append((offsets, options))
        return "0,{}"

    def MovL(self, *pose, **options):
        self.calls.append(("MovL", pose, options))
        return "0,{}"

    def MovJ(self, *pose, **options):
        self.calls.append(("MovJ", pose, options))
        return "0,{}"

    def PositiveKin(self, *joints, **options):
        self.calls.append(("PositiveKin", joints, options))
        return "0,{1,2,3,4,5,6}"

    def SpeedFactor(self, percent):
        self.calls.append(("SpeedFactor", percent))
        return "0,{}"

    def MoveJog(self, command, **options):
        self.calls.append(("MoveJog", command, options))
        return "0,{}"

    def SetTool(self, index, table):
        self.calls.append(("SetTool", index, table))
        return "0,{}"

    def Tool(self, index):
        self.calls.append(("Tool", index))
        return "0,{}"


@unittest.skipIf(QCoreApplication is None, "PySide6 is not installed in this Python environment")
class RobotDeviceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QCoreApplication.instance() or QCoreApplication([])

    def test_large_relative_rotation_is_split_at_orientation_boundary(self):
        dashboard = FakeDashboard()
        robot = RobotDevice()
        robot.dashboard = dashboard

        self.assertTrue(robot.relative_tool_rotation("Rz", 180.0, 5))

        self.assertEqual(len(dashboard.calls), 2)
        for offsets, options in dashboard.calls:
            self.assertEqual(offsets[:5], (0.0, 0.0, 0.0, 0.0, 0.0))
            self.assertAlmostEqual(offsets[5], 90.0)
            self.assertEqual(options["v"], 5)
        self.assertEqual(dashboard.calls[0][1]["cp"], 100)
        self.assertEqual(dashboard.calls[1][1]["cp"], 0)

    def test_relative_rotation_velocity_is_capped_at_full_speed(self):
        dashboard = FakeDashboard()
        robot = RobotDevice()
        robot.dashboard = dashboard

        self.assertTrue(robot.relative_tool_rotation("Rz", 90.0, 150))

        self.assertEqual(dashboard.calls[0][1]["v"], 100)

    def test_pose_and_joint_moves_are_capped_at_full_speed(self):
        dashboard = FakeDashboard()
        robot = RobotDevice()
        robot.dashboard = dashboard

        self.assertTrue(robot.move_pose_j((0.0,) * 6, 150, user=0, tool=1))
        self.assertTrue(robot.move_joints((0.0,) * 6, 150))

        self.assertEqual(dashboard.calls[0][2]["v"], 100)
        self.assertEqual(dashboard.calls[1][2]["v"], 100)

    def test_joint_move_supports_acceleration_and_blending(self):
        dashboard = FakeDashboard()
        robot = RobotDevice()
        robot.dashboard = dashboard

        self.assertTrue(robot.move_joints((0.0,) * 6, 12, 30, 85))

        self.assertEqual(
            dashboard.calls[0][2], {"a": 30, "v": 12, "cp": 85}
        )

    def test_positive_kinematics_parses_controller_pose(self):
        dashboard = FakeDashboard()
        robot = RobotDevice()
        robot.dashboard = dashboard

        self.assertEqual(
            robot.positive_kinematics((0.0,) * 6, 0, 1),
            (1.0, 2.0, 3.0, 4.0, 5.0, 6.0),
        )
        self.assertEqual(
            dashboard.calls[0],
            ("PositiveKin", (0.0,) * 6, {"user": 0, "tool": 1}),
        )

    def test_negative_half_turn_keeps_reverse_direction_in_both_segments(self):
        dashboard = FakeDashboard()
        robot = RobotDevice()
        robot.dashboard = dashboard

        self.assertTrue(robot.relative_tool_rotation("Rz", -180.0, 20))

        self.assertEqual(len(dashboard.calls), 2)
        self.assertEqual([call[0][5] for call in dashboard.calls], [-90.0, -90.0])
        self.assertEqual([call[1]["cp"] for call in dashboard.calls], [100, 0])

    def test_relative_rotation_accepts_lower_acceleration_for_smooth_endpoint(self):
        dashboard = FakeDashboard()
        robot = RobotDevice()
        robot.dashboard = dashboard

        self.assertTrue(
            robot.relative_tool_rotation(
                "Ry", 8.0, velocity_percent=15, acceleration_percent=5
            )
        )

        self.assertEqual(dashboard.calls[0][1]["v"], 15)
        self.assertEqual(dashboard.calls[0][1]["a"], 5)

    def test_tool_jog_sets_speed_and_uses_tool_coordinates(self):
        dashboard = FakeDashboard()
        robot = RobotDevice()
        robot.dashboard = dashboard

        self.assertTrue(robot.start_tool_jog("Rx", True, 15, user=0, tool=1))
        self.assertTrue(robot.stop_tool_jog())

        self.assertEqual(dashboard.calls[0], ("SpeedFactor", 15))
        self.assertEqual(
            dashboard.calls[1],
            ("MoveJog", "Rx+", {"coordtype": 2, "user": 0, "tool": 1}),
        )
        self.assertEqual(dashboard.calls[2], ("MoveJog", "", {}))

    def test_tool_jog_supports_all_rotation_axes_and_directions(self):
        for axis in ("Rx", "Ry", "Rz"):
            for positive in (True, False):
                with self.subTest(axis=axis, positive=positive):
                    dashboard = FakeDashboard()
                    robot = RobotDevice()
                    robot.dashboard = dashboard

                    self.assertTrue(
                        robot.start_tool_jog(axis, positive, 15, user=0, tool=1)
                    )

                    self.assertEqual(dashboard.calls[0], ("SpeedFactor", 15))
                    self.assertEqual(
                        dashboard.calls[1],
                        (
                            "MoveJog",
                            f"{axis}{'+' if positive else '-'}",
                            {"coordtype": 2, "user": 0, "tool": 1},
                        ),
                    )

    def test_switch_tool_jog_reuses_existing_speed_factor(self):
        dashboard = FakeDashboard()
        robot = RobotDevice()
        robot.dashboard = dashboard

        self.assertTrue(robot.switch_tool_jog("Ry", False, user=0, tool=1))

        self.assertEqual(
            dashboard.calls,
            [
                (
                    "MoveJog",
                    "Ry-",
                    {"coordtype": 2, "user": 0, "tool": 1},
                )
            ],
        )

    def test_tool_frame_is_saved_before_being_activated(self):
        dashboard = FakeDashboard()
        robot = RobotDevice()
        robot.dashboard = dashboard

        self.assertTrue(
            robot.set_and_activate_tool(1, (0.0, 0.0, 100.0, 0.0, 0.0, 50.0))
        )

        self.assertEqual(
            dashboard.calls,
            [
                ("SetTool", 1, "{0.000000,0.000000,100.000000,0.000000,0.000000,50.000000}"),
                ("Tool", 1),
            ],
        )

    def test_combined_relative_tool_move_uses_all_six_offsets(self):
        dashboard = FakeDashboard()
        robot = RobotDevice()
        robot.dashboard = dashboard

        self.assertTrue(robot.relative_tool_move((10.0, -5.0, 2.0, 1.0, -2.0, 3.0), 25))

        offsets, options = dashboard.calls[0]
        self.assertEqual(offsets, (10.0, -5.0, 2.0, 1.0, -2.0, 3.0))
        self.assertEqual(options, {"user": -1, "tool": -1, "a": 20, "v": 25, "cp": 0})

    def test_combined_relative_tool_move_accepts_blend_and_acceleration(self):
        dashboard = FakeDashboard()
        robot = RobotDevice()
        robot.dashboard = dashboard

        self.assertTrue(
            robot.relative_tool_move(
                (0.0, 0.0, 0.0, 15.0, -25.0, 0.0),
                35,
                acceleration_percent=30,
                blend_percent=50,
            )
        )

        offsets, options = dashboard.calls[0]
        self.assertEqual(offsets, (0.0, 0.0, 0.0, 15.0, -25.0, 0.0))
        self.assertEqual(
            options,
            {"user": -1, "tool": -1, "a": 30, "v": 35, "cp": 50},
        )

    def test_absolute_tcp_move_uses_pose_mode_and_explicit_frames(self):
        dashboard = FakeDashboard()
        robot = RobotDevice()
        robot.dashboard = dashboard
        target = (420.0, -35.0, 510.0, 180.0, 0.0, 50.0)

        self.assertTrue(robot.move_pose_l(target, 25, user=0, tool=1))

        self.assertEqual(
            dashboard.calls,
            [
                (
                    "MovL",
                    (*target, 0),
                    {"user": 0, "tool": 1, "a": 20, "v": 25, "cp": 0},
                )
            ],
        )


if __name__ == "__main__":
    unittest.main()
