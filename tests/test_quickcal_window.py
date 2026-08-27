import os
import time
from types import SimpleNamespace
import unittest

from imu_calibration.quickcal_station.workflow import YawLimits
from imu_calibration.quickcal_station.action_config import DEFAULT_ACTIONS

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    from imu_calibration.quickcal_station.coordinator import QuickCalCoordinator, RunState
    from imu_calibration.quickcal_station.robot_device import RobotState
    from imu_calibration.quickcal_station.window import (
        MAG_AUTO_STEPS,
        MAG_TRAJECTORIES,
        QuickCalWindow,
    )
except ModuleNotFoundError as exc:
    if exc.name != "PySide6":
        raise
    QuickCalCoordinator = QuickCalWindow = RobotState = RunState = None
    MAG_AUTO_STEPS = MAG_TRAJECTORIES = None


class ManualMotionHarness:
    if QuickCalWindow is not None:
        _update_manual_motion_result = QuickCalWindow._update_manual_motion_result
        _robot_mode_is_enabled = staticmethod(QuickCalWindow._robot_mode_is_enabled)

    def __init__(self):
        self._manual_motion_target = None
        self._manual_motion_frame = None
        self._manual_motion_kind = ""
        self._manual_motion_started_ns = 0
        self._manual_motion_stable_since_ns = 0
        self._manual_motion_seen_motion = False
        self.result_text = ""
        self.result_level = ""

    @staticmethod
    def _manual_target_errors(target_pose, actual_pose):
        return QuickCalWindow._manual_target_errors(target_pose, actual_pose)

    def _set_manual_motion_result(self, message, level="idle"):
        self.result_text = message
        self.result_level = level

    def _finish_manual_motion(self, message, level):
        self._manual_motion_target = None
        self._manual_motion_frame = None
        self._manual_motion_kind = ""
        self._manual_motion_started_ns = 0
        self._manual_motion_stable_since_ns = 0
        self._manual_motion_seen_motion = False
        self._set_manual_motion_result(message, level)


class GyroMotionHarness:
    if QuickCalWindow is not None:
        _gyro_motion_parameters = QuickCalWindow._gyro_motion_parameters
        _gyro_decel_endpoint = QuickCalWindow._gyro_decel_endpoint
        _on_run_state = QuickCalWindow._on_run_state

    def __init__(self):
        self.coordinator = SimpleNamespace(limits=YawLimits())
        self.robot_actions = dict(DEFAULT_ACTIONS)
        self._auto_action_step = "G01"
        self._gyro_x_phase = "capturing"
        self._gyro_x_phase_started_ns = 0
        self.decel_levels = []
        self.session_badge = object()

    def _config_for_step(self, step_id):
        return self.robot_actions.get(step_id)

    def _set_gyro_x_decel_level(self, level):
        self.decel_levels.append(level)
        return True

    def _fail_gyro_x_auto_action(self, message):
        self.fail_message = message

    def _set_badge(self, *_args):
        pass

    def _update_action_controls(self):
        pass


class MagneticMotionHarness:
    if QuickCalWindow is not None:
        _relative_tool_axis_deg = QuickCalWindow._relative_tool_axis_deg
        _mag_limit_error = QuickCalWindow._mag_limit_error

    def __init__(self):
        self.coordinator = SimpleNamespace(
            limits=YawLimits(),
            _tool_axes=QuickCalCoordinator._tool_axes,
        )
        self._mag_reference_pose = (0.0,) * 6


class FullAutoHarness:
    if QuickCalWindow is not None:
        _try_start_full_auto_step = QuickCalWindow._try_start_full_auto_step

    def __init__(self, step_id):
        self._full_auto_enabled = True
        self._auto_action_step = None
        self.robot_actions = {"A01": object()}
        self.config = SimpleNamespace(enabled=True)
        self.calls = []
        self.coordinator = SimpleNamespace(
            running=True,
            state=RunState.READY,
            current_step=SimpleNamespace(step_id=step_id),
            _check_motion_condition=lambda _step: "",
            confirm_current_action=lambda: self.calls.append(("confirm", step_id)),
        )

    def _config_for_step(self, step_id):
        return self.config

    def _start_accel_auto_action(self, step_id, config):
        self.calls.append(("accel", step_id, config))

    def _start_gyro_x_auto_action(self, step_id):
        self.calls.append(("gyro", step_id))

    def _start_mag_auto_action(self, step_id):
        self.calls.append(("mag", step_id))

    def _update_full_auto_safe_return(self):
        self.calls.append(("safe", "S02"))
        return True

    def _show_error(self, message, modal):
        self.calls.append(("error", message, modal))


class ReportSummaryHarness:
    if QuickCalWindow is not None:
        _format_report_summary = staticmethod(QuickCalWindow._format_report_summary)


@unittest.skipIf(QuickCalWindow is None, "PySide6 is not installed in this Python environment")
class ManualMotionTrackingTests(unittest.TestCase):
    def setUp(self):
        self.window = ManualMotionHarness()

    @staticmethod
    def state(pose, mode=5, user=0, tool=1):
        return RobotState(
            received_monotonic_ns=time.monotonic_ns(),
            controller_timestamp=1,
            mode=mode,
            speed_scaling=1.0,
            joints=(0.0,) * 6,
            pose=tuple(pose),
            tcp_speed=(0.0,) * 6,
            user=user,
            tool=tool,
            digital_inputs=0,
            digital_outputs=0,
        )

    def prepare_pending(self, target):
        now = time.monotonic_ns()
        self.window._manual_motion_target = tuple(target)
        self.window._manual_motion_frame = (0, 1)
        self.window._manual_motion_kind = "姿态运动"
        self.window._manual_motion_started_ns = now - 2_000_000_000
        self.window._manual_motion_stable_since_ns = now - 600_000_000
        self.window._manual_motion_seen_motion = True

    def test_manual_motion_reports_completed_only_inside_pose_tolerance(self):
        target = (420.0, -115.0, 323.0, 0.0, -90.0, 0.0)
        self.prepare_pending(target)

        self.window._update_manual_motion_result(
            self.state((421.0, -115.5, 323.0, 0.4, -90.5, 0.0))
        )

        self.assertIsNone(self.window._manual_motion_target)
        self.assertIn("姿态运动完成", self.window.result_text)

    def test_manual_motion_reports_stopped_short_of_target_as_failure(self):
        target = (420.0, -115.0, 323.0, 0.0, -90.0, 0.0)
        self.prepare_pending(target)

        self.window._update_manual_motion_result(
            self.state((420.0, -115.0, 323.0, 0.0, -76.0, 0.0))
        )

        self.assertIsNone(self.window._manual_motion_target)
        self.assertIn("停止但未到达目标", self.window.result_text)
        self.assertIn("14.00°", self.window.result_text)

    def test_manual_motion_reports_collision_immediately(self):
        target = (420.0, -115.0, 323.0, 0.0, -90.0, 0.0)
        self.prepare_pending(target)

        self.window._update_manual_motion_result(self.state(target, mode=11))

        self.assertIsNone(self.window._manual_motion_target)
        self.assertIn("碰撞", self.window.result_text)

    def test_manual_motion_reports_disabled_robot_immediately(self):
        target = (420.0, -115.0, 323.0, 0.0, -90.0, 0.0)
        self.prepare_pending(target)

        self.window._update_manual_motion_result(self.state(target, mode=4))

        self.assertIsNone(self.window._manual_motion_target)
        self.assertIn("退出使能状态", self.window.result_text)

    def test_angle_error_wraps_across_180_degrees(self):
        position_error, angle_error = self.window._manual_target_errors(
            (0.0, 0.0, 0.0, 179.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, -179.0, 0.0, 0.0),
        )

        self.assertEqual(position_error, 0.0)
        self.assertAlmostEqual(angle_error, 2.0)

    def test_equivalent_euler_angles_at_gimbal_lock_have_zero_error(self):
        _, angle_error = self.window._manual_target_errors(
            (0.0, 0.0, 0.0, 0.0, -90.0, 0.0),
            (0.0, 0.0, 0.0, 180.0, -90.0, 180.0),
        )

        self.assertAlmostEqual(angle_error, 0.0)


@unittest.skipIf(QuickCalWindow is None, "PySide6 is not installed in this Python environment")
class GyroMotionParameterTests(unittest.TestCase):
    def test_g01_starts_decelerating_as_soon_as_six_second_capture_closes(self):
        harness = GyroMotionHarness()

        harness._on_run_state(RunState.WAIT_STAGE_CLOSE.value)

        self.assertEqual(harness._gyro_x_phase, "smooth_decel")
        self.assertEqual(harness.decel_levels, [0])

    def test_protocol_stages_use_configured_tool_axis_and_direction(self):
        harness = GyroMotionHarness()

        self.assertEqual(
            {step: harness._gyro_motion_parameters(step) for step in ("G01", "G02")},
            {
                "G01": ("Rx", -55.0, -45.0, 15.0),
                "G02": ("Rx", 55.0, 45.0, -15.0),
            },
        )
        self.assertEqual(
            {step: harness._gyro_motion_parameters(step) for step in ("G05", "G06")},
            {
                "G05": ("Rz", -85.0, -75.0, 15.0),
                "G06": ("Rz", 85.0, 75.0, -15.0),
            },
        )
        self.assertEqual(
            {
                step: harness._gyro_decel_endpoint(step)
                for step in ("G01", "G02", "G03", "G04", "G05", "G06")
            },
            {
                "G01": 55.0,
                "G02": -55.0,
                "G03": 85.0,
                "G04": -85.0,
                "G05": 85.0,
                "G06": -85.0,
            },
        )

    def test_magnetic_trajectories_keep_work_order_actions_at_fifteen_seconds(self):
        self.assertEqual(tuple(MAG_TRAJECTORIES), MAG_AUTO_STEPS)
        for step_id in ("M01", "M02", "M03"):
            with self.subTest(step_id=step_id):
                self.assertAlmostEqual(
                    sum(duration_s for _axis, _positive, duration_s in MAG_TRAJECTORIES[step_id]),
                    15.0,
                )

        m01 = MAG_TRAJECTORIES["M01"]
        self.assertEqual({axis for axis, _positive, _duration in m01}, {"Rx", "Ry"})
        for axis in ("Rx", "Ry"):
            self.assertEqual(
                {
                    positive
                    for segment_axis, positive, _duration in m01
                    if segment_axis == axis
                },
                {False, True},
            )
        self.assertEqual(
            MAG_TRAJECTORIES["M02"],
            (("Rz", True, 7.5), ("Rz", False, 7.5)),
        )
        self.assertEqual(
            MAG_TRAJECTORIES["M03"],
            (("Rz", False, 7.5), ("Rz", True, 7.5)),
        )
        self.assertEqual(MAG_TRAJECTORIES["M04"], ())

    def test_magnetic_limits_follow_each_work_order_axis(self):
        harness = MagneticMotionHarness()

        self.assertEqual(
            harness._mag_limit_error(
                "M01", SimpleNamespace(pose=(0.0, 0.0, 0.0, 45.0, 0.0, 0.0))
            ),
            "",
        )
        self.assertIn(
            "Tool Rx",
            harness._mag_limit_error(
                "M01", SimpleNamespace(pose=(0.0, 0.0, 0.0, 47.0, 0.0, 0.0))
            ),
        )
        self.assertEqual(
            harness._mag_limit_error(
                "M02", SimpleNamespace(pose=(0.0, 0.0, 0.0, 0.0, 0.0, 75.0))
            ),
            "",
        )
        self.assertIn(
            "Tool Rz",
            harness._mag_limit_error(
                "M03", SimpleNamespace(pose=(0.0, 0.0, 0.0, 0.0, 0.0, -81.0))
            ),
        )
        self.assertEqual(
            harness._mag_limit_error(
                "M04", SimpleNamespace(pose=(0.0, 0.0, 0.0, 0.0, 0.0, 120.0))
            ),
            "",
        )

    def test_full_auto_dispatches_each_ready_step_without_button_clicks(self):
        cases = {
            "P1": [],
            "A01": [("accel", "A01", None)],
            "G02": [("gyro", "G02")],
            "S01": [("confirm", "S01")],
            "S02": [("safe", "S02"), ("confirm", "S02")],
        }
        for step_id, expected in cases.items():
            with self.subTest(step_id=step_id):
                harness = FullAutoHarness(step_id)
                harness._try_start_full_auto_step()
                normalized = [
                    (call[0], call[1], None)
                    if call[0] == "accel"
                    else call
                    for call in harness.calls
                ]
                self.assertEqual(normalized, expected)


@unittest.skipIf(QuickCalWindow is None, "PySide6 is not installed in this Python environment")
class ReportSummaryTests(unittest.TestCase):
    @staticmethod
    def failed_report():
        gyro_quality = tuple(
            SimpleNamespace(
                ok=index in (3, 8, 9),
                reject_flags=0 if index in (3, 8, 9) else 0x01,
                window_count=40 if index in (3, 8, 9) else 20,
            )
            for index in range(11)
        )
        accel_quality = tuple(SimpleNamespace(ok=True) for _ in range(11))
        return SimpleNamespace(
            version=3,
            status=11,
            imu_count=11,
            calibrated_count=0,
            flash_sequence=4,
            mean_rms_mdeg=0,
            gyro_quality=gyro_quality,
            accel_quality=accel_quality,
            gyro_all_ok=False,
            factory_pass=False,
        )

    def test_failed_report_summary_counts_each_quality_row(self):
        text = ReportSummaryHarness._format_report_summary(self.failed_report())

        self.assertIn("未通过：质量不足（status=11）", text)
        self.assertIn("Gyro=3/11（报告头 nCal=0）", text)
        self.assertIn("Accel=11/11", text)
        self.assertIn("本次 Flash 未写入｜当前 calSeq=4", text)
        self.assertIn("平均 RMS=--", text)


if __name__ == "__main__":
    unittest.main()
