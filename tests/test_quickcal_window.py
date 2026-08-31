import os
import math
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from imu_calibration.quickcal_station.workflow import YawLimits
from imu_calibration.quickcal_station.action_config import DEFAULT_ACTIONS

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    from imu_calibration.quickcal_station.coordinator import QuickCalCoordinator, RunState
    from imu_calibration.quickcal_station.robot_device import RobotState
    from imu_calibration.quickcal_station.window import (
        MAG_AUTO_STEPS,
        MAG_M01_BLEND_PERCENT,
        MAG_M01_J6_SAFE_DEG,
        MAG_M01_POSE_WAYPOINTS,
        MAG_M01_SEGMENT_S,
        MAG_M01_SUBSEGMENTS_PER_LEG,
        MAG_M01_TURN_BLEND_PERCENT,
        MAG_M01_VELOCITY_PERCENT,
        MAG_M01_XYZ_TOLERANCE_MM,
        MAG_M04_MOTION_GRACE_S,
        MAG_M04_STOP_SETTLE_S,
        MAG_TRAJECTORIES,
        MAG_YAW_STOP_LEAD_DEG,
        MAG_YAW_TURN_SETTLE_S,
        NEUTRAL_RETURN_SPEED_PERCENT,
        QuickCalWindow,
    )
except ModuleNotFoundError as exc:
    if exc.name != "PySide6":
        raise
    QuickCalCoordinator = QuickCalWindow = RobotState = RunState = None
    MAG_AUTO_STEPS = MAG_M01_BLEND_PERCENT = MAG_M01_J6_SAFE_DEG = None
    MAG_M01_POSE_WAYPOINTS = MAG_TRAJECTORIES = None
    MAG_M01_SEGMENT_S = MAG_M01_SUBSEGMENTS_PER_LEG = None
    MAG_M01_TURN_BLEND_PERCENT = MAG_M01_XYZ_TOLERANCE_MM = None
    MAG_M01_VELOCITY_PERCENT = None
    MAG_M04_MOTION_GRACE_S = MAG_M04_STOP_SETTLE_S = None
    MAG_YAW_STOP_LEAD_DEG = MAG_YAW_TURN_SETTLE_S = None
    NEUTRAL_RETURN_SPEED_PERCENT = None


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
        _matmul3 = staticmethod(QuickCalWindow._matmul3)
        _m01_pose_samples = staticmethod(QuickCalWindow._m01_pose_samples)
        _m01_fixed_frame_angles = QuickCalWindow._m01_fixed_frame_angles
        _mag_limit_error = QuickCalWindow._mag_limit_error

    def __init__(self):
        self.coordinator = SimpleNamespace(
            limits=YawLimits(),
            _tool_axes=QuickCalCoordinator._tool_axes,
        )
        self._mag_reference_pose = (0.0,) * 6
        self._mag_reference_joints = (0.0,) * 6


class MagYawMotionHarness:
    if QuickCalWindow is not None:
        _update_mag_auto_action = QuickCalWindow._update_mag_auto_action
        _start_mag_segment = QuickCalWindow._start_mag_segment
        _relative_tool_axis_deg = QuickCalWindow._relative_tool_axis_deg

    def __init__(self, step_id="M02"):
        self._auto_action_step = step_id
        self._mag_phase = "capturing"
        self._mag_phase_started_ns = 0
        self._mag_segment_index = 0
        self._mag_pending_segment_index = -1
        self._mag_reference_pose = (0.0,) * 6
        self._auto_action_stable_since_ns = 0
        self._mag_jog_active = True
        self._mag_speed_factor = 22
        self._mag_speed_source = "test"
        self.logs = []
        self.fail_message = ""
        self.robot = SimpleNamespace(stops=0, switches=[])
        self.robot.stop_tool_jog = self._stop_tool_jog
        self.robot.switch_tool_jog = self._switch_tool_jog
        self.coordinator = SimpleNamespace(
            state=RunState.CAPTURING,
            capture_started_ns=0,
            _tool_axes=QuickCalCoordinator._tool_axes,
            recorder=SimpleNamespace(marker=lambda *_args: None),
        )

    def _stop_tool_jog(self):
        self.robot.stops += 1
        return True

    def _switch_tool_jog(self, axis, positive, user, tool):
        self.robot.switches.append((axis, positive, user, tool))
        return True

    def _check_auto_action_timeout(self, _now):
        pass

    def _mag_limit_error(self, _step_id, _state):
        return ""

    def _fail_mag_auto_action(self, message):
        self.fail_message = message

    def _append_log(self, message, level="info"):
        self.logs.append((message, level))

    def update_at(self, seconds, rx_deg, *, mode=5, angular=0.0):
        state = SimpleNamespace(
            mode=mode,
            linear_speed_norm=0.0,
            angular_speed_norm=angular,
            pose=(0.0, 0.0, 0.0, rx_deg, 0.0, 0.0),
            user=0,
            tool=1,
        )
        with patch(
            "imu_calibration.quickcal_station.window.time.monotonic_ns",
            return_value=int(seconds * 1_000_000_000),
        ):
            self._update_mag_auto_action(state)


class FakeM01Robot:
    def __init__(self):
        self.moves = []
        self.speed_factors = []

    def set_speed_factor(self, percent):
        self.speed_factors.append(percent)
        return True

    def move_pose_l(
        self,
        target,
        velocity,
        *,
        user,
        tool,
        acceleration_percent,
        blend_percent,
    ):
        self.moves.append(
            (
                target,
                velocity,
                user,
                tool,
                acceleration_percent,
                blend_percent,
            )
        )
        return True

    def move_joints(
        self,
        target,
        velocity_percent,
        acceleration_percent,
        blend_percent,
    ):
        self.moves.append(
            (
                target,
                velocity_percent,
                acceleration_percent,
                blend_percent,
            )
        )
        return True


class M01TrajectoryHarness:
    if QuickCalWindow is not None:
        _matmul3 = staticmethod(QuickCalWindow._matmul3)
        _continuous_euler_xyz = staticmethod(QuickCalWindow._continuous_euler_xyz)
        _m01_pose_samples = staticmethod(QuickCalWindow._m01_pose_samples)
        _m01_absolute_targets = QuickCalWindow._m01_absolute_targets
        _m01_fixed_frame_angles = QuickCalWindow._m01_fixed_frame_angles
        _orientation_error_deg = QuickCalWindow._orientation_error_deg
        _m01_post_solve_state_error = QuickCalWindow._m01_post_solve_state_error
        _start_m01_combined_trajectory = QuickCalWindow._start_m01_combined_trajectory
        _cancel_m01_queue = QuickCalWindow._cancel_m01_queue
        _queue_next_m01_target = QuickCalWindow._queue_next_m01_target

    def __init__(self):
        self.reference_joints = (-6.0, -8.0, -114.0, -58.0, -96.0, 40.0)
        self._mag_reference_pose = (0.0,) * 6
        self._mag_reference_joints = self.reference_joints
        samples = self._m01_pose_samples()
        targets = []
        for pitch, roll, leg, subsegment in samples:
            joints = list(self.reference_joints)
            joints[1] += pitch * 0.2
            joints[2] -= pitch * 0.3
            joints[3] += pitch * 0.1
            joints[5] += roll
            targets.append(
                SimpleNamespace(
                    joints=tuple(joints),
                    pitch_deg=pitch,
                    roll_deg=roll,
                    leg_index=leg,
                    subsegment_index=subsegment,
                    position_error_mm=0.01,
                    orientation_error_deg=0.02,
                )
            )
        self._mag_m01_joint_trajectory = SimpleNamespace(targets=tuple(targets))
        self._mag_m01_queue_index = 0
        self._mag_m01_queue_token = 0
        self._mag_speed_factor = 0
        self._mag_speed_source = ""
        self._auto_action_seen_motion = False
        self._auto_action_step = "M01"
        self._mag_phase = "combined_capturing"
        self.fail_message = ""
        self.logs = []
        self.robot = FakeM01Robot()
        self.coordinator = SimpleNamespace(
            _tool_axes=QuickCalCoordinator._tool_axes,
            recorder=SimpleNamespace(marker=lambda *_args: None),
            state=RunState.CAPTURING,
        )

    def _fail_mag_auto_action(self, message):
        self.fail_message = message

    def _append_log(self, message, level="info"):
        self.logs.append((message, level))


class M01CaptureHarness:
    if QuickCalWindow is not None:
        _update_mag_auto_action = QuickCalWindow._update_mag_auto_action
        _m01_at_final_reference = QuickCalWindow._m01_at_final_reference
        _orientation_error_deg = QuickCalWindow._orientation_error_deg
        _angle_distance_deg = staticmethod(QuickCalWindow._angle_distance_deg)
        _at_taught_pose = QuickCalWindow._at_taught_pose

    def __init__(self, phase="combined_capturing"):
        self._auto_action_step = "M01"
        self._mag_phase = phase
        self._mag_phase_started_ns = 0
        self._mag_reference_pose = (500.0, -170.0, 344.0, 0.0, 0.0, 0.0)
        self._mag_reference_joints = (-6.0, -8.0, -114.0, -58.0, -96.0, 45.0)
        self._auto_action_seen_motion = True
        self._mag_m01_idle_since_ns = 0
        self._mag_m01_completed_motion = False
        self.logs = []
        self.fail_message = ""
        neutral = SimpleNamespace(
            pose=self._mag_reference_pose,
            joints=self._mag_reference_joints,
            user=0,
            tool=1,
        )
        self.taught_poses = {"neutral": neutral}
        self.robot = SimpleNamespace(
            latest_state=None,
            moves=[],
            activated=[],
        )
        self.robot.activate_frames = lambda user, tool: (
            self.robot.activated.append((user, tool)) or True
        )
        self.robot.move_joints = lambda joints, velocity: (
            self.robot.moves.append(("joints", joints, velocity)) or True
        )
        self.robot.move_pose_j = lambda pose, velocity, user, tool: (
            self.robot.moves.append(("pose", pose, velocity, user, tool)) or True
        )
        m01_step = SimpleNamespace(step_id="M01", capture_s=40.0)
        self.coordinator = SimpleNamespace(
            state=RunState.CAPTURING if phase == "combined_capturing" else RunState.READY,
            capture_started_ns=0,
            current_step=(
                m01_step if phase == "combined_capturing" else SimpleNamespace(step_id="M02")
            ),
            steps=(m01_step,),
            _tool_axes=QuickCalCoordinator._tool_axes,
        )

    def _check_auto_action_timeout(self, _now):
        pass

    def _mag_limit_error(self, _step_id, _state):
        return ""

    def _restore_mag_speed_factor(self):
        pass

    def _fail_mag_auto_action(self, message):
        self.fail_message = message

    def _append_log(self, message, level="info"):
        self.logs.append((message, level))

    def state(self, *, final=True):
        return SimpleNamespace(
            mode=5,
            linear_speed_norm=0.0,
            angular_speed_norm=0.0,
            pose=(
                self._mag_reference_pose
                if final
                else (500.0, -170.0, 344.0, 0.0, 20.0, 0.0)
            ),
            joints=(
                self._mag_reference_joints
                if final
                else (-6.0, -20.0, -100.0, -70.0, -96.0, 30.0)
            ),
            user=0,
            tool=1,
        )


class M04MotionHarness:
    if QuickCalWindow is not None:
        _update_mag_auto_action = QuickCalWindow._update_mag_auto_action

    def __init__(self, phase="static_wait_stop"):
        self._auto_action_step = "M04"
        self._auto_action_stable_since_ns = 0
        self._mag_phase = phase
        self._mag_phase_started_ns = 10_000_000_000
        self._mag_m04_motion_since_ns = 0
        self._mag_reference_pose = (0.0,) * 6
        self.logs = []
        self.fail_message = ""
        self.confirm_calls = 0
        self.coordinator = SimpleNamespace(
            current_step=SimpleNamespace(exit_s=5.0),
            condition_stable_since_ns=0,
            confirm_current_action=self._confirm,
        )

    def _check_auto_action_timeout(self, _now):
        pass

    def _mag_limit_error(self, _step_id, _state):
        return ""

    def _fail_mag_auto_action(self, message):
        self.fail_message = message

    def _append_log(self, message, level="info"):
        self.logs.append((message, level))

    def _confirm(self):
        self.confirm_calls += 1
        return True

    def update_at(self, seconds, *, mode=5, linear=0.0, angular=0.0):
        state = SimpleNamespace(
            mode=mode,
            linear_speed_norm=linear,
            angular_speed_norm=angular,
            pose=(0.0,) * 6,
        )
        with patch(
            "imu_calibration.quickcal_station.window.time.monotonic_ns",
            return_value=int(seconds * 1_000_000_000),
        ):
            self._update_mag_auto_action(state)


class FullAutoHarness:
    if QuickCalWindow is not None:
        _try_start_full_auto_step = QuickCalWindow._try_start_full_auto_step

    def __init__(self, step_id):
        self._full_auto_enabled = True
        self._full_auto_next_start_not_before_ns = 0
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

    def _update_full_auto_neutral_return(self):
        self.calls.append(("neutral", "S02"))
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

    def test_magnetic_trajectories_match_latest_work_order_times_and_endpoints(self):
        self.assertEqual(tuple(MAG_TRAJECTORIES), MAG_AUTO_STEPS)
        self.assertEqual(
            {
                step_id: sum(
                    duration_s
                    for _axis, _positive, duration_s in MAG_TRAJECTORIES[step_id]
                )
                for step_id in ("M02", "M03")
            },
            {"M02": 10.0, "M03": 10.0},
        )
        self.assertEqual(MAG_TRAJECTORIES["M01"], ())
        self.assertAlmostEqual(
            (len(MAG_M01_POSE_WAYPOINTS) - 1) * MAG_M01_SEGMENT_S,
            40.0,
        )
        self.assertEqual(MAG_M01_POSE_WAYPOINTS[0], (0.0, 0.0))
        self.assertEqual(MAG_M01_POSE_WAYPOINTS[-1], (0.0, 0.0))
        self.assertAlmostEqual(
            max(point[0] for point in MAG_M01_POSE_WAYPOINTS), 75.0
        )
        self.assertAlmostEqual(
            min(point[0] for point in MAG_M01_POSE_WAYPOINTS), -75.0
        )
        self.assertAlmostEqual(
            max(point[1] for point in MAG_M01_POSE_WAYPOINTS), 75.0
        )
        self.assertAlmostEqual(
            min(point[1] for point in MAG_M01_POSE_WAYPOINTS), -75.0
        )
        harness = M01TrajectoryHarness()
        self.assertEqual(MAG_M01_BLEND_PERCENT, 100)
        self.assertEqual(
            MAG_TRAJECTORIES["M02"],
            (("Rx", True, 5.0), ("Rx", False, 5.0)),
        )
        self.assertEqual(
            MAG_TRAJECTORIES["M03"],
            (("Rx", False, 5.0), ("Rx", True, 5.0)),
        )
        self.assertEqual(MAG_TRAJECTORIES["M04"], ())
        self.assertEqual(MAG_YAW_STOP_LEAD_DEG, 7.0)
        self.assertEqual(MAG_YAW_TURN_SETTLE_S, 0.5)

    def test_m02_brakes_early_waits_for_continuous_idle_then_reverses(self):
        harness = MagYawMotionHarness("M02")

        harness.update_at(4.2, 38.0, mode=8, angular=9.0)
        self.assertEqual(harness.robot.stops, 1)
        self.assertEqual(harness._mag_phase, "wait_segment_stop")
        self.assertEqual(harness._mag_pending_segment_index, 1)

        harness.update_at(5.0, 44.7)
        harness.update_at(5.49, 44.7)
        self.assertEqual(harness.robot.switches, [])
        harness.update_at(5.5, 44.7)

        self.assertEqual(harness._mag_phase, "capturing")
        self.assertEqual(harness._mag_segment_index, 1)
        self.assertEqual(harness.robot.switches, [("Rx", False, 0, 1)])
        self.assertEqual(harness.fail_message, "")

    def test_m03_return_leg_brakes_before_crossing_neutral(self):
        harness = MagYawMotionHarness("M03")
        harness._mag_segment_index = 1

        harness.update_at(9.4, -7.0, mode=8, angular=9.0)

        self.assertEqual(harness.robot.stops, 1)
        self.assertEqual(harness._mag_phase, "capture_final_stop")
        self.assertEqual(harness.fail_message, "")

    def test_m01_queues_constrained_j234_and_j6_overlay(self):
        harness = M01TrajectoryHarness()
        state = SimpleNamespace(user=0, tool=1)
        callbacks = []

        with patch(
            "imu_calibration.quickcal_station.window.QTimer.singleShot",
            side_effect=lambda _delay, callback: callbacks.append(callback),
        ):
            self.assertTrue(harness._start_m01_combined_trajectory(state))
            self.assertEqual(harness.robot.moves, [])
            while callbacks:
                callbacks.pop(0)()

        self.assertEqual(harness.fail_message, "")
        self.assertEqual(harness.robot.speed_factors, [100])
        expected_count = (
            len(MAG_M01_POSE_WAYPOINTS) - 1
        ) * MAG_M01_SUBSEGMENTS_PER_LEG
        self.assertEqual(len(harness.robot.moves), expected_count)
        self.assertEqual(
            [move[3] for move in harness.robot.moves],
            [MAG_M01_BLEND_PERCENT] * (expected_count - 1) + [0],
        )
        self.assertTrue(
            all(
                move[1:3] == (MAG_M01_VELOCITY_PERCENT, 20)
                for move in harness.robot.moves
            )
        )
        self.assertTrue(
            all(
                move[0][0] == harness.reference_joints[0]
                and move[0][4] == harness.reference_joints[4]
                for move in harness.robot.moves
            )
        )
        self.assertTrue(
            any(
                abs(target.pitch_deg) > 1.0 and abs(target.roll_deg) > 1.0
                for target in harness._mag_m01_joint_trajectory.targets
            )
        )
        self.assertEqual(harness.robot.moves[-1][0], harness.reference_joints)

    def test_m01_cancel_prevents_scheduled_batches_from_sending_more_targets(self):
        harness = M01TrajectoryHarness()
        callbacks = []

        with patch(
            "imu_calibration.quickcal_station.window.QTimer.singleShot",
            side_effect=lambda _delay, callback: callbacks.append(callback),
        ):
            self.assertTrue(
                harness._start_m01_combined_trajectory(
                    SimpleNamespace(user=0, tool=1)
                )
            )
            callbacks.pop(0)()
            queued_before_cancel = len(harness.robot.moves)
            harness._cancel_m01_queue()
            while callbacks:
                callbacks.pop(0)()

        self.assertEqual(queued_before_cancel, 4)
        self.assertEqual(len(harness.robot.moves), queued_before_cancel)

    def test_m01_post_solve_recheck_requires_fresh_unchanged_neutral(self):
        harness = M01TrajectoryHarness()
        now = 20_000_000_000
        reference = SimpleNamespace(
            received_monotonic_ns=now - 100_000_000,
            mode=5,
            linear_speed_norm=0.0,
            angular_speed_norm=0.0,
            user=0,
            tool=1,
            joints=harness.reference_joints,
            pose=(500.0, -170.0, 344.0, -180.0, -15.0, 0.0),
        )
        current = SimpleNamespace(**vars(reference))
        with patch(
            "imu_calibration.quickcal_station.window.time.monotonic_ns",
            return_value=now,
        ):
            self.assertEqual(
                harness._m01_post_solve_state_error(reference, current), ""
            )
            current.received_monotonic_ns = now - 1_100_000_000
            self.assertIn(
                "反馈已超时",
                harness._m01_post_solve_state_error(reference, current),
            )
            current.received_monotonic_ns = now - 100_000_000
            current.joints = (
                reference.joints[0],
                reference.joints[1] + 0.3,
                *reference.joints[2:],
            )
            self.assertIn(
                "中位发生变化",
                harness._m01_post_solve_state_error(reference, current),
            )

    def test_m01_accepts_a_completed_trajectory_at_capture_end(self):
        harness = M01CaptureHarness()
        with patch(
            "imu_calibration.quickcal_station.window.time.monotonic_ns",
            return_value=40_000_000_000,
        ):
            harness._update_mag_auto_action(harness.state())
        with patch(
            "imu_calibration.quickcal_station.window.time.monotonic_ns",
            return_value=40_400_000_000,
        ):
            harness._update_mag_auto_action(harness.state())

        self.assertTrue(harness._mag_m01_completed_motion)
        self.assertEqual(harness.fail_message, "")
        self.assertTrue(any("完整回到关节中位" in item[0] for item in harness.logs))

    def test_m01_capture_end_commands_separate_joint_return_before_m02(self):
        harness = M01CaptureHarness(phase="wait_jog_stop")
        stopped_state = harness.state(final=False)
        harness.robot.latest_state = stopped_state

        with patch(
            "imu_calibration.quickcal_station.window.time.monotonic_ns",
            return_value=40_500_000_000,
        ):
            harness._update_mag_auto_action(stopped_state)

        self.assertEqual(harness.fail_message, "")
        self.assertEqual(harness._mag_phase, "return")
        self.assertEqual(harness._auto_action_step, "M01")
        self.assertEqual(
            harness.robot.moves,
            [
                (
                    "joints",
                    harness._mag_reference_joints,
                    NEUTRAL_RETURN_SPEED_PERCENT,
                )
            ],
        )

    def test_m04_requires_one_continuous_second_before_hold(self):
        harness = M04MotionHarness()

        harness.update_at(10.0)
        harness.update_at(10.9)
        self.assertEqual(harness._mag_phase, "static_wait_stop")

        harness.update_at(10.0 + MAG_M04_STOP_SETTLE_S)

        self.assertEqual(harness._mag_phase, "static_settle")
        self.assertIn("连续停稳 1.0 秒", harness.logs[-1][0])

    def test_m04_single_speed_spike_restarts_the_full_hold(self):
        harness = M04MotionHarness("static_settle")

        harness.update_at(12.0, linear=6.3, angular=0.93)

        self.assertEqual(harness._mag_phase, "static_recover")
        self.assertEqual(harness.fail_message, "")
        self.assertIn("线速度=6.30 mm/s", harness.logs[-1][0])

        harness.update_at(12.02)
        harness.update_at(12.02 + MAG_M04_STOP_SETTLE_S)

        self.assertEqual(harness._mag_phase, "static_settle")
        restarted_at = harness._mag_phase_started_ns
        harness.update_at(17.01)
        self.assertEqual(harness.confirm_calls, 0)
        harness.update_at(restarted_at / 1_000_000_000.0 + 5.0)
        self.assertEqual(harness.confirm_calls, 1)

    def test_m04_sustained_motion_still_fails(self):
        harness = M04MotionHarness("static_settle")

        harness.update_at(12.0, linear=3.0)
        harness.update_at(12.0 + MAG_M04_MOTION_GRACE_S, linear=3.0)

        self.assertIn("持续机械臂运动", harness.fail_message)
        self.assertIn("线速度=3.00 mm/s", harness.fail_message)

    def test_m04_alarm_mode_fails_immediately(self):
        harness = M04MotionHarness("static_settle")

        harness.update_at(12.0, mode=9)

        self.assertIn("异常模式：mode=9", harness.fail_message)

    def test_magnetic_limits_follow_each_work_order_axis(self):
        harness = MagneticMotionHarness()
        m01_state = lambda pose, j6=0.0: SimpleNamespace(
            pose=pose,
            joints=(0.0, 0.0, 0.0, 0.0, 0.0, j6),
        )

        self.assertEqual(
            harness._mag_limit_error(
                "M01", m01_state((0.0, 0.0, 0.0, 0.0, 0.0, 75.0), 75.0)
            ),
            "",
        )
        self.assertIn(
            "J6 往复",
            harness._mag_limit_error(
                "M01", m01_state((0.0, 0.0, 0.0, 0.0, 0.0, 81.0), 81.0)
            ),
        )
        self.assertEqual(
            harness._mag_limit_error(
                "M01", m01_state((0.0, 0.0, 0.0, 0.0, 75.0, 0.0))
            ),
            "",
        )
        self.assertIn(
            "Tool Ry",
            harness._mag_limit_error(
                "M01", m01_state((0.0, 0.0, 0.0, 0.0, 81.0, 0.0))
            ),
        )
        self.assertIn(
            "TCP XYZ",
            harness._mag_limit_error(
                "M01",
                m01_state(
                    (MAG_M01_XYZ_TOLERANCE_MM + 0.1, 0.0, 0.0, 0.0, 0.0, 0.0)
                ),
            ),
        )
        # M01 combines fixed-XYZ Tool Ry pitch and the J6-direction roll.
        self.assertEqual(
            harness._mag_limit_error(
                "M01", m01_state((0.0, 0.0, 0.0, 0.0, 75.0, 53.03), 53.03)
            ),
            "",
        )
        self.assertEqual(
            harness._mag_limit_error(
                "M02", SimpleNamespace(pose=(0.0, 0.0, 0.0, 45.0, 0.0, 0.0))
            ),
            "",
        )
        # M02/M03 no longer impose an application-side Rx angle limit.  Robot
        # alarm, collision and controller workspace protections remain active.
        self.assertEqual(
            harness._mag_limit_error(
                "M03", SimpleNamespace(pose=(0.0, 0.0, 0.0, -120.0, 0.0, 0.0))
            ),
            "",
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
            "S02": [("neutral", "S02"), ("confirm", "S02")],
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

    def test_full_auto_does_not_reenter_m01_while_solver_pumps_events(self):
        harness = FullAutoHarness("M01")
        harness._mag_m01_solving = True

        harness._try_start_full_auto_step()

        self.assertEqual(harness.calls, [])

    def test_full_auto_waits_for_magnetic_interstep_feedback_delay(self):
        harness = FullAutoHarness("M03")
        harness._full_auto_next_start_not_before_ns = 10_500_000_000

        with patch(
            "imu_calibration.quickcal_station.window.time.monotonic_ns",
            return_value=10_250_000_000,
        ):
            harness._try_start_full_auto_step()
        self.assertEqual(harness.calls, [])

        with patch(
            "imu_calibration.quickcal_station.window.time.monotonic_ns",
            return_value=10_500_000_000,
        ):
            harness._try_start_full_auto_step()
        self.assertEqual(harness.calls, [("mag", "M03")])


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
            version=4,
            status=11,
            imu_count=11,
            calibrated_count=0,
            flash_sequence=4,
            mean_rms_mdeg=0,
            gyro_quality=gyro_quality,
            accel_quality=accel_quality,
            gyro_all_ok=False,
            mag_all_ok=False,
            mag_quality=SimpleNamespace(
                reject_reasons=("MMC5983MA X 轴覆盖不足",),
                slots=(SimpleNamespace(
                    sample_count=20,
                    span_x=100,
                    span_y=500,
                    span_z=500,
                    offset_x=1,
                    offset_y=2,
                    offset_z=3,
                    scale_x1000=1000,
                    scale_y1000=1000,
                    scale_z1000=1000,
                ),),
            ),
            factory_pass=False,
        )

    def test_failed_report_summary_counts_each_quality_row(self):
        text = ReportSummaryHarness._format_report_summary(self.failed_report())

        self.assertIn("未通过：质量不足（status=11）", text)
        self.assertIn("Gyro=3/11（报告头 nCal=0）", text)
        self.assertIn("Accel=11/11", text)
        self.assertIn("Mag=失败", text)
        self.assertIn("本次 Flash 未写入｜当前 calSeq=4", text)
        self.assertIn("平均 RMS=--", text)


if __name__ == "__main__":
    unittest.main()
