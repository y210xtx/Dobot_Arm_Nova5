from pathlib import Path
from types import SimpleNamespace
import tempfile
import time
import unittest

try:
    from PySide6.QtCore import QCoreApplication

    from imu_calibration.quickcal_station.coordinator import QuickCalCoordinator, RunState
    from imu_calibration.quickcal_station.protocol import (
        ALL_IMU_MASK,
        AckFrame,
        CMD_MCAL_ABORT,
        CMD_MCAL_BEGIN,
        CMD_MCAL_COMMIT,
        CMD_MCAL_STAGE,
        ImuSample,
        RawImuFrame,
        RegisterImuSample,
        RegisterRawImuFrame,
        VersionFrame,
    )
    from imu_calibration.quickcal_station.robot_device import RobotState
except ModuleNotFoundError as exc:
    if exc.name != "PySide6":
        raise
    QCoreApplication = None
    QuickCalCoordinator = RunState = RobotState = None
    AckFrame = None
    CMD_MCAL_ABORT = CMD_MCAL_BEGIN = CMD_MCAL_COMMIT = CMD_MCAL_STAGE = None
    ALL_IMU_MASK = ImuSample = RawImuFrame = None
    RegisterImuSample = RegisterRawImuFrame = VersionFrame = None
from imu_calibration.quickcal_station.workflow import YawLimits


class FakeSignal:
    def __init__(self):
        self.callbacks = []

    def connect(self, callback):
        self.callbacks.append(callback)


class FakeGlove:
    def __init__(self):
        self.is_open = True
        self.commands = []
        for name in (
            "raw_imu_received",
            "register_raw_imu_received",
            "raw_mag_received",
            "factory_mag_pair_received",
            "version_received",
            "ack_received",
            "mcal_report_received",
            "error_occurred",
        ):
            setattr(self, name, FakeSignal())

    def send_command(self, command, argument=0, payload=b""):
        self.commands.append((command, argument, payload))
        return True


class FakeRobot:
    def __init__(self):
        self.connected = True
        self.stop_count = 0
        self.state_received = FakeSignal()
        self.error_occurred = FakeSignal()

    def stop(self):
        self.stop_count += 1
        return True


@unittest.skipIf(QCoreApplication is None, "PySide6 is not installed in this Python environment")
class CoordinatorCompatibilityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QCoreApplication.instance() or QCoreApplication([])

    @staticmethod
    def _prime_raw_streams(coordinator, stage_id=0, capture_mask=0):
        engineering = ImuSample(0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        registers = RegisterImuSample(0, 0, 0, 0, 0, 16384)
        coordinator.on_raw_imu(
            RawImuFrame(1, 1, ALL_IMU_MASK, stage_id, capture_mask, (engineering,) * 11)
        )
        coordinator.on_register_raw_imu(
            RegisterRawImuFrame(
                1, 1, ALL_IMU_MASK, 0, 0, stage_id, capture_mask, (registers,) * 11
            )
        )

    def test_type9_and_type11_are_mandatory_r024_inputs(self):
        glove = FakeGlove()
        robot = FakeRobot()
        coordinator = QuickCalCoordinator(glove, robot)
        coordinator.version = VersionFrame(
            payload_version=1,
            revision=24,
            revision_tag="r024-fac-magq",
            build_date="2026-08-27",
            build_time="11:00:00",
            imu_model="LSM6DSV16X",
            hand_side="right",
            features=0xF0,
            payload=bytes(48),
        )
        coordinator.latest_robot_state = RobotState(
            received_monotonic_ns=time.monotonic_ns(),
            controller_timestamp=1,
            mode=5,
            speed_scaling=1.0,
            joints=(0.0,) * 6,
            pose=(0.0,) * 6,
            tcp_speed=(0.0,) * 6,
            user=0,
            tool=0,
            digital_inputs=0,
            digital_outputs=0,
        )
        with tempfile.TemporaryDirectory() as directory:
            coordinator.configure(
                "SN001", "QC-01", "tester", Path(directory), YawLimits(), True,
                neutral_pose=(0.0,) * 6,
            )
            self.assertTrue(any("type=9" in error for error in coordinator.preflight_errors()))
            self._prime_raw_streams(coordinator)
            self.assertEqual(coordinator.preflight_errors(), [])
            self.assertTrue(coordinator.start_session())
            self.assertEqual(coordinator.state, RunState.WAIT_BEGIN_ACK)
            self.assertEqual(glove.commands[0][0], CMD_MCAL_BEGIN)
            coordinator.abort("test cleanup")
        coordinator.tick_timer.stop()

    def test_failed_stage_aborts_session_and_disallows_local_retry(self):
        glove = FakeGlove()
        robot = FakeRobot()
        coordinator = QuickCalCoordinator(glove, robot)
        coordinator.state = RunState.READY
        coordinator.current_index = 1

        coordinator._fail_current_step("synthetic stage failure")

        self.assertEqual(coordinator.state, RunState.ABORTED)
        self.assertEqual(robot.stop_count, 1)
        self.assertTrue(any(command[0] == CMD_MCAL_ABORT for command in glove.commands))
        self.assertEqual(coordinator.step_status[1], "失败")
        coordinator.tick_timer.stop()

    def test_capture_audit_does_not_treat_pre_open_raw_frame_as_new_stage(self):
        glove = FakeGlove()
        robot = FakeRobot()
        coordinator = QuickCalCoordinator(glove, robot)
        coordinator.current_index = next(
            index for index, step in enumerate(coordinator.steps)
            if step.step_id == "M01"
        )
        coordinator.state = RunState.CAPTURING
        self._prime_raw_streams(coordinator, stage_id=0, capture_mask=0)
        now = time.monotonic_ns()
        coordinator.capture_started_ns = now - coordinator.RAW_STAGE_SYNC_NS - 10_000_000
        coordinator.raw_imu_ns = coordinator.capture_started_ns - 1
        coordinator.register_imu_ns = coordinator.capture_started_ns - 1

        self.assertIn(
            "阶段开启后尚未收到新的 type=9",
            coordinator._raw_capture_health_error(now),
        )

        step = coordinator.current_step
        self._prime_raw_streams(
            coordinator, stage_id=step.stage_code, capture_mask=step.capture_mask
        )
        self.assertEqual(coordinator._raw_capture_health_error(time.monotonic_ns()), "")
        coordinator.tick_timer.stop()

    def test_capture_audit_waits_for_matching_stage_frames_before_timeout(self):
        coordinator = QuickCalCoordinator(FakeGlove(), FakeRobot())
        coordinator.current_index = next(
            index for index, step in enumerate(coordinator.steps)
            if step.step_id == "M01"
        )
        coordinator.state = RunState.CAPTURING
        now = time.monotonic_ns()
        coordinator.capture_started_ns = now - 500_000_000
        self._prime_raw_streams(coordinator, stage_id=0, capture_mask=0)

        self.assertEqual(coordinator._raw_capture_health_error(now), "")

        after_timeout = (
            coordinator.capture_started_ns + coordinator.RAW_STAGE_SYNC_NS + 1
        )
        self.assertIn(
            "板端阶段审计不一致",
            coordinator._raw_capture_health_error(after_timeout),
        )
        coordinator.tick_timer.stop()

    def test_p1_starts_automatically_after_two_seconds_of_robot_stillness(self):
        glove = FakeGlove()
        robot = FakeRobot()
        coordinator = QuickCalCoordinator(glove, robot)
        coordinator.state = RunState.READY
        coordinator.current_index = 1
        coordinator.latest_robot_state = RobotState(
            received_monotonic_ns=time.monotonic_ns(),
            controller_timestamp=1,
            mode=5,
            speed_scaling=1.0,
            joints=(0.0,) * 6,
            pose=(0.0,) * 6,
            tcp_speed=(0.0,) * 6,
            user=0,
            tool=0,
            digital_inputs=0,
            digital_outputs=0,
        )
        coordinator.condition_stable_since_ns = time.monotonic_ns() - 2_100_000_000
        self._prime_raw_streams(coordinator)

        coordinator._tick()

        self.assertEqual(coordinator.state, RunState.WAIT_STAGE_OPEN)
        self.assertEqual(glove.commands[-1], (CMD_MCAL_STAGE, 0x01, b"\x01"))
        coordinator.tick_timer.stop()

    def test_p1_capture_health_check_does_not_restart_pre_capture_settle_timer(self):
        glove = FakeGlove()
        robot = FakeRobot()
        coordinator = QuickCalCoordinator(glove, robot)
        coordinator.current_index = 1
        coordinator.latest_robot_state = RobotState(
            received_monotonic_ns=time.monotonic_ns(),
            controller_timestamp=1,
            mode=5,
            speed_scaling=1.0,
            joints=(0.0,) * 6,
            pose=(0.0,) * 6,
            tcp_speed=(0.0,) * 6,
            user=0,
            tool=0,
            digital_inputs=0,
            digital_outputs=0,
        )

        self.assertIn("继续保持", coordinator._check_motion_condition(coordinator.current_step))
        self.assertEqual(
            coordinator._check_motion_condition(
                coordinator.current_step, require_settle=False
            ),
            "",
        )
        coordinator.tick_timer.stop()

    def test_p1_type9_gate_detects_rotation_and_acceleration_vibration(self):
        glove = FakeGlove()
        robot = FakeRobot()
        coordinator = QuickCalCoordinator(glove, robot)
        still = ImuSample(0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        below_new_limit = ImuSample(0.099, 0.0, 0.0, 0.0, 0.0, 1.0)
        rotating = ImuSample(0.101, 0.0, 0.0, 0.0, 0.0, 1.0)
        vibrating = ImuSample(0.0, 0.0, 0.0, 0.1, 0.0, 0.995)

        self.assertEqual(
            coordinator._evaluate_p1_imu_frame(
                RawImuFrame(1, 1, ALL_IMU_MASK, 0, 0, (still,) * 11)
            ),
            "",
        )
        self.assertEqual(
            coordinator._evaluate_p1_imu_frame(
                RawImuFrame(2, 1, ALL_IMU_MASK, 0, 0, (below_new_limit,) + (still,) * 10)
            ),
            "",
        )
        rotating_frame = RawImuFrame(3, 1, ALL_IMU_MASK, 0, 0, (rotating,) + (still,) * 10)
        self.assertEqual(coordinator._evaluate_p1_imu_frame(rotating_frame), "")
        self.assertEqual(coordinator._evaluate_p1_imu_frame(rotating_frame), "")
        self.assertIn("检测到持续转动", coordinator._evaluate_p1_imu_frame(rotating_frame))
        coordinator.p1_previous_accel = ((0.0, 0.0, 1.0),) * 11
        vibrating_frame = RawImuFrame(4, 1, ALL_IMU_MASK, 0, 0, (vibrating,) + (still,) * 10)
        still_frame = RawImuFrame(5, 1, ALL_IMU_MASK, 0, 0, (still,) * 11)
        self.assertEqual(coordinator._evaluate_p1_imu_frame(vibrating_frame), "")
        self.assertEqual(coordinator._evaluate_p1_imu_frame(still_frame), "")
        self.assertIn("检测到持续振动", coordinator._evaluate_p1_imu_frame(vibrating_frame))
        coordinator.tick_timer.stop()

    def test_p1_ignores_isolated_zero_accel_gyro_spikes(self):
        glove = FakeGlove()
        robot = FakeRobot()
        coordinator = QuickCalCoordinator(glove, robot)
        still = ImuSample(0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        corrupt_0274 = ImuSample(0.2736676335, 0.0, 0.0, 0.0, 0.0, 0.0)
        corrupt_9697 = ImuSample(-9.6968755722, 0.0, 0.0, 0.0, 0.0, 0.0)
        still_frame = RawImuFrame(1, 1, ALL_IMU_MASK, 0, 0, (still,) * 11)
        spike_0274 = RawImuFrame(2, 1, ALL_IMU_MASK, 0, 0, (still,) * 2 + (corrupt_0274,) + (still,) * 8)
        spike_9697 = RawImuFrame(3, 1, ALL_IMU_MASK, 0, 0, (still,) * 2 + (corrupt_9697,) + (still,) * 8)

        self.assertEqual(coordinator._evaluate_p1_imu_frame(still_frame), "")
        self.assertEqual(coordinator._evaluate_p1_imu_frame(spike_0274), "")
        self.assertTrue(coordinator.p1_imu_transient_issue)
        self.assertEqual(coordinator._evaluate_p1_imu_frame(still_frame), "")
        self.assertFalse(coordinator.p1_imu_transient_issue)
        self.assertEqual(coordinator._evaluate_p1_imu_frame(spike_9697), "")
        self.assertEqual(coordinator.p1_imu_rejected_frames, 2)
        self.assertEqual(coordinator._evaluate_p1_imu_frame(still_frame), "")
        coordinator.tick_timer.stop()

    def test_p1_rejects_persistent_zero_accel_corruption(self):
        glove = FakeGlove()
        robot = FakeRobot()
        coordinator = QuickCalCoordinator(glove, robot)
        still = ImuSample(0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        corrupt = ImuSample(9.697, 0.0, 0.0, 0.0, 0.0, 0.0)
        frame = RawImuFrame(1, 1, ALL_IMU_MASK, 0, 0, (still,) * 2 + (corrupt,) + (still,) * 8)

        self.assertEqual(coordinator._evaluate_p1_imu_frame(frame), "")
        self.assertEqual(coordinator._evaluate_p1_imu_frame(frame), "")
        self.assertIn("THUMB_2 加速度接近 0 g", coordinator._evaluate_p1_imu_frame(frame))
        coordinator.tick_timer.stop()

    def test_six_face_reference_poses_align_with_base_gravity(self):
        reference_poses = {
            "A01": (0.0, 90.0, 0.0),
            "A02": (0.0, -90.0, 0.0),
            "A03": (-90.0, 0.0, 0.0),
            "A04": (90.0, 0.0, 0.0),
            "A05": (180.0, 0.0, 0.0),
            "A06": (0.0, 0.0, 0.0),
        }
        for step_id, rotation in reference_poses.items():
            with self.subTest(step_id=step_id):
                result = QuickCalCoordinator._accel_face_alignment(
                    step_id, (0.0, 0.0, 0.0, *rotation)
                )
                self.assertIsNotNone(result)
                self.assertAlmostEqual(result[0], 0.0, places=6)

    def test_six_face_gate_rejects_more_than_five_degrees(self):
        glove = FakeGlove()
        robot = FakeRobot()
        coordinator = QuickCalCoordinator(glove, robot)
        coordinator.current_index = next(
            index for index, step in enumerate(coordinator.steps) if step.step_id == "A06"
        )
        coordinator.latest_robot_state = RobotState(
            received_monotonic_ns=time.monotonic_ns(),
            controller_timestamp=1,
            mode=5,
            speed_scaling=1.0,
            joints=(0.0,) * 6,
            pose=(0.0, 0.0, 0.0, 6.0, 0.0, 0.0),
            tcp_speed=(0.0,) * 6,
            user=0,
            tool=0,
            digital_inputs=0,
            digital_outputs=0,
        )

        error = coordinator._check_motion_condition(coordinator.current_step)

        self.assertIn("姿态偏差 6.00°", error)
        self.assertIn("超过允许值 5.0°", error)
        coordinator.tick_timer.stop()

    def test_six_face_warning_band_remains_eligible(self):
        result = QuickCalCoordinator._accel_face_alignment(
            "A06", (0.0, 0.0, 0.0, 3.0, 0.0, 0.0)
        )

        self.assertIsNotNone(result)
        self.assertGreater(result[0], QuickCalCoordinator.ACCEL_FACE_WARNING_DEG)
        self.assertLess(result[0], QuickCalCoordinator.ACCEL_FACE_MAX_DEG)

    def test_gyro_direction_gates_follow_configured_r024_axis_mapping(self):
        glove = FakeGlove()
        robot = FakeRobot()
        coordinator = QuickCalCoordinator(glove, robot)
        expected = {
            "G01": (15.0, 0.0, 0.0),
            "G02": (-15.0, 0.0, 0.0),
            "G03": (0.0, 15.0, 0.0),
            "G04": (0.0, -15.0, 0.0),
            "G05": (0.0, 0.0, 15.0),
            "G06": (0.0, 0.0, -15.0),
        }
        steps = {step.step_id: step for step in coordinator.steps}
        for step_id, angular_speed in expected.items():
            state = RobotState(
                received_monotonic_ns=time.monotonic_ns(),
                controller_timestamp=1,
                mode=7,
                speed_scaling=1.0,
                joints=(0.0,) * 6,
                pose=(0.0,) * 6,
                tcp_speed=(0.0, 0.0, 0.0, *angular_speed),
                user=0,
                tool=0,
                digital_inputs=0,
                digital_outputs=0,
            )
            with self.subTest(step_id=step_id):
                self.assertEqual(coordinator._gyro_motion_error(steps[step_id], state), "")
        coordinator.tick_timer.stop()

    def test_all_gyro_jog_modes_can_open_and_continue_dynamic_capture(self):
        rates = {
            "G01": (15.0, 0.0, 0.0),
            "G02": (-15.0, 0.0, 0.0),
            "G03": (0.0, 15.0, 0.0),
            "G04": (0.0, -15.0, 0.0),
            "G05": (0.0, 0.0, 15.0),
            "G06": (0.0, 0.0, -15.0),
        }
        for step_id, angular_speed in rates.items():
            with self.subTest(step_id=step_id):
                glove = FakeGlove()
                robot = FakeRobot()
                coordinator = QuickCalCoordinator(glove, robot)
                coordinator.current_index = next(
                    index
                    for index, step in enumerate(coordinator.steps)
                    if step.step_id == step_id
                )
                coordinator.state = RunState.READY
                coordinator.neutral_pose = (0.0,) * 6
                coordinator.latest_robot_state = RobotState(
                    received_monotonic_ns=time.monotonic_ns(),
                    controller_timestamp=1,
                    mode=8,
                    speed_scaling=15.0,
                    joints=(0.0,) * 6,
                    pose=(0.0,) * 6,
                    tcp_speed=(0.0, 0.0, 0.0, *angular_speed),
                    user=0,
                    tool=1,
                    digital_inputs=0,
                    digital_outputs=0,
                )
                coordinator.condition_stable_since_ns = (
                    time.monotonic_ns() - 1_100_000_000
                )
                self._prime_raw_streams(coordinator)

                self.assertTrue(coordinator.confirm_current_action())
                self.assertEqual(coordinator.state, RunState.WAIT_STAGE_OPEN)
                step = coordinator.current_step
                self.assertEqual(
                    glove.commands[-1],
                    (CMD_MCAL_STAGE, step.stage_code, bytes((step.capture_mask,))),
                )
                coordinator.on_ack(
                    AckFrame(
                        CMD_MCAL_STAGE, 0, step.stage_code, step.capture_mask, 1
                    )
                )
                self.assertEqual(coordinator.state, RunState.CAPTURING)
                self.assertEqual(
                    coordinator._live_capture_fault(time.monotonic_ns()), ""
                )
                coordinator.abort("test cleanup")
                coordinator.tick_timer.stop()

    def test_collision_mode_is_rejected_and_aborts_running_session(self):
        glove = FakeGlove()
        robot = FakeRobot()
        coordinator = QuickCalCoordinator(glove, robot)
        coordinator.current_index = next(
            index for index, step in enumerate(coordinator.steps) if step.step_id == "G01"
        )
        coordinator.state = RunState.READY
        collision = RobotState(
            received_monotonic_ns=time.monotonic_ns(),
            controller_timestamp=1,
            mode=11,
            speed_scaling=0.0,
            joints=(0.0,) * 6,
            pose=(0.0,) * 6,
            tcp_speed=(0.0,) * 6,
            user=0,
            tool=1,
            digital_inputs=0,
            digital_outputs=0,
        )
        coordinator.latest_robot_state = collision

        self.assertIn(
            "碰撞检测已触发",
            coordinator._check_motion_condition(coordinator.current_step),
        )
        coordinator.on_robot_state(collision)
        self.assertEqual(coordinator.state, RunState.ABORTED)
        self.assertEqual(robot.stop_count, 1)
        self.assertEqual(glove.commands[-1][0], CMD_MCAL_ABORT)
        coordinator.tick_timer.stop()

    def test_g06_advances_to_required_magnetic_stages(self):
        glove = FakeGlove()
        robot = FakeRobot()
        coordinator = QuickCalCoordinator(glove, robot)
        g06_index = next(
            index
            for index, step in enumerate(coordinator.steps)
            if step.step_id == "G06"
        )
        coordinator.current_index = g06_index
        for index in range(1, g06_index + 1):
            coordinator.step_status[index] = "完成"

        coordinator._advance()

        self.assertFalse(coordinator.SKIP_MAGNETIC_STAGES)
        self.assertEqual(coordinator.current_step.step_id, "M01")
        self.assertEqual(
            [step.step_id for step in coordinator.steps if step.step_id.startswith("M")],
            ["M01", "M02", "M03", "M04"],
        )
        coordinator.tick_timer.stop()

    def test_r024_mag_workflow_contains_exactly_seventeen_formal_stages(self):
        glove = FakeGlove()
        robot = FakeRobot()
        coordinator = QuickCalCoordinator(glove, robot)
        formal = [step.stage_code for step in coordinator.steps if step.stage_code is not None]
        self.assertEqual(formal, [0x01, *range(0x10, 0x16), *range(0x20, 0x26), *range(0x30, 0x34)])
        coordinator.tick_timer.stop()

    def test_failed_commit_ack_still_consumes_type7_diagnostics(self):
        glove = FakeGlove()
        robot = FakeRobot()
        coordinator = QuickCalCoordinator(glove, robot)
        coordinator.current_index = next(
            index for index, step in enumerate(coordinator.steps) if step.step_id == "S01"
        )
        coordinator.state = RunState.WAIT_COMMIT_ACK
        coordinator.on_ack(AckFrame(CMD_MCAL_COMMIT, 11, 7, 11, 22))
        self.assertEqual(coordinator.state, RunState.WAIT_REPORT)

        report = SimpleNamespace(
            factory_pass=False,
            version=4,
            status=11,
            gyro_quality=(
                SimpleNamespace(
                    ok=False,
                    reject_flags=0x01,
                    reject_reasons=("有效窗口不足",),
                    window_count=23,
                    rms_mdeg=6100,
                    max_off_axis=12,
                ),
            ),
            accel_quality=(),
            mag_all_ok=False,
            mag_quality=SimpleNamespace(
                reject_reasons=("MMC5983MA X 轴覆盖不足",),
                slots=(SimpleNamespace(sample_count=20, span_x=100, span_y=500, span_z=500),),
            ),
            payload=b"",
        )
        coordinator.on_mcal_report(report)

        self.assertEqual(coordinator.state, RunState.ABORTED)
        self.assertEqual(glove.commands[-1][0], CMD_MCAL_ABORT)
        self.assertIn("有效窗口不足", coordinator.step_detail[coordinator.current_index])
        coordinator.tick_timer.stop()

    def test_all_gyro_steps_use_fixed_fifteen_degree_rate(self):
        glove = FakeGlove()
        robot = FakeRobot()
        coordinator = QuickCalCoordinator(glove, robot)
        coordinator.limits = YawLimits()
        steps = {step.step_id: step for step in coordinator.steps}
        state = RobotState(
            received_monotonic_ns=time.monotonic_ns(),
            controller_timestamp=1,
            mode=7,
            speed_scaling=1.0,
            joints=(0.0,) * 6,
            pose=(0.0,) * 6,
            tcp_speed=(0.0, 0.0, 0.0, 15.0, 0.0, 0.0),
            user=0,
            tool=0,
            digital_inputs=0,
            digital_outputs=0,
        )

        self.assertEqual(coordinator._gyro_motion_error(steps["G01"], state), "")
        slow_x_state = RobotState(
            received_monotonic_ns=time.monotonic_ns(),
            controller_timestamp=1,
            mode=7,
            speed_scaling=1.0,
            joints=(0.0,) * 6,
            pose=(0.0,) * 6,
            tcp_speed=(0.0, 0.0, 0.0, 10.0, 0.0, 0.0),
            user=0,
            tool=0,
            digital_inputs=0,
            digital_outputs=0,
        )
        self.assertIn(
            "+15.0°/s",
            coordinator._gyro_motion_error(steps["G01"], slow_x_state),
        )
        z_state = RobotState(
            received_monotonic_ns=time.monotonic_ns(),
            controller_timestamp=1,
            mode=7,
            speed_scaling=1.0,
            joints=(0.0,) * 6,
            pose=(0.0,) * 6,
            tcp_speed=(0.0, 0.0, 0.0, 0.0, 0.0, 15.0),
            user=0,
            tool=0,
            digital_inputs=0,
            digital_outputs=0,
        )
        self.assertEqual(coordinator._gyro_motion_error(steps["G05"], z_state), "")
        coordinator.tick_timer.stop()

    def test_limited_gyro_angle_uses_runtime_negative_y_reference(self):
        coordinator = QuickCalCoordinator(FakeGlove(), FakeRobot())
        coordinator.neutral_pose = (0.0,) * 6
        runtime_reference = (0.0, 0.0, 0.0, 30.0, 0.0, 0.0)
        coordinator.gyro_limited_reference_pose = runtime_reference
        coordinator.current_index = next(
            index
            for index, step in enumerate(coordinator.steps)
            if step.step_id == "G01"
        )

        self.assertAlmostEqual(
            coordinator._relative_tool_axis_deg(runtime_reference, "Rx"), 0.0
        )
        coordinator.gyro_limited_reference_pose = None
        self.assertAlmostEqual(
            coordinator._relative_tool_axis_deg(runtime_reference, "Rx"), 30.0
        )
        coordinator.tick_timer.stop()

    def test_magnetic_coverage_matches_each_work_order_action(self):
        glove = FakeGlove()
        robot = FakeRobot()
        coordinator = QuickCalCoordinator(glove, robot)
        coordinator.neutral_pose = (0.0,) * 6
        coordinator._reset_motion_coverage()
        for key in ("z_positive", "z_negative", "y_positive", "y_negative"):
            coordinator.motion_coverage[key] = True
        self.assertEqual(coordinator._motion_coverage_error("M01"), "")
        coordinator.motion_coverage["y_negative"] = False
        self.assertIn("J6", coordinator._motion_coverage_error("M01"))
        self.assertIn("J2/J3/J4", coordinator._motion_coverage_error("M01"))

        coordinator._reset_motion_coverage()
        coordinator.motion_coverage["x_positive"] = True
        coordinator.motion_coverage["x_negative"] = True
        self.assertEqual(coordinator._motion_coverage_error("M02"), "")
        self.assertEqual(coordinator._motion_coverage_error("M03"), "")
        coordinator.motion_coverage["x_negative"] = False
        self.assertIn("Tool Rx", coordinator._motion_coverage_error("M02"))
        self.assertEqual(coordinator._motion_coverage_error("M04"), "")
        coordinator.tick_timer.stop()

    def test_m02_m03_allow_final_braking_after_both_rx_directions_are_covered(self):
        coordinator = QuickCalCoordinator(FakeGlove(), FakeRobot())
        coordinator.current_index = next(
            index
            for index, step in enumerate(coordinator.steps)
            if step.step_id == "M02"
        )
        coordinator._reset_motion_coverage()
        stopped = SimpleNamespace(
            pose=(0.0,) * 6,
            angular_speed=(0.0,) * 3,
        )

        self.assertIn("慢速往返", coordinator._mag_motion_error(stopped))
        coordinator.motion_coverage["x_positive"] = True
        self.assertIn("慢速往返", coordinator._mag_motion_error(stopped))
        coordinator.motion_coverage["x_negative"] = True
        self.assertEqual(coordinator._mag_motion_error(stopped), "")

        coordinator.current_index = next(
            index
            for index, step in enumerate(coordinator.steps)
            if step.step_id == "M03"
        )
        self.assertEqual(coordinator._mag_motion_error(stopped), "")
        coordinator.tick_timer.stop()


if __name__ == "__main__":
    unittest.main()
