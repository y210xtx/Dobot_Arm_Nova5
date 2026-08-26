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
        CMD_MCAL_STAGE,
        MCAL_CAPTURE_MAG,
        ImuSample,
        RawImuFrame,
    )
    from imu_calibration.quickcal_station.robot_device import RobotState
except ModuleNotFoundError as exc:
    if exc.name != "PySide6":
        raise
    QCoreApplication = None
    QuickCalCoordinator = RunState = RobotState = None
    AckFrame = None
    CMD_MCAL_ABORT = CMD_MCAL_BEGIN = CMD_MCAL_STAGE = MCAL_CAPTURE_MAG = None
    ALL_IMU_MASK = ImuSample = RawImuFrame = None
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

    def test_type9_and_type11_are_optional_diagnostics(self):
        glove = FakeGlove()
        robot = FakeRobot()
        coordinator = QuickCalCoordinator(glove, robot)
        coordinator.version = SimpleNamespace(
            payload_version=1,
            revision=6,
            revision_tag="r006-LSM6DSV16X",
            imu_model="LSM6DSV16X",
            hand_side="right",
            features=0x60,
            factory_intrinsic=True,
            accel_intrinsic=True,
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

        coordinator._tick()

        self.assertEqual(coordinator.state, RunState.WAIT_STAGE_OPEN)
        self.assertEqual(glove.commands[-1], (CMD_MCAL_STAGE, 0x01, b"\x01"))
        coordinator.tick_timer.stop()

    def test_p1_type9_gate_detects_rotation_and_acceleration_vibration(self):
        glove = FakeGlove()
        robot = FakeRobot()
        coordinator = QuickCalCoordinator(glove, robot)
        still = ImuSample(0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        rotating = ImuSample(0.06, 0.0, 0.0, 0.0, 0.0, 1.0)
        vibrating = ImuSample(0.0, 0.0, 0.0, 0.1, 0.0, 0.995)

        self.assertEqual(
            coordinator._evaluate_p1_imu_frame(
                RawImuFrame(1, 1, ALL_IMU_MASK, (still,) * 11)
            ),
            "",
        )
        self.assertIn(
            "检测到转动",
            coordinator._evaluate_p1_imu_frame(
                RawImuFrame(2, 1, ALL_IMU_MASK, (rotating,) + (still,) * 10)
            ),
        )
        coordinator.p1_previous_accel = ((0.0, 0.0, 1.0),) * 11
        self.assertIn(
            "检测到振动",
            coordinator._evaluate_p1_imu_frame(
                RawImuFrame(3, 1, ALL_IMU_MASK, (vibrating,) + (still,) * 10)
            ),
        )
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

    def test_gyro_direction_gates_follow_excel_work_order(self):
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

    def test_g01_jog_mode_can_open_and_continue_dynamic_capture(self):
        glove = FakeGlove()
        robot = FakeRobot()
        coordinator = QuickCalCoordinator(glove, robot)
        coordinator.current_index = next(
            index for index, step in enumerate(coordinator.steps) if step.step_id == "G01"
        )
        coordinator.state = RunState.READY
        coordinator.latest_robot_state = RobotState(
            received_monotonic_ns=time.monotonic_ns(),
            controller_timestamp=1,
            mode=8,
            speed_scaling=15.0,
            joints=(0.0,) * 6,
            pose=(0.0,) * 6,
            tcp_speed=(0.0, 0.0, 0.0, 15.0, 0.0, 0.0),
            user=0,
            tool=1,
            digital_inputs=0,
            digital_outputs=0,
        )
        coordinator.condition_stable_since_ns = time.monotonic_ns() - 1_100_000_000

        self.assertTrue(coordinator.confirm_current_action())
        self.assertEqual(coordinator.state, RunState.WAIT_STAGE_OPEN)
        step = coordinator.current_step
        self.assertEqual(
            glove.commands[-1],
            (CMD_MCAL_STAGE, step.stage_code, bytes((step.capture_mask,))),
        )
        coordinator.on_ack(
            AckFrame(CMD_MCAL_STAGE, 0, step.stage_code, step.capture_mask, 1)
        )
        self.assertEqual(coordinator.state, RunState.CAPTURING)
        self.assertEqual(coordinator._live_capture_fault(time.monotonic_ns()), "")
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

    def test_m04_is_a_formal_fifteen_second_magnetic_stage(self):
        glove = FakeGlove()
        robot = FakeRobot()
        coordinator = QuickCalCoordinator(glove, robot)
        coordinator.neutral_pose = (0.0,) * 6
        coordinator.environment_confirmed = True
        coordinator.current_index = next(
            index for index, step in enumerate(coordinator.steps) if step.step_id == "M04"
        )
        coordinator.latest_robot_state = RobotState(
            received_monotonic_ns=time.monotonic_ns(),
            controller_timestamp=1,
            mode=7,
            speed_scaling=1.0,
            joints=(0.0,) * 6,
            pose=(0.0,) * 6,
            tcp_speed=(0.0, 0.0, 0.0, 2.0, 2.0, 0.0),
            user=0,
            tool=0,
            digital_inputs=0,
            digital_outputs=0,
        )
        coordinator.condition_stable_since_ns = time.monotonic_ns() - 400_000_000

        self.assertEqual(coordinator._check_motion_condition(coordinator.current_step), "")
        coordinator.state = RunState.READY
        self.assertTrue(coordinator.confirm_current_action())
        self.assertEqual(glove.commands[-1], (CMD_MCAL_STAGE, 0x33, bytes((MCAL_CAPTURE_MAG,))))
        coordinator.on_ack(AckFrame(CMD_MCAL_STAGE, 0, 0x33, MCAL_CAPTURE_MAG, 1))
        self.assertEqual(coordinator.state, RunState.CAPTURING)
        coordinator.motion_coverage["x_positive"] = True
        coordinator.motion_coverage["y_negative"] = True
        coordinator._request_stage_close()
        coordinator.on_ack(AckFrame(CMD_MCAL_STAGE, 0, 0x33, 11, 2))
        self.assertEqual(coordinator.current_step.step_id, "S01")
        self.assertEqual(coordinator.valid_mag_pairs_this_step, 0)
        coordinator.tick_timer.stop()

    def test_roll_pitch_rate_does_not_follow_yaw_setting(self):
        glove = FakeGlove()
        robot = FakeRobot()
        coordinator = QuickCalCoordinator(glove, robot)
        coordinator.limits = YawLimits(rate_deg_s=10.0)
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
        self.assertIn("+10.0°/s", coordinator._gyro_motion_error(steps["G05"], state))
        coordinator.tick_timer.stop()

    def test_magnetic_coverage_requires_at_least_two_tool_axes(self):
        glove = FakeGlove()
        robot = FakeRobot()
        coordinator = QuickCalCoordinator(glove, robot)
        coordinator.neutral_pose = (0.0,) * 6
        coordinator.current_index = next(
            index for index, step in enumerate(coordinator.steps) if step.step_id == "M02"
        )
        coordinator._reset_motion_coverage()

        for angular_speed in ((2.0, 0.0, 0.0), (0.0, -2.0, 0.0)):
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
            coordinator.latest_robot_state = state
            coordinator._update_motion_coverage(state)

        self.assertEqual(coordinator._motion_coverage_error("M02"), "")
        coordinator._reset_motion_coverage()
        coordinator.motion_coverage["x_positive"] = True
        self.assertIn("至少需要两个旋转轴", coordinator._motion_coverage_error("M02"))
        coordinator.tick_timer.stop()


if __name__ == "__main__":
    unittest.main()
