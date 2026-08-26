import struct
import unittest

from imu_calibration.quickcal_station.protocol import (
    CMD_MCAL_BEGIN,
    EXPECTED_MCAL_PAYLOAD_LENGTH,
    MCAL_CAPTURE_MAG,
    TYPE_RAW_IMU,
    ProtocolParser,
    RawImuFrame,
    build_command,
    crc16_ccitt,
)
from imu_calibration.quickcal_station.workflow import (
    YawLimits,
    expected_capture_seconds,
    expected_total_seconds,
    steps_for_limits,
)


class ProtocolTests(unittest.TestCase):
    def test_crc_and_begin_command_match_cpp_protocol(self):
        body = bytes.fromhex("A5 5A 10 00 00 00")
        self.assertEqual(crc16_ccitt(body), 0xE402)
        self.assertEqual(build_command(CMD_MCAL_BEGIN), bytes.fromhex("A5 5A 10 00 00 00 02 E4"))

    def test_fragmented_raw_imu_frame(self):
        payload = bytearray((1, 11))
        payload.extend(struct.pack("<H", 0x07FF))
        payload.extend(b"\0" * 4)
        for index in range(11):
            payload.extend(struct.pack("<6f", index, 0.0, 0.0, 0.0, 0.0, 1.0))
        frame = b"\x55\xAA" + bytes((TYPE_RAW_IMU, 7)) + struct.pack("<I", len(payload)) + payload
        parser = ProtocolParser()
        self.assertEqual(parser.feed(frame[:9]), [])
        parsed = parser.feed(frame[9:])
        self.assertEqual(len(parsed), 1)
        self.assertEqual(parsed[0][0], TYPE_RAW_IMU)
        self.assertIsInstance(parsed[0][1], RawImuFrame)
        self.assertEqual(parsed[0][1].presence_mask, 0x07FF)
        self.assertEqual(len(parsed[0][1].samples), 11)

    def test_type8_preserves_capabilities_and_raw_payload(self):
        payload = bytearray(48)
        payload[0] = 0x08
        payload[1] = 1
        payload[41] = 0x60
        version = ProtocolParser()._parse_version(bytes(payload))
        self.assertTrue(version.factory_intrinsic)
        self.assertTrue(version.accel_intrinsic)
        self.assertEqual(version.payload, bytes(payload))
        with self.assertRaises(ValueError):
            ProtocolParser()._parse_version(bytes(payload[:-1]))

    @staticmethod
    def valid_mcal_payload() -> bytes:
        payload = bytearray(EXPECTED_MCAL_PAYLOAD_LENGTH)
        payload[0] = 0x11
        payload[1] = 2
        payload[2] = 11
        payload[3] = 11
        payload[6] = 0
        payload[7] = 0x03
        identity = struct.pack("<9f", 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
        for index in range(11):
            gyro_quality = 12 + index * 8
            payload[gyro_quality] = 1
            struct.pack_into("<H", payload, gyro_quality + 4, 6)
            payload[100 + index * 36 : 100 + (index + 1) * 36] = identity

            accel_quality = 496 + index * 14
            payload[accel_quality] = 1
            payload[650 + index * 36 : 650 + (index + 1) * 36] = identity
        return bytes(payload)

    def test_type7_v2_requires_full_report_and_matrices(self):
        report = ProtocolParser()._parse_mcal(7, self.valid_mcal_payload())
        self.assertTrue(report.format_valid)
        self.assertTrue(report.gyro_all_ok)
        self.assertTrue(report.accel_all_ok)
        self.assertTrue(report.factory_pass)
        self.assertEqual(len(report.gyro_matrices), 11)
        self.assertEqual(len(report.accel_matrices), 11)

    def test_type7_rejects_truncated_or_wrong_version_report(self):
        parser = ProtocolParser()
        with self.assertRaises(ValueError):
            parser._parse_mcal(1, self.valid_mcal_payload()[:650])
        wrong_version = bytearray(self.valid_mcal_payload())
        wrong_version[1] = 99
        with self.assertRaises(ValueError):
            parser._parse_mcal(1, bytes(wrong_version))

    def test_type7_requires_gyro_flag_and_all_six_segments(self):
        missing_flag = bytearray(self.valid_mcal_payload())
        missing_flag[7] = 0x02
        self.assertFalse(ProtocolParser()._parse_mcal(1, bytes(missing_flag)).gyro_all_ok)

        too_few_segments = bytearray(self.valid_mcal_payload())
        struct.pack_into("<H", too_few_segments, 12 + 4, 5)
        self.assertFalse(ProtocolParser()._parse_mcal(1, bytes(too_few_segments)).gyro_all_ok)


class WorkflowTests(unittest.TestCase):
    def test_excel_work_order_totals_and_stage_contract(self):
        limits = YawLimits()
        self.assertTrue(limits.valid)
        self.assertAlmostEqual(limits.negative_safe_deg, -40.0)
        self.assertAlmostEqual(limits.positive_safe_deg, 40.0)
        self.assertAlmostEqual(limits.capture_s, 16.0 / 3.0)
        steps = steps_for_limits(limits)
        self.assertEqual(len(steps), 20)
        self.assertAlmostEqual(expected_total_seconds(limits), 325.6666666667)
        self.assertAlmostEqual(expected_capture_seconds(limits), 158.6666666667)
        formal = [step for step in steps if step.stage_code is not None]
        self.assertEqual(len(formal), 17)
        self.assertEqual([step.stage_code for step in formal], [
            0x01,
            *range(0x10, 0x16),
            *range(0x20, 0x26),
            *range(0x30, 0x34),
        ])
        m04 = next(step for step in steps if step.step_id == "M04")
        self.assertTrue(m04.sample_enabled)
        self.assertEqual(m04.capture_mask, MCAL_CAPTURE_MAG)
        self.assertEqual(m04.capture_s, 15)
        self.assertEqual(m04.exit_s, 0)

    def test_excel_dynamic_stage_meanings_and_yaw_formula(self):
        limits = YawLimits()
        steps = {step.step_id: step for step in steps_for_limits(limits)}
        self.assertEqual(
            [steps[step_id].name for step_id in ("G01", "G02", "G03", "G04", "G05", "G06")],
            [
                "+X，15 deg/s",
                "-X，15 deg/s",
                "+Y，15 deg/s",
                "-Y，15 deg/s",
                "+Z（Yaw），15 deg/s",
                "-Z（Yaw），15 deg/s",
            ],
        )
        self.assertEqual(steps["G01"].capture_s, 10)
        self.assertEqual(steps["G04"].capture_s, 10)
        self.assertAlmostEqual(steps["G05"].capture_s, limits.capture_s)
        self.assertAlmostEqual(steps["G06"].capture_s, limits.capture_s)
        self.assertEqual(
            [steps[step_id].capture_s for step_id in ("M01", "M02", "M03", "M04")],
            [15, 15, 15, 15],
        )

    def test_yaw_limits_reject_non_finite_or_non_positive_minimum(self):
        self.assertFalse(YawLimits(negative_soft_limit_deg=float("-inf")).valid)
        self.assertFalse(YawLimits(minimum_capture_s=0).valid)


if __name__ == "__main__":
    unittest.main()
