import struct
import unittest

from imu_calibration.quickcal_station.protocol import (
    CMD_MCAL_BEGIN,
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


class WorkflowTests(unittest.TestCase):
    def test_approved_default_work_order_totals(self):
        limits = YawLimits()
        self.assertTrue(limits.valid)
        self.assertAlmostEqual(limits.negative_safe_deg, -40.0)
        self.assertAlmostEqual(limits.positive_safe_deg, 40.0)
        self.assertAlmostEqual(limits.capture_s, 8.0 / 3.0)
        self.assertEqual(len(steps_for_limits(limits)), 20)
        self.assertAlmostEqual(expected_total_seconds(limits), 305.3333333333)
        self.assertAlmostEqual(expected_capture_seconds(limits), 133.3333333333)


if __name__ == "__main__":
    unittest.main()
