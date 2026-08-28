import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from imu_calibration.quickcal_station.protocol import (
    ALL_IMU_MASK,
    ImuSample,
    RawImuFrame,
    RegisterImuSample,
    RegisterRawImuFrame,
)
from imu_calibration.quickcal_station.session_recorder import SessionRecorder


class SessionRecorderTests(unittest.TestCase):
    def test_r024_raw_streams_and_diagnostics_are_archived(self):
        recorder = SessionRecorder()
        with tempfile.TemporaryDirectory() as directory:
            session_dir = recorder.start(
                Path(directory), "SN001", "QC-01", {"firmware": "r024-fac-magq"}
            )
            physical = ImuSample(0.0, 0.0, 0.262, 0.0, 0.0, 1.0)
            register = RegisterImuSample(0, 0, 1000, 0, 0, 16384)
            recorder.raw_imu(
                "G01",
                RawImuFrame(7, 1, ALL_IMU_MASK, 0x20, 0x04, (physical,) * 11),
            )
            recorder.register_imu(
                "G01",
                RegisterRawImuFrame(
                    9,
                    1,
                    ALL_IMU_MASK,
                    0,
                    0,
                    0x20,
                    0x04,
                    (register,) * 11,
                ),
            )
            recorder.save_commit_ack(SimpleNamespace(cmd=0x11, status=11, detail0=7, detail1=11, seq=0))
            recorder.finish(False, "quality", {"G01": "完成"})

            raw_summary = json.loads(
                (session_dir / "raw_stream_diagnostics.json").read_text(encoding="utf-8")
            )
            self.assertEqual(raw_summary["G01"]["board_stage_ids"], [0x20])
            self.assertEqual(raw_summary["G01"]["capture_masks"], [0x04])
            self.assertAlmostEqual(
                raw_summary["G01"]["imus"][0]["gyro_norm_mean_rads"], 0.262
            )
            self.assertTrue((session_dir / "raw_imu.csv").stat().st_size > 100)
            self.assertTrue((session_dir / "register_raw_imu.csv").stat().st_size > 100)
            final = json.loads(
                (session_dir / "final_summary.json").read_text(encoding="utf-8")
            )
            self.assertEqual(final["mcal_ack"]["status"], 11)


if __name__ == "__main__":
    unittest.main()
