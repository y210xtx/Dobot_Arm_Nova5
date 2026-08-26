"""Traceable session logging for QuickCal V1."""

from __future__ import annotations

import csv
from dataclasses import asdict
from datetime import datetime
import json
from pathlib import Path
import re
import time
from typing import Any


def _safe_name(value: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_-]+", "_", value.strip())
    return cleaned or "UNKNOWN"


class SessionRecorder:
    def __init__(self) -> None:
        self.directory: Path | None = None
        self.started_monotonic_ns = 0
        self._files: dict[str, Any] = {}
        self._writers: dict[str, csv.writer] = {}

    @property
    def active(self) -> bool:
        return self.directory is not None

    def start(self, base_directory: Path, product_sn: str, station_id: str, metadata: dict[str, Any]) -> Path:
        self.close()
        base_directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.directory = base_directory / f"SDB_QuickCal_{stamp}_{_safe_name(product_sn)}"
        self.directory.mkdir(parents=True, exist_ok=False)
        self.started_monotonic_ns = time.monotonic_ns()
        self._open_csv("markers", "stage_markers.csv", ("host_ns", "elapsed_ms", "event", "step", "detail"))
        self._open_csv(
            "cdc",
            "cdc_bytes.csv",
            ("host_ns", "elapsed_ms", "direction", "byte_count", "data_hex"),
        )
        self._open_csv(
            "raw_imu",
            "raw_imu.csv",
            ("host_ns", "elapsed_ms", "step", "seq", "presence_mask", "imu", "gx_rads", "gy_rads", "gz_rads", "ax_g", "ay_g", "az_g"),
        )
        self._open_csv(
            "register_imu",
            "register_raw_imu.csv",
            ("host_ns", "elapsed_ms", "step", "seq", "presence_mask", "imu", "gx_lsb", "gy_lsb", "gz_lsb", "ax_lsb", "ay_lsb", "az_lsb"),
        )
        self._open_csv("raw_mag", "raw_mag.csv", ("host_ns", "elapsed_ms", "step", "seq", "flags", "source", "unit", "mx", "my", "mz"))
        self._open_csv(
            "mag_pair",
            "mmc_set_reset_pair.csv",
            ("host_ns", "elapsed_ms", "step", "seq", "flags", "source", "unit", "field_x", "field_y", "field_z", "offset_x", "offset_y", "offset_z"),
        )
        self._open_csv(
            "robot",
            "robot_feedback.csv",
            ("host_ns", "elapsed_ms", "step", "controller_timestamp", "mode", "speed_scaling", "x", "y", "z", "rx", "ry", "rz", "vx", "vy", "vz", "wx", "wy", "wz"),
        )
        session = {
            "schema": "SDB.quick_cal.robot.v1",
            "started_at": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            "product_sn": product_sn,
            "station_id": station_id,
            **metadata,
        }
        self._write_json("session.json", session)
        return self.directory

    def _open_csv(self, key: str, name: str, header: tuple[str, ...]) -> None:
        if self.directory is None:
            return
        handle = (self.directory / name).open("w", newline="", encoding="utf-8-sig")
        writer = csv.writer(handle)
        writer.writerow(header)
        self._files[key] = handle
        self._writers[key] = writer

    def _time(self) -> tuple[int, float]:
        now = time.monotonic_ns()
        return now, (now - self.started_monotonic_ns) / 1_000_000.0

    def marker(self, event: str, step: str, detail: str = "") -> None:
        if not self.active:
            return
        now, elapsed = self._time()
        self._writers["markers"].writerow((now, f"{elapsed:.3f}", event, step, detail))
        self._files["markers"].flush()

    def cdc_bytes(self, direction: str, data: bytes) -> None:
        """Persist every TX frame and RX read chunk losslessly as hexadecimal bytes."""
        if not self.active or not data:
            return
        now, elapsed = self._time()
        raw = bytes(data)
        self._writers["cdc"].writerow(
            (now, f"{elapsed:.3f}", direction, len(raw), raw.hex(" "))
        )
        self._files["cdc"].flush()

    def raw_imu(self, step: str, frame) -> None:
        if not self.active:
            return
        now, elapsed = self._time()
        writer = self._writers["raw_imu"]
        for index, sample in enumerate(frame.samples):
            writer.writerow((now, f"{elapsed:.3f}", step, frame.seq, frame.presence_mask, index, sample.gx, sample.gy, sample.gz, sample.ax, sample.ay, sample.az))

    def register_imu(self, step: str, frame) -> None:
        if not self.active:
            return
        now, elapsed = self._time()
        writer = self._writers["register_imu"]
        for index, sample in enumerate(frame.samples):
            writer.writerow((now, f"{elapsed:.3f}", step, frame.seq, frame.presence_mask, index, sample.gx, sample.gy, sample.gz, sample.ax, sample.ay, sample.az))

    def raw_mag(self, step: str, frame) -> None:
        if not self.active:
            return
        now, elapsed = self._time()
        self._writers["raw_mag"].writerow((now, f"{elapsed:.3f}", step, frame.seq, frame.flags, frame.source, frame.unit, *frame.field))

    def mag_pair(self, step: str, frame) -> None:
        if not self.active:
            return
        now, elapsed = self._time()
        self._writers["mag_pair"].writerow((now, f"{elapsed:.3f}", step, frame.seq, frame.flags, frame.source, frame.unit, *frame.field, *frame.offset))

    def robot_state(self, step: str, state) -> None:
        if not self.active:
            return
        now, elapsed = self._time()
        self._writers["robot"].writerow((now, f"{elapsed:.3f}", step, state.controller_timestamp, state.mode, state.speed_scaling, *state.pose, *state.tcp_speed))

    def save_report(self, report) -> None:
        if self.directory is None:
            return
        (self.directory / "mcal_report.bin").write_bytes(report.payload)
        summary = {
            "seq": report.seq,
            "context": report.context,
            "version": report.version,
            "imu_count": report.imu_count,
            "calibrated_count": report.calibrated_count,
            "flash_sequence": report.flash_sequence,
            "status": report.status,
            "flags": report.flags,
            "mean_rms_mdeg": report.mean_rms_mdeg,
            "bad_off_axis_count": report.bad_off_axis_count,
            "format_valid": report.format_valid,
            "gyro_all_ok": report.gyro_all_ok,
            "accel_all_ok": report.accel_all_ok,
            "factory_pass": report.factory_pass,
            "gyro_quality": [asdict(item) for item in report.gyro_quality],
            "accel_quality": [
                {"ok": item.ok, "raw_hex": item.raw.hex()} for item in report.accel_quality
            ],
            "gyro_matrices": [list(matrix) for matrix in report.gyro_matrices],
            "accel_matrices": [list(matrix) for matrix in report.accel_matrices],
        }
        self._write_json("mcal_report.json", summary)

    def finish(self, passed: bool, reason: str, steps: dict[str, str]) -> None:
        if self.directory is None:
            return
        self._write_json(
            "result.json",
            {
                "finished_at": datetime.now().astimezone().isoformat(timespec="milliseconds"),
                "pass": passed,
                "reason": reason,
                "steps": steps,
            },
        )
        self.close()

    def _write_json(self, name: str, value: dict[str, Any]) -> None:
        if self.directory is not None:
            (self.directory / name).write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")

    def close(self) -> None:
        for handle in self._files.values():
            try:
                handle.flush()
                handle.close()
            except Exception:
                pass
        self._files.clear()
        self._writers.clear()
        self.directory = None
