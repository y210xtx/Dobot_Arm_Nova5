"""Traceable session logging for QuickCal V1."""

from __future__ import annotations

import csv
from dataclasses import asdict
from datetime import datetime
import json
import math
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
        self._raw_diagnostics: dict[str, Any] = {}
        self._last_raw_seq: dict[tuple[str, int], int] = {}
        self._commit_ack: dict[str, Any] | None = None
        self._report_summary: dict[str, Any] | None = None

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
        self._raw_diagnostics = {}
        self._last_raw_seq = {}
        self._commit_ack = None
        self._report_summary = None
        self._open_csv("markers", "stage_markers.csv", ("host_ns", "elapsed_ms", "event", "step", "detail"))
        self._open_csv(
            "cdc",
            "cdc_bytes.csv",
            ("host_ns", "elapsed_ms", "direction", "byte_count", "data_hex"),
        )
        self._open_csv(
            "raw_imu",
            "raw_imu.csv",
            ("host_ns", "elapsed_ms", "step", "seq", "presence_mask", "board_stage_id", "capture_mask", "imu", "gx_rads", "gy_rads", "gz_rads", "ax_g", "ay_g", "az_g"),
        )
        self._open_csv(
            "register_imu",
            "register_raw_imu.csv",
            ("host_ns", "elapsed_ms", "step", "seq", "presence_mask", "imu_model", "flags", "board_stage_id", "capture_mask", "imu", "gx_lsb", "gy_lsb", "gz_lsb", "ax_lsb", "ay_lsb", "az_lsb"),
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
            "schema": "SDB.quick_cal.robot.r024.v1",
            "started_at": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            "product_sn": product_sn,
            "station_id": station_id,
            **metadata,
        }
        self._write_json("session.json", session)
        return self.directory

    def save_version(self, frame) -> None:
        if self.directory is None:
            return
        (self.directory / "version_type8.bin").write_bytes(frame.payload)
        self._write_json(
            "version_type8.json",
            {
                "payload_version": frame.payload_version,
                "revision": frame.revision,
                "revision_tag": frame.revision_tag,
                "build_date": frame.build_date,
                "build_time": frame.build_time,
                "imu_model": frame.imu_model,
                "hand_side": frame.hand_side,
                "features": frame.features,
                "factory_intrinsic": frame.factory_intrinsic,
                "accel_intrinsic": frame.accel_intrinsic,
                "factory_raw_streams": frame.factory_raw_streams,
                "magnetic_factory": frame.magnetic_factory,
                "r024_compatible": frame.r024_compatible,
                "payload_hex": frame.payload.hex(),
            },
        )

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
            writer.writerow((now, f"{elapsed:.3f}", step, frame.seq, frame.presence_mask, frame.stage_id, frame.capture_mask, index, sample.gx, sample.gy, sample.gz, sample.ax, sample.ay, sample.az))
        self._update_raw_diagnostics(step, 9, frame)

    def register_imu(self, step: str, frame) -> None:
        if not self.active:
            return
        now, elapsed = self._time()
        writer = self._writers["register_imu"]
        for index, sample in enumerate(frame.samples):
            writer.writerow((now, f"{elapsed:.3f}", step, frame.seq, frame.presence_mask, frame.imu_model, frame.flags, frame.stage_id, frame.capture_mask, index, sample.gx, sample.gy, sample.gz, sample.ax, sample.ay, sample.az))
        self._update_raw_diagnostics(step, 11, frame)

    @staticmethod
    def _empty_imu_diagnostic() -> dict[str, Any]:
        return {
            "type9_count": 0,
            "type11_count": 0,
            "physical_sum": [0.0] * 6,
            "physical_sum_sq": [0.0] * 6,
            "gyro_norm_sum": 0.0,
            "gyro_norm_sum_sq": 0.0,
            "gyro_norm_max": 0.0,
            "accel_norm_sum": 0.0,
            "accel_norm_sum_sq": 0.0,
            "accel_norm_min": math.inf,
            "accel_norm_max": -math.inf,
            "register_min": [32767] * 6,
            "register_max": [-32768] * 6,
        }

    def _step_diagnostic(self, step: str) -> dict[str, Any]:
        return self._raw_diagnostics.setdefault(
            step,
            {
                "type9_frames": 0,
                "type11_frames": 0,
                "type9_sequence_gaps": 0,
                "type11_sequence_gaps": 0,
                "presence_masks": set(),
                "board_stage_ids": set(),
                "capture_masks": set(),
                "imus": [self._empty_imu_diagnostic() for _ in range(11)],
            },
        )

    def _update_raw_diagnostics(self, step: str, frame_type: int, frame) -> None:
        stats = self._step_diagnostic(step)
        stats[f"type{frame_type}_frames"] += 1
        stats["presence_masks"].add(int(frame.presence_mask))
        stats["board_stage_ids"].add(int(frame.stage_id))
        stats["capture_masks"].add(int(frame.capture_mask))
        sequence_key = (step, frame_type)
        previous = self._last_raw_seq.get(sequence_key)
        if previous is not None:
            delta = (int(frame.seq) - previous) & 0xFF
            if 1 < delta < 128:
                stats[f"type{frame_type}_sequence_gaps"] += delta - 1
        self._last_raw_seq[sequence_key] = int(frame.seq)
        for index, sample in enumerate(frame.samples):
            imu = stats["imus"][index]
            values = (
                float(sample.gx),
                float(sample.gy),
                float(sample.gz),
                float(sample.ax),
                float(sample.ay),
                float(sample.az),
            )
            if frame_type == 9:
                imu["type9_count"] += 1
                for axis, value in enumerate(values):
                    imu["physical_sum"][axis] += value
                    imu["physical_sum_sq"][axis] += value * value
                gyro_norm = math.sqrt(sum(value * value for value in values[:3]))
                accel_norm = math.sqrt(sum(value * value for value in values[3:]))
                imu["gyro_norm_sum"] += gyro_norm
                imu["gyro_norm_sum_sq"] += gyro_norm * gyro_norm
                imu["gyro_norm_max"] = max(imu["gyro_norm_max"], gyro_norm)
                imu["accel_norm_sum"] += accel_norm
                imu["accel_norm_sum_sq"] += accel_norm * accel_norm
                imu["accel_norm_min"] = min(imu["accel_norm_min"], accel_norm)
                imu["accel_norm_max"] = max(imu["accel_norm_max"], accel_norm)
            else:
                imu["type11_count"] += 1
                for axis, value in enumerate(values):
                    imu["register_min"][axis] = min(imu["register_min"][axis], int(value))
                    imu["register_max"][axis] = max(imu["register_max"][axis], int(value))

    @staticmethod
    def _mean_std(total: float, total_sq: float, count: int) -> tuple[float, float]:
        if count <= 0:
            return 0.0, 0.0
        mean = total / count
        return mean, math.sqrt(max(0.0, total_sq / count - mean * mean))

    def raw_diagnostic_summary(self) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for step, stats in self._raw_diagnostics.items():
            imus = []
            for index, source in enumerate(stats["imus"]):
                count = source["type9_count"]
                physical_mean = []
                physical_std = []
                for axis in range(6):
                    mean, std = self._mean_std(
                        source["physical_sum"][axis],
                        source["physical_sum_sq"][axis],
                        count,
                    )
                    physical_mean.append(mean)
                    physical_std.append(std)
                gyro_mean, gyro_std = self._mean_std(
                    source["gyro_norm_sum"], source["gyro_norm_sum_sq"], count
                )
                accel_mean, accel_std = self._mean_std(
                    source["accel_norm_sum"], source["accel_norm_sum_sq"], count
                )
                imus.append(
                    {
                        "index": index,
                        "type9_count": count,
                        "type11_count": source["type11_count"],
                        "physical_mean": physical_mean,
                        "physical_std": physical_std,
                        "gyro_norm_mean_rads": gyro_mean,
                        "gyro_norm_std_rads": gyro_std,
                        "gyro_norm_max_rads": source["gyro_norm_max"],
                        "accel_norm_mean_g": accel_mean,
                        "accel_norm_std_g": accel_std,
                        "accel_norm_min_g": (
                            source["accel_norm_min"] if count else None
                        ),
                        "accel_norm_max_g": (
                            source["accel_norm_max"] if count else None
                        ),
                        "register_min": (
                            source["register_min"]
                            if source["type11_count"]
                            else None
                        ),
                        "register_max": (
                            source["register_max"]
                            if source["type11_count"]
                            else None
                        ),
                    }
                )
            result[step] = {
                key: value
                for key, value in stats.items()
                if key != "imus"
            }
            for key in ("presence_masks", "board_stage_ids", "capture_masks"):
                result[step][key] = sorted(result[step][key])
            result[step]["imus"] = imus
        return result

    def raw_diagnostic_issues(self, imu_names: tuple[str, ...]) -> list[str]:
        issues: list[str] = []
        for step, stats in self.raw_diagnostic_summary().items():
            if stats["type9_frames"] == 0 or stats["type11_frames"] == 0:
                issues.append(
                    f"{step} 原始流不足：type9={stats['type9_frames']}，"
                    f"type11={stats['type11_frames']}"
                )
                continue
            for imu in stats["imus"]:
                name = imu_names[imu["index"]]
                accel_mean = imu["accel_norm_mean_g"]
                accel_std = imu["accel_norm_std_g"]
                gyro_mean = imu["gyro_norm_mean_rads"]
                gyro_max = imu["gyro_norm_max_rads"]
                if step == "P1":
                    if gyro_max > 0.10:
                        issues.append(f"P1 {name} 静止角速度峰值 {gyro_max:.3f} rad/s")
                    if not 0.7 <= accel_mean <= 1.3 or accel_std > 0.05:
                        issues.append(
                            f"P1 {name} 加速度模长 {accel_mean:.3f}±{accel_std:.3f} g"
                        )
                elif step.startswith("A"):
                    if not 0.7 <= accel_mean <= 1.3 or accel_std > 0.03:
                        issues.append(
                            f"{step} {name} 加速度模长 {accel_mean:.3f}±{accel_std:.3f} g"
                        )
                    if gyro_max > 0.05:
                        issues.append(f"{step} {name} 静止角速度峰值 {gyro_max:.3f} rad/s")
                elif step.startswith("G"):
                    if not 0.15 <= gyro_mean <= 0.40 or imu["gyro_norm_std_rads"] > 0.05:
                        issues.append(
                            f"{step} {name} 角速度模长 {gyro_mean:.3f}±"
                            f"{imu['gyro_norm_std_rads']:.3f} rad/s"
                        )
                    if not 0.7 <= accel_mean <= 1.3 or accel_std > 0.08:
                        issues.append(
                            f"{step} {name} 动态加速度模长 {accel_mean:.3f}±{accel_std:.3f} g"
                        )
        return issues

    def save_commit_ack(self, frame) -> None:
        self._commit_ack = {
            "cmd": frame.cmd,
            "status": frame.status,
            "cal_seq": frame.seq,
            "detail0": frame.detail0,
            "detail1": frame.detail1,
            "interpretation": (
                {
                    "gyro_pass_count": frame.detail0,
                    "mean_gyro_rms_deg_rounded": frame.detail1,
                    "flash_written": frame.seq != 0,
                }
                if frame.status == 0
                else {
                    "gyro_pass_count": frame.detail0,
                    "accel_pass_count": frame.detail1,
                    "flash_written": False,
                }
                if frame.status == 11
                else {"flash_written": False}
            ),
        }
        self._write_json("mcal_ack.json", self._commit_ack)

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
        (self.directory / "mcal_report_type7.bin").write_bytes(report.payload)
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
            "gyro_quality": [
                {**asdict(item), "reject_reasons": list(item.reject_reasons)}
                for item in report.gyro_quality
            ],
            "accel_quality": [asdict(item) for item in report.accel_quality],
            "mag_all_ok": report.mag_all_ok,
            "mag_quality": {
                **asdict(report.mag_quality),
                "reject_reasons": list(report.mag_quality.reject_reasons),
            },
            "gyro_matrices": [list(matrix) for matrix in report.gyro_matrices],
            "accel_matrices": [list(matrix) for matrix in report.accel_matrices],
        }
        self._report_summary = summary
        self._write_json("mcal_report.json", summary)
        self._write_json("mcal_report_type7.json", summary)

    def finish(self, passed: bool, reason: str, steps: dict[str, str]) -> None:
        if self.directory is None:
            return
        finished_at = datetime.now().astimezone().isoformat(timespec="milliseconds")
        diagnostics = self.raw_diagnostic_summary()
        result = {
            "finished_at": finished_at,
            "pass": passed,
            "reason": reason,
            "steps": steps,
        }
        self._write_json("result.json", result)
        self._write_json("raw_stream_diagnostics.json", diagnostics)
        self._write_json(
            "final_summary.json",
            {
                **result,
                "mcal_ack": self._commit_ack,
                "mcal_report": self._report_summary,
                "raw_stream_diagnostics": diagnostics,
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
