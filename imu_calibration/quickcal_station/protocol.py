"""Binary glove protocol shared with GloveFactoryCalibrationStation.

The implementation intentionally mirrors ``src/Protocol.cpp`` in the C++
factory station.  Keep this module free of Qt so its byte-level behaviour can
be regression tested without hardware or a GUI.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import struct
from typing import Any


TYPE_CAMERA = 2
TYPE_JOINT_POSE = 5
TYPE_ACK = 6
TYPE_MCAL_REPORT = 7
TYPE_VERSION = 8
TYPE_RAW_IMU = 9
TYPE_RAW_MAG = 10
TYPE_REGISTER_RAW_IMU = 11
TYPE_FACTORY_MAG_PAIR = 12

CMD_GET_VERSION = 0x08
CMD_MCAL_BEGIN = 0x10
CMD_MCAL_COMMIT = 0x11
CMD_MCAL_ABORT = 0x12
CMD_MCAL_STAGE = 0x13

MCAL_CAPTURE_GYRO_BIAS = 0x01
MCAL_CAPTURE_ACCEL = 0x02
MCAL_CAPTURE_GYRO_M = 0x04
MCAL_CAPTURE_MAG = 0x08

ALL_IMU_MASK = 0x07FF
MAX_PAYLOAD_LENGTH = 256 * 1024
EXPECTED_VERSION_PAYLOAD_VERSION = 1
EXPECTED_MCAL_REPORT_VERSION = 2
EXPECTED_MCAL_PAYLOAD_LENGTH = 1046
EXPECTED_GYRO_SEGMENTS = 6


@dataclass(frozen=True)
class ImuSample:
    gx: float
    gy: float
    gz: float
    ax: float
    ay: float
    az: float


@dataclass(frozen=True)
class RawImuFrame:
    seq: int
    version: int
    presence_mask: int
    samples: tuple[ImuSample, ...]


@dataclass(frozen=True)
class RegisterImuSample:
    gx: int
    gy: int
    gz: int
    ax: int
    ay: int
    az: int


@dataclass(frozen=True)
class RegisterRawImuFrame:
    seq: int
    version: int
    presence_mask: int
    imu_model: int
    flags: int
    samples: tuple[RegisterImuSample, ...]


@dataclass(frozen=True)
class RawMagFrame:
    seq: int
    version: int
    flags: int
    source: int
    unit: int
    field: tuple[float, float, float]


@dataclass(frozen=True)
class FactoryMagPairFrame:
    seq: int
    version: int
    flags: int
    source: int
    unit: int
    field: tuple[float, float, float]
    offset: tuple[float, float, float]


@dataclass(frozen=True)
class AckFrame:
    cmd: int
    status: int
    detail0: int
    detail1: int
    seq: int


@dataclass(frozen=True)
class VersionFrame:
    payload_version: int
    revision: int
    revision_tag: str
    build_date: str
    build_time: str
    imu_model: str
    hand_side: str
    features: int
    payload: bytes = field(default=b"", repr=False)

    @property
    def factory_intrinsic(self) -> bool:
        return bool(self.features & 0x20)

    @property
    def accel_intrinsic(self) -> bool:
        return bool(self.features & 0x40)


@dataclass(frozen=True)
class GyroQuality:
    ok: bool
    rms_mdeg: int
    window_count: int
    max_off_axis: int


@dataclass(frozen=True)
class AccelQuality:
    ok: bool
    raw: bytes


@dataclass(frozen=True)
class McalReportFrame:
    seq: int
    context: int
    version: int
    imu_count: int
    calibrated_count: int
    flash_sequence: int
    status: int
    flags: int
    mean_rms_mdeg: int
    bad_off_axis_count: int
    gyro_quality: tuple[GyroQuality, ...]
    accel_quality: tuple[AccelQuality, ...]
    gyro_matrices: tuple[tuple[float, ...], ...]
    accel_matrices: tuple[tuple[float, ...], ...]
    payload: bytes = field(repr=False)

    @property
    def format_valid(self) -> bool:
        return (
            len(self.payload) == EXPECTED_MCAL_PAYLOAD_LENGTH
            and self.context == CMD_MCAL_COMMIT
            and self.version == EXPECTED_MCAL_REPORT_VERSION
            and self.imu_count == 11
        )

    @property
    def gyro_all_ok(self) -> bool:
        return (
            self.format_valid
            and self.status == 0
            and self.calibrated_count == 11
            and bool(self.flags & 0x01)
            and len(self.gyro_quality) == 11
            and len(self.gyro_matrices) == 11
            and all(item.ok and item.window_count >= EXPECTED_GYRO_SEGMENTS for item in self.gyro_quality)
        )

    @property
    def accel_all_ok(self) -> bool:
        return (
            self.format_valid
            and self.status == 0
            and self.calibrated_count == 11
            and bool(self.flags & 0x02)
            and len(self.accel_quality) == 11
            and len(self.accel_matrices) == 11
            and all(item.ok for item in self.accel_quality)
        )

    @property
    def factory_pass(self) -> bool:
        return self.gyro_all_ok and self.accel_all_ok


@dataclass
class ProtocolStats:
    accepted_frames: int = 0
    invalid_frames: int = 0
    invalid_lengths: int = 0
    payload_errors: int = 0
    unknown_types: int = 0
    sequence_gaps: int = 0
    bytes_received: int = 0
    frames_by_type: dict[int, int] = field(default_factory=dict)


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_command(command: int, argument: int = 0, payload: bytes = b"", include_crc: bool = True) -> bytes:
    payload = bytes(payload[:0xFFFF])
    body = bytes((0xA5, 0x5A, command & 0xFF, argument & 0xFF)) + struct.pack("<H", len(payload)) + payload
    crc = crc16_ccitt(body) if include_crc else 0
    return body + struct.pack("<H", crc)


def _fixed_string(payload: bytes, offset: int, size: int) -> str:
    return payload[offset : offset + size].split(b"\0", 1)[0].decode("latin-1", errors="replace").strip()


def _finite(values: tuple[float, ...]) -> bool:
    return all(math.isfinite(value) for value in values)


class ProtocolParser:
    """Incremental parser for the device-to-host ``55 AA`` stream."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.stats = ProtocolStats()
        self._last_seq: dict[int, int] = {}

    def reset(self) -> None:
        self.buffer.clear()
        self.stats = ProtocolStats()
        self._last_seq.clear()

    def feed(self, data: bytes) -> list[tuple[int, Any]]:
        if not data:
            return []
        self.stats.bytes_received += len(data)
        self.buffer.extend(data)
        frames: list[tuple[int, Any]] = []
        while True:
            sync = self.buffer.find(b"\x55\xAA")
            if sync < 0:
                if len(self.buffer) > 1:
                    del self.buffer[:-1]
                break
            if sync:
                del self.buffer[:sync]
            if len(self.buffer) < 8:
                break
            frame_type = self.buffer[2]
            seq = self.buffer[3]
            payload_length = struct.unpack_from("<I", self.buffer, 4)[0]
            if payload_length < 4 or payload_length > MAX_PAYLOAD_LENGTH:
                self.stats.invalid_lengths += 1
                self.stats.invalid_frames += 1
                del self.buffer[:2]
                continue
            frame_length = 8 + payload_length
            if len(self.buffer) < frame_length:
                break
            payload = bytes(self.buffer[8:frame_length])
            del self.buffer[:frame_length]
            try:
                parsed = self._parse_payload(frame_type, seq, payload)
            except (ValueError, IndexError, struct.error, UnicodeError):
                parsed = None
            if parsed is None:
                self.stats.invalid_frames += 1
                continue
            self.stats.accepted_frames += 1
            self.stats.frames_by_type[frame_type] = self.stats.frames_by_type.get(frame_type, 0) + 1
            self._note_sequence(frame_type, seq)
            frames.append((frame_type, parsed))
        return frames

    def _note_sequence(self, frame_type: int, seq: int) -> None:
        previous = self._last_seq.get(frame_type)
        if previous is not None:
            delta = (seq - previous) & 0xFF
            if 1 < delta < 128:
                self.stats.sequence_gaps += delta - 1
        self._last_seq[frame_type] = seq

    def _parse_payload(self, frame_type: int, seq: int, payload: bytes) -> Any | None:
        if frame_type == TYPE_RAW_IMU:
            return self._parse_raw_imu(seq, payload)
        if frame_type == TYPE_REGISTER_RAW_IMU:
            return self._parse_register_raw_imu(seq, payload)
        if frame_type == TYPE_RAW_MAG:
            return self._parse_raw_mag(seq, payload)
        if frame_type == TYPE_FACTORY_MAG_PAIR:
            return self._parse_factory_mag_pair(seq, payload)
        if frame_type == TYPE_ACK:
            return self._parse_ack(payload)
        if frame_type == TYPE_VERSION:
            return self._parse_version(payload)
        if frame_type == TYPE_MCAL_REPORT:
            return self._parse_mcal(seq, payload)
        if frame_type in (TYPE_CAMERA, TYPE_JOINT_POSE):
            return payload
        self.stats.unknown_types += 1
        return None

    def _parse_raw_imu(self, seq: int, payload: bytes) -> RawImuFrame:
        if len(payload) < 8 or payload[0] != 1 or payload[1] != 11 or len(payload) != 8 + 11 * 24:
            raise ValueError("invalid type=9 payload")
        presence_mask = struct.unpack_from("<H", payload, 2)[0]
        samples = []
        for index in range(11):
            values = struct.unpack_from("<6f", payload, 8 + index * 24)
            if not _finite(values):
                raise ValueError("non-finite type=9 sample")
            samples.append(ImuSample(*values))
        return RawImuFrame(seq, payload[0], presence_mask, tuple(samples))

    def _parse_register_raw_imu(self, seq: int, payload: bytes) -> RegisterRawImuFrame:
        if len(payload) < 8 or payload[0] != 1 or payload[1] != 11 or len(payload) != 8 + 11 * 12:
            raise ValueError("invalid type=11 payload")
        presence_mask = struct.unpack_from("<H", payload, 2)[0]
        samples = tuple(
            RegisterImuSample(*struct.unpack_from("<6h", payload, 8 + index * 12)) for index in range(11)
        )
        return RegisterRawImuFrame(seq, payload[0], presence_mask, payload[4], payload[5], samples)

    def _parse_raw_mag(self, seq: int, payload: bytes) -> RawMagFrame:
        if len(payload) != 16 or payload[0] != 1:
            raise ValueError("invalid type=10 payload")
        field_value = struct.unpack_from("<3f", payload, 4)
        if not _finite(field_value):
            raise ValueError("non-finite magnetometer sample")
        return RawMagFrame(seq, payload[0], payload[1], payload[2], payload[3], field_value)

    def _parse_factory_mag_pair(self, seq: int, payload: bytes) -> FactoryMagPairFrame:
        if len(payload) != 28 or payload[0] != 1:
            raise ValueError("invalid type=12 payload")
        field_value = struct.unpack_from("<3f", payload, 4)
        offset = struct.unpack_from("<3f", payload, 16)
        if not _finite(field_value) or not _finite(offset):
            raise ValueError("non-finite factory magnetometer pair")
        return FactoryMagPairFrame(seq, payload[0], payload[1], payload[2], payload[3], field_value, offset)

    @staticmethod
    def _parse_ack(payload: bytes) -> AckFrame:
        if len(payload) < 8:
            raise ValueError("short ACK")
        return AckFrame(payload[0], payload[1], payload[2], payload[3], struct.unpack_from("<H", payload, 4)[0])

    @staticmethod
    def _parse_version(payload: bytes) -> VersionFrame:
        if len(payload) != 48 or payload[0] != CMD_GET_VERSION:
            raise ValueError("invalid version payload")
        model = "IIM-42652" if payload[39] == 1 else "LSM6DSV16X"
        hand = "左手" if payload[40] == 1 else "右手"
        return VersionFrame(
            payload[1],
            struct.unpack_from("<H", payload, 2)[0],
            _fixed_string(payload, 4, 16),
            _fixed_string(payload, 20, 11),
            _fixed_string(payload, 31, 8),
            model,
            hand,
            payload[41],
            bytes(payload),
        )

    @staticmethod
    def _matrix_block(payload: bytes, offset: int, count: int) -> tuple[tuple[float, ...], ...]:
        if len(payload) < offset + count * 36:
            return ()
        matrices = []
        for index in range(count):
            values = struct.unpack_from("<9f", payload, offset + index * 36)
            if not _finite(values):
                return ()
            matrices.append(values)
        return tuple(matrices)

    def _parse_mcal(self, seq: int, payload: bytes) -> McalReportFrame:
        if len(payload) != EXPECTED_MCAL_PAYLOAD_LENGTH:
            raise ValueError("invalid MCAL report length")
        if payload[0] != CMD_MCAL_COMMIT or payload[1] != EXPECTED_MCAL_REPORT_VERSION or payload[2] != 11:
            raise ValueError("unsupported MCAL report header")
        imu_count = 11
        gyro_quality: list[GyroQuality] = []
        if len(payload) >= 12 + imu_count * 8:
            for index in range(imu_count):
                offset = 12 + index * 8
                gyro_quality.append(
                    GyroQuality(
                        bool(payload[offset]),
                        struct.unpack_from("<H", payload, offset + 2)[0],
                        struct.unpack_from("<H", payload, offset + 4)[0],
                        struct.unpack_from("<H", payload, offset + 6)[0],
                    )
                )
        accel_quality: list[AccelQuality] = []
        accel_quality_offset = 12 + 11 * 8 + 11 * 36
        if payload[7] & 0x02 and len(payload) >= accel_quality_offset + imu_count * 14:
            for index in range(imu_count):
                raw = payload[accel_quality_offset + index * 14 : accel_quality_offset + (index + 1) * 14]
                accel_quality.append(AccelQuality(bool(raw[0]), raw))
        return McalReportFrame(
            seq=seq,
            context=payload[0],
            version=payload[1],
            imu_count=payload[2],
            calibrated_count=payload[3],
            flash_sequence=struct.unpack_from("<H", payload, 4)[0],
            status=payload[6],
            flags=payload[7],
            mean_rms_mdeg=struct.unpack_from("<H", payload, 8)[0],
            bad_off_axis_count=payload[10],
            gyro_quality=tuple(gyro_quality),
            accel_quality=tuple(accel_quality),
            gyro_matrices=self._matrix_block(payload, 12 + 11 * 8, imu_count),
            accel_matrices=self._matrix_block(payload, accel_quality_offset + 11 * 14, imu_count),
            payload=payload,
        )
