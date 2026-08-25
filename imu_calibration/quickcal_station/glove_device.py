"""Qt SerialPort adapter for the 11-IMU glove."""

from __future__ import annotations

from PySide6.QtCore import QIODevice, QObject, Signal, Slot
from PySide6.QtSerialPort import QSerialPort, QSerialPortInfo

from .protocol import (
    CMD_GET_VERSION,
    TYPE_ACK,
    TYPE_FACTORY_MAG_PAIR,
    TYPE_MCAL_REPORT,
    TYPE_RAW_IMU,
    TYPE_RAW_MAG,
    TYPE_REGISTER_RAW_IMU,
    TYPE_VERSION,
    ProtocolParser,
    build_command,
)


class GloveDevice(QObject):
    connection_changed = Signal(bool, str)
    error_occurred = Signal(str)
    log_message = Signal(str)
    raw_imu_received = Signal(object)
    register_raw_imu_received = Signal(object)
    raw_mag_received = Signal(object)
    factory_mag_pair_received = Signal(object)
    ack_received = Signal(object)
    version_received = Signal(object)
    mcal_report_received = Signal(object)
    stats_changed = Signal(object)

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.serial = QSerialPort(self)
        self.parser = ProtocolParser()
        self.serial.readyRead.connect(self._on_ready_read)
        self.serial.errorOccurred.connect(self._on_serial_error)

    @staticmethod
    def available_ports() -> list[dict[str, str]]:
        ports = []
        for item in QSerialPortInfo.availablePorts():
            vid = f"{item.vendorIdentifier():04X}" if item.hasVendorIdentifier() else "----"
            pid = f"{item.productIdentifier():04X}" if item.hasProductIdentifier() else "----"
            ports.append(
                {
                    "name": item.portName(),
                    "description": item.description() or item.manufacturer() or "USB CDC",
                    "serial": item.serialNumber(),
                    "vid_pid": f"{vid}:{pid}",
                }
            )
        return ports

    @property
    def is_open(self) -> bool:
        return self.serial.isOpen()

    @Slot(str)
    def open(self, port_name: str) -> bool:
        if self.serial.isOpen():
            self.close()
        self.parser.reset()
        self.serial.setPortName(port_name)
        self.serial.setBaudRate(115200)
        self.serial.setDataBits(QSerialPort.DataBits.Data8)
        self.serial.setParity(QSerialPort.Parity.NoParity)
        self.serial.setStopBits(QSerialPort.StopBits.OneStop)
        self.serial.setFlowControl(QSerialPort.FlowControl.NoFlowControl)
        if not self.serial.open(QIODevice.OpenModeFlag.ReadWrite):
            self.error_occurred.emit(f"无法打开手套串口 {port_name}：{self.serial.errorString()}")
            return False
        self.connection_changed.emit(True, port_name)
        self.log_message.emit(f"手套已连接：{port_name}")
        return True

    @Slot()
    def close(self) -> None:
        port_name = self.serial.portName()
        if self.serial.isOpen():
            self.serial.close()
        self.connection_changed.emit(False, port_name)

    def send_command(self, command: int, argument: int = 0, payload: bytes = b"") -> bool:
        if not self.serial.isOpen():
            self.error_occurred.emit("手套串口未连接")
            return False
        frame = build_command(command, argument, payload, include_crc=True)
        written = self.serial.write(frame)
        if written != len(frame):
            self.error_occurred.emit(f"手套命令 0x{command:02X} 发送失败：{self.serial.errorString()}")
            return False
        self.serial.flush()
        self.log_message.emit(f"TX cmd=0x{command:02X} arg=0x{argument:02X} payload={payload.hex(' ')}")
        return True

    def query_version(self) -> bool:
        return self.send_command(CMD_GET_VERSION)

    @Slot()
    def _on_ready_read(self) -> None:
        data = bytes(self.serial.readAll())
        for frame_type, frame in self.parser.feed(data):
            if frame_type == TYPE_RAW_IMU:
                self.raw_imu_received.emit(frame)
            elif frame_type == TYPE_REGISTER_RAW_IMU:
                self.register_raw_imu_received.emit(frame)
            elif frame_type == TYPE_RAW_MAG:
                self.raw_mag_received.emit(frame)
            elif frame_type == TYPE_FACTORY_MAG_PAIR:
                self.factory_mag_pair_received.emit(frame)
            elif frame_type == TYPE_ACK:
                self.ack_received.emit(frame)
            elif frame_type == TYPE_VERSION:
                self.version_received.emit(frame)
            elif frame_type == TYPE_MCAL_REPORT:
                self.mcal_report_received.emit(frame)
        self.stats_changed.emit(self.parser.stats)

    @Slot(QSerialPort.SerialPortError)
    def _on_serial_error(self, error: QSerialPort.SerialPortError) -> None:
        if error in (QSerialPort.SerialPortError.NoError, QSerialPort.SerialPortError.NotOpenError):
            return
        message = self.serial.errorString()
        self.error_occurred.emit(f"手套串口异常：{message}")
        if error == QSerialPort.SerialPortError.ResourceError:
            self.close()
