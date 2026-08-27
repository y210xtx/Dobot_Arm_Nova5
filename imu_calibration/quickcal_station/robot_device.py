"""Small, calibration-focused adapter around the official Dobot Python API."""

from __future__ import annotations

from dataclasses import dataclass
import math
import re
import time
from typing import Any

import numpy as np
from PySide6.QtCore import QObject, QThread, Signal, Slot

from dobot_api import DobotApiDashboard, DobotApiFeedBack


@dataclass(frozen=True)
class RobotState:
    received_monotonic_ns: int
    controller_timestamp: int
    mode: int
    speed_scaling: float
    joints: tuple[float, ...]
    pose: tuple[float, ...]
    tcp_speed: tuple[float, ...]
    user: int
    tool: int
    digital_inputs: int
    digital_outputs: int

    @property
    def angular_speed(self) -> tuple[float, float, float]:
        return self.tcp_speed[3], self.tcp_speed[4], self.tcp_speed[5]

    @property
    def angular_speed_norm(self) -> float:
        wx, wy, wz = self.angular_speed
        return (wx * wx + wy * wy + wz * wz) ** 0.5

    @property
    def linear_speed_norm(self) -> float:
        vx, vy, vz = self.tcp_speed[:3]
        return (vx * vx + vy * vy + vz * vz) ** 0.5


class RobotFeedbackThread(QThread):
    state_received = Signal(object)
    connection_lost = Signal(str)

    def __init__(self, client: DobotApiFeedBack, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.client = client
        self.running = True

    def stop(self) -> None:
        self.running = False
        try:
            self.client.close()
        except Exception:
            pass

    def run(self) -> None:
        while self.running:
            try:
                packet = self.client.feedBackData()
                if packet is None or len(packet) == 0:
                    continue
                if int(packet["TestValue"][0]) != 0x123456789ABCDEF:
                    continue
                tcp_speed = tuple(float(value) for value in np.asarray(packet["TCPSpeedActual"][0], dtype=float))
                state = RobotState(
                    received_monotonic_ns=time.monotonic_ns(),
                    controller_timestamp=int(packet["TimeStamp"][0]),
                    mode=int(packet["RobotMode"][0]),
                    speed_scaling=float(packet["SpeedScaling"][0]),
                    joints=tuple(float(value) for value in np.asarray(packet["QActual"][0], dtype=float)),
                    pose=tuple(float(value) for value in np.asarray(packet["ToolVectorActual"][0], dtype=float)),
                    tcp_speed=tcp_speed,
                    user=int(packet["User"][0]),
                    tool=int(packet["Tool"][0]),
                    digital_inputs=int(packet["DigitalInputs"][0]),
                    digital_outputs=int(packet["DigitalOutputs"][0]),
                )
                self.state_received.emit(state)
            except Exception as exc:
                if self.running:
                    self.connection_lost.emit(str(exc))
                return


class RobotDevice(QObject):
    connection_changed = Signal(bool, str)
    state_received = Signal(object)
    error_occurred = Signal(str)
    log_message = Signal(str)

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.dashboard: DobotApiDashboard | None = None
        self.feedback_client: DobotApiFeedBack | None = None
        self.feedback_thread: RobotFeedbackThread | None = None
        self.latest_state: RobotState | None = None
        self.ip = ""

    @property
    def connected(self) -> bool:
        return self.dashboard is not None and self.feedback_thread is not None and self.feedback_thread.isRunning()

    @staticmethod
    def response_error_id(response: Any) -> int | None:
        match = re.match(r"\s*(-?\d+)", str(response))
        return int(match.group(1)) if match else None

    @Slot(str, int, int)
    def connect_robot(self, ip: str, dashboard_port: int = 29999, feedback_port: int = 30004) -> bool:
        self.disconnect_robot()
        try:
            dashboard = DobotApiDashboard(ip, dashboard_port)
            feedback = DobotApiFeedBack(ip, feedback_port)
            for client in (dashboard, feedback):
                socket_value = getattr(client, "socket_dobot", 0)
                if not socket_value:
                    raise ConnectionError("Dobot API 未创建有效 socket")
                socket_value.getpeername()
            self.dashboard = dashboard
            self.feedback_client = feedback
            self.feedback_thread = RobotFeedbackThread(feedback, self)
            self.feedback_thread.state_received.connect(self._on_state)
            self.feedback_thread.connection_lost.connect(self._on_connection_lost)
            self.feedback_thread.start()
            self.ip = ip
        except Exception as exc:
            self._close_clients()
            self.error_occurred.emit(f"机械臂连接失败：{exc}")
            return False
        self.connection_changed.emit(True, ip)
        self.log_message.emit(f"机械臂已连接：{ip}")
        return True

    @Slot()
    def disconnect_robot(self) -> None:
        if self.feedback_thread is not None:
            self.feedback_thread.stop()
            self.feedback_thread.wait(1500)
            self.feedback_thread = None
        self._close_clients()
        self.latest_state = None
        if self.ip:
            self.connection_changed.emit(False, self.ip)
        self.ip = ""

    def _close_clients(self) -> None:
        for client in (self.feedback_client, self.dashboard):
            if client is not None:
                try:
                    client.close()
                except Exception:
                    pass
        self.feedback_client = None
        self.dashboard = None

    def _command(self, description: str, callback) -> bool:
        if self.dashboard is None:
            self.error_occurred.emit("机械臂未连接")
            return False
        try:
            response = callback()
        except Exception as exc:
            self.error_occurred.emit(f"{description}失败：{exc}")
            return False
        error_id = self.response_error_id(response)
        self.log_message.emit(f"{description}: {response}")
        if error_id is None:
            self.error_occurred.emit(f"{description}返回无法解析，按失败处理：{response!r}")
            return False
        if error_id != 0:
            self.error_occurred.emit(f"{description}被控制器拒绝：{response}")
            return False
        return True

    def enable(self) -> bool:
        return self._command("机械臂使能", self.dashboard.EnableRobot if self.dashboard else lambda: None)

    def disable(self) -> bool:
        return self._command("机械臂下使能", self.dashboard.DisableRobot if self.dashboard else lambda: None)

    def clear_error(self) -> bool:
        return self._command("清除机械臂报警", self.dashboard.ClearError if self.dashboard else lambda: None)

    def stop(self) -> bool:
        return self._command("停止机械臂", self.dashboard.Stop if self.dashboard else lambda: None)

    def set_speed_factor(self, percent: int) -> bool:
        percent = max(1, min(100, int(percent)))
        return self._command("设置机械臂速度比例", lambda: self.dashboard.SpeedFactor(percent))

    def activate_frames(self, user: int, tool: int) -> bool:
        if not 0 <= int(user) <= 9 or not 0 <= int(tool) <= 9:
            self.error_occurred.emit("User/Tool 编号必须在 0～9 范围内")
            return False
        if not self._command("启用 User 坐标系", lambda: self.dashboard.User(int(user))):
            return False
        return self._command("启用 Tool 坐标系", lambda: self.dashboard.Tool(int(tool)))

    def set_and_activate_tool(
        self, index: int, offset: tuple[float, ...]
    ) -> bool:
        index = int(index)
        values = tuple(float(value) for value in offset)
        if not 1 <= index <= 9:
            self.error_occurred.emit("可配置的 Tool 编号必须在 1～9 范围内")
            return False
        if len(values) != 6 or not all(math.isfinite(value) for value in values):
            self.error_occurred.emit("Tool 偏置必须包含 6 个有限数值")
            return False
        if any(abs(value) > 1000.0 for value in values[:3]):
            self.error_occurred.emit("Tool X/Y/Z 偏置绝对值不能超过 1000 mm")
            return False
        if any(abs(value) > 180.0 for value in values[3:]):
            self.error_occurred.emit("Tool Rx/Ry/Rz 偏置必须在 -180°～180°之间")
            return False
        table = "{" + ",".join(f"{value:.6f}" for value in values) + "}"
        if not self._command(
            f"保存 Tool {index} 坐标系",
            lambda: self.dashboard.SetTool(index, table),
        ):
            return False
        return self._command(
            f"启用 Tool {index} 坐标系", lambda: self.dashboard.Tool(index)
        )

    def activate_tool(self, index: int) -> bool:
        index = int(index)
        if not 0 <= index <= 9:
            self.error_occurred.emit("Tool 编号必须在 0～9 范围内")
            return False
        return self._command(
            f"启用 Tool {index} 坐标系", lambda: self.dashboard.Tool(index)
        )

    def move_pose_j(
        self,
        pose: tuple[float, ...],
        velocity_percent: int = 20,
        user: int = -1,
        tool: int = -1,
    ) -> bool:
        if len(pose) != 6:
            self.error_occurred.emit("目标位姿必须包含 X/Y/Z/Rx/Ry/Rz 六个值")
            return False
        velocity_percent = max(1, min(100, int(velocity_percent)))
        return self._command(
            "机械臂移动到目标位姿",
            lambda: self.dashboard.MovJ(
                *pose,
                0,
                user=int(user),
                tool=int(tool),
                a=20,
                v=velocity_percent,
                cp=0,
            ),
        )

    def move_pose_l(
        self,
        pose: tuple[float, ...],
        velocity_percent: int = 20,
        user: int = -1,
        tool: int = -1,
    ) -> bool:
        values = tuple(float(value) for value in pose)
        if len(values) != 6 or not all(math.isfinite(value) for value in values):
            self.error_occurred.emit("绝对目标位姿必须包含 6 个有限数值")
            return False
        if not 0 <= int(user) <= 9 or not 0 <= int(tool) <= 9:
            self.error_occurred.emit("绝对运动的 User/Tool 编号必须在 0～9 范围内")
            return False
        velocity_percent = max(1, min(100, int(velocity_percent)))
        return self._command(
            "机械臂直线移动到绝对 TCP 位姿",
            lambda: self.dashboard.MovL(
                *values,
                0,
                user=int(user),
                tool=int(tool),
                a=20,
                v=velocity_percent,
                cp=0,
            ),
        )

    def move_joints(self, joints: tuple[float, ...], velocity_percent: int = 15) -> bool:
        if len(joints) != 6:
            self.error_occurred.emit("目标关节位姿必须包含 J1～J6 六个值")
            return False
        velocity_percent = max(1, min(100, int(velocity_percent)))
        return self._command(
            "机械臂移动到示教关节位",
            lambda: self.dashboard.MovJ(
                *joints,
                1,
                a=20,
                v=velocity_percent,
                cp=0,
            ),
        )

    def relative_tool_rotation(
        self,
        axis: str,
        degrees: float,
        velocity_percent: int = 20,
        acceleration_percent: int = 20,
    ) -> bool:
        try:
            axis_index = {"Rx": 3, "Ry": 4, "Rz": 5}[axis]
        except KeyError:
            self.error_occurred.emit(f"不支持的 Tool 旋转轴：{axis}")
            return False
        degrees = float(degrees)
        if not math.isfinite(degrees) or abs(degrees) < 0.1 or abs(degrees) > 180.0:
            self.error_occurred.emit("Tool 相对旋转角度绝对值必须在 0.1°～180°之间")
            return False
        velocity_percent = max(1, min(100, int(velocity_percent)))
        acceleration_percent = max(1, min(100, int(acceleration_percent)))
        # Avoid crossing the controller's +/-180 degree orientation boundary in
        # one command. Equal segments also avoid leaving a tiny final segment.
        segment_count = max(1, math.ceil(abs(degrees) / 170.0))
        segment_degrees = degrees / segment_count
        for index in range(segment_count):
            offsets = [0.0] * 6
            offsets[axis_index] = segment_degrees
            cp = 100 if index < segment_count - 1 else 0
            if not self._command(
                f"Tool {axis} 相对旋转 {segment_degrees:+.2f}°"
                f"（{index + 1}/{segment_count}）",
                lambda offsets=tuple(offsets), cp=cp: self.dashboard.RelMovLTool(
                    *offsets,
                    user=-1,
                    tool=-1,
                    a=acceleration_percent,
                    v=velocity_percent,
                    cp=cp,
                ),
            ):
                return False
        return True

    def relative_tool_move(
        self, offset: tuple[float, ...], velocity_percent: int = 20
    ) -> bool:
        values = tuple(float(value) for value in offset)
        if len(values) != 6 or not all(math.isfinite(value) for value in values):
            self.error_occurred.emit("Tool 相对运动必须包含 6 个有限数值")
            return False
        if not any(abs(value) >= 0.001 for value in values):
            self.error_occurred.emit("Tool 相对运动量不能全部为 0")
            return False
        if any(abs(value) > 500.0 for value in values[:3]):
            self.error_occurred.emit("单次 Tool X/Y/Z 相对位移绝对值不能超过 500 mm")
            return False
        if any(abs(value) > 180.0 for value in values[3:]):
            self.error_occurred.emit("单次 Tool Rx/Ry/Rz 相对角度必须在 -180°～180°之间")
            return False
        velocity_percent = max(1, min(100, int(velocity_percent)))
        return self._command(
            "Tool 坐标系末端相对运动",
            lambda: self.dashboard.RelMovLTool(
                *values, user=-1, tool=-1, a=20, v=velocity_percent, cp=0
            ),
        )

    def start_tool_jog(
        self,
        axis: str,
        positive: bool,
        speed_factor_percent: int,
        user: int = -1,
        tool: int = -1,
    ) -> bool:
        if axis not in ("Rx", "Ry", "Rz"):
            self.error_occurred.emit(f"不支持的 Tool 点动轴：{axis}")
            return False
        speed_factor_percent = max(1, min(100, int(speed_factor_percent)))
        if not self.set_speed_factor(speed_factor_percent):
            return False
        command = f"{axis}{'+' if positive else '-'}"
        return self._command(
            f"Tool {axis} {'正向' if positive else '反向'}连续点动",
            lambda: self.dashboard.MoveJog(
                command, coordtype=2, user=int(user), tool=int(tool)
            ),
        )

    def switch_tool_jog(
        self,
        axis: str,
        positive: bool,
        user: int = -1,
        tool: int = -1,
    ) -> bool:
        """Start another Tool jog direction without changing SpeedFactor.

        Magnetic trajectories switch axes while the controller is decelerating
        from the previous jog.  Reusing the already accepted speed factor avoids
        a SpeedFactor command being rejected during that short transition.
        """
        if axis not in ("Rx", "Ry", "Rz"):
            self.error_occurred.emit(f"不支持的 Tool 点动轴：{axis}")
            return False
        command = f"{axis}{'+' if positive else '-'}"
        return self._command(
            f"切换 Tool {axis} {'正向' if positive else '反向'}连续点动",
            lambda: self.dashboard.MoveJog(
                command, coordtype=2, user=int(user), tool=int(tool)
            ),
        )

    def stop_tool_jog(self) -> bool:
        return self._command(
            "停止 Tool 连续点动",
            lambda: self.dashboard.MoveJog(""),
        )

    @Slot(object)
    def _on_state(self, state: RobotState) -> None:
        self.latest_state = state
        self.state_received.emit(state)

    @Slot(str)
    def _on_connection_lost(self, message: str) -> None:
        self.error_occurred.emit(f"机械臂反馈连接中断：{message}")
        self.disconnect_robot()
