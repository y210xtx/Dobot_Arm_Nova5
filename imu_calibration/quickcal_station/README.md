# SDB QuickCal V1：11 路 IMU 机械臂标定站

这是从通用 Dobot 调试界面中拆出的专用 PySide6 工站界面。流程来源为
`QuickCal_V1_Robot_Control_Steps(1).xlsx`，手套字节协议与
`GloveFactoryCalibrationStation_Source_20260825/src/Protocol.*` 保持一致。

## 当前工作方式

- 同时连接 Dobot Nova 5（Dashboard 29999 / Feedback 30004）与手套 USB CDC。
- 只有机械臂实时状态、11 路 `type=9/type=11` 在线掩码、固件版本、SN、工位和
  Yaw 限位全部通过时，才能发送 `MCAL_BEGIN`。
- 机械臂动作采用监督执行：操作员或既有机器人程序完成表中动作；界面根据实际 TCP
  状态验证静止/匀速条件，然后开启对应 `MCAL_STAGE` 采集窗口。
- 任一设备断线、机械臂报警、原始流超时、速度偏离或阶段 ACK 未达到 11/11，界面都会
  停止机械臂并发送 `MCAL_ABORT`。
- 全部动作通过后发送 `MCAL_COMMIT`，等待 ACK 和 `type=7`，然后保存原始数据、机器人
  反馈、阶段标记、矩阵报告及最终结果。

## 启动

安装项目依赖后运行：

```powershell
python imu_calibration\quickcal_station_main.py
```

也可以双击项目根目录的 `run_quickcal_station.bat`。

## 重要限制

软件不会根据机械本体理论极限推测 Yaw 行程。工站使用前必须在“Yaw 限位与流程参数”页
填写控制器的实际软限位。当前版本不会自动生成六面和磁翻转轨迹；它负责联合通信、反馈
门控、阶段采集和结果追溯，机械臂动作由已验证的 Dobot 程序或示教轨迹执行。
