# 六轴臂自动标定 - 手套 USB 读写接口说明

> 版本：v1.2  
> 日期：2026-08-05  
> 适用协议：`USB_HOST_PROTOCOL.md`（`OUTPUT_JOINTS_21`、M 标定 `type=7`）  
> 适用范围：六轴机械臂自动标定程序与动捕手套之间的 USB 通信。**本文件不包含六轴机械臂厂商的运动控制 API。**

> **本文仅覆盖 IMU M 标定：**用于修正每颗 IMU 的陀螺 M 矩阵、加速度计 W 矩阵及加速度计零偏。自动标定时，手套或模型手保持刚性，由六轴臂带动其进行慢速、多轴、全方位转动；控制命令仅使用 `MCAL_BEGIN (0x10)`、`MCAL_COMMIT (0x11)` 与 `MCAL_ABORT (0x12)`。

| M 标定目的 | 自动化动作 | 命令 | 可读取结果 |
|---|---|---|---|
| 修正 IMU 陀螺交叉轴/比例误差，并标定加速度计矩阵与零偏 | 六轴臂慢速覆盖各方向约 40 秒；手套保持刚性，手指不做屈伸 | `MCAL_BEGIN (0x10)` -> `MCAL_COMMIT (0x11)` | `type=7` 中的 M/W 矩阵、bias、RMS、每颗 IMU 质量结果 |

## 1. 对接范围与边界

自动标定系统由三个独立部分组成：

```text
六轴机械臂控制程序 ──(机械臂厂商 SDK/API)──> 六轴机械臂
             │
             │ 记录机械臂 TCP 位姿与主机时间
             ▼
自动标定主程序 ──(USB CDC / Bulk)──> 动捕手套
             │                         │
             │                         ├─ 上送：位姿、JPEG、ACK、M 标定报告
             │                         └─ 下发：IMU M 标定控制命令
             ▼
        保存视频、原始帧、日志与标定结果
```

- 手套接口负责**读取动捕数据**和**写入标定控制命令**。
- 六轴机械臂的使能、回零、轨迹执行、TCP 位姿读取、急停和安全限制，必须使用机械臂厂家提供的 SDK/API。
- 自动标定主程序应记录同一主机时钟下的“机械臂 TCP 位姿”和“手套数据到达时刻”，用于离线对齐。

## 2. USB 连接与传输层

### 2.1 连接方式

| 项目 | 定义 |
|---|---|
| USB 类型 | USB High-Speed，CDC-ACM 虚拟串口 / Bulk 管道 |
| 上行 | 设备 -> 主机，Bulk IN `0x82` |
| 下行 | 主机 -> 设备，Bulk OUT `0x03` |
| 开始上送条件 | 主机打开虚拟串口或 claim USB 接口后，设备主动开始推送 |
| 虚拟串口波特率 | CDC Bulk 不是真 UART，波特率设置被设备忽略 |

### 2.2 上行数据帧头

设备所有上行数据都使用 8 字节传输层头：

```text
offset  size  field
0       1     0x55
1       1     0xAA
2       1     type
3       1     seq              # 每个 type 独立计数，uint8 回绕
4       4     payloadLength    # uint32 little-endian
8       N     payload
```

解析要求：

1. 在字节流中寻找 `55 AA`。
2. 按 `payloadLength` 等待完整负载，不以读取次数作为帧边界。
3. 只在长度或校验失败时重新找同步。
4. 丢包统计必须按 `type` 分开维护 `last_seq[type]`，不能把摄像头帧、ACK 和关节帧混为同一个序号流。

### 2.3 可读取的上行数据类型

| type | 名称 | 是否可获取 | 自动标定用途 |
|---|---|---:|---|
| `2` | 摄像头帧 | 是 | 记录测试视频或视觉对照；负载为完整 JPEG |
| `5` | 21 关节位姿 | 是，默认持续输出 | 六轴臂姿态与手套输出的主要对比数据 |
| `6` | 校准 ACK / STATUS | 是 | 确认命令执行、读取姿态标定与 M 标定状态 |
| `7` | M 标定详细报告 | 是，`MCAL_COMMIT` 后输出 | 获取每颗 IMU 的 M/W 矩阵与质量指标 |

## 3. 可读取数据一：摄像头帧 `type=2`

| 项目 | 定义 |
|---|---|
| 负载内容 | 一整帧 JPEG 字节流 |
| JPEG 边界 | `FF D8` 开头，`FF D9` 结束 |
| 默认分辨率 | QVGA `320 x 240` |
| 可配置分辨率 | HVGA `480 x 320`、VGA `640 x 480` |
| 单帧大小 | 通常约 10-60 KB，取决于图像内容与分辨率 |

自动标定程序可选择保存 `type=2` JPEG，用于记录机械臂执行动作时的实际画面。不要假定 JPEG 与关节帧具有相同序号或一一对应关系；两类 `seq` 独立计数。

## 4. 可读取数据二：21 关节位姿 `type=5`

### 4.1 数据定位

| 项目 | 定义 |
|---|---|
| 外层 `type` | `5` |
| 外层 `payloadLength` | 固定 `1100` 字节 |
| 负载内同步字 | `AA 55` |
| 关节数 | 固定 `21` |
| 节点记录长度 | 每节点固定 `52` 字节 |
| 校验 | `sum(payload[0..1097]) & 0xFFFF`，小端保存于最后 2 字节 |

负载布局：

```text
0..1       AA 55
2          jointCount = 21
3          innerSeq
4          flags
5          reserved
6..1097    joints[21]，每个 52 字节
1098..1099 checksum u16 LE
```

### 4.2 位姿状态 `flags`

| 位 | 名称 | 含义 |
|---|---|---|
| bit0 | isUpdate | 恒为 1 |
| bit1 | calibrated | `1` = 已完成姿态校准，输出可作为有效绝对位姿使用 |
| bit2 | recalibrationRecommended | `1` = 设备建议重新校准 |
| bit4 | fusionMode | `0` = RAW + Mahony 融合模式 |

自动标定程序应将 bit1、bit2 与每帧位姿数据一起保存。`bit1=0` 时位姿仍会输出，但基坐标零位尚未锁定，只能作为预览数据。

### 4.3 单关节记录 `Joint[52]`

所有数值为 `float32`、小端、IEEE-754：

| 相对偏移 | 字段 | 单位 / 定义 |
|---:|---|---|
| `+0` | `position.x` | 米 |
| `+4` | `position.y` | 米 |
| `+8` | `position.z` | 米 |
| `+12` | `rotation.w` | 单位四元数，**w 在前** |
| `+16` | `rotation.x` | 单位四元数 |
| `+20` | `rotation.y` | 单位四元数 |
| `+24` | `rotation.z` | 单位四元数 |
| `+28` | `euler.yaw` | 度，绕 Z；由四元数换算 |
| `+32` | `euler.pitch` | 度，绕 Y；由四元数换算 |
| `+36` | `euler.roll` | 度，绕 X；由四元数换算 |
| `+40` | `scale.x` | 当前通常为 1.0 |
| `+44` | `scale.y` | 当前通常为 1.0 |
| `+48` | `scale.z` | 当前通常为 1.0 |

### 4.4 21 节点顺序

```text
0  WRIST
1  THUMB_CMC      2  THUMB_MCP      3  THUMB_IP       4  THUMB_TIP
5  INDEX_MCP      6  INDEX_PIP      7  INDEX_DIP      8  INDEX_TIP
9  MIDDLE_MCP    10  MIDDLE_PIP    11  MIDDLE_DIP    12  MIDDLE_TIP
13 RING_MCP      14  RING_PIP      15  RING_DIP      16  RING_TIP
17 PINKY_MCP     18  PINKY_PIP     19  PINKY_DIP     20  PINKY_TIP
```

`*_TIP` 为 FK 末端点，无独立 IMU；其 rotation 跟随末节骨。

### 4.5 坐标系与姿态约定

- 坐标系为**右手系**。
- 协议固定基坐标系中：X 约指向指尖，Y 约横跨掌面，Z 约指向掌背法线。
- `rotation` 是权威姿态数据，格式为 Hamilton 单位四元数 `(w,x,y,z)`。
- `euler` 只是四元数按 ZYX / yaw-pitch-roll 换算出的便捷显示值；自动标定程序应优先使用四元数。
- r46 及以后，21 个节点的 `position` 和 `rotation` 都已处于同一个固定基坐标系，**可直接使用，不应再次乘掌心四元数或重复做 FK**。
- 若需要计算父子关节相对姿态：

```text
q_relative = conjugate(q_parent_world) ⊗ q_child_world
```

- 若需要得到相对掌心的姿态：

```text
q_hand_relative[i] = conjugate(q_wrist) ⊗ q_node[i]
```

> 兼容提醒：r45 及以前存在“WRIST 绝对、手指相对 WRIST”的历史输出方式。自动标定程序必须记录实际固件版本或固件编译配置；当前协议没有定义独立的固件版本查询命令，见第 9 节。

## 5. 可读取数据三：M 标定命令应答 `type=6`

### 5.1 ACK 负载格式

`type=6` 的负载固定为 8 字节：

```text
offset  size  field
0       1     cmd       # 被处理的命令码
1       1     status
2       1     detail0
3       1     detail1
4       2     seq       # uint16 LE，COMMIT / STATUS 时有意义
6       2     reserved
```

对于 M 标定，自动化程序应以 `cmd=0x10/0x11/0x12` 对应的 ACK 为准。`MCAL_COMMIT` 成功后，紧接着继续接收 `type=7` 详细报告。

常用状态码：

| status | 名称 | 含义 |
|---:|---|---|
| `0` | OK | 成功 |
| `1` | BUSY | 当前 M 标定会话忙，或设备尚未准备好接收命令 |
| `3` | NO_PALM | 掌心 IMU 缺失 |
| `5` | FLASH_FAIL | Flash 写入或回读校验失败 |
| `11` | MCAL_INSUFF | M 标定有效运动不足 |

## 6. 可读取数据四：M 标定详细报告 `type=7`

### 6.1 何时获取

下发 `MCAL_COMMIT (0x11)` 后，设备先返回 `type=6` ACK，随后紧接着发送 `type=7` 详细报告。自动标定程序必须在收到 COMMIT ACK 后继续读取，不能在 ACK 到达后立即停止接收。

### 6.2 报告头

| 项目 | 定义 |
|---|---|
| 外层 `type` | `7` |
| 负载长度 | 固定 `1046` 字节 |
| 字节序 | 小端 |
| 报告版本 | 当前为 `2`，表示包含加速度计段 |
| IMU 数量 | 当前为 `11` |

| 偏移 | 字段 | 含义 |
|---:|---|---|
| 0 | context | 固定 `0x11`，表示来自 `MCAL_COMMIT` |
| 1 | ver | 报告版本，当前 `2` |
| 2 | nImu | 当前 `11` |
| 3 | nCal | 陀螺成功标定的 IMU 数量 |
| 4..5 | seq | 本次写入的 Flash 记录号，u16 LE |
| 6 | status | `0` OK / `11` 采集不足 / `5` Flash 失败 |
| 7 | flags | bit0=附带陀螺 M，bit1=附带加速度计段 |
| 8..9 | meanRms | 陀螺平均拟合残差，毫度；`2500 = 2.5°` |
| 10 | nBadOff | 陀螺交叉轴大于约 `1.7°` 的 lane 数 |

### 6.3 每颗 IMU 的可读取结果

| 数据 | 偏移 / 数量 | 说明 |
|---|---|---|
| 陀螺质量块 | `12 + i*8`，共 11 块 | `ok`、RMS、参与窗口数、最大非对角偏移 |
| 陀螺 M 矩阵 | `100 + i*36`，共 11 块 | `9 x float32` 行主序，`w_true = M * w_raw` |
| 加速度计质量块 | `496 + i*14`，共 11 块 | `ok`、RMS、比例误差、交叉轴、三轴 bias |
| 加速度计 W 矩阵 | `650 + i*36`，共 11 块 | `9 x float32` 行主序，`a_true = W * (a_raw - bias)` |

建议质量判据：

- 陀螺 `ok=1` 且 RMS 小于 `3000` 毫度（约 `3°`）为较好。
- RMS 大于 `5000` 毫度（约 `5°`）建议重新采集。
- `nBadOff > 0` 说明全方位翻滚不足、动作过快或个别 IMU 轴对齐存在异常。

### 6.4 M 标定界面字段与协议来源对照

下表对应“陀螺 M 标定 (v2)”界面中需要显示的信息：

| 界面信息 | 是否可从设备获得 | 协议来源 | 说明 |
|---|---|---|---|
| 标定状态：就绪、采集中、完成、失败 | 部分可获得 | `type=6` ACK | `MCAL_BEGIN` 成功后主机显示“采集中”；`MCAL_COMMIT` 的 ACK / `type=7.status` 决定成功、采集不足或 Flash 失败。`就绪`是主机自身状态。 |
| 进度条百分比 | **不能实时从设备获得** | 无持续进度字段 | 当前只能由自动标定程序根据“开始命令后的已运行时间 / 40 秒”估算。它不是设备已采样窗口数的真实百分比。 |
| 标定结果摘要 | 是 | `type=7` 头部 | 可显示报告版本 `ver`、成功 IMU 数 `nCal`、总 IMU 数 `nImu`、记录号 `seq`、平均残差 `meanRms`、异常 lane 数 `nBadOff`。 |
| Lane | 是 | `type=7`，共 11 组 | 物理 IMU 编号 `0..10`。 |
| 状态 | 是 | 陀螺质量块 `ok` | `1` 表示该 IMU 本次 M 标定有效；`0` 表示该 IMU 未通过。 |
| 残差(度) | 是 | 陀螺质量块 `rms` | 原始单位为毫度，界面显示时除以 `1000`，例如 `2500` 显示为 `2.5°`。 |
| 窗口数 | 是 | 陀螺质量块 `nseg` | 参与该 IMU 求解的有效运动窗口数量。 |
| 最大偏差 | 是 | 陀螺质量块 `maxoff` | 最大非对角项的缩放值，已包含 A/G 轴不对齐影响；按现有判据，`<16` 较好、`16..30` 可用、`>30` 建议重标。 |
| M 矩阵、W 矩阵、bias | 是 | `type=7` | 分别用于陀螺修正和加速度计修正，适合保存到测试记录中，不建议直接让操作员手工修改。 |

> 注意：界面中的 `v2` 是 **M 标定报告格式版本**（`type=7.ver`），不是设备固件版本。当前协议没有独立的固件版本读取命令。

## 7. 可写控制接口

### 7.1 命令帧通用格式

所有命令从主机写入 CDC Bulk OUT：

```text
offset  size  field
0       1     0xA5
1       1     0x5A
2       1     cmd
3       1     arg
4       2     payloadLength      # uint16 LE
6       N     payload
6+N     2     crc16              # CRC16-CCITT，uint16 LE
```

CRC16 覆盖 `[0 .. 6+N-1]`，多项式 `0x1021`，初值 `0xFFFF`。调试阶段允许将 CRC 字段写为 `00 00`，设备会跳过 CRC 校验；正式自动化程序应使用真实 CRC。

```python
import struct

def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc

def build_command(cmd: int, arg: int = 0, payload: bytes = b"") -> bytes:
    body = bytes((0xA5, 0x5A, cmd & 0xFF, arg & 0xFF)) + struct.pack("<H", len(payload)) + payload
    return body + struct.pack("<H", crc16_ccitt(body))
```

### 7.2 M 标定命令

| cmd | 命令 | 用途 |
|---:|---|---|
| `0x10` | `MCAL_BEGIN` | 清零累加器，开始采集陀螺与加速度计样本 |
| `0x11` | `MCAL_COMMIT` | 求解每颗 IMU 的陀螺 M、加速度计 W/bias，写 Flash 并立即生效 |
| `0x12` | `MCAL_ABORT` | 放弃本次 M 标定 |

六轴臂执行 M 标定的建议流程：

1. 手套或模型手保持刚性，手指不做屈伸动作。
2. 下发 `MCAL_BEGIN`，等待 `type=6 / status=0`。
3. 六轴臂以低动态加速度、慢速连续覆盖 pitch / roll / yaw 多个方向，使重力矢量在各 IMU 局部坐标中充分变化。建议约 40 秒。
4. 下发 `MCAL_COMMIT`，等待 `type=6` ACK。
5. 继续读取紧随其后的 `type=7` 报告，检查 11 颗 IMU 的 `ok`、RMS、矩阵与偏移质量。
6. 下发 `STATUS`，确认 ACK 的 `detail0.bit1=1`，表明 M 标定已加载并生效。

> M 标定采集期间，关节数据流通常仍可读取；`MCAL_COMMIT` 写 Flash 的几十至数百毫秒内可能短暂无响应。

## 8. 六轴臂自动标定程序建议采集记录

每个实验 / 自动轨迹应保存以下信息：

| 类别 | 必须记录 |
|---|---|
| 手套数据 | 原始完整 `type=5` payload、外层 type/seq、主机接收时间 |
| M 标定状态 | `MCAL_BEGIN / MCAL_COMMIT` 的 `type=6` ACK、M 标定记录 seq、失败状态码 |
| M 标定结果 | `type=7` 完整 payload、每颗 IMU 的质量字段与 M/W 矩阵 |
| 机械臂数据 | TCP 位姿、关节角、轨迹段编号、主机时间、机械臂控制器时间（如可取） |
| 版本与配置 | 固件版本、协议版本、上位机版本、硬件版本、IMU 安装版本、模型手版本 |
| 视频 | 摄像头 JPEG 或外部相机视频，以及对应主机时间基准 |

## 9. 当前协议没有提供、需补充确认的接口

以下字段或能力在当前 `USB_HOST_PROTOCOL.md` 中**没有定义为可读取接口**。六轴臂自动标定若需要这些数据，应由固件和协议新增：

| 缺失项 | 当前状态 | 建议 |
|---|---|---|
| 固件版本查询 | 未定义 `GET_VERSION` / 版本 `type` 帧 | 增加设备信息查询命令，返回固件版本、Git 提交号、构建日期、协议版本、硬件版本 |
| 设备时间戳 | `type=5` 仅有 uint8 `seq`，无设备时间戳 | 增加 `timestamp_us` 或统一时钟同步机制；当前只能使用主机接收时间对齐 |
| 原始 IMU 流 | 未定义加速度、角速度、磁场、每颗原始 IMU 四元数的持续上送 | 若自动标定算法需要自主解算，应新增 RAW_IMU 数据类型 |
| M 标定实时进度 | 未定义有效窗口数或完成百分比的持续上送 | 若六轴臂程序需要依据真实采样覆盖度自动结束，应新增 `MCAL_PROGRESS` 状态帧，至少返回每颗 IMU 的 `nseg`、覆盖度和采集状态 |
| 六轴臂控制 | 不属于手套协议 | 由六轴臂厂家 SDK 提供运动、TCP 位姿、回零、安全和急停接口 |

## 10. 最小联调验收顺序

1. 打开 USB 连接，连续收到 `type=5`，关节数为 21，checksum 正确。
2. 执行 M 标定：`MCAL_BEGIN -> 六轴臂慢速全方位翻滚 -> MCAL_COMMIT`。
3. 收到 `type=6` 成功 ACK 后，继续接收并保存 `type=7`，确认 11 颗 IMU 都有有效结果。
4. 对每颗 IMU 检查 `ok`、RMS、交叉轴偏差、M/W 矩阵和加速度计 bias；若收到 `MCAL_INSUFF`，扩大运动方向并放慢速度后重新执行。
5. 执行六轴臂固定角度与连续轨迹，保存机械臂 TCP 位姿、手套 `type=5`、主机时间和视频。

## 11. 协议来源

- `USB_HOST_PROTOCOL.md`：关节位姿、校准控制、M 标定和报告的权威定义。
- `USB_FIRMWARE_UPDATE.md`：固件升级协议；与六轴臂自动标定的运行时读写流程无直接依赖。
