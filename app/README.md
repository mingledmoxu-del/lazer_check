# Servo Motor Control System (双 P100S 伺服控制上位机)

针对 STM32H743 桥接 RS485 控制两台 P100S 伺服驱动器（Motor 1 与 Motor 2）的 Windows 专业 PC 上位机软件。

---

## 一、软件架构

系统采用分层解耦的工业控制软件架构：

```text
               +-------------------------------------------+
               |        GUI 表现层 (PySide6 UI)            |
               |  MainWindow / SerialPanel / ServoPanel    |
               +-------------------------------------------+
                                     │ Qt Signals / Slots
                                     ▼
               +-------------------------------------------+
               |       业务控制层 (ServoController)        |
               |  - 指令排队 FIFO Queue                    |
               |  - 1000ms 超时判定与重试控制              |
               |  - 双电机在线/离线判定 (3次失败标记离线)  |
               |  - 后台定时状态轮询 (500ms 速度/位置/报警)|
               |  - 批量参数读写调度                       |
               +-------------------------------------------+
                                     │ Request / Response
                                     ▼
               +-------------------------------------------+
               |       通信驱动层 (Communication)          |
               |  - SerialManager: QThread 异步读写        |
               |  - PacketParser: 状态机分包、粘包与CRC校验|
               |  - VirtualStm32Simulator: 离线仿真机      |
               +-------------------------------------------+
                                     │ 二进制数据流 (0xAA 0x55)
                                     ▼
                       +---------------------------+
                       |   底层物理接口 (RS232/USB) |
                       |        STM32H743          |
                       +---------------------------+
```

---

## 二、PC ↔ STM32 自定义二进制通信协议

PC 与 STM32 采用轻量高效的定界二进制数据帧，末尾统一追加 Modbus CRC-16 校验码（先低字节后高字节）。

### 1. 发送帧格式 (PC → STM32)

| 偏移 | 字段名称 | 字节数 | 说明 |
|---|---|---|---|
| `[0..1]` | 帧头 (Header) | 2 | 固定为 `0xAA 0x55` |
| `[2]` | 长度 (Length) | 1 | 本字节之后直到帧末尾（含CRC）的总字节数 |
| `[3]` | 设备地址 (Device ID) | 1 | `1` = Motor 1, `2` = Motor 2 |
| `[4]` | 命令字 (Command) | 1 | 详见命令字列表 |
| `[5..6]` | 参数编号 (Parameter) | 2 | 大端序，如 `0x0015` 表示 PA21 |
| `[7..]` | 附加数据 (Data) | 可变 | 如写参数时为 `[save_mode: 1B, value: 2B]` |
| `[-2..-1]` | CRC16 | 2 | Modbus CRC-16 (低字节在前，高字节在后) |

**写参数示例 (向 Motor 1 永久写入 PA21 = 60)**：
```text
AA 55 09 01 01 00 15 01 00 3C 11 50
- AA 55: 帧头
- 09: 长度 (后面 9 个字节: 1+1+2+1+2+2)
- 01: Motor 1
- 01: WRITE_PA
- 00 15: PA21
- 01: 写入模式 (1 = Permanent, 0 = Temporary)
- 00 3C: 数值 60
- 11 50: CRC16 (低字节 0x11, 高字节 0x50)
```

**读参数示例 (读取 Motor 1 的 PA21)**：
```text
AA 55 06 01 02 00 15 8C B5
- 06: 后面 6 个字节 (Dev 1B + Cmd 1B + Param 2B + CRC 2B)
- 01: Motor 1
- 02: READ_PA
- 00 15: PA21
- 8C B5: CRC16
```

### 2. 应答帧格式 (STM32 → PC)

| 偏移 | 字段名称 | 字节数 | 说明 |
|---|---|---|---|
| `[0..1]` | 帧头 (Header) | 2 | 固定为 `0xAA 0x55` |
| `[2]` | 长度 (Length) | 1 | 本字节之后包含 CRC 的总字节数 |
| `[3]` | 设备地址 (Device ID) | 1 | 电机 ID |
| `[4]` | 命令字 (Command) | 1 | 回传对应的请求命令字 |
| `[5]` | 执行状态 (Status) | 1 | `0x00` 成功，非 0 为错误码 |
| `[6..7]` | 参数编号 (Parameter) | 2 | 回传对应的 PA 号 |
| `[8..]` | 返回数据 (Data) | 可变 | 读回的参数值或实时监控数据 |
| `[-2..-1]` | CRC16 | 2 | Modbus CRC-16 |

### 3. 命令字定义 (Command)
- `0x01`: `WRITE_PA`
- `0x02`: `READ_PA`
- `0x03`: `SERVO_ON`
- `0x04`: `SERVO_OFF`
- `0x05`: `START`
- `0x06`: `STOP`
- `0x07`: `READ_SPEED`
- `0x08`: `READ_POSITION`
- `0x09`: `READ_ALARM`
- `0x0A`: `RESET_ALARM`
- `0x0B`: `EMERGENCY_STOP`
- `0x10`: `SET_SPEED` (设定目标转速 rpm)
- `0x11`: `MOVE_RELATIVE` (相对位置运动: 圈数/角度脉冲)
- `0x12`: `MOVE_ABSOLUTE` (绝对位置运动)
- `0x13`: `JOG` (点动步进运行)

### 4. 状态码定义 (StatusCode)
- `0x00`: `SUCCESS` (执行成功)
- `0x01`: `TIMEOUT` (Modbus从站超时)
- `0x02`: `CRC_ERROR` (CRC校验失败)
- `0x03`: `SERVO_NO_RESPONSE` (伺服驱动器无响应)
- `0x04`: `ILLEGAL_PARAM` (非法参数编号或越界)
- `0x05`: `RS485_ERROR` (总线冲突或物理故障)
- `0x06`: `BUSY` (系统忙)
- `0x07`: `UNKNOWN_CMD` (未知指令)

---

## 三、目录结构

```text
E:\Code\LightCheck\app\
│
├── main.py                     # 上位机程序启动入口
├── requirements.txt            # Python 依赖清单
├── start_app.bat               # Windows 一键启动脚本 (纯 ASCII 编码)
├── test_app.py                 # 自动化单元与模拟测试脚本
│
├── config/
│   ├── __init__.py
│   └── settings.json           # 默认波特率、轮询周期与路径配置
│
├── gui/
│   ├── __init__.py
│   ├── serial_panel.py         # 串口连接与状态指示面板
│   ├── motion_panel.py         # 运动控制、定角旋转、圈数与点动面板
│   ├── servo_panel.py          # PA 参数读写、常用表与批量配置面板
│   └── main_window.py          # 主界面 (整合双电机监控、控制区、日志与状态栏)
│
├── communication/
│   ├── __init__.py
│   ├── protocol.py             # 协议帧结构定义与编解码
│   ├── packet_parser.py        # 流式分包、粘包、断帧与CRC对齐解析器
│   └── serial_manager.py       # QThread 异步串口 I/O 管理器与内置仿真器
│
├── servo/
│   ├── __init__.py
│   ├── servo_parameters.py     # PA 参数字典映射与文本解析
│   └── servo_controller.py     # 调度中心 (指令队列、超时监测、离线判定、轮询)
│
└── utils/
    ├── __init__.py
    ├── crc.py                  # Modbus CRC-16 计算与校验
    └── logger.py               # TX/RX 报文格式化与日期日志保存
```

---

## 四、Windows 运行方法

### 1. 安装依赖
打开 PowerShell 或 CMD，进入 `app` 目录：
```cmd
cd E:\Code\LightCheck\app
pip install -r requirements.txt
```

### 2. 启动上位机
方式 1：双击运行 `start_app.bat`。  
方式 2：命令行运行：
```cmd
python main.py
```

### 3. 无硬件时的离线模拟测试
打开软件后，在顶部串口下拉框中选择 **`VIRTUAL_SIM (虚拟STM32)`** 并点击“打开串口”：
- 可无缝读写 PA1、PA21、PA53、PA71、PA72、PA73 等参数；
- 当前仿真器默认与已烧录 STM32 固件一致，支持 PA、使能、速度、位置、点动和状态读取；
- 如需兼容旧版 PA-only 固件，可显式创建 `VirtualStm32Simulator(full_mode=False)`；
- 可测试一键批量读取和 JSON 参数配置保存与加载；
- 底部通信日志将实时打印 TX / RX 完整十六进制帧。

### 4. 当前固件能力

当前 STM32H743 下位机固件已实现 `0x01~0x0B` 与 `0x10~0x13`：PA 参数读写、软件强制使能、速度启动/停止、速度/位置/报警读取、报警复位、急停、速度设定、相对/绝对位置和点动。上位机只在收到成功应答后标记对应驱动器在线。

速度运行使用 P100S 的 PA4/PA22/PA24，软件使能使用 PA53；位置运行使用内部位置 P1（P4-0、P4-2、P4-3、P4-4）并通过 P3-18 的 CTRG 强制输入触发。首次烧录后必须空载、低速验证，并保留独立硬件急停；串口急停不能替代硬接线急停。

使用 DB44 外部 SON 时应设置 `PA53=0、P3-0=1`。运动命令会读取 `0x1016` 的真实运行状态，值为 `3` 即认为外部使能已经生效，不要求先点击上位机 Servo ON。上位机 Servo ON 仅用于需要 `PA53=1` 的软件强制使能模式。

---

## 五、如何与 STM32 联调

1. **串口硬件连接**：
   - 将 USB 转 TTL 串口线连接到 STM32H743 的对应串口（TX 接 RX，RX 接 TX，GND 共地）。
   - PC 上位机默认使用 `115200` 波特率、8 数据位、1 停止位、无校验位（8N1）。
2. **STM32 接收解析**：
   - 收到 `0xAA 0x55` 开头的数据后，读取第 3 字节 `Length`，收齐 `3 + Length` 字节后验证末尾 2 字节 Modbus CRC16。
   - 读取 `Device ID`：
     - 若为 `1`，则向 RS485 从站地址 1 发送 Modbus RTU 读写帧；
     - 若为 `2`，则向 RS485 从站地址 2 发送 Modbus RTU 读写帧。
   - STM32 将 PA 编号映射为 Modbus 保持寄存器（例如 PA21 对应寄存器地址 `0x0015`）。
3. **STM32 应答回传**：
   - 收到 P100S 驱动器响应后，STM32 打包生成 `AA 55 [Length] [DevID] [Cmd] [Status] [Param] [Data] [CRC_L] [CRC_H]` 应答帧发回 PC。
   - 若 P100S 驱动器在 500ms 内未回复，STM32 返回 `Status = 0x03 (SERVO_NO_RESPONSE)`。
