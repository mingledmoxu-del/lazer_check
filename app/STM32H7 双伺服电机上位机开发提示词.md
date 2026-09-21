请帮我开发一套完整的 Windows PC 上位机软件，用于通过串口与 STM32H743 通信，并由 STM32H743 再通过 RS485 / Modbus RTU 控制两台 P100S 伺服驱动器。

## 一、系统总体架构

整个系统通信结构如下：

PC 上位机
↓
USB 转串口
↓
STM32H743
↓
RS485
↓
P100S 伺服驱动器 #1
↓
CN4 RJ45 串接
↓
P100S 伺服驱动器 #2 CN3

PC 不直接发送 P100S 的 Modbus RTU 数据，而是采用自定义串口协议向 STM32 发送高级控制命令。

STM32 负责：

1. 接收 PC 命令
2. 解析上位机协议
3. 根据目标电机地址生成对应的 Modbus RTU 指令
4. 通过 RS485 发送给对应 P100S 驱动器
5. 接收驱动器响应
6. 将执行结果重新返回 PC

两台伺服驱动器具有不同的 Modbus 从站地址，例如：

Motor 1：Slave ID = 1  
Motor 2：Slave ID = 2

---

# 二、开发环境

请使用：

Python 3.12

GUI：

PySide6

串口：

pyserial

软件运行平台：

Windows 10 / Windows 11

程序需要采用模块化设计，不要把所有代码写到一个 py 文件中。

推荐目录结构：

```text
servo_control_pc/
│
├── main.py
│
├── gui/
│   ├── main_window.py
│   ├── servo_panel.py
│   └── serial_panel.py
│
├── communication/
│   ├── serial_manager.py
│   ├── protocol.py
│   └── packet_parser.py
│
├── servo/
│   ├── servo_controller.py
│   └── servo_parameters.py
│
├── utils/
│   ├── crc.py
│   └── logger.py
│
└── config/
    └── settings.json
```

要求代码结构清晰，便于后期扩展激光检测仪、DI/DO、Ethernet 等设备。

---

# 三、主界面设计

软件主界面名称：

Servo Motor Control System

界面整体风格偏工业控制软件，简洁、清晰。

主界面分为以下几个区域。

## 1. 串口连接区域

显示：

串口号：
COM1 / COM2 / COM3...

波特率：
9600
19200
38400
57600
115200

数据位：
8

停止位：
1

校验：
None

按钮：

刷新串口

打开串口

关闭串口

连接成功后显示：

```text
Serial Status: Connected
```

连接失败显示：

```text
Serial Status: Disconnected
```

并使用明显的状态指示灯。

---

# 四、双伺服选择

增加：

```text
Servo Selection
```

下拉框：

```text
Motor 1
Motor 2
```

内部对应：

```text
Motor 1 → device_id = 1
Motor 2 → device_id = 2
```

后续所有控制操作均作用于当前选中的伺服电机。

---

# 五、PA 参数操作区域

这是软件目前最核心的功能。

界面显示：

```text
Servo Parameter
```

包含：

```text
Motor:
[Motor 1 ▼]

Parameter:
[PA21]

Value:
[60]

[Read Parameter]
[Write Parameter]
```

例如：

选择：

```text
Motor 1

PA21

60
```

点击：

```text
Write Parameter
```

PC 向 STM32 发送：

```text
Motor ID = 1
Command = WRITE_PA
PA Number = 21
Value = 60
```

STM32 最终通过 RS485 操作：

```text
Slave 1
PA21 = 60
```

读取参数时：

```text
Motor ID = 1
Command = READ_PA
PA Number = 21
```

STM32 返回：

```text
Motor 1
PA21
Value = 60
```

界面显示：

```text
PA21 = 60
Read Successful
```

---

# 六、PA 参数输入方式

Parameter 输入框允许以下格式：

```text
PA1
PA21
PA53
PA71
```

程序内部自动转换：

```text
PA21 → 21
PA53 → 53
```

不要要求用户输入 Modbus 地址。

因为具体：

```text
PA21 → Modbus register 0x0015
```

这个转换工作由 STM32 完成。

PC 只处理：

```text
PA编号
```

---

# 七、参数写入模式

增加两个写入模式：

```text
Permanent
Temporary
```

Permanent：

写入驱动器正常参数区，允许保存到 EEPROM。

Temporary：

运行过程中临时修改参数，不要求永久保存。

PC 协议中增加：

```text
save_mode
```

例如：

```text
0 = Temporary
1 = Permanent
```

界面使用：

```text
Write Mode:

○ Temporary
● Permanent
```

---

# 八、常用伺服参数

增加一个：

```text
Common Parameters
```

表格。

例如：

| Parameter | Description | Value |
|---|---|---|
| PA1 | 控制相关参数 | 60 |
| PA21 | 参数 | 60 |
| PA53 | 参数 | 1 |
| PA71 | RS485 Slave ID | 1 |
| PA72 | RS485 Baud Rate | 96 |
| PA73 | RS485 Protocol | 0 |

暂时不要自行猜测 P100S 参数具体含义。

servo_parameters.py 中建立参数字典，后期可以根据 P100S 手册补充。

例如：

```python
SERVO_PARAMETERS = {
    1: {
        "name": "PA1",
        "description": "",
    },

    21: {
        "name": "PA21",
        "description": "",
    }
}
```

---

# 九、基本伺服控制

预留以下控制按钮：

```text
Servo ON
Servo OFF

Start
Stop

Reset Alarm
Emergency Stop
```

命令编号暂定：

```text
0x01 WRITE_PA
0x02 READ_PA

0x03 SERVO_ON
0x04 SERVO_OFF

0x05 MOTOR_START
0x06 MOTOR_STOP

0x07 READ_SPEED
0x08 READ_POSITION

0x09 READ_ALARM
0x0A RESET_ALARM
```

命令编号必须集中放在 protocol.py 中定义，例如：

```python
class Command:
    WRITE_PA = 0x01
    READ_PA = 0x02

    SERVO_ON = 0x03
    SERVO_OFF = 0x04

    START = 0x05
    STOP = 0x06

    READ_SPEED = 0x07
    READ_POSITION = 0x08

    READ_ALARM = 0x09
    RESET_ALARM = 0x0A
```

方便后续与 STM32 固件保持一致。

---

# 十、PC 与 STM32 自定义通信协议

请设计一个简单、可靠、容易在 STM32 C 语言中解析的二进制通信协议。

暂定格式：

```text
Header
Length
Device ID
Command
Parameter
Data
CRC16
```

建议：

```text
Byte0      0xAA
Byte1      0x55

Byte2      Length

Byte3      Device ID

Byte4      Command

Byte5-6    Parameter

Byte7...   Data

Last-2     CRC Low
Last-1     CRC High
```

例如：

```text
AA 55 08 01 01 00 15 00 3C CRC_L CRC_H
```

含义：

```text
AA 55

帧头

01

Motor 1

01

WRITE_PA

0015

PA21

003C

Value = 60
```

注意：

PC 与 STM32 的协议与 STM32 到 P100S 的 Modbus RTU 是两套不同协议。

不要混淆。

---

# 十一、串口数据接收

串口接收绝对不能阻塞 GUI 主线程。

使用：

```text
QThread
```

或者：

```text
QSerialPort
```

实现异步接收。

必须解决 TCP/串口类似的拆包问题，包括：

半包

粘包

连续多包

错误数据

帧头错位

CRC错误

packet_parser.py 应该维护一个接收缓存：

```python
rx_buffer = bytearray()
```

每次收到数据：

```text
append
↓
搜索 AA 55
↓
检查 Length
↓
检查完整帧
↓
CRC校验
↓
解析
```

不要假设一次 serial.read() 就一定得到完整数据包。

---

# 十二、命令应答机制

PC 每次向 STM32 发送命令以后，需要等待 STM32 应答。

例如：

```text
PC

WRITE_PA
Motor 1
PA21
60

↓

STM32

执行

↓

P100S Motor1

执行成功

↓

STM32

回复 PC
```

STM32 回复：

```text
ACK
Motor ID = 1
Command = WRITE_PA
Status = SUCCESS
```

PC 显示：

```text
Motor 1
PA21 = 60

Write Successful
```

如果失败：

```text
Write Failed
```

错误信息包括：

```text
Timeout

CRC Error

Servo No Response

Illegal Parameter

RS485 Communication Error
```

---

# 十三、超时机制

每条命令需要设置：

```text
timeout = 1000 ms
```

如果 1 秒没有收到 STM32 回复：

显示：

```text
Communication Timeout
```

不要无限等待。

---

# 十四、状态监控

增加：

```text
Servo Monitor
```

显示：

```text
Motor 1

Status:
Servo ON

Speed:
1500 rpm

Position:
123456

Alarm:
None
```

以及：

```text
Motor 2

Status:
Servo ON

Speed:
1200 rpm

Position:
98765

Alarm:
None
```

支持定时轮询，例如：

```text
500 ms
```

轮询：

```text
Motor1 Speed

Motor1 Position

Motor1 Alarm

Motor2 Speed

Motor2 Position

Motor2 Alarm
```

不能在上一个指令尚未收到响应时立即发送下一条指令。

需要使用命令队列。

---

# 十五、通信日志

界面底部增加：

```text
Communication Log
```

显示时间：

```text
14:21:10.152
```

TX：

```text
TX >> AA 55 ...
```

RX：

```text
RX << AA 55 ...
```

例如：

```text
[14:21:10.152] TX >> AA 55 08 01 ...
[14:21:10.183] RX << AA 55 06 01 ...
```

增加按钮：

```text
Clear Log

Save Log
```

日志同时保存到：

```text
logs/
```

例如：

```text
2026-09-15.log
```

---

# 十六、双电机状态

主界面应该能够同时看到两台电机基本状态。

例如：

```text
────────────────────────────

Motor 1

Communication: Online

Servo: ON

Speed:
1500 rpm

Alarm:
None

────────────────────────────

Motor 2

Communication: Online

Servo: ON

Speed:
1500 rpm

Alarm:
None

────────────────────────────
```

---

# 十七、通信状态检测

需要判断：

```text
STM32 Online / Offline

Motor1 Online / Offline

Motor2 Online / Offline
```

如果连续多次读取失败：

例如：

```text
3次
```

则状态显示：

```text
Motor 1 Offline
```

---

# 十八、参数批量读取

增加：

```text
Read All Parameters
```

支持用户输入：

```text
PA1
PA21
PA53
PA71
PA72
PA73
```

点击：

```text
Read All
```

程序按照队列依次读取：

```text
PA1
↓
PA21
↓
PA53
↓
PA71
...
```

不要一次发送全部命令。

---

# 十九、参数批量写入

增加参数配置表。

例如：

```text
Parameter       Value

PA21            60

PA53            1

PA71            1
```

支持：

```text
Save Configuration
```

保存为：

```text
JSON
```

例如：

```json
{
    "motor1": {
        "PA21": 60,
        "PA53": 1
    },

    "motor2": {
        "PA21": 60,
        "PA53": 1
    }
}
```

然后支持：

```text
Load Configuration

Write All
```

---

# 二十、安全限制

Servo OFF

Stop

Emergency Stop

这些命令优先级应该高于普通参数读取。

如果出现：

```text
Servo Alarm
```

界面突出显示。

写参数前：

检查输入是否为合法整数。

不要允许：

```text
PAabc

空值

超过 int16 / uint16 范围的数据
```

实际参数范围以后根据 P100S 手册添加。

---

# 二十一、软件状态栏

底部状态栏显示：

```text
COM3 | 115200 | STM32 Connected | Motor1 Online | Motor2 Online
```

---

# 二十二、后续扩展

软件架构必须考虑以后增加：

```text
激光检测仪

DI输入

DO输出

自动检测流程

电机运动控制

数据采集

CSV数据保存

波形显示

Ethernet通信

TCP/IP

自动扇叶检测
```

因此不要让 Servo 功能和 GUI 严重耦合。

建议使用：

```text
GUI
↓
Controller
↓
Communication Manager
↓
Serial
```

架构。

---

# 二十三、代码要求

请严格按照以下要求开发：

1. 提供完整可运行代码，不要只给伪代码。

2. 每个 Python 文件给出完整内容。

3. 所有重要函数使用中文注释。

4. 使用面向对象设计。

5. GUI 与串口通信分离。

6. GUI 主线程绝对不能执行阻塞式 serial.read()。

7. CRC16 单独封装。

8. 串口协议单独封装。

9. Servo 控制逻辑单独封装。

10. 所有异常进行 try/except 处理。

11. 串口断开以后不能导致程序崩溃。

12. 支持重新连接串口。

13. 软件关闭时自动关闭串口和线程。

14. 不要自行假定 STM32 已经实现某些功能。

15. 对 PC 与 STM32 的协议进行清晰定义。

16. 在正式代码之前先给出整个软件架构设计。

---

# 二十四、第一阶段目标

第一阶段暂时不要实现复杂运动控制。

首先完成最小可运行版本：

```text
PC
↓
串口
↓
STM32
↓
RS485
↓
两台P100S
```

实现以下功能：

```text
① 搜索COM口

② 打开/关闭串口

③ Motor1 / Motor2选择

④ 输入PA参数号

⑤ 输入参数值

⑥ Read PA

⑦ Write PA

⑧ 显示STM32返回结果

⑨ TX/RX通信日志

⑩ CRC16

⑪ 通信超时

⑫ 两台电机地址区分
```

第一阶段稳定以后，再逐步增加：

```text
Servo ON/OFF

速度读取

位置读取

报警读取

运动控制

自动检测流程
```

---

请首先输出：

1. 整个上位机软件架构
2. PC ↔ STM32 通信协议定义
3. 项目目录
4. 第一阶段 GUI 界面设计
5. 第一阶段完整 Python 源码
6. requirements.txt
7. Windows 下运行方法
8. 如何与 STM32 联调

不要一次把后续所有复杂功能混在第一阶段代码中，优先保证第一阶段的 PA 参数读取、修改和双伺服选择功能稳定可靠。