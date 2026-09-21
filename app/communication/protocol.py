"""PC 与 STM32 自定义二进制通信协议定义.

本协议采用定界符+长度+CRC16的严谨二进制帧结构.
帧结构说明:
-------------------------------------------------------------------------------------
发送帧 (PC -> STM32):
[0..1] 帧头: 0xAA 0x55 (2 字节)
[2]    长度 (Length): 1 字节 (表示 Length 字段之后直到帧末尾包含 CRC 的总字节数)
[3]    设备地址 (Device ID): 1 字节 (1 = Motor 1, 2 = Motor 2)
[4]    命令字 (Command): 1 字节
[5..6] 参数编号 (Parameter): 2 字节, 大端序 (如 0x0015 = 21, 表示 PA21)
[7..]  附加数据 (Data): 可变长度 (如写参数时为 [保存模式 1B, 参数值 2B])
[-2..] CRC16: 2 字节 (Modbus CRC-16, 低字节在前, 高字节在后)

-------------------------------------------------------------------------------------
应答帧 (STM32 -> PC):
[0..1] 帧头: 0xAA 0x55 (2 字节)
[2]    长度 (Length): 1 字节
[3]    设备地址 (Device ID): 1 字节
[4]    命令字 (Command): 1 字节
[5]    执行状态 (Status): 1 字节 (0x00 = 成功, 其它为错误码)
[6..7] 参数编号 (Parameter): 2 字节, 大端序 (回传对应 PA 号)
[8..]  返回数据 (Data): 可变长度 (如读取到的参数值、转速、位置等)
[-2..] CRC16: 2 字节 (Modbus CRC-16, 低字节在前, 高字节在后)
-------------------------------------------------------------------------------------
"""

from dataclasses import dataclass, field
import struct
from utils.crc import append_crc16, verify_crc16


class Command:
    """PC 与 STM32 控制命令定义."""
    WRITE_PA = 0x01        # 写伺服 PA 参数
    READ_PA = 0x02         # 读伺服 PA 参数
    SERVO_ON = 0x03        # 伺服使能 (ON)
    SERVO_OFF = 0x04       # 伺服去使能 (OFF)
    START = 0x05           # 启动电机运行
    STOP = 0x06            # 停止电机
    READ_SPEED = 0x07      # 读取当前转速 (rpm)
    READ_POSITION = 0x08   # 读取当前位置 (pulse)
    READ_ALARM = 0x09      # 读取报警码
    RESET_ALARM = 0x0A     # 复位报警
    EMERGENCY_STOP = 0x0B  # 紧急急停 (最高优先级)

    # 运动与位置控制指令
    SET_SPEED = 0x10       # 设定运行速度 (rpm)
    MOVE_RELATIVE = 0x11   # 相对位置运动 (圈数/角度折算脉冲)
    MOVE_ABSOLUTE = 0x12   # 绝对位置运动
    JOG = 0x13             # 点动步进运行

    _NAMES = {
        WRITE_PA: "WRITE_PA",
        READ_PA: "READ_PA",
        SERVO_ON: "SERVO_ON",
        SERVO_OFF: "SERVO_OFF",
        START: "START",
        STOP: "STOP",
        READ_SPEED: "READ_SPEED",
        READ_POSITION: "READ_POSITION",
        READ_ALARM: "READ_ALARM",
        RESET_ALARM: "RESET_ALARM",
        EMERGENCY_STOP: "EMERGENCY_STOP",
        SET_SPEED: "SET_SPEED",
        MOVE_RELATIVE: "MOVE_RELATIVE",
        MOVE_ABSOLUTE: "MOVE_ABSOLUTE",
        JOG: "JOG",
    }

    @classmethod
    def get_name(cls, cmd: int) -> str:
        return cls._NAMES.get(cmd, f"CMD_0x{cmd:02X}")


# 与当前 STM32 固件保持一致的完整命令集合。
CURRENT_FIRMWARE_COMMANDS = frozenset(Command._NAMES.keys())


def is_current_firmware_command_supported(command: int) -> bool:
    """判断命令是否由当前 STM32 固件实现。"""
    return command in CURRENT_FIRMWARE_COMMANDS


class SaveMode:
    """参数写入模式."""
    TEMPORARY = 0x00  # 临时写入 (掉电不保存, 适用于动态调试)
    PERMANENT = 0x01  # 永久写入 (保存至驱动器 EEPROM)

    @classmethod
    def get_name(cls, mode: int) -> str:
        return "Permanent(永久)" if mode == cls.PERMANENT else "Temporary(临时)"


class StatusCode:
    """STM32 回复的执行状态码."""
    SUCCESS = 0x00             # 执行成功
    TIMEOUT = 0x01             # 超时无应答
    CRC_ERROR = 0x02           # CRC校验错误
    SERVO_NO_RESPONSE = 0x03   # 伺服驱动器无响应 (Modbus从站未回复)
    ILLEGAL_PARAM = 0x04       # 非法参数编号或超出范围
    RS485_ERROR = 0x05         # RS485硬件通信故障/总线冲突
    BUSY = 0x06                # STM32正在处理前序指令
    UNKNOWN_CMD = 0x07         # 不支持的未知命令

    _MESSAGES = {
        SUCCESS: "执行成功 (SUCCESS)",
        TIMEOUT: "通信超时 (Timeout)",
        CRC_ERROR: "CRC校验错误 (CRC Error)",
        SERVO_NO_RESPONSE: "伺服驱动器无响应 (Servo No Response)",
        ILLEGAL_PARAM: "非法参数 (Illegal Parameter)",
        RS485_ERROR: "RS485通信故障 (RS485 Error)",
        BUSY: "STM32系统繁忙 (Busy)",
        UNKNOWN_CMD: "未知指令 (Unknown Command)",
    }

    @classmethod
    def get_description(cls, code: int) -> str:
        return cls._MESSAGES.get(code, f"未知错误代码 (0x{code:02X})")


FRAME_HEADER = bytes([0xAA, 0x55])
MIN_FRAME_LENGTH = 7  # 最小帧: AA 55 + len(1) + dev(1) + cmd(1) + crc(2)


@dataclass
class RequestPacket:
    """上位机发送给 STM32 的请求帧结构体."""
    device_id: int               # 目标电机 ID (1 或 2)
    command: int                 # 命令字 (如 Command.WRITE_PA)
    parameter: int = 0           # PA 参数编号 (如 21 对应 PA21)
    data: bytes = field(default_factory=bytes)  # 负载数据

    def build_frame(self) -> bytes:
        """构建待发送的完整二进制数据帧 (包含 AA 55、Length 和 CRC16)."""
        payload = bytearray()
        payload.append(self.device_id & 0xFF)
        payload.append(self.command & 0xFF)
        payload.extend(struct.pack(">H", self.parameter & 0xFFFF))
        payload.extend(self.data)

        # Length = payload 长度 + 2 字节 CRC
        length_field = len(payload) + 2
        raw_without_crc = bytearray()
        raw_without_crc.extend(FRAME_HEADER)
        raw_without_crc.append(length_field & 0xFF)
        raw_without_crc.extend(payload)

        # 追加 Modbus CRC16
        return append_crc16(raw_without_crc)


@dataclass
class ResponsePacket:
    """STM32 返回给上位机的应答帧结构体."""
    device_id: int               # 电机 ID
    command: int                 # 命令字
    status: int                  # 状态码 (StatusCode)
    parameter: int = 0           # PA 参数编号
    data: bytes = field(default_factory=bytes)  # 附加数据 (如读取的值)
    raw_frame: bytes = field(default_factory=bytes)  # 原始二进制帧

    @property
    def is_success(self) -> bool:
        return self.status == StatusCode.SUCCESS

    @property
    def status_desc(self) -> str:
        return StatusCode.get_description(self.status)

    def get_value_int16(self) -> int | None:
        """若数据为 2 字节, 解析为有符号 16 位整数."""
        if len(self.data) >= 2:
            return struct.unpack(">h", self.data[:2])[0]
        return None

    def get_value_uint16(self) -> int | None:
        """若数据为 2 字节, 解析为无符号 16 位整数."""
        if len(self.data) >= 2:
            return struct.unpack(">H", self.data[:2])[0]
        return None

    def get_value_int32(self) -> int | None:
        """若数据为 4 字节, 解析为 32 位有符号整数 (常用于位置)."""
        if len(self.data) >= 4:
            return struct.unpack(">i", self.data[:4])[0]
        return None


def parse_response_frame(frame: bytes | bytearray) -> ResponsePacket | None:
    """解析已完成校验的完整应答帧.

    帧格式:
    AA 55 | Length | Device ID | Command | Status | Parameter(2B) | Data(可变) | CRC_L CRC_H
    """
    if len(frame) < 9:  # 2 + 1 + 1 + 1 + 1 + 2 + 2(CRC) = 10 最小带 Status/Param 帧
        return None

    if frame[0] != 0xAA or frame[1] != 0x55:
        return None

    if not verify_crc16(frame):
        return None

    length = frame[2]
    # 检查总长度是否与帧长度一致 (2 + 1 + length)
    if len(frame) != length + 3:
        return None

    device_id = frame[3]
    command = frame[4]
    status = frame[5]
    parameter = struct.unpack(">H", frame[6:8])[0]
    # 数据区位于 8 到倒数第 2 字节 (即去掉末尾 2 字节 CRC)
    data = bytes(frame[8:-2])

    return ResponsePacket(
        device_id=device_id,
        command=command,
        status=status,
        parameter=parameter,
        data=data,
        raw_frame=bytes(frame)
    )
