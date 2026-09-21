"""CRC16-Modbus 计算模块.

用于 PC 与 STM32 之间二进制通信协议的完整性校验.
多项式: 0xA001 (反向多项式, 对应标准 0x8005)
初始值: 0xFFFF
低字节在先, 高字节在后 (Little Endian).
"""


def calculate_crc16(data: bytes | bytearray) -> int:
    """计算 Modbus CRC-16 校验码.

    :param data: 待校验的原始字节流
    :return: 16位无符号整数 CRC 值 (0x0000 - 0xFFFF)
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def append_crc16(data: bytes | bytearray) -> bytes:
    """在数据末尾附加 2 字节 CRC-16 (先低字节, 后高字节).

    :param data: 待计算的数据
    :return: 附加了 CRC16 的完整字节串
    """
    crc = calculate_crc16(data)
    crc_low = crc & 0xFF
    crc_high = (crc >> 8) & 0xFF
    return bytes(data) + bytes([crc_low, crc_high])


def verify_crc16(frame: bytes | bytearray) -> bool:
    """验证包含末尾 2 字节 CRC 的数据帧是否正确.

    :param frame: 包含末尾 2 字节 CRC 的完整帧 (长度至少为 2)
    :return: 验证通过返回 True, 否则返回 False
    """
    if len(frame) < 2:
        return False
    data = frame[:-2]
    expected_crc = calculate_crc16(data)
    actual_crc = frame[-2] | (frame[-1] << 8)
    return expected_crc == actual_crc
