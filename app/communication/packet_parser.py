"""串口二进制流分包与粘包解析器.

负责维护接收缓冲区, 处理半包、粘包、连续多包、干扰杂波、CRC错误与帧头对齐等问题.
保证只有经过严格 CRC16 校验的完整数据帧才会提交给上层处理.
"""

from typing import List
from communication.protocol import (
    FRAME_HEADER,
    ResponsePacket,
    parse_response_frame,
)
from utils.crc import verify_crc16


class PacketParser:
    """流式数据包解析器, 具有状态机式断帧与重同步能力."""

    def __init__(self, max_buffer_size: int = 4096):
        self._rx_buffer = bytearray()
        self._max_buffer_size = max_buffer_size
        self._crc_error_count = 0

    @property
    def crc_error_count(self) -> int:
        return self._crc_error_count

    def reset(self) -> None:
        """清空接收缓冲区与计数."""
        self._rx_buffer.clear()
        self._crc_error_count = 0

    def feed_data(self, chunk: bytes | bytearray) -> List[ResponsePacket]:
        """接收新到达的二进制字节流, 并解析出所有完整合法的数据包.

        :param chunk: 串口收到的数据片段
        :return: 成功解析出的合法 ResponsePacket 列表
        """
        if not chunk:
            return []

        self._rx_buffer.extend(chunk)

        # 保护机制: 避免缓冲区无限制增长
        if len(self._rx_buffer) > self._max_buffer_size:
            self._rx_buffer = self._rx_buffer[-self._max_buffer_size:]

        packets: List[ResponsePacket] = []

        while True:
            # 1. 寻找帧头 0xAA 0x55
            header_idx = self._rx_buffer.find(FRAME_HEADER)
            if header_idx == -1:
                # 没找到帧头, 但如果末尾是 0xAA, 保留它以防半个帧头
                if len(self._rx_buffer) > 0 and self._rx_buffer[-1] == 0xAA:
                    self._rx_buffer = self._rx_buffer[-1:]
                else:
                    self._rx_buffer.clear()
                break

            # 如果帧头前有杂散字节, 丢弃前面的噪声
            if header_idx > 0:
                del self._rx_buffer[:header_idx]

            # 2. 检查是否有足够的字节读取 Length 字段 (至少需要 3 字节: AA 55 Len)
            if len(self._rx_buffer) < 3:
                break

            length = self._rx_buffer[2]

            # 合法性粗检: 协议规定的合法负载范围 (例如 6 到 64 字节)
            # 最小应答: dev(1) + cmd(1) + status(1) + param(2) + crc(2) = 7
            if length < 7 or length > 64:
                # Length 异常, 说明该 0xAA 0x55 可能是随机噪声或错位帧头, 滑动跳过 1 字节
                del self._rx_buffer[:1]
                continue

            # 3. 完整帧长度 = 帧头(2) + Length字段(1) + length (包含CRC)
            total_frame_len = 3 + length

            # 检查当前缓冲区是否已收齐一整帧
            if len(self._rx_buffer) < total_frame_len:
                # 半包, 留待下次数据到来继续拼接
                break

            # 4. 提取一整帧候选数据
            candidate_frame = bytes(self._rx_buffer[:total_frame_len])

            # 5. CRC16 校验
            if verify_crc16(candidate_frame):
                # 校验通过, 移出缓冲区
                del self._rx_buffer[:total_frame_len]
                packet = parse_response_frame(candidate_frame)
                if packet is not None:
                    packets.append(packet)
            else:
                # CRC 校验失败, 记录并丢弃误匹配的帧头 (移出 2 字节以继续向后搜索)
                self._crc_error_count += 1
                del self._rx_buffer[:2]

        return packets
