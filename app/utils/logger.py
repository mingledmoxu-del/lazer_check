"""系统日志与通信日志管理模块.

负责将 TX/RX 数据帧格式化输出至界面, 并支持按日期保存到本地文件 logs/YYYY-MM-DD.log.
"""

from datetime import datetime
import os
from pathlib import Path


class CommLogger:
    """通信记录器, 负责格式化串口收发记录并写入日志文件."""

    def __init__(self, log_dir: str = "logs"):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)

    def _get_log_file(self) -> Path:
        """获取当前日期的日志文件路径."""
        today_str = datetime.now().strftime("%Y-%m-%d")
        return self.log_dir / f"{today_str}.log"

    def format_hex(self, data: bytes | bytearray) -> str:
        """将字节序列转为十六进制大写字符串 (带空格分隔)."""
        return " ".join(f"{b:02X}" for b in data)

    def log_tx(self, data: bytes | bytearray, note: str = "") -> str:
        """记录发送数据 (TX).

        :param data: 发送的字节序列
        :param note: 附加说明
        :return: 格式化后的日志字符串
        """
        now = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        hex_str = self.format_hex(data)
        msg = f"[{now}] TX >> {hex_str}"
        if note:
            msg += f" ({note})"
        self._write_file(msg)
        return msg

    def log_rx(self, data: bytes | bytearray, note: str = "") -> str:
        """记录接收数据 (RX).

        :param data: 接收到的字节序列
        :param note: 附加说明
        :return: 格式化后的日志字符串
        """
        now = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        hex_str = self.format_hex(data)
        msg = f"[{now}] RX << {hex_str}"
        if note:
            msg += f" ({note})"
        self._write_file(msg)
        return msg

    def log_info(self, text: str) -> str:
        """记录系统普通事件/提示信息."""
        now = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        msg = f"[{now}] [INFO] {text}"
        self._write_file(msg)
        return msg

    def log_warn(self, text: str) -> str:
        """记录警告信息."""
        now = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        msg = f"[{now}] [WARN] {text}"
        self._write_file(msg)
        return msg

    def log_error(self, text: str) -> str:
        """记录错误信息."""
        now = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        msg = f"[{now}] [ERROR] {text}"
        self._write_file(msg)
        return msg

    def _write_file(self, line: str) -> None:
        """向本地文件追加写入一行."""
        try:
            log_file = self._get_log_file()
            with open(log_file, "a", encoding="utf-8") as f:
                f.write(line + "\n")
        except Exception as e:
            print(f"[Logger Error] 无法写入日志文件: {e}")
