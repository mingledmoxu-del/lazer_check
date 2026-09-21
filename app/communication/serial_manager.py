"""异步串口通信管理模块 (基于 pyserial + QThread).

运行于独立工作线程, 保证任何串口阻塞式 I/O 均不影响 GUI 界面流畅度.
具备自动重连、异常捕获、优雅退出、以及内置离线模拟仿真机 (Simulator) 模式.
"""

import threading
import time
from typing import List, Optional
import serial
import serial.tools.list_ports
from PySide6.QtCore import QObject, Signal, QThread
from communication.protocol import (
    Command,
    StatusCode,
    RequestPacket,
    ResponsePacket,
    parse_response_frame,
    CURRENT_FIRMWARE_COMMANDS,
)
import struct


class SerialReaderThread(QThread):
    """串口后台异步读取线程."""

    data_received = Signal(bytes)
    data_sent = Signal(bytes)
    error_occurred = Signal(str)
    connection_lost = Signal(str)

    def __init__(self, ser: serial.Serial):
        super().__init__()
        self._ser = ser
        self._running = True
        self._write_lock = threading.Lock()

    def stop(self) -> None:
        """停止线程."""
        self._running = False
        self.wait(1000)

    def send_data(self, data: bytes) -> bool:
        """线程安全地发送数据."""
        if not self._ser or not self._ser.is_open:
            return False
        try:
            with self._write_lock:
                self._ser.write(data)
                self._ser.flush()
            self.data_sent.emit(data)
            return True
        except Exception as e:
            self.error_occurred.emit(f"发送数据失败: {e}")
            return False

    def run(self) -> None:
        """线程执行循环, 持续读取串口数据."""
        while self._running:
            if not self._ser or not self._ser.is_open:
                break
            try:
                # 等待并读取可用数据
                waiting = self._ser.in_waiting
                if waiting > 0:
                    chunk = self._ser.read(waiting)
                    if chunk:
                        self.data_received.emit(chunk)
                else:
                    time.sleep(0.005)
            except (serial.SerialException, OSError) as e:
                if self._running:
                    self.connection_lost.emit(f"串口异常断开: {e}")
                break
            except Exception as e:
                if self._running:
                    self.error_occurred.emit(f"读取串口数据异常: {e}")
                time.sleep(0.01)


class VirtualStm32Simulator:
    """内置虚拟 STM32 仿真器 (用于无硬件时的联调与界面验证).

    完整模拟 STM32H743 对 Motor 1 (Slave 1) 与 Motor 2 (Slave 2) 的控制与寄存器响应.
    """

    def __init__(self, full_mode: bool = True):
        """创建仿真器；默认与当前物理固件一致，支持完整控制命令。

        ``full_mode=False`` 可用于兼容旧的 PA-only 固件联调。
        """
        self.full_mode = full_mode
        # 模拟两个电机的 PA 参数字典
        self.motor_params = {
            1: {1: 60, 21: 60, 53: 1, 71: 1, 72: 96, 73: 0},
            2: {1: 60, 21: 60, 53: 1, 71: 2, 72: 96, 73: 0},
        }
        self.motor_states = {
            1: {"servo_on": False, "speed": 0, "pos": 1000, "alarm": 0},
            2: {"servo_on": False, "speed": 0, "pos": 2000, "alarm": 0},
        }

    def process_request(self, raw_data: bytes) -> Optional[bytes]:
        """解析上位机发给 STM32 的帧并生成模拟应答帧."""
        # 简单检查帧头与长度
        if len(raw_data) < 7 or raw_data[0] != 0xAA or raw_data[1] != 0x55:
            return None

        dev_id = raw_data[3]
        cmd = raw_data[4]
        param = struct.unpack(">H", raw_data[5:7])[0]

        if dev_id not in (1, 2):
            return self._build_resp(dev_id, cmd, StatusCode.SERVO_NO_RESPONSE, param, b"")

        if not self.full_mode and cmd not in CURRENT_FIRMWARE_COMMANDS:
            return self._build_resp(dev_id, cmd, StatusCode.UNKNOWN_CMD, param, b"")

        resp_status = StatusCode.SUCCESS
        resp_data = bytearray()

        if cmd == Command.WRITE_PA:
            # 负载: save_mode(1B) + value(2B) 或者 直接 value(2B)
            val = 0
            if len(raw_data) >= 12:  # 带 save_mode
                val = struct.unpack(">h", raw_data[8:10])[0]
            elif len(raw_data) >= 11:
                val = struct.unpack(">h", raw_data[7:9])[0]
            self.motor_params[dev_id][param] = val
            resp_data.extend(struct.pack(">h", val))

        elif cmd == Command.READ_PA:
            val = self.motor_params[dev_id].get(param, 0)
            resp_data.extend(struct.pack(">h", val))

        elif cmd == Command.SERVO_ON:
            self.motor_states[dev_id]["servo_on"] = True
            resp_data.append(0x01)

        elif cmd == Command.SERVO_OFF:
            self.motor_states[dev_id]["servo_on"] = False
            self.motor_states[dev_id]["speed"] = 0
            resp_data.append(0x00)

        elif cmd == Command.START:
            if self.motor_states[dev_id]["servo_on"]:
                self.motor_states[dev_id]["speed"] = 1500 if dev_id == 1 else 1200
            resp_data.append(0x01)

        elif cmd == Command.STOP or cmd == Command.EMERGENCY_STOP:
            self.motor_states[dev_id]["speed"] = 0
            resp_data.append(0x00)

        elif cmd == Command.READ_SPEED:
            speed = self.motor_states[dev_id]["speed"]
            run_state = 3 if self.motor_states[dev_id]["servo_on"] else 1
            resp_data.extend(struct.pack(">hH", speed, run_state))

        elif cmd == Command.READ_POSITION:
            self.motor_states[dev_id]["pos"] += self.motor_states[dev_id]["speed"] // 10
            pos = self.motor_states[dev_id]["pos"]
            resp_data.extend(struct.pack(">i", pos))

        elif cmd == Command.READ_ALARM:
            alarm = self.motor_states[dev_id]["alarm"]
            resp_data.extend(struct.pack(">H", alarm))

        elif cmd == Command.RESET_ALARM:
            self.motor_states[dev_id]["alarm"] = 0
            resp_data.append(0x00)

        elif cmd == Command.SET_SPEED:
            # 负载: 目标速度 (2 字节大端整数)
            if len(raw_data) >= 9:
                speed = struct.unpack(">h", raw_data[7:9])[0]
                self.motor_states[dev_id]["speed"] = speed
                resp_data.extend(struct.pack(">h", speed))
            else:
                resp_status = StatusCode.ILLEGAL_PARAM

        elif cmd == Command.MOVE_RELATIVE:
            # 负载: 运行速度 (2B) + 相对脉冲数 (4B)
            if len(raw_data) >= 13:
                spd, pulses = struct.unpack(">hi", raw_data[7:13])
                self.motor_states[dev_id]["speed"] = spd
                self.motor_states[dev_id]["pos"] += pulses
                resp_data.extend(struct.pack(">i", self.motor_states[dev_id]["pos"]))
            else:
                resp_status = StatusCode.ILLEGAL_PARAM

        elif cmd == Command.MOVE_ABSOLUTE:
            # 负载: 运行速度 (2B) + 目标绝对脉冲位置 (4B)
            if len(raw_data) >= 13:
                spd, target_pos = struct.unpack(">hi", raw_data[7:13])
                self.motor_states[dev_id]["speed"] = spd
                self.motor_states[dev_id]["pos"] = target_pos
                resp_data.extend(struct.pack(">i", target_pos))
            else:
                resp_status = StatusCode.ILLEGAL_PARAM

        elif cmd == Command.JOG:
            # 负载: 方向 (1B, 1=正, 0=反) + 速度 (2B) + 步进脉冲数 (4B)
            if len(raw_data) >= 14:
                direction = raw_data[7]
                spd, step_pulses = struct.unpack(">hi", raw_data[8:14])
                delta = step_pulses if direction == 1 else -step_pulses
                self.motor_states[dev_id]["speed"] = spd
                self.motor_states[dev_id]["pos"] += delta
                resp_data.extend(struct.pack(">i", self.motor_states[dev_id]["pos"]))
            else:
                resp_status = StatusCode.ILLEGAL_PARAM
        else:
            resp_status = StatusCode.UNKNOWN_CMD

        return self._build_resp(dev_id, cmd, resp_status, param, resp_data)

    def _build_resp(self, dev_id: int, cmd: int, status: int, param: int, data: bytes | bytearray) -> bytes:
        from utils.crc import append_crc16
        payload = bytearray()
        payload.append(dev_id & 0xFF)
        payload.append(cmd & 0xFF)
        payload.append(status & 0xFF)
        payload.extend(struct.pack(">H", param & 0xFFFF))
        payload.extend(data)

        length_field = len(payload) + 2
        raw = bytearray([0xAA, 0x55, length_field & 0xFF])
        raw.extend(payload)
        return append_crc16(raw)


class SerialManager(QObject):
    """串口通信管理器, 提供设备扫描、连接管理与收发信号绑定."""

    connected = Signal(str)            # 连接成功 (参数为端口名称)
    disconnected = Signal()            # 连接已关闭
    data_received = Signal(bytes)      # 收到下位机数据流 (含模拟数据)
    data_sent = Signal(bytes)          # 已向串口发送数据流
    error_occurred = Signal(str)       # 错误发生

    def __init__(self, parent: Optional[QObject] = None):
        super().__init__(parent)
        self._serial: Optional[serial.Serial] = None
        self._reader_thread: Optional[SerialReaderThread] = None
        self._is_simulator = False
        self._simulator = VirtualStm32Simulator()
        self._current_port = ""

    @property
    def is_open(self) -> bool:
        """检查当前串口是否打开 (包含模拟器模式)."""
        if self._is_simulator:
            return True
        return self._serial is not None and self._serial.is_open

    @property
    def port_name(self) -> str:
        return self._current_port

    @property
    def is_simulator(self) -> bool:
        return self._is_simulator

    @staticmethod
    def list_ports() -> List[str]:
        """获取系统当前可用的物理串口列表."""
        ports = []
        try:
            for p in serial.tools.list_ports.comports():
                ports.append(p.device)
        except Exception:
            pass
        return sorted(ports)

    def open_simulator(self) -> bool:
        """开启内置模拟仿真模式 (无需物理硬件)."""
        self.close()
        self._is_simulator = True
        self._current_port = "VIRTUAL_SIM"
        self.connected.emit("VIRTUAL_SIM (模拟器)")
        return True

    def open(
        self,
        port: str,
        baudrate: int = 115200,
        bytesize: int = 8,
        parity: str = "None",
        stopbits: float = 1,
        timeout: float = 0.05,
    ) -> bool:
        """打开指定串口.

        :param port: 串口号 (如 COM3)
        :param baudrate: 波特率
        :param bytesize: 数据位 (5, 6, 7, 8)
        :param parity: 校验位 ('None', 'Even', 'Odd', 'Mark', 'Space')
        :param stopbits: 停止位 (1, 1.5, 2)
        :param timeout: 超时时间 (秒)
        """
        self.close()

        if port == "VIRTUAL_SIM":
            return self.open_simulator()

        parity_map = {
            "None": serial.PARITY_NONE,
            "Even": serial.PARITY_EVEN,
            "Odd": serial.PARITY_ODD,
            "Mark": serial.PARITY_MARK,
            "Space": serial.PARITY_SPACE,
        }
        stopbits_map = {
            1: serial.STOPBITS_ONE,
            1.5: serial.STOPBITS_ONE_POINT_FIVE,
            2: serial.STOPBITS_TWO,
        }

        try:
            self._serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=bytesize,
                parity=parity_map.get(parity, serial.PARITY_NONE),
                stopbits=stopbits_map.get(stopbits, serial.STOPBITS_ONE),
                timeout=timeout,
            )
            self._is_simulator = False
            self._current_port = port

            # 启动后台读取线程
            self._reader_thread = SerialReaderThread(self._serial)
            self._reader_thread.data_received.connect(self.data_received)
            self._reader_thread.data_sent.connect(self.data_sent)
            self._reader_thread.error_occurred.connect(self.error_occurred)
            self._reader_thread.connection_lost.connect(self._handle_connection_lost)
            self._reader_thread.start()

            self.connected.emit(port)
            return True
        except Exception as e:
            self.error_occurred.emit(f"打开串口 {port} 失败: {e}")
            self.close()
            return False

    def close(self) -> None:
        """关闭串口和后台线程."""
        # 必须在清理底层句柄前记录状态，否则 is_open 会被清成 False，
        # 导致物理串口关闭时漏发 disconnected 信号。
        was_open = self.is_open or self._is_simulator
        if self._reader_thread:
            try:
                self._reader_thread.stop()
            except Exception:
                pass
            self._reader_thread = None

        if self._serial:
            try:
                if self._serial.is_open:
                    self._serial.close()
            except Exception:
                pass
            self._serial = None

        self._is_simulator = False
        self._current_port = ""
        if was_open:
            self.disconnected.emit()

    def send_bytes(self, data: bytes) -> bool:
        """向串口发送原始字节流."""
        if self._is_simulator:
            self.data_sent.emit(data)
            # 仿真异步应答 (微延迟 10ms 模拟通信时延)
            threading.Thread(target=self._sim_reply, args=(data,), daemon=True).start()
            return True

        if self._reader_thread and self._serial and self._serial.is_open:
            return self._reader_thread.send_data(data)
        else:
            self.error_occurred.emit("串口未打开, 无法发送数据")
            return False

    def _sim_reply(self, request_bytes: bytes) -> None:
        """模拟器延迟应答."""
        time.sleep(0.015)
        resp = self._simulator.process_request(request_bytes)
        if resp:
            self.data_received.emit(resp)

    def _handle_connection_lost(self, err_msg: str) -> None:
        """处理由于硬件拔出导致的意外断开."""
        self.error_occurred.emit(err_msg)
        self.close()
