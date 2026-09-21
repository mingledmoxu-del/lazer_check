"""伺服控制器核心业务逻辑模块.

负责管理 PC 发给 STM32 的所有指令队列、等待应答状态机、1000ms 超时监测、
高优先级急停优先调度、双电机在线状态判定 (连续3次失败离线)、
定时状态轮询以及批量参数读写调度.
"""

from collections import deque
from dataclasses import dataclass, field
import json
from pathlib import Path
import struct
from typing import Deque, Dict, List, Optional
from PySide6.QtCore import QObject, Signal, QTimer

from communication.protocol import (
    Command,
    SaveMode,
    StatusCode,
    RequestPacket,
    ResponsePacket,
    is_current_firmware_command_supported,
)
from communication.packet_parser import PacketParser
from communication.serial_manager import SerialManager


@dataclass
class QueueItem:
    """队列中的指令项."""
    request: RequestPacket
    is_high_priority: bool = False
    callback_note: str = ""


@dataclass
class MotorState:
    """单个电机的实时状态."""
    device_id: int
    is_online: bool = False
    consecutive_failures: int = 0
    servo_on: bool = False
    speed_rpm: int = 0
    position: int = 0
    alarm_code: int = 0
    last_pa_values: Dict[int, int] = field(default_factory=dict)


class ServoController(QObject):
    """双伺服驱动控制与调度引擎."""

    # 信号定义
    packet_sent = Signal(bytes, str)            # 发送数据帧 (raw, note)
    packet_received = Signal(bytes, str)        # 接收数据帧 (raw, note)
    command_completed = Signal(ResponsePacket)  # 指令成功应答
    command_failed = Signal(int, int, str)      # 指令失败 (device_id, command, reason)
    device_status_changed = Signal(int, dict)   # 电机状态变动 (device_id, state_dict)
    batch_progress = Signal(int, int, str)      # 批量任务进度 (current, total, info)
    log_message = Signal(str, str)              # 日志消息 (level, text)

    def __init__(self, serial_mgr: SerialManager, parent: Optional[QObject] = None):
        super().__init__(parent)
        self.serial_mgr = serial_mgr
        self.parser = PacketParser()

        # 双电机状态管理 (ID=1, ID=2)
        self.motors: Dict[int, MotorState] = {
            1: MotorState(device_id=1),
            2: MotorState(device_id=2),
        }

        # 指令队列与当前正在执行的指令
        self._queue: Deque[QueueItem] = deque()
        self._current_item: Optional[QueueItem] = None

        # 超时定时器 (默认 1000ms)
        self._timeout_timer = QTimer(self)
        self._timeout_timer.setSingleShot(True)
        self._timeout_timer.setInterval(1000)
        self._timeout_timer.timeout.connect(self._handle_command_timeout)

        # 状态轮询定时器
        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(500)
        self._poll_timer.timeout.connect(self._poll_next_status)
        self._poll_step = 0

        # 批量读写任务状态
        self._batch_in_progress = False
        self._batch_total = 0
        self._batch_current = 0

        # 关联串口信号
        self.serial_mgr.data_received.connect(self._on_serial_data_received)
        self.serial_mgr.disconnected.connect(self._on_serial_disconnected)

    @property
    def is_polling(self) -> bool:
        return self._poll_timer.isActive()

    def set_polling(self, enable: bool, interval_ms: int = 500) -> None:
        """开启或关闭后台电机状态轮询."""
        if enable:
            self._poll_timer.setInterval(interval_ms)
            self._poll_timer.start()
            self.log_message.emit("INFO", f"启动状态轮询 (间隔: {interval_ms} ms)")
        else:
            self._poll_timer.stop()
            self.log_message.emit("INFO", "已停止状态轮询")

    def enqueue_command(
        self,
        device_id: int,
        command: int,
        parameter: int = 0,
        data: bytes = b"",
        high_priority: bool = False,
        note: str = ""
    ) -> None:
        """向队列投递一条控制指令.

        :param device_id: 目标电机 ID (1 或 2)
        :param command: 命令字
        :param parameter: PA 参数号
        :param data: 附加负载
        :param high_priority: 是否急停/急保高优先级
        :param note: 操作描述文本
        """
        if not is_current_firmware_command_supported(command):
            reason = f"当前下位机固件不支持 {Command.get_name(command)}"
            self.command_failed.emit(device_id, command, reason)
            self.log_message.emit("WARN", f"Motor {device_id}: {reason}")
            return

        req = RequestPacket(
            device_id=device_id,
            command=command,
            parameter=parameter,
            data=data
        )
        item = QueueItem(request=req, is_high_priority=high_priority, callback_note=note)

        if high_priority:
            # 高优先级指令插入队列最前沿
            self._queue.appendleft(item)
            self.log_message.emit("WARN", f"[高优先级插队] {note or Command.get_name(command)} 作用于 Motor {device_id}")
        else:
            self._queue.append(item)

        self._process_queue()

    def emergency_stop_all(self) -> None:
        """对全部电机触发紧急停止 (清空普通队列, 优先发送急停指令)."""
        self._queue.clear()
        self.log_message.emit("WARN", "【系统急停触发】清空指令队列并下发全部急停")
        self.enqueue_command(1, Command.EMERGENCY_STOP, high_priority=True, note="Motor1 急停")
        self.enqueue_command(2, Command.EMERGENCY_STOP, high_priority=True, note="Motor2 急停")

    def read_pa(self, device_id: int, pa_num: int) -> None:
        """读取指定电机的单个 PA 参数."""
        self.enqueue_command(
            device_id=device_id,
            command=Command.READ_PA,
            parameter=pa_num,
            data=b"",
            note=f"读 PA{pa_num}"
        )

    def write_pa(self, device_id: int, pa_num: int, value: int, save_mode: int = SaveMode.PERMANENT) -> None:
        """写入指定电机的 PA 参数.

        数据格式: [save_mode(1字节), value(2字节大端有符号数)]
        """
        # UI 允许输入无符号 16 位范围；负数按 16 位补码原样上线路。
        val_bytes = struct.pack(">H", value & 0xFFFF)
        payload = bytes([save_mode & 0xFF]) + val_bytes
        mode_str = "永久" if save_mode == SaveMode.PERMANENT else "临时"
        self.enqueue_command(
            device_id=device_id,
            command=Command.WRITE_PA,
            parameter=pa_num,
            data=payload,
            note=f"写 PA{pa_num}={value} ({mode_str})"
        )

    def set_servo_power(self, device_id: int, enable: bool) -> None:
        """控制电机使能 (Servo ON / OFF)."""
        cmd = Command.SERVO_ON if enable else Command.SERVO_OFF
        note = f"Servo {'ON' if enable else 'OFF'}"
        self.enqueue_command(device_id, cmd, high_priority=not enable, note=note)

    def start_motor(self, device_id: int) -> None:
        """启动电机."""
        self.enqueue_command(device_id, Command.START, note="启动电机")

    def stop_motor(self, device_id: int) -> None:
        """停止电机 (高优先级)."""
        self.enqueue_command(device_id, Command.STOP, high_priority=True, note="停止电机")

    def reset_alarm(self, device_id: int) -> None:
        """复位电机报警."""
        self.enqueue_command(device_id, Command.RESET_ALARM, note="复位报警")

    def set_motor_speed(self, device_id: int, speed_rpm: int) -> None:
        """设置电机目标转速 (rpm)."""
        data = struct.pack(">h", int(speed_rpm))
        self.enqueue_command(
            device_id=device_id,
            command=Command.SET_SPEED,
            data=data,
            note=f"设定转速 {speed_rpm} rpm"
        )

    def move_relative(self, device_id: int, speed_rpm: int, pulses: int) -> None:
        """相对位置运动 (直接指定脉冲数)."""
        data = struct.pack(">hi", int(speed_rpm), int(pulses))
        self.enqueue_command(
            device_id=device_id,
            command=Command.MOVE_RELATIVE,
            data=data,
            note=f"相对运动 {pulses} 脉冲 ({speed_rpm} rpm)"
        )

    def move_by_turns(self, device_id: int, speed_rpm: int, turns: float, pulses_per_rev: int = 10000) -> None:
        """按旋转圈数运动 (支持小数, 例如 2.5 圈)."""
        pulses = int(round(turns * pulses_per_rev))
        self.move_relative(device_id, speed_rpm, pulses)

    def move_by_angle(self, device_id: int, speed_rpm: int, angle_deg: float, pulses_per_rev: int = 10000) -> None:
        """按旋转角度运动 (例如 90°, 180°, 360° 等)."""
        pulses = int(round((angle_deg / 360.0) * pulses_per_rev))
        self.move_relative(device_id, speed_rpm, pulses)

    def move_absolute(self, device_id: int, speed_rpm: int, target_pos: int) -> None:
        """绝对位置运动 (运动至绝对脉冲坐标)."""
        data = struct.pack(">hi", int(speed_rpm), int(target_pos))
        self.enqueue_command(
            device_id=device_id,
            command=Command.MOVE_ABSOLUTE,
            data=data,
            note=f"绝对定位至 {target_pos} ({speed_rpm} rpm)"
        )

    def jog(self, device_id: int, direction: int, speed_rpm: int, step_pulses: int) -> None:
        """点动微调运行."""
        dir_byte = 1 if direction > 0 else 0
        data = bytes([dir_byte]) + struct.pack(">hi", int(speed_rpm), int(step_pulses))
        dir_str = "正向" if dir_byte == 1 else "反向"
        self.enqueue_command(
            device_id=device_id,
            command=Command.JOG,
            data=data,
            note=f"点动{dir_str} {step_pulses} 脉冲"
        )

    def batch_read_parameters(self, device_id: int, pa_list: List[int]) -> None:
        """批量排队读取 PA 参数列表."""
        if not pa_list:
            return
        self._batch_in_progress = True
        self._batch_total = len(pa_list)
        self._batch_current = 0
        self.log_message.emit("INFO", f"启动批量读取 Motor {device_id} 共 {len(pa_list)} 个参数")
        for pa in pa_list:
            self.read_pa(device_id, pa)

    def batch_write_parameters(self, device_id: int, param_dict: Dict[int, int], save_mode: int) -> None:
        """批量排队写入 PA 参数字典."""
        if not param_dict:
            return
        self._batch_in_progress = True
        self._batch_total = len(param_dict)
        self._batch_current = 0
        self.log_message.emit("INFO", f"启动批量写入 Motor {device_id} 共 {len(param_dict)} 个参数")
        for pa, val in param_dict.items():
            self.write_pa(device_id, pa, val, save_mode)

    # ---------------- 队列调度与网络交互 ----------------

    def _process_queue(self) -> None:
        """调度执行队列中的下一条指令."""
        # 若已有指令在等待应答，或者队列为空，则直接返回
        if self._current_item is not None or not self._queue:
            return

        if not self.serial_mgr.is_open:
            return

        item = self._queue.popleft()
        self._current_item = item

        # 构建完整帧
        raw_frame = item.request.build_frame()

        # 启动超时计时 (1000 ms)
        self._timeout_timer.start(1000)

        # 发送
        self.serial_mgr.send_bytes(raw_frame)
        self.packet_sent.emit(raw_frame, item.callback_note)

    def _on_serial_data_received(self, chunk: bytes) -> None:
        """接收串口原始数据，分包解析."""
        packets = self.parser.feed_data(chunk)
        for pkt in packets:
            self._handle_response_packet(pkt)

    def _handle_response_packet(self, pkt: ResponsePacket) -> None:
        """处理解析出来的应答包."""
        self.packet_received.emit(pkt.raw_frame, f"Cmd: 0x{pkt.command:02X}, Status: 0x{pkt.status:02X}")

        # 匹配当前正在等待的指令
        if self._current_item is not None:
            curr_req = self._current_item.request
            # 校验设备与命令字一致性
            if pkt.device_id == curr_req.device_id and pkt.command == curr_req.command:
                self._timeout_timer.stop()
                item = self._current_item
                self._current_item = None

                if pkt.is_success:
                    # 只有下位机实际执行成功的命令才证明对应驱动器在线。
                    self._update_device_success(pkt.device_id)
                    self._apply_response_data(pkt)
                    self.command_completed.emit(pkt)
                else:
                    err_msg = pkt.status_desc
                    if pkt.status in (StatusCode.TIMEOUT, StatusCode.SERVO_NO_RESPONSE,
                                      StatusCode.RS485_ERROR, StatusCode.CRC_ERROR):
                        self._update_device_failure(pkt.device_id)
                    self.command_failed.emit(pkt.device_id, pkt.command, err_msg)
                    self.log_message.emit("ERROR", f"Motor {pkt.device_id} 指令失败: {err_msg}")

                self._check_batch_progress(pkt)
                # 继续调度队列
                self._process_queue()
                return

        # 若属于自发应答或非当前请求，更新状态
        self._apply_response_data(pkt)

    def _apply_response_data(self, pkt: ResponsePacket) -> None:
        """将应答数据解析到电机实时状态."""
        dev = self.motors.get(pkt.device_id)
        if not dev:
            return

        if pkt.command == Command.READ_PA and pkt.is_success:
            val = pkt.get_value_int16()
            if val is not None:
                dev.last_pa_values[pkt.parameter] = val

        elif pkt.command == Command.WRITE_PA and pkt.is_success:
            val = pkt.get_value_int16()
            if val is not None:
                dev.last_pa_values[pkt.parameter] = val

        elif pkt.command in (Command.SERVO_ON, Command.SERVO_OFF):
            if pkt.is_success:
                dev.servo_on = (pkt.command == Command.SERVO_ON)

        elif pkt.command == Command.READ_SPEED and pkt.is_success:
            val = pkt.get_value_int16()
            if val is not None:
                dev.speed_rpm = val
            if len(pkt.data) >= 4:
                run_state = struct.unpack(">H", pkt.data[2:4])[0]
                dev.servo_on = (run_state == 3)

        elif pkt.command == Command.READ_POSITION and pkt.is_success:
            val = pkt.get_value_int32()
            if val is not None:
                dev.position = val

        elif pkt.command == Command.READ_ALARM and pkt.is_success:
            val = pkt.get_value_uint16()
            if val is not None:
                dev.alarm_code = val

        elif pkt.command == Command.RESET_ALARM and pkt.is_success:
            dev.alarm_code = 0

        elif pkt.command == Command.SET_SPEED and pkt.is_success:
            val = pkt.get_value_int16()
            if val is not None:
                dev.speed_rpm = val

        elif pkt.command in (Command.MOVE_RELATIVE, Command.MOVE_ABSOLUTE, Command.JOG) and pkt.is_success:
            pos = pkt.get_value_int32()
            if pos is not None:
                dev.position = pos

        self._notify_device_status(pkt.device_id)

    def _handle_command_timeout(self) -> None:
        """指令超时 (1000ms 未应答) 回调."""
        if self._current_item is None:
            return

        item = self._current_item
        self._current_item = None
        dev_id = item.request.device_id
        cmd = item.request.command

        self._update_device_failure(dev_id)
        timeout_desc = f"Communication Timeout (1000ms)"
        self.command_failed.emit(dev_id, cmd, timeout_desc)
        self.log_message.emit("ERROR", f"Motor {dev_id} {item.callback_note or Command.get_name(cmd)}: {timeout_desc}")

        self._check_batch_progress(None)
        # 继续调度队列中的下一条
        self._process_queue()

    def _update_device_success(self, device_id: int) -> None:
        """通信成功, 清零失败计数并标记在线."""
        dev = self.motors.get(device_id)
        if dev:
            dev.consecutive_failures = 0
            if not dev.is_online:
                dev.is_online = True
                self.log_message.emit("INFO", f"Motor {device_id} 恢复在线")
            self._notify_device_status(device_id)

    def _update_device_failure(self, device_id: int) -> None:
        """通信失败, 失败计数累加, 连续3次离线."""
        dev = self.motors.get(device_id)
        if dev:
            dev.consecutive_failures += 1
            if dev.consecutive_failures >= 3 and dev.is_online:
                dev.is_online = False
                self.log_message.emit("WARN", f"Motor {device_id} 连续3次通信超时, 标记为离线 (Offline)")
            self._notify_device_status(device_id)

    def _notify_device_status(self, device_id: int) -> None:
        """向外部发送电机状态刷新信号."""
        dev = self.motors.get(device_id)
        if dev:
            self.device_status_changed.emit(device_id, {
                "device_id": dev.device_id,
                "is_online": dev.is_online,
                "consecutive_failures": dev.consecutive_failures,
                "servo_on": dev.servo_on,
                "speed_rpm": dev.speed_rpm,
                "position": dev.position,
                "alarm_code": dev.alarm_code,
                "last_pa_values": dev.last_pa_values.copy(),
            })

    def _check_batch_progress(self, pkt: Optional[ResponsePacket]) -> None:
        """更新批量任务进度."""
        if self._batch_in_progress:
            self._batch_current += 1
            info = f"正在处理 ({self._batch_current}/{self._batch_total})"
            if self._batch_current >= self._batch_total:
                self._batch_in_progress = False
                info = "批量任务完成"
            self.batch_progress.emit(self._batch_current, self._batch_total, info)

    def _poll_next_status(self) -> None:
        """后台轮询电机状态 (仅在队列空闲且串口连通时轮询)."""
        if not self.serial_mgr.is_open or self._current_item is not None or len(self._queue) > 0:
            return

        # 轮流读取 Motor 1 与 Motor 2 的 速度、位置与报警
        # poll_step 0..5:
        # 0: M1 Speed, 1: M1 Position, 2: M1 Alarm
        # 3: M2 Speed, 4: M2 Position, 5: M2 Alarm
        dev_id = 1 if self._poll_step < 3 else 2
        sub_step = self._poll_step % 3

        if sub_step == 0:
            self.enqueue_command(dev_id, Command.READ_SPEED, note="轮询转速")
        elif sub_step == 1:
            self.enqueue_command(dev_id, Command.READ_POSITION, note="轮询位置")
        elif sub_step == 2:
            self.enqueue_command(dev_id, Command.READ_ALARM, note="轮询报警")

        self._poll_step = (self._poll_step + 1) % 6

    def _on_serial_disconnected(self) -> None:
        """串口断开时清理状态."""
        self._timeout_timer.stop()
        self._current_item = None
        self._queue.clear()
        self.parser.reset()
        for dev in self.motors.values():
            dev.is_online = False
            dev.servo_on = False
            self._notify_device_status(dev.device_id)
        self.log_message.emit("WARN", "串口已断开, 已重置电机在线状态")
