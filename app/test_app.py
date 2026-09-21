"""上位机协议与数据流自动化单元测试.

用于验证 CRC16 校验、帧编解码、分包粘包解析、状态机及模拟器交互.
"""

import struct
from PySide6.QtCore import QObject, Signal, QCoreApplication
from utils.crc import calculate_crc16, append_crc16, verify_crc16
from communication.protocol import (
    Command,
    SaveMode,
    StatusCode,
    RequestPacket,
    ResponsePacket,
    parse_response_frame,
    CURRENT_FIRMWARE_COMMANDS,
    is_current_firmware_command_supported,
)
from communication.packet_parser import PacketParser
from communication.serial_manager import VirtualStm32Simulator, SerialManager
from servo.servo_controller import ServoController


class _FakeSerialManager(QObject):
    """用于控制器单元测试的最小串口替身。"""

    data_received = Signal(bytes)
    disconnected = Signal()

    def __init__(self):
        super().__init__()
        self.is_open = True
        self.sent = []

    def send_bytes(self, data: bytes) -> bool:
        self.sent.append(data)
        return True


class _FakePhysicalSerial:
    def __init__(self):
        self.is_open = True
        self.closed = False

    def close(self):
        self.closed = True
        self.is_open = False


def test_controller_current_firmware_capability():
    """完整固件能力、在线状态和轮询行为。"""
    app = QCoreApplication.instance() or QCoreApplication([])
    serial_mgr = _FakeSerialManager()
    controller = ServoController(serial_mgr)
    controller.move_relative(1, 500, 10000)
    assert len(serial_mgr.sent) == 1
    assert controller._current_item is not None
    assert controller._current_item.request.command == Command.MOVE_RELATIVE
    assert is_current_firmware_command_supported(Command.MOVE_RELATIVE)

    controller._handle_response_packet(
        ResponsePacket(1, Command.MOVE_RELATIVE, StatusCode.SUCCESS, 0, struct.pack(">i", 10000))
    )
    assert controller.motors[1].is_online
    controller.set_polling(True)
    assert controller.is_polling
    controller.set_polling(False)

    # 失败应答不能把尚未在线的另一台电机标记为在线。
    controller.read_pa(2, 21)
    unknown = ResponsePacket(2, Command.READ_PA, StatusCode.UNKNOWN_CMD, 21)
    controller._handle_response_packet(unknown)
    assert not controller.motors[2].is_online

    controller.read_pa(2, 21)
    success = ResponsePacket(2, Command.READ_PA, StatusCode.SUCCESS, 21, struct.pack(">h", 60))
    controller._handle_response_packet(success)
    assert controller.motors[2].is_online

    # READ_SPEED 的扩展数据携带 0x1016，可反映 DB44 外部 SON 使能状态。
    controller.enqueue_command(2, Command.READ_SPEED)
    speed_with_run_state = ResponsePacket(
        2, Command.READ_SPEED, StatusCode.SUCCESS, 0, struct.pack(">hH", 0, 3)
    )
    controller._handle_response_packet(speed_with_run_state)
    assert controller.motors[2].servo_on
    controller.deleteLater()
    serial_mgr.deleteLater()
    print("[PASS] test_controller_current_firmware_capability")


def test_write_pa_16bit_wire_boundaries():
    """PA 写入使用 16 位线缆位模式覆盖完整 UI 范围。"""
    serial_mgr = _FakeSerialManager()
    serial_mgr.is_open = False
    controller = ServoController(serial_mgr)
    for value, expected in ((-32768, b"\x80\x00"), (-1, b"\xff\xff"),
                             (32768, b"\x80\x00"), (65535, b"\xff\xff")):
        controller.write_pa(1, 21, value)
        item = controller._queue.pop()
        assert item.request.data[-2:] == expected
    controller.deleteLater()
    serial_mgr.deleteLater()
    print("[PASS] test_write_pa_16bit_wire_boundaries")


def test_serial_manager_physical_close_emits_disconnected():
    """物理串口关闭时应发出断开信号并清理连接字段。"""
    mgr = SerialManager()
    fake_serial = _FakePhysicalSerial()
    mgr._serial = fake_serial
    mgr._current_port = "COM_TEST"
    disconnected = []
    mgr.disconnected.connect(lambda: disconnected.append(True))
    mgr.close()
    assert disconnected == [True]
    assert fake_serial.closed
    assert mgr._serial is None
    assert mgr._current_port == ""
    print("[PASS] test_serial_manager_physical_close_emits_disconnected")


def test_crc16():
    """测试 Modbus CRC16 算法."""
    # 标准 Modbus RTU 测试数据: 01 03 00 00 00 01 -> CRC = 84 0A (Little Endian: 0x0A 0x84)
    test_bytes = bytes([0x01, 0x03, 0x00, 0x00, 0x00, 0x01])
    crc = calculate_crc16(test_bytes)
    assert crc == 0x0A84, f"CRC16 test failed, got 0x{crc:04X}"

    appended = append_crc16(test_bytes)
    assert len(appended) == len(test_bytes) + 2
    assert appended[-2] == 0x84
    assert appended[-1] == 0x0A
    assert verify_crc16(appended) is True
    print("[PASS] test_crc16")


def test_packet_builder():
    """测试请求帧构建."""
    # Write PA21 = 60 on Motor 1, Permanent mode
    val_bytes = struct.pack(">h", 60)
    data = bytes([SaveMode.PERMANENT]) + val_bytes
    req = RequestPacket(
        device_id=1,
        command=Command.WRITE_PA,
        parameter=21,
        data=data
    )
    frame = req.build_frame()
    assert frame[0] == 0xAA and frame[1] == 0x55
    length = frame[2]
    assert len(frame) == 3 + length
    assert frame[3] == 1  # Motor 1
    assert frame[4] == Command.WRITE_PA
    pa = struct.unpack(">H", frame[5:7])[0]
    assert pa == 21
    assert verify_crc16(frame) is True
    print(f"[PASS] test_packet_builder (Frame: {frame.hex(' ').upper()})")


def test_packet_parser_and_sticky_frames():
    """测试粘包、半包与杂散噪声过滤."""
    parser = PacketParser()

    # 1. 模拟一个合法应答包: Dev=1, Cmd=READ_PA, Status=SUCCESS, Param=21, Value=60
    sim = VirtualStm32Simulator()
    req_frame = RequestPacket(device_id=1, command=Command.READ_PA, parameter=21).build_frame()
    resp_frame = sim.process_request(req_frame)
    assert resp_frame is not None

    # 2. 模拟前端噪声 + 两个完整帧粘包 + 一个半包
    noisy_stream = bytes([0xFF, 0x00, 0xAA, 0x12]) + resp_frame + resp_frame + resp_frame[:4]
    packets = parser.feed_data(noisy_stream)
    assert len(packets) == 2, f"Expected 2 packets, got {len(packets)}"
    assert packets[0].device_id == 1
    assert packets[0].parameter == 21
    assert packets[0].get_value_int16() == 60

    # 3. 补齐半包的后续数据
    packets2 = parser.feed_data(resp_frame[4:])
    assert len(packets2) == 1
    assert packets2[0].device_id == 1
    assert packets2[0].parameter == 21
    assert packets2[0].get_value_int16() == 60
    print("[PASS] test_packet_parser_and_sticky_frames")


def test_simulator_features():
    """测试模拟器对双电机的参数写入、读取与状态切换."""
    sim = VirtualStm32Simulator()

    # 向 Motor 2 写入 PA21 = 120
    w_req = RequestPacket(
        device_id=2,
        command=Command.WRITE_PA,
        parameter=21,
        data=bytes([SaveMode.PERMANENT]) + struct.pack(">h", 120)
    ).build_frame()
    w_resp = sim.process_request(w_req)
    assert w_resp is not None
    assert verify_crc16(w_resp) is True

    # 读取 Motor 2 PA21
    r_req = RequestPacket(device_id=2, command=Command.READ_PA, parameter=21).build_frame()
    r_resp = sim.process_request(r_req)
    pkt = parse_response_frame(r_resp)
    assert pkt is not None
    assert pkt.is_success is True
    assert pkt.device_id == 2
    assert pkt.parameter == 21
    assert pkt.get_value_int16() == 120

    # 当前物理固件与默认模拟器均支持完整控制命令。
    on_req = RequestPacket(device_id=1, command=Command.SERVO_ON).build_frame()
    on_pkt = parse_response_frame(sim.process_request(on_req))
    assert on_pkt is not None and on_pkt.status == StatusCode.SUCCESS

    assert CURRENT_FIRMWARE_COMMANDS == frozenset(Command._NAMES.keys())
    assert is_current_firmware_command_supported(Command.READ_PA)
    assert is_current_firmware_command_supported(Command.MOVE_RELATIVE)

    print("[PASS] test_simulator_features")


def test_motion_commands():
    """测试速度设置、圈数折算、定角运动与点动微调."""
    sim = VirtualStm32Simulator(full_mode=True)
    ppr = 10000  # 每转 10000 脉冲

    # 1. 设定转速 800 rpm
    spd_req = RequestPacket(
        device_id=1,
        command=Command.SET_SPEED,
        data=struct.pack(">h", 800)
    ).build_frame()
    resp = sim.process_request(spd_req)
    pkt = parse_response_frame(resp)
    assert pkt is not None and pkt.is_success
    assert sim.motor_states[1]["speed"] == 800

    # 2. 旋转 2.5 圈 (25000 脉冲)
    turns = 2.5
    pulses = int(round(turns * ppr))
    initial_pos = sim.motor_states[1]["pos"]
    move_req = RequestPacket(
        device_id=1,
        command=Command.MOVE_RELATIVE,
        data=struct.pack(">hi", 800, pulses)
    ).build_frame()
    resp = sim.process_request(move_req)
    pkt = parse_response_frame(resp)
    assert pkt is not None and pkt.is_success
    assert pkt.get_value_int32() == initial_pos + 25000

    # 3. 旋转 90° ((90 / 360) * 10000 = 2500 脉冲)
    angle = 90.0
    angle_pulses = int(round((angle / 360.0) * ppr))
    assert angle_pulses == 2500
    curr_pos = sim.motor_states[1]["pos"]
    angle_req = RequestPacket(
        device_id=1,
        command=Command.MOVE_RELATIVE,
        data=struct.pack(">hi", 500, angle_pulses)
    ).build_frame()
    resp = sim.process_request(angle_req)
    pkt = parse_response_frame(resp)
    assert pkt is not None and pkt.is_success
    assert pkt.get_value_int32() == curr_pos + 2500

    # 4. 点动步进测试 (正转 500 脉冲)
    jog_curr = sim.motor_states[1]["pos"]
    jog_req = RequestPacket(
        device_id=1,
        command=Command.JOG,
        data=bytes([1]) + struct.pack(">hi", 300, 500)
    ).build_frame()
    resp = sim.process_request(jog_req)
    pkt = parse_response_frame(resp)
    assert pkt is not None and pkt.is_success
    assert pkt.get_value_int32() == jog_curr + 500

    print("[PASS] test_motion_commands")


if __name__ == "__main__":
    test_crc16()
    test_packet_builder()
    test_packet_parser_and_sticky_frames()
    test_simulator_features()
    test_motion_commands()
    test_controller_current_firmware_capability()
    test_write_pa_16bit_wire_boundaries()
    test_serial_manager_physical_close_emits_disconnected()
    print("\nAll unit tests passed successfully!")
