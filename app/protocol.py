"""LightCheck 精简上位机使用的二进制协议。"""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct


HEADER = b"\xAA\x55"
MAX_FRAME_SIZE = 64

CMD_WRITE_PA = 0x01
CMD_STOP = 0x06
CMD_READ_PA = 0x02
CMD_READ_SPEED = 0x07
CMD_READ_POSITION = 0x08
CMD_READ_ALARM = 0x09
CMD_RESET_ALARM = 0x0A
CMD_MOVE_RELATIVE = 0x11
CMD_MOVE_OUTPUT_RELATIVE = 0x19  # v3.8.5: feedback-tracked point task; old firmware rejects it.
CMD_MOVE_AXES_POINT = 0x1A  # v3.9.0: prepare both absolute targets before either axis runs.
CMD_SET_MOTION_PROFILE = 0x15
CMD_LIMIT_DEBUG = 0x16
CMD_ROTATE_CONTINUOUS = 0x17
CMD_ROTATE_KEEPALIVE = 0x18
CMD_READ_FEEDBACK = 0x20
CMD_CONFIG_GUARD = 0x21
CMD_SET_ZERO = 0x22
CMD_HOME = 0x23
CMD_READ_GUARD = 0x24
CMD_SET_REFERENCE = 0x25
CMD_CONFIG_DUAL_LIMIT = 0x26
CMD_READ_DUAL_LIMIT = 0x27
CMD_HOME_DUAL_LIMIT = 0x28
CMD_TRAJECTORY_BEGIN = 0x30
CMD_TRAJECTORY_POINT = 0x31
CMD_TRAJECTORY_SEAL = 0x32
CMD_TRAJECTORY_START = 0x33
CMD_TRAJECTORY_STATUS = 0x34
CMD_TRAJECTORY_STOP = 0x35
CMD_TRAJECTORY_KEEPALIVE = 0x36
CMD_READ_AXES = 0x37
CMD_TRAJECTORY_CLEAR = 0x38


def build_trajectory_begin(session: int, count: int, *, cycle_ms: int = 250,
                           max_speed: float = 20, max_accel: float = 30,
                           max_error: float = 5, tolerance: float = 0.5,
                           adaptive: bool = False) -> bytes:
    if type(session) is not int or not 1 <= session <= 65535:
        raise ValueError("轨迹会话号必须为1～65535")
    if type(count) is not int or not 2 <= count <= 128 or type(cycle_ms) is not int or not 50 <= cycle_ms <= 500:
        raise ValueError("轨迹2～128点，控制周期50～500ms（小于250ms需要新固件）")
    if type(adaptive) is not bool:
        raise ValueError("adaptive必须是布尔值")
    if not all(math.isfinite(x) for x in (max_speed, max_accel, max_error, tolerance)):
        raise ValueError("轨迹约束必须为有限数字")
    v, a, e, tol = (round(x*1000) for x in (max_speed, max_accel, max_error, tolerance))
    if not 1 <= v <= 60000 or not 1 <= a <= 120000 or not 500 <= e <= 10000 or not 50 <= tol <= 1000 or tol >= e:
        raise ValueError("最大速度≤60°/秒，加速度≤120°/秒²，误差0.5～10°，容差0.05～1°且小于误差限")
    return build_request(1, CMD_TRAJECTORY_BEGIN, struct.pack(">HHiiii", count, cycle_ms | (0x8000 if adaptive else 0), v, a, e, tol), parameter=session)


def build_trajectory_point(session: int, index: int, time_ms: int, axis1: float, axis2: float) -> bytes:
    if type(session) is not int or not 1 <= session <= 65535 or type(index) is not int or not 0 <= index < 128:
        raise ValueError("轨迹会话／点序号非法")
    if type(time_ms) is not int or not 0 <= time_ms <= 600000 or not all(math.isfinite(x) for x in (axis1, axis2)):
        raise ValueError("轨迹时间0～600000ms，角度必须为有限数字")
    angles = [round(x*1000) for x in (axis1, axis2)]
    if any(not -(2**31) <= x < 2**31 for x in angles):
        raise ValueError("轨迹角度超出协议范围")
    return build_request(1, CMD_TRAJECTORY_POINT, struct.pack(">HIii", index, time_ms, *angles), parameter=session)


STATUS_TEXT = {
    0x00: "成功",
    0x01: "超时",
    0x02: "CRC 错误",
    0x03: "驱动器无响应",
    0x04: "参数非法",
    0x05: "RS485 通信错误",
    0x06: "设备忙",
    0x07: "未知命令",
    0x08: "限位触发/断线或尚未确认停止，禁止启动",
    0x09: "超出软范围或此原始命令不允许绕过保护",
    0x0A: "未建立可信零点或未确认回零配置",
}


def build_set_reference(device_id: int, angle_deg: float = 0) -> bytes:
    """Capture this physical pose as the supplied output-axis angle; no movement."""
    if type(device_id) is not int or device_id not in (1,2):raise ValueError("电机编号须为1或2")
    if not math.isfinite(angle_deg) or not -36000<=angle_deg<=36000:
        raise ValueError("参考角度须为有限值，范围±36000°")
    if device_id==1 and not -35<=angle_deg<=35:
        raise ValueError("电机1参考必须对应真实水平坐标且在±35°内")
    return build_request(device_id,CMD_SET_REFERENCE,struct.pack(">i",round(angle_deg*1000)),parameter=0x5246)


def build_guard_inputs(device_id: int, negative: int | None, positive: int | None,
                       home: int | None, active_high: bool = True) -> bytes:
    channels = [255 if ch is None else ch for ch in (negative, positive, home)]
    if any(type(ch) is not int or ch not in (*range(6), 255) for ch in channels):
        raise ValueError("通道只能为DI0～DI5或None")
    if channels[0] != 255 and channels[0] == channels[1]:
        raise ValueError("负限位和正限位不能为同一个输入")
    return build_request(device_id, CMD_CONFIG_GUARD, bytes((*channels, int(active_high))), parameter=1)


def build_soft_range(device_id: int, minimum: float, maximum: float) -> bytes:
    if not all(math.isfinite(x) for x in (minimum, maximum)) or not -36000 <= minimum <= 0 <= maximum <= 36000 or minimum >= maximum:
        raise ValueError("范围必须包含零点，且 -36000≤最小值<最大值≤36000°")
    lo, hi = round(minimum*1000), round(maximum*1000)
    if lo >= hi:
        raise ValueError("范围小于协议分辨率")
    return build_request(device_id, CMD_CONFIG_GUARD, struct.pack(">ii", lo, hi), parameter=2)


def build_guard_policy(device_id: int, *, hardware: bool, soft: bool,
                       home_direction: int = 0, confirmed: bool = False, home_at_end: bool = False) -> bytes:
    if home_direction not in (-1, 0, 1) or confirmed and home_direction == 0:
        raise ValueError("确认回零前必须明确正/负方向")
    flags = int(hardware) | (int(soft)<<1) | (int(confirmed)<<2) | (int(home_at_end)<<3)
    return build_request(device_id, CMD_CONFIG_GUARD, bytes((flags, home_direction & 255)), parameter=3)


def build_home(device_id: int, speed_deg_s: float = 0.5, max_travel_deg: float = 30.0,
               backoff_deg: float = 2.0, timeout_s: float = 90.0) -> bytes:
    if not all(math.isfinite(x) for x in (speed_deg_s,max_travel_deg,backoff_deg,timeout_s)):
        raise ValueError("回零参数必须为有限数字")
    if not 0 < speed_deg_s <= 5 or not 0 < backoff_deg <= min(10,max_travel_deg) or not max_travel_deg <= 360 or not 1 <= timeout_s <= 120:
        raise ValueError("回零速度≤5°/秒，总行程≤360°，退让≤10°且不超总行程，超时1～120秒")
    values = [round(x*1000) for x in (speed_deg_s,max_travel_deg,backoff_deg,timeout_s)]
    if any(v <= 0 for v in values):
        raise ValueError("回零参数小于协议分辨率")
    return build_request(device_id, CMD_HOME, struct.pack(">iiii", *values), parameter=0x484D)


def crc16_modbus(data: bytes | bytearray) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def append_crc(data: bytes | bytearray) -> bytes:
    raw = bytes(data)
    crc = crc16_modbus(raw)
    return raw + bytes((crc & 0xFF, crc >> 8))


def verify_crc(frame: bytes | bytearray) -> bool:
    if len(frame) < 2:
        return False
    expected = frame[-2] | (frame[-1] << 8)
    return crc16_modbus(frame[:-2]) == expected


def build_dual_limits(*, horizontal_channel=2, raised_channel=0, active_high=True,
                      positive_is_up=True, horizontal_deg=None, raised_deg=None, enabled=True):
    """Two independently measured stopped sensor poses; None means unmeasured."""
    if any(type(ch) is not int or not 0 <= ch < 6 for ch in (horizontal_channel, raised_channel)) or horizontal_channel==raised_channel:
        raise ValueError("两个限位器必须使用不同的 DI0～DI5")
    if any(type(v) is not bool for v in (enabled, active_high, positive_is_up)):
        raise ValueError("启用、触发电平和方向必须明确选择")
    values=[];valid=0
    for n,angle in enumerate((horizontal_deg,raised_deg)):
        if angle is None: values.append(0);continue
        if isinstance(angle,bool) or not math.isfinite(angle) or not -29 <= angle <= 35:
            raise ValueError("双限位实测角度须在电机1的 −29°～+35°范围内")
        values.append(round(angle*1000));valid|=1<<n
    if valid==3 and values[0]>=values[1]:raise ValueError("抬头限位的角度必须高于水平侧限位")
    data=struct.pack(">BBBBbBii",enabled,horizontal_channel,raised_channel,active_high,
                     1 if positive_is_up else -1,valid,*values)
    return build_request(1,CMD_CONFIG_DUAL_LIMIT,data,0x444C)


def build_dual_home(direction, *, speed_deg_s=0.3, max_travel_deg=40, timeout_s=120, raised_only=False):
    """Direction is raw drive-command sign, independent of physical pitch sign."""
    if type(direction) is not int or direction not in (-1,1):raise ValueError("搜索方向必须是 +1 或 −1")
    if type(raised_only) is not bool or (raised_only and direction!=-1):raise ValueError("DI0抬头回零必须使用原始反向命令")
    values=(speed_deg_s,max_travel_deg,timeout_s)
    if any(isinstance(v,bool) or not math.isfinite(v) for v in values):raise ValueError("回零参数必须是有限数值")
    speed,budget,timeout=(round(v*1000) for v in values)
    if not 1<=speed<=9000 or not 1<=budget<=70000 or not 1000<=timeout<=120000:
        raise ValueError("搜索速度须在 0.001～9°/秒，行程≤70°，超时1～120秒")
    return build_request(1,CMD_HOME_DUAL_LIMIT,struct.pack(">biii",direction,speed,budget,timeout),0x4449 if raised_only else 0x4448)


def build_request(device_id: int, command: int, data: bytes = b"", parameter: int = 0) -> bytes:
    if device_id not in (1, 2):
        raise ValueError("电机编号只能是 1 或 2")
    payload = bytes((device_id, command)) + struct.pack(">H", parameter) + data
    if len(payload) + 5 > MAX_FRAME_SIZE:
        raise ValueError("协议帧过长")
    return append_crc(HEADER + bytes((len(payload) + 2,)) + payload)


def build_continuous_rotation(device_id: int, degrees_per_second: float) -> bytes:
    if not math.isfinite(degrees_per_second):
        raise ValueError("角速度必须为有限数字")
    raw = int(math.floor(abs(degrees_per_second) * 1000 + 0.5))
    if degrees_per_second < 0:
        raw = -raw
    if raw == 0 or not -(2**31) <= raw <= 2**31 - 1:
        raise ValueError("角速度为零或超出协议范围")
    return build_request(device_id, CMD_ROTATE_CONTINUOUS, struct.pack(">i", raw))


def build_rotation_keepalive(device_id: int) -> bytes:
    return build_request(device_id, CMD_ROTATE_KEEPALIVE, parameter=0x4352)


def build_limit_debug(device_id: int, action: int) -> bytes:
    """0=恢复保护，1=申请旁路（须停稳），2=续期；下位机5秒未续期恢复保护。"""
    if action not in (0, 1, 2):
        raise ValueError("无效的限位调试操作")
    return build_request(device_id, CMD_LIMIT_DEBUG, bytes((action,)), parameter=0x4C44)


def build_move_relative(device_id: int, speed_rpm: int, pulses: int) -> bytes:
    if not 1 <= speed_rpm <= 5000:
        raise ValueError("速度范围必须是 1～5000 rpm")
    if not -(2**31) <= pulses <= 2**31 - 1 or pulses == 0:
        raise ValueError("脉冲数超出 int32 范围或为零")
    return build_request(device_id, CMD_MOVE_RELATIVE, struct.pack(">hi", speed_rpm, pulses))


def build_write_pa(device_id: int, parameter: int, value: int, save: bool = False) -> bytes:
    """构造 PA 参数写入帧；默认临时生效，避免频繁写驱动器 EEPROM。"""
    if not 0 <= parameter < 128:
        raise ValueError("PA 参数编号必须在 0～127 之间。")
    if not -32768 <= value <= 65535:
        raise ValueError("PA 参数值超出 16 位范围。")
    data = bytes((1 if save else 0,)) + struct.pack(">H", value & 0xFFFF)
    return build_request(device_id, CMD_WRITE_PA, data=data, parameter=parameter)


def build_move_output_relative(device_id: int, output_speed_rpm: float, output_angle_deg: float) -> bytes:
    """v3.8.5实际位置追踪帧0x19/PT：0.01 rpm、0.001°；旧固件拒绝。"""
    if not math.isfinite(output_speed_rpm) or not math.isfinite(output_angle_deg):
        raise ValueError("输出轴速度和角度必须是有限数字。")
    speed_centi = int(math.floor(output_speed_rpm * 100.0 + 0.5))
    angle_magnitude = int(math.floor(abs(output_angle_deg) * 1000.0 + 0.5))
    angle_millideg = angle_magnitude if output_angle_deg >= 0 else -angle_magnitude
    if not 1 <= speed_centi <= 65535:
        raise ValueError("输出轴速度必须在 0.01～655.35 rpm 之间。")
    if angle_millideg == 0 or not -(2**31) <= angle_millideg <= 2**31 - 1:
        raise ValueError("输出轴角度小于 0.001°或超出协议范围。")
    return build_request(device_id, CMD_MOVE_OUTPUT_RELATIVE, struct.pack(">Hi", speed_centi, angle_millideg), parameter=0x5054)


def build_move_axes_point(targets_deg, speed_rpm: float, *, tolerance_deg=None) -> bytes:
    if len(targets_deg)!=2 or not all(math.isfinite(v) for v in targets_deg):
        raise ValueError('需要两个有限目标角度')
    if not math.isfinite(speed_rpm) or not 0.01<=speed_rpm<=100:
        raise ValueError('实际输出轴速度须在0.01～100rpm内')
    targets=[round(v*1000) for v in targets_deg]
    if any(not -(2**31)<=v<2**31 for v in targets):raise ValueError('角度超出协议范围')
    data=struct.pack('>Hii',int(math.floor(speed_rpm*100+.5)),*targets)
    if tolerance_deg is not None:
        if not math.isfinite(tolerance_deg) or not .01<=tolerance_deg<=.5:raise ValueError('终点容差须为0.01～0.5°')
        data+=struct.pack('>H',int(math.floor(tolerance_deg*1000+.5)))
    return build_request(1,CMD_MOVE_AXES_POINT,data,parameter=0x4A50)


def build_motion_profile(device_id: int, accel_ms: int, decel_ms: int, s_curve_ms: int) -> bytes:
    """设置P100S内部位置模式的PA40、PA41、PA42，均为临时写入。"""
    if not 1 <= accel_ms <= 10000 or not 1 <= decel_ms <= 10000:
        raise ValueError("PA40/PA41必须在 1～10000 ms 之间。")
    if not 0 <= s_curve_ms <= 1000:
        raise ValueError("PA42必须在 0～1000 ms 之间。")
    return build_request(device_id, CMD_SET_MOTION_PROFILE, struct.pack(">HHH", accel_ms, decel_ms, s_curve_ms))


@dataclass(frozen=True)
class Response:
    device_id: int
    command: int
    status: int
    parameter: int
    data: bytes
    raw: bytes

    @property
    def ok(self) -> bool:
        return self.status == 0

    @property
    def status_text(self) -> str:
        return STATUS_TEXT.get(self.status, f"未知状态 0x{self.status:02X}")


def parse_response(frame: bytes) -> Response | None:
    if len(frame) < 10 or frame[:2] != HEADER or len(frame) != frame[2] + 3:
        return None
    if frame[2] < 7 or frame[2] > MAX_FRAME_SIZE - 3 or not verify_crc(frame):
        return None
    return Response(
        device_id=frame[3],
        command=frame[4],
        status=frame[5],
        parameter=struct.unpack(">H", frame[6:8])[0],
        data=frame[8:-2],
        raw=frame,
    )


class ResponseParser:
    """处理串口半包、粘包、噪声和 CRC 错帧。"""

    def __init__(self) -> None:
        self.buffer = bytearray()

    def reset(self) -> None:
        self.buffer.clear()

    def feed(self, data: bytes) -> list[Response]:
        self.buffer.extend(data)
        if len(self.buffer) > 4096:
            del self.buffer[:-4096]

        packets: list[Response] = []
        while True:
            start = self.buffer.find(HEADER)
            if start < 0:
                self.buffer[:] = b"\xAA" if self.buffer[-1:] == b"\xAA" else b""
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 3:
                break

            length = self.buffer[2]
            if length < 7 or length > MAX_FRAME_SIZE - 3:
                del self.buffer[0]
                continue

            total = length + 3
            if len(self.buffer) < total:
                # 若错误长度挡在合法完整帧前，优先恢复到后续合法帧。
                recovered = False
                search_from = 1
                while True:
                    nested = self.buffer.find(HEADER, search_from)
                    if nested < 0 or len(self.buffer) < nested + 3:
                        break
                    nested_total = self.buffer[nested + 2] + 3
                    if (10 <= nested_total <= MAX_FRAME_SIZE and
                            len(self.buffer) >= nested + nested_total and
                            verify_crc(self.buffer[nested:nested + nested_total])):
                        del self.buffer[:nested]
                        recovered = True
                        break
                    search_from = nested + 1
                if recovered:
                    continue
                break

            candidate = bytes(self.buffer[:total])
            packet = parse_response(candidate)
            if packet is None:
                del self.buffer[0]
            else:
                packets.append(packet)
                del self.buffer[:total]
        return packets
