"""故障处理目录：只生成建议，不执行复位、标定或运动。"""
from dataclasses import dataclass

from .protocol import STATUS_TEXT
from .trajectory import FAULTS


@dataclass(frozen=True)
class FaultAdvice:
    domain: str
    code: int | str
    title: str
    steps: tuple[str, ...]
    apis: tuple[str, ...]
    motor_id: int | None = None
    detail: str = ""
    execution_unknown: bool = False


# Numeric codes belong to different namespaces; never infer the domain from a number.
_REFERENCE = ("核查反馈单位、计数和真实姿态；参考失效时在已知真实角度停稳后重新标定，禁止随意设零。",
              "重新标定会作废旧轨迹；重新规划并明确启动。")
_COMM = ("先确认机构停止；通信失效无法确认停止时使用硬件急停。",
         "检查供电、串口占用、RS485接线、地址及波特率；驱动PA72=1152、PA73=0。",
         "close()后重新connect()并读回状态、角度；禁止自动重发结果未知的运动。")
_TRAJECTORY = {
    0: (("无轨迹故障；是否允许启动还须检查参考、位置、报警和任务状态。",), ("get_axis_positions",)),
    1: (("人工取消后确认两轴停稳，再清除旧任务并重新准备。",), ("clear_trajectory_fault",)),
    2: (_REFERENCE, ("get_axis_positions", "get_guard_config", "set_current_angle", "clear_guard_fault")),
    3: (("检查两轴路径和制动余量；电机1固定±35°，降低速度或修改路径，不扩大范围绕过保护。",), ("get_guard_config", "get_axis_positions", "prepare_business_trajectory")),
    4: (("检查实际位置、限位输入及断线；解除原因后才能确认清除，软件不提供限位旁路或自动脱离。",), ("get_feedback", "get_guard_config", "clear_guard_fault")),
    5: (_COMM, ("close", "connect", "get_axis_positions")),
    6: (("读取具体驱动报警码，按驱动器说明书排除原因，再复位并确认返回0。",), ("read_alarm", "reset_alarm")),
    7: (("记录目标与实际角度、误差和周期；降低速度/加速度、延长轨迹时间，排查负载、间隙和驱动响应。",
         "不要只扩大误差阈值；原因处理并停稳后清除旧轨迹，按实际位置重新准备。"), ("get_trajectory_status", "get_axis_positions", "get_motion_profile", "prepare_business_trajectory")),
    8: (("控制周期超过1秒；检查下位机阻塞、RS485超时及额外读写，修复后重新准备。",), ("get_trajectory_status",)),
    9: (("3秒心跳丢失；保持同一个SDK连接，检查后台续期异常、进程暂停和串口阻塞。",), ("check_auto_rotation",)),
    10: (("首点与实际位置不符；确认返回路径安全后预载返回轨迹并明确启动，或按当前位置重新规划。",), ("get_axis_positions", "prepare_return_to_start", "start_trajectory")),
    11: (("检查点数、时间递增、会话、上传完整性和速度/加速度约束；重新上传并校验。",), ("prepare_trajectory",)),
    12: (("终点收敛超时；核查实际误差、负载、驱动响应和机械卡滞，修复后降低运动强度。",), ("get_axis_positions", "get_motion_profile")),
    13: (("路径长期无法推进；排查持续跟不上、通信延迟或机构卡滞；修复后延长轨迹时间。",), ("get_axis_positions", "get_trajectory_status")),
}
_GUARD = {
    0: ("无单轴保护故障", ("继续检查参考及限位状态。",), ("get_feedback",)),
    1: ("限位/传感器异常", ("检查限位输入、接线、极性和传感器，解除原因；不自动脱离限位。",), ("get_feedback", "get_guard_config")),
    2: ("软范围或回零行程超限", ("核实真实角度、允许范围和回零行程；机构在范围外时需人工安全恢复，不扩大范围或伪造参考。",), ("get_guard_config", "get_feedback")),
    3: ("反馈异常", _REFERENCE, ("get_feedback", "set_current_angle")),
    4: ("回零超时", ("检查传感器能否到达、运动方向和负载；在正确配置下重新回零，不能只增加超时时间。",), ("get_guard_config", "start_homing", "wait_homing")),
    5: ("驱动通信故障", _COMM, ("close", "connect", "get_feedback")),
    6: ("驱动器报警", ("排除驱动报警原因，再复位并确认报警码为0。",), ("read_alarm", "reset_alarm")),
    7: ("回零被取消", ("确认停稳；回零未完成的参考不可用，需重新回零或在已知姿态标定。",), ("start_homing", "set_current_angle")),
    8: ("定位执行异常", ("新位置命令下发后方向相反或3秒没有可测位移，固件已请求停止；核查P4参数与CTRG/HOLD触发条件。禁止循环重发；停稳并处理原因后明确清除保护。",), ("get_feedback", "clear_guard_fault")),
}
_STATUS = {
    0: (("命令成功不代表运动已完成；读回实际位置和运行状态。",), ("get_axis_positions",)),
    1: (_COMM, ("close", "connect")), 2: (_COMM, ("close", "connect")),
    3: (_COMM, ("close", "connect")), 5: (_COMM, ("close", "connect")),
    4: (("检查本次命令参数、会话、状态和固件版本，修正后重试；无需清报警。",), ("get_trajectory_status",)),
    6: (("设备忙或未停稳；先停止已有任务并读回状态，禁止循环重发启动。",), ("stop_trajectory", "stop", "get_feedback")),
    7: (("核对SDK与固件版本；参考角度API需v3.5固件、独立轨迹清除需v3.4；不能以旧设零命令代替任意角度参考。",), ()),
    8: (("检查硬件限位、断线和停止确认；排除原因后再确认单轴保护。",), ("get_feedback", "clear_guard_fault")),
    9: (("检查目标、路径、制动余量与固定范围；使用受保护的角度API，不用原始命令绕过。",), ("get_guard_config", "move_to_angle")),
    10: (_REFERENCE, ("get_feedback", "get_guard_config", "set_current_angle")),
}

# P100S 第39版：PDF第95～100页（印刷页88～93）。不把厂家维修/编码器复位当作普通报警复位。
_DRIVE = {
    1: ("超速", "降低指令速度，增大驱动加减速时间；检查电子齿轮、负载、编码器与UVW接线，必要时由厂家调整增益。"),
    2: ("主电路过压", "检查供电和制动电阻/回路；降低启停频率、增大减速时间；制动元件损坏需维修。"),
    3: ("主电路欠压", "检查供电电压、容量及瞬时掉电；电源正常仍报警时检查驱动电源电路。"),
    4: ("位置超差", "检查指令速度、负载、转矩能力及编码器；由调试人员核对增益和位置偏差设置，不盲目放宽阈值。"),
    5: ("驱动器过热", "停止并冷却，检查散热、环境温度及负载；持续报警需检修驱动器。"),
    6: ("速度放大器饱和", "检查机械卡死和过大负载，消除卡滞或调整电机/驱动器选型。"),
    7: ("驱动禁止异常", "检查CCW/CW驱动禁止输入及接线；不能短接或关闭限位来绕过原因。"),
    8: ("位置偏差计数器溢出", "检查机械卡死、指令异常及电机是否实际响应，先解除原因。"),
    10: ("动力线断开", "断电后由电气人员检查UVW动力线及接触，不带使能插拔。"),
    11: ("IPM模块故障", "检查供电、过热、UVW短路、绝缘、接地和干扰；模块损坏需维修。"),
    13: ("驱动器过负载", "检查负载、抱闸未释放、缺相及振荡；降低启停频率并调整加减速，必要时重新选型。"),
    14: ("制动故障", "检查制动电路，驱动器故障需厂家维修或更换。"),
    18: ("继电器开关故障", "继电器状态异常，联系厂家维修。"),
    19: ("抱闸延时错误", "检查抱闸释放时序和PA94；由调试人员核对实际释放时间后调整，普通报警复位不能替代时序修正。"),
    20: ("EEPROM错误", "联系厂家检修；修复后的型号和参数恢复由专业人员完成，不自动恢复出厂。"),
    21: ("FPGA模块故障", "联系厂家维修或更换驱动器。"),
    23: ("电流采集电路故障", "联系厂家检修电流采集电路或更换驱动器。"),
    28: ("伺服单元过载", "检查持续负载，减小负载或更换匹配功率的伺服单元。"),
    29: ("用户转矩过载", "检查意外负载和机械卡滞，核对PA30/PA31，不直接扩大保护值。"),
    38: ("编码器EEPROM通信失败", "断电检查编码器线及接口；接口电路故障需检修。"),
    39: ("编码器数据CRC错误", "由厂家写入匹配电机的编码器数据；SDK不提供编码器参数写入。"),
    40: ("不支持机型", "核对电机与驱动器兼容性，使用匹配的电机。"),
    41: ("需要切换电机型号", "核实铭牌后由调试人员选择正确电机型号，不自动修改型号。"),
    42: ("AC输入电压过低", "区分正常断电和运行中欠压；运行中检查交流输入及供电连接。"),
    46: ("主电路上电欠压", "检查供电及PA122设置，由电气人员确认；不通过放宽欠压保护掩盖供电问题。"),
    47: ("上电主电路过压", "检查交流输入，供电正常仍报错时检修主电路。"),
    50: ("编码器通信故障", "断电检查编码器接线，按手册重新上电；重新核查参考，丢失时重新真实标定。"),
    51: ("编码器通信异常", "检查松动、断线、屏蔽及干扰，按手册重新上电并核查参考。"),
    52: ("编码器电池电压不足", "按厂家操作规程尽快更换电池；更换后核查位置数据和参考。"),
    53: ("编码器电池电压错误", "更换耗尽的电池，并由厂家指导复位编码器；储存信息已不可信，之后必须重新建立真实参考。"),
    54: ("编码器多圈信息错误", "按厂家流程处理编码器多圈故障并重新上电；重新建立真实参考，不能沿用旧坐标。"),
    55: ("编码器连续CRC错误", "检查编码器通信线路、连接器、屏蔽和干扰。"),
    56: ("MODBUS帧过长", "检查帧长、协议与通信干扰；使用配套固件和SDK，不自动重发运动。"),
    57: ("MODBUS通信格式异常", "核对通信格式、地址、寄存器及数值；修正后仍异常联系厂家。"),
    58: ("单圈位置值错误", "按手册重新上电；再次出现需厂家处理，核查参考后才能运动。"),
    59: ("编码器上报CF错误", "按厂家流程复位编码器并核查/重建真实参考；reset_alarm不是编码器复位接口。"),
}


def get_fault_advice(domain: str, code: int | str, *, motor_id=None, detail="") -> FaultAdvice:
    """domain: trajectory / guard / status / drive / sdk；未知代码也返回保守排查步骤。"""
    unknown = False
    if domain == "trajectory" and code in _TRAJECTORY:
        title = FAULTS[code]
        steps, apis = _TRAJECTORY[code]
        if code:
            steps += ("原因排除、参考有效且两轴停稳后清除旧轨迹；重新准备，明确启动，不续跑旧会话。",)
            apis += ("clear_trajectory_fault",)
    elif domain == "guard" and code in _GUARD:
        title, steps, apis = _GUARD[code]
        if code:
            steps += ("参考仍有效时可确认清除单轴故障；参考失效需真实标定或回零，不能强清。",)
            apis += ("clear_guard_fault",)
    elif domain == "status" and code in _STATUS:
        title = STATUS_TEXT[code]
        steps, apis = _STATUS[code]
        unknown = code in (1, 2, 3, 5)
    elif domain == "drive":
        if code == 0:
            title, steps, apis = "驱动器无报警", ("继续检查软件保护及参考状态。",), ("read_alarm",)
        elif code in _DRIVE:
            name, remedy = _DRIVE[code]
            title = f"驱动器报警 {code}：{name}"
            steps = (remedy,
                     "原因排除后可显式尝试reset_alarm(motor_id)，返回0才算清除；不支持通信复位或仍报警时按手册/厂家流程处理。",
                     "普通报警复位不等于编码器复位，不修复硬件；复位/断电后读回参考，另行处理单轴和轨迹保护。")
            apis = ("read_alarm", "reset_alarm", "get_axis_positions")
        else:
            title = f"未识别的驱动器报警 {code}"
            steps = ("保存原始报警码并核对实际驱动型号及手册；未识别代码不得猜测复位，联系厂家确认。",)
            apis = ("read_alarm",)
    elif domain == "sdk":
        title = {"communication": "通信结果不确定", "parameter": "参数校验失败",
                 "condition": "调用前置条件或等待检查失败"}.get(code, "SDK异常")
        if code == "communication":
            steps, apis, unknown = _COMM, ("close", "connect", "get_axis_positions"), True
        elif code == "parameter":
            steps, apis = ("按异常原文修正单位、数值范围、轨迹点或时间；参数校验异常无需清故障；复合操作仍应读回状态。",), ()
        else:
            steps = ("保留异常原文；停止确认或运动等待超时时先核实机构停止，无法确认时使用硬件急停。",
                     "读取轨迹、参考、单轴保护和驱动报警，按对应故障处理；不要盲目复位或重发运动。")
            apis = ("get_fault_report", "get_axis_positions", "get_trajectory_status")
    else:
        title = f"未识别的{domain}代码 {code}"
        steps = ("保留原始代码、命令及固件版本；确认停止并联系维护人员核对，未知代码不可视为成功或自动重启。",)
        apis = ("get_fault_report",)
    return FaultAdvice(domain, code, title, steps, tuple(dict.fromkeys(apis)), motor_id, detail, unknown)


def explain_error(error: Exception) -> FaultAdvice:
    """把API异常转换为处理建议；不访问串口。"""
    # Local import avoids a controller/catalog import cycle.
    from .motor_controller import CommandRejected, CommunicationError, DriveAlarm
    if isinstance(error, CommandRejected):
        return get_fault_advice("status", error.status, motor_id=error.response.device_id,
                                detail=f"command=0x{error.response.command:02X}; {error}")
    if isinstance(error, DriveAlarm):
        return get_fault_advice("drive", error.alarm_code, motor_id=error.motor_id, detail=str(error))
    if isinstance(error, CommunicationError):
        return get_fault_advice("sdk", "communication", detail=str(error))
    if isinstance(error, ValueError):
        return get_fault_advice("sdk", "parameter", detail=str(error))
    return get_fault_advice("sdk", "condition", detail=str(error))


@dataclass(frozen=True)
class FaultReport:
    issues: tuple[FaultAdvice, ...]
    complete: bool
    trajectory: object | None = None
    axes: tuple = ()


def collect_fault_report(controller) -> FaultReport:
    """只查询，不发送停止/复位/参考设置；非原子快照，不作为启动许可。"""
    issues = []
    trajectory, axes, complete = None, (), True
    try:
        trajectory = controller.get_trajectory_status()
        if trajectory.fault:
            issues.append(get_fault_advice("trajectory", trajectory.fault))
    except Exception as exc:
        issues.append(explain_error(exc)); complete = False
    try:
        axes = controller.get_axis_positions()
        for axis, position in enumerate(axes, 1):
            if position.alarm_code:
                issues.append(get_fault_advice("drive", position.alarm_code, motor_id=axis))
            if position.guard_fault:
                issues.append(get_fault_advice("guard", position.guard_fault, motor_id=axis))
            if not position.usable:
                complete = False
                advice = get_fault_advice("trajectory", 2, motor_id=axis,
                    detail=f"反馈不可用: source={position.source}, age_ms={position.sample_age_ms}")
                issues.append(advice)
    except Exception as exc:
        issues.append(explain_error(exc)); complete = False
    return FaultReport(tuple(issues), complete, trajectory, axes)
