"""P100S 伺服驱动器 PA 参数表与映射工具.

根据设计要求预留常用 PA 参数字典, 避免用户手动换算 Modbus 寄存器地址.
"""

import re
from typing import Dict, Optional, Any

# 常用伺服参数字典定义 (可根据实际 P100S 手册随时扩充)
SERVO_PARAMETERS: Dict[int, Dict[str, Any]] = {
    1: {
        "name": "PA1",
        "description": "控制模式 / 基本控制参数",
        "default": 60,
        "min": 0,
        "max": 65535,
    },
    21: {
        "name": "PA21",
        "description": "核心控制参数 / 速度/扭矩限额",
        "default": 60,
        "min": 0,
        "max": 65535,
    },
    53: {
        "name": "PA53",
        "description": "使能/运行控制标志",
        "default": 1,
        "min": 0,
        "max": 65535,
    },
    71: {
        "name": "PA71",
        "description": "RS485 从站地址 (Slave ID, 默认 Motor1=1, Motor2=2)",
        "default": 1,
        "min": 1,
        "max": 247,
    },
    72: {
        "name": "PA72",
        "description": "RS485 通信波特率 (96=9600, 1152=115200 等)",
        "default": 96,
        "min": 1,
        "max": 115200,
    },
    73: {
        "name": "PA73",
        "description": "RS485 校验与协议格式",
        "default": 0,
        "min": 0,
        "max": 10,
    },
}


def parse_pa_number(input_str: str) -> Optional[int]:
    """将用户输入的参数字符串自动解析为数字 PA 编号.

    支持:
    - 'PA21' -> 21
    - 'pa21' -> 21
    - '21'   -> 21
    - 'PA01' -> 1
    若格式不合法则返回 None.
    """
    if not input_str:
        return None
    cleaned = input_str.strip()
    match = re.match(r"^(?:PA|pa)?(\d+)$", cleaned)
    if match:
        try:
            return int(match.group(1))
        except ValueError:
            return None
    return None


def format_pa_string(pa_num: int) -> str:
    """将整数转换为标准 'PAxx' 格式."""
    return f"PA{pa_num}"


def get_parameter_desc(pa_num: int) -> str:
    """获取参数中文描述, 若未知则返回默认说明."""
    if pa_num in SERVO_PARAMETERS:
        return SERVO_PARAMETERS[pa_num].get("description", "")
    return "自定义驱动器参数"
