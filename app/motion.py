"""减速器输出轴运动参数换算。"""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class MotionConversion:
    output_speed_rpm: float
    output_angle_deg: float
    gear_ratio: float
    motor_speed_rpm: int
    motor_angle_deg: float
    pulses: int

    @property
    def actual_output_speed_rpm(self) -> float:
        return self.motor_speed_rpm / self.gear_ratio


def convert_output_motion(
    output_speed_rpm: float,
    output_angle_deg: float,
    pulses_per_motor_rev: int,
    gear_ratio: float,
) -> MotionConversion:
    """把减速器输出端的速度和角度换算为电机指令。"""
    values = (output_speed_rpm, output_angle_deg, gear_ratio)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("速度、角度和减速比必须是有限数字。")
    if output_speed_rpm <= 0:
        raise ValueError("输出轴速度必须大于 0 rpm。")
    if not 0 < output_angle_deg <= 36000:
        raise ValueError("输出轴角度必须在 0～36000°之间。")
    if not 0.001 <= gear_ratio <= 10000:
        raise ValueError("减速比必须在 0.001～10000 之间。")
    if not 1 <= pulses_per_motor_rev <= 1_000_000:
        raise ValueError("PA11 每圈指令脉冲必须在 1～1000000 之间。")

    motor_speed = int(math.floor(output_speed_rpm * gear_ratio + 0.5))
    if not 1 <= motor_speed <= 5000:
        raise ValueError("换算后的电机速度必须在 1～5000 rpm 之间。")

    motor_angle = output_angle_deg * gear_ratio
    pulses = int(math.floor(motor_angle / 360.0 * pulses_per_motor_rev + 0.5))
    if pulses == 0 or pulses > 2**31 - 1:
        raise ValueError("换算后的脉冲数为零或超出 32 位范围。")

    return MotionConversion(
        output_speed_rpm=output_speed_rpm,
        output_angle_deg=output_angle_deg,
        gear_ratio=gear_ratio,
        motor_speed_rpm=motor_speed,
        motor_angle_deg=motor_angle,
        pulses=pulses,
    )
