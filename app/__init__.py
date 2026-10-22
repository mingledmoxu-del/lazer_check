"""双电机控制接口；导入时不创建界面、不连接串口、不启动电机。"""

from .motor_controller import (
    MotorController, MotorError, CommunicationError, CommandRejected,
    DriveAlarm, MotorStatus, MotionAccepted, MotionProfile, ContinuousRotation,
    PositionFeedback, GuardConfig,
)
from .trajectory import TrajectoryPoint, TrajectoryStatus, AxisPosition, joint_ellipse
from .faults import FaultAdvice, FaultReport, get_fault_advice, explain_error

__all__ = [
    "MotorController", "MotorError", "CommunicationError", "CommandRejected",
    "DriveAlarm", "MotorStatus", "MotionAccepted", "MotionProfile", "ContinuousRotation",
    "PositionFeedback", "GuardConfig",
    "TrajectoryPoint", "TrajectoryStatus", "AxisPosition", "joint_ellipse",
    "FaultAdvice", "FaultReport", "get_fault_advice", "explain_error",
]
