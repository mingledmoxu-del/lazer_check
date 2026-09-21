"""伺服电机运动控制面板 (速度、旋转圈数、角度与点动微调).

支持:
1. 目标速度设置 (rpm) 与常用速度一键切换
2. 连续正转/反转/停止控制
3. 定角运动与定圈运动 (角度与圈数实时双向联动折算为脉冲数)
4. 常用角度快捷按钮 (45°, 90°, 180°, 360°, 720°)
5. 相对运动与绝对位置运动
6. 点动步进微调 (Jog)
7. 实时转速、位置脉冲、当前机械角度与累计圈数显示
"""

from typing import Optional
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QWidget,
    QGroupBox,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QFormLayout,
    QLabel,
    QComboBox,
    QSpinBox,
    QDoubleSpinBox,
    QPushButton,
    QRadioButton,
    QButtonGroup,
    QMessageBox,
    QFrame,
)

from communication.protocol import ResponsePacket, Command
from servo.servo_controller import ServoController


class MotionPanel(QWidget):
    """运动控制与角度/圈数精确定位面板."""

    def __init__(self, controller: ServoController, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.controller = controller
        self._updating_inputs = False
        self._init_ui()
        self._bind_events()

    def _init_ui(self) -> None:
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(12)

        # 1. 目标电机选择与电机机械参数
        header_box = QGroupBox("控制目标与机械参数 (Target Motor & Parameters)")
        header_layout = QHBoxLayout(header_box)

        header_layout.addWidget(QLabel("目标电机:"))
        self.combo_motor = QComboBox()
        self.combo_motor.addItems(["Motor 1 (从站地址 1)", "Motor 2 (从站地址 2)"])
        header_layout.addWidget(self.combo_motor)

        header_layout.addSpacing(20)
        header_layout.addWidget(QLabel("每转指令脉冲数 (PA11):"))
        self.spin_pulses_per_rev = QSpinBox()
        self.spin_pulses_per_rev.setRange(100, 1000000)
        self.spin_pulses_per_rev.setSingleStep(1000)
        self.spin_pulses_per_rev.setValue(10000)  # P100S 默认 10000 脉冲/圈
        self.spin_pulses_per_rev.setSuffix(" pulse/rev")
        header_layout.addWidget(self.spin_pulses_per_rev)

        header_layout.addStretch()
        main_layout.addWidget(header_box)

        # 2. 速度设置与连续运行控制
        speed_box = QGroupBox("速度控制 (Speed & Velocity Run)")
        speed_layout = QVBoxLayout(speed_box)

        row1 = QHBoxLayout()
        row1.addWidget(QLabel("目标运行转速:"))
        self.spin_speed = QSpinBox()
        self.spin_speed.setRange(10, 3000)
        self.spin_speed.setSingleStep(50)
        self.spin_speed.setValue(500)
        self.spin_speed.setSuffix(" rpm")
        self.spin_speed.setFixedWidth(130)
        row1.addWidget(self.spin_speed)

        # 常用转速预设按钮
        row1.addWidget(QLabel("快捷预设:"))
        for spd in [100, 300, 500, 1000, 1500, 2000]:
            btn = QPushButton(f"{spd}")
            btn.setFixedWidth(55)
            btn.clicked.connect(lambda _, s=spd: self.spin_speed.setValue(s))
            row1.addWidget(btn)

        row1.addStretch()
        speed_layout.addLayout(row1)

        row2 = QHBoxLayout()
        self.btn_set_speed = QPushButton("设定运行速度 (Set Speed)")
        self.btn_set_speed.setStyleSheet("padding: 5px 12px;")

        self.btn_run_cw = QPushButton("持续正向运转 (Run CW)")
        self.btn_run_cw.setStyleSheet("background-color: #2e7d32; color: white; font-weight: bold; padding: 5px 12px;")

        self.btn_run_ccw = QPushButton("持续反向运转 (Run CCW)")
        self.btn_run_ccw.setStyleSheet("background-color: #388e3c; color: white; font-weight: bold; padding: 5px 12px;")

        self.btn_speed_stop = QPushButton("平滑停止 (Stop)")
        self.btn_speed_stop.setStyleSheet("background-color: #d32f2f; color: white; font-weight: bold; padding: 5px 12px;")

        row2.addWidget(self.btn_set_speed)
        row2.addWidget(self.btn_run_cw)
        row2.addWidget(self.btn_run_ccw)
        row2.addWidget(self.btn_speed_stop)
        row2.addStretch()
        speed_layout.addLayout(row2)

        main_layout.addWidget(speed_box)

        # 3. 定角与定圈定位控制 (核心功能)
        pos_box = QGroupBox("定角 / 定圈旋转运动 (Angle & Revolutions Positioning)")
        pos_layout = QVBoxLayout(pos_box)

        form_grid = QGridLayout()
        form_grid.setHorizontalSpacing(15)
        form_grid.setVerticalSpacing(8)

        # 旋转圈数输入
        form_grid.addWidget(QLabel("旋转圈数 (Revolutions):"), 0, 0)
        self.spin_turns = QDoubleSpinBox()
        self.spin_turns.setRange(-30000.0, 30000.0)
        self.spin_turns.setDecimals(3)
        self.spin_turns.setSingleStep(0.25)
        self.spin_turns.setValue(1.0)
        self.spin_turns.setSuffix(" 圈 (rev)")
        self.spin_turns.setFixedWidth(160)
        form_grid.addWidget(self.spin_turns, 0, 1)

        # 旋转角度输入
        form_grid.addWidget(QLabel("旋转角度 (Degrees):"), 0, 2)
        self.spin_angle = QDoubleSpinBox()
        self.spin_angle.setRange(-10800000.0, 10800000.0)
        self.spin_angle.setDecimals(1)
        self.spin_angle.setSingleStep(15.0)
        self.spin_angle.setValue(360.0)
        self.spin_angle.setSuffix(" ° (度)")
        self.spin_angle.setFixedWidth(160)
        form_grid.addWidget(self.spin_angle, 0, 3)

        # 换算脉冲数显示
        form_grid.addWidget(QLabel("折算指令脉冲数:"), 1, 0)
        self.lbl_calculated_pulses = QLabel("10000 pulse")
        self.lbl_calculated_pulses.setStyleSheet("font-family: Consolas; font-weight: bold; color: #1565c0;")
        form_grid.addWidget(self.lbl_calculated_pulses, 1, 1)

        # 定位模式 (相对运动 / 绝对运动)
        mode_box = QHBoxLayout()
        self.radio_relative = QRadioButton("相对位置 (Relative, 增量转动)")
        self.radio_absolute = QRadioButton("绝对位置 (Absolute, 旋转到目标坐标)")
        self.radio_relative.setChecked(True)
        self.pos_mode_group = QButtonGroup(self)
        self.pos_mode_group.addButton(self.radio_relative, 0)
        self.pos_mode_group.addButton(self.radio_absolute, 1)
        mode_box.addWidget(self.radio_relative)
        mode_box.addWidget(self.radio_absolute)
        form_grid.addLayout(mode_box, 1, 2, 1, 2)

        pos_layout.addLayout(form_grid)

        # 常用角度快捷按钮栏
        quick_angle_row = QHBoxLayout()
        quick_angle_row.addWidget(QLabel("角度快捷选择:"))
        angles = [
            ("45°", 45.0),
            ("90° (1/4圈)", 90.0),
            ("180° (半圈)", 180.0),
            ("270°", 270.0),
            ("360° (1圈)", 360.0),
            ("720° (2圈)", 720.0),
            ("1800° (5圈)", 1800.0),
        ]
        for text, ang in angles:
            btn = QPushButton(text)
            btn.clicked.connect(lambda _, a=ang: self._set_angle_directly(a))
            quick_angle_row.addWidget(btn)
        quick_angle_row.addStretch()
        pos_layout.addLayout(quick_angle_row)

        # 运动操作按钮栏
        action_row = QHBoxLayout()
        self.btn_execute_motion = QPushButton("▶ 开始旋转运动 (Start Motion)")
        self.btn_execute_motion.setStyleSheet(
            "background-color: #0277bd; color: white; font-size: 13px; font-weight: bold; padding: 7px 16px;"
        )

        self.btn_reverse_motion = QPushButton("◀ 反向同量旋转 (Reverse Move)")
        self.btn_reverse_motion.setStyleSheet(
            "background-color: #00838f; color: white; font-size: 13px; font-weight: bold; padding: 7px 16px;"
        )

        self.btn_home = QPushButton("回原点 / 绝对零位 (Home 0°)")
        self.btn_home.setStyleSheet("font-weight: bold; padding: 7px 16px;")

        action_row.addWidget(self.btn_execute_motion)
        action_row.addWidget(self.btn_reverse_motion)
        action_row.addWidget(self.btn_home)
        action_row.addStretch()
        pos_layout.addLayout(action_row)

        main_layout.addWidget(pos_box)

        # 4. 点动步进微调控制 (Jog)
        jog_box = QGroupBox("点动微调 (Jog Control)")
        jog_layout = QHBoxLayout(jog_box)

        jog_layout.addWidget(QLabel("微调步长:"))
        self.combo_jog_step = QComboBox()
        self.combo_jog_step.addItems([
            "1° (微小步进)",
            "5° 步进",
            "15° 步进",
            "45° 步进",
            "90° 步进 (1/4圈)",
            "360° 步进 (1圈)",
            "100 脉冲",
            "1000 脉冲",
        ])
        self.combo_jog_step.setCurrentIndex(1)  # 默认 5°
        self.combo_jog_step.setFixedWidth(160)
        jog_layout.addWidget(self.combo_jog_step)

        self.btn_jog_ccw = QPushButton("◀ 逆时针点动 (-)")
        self.btn_jog_ccw.setStyleSheet("font-weight: bold; padding: 6px 14px;")

        self.btn_jog_cw = QPushButton("顺时针点动 (+) ▶")
        self.btn_jog_cw.setStyleSheet("font-weight: bold; padding: 6px 14px;")

        jog_layout.addWidget(self.btn_jog_ccw)
        jog_layout.addWidget(self.btn_jog_cw)
        jog_layout.addStretch()

        main_layout.addWidget(jog_box)

        # 5. 当前实时运动状态看板
        status_box = QGroupBox("当前电机运动反馈 (Live Feedback)")
        status_grid = QGridLayout(status_box)
        status_grid.setContentsMargins(15, 12, 15, 10)
        status_grid.setHorizontalSpacing(25)

        status_grid.addWidget(QLabel("当前转速:"), 0, 0)
        self.lbl_feedback_speed = QLabel("0 rpm")
        self.lbl_feedback_speed.setStyleSheet("font-family: Consolas; font-size: 14px; font-weight: bold; color: #1565c0;")
        status_grid.addWidget(self.lbl_feedback_speed, 0, 1)

        status_grid.addWidget(QLabel("绝对脉冲坐标:"), 0, 2)
        self.lbl_feedback_pos = QLabel("0 pulse")
        self.lbl_feedback_pos.setStyleSheet("font-family: Consolas; font-size: 14px; font-weight: bold; color: #2e7d32;")
        status_grid.addWidget(self.lbl_feedback_pos, 0, 3)

        status_grid.addWidget(QLabel("累计旋转圈数:"), 1, 0)
        self.lbl_feedback_turns = QLabel("0.000 rev")
        self.lbl_feedback_turns.setStyleSheet("font-family: Consolas; font-size: 14px; font-weight: bold; color: #6a1b9a;")
        status_grid.addWidget(self.lbl_feedback_turns, 1, 1)

        status_grid.addWidget(QLabel("单圈机械角度:"), 1, 2)
        self.lbl_feedback_angle = QLabel("0.0 °")
        self.lbl_feedback_angle.setStyleSheet("font-family: Consolas; font-size: 14px; font-weight: bold; color: #e65100;")
        status_grid.addWidget(self.lbl_feedback_angle, 1, 3)

        main_layout.addWidget(status_box)
        main_layout.addStretch()

    def _bind_events(self) -> None:
        """绑定信号槽与事件处理."""
        # 圈数与角度联动
        self.spin_turns.valueChanged.connect(self._on_turns_changed)
        self.spin_angle.valueChanged.connect(self._on_angle_changed)
        self.spin_pulses_per_rev.valueChanged.connect(self._recalculate_pulses)

        # 速度运转按钮
        self.btn_set_speed.clicked.connect(self._on_set_speed_clicked)
        self.btn_run_cw.clicked.connect(self._on_run_cw_clicked)
        self.btn_run_ccw.clicked.connect(self._on_run_ccw_clicked)
        self.btn_speed_stop.clicked.connect(self._on_stop_clicked)

        # 定位运动按钮
        self.btn_execute_motion.clicked.connect(lambda: self._execute_motion(forward=True))
        self.btn_reverse_motion.clicked.connect(lambda: self._execute_motion(forward=False))
        self.btn_home.clicked.connect(self._on_home_clicked)

        # 点动按钮
        self.btn_jog_cw.clicked.connect(lambda: self._execute_jog(direction=1))
        self.btn_jog_ccw.clicked.connect(lambda: self._execute_jog(direction=0))

        # 控制器设备状态更新
        self.controller.device_status_changed.connect(self._on_device_status_changed)

        # 初始计算脉冲
        self._recalculate_pulses()

    def get_selected_motor_id(self) -> int:
        return 1 if self.combo_motor.currentIndex() == 0 else 2

    def set_selected_motor_id(self, motor_id: int) -> None:
        self.combo_motor.setCurrentIndex(0 if motor_id == 1 else 1)

    def _on_turns_changed(self, turns: float) -> None:
        """当圈数改变时，自动折算对应角度并刷新脉冲数."""
        if self._updating_inputs:
            return
        self._updating_inputs = True
        calc_angle = turns * 360.0
        self.spin_angle.setValue(calc_angle)
        self._recalculate_pulses()
        self._updating_inputs = False

    def _on_angle_changed(self, angle: float) -> None:
        """当角度改变时，自动折算对应圈数并刷新脉冲数."""
        if self._updating_inputs:
            return
        self._updating_inputs = True
        calc_turns = angle / 360.0
        self.spin_turns.setValue(calc_turns)
        self._recalculate_pulses()
        self._updating_inputs = False

    def _set_angle_directly(self, angle: float) -> None:
        """快捷角度按钮直接赋值."""
        self.spin_angle.setValue(angle)

    def _recalculate_pulses(self) -> None:
        """根据当前圈数与每转指令脉冲数计算总脉冲."""
        ppr = self.spin_pulses_per_rev.value()
        turns = self.spin_turns.value()
        total_pulses = int(round(turns * ppr))
        self.lbl_calculated_pulses.setText(f"{total_pulses:,} pulse")

    def _on_set_speed_clicked(self) -> None:
        """设置电机转速."""
        dev_id = self.get_selected_motor_id()
        spd = self.spin_speed.value()
        self.controller.set_motor_speed(dev_id, spd)

    def _on_run_cw_clicked(self) -> None:
        """持续正向运转."""
        dev_id = self.get_selected_motor_id()
        spd = self.spin_speed.value()
        self.controller.set_motor_speed(dev_id, spd)
        self.controller.start_motor(dev_id)

    def _on_run_ccw_clicked(self) -> None:
        """持续反向运转."""
        dev_id = self.get_selected_motor_id()
        spd = -abs(self.spin_speed.value())
        self.controller.set_motor_speed(dev_id, spd)
        self.controller.start_motor(dev_id)

    def _on_stop_clicked(self) -> None:
        """停止电机."""
        dev_id = self.get_selected_motor_id()
        self.controller.stop_motor(dev_id)

    def _execute_motion(self, forward: bool = True) -> None:
        """执行定角/定圈旋转."""
        dev_id = self.get_selected_motor_id()
        spd = self.spin_speed.value()
        ppr = self.spin_pulses_per_rev.value()
        turns = self.spin_turns.value()

        if not forward:
            turns = -turns

        pulses = int(round(turns * ppr))

        if self.radio_relative.isChecked():
            # 相对运动
            self.controller.move_relative(dev_id, spd, pulses)
        else:
            # 绝对运动: 将目标圈数/角度直接作为目标绝对脉冲
            self.controller.move_absolute(dev_id, spd, pulses)

    def _on_home_clicked(self) -> None:
        """绝对回零."""
        dev_id = self.get_selected_motor_id()
        spd = self.spin_speed.value()
        reply = QMessageBox.question(
            self,
            "确认回零",
            f"确定要使 Motor {dev_id} 回到绝对零位 (0°) 吗?",
            QMessageBox.Yes | QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.controller.move_absolute(dev_id, spd, 0)

    def _execute_jog(self, direction: int) -> None:
        """执行步进点动."""
        dev_id = self.get_selected_motor_id()
        spd = min(self.spin_speed.value(), 500)  # 点动限制最高 500 rpm 保证安全
        ppr = self.spin_pulses_per_rev.value()

        step_text = self.combo_jog_step.currentText()
        step_pulses = 100

        if "1°" in step_text:
            step_pulses = int(round((1.0 / 360.0) * ppr))
        elif "5°" in step_text:
            step_pulses = int(round((5.0 / 360.0) * ppr))
        elif "15°" in step_text:
            step_pulses = int(round((15.0 / 360.0) * ppr))
        elif "45°" in step_text:
            step_pulses = int(round((45.0 / 360.0) * ppr))
        elif "90°" in step_text:
            step_pulses = int(round((90.0 / 360.0) * ppr))
        elif "360°" in step_text:
            step_pulses = ppr
        elif "100 脉冲" in step_text:
            step_pulses = 100
        elif "1000 脉冲" in step_text:
            step_pulses = 1000

        self.controller.jog(dev_id, direction, spd, step_pulses)

    def _on_device_status_changed(self, device_id: int, state: dict) -> None:
        """实时刷新运动状态看板."""
        if device_id != self.get_selected_motor_id():
            return

        spd = state.get("speed_rpm", 0)
        pos = state.get("position", 0)
        ppr = self.spin_pulses_per_rev.value()

        self.lbl_feedback_speed.setText(f"{spd} rpm")
        self.lbl_feedback_pos.setText(f"{pos:,} pulse")

        if ppr > 0:
            turns = pos / ppr
            angle = (pos % ppr) / ppr * 360.0
            self.lbl_feedback_turns.setText(f"{turns:.3f} rev")
            self.lbl_feedback_angle.setText(f"{angle:.1f} °")
