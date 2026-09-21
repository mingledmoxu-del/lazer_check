"""主界面窗口 (Main Window).

整合串口配置、双伺服状态监视器、PA 参数读写面板、通信日志以及底部状态栏.
符合工业控制软件简洁、清晰、高可靠风格.
"""

from datetime import datetime
import os
from pathlib import Path
from typing import Optional, Dict
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QCloseEvent, QFont, QColor
from PySide6.QtWidgets import (
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QGroupBox,
    QLabel,
    QPushButton,
    QCheckBox,
    QSpinBox,
    QTextEdit,
    QStatusBar,
    QFileDialog,
    QMessageBox,
    QSplitter,
    QFrame,
    QTabWidget,
)

from communication.serial_manager import SerialManager
from servo.servo_controller import ServoController
from gui.serial_panel import SerialPanel
from gui.servo_panel import ServoPanel
from gui.motion_panel import MotionPanel
from utils.logger import CommLogger


class MotorMonitorCard(QGroupBox):
    """单台电机状态实时显示卡片 (Motor 1 / Motor 2)."""

    def __init__(self, title: str, device_id: int, parent: Optional[QWidget] = None):
        super().__init__(title, parent)
        self.device_id = device_id
        self._init_ui()

    def _init_ui(self) -> None:
        layout = QGridLayout(self)
        layout.setContentsMargins(10, 15, 10, 10)
        layout.setHorizontalSpacing(10)
        layout.setVerticalSpacing(6)

        # 通信状态
        layout.addWidget(QLabel("通信状态:"), 0, 0)
        self.lbl_comm = QLabel("Offline (离线)")
        self.lbl_comm.setStyleSheet("font-weight: bold; color: #757575;")
        layout.addWidget(self.lbl_comm, 0, 1)

        # 伺服使能
        layout.addWidget(QLabel("伺服使能:"), 1, 0)
        self.lbl_servo = QLabel("OFF")
        self.lbl_servo.setStyleSheet("font-weight: bold; color: #757575;")
        layout.addWidget(self.lbl_servo, 1, 1)

        # 当前转速
        layout.addWidget(QLabel("当前转速:"), 2, 0)
        self.lbl_speed = QLabel("0 rpm")
        self.lbl_speed.setStyleSheet("font-weight: bold; font-family: Consolas; color: #1565c0;")
        layout.addWidget(self.lbl_speed, 2, 1)

        # 当前位置
        layout.addWidget(QLabel("当前位置:"), 3, 0)
        self.lbl_pos = QLabel("0")
        self.lbl_pos.setStyleSheet("font-weight: bold; font-family: Consolas; color: #2e7d32;")
        layout.addWidget(self.lbl_pos, 3, 1)

        # 报警状态
        layout.addWidget(QLabel("伺服报警:"), 4, 0)
        self.lbl_alarm = QLabel("None (正常)")
        self.lbl_alarm.setStyleSheet("font-weight: bold; color: #2e7d32;")
        layout.addWidget(self.lbl_alarm, 4, 1)

    def update_state(self, state: dict) -> None:
        """根据控制器上报的字典刷新卡片显示."""
        is_online = state.get("is_online", False)
        if is_online:
            self.lbl_comm.setText("Online (在线)")
            self.lbl_comm.setStyleSheet("font-weight: bold; color: #2e7d32;")
        else:
            self.lbl_comm.setText("Offline (离线)")
            self.lbl_comm.setStyleSheet("font-weight: bold; color: #d32f2f;")

        servo_on = state.get("servo_on", False)
        self.lbl_servo.setText("ON (使能)" if servo_on else "OFF (去使能)")
        self.lbl_servo.setStyleSheet(
            "font-weight: bold; color: #2e7d32;" if servo_on else "font-weight: bold; color: #757575;"
        )
        self.lbl_speed.setText(f"{state.get('speed_rpm', 0)} rpm")
        self.lbl_pos.setText(str(state.get("position", 0)))
        alarm = state.get("alarm_code", 0)
        if alarm == 0:
            self.lbl_alarm.setText("None (正常)")
            self.lbl_alarm.setStyleSheet("font-weight: bold; color: #2e7d32;")
        else:
            self.lbl_alarm.setText(f"ALM: 0x{alarm:04X}")
            self.lbl_alarm.setStyleSheet("font-weight: bold; color: #c62828; background: #ffcdd2; padding: 2px 4px;")


class MainWindow(QMainWindow):
    """上位机软件主窗口."""

    def __init__(self, serial_mgr: SerialManager, controller: ServoController, logger: CommLogger):
        super().__init__()
        self.serial_mgr = serial_mgr
        self.controller = controller
        self.logger = logger

        self.setWindowTitle("Servo Motor Control System - 双伺服电机控制系统")
        self.resize(1180, 820)
        self.setMinimumSize(960, 680)

        self._init_ui()
        self._bind_signals()
        self._update_status_bar()

    def _init_ui(self) -> None:
        """构建整体用户界面."""
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        main_vbox = QVBoxLayout(central_widget)
        main_vbox.setContentsMargins(8, 8, 8, 8)
        main_vbox.setSpacing(8)

        # 1. 顶部串口控制面板
        self.serial_panel = SerialPanel(self.serial_mgr, self)
        main_vbox.addWidget(self.serial_panel)

        # 2. 中间主体区域 (使用 Splitter 分割: 左侧状态与控制, 右侧参数配置)
        center_splitter = QSplitter(Qt.Horizontal)

        # 左侧控制与双伺服状态卡片
        left_widget = QWidget()
        left_vbox = QVBoxLayout(left_widget)
        left_vbox.setContentsMargins(0, 0, 0, 0)
        left_vbox.setSpacing(8)

        # 双伺服状态监控器
        monitor_box = QGroupBox("双伺服状态监视 (Servo Monitor)")
        monitor_layout = QVBoxLayout(monitor_box)
        monitor_layout.setContentsMargins(8, 12, 8, 8)
        self.card_motor1 = MotorMonitorCard("Motor 1 (Slave 1)", 1)
        self.card_motor2 = MotorMonitorCard("Motor 2 (Slave 2)", 2)
        monitor_layout.addWidget(self.card_motor1)
        monitor_layout.addWidget(self.card_motor2)

        # 自动轮询控制
        poll_row = QHBoxLayout()
        self.chk_polling = QCheckBox("启用定时状态轮询")
        self.spin_poll_interval = QSpinBox()
        self.spin_poll_interval.setRange(100, 5000)
        self.spin_poll_interval.setSingleStep(100)
        self.spin_poll_interval.setValue(500)
        self.spin_poll_interval.setSuffix(" ms")
        poll_row.addWidget(self.chk_polling)
        poll_row.addWidget(self.spin_poll_interval)
        poll_row.addStretch()
        monitor_layout.addLayout(poll_row)
        capability_label = QLabel("当前下位机固件：完整双伺服控制协议已启用")
        capability_label.setStyleSheet("color: #2e7d32; font-weight: bold;")
        capability_label.setWordWrap(True)
        monitor_layout.addWidget(capability_label)
        left_vbox.addWidget(monitor_box)

        # 基本运动与急停控制操作区
        ctrl_box = QGroupBox("伺服基础控制 (Motion Control)")
        ctrl_layout = QGridLayout(ctrl_box)
        ctrl_layout.setContentsMargins(8, 12, 8, 8)
        ctrl_layout.setSpacing(8)

        self.btn_servo_on = QPushButton("Servo ON (使能)")
        self.btn_servo_on.setStyleSheet("background-color: #2e7d32; color: white; font-weight: bold; padding: 6px;")
        self.btn_servo_on.setToolTip("仅用于 PA53=1 软件强制使能；使用 DB44 外部 SON 时不需要点击")
        self.btn_servo_off = QPushButton("Servo OFF (去使能)")
        self.btn_servo_off.setStyleSheet("background-color: #616161; color: white; font-weight: bold; padding: 6px;")
        self.btn_servo_off.setToolTip("恢复 PA53=0；外部 SON 触点闭合时驱动器仍可能保持使能")

        self.btn_start = QPushButton("Start (启动)")
        self.btn_start.setStyleSheet("font-weight: bold; padding: 6px;")
        self.btn_stop = QPushButton("Stop (停止)")
        self.btn_stop.setStyleSheet("background-color: #f57c00; color: white; font-weight: bold; padding: 6px;")

        self.btn_reset_alarm = QPushButton("Reset Alarm (复位报警)")
        self.btn_reset_alarm.setStyleSheet("padding: 6px;")

        self.btn_estop = QPushButton("EMERGENCY STOP (急停)")
        self.btn_estop.setStyleSheet("background-color: #b71c1c; color: white; font-size: 14px; font-weight: bold; padding: 10px;")

        ctrl_layout.addWidget(self.btn_servo_on, 0, 0)
        ctrl_layout.addWidget(self.btn_servo_off, 0, 1)
        ctrl_layout.addWidget(self.btn_start, 1, 0)
        ctrl_layout.addWidget(self.btn_stop, 1, 1)
        ctrl_layout.addWidget(self.btn_reset_alarm, 2, 0, 1, 2)
        ctrl_layout.addWidget(self.btn_estop, 3, 0, 1, 2)
        left_vbox.addWidget(ctrl_box)

        left_vbox.addStretch()
        left_widget.setMinimumWidth(320)
        left_widget.setMaximumWidth(420)
        center_splitter.addWidget(left_widget)

        # 右侧: 选项卡 (Tab 1: 运动与角度/圈数控制, Tab 2: PA 参数读写与配置)
        self.right_tab_widget = QTabWidget()
        self.motion_panel = MotionPanel(self.controller, self)
        self.servo_panel = ServoPanel(self.controller, self)
        self.right_tab_widget.addTab(self.motion_panel, "🎯 运动控制与定角旋转 (Motion & Angle/Turns)")
        self.right_tab_widget.addTab(self.servo_panel, "⚙️ PA 参数读写与批量配置 (Servo Parameters)")
        self.right_tab_widget.setCurrentWidget(self.motion_panel)

        center_splitter.addWidget(self.right_tab_widget)
        center_splitter.setStretchFactor(0, 0)
        center_splitter.setStretchFactor(1, 1)

        main_vbox.addWidget(center_splitter, stretch=1)

        # 3. 底部通信日志面板 (可折叠/伸缩)
        log_box = QGroupBox("通信日志 (Communication Log)")
        log_layout = QVBoxLayout(log_box)
        log_layout.setContentsMargins(8, 10, 8, 6)
        log_layout.setSpacing(6)

        self.txt_log = QTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setFont(QFont("Consolas", 9))
        self.txt_log.setStyleSheet("background-color: #1e1e1e; color: #d4d4d4;")
        self.txt_log.setFixedHeight(140)
        log_layout.addWidget(self.txt_log)

        log_btn_row = QHBoxLayout()
        self.btn_clear_log = QPushButton("清空日志 (Clear Log)")
        self.btn_save_log = QPushButton("导出日志文件 (Save Log)")
        log_btn_row.addWidget(self.btn_clear_log)
        log_btn_row.addWidget(self.btn_save_log)
        log_btn_row.addStretch()
        log_layout.addLayout(log_btn_row)

        main_vbox.addWidget(log_box)

        # 4. 底部状态栏 (COM | 波特率 | STM32状态 | 电机在线状态)
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)

    def _bind_signals(self) -> None:
        """事件与信号绑定."""
        # 串口打开与关闭
        self.serial_panel.open_requested.connect(self._on_open_serial)
        self.serial_panel.close_requested.connect(self._on_close_serial)

        # 串口状态变动
        self.serial_mgr.connected.connect(self._on_serial_connected)
        self.serial_mgr.disconnected.connect(self._on_serial_disconnected)
        self.serial_mgr.error_occurred.connect(lambda err: self.append_log(f"[ERROR] {err}"))

        # 控制器信号
        self.controller.packet_sent.connect(self._on_packet_sent)
        self.controller.packet_received.connect(self._on_packet_received)
        self.controller.device_status_changed.connect(self._on_device_status_changed)
        self.controller.log_message.connect(self._on_controller_log)

        # 双向同步目标电机选择
        self.motion_panel.combo_motor.currentIndexChanged.connect(self.servo_panel.combo_motor.setCurrentIndex)
        self.servo_panel.combo_motor.currentIndexChanged.connect(self.motion_panel.combo_motor.setCurrentIndex)

        # 基础动作按钮绑定
        self.btn_servo_on.clicked.connect(lambda: self.controller.set_servo_power(self.servo_panel.get_selected_motor_id(), True))
        self.btn_servo_off.clicked.connect(lambda: self.controller.set_servo_power(self.servo_panel.get_selected_motor_id(), False))
        self.btn_start.clicked.connect(self._on_start_motor)
        self.btn_stop.clicked.connect(lambda: self.controller.stop_motor(self.servo_panel.get_selected_motor_id()))
        self.btn_reset_alarm.clicked.connect(lambda: self.controller.reset_alarm(self.servo_panel.get_selected_motor_id()))
        self.btn_estop.clicked.connect(self.controller.emergency_stop_all)

        # 轮询配置
        self.chk_polling.toggled.connect(lambda on: self.controller.set_polling(on, self.spin_poll_interval.value()))
        self.spin_poll_interval.valueChanged.connect(lambda v: self.controller.set_polling(self.chk_polling.isChecked(), v))

        # 日志操作
        self.btn_clear_log.clicked.connect(self.txt_log.clear)
        self.btn_save_log.clicked.connect(self._on_save_log_file)

    def _on_open_serial(self, port: str, baud: int, data_bits: int, parity: str, stop_bits: float) -> None:
        """打开串口请求."""
        self.append_log(f"[SYS] 正在连接 {port} (Baud: {baud}, 8N1)...")
        success = self.serial_mgr.open(port, baud, data_bits, parity, stop_bits)
        if not success:
            QMessageBox.critical(self, "连接失败", f"无法打开串口 {port}, 请检查是否被占用或驱动异常")

    def _on_close_serial(self) -> None:
        """关闭串口."""
        self.serial_mgr.close()
        self.append_log("[SYS] 串口已主动关闭")

    def _on_start_motor(self) -> None:
        """使用运动面板当前速度设置目标值后再启动电机."""
        device_id = self.servo_panel.get_selected_motor_id()
        speed_rpm = self.motion_panel.spin_speed.value()
        self.controller.set_motor_speed(device_id, speed_rpm)
        self.controller.start_motor(device_id)

    def _on_serial_connected(self, port: str) -> None:
        """串口成功连接."""
        self.append_log(f"[SYS] 成功连接至 {port}")
        self._update_status_bar()

    def _on_serial_disconnected(self) -> None:
        """串口断开."""
        self.append_log("[SYS] 串口断开连接")
        self._update_status_bar()

    def _on_packet_sent(self, raw: bytes, note: str) -> None:
        """记录 TX 日志."""
        msg = self.logger.log_tx(raw, note)
        self.append_log(f"<span style='color: #4fc3f7;'>{msg}</span>")

    def _on_packet_received(self, raw: bytes, note: str) -> None:
        """记录 RX 日志."""
        msg = self.logger.log_rx(raw, note)
        self.append_log(f"<span style='color: #81c784;'>{msg}</span>")

    def _on_controller_log(self, level: str, text: str) -> None:
        """控制器内部事件日志."""
        color = "#e57373" if level == "ERROR" else ("#ffb74d" if level == "WARN" else "#ce93d8")
        now = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.append_log(f"<span style='color: {color};'>[{now}] [{level}] {text}</span>")

    def _on_device_status_changed(self, device_id: int, state: dict) -> None:
        """刷新双电机卡片与状态栏."""
        if device_id == 1:
            self.card_motor1.update_state(state)
        elif device_id == 2:
            self.card_motor2.update_state(state)
        self._update_status_bar()

    def append_log(self, html_or_text: str) -> None:
        """向底部文本框追加日志 (平滑自动滚动至底部)."""
        self.txt_log.append(html_or_text)
        sb = self.txt_log.verticalScrollBar()
        if sb:
            sb.setValue(sb.maximum())

    def _update_status_bar(self) -> None:
        """更新底部状态栏文字."""
        port = self.serial_mgr.port_name if self.serial_mgr.is_open else "Disconnected"
        baud = self.serial_panel.baud_combo.currentText()
        stm32_status = "STM32 Connected" if self.serial_mgr.is_open else "STM32 Offline"

        m1_online = self.controller.motors[1].is_online
        m2_online = self.controller.motors[2].is_online

        m1_str = "Motor 1 Online" if m1_online else "Motor 1 Offline"
        m2_str = "Motor 2 Online" if m2_online else "Motor 2 Offline"

        status_text = f"串口: {port} | 波特率: {baud} | 状态: {stm32_status} | {m1_str} | {m2_str}"
        self.status_bar.showMessage(status_text)

    def _on_save_log_file(self) -> None:
        """手动保存日志文件."""
        filepath, _ = QFileDialog.getSaveFileName(self, "保存当前日志", "comm_log.txt", "Text Files (*.txt)")
        if filepath:
            try:
                with open(filepath, "w", encoding="utf-8") as f:
                    f.write(self.txt_log.toPlainText())
                QMessageBox.information(self, "导出成功", f"日志已保存至:\n{filepath}")
            except Exception as e:
                QMessageBox.critical(self, "导出失败", str(e))

    def closeEvent(self, event: QCloseEvent) -> None:
        """退出前确保停止轮询并安全关闭串口与后台线程."""
        self.controller.set_polling(False)
        self.serial_mgr.close()
        event.accept()
