"""串口配置与连接控制面板 (GUI 部件).

包含串口号、波特率、数据位、停止位、校验选择, 刷新/打开/关闭按钮以及高亮状态指示灯.
"""

from typing import Optional
from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QWidget,
    QGroupBox,
    QHBoxLayout,
    QVBoxLayout,
    QLabel,
    QComboBox,
    QPushButton,
)
from communication.serial_manager import SerialManager


class SerialPanel(QGroupBox):
    """串口连接与配置控件面板."""

    open_requested = Signal(str, int, int, str, float)  # port, baud, data_bits, parity, stop_bits
    close_requested = Signal()

    def __init__(self, serial_mgr: SerialManager, parent: Optional[QWidget] = None):
        super().__init__("串口连接 (Serial Connection)", parent)
        self.serial_mgr = serial_mgr
        self._init_ui()
        self._bind_events()
        self.refresh_ports()
        self.update_status(False)

    def _init_ui(self) -> None:
        """初始化面板布局与控件."""
        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 15, 10, 10)
        layout.setSpacing(12)

        # 1. 串口选择
        layout.addWidget(QLabel("串口号:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(120)
        layout.addWidget(self.port_combo)

        # 刷新按键
        self.btn_refresh = QPushButton("刷新串口")
        self.btn_refresh.setFixedWidth(80)
        layout.addWidget(self.btn_refresh)

        # 2. 波特率
        layout.addWidget(QLabel("波特率:"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.baud_combo.setCurrentText("115200")
        layout.addWidget(self.baud_combo)

        # 3. 数据位
        layout.addWidget(QLabel("数据位:"))
        self.databits_combo = QComboBox()
        self.databits_combo.addItems(["8", "7"])
        layout.addWidget(self.databits_combo)

        # 4. 停止位
        layout.addWidget(QLabel("停止位:"))
        self.stopbits_combo = QComboBox()
        self.stopbits_combo.addItems(["1", "2"])
        layout.addWidget(self.stopbits_combo)

        # 5. 校验位
        layout.addWidget(QLabel("校验:"))
        self.parity_combo = QComboBox()
        self.parity_combo.addItems(["None", "Even", "Odd"])
        layout.addWidget(self.parity_combo)

        # 6. 打开 / 关闭串口按钮
        self.btn_open = QPushButton("打开串口")
        self.btn_open.setFixedWidth(90)
        self.btn_open.setStyleSheet("background-color: #2e7d32; color: white; font-weight: bold;")
        layout.addWidget(self.btn_open)

        self.btn_close = QPushButton("关闭串口")
        self.btn_close.setFixedWidth(90)
        self.btn_close.setEnabled(False)
        self.btn_close.setStyleSheet("background-color: #c62828; color: white; font-weight: bold;")
        layout.addWidget(self.btn_close)

        # 7. 状态指示灯与文字
        self.lbl_status_led = QLabel("●")
        self.lbl_status_led.setStyleSheet("color: #d32f2f; font-size: 18px;")
        layout.addWidget(self.lbl_status_led)

        self.lbl_status_text = QLabel("Serial Status: Disconnected")
        self.lbl_status_text.setStyleSheet("font-weight: bold; color: #555;")
        layout.addWidget(self.lbl_status_text)

        layout.addStretch()

    def _bind_events(self) -> None:
        """绑定信号槽."""
        self.btn_refresh.clicked.connect(self.refresh_ports)
        self.btn_open.clicked.connect(self._on_open_clicked)
        self.btn_close.clicked.connect(self._on_close_clicked)
        self.serial_mgr.connected.connect(lambda name: self.update_status(True, name))
        self.serial_mgr.disconnected.connect(lambda: self.update_status(False))

    def refresh_ports(self) -> None:
        """刷新物理串口列表, 并预留模拟器选项."""
        current = self.port_combo.currentText()
        self.port_combo.clear()
        ports = self.serial_mgr.list_ports()
        for p in ports:
            self.port_combo.addItem(p)
        # 添加虚拟仿真机选项，方便无硬件调试
        self.port_combo.addItem("VIRTUAL_SIM (虚拟STM32)")

        if current and self.port_combo.findText(current) != -1:
            self.port_combo.setCurrentText(current)
        elif ports:
            self.port_combo.setCurrentIndex(0)
        else:
            self.port_combo.setCurrentText("VIRTUAL_SIM (虚拟STM32)")

    def _on_open_clicked(self) -> None:
        """点击打开串口."""
        port = self.port_combo.currentText().split()[0]
        baud = int(self.baud_combo.currentText())
        data_bits = int(self.databits_combo.currentText())
        parity = self.parity_combo.currentText()
        stop_bits = float(self.stopbits_combo.currentText())
        self.open_requested.emit(port, baud, data_bits, parity, stop_bits)

    def _on_close_clicked(self) -> None:
        """点击关闭串口."""
        self.close_requested.emit()

    def update_status(self, is_connected: bool, port_name: str = "") -> None:
        """更新面板连接指示状态."""
        self.btn_open.setEnabled(not is_connected)
        self.btn_close.setEnabled(is_connected)
        self.port_combo.setEnabled(not is_connected)
        self.baud_combo.setEnabled(not is_connected)
        self.databits_combo.setEnabled(not is_connected)
        self.stopbits_combo.setEnabled(not is_connected)
        self.parity_combo.setEnabled(not is_connected)
        self.btn_refresh.setEnabled(not is_connected)

        if is_connected:
            self.lbl_status_led.setStyleSheet("color: #2e7d32; font-size: 18px;")
            self.lbl_status_text.setText(f"Serial Status: Connected ({port_name})")
            self.lbl_status_text.setStyleSheet("font-weight: bold; color: #2e7d32;")
        else:
            self.lbl_status_led.setStyleSheet("color: #d32f2f; font-size: 18px;")
            self.lbl_status_text.setText("Serial Status: Disconnected")
            self.lbl_status_text.setStyleSheet("font-weight: bold; color: #d32f2f;")
