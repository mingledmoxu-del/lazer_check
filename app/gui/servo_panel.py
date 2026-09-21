"""伺服电机 PA 参数操作与配置面板 (GUI 部件).

涵盖:
1. 单参数读写 (支持 PA21 / 21 自动解析、Permanent/Temporary 写入模式切换、输入合法性校验)
2. 常用伺服参数快速浏览与选择表
3. 参数批量读取 (队列排队读取)
4. 参数批量写入与 JSON 配置保存/加载
"""

import json
from pathlib import Path
from typing import Optional, Dict, List
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QWidget,
    QGroupBox,
    QVBoxLayout,
    QHBoxLayout,
    QFormLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QComboBox,
    QRadioButton,
    QButtonGroup,
    QTableWidget,
    QTableWidgetItem,
    QHeaderView,
    QTabWidget,
    QTextEdit,
    QMessageBox,
    QFileDialog,
)

from communication.protocol import SaveMode, Command, ResponsePacket
from servo.servo_parameters import (
    SERVO_PARAMETERS,
    parse_pa_number,
    format_pa_string,
    get_parameter_desc,
)
from servo.servo_controller import ServoController


class ServoPanel(QWidget):
    """伺服参数配置与调试主面板."""

    def __init__(self, controller: ServoController, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.controller = controller
        self._init_ui()
        self._bind_events()

    def _init_ui(self) -> None:
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(5, 5, 5, 5)
        main_layout.setSpacing(10)

        # 选项卡切换: 单参数操作 / 常用参数 / 批量读写
        self.tab_widget = QTabWidget()

        # 选项卡 1: 单个 PA 参数操作
        tab_single = QWidget()
        self._init_single_pa_ui(tab_single)
        self.tab_widget.addTab(tab_single, "单个 PA 参数读写 (Single PA)")

        # 选项卡 2: 常用参数速查表
        tab_common = QWidget()
        self._init_common_params_ui(tab_common)
        self.tab_widget.addTab(tab_common, "常用参数列表 (Common Params)")

        # 选项卡 3: 批量读写与配置导出导入
        tab_batch = QWidget()
        self._init_batch_ui(tab_batch)
        self.tab_widget.addTab(tab_batch, "批量读写与配置 (Batch & Config)")

        main_layout.addWidget(self.tab_widget)

    def _init_single_pa_ui(self, parent: QWidget) -> None:
        """初始化单个参数读写面板."""
        layout = QVBoxLayout(parent)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(12)

        box = QGroupBox("PA 参数操作 (Servo Parameter)")
        form = QFormLayout(box)
        form.setLabelAlignment(Qt.AlignRight)
        form.setSpacing(10)

        # 目标电机选择
        self.combo_motor = QComboBox()
        self.combo_motor.addItems(["Motor 1 (从站地址 1)", "Motor 2 (从站地址 2)"])
        form.addRow("目标电机 (Motor):", self.combo_motor)

        # 参数编号输入
        self.edit_pa = QLineEdit()
        self.edit_pa.setPlaceholderText("例如: PA21 或 21")
        self.edit_pa.setText("PA21")
        form.addRow("参数编号 (Parameter):", self.edit_pa)

        # 参数值输入
        self.edit_val = QLineEdit()
        self.edit_val.setPlaceholderText("整数数值 (-32768 ~ 65535)")
        self.edit_val.setText("60")
        form.addRow("设定数值 (Value):", self.edit_val)

        # 写入模式单选框
        mode_layout = QHBoxLayout()
        self.radio_temp = QRadioButton("Temporary (临时修改, 掉电丢失)")
        self.radio_perm = QRadioButton("Permanent (永久保存到 EEPROM)")
        self.radio_perm.setChecked(True)
        self.mode_group = QButtonGroup(self)
        self.mode_group.addButton(self.radio_temp, 0)
        self.mode_group.addButton(self.radio_perm, 1)
        mode_layout.addWidget(self.radio_temp)
        mode_layout.addWidget(self.radio_perm)
        mode_layout.addStretch()
        form.addRow("写入模式 (Write Mode):", mode_layout)

        # 读写操作按钮
        btn_layout = QHBoxLayout()
        self.btn_read_pa = QPushButton("读取参数 (Read Parameter)")
        self.btn_read_pa.setStyleSheet("font-weight: bold; padding: 6px 12px;")
        self.btn_write_pa = QPushButton("写入参数 (Write Parameter)")
        self.btn_write_pa.setStyleSheet("font-weight: bold; background-color: #1976d2; color: white; padding: 6px 12px;")

        btn_layout.addWidget(self.btn_read_pa)
        btn_layout.addWidget(self.btn_write_pa)
        btn_layout.addStretch()
        form.addRow("", btn_layout)

        # 操作结果显示区域
        self.lbl_result = QLabel("就绪 (Ready)")
        self.lbl_result.setStyleSheet("padding: 8px; background: #f0f4f8; border-radius: 4px; font-weight: bold; color: #333;")
        form.addRow("执行结果 (Result):", self.lbl_result)

        layout.addWidget(box)
        layout.addStretch()

    def _init_common_params_ui(self, parent: QWidget) -> None:
        """初始化常用伺服参数表格."""
        layout = QVBoxLayout(parent)
        layout.setContentsMargins(10, 10, 10, 10)

        info_lbl = QLabel("点击下方常用参数行可自动填入单参数操作框进行快速读写:")
        info_lbl.setStyleSheet("color: #666; font-size: 12px;")
        layout.addWidget(info_lbl)

        self.table_common = QTableWidget()
        self.table_common.setColumnCount(4)
        self.table_common.setHorizontalHeaderLabels(["参数编号", "参数说明", "推荐/默认值", "快速操作"])
        self.table_common.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self.table_common.setSelectionBehavior(QTableWidget.SelectRows)

        # 填充常用参数
        self.table_common.setRowCount(len(SERVO_PARAMETERS))
        for row, (pa_num, info) in enumerate(sorted(SERVO_PARAMETERS.items())):
            pa_name = format_pa_string(pa_num)
            self.table_common.setItem(row, 0, QTableWidgetItem(pa_name))
            self.table_common.setItem(row, 1, QTableWidgetItem(info.get("description", "")))
            self.table_common.setItem(row, 2, QTableWidgetItem(str(info.get("default", 0))))

            btn_load = QPushButton("载入")
            btn_load.clicked.connect(lambda _, p=pa_name, v=str(info.get("default", 0)): self._load_to_inputs(p, v))
            self.table_common.setCellWidget(row, 3, btn_load)

        self.table_common.itemClicked.connect(self._on_table_row_clicked)
        layout.addWidget(self.table_common)

    def _init_batch_ui(self, parent: QWidget) -> None:
        """初始化批量读写与配置保存加载."""
        layout = QVBoxLayout(parent)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        # 1. 批量读取区
        read_box = QGroupBox("参数批量读取 (Read All Parameters)")
        read_layout = QVBoxLayout(read_box)
        read_layout.addWidget(QLabel("输入要读取的 PA 编号列表 (用逗号或换行分隔):"))
        self.edit_batch_read_list = QLineEdit("PA1, PA21, PA53, PA71, PA72, PA73")
        read_layout.addWidget(self.edit_batch_read_list)

        read_btn_layout = QHBoxLayout()
        self.btn_batch_read = QPushButton("一键读取全部 (Read All)")
        self.btn_batch_read.setStyleSheet("font-weight: bold;")
        read_btn_layout.addWidget(self.btn_batch_read)
        read_btn_layout.addStretch()
        read_layout.addLayout(read_btn_layout)
        layout.addWidget(read_box)

        # 2. 批量配置与写入区
        write_box = QGroupBox("参数配置表与批量写入 (Batch Config)")
        write_layout = QVBoxLayout(write_box)

        btn_row = QHBoxLayout()
        self.btn_save_cfg = QPushButton("保存配置至文件 (Save JSON)")
        self.btn_load_cfg = QPushButton("从文件载入配置 (Load JSON)")
        self.btn_batch_write = QPushButton("一键写入全部 (Write All)")
        self.btn_batch_write.setStyleSheet("font-weight: bold; background-color: #e65100; color: white;")

        btn_row.addWidget(self.btn_save_cfg)
        btn_row.addWidget(self.btn_load_cfg)
        btn_row.addWidget(self.btn_batch_write)
        btn_row.addStretch()
        write_layout.addLayout(btn_row)

        self.table_batch = QTableWidget(6, 3)
        self.table_batch.setHorizontalHeaderLabels(["参数编号 (如 PA21)", "Motor 1 设定值", "Motor 2 设定值"])
        self.table_batch.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

        # 预设常用数据
        default_batch = [
            ("PA1", "60", "60"),
            ("PA21", "60", "60"),
            ("PA53", "1", "1"),
            ("PA71", "1", "2"),
            ("PA72", "96", "96"),
            ("PA73", "0", "0"),
        ]
        for row, (p, v1, v2) in enumerate(default_batch):
            self.table_batch.setItem(row, 0, QTableWidgetItem(p))
            self.table_batch.setItem(row, 1, QTableWidgetItem(v1))
            self.table_batch.setItem(row, 2, QTableWidgetItem(v2))

        write_layout.addWidget(self.table_batch)
        layout.addWidget(write_box)

    def _bind_events(self) -> None:
        """事件绑定与控制器回调."""
        self.btn_read_pa.clicked.connect(self._on_read_pa_clicked)
        self.btn_write_pa.clicked.connect(self._on_write_pa_clicked)
        self.btn_batch_read.clicked.connect(self._on_batch_read_clicked)
        self.btn_batch_write.clicked.connect(self._on_batch_write_clicked)
        self.btn_save_cfg.clicked.connect(self._on_save_cfg_clicked)
        self.btn_load_cfg.clicked.connect(self._on_load_cfg_clicked)

        self.controller.command_completed.connect(self._on_command_completed)
        self.controller.command_failed.connect(self._on_command_failed)

    def get_selected_motor_id(self) -> int:
        """获取当前选择的电机 ID (1 或 2)."""
        return 1 if self.combo_motor.currentIndex() == 0 else 2

    def set_selected_motor_id(self, motor_id: int) -> None:
        """外部切换选中的电机."""
        self.combo_motor.setCurrentIndex(0 if motor_id == 1 else 1)

    def _load_to_inputs(self, pa_name: str, value: str) -> None:
        """将选定参数加载到编辑框中并切换到单参数页."""
        self.edit_pa.setText(pa_name)
        self.edit_val.setText(value)
        self.tab_widget.setCurrentIndex(0)

    def _on_table_row_clicked(self, item: QTableWidgetItem) -> None:
        """点击常用参数表格行."""
        row = item.row()
        pa_item = self.table_common.item(row, 0)
        val_item = self.table_common.item(row, 2)
        if pa_item and val_item:
            self._load_to_inputs(pa_item.text(), val_item.text())

    def _on_read_pa_clicked(self) -> None:
        """执行单参数读取."""
        pa_num = parse_pa_number(self.edit_pa.text())
        if pa_num is None:
            QMessageBox.warning(self, "输入错误", "请输入合法的 PA 参数编号 (如 PA21 或 21)")
            return

        motor_id = self.get_selected_motor_id()
        self.lbl_result.setText(f"正在读取 Motor {motor_id} PA{pa_num}...")
        self.lbl_result.setStyleSheet("padding: 8px; background: #fff8e1; color: #f57f17; font-weight: bold;")
        self.controller.read_pa(motor_id, pa_num)

    def _on_write_pa_clicked(self) -> None:
        """执行单参数写入."""
        pa_num = parse_pa_number(self.edit_pa.text())
        if pa_num is None:
            QMessageBox.warning(self, "输入错误", "请输入合法的 PA 参数编号 (如 PA21 或 21)")
            return

        try:
            val_text = self.edit_val.text().strip()
            val = int(val_text)
            if not (-32768 <= val <= 65535):
                raise ValueError("超出16位整型取值范围")
        except Exception:
            QMessageBox.warning(self, "数值非法", "参数值必须为有效的整数 (-32768 ~ 65535)")
            return

        motor_id = self.get_selected_motor_id()
        save_mode = SaveMode.PERMANENT if self.radio_perm.isChecked() else SaveMode.TEMPORARY
        mode_str = "永久" if save_mode == SaveMode.PERMANENT else "临时"

        self.lbl_result.setText(f"正在向 Motor {motor_id} 写入 PA{pa_num} = {val} ({mode_str})...")
        self.lbl_result.setStyleSheet("padding: 8px; background: #fff8e1; color: #f57f17; font-weight: bold;")
        self.controller.write_pa(motor_id, pa_num, val, save_mode)

    def _on_command_completed(self, pkt: ResponsePacket) -> None:
        """处理控制器回传的成功指令."""
        if pkt.command == Command.READ_PA:
            val = pkt.get_value_int16()
            pa_str = format_pa_string(pkt.parameter)
            msg = f"Motor {pkt.device_id} {pa_str} = {val} | Read Successful"
            self.lbl_result.setText(msg)
            self.lbl_result.setStyleSheet("padding: 8px; background: #e8f5e9; color: #2e7d32; font-weight: bold;")
            if pkt.device_id == self.get_selected_motor_id():
                self.edit_val.setText(str(val))

        elif pkt.command == Command.WRITE_PA:
            pa_str = format_pa_string(pkt.parameter)
            msg = f"Motor {pkt.device_id} {pa_str} 写入成功 | Write Successful"
            self.lbl_result.setText(msg)
            self.lbl_result.setStyleSheet("padding: 8px; background: #e8f5e9; color: #2e7d32; font-weight: bold;")

    def _on_command_failed(self, device_id: int, command: int, reason: str) -> None:
        """处理指令失败或超时."""
        if command in (Command.READ_PA, Command.WRITE_PA):
            cmd_name = "读取" if command == Command.READ_PA else "写入"
            msg = f"Motor {device_id} {cmd_name}失败: {reason}"
            self.lbl_result.setText(msg)
            self.lbl_result.setStyleSheet("padding: 8px; background: #ffebee; color: #c62828; font-weight: bold;")

    def _on_batch_read_clicked(self) -> None:
        """执行批量读取."""
        raw_text = self.edit_batch_read_list.text().replace("\n", ",").replace(";", ",")
        pa_list = []
        for part in raw_text.split(","):
            pa = parse_pa_number(part)
            if pa is not None and pa not in pa_list:
                pa_list.append(pa)

        if not pa_list:
            QMessageBox.warning(self, "提示", "未能识别到有效的 PA 编号列表")
            return

        motor_id = self.get_selected_motor_id()
        self.controller.batch_read_parameters(motor_id, pa_list)

    def _on_batch_write_clicked(self) -> None:
        """执行批量参数写入."""
        motor_id = self.get_selected_motor_id()
        col_idx = 1 if motor_id == 1 else 2
        save_mode = SaveMode.PERMANENT if self.radio_perm.isChecked() else SaveMode.TEMPORARY

        params_to_write: Dict[int, int] = {}
        for r in range(self.table_batch.rowCount()):
            pa_item = self.table_batch.item(r, 0)
            val_item = self.table_batch.item(r, col_idx)
            if pa_item and val_item:
                pa = parse_pa_number(pa_item.text())
                try:
                    val = int(val_item.text().strip())
                    if pa is not None:
                        params_to_write[pa] = val
                except ValueError:
                    continue

        if not params_to_write:
            QMessageBox.warning(self, "提示", "没有可写入的有效参数行")
            return

        reply = QMessageBox.question(
            self,
            "确认批量写入",
            f"确定要向 Motor {motor_id} 批量写入 {len(params_to_write)} 个参数吗?",
            QMessageBox.Yes | QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.controller.batch_write_parameters(motor_id, params_to_write, save_mode)

    def _on_save_cfg_clicked(self) -> None:
        """保存配置至 JSON 文件."""
        cfg_data = {"motor1": {}, "motor2": {}}
        for r in range(self.table_batch.rowCount()):
            pa_item = self.table_batch.item(r, 0)
            m1_item = self.table_batch.item(r, 1)
            m2_item = self.table_batch.item(r, 2)
            if pa_item:
                pa = pa_item.text().strip()
                if m1_item and m1_item.text().strip():
                    try:
                        cfg_data["motor1"][pa] = int(m1_item.text().strip())
                    except ValueError:
                        pass
                if m2_item and m2_item.text().strip():
                    try:
                        cfg_data["motor2"][pa] = int(m2_item.text().strip())
                    except ValueError:
                        pass

        filepath, _ = QFileDialog.getSaveFileName(self, "保存参数配置", "servo_config.json", "JSON Files (*.json)")
        if filepath:
            try:
                with open(filepath, "w", encoding="utf-8") as f:
                    json.dump(cfg_data, f, indent=4, ensure_ascii=False)
                QMessageBox.information(self, "保存成功", f"配置已保存至:\n{filepath}")
            except Exception as e:
                QMessageBox.critical(self, "保存失败", str(e))

    def _on_load_cfg_clicked(self) -> None:
        """从 JSON 文件载入配置."""
        filepath, _ = QFileDialog.getOpenFileName(self, "加载参数配置", "", "JSON Files (*.json)")
        if not filepath:
            return

        try:
            with open(filepath, "r", encoding="utf-8") as f:
                data = json.load(f)

            m1_dict = data.get("motor1", {})
            m2_dict = data.get("motor2", {})

            all_pas = sorted(list(set(list(m1_dict.keys()) + list(m2_dict.keys()))), key=lambda x: parse_pa_number(x) or 0)
            self.table_batch.setRowCount(len(all_pas))

            for r, pa_name in enumerate(all_pas):
                self.table_batch.setItem(r, 0, QTableWidgetItem(pa_name))
                v1 = str(m1_dict.get(pa_name, ""))
                v2 = str(m2_dict.get(pa_name, ""))
                self.table_batch.setItem(r, 1, QTableWidgetItem(v1))
                self.table_batch.setItem(r, 2, QTableWidgetItem(v2))

            QMessageBox.information(self, "加载成功", f"已成功从 {Path(filepath).name} 载入参数配置")
        except Exception as e:
            QMessageBox.critical(self, "加载失败", str(e))
