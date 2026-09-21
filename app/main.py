"""上位机软件入口主程序 (Main Entry).

Servo Motor Control System
双 P100S 伺服驱动器与 STM32H743 上位机联调测试系统.
"""

import json
import os
from pathlib import Path
import sys

# 将当前脚本所在目录添加至 sys.path, 确保各模块正常 import
APP_DIR = Path(__file__).resolve().parent
if str(APP_DIR) not in sys.path:
    sys.path.insert(0, str(APP_DIR))

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QApplication

from communication.serial_manager import SerialManager
from servo.servo_controller import ServoController
from utils.logger import CommLogger
from gui.main_window import MainWindow


def load_config() -> dict:
    """加载 settings.json 配置文件."""
    cfg_file = APP_DIR / "config" / "settings.json"
    if cfg_file.exists():
        try:
            with open(cfg_file, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception as e:
            print(f"读取 settings.json 失败: {e}")
    return {}


def main():
    """程序入口."""
    # 启用高 DPI 支持
    QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough
    )

    app = QApplication(sys.argv)
    app.setApplicationName("ServoControlPC")
    app.setOrganizationName("LightCheck")

    # 1. 载入配置
    config = load_config()
    log_dir = APP_DIR / config.get("log", {}).get("log_dir", "logs")

    # 2. 实例化核心组件
    logger = CommLogger(log_dir=str(log_dir))
    serial_mgr = SerialManager()
    controller = ServoController(serial_mgr=serial_mgr)

    # 3. 创建主窗口
    window = MainWindow(serial_mgr=serial_mgr, controller=controller, logger=logger)
    window.show()

    # 4. 进入事件循环
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
