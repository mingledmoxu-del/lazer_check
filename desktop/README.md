# P100S Windows 可视化调试工具

安装：`py -m pip install -r requirements.txt`；启动：`py main.py`。选择 USB 串口后连接；也可点击“模拟设备”离线验证界面、粘包解析和状态过期。

Windows 用户可双击 `start_p100s.bat` 启动。脚本会优先使用项目配套 Python，检测并安装缺失的 PySide6/pyserial，然后启动上位机。

STM32 USART1 协议使用 ASCII 行（115200/8N1）：

`@C <请求编号> <命令> [key=value...]\n`

命令为 `STATUS CHECK ENABLE RUN rpm=-100 STOP DISABLE`。应答为 `@R <编号> <OK|ACCEPT|REJECT|FAIL> ...`，周期状态为 `@S 0 OK ...`。`age_ms` 超过 1500 时上位机显示“状态未知”。STM32 必须负责 2 秒试转超时，PC 计时只用于显示。

首次试转：参数检查通过 → 零速使能 → 输入 1～100 rpm → 正/反转试转 → 始终用“停止”收尾。解除使能可能自由滑行，不是硬件急停。界面关闭会尝试 STOP，但断线时不会谎称已停止。

本工具不自动下载固件、不自动使能、不自动运动。带电测试前必须先排查现有使能时尖锐噪声和 HDMI 闪烁问题。
