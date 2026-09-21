# P100S Windows 工具与 STM32 协议

工具位于 `desktop`，依赖 Python 3.10+、PySide6、pyserial。运行 `py -m pip install -r requirements.txt` 后执行 `py main.py`。工具默认不连接、不使能、不恢复运动；“模拟设备”用于无硬件验证。

## USART1 协议

115200/8N1，命令和应答按 `\n` 分帧。请求：`@C 请求编号 命令 key=value...`。应答：`@R 请求编号 OK|ACCEPT|REJECT|FAIL 字段...`；周期状态：`@S 0 OK|FAIL 字段...`。请求编号只用于关联，STM32 不自动重发运动或使能请求；停止/解除使能优先于普通命令。

命令：`STATUS`、`CHECK`、`ENABLE`、`RUN rpm=-100..100`、`STOP`、`DISABLE`。RUN 仍由固件约 2 秒后自主写零速，等待实际速度接近零再解除使能；通信异常或报警进入故障。`DISABLE` 可能自由滑行，不是硬件急停。

异常码示例：`LINE_TOO_LONG`、`BAD_FRAME`、`BAD_ID`、`BAD_COMMAND`、`RPM_RANGE`、`BUSY`。只有收到 `@R ... OK` 且周期 `@S` 未过期时界面才显示在线/已停止；断线、状态过期、回包丢失均显示“状态未知”。

STM32 仍复用 `p100s_bench.c` 的原厂寄存器检查和保护逻辑：PA-53 暂存 `0x00B5`，PA-24 暂存 `0x0098`，不写 EEPROM。该协议层不在中断里执行 Modbus。
