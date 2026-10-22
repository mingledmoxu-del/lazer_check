# F407 备用下位机

由当前 `LightCheck` 的业务代码独立移植，原 H743 工程保持不变。上位机继续使用仓库根目录的 `app`，协议和 SDK 调用不变。

默认目标为 STM32F407VE：512 KB Flash、128 KB 普通 SRAM，启动地址 0x08000000。具体板型尚待确认；引脚是否可用、是否连接板载外设须按实际板子核对。使用内部 HSI 经 PLL 产生 168 MHz，不依赖板载晶振频率；115200 波特率的实物可靠性仍需测试。

## 默认接线

| 功能 | F407 引脚 | 外部连接 |
| --- | --- | --- |
| 上位机 USART1 TX | PA9 | USB-TTL 的 RXD |
| 上位机 USART1 RX | PA10 | USB-TTL 的 TXD |
| 电机 USART2 TX | PA2 | 485 收发器 DI |
| 电机 USART2 RX | PA3 | 485 收发器 RO |
| 485 方向控制 | PB1 | DE 与 /RE 并接，高发送、低接收 |
| 六路输入 DI0～DI5 | PC0～PC5 | 对应输入信号 |
| 地 | GND | USB-TTL、收发器的信号地 |

USB-TTL 使用 **3.3V 逻辑电平**，板子单独供电时不接 USB-TTL 的 VCC。485 模块按额定电压供电，MCU 侧应兼容 3.3V 电平；不能把具有 5V RO 输出的模块直接当作 3.3V 模块使用。A/B 接原电机总线，驱动器地址仍为 1、2，端接按总线两端配置。

默认适配带 DE、/RE 的半双工模块；自动方向模块可不接 PB1，但需要验证模块的方向切换时序。不能把 A/B 接到 MCU 的 TX/RX 上。

**F407 裸 GPIO 不具备原板的 24V 光耦输入电路。PC0～PC5 只允许接适配后的 3.3V 信号，24V 传感器需要外置光耦/电平转换。** 默认下拉，输入高电平对应 raw 位 1；保护极性和策略由上位机显式配置、启用。

## 功能对应

保留双 P100S 电机 Modbus RTU 控制、位置/速度/报警读取、输出轴角度与参考捕获、限位采样、可配置保护与回零、软范围、轨迹缓存与双轴执行、停止与故障恢复、连续旋转通信看门狗。原有功能范围和精度限制仍适用，相关 API 见根目录 `docs`。

| H743 实现 | F407 实现 |
| --- | --- |
| USART1 PA9/PA10，115200/8N1 | USART1 PA9/PA10，115200/8N1 |
| USART3 PC10/PC11，115200/8N2 | USART2 PA2/PA3，115200/8N2 |
| PC12 控制板载 485 | PB1 控制外置 485 |
| PF5～PF0 的六路隔离输入 | PC0～PC5 的六路 3.3V 输入 |
| DMA1/DMAMUX 与 D2 SRAM | DMA2 Stream2 Channel4，普通 SRAM |
| H7 错误标志清除、MPU、时钟树 | F4 SR/DR 清错、无 MPU 配置、F4 时钟树 |

保留原来的 `MOTOR_LIMITS_ENABLED=0`：旧版固定限位策略默认关闭，新的运行时保护策略仍需显式启用。急停和驱动器 SON 使用外部硬件。

## 编译与验证

本目录执行 `powershell -ExecutionPolicy Bypass -File build.ps1`。优先使用 PATH 工具，缺失时查找 STM32Cube 已安装的 GCC、Ninja、CMake。也可配置好工具路径后执行 `cmake --preset Debug` 和 `cmake --build --preset Debug`。

已包含来自本机 STM32CubeF4 V1.28.3 的 HAL、CMSIS 和启动文件，不依赖 H743 工程或本机 CubeF4 目录。使用 CMake 工程维护，没有提供未经验证的 CubeMX `.ioc`。

输出在 `build/Debug/LightCheck_F407.elf`、`.hex`、`.bin`。BIN 烧录地址 **0x08000000**。链接脚本为 `STM32F407VE_FLASH.ld`；修改引脚时同步修改 `Core/Inc/main.h`、`Core/Src/usart.c`、`Core/Src/gpio.c`。

本次 Release 固件另存于 `firmware/`，同时提供 ELF、HEX、BIN 和 SHA256 清单 `manifest.json`。Debug、Release 编译及 8 组主机测试通过，29 个业务源码文件与 H743 版本一致（仅替换 HAL 头文件），HEX/BIN 内容与启动向量检查通过。Release Flash 占用约 24 KB，包含预留堆栈的 RAM 占用约 12.4 KB。

主机回归测试：`powershell -ExecutionPolicy Bypass -File tests/run_host_tests.ps1`，需要主机版 GCC。测试覆盖业务逻辑，不覆盖实际 DMA、中断、波特率、电气接线或电机动作。

目前为已编译、通过业务回归测试的备用移植，**尚未实机验证**。首次上电先关闭驱动器 SON，验证上位机通信、驱动器只读状态和六路输入，再按原调试文档设置参考、测试停止和保护，最后进行小幅运动。

硬件参考：[ST STM32F407 数据手册](https://www.st.com/resource/en/datasheet/stm32f407ve.pdf)。
