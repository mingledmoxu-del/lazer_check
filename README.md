# 双电机控制项目

## 当前版本：上位机/SDK v3.8.8，固件v3.8.7

v3.8.8增加指令阶段耗时日志，不需重烧已有v3.8.7固件，详见[耗时日志说明](docs/v3.8.8_指令阶段耗时日志.md)。GUI自动保存日志，SDK支持`timing_log`/`timing_callback`。

v3.8.7新增起步分段限速、板端停稳完成证据和两轴直接快照/组合等待API，保留精度及保护；详见[v3.8.7说明](docs/v3.8.7_起步平滑与停稳完成反馈.md)。需同步更新固件和SDK，实机抖动及耗时仍待复测。

STM32H743 主网口 TCP 双 P100S 控制工程，配套 Python 上位机与 SDK。新同事或 AI 接入请先阅读[统一 API 指南](docs/控制API_同事与AI接入指南.md)，再阅读[v3.8.6 更新说明](docs/v3.8.6_到位速度与停稳延迟优化.md)。下文旧版本说明属于开发历史，冲突时以当前 API 指南为准。

- 主工程：`LightCheck/`；备用 F407 工程：`LightCheck_F407/`，未同步当前 H7 全部功能。
- 上位机启动：Windows 下运行 `app/start_simple.bat`。
- 固件及配套 SDK：[v3.8.7 发布目录](output/firmware/v3.8.7-smooth-completion-h7/README.md)，需同时更新固件和上位机。
- 耗时日志版上位机/SDK：[v3.8.8 发布目录](output/upper-computer/v3.8.8-motion-timing/README.md)，继续配套v3.8.7固件。
- TCP 默认地址 `192.168.10.50:5000`。SDK 示例：`MotorController("tcp://192.168.10.50:5000").connect()`，地址及端口可由调用方配置。
- 电机1：DI0 对应 +35°，DI2 对应 −29°，MCU 输入低电平触发；普通停止保留已建立参考。
- v3.8.6 优化点位减速、反馈查询及重复停稳等待；仍保留连续两次零速确认和限位保护。
- 验证：固件v3.8.7沿用12组C测试及H743构建；主机v3.8.8通过191项Python测试及GUI构造检查；实机动态性能仍待验证。

构建 H7 固件（需 ARM GNU 工具链、CMake、Ninja）：

```powershell
cd LightCheck
cmake --preset Debug
cmake --build --preset Debug
```

执行回归测试（根目录运行，C 测试需本机 GCC）：

```powershell
python -m unittest discover -s app -p "test_*.py"
./LightCheck/tests/run_host_tests.ps1
```

## 历史开发记录

当前只维护一套上位机：`app`。

**最新v3.8.1**修复首次未建参考被误报为反馈异常并反复停机的问题，新增显式“清除电机1参考”按钮和`clear_axis1_reference()` API。需同步更新固件与上位机；同事接入仍以[统一API指南](docs/控制API_同事与AI接入指南.md)为准。

**当前统一接入版本v3.8.0：先读 [控制API：同事与AI接入指南](docs/控制API_同事与AI接入指南.md)。** 电机1复位保留已有参考；电机2提供复位与确认当前0°两种API；复位/回0默认0.6输出轴rpm、可指定0.2～1.5。本版需一起更新固件与SDK/GUI，旧版本速度单位说明以本指南为准。

上位机最新v3.7.9修复重复“回到+35°”清除参考的问题：有效参考用于直接坐标定位，无参考才寻找DI0。普通停止保留已有参考，继续使用v3.7.7固件。见 [重复定位保留参考](docs/v3.7.9_重复定位保留参考.md)。

**最新 v3.7.7** 修复已标定+35°后数个编码器count波动引发的反复停机。双限位反馈检查增加0.05°容差，目标仍严格限于−29°～+35°，见 [边界反馈修复](docs/v3.7.7_限位边界反馈容差.md)。需更新固件。

**最新 v3.7.6：DI0参考改为+35°，DI2保持−29°，低电平触发。** 同步修复停稳后的界面占用清理和只读查询拦截操作问题。本次需要一起更新固件与上位机，见 [35°参考与停稳解锁](docs/v3.7.6_35度参考与停稳解锁.md)。下面历史版本说明以本版为准。

**当前 v3.7.5 已按两只限位的现场触发读数改为低电平触发（未触发1，触发0）**。重启上位机、头部摆回接近水平后，点击“复位到+37°”会下发新配置；无需重烧 v3.7.2 固件。见 [低电平复位操作](docs/v3.7.5_低电平限位复位.md)。下面旧版高电平说明已被本版替代。

上位机已精简为 v3.7.3：高电平触发固定，主窗口“① 复位到 +37°”自动下发配置并抬头复位；旧 +25°与通用限位配置入口已删除。操作见 [精简复位流程](docs/v3.7.3_上位机精简复位.md)，继续配套 v3.7.2 固件。

最新实测配置：**DI0=+37°，DI2=−29°，电机原始反向抬头**。固件行为见 [v3.7.2 说明](docs/v3.7.2_DI0抬头回零.md)，当前界面操作按上面的 v3.7.3 说明。

当前硬件继续使用原 H743 板，默认通过**主网口 CN2 TCP**连接，见 [v3.7 主网口TCP通信](docs/v3.7_主网口TCP通信.md)。电机1双限位为水平侧 DI2、抬头侧 DI0，设置与测试见 [v3.6 双限位标定与回零](docs/v3.6_双限位标定与回零.md)。`LightCheck_F407` 保留为备用工程，当前功能在 `LightCheck` 中开发。

- 启动上位机：[app/start_simple.bat](app/start_simple.bat)
- 上位机使用说明：[app/README.md](app/README.md)
- 正式软件集成接口：[app/motor_controller.py](app/motor_controller.py)，通过 `from app import MotorController` 调用，不需要启动界面。
- STM32 固件工程：`LightCheck`
- 接线、协议和驱动器资料：`docs`
- v2.0综合调试流程与验收：[docs/综合调试与验收.md](docs/综合调试与验收.md)
- v3.0两轴轨迹接口、CSV格式和验收：[docs/两轴轨迹接口与验收.md](docs/两轴轨迹接口与验收.md)；这是现有RS485接线下的实验跟随模式，不保证5秒高精度空间圆。

## 旧版上位机

旧的 `app`（PySide6 完整版）和 `desktop`（ASCII 协议调试版）已从项目运行目录移除。
当前 `app` 是后续精简Tkinter版本，复用了目录名称，不是上面的旧PySide6程序。
源码、配置、日志和缓存保存在 [_archive/legacy-upper-computers-20261013.zip](_archive/legacy-upper-computers-20261013.zip)，需要时可解压恢复。
日常启动只使用 `app/start_simple.bat`，不要使用备份中的旧入口。
