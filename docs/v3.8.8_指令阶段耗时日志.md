# v3.8.8：指令阶段耗时日志（主机更新）

配套固件仍为v3.8.7，不改运动控制、不新增通信查询。GUI自动写入`app/logs/motion_timing.jsonl`并显示TIMING记录，SDK需配置日志文件、回调或Python标准日志INFO处理器。文件追加写入；对比实验前可把旧日志另存。

```python
from app import MotorController

with MotorController(
    "tcp://192.168.10.50:5000",
    timing_log="logs/motion_timing.jsonl",
    timing_callback=lambda row: print(row),
) as controller:
    # 已建立参考；发送后需读取反馈/等待，才能记录运动及完成观测。
    controller.move_to_angle(1, 10, speed_rpm=0.2)
    controller.wait_for_angle(1, 10)
```

回调仅用于快速收集日志，勿在回调内发送控制命令或做长时间阻塞操作。文件/回调写入失败只警告，不改变运动行为。

## 事件与耗时

同一trace_id对应一轮运动，axis区分两轴。SDK相对/绝对定位及电机1复位从调用准备阶段开始；GUI主定角和调试坐标按钮同样计入停止准备。其他仅由通信帧识别的运动，call_start从发送附近开始，不虚构按钮等待时间。

| 事件 | 含义 |
|---|---|
| call_start | 主机开始处理该运动请求 |
| stop_sent / stop_ack / stop_confirmed | 准备停止、应答、主机两次零速确认 |
| motion_sent | 开始调用传输写入，包含网络发送开销，不等于驱动器开始运动 |
| motion_ack | 收到匹配的板端成功应答，仅表示接受 |
| motion_observed | 首次从现有反馈看到非零转速或可判断的编码器变化 |
| angle_in_window | 首次观察到进入目标±0.5°，目标未知时不会伪造该事件 |
| completed | 观察到板端完成/回零完成，或SDK双样本到位确认；basis说明依据 |
| cancelled / replaced / rejected / fault / communication_error / connection_closed | 停止取消、替换、拒绝、故障或未知结束；不记成功完成 |

每条记录的`elapsed_ms`从call_start算起，用主机单调时钟计算，避免系统校时改变耗时。记录中的timestamp只作带时区的人工对照。关键字段：

- `call_to_send_ms`：调用准备到运动发送，含停止等待、报警/坐标读取。
- `tx_ack_ms`：运动发送到板端应答，含网络、固件处理、RS485配置等。
- `from_sent_ms`：当前阶段距发送的时间；在completed记录中即发送到主机观察到完成的总耗时。
- `from_ack_ms`：收到接受应答后的耗时。
- `from_motion_observed_ms`：首次观测运动之后的耗时，不能当作真实硬件运行时长。
- `from_angle_window_ms`：进入±0.5°后到当前阶段的耗时，用来分析末段收敛/确认。
- `sample_age_ms`、`source`：两轴快照提供时原样保留，用于判断缓存年龄。

没有首次运动观测就不会生成相应耗时字段。相对运动若无已知起点角度，不额外读取起点，只保留relative_deg，窗口阶段可缺失。一个命令即使收到成功应答，也必须有后续有效反馈才会记completed；停止中、故障、过期快照不作为完成证明。同一运动完成只记录一次。

## 采样限制与排查方式

日志记录的是主机得知事件的时刻，无法直接测量驱动器内部启动时刻、板端停止时刻或网线单程延迟。需要精确板端阶段时间时仍需固件计时/外部测量，本版不推断这些未知量。日志只对已有反馈做标注，不增加查询；写文件及用户回调本身仍有少量开销。

复现一次换点后，按trace_id对齐两轴，比较call_to_send_ms、tx_ack_ms、motion_observed、angle_in_window及completed。若调用准备长，查停止/前置查询；若窗口后长，结合实际rpm和采样年龄区分收敛与反馈延迟。不要只用motion_ack推断运动已经开始。

验证：191项Python测试、GUI构造检查；新增阶段时间计算、失败/取消不误记成功、重复完成去重、日志回调失败隔离及无额外通信查询测试。固件沿用v3.8.7已构建版本，本次未连接真实板子。
