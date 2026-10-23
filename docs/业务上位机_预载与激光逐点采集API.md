# 业务上位机接入：六套预载、激光采集、显式下一点（v3.9.4）

本文面向接入同事及AI。固件使用 **H743 v3.9.4**，SDK使用本版源码；库5、6在旧固件不可用，不自动降级。F407备用工程未同步本版。

## 职责与点位格式

业务上位机负责让操作者选好需要的点位、组合路线、保存/命名预设，并进行激光采集及判定。我们的Python SDK接收**选定后的两轴绝对角度列表**，上传到下位机，并按业务事件逐点运行；不负责推算激光坐标对应的机械角度。

```python
selected_points = [(0, 0), (6.9, 0), (16.1, -9.7), (24.8, 0), (16.1, 9.7), (0, 0)]
# 每项：(电机1俯仰角度°, 电机2水平角度°)，顺序就是业务访问顺序。
```

路线没有“四点”限制。**板端同时保存6套RAM库，编号1～6，每套1～16个双轴点位，编号从0开始。** 六个、八个、十个测量点都适用；如果另加起点和回零点，它们也计入16点。例如10个测量点加起终原点共12点。不同库可以有不同点数、速度和终点容差。

库数和点数是板端容量；业务软件本地可维护更多命名路线，选中需要使用的最多6套后再上传。超过16点明确拒绝，不静默截断或拆批。上传不会运动，不会自动建立/清除参考；封存前需要两轴已经建立参考并停稳。覆盖同编号库必须使用新token，旧会话失效。token默认由SDK生成。

速度单位是真实输出轴rpm：`speed_rpm=3`表示实际3rpm，不除以6。范围0.01～100，默认0.1；这只是协议允许范围，沿用现场已验证的工作速度。默认终点容差±0.2°，允许0.01～0.5°。电机1固定实测范围−29°～+35°；最终以板端软限位、硬件输入和驱动保护为准。

## 推荐流程API

```python
from app import MotorController, PointScan, MotorError

controller = MotorController("tcp://192.168.10.50:5000").connect()
# IP/端口由他们的界面填写，例如：
# controller = MotorController(f"tcp://{selected_ip}:{selected_port}").connect()
# 先按现场流程建立两轴参考；连接本身不会运动。

scan = controller.prepare_point_scan(
    selected_points, name="用户选中的路线", route_id=1,
    speed_rpm=3.0, tolerance_deg=0.2,
)
# 到这里仅上传，scan.status()['phase'] == 'READY'，没有发RUN。

arrived = scan.move_next_and_wait(timeout_s=30)  # 仅运行点0，阻塞至真实到位停稳。
# arrived['positions']：两轴最终完成快照，直接复用，不要再停止/全量核验一遍。
index = arrived['point_index']
target = arrived['targets_deg']

# 以下由业务系统执行，不是本SDK提供的激光接口：
# acquisition = your_laser_acquire(index, target, arrived['positions'])
# ok = your_decision(acquisition)
# scan.confirm_acquisition(index, acquisition.id, passed=ok)
```

确认 `passed=True` 只开放下一点，**不发送运动**。必须再调用一次 `scan.move_next_and_wait()` 才会去下一点。`passed=False` 保持当前点、等待业务层重新采集或停止，不能调用下一点绕过。最后一点到位后仍需确认；最后一次确认通过后状态为 `COMPLETED`，再调用下一点会被拒绝。

起点/回零点如果不需要采集激光，也要由业务层显式完成该点检查，再以相应业务记录ID确认，例如 `origin-checked-001`；SDK不会隐式跳过这些点，也不会自动追加回零运动。业务软件自行决定哪些点是测量点。

| API | 作用与返回 | 是否通信/运动 |
| --- | --- | --- |
| `controller.prepare_point_scan(points, name='', route_id=1, speed_rpm=.1, tolerance_deg=.2, token=None)` | 上传并封存，返回`PointScan` | 上传；不运动 |
| `scan.status()` | 本地流程状态字典；不是实时位置查询 | 无通信 |
| `scan.move_next_and_wait(timeout_s=120)` | 推荐组合调用；运行一个点，等待完成，返回完成快照与耗时 | 一个RUN加必要反馈查询 |
| `scan.start_next_point()` | 只发送一个点，返回接受应答；`completed=False` | 一个RUN；应答不代表到位 |
| `scan.wait_current_point(timeout_s=120)` | 等当前点实际完成，进入待采集 | 只查询；不会重发RUN |
| `scan.confirm_acquisition(point_index, acquisition_id, passed=...)` | 确认必须属于当前已到位点；返回流程状态 | 无通信、不运动 |
| `scan.stop(wait=True, timeout_s=10)` | 取消流程并尝试停止两轴；默认等待停止确认 | STOP及必要查询 |

推荐在他们的工作线程/任务执行器调用阻塞方法，GUI主线程继续响应界面。SDK不创建激光采集线程，也不调他们的外部服务。

若他们需要异步UI事件分派，可以拆成：

```python
accepted = scan.start_next_point()  # 相当于“发运动”按钮；仅接受应答。
# 在工作线程执行这一句：
arrived = scan.wait_current_point(timeout_s=30)
# 只有这一句成功返回后，才允许开启该点采集。
# 采集回调保存 scan对象、scan.token、point_index，避免用全局“当前点”接收迟到回调。
scan.confirm_acquisition(arrived['point_index'], "本次唯一采集ID", passed=True)
# 业务事件稍后再调用 start_next_point()，不会因确认自动跳点。
```

不建议反复调用 `start_next_point()` 查询状态：它是运动指令。等待期间重复启动会被拒绝；同时只能有一个 `wait_current_point()`。已到位后再次等待只返回已有完成结果，不额外查询。

## 六套路线预载与选择

```python
# selected_routes由他们的界面给出，1～6套；每项的points长度可以不同。
selected_routes = [
    {"name": "路线A", "points": points_a, "speed_rpm": 3.0},
    {"name": "路线B", "points": points_b, "speed_rpm": 3.0},
    {"name": "路线C", "points": points_c, "speed_rpm": 3.0},
    {"name": "路线D", "points": points_d, "speed_rpm": 3.0},
    {"name": "路线E", "points": points_e, "speed_rpm": 3.0},
    {"name": "路线F", "points": points_f, "speed_rpm": 3.0},
]
if not 1 <= len(selected_routes) <= 6:
    raise ValueError("同时预载1～6套")

scans = {}
for slot, item in enumerate(selected_routes, 1):
    scans[slot] = controller.prepare_point_scan(
        item["points"], name=item["name"], route_id=slot,
        speed_rpm=item["speed_rpm"], tolerance_deg=0.2,
    )
# 六套到此全部只是预载。界面选中哪一套，就用对应scans[slot]。
scan = scans[chosen_slot]  # chosen_slot来自他们的界面，1～6。
arrived = scan.move_next_and_wait(timeout_s=30)
```

按一条物理设备、一个`MotorController`连接管理。选中的流程运动或等待采集/下一点期间，本SDK会拒绝另一条`PointScan`流程启动，防止在当前点采集时被另一套路线移动。当前流程完成后可选其他已预载流程。不要并行混用GUI、原始运动API、另一进程或同一设备的其他连接；SDK本地状态无法管理绕过本流程的外部命令。

## 状态与关键返回值

```text
prepare → READY
start_next_point → MOVING
wait_current_point成功 → AWAITING_ACQUISITION
confirm_acquisition(passed=False) → AWAITING_ACQUISITION（保持）
confirm_acquisition(passed=True) → READY_NEXT（尚未运动）
再次显式start_next_point → MOVING
最后一点确认通过 → COMPLETED
stop → STOPPED
通信、参考、预载身份或反馈异常 → FAULT
```

`status()`字段：`name`、`route_id`、`token`、`phase`、`point_count`、`current_index`、`current_target`、`next_index`、`can_start_next`、`acquisition_required`、`fault`、`speed_rpm`、`tolerance_deg`、`upload_elapsed_ms`。`next_index`只是候选序号，**必须看`can_start_next`**，待采集时即使有候选也不能运行。

`move_next_and_wait()`/`wait_current_point()`完成结果包含：

| 字段 | 含义 |
| --- | --- |
| `route_id / point_index / targets_deg` | 当前运行库、0起始序号、两轴绝对目标 |
| `completed=True / phase='AWAITING_ACQUISITION'` | 电机真实到位并停稳，尚未由业务采集确认 |
| `positions` | 两个`AxisPosition`完成快照；包括角度、原始计数、速度、年龄、来源、报警和保护状态 |
| `tolerance_deg` | 本库完成容差 |
| `board_prepare_ms` | 板端两轴准备计时，单位ms |
| `accepted_ms / wait_ms / elapsed_ms` | 接受应答、应答后等待、该点从运行调用至主机确认完成；不包含预载和业务采集时间 |

这些反馈证明编码器/电机满足完成条件，不代表激光数据已采集，或光斑振动已经过光学测量。是否开始/结束采集、是否通过，由业务层决定。

`acquisition_id`由业务系统提供非空字符串。当前点确认通过后的完全相同重复确认是幂等的，不跳过下一点；迟到确认如果不属于当前已到位点会被拒绝。不要把所有回调都发给一个可变的全局scan变量。

## 错误、停止与重连

业务层应捕获 `MotorError`（包括通信、板端拒绝等子类），暂停采集/推进并展示原始错误；不要自动重发RUN或自动清除参考。超时、参考失效或反馈异常不会被当作到位。

`scan.stop()`取消推进，两个轴都会尝试停止。`wait=True`成功返回才有`stop_confirmed=True`；失败会抛异常，不能把“已取消业务流程”当作“硬件已停稳”。`wait=False`仅请求停止，不宣称停稳。普通停止不主动清除板端已建立的参考，但旧`PointScan`流程不能自动续跑。

断开/重连、外部停止、同编号库被新token覆盖、通信不确定时，旧会话禁止继续。重新检查实际位置与参考，必要时重新建立参考，再重新预载创建会话。板子断电/复位使RAM预载失效，必须重新上传；不能仅凭SDK本地`status()`判断RAM仍有效。每次RUN板端仍校验库token、就绪和参考。

## 原始单点接口仍保留

若业务层已有完整状态机，可直接使用：

```python
record = controller.preload_route(selected_points, route_id=6, speed_rpm=3, tolerance_deg=.2)
result = controller.move_preloaded_point_and_wait(6, chosen_index, timeout_s=30)
# 业务采集与判断……
# 然后业务层显式选择另一个chosen_index并调用上式。
```

还提供 `get_preloaded_route(route_id)`（查板端就绪/token等）、`run_preloaded_point(route_id, point_index)`（只接受应答）和 `wait_for_axes(targets, tolerance_deg=.2)`。这些底层接口没有采集确认门禁，调用方自己管理，不要与正在运行的`PointScan`混用。上述为Python SDK方法，未额外启动HTTP服务器。

非Python软件可按既有TCP二进制协议接入：设备号1，Parameter为库1～6；命令0x40开始、0x41逐点上传、0x42封存、0x43单点运行、0x44查询，数据布局与v3.9.1一致。采集确认是主机业务状态，不额外给板子发送“假运动”指令，门禁需在其业务软件实现。详细包格式见 `v3.9.1_预载点位与0.2度完成容差.md`，其“四套”容量在本版升级为六套。

## 发布和验证

配套固件与源码包：`output/firmware/v3.9.4-six-bank-scan-h7/`。本次未修改定位控制或停稳判定参数，没有新增每点启动前的独立板端状态查询；性能优化沿用v3.9.3。230项Python、15组C、H743构建及Tk六库切换检查通过，覆盖六套独立库、6/8/10/12/14/16点、采集门禁、重复/迟到确认、异步等待取消、重连和覆盖失效。未执行实机运动或激光采集。
