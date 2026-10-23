"""面向 DB44 外部使能场景的 P100S 精简上位机。"""

from __future__ import annotations

import datetime as dt
import json
from pathlib import Path
from timing import MotionTiming
import math
import struct
import time
import tkinter as tk
from tkinter import messagebox, ttk

from motion import MotionConversion, convert_output_motion

from protocol import (
    CMD_MOVE_RELATIVE,
    CMD_MOVE_OUTPUT_RELATIVE,
    CMD_READ_PA,
    CMD_READ_SPEED,
    CMD_READ_ALARM,
    CMD_RESET_ALARM,
    CMD_STOP,
    CMD_SET_MOTION_PROFILE,
    CMD_LIMIT_DEBUG,
    CMD_ROTATE_CONTINUOUS,
    CMD_ROTATE_KEEPALIVE,
    Response,
    ResponseParser,
    build_motion_profile,
    build_limit_debug,
    build_continuous_rotation,
    build_rotation_keepalive,
    build_move_output_relative,
    build_request,
)

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # 启动脚本会安装；直接运行时给出友好提示。
    serial = None
    list_ports = None


class SimpleServoApp:
    RESPONSE_TIMEOUT_S = 4.0
    GEAR_RATIO = 50.0
    STARTUP_PA_PARAMETERS = (4, 14, 22, 24, 25, 26, 27, 53, 72, 73)
    # 限位功能暂时停用：保留实现供后续恢复，不创建界面、不发旁路/续期命令。
    # 此开关只影响 PC，不能关闭 STM32 的保护。
    LIMIT_FEATURES_ENABLED = False

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self._log_queue=[]
        self._log_after=None
        self._timing=MotionTiming(Path(__file__).parent/'logs'/'motion_timing.jsonl',
            lambda row:self._log('TIMING '+json.dumps(row,ensure_ascii=False)))
        self.root.title("P100S 电机控制 v3.9.3 · 主网口TCP / 参数块读取与联动计时")
        self.root.geometry("1000x900")
        self.root.minsize(960, 740)

        self.port = None
        self.parser = ResponseParser()
        self.pending: tuple[int, int, float] | None = None
        self.pending_background = False
        self.pending_parameter = None
        self.deferred_send = None
        self.diagnostic_active = False
        self.last_drive_status = ""
        self.queued_motion: tuple[int, int, MotionConversion] | None = None
        self.parameter_check_queue: list[int] | None = None
        self.parameter_values: dict[int, int] = {}
        self.profile_read_queue: list[int] | None = None
        self.profile_values: dict[int, int] = {}
        self.monitor_until = 0.0
        self.next_monitor_seconds = 5.0
        self.motion_seen_running = False
        self.stop_waiting = False
        self.limit_modes: dict[int, bool | None] = {1: None, 2: None}
        self.limit_renew = {1: False, 2: False}
        self.limit_renew_at = {1: 0.0, 2: 0.0}
        self.limit_action: tuple[int, int] | None = None
        self.auto_motor: int | None = None
        self.auto_renew = False
        self.auto_renew_at = 0.0
        self.queued_auto: tuple[int, float] | None = None
        self.home_running = False
        self.debug_panel = None
        self.active_motor = None
        self.trajectory_running = False
        self.trajectory_panel = None
        self.business_panel = None
        self.joint_timing_panel = None

        self.port_name = tk.StringVar()
        self.connection_mode=tk.StringVar(value="TCP网口")
        self.tcp_host=tk.StringVar(value="192.168.10.50")
        self.tcp_port=tk.StringVar(value="5000")
        self.motor_id = tk.IntVar(value=1)
        self.speed_rpm = tk.DoubleVar(value=0.2)
        self.reset_speed_rpm = tk.StringVar(value="0.6")
        self.angle_deg = tk.DoubleVar(value=5.0)
        self.pulses_per_rev = tk.IntVar(value=10000)
        self.accel_ms = tk.IntVar(value=1500)
        self.decel_ms = tk.IntVar(value=1500)
        self.s_curve_ms = tk.IntVar(value=100)
        self.calculated = tk.StringVar(value="电机 10 rpm / 250° / 6,944 pulse")
        self.connection_text = tk.StringVar(value="未连接")
        self.result_text = tk.StringVar(value="请先低速、空载试转")
        self.limit_debug = tk.BooleanVar(value=False)
        self.limit_mode_text = tk.StringVar(value="限位模式：待下位机确认")
        self.auto_speed = tk.DoubleVar(value=5.0)
        self.auto_direction = tk.IntVar(value=1)

        self._build_ui()
        if self.LIMIT_FEATURES_ENABLED:
            self.motor_id.trace_add("write", lambda *_args: self._show_limit_mode())
        for variable in (self.speed_rpm, self.angle_deg, self.pulses_per_rev):
            variable.trace_add("write", lambda *_args: self._update_calculation())
        self._refresh_ports()
        self._update_calculation()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(30, self._poll_serial)

    def _build_ui(self) -> None:
        outer = ttk.Frame(self.root, padding=16)
        outer.pack(fill="both", expand=True)

        connection = ttk.LabelFrame(outer, text="下位机连接", padding=12)
        connection.pack(fill="x")
        ttk.Combobox(connection,textvariable=self.connection_mode,values=("TCP网口","串口"),state="readonly",width=10).grid(row=1,column=0,pady=6)
        ttk.Entry(connection,textvariable=self.tcp_host,width=20).grid(row=1,column=1,sticky="ew")
        ttk.Label(connection,text="端口").grid(row=1,column=2)
        ttk.Entry(connection,textvariable=self.tcp_port,width=7).grid(row=1,column=3)
        ttk.Label(connection,text="主网口 CN2；电脑需设为同一网段").grid(row=1,column=4,padx=8)
        ttk.Label(connection, text="串口").grid(row=0, column=0, padx=(0, 8))
        self.port_combo = ttk.Combobox(connection, textvariable=self.port_name, width=20, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky="ew")
        ttk.Button(connection, text="刷新", command=self._refresh_ports).grid(row=0, column=2, padx=8)
        self.connect_button = ttk.Button(connection, text="连接", command=self._toggle_connection)
        self.connect_button.grid(row=0, column=3)
        ttk.Label(connection, textvariable=self.connection_text).grid(row=0, column=4, padx=(14, 0))
        connection.columnconfigure(1, weight=1)

        from business_panel import BusinessPanel
        self.business_panel=BusinessPanel(self,outer)
        ttk.Button(outer,text="两轴联动计时 / 预载固定点位",command=self._open_joint_timing).pack(anchor="w",pady=4)
        self.advanced_open=tk.BooleanVar(value=False)
        ttk.Checkbutton(outer,text="展开运动与参数调试",variable=self.advanced_open,
                        command=self._toggle_advanced).pack(anchor="w")

        control = ttk.LabelFrame(outer, text="定角运动", padding=14)
        control.pack(fill="x", pady=14)
        self.advanced_control=control

        ttk.Label(control, text="电机").grid(row=0, column=0, sticky="w", pady=5)
        motor_box = ttk.Frame(control)
        motor_box.grid(row=0, column=1, sticky="w")
        self.motor_buttons = []
        for device in (1, 2):
            button = ttk.Radiobutton(motor_box, text=f"Motor {device}", variable=self.motor_id, value=device)
            button.pack(side="left", padx=(0, 14))
            self.motor_buttons.append(button)

        ttk.Label(control, text="输出轴速度 (rpm)").grid(row=1, column=0, sticky="w", pady=5)
        ttk.Spinbox(control, from_=0.01, to=100, increment=0.1, textvariable=self.speed_rpm, width=14).grid(row=1, column=1, sticky="w")

        ttk.Label(control, text="输出轴角度 (°)").grid(row=2, column=0, sticky="w", pady=5)
        ttk.Spinbox(control, from_=0.01, to=36000, increment=1, textvariable=self.angle_deg, width=14).grid(row=2, column=1, sticky="w")

        ttk.Label(control, text="减速比（电机:输出）").grid(row=3, column=0, sticky="w", pady=5)
        ttk.Label(control, text="50:1（由 STM32 统一换算）").grid(row=3, column=1, columnspan=2, sticky="w")

        ttk.Label(control, text="电机每圈指令脉冲 PA11").grid(row=4, column=0, sticky="w", pady=5)
        ppr = ttk.Spinbox(control, from_=1, to=1000000, textvariable=self.pulses_per_rev, width=14)
        ppr.grid(row=4, column=1, sticky="w")
        ttk.Button(control, text="读取 PA11", command=self._read_ppr).grid(row=4, column=2, sticky="w", padx=8)

        ttk.Label(control, text="换算结果").grid(row=5, column=0, sticky="nw", pady=5)
        ttk.Label(control, textvariable=self.calculated, justify="left").grid(row=5, column=1, columnspan=2, sticky="w")

        profile = ttk.LabelFrame(control, text="驱动器内部加减速（临时生效）", padding=8)
        profile.grid(row=6, column=0, columnspan=3, sticky="ew", pady=(8, 4))
        ttk.Label(profile, text="PA40 加速").grid(row=0, column=0, padx=(0, 4))
        ttk.Spinbox(profile, from_=1, to=10000, textvariable=self.accel_ms, width=8).grid(row=0, column=1)
        ttk.Label(profile, text="ms   PA41 减速").grid(row=0, column=2, padx=(8, 4))
        ttk.Spinbox(profile, from_=1, to=10000, textvariable=self.decel_ms, width=8).grid(row=0, column=3)
        ttk.Label(profile, text="ms   PA42 S曲线").grid(row=0, column=4, padx=(8, 4))
        ttk.Spinbox(profile, from_=0, to=1000, textvariable=self.s_curve_ms, width=8).grid(row=0, column=5)
        ttk.Label(profile, text="ms").grid(row=0, column=6, padx=(4, 8))
        ttk.Button(profile, text="应用", command=self._apply_profile).grid(row=0, column=7, padx=4)
        ttk.Button(profile, text="读取", command=self._read_profile).grid(row=0, column=8, padx=4)
        ttk.Label(profile, text="PA42=0 为梯形；大于0为S型").grid(row=1, column=0, columnspan=9, sticky="w", pady=(6, 0))

        button_row = ttk.Frame(control)
        button_row.grid(row=7, column=0, columnspan=3, sticky="ew", pady=(14, 4))
        ttk.Button(button_row, text="◀ 反向转动", command=lambda: self._move(-1)).pack(side="left", expand=True, fill="x")
        ttk.Button(button_row, text="正向转动 ▶", command=lambda: self._move(1)).pack(side="left", expand=True, fill="x", padx=10)
        stop = tk.Button(button_row, text="停止", command=self._stop, bg="#c62828", fg="white", font=("Microsoft YaHei UI", 10, "bold"))
        stop.pack(side="left", expand=True, fill="x")

        tools = ttk.Frame(control)
        tools.grid(row=8, column=0, columnspan=3, sticky="ew", pady=(8, 0))
        ttk.Button(tools, text="检查关键参数", command=self._check_parameters).pack(side="left")
        ttk.Button(tools, text="检查运行状态", command=self._check_status).pack(side="left")
        ttk.Button(tools, text="读取报警", command=self._read_alarm).pack(side="left")
        ttk.Button(tools, text="复位报警", command=self._reset_alarm).pack(side="left", padx=8)
        ttk.Button(tools, text="位置与限位状态", command=self._open_debug).pack(side="left", padx=4)
        ttk.Label(tools, text="DB44 外部 SON：本软件不发送 Servo ON/OFF").pack(side="right")

        auto_box = ttk.LabelFrame(control, text="自动连续旋转（直到停止）", padding=8)
        auto_box.grid(row=9, column=0, columnspan=3, sticky="ew", pady=(10, 0))
        ttk.Label(auto_box, text="输出轴角速度").pack(side="left")
        ttk.Spinbox(auto_box, from_=0.12, to=600, increment=1,
                    textvariable=self.auto_speed, width=8).pack(side="left", padx=6)
        ttk.Label(auto_box, text="°/秒").pack(side="left")
        ttk.Radiobutton(auto_box, text="正向", variable=self.auto_direction, value=1).pack(side="left", padx=6)
        ttk.Radiobutton(auto_box, text="反向", variable=self.auto_direction, value=-1).pack(side="left")
        ttk.Button(auto_box, text="开始自动旋转", command=self._start_auto).pack(side="left", padx=12)
        ttk.Button(auto_box, text="停止自动旋转", command=self._stop).pack(side="left")
        ttk.Button(auto_box, text="两轴轨迹试验", command=self._open_trajectory).pack(side="right",padx=8)
        ttk.Button(auto_box, text="一键双轴小幅调试", command=self._quick_dual_test).pack(side="right",padx=4)

        if self.LIMIT_FEATURES_ENABLED:
            limit_row = ttk.Frame(control)
            limit_row.grid(row=10, column=0, columnspan=3, sticky="ew", pady=(12, 0))
            ttk.Checkbutton(limit_row, text="无传感器调试（旁路当前电机限位）",
                            variable=self.limit_debug, command=self._toggle_limit_debug).pack(anchor="w")
            tk.Label(limit_row, textvariable=self.limit_mode_text, fg="#c62828",
                     anchor="w").pack(fill="x", pady=(4, 0))

        result = ttk.LabelFrame(outer, text="状态", padding=12)
        self.result_box=result
        result.pack(fill="both", expand=True)
        ttk.Label(result, textvariable=self.result_text, font=("Microsoft YaHei UI", 11, "bold")).pack(anchor="w")
        self.log = tk.Text(result, height=12, state="disabled", wrap="word")
        self.log.pack(fill="both", expand=True, pady=(10, 0))
        self._log("界面速度和角度均指减速器输出轴；软件停止不能代替硬件急停。")
        control.pack_forget()

    def _open_joint_timing(self):
        if self.joint_timing_panel is None:
            from joint_timing_panel import JointTimingPanel
            self.joint_timing_panel=JointTimingPanel(self)
        else:
            self.joint_timing_panel.window.deiconify()
            self.joint_timing_panel.window.lift()

    def _open_axis1_homing(self):
        self._open_debug()
        self.debug_panel.show_dual()

    def _reset_axis1(self):
        self._open_axis1_homing()

    def _toggle_advanced(self):
        if self.advanced_open.get():self.advanced_control.pack(fill="x",pady=8,before=self.result_box)
        else:self.advanced_control.pack_forget()

    def _refresh_ports(self) -> None:
        names = [item.device for item in list_ports.comports()] if list_ports else []
        self.port_combo["values"] = names
        if names and self.port_name.get() not in names:
            self.port_name.set(names[0])
        if not names:
            self.port_name.set("")

    def _open_debug(self) -> None:
        if self.debug_panel is None:
            from debug_panel import DebugPanel
            self.debug_panel = DebugPanel(self)
        else:
            self.debug_panel.window.deiconify()
            self.debug_panel.window.lift()

    def _lock_auto_axis(self, locked: bool) -> None:
        for button in self.motor_buttons:
            button.configure(state="disabled" if locked else "normal")

    def _open_trajectory(self) -> None:
        if self.trajectory_panel is None:
            from trajectory_panel import TrajectoryPanel
            self.trajectory_panel=TrajectoryPanel(self)
        else:
            self.trajectory_panel.window.deiconify();self.trajectory_panel.window.lift()

    def _quick_dual_test(self) -> None:
        self._open_trajectory()
        self.trajectory_panel.quick_test()

    def _start_auto(self) -> None:
        if getattr(self,"trajectory_running",False) or getattr(getattr(self,"trajectory_panel",None),"quick_phase",None):
            messagebox.showwarning("轨迹运行中","请先停止两轴轨迹并确认停稳")
            return
        if self.port is None:
            messagebox.showwarning("未连接", "请先连接STM32")
            return
        if (self.auto_motor is not None or getattr(self,"active_motor",None) is not None or (self.pending and not getattr(self,"pending_background",False)) or self.queued_motion or self.diagnostic_active
                or self.stop_waiting or self.monitor_until > time.monotonic()):
            messagebox.showwarning("设备忙", "请先停止当前动作并等待操作完成，再开始自动旋转")
            return
        try:
            speed = float(self.auto_speed.get()) * self.auto_direction.get()
            frame = build_continuous_rotation(self.motor_id.get(), speed)
            motor_rpm = int(math.floor(abs(speed) * self.GEAR_RATIO / 6 + 0.5))
            if not 1 <= motor_rpm <= 5000:
                raise ValueError("换算后的电机速度必须为1～5000 rpm")
        except (ValueError, tk.TclError) as exc:
            messagebox.showerror("角速度错误", str(exc))
            return
        if not messagebox.askyesno("连续旋转", "电机会持续旋转，直到点击停止。\n"
                                  "当前没有限位保护，请确认行程、线缆不会撞限位或缠绕，且硬件急停可用。\n"
                                  f"确认开始 Motor {self.motor_id.get()}：{speed:+g}°/秒？"):
            return
        self.auto_motor = frame[3]
        self.queued_auto = (self.auto_motor, speed)
        self.auto_renew = False
        self._lock_auto_axis(True)
        self.stop_waiting = True
        self.diagnostic_active = False
        if not self._send(build_request(self.auto_motor, CMD_STOP), CMD_STOP, "自动旋转前先停止旧动作"):
            self.queued_auto = None
            self.auto_motor = None
            self.stop_waiting = False
            self._lock_auto_axis(False)

    def _send_queued_auto(self) -> None:
        if self.queued_auto is None:
            return
        device, speed = self.queued_auto
        self.queued_auto = None
        self._send(build_continuous_rotation(device, speed), CMD_ROTATE_CONTINUOUS,
                   f"Motor {device} 连续旋转 {speed:+g}°/秒")

    def _service_auto(self) -> None:
        if (not self.auto_renew or self.auto_motor is None or self.port is None or self.pending
                or self.diagnostic_active or self.stop_waiting or self.queued_auto is not None
                or self.parameter_check_queue is not None or self.profile_read_queue is not None):
            return
        if time.monotonic() >= self.auto_renew_at:
            self._send(build_rotation_keepalive(self.auto_motor), CMD_ROTATE_KEEPALIVE,
                       f"Motor {self.auto_motor} 连续旋转心跳")

    def _show_limit_mode(self) -> None:
        if not self.LIMIT_FEATURES_ENABLED:
            return
        device = self.motor_id.get()
        mode = self.limit_modes[device]
        self.limit_debug.set(mode is True)
        if mode is True:
            self.limit_mode_text.set(f"Motor {device}：限位保护已旁路！仅用于有人看护的低速调试")
        elif mode is False:
            self.limit_mode_text.set(f"Motor {device}：限位保护开启")
        else:
            self.limit_mode_text.set(f"Motor {device}：限位模式未知，请检查运行状态；不会自动开启旁路")

    def _toggle_limit_debug(self) -> None:
        if not self.LIMIT_FEATURES_ENABLED:
            return
        device = self.motor_id.get()
        enable = self.limit_debug.get()
        self._show_limit_mode()  # Only an acknowledged state may stay checked.
        if self.pending is not None or self.queued_motion is not None or self.stop_waiting or self.diagnostic_active:
            messagebox.showwarning("设备忙", "请等待当前操作完成；运动中请先点击停止并确认停稳。")
            return
        if enable and not messagebox.askyesno(
            "旁路限位保护",
            f"仅对 Motor {device} 开启无传感器调试。\n"
            "遮挡、断线都将不再触发该轴限位停机，可能撞到机械极限。\n"
            "请确保有人看护、低速小角度试转，并准备硬件急停。\n"
            "此操作先确认停稳，不会自动开始转动。确认开启？",
        ):
            return
        self.queued_motion = None
        self.monitor_until = 0.0
        self.limit_renew[device] = False
        action = 1 if enable else 0
        self.limit_action = (device, action)
        if not self._send(build_limit_debug(device, action), CMD_LIMIT_DEBUG,
                          f"Motor {device} {'申请限位旁路' if enable else '恢复限位保护并停止'}"):
            self.limit_action = None

    def _service_limit_debug(self) -> None:
        if not self.LIMIT_FEATURES_ENABLED:
            return
        if self.port is None or self.pending is not None or self.diagnostic_active or self.queued_motion is not None:
            return
        now = time.monotonic()
        for device in (1, 2):
            if self.limit_renew[device] and now >= self.limit_renew_at[device]:
                self.limit_action = (device, 2)
                if not self._send(build_limit_debug(device, 2), CMD_LIMIT_DEBUG,
                                  f"Motor {device} 限位旁路续期（5秒）"):
                    self.limit_renew[device] = False
                    self.limit_action = None
                break

    def _toggle_connection(self) -> None:
        if self.port is not None:
            self._disconnect()
            return
        if self.connection_mode.get()=="串口" and serial is None:
            messagebox.showerror("缺少依赖", "未安装 pyserial。请双击 start_simple.bat 启动。")
            return
        name = (f"tcp://{self.tcp_host.get().strip()}:{self.tcp_port.get().strip()}"
                if self.connection_mode.get()=="TCP网口" else self.port_name.get())
        if not name:
            messagebox.showwarning("没有串口", "没有检测到可用串口，请检查 USB 串口线。")
            return
        try:
            from transport import open_transport
            self.port = open_transport(name,timeout=3.0,write_timeout=0.5)
            self.port.reset_input_buffer()
            self.parser.reset()
            self.pending = None
        except Exception as exc:
            self.port = None
            messagebox.showerror("连接失败", str(exc))
            return
        self.connection_text.set(f"已连接 {name}" + (" / 115200 8N1" if not name.startswith('tcp://') else ""))
        self.connect_button.configure(text="断开")
        self._log(f"已连接 {name}")
        self.limit_modes = {1: None, 2: None}
        self.limit_renew = {1: False, 2: False}
        self.limit_action = None
        self._show_limit_mode()

    def _disconnect(self) -> None:
        if getattr(self,"joint_timing_panel",None):self.joint_timing_panel.disconnected()
        if getattr(self,'_timing',None):self._timing.finish_all('connection_closed',execution_state='unknown_if_unfinished')
        # Preserve active_motor until the existing disconnect stop has been sent.
        business=getattr(self,"business_panel",None)
        if getattr(self,"trajectory_panel",None):
            if self.port is not None and (getattr(self,"trajectory_running",False) or self.trajectory_panel.jobs or self.trajectory_panel.quick_phase):
                try:self.port.write(build_request(1,0x35))
                except Exception:pass
                self._log("已尽力请求停止两轴轨迹；未确认停稳，断线后不续期；硬件急停仍必需")
            self.trajectory_panel.disconnected()
        if getattr(self,"debug_panel",None): self.debug_panel.cancel()
        self.home_running = False
        known_axis = self.auto_motor if self.auto_motor is not None else getattr(self,"active_motor",None)
        if known_axis is not None and self.port is not None:
            try:
                self.port.write(build_request(known_axis, CMD_STOP))
            except Exception:
                pass
            self._log("已尝试停止；自动旋转/回零另有心跳超时停机，定角动作断线不保证停机；不能代替硬件急停")
        self.auto_renew = False
        self.queued_auto = None
        self.auto_motor = None
        self.active_motor = None
        if business:business.cancel("通信已断开，实际位置未知；重新连接后读取参考")
        self._lock_auto_axis(False)
        if any(self.limit_renew.values()):
            self._log("断开后不再续期限位旁路：STM32在最后一次续期约5秒后恢复保护并请求停止。")
        if self.port is not None:
            try:
                self.port.close()
            except Exception:
                pass
        self.port = None
        self.pending = None
        self.deferred_send = None
        self.pending_background = False
        self.queued_motion = None
        self.parameter_check_queue = None
        self.profile_read_queue = None
        self.monitor_until = 0.0
        self.motion_seen_running = False
        self.stop_waiting = False
        self.limit_modes = {1: None, 2: None}
        self.limit_renew = {1: False, 2: False}
        self.limit_action = None
        self._show_limit_mode()
        self.connection_text.set("未连接")
        self.connect_button.configure(text="连接")
        self._log("串口已断开")

    def _update_calculation(self) -> None:
        try:
            motion = convert_output_motion(
                abs(float(self.speed_rpm.get())),
                abs(float(self.angle_deg.get())),
                int(self.pulses_per_rev.get()),
                self.GEAR_RATIO,
            )
            self.calculated.set(
                f"电机 {motion.motor_speed_rpm} rpm / {motion.motor_angle_deg:g}° / "
                f"{motion.pulses:,} pulse\n"
                f"取整后的输出速度 {motion.actual_output_speed_rpm:g} rpm"
            )
        except (ValueError, tk.TclError):
            self.calculated.set("输入无效")

    def _motion_values(self) -> MotionConversion | None:
        try:
            motion = convert_output_motion(
                abs(float(self.speed_rpm.get())),
                abs(float(self.angle_deg.get())),
                int(self.pulses_per_rev.get()),
                self.GEAR_RATIO,
            )
        except (ValueError, tk.TclError):
            messagebox.showerror("输入错误", "请检查输出轴速度、角度和 PA11。")
            return None
        return motion

    def _schedule_motion_callback(self, delay, callback):
        generation=getattr(self,"motion_generation",0)
        self.root.after(delay,lambda: callback() if generation==getattr(self,"motion_generation",0) else None)

    def _move(self, direction: int) -> None:
        if getattr(self,"trajectory_running",False) or getattr(getattr(self,"trajectory_panel",None),"quick_phase",None):
            messagebox.showwarning("轨迹运行中","请先停止两轴轨迹并确认停稳")
            return
        if self.auto_motor is not None:
            messagebox.showwarning("自动旋转中", "请先停止自动旋转并确认停稳，再执行定角动作")
            return
        motion = self._motion_values()
        if motion is None:
            return
        device_id = self.motor_id.get()
        if getattr(self,'_timing',None):self._timing.begin(device_id,'gui_angle_move')
        self.motion_generation=getattr(self,"motion_generation",0)+1
        self.stop_zero_samples=0
        self.queued_motion = (device_id, direction, motion)
        if self.stop_waiting:
            self.result_text.set("正在停止，已用最新方向命令替换等待命令…")
            self._schedule_motion_callback(20,self._monitor_motion)
            return
        if self.monitor_until > time.monotonic() or getattr(self,"active_motor",None) is not None:
            self.stop_waiting = True
            self.diagnostic_active = False
            self.last_drive_status = ""
            self.result_text.set("方向改变：正在减速到0 rpm，随后自动执行新方向…")
            frame = build_request(device_id, CMD_STOP)
            if not self._send(frame, CMD_STOP, f"Motor {device_id} 改向前停止", allow_override=True):
                self.stop_waiting = False
                self.queued_motion = None
            return
        self.diagnostic_active = True
        self.last_drive_status = ""
        self.result_text.set("运动前检查 SON 和报警…")
        if not self._send(build_request(device_id, CMD_READ_SPEED), CMD_READ_SPEED,
                          f"运动前检查 Motor {device_id} 运行状态"):
            self.queued_motion = None

    def _send_queued_motion(self) -> None:
        if self.queued_motion is None:
            return
        device_id, direction, motion = self.queued_motion
        self.queued_motion = None
        signed_angle = motion.output_angle_deg * direction
        expected_seconds = motion.output_angle_deg / (6.0 * motion.output_speed_rpm)
        self.next_monitor_seconds = min(max(expected_seconds + 8.0, 5.0), 3600.0)
        self.motion_seen_running = False
        frame = build_move_output_relative(device_id, motion.output_speed_rpm, signed_angle)
        side = "正向" if direction > 0 else "反向"
        self._send(frame, CMD_MOVE_OUTPUT_RELATIVE,
                   f"Motor {device_id} {side}输出 {signed_angle:+g}° / "
                   f"{motion.output_speed_rpm:g} rpm；电机 {motion.motor_angle_deg:g}° / "
                   f"{motion.motor_speed_rpm} rpm；STM32按50:1换算")

    def _stop(self) -> None:
        joint=getattr(self,"joint_timing_panel",None)
        if joint and joint.phase:
            joint.stop()
            return
        self.motion_generation=getattr(self,"motion_generation",0)+1
        self.stop_zero_samples=0
        business=getattr(self,"business_panel",None)
        if business:
            owned_axis=business.axis if business.phase else None
            business.cancel("已请求停止，所有准备中的后续动作已取消")
            if owned_axis is not None:self.active_motor=owned_axis
        if getattr(self,"trajectory_panel",None) and (getattr(self,"trajectory_running",False) or self.trajectory_panel.jobs or self.trajectory_panel.quick_phase):
            self.trajectory_panel.stop()
            return
        # 停止命令允许覆盖正在等待的普通命令，但仍不能代替硬件急停。
        self.pending = None
        self.deferred_send = None
        self.pending_background = False
        self.parameter_check_queue = None
        self.profile_read_queue = None
        if getattr(self,"debug_panel",None): self.debug_panel.cancel()
        self.home_running = False
        self.auto_renew = False
        self.queued_auto = None
        self.queued_motion = None
        self.stop_waiting = True
        self.diagnostic_active = False
        self.monitor_until = time.monotonic() + 30.0
        self.motion_seen_running = False
        device = self.auto_motor if self.auto_motor is not None else getattr(self,"active_motor",None)
        if device is None: device = self.motor_id.get()
        frame = build_request(device, CMD_STOP)
        if not self._send(frame, CMD_STOP, f"Motor {device} 停止并等待0 rpm", allow_override=True):
            self.stop_waiting = False

    def _check_status(self) -> None:
        self.queued_motion = None
        if self.auto_motor is None and getattr(self,"active_motor",None) is None:
            self.monitor_until = 0.0
        self.diagnostic_active = True
        self.last_drive_status = ""
        self._send(build_request(self.motor_id.get(), CMD_READ_SPEED), CMD_READ_SPEED,
                   f"检查 Motor {self.motor_id.get()} 运行状态")

    def _monitor_motion(self) -> None:
        if getattr(self,"home_running",False):
            return  # The feedback service monitors STM32's homing phases.
        if self.port is None:
            self.monitor_until = 0.0
            return
        if time.monotonic() >= self.monitor_until:
            self.monitor_until = 0.0
            if self.stop_waiting:
                self.stop_waiting = False
                self.queued_motion = None
                self.queued_auto = None
                self.diagnostic_active = False
                message = "停止确认超时，已取消自动换向；请检查驱动器状态后重新操作"
                self.result_text.set(message)
                self._log(message)
            return
        if self.pending is not None:
            self._schedule_motion_callback(20 if self.stop_waiting else 100, self._monitor_motion)
            return
        self.diagnostic_active = True
        self.last_drive_status = ""
        self._send(build_request(self.motor_id.get(), CMD_READ_SPEED), CMD_READ_SPEED,
                   f"监测 Motor {self.motor_id.get()} 运动状态")

    def _release_stopped_axis(self, axis):
        """Release UI ownership only after an actual zero-speed response."""
        owner=self.auto_motor if self.auto_motor is not None else getattr(self,"active_motor",None)
        if owner is not None and owner!=axis:return False
        self.stop_waiting=False;self.monitor_until=0.0;self.motion_seen_running=False
        self.diagnostic_active=False;self.auto_renew=False;self.home_running=False
        self.auto_motor=None;self.active_motor=None
        self._lock_auto_axis(False)
        return True

    def _read_alarm(self, manual: bool = True) -> None:
        if manual:
            self.diagnostic_active = False
        self._send(build_request(self.motor_id.get(), CMD_READ_ALARM), CMD_READ_ALARM, f"读取 Motor {self.motor_id.get()} 报警")

    def _read_ppr(self) -> None:
        self._send(build_request(self.motor_id.get(), CMD_READ_PA, parameter=11), CMD_READ_PA,
                   f"读取 Motor {self.motor_id.get()} PA11")

    def _profile_inputs(self) -> tuple[int, int, int] | None:
        try:
            accel = int(self.accel_ms.get())
            decel = int(self.decel_ms.get())
            s_curve = int(self.s_curve_ms.get())
            build_motion_profile(self.motor_id.get(), accel, decel, s_curve)
        except (ValueError, tk.TclError) as exc:
            messagebox.showerror("曲线参数错误", str(exc) or "请输入有效的PA40、PA41、PA42。")
            return None
        return accel, decel, s_curve

    def _apply_profile(self) -> None:
        values = self._profile_inputs()
        if values is None:
            return
        if self.pending is not None and not getattr(self,"pending_background",False):
            messagebox.showwarning("设备忙", "上一条命令仍在等待应答。")
            return
        accel, decel, s_curve = values
        profile_name = "梯形" if s_curve == 0 else "S型"
        if not messagebox.askyesno(
            "应用运动曲线",
            f"确认在电机停止时向 Motor {self.motor_id.get()} 临时应用{profile_name}参数？\n"
            f"PA40={accel}，PA41={decel}，PA42={s_curve}\n掉电后恢复驱动器原保存值。",
        ):
            return
        frame = build_motion_profile(self.motor_id.get(), accel, decel, s_curve)
        self._send(frame, CMD_SET_MOTION_PROFILE,
                   f"Motor {self.motor_id.get()} 临时设置 PA40={accel}, PA41={decel}, PA42={s_curve}")

    def _read_profile(self) -> None:
        if self.pending is not None and not getattr(self,"pending_background",False):
            messagebox.showwarning("设备忙", "上一条命令仍在等待应答。")
            return
        self.profile_values.clear()
        self.profile_read_queue = [40, 41, 42]
        self.result_text.set("正在读取 PA40、PA41、PA42…")
        self._read_next_profile_parameter()

    def _read_next_profile_parameter(self) -> None:
        if not self.profile_read_queue:
            self.profile_read_queue = None
            accel = self.profile_values.get(40)
            decel = self.profile_values.get(41)
            s_curve = self.profile_values.get(42)
            if None not in (accel, decel, s_curve):
                self.accel_ms.set(accel)
                self.decel_ms.set(decel)
                self.s_curve_ms.set(s_curve)
                profile_name = "梯形" if s_curve == 0 else "S型"
                message = f"Motor {self.motor_id.get()} {profile_name}：PA40={accel}, PA41={decel}, PA42={s_curve}"
                self.result_text.set(message)
                self._log(message)
            return
        number = self.profile_read_queue.pop(0)
        if not self._send(build_request(self.motor_id.get(), CMD_READ_PA, parameter=number), CMD_READ_PA,
                          f"读取 Motor {self.motor_id.get()} PA{number}"):
            self.profile_read_queue = None

    def _check_parameters(self, device_id: int | None = None) -> None:
        if self.pending is not None and not getattr(self,"pending_background",False):
            messagebox.showwarning("设备忙", "上一条命令仍在等待应答。")
            return
        if (getattr(self,"trajectory_running",False) or self.auto_motor is not None or
            getattr(getattr(self,"business_panel",None),"phase",None) or self.parameter_check_queue is not None):
            messagebox.showwarning("请先停止","请先停止业务／准备流程，再读取上电参数；保持外部SON关闭。")
            return
        self.parameter_check_axis=self.motor_id.get() if device_id is None else device_id
        self.parameter_values.clear()
        self.parameter_check_queue = list(self.STARTUP_PA_PARAMETERS)
        self.result_text.set(f"正在读取电机{self.parameter_check_axis}上电相关参数；仅查询，不使能、不改参数")
        self._read_next_parameter()

    def _read_next_parameter(self) -> None:
        if not self.parameter_check_queue:
            self.parameter_check_queue = None
            axis=getattr(self,"parameter_check_axis",self.motor_id.get())
            values=self.parameter_values
            raw="，".join(f"PA{n}={values.get(n,'?')}" for n in self.STARTUP_PA_PARAMETERS)
            self._log(f"Motor {axis} 上电参数实读：{raw}")
            expected = {4: 3, 14: 3, 53: 0, 72: 1152, 73: 0}
            errors = [f"PA{number}={self.parameter_values.get(number, '?')}（应为{value}）"
                      for number, value in expected.items()
                      if self.parameter_values.get(number) != value]
            if errors:
                message = f"Motor {axis} 配套参数不一致：" + "，".join(errors)
                self.result_text.set(message)
                self._log(message)
            else:
                message = f"Motor {axis} 控制/通信参数读回匹配；尚不能据此确认上电不会运动"
                self.result_text.set(message)
                self._log(message)
            if values.get(22)!=1 or any(values.get(n)!=0 for n in (24,25,26,27)):
                risk="若这是冷启动后、尚未发送运动指令时的参数，存在上电速度来源或残留速度风险；保持SON关闭，保存完整日志核对。"
                self._log(risk);self.result_text.set(message+"；"+risk)
            else:
                self._log("速度来源为内部且四档为0；仍需核对P3强制输入、实际控制模式与外部输入，不能仅凭PA值认定自转原因已排除。")
            return
        number = self.parameter_check_queue.pop(0)
        axis=getattr(self,"parameter_check_axis",self.motor_id.get())
        if not self._send(build_request(axis, CMD_READ_PA, parameter=number), CMD_READ_PA,
                          f"读取 Motor {axis} PA{number}"):
            self.parameter_check_queue = None

    def _reset_alarm(self) -> None:
        if self.auto_motor is not None:
            messagebox.showwarning("请先停止", "自动旋转期间请先停止并确认停稳，再复位报警，避免恢复旧速度指令")
            return
        if not messagebox.askyesno("复位报警", "确认故障原因已经排除，并复位当前电机报警吗？"):
            return
        self._send(build_request(self.motor_id.get(), CMD_RESET_ALARM), CMD_RESET_ALARM, f"复位 Motor {self.motor_id.get()} 报警")

    def _service_foreground(self) -> None:
        if self.pending is None and getattr(self,"deferred_send",None) is not None:
            frame,command,description = self.deferred_send
            self.deferred_send = None
            self._send(frame,command,description)

    def _send(self, frame: bytes, command: int, description: str, allow_override: bool = False,
              *, background: bool = False, preparation_step: bool = False, timing_step: bool = False) -> bool:
        joint=getattr(self,"joint_timing_panel",None)
        if joint and joint.phase and not timing_step:
            if command==CMD_STOP:joint.stop()
            elif not background:messagebox.showwarning("联动计时中","请先停止两轴联动计时，再执行其他操作。")
            return False
        quick=getattr(getattr(self,"trajectory_panel",None),"quick_phase",None)
        preparation=getattr(getattr(self,"business_panel",None),"phase",None)
        if preparation and not preparation_step and command not in (0x06,0x37):
            if not background:messagebox.showwarning("业务准备中","请先完成或停止准备流程")
            return False
        if quick and not (0x30<=command<=0x38 or command in (0x06,0x09,0x20,0x24)):
            self.diagnostic_active=False;self.parameter_check_queue=None;self.profile_read_queue=None
            messagebox.showwarning("一键调试准备中","请先完成或停止一键调试，再操作其他运动／参数")
            return False
        if getattr(self,"trajectory_running",False) and not (0x30<=command<=0x38 or command in (0x06,0x07,0x08,0x09,0x20,0x24)):
            messagebox.showwarning("轨迹运行中","轨迹运行／停止确认期间禁止其他运动或参数修改，请先停止两轴轨迹")
            return False
        if allow_override:
            self.deferred_send = None
        if self.port is None:
            messagebox.showwarning("未连接", "请先连接 STM32 串口。")
            return False
        if self.pending is not None and not allow_override:
            if not background and getattr(self,"pending_background",False):
                queued=getattr(self,"deferred_send",None)
                if queued is None or (queued[0][3],queued[1])==(frame[3],command):
                    self.deferred_send=(frame,command,description)
                    self.result_text.set("操作已接收：等待当前反馈返回后优先执行，不等待电机走完")
                    return True
            if background:
                return False
            messagebox.showwarning("设备忙", "上一条命令仍在等待应答。")
            return False
        try:
            if command in (CMD_MOVE_RELATIVE,CMD_MOVE_OUTPUT_RELATIVE,0x1A,0x43):
                self.active_motor=frame[3]
                self._lock_auto_axis(True)
            if getattr(self,'_timing',None):self._timing.sent(frame)
            self.port.write(frame)
            if command in (CMD_MOVE_RELATIVE,CMD_MOVE_OUTPUT_RELATIVE,0x22,0x28,0x17):
                self.last_motion_sent_at=time.monotonic()
        except Exception as exc:
            self.result_text.set("发送失败")
            self._log(f"发送失败：{exc}")
            self._disconnect()
            return False
        self.pending = (frame[3], command, time.monotonic())
        self.pending_parameter = struct.unpack(">H",frame[5:7])[0]
        self.pending_background = background
        self.result_text.set("等待 STM32 应答…")
        self._log(f"TX {frame.hex(' ').upper()}  {description}")
        return True

    def _poll_serial(self) -> None:
        if self.port is not None:
            try:
                waiting = self.port.in_waiting
                if waiting:
                    raw = self.port.read(waiting)
                    for packet in self.parser.feed(raw):
                        self._handle_response(packet)
                if self.pending is not None and time.monotonic() - self.pending[2] > self.RESPONSE_TIMEOUT_S:
                    if getattr(self,'_timing',None):self._timing.finish(self.pending[0],'response_timeout',command=self.pending[1])
                    command = self.pending[1]
                    self.pending = None
                    self.deferred_send = None
                    self.pending_background = False
                    self.diagnostic_active = False
                    self.parameter_check_queue = None
                    self.profile_read_queue = None
                    if self.stop_waiting:
                        self.stop_waiting = False
                        self.queued_motion = None
                        self.monitor_until = 0.0
                        self.diagnostic_active = False
                    self.result_text.set("通信超时：未收到 STM32 应答")
                    self._log(f"命令 0x{command:02X} 应答超时")
                    if getattr(self,"joint_timing_panel",None):self.joint_timing_panel.stop("通信应答超时，取消后续段")
                    if getattr(self,"debug_panel",None): self.debug_panel.cancel()
                    if getattr(self,"trajectory_panel",None):self.trajectory_panel.on_error()
                    if getattr(self,"business_panel",None):self.business_panel.fail("通信超时，已取消准备；位置未知，请检查停机状态")
                    self.auto_renew = False
                    self.queued_auto = None
                    # Do not keep a safety bypass alive after uncertain communication.
                    self.limit_renew = {1: False, 2: False}
                    self.limit_modes = {1: None, 2: None}
                    self.limit_action = None
                    self.queued_motion = None
                    self._show_limit_mode()
                self._service_foreground()
                joint=getattr(self,"joint_timing_panel",None)
                if joint and joint.phase:
                    joint.service()
                    self.root.after(30, self._poll_serial)
                    return
                self._service_limit_debug()
                self._service_auto()
                if self.debug_panel is not None: self.debug_panel.service()
                if getattr(self,"business_panel",None):self.business_panel.service()
                if getattr(self,"trajectory_panel",None):self.trajectory_panel.service()
            except Exception as exc:
                self._log(f"串口异常：{exc}")
                self._disconnect()
        self.root.after(30, self._poll_serial)

    def _handle_response(self, packet: Response) -> None:
        self._log(f"RX {packet.raw.hex(' ').upper()}  {packet.status_text}")
        if (self.pending is None or (packet.device_id, packet.command) != self.pending[:2] or
            (getattr(self,"pending_parameter",None) is not None and packet.parameter!=self.pending_parameter)):
            self._log("忽略迟到或不匹配的应答")
            return
        self.pending = None
        self.pending_background = False
        self.pending_parameter = None
        if getattr(self,'_timing',None):self._timing.received(packet)
        joint=getattr(self,"joint_timing_panel",None)
        if joint and joint.handle(packet):return
        if packet.ok and packet.command in (0x22,0x25) and getattr(self,"trajectory_panel",None):
            self.trajectory_panel.invalidate()
        if getattr(self,"business_panel",None) and self.business_panel.handle(packet):return
        if not packet.ok:
            self.deferred_send = None
            self.queued_motion = None
            self.diagnostic_active = False
            self.parameter_check_queue = None
            self.profile_read_queue = None
        if getattr(self,"trajectory_panel",None) and self.trajectory_panel.handle(packet):
            return
        if getattr(self,"debug_panel",None) and self.debug_panel.handle(packet):
            return

        if not packet.ok:
            self.auto_renew = False
            self.queued_auto = None
            self.parameter_check_queue = None
            self.profile_read_queue = None
            if self.stop_waiting or packet.status == 0x08:
                self.stop_waiting = False
                self.queued_motion = None
                self.monitor_until = 0.0
                self.diagnostic_active = False
            self.result_text.set(f"Motor {packet.device_id} 命令失败：{packet.status_text}")
            if packet.command in (CMD_ROTATE_CONTINUOUS, CMD_ROTATE_KEEPALIVE) and packet.status == 7:
                self.result_text.set("固件不支持连续旋转，请烧录配套新固件；未自动重试启动")
            if self.LIMIT_FEATURES_ENABLED and packet.command == CMD_LIMIT_DEBUG:
                self.limit_renew[packet.device_id] = False
                self.limit_modes[packet.device_id] = None
                self.limit_action = None
                self._show_limit_mode()
                if packet.status == 0x07:
                    self.result_text.set("STM32固件不支持限位调试选项，请烧录配套v1.7固件；未确认开启旁路")
            return
        if packet.command == CMD_ROTATE_CONTINUOUS:
            if len(packet.data) != 2:
                self.auto_renew = False
                self.result_text.set("连续旋转应答损坏，请停止并核查状态")
                return
            rpm = struct.unpack(">h", packet.data)[0]
            self.auto_renew = True
            self.auto_renew_at = time.monotonic() + 1.0
            self.monitor_until = float("inf")
            self.result_text.set(f"Motor {packet.device_id} 自动旋转中：实际指令约 {rpm * 6 / self.GEAR_RATIO:+g}°/秒；点击停止结束")
            self._schedule_motion_callback(250, self._monitor_motion)
        elif packet.command == CMD_ROTATE_KEEPALIVE:
            self.auto_renew_at = time.monotonic() + 1.0
        elif self.LIMIT_FEATURES_ENABLED and packet.command == CMD_LIMIT_DEBUG:
            action = self.limit_action
            self.limit_action = None
            if action is None or action[0] != packet.device_id or len(packet.data) != 2:
                self.limit_renew[packet.device_id] = False
                self.limit_modes[packet.device_id] = None
                self._show_limit_mode()
                return
            enabled = struct.unpack(">H", packet.data)[0] == 1
            self.limit_modes[packet.device_id] = enabled
            self.limit_renew[packet.device_id] = enabled and action[1] in (1, 2)
            self.limit_renew_at[packet.device_id] = time.monotonic() + 1.0
            self._show_limit_mode()
            self.result_text.set(f"Motor {packet.device_id}：" + (
                "限位保护已旁路，仅限低速调试！" if enabled else "已恢复限位保护，正在确认停止"))
        elif packet.command in (CMD_MOVE_RELATIVE, CMD_MOVE_OUTPUT_RELATIVE):
            if packet.command == CMD_MOVE_OUTPUT_RELATIVE and len(packet.data) >= 4:
                accepted_millideg = struct.unpack(">i", packet.data[:4])[0]
                self._log(f"STM32 已接受带符号输出角度 {accepted_millideg / 1000:+g}°")
            self.result_text.set(f"转动指令已接收，预计监测 {self.next_monitor_seconds:.1f} 秒…")
            self.monitor_until = time.monotonic() + self.next_monitor_seconds
            self._schedule_motion_callback(250, self._monitor_motion)
        elif packet.command == CMD_SET_MOTION_PROFILE:
            self.result_text.set("曲线参数已临时写入，正在读回校验…")
            self.root.after(80, self._read_profile)
        elif packet.command == CMD_STOP:
            self.result_text.set("停止参数写入已应答，正在确认实际转速降至0 rpm…")
            self.monitor_until = time.monotonic() + 30.0
            self._schedule_motion_callback(20, self._monitor_motion)
        elif packet.command == CMD_READ_SPEED and len(packet.data) >= 4:
            speed, run_state = struct.unpack(">hH", packet.data[:4])
            if run_state == 3:
                state_text = "SON 已生效"
            else:
                state_text = f"运行状态={run_state}，SON 可能未生效"
                self.queued_motion = None
                self.queued_auto = None
            output_speed = speed / self.GEAR_RATIO
            self.last_drive_status = (
                f"Motor {packet.device_id}：{state_text}，电机 {speed} rpm，"
                f"输出轴约 {output_speed:g} rpm"
            )
            limit_flags = packet.data[4] if self.LIMIT_FEATURES_ENABLED and len(packet.data) >= 5 else 0
            if self.LIMIT_FEATURES_ENABLED and len(packet.data) >= 5:
                self.limit_modes[packet.device_id] = bool(limit_flags & 4)
                if not (limit_flags & 4):
                    self.limit_renew[packet.device_id] = False
                self._show_limit_mode()
            if limit_flags & 4:
                self.last_drive_status += "；警告：限位保护已旁路（无传感器调试）"
            if limit_flags & 3 and not (limit_flags & 4):
                self.queued_motion = None
                self.stop_waiting = False
                self.last_drive_status += "；限位触发/断线" if limit_flags & 1 else "；限位停机确认中"
                if speed == 0 and not (limit_flags & 2):
                    self.monitor_until = 0.0
            elif self.stop_waiting:
                self.stop_zero_samples=getattr(self,"stop_zero_samples",0)+1 if speed==0 else 0
                if self.stop_zero_samples >= 2:
                    if getattr(self,'_timing',None):self._timing.stop_confirmed(packet.device_id)
                    self.stop_waiting = False
                    self.monitor_until = 0.0
                    self.motion_seen_running = False
                    if self.queued_motion is None and self.queued_auto is None:
                        self.last_drive_status += "，已读到0 rpm；新动作需重新触发"
                        self._release_stopped_axis(packet.device_id)
                        if getattr(self,"debug_panel",None): self.root.after(0,self.debug_panel.stopped)
                    else:
                        self.last_drive_status += "，已停止，准备执行新方向"
                        self.diagnostic_active = True
                else:
                    self.last_drive_status += "，第一次零速，继续确认停稳" if speed==0 else "，正在停止"
                    self.diagnostic_active=False
                    self._schedule_motion_callback(20,self._monitor_motion)
            elif self.auto_motor is not None:
                self.last_drive_status += "；自动旋转中" if self.auto_renew else "；自动旋转停止待确认"
            elif self.monitor_until > time.monotonic():
                if abs(speed) > 0:
                    self.motion_seen_running = True
                elif self.motion_seen_running:
                    self.monitor_until = 0.0
                    self.motion_seen_running = False
                    self.active_motor = None
                    self._lock_auto_axis(False)
                    self.last_drive_status += "，已检测停转；请核对位置反馈是否到位"
            self.result_text.set(self.last_drive_status)
            self._log(self.last_drive_status)
            if self.diagnostic_active:
                self._schedule_motion_callback(0, lambda: self._read_alarm(False))
        elif packet.command == CMD_READ_ALARM and len(packet.data) >= 2:
            alarm = struct.unpack(">H", packet.data[:2])[0]
            alarm_text = "无报警" if alarm == 0 else f"驱动器报警 Err {alarm}"
            if self.diagnostic_active and self.last_drive_status:
                self.result_text.set(f"{self.last_drive_status}；{alarm_text}")
                self.diagnostic_active = False
                if alarm == 0 and self.queued_motion is not None and not self.stop_waiting:
                    self._schedule_motion_callback(0, self._send_queued_motion)
                elif alarm == 0 and self.queued_auto is not None and not self.stop_waiting:
                    self._schedule_motion_callback(0, self._send_queued_auto)
                elif alarm != 0:
                    self.auto_renew = False
                    self.queued_auto = None
                    self.queued_motion = None
                    self.monitor_until = 0.0
                elif self.monitor_until > time.monotonic():
                    self._schedule_motion_callback(250, self._monitor_motion)
            else:
                self.result_text.set(alarm_text)
        elif packet.command == CMD_READ_PA and packet.parameter == 11 and len(packet.data) >= 2:
            ppr = struct.unpack(">H", packet.data[:2])[0]
            if ppr == 0:
                self.result_text.set("PA11 返回 0，不能用于角度换算")
            else:
                self.pulses_per_rev.set(ppr)
                self._update_calculation()
                self.result_text.set(f"已从 Motor {packet.device_id} 读取 PA11 = {ppr}")
        elif (packet.command == CMD_READ_PA and self.parameter_check_queue is not None
              and packet.device_id==getattr(self,"parameter_check_axis",self.motor_id.get())
              and packet.parameter in self.STARTUP_PA_PARAMETERS and len(packet.data) >= 2):
            value = struct.unpack(">H", packet.data[:2])[0]
            self.parameter_values[packet.parameter] = value
            self._log(f"Motor {packet.device_id} PA{packet.parameter} = {value}")
            self.root.after(60, self._read_next_parameter)
        elif (packet.command == CMD_READ_PA and self.profile_read_queue is not None
              and packet.parameter in (40, 41, 42) and len(packet.data) >= 2):
            value = struct.unpack(">H", packet.data[:2])[0]
            self.profile_values[packet.parameter] = value
            self._log(f"Motor {packet.device_id} PA{packet.parameter} = {value}")
            self.root.after(60, self._read_next_profile_parameter)
        elif packet.command == CMD_RESET_ALARM:
            self.result_text.set("报警复位命令已执行")
        else:
            self.result_text.set("命令执行成功")

    def _log(self, text: str) -> None:
        stamp = dt.datetime.now().strftime("%H:%M:%S")
        self._log_queue.append(f"[{stamp}] {text}\n")
        if self._log_after is None:
            self._log_after=self.root.after(50,self._flush_log)

    def _flush_log(self) -> None:
        self._log_after=None
        if not self._log_queue:return
        text=''.join(self._log_queue)
        self._log_queue.clear()
        self.log.configure(state="normal")
        self.log.insert("end",text)
        # Bound Tk text layout work across long test sessions. Detailed motion
        # events remain in motion_timing.jsonl, independent of this display.
        lines=int(self.log.index('end-1c').split('.')[0])
        if lines>2000:self.log.delete('1.0',f'{lines-2000+1}.0')
        self.log.see("end")
        self.log.configure(state="disabled")

    def _on_close(self) -> None:
        if self.port is not None:
            try:
                self.port.write(build_request(self.motor_id.get(), CMD_STOP))
            except Exception:
                pass
            self._disconnect()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    try:
        ttk.Style(root).theme_use("vista")
    except tk.TclError:
        pass
    SimpleServoApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
