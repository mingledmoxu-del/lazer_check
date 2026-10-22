"""共享主窗口通信的电机1固定复位流程与状态窗口。"""
import math
import struct
import time
from collections import deque
import tkinter as tk
from tkinter import ttk, messagebox

from protocol import (CMD_READ_FEEDBACK,CMD_CONFIG_GUARD,CMD_SET_ZERO,CMD_HOME,CMD_READ_GUARD,
                      CMD_MOVE_OUTPUT_RELATIVE,build_request,build_move_output_relative)
from protocol import (CMD_CONFIG_DUAL_LIMIT,CMD_READ_DUAL_LIMIT,CMD_HOME_DUAL_LIMIT,
                      build_dual_limits,build_dual_home)

PHASES = ("未回零","寻找原点","触发后停稳","反向退出","退出后停稳","慢速二次接近","最终停稳","回零完成","回零失败")
FAULTS = ("无","输入触发/无法退出","软范围越界/搜索超行程","反馈单位变化/计数不连续","超时/心跳丢失","驱动通信异常","驱动器报警","人工取消","定位执行异常：方向错误或3秒未启动")
CHANNELS = ("未使用", "DI0", "DI1", "DI2", "DI3", "DI4", "DI5")

def format_inputs(raw, active, config=None):
    """Only interpret assigned inputs using a configuration read from firmware."""
    roles = {}
    if config is not None:
        for role, channel in zip(("负限位", "正限位", "原点"), config[:3]):
            if 0 <= channel < 6:
                roles.setdefault(channel, []).append(role)
    rows = []
    for n in range(6):
        value = (raw >> n) & 1
        if config is None:
            state = "未读取映射"
        elif n not in roles:
            state = "未使用"
        else:
            state = "/".join(roles[n]) + ("：触发" if active & (1 << n) else "：未触发")
        rows.append(f"DI{n}: raw={value} {state}")
    return "  ".join(rows)

class DebugPanel:
    def __init__(self, app):
        self.app=app
        self.window=tk.Toplevel(app.root)
        self.window.title("电机1复位 / 位置状态")
        self.window.geometry("900x480")
        self.window.protocol("WM_DELETE_WINDOW",self.close)
        self.jobs=deque();self.after_stop=None;self.goto_target=None;self.refresh_at=0.0
        self.start_error=None
        self.raised_request=None
        self.records=deque(maxlen=10000);self.actual_inputs={};self.models={}
        for axis in (1,2):
            self.models[axis]={key:tk.StringVar(value="0" if key=="target" else "") for key in
                ("negative","positive","home","polarity","direction","minimum","maximum","location","target")}
            for key in ("confirmed","hardware","soft"):
                self.models[axis][key]=tk.BooleanVar(value=False)
        self.dual={key:tk.StringVar(value=value) for key,value in dict(
            horizontal="DI2",raised="DI0",horizontal_deg="-29",raised_deg="35",
            polarity="低电平触发",sign="负向抬头",direction="负向",
            speed="0.6",budget="45",timeout="120").items()}
        self.dual["speed"]=getattr(app,"reset_speed_rpm",self.dual["speed"])
        self.last_feedback=None
        self.live=tk.StringVar(value="已有参考时直接回到+35°；无参考时寻找DI0建立参考。停止不会清除已有坐标。")
        self.refresh=tk.BooleanVar(value=True)
        self.input_text=tk.StringVar(value="尚未读取限位输入")
        self.config_text=tk.StringVar(value="固定配置：低电平触发；DI0=+35°，DI2=−29°；反向抬头。")
        outer=ttk.Frame(self.window,padding=16);outer.pack(fill="both",expand=True)
        ttk.Label(outer,text="电机1回到 +35° / 建立参考",font=("Microsoft YaHei UI",13,"bold")).pack(anchor="w")
        ttk.Label(outer,text="低电平触发（未触发=1，触发=0） · DI0：+35° · DI2：−29° · 原始反向抬头",wraplength=850).pack(anchor="w",pady=8)
        speed_row=ttk.Frame(outer);speed_row.pack(fill="x",pady=4)
        ttk.Label(speed_row,text="速度设定值（实际输出轴rpm=数值÷6）").pack(side="left")
        ttk.Combobox(speed_row,textvariable=self.dual["speed"],values=("0.2","0.4","0.6","0.8","1.0","1.2","1.5"),
                     state="normal",width=8).pack(side="left",padx=8)
        ttk.Label(speed_row,text="范围0.2～1.5；填0.5=0.0833rpm=0.5°/秒").pack(side="left")
        actions=ttk.Frame(outer);actions.pack(fill="x",pady=12)
        ttk.Button(actions,text="回到 +35° / 建立参考",command=self.dual_raised_home).pack(side="left",padx=(0,8))
        ttk.Button(actions,text="移动到水平 0°",command=self.dual_zero).pack(side="left",padx=8)
        ttk.Button(actions,text="读取限位状态",command=self.read_inputs).pack(side="left",padx=8)
        ttk.Button(actions,text="清除电机1参考",command=self.clear_reference).pack(side="left",padx=8)
        ttk.Button(actions,text="清除执行故障",command=self.clear_execution_fault).pack(side="left",padx=8)
        tk.Button(actions,text="停止",command=app._stop,bg="#c62828",fg="white").pack(side="right")
        ttk.Label(outer,textvariable=self.live,wraplength=850,font=("Microsoft YaHei UI",10,"bold")).pack(anchor="w",pady=8)
        ttk.Label(outer,textvariable=self.input_text,wraplength=850).pack(anchor="w",pady=8)
        ttk.Label(outer,textvariable=self.config_text,wraplength=850).pack(anchor="w",pady=8)
        ttk.Checkbutton(outer,text="刷新位置与限位状态",variable=self.refresh).pack(anchor="w",pady=8)
        ttk.Label(outer,text="无参考时：从接近水平处寻找DI0，停稳建立+35°参考。已有参考时：按现有坐标回到+35°，停止保留参考。\n上电不自动运动；板子掉电或复位后需重新建立参考。",wraplength=850).pack(anchor="w",pady=8)

    def select(self, axis):
        if self.app.auto_motor is None and not self.app.pending:
            self.app.motor_id.set(axis)

    def show_dual(self):
        self.select(1)
        self.refresh.set(True);self.refresh_at=0

    def read_inputs(self):
        if not self.app.port:return
        if self.after_stop is not None or self.jobs:return
        self.jobs.extend([(build_request(1,CMD_READ_DUAL_LIMIT),"读取限位配置"),
                          (build_request(1,CMD_READ_FEEDBACK),"读取 DI0、DI2 原始电平与触发状态")])
        self.pump()

    def values(self, axis):
        v={k:x.get() for k,x in self.models[axis].items()}
        return v

    def release_idle_owner(self,axis=1):
        a=self.app
        snapshot=getattr(self,"last_feedback",None)
        if snapshot and axis==1:
            sampled,rpm,flags,phase,fault=snapshot
            now=time.monotonic()
            idle=(now-sampled<=2 and sampled-getattr(a,"last_motion_sent_at",float('-inf'))>=0.3 and
                  rpm==0 and flags&1 and not flags&16 and phase in (0,7) and not fault)
            no_work=(not a.home_running and not a.auto_renew and not a.queued_motion and
                     getattr(a,"queued_auto",None) is None and getattr(a,"deferred_send",None) is None and
                     self.after_stop is None and getattr(self,"raised_request",None) is None and
                     not getattr(getattr(a,"business_panel",None),"phase",None) and
                     not getattr(a,"trajectory_running",False))
            if idle and no_work and not (a.pending and not getattr(a,"pending_background",False)):
                a._release_stopped_axis(axis)

    def ready(self,axis):
        a=self.app
        self.release_idle_owner(axis)
        # Post-homing readbacks must not block the next operator action.
        if self.jobs and all(frame[4] in (CMD_READ_GUARD,CMD_READ_DUAL_LIMIT,CMD_READ_FEEDBACK) for frame,_ in self.jobs):
            self.jobs.clear()
        if getattr(a,"trajectory_running",False) or getattr(getattr(a,"trajectory_panel",None),"quick_phase",None):
            messagebox.showwarning("轨迹运行中","请先停止两轴轨迹并确认停稳，再改配置或执行其他运动。",parent=self.window)
            return False
        if a.port is None or (a.pending and not getattr(a,"pending_background",False)) or getattr(a,"deferred_send",None) is not None or a.auto_motor is not None or getattr(a,"active_motor",None) is not None or a.diagnostic_active or a.queued_motion or a.stop_waiting or self.jobs or self.after_stop is not None or getattr(self,"raised_request",None) is not None or a.monitor_until>time.monotonic():
            reasons=[]
            for blocked,label in ((a.port is None,"未连接"),(a.pending and not getattr(a,"pending_background",False),"命令等待应答"),
                (a.auto_motor is not None or getattr(a,"active_motor",None) is not None,"运动占用"),
                (a.stop_waiting,"等待停止确认"),(a.diagnostic_active,"状态查询中"),
                (bool(self.jobs) or self.after_stop is not None or getattr(self,"raised_request",None) is not None,"流程待执行"),
                (a.monitor_until>time.monotonic(),"运动监测中")):
                if blocked:reasons.append(label)
            messagebox.showwarning("请先停止/等待","当前阻塞原因："+"、".join(reasons or ["有待执行命令"]),parent=self.window)
            return False
        a.motor_id.set(axis)
        return True

    def pump(self):
        if self.jobs and self.app.port and not self.app.pending:
            frame,text=self.jobs.popleft()
            readonly=frame[4] in (CMD_READ_GUARD,CMD_READ_DUAL_LIMIT,CMD_READ_FEEDBACK)
            if not self.app._send(frame,frame[4],text,background=readonly): self.jobs.clear()
            elif frame[4] in (CMD_HOME,CMD_HOME_DUAL_LIMIT):
                self.app.auto_motor=frame[3];self.app._lock_auto_axis(True)

    def dual_zero(self):
        self.models[1]["target"].set("0")
        try:
            self.goto(1,speed_rpm=self.selected_speed())
        except (ValueError,TypeError) as e:self.error(e)

    def selected_speed(self):
        speed=float(self.dual["speed"].get())
        if not math.isfinite(speed) or not 0.2<=speed<=1.5:raise ValueError("速度设定值须在0.2～1.5内，实际rpm=数值/6")
        return speed/6

    def clear_reference(self):
        if not self.ready(1):return
        if not messagebox.askyesno("清除电机1参考","确认清除电机1角度参考及旧轨迹？\n先停止，不进行定位或限位搜索。清除后需重新寻找DI0建立参考，才能按角度运动。",parent=self.window):return
        self.app._stop()
        self.after_stop=(1,"clear_reference",[
            (build_dual_limits(horizontal_deg=-29,raised_deg=35,positive_is_up=False,active_high=False),"显式清除电机1参考；不搜索"),
            (build_request(1,CMD_READ_DUAL_LIMIT),"读回双限位配置"),
            (build_request(1,CMD_READ_FEEDBACK),"确认电机1参考已清除")])

    def clear_execution_fault(self):
        if not self.ready(1):return
        if not messagebox.askyesno("确认执行故障原因已处理","确认已检查换向异常原因？\n先停止，再清除执行故障；保留角度参考，不自动启动。",parent=self.window):return
        self.app._stop()
        self.after_stop=(1,"clear_execution",[
            (build_request(1,CMD_CONFIG_GUARD,parameter=4),"明确确认定位执行故障；保留坐标"),
            (build_request(1,CMD_READ_FEEDBACK),"读回执行故障与参考")])

    def dual_raised_home(self):
        try:
            config=build_dual_limits(horizontal_deg=-29,raised_deg=35,
                positive_is_up=False,active_high=False)
            speed=self.selected_speed()
            home=build_dual_home(-1,speed_deg_s=speed*6,max_travel_deg=45,timeout_s=120,raised_only=True)
            if not self.ready(1):return
            if not messagebox.askyesno("回到 +35°",f"本次速度：{speed:g}输出轴rpm（{speed*6:g}°/秒）。\n已有有效参考：按现有坐标定位，停止保留参考。\n没有参考：从接近水平、两限位未触发处寻找DI0建立参考。确认执行？",parent=self.window):return
            self.start_error=None
            self.app._stop()
            self.after_stop=(1,"raised_home",(speed,[(config,"应用实测双限位与反向抬头配置"),
                (build_request(1,CMD_READ_DUAL_LIMIT),"读回实测双限位配置"),
                (home,f"以{speed:g}输出轴rpm反向抬头寻找 DI0，停稳标定+35°")]))
        except (ValueError,TypeError) as e:self.error(e)

    def goto(self,axis,*,speed_rpm=None):
        try:
            target=float(self.values(axis)["target"])
            if not math.isfinite(target): raise ValueError("目标角度无效")
            if not self.ready(axis): return
            if not messagebox.askyesno("坐标定位",f"确认 Motor {axis} 移动到软件坐标 {target:g}°？",parent=self.window): return
            self.goto_speed=speed_rpm
            self.app._stop()
            self.after_stop=(axis,"goto",target)
        except (ValueError,TypeError) as e: self.error(e)

    def stopped(self):
        if self.after_stop is None: return
        axis,kind,value=self.after_stop;self.after_stop=None
        if kind=="home":
            self.app.auto_motor=axis;self.app._lock_auto_axis(True)
            self.app._send(value,value[4],"启动STM32回零状态机")
        elif kind=="raised_home":
            self.raised_request=value
            if not self.app._send(build_request(1,CMD_READ_FEEDBACK),CMD_READ_FEEDBACK,"检查现有角度参考；不清除坐标"):
                self.raised_request=None
        elif kind in ("clear_reference","clear_execution"):
            self.jobs.extend(value);self.pump()
        else:
            self.goto_target=(axis,value)
            self.app._send(build_request(axis,CMD_READ_FEEDBACK),CMD_READ_FEEDBACK,"定位前读取可信坐标")

    def cancel(self):
        self.after_stop=None;self.goto_target=None;self.raised_request=None;self.jobs.clear()

    def service(self):
        a=self.app
        if getattr(getattr(a,"business_panel",None),"phase",None):return
        trajectory=getattr(a,"trajectory_panel",None)
        if trajectory is not None and (getattr(a,"trajectory_running",False) or trajectory.jobs or trajectory.quick_phase):return
        if a.pending or getattr(a,"deferred_send",None) is not None or a.diagnostic_active or a.queued_motion or a.queued_auto is not None or a.stop_waiting or a.parameter_check_queue or a.profile_read_queue or self.after_stop is not None: return
        if self.jobs:
            self.pump();return
        if a.port and (a.home_running or self.refresh.get()) and time.monotonic()>=self.refresh_at:
            axis=a.auto_motor if a.home_running else 1
            moving=bool(a.home_running or a.auto_motor is not None or getattr(a,"active_motor",None) is not None)
            self.refresh_at=time.monotonic()+(0.25 if moving else 1.0)
            a._send(build_request(axis,CMD_READ_FEEDBACK),CMD_READ_FEEDBACK,"刷新位置与限位反馈",background=True)

    def handle(self,packet):
        if packet.command not in (CMD_READ_FEEDBACK,CMD_READ_GUARD,CMD_CONFIG_GUARD,CMD_SET_ZERO,CMD_HOME,
                CMD_CONFIG_DUAL_LIMIT,CMD_READ_DUAL_LIMIT,CMD_HOME_DUAL_LIMIT): return False
        a=self.app
        if not packet.ok:
            self.cancel();a.auto_renew=False
            if packet.command in (CMD_HOME,CMD_HOME_DUAL_LIMIT):
                a.auto_motor=None;a.home_running=False;a.monitor_until=0;a._lock_auto_axis(False)
            text=f"Motor {packet.device_id} 调试命令失败：{packet.status_text}"
            if packet.command==CMD_HOME_DUAL_LIMIT and packet.status==0x08:
                text="复位未启动：板端判定 DI0 或 DI2 已触发。正在读取实际电平。"
                self.start_error=text
                self.jobs.append((build_request(1,CMD_READ_FEEDBACK),"启动被限位拦截，读取 DI0、DI2"))
                a.root.after(80,self.pump)
            self.live.set(text);a.result_text.set(text)
            return True
        if packet.command==CMD_READ_DUAL_LIMIT:
            if len(packet.data)!=16:self.cancel();self.live.set("双限位回包长度错误");return True
            enabled,horizontal,raised,high,sign,valid,lo,hi,hit,pending=struct.unpack(">BBBBbBiiBB",packet.data)
            for key,value in (() if not enabled else (("horizontal",f"DI{horizontal}"),("raised",f"DI{raised}"),
                ("polarity","高电平触发" if high else "低电平触发"),("sign","正向抬头" if sign==1 else "负向抬头"),
                ("horizontal_deg",str(lo/1000) if valid&1 else ""),("raised_deg",str(hi/1000) if valid&2 else ""))):
                self.dual[key].set(value)
            self.actual_dual_inputs=(horizontal,raised,255,high) if enabled else None
            if enabled:self.actual_inputs[1]=(horizontal,raised,255,high)
            self.config_text.set(f"双限位{'已启用' if enabled else '未启用'}；最近触发：{ {0:'无',1:'水平侧',2:'抬头极限'}.get(hit,'未知')}；{'等待停稳及有效角度' if pending else '无待处理标定'}；实测角度有效位={valid}")
        elif packet.command in (CMD_HOME,CMD_HOME_DUAL_LIMIT):
            a.home_running=True;a.auto_renew=True;a.auto_renew_at=time.monotonic()+1
            a.monitor_until=float("inf");self.refresh_at=0
            self.live.set("回零状态机已启动；可随时按停止取消")
        elif packet.command==CMD_READ_GUARD:
            if len(packet.data)!=17: self.cancel();self.live.set("配置回包长度错误");return True
            neg,pos,home,high,direction,confirm,hw,soft,lo,hi,end=struct.unpack(">BBBBbBBBiiB",packet.data)
            self.actual_inputs[packet.device_id]=(neg,pos,home,high)
            v=self.models[packet.device_id]
            for key,ch in (("negative",neg),("positive",pos),("home",home)):
                v[key].set("未使用" if ch==255 else f"DI{ch}")
            for key,value in (("polarity","高电平触发" if high else "低电平触发"),
                ("direction",{-1:"负向",0:"未确认",1:"正向"}.get(direction,"未确认")),
                ("confirmed",bool(confirm)),("hardware",bool(hw)),("soft",bool(soft)),
                ("minimum",str(lo/1000)),("maximum",str(hi/1000)),
                ("location","端点" if end else "中间（已确认安全接近侧）" if confirm else "未确认")):
                v[key].set(value)
            self.config_text.set(f"Motor {packet.device_id} 实际策略：{'高' if high else '低'}电平触发；硬限位={'开' if hw else '关'}，软范围={'开' if soft else '关'} [{lo/1000:g}, {hi/1000:g}]°；输入触发不等于回零完成；掉电不保留")
        elif packet.command==CMD_READ_FEEDBACK:
            if len(packet.data)!=34: self.cancel();a.auto_renew=False;self.live.set("位置回包长度错误，状态未知");return True
            actual,cmd,error,mode,ppr,bits,angle,flags,raw,active,phase,fault,origin,rpm=struct.unpack(">iiiHHHiHBBBBih",packet.data)
            if packet.device_id==1:self.last_feedback=(time.monotonic(),rpm,flags,phase,fault)
            angle_text=f"{angle/1000:+.3f}°" if flags&1 else "未建立/已失效"
            self.live.set(f"Motor {packet.device_id} 实际={actual}，指令={cmd}，偏差={error} count\n"
                f"P3-32={mode}，PA11={ppr}，PA95={bits}（仅读）  {'水平姿态角' if packet.device_id==1 else '输出软件坐标'}：{angle_text}\n"
                f"回零：{PHASES[phase] if phase<len(PHASES) else phase}；原因：{FAULTS[fault] if fault<len(FAULTS) else fault}；转速={rpm} rpm；停机待确认={bool(flags&16)}")
            dual=getattr(self,"actual_dual_inputs",None) if packet.device_id==1 else None
            text=format_inputs(raw,active,dual or self.actual_inputs.get(packet.device_id))
            if dual:text=text.replace("负限位","水平侧").replace("正限位","抬头极限")
            self.input_text.set(text)
            if packet.device_id==1 and getattr(self,"start_error",None):
                if dual:
                    triggered=[f"DI{ch}" for ch in dual[:2] if active&(1<<ch)]
                    state="、".join(triggered) if triggered else "当前均未触发"
                    self.live.set(f"复位未启动；板端触发状态：{state}。\nDI0原始电平={(raw>>0)&1}，DI2原始电平={(raw>>2)&1}；当前按{'高' if dual[3] else '低'}电平触发判定。\n请在两个限位均未触发时读取，再分别触发DI0、DI2观察变化。")
                else:self.live.set(self.start_error+"\n请点击“读取限位状态”核对映射与电平。")
            self.records.append((time.time(),packet.device_id,actual,cmd,error,mode,ppr,bits,angle/1000 if flags&1 else "",flags,raw,active,phase,fault,rpm))
            if a.home_running and phase==7 and (rpm or flags&16):return True
            if a.home_running and phase in (0,7,8):
                a.auto_renew=False;a.home_running=False
                if phase==7 and flags&1:
                    a._release_stopped_axis(packet.device_id)
                    a.result_text.set(f"参考建立完成，当前实际角度 {angle/1000:+.3f}°；未自动移动到0°")
                    self.jobs.append((build_request(packet.device_id,CMD_READ_GUARD),"回零后读取实际保护配置"))
                    if packet.device_id==1:self.jobs.append((build_request(1,CMD_READ_DUAL_LIMIT),"读取最近触发的限位"))
                else:
                    a._stop()
            if packet.device_id==1 and getattr(self,"raised_request",None) is not None:
                speed,initial_jobs=self.raised_request;self.raised_request=None
                if rpm or flags&16 or phase in range(1,7):
                    self.error(ValueError("尚未确认停稳，已取消；未清除现有参考"));return True
                if flags&1:
                    if fault:
                        self.error(ValueError("现有参考保留，但存在保护故障，请先处理故障"));return True
                    if abs(angle-35000)<=50:
                        a.result_text.set("已在+35°附近，保留现有参考，无需运动")
                    else:
                        self.goto_speed=speed
                        self.goto_target=(1,35.0)
                else:
                    self.jobs.extend(initial_jobs)
            if self.goto_target is not None:
                axis,target=self.goto_target;self.goto_target=None
                if not flags&1: self.error(ValueError("未建立可信零点，不能按坐标定位"))
                else:
                    delta=round(target-angle/1000,3)
                    if delta:
                        speed=getattr(self,"goto_speed",None) or float(a.speed_rpm.get())
                        self.goto_speed=None
                        frame=build_move_output_relative(axis,speed,delta)
                        a.next_monitor_seconds=min(3600,max(5,abs(delta)/(6*speed)+8))
                        a.motion_seen_running=False
                        a._send(frame,CMD_MOVE_OUTPUT_RELATIVE,f"走到软件坐标 {target:g}°（相对 {delta:+g}°）")
                    else: a.result_text.set("已在目标坐标附近，无需运动")
        if self.jobs: a.root.after(80,self.pump)
        return True

    def error(self,exc): messagebox.showerror("调试配置",str(exc),parent=self.window)

    def close(self):
        self.refresh.set(False)
        if self.app.home_running or self.after_stop is not None:self.app._stop()
        self.window.withdraw()
