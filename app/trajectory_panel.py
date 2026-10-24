"""Buffered two-axis trajectory UI, sharing the main window's serial transport."""
import csv
import struct
import time
from collections import deque
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

from protocol import (CMD_TRAJECTORY_BEGIN, CMD_TRAJECTORY_POINT, CMD_TRAJECTORY_SEAL,
    CMD_TRAJECTORY_START, CMD_TRAJECTORY_STATUS, CMD_TRAJECTORY_STOP, CMD_TRAJECTORY_KEEPALIVE,
    CMD_READ_GUARD,CMD_READ_FEEDBACK,CMD_READ_ALARM,CMD_READ_AXES,CMD_TRAJECTORY_CLEAR,
    build_request, build_trajectory_begin, build_trajectory_point)
from trajectory import (TrajectoryPoint, validate_points, joint_ellipse, decode_status, quick_joint_test,
                        decode_axes, return_to_start_points, business_trajectory, business_loop)


class TrajectoryPanel:
    def __init__(self, app):
        self.app=app;self.jobs=deque();self.points=();self.session=0;self.sealed=False
        self.renew=False;self.blocked=False;self.next_renew=0;self.next_status=0
        self.revision=0;self.upload_revision=-1
        self.saved_points=();self.saved_values=None;self.next_positions=0;self.last_positions_at=0
        self.adaptive=tk.BooleanVar(value=True)
        self.quick_phase=None;self.quick_generation=0;self.quick_configs={};self.quick_feedback={};self.quick_alarms={}
        self.window=tk.Toplevel(app.root);self.window.title("两轴业务轨迹 v3.5")
        self.window.geometry("1050x780");self.window.protocol("WM_DELETE_WINDOW",self.close)
        outer=ttk.Frame(self.window,padding=12);outer.pack(fill="both",expand=True)
        ttk.Label(outer,text="先完成双限位回零和业务准备；电机1双限位范围−29°～+35°，以板端读回范围为准。电机2必须有参考。普通轨迹首点须与当前位置一致。\n"
            "配套固件115200／8N2；共同减速模式允许延长运行时间，不保证固定圈时或空间圆精度。",wraplength=1000).pack(anchor="w")
        self.status=tk.StringVar(value="尚未读取轨迹状态；不会自动启动")
        ttk.Label(outer,textvariable=self.status,wraplength=1000,font=("Microsoft YaHei UI",10,"bold")).pack(anchor="w",pady=10)
        self.positions_text=tk.StringVar(value="两轴实际位置尚未读取")
        ttk.Label(outer,textvariable=self.positions_text,wraplength=1000).pack(anchor="w")
        recovery=ttk.Frame(outer);recovery.pack(fill="x",pady=4)
        ttk.Checkbutton(recovery,text="两轴共同减速跟随（需要新固件，运行时间可延长）",
                        variable=self.adaptive,command=self.invalidate).pack(side="left")
        ttk.Button(recovery,text="准备回首点（不启动）",command=self.prepare_return).pack(side="left",padx=5)
        ttk.Button(recovery,text="恢复原轨迹草稿",command=self.restore_original).pack(side="left")
        ttk.Button(recovery,text="确认清除轨迹故障（不运动）",command=self.clear_fault).pack(side="left",padx=5)
        quick=ttk.Frame(outer);quick.pack(fill="x",pady=(0,8))
        ttk.Button(quick,text="一键双轴小幅调试",command=self.quick_test).pack(side="left")
        ttk.Label(quick,text="  自动读状态→确认→上传→启动；两轴同相±1～2°，20秒往返一圈，回到起点附近。",wraplength=800).pack(side="left")
        fields=ttk.LabelFrame(outer,text="离线示例／约束（角度为输出轴；电机1水平0°）",padding=8);fields.pack(fill="x")
        defaults={"center1":"0","center2":"0","amplitude1":"5","amplitude2":"5","period":"20","cycles":"1",
                  "cycle_ms":"100","speed":"1.5","accel":"3","error":"0.75","tolerance":"0.3"}
        labels=("电机1中心 °","电机2中心 °","电机1幅度 °","电机2幅度 °","周期 秒","圈数",
                "控制周期 ms（50～500）","最大速度 °/秒","最大加速度 °/秒²","最大跟随误差 °","终点容差 °")
        self.values={key:tk.StringVar(value=value) for key,value in defaults.items()}
        for n,(key,label) in enumerate(zip(defaults,labels)):
            row=n//3;col=(n%3)*2
            ttk.Label(fields,text=label).grid(row=row,column=col,sticky="w",padx=6,pady=4)
            ttk.Entry(fields,textvariable=self.values[key],width=9).grid(row=row,column=col+1,sticky="w")
            self.values[key].trace_add("write",lambda *_args:self.invalidate())
        self.advanced_fields=fields;fields.pack_forget()
        self.show_advanced=tk.BooleanVar(value=False)
        ttk.Checkbutton(outer,text="高级轨迹参数（日常使用默认预设即可）",variable=self.show_advanced,
                        command=lambda:fields.pack(fill="x",before=buttons) if self.show_advanced.get() else fields.pack_forget()).pack(anchor="w")
        buttons=ttk.Frame(outer);buttons.pack(fill="x",pady=10)
        for text,action in (("生成正弦示例（不是标定空间圆）",self.generate),("导入CSV",self.load),
                            ("上传并校验（不启动）",self.upload),("启动已校验轨迹",self.start),
                            ("读取状态",self.read)):
            ttk.Button(buttons,text=text,command=action).pack(side="left",padx=3)
        tk.Button(buttons,text="停止两轴",command=self.stop,bg="#c62828",fg="white").pack(side="right")
        self.preview=tk.Text(outer,height=17,wrap="none",state="disabled");self.preview.pack(fill="both",expand=True)
        ttk.Label(outer,text="CSV列名：time_s,axis1_deg,axis2_deg；2～128点，首点t=0，间隔≥50ms，总时长1～600秒。\n"
            "轨迹点须距软边界至少1°，运行还会检查通信延迟与制动余量；不能贴边运行。开始／结束过渡与停稳时间不计入设定周期。",
            wraplength=1000).pack(anchor="w",pady=8)

    def invalidate(self):
        self.sealed=False;self.revision+=1

    def error(self, exc): messagebox.showerror("两轴轨迹",str(exc),parent=self.window)

    def ready(self):
        a=self.app
        timing=getattr(a,'_active_timing_panel',lambda:None)()
        if timing and timing.phase:
            messagebox.showwarning('测试进行中','先停止当前两轴测试，再操作连续轨迹。',parent=self.window)
            return False
        busy=(getattr(a,"trajectory_running",False) or getattr(a,"home_running",False) or self.quick_phase or self.jobs or a.auto_motor is not None or
              a.active_motor is not None or a.stop_waiting or a.diagnostic_active or a.queued_motion or
              a.queued_auto is not None or a.parameter_check_queue or a.profile_read_queue or
              getattr(a,"deferred_send",None) is not None or (a.pending and not getattr(a,"pending_background",False)))
        if not a.port or busy:
            messagebox.showwarning("请先连接／停稳","先连接串口并停止其他动作，等待现有操作完成。",parent=self.window)
            return False
        return True

    def show_points(self):
        text=f"共{len(self.points)}点，计划运动时间{self.points[-1].time_s:g}秒；"
        text+=f"首点：电机1={self.points[0].axis1_deg:g}°，电机2={self.points[0].axis2_deg:g}°"
        text+=("（一键调试：当前姿态，无需另行预定位）\n" if self.quick_phase else "（普通轨迹需人工预定位）\n")
        text+="time_s,axis1_deg,axis2_deg\n"
        text+="\n".join(f"{p.time_s:.3f},{p.axis1_deg:.3f},{p.axis2_deg:.3f}" for p in self.points)
        self.preview.configure(state="normal");self.preview.delete("1.0","end");self.preview.insert("end",text);self.preview.configure(state="disabled")
        self.sealed=False

    def generate(self):
        try:
            if self.quick_phase or self.jobs or self.app.trajectory_running: raise ValueError("运行／准备中不能更换轨迹")
            if not self.show_advanced.get():
                business=getattr(self.app,"business_panel",None)
                if (not business or time.monotonic()-business.received_at>2 or business.horizontal_samples<2):
                    raise ValueError("先在主窗口完成参考和水平0°准备，等待实际位置刷新；离线示例可在高级模式生成")
                self.points=business_loop(tuple(p.angle_deg for p in business.positions))
                self.show_points();return
            v={key:value.get() for key,value in self.values.items()}
            self.points=joint_ellipse(center1=float(v["center1"]),center2=float(v["center2"]),
                amplitude1=float(v["amplitude1"]),amplitude2=float(v["amplitude2"]),
                period_s=float(v["period"]),cycles=int(v["cycles"]))
            self.show_points()
        except (ValueError,TypeError) as exc:self.error(exc)

    def load(self):
        if self.quick_phase or self.jobs or self.app.trajectory_running:return
        path=filedialog.askopenfilename(parent=self.window,filetypes=[("CSV","*.csv")])
        if not path:return
        try:
            with open(path,encoding="utf-8-sig",newline="") as stream:
                rows=[]
                for row in csv.DictReader(stream):
                    rows.append(TrajectoryPoint(float(row["time_s"]),float(row["axis1_deg"]),float(row["axis2_deg"])))
                    if len(rows)>128: raise ValueError("CSV超过128点")
            self.points=validate_points(rows);self.show_points()
        except (OSError,ValueError,KeyError,TypeError) as exc:self.error(exc)

    def upload(self):
        try:
            if not self.ready():return
            points=validate_points(self.points);v={key:value.get() for key,value in self.values.items()}
            daily=not self.show_advanced.get()
            if daily:
                # Collapsing the editor hides controls; it must not silently
                # replace an explicitly selected acceleration with the default.
                points=business_trajectory(points,max_speed=float(v["speed"]),max_accel=float(v["accel"]))
                self.points=points;self.show_points()
            session=(time.monotonic_ns()&65535) or 1
            header=build_trajectory_begin(session,len(points),cycle_ms=int(v["cycle_ms"]),
                max_speed=float(v["speed"]),max_accel=float(v["accel"]),max_error=float(v["error"]),tolerance=float(v["tolerance"]),
                adaptive=self.adaptive.get())
            frames=[build_trajectory_point(session,n,round(p.time_s*1000),p.axis1_deg,p.axis2_deg) for n,p in enumerate(points)]
            self.session=session;self.sealed=False;self.blocked=False;self.renew=False
            self.upload_revision=self.revision
            self.jobs.append((header,"建立轨迹上传会话（不运动）"))
            self.jobs.extend((frame,f"上传轨迹点 {n+1}/{len(points)}") for n,frame in enumerate(frames))
            self.jobs.append((build_request(1,CMD_TRAJECTORY_SEAL,parameter=session),"整条轨迹范围／速度／加速度校验"))
            self.service()
        except (ValueError,TypeError) as exc:self.error(exc)

    def cancel_quick(self):
        self.quick_phase=None;self.quick_generation+=1
        self.quick_configs.clear();self.quick_feedback.clear();self.quick_alarms.clear()

    def quick_test(self):
        if not self.ready():return
        self.cancel_quick();self.quick_phase="checking"
        self.blocked=False;self.sealed=False;self.renew=False
        self.status.set("一键调试：正在读取两轴参考、范围、速度和报警；尚未发送运动命令")
        self.jobs.append((build_request(1,CMD_TRAJECTORY_STATUS),"一键调试：探测轨迹固件／现有任务"))
        for axis in (1,2):
            self.jobs.extend(((build_request(axis,CMD_READ_GUARD),f"一键调试：读取电机{axis}实际保护"),
                (build_request(axis,CMD_READ_FEEDBACK),f"一键调试：读取电机{axis}当前姿态"),
                (build_request(axis,CMD_READ_ALARM),f"一键调试：读取电机{axis}报警")))
        self.service()

    def _finish_quick_check(self):
        try:
            centers=[];bounds=[];hardware=[]
            for axis in (1,2):
                config=self.quick_configs[axis];feedback=self.quick_feedback[axis]
                neg,pos,home,high,direction,confirm,hw,soft,lo,hi,end=config
                actual,cmd,error,mode,ppr,bits,angle,flags,raw,active,phase,fault,origin,rpm=feedback
                if not flags&1 or mode!=3 or not ppr:
                    raise ValueError(f"电机{axis}没有有效参考；先回零或在已确认姿态建立参考，一键功能不会自动置零")
                if axis==1 and (not soft or not flags&2):
                    raise ValueError(f"电机{axis}软范围未开启，请确认机械安全范围并启用；不会自动扩大范围或绕过保护")
                if rpm or flags&16 or phase in range(1,7) or fault:
                    raise ValueError(f"电机{axis}未停稳、正在回零或保护未解除，请先检查")
                if self.quick_alarms[axis]:raise ValueError(f"电机{axis}存在报警{self.quick_alarms[axis]}，不会自动复位")
                channels=(neg,pos,home) if end else (neg,pos)
                if hw and any(ch<6 and active&(1<<ch) for ch in channels):
                    raise ValueError(f"电机{axis}有效端点限位已触发，禁止一键运动")
                centers.append(angle/1000);bounds.append((lo/1000,hi/1000) if soft else None);hardware.append(bool(hw))
            points,amplitude=quick_joint_test(centers,bounds)
            generation=self.quick_generation
            accepted=messagebox.askyesno("一键双轴小幅运动确认",
                f"电机1：{centers[0]:+.3f}° 附近 ±{amplitude:g}°\n"
                f"电机2：{centers[1]:+.3f}° 附近 ±{amplitude:g}°\n"
                "两轴同相往返一圈，约20秒，结束回到当前起点附近；不是圆轨迹。\n"
                f"硬输入策略：电机1{'开' if hardware[0] else '关'}，电机2{'开' if hardware[1] else '关'}；电机1固定范围，电机2按实际策略。\n"
                "确认机构及线缆在以上范围内安全、人员避让、硬件急停就绪？\n确认后将自动上传并启动；不需要填写参数或另行定位。",parent=self.window)
            if self.quick_phase!="checking" or self.quick_generation!=generation:return
            if not accepted:
                self.cancel_quick();self.status.set("已取消一键调试，未发送运动命令");return
            self.quick_phase="uploading"
            preset={"center1":centers[0],"center2":centers[1],"amplitude1":amplitude,"amplitude2":amplitude,
                "period":20,"cycles":1,"cycle_ms":250,"speed":1.5,"accel":3,"error":0.75,"tolerance":0.3}
            for key,value in preset.items():self.values[key].set(str(value))
            self.points=points;self.show_points()
            self._queue_upload(points,cycle_ms=250,max_speed=1.5,max_accel=3,max_error=0.75,tolerance=0.3,
                               adaptive=getattr(self,"adaptive",None).get() if getattr(self,"adaptive",None) else False)
        except (ValueError,KeyError,TypeError) as exc:
            self.jobs.clear();self.cancel_quick();self.status.set(f"一键调试未启动：{exc}");self.error(exc)

    def _queue_upload(self,points,**limits):
        session=(time.monotonic_ns()&65535) or 1
        header=build_trajectory_begin(session,len(points),**limits)
        frames=[build_trajectory_point(session,n,round(p.time_s*1000),p.axis1_deg,p.axis2_deg) for n,p in enumerate(points)]
        self.session=session;self.sealed=False;self.blocked=False;self.renew=False;self.upload_revision=self.revision
        self.jobs.append((header,"建立小幅调试轨迹会话（尚未运动）"))
        self.jobs.extend((frame,f"上传小幅调试点 {n+1}/{len(points)}") for n,frame in enumerate(frames))
        self.jobs.append((build_request(1,CMD_TRAJECTORY_SEAL,parameter=session),"校验轨迹；返回轨迹须另行点击启动"))
        self.service()

    def prepare_return(self):
        try:
            if not self.ready():return
            original=getattr(self,"saved_points",()) or validate_points(self.points)
            self.cancel_quick();self.quick_phase="return_check";self.return_original=original
            self.return_revision=self.revision;self.return_positions=None
            self.sealed=False;self.blocked=False
            self.status.set("检查两轴停稳位置与软范围；不会自动返回或重新置零")
            self.jobs.append((build_request(1,CMD_TRAJECTORY_STATUS),"恢复前确认组任务已停止"))
            for axis in (1,2):
                self.jobs.append((build_request(axis,CMD_READ_GUARD),f"读取电机{axis}返回范围"))
            self.jobs.append((build_request(1,CMD_READ_AXES),"读取两轴真实停稳位置"))
            self.service()
        except (ValueError,TypeError) as exc:self.error(exc)

    def clear_fault(self):
        if not self.ready():return
        generation=self.quick_generation
        if not messagebox.askyesno("确认取消故障任务",
            "确认已检查故障原因、两轴已停稳？\n只清除轨迹故障和旧任务，不复位驱动器、不置零、不恢复运动。\n"
            "存在驱动报警、参考无效或限位未解除时，下位机会拒绝。",parent=self.window):return
        if generation!=self.quick_generation or not self.ready():return
        self.blocked=False;self.sealed=False;self.renew=False;self.quick_phase="clear_check"
        self.jobs.append((build_request(1,CMD_TRAJECTORY_STATUS),"读取需确认清除的故障会话"))
        self.service()

    def _finish_return_check(self):
        try:
            positions=self.return_positions
            if any(not p.usable or p.source!=0 or p.motor_rpm or p.alarm_code or p.guard_fault or
                   p.flags&16 or (i==0 and not p.flags&2) for i,p in enumerate(positions)):
                raise ValueError("未停稳、参考／软范围无效或仍有故障，不能准备返回")
            bounds=[]
            for axis in (1,2):
                cfg=self.quick_configs[axis]
                if axis==1 and not cfg[7]:raise ValueError("电机1固定范围未启用")
                bounds.append((cfg[8]/1000,cfg[9]/1000) if cfg[7] else None)
            original=self.return_original;target=(original[0].axis1_deg,original[0].axis2_deg)
            current=tuple(p.angle_deg for p in positions)
            points=return_to_start_points(current,target,bounds)
            generation=self.quick_generation
            accepted=messagebox.askyesno("准备回首点（不会自动启动）",
                f"实际位置：{current[0]:+.3f}° / {current[1]:+.3f}°\n"
                f"原首点：{target[0]:+.3f}° / {target[1]:+.3f}°\n"
                "将生成缓慢的两轴关节角返回路径；它不是沿原圆倒退。\n"
                "请确认整个返回路径的机构、线缆和障碍物余量。\n"
                "确认后只上传，仍须另行点击启动；不清零、不扩大范围。",parent=self.window)
            if self.quick_phase!="return_check" or generation!=self.quick_generation:return
            if self.revision!=self.return_revision:raise ValueError("检查期间草稿已变化，请重新准备")
            if not accepted:
                self.cancel_quick();self.status.set("已取消返回，未发送运动命令");return
            if not getattr(self,"saved_points",()):
                self.saved_values={key:value.get() for key,value in self.values.items()}
                self.saved_adaptive=self.adaptive.get() if getattr(self,"adaptive",None) else False
            self.saved_points=original;self.points=points;self.quick_phase="return_uploading"
            for key,value in {"cycle_ms":100,"speed":1.5,"accel":3,"error":0.75,"tolerance":0.3}.items():
                self.values[key].set(str(value))
            if getattr(self,"adaptive",None):self.adaptive.set(True)
            self.show_points()
            self._queue_upload(points,cycle_ms=100,max_speed=1.5,max_accel=3,max_error=0.75,tolerance=0.3,adaptive=True)
        except (ValueError,TypeError,KeyError) as exc:
            self.jobs.clear();self.cancel_quick();self.error(exc)

    def restore_original(self):
        if not self.ready():return
        if not getattr(self,"saved_points",()):
            self.error(ValueError("没有保存的原轨迹草稿"));return
        self.points=self.saved_points;self.saved_points=();self.show_points();self.invalidate()
        if getattr(self,"saved_values",None):
            for key,value in self.saved_values.items():self.values[key].set(value)
            self.saved_values=None
            if getattr(self,"adaptive",None):self.adaptive.set(self.saved_adaptive)
        self.status.set("原轨迹已恢复为草稿；重新上传校验后才能启动，不沿用旧会话")

    def _send_start(self):
        self.app.trajectory_running=True;self.app._lock_auto_axis(True)
        self.blocked=False;self.next_status=0
        if not self.app._send(build_request(1,CMD_TRAJECTORY_START,parameter=self.session),CMD_TRAJECTORY_START,"启动两轴共同时间轨迹"):
            self.on_error()

    def start(self):
        if not self.ready():return
        if not self.sealed: self.error(ValueError("先上传并校验当前轨迹；改动参数后须重新上传"));return
        generation,revision,session=self.quick_generation,self.revision,self.session
        if not messagebox.askyesno("两轴共同启动","确认两轴已建立参考、开启软范围并停在预览首点？\n"
            "本模式为RS485低频跟随试验，不保证5秒空间圆精度；请低幅度、长周期起步，人员避让并保留硬件急停。",parent=self.window):return
        if (not self.sealed or generation!=self.quick_generation or revision!=self.revision or
                session!=self.session or not self.ready()):return
        self._send_start()

    def read(self):
        if self.quick_phase:return
        if self.app.port and not self.app.pending:
            self.blocked=False
            self.app._send(build_request(1,CMD_TRAJECTORY_STATUS),CMD_TRAJECTORY_STATUS,"读取轨迹缓存状态")

    def stop(self):
        self.cancel_quick()
        self.jobs.clear();self.renew=False;self.sealed=False;self.blocked=False
        a=self.app
        a.deferred_send=None;a.queued_motion=None;a.queued_auto=None;a.parameter_check_queue=None;a.profile_read_queue=None
        if getattr(a,"debug_panel",None):a.debug_panel.cancel()
        if not a.port:return
        a.trajectory_running=True;a._lock_auto_axis(True)
        self.next_status=0
        a._send(build_request(1,CMD_TRAJECTORY_STOP),CMD_TRAJECTORY_STOP,"请求停止两轴轨迹并取消缓冲",allow_override=True)

    def on_error(self):
        self.cancel_quick()
        self.jobs.clear();self.renew=False;self.sealed=False;self.blocked=True
        self.status.set("通信／命令异常：已取消待发操作和心跳。运动状态可能未知，请核查或使用硬件急停；不会自动重试。")

    def disconnected(self):
        self.cancel_quick()
        self.jobs.clear();self.renew=False;self.sealed=False;self.blocked=True
        self.app.trajectory_running=False
        self.status.set("串口已断开，参考／任务状态需重新读取；不自动恢复运动")
        if getattr(self,"positions_text",None):self.positions_text.set("通信断开，当前位置未知；旧角度不可用于恢复")

    def service(self):
        a=self.app
        if getattr(self,"positions_text",None) and time.monotonic()-self.last_positions_at>3:
            self.positions_text.set("位置反馈未更新超过3秒；当前位置未知，禁止按旧数值恢复")
        if not a.port or a.pending or self.blocked or getattr(a,"deferred_send",None) is not None:return
        if self.jobs:
            frame,text=self.jobs.popleft()
            if not a._send(frame,frame[4],text):self.on_error()
            return
        if a.trajectory_running:
            now=time.monotonic()
            if self.renew and now>=self.next_renew:
                self.next_renew=now+0.75
                a._send(build_request(1,CMD_TRAJECTORY_KEEPALIVE,parameter=self.session),CMD_TRAJECTORY_KEEPALIVE,"两轴轨迹心跳")
            elif getattr(self,"positions_text",None) and now>=self.next_positions:
                self.next_positions=now+getattr(self,'position_interval',1)
                a._send(build_request(1,CMD_READ_AXES),CMD_READ_AXES,"读取控制反馈及采样年龄",background=True)
            elif now>=self.next_status:
                self.next_status=now+getattr(self,'status_interval',.5)
                a._send(build_request(1,CMD_TRAJECTORY_STATUS),CMD_TRAJECTORY_STATUS,"刷新两轴轨迹缓存状态",background=True)
                return
            else:return
            return
        if (getattr(self,"positions_text",None) and not self.quick_phase and
            self.window.state()!="withdrawn" and time.monotonic()>=self.next_positions):
            self.next_positions=time.monotonic()+1
            a._send(build_request(1,CMD_READ_AXES),CMD_READ_AXES,"读取两轴实际位置及采样年龄",background=True)

    def handle(self,packet):
        if packet.command==CMD_READ_AXES:
            try:
                if not packet.ok:raise ValueError(f"位置读取失败：{packet.status_text}；需要配套v3.2固件")
                positions=decode_axes(packet.data);self.last_positions_at=time.monotonic()
                text=[]
                for axis,p in enumerate(positions,1):
                    text.append(f"电机{axis}："+(f"{p.angle_deg:+.3f}°，{p.motor_rpm}rpm，"
                        f"采样年龄{p.sample_age_ms}ms（{'控制缓存' if p.source else '驱动读取'}）"
                        if p.usable else "角度无效／反馈过期，不可用于恢复"))
                if getattr(self,"positions_text",None):self.positions_text.set("实际位置｜"+"；".join(text))
                if self.quick_phase=="return_check":
                    self.return_positions=positions;self._finish_return_check()
            except (ValueError,struct.error) as exc:
                if getattr(self,"positions_text",None):self.positions_text.set(str(exc))
                if self.quick_phase=="return_check":self.jobs.clear();self.cancel_quick();self.error(exc)
            return True
        if self.quick_phase=="return_check" and packet.command==CMD_READ_GUARD:
            try:
                if not packet.ok:raise ValueError(packet.status_text)
                self.quick_configs[packet.device_id]=struct.unpack(">BBBBbBBBiiB",packet.data)
            except (ValueError,struct.error) as exc:self.jobs.clear();self.cancel_quick();self.error(exc)
            return True
        if self.quick_phase=="checking" and packet.command in (CMD_READ_GUARD,CMD_READ_FEEDBACK,CMD_READ_ALARM):
            try:
                if not packet.ok:raise ValueError(f"电机{packet.device_id}状态读取失败：{packet.status_text}")
                if packet.command==CMD_READ_GUARD:
                    self.quick_configs[packet.device_id]=struct.unpack(">BBBBbBBBiiB",packet.data)
                elif packet.command==CMD_READ_FEEDBACK:
                    self.quick_feedback[packet.device_id]=struct.unpack(">iiiHHHiHBBBBih",packet.data)
                else:self.quick_alarms[packet.device_id]=struct.unpack(">H",packet.data)[0]
                if not self.jobs and len(self.quick_configs)==len(self.quick_feedback)==len(self.quick_alarms)==2:
                    self._finish_quick_check()
            except (ValueError,struct.error) as exc:
                self.on_error();self.error(exc)
            return True
        if not CMD_TRAJECTORY_BEGIN<=packet.command<=CMD_TRAJECTORY_CLEAR:return False
        try:
            result=decode_status(packet.data)
        except ValueError as exc:
            self.on_error()
            if packet.status==7:
                self.app.trajectory_running=False;self.app._lock_auto_axis(False)
                self.status.set("固件不支持该轨迹功能；清故障需配套v3.4固件，未自动重试")
            else:self.error(exc)
            return True
        self.app.trajectory_running=result.active
        self.app._lock_auto_axis(result.active)
        if self.quick_phase=="clear_check" and packet.command==CMD_TRAJECTORY_STATUS:
            if not packet.ok or result.state not in (6,7,8):
                self.cancel_quick();self.status.set("不是已停稳的终态任务；请先停止两轴并读取实际位置，未清故障")
            else:
                self.quick_phase="clearing"
                self.jobs.append((build_request(1,CMD_TRAJECTORY_CLEAR,parameter=result.session),"确认清除轨迹故障并作废旧会话；不运动"))
            return True
        if packet.command==CMD_TRAJECTORY_CLEAR:
            self.jobs.clear();self.cancel_quick();self.sealed=False
            if packet.ok and result.state==0 and result.fault==0:
                self.session=0;self.status.set("轨迹故障已确认清除；参考保留、旧会话作废、未运动。核对实际位置后重新准备轨迹。")
            else:self.status.set(f"清除被拒绝：{packet.status_text}；故障／参考／报警需检查，未恢复运动")
            return True
        if self.quick_phase=="checking" and result.active:
            self.jobs.clear();self.cancel_quick();self.status.set("下位机已有活动任务，请先停止并确认停稳；没有覆盖原任务")
            return True
        if self.quick_phase=="return_check" and result.active:
            self.jobs.clear();self.cancel_quick();self.status.set("两轴任务尚未停稳，请先停止两轴再准备返回")
            return True
        if not packet.ok:
            self.jobs.clear();self.renew=False;self.sealed=False;self.cancel_quick()
        if packet.command==CMD_TRAJECTORY_START and packet.ok:
            self.cancel_quick()
            self.renew=True;self.next_renew=time.monotonic()+0.75
        if not result.active:self.renew=False
        if packet.command==CMD_TRAJECTORY_SEAL and packet.ok:
            self.sealed=result.state==2 and self.revision==self.upload_revision
        prefix="" if packet.ok else f"命令拒绝：{packet.status_text}；"
        feedback=(f"缓存角度：{result.axis1_deg:+.3f}° / {result.axis2_deg:+.3f}°；"
                  f"跟随误差：{result.error1_deg:+.3f}° / {result.error2_deg:+.3f}°"
                  if result.feedback_available else "位置尚未采样或参考／通信无效；不要将缓存0值视为水平")
        self.status.set(prefix+f"会话{result.session}：{result.state_text}；{result.fault_text}\n"
            f"上传{result.uploaded_count}/{result.point_count}点，路径计划进度{result.elapsed_s:.2f}/{result.duration_s:.2f}秒（共同减速时不等于实际用时）；"
            f"最大调度间隔{result.max_gap_ms}ms，本轮通信耗时{result.last_cycle_ms}ms\n"
            +feedback)
        self.app.result_text.set(f"两轴轨迹：{result.state_text}；{result.fault_text}")
        if packet.command==CMD_TRAJECTORY_SEAL and self.quick_phase=="uploading":
            if self.sealed:
                self.quick_phase="starting";self._send_start()
            else:
                self.cancel_quick();self.status.set("编辑参数已变化，已取消一键自动启动；请重新确认")
        if packet.command==CMD_TRAJECTORY_SEAL and self.quick_phase=="return_uploading":
            self.cancel_quick()
            self.status.set("返回路径已校验，尚未启动。核对预览后点击“启动已校验轨迹”；返回后恢复原轨迹草稿。"
                            if self.sealed else "返回路径未校验，禁止启动")
        return True

    def close(self):
        if self.app.trajectory_running or self.jobs or self.quick_phase:self.stop()
        self.window.withdraw()
