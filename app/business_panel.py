"""Daily preparation flow using the main window's serialized transport."""
import struct
import time
import tkinter as tk
from tkinter import ttk, messagebox

from protocol import (CMD_STOP, CMD_SET_ZERO, CMD_READ_AXES, CMD_MOVE_OUTPUT_RELATIVE,
                      build_request, build_move_output_relative)
from trajectory import decode_axes


def business_ready(positions):
    return (positions is not None and all(p.usable and not p.motor_rpm and not p.alarm_code and
            not p.guard_fault and not p.flags&16 for p in positions) and
            bool(positions[0].flags&2) and abs(positions[0].angle_deg)<=0.3)


class BusinessPanel:
    def __init__(self, app, parent):
        self.app=app;self.phase=None;self.positions=None;self.received_at=0
        self.next_read=0;self.next_frame=None;self.axis=1;self.zeros=0;self.horizontal_samples=0;self.generation=0
        self.message=tk.StringVar(value="首次建立参考后即可按坐标运动；再次回到+35°和停止均保留已有参考。")
        self.position_text=tk.StringVar(value="未读取两轴位置")
        self.readiness=tk.StringVar(value="尚未就绪")
        box=ttk.LabelFrame(parent,text="日常业务准备",padding=10);box.pack(fill="x",pady=10)
        ttk.Label(box,text="① 复位到+35° → ② 移到水平0° → ③ 两轴业务。低电平触发，DI0=+35°、DI2=−29°；电机2仍需参考。",
                  wraplength=900).pack(anchor="w")
        buttons=ttk.Frame(box);buttons.pack(fill="x",pady=8)
        speed_row=ttk.Frame(box);speed_row.pack(fill="x",pady=4)
        ttk.Label(speed_row,text="复位 / 回水平速度设定（0.2～1.5，实际rpm=数值÷6）").pack(side="left")
        ttk.Spinbox(speed_row,from_=0.2,to=1.5,increment=0.1,textvariable=app.reset_speed_rpm,width=8).pack(side="left",padx=8)
        for label,action in (("① 回到 +35° / 建立参考",app._reset_axis1),
                             ("② 电机1移到水平0°",self.horizontal),
                             ("电机2：确认当前位置为0°",lambda:self.confirm_reference(2)),
                             ("③ 两轴业务轨迹",self.open_business)):
            ttk.Button(buttons,text=label,command=action).pack(side="left",padx=3)
        tk.Button(buttons,text="停止",command=app._stop,bg="#c62828",fg="white").pack(side="right")
        ttk.Button(buttons,text="故障处理",command=self.open_recovery).pack(side="left",padx=3)
        ttk.Label(box,textvariable=self.position_text,wraplength=900).pack(anchor="w")
        ttk.Label(box,textvariable=self.readiness,font=("Microsoft YaHei UI",11,"bold")).pack(anchor="w",pady=4)
        ttk.Label(box,textvariable=self.message,wraplength=900).pack(anchor="w")

    def available(self):
        a=self.app;p=getattr(a,"trajectory_panel",None)
        joint=getattr(a,"joint_timing_panel",None)
        if joint and joint.phase:
            messagebox.showwarning("联动计时中","请先停止两轴联动计时。",parent=a.root)
            return False
        debug=getattr(a,"debug_panel",None)
        if not self.phase and debug is not None:debug.release_idle_owner(1)
        busy=(self.phase or not a.port or (a.pending and not getattr(a,"pending_background",False)) or getattr(a,"deferred_send",None) or
              a.trajectory_running or a.home_running or a.auto_motor is not None or a.active_motor is not None or
              a.stop_waiting or a.diagnostic_active or (p and (p.quick_phase or p.jobs)))
        busy=busy or a.parameter_check_queue is not None or a.profile_read_queue is not None
        if busy:
            messagebox.showwarning("准备未完成","请先连接、停止其他动作并等停稳。",parent=a.root)
            return False
        return True

    def begin(self, axis, action):
        self.axis=axis;self.action=action;self.phase="stopping";self.zeros=0
        self.horizontal_samples=0;self.deadline=time.monotonic()+90
        self.app.active_motor=axis;self.app._lock_auto_axis(True)
        self.next_frame=build_request(axis,CMD_STOP)
        self.message.set("正在停止并确认连续两次零速；尚未发送参考／定位命令")

    def confirm_reference(self, axis):
        if axis!=2:return
        if not self.available():return
        generation=self.generation
        prompt="确认电机2当前位置为业务0°？仅建立参考，不启用角度范围。"
        if messagebox.askyesno("确认真实参考姿态",prompt+"\n掉电后需重新确认；不要在任意姿态反复置零消除报错。",parent=self.app.root):
            # The modal dialog can process a Stop/disconnect event. Recheck availability.
            if generation==self.generation and self.available():self.begin(axis,"reference")

    def horizontal(self):
        try:
            speed=float(self.app.reset_speed_rpm.get())
            if not 0.2<=speed<=1.5:raise ValueError()
        except (ValueError,TypeError):
            messagebox.showerror("速度非法","输出轴速度须在0.2～1.5 rpm内",parent=self.app.root);return
        if not self.available():return
        generation=self.generation
        if not messagebox.askyesno("移到水平0°","电机1将从当前可信坐标缓慢移到水平0°。确认运动空间和线缆安全？",parent=self.app.root):return
        if generation==self.generation and self.available():
            self.horizontal_speed=speed/6;self.begin(1,"horizontal")

    def cancel(self, text="准备已取消；需重新读取位置"):
        owned=self.phase is not None
        self.generation+=1;self.phase=None;self.next_frame=None;self.positions=None;self.received_at=0
        self.horizontal_samples=0;self.message.set(text);self.readiness.set("尚未就绪")
        if owned:
            self.app.active_motor=None;self.app._lock_auto_axis(False)

    def fail(self, text):
        axis=self.axis;moving=self.phase in ("moving","commanding_move")
        self.cancel(text)
        if moving and self.app.port:
            self.app._send(build_request(axis,CMD_STOP),CMD_STOP,"准备流程失败，请求停止",allow_override=True)

    def open_business(self):
        if self.phase or time.monotonic()-self.received_at>2 or self.horizontal_samples<2 or not business_ready(self.positions):
            messagebox.showwarning("尚未就绪","先确认两轴参考，并让电机1到水平0°、两轴停稳；等待实际位置刷新。",parent=self.app.root)
            return
        self.app._open_trajectory()

    def open_recovery(self):
        if self.phase:
            messagebox.showwarning("准备中","请先停止或完成准备流程",parent=self.app.root);return
        # Viewing/acknowledging a fault must remain accessible when axis 1 is
        # away from horizontal; opening the window sends no motion command.
        self.app._open_trajectory()

    def service(self):
        a=self.app;now=time.monotonic()
        if now-self.received_at>2:
            self.readiness.set("位置未更新，尚未就绪");self.horizontal_samples=0
        if self.phase and now>self.deadline:
            self.fail("准备超时，已取消后续动作；请检查实际位置和使能状态")
            return
        if not a.port or a.pending or getattr(a,"deferred_send",None):return
        if self.next_frame is not None:
            frame=self.next_frame;self.next_frame=None
            if not a._send(frame,frame[4],"业务准备步骤",preparation_step=True):self.fail("准备命令未发送，已取消")
            return
        panel=getattr(a,"trajectory_panel",None)
        debug=getattr(a,"debug_panel",None)
        if (not self.phase and (a.diagnostic_active or a.parameter_check_queue or a.profile_read_queue or
                (panel and (panel.jobs or panel.quick_phase)) or (debug and getattr(debug,"jobs",None)))):return
        if now>=self.next_read:
            moving=bool(self.phase or a.auto_motor is not None or getattr(a,"active_motor",None) is not None)
            self.next_read=now+(0.2 if moving else 0.75)
            a._send(build_request(1,CMD_READ_AXES),CMD_READ_AXES,"两轴实际位置",background=not bool(self.phase))

    def handle(self, packet):
        if packet.command==CMD_READ_AXES:
            try:
                if not packet.ok:raise ValueError("实际位置读取失败，请检查配套固件及通信")
                positions=decode_axes(packet.data);self.positions=positions;self.received_at=time.monotonic()
                self.position_text.set("；".join(f"电机{i}："+(f"{p.angle_deg:+.3f}°，{p.motor_rpm} rpm，反馈{p.sample_age_ms}ms前"
                    if p.usable else "参考无效／位置未知") for i,p in enumerate(positions,1)))
                if business_ready(positions) and not self.app.trajectory_running:
                    self.horizontal_samples+=1
                else:self.horizontal_samples=0
                self.readiness.set("业务就绪：电机1已在水平0°、两轴参考有效且停稳" if self.horizontal_samples>=2 else
                                   "尚未就绪：需要两轴参考有效、电机1水平0°并停稳")
                if self.phase=="zero_wait":
                    p=positions[self.axis-1]
                    if p.source!=0 or p.sample_age_ms>1000 or p.alarm_code:
                        raise ValueError("未获得可信停稳反馈或存在驱动报警")
                    self.zeros=self.zeros+1 if p.motor_rpm==0 else 0
                    if self.zeros>=2:
                        if self.action=="reference":
                            self.phase="setting_reference";self.next_frame=build_request(self.axis,CMD_SET_ZERO,parameter=0x5A52)
                        else:
                            if not p.usable or not p.flags&2 or p.guard_fault:raise ValueError("先复位到+35°建立参考，不能从未知位置定位")
                            if abs(p.angle_deg)<=0.3:self.finish("电机1已在水平0°")
                            else:
                                self.phase="commanding_move"
                                self.next_frame=build_move_output_relative(1,getattr(self,"horizontal_speed",0.1),-p.angle_deg)
                elif self.phase=="verify_reference":
                    p=positions[self.axis-1];expected=0
                    if not p.usable or p.source!=0 or p.motor_rpm or p.guard_fault or abs(p.angle_deg-expected)>0.001:
                        raise ValueError("参考读回不一致，未继续运动")
                    if self.axis==1 and not p.flags&2:raise ValueError("电机1固定保护未启用")
                    self.finish(f"电机{self.axis}参考已由下位机确认：{expected:+g}°；未发送定位动作")
                elif self.phase=="moving":
                    p=positions[0]
                    if not p.usable or p.guard_fault or p.alarm_code:raise ValueError("定位反馈或保护异常")
                    if abs(p.angle_deg)<=0.3 and p.motor_rpm==0:
                        self.zeros+=1
                        if self.zeros>=2:self.finish("电机1已到水平0°并连续停稳；电机2参考有效即可开始业务")
                    else:self.zeros=0
            except (ValueError,struct.error) as exc:
                self.positions=None;self.horizontal_samples=0;self.readiness.set("位置未知，尚未就绪")
                if self.phase:self.fail(str(exc))
            return False  # Also deliver position snapshots to the trajectory window.
        if self.phase and packet.device_id==self.axis:
            if not packet.ok:self.fail(f"准备命令失败：{packet.status_text}");return True
            if self.phase=="stopping" and packet.command==CMD_STOP:self.phase="zero_wait";self.next_read=0
            elif self.phase=="setting_reference" and packet.command==CMD_SET_ZERO:self.phase="verify_reference";self.next_read=0
            elif self.phase=="commanding_move" and packet.command==CMD_MOVE_OUTPUT_RELATIVE:
                self.phase="moving";self.zeros=0;self.next_read=0
            return True
        return False

    def finish(self, text):
        self.phase=None;self.next_frame=None;self.app.active_motor=None
        self.app._lock_auto_axis(False);self.message.set(text)
