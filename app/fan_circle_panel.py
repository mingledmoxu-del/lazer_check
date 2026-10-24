"""Geometry frontend to the existing shared finite-trajectory engine."""
import csv
import json
import math
import time
from dataclasses import asdict
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

from fan_circle import FanCircle
from trajectory import decode_status
from protocol import (CMD_TRAJECTORY_START,CMD_TRAJECTORY_STATUS,CMD_TRAJECTORY_BEGIN,
                      CMD_TRAJECTORY_POINT,CMD_TRAJECTORY_SEAL,CMD_TRAJECTORY_STOP)


class FanCirclePanel:
    def __init__(self,app,engine):
        self.app=app;self.engine=engine;self.plan=None;self.points=();self.dirty=True
        self.operation=None;self.session=None;self.clock=None;self.samples=[];self.events=[];self.report={}
        self.run_plan=None;self.uploaded_revision=None;self.intervals=None
        self.window=tk.Toplevel(app.root);self.window.title('连续圆周测试 v3.9.6 · 沿用v3.9.4固件')
        self.window.geometry('1360x900');self.window.protocol('WM_DELETE_WINDOW',self.close)
        outer=ttk.Frame(self.window,padding=10);outer.pack(fill='both',expand=True)
        ttk.Label(outer,text='先生成并预览 → 定位到起点 → 上传整条圆周轨迹（不运动） → 单独启动。中间不停点，不采集激光，不做真实叶片锁相。').pack(anchor='w')
        ttk.Label(outer,text='模型：理想正交俯仰/水平转轴、激光器在坐标原点；距离沿前方Z轴，右侧X为正。真实安装需核对零位、几何和路径。').pack(anchor='w')
        row=ttk.Frame(outer);row.pack(fill='x',pady=4)
        self.scene=tk.StringVar(value='模型台架')
        combo=ttk.Combobox(row,textvariable=self.scene,values=('模型台架','实机（必填实测几何）'),state='readonly',width=25)
        combo.pack(side='left');combo.bind('<<ComboboxSelected>>',self.select_scene)
        self.direction=tk.StringVar(value='顺时针')
        ttk.Combobox(row,textvariable=self.direction,values=('顺时针','逆时针'),state='readonly',width=10).pack(side='left',padx=8)
        self.direction.trace_add('write',self.changed)
        self.fields={};defaults=asdict(FanCircle())
        labels=(('diameter_m','风扇直径m'),('radius_percent','跟踪半径占叶轮半径%'),('fan_rpm','稳速段风扇rpm'),('blade_count','叶片数'),
            ('distance_m','轴向距离Z m'),('fan_height_m','风扇中心离地m'),('laser_height_m','激光出射点离地m'),('center_x_m','中心横向偏移X m'),
            ('plane_yaw_deg','叶轮平面偏航°'),('plane_tilt_deg','叶轮平面倾斜°'),('pitch_origin_deg','俯仰原点修正°'),('yaw_origin_deg','水平原点修正°'),
            ('start_phase_deg','起始转子相位°'),('cycles','总圈数'),('segments_per_cycle','每圈离散段数'),('ramp_s','渐入/渐出各秒'),
            ('cycle_ms','控制周期ms'),('max_speed_dps','最大轴速度°/s'),('max_accel_dps2','最大轴加速度°/s²'),
            ('max_error_deg','跟随保护误差°'),('end_tolerance_deg','最终停点容差°'))
        grid=ttk.Frame(outer);grid.pack(fill='x')
        for n,(key,label) in enumerate(labels):
            r=n//4;c=(n%4)*2;var=tk.StringVar(value=str(defaults[key]));self.fields[key]=var
            ttk.Label(grid,text=label).grid(row=r,column=c,sticky='w',padx=5,pady=3)
            ttk.Entry(grid,textvariable=var,width=11).grid(row=r,column=c+1,sticky='w')
            var.trace_add('write',self.changed)
        self.checked=tk.BooleanVar(value=False)
        ttk.Checkbutton(outer,variable=self.checked,text='已核对真实参考、几何和整圈可运动路径；输入的风扇rpm不是电机输出轴rpm。').pack(anchor='w',pady=5)
        controls=ttk.Frame(outer);controls.pack(fill='x')
        self.position_rpm=tk.StringVar(value='.1')
        ttk.Label(controls,text='起点定位轴rpm').pack(side='left');ttk.Entry(controls,textvariable=self.position_rpm,width=6).pack(side='left')
        for label,action in (('①生成/预览',self.generate),('②定位起点（运动）',self.position),
                             ('③上传圆周（不运动）',self.upload),('④开始连续画圆',self.start),
                             ('停止两轴',self.stop),('导出轨迹/结果',self.export)):
            ttk.Button(controls,text=label,command=action).pack(side='left',padx=3)
        self.status=tk.StringVar(value='尚未生成。默认模型2.5m、3rpm、1圈，渐入渐出各2s；不会自动上传或运动。')
        ttk.Label(outer,textvariable=self.status,wraplength=1300).pack(anchor='w',pady=5)
        self.metrics=tk.StringVar();ttk.Label(outer,textvariable=self.metrics,wraplength=1300).pack(anchor='w')
        panes=ttk.Frame(outer);panes.pack(fill='both',expand=True)
        self.canvas=tk.Canvas(panes,width=440,height=340,bg='white');self.canvas.pack(side='left',fill='both',expand=True)
        self.canvas.bind('<Configure>',lambda _:self.draw())
        self.preview=tk.Text(panes,width=66,height=16,wrap='none',state='disabled');self.preview.pack(side='left',fill='both',expand=True)
        self.visual=tk.StringVar(value='未观察');self.note=tk.StringVar()
        observation=ttk.Frame(outer);observation.pack(fill='x',pady=4)
        ttk.Label(observation,text='本次目测抖动').pack(side='left')
        ttk.Combobox(observation,textvariable=self.visual,values=('未观察','无明显','轻微','明显'),state='readonly',width=10).pack(side='left')
        ttk.Entry(observation,textvariable=self.note,width=70).pack(side='left',padx=5)
        ttk.Label(outer,text='总点数≤128，点间隔≥50ms，总时长≤600s；固定时间执行，跟不上报错停止。模型投影/编码器误差不能证明光斑稳定或采集有效。').pack(anchor='w')
        self.generate()

    def changed(self,*_):
        self.dirty=True;self.uploaded_revision=None

    def select_scene(self,*_):
        # Unknown real fan parameters must be entered rather than guessed.
        changes=dict(diameter_m='1.6',distance_m='2.5',fan_height_m='1.5',laser_height_m='.77',pitch_origin_deg='.267',yaw_origin_deg='.234')
        if self.scene.get().startswith('实机'):
            changes={k:'' for k in changes}
        for key,value in changes.items():self.fields[key].set(value)
        self.checked.set(False)

    def config(self):
        ints=('blade_count','cycles','segments_per_cycle','cycle_ms')
        values={key:(int(var.get()) if key in ints else float(var.get())) for key,var in self.fields.items()}
        return FanCircle(**values,clockwise=self.direction.get()=='顺时针')

    def generate(self):
        if self.operation in ('uploading','starting','running','positioning','stopping'):
            self.status.set('当前动作未结束，先停止后再生成新轨迹。');return False
        try:plan=self.config();points=plan.points()
        except (ValueError,TypeError) as exc:
            self.status.set('生成失败：'+str(exc));return False
        self.plan=plan;self.points=points;self.dirty=False;self.uploaded_revision=None
        req=plan.requirements(points)
        text=(f'{len(points)}个时间点；{plan.cycles}圈，总计划{plan.duration_s:.3f}s（含渐入/渐出）；\n'
              f'稳速段{plan.fan_rpm:g}rpm / {plan.period_s:g}s每转；半径{plan.radius_m:g}m。\n'
              f'首点/末点：({points[0].axis1_deg:g}°, {points[0].axis2_deg:g}°)。\n'
              f'离散角度轨迹峰值约{req["peak_speed_dps"]:.3f}°/s（{req["peak_speed_dps"]/6:.3f}轴rpm），'
              f'{req["peak_accel_dps2"]:.3f}°/s²。\n'
              'time_s,axis1_deg,axis2_deg\n'+'\n'.join(f'{p.time_s:.3f},{p.axis1_deg:.3f},{p.axis2_deg:.3f}' for p in points))
        self.preview.configure(state='normal');self.preview.delete('1.0','end');self.preview.insert('end',text);self.preview.configure(state='disabled')
        self.status.set('已离线生成：可预览，再定位起点、上传和启动。修改参数须重新生成/上传。')
        self.draw();return True

    def available(self):
        if self.operation in ('uploading','starting','running','positioning','stopping'):return False
        return self.app.business_panel.available()

    def draft(self):
        if self.dirty or not self.points:
            messagebox.showwarning('先生成','参数已改变或轨迹为空，请先重新生成并预览。',parent=self.window);return False
        if not self.checked.get():
            messagebox.showwarning('核对实物','请先核对并勾选真实参考、几何和路径。',parent=self.window);return False
        return True

    def position(self):
        if not self.available() or not self.draft():return
        try:
            speed=float(self.position_rpm.get())
            if not math.isfinite(speed) or not .01<=speed<=100:raise ValueError('起点定位速度为实际轴0.01～100rpm')
        except ValueError as exc:self.status.set(str(exc));return
        first=self.points[0]
        if not messagebox.askyesno('定位将实际运动',f'两轴前往首点{first.axis1_deg:g}° / {first.axis2_deg:g}°，轴速度{speed:g}rpm。\n这是到首点的关节角定位，请核对整个返回路径；不会开始画圆。',parent=self.window):return
        if not self.available():return
        panel=getattr(self.app,'joint_timing_panel',None)
        if panel is None:
            from joint_timing_panel import JointTimingPanel
            panel=JointTimingPanel(self.app);panel.window.withdraw();self.app.joint_timing_panel=panel
        panel.completion_tolerance=self.plan.end_tolerance_deg
        self.operation='positioning';self.position_row_start=len(panel.rows)
        panel._begin_run([(first.axis1_deg,first.axis2_deg)],speed,append_records=True)
        self.status.set('起点定位中；完成后还需上传圆周并单独启动。')

    def upload(self):
        if not self.available() or not self.draft():return
        e=self.engine;plan=self.plan
        e.points=self.points;e.saved_points=();e.saved_values=None
        e.show_advanced.set(True);e.adaptive.set(False)
        for key,value in dict(cycle_ms=plan.cycle_ms,speed=plan.max_speed_dps,accel=plan.max_accel_dps2,
                              error=plan.max_error_deg,tolerance=plan.end_tolerance_deg).items(): e.values[key].set(str(value))
        e.invalidate()
        self.operation='uploading';self.events=[];self.samples=[];self.clock=None;self.run_plan=plan
        initial=self.report.get('initial_position_rows',[])
        self.report=dict(version='3.9.6',firmware='3.9.4 unchanged',scene=self.scene.get(),config=asdict(plan),
            created_at=datetime.now().astimezone().isoformat(),adaptive=False,laser_acquisition=False,
            real_rotor_phase_locked=False,geometry='ideal coincident orthogonal pan/tilt axes',requirements=plan.requirements(self.points))
        self.report['run_trajectory']=[asdict(p) for p in self.points]
        self.report['initial_position_rows']=initial
        self.visual.set('未观察');self.note.set('')
        self.intervals=(getattr(e,'status_interval',.5),getattr(e,'position_interval',1))
        e.status_interval=.1;e.position_interval=.5
        e.upload();self.session=e.session
        if not e.jobs and not self.app.pending:
            self.operation=None;self.restore_intervals()
            self.status.set('上传未启动或发送失败，请检查连接、参数和轨迹状态。');return
        self.status.set('正在上传整条轨迹；上传/校验不会启动电机。')

    def start(self):
        if not self.available() or not self.draft():return
        e=self.engine
        if self.session!=e.session or self.uploaded_revision!=e.revision or not e.sealed or tuple(e.points)!=tuple(self.points):
            self.status.set('当前圆周尚未校验或被其他轨迹替换，请重新上传。');return
        self.operation='starting'
        self.intervals=(getattr(e,'status_interval',.5),getattr(e,'position_interval',1))
        e.status_interval=.1;e.position_interval=.5
        e.start()
        if not self.app.trajectory_running:
            self.operation='ready';self.restore_intervals()

    def sent(self,frame,now):
        if self.operation not in ('uploading','starting','running','stopping'):return
        command=frame[4]
        if command in (CMD_TRAJECTORY_BEGIN,CMD_TRAJECTORY_POINT,CMD_TRAJECTORY_SEAL,CMD_TRAJECTORY_START,CMD_TRAJECTORY_STOP):
            self.events.append(dict(event='sent',command=command,monotonic=now))
        if command==CMD_TRAJECTORY_START and self.operation=='starting':
            self.clock=now;self.samples=[];self.operation='running'
            self.report['start_sent_monotonic']=now
            self.uploaded_revision=None
        if command==CMD_TRAJECTORY_STOP:
            self.operation='stopping';self.uploaded_revision=None

    def observe(self,packet):
        if self.operation=='positioning':
            p=self.app.joint_timing_panel
            self.status.set('起点定位：'+p.status.get())
            if not p.phase:
                self.operation=None;self.report['initial_position_rows']=p.rows[self.position_row_start:]
            return
        if packet.command not in range(0x30,0x39) or self.operation not in ('uploading','starting','running','stopping'):return
        now=time.monotonic()
        self.events.append(dict(event='response',command=packet.command,monotonic=now,ok=packet.ok))
        try:state=decode_status(packet.data)
        except ValueError:
            self.operation=None;self.restore_intervals();self.status.set('轨迹响应无效；流程已取消，请检查停机状态。');return
        if state.session!=self.session:return
        if not packet.ok:
            self.operation=None;self.restore_intervals();self.report['failure']=packet.status_text
            self.status.set('下位机拒绝：'+self.engine.status.get());return
        if packet.command==CMD_TRAJECTORY_SEAL and self.operation=='uploading':
            begin=next((event['monotonic'] for event in self.events if event['event']=='sent' and event['command']==CMD_TRAJECTORY_BEGIN),None)
            if begin is not None:self.report['upload_elapsed_s']=now-begin
            self.operation='ready' if self.engine.sealed else None
            self.uploaded_revision=self.engine.revision if self.engine.sealed else None
            self.status.set('圆周轨迹已预载，未运动；确认起点后点击④开始。' if self.engine.sealed else '预载草稿已失效，请重新生成/上传。')
            self.restore_intervals();return
        if self.clock is not None:
            if packet.command==CMD_TRAJECTORY_START:self.report['start_ack_s']=now-self.clock
            row=dict(observed_s=now-self.clock,state=state.state,state_text=state.state_text,fault=state.fault,
                elapsed_s=state.elapsed_s,angle1_deg=state.axis1_deg,angle2_deg=state.axis2_deg,
                error1_deg=state.error1_deg,error2_deg=state.error2_deg,feedback_available=state.feedback_available,
                max_gap_ms=state.max_gap_ms,last_cycle_ms=state.last_cycle_ms)
            if state.feedback_available:
                uv=self.run_plan.project_angles(state.axis1_deg,state.axis2_deg)
                if state.elapsed_s>0:self.report.setdefault('first_control_feedback_s',now-self.clock)
                if uv is not None:
                    row.update(plane_u_m=uv[0],plane_v_m=uv[1],radial_error_m=math.hypot(*uv)-self.run_plan.radius_m)
                    if math.hypot(*uv)>1e-6:
                        delta=math.atan2(-uv[0],-uv[1])-self.run_plan.phase(state.elapsed_s)
                        row['phase_error_deg']=math.degrees(math.atan2(math.sin(delta),math.cos(delta)))
            if state.state in (4,5,6) and state.elapsed_s>=state.duration_s:
                self.report.setdefault('planned_end_observed_s',now-self.clock)
            if len(self.samples)<10000:self.samples.append(row)
            else:self.report['dropped_samples']=self.report.get('dropped_samples',0)+1
            self.report['last_state']=row
        self.status.set(self.engine.status.get());self.update_metrics();self.draw()
        if state.state in (6,7,8):
            self.operation=None;self.restore_intervals()
            self.report['result']=state.state_text
            if self.clock is not None:
                self.report['wall_elapsed_s']=now-self.clock
                if 'planned_end_observed_s' in self.report:
                    self.report['completion_tail_s']=now-self.clock-self.report['planned_end_observed_s']

    def update_metrics(self):
        rows=[r for r in self.samples if r['feedback_available']]
        if not rows:self.metrics.set('暂无有效控制反馈；不把未采样的0当作真实位置。');return
        peak=[max(abs(r[f'error{axis}_deg']) for r in rows) for axis in (1,2)]
        rms=[math.sqrt(sum(r[f'error{axis}_deg']**2 for r in rows)/len(rows)) for axis in (1,2)]
        self.report['summary']=dict(samples=len(rows),max_error_deg=peak,rms_error_deg=rms,
            max_gap_ms=max(r['max_gap_ms'] for r in rows),max_cycle_ms=max(r['last_cycle_ms'] for r in rows))
        radial=[abs(r['radial_error_m']) for r in rows if 'radial_error_m' in r]
        self.report['summary']['max_radial_error_m']=max(radial) if radial else None
        self.metrics.set(f'有效控制反馈{len(rows)}条；两轴峰值误差{peak[0]:.3f}° / {peak[1]:.3f}°；RMS {rms[0]:.3f}° / {rms[1]:.3f}°；最大调度间隔{self.report["summary"]["max_gap_ms"]}ms。红线为编码器+理想几何投影，并非实测光斑。')

    def restore_intervals(self):
        if self.intervals is not None:
            self.engine.status_interval,self.engine.position_interval=self.intervals;self.intervals=None

    def stop(self):
        if self.operation=='positioning':self.app.joint_timing_panel.stop('停止圆周起点定位');return
        if self.operation in ('uploading','starting','running','stopping') or (self.session is not None and self.session==self.engine.session):
            self.operation='stopping';self.engine.stop();self.uploaded_revision=None

    def interrupted(self,reason):
        if self.operation:
            self.report['interrupted']=reason;self.operation=None;self.restore_intervals();self.uploaded_revision=None
            self.status.set(reason+'；不会自动恢复。')

    def draw(self):
        plan=self.run_plan if self.operation in ('starting','running','stopping') else self.plan
        if plan is None:return
        c=self.canvas;c.delete('all');w=max(200,c.winfo_width());h=max(200,c.winfo_height())
        scale=min(w-40,h-70)/plan.diameter_m;cx=w/2;cy=h/2+10
        def xy(u,v):return cx+u*scale,cy-v*scale
        for radius,color in ((plan.diameter_m/2,'#bbb'),(plan.radius_m,'#2766b5')):
            x,y=xy(-radius,radius);X,Y=xy(radius,-radius);c.create_oval(x,y,X,Y,outline=color,width=2)
        c.create_text(10,12,anchor='nw',text='叶轮平面：蓝色计划圈 / 红色角度反馈投影（非光斑实测）')
        stride=max(1,math.ceil(len(self.samples)/1500))
        actual=[xy(r['plane_u_m'],r['plane_v_m']) for r in self.samples[::stride] if 'plane_u_m' in r] if plan==self.run_plan else []
        if len(actual)>1:c.create_line(*[v for p in actual for v in p],fill='#cb3030',width=2)
        if self.points:
            start=plan.project_angles(self.points[0].axis1_deg,self.points[0].axis2_deg)
            if start:
                x,y=xy(*start);c.create_oval(x-4,y-4,x+4,y+4,fill='#2766b5')

    def export(self):
        if self.operation in ('uploading','starting','running','positioning','stopping'):
            self.status.set('请在本次完成或停止确认后导出，避免文件写入阻塞运动通信。');return
        if not self.points:return
        path=filedialog.asksaveasfilename(parent=self.window,defaultextension='.json',filetypes=[('JSON','*.json')])
        if not path:return
        try:
            self.update_metrics()
            report=dict(self.report,visual=self.visual.get(),note=self.note.get(),events=self.events,samples=self.samples,
                draft_config=asdict(self.plan),trajectory=[asdict(p) for p in self.points])
            Path(path).write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
            with Path(path).with_suffix('.csv').open('w',encoding='utf-8-sig',newline='') as f:
                writer=csv.DictWriter(f,fieldnames=('observed_s','state','state_text','fault','elapsed_s','angle1_deg','angle2_deg','error1_deg','error2_deg','feedback_available','max_gap_ms','last_cycle_ms','plane_u_m','plane_v_m','radial_error_m','phase_error_deg'))
                writer.writeheader();writer.writerows(self.samples)
            with Path(path).with_name(Path(path).stem+'_trajectory.csv').open('w',encoding='utf-8-sig',newline='') as f:
                writer=csv.writer(f);writer.writerow(('time_s','axis1_deg','axis2_deg'))
                writer.writerows((p['time_s'],p['axis1_deg'],p['axis2_deg']) for p in self.report.get('run_trajectory',[asdict(p) for p in self.points]))
        except (ValueError,OSError) as exc:messagebox.showerror('导出失败',str(exc),parent=self.window)

    def close(self):
        self.stop();self.window.withdraw()
