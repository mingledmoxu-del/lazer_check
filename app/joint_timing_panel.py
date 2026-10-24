"""Two-axis point-to-point timing experiment on the shared GUI transport."""
import csv
import json
import math
import struct
import secrets
import time
from collections import deque
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

from protocol import build_request, build_move_axes_point
from trajectory import decode_axes
from point_bank import PRESETS,upload_frames,run_route_point,decode_route


def parse_plan(text, cycles, speed):
    points = [tuple(float(v.strip()) for v in line.split(','))
              for line in text.strip().splitlines() if line.strip()]
    if not points or len(points) > 100 or any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in points):
        raise ValueError('每行填写两个有限角度：电机1角度,电机2角度；最多100行')
    if any(not -29 <= p[0] <= 35 for p in points):
        raise ValueError('电机1目标须在 −29°～+35° 内；仍以板端保护为准')
    if not 1 <= cycles <= 100 or not math.isfinite(speed) or not 0.01 <= speed <= 100:
        raise ValueError('循环1～100次；实际输出轴速度0.01～100 rpm（50:1）；仍受板端及驱动器限制')
    return points * cycles, speed


class JointTimingPanel:
    def __init__(self, app):
        self.app = app
        self.phase = None
        self.loaded_route=None;self.run_selection=None;self.loaded_routes={};self.restoring_bank=False
        self.rows = []
        self.window = tk.Toplevel(app.root)
        self.window.title('两轴联动计时 / 固定点位预载')
        self.window.geometry('1250x750')
        self.window.protocol('WM_DELETE_WINDOW', self.close)
        outer = ttk.Frame(self.window, padding=12); outer.pack(fill='both', expand=True)
        ttk.Label(outer, text='两轴均建立参考后使用。每行一个联动目标（电机1°,电机2°），两轴均停稳后进入下一行。').pack(anchor='w')
        ttk.Label(outer, text='预载功能需要v3.9.1固件：上传不运行；运行选中点后停稳等待下一条指令。RAM预载，复位后重新上传。').pack(anchor='w')
        ttk.Label(outer, text='v3.9.4：支持6套预载库，每套1～16点；实际输出轴rpm，默认0.1。库5、6需配套新固件；修改速度后重新上传。').pack(anchor='w')
        presets=ttk.Frame(outer);presets.pack(fill='x',pady=4)
        ttk.Label(presets,text='固定路线').pack(side='left')
        self.preset=tk.StringVar(value='自定义')
        combo=ttk.Combobox(presets,textvariable=self.preset,values=('自定义',*PRESETS),state='readonly',width=28)
        combo.pack(side='left',padx=6);combo.bind('<<ComboboxSelected>>',self.select_preset)
        self.route_id=tk.StringVar(value='1')
        ttk.Label(presets,text='板端库编号1～6').pack(side='left')
        ttk.Entry(presets,textvariable=self.route_id,width=4).pack(side='left',padx=6)
        self.plan = tk.Text(outer, height=6, width=35); self.plan.pack(anchor='w', pady=6)
        self.plan.insert('1.0', '10,10\n0,0\n10,10')
        self.plan.edit_modified(False);self.plan.bind('<<Modified>>',self.plan_changed)
        controls = ttk.Frame(outer); controls.pack(fill='x')
        self.speed = tk.StringVar(value='0.1'); self.cycles = tk.StringVar(value='1');self.tolerance=tk.StringVar(value='0.2')
        for label, variable in (('实际输出轴速度 rpm', self.speed), ('循环次数', self.cycles)):
            ttk.Label(controls, text=label).pack(side='left')
            ttk.Entry(controls, textvariable=variable, width=8).pack(side='left', padx=6)
        ttk.Label(controls,text='终点容差±°').pack(side='left')
        ttk.Combobox(controls,textvariable=self.tolerance,values=('0.05','0.1','0.2'),width=5,state='readonly').pack(side='left',padx=6)
        ttk.Button(controls, text='开始联动计时', command=self.start).pack(side='left', padx=6)
        ttk.Button(controls, text='停止两轴', command=self.stop).pack(side='left', padx=6)
        ttk.Button(controls, text='导出CSV', command=self.export).pack(side='left', padx=6)
        ttk.Button(controls, text='读取板端耗时', command=self.read_board_timing).pack(side='left', padx=6)
        bank_controls=ttk.Frame(outer);bank_controls.pack(fill='x',pady=6)
        ttk.Button(bank_controls,text='提前上传点位（不运动）',command=self.preload).pack(side='left')
        self.selected_point=tk.StringVar()
        self.point_combo=ttk.Combobox(bank_controls,textvariable=self.selected_point,state='readonly',width=35)
        self.point_combo.pack(side='left',padx=8)
        ttk.Button(bank_controls,text='运行选中预载点',command=self.run_point).pack(side='left')
        ttk.Label(bank_controls,text='每个点含电机1、电机2绝对角度；不会自动跳到下一点。').pack(side='left',padx=8)
        self.speed.trace_add('write',lambda *_:self.invalidate_route() if not self.restoring_bank else None)
        self.tolerance.trace_add('write',lambda *_:self.invalidate_route() if not self.restoring_bank else None)
        self.route_id.trace_add('write',lambda *_:self.select_bank())
        self.status = tk.StringVar(value='尚未开始；不会自动建立或清除参考。')
        ttk.Label(outer, textvariable=self.status, wraplength=1100).pack(anchor='w', pady=10)
        columns = ('step','axis','target','tol','prep','ack','start','window','settle','total','joint_total','skew','result','board_prep','ack_other')
        self.table = ttk.Treeview(outer, columns=columns, show='headings', height=16)
        for key, label in zip(columns, ('段','轴','目标°','终点容差°','准备s','应答s','启动s','到位s','到位→停稳s','轴总耗时s','联动总耗时s','下发间隔s','结果','板端双轴准备s','应答其余s')):
            self.table.heading(key, text=label); self.table.column(key, width=94)
        self.table.pack(fill='both', expand=True)
        scroll=ttk.Scrollbar(outer,orient='horizontal',command=self.table.xview)
        scroll.pack(fill='x');self.table.configure(xscrollcommand=scroll.set)

    def start(self):
        if self.phase: return
        try:
            points, speed = parse_plan(self.plan.get('1.0','end'), int(self.cycles.get()), float(self.speed.get()))
            self.completion_tolerance=float(self.tolerance.get())
            if not .01<=self.completion_tolerance<=.5:raise ValueError('终点容差须在0.01～0.5°内')
        except ValueError as exc:
            messagebox.showerror('计时参数', str(exc), parent=self.window); return
        a = self.app
        debug = getattr(a,'debug_panel',None)
        if debug and (debug.jobs or debug.after_stop is not None or debug.goto_target is not None or debug.raised_request is not None):
            messagebox.showwarning('流程未完成','请先完成或停止复位/定位流程',parent=self.window); return
        if not a.business_panel.available(): return
        if not messagebox.askyesno('开始两轴联动', f'执行{len(points)}段，两轴实际输出轴速度{speed:g}rpm。\n请确认两轴已有参考，且整段路径可运动。', parent=self.window): return
        self._begin_run(points,speed)

    def _begin_run(self,points,speed,selection=None,*,append_records=False):
        self.run_selection=selection
        self.points=deque(points);self.speed_rpm=speed
        if selection is None and not append_records:
            self.rows=[];self.table.delete(*self.table.get_children());self.step=0
        else:self.step=len(self.rows)//2
        self.run_steps=0;self.run_started=time.monotonic();self.next_read=0
        self.phase = 'preflight'; self.deadline = self.run_started + 10
        self.jobs = deque(); self.segment = None; self.awaiting = None
        a=self.app;a.motion_generation = getattr(a,'motion_generation',0) + 1
        a.monitor_until = 0; a._lock_auto_axis(True)
        self.status.set('读取两轴新鲜反馈，检查参考与停稳状态…')
        if selection is not None:
            # Bank RUN verifies reference/pose/stop inside the same board call.
            # Do not add a redundant PC preflight before each event trigger.
            self.begin_segment(None)

    def invalidate_route(self):
        self.loaded_route=None
        self.point_combo['values']=();self.selected_point.set('')

    def plan_changed(self,_event=None):
        if self.plan.edit_modified():
            if not self.restoring_bank:self.invalidate_route()
            self.plan.edit_modified(False)

    def select_bank(self):
        self.invalidate_route()
        try:record=self.loaded_routes.get(int(self.route_id.get()))
        except ValueError:return
        if record is None:return
        self.restoring_bank=True
        try:
            self.plan.delete('1.0','end');self.plan.insert('1.0',record['text']);self.plan.edit_modified(False)
            self.speed.set(record['setting']);self.preset.set(record['preset'])
            self.tolerance.set(str(record['tolerance_deg']))
            self.loaded_route=record
            labels=tuple(f'{n}: ({a:g}°, {b:g}°)' for n,(a,b) in enumerate(record['points']))
            self.point_combo['values']=labels;self.selected_point.set(labels[0])
            self.status.set(f"已选择库{record['route_id']}的预载点；未运动")
        finally:self.restoring_bank=False

    def select_preset(self,_event=None):
        if self.preset.get() in PRESETS:
            self.plan.delete('1.0','end')
            self.plan.insert('1.0','\n'.join(f'{a:g},{b:g}' for a,b in PRESETS[self.preset.get()]))
            self.invalidate_route()

    def preload(self):
        if self.phase:return
        try:
            points,speed=parse_plan(self.plan.get('1.0','end'),1,float(self.speed.get()))
            slot=int(self.route_id.get());token=secrets.randbits(32) or 1
            tolerance=float(self.tolerance.get())
            frames=upload_frames(points,slot=slot,token=token,speed_rpm=speed,tolerance_deg=tolerance)
        except (ValueError,TypeError) as exc:
            messagebox.showerror('预载参数',str(exc),parent=self.window);return
        if not self.app.business_panel.available():return
        debug=getattr(self.app,'debug_panel',None)
        if debug and (debug.jobs or debug.after_stop or debug.goto_target or debug.raised_request):return
        self.invalidate_route()
        self.loaded_routes.pop(slot,None)
        self.upload_info=dict(route_id=slot,token=token,points=tuple(points),speed_rpm=speed,tolerance_deg=tolerance)
        self.upload_info['preset']=self.preset.get()
        self.upload_text=self.plan.get('1.0','end').strip();self.upload_speed=self.speed.get()
        self.upload_tolerance=self.tolerance.get()
        self.jobs=deque(frames);self.points=deque();self.segment=None;self.awaiting=None
        self.upload_started=time.monotonic()
        self.phase='uploading';self.deadline=self.upload_started+60;self.next_read=0
        self.app._lock_auto_axis(True);self.status.set('正在预载并校验；不会启动运动…')

    def run_point(self):
        if self.phase:return
        record=self.loaded_route
        if record is None:
            messagebox.showwarning('尚未预载','请先选择路线并上传点位；修改目标/速度后需重新上传。',parent=self.window);return
        try:index=int(self.selected_point.get().split(':',1)[0]);target=record['points'][index]
        except (ValueError,IndexError):return
        if not self.app.business_panel.available():return
        debug=getattr(self.app,'debug_panel',None)
        if debug and (debug.jobs or debug.after_stop or debug.goto_target or debug.raised_request):return
        self.completion_tolerance=record['tolerance_deg']
        self._begin_run([target],record['speed_rpm'],(record.copy(),index))

    def service(self):
        if not self.phase: return
        a = self.app
        if not a.port: self.disconnected(); return
        if time.monotonic() > self.deadline:
            if self.phase == 'stopping': self.disconnected('停止确认超时，状态未知，请检查两轴'); return
            self.stop('计时超时，已取消后续段'); return
        if a.pending or a.deferred_send: return
        if self.jobs:
            frame = self.jobs.popleft()
            if frame[4] in (0x1A,0x43):
                sent=time.monotonic()
                for axis in (1,2):
                    self.segment['axes'][axis]['sent']=sent
                    a._timing.begin(axis, 'gui_joint_timing', self.segment['target'][axis-1],finish_tolerance_deg=getattr(self,'completion_tolerance',.2))
            self.awaiting = (frame[3],frame[4])
            if not a._send(frame, frame[4], '两轴联动计时', timing_step=True):self.stop('发送失败')
        elif time.monotonic() >= self.next_read:
            self.next_read = time.monotonic() + .05
            self.awaiting = (1,0x37)
            parameter=0x4652 if self.phase in ('preflight','stopping') else 0
            if not a._send(build_request(1,0x37,parameter=parameter),0x37,'联动计时：共享控制反馈/完成直接核验',timing_step=True):self.stop('发送失败')

    def handle(self, packet):
        if packet.command==0x1B:
            if not packet.ok or len(packet.data)!=32:
                self.status.set('板端计时读取失败；需要v3.9.0配套固件');return True
            values=struct.unpack('>8I',packet.data)
            text='；'.join(f'轴{i+1}：初始化{values[4*i]}ms，控制工作{values[4*i+1]}ms，周期{values[4*i+2]}ms，最大周期{values[4*i+3]}ms' for i in (0,1))
            self.status.set(text);self.app._log('BOARD_TIMING '+text);return True
        if not self.phase: return False
        if (packet.device_id,packet.command) != self.awaiting: return False
        self.awaiting = None
        if not packet.ok:
            if self.phase=='uploading' or packet.command==0x43:
                self.loaded_routes.clear();self.invalidate_route()
            self.stop('命令失败：' + packet.status_text); return True
        now = time.monotonic()
        if self.phase=='uploading' and packet.command in (0x40,0x41,0x42):
            try:state=decode_route(packet.data)
            except ValueError as exc:self.stop(str(exc));return True
            info=self.upload_info
            if state['token']!=info['token'] or state['count']!=len(info['points']):self.stop('预载读回不匹配');return True
            self.status.set(f"正在预载：{state['loaded']}/{state['count']}点")
            if packet.command==0x42:
                if not state['ready'] or state['loaded']!=state['count']:self.stop('预载未就绪');return True
                self.phase=None
                for axis in (1,2):self.app._release_stopped_axis(axis)
                if self.upload_text==self.plan.get('1.0','end').strip() and self.upload_speed==self.speed.get() and self.upload_tolerance==self.tolerance.get() and str(info['route_id'])==self.route_id.get():
                    info['speed_rpm']=state['speed_rpm']
                    info['upload_elapsed_ms']=round((now-getattr(self,'upload_started',now))*1000,3)
                    info['text']=self.upload_text;info['setting']=self.upload_speed
                    self.loaded_route=info
                    self.loaded_routes[info['route_id']]=info
                    labels=tuple(f'{n}: ({a:g}°, {b:g}°)' for n,(a,b) in enumerate(info['points']))
                    self.point_combo['values']=labels;self.selected_point.set(labels[0])
                    self.status.set(f"库{info['route_id']}已预载{len(labels)}点，耗时{info['upload_elapsed_ms']/1000:.3f}s，未运动；选择点编号后点击运行")
                    self.app._log('PRELOAD_TIMING '+json.dumps(info,ensure_ascii=False))
                else:self.status.set('已上传旧草稿；界面内容已修改，请重新上传')
            return True
        if packet.command in (0x1A,0x43):
            if self.phase != 'stopping':
                for axis in (1,2):self.segment['axes'][axis]['ack'] = now
                if len(packet.data)==8:
                    board_ms=struct.unpack('>II',packet.data)
                    self.segment['board_prep']=sum(board_ms)/1000
                    self.app._log('JOINT_PREPARE_MS '+str(board_ms))
                    sent=self.segment['axes'][1].get('sent')
                    if sent is not None:
                        self.app._log('JOINT_ACK_TIMING '+json.dumps(dict(
                            ack_ms=round((now-sent)*1000,3),board_axes_ms=board_ms,
                            other_ms=round(max(0,(now-sent)*1000-sum(board_ms)),3))))
            return True
        if packet.command == 6: return True
        if packet.command != 0x37: return False
        try: positions = decode_axes(packet.data)
        except ValueError as exc: self.stop(str(exc)); return True
        if self.phase == 'stopping':
            if self.jobs: return True
            stopped = all(p.source==0 and p.sample_age_ms<=200 and p.motor_rpm==0 and not p.flags&0x30 for p in positions)
            self.zeros = self.zeros + 1 if stopped else 0
            if self.zeros >= 2:
                for axis in (1,2):self.app._timing.stop_confirmed(axis)
                self.phase = None
                for axis in (1,2):self.app._release_stopped_axis(axis)
                self.status.set(self.stop_reason+'；两轴已确认停稳')
            return True
        if any(not p.usable or p.sample_age_ms>200 or p.alarm_code or p.guard_fault for p in positions):
            self.stop('参考/直接反馈失效或存在故障'); return True
        if self.phase == 'preflight':
            if any(p.source!=0 for p in positions):self.stop('预检必须取得直接反馈');return True
            if any(p.motor_rpm or p.flags&0x30 for p in positions):
                self.stop('开始前两轴未停稳'); return True
            self.begin_segment(positions); return True
        for axis, p in enumerate(positions,1):
            m = self.segment['axes'][axis]
            if 'sent' not in m or 'done' in m: continue
            if p.motor_rpm: m.setdefault('motion',now)
            error = abs(p.angle_deg-self.segment['target'][axis-1])
            if error <= .5: m.setdefault('window',now)
            # Completion uses firmware's independent two-zero confirmation, never a bare zero-speed read.
            if error <= getattr(self,'completion_tolerance',.2) and p.source==0 and p.flags&0x40 and not p.flags&0x30 and p.motor_rpm==0:
                m['done'] = now
        if all('done' in m for m in self.segment['axes'].values()):
            self.record_segment('完成')
            self.begin_segment(positions)
        return True

    def begin_segment(self, positions):
        if not self.points:
            self.phase = None
            for axis in (1,2):self.app._release_stopped_axis(axis)
            self.status.set(f'本次完成：{getattr(self,"run_steps",self.step)}段，耗时{time.monotonic()-self.run_started:.3f}s；已停稳，等待下一次指令'); return
        self.step += 1; target = self.points.popleft(); now = time.monotonic()
        self.run_steps=getattr(self,'run_steps',0)+1
        self.segment = dict(target=target, began=now, axes={1:{},2:{}})
        self.phase = 'running'
        self.deadline = now + (max(abs(t-p.angle_deg) for t,p in zip(target,positions))/(6*self.speed_rpm)+20 if positions is not None else 3600)
        if positions is not None:
            for axis,p in enumerate(positions,1):
                delta = round(target[axis-1]-p.angle_deg,3)
                if abs(delta) <= .001:self.segment['axes'][axis]['no_motion']=True
        selection=getattr(self,'run_selection',None)
        if selection:
            record,index=selection
            self.jobs.append(run_route_point(record['route_id'],record['token'],index))
        else:self.jobs.append(build_move_axes_point(target,self.speed_rpm,tolerance_deg=getattr(self,'completion_tolerance',.2)))
        self.status.set(f'第{self.step}段：两轴前往{target}；实际速度{self.speed_rpm:g}rpm')

    def record_segment(self, result):
        s = self.segment
        if s is None: return
        sends = [m['sent'] for m in s['axes'].values() if 'sent' in m]
        skew = round(max(sends)-min(sends),6) if len(sends)==2 else ''
        joint_total = round(max(m['done'] for m in s['axes'].values())-s['began'],6) if all('done' in m for m in s['axes'].values()) else ''
        for axis,m in s['axes'].items():
            sent = m.get('sent')
            def duration(end, start):
                return round(end-start,6) if end is not None and start is not None else ''
            row = dict(step=self.step,axis=axis,target=s['target'][axis-1],tol=getattr(self,'completion_tolerance',.2),
                prep=duration(sent,s['began']),ack=duration(m.get('ack'),sent),
                start=duration(m.get('motion'),sent),window=duration(m.get('window'),sent),
                settle=duration(m.get('done'),m.get('window')),total=duration(m.get('done'),s['began']),
                joint_total=joint_total,skew=skew,result='已在目标，无需运动' if m.get('no_motion') and 'done' in m else result)
            row['board_prep']=round(s['board_prep'],6) if 'board_prep' in s else ''
            row['ack_other']=round(max(0,row['ack']-s['board_prep']),6) if row['ack']!='' and 'board_prep' in s else ''
            self.rows.append(row); self.table.insert('', 'end', values=tuple(row.values()))
            self.app._log('JOINT_TIMING '+json.dumps(row,ensure_ascii=False))
        self.segment = None

    def stop(self, reason='人工停止，后续段已取消'):
        if not self.phase: return
        if self.phase == 'stopping': return
        self.record_segment(reason)
        self.points.clear(); self.jobs = deque([build_request(1,6),build_request(2,6)])
        self.phase = 'stopping'; self.zeros = 0; self.stop_reason = reason
        self.deadline = time.monotonic()+30; self.status.set(reason+'；正在停止两轴…')

    def disconnected(self, reason='连接断开，未完成段状态未知'):
        self.loaded_routes.clear()
        self.invalidate_route()
        if not self.phase: return
        self.record_segment(reason)
        # Disconnect cannot wait for responses: best-effort STOP both independent axes.
        if self.app.port:
            for axis in (1,2):
                try: self.app.port.write(build_request(axis,6))
                except Exception: pass
        self.phase = None; self.jobs.clear(); self.points.clear(); self.status.set(reason)

    def read_board_timing(self):
        if self.phase:return
        self.app._send(build_request(1,0x1B,parameter=0x544D),0x1B,'读取板端点位耗时（不运动）')

    def export(self):
        if not self.rows:
            messagebox.showinfo('尚无记录','完成至少一段计时后再导出；当前窗口没有测试记录。',parent=self.window)
            return
        path = filedialog.asksaveasfilename(parent=self.window,defaultextension='.csv',filetypes=[('CSV','*.csv')])
        if not path: return
        try:
            with open(path,'w',encoding='utf-8-sig',newline='') as f:
                writer = csv.DictWriter(f,fieldnames=('step','axis','target','tol','prep','ack','start','window','settle','total','joint_total','skew','result','board_prep','ack_other'))
                writer.writeheader(); writer.writerows(self.rows)
        except (OSError,ValueError) as exc: messagebox.showerror('导出失败',str(exc),parent=self.window)

    def close(self):
        self.stop('关闭窗口，取消后续段'); self.window.withdraw()
