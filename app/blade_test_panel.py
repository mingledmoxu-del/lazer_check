"""Bench test: simulated blade clock, real motors, no acquisition API calls."""
import csv
import json
import secrets
import struct
import time
from collections import deque
from dataclasses import asdict
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

from blade_simulation import BladePlan, encoder_stats
from joint_timing_panel import JointTimingPanel
from point_bank import upload_frames, decode_route
from protocol import build_request, CMD_READ_PA
from trajectory import decode_axes


class BladeTestPanel(JointTimingPanel):
    EXTRA = ('planned_send','sent_at','send_late','blade_at','margin','verdict','encoder_pp','visual','note')
    BASE = ('step','axis','target','tol','prep','ack','start','window','settle','total','joint_total','skew','result','board_prep','ack_other')

    def __init__(self, app):
        self.app=app; self.phase=None; self.mode=None; self.segment=None
        self.rows=[]; self.samples=[]; self.profiles={}; self.loaded_routes={}
        self.jobs=deque(); self.points=deque(); self.loaded_route=None
        self.awaiting=None; self.prepared=None; self.latest=None; self.latest_at=0
        self.metadata={}; self.sample_rows={}; self.active_leg=None
        self.window=tk.Toplevel(app.root); self.window.title('模拟叶片经过 / 三点四点跟随测试 v3.9.5')
        self.window.geometry('1400x850'); self.window.protocol('WM_DELETE_WINDOW', self.close)
        outer=ttk.Frame(self.window,padding=10); outer.pack(fill='both',expand=True)
        ttk.Label(outer,text='不启动激光采集：模拟目标叶片经过时刻，实际驱动两轴。先准备并定位起点，再点击开始设置 t0。').pack(anchor='w')
        ttk.Label(outer,text='按绝对时间发下一点；未及时取得停稳反馈则停止，不补发追赶。编码器波动不能代表机头或光斑抖动。').pack(anchor='w')
        defaults=asdict(BladePlan()); self.fields={}
        labels=(('count','点数'),('radius_index','P圈1～5'),('period_s','模拟每转秒'),('mode','延迟模式A/B'),
                ('speed_rpm','实际轴rpm'),('cycles','圈数'),('distance_m','距离m'),('height_m','中心高差m'),
                ('pitch_origin','俯仰原点°'),('yaw_origin','水平原点°'),('tolerance_deg','完成容差°'),('hold_s','末段观察秒'))
        grid=ttk.Frame(outer); grid.pack(fill='x',pady=6)
        for n,(key,label) in enumerate(labels):
            var=tk.StringVar(value=str(defaults[key])); self.fields[key]=var
            ttk.Label(grid,text=label).grid(row=n//6*2,column=n%6,sticky='w',padx=6)
            if key in ('count','radius_index','mode'):
                values={'count':(3,4),'radius_index':(1,2,3,4,5),'mode':('A','B')}[key]
                widget=ttk.Combobox(grid,textvariable=var,values=values,width=13,state='readonly')
            else: widget=ttk.Entry(grid,textvariable=var,width=15)
            widget.grid(row=n//6*2+1,column=n%6,sticky='w',padx=6,pady=3)
        self.confirmed=tk.BooleanVar(value=False)
        ttk.Checkbutton(outer,variable=self.confirmed,text='已核对真实参考与几何原点、整圈路径及负载；默认原点只是测试表的待核验值。准备会覆盖板端库6。').pack(anchor='w')
        buttons=ttk.Frame(outer); buttons.pack(fill='x',pady=8)
        for label,action in (('预览路线/时序',self.preview),('① 预载并定位起点（运动）',self.prepare),
                             ('② 开始模拟 t0',self.start),('停止两轴',self.stop),('导出CSV+JSON',self.export)):
            ttk.Button(buttons,text=label,command=action).pack(side='left',padx=4)
        self.status=tk.StringVar(value='尚未准备；默认距离2.5m、每转5秒。可按实测修改。')
        ttk.Label(outer,textvariable=self.status,wraplength=1300).pack(anchor='w',pady=4)
        self.preview_text=tk.StringVar()
        ttk.Label(outer,textvariable=self.preview_text,wraplength=1300).pack(anchor='w',pady=4)
        self.summary=tk.StringVar(value='统计尚无记录。到位窗口±0.5°；完成须两轴板端确认停稳且在终点容差内。')
        ttk.Label(outer,textvariable=self.summary,wraplength=1300).pack(anchor='w',pady=4)
        columns=self.BASE+self.EXTRA
        self.table=ttk.Treeview(outer,columns=columns,show='headings',height=18)
        names=('段','轴','目标°','容差°','准备s','应答s','启动s','到位s','到位→停稳s','轴总s','联动总s','下发间隔s','执行结果','板端准备s','应答其余s',
               '计划发令t','实际发令t','发令迟到s','模拟叶片t','赶点余量s','赶点结果','编码器峰峰°','目测抖动','备注')
        for key,label in zip(columns,names):
            self.table.heading(key,text=label); self.table.column(key,width=100)
        self.table.pack(fill='both',expand=True)
        scroll=ttk.Scrollbar(outer,orient='horizontal',command=self.table.xview); scroll.pack(fill='x'); self.table.configure(xscrollcommand=scroll.set)
        edit=ttk.Frame(outer); edit.pack(fill='x',pady=7)
        self.visual=tk.StringVar(value='未观察'); self.note=tk.StringVar()
        ttk.Label(edit,text='选中某段某轴，填写停止瞬间目测：').pack(side='left')
        ttk.Combobox(edit,textvariable=self.visual,values=('未观察','无明显','轻微','明显'),width=10,state='readonly').pack(side='left')
        ttk.Entry(edit,textvariable=self.note,width=50).pack(side='left',padx=5)
        ttk.Button(edit,text='保存所选行观察',command=self.annotate).pack(side='left')
        self.preview()

    def config(self):
        values={k:(v.get() if k=='mode' else int(v.get()) if k in ('count','radius_index','cycles') else float(v.get())) for k,v in self.fields.items()}
        return BladePlan(**values)

    def preview(self):
        try:
            plan=self.config()
            points=' → '.join(f'{n}:({a:g}°, {b:g}°)' for n,(a,b) in enumerate(plan.points))+' → 0'
            self.preview_text.set(f'{points}\n模拟转子{60/plan.period_s:g}rpm；点间隔{plan.interval_s:.3f}s；估计经过宽度{plan.passage_s:.3f}s；发令延迟{plan.delay_s:.3f}s；每段运动预算{plan.interval_s-plan.delay_s:.3f}s。A=经过宽度+30ms；B另加100ms。')
            return plan
        except ValueError as exc:
            self.status.set(str(exc)); return None

    def prepare(self):
        if self.phase: return
        plan=self.preview()
        if plan is None: return
        if not self.confirmed.get():
            messagebox.showwarning('核对参考','请先核对并勾选真实参考、几何原点和可运动路径。',parent=self.window); return
        debug=getattr(self.app,'debug_panel',None)
        if debug and (debug.jobs or debug.after_stop is not None or debug.goto_target is not None or debug.raised_request is not None): return
        if not self.app.business_panel.available(): return
        if not messagebox.askyesno('准备将实际运动',f'覆盖库6，上传 P{plan.radius_index} 的{plan.count}个点，然后两轴定位到{plan.points[0]}。\n实际轴速度{plan.speed_rpm:g}rpm；不修改参考及PA参数。',parent=self.window): return
        # A modal dialog can run Stop/disconnect callbacks; check again.
        if self.phase or not self.app.business_panel.available(): return
        self.prepared=plan; self.mode='preparing'; self.profiles={}; self.samples=[]; self.sample_rows={}
        self.metadata=dict(app_version='3.9.5',firmware_required='3.9.4',created_at=datetime.now().astimezone().isoformat(),
                           config=asdict(plan),points=plan.points,clock='time.monotonic',laser_acquisition=False,
                           passage_width='estimated, scaled linearly with rotor period',events=[])
        # No stale UI selection can be reused after this bank is overwritten.
        old=getattr(self.app,'joint_timing_panel',None)
        if old:
            old.loaded_routes.pop(6,None)
            if old.loaded_route and old.loaded_route['route_id']==6: old.invalidate_route()
        self.token=secrets.randbits(32) or 1
        self.completion_tolerance=plan.tolerance_deg
        self.run_selection=None
        self._begin_run([plan.points[0]],plan.speed_rpm)

    def begin_segment(self, positions):
        if self.mode=='preparing':
            self.latest=positions; self.latest_at=time.monotonic()
            self.jobs=deque(build_request(axis,CMD_READ_PA,parameter=pa) for axis in (1,2) for pa in (40,41,42))
            self.jobs.extend(upload_frames(self.prepared.points,slot=6,token=self.token,
                speed_rpm=self.prepared.speed_rpm,tolerance_deg=self.prepared.tolerance_deg))
            self.phase='uploading'; self.deadline=time.monotonic()+60
        elif self.mode=='positioning':
            self.mode='armed'; self.phase='armed'; self.deadline=time.monotonic()+3600
            self.status.set('起点已停稳；点击②开始模拟设置 t0，当前尚未开始模拟叶片计时。')
        elif self.mode=='test':
            self.phase='waiting'; self.deadline=self.t0+self.prepared.cycles*self.prepared.period_s+self.prepared.hold_s+5
            if getattr(self,'missed',False): self.stop('未在模拟叶片到达前取得停稳反馈，取消后续段')
        else: super().begin_segment(positions)

    def start(self):
        if self.phase!='armed':
            messagebox.showinfo('先准备','请先预载并定位起点，待起点停稳后开始。',parent=self.window); return
        plan=self.preview()
        if plan!=self.prepared or not self.confirmed.get():
            self.status.set('参数/核对状态已改变，请停止并重新准备。'); return
        now=time.monotonic()
        if self.latest is None or now-self.latest_at>.25 or any(
            p.source!=0 or not p.usable or p.motor_rpm or p.flags&0x30 or p.alarm_code or p.guard_fault or
            abs(p.angle_deg-target)>plan.tolerance_deg for p,target in zip(self.latest,plan.points[0])):
            self.status.set('尚无新鲜起点停稳反馈，请等待刷新；若位置改变请重新准备。'); return
        self.mode='test'; self.t0=now; self.run_started=now; self.next_leg=0; self.step=0; self.rows=[]
        self.samples=[]; self.sample_rows={}; self.active_leg=None; self.missed=False
        self.metadata['t0_monotonic']=now; self.metadata['events'].append(dict(event='start',t=0))
        self.table.delete(*self.table.get_children()); self.begin_segment(self.latest)

    def service(self):
        if not self.phase: return
        now=time.monotonic()
        if self.mode=='test' and self.phase!='stopping':
            elapsed=now-self.t0
            if self.segment is not None and elapsed>=self.active_leg['blade_s']:
                self.missed=True; self.stop('模拟叶片已到，尚未取得两轴完成反馈；取消后续段'); return
            if self.phase=='waiting':
                if self.next_leg>=self.prepared.count*self.prepared.cycles:
                    if elapsed>=self.prepared.cycles*self.prepared.period_s+self.prepared.hold_s:
                        self.metadata['elapsed_s']=elapsed; self.phase=None
                        for axis in (1,2): self.app._release_stopped_axis(axis)
                        self.status.set(f'全部完成；模拟{self.prepared.cycles}圈；含末段观察共{elapsed:.3f}s。未启动激光采集。')
                        self.render(); return
                else:
                    leg=self.prepared.leg(self.next_leg)
                    if elapsed+1e-9>=leg['send_s']:
                        # Desktop dispatch is best effort: never issue a burst of late points.
                        if elapsed-leg['send_s']>.100:
                            self.metadata['failure']=dict(reason='发令调度迟到超过100ms，未发该点',leg=leg,observed_s=elapsed)
                            self.stop('发令调度迟到超过100ms，取消后续段'); return
                        if not self.app.pending and not self.app.deferred_send:
                            self.active_leg=leg; self.next_leg+=1; self.step+=1
                            self.segment=dict(target=leg['target'],began=now,axes={1:{},2:{}})
                            from point_bank import run_route_point
                            self.jobs.append(run_route_point(6,self.token,leg['point']))
                            self.phase='running'
            self.status.set(f'模拟t={elapsed:.3f}s / {self.prepared.cycles*self.prepared.period_s:.3f}s；已发{self.next_leg}段，{self.phase}；未采集激光')
        super().service()

    def handle(self, packet):
        if not self.phase or (packet.device_id,packet.command)!=self.awaiting: return False
        now=time.monotonic()
        if self.phase=='uploading' and packet.command in (CMD_READ_PA,0x40,0x41,0x42):
            self.awaiting=None
            if not packet.ok: self.stop('准备失败：'+packet.status_text); return True
            if packet.command==CMD_READ_PA:
                if len(packet.data)!=2: self.stop('PA反馈长度错误'); return True
                self.profiles[f'axis{packet.device_id}_PA{packet.parameter}']=struct.unpack('>H',packet.data)[0]
            else:
                try: state=decode_route(packet.data)
                except ValueError as exc: self.stop(str(exc)); return True
                if state['token']!=self.token or state['count']!=self.prepared.count:
                    self.stop('库6预载回读不匹配'); return True
                if packet.command==0x42:
                    if not state['ready']: self.stop('预载未就绪'); return True
                    self.metadata['board_bank_settings']=state
                    self.metadata['profiles']=self.profiles.copy()
                    self.metadata['prepare_upload_s']=now-self.run_started
                    self.mode='initial_move'; self.run_selection=(dict(route_id=6,token=self.token),0)
                    super().begin_segment(None)
                    self.mode='positioning'
            return True
        if packet.command==0x37 and packet.ok:
            try: positions=decode_axes(packet.data)
            except ValueError: return super().handle(packet)
            self.latest=positions; self.latest_at=now
            if self.phase in ('armed','waiting'):
                self.awaiting=None
                if any(not p.usable or p.sample_age_ms>200 or p.alarm_code or p.guard_fault or p.motor_rpm or p.flags&0x30 for p in positions):
                    self.stop('等候期间参考/反馈无效或发生非预期运动'); return True
                self.capture(positions,now)
                target=self.active_leg['target'] if self.mode=='test' and self.active_leg else self.prepared.points[0]
                if any(abs(p.angle_deg-a)>self.completion_tolerance+1e-6 for p,a in zip(positions,target)):
                    if self.mode=='test':
                        self.metadata['failure']=dict(reason='停稳等候期间离开目标容差',observed_s=now-self.t0)
                        for row in self.rows[-2:]: row['verdict']='停稳后离开目标容差'
                        self.render()
                    self.stop('停稳等候期间离开目标容差，取消后续段'); return True
                return True
            if self.mode=='test' and self.phase=='running':
                # An earlier completed axis must still prove its pose when the other finishes.
                for axis,p in enumerate(positions,1):
                    m=self.segment['axes'][axis]
                    valid=(p.usable and p.source==0 and p.sample_age_ms<=200 and p.motor_rpm==0 and
                           bool(p.flags&0x40) and not p.flags&0x30 and
                           abs(p.angle_deg-self.segment['target'][axis-1])<=self.completion_tolerance)
                    if not valid: m.pop('done',None)
                self.capture(positions,now)
        return super().handle(packet)

    def capture(self, positions, now):
        if self.mode!='test': return
        # Preserve moving and idle observations, but stats use direct post-completion samples only.
        for axis,p in enumerate(positions,1):
            target=self.active_leg['target'][axis-1] if self.active_leg else self.prepared.points[0][axis-1]
            row=dict(step=self.step,axis=axis,observed_s=round(now-self.t0,6),angle_deg=p.angle_deg,target_deg=target,
                     rpm=p.motor_rpm,age_ms=p.sample_age_ms,source=p.source,flags=p.flags)
            self.samples.append(row)
            if len(self.samples)>200000:
                self.samples.pop(0)
                self.metadata['discarded_old_samples']=self.metadata.get('discarded_old_samples',0)+1
            key=(self.step,axis)
            if key in self.sample_rows and p.source==0 and p.sample_age_ms<=200:
                self.sample_rows[key].append(row)
                if len(self.sample_rows[key])>2000: self.sample_rows[key].pop(0)

    def record_segment(self, result):
        segment=self.segment
        if segment is None: return
        before=len(self.rows)
        super().record_segment(result)
        if self.mode!='test':
            self.metadata['initial_move']=self.rows[before:]; self.rows=[]
            self.table.delete(*self.table.get_children()); return
        leg=self.active_leg
        for row in self.rows[before:]:
            axis=row['axis']; m=segment['axes'][axis]; sent=m.get('sent')
            done=m.get('done'); margin=None if done is None else self.t0+leg['blade_s']-done
            passed=margin is not None and margin>=0
            if not passed: self.missed=True
            row.update(planned_send=leg['send_s'],sent_at=None if sent is None else sent-self.t0,
                       send_late=None if sent is None else sent-self.t0-leg['send_s'],blade_at=leg['blade_s'],
                       margin=margin,verdict='赶上模拟叶片' if passed else '未及时取得完成反馈',encoder_pp=None,visual='未观察',note='')
            self.sample_rows[(self.step,axis)]=[]
            self.app._log('BLADE_TIMING '+json.dumps(row,ensure_ascii=False))
        self.render()

    def render(self):
        selected=self.table.selection()
        self.table.delete(*self.table.get_children())
        for n,row in enumerate(self.rows):
            stats=encoder_stats(self.sample_rows.get((row['step'],row['axis']),[]))
            row['encoder_pp']=stats['pp_deg']; row['encoder_stats']=stats
            self.table.insert('','end',iid=str(n),values=tuple(round(row[k],6) if isinstance(row.get(k),float) else row.get(k,'') for k in self.BASE+self.EXTRA))
        for item in selected:
            if self.table.exists(item): self.table.selection_add(item)
        times=[r['joint_total'] for r in self.rows[::2] if isinstance(r.get('joint_total'),(float,int))]
        good=sum(all(r['verdict']=='赶上模拟叶片' for r in self.rows[n:n+2]) for n in range(0,len(self.rows),2))
        observed=sum(r['visual']!='未观察' for r in self.rows)
        ratings={label:sum(r['visual']==label for r in self.rows) for label in ('未观察','无明显','轻微','明显')}
        self.metadata['summary']=dict(legs=len(self.rows)//2,on_time=good,visual_ratings=ratings,
            mean_joint_s=sum(times)/len(times) if times else None,max_joint_s=max(times) if times else None)
        self.summary.set(f'记录{len(self.rows)//2}段；赶上模拟叶片{good}段；联动耗时均值{sum(times)/len(times):.3f}s，最大{max(times):.3f}s；目测已填{observed}轴次：轻微{ratings["轻微"]}、明显{ratings["明显"]}。' if times else f'记录{len(self.rows)//2}段；尚无完成耗时。')

    def annotate(self):
        for item in self.table.selection():
            self.rows[int(item)].update(visual=self.visual.get(),note=self.note.get())
        self.render()

    def stop(self, reason='人工停止，后续段已取消'):
        if self.phase and self.phase!='stopping':
            self.metadata['events'].append(dict(event='stop',reason=reason,monotonic=time.monotonic()))
        super().stop(reason)

    def invalidate_route(self):
        self.loaded_route=None; self.prepared=None

    def disconnected(self, reason='连接断开，未完成段状态未知'):
        super().disconnected(reason)
        self.prepared=None

    def export(self):
        if not self.rows and not self.metadata:
            messagebox.showinfo('尚无记录','先运行一次测试。',parent=self.window); return
        path=filedialog.asksaveasfilename(parent=self.window,defaultextension='.csv',filetypes=[('CSV','*.csv')])
        if not path: return
        self.render()
        try:
            with open(path,'w',encoding='utf-8-sig',newline='') as f:
                writer=csv.DictWriter(f,fieldnames=self.BASE+self.EXTRA,extrasaction='ignore'); writer.writeheader(); writer.writerows(self.rows)
            Path(path).with_suffix('.json').write_text(json.dumps(dict(metadata=self.metadata,rows=self.rows,samples=self.samples,
                limitation='Encoder feedback is not optical/mechanical vibration measurement; blade timing is simulated.'),ensure_ascii=False,indent=2),encoding='utf-8')
        except (OSError,ValueError) as exc: messagebox.showerror('导出失败',str(exc),parent=self.window)
