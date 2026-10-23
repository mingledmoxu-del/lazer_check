"""Host-observed motion milestones; never sends traffic or controls motion."""
import datetime
import json
import logging
from pathlib import Path
import struct
import time


class MotionTiming:
    def __init__(self, path=None, emit=None, clock=time.monotonic):
        self.path=Path(path) if path else None
        self.emit=emit;self.clock=clock;self.axes={};self.sequence=0
        self.last_angles={};self.last_counts={}

    def _event(self, axis, event, **detail):
        trace=self.axes.get(axis)
        if trace is None:return
        now=self.clock()
        row=dict(timestamp=datetime.datetime.now().astimezone().isoformat(timespec='milliseconds'),
                 trace_id=trace['id'],axis=axis,operation=trace['operation'],event=event,
                 elapsed_ms=round((now-trace['call'])*1000,3),finish_tolerance_deg=trace.get('finish_tolerance_deg'),**detail)
        for phase in ('sent','ack'):
            if phase in trace:row[f'from_{phase}_ms']=round((now-trace[phase])*1000,3)
        if 'sent' in trace:row['call_to_send_ms']=round((trace['sent']-trace['call'])*1000,3)
        if 'ack' in trace:row['tx_ack_ms']=round((trace['ack']-trace['sent'])*1000,3)
        for phase in ('motion_observed','angle_window'):
            if phase in trace:row[f'from_{phase}_ms']=round((now-trace[phase])*1000,3)
        logging.getLogger('motor.motion_timing').info('%s',json.dumps(row,ensure_ascii=False))
        if self.path:
            try:
                self.path.parent.mkdir(parents=True,exist_ok=True)
                with self.path.open('a',encoding='utf-8') as file:
                    file.write(json.dumps(row,ensure_ascii=False)+'\n')
            except OSError:
                logging.getLogger(__name__).warning('Motion timing file unavailable: %s',self.path)
                self.path=None
        if self.emit:
            try:self.emit(row)
            except Exception:logging.getLogger(__name__).warning('Motion timing callback failed',exc_info=True)

    def begin(self, axis, operation, target=None, *, finish_tolerance_deg=None):
        if axis in self.axes:self.finish(axis,'replaced')
        self.sequence+=1
        self.axes[axis]=dict(id=f'{time.time_ns()}-{self.sequence}',operation=operation,
                             call=self.clock(),target=target,finish_tolerance_deg=finish_tolerance_deg,seen=set())
        self._event(axis,'call_start',target_deg=target)

    def sent(self, frame):
        axis,command=frame[3:5]
        if command in (0x1A,0x43):
            if command==0x1A:
                speed,first,second=struct.unpack('>Hii',frame[7:17]);targets=(first/1000,second/1000)
            else:
                speed=None;targets=tuple(self.axes.get(i,{}).get('target') for i in (1,2))
            for index,target in enumerate(targets,1):
                trace=self.axes.get(index)
                if trace is None or 'sent' in trace:self.begin(index,'move_axes_to' if command==0x1A else 'run_preloaded_point',target)
            sent_at=self.clock()
            for index,target in enumerate(targets,1):
                trace=self.axes[index];trace['sent']=sent_at;trace['command']=command
                if command==0x1A and len(frame)>=21:trace['finish_tolerance_deg']=struct.unpack('>H',frame[17:19])[0]/1000
                trace['target']=target;trace['baseline']=self.last_counts.get(index)
                self._event(index,'motion_sent',command=f'0x{command:02X}',target_deg=target,speed_rpm=speed/100 if speed is not None else None)
            return
        if command in (0x19,0x14,0x28,0x23,0x17):
            trace=self.axes.get(axis)
            if trace is None or 'sent' in trace:
                self.begin(axis,'wire_motion');trace=self.axes[axis]
            if command in (0x19,0x14) and len(frame)>=15:
                speed,angle=struct.unpack('>Hi',frame[7:13])
                if trace['target'] is None and axis in self.last_angles:
                    trace['target']=self.last_angles[axis]+angle/1000
                detail=dict(relative_deg=angle/1000,speed_rpm=speed/100)
            else:detail={}
            trace['sent']=self.clock();trace['command']=command
            trace['baseline']=self.last_counts.get(axis)
            self._event(axis,'motion_sent',command=f'0x{command:02X}',target_deg=trace['target'],**detail)
        elif command==6:
            if axis not in self.axes:self.begin(axis,'stop')
            trace=self.axes[axis]
            trace['stop_sent']=self.clock()
            if 'sent' in trace:trace['stopping']=True
            self._event(axis,'stop_sent')

    def received(self, packet):
        axis,command=packet.device_id,packet.command
        if command in (0x1A,0x43):
            received_at=self.clock()
            for index in (1,2):
                trace=self.axes.get(index)
                if trace and trace.get('command')==command:
                    if not packet.ok:self.finish(index,'rejected',command=command,status=packet.status)
                    elif 'ack' not in trace:
                        trace['ack']=received_at;self._event(index,'motion_ack',board_prepare_ms=list(struct.unpack('>II',packet.data)) if len(packet.data)==8 else None)
            return
        trace=self.axes.get(axis)
        if not packet.ok:
            if trace and command in (6,0x19,0x14,0x28,0x23,0x17,0x20,0x37):
                self.finish(axis,'rejected',command=command,status=packet.status)
            return
        if trace and command==trace.get('command') and 'ack' not in trace:
            trace['ack']=self.clock();self._event(axis,'motion_ack')
        if trace and command==6 and 'stop_sent' in trace:
            self._event(axis,'stop_ack',stop_rtt_ms=round((self.clock()-trace['stop_sent'])*1000,3))
        try:
            if command==0x37 and len(packet.data)==40:
                for i in (0,1):
                    angle,counts,rpm,flags,age,alarm,fault,source=struct.unpack('>iihHIHBB',packet.data[i*20:(i+1)*20])
                    if source!=2 and age<=1000:
                        self.observe(i+1,angle/1000 if flags&1 else None,counts,rpm,flags,
                                     fault or alarm,sample_age_ms=age,source=source)
            elif command==0x20 and len(packet.data)==34:
                counts,_,_,_,_,_,angle,flags,_,_,phase,fault,_,rpm=struct.unpack('>iiiHHHiHBBBBih',packet.data)
                self.observe(axis,angle/1000 if flags&1 else None,counts,rpm,flags,fault,home_phase=phase)
            elif command==7 and len(packet.data)>=4:
                self.observe(axis,None,None,struct.unpack('>h',packet.data[:2])[0],0,0)
        except (ValueError,struct.error):pass  # Real protocol handling retains validation authority.

    def observe(self, axis, angle, counts, rpm, flags, fault=0, **detail):
        if detail.get('sample_age_ms',0)>200:return
        if angle is not None:self.last_angles[axis]=angle
        if counts is not None:self.last_counts[axis]=counts
        trace=self.axes.get(axis)
        if trace is None:return
        if fault:self.finish(axis,'fault',fault=fault,**detail);return
        if 'sent' not in trace:return
        if trace.get('stopping'):return
        seen=trace['seen'];baseline=trace.get('baseline')
        if 'motion' not in seen and (rpm or (baseline is not None and counts is not None and abs(counts-baseline)>12)):
            seen.add('motion');trace['motion_observed']=self.clock()
            self._event(axis,'motion_observed',angle_deg=angle,rpm=rpm,**detail)
        if angle is not None and trace['target'] is not None and 'window' not in seen and abs(angle-trace['target'])<=0.5:
            seen.add('window');trace['angle_window']=self.clock()
            self._event(axis,'angle_in_window',angle_deg=angle,tolerance_deg=0.5,**detail)
        if flags&0x40 and not flags&0x30 and not rpm:
            self.finish(axis,'completed',basis='firmware_two_zero_samples',angle_deg=angle,**detail)
        elif trace.get('command') in (0x28,0x23) and detail.get('home_phase')==7 and flags&1 and not flags&16 and not rpm:
            self.finish(axis,'completed',basis='firmware_homing_complete',angle_deg=angle,**detail)

    def stop_confirmed(self, axis):
        trace=self.axes.get(axis)
        if trace is None:return
        self._event(axis,'stop_confirmed',basis='two_host_zero_samples')
        if trace.get('stopping'):self.finish(axis,'cancelled')
        elif trace['operation']=='stop':self.finish(axis,'stopped')

    def finish(self, axis, event, **detail):
        self._event(axis,event,**detail);self.axes.pop(axis,None)

    def finish_all(self, event, **detail):
        for axis in list(self.axes):self.finish(axis,event,**detail)
