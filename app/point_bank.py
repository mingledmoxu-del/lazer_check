"""Fixed, event-triggered two-axis waypoints; RAM banks on v3.9.1 H7."""
import math
import struct

try:
    from .protocol import build_request
except ImportError:
    from protocol import build_request

PRESETS = {
    '四点测量（含起终原点）': ((0,0),(6.9,0),(16.1,-9.7),(24.8,0),(16.1,9.7),(0,0)),
    '10°往返（含起终原点）': ((0,0),(10,10),(0,0),(10,10),(0,0)),
}

def _identity(slot,token):
    if isinstance(slot,bool) or not isinstance(slot,int) or not 1<=slot<=6:raise ValueError('点位库编号须为1～6（库5、6需要v3.9.4固件）')
    if isinstance(token,bool) or not isinstance(token,int) or not 1<=token<=0xFFFFFFFF:raise ValueError('预载令牌须为非零uint32')

def begin_route(slot,token,points,speed_rpm,*,tolerance_deg=.2):
    _identity(slot,token)
    if not 1<=len(points)<=16:raise ValueError('每套点位库须包含1～16个点')
    if not math.isfinite(speed_rpm) or not .01<=speed_rpm<=100:raise ValueError('实际输出轴速度须为0.01～100rpm')
    if not math.isfinite(tolerance_deg) or not .01<=tolerance_deg<=.5:raise ValueError('终点容差须为0.01～0.5°')
    return build_request(1,0x40,struct.pack('>IHHB',token,int(math.floor(speed_rpm*100+.5)),int(math.floor(tolerance_deg*1000+.5)),len(points)),parameter=slot)

def route_point(slot,token,index,angles):
    _identity(slot,token)
    if isinstance(index,bool) or not isinstance(index,int) or not 0<=index<16:raise ValueError('点编号须为0～15')
    if len(angles)!=2 or not all(math.isfinite(v) for v in angles):raise ValueError('每个点需要两个有限角度')
    target=[round(v*1000) for v in angles]
    if any(not -(2**31)<=v<2**31 for v in target):raise ValueError('目标超出协议范围')
    return build_request(1,0x41,struct.pack('>IBii',token,index,*target),parameter=slot)

def seal_route(slot,token):
    _identity(slot,token)
    return build_request(1,0x42,struct.pack('>I',token),parameter=slot)

def run_route_point(slot,token,index):
    _identity(slot,token)
    if isinstance(index,bool) or not isinstance(index,int) or not 0<=index<16:raise ValueError('点编号须为0～15')
    return build_request(1,0x43,struct.pack('>IB',token,index),parameter=slot)

def route_status(slot):
    _identity(slot,1)
    return build_request(1,0x44,parameter=slot)

def decode_route(data):
    if len(data)!=11:raise ValueError('预载状态长度错误')
    token,speed,tolerance,count,loaded,ready=struct.unpack('>IHHBBB',data)
    if count>16 or loaded>count or ready>1:raise ValueError('预载状态字段错误')
    if ready and (not token or not speed or not count or loaded!=count or not 10<=tolerance<=500):raise ValueError('预载就绪状态无效')
    return dict(token=token,speed_rpm=speed/100,tolerance_deg=tolerance/1000,count=count,loaded=loaded,ready=bool(ready))

def upload_frames(points,*,slot,token,speed_rpm,tolerance_deg=.2):
    # Validate the entire upload locally before sending even BEGIN.
    frames=[begin_route(slot,token,points,speed_rpm,tolerance_deg=tolerance_deg)]
    frames.extend(route_point(slot,token,n,p) for n,p in enumerate(points))
    frames.append(seal_route(slot,token))
    return frames
