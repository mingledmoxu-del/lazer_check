"""Offline trajectory points and status: no serial access or automatic motion."""
from dataclasses import dataclass
import math
import struct

STATES = ("空闲", "上传中", "已校验待启动", "轨迹运行", "终点收敛", "两轴停止确认", "完成", "故障", "已取消")
FAULTS = ("无", "人工停止", "参考/保护无效", "范围/制动余量不足", "限位触发", "驱动通信故障",
          "驱动器报警", "跟随误差超限", "调度超过1秒", "3秒心跳丢失", "未停在起点", "轨迹数据错误", "终点收敛超时", "路径推进超时")
ACTIVE_STATES = (3, 4, 5)

@dataclass(frozen=True)
class AxisPosition:
    angle_deg: float | None
    actual_counts: int
    motor_rpm: int
    flags: int
    sample_age_ms: int
    alarm_code: int
    guard_fault: int
    source: int  # 0=fresh drive reply, 1=controller cache, 2=read failure

    @property
    def usable(self):
        return self.angle_deg is not None and self.source != 2 and self.sample_age_ms <= 1000

    @property
    def point_active(self):
        return bool(self.flags & 0x20)

    @property
    def point_completed(self):
        return bool(self.flags & 0x40) and not self.point_active

def decode_axes(data):
    if len(data) != 40: raise ValueError("两轴位置数据长度错误")
    axes=[]
    for offset in (0,20):
        angle,counts,rpm,flags,age,alarm,fault,source=struct.unpack_from(">iihHIHBB",data,offset)
        if source>2: raise ValueError("位置来源字段错误")
        axes.append(AxisPosition(angle/1000 if flags&1 and source!=2 else None,
                                 counts,rpm,flags,age,alarm,fault,source))
    return tuple(axes)

def return_to_start_points(current, target, bounds, *, max_speed=1.5, max_accel=3):
    """Joint-space cubic return; caller must confirm the swept mechanical path.
    Does not reset references, enlarge bounds, upload or start any motion.
    """
    if len(current)!=2 or len(target)!=2 or len(bounds)!=2:
        raise ValueError("需要两轴当前位置、目标及软范围")
    if not all(math.isfinite(x) and x>0 for x in (max_speed,max_accel)):
        raise ValueError("恢复速度和加速度须为正数")
    distance=0
    for axis,(now,end,bound) in enumerate(zip(current,target,bounds)):
        if not all(math.isfinite(x) for x in (now,end)):
            raise ValueError("当前位置或首点无效")
        if bound is None:
            if axis==0:raise ValueError("电机1必须保留固定范围")
            distance=max(distance,abs(end-now));continue
        lo,hi=bound
        if not all(math.isfinite(x) for x in (lo,hi)) or not lo+1<=now<=hi-1 or not lo+1<=end<=hi-1:
            raise ValueError("当前位置或首点无效／距软边界不足1°，不能自动规划返回")
        distance=max(distance,abs(end-now))
    duration=max(2,2*distance/max_speed,math.sqrt(12*distance/max_accel))
    # Equal millisecond intervals; conservative speed/accel margins for rounding.
    interval=math.ceil(duration*1000/16)
    points=[]
    for n in range(17):
        u=n/16;s=3*u*u-2*u*u*u
        points.append(TrajectoryPoint(n*interval/1000,
            current[0]+(target[0]-current[0])*s,current[1]+(target[1]-current[1])*s))
    return validate_points(points)

@dataclass(frozen=True)
class TrajectoryPoint:
    time_s: float
    axis1_deg: float
    axis2_deg: float

@dataclass(frozen=True)
class TrajectoryStatus:
    session: int
    state: int
    fault: int
    point_count: int
    uploaded_count: int
    elapsed_s: float
    duration_s: float
    max_gap_ms: int
    last_cycle_ms: int
    axis1_deg: float
    axis2_deg: float
    error1_deg: float
    error2_deg: float

    @property
    def active(self): return self.state in ACTIVE_STATES

    @property
    def feedback_available(self):
        return (self.state in (3, 4) or self.max_gap_ms > 0) and self.fault not in (2, 5)

    @property
    def state_text(self): return STATES[self.state] if self.state < len(STATES) else str(self.state)

    @property
    def fault_text(self): return FAULTS[self.fault] if self.fault < len(FAULTS) else str(self.fault)

def decode_status(data: bytes) -> TrajectoryStatus:
    if len(data) != 40: raise ValueError("轨迹状态长度错误")
    session, state, fault, count, uploaded, elapsed, duration, gap, cycle, a1, a2, e1, e2 = struct.unpack(">HBBHHIIIIiiii", data)
    if state > 8 or fault > 13 or count > 128 or uploaded > count:
        raise ValueError("轨迹状态字段错误")
    return TrajectoryStatus(session, state, fault, count, uploaded, elapsed/1000, duration/1000,
                            gap, cycle, a1/1000, a2/1000, e1/1000, e2/1000)

def validate_points(points):
    """Normalize to millisecond/millidegree resolution BEFORE any hardware writes."""
    points = tuple(points)
    if not 2 <= len(points) <= 128: raise ValueError("轨迹需2～128点")
    result = []
    previous = -50
    for p in points:
        if not all(math.isfinite(x) for x in (p.time_s, p.axis1_deg, p.axis2_deg)):
            raise ValueError("轨迹点必须为有限数字")
        ms = round(p.time_s*1000)
        if not 0 <= ms <= 600000 or ms-previous < 50:
            raise ValueError("轨迹时间需递增至少50ms，且不超过600秒")
        a1, a2 = round(p.axis1_deg*1000), round(p.axis2_deg*1000)
        if any(not -(2**31) <= a < 2**31 for a in (a1, a2)): raise ValueError("轨迹角度溢出")
        result.append(TrajectoryPoint(ms/1000, a1/1000, a2/1000));previous = ms
    if result[0].time_s != 0 or result[-1].time_s < 1:
        raise ValueError("首点时间必须0，持续至少1秒")
    return tuple(result)

def joint_ellipse(*, center1=0.0, center2=0.0, amplitude1=5.0, amplitude2=5.0,
                  period_s=5.0, cycles=1, samples_per_cycle=32):
    """Joint-space sine/cosine EXAMPLE, not calibrated Cartesian/optical circle."""
    if not all(math.isfinite(x) for x in (center1, center2, amplitude1, amplitude2, period_s)):
        raise ValueError("示例参数必须为有限数字")
    if amplitude1 < 0 or amplitude2 < 0 or period_s <= 0 or type(cycles) is not int or cycles < 1:
        raise ValueError("幅度非负，周期和圈数须为正")
    if type(samples_per_cycle) is not int or samples_per_cycle < 4 or cycles*samples_per_cycle+1 > 128:
        raise ValueError("采样数至少4且总点数不能超过128")
    points = (TrajectoryPoint(n*period_s/samples_per_cycle,
              center1+amplitude1*math.cos(2*math.pi*n/samples_per_cycle),
              center2+amplitude2*math.sin(2*math.pi*n/samples_per_cycle))
              for n in range(cycles*samples_per_cycle+1))
    return validate_points(points)


def business_trajectory(points, *, max_speed=1.5, max_accel=3):
    """Retain path geometry; retime to the supplied output-axis limits."""
    if (not all(math.isfinite(x) for x in (max_speed,max_accel)) or
            not 0.001<=max_speed<=60 or not 0.001<=max_accel<=120):
        raise ValueError("轨迹速度需在0.001～60°/秒，加速度0.001～120°/秒²")
    points=validate_points(points);factor=1.0
    for field in ("axis1_deg","axis2_deg"):
        previous=None;previous_dt=0
        for a,b in zip(points,points[1:]):
            dt=b.time_s-a.time_s;velocity=(getattr(b,field)-getattr(a,field))/dt
            factor=max(factor,abs(velocity)/max_speed)
            if previous is not None:
                accel=abs(velocity-previous)*2/(dt+previous_dt)
                factor=max(factor,math.sqrt(accel/max_accel))
            previous=velocity;previous_dt=dt
    if factor<=1:return points
    factor*=1.01
    return validate_points(TrajectoryPoint(p.time_s*factor,p.axis1_deg,p.axis2_deg) for p in points)


def business_loop(current):
    """Joint-space loop from/to current pose; not a calibrated Cartesian circle."""
    if len(current)!=2 or not all(math.isfinite(x) for x in current):raise ValueError("需要两轴实际位置")
    return business_trajectory(TrajectoryPoint(n*20/32,
        current[0]+5*math.sin(2*math.pi*n/32),
        current[1]+5*(1-math.cos(2*math.pi*n/32))) for n in range(33))


def quick_joint_test(centers, bounds):
    """One slow, in-phase excursion from/to current pose; never changes references/ranges."""
    if len(centers)!=2 or len(bounds)!=2:
        raise ValueError("需要两轴当前位置及软范围")
    clearance=2.0
    room=[]
    for axis,(center,bound) in enumerate(zip(centers,bounds),1):
        if bound is None:
            if axis==1 or not math.isfinite(center):raise ValueError("电机1必须保留范围，位置须有效")
            room.append(2.0);continue
        lo,hi=bound
        if not all(math.isfinite(x) for x in (center,lo,hi)) or lo>=hi or not lo<=center<=hi:
            raise ValueError(f"电机{axis}当前位置／软范围无效")
        room.append(min(center-lo,hi-center)-clearance)
    amplitude=math.floor(min(2.0,*room)*1000)/1000
    if amplitude<1.0:
        raise ValueError("当前位置距软边界不足3°，无法保留余量做±1°小幅测试；请先人工移动到已确认安全的位置，不要扩大范围绕过检查")
    points=validate_points(TrajectoryPoint(n*20/32,
        centers[0]+amplitude*math.sin(2*math.pi*n/32),
        centers[1]+amplitude*math.sin(2*math.pi*n/32)) for n in range(33))
    return points,amplitude
