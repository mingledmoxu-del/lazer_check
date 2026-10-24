"""Ideal pan/tilt optical circle for the existing finite trajectory protocol."""
from dataclasses import dataclass, asdict
import math

try:
    from .trajectory import TrajectoryPoint, validate_points
except ImportError:
    from trajectory import TrajectoryPoint, validate_points


@dataclass(frozen=True)
class FanCircle:
    diameter_m: float = 1.6
    radius_percent: float = 55
    distance_m: float = 2.5
    fan_height_m: float = 1.5
    laser_height_m: float = .77
    center_x_m: float = 0
    plane_yaw_deg: float = 0
    plane_tilt_deg: float = 0
    pitch_origin_deg: float = .267
    yaw_origin_deg: float = .234
    fan_rpm: float = 3
    blade_count: int = 3
    clockwise: bool = True
    start_phase_deg: float = 0
    cycles: int = 1
    segments_per_cycle: int = 64
    ramp_s: float = 2
    cycle_ms: int = 100
    max_speed_dps: float = 30
    max_accel_dps2: float = 60
    max_error_deg: float = .75
    end_tolerance_deg: float = .2

    def __post_init__(self):
        if any(not math.isfinite(v) for v in asdict(self).values() if isinstance(v,(int,float))):
            raise ValueError('参数须为有限数值')
        if self.diameter_m<=0 or self.distance_m<=0 or not 0<self.radius_percent<=100:
            raise ValueError('直径、距离须大于0；跟踪半径为叶轮半径的0～100%（不含0）')
        if not .1<=self.fan_rpm<=600 or not 1<=self.blade_count<=12 or type(self.blade_count) is not int:
            raise ValueError('模拟风扇稳速转速0.1～600rpm；叶片数1～12')
        if type(self.cycles) is not int or self.cycles<1 or type(self.segments_per_cycle) is not int or self.segments_per_cycle<16:
            raise ValueError('圈数为正整数，每圈至少16段')
        if self.cycles*self.segments_per_cycle+1>128:
            raise ValueError('现有固件一次最多128点：圈数×每圈段数+1须≤128')
        if type(self.cycle_ms) is not int or not 50<=self.cycle_ms<=500:
            raise ValueError('下位机控制周期为50～500ms，实际以板端统计为准')
        if not 0<=self.ramp_s<self.cycles*self.period_s or self.duration_s>600:
            raise ValueError('渐入/渐出时间须≥0且小于圈数×周期，总时长不得超过600秒')
        if not .001<=self.max_speed_dps<=60 or not .001<=self.max_accel_dps2<=120:
            raise ValueError('最大轴速度0.001～60°/秒，加速度0.001～120°/秒²')
        if not .5<=self.max_error_deg<=10 or not .05<=self.end_tolerance_deg<=1 or self.end_tolerance_deg>=self.max_error_deg:
            raise ValueError('固件跟随保护误差0.5～10°；终点容差0.05～1°且小于保护误差')
        if abs(self.plane_yaw_deg)>=85 or abs(self.plane_tilt_deg)>=85:
            raise ValueError('叶轮平面偏航/倾斜须在±85°以内，不能近乎侧面对准')
        u,v,n=self.basis
        if self.distance_m-self.radius_m*math.hypot(u[2],v[2])<=0:
            raise ValueError('整圈须在激光器前方，请检查距离、尺寸和叶轮倾角')
        center=self.center
        if abs(sum(a*b for a,b in zip(center,n)))/math.sqrt(sum(a*a for a in center))<.1:
            raise ValueError('叶轮平面近乎侧对光路，几何投影不可用于本测试')

    @property
    def radius_m(self): return self.diameter_m/2*self.radius_percent/100

    @property
    def period_s(self): return 60/self.fan_rpm

    @property
    def duration_s(self): return self.cycles*self.period_s+self.ramp_s

    @property
    def center(self): return self.center_x_m,self.fan_height_m-self.laser_height_m,self.distance_m

    @property
    def basis(self):
        yaw=math.radians(self.plane_yaw_deg); tilt=math.radians(self.plane_tilt_deg)
        u=(math.cos(yaw),0,-math.sin(yaw))
        v=(math.sin(yaw)*math.sin(tilt),math.cos(tilt),math.cos(yaw)*math.sin(tilt))
        n=(math.sin(yaw)*math.cos(tilt),-math.sin(tilt),math.cos(yaw)*math.cos(tilt))
        return u,v,n

    def phase(self,t):
        """Rest-to-rest speed ramps; constant specified fan rpm in the middle."""
        t=max(0,min(self.duration_s,t));r=self.ramp_s
        if not r: progress=t
        elif t<r:
            q=t/r;progress=r*(q**3-.5*q**4)
        elif t<=self.duration_s-r: progress=t-r/2
        else:
            q=(self.duration_s-t)/r
            progress=self.cycles*self.period_s-r*(q**3-.5*q**4)
        sign=1 if self.clockwise else -1
        return math.radians(self.start_phase_deg)+sign*2*math.pi*progress/self.period_s

    def position(self,t):
        phase=self.phase(t);u,v,_=self.basis
        return tuple(c-self.radius_m*(math.sin(phase)*a+math.cos(phase)*b) for c,a,b in zip(self.center,u,v))

    def angles(self,t):
        x,y,z=self.position(t)
        return (math.degrees(math.atan2(y,math.hypot(x,z)))+self.pitch_origin_deg,
                math.degrees(math.atan2(x,z))+self.yaw_origin_deg)

    def points(self):
        count=self.cycles*self.segments_per_cycle
        points=validate_points(TrajectoryPoint(n*self.duration_s/count,*self.angles(n*self.duration_s/count)) for n in range(count+1))
        # Fixed pitch guard has 1 degree trajectory clearance; the board checks actual policies too.
        if any(not -28<=p.axis1_deg<=34 for p in points):
            raise ValueError('轨迹超出电机1固定软范围的1°余量（−28°～+34°），请修改几何/半径')
        requirements=self.requirements(points)
        if requirements['peak_speed_dps']>self.max_speed_dps+1e-9 or requirements['peak_accel_dps2']>self.max_accel_dps2+1e-9:
            raise ValueError(f'轨迹需要约{requirements["peak_speed_dps"]:.3f}°/s、{requirements["peak_accel_dps2"]:.3f}°/s²，超过输入约束；请降低风扇rpm、增加渐入时间或核对合理约束，不自动放慢周期')
        return points

    @staticmethod
    def requirements(points):
        speed=accel=0
        for field in ('axis1_deg','axis2_deg'):
            previous=None;previous_dt=0
            for a,b in zip(points,points[1:]):
                dt=b.time_s-a.time_s;v=(getattr(b,field)-getattr(a,field))/dt
                speed=max(speed,abs(v))
                if previous is not None:accel=max(accel,2*abs(v-previous)/(dt+previous_dt))
                previous=v;previous_dt=dt
        return dict(peak_speed_dps=speed,peak_accel_dps2=accel)

    def project_angles(self,pitch,yaw):
        """Estimated beam intersection in rotor-plane coordinates, no optical measurement."""
        p=math.radians(pitch-self.pitch_origin_deg);a=math.radians(yaw-self.yaw_origin_deg)
        ray=(math.sin(a)*math.cos(p),math.sin(p),math.cos(a)*math.cos(p))
        u,v,n=self.basis;den=sum(x*y for x,y in zip(ray,n))
        if abs(den)<1e-9:return None
        length=sum(x*y for x,y in zip(self.center,n))/den
        if length<=0:return None
        delta=tuple(length*x-y for x,y in zip(ray,self.center))
        return tuple(sum(x*y for x,y in zip(delta,b)) for b in (u,v))
