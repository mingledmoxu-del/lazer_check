"""Offline geometry, absolute blade deadlines and encoder-only statistics."""
from dataclasses import dataclass, asdict
import math


@dataclass(frozen=True)
class BladePlan:
    count: int = 4
    radius_index: int = 3
    period_s: float = 5.0
    mode: str = 'A'
    speed_rpm: float = 1.0
    cycles: int = 1
    distance_m: float = 2.5
    height_m: float = .73
    pitch_origin: float = .267
    yaw_origin: float = .234
    tolerance_deg: float = .2
    hold_s: float = .5

    def __post_init__(self):
        if self.count not in (3, 4) or self.radius_index not in range(1, 6):
            raise ValueError('选择三点/四点及 P1～P5 中的一圈')
        if self.mode not in ('A', 'B') or not isinstance(self.cycles, int) or not 1 <= self.cycles <= 100:
            raise ValueError('模式 A/B；循环次数1～100')
        values = asdict(self)
        if any(not math.isfinite(v) for v in values.values() if isinstance(v, (int, float))):
            raise ValueError('参数必须是有限数值')
        if not .01 <= self.speed_rpm <= 100 or not .1 <= self.period_s <= 3600:
            raise ValueError('实际输出轴速度0.01～100rpm；每转周期0.1～3600秒')
        if self.distance_m <= 0 or not .01 <= self.tolerance_deg <= .5 or not .1 <= self.hold_s <= 30:
            raise ValueError('距离须大于0；容差0.01～0.5°；末段观察0.1～30秒')
        if self.delay_s >= self.interval_s:
            raise ValueError('离开确认/业务延迟已用完换点间隔，请增加模拟周期')
        if any(not -29 <= a <= 35 or abs(b) > 36000 for a, b in self.points):
            raise ValueError('几何目标超出电机角度范围；请核对参考原点和安装几何')

    @property
    def interval_s(self):
        return self.period_s / self.count

    @property
    def passage_s(self):
        # The supplied widths are estimates at a five-second rotor period.
        return (.380, .170, .070, .050, .030)[self.radius_index-1] * self.period_s / 5

    @property
    def delay_s(self):
        return self.passage_s + .030 + (.100 if self.mode == 'B' else 0)

    @property
    def points(self):
        radius = (.16, .30, .44, .58, .72)[self.radius_index-1]
        result = []
        for n in range(self.count):
            phase = 2*math.pi*n/self.count
            x = -radius*math.sin(phase)
            y = self.height_m-radius*math.cos(phase)
            result.append((round(math.degrees(math.atan2(y, math.hypot(self.distance_m, x)))+self.pitch_origin, 3),
                           round(math.degrees(math.atan2(x, self.distance_m))+self.yaw_origin, 3)))
        return tuple(result)

    def leg(self, index):
        if not 0 <= index < self.cycles*self.count:
            raise ValueError('段编号超出本次测试')
        return dict(index=index, point=(index+1) % self.count,
                    send_s=index*self.interval_s+self.delay_s,
                    blade_s=(index+1)*self.interval_s,
                    target=self.points[(index+1) % self.count])


def encoder_stats(samples):
    """Post-completion drive encoder angle only; no mechanical/optical claim."""
    if not samples:
        return dict(samples=0, span_s=0, pp_deg=None, rms_deg=None, drift_deg=None, max_error_deg=None)
    angles = [s['angle_deg'] for s in samples]
    mean = sum(angles)/len(angles)
    return dict(samples=len(samples), span_s=round(samples[-1]['observed_s']-samples[0]['observed_s'], 6),
                pp_deg=round(max(angles)-min(angles), 6),
                rms_deg=round(math.sqrt(sum((a-mean)**2 for a in angles)/len(angles)), 6),
                drift_deg=round(angles[-1]-angles[0], 6),
                max_error_deg=round(max(abs(s['angle_deg']-s['target_deg']) for s in samples), 6))
