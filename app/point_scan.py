"""Business-controlled point sequence; never performs laser acquisition itself."""
import threading
import time

from .motor_controller import MotorError


class PointScan:
    """Created by MotorController.prepare_point_scan(); one explicit event per step.

    The caller owns point selection, laser acquisition, persistence and decisions.
    Use one active scan at a time per physical controller; prepared scans may share
    six different RAM banks. No background thread, capture callback or auto-next.
    """
    def __init__(self, controller, record, *, name=''):
        self._controller=controller
        self.name=name;self.route_id=record['route_id'];self.token=record['token']
        self._points=tuple(tuple(p) for p in record['points'])
        self.speed_rpm=record['speed_rpm'];self.tolerance_deg=record['tolerance_deg']
        self.upload_elapsed_ms=record['upload_elapsed_ms']
        self._generation=controller._connection_generation
        self._stop_versions=dict(controller._stop_versions)
        self._phase='READY';self._index=None;self._fault=None
        self._accepted=None;self._completion=None;self._confirmed=None
        self._waiting=False;self._epoch=0;self._lock=threading.RLock()

    def _valid(self):
        if self._phase in ('STOPPED','FAULT'):raise MotorError(self._fault or '流程已取消，需重新预载后创建新流程')
        c=self._controller
        record=c._preloaded_routes.get(self.route_id)
        if (c._port is None or c._faulted or c._connection_generation!=self._generation or
            c._stop_versions!=self._stop_versions or record is None or record['token']!=self.token):
            self._phase='FAULT';self._fault='连接、停止状态或预载库已变化；旧流程不能续跑，请重新预载'
            raise MotorError(self._fault)

    def status(self):
        """Local business state only; zero wire traffic, not a live hardware query."""
        with self._lock:
            if self._phase not in ('STOPPED','FAULT','COMPLETED'):
                try:self._valid()
                except MotorError:pass
            next_index=(0 if self._index is None else self._index+1)
            return dict(name=self.name,route_id=self.route_id,token=self.token,phase=self._phase,
                        point_count=len(self._points),current_index=self._index,
                        current_target=self._points[self._index] if self._index is not None else None,
                        next_index=next_index if next_index<len(self._points) else None,
                        can_start_next=self._phase in ('READY','READY_NEXT'),
                        acquisition_required=self._phase=='AWAITING_ACQUISITION',
                        fault=self._fault,speed_rpm=self.speed_rpm,tolerance_deg=self.tolerance_deg,
                        upload_elapsed_ms=self.upload_elapsed_ms)

    def start_next_point(self):
        """Send exactly one RUN. ACK means accepted, not ready for acquisition."""
        with self._lock:
            self._valid()
            if self._phase not in ('READY','READY_NEXT'):
                raise MotorError('下一点尚未允许：需要当前点到位并确认采集通过；完成/停止后须新建流程')
            with self._controller._lock:
                owner=self._controller._point_scan_owner
                owner_record=self._controller._preloaded_routes.get(owner.route_id) if owner else None
                owner_current=bool(owner and owner_record and owner_record['token']==owner.token and
                    owner._generation==self._controller._connection_generation and owner._stop_versions==self._controller._stop_versions)
                if owner_current and owner is not self and owner._phase not in ('COMPLETED','STOPPED','FAULT'):
                    raise MotorError('已有其他逐点流程正在运动或等待采集/下一点；请先完成或停止该流程')
                self._controller._point_scan_owner=self
            index=0 if self._index is None else self._index+1
            self._phase='MOVING';self._index=index;self._completion=None
            self._began=time.monotonic()
            try:self._accepted=self._controller.run_preloaded_point(self.route_id,index,token=self.token)
            except MotorError as exc:
                self._phase='FAULT';self._fault=str(exc);raise
            self._acknowledged=time.monotonic()
            return dict(self._accepted,completed=False,phase='MOVING')

    def wait_current_point(self, *, timeout_s=120):
        """Wait for actual position + zero-speed proof; does not acquire or advance."""
        c=self._controller;c._action_timeout(timeout_s)
        with self._lock:
            self._valid()
            if self._phase=='AWAITING_ACQUISITION':return dict(self._completion)
            if self._phase!='MOVING' or self._waiting:raise MotorError('当前没有可等待的单点，或已有等待调用')
            self._waiting=True;epoch=self._epoch;index=self._index
        try:
            positions=c.wait_for_axes(self._points[index],tolerance_deg=self.tolerance_deg,timeout_s=timeout_s)
            finished=time.monotonic()
            with self._lock:
                if epoch!=self._epoch or self._phase!='MOVING':raise MotorError('逐点流程已停止；不允许进入采集')
                self._valid()
                self._phase='AWAITING_ACQUISITION'
                self._completion=dict(self._accepted,completed=True,positions=positions,
                    phase=self._phase,accepted_ms=round((self._acknowledged-self._began)*1000,3),
                    wait_ms=round((finished-self._acknowledged)*1000,3),elapsed_ms=round((finished-self._began)*1000,3))
                return dict(self._completion)
        except MotorError as exc:
            with self._lock:
                needs_stop=self._phase!='STOPPED'
                if needs_stop:self._phase='FAULT';self._fault=str(exc)
            if needs_stop:c._request_pair_stop()
            raise
        finally:
            with self._lock:self._waiting=False

    def move_next_and_wait(self, *, timeout_s=120):
        """Recommended blocking API: one point, actual completion, then wait for business decision."""
        self._controller._action_timeout(timeout_s)
        self.start_next_point()
        return self.wait_current_point(timeout_s=timeout_s)

    def confirm_acquisition(self, point_index: int, acquisition_id: str, *, passed: bool):
        """Record the caller's laser decision locally; never sends a motion command.

        passed=False holds the same point for another acquisition. passed=True
        allows a later explicit next call. Exact successful duplicates are idempotent.
        """
        if type(point_index) is not int:raise ValueError('point_index 须为整数，起始编号0')
        if not isinstance(acquisition_id,str) or not acquisition_id.strip():raise ValueError('acquisition_id 须为非空字符串')
        if type(passed) is not bool:raise ValueError('passed 须为bool')
        with self._lock:
            self._valid()
            if passed and self._confirmed==(point_index,acquisition_id) and self._phase in ('READY_NEXT','COMPLETED'):
                return self.status()
            if self._phase!='AWAITING_ACQUISITION' or point_index!=self._index:
                raise MotorError('采集确认必须对应当前已到位的点，拒绝迟到/不匹配确认')
            if passed:
                self._confirmed=(point_index,acquisition_id)
                self._phase='COMPLETED' if point_index==len(self._points)-1 else 'READY_NEXT'
            return self.status()

    def stop(self, *, wait=True, timeout_s=10):
        """Cancel progression and attempt both STOPs; confirmation failure raises."""
        self._controller._action_timeout(timeout_s)
        with self._lock:self._phase='STOPPED';self._epoch+=1
        failures=[]
        for axis in (1,2):
            try:self._controller.stop(axis,wait=wait,timeout=timeout_s)
            except MotorError as exc:failures.append(f'axis{axis}: {exc}')
        if failures:
            self._controller._request_pair_stop()
            with self._lock:self._fault='；'.join(failures)
            raise MotorError('逐点流程已取消，但停止未确认：'+self._fault)
        return dict(self.status(),stop_confirmed=bool(wait))
