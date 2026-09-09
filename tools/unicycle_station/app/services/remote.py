"""Latest-intent remote control. Motion never enters the configuration queue."""
import math
import time


class RemoteControl:
    PERIOD = .10
    INTENT_TIMEOUT = .30
    STATUS_TIMEOUT = .8

    def __init__(self, engine, clock=time.monotonic):
        self.engine, self.clock = engine, clock
        self.reset()

    def reset(self):
        self.armed = False
        self.wanted = False
        self.supported = None
        self.active = False
        self.seen = False
        self.confirmed = 0
        self.probe_at = 0
        self.intent_at = 0
        self.sent_at = -1
        self.intent = (0., 0.)
        self.neutral_remaining = 0
        self.message = '请在车身 Run Test 中进入 Remote'
        self.echo = (0., 0., 0)

    def availability(self):
        e = self.engine
        if e.replay or e.mode not in ('serial', 'mock') or not e.connected or not e.protocol_ready:
            return False, '连接车辆并完成握手后可用'
        if self.supported is False:
            return False, '请烧录支持遥控状态查询的新版车端程序'
        if self.supported is not True or self.clock()-self.confirmed > self.STATUS_TIMEOUT:
            return False, '等待车辆确认 Remote 状态'
        if len(e.status) < 15 or self.clock()-e.status_time > .7:
            return False, '车辆状态已过期，遥控暂停'
        if not self.active or e.status[1] != 2 or any(e.status[i] for i in (2, 3, 4)):
            return False, '请在车身 Run Test → Remote 确认启动'
        if e.stop_requested.is_set():
            return False, '正在立即停车'
        return True, 'Remote 已确认 · 按住方向才运动'

    def arm(self):
        with self.engine.tx_lock:
            ok, self.message = self.availability()
            self.armed = ok
            self.intent = (0., 0.)
            self.intent_at = self.clock()
            self.sent_at = -1
            return ok

    def update_intent(self, steer, speed):
        if not all(math.isfinite(v) for v in (steer, speed)) or abs(steer)>30 or abs(speed)>.5:
            self.disarm('输入超出遥控限制')
            return False
        with self.engine.tx_lock:
            if not self.armed:
                return False
            self.intent = (float(steer), float(speed))
            self.intent_at = self.clock()
            return True

    def disarm(self, reason='遥控已释放', flush=False):
        e = self.engine
        with e.tx_lock:
            was_armed = self.armed
            self.armed = False
            self.intent = (0., 0.)
            self.message = reason
            if was_armed:
                self.neutral_remaining = 3
                self.sent_at = -1
                if flush and e.connected and e.transport and not e.stop_requested.is_set():
                    try:
                        self._write(0., 0.)
                    except (OSError, RuntimeError) as exc:
                        e.event('REMOTE_RELEASE_FAILED', str(exc))

    def _write(self, steer, speed):
        # Wire speed input uses counts of 1/40 m/s. Existing firmware validates mode.
        data = f'speed:{steer:.3f},{speed*40:.3f}{self.engine.config.ending}'.encode('ascii')
        self.engine._send(data)
        self.sent_at = self.clock()

    def accept_status(self, ok, fields):
        with self.engine.tx_lock:
            try:
                if not ok or len(fields)!=7 or fields[:2]!=['remote','1']:
                    if fields and fields[0] in ('UNKNOWN_COMMAND','UNSUPPORTED'):
                        self.supported = False
                    self.active = False
                    self.disarm('Remote 状态未确认')
                    return
                active, steer, speed, age, seen = map(float, fields[2:])
                if not all(math.isfinite(v) for v in (active,steer,speed,age,seen)) or active not in (0,1) or seen not in (0,1) or not 0<=age<=65535:
                    raise ValueError('Invalid remote status')
                self.supported = True
                self.active = bool(active)
                self.confirmed = self.clock()
                self.echo = (steer, speed, age)
                self.seen = bool(seen)
                if not self.active:self.disarm('车端已退出 Remote')
            except (ValueError, TypeError):
                self.active = False
                self.disarm('Remote 应答无效')

    def tick(self):
        e, now = self.engine, self.clock()
        with e.tx_lock:
            if e.stop_requested.is_set():
                self.armed = False; self.intent = (0.,0.); self.neutral_remaining = 0
                return
            if self.armed:
                ok, reason = self.availability()
                if not ok or now-self.intent_at > self.INTENT_TIMEOUT:
                    self.disarm(reason if not ok else '操作心跳中断，请重新启用遥控')
            if e.connected and not e.replay and now-self.sent_at >= self.PERIOD:
                if self.neutral_remaining:
                    self._write(0., 0.); self.neutral_remaining -= 1
                elif self.armed:
                    self._write(*self.intent)
            if (self.wanted or self.armed) and e.connected and e.protocol_ready and not e.replay and self.supported is not False and now-self.probe_at >= .25 and not e.requests.pending and not e.requests.queue:
                self.probe_at = now
                e.requests.enqueue('remote', callback=self.accept_status)
