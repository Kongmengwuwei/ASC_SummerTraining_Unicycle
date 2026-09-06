from collections import deque
import math
import threading


class TaskObserver:
    """MCU observations and sequenced transition history, independent of control."""
    def __init__(self, profile):
        self.labels = profile.data.get("task_states", {})
        self.lock = threading.RLock()
        self.reset()

    def reset(self):
        self.current = None
        self.stamp = 0
        self.events = deque(maxlen=500)
        self.last_seq = None
        self.last_event_uptime = None
        self.last_dropped = 0

    def label(self, code):
        return self.labels.get(str(int(code)), f"未知状态 {int(code)}")

    def accept(self, frame):
        messages = []
        with self.lock:
            if frame.tag == "task":
                values = frame.values
                if self.current is not None and values[0] < self.current[0] and self.current[0] - values[0] < 0x80000000:
                    if self.last_event_uptime is None or self.last_event_uptime > values[0]:
                        self.last_seq = None
                    messages.append(("TASK_RESET", "MCU 时间复位，任务历史分段"))
                self.current, self.stamp = list(values), frame.received
                dropped = int(values[9])
                if dropped > self.last_dropped:
                    messages.append(("TASK_EVENT_DROP", f"MCU 任务事件队列累计丢弃 {dropped} 条，历史可能不完整"))
                self.last_dropped = dropped
            elif frame.tag == "taskevt":
                seq, uptime, previous, current, element, phase, running = map(int, frame.values)
                if self.last_event_uptime is not None and uptime < self.last_event_uptime and self.last_event_uptime - uptime < 0x80000000:
                    self.last_seq = None
                    self.last_dropped = 0
                    messages.append(("TASK_RESET", "MCU 时间复位，任务历史分段"))
                self.last_event_uptime = uptime
                if self.last_seq is not None:
                    delta = (seq - self.last_seq) & 0xffffffff
                    if delta == 0 or delta >= 0x80000000:
                        return []
                    if delta > 1:
                        messages.append(("TASK_EVENT_GAP", f"任务事件序号缺口 {delta-1} 条，不补造进入/退出事件"))
                self.last_seq = seq
                if previous != current:
                    if previous:
                        messages.append(("TASK_EXIT", f"退出 {self.label(previous)}"))
                    messages.append(("TASK_ENTER", f"进入 {self.label(current)}" if previous else f"首次观测 {self.label(current)}"))
                else:
                    messages.append(("TASK_PHASE", f"{self.label(current)} · 环岛阶段 {phase} · {'Run' if running else '观察/未Run'}"))
                for kind, text in messages:
                    self.events.append({"seq":seq,"uptime":uptime,"time":frame.received,"kind":kind,"message":text,"phase":phase})
        return messages


class TrajectoryEstimator:
    """Relative planar dead reckoning. No position or heading corrections invented."""
    def __init__(self):
        self.lock = threading.RLock()
        self.yaw_sign = 1
        self.speed_sign = 1
        self.reset()

    def reset(self):
        with self.lock:
            self.points = deque(maxlen=30000)
            self.last = None
            self.x = self.y = self.distance = 0.0
            self.gaps = 0
            self.has_synchronized_source = False
            self.source = "等待速度与航向"

    def sample(self, uptime, speed, yaw, running, source="MCU 同步快照"):
        if not all(math.isfinite(v) for v in (uptime,speed,yaw)):
            return
        with self.lock:
            if not running:
                self.last = None
                return
            heading = math.radians(yaw * self.yaw_sign)
            speed *= self.speed_sign
            previous = self.last
            self.last = (int(uptime), speed, heading)
            self.source = source
            if previous is None:
                if self.points:self.points.append((float("nan"),float("nan"),int(uptime)))
                self.points.append((self.x,self.y,int(uptime)))
                return
            dt = ((int(uptime)-previous[0]) & 0xffffffff)/1000
            if dt == 0:return
            if dt > .5:
                self.gaps += 1
                self.points.append((float("nan"),float("nan"),int(uptime)))
                self.points.append((self.x,self.y,int(uptime)))
                return
            delta_heading = math.atan2(math.sin(heading-previous[2]), math.cos(heading-previous[2]))
            mid = previous[2] + delta_heading/2
            step = (speed+previous[1])*.5*dt
            self.x += step*math.cos(mid); self.y += step*math.sin(mid)
            self.distance += abs(step)
            self.points.append((self.x,self.y,int(uptime)))

    def accept(self, frame, latest, stamps):
        if frame.tag == "task":
            if not self.has_synchronized_source:
                with self.lock:self.last = None
            self.has_synchronized_source = True
            self.sample(frame.values[0],frame.values[11],frame.values[10],frame.values[5])
        elif frame.tag == "run" and not self.has_synchronized_source:
            if "yaw" in latest and 0 <= frame.received-stamps.get("yaw",0) <= .25:
                self.sample(frame.values[0],frame.values[20],latest["yaw"],int(frame.values[24])&1,"旧遥测近时配对 · 误差较大")
            else:
                self.last = None
                self.source = "缺少新鲜航向，暂停估算"
