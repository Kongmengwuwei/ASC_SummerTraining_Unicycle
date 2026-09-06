from collections import deque
import ast
import operator
import threading
import time
from app.protocols.firewater import UptimeTracker


def derive(expression, values):
    """Small arithmetic language, never eval user supplied code."""
    ops = {ast.Add: operator.add, ast.Sub: operator.sub, ast.Mult: operator.mul,
           ast.Div: operator.truediv, ast.Gt: operator.gt, ast.Lt: operator.lt}
    def visit(node):
        if isinstance(node, ast.Constant) and type(node.value) in (int, float):
            return node.value
        if isinstance(node, ast.Name):
            return values[node.id]
        if isinstance(node, ast.BinOp) and type(node.op) in ops:
            return ops[type(node.op)](visit(node.left), visit(node.right))
        if isinstance(node, ast.Compare) and len(node.ops) == 1 and type(node.ops[0]) in ops:
            return float(ops[type(node.ops[0])](visit(node.left), visit(node.comparators[0])))
        if isinstance(node, ast.UnaryOp) and isinstance(node.op, ast.USub):
            return -visit(node.operand)
        raise ValueError("不支持的派生表达式")
    if len(expression) > 256:
        raise ValueError("表达式过长")
    return visit(ast.parse(expression, mode="eval").body)


class DataStore:
    def __init__(self, profile, capacity=6500):
        self.profile = profile
        self.capacity = capacity
        self.lock = threading.RLock()
        self.event_hook = None
        self.derived_cache = {}
        self.reset()

    def reset(self):
        with self.lock:
            self.buffers = {}
            self.latest = {}
            self.stamps = {}
            self.events = deque(maxlen=500)
            self.tracker = UptimeTracker(self.profile.data.get("telemetry_period_ms", 20))
            self.started = time.monotonic()
            self.display_time = None
            self.frame_count = self.channel_drops = 0

    def event(self, kind, message, stamp=None):
        with self.lock:
            self.events.append({"time": time.monotonic() if stamp is None else stamp, "kind": kind, "message": message})

    def accept(self, frame):
        if frame.error or frame.tag in ("rsp", "par"):
            return
        with self.lock:
            self.frame_count += 1
            names = self.profile.channel_names(frame.tag, len(frame.values))
            values = dict(zip(names, frame.values))
            uptime_tag = self.profile.data.get("uptime_tag")
            if frame.tag == uptime_tag:
                _, missing, event = self.tracker.update(frame.values[0])
                if event:
                    if self.event_hook:
                        self.event_hook(event, f"遥测缺口估计 {missing} 帧；不推断控制停顿")
                    else:
                        self.event(event, f"遥测缺口估计 {missing} 帧；不推断控制停顿", frame.received)
                values["telemetry_lost"] = self.tracker.lost
            bitdef = self.profile.data.get("status_bits", {})
            for key, bits in bitdef.items():
                channel, shift, mask = bits
                if channel in values:
                    values[key] = (int(values[channel]) >> shift) & mask
            context = {**self.latest, **values}
            for name, expression in self.profile.data.get("derived_channels", {}).items():
                try:
                    # Only derive from a frame that supplies at least one operand.
                    operands = self.derived_cache.get(expression)
                    if operands is None:
                        operands = {n.id for n in ast.walk(ast.parse(expression, mode="eval")) if isinstance(n, ast.Name)}
                        if len(self.derived_cache)>=256:self.derived_cache.clear()
                        self.derived_cache[expression] = operands
                    if operands.intersection(values):
                        values[name] = float(derive(expression, context))
                except (KeyError, ValueError, ZeroDivisionError, SyntaxError):
                    pass
            for name, value in values.items():
                if name not in self.buffers:
                    if len(self.buffers) >= 256:
                        self.channel_drops += 1
                        continue
                    self.buffers[name] = deque(maxlen=self.capacity)
                self.buffers[name].append((frame.received, value))
                self.latest[name] = value
                self.stamps[name] = frame.received

    def snapshot(self):
        with self.lock:
            return dict(self.latest), dict(self.stamps), list(self.events)

    def clock(self):
        return time.monotonic() if self.display_time is None else self.display_time

    def series(self, names, seconds=10, end=None):
        end = self.clock() if end is None else end
        with self.lock:
            return {name: [(t, v) for t, v in self.buffers.get(name, ()) if end-seconds <= t <= end] for name in names}
