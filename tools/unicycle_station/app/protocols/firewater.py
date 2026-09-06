from collections import deque
import math
import re
import time
from app.core.models import Frame

NUMBER = re.compile(r"^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?$")
IDENT = re.compile(r"^[A-Za-z_][A-Za-z0-9_.-]{0,47}$")


def number(text):
    if not NUMBER.fullmatch(text.strip()):
        raise ValueError("INVALID_NUMBER")
    n = float(text)
    if not math.isfinite(n):
        raise ValueError("NON_FINITE")
    return n


class FireWaterParser:
    """Bounded incremental line decoder. Bad/unknown lines remain observable."""
    def __init__(self, counts=None, encoding="ascii", ending="\r\n"):
        self.counts = {"att": 3, "run": 25, "stat": 15} if counts is None else counts
        self.encoding, self.ending = encoding, ending
        self.reset()

    def reset(self):
        self.buffer = bytearray()
        self.overflow = False
        self.frames = deque(maxlen=4096)
        self.errors = 0
        self.queue_drops = 0

    def feed(self, data, received=None):
        stamp = time.monotonic() if received is None else received
        for b in data:
            if b in (10, 13):
                if self.buffer or self.overflow:
                    raw = self.buffer.decode(self.encoding, errors="replace")
                    frame = self.parse(raw, stamp) if not self.overflow else Frame("invalid", [], raw, stamp, "LINE_TOO_LONG")
                    self.errors += bool(frame.error)
                    self.queue_drops += len(self.frames) == self.frames.maxlen
                    self.frames.append(frame)
                self.buffer.clear()
                self.overflow = False
            elif len(self.buffer) < 4096:
                self.buffer.append(b)
            else:
                self.overflow = True

    def parse(self, raw, stamp):
        try:
            tag, payload = raw.split(":", 1)
            if not IDENT.fullmatch(tag):
                raise ValueError("INVALID_TAG")
            fields = payload.split(",")
            if tag in ("rsp", "par"):
                if not fields[0].isdigit() or not 1 <= int(fields[0]) <= 65535:
                    raise ValueError("INVALID_SEQ")
                if tag == "rsp" and (len(fields) < 3 or fields[1] not in ("ok", "err")):
                    raise ValueError("BAD_RESPONSE")
                if tag == "par":
                    if len(fields) != 9 or fields[2] not in ("float", "int", "bool", "enum", "string") or not IDENT.fullmatch(fields[1]):
                        raise ValueError("BAD_SCHEMA")
                    for i in (4, 5, 7, 8):
                        number(fields[i])
                    if fields[2] != "string":
                        number(fields[3])
                    if float(fields[4]) > float(fields[5]) or float(fields[7]) <= 0:
                        raise ValueError("BAD_SCHEMA_RANGE")
                return Frame(tag, fields, raw, stamp)
            if tag in ("task", "taskevt") and len(fields) != (13 if tag == "task" else 7):
                raise ValueError("FIELD_COUNT")
            if tag in self.counts and len(fields) != self.counts[tag]:
                raise ValueError("FIELD_COUNT")
            if len(fields) > 128:
                raise ValueError("TOO_MANY_CHANNELS")
            values = [number(x) for x in fields]
            if tag == "run":
                if not 0 <= values[0] <= 0xffffffff or values[0] != int(values[0]):
                    raise ValueError("BAD_UPTIME")
                if not 0 <= values[24] <= 65535 or values[24] != int(values[24]):
                    raise ValueError("BAD_FLAGS")
            if tag in ("task", "taskevt"):
                integers = [v for i,v in enumerate(values) if tag == "taskevt" or i not in (7,10,11)]
                if any(v < 0 or v > 0xffffffff or v != int(v) for v in integers):raise ValueError("BAD_TASK")
                if tag == "task" and (values[1]>12 or values[2]>5 or values[5]>1 or not 0<=values[7]<=1):raise ValueError("BAD_TASK")
                if tag == "taskevt" and (values[2]>12 or values[3]>12 or values[6]>1):raise ValueError("BAD_TASK_EVENT")
            if tag == "stat" and any(x < 0 or x > 0xffffffff or x != int(x) for x in values):
                raise ValueError("BAD_STATUS")
            return Frame(tag, values, raw, stamp)
        except (ValueError, OverflowError) as exc:
            return Frame("invalid", [], raw, stamp, str(exc))

    def parsed_frames(self):
        result = list(self.frames)
        self.frames.clear()
        return result

    def encode_command(self, operation, seq, *args):
        if operation == "stop":
            return b"stop\r\n"
        if operation not in ("hello", "schema", "get", "set", "save", "status"):
            raise ValueError("UNKNOWN_COMMAND / remote starts are forbidden")
        if not 1 <= seq <= 65535:
            raise ValueError("INVALID_SEQ")
        arity = {"hello": 0, "schema": 0, "get": 1, "set": 2, "save": 1, "status": 0}
        if len(args) != arity[operation]:
            raise ValueError("BAD_FORMAT")
        values = [str(v) for v in args]
        if any(any(c in s for c in ",\r\n\x00") or not s for s in values):
            raise ValueError("BAD_FORMAT")
        if operation == "set":
            number(values[1])
        line = ",".join([f"cfg:{operation}", str(seq), *values])
        if len(line.encode(self.encoding)) > 63:
            raise ValueError("COMMAND_TOO_LONG (63 bytes max)")
        return (line + self.ending).encode(self.encoding)


class UptimeTracker:
    def __init__(self, period_ms=20):
        self.period = period_ms
        self.last = None
        self.elapsed = 0
        self.lost = self.wraps = self.resets = self.received = 0

    def update(self, uptime):
        uptime = int(uptime)
        event = ""
        missing = 0
        if self.last is not None:
            delta = (uptime - self.last) & 0xffffffff
            if delta >= 0x80000000:
                self.resets += 1
                event = "MCU_RESET_OR_OUT_OF_ORDER"
                delta = self.period
            elif uptime < self.last:
                self.wraps += 1
                event = "UPTIME_WRAP"
            if delta < 10000:
                missing = max(0, round(delta / self.period) - 1)
                self.lost += missing
                if missing:
                    event = "TELEMETRY_GAP"  # Not evidence of a stalled 1ms controller.
            elif not event:
                event = "TELEMETRY_DISCONTINUITY"
            self.elapsed += delta / 1000
        self.last = uptime
        self.received += 1
        return self.elapsed, missing, event
