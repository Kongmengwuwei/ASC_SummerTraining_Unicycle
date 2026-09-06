from __future__ import annotations
from collections import deque
import math
import base64
import json
from pathlib import Path
import queue
import threading
import time
from app.core.models import write_document, read_document


class SessionRecorder:
    """Bounded nonblocking producer; one background writer, flush every 0.5 s."""
    def __init__(self, path, profile):
        self.path = Path(path)
        self.path.mkdir(parents=True, exist_ok=False)
        self.queue = queue.Queue(maxsize=20000)
        self.dropped = 0
        self.error = ""
        self.origin = time.monotonic()
        self.finished = threading.Event()
        self.metadata_lock = threading.RLock()
        self.metadata = {"profile": profile.data, "created": time.time(), "protocol": "unconfirmed",
                         "firmware": "unconfirmed", "param_revision": None}
        write_document(self.path / "session.json", "session", self.metadata)
        self.worker = threading.Thread(target=self._run, name="session-writer", daemon=True)
        self.worker.start()

    def update_metadata(self, **data):
        with self.metadata_lock:self.metadata.update(data)

    def save_metadata(self):
        with self.metadata_lock:write_document(self.path / "session.json", "session", self.metadata)

    def put(self, kind, **data):
        record = {"schema_version": 1, "kind": kind, "time": time.monotonic()-self.origin, "pc_time": time.time(), **data}
        try:
            self.queue.put_nowait(record)
        except queue.Full:
            self.dropped += 1

    def raw(self, data):
        self.put("raw", base64=base64.b64encode(data).decode("ascii"))

    def close(self):
        self.finished.set()
        self.worker.join(timeout=5)
        if self.worker.is_alive():
            self.error = "日志线程未在 5 秒内退出"
        self.update_metadata(dropped=self.dropped, error=self.error)
        self.save_metadata()

    def _run(self):
        try:
            with (self.path / "raw.txt").open("wb") as raw, (self.path / "frames.jsonl").open("w", encoding="utf-8", buffering=1) as structured:
                last_flush = time.monotonic()
                while not self.finished.is_set() or not self.queue.empty():
                    try:
                        entry = self.queue.get(timeout=.1)
                    except queue.Empty:
                        entry = None
                    if entry:
                        if entry["kind"] == "raw":
                            raw.write(base64.b64decode(entry["base64"]))
                        structured.write(json.dumps(entry, ensure_ascii=False, allow_nan=False) + "\n")
                    if time.monotonic() - last_flush > .5:
                        raw.flush()
                        structured.flush()
                        last_flush = time.monotonic()
                        self.save_metadata()
        except Exception as exc:
            self.error = str(exc)


class ReplayDataSource:
    """Disk backed playback; sparse seek index bounds memory even for long sessions."""
    def __init__(self, path, progress=lambda n:None):
        self.path = Path(path)
        if self.path.is_dir():
            self.path /= "frames.jsonl"
        sidecar = self.path.with_suffix(".session.json")
        self.metadata = read_document(sidecar if sidecar.exists() else self.path.parent / "session.json", "session")
        self.index = []
        self.events = []
        self.duration = 0
        self.count = 0
        self.statistics = {}
        self.pending_events = deque(maxlen=500)
        self.last_time = 0
        total_size=max(1,self.path.stat().st_size);progress(0)
        with self.path.open("rb") as f:
            last_index = -1
            while True:
                offset = f.tell()
                line = f.readline()
                if not line:
                    break
                try:
                    item = json.loads(line)
                    at = float(item["time"])
                except (ValueError, KeyError):
                    continue  # A crash can leave an incomplete final line.
                self.count += 1
                if self.count%1000==0:progress(min(99,int(f.tell()*100/total_size)))
                if item["kind"] == "frame" and not item.get("error"):
                    for i, value in enumerate(item.get("values", [])[:128]):
                        if type(value) not in (float, int) or not math.isfinite(value):
                            continue
                        key = f"{item.get('tag', 'data')}.ch{i}"
                        if key not in self.statistics and len(self.statistics) >= 256:
                            continue
                        stat = self.statistics.setdefault(key, {"count":0,"min":value,"max":value,"sum":0,"sum_square":0})
                        stat["count"] += 1
                        stat["min"] = min(stat["min"], value)
                        stat["max"] = max(stat["max"], value)
                        stat["sum"] += value
                        stat["sum_square"] += value*value
                self.duration = max(self.duration, at)
                if at - last_index >= 5:
                    self.index.append((at, offset))
                    last_index = at
                if item["kind"] == "event" and len(self.events) < 10000:
                    self.events.append((at, item.get("message", "event")))
        progress(100)
        self.speed = 1
        self.paused = True
        self.position = 0
        self.file = None

    def open(self):
        self.file = self.path.open("rb")
        self.pending = None
        self.clock = time.monotonic()

    def close(self):
        if self.file:
            self.file.close()
        self.file = None

    def seek(self, position):
        self.position = min(self.duration, max(0, position))
        offset = 0
        for at, candidate in self.index:
            if at > self.position:
                break
            offset = candidate
        self.pending_events.clear()
        self.file.seek(offset)
        self.pending = None
        while line := self.file.readline():
            try:
                item = json.loads(line)
                if item["time"] >= self.position:
                    self.pending = item
                    break
            except (ValueError, KeyError):
                continue
        self.clock = time.monotonic()

    def read(self):
        now = time.monotonic()
        if not self.paused:
            self.position = min(self.duration, self.position + (now-self.clock)*self.speed)
        self.clock = now
        if self.paused:
            return b""
        for _ in range(1000):
            if self.pending is None:
                line = self.file.readline()
                if not line:
                    self.paused = True
                    return b""
                try:
                    self.pending = json.loads(line)
                except ValueError:
                    continue
            if self.pending["time"] > self.position:
                return b""
            item, self.pending = self.pending, None
            if item["kind"] == "event":
                self.pending_events.append(item)
            if item["kind"] == "raw":
                self.last_time = float(item["time"])
                return base64.b64decode(item["base64"])
        return b""

    def export(self, path, start=0, end=None):
        end = self.duration if end is None else end
        write_document(Path(path).with_suffix(".session.json"), "session", self.metadata)
        with self.path.open(encoding="utf-8") as source, Path(path).open("w", encoding="utf-8") as dest:
            for line in source:
                try:
                    item = json.loads(line)
                    if start <= item["time"] <= end:
                        dest.write(line)
                except (ValueError, KeyError):
                    continue

    def summary(self):
        return {"duration_s": self.duration, "records": self.count, "events": len(self.events), "channels": {k: {"count":s["count"], "min":s["min"], "max":s["max"], "mean":s["sum"]/s["count"], "rms":math.sqrt(s["sum_square"]/s["count"])} for k,s in self.statistics.items()}}
