from collections import deque
from dataclasses import dataclass
import time


@dataclass
class Request:
    operation: str
    args: tuple
    seq: int = 0
    sent: float = 0
    retries: int = 0
    callback: object = None


class RequestManager:
    def __init__(self, parser, send, notify=lambda *a: None, timeout=2):
        self.parser, self.send, self.notify = parser, send, notify
        self.timeout = timeout
        self.queue = deque(maxlen=512)
        self.pending = None
        self.sequence = 0
        self.late = 0
        self.rtt_ms = None

    def enqueue(self, operation, *args, callback=None):
        if len(self.queue) >= self.queue.maxlen:
            raise ValueError("BUSY: 配置队列已满")
        self.queue.append(Request(operation, args, callback=callback))

    def cancel(self, reason="CANCELLED"):
        requests = ([self.pending] if self.pending else []) + list(self.queue)
        self.pending = None
        self.queue.clear()
        for request in requests:
            if request.callback:
                request.callback(False, [reason])

    def tick(self, now=None):
        now = time.monotonic() if now is None else now
        if self.pending and now - self.pending.sent > (15 if self.pending.operation == 'schema' else self.timeout):
            p = self.pending
            # Only idempotent reads retry. Never repeat a set/save with uncertain outcome.
            if p.operation in ("hello", "get", "status") and p.retries < 1:
                p.retries += 1
                p.sent = now
                self.send(self.parser.encode_command(p.operation, p.seq, *p.args))
            else:
                self.notify("TIMEOUT", f"{p.operation} #{p.seq} 未确认；后续批次已取消")
                self.cancel("TIMEOUT")
        if not self.pending and self.queue:
            p = self.queue.popleft()
            self.sequence = self.sequence % 65535 + 1
            p.seq, p.sent = self.sequence, now
            self.pending = p
            self.send(self.parser.encode_command(p.operation, p.seq, *p.args))

    def accept(self, frame):
        if frame.error or frame.tag != "rsp":
            return False
        seq, status, *fields = frame.values
        if self.pending is None or int(seq) != self.pending.seq:
            self.late += 1
            return False
        p = self.pending
        if status == "ok" and fields[0] != p.operation:
            self.notify("MISMATCH", "应答操作与请求不匹配")
            return False
        self.rtt_ms = max(0, (time.monotonic() - p.sent)*1000)
        self.pending = None
        success = status == "ok"
        if p.callback:
            p.callback(success, fields)
        if not success:
            self.notify(fields[0], ",".join(fields[1:]))
            self.cancel(fields[0])
        return True
