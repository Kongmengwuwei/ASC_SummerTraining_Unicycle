from collections import deque
from dataclasses import asdict
from pathlib import Path
import queue
import threading
import time
from app.core.models import ConnectionConfig, DeviceProfile
from app.core.store import DataStore
from app.plugins.registry import PROTOCOLS
from app.recording.session import SessionRecorder, ReplayDataSource
from app.services.requests import RequestManager
from app.transports.serial_transport import SerialTransport
from app.transports.mock import MockTransport


class SerialDataSource(SerialTransport):
    pass


class MockDataSource(MockTransport):
    pass


class StationEngine:
    """GUI polls snapshots at 25 Hz; transport/parser and disk never run on GUI thread."""
    def __init__(self, profile):
        self.profile = profile
        self.store = DataStore(profile)
        self.store.event_hook = self.event
        self.config = ConnectionConfig(**profile.connection_defaults)
        self.parser = PROTOCOLS[profile.protocol_plugin]({t: len(v) for t, v in profile.channels.items()})
        self.requests = RequestManager(self.parser, self._send, self.event)
        self.actions = queue.Queue(maxsize=512)
        self.shutdown = threading.Event()
        self.stop_requested = threading.Event()
        self.stop_done = threading.Event()
        self.tx_lock = threading.RLock()
        self.safety_thread = None
        self.connected = False
        self.protocol_ready = False
        self.protocol_version = "未知"
        self.firmware = "未知"
        self.status_time = 0
        self.status = []
        self.params = {}
        self.schema_buffer = {}
        self.raw_lines = deque(maxlen=300)
        self.diagnostics = deque(maxlen=500)
        self.lock = threading.RLock()
        self.transport = None
        self.recorder = None
        self.replay = None
        self.stop_message = ""
        self.rx = self.tx = 0
        self.batch_results = []
        self.thread = None
        self.mode = ""
        self.last_revision = None
        self.schema_refresh_needed = False
        self.last_protection = None
        self.last_running = False
        self.protected_until = 0
        self.stopping_record = False
        self.auto_record = True
        self.log_directory = Path.home() / "Documents" / "EmbeddedStation" / "sessions"

    def event(self, kind, message):
        self.store.event(kind, message)
        self.diagnostics.append(f"{time.strftime('%H:%M:%S')} {kind}: {message}")
        if self.recorder:
            self.recorder.put("event", event=kind, message=message)

    def start(self, mode="mock", config=None, faults=False):
        self.close()
        self.mode = mode
        self.config = config or self.config
        self.config.validate()
        self.parser.encoding, self.parser.ending = self.config.encoding, self.config.ending
        self.parser.reset()
        self.store.reset()
        self.shutdown.clear()
        self.stop_requested.clear()
        self.stop_message = ""
        self.transport = MockDataSource(self.profile, faults) if mode == "mock" else SerialDataSource(self.config)
        self.thread = threading.Thread(target=self._run, name="device-io", daemon=True)
        self.safety_thread = threading.Thread(target=self._stop_loop, name='priority-stop', daemon=True)
        self.safety_thread.start()
        self.thread.start()

    def close(self):
        self.shutdown.set()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=5)
            if self.thread.is_alive():
                raise RuntimeError("接收线程尚未退出，不能建立第二连接")
        self.thread = None
        if self.safety_thread and self.safety_thread.is_alive():
            self.safety_thread.join(timeout=1)
        if self.replay:
            self.replay.close()
            self.replay = None
        self.stop_recording(force=True)
        self.connected = self.protocol_ready = False
        self.status_time = 0
        self.requests.cancel("DISCONNECTED")
        while not self.actions.empty():
            try:
                self.actions.get_nowait()
            except queue.Empty:
                break

    def emergency_stop(self):
        if self.replay or not self.connected:
            self.stop_message = "未连接车辆，停车命令未发送"
            return
        self.stop_done.clear()
        self.stop_message = "停车命令等待串口发送"
        self.stop_requested.set()  # independent of normal queue, consumed before reads/actions

    def _stop_loop(self):
        while not self.shutdown.is_set():
            if not self.stop_requested.wait(.03):
                continue
            if self.stop_done.is_set():
                self.shutdown.wait(.005)
                continue
            try:
                with self.tx_lock:
                    if self.connected:
                        self.tx += self.transport.emergency_stop()
                        self.stop_message = "停车命令已发送，等待 MCU 确认"
                        self.event("STOP_SENT", "stop 已优先发送；等待实际状态确认")
            except OSError as exc:
                self.stop_message = "停车发送失败：" + str(exc)
                self.event("STOP_FAILED", str(exc))
            finally:
                self.stop_done.set()

    def _send(self, data):
        with self.tx_lock:
            if self.stop_requested.is_set():
                return
            self.tx += self.transport.write(data)
        if self.recorder:
            self.recorder.put("tx", text=data.decode(self.config.encoding, errors="replace"))

    def _confirm_stop(self, ok, fields):
        # A sequenced status reply proves the MCU handled a request sent after stop.
        if ok and self.stopped() and self.status_time >= self.stop_query_since:
            self.stop_message = "MCU 已确认车辆 STOP"
            self.event("STOP_CONFIRMED", "停车后的状态查询已确认 STOP")
        elif ok:
            self.stop_message = "MCU 应答仍未确认 STOP"

    def _handshake(self):
        self.protocol_ready = False
        self.status_time = 0
        self.schema_buffer = {}
        self.last_revision = None
        self.schema_refresh_needed = False
        with self.lock:
            self.params.clear()
        self.requests.cancel("RECONNECT")
        if self.profile.data.get('supports_configuration', False):
            self.requests.enqueue("hello", callback=self._hello)

    def _hello(self, ok, fields):
        if not ok or len(fields) < 3 or fields[1] != "1":
            self.event("VERSION_MISMATCH", "旧固件或不兼容协议：仅监看及停车，参数锁定")
            return
        self.protocol_version, self.firmware = fields[1:3]
        self.protocol_ready = True
        if self.recorder:
            self.recorder.metadata.update(protocol=self.protocol_version, firmware=self.firmware)
        self.requests.enqueue("status")
        self.requests.enqueue("schema", callback=self._schema_complete)

    def _schema_complete(self, ok, fields):
        if not ok or len(fields) != 2 or not fields[1].isdigit() or int(fields[1]) != len(self.schema_buffer):
            self.event("SCHEMA_INCOMPLETE", "参数同步未完成，保持锁定；可重新读取")
            self.schema_buffer.clear()
            return
        with self.lock:
            for name,p in self.schema_buffer.items():
                old = self.params.get(name)
                if old:
                    p.pending, p.previous = old.pending, old.previous
                    changed = p.value != old.value
                    p.ram_dirty = old.ram_dirty or changed
                    p.flash_state = "外部变更 / 未确认" if changed else old.flash_state
            self.params = dict(self.schema_buffer)
        self.schema_buffer.clear()
        self.event("SCHEMA", f"已同步 {len(self.params)} 项参数")

    def submit(self, action, *args):
        try:
            self.actions.put_nowait((action, args))
        except queue.Full:
            self.event("BUSY", "普通命令队列已满")

    def stopped(self):
        return (self.connected and self.protocol_ready and time.monotonic()-self.status_time < .7
                and len(self.status) >= 15 and all(self.status[i] == 0 for i in (1, 2, 3, 4)))

    def permission(self, p, advanced=False):
        if self.replay or not self.connected or not self.protocol_ready or time.monotonic()-self.status_time >= .7:
            return False, "状态未知 / 超时 / 离线"
        values = {n: x.value for n, x in self.params.items()}
        if p.read_only or not p.condition(p.enabled_if, values):
            return False, "只读或条件未满足"
        if self.status[4]:
            return False, "Jog 禁止修改"
        if p.dangerous and (not self.stopped() or not advanced):
            return False, "危险参数需停车并解锁高级模式"
        if self.stopped():
            return True, "STOP 可编辑"
        bit = self.profile.data.get("pid_mask", {}).get(p.name)
        if p.runtime_writable and bit is not None and int(self.status[14]) & (1 << bit):
            return True, "当前闭环 PID"
        return False, "运行锁定"

    def _apply(self, changes, advanced=False):
        self.batch_results = []
        def next_item(index):
            if index == len(changes):
                self.event("BATCH", f"完成 {len(changes)} 项，RAM 已应用，尚未保存 Flash")
                return
            name, value = changes[index]
            p = self.params.get(name)
            allowed, why = self.permission(p, advanced) if p else (False, "未知参数")
            if not allowed:
                self.batch_results.append((name, False, why))
                self.event("BATCH_STOPPED", str(self.batch_results))
                return
            value = p.coerce(value)
            def result(ok, fields):
                valid = ok and len(fields) >= 3 and fields[1] == name
                if valid:
                    try:
                        actual = p.coerce(fields[2])
                    except ValueError:
                        valid = False
                if valid:
                    with self.lock:
                        p.previous, p.value, p.pending = p.value, actual, None
                        p.ram_dirty, p.flash_state = True, "未保存"
                    self.event("PARAMETER", f"{name}: {p.previous} → {p.value} ({','.join(fields[3:])})")
                self.batch_results.append((name, valid, fields))
                if valid:
                    next_item(index+1)
                else:
                    self.event("BATCH_STOPPED", str(self.batch_results))
            self.requests.enqueue("set", name, format(value, ".9g") if isinstance(value, float) else value, callback=result)
        next_item(0)

    def _action(self, action, args):
        if action == 'record_start':
            self.start_recording()
            return
        if action == 'record_stop':
            self.stop_recording()
            return
        if not self.protocol_ready:
            self.event("LOCKED", "请先连接支持 cfg v1 的固件")
            return
        if action == "apply":
            if self.requests.pending or self.requests.queue:
                self.event("BUSY", "等待当前参数请求完成")
                return
            self._apply(*args)
        elif action == "schema":
            if self.requests.pending or self.requests.queue:
                self.event("BUSY", "同步请求正在处理")
                return
            self.schema_buffer.clear()
            self.requests.enqueue("schema", callback=self._schema_complete)
        elif action == "get":
            name = args[0]
            def result(ok, fields):
                if ok and len(fields) == 3 and fields[1] == name and name in self.params:
                    self.params[name].value = self.params[name].coerce(fields[2])
            self.requests.enqueue("get", name, callback=result)
        elif action == "save":
            if not self.stopped() or self.requests.pending or self.requests.queue:
                self.event("SAVE_BLOCKED", "仅停车且参数请求全部完成后可保存")
                return
            group = args[0]
            def result(ok, fields):
                if ok and fields == ["save", group, "VERIFIED"]:
                    for p in self.params.values():
                        if group in ("all", p.group):
                            p.ram_dirty, p.flash_state = False, "已回读验证"
                    self.event("FLASH", f"{group} 保存及回读验证成功")
                else:
                    self.event("FLASH_FAILED", str(fields))
            self.requests.enqueue("save", group, callback=result)

    def ingest(self, data, received=None):
        self.rx += len(data)
        if self.recorder:
            self.recorder.raw(data)
        self.parser.feed(data, received=received)
        for frame in self.parser.parsed_frames():
            self.raw_lines.append(frame.raw)
            if self.recorder:
                self.recorder.put("frame", tag=frame.tag, values=frame.values, raw=frame.raw, error=frame.error,
                                  mcu_uptime=frame.values[0] if frame.tag in ("run", "stat") else None)
            if frame.error:
                self.event("PARSE", frame.error)
                continue
            if frame.tag == "par" and self.requests.pending and self.requests.pending.operation == "schema" and int(frame.values[0]) == self.requests.pending.seq:
                try:
                    p = self.profile.parameter(frame)
                    if len(self.schema_buffer) >= 1024:
                        raise ValueError("Schema 超出 1024 项")
                    self.schema_buffer[p.name] = p
                except ValueError as exc:
                    self.event("SCHEMA_ERROR", str(exc))
            elif frame.tag == "rsp" and not self.replay:
                self.requests.accept(frame)
            elif frame.tag == "stat":
                self.status, self.status_time = frame.values, frame.received
                running = any(frame.values[i] for i in (1, 2, 3, 4))
                if running != self.last_running:
                    self.event("MODE", "车辆进入运行" if running else "MCU 确认 STOP")
                    if not running:
                        self.protected_until = time.monotonic()+2
                    self.last_running = running
                # A periodic buffered stat alone must not confirm a newly sent stop.
                revision = int(frame.values[11])
                if self.last_revision is not None and revision != self.last_revision:
                    self.schema_refresh_needed = True
                self.last_revision = revision
                protection = (int(frame.values[9]), int(frame.values[10]))
                if protection != self.last_protection:
                    if self.last_protection is not None and any(protection):
                        self.event("PROTECTION", f"停车原因 {protection[0]} / 测试状态 {protection[1]}")
                    self.last_protection = protection
                if self.recorder:
                    self.recorder.metadata["param_revision"] = int(frame.values[11])
            self.store.accept(frame)

    def _run(self):
        try:
            while not self.shutdown.is_set():
                try:
                    if not self.connected:
                        self.transport.open()
                        self.connected = True
                        if self.auto_record and self.recorder is None:
                            self.start_recording()
                        self.event("CONNECTED", self.mode)
                        self._handshake()
                    if self.stop_requested.is_set():
                        self.requests.cancel("STOP")
                        while not self.actions.empty():
                            self.actions.get_nowait()
                        if not self.stop_done.is_set():
                            self.shutdown.wait(.002)
                            continue
                        self.stop_requested.clear()
                        if self.protocol_ready:
                            self.stop_query_since = time.monotonic()
                            self.requests.enqueue("status", callback=self._confirm_stop)
                    self.ingest(self.transport.read())
                    if self.stop_requested.is_set():
                        continue
                    try:
                        action, args = self.actions.get_nowait()
                        self._action(action, args)
                    except queue.Empty:
                        pass
                    if self.requests.pending and self.requests.pending.operation in ("set", "save") and time.monotonic()-self.status_time >= .7:
                        self.requests.cancel("STALE_STATUS")
                    self.requests.tick()
                    if self.schema_refresh_needed and self.protocol_ready and not self.requests.pending and not self.requests.queue and self.actions.empty():
                        self.schema_refresh_needed = False
                        self.schema_buffer.clear()
                        self.requests.enqueue("schema", callback=self._schema_complete)
                    if self.stopping_record and not self.last_running and time.monotonic() >= self.protected_until:
                        self.stop_recording(force=True)
                except (OSError, RuntimeError) as exc:
                    self.event("DISCONNECTED", str(exc))
                    self.connected = self.protocol_ready = False
                    self.status_time = 0
                    self.requests.cancel("DISCONNECTED")
                    while not self.actions.empty():
                        self.actions.get_nowait()
                    self.transport.close()
                    if not self.config.auto_reconnect or self.shutdown.wait(self.config.reconnect_interval):
                        break
                except (ValueError, KeyError, IndexError) as exc:
                    self.event("COMMAND_ERROR", str(exc))
                    self.requests.cancel("INVALID")
        finally:
            self.transport.close()
            self.connected = self.protocol_ready = False
            self.status_time = 0
            self.stop_recording(force=True)

    def start_recording(self):
        if self.recorder:
            return
        self.log_directory.mkdir(parents=True, exist_ok=True)
        path = self.log_directory / (time.strftime("%Y%m%d_%H%M%S")+f"_{time.time_ns()%1000000:06d}")
        self.recorder = SessionRecorder(path, self.profile)
        self.stopping_record = False
        self.event("RECORD", str(path))

    def stop_recording(self, force=False):
        if self.recorder is None:
            return
        if not force and (self.last_running or time.monotonic() < self.protected_until):
            self.stopping_record = True
            self.event("RECORD", "运行中继续记录，停车后保留至少 2 秒")
            return
        recorder, self.recorder = self.recorder, None
        recorder.close()
        self.stopping_record = False

    def open_replay(self, path):
        self.close()
        self.replay = ReplayDataSource(path)
        self.replay.open()
        self.parser.reset()
        self.store.reset()
        self.status = []
        self.params.clear()
        self.store.display_time = self.store.started
        self.event("REPLAY", str(path))

    def replay_tick(self):
        if self.replay:
            for _ in range(30):
                data = self.replay.read()
                if not data:
                    break
                self.ingest(data, received=self.store.started+self.replay.last_time)
            self.store.display_time = self.store.started+self.replay.position
            while self.replay.pending_events:
                entry = self.replay.pending_events.popleft()
                self.store.event(entry.get("event", "REPLAY_EVENT"), entry.get("message", ""), self.store.started+entry["time"])

    def replay_seek(self, seconds):
        if self.replay:
            self.replay.seek(seconds)
            self.parser.reset()
            self.store.reset()
            self.store.display_time = self.store.started+self.replay.position
            self.status_time = 0
