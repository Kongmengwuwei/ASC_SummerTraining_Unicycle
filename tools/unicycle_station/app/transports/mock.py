from collections import deque
import math
import random
import threading
import time
from app.protocols.firewater import FireWaterParser
from app.core.live_parameters import LIVE_CONTROL_PARAMETERS


class MockTransport:
    """Deterministic protocol simulator; never contains a remote start command."""
    def __init__(self, profile, faults=False, seed=7):
        self.profile = profile
        self.faults = faults
        self.random = random.Random(seed)
        self.queue = bytearray()
        self.lock = threading.RLock()
        self.params = {p["name"]: dict(p) for p in profile.data.get("mock_parameters", [])}
        for name,p in self.params.items():
            if name in LIVE_CONTROL_PARAMETERS:p["flags"] |= 17
        self.connected = False
        self.mode = "STOP"
        self.manual_stopped = False
        self.revision = 1
        self.command_fault = ""
        self.saved = {}
        self.remote_steer = self.remote_speed = self.remote_at = 0.

    def open(self):
        self.connected = True
        self.origin = self.next_tick = time.monotonic()
        self.index = 0
        self.task_sequence = 0; self.task_previous = None
        self.manual_stopped = False
        self.queue.clear()

    def close(self):
        self.connected = False

    def append(self, line):
        self.queue.extend((line + "\n").encode("ascii"))

    def status_values(self, ms):
        return [ms, 2 if self.mode in ("Run", "Balance", "Remote") else 0, int(self.mode == "Run"),
                1 if self.mode == "Test" else 0, int(self.mode == "Jog"), 7, 1, 1, 1, 0,
                0, self.revision, 0, 0, 0x01ffffff if self.mode in ("Balance", "Run", "Remote") else 0x00ffffff if self.mode=="STOP" else 7]

    def enter_remote_demo(self):
        with self.lock:
            self.mode = "Remote"
            self.manual_stopped = True
            self.remote_steer = self.remote_speed = self.remote_at = 0.

    def telemetry(self):
        if self.mode == "Remote" and time.monotonic()-self.remote_at > 1:
            self.remote_steer = self.remote_speed = 0.
        t = self.index * .02
        if not self.manual_stopped:
            self.mode = "Run" if 5 <= t % 30 < 25 else "STOP"
        ms = (int(t * 1000) + (0xfffff800 if self.faults else 0)) & 0xffffffff
        curve = math.sin(t * .4)
        roll, pitch, yaw = 6 * math.sin(t), 3 * math.sin(t * .7), t * 9
        if "att" in self.profile.channels:
            self.append(f"att:{roll:.3f},{pitch:.3f},{yaw:.3f}")
        if "run" in self.profile.channels:
            phase = int(t // 5) % 6
            track = phase != 2
            age = 180 if phase == 3 else 20
            permission = .15 if phase == 4 else 1
            running = self.mode == "Run"
            flags = int(running) | (int(self.mode in ("Run", "Balance", "Remote")) << 1) | (int(track) << 2) | 16
            values = [ms, roll, 1.5 * curve, 6 * math.cos(t), 350 * curve, .2 * curve, 500 * curve,
                      curve, 20 * curve, 18 * curve, 16 * curve, 900 * curve, 700 * curve,
                      6800 if phase == 4 else 800 * curve, 12 * curve, .3 * curve, 8 * curve,
                      .04 * curve, .6 * running, .55 * running, .5 * running, permission,
                      .9 if track else .1, age, flags]
            if not self.faults or self.random.random() > .08:
                self.append("run:" + ",".join(f"{v:.6g}" if i not in (0, 24) else str(int(v)) for i, v in enumerate(values)))
            if self.index % 10 == 0 and self.profile.data.get("task_states"):
                road = [1,2,4,1,6,7,8,9,10,3,1][int(t/2)%11]
                element = {4:2,5:3,6:4,7:5,8:1}.get(road,0)
                phase = 1+int(t)%5 if road in (5,6) else 0
                state = (road,phase,int(running))
                if state != self.task_previous:
                    self.task_sequence += 1
                    previous = self.task_previous[0] if self.task_previous else 0
                    self.append(f"taskevt:{self.task_sequence},{ms},{previous},{road},{element},{phase},{int(running)}")
                    self.task_previous = state
                self.append(f"task:{ms},{road},{element},{phase},0,{int(running)},{age},0.9,{self.task_sequence},0,{yaw:.3f},{.5*running:.3f},{self.index+1}")
            if self.index % 10 == 0:
                self.append("stat:" + ",".join(map(str, self.status_values(ms))))
        else:
            for tag, channels in self.profile.channels.items():
                self.append(tag + ":" + ",".join(f"{math.sin(t+i):.4f}" for i in range(len(channels))))
        if self.faults and self.index % 47 == 0:
            self.append("noise from radio")
            self.append("att:NaN,1,2")
        self.index += 1

    def read(self):
        if not self.connected:
            raise OSError("Mock disconnected")
        with self.lock:
            now = time.monotonic()
            if now >= self.next_tick:
                self.telemetry()
                self.next_tick = max(self.next_tick + .02, now - .02)
            if self.queue:
                count = min(len(self.queue), self.random.randint(1, 400) if self.faults else 4096)
                data = bytes(self.queue[:count])
                del self.queue[:count]
                return data
        time.sleep(.002)
        return b""

    def write(self, data):
        with self.lock:
            for line in data.decode("ascii").splitlines():
                if line.lower() == "stop":
                    self.mode = "STOP"
                    self.manual_stopped = True
                    self.append("stat:" + ",".join(map(str, self.status_values(int((time.monotonic()-self.origin)*1000)))))
                    continue
                if line.lower().startswith("speed:"):
                    try:
                        steer, raw = map(float, line[6:].split(","))
                        if self.mode == "Remote" and all(math.isfinite(v) for v in (steer,raw)) and abs(steer)<=90 and abs(raw)<=60:
                            self.remote_steer, self.remote_speed, self.remote_at = steer, raw/40, time.monotonic()
                    except ValueError:pass
                    continue
                if not line:
                    continue
                fields = line.split(",")
                if len(fields) < 2 or not fields[0].startswith("cfg:"):
                    continue
                op, seq, *args = [fields[0][4:], *fields[1:]]
                prefix = f"rsp:{seq},"
                fault, self.command_fault = self.command_fault, ""
                if fault == "timeout":
                    continue
                if fault == "reject":
                    self.append(prefix + "err,RUNNING_LOCKED,Mock rejection")
                    continue
                if op == "hello":
                    self.append(prefix + "ok,hello,1,mock-0.1,63")
                elif op == "remote":
                    age=min(65535,int((time.monotonic()-self.remote_at)*1000))
                    self.append(prefix + f"ok,remote,1,{int(self.mode=='Remote')},{self.remote_steer:.3f},{self.remote_speed:.3f},{age},{int(self.remote_at>0)}")
                elif op == "status":
                    self.append("stat:" + ",".join(map(str, self.status_values(int((time.monotonic()-self.origin)*1000)))))
                    self.append(prefix + "ok,status")
                elif op == "schema":
                    for p in self.params.values():
                        self.append(f"par:{seq},{p['name']},{p['type']},{p['value']},{p['min']},{p['max']},{p['group']},{p['step']},{p['flags']}")
                    self.append(prefix + f"ok,schema,{len(self.params)}")
                elif op in ("get", "set") and args:
                    p = self.params.get(args[0])
                    if not p:
                        self.append(prefix + "err,UNKNOWN_PARAM,Unknown parameter")
                    elif op == "get":
                        self.append(prefix + f"ok,get,{p['name']},{p['value']}")
                    elif self.mode != "STOP" and (self.mode == "Jog" or not p["flags"] & 1 or (p["flags"] & 16 and self.mode not in ("Balance","Run","Remote"))):
                        self.append(prefix + "err,RUNNING_LOCKED,Locked")
                    else:
                        try:
                            v = float(args[1])
                            if not math.isfinite(v) or (p["type"] == "int" and v != int(v)):
                                raise ValueError()
                            applied = max(p["min"], min(p["max"], v))
                            p["value"] = int(applied) if p["type"] == "int" else applied
                            self.revision += 1
                            self.append(prefix + f"ok,set,{p['name']},{p['value']},{'CLAMPED' if v != applied else 'APPLIED'}")
                        except (ValueError, IndexError):
                            self.append(prefix + "err,INVALID_VALUE,Invalid numeric value")
                elif op == "save" and args:
                    if self.mode != "STOP":
                        self.append(prefix + "err,SAVE_BLOCKED,Stop first")
                    else:
                        for p in self.params.values():
                            if args[0] in ("all", p["group"]):
                                self.saved[p["name"]] = p["value"]
                        self.append(prefix + f"ok,save,{args[0]},VERIFIED")
                else:
                    self.append(prefix + "err,UNKNOWN_COMMAND,Not supported")
        return len(data)

    def emergency_stop(self):
        return self.write(b"stop\r\n")

    def status(self):
        return {"connected": self.connected, "mock": True}

    @staticmethod
    def available_devices():
        return [{"port": "Mock", "description": "No hardware required"}]
