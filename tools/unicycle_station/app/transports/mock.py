from collections import deque
import math
import random
import threading
import time
from app.protocols.firewater import FireWaterParser


class MockTransport:
    """Deterministic protocol simulator; never contains a remote start command."""
    def __init__(self, profile, faults=False, seed=7):
        self.profile = profile
        self.faults = faults
        self.random = random.Random(seed)
        self.queue = bytearray()
        self.lock = threading.RLock()
        self.params = {p["name"]: dict(p) for p in profile.data.get("mock_parameters", [])}
        self.connected = False
        self.mode = "STOP"
        self.manual_stopped = False
        self.revision = 1
        self.command_fault = ""
        self.saved = {}

    def open(self):
        self.connected = True
        self.origin = self.next_tick = time.monotonic()
        self.index = 0
        self.manual_stopped = False
        self.queue.clear()

    def close(self):
        self.connected = False

    def append(self, line):
        self.queue.extend((line + "\n").encode("ascii"))

    def status_values(self, ms):
        return [ms, 2 if self.mode in ("Run", "Balance") else 0, int(self.mode == "Run"),
                1 if self.mode == "Test" else 0, int(self.mode == "Jog"), 7, 1, 1, 1, 0,
                0, self.revision, 0, 0, 0x00ffffff if self.mode in ("STOP", "Balance", "Run") else 7]

    def telemetry(self):
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
            flags = int(running) | (int(self.mode in ("Run", "Balance")) << 1) | (int(track) << 2) | 16
            values = [ms, roll, 1.5 * curve, 6 * math.cos(t), 350 * curve, .2 * curve, 500 * curve,
                      curve, 20 * curve, 18 * curve, 16 * curve, 900 * curve, 700 * curve,
                      6800 if phase == 4 else 800 * curve, 12 * curve, .3 * curve, 8 * curve,
                      .04 * curve, .6 * running, .55 * running, .5 * running, permission,
                      .9 if track else .1, age, flags]
            if not self.faults or self.random.random() > .08:
                self.append("run:" + ",".join(f"{v:.6g}" if i not in (0, 24) else str(int(v)) for i, v in enumerate(values)))
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
                    elif self.mode != "STOP" and (self.mode == "Jog" or not p["flags"] & 1):
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
