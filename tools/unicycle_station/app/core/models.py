from __future__ import annotations

from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Any, Protocol
import json
import math
import os


def read_document(path: str | Path, kind: str) -> dict:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if data.get("schema_version") != 1 or data.get("kind") != kind:
        raise ValueError(f"VERSION_MISMATCH: expected {kind} schema_version=1")
    return data


def write_document(path: str | Path, kind: str, data: dict) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps({**data, "schema_version": 1, "kind": kind},
                               ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    os.replace(temp, path)


@dataclass
class ConnectionConfig:
    port: str = ""
    device_vid: int | None = None
    device_pid: int | None = None
    device_serial: str = ""
    device_location: str = ""
    baudrate: int = 115200
    bytesize: int = 8
    parity: str = "N"
    stopbits: float = 1
    flow_control: str = "none"
    dtr: bool = False
    rts: bool = False
    encoding: str = "ascii"
    ending: str = "\r\n"
    timeout: float = 0.02
    write_timeout: float = 0.05
    auto_reconnect: bool = False
    reconnect_interval: float = 2
    connect_last: bool = False

    def validate(self):
        if not 300 <= self.baudrate <= 4_000_000:
            raise ValueError("波特率应在 300–4000000")
        if self.bytesize not in (5, 6, 7, 8) or self.parity not in "NEOMS" or self.stopbits not in (1, 1.5, 2):
            raise ValueError("串口格式非法")
        if not 0 < self.timeout <= 0.1 or not 0 < self.write_timeout <= 0.1:
            raise ValueError("读写超时应为 0–0.1 秒，以保证停车响应")
        if not 0.2 <= self.reconnect_interval <= 60:
            raise ValueError("重连间隔应为 0.2–60 秒")
        "test".encode(self.encoding)
        if not self.ending or len(self.ending) > 8:
            raise ValueError("行结束符应为 1–8 个字符")


@dataclass
class Frame:
    tag: str
    values: list[Any]
    raw: str
    received: float
    error: str = ""


@dataclass
class Parameter:
    name: str
    type: str = "float"
    value: Any = 0
    min: float = -1e30
    max: float = 1e30
    group: str = "未分类参数"
    step: float = 0.01
    label: str = ""
    default: Any = None
    decimals: int = 6
    unit: str = ""
    description: str = ""
    read_only: bool = False
    runtime_writable: bool = False
    persistent: bool = True
    dangerous: bool = False
    requires_stopped: bool = True
    visible_if: dict = field(default_factory=dict)
    enabled_if: dict = field(default_factory=dict)
    enum_options: dict = field(default_factory=dict)
    sort_order: int = 0
    pending: Any = None
    edit_revision: int = 0
    edit_target: Any = None
    previous: Any = None
    ram_dirty: bool = False
    flash_state: str = "未知"

    def coerce(self, value):
        if self.type == "string":
            value = str(value)
            if any(c in value for c in ",\r\n"):
                raise ValueError("字符串不能包含协议分隔符")
            return value
        n = float(value)
        if not math.isfinite(n):
            raise ValueError("拒绝 NaN / Inf")
        if self.type in ("int", "bool", "enum"):
            if n != int(n):
                raise ValueError("请输入整数")
            n = int(n)
        if self.type == "bool" and n not in (0, 1):
            raise ValueError("布尔值仅允许 0 / 1")
        if self.enum_options and str(n) not in self.enum_options:
            raise ValueError("枚举值无效")
        return n

    def condition(self, condition, values):
        return all(values.get(k) == v for k, v in condition.items())


class Transport(Protocol):
    def open(self): ...
    def close(self): ...
    def read(self) -> bytes: ...
    def write(self, data: bytes) -> int: ...
    def status(self) -> dict: ...
    def available_devices(self) -> list[dict]: ...


class ProtocolParser(Protocol):
    def feed(self, data: bytes): ...
    def parsed_frames(self) -> list[Frame]: ...
    def encode_command(self, operation: str, seq: int, *args) -> bytes: ...
    def reset(self): ...


class DataSource(Protocol):
    def open(self): ...
    def close(self): ...
    def read(self) -> bytes: ...


class VisualizationPlugin(Protocol):
    def create_widget(self, profile, store): ...
    def save_state(self) -> dict: ...
    def restore_state(self, state: dict): ...


@dataclass
class DeviceProfile:
    data: dict
    path: str = ""

    @classmethod
    def load(cls, path):
        data = read_document(path, "device_profile")
        for key in ("name", "connection_defaults", "channels", "parameters", "dashboards",
                    "safety_rules", "orientation_mapping", "protocol_plugin"):
            if key not in data:
                raise ValueError(f"Profile 缺少 {key}")
        if data["protocol_plugin"] != "firewater_v1":
            from app.plugins.registry import PROTOCOLS
            if data["protocol_plugin"] not in PROTOCOLS:
                raise ValueError("未注册的协议插件")
        return cls(data, str(path))

    def __post_init__(self):
        if self.data.get("schema_version") != 1 or self.data.get("kind") != "device_profile":
            raise ValueError("VERSION_MISMATCH: profile v1 required")
        for key in ("name", "connection_defaults", "channels", "parameters", "dashboards", "safety_rules", "orientation_mapping", "protocol_plugin"):
            if key not in self.data:
                raise ValueError(f"Profile missing {key}")
        if len(self.data["channels"]) > 64 or sum(len(v) for v in self.data["channels"].values()) > 256:
            raise ValueError("Profile channel limit exceeded")
        from app.plugins.registry import PROTOCOLS
        if self.data["protocol_plugin"] not in PROTOCOLS:
            raise ValueError("Unknown protocol plugin")

    def __getattr__(self, name):
        try:
            return self.data[name]
        except KeyError:
            raise AttributeError(name) from None

    def channel_names(self, tag, count):
        configured = self.channels.get(tag, [])
        return [configured[i]["name"] if i < len(configured) else f"{tag}.ch{i}" for i in range(count)]

    def parameter(self, frame):
        seq, name, typ, value, low, high, group, step, flags = frame.values
        metadata = self.parameters.get(name, {})
        flags = int(flags)
        p = Parameter(name=name, type=typ, value=value, min=float(low), max=float(high),
                      group=group if group in self.data.get("parameter_groups", []) else "未分类参数",
                      step=float(step), runtime_writable=bool(flags & 1), dangerous=bool(flags & 2),
                      persistent=bool(flags & 4), read_only=bool(flags & 8), requires_stopped=not bool(flags & 1))
        for k, v in metadata.items():
            if k in ("label", "unit", "description", "decimals", "default", "visible_if", "enabled_if", "enum_options", "sort_order", "step"):
                setattr(p, k, v)
        p.value = p.coerce(value)
        return p
