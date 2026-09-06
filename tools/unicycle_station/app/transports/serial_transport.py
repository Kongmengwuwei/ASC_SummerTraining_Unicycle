import threading
from dataclasses import asdict
import serial
from serial.tools import list_ports
from app.core.models import ConnectionConfig


def matching_devices(config, devices):
    def matches(d):
        same_id = d.get("vid") == config.device_vid and d.get("pid") == config.device_pid
        if config.device_serial:
            return same_id and d.get("serial_number") == config.device_serial
        return same_id and bool(config.device_location) and d.get("location") == config.device_location
    return [d for d in devices if matches(d)]


class SerialTransport:
    def __init__(self, config: ConnectionConfig):
        self.config = config
        self.device = None
        self.write_lock = threading.Lock()

    def open(self):
        self.config.validate()
        c = self.config
        if c.device_serial or c.device_location:
            matches = matching_devices(c, self.available_devices())
            if len(matches) != 1:
                raise OSError("记住的串口设备未找到或不唯一，请重新选择设备")
            c.port = matches[0]["port"]
        dev = serial.Serial(port=None, baudrate=c.baudrate, bytesize=c.bytesize, parity=c.parity,
                            stopbits=c.stopbits, timeout=c.timeout, write_timeout=c.write_timeout,
                            xonxoff=c.flow_control == "xonxoff", rtscts=c.flow_control == "rtscts",
                            dsrdtr=c.flow_control == "dsrdtr")
        dev.dtr, dev.rts = c.dtr, c.rts
        dev.port = c.port
        dev.open()
        self.device = dev

    def close(self):
        if self.device:
            self.device.close()
        self.device = None

    def read(self):
        if self.device is None:
            raise OSError("串口未连接")
        return self.device.read(min(4096, max(1, self.device.in_waiting)))

    def write(self, data):
        with self.write_lock:
            if self.device is None:
                raise OSError("串口未连接")
            count = self.device.write(data)
            if count != len(data):
                raise OSError("串口发送不完整，连接将关闭")
            return count

    def emergency_stop(self):
        # Worker is the sole writer. Clear OS pending output before sending stop.
        if self.device is None:
            raise OSError("串口未连接")
        self.device.reset_output_buffer()
        # A separator invalidates any partially transmitted config command.
        return self.write(b"\r\nstop\r\n")

    def status(self):
        return {"connected": bool(self.device and self.device.is_open), "config": asdict(self.config)}

    @staticmethod
    def available_devices():
        return [{"port": p.device, "description": p.description, "vid": p.vid, "pid": p.pid,
                 "serial_number": p.serial_number, "location": p.location} for p in list_ports.comports()]
