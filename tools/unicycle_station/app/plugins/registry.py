"""Register trusted local plugins explicitly; profiles never execute Python."""
from app.protocols.firewater import FireWaterParser

PROTOCOLS = {"firewater_v1": FireWaterParser}
VISUALIZATIONS = {}
TRANSPORTS = {}  # Extension points: TCP, UDP, CAN, HID, Bluetooth, separate image source.


def register_protocol(name, factory):
    if name in PROTOCOLS:
        raise ValueError("duplicate protocol plugin")
    PROTOCOLS[name] = factory


def register_visualization(name, plugin):
    VISUALIZATIONS[name] = plugin
