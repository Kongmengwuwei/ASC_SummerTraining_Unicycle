from pathlib import Path
from types import SimpleNamespace

import pytest

from app.core.models import DeviceProfile
from app.services.engine import StationEngine


def make_engine():
    profile = DeviceProfile.load(Path(__file__).parents[1] / "app/profiles/tc264_unicycle/profile.json")
    e = StationEngine(profile)
    e.connected = True
    return e


def kinds(engine):
    return [event["kind"] for event in engine.store.events]


def test_silent_uart_is_not_a_version_mismatch_even_after_previous_session_data():
    e = make_engine()
    e.rx = 12345
    e._handshake()
    assert e.handshake_rx_start == 12345
    e.requests.cancel("TIMEOUT")
    assert kinds(e) == ["NO_RX"]


def test_no_rx_timeout_keeps_parameters_locked():
    e = make_engine()
    e.rx = e.handshake_rx_start = 12345
    e._hello(False, ["TIMEOUT"])
    assert kinds(e) == ["NO_RX"]
    assert not e.protocol_ready
    assert "未收到任何数据" in e.link_status()


def test_telemetry_without_hello_is_a_handshake_timeout():
    e = make_engine()
    e.ingest(b"att:1,2,3\n")
    e._hello(False, ["TIMEOUT"])
    assert kinds(e)[-1] == "HANDSHAKE_TIMEOUT"
    assert "VERSION_MISMATCH" not in kinds(e)
    assert e.store.frame_count == 1 and len(e.store.latest) == 3
    assert not e.protocol_ready


@pytest.mark.parametrize("reason", ["STOP", "DISCONNECTED", "RECONNECT"])
def test_cancelling_hello_does_not_report_a_firmware_failure(reason):
    e = make_engine()
    e._hello(False, [reason])
    assert not kinds(e)
    assert not e.protocol_ready


def test_version_mismatch_requires_an_actual_incompatible_response():
    e = make_engine()
    e._hello(True, ["hello", "2", "tc264-cfg2", "63"])
    assert kinds(e) == ["VERSION_MISMATCH"]
    assert not e.protocol_ready


def test_current_mcu_hello_accepts_the_same_sequence_and_queues_schema():
    e = make_engine()
    sent = []
    e.requests.send = sent.append
    e._handshake()
    e.requests.tick(now=1)
    assert sent == [b"cfg:hello,1\r\n"]
    e.ingest(b"rsp:1,ok,hello,1,tc264-cfg1,63\n")
    assert e.protocol_ready and e.firmware == "tc264-cfg1"
    assert [request.operation for request in e.requests.queue] == ["status", "schema"]
    assert "HANDSHAKE_OK" in kinds(e)


def test_empty_serial_reads_do_not_create_fake_received_log_packets():
    e = make_engine()
    received = []
    e.recorder = SimpleNamespace(raw=received.append)
    e.ingest(b"")
    assert received == [] and e.rx == 0


def test_manual_handshake_retry_available_while_protocol_is_unconfirmed():
    e = make_engine()
    e._action("handshake", ())
    assert [request.operation for request in e.requests.queue] == ["hello"]
