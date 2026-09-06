from pathlib import Path
import time
import pytest
from app.core.models import DeviceProfile
from app.services.engine import StationEngine


def wait_until(condition,timeout=5):
    until=time.monotonic()+timeout
    while time.monotonic()<until:
        if condition():return
        time.sleep(.01)
    raise AssertionError("condition timed out")


def test_mock_handshake_parameters_record_stop_and_faults(tmp_path):
    p=DeviceProfile.load(Path(__file__).parents[1]/"app/profiles/tc264_unicycle/profile.json")
    e=StationEngine(p);e.log_directory=tmp_path
    try:
        e.start("mock",faults=True)
        wait_until(lambda:len(e.params)==72)
        wait_until(e.stopped)
        e.submit("apply",[("r_rate_kp",5000)],False)
        wait_until(lambda:e.params["r_rate_kp"].value==2000)
        assert e.params["r_rate_kp"].flash_state=="未保存"
        e.submit("save","Roll")
        wait_until(lambda:e.params["r_rate_kp"].flash_state=="已回读验证")
        e.emergency_stop();wait_until(lambda:"已确认" in e.stop_message)
        assert e.connected and e.parser.errors>0
    finally:e.close()
    assert list(tmp_path.glob("*/raw.txt"))


def test_schema_end_count_mismatch_keeps_locked():
    p=DeviceProfile.load(Path(__file__).parents[1]/"app/profiles/tc264_unicycle/profile.json")
    e=StationEngine(p);e._schema_complete(True,["schema","72"])
    assert not e.params


def test_generic_profile_has_no_unicycle_channels():
    p=DeviceProfile.load(Path(__file__).parents[1]/"app/profiles/generic_sensor/profile.json")
    e=StationEngine(p);e.auto_record=False
    try:
        e.start("mock");wait_until(lambda:bool(e.store.latest))
        assert set(e.store.latest)=={"sensor_a","sensor_b"}
    finally:e.close()
