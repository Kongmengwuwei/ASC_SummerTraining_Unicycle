from pathlib import Path
import time
from app.core.models import DeviceProfile,Parameter
from app.services.engine import StationEngine

def engine():return StationEngine(DeviceProfile.load(Path(__file__).parents[1]/"app/profiles/tc264_unicycle/profile.json"))

def test_schema_refresh_keeps_pending_and_marks_external_ram_change():
    e=engine();e.params={"x":Parameter("x",value=1,pending=4,flash_state="已回读验证")}
    e.schema_buffer={"x":Parameter("x",value=2)}
    e._schema_complete(True,["schema","1"])
    assert e.params["x"].value==2 and e.params["x"].pending==4
    assert e.params["x"].ram_dirty and "外部变更" in e.params["x"].flash_state

def test_buffered_periodic_status_cannot_confirm_stop():
    e=engine();e.connected=e.protocol_ready=True
    e.stop_message="停车命令已发送，等待 MCU 确认"
    e.ingest(b"stat:100,0,0,0,0,7,1,1,1,0,0,1,0,0,16777215\n")
    assert "等待" in e.stop_message
    e.stop_query_since=time.monotonic()-1
    e._confirm_stop(True,["status"])
    assert "已确认" in e.stop_message
