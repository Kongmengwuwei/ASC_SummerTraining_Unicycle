import base64
import json
from pathlib import Path
import threading
import time
from app.core.models import DeviceProfile,write_document
from app.services.engine import StationEngine
from app.recording.session import ReplayDataSource


def profile():return DeviceProfile.load(Path(__file__).parents[1]/"app/profiles/tc264_unicycle/profile.json")

def test_priority_stop_bypasses_configuration_and_slow_recorder():
    e=StationEngine(profile());sent=threading.Event()
    class SlowTransport:
        def emergency_stop(self):sent.set();return 6
    e.transport=SlowTransport();e.connected=True
    e.shutdown.clear()
    t=threading.Thread(target=e._stop_loop);t.start()
    try:
        for i in range(100):e.actions.put_nowait(("get",("r_rate_kp",)))
        start=time.monotonic();e.emergency_stop()
        assert sent.wait(.2)
        assert time.monotonic()-start<.2
        assert not e.actions.empty()  # stop sent before normal work is serviced
    finally:e.shutdown.set();t.join(1)

def test_replay_preserves_time_and_exports_reopenable_session(tmp_path):
    p=profile();path=tmp_path/"frames.jsonl"
    write_document(tmp_path/"session.json","session",{"profile":p.data})
    rows=[{"schema_version":1,"kind":"raw","time":t,"base64":base64.b64encode(b"att:1,2,3\n").decode()} for t in (0,1,2)]
    rows.insert(1,{"schema_version":1,"kind":"event","time":.5,"event":"PARAMETER","message":"x changed"})
    path.write_text("\n".join(json.dumps(x) for x in rows)+"\n",encoding="utf-8")
    e=StationEngine(p)
    try:
        e.open_replay(path);e.replay.paused=False;e.replay.speed=4;e.replay.clock-=.8
        e.replay_tick()
        series=list(e.store.buffers["att_roll"])
        assert len(series)==3
        assert abs(series[1][0]-series[0][0]-1)<1e-6
        assert any(x["kind"]=="PARAMETER" for x in e.store.events)
        out=tmp_path/"export"/"selected.jsonl";out.parent.mkdir()
        e.replay.export(out,1,2)
        replay=ReplayDataSource(out);assert replay.count==2;replay.close()
    finally:e.close()
