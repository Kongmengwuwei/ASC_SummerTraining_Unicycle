import time
import pytest
from app.services.engine import StationEngine
from app.services.remote import RemoteControl
from app.ui.window import builtin_profile


class Transport:
    def __init__(self):self.writes=[]
    def write(self,data):self.writes.append(data);return len(data)


@pytest.fixture
def remote():
    e=StationEngine(builtin_profile());e.auto_record=False
    now=[100.]
    e.remote=RemoteControl(e,lambda:now[0]);e.transport=Transport()
    e.connected=e.protocol_ready=True;e.mode='serial';e.status=[0]*15;e.status[1]=2;e.status_time=now[0]
    e.remote.accept_status(True,['remote','1','1','0','0','0','0'])
    yield e,now
    e.connected=False;e.close()


def test_latest_intent_release_and_gui_stall(remote):
    e,now=remote;r=e.remote
    assert r.arm()
    assert r.update_intent(10,.15)
    assert r.update_intent(-10,-.2)
    r.tick();assert e.transport.writes==[b'speed:-10.000,-8.000\r\n']
    now[0]+=.11;r.update_intent(0,0);r.tick()
    assert e.transport.writes[-1]==b'speed:0.000,0.000\r\n'
    r.update_intent(15,.3);now[0]+=.31;r.tick()
    assert not r.armed and e.transport.writes[-1]==b'speed:0.000,0.000\r\n'
    assert not r.update_intent(10,.1)  # GUI recovery cannot resume without explicit arm.


def test_disconnect_or_emergency_cannot_replay_motion(remote):
    e,now=remote;r=e.remote;r.arm();r.update_intent(10,.2);r.tick()
    e.emergency_stop();now[0]+=.11;r.tick()
    assert len(e.transport.writes)==1 and not r.armed
    e.stop_requested.clear();now[0]+=.11;r.tick()
    assert len(e.transport.writes)==1
    r.reset();assert not r.arm()


@pytest.mark.parametrize('failure',['old','inactive','run','stale','replay','nan'])
def test_unconfirmed_or_unsafe_remote_is_blocked(remote,failure):
    e,now=remote;r=e.remote
    if failure=='old':r.accept_status(False,['UNKNOWN_COMMAND'])
    elif failure=='inactive':r.accept_status(True,['remote','1','0','0','0','0','0'])
    elif failure=='run':e.status[2]=1
    elif failure=='stale':now[0]+=1
    elif failure=='replay':e.replay=object()
    else:r.accept_status(True,['remote','1','1','nan','0','0','0'])
    assert not r.arm();r.tick();assert not e.transport.writes
    e.replay=None


def test_limits_and_release_repeats_neutral_only(remote):
    e,now=remote;r=e.remote;r.arm();r.update_intent(10,.2);r.tick()
    assert not r.update_intent(31,.2)
    for i in range(3):now[0]+=.11;r.tick()
    assert e.transport.writes[1:]==[b'speed:0.000,0.000\r\n']*3
    assert not r.armed


def test_remote_probe_and_mock_wire_path():
    e=StationEngine(builtin_profile());e.auto_record=False
    try:
        e.start('mock');deadline=time.monotonic()+8
        while time.monotonic()<deadline and (not e.protocol_ready or len(e.params)<72):time.sleep(.02)
        assert e.protocol_ready and len(e.params)==72
        e.transport.enter_remote_demo();e.remote.wanted=True
        deadline=time.monotonic()+3
        while time.monotonic()<deadline and not e.remote.availability()[0]:time.sleep(.02)
        assert e.remote.arm()
        deadline=time.monotonic()+.25
        while time.monotonic()<deadline:e.remote.update_intent(10,-.15);time.sleep(.02)
        assert e.transport.remote_steer==10 and e.transport.remote_speed==-.15
        # No UI heartbeat: I/O independently replaces motion by neutral.
        time.sleep(.5)
        assert not e.remote.armed and e.transport.remote_speed==0 and e.transport.remote_steer==0
        e.close();e.start('mock');assert not e.remote.armed
    finally:e.close()


def test_close_releases_before_transport_shutdown(remote):
    e,now=remote;r=e.remote;r.arm();r.update_intent(10,.2);r.tick()
    e.close()
    assert e.transport.writes[-1]==b'speed:0.000,0.000\r\n'
    assert not r.armed and r.supported is None


def test_mode_loss_and_query_expiry_release_active_input(remote):
    e,now=remote;r=e.remote;r.arm();r.update_intent(10,.2);r.tick()
    e.status[2]=1;now[0]+=.11;r.tick()
    assert not r.armed and e.transport.writes[-1]==b'speed:0.000,0.000\r\n'
    e.status[2]=0;r.arm();r.update_intent(10,.2)
    now[0]+=.81;e.status_time=now[0];r.update_intent(10,.2);r.tick()
    assert not r.armed  # Fresh ordinary telemetry cannot replace a Remote confirmation.
