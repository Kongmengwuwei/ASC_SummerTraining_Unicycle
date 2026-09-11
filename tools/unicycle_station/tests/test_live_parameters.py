import time
from app.core.models import Frame
from app.core.live_parameters import LIVE_CONTROL_PARAMETERS
from app.services.engine import StationEngine
from app.ui.window import builtin_profile
def wait_until(condition,timeout=5):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        if condition():return
        time.sleep(.01)
    raise AssertionError('condition timed out')


def test_schema_flags_and_mode_mask_both_required():
    e=StationEngine(builtin_profile());e.connected=e.protocol_ready=True;e.status_time=time.monotonic()
    e.status=[0]*15;e.status[1]=2;e.status[14]=0xffffff
    frame=Frame('par',['1','lean_speed_kp','float','1','0','2','Lean','0.1','21'],'',time.monotonic())
    p=e.profile.parameter(frame)
    assert p.runtime_control and not e.permission(p)[0]
    e.status[14]|=1<<24;assert e.permission(p)[0]
    frame.values[-1]='4';old=e.profile.parameter(frame);assert not e.permission(old)[0]
    e.status[3]=1;assert not e.permission(p)[0]
    e.status[3]=0;e.status[4]=1;assert not e.permission(p)[0]
    e.status[4]=0;e.status_time-=1;assert not e.permission(p)[0]


def test_mock_live_writes_confirm_without_stopping_or_saving():
    e=StationEngine(builtin_profile());e.auto_record=False
    try:
        e.start('mock');wait_until(lambda:len(e.params)==len(e.profile.data['mock_parameters']))
        with e.transport.lock:e.transport.mode='Run';e.transport.manual_stopped=True
        wait_until(lambda:e.status and e.status[2]==1)
        assert all(e.permission(e.params[name])[0] for name in LIVE_CONTROL_PARAMETERS)
        assert not e.permission(e.params['motor_dir_a'],True)[0]
        e.submit('apply',[('run_speed_straight',.3),('direction_heading_kp',1.0),('lean_speed_kp',1.2)],False)
        wait_until(lambda:len(e.batch_results)==3)
        assert all(ok for _,ok,_ in e.batch_results)
        assert e.params['lean_speed_kp'].value==1.2 and e.params['lean_speed_kp'].ram_dirty
        assert e.transport.mode=='Run' and not e.transport.saved
        with e.transport.lock:e.transport.mode='Balance'
        wait_until(lambda:e.status and not e.status[2])
        assert e.permission(e.params['p_vel_kp'])[0]
    finally:e.close()


def test_mock_allowlist_matches_firmware():
    import re
    from pathlib import Path
    source=(Path(__file__).parents[3]/'code/vofa.c').read_text(encoding='utf-8')
    block=source.split('static uint8 cfg_live_control(')[1].split('static const char *cfg_group(')[0]
    assert set(re.findall(r'"([a-z_0-9]+)"',block))==set(LIVE_CONTROL_PARAMETERS)


def test_live_filter_and_same_screen_workbenches():
    from PySide6 import QtWidgets as W
    from app.ui.window import MainWindow
    app=W.QApplication.instance() or W.QApplication([])
    w=MainWindow(restore=False);w.persist_enabled=False;w.timer.stop();e=w.engine;e.auto_record=False
    try:
        e.connected=e.protocol_ready=True;e.status=[0]*15;e.status[1]=2;e.status[14]=0x1ffffff;e.status_time=time.monotonic()
        for name,group,flags in [('lean_speed_kp','Lean','21'),('cam_exposure','Camera','4')]:
            e.params[name]=e.profile.parameter(Frame('par',['1',name,'float','5','0','100',group,'0.1',flags],'',time.monotonic()))
        w.params.update_data();w.params.editable_only.setChecked(True);w.params.update_data()
        assert not w.params.table.isRowHidden(w.params.rows['lean_speed_kp'])
        assert w.params.table.isRowHidden(w.params.rows['cam_exposure'])
        assert [p.fixed_group for p,_ in w.workbenches][-2:]==['Run','Lean']
        e.status[14]=0xffffff;w.params.update_data()
        assert w.params.table.isRowHidden(w.params.rows['lean_speed_kp'])
    finally:w.close();w.deleteLater();app.processEvents()
