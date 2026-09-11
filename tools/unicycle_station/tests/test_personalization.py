import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import time
import pytest
from PySide6 import QtCore,QtWidgets as W
from app.core.models import Parameter,Frame
from app.services.engine import StationEngine
from app.services.task_state import RunSession
from app.ui.window import MainWindow,builtin_profile


@pytest.fixture
def window():
    app=W.QApplication.instance() or W.QApplication([])
    w=MainWindow(restore=False);w.persist_enabled=False;w.timer.stop();w.engine.auto_record=False
    w.engine.params={'r_rate_kp':Parameter('r_rate_kp',group='Roll',value=1,min=-100,max=100,label='横滚 KP')}
    w.params.update_data()
    yield w
    w.close();w.deleteLater();app.processEvents()


def feed(engine,tag,values):
    engine.ingest((tag+':'+','.join(map(str,values))+'\n').encode(),received=time.monotonic())


def task(engine,t,running,state=1):
    feed(engine,'task',[t,state,0,0,0,running,10,1,1,0,0,1,1])


def status(engine,t,running):
    feed(engine,'stat',[t,2,running,0,0,7,3,1,1,0,0,1,0,0,0])


def test_task_capture_is_run_only_and_restarts_at_origin():
    e=StationEngine(builtin_profile());e.auto_record=False
    try:
        status(e,0,0);task(e,0,0)
        feed(e,'taskevt',[1,0,0,1,0,0,0])
        assert e.task_observer.current is None and not e.task_observer.events and not e.trajectory.points
        status(e,1000,1);task(e,1000,1);task(e,1200,1)
        feed(e,'taskevt',[2,1200,1,4,0,0,1])
        assert e.run_session.active and e.trajectory.distance==pytest.approx(.2)
        assert e.task_observer.events
        status(e,1300,0)
        points=list(e.trajectory.points);events=list(e.task_observer.events);current=e.task_observer.current
        task(e,1400,0,state=3);feed(e,'taskevt',[3,1400,4,3,0,0,0])
        task(e,1200,1)  # buffered older packet cannot restart the finished Run
        assert not e.run_session.active and list(e.trajectory.points)==points
        assert list(e.task_observer.events)==events and e.task_observer.current==current
        status(e,2000,1)
        assert e.run_session.number==2 and not e.trajectory.points and not e.task_observer.events
        task(e,2000,1);assert e.trajectory.x==0 and len(e.trajectory.points)==1
    finally:e.close()


def test_run_capture_wrap_and_reboot():
    session=RunSession()
    def frame(t,on):return Frame('task',[t,1,0,0,0,on,0,1,1,0,0,0,0],'',0)
    assert session.observe(frame(0xffffff00,1))=='start'
    assert session.observe(frame(100,1)) is None and session.number==1
    session.observe(frame(5000,1))
    assert session.observe(frame(0,1)) is None and session.number==1
    assert session.observe(frame(100,1))=='start' and session.number==2
    assert session.observe(frame(200,0))=='stop'


def test_aliases_preserve_wire_names_search_undo_and_workspace(window):
    page=window.params;p=window.engine.params['r_rate_kp'];original=(p.name,p.group,p.label)
    page.ui.rename('parameter',p.name,'平衡响应');page.ui.rename('group','Roll/rate','内环精调');page.update_data()
    assert (p.name,p.group,p.label)==original
    assert '平衡响应' in page.table.item(page.rows[p.name],1).text()
    assert '内环精调' in page.table.item(page.group_rows['Roll/rate'],0).text()
    for query in ('平衡响应','r_rate_kp','内环精调'):
        page.search.setText(query);page.update_data();assert not page.table.isRowHidden(page.rows[p.name])
    window.history.undo();assert page.ui.group_label('Roll/rate')!='内环精调'
    window.history.redo();state=window.workspace();window.restore_workspace(state)
    assert window.params.ui.names['parameter'][p.name]=='平衡响应'
    assert window.params.ui.names['group']['Roll/rate']=='内环精调'
    assert window.engine.actions.empty()


def test_overview_selection_empty_state_readback_and_undo(window):
    view=window.overview;original=list(view.order)
    view.set_items(['roll','param:r_rate_kp']);assert view.order==['roll','param:r_rate_kp']
    window.engine.params['r_rate_kp'].pending=9;view.update_data()
    assert view.cards['param:r_rate_kp'].text()=='1'
    view.set_items([]);assert not view.order and not view.empty.isHidden()
    window.history.undo();assert view.order==['roll','param:r_rate_kp']
    window.history.undo();assert view.order==original
    view.set_items([]);state=window.workspace();window.restore_workspace(state)
    assert window.overview.order==[]


def test_waveform_restore_autorange_live_recovery_and_checkbox(window):
    window.show();window.nav.setCurrentRow(1);W.QApplication.processEvents()
    scope=window.scope;panel=scope.panels[0][1]
    panel.restore_state({'channels':['att_roll'],'ranges':[[0,10],[-.1,.1]]})
    assert panel.plot.getViewBox().autoRangeEnabled()[1]
    feed(window.engine,'att',[12,2,3]);feed(window.engine,'att',[14,3,4])
    panel.update_data();W.QApplication.processEvents();assert panel.curves['att_roll'].getData()[1][-1]==14
    panel.pause.setChecked(True);panel.follow.setChecked(False);panel.plot.setYRange(-.1,.1)
    panel.resume_live()
    assert not panel.pause.isChecked() and panel.follow.isChecked() and panel.plot.getViewBox().autoRangeEnabled()[1]
    scope.update_data();scope.target.setCurrentIndex(0)
    item=next(scope.tree.topLevelItem(i) for i in range(scope.tree.topLevelItemCount()) if scope.tree.topLevelItem(i).data(0,QtCore.Qt.UserRole)=='pitch')
    item.setCheckState(0,QtCore.Qt.Checked)
    assert 'pitch' in panel.channels
    window.history.undo();scope.update_data();assert item.checkState(0)==QtCore.Qt.Unchecked
    scope.show_attitude();assert panel.channels==['att_roll','pitch','yaw']


def test_waveform_empty_and_missing_channel_hints(window):
    panel=window.scope.panels[0][1]
    panel.set_channels([]);panel.update_data();assert '未选择' in panel.hint.text()
    panel.set_channels(['yaw_rate_actual']);panel.update_data();assert 'Run' in panel.hint.text()
    panel.pause.setChecked(True);panel.update_data();assert '已暂停' in panel.hint.text()


def test_group_alias_filter_keeps_fixed_flash_group(window):
    page=window.params
    page.ui.rename('group','Roll','横滚调参');page.update_data()
    page.group.setCurrentText('横滚调参')
    assert page.group.currentData()=='Roll'
    page.ui.edit({'r_rate_kp':2})
    assert page.changes()==[('r_rate_kp',2.0)]
    assert window.engine.params['r_rate_kp'].group=='Roll'


def test_old_snapshot_cannot_restart_stopped_run():
    e=StationEngine(builtin_profile());e.auto_record=False
    try:
        status(e,1000,1);task(e,1000,1);task(e,1200,1)
        status(e,10000,0);distance=e.trajectory.distance
        task(e,1500,1)
        assert not e.run_session.active and e.run_session.number==1 and e.trajectory.distance==distance
        status(e,10100,0)  # fresh status cancels the suspected reset
        task(e,1600,1)
        assert not e.run_session.active and e.run_session.number==1
        status(e,10200,0)
    finally:e.close()


def test_out_of_order_task_keeps_state_and_distance_monotonic():
    e=StationEngine(builtin_profile());e.auto_record=False
    try:
        task(e,1000,1);task(e,1200,1,state=4)
        task(e,1100,1,state=2);task(e,1200,1,state=2);task(e,1400,1,state=4)
        assert e.trajectory.distance==pytest.approx(.4) and e.trajectory.gaps==0
        assert e.task_observer.current[1]==4
        assert e.run_session.number==1
    finally:e.close()


def test_duplicate_and_old_trajectory_samples_do_not_replace_integration_base():
    from app.services.task_state import TrajectoryEstimator
    t=TrajectoryEstimator();t.sample(1000,1,0,True);t.sample(1200,1,0,True)
    t.sample(1100,90,90,True);t.sample(1200,90,90,True);t.sample(1400,1,0,True)
    assert t.distance==pytest.approx(.4) and t.y==0 and t.gaps==0


def test_non_ring_event_does_not_show_ring_phase():
    from app.services.task_state import TaskObserver
    observer=TaskObserver(builtin_profile())
    def evt(seq,state):return Frame('taskevt',[seq,seq*100,state,state,0,0,1],'',seq/10)
    messages=observer.accept(evt(1,4))
    assert messages==[('TASK_ENTER','首次观测 十字路口')]
    assert '环岛阶段' not in observer.accept(evt(2,4))[-1][1]
    observer.accept(evt(3,5))
    assert '环岛阶段' in observer.accept(evt(4,5))[-1][1]


def test_balance_diagnostic_frames_feed_waveforms_without_starting_run(window):
    e=window.engine
    channel=e.profile.channels['run'][1]['name']
    panel=window.scope.panels[0][1]
    panel.set_channels([channel])
    stamp=time.monotonic()-.1
    for i in range(3):
        values=[0.0]*25;values[0]=1000+i*20;values[1]=2+i;values[24]=2  # Balance, Run inactive.
        e.ingest(('run:'+','.join(map(str,values))+'\n').encode(),received=stamp+i*.02)
    panel.update_data()
    x,y=panel.curves[channel].getData()
    assert list(y)==[2,3,4]
    assert not e.run_session.active
    assert not e.trajectory.points
