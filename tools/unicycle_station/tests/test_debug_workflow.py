import math
import json
import time
from pathlib import Path
import pytest
from app.core.models import Frame, ConnectionConfig, Parameter, write_document
from app.ui.window import builtin_profile
from app.services.task_state import TaskObserver, TrajectoryEstimator
from app.services.experiments import load_experiment, export_capture, TrialManager
from app.protocols.firewater import FireWaterParser
from app.plotting.measurement import points_with_gaps
from app.transports.serial_transport import matching_devices


def event(seq, uptime, previous, current, phase=0):
    return Frame('taskevt',[seq,uptime,previous,current,0,phase,1],'',uptime/1000)


def task(uptime, state=1, speed=1, yaw=0):
    return Frame('task',[uptime,state,0,0,0,1,10,1,1,0,yaw,speed,1],'',uptime/1000)


def test_task_transition_duplicate_missing_and_reboot():
    observer=TaskObserver(builtin_profile())
    first=observer.accept(event(1,1000,0,1))
    assert first[0][0]=='TASK_ENTER'
    assert observer.accept(event(1,1000,0,1))==[]
    messages=observer.accept(event(4,2000,1,4))
    assert [kind for kind,_ in messages]==['TASK_EVENT_GAP','TASK_EXIT','TASK_ENTER']
    assert '十字' in messages[-1][1]
    messages=observer.accept(event(1,100,0,1))
    assert messages[0][0]=='TASK_RESET' and messages[-1][0]=='TASK_ENTER'
    assert observer.last_seq==1


@pytest.mark.parametrize('raw',[
    'task:1,1,0,0,0,1,0,1,1,0,0,1',
    'task:1,13,0,0,0,1,0,1,1,0,0,1,1',
    'task:1,1,0,0,0,1,0,1.2,1,0,0,1,1',
    'task:1,1,0,0,0,1,0,1,1,0,nan,1,1',
    'taskevt:1,1,0,1,0,0,2',
    'taskevt:1.5,1,0,1,0,0,1',
])
def test_invalid_task_frames_rejected(raw):
    assert FireWaterParser().parse(raw,0).error


def test_trajectory_straight_signed_heading_gap_and_pause():
    estimator=TrajectoryEstimator()
    for t in range(0,1001,200):estimator.sample(t,1,0,True)
    assert estimator.x==pytest.approx(1) and estimator.y==0
    estimator.sample(3000,1,0,True)
    assert estimator.x==pytest.approx(1) and estimator.gaps==1
    assert any(math.isnan(p[0]) for p in estimator.points)
    estimator.sample(3200,1,0,False)
    estimator.sample(3400,1,0,True)
    assert estimator.x==pytest.approx(1)
    estimator.reset();estimator.yaw_sign=-1
    estimator.sample(0,1,90,True);estimator.sample(200,1,90,True)
    assert estimator.y==pytest.approx(-.2)
    estimator.reset();estimator.speed_sign=-1
    estimator.sample(0,1,0,True);estimator.sample(200,1,0,True)
    assert estimator.x==pytest.approx(-.2)


def test_trajectory_circle_wrap_and_source_change():
    estimator=TrajectoryEstimator()
    for i in range(101):estimator.sample(i*100,2*math.pi/10,i*3.6,True)
    assert abs(estimator.x)<1e-8 and abs(estimator.y)<1e-8
    assert estimator.distance==pytest.approx(2*math.pi)
    estimator.reset();estimator.sample(0xffffff9c,1,0,True);estimator.sample(100,1,0,True)
    assert estimator.x==pytest.approx(.2)
    estimator.reset();estimator.sample(0,1,0,True,'旧数据')
    estimator.accept(task(200,speed=10),{}, {})
    assert estimator.x==0
    estimator.accept(task(400,speed=10),{}, {})
    assert estimator.x==pytest.approx(2)
    assert estimator.points.maxlen==30000


def test_com_identity_survives_port_renumbering_and_reports_ambiguity():
    config=ConnectionConfig(port='COM1',device_vid=123,device_pid=456,device_serial='device')
    items=[dict(device='COM7',vid=123,pid=456,serial_number='device',location='1'),
           dict(device='COM8',vid=123,pid=456,serial_number='other',location='2')]
    assert matching_devices(config,items)==[items[0]]
    assert len(matching_devices(config,items+[dict(items[0],device='COM9')]))==2


def test_plot_data_gap_is_explicit():
    points=points_with_gaps([(0,1),(.02,2),(.04,3),(1,4)])
    assert len(points)==5 and math.isnan(points[3,1])


def test_trial_snapshot_comparison_capture_and_portable_session(tmp_path):
    session=tmp_path/'session';session.mkdir()
    write_document(session/'session.json','session',{'profile':builtin_profile().data})
    lines=[{'kind':'frame','time':t,'tag':'att','values':[t,2*t,3*t]} for t in range(6)]
    lines.append({'kind':'frame','time':2,'tag':'par','values':['1','future','float','99','0','100','Roll','1','4']})
    (session/'frames.jsonl').write_text(''.join(json.dumps(item)+'\n' for item in lines),encoding='utf-8')
    path=session/'trials'/'one.trial.json'
    write_document(path,'trial',{'session':str(tmp_path/'missing'),'start':1,'end':4,'name':'one','start_parameters':{'gain':{'value':2}}})
    result=load_experiment(path)
    assert result['parameters']=={'gain':{'value':2}}
    assert result['stats']['att_roll']['mean']==pytest.approx(2.5)
    assert result['stats']['att_roll']['count']==4
    capture=export_capture(session,1,3,'PROTECTION','test')
    result=load_experiment(capture)
    assert result['metadata']['capture']['tail_complete']
    assert result['stats']['att_roll']['count']==3


def test_archive_completion_visible_only_after_success(tmp_path):
    from concurrent.futures import Future
    manager=TrialManager(None)
    success=Future();failure=Future()
    manager.jobs.extend([success,failure]);manager.archive_paths={success:'ok',failure:'bad'}
    manager.reap();assert not manager.completed
    success.set_result(None);failure.set_exception(OSError('disk failure'))
    manager.reap()
    assert list(manager.completed)==['ok'] and manager.error=='disk failure'
    assert not manager.jobs and not manager.archive_paths


def test_automatic_trial_short_stop_and_resume_keeps_tail_open():
    manager=TrialManager(None)
    manager.current={"automatic":True};manager.tail_deadline=1
    manager.start(automatic=True)
    assert manager.current and manager.tail_deadline is None


def test_status_after_reboot_event_does_not_erase_new_sequence():
    observer=TaskObserver(builtin_profile())
    observer.accept(task(5000));observer.accept(event(10,5000,0,1))
    observer.accept(event(1,100,0,1));observer.accept(task(200))
    assert observer.accept(event(1,100,0,1))==[]
