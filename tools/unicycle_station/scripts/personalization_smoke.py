"""Run/session and personalization visual validation using only Mock transport."""
import sys,json,traceback,argparse
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from PySide6 import QtCore,QtGui,QtWidgets as W
from app.ui.window import MainWindow

root=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser();parser.add_argument('--output',default='docs/personalization-screenshots');args=parser.parse_args()
out=root/args.output;out.mkdir(parents=True,exist_ok=True)
fmt=QtGui.QSurfaceFormat();fmt.setDepthBufferSize(24);fmt.setSamples(4);QtGui.QSurfaceFormat.setDefaultFormat(fmt)
app=W.QApplication([]);window=MainWindow(restore=False);window.persist_enabled=False
window.engine.log_directory=root/'sessions'/out.name
window.show();window.connect_mock();results=[];frozen=None


def mode(name):
    with window.engine.transport.lock:
        window.engine.transport.manual_stopped=True;window.engine.transport.mode=name


def capture(name,widget=None):
    (widget or window).grab().save(str(out/(name+'.png')))
    e=window.engine
    results.append({'name':name,'size':[window.width(),window.height()],'scale':window.interactions.factor,
                    'run_active':e.run_session.active,'run_number':e.run_session.number,
                    'distance':e.trajectory.distance,'events':len(e.task_observer.events),'frames':e.store.frame_count})


def finish():
    (out/'result.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
    window.close();app.quit()


def step(index=0):
    global frozen
    try:
        if index==0:
            window.nav.setCurrentRow(3);assert not window.engine.run_session.active and not window.engine.trajectory.points
        elif index==1:mode('Run')
        elif index==2:mode('STOP')
        elif index==3:frozen=window.engine.trajectory.distance
        elif index==4:
            assert window.engine.trajectory.distance==frozen
            mode('Run')
        elif index==5:
            assert window.engine.run_session.number==2 and window.engine.trajectory.distance<frozen
            window.nav.setCurrentRow(4);page=window.params
            page.ui.rename('parameter','r_rate_kp','平衡响应');page.ui.rename('group','Roll/rate','横滚 · 内环精调')
            page.update_data();page.expand_all(False);page.toggle_group('Roll/rate')
        elif index==6:
            window.nav.setCurrentRow(0);window.overview.set_items(['att_roll','pitch','yaw','speed_actual','param:r_rate_kp','param:r_rate_kd'])
        elif index==7:
            def dialog_capture():
                dialog=next(w for w in app.topLevelWidgets() if isinstance(w,W.QDialog) and w.windowTitle()=='选择总览显示项目')
                capture('overview-selector',dialog);dialog.reject()
            QtCore.QTimer.singleShot(400,dialog_capture);window.overview.choose_items();step(8);return
        elif index==8:
            window.nav.setCurrentRow(1);window.scope.target.setCurrentIndex(0);window.scope.show_attitude();window.scope.resume_live()
        elif index==9:window.interactions.set_zoom(.8)
        else:finish();return
        names=['run-waiting','run-active','run-stopped','run-frozen','run-restarted','parameter-aliases','overview-custom','','waveforms-100','waveforms-80']
        def save():capture(names[index]);step(index+1)
        QtCore.QTimer.singleShot(1400 if index==1 else 700,save)
    except Exception:
        traceback.print_exc();results.append({'error':traceback.format_exc()});finish()


QtCore.QTimer.singleShot(200,lambda:mode('STOP'))
QtCore.QTimer.singleShot(3500,step)
QtCore.QTimer.singleShot(45000,finish)
sys.exit(app.exec())
