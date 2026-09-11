"""Mock-only live tuning and graph visual validation."""
import sys,time,json,traceback
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from PySide6 import QtCore,QtWidgets as W
from app.ui.window import MainWindow
root=Path(__file__).resolve().parents[1];out=root/'artifacts/live-tuning-smoke';out.mkdir(parents=True,exist_ok=True)
app=W.QApplication([]);w=MainWindow(restore=False);w.persist_enabled=False;w.resize(1440,940);w.engine.log_directory=out/'sessions';w.show();w.connect_mock();deadline=time.monotonic()+25

def bench(index):
    w.nav.setCurrentRow(5)
    parent=w.workbenches[index][0].parentWidget()
    while not isinstance(parent,W.QTabWidget):parent=parent.parentWidget()
    parent.setCurrentIndex(index)

def step(stage=0):
    try:
        e=w.engine
        if stage==0:
            if len(e.params)<72:
                assert time.monotonic()<deadline
                QtCore.QTimer.singleShot(100,lambda:step(0));return
            with e.transport.lock:e.transport.mode='Balance';e.transport.manual_stopped=True
            bench(3)
        elif stage==1:
            assert e.permission(e.params['run_speed_straight'])[0]
            w.grab().save(str(out/'speed-steering.png'));bench(4)
            e.submit('apply',[('lean_roll_kp',3.0)],False)
        elif stage==2:
            if e.params['lean_roll_kp'].value!=3:
                assert time.monotonic()<deadline
                QtCore.QTimer.singleShot(100,lambda:step(2));return
            w.workbenches[4][0].update_data();w.grab().save(str(out/'lean.png'))
            assert not e.run_session.active
            (out/'result.json').write_text(json.dumps(dict(mode=e.transport.mode,lean=e.params['lean_roll_kp'].value,run_active=e.run_session.active,pages=2)),encoding='utf-8')
            w.close();app.quit();return
        QtCore.QTimer.singleShot(1200,lambda:step(stage+1))
    except Exception:
        (out/'error.txt').write_text(traceback.format_exc(),encoding='utf-8');w.close();app.exit(1)
QtCore.QTimer.singleShot(200,step)
raise SystemExit(app.exec())
