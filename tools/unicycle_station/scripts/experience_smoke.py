"""Mock-only visual checks; all outputs are disposable."""
import sys,json,time,traceback
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from PySide6 import QtCore,QtGui,QtWidgets as W
from app.ui.window import MainWindow
root=Path(__file__).resolve().parents[1];out=root/'artifacts/experience-smoke';out.mkdir(parents=True,exist_ok=True)
fmt=QtGui.QSurfaceFormat();fmt.setDepthBufferSize(24);fmt.setSamples(4);QtGui.QSurfaceFormat.setDefaultFormat(fmt)
app=W.QApplication([]);window=MainWindow(restore=False);window.persist_enabled=False;window.engine.log_directory=out/'sessions'
window.resize(1320,860);window.show();window.connect_mock();results=[];started=time.monotonic()

def fail():
    (out/'error.txt').write_text(traceback.format_exc(),encoding='utf-8');window.close();app.exit(1)

def capture(name):
    assert window.grab().save(str(out/(name+'.png')))
    results.append(dict(name=name,width=window.width(),height=window.height(),zoom=window.interactions.factor,columns=window.overview.columns))

def run(step=0):
    try:
        if step==0:
            if len(window.engine.params)<72:
                assert time.monotonic()-started<20
                QtCore.QTimer.singleShot(200,lambda:run(0));return
            capture('overview-100');window.resize(980,760);window.interactions.set_zoom(1.25)
        elif step==1:
            capture('overview-small-125');window.interactions.set_zoom(1);window.resize(1320,860);window.nav.setCurrentRow(1)
            window.scope.show_attitude() if hasattr(window.scope,'show_attitude') else None
        elif step==2:
            capture('scope');window.nav.setCurrentRow(window.nav.count()-1);window.engine.transport.enter_remote_demo()
        elif step==3:
            if not window.engine.remote.availability()[0]:
                assert time.monotonic()-started<25
                QtCore.QTimer.singleShot(200,lambda:run(3));return
            capture('remote-dark');window.toggle_theme()
        elif step==4:
            capture('remote-light');(out/'result.json').write_text(json.dumps(results,indent=2),encoding='utf-8');window.close();app.quit();return
        QtCore.QTimer.singleShot(1000,lambda:run(step+1))
    except Exception:fail()
QtCore.QTimer.singleShot(200,run)
raise SystemExit(app.exec())
