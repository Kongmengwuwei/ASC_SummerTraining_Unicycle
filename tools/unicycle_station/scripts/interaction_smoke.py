"""Mock-only visual checks at multiple densities; never persist user preferences."""
import sys,time,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from PySide6 import QtCore,QtGui,QtWidgets as W
from app.ui.window import MainWindow

root=Path(__file__).resolve().parents[1]
out=root/'docs/interaction-screenshots';out.mkdir(exist_ok=True)
fmt=QtGui.QSurfaceFormat();fmt.setDepthBufferSize(24);fmt.setSamples(4);QtGui.QSurfaceFormat.setDefaultFormat(fmt)
app=W.QApplication([]);window=MainWindow(restore=False);window.persist_enabled=False
window.engine.log_directory=root/'sessions/interaction-smoke'
window.show();window.connect_mock()
results=[]

def capture(name):
    window.grab().save(str(out/(name+'.png')))
    results.append({'name':name,'scale':window.interactions.factor,'size':[window.width(),window.height()],
                    'parameters':len(window.params.editors),'history':window.history.count(),'frames':window.engine.store.frame_count})

def step(index=0):
    if index==0:
        window.nav.setCurrentRow(4);window.params.update_data();window.params.expand_all(True)
    elif index==1:
        window.params.expand_all(False);window.params.toggle_group('Roll/rate');window.params.toggle_group('Roll/angle')
    elif index==2:window.interactions.set_zoom(.8)
    elif index==3:window.interactions.set_zoom(1.4)
    elif index==4:
        window.interactions.set_zoom(1);window.nav.setCurrentRow(0)
    elif index==5:
        window.interactions.set_zoom(.8);window.nav.setCurrentRow(0)
    elif index==6:
        window.interactions.set_zoom(1);window.nav.setCurrentRow(5)
    elif index==7:
        window.interactions.set_zoom(.8);window.nav.setCurrentRow(1)
    else:
        (out/'result.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
        window.close();app.quit();return
    def save():capture(['parameters-100','groups-100','groups-80','groups-140','overview-100','overview-80','workbench-100','scope-80'][index]);step(index+1)
    QtCore.QTimer.singleShot(700,save)

QtCore.QTimer.singleShot(5000,step)
QtCore.QTimer.singleShot(40000,app.quit)
sys.exit(app.exec())
