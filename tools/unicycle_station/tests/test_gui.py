import os
os.environ.setdefault("QT_QPA_PLATFORM","offscreen")
from pathlib import Path
import time
import pytest
from PySide6 import QtWidgets as W
from app.ui.window import MainWindow
from app.core.models import DeviceProfile, Parameter


@pytest.fixture(scope="module")
def qtapp():
    app=W.QApplication.instance() or W.QApplication([])
    return app


def test_workspace_profile_switch_and_small_window(qtapp):
    window=MainWindow(restore=False)
    window.engine.auto_record=False
    try:
        window.resize(1100,760)
        workspace=window.workspace()
        workspace["plots"][0]["seconds"]=30
        window.restore_workspace(workspace)
        assert window.scope.panels[0][1].window.currentText()=="30"
        generic=DeviceProfile.load(Path(__file__).parents[1]/"app/profiles/generic_sensor/profile.json")
        window.change_profile(generic)
        names=[window.nav.item(i).text() for i in range(window.nav.count())]
        assert "3D 姿态" not in names and "调参工作台" not in names and "参数调节" not in names
        assert not window.engine.params and not window.engine.store.latest
    finally:window.engine.close();window.deleteLater()


def test_schema_controls_and_permissions(qtapp):
    window=MainWindow(restore=False)
    e=window.engine;e.connected=e.protocol_ready=True;e.status=[0]*15;e.status_time=time.monotonic()
    e.params={"dynamic_new":Parameter("dynamic_new",label="动态参数",value=1),
              "danger":Parameter("danger",dangerous=True),"boolean":Parameter("boolean",type="bool"),
              "text":Parameter("text",type="string",value="example")}
    try:
        window.params.update_data()
        assert set(window.params.editors)==set(e.params)
        assert window.params.editors["dynamic_new"].isEnabled()
        assert not window.params.editors["danger"].isEnabled()
        window.params.advanced.setChecked(True);window.params.update_data()
        assert window.params.editors["danger"].isEnabled()
        e.status_time-=1;window.params.update_data()
        assert all(not editor.isEnabled() for editor in window.params.editors.values())
    finally:e.close();window.deleteLater()


def test_orientation_stale_freezes_display(qtapp):
    window=MainWindow(restore=False);view=window.orientation
    from app.core.models import Frame
    import numpy as np
    try:
        window.engine.store.accept(Frame("att",[10,20,30],"",time.monotonic()))
        view.update_data();before=view.display_rotation.copy()
        for name in view.mapping["channels"]:window.engine.store.stamps[name]-=2
        view.update_data()
        assert "STALE" in view.caption.text()
        assert np.array_equal(before,view.display_rotation)
    finally:window.engine.close();window.deleteLater()
