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


def test_direction_switches_persist_without_changing_raw_data(qtapp):
    from app.core.models import Frame
    window=MainWindow(restore=False)
    try:
        view=window.orientation
        window.engine.store.accept(Frame("att",[10,20,30],"",time.monotonic()))
        for axis,box in enumerate(view.axis_switches):
            box.setChecked(True)
            assert view.mapping["signs"][axis]==-1
            box.setChecked(False)
            assert view.mapping["signs"][axis]==1
        assert window.engine.store.latest["att_roll"]==10
        assert window.engine.store.latest["pitch"]==20
        view.axis_switches[0].setChecked(True)
        workspace=window.workspace()
        window.restore_workspace(workspace)
        assert window.orientation.axis_switches[0].isChecked()
        assert not window.orientation.axis_switches[1].isChecked()
    finally:window.engine.close();window.deleteLater()


def test_parameter_editor_identity_and_ring_filter(qtapp):
    window=MainWindow(restore=False)
    e=window.engine;e.connected=e.protocol_ready=True;e.status=[0]*15;e.status_time=time.monotonic()
    e.params={"r_rate_kp":Parameter("r_rate_kp",group="Roll",value=1),"r_angle_kp":Parameter("r_angle_kp",group="Roll",value=2)}
    try:
        params,plots=window.workbenches[0]
        params.update_data();editor=params.editors["r_rate_kp"]
        params.table.selectRow(params.rows["r_rate_kp"])
        e.params["r_rate_kp"].value=5;params.update_data()
        assert params.editors["r_rate_kp"] is editor and editor.value()==5
        params.nudge(1,False);params.update_data()
        assert params.editors["r_rate_kp"] is editor
        assert e.params["r_rate_kp"].pending>5
        plots.ring.setCurrentIndex(1)
        assert params.table.isRowHidden(params.rows["r_angle_kp"])
        assert not params.table.isRowHidden(params.rows["r_rate_kp"])
        e.favorites.add("r_rate_kp");window.params.update_data()
        assert window.params.table.cellWidget(window.params.rows["r_rate_kp"],0).isChecked()
    finally:e.close();window.deleteLater()


def test_scope_linked_controls_and_layout_restore(qtapp):
    window=MainWindow(restore=False)
    try:
        panels=[panel for _,panel in window.scope.panels]
        panels[0].window.setCurrentText("30")
        assert all(panel.window.currentText()=="30" for panel in panels)
        panels[0].pause.setChecked(True)
        assert all(panel.pause.isChecked() for panel in panels)
        state=window.workspace();window.restore_workspace(state)
        assert all(panel.window.currentText()=="30" for _,panel in window.scope.panels)
        assert window.scope.linked.isChecked()
    finally:window.engine.close();window.deleteLater()


def test_background_experiment_load_and_old_workspace_upgrade(qtapp):
    window=MainWindow(restore=False)
    try:
        workspace=window.workspace()
        for key in ("task_states","value_labels","bit_labels"):workspace["profile"].pop(key,None)
        for tag in ("task","taskevt"):workspace["profile"]["channels"].pop(tag,None)
        window.restore_workspace(workspace)
        assert window.task_page is not None
        demo=Path(__file__).parents[1]/"examples/demo_session/frames.jsonl"
        page=window.experiments
        page.load("A",str(demo));page.load("B",str(demo))
        deadline=time.monotonic()+10
        while page.jobs and time.monotonic()<deadline:
            qtapp.processEvents();time.sleep(.01)
        assert not page.jobs and set(page.datasets)=={"A","B"}
        assert len(page.curves["A"].xData)>0
        assert page.diff.rowCount()==0
        assert "RMS" in page.summary.text()
    finally:window.engine.close();window.deleteLater()


def test_validation_mode_cannot_overwrite_saved_workspace(qtapp):
    window=MainWindow(restore=False)
    class Settings:
        def setValue(self,*args):raise AssertionError("must not save validation preferences")
    try:
        window.settings=Settings();window.persist_enabled=False;window.persist()
    finally:window.engine.close();window.deleteLater()
