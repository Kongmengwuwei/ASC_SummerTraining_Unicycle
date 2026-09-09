import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import time
import json
import pytest
from PySide6 import QtCore,QtGui,QtWidgets as W,QtTest
from app.ui.window import MainWindow
from app.core.models import Parameter
from app.ui.parameter_table import ParameterTable


@pytest.fixture
def window():
    app=W.QApplication.instance() or W.QApplication([])
    window=MainWindow(restore=False);window.persist_enabled=False;window.timer.stop()
    e=window.engine;e.auto_record=False;e.connected=e.protocol_ready=True;e.status=[0]*15;e.status_time=time.monotonic()
    e.params={name:Parameter(name,group=group,value=value,min=-100,max=100,step=.1) for name,group,value in
              [('r_rate_kp','Roll',1),('r_rate_ki','Roll',2),('r_angle_kp','Roll',3),('p_rate_kp','Pitch',4),('cam_exposure','Camera',5)]}
    window.params.update_data()
    yield window
    window.close();window.deleteLater();app.processEvents()


def test_parameter_history_edit_nudge_clear_and_redo(window):
    page=window.params;e=window.engine
    page.pending_changed('r_rate_kp',2)
    page.table.selectRow(page.rows['r_rate_kp']);page.nudge(1,False)
    assert e.params['r_rate_kp'].pending==pytest.approx(2.1)
    window.history.undo();assert e.params['r_rate_kp'].pending==2
    window.history.undo();assert e.params['r_rate_kp'].pending is None
    window.history.redo();assert e.params['r_rate_kp'].pending==2
    page.revert_pending();assert e.params['r_rate_kp'].pending is None
    window.history.undo();assert e.params['r_rate_kp'].pending==2
    assert e.params['r_rate_kp'].value==1 and e.actions.empty()


def test_undo_after_mcu_apply_only_stages_and_inflight_ack_preserves_it(window):
    e=window.engine;page=window.params;p=e.params['r_rate_kp'];callbacks=[]
    page.pending_changed(p.name,2)
    e.requests.enqueue=lambda *args,**kw:callbacks.append(kw['callback'])
    e._apply([(p.name,2)])
    window.history.undo();assert p.pending is None
    callbacks.pop()(True,['set',p.name,'2','APPLIED'])
    assert p.value==2 and p.pending==1
    assert e.actions.empty() and not callbacks
    window.history.redo();assert p.pending is None
    window.history.undo();assert p.pending==1


def test_group_collapse_search_and_shared_layout(window):
    page=window.params
    assert set(page.group_rows)=={'Roll/rate','Roll/angle','Pitch/rate','Camera'}
    original=page.editors['r_rate_kp']
    page.toggle_group('Roll/rate');assert page.table.isRowHidden(page.rows['r_rate_kp'])
    page.search.setText('r_rate_kp');page.update_data()
    assert not page.table.isRowHidden(page.rows['r_rate_kp'])
    assert page.table.isRowHidden(page.group_rows['Camera'])
    page.search.clear();page.update_data()
    assert page.table.isRowHidden(page.rows['r_rate_kp']) and page.editors['r_rate_kp'] is original
    bench=window.workbenches[0][0];bench.update_data()
    assert bench.ui is page.ui and bench.table.isRowHidden(bench.rows['r_rate_kp'])


def test_group_and_parameter_drop_order_undo_and_workspace(window):
    page=window.params
    assert page.ui.move(('group','Camera'),('group','Roll/rate'))
    page.update_data();assert page.group_rows['Camera']==0
    assert page.ui.move(('parameter','r_rate_kp'),('parameter','r_rate_ki'),True)
    page.update_data();assert page.rows['r_rate_ki']<page.rows['r_rate_kp']
    assert not page.ui.move(('parameter','r_rate_kp'),('parameter','cam_exposure'))
    window.history.undo();page.update_data();assert page.rows['r_rate_kp']<page.rows['r_rate_ki']
    page.toggle_group('Roll/rate')
    workspace=window.workspace();params=window.engine.params
    window.restore_workspace(workspace);window.engine.params=params;window.params.update_data()
    assert window.params.group_rows['Camera']==0
    assert window.params.table.isRowHidden(window.params.rows['r_rate_kp'])


def test_drop_handler_uses_display_metadata(window):
    page=window.params;table=page.table
    mime=QtCore.QMimeData();mime.setData(ParameterTable.MIME,json.dumps(['parameter','r_rate_kp']).encode())
    row=page.rows['r_rate_ki']
    class Drop:
        accepted=False
        def source(self):return table
        def mimeData(self):return mime
        def position(self):return QtCore.QPointF(20,table.rowViewportPosition(row)+table.rowHeight(row)-1)
        def acceptProposedAction(self):self.accepted=True
        def ignore(self):self.accepted=False
    event=Drop();table.dropEvent(event)
    assert event.accepted
    page.update_data();assert page.rows['r_rate_ki']<page.rows['r_rate_kp']


def test_ctrl_wheel_changes_ui_not_parameter_or_plot_range(window):
    window.show();window.nav.setCurrentRow(4);W.QApplication.processEvents()
    window.engine.status_time=time.monotonic()
    window.params.update_data()
    editor=window.params.editors['r_rate_kp'];before=editor.value()
    event=QtGui.QWheelEvent(QtCore.QPointF(5,5),QtCore.QPointF(editor.mapToGlobal(QtCore.QPoint(5,5))),QtCore.QPoint(),QtCore.QPoint(0,120),QtCore.Qt.NoButton,QtCore.Qt.ControlModifier,QtCore.Qt.NoScrollPhase,False)
    W.QApplication.sendEvent(editor,event)
    assert window.interactions.factor==1.1 and editor.value()==before
    assert window.engine.params['r_rate_kp'].pending is None
    window.interactions.set_zoom(.7);height=window.params.table.rowHeight(window.params.rows['r_rate_kp'])
    window.interactions.set_zoom(1.6);assert window.params.table.rowHeight(window.params.rows['r_rate_kp'])>height
    QtTest.QTest.keyClick(editor,QtCore.Qt.Key_0,QtCore.Qt.ControlModifier)
    assert window.interactions.factor==1


def test_ctrl_z_in_parameter_editor_and_redo(window):
    window.show();window.nav.setCurrentRow(4);W.QApplication.processEvents()
    page=window.params;window.engine.status_time=time.monotonic();page.pending_changed('r_rate_kp',9);page.update_data()
    editor=page.editors['r_rate_kp'];editor.setFocus()
    QtTest.QTest.keyClick(editor,QtCore.Qt.Key_Z,QtCore.Qt.ControlModifier)
    assert window.engine.params['r_rate_kp'].pending is None and editor.value()==1
    QtTest.QTest.keyClick(editor,QtCore.Qt.Key_Z,QtCore.Qt.ControlModifier|QtCore.Qt.ShiftModifier)
    assert window.engine.params['r_rate_kp'].pending==9 and editor.value()==9


def test_column_order_zoom_roundtrip_and_history(window):
    header=window.params.table.horizontalHeader();header.moveSection(7,2)
    assert header.logicalIndex(2)==7
    window.history.undo();assert header.logicalIndex(2)==2
    window.history.redo();window.interactions.set_zoom(.8)
    state=window.workspace();window.restore_workspace(state)
    assert window.interactions.factor==.8
    assert window.params.table.horizontalHeader().logicalIndex(2)==7
    assert window.history.count()==0


def test_batch_history_is_atomic_on_invalid_import(window):
    page=window.params
    with pytest.raises(ValueError):page.ui.edit({'r_rate_kp':5,'r_rate_ki':'not a number'},'导入预设')
    assert all(p.pending is None for p in window.engine.params.values())
    page.ui.edit({'r_rate_kp':5,'r_rate_ki':6},'导入预设')
    window.history.undo();assert all(p.pending is None for p in window.engine.params.values())


def test_channel_order_stable_and_card_history(window):
    scope=window.scope;scope.update_data();original=scope.tree.order();order=list(reversed(original))
    scope.reorder_channels(original,order);item=scope.tree.topLevelItem(0)
    scope.update_data();assert scope.tree.topLevelItem(0) is item
    window.history.undo();assert scope.tree.order()==original
    original=list(window.overview.order)
    from app.ui.edit_history import change
    change(window.history,'调整总览卡片顺序',original,list(reversed(original)),window.overview.restore_order)
    assert window.overview.order==list(reversed(original))
    window.history.undo();assert window.overview.order==original


def test_history_skips_deleted_display(window):
    from app.ui.edit_history import change
    widget=W.QLabel('before')
    change(window.history,'显示修改','before','after',widget.setText)
    widget.deleteLater()
    W.QApplication.sendPostedEvents(None,QtCore.QEvent.DeferredDelete)
    window.history.undo()
    assert window.history.count()==0


def test_native_draft_undo_does_not_commit_parameter(window):
    window.show();window.nav.setCurrentRow(4);W.QApplication.processEvents()
    page=window.params;window.engine.status_time=time.monotonic();page.update_data()
    editor=page.editors['r_rate_kp'];line=editor.lineEdit();line.setFocus();line.selectAll()
    QtTest.QTest.keyClicks(line,'12')
    QtTest.QTest.keyClick(line,QtCore.Qt.Key_Z,QtCore.Qt.ControlModifier)
    assert window.engine.params['r_rate_kp'].pending is None
    QtTest.QTest.keyClick(line,QtCore.Qt.Key_Z,QtCore.Qt.ControlModifier|QtCore.Qt.ShiftModifier)
    assert line.text()=='12' and window.engine.params['r_rate_kp'].pending is None
    QtTest.QTest.keyClick(line,QtCore.Qt.Key_Return)
    page.update_data()
    QtTest.QTest.keyClick(line,QtCore.Qt.Key_Z,QtCore.Qt.ControlModifier)
    assert window.engine.params['r_rate_kp'].pending is None and editor.value()==1


def test_orientation_direction_undo_keeps_device_untouched(window):
    from copy import deepcopy
    view=window.orientation;before=deepcopy(view.mapping)
    view.flip_axis(0,before.get('signs',[1,1,1])[0]!=-1)
    assert view.mapping!=before
    window.history.undo();assert view.mapping==before
    window.history.redo();assert view.mapping!=before
    assert window.engine.actions.empty()
