import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from PySide6 import QtCore,QtGui,QtWidgets as W,QtTest
from app.ui.window import MainWindow


def test_responsive_zoom_and_motion_preferences():
    app=W.QApplication.instance() or W.QApplication([])
    window=MainWindow(restore=False);window.persist_enabled=False;window.engine.auto_record=False
    try:
        window.show();app.processEvents()
        window.resize(1400,850);app.processEvents();wide=window.overview.columns
        window.resize(850,700);app.processEvents();assert window.overview.columns<wide
        window.interactions.set_zoom(1.6);app.processEvents();large=window.overview.columns
        window.interactions.set_zoom(.7);app.processEvents();assert window.overview.columns>large
        window.interactions.set_zoom(1);app.processEvents()
        label=window.overview.state;size=label.font().pixelSize()
        for scale in (1.4,.8,1.6,1):window.interactions.set_zoom(scale)
        app.processEvents();assert label.font().pixelSize()==size
        window.set_motion(False);saved=window.workspace();window.restore_workspace(saved)
        assert not window.nav.motion_enabled and window.interactions.factor==1
    finally:window.close();window.deleteLater();app.processEvents()


def test_remote_keys_release_on_page_change_and_focus_loss():
    app=W.QApplication.instance() or W.QApplication([])
    window=MainWindow(restore=False);window.persist_enabled=False;window.timer.stop();e=window.engine;e.auto_record=False
    try:
        window.show();window.nav.setCurrentRow(window.nav.count()-1);app.processEvents()
        page=window.remote_page;r=e.remote
        r.armed=True
        QtTest.QTest.keyPress(page,QtCore.Qt.Key_W);QtTest.QTest.keyPress(page,QtCore.Qt.Key_A)
        assert page.keys=={QtCore.Qt.Key_W,QtCore.Qt.Key_A}
        QtTest.QTest.keyRelease(page,QtCore.Qt.Key_W);assert page.keys=={QtCore.Qt.Key_A}
        W.QApplication.sendEvent(window,QtCore.QEvent(QtCore.QEvent.WindowDeactivate))
        assert not r.armed and not page.keys
        r.armed=True;page.mouse.add('forward');window.nav.setCurrentRow(0);app.processEvents()
        assert not r.armed and not page.mouse
    finally:window.close();window.deleteLater();app.processEvents()
