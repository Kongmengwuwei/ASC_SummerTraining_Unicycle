"""Window-scoped UI zoom, leaving plain-wheel plot/camera controls intact."""
import re
from shiboken6 import isValid
from PySide6 import QtCore,QtGui,QtWidgets as W


class UiInteractions(QtCore.QObject):
    def __init__(self,window):
        super().__init__(window)
        self.window=window;self.factor=1.0;self.busy=False;self.wheel_delta=0
        self.pending_factor=1.0
        self.zoom_timer=QtCore.QTimer(self);self.zoom_timer.setSingleShot(True);self.zoom_timer.setInterval(35)
        self.zoom_timer.timeout.connect(lambda:self.set_zoom(self.pending_factor))
        W.QApplication.instance().installEventFilter(self)

    def owns(self,obj):
        return isValid(self.window) and isinstance(obj,W.QWidget) and (obj is self.window or self.window.isAncestorOf(obj))

    def parameter_parent(self,obj):
        while isinstance(obj,W.QWidget):
            if obj.property('parameter_page'):return obj
            obj=obj.parentWidget()
        return None

    def eventFilter(self,obj,event):
        if self.busy or not self.owns(obj):return False
        kind=event.type()
        if kind==QtCore.QEvent.Wheel and event.modifiers()&QtCore.Qt.ControlModifier:
            delta=event.angleDelta().y() or event.pixelDelta().y()*3
            self.wheel_delta+=delta
            if abs(self.wheel_delta)>=120:
                steps=int(self.wheel_delta/120);self.wheel_delta-=steps*120
                base=self.pending_factor if self.zoom_timer.isActive() else self.factor
                self.pending_factor=max(.7,min(1.6,round(base+steps*.05,2)))
                if not self.zoom_timer.isActive():self.zoom_timer.start()
            event.accept();return True
        if kind in (QtCore.QEvent.ShortcutOverride,QtCore.QEvent.KeyPress) and event.modifiers()&QtCore.Qt.ControlModifier:
            key=event.key()
            if key==QtCore.Qt.Key_0:
                if kind==QtCore.QEvent.KeyPress:self.set_zoom(1)
                event.accept();return True
            if key in (QtCore.Qt.Key_Z,QtCore.Qt.Key_Y) and self.parameter_parent(obj):
                if kind==QtCore.QEvent.KeyPress:
                    redo=key==QtCore.Qt.Key_Y or bool(event.modifiers()&QtCore.Qt.ShiftModifier)
                    # Text still being typed uses the line editor's native undo.
                    line=obj if isinstance(obj,W.QLineEdit) else None
                    draft=False
                    if line:
                        parent=line.parentWidget()
                        if isinstance(parent,W.QAbstractSpinBox):
                            draft=line.text()!=parent.textFromValue(parent.value())
                        else:draft=line.isModified()
                    if line and (draft or line.property("_parameter_native_undo")) and (line.isRedoAvailable() if redo else line.isUndoAvailable()):
                        line.redo() if redo else line.undo()
                        line.setProperty("_parameter_native_undo",True)
                    else:
                        self.window.history.redo() if redo else self.window.history.undo()
                        self.parameter_parent(obj).update_data()
                event.accept();return True
        if kind in (QtCore.QEvent.StyleChange,QtCore.QEvent.Show):
            self.scale_widget(obj)
        return False

    def scale_widget(self,widget):
        self.busy=True
        try:
            style=widget.styleSheet()
            if style!=widget.property('_zoom_applied_style'):
                widget.setProperty('_zoom_base_style',style)
            base=widget.property('_zoom_base_style') or ''
            scaled=re.sub(r'(?<![\w#])([0-9]+(?:\.[0-9]+)?)(px|pt)\b',lambda m:f'{float(m[1])*self.factor:g}{m[2]}',base)
            widget.setProperty('_zoom_applied_style',scaled)
            if scaled!=style:widget.setStyleSheet(scaled)
            # Fonts inherit the single root QSS size. Per-widget font multiplication
            # captured already-scaled inherited fonts and caused cumulative drift.
            if widget is not self.window and not isinstance(widget,(W.QDockWidget,W.QStackedWidget,W.QToolBar,W.QScrollArea)):
                if widget.property('_zoom_limits') is None:
                    widget.setProperty('_zoom_limits',[widget.minimumWidth(),widget.minimumHeight(),widget.maximumWidth(),widget.maximumHeight()])
                limits=widget.property('_zoom_limits')
                adjusted=[round(v*self.factor) if v<16777215 else v for v in limits]
                widget.setMinimumSize(*adjusted[:2]);widget.setMaximumSize(*adjusted[2:])
            layout=widget.layout()
            if layout:
                if layout.property('_zoom_margins') is None:
                    m=layout.contentsMargins();layout.setProperty('_zoom_margins',[m.left(),m.top(),m.right(),m.bottom()]);layout.setProperty('_zoom_spacing',layout.spacing())
                layout.setContentsMargins(*(round(v*self.factor) for v in layout.property('_zoom_margins')))
                spacing=layout.property('_zoom_spacing')
                if spacing>=0:layout.setSpacing(round(spacing*self.factor))
        finally:self.busy=False

    def dispose(self):
        app=W.QApplication.instance()
        self.zoom_timer.stop()
        if app:app.removeEventFilter(self)

    def set_zoom(self,factor):
        self.zoom_timer.stop()
        self.factor=max(.7,min(1.6,round(float(factor),2)))
        self.pending_factor=self.factor
        window=self.window
        scroll=window.stack.currentWidget()
        bar=scroll.verticalScrollBar() if isinstance(scroll,W.QScrollArea) else None
        ratio=bar.value()/max(1,bar.maximum()) if bar else 0
        window.setUpdatesEnabled(False)
        try:
            widgets=window.findChildren(W.QWidget)
            for widget in widgets:self.scale_widget(widget)
            self.scale_widget(window)
            for page in [window.params]+[p for p,_ in window.workbenches]:page.set_zoom(self.factor)
            window.overview.reflow(self.factor)
            window.nav.setIconSize(QtCore.QSize(round(18*self.factor),round(18*self.factor)))
            import pyqtgraph as pg
            for plot in window.findChildren(pg.PlotWidget):
                font=QtGui.QFont('Microsoft YaHei UI');font.setPixelSize(round(12*self.factor))
                for name in ('bottom','left'):plot.getAxis(name).setStyle(tickFont=font)
            window.zoom_label.setText(f'{self.factor:.0%} ▾')
            window.zoom_label.setToolTip('Ctrl + 滚轮：每格 5% · Ctrl + 0：恢复 100% · 点击选择比例')
        finally:window.setUpdatesEnabled(True)
        def restore_position():
            if bar is not None and isValid(bar):bar.setValue(round(ratio*bar.maximum()))
        QtCore.QTimer.singleShot(0,restore_position)
