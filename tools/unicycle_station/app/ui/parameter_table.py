import json
from PySide6 import QtCore,QtGui,QtWidgets as W


class ParameterTable(W.QTableWidget):
    """Keep live editors stable. Row drags reorder display metadata only."""
    MIME='application/x-embedded-station-parameter-order'
    def __init__(self,page):
        super().__init__(0,8)
        self.page=page;self.entries={};self._pressed=None
        self.drop_line=W.QFrame(self.viewport());self.drop_line.setStyleSheet("background:#56b6f7;");self.drop_line.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents);self.drop_line.hide()
        self.drag_pos=None;self.scroll_timer=QtCore.QTimer(self);self.scroll_timer.setInterval(80);self.scroll_timer.timeout.connect(self.drag_scroll)
        self.setDragEnabled(True);self.setAcceptDrops(True)
        self.setDragDropMode(W.QAbstractItemView.DragDrop)
        self.setDefaultDropAction(QtCore.Qt.MoveAction);self.setDropIndicatorShown(True)
        self.setEditTriggers(W.QAbstractItemView.NoEditTriggers)
        self.verticalHeader().hide()
        self.setToolTip('拖动分组标题调整分组顺序；拖动参数名称调整组内顺序；点击分组标题展开/收起')

    def resizeEvent(self,event):
        super().resizeEvent(event)
        if hasattr(self.page,"table"):self.page.fit_name_column()

    def mousePressEvent(self,event):
        row=self.rowAt(int(event.position().y()));col=self.columnAt(int(event.position().x()))
        self._pressed=(row,col,event.position().toPoint()) if event.button()==QtCore.Qt.LeftButton else None
        super().mousePressEvent(event)

    def mouseReleaseEvent(self,event):
        if self._pressed:
            row,_,pos=self._pressed
            if (event.position().toPoint()-pos).manhattanLength()<W.QApplication.startDragDistance():
                entry=self.entries.get(row)
                if entry and entry[0]=='group':self.page.toggle_group(entry[1])
        self._pressed=None;super().mouseReleaseEvent(event)

    def startDrag(self,actions):
        if not self._pressed:return
        row,col,_=self._pressed;entry=self.entries.get(row)
        if not entry or (entry[0]=='parameter' and col!=1):return
        mime=QtCore.QMimeData();mime.setData(self.MIME,json.dumps(entry).encode())
        drag=QtGui.QDrag(self);drag.setMimeData(mime)
        self._pressed=None;drag.exec(QtCore.Qt.MoveAction)

    def dragEnterEvent(self,event):
        if event.source() is self and event.mimeData().hasFormat(self.MIME):event.acceptProposedAction()
        else:event.ignore()

    def show_drop_line(self):
        if self.drag_pos is None:return
        row=self.rowAt(self.drag_pos.y())
        if row<0:self.drop_line.hide();return
        top=self.rowViewportPosition(row);height=self.rowHeight(row)
        y=top+height if self.drag_pos.y()>top+height/2 else top
        self.drop_line.setGeometry(0,y-1,self.viewport().width(),2);self.drop_line.show();self.drop_line.raise_()

    def drag_scroll(self):
        if self.drag_pos is None:return
        y=self.drag_pos.y();bar=self.verticalScrollBar()
        step=-1 if y<28 else 1 if y>self.viewport().height()-28 else 0
        if step:bar.setValue(bar.value()+step*bar.singleStep());self.show_drop_line()

    def dragMoveEvent(self,event):
        if event.source() is self and event.mimeData().hasFormat(self.MIME):
            self.drag_pos=event.position().toPoint();self.show_drop_line();self.scroll_timer.start();event.acceptProposedAction()
        else:event.ignore()

    def dragLeaveEvent(self,event):
        self.drop_line.hide();self.scroll_timer.stop();self.drag_pos=None;event.accept()

    def dropEvent(self,event):
        self.drop_line.hide();self.scroll_timer.stop();self.drag_pos=None
        if event.source() is not self or not event.mimeData().hasFormat(self.MIME):event.ignore();return
        try:source=tuple(json.loads(bytes(event.mimeData().data(self.MIME))))
        except (ValueError,TypeError):event.ignore();return
        row=self.rowAt(int(event.position().y()));target=self.entries.get(row)
        if target and source[0]=='group' and target[0]=='parameter':
            from app.ui.edit_history import display_group
            target=('group',display_group(self.page.engine.params[target[1]]))
        after=event.position().y()>self.rowViewportPosition(row)+self.rowHeight(row)/2 if row>=0 else False
        if target and self.page.ui.move(source,target,after):
            # Queue rebuilding until Qt has finished delivering the drag event.
            QtCore.QTimer.singleShot(0,self.page.update_data);event.acceptProposedAction()
        else:event.ignore()

    def keyPressEvent(self,event):
        entry=self.entries.get(self.currentRow())
        if entry and entry[0]=='group' and event.key() in (QtCore.Qt.Key_Left,QtCore.Qt.Key_Right,QtCore.Qt.Key_Return):
            self.page.toggle_group(entry[1]);event.accept();return
        super().keyPressEvent(event)
