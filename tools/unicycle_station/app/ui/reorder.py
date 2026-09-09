from PySide6 import QtCore,QtGui,QtWidgets as W
from app.ui.edit_history import change


class MetricCard(W.QFrame):
    MIME='application/x-embedded-station-metric'
    def __init__(self,owner,name):
        super().__init__(owner);self.owner=owner;self.name=name;self.press=None;self.setAcceptDrops(True)
        self.setToolTip('按住并拖动卡片调整顺序 · Ctrl+Z 撤销排序')

    def mousePressEvent(self,event):
        if event.button()==QtCore.Qt.LeftButton:self.press=event.position().toPoint()
        super().mousePressEvent(event)

    def mouseMoveEvent(self,event):
        if self.press is not None and event.buttons()&QtCore.Qt.LeftButton and (event.position().toPoint()-self.press).manhattanLength()>=W.QApplication.startDragDistance():
            mime=QtCore.QMimeData();mime.setData(self.MIME,self.name.encode())
            drag=QtGui.QDrag(self);drag.setMimeData(mime);self.press=None;drag.exec(QtCore.Qt.MoveAction)
        else:super().mouseMoveEvent(event)

    def dragEnterEvent(self,event):
        if isinstance(event.source(),MetricCard) and event.source().owner is self.owner:event.acceptProposedAction()
        else:event.ignore()

    def dropEvent(self,event):
        if not isinstance(event.source(),MetricCard) or event.source().owner is not self.owner:event.ignore();return
        source=event.source().name
        if source!=self.name:
            before=list(self.owner.order);after=list(before);after.remove(source);after.insert(after.index(self.name),source)
            change(self.owner.engine.ui_history,'调整总览卡片顺序',before,after,self.owner.restore_order)
        event.acceptProposedAction()


class ChannelTree(W.QTreeWidget):
    orderChanged=QtCore.Signal(list,list)
    def __init__(self):
        super().__init__();self.setDragDropMode(W.QAbstractItemView.InternalMove);self.setDefaultDropAction(QtCore.Qt.MoveAction)
        self.setToolTip('拖动通道调整显示和曲线选择顺序')

    def order(self):return [self.topLevelItem(i).data(0,QtCore.Qt.UserRole) for i in range(self.topLevelItemCount())]

    def restore_order(self,order):
        rank={name:i for i,name in enumerate(order)}
        current=self.order()
        if sorted(current,key=lambda n:rank.get(n,len(rank)))==current:return
        items=[self.takeTopLevelItem(0) for _ in range(self.topLevelItemCount())]
        self.addTopLevelItems(sorted(items,key=lambda item:rank.get(item.data(0,QtCore.Qt.UserRole),len(rank))))

    def dropEvent(self,event):
        before=self.order();super().dropEvent(event);after=self.order()
        if after!=before:self.orderChanged.emit(before,after)
