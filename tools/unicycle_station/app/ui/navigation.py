"""A small animated selection accent; no effects on OpenGL or live plots."""
from PySide6 import QtCore,QtGui,QtWidgets as W


class Navigation(W.QListWidget):
    def __init__(self):
        super().__init__()
        self.motion_enabled=True
        self.accent_y=0.
        self.animation=QtCore.QVariantAnimation(self)
        self.animation.setDuration(160)
        self.animation.setEasingCurve(QtCore.QEasingCurve.OutCubic)
        self.animation.valueChanged.connect(self.animate)
        self.currentRowChanged.connect(self.move_accent)
        self.verticalScrollBar().valueChanged.connect(lambda _:self.snap())

    def animate(self,value):
        self.accent_y=float(value);self.viewport().update()

    def snap(self):
        if self.currentItem():
            self.animation.stop();self.accent_y=float(self.visualItemRect(self.currentItem()).top());self.viewport().update()

    def move_accent(self,row):
        if row<0:return
        target=float(self.visualItemRect(self.item(row)).top())
        self.animation.stop()
        if not self.motion_enabled or not self.isVisible():self.animate(target);return
        self.animation.setStartValue(self.accent_y);self.animation.setEndValue(target);self.animation.start()

    def resizeEvent(self,event):
        super().resizeEvent(event);self.snap()

    def paintEvent(self,event):
        super().paintEvent(event)
        if not self.currentItem():return
        rect=self.visualItemRect(self.currentItem())
        painter=QtGui.QPainter(self.viewport());painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.setPen(QtCore.Qt.NoPen);painter.setBrush(QtGui.QColor('#65b7e8'))
        painter.drawRoundedRect(QtCore.QRectF(2,self.accent_y+9,3,max(4,rect.height()-18)),1.5,1.5)
