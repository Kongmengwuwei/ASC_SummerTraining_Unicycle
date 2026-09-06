import numpy as np
import pyqtgraph as pg
from PySide6 import QtWidgets as W


def points_with_gaps(series):
    points=np.asarray(series,dtype=float)
    if len(points)<2:return points
    delta=np.diff(points[:,0])
    threshold=max(.2,float(np.median(delta[delta>0]))*3) if np.any(delta>0) else .2
    positions=np.flatnonzero((delta>threshold)|(delta<0))+1
    if not len(positions):return points
    return np.insert(points,positions,np.nan,axis=0)


class CursorMeasurement(W.QLabel):
    def __init__(self,panel):
        super().__init__()
        self.panel=panel;self.setWordWrap(True)
        self.a=pg.InfiniteLine(movable=True,pen=pg.mkPen("#56b6f7",width=2),label="A")
        self.b=pg.InfiniteLine(movable=True,pen=pg.mkPen("#e5b567",width=2),label="B")
        for line in (self.a,self.b):
            panel.plot.addItem(line,ignoreBounds=True);line.hide()
            line.sigPositionChanged.connect(self.refresh)
        self.hide()

    def toggle(self,enabled):
        self.setVisible(enabled)
        for line in (self.a,self.b):line.setVisible(enabled)
        if enabled:
            start,end=self.panel.plot.viewRange()[0]
            self.a.setValue(start+(end-start)*.3);self.b.setValue(start+(end-start)*.7)
            self.refresh()

    def refresh(self):
        if not self.isVisible():return
        start,end=sorted((self.a.value(),self.b.value()))
        store=self.panel.store;rows=[f"Δt {end-start:.4f} s"]
        for name in self.panel.channels:
            data=store.series([name],end-start+.5,store.started+end+.25)[name]
            if not data:continue
            pa=min(data,key=lambda p:abs(p[0]-store.started-start))
            pb=min(data,key=lambda p:abs(p[0]-store.started-end))
            if abs(pa[0]-store.started-start)>.25 or abs(pb[0]-store.started-end)>.25:continue
            rows.append(f"{name}：A {pa[1]:.4g} / B {pb[1]:.4g} / Δ {pb[1]-pa[1]:.4g}")
        self.setText("    ".join(rows[:3]));self.setToolTip("\n".join(rows))
