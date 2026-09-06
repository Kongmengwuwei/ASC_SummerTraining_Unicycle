import uuid
import csv
import math
import time
import numpy as np
import pyqtgraph as pg
import pyqtgraph.exporters
from PySide6 import QtCore, QtGui, QtWidgets as W

from app.plotting.measurement import CursorMeasurement, points_with_gaps

PALETTE = ["#56b6f7", "#e5b567", "#b294e2", "#56c5a7", "#e58caf", "#90a7bc"]


class PlotPanel(W.QWidget):
    cursor_moved = QtCore.Signal(float)
    def __init__(self, store, title="图表", channels=None, parent=None):
        super().__init__(parent)
        self.store = store
        self.title = title
        self.channels = channels or []
        self.styles = {}
        self.curves = {}
        self.events = []
        self.event_items = {}
        self.held_at = None
        layout = W.QVBoxLayout(self)
        layout.setContentsMargins(4,4,4,4)
        bar = W.QHBoxLayout()
        self.caption = W.QLabel(title)
        bar.addWidget(self.caption)
        bar.addStretch()
        self.window = W.QComboBox()
        self.window.addItems(["5", "10", "30", "60", "120"])
        self.window.setCurrentText("10")
        bar.addWidget(W.QLabel("秒"))
        bar.addWidget(self.window)
        self.pause = W.QPushButton("暂停")
        self.pause.setCheckable(True)
        self.pause.toggled.connect(self.set_paused)
        bar.addWidget(self.pause)
        self.follow = W.QCheckBox("跟随")
        self.follow.setChecked(True)
        bar.addWidget(self.follow)
        for label, slot in [("范围", self.range_dialog), ("样式", self.style_dialog), ("CSV", self.export_csv), ("PNG", self.export_png)]:
            b = W.QPushButton(label)
            b.clicked.connect(slot)
            bar.addWidget(b)
        layout.addLayout(bar)
        self.plot = pg.PlotWidget(background="#151a20")
        self.plot.showGrid(x=True,y=True,alpha=.18)
        self.plot.addLegend(offset=(12,8))
        self.plot.setLabel("bottom", "PC 接收时间", units="s")
        self.plot.setMinimumHeight(160)
        layout.addWidget(self.plot,1)
        self.stats = W.QLabel("选择通道以显示统计")
        self.stats.setSizePolicy(W.QSizePolicy.Ignored,W.QSizePolicy.Maximum)
        self.stats.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.stats.setStyleSheet("font: 10pt 'Consolas'; color:#9bacbd")
        layout.addWidget(self.stats)
        self.vline = pg.InfiniteLine(angle=90, movable=False, pen="#627181")
        self.hline = pg.InfiniteLine(angle=0, movable=False, pen="#627181")
        self.plot.addItem(self.vline, ignoreBounds=True)
        self.plot.addItem(self.hline, ignoreBounds=True)
        self.plot.scene().sigMouseMoved.connect(self.cursor)
        self.measurement = CursorMeasurement(self)
        layout.addWidget(self.measurement)
        measure=W.QPushButton("双光标");measure.setCheckable(True);measure.toggled.connect(self.measurement.toggle);bar.addWidget(measure)
        self.set_channels(self.channels)

    def cursor(self, point):
        if self.plot.sceneBoundingRect().contains(point):
            mapped = self.plot.plotItem.vb.mapSceneToView(point)
            self.vline.setPos(mapped.x())
            self.hline.setPos(mapped.y())
            self.cursor_moved.emit(mapped.x())

    def set_paused(self, paused):
        self.held_at = self.store.clock() if paused else None

    def set_channels(self, channels):
        self.channels = list(dict.fromkeys(channels))[:32]
        for curve in self.curves.values():
            self.plot.removeItem(curve)
        self.curves = {}
        for i, name in enumerate(self.channels):
            style = self.styles.get(name, {})
            label = name
            for tag in self.store.profile.channels.values():
                for ch in tag:
                    if ch["name"] == name:
                        label = f"{ch.get('label',name)} [{ch.get('unit','')}]"
            pen = pg.mkPen(style.get("color", PALETTE[i % len(PALETTE)]), width=style.get("width",1.5),
                           style=QtCore.Qt.DashLine if style.get("dash") else QtCore.Qt.SolidLine)
            curve = self.plot.plot(name=style.get("alias", label), pen=pen)
            curve.setClipToView(True)
            curve.setDownsampling(auto=True, method="peak")
            self.curves[name] = curve

    def update_data(self):
        end = self.held_at if self.held_at is not None else self.store.clock()
        seconds = int(self.window.currentText())
        data = self.store.series(self.channels, seconds, end)
        for name, series in data.items():
            if series:
                points = points_with_gaps(series)
                self.curves[name].setData(points[:,0]-self.store.started, points[:,1], connect="finite")
            else:
                self.curves[name].setData([], [])
        if self.follow.isChecked():
            self.plot.setXRange(end-self.store.started-seconds, end-self.store.started, padding=0)
        lines = []
        for name, series in data.items():
            if not series:
                continue
            a = np.asarray(series)[:,1]
            lines.append(f"{name}: {a[-1]:.4g}  min {a.min():.4g}  max {a.max():.4g}  mean {a.mean():.4g}  P-P {np.ptp(a):.4g}  RMS {np.sqrt(np.mean(a*a)):.4g}")
        self.stats.setText("\n".join(lines[:2]) or "等待通道数据")
        self.stats.setToolTip("\n".join(lines))
        self.measurement.refresh()
        _, _, events = self.store.snapshot()
        event_labels={"TASK_ENTER":"道路状态","PARAMETER":"参数修改","PROTECTION":"保护","STOP_SENT":"停车","MODE":"模式","TRIAL_START":"试验开始"}
        visible={(event["time"],event["kind"],event["message"]):event for event in events[-30:] if end-seconds<=event["time"]<=end and event["kind"] in event_labels}
        for key in set(self.event_items)-set(visible):self.plot.removeItem(self.event_items.pop(key))
        for index,(key,event) in enumerate(visible.items()):
            if key in self.event_items:continue
            item=pg.InfiniteLine(event["time"]-self.store.started,pen=pg.mkPen("#78694e",style=QtCore.Qt.DotLine),label=event_labels[event["kind"]],labelOpts={"position":.9-(index%4)*.09,"color":"#ac9975"})
            item.setToolTip(event["message"]);self.plot.addItem(item,ignoreBounds=True);self.event_items[key]=item

    def range_dialog(self):
        text, ok = W.QInputDialog.getText(self, "Y 轴范围", "输入 min,max；留空自动缩放")
        if not ok:
            return
        try:
            if not text.strip():
                self.plot.enableAutoRange(axis="y")
            else:
                low, high = map(float,text.split(","))
                if not math.isfinite(low+high) or low >= high:
                    raise ValueError()
                self.plot.setYRange(low,high,padding=0)
        except ValueError:
            W.QMessageBox.information(self,"范围","请输入有效的 min,max")

    def style_dialog(self):
        if not self.channels:
            return
        name, ok = W.QInputDialog.getItem(self,"曲线样式","通道",self.channels,editable=False)
        if not ok:
            return
        color = W.QColorDialog.getColor(parent=self)
        if not color.isValid():
            return
        width, ok = W.QInputDialog.getDouble(self,"线宽","像素",1.5,.5,6,1)
        if not ok:
            return
        alias, ok = W.QInputDialog.getText(self,"通道别名","别名（空白使用固件名）")
        dash = W.QMessageBox.question(self,"线型","使用虚线？") == W.QMessageBox.Yes
        self.styles[name] = {"color":color.name(),"width":width,"alias":alias or name,"dash":dash}
        self.set_channels(self.channels)

    def export_csv(self):
        path,_ = W.QFileDialog.getSaveFileName(self,"导出当前窗口","waveform.csv","CSV (*.csv)")
        if path:
            with open(path,"w",newline="",encoding="utf-8-sig") as f:
                writer=csv.writer(f)
                writer.writerow(["pc_monotonic_s","channel","value"])
                for name,series in self.store.series(self.channels,int(self.window.currentText()),self.held_at).items():
                    writer.writerows((t,name,v) for t,v in series)

    def export_png(self):
        path,_ = W.QFileDialog.getSaveFileName(self,"导出图表","waveform.png","PNG (*.png)")
        if path:
            pg.exporters.ImageExporter(self.plot.plotItem).export(path)

    def save_state(self):
        return dict(title=self.title,channels=self.channels,seconds=int(self.window.currentText()),styles=self.styles,
                    ranges=self.plot.viewRange(), follow=self.follow.isChecked())

    def restore_state(self,state):
        self.styles=state.get("styles",{})
        self.window.setCurrentText(str(state.get("seconds",10)))
        self.follow.setChecked(state.get("follow",True))
        self.set_channels(state.get("channels",[]))
        if "ranges" in state:
            self.plot.setRange(xRange=state["ranges"][0],yRange=state["ranges"][1])


class ScopePage(W.QWidget):
    def __init__(self,store):
        super().__init__()
        self.store=store
        self.panels=[]
        outer=W.QVBoxLayout(self)
        bar=W.QHBoxLayout()
        self.search=W.QLineEdit()
        self.search.setPlaceholderText("搜索通道 / 中文名")
        self.search.textChanged.connect(self.filter_channels)
        bar.addWidget(self.search)
        self.target=W.QComboBox()
        bar.addWidget(self.target)
        for text,slot in [("添加图表",lambda:self.add_plot()),("删除图表",self.remove_plot),("选择应用到图表",self.assign)]:
            b=W.QPushButton(text);b.clicked.connect(slot);bar.addWidget(b)
        self.linked=W.QCheckBox("多图联动");self.linked.setChecked(True);self.linked.toggled.connect(self.link_panels);bar.addWidget(self.linked)
        self.presets=W.QComboBox()
        self.presets.addItems(list(store.profile.data.get("plot_presets",{})))
        bar.addWidget(self.presets)
        b=W.QPushButton("添加预设");b.clicked.connect(self.add_preset);bar.addWidget(b)
        outer.addLayout(bar)
        split=W.QSplitter()
        self.tree=W.QTreeWidget();self.tree.setHeaderLabels(["通道", "单位"])
        self.tree.setMaximumWidth(310)
        self.tree.setMinimumWidth(230)
        self.tree.header().setSectionResizeMode(0,W.QHeaderView.Stretch)
        self.tree.header().setSectionResizeMode(1,W.QHeaderView.ResizeToContents)
        self.tree.itemDoubleClicked.connect(lambda *args:self.assign())
        split.addWidget(self.tree)
        self.host=W.QMainWindow();self.host.setDockNestingEnabled(True)
        split.addWidget(self.host);split.setStretchFactor(1,1)
        outer.addWidget(split)
        self.known=set()
        for title,channels in list(store.profile.data.get("plot_presets",{}).items())[:2]:
            self.add_plot(title,store.profile.data.get("default_plot_channels",{}).get(title,channels))

    def add_plot(self,title=None,channels=None):
        if len(self.panels)>=8:
            return
        title=title or f"图表 {len(self.panels)+1}"
        panel=PlotPanel(self.store,title,channels)
        dock=W.QDockWidget(title,self.host);dock.setObjectName("plot_"+uuid.uuid4().hex)
        dock.setWidget(panel);self.host.addDockWidget(QtCore.Qt.BottomDockWidgetArea,dock)
        if self.panels:self.host.splitDockWidget(self.panels[-1][0],dock,QtCore.Qt.Vertical)
        self.panels.append((dock,panel));self.target.addItem(title)
        panel.cursor_moved.connect(lambda x,p=panel:self.sync_cursor(p,x))
        panel.window.currentTextChanged.connect(lambda *args,p=panel:self.sync_controls(p))
        panel.pause.toggled.connect(lambda *args,p=panel:self.sync_controls(p))
        panel.follow.toggled.connect(lambda *args,p=panel:self.sync_controls(p))
        self.link_panels()

    def remove_plot(self):
        i=self.target.currentIndex()
        if i>=0:
            dock,panel=self.panels.pop(i);self.host.removeDockWidget(dock);dock.deleteLater();self.target.removeItem(i);self.link_panels()

    def assign(self):
        i=self.target.currentIndex()
        if i<0:return
        names=[]
        for j in range(self.tree.topLevelItemCount()):
            item=self.tree.topLevelItem(j)
            if item.checkState(0)==QtCore.Qt.Checked:names.append(item.data(0,QtCore.Qt.UserRole))
        self.panels[i][1].set_channels(names)

    def add_preset(self):
        name=self.presets.currentText()
        self.add_plot(name,self.store.profile.data.get("plot_presets",{}).get(name,[]))

    def filter_channels(self):
        query=self.search.text().lower()
        for i in range(self.tree.topLevelItemCount()):
            item=self.tree.topLevelItem(i);item.setHidden(query not in item.text(0).lower())

    def update_data(self):
        latest,_,_=self.store.snapshot()
        definitions={ch["name"]:ch for channels in self.store.profile.channels.values() for ch in channels}
        for name in sorted(set(latest)|set(definitions)):
            if name in self.known:continue
            self.known.add(name);d=definitions.get(name,{})
            item=W.QTreeWidgetItem([f"{d.get('label',name)} · {name}",d.get("unit","")])
            item.setData(0,QtCore.Qt.UserRole,name);item.setCheckState(0,QtCore.Qt.Unchecked);self.tree.addTopLevelItem(item)
        self.filter_channels()
        for dock,panel in self.panels:
            if dock.isVisible():panel.update_data()

    def sync_cursor(self,source,x):
        if self.linked.isChecked():
            for _,panel in self.panels:
                if panel is not source:panel.vline.setPos(x)

    def sync_controls(self,source):
        if not self.linked.isChecked():return
        for _,panel in self.panels:
            if panel is source:continue
            for target,origin in ((panel.window,source.window),(panel.pause,source.pause),(panel.follow,source.follow)):
                with QtCore.QSignalBlocker(target):
                    if isinstance(target,W.QComboBox):target.setCurrentText(origin.currentText())
                    else:target.setChecked(origin.isChecked())
            panel.held_at=source.held_at

    def link_panels(self,*args):
        if not self.panels:return
        first=self.panels[0][1]
        for _,panel in self.panels:
            panel.plot.setXLink(first.plot if self.linked.isChecked() and panel is not first else None)
        self.sync_controls(first)

    def layout_state(self):
        return {"docks":bytes(self.host.saveState().toBase64()).decode(),"linked":self.linked.isChecked()}

    def restore_layout(self,state):
        self.linked.setChecked(state.get("linked",True))
        if state.get("docks"):self.host.restoreState(QtCore.QByteArray.fromBase64(state["docks"].encode()))
        self.link_panels()

    def save_state(self):return [{**p.save_state(),"dock_name":d.objectName()} for d,p in self.panels]

    def restore_state(self,states):
        linked=self.linked.isChecked()
        with QtCore.QSignalBlocker(self.linked):self.linked.setChecked(False)
        while self.panels:self.target.setCurrentIndex(0);self.remove_plot()
        for state in states[:8]:
            self.add_plot(state.get("title","图表"),state.get("channels",[]));self.panels[-1][1].restore_state(state)
            if state.get("dock_name"):self.panels[-1][0].setObjectName(state["dock_name"])
        with QtCore.QSignalBlocker(self.linked):self.linked.setChecked(linked)
        self.link_panels()
