import csv
import numpy as np
import pyqtgraph as pg
from PySide6 import QtWidgets as W


PHASES = {0:"空闲",1:"入环",2:"进环",3:"沿环",4:"环内",5:"出环"}


class TaskPage(W.QWidget):
    def __init__(self, engine):
        super().__init__()
        self.engine = engine
        layout = W.QVBoxLayout(self)
        self.title = W.QLabel("等待 Run 开始")
        self.title.setStyleSheet("font-size:25px;font-weight:600;padding:8px")
        layout.addWidget(self.title)
        self.detail = W.QLabel("仅在 Run 期间记录；结束后保留本次结果，下次 Run 自动清空")
        self.detail.setWordWrap(True); layout.addWidget(self.detail)
        split = W.QSplitter()
        history = W.QWidget(); history.setMinimumWidth(370); left = W.QVBoxLayout(history)
        left.addWidget(W.QLabel("进入 / 退出记录 · MCU 时间"))
        self.events = W.QTableWidget(0,3)
        self.events.setHorizontalHeaderLabels(["时间 / 秒","事件","阶段"])
        self.events.horizontalHeader().setSectionResizeMode(1,W.QHeaderView.Stretch)
        self.events.verticalHeader().hide()
        self.events.setColumnWidth(0,78);self.events.setColumnWidth(2,58)
        self.events.setEditTriggers(W.QAbstractItemView.NoEditTriggers)
        left.addWidget(self.events); split.addWidget(history)
        right = W.QWidget(); box = W.QVBoxLayout(right)
        bar = W.QToolBar()
        bar.addAction("轨迹归零").triggered.connect(engine.trajectory.reset)
        bar.addAction("导出轨迹").triggered.connect(self.export)
        self.yaw_reverse = W.QCheckBox("航向反向")
        self.speed_reverse = W.QCheckBox("速度反向")
        self.yaw_reverse.toggled.connect(self.mapping_changed)
        self.speed_reverse.toggled.connect(self.mapping_changed)
        bar.addWidget(self.yaw_reverse);bar.addWidget(self.speed_reverse);box.addWidget(bar)
        self.plot = pg.PlotWidget(background="#151a20")
        self.plot.setMinimumHeight(220);self.plot.setSizePolicy(W.QSizePolicy.Expanding,W.QSizePolicy.Ignored)
        self.plot.setAspectLocked(True);self.plot.showGrid(x=True,y=True,alpha=.2)
        self.plot.setLabel("bottom","相对 X",units="m");self.plot.setLabel("left","相对 Y",units="m")
        for axis in ("bottom","left"):self.plot.getAxis(axis).enableAutoSIPrefix(False)
        self.curve = self.plot.plot(pen=pg.mkPen("#56b6f7",width=2),connect="finite")
        self.position = self.plot.plot(pen=None,symbol="o",symbolSize=8,symbolBrush="#e5b567")
        box.addWidget(self.plot,1)
        self.estimate = W.QLabel();self.estimate.setWordWrap(True);box.addWidget(self.estimate)
        split.addWidget(right);split.setStretchFactor(0,2);split.setStretchFactor(1,3);split.setSizes([470,660])
        layout.addWidget(split,1)
        note=W.QLabel("相对轨迹估算：速度与航向积分，存在打滑和航向漂移；不是真实定位或赛道地图。数据缺口断开绘制，不补造路径。\n直道/弯道来自 MCU 视觉曲率诊断，独立于控制决策；十字、环岛、坡道等来自现有元素状态机。")
        note.setWordWrap(True);layout.addWidget(note)
        self.signature=None

    def mapping_changed(self):
        trajectory=self.engine.trajectory
        with trajectory.lock:
            trajectory.yaw_sign=-1 if self.yaw_reverse.isChecked() else 1
            trajectory.speed_sign=-1 if self.speed_reverse.isChecked() else 1
            trajectory.reset()

    def export(self):
        path,_=W.QFileDialog.getSaveFileName(self,"导出估算轨迹","estimated-trajectory.csv","CSV (*.csv)")
        if path:
            with self.engine.trajectory.lock:points=list(self.engine.trajectory.points)
            with open(path,"w",encoding="utf-8-sig",newline="") as stream:
                writer=csv.writer(stream);writer.writerow(["estimated_x_m","estimated_y_m","mcu_uptime_ms"]);writer.writerows(points)

    def update_data(self):
        observer=self.engine.task_observer
        with observer.lock:
            current=observer.current;stamp=observer.stamp;events=list(observer.events)
        age=self.engine.store.clock()-stamp
        session=self.engine.run_session
        if current:
            self.title.setText(f"第 {session.number} 次 Run · "+observer.label(current[1]) + (" · 已结束" if not session.active else " · 等待数据" if age>.7 else ""))
            self.title.setStyleSheet(f"font-size:25px;font-weight:600;padding:8px;color:{'#d1a96a' if age>.7 else '#70c9ae'}")
            phase_detail=(' · 环岛 '+PHASES.get(int(current[3]),str(int(current[3])))) if int(current[1]) in (5,6) else ''
            self.detail.setText(f"{'记录中' if session.active else '本次结果已冻结'}{phase_detail} · 视觉年龄 {int(current[6])} ms · 质量 {current[7]:.2f} · 状态序号 {int(current[8])} · 队列丢弃 {int(current[9])}")
        else:
            self.title.setText(f"第 {session.number} 次 Run · 等待任务数据" if session.active else "Run 已结束" if session.number else "等待 Run 开始")
            self.detail.setText("仅在 Run 期间记录任务与轨迹；结束后保留结果，下次 Run 自动清空。")
        signature=(session.number,len(events),events[-1]["seq"] if events else None)
        if signature!=self.signature:
            self.signature=signature;self.events.setRowCount(len(events))
            for row,event in enumerate(reversed(events)):
                for col,text in enumerate((f"{event['uptime']/1000:.3f}",event["message"],(PHASES.get(event["phase"],str(event["phase"])) if event.get("state") in (5,6) else "—"))):
                    self.events.setItem(row,col,W.QTableWidgetItem(text))
        with self.engine.trajectory.lock:
            trajectory=self.engine.trajectory;points=list(trajectory.points)
            self.estimate.setText(f"估算累计路程 {trajectory.distance:.2f} m · 数据缺口 {trajectory.gaps} · {trajectory.source}\n显示方向独立于 3D 模型，须用实车前推、转弯校验。")
        if points:
            a=np.asarray(points);self.curve.setData(a[:,0],a[:,1],connect="finite")
            self.position.setData([trajectory.x],[trajectory.y])
        else:self.curve.setData([],[]);self.position.setData([],[])

    def save_state(self):
        return {"yaw_reverse":self.yaw_reverse.isChecked(),"speed_reverse":self.speed_reverse.isChecked()}

    def restore_state(self,state):
        self.yaw_reverse.setChecked(state.get("yaw_reverse",False));self.speed_reverse.setChecked(state.get("speed_reverse",False))
