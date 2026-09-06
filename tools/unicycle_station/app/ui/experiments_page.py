import json
import numpy as np
import pyqtgraph as pg
from PySide6 import QtCore,QtWidgets as W
from app.services.background import BackgroundJob
from app.plotting.measurement import points_with_gaps
from app.services.experiments import load_experiment
from app.core.models import write_document


class ExperimentsPage(W.QWidget):
    def __init__(self,engine):
        super().__init__();self.engine=engine;self.datasets={};self.jobs=[]
        layout=W.QVBoxLayout(self)
        bar=W.QHBoxLayout()
        self.name=W.QLineEdit();self.name.setPlaceholderText("试验名称，例如 Roll 角度环第 3 次")
        self.notes=W.QLineEdit();self.notes.setPlaceholderText("备注：场地、速度、观察结果")
        bar.addWidget(self.name,2);bar.addWidget(self.notes,3)
        self.start=W.QPushButton("开始观察记录");self.start.clicked.connect(lambda:engine.submit("trial_start",self.name.text() or "手动试验",self.notes.text()))
        self.finish=W.QPushButton("结束观察记录");self.finish.clicked.connect(lambda:engine.submit("trial_stop"))
        bar.addWidget(self.start);bar.addWidget(self.finish);layout.addLayout(bar)
        self.info=W.QLabel("试验按钮仅记录数据。车辆运行仍由车身按键操作；自动记录每段运行及停止后的 2 秒。")
        self.info.setWordWrap(True);layout.addWidget(self.info)
        toolbar=W.QToolBar()
        toolbar.addAction("载入对照 A").triggered.connect(lambda:self.load("A"))
        toolbar.addAction("载入试验 B").triggered.connect(lambda:self.load("B"))
        toolbar.addAction("导出对比摘要").triggered.connect(self.export)
        self.channel=W.QComboBox();self.channel.setMinimumWidth(170);self.channel.currentIndexChanged.connect(self.render)
        self.anchor=W.QComboBox()
        for title,key in (("试验起点","start"),("Run 开始","run"),("首次参数修改","parameter")):self.anchor.addItem(title,key)
        self.anchor.currentIndexChanged.connect(self.render)
        toolbar.addWidget(self.channel);toolbar.addWidget(self.anchor)
        self.capture=W.QCheckBox("故障自动截取");self.capture.setChecked(True);self.capture.toggled.connect(lambda checked:setattr(engine.trials,"capture_enabled",checked));toolbar.addWidget(self.capture)
        layout.addWidget(toolbar)
        self.plot=pg.PlotWidget(background="#151a20");self.plot.addLegend();self.plot.showGrid(x=True,y=True,alpha=.2)
        self.plot.setLabel("bottom","相对对齐时刻",units="s");layout.addWidget(self.plot,2)
        self.curves={"A":self.plot.plot(name="对照 A",pen=pg.mkPen("#56b6f7",width=2)),"B":self.plot.plot(name="试验 B",pen=pg.mkPen("#e5b567",width=2))}
        self.summary=W.QLabel("载入 trial.json 或 frames.jsonl，按相同条件比较；曲线显示最多保留每通道约 12000 个采样点，统计使用全部有效点。")
        self.summary.setWordWrap(True);layout.addWidget(self.summary)
        self.tabs=W.QTabWidget();layout.addWidget(self.tabs,1)
        self.diff=W.QTableWidget(0,3);self.diff.setHorizontalHeaderLabels(["参数差异","对照 A","试验 B"])
        self.diff.horizontalHeader().setSectionResizeMode(0,W.QHeaderView.Stretch);self.diff.setEditTriggers(W.QAbstractItemView.NoEditTriggers)
        self.recent=W.QListWidget();self.recent.itemDoubleClicked.connect(lambda item:self.load("B",item.text()))
        self.tabs.addTab(self.diff,"起始参数差异");self.tabs.addTab(self.recent,"本次连接试验")
        self.recent_signature=None

    def load(self,slot,path=None):
        if not path:
            path,_=W.QFileDialog.getOpenFileName(self,f"选择试验 {slot}",str(self.engine.log_directory),"试验 / 日志 (*.trial.json *.jsonl)")
        if not path:return
        progress=W.QProgressDialog("读取试验与统计…","取消",0,100,self);progress.setWindowTitle(f"载入 {slot}");progress.setMinimumDuration(0)
        job=BackgroundJob(lambda report:load_experiment(path,report));self.jobs.append(job)
        self.destroyed.connect(job.cancelled.set)
        progress.canceled.connect(job.cancelled.set);job.signals.progress.connect(progress.setValue)
        def finished(data):
            if job.cancelled.is_set():return
            progress.close();self.jobs.remove(job);self.datasets[slot]=data
            chosen=self.channel.currentData()
            with QtCore.QSignalBlocker(self.channel):
                self.channel.clear()
                names=sorted({n for dataset in self.datasets.values() for n in dataset["channels"]})
                for name in names:
                    label=data["labels"].get(name,{}).get("label",name);self.channel.addItem(label,name)
                self.channel.setCurrentIndex(max(0,self.channel.findData(chosen or "roll")))
            self.render()
        def failed(error):
            if job in self.jobs:self.jobs.remove(job)
            if job.cancelled.is_set():return
            progress.close();self.summary.setText(error)
        job.signals.finished.connect(finished);job.signals.failed.connect(failed)
        QtCore.QThreadPool.globalInstance().start(job)

    def render(self,*args):
        name=self.channel.currentData();anchor=self.anchor.currentData();lines=[]
        profiles={d["metadata"]["profile"].get("id") for d in self.datasets.values()}
        if len(profiles)>1:
            self.summary.setText("设备 Profile 不同，不能直接叠加比较")
            for curve in self.curves.values():curve.setData([],[])
            return
        for slot,curve in self.curves.items():
            data=self.datasets.get(slot)
            if not data or name not in data["channels"]:curve.setData([],[]);continue
            at=data["anchors"].get(anchor)
            if at is None:curve.setData([],[]);lines.append(f"{slot} 缺少所选对齐事件");continue
            points=points_with_gaps(data["channels"][name]);curve.setData(points[:,0]-at,points[:,1],connect="finite")
            stat=data["stats"][name]
            lines.append(f"{slot} · {data['name']} · 均值 {stat['mean']:.4g} · RMS {stat['rms']:.4g} · 峰峰值 {stat['peak_to_peak']:.4g} · 最小/最大 {stat['min']:.4g}/{stat['max']:.4g}")
            if data.get("trial"):lines.append(f"备注：{data['trial'].get('notes','')} · 固件 {data['trial'].get('firmware','未知')} · 来源 {data['trial'].get('source','未知')}")
        self.summary.setText("\n".join(lines) or "请选择对比通道")
        a=self.datasets.get("A",{}).get("parameters",{});b=self.datasets.get("B",{}).get("parameters",{})
        differences=[(n,a.get(n,{}).get("value","未知"),b.get(n,{}).get("value","未知")) for n in sorted(set(a)|set(b)) if a.get(n,{}).get("value")!=b.get(n,{}).get("value")]
        self.diff.setRowCount(len(differences))
        for row,values in enumerate(differences):
            for col,value in enumerate(values):self.diff.setItem(row,col,W.QTableWidgetItem(str(value)))

    def export(self):
        path,_=W.QFileDialog.getSaveFileName(self,"导出对比摘要","comparison.json","JSON (*.json)")
        if path:write_document(path,"experiment_comparison",{"alignment":self.anchor.currentData(),"datasets":{slot:{k:v for k,v in data.items() if k not in ("channels","labels")} for slot,data in self.datasets.items()}})

    def update_data(self):
        trials=self.engine.trials
        if not self.engine.connected:trials.poll()
        self.start.setEnabled(self.engine.connected and trials.current is None)
        self.finish.setEnabled(trials.current is not None)
        if trials.current:self.info.setText(f"观察记录中：{trials.current['name']} · RAM 参数与修改过程随日志归档；按钮不启动车辆")
        else:self.info.setText("仅记录与比较。运行片段自动归档；普通参数修改、Flash 状态和人工备注随试验保存。"+(f" · {trials.error}" if trials.error else ""))
        items=list(trials.completed)
        if items!=self.recent_signature:
            self.recent_signature=items;self.recent.clear();self.recent.addItems(reversed(items))
