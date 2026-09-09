from dataclasses import asdict
import json
from pathlib import Path
import time
from PySide6 import QtCore, QtGui, QtWidgets as W
from app.core.models import DeviceProfile, ConnectionConfig, read_document, write_document
from app.services.engine import StationEngine
from app.plotting.scope import ScopePage, PlotPanel
from app.visualization.orientation import OrientationWidget, OrientationPlugin
from app.plugins.registry import VISUALIZATIONS
VISUALIZATIONS.setdefault("orientation", OrientationPlugin)
from app.ui.parameters import ParameterPage
from app.ui.workbench import TuningPlots
from app.ui.interactions import UiInteractions
from app.ui.overview import Overview
from app.ui.edit_history import change
from app.ui.connection import ConnectionDialog
from app.ui.task_page import TaskPage
from app.ui.experiments_page import ExperimentsPage
from app.services.background import BackgroundJob
from app.recording.session import ReplayDataSource

BUILTINS=Path(__file__).resolve().parents[1]/"profiles"


def builtin_profile():return DeviceProfile.load(BUILTINS/"tc264_unicycle/profile.json")




class LogPage(W.QWidget):
    def __init__(self,engine,open_replay):
        super().__init__();self.engine=engine;self.dragging=False
        layout=W.QVBoxLayout(self);bar=W.QHBoxLayout()
        for text,slot in [("开始记录",lambda:engine.submit("record_start")),("停止记录",lambda:engine.submit("record_stop")),("打开会话",open_replay),("播放 / 暂停",self.toggle),("导出区间",self.export),("统计摘要",self.summary)]:
            b=W.QPushButton(text);b.clicked.connect(lambda checked=False,s=slot:s());bar.addWidget(b)
        self.speed=W.QComboBox();self.speed.addItems(["0.25","0.5","1","2","4"]);self.speed.setCurrentText("1");bar.addWidget(self.speed)
        self.speed.currentTextChanged.connect(self.set_speed);layout.addLayout(bar)
        self.position=W.QSlider(QtCore.Qt.Horizontal);self.position.setRange(0,10000)
        self.position.sliderPressed.connect(lambda:setattr(self,"dragging",True));self.position.sliderReleased.connect(self.seek);layout.addWidget(self.position)
        self.info=W.QLabel("原始字节 + 结构化 JSONL + 会话元数据；图表暂停不影响记录")
        layout.addWidget(self.info)
        rangebar=W.QHBoxLayout();self.start=W.QDoubleSpinBox();self.end=W.QDoubleSpinBox()
        for label,field in [("区间起点 / 秒",self.start),("终点 / 秒",self.end)]:
            field.setRange(0,1e8);rangebar.addWidget(W.QLabel(label));rangebar.addWidget(field)
        layout.addLayout(rangebar)
        self.events=W.QListWidget();self.events.itemDoubleClicked.connect(self.jump);layout.addWidget(self.events)
        self.text=W.QPlainTextEdit();self.text.setReadOnly(True);self.text.document().setMaximumBlockCount(500);layout.addWidget(self.text)
        self.last=""

    def summary(self):
        if self.engine.replay:
            dialog=W.QDialog(self);dialog.setWindowTitle("会话统计摘要");dialog.resize(720,600)
            layout=W.QVBoxLayout(dialog);text=W.QPlainTextEdit(json.dumps(self.engine.replay.summary(),ensure_ascii=False,indent=2));text.setReadOnly(True);layout.addWidget(text);dialog.exec()

    def toggle(self):
        if self.engine.replay:self.engine.replay.paused=not self.engine.replay.paused
    def set_speed(self,text):
        if self.engine.replay:self.engine.replay.speed=float(text)
    def seek(self):
        self.dragging=False
        if self.engine.replay:self.engine.replay_seek(self.position.value()/10000*self.engine.replay.duration)
    def jump(self,item):self.engine.replay_seek(item.data(QtCore.Qt.UserRole))
    def export(self):
        if not self.engine.replay:return
        path,_=W.QFileDialog.getSaveFileName(self,"导出选定区间","selection.jsonl","JSONL (*.jsonl)")
        if path:self.engine.replay.export(path,self.start.value(),self.end.value() or self.engine.replay.duration)
    def update_data(self):
        replay=self.engine.replay
        if replay:
            if not self.dragging:self.position.setValue(int(replay.position/max(.001,replay.duration)*10000))
            self.info.setText(f"{replay.position:.2f} / {replay.duration:.2f} 秒 · {replay.count} 条记录 / {len(replay.events)} 个事件")
            if self.events.count()!=len(replay.events):
                self.events.clear()
                for at,msg in replay.events:
                    item=W.QListWidgetItem(f"{at:.2f}s  {msg}");item.setData(QtCore.Qt.UserRole,at);self.events.addItem(item)
        elif self.engine.recorder:
            r=self.engine.recorder;self.info.setText(f"记录中：{r.path} · 队列丢弃 {r.dropped} · {r.error}")
        text="\n".join(self.engine.diagnostics)
        if text!=self.last:self.text.setPlainText(text);self.text.verticalScrollBar().setValue(self.text.verticalScrollBar().maximum());self.last=text


class MainWindow(W.QMainWindow):
    def __init__(self,profile=None,restore=True):
        super().__init__()
        self.settings=QtCore.QSettings("EmbeddedStation","Station")
        self.workspace_path=None;self.dark=True;self.persist_enabled=True
        self.setWindowTitle("Embedded Station · 嵌入式设备调试平台");self.resize(1520,940)
        self.history=QtGui.QUndoStack(self);self.history.setUndoLimit(100)
        self.engine=StationEngine(profile or builtin_profile());self.engine.ui_history=self.history
        self.workbenches=[]
        self.replay_jobs=[]
        self.profile_generation=0
        self.make_toolbar();self.make_menu();self.build_pages()
        self.apply_theme()
        self.interactions=UiInteractions(self)
        self.interactions.set_zoom(1)
        if restore:
            try:
                data=self.settings.value("workspace_json","")
                if data:self.restore_workspace(json.loads(data))
                geometry=self.settings.value("geometry")
                if geometry:self.restoreGeometry(geometry)
            except (ValueError,KeyError,TypeError) as exc:self.engine.event("SETTINGS",str(exc))
        self.timer=QtCore.QTimer(self);self.timer.setInterval(40);self.timer.timeout.connect(self.refresh);self.timer.start()
        self.rates=(time.monotonic(),0,0,0);self.rate_text=""
        if restore and self.engine.config.connect_last and self.engine.config.port:
            QtCore.QTimer.singleShot(0,self.connect_serial)

    def make_toolbar(self):
        self.toolbar=self.addToolBar("连接与安全停车");self.toolbar.setObjectName("connection_toolbar");self.toolbar.setMovable(False)
        self.device=W.QLabel();self.toolbar.addWidget(self.device)
        self.endpoint=W.QLabel();self.toolbar.addWidget(self.endpoint)
        for label,slot in [("串口设置",self.configure),("连接",self.connect_serial),("断开",self.disconnect),("Mock 演示",self.connect_mock),("重新握手",lambda:self.engine.submit("handshake"))]:
            action=self.toolbar.addAction(label);action.triggered.connect(slot)
        self.reconnect=W.QCheckBox("重连");self.reconnect.toggled.connect(lambda v:setattr(self.engine.config,"auto_reconnect",v));self.toolbar.addWidget(self.reconnect)
        self.toolbar.addSeparator()
        self.stop=W.QPushButton("■ 立即停车");self.stop.setObjectName("emergency");self.stop.setMinimumWidth(140);self.stop.setMinimumHeight(38)
        self.stop.clicked.connect(lambda:self.engine.emergency_stop());self.toolbar.insertWidget(self.toolbar.actions()[0],self.stop)
        self.stop.setToolTip("优先发送 stop；Space 快捷键。等待 MCU 状态确认停止。")
        action=QtGui.QAction(self);action.setShortcut(QtGui.QKeySequence("Space"));action.triggered.connect(lambda:self.engine.emergency_stop());self.addAction(action)
        self.zoom_label=W.QLabel("100%");self.statusBar().addPermanentWidget(self.zoom_label)
        self.metrics=W.QLabel();self.metrics.setSizePolicy(W.QSizePolicy.Ignored,W.QSizePolicy.Maximum);self.statusBar().addPermanentWidget(self.metrics,1)
        self.addToolBarBreak()
        self.metricsbar=self.addToolBar("测量状态");self.metricsbar.setObjectName("metrics");self.metricsbar.setMovable(False)
        self.topmetrics=W.QLabel();self.topmetrics.setWordWrap(True);self.topmetrics.setSizePolicy(W.QSizePolicy.Expanding,W.QSizePolicy.Preferred);self.metricsbar.addWidget(self.topmetrics)

    def make_menu(self):
        file=self.menuBar().addMenu("工作区")
        for label,slot in [("新建 / 恢复默认布局",self.reset_workspace),("打开 / 导入",self.open_workspace),("保存",self.save_workspace),("另存为 / 导出",lambda:self.save_workspace(True)),("加载设备 Profile",self.load_profile)]:
            file.addAction(label).triggered.connect(slot)
        edit=self.menuBar().addMenu("编辑")
        undo=self.history.createUndoAction(self,"撤销");undo.setShortcut(QtGui.QKeySequence.Undo);edit.addAction(undo)
        redo=self.history.createRedoAction(self,"重做");redo.setShortcuts([QtGui.QKeySequence("Ctrl+Shift+Z"),QtGui.QKeySequence("Ctrl+Y")]);edit.addAction(redo)
        self.view_menu=self.menuBar().addMenu("视图")
        for label,delta in (("放大界面",.1),("缩小界面",-.1),("恢复 100% · Ctrl+0",0)):
            self.view_menu.addAction(label).triggered.connect(lambda checked=False,d=delta:self.interactions.set_zoom(self.interactions.factor+d if d else 1))
        self.view_menu.addAction("深色 / 浅色").triggered.connect(self.toggle_theme)
        self.menuBar().addMenu("帮助").addAction("使用说明").triggered.connect(lambda:W.QMessageBox.information(self,"首次联调","先运行 Mock；实车固定于台架，先读取状态与参数，再验证停车。\n上位机没有远程发车、Balance、Test 或 Jog 启动入口。\n详见项目 tools/unicycle_station/README.md。"))

    def build_pages(self):
        if hasattr(self,"navdock"):
            self.removeDockWidget(self.navdock);self.navdock.deleteLater()
        self.stack=W.QStackedWidget();self.setCentralWidget(self.stack)
        self.nav=W.QListWidget();self.nav.setSpacing(4);self.nav.setMinimumWidth(135)
        self.nav.currentRowChanged.connect(self.stack.setCurrentIndex)
        self.navdock=W.QDockWidget("工作页面",self);self.navdock.setObjectName("navigation");self.navdock.setWidget(self.nav)
        self.addDockWidget(QtCore.Qt.LeftDockWidgetArea,self.navdock)
        self.view_menu.addAction(self.navdock.toggleViewAction())
        self.overview=Overview(self.engine);self.scope=ScopePage(self.engine.store,self.history)
        self.orientation=OrientationWidget(self.engine.profile,self.engine.store) if self.engine.profile.orientation_mapping else None
        if self.orientation:self.orientation.history=self.history
        self.plugin_pages=[]
        for spec in self.engine.profile.data.get("visualization_plugins", []):
            name=spec["plugin"]
            if name not in VISUALIZATIONS:
                self.engine.event("PLUGIN", f"未注册可视化插件 {name}")
                continue
            plugin=VISUALIZATIONS[name]()
            widget=plugin.create_widget(self.engine.profile,self.engine.store)
            self.plugin_pages.append((spec.get("title",name),widget,plugin))
        self.task_page=TaskPage(self.engine) if self.engine.profile.data.get("task_states") else None
        self.params=ParameterPage(self.engine)
        self.workbenches=[];bench=W.QTabWidget()
        for group,channels in self.engine.profile.data.get("workbenches",{}).items():
            widget=W.QSplitter(QtCore.Qt.Vertical);params=ParameterPage(self.engine,group);plot=TuningPlots(self.engine,group,channels,params)
            widget.addWidget(params);widget.addWidget(plot);widget.setSizes([420,370]);bench.addTab(widget,group);self.workbenches.append((params,plot))
        self.logs=LogPage(self.engine,self.open_replay)
        self.experiments=ExperimentsPage(self.engine)
        self.diagnostics=W.QPlainTextEdit();self.diagnostics.setReadOnly(True);self.diagnostics.document().setMaximumBlockCount(600)
        self.settings_page=self.make_settings()
        pages=[("总览",self.overview),("实时波形",self.scope)]
        if self.orientation:pages.append(("3D 姿态",self.orientation))
        if self.task_page:pages.append(("任务与轨迹",self.task_page))
        if self.engine.profile.data.get("supports_configuration") or self.engine.profile.data.get("parameters") or self.engine.profile.data.get("parameter_groups"):pages.append(("参数调节",self.params))
        if self.workbenches:pages.append(("调参工作台",bench))
        pages.extend((title,widget) for title,widget,plugin in self.plugin_pages)
        pages.extend([("试验与对比",self.experiments),("日志与回放",self.logs),("通信诊断",self.diagnostics),("设置",self.settings_page)])
        for label,page in pages:
            self.nav.addItem(label)
            scroll=W.QScrollArea();scroll.setWidgetResizable(True);scroll.setFrameShape(W.QFrame.NoFrame)
            scroll.setWidget(page);self.stack.addWidget(scroll)
        self.nav.setCurrentRow(0)

    def make_settings(self):
        widget=W.QWidget();layout=W.QFormLayout(widget)
        self.logdir=W.QLineEdit(str(self.engine.log_directory));self.logdir.editingFinished.connect(lambda:setattr(self.engine,"log_directory",Path(self.logdir.text())))
        layout.addRow("会话日志目录",self.logdir)
        record=W.QCheckBox("连接后自动记录（确保发车前已有原始日志）");record.setChecked(self.engine.auto_record);record.toggled.connect(lambda v:setattr(self.engine,"auto_record",v));layout.addRow(record)
        self.faults=W.QCheckBox("Mock 注入半包、噪声、时间戳回绕与随机丢帧");layout.addRow(self.faults)
        fault=W.QComboBox();fault.addItems(["timeout","reject"])
        button=W.QPushButton("Mock 下一请求模拟故障")
        button.clicked.connect(lambda:setattr(self.engine.transport,"command_fault",fault.currentText()) if self.engine.mode=="mock" and self.engine.transport else None)
        row=W.QHBoxLayout();row.addWidget(fault);row.addWidget(button);layout.addRow(row)
        derived=W.QPlainTextEdit(json.dumps(self.engine.profile.data.get("derived_channels",{}),indent=2,ensure_ascii=False))
        layout.addRow("PC 派生通道（JSON）",derived)
        b=W.QPushButton("应用派生通道")
        def apply():
            try:
                from app.core.store import derive
                import ast
                data=json.loads(derived.toPlainText())
                if not isinstance(data,dict) or len(data)>32:raise ValueError("最多 32 个派生通道")
                for name,expression in data.items():
                    names={n.id:1 for n in ast.walk(ast.parse(expression,mode="eval")) if isinstance(n,ast.Name)}
                    derive(expression,names)
                self.engine.profile.data["derived_channels"]=data
            except (ValueError,TypeError,SyntaxError,ZeroDivisionError) as exc:W.QMessageBox.information(self,"派生通道",str(exc))
        b.clicked.connect(apply);layout.addRow(b)
        note=W.QLabel("Device Profile、工作区、参数预设与日志分别保存并带 schema_version。\n未来 TCP/UDP/CAN/HID/Bluetooth 与图像流可通过插件扩展；本版单设备。")
        note.setWordWrap(True);layout.addRow(note)
        return widget

    def configure(self):
        dialog=ConnectionDialog(self.engine.config,self)
        if dialog.exec()==W.QDialog.Accepted:
            self.engine.config=dialog.config();self.reconnect.setChecked(self.engine.config.auto_reconnect)
            self.persist()

    def connect_serial(self):
        if not self.engine.config.port:self.configure()
        if self.engine.config.port:self.engine.start("serial")
    def connect_mock(self):self.engine.start("mock",faults=self.faults.isChecked())
    def disconnect(self):self.engine.close()

    def change_profile(self,profile):
        self.profile_generation+=1
        self.engine.close();self.history.clear();self.engine=StationEngine(profile);self.engine.ui_history=self.history;self.build_pages();self.apply_theme()
        if hasattr(self,"interactions"):self.interactions.set_zoom(self.interactions.factor)

    def load_profile(self):
        path,_=W.QFileDialog.getOpenFileName(self,"选择 Device Profile",str(BUILTINS),"JSON (*.json)")
        if not path:return
        try:self.change_profile(DeviceProfile.load(path))
        except (ValueError,OSError) as exc:W.QMessageBox.information(self,"Profile 无效",str(exc))

    def reset_workspace(self):self.change_profile(builtin_profile());self.workspace_path=None

    def workspace(self):
        return {"schema_version":1,"kind":"workspace","profile":self.engine.profile.data,"connection":asdict(self.engine.config),
                "plots":self.scope.save_state(),"plot_layout":self.scope.layout_state(),"plugins":{title:plugin.save_state() for title,widget,plugin in self.plugin_pages},"orientation":self.orientation.save_state() if self.orientation else {},
                "ui_scale":self.interactions.factor,"parameter_layout":self.params.ui.save(),"parameter_view":self.params.save_view(),"overview_order":self.overview.order,"overview_items":self.overview.order,
                "workbenches":[{"ring":plot.ring.currentIndex(),"tab":plot.currentIndex(),"parameter_view":params.save_view()} for params,plot in self.workbenches],"fault_capture":self.experiments.capture.isChecked(),
                "task_view":self.task_page.save_state() if self.task_page else {},"favorites":sorted(self.params.favorites),"log_directory":str(self.engine.log_directory),"page":self.nav.currentRow(),
                "layout":bytes(self.saveState().toBase64()).decode(),"dark":self.dark,"auto_record":self.engine.auto_record}

    def restore_workspace(self,data):
        if data.get("schema_version")!=1 or data.get("kind")!="workspace":raise ValueError("VERSION_MISMATCH: workspace v1 required")
        profile=data["profile"]
        if profile.get("schema_version")!=1 or profile.get("kind")!="device_profile":raise ValueError("VERSION_MISMATCH: profile")
        if profile.get("id")=="tc264_unicycle":
            profile=json.loads(json.dumps(profile))
            current=builtin_profile().data
            for key in ("task_states","value_labels","bit_labels"):profile.setdefault(key,current[key])
            for tag in ("task","taskevt"):profile.setdefault("channels",{}).setdefault(tag,current["channels"][tag])
        self.change_profile(DeviceProfile(profile))
        self.engine.config=ConnectionConfig(**data.get("connection",{}));self.engine.config.validate()
        self.scope.restore_state(data.get("plots",[]))
        self.scope.restore_layout(data.get("plot_layout",{}))
        self.params.ui.restore(data.get("parameter_layout",{}));self.params.restore_view(data.get("parameter_view",{}))
        if 'overview_items' in data:self.overview.restore_items(data['overview_items'])
        else:self.overview.restore_order(data.get("overview_order",[]))
        for state,(params,plot) in zip(data.get("workbenches",[]),self.workbenches):
            plot.ring.setCurrentIndex(state.get("ring",0));plot.setCurrentIndex(state.get("tab",0));params.restore_view(state.get("parameter_view",{}))
        self.experiments.capture.setChecked(data.get("fault_capture",True))
        self.engine.favorites.clear();self.engine.favorites.update(data.get("favorites",[]))
        for title,widget,plugin in self.plugin_pages:
            if title in data.get("plugins",{}):plugin.restore_state(data["plugins"][title])
        if self.orientation and data.get("orientation"):self.orientation.restore_state(data["orientation"])
        if self.task_page:self.task_page.restore_state(data.get("task_view",{}))
        self.engine.log_directory=Path(data.get("log_directory",str(self.engine.log_directory)))
        self.logdir.setText(str(self.engine.log_directory));self.engine.auto_record=data.get("auto_record",True)
        self.nav.setCurrentRow(min(data.get("page",0),self.nav.count()-1))
        if data.get("layout"):self.restoreState(QtCore.QByteArray.fromBase64(data["layout"].encode()))
        self.dark=data.get("dark",True);self.apply_theme()
        self.interactions.set_zoom(data.get("ui_scale",1))
        self.history.clear()

    def open_workspace(self):
        path,_=W.QFileDialog.getOpenFileName(self,"打开工作区","","JSON (*.json)")
        if not path:return
        try:self.restore_workspace(read_document(path,"workspace"));self.workspace_path=path
        except (ValueError,OSError,KeyError) as exc:W.QMessageBox.information(self,"工作区无效",str(exc))

    def save_workspace(self,save_as=False):
        path=self.workspace_path
        if save_as or not path:path,_=W.QFileDialog.getSaveFileName(self,"保存工作区","station.workspace.json","JSON (*.json)")
        if path:write_document(path,"workspace",self.workspace());self.workspace_path=path

    def open_replay(self):
        path,_=W.QFileDialog.getOpenFileName(self,"打开会话或区间日志",str(self.engine.log_directory),"JSONL (*.jsonl)")
        if not path:return
        generation=self.profile_generation
        progress=W.QProgressDialog("建立回放索引…","取消",0,100,self);progress.setMinimumDuration(0)
        job=BackgroundJob(lambda report:ReplayDataSource(path,progress=report));self.replay_jobs.append(job)
        progress.canceled.connect(job.cancelled.set);job.signals.progress.connect(progress.setValue)
        def finished(source):
            if job in self.replay_jobs:self.replay_jobs.remove(job)
            if job.cancelled.is_set():return
            progress.close()
            if generation!=self.profile_generation:return
            try:
                if "profile" in source.metadata:self.change_profile(DeviceProfile(source.metadata["profile"]))
                self.engine.open_replay(path,source=source)
            except (ValueError,OSError,KeyError) as exc:W.QMessageBox.information(self,"回放失败",str(exc))
        def failed(error):
            if job in self.replay_jobs:self.replay_jobs.remove(job)
            if job.cancelled.is_set():return
            progress.close()
            self.engine.event("REPLAY_ERROR",error)
        job.signals.finished.connect(finished);job.signals.failed.connect(failed)
        QtCore.QThreadPool.globalInstance().start(job)

    def toggle_theme(self):self.dark=not self.dark;self.apply_theme()

    def apply_theme(self):
        bg,fg,panel,border=("#10151b","#d7e0ea","#1b232d","#303d4b") if self.dark else ("#edf1f5","#202b38","#ffffff","#c7d1dc")
        self.setStyleSheet(f"""
            QWidget {{ background:{bg}; color:{fg}; font-family:'Microsoft YaHei UI'; font-size:12px; }}
            QToolBar {{ padding:6px; spacing:8px; border-bottom:1px solid {border}; }}
            QPushButton, QComboBox, QLineEdit, QSpinBox, QDoubleSpinBox {{ background:{panel}; border:1px solid {border}; border-radius:5px; padding:6px; }}
            QPushButton:hover {{ border-color:#5489b5; }}
            QPushButton:disabled {{ color:#647180; }}
            QPushButton#emergency {{ background:#b93f46; color:white; font-size:15px; font-weight:bold; border:none; padding:8px; }}
            QFrame#card {{ background:{panel}; border:1px solid {border}; border-radius:8px; }}
            QFrame#card QLabel {{ background:transparent; border:none; }}
            QListWidget::item {{ padding:12px 8px; }}
            QListWidget::item:selected {{ background:#244663; color:#dcecff; border-radius:5px; }}
            QHeaderView::section {{ background:{panel}; padding:8px; border:0; }}
            QTableWidget {{ gridline-color:{border}; }}
            QDockWidget::title {{ background:{panel}; padding:7px; }}
            QPlainTextEdit {{ font-family:Consolas; }}
        """)

        if hasattr(self,"scope"):
            for dock,panel_widget in self.scope.panels:
                panel_widget.plot.setBackground("#151a20" if self.dark else "#ffffff")
                for axis in ("left","bottom"):
                    panel_widget.plot.getAxis(axis).setTextPen("#acbac9" if self.dark else "#374959")
            if self.orientation:
                self.orientation.view.setBackgroundColor("#151a20" if self.dark else "#e2e8ee")

    def refresh(self):
        self.engine.replay_tick()
        self.device.setText(self.engine.profile.name+"  ")
        c=self.engine.config;self.endpoint.setText(f"{c.port or '未选端口'}  {c.baudrate}  ")
        now=time.monotonic();at,rx,tx,frames=self.rates
        if now-at>=1:
            elapsed=now-at;count=self.engine.store.frame_count
            self.rate_text=f"RX {(self.engine.rx-rx)/elapsed:.0f} B/s  TX {(self.engine.tx-tx)/elapsed:.0f} B/s  {max(0,count-frames)/elapsed:.0f} fps"
            self.rates=(now,self.engine.rx,self.engine.tx,count)
        tracker=self.engine.store.tracker
        loss=100*tracker.lost/max(1,tracker.received+tracker.lost)
        self.metrics.setText(f"{self.engine.link_status()}  协议 {self.engine.protocol_version}  {self.rate_text}  丢帧估计 {loss:.1f}%  解析错误 {self.engine.parser.errors}  {'● REC' if self.engine.recorder else '未记录'}  {self.engine.stop_message}")
        rtt = self.engine.requests.rtt_ms
        self.topmetrics.setToolTip(f"最近配置往返耗时 {rtt:.1f} ms" if rtt is not None else "尚无应答；绝对单向链路时延未知")
        self.topmetrics.setText(f"{self.engine.link_status()} · 协议 {self.engine.protocol_version} · {self.rate_text} · 累计 RX {self.engine.rx} B / TX {self.engine.tx} B · 遥测缺口 {loss:.1f}% · 解析错误 {self.engine.parser.errors} · {'REC' if self.engine.recorder else '未记录'}")
        self.overview.update_data()
        if self.task_page and self.task_page.isVisible():self.task_page.update_data()
        if self.scope.isVisible():self.scope.update_data()
        if self.orientation and self.orientation.isVisible():self.orientation.update_data()
        if self.params.isVisible():self.params.update_data()
        for params,plot in self.workbenches:
            if params.isVisible():params.update_data();plot.update_data()
        for title,widget,plugin in self.plugin_pages:
            if widget.isVisible() and hasattr(widget,"update_data"):widget.update_data()
        if self.logs.isVisible():self.logs.update_data()
        if self.experiments.isVisible():self.experiments.update_data()
        if self.diagnostics.isVisible():
            self.diagnostics.setPlainText("\n".join(self.engine.diagnostics)+"\n\n最近原始行\n"+"\n".join(self.engine.raw_lines))

    def persist(self):
        if not self.persist_enabled:return
        self.settings.setValue("workspace_json",json.dumps(self.workspace(),ensure_ascii=False))
        self.settings.setValue("geometry",self.saveGeometry())

    def closeEvent(self,event):
        self.interactions.dispose()
        for job in self.replay_jobs + self.experiments.jobs:job.cancelled.set()
        self.persist();self.engine.close();event.accept()
