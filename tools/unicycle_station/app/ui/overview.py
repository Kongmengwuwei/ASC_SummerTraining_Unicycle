from PySide6 import QtCore,QtWidgets as W
from app.ui.reorder import MetricCard
from app.ui.edit_history import change,parameter_ui


class Overview(W.QWidget):
    def __init__(self,engine):
        super().__init__();self.engine=engine;self.ui=parameter_ui(engine)
        self.cards={};self.card_widgets={};self.titles={};self.units={}
        self.order=[];self.zoom=1.0
        outer=W.QVBoxLayout(self);bar=W.QHBoxLayout()
        title=W.QLabel('设备总览');title.setStyleSheet('font-size:25px;font-weight:600');bar.addWidget(title);bar.addStretch()
        button=W.QPushButton('显示项目');button.clicked.connect(self.choose_items);bar.addWidget(button);outer.addLayout(bar)
        self.state=W.QLabel('未连接 · 等待设备状态');self.state.setStyleSheet('font-size:16px;color:#9aabbc');outer.addWidget(self.state)
        self.grid=W.QGridLayout();self.grid.setSpacing(12);outer.addLayout(self.grid)
        self.empty=W.QLabel('暂无显示项目，点击右上角“显示项目”添加。');outer.addWidget(self.empty);outer.addStretch()
        self.note=W.QLabel('拖动卡片排序 · 右键隐藏 · 参数卡显示 MCU 已读回值');self.note.setWordWrap(True);outer.addWidget(self.note)
        self.restore_items(list(engine.profile.dashboards))

    def definitions(self):
        items={ch['name']:dict(ch,category='运行数据') for values in self.engine.profile.channels.values() for ch in values}
        for key in set(self.engine.profile.data.get('status_bits',{}))|set(self.engine.profile.data.get('derived_channels',{}))|set(self.engine.store.snapshot()[0]):
            items.setdefault(key,{'label':key,'unit':'','category':'运行数据'})
        for name,meta in self.engine.profile.parameters.items():
            items['param:'+name]={'label':self.ui.names['parameter'].get(name,meta.get('label',name)),'unit':meta.get('unit',''),'category':'调参读回值'}
        for name,p in self.engine.params.copy().items():
            items['param:'+name]={'label':self.ui.parameter_label(p),'unit':p.unit,'category':'调参读回值'}
        return items

    def restore_items(self,names):
        definitions=self.definitions()
        self.order=[n for n in dict.fromkeys(names) if isinstance(n,str) and (n in definitions or n.startswith('param:'))][:256]
        for name in self.order:
            if name in self.card_widgets:continue
            box=MetricCard(self,name);box.setObjectName('card');box.setCursor(QtCore.Qt.OpenHandCursor)
            layout=W.QVBoxLayout(box);title=W.QLabel();value=W.QLabel('—');unit=W.QLabel()
            title.setWordWrap(True);title.setStyleSheet('color:#a3b0bf;font-size:12px');unit.setStyleSheet('color:#899aac')
            for label in (title,value,unit):layout.addWidget(label);label.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents)
            self.card_widgets[name]=box;self.cards[name]=value;self.titles[name]=title;self.units[name]=unit
            box.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
            box.customContextMenuRequested.connect(lambda pos,n=name:self.card_menu(n,pos))
        self.reflow(self.zoom);self.update_data()

    def set_items(self,names):change(self.engine.ui_history,'修改总览显示项目',self.order,list(names),self.restore_items)

    def restore_order(self,order):
        self.restore_items([n for n in order if n in self.order]+[n for n in self.order if n not in order])

    def card_menu(self,name,pos):
        menu=W.QMenu(self);menu.addAction('隐藏此项目').triggered.connect(lambda:self.set_items([n for n in self.order if n!=name]))
        menu.addAction('选择显示项目…').triggered.connect(self.choose_items)
        menu.exec(self.card_widgets[name].mapToGlobal(pos))

    def choose_items(self):
        dialog=W.QDialog(self);dialog.setWindowTitle('选择总览显示项目');dialog.resize(640,620)
        layout=W.QVBoxLayout(dialog);search=W.QLineEdit();search.setPlaceholderText('搜索中文名称或程序名称');layout.addWidget(search)
        tree=W.QTreeWidget();tree.setHeaderLabels(['显示项目','来源']);tree.setRootIsDecorated(False);layout.addWidget(tree)
        items=self.definitions()
        for name in list(self.order)+sorted(set(items)-set(self.order)):
            d=items.get(name,{});item=W.QTreeWidgetItem([d.get('label',name)+' · '+name,d.get('category','调参读回值')])
            item.setData(0,QtCore.Qt.UserRole,name);item.setToolTip(0,item.text(0));item.setCheckState(0,QtCore.Qt.Checked if name in self.order else QtCore.Qt.Unchecked);tree.addTopLevelItem(item)
        tree.header().setSectionResizeMode(0,W.QHeaderView.Stretch)
        def filter_items(text):
            for i in range(tree.topLevelItemCount()):
                item=tree.topLevelItem(i);item.setHidden(text.lower() not in item.text(0).lower())
        search.textChanged.connect(filter_items)
        tools=W.QHBoxLayout()
        for label,selected in [('全选搜索结果',True),('取消搜索结果',False)]:
            b=W.QPushButton(label)
            def select(checked=False,selected=selected):
                for i in range(tree.topLevelItemCount()):
                    item=tree.topLevelItem(i)
                    if not item.isHidden():item.setCheckState(0,QtCore.Qt.Checked if selected else QtCore.Qt.Unchecked)
            b.clicked.connect(select);tools.addWidget(b)
        reset=W.QPushButton('默认项目');tools.addWidget(reset)
        def defaults():
            for i in range(tree.topLevelItemCount()):
                item=tree.topLevelItem(i);item.setCheckState(0,QtCore.Qt.Checked if item.data(0,QtCore.Qt.UserRole) in self.engine.profile.dashboards else QtCore.Qt.Unchecked)
        reset.clicked.connect(defaults);layout.addLayout(tools)
        buttons=W.QDialogButtonBox(W.QDialogButtonBox.Ok|W.QDialogButtonBox.Cancel);buttons.button(W.QDialogButtonBox.Ok).setText('确定');buttons.button(W.QDialogButtonBox.Cancel).setText('取消');buttons.accepted.connect(dialog.accept);buttons.rejected.connect(dialog.reject);layout.addWidget(buttons)
        if dialog.exec()==W.QDialog.Accepted:
            self.set_items([tree.topLevelItem(i).data(0,QtCore.Qt.UserRole) for i in range(tree.topLevelItemCount()) if tree.topLevelItem(i).checkState(0)==QtCore.Qt.Checked])

    def reflow(self,factor):
        self.zoom=factor
        while self.grid.count():self.grid.takeAt(0)
        for name,widget in self.card_widgets.items():widget.setVisible(name in self.order)
        columns=max(2,min(6,round(4/factor)))
        for index,name in enumerate(self.order):self.grid.addWidget(self.card_widgets[name],index//columns,index%columns)
        self.empty.setVisible(not self.order)

    def update_data(self):
        latest,stamps,_=self.engine.store.snapshot();now=self.engine.store.clock();definitions=self.definitions()
        for name in self.order:
            label=self.cards[name];d=definitions.get(name,{})
            self.titles[name].setText(d.get('label',name));self.units[name].setText(d.get('unit',''))
            self.card_widgets[name].setToolTip(name+' · 拖动排序，右键隐藏')
            stale=now-stamps.get(name,0)>.7
            text='—' if name not in latest else f'{latest[name]:.5g}'
            if name.startswith('param:'):
                p=self.engine.params.get(name[6:]);text=str(p.value) if p else '待读取';stale=p is None
            elif name in latest:
                text=self.engine.profile.data.get('value_labels',{}).get(name,{}).get(str(int(latest[name])),text)
                bits=self.engine.profile.data.get('bit_labels',{}).get(name)
                if bits:text=' / '.join(f"{title}{'✓' if int(latest[name])&(1<<i) else '—'}" for i,title in enumerate(bits))
            label.setText(text)
            label.setStyleSheet(f"font:14pt 'Consolas';color:{'#647281' if stale else ('#dce5ef' if getattr(self.window(),'dark',True) else '#202b38')}")
        if self.engine.replay:mode="离线回放 · 控制锁定"
        elif now-self.engine.status_time>.7:mode="状态未知 / STALE · 参数写入锁定"
        elif self.engine.status[4]:mode="Jog · 参数写入锁定"
        elif self.engine.status[3]:mode=f"Test {int(self.engine.status[3])} · 当前闭环 PID 可调"
        elif self.engine.status[2]:mode="Run · 危险参数锁定"
        elif self.engine.status[1]==2:mode="Balance · 三轴闭环"
        elif self.engine.stopped():mode="STOP · MCU 确认停止"
        else:mode="其他运行状态"
        self.state.setText(mode)
