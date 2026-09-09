import json
from PySide6 import QtCore, QtGui, QtWidgets as W
from app.ui.edit_history import parameter_ui, group_label, display_group
from app.ui.parameter_table import ParameterTable
from app.core.models import read_document, write_document


class ParameterPage(W.QWidget):
    def __init__(self,engine,group=None):
        super().__init__()
        self.engine=engine
        self.ui=parameter_ui(engine)
        self.setProperty("parameter_page",True)
        self.group_rows={}
        self.zoom=1.0
        self.header_state=None
        self.fixed_group=group
        self.name_prefix=""
        self.favorites=engine.favorites
        self.signature=None
        self.editors={}
        self.rows={}
        layout=W.QVBoxLayout(self)
        bar=W.QHBoxLayout()
        self.search=W.QLineEdit();self.search.setPlaceholderText("搜索名称 / 固件参数名")
        bar.addWidget(self.search)
        self.group=W.QComboBox();self.group.addItem("全部","全部")
        for key in engine.profile.data.get("parameter_groups",[])+["未分类参数"]:self.group.addItem(key,key)
        if group:self.group.setCurrentText(group);self.group.setEnabled(False)
        bar.addWidget(self.group)
        self.modified=W.QCheckBox("只看修改");bar.addWidget(self.modified)
        self.starred=W.QCheckBox("只看收藏");bar.addWidget(self.starred)
        self.advanced=W.QCheckBox("解锁高级模式");bar.addWidget(self.advanced)
        layout.addLayout(bar)
        bar2=W.QToolBar()
        actions=[("读取全部",lambda:engine.submit("schema")),("读取所选",self.read_selected),
                 ("应用所选",self.apply_selected),("应用当前组",self.apply_group),("清除待修改",self.revert_pending),
                 ("回退上次值",self.rollback),("差异预览",self.preview),("导入预设",self.import_preset),("导出预设",self.export_preset)]
        for label,slot in actions:
            bar2.addAction(label).triggered.connect(slot)
        bar2.addAction(self.ui.history.createUndoAction(self,"撤销"))
        bar2.addAction(self.ui.history.createRedoAction(self,"重做"))
        bar2.addAction("显示名称").triggered.connect(self.rename_selected)
        bar2.addAction("全部展开").triggered.connect(lambda:self.expand_all(True))
        bar2.addAction("全部收起").triggered.connect(lambda:self.expand_all(False))
        layout.addWidget(bar2)
        bar3=W.QHBoxLayout()
        for label,amount,percent in [("−1 步",-1,False),("+1 步",1,False),("−10%",-.1,True),("+10%",.1,True)]:
            b=W.QPushButton(label);b.clicked.connect(lambda checked=False,a=amount,p=percent:self.nudge(a,p));bar3.addWidget(b)
        self.save_group=W.QPushButton("当前组保存 Flash");self.save_group.clicked.connect(lambda:engine.submit("save",self.group.currentData()))
        self.save_all=W.QPushButton("全部保存 Flash");self.save_all.clicked.connect(lambda:engine.submit("save","all"))
        bar3.addStretch();bar3.addWidget(self.save_group);bar3.addWidget(self.save_all);layout.addLayout(bar3)
        self.status=W.QLabel("等待 cfg Schema；旧固件仍可监看遥测")
        layout.addWidget(self.status)
        self.table=ParameterTable(self)
        self.table.setHorizontalHeaderLabels(["收藏","参数 / 固件名","MCU 当前值","待应用值","范围 / 步长","单位","RAM / Flash","权限"])
        self.table.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.table.customContextMenuRequested.connect(self.name_menu)
        rename=QtGui.QAction(self);rename.setShortcut(QtGui.QKeySequence('F2'));rename.setShortcutContext(QtCore.Qt.WidgetWithChildrenShortcut)
        rename.triggered.connect(self.rename_selected);self.table.addAction(rename)
        self.table.setSelectionBehavior(W.QAbstractItemView.SelectRows)
        self.table.horizontalHeader().setSectionResizeMode(1,W.QHeaderView.Interactive)
        self.table.setColumnWidth(0,48);self.table.setColumnWidth(2,105);self.table.setColumnWidth(3,135)
        self.table.setColumnWidth(4,150);self.table.setColumnWidth(6,150);self.table.setColumnWidth(7,200)
        self.table.horizontalHeader().setSectionsMovable(True)
        self.table.horizontalHeader().sectionMoved.connect(self.columns_moved)
        layout.addWidget(self.table)
        self.notice=W.QLabel(engine.profile.safety_rules.get("warning","RAM 写入不会自动保存 Flash。"))
        self.notice.setWordWrap(True);self.notice.setStyleSheet("color:#c7ac7e; padding:8px")
        layout.addWidget(self.notice)

    def rename_selected(self):
        entry=self.table.entries.get(self.table.currentRow())
        if not entry:return
        kind,key=entry
        label=self.ui.group_label(key) if kind=='group' else self.ui.parameter_label(self.engine.params[key])
        text,ok=W.QInputDialog.getText(self,'自定义显示名称','仅改变界面显示，固件名称保持不变；留空恢复默认。\n'+key,text=label)
        if ok:
            try:self.ui.rename(kind,key,text);self.update_data()
            except ValueError as exc:W.QMessageBox.information(self,'显示名称',str(exc))

    def name_menu(self,pos):
        row=self.table.rowAt(pos.y());entry=self.table.entries.get(row)
        if not entry:return
        self.table.setCurrentCell(row,0 if entry[0]=='group' else 1)
        menu=W.QMenu(self)
        menu.addAction('自定义显示名称 · F2').triggered.connect(self.rename_selected)
        menu.addAction('恢复默认名称').triggered.connect(lambda:self.ui.rename(*entry,''))
        menu.exec(self.table.viewport().mapToGlobal(pos))

    def selected_name(self):
        row=self.table.currentRow()
        return self.table.item(row,1).data(QtCore.Qt.UserRole) if row>=0 and self.table.item(row,1) else None

    def read_selected(self):
        name=self.selected_name()
        if name:self.engine.submit("get",name)

    def changes(self,only_selected=False):
        result=[]
        selected=self.selected_name()
        for name,p in self.engine.params.copy().items():
            if only_selected and name!=selected:continue
            if not only_selected and self.name_prefix and not name.startswith(self.name_prefix):continue
            if not only_selected and self.group.currentData() not in ("全部",p.group):continue
            if p.pending is not None and p.pending!=p.value:result.append((name,p.pending))
        return result

    def confirm_apply(self,changes):
        if not changes:return
        lines=[]
        danger=False
        for name,value in changes:
            p=self.engine.params[name]
            allowed,why=self.engine.permission(p,self.advanced.isChecked())
            if not allowed:
                W.QMessageBox.information(self,"参数锁定",f"{name}: {why}");return
            danger|=p.dangerous
            lines.append(f"{self.ui.parameter_label(p)}\n{name}: {p.value} → {value}  [{p.min}, {p.max}]")
        message="\n\n".join(lines)+"\n\n仅写入 RAM；逐项等待 MCU 实际值应答。"
        if W.QMessageBox.question(self,"危险参数修改确认" if danger else "参数差异确认",message)!=W.QMessageBox.Yes:return
        if danger and W.QMessageBox.warning(self,"二次确认","车辆必须停止。确认极性、机械零点、保护或 IPM 的新值符合本车实际情况？",W.QMessageBox.Yes|W.QMessageBox.Cancel,W.QMessageBox.Cancel)!=W.QMessageBox.Yes:return
        self.engine.submit("apply",changes,self.advanced.isChecked())

    def apply_selected(self):self.confirm_apply(self.changes(True))
    def apply_group(self):self.confirm_apply(self.changes())

    def preview(self):
        text="\n".join(f"{n}: {self.engine.params[n].value} → {v}" for n,v in self.changes()) or "没有待应用差异"
        W.QMessageBox.information(self,"参数差异",text)

    def revert_pending(self):
        self.ui.edit({n:p.value for n,p in self.engine.params.copy().items() if p.pending is not None},"清除待应用修改")
        self.table.setFocus()

    def rollback(self):
        name=self.selected_name()
        if name and self.engine.params[name].previous is not None:
            self.ui.edit({name:self.engine.params[name].previous},"回退上次值");self.table.setFocus()

    def nudge(self,amount,percent):
        name=self.selected_name()
        if not name:return
        p=self.engine.params[name]
        if p.type=="string":return
        value=p.value if p.pending is None else p.pending
        delta=max(abs(value)*abs(amount),p.step)*(1 if amount>=0 else -1) if percent else p.step*amount
        if p.type in ("int","bool","enum"):delta=round(delta)
        try:self.ui.edit({name:p.coerce(value+delta)},"参数微调");self.table.setFocus()
        except ValueError as exc:self.status.setText(str(exc))

    def import_preset(self):
        path,_=W.QFileDialog.getOpenFileName(self,"导入参数预设","","JSON (*.json)")
        if not path:return
        try:
            data=read_document(path,"parameter_preset")
            if data.get("profile_id")!=self.engine.profile.data.get("id"):raise ValueError("预设设备不匹配")
            unknown=[];changes={}
            for name,value in data["values"].items():
                if name in self.engine.params:changes[name]=self.engine.params[name].coerce(value)
                else:unknown.append(name)
            self.ui.edit(changes,"导入参数预设")
            self.table.setFocus()
            self.status.setText("已导入本地待应用值"+("；未知项: "+", ".join(unknown) if unknown else ""))
            self.preview()
        except (ValueError,KeyError,OSError) as exc:W.QMessageBox.information(self,"预设导入失败",str(exc))

    def export_preset(self):
        path,_=W.QFileDialog.getSaveFileName(self,"导出 MCU 当前值","parameters.preset.json","JSON (*.json)")
        if path:write_document(path,"parameter_preset",{"profile_id":self.engine.profile.data.get("id"),"values":{n:p.value for n,p in self.engine.params.items() if self.group.currentData() in ("全部",p.group)}})

    def pending_changed(self,name,value):
        try:
            self.ui.edit({name:value})
            editor=self.editors.get(name)
            if isinstance(editor,W.QLineEdit):
                editor.setModified(False);editor.setProperty('_parameter_native_undo',False)
        except ValueError as exc:self.status.setText(str(exc))

    def toggle_group(self,key):
        if key in self.ui.collapsed:self.ui.collapsed.remove(key)
        else:self.ui.collapsed.add(key)
        self.update_data()

    def expand_all(self,expanded):
        groups=[g for g,names in self.ui.groups() if not self.fixed_group or any(self.engine.params[n].group==self.fixed_group for n in names)]
        if expanded:self.ui.collapsed.difference_update(groups)
        else:self.ui.collapsed.update(groups)
        self.update_data()

    def columns_moved(self,logical,old,new):
        header=self.table.horizontalHeader()
        after=[header.logicalIndex(i) for i in range(header.count())]
        before=list(after);before.insert(old,before.pop(new))
        from app.ui.edit_history import change
        change(self.ui.history,"调整参数列顺序",before,after,self.restore_columns)

    def restore_columns(self,order):
        header=self.table.horizontalHeader()
        with QtCore.QSignalBlocker(header):
            for visual,logical in enumerate(order):
                if 0<=logical<header.count():header.moveSection(header.visualIndex(logical),visual)

    def save_view(self):
        header=self.table.horizontalHeader()
        return {"columns":[header.logicalIndex(i) for i in range(header.count())],"widths":[round(header.sectionSize(i)/self.zoom) for i in range(header.count())]}

    def restore_view(self,state):
        self.restore_columns(state.get("columns",list(range(8))))
        for i,width in enumerate(state.get("widths",[])[:8]):self.table.setColumnWidth(i,max(30,round(min(1500,width)*self.zoom)))

    def fit_name_column(self):
        used=sum(self.table.columnWidth(i) for i in range(8) if i!=1)
        self.table.setColumnWidth(1,max(round(220*self.zoom),self.table.viewport().width()-used))

    def set_zoom(self,factor):
        old=self.zoom;self.zoom=factor
        for row,entry in self.table.entries.items():self.table.setRowHeight(row,round((34 if entry[0]=="group" else 48)*factor))
        for col in range(self.table.columnCount()):
            if col!=1:
                self.table.setColumnWidth(col,max(24,round(self.table.columnWidth(col)*factor/old)))
        self.fit_name_column()

    def add_parameter_row(self,row,name,p):
        star=W.QCheckBox();star.setChecked(name in self.favorites)
        star.toggled.connect(lambda checked,n=name:self.ui.favorite(n,checked))
        self.table.setCellWidget(row,0,star)
        item=W.QTableWidgetItem(f"⠿  {self.ui.parameter_label(p)}\n    {name}");item.setData(QtCore.Qt.UserRole,name);item.setToolTip(p.description);item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
        self.table.setItem(row,1,item)
        for col,text in [(2,str(p.value)),(4,f"{p.min:g} … {p.max:g}\n步长 {p.step:g}"),(5,p.unit),(6,""),(7,"")]:
            item=W.QTableWidgetItem(text);item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable);self.table.setItem(row,col,item)
        value=p.value if p.pending is None else p.pending
        if p.enum_options or p.type=="bool":
            editor=W.QComboBox()
            options=p.enum_options or {"0":"否","1":"是"}
            for key,label in options.items():editor.addItem(label,key)
            editor.setCurrentIndex(max(0,editor.findData(str(value))))
            editor.currentIndexChanged.connect(lambda i,n=name,e=editor:self.pending_changed(n,e.currentData()))
        elif p.type in ("int","enum"):
            editor=W.QSpinBox();editor.setKeyboardTracking(False);editor.setRange(max(-2147483647,int(p.min)),min(2147483647,int(p.max)));editor.setSingleStep(max(1,int(p.step)));editor.setValue(int(value))
            editor.valueChanged.connect(lambda v,n=name:self.pending_changed(n,v))
        elif p.type=="string":
            editor=W.QLineEdit(str(value));editor.editingFinished.connect(lambda n=name,e=editor:self.pending_changed(n,e.text()))
        else:
            editor=W.QDoubleSpinBox();editor.setKeyboardTracking(False);editor.setDecimals(p.decimals);editor.setRange(p.min,p.max);editor.setSingleStep(p.step);editor.setValue(float(value))
            editor.valueChanged.connect(lambda v,n=name:self.pending_changed(n,v))
        editor.setProperty("_edit_revision",p.edit_revision)
        self.table.setCellWidget(row,3,editor);self.editors[name]=editor
        self.table.setRowHeight(row,round(48*self.zoom))

    def update_data(self):
        params=self.engine.params.copy()
        with QtCore.QSignalBlocker(self.group):
            for i in range(1,self.group.count()):
                key=self.group.itemData(i);self.group.setItemText(i,self.ui.names['group'].get(key,key))
        signature=(self.ui.revision,tuple((n,p.type,p.group,p.label,p.min,p.max,p.step,tuple(p.enum_options.items())) for n,p in params.items()))
        if signature!=self.signature:
            selected=self.selected_name();scroll=self.table.verticalScrollBar().value()
            self.signature=signature;self.table.clearSpans();self.table.setRowCount(0)
            self.editors.clear();self.rows.clear();self.group_rows.clear();self.table.entries.clear()
            for key,names in self.ui.groups(params):
                if self.fixed_group and not any(params[n].group==self.fixed_group for n in names):continue
                row=self.table.rowCount();self.table.insertRow(row);self.group_rows[key]=row
                self.table.entries[row]=("group",key)
                item=W.QTableWidgetItem();item.setFlags(QtCore.Qt.ItemIsEnabled|QtCore.Qt.ItemIsSelectable|QtCore.Qt.ItemIsDragEnabled|QtCore.Qt.ItemIsDropEnabled)
                self.table.setItem(row,0,item);self.table.setSpan(row,0,1,8);self.table.setRowHeight(row,round(34*self.zoom))
                for name in names:
                    row=self.table.rowCount();self.table.insertRow(row);self.rows[name]=row
                    self.table.entries[row]=("parameter",name);self.add_parameter_row(row,name,params[name])
            if selected in self.rows:self.table.selectRow(self.rows[selected])
            self.table.verticalScrollBar().setValue(scroll)
        values={n:p.value for n,p in params.items()};visible={};changed={}
        query=self.search.text().lower()
        for name,row in self.rows.items():
            p=params[name];key=display_group(p)
            allowed,why=self.engine.permission(p,self.advanced.isChecked())
            editor=self.editors[name];editor.setEnabled(allowed)
            desired=p.value if p.pending is None else p.pending
            if editor.property("_edit_revision")!=p.edit_revision or (not editor.hasFocus() and not editor.isAncestorOf(W.QApplication.focusWidget())):
                with QtCore.QSignalBlocker(editor):
                    if isinstance(editor,W.QComboBox):editor.setCurrentIndex(max(0,editor.findData(str(desired))))
                    elif isinstance(editor,W.QLineEdit):
                        if editor.text()!=str(desired):editor.setText(str(desired))
                    elif editor.value()!=desired:editor.setValue(desired)
                editor.setProperty("_edit_revision",p.edit_revision)
                if isinstance(editor,W.QLineEdit):editor.setModified(False)
            star=self.table.cellWidget(row,0)
            with QtCore.QSignalBlocker(star):star.setChecked(name in self.favorites)
            self.table.item(row,2).setText(str(p.value))
            self.table.item(row,6).setText(("RAM 已改 / " if p.ram_dirty else "RAM 同步 / ")+p.flash_state)
            self.table.item(row,7).setText(("危险 · " if p.dangerous else "")+why)
            hidden=((self.name_prefix and not name.startswith(self.name_prefix)) or self.group.currentData() not in ("全部",p.group) or query not in (name+p.label+self.ui.parameter_label(p)+self.ui.group_label(key)).lower()
                    or (self.modified.isChecked() and p.pending in (None,p.value) and not p.ram_dirty)
                    or (self.starred.isChecked() and name not in self.favorites) or not p.condition(p.visible_if,values))
            if not hidden:visible[key]=visible.get(key,0)+1
            if p.pending is not None and p.pending!=p.value:changed[key]=changed.get(key,0)+1
            self.table.setRowHidden(row,bool(hidden or (key in self.ui.collapsed and not query)))
        for key,row in self.group_rows.items():
            self.table.setRowHidden(row,not visible.get(key))
            item=self.table.item(row,0)
            text=f"{'▸' if key in self.ui.collapsed and not query else '▾'}  {self.ui.group_label(key)}   ·   {visible.get(key,0)} 项"
            if changed.get(key):text+=f"   /   {changed[key]} 项待应用"
            item.setText(text);font=item.font();font.setBold(True);item.setFont(font)
            item.setToolTip("点击展开 / 收起，拖动标题调整组序；搜索时自动显示匹配项")
            dark=getattr(self.window(),"dark",True)
            item.setBackground(QtGui.QColor("#223141" if dark else "#dfe8f1"))
            item.setForeground(QtGui.QColor("#bcd5e9" if dark else "#263f54"))
        self.save_all.setEnabled(self.engine.stopped());self.save_group.setEnabled(self.engine.stopped() and self.group.currentData() not in ("全部","未分类参数"))
        self.status.setText(f"{len(params)} 项 · "+("STOP 已确认" if self.engine.stopped() else "运行或状态未确认")+" · "+self.ui.note)
        self.status.setWordWrap(True)
