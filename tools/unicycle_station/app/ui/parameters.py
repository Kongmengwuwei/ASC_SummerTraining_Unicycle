import json
from PySide6 import QtCore, QtWidgets as W
from app.core.models import read_document, write_document


class ParameterPage(W.QWidget):
    def __init__(self,engine,group=None):
        super().__init__()
        self.engine=engine
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
        self.group=W.QComboBox();self.group.addItem("全部")
        self.group.addItems(engine.profile.data.get("parameter_groups",[])+["未分类参数"])
        if group:self.group.setCurrentText(group);self.group.setEnabled(False)
        bar.addWidget(self.group)
        self.modified=W.QCheckBox("只看修改");bar.addWidget(self.modified)
        self.starred=W.QCheckBox("只看收藏");bar.addWidget(self.starred)
        self.advanced=W.QCheckBox("解锁高级模式");bar.addWidget(self.advanced)
        layout.addLayout(bar)
        bar2=W.QToolBar()
        actions=[("读取全部",lambda:engine.submit("schema")),("读取所选",self.read_selected),
                 ("应用所选",self.apply_selected),("应用当前组",self.apply_group),("撤销待修改",self.revert_pending),
                 ("回退上次值",self.rollback),("差异预览",self.preview),("导入预设",self.import_preset),("导出预设",self.export_preset)]
        for label,slot in actions:
            bar2.addAction(label).triggered.connect(slot)
        layout.addWidget(bar2)
        bar3=W.QHBoxLayout()
        for label,amount,percent in [("−1 步",-1,False),("+1 步",1,False),("−10%",-.1,True),("+10%",.1,True)]:
            b=W.QPushButton(label);b.clicked.connect(lambda checked=False,a=amount,p=percent:self.nudge(a,p));bar3.addWidget(b)
        self.save_group=W.QPushButton("当前组保存 Flash");self.save_group.clicked.connect(lambda:engine.submit("save",self.group.currentText()))
        self.save_all=W.QPushButton("全部保存 Flash");self.save_all.clicked.connect(lambda:engine.submit("save","all"))
        bar3.addStretch();bar3.addWidget(self.save_group);bar3.addWidget(self.save_all);layout.addLayout(bar3)
        self.status=W.QLabel("等待 cfg Schema；旧固件仍可监看遥测")
        layout.addWidget(self.status)
        self.table=W.QTableWidget(0,8)
        self.table.setHorizontalHeaderLabels(["收藏","参数 / 固件名","MCU 当前值","待应用值","范围 / 步长","单位","RAM / Flash","权限"])
        self.table.setSelectionBehavior(W.QAbstractItemView.SelectRows)
        self.table.horizontalHeader().setSectionResizeMode(1,W.QHeaderView.Stretch)
        self.table.setColumnWidth(0,48);self.table.setColumnWidth(2,105);self.table.setColumnWidth(3,135)
        self.table.setColumnWidth(4,150);self.table.setColumnWidth(6,150);self.table.setColumnWidth(7,200)
        layout.addWidget(self.table)
        self.notice=W.QLabel(engine.profile.safety_rules.get("warning","RAM 写入不会自动保存 Flash。"))
        self.notice.setWordWrap(True);self.notice.setStyleSheet("color:#c7ac7e; padding:8px")
        layout.addWidget(self.notice)

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
            if not only_selected and self.group.currentText() not in ("全部",p.group):continue
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
            lines.append(f"{p.label or name}\n{name}: {p.value} → {value}  [{p.min}, {p.max}]")
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
        for p in self.engine.params.values():p.pending=None
        self.table.setFocus()

    def rollback(self):
        name=self.selected_name()
        if name and self.engine.params[name].previous is not None:
            self.engine.params[name].pending=self.engine.params[name].previous;self.table.setFocus()

    def nudge(self,amount,percent):
        name=self.selected_name()
        if not name:return
        p=self.engine.params[name]
        if p.type=="string":return
        value=p.value if p.pending is None else p.pending
        delta=max(abs(value)*abs(amount),p.step)*(1 if amount>=0 else -1) if percent else p.step*amount
        if p.type in ("int","bool","enum"):delta=round(delta)
        try:p.pending=p.coerce(value+delta);self.table.setFocus()
        except ValueError as exc:self.status.setText(str(exc))

    def import_preset(self):
        path,_=W.QFileDialog.getOpenFileName(self,"导入参数预设","","JSON (*.json)")
        if not path:return
        try:
            data=read_document(path,"parameter_preset")
            if data.get("profile_id")!=self.engine.profile.data.get("id"):raise ValueError("预设设备不匹配")
            unknown=[]
            for name,value in data["values"].items():
                if name in self.engine.params:self.engine.params[name].pending=self.engine.params[name].coerce(value)
                else:unknown.append(name)
            self.signature=None
            self.status.setText("已导入本地待应用值"+("；未知项: "+", ".join(unknown) if unknown else ""))
            self.preview()
        except (ValueError,KeyError,OSError) as exc:W.QMessageBox.information(self,"预设导入失败",str(exc))

    def export_preset(self):
        path,_=W.QFileDialog.getSaveFileName(self,"导出 MCU 当前值","parameters.preset.json","JSON (*.json)")
        if path:write_document(path,"parameter_preset",{"profile_id":self.engine.profile.data.get("id"),"values":{n:p.value for n,p in self.engine.params.items() if self.group.currentText() in ("全部",p.group)}})

    def pending_changed(self,name,value):
        try:self.engine.params[name].pending=self.engine.params[name].coerce(value)
        except ValueError as exc:self.status.setText(str(exc))

    def update_data(self):
        params=self.engine.params.copy()
        signature=tuple((n,p.type,p.min,p.max,p.step,tuple(p.enum_options.items())) for n,p in params.items())
        if signature!=self.signature:
            selected=self.selected_name(); scroll=self.table.verticalScrollBar().value()
            self.signature=signature;self.table.setRowCount(0);self.editors.clear();self.rows.clear()
            for row,(name,p) in enumerate(sorted(params.items(),key=lambda x:(x[1].group,x[1].sort_order,x[0]))):
                self.table.insertRow(row);self.rows[name]=row
                star=W.QCheckBox();star.setChecked(name in self.favorites)
                star.toggled.connect(lambda checked,n=name:self.favorites.add(n) if checked else self.favorites.discard(n))
                self.table.setCellWidget(row,0,star)
                item=W.QTableWidgetItem(f"{p.label or name}\n{name}");item.setData(QtCore.Qt.UserRole,name);item.setToolTip(p.description);item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
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
                    editor=W.QSpinBox();editor.setRange(max(-2147483647,int(p.min)),min(2147483647,int(p.max)));editor.setSingleStep(max(1,int(p.step)));editor.setValue(int(value))
                    editor.valueChanged.connect(lambda v,n=name:self.pending_changed(n,v))
                elif p.type=="string":
                    editor=W.QLineEdit(str(value));editor.textEdited.connect(lambda v,n=name:self.pending_changed(n,v))
                else:
                    editor=W.QDoubleSpinBox();editor.setDecimals(p.decimals);editor.setRange(p.min,p.max);editor.setSingleStep(p.step);editor.setValue(float(value))
                    editor.valueChanged.connect(lambda v,n=name:self.pending_changed(n,v))
                self.table.setCellWidget(row,3,editor);self.editors[name]=editor
                self.table.setRowHeight(row,48)
            if selected in self.rows:self.table.selectRow(self.rows[selected])
            self.table.verticalScrollBar().setValue(scroll)
        values={n:p.value for n,p in params.items()}
        for name,row in self.rows.items():
            p=params[name]
            allowed,why=self.engine.permission(p,self.advanced.isChecked())
            editor=self.editors[name]
            editor.setEnabled(allowed)
            desired=p.value if p.pending is None else p.pending
            if not editor.hasFocus() and not editor.isAncestorOf(W.QApplication.focusWidget()):
                with QtCore.QSignalBlocker(editor):
                    if isinstance(editor,W.QComboBox):editor.setCurrentIndex(max(0,editor.findData(str(desired))))
                    elif isinstance(editor,W.QLineEdit):
                        if editor.text()!=str(desired):editor.setText(str(desired))
                    elif editor.value()!=desired:editor.setValue(desired)
            star=self.table.cellWidget(row,0)
            with QtCore.QSignalBlocker(star):star.setChecked(name in self.favorites)
            self.table.item(row,2).setText(str(p.value))
            self.table.item(row,6).setText(("RAM 已改 / " if p.ram_dirty else "RAM 同步 / ")+p.flash_state)
            self.table.item(row,7).setText(("危险 · " if p.dangerous else "")+why)
            hidden=((self.name_prefix and not name.startswith(self.name_prefix)) or self.group.currentText() not in ("全部",p.group) or self.search.text().lower() not in (name+p.label).lower()
                    or (self.modified.isChecked() and p.pending in (None,p.value) and not p.ram_dirty)
                    or (self.starred.isChecked() and name not in self.favorites) or not p.condition(p.visible_if,values))
            self.table.setRowHidden(row,hidden)
        self.save_all.setEnabled(self.engine.stopped());self.save_group.setEnabled(self.engine.stopped() and self.group.currentText() not in ("全部","未分类参数"))
        self.status.setText(f"{len(params)} 项 · 协议 {self.engine.protocol_version} · "+("STOP 已确认" if self.engine.stopped() else "运行或状态未确认")+f" · 批次 {self.engine.batch_results[-1:]}"[:180])
