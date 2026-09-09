"""Hold-to-drive controls; GUI heartbeat expires independently in device I/O."""
from PySide6 import QtCore, QtWidgets as W


class RemotePage(W.QWidget):
    KEYS = {QtCore.Qt.Key_W:'forward', QtCore.Qt.Key_Up:'forward',
            QtCore.Qt.Key_S:'back', QtCore.Qt.Key_Down:'back',
            QtCore.Qt.Key_A:'left', QtCore.Qt.Key_Left:'left',
            QtCore.Qt.Key_D:'right', QtCore.Qt.Key_Right:'right'}

    def __init__(self, engine):
        super().__init__()
        self.engine = engine
        self.keys = set(); self.mouse = set()
        layout = W.QVBoxLayout(self); layout.setSpacing(18)
        title = W.QLabel('遥控驾驶'); title.setObjectName('pageTitle'); layout.addWidget(title)
        subtitle = W.QLabel('按住前进、后退或转向，松开回零。支持 W A S D 与方向键组合。')
        subtitle.setWordWrap(True); subtitle.setObjectName('muted'); layout.addWidget(subtitle)
        self.state = W.QLabel(); self.state.setWordWrap(True); self.state.setObjectName('remoteStatus'); layout.addWidget(self.state)
        controls = W.QHBoxLayout()
        self.enable = W.QPushButton('启用本次遥控'); self.enable.setCheckable(True); self.enable.setObjectName('remoteEnable'); self.enable.toggled.connect(self.toggle); controls.addWidget(self.enable)
        controls.addStretch()
        self.demo = W.QPushButton('模拟进入 Remote'); self.demo.clicked.connect(self.enter_demo); controls.addWidget(self.demo)
        layout.addLayout(controls)
        panel = W.QFrame(); panel.setObjectName('card'); body = W.QVBoxLayout(panel); body.setContentsMargins(24,24,24,24)
        settings = W.QHBoxLayout()
        self.speed = W.QDoubleSpinBox(); self.speed.setRange(.05,.50); self.speed.setSingleStep(.05); self.speed.setValue(.15); self.speed.setSuffix(' m/s')
        self.turn = W.QSpinBox(); self.turn.setRange(5,30); self.turn.setSingleStep(5); self.turn.setValue(10); self.turn.setSuffix(' °')
        for text, widget in [('行驶速度',self.speed),('转向幅度',self.turn)]:
            settings.addWidget(W.QLabel(text)); settings.addWidget(widget)
        settings.addStretch(); body.addLayout(settings)
        pad = W.QGridLayout(); pad.setSpacing(12); pad.setColumnStretch(0,1); pad.setColumnStretch(4,1)
        self.buttons = {}
        for name,text,row,col in [('forward','↑  前进\nW',0,2),('left','←  左转\nA',1,1),('back','↓  后退\nS',2,2),('right','右转  →\nD',1,3)]:
            button=W.QPushButton(text); button.setObjectName('driveButton'); button.setMinimumSize(120,90); button.setFocusPolicy(QtCore.Qt.NoFocus)
            button.pressed.connect(lambda n=name:self.mouse.add(n)); button.released.connect(lambda n=name:self.mouse.discard(n))
            pad.addWidget(button,row,col); self.buttons[name]=button
        release=W.QPushButton('释放\nEsc'); release.setMinimumHeight(90); release.clicked.connect(lambda:self.release('遥控已释放')); pad.addWidget(release,1,2)
        body.addLayout(pad)
        self.target=W.QLabel('目标速度 0.00 m/s    转向 0°'); self.target.setObjectName('remoteTarget'); self.target.setAlignment(QtCore.Qt.AlignCenter); body.addWidget(self.target)
        self.actual=W.QLabel('等待车端命令状态'); self.actual.setAlignment(QtCore.Qt.AlignCenter); self.actual.setWordWrap(True); body.addWidget(self.actual)
        layout.addWidget(panel)
        note=W.QLabel('先在车身 Run Test → Remote 确认启动，再启用本次遥控。\n松手回零后保持平衡；顶部“立即停车”或空格键会停止电机。切换页面、窗口失焦后需重新启用。\n左右方向遵循车端 steer_dir，首次使用请在台架低速核对。')
        note.setWordWrap(True); note.setObjectName('muted'); layout.addWidget(note); layout.addStretch()
        self.timer=QtCore.QTimer(self); self.timer.setInterval(50); self.timer.timeout.connect(self.update_data); self.timer.start()
        W.QApplication.instance().installEventFilter(self)

    def enter_demo(self):
        if self.engine.mode=='mock' and self.engine.transport:
            self.release('模拟模式切换')
            self.engine.transport.enter_remote_demo()

    def toggle(self, checked):
        if checked:
            if not self.engine.remote.arm():
                self.enable.blockSignals(True); self.enable.setChecked(False); self.enable.blockSignals(False)
        else:self.release('遥控已释放')

    def release(self, reason):
        self.keys.clear(); self.mouse.clear()
        self.engine.remote.disarm(reason)
        self.enable.blockSignals(True); self.enable.setChecked(False); self.enable.blockSignals(False)
        for button in self.buttons.values():button.setDown(False)

    def hideEvent(self,event):
        self.engine.remote.wanted=False
        self.release('已离开遥控页面')
        super().hideEvent(event)

    def eventFilter(self,obj,event):
        kind=event.type()
        if not self.isVisible():return False
        if kind in (QtCore.QEvent.WindowDeactivate,QtCore.QEvent.ApplicationDeactivate):
            self.release('窗口失焦，遥控已释放')
        if kind in (QtCore.QEvent.ShortcutOverride,QtCore.QEvent.KeyPress,QtCore.QEvent.KeyRelease) and self.engine.remote.armed:
            key=event.key()
            if key==QtCore.Qt.Key_Escape:
                if kind==QtCore.QEvent.KeyPress:self.release('遥控已释放')
                event.accept();return True
            focus=W.QApplication.focusWidget()
            if isinstance(focus,(W.QLineEdit,W.QAbstractSpinBox,W.QTextEdit,W.QPlainTextEdit)):
                return False
            if key in self.KEYS and not event.modifiers() & (QtCore.Qt.ControlModifier|QtCore.Qt.AltModifier|QtCore.Qt.MetaModifier):
                if not event.isAutoRepeat():
                    if kind==QtCore.QEvent.KeyPress:self.keys.add(key)
                    elif kind==QtCore.QEvent.KeyRelease:self.keys.discard(key)
                event.accept();return True
        return False

    def update_data(self):
        remote=self.engine.remote
        visible=self.isVisible(); remote.wanted=visible
        if not visible:return
        if not self.window().isActiveWindow() and remote.armed:self.release('窗口失焦，遥控已释放')
        focus=W.QApplication.focusWidget()
        if self.keys and isinstance(focus,(W.QLineEdit,W.QAbstractSpinBox,W.QTextEdit,W.QPlainTextEdit)):
            self.release('编辑数值时已释放方向键')
        ok,why=remote.availability()
        if remote.armed and not ok:self.release(why)
        directions={self.KEYS[k] for k in self.keys}|self.mouse
        steer=(int('left' in directions)-int('right' in directions))*self.turn.value()
        speed=(int('forward' in directions)-int('back' in directions))*self.speed.value()
        if remote.armed:remote.update_intent(steer,speed)
        else:
            self.keys.clear(); self.mouse.clear(); steer=speed=0
        self.enable.setEnabled(ok)
        self.enable.blockSignals(True); self.enable.setChecked(remote.armed); self.enable.blockSignals(False)
        for name,button in self.buttons.items():
            button.setEnabled(remote.armed); button.setDown(remote.armed and name in directions)
        self.demo.setVisible(self.engine.mode=='mock')
        self.enable.setText('● 已启用 · 点击释放' if remote.armed else '启用本次遥控')
        self.state.setText(('● 遥控已启用 · '+why) if remote.armed else (why+' · 点击启用本次遥控' if ok else why))
        self.target.setText(f'目标速度 {speed:+.2f} m/s    转向 {steer:+.0f}°')
        a,v,age=remote.echo
        self.actual.setText(f'车端最近命令：{v:+.2f} m/s · {a:+.0f}° · 命令年龄 {age:.0f} ms' if remote.supported and remote.active and remote.seen else '尚未收到遥控指令 · 目标值不代表实际运动')
