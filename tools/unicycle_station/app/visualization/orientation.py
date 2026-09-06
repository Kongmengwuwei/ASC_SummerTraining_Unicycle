import json
import time
import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets as W
import pyqtgraph.opengl as gl


def rotation_matrix(angles, mapping):
    axes=mapping.get("axes",[0,1,2]);signs=mapping.get("signs",[1,1,1])
    if sorted(axes)!=[0,1,2] or len(signs)!=3 or any(s not in (-1,1) for s in signs):
        raise ValueError("轴映射应为 0/1/2 排列，符号应为 ±1")
    order=mapping.get("order","ZYX")
    if sorted(order)!=["X","Y","Z"]:
        raise ValueError("旋转顺序必须包含 XYZ 各一次")
    a=np.asarray(angles,dtype=float)[axes]*signs
    if mapping.get("units","deg")=="deg":a=np.deg2rad(a)
    x,y,z=a
    rx=np.array([[1,0,0],[0,np.cos(x),-np.sin(x)],[0,np.sin(x),np.cos(x)]])
    ry=np.array([[np.cos(y),0,np.sin(y)],[0,1,0],[-np.sin(y),0,np.cos(y)]])
    rz=np.array([[np.cos(z),-np.sin(z),0],[np.sin(z),np.cos(z),0],[0,0,1]])
    r=np.eye(3)
    for axis in order:r=r@{"X":rx,"Y":ry,"Z":rz}[axis]
    return r


class OrientationWidget(W.QWidget):
    def __init__(self,profile,store):
        super().__init__()
        self.profile,self.store=profile,store
        self.mapping=dict(profile.orientation_mapping)
        self.zero=np.eye(3)
        self.current=np.eye(3)
        self.last_sample=None
        self.quat=QtGui.QQuaternion()
        layout=W.QVBoxLayout(self)
        bar=W.QHBoxLayout()
        for text,elev,az in [("正视",0,0),("侧视",0,90),("俯视",90,0),("透视",25,35)]:
            b=W.QPushButton(text);b.clicked.connect(lambda checked=False,e=elev,a=az:self.view.setCameraPosition(elevation=e,azimuth=a));bar.addWidget(b)
        for text,slot in [("恢复视角",self.reset_view),("显示归零",self.zero_display),("轴映射校验",self.edit_mapping)]:
            b=W.QPushButton(text);b.clicked.connect(slot);bar.addWidget(b)
        self.smooth=W.QCheckBox("显示平滑");self.smooth.setChecked(True);bar.addWidget(self.smooth)
        self.follow=W.QCheckBox("跟随");bar.addWidget(self.follow)
        layout.addLayout(bar)
        signs = W.QHBoxLayout()
        signs.addWidget(W.QLabel("仅 3D 显示方向"))
        self.axis_switches = []
        for i, name in enumerate(("Roll 反向", "Pitch 反向", "Yaw 反向")):
            box = W.QCheckBox(name)
            box.setChecked(self.mapping.get("signs", [1,1,1])[i] < 0)
            box.toggled.connect(lambda checked, axis=i: self.flip_axis(axis, checked))
            self.axis_switches.append(box); signs.addWidget(box)
        signs.addStretch(); layout.addLayout(signs)
        self.caption=W.QLabel("等待姿态 · 显示坐标映射尚未实车校验")
        self.caption.setWordWrap(True);self.caption.setSizePolicy(W.QSizePolicy.Ignored,W.QSizePolicy.Maximum);layout.addWidget(self.caption)
        self.view=gl.GLViewWidget();self.view.setBackgroundColor("#151a20")
        self.view.setMinimumHeight(250);layout.addWidget(self.view,1)
        grid=gl.GLGridItem(color=(100,120,140,70));grid.setSize(12,12);grid.setSpacing(1,1);grid.translate(0,0,-1)
        self.view.addItem(grid)
        for vector,color,label in [([4,0,0],(1,.4,.35,1),"X"),([0,4,0],(.35,.85,.6,1),"Y"),([0,0,4],(.4,.65,1,1),"Z")]:
            self.view.addItem(gl.GLLinePlotItem(pos=np.array([[0,0,0],vector]),color=color,width=2))
            self.view.addItem(gl.GLTextItem(pos=vector,text=label,color=color))
        self.faces=np.array([[0,1,2],[0,2,3],[4,6,5],[4,7,6],[0,4,5],[0,5,1],[1,5,6],[1,6,2],[2,6,7],[2,7,3],[3,7,4],[3,4,0]])
        self.colors=np.array([[.17,.23,.3,1]]*2+[[.28,.55,.73,1]]*2+[[.27,.35,.45,1]]*2+[[.78,.59,.31,1]]*2+[[.39,.34,.5,1]]*2+[[.2,.3,.37,1]]*2)
        self.mesh=gl.GLMeshItem(vertexes=self.vertices(),faces=self.faces,faceColors=self.colors,smooth=False,drawEdges=True,edgeColor=(.6,.7,.8,1))
        self.view.addItem(self.mesh)
        self.arrow=gl.GLLinePlotItem(pos=np.array([[0,0,.7],[2,0,.7],[1.5,.3,.7],[2,0,.7],[1.5,-.3,.7]]),color=(1,.75,.35,1),width=3)
        self.view.addItem(self.arrow)
        self.front=gl.GLTextItem(pos=(2,0,1),text="FRONT 车头",color=(1,.8,.5,1));self.view.addItem(self.front)
        self.note=W.QLabel("FRONT / 车头：箭头所指方向；蓝色为顶部，金色为默认 +X 端。\n世界坐标：X 红 / Y 绿 / Z 蓝；默认 Rz(Yaw)·Ry(Pitch)·Rx(Roll)。\n轴映射仅用于显示，须手动逐轴旋转车体校验。显示归零不会修改 MCU 机械零点。")
        self.note.setWordWrap(True);self.note.setSizePolicy(W.QSizePolicy.Preferred,W.QSizePolicy.Maximum);layout.addWidget(self.note)
        self.reset_view()

    def sync_switches(self):
        for i, box in enumerate(self.axis_switches):
            with QtCore.QSignalBlocker(box):
                box.setChecked(self.mapping.get("signs", [1,1,1])[i] < 0)

    def flip_axis(self, axis, reverse):
        signs = list(self.mapping.get("signs", [1,1,1]))
        signs[axis] = -1 if reverse else 1
        self.mapping["signs"] = signs
        self.mapping["verified"] = False
        self.zero = np.eye(3)
        self.quat = QtGui.QQuaternion()

    def vertices(self):
        x,y,z=np.asarray(self.mapping.get("dimensions",[3,1.3,.65]))/2
        return np.array([[-x,-y,-z],[x,-y,-z],[x,y,-z],[-x,y,-z],[-x,-y,z],[x,-y,z],[x,y,z],[-x,y,z]])

    def reset_view(self):
        self.view.setCameraPosition(distance=9,elevation=25,azimuth=35)
        self.view.opts["center"]=QtGui.QVector3D(0,0,0)

    def zero_display(self):self.zero=self.current.T.copy()

    def edit_mapping(self):
        text,ok=W.QInputDialog.getMultiLineText(self,"显示轴映射校验","逐轴转动车体，核对 FRONT、正方向与读数；通过后将 verified 设为 true。",json.dumps(self.mapping,ensure_ascii=False,indent=2))
        if not ok:return
        try:
            mapping=json.loads(text)
            rotation_matrix([0,0,0],mapping)
            if len(mapping.get("channels",[]))!=3 or mapping.get("front","+X") not in ("+X","-X","+Y","-Y"):
                raise ValueError("需要三个姿态通道及有效 FRONT 方向")
            dims=mapping.get("dimensions",[3,1.3,.65])
            if len(dims)!=3 or any(not np.isfinite(v) or v<=0 for v in dims):raise ValueError("模型尺寸非法")
            self.mapping=mapping;self.zero=np.eye(3);self.sync_switches()
        except (ValueError,TypeError) as exc:W.QMessageBox.information(self,"映射无效",str(exc))

    def update_data(self):
        latest,stamps,_=self.store.snapshot()
        names=self.mapping.get("channels",[])
        if len(names)!=3:return
        stamp=min((stamps.get(n,0) for n in names),default=0)
        age=self.store.clock()-stamp
        stale=age>.7
        angles=[latest.get(n,0) for n in names]
        verified="已校验" if self.mapping.get("verified") else "轴映射待校验"
        self.caption.setText(f"{'STALE' if stale else 'LIVE'} · Roll {angles[0]:.2f}°  Pitch {angles[1]:.2f}°  Yaw {angles[2]:.2f}° · 接收年龄 {age*1000:.0f} ms · {verified}")
        self.caption.setStyleSheet("color:#d1a96a" if stale else "color:#70c9ae")
        # On stale data stop all interpolation; never extrapolate.
        if not stale:
            self.current=rotation_matrix(angles,self.mapping)
            r=self.zero@self.current
            matrix=QtGui.QMatrix3x3(r.flatten().tolist())
            target=QtGui.QQuaternion.fromRotationMatrix(matrix)
            self.quat=QtGui.QQuaternion.slerp(self.quat,target,.35) if self.smooth.isChecked() else target
            qmatrix=self.quat.toRotationMatrix()
            r=np.array([[qmatrix[row,col] for col in range(3)] for row in range(3)])
            self.display_rotation=r
        else:r=getattr(self,"display_rotation",np.eye(3))
        colors=self.colors.copy();colors[:,:3]*=.4 if stale else 1
        self.mesh.setMeshData(vertexes=self.vertices()@r.T,faces=self.faces,faceColors=colors)
        arrow=np.array([[0,0,.7],[2,0,.7],[1.5,.3,.7],[2,0,.7],[1.5,-.3,.7]])
        front=self.mapping.get("front","+X")
        theta={"+X":0,"-X":np.pi,"+Y":np.pi/2,"-Y":-np.pi/2}[front]
        front_r=rotation_matrix([0,0,theta],{"units":"rad"})
        self.arrow.setData(pos=arrow@front_r.T@r.T)
        self.front.setData(pos=np.array([2,0,1])@front_r.T@r.T)
        if self.follow.isChecked() and not stale:self.view.setCameraPosition(azimuth=35+np.degrees(np.arctan2(r[1,0],r[0,0])))

    def save_state(self):
        return {"mapping":self.mapping,"distance":self.view.opts["distance"],"elevation":self.view.opts["elevation"],"azimuth":self.view.opts["azimuth"],"smooth":self.smooth.isChecked(),"follow":self.follow.isChecked()}

    def restore_state(self,state):
        mapping=state.get("mapping",state)
        rotation_matrix([0,0,0],mapping)
        self.mapping=mapping;self.zero=np.eye(3);self.sync_switches()
        self.view.setCameraPosition(distance=state.get("distance",9),elevation=state.get("elevation",25),azimuth=state.get("azimuth",35))
        self.smooth.setChecked(state.get("smooth",True));self.follow.setChecked(state.get("follow",False))


class OrientationPlugin:
    def create_widget(self,profile,store):
        self.widget=OrientationWidget(profile,store)
        return self.widget
    def save_state(self):return self.widget.save_state()
    def restore_state(self,state):self.widget.restore_state(state)
