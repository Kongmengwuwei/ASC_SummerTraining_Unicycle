"""GUI-only undo; parameter history stages values, never sends commands."""
from copy import deepcopy
from PySide6 import QtCore, QtGui
from shiboken6 import isValid


class Change(QtGui.QUndoCommand):
    def __init__(self, title, before, after, apply):
        super().__init__(title)
        self.before, self.after = deepcopy(before), deepcopy(after)
        self.apply = apply

    def restore(self, value):
        owner=getattr(self.apply, '__self__', None)
        if isinstance(owner, QtCore.QObject) and not isValid(owner):
            self.setObsolete(True)
            return
        self.apply(deepcopy(value))

    def undo(self):self.restore(self.before)
    def redo(self):self.restore(self.after)


def change(stack, title, before, after, apply):
    if before != after:stack.push(Change(title,before,after,apply))


RINGS = {'rate':'角速度环','angle':'角度环','rcy':'飞轮回收环','vel':'速度环'}
GROUPS = {'Roll':'Roll 横滚','Pitch':'Pitch 俯仰','Yaw':'Yaw 航向','Lean':'压弯','Run':'跑车',
          'Camera':'摄像头','Element':'赛道元素','Motor':'电机','Zero':'零点与保护','Odometry':'里程','IPM':'逆透视'}


def display_group(parameter):
    parts=parameter.name.split('_')
    if parameter.group in ('Roll','Pitch','Yaw') and len(parts)==3 and parts[1] in RINGS:
        return parameter.group+'/'+parts[1]
    return parameter.group


def group_label(key):
    group,_,ring=key.partition('/')
    return GROUPS.get(group,group)+(' · '+RINGS.get(ring,ring) if ring else '')


class ParameterUiState:
    def __init__(self,engine):
        self.engine=engine
        self.history=getattr(engine,'ui_history',None)
        if self.history is None:
            self.history=QtGui.QUndoStack();self.history.setUndoLimit(100)
        self.order=[];self.members={};self.collapsed=set();self.revision=0;self.names={"parameter":{},"group":{}}
        self.note='Ctrl+Z 撤销 · Ctrl+Shift+Z 重做；回退值需应用后才写入 MCU'

    def groups(self,params=None):
        params=self.engine.params.copy() if params is None else params
        groups={}
        for name,p in params.items():groups.setdefault(display_group(p),[]).append(name)
        defaults=self.engine.profile.data.get('parameter_groups',[])
        def key(g):
            base,_,ring=g.partition('/')
            return (defaults.index(base) if base in defaults else len(defaults),base,
                    list(RINGS).index(ring) if ring in RINGS else 9,g)
        ordered=[g for g in self.order if g in groups]+sorted(set(groups)-set(self.order),key=key)
        for g,names in groups.items():
            saved=self.members.get(g,[])
            groups[g]=[n for n in saved if n in names]+sorted(set(names)-set(saved),key=lambda n:(params[n].sort_order,{"kp":0,"ki":1,"kd":2}.get(n.rsplit("_",1)[-1],3),n))
        return [(g,groups[g]) for g in ordered]

    def save(self):
        return {'groups':list(self.order),'parameters':deepcopy(self.members),'collapsed':sorted(self.collapsed),'names':deepcopy(self.names)}

    def restore(self,state):
        self.order=list(dict.fromkeys(str(g) for g in state.get('groups',[])[:256]))
        self.members={str(g):list(dict.fromkeys(str(n) for n in names[:1024])) for g,names in list(state.get('parameters',{}).items())[:256] if isinstance(names,list)}
        self.names={kind:{str(k):str(v)[:80] for k,v in list(state.get('names',{}).get(kind,{}).items())[:1024] if str(v).strip()} for kind in ('parameter','group')}
        self.collapsed=set(str(g) for g in state.get('collapsed',[])[:256]);self.revision+=1

    def parameter_label(self,p):return self.names['parameter'].get(p.name,p.label or p.name)

    def group_label(self,key):
        if key in self.names['group']:return self.names['group'][key]
        base,_,ring=key.partition('/')
        if base in self.names['group']:return self.names['group'][base]+(' · '+RINGS.get(ring,ring) if ring else '')
        return group_label(key)

    def rename(self,kind,key,label):
        if kind not in self.names:raise ValueError('未知名称类型')
        label=label.strip()
        if len(label)>80 or '\n' in label or '\r' in label:raise ValueError('显示名称请使用 80 字以内的单行文字')
        before=deepcopy(self.names);after=deepcopy(before)
        if label:after[kind][key]=label
        else:after[kind].pop(key,None)
        def apply(names):self.names=deepcopy(names);self.revision+=1
        change(self.history,'修改显示名称',before,after,apply)

    def edit(self,values,title='修改待应用参数'):
        with self.engine.lock:
            after={n:self.engine.params[n].coerce(v) for n,v in values.items() if n in self.engine.params}
            before={n:(p.value if p.pending is None else p.pending) for n in after for p in [self.engine.params[n]]}
        change(self.history,title,before,after,self.apply_values)

    def apply_values(self,values):
        skipped=[]
        with self.engine.lock:
            for name,value in values.items():
                p=self.engine.params.get(name)
                if p is None:skipped.append(name);continue
                try:
                    value=p.coerce(value)
                    p.pending=None if value==p.value else value
                    p.edit_revision+=1;p.edit_target=value
                except (ValueError,TypeError):skipped.append(name)
        self.note=('部分参数已移除或类型变化，未恢复：'+', '.join(skipped)) if skipped else '已更新待应用值；MCU 当前值不变，点击应用才写入'

    def move(self,source,target,after=False):
        if source==target:return False
        groups=dict(self.groups());state=self.save();updated=deepcopy(state)
        if source[0]=='group' and target[0]=='group':
            values=list(groups);original=list(values);key=source[1];dest=target[1]
            if key not in values or dest not in values:return False
            values.remove(key);values.insert(values.index(dest)+int(after),key)
            if values==original:return False
            updated['groups']=values
        elif source[0]=='parameter' and target[0]=='parameter':
            a=self.engine.params.get(source[1]);b=self.engine.params.get(target[1])
            if not a or not b or display_group(a)!=display_group(b):
                self.note='参数可在同组内排序；跨组请拖动整个分组';return False
            g=display_group(a);values=groups[g];original=list(values);values.remove(source[1]);values.insert(values.index(target[1])+int(after),source[1])
            if values==original:return False
            updated["parameters"][g]=values
        else:return False
        change(self.history,'调整参数显示顺序',state,updated,lambda data:self.restore({**data,'collapsed':sorted(self.collapsed)}))
        return state!=updated

    def favorite(self,name,checked):
        before=sorted(self.engine.favorites);after=set(before)
        if checked:after.add(name)
        else:after.discard(name)
        def apply(values):self.engine.favorites.clear();self.engine.favorites.update(values)
        change(self.history,'修改参数收藏',before,sorted(after),apply)


def parameter_ui(engine):
    if not hasattr(engine,'parameter_ui'):engine.parameter_ui=ParameterUiState(engine)
    return engine.parameter_ui
