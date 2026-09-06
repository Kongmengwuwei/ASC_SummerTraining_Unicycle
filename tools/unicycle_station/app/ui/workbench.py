from PySide6 import QtCore, QtWidgets as W
from app.ui.parameters import ParameterPage
from app.plotting.scope import PlotPanel


class TuningPlots(W.QTabWidget):
    """Group graphs by measurement unit; ring choice is display-only."""
    def __init__(self, engine, group, channels, params):
        super().__init__()
        self.panels=[]
        params.notice.hide();params.table.setMinimumHeight(210)
        definitions={ch['name']:ch for items in engine.profile.channels.values() for ch in items}
        units={}
        for name in channels:
            unit=definitions.get(name,{}).get('unit','')
            if name=='speed_error':unit='m/s'
            units.setdefault(unit,[]).append(name)
        for unit,names in units.items():
            panel=PlotPanel(engine.store,group+' · '+(unit or '无量纲'),names)
            self.addTab(panel,unit or '无量纲');self.panels.append(panel)
        self.ring=W.QComboBox()
        self.ring.addItem('全部环路','')
        prefix={'Roll':'r','Pitch':'p','Yaw':'y'}.get(group)
        if prefix:
            for label,name in [('角速度环','rate'),('角度环','angle')]+([('回收环','rcy')] if group=='Roll' else [('速度环','vel')] if group=='Pitch' else []):
                self.ring.addItem(label,f'{prefix}_{name}_')
        self.ring.setToolTip('只筛选参数与选择观测图，不改变车辆测试模式')
        params.layout().itemAt(0).layout().insertWidget(1,self.ring)
        def select():
            params.name_prefix=self.ring.currentData()
            ring=params.name_prefix
            preferred='°/s' if '_rate_' in ring else '°' if '_angle_' in ring else 'RPM' if '_rcy_' in ring else 'm/s'
            for index,unit in enumerate(units):
                if unit==preferred:self.setCurrentIndex(index);break
            params.update_data()
        self.ring.currentIndexChanged.connect(select)

    def update_data(self):
        panel=self.currentWidget()
        if panel:panel.update_data()
