"""Read fixed log prefixes, verify zero-speed Balance, quantify Roll motion. No serial access."""
from pathlib import Path
import json,collections
import numpy as np
from scipy.signal import periodogram
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(r'C:/Users/kongmeng/Documents/EmbeddedStation/sessions');OUT=Path(__file__).resolve().parent
FILES=['20260911_175605_792100','20260911_180126_149700'];DATA={};manifest=[]
for name in FILES:
 path=ROOT/name/'frames.jsonl';size=path.stat().st_size;rows=[];params=[];responses=[]
 with path.open('rb') as f:
  while f.tell()<size:
   try:r=json.loads(f.readline())
   except ValueError:continue
   if r.get('kind')!='frame' or r.get('error'):continue
   v=r.get('values',[])
   if r.get('tag')=='run' and len(v)==25:rows.append([r['time'],*v])
   if r.get('tag')=='par' and len(v)>3:params.append((r['time'],v[1],v[3]))
   if r.get('tag')=='rsp' and 'set' in v:responses.append(dict(time=r['time'],raw=r['raw']))
 DATA[name]=(np.array(rows),params);manifest.append(dict(path=str(path),prefix_bytes=size,set_responses=responses))
WINDOWS=[('rate_baseline',FILES[0],10,40),('rate_P_minus20',FILES[0],65,80),('rate_D_minus10',FILES[0],133,145),('latest_early',FILES[1],15,50),('latest_late',FILES[1],660,678)]
results=[];selected={}
for label,name,a,b in WINDOWS:
 allrows,pars=DATA[name];x=allrows[(allrows[:,0]>=a)&(allrows[:,0]<=b)];assert len(x)>100
 assert np.all((x[:,25].astype(int)&3)==2) and np.max(np.abs(x[:,20]))<.001 and np.max(np.abs(x[:,8]))<.001
 t=x[:,1]*.001;assert np.all(np.diff(t)>0) and np.max(np.diff(t))<.1
 p={k:float(v) for when,k,v in pars if when<=a};p={k:v for k,v in p.items() if k.startswith(('r_','p_')) or k in ['roll_zero_init','fly_speed_limit']}
 # Check emitted target/feedback consistency to reported parameters; rounding tolerance.
 residual=np.max(np.abs(x[:,3]-(p['roll_zero_init']-x[:,6])));assert residual<.003
 rcyres=np.max(np.abs(x[:,6]-p['r_rcy_kp']*x[:,5]));assert rcyres<.003
 err=x[:,2]-x[:,3];grid=np.arange(t[0],t[-1],.02);peaks={}
 for key,col in [('roll',2),('target',3),('recovery',6)]:
  freq,power=periodogram(np.interp(grid,t,x[:,col]),fs=50,window='hann',detrend='linear');mask=(freq>=.15)&(freq<=3);peaks[key]=float(freq[mask][np.argmax(power[mask])])
 row=dict(label=label,session=name,pc_window_s=[a,b],n=len(x),parameters=p,roll_q05_median_q95=np.percentile(x[:,2],[5,50,95]).tolist(),target_q05_median_q95=np.percentile(x[:,3],[5,50,95]).tolist(),recovery_q05_median_q95=np.percentile(x[:,6],[5,50,95]).tolist(),roll_error_rms_deg=float(np.sqrt(np.mean(err**2))),roll_rate_rms_deg_s=float(np.sqrt(np.mean(x[:,4]**2))),max_abs_roll_output=float(np.max(np.abs(x[:,7]))),max_abs_individual_rpm=float(np.max(np.abs(x[:,14])+np.abs(x[:,5])/2)),yaw_clip_share=float(np.mean(np.abs(x[:,12]-x[:,13])>50)),dominant_hz=peaks,max_mcu_gap_ms=float(np.max(np.diff(t))*1000),target_formula_max_residual_deg=float(residual),recovery_formula_max_residual_deg=float(rcyres))
 results.append(row);selected[label]=x
 print(json.dumps(row,ensure_ascii=False))
 np.savetxt(OUT/f'{label}.csv',x,delimiter=',',fmt='%.7g',header='pc_elapsed_s,'+','.join(f'ch{i}' for i in range(25)),comments='')
plt.rcParams['font.sans-serif']=['Microsoft YaHei','SimHei','DejaVu Sans'];plt.rcParams['axes.unicode_minus']=False
fig,axs=plt.subplots(3,2,figsize=(13,9),constrained_layout=True)
for j,(label,title) in enumerate([('latest_early','18:01 会话：15–50 s'),('latest_late','同会话：660–678 s')]):
 x=selected[label];time=x[:,1]*.001;time-=time[0]
 axs[0,j].plot(time,x[:,2],lw=1,label='Roll 估计角');axs[0,j].plot(time,x[:,3],lw=1,label='Roll 合成目标');axs[0,j].axhline(.6,color='k',ls=':',lw=.8,label='机械零点 0.6°');axs[0,j].set_title(title);axs[0,j].set_ylabel('角度 / °')
 axs[1,j].plot(time,-x[:,6],label='−回收输出',color='#9851a7',lw=1);axs[1,j].plot(time,x[:,8],label='压弯偏移（本段为 0）',color='#29a07a',lw=1);axs[1,j].set_ylabel('目标偏移 / °')
 axs[2,j].plot(time,x[:,14]+x[:,5]/2,label='A RPM（重建）',lw=1);axs[2,j].plot(time,x[:,14]-x[:,5]/2,label='B RPM（重建）',lw=1);axs[2,j].set_ylabel('转速 / RPM');axs[2,j].set_xlabel('片段内 MCU 时间 / s')
 for ax in axs[:,j]:ax.grid(alpha=.25);ax.legend(loc='upper right',fontsize=8)
fig.suptitle('原地 Roll：车身摇摆与回收目标变化同时存在\n两段 Roll 参数相同，Pitch 参数及起始条件不同，不能当成严格对照',fontsize=14)
fig.savefig(OUT/'roll_balance.png',dpi=140);plt.close(fig)
(OUT/'metrics.json').write_text(json.dumps(dict(manifest=manifest,method='Zero-speed Balance validated by run flags; linear interpolation on MCU time; 0.15–3 Hz Hann periodogram only; formula consistency checked, correlations do not establish causation.',windows=results),ensure_ascii=False,indent=2),encoding='utf-8')