"""Read-only analysis of fixed prefixes of recorded MCU sessions; no serial writes."""
from pathlib import Path
import json, collections
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.signal import periodogram
ROOT=Path(r'C:/Users/kongmeng/Documents/EmbeddedStation/sessions')
OUT=Path(__file__).resolve().parent
NAMES=['20260911_160605_934300','20260911_170857_597700']
DATA={};manifest=[]
for name in NAMES:
 p=ROOT/name/'frames.jsonl';size=p.stat().st_size;frames=collections.defaultdict(list);changes=[]
 with p.open('rb') as f:
  while f.tell()<size:
   line=f.readline()
   try:r=json.loads(line)
   except ValueError:continue
   if r.get('kind')=='frame' and not r.get('error'):
    if r.get('tag') in ('run','att','stat','task'):frames[r['tag']].append([r['time'],*r['values']])
    if r.get('tag')=='rsp' and 'set' in r.get('values',[]):changes.append(r)
 DATA[name]={k:np.array(v,dtype=float) for k,v in frames.items()}
 manifest.append(dict(path=str(p),prefix_bytes=size,last_record_time_s=r['time'],set_responses=changes))
WINDOWS=[('baseline_early',NAMES[0],20,60,.065,.004,-4),('baseline_later',NAMES[0],300,350,.065,.004,-4),('before_P_change',NAMES[0],404,414,.065,.004,-4),('P_increased',NAMES[0],437,447,.08,.004,-4),('P_I_increased',NAMES[0],452,462,.08,.006,-4),('latest',NAMES[1],5,45,.065,.004,-3.8)]
results=[]
for label,name,a,b,kp,ki,zero in WINDOWS:
 data=DATA[name];run=data['run'];run=run[(run[:,0]>=a)&(run[:,0]<=b)];att=data['att'];att=att[(att[:,0]>=a)&(att[:,0]<=b)];stat=data['stat'];stat=stat[(stat[:,0]>=a)&(stat[:,0]<=b)]
 assert len(run)>20 and np.all(stat[:,2]==2) and np.all(stat[:,3:6]==0)
 assert np.all((run[:,25].astype(int)&3)==2) and np.all(np.abs(run[:,20])<.001)
 t=run[:,1]/1000;v=run[:,21];dt=np.diff(t);assert np.all(dt>0)
 grid=np.arange(t[0],t[-1],.02);uniform=np.interp(grid,t,v)
 freq,power=periodogram(uniform,fs=50,window='hann',detrend='linear');mask=(freq>=.2)&(freq<=4);peak=freq[mask][np.argmax(power[mask])]
 q=np.percentile(v,[5,50,95]);pq=np.percentile(att[:,2],[5,50,95])
 row=dict(label=label,session=name,pc_window_s=[a,b],speed_Kp=kp,speed_Ki=ki,pitch_zero=zero,run_samples=len(run),att_samples=len(att),speed_mean_mps=float(v.mean()),speed_rms_mps=float(np.sqrt(np.mean(v*v))),speed_q05_median_q95=q.tolist(),speed_minmax=[float(v.min()),float(v.max())],pitch_q05_median_q95=pq.tolist(),pitch_minmax=[float(att[:,2].min()),float(att[:,2].max())],speed_dominant_hz=float(peak),period_s=float(1/peak),dt_median_ms=float(np.median(dt)*1000),dt_max_ms=float(dt.max()*1000),missing_estimate=int(np.sum(np.maximum(np.rint(dt/.02)-1,0))),max_individual_fly_rpm=float(np.max(np.abs(run[:,14])+np.abs(run[:,5])/2)))
 results.append(row)
 np.savetxt(OUT/f'{label}_run.csv',run,delimiter=',',header='pc_elapsed_s,'+','.join('ch'+str(i) for i in range(25)),comments='')
 print(json.dumps(row,ensure_ascii=False))
plt.rcParams['font.sans-serif']=['Microsoft YaHei','SimHei','DejaVu Sans']
plt.rcParams['axes.unicode_minus']=False
fig,axes=plt.subplots(4,1,figsize=(12,10),sharex=True,constrained_layout=True)
data=DATA[NAMES[1]];run=data['run'];run=run[(run[:,0]>=5)&(run[:,0]<=45)];att=data['att'];att=att[(att[:,0]>=5)&(att[:,0]<=45)]
axes[0].plot(run[:,0],run[:,21],lw=1,label='C 轮实测速度');axes[0].axhline(0,c='k',ls='--',lw=.8,label='目标速度 0');axes[0].set_ylabel('速度 / m/s');axes[0].legend(loc='upper right')
axes[1].plot(att[:,0],att[:,2],lw=1,color='#df7b14',label='Pitch 估计角（约 10 Hz）');axes[1].axhline(-3.8,c='k',ls='--',lw=.8,label='机械零点 -3.8°');axes[1].set_ylabel('Pitch / °');axes[1].legend(loc='upper right')
axes[2].plot(run[:,0],run[:,2],lw=1,label='Roll 估计角');axes[2].plot(run[:,0],run[:,3],lw=.9,label='Roll 合成目标');axes[2].set_ylabel('Roll / °');axes[2].legend(loc='upper right')
axes[3].plot(run[:,0],run[:,14]+run[:,5]/2,lw=1,label='A RPM（由共模与差速重建）');axes[3].plot(run[:,0],run[:,14]-run[:,5]/2,lw=1,label='B RPM');axes[3].set_ylabel('飞轮 / RPM');axes[3].set_xlabel('会话开始后的时间 / s');axes[3].legend(loc='upper right')
for ax in axes:ax.grid(alpha=.25)
fig.suptitle('真实零速 Balance：持续往复运动\n2026-09-11 17:08:57 会话 · 取 5–45 s，排除末端保护事件',fontsize=15)
fig.savefig(OUT/'latest_balance.png',dpi=150);plt.close(fig)
(OUT/'metrics.json').write_text(json.dumps(dict(method='Fixed file prefixes; stat Balance/no Run/Test/Jog + run flags and zero ramp target. Statistics by PC receive time; frequency by MCU uptime with linear interpolation onto 20 ms grid; 0.2–4 Hz Hann periodogram, not an identified closed-loop mode.',manifest=manifest,windows=results),ensure_ascii=False,indent=2),encoding='utf-8')