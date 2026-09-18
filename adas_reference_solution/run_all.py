"""Run every experiment and save figures, CSV histories and JSON evidence."""
from pathlib import Path
import argparse
import csv
import json
import os
import platform
import sys

ROOT = Path(__file__).resolve().parent
os.environ.setdefault('MPLCONFIGDIR', str(ROOT/'output'/'.mplcache'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from adas import braking, trajectory, tracking, speed

OUT = ROOT/'output'


def csv_save(path, rows):
    rows = list(rows)
    if not rows:
        return
    with path.open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)


def series_save(name, data):
    csv_save(OUT/'data'/f'{name}.csv', (dict(zip(data, row)) for row in zip(*data.values())))


def figure_save(name):
    plt.tight_layout()
    plt.savefig(OUT/'figures'/f'{name}.png', dpi=180, bbox_inches='tight')
    plt.savefig(OUT/'figures'/f'{name}.svg', bbox_inches='tight')
    plt.close()


def run_braking():
    fig, axes = plt.subplots(2,2,figsize=(11,7))
    summaries = []
    for mu in (.15,.5,.85):
        result,data = braking.scenario(mu); summaries.append(result)
        series_save(f'task1_mu_{mu}',data)
        for ax,key,title in zip(axes.flat,['travelled_m','remaining_stop_m','pure_steer_m','combined_frozen_m'],
                               ['Gidilen mesafe','Kalan durma mesafesi','Yalnız dümenleme: anlık hız yaklaşımı',
                                'Frenleme altında dümenleme: sabit hız yaklaşımı']):
            if np.any(np.isfinite(data[key])):
                ax.plot(data['t_s'],data[key],label=f'μ = {mu}')
            ax.set(xlabel='Zaman [s]',ylabel='Mesafe [m]',title=title)
            ax.grid(alpha=.25)
    for ax in axes.flat: ax.legend()
    axes[1,1].text(.03,.8,'μ=0,15 ve 0,50: kalan yanal kapasite sıfır',transform=axes[1,1].transAxes,fontsize=9)
    figure_save('task1_braking')
    csv_save(OUT/'data'/'task1_summary.csv',summaries)
    return summaries


def run_trajectory():
    cfg=trajectory.Config(); pool=trajectory.candidates(cfg)
    csv_save(OUT/'data'/'task2_candidates.csv', [{k:v for k,v in c.items() if k!='coefficients'} for c in pool])
    sweeps=[]
    for j,name in enumerate(['k_j','k_t','k_s']):
        for weight in (.1,1.,10.,100.):
            weights=[1.,1.,1.]; weights[j]=weight
            for filtered in (False,True):
                choice=trajectory.select(pool, weights, filtered)
                if choice is None:
                    raise RuntimeError('Declared grid contains no feasible trajectory')
                sweeps.append(dict(choice,parameter=name,value=weight))
    csv_save(OUT/'data'/'task2_sweep.csv', [{k:v for k,v in c.items() if k not in ('coefficients','weights')} for c in sweeps])
    fig,axes=plt.subplots(2,3,figsize=(12,7))
    for j,name in enumerate(['k_j','k_t','k_s']):
        for filtered,label,style in [(False,'Ödev maliyeti','o--'),(True,'Fiziksel filtreli','s-')]:
            rows=[c for c in sweeps if c['parameter']==name and c['filtered']==filtered]
            for i,key in enumerate(['T_s','rms_jerk_mps3']):
                axes[i,j].plot([c['value'] for c in rows],[c[key] for c in rows],style,label=label)
                axes[i,j].set_xscale('log'); axes[i,j].set_xlabel(name); axes[i,j].grid(alpha=.25)
        axes[0,j].set_title(name+' etkisi'); axes[0,j].set_ylabel('Seçilen süre [s]')
        axes[1,j].set_ylabel('RMS jerk [m/s³]'); axes[0,j].legend(fontsize=8)
    figure_save('task2_weights')
    base=trajectory.select(pool); safe=trajectory.select(pool,filtered=True)
    fig,axes=plt.subplots(2,2,figsize=(11,7))
    for c,label in [(base,'Ödev maliyeti'),(safe,'Fiziksel filtreli')]:
        d=trajectory.sample(c,cfg); series_save('task2_'+('filtered' if c['filtered'] else 'original'),d)
        for ax,key,title,unit in zip(axes.flat,['speed_mps','accel_mps2','jerk_mps3','margin_m'],
                                   ['Hız','İvme','Jerk','Takip mesafesi marjı'],['m/s','m/s²','m/s³','m']):
            ax.plot(d['t_s'],d[key],label=f'{label}: T={c["T_s"]:.2f} s')
            ax.set(xlabel='Zaman [s]',ylabel=unit,title=title); ax.grid(alpha=.25); ax.legend(fontsize=8)
    axes[1,1].axhline(0,color='red',ls='--',lw=1)
    figure_save('task2_trajectories')
    # Grid refinement is evidence about discrete search, not a continuous optimum proof.
    refined=trajectory.candidates(cfg,time_step=.125,offset_step=.5)
    return dict(config=vars(cfg),candidate_count=len(pool),feasible_count=sum(c['feasible'] for c in pool),
                baseline=base,filtered=safe,sweeps=sweeps,
                refined_baseline=trajectory.select(refined),refined_filtered=trajectory.select(refined,filtered=True))


def run_tracking():
    cases=[('fb_baseline',dict(controller='feedback',ky=.01,kpsi=2.)),
           ('fb_high_ky',dict(controller='feedback',ky=1.,kpsi=2.)),
           ('fb_low_ky',dict(controller='feedback',ky=1e-6,kpsi=2.)),
           ('fb_high_kpsi',dict(controller='feedback',ky=.01,kpsi=5.)),
           ('fb_low_kpsi',dict(controller='feedback',ky=.01,kpsi=.5)),
           ('pp_10',dict(controller='pure_pursuit',lookahead=10.)),
           ('pp_20',dict(controller='pure_pursuit',lookahead=20.)),
           ('pp_55',dict(controller='pure_pursuit',lookahead=55.))]
    rows=[]; histories={}
    for name,kwargs in cases:
        print('  Tracking:',name,flush=True)
        row,data,path=tracking.simulate(**kwargs)
        rows.append(dict(name=name,**row)); histories[name]=data; series_save('task3_'+name,data)
    csv_save(OUT/'data'/'task3_summary.csv',rows)
    fig,axes=plt.subplots(2,2,figsize=(12,7))
    for ax,names in [(axes[0,0],['fb_baseline','pp_20']),(axes[0,1],['pp_10','pp_20','pp_55'])]:
        ax.plot(path[:,0],path[:,1],'k--',label='Referans')
        for name in names:
            d=histories[name]; ax.plot(d['x_m'],d['y_m'],label=name)
        ax.set(xlabel='X [m]',ylabel='Y [m]',xlim=(0,240)); ax.grid(alpha=.25); ax.legend(fontsize=8)
    for name in ['fb_baseline','pp_20','pp_10']:
        d=histories[name]; axes[1,0].plot(d['t_s'],d['distance_error_m'],label=name)
    axes[1,0].set(xlabel='Zaman [s]',ylabel='Yola uzaklık [m]'); axes[1,0].legend(fontsize=8); axes[1,0].grid(alpha=.25)
    axes[1,1].barh([r['name'] for r in rows],[r['rmse_m'] for r in rows])
    axes[1,1].set(xlabel='Yola uzaklık RMSE [m]',xscale='log'); axes[1,1].grid(axis='x',alpha=.25)
    figure_save('task3_tracking')
    convergence=[]
    for controller in ('feedback','pure_pursuit'):
        # dt and reference discretization changed independently.
        for dt,spacing in [(.01,.25),(.005,.25),(.01,.125)]:
            row,_,_=tracking.simulate(controller=controller,dt=dt,spacing=spacing)
            convergence.append(row)
    csv_save(OUT/'data'/'task3_convergence.csv',convergence)
    return dict(cases=rows,convergence=convergence)


def run_speed():
    cases=[('balanced',2.,1.5,25.),('strong_braking',5.,3.,25.),
           ('weak_braking',.01,.01,25.),('gentle',1.,.8,25.),('zero_limits',0.,0.,45.)]
    rows=[]; fig,axes=plt.subplots(2,2,figsize=(12,7))
    for name,b,ay,v0 in cases:
        row,d=speed.plan(b,ay,v0); rows.append(dict(name=name,**row))
        if d is None: continue
        series_save('task4_'+name,d)
        axes[0,0].plot(d['s_m'],d['speed_mps']*3.6,label=name)
        axes[0,1].plot(d['s_m'],d['accel_mps2'],label=name)
        axes[1,0].plot(d['s_m'],d['combined_limit_mps']*3.6,label=name)
    axes[0,0].plot(speed.BOUNDARIES,np.r_[speed.TRAFFIC,speed.TRAFFIC[-1]]*3.6,'k--',drawstyle='steps-post',label='Trafik limiti')
    for ax,title,unit in [(axes[0,0],'Planlanan hız','km/sa'),(axes[0,1],'Parça sabit ivme','m/s²'),
                          (axes[1,0],'Birleşik hız limitleri','km/sa')]:
        ax.set(xlabel='Yol boyunca mesafe [m]',ylabel=unit,title=title); ax.grid(alpha=.25); ax.legend(fontsize=8)
    x,y=speed.geometry(np.linspace(0,3000,3001)); axes[1,1].plot(x,y)
    axes[1,1].set(xlabel='X [m]',ylabel='Y [m]',title='Ortak yol geometrisi; takip performansı değildir')
    axes[1,1].axis('equal'); axes[1,1].grid(alpha=.25)
    figure_save('task4_speed')
    # Homogeneous columns even when an experiment is infeasible.
    keys=list(dict.fromkeys(k for r in rows for k in r))
    csv_save(OUT/'data'/'task4_summary.csv',[{k:r.get(k) for k in keys} for r in rows])
    convergence=[speed.plan(ds=ds)[0] for ds in (2.,1.,.5)]
    csv_save(OUT/'data'/'task4_convergence.csv',convergence)
    return dict(cases=rows,convergence=convergence)


def main():
    global OUT
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=OUT,help='Output directory; default is relative to this script')
    args=parser.parse_args()
    OUT=args.output.resolve()
    for folder in ('data','figures','pdf'): (OUT/folder).mkdir(parents=True,exist_ok=True)
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    result={'environment':dict(python=sys.version.split()[0],numpy=np.__version__,matplotlib=matplotlib.__version__,platform=platform.platform())}
    for name,fn in [('task1',run_braking),('task2',run_trajectory),('task3',run_tracking),('task4',run_speed)]:
        print(name,flush=True); result[name]=fn()
    (OUT/'data'/'results.json').write_text(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8')
    print('Results:',OUT,flush=True)


if __name__=='__main__':
    main()
