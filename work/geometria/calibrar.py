"""Calibración reproducible: sólo semillas de entrenamiento y mapa de competencia."""
import sys, os, json, time
from pathlib import Path
from dataclasses import replace
from concurrent.futures import ProcessPoolExecutor
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tools'))
import sim_extremos as S
import sim_pistas as P
from navigation_candidates import ObservedMapSim
from navigation import firmware_config

def job(args):
    threshold,look,stop,seed,mode=args
    fw=replace(firmware_config(S),cell_side_wall=threshold,side_look_ahead=look,
               front_margin=stop-10,goal_grid=True,precision_turns=True)
    maze=S.comp_maze() if seed==0 else S.random_maze(seed=seed)
    sim=ObservedMapSim(S.Case(str(seed),maze,P.START,P.GOAL,mode=mode,fw=fw,
        use_goal=mode==5,known=maze if seed==0 else None,seconds=120 if seed==0 else 600,
        controller='corrected'),seed=1).run()
    return dict(threshold=threshold,look=look,stop=stop,seed=seed,mode=mode,
        reached=sim.reached,contacts=sim.collisions,elapsed=sim.t,failure=sim.failure)

if __name__=='__main__':
    # Coordenadas discretas: barrido de umbral/anticipación; parada nominal y dos márgenes seguros.
    configs=[(t,l,30) for t in (50,70,90) for l in (50,70,90)]
    configs += [(70,70,s) for s in (25,35)]
    jobs=[(*c,seed,mode) for c in configs for seed in range(1000,1020) for mode in (3,)]
    jobs += [(*c,0,4) for c in configs]
    rows=[]
    Path(__file__).with_name("calibracion.json").unlink(missing_ok=True)
    with ProcessPoolExecutor(max_workers=4) as pool:
        for r in pool.map(job,jobs):
            rows.append(r)
            if len(rows)%21==0:print(len(rows),len(jobs),flush=True)
    summaries=[]
    for t,l,s in configs:
        rr=[r for r in rows if (r['threshold'],r['look'],r['stop'])==(t,l,s)]
        summaries.append(dict(threshold=t,look=l,stop=s,n=len(rr),
            clean=sum(r['reached'] is not None and r['contacts']==0 for r in rr),
            arrivals=sum(r['reached'] is not None for r in rr),contacts=sum(r['contacts'] for r in rr),
            mean_time=sum(r['elapsed'] for r in rr)/len(rr)))
    Path(__file__).with_name('calibracion.json').write_text(json.dumps(dict(complete=True,results=rows,summary=summaries),indent=2))
    print(json.dumps(summaries,indent=2))
