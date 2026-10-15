#!/usr/bin/env python3
"""Save representative traces, maneuver tests and an offline replay (no server or network)."""
import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
import json
from pathlib import Path
import sim_extremos as S
import sim_pistas as P
from navigation_experiments import Predictor, run_job


def isolated():
    fw=P.FW
    mirror=lambda maze:[line[::-1] for line in maze]
    cases=[
        ('straight',S.M_L,(2,0),(2,2)),
        ('left',S.M_L,(2,0),(0,2)),
        ('right',mirror(S.M_L),(0,0),(2,2)),
        ('T',S.M_T,(2,0),(0,2)),
        ('dead-end-reverse',S.M_DEAD,(1,0),(0,2)),
        ('zigzag',S.random_maze(5,5,seed=3,start=(4,0)),(4,0),(0,4)),
    ]
    results=[]
    for name,maze,start,goal in cases:
        for off in (0,5):
            sim=S.Sim(S.Case(name,maze,start,goal,mode=3,fw=fw,controller='corrected',
                             off_x=off,off_deg=0 if off==0 else 3,seconds=60),seed=1).run()
            results.append(dict(case=name,offset_mm=off,reached=sim.reached,contacts=sim.collisions,
                                clean_arrival=sim.reached is not None and sim.collisions==0,
                                failure=sim.failure,motions=sim.motions))
    return results


def main():
    p=argparse.ArgumentParser();p.add_argument('--out',required=True);p.add_argument('--model',required=True)
    args=p.parse_args();out=Path(args.out);out.mkdir(parents=True,exist_ok=True)
    jobs=[(0,1,4,'corrected',args.model,None,'competition',None,True),
          (101,1,3,'corrected',args.model,None,'random',None,True),
          (100,1,5,'corrected',args.model,None,'random',None,True)]
    replays=[]
    with ProcessPoolExecutor(max_workers=3) as pool:
        for row in pool.map(run_job,jobs):
            name=f'{row["suite"]}-{row["maze"]}-mode{row["mode"]}'
            (out/f'{name}-trace.json').write_text(json.dumps(row))
            maze=S.comp_maze() if row['suite']=='competition' else S.random_maze(seed=row['maze'])
            case=S.Case(name,maze,P.START,P.GOAL)
            walls,pitch,rows,cols=S.build_walls(case)
            frames=[[r['t'],*r['truth']] for r in row['trace'][::5]]
            if row['trace'] and frames[-1][0]!=row['trace'][-1]['t']:
                frames.append([row['trace'][-1]['t'],*row['trace'][-1]['truth']])
            contact=next((e for e in row['events'] if e[1]=='choque'),None)
            diagnostic=None
            if contact:
                overlapping=[m for m in row['motions'] if m['t']<=contact[0]<=m['end']]
                m=min(overlapping,key=lambda m:m['end']-m['t']) if overlapping else None
                samples=[r for r in row['trace'] if r['t']<=contact[0]]
                sample=samples[-1] if samples else None
                diagnostic=dict(first_contact_s=contact[0],phase=m['kind'] if m else 'unknown',
                                observation=sample['sensors'] if sample else None,
                                sample_age_ms=1000*(contact[0]-sample['t']) if sample else None,
                                note='Phase correlation, not proof of a physical root cause')
            replays.append(dict(name=name,frames=frames,walls=walls,pitch=pitch,width=cols*pitch,height=rows*pitch,
                                goal=P.GOAL,events=row['events'],reached=row['reached'],failure=row['failure'],
                                contacts=row['contacts'],diagnostic=diagnostic))
    (out/'isolated-maneuvers.json').write_text(json.dumps(isolated()))
    (out/'replay-data.json').write_text(json.dumps(replays))
    template=Path(__file__).with_name('navigation_replay_template.html').read_text()
    (out/'navigation-replays.html').write_text(template.replace('/* REPLAY_DATA */[]',json.dumps(replays)))
    print('Saved traces, isolated maneuver results and navigation-replays.html')


if __name__=='__main__':main()
