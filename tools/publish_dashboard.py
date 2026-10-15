#!/usr/bin/env python3
"""Publish measured results and reproducible traces to the local simulation/telemetry pages."""
import argparse
from concurrent.futures import ProcessPoolExecutor
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sim_extremos as S
import sim_pistas as P
from navigation_experiments import run_job

LABELS={'precision':'Giro preciso sin barrido (prueba)','unguarded':'Sin barrido (prueba)','original':'Control anterior','corrected':'Barrido observado','precision-guard':'Barrido + corrección angular',
        'hybrid':'Control con memoria y corrección angular','hybrid-neural':'Control con memoria + red de ronda 2','mapped':'Barrido con memoria de paredes','mapped-raw':'Memoria de paredes + lecturas crudas','survey':'20 observaciones por celda','ranked':'Red de ronda 2 con reversa como último recurso'}

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--evaluation',required=True)
    parser.add_argument('--model',required=True)
    parser.add_argument('--latest-model')
    parser.add_argument('--out',default=str(Path(__file__).parent/'sim_resultados'))
    parser.add_argument('--controller',default='hybrid')
    args=parser.parse_args()
    evaluation=json.loads(Path(args.evaluation).read_text())
    if not evaluation.get('complete'):raise ValueError('Evaluation is incomplete; refusing to publish it as complete')
    rows=evaluation['results']; selected=[r for r in rows if r['controller']==args.controller]
    if not selected:raise ValueError('Controller missing')
    examples=[]
    for candidates in ([r for r in selected if r['clean_arrival']],
                       [r for r in selected if r['contacts']>0],
                       [r for r in selected if r['reached'] is None]):
        candidate=next((r for r in candidates if r not in examples),None)
        if candidate:examples.append(candidate)
    jobs=[(r['maze'],r['noise_seed'],r['mode'],r['controller'],args.model,None,r['suite'],None,True) for r in examples]
    out=Path(args.out);out.mkdir(parents=True,exist_ok=True);replays=[]
    with ProcessPoolExecutor(max_workers=3) as pool:
        for expected,row in zip(examples,pool.map(run_job,jobs)):
            for key in ('contacts','clean_arrival','reached','failure'):
                if row[key]!=expected[key]:raise ValueError('Replay differs from evaluation: '+key)
            maze=S.comp_maze() if row['suite']=='competition' else P.altered_comp() if row['suite']=='altered' else S.random_maze(seed=row['maze'])
            walls,pitch,height,width=S.build_walls(S.Case('replay',maze,P.START,P.GOAL))
            row.update(maze=maze,maze_seed=expected['maze'],walls=walls,pitch=pitch,width=width*pitch,height=height*pitch,start=P.START,goal=P.GOAL)
            row['trace']=row['trace'][::5]+([row['trace'][-1]] if (len(row['trace'])-1)%5 else [])
            for motion in row['motions']:motion.pop('features',None)
            ident=f"{expected['suite']}-{expected['maze']}-n{expected['noise_seed']}-m{expected['mode']}-{args.controller}"
            filename=ident+'.json';(out/filename).write_text(json.dumps(row,separators=(',',':')))
            state='Llegada sin contacto' if row['clean_arrival'] else 'Llegada con contacto' if row['reached'] is not None else 'Fallo de navegación'
            replays.append(dict(id=ident,label=state,summary=f"Modo {row['mode']} · mapa {expected['maze']} · {row['contacts']} contactos",file='sim_resultados/'+filename))
    catalog=dict(generated_at=datetime.now(timezone.utc).isoformat(),run_id='Segunda ronda · '+hashlib.sha256(Path(args.evaluation).read_bytes()).hexdigest()[:12],
        objective_met=all(r['clean_arrival'] for r in selected),default_controller=args.controller,controller_labels=LABELS,results=rows,replays=replays,
        model_metadata={k:v for k,v in json.loads(Path(args.model).read_text()).items() if k not in ('w1','w2','w3','b1','b2','b3')},complete=True,model_enabled=False,scope='Simulación experimental; firmware sin activar estos candidatos')
    catalog['model_validation']=evaluation.get('model_validation')
    if args.latest_model:catalog['latest_training']={k:v for k,v in json.loads(Path(args.latest_model).read_text()).items() if k not in ('w1','w2','w3','b1','b2','b3')}
    (out/'dashboard.json').write_text(json.dumps(catalog,separators=(',',':')))
    print(json.dumps(dict(cases=len(rows),replays=len(replays),objective_met=catalog['objective_met'])))

if __name__=='__main__':main()
