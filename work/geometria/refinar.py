"""Segunda etapa: comprobar la parada con el mejor umbral/anticipacion de entrenamiento."""
import json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
from calibrar import job
ROOT=Path(__file__).resolve().parent

def key(r):
    return r['clean'],r['arrivals'],-r['contacts'],-r['mean_time']

def summary(rows,t,l,s):
    return dict(threshold=t,look=l,stop=s,n=len(rows),
        clean=sum(r['reached'] is not None and r['contacts']==0 for r in rows),
        arrivals=sum(r['reached'] is not None for r in rows),contacts=sum(r['contacts'] for r in rows),
        mean_time=sum(r['elapsed'] for r in rows)/len(rows))

if __name__=='__main__':
    data=json.loads((ROOT/'calibracion.json').read_text())
    best=max((r for r in data['summary'] if r['stop']==30),key=key)
    configs=[(t,l,35) for t in (50,70,90) for l in (50,70,90)
             if not any((r['threshold'],r['look'],r['stop'])==(t,l,35) for r in data['summary'])]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for t,l,s in configs:
            jobs=[(t,l,s,seed,3) for seed in range(1000,1020)]+[(t,l,s,0,4)]
            rows=list(pool.map(job,jobs))
            data['results']+=rows
            data['summary'].append(summary(rows,t,l,s))
    best=max(data['summary'],key=key).copy()
    best['effective_stop']=max(30,best['stop'])
    data['selected']=best
    (ROOT/'calibracion_final.json').write_text(json.dumps(data,indent=2))
    print(json.dumps(best,indent=2))
