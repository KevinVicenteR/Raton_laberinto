#!/usr/bin/env python3
"""Reproducible training/evaluation; numpy is needed only for training.

Examples:
  python3 tools/navigation_experiments.py collect --out work/nav-data.json --budget 7200
  python3 tools/navigation_experiments.py train --data work/nav-data.json --out work/risk.json
  python3 tools/navigation_experiments.py evaluate --out work/evaluation.json --model work/risk.json
Set RATON_NATIVE_LIB to the CMake raton_navigation library. All paths are explicit.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import random
import sys
import time

import sim_extremos as S
import sim_pistas as P
from navigation import firmware_config


class Predictor:
    def __init__(self, model):
        self.m = model
    def __call__(self, x):
        if len(x)!=43 or not all(math.isfinite(v) for v in x):
            return float('nan')
        def layer(w,b,x):
            return [max(0,bi+sum(a*v for a,v in zip(row,x))) for row,bi in zip(w,b)]
        a=layer(self.m['w1'],self.m['b1'],x)
        b=layer(self.m['w2'],self.m['b2'],a)
        z=self.m['b3']+sum(a*v for a,v in zip(self.m['w3'],b))
        return 1/(1+math.exp(-max(-80,min(80,z))))


def run_job(job):
    maze_seed,noise_seed,mode,controller,model_path,threshold,suite,baseline_path,trace = job
    module=S
    if controller=='original':
        spec=importlib.util.spec_from_file_location('nav_original',baseline_path)
        module=importlib.util.module_from_spec(spec)
        sys.modules['nav_original']=module
        spec.loader.exec_module(module)
    fw=firmware_config(module)
    if controller in ('precision','precision-guard','precision-neural','precision-guard-neural','hybrid','hybrid-neural','survey','ranked','mapped','mapped-raw','mapped-ranked','mapped-ctrl','mapped-logic','mapped-both'):
        fw=replace(fw,precision_turns=True)
    if controller in ('hybrid','hybrid-neural','survey','ranked','mapped','mapped-raw','mapped-ranked','mapped-ctrl','mapped-logic','mapped-both'):
        fw=replace(fw,goal_grid=True,risk_rank=controller in ('hybrid-neural','ranked','mapped-ranked'),risk_reverse_last=controller in ('ranked','mapped-ranked'))
    if controller in ('survey','mapped-raw'):fw=replace(fw,survey_samples=20,raw_survey=controller=='mapped-raw')
    known=None
    if suite in ('competition','altered'):
        maze=S.comp_maze() if suite=='competition' else P.altered_comp()
        known=S.comp_maze() if suite=='altered' else None
        seconds=120 if suite=='competition' else 300
    else:
        maze=S.random_maze(seed=maze_seed)
        seconds=600
    sensor=module.Sensor()
    if suite=='data':
        rng=random.Random(maze_seed*10+noise_seed)
        sensor=replace(sensor,noise=rng.choice((1.,3.,6.)),dropout=rng.choice((.0,.02,.08)))
        if module is S:
            sensor=replace(sensor,latency_ms=rng.choice((0,20,40)))
    opts=dict(mode=mode,fw=fw,use_goal=mode==5,known=known,seconds=seconds,sensor=sensor)
    if module is S:
        kind='neural' if controller in ('neural','precision-neural','precision-guard-neural','hybrid-neural','ranked','mapped-ranked') else ('baseline' if controller=='original' else 'corrected')
        opts.update(controller=kind,trace=trace,sweep_guard=controller not in ('precision','precision-neural','unguarded'))
    simulator=module.Sim
    if controller in ('mapped','mapped-raw','mapped-ranked','mapped-ctrl','mapped-logic','mapped-both'):
        from navigation_candidates import ObservedMapSim
        simulator=ObservedMapSim
        if mode==4:opts['known']=S.comp_maze()
    sim=simulator(module.Case(str(maze_seed),maze,P.START,P.GOAL,**opts),seed=noise_seed)
    if controller in ('mapped-ctrl','mapped-both'):
        import nets
        sim.ctrl_net=nets.load(os.environ.get('RATON_CONTROL_MODEL',str(Path(__file__).parent/'models/control_model.json')))
        if sim.ctrl_net is None:raise RuntimeError('falta el modelo de control')
    if controller in ('mapped-logic','mapped-both'):
        import nets
        sim.logic_net=nets.load(os.environ.get('RATON_LOGIC_MODEL',str(Path(__file__).parent/'models/logic_model.json')))
        if sim.logic_net is None:raise RuntimeError('falta el modelo de logica')
    if model_path and module is S:
        model=json.loads(Path(model_path).read_text())
        sim.predictor=Predictor(model)
        sim.risk_threshold=threshold
    sim.run()
    cx,cy=sim.x//sim.pitch,sim.y//sim.pitch
    center=((cx+.5)*sim.pitch,(cy+.5)*sim.pitch)
    # Offline evaluation only; these truth coordinates are never a model input.
    axle=(sim.c.robot.length_front-sim.c.robot.length_back)/2
    body_center=(sim.x+axle*math.cos(sim.th),sim.y+axle*math.sin(sim.th))
    position_error=math.hypot(body_center[0]-center[0],body_center[1]-center[1])
    out=dict(maze=maze_seed,noise_seed=noise_seed,mode=mode,controller=controller,suite=suite,
             reached=sim.reached,contacts=sim.collisions,clean_arrival=sim.reached is not None and sim.collisions==0,
             stopped=sim.reached is None,failure=sim.failure,elapsed=sim.t,position_error_mm=position_error,
             turns=sim.turns,navigation_policy='cell-memory' if getattr(fw,'goal_grid',False) else 'original-mode',
             precision_turns=fw.precision_turns if hasattr(fw,'precision_turns') else False,
             survey_samples=fw.survey_samples if hasattr(fw,'survey_samples') else 5,
             raw_survey=fw.raw_survey if hasattr(fw,'raw_survey') else False,
             mapped_guard=controller in ('mapped','mapped-raw','mapped-ranked','mapped-ctrl','mapped-logic','mapped-both'),
             goal_position_error_mm=math.hypot(body_center[0]-(P.GOAL[0]+.5)*sim.pitch,
                                               body_center[1]-(P.GOAL[1]+.5)*sim.pitch))
    if suite=='data' or trace:
        out['motions']=sim.motions
    if trace:
        if not sim.trace or sim.trace[-1]['t']<sim.t:
            sim.trace.append(dict(t=sim.t,raw=sim.raw_readings.copy(),filtered=sim.readings.copy(),
                sensors=sim.observation(),encoder_mm=sim.travelled,heading_deg=math.degrees(sim.th),
                truth=(sim.x,sim.y,sim.th)))
        out.update(trace=sim.trace,trail=sim.trail,events=sim.events,ctrl_trace=sim.ctrl_trace,logic_trace=sim.logic_trace)
    return out


def evaluate(args):
    jobs=[]
    if args.model and args.threshold is None:
        args.threshold=json.loads(Path(args.model).read_text())['threshold']
    controllers=tuple(args.controllers.split(',')) if args.controllers else (('original','corrected','neural') if args.model else ('original','corrected'))
    allowed={'original','corrected','neural','precision','precision-guard','precision-neural','precision-guard-neural','unguarded','hybrid','hybrid-neural','survey','ranked','mapped','mapped-raw','mapped-ranked','mapped-ctrl','mapped-logic','mapped-both'}
    if not set(controllers)<=allowed:raise ValueError('unknown controller')
    for controller in controllers:
        if args.suite in ('21','all'):
            for seed in (1,2,3):
                jobs.append((0,seed,4,controller,args.model,args.threshold,'competition',args.baseline,False))
            for seed in (1,2):
                jobs.append((0,seed,4,controller,args.model,args.threshold,'altered',args.baseline,False))
            for maze in range(100,108):
                for mode in (3,5):jobs.append((maze,1,mode,controller,args.model,args.threshold,'random',args.baseline,False))
        if args.suite in ('reserved','fresh','all'):
            for maze in (range(400,430) if args.suite=='fresh' else range(200,230)):
                for noise in (1,2,3):
                    for mode in (3,5):jobs.append((maze,noise,mode,controller,args.model,args.threshold,'fresh' if args.suite=='fresh' else 'reserved',args.baseline,False))
    rows=[]
    source_hashes={n:hashlib.sha256((Path(__file__).parent/n).read_bytes()).hexdigest()
                   for n in ('sim_extremos.py','navigation.py','navigation_experiments.py','navigation_candidates.py')}
    model_hash=hashlib.sha256(Path(args.model).read_bytes()).hexdigest() if args.model else None
    start=time.monotonic()
    # Progress is durable after each completed case; a killed run is explicitly incomplete.
    output=Path(args.out); output.parent.mkdir(parents=True,exist_ok=True)
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for row in pool.map(run_job,jobs,chunksize=1):
            rows.append(row)
            output.write_text(json.dumps(dict(complete=len(rows)==len(jobs),expected=len(jobs),
                                              elapsed_wall=time.monotonic()-start,results=rows,
                                              threshold=args.threshold,config=str(P.FW),
                                              source_sha256=source_hashes,model_sha256=model_hash)))
            print(f'{len(rows)}/{len(jobs)} {row["controller"]} maze={row["maze"]} mode={row["mode"]} '
                  f'clean={row["clean_arrival"]} contacts={row["contacts"]} reason={row["failure"]}',flush=True)


def collect(args):
    start=time.monotonic(); jobs=[]
    for split,seeds in [('train',range(1000,1040)),('validation',range(1100,1110))]:
        for maze in seeds:
            for noise in args.noises:
                for mode in (3,5):
                    controller=args.training_controller if args.training_controller else ('precision-guard' if args.precision else ('baseline' if noise==1 else 'corrected'))
                    jobs.append((maze,noise,mode,controller,None,None,'data',None,False))
    rows=[];examples=[]
    pool=ProcessPoolExecutor(max_workers=args.workers)
    try:
        futures=[pool.submit(run_job,j) for j in jobs]
        try:
            for future in futures:
                remaining=args.budget-(time.monotonic()-start)
                if remaining<=0:break
                try:row=future.result(timeout=remaining)
                except TimeoutError:break
                rows.append({k:v for k,v in row.items() if k!='motions'})
                split='train' if row['maze']<1100 else 'validation'
                for m in row['motions']:
                    if m['reason'] in ('goal_reached','simulation_closed'):continue
                    if m['kind'] not in ('safe_turn','turn_corner_track','run_cells'):continue
                    if m['kind']=='run_cells' and m['actual_mm']<0:continue
                    if abs(m['requested']) not in (90,) and m['kind']!='run_cells':continue
                    examples.append(dict(maze=row['maze'],split=split,x=m['features'],
                                         y=int(m['contacts']>0 or not m['completed']),kind=m['kind']))
                print(f'{len(rows)}/{len(jobs)} episodes; {len(examples)} labelled maneuvers',flush=True)
        finally:
            for f in futures:f.cancel()
    finally:
        if time.monotonic()-start >= args.budget:
            # Python <=3.13 has no terminate_workers public API. Stop child work at the deadline.
            for process in (pool._processes or {}).values():process.terminate()
        pool.shutdown(wait=True,cancel_futures=True)
    Path(args.out).write_text(json.dumps(dict(complete=len(rows)==len(jobs),expected=len(jobs),
        wall_seconds=time.monotonic()-start,budget_seconds=args.budget,examples=examples,episodes=rows)))


def train(args):
    start=time.monotonic()
    import numpy as np
    rng=np.random.default_rng(42)
    data=json.loads(Path(args.data).read_text())
    args.budget=min(args.budget,max(0,7200-data['wall_seconds']))
    if args.budget<=0:raise RuntimeError('Combined two-hour data/training budget exhausted')
    if not data['complete']:
        raise RuntimeError('Data generation incomplete; refusing to present a full training run')
    groups={k:[r for r in data['examples'] if r['split']==k] for k in ('train','validation')}
    if set(r['maze'] for r in groups['train']) & set(r['maze'] for r in groups['validation']):
        raise RuntimeError('maze leakage')
    x=np.array([r['x'] for r in groups['train']],dtype=np.float32)
    y=np.array([r['y'] for r in groups['train']],dtype=np.float32)[:,None]
    vx=np.array([r['x'] for r in groups['validation']],dtype=np.float32)
    vy=np.array([r['y'] for r in groups['validation']],dtype=np.float32)[:,None]
    if min(y.sum(),len(y)-y.sum(),vy.sum(),len(vy)-vy.sum())<=0:
        raise RuntimeError('both outcomes required in both splits')
    params=[rng.normal(0,np.sqrt(2/43),(43,32)).astype('f'),np.zeros(32,'f'),
            rng.normal(0,np.sqrt(2/32),(32,16)).astype('f'),np.zeros(16,'f'),
            rng.normal(0,np.sqrt(2/16),(16,1)).astype('f'),np.zeros(1,'f')]
    def forward(x):
        a=np.maximum(0,x@params[0]+params[1]); b=np.maximum(0,a@params[2]+params[3])
        z=np.clip(b@params[4]+params[5],-30,30)
        return a,b,1/(1+np.exp(-z))
    best=float('inf'); saved=None; epochs=0
    for epoch in range(200):
        if time.monotonic()-start>args.budget:break
        order=rng.permutation(len(x))
        for indices in np.array_split(order,max(1,int(np.ceil(len(x)/128)))):
            xx,yy=x[indices],y[indices];a,b,p=forward(xx)
            dz=(p-yy)*np.where(yy>0,args.positive_weight,1)/len(xx)
            db=(dz@params[4].T)*(b>0);da=(db@params[2].T)*(a>0)
            grads=[xx.T@da,da.sum(0),a.T@db,db.sum(0),b.T@dz,dz.sum(0)]
            for v,g in zip(params,grads):v-=.03*np.clip(g,-1,1)
        pred=np.clip(forward(vx)[2],1e-6,1-1e-6)
        loss=float(-(vy*np.log(pred)+(1-vy)*np.log(1-pred)).mean());epochs=epoch+1
        if loss<best:best=loss;saved=[v.copy() for v in params]
    params[:]=saved
    pred=forward(vx)[2].ravel();labels=vy.ravel()
    # Validation-only threshold: reject at least 90% of dangerous maneuvers, minimize rejected safe ones.
    options=[]
    for threshold in (.05,.1,.2,.3,.4,.5,.6,.7,.8,.9):
        recall=float((pred[labels==1]>threshold).mean())
        false_reject=float((pred[labels==0]>threshold).mean())
        if recall>=.9:options.append((false_reject,-threshold,threshold,recall))
    choice=min(options) if options else None
    model=dict(w1=params[0].T.tolist(),b1=params[1].tolist(),w2=params[2].T.tolist(),
               b2=params[3].tolist(),w3=params[4].ravel().tolist(),b3=float(params[5][0]),
               threshold=choice[2] if choice else None,enabled=False,architecture=[43,32,16,1],
               seed=42,epochs=epochs,positive_weight=args.positive_weight,validation_bce=best,train_examples=len(x),validation_examples=len(vx),
               threshold_validation=choice,wall_seconds=time.monotonic()-start,
               data_sha256=hashlib.sha256(Path(args.data).read_bytes()).hexdigest(),
               train_mazes=sorted(set(r['maze'] for r in groups['train'])),
               validation_mazes=sorted(set(r['maze'] for r in groups['validation'])))
    Path(args.out).write_text(json.dumps(model))
    if args.export:
        def cpp(v):
            if isinstance(v,list):return '{'+','.join(cpp(x) for x in v)+'}'
            text=format(v,'.9g')
            if '.' not in text and 'e' not in text:text+='.0'
            return text+'f'
        initializer='{'+','.join(cpp(model[k]) for k in ('w1','b1','w2','b2','w3','b3'))+'}'
        Path(args.export).write_text('#pragma once\n#include "risk_predictor.h"\nnamespace raton {\n'
            'constexpr bool kRiskModelTrained=true;\nconstexpr bool kRiskModelEnabled=false;\n'
            f'constexpr float kRiskThreshold={cpp(model["threshold"] or .05)};\n'
            f'static const RiskModel kRiskModel={initializer};\n}}\n')
    print(json.dumps({k:v for k,v in model.items() if k not in ('w1','w2','w3','b1','b2','b3')}))


def calibrate(args):
    """Navigation validation, strictly on the existing validation mazes, never reserved test maps."""
    model=json.loads(Path(args.model).read_text())
    thresholds=sorted(set((model['threshold'],.5,.8,.95,.99))-{None})
    jobs=[]
    for threshold in [None]+thresholds:
        for maze in model['validation_mazes']:
            for noise in (1,2):
                for mode in (3,5):
                    jobs.append((maze,noise,mode,args.controller if threshold is None else ('precision-neural' if args.controller=='precision' else 'neural'),
                                 args.model,threshold,'validation',None,False))
    rows=[]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for job,row in zip(jobs,pool.map(run_job,jobs)):
            row['threshold']=job[5];rows.append(row)
            print(f'{len(rows)}/{len(jobs)} threshold={job[5]} clean={row["clean_arrival"]}',flush=True)
    def metrics(threshold):
        r=[x for x in rows if x['threshold']==threshold]
        return dict(threshold=threshold,clean=sum(x['clean_arrival'] for x in r),
                    arrivals=sum(x['reached'] is not None for x in r),contacts=sum(x['contacts'] for x in r))
    base=metrics(None); candidates=[metrics(t) for t in thresholds]
    old_clean={(r['maze'],r['noise_seed'],r['mode']) for r in rows if r['threshold'] is None and r['clean_arrival']}
    eligible=[]
    for c in candidates:
        new_clean={(r['maze'],r['noise_seed'],r['mode']) for r in rows if r['threshold']==c['threshold'] and r['clean_arrival']}
        if c['clean']>base['clean'] and c['arrivals']>=base['arrivals'] and old_clean<=new_clean:
            eligible.append(c)
    selected=max(eligible,key=lambda c:(c['clean'],c['arrivals'],-c['contacts'],c['threshold'])) if eligible else None
    Path(args.out).write_text(json.dumps(dict(complete=True,expected=len(jobs),results=rows,baseline=base,
        candidates=candidates,selected_threshold=selected['threshold'] if selected else None,
        model_sha256=hashlib.sha256(Path(args.model).read_bytes()).hexdigest(),
        test_mazes_used=False)))



def validate_ranking(args):
    model=json.loads(Path(args.model).read_text())
    jobs=[(maze,noise,mode,controller,args.model,None,'validation',None,False)
          for controller in ('mapped','mapped-ranked') for maze in model['validation_mazes']
          for noise in (1,2) for mode in (3,5)]
    rows=[]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for row in pool.map(run_job,jobs):
            rows.append(row)
            print(f'{len(rows)}/{len(jobs)} {row["controller"]} clean={row["clean_arrival"]}',flush=True)
    metrics={}
    for controller in ('mapped','mapped-ranked'):
        part=[r for r in rows if r['controller']==controller]
        metrics[controller]=dict(clean=sum(r['clean_arrival'] for r in part),arrivals=sum(r['reached'] is not None for r in part),contacts=sum(r['contacts'] for r in part))
    a,b=metrics['mapped'],metrics['mapped-ranked']
    clean=lambda c:{(r['maze'],r['noise_seed'],r['mode']) for r in rows if r['controller']==c and r['clean_arrival']}
    qualifies=b['clean']>a['clean'] and b['arrivals']>=a['arrivals'] and clean('mapped')<=clean('mapped-ranked')
    Path(args.out).write_text(json.dumps(dict(complete=True,expected=len(jobs),results=rows,metrics=metrics,
       qualifies_for_test=qualifies,model_enabled=False,test_mazes_used=False,
       model_sha256=hashlib.sha256(Path(args.model).read_bytes()).hexdigest())))


def main():
    p=argparse.ArgumentParser(); sub=p.add_subparsers(dest='command',required=True)
    c=sub.add_parser('collect');c.add_argument('--out',required=True);c.add_argument('--budget',type=float,default=7200)
    c.add_argument('--noises',type=lambda value:tuple(int(n) for n in value.split(',')),default=(1,2));c.add_argument('--workers',type=int,default=4);c.add_argument('--precision',action='store_true');c.add_argument('--training-controller',choices=('precision-guard','hybrid','mapped','mapped-raw'))
    t=sub.add_parser('train');t.add_argument('--data',required=True);t.add_argument('--out',required=True)
    t.add_argument('--positive-weight',type=float,default=1.0);t.add_argument('--export');t.add_argument('--budget',type=float,default=1200)
    e=sub.add_parser('evaluate');e.add_argument('--out',required=True);e.add_argument('--model')
    e.add_argument('--baseline',required=True);e.add_argument('--threshold',type=float)
    e.add_argument('--controllers');e.add_argument('--suite',choices=('21','reserved','fresh','all'),default='all');e.add_argument('--workers',type=int,default=4)
    c=sub.add_parser('calibrate');c.add_argument('--model',required=True);c.add_argument('--out',required=True)
    c.add_argument('--workers',type=int,default=4);c.add_argument('--controller',choices=('corrected','precision'),default='corrected')
    v=sub.add_parser('validate-ranking');v.add_argument('--model',required=True);v.add_argument('--out',required=True);v.add_argument('--workers',type=int,default=4)
    args=p.parse_args()
    if args.command=='train' and not 1<=args.positive_weight<=100:p.error('positive-weight must be between 1 and 100')
    if args.command=='collect' and (not args.noises or any(n<0 for n in args.noises)):p.error('nonnegative noise seeds required')
    dict(collect=collect,train=train,evaluate=evaluate,calibrate=calibrate,**{'validate-ranking':validate_ranking})[args.command](args)


if __name__=='__main__':main()
