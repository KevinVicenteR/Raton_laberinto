#!/usr/bin/env python3
"""Compile an exported candidate model and compare real C++ inference with Python."""
import argparse
import json
from pathlib import Path
import random
import shutil
import subprocess
import tempfile
from navigation_experiments import Predictor

def main():
    p=argparse.ArgumentParser();p.add_argument('--model',required=True);p.add_argument('--header',required=True);p.add_argument('--workdir',required=True);args=p.parse_args()
    root=Path(__file__).resolve().parents[1];header=Path(args.header).resolve()
    compiler=shutil.which('c++')
    if not compiler:raise RuntimeError('C++ compiler required; comparison is not skipped')
    destination=Path(args.workdir);destination.mkdir(parents=True,exist_ok=True)
    model=json.loads(Path(args.model).read_text());predict=Predictor(model);rng=random.Random(106)
    samples=[[rng.random() for _ in range(43)] for _ in range(100)]
    source='#include <iostream>\n#include <iomanip>\n#include '+json.dumps(str(header))+'\nint main(){float x[43];while(std::cin>>x[0]){for(int i=1;i<43;++i)if(!(std::cin>>x[i]))return 2;std::cout<<std::setprecision(10)<<raton::predictRisk(raton::kRiskModel,x)<<"\\n";}}\n'
    with tempfile.TemporaryDirectory(dir=destination) as directory:
        directory=Path(directory);(directory/'check.cpp').write_text(source)
        subprocess.run([compiler,'-std=c++17','-O2','-I'+str(root/'lib/raton_core/src'),str(directory/'check.cpp'),str(root/'lib/raton_core/src/risk_predictor.cpp'),'-o',str(directory/'check')],check=True)
        inputs=''.join(' '.join(format(v,'.9g') for v in x)+'\n' for x in samples)
        output=subprocess.run([str(directory/'check')],input=inputs,text=True,capture_output=True,check=True)
        native=[float(v) for v in output.stdout.split()]
    if len(native)!=len(samples):raise RuntimeError('incomplete inference output')
    error=max(abs(a-predict(x)) for a,x in zip(native,samples))
    if error>2e-5:raise RuntimeError(f'inference mismatch: {error}')
    print(json.dumps(dict(passed=True,samples=len(samples),max_absolute_error=error,model=str(Path(args.model).resolve()))))
if __name__=='__main__':main()
