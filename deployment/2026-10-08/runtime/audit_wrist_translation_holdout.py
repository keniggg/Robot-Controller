#!/usr/bin/env python3
"""Offline fixed-X translation diagnostic with nonparallel wrist holdouts.
Uses each recording's own baseline; never compares board coordinates across bags.
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('fixed_translation',ROOT/'.runtime/analyze_fixed_handeye_translation.py')
F=importlib.util.module_from_spec(spec);spec.loader.exec_module(F)

def wrist_session(resume,joint,record):
    path=resume/('wrist_multiframe_j%d'%joint)/'samples.json'
    directory=F.ARCHIVE/'ros_log'/record
    return dict(joint=joint,rows=json.loads(path.read_text()),geometry=json.loads((directory/'baseline_geometry.json').read_text()),directory=str(directory),source=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest())

def run(resume,output,nboot=200):
    sessions=F.load()+[wrist_session(resume,4,'bounded_wrist_response_diagnostic_20260928T125306Z_126028'),wrist_session(resume,6,'bounded_wrist_response_diagnostic_20260928T125421Z_130257')]
    xs=[np.asarray(s['geometry']['T_tool_camera']) for s in sessions]
    if not all(np.allclose(x,xs[0],atol=1e-10,rtol=0) for x in xs):raise ValueError('handeye not constant across recordings')
    report=dict(kind='offline_fixed_translation_fit_with_nonparallel_wrist_holdout',sessions=[{k:s[k] for k in ('joint','directory','source','sha256')} for s in sessions],fits={},limitations=['Only within-recording board changes are used. Each recording has its own unknown board origin.','Fixed rotation held unchanged. This cannot repair rotation-angle invariance errors.','Block bootstrap is conditional on measured scenes; systematic camera/board/kinematic errors are not covered.','Diagnostic only; no TF, URDF or joint parameter writes.'])
    for label,point in [('board_origin',np.zeros(3)),('board_center',F.CENTER)]:
        contrasts,returns,windows=F.build(sessions,point)
        c_old=[c for c in contrasts if c['joint'] in (2,3,5)]
        c_wrist=[c for c in contrasts if c['joint'] in (4,6)]
        fits={}
        for name,train,held in [('old_parallel_only',c_old,c_wrist),('wrist_only',c_wrist,c_old),('all_five_axes',contrasts,[])]:
            solution=F.fit(train)
            fits[name]=dict(correction_tool_mm=solution['correction']*1000,rank=solution['rank'],singular=solution['singular'],condition=solution['condition'],training=F.evaluate(train,solution['correction']),heldout=F.evaluate(held,solution['correction']),return_holdout=F.evaluate(returns,solution['correction']))
        rng=np.random.default_rng(2809);bs=[]
        for _ in range(nboot):
            c,_r,_w=F.build(sessions,point,rng);bs.append(F.fit(c)['correction']*1000)
        fits['all_five_axes']['conditional_half_sec_bootstrap_mm']=F.percentile(bs)
        fits['all_five_axes']['leave_one_axis_out']=[]
        for c in contrasts:
            solution=F.fit([v for v in contrasts if v['joint']!=c['joint']])
            fits['all_five_axes']['leave_one_axis_out'].append(dict(joint=c['joint'],rank=solution['rank'],condition=solution['condition'],correction_tool_mm=solution['correction']*1000,heldout=F.evaluate([c],solution['correction'])[0]))
        report['fits'][label]=fits
    output.mkdir(parents=True,exist_ok=True)
    (output/'wrist_translation_holdout.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,default=F.as_json,allow_nan=False)+'\n')
    (output/'analysis_source.py').write_bytes(Path(__file__).read_bytes())
    for point,fits in report['fits'].items():
        for name,fit in fits.items():print(json.dumps(dict(point=point,fit=name,correction=fit['correction_tool_mm'],rank=fit['rank'],condition=fit['condition'],training=fit['training'],heldout=fit['heldout']),default=F.as_json))
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('resume',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--bootstrap',type=int,default=200);a=p.parse_args()
    if not os.path.ismount(F.MOUNT) or F.MOUNT not in a.output.resolve().parents:raise RuntimeError('SanStand output required')
    run(a.resume,a.output,a.bootstrap)
