#!/usr/bin/env python3
"""Bounded diagnostic SDK lattice search, no path or motion authority."""
import argparse
import hashlib
from itertools import product
import json
import math
import os
from pathlib import Path
import platform
import sys
import time
sys.dont_write_bytecode = True
from replay_gen59_held_axes_20261022 import C, N, CONTEXT, RELEASE, sha


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--input',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seed-kind',choices=('hard-boundary','recorded'),default='hard-boundary')
    args=p.parse_args(); out=args.output.resolve()
    assert str(out).startswith(str(N)+'/') and Path('/media/zhuyupei/SanStand').is_mount()
    out.mkdir(parents=True,exist_ok=False)
    platform._uname_cache=platform.uname_result(*os.uname(),'')
    blocked=[]
    def guard(event, values):
        deny=event.startswith('socket.') or event in ('subprocess.Popen','os.system','os.fork','os.posix_spawn','os.exec')
        if event=='open' and isinstance(values[0],(str,bytes,os.PathLike)):
            file=Path(values[0]).resolve(); mode,flags=values[1:3]
            write=(isinstance(mode,str) and any(x in mode for x in 'wax+')) or (isinstance(flags,int) and bool(flags & (os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC|os.O_APPEND)))
            deny |= write and out not in file.parents
            deny |= str(file).startswith('/dev/') and str(file)!='/dev/null'
        if deny:
            blocked.append(dict(event=event,arguments=str(values)[:200]));raise RuntimeError('offline denied '+event)
    sys.addaudithook(guard)
    sys.path[:0]=[str(C/'src/alicia_flexible_grasp_supervisor/src'),'/opt/ros/noetic/lib/python3/dist-packages','/usr/lib/python3/dist-packages']
    import numpy as np
    from alicia_flexible_grasp.robot.stationary_following import SerialUrdfFk,ARM_NAMES,SDK_QUANTUM_RAD as Q
    from alicia_flexible_grasp.robot.pregrasp_compensation import pose_matrix
    from alicia_flexible_grasp.robot.held_axes_prediction import predict_held_axes_endpoint,HeldAxesPredictionError
    from alicia_flexible_grasp.robot.endpoint_correction import sdk_counts
    source=json.loads(args.input.read_text()); release=json.loads(RELEASE.read_text())
    for name,digest in source['input_hashes'].items():assert sha(name)==digest
    sources={}
    for m in list(sys.modules.values()):
        f=getattr(m,'__file__',None)
        if f and str(f).startswith(str(C)+'/'):
            rel=str(Path(f).relative_to(C));sources[rel]=sha(f);assert sources[rel]==release['source_sha256'][rel]
    fk=SerialUrdfFk(json.loads(CONTEXT.read_text())['/robot_description'],ARM_NAMES)
    def residual(q,target):
        if np.any(q<fk.limits[:,0]) or np.any(q>fk.limits[:,1]):return 1e3,math.pi
        t=fk(q)
        return float(np.linalg.norm(t[:3,3]-target[:3,3])),math.acos(float(np.clip((np.trace(t[:3,:3].T@target[:3,:3])-1)/2,-1,1)))
    rows=[];started=time.monotonic()
    for row in source['rows']:
        sel=row['recorded']['held_pregrasp_selection'];offset=np.array(sel['sdk_minus_encoder_rad'])
        nominal=np.array(sel['original_nominal_command_rad']);pg,ag=map(pose_matrix,(sel['fixed_pregrasp_goal'],sel['fixed_approach_goal']))
        low=np.maximum.reduce((nominal-.25,fk.limits[:,0],fk.limits[:,0]-offset,np.full(6,-math.pi)-offset))
        high=np.minimum.reduce((nominal+.25,fk.limits[:,1],fk.limits[:,1]-offset,np.full(6,(4095-2048)*Q)-offset))
        proposed={};examined=0
        seeds=row['counterfactuals']
        if args.seed_kind=='recorded':
            seeds=[]
            for trial in sel['trials']:
                endpoint=np.array(trial['solver_endpoint']);endpoint[6:]*=Q
                seeds.append(dict(quantized_command_rad=trial['quantized_command_rad'],endpoint=endpoint.tolist()))
        for v in seeds:
            centre=sdk_counts(v['quantized_command_rad']);delta=np.zeros(6);delta[[0,2,3,5]]=np.array(v['endpoint'][6:])
            for d in product((-1,0,1),repeat=6):
                counts=centre+np.array(d);key=tuple(int(i) for i in counts)
                if key in proposed:continue
                command=(counts-2048)*Q;actual=command-offset
                if np.any(actual<low) or np.any(actual>high):continue
                pe,ae=residual(actual,pg);_,a0=residual(actual,ag)
                if pe>.010 or ae>math.radians(5) or a0>math.radians(5):continue
                ap,aa=residual(actual+delta,ag);examined+=1
                proposed[key]=(max(ap/.006,aa/math.radians(5)),command,pe,ae)
        checked=[]
        for _,command,pe,ae in sorted(proposed.values(),key=lambda x:(x[0],tuple(x[1])))[:16]:
            try:
                _,pre=predict_held_axes_endpoint(fk,command,command-offset,sel['fixed_pregrasp_goal'],position_tolerance_m=.010,orientation_tolerance_rad=math.radians(5),allow_already_converged=True)
                _,approach=predict_held_axes_endpoint(fk,pre['predicted_command_positions_rad'],pre['predicted_positions_rad'],sel['fixed_approach_goal'],position_tolerance_m=.006,orientation_tolerance_rad=math.radians(5),allow_already_converged=True)
                checked.append(dict(qualified=True,command_rad=command.tolist(),pregrasp=pre,approach=approach))
            except HeldAxesPredictionError as exc:
                checked.append(dict(qualified=False,command_rad=command.tolist(),code=str(exc),evidence=exc.evidence))
        rows.append(dict(lineage_index=row['lineage_index'],plan_id=row['plan_id'],unique_pregrasp_admissible_count=len(proposed),checked=checked))
        print(json.dumps(dict(lineage_index=row['lineage_index'],admissible=len(proposed),checked=len(checked),qualified=sum(x['qualified'] for x in checked))),flush=True)
    result=dict(kind='bounded_quantized_endpoint_counterfactual',seed_kind=args.seed_kind,source_result=str(args.input),source_sha256=sha(args.input),imported_sources=sources,model_sha256=fk.model_sha256,elapsed_sec=time.monotonic()-started,blocked_events=blocked,rows=rows,limits=['Diagnostic search only: original 10mm/6mm/5degree gates, Joint2/5 held, 32 count correction budget and unchanged +/-0.25 nominal local branch.', 'Up to 3**6 integer neighbours per continuous seed; at most 16 unchanged production predictor checks per candidate.', 'No collision/path/execution/deadline/current feedback proof; failure is not infeasibility proof.'],execution_authorized=False)
    (out/'result.json').write_text(json.dumps(result,indent=2,sort_keys=True,allow_nan=False)+'\n')
    print(json.dumps(dict(result=str(out/'result.json'),sha256=sha(out/'result.json'))),flush=True)


if __name__=='__main__':main()
