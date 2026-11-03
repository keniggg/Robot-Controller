#!/usr/bin/env python3
"""Bounded, fully offline numerical IK against frozen gen28 URDF and targets."""
import argparse
import hashlib
import json
from pathlib import Path
import time
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk

ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/user_grasp_20261008T090751Z/source_snapshot_review')
NAMES=['Joint%d'%i for i in range(1,7)]
rotation_from_matrix=getattr(Rotation,'from_matrix',None) or Rotation.from_dcm


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--indices',type=int,nargs='+',default=[14,8,0])
    parser.add_argument('--seeds',type=int,default=8)
    parser.add_argument('--seconds',type=float,default=120.)
    args=parser.parse_args()
    if not Path('/media/zhuyupei/SanStand').is_mount() or ROOT not in args.output.resolve().parents:
        raise ValueError('output must be on mounted SanStand in review directory')
    data=json.loads((ROOT/'gen28_endpoints_prepared.jsonl').read_text().splitlines()[0])
    xml=(ROOT/'gen28_fk_ik_100ms.model.urdf').read_text()
    if hashlib.sha256(xml.encode()).hexdigest()!=data['robot_description_sha256']:
        raise ValueError('model differs from frozen production model')
    fk=SerialUrdfFk(xml,NAMES);bounds=(fk.limits[:,0]+1e-10,fk.limits[:,1]-1e-10)
    q0=np.array([dict(zip(data['seed']['name'],data['seed']['position']))[n] for n in NAMES])
    rng=np.random.RandomState(28)
    seeds=[q0,np.mean(fk.limits,axis=1)]
    while len(seeds)<args.seeds:
        seeds.append(rng.uniform(bounds[0],bounds[1]))
    started=time.monotonic()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as output:
        def emit(row):
            output.write(json.dumps(row,allow_nan=False)+'\n');output.flush()
        emit(dict(kind='scope',offline=True,execution_authorized=False,
            model_sha256=fk.model_sha256,source_audit_sha256=data['audit_sha256'],
            script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),bounds=fk.limits.tolist(),q0=q0.tolist()))
        for index in args.indices:
            target=np.asarray(data['endpoints'][index]['T_base_tool0'])
            def residual(q):
                if time.monotonic()-started>args.seconds:raise TimeoutError('offline numerical budget reached')
                actual=fk(q)
                return np.r_[5.*(actual[:3,3]-target[:3,3]),rotation_from_matrix(actual[:3,:3]@target[:3,:3].T).as_rotvec()]
            best=None
            for i,seed in enumerate(seeds):
                result=least_squares(residual,seed,bounds=bounds,max_nfev=220,ftol=1e-10,xtol=1e-10,gtol=1e-10)
                actual=fk(result.x);error=residual(result.x)
                row=dict(kind='ik_fit',endpoint_index=index,seed_index=i,q=result.x.tolist(),nfev=result.nfev,
                    cost=float(np.linalg.norm(error)),position_error_m=float(np.linalg.norm(error[:3])/5),
                    rotation_error_rad=float(np.linalg.norm(error[3:])),active_bounds=result.active_mask.tolist(),
                    T_base_tool0=actual.tolist(),within_original_limits=True,scipy_success=bool(result.success),
                    exact_pose_match=bool(np.linalg.norm(error[:3])/5<1e-5 and np.linalg.norm(error[3:])<1e-4))
                emit(row)
                if best is None or row['cost']<best['cost']:best=row
                if row['exact_pose_match']:break
            print(json.dumps(dict(endpoint_index=index,position_error_m=best['position_error_m'],rotation_error_rad=best['rotation_error_rad'],active_bounds=best['active_bounds'],exact_pose_match=best['exact_pose_match'])),flush=True)
            emit(dict(kind='best_endpoint',**{k:v for k,v in best.items() if k!='kind'}))
        emit(dict(kind='complete',elapsed_sec=time.monotonic()-started,execution_authorized=False))


if __name__=='__main__':main()
