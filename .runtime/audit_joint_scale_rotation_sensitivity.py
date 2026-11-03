#!/usr/bin/env python3
"""Offline per-joint/common encoder-scale sensitivity; all zero offsets fixed.

Uses the same original 22 training captures, pair selection and independent
holdouts as the zero-offset audit. q_actual_hypothesis = scale * q_recorded.
Never combines zero-offset and scale fitting; no ROS or deployment interface.
"""
import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares

ROOT=Path(__file__).resolve().parents[1];MOUNT=Path('/media/zhuyupei/SanStand')
spec=importlib.util.spec_from_file_location('zero_audit_for_data_only',ROOT/'.runtime/audit_joint_zero_rotation_sensitivity.py')
Z=importlib.util.module_from_spec(spec);spec.loader.exec_module(Z)


def residual(rows,pair_list,scale):
    gains=np.asarray(scale,dtype=float)
    if gains.shape==(1,):gains=np.repeat(gains[0],6)
    if gains.shape!=(6,) or not np.isfinite(gains).all():raise ValueError('one common or six finite scales required')
    used=set(k for p in pair_list for k in (p['i'],p['j']))
    rotations={i:rows[i]['fk'](rows[i]['q']*gains) for i in used}
    return np.asarray([p['camera_deg']-Z.angle(rotations[p['i']].T@rotations[p['j']]) for p in pair_list])


def solve(rows,pp,common=False):
    result=least_squares(lambda s:residual(rows,pp,s),np.ones(1 if common else 6),
        bounds=(.9,1.1),xtol=1e-11,ftol=1e-11,gtol=1e-10,max_nfev=150,diff_step=1e-5,x_scale='jac')
    return result


def evaluate(rows,pp,scale):
    r=residual(rows,pp,scale)
    return dict(pairs=len(r),rms_deg=float(np.sqrt(np.mean(r*r))) if len(r) else None,
        median_absolute_deg=float(np.median(abs(r))) if len(r) else None,
        p95_absolute_deg=float(np.percentile(abs(r),95)) if len(r) else None,
        maximum_absolute_deg=float(np.max(abs(r))) if len(r) else None,
        pair_rows=[dict(first=rows[p['i']]['capture'],second=rows[p['j']]['capture'],camera_deg=p['camera_deg'],
            nominal_fk_deg=p['nominal_fk_deg'],hypothetical_fk_deg=float(p['camera_deg']-v),
            camera_minus_hypothetical_fk_deg=float(v),camera_minus_nominal_fk_deg=float(p['camera_deg']-p['nominal_fk_deg'])) for p,v in zip(pp,r)])


def jacobian(rows,pp,scale):
    scale=np.asarray(scale);step=1e-5;columns=[]
    for i in range(len(scale)):
        d=np.zeros(len(scale));d[i]=step
        columns.append((residual(rows,pp,scale+d)-residual(rows,pp,scale-d))/(2*step))
    matrix=np.asarray(columns).T;_u,s,vt=np.linalg.svd(matrix,full_matrices=False)
    return dict(singular_values_deg_per_unit_gain=s.tolist(),rank=int(np.sum(s>1e-7)),condition=float(s[0]/s[-1]) if s[-1]>1e-12 else None,
        weakest_parameter_direction=vt[-1].tolist())


def run(rows,sources):
    roles=list(dict.fromkeys(r['role'] for r in rows));indices={role:[i for i,r in enumerate(rows) if r['role']==role] for role in roles}
    pair_sets={role:Z.pairs(rows,ii) for role,ii in indices.items()};train=pair_sets['training']
    if len(indices['training'])!=22:raise ValueError('Exactly original 22 reserved training windows required')
    models={}
    for name,common in [('six_separate_scales',False),('one_common_scale',True)]:
        fit=solve(rows,train,common)
        folds=[]
        for fold in range(4):
            fit_indices=[i for k,i in enumerate(indices['training']) if k%4!=fold]
            held_indices=[i for k,i in enumerate(indices['training']) if k%4==fold]
            tr=Z.pairs(rows,fit_indices);he=Z.pairs(rows,held_indices);fitted=solve(rows,tr,common)
            folds.append(dict(fold=fold,training_captures=len(fit_indices),held_captures=len(held_indices),scale=fitted.x.tolist(),
                active_mask=fitted.active_mask.tolist(),training=evaluate(rows,tr,fitted.x),held=evaluate(rows,he,fitted.x),held_nominal=evaluate(rows,he,np.ones(1 if common else 6))))
        evaluations={role:dict(captures=len(indices[role]),participating_captures=len(set(k for p in pp for k in (p['i'],p['j']))),
            nominal=evaluate(rows,pp,np.ones(1 if common else 6)),diagnostic=evaluate(rows,pp,fit.x)) for role,pp in pair_sets.items()}
        critical=[p for p in evaluations['task_pose_validation']['diagnostic']['pair_rows'] if Path(p['first']).parent.name=='capture_014' and Path(p['second']).parent.name=='capture_020']
        models[name]=dict(scale=fit.x.tolist(),change_percent=((fit.x-1)*100).tolist(),bounds=[.9,1.1],active_mask=fit.active_mask.tolist(),
            solver_success=bool(fit.success),training_jacobian=jacobian(rows,train,fit.x),evaluations=evaluations,critical_14_to_20=critical,grouped_four_fold=folds)
    return dict(schema=1,scope='Diagnostic only; all zero offsets unchanged. No URDF, hand-eye, camera, board, driver or controller update.',
        hypothesis='q_hypothesis[j] = scale[j] * q_recorded[j], fixed zero at recorded zero; no centering or simultaneous offset fit.',
        input_camera='Unchanged original SDK rays, baseline virtual-K PnP; not the camera-intrinsics candidate.',
        pair_selection='Same fixed-board dataset and same epoch, nominal FK relative angle >=2 deg, identical to zero-offset audit.',
        sources=sources,models=models,limitations=['Scale values are hypotheses inferred from camera rotation, not independently measured encoder conversion factors.',
            'Per-joint scales can partially mimic internal orientation changes around the narrow training pose range; current task poses are essential holdout.',
            'Pair rows are correlated through shared capture windows; four folds reserve capture windows, not pairs.',
            'No translations, support height, tool calibration or real grasp success are validated by this angle-only objective.',
            'Camera/board geometry, elastic effects, axis directions and feedback timing remain alternative explanations.',
            'Task singleton/<2-degree groups do not participate and are counted separately.'])


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('camera_audit',type=Path);ap.add_argument('--output',required=True,type=Path);args=ap.parse_args()
    if not os.path.ismount(MOUNT) or MOUNT not in args.output.resolve().parents:raise RuntimeError('Mounted SanStand required')
    rows,sources=Z.load(args.camera_audit);result=run(rows,sources)
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'joint_scale_rotation_sensitivity.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    (args.output/'analysis_source.py').write_bytes(Path(__file__).read_bytes())
    print(json.dumps(dict(output=str(args.output),models={name:dict(scale=m['scale'],jacobian=m['training_jacobian'],critical=m['critical_14_to_20'],evaluations={role:{k:v[k]['rms_deg'] for k in ('nominal','diagnostic')} for role,v in m['evaluations'].items()}) for name,m in result['models'].items()}),ensure_ascii=False))

if __name__=='__main__':main()
