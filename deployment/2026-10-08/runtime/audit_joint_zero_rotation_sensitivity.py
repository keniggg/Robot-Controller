#!/usr/bin/env python3
"""Offline X-independent relative rotation sensitivity to internal joint zeros.

Only yesterday's reserved 22 training captures fit three diagnostic quantities:
J2+J3 sum offset (implemented on J2), J4 offset, J5 offset, each within +/-15 deg.
J1/J6 offsets, hand-eye, URDF, camera K and board geometry are never fitted here.
No ROS imports or commands. Every candidate remains diagnostic, never deployed.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares

ROOT=Path(__file__).resolve().parents[1]
MOUNT=Path('/media/zhuyupei/SanStand')


def rodrigues(axis,angle):
    axis=np.asarray(axis,dtype=float);x,y,z=axis
    k=np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
    return np.eye(3)+math.sin(angle)*k+(1.-math.cos(angle))*(k@k)


def angle(rotation):
    return math.degrees(math.acos(float(np.clip((np.trace(rotation)-1.)/2.,-1.,1.))))


class RotationFk:
    def __init__(self,xml):
        robot=ET.fromstring(xml);parents={j.find('child').get('link'):j for j in robot.findall('joint')}
        chain=[];child='tool0'
        while child!='base_link':j=parents[child];chain.append(j);child=j.find('parent').get('link')
        self.chain=[]
        for j in reversed(chain):
            origin=j.find('origin');rpy=np.fromstring(origin.get('rpy','0 0 0'),sep=' ')
            r=rodrigues([0,0,1],rpy[2])@rodrigues([0,1,0],rpy[1])@rodrigues([1,0,0],rpy[0])
            index=None;axis=None
            if j.get('type')=='revolute':
                index=int(j.get('name').replace('Joint',''))-1;axis=np.fromstring(j.find('axis').get('xyz'),sep=' ');axis/=np.linalg.norm(axis)
            self.chain.append((r,index,axis))

    def __call__(self,q,offset_degrees=(0,0,0)):
        actual=np.array(q,dtype=float,copy=True)
        actual[[1,3,4]]+=np.radians(offset_degrees)
        r=np.eye(3)
        for origin,index,axis in self.chain:
            r=r@origin
            if index is not None:r=r@rodrigues(axis,actual[index])
        return r


def load(directory):
    detected=json.loads((directory/'detected_rays.json').read_text())
    poses=json.loads((directory/'report.json').read_text())['frames'];poseby={r['capture']:r for r in poses}
    rows=[];models={};sources=[]
    for d in detected:
        path=Path(d['capture']);payload=path.read_bytes()
        if hashlib.sha256(payload).hexdigest()!=d['capture_sha256']:raise ValueError('capture hash changed')
        c=json.loads(payload);sample=next(r for r in c['samples'] if r['source_stamp_ns']==d['source_stamp_ns'])
        names=sample['accepted_joint_names'];positions=sample['accepted_joint_positions'];byname=dict(zip(names,positions))
        q=np.array([byname['Joint%d'%j] for j in range(1,7)])
        urdf=Path(d['dataset'])/'robot_description.urdf'
        if not urdf.exists():urdf=Path(d['dataset']).parent/'robot_description.urdf'
        text=urdf.read_text();sha=hashlib.sha256(text.encode()).hexdigest()
        if sha not in models:models[sha]=RotationFk(text);sources.append(dict(path=str(urdf),sha256=sha))
        camera=np.asarray(poseby[d['capture']]['baseline_T_camera_board'])[:3,:3]
        fk=models[sha];rotation=fk(q)
        rows.append(dict(capture=d['capture'],role=d['role'],dataset=d['dataset'],epoch=d['epoch'],
            source_stamp_ns=d['source_stamp_ns'],urdf_sha256=sha,q=q,camera=camera,fk=fk,
            fk_vs_archived_tool_rotation_deg=angle(rotation.T@np.asarray(d['T_base_tool'])[:3,:3]),
            baseline_virtual_pixel_rms=poseby[d['capture']]['baseline_virtual_rms_px']))
    return rows,sources


def pairs(rows,indices):
    output=[]
    for pos,i in enumerate(indices):
        first=rows[i]
        for j in indices[pos+1:]:
            second=rows[j]
            if (first['dataset'],first['epoch'])!=(second['dataset'],second['epoch']):continue
            if first['urdf_sha256']!=second['urdf_sha256']:raise ValueError('model changes within pair')
            nominal=angle(first['fk'](first['q']).T@second['fk'](second['q']))
            if nominal<2.:continue
            camera=angle(first['camera'].T@second['camera'])
            output.append(dict(i=i,j=j,nominal_fk_deg=nominal,camera_deg=camera))
    return output


def residual(rows,pair_list,offset):
    used=set(k for p in pair_list for k in (p['i'],p['j']))
    rotations={i:rows[i]['fk'](rows[i]['q'],offset) for i in used}
    return np.asarray([p['camera_deg']-angle(rotations[p['i']].T@rotations[p['j']]) for p in pair_list])


def evaluate(rows,pair_list,offset):
    r=residual(rows,pair_list,offset)
    return dict(pairs=len(r),rms_deg=float(np.sqrt(np.mean(r*r))) if len(r) else None,
        median_absolute_deg=float(np.median(abs(r))) if len(r) else None,
        p95_absolute_deg=float(np.percentile(abs(r),95)) if len(r) else None,
        maximum_absolute_deg=float(np.max(abs(r))) if len(r) else None,
        mean_signed_deg=float(np.mean(r)) if len(r) else None,
        pair_rows=[dict(first=rows[p['i']]['capture'],second=rows[p['j']]['capture'],nominal_fk_deg=p['nominal_fk_deg'],
            camera_deg=p['camera_deg'],diagnostic_fk_deg=float(p['camera_deg']-v),camera_minus_diagnostic_fk_deg=float(v),
            camera_minus_nominal_fk_deg=float(p['camera_deg']-p['nominal_fk_deg'])) for p,v in zip(pair_list,r)])


def solve(rows,pair_list,start=(0,0,0),loss='linear'):
    result=least_squares(lambda x:residual(rows,pair_list,x),np.asarray(start,dtype=float),
        bounds=(-15.,15.),loss=loss,f_scale=.05,xtol=1e-11,ftol=1e-11,gtol=1e-10,max_nfev=150,
        diff_step=1e-4)
    return result


def jacobian(rows,pair_list,offset):
    step=.001;columns=[]
    for j in range(3):
        d=np.zeros(3);d[j]=step
        columns.append((residual(rows,pair_list,np.asarray(offset)+d)-residual(rows,pair_list,np.asarray(offset)-d))/(2*step))
    matrix=np.asarray(columns).T
    _u,s,vt=np.linalg.svd(matrix,full_matrices=False)
    return dict(matrix=matrix,singular=s,rank=int(np.sum(s>1e-8)),condition=float(s[0]/s[-1]) if s[-1]>1e-12 else None,weakest=vt[-1])


def run(rows,sources):
    roles=list(dict.fromkeys(r['role'] for r in rows))
    indices={role:[i for i,r in enumerate(rows) if r['role']==role] for role in roles}
    pp={role:pairs(rows,index) for role,index in indices.items()}
    train=pp['training'];starts=[[0,0,0],[10,0,0],[-10,0,0],[0,10,0],[0,-10,0],[0,0,10],[0,0,-10]]
    solutions=[solve(rows,train,start) for start in starts]
    selected=min(solutions,key=lambda r:np.sum(r.fun*r.fun));offset=selected.x
    robust=solve(rows,train,loss='soft_l1')
    sensitivity=jacobian(rows,train,offset)
    folds=[];training_indices=indices['training']
    for fold in range(4):
        fit_indices=[i for k,i in enumerate(training_indices) if k%4!=fold]
        held_indices=[i for k,i in enumerate(training_indices) if k%4==fold]
        fit_pairs=pairs(rows,fit_indices);held_pairs=pairs(rows,held_indices)
        fitted=solve(rows,fit_pairs)
        folds.append(dict(fold=fold,training_captures=len(fit_indices),held_captures=len(held_indices),offset_deg=fitted.x.tolist(),
            training=evaluate(rows,fit_pairs,fitted.x),held=evaluate(rows,held_pairs,fitted.x),held_nominal=evaluate(rows,held_pairs,[0,0,0])))
    # Leave one CAPTURE out, never pretend pair combinations are independent.
    jackknife=[]
    for excluded in training_indices:
        sub=pairs(rows,[i for i in training_indices if i!=excluded]);fitted=solve(rows,sub,start=offset)
        jackknife.append(dict(excluded=rows[excluded]['capture'],offset_deg=fitted.x.tolist(),training_rms_deg=float(np.sqrt(np.mean(fitted.fun**2)))))
    evaluations={role:dict(captures=len(indices[role]),participating_captures=len(set(k for p in pp[role] for k in (p['i'],p['j']))),
        nominal=evaluate(rows,pp[role],[0,0,0]),diagnostic=evaluate(rows,pp[role],offset),robust_diagnostic=evaluate(rows,pp[role],robust.x)) for role in roles}
    critical=[]
    for p in evaluations['task_pose_validation']['diagnostic']['pair_rows']:
        if Path(p['first']).parent.name=='capture_014' and Path(p['second']).parent.name=='capture_020':critical.append(p)
    return dict(schema=1,scope='Offline sensitivity only; no runtime, URDF, hand-eye, K, encoder, control or board changes.',
        fitted_parameters=['J2_plus_J3_zero_sum_deg_implemented_on_J2','J4_zero_deg','J5_zero_deg'],
        parameter_bounds_deg=[-15,15],offset_deg=offset.tolist(),training_objective='unweighted squared camera-minus-FK relative rotation angles; nominal FK pair angle>=2 degrees; within same fixed-board dataset and driver epoch only',
        camera_model='Unchanged original SDK rays; baseline virtual-K PnP from detected_rays/report cache, not fitted camera K.',
        sources=sources,capture_q=[{k:r[k] for k in ('capture','role','dataset','epoch','source_stamp_ns','urdf_sha256','q','fk_vs_archived_tool_rotation_deg','baseline_virtual_pixel_rms')} for r in rows],
        multi_start=[dict(start=start,offset_deg=r.x.tolist(),rms_deg=float(np.sqrt(np.mean(r.fun*r.fun))),active_mask=r.active_mask.tolist(),success=bool(r.success)) for start,r in zip(starts,solutions)],
        training_jacobian=dict(singular_values_deg_per_deg=sensitivity['singular'].tolist(),rank=sensitivity['rank'],condition=sensitivity['condition'],weakest_parameter_direction=sensitivity['weakest'].tolist()),
        robust_fit=dict(loss='soft_l1',scale_deg=.05,offset_deg=robust.x.tolist()),
        capture_leaveout=jackknife,grouped_four_fold=folds,evaluations=evaluations,critical_14_to_20=critical,
        limitations=['J2 and J3 parallel axes identify only their zero SUM for orientation; the fitted sum cannot assign an individual encoder zero.',
            'J1 zero is a global base rotation; J6 zero is a terminal rotation absorbed by hand-eye. Their offsets cannot be identified by relative-angle invariants and are not fitted.',
            'Relative-angle residuals are independent of fixed X, but shared camera/PnP/board-deformation/systematic errors may mimic kinematic changes.',
            'Pair rows share capture frames and are correlated; pair count is not independent sample size. Capture-level folds and leaveout are sensitivity diagnostics, not absolute confidence intervals.',
            'A zero-offset-only model does not exclude encoder scale, axis skew, elastic deflection, time pairing or camera geometry alternatives.',
            'Task-pose groups with a singleton or <2-degree pair contribute no relative-angle validation; participating captures are separately reported.',
            'Even improved angle holdouts do not validate tool position, grasp height or permit model deployment.'])


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('camera_audit',type=Path);ap.add_argument('--output',required=True,type=Path);args=ap.parse_args()
    if not os.path.ismount(MOUNT) or MOUNT not in args.output.resolve().parents:raise RuntimeError('Mounted SanStand output required')
    rows,sources=load(args.camera_audit)
    if sum(r['role']=='training' for r in rows)!=22:raise ValueError('Expected exactly reserved 22 training captures')
    result=run(rows,sources)
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'joint_zero_rotation_sensitivity.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,default=lambda v:v.tolist(),allow_nan=False)+'\n')
    (args.output/'analysis_source.py').write_bytes(Path(__file__).read_bytes())
    print(json.dumps(dict(output=str(args.output),offset_deg=result['offset_deg'],jacobian=result['training_jacobian'],critical=result['critical_14_to_20'],evaluations={role:{k:{metric:v[k][metric] for metric in ('pairs','rms_deg','median_absolute_deg','p95_absolute_deg','maximum_absolute_deg')} for k in ('nominal','diagnostic','robust_diagnostic')} for role,v in result['evaluations'].items()},robust=result['robust_fit']),ensure_ascii=False))

if __name__=='__main__':main()
