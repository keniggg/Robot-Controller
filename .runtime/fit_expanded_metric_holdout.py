#!/usr/bin/env python3
"""One predeclared 45-frame expansion; preserve original model, bounds and loss.

Original task 16/2/5 groups become TRAINING. Eight independent frames, failed
08:42 grasp planes, and later J1/centering observations remain strictly held out.
No ROS, model deployment, calibration writes, or fit/weight selection.
"""
import argparse
from collections import Counter,defaultdict
import hashlib,json,math,os
from pathlib import Path
import numpy as np
import audit_joint_metric_identifiability as M
import evaluate_frozen_metric_failed_grasp as F


def avg(T):
    T=np.asarray(T);out=np.eye(4);out[:3,3]=T[:,:3,3].mean(axis=0)
    out[:3,:3]=M.matrix(M.rotation_mean(M.from_matrix(T[:,:3,:3])))
    return out


def inv(T):
    out=np.eye(4);out[:3,:3]=T[:3,:3].T;out[:3,3]=-out[:3,:3]@T[:3,3];return out


def log(R):return M.from_matrix(R).as_rotvec()


def modified_X(X,z):
    XX=X.copy();XX[:3,3]+=np.asarray(z[:3])*.001
    XX[:3,:3]=M.matrix(M.Rotation.from_rotvec(np.radians(z[3:6])))@X[:3,:3]
    return XX


def local_metrics(rows,fk,X,z):
    offsets=np.r_[0.,np.radians(z[6:10]),0.];XX=modified_X(X,z);groups=defaultdict(list)
    for r in rows:
        A=fk(np.asarray(r['accepted_positions_rad'])[:6]+offsets);C=np.asarray(r['T_camera_board']);Y=A@XX@C
        groups[r['offline_stationary_phase']].append((r,A,C,Y))
    order=sorted(groups,key=lambda k:min(r[0]['source_stamp_ns'] for r in groups[k]));means={k:[avg([v[i] for v in values]) for i in (1,2,3)] for k,values in groups.items()}
    output={}
    for i in range(1,len(order)):
        for j in sorted(set([0,i-1])):
            A0,C0,Y0=means[order[j]];A1,C1,Y1=means[order[i]];Ro=C1[:3,:3].T@C0[:3,:3];Rp=Y0[:3,:3].T@A1[:3,:3]@A0[:3,:3].T@Y0[:3,:3]
            vo,vp=log(Ro),log(Rp);center=np.array([.09,.0675,0.]);d=Y1[:3,:3]@center+Y1[:3,3]-Y0[:3,:3]@center-Y0[:3,3]
            observed=inv(C1)[:3,3]-inv(C0)[:3,3];predicted=(inv(Y0)@A1@XX)[:3,3]-(inv(Y0)@A0@XX)[:3,3]
            theta=lambda a,b:float(np.degrees(np.arccos(np.clip(abs(np.dot(a,b)/(np.linalg.norm(a)*np.linalg.norm(b))),-1,1))))
            output[order[i]+'__minus__'+order[j]]=dict(board_center_drift_mm=float(np.linalg.norm(d)*1000),board_center_drift_xyz_mm=(d*1000).tolist(),
                relative_rotation_residual_deg=float(np.degrees(np.linalg.norm(log(Rp.T@Ro)))),camera_travel_vector_error_mm=float(np.linalg.norm(predicted-observed)*1000),
                observed_to_model_axis_deg=theta(vo,vp) if min(np.linalg.norm(vo),np.linalg.norm(vp))>math.radians(.2) else None)
    return dict(windows={k:dict(count=len(v),source_stamp_ns=[r[0]['source_stamp_ns'] for r in v]) for k,v in groups.items()},comparisons=output)


def failed_metrics(g,fk,X,z):
    XX=modified_X(X,z);offset=np.r_[0.,np.radians(z[6:10]),0.];T={};old={}
    for r in g['frames']:
        q=np.asarray(r['q_rad']);old[r['label']]=np.asarray(r['T_base_optical'])
        np.testing.assert_allclose(fk(q)@X@F.O,old[r['label']],atol=1e-10,rtol=0)
        T[r['label']]=fk(q+offset)@XX@F.O
    D0=T['initial']@inv(old['initial']);result=[]
    for row in g['common_patch_comparisons']:
        label=row['stage'];first=row['fits']['initial'];last=row['fits'][label]
        p0,_=F.transport(F.plane(first),old['initial'],T['initial']);p1,_=F.transport(F.plane(last),old[label],T[label])
        anchor=np.r_[row['center_xy_base_m'],first['z_at_common_center_mm']*.001,1.];anchor=(D0@anchor)[:3]
        result.append(dict(stage=label,radius_m=row['radius_m'],**F.compare(p0,p1,anchor)))
    return result


def main():
    p=argparse.ArgumentParser();p.add_argument('resume',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
    if not os.path.ismount(F.MOUNT) or F.MOUNT not in a.output.resolve().parents:raise ValueError('SanStand mount/output required')
    a.output.mkdir(parents=True,exist_ok=True)
    if (a.output/'fit_result.json').exists():raise ValueError('single declared fit already exists; no silent rerun')
    rows,sources=M.load(a.resume/'camera_intrinsics_sensitivity');rank=json.loads((a.resume/'joint_metric_identifiability/rank_report.json').read_text())
    X,xsha=M.handeye(Path('/home/zhuyupei/.ros/easy_handeye/d405_cc200_auto_20260927_validated_eye_on_hand.yaml'))
    if xsha!=rank['handeye_sha256']:raise ValueError('baseline handeye hash mismatch')
    counts=Counter(r['group'] for r in rows if r['role']=='task_pose_validation');train=[];independent=[];excluded=[];bindings=[]
    for r in rows:
        c=json.loads(Path(r['capture']).read_text());bindings.append(c['model_fingerprint'])
        np.testing.assert_allclose(r['fk'](r['q']),r['A'],atol=1e-9,rtol=0)
        if r['role']=='training' or (r['role']=='task_pose_validation' and counts[r['group']]>1):
            train.append(dict(r,role='training',original_role=r['role']))
        elif r['role']=='independent_validation':independent.append(r)
        else:excluded.append(r['capture'])
    if len(train)!=45 or len(independent)!=8 or len(excluded)!=2:raise ValueError('declared 45/8/2 split mismatch')
    urdfs={s['urdf_sha256'] for s in sources}
    if len(urdfs)!=1 or set(bindings)!={rank['sources'] and json.loads(Path(sources[0]['capture']).read_text())['model_fingerprint']}:raise ValueError('input model incompatibility')
    sha=next(iter(urdfs));camera=bindings[0]
    declaration=dict(diagnostic_only=True,model='X6DOF + q2/q3/q4/q5 zeros; q1/q6 fixed',orientation_residual_unit_deg=.1,position_residual_unit_mm=1.,
        bounds_abs=[10.,10.,10.,3.,3.,3.,10.,10.,10.,10.],start=[0.]*10,loss='linear',maximum_function_evaluations=150,
        training_count=45,original_training_count=22,former_task_multi_frame_training_count=23,task_groups_count=[16,2,5],
        task_validation_reclassified_as_training=True,independent_validation_count=8,task_singletons_excluded=excluded,
        allowed_heldouts=['original independent8','084245 failed grasp exact same fitted camera planes','140754 J1','141708 J2/J3 centering'],
        forbidden_training=['independent8','084245 grasp','current epoch J1/J2/J3'],
        nuisance_board_poses='independent centered mean per exact dataset/epoch; no cross-group absolute board coordinates',
        source_sha256=F.sha(__file__),dependency_sha256=F.sha(M.__file__),handeye_sha256=xsha,urdf_sha256=sha,camera_model_fingerprint=camera,
        training_captures=[r['capture'] for r in train],heldout_independent_captures=[r['capture'] for r in independent],all_original_sources=sources)
    (a.output/'split_declaration.json').write_text(json.dumps(declaration,ensure_ascii=False,indent=2)+'\n')
    # Identifiability is assessed only on this declared training set.
    fun=lambda z:M.centered_residual(train,X,z,.1);J=M.central_jacobian(fun,np.zeros(10),.001);s=np.linalg.svd(J,compute_uv=False);threshold=max(s[0]*1e-7,1e-6)
    identification=dict(rank=int(np.sum(s>threshold)),condition=float(s[0]/s[-1]),singular_values=s.tolist())
    if identification['rank']!=10 or identification['condition']>=1e6:raise ValueError('expanded-training rank deficient; no fitting')
    fitted=M.fit_training(train,X,.1)
    record=dict(declaration_file='split_declaration.json',identifiability=identification,fit=fitted,
        training_baseline=M.metrics(train,X,np.zeros(10)),training_fitted=M.metrics(train,X,np.asarray(fitted['z'])),
        training_joint_span_deg=np.degrees(np.ptp([r['q'] for r in train],axis=0)).tolist())
    (a.output/'fit_result.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(stage='fit_completed_once',identifiability=identification,fit=fitted),ensure_ascii=False),flush=True)
    zz=np.asarray(fitted['z']);oldfit=json.loads((a.resume/'joint_metric_identifiability/fit_report.json').read_text());oldz=np.asarray(next(f for f in oldfit['fits'] if f['orientation_unit_deg']==.1)['z'])
    models={'runtime':np.zeros(10),'original_22_train':oldz,'expanded_45_train':zz};arch=a.resume.parent;report=dict(diagnostic_only=True,runtime_changed=False,refit_heldouts=False,
        original_task_groups_are_now_training=True,models={},inputs={})
    gs=arch/'grasp_diagnostic_20260928T084245Z';gp=gs/'geometry_forensics/geometry_report.json';g=json.loads(gp.read_text());gfk=M.MetricFk((gs/'robot_description.urdf').read_text())
    if gfk.sha!=sha or g['camera_fingerprint']!=camera:raise ValueError('grasp holdout model mismatch')
    report['inputs']['failed_grasp']=dict(path=str(gp),sha256=F.sha(gp))
    configs=[('J1',arch/'bounded_motor_centering_20260928T140754Z_327901',a.resume/'j1_board_axis_140754/selected_geometries.json'),
        ('centering_J2_J3',arch/'bounded_motor_centering_20260928T141708Z_351496',a.resume/'frozen_metric_centering_141708/selected_geometries.json')]
    local={}
    for label,session,path in configs:
        rr=list(json.loads(path.read_text()).values());fk=M.MetricFk((session/'robot_description.urdf').read_text())
        if fk.sha!=sha:raise ValueError('local heldout URDF mismatch')
        for r in rr:
            np.testing.assert_allclose(r['T_tool_camera'],X,atol=1e-12,rtol=0)
            if r['quality']['camera_model_fingerprint']!=camera:raise ValueError('local camera fingerprint mismatch')
        local[label]=(rr,fk);report['inputs'][label]=dict(path=str(path),sha256=F.sha(path),count=len(rr))
    for label,z in models.items():
        report['models'][label]=dict(independent8=M.metrics(independent,X,z),failed_grasp=failed_metrics(g,gfk,X,z),
            local_heldouts={key:local_metrics(rr,fk,X,z) for key,(rr,fk) in local.items()})
    report['limits']=['Only one predeclared fitting run from zero, same 10 parameters/bounds/weight; no alternative fits selected.',
        'Former task multi-frame groups are now training and cannot be cited as independent validation.',
        'Same URDF, camera fingerprint, baseline X and numerically identical archived FK verified; different reference epochs retain independent fixed-board origins.',
        'No mechanical reassembly metadata guarantees are inferred solely from matching hashes.',
        'J1/centering means summarize correlated images. Failed-grasp patches are unchanged archived point fits, not tracked material correspondences.',
        'No handeye, URDF, camera intrinsics, scale, robot status or commands were modified.']
    (a.output/'heldout_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');(a.output/Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    compact={}
    for label,v in report['models'].items():
        compact[label]=dict(independent8_rms_mm=v['independent8'][0]['position_centered_rms_mm'],independent8_max_pair_mm=v['independent8'][0]['position_pairwise_max_mm'],
            grasp_before_close_z_mm=[r['later_minus_initial_z_mm'] for r in v['failed_grasp'] if r['stage']=='before_close'],
            local={key:m['comparisons'] for key,m in v['local_heldouts'].items()})
    print(json.dumps(dict(stage='heldouts_evaluated',summary=compact),ensure_ascii=False),flush=True)

if __name__=='__main__':main()
