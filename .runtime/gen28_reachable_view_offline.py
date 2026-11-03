#!/usr/bin/env python3
"""Find diagnostic views in actual joint space, with original endpoint gates.

No ROS I/O, execution, relaxed joint limits, or production modification.
Numerical proposals still require real MoveIt whole-arm/path/following gates.
"""
from pathlib import Path
import argparse
import hashlib
import json
import time
from types import SimpleNamespace
from dataclasses import asdict
import numpy as np
from scipy.optimize import minimize
import rosbag
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.robot.observation_path_guard import observation_endpoint_support_bound
from alicia_flexible_grasp.robot.observation_view_guard import validate_observation_support_projection
from alicia_flexible_grasp.grasp.observation_working_volume import (
    observation_obb_corners,validate_planned_observation_working_volume)
from alicia_flexible_grasp.grasp.gripper_geometry import (
    GripperGeometry,evaluate_open_gripper_observation_envelope,_stage_boxes,_box_corners)
from alicia_grasp_modes.observe_return_visual import missing_side_view_gain

ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/user_grasp_20261008T090751Z/source_snapshot_review')
BAG=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261008T092818.564023Z_decf934e904a4ca1916b775e621829ce/keyframes.bag')
NAMES=['Joint%d'%i for i in range(1,7)]


def numpy_minimize(q0,seed,constraints,bounds,maxiter=70):
    def residual(q):return np.r_[.01*(q-q0),100.*np.minimum(constraints(q),0.)]
    q=np.clip(seed,bounds[:,0],bounds[:,1]);damping=1e-4;iterations=0
    for iterations in range(maxiter):
        r=residual(q);cost=float(r@r);jac=np.zeros((len(r),len(q)))
        for j in range(len(q)):
            trial=q.copy();sign=1. if q[j]+1e-5<bounds[j,1] else -1.;trial[j]+=sign*1e-5
            jac[:,j]=(residual(trial)-r)/(sign*1e-5)
        delta=-np.linalg.solve(jac.T@jac+damping*np.eye(len(q)),jac.T@r)
        delta*=min(1.,.25/max(1e-12,float(np.max(np.abs(delta)))))
        accepted=False
        for factor in [1.,.5,.25,.125,.0625,.03125]:
            trial=np.clip(q+factor*delta,bounds[:,0],bounds[:,1]);rr=residual(trial)
            if float(rr@rr)<cost:
                q=trial;accepted=True;damping=max(1e-8,damping*.5);break
        if not accepted:damping=min(1e3,damping*10.)
        if np.min(constraints(q))>=-1e-7:break
    return SimpleNamespace(x=q,success=bool(np.min(constraints(q))>=-1e-7),message='numpy bounded penalty iterations=%d'%iterations)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=90.)
    parser.add_argument('--numpy',action='store_true');parser.add_argument('--following',action='store_true')
    parser.add_argument('--contract-result',type=Path);parser.add_argument('--image-reserve',type=float,default=.1);args=parser.parse_args()
    if not Path('/media/zhuyupei/SanStand').is_mount() or ROOT not in args.output.resolve().parents:
        raise ValueError('output must be mounted SanStand review directory')
    audit_bytes=(ROOT/'gen28_observation_frozen_audit.json').read_bytes();audit=json.loads(audit_bytes)
    data=json.loads((ROOT/'gen28_endpoints_prepared.jsonl').read_text().splitlines()[0])
    if hashlib.sha256(audit_bytes).hexdigest()!=data['audit_sha256']:raise ValueError('audit mismatch')
    fk=SerialUrdfFk((ROOT/'gen28_fk_ik_100ms.model.urdf').read_text(),NAMES)
    if fk.model_sha256!=data['robot_description_sha256']:raise ValueError('model mismatch')
    q0=np.array([dict(zip(data['seed']['name'],data['seed']['position']))[n] for n in NAMES]);t0=fk(q0)
    handeye=np.array(data['T_tool0_camera_link']);geometry=audit['replay_geometry'];normal=np.array(geometry['support_normal_base'])
    offset=geometry['support_offset_m'];center=np.array(geometry['obb_center_base_m'])
    corners=observation_obb_corners(center,np.array(geometry['R_base_obb']),np.array(geometry['obb_size_xyz_m']))
    gripper=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003)
    with rosbag.Bag(str(BAG)) as bag:
        for _,message,_ in bag.read_messages(topics=['/grasp_record/camera_runtime_metadata']):
            metadata=json.loads(message.data); camera=metadata['parameters'];break
    k=[camera[n] for n in ['fx','fy','cx','cy']];width,height=653,489
    # The configured frame dimensions are independently frozen in the production audit.
    working=audit['stable_evaluations'][3]['observation_view']['working_volume']
    if (working['image_width'],working['image_height'])!=(width,height):raise ValueError('image geometry mismatch')
    fx,fy,cx,cy=k;gaps=audit['observe_return']['measured_missing_sides']
    uncertainty=audit['stable_evaluations'][3]['observation_view']['observe_return_missing_side_gain']['depth_uncertainty_m']
    started=time.monotonic();results=[]
    errors=np.full(6,.12+np.pi/4096.)
    contract_evidence=None
    if args.contract_result:
        raw=args.contract_result.read_bytes();contract=json.loads(raw)
        proof=contract['goal_support']
        if proof['robot_description_sha256']!=fk.model_sha256 or proof['joint_names']!=NAMES:raise ValueError('contract model/names mismatch')
        errors=np.array(proof['joint_error_bounds_rad'])
        contract_evidence=dict(path=str(args.contract_result),sha256=hashlib.sha256(raw).hexdigest(),joint_error_bounds_rad=errors.tolist())
    source_root=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/supplement_grasp_20261008T103117Z/frozen_view_source')
    source=json.loads((source_root/'source.json').read_text());points=np.load(str(source_root/'source.npz'))['points_base']
    with args.output.open('x') as output:
        def emit(row):output.write(json.dumps(row,allow_nan=False)+'\n');output.flush()
        emit(dict(kind='source',model_sha256=fk.model_sha256,audit_sha256=data['audit_sha256'],intrinsics=k,
            camera_metadata_association=metadata['image_time_association'],joint_limits=fk.limits.tolist(),
            script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),contract_evidence=contract_evidence,offline=True,execution_authorized=False))
        for gap_index,gap in enumerate(gaps):
            gc=np.array(gap['target_center_base_m']);jaw=np.array(gap['jaw_axis_base']);sign=gap['missing_side_signs'][0]
            initial=np.array(gap['initial_camera_origin_base_m'])-gc
            previous=gap['object_height_m']*max(0.,sign*np.dot(jaw,initial)/np.linalg.norm(initial))
            required=(previous+uncertainty+.0002)/gap['object_height_m']
            cached={}
            def constraints(q):
                if time.monotonic()-started>args.seconds:raise TimeoutError('bounded offline view search elapsed')
                # SLSQP probes finite-difference points just outside its box.
                # Evaluate the projection into the original physical box;
                # every final proposal is separately recomputed and checked.
                q=np.clip(q,fk.limits[:,0],fk.limits[:,1])
                key=q.tobytes()
                if key in cached:return cached[key]
                tool=fk(q);cam=tool@handeye;pc=(corners-cam[:3,3])@cam[:3,:3];x,y,z=pc.T
                delta=cam[:3,3]-gc
                gripper_corners=np.concatenate([_box_corners(c,tool[:3,:3],size) for _,c,size in _stage_boxes(tool,gripper,.05,'y','z')])
                margin=36.+args.image_reserve
                values=np.r_[x-.0701,.4999-x,((cx-margin)*x-fx*y)/fx,
                    ((width-1.-margin-cx)*x+fx*y)/fx,((cy-margin)*x-fy*z)/fy,
                    ((height-1.-margin-cy)*x+fy*z)/fy,
                    .1*(sign*np.dot(jaw,delta)/np.linalg.norm(delta)-required),
                    sign*np.dot(jaw,delta)-.5*gap['whole_obb_width_m']-.0001,
                    gripper_corners@normal+offset-.0031,
                    np.linalg.norm(tool[:3,3]-t0[:3,3])-.0151,
                    -.01*np.dot(cam[:3,0],normal)]
                if args.following:
                    bound=observation_endpoint_support_bound(fk,q,errors,normal,offset,gripper,.05)
                    values=np.r_[values,bound['minimum_support_clearance_lower_bound_m']-.0031]
                cached[key]=values
                return values
            seeds=[q0]
            prior=ROOT/'gen28_offline_ik_nearest_8seeds_v2.jsonl'
            if prior.exists():seeds += [np.array(r['q']) for r in map(json.loads,prior.read_text().splitlines()) if r['kind']=='best_endpoint'][:2]
            for seed_index,seed in enumerate(seeds):
                result=(numpy_minimize(q0,seed,constraints,fk.limits) if args.numpy else
                    minimize(lambda q:float(np.sum((q-q0)**2)),seed,method='SLSQP',bounds=fk.limits.tolist(),
                    constraints=[dict(type='ineq',fun=constraints)],options=dict(maxiter=160,ftol=1e-10)))
                q=np.clip(result.x,fk.limits[:,0],fk.limits[:,1]);tool=fk(q);cam=tool@handeye
                row=dict(kind='proposal',gap_index=gap_index,seed_index=seed_index,q=q.tolist(),T_base_tool0=tool.tolist(),
                    T_base_camera_link=cam.tolist(),joint_max_delta_rad=float(np.max(np.abs(q-q0))),
                    tool_translation_delta_m=float(np.linalg.norm(tool[:3,3]-t0[:3,3])),
                    solver_success=bool(result.success),solver_message=str(result.message),
                    minimum_optimization_constraint=float(np.min(constraints(q))),within_original_joint_limits=True)
                row['missing_side_view_gain']=missing_side_view_gain(gaps,cam[:3,3],uncertainty)
                try:row['working_volume']=validate_planned_observation_working_volume(corners,cam,k,width,height,
                    snapshot_stamp_ns=data['source_stamp_ns'])
                except Exception as exc:row['working_volume']=dict(ok=False,reason=str(exc))
                row['observation_envelope']=asdict(evaluate_open_gripper_observation_envelope(gripper=gripper,T_base_tool0=tool,
                    opening_width_m=.05,support_normal_base=normal,support_offset_m=offset,obb_center_base=center,
                    R_base_obb=np.array(geometry['R_base_obb']),obb_size_xyz_m=np.array(geometry['obb_size_xyz_m'])))
                if args.following:row['conditional_following_endpoint_support']=observation_endpoint_support_bound(fk,q,errors,normal,offset,gripper,.05)
                try:row['measured_support_projection']=validate_observation_support_projection(cam,points,source['center_base'],k,width,height,target_corners_base=corners,**source['view_limits']['effective_visibility_limits'])
                except Exception as exc:row['measured_support_projection']=dict(ok=False,reason=str(exc),audit=getattr(exc,'audit',{}))
                row['original_endpoint_gates_pass']=bool(row['missing_side_view_gain']['ok'] and row['working_volume']['ok']
                    and row['observation_envelope']['ok'] and row['tool_translation_delta_m']>=.015
                    and (not args.following or row['conditional_following_endpoint_support']['ok'])
                    and row['measured_support_projection']['ok'])
                row.update(whole_arm_collision_checked=False,path_visibility_following_stop_checked=False,motion_authorized=False)
                emit(row);results.append(row)
                print(json.dumps({k:row[k] for k in ['gap_index','seed_index','original_endpoint_gates_pass','joint_max_delta_rad','minimum_optimization_constraint']}),flush=True)
                if row['original_endpoint_gates_pass']:break
        passing=sorted([r for r in results if r['original_endpoint_gates_pass']],key=lambda r:r['joint_max_delta_rad'])
        emit(dict(kind='summary',passing_count=len(passing),selected=passing[0] if passing else None,
            elapsed_sec=time.monotonic()-started,execution_authorized=False))


if __name__=='__main__':main()
