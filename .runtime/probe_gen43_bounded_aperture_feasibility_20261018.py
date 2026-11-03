#!/usr/bin/env python3
"""Same original episode box, frozen plan and constant-bias prediction only."""
import base64
import hashlib
import json
import math
from pathlib import Path
import socket
import sys
import time

W = Path(__file__).resolve().parent.parent
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N = E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
INPUT = N/'verification/gen43_actual_aperture_failure_r1'
OUT = N/'verification/gen43_original_budget_aperture_feasibility_r4'
URDF = W/'.runtime/gen42_source_before_gen43/deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf'
sys.path[:0] = [str(W/'src/alicia_flexible_grasp_supervisor/src'), str(W/'devel/lib/python3/dist-packages')]
boundary = []
class OfflineSocket(socket.socket):
    def __new__(cls, *args, **kwargs):
        boundary.append('socket')
        raise RuntimeError('Offline feasibility has no network boundary')
socket.socket = OfflineSocket

import numpy as np
from scipy.optimize import minimize
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan
from alicia_flexible_grasp.grasp.contact_execution import plan_aperture_evaluator
from alicia_flexible_grasp.grasp.contact_execution import frozen_contact_geometry
from alicia_flexible_grasp.grasp.gripper_geometry import _box_corners, _stage_boxes
from alicia_flexible_grasp.grasp.rich_plan_integrity import plan_id_matches_content, pose_values
from alicia_flexible_grasp.robot.held_axes_prediction import predict_held_axes_endpoint
from alicia_flexible_grasp.robot.pregrasp_compensation import pose_matrix, remaining_budget_guide
from alicia_flexible_grasp.robot.stationary_following import SerialUrdfFk, ARM_NAMES, SDK_QUANTUM_RAD as Q

def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def put(name, x):
    p=OUT/name
    p.write_text(json.dumps(x,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    return dict(path=str(p),sha256=sha(p))

def main():
    assert E.is_dir()
    manifest=N/'verification/gen43_release_manifest.json'
    source_pins=json.loads(manifest.read_text())['source_sha256']
    assert len(source_pins)==1036
    assert all(sha(W/p)==h for p,h in source_pins.items())
    c=json.loads((INPUT/'runtime_compensation_report.json').read_text())
    r=json.loads((INPUT/'runtime_aperture_report.json').read_text())
    wire=json.loads((INPUT/'frozen_plan.json').read_text())
    raw=base64.b64decode(wire['wire_base64'])
    assert hashlib.sha256(raw).hexdigest()==wire['wire_sha256']
    plan=Grasp6DPlan(); plan.deserialize(raw)
    assert plan_id_matches_content(plan) and plan.plan_id==c['plan_id']==r['plan_id']
    assert sha(URDF)==c['model_sha256']==r['model_sha256']
    fk=SerialUrdfFk(URDF.read_text(),ARM_NAMES)
    assert fk.model_sha256==c['model_sha256']
    goal=pose_matrix(pose_values(plan.poses[0]))
    sdk=np.array(c['command_start_positions_rad'],dtype=float)
    initial=np.array(c['measured_start_positions_rad'],dtype=float)
    counts=np.array(c['initial_sdk_counts'],dtype=int)
    assert np.allclose(sdk,(counts-2048)*Q,atol=1e-12,rtol=0.)
    offset=sdk-initial
    evaluate=plan_aperture_evaluator(plan,{},r['measured_opening_width_m'],0)
    frozen=frozen_contact_geometry(plan,{},r['measured_opening_width_m'])
    cache={}; records=[]; start=time.monotonic(); maximum_seconds=50.
    def residual(q):
        T=fk(q)
        p=float(np.linalg.norm(T[:3,3]-goal[:3,3]))
        a=math.acos(float(np.clip((np.trace(goal[:3,:3].T@T[:3,:3])-1)/2,-1.,1.)))
        return p,a
    original_target,prediction=predict_held_axes_endpoint(fk,sdk,initial,pose_values(plan.poses[0]),
        position_tolerance_m=.010,orientation_tolerance_rad=math.radians(5.))
    original=original_target-counts
    assert original.tolist()==c['proposed_delta_counts'], (original.tolist(),c['proposed_delta_counts'])
    assert abs(prediction['predicted_position_error_m']-c['predicted_position_error_m'])<1e-12
    initial_p,initial_a=residual(initial)
    def compute(delta,label=None):
        if time.monotonic()-start>maximum_seconds: raise TimeoutError('bounded offline search deadline')
        delta=np.asarray(delta,dtype=float); key=tuple(delta)
        if key not in cache:
            assert delta.shape==(6,) and np.all(np.isfinite(delta))
            assert np.max(np.abs(delta))<=32+1e-8 and delta[1]==delta[4]==0.
            command=(counts+delta-2048)*Q
            fk(command)
            q=command-offset; p,a=residual(q); report=evaluate(fk(q))
            T=fk(q); projected=T@np.linalg.inv(goal)@pose_matrix(pose_values(plan.poses[2]))
            clearances={}
            for stage_name,stage_T in [('actual_current',T),('projected_contact',projected)]:
                clearances[stage_name]=min(float(np.min(_box_corners(box_center,stage_T[:3,:3],box_size)
                    @np.asarray(frozen['support_normal_base'])+frozen['support_offset_m']))
                    for _,box_center,box_size in _stage_boxes(stage_T,frozen['gripper'],r['measured_opening_width_m'],'y','z'))
            row=dict(delta_counts=delta.tolist(),position_error_m=p,orientation_error_deg=math.degrees(a),
                predicted_positions_rad=q.tolist(),predicted_command_positions_rad=command.tolist(),
                aperture=report,position_contract_ok=p<=.010+1e-12,orientation_contract_ok=a<=math.radians(5.)+1e-12,
                original_stage_support_clearances_m=clearances,
                cartesian_progress=p<initial_p-1e-6,following_gap_unchanged=bool(np.max(np.abs(command-q))<=.035),
                quantized=bool(np.all(delta==np.rint(delta))),prediction_only=True,
                future_response_verified=False,path_checked=False,execution_authorized=False)
            row['endpoint_all_original_gates_ok']=bool(row['position_contract_ok'] and row['orientation_contract_ok']
                and row['cartesian_progress'] and row['following_gap_unchanged'] and report['ok'])
            cache[key]=row
        row=cache[key]
        if label is not None: records.append(dict(label=label,result=row))
        return row
    compute(np.zeros(6),'initial_before_original_correction')
    compute(original,'original_pose_only_guide')
    def geometry_cost(q):
        delta=(np.asarray(q)-initial)/Q
        rounded=np.rint(delta)
        assert np.max(np.abs(delta-rounded))<1e-9
        z=compute(rounded)
        pose_cost=(z['position_error_m']/.010)**2
        if z['aperture']['ok']: return pose_cost
        aperture_deficit=max(0.,-z['aperture'].get('aperture_margin_m',-.1))
        support_deficit=max(0.,.003-min(z['original_stage_support_clearances_m'].values()))
        return 2.+aperture_deficit/.05+4.*support_deficit/.003+1e-6*pose_cost
    guides=[]
    for label,seed_delta in [('original_initial_seed',np.zeros(6,dtype=int)),('original_pose_guide_seed',original)]:
        seed_counts=counts+seed_delta
        seed_q=initial+seed_delta*Q
        guide,best_position=remaining_budget_guide(fk,residual,seed_q,seed_counts,counts,
            maximum_steps=12,steps=0,maximum_step_counts=4,maximum_total_counts=32,
            held_axes=(1,4),response_gains=np.ones(6),tolerances=(.010,math.radians(5.)),cost=geometry_cost)
        final=seed_delta+guide
        z=compute(final,label+'_geometry_guide')
        guides.append(dict(label=label,numerical_seed_counts=seed_delta.tolist(),remaining_guide_counts=guide.tolist(),
            single_command_delta_from_original_initial_counts=final.tolist(),best_position_m=best_position,
            endpoint_ok=z['endpoint_all_original_gates_ok']))
    free=[0,2,3,5]
    def expanded(x):
        delta=np.zeros(6);delta[free]=x;return delta
    def objective(x):
        z=compute(expanded(x))
        return (z['position_error_m']/.010)**2+1e-6*float(np.sum(np.asarray(x)**2))/32**2
    def constraint(x):
        z=compute(expanded(x));aperture=z['aperture']
        return np.array([(.010-z['position_error_m'])/.010,
            (5.-z['orientation_error_deg'])/5.,
            aperture.get('aperture_margin_m',-.1)/.05,
            (initial_p-z['position_error_m']-1e-6)/.010,
            (min(z['original_stage_support_clearances_m'].values())-.003)/.003])
    def bounded_jacobian(fun):
        def jac(x):
            base=np.asarray(fun(x),dtype=float);columns=[]
            for axis in range(4):
                step=1e-3 if x[axis]+1e-3<=32. else -1e-3
                trial=np.array(x,copy=True);trial[axis]+=step
                columns.append((np.asarray(fun(trial),dtype=float)-base)/step)
            return np.asarray(columns).T
        return jac
    diagnostic=minimize(objective,original[free].astype(float),method='SLSQP',jac=bounded_jacobian(objective),
        bounds=[(-32.,32.)]*4,constraints=[dict(type='ineq',fun=constraint,jac=bounded_jacobian(constraint))],
        options=dict(maxiter=50,ftol=1e-10,eps=1e-3,disp=False))
    compute(expanded(diagnostic.x),'continuous_pose_aperture_support_diagnostic')
    from itertools import product
    centre=np.rint(diagnostic.x).astype(int)
    for shift in product((-1,0,1),repeat=4):
        point=centre+np.asarray(shift)
        if np.max(np.abs(point))<=32:compute(expanded(point))
    candidates=[z for z in cache.values() if z['endpoint_all_original_gates_ok'] and z['quantized']]
    candidates.sort(key=lambda z:(z['position_error_m'], -z['aperture'].get('aperture_margin_m',-.1),z['delta_counts']))
    if candidates: compute(candidates[0]['delta_counts'],'best_quantized_constrained_diagnostic')
    traces=put('bounded_search_evaluations.json',list(cache.values()))
    selected=put('selected_comparisons.json',records)
    inputs=put('original_episode_inputs.json',dict(model_path=str(URDF),model_sha256=sha(URDF),
        fixed_plan_id=plan.plan_id,plan_wire_sha256=wire['wire_sha256'],initial_sdk_counts=counts.tolist(),
        initial_sdk_positions_rad=sdk.tolist(),initial_accepted_positions_rad=initial.tolist(),
        sdk_minus_accepted_rad=offset.tolist(),held_joints=['Joint2','Joint5'],
        original_per_axis_total_count_limit=32,command_domain=[0,4095],
        position_tolerance_m=.010,orientation_tolerance_deg=5.,opening_m=r['measured_opening_width_m'],
        original_guide_delta_counts=original.tolist(),original_prediction=prediction,
        initial_position_error_m=initial_p,initial_orientation_error_deg=math.degrees(initial_a)))
    assert all(sha(W/p)==h for p,h in source_pins.items())
    report=dict(kind='gen43_original_episode_aperture_endpoint_feasibility',completed_ns=time.time_ns(),
        scope='Counterfactual replacement of the sole original pregrasp correction from its initial state; not a second current episode',
        evaluated_original_plan=True,original_guide_exactly_reproduced=True,model_sha256=sha(URDF),
        original_source_count=len(source_pins),source_before_after_unchanged=True,release_manifest_sha256=sha(manifest),
        input=inputs,comparison=selected,evaluations=traces,evaluation_count=len(cache),
        solver=dict(name='original remaining_budget_guide with diagnostic stage0 aperture/support merit callback',
            fixed_seeds=2,maximum_coordinate_rounds_per_call=8,original_algorithm_unchanged=True,guides=guides,
            continuous_diagnostic=dict(method='single-start SLSQP including unchanged3mm support condition; production not changed',
                success=bool(diagnostic.success),status=int(diagnostic.status),message=str(diagnostic.message),
                iterations=int(diagnostic.nit),function_evaluations=int(diagnostic.nfev),
                x_counts=diagnostic.x.tolist(),constraints=constraint(diagnostic.x).tolist())),
        feasible_integer_endpoint_count=len(candidates),best_integer_endpoint=candidates[0]if candidates else None,
        geometric_feasibility_proven_under_constant_bias=bool(candidates),global_infeasibility_proven=False,
        future_following_verified=False,trajectory_ik_or_collision_path_verified=False,
        new_plan_required_proven=False,production_algorithm_changed=False,ros_init_called=False,
        network_boundary_attempts=boundary,temperature_read=False,hardware_contacted=False,
        motion_authorized=False,elapsed_sec=time.monotonic()-start,helper_sha256=sha(Path(__file__)))
    assert not boundary
    ref=put('result.json',report)
    put('seal.json',dict(result=ref,inputs=inputs,comparisons=selected,evaluations=traces))
    print(json.dumps(dict(result=ref,feasible_integer_endpoints=len(candidates),
        original_prediction=records[1]['result'],best=candidates[0]if candidates else None,
        elapsed_sec=report['elapsed_sec']),ensure_ascii=False))

if __name__=='__main__':
    OUT.mkdir(exist_ok=False)
    main()
