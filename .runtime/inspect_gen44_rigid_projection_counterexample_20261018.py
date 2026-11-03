#!/usr/bin/env python3
"""Expose the constant-residual assumption using closed G43 evidence only."""
import base64
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import socket
import sys
import time
W=Path(__file__).resolve().parent.parent
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N=E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
V=N/'verification'
OUT=V/'gen44_rigid_projection_semantics_review_r1'
sys.path[:0]=[str(W/'src/alicia_flexible_grasp_supervisor/src'),str(W/'devel/lib/python3/dist-packages')]
attempts=[]
class OfflineSocket(socket.socket):
    def __new__(cls,*a,**k):
        attempts.append('socket');raise RuntimeError('offline closed evidence only')
socket.socket=OfflineSocket
import numpy as np
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan
from alicia_flexible_grasp.grasp.contact_execution import frozen_contact_geometry,plan_aperture_evaluator
from alicia_flexible_grasp.grasp.gripper_geometry import evaluate_open_gripper_observation_envelope
from alicia_flexible_grasp.grasp.rich_plan_integrity import pose_values,validate_rich_geometry,plan_id_matches_content
from alicia_flexible_grasp.robot.pregrasp_compensation import pose_matrix
from alicia_flexible_grasp.robot.stationary_following import SerialUrdfFk,ARM_NAMES
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def ref(p):return dict(path=str(p),sha256=sha(p))
def put(name,value):
    p=OUT/name;p.write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    assert json.loads(p.read_text())==value
    return ref(p)
def main():
    assert E.is_dir();OUT.mkdir(exist_ok=False)
    source=W/'src/alicia_flexible_grasp_supervisor/tests/fixtures/gen44_g43_stage0_replay.json'
    assert sha(source)=='da66a49b1141e943e1b9e734621400a4130c17443c09516fabc9bc1e8a4c50f0'
    data=json.loads(source.read_text());plan=Grasp6DPlan()
    raw=base64.b64decode(data['plan_wire']['wire_base64'])
    assert hashlib.sha256(raw).hexdigest()==data['plan_wire']['wire_sha256']
    plan.deserialize(raw);assert plan_id_matches_content(plan)
    xml=W/data['robot_model_path'];assert sha(xml)==data['model_sha256']
    fk=SerialUrdfFk(xml.read_text(),ARM_NAMES)
    opening=data['actual_reached_pregrasp']['measured_opening_width_m']
    frozen=frozen_contact_geometry(plan,{},opening)
    obb,size,_=validate_rich_geometry(plan.object_geometry)
    from tf.transformations import quaternion_matrix
    kwargs=dict(gripper=frozen['gripper'],opening_width_m=opening,
        support_normal_base=frozen['support_normal_base'],support_offset_m=frozen['support_offset_m'],
        obb_center_base=obb[:3],R_base_obb=quaternion_matrix(obb[3:])[:3,:3],obb_size_xyz_m=size)
    pc=data['original_precheck'];hp=pc['held_approach_prediction']
    T0=fk(hp['predicted_pregrasp']['predicted_positions_rad'])
    T1=fk(hp['predicted_approach']['predicted_positions_rad'])
    measured=np.asarray(data['actual_reached_pregrasp']['actual_T_base_tool0'])
    cases={}
    for label,T,index in [('actual_reached_pregrasp',measured,0),('original_virtual_pregrasp',T0,0),
                          ('original_virtual_corrected_approach',T1,1)]:
        current=asdict(evaluate_open_gripper_observation_envelope(T_base_tool0=T,**kwargs))
        projection=plan_aperture_evaluator(plan,{},opening,index)(T)
        cases[label]=dict(current_noncontact_CAD=current,rigid_contact_projection=projection,
            source='actual recorded runtime FK'if label.startswith('actual')else'original conditional precheck q and exact URDF FK',
            future_measurement_claimed=False)
    assert cases['actual_reached_pregrasp']['current_noncontact_CAD']['ok']
    assert not cases['actual_reached_pregrasp']['rigid_contact_projection']['ok']
    assert cases['original_virtual_pregrasp']['current_noncontact_CAD']['ok']
    assert not cases['original_virtual_pregrasp']['rigid_contact_projection']['ok']
    rigid_approach=T0@np.linalg.inv(pose_matrix(pose_values(plan.poses[0])))@pose_matrix(pose_values(plan.poses[1]))
    contact=pc['contact_prediction'];curves=hp['guide_path_prediction']['curves']
    report=dict(kind='gen44_rigid_residual_projection_semantics_counterexample',completed_ns=time.time_ns(),
        plan_id=plan.plan_id,model_sha256=fk.model_sha256,fixture=ref(source),cases=cases,
        original_precheck_passed=pc['success'],original_precheck_is_conditional_not_future_feedback=True,
        original_held_approach_delta_counts=hp['predicted_approach']['proposed_delta_counts'],
        original_held_approach_position_error_m=hp['predicted_approach']['predicted_position_error_m'],
        rigid_vs_independently_corrected_approach_position_difference_m=float(np.linalg.norm(T1[:3,3]-rigid_approach[:3,3])),
        original_held_approach_command_curve=curves['command'],
        original_held_approach_predicted_actual_curve=curves['predicted_actual'],
        original_contact_prediction=contact,
        trajectory_arrays_recorded=False,original_service_audit_recalculated_as_full_path=False,
        actual_post_reached_next_motion_submitted=False,actual_future_path_pass_proven=False,
        conclusion='Rigid residual projection is not the commanded independent corrective route. Its failure alone is not a necessary future-path or grasp impossibility proof.',
        correction_to_prior_direction=dict(previous=ref(V/'gen43_stage0_contract_direction_review_r1/result.json'),
            preserved_original=True,early_rigid_projection_gate_recommendation_superseded=True,
            original_actual_aperture_numbers_still_correct=True,
            original_bounded_search_does_not_prove_intended_next_stage_infeasible=True),
        proposed_boundary='Current actual finite CAD and fresh authority remain hard; separately checked next held/contact path must still pass original full validators; final stage2 measured aperture/contact/height remain hard.',
        release_or_motion_authority_granted=False,production_modified_by_this_helper=False,
        network_boundary_attempts=attempts,ros_init_called=False,temperature_read=False,
        source={str(p):sha(p)for p in [xml,W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/gripper_geometry.py',
            W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/contact_linear_compensation.py',
            W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/held_axes_prediction.py']},
        helper_sha256=sha(Path(__file__)))
    assert not attempts
    result=put('result.json',report);put('seal.json',dict(result=result,fixture=ref(source)))
    print(json.dumps(dict(result=result,case_results={k:dict(current_ok=z['current_noncontact_CAD']['ok'],
        current_clearance_m=z['current_noncontact_CAD']['minimum_support_clearance_m'],
        old_projection_code=z['rigid_contact_projection']['code'])for k,z in cases.items()},
        next_route_difference_m=report['rigid_vs_independently_corrected_approach_position_difference_m']),ensure_ascii=False))
if __name__=='__main__':main()
