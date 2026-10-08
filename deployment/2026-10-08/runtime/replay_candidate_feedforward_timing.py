#!/usr/bin/env python3
"""Offline exact-action timing/certificate replay; no ROS node or service calls."""
from pathlib import Path
from copy import deepcopy
import hashlib, io, json, math
import numpy as np
from control_msgs.msg import FollowJointTrajectoryActionGoal
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan
from alicia_flexible_grasp.robot.observation_path_guard import (
    FrozenObservationScene, SerialUrdfFk, wire_digest,
    validate_observation_trajectory_velocity, segment_coefficients, _polynomial_bounds)
from alicia_flexible_grasp.robot.observation_tracking_contract import qualify_contract_path
from alicia_flexible_grasp.robot.strict_collinear_timing import smooth_strict_collinear, validate_collinear_refinement
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry

ROOT=Path(__file__).resolve().parents[1]
ARCHIVE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
OUT=ARCHIVE/'ros_log/candidate_real_grasp_20260929T052500Z/timing_analysis'
if not Path('/media/zhuyupei/SanStand').is_mount():raise SystemExit('external archive missing')
model=ARCHIVE/'ros_log/grasp_repair_resume_20260928T120606Z/expanded_metric_training_once/candidate_model/alicia_expanded_metric_candidate.urdf'
xml=model.read_text()
assert hashlib.sha256(xml.encode()).hexdigest()=='7812d6b69e854ed9f37b4e3cbd813ecd53dab3504065b2d04d287abc6fd852e0'
plan=Grasp6DPlan();plan.deserialize((OUT/'frozen_plan.roswire').read_bytes())
scene=FrozenObservationScene.from_contact_plan(plan)
logs=json.loads((OUT/'selected_rosout.json').read_text())
audits=[json.loads(r['text'].split(': ',1)[1]) for r in logs if r['text'].startswith('Frozen observation continuous path audit: ')][:2]
proposals=[json.loads(r['text'].split(': ',1)[1]) for r in logs if r['text'].startswith('Pregrasp feedforward proposal: ')]


def count_box_proof(tr,hold,proposal):
    # Exact copy of the production feedforward geometric box conditions, with
    # no gain from samples: every Noetic polynomial gets an interval enclosure.
    q=math.pi/2048
    baseline=(np.asarray(proposal['initial_sdk_counts'])-2048)*q
    delta=np.asarray(proposal['proposed_delta_counts'])
    points=tr.joint_trajectory.points
    ts=[p.time_from_start.to_sec() for p in points]
    first=next(i for i,t in enumerate(ts) if t>0)
    pairs=[(hold,points[first],ts[first])]+[(points[i],points[i+1],ts[i+1]-ts[i]) for i in range(first,len(points)-1)]
    rows=[]
    for start,end,duration in pairs:
        low,high=_polynomial_bounds(segment_coefficients(start,end,duration,6),0.,1.)
        if np.any(low<baseline-32*q-1e-9) or np.any(high>baseline+32*q+1e-9):raise ValueError('total count box exceeded')
        for j in np.flatnonzero(delta==0):
            if low[j]<baseline[j]-q/2 or high[j]>=baseline[j]+q/2:raise ValueError('held axis cell crossed')
        rows.append(dict(low=low.tolist(),high=high.tolist()))
    return dict(passed=True,continuous_polynomial_intervals=rows)

rows=[]
for index,(audit,proposal) in enumerate(zip(audits,proposals),1):
    message=FollowJointTrajectoryActionGoal();message.deserialize((OUT/('action_goal_%d.roswire'%index)).read_bytes())
    original=RobotTrajectory();original.joint_trajectory=deepcopy(message.goal.trajectory)
    assert wire_digest(original)==audit['trajectory_sha256']
    assert scene.digest==audit['scene_sha256']
    assert scene.plan_id==audit['plan_id']==proposal['plan_id']
    fk=SerialUrdfFk(xml,original.joint_trajectory.joint_names)
    assert fk.model_sha256==audit['model_sha256']
    hold=JointTrajectoryPoint(positions=audit['bridge_start_joint_positions_rad'],velocities=[0.]*6,accelerations=[0.]*6)
    cfg=audit['gripper_geometry_snapshot']
    gripper=GripperGeometry(cfg['max_inner_gap_m'],cfg['width_safety_margin_per_side_m'],np.array(cfg['finger_box_xyz_m']),np.array(cfg['palm_box_xyz_m']),cfg.get('support_clearance_m',.003))
    case=dict(pose_index=proposal['fixed_goal_pose_index'],exact_goal_id=message.goal_id.id,
        original_sha256=wire_digest(original),historical_audit=audit,
        historical_actual_endpoint=next(json.loads(r['text'].split(': ',1)[1]) for r in logs if r['text'].startswith('Pregrasp feedforward measured result: ') and json.loads(r['text'].split(': ',1)[1])['fixed_goal_pose_index']==proposal['fixed_goal_pose_index']),variants=[])
    for cap in (.002,.008):
        limits={name:cap for name in fk.names}
        candidate=original if cap==.002 else smooth_strict_collinear(original,limits,minimum_duration=3.)
        if candidate is None:raise ValueError('noncollinear recorded path')
        item=dict(global_velocity_cap_rad_s=cap,duration_sec=candidate.joint_trajectory.points[-1].time_from_start.to_sec(),trajectory_sha256=wire_digest(candidate))
        if cap!=.002:item['collinear_refinement']=validate_collinear_refinement(original,candidate)
        item['continuous_count_box']=count_box_proof(candidate,hold,proposal)
        try:
            item['velocity_proof']=validate_observation_trajectory_velocity(candidate,hold,limits)
            item['certificate']=qualify_contract_path(candidate,hold,scene,fk,gripper,audit['opening_width_m'],audit['controller_constraints_snapshot'],audit['execution_tracking_contract']['stop_trajectory_duration_sec'],trajectory_stop_reserve=True)
            item['passed_static_geometry_and_derivatives']=item['certificate']['ok']
        except Exception as exc:
            item.update(passed_static_geometry_and_derivatives=False,error=str(exc))
        buf=io.BytesIO();candidate.serialize(buf);(OUT/('probe%d_cap_%s_robot_trajectory.roswire'%(index,str(cap).replace('.','p')))).write_bytes(buf.getvalue())
        case['variants'].append(item)
        print(json.dumps(dict(pose_index=case['pose_index'],cap=cap,duration=item['duration_sec'],passed=item['passed_static_geometry_and_derivatives'],reason=item.get('certificate',{}).get('reason',item.get('error','')),support=item.get('certificate',{}).get('continuous_path',{}).get('following_support'))),flush=True)
    rows.append(case)
paths=[Path(__file__),ROOT/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/strict_collinear_timing.py',ROOT/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observation_path_guard.py',ROOT/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observation_tracking_contract.py']
report=dict(scope='offline recorded two probe curves, same stationary controller holds, frozen model/scene/opening, same stop duration and finite tracking-contract search',motion_authorized=False,hardware_or_future_tracking_validated=False,live_MoveIt_or_service_calls=0,retime_semantics='production smooth_strict_collinear retains every recorded waypoint; not duration division, fresh IK/MoveIt search or live authorization',source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},cases=rows)
(OUT/'feedforward_008_static_replay.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
