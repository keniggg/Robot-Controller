#!/usr/bin/env python3
"""Offline recorded-start replay with explicit hypothetical actuator responses.

No node, publisher, service, TF query or device access. Optional paths are
synthetic three-second quintics, not executed or MoveIt-planned trajectories.
"""
import argparse
import base64
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from alicia_flexible_grasp.robot.pregrasp_compensation import PregraspCompensation
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SerialUrdfFk, SDK_QUANTUM_RAD as Q


def hypothetical_sample(fk, sdk, measured, now):
    stamp = int((now-.05)*1e9)
    return dict(measurement_kind='stationary_sdk_following_not_executed_endpoint',
        model_sha256=fk.model_sha256,epoch_ns=1_000_000_000,joint_names=list(ARM_NAMES),
        stamp_sec=now,sdk_stamp_ns=stamp,accepted_stamp_ns=stamp,
        stationary_window_start_ns=stamp-300_000_000,stationary_window_end_ns=stamp,
        sdk_positions_rad=sdk.tolist(),accepted_positions_rad=measured.tolist(),
        position_error_m=float(np.linalg.norm(fk(sdk)[:3,3]-fk(measured)[:3,3])))


def path_check(record, fk, sdk, step):
    import rospy
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan
    from alicia_flexible_grasp.robot.observation_path_guard import FrozenObservationScene
    from alicia_flexible_grasp.robot.observation_tracking_contract import qualify_contract_path
    from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
    plan=Grasp6DPlan()
    plan.deserialize(base64.b64decode(record['frozen_contact_plan_wire_base64'],validate=True))
    scene=FrozenObservationScene.from_contact_plan(plan)
    trajectory=RobotTrajectory()
    trajectory.joint_trajectory.joint_names=list(ARM_NAMES)
    for q,t in [(sdk,0.),(step.positions,3.)]:
        p=JointTrajectoryPoint()
        p.positions=list(q);p.velocities=[0.]*6;p.accelerations=[0.]*6
        p.time_from_start=rospy.Duration(t)
        trajectory.joint_trajectory.points.append(p)
    hold=SimpleNamespace(positions=sdk.tolist(),velocities=[0.]*6,accelerations=[0.]*6)
    gripper=GripperGeometry(.05,.002,np.array([.0434,.0286,.0600]),
                            np.array([.1175,.1550,.0774]),.003)
    cfg=record['path_check_controller_configuration']
    result=qualify_contract_path(trajectory,hold,scene,fk,gripper,record['measured_opening_m'],
                                cfg['constraints'],cfg['stop_trajectory_duration'],
                                trajectory_stop_reserve=True)
    result['synthetic_path_not_execution_authority']=True
    return result


def replay(record, response, *, check_paths=False):
    fk=SerialUrdfFk(record['robot_description'],ARM_NAMES)
    if fk.model_sha256!=record['model_sha256']:
        raise ValueError('recorded model checksum mismatch')
    sdk=(np.array(record['initial_sdk_counts'])-2048)*Q
    actual=(np.array(record['initial_measured_counts'])-2048)*Q
    c=PregraspCompensation(fk,hypothetical_sample(fk,sdk,actual,10.),
        record['goal_position_m']+record['goal_quaternion_xyzw'])
    rows=[]
    try:
        for index in range(c.MAX_STEPS+1):
            code,step,evidence=c.evaluate(hypothetical_sample(fk,sdk,actual,10.+3*index))
            rows.append(dict(evidence,code=code))
            if step is None:
                break
            if check_paths:
                rows[-1]['conditional_path']=path_check(record,fk,sdk,step)
            delta=np.array(step.target_counts)-step.baseline_counts
            c.committed(step,11.+3*index)
            sdk=np.array(step.positions)
            if response=='other_five_axes_repeatable':actual+=delta*Q
            elif response=='joint3_stalled':
                delta[2]=0;actual+=delta*Q
        terminal=code
    except ValueError as exc:
        terminal=str(exc)
        if c.last_evidence is not None:rows.append(dict(c.last_evidence,code=terminal))
    return dict(hypothesis=response,terminal=terminal,steps=c.steps,trace=rows,
                physical_convergence_proven=False,motion_executed=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fixture',type=Path,default=Path(__file__).resolve().parents[1]/
        'src/alicia_flexible_grasp_supervisor/tests/fixtures/pregrasp_following_20260919.json')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--check-paths',action='store_true')
    args=parser.parse_args()
    record=json.loads(args.fixture.read_text())
    results=[replay(record,response,check_paths=args.check_paths and response=='other_five_axes_repeatable')
             for response in ('other_five_axes_repeatable','all_axes_stalled','joint3_stalled')]
    report=dict(scope='recorded_initial_encoders_plus_explicit_response_hypotheses',
        model_sha256=record['model_sha256'],synthetic_clock=True,grasp_success=False,
        physical_motion_executed=False,results=results)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as f:json.dump(report,f,indent=2,allow_nan=False)
    for result in results:
        print(result['hypothesis'],result['terminal'],'steps=',result['steps'])
        for row in result['trace']:
            if 'conditional_path' in row:
                proof=row['conditional_path']
                print('  synthetic path',row['steps_completed']+1,proof['ok'],proof.get('reason',''))


if __name__=='__main__':
    main()
