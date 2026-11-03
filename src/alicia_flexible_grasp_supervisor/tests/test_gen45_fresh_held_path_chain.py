"""Task -> original pose0/held services -> original complete path validators.

G44 plan/start wires are real. The short joint curve and future controller
response are explicit counterfactual providers, not historical MoveIt arrays
or proof of a future physical grasp. All held/CAD/tracking/stop kernels run.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

import numpy as np
import pytest
import rospy
from moveit_msgs.msg import RobotTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from alicia_flexible_grasp.grasp.contact_execution import plan_aperture_evaluator
from alicia_flexible_grasp.grasp.rich_plan_integrity import pose_values
from alicia_flexible_grasp.robot.observation_path_guard import wire_digest
from alicia_flexible_grasp.robot.pregrasp_compensation import pose_matrix
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES,SDK_QUANTUM_RAD as Q
from test_contact_execution import make_task,module as task
from test_endpoint_correction import sample as simulated_following_sample
from test_gen45_pose0_handoff_contract import g44_gateway,no_external_boundaries

PATH_SHA='bf5b0b8679766a9e5856f05f34698e54977b1f0a1902a452f6bad56c9f50b037'


def chain(monkeypatch,bad_held_curve=False):
    r=g44_gateway(monkeypatch)
    p=Path(__file__).parent/'fixtures/gen45_g44_held_path.json'
    assert hashlib.sha256(p.read_bytes()).hexdigest()==PATH_SHA
    path=json.loads(p.read_text())
    assert path['plan_id']==r.c.plan.plan_id and path['original_full_tracking_kernel']['ok']
    assert not path['historical_MoveIt_trajectory'] and not path['hardware_executed']
    node,_,clock,_=make_task(monkeypatch)
    monkeypatch.setattr(rospy,'get_param',lambda key,default=None:r.params.get(key,default))
    monkeypatch.setattr(rospy,'get_time',lambda:r.state['now'])
    monkeypatch.setattr(rospy.Time,'now',staticmethod(lambda:rospy.Time.from_sec(r.state['now'])))
    node._following_epoch_ns=r.first['epoch_ns']
    def task_feedback(row):
        j=JointState();j.header.frame_id='sdk_measured'
        ns=row['accepted_stamp_ns'];j.header.stamp=rospy.Time(ns//10**9,ns%10**9)
        j.name=list(ARM_NAMES)+['right_finger']
        j.position=list(row['accepted_positions_rad'])+[row['accepted_gripper_opening_m']]
        node.latest_accepted_joint_state=j
    task_feedback(r.first)
    r.params['/alicia_controller/constraints']=deepcopy(path['controller_constraints'])
    r.params['/alicia_controller/stop_trajectory_duration']=path['stop_duration_sec']
    r.node.joint_names=list(ARM_NAMES)+['right_finger']
    r.node._observation_measured_opening=lambda:r.first['accepted_gripper_opening_m']
    r.node._observation_controller_hold=lambda names:JointTrajectoryPoint(
        positions=r.state['sdk'].tolist(),velocities=[0.]*6,accelerations=[0.]*6)
    # Contact scenes do not use an observation-view image provider. Current
    # frozen geometry, authority and the entire path remain production code.
    r.node._unknown_observation_view_audit=lambda *a:None
    r.node._revalidate_unknown_observation_view=lambda scene,audit:None
    r.node._observation_sync_hold_evidence=dict(kind='explicit_replay_desired_reference')
    r.planner.manipulator=NS(get_planning_frame=lambda:'base_link',get_end_effector_link=lambda:'tool0')
    r.planner._observation_branch_signature=lambda:(r.c.fk.model_sha256,)
    original_context=r.node._pregrasp_context
    def context(plan,index=0):
        result=original_context(plan,index);result['pose_index']=index
        return result
    r.node._pregrasp_context=context
    source_snapshot=r.node._endpoint_following_snapshot
    def snapshot(*a,**kw):
        if r.context['pose_index']==0:
            row=source_snapshot(*a,**kw)
        elif kw.get('after_ns'):
            kw['context_validator']()
            r.state['now']+=1.
            row=simulated_following_sample(r.c.fk,r.state['sdk'],r.state['actual'],r.state['now'])
            row['epoch_ns']=r.first['epoch_ns']
            row['accepted_gripper_opening_m']=r.first['accepted_gripper_opening_m']
            row['opening_stamp_ns']=row['accepted_stamp_ns']
        else:
            kw['context_validator']()
            row=deepcopy(r.second)
        task_feedback(row)
        return row
    r.node._endpoint_following_snapshot=snapshot
    submissions=[];audits=[];requests=[]
    def probe(goal,execute,**kw):
        assert execute and kw['start_joint_positions']==pytest.approx(path['curve_positions_rad'][0])
        assert list(goal)==pytest.approx(path['curve_positions_rad'][-1])
        tr=RobotTrajectory();tr.joint_trajectory.joint_names=list(ARM_NAMES)
        positions=deepcopy(path['curve_positions_rad'])
        if bad_held_curve:
            mid=((np.array(positions[0])+positions[-1])/2).tolist();mid[1]+=2*Q
            positions.insert(1,mid)
        for i,q in enumerate(positions):
            tr.joint_trajectory.points.append(JointTrajectoryPoint(positions=q,
                velocities=[0.]*6,accelerations=[0.]*6,
                time_from_start=rospy.Duration(path['duration_sec']*i/(len(positions)-1))))
        if not bad_held_curve:assert wire_digest(tr)==path['trajectory_sha256']
        try:
            # This executes the real _validate_pregrasp_step_path, real frozen
            # path method, real continuous CAD+following-support+stop kernels.
            audit=r.planner.observation_path_validator(tr)
        except Exception as exc:
            audits.append(dict(ok=False,reason=str(exc)))
            return False,str(exc)
        audits.append(dict(audit,ok=True))
        kw['before_execute']()
        submissions.append(wire_digest(tr))
        bias=r.state['sdk']-r.state['actual']
        r.state['sdk']=np.asarray(goal);r.state['actual']=r.state['sdk']-bias
        r.state['now']+=path['duration_sec']
        return True,'explicit inert controller provider completed'
    r.planner.plan_and_execute_joint_probe=Mock(side_effect=probe)
    def discovery(name,timeout):
        assert name in ('/supervisor/compensate_pregrasp','/supervisor/compensate_approach')
        assert timeout==2.;r.state['now']+=.005
    def service(name,kind):
        callback={'/supervisor/compensate_pregrasp':r.node.handle_compensate_pregrasp,
                  '/supervisor/compensate_approach':r.node.handle_compensate_approach}[name]
        def invoke(plan,execute):
            requests.append((name,bool(execute)))
            return callback(NS(plan=plan,execute=execute))
        return invoke
    monkeypatch.setattr(rospy,'wait_for_service',discovery)
    monkeypatch.setattr(rospy,'ServiceProxy',service)
    r.task=node;r.path=path;r.submissions=submissions;r.audits=audits;r.requests=requests
    return r


def begin_held_phase(r):
    r.node._pregrasp_task_stage=task.GraspStages.APPROACH_TARGET
    r.node._compensation_task_label='planning bounded 6D approach'


def test_original_g44_task_noop_then_fresh_held_service_runs_complete_kernel_once(monkeypatch):
    r=chain(monkeypatch)
    assert r.task._compensate_fixed_endpoint(r.c.plan,r.context['config'],0)
    first=r.task._last_fixed_endpoint_compensation_report
    assert first['no_motion_submitted'] and first['steps_completed']==0
    assert not r.submissions and not r.audits
    r.node._ensure_trajectory_controllers_started.assert_not_called()
    begin_held_phase(r)
    assert r.task._compensate_fixed_endpoint(r.c.plan,r.context['config'],1),r.task.set_state.call_args
    report=r.task._last_fixed_endpoint_compensation_report
    assert report['execution_policy']=='held_axes_approach'
    assert report['position_tolerance_m']==min(.012,
        r.context['config']['unknown_measured_endpoint_position_tolerance_m'])
    assert report['approach_convergence_policy']=='bounded_approach_max_12mm'
    assert report['steps_completed']==1 and report['position_error_m']==pytest.approx(.0044989782849423786)
    assert len(r.submissions)==1 and len(r.audits)==1
    audit=r.audits[0]
    assert audit['interval_checks']>0 and audit['following_support']['box_checks']>0
    assert audit['following_support']['minimum_support_clearance_lower_bound_m']>=.003-1e-9
    assert audit['execution_tracking_contract']['commanded_stop_reserve_rad']>0
    assert r.task._last_measured_aperture_sample['required_next_path_contract']=='fresh_held_axes_approach'
    assert not r.task._compensate_fixed_endpoint(r.c.plan,r.context['config'],1)
    assert len(r.submissions)==1


def test_noop_cannot_skip_original_next_held_axis_path_rejection(monkeypatch):
    r=chain(monkeypatch,bad_held_curve=True)
    assert r.task._compensate_fixed_endpoint(r.c.plan,r.context['config'],0)
    begin_held_phase(r)
    assert not r.task._compensate_fixed_endpoint(r.c.plan,r.context['config'],1)
    assert r.audits[-1]['ok'] is False and 'MOVES_HELD_AXIS' in r.audits[-1]['reason']
    assert not r.submissions
    assert not any(k.startswith('/supervisor/pregrasp_compensation_episodes/')for k in r.params)


def test_manual_revocation_after_zero_step_blocks_next_original_gateway(monkeypatch):
    r=chain(monkeypatch)
    assert r.task._compensate_fixed_endpoint(r.c.plan,r.context['config'],0)
    begin_held_phase(r);r.authority['manual']=True
    assert not r.task._compensate_fixed_endpoint(r.c.plan,r.context['config'],1)
    assert not r.submissions and not r.audits
    r.planner.plan_and_execute_joint_probe.assert_not_called()


def test_zero_step_contract_never_relaxes_final_measured_contact_opening():
    from test_gen45_pose0_handoff_contract import original
    c=original();actual_contact=pose_matrix(pose_values(c.plan.poses[2]))
    report=plan_aperture_evaluator(c.plan,{},.020,2)(actual_contact)
    assert not report['ok'] and report['code']=='MEASURED_APERTURE_INSUFFICIENT'
    assert report['projected_contact_is_measured'] is True
    assert report['required_open_width_m']>.020
