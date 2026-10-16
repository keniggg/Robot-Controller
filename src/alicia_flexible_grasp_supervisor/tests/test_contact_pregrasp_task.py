"""The dedicated nominal route keeps task retirement and actual arrival gates."""
from copy import deepcopy
import json
from types import SimpleNamespace as NS
from unittest import mock

import pytest
from test_pregrasp_task_compensation import task_case,task
from alicia_flexible_grasp.grasp.rich_plan_integrity import pose_values


def case(monkeypatch):
    n,p,pose,cfg,old=task_case();p.model_choice='unknown_tabletop'
    cfg['unknown_approach_held_axes_feedforward_enabled']=True
    events=[]
    def rpc(plan,execute):
        events.append('execute' if execute else 'check')
        assert plan is not p and plan==p
        return NS(success=True,message=json.dumps(dict(
            execution_policy='frozen_contact_held_approach_pregrasp_start',
            plan_id=p.plan_id,plan_source_ns=p.header.stamp.to_nsec(),
            fixed_goal_pose=list(pose_values(p.poses[0])),
            code='CONTACT_PREGRASP_EXECUTED' if execute else 'CONTACT_PREGRASP_CHECKED_ONLY',
            endpoint_confirmation_required=True,complete_execution_path_checked=bool(execute))))
    proxy=mock.Mock(side_effect=rpc)
    monkeypatch.setattr(task.rospy,'wait_for_service',mock.Mock())
    monkeypatch.setattr(task.rospy,'ServiceProxy',lambda name,kind:proxy)
    def bound(plan,config,label,action):
        events.append('bound_submit');return task.PlanValidationResult(True),action()
    n._invoke_plan_bound_action=mock.Mock(side_effect=bound)
    n._wait_for_motion_settle.side_effect=lambda *_:events.append('settle') or True
    n._compensate_pregrasp_endpoint.side_effect=lambda *_:events.append('original_correction') or True
    n._wait_for_measured_endpoint_contract.side_effect=lambda *_:events.append('actual_endpoint') or (True,.004,2.)
    return n,p,pose,cfg,old,proxy,events


def run(n,p,pose,cfg,old):
    return n._plan_and_execute_pose(task.GraspStages.MOVE_PREGRASP,'6D pregrasp',pose,
        old,'pregrasp',execution_plan=p,gcfg=cfg,execute_pose=old)


def test_full_plan_route_keeps_submission_boundary_and_original_measured_arrival(monkeypatch):
    n,p,pose,cfg,old,proxy,events=case(monkeypatch)
    assert run(n,p,pose,cfg,old)
    assert events==['check','bound_submit','execute','settle','original_correction','actual_endpoint']
    old.assert_not_called();assert proxy.call_count==2
    n._record_and_validate_measured_endpoint.assert_called_once()


@pytest.mark.parametrize('field',['plan_id','plan_source_ns','fixed_goal_pose','execution_policy','code','endpoint_confirmation_required','complete_execution_path_checked'])
def test_mismatched_service_report_cannot_grant_later_stage(monkeypatch,field):
    n,p,pose,cfg,old,proxy,events=case(monkeypatch);original=proxy.side_effect
    def wrong(plan,execute):
        result=original(plan,execute)
        if execute:
            report=json.loads(result.message);report[field]='wrong';result.message=json.dumps(report)
        return result
    proxy.side_effect=wrong
    assert not run(n,p,pose,cfg,old)
    old.assert_not_called();n._wait_for_motion_settle.assert_not_called()


@pytest.mark.parametrize('fault',['rpc_failure','bound_rejected','actual_unstable','correction_failed'])
def test_selected_route_failure_never_falls_back_to_pose_ik(monkeypatch,fault):
    n,p,pose,cfg,old,proxy,_=case(monkeypatch)
    if fault=='rpc_failure':proxy.return_value=NS(success=False,message='collision');proxy.side_effect=None
    if fault=='bound_rejected':n._invoke_plan_bound_action.side_effect=lambda *_:(task.PlanValidationResult(False,'CHANGED'),None)
    if fault=='actual_unstable':n._wait_for_measured_endpoint_contract.side_effect=None;n._wait_for_measured_endpoint_contract.return_value=(False,.02,6.)
    if fault=='correction_failed':n._compensate_pregrasp_endpoint.side_effect=None;n._compensate_pregrasp_endpoint.return_value=False
    assert not run(n,p,pose,cfg,old)
    old.assert_not_called()


@pytest.mark.parametrize('scope',['disabled','known_model','observation'])
def test_other_nominal_routes_keep_existing_behavior(monkeypatch,scope):
    n,p,pose,cfg,old,proxy,_=case(monkeypatch)
    if scope=='disabled':cfg['unknown_approach_held_axes_feedforward_enabled']=False
    if scope=='known_model':p.model_choice='carton_segment'
    if scope=='observation':p.diagnostic='FAR_FIELD_OBSERVATION_PLAN'
    assert run(n,p,pose,cfg,old)
    assert old.call_count==2;proxy.assert_not_called()


def mode_case(monkeypatch, fault=None):
    """Real production task MRO and ACK receiver; participant ACKs are synthetic."""
    import threading
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from test_task_modes import module as mode
    from test_contact_station_retirement import producer_context
    from alicia_grasp_modes.contact_station_retirement import ack
    from alicia_grasp_modes.rgb_observation_source import JOINT_NAMES
    c=producer_context(monkeypatch)
    old_node,p,pose,cfg,old,proxy,events=case(monkeypatch)
    n=mode.ModeAwareGraspTaskNode.__new__(mode.ModeAwareGraspTaskNode)
    n.__dict__.update(old_node.__dict__)
    n._observe_return_condition=threading.Condition(threading.RLock())
    n._contact_station_acks={}
    n._observation_measurement_request=deepcopy(c.node._observation_measurement_status['request'])
    p.target_track_id=n._observation_measurement_request['target_track_id']
    p.header.stamp=task.rospy.Time.from_sec(c.clock[0]-.01)
    n._mode_bound_selection=deepcopy(c.req['selection'])
    n._published_near_field_phase=deepcopy(c.phase)
    n._following_evidence_lock=threading.RLock()
    n._following_epoch_ns=c.req['control_epoch_ns']
    row=JointState();row.header.frame_id='sdk_measured';row.header.stamp=task.rospy.Time.now()
    row.name,row.position=list(JOINT_NAMES),[0.]*6
    n._following_accepted_history=[row]
    n._observe_return_live=lambda:None
    n._observation_measurement_live=lambda _:None
    n._observation_measurement_scope=lambda _:False
    n._validate_bound_plan_locked=lambda *_:NS(ok=n.active,code='CANCELLED',reason='task inactive')
    def publish(message):
        request=json.loads(message.data);events.append(request['state'])
        for participant in ('unknown','remote'):
            if fault=='missing_ack' and participant=='remote':
                c.clock[0]+=.81
                continue
            result=ack(request,participant)
            if fault=='wrong_digest' and participant=='remote':result['request_sha256']='f'*64
            n._contact_station_ack_cb(String(data=json.dumps(result)))
        if fault=='state_change' and request['state']=='retire':n.active=False
        if fault=='q_change' and request['state']=='retire':row.position[0]+=.006
        if fault=='epoch_change' and request['state']=='retire':n._following_epoch_ns+=1
    n._contact_station_pub=NS(publish=publish)
    return n,p,pose,cfg,old,proxy,events


@pytest.mark.parametrize('reuse',[False,True])
@pytest.mark.parametrize('fault',[None,'missing_ack','wrong_digest','state_change','q_change','epoch_change'])
def test_real_mode_mro_retires_station_before_selected_service_execution(monkeypatch,reuse,fault):
    n,p,pose,cfg,old,proxy,events=mode_case(monkeypatch,fault)
    result=n._validate_reused_contact_pregrasp(p,pose,cfg) if reuse else run(n,p,pose,cfg,old)
    assert result is (fault is None),events
    assert ('execute' in events) is (fault is None),events
    old.assert_not_called()
    if fault is None:
        assert events==['check','bound_submit','prepare','retire','execute','settle','original_correction','actual_endpoint']
        assert n._contact_station_retired_measurement
    else:
        assert not any(call.args[1] is True for call in proxy.call_args_list)


@pytest.mark.parametrize('wrong',[None,'verified','submission','path'])
def test_no_motion_receipt_retains_real_arrival_checks(monkeypatch,wrong):
    n,p,pose,cfg,old,proxy,events=case(monkeypatch);original=proxy.side_effect
    def reply(plan,execute):
        response=original(plan,execute)
        if execute:
            report=json.loads(response.message)
            report.update(code='CONTACT_PREGRASP_ALREADY_SATISFIED',current_joint_start_verified=True,
                no_motion_submitted=True,execution_submitted=False,complete_execution_path_checked=False)
            if wrong=='verified':report['current_joint_start_verified']=False
            if wrong=='submission':report['execution_submitted']=True
            if wrong=='path':report['no_motion_submitted']=False
            response.message=json.dumps(report)
        return response
    proxy.side_effect=reply
    assert run(n,p,pose,cfg,old) is (wrong is None)
    if wrong is None:
        assert events[-3:]==['settle','original_correction','actual_endpoint']
    else:n._wait_for_motion_settle.assert_not_called()


def test_selected_nominal_failure_cannot_enter_cached_pose_fk_recovery(monkeypatch):
    n,p,pose,cfg,old,proxy,events=case(monkeypatch);original=proxy.side_effect
    def fail(plan,execute):
        if execute:return NS(success=False,message='execute failed from cached plan (strict pose): owner changed')
        return original(plan,execute)
    proxy.side_effect=fail
    assert not run(n,p,pose,cfg,old)
    n._wait_for_motion_settle.assert_not_called()
    n._compensate_pregrasp_endpoint.assert_not_called()
