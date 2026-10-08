"""Exercise the real near-field loop across separately validated view handoffs."""
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from test_grasp_task_sequence import grasp_task_node as task


CFG = dict(near_field_strategy='single_snapshot_direct', near_field_replan_enabled=True,
           near_field_replan_required=True, near_field_replan_timeout_sec=30.)


def configured_loop(monkeypatch):
    node = task.GraspTaskNode.__new__(task.GraspTaskNode)
    node.active = True
    node._near_field_phase_started_sec = 9.5
    node._near_field_phase_deadline_sec = 40.
    node.set_state = Mock()
    node._set_near_field_preview_stream = Mock(return_value=True)
    node._freeze_execution_plan = lambda plan, **kw: plan
    node._coordinated_observation_views = lambda: True
    node._execute_clear_view_reacquisition = Mock(side_effect=AssertionError('unbudgeted old retry'))
    monkeypatch.setattr(task.rospy.Time, 'now', lambda: task.rospy.Time.from_sec(10.))
    monkeypatch.setattr(task.rospy, 'sleep', lambda _: None)
    def plan(name, stamp):
        return NS(plan_id=name, header=NS(stamp=task.rospy.Time.from_sec(stamp)),
                  diagnostic='FAR_FIELD_OBSERVATION_PLAN')
    initial, second, third = plan('origin',9.), plan('view2',11.), plan('view3',13.)
    return node, initial, second, third


def test_two_supplements_rebind_plan_and_cutoff_before_contact(monkeypatch):
    node, initial, second, third = configured_loop(monkeypatch)
    success = NS(plan_id='contact', diagnostic='CONTACT_EXECUTION_PLAN')
    calls = []
    def candidate(plan, stamp, cfg):
        calls.append((plan, stamp))
        return ((task.PlanValidationResult(True, age_sec=.1), success) if plan is third
                else (task.PlanValidationResult(False, 'NEAR_FIELD_SURFACE_VIEW_REQUIRED', 'gap'), None))
    node._copy_near_field_preview_candidate = candidate
    handoffs = iter([dict(waiting=True), dict(retry=True,current_plan=second,
        minimum_stamp_ns=12_000_000_000,phase_deadline_sec=60.), dict(retry=True,current_plan=third,
        minimum_stamp_ns=14_000_000_000,phase_deadline_sec=80.)])
    node._before_near_field_terminal_recovery = lambda *a: next(handoffs)
    result = node._maybe_rebind_near_field_grasp6d_plan(CFG,{},initial)
    assert result is success
    assert [item[0] for item in calls] == [initial,initial,second,third]
    assert calls[-2][1] == 12_000_000_000 and calls[-1][1] == 14_000_000_000
    node._execute_clear_view_reacquisition.assert_not_called()


@pytest.mark.parametrize('mutation',['missing_plan','stale_cutoff','expired','nan','invalid_retry'])
def test_malformed_continuation_cannot_publish_contact(monkeypatch,mutation):
    node,initial,second,_=configured_loop(monkeypatch)
    node._copy_near_field_preview_candidate=lambda *a:(task.PlanValidationResult(
        False,'NEAR_FIELD_NO_REACHABLE_CANDIDATE','complete failed search'),None)
    handoff=dict(retry=True,current_plan=second,minimum_stamp_ns=12_000_000_000,phase_deadline_sec=60.)
    if mutation=='missing_plan':del handoff['current_plan']
    elif mutation=='stale_cutoff':handoff['minimum_stamp_ns']=1
    elif mutation=='expired':handoff['phase_deadline_sec']=10.
    elif mutation=='nan':handoff['phase_deadline_sec']=float('nan')
    else:handoff['retry']=False
    node._before_near_field_terminal_recovery=lambda *a:handoff
    assert node._maybe_rebind_near_field_grasp6d_plan(CFG,{},initial) is None
    assert node.set_state.call_args[0][1]=='OBSERVATION_CONTINUATION_INVALID'
    assert node._set_near_field_preview_stream.call_args[0][1] is False


def test_target_loss_checks_active_view_after_rebind(monkeypatch):
    node,initial,second,_=configured_loop(monkeypatch)
    node._copy_near_field_preview_candidate=lambda *a:(task.PlanValidationResult(
        False,'NEAR_FIELD_SURFACE_VIEW_REQUIRED','gap'),None)
    node._before_near_field_terminal_recovery=lambda *a:dict(retry=True,current_plan=second,
        minimum_stamp_ns=12_000_000_000,phase_deadline_sec=60.)
    seen=[]
    def failure(plan,cfg):
        seen.append(plan)
        if plan is second:return task.PlanValidationResult(False,'TARGET_LOST','new frame missing')
    node._near_field_wait_failure=failure
    assert node._maybe_rebind_near_field_grasp6d_plan(CFG,{},initial) is None
    assert seen[-1] is second
    assert node.set_state.call_args[0][1].startswith('TARGET_LOST:')


def test_v2_without_new_view_does_not_take_legacy_clear_view_motion(monkeypatch):
    node,initial,_,_=configured_loop(monkeypatch)
    node._before_near_field_terminal_recovery=lambda *a:None
    node._copy_near_field_preview_candidate=lambda *a:(NS(ok=False,code='NEAR_FIELD_SURFACE_VIEW_REQUIRED',
        reason='gap with no new view',terminal_for_request=True),None)
    assert node._maybe_rebind_near_field_grasp6d_plan(CFG,{},initial,
        move_pose=lambda:None,strict_execute_pose=lambda:None) is None
    node._execute_clear_view_reacquisition.assert_not_called()
