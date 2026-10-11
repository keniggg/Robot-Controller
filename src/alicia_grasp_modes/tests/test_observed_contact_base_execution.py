"""Current-view handoff through the original task loop; no ROS/hardware I/O.

Mode-specific session/audit admission is tested separately. Here its None
result feeds the real preview, freeze, checkpoint and stage orchestration.
Motion/encoder responses are transport fixtures, not physical feasibility.
"""
from copy import deepcopy
from types import SimpleNamespace as NS
from unittest import mock

import pytest

from test_observe_return_handoff import make_loop, module as base
from test_grasp_task_sequence import compute_plan_id
from alicia_grasp_modes.observe_return_task import ObserveReturnTask


def current_view_case(monkeypatch):
    node, observation, contact, _, clock, cfg, states, helper = make_loop(monkeypatch)
    observation.candidate_source = 'measured_observation'
    observation.candidate_source_lineage = ['measured_observation']
    observation.poses = observation.poses[:1]
    observation.poses[0].position.x -= .03
    observation.has_candidate_model_width = False
    observation.candidate_width_m = observation.required_open_width_m = 0.
    observation.contact_geometry_evidence = ''
    observation.plan_id = compute_plan_id(observation)
    base.GraspTaskNode._freeze_execution_plan(node, observation)
    node._freeze_execution_plan.reset_mock()
    node._before_near_field_contact_freeze = mock.Mock(return_value=None)
    node._mode_bound_selection = dict(mode='unknown', strategy='observe_return')
    node._observe_return_session = dict(version=2, phase='observing',
        origin_plan_id=observation.plan_id, view_plan_id=observation.plan_id)
    return NS(node=node, observation=observation, contact=contact, clock=clock,
              cfg=cfg, states=states, helper=helper)


def test_current_view_handoff_freezes_only_the_new_contact_and_keeps_occlusion_policy(monkeypatch):
    c = current_view_case(monkeypatch)
    frozen = c.node._maybe_rebind_near_field_grasp6d_plan(c.cfg, {}, c.observation)
    assert frozen.plan_id == c.contact.plan_id != c.observation.plan_id
    assert frozen is not c.contact
    c.node._before_near_field_contact_freeze.assert_called_once()
    c.node._freeze_execution_plan.assert_called_once_with(c.contact, allow_target_occlusion=True)
    assert c.node._bound_target_occlusion_allowed
    assert c.node._bound_execution_plan.plan_id == c.contact.plan_id
    assert len(frozen.poses) == 4
    # A CONTACT plan never starts another observation-measurement lease.
    c.node._diagnostic_observation_only = lambda: False
    assert not ObserveReturnTask._observation_measurement_scope(c.node, frozen)


@pytest.mark.parametrize('fault', ['stale_source', 'target_moved', 'changed_content'])
def test_original_preview_gates_reject_before_current_contact_handoff(monkeypatch, fault):
    c = current_view_case(monkeypatch)
    if fault == 'stale_source':
        c.contact.header.stamp = base.rospy.Time.from_sec(9.9)
        c.contact.plan_id = compute_plan_id(c.contact)
    elif fault == 'target_moved':
        c.node.latest_obj = c.helper._object_at(.46, stamp_sec=10.)
    else:
        c.contact.poses[0].position.x += .01
    result = c.node._maybe_rebind_near_field_grasp6d_plan(c.cfg, {}, c.observation)
    assert result is None
    c.node._before_near_field_contact_freeze.assert_not_called()
    c.node._freeze_execution_plan.assert_not_called()
    assert c.node._bound_execution_plan.plan_id == c.observation.plan_id


def execution_case(monkeypatch, fault=None):
    c = current_view_case(monkeypatch)
    node, events = c.node, []
    cfg = dict(c.cfg, observation_reuse_position_tolerance_m=.01,
        final_visual_refine_enabled=False, unknown_lift_after_close_enabled=False,
        measured_endpoint_check_enabled=True, contact_endpoint_precision_enabled=True,
        pregrasp_cartesian_compensation_enabled=True)
    node._position_only_execute_globally_enabled = lambda: False
    node._current_tool_pose_base = lambda: base.observation_plan_pose(c.observation)
    node._current_observation_view_reusable = lambda *a: True
    node._wait_for_motion_settle = lambda *a: True
    node._record_reused_observation_following = lambda *a: True
    node._command_gripper_position = lambda *a, **k: True
    node._validate_measured_observation_envelope = lambda *a: True
    node._wait_for_fresh_observation_camera_target_range = lambda *a: base.PlanValidationResult(True)
    node._set_near_field_active = lambda *a, **k: None
    node._set_contact_endpoint_precision = lambda *a, **k: True
    node._compensate_pregrasp_endpoint = lambda *a: events.append('compensate') or True
    node._record_and_validate_measured_endpoint = lambda *a, **k: events.append('endpoint_record') or True
    node._wait_for_measured_endpoint_contract = lambda *a: (
        events.append('endpoint_check') or (fault != 'arrival_mismatch', .001, .1))
    node._validate_measured_contact_before_close = lambda *a: events.append('contact_before_close') or True
    node._close_gripper = lambda *a, **k: (events.append('close') or (True, 'fixture'))
    node._finish_grasp_without_lift = lambda *a: events.append('finish_without_lift') or True

    def planned(pose, execute):
        events.append(('plan', pose.pose.position.x, execute))
        assert execute is False
        if fault == 'replaced_during_planning':
            node._bound_execution_plan_id = 'f' * 24
        return NS(success=True, message='strict fixture planning')

    def strict(pose, execute):
        events.append(('strict', pose.pose.position.x, execute))
        assert execute is True
        assert node._bound_execution_plan_id == c.contact.plan_id
        return NS(success=True, message='strict fixture execution')

    def linear(pose, execute):
        return strict(pose, execute) if execute else planned(pose, execute)

    c.events, c.cfg, c.planned, c.strict, c.linear = events, cfg, planned, strict, linear
    return c


def run_execution(c):
    return c.node._execute_grasp6d_plan(c.cfg,
        dict(open_position_m=.05, use_compliant_close=False), .05,
        c.planned, c.linear, object(), None, c.observation,
        strict_execute_pose=c.strict)


def test_actual_base_current_view_rebind_runs_pregrasp_approach_grasp_and_close(monkeypatch):
    c = execution_case(monkeypatch)
    assert run_execution(c), c.states
    planned = [event for event in c.events if isinstance(event, tuple) and event[0] == 'plan']
    strict = [event for event in c.events if isinstance(event, tuple) and event[0] == 'strict']
    assert [event[1] for event in planned] == [pose.position.x for pose in c.contact.poses[:3]]
    assert [event[1] for event in strict] == [pose.position.x for pose in c.contact.poses[:3]]
    assert c.events.count('endpoint_check') == c.events.count('endpoint_record') == 3
    assert c.events.index('contact_before_close') < c.events.index('close')
    assert c.events[-1] == 'finish_without_lift'
    c.node._freeze_execution_plan.assert_called_once()


@pytest.mark.parametrize('fault', ['replaced_during_planning', 'arrival_mismatch'])
def test_contact_handoff_does_not_skip_commit_or_measured_arrival(monkeypatch, fault):
    c = execution_case(monkeypatch, fault)
    assert not run_execution(c)
    assert 'close' not in c.events and 'contact_before_close' not in c.events
    strict = [event for event in c.events if isinstance(event, tuple) and event[0] == 'strict']
    assert len(strict) == (0 if fault == 'replaced_during_planning' else 1)
    if fault == 'replaced_during_planning':
        assert any('EXECUTION_PLAN_MISMATCH' in str(state) for state in c.states)
    else:
        assert 'endpoint_check' in c.events
