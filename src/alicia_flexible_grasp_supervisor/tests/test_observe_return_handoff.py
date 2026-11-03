"""Actual near-field loop, candidate validation and freeze; all ROS I/O mocked."""
from unittest import mock

import pytest

import test_grasp_task_sequence as fixture_module


module = fixture_module.grasp_task_node


def make_loop(monkeypatch, strategy='single_snapshot_direct'):
    helper = fixture_module.GraspTaskSequenceTest()
    node = module.GraspTaskNode.__new__(module.GraspTaskNode)
    clock = [10.05]

    def contact(stamp):
        plan = helper._rich_plan(stamp_sec=stamp)
        plan.model_choice = 'unknown_tabletop'
        plan.diagnostic = module._CONTACT_EXECUTION_PLAN
        helper._mark_valid_3d(plan)
        helper._mark_contact_geometry(plan)
        return plan

    origin = helper._rich_plan(stamp_sec=9.)
    origin.model_choice = 'unknown_tabletop'
    origin.diagnostic = module._FAR_FIELD_OBSERVATION_PLAN
    origin.plan_id = fixture_module.compute_plan_id(origin)
    supplemental = contact(10.)
    node.latest_grasp6d_plan = origin
    node.latest_grasp6d_preview_plan = supplemental
    node.latest_obj = helper._object_at(.40, stamp_sec=10.)
    node.latest_obj_time = node.latest_obj.header.stamp
    node.active = True
    node._near_field_phase_started_sec = 10.
    node._near_field_phase_deadline_sec = 40.
    states = []
    node.set_state = lambda *args, **kwargs: states.append(args)
    node._freeze_execution_plan(origin)
    node._freeze_execution_plan = mock.Mock(wraps=node._freeze_execution_plan)
    node._copy_near_field_preview_candidate = mock.Mock(wraps=node._copy_near_field_preview_candidate)
    node._set_near_field_preview_stream = mock.Mock(return_value=True)
    node._request_near_field_preview_stream = mock.Mock(return_value=True)
    node._simulate_grasp6d_plan_if_required = mock.Mock(return_value=True)
    monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(lambda:module.rospy.Time.from_sec(clock[0])))
    monkeypatch.setattr(module.rospy, 'get_param', lambda name, default=None:default)
    monkeypatch.setattr(module.rospy, 'is_shutdown', lambda:False)
    monkeypatch.setattr(module.time, 'monotonic', lambda:clock[0])
    monkeypatch.setattr(module.rospy, 'sleep', lambda seconds:clock.__setitem__(0, clock[0]+seconds))
    config = dict(near_field_strategy=strategy, near_field_replan_enabled=True,
        near_field_replan_required=True, near_field_replan_timeout_sec=30.,
        near_field_replan_snapshot_slack_sec=1., plan_validity_sec=5.,
        target_max_drift_m=.02, target_observation_validity_sec=1.5)
    return node, origin, supplemental, contact, clock, config, states, helper


def test_supplemental_contact_is_not_frozen_before_return_and_new_contact(monkeypatch):
    node, origin, supplemental, contact, clock, config, states, helper = make_loop(monkeypatch)
    returned = contact(50.1)
    hooks, waits = [], []

    def before_freeze(candidate, current, cfg):
        hooks.append(candidate.plan_id)
        assert current.plan_id == origin.plan_id
        assert node._bound_execution_plan.plan_id == origin.plan_id
        node._freeze_execution_plan.assert_not_called()
        if len(hooks) == 1:
            assert candidate.plan_id == supplemental.plan_id
            # Return motion outlasts the old near-field deadline. Its service
            # authorizes a separate bounded, post-return contact interval.
            clock[0] = 50.05
            node._near_field_phase_started_sec = 50.05
            node._near_field_phase_deadline_sec = 60.
            return dict(retry=True, minimum_stamp_ns=50_050_000_001,
                        phase_deadline_sec=60.)
        assert candidate.plan_id == returned.plan_id
        return None

    def fresh_contact_after_one_wait(seconds):
        waits.append(seconds)
        assert len(waits) == 1
        assert node.latest_grasp6d_preview_plan.plan_id == supplemental.plan_id
        node._freeze_execution_plan.assert_not_called()
        clock[0] = 50.15
        node.latest_grasp6d_preview_plan = returned
        node.latest_obj = helper._object_at(.40, stamp_sec=50.1)
        node.latest_obj_time = node.latest_obj.header.stamp

    node._before_near_field_contact_freeze = before_freeze
    monkeypatch.setattr(module.rospy, 'sleep', fresh_contact_after_one_wait)
    result = node._maybe_rebind_near_field_grasp6d_plan(config, {}, origin)
    assert result.plan_id == returned.plan_id
    assert hooks == [supplemental.plan_id, returned.plan_id]
    assert len(waits) == 1  # The old supplemental frame was rejected by the real timestamp gate.
    candidate_calls = node._copy_near_field_preview_candidate.call_args_list
    assert len(candidate_calls) == 3
    assert candidate_calls[0].args[1] == 10_000_000_000
    assert [call.args[1] for call in candidate_calls[1:]] == [50_050_000_001]*2
    node._freeze_execution_plan.assert_called_once()
    assert node._freeze_execution_plan.call_args.args[0].plan_id == returned.plan_id
    assert node._bound_execution_plan.plan_id == returned.plan_id
    assert node._bound_target_occlusion_allowed
    node._simulate_grasp6d_plan_if_required.assert_not_called()
    node._set_near_field_preview_stream.assert_called_once_with(config, True)


def test_return_failure_preserves_far_field_authority_and_never_freezes_contact(monkeypatch):
    node, origin, supplemental, contact, clock, config, states, helper = make_loop(monkeypatch)
    def failed_return(*args):
        node.set_state(module.GraspStages.FAILED, 'OBSERVE_RETURN_FAILED: measured return rejected')
        return dict(failed=True)
    node._before_near_field_contact_freeze = mock.Mock(side_effect=failed_return)
    result = node._maybe_rebind_near_field_grasp6d_plan(config, {}, origin)
    assert result is None
    node._before_near_field_contact_freeze.assert_called_once()
    node._freeze_execution_plan.assert_not_called()
    assert node._bound_execution_plan.plan_id == origin.plan_id
    node._simulate_grasp6d_plan_if_required.assert_not_called()
    assert states[-1][0] == module.GraspStages.FAILED


@pytest.mark.parametrize('handoff,code', [
    ({'retry':False}, 'CONTACT_HANDOFF_INVALID'),
    ({'retry':True, 'minimum_stamp_ns':11_000_000_000, 'phase_deadline_sec':10.}, 'CONTACT_HANDOFF_EXPIRED'),
    ({'retry':True, 'minimum_stamp_ns':11_000_000_000, 'phase_deadline_sec':float('nan')}, 'CONTACT_HANDOFF_EXPIRED')])
def test_invalid_return_handoff_never_freezes_contact(monkeypatch, handoff, code):
    node, origin, supplemental, contact, clock, config, states, helper = make_loop(monkeypatch)
    node._before_near_field_contact_freeze = mock.Mock(return_value=handoff)
    result = node._maybe_rebind_near_field_grasp6d_plan(config, {}, origin)
    assert result is None
    node._freeze_execution_plan.assert_not_called()
    assert node._bound_execution_plan.plan_id == origin.plan_id
    assert states[-1] == (module.GraspStages.FAILED, code)


@pytest.mark.parametrize('strategy', ['legacy_gated', 'single_snapshot_direct'])
def test_existing_no_hook_strategy_retains_its_original_freeze_and_simulation(monkeypatch, strategy):
    node, origin, supplemental, contact, clock, config, states, helper = make_loop(monkeypatch, strategy)
    assert not hasattr(node, '_before_near_field_contact_freeze')
    result = node._maybe_rebind_near_field_grasp6d_plan(config, {}, origin)
    assert result.plan_id == supplemental.plan_id
    node._freeze_execution_plan.assert_called_once()
    assert node._bound_execution_plan.plan_id == supplemental.plan_id
    if strategy == 'legacy_gated':
        node._request_near_field_preview_stream.assert_called_once_with(config)
        node._simulate_grasp6d_plan_if_required.assert_called_once()
        node._set_near_field_preview_stream.assert_not_called()
    else:
        node._set_near_field_preview_stream.assert_called_once_with(config, True)
        node._simulate_grasp6d_plan_if_required.assert_not_called()
        node._request_near_field_preview_stream.assert_not_called()


def test_direct_configuration_skips_near_field_hook_and_preserves_bound_plan(monkeypatch):
    node, origin, supplemental, contact, clock, config, states, helper = make_loop(monkeypatch)
    config['near_field_replan_enabled'] = False
    node._before_near_field_contact_freeze = mock.Mock(side_effect=AssertionError('unexpected handoff'))
    result = node._maybe_rebind_near_field_grasp6d_plan(config, {}, origin)
    assert result is origin
    node._before_near_field_contact_freeze.assert_not_called()
    node._freeze_execution_plan.assert_not_called()
    node._set_near_field_preview_stream.assert_not_called()
    node._request_near_field_preview_stream.assert_not_called()
