"""Terminal messages traverse the real parser and near-field consumer loop.

ROS transport is mocked by the common fixture. The preview parser, freshness
checks, terminal handling, and execution-plan freeze remain production code.
"""
from unittest.mock import Mock

import pytest

from test_observe_return_handoff import make_loop, module


CODES = (
    'UNKNOWN_OBSERVATION_NO_SAFE_VIEW',
    'SILHOUETTE_OBSERVATION_NO_SAFE_VIEW',
    'OBSERVATION_REFERENCE_UNAVAILABLE',
    'OBSERVE_RETURN_TARGET_CHANGED',
    'OBSERVE_RETURN_REGISTRATION_FAILED',
    'OBSERVE_RETURN_INFORMATION_GAIN_FAILED',
)
REASON = ('OBSERVATION_SEARCH_INCOMPLETE: bounded comparison checked 4/220 '
          'views; unchecked views are not proven unreachable')


def terminal(code, stamp=10.01):
    message = module.Grasp6DPlan()
    message.header.frame_id = 'base_link'
    message.header.stamp = module.rospy.Time.from_sec(stamp)
    message.valid = False
    message.candidate_source = 'near_field_terminal'
    message.diagnostic = code + ': ' + REASON
    return message


@pytest.mark.parametrize('code', CODES)
def test_fresh_supported_observation_failure_survives_actual_parser(monkeypatch, code):
    node, origin, _, _, _, cfg, _, _ = make_loop(monkeypatch)
    node.latest_grasp6d_preview_plan = terminal(code)
    result, candidate = node._copy_near_field_preview_candidate(origin, 10_000_000_000, cfg)
    assert not result.ok
    assert candidate is None
    assert (result.code, result.reason) == (code, REASON)
    node._freeze_execution_plan.assert_not_called()


@pytest.mark.parametrize('code', CODES)
def test_actual_consumer_ends_with_producer_reason_without_executing(monkeypatch, code):
    node, origin, _, _, _, cfg, states, _ = make_loop(monkeypatch)
    node.latest_grasp6d_preview_plan = terminal(code)
    move, execute = Mock(), Mock()
    result = node._maybe_rebind_near_field_grasp6d_plan(cfg, {}, origin,
        move_pose=move, strict_execute_pose=execute)
    assert result is None
    assert states[-1] == (module.GraspStages.FAILED, code + ': ' + REASON)
    assert node._copy_near_field_preview_candidate.call_count == 1
    assert [call.args[1] for call in node._set_near_field_preview_stream.call_args_list] == [True, False]
    node._freeze_execution_plan.assert_not_called()
    node._simulate_grasp6d_plan_if_required.assert_not_called()
    move.assert_not_called()
    execute.assert_not_called()
    assert node._bound_execution_plan.plan_id == origin.plan_id


@pytest.mark.parametrize('stamp,minimum', [(0., 10_000_000_000), (9.9, 10_000_000_000),
                                          (10.01, 10_020_000_000)])
def test_missing_old_or_previous_request_terminal_does_not_override_current_window(monkeypatch, stamp, minimum):
    node, origin, _, _, _, cfg, _, _ = make_loop(monkeypatch)
    node.latest_grasp6d_preview_plan = terminal(CODES[0], stamp)
    result, candidate = node._copy_near_field_preview_candidate(origin, minimum, cfg)
    assert candidate is None and not result.ok
    assert result.code == 'FINAL_REFINE_3D_INVALID'
    assert not result.terminal_for_request
    node._freeze_execution_plan.assert_not_called()


def test_previous_request_terminal_is_waited_past_then_current_reason_is_consumed(monkeypatch):
    node, origin, _, _, clock, cfg, states, _ = make_loop(monkeypatch)
    node.latest_grasp6d_preview_plan = terminal(CODES[0], 9.99)
    sleeps = []
    def next_request_message(seconds):
        sleeps.append(seconds)
        assert len(sleeps) == 1
        node._freeze_execution_plan.assert_not_called()
        node.latest_grasp6d_preview_plan = terminal(CODES[2], 10.06)
        clock[0] = 10.07
    monkeypatch.setattr(module.rospy, 'sleep', next_request_message)
    assert node._maybe_rebind_near_field_grasp6d_plan(cfg, {}, origin) is None
    assert len(sleeps) == 1
    assert states[-1] == (module.GraspStages.FAILED, CODES[2] + ': ' + REASON)
    assert node._copy_near_field_preview_candidate.call_count == 2
    node._freeze_execution_plan.assert_not_called()


@pytest.mark.parametrize('mutation', ['wrong_source', 'marked_valid', 'missing_separator',
                                      'unsupported_code', 'direct_disabled'])
def test_untyped_or_malformed_preview_never_becomes_terminal_authority_or_contact(monkeypatch, mutation):
    node, origin, _, _, _, cfg, _, _ = make_loop(monkeypatch)
    message = terminal(CODES[0])
    if mutation == 'wrong_source':
        message.candidate_source = 'measured_observation'
    elif mutation == 'marked_valid':
        message.valid = True
    elif mutation == 'missing_separator':
        message.diagnostic = CODES[0]
    elif mutation == 'unsupported_code':
        message.diagnostic = 'UNKNOWN_UNSUPPORTED_FAILURE: ' + REASON
    else:
        cfg['near_field_strategy'] = 'legacy'
    node.latest_grasp6d_preview_plan = message
    result, candidate = node._copy_near_field_preview_candidate(origin, 10_000_000_000, cfg)
    assert candidate is None and not result.ok
    assert result.code == 'NEAR_FIELD_PLAN_PHASE_INVALID'
    node._freeze_execution_plan.assert_not_called()
