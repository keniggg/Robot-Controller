"""Preserve the first cancellation cause without changing execution authority."""
import json
import time
from unittest import mock

import pytest

from test_pregrasp_gateway import real_context, make_gateway, GraspStages, NS, module


@pytest.mark.parametrize('fault,expected', [
    ('manual', 'manual_control_active'),
    ('stage', 'task_stage'),
    ('epoch', 'control_reference_epoch_ns'),
    ('revocation', 'revocation'),
    ('config', 'grasp_configuration_changed'),
    ('deadline', 'deadline_expired'),
    ('actuation', "actuation_status='UNCONFIRMED:ENCODER_RESPONSE_TIMEOUT'"),
    ('disconnect', 'active task publisher is unavailable'),
])
def test_first_authority_rejection_keeps_specific_cause(monkeypatch, fault, expected):
    node, context, params = real_context(monkeypatch)
    logger = mock.Mock()
    monkeypatch.setattr(module.rospy, 'logwarn', logger)
    if fault == 'manual': node._manual_control_active = lambda: True
    if fault == 'stage': node._pregrasp_task_stage = GraspStages.APPROACH_TARGET
    if fault == 'epoch': node._control_reference_epoch_ns += 1
    if fault == 'revocation': node._observation_contexts.revocation += 1
    if fault == 'config': params['/grasp']['new_setting'] = True
    if fault == 'deadline': context['deadline'] = time.monotonic()-1.
    if fault == 'actuation': node._fresh_actuation_status = lambda: 'UNCONFIRMED:ENCODER_RESPONSE_TIMEOUT'
    if fault == 'disconnect': node._require_live_observation_task.side_effect = ValueError('active task publisher is unavailable')
    assert not node._pregrasp_still_authorized(context)
    first = dict(context['authority_rejection'])
    assert expected in first['reason']
    assert first['plan_id'] == context['scene'].plan_id
    assert first['deadline_monotonic'] == context['deadline']
    node._validate_pregrasp_context = mock.Mock(side_effect=ValueError('later terminal callback'))
    assert not node._pregrasp_still_authorized(context)
    assert context['authority_rejection'] == first
    assert logger.call_count == 1


def test_valid_authority_has_no_rejection_record_or_log(monkeypatch):
    node, context, _ = real_context(monkeypatch)
    logger = mock.Mock()
    monkeypatch.setattr(module.rospy, 'logwarn', logger)
    assert node._pregrasp_still_authorized(context)
    assert 'authority_rejection' not in context
    logger.assert_not_called()


def test_rejection_diagnostic_reaches_compensation_service_failure(monkeypatch):
    node, planner, plan, _, calls, _ = make_gateway(monkeypatch, day='20260920_wrist')
    def execute(*args, **kwargs):
        node._validate_pregrasp_context = mock.Mock(side_effect=ValueError('test owner revoked'))
        assert not planner.observation_execution_authorized()
        return False, 'observation authority revoked during execution'
    planner.plan_and_execute_joint_probe = execute
    result = node.handle_compensate_pregrasp(NS(plan=plan, execute=True))
    assert not result.success and not calls
    evidence = json.loads(result.message)
    assert evidence['authority_rejection']['reason'] == 'test owner revoked'
    assert 'PREGRASP_STEP_REJECTED' in evidence['code']
