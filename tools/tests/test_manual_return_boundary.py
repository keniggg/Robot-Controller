from copy import deepcopy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from test_finish_returned_grasp import idle_fixture, Adapter, finish, snapshot

path = Path(__file__).resolve().parents[1]/'grasp_attempt_boundary.py'
spec = importlib.util.spec_from_file_location('manual_return_boundary_test', path)
boundary = importlib.util.module_from_spec(spec)
spec.loader.exec_module(boundary)


def test_unconfirmed_terminal_task_can_return_to_manual_without_motion_readiness():
    validator, samples, states, now, epoch = idle_fixture()
    states['actuation'][0].data = 'UNCONFIRMED:DISCONTINUOUS_FEEDBACK_RECOVERY'
    states['enabled'][0].data = False
    with pytest.raises(AssertionError):
        validator.validate_idle_snapshot(samples, states, now, epoch)
    freeze = lambda: json.dumps((samples, states), default=vars, sort_keys=True)
    original = freeze()
    result = boundary.validate_manual_return_snapshot(samples, states, now, epoch, validator.OWNERS)
    assert result['automatic_motion_authorized'] is False
    assert result['actuation_status'] == 'UNCONFIRMED:DISCONTINUOUS_FEEDBACK_RECOVERY'
    assert result['motion_enabled'] is False and freeze() == original
    adapter = Adapter(*[dict(snapshot(manual), idle=result) for manual in (False, False, True)])
    assert finish.finish_returned_attempt(adapter)['outcome'] == 'restored_manual'
    assert len(adapter.requests) == 1


@pytest.mark.parametrize('fault', ['task', 'action', 'moving', 'stale', 'owner', 'epoch', 'repeated'])
def test_return_to_manual_retains_activity_feedback_and_ownership_gates(fault):
    validator, samples, states, now, epoch = idle_fixture()
    if fault == 'task': states['task'][0].active = True
    if fault == 'action': states['arm'][0].status_list = [NS(status=1)]
    if fault == 'moving': samples[-1]['positions'][0] += .01
    if fault == 'stale': now += 1_000_000_000
    if fault == 'owner': states['manual'][0]._connection_header['callerid'] = '/wrong'
    if fault == 'epoch': epoch += 1
    if fault == 'repeated': samples[-1]['stamp_ns'] = samples[-2]['stamp_ns']
    with pytest.raises(ValueError, match='MANUAL_RETURN_BLOCKED'):
        boundary.validate_manual_return_snapshot(samples, states, now, epoch, validator.OWNERS)


@pytest.mark.parametrize('payload', ['{}', '{"active":false,"pending":""}',
    '{"active":false,"pending":false,"start_reserved":false}',
    '{"active":false,"pending":null,"start_reserved":false}',
    '{"active":false,"pending":"unknown","start_reserved":false}'])
def test_manual_return_rejects_missing_or_malformed_router_state(payload):
    validator, samples, states, now, epoch = idle_fixture()
    states['mode'][0].data = payload
    with pytest.raises(ValueError, match='MANUAL_RETURN_BLOCKED'):
        boundary.validate_manual_return_snapshot(samples, states, now, epoch, validator.OWNERS)
