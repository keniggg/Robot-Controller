"""Ending sequence and original idle gates, without ROS or control transport."""

from copy import deepcopy
import importlib.util
import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


TOOLS = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('finish_returned_grasp_test', TOOLS / 'finish_returned_grasp.py')
finish = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(finish)


def snapshot(manual):
    return dict(manual_parameter=manual, gui_manual_ack=manual,
        node_identities={'driver': {'pid': 10, 'start_ticks': '200'}},
        epoch_ns=1_000_000_000, selection=dict(generation=6, stamp_ns=2_000_000_000),
        policy={'require_actuation_confirmation': True})


class Adapter:
    def __init__(self, *observations):
        self.observations = list(observations)
        self.calls, self.requests = [], []

    def verify_closed(self):
        self.calls.append('closed')

    def recheck(self):
        self.calls.append('recheck')

    def observe_idle(self):
        self.calls.append('observe')
        return self.observations.pop(0)

    def prepare_manual_request(self):
        self.calls.append('prepare')

    def request_manual(self, value):
        self.calls.append('request')
        self.requests.append(deepcopy(value))
        return 100

    def wait_gui_own_ack(self, requested):
        assert requested == 100
        self.calls.append('own_ack')


def test_already_manual_finishes_without_publisher_or_request_or_new_ack():
    adapter = Adapter(snapshot(True))
    result = finish.finish_returned_attempt(adapter)
    assert result['passed'] and result['outcome'] == 'already_manual'
    assert result['manual_request_sent'] is False and not adapter.requests
    assert adapter.calls == ['closed', 'recheck', 'observe']


def test_auto_requests_once_only_after_connection_and_fresh_boundary():
    adapter = Adapter(snapshot(False), snapshot(False), snapshot(True))
    result = finish.finish_returned_attempt(adapter)
    assert result['passed'] and result['outcome'] == 'restored_manual'
    assert len(adapter.requests) == 1
    assert adapter.calls == ['closed', 'recheck', 'observe', 'prepare', 'closed',
        'recheck', 'observe', 'request', 'own_ack', 'closed', 'recheck', 'observe']


def test_original_runner_can_restore_manual_while_publisher_connects():
    adapter = Adapter(snapshot(False), snapshot(True))
    assert finish.finish_returned_attempt(adapter)['outcome'] == 'already_manual'
    assert not adapter.requests and 'own_ack' not in adapter.calls


@pytest.mark.parametrize('field,value', [('manual_parameter', 1), ('gui_manual_ack', 1),
                                       ('manual_parameter', False)])
def test_ambiguous_manual_state_does_not_request(field, value):
    observed = snapshot(True)
    observed[field] = value
    adapter = Adapter(observed)
    with pytest.raises(finish.FinishError):
        finish.finish_returned_attempt(adapter)
    assert not adapter.requests


@pytest.mark.parametrize('method', ['verify_closed', 'recheck'])
def test_unsealed_record_or_revoked_authority_stops_before_observation(method):
    adapter = Adapter(snapshot(True))
    def reject():
        raise finish.FinishError('not valid')
    setattr(adapter, method, reject)
    with pytest.raises(finish.FinishError, match='not valid'):
        finish.finish_returned_attempt(adapter)
    assert 'observe' not in adapter.calls and not adapter.requests


@pytest.mark.parametrize('field,value', [('epoch_ns', 2_000_000_000),
    ('node_identities', {'driver': {'pid': 11, 'start_ticks': '300'}}),
    ('selection', dict(generation=7, stamp_ns=3_000_000_000)),
    ('policy', {'require_actuation_confirmation': False})])
def test_changed_binding_after_wait_prevents_request(field, value):
    second = snapshot(False)
    second[field] = value
    adapter = Adapter(snapshot(False), second)
    with pytest.raises(finish.FinishError, match='binding changed'):
        finish.finish_returned_attempt(adapter)
    assert not adapter.requests


def test_missing_fresh_ack_fails_without_resending():
    adapter = Adapter(snapshot(False), snapshot(False))
    def missing(_):
        raise finish.FinishError('no new GUI acknowledgement')
    adapter.wait_gui_own_ack = missing
    with pytest.raises(finish.FinishError, match='no new GUI'):
        finish.finish_returned_attempt(adapter)
    assert len(adapter.requests) == 1


def test_changed_epoch_after_ack_does_not_report_success():
    final = snapshot(True)
    final['epoch_ns'] += 1
    adapter = Adapter(snapshot(False), snapshot(False), final)
    with pytest.raises(finish.FinishError, match='binding changed'):
        finish.finish_returned_attempt(adapter)
    assert len(adapter.requests) == 1


class Stamp:
    def __init__(self, value): self.value = value
    def to_nsec(self): return self.value


def idle_fixture():
    validator = finish.checked_module(TOOLS.parent / '.runtime/restore_gen62_after_handoff_20261022.py',
        'fc8317a94a95d91b0addcd3c57a4054c29713ef77be2fa921a10b35b07849c9f', 'original_finish_idle_test')
    now, epoch = 5_000_000_000, 1_000_000_000
    names = ['Joint%d' % i for i in range(1, 7)]
    samples = [dict(stamp_ns=now-1_950_000_000+i*100_000_000,
        received_ns=now-1_945_000_000+i*100_000_000, names=names, positions=[.1]*6,
        frame_id='sdk_measured', caller=validator.DRIVER) for i in range(20)]
    states = {}
    for key, owner in validator.OWNERS.items():
        message = SimpleNamespace(_connection_header=dict(callerid=owner, latching='1'),
            data=True, active=False, stamp=Stamp(epoch),
            header=SimpleNamespace(stamp=Stamp(now-10_000_000)), status_list=[])
        if key == 'mode': message.data = json.dumps(dict(active=False, pending='', start_reserved=False))
        if key == 'actuation': message.data = 'CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE'
        states[key] = (message, now-10_000_000)
    return validator, samples, states, now, epoch


@pytest.mark.parametrize('change', ['none', 'stale_sdk', 'moving', 'wrong_gui_owner',
                                 'wrong_epoch', 'unconfirmed', 'task_active', 'action_active'])
def test_original_idle_validator_remains_enforced(change):
    validator, samples, states, now, epoch = idle_fixture()
    if change == 'stale_sdk':
        now += 300_000_000
    elif change == 'moving':
        samples[-1]['positions'] = [.2]*6
    elif change == 'wrong_gui_owner':
        states['manual'][0]._connection_header['callerid'] = '/foreign'
    elif change == 'wrong_epoch':
        epoch += 1
    elif change == 'unconfirmed':
        states['actuation'][0].data = 'UNCONFIRMED:FEEDBACK_STALE'
    elif change == 'task_active':
        states['task'][0].active = True
    elif change == 'action_active':
        states['arm'][0].status_list = [SimpleNamespace(status=1)]
    if change == 'none':
        assert validator.validate_idle_snapshot(samples, states, now, epoch)['actuation_status'].startswith('CONFIRMED:')
    else:
        with pytest.raises(AssertionError):
            validator.validate_idle_snapshot(samples, states, now, epoch)


def test_legacy_own_ack_requires_gui_latch_and_new_reception():
    validator, _, states, now, _ = idle_fixture()
    ack = states['manual']
    assert validator.own_ack(ack, now-20_000_000)
    assert not validator.own_ack(ack, now)
    ack[0]._connection_header['latching'] = '0'
    assert not validator.own_ack(ack, now-20_000_000)


@pytest.mark.parametrize('initial_passed', [True, False])
def test_cleanup_failure_preserves_original_outcome_and_allows_receipt(initial_passed):
    result = dict(passed=initial_passed, error='original idle rejection' if not initial_passed else None)
    calls = []
    def unregister_fail():
        calls.append('failing')
        raise RuntimeError('unsubscribe failure')
    adapter = object.__new__(finish.RosFinishAdapter)
    adapter.publisher = SimpleNamespace(unregister=unregister_fail)
    adapter.subscriptions = [SimpleNamespace(unregister=lambda: calls.append('remaining'))]
    finish.close_adapter(adapter, result)
    assert calls == ['failing', 'remaining']
    assert result['passed'] is False
    assert result['error'] == ('original idle rejection' if not initial_passed else None)
    assert 'unsubscribe failure' in result['cleanup_error']
    # Cleanup returns to the unconditional receipt writer, even on failure.
    assert json.loads(json.dumps(result))['cleanup_error'] == result['cleanup_error']


def test_validator_keeps_original_sibling_imports_for_delayed_callbacks(tmp_path):
    sibling = tmp_path / 'finish_sibling_fixture.py'
    sibling.write_text('value = 42\n')
    validator = tmp_path / 'validator_fixture.py'
    raw = b'def later():\n    from finish_sibling_fixture import value\n    return value\n'
    validator.write_bytes(raw)
    prior = list(sys.path)
    try:
        with finish.validator_imports(validator, validator):
            mod = finish.checked_module(validator, hashlib.sha256(raw).hexdigest(), 'finish_delayed_fixture')
            assert mod.later() == 42
            assert sys.path.count(str(tmp_path)) == 1
        assert sys.path == prior
    finally:
        sys.modules.pop('finish_sibling_fixture', None)
        sys.modules.pop('finish_delayed_fixture', None)
