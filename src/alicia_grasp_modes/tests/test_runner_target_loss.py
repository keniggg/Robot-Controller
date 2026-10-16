"""Same-selection irreversible target loss ends planning only; no ROS I/O."""
from copy import deepcopy
import json
from types import SimpleNamespace

import pytest

from alicia_grasp_modes.runner_contract import unknown_target_loss_reason
from test_mode_runner import evidence, fake_run, module


def lost_status(selection, observed=20200000000):
    return dict(mode='unknown', state='unavailable', target_locked=True, target_lost=True,
        generation=selection['generation'], selection_stamp_ns=int(selection['stamp_ns']),
        anchor_source_stamp_ns=19500000000,
        first_loss=dict(generation=selection['generation'], selection_stamp_ns=int(selection['stamp_ns']),
            observed_stamp_ns=observed, anchor_source_stamp_ns=19500000000,
            diagnostic_only=True, target_recovered=False, reason='exact_rgbd_stream_stale',
            source_stamp_ns=None, source_pair_available=False))


def setup_runner(monkeypatch, strategy='two_stage'):
    selection, _, _ = evidence()
    selection['strategy'] = strategy
    runner = module.ModeAwareGraspRunner()
    runner.selection = selection
    runner.minimum_stamp_ns = 20000000000
    runner.plan_validity_sec = 120.
    runner.expected_model = 'unknown_tabletop'
    monkeypatch.setattr(module.rospy, 'get_param', lambda *_: dict(selection, stamp_ns=str(selection['stamp_ns'])))
    monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(lambda: module.rospy.Time(21)))
    monkeypatch.setattr(module.rospy, 'is_shutdown', lambda: False)
    return runner, selection


def deliver(runner, report):
    runner.on_unknown_detector_status(SimpleNamespace(data=json.dumps(report)))


def test_exact_gen10_closed_bag_first_loss_is_terminal():
    selection = dict(mode='unknown', strategy='two_stage', generation=10, stamp_ns='1790667082936385869')
    # These exact identity/times come from the 073124Z sealed diagnostic bag.
    report = lost_status(selection, 1790667088425882816)
    report['anchor_source_stamp_ns'] = 1790667083386823654
    report['first_loss']['anchor_source_stamp_ns'] = report['anchor_source_stamp_ns']
    reason = unknown_target_loss_reason(report, selection, 1790667088431209325)
    assert reason.startswith('TARGET_LOST: exact_rgbd_stream_stale')
    assert 'generation=10' in reason


@pytest.mark.parametrize('strategy', ['direct', 'two_stage', 'observe_return'])
def test_current_loss_finishes_300_second_preview_wait_without_services(monkeypatch, strategy):
    runner, selection = setup_runner(monkeypatch, strategy)
    deliver(runner, lost_status(selection))
    # A condition wait would mean the callback did not end the planning wait.
    monkeypatch.setattr(runner.condition, 'wait', lambda *_: pytest.fail('wait after irrevocable loss'))
    assert runner.wait_for_preview(300.) is None
    assert runner.terminal_reason.startswith('TARGET_LOST: exact_rgbd_stream_stale')


def test_loss_before_runner_but_within_same_selection_is_still_irreversible(monkeypatch):
    runner, selection = setup_runner(monkeypatch)
    report = lost_status(selection, 19900000000)
    assert report['first_loss']['observed_stamp_ns'] < runner.minimum_stamp_ns
    deliver(runner, report)
    assert runner.terminal_reason
    report.update(state='ready', target_lost=False)
    deliver(runner, report)
    assert runner.terminal_reason  # no automatic recovery/reset of this attempt


@pytest.mark.parametrize('mutation', ['foreign_generation', 'foreign_selection', 'loss_generation',
    'loss_selection', 'future', 'before_selection', 'bool_time', 'string_time', 'null_time',
    'missing_first_loss', 'missing_time', 'anchor_mismatch', 'not_lost', 'not_locked', 'recovered',
    'wrong_mode', 'bool_generation', 'missing_reason', 'blank_reason', 'bad_json'])
def test_unbound_or_malformed_loss_cannot_end_this_attempt(monkeypatch, mutation):
    runner, selection = setup_runner(monkeypatch)
    report = lost_status(selection)
    if mutation == 'foreign_generation': report['generation'] += 1
    elif mutation == 'foreign_selection': report['selection_stamp_ns'] += 1
    elif mutation == 'loss_generation': report['first_loss']['generation'] += 1
    elif mutation == 'loss_selection': report['first_loss']['selection_stamp_ns'] += 1
    elif mutation == 'future': report['first_loss']['observed_stamp_ns'] = 21000000001
    elif mutation == 'before_selection': report['first_loss']['observed_stamp_ns'] = selection['stamp_ns']
    elif mutation == 'bool_time': report['first_loss']['observed_stamp_ns'] = True
    elif mutation == 'string_time': report['first_loss']['observed_stamp_ns'] = '20200000000'
    elif mutation == 'null_time': report['first_loss']['observed_stamp_ns'] = None
    elif mutation == 'missing_first_loss': del report['first_loss']
    elif mutation == 'missing_time': del report['first_loss']['observed_stamp_ns']
    elif mutation == 'anchor_mismatch': report['anchor_source_stamp_ns'] += 1
    elif mutation == 'not_lost': report['target_lost'] = False
    elif mutation == 'not_locked': report['target_locked'] = False
    elif mutation == 'recovered': report['first_loss']['target_recovered'] = True
    elif mutation == 'wrong_mode': report['mode'] = 'carton'
    elif mutation == 'bool_generation': report['generation'] = True
    elif mutation == 'missing_reason': del report['first_loss']['reason']
    elif mutation == 'blank_reason': report['first_loss']['reason'] = ' '
    if mutation == 'bad_json': runner.on_unknown_detector_status(SimpleNamespace(data='{'))
    else: deliver(runner, report)
    assert not runner.terminal_reason
    assert runner.preview is None


def test_changed_authoritative_selection_ignores_even_bound_old_loss(monkeypatch):
    runner, selection = setup_runner(monkeypatch)
    monkeypatch.setattr(module.rospy, 'get_param', lambda *_: dict(selection, generation=3))
    deliver(runner, lost_status(selection))
    assert runner.selection_changed
    assert not runner.terminal_reason


def test_inflight_execution_retains_task_control_and_inference(monkeypatch):
    runner, selection = setup_runner(monkeypatch)
    runner.execution_inflight = True
    runner.request_plan = lambda *_: pytest.fail('inflight inference revoked')
    deliver(runner, lost_status(selection))
    runner.stop_candidate_computation()
    assert not runner.terminal_reason


def test_loss_ends_authority_wait_before_promote_or_current_plan_rpc(monkeypatch):
    runner, selection = setup_runner(monkeypatch)
    deliver(runner, lost_status(selection))
    def forbidden(*_): pytest.fail('authority RPC after revoked target')
    assert runner.wait_for_execution_authority(forbidden, forbidden, None, 'a'*24, 60.) is None


def test_valid_preview_cannot_replace_latched_loss(monkeypatch):
    runner, selection = setup_runner(monkeypatch)
    _, _, preview = evidence()
    preview.diagnostic = 'FAR_FIELD_OBSERVATION_PLAN'
    preview.candidate_source = 'tabletop_geometry'
    monkeypatch.setattr(module, 'plan_id_matches_content', lambda *_: True)
    deliver(runner, lost_status(selection))
    runner.on_preview(preview)
    assert runner.preview is None


def test_loss_during_wait_exits_and_stops_only_own_inference(monkeypatch):
    runner, params, calls = fake_run(monkeypatch, lambda *_: pytest.fail('lost target started'))
    monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(lambda: module.rospy.Time(21)))
    def wait(_):
        deliver(runner, lost_status(runner.selection))
        return None
    runner.wait_for_preview = wait
    assert runner.run() == 3
    assert calls == [('inference', True), ('inference', False)]


def test_latched_loss_at_subscription_does_not_even_start_inference(monkeypatch):
    runner, params, calls = fake_run(monkeypatch, lambda *_: pytest.fail('lost target started'))
    def subscribe(topic, _kind, callback, **_):
        if topic == '/perception/unknown/detector_status':
            callback(SimpleNamespace(data=json.dumps(lost_status(runner.selection, 19900000000))))
    monkeypatch.setattr(module.rospy, 'Subscriber', subscribe)
    assert runner.run() == 3
    assert calls == []


def test_loss_after_last_audit_check_is_serialized_before_start_claim(monkeypatch):
    runner, params, calls = fake_run(monkeypatch, lambda *_: pytest.fail('lost target started'))
    monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(lambda: module.rospy.Time(21)))
    original = runner.audit_matches_preview
    def checked_then_lost(*args):
        result = original(*args)
        deliver(runner, lost_status(runner.selection))
        return result
    runner.audit_matches_preview = checked_then_lost
    assert runner.run() == 5
    assert calls == [('inference', True), ('inference', False)]
    assert not runner.execution_inflight
