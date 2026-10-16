"""Serial entry/real measured handoff policy; no ROS master or robot I/O."""
from copy import deepcopy
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'tools/diagnostics'))
import confirmed_unknown_dispatch as dispatch


def fixture(metric=False):
    selection = dict(mode='unknown', strategy='observe_return', generation=11, stamp_ns=1_000_000_000)
    parameters = dict(camera={'width':640}, perception={}, handeye={},
        model_sha256='a'*64, observation_view_parameters={})
    context = dict(parameters, selection=selection, epoch_ns=2_000_000_000, tool_camera=[])
    source = dispatch.digest(context)
    stamp = 10_000_000_000
    status = dict(mode='unknown', generation=11, selection_stamp_ns=selection['stamp_ns'],
        state='ready' if metric else 'unavailable', target_locked=metric, target_lost=False,
        source_stamp_ns=stamp, color_stamp_ns=stamp, depth_stamp_ns=stamp,
        observation_interest=dict(observation_interest_ready=True, source_stamp_ns=stamp,
            source_id=source, source_context=context, generation=11,
            selection_stamp_ns=selection['stamp_ns'], epoch_ns=context['epoch_ns'],
            rgb_track=dict(confirmed=True, source_id=source, generation=11,
                source_stamps_ns=[stamp-200_000_000, stamp-100_000_000, stamp])))
    decision = dispatch.EntryDecision(selection)
    decision.observe_epoch(context['epoch_ns'])
    decision.observe(status)
    return decision, status, parameters


def measured_report(decision):
    ledger = dict(phase='awaiting_measured_target', selection=decision.selection,
        duration_sec=120., max_observations=3, attempts=1, deadline_ns='130000000000',
        deadline_monotonic=120., settled_ns='9000000000', reference_id='r'*64,
        request_id='request', source_id=decision.status['observation_interest']['source_id'],
        observation_index=1)
    handoff = dict(state='measured_ready', metric_geometry_authority=True,
        motion_authorized=False, source_stamp_ns=decision.status['source_stamp_ns'],
        **{key:ledger[key] for key in ('reference_id','request_id','source_id','observation_index')})
    status = dict(decision.status, state='ready', target_locked=True,
        rgb_acquisition_handoff=deepcopy(handoff))
    decision.observe(status)
    return dict(preview_only=False, measured_target_available=True, observation_completed=True,
        grasp_started=False, torque_commands_sent=False,
        requests=[dict(success=True, message='settled', ledger=ledger)], handoff=handoff)


def test_metric_priority_even_when_stale_rgb_interest_is_present():
    decision, _, _ = fixture(metric=True)
    decision.status['observation_interest']['source_context'] = None
    assert decision.choose(decision.selection, 10_100_000_000) == 'metric'


def test_uninitialized_current_rgb_entry_is_immediate():
    decision, _, parameters = fixture()
    assert decision.choose(decision.selection, 10_100_000_000) == 'rgb'
    decision.require_live_parameters(parameters, deepcopy(parameters), rgb=True)


@pytest.mark.parametrize('change', ['source','generation','context','epoch','track','duplicate',
    'old_track','reference','rgbd','future','stale','not_ready','locked_pending','lost'])
def test_changed_or_ineligible_frame_cannot_dispatch_rgb(change):
    decision, status, _ = fixture()
    interest = status['observation_interest']
    if change == 'source': interest['source_stamp_ns'] -= 1
    elif change == 'generation': interest['generation'] += 1
    elif change == 'context': interest['source_context']['camera'] = {}
    elif change == 'epoch': interest['epoch_ns'] += 1
    elif change == 'track': interest['rgb_track']['confirmed'] = False
    elif change == 'duplicate': interest['rgb_track']['source_stamps_ns'][0] = interest['rgb_track']['source_stamps_ns'][1]
    elif change == 'old_track': interest['rgb_track']['source_stamps_ns'][0] = decision.epoch_ns
    elif change == 'reference': interest['reference_id'] = 'previous episode'
    elif change == 'rgbd': status['depth_stamp_ns'] -= 1
    elif change == 'future': status['source_stamp_ns'] += 1_000_000_000
    elif change == 'stale': status['source_stamp_ns'] -= 1_000_000_000
    elif change == 'not_ready': interest['observation_interest_ready'] = False
    elif change == 'locked_pending': status['target_locked'] = True
    elif change == 'lost': status['target_lost'] = True
    decision.observe(status)
    if change in ('future', 'stale', 'not_ready', 'locked_pending'):
        assert decision.choose(decision.selection, 10_100_000_000) == 'wait'
    else:
        with pytest.raises(ValueError):
            decision.choose(decision.selection, 10_100_000_000)


def test_latest_false_tombstone_cannot_reuse_earlier_ready_interest():
    decision, status, _ = fixture()
    status['observation_interest'] = dict(observation_interest_ready=False)
    decision.observe(status)
    assert decision.choose(decision.selection, 10_100_000_000) == 'wait'


def test_foreign_status_is_ignored_and_same_generation_loss_is_latched():
    decision, status, _ = fixture()
    foreign = dict(status, generation=10, target_lost=True)
    decision.observe(foreign)
    assert decision.choose(decision.selection, 10_100_000_000) == 'rgb'
    decision.observe(dict(status, target_lost=True))
    decision.observe(status)
    with pytest.raises(ValueError, match='target_lost'):
        decision.choose(decision.selection, 10_100_000_000)


def test_epoch_excursion_and_return_stays_revoked():
    decision, _, _ = fixture()
    old = decision.epoch_ns
    decision.observe_epoch(old+1)
    decision.observe_epoch(old)
    with pytest.raises(ValueError, match='epoch_changed'):
        decision.choose(decision.selection, 10_100_000_000)


@pytest.mark.parametrize('age', [.8, .3])
def test_exact_existing_age_limit_is_not_extended(age):
    decision, _, _ = fixture()
    decision.max_age_ns = int(age*1e9)
    assert decision.choose(decision.selection, 10_000_000_000+decision.max_age_ns) == 'rgb'
    assert decision.choose(decision.selection, 10_000_000_001+decision.max_age_ns) == 'wait'


@pytest.mark.parametrize('change', ['only_completed','preview','failure','ref','source','request',
    'index','unsettled','expired','mono_expired','budget','selection','stale_handoff','authority'])
def test_camera_move_or_changed_receipt_cannot_start_grasp(change):
    decision, _, _ = fixture()
    report = measured_report(decision)
    ledger = report['requests'][0]['ledger']
    if change == 'only_completed': report['measured_target_available'] = False
    elif change == 'preview': report['preview_only'] = True
    elif change == 'failure': report['failure'] = 'rejected'
    elif change == 'ref': decision.status['rgb_acquisition_handoff']['reference_id'] = 'other'
    elif change == 'source': report['handoff']['source_id'] = 'other'
    elif change == 'request': report['handoff']['request_id'] = 'other'
    elif change == 'index': report['handoff']['observation_index'] += 1
    elif change == 'unsettled': ledger['settled_ns'] = '10000000000'
    elif change == 'expired': ledger['deadline_ns'] = '10100000000'
    elif change == 'mono_expired': ledger['deadline_monotonic'] = 1.
    elif change == 'budget': ledger['attempts'] = 4
    elif change == 'selection': ledger['selection'] = dict(decision.selection, generation=12)
    elif change == 'stale_handoff': report['handoff']['source_stamp_ns'] = 9_000_000_000
    elif change == 'authority': report['handoff']['metric_geometry_authority'] = False
    with pytest.raises(ValueError):
        decision.require_measured_handoff(report, decision.selection, 10_100_000_000, 1.)


class ClockCondition:
    def __init__(self):
        self.mono = 1.
        self.ns = 10_100_000_000
        self.on_wait = None
    def __enter__(self): return self
    def __exit__(self, *_): return False
    def wait(self, seconds):
        self.mono += seconds
        self.ns += int(seconds*1e9)
        if self.on_wait: self.on_wait()


def run_fixture(metric=False, change=None):
    decision, status, parameters = fixture(metric)
    clock = ClockCondition()
    selected = deepcopy(decision.selection)
    calls, events = [], []
    def rgb(expected):
        calls.append(('rgb', deepcopy(expected)))
        assert len(calls) == 1
        report = measured_report(decision)
        if change: change(decision, clock, selected, parameters, report)
        calls.append(('rgb_returned',))
        return (2 if report.get('failure') else 0), report, {'path':'exact/result.json'}
    def metric_run(expected):
        calls.append(('metric', deepcopy(expected)))
        return 0
    arguments = dict(condition=clock, selection=lambda: selected, now_ns=lambda:clock.ns,
        monotonic=lambda:clock.mono, parameters=lambda:deepcopy(parameters),
        run_rgb=rgb, run_metric=metric_run, record=events.append, is_shutdown=lambda:False)
    return decision, clock, calls, events, arguments


def test_ready_3d_uses_original_runner_without_rgb_or_preview_wait():
    decision, clock, calls, events, kwargs = run_fixture(metric=True)
    assert dispatch.dispatch_attempt(decision, **kwargs) == 0
    assert [call[0] for call in calls] == ['metric']
    assert clock.mono == 1.


def test_rgb_then_real_fresh_handoff_then_metric_is_strictly_serial():
    decision, clock, calls, events, kwargs = run_fixture()
    assert dispatch.dispatch_attempt(decision, **kwargs) == 0
    assert [call[0] for call in calls] == ['rgb', 'rgb_returned', 'metric']
    assert clock.mono == 1.
    assert any(event['phase'] == 'measured_handoff_verified' for event in events)
    assert 'separate' in events[0]['budget_policy']


def test_metric_arriving_before_rgb_launch_keeps_original_path_priority():
    decision, _, calls, events, kwargs = run_fixture()
    def record(event):
        events.append(event)
        if event['phase'] == 'rgb_entry':
            decision.observe(dict(decision.status, state='ready', target_locked=True))
    kwargs['record'] = record
    assert dispatch.dispatch_attempt(decision, **kwargs) == 0
    assert [call[0] for call in calls] == ['metric']


@pytest.mark.parametrize('phase', ['rgb_entry','measured_handoff_verified','metric_runner_start_requested'])
def test_parameter_change_during_disk_record_rejects_before_next_child(phase):
    decision, _, calls, events, kwargs = run_fixture()
    parameters = kwargs['parameters']()
    kwargs['parameters'] = lambda:deepcopy(parameters)
    def record(event):
        events.append(event)
        if event['phase'] == phase:
            parameters['camera']['width'] += 1
    kwargs['record'] = record
    with pytest.raises(ValueError, match='source_parameters_changed'):
        dispatch.dispatch_attempt(decision, **kwargs)
    assert [call[0] for call in calls] == ([] if phase == 'rgb_entry' else ['rgb','rgb_returned'])


@pytest.mark.parametrize('metric', [False, True])
@pytest.mark.parametrize('change', ['parameters','selection','lost','epoch','source_age'])
def test_last_start_log_cannot_bypass_live_admission(metric, change):
    decision, clock, calls, events, kwargs = run_fixture(metric=metric)
    parameters = kwargs['parameters']()
    kwargs['parameters'] = lambda:deepcopy(parameters)
    def record(event):
        events.append(event)
        if event['phase'] != 'metric_runner_start_requested':
            return
        if change == 'parameters': parameters['camera']['width'] += 1
        elif change == 'selection': kwargs['selection']()['generation'] += 1
        elif change == 'lost': decision.observe(dict(decision.status, target_lost=True))
        elif change == 'epoch': decision.observe_epoch(decision.epoch_ns+1)
        elif change == 'source_age': clock.ns += 800_000_000
    kwargs['record'] = record
    with pytest.raises(ValueError):
        dispatch.dispatch_attempt(decision, **kwargs)
    assert [call[0] for call in calls] == ([] if metric else ['rgb','rgb_returned'])


def test_last_start_log_cannot_replace_bound_measured_handoff():
    decision, _, calls, events, kwargs = run_fixture()
    def record(event):
        events.append(event)
        if event['phase'] == 'metric_runner_start_requested':
            decision.status['rgb_acquisition_handoff']['reference_id'] = 'replacement'
    kwargs['record'] = record
    with pytest.raises(ValueError, match='handoff_binding_changed'):
        dispatch.dispatch_attempt(decision, **kwargs)
    assert [call[0] for call in calls] == ['rgb','rgb_returned']


def test_entry_wait_deadline_does_not_interrupt_existing_rgb_motion_rpc():
    def completed_later(decision, clock, _selected, _parameters, report):
        clock.mono += 40.
        clock.ns += 40_000_000_000
        stamp = clock.ns-100_000_000
        decision.status.update(source_stamp_ns=stamp, color_stamp_ns=stamp, depth_stamp_ns=stamp)
        decision.status['rgb_acquisition_handoff']['source_stamp_ns'] = stamp
        report['handoff']['source_stamp_ns'] = stamp
        report['requests'][0]['ledger']['settled_ns'] = str(stamp-100_000_000)
    decision, _, calls, _, kwargs = run_fixture(change=completed_later)
    assert dispatch.dispatch_attempt(decision, **kwargs) == 0
    assert [call[0] for call in calls] == ['rgb','rgb_returned','metric']


@pytest.mark.parametrize('change', ['selection','epoch','lost','pending','stale','context','only_moved','enable_pending'])
def test_rgb_child_finishes_before_failed_handoff_without_start_or_retry(change):
    def mutation(decision, clock, selected, parameters, report):
        if change == 'selection': selected['generation'] += 1
        elif change == 'epoch': decision.observe_epoch(decision.epoch_ns+1)
        elif change == 'lost': decision.observe(dict(decision.status, target_lost=True))
        elif change == 'pending': decision.observe(dict(decision.status, state='unavailable'))
        elif change == 'stale': clock.ns += 800_000_000
        elif change == 'context': parameters['camera']['width'] += 1
        elif change == 'only_moved': report['measured_target_available'] = False
        elif change == 'enable_pending':
            report['failure'] = 'gateway_rejected'
            report['requests'][0]['message'] = 'MOTION_ENABLE_CONFIRMATION_REQUIRED: PENDING'
    decision, clock, calls, events, kwargs = run_fixture(change=mutation)
    with pytest.raises(ValueError):
        dispatch.dispatch_attempt(decision, **kwargs)
    assert [call[0] for call in calls] == ['rgb','rgb_returned']
    if change == 'enable_pending':
        assert 'PENDING' in events[-1]['request_messages'][0]


def test_no_entry_has_one_finite_deadline_and_no_child():
    decision, clock, calls, _, kwargs = run_fixture()
    decision.status = None
    with pytest.raises(ValueError, match='within_budget'):
        dispatch.dispatch_attempt(decision, **kwargs)
    assert clock.mono == pytest.approx(1.+dispatch.ENTRY_WAIT_SEC)
    assert calls == []


def test_fresh_interest_wakes_wait_without_waiting_for_3d_preview():
    decision, clock, calls, _, kwargs = run_fixture()
    status, decision.status = decision.status, None
    clock.on_wait = lambda: decision.observe(status)
    assert dispatch.dispatch_attempt(decision, **kwargs) == 0
    assert clock.mono == pytest.approx(1.05)
    assert [call[0] for call in calls] == ['rgb','rgb_returned','metric']


@pytest.mark.parametrize('kind', ['rgb_identity_hold','source_qualification'])
@pytest.mark.parametrize('recovers', [True, False])
def test_locked_hold_or_pending_waits_for_producer_recovery_or_loss(kind, recovers):
    decision, clock, calls, _, kwargs = run_fixture(metric=True)
    good = deepcopy(decision.status)
    decision.observe(dict(good, state='unavailable', **{kind:{'state':'pending'}}))
    def next_frame():
        decision.observe(good if recovers else dict(good, state='unavailable', target_lost=True))
    clock.on_wait = next_frame
    if recovers:
        assert dispatch.dispatch_attempt(decision, **kwargs) == 0
        assert [call[0] for call in calls] == ['metric']
    else:
        with pytest.raises(ValueError, match='target_lost'):
            dispatch.dispatch_attempt(decision, **kwargs)
        assert calls == []
    assert clock.mono == pytest.approx(1.05)


def test_post_rgb_transient_hold_can_recover_without_refreshing_handoff_source():
    def hold(decision, clock, _selected, _parameters, _report):
        good = deepcopy(decision.status)
        decision.observe(dict(good, state='unavailable'))
        clock.on_wait = lambda: decision.observe(good)
    decision, clock, calls, _, kwargs = run_fixture(change=hold)
    assert dispatch.dispatch_attempt(decision, **kwargs) == 0
    assert [call[0] for call in calls] == ['rgb','rgb_returned','metric']
    assert clock.mono == pytest.approx(1.05)


def test_post_rgb_hold_without_image_expires_at_original_handoff_age():
    def hold(decision, _clock, _selected, _parameters, _report):
        decision.observe(dict(decision.status, state='unavailable'))
    decision, clock, calls, _, kwargs = run_fixture(change=hold)
    with pytest.raises(ValueError, match='fresh_measured_target_required|handoff_expired'):
        dispatch.dispatch_attempt(decision, **kwargs)
    assert clock.mono == pytest.approx(1.7)
    assert [call[0] for call in calls] == ['rgb','rgb_returned']


def test_exact_child_artifact_path_and_hash(tmp_path):
    root = tmp_path/'archive'
    session = root/'rgb_acquisition_run_test'
    session.mkdir(parents=True)
    (session/'result.json').write_text('{"measured_target_available": false}')
    log = tmp_path/'child.log'
    log.write_text('SESSION '+str(session)+'\n')
    report, artifact = dispatch.acquisition_result(log, root)
    assert report['measured_target_available'] is False
    assert artifact['path'] == str(session/'result.json') and len(artifact['sha256']) == 64
    log.write_text('SESSION '+str(session)+'\nSESSION '+str(session)+'\n')
    with pytest.raises(ValueError, match='exact_rgb_result_path'):
        dispatch.acquisition_result(log, root)


def test_rgb_cli_expected_selection_mismatch_never_calls_gateway(monkeypatch, tmp_path):
    import rospy
    import run_rgb_acquisition as runner
    decision, _, _ = fixture()
    expected = deepcopy(decision.selection)
    selected = dict(expected, generation=12)
    params = {'/grasp_mode/selection':selected, '/robot_description':'model',
              '/robot/joint_min_rad':[-1]*6, '/robot/joint_max_rad':[1]*6}
    monkeypatch.setattr(runner, 'MOUNT', NS(is_mount=lambda:True))
    monkeypatch.setattr(runner, 'ROOT', tmp_path)
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    monkeypatch.setattr(sys, 'argv', ['rgb_runner','--expected-selection-json',json.dumps(expected)])
    monkeypatch.setattr(rospy, 'init_node', lambda *a, **k:None)
    monkeypatch.setattr(rospy, 'Subscriber', lambda *a, **k:NS(unregister=lambda:None))
    monkeypatch.setattr(rospy, 'get_param', lambda key, default=None:deepcopy(params.get(key,default)))
    monkeypatch.setattr(rospy, 'wait_for_service', lambda *a, **k:pytest.fail('service admission after selection change'))
    monkeypatch.setattr(rospy, 'ServiceProxy', lambda *a, **k:pytest.fail('gateway call after selection change'))
    assert runner.main() == 2
    result = json.loads(next(tmp_path.glob('*/result.json')).read_text())
    assert result['requests'] == []
    assert result['failure'] == 'active selection differs from confirmed RGB acquisition'
