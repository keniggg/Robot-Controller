"""Task SDK ingress under real serial TCPROS dispatch and held plan ownership."""
from copy import deepcopy
import threading
from types import SimpleNamespace as NS

import pytest
from rospy.topics import _SubscriberImpl
from sensor_msgs.msg import JointState

from test_task_modes import configured, module
from test_station_compute_wait_task import station, frame, deliver, at_ns, feedback
from alicia_grasp_modes.observe_return_task import ObserveReturnTask
from alicia_grasp_modes import station_compute_wait_task as wait


def test_real_task_serial_feedback_returns_under_held_planning_lock(station):
    c = station
    node = c.node
    deliver(c, frame(c))
    contract = deepcopy(node._station_task_record['contract'])
    c.now[0] = 100_900_000_000
    messages = []
    # The older first frame is legal delivery. A blocked callback must not
    # prevent the two already-available fresh frames from entering history.
    for ns in (100_020_000_000, 100_800_000_000, 100_900_000_000):
        msg = deepcopy(node.latest_accepted_joint_state)
        msg.header.stamp = at_ns(ns)
        messages.append(msg)
    entered = threading.Event()
    finished = threading.Event()
    errors = []
    def callback(msg):
        entered.set()
        node.accepted_joint_cb(msg)
    transport = NS(callbacks=[(callback, None)], statistics_logger=None,
                   _invoke_callback=lambda msg, cb, args: cb(msg))
    def receive():
        try:
            _SubscriberImpl.receive_callback(transport, messages, None)
        except BaseException as exc:
            errors.append(exc)
        finally:
            finished.set()
    worker = threading.Thread(target=receive, daemon=True)
    with node._grasp6d_plan_guard():
        worker.start()
        assert entered.wait(1.)
        completed_while_locked = finished.wait(.2)
        with node._following_evidence_lock:
            latest = max(msg.header.stamp.to_nsec()
                         for msg in node._following_accepted_history)
        try:
            node._station_task_context(contract)
            gate_reason = None
        except ValueError as exc:
            gate_reason = str(exc)
    worker.join(1.)
    assert not worker.is_alive() and not errors
    assert completed_while_locked, 'Task TCPROS blocked behind planning lock; gate=%s; latest=%s' % (gate_reason, latest)
    assert latest == c.now[0]
    assert gate_reason is None


def accepted(ns, delta=0.):
    message = JointState()
    message.header.stamp = at_ns(ns)
    message.header.frame_id = 'sdk_measured'
    message.name = ['Joint%d' % i for i in range(1, 7)] + ['right_finger']
    message.position = [delta] + [0.] * 5 + [.05]
    return message


def serial_receive(node, messages):
    transport = NS(callbacks=[(node.accepted_joint_cb, None)], statistics_logger=None,
                   _invoke_callback=lambda msg, cb, args: cb(msg))
    _SubscriberImpl.receive_callback(transport, messages, None)


def receive_while_plan_locked(node, messages):
    done = threading.Event()
    errors = []
    def receive():
        try:
            serial_receive(node, messages)
        except BaseException as exc:
            errors.append(exc)
        finally:
            done.set()
    worker = threading.Thread(target=receive, daemon=True)
    with node._grasp6d_plan_guard():
        worker.start()
        finished = done.wait(1.)
    worker.join(1.)
    assert finished and not worker.is_alive(), 'serial SDK ingress waited for planning'
    assert not errors


def test_available_plan_lock_does_not_move_join_or_parameter_io_into_ingress(station, monkeypatch):
    c = station
    deliver(c, frame(c))
    original = deepcopy(c.node._station_task_record)
    def forbidden(*args, **kwargs):
        raise AssertionError('SDK ingress invoked join or parameter I/O')
    with monkeypatch.context() as patch:
        patch.setattr(c.node, '_station_task_join', forbidden)
        patch.setattr(module.rospy, 'get_param', forbidden)
        serial_receive(c.node, [accepted(100_030_000_000), accepted(100_040_000_000)])
    assert c.node.latest_accepted_joint_state.header.stamp.to_nsec() == 100_040_000_000
    assert c.node._station_task_record == original
    joins = []
    monkeypatch.setattr(c.node, '_station_task_join', lambda: joins.append(True))
    c.node._station_compute_task_tick()
    assert joins == [True]
    c.node._station_task_context(original['contract'])


@pytest.mark.parametrize('delta', [.005, -.01, .2])
def test_motion_and_return_while_planning_is_locked_never_authorizes_action(station, delta):
    c = station
    deliver(c, frame(c))
    receive_while_plan_locked(c.node, [accepted(100_030_000_000, delta),
                                     accepted(100_040_000_000)])
    invoked = []
    result, _ = c.node._invoke_plan_bound_action(
        c.plan, {}, 'supplemental observation rebind', lambda: invoked.append(True))
    assert not result.ok and not invoked
    assert c.node._station_task_retired and c.node._execution_authority_revoked
    assert 'station_task_pose_changed' in c.node._unknown_target_loss['reason']


@pytest.mark.parametrize('kind', ['nan', 'duplicate_names', 'missing_joint', 'zero_stamp'])
def test_malformed_sdk_before_a_good_return_is_rejected_at_final_gate(station, kind):
    c = station
    deliver(c, frame(c))
    bad = accepted(100_030_000_000)
    if kind == 'nan':
        bad.position[1] = float('nan')
    elif kind == 'duplicate_names':
        bad.name[1] = bad.name[0]
    elif kind == 'missing_joint':
        bad.name[1] = 'other_joint'
    else:
        bad.header.stamp = at_ns(0)
    receive_while_plan_locked(c.node, [bad, accepted(100_040_000_000)])
    c.node._station_compute_task_tick()
    assert c.node._station_task_retired
    assert 'station_task_invalid_feedback' in c.node._unknown_target_loss['reason']
    invoked = []
    result, _ = c.node._invoke_plan_bound_action(
        c.plan, {}, 'supplemental observation rebind', lambda: invoked.append(True))
    assert not result.ok and not invoked


def test_unconsumed_overflow_does_not_turn_evicted_motion_into_stationarity(station):
    c = station
    deliver(c, frame(c))
    receive_while_plan_locked(c.node,
        [accepted(100_030_000_000, .02)] +
        [accepted(100_031_000_000 + i * 100_000) for i in range(129)])
    assert all(message.position[0] == 0. for message in c.node._following_accepted_history)
    c.node._station_compute_task_tick()
    assert c.node._station_task_retired
    assert 'station_task_feedback_buffer_overrun' in c.node._unknown_target_loss['reason']


@pytest.mark.parametrize('kind,reason', [
    ('motion', 'station_task_pose_changed'),
    ('invalid', 'station_task_invalid_feedback'),
    ('overrun', 'station_task_feedback_buffer_overrun')])
def test_failed_context_remains_sticky_after_later_good_rows_and_ready(station, kind, reason):
    c = station
    deliver(c, frame(c))
    contract = deepcopy(c.node._station_task_record['contract'])
    first = accepted(100_025_000_000, .02 if kind == 'motion' else 0.)
    if kind == 'invalid':
        first.position[2] = float('nan')
    messages = [first]
    if kind == 'overrun':
        messages += [accepted(100_026_000_000 + i * 10_000) for i in range(129)]
    receive_while_plan_locked(c.node, messages)
    with c.node._grasp6d_plan_guard():
        with pytest.raises(ValueError, match='^' + reason + '$'):
            c.node._station_task_context(contract)
    # This consumes/evicts the contradicting row from BOTH ordinary histories.
    # A previous context exception must not convert a cursor advance to proof.
    receive_while_plan_locked(c.node,
        [accepted(100_030_000_000 + i * 100_000) for i in range(128)])
    with c.node._grasp6d_plan_guard():
        with pytest.raises(ValueError, match='^' + reason + '$'):
            c.node._station_task_context(contract)
    c.now[0] = 100_150_000_000
    receive_while_plan_locked(c.node, [accepted(100_090_000_000),
                                     accepted(100_110_000_000)])
    deliver(c, frame(c, 100_100_000_000))
    assert c.node._station_task_retired and c.node._station_task_record is None
    # A received object can still be displayed. It has no station receipt or
    # execution permission after the original fault has retired its owner.
    invoked = []
    result, _ = c.node._invoke_plan_bound_action(
        c.plan, {}, 'supplemental observation rebind', lambda: invoked.append(True))
    assert not result.ok and not invoked and c.node._execution_authority_revoked


@pytest.mark.parametrize('fault', ['manual', 'camera', 'selection', 'epoch', 'stale', 'future'])
def test_local_feedback_opportunity_never_replaces_full_final_context(station, fault):
    c = station
    deliver(c, frame(c))
    if fault == 'manual':
        c.params['/gui/joint_direct_mode'] = True
    elif fault == 'camera':
        c.params['/camera']['fx'] += 1.
    elif fault == 'selection':
        c.params[module.SELECTION_PARAM] = dict(c.selection, generation=4)
    elif fault == 'epoch':
        c.node._following_epoch_ns += 1
    elif fault == 'stale':
        c.now[0] = 101_700_000_000
    message = accepted(100_040_000_000 if fault != 'future' else 100_060_000_000)
    serial_receive(c.node, [message])
    invoked = []
    result, _ = c.node._invoke_plan_bound_action(
        c.plan, {}, 'supplemental observation rebind', lambda: invoked.append(True))
    assert not result.ok and not invoked
    assert c.node._station_task_retired


def qualified_request(c):
    request = deepcopy(c.node._observation_measurement_request)
    request['settle'] = dict(accepted_positions_rad=[0.] * 6)
    return request


def test_qualified_station_excludes_legal_observation_movement_before_its_source(station, monkeypatch):
    c = station
    # The arm was moving before this qualified station's measured source.
    serial_receive(c.node, [accepted(98_000_000_000, .8), accepted(100_020_000_000)])
    published = []
    monkeypatch.setattr(module.rospy, 'is_shutdown', lambda: False)
    monkeypatch.setattr(ObserveReturnTask, '_observation_measurement_publish',
                        lambda self, request, **kw: published.append(deepcopy(request)))
    c.node._observation_measurement_publish(qualified_request(c))
    assert len(published) == 1
    deliver(c, frame(c))
    assert c.node._station_task_record is not None and not c.node._station_task_retired
    c.node._station_task_context(c.node._station_task_record['contract'])


def test_qualified_cursor_is_registered_before_participant_ack_wait(station, monkeypatch):
    c = station
    entered = threading.Event()
    release = threading.Event()
    errors = []
    monkeypatch.setattr(module.rospy, 'is_shutdown', lambda: False)
    def ack_wait(self, request, **kwargs):
        entered.set()
        assert release.wait(2.)
        return []
    monkeypatch.setattr(ObserveReturnTask, '_observation_measurement_publish', ack_wait)
    request = qualified_request(c)
    def publish():
        try:
            c.node._observation_measurement_publish(request)
        except BaseException as exc:
            errors.append(exc)
    worker = threading.Thread(target=publish, daemon=True)
    worker.start()
    try:
        assert entered.wait(1.)
        receive_while_plan_locked(c.node,
            [accepted(100_025_000_000, .02)] +
            [accepted(100_030_000_000 + i * 100_000) for i in range(129)])
    finally:
        release.set()
        worker.join(1.)
    assert not worker.is_alive() and not errors
    deliver(c, frame(c))
    assert c.node._station_task_record is None
    with c.node._grasp6d_plan_guard():
        with pytest.raises(ValueError, match='station_task_feedback_buffer_overrun'):
            c.node._station_task_feedback_pose(request, [0.] * 6)


def test_inflight_callback_keeps_old_journal_across_real_selection_and_epoch_reset(station, monkeypatch):
    c = station
    old = c.node._station_task_feedback
    entered = threading.Event()
    release = threading.Event()
    ingest = old.ingest
    errors = []
    def delayed_ingest(*args):
        entered.set()
        assert release.wait(2.)
        return ingest(*args)
    monkeypatch.setattr(old, 'ingest', delayed_ingest)
    def receive():
        try:
            serial_receive(c.node, [accepted(100_030_000_000, .02)])
        except BaseException as exc:
            errors.append(exc)
    worker = threading.Thread(target=receive, daemon=True)
    worker.start()
    try:
        assert entered.wait(1.)
        c.node.active = False
        c.params[module.SELECTION_PARAM] = dict(c.selection,
            generation=c.selection['generation'] + 1, stamp_ns='101000000000')
        assert c.node.reset_mode_cb(None).success
        c.node.following_epoch_cb(NS(stamp=at_ns(101_000_000_000),
                                    frame_id='sdk_reference_epoch'))
        new = c.node._station_task_feedback
        assert new is not old and list(new) == []
    finally:
        release.set()
        worker.join(1.)
    assert not worker.is_alive() and not errors
    assert list(new) == []
    assert any(ns == 100_030_000_000 for ns, _ in old)
    c.now[0] = 101_050_000_000
    c.node._mode_bound_selection = deepcopy(c.node._mode_selection)
    serial_receive(c.node, [accepted(101_040_000_000)])
    assert [ns for ns, _ in new] == [101_040_000_000]


def test_requalification_feedback_opportunity_has_no_join_or_parameter_io(station, monkeypatch):
    from test_station_requalification_task_finalize import enrolled, DELTA
    c = station
    hold = enrolled(c)
    def forbidden(*args, **kwargs):
        raise AssertionError('requalification ingress invoked join or parameter I/O')
    with monkeypatch.context() as patch:
        patch.setattr(c.node, '_station_task_join', forbidden)
        patch.setattr(module.rospy, 'get_param', forbidden)
        serial_receive(c.node, [accepted(100_090_000_000, DELTA)])
    assert c.node._station_requalification_task is hold
    c.params['/gui/joint_direct_mode'] = True
    invoked = []
    result, _ = c.node._invoke_plan_bound_action(
        c.plan, {}, 'supplemental observation rebind', lambda: invoked.append(True))
    assert not result.ok and not invoked
    assert c.node._station_task_retired


@pytest.mark.parametrize('broken_logger', [False, True])
def test_retirement_keeps_bounded_ingress_timing_without_granting_authority(
        station, monkeypatch, broken_logger):
    c = station
    deliver(c, frame(c))
    receive_while_plan_locked(c.node, [accepted(100_030_000_000, .02),
                                     accepted(100_040_000_000)])
    captured = []
    monkeypatch.setattr(wait, 'log_task_retirement',
                        lambda node, reason, before: captured.append(deepcopy(before)))
    if broken_logger:
        # The diagnostic is best effort; even an instrumentation failure must
        # leave the original retirement and authority revocation effective.
        original = wait.task_station_snapshot
        class RejectAdditionalDiagnostic(dict):
            def __setitem__(self, key, value):
                if key == 'accepted_feedback_ingress':
                    raise RuntimeError('diagnostic unavailable')
                super().__setitem__(key, value)
        monkeypatch.setattr(wait, 'task_station_snapshot',
                            lambda node: RejectAdditionalDiagnostic(original(node)))
    c.node._station_compute_task_tick()
    assert c.node._station_task_retired and c.node._execution_authority_revoked
    assert captured
    if not broken_logger:
        diagnostic = captured[-1]['accepted_feedback_ingress']
        assert diagnostic['latest_source_stamp_ns'] == 100_040_000_000
        assert diagnostic['latest_receive_ros_ns'] == c.now[0]
        assert diagnostic['validation_now_ns'] == c.now[0]
        assert diagnostic['latest_source_age_ns'] == 10_000_000
        assert diagnostic['buffer_count'] <= 128 and diagnostic['sequence'] >= 4


@pytest.mark.parametrize('selection', [
    None,
    dict(mode='unknown', strategy='direct'),
    dict(mode='unknown', strategy='two_stage'),
    dict(mode='carton', strategy='direct'),
    dict(mode='carton', strategy='observe_return')])
def test_unowned_nonstation_callbacks_keep_original_hooks_without_new_clock_or_journal(
        configured, monkeypatch, selection):
    from test_support_source_qualification import install
    node, _ = configured
    node._mode_bound_selection = deepcopy(selection)
    install(node, [], [])
    calls = []
    preflight = node._initial_preflight_feedback
    support = node._refresh_unknown_support_sources
    def before(message):
        calls.append('preflight')
        return preflight(message)
    def refresh():
        calls.append('support')
        return support()
    def forbidden(*args, **kwargs):
        raise AssertionError('unowned nonstation callback requested station clock/journal')
    monkeypatch.setattr(node, '_initial_preflight_feedback', before)
    monkeypatch.setattr(node, '_refresh_unknown_support_sources', refresh)
    monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(forbidden))
    monkeypatch.setattr(node._station_task_feedback, 'ingest', forbidden)
    message = accepted(100_030_000_000)
    serial_receive(node, [message])
    assert calls == ['preflight', 'support']
    assert node.latest_accepted_joint_state == message
    assert node._following_accepted_history[-1] == message
    assert list(node._station_task_feedback) == []


@pytest.mark.parametrize('active,stage', [(True, 3), (True, 2), (False, 0)])
def test_bound_observation_ingress_covers_motion_and_prequalification_without_plan_or_stage_gate(
        station, active, stage):
    c = station
    node = c.node
    assert node._mode_bound_selection['mode'] == 'unknown'
    assert node._mode_bound_selection['strategy'] == 'observe_return'
    assert node._station_task_record is None and node._station_requalification_task is None
    node.active = active
    node.stage = stage
    node._bound_execution_plan = None
    node._observation_measurement_request = None
    node._observe_return_session = None
    serial_receive(node, [accepted(100_030_000_000, .2), accepted(100_040_000_000)])
    rows = dict(node._station_task_feedback)
    assert rows[100_030_000_000][0] == .2
    assert rows[100_040_000_000][0] == 0.
    assert node._station_task_record is None and not node._station_task_retired


@pytest.mark.parametrize('kind', ['record', 'requalification'])
@pytest.mark.parametrize('selection', [
    None,
    dict(mode='unknown', strategy='direct'),
    dict(mode='carton', strategy='observe_return')])
def test_existing_station_owner_keeps_collecting_after_selection_change_and_final_gate_rejects(
        station, kind, selection):
    c = station
    if kind == 'record':
        deliver(c, frame(c))
    else:
        from test_station_requalification_task_finalize import enrolled
        enrolled(c)
    journal = c.node._station_task_feedback
    c.node._mode_bound_selection = deepcopy(selection)
    source = c.now[0]
    serial_receive(c.node, [accepted(source, .02)])
    assert dict(journal)[source][0] == .02
    assert c.node._station_task_retired and c.node._execution_authority_revoked
    invoked = []
    result, _ = c.node._invoke_plan_bound_action(
        c.plan, {}, 'supplemental observation rebind', lambda: invoked.append(True))
    assert not result.ok and not invoked
