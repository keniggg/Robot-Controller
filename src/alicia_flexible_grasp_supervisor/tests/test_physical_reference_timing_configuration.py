"""Actual launch expansion and bounded physical-reference holds, without ROS I/O.

The original gateway method, trajectory builder and typed lease are exercised.
Only transport, clocks and incoming controller/accepted-reference samples are
fixtures. Expanding a launch reads its configuration; it does not start nodes.
"""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
import rospy
import yaml

from alicia_flexible_grasp.robot.stationary_following import SDK_QUANTUM_RAD
from test_gateway_command_reference import COUNT, node as controller_node
from test_gateway_reference_handoff import setup_reference_action
from test_r22_gateway_reference_independent import gateway
from test_r22_typed_client_independent import case, expired_manual
from test_motion_gateway_controller_start import motion_gateway_node as gateway_module


WORKSPACE = Path(__file__).resolve().parents[3]
TIMEOUT = 'strict_execution_controller_sync_timeout_sec'
DURATION = 'strict_execution_controller_sync_duration_sec'


@pytest.fixture(scope='module')
def expanded_configuration(tmp_path_factory):
    # Use the actual full-stack parser, including its launch order and source
    # checks. No XML/YAML merge reimplementation and no parameter-server query.
    import full_stack_start
    return full_stack_start.configuration(
        session=tmp_path_factory.mktemp('physical-reference-config'))


@pytest.fixture
def robot_configuration(expanded_configuration):
    return {name[len('/robot/'):]: deepcopy(value)
            for name, value in expanded_configuration['parameters'].items()
            if name.startswith('/robot/')}


def test_actual_launch_keeps_bounded_handoff_at_original_base_values(
        expanded_configuration, robot_configuration):
    base = yaml.safe_load((WORKSPACE / 'src/alicia_flexible_grasp_supervisor/'
                           'config/grasp_params.yaml').read_text())['robot']
    assert robot_configuration[TIMEOUT] == base[TIMEOUT]
    assert robot_configuration[DURATION] == base[DURATION]
    assert expanded_configuration['parameters_uploaded'] is False
    assert expanded_configuration['launch'] == str(
        WORKSPACE / '.runtime/full_latest_gen36_single_observation_20261016.launch')
    assert str(WORKSPACE / 'src/alicia_grasp_modes/launch/full_system_modes.launch') in (
        expanded_configuration['launch_source_sha256'])


def transaction(monkeypatch, parameters, manual_cleanup, *, prepare=True):
    """Real lease/hold with two synthetic typed peers and independent receipt."""
    c = expired_manual() if manual_cleanup else case()
    node = gateway(c, monkeypatch)
    node._manual_control_active = lambda: manual_cleanup
    node._observation_sync_hold_evidence = None
    if prepare:
        if manual_cleanup:
            c.lease.manual_cleanup_prepare()
        else:
            c.lease.prepare()
    words = (tuple(c.hardware.overrides[c.lease.QUERY]['sdk_counts'])
             if manual_cleanup else tuple(c.lease.template.baseline_sdk_counts))
    target = tuple((word - 2048) * SDK_QUANTUM_RAD for word in words)
    start = c.clock.mono
    sent_ros = c.clock.ros * 1e-9
    state = NS(c=c, node=node, words=words, target=target, holds=[],
               start=start, receipt_after=parameters[DURATION] + .01,
               controller_fault=None, accepted_checks=0, context_checks=0,
               tick=lambda: None, phase=c.lease.RELEASE_PENDING if manual_cleanup
               else c.lease.PREPARED, manual_cleanup=manual_cleanup)
    node.trajectory_command_pub = NS(get_num_connections=lambda: 1,
        publish=lambda message: state.holds.append(deepcopy(message)))

    def accepted():
        state.accepted_checks += 1
        return list(target)

    def controller(names, **kwargs):
        assert tuple(names) == tuple(node.joint_names[:-1])
        assert kwargs == {'require_stationary': False}
        received = c.clock.mono - start >= state.receipt_after
        result = NS(positions=list(target), velocities=[0.] * 6,
                    accelerations=[0.] * 6,
                    controller_stamp_sec=c.clock.ros * 1e-9 if received else sent_ros)
        if state.controller_fault == 'positions':
            result.positions[0] += 1e-4
        elif state.controller_fault in ('velocities', 'accelerations'):
            getattr(result, state.controller_fault)[0] = 1e-4
        return result

    def advance(seconds):
        c.clock.mono = round(c.clock.mono + seconds, 9)
        c.clock.ros += round(seconds * 1e9)
        # New synthetic SDK/encoder samples retain the identical frozen words;
        # do not treat refreshing a sample as renewing the lease deadlines.
        if manual_cleanup:
            for peer in (c.driver, c.hardware):
                for operation in (c.lease.QUERY, c.lease.RELEASE_COMMIT):
                    peer.overrides[operation].update(
                        sdk_stamp_ns=c.clock.ros - 1, encoder_stamp_ns=c.clock.ros - 1)
        state.tick()

    def context():
        state.context_checks += 1

    node._fresh_accepted_reference = accepted
    node._stationary_controller_reference = controller
    state.context = context
    monkeypatch.setattr(gateway_module.rospy, 'get_param',
        lambda name, default=None: parameters if name == '/robot' else default)
    monkeypatch.setattr(gateway_module.rospy, 'sleep', advance)
    monkeypatch.setattr(gateway_module.rospy, 'is_shutdown', lambda: False)
    c.events.clear()
    return state


def hold(state):
    return state.node._physical_reference_transaction_hold(
        state.c.lease, state.target, state.words, state.phase,
        manual_cleanup=state.manual_cleanup, context_validator=state.context)


def deadlines(lease):
    return (lease.local_deadline, lease.outer_deadline,
            lease.template.deadline_ns, lease.template.episode_deadline_ns)


@pytest.mark.parametrize('manual_cleanup', [False, True], ids=['automatic', 'manual-cleanup'])
def test_expanded_configuration_completes_real_hold_with_fresh_receipt(
        monkeypatch, robot_configuration, manual_cleanup):
    state = transaction(monkeypatch, robot_configuration, manual_cleanup)
    before = deadlines(state.c.lease)
    result = hold(state)
    assert tuple(result.positions) == state.target
    assert len(state.holds) == 1
    point = state.holds[0].points[0]
    assert point.time_from_start.to_sec() == pytest.approx(robot_configuration[DURATION])
    assert tuple(point.positions) == state.target
    assert point.velocities == point.accelerations == [0.] * 6
    assert robot_configuration[DURATION] < state.c.clock.mono - state.start < robot_configuration[TIMEOUT]
    assert state.accepted_checks == state.context_checks >= 3
    assert all(operation == state.c.lease.QUERY for _, operation, _ in state.c.events)
    assert deadlines(state.c.lease) == before
    assert not state.c.lease.released
    if manual_cleanup:
        # Cleanup is permitted after the failed automatic lease expired; this
        # does not reopen or renew its automatic execution authority.
        assert state.start > before[0]


@pytest.mark.parametrize('manual_cleanup', [False, True], ids=['automatic', 'manual-cleanup'])
@pytest.mark.parametrize('field,value', [
    (TIMEOUT, 4.0), (TIMEOUT, 3.000001), (TIMEOUT, float('nan')),
    (TIMEOUT, float('inf')), (TIMEOUT, -1.0), (TIMEOUT, 0.30),
    (DURATION, 0.049), (DURATION, 2.0), (DURATION, float('nan')),
    (DURATION, float('inf')),
], ids=['recorded-four-seconds', 'above-upper-bound', 'timeout-nan', 'timeout-infinite',
        'negative-timeout', 'equal-duration-timeout', 'short-duration',
        'duration-equals-timeout', 'duration-nan', 'duration-infinite'])
def test_invalid_timing_still_rejects_before_any_hold_or_typed_query(
        monkeypatch, robot_configuration, manual_cleanup, field, value):
    state = transaction(monkeypatch, robot_configuration, manual_cleanup)
    robot_configuration[field] = value
    before = deadlines(state.c.lease)
    with pytest.raises(ValueError, match='PHYSICAL_TRANSACTION_TIMING_INVALID'):
        hold(state)
    assert not state.holds and not state.c.events
    assert state.context_checks == state.accepted_checks == 0
    assert state.c.clock.mono == state.start and deadlines(state.c.lease) == before


@pytest.mark.parametrize('manual_cleanup', [False, True], ids=['automatic', 'manual-cleanup'])
@pytest.mark.parametrize('fault', ['owner', 'epoch', 'accepted', 'gated', 'pending',
                                  'stationary', 'sdk-words', 'typed-stale'])
def test_valid_configuration_does_not_bypass_prepublication_evidence(
        monkeypatch, robot_configuration, manual_cleanup, fault):
    state = transaction(monkeypatch, robot_configuration, manual_cleanup)
    node, c = state.node, state.c
    overrides = c.hardware.overrides.setdefault(c.lease.QUERY, {})
    if fault == 'owner':
        node._manual_control_active = lambda: not manual_cleanup
    elif fault == 'epoch':
        node._control_reference_epoch_ns += 1
    elif fault == 'accepted':
        def stale():
            raise ValueError('synthetic accepted reference stale')
        node._fresh_accepted_reference = stale
    elif fault == 'sdk-words':
        overrides['sdk_counts'] = [word + 1 for word in state.words]
    elif fault == 'typed-stale':
        overrides['header'] = NS(stamp=rospy.Time.from_sec(c.clock.ros * 1e-9 - .6))
    else:
        overrides[{'gated': 'output_gated', 'pending': 'pending_complete',
                   'stationary': 'stationary'}[fault]] = False
    with pytest.raises(ValueError):
        hold(state)
    assert not state.holds and not c.lease.released


@pytest.mark.parametrize('manual_cleanup', [False, True], ids=['automatic', 'manual-cleanup'])
@pytest.mark.parametrize('fault', ['late-stamp', 'positions', 'velocities', 'accelerations'])
def test_controller_receipt_must_complete_within_same_configured_timeout(
        monkeypatch, robot_configuration, manual_cleanup, fault):
    state = transaction(monkeypatch, robot_configuration, manual_cleanup)
    if fault == 'late-stamp':
        state.receipt_after = robot_configuration[TIMEOUT] + .01
    else:
        state.controller_fault = fault
    before = deadlines(state.c.lease)
    with pytest.raises(ValueError, match='PHYSICAL_TRANSACTION_HOLD_NOT_CONFIRMED'):
        hold(state)
    assert len(state.holds) == 1
    assert state.c.clock.mono - state.start == pytest.approx(robot_configuration[TIMEOUT])
    assert deadlines(state.c.lease) == before and not state.c.lease.released


def test_automatic_hold_cannot_use_config_timeout_to_extend_shorter_lease(
        monkeypatch, robot_configuration):
    state = transaction(monkeypatch, robot_configuration, False)
    state.c.lease.local_deadline = state.start + .45
    state.receipt_after = .60
    before = deadlines(state.c.lease)
    with pytest.raises(ValueError, match='PHYSICAL_TRANSACTION_HOLD_NOT_CONFIRMED'):
        hold(state)
    assert state.c.clock.mono - state.start == pytest.approx(.45)
    assert len(state.holds) == 1 and deadlines(state.c.lease) == before


@pytest.mark.parametrize('manual_cleanup', [False, True], ids=['automatic', 'manual-cleanup'])
def test_owner_change_after_publication_cannot_confirm_or_republish(
        monkeypatch, robot_configuration, manual_cleanup):
    state = transaction(monkeypatch, robot_configuration, manual_cleanup)
    state.tick = lambda: setattr(state.node, '_control_reference_epoch_ns', -1)
    with pytest.raises(ValueError, match='PHYSICAL_TRANSACTION_OWNER_CHANGED'):
        hold(state)
    assert len(state.holds) == 1 and not state.c.lease.released


def test_manual_cleanup_runs_original_hold_before_same_word_release(
        monkeypatch, robot_configuration):
    state = transaction(monkeypatch, robot_configuration, True, prepare=False)
    before = deadlines(state.c.lease)
    assert state.node._cleanup_manual_physical_reference_lease(
        state.c.lease, context_validator=state.context)
    assert len(state.holds) == 1 and state.c.lease.released
    assert state.node._physical_reference_lease is None
    receipt = state.node._last_physical_manual_cleanup
    assert tuple(receipt['sdk_words']) == state.words
    assert receipt['original_execution_result_preserved'] is True
    assert receipt['automatic_owner_restored'] is False
    assert deadlines(state.c.lease) == before


def configured_reacquisition(node, monkeypatch, parameters, receipt_after):
    """Keep the original feedback validator and the synthetic action transport.

    The shared handoff fixture uses short test-only timing values; replace
    only those inputs with the actual expanded launch configuration here.
    An action result is independently delayed past the requested duration.
    """
    clock, client = setup_reference_action(node, monkeypatch)
    node._test_params['/robot'] = parameters
    node._trajectory_controller_state.desired.positions[0] += COUNT * .1
    clock.selected = tuple(node._trajectory_controller_state.desired.positions)
    client.terminal_observed_at = None

    def action_state():
        if client.goals and clock.now - client.goals[0][0] >= receipt_after - 1e-9:
            if client.terminal_observed_at is None:
                client.terminal_observed_at = clock.elapsed
            return 3
        return 1

    client.get_state = action_state
    return clock, client


@pytest.mark.parametrize('receipt_after', [.34, 1., 1.98])
def test_actual_launch_reacquisition_preserves_raw_controller_reference(
        controller_node, monkeypatch, robot_configuration, receipt_after):
    clock, client = configured_reacquisition(
        controller_node, monkeypatch, robot_configuration, receipt_after)
    sdk_before = list(controller_node._sdk_command_feedback.position)
    assert clock.selected != tuple(sdk_before)
    ok, reason = controller_node._reacquire_stationary_sdk_reference()
    assert ok, reason
    assert len(client.goals) == 1 and not clock.holds and not clock.explicit
    point = client.goals[0][1].trajectory.points[0]
    assert point.time_from_start.to_sec() == pytest.approx(robot_configuration[DURATION])
    assert tuple(point.positions) == clock.selected
    assert point.velocities == point.accelerations == [0.] * 6
    assert robot_configuration[DURATION] < clock.elapsed < robot_configuration[TIMEOUT]
    assert client.terminal_observed_at == pytest.approx(receipt_after)
    # Action ACK and post-completion feedback are independent. The original
    # provider may wait for a newer controller callback within its .10 s
    # budget; this fixture supplies one on its next tick without rebasing q.
    assert client.terminal_observed_at <= clock.elapsed < client.terminal_observed_at + .10
    assert controller_node._sdk_command_feedback.position == sdk_before
    completed = controller_node._completed_command_reference
    assert completed['kind'] == 'completed_stationary_sdk_reacquisition_action'
    assert completed['positions'] == clock.selected
    assert completed['end_sec'] == pytest.approx(clock.start + client.terminal_observed_at)
    assert controller_node._trajectory_controller_state.header.stamp.to_sec() >= completed['end_sec']


def test_actual_launch_reacquisition_keeps_original_deadline_for_late_result(
        controller_node, monkeypatch, robot_configuration):
    clock, client = configured_reacquisition(controller_node, monkeypatch,
        robot_configuration, robot_configuration[TIMEOUT] + .02)
    ok, reason = controller_node._reacquire_stationary_sdk_reference()
    assert not ok and 'reference reacquisition action timed out' in reason
    assert clock.elapsed == pytest.approx(robot_configuration[TIMEOUT])
    assert len(client.goals) == 1 and not clock.holds and not clock.explicit
    assert controller_node._completed_command_reference is None
    # A terminal result arriving after return cannot retroactively authorize
    # the failed transaction or create a replacement goal.
    clock.elapsed = robot_configuration[TIMEOUT] + .04
    clock.refresh()
    assert client.get_state() == 3
    assert controller_node._completed_command_reference is None
    assert len(client.goals) == 1


@pytest.mark.parametrize('field,value', [(TIMEOUT, 4.), (TIMEOUT, 3.000001),
                                       (DURATION, .049)])
def test_reacquisition_rejects_invalid_timing_before_any_action(
        controller_node, monkeypatch, robot_configuration, field, value):
    clock, client = configured_reacquisition(
        controller_node, monkeypatch, robot_configuration, .34)
    robot_configuration[field] = value
    ok, reason = controller_node._reacquire_stationary_sdk_reference()
    assert not ok and reason == 'CONTROLLER_REFERENCE_INVALID: invalid reacquisition deadline'
    assert not client.goals and not clock.holds and not clock.explicit
    assert clock.elapsed == 0. and controller_node._completed_command_reference is None


@pytest.mark.parametrize('fault', ['manual', 'epoch', 'accepted', 'sdk-word',
                                  'controller-q', 'velocity', 'acceleration'])
def test_actual_launch_reacquisition_does_not_bypass_current_evidence(
        controller_node, monkeypatch, robot_configuration, fault):
    clock, client = configured_reacquisition(
        controller_node, monkeypatch, robot_configuration, .34)

    def change(current):
        if current.elapsed < .04:
            return
        if fault == 'manual':
            controller_node._gui_direct_mode = True
        elif fault == 'epoch':
            current.change_epoch()
        elif fault == 'accepted':
            current.stale_source = 'accepted'
            current.refresh()
        elif fault == 'sdk-word':
            current.acknowledge(wrong_word=True)
        elif fault == 'controller-q':
            controller_node._trajectory_controller_state.desired.positions[0] += COUNT * .1
        elif fault == 'velocity':
            controller_node._trajectory_controller_state.desired.velocities[0] = .001
        else:
            controller_node._trajectory_controller_state.desired.accelerations[0] = .001

    clock.on_tick = change
    ok, reason = controller_node._reacquire_stationary_sdk_reference()
    assert not ok and reason.startswith('CONTROLLER_REFERENCE_INVALID:')
    assert len(client.goals) == 1 and not clock.holds and not clock.explicit
    assert controller_node._completed_command_reference is None
    assert clock.elapsed <= .06
