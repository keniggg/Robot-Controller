"""Independent actual-handler deadline checks with a synthetic controller.

The long duration is recorded GEN24 trajectory 04e3a3... (not a new motion).
Planner/physics are explicit doubles; service, contexts and live authority are
real production methods. Fake clocks advance through the complete execution.
"""
from copy import deepcopy
import json
from types import SimpleNamespace as NS

import pytest
import rospy

from test_contact_pregrasp_gateway import case, captured_session, path
from alicia_flexible_grasp.robot import contact_pregrasp_gateway as module


RECORDED_DURATION = 135.853162666


def scenario(monkeypatch, *, duration=RECORDED_DURATION, compute=6.,
             origin_seconds=600., session_seconds=600., bound=True):
    c = case(monkeypatch)
    clock = {'mono': 1000.}
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock['mono'])
    def advance(seconds):
        clock['mono'] += seconds
        c.state['now'] += seconds
    c.advance, c.clock = advance, clock
    if bound:
        origin = captured_session(c, monkeypatch)
        # Fixture setup happens before binding. Runtime replacement is tested
        # separately; the handler still validates the actual captured session.
        origin['deadline'] = clock['mono'] + origin_seconds
        origin['view_session']['deadline_ns'] = int((c.state['now'] + session_seconds)*1e9)
        c.origin = origin
    c.node._validate_frozen_observation_path.return_value = {
        'controller_constraints_snapshot': {'goal_time': 8.}}
    select = c.select.side_effect
    def selection(*args, **kwargs):
        result = select(*args, **kwargs)
        advance(compute)
        return result
    c.select.side_effect = selection
    c.phase_hook = lambda phase: None
    c.authority_samples = []
    c.contexts = []
    bind = c.node._bind_contact_pregrasp_session
    def binding(context):
        bind(context)
        c.contexts.append(context)
    c.node._bind_contact_pregrasp_session = binding
    def execute(goal, execute, **kwargs):
        trajectory = path(kwargs['start_joint_positions'], goal)
        trajectory.joint_trajectory.points[-1].time_from_start = rospy.Duration.from_sec(duration)
        c.planner.observation_path_validator(trajectory)
        c.phase_hook('before_submit')
        kwargs['before_execute']()
        if not c.planner.observation_execution_authorized():
            return False, 'synthetic controller observed authority revoked before send'
        c.calls.append((goal, execute))
        c.state['moves'] += 1
        # Exercise the real still-authorized callback before/after 60 seconds,
        # not merely a planner that instantly returns a claimed success.
        consumed = 0.
        for elapsed in sorted(set([min(30., duration), min(61., duration), duration])):
            advance(elapsed-consumed)
            consumed = elapsed
            c.phase_hook('during_execution')
            accepted = c.planner.observation_execution_authorized()
            c.authority_samples.append((elapsed, accepted))
            if not accepted:
                return False, 'synthetic controller observed authority revoked during execution'
        c.phase_hook('after_execution')
        return True, 'synthetic controller completed the recorded-duration trajectory'
    c.planner.plan_and_execute_joint_probe.side_effect = execute
    return c


def run(c):
    response = c.node.handle_contact_pregrasp(NS(plan=c.plan, execute=True))
    return response, json.loads(response.message)


def test_recorded_136_second_path_survives_compute_60_with_original_outer_lease(monkeypatch):
    c = scenario(monkeypatch)
    before = deepcopy(c.origin['view_session'])
    response, report = run(c)
    assert response.success, report
    assert c.state['moves'] == 1 and len(c.calls) == 1
    assert c.authority_samples == [(30., True), (61., True), (RECORDED_DURATION, True)]
    assert c.origin['deadline'] == 1600. and c.origin['view_session'] == before
    assert report['code'] == 'CONTACT_PREGRASP_EXECUTED'
    assert report['complete_execution_path_checked'] is True
    assert report['endpoint_confirmation_required'] is True
    assert report['real_grasp_success'] is False and report['hardware_response_verified'] is False
    # Nominal travel must not change the separate local-correction context.
    local = c.node._pregrasp_context(deepcopy(c.plan), 0)
    assert local['deadline']-c.clock['mono'] == pytest.approx(60.)


@pytest.mark.parametrize('origin_seconds,session_seconds', [(100., 600.), (600., 100.)])
def test_long_motion_cannot_borrow_past_either_original_outer_clock(monkeypatch, origin_seconds, session_seconds):
    c = scenario(monkeypatch, origin_seconds=origin_seconds, session_seconds=session_seconds)
    response, report = run(c)
    assert not response.success, report
    assert 'INSUFFICIENT_EXECUTION_TIME' in report['reason']
    assert c.state['moves'] == 0 and not c.calls


def test_no_captured_outer_session_does_not_create_long_motion_authority(monkeypatch):
    c = scenario(monkeypatch, bound=False)
    response, report = run(c)
    assert not response.success, report
    assert c.state['moves'] == 0 and not c.calls


def test_long_outer_lease_does_not_extend_60_second_computation(monkeypatch):
    c = scenario(monkeypatch, compute=60.001, duration=2.)
    response, report = run(c)
    assert not response.success, report
    assert c.state['moves'] == 0 and not c.calls


def test_elapsed_transport_setup_rechecks_full_duration_before_submission(monkeypatch):
    c = scenario(monkeypatch, origin_seconds=160.)
    def delay(phase):
        if phase == 'before_submit':
            # 154 s remained during path validation. Setup consumes 9 s;
            # 145 s no longer covers 135.853162666 + 8 + 2 s.
            c.advance(9.)
    c.phase_hook = delay
    response, report = run(c)
    assert not response.success, report
    assert 'INSUFFICIENT_EXECUTION_TIME' in report['reason']
    assert c.state['moves'] == 0 and not c.calls


@pytest.mark.parametrize('clock_kind', ['ros', 'monotonic'])
def test_clock_rollback_during_computation_never_renews_captured_lease(monkeypatch, clock_kind):
    c = scenario(monkeypatch, duration=2., compute=6.)
    original = c.select.side_effect
    def select(*args, **kwargs):
        result = original(*args, **kwargs)
        if clock_kind == 'ros':
            c.state['now'] = 9.
        else:
            c.clock['mono'] = 999.
        return result
    c.select.side_effect = select
    response, report = run(c)
    assert not response.success, report
    assert c.state['moves'] == 0 and not c.calls


@pytest.mark.parametrize('clock_kind', ['ros', 'monotonic'])
def test_small_rollback_after_validation_is_rejected_above_capture_time(monkeypatch, clock_kind):
    c = scenario(monkeypatch, duration=2.)
    def rollback(phase):
        if phase == 'before_submit':
            if clock_kind == 'ros': c.state['now'] -= .1
            else: c.clock['mono'] -= .1
    c.phase_hook = rollback
    response, report = run(c)
    assert not response.success, report
    assert c.state['moves'] == 0 and not c.calls


@pytest.mark.parametrize('fault', ['manual', 'epoch', 'revocation', 'session', 'config'])
def test_final_submit_rechecks_context_after_long_path_admission(monkeypatch, fault):
    c = scenario(monkeypatch)
    def change(phase):
        if phase != 'before_submit':
            return
        if fault == 'manual': c.state['manual'] = True
        elif fault == 'epoch': c.node._control_reference_epoch_ns += 1
        elif fault == 'revocation': c.node._observation_contexts.clear()
        elif fault == 'session': c.origin['view_session']['session_id'] = 'changed-session'
        else: c.params['/grasp']['measured_endpoint_position_tolerance_m'] += .001
    c.phase_hook = change
    response, report = run(c)
    assert not response.success, report
    assert c.state['moves'] == 0 and not c.calls
    assert 'INSUFFICIENT_EXECUTION_TIME' not in report['reason'], report


@pytest.mark.parametrize('fault', ['manual', 'epoch', 'outer_deadline'])
def test_execution_past_60_still_stops_on_real_revocation(monkeypatch, fault):
    c = scenario(monkeypatch)
    def change(phase):
        if phase != 'during_execution' or c.clock['mono'] < 1067.:
            return
        if fault == 'manual': c.state['manual'] = True
        elif fault == 'epoch': c.node._control_reference_epoch_ns += 1
        else: c.clock['mono'] = 1600.
    c.phase_hook = change
    response, report = run(c)
    assert not response.success, report
    assert c.state['moves'] == 1 and len(c.calls) == 1
    assert c.authority_samples == [(30., True), (61., False)]


def test_late_execution_completion_does_not_mask_expired_ros_session(monkeypatch):
    c = scenario(monkeypatch)
    def change(phase):
        if phase == 'after_execution':
            c.state['now'] = c.origin['view_session']['deadline_ns']*1e-9
    c.phase_hook = change
    response, report = run(c)
    assert not response.success, report
    assert c.state['moves'] == 1
    assert report['real_grasp_success'] is False
