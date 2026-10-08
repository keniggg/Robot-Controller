"""Offline evidence/session tests; no ROS master and no motion commands."""
from copy import deepcopy
from dataclasses import replace
import json
import types

import pytest

from test_remote_modes import configured, module, base, Publisher, direct_observation, select_unknown
from alicia_grasp_modes.observe_return_session import parse_session, structured_view_deficit


def setup_observe_return(configured):
    node, params = configured
    select_unknown(params)
    params[module.SELECTION_PARAM]['strategy'] = 'observe_return'
    assert node.reset_mode_cb(None).success
    clock = [10_000_000_000]
    node._ros_source_clock_ns = lambda: clock[0]
    node.near_field_direct_timeout_sec = 60.
    node._observe_return_ack_pub = Publisher()
    assert node.start_streaming()
    observation = direct_observation(node)
    node._latest_target_observation = observation
    node._register_near_field_surface(observation)
    clock[0] = 12_000_000_000
    return node, params, clock


def deficit():
    return dict(tabletop_geometry=dict(plan_phase='CONTACT_EXECUTION_PLAN',
        contact_execution_gate_deferred=False,
        failure_code='BILATERAL_SURFACE_EVIDENCE_MISSING', materialized_count=0))


def prefix_session(node):
    identity = node._current_stream_target_identity()
    plan = base.Grasp6DPlan()
    plan.valid = True
    plan.plan_id = 'far-prefix'
    plan.header.stamp = module.rospy.Time.from_sec(13.)
    plan.target_track_id = identity.track_id
    plan.diagnostic = 'FAR_FIELD_OBSERVATION_PLAN'
    node._observe_return_prefix = plan
    return dict(version=1, session_id='session-test', selection=deepcopy(node._mode_selection),
        phase='observing', stamp_ns=14_000_000_000, deadline_ns=500_000_000_000,
        target_epoch=identity.epoch, target_track_id=identity.track_id,
        origin_plan_id=plan.plan_id, origin_snapshot_stamp_ns=13_000_000_000,
        return_verified_stamp_ns=0, contact_deadline_ns=0)


def send(node, message):
    node._observe_return_session_cb(types.SimpleNamespace(data=json.dumps(message)))
    return json.loads(node._observe_return_ack_pub.messages[-1].data)


def start_session(configured):
    node, params, clock = setup_observe_return(configured)
    initial_surface = node._active_multiview_surface()[0]
    assert node._observe_return_fallback(types.SimpleNamespace(remote_diagnostics=deficit()))
    session = prefix_session(node)
    clock[0] = 15_000_000_000
    ack = send(node, session)
    assert ack['accepted'], ack
    return node, params, clock, session, initial_surface


def append_measured_view(node, ns):
    obs = direct_observation(node, ns)
    node._latest_target_observation = obs
    result = node._register_near_field_surface(obs)
    assert result.ok, result
    return obs


def contact_preview(node, observation):
    plan = base.Grasp6DPlan()
    plan.valid = True
    plan.plan_id = 'contact-at-observation'
    plan.diagnostic = 'CONTACT_EXECUTION_PLAN'
    plan.header.stamp = module.rospy.Time.from_sec(observation.stamp_ns / 1e9)
    plan.target_track_id = observation.identity.track_id
    plan.fused_view_count = len(node._active_multiview_surface()[0].view_stamps_ns)
    node.latest_preview_rich_plan = plan
    return plan


def test_initial_is_current_contact_and_only_structured_visual_failure_falls_back(configured):
    node, _, clock = setup_observe_return(configured)
    identity = node._current_stream_target_identity()
    assert node._direct_strategy_selected()
    assert not node.camera_visibility_gate_enabled
    for code in ('NO_HARD_SAFE_CANDIDATE', 'TABLETOP_CONTACT_PATCH_MISS',
                 'MUJOCO_GRASP_FAILED', 'MOVEIT_IK_FAILED',
                 'CONTACT_SEQUENCE_PRECHECK_FAILED', 'CONTACT_LINEAR_COMPENSATION_FAILED'):
        data = deficit()
        data['tabletop_geometry']['failure_code'] = code
        assert not node._observe_return_fallback(types.SimpleNamespace(remote_diagnostics=data))
    assert node._observe_return_fallback(types.SimpleNamespace(remote_diagnostics=deficit()))
    assert node._observe_return_phase == 'observation_prefix'
    assert node._current_stream_target_identity() == identity
    assert node._observe_return_saved[0].view_stamps_ns == (11_000_000_000,)
    assert node.streaming_enabled and not node.near_field_planning_active
    assert not node._direct_strategy_selected()


def test_observing_ack_is_paused_same_identity_barrier(configured):
    node, _, _, session, initial = start_session(configured)
    assert node._observe_return_paused
    assert node._observe_return_saved[0] is initial
    assert node._current_stream_target_identity() == initial.identity
    assert node._poll_stream_snapshot() is False
    assert node.submit_stream_snapshot(object()) is False
    assert node._stream_ticket_cancellation_code_locked(None) == 'OBSERVE_RETURN_PHASE_PAUSED'
    assert send(node, session)['accepted'] is False  # stale replay


def test_observing_registers_new_reached_view_without_reseeding(configured, monkeypatch):
    node, _, clock, session, initial = start_session(configured)
    clock[0] = 21_000_000_000
    obs = direct_observation(node, 19_000_000_000)
    monkeypatch.setattr(node, '_resolve_reached_view_observation', lambda msg: obs)
    phase = types.SimpleNamespace(active=True, phase_id=1,
        header=types.SimpleNamespace(stamp=module.rospy.Time.from_sec(20.), frame_id='base_link'),
        deadline=module.rospy.Time.from_sec(70.))
    node.near_field_state_cb(phase)
    surface, reference = node._active_multiview_surface()
    assert surface.identity == initial.identity
    assert surface.view_stamps_ns == (11_000_000_000, 19_000_000_000)
    assert reference.stamp_ns == 11_000_000_000
    assert node._observe_return_phase == 'observing_contact'
    assert not node._observe_return_paused
    assert node._latest_registration_evidence.result.ok
    assert 'current_view_only' != node._multiview_surface_audit().get('evidence_kind')
    generation = node._stream_generation
    node.near_field_state_cb(phase)
    assert node._stream_generation == generation


def test_complete_return_keeps_initial_supplemental_and_fresh_return_views(configured):
    node, _, clock, session, initial = start_session(configured)
    node._observe_return_advance('observing_contact', 20_000_000_000,
                               80_000_000_000, active=True, paused=False)
    obs = append_measured_view(node, 22_000_000_000)
    contact_preview(node, obs)
    clock[0] = 24_000_000_000
    returning = dict(session, phase='returning', stamp_ns=23_000_000_000)
    assert send(node, returning)['accepted']
    assert node._observe_return_paused and node._observe_return_phase == 'returning'
    returned = dict(returning, phase='returned', stamp_ns=40_000_000_000,
        return_verified_stamp_ns=39_000_000_000, contact_deadline_ns=90_000_000_000)
    clock[0] = 41_000_000_000
    assert send(node, returned)['accepted']
    assert node._near_field_phase_started_sec == 40.
    assert node._near_field_phase_deadline_sec == 90.
    assert not node._observe_return_paused
    obs = append_measured_view(node, 42_000_000_000)
    assert node._active_multiview_surface()[0].view_stamps_ns == (
        11_000_000_000, 22_000_000_000, 42_000_000_000)
    report = {}
    node._observe_return_audit(report, types.SimpleNamespace(snapshot=types.SimpleNamespace(stamp_ns=obs.stamp_ns)))
    assert report['execution_strategy'] == 'observe_return'
    assert report['planning_origin'] == 'observed_return_contact_snapshot'
    assert report['observe_return']['initial_surface_stamp_ns'] == 11_000_000_000
    assert report['observe_return']['fused_source_stamps_ns'] == [11_000_000_000, 22_000_000_000, 42_000_000_000]
    generation = node._stream_generation
    node.near_field_state_cb(types.SimpleNamespace(active=True))
    assert node._stream_generation == generation
    assert node._near_field_phase_deadline_sec == 90.
    node.stop_streaming()
    assert node._observe_return_session is None
    assert node._observe_return_saved is None


@pytest.mark.parametrize('fault', ['no_contact', 'one_view', 'wrong_track', 'wrong_epoch', 'old_supplement'])
def test_returning_requires_real_current_contact_evidence(configured, fault):
    node, _, clock, session, initial = start_session(configured)
    node._observe_return_advance('observing_contact', 20_000_000_000,
                               80_000_000_000, active=True, paused=False)
    obs = append_measured_view(node, 22_000_000_000)
    plan = contact_preview(node, obs)
    if fault == 'no_contact':
        plan.valid = False
    elif fault == 'one_view':
        plan.fused_view_count = 1
    elif fault == 'wrong_track':
        plan.target_track_id = 'different'
    elif fault == 'wrong_epoch':
        node.target_instance_epoch += 1
    else:
        node._near_field_fused_surface = initial
    clock[0] = 24_000_000_000
    assert not send(node, dict(session, phase='returning', stamp_ns=23_000_000_000))['accepted']
    assert node._observe_return_session['phase'] == 'observing'


@pytest.mark.parametrize('change', [dict(target_epoch=999), dict(origin_plan_id='other'),
                                  dict(phase='returned'), dict(stamp_ns=16_000_000_000),
                                  dict(deadline_ns=900_000_000_000)])
def test_session_strict_rejects_wrong_identity_origin_transition_and_deadline(configured, change):
    node, _, clock = setup_observe_return(configured)
    node._observe_return_fallback(types.SimpleNamespace(remote_diagnostics=deficit()))
    session = prefix_session(node)
    clock[0] = 15_000_000_000
    assert not send(node, dict(session, **change))['accepted']
    assert node._observe_return_session is None


def test_structured_deficit_does_not_treat_existing_candidates_as_view_problem():
    data = deficit()
    data['tabletop_geometry']['materialized_count'] = 1
    assert structured_view_deficit(data) == ''
    data['tabletop_geometry']['materialized_count'] = 0
    assert structured_view_deficit(data) == 'BILATERAL_SURFACE_EVIDENCE_MISSING'


def test_new_view_baseline_filter_applies_without_optional_following_guard(configured, monkeypatch):
    node, _, _ = setup_observe_return(configured)
    node._observe_return_fallback(types.SimpleNamespace(remote_diagnostics=deficit()))
    node.observation_following_guard_required = False
    node.grasp_config = {}
    monkeypatch.setattr(node, '_observe_return_view_gain', lambda sequence: dict(
        ok=True,predicted_new_side_gain_m=.012,depth_uncertainty_m=.003))
    monkeypatch.setattr(node, '_camera_intrinsics', lambda: types.SimpleNamespace(fx=400.,fy=400.))
    variants = tuple(dict(observation_translation_delta_m=value, sequence=types.SimpleNamespace(
        observation_view_audit=dict(working_volume=dict(target_corner_depths_m=[.3]*8))))
                     for value in (None, 0., .0149, .015, .03))
    monkeypatch.setattr(base.RemoteGrasp6DNode, '_far_field_observation_variants',
                        lambda self, *args, **kwargs: (variants, ()))
    admitted, rejected = node._far_field_observation_variants()
    assert [item['observation_translation_delta_m'] for item in admitted] == [.015, .03]
    assert len(rejected) == 3
    node._observe_return_phase = 'initial_contact'
    assert node._far_field_observation_variants() == (variants, ())


def test_accepted_ack_echoes_all_normalized_bindings(configured):
    node, _, clock = setup_observe_return(configured)
    node._observe_return_fallback(types.SimpleNamespace(remote_diagnostics=deficit()))
    session = prefix_session(node)
    clock[0] = 15_000_000_000
    ack = send(node, session)
    assert ack['accepted']
    assert {key: ack[key] for key in session} == session


def test_reached_registration_failure_is_terminal_and_never_erases_initial(configured, monkeypatch):
    node, _, clock, _, initial = start_session(configured)
    clock[0] = 21_000_000_000
    obs = direct_observation(node, 19_000_000_000)
    obs = replace(obs, points_base=obs.points_base + (1., 0., 0.))
    monkeypatch.setattr(node, '_resolve_reached_view_observation', lambda msg: obs)
    phase = types.SimpleNamespace(active=True, phase_id=1,
        header=types.SimpleNamespace(stamp=module.rospy.Time.from_sec(20.), frame_id='base_link'),
        deadline=module.rospy.Time.from_sec(70.))
    node.near_field_state_cb(phase)
    assert node._observe_return_paused
    assert node._active_multiview_surface()[0] is initial
    terminal = node.preview_rich_plan_pub.messages[-1]
    assert not terminal.valid
    assert terminal.diagnostic.startswith('OBSERVE_RETURN_REGISTRATION_FAILED:')


def test_return_ack_waits_for_inflight_worker(configured):
    import threading
    node, _, clock = setup_observe_return(configured)
    node._observe_return_fallback(types.SimpleNamespace(remote_diagnostics=deficit()))
    session = prefix_session(node)
    clock[0] = 15_000_000_000
    entered = threading.Event()
    release = threading.Event()
    def work():
        with node._mode_work_lock:
            entered.set()
            assert release.wait(2.)
    worker = threading.Thread(target=work)
    worker.start()
    assert entered.wait(1.)
    callback = threading.Thread(target=lambda: send(node, session))
    callback.start()
    try:
        assert not node._observe_return_ack_pub.messages
    finally:
        release.set()
        worker.join(2.)
        callback.join(2.)
    assert not callback.is_alive()
    assert json.loads(node._observe_return_ack_pub.messages[-1].data)['accepted']


@pytest.mark.parametrize('fault', ['', 'ik', 'local_mixed', 'stale_audit', 'local_pass', 'terminal', 'server'])
def test_fallback_requires_exact_visual_deficit_request_before_motion_comparison(configured, fault):
    node, _, _ = setup_observe_return(configured)
    prepared = types.SimpleNamespace(ticket=types.SimpleNamespace(request_id=7, generation=8, target_epoch=9),
        snapshot=types.SimpleNamespace(stamp_ns=11_000_000_000), remote_failure_code='')
    report = dict(request_id=7, generation=8, target_epoch=9, snapshot_stamp_ns=11_000_000_000,
        moveit_selection=dict(checked_count=0), pipeline_funnel=dict(
            stage_counts=dict(locally_valid=dict(passed=0)),
            rejection_counts=dict(BILATERAL_SURFACE_EVIDENCE_MISSING=1)))
    code = 'NEAR_FIELD_NO_HARD_SAFE_CANDIDATE'
    if fault == 'ik':
        report['moveit_selection']['checked_count'] = 1
    elif fault == 'local_mixed':
        report['pipeline_funnel']['rejection_counts'].update(
            GRASPNET_STAGE_PROFILE_UNAVAILABLE=18, GRIPPER_SWEEP_COLLISION=3)
    elif fault == 'stale_audit':
        report['request_id'] -= 1
    elif fault == 'local_pass':
        report['pipeline_funnel']['stage_counts']['locally_valid']['passed'] = 1
    elif fault == 'terminal':
        code = 'MUJOCO_GRASP_FAILED'
    elif fault == 'server':
        prepared.remote_failure_code = 'WSL_UNAVAILABLE'
    node._active_gate_audit_report = report
    assert node._observe_return_precheck_visual_failure(prepared, code) is (fault in ('', 'local_mixed'))


@pytest.mark.parametrize('phase', ['observing', 'returning'])
def test_inactive_task_cleans_observation_and_return_session(configured, monkeypatch, phase):
    node, _, _, _, _ = start_session(configured)
    node._observe_return_phase = phase
    node.robot_execution_active = True
    def inactive(self, msg):
        self.robot_execution_active = msg.active
    monkeypatch.setattr(base.RemoteGrasp6DNode, 'grasp_state_cb', inactive)
    node.grasp_state_cb(types.SimpleNamespace(active=False))
    assert not node.streaming_enabled
    assert node._observe_return_session is None
    assert node._observe_return_saved is None
    reply = node.request_plan_cb(types.SimpleNamespace(trigger=True))
    assert reply.success
    assert node.streaming_enabled and not node._observe_return_paused
    assert node._observe_return_phase == 'initial_contact'
