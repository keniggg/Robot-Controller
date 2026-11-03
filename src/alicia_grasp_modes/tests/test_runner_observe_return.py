"""Observe/return admission uses explicit provenance and original physical gates."""
from copy import deepcopy
import types

import pytest

from alicia_grasp_modes.runner_contract import observe_return_audit_error
from test_mode_runner import evidence, fake_run, module, working_volume_view


def observe_evidence(phase='initial_contact'):
    selection, report, preview = evidence()
    selection['strategy'] = 'observe_return'
    report['execution_strategy'] = 'observe_return'
    report['target_epoch'] = 7
    preview.target_track_id = 'unknown-target-7'
    preview.candidate_source = 'tabletop_geometry'
    report['observe_return'] = dict(phase=phase, target_epoch=7,
        target_track_id=preview.target_track_id,
        origin_snapshot_stamp_ns=report['snapshot_stamp_ns'],
        initial_surface_stamp_ns=19500000000, trigger_code='')
    if phase == 'observation_prefix':
        report['planning_origin'] = 'observe_return_observation_prefix'
        report['observe_return']['trigger_code'] = 'BILATERAL_SURFACE_EVIDENCE_MISSING'
        report['moveit_selection'] = dict(policy='FIRST_REACHABLE_BY_AUTHORITATIVE_RANK',
            checked_count=2, reachable_count=1)
        selected = report['selected']
        selected['execution_sequence']['kind'] = 'far_field_observation'
        selected['execution_sequence']['stages'] = selected['execution_sequence']['stages'][:1]
        selected['execution_sequence']['stages'][0]['stage'] = 'observation'
        selected['observation_view'] = working_volume_view(report['snapshot_stamp_ns'])
        selected['observation_envelope'] = dict(ok=True, minimum_support_clearance_m=.003)
        selected['moveit']['joint_max_delta_rad'] = .3
        preview.diagnostic = 'FAR_FIELD_OBSERVATION_PLAN'
    elif phase == 'returned_contact':
        report['planning_origin'] = 'observed_return_contact_snapshot'
        report['observation_motion_performed'] = True
        report['snapshot_stamp_ns'] = 24000000000
        preview.header.stamp.secs, preview.header.stamp.nsecs = 24, 0
        report['observe_return'].update(session_id='observe-return-session',
            origin_plan_id='c'*24, return_verified_stamp_ns=23000000000,
            contact_deadline_ns=25000000000, deadline_ns=26000000000,
            fused_source_stamps_ns=[19500000000, 21000000000, 24000000000])
    return selection, report, preview


def runner_for(selection):
    runner = module.ModeAwareGraspRunner()
    runner.selection = selection
    runner.minimum_stamp_ns = 20000000000
    runner.plan_validity_sec = 120.
    runner.expected_model = 'unknown_tabletop'
    return runner


@pytest.mark.parametrize('phase', ['initial_contact', 'observation_prefix'])
def test_each_initial_entry_has_its_own_strict_audit_and_actual_preview_phase(phase):
    selection, report, preview = observe_evidence(phase)
    runner = runner_for(selection)
    assert not observe_return_audit_error(report)
    assert not runner.audit_error(report)
    assert runner.audit_matches_preview(report, preview)
    preview.diagnostic = ('FAR_FIELD_OBSERVATION_PLAN' if phase == 'initial_contact'
                          else 'CONTACT_EXECUTION_PLAN')
    assert not runner.audit_matches_preview(report, preview)


@pytest.mark.parametrize('mutation', ['generation', 'stamp', 'plan_id', 'target_track',
                                     'target_epoch', 'foreign_strategy', 'carton'])
def test_cross_source_selection_or_target_cannot_borrow_valid_geometry(mutation):
    selection, report, preview = observe_evidence()
    runner = runner_for(deepcopy(selection))
    if mutation == 'generation':
        report['grasp_mode_selection']['generation'] += 1
    elif mutation == 'stamp':
        preview.header.stamp.nsecs += 1
    elif mutation == 'plan_id':
        preview.plan_id = 'd'*24
    elif mutation == 'target_track':
        preview.target_track_id = 'another-object'
    elif mutation == 'target_epoch':
        report['observe_return']['target_epoch'] += 1
    elif mutation == 'foreign_strategy':
        report['execution_strategy'] = 'direct'
    elif mutation == 'carton':
        report['grasp_mode_selection']['mode'] = 'carton'
    assert not runner.audit_matches_preview(report, preview)


@pytest.mark.parametrize('phase', ['initial_contact', 'observation_prefix'])
@pytest.mark.parametrize('mutation', ['missing_metadata', 'missing_epoch', 'bool_epoch',
    'missing_initial_source', 'future_initial_source', 'old_initial_source', 'origin_source',
    'unknown_phase', 'wrong_origin', 'motion_already_performed', 'weak_moveit', 'missing_points'])
def test_initial_admission_rejects_incomplete_provenance_and_hard_gate_failures(phase, mutation):
    _, report, _ = observe_evidence(phase)
    meta = report['observe_return']
    if mutation == 'missing_metadata':
        del report['observe_return']
    elif mutation == 'missing_epoch':
        del meta['target_epoch']
    elif mutation == 'bool_epoch':
        meta['target_epoch'] = True
    elif mutation == 'missing_initial_source':
        del meta['initial_surface_stamp_ns']
    elif mutation == 'future_initial_source':
        meta['initial_surface_stamp_ns'] = report['snapshot_stamp_ns'] + 1
    elif mutation == 'old_initial_source':
        meta['initial_surface_stamp_ns'] = report['grasp_mode_selection']['stamp_ns']
    elif mutation == 'origin_source':
        meta['origin_snapshot_stamp_ns'] -= 1
    elif mutation == 'unknown_phase':
        meta['phase'] = 'contact'
    elif mutation == 'wrong_origin':
        report['planning_origin'] = 'two_stage_observation'
    elif mutation == 'motion_already_performed':
        report['observation_motion_performed'] = True
    elif mutation == 'weak_moveit':
        report['selected']['moveit']['collision_free'] = False
    elif mutation == 'missing_points':
        report['replay_geometry']['object_points_count'] = 0
    assert observe_return_audit_error(report)


@pytest.mark.parametrize('trigger', [None, '', 'MOVEIT_IK_FAILED', 'CONTACT_COLLISION',
                                     'NEAR_FIELD_DIRECT_TIMEOUT'])
def test_only_structured_view_insufficiency_can_authorize_an_observation_prefix(trigger):
    _, report, _ = observe_evidence('observation_prefix')
    report['observe_return']['trigger_code'] = trigger
    assert observe_return_audit_error(report)


@pytest.mark.parametrize('mutation', ['view_distance', 'support_clearance', 'aperture_gate',
                                     'final_sequence', 'final_plan_id'])
def test_new_strategy_preserves_original_observation_and_contact_physical_limits(mutation):
    phase = 'observation_prefix' if mutation in ('view_distance', 'support_clearance') else 'initial_contact'
    _, report, _ = observe_evidence(phase)
    selected = report['selected']
    if mutation == 'view_distance':
        selected['observation_view']['actual_camera_target_distance_m'] = .31
    elif mutation == 'support_clearance':
        selected['observation_envelope']['minimum_support_clearance_m'] = .002
    elif mutation == 'aperture_gate':
        selected['final_geometry_gate']['required_open_width_m'] = 0.
    elif mutation == 'final_sequence':
        selected['execution_sequence']['stages'][-1]['linear_from_prior_state'] = False
    elif mutation == 'final_plan_id':
        selected['final_strict_plan_id'] = 'd'*24
    assert observe_return_audit_error(report)


def test_returned_contact_is_internal_continuation_never_a_new_runner_entry():
    selection, report, preview = observe_evidence('returned_contact')
    assert not observe_return_audit_error(report, allow_returned=True)
    assert observe_return_audit_error(report)
    runner = runner_for(selection)
    assert runner.audit_error(report)
    assert not runner.audit_matches_preview(report, preview)


@pytest.mark.parametrize('mutation', ['missing_return', 'return_after_snapshot', 'return_before_origin',
    'missing_session', 'missing_origin_plan', 'expired_contact', 'expired_session',
    'missing_initial_view', 'missing_supplement', 'missing_return_view', 'unregistered_future_view',
    'weak_contact', 'false_motion'])
def test_internal_returned_contact_requires_actual_return_and_registered_views(mutation):
    _, report, _ = observe_evidence('returned_contact')
    meta = report['observe_return']
    if mutation == 'missing_return':
        del meta['return_verified_stamp_ns']
    elif mutation == 'return_after_snapshot':
        meta['return_verified_stamp_ns'] = report['snapshot_stamp_ns']
    elif mutation == 'return_before_origin':
        meta['return_verified_stamp_ns'] = meta['origin_snapshot_stamp_ns']
    elif mutation == 'missing_session':
        del meta['session_id']
    elif mutation == 'missing_origin_plan':
        meta['origin_plan_id'] = ''
    elif mutation == 'expired_contact':
        meta['contact_deadline_ns'] = report['snapshot_stamp_ns']
    elif mutation == 'expired_session':
        meta['deadline_ns'] = meta['contact_deadline_ns'] - 1
    elif mutation == 'missing_initial_view':
        meta['fused_source_stamps_ns'].pop(0)
    elif mutation == 'missing_supplement':
        # An original FAR snapshot is not a reached supplemental view.
        meta['fused_source_stamps_ns'][1] = meta['origin_snapshot_stamp_ns']
    elif mutation == 'missing_return_view':
        meta['fused_source_stamps_ns'].pop()
    elif mutation == 'unregistered_future_view':
        meta['fused_source_stamps_ns'].append(report['snapshot_stamp_ns'] + 1)
    elif mutation == 'weak_contact':
        report['selected']['moveit']['planning_success'] = False
    elif mutation == 'false_motion':
        report['observation_motion_performed'] = False
    assert observe_return_audit_error(report, allow_returned=True)


@pytest.mark.parametrize('phase', ['initial_contact', 'observation_prefix'])
def test_mocked_one_attempt_executes_only_the_exact_selected_initial_plan(monkeypatch, phase):
    runner, params, calls = fake_run(monkeypatch,
        lambda *_: types.SimpleNamespace(success=True, message='done'))
    selection, report, preview = observe_evidence(phase)
    params[module.SELECTION_PARAM] = deepcopy(selection)
    runner.expected_strategy = 'observe_return'
    runner.wait_for_preview = lambda _timeout: preview
    runner.committed_audits[preview.plan_id] = report
    assert runner.run() == 0
    assert calls == [('inference', True), ('start', True, preview.plan_id), ('inference', False)]


@pytest.mark.parametrize('strategy', ['direct', 'two_stage', 'observe_return'])
@pytest.mark.parametrize('phase', ['initial_contact', 'observation_prefix'])
def test_preview_callback_widens_only_observe_return_phase_admission(monkeypatch, strategy, phase):
    selection, _, preview = observe_evidence(phase)
    selection['strategy'] = strategy
    runner = runner_for(selection)
    monkeypatch.setattr(module.rospy, 'get_param', lambda *_: selection)
    monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(lambda: module.rospy.Time(21)))
    content_checks = []
    monkeypatch.setattr(module, 'plan_id_matches_content',
        lambda message: content_checks.append(message) or True)
    runner.on_preview(preview)
    allowed = strategy == 'observe_return' or (strategy == 'direct') == (phase == 'initial_contact')
    assert (runner.preview is preview) == allowed
    assert bool(content_checks) == allowed


@pytest.mark.parametrize('strategy', ['direct', 'two_stage', 'observe_return'])
def test_final_near_field_terminal_ends_direct_and_observe_return_wait(monkeypatch, strategy):
    selection, _, terminal = observe_evidence()
    selection['strategy'] = strategy
    runner = runner_for(selection)
    terminal.valid = False
    terminal.candidate_source = 'near_field_terminal'
    terminal.diagnostic = 'NEAR_FIELD_DIRECT_TIMEOUT'
    monkeypatch.setattr(module.rospy, 'get_param', lambda *_: selection)
    monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(lambda: module.rospy.Time(21)))
    runner.on_preview(terminal)
    assert bool(runner.terminal_reason) == (strategy != 'two_stage')
    assert runner.preview is None
