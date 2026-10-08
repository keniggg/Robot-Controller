"""A source-bound diagnostic observation never becomes a grasp or retry."""
from copy import deepcopy
import json
from pathlib import Path
import threading
import types

import pytest

from test_observe_return_task import context, audit_reference, at_ns, begin
from test_task_modes import module, base
from test_silhouette_observation_remote import diagnostic_report
from alicia_grasp_modes import observe_return_task as task


@pytest.fixture
def diagnostic(context, monkeypatch):
    c = context
    c.report = diagnostic_report()
    c.node._observe_return_audit_cb(audit_reference(c.tmp_path, c.report))
    monkeypatch.setattr(task, '_EXPERIMENT_ROOT', str(c.tmp_path))
    monkeypatch.setattr(task.os.path, 'ismount', lambda _path: True)
    c.events = []
    c.fault = {'at': None}
    c.config = dict(observation_camera_target_range_check_enabled=True,
        measured_endpoint_check_enabled=True, contact_endpoint_precision_enabled=True,
        pregrasp_cartesian_compensation_enabled=True,
        observation_camera_target_retreat_correction_enabled=True)
    c.node._validate_bound_plan = lambda *_: base.PlanValidationResult(True)
    c.node._invoke_plan_bound_action = lambda _plan, _cfg, _label, action: (
        base.PlanValidationResult(True), action())
    c.node._execution_checkpoint = lambda _plan, _cfg, label: (
        c.events.append(('checkpoint', label)) or c.fault['at'] != label)
    c.node._position_only_execute_globally_enabled = lambda: False

    def forbidden(*_args, **_kwargs):
        pytest.fail('diagnostic attempted gripper/contact/return/correction')
    c.forbidden = forbidden
    c.node._maybe_execute_observation_camera_retreat = forbidden
    c.node._converge_far_field_observation_endpoint = forbidden
    c.node._command_gripper_position = forbidden
    c.node._compensate_pregrasp_endpoint = forbidden
    c.node._set_contact_endpoint_precision = forbidden
    c.node._set_near_field_active = forbidden
    monkeypatch.setattr(base.GraspTaskNode, '_execute_grasp6d_plan', forbidden)

    def preflight(pose, execute):
        c.events.append(('preflight', execute))
        assert not execute
        return types.SimpleNamespace(success=c.fault['at'] != 'preflight', message='strict preview')
    def execute(pose, execute):
        c.events.append(('motion', execute))
        assert execute
        return types.SimpleNamespace(success=c.fault['at'] != 'motion', message='strict execution')
    def settle(_reason):
        c.events.append(('settle', True))
        c.now[0] = 24_000_000_000
        if c.fault['at'] == 'settle': return False
        c.node._last_motion_settle_sample = dict(epoch_ns=20_000_000_000,
            accepted_stamp_ns=23_950_000_000, stationary_window_start_ns=23_050_000_000,
            stationary_window_end_ns=23_950_000_000, accepted_sample_count=10,
            accepted_positions_rad=[0.] * 7, joint_peak_to_peak_rad=[.0001] * 7)
        return True
    def endpoint(*_args, **kwargs):
        c.events.append(('endpoint', kwargs['required']))
        assert not kwargs['required']
        return True
    def envelope(*_args):
        c.events.append(('envelope', True))
        c.node._last_measured_observation_envelope_sample = dict(
            plan_id=c.plan.plan_id, ok=True, required_support_clearance_m=.002)
        return c.fault['at'] != 'envelope'
    def fresh(_plan, _cfg, cutoff):
        c.events.append(('fresh', cutoff))
        c.now[0] = 24_300_000_000
        stamp = cutoff - 1 if c.fault['at'] == 'old_source' else cutoff + 200_000_000
        c.node._last_observation_camera_range_evidence = dict(
            image_source_stamp_ns=stamp, observation_stamp_ns=cutoff + 250_000_000,
            source_stamp_ns=cutoff + 250_000_000, distance_m=.25,
            target_position_m=[0., 0., .02], camera_position_m=[0., -.15, .22])
        return base.PlanValidationResult(c.fault['at'] != 'range',
            'OBSERVATION_CAMERA_TARGET_OUT_OF_RANGE', 'outside original interval')
    c.preflight, c.execute = preflight, execute
    c.node._wait_for_motion_settle = settle
    c.node._record_and_validate_measured_endpoint = endpoint
    c.node._validate_measured_observation_envelope = envelope
    c.node._wait_for_fresh_observation_camera_target_range = fresh
    monkeypatch.setattr(base, 'split_rich_plan_poses', lambda _: ('observation', None, None, None))
    return c


def run(c):
    return c.node._execute_grasp6d_plan(c.config, {}, .05, c.preflight,
        c.forbidden, c.forbidden, c.forbidden, c.plan, strict_execute_pose=c.execute)


def test_one_real_shared_executor_prefix_saves_diagnostic_source_without_gripper_contact_or_return(diagnostic):
    c = diagnostic
    assert run(c), c.states[-1]
    assert [event for event in c.events if event[0] in ('preflight', 'motion', 'settle', 'endpoint', 'envelope', 'fresh')] == [
        ('preflight', False), ('motion', True), ('settle', True), ('endpoint', False),
        ('envelope', True), ('fresh', 24_000_000_000)]
    assert [p['phase'] for p in c.publications] == ['observing']
    assert c.gateways == [('/supervisor/capture_observe_return_origin', c.plan.plan_id, True)]
    reference = c.node._diagnostic_observation_sample_reference
    marker = json.loads(Path(reference['path']).read_text())
    assert marker['source_stamp_ns'] == 24_200_000_000
    assert marker['camera_range']['observation_stamp_ns'] == 24_250_000_000
    assert marker['sampling_after_ns'] == 24_000_000_000
    assert marker['grasp_completed'] is marker['contact_and_return_authorized'] is False
    assert marker['gripper_command_submitted'] is marker['additional_motion_submitted'] is False
    assert marker['sampling_marker_only'] is True
    assert marker['original_audit_reference'] == c.node._observe_return_bound_reference
    assert c.states[-1][0] == base.GraspStages.SUCCESS
    assert c.states[-1][1].startswith('DIAGNOSTIC_OBSERVATION_COMPLETE:')


@pytest.mark.parametrize('fault', ['preflight', 'motion', 'settle', 'envelope', 'range',
    'old_source', 'diagnostic observation arrival', 'diagnostic observation sampling'])
def test_any_failure_is_terminal_without_correction_contact_or_success_marker(diagnostic, fault):
    c = diagnostic
    c.fault['at'] = fault
    assert not run(c)
    assert c.node._diagnostic_observation_sample_reference is None
    assert len([event for event in c.events if event[0] == 'motion']) <= 1
    assert not list(c.tmp_path.glob('diagnostic_observation_*.json'))
    assert [p['phase'] for p in c.publications] == ['observing']
    assert len(c.gateways) == 1


@pytest.mark.parametrize('fault', ['support_loss', 'manual', 'epoch', 'phase', 'generation', 'unmounted',
                                   'outside_storage', 'tampered_audit', 'range_disabled'])
def test_invalid_context_or_storage_prevents_first_motion(diagnostic, monkeypatch, fault):
    c = diagnostic
    begin(c)
    if fault == 'support_loss':
        c.node._unknown_target_loss = dict(selection=deepcopy(c.selection),
            reason='TARGET_LOST: support_plane_local_distance_inconsistent')
    elif fault == 'manual': c.fault['at'] = 'diagnostic observation entry'
    elif fault == 'epoch': c.node._observe_return_session['target_epoch'] += 1
    elif fault == 'phase': c.node._observe_return_session['phase'] = 'returned'
    elif fault == 'generation': c.params[module.SELECTION_PARAM] = dict(c.selection, generation=99)
    elif fault == 'unmounted': monkeypatch.setattr(task.os.path, 'ismount', lambda _: False)
    elif fault == 'outside_storage': monkeypatch.setattr(task, '_EXPERIMENT_ROOT', str(c.tmp_path/'other'))
    elif fault == 'tampered_audit': Path(c.node._observe_return_bound_reference['path']).write_text('{}')
    elif fault == 'range_disabled': c.config['observation_camera_target_range_check_enabled'] = False
    assert not c.node._execute_diagnostic_observation(c.plan, c.config, 'observation',
        c.preflight, c.execute, base.write_mujoco_execution_audit)
    assert not any(e[0] == 'motion' for e in c.events)
    assert c.node._diagnostic_observation_sample_reference is None


def test_failed_once_cannot_be_retried_from_same_bound_source(diagnostic):
    c = diagnostic
    c.fault['at'] = 'range'
    assert not run(c)
    c.fault['at'] = None
    assert not c.node._execute_diagnostic_observation(c.plan, c.config, 'observation',
        c.preflight, c.execute, base.write_mujoco_execution_audit)
    assert len([e for e in c.events if e[0] == 'motion']) == 1
    assert 'ALREADY_ATTEMPTED' in c.states[-1][1]


def test_contact_freeze_defense_rejects_diagnostic_even_if_unexpected_contact_preview_arrives(diagnostic):
    c = diagnostic
    begin(c)
    candidate = deepcopy(c.plan)
    candidate.diagnostic = 'CONTACT_EXECUTION_PLAN'
    assert c.node._before_near_field_contact_freeze(candidate, c.plan, c.config) == {'failed': True}
    assert 'CONTACT_AND_RETURN_FORBIDDEN' in c.states[-1][1]
    assert len(c.gateways) == 1
    assert [p['phase'] for p in c.publications] == ['observing']


def test_persistence_failure_never_reports_success_or_retries(diagnostic):
    c = diagnostic
    begin(c)
    def fail_write(*_): raise OSError('external disk full')
    assert not c.node._execute_diagnostic_observation(c.plan, c.config, 'observation',
        c.preflight, c.execute, fail_write)
    assert 'external disk full' in c.states[-1][1]
    assert c.node._diagnostic_observation_sample_reference is None
    assert len([e for e in c.events if e[0] == 'motion']) == 1


@pytest.fixture
def diagnostic_start(diagnostic, monkeypatch):
    """Exercise both real start callbacks and real diagnostic executor."""
    c = diagnostic
    c.node.active = False
    c.params['/grasp'] = dict(c.config, use_grasp6d_plan=True)
    c.node._copy_requested_grasp6d_plan = lambda plan_id, _cfg: (
        base.PlanValidationResult(plan_id == c.plan.plan_id), deepcopy(c.plan))
    # The fixture's lightweight plan stands in for an already validated ROS
    # message. Keep the real plan freeze/cleanup and callback control flow.
    monkeypatch.setattr(base, 'plan_id_matches_content', lambda _plan: True)
    c.node._execution_plan_digest = lambda _plan: 'a' * 64
    c.node.execute = lambda grasp6d_plan=None: (
        c.node._execute_grasp6d_plan(c.config, {}, .05, c.preflight,
            c.forbidden, c.forbidden, c.forbidden, grasp6d_plan,
            strict_execute_pose=c.execute))
    c.statuses = []
    def state(stage, message='', success=False):
        c.node.stage = stage
        c.node._last_state_message = message
        c.states.append((stage, message, success))
        c.statuses.append(dict(stage=stage, message=message, success=bool(success),
            active=bool(c.node.active)))
    c.node.set_state = state
    def near_field(active, **_kwargs):
        # Callback setup/cleanup publishes inactive; diagnostic must never
        # activate contact replanning or return.
        assert active is False
        c.events.append(('near_field', active))
    c.node._set_near_field_active = near_field
    return c


def test_full_start_first_inactive_success_and_service_are_diagnostic(diagnostic_start):
    c = diagnostic_start
    response = c.node.start_cb(types.SimpleNamespace(execute=True, plan_id=c.plan.plan_id))
    assert response.success, response.message
    assert response.message.startswith('DIAGNOSTIC_OBSERVATION_COMPLETE:')
    inactive = [status for status in c.statuses if not status['active']]
    assert len(inactive) == 1
    assert inactive[0] == dict(stage=base.GraspStages.SUCCESS, message=response.message,
        success=True, active=False)
    assert all(status['message'].startswith('DIAGNOSTIC_OBSERVATION_COMPLETE:')
        for status in c.statuses if status['success'])
    assert 'no grasp performed' in response.message
    assert not c.node.active and not c.node._start_inflight
    assert not c.node._mode_start_reserved and c.node._mode_bound_selection is None
    assert c.node._bound_execution_plan is None
    assert len([event for event in c.events if event[0] == 'motion']) == 1
    assert [p['phase'] for p in c.publications] == ['observing']
    marker = json.loads(Path(c.node._diagnostic_observation_sample_reference['path']).read_text())
    assert marker['sampling_marker_only'] is True
    assert marker['grasp_completed'] is marker['physical_contour_certified'] is False


@pytest.mark.parametrize('fault', ['preflight', 'motion', 'settle', 'envelope', 'range', 'old_source'])
def test_full_start_failed_diagnostic_never_publishes_success(diagnostic_start, fault):
    c = diagnostic_start
    c.fault['at'] = fault
    response = c.node.start_cb(types.SimpleNamespace(execute=True, plan_id=c.plan.plan_id))
    assert not response.success
    assert not any(status['success'] for status in c.statuses)
    inactive = [status for status in c.statuses if not status['active']]
    assert len(inactive) == 1 and not inactive[0]['success']
    assert not response.message.startswith('DIAGNOSTIC_OBSERVATION_COMPLETE:')
    assert c.node._diagnostic_observation_sample_reference is None
    assert c.node._bound_execution_plan is None
    assert not c.node.active and not c.node._mode_start_reserved


def test_full_start_ordinary_success_retains_original_service_and_release_messages(diagnostic_start):
    c = diagnostic_start
    def ordinary(grasp6d_plan=None):
        assert c.node.active and grasp6d_plan.plan_id == c.plan.plan_id
        assert c.node._diagnostic_observation_sample_reference is None
        c.node.set_state(base.GraspStages.SUCCESS, 'ordinary grasp complete', True)
        return True
    c.node.execute = ordinary
    # An old attempt's marker must not relabel a later ordinary success.
    c.node._diagnostic_observation_sample_reference = dict(path='/old/marker.json', sha256='f'*64)
    response = c.node.start_cb(types.SimpleNamespace(execute=True, plan_id=c.plan.plan_id))
    assert response.success and response.message == 'success'
    inactive = [status for status in c.statuses if not status['active']]
    assert inactive == [dict(stage=base.GraspStages.SUCCESS,
        message='execution slot released: success', success=True, active=False)]
    assert c.node._diagnostic_observation_sample_reference is None


def test_full_start_selection_change_after_observation_cannot_release_sample_or_contact(diagnostic_start):
    c = diagnostic_start
    execute = c.execute
    def changed_selection(pose, submit):
        response = execute(pose, submit)
        c.params[module.SELECTION_PARAM] = dict(c.selection, generation=c.selection['generation'] + 1)
        return response
    c.execute = changed_selection
    response = c.node.start_cb(types.SimpleNamespace(execute=True, plan_id=c.plan.plan_id))
    assert not response.success and 'OBSERVE_RETURN_OWNER_CHANGED' in response.message
    assert not any(status['success'] for status in c.statuses)
    assert len([event for event in c.events if event[0] == 'motion']) == 1
    assert c.node._diagnostic_observation_sample_reference is None
    assert not list(c.tmp_path.glob('diagnostic_observation_*.json'))
    assert [p['phase'] for p in c.publications] == ['observing']


def test_full_start_cleanup_does_not_rearm_same_diagnostic_source_budget(diagnostic_start):
    c = diagnostic_start
    request = types.SimpleNamespace(execute=True, plan_id=c.plan.plan_id)
    first = c.node.start_cb(request)
    assert first.success
    c.statuses.clear()
    # Let the transport fixture accept a repeated capture so that task-side
    # one-source protection is exercised independently of remote admission.
    c.transport['previous'] = None
    second = c.node.start_cb(request)
    assert not second.success and 'ALREADY_ATTEMPTED' in second.message
    assert not any(status['success'] for status in c.statuses)
    assert len([event for event in c.events if event[0] == 'motion']) == 1
    assert c.node._diagnostic_observation_sample_reference is None
    assert len(list(c.tmp_path.glob('diagnostic_observation_*.json'))) == 1
