"""Rejected view searches retain evidence without creating plan authority."""
from copy import deepcopy
import json

import pytest

from test_remote_modes import configured, module, base
from test_silhouette_observation_planning import context


def failed_context(configured, monkeypatch, tmp_path, ordinary=True):
    node, prepared, checks, variant = context(
        configured, monkeypatch, tmp_path, ordinary=ordinary)
    binding = dict(request_id=prepared.ticket.request_id,
        generation=prepared.ticket.generation, target_epoch=prepared.ticket.target_epoch,
        target_track_id=prepared.snapshot.target_identity.track_id,
        snapshot_stamp_ns=prepared.snapshot.stamp_ns)
    source = dict(binding=binding, audit=dict(support_points_sha256='frozen-cloud',
        source_stamp_ns=prepared.snapshot.stamp_ns))
    node._diagnostic_observation_view_source = source
    node._diagnostic_observation_view_prepared = prepared
    node._joint_space_observation_cache = dict(prepared=prepared, source=source, key='validated',
        candidates=({'binding': deepcopy(binding), 'execution_authority': False},),
        audit=dict(status='complete', candidate_count=1, directions=[dict(index=0)]), used_sec=.3)
    variant['search_family'] = 'joint_space_fk'
    variant['camera_target_distance_m'] = .29
    variant['observation_view']['check_scheduling_index'] = 0
    variant['observation_view']['direction'] = dict(missing_side_sign=-1)
    variant['sequence'].observation_view_audit = deepcopy(variant['observation_view'])
    # A gateway success followed by a failed view qualification is precisely
    # the previously lost evidence. The real accept/audit flow remains active.
    def reject_view(*_args):
        raise ValueError('original path FOV qualification failed')
    monkeypatch.setattr(node, '_diagnostic_observation_view_evaluation', reject_view)
    return node, prepared, checks, variant, source


@pytest.mark.parametrize('ordinary', [True, False])
def test_failed_strict_success_retains_view_reason_candidates_and_source(
        configured, monkeypatch, tmp_path, ordinary):
    node, prepared, checks, variant, source = failed_context(
        configured, monkeypatch, tmp_path, ordinary)
    before = len(node.rich_plan_pub.messages)
    result = node._accept_prediction(prepared)
    assert result['status'].endswith('OBSERVATION_NO_SAFE_VIEW')
    assert len(checks) == 1 and checks[0][1]
    report = json.loads((tmp_path / 'planning.json').read_text())
    assert report['selected'] is None and report['stable_evaluations'] == []
    assert report['plan_id'] == '' and not report['outcome']['valid_plan']
    assert len(node.rich_plan_pub.messages) == before
    diagnostics = report['observation_failure_diagnostics']
    assert diagnostics['execution_authority'] is False
    rows = diagnostics['observation_orientation_search']['variant_results']
    assert len(rows) == 1 and rows[0]['checked'] and not rows[0]['reachable']
    assert rows[0]['failure_code'].endswith('PATH_VIEW_INVALID')
    assert rows[0]['following_support']['failure_location'] == 'path_view'
    assert 'original path FOV' in rows[0]['reason']
    candidate = diagnostics['observation_candidates'][0]
    assert candidate['search_family'] == 'joint_space_fk'
    assert candidate['camera_target_distance_m'] == .29
    assert candidate['frame_id'] == 'base_link'
    assert candidate['source_stamp_ns'] == prepared.snapshot.stamp_ns
    assert candidate['observation_view']['check_scheduling_index'] == 0
    assert diagnostics['source']['binding_matches_current_request']
    assert diagnostics['source']['prepared_identity_matches']
    assert diagnostics['source']['source_validation_repeated'] is False
    assert diagnostics['joint_space_fk_search']['available']
    assert diagnostics['joint_space_fk_search']['audit']['candidate_count'] == 1
    # Audit serialization must not mutate or consume proposal/path evidence.
    assert variant['observation_view']['direction'] == dict(missing_side_sign=-1)
    assert node._joint_space_observation_cache['candidates'][0]['execution_authority'] is False
    assert source['audit']['support_points_sha256'] == 'frozen-cloud'


@pytest.mark.parametrize('mismatch', ['request_binding', 'prepared', 'cache_prepared', 'cache_source'])
def test_failed_audit_does_not_claim_other_source_cache_matches(
        configured, monkeypatch, tmp_path, mismatch):
    node, prepared, _, _, source = failed_context(configured, monkeypatch, tmp_path)
    if mismatch == 'request_binding':
        source['binding']['request_id'] += 1
    elif mismatch == 'prepared':
        node._diagnostic_observation_view_prepared = object()
    elif mismatch == 'cache_prepared':
        node._joint_space_observation_cache['prepared'] = object()
    else:
        node._joint_space_observation_cache['source'] = deepcopy(source)
    node._accept_prediction(prepared)
    report = json.loads((tmp_path / 'planning.json').read_text())
    details = report['observation_failure_diagnostics']
    assert details['joint_space_fk_search']['available'] is False
    assert 'audit' not in details['joint_space_fk_search']
    assert 'candidates' not in details['joint_space_fk_search']
    if mismatch == 'request_binding':
        assert details['source']['binding_matches_current_request'] is False
    if mismatch == 'prepared':
        assert details['source']['prepared_identity_matches'] is False
    assert report['selected'] is None and not report['outcome']['valid_plan']


def test_no_endpoint_preserves_rejections_without_consuming_strict_budget(
        configured, monkeypatch, tmp_path):
    node, prepared, checks, _, _ = failed_context(configured, monkeypatch, tmp_path)
    rejection = dict(search_family='joint_space_fk', failure_code='OBSERVATION_ENVELOPE_INVALID',
        minimum_support_clearance_m=-.001)
    monkeypatch.setattr(node, '_far_field_observation_variants', lambda *_: ((), (rejection,)))
    node._accept_prediction(prepared)
    report = json.loads((tmp_path / 'planning.json').read_text())
    details = report['observation_failure_diagnostics']
    assert checks == []
    assert details['observation_candidates'] == []
    assert details['observation_endpoint_rejections'] == [rejection]
    assert details['joint_space_fk_search']['audit']['status'] == 'complete'
    assert not report['outcome']['valid_plan']


def test_failed_audit_copies_nested_runtime_evidence(configured, monkeypatch, tmp_path):
    node, prepared, _, _, _ = failed_context(configured, monkeypatch, tmp_path)
    captured = []
    build = node._build_diagnostic_observation_gate_audit
    def capture(*args, **kwargs):
        result = build(*args, **kwargs)
        captured.append(result[0])
        return result
    monkeypatch.setattr(node, '_build_diagnostic_observation_gate_audit', capture)
    node._accept_prediction(prepared)
    runtime = node._stable_variant_runtime[(0, 0)]
    report = captured[-1]
    details = deepcopy(report['observation_failure_diagnostics'])
    runtime['observation_orientation_search']['variant_results'][0]['reason'] = 'mutated after audit'
    runtime['observation_variants'][0]['observation_view']['direction']['missing_side_sign'] = 1
    node._joint_space_observation_cache['audit']['directions'][0]['index'] = 99
    node._diagnostic_observation_view_source['audit']['support_points_sha256'] = 'changed'
    assert report['observation_failure_diagnostics'] == details


# REC4 crossed the measured-source reserve after its original strict search.
# These cases keep the real ordinary=True accept/checker/guard/writer chain;
# only the pre-existing ROS service response and its source clock are supplied.
def reserve_context(configured, monkeypatch, tmp_path):
    from types import SimpleNamespace
    node, prepared, checks, variant, source = failed_context(
        configured, monkeypatch, tmp_path, ordinary=True)
    clock = SimpleNamespace(now=prepared.ticket.snapshot_stamp_sec + .1)
    node._stream_source_clock = lambda: clock.now
    node.execution_plan_validity_sec = 120.
    node.mujoco_selection_snapshot_reserve_sec = 30.
    node._measurement_remote_epoch_ns = 123456789
    node._geometry_robot_description = 'committed test model'
    return node, prepared, checks, variant, source, clock


def expire_during_original_strict(node, prepared, checks, clock, monkeypatch,
        *, success=False, change=None, future=False, forbid_ros=False):
    def strict(pose, observation=False):
        assert observation is True
        checks.append((pose, observation))
        clock.now = prepared.ticket.snapshot_stamp_sec + (-.01 if future else 90.)
        if change is not None:
            change()
        if forbid_ros:
            def forbidden(*args, **kwargs):
                pytest.fail('negative evidence archival queried ROS parameters')
            monkeypatch.setattr(module.rospy, 'get_param', forbidden)
        return (base.MoveItResult(success, .3 if success else 0., .2 if success else 0.,
            'controlled original strict response',
            failure_code='' if success else 'OBSERVATION_PLANNING_TIMEOUT'), {}, '')
    monkeypatch.setattr(node, '_strict_moveit_evaluation', strict)


@pytest.mark.parametrize('strict_success', [False, True])
@pytest.mark.parametrize('forbid_ros_after_expiry', [False, True])
def test_measured_expiry_archives_only_completed_negative_search(
        configured, monkeypatch, tmp_path, strict_success, forbid_ros_after_expiry):
    node, prepared, checks, variant, source, clock = reserve_context(
        configured, monkeypatch, tmp_path)
    expire_during_original_strict(node, prepared, checks, clock, monkeypatch,
        success=strict_success, forbid_ros=forbid_ros_after_expiry)
    before_plans = deepcopy(node.rich_plan_pub.messages)
    with pytest.raises(base.CandidateContractError) as caught:
        node._accept_prediction(prepared)
    assert caught.value.code == 'OBSERVATION_SOURCE_RESERVE_REACHED'
    assert len(checks) == 1
    if strict_success:
        # The original following/source validator rejects this positive RPC
        # inside the checker, before a complete negative search exists. Do
        # not manufacture that missing completed-result evidence.
        assert not (tmp_path / 'planning.json').exists()
        assert not (tmp_path / 'planning.json.execution').exists()
        assert node.rich_plan_pub.messages == before_plans
        assert not any(p.valid for p in node.rich_plan_pub.messages)
        return
    report = json.loads((tmp_path / 'planning.json').read_text())
    assert report['snapshot_stamp_ns'] == prepared.snapshot.stamp_ns
    assert report['request_id'] == prepared.ticket.request_id
    assert report['mode'] == 'continuous_preview'
    assert report['selected'] is None and report['plan_id'] == ''
    assert report['stable_evaluations'] == report['lineage'] == report['rows'] == []
    assert report['outcome']['valid_plan'] is False
    assert report['outcome']['preview_valid'] is False
    assert report['outcome']['code'] == 'OBSERVATION_SOURCE_RESERVE_REACHED'
    details = report['observation_failure_diagnostics']
    assert details['execution_authority'] is False
    assert details['source']['binding_matches_current_request']
    assert details['joint_space_fk_search']['available']
    assert details['joint_space_fk_search']['audit']['candidate_count'] == 1
    assert details['observation_candidates'][0]['search_family'] == 'joint_space_fk'
    search = details['observation_orientation_search']
    assert search['stop_reason'] == 'OBSERVATION_SOURCE_RESERVE_REACHED'
    assert search['search_complete'] is False
    assert search['variant_results'][0]['result_authority_expired'] is True
    assert report['diagnostic_observation_failure']['failure_code'] == 'OBSERVATION_SEARCH_INCOMPLETE'
    assert node.rich_plan_pub.messages == before_plans
    assert not (tmp_path / 'planning.json.execution').exists()
    assert node._stable_variant_runtime[(0, 0)]['_observation_qualified_incumbent_ticket'] is None
    bounded = json.loads(node.gate_audit_pub.messages[-1].data)
    assert bounded['selected'] is None and bounded['report_sha256']
    assert bounded['summary']['status'] == 'OBSERVATION_SOURCE_RESERVE_REACHED'
    # Durable evidence does not retain mutable runtime aliases.
    frozen = (tmp_path / 'planning.json').read_bytes()
    source['audit']['support_points_sha256'] = 'later source'
    variant['observation_view']['direction']['missing_side_sign'] = 1
    assert (tmp_path / 'planning.json').read_bytes() == frozen


@pytest.mark.parametrize('mutation', [
    'generation', 'target_epoch', 'selection', 'phase', 'paused', 'control_epoch',
    'session', 'pending_supplement', 'grasp_config', 'committed_model',
    'geometry_generation', 'source_owner', 'runtime_owner', 'audit_destination',
    'trigger_evidence'])
def test_measured_expiry_cannot_overwrite_another_owner_or_configuration(
        configured, monkeypatch, tmp_path, mutation):
    node, prepared, checks, variant, source, clock = reserve_context(
        configured, monkeypatch, tmp_path)
    sentinel = b'{"existing_owner":"must remain byte-identical"}\n'
    old_path = tmp_path / 'planning.json'
    old_path.write_bytes(sentinel)
    node._latest_gate_audit_reference = dict(report_sha256='original-reference')
    original_reference = deepcopy(node._latest_gate_audit_reference)
    def mutate():
        if mutation == 'generation':
            node._stream_generation += 1
        elif mutation == 'target_epoch':
            node.target_instance_epoch += 1
        elif mutation == 'selection':
            node._mode_selection = dict(node._mode_selection, generation=999)
        elif mutation == 'phase':
            node._observe_return_phase = 'returned_contact'
        elif mutation == 'paused':
            node._observe_return_paused = True
        elif mutation == 'control_epoch':
            node._measurement_remote_epoch_ns += 1
        elif mutation == 'session':
            node._observe_return_session = dict(session_id='new owner',
                deadline_ns=prepared.snapshot.stamp_ns + 1000 * 10**9)
        elif mutation == 'pending_supplement':
            node._observe_return_pending_supplement = dict(request_id='new supplement')
        elif mutation == 'grasp_config':
            node.grasp_config = dict(node.grasp_config, observation_camera_target_nominal_distance_m=.275)
        elif mutation == 'committed_model':
            node._geometry_robot_description = 'changed committed model'
        elif mutation == 'geometry_generation':
            node._geometry_invalidation_generation += 1
        elif mutation == 'source_owner':
            node._diagnostic_observation_view_source = deepcopy(source)
        elif mutation == 'runtime_owner':
            node._stable_variant_runtime[(0, 0)] = dict(
                node._stable_variant_runtime[(0, 0)], prepared=object())
        elif mutation == 'trigger_evidence':
            # Same ticket, phase, gap geometry and noise; only the frozen
            # prediction-source contract changes after its context capture.
            node._observe_return_trigger_evidence = dict(
                node._observe_return_trigger_evidence,
                predicted_gain_evidence=dict(policy='changed prediction contract',
                    predicted_gain_target_m=.004))
        else:
            node.gate_audit_output_path = str(tmp_path / 'new_owner.json')
    expire_during_original_strict(node, prepared, checks, clock, monkeypatch, change=mutate)
    with pytest.raises((base.CandidateContractError, base.StreamResultCancelled)):
        node._accept_prediction(prepared)
    assert old_path.read_bytes() == sentinel
    assert node._latest_gate_audit_reference == original_reference
    assert not (tmp_path / 'new_owner.json').exists()
    assert not (tmp_path / 'planning.json.execution').exists()
    assert not any(p.valid for p in node.rich_plan_pub.messages)


def test_future_source_is_rejected_without_claiming_it_expired(
        configured, monkeypatch, tmp_path):
    node, prepared, checks, variant, source, clock = reserve_context(
        configured, monkeypatch, tmp_path)
    expire_during_original_strict(node, prepared, checks, clock, monkeypatch, future=True)
    with pytest.raises(base.CandidateContractError) as caught:
        node._accept_prediction(prepared)
    assert caught.value.code == 'OBSERVATION_SOURCE_RESERVE_REACHED'
    assert not (tmp_path / 'planning.json').exists()
    assert not any(p.valid for p in node.rich_plan_pub.messages)


def test_positive_search_result_still_needs_original_fresh_source(
        configured, monkeypatch, tmp_path):
    node, prepared, checks, variant = context(configured, monkeypatch, tmp_path, ordinary=True)
    from types import SimpleNamespace
    clock = SimpleNamespace(now=prepared.ticket.snapshot_stamp_sec + .1)
    node._stream_source_clock = lambda: clock.now
    node.execution_plan_validity_sec = 120.
    node.mujoco_selection_snapshot_reserve_sec = 30.
    original_check = node._check_moveit_stable_candidate
    def result_then_expire(seed):
        result = original_check(seed)
        assert result.reachable
        clock.now = prepared.ticket.snapshot_stamp_sec + 90.
        return result
    monkeypatch.setattr(node, '_check_moveit_stable_candidate', result_then_expire)
    with pytest.raises(base.CandidateContractError) as caught:
        node._accept_prediction(prepared)
    assert caught.value.code == 'OBSERVATION_SOURCE_RESERVE_REACHED'
    assert not (tmp_path / 'planning.json').exists()
    assert not (tmp_path / 'planning.json.execution').exists()
    assert not any(p.valid for p in node.rich_plan_pub.messages)


@pytest.mark.parametrize('extension', ['unsupported', 'nonfinite'])
def test_optional_audit_context_cannot_reject_an_original_qualified_view(
        configured, monkeypatch, tmp_path, extension):
    node, prepared, checks, _ = context(configured, monkeypatch, tmp_path, ordinary=True)
    node._diagnostic_observation_view_prepared = prepared
    node._diagnostic_observation_view_source = dict(
        diagnostic_extension=object() if extension == 'unsupported' else float('nan'))
    original_context = node._negative_observation_audit_context
    context_errors = []
    def capture_context(*args):
        try:
            return original_context(*args)
        except Exception as exc:
            context_errors.append(exc)
            raise
    monkeypatch.setattr(node, '_negative_observation_audit_context', capture_context)
    result = node._accept_prediction(prepared)
    assert len(context_errors) == 1 and isinstance(context_errors[0], ValueError)
    assert result['status'] == 'PREVIEW_READY'
    assert len(checks) == 1 and checks[0][1]
    assert node._execution_promotion_audit_ready(node.latest_preview_rich_plan)
    assert node._active_execution_audit_report['outcome']['valid_plan'] is True


@pytest.mark.parametrize('extension', ['unsupported', 'nonfinite'])
def test_unavailable_context_preserves_original_rejection_without_claiming_archive(
        configured, monkeypatch, tmp_path, extension):
    node, prepared, checks, _, source, clock = reserve_context(
        configured, monkeypatch, tmp_path)
    source['diagnostic_extension'] = object() if extension == 'unsupported' else float('nan')
    expire_during_original_strict(node, prepared, checks, clock, monkeypatch)
    warnings = []
    monkeypatch.setattr(module.rospy, 'logwarn', lambda *args: warnings.append(args))
    with pytest.raises(base.CandidateContractError) as caught:
        node._accept_prediction(prepared)
    assert caught.value.code == 'OBSERVATION_SOURCE_RESERVE_REACHED'
    assert isinstance(caught.value.observation_failure_audit_error, ValueError)
    assert len(checks) == 1 and len(warnings) == 1
    assert not (tmp_path / 'planning.json').exists()
    assert not (tmp_path / 'planning.json.execution').exists()
    assert node.gate_audit_pub.messages == []
    assert not any(p.valid for p in node.rich_plan_pub.messages)


def test_malformed_optional_trigger_context_keeps_original_reserve_rejection(
        configured, monkeypatch, tmp_path):
    node, prepared, checks, _, _, clock = reserve_context(configured, monkeypatch, tmp_path)
    node._observe_return_trigger_evidence['optional_prediction_diagnostic'] = object()
    sentinel = b'{"existing_owner":"not replaced by malformed optional context"}\n'
    path = tmp_path/'planning.json'; path.write_bytes(sentinel)
    expire_during_original_strict(node, prepared, checks, clock, monkeypatch)
    with pytest.raises(base.CandidateContractError) as caught:
        node._accept_prediction(prepared)
    assert caught.value.code == 'OBSERVATION_SOURCE_RESERVE_REACHED'
    assert getattr(caught.value, 'observation_failure_audit_error', None) is not None
    assert path.read_bytes() == sentinel
    assert not (tmp_path/'planning.json.execution').exists()
    assert not any(p.valid for p in node.rich_plan_pub.messages)


@pytest.mark.parametrize('failure', ['serialize', 'replace', 'bounded_publish', 'error_log'])
def test_negative_audit_failure_keeps_original_exception_and_no_plan_authority(
        configured, monkeypatch, tmp_path, failure):
    node, prepared, checks, _, _, clock = reserve_context(configured, monkeypatch, tmp_path)
    expire_during_original_strict(node, prepared, checks, clock, monkeypatch)
    original_require = node._require_diagnostic_observation_prepared
    original_rejections = []
    def require(value):
        try:
            return original_require(value)
        except base.CandidateContractError as exc:
            original_rejections.append(exc)
            raise
    monkeypatch.setattr(node, '_require_diagnostic_observation_prepared', require)
    if failure == 'serialize':
        original_assemble = node._assemble_diagnostic_observation_gate_audit
        def assemble(*args, **kwargs):
            result = original_assemble(*args, **kwargs)
            result[0]['controlled_invalid_diagnostic_value'] = float('nan')
            return result
        monkeypatch.setattr(node, '_assemble_diagnostic_observation_gate_audit', assemble)
    elif failure in ('replace', 'error_log'):
        original_replace = base.os.replace
        def replace(source, destination):
            if str(destination) == str(tmp_path / 'planning.json'):
                raise OSError('controlled negative audit disk failure')
            return original_replace(source, destination)
        monkeypatch.setattr(base.os, 'replace', replace)
        if failure == 'error_log':
            def log_failure(*args, **kwargs):
                raise RuntimeError('controlled logging failure')
            monkeypatch.setattr(module.rospy, 'logwarn', log_failure)
    else:
        def publish_failure(message):
            raise RuntimeError('controlled bounded audit publisher failure')
        monkeypatch.setattr(node.gate_audit_pub, 'publish', publish_failure)
    before_plans = deepcopy(node.rich_plan_pub.messages)
    with pytest.raises(base.CandidateContractError) as caught:
        node._accept_prediction(prepared)
    assert caught.value is original_rejections[-1]
    assert caught.value.code == 'OBSERVATION_SOURCE_RESERVE_REACHED'
    assert caught.value.observation_failure_audit_error.code == (
        'PLANNING_AUDIT_FAILED' if failure == 'bounded_publish' else 'PLANNING_AUDIT_WRITE_FAILED')
    assert len(checks) == 1
    assert node.rich_plan_pub.messages == before_plans
    assert node.gate_audit_pub.messages == []
    assert not (tmp_path / 'planning.json.execution').exists()
    assert not list(tmp_path.glob('planning.json.tmp-*'))
    if failure == 'bounded_publish':
        # The disk commit really happened, but no bounded publication succeeded.
        report = json.loads((tmp_path / 'planning.json').read_text())
        assert report['outcome']['valid_plan'] is False
        assert report['selected'] is None and report['plan_id'] == ''
    else:
        assert not (tmp_path / 'planning.json').exists()


def test_negative_audit_keeps_original_ticket_cancellation_semantics(
        configured, monkeypatch, tmp_path):
    node, prepared, checks, _, _, clock = reserve_context(configured, monkeypatch, tmp_path)
    expire_during_original_strict(node, prepared, checks, clock, monkeypatch)
    writer = node._write_gate_audit_report
    def revoke_before_commit(*args, **kwargs):
        node._stream_generation += 1
        return writer(*args, **kwargs)
    monkeypatch.setattr(node, '_write_gate_audit_report', revoke_before_commit)
    with pytest.raises(base.StreamResultCancelled) as caught:
        node._accept_prediction(prepared)
    assert caught.value.code == 'GENERATION_STALE'
    assert not (tmp_path / 'planning.json').exists()
    assert not (tmp_path / 'planning.json.execution').exists()
    assert node.gate_audit_pub.messages == []
    assert not any(p.valid for p in node.rich_plan_pub.messages)

