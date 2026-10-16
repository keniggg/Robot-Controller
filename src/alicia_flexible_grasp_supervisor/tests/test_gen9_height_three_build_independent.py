"""Real selection -> MuJoCo -> publication methods, with no ROS or HTTP IO.

Frozen post-registration inputs and service replies are test doubles. Plan
construction, recipe selection, height placement/margin, fresh precheck adapter,
phase validation, MuJoCo response validator and three caller methods are real.
The controlled clock is a counterexample replay, not a future latency guarantee.
"""
from copy import deepcopy
from dataclasses import dataclass, fields
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_contact_height_window_proposals import fixture as height_fixture

# The installed test lives in supervisor/tests, while its ModeAware fixture
# belongs to the sibling modes package. External review runners may already
# supply both fixture paths; normal pytest invocation needs this one explicitly.
modes_tests = Path(__file__).resolve().parents[2] / 'alicia_grasp_modes/tests'
if modes_tests.is_dir() and str(modes_tests) not in sys.path:
    sys.path.insert(0, str(modes_tests))
from test_remote_modes import module as mode

base = mode.original

# GEN9 log differences; relative to original station entry.
FIRST_ACCEPTED = 1791808341.288757 - 1791808264.957453250
DUPLICATE_REBUILD = 7.786099
SIMULATION_DURATION = 7.6057729280000785
PHASE_DURATION = (1791808354957432985-1791808264957453250)/1e9


@dataclass
class Candidate:
    track_id: int
    variant_index: int
    stable_candidate: object
    payload: object
    pre_moveit_score: float
    evaluation_request_id: int
    evaluation_snapshot_stamp_sec: float
    soft_features: object
    score_weights: object
    moveit_result: object = None
    final_score: float = 0.


def chain_fixture(monkeypatch, *, timed=False, fresh_duration=1., failure=None):
    old, source, prepared, runtime, good, bad, _, _, geometry = height_fixture()
    node = mode.ModeAwareRemoteGrasp6DNode.__new__(mode.ModeAwareRemoteGrasp6DNode)
    node.__dict__.update(old.__dict__)
    # Use the real adapter belonging to the same class as the actual caller.
    del node.__dict__['_final_contact_sequence_evaluation']
    deadline = 100. + PHASE_DURATION
    clock = [100.]
    state = dict(revoked=False, first_accept=False, pending_duplicate=False)
    service_calls, plans, published, guard_calls, builds = [], [], [], [], []
    prepared.ticket = NS(request_id=40, generation=9, target_epoch=1791800107306397756)
    prepared.request_invalidation_generation = 3
    prepared.near_field = True
    prepared.near_field_phase_id = 2
    prepared.near_field_phase_deadline_sec = deadline
    prepared.geometry = NS()
    prepared.snapshot = NS()
    prepared.stamp = source.header.stamp
    node._stream_condition = threading.Condition(threading.RLock())
    node._geometry_state_lock = threading.RLock()
    node._execution_plan_validity_now_sec = lambda: clock[0]
    node._direct_near_field_candidate_deadline_sec = lambda p, r: deadline
    node._direct_near_field_deadline_gate = lambda p: lambda: clock[0] < deadline
    node._near_field_phase_id = 2
    node._near_field_phase_deadline_sec = deadline
    node.near_field_planning_active = True
    node._stream_generation = 9
    node._mode_selection = dict(mode='unknown', generation=9, stamp_ns=1, strategy='two_stage')
    node._direct_strategy_selected = lambda: False
    node.grasp_config = {}
    node.contact_sequence_precheck_enabled = True
    node.latest_joint_state = NS(name=['Joint%d' % i for i in range(1, 7)], position=[0.]*6)
    node.mujoco_config = dict(enabled=True, execution_gate_enabled=True, min_score=80)
    node._early_contact_goal_rejection = lambda *args: None
    node._check_moveit_stable_candidate = lambda candidate: good
    node._check_refinement_candidate_continuity = lambda *args: None
    node._request_multiview_surface_audit = lambda p: {'frozen_fixture': True}
    node._preview_target_signature = lambda stable: 'frozen-target'
    # The independent test starts at the already registered family. This is
    # the input to the changed code, not a bypass of a tested registration.
    node._prepare_final_registered_contact_plan = lambda *args: None

    def ticket_guard(ticket):
        assert ticket is prepared.ticket
        guard_calls.append(clock[0])
        if state['revoked'] or clock[0] >= deadline:
            raise base.StreamResultCancelled('fixture current ticket/phase retired')
    node._require_stream_ticket_current = ticket_guard
    node._require_stream_ticket_current_locked = ticket_guard

    source.candidate_source = 'tabletop_geometry'
    source.candidate_source_lineage = ['tabletop_geometry']
    source.object_geometry.header = deepcopy(source.header)
    sequence = node._rich_plan_execution_sequence(source)
    transform = base.pose_matrix(sequence.grasp)
    gate = old._registered_sequence_geometry_gate(source, prepared, runtime, sequence)
    normalized = base.NormalizedPlanningCandidate(candidate_source='tabletop_geometry',
        source_index=2, variant_index=0, source_lineage=('tabletop_geometry',),
        contact_center_base=transform[:3, 3], T_base_tool0=transform,
        insertion_axis_base=(0., 0., -1.), jaw_axis_base=(0., 1., 0.),
        required_open_width_m=.04, model_width_m=None, model_score=None,
        source_local_score=.4, common_physical_cost=.4, geometry_gate=gate,
        grasp_sequence=sequence, payload=None, audit={})
    stable = NS(track_id=3, hit_count=3, hit_request_ids=(1, 2, 40), request_id=40,
        snapshot_stamp_sec=source.header.stamp.to_sec(), target_epoch=prepared.ticket.target_epoch,
        target_track_id='fixture-target', target_label='fixture', model_choice='unknown_tabletop',
        center_base_xyz=tuple(transform[:3, 3]), approach_base_xyz=(0., 0., -1.),
        required_open_width_m=.04, model_width_m=None, model_score=None,
        candidate_source='tabletop_geometry', source_lineage=('tabletop_geometry',))
    candidate = Candidate(3, 0, stable, normalized, .4, 40, source.header.stamp.to_sec(),
        base.SoftCandidateFeatures(**{f.name: 0. for f in fields(base.SoftCandidateFeatures)}),
        base.SoftScoreWeights(), good)
    runtime.update(prepared=prepared, sequence=sequence, grasp_pose=sequence.grasp,
                   geometry_gate=gate, scored_candidate=candidate)
    node._stable_variant_runtime = {(3, 0): runtime}
    monkeypatch.setattr(base, 'geometry_estimate_to_message',
                        lambda *a, **kw: deepcopy(source.object_geometry))

    def frozen_registration(plan, observation):
        for name in ('refinement_status', 'refinement_translation_m',
                     'refinement_inlier_count', 'fused_view_count', 'refinement_overlap_fraction',
                     'refinement_rmse_m', 'refinement_rotation_deg', 'contact_geometry_evidence'):
            setattr(plan, name, deepcopy(getattr(source, name)))
        return True
    node._apply_registration_evidence_to_plan = frozen_registration

    # Each invocation of the real adapter reaches this new service response;
    # its original before/wait/after phase checks remain active.
    original_id = [None]
    def service(request):
        if original_id[0] is None:
            original_id[0] = request.plan.plan_id
        passed = request.plan.plan_id != original_id[0]
        service_calls.append(deepcopy(request))
        if timed:
            if passed and not state['first_accept']:
                clock[0] = 100. + FIRST_ACCEPTED
                state['first_accept'] = True
            elif not passed and state['first_accept']:
                clock[0] += DUPLICATE_REBUILD - fresh_duration
            elif passed:
                clock[0] += fresh_duration
        if passed and failure == 'strict_ticket' and len(service_calls) >= 3:
            state['revoked'] = True
        if passed and failure == 'strict_deadline' and len(service_calls) >= 3:
            clock[0] = deadline
        reason = bad.reason if not passed else 'strict pose sequence planned stages=pregrasp,approach,grasp'
        evidence = dict(plan_id=request.plan.plan_id, scope='read_only_candidate_constant_bias_prediction',
            execution_authorized=False, success=passed, failed_stage='grasp', reason=reason,
            failure_code='' if passed else 'CONTACT_SEQUENCE_PRECHECK_FAILED', fresh_call=len(service_calls),
            checked_stage_names=['pregrasp', 'approach', 'grasp'],
            contact_prediction=dict(scope='constant_current_bias_prediction', future_following_verified=False,
                execution_authorized=False, nominal_geometry={'fresh': len(service_calls)},
                predicted_command_geometry={'fresh': len(service_calls)}))
        return NS(success=passed, evidence_json=json.dumps(evidence), joint_path_cost=1.,
                  joint_max_delta=.5, message=reason, failed_stage='grasp',
                  failure_code=evidence['failure_code'])
    monkeypatch.setattr(base.rospy, 'wait_for_service', lambda *a, **kw: None)
    monkeypatch.setattr(base.rospy, 'ServiceProxy', lambda *a, **kw: service)
    monkeypatch.setattr(base.rospy, 'get_param', lambda name, default=None:
        False if name == '/mujoco_digital_twin/unknown_lift_gate_enabled' else default)

    fixture_path = Path(__import__('test_contact_height_window_proposals').__file__).parent / 'fixtures/mujoco_lift_contact_loss.json'
    response_template = json.loads(fixture_path.read_text())
    # A local payload sentinel: no HTTP client can be constructed by this test.
    monkeypatch.setattr(base, 'build_mujoco_payload', lambda plan, *a:
        dict(plan_id=plan.plan_id, candidate_source=plan.candidate_source,
             candidate_source_lineage=list(plan.candidate_source_lineage)))
    def simulate(payload):
        plans.append(deepcopy(payload))
        if timed:
            clock[0] += SIMULATION_DURATION
        if failure == 'simulation_ticket':
            state['revoked'] = True
        if failure == 'simulation_deadline':
            clock[0] = deadline
        response = deepcopy(response_template)
        response.update(payload)
        return response
    node._mujoco_client_factory = lambda *a, **kw: NS(simulate_grasp=simulate)

    real_build = node._build_selected_preview_bundle
    def recording_build(*a, **kw):
        bundle = real_build(*a, **kw)
        builds.append(deepcopy(bundle['rich_plan']))
        return bundle
    node._build_selected_preview_bundle = recording_build
    def sink(rich, legacy, ticket, commit_callback, proposal_metadata):
        ticket_guard(ticket)
        published.append(deepcopy(rich))
        commit_callback()
    node._publish_preview_plan = sink
    return NS(node=node, candidate=candidate, prepared=prepared, runtime=runtime,
              source=source, sequence=deepcopy(sequence), clock=clock, state=state,
              service_calls=service_calls, plans=plans, published=published,
              guards=guard_calls, builds=builds, deadline=deadline)


def run_three_builds(env):
    result = env.node._check_direct_registered_candidate(env.candidate)
    assert result.reachable, result.reason
    proposal = env.node._publish_selected_preview(env.candidate, ticket=env.prepared.ticket)
    return proposal


def assert_three_fresh_builds(env):
    assert len(env.builds) == 3 and len(env.published) == 1 and len(env.plans) == 1
    assert env.builds[0] == env.builds[1] == env.builds[2] == env.published[0]
    final_id = env.builds[0].plan_id
    assert env.plans[0]['plan_id'] == final_id == base.compute_plan_id(env.builds[0])
    # Baseline issues six requests [original, height] * 3; the fix must issue
    # [original, height, height, height], all through the original fresh adapter.
    ids = [r.plan.plan_id for r in env.service_calls]
    assert len(ids) == 4, ids
    assert ids[0] != final_id and ids[1:] == [final_id]*3
    assert all(r.deadline.to_sec() == pytest.approx(env.deadline) for r in env.service_calls)
    assert env.runtime['contact_sequence_precheck']['fresh_call'] == 4
    assert env.runtime['mujoco_selection']['code'] == 'MUJOCO_LIFT_DIAGNOSTIC_ONLY'
    assert env.runtime['mujoco_selection']['passed'] is True
    assert env.runtime['sequence'] == env.sequence
    source_positions = np.array([[p.position.x, p.position.y, p.position.z] for p in env.source.poses])
    final_positions = np.array([[p.position.x, p.position.y, p.position.z] for p in env.builds[0].poses])
    shifts = final_positions - source_positions
    np.testing.assert_allclose(shifts, np.tile(shifts[2], (4, 1)), atol=1e-15)
    assert np.linalg.norm(shifts[2]) > 0


def test_real_mode_selection_simulation_and_publication_build_same_height_fresh(monkeypatch):
    env = chain_fixture(monkeypatch)
    run_three_builds(env)
    assert_three_fresh_builds(env)


@pytest.mark.parametrize('fresh_duration', [.1, 1., 2.])
def test_recorded_delay_counterexample_three_builds_fit_only_without_original_repeat(monkeypatch, fresh_duration):
    env = chain_fixture(monkeypatch, timed=True, fresh_duration=fresh_duration)
    run_three_builds(env)
    assert_three_fresh_builds(env)
    assert env.clock[0] == pytest.approx(100. + FIRST_ACCEPTED + 2*fresh_duration + SIMULATION_DURATION)
    assert env.clock[0] < env.deadline - 2.  # Original 2-second selection reserve.
    assert FIRST_ACCEPTED + DUPLICATE_REBUILD + SIMULATION_DURATION > PHASE_DURATION


@pytest.mark.parametrize('failure', ['strict_ticket', 'strict_deadline', 'simulation_ticket', 'simulation_deadline'])
def test_fresh_success_does_not_override_later_ticket_or_deadline(monkeypatch, failure):
    env = chain_fixture(monkeypatch, failure=failure)
    with pytest.raises(base.StreamResultCancelled):
        run_three_builds(env)
    assert not env.published
    assert len(env.plans) == (1 if failure.startswith('simulation') else 0)


@pytest.mark.parametrize('failure', ['ticket', 'deadline'])
def test_simulation_success_does_not_authorize_expired_publication(monkeypatch, failure):
    env = chain_fixture(monkeypatch)
    result = env.node._check_direct_registered_candidate(env.candidate)
    assert result.reachable and len(env.plans) == 1
    if failure == 'ticket':
        env.state['revoked'] = True
    else:
        env.clock[0] = env.deadline
    with pytest.raises(base.StreamResultCancelled):
        env.node._publish_selected_preview(env.candidate, ticket=env.prepared.ticket)
    assert len(env.builds) == 2 and not env.published
