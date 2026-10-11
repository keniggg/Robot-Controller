"""Measured observation wire/executor boundaries; offline mocked transport only."""
from copy import deepcopy
import io
import json
from types import SimpleNamespace as NS

import pytest

from alicia_flexible_grasp.grasp import rich_plan_integrity as integrity
from alicia_flexible_grasp.robot.observation_path_guard import FrozenObservationScene, ObservationPathError
from test_silhouette_observation_plan_contract import observation_plan as silhouette_plan
from test_task_modes import base


def measured_plan():
    plan = silhouette_plan()
    plan.candidate_source = 'measured_observation'
    plan.candidate_source_lineage = ['measured_observation']
    plan.plan_id = integrity.compute_plan_id(plan)
    return plan


def test_measured_single_pose_wire_scene_and_executor_validation():
    plan = measured_plan()
    assert integrity.is_independent_observation_plan(plan)
    assert not integrity.is_silhouette_observation_plan(plan)
    integrity.validate_independent_observation_plan(plan)
    wire = io.BytesIO(); plan.serialize(wire)
    restored = type(plan)().deserialize(wire.getvalue())
    assert integrity.plan_id_matches_content(restored)
    assert len(restored.poses) == 1 and restored.required_open_width_m == 0.
    scene = FrozenObservationScene.from_plan(restored)
    assert scene.require_target_visibility
    assert list(scene.size) == [.20, .10, .08]
    result = base.validate_execution_plan(restored, 123.3, 2.)
    assert result.ok, (result.code, result.reason)
    assert base.observation_plan_pose(restored).pose == restored.poses[0]
    with pytest.raises(ValueError): base.split_rich_plan_poses(restored)
    with pytest.raises(ObservationPathError): FrozenObservationScene.from_contact_plan(restored)


@pytest.mark.parametrize('field,value', [
    ('candidate_source_lineage', ['graspnet', 'measured_observation']),
    ('candidate_source_lineage', ['measured_observation', 'tabletop_geometry']),
    ('candidate_source_lineage', ['silhouette_observation']),
    ('candidate_source_lineage', ['measured_observation', 'silhouette_observation']),
    ('diagnostic', 'CONTACT_EXECUTION_PLAN'), ('model_choice', 'carton'),
    ('required_open_width_m', .04), ('required_open_width_m', float('nan')),
    ('candidate_width_m', .03), ('has_candidate_model_width', True),
    ('contact_geometry_evidence', '{}'), ('refinement_status', 'VALID_3D'),
    ('poses', [])])
def test_measured_source_cannot_mix_contact_lineage_or_claim_contact_width(field, value):
    plan = measured_plan(); setattr(plan, field, value)
    with pytest.raises(ValueError): integrity.compute_plan_id(plan)
    assert not base.validate_execution_plan(plan, 123.3, 2.).ok


def test_generic_grasp_executor_rejects_measured_observation_before_actuation():
    states = []
    node = NS(set_state=lambda *args: states.append(args))
    def forbidden(*args, **kwargs): pytest.fail('observation invoked contact actuation')
    assert not base.GraspTaskNode._execute_grasp6d_plan(node, {}, {}, 0.,
        forbidden, forbidden, forbidden, forbidden, measured_plan(), strict_execute_pose=forbidden)
    assert len(states) == 1


@pytest.mark.parametrize('strategy', ['direct', 'two_stage'])
def test_runner_does_not_accept_measured_view_under_other_strategies(monkeypatch, strategy):
    from test_mode_runner import module
    plan = measured_plan()
    runner = module.ModeAwareGraspRunner('unknown', strategy)
    runner.selection = dict(mode='unknown', strategy=strategy, generation=20, stamp_ns=122000000000)
    runner.minimum_stamp_ns = 123000000000; runner.plan_validity_sec = 5.; runner.expected_model = 'unknown_tabletop'
    monkeypatch.setattr(runner, 'selection_current', lambda: True)
    monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(lambda: module.rospy.Time(124)))
    runner.on_preview(plan)
    assert runner.preview is None
    assert not runner._authority_preview_matches_attempt(plan, plan)


def test_measured_observation_can_use_original_return_contract_without_becoming_diagnostic(monkeypatch):
    from test_observe_return_gateway import fixture, capture, begin_return, run_return
    from alicia_flexible_grasp.robot.observation_path_guard import CommittedObservationContexts
    node, planner, plan, state, params, calls = fixture(monkeypatch)
    plan.candidate_source = 'measured_observation'
    plan.candidate_source_lineage = ['measured_observation']
    plan.poses = plan.poses[:1]
    plan.has_candidate_model_width = False
    plan.candidate_width_m = plan.required_open_width_m = 0.
    plan.refinement_status = 'NOT_EVALUATED'; plan.contact_geometry_evidence = ''
    plan.plan_id = integrity.compute_plan_id(plan)
    node._observation_contexts = CommittedObservationContexts()
    node._observation_contexts.set_active(True)
    node._observation_contexts.ingest(plan)
    result = capture(node, plan)
    assert result.success, result.message
    assert not calls
    begin_return(node, state)
    result = run_return(node, plan)
    assert result.success, result.message
    assert json.loads(result.message)['code'] == 'OBSERVE_RETURN_MEASURED_ARRIVAL'
    assert len(calls) == 1
    node._validate_frozen_observation_path.assert_called_once()


def test_silhouette_remains_narrow_and_cannot_inherit_measured_return_permission():
    from alicia_flexible_grasp.robot.observe_return_gateway import ObserveReturnGateway
    plan = silhouette_plan()
    assert integrity.is_independent_observation_plan(plan)
    assert integrity.is_silhouette_observation_plan(plan)
    with pytest.raises(ObservationPathError, match='DIAGNOSTIC_OBSERVATION_RETURN_FORBIDDEN'):
        ObserveReturnGateway._validate_observe_return_context(object(), {'plan': plan}, returning=True)


def test_runner_same_phase_cannot_promote_a_different_observation_source(monkeypatch):
    from test_mode_runner import module
    from test_rich_plan_candidate_source import neutral_refinement_ros_plan
    initial = measured_plan()
    runner = module.ModeAwareGraspRunner('unknown', 'observe_return')
    runner.selection = dict(mode='unknown', strategy='observe_return', generation=20, stamp_ns=122000000000)
    runner.minimum_stamp_ns = 123000000000
    runner.plan_validity_sec = 5.
    runner.expected_model = 'unknown_tabletop'
    monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(lambda: module.rospy.Time(124)))
    assert runner._authority_preview_matches_attempt(initial, initial)
    for replacement in (silhouette_plan(), neutral_refinement_ros_plan()):
        replacement.model_choice = initial.model_choice
        replacement.diagnostic = initial.diagnostic
        replacement.plan_id = integrity.compute_plan_id(replacement)
        assert not runner._authority_preview_matches_attempt(replacement, initial)
        assert not runner._authority_preview_matches_attempt(initial, replacement)


@pytest.mark.parametrize('rebind', ['fresh_contact', 'same_observation', 'missing'])
def test_single_measured_view_requires_new_four_pose_contact_before_close(monkeypatch, rebind):
    """Real task orchestration with accepted gate/transport test doubles only.

    This proves phase and stage wiring, never physical contact feasibility.
    """
    import threading
    from test_grasp_task_sequence import GraspTaskSequenceTest
    helper = GraspTaskSequenceTest()
    contact = helper._rich_plan(stamp_sec=123.)
    contact.model_choice = 'unknown_tabletop'
    contact.diagnostic = 'CONTACT_EXECUTION_PLAN'
    contact = helper._mark_contact_geometry(contact)
    observation = deepcopy(contact)
    observation.candidate_source = 'measured_observation'
    observation.candidate_source_lineage = ['measured_observation']
    observation.diagnostic = 'FAR_FIELD_OBSERVATION_PLAN'
    observation.poses = observation.poses[:1]
    observation.poses[0].position.x -= .03
    observation.has_candidate_model_width = False
    observation.candidate_width_m = observation.required_open_width_m = 0.
    observation.contact_geometry_evidence = ''
    observation.plan_id = integrity.compute_plan_id(observation)
    node = base.GraspTaskNode.__new__(base.GraspTaskNode)
    node.active = True
    node._near_field_active = False
    node._grasp6d_plan_lock = threading.RLock()
    node._bound_target_occlusion_allowed = False
    node._mode_bound_selection = dict(mode='unknown', strategy='observe_return')
    node._observe_return_session = dict(phase='observing', origin_plan_id=observation.plan_id)
    states, moves, refinements, contact_checks = [], [], [], []
    node.set_state = lambda *args, **kwargs: states.append(args)
    node._position_only_execute_globally_enabled = lambda: False
    node._execution_checkpoint = lambda *a, **k: True
    node._current_tool_pose_base = lambda: base.observation_plan_pose(observation)
    node._current_observation_view_reusable = lambda *a, **k: True
    node._wait_for_motion_settle = lambda *a, **k: None
    node._command_gripper_position = lambda *a, **k: True
    node._validate_measured_observation_envelope = lambda *a, **k: True
    node._wait_for_fresh_observation_camera_target_range = lambda *a, **k: base.PlanValidationResult(True)
    node._set_near_field_active = lambda active, **k: setattr(node, '_near_field_active', bool(active))
    node._maybe_rebind_near_field_grasp6d_plan = lambda *a, **k: (
        contact if rebind == 'fresh_contact' else observation if rebind == 'same_observation' else None)
    node._simulate_grasp6d_plan_if_required = lambda *a, **k: pytest.fail('observation reached contact simulation')
    node._maybe_final_refine_grasp6d_plan = lambda _g, _c, plan: (refinements.append(plan.plan_id) or plan)
    node._plan_and_execute_pose = lambda _stage, label, *a, **k: (moves.append(label) or True)
    node._validate_measured_contact_before_close = lambda plan, _cfg: (contact_checks.append(plan.plan_id) or True)
    node._close_gripper = lambda *a, **k: (moves.append('gripper close') or (True, 'unit transport accepted'))
    node._post_lift_visual_verification_result = lambda *a, **k: base.PlanValidationResult(True)
    monkeypatch.setattr(base.rospy, 'get_param', lambda name, default=None: default)
    monkeypatch.setattr(base.rospy.Time, 'now', staticmethod(lambda: base.rospy.Time.from_sec(124.)))
    result = node._execute_grasp6d_plan(dict(near_field_strategy='single_snapshot_direct',
        plan_validity_sec=5., near_field_replan_enabled=True,
        near_field_replan_required=True, observation_reuse_position_tolerance_m=.025),
        dict(open_position_m=.05, use_compliant_close=False), .05,
        object(), object(), object(), None, observation, strict_execute_pose=lambda *a, **k: None)
    if rebind == 'fresh_contact':
        assert result
        assert refinements == contact_checks == [contact.plan_id]
        assert moves == ['6D near-field pregrasp', 'linear 6D approach',
            'linear 6D grasp pose', 'gripper close', 'linear 6D lift']
    else:
        assert not result
        assert not moves and not refinements and not contact_checks
        if rebind == 'same_observation':
            assert any('NEAR_FIELD_PLAN_PHASE_INVALID' in str(state) for state in states)
