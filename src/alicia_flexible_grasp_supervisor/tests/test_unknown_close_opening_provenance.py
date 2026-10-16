"""The measured aperture margin must survive planning, ROS and fixed closure."""
from copy import deepcopy
from dataclasses import replace
import io
import json
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from alicia_flexible_grasp.grasp.contact_evidence import (
    decode_contact_evidence, encode_contact_evidence)
from alicia_flexible_grasp.grasp.gripper_close_contract import (
    bind_simulated_fixed_close, fixed_close_contract)
from alicia_flexible_grasp.grasp.rich_plan_integrity import (
    compute_plan_id, float32_wire_value, plan_id_matches_content)
from test_contact_geometry_evidence import bound_plan, final_family_fixture
from test_gripper_close_contract import settings, grasp_task_node


REAL_OPENINGS = (.035728940737994604, .03619451612920726)


def measured_plan(opening=REAL_OPENINGS[0], clearance=.0005):
    plan = bound_plan()
    plan.required_open_width_m = opening
    previous = decode_contact_evidence(plan)
    plan.contact_geometry_evidence = encode_contact_evidence(
        contact_center_base=previous['contact_center_base'],
        contact_height_axis_base=previous['contact_height_axis_base'],
        contact_height_bounds_m=previous['contact_height_bounds_m'],
        minimum_contact_overlap_m=previous['minimum_contact_overlap_m'],
        grasp_pose=plan.poses[2], measured_contact_width_m=opening - 2. * clearance,
        opening_fit_clearance_each_side_m=clearance, required_open_width_m=opening)
    plan.plan_id = compute_plan_id(plan)
    return plan


@pytest.mark.parametrize('opening', REAL_OPENINGS)
def test_actual_candidate_margin_survives_wire_simulation_and_real_close(opening):
    plan = measured_plan(opening)
    encoded = io.BytesIO(); plan.serialize(encoded)
    received = type(plan)().deserialize(encoded.getvalue())
    assert plan_id_matches_content(received)
    cfg = settings()  # Legacy global 2 mm margin remains intentionally present.
    expected = float32_wire_value(opening) - .001 - .002
    for candidate in (plan, received):
        bound = bind_simulated_fixed_close(candidate, {}, cfg)
        assert bound['gripper_model']['close_target_inner_gap_m'] == pytest.approx(expected, abs=1e-14)
        node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
        node._simulated_fixed_close_contract = (candidate.plan_id, fixed_close_contract(plan, cfg))
        node._command_gripper_position = Mock(return_value=True)
        assert node._close_gripper(cfg, object(), None, execution_plan=candidate)[0]
        assert node._command_gripper_position.call_args.args[1] == bound['gripper_model']['close_target_inner_gap_m']
    assert fixed_close_contract(plan, cfg) == fixed_close_contract(received, cfg)


def test_unknown_reads_frozen_gate_margin_not_later_global_configuration():
    plan = measured_plan(); cfg = settings()
    original = fixed_close_contract(plan, cfg)
    cfg['plan_bound_opening_clearance_each_side_m'] = .004
    assert fixed_close_contract(plan, cfg) == original
    cfg['plan_bound_close_preload_m'] += .001
    node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
    node._simulated_fixed_close_contract = (plan.plan_id, original)
    node._command_gripper_position = Mock()
    ok, reason = node._close_gripper(cfg, object(), None, execution_plan=plan)
    assert not ok and 'PLAN_GRIPPER_COMMAND_CHANGED' in reason
    node._command_gripper_position.assert_not_called()


@pytest.mark.parametrize('missing', ['', None, 'schema1'])
def test_unknown_missing_aperture_provenance_never_falls_back_to_global_margin(missing):
    plan = bound_plan()
    if missing != 'schema1': plan.contact_geometry_evidence = missing
    with pytest.raises(ValueError, match='PLAN_GRIPPER_CLEARANCE_INVALID'):
        fixed_close_contract(plan, settings())


@pytest.mark.parametrize('field,value', [
    ('opening_fit_clearance_each_side_m', -.001),
    ('opening_fit_clearance_each_side_m', True),
    ('opening_fit_clearance_each_side_m', float('nan')),
    ('opening_fit_clearance_each_side_m', .002),
    ('measured_contact_width_m', 0.),
    ('required_open_width_m', .06),
])
def test_rehashed_invalid_opening_evidence_cannot_authorize_close(field, value):
    plan = measured_plan(); evidence = json.loads(plan.contact_geometry_evidence)
    evidence[field] = value
    plan.contact_geometry_evidence = json.dumps(evidence, sort_keys=True, separators=(',', ':'))
    plan.plan_id = compute_plan_id(plan)
    with pytest.raises(ValueError, match='PLAN_GRIPPER_CLEARANCE_INVALID'):
        fixed_close_contract(plan, settings())


def test_changed_wire_opening_cannot_use_old_measured_margin_even_after_rehash():
    plan = measured_plan(); plan.required_open_width_m += .001
    plan.plan_id = compute_plan_id(plan)
    with pytest.raises(ValueError, match='final plan opening'):
        fixed_close_contract(plan, settings())


def test_partial_encoder_metadata_is_rejected():
    plan = measured_plan(); evidence = decode_contact_evidence(plan)
    with pytest.raises(ValueError, match='complete measured opening provenance'):
        encode_contact_evidence(contact_center_base=evidence['contact_center_base'],
            contact_height_axis_base=evidence['contact_height_axis_base'],
            contact_height_bounds_m=evidence['contact_height_bounds_m'],
            minimum_contact_overlap_m=.002, grasp_pose=plan.poses[2],
            opening_fit_clearance_each_side_m=.0005)


def test_legacy_carton_and_unbound_manual_commands_keep_original_contract():
    cfg = settings()
    for plan in (NS(required_open_width_m=.04),
                 NS(model_choice='carton_segment', diagnostic='CONTACT_EXECUTION_PLAN',
                    required_open_width_m=.04, contact_geometry_evidence='')):
        assert fixed_close_contract(plan, cfg)['close_target_inner_gap_m'] == pytest.approx(.034)
    assert fixed_close_contract(None, cfg)['close_target_inner_gap_m'] == 0.


@pytest.mark.parametrize('fit_clearance', [None, .0005])
def test_final_tabletop_measurement_producer_freezes_its_actual_gate_margin(fit_clearance):
    node, prepared, runtime, plan, candidate = final_family_fixture()
    node.opening_fit_clearance_each_side_m = fit_clearance
    expected_clearance = (node.gripper_geometry.jaw_clearance_each_side_m
                          if fit_clearance is None else fit_clearance)
    node.tabletop_geometry_config = replace(node.tabletop_geometry_config,
        opening_fit_clearance_each_side_m=expected_clearance)
    # Generate the proposal under this actual planner configuration as well;
    # do not hand-adjust its required opening to make the final gate agree.
    candidates, _ = node._generate_tabletop_candidates(prepared.geometry, snapshot=prepared.snapshot)
    candidate = candidates[0]
    normalized = node._normalize_tabletop_candidate(prepared, candidate)
    sequence = normalized.grasp_sequence
    _, profile = node._make_contact_sequence(sequence.grasp, prepared.geometry,
        insertion_axis_base=candidate.insertion_axis_base, snapshot=prepared.snapshot)
    runtime.update(grasp_pose=sequence.grasp, adaptive_stage_profile=profile,
        scored_candidate=NS(payload=normalized, stable_candidate=NS(
            center_base_xyz=candidate.contact_center_base,
            required_open_width_m=candidate.required_open_width_m)))
    plan.required_open_width_m = candidate.required_open_width_m
    plan.poses = [deepcopy(getattr(sequence, stage).pose)
                  for stage in ('pregrasp', 'approach', 'grasp', 'lift')]
    node._prepare_final_registered_contact_plan(plan, prepared, runtime)
    evidence = decode_contact_evidence(plan)
    assert evidence['schema_version'] == 2
    assert evidence['opening_fit_clearance_each_side_m'] == pytest.approx(expected_clearance)
    assert evidence['required_open_width_m'] == plan.required_open_width_m
    assert evidence['measured_contact_width_m'] + 2.*evidence['opening_fit_clearance_each_side_m'] == pytest.approx(plan.required_open_width_m)
    expected = max(0., float32_wire_value(plan.required_open_width_m) - 2.*expected_clearance - .002)
    assert fixed_close_contract(plan, settings())['close_target_inner_gap_m'] == pytest.approx(expected)


def test_graspnet_producer_carries_same_measured_opening_provenance(monkeypatch):
    import test_remote_grasp6d_streaming as fixture
    captured = []
    def encode(**kwargs):
        result = encode_contact_evidence(**kwargs)
        captured.append(json.loads(result))
        return result
    monkeypatch.setattr(fixture.remote_node, 'encode_contact_evidence', encode)
    fixture.test_graspnet_contact_gate_uses_current_fused_surface_and_fails_closed(monkeypatch)
    assert len(captured) == 1
    evidence = captured[0]
    assert evidence['schema_version'] == 2
    assert evidence['opening_fit_clearance_each_side_m'] == pytest.approx(.002)
    assert evidence['required_open_width_m'] == pytest.approx(.039)
    assert evidence['measured_contact_width_m'] == pytest.approx(.035)
