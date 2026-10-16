"""Real ModeAware report dispatch, with inert candidate/strict-service inputs.

This validates selected and rejected report construction, not motion validity.
Neither multiview audit hook nor either report entry is replaced.
"""
from types import SimpleNamespace as NS
from copy import deepcopy

import pytest

from test_remote_modes import configured, module, base, select_unknown
import test_remote_grasp6d_streaming as streaming


@pytest.fixture
def report_case(configured, monkeypatch):
    node, params = configured
    select_unknown(params)
    params[module.SELECTION_PARAM]['strategy'] = 'observe_return'
    assert node.reset_mode_cb(None).success
    node.grasp_config = {}
    node.gripper_tool_jaw_axis = 'y'
    observation = streaming.seed_measured_reference_surface(node)
    node._observe_return_phase = 'initial_contact'
    node._latest_fused_scene_audit = {'unrelated_later_request': True}
    # Stream admission, contact geometry, and strict planning have their own
    # tests. Isolate the two reporting paths after those dependencies finish.
    monkeypatch.setattr(node, '_require_stream_ticket_current', lambda *_: None)
    monkeypatch.setattr(node, '_require_stream_ticket_current_locked', lambda *_: None)
    monkeypatch.setattr(node, '_prepare_final_registered_contact_plan', lambda *_: None)
    result = base.MoveItResult(reachable=True, joint_path_cost=.2,
        joint_max_delta_rad=.1, reason='inert strict-service receipt',
        collision_free=True, within_joint_limits=True, ik_valid=True, planning_success=True)
    monkeypatch.setattr(node, '_strict_check_final_near_field_plan',
        lambda *_: (None, result, {'joint_path_cost': .2}))
    ticket = NS(request_id=19, generation=1, target_epoch=observation.identity.epoch)
    stamp = base.rospy.Time(20, 100_000_000)
    expected = {'snapshot_stamp_ns': stamp.to_nsec(), 'policy': 'request_fixture',
                'center_base_m': [.1, -.2, .03], 'nested': {'retained': [1, 2]}}
    prepared = NS(ticket=ticket, stamp=stamp, geometry=streaming.tabletop_geometry((.05, .035, .021)),
        snapshot=NS(target_identity=observation.identity, source_mode='instance_mask'),
        near_field=True, request_invalidation_generation=0,
        fused_scene_audit=base._freeze_request_evidence(expected))
    pose = base.make_pose_stamped('base_link', (.1, -.2, .12), (0., 0., 0., 1.), stamp=20.1)
    sequence = base.make_grasp_sequence_from_grasp_pose(pose, pregrasp_distance_m=.08,
        approach_offset_m=.03, lift_height_m=.05, approach_direction_base=(0., 0., -1.))
    gate = base.CandidateGateResult(ok=True, failure_code='', failure_reason='',
        required_open_width_m=.039, center_distance_m=0., support_clearance_m=.003,
        jaw_alignment=1., motion_cost=0., geometry_cost=0., failed_gate='', passed_gate_count=6)
    transform = base.pose_matrix(pose)
    normalized = base.NormalizedPlanningCandidate(candidate_source='tabletop_geometry',
        source_index=2, variant_index=0, source_lineage=('tabletop_geometry',),
        contact_center_base=transform[:3, 3], T_base_tool0=transform,
        insertion_axis_base=(0., 0., -1.), jaw_axis_base=(0., 1., 0.),
        required_open_width_m=.039, model_width_m=None, model_score=None,
        source_local_score=.4, common_physical_cost=.4, geometry_gate=gate,
        grasp_sequence=sequence, payload=None, audit={})
    stable = NS(track_id=3, hit_count=3, hit_request_ids=(1, 2, 3), request_id=19,
        snapshot_stamp_sec=20.1, target_epoch=ticket.target_epoch,
        target_track_id=observation.identity.track_id, target_label='unknown',
        model_choice='known-mask-bootstrap', center_base_xyz=tuple(transform[:3, 3]),
        approach_base_xyz=(0., 0., -1.), required_open_width_m=.039, model_width_m=None,
        model_score=None, candidate_source='tabletop_geometry', source_lineage=('tabletop_geometry',))
    features = base.SoftCandidateFeatures(model_score=None, cloud_distance_m=0.,
        center_distance_m=0., downward_approach_cos=1., visibility_center_cost=0.,
        support_margin_m=.003, jaw_tilt_cos=1., geometry_margin_m=.003,
        joint_path_cost=.2, joint_max_delta_rad=.1, stability_hit_ratio=.6,
        position_dispersion_m=.002, orientation_dispersion_rad=.03)
    selected = NS(track_id=3, variant_index=0, stable_candidate=stable, payload=normalized,
        pre_moveit_score=.4, evaluation_request_id=19, evaluation_snapshot_stamp_sec=20.1,
        final_score=.6, soft_features=features, score_weights=base.SoftScoreWeights(), moveit_result=result)
    node._stable_variant_runtime = {(3, 0): dict(prepared=prepared, sequence=sequence,
        grasp_pose=pose, geometry_gate=gate)}
    return node, prepared, selected, expected


@pytest.mark.parametrize('entry', ['selected', 'rejected'])
@pytest.mark.parametrize('phase', ['initial_contact', 'observing_contact'])
@pytest.mark.parametrize('have_frozen', [True, False])
def test_actual_modeaware_reports_keep_polymorphic_fields_and_request_scene(report_case, entry, phase, have_frozen):
    node, prepared, selected, expected = report_case
    node._observe_return_phase = phase
    if not have_frozen:
        prepared.fused_scene_audit = None
    if entry == 'selected':
        bundle = node._build_selected_preview_bundle(selected)
        bundle['commit_callback']()
        report = node._latest_streaming_audit
    else:
        report = node._run_candidate_gate_audit((), prepared.stamp, 'camera_color_optical_frame',
            candidate_pose_estimator=NS(audit_metadata=lambda **_: {}), prepared=prepared,
            outcome_code='NEAR_FIELD_NO_HARD_SAFE_CANDIDATE', commit_state=False)
        assert report['selected'] is None and not report['outcome']['valid_plan']
    audit = report['multiview_surface']
    if phase == 'initial_contact':
        assert audit['evidence_kind'] == 'current_view_only'
        assert audit['cross_view_registration_performed'] is False
    else:
        assert 'evidence_kind' not in audit
    assert audit['fused_scene_geometry'] == (expected if have_frozen else None)
    assert node._latest_fused_scene_audit == {'unrelated_later_request': True}
    if have_frozen:
        audit['fused_scene_geometry']['nested']['retained'][0] = 99
        assert prepared.fused_scene_audit['nested']['retained'][0] == 1
    # Existing no-argument callers retain the subclass interface and metadata.
    plain = node._multiview_surface_audit()
    assert plain['fused_scene_geometry'] == {'unrelated_later_request': True}
