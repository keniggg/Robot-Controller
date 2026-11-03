"""GEN40 handoffs and the exact GEN39 final source, without real devices.

The first cases use the unchanged GEN35 task main loop and explicit SDK/FK,
MoveIt and actuator simulators. The closed-record case separately rebuilds
the real three-frame pixels, qualifies all 2610 original points, and runs the
actual producer and typed consumers. It does not manufacture a historical
contact trajectory, a real contact FK sample, or physical grasp success.
"""
from copy import deepcopy
from dataclasses import asdict
import hashlib
import io
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

import test_gen35_minimal_chain_replay as chain
from test_gen38_cuboid_chain import cuboid_scene
from test_gen39_registration_chain import _install_real_reached_source_adapter


@pytest.fixture(autouse=True)
def forbid_unsimulated_boundaries(monkeypatch):
    import rospy
    import socket

    def forbidden(*args, **kwargs):
        raise chain.UnsimulatedExternalCall('GEN40 external boundary '+repr(args))

    for name in ('connect', 'connect_ex', 'sendto', 'bind', 'listen'):
        monkeypatch.setattr(socket.socket, name, forbidden)
    monkeypatch.setattr(socket, 'create_connection', forbidden)
    for name in ('init_node', 'Publisher', 'Subscriber', 'Service', 'ServiceProxy',
                 'wait_for_service', 'set_param', 'delete_param'):
        monkeypatch.setattr(rospy, name, forbidden)


@pytest.mark.parametrize('order', [
    ('status', 'ack', 'object', 'mask'),
    ('mask', 'object', 'status', 'ack'),
    ('ack', 'mask', 'status', 'object'),
])
def test_current_geometry_policy_keeps_the_original_perturbed_task_chain(
        monkeypatch, tmp_path, order):
    """No contact/geometry gate is doubled; device providers remain simulated."""
    from alicia_flexible_grasp.grasp.contact_evidence import decode_contact_evidence
    from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'runtime_logs'))
    monkeypatch.setattr(chain, 'ray_scene', cuboid_scene)
    _install_real_reached_source_adapter(monkeypatch)
    c = chain.unified_replay(monkeypatch, tmp_path, order, perturb=True)
    evidence = decode_contact_evidence(c.contact, required=True)
    assert c.registration.ok and c.prepared.geometry.ok
    assert evidence['schema_version'] == 3
    assert evidence['measurement_source'] == 'regular_box_model_inference'
    assert evidence['box_model_evidence']['source']['stamp_ns'] == c.current_row['object'].header.stamp.to_nsec()
    assert c.contact.plan_id == compute_plan_id(c.contact)
    assert [row['state'] for row in c.requests] == ['executing', 'requalifying', 'qualified']
    assert len(c.phases) == 1 and c.session['max_observations'] == 1
    assert len(c.task._frozen_contact_pub.messages) == 1
    assert c.execution_result is False and c.states[-1].state == 'HOLDING'
    assert 'GRASP_HOLD_UNVERIFIED' in c.states[-1].message
    assert sum(row[0] == 'motion' and row[1] == c.origin.plan_id for row in c.device_calls) == 1
    assert sum(row[0] == 'motion' and row[1] == c.contact.plan_id for row in c.device_calls) == 3
    assert sum(row[0] == 'gripper' and row[2] < .05 for row in c.device_calls) == 1
    assert not any(row[0] == 'lift' for row in c.device_calls)
    assert 'qualified_then_late_pending_and_duplicate_ack' in c.injected_events
    assert 'duplicate_start_rejected' in c.injected_events
    assert c.task._last_measured_aperture_sample['ok']
    (tmp_path/'gen40_perturbed_chain.json').write_text(json.dumps(dict(
        plan_id=c.contact.plan_id, source_stamp_ns=c.contact.header.stamp.to_nsec(),
        model_sha256=evidence['box_model_evidence']['model_sha256'],
        injected_events=c.injected_events, physical_grasp_verified=False,
        SDK_FK_IK_and_actuator_providers='explicit deterministic simulators',
        actual_contact_aperture=c.task._last_measured_aperture_sample), indent=2)+'\n')


_E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
_D = _E/'ros_log/full_latest_gen36_restart_20261017T012037Z'
_G = _E/'grasp_records/20261017T070447.445347Z_83343e1525b54994a6f491259995a100'
_A = _D/'verification/gen39_actual_attempt1_source_replay_r1'
_C = _D/'verification/gen39_actual_attempt1_chain_index_r1'
_AUDIT_SHA = '9455bc56d73cbdf2a04ffbf86128abde2c71866ad9b31abe61b477207c174f6f'
_HISTORICAL_MODEL_SHA = 'a52f80400af701553a7866f4791b6ff2913b70382e57ebb352bc37bea823e322'
_MODEL_SHA = 'f0e3916e28ce6a0372c953f41ce0b82013c40dc49210f17af8c2d33f5219b336'
_GEN43_COVERAGE_FIELDS = (
    'coverage_policy', 'silhouette_mask_intersection_pixels',
    'silhouette_mask_eroded_polygon_pixels', 'top_mask_intersection_pixels',
    'top_mask_eroded_polygon_pixels', 'top_mask_coverage',
)
_CURRENT_CAS_SHA = '5ea856632a6495c5cc45ed3ff6bf4240a86bce253be10a47118a61c337502161'


def _read(path):
    return json.loads(path.read_bytes())


def _sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _array_sha(value):
    return hashlib.sha256(np.asarray(value).tobytes()).hexdigest()



def _assert_historical_model_compatibility(model):
    """New signed identity includes coverage policy; old numerical body is exact.

    This read-only projection is never supplied to a planner or consumer. It
    verifies the closed source retained every old geometry, uncertainty and
    source field while its six explicit GEN43 coverage fields enter the new
    model digest used on the actual wire.
    """
    actual = model.to_evidence_dict()
    assert actual['model_sha256'] == _MODEL_SHA
    legacy = deepcopy(actual)
    legacy.pop('model_sha256')
    interval = legacy['qualification']['cuboid_rgb_interval']
    assert interval['coverage_policy'] == 'measured_top_mask_with_complete_cuboid_rgb_v1'
    for key in _GEN43_COVERAGE_FIELDS:
        interval.pop(key)
    encoded = json.dumps(legacy, sort_keys=True, separators=(',', ':'),
        allow_nan=False, ensure_ascii=True).encode('utf-8')
    assert hashlib.sha256(encoded).hexdigest() == _HISTORICAL_MODEL_SHA
    # Keep the actual generated evidence untouched, including all new fields.
    assert model.to_evidence_dict() == actual


def _actual_final_inputs():
    """Exact archived input, never a substitute scene or hand-written model."""
    import cv2
    from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
    from alicia_flexible_grasp.vision.rgbd_snapshot import RgbdSample, fuse_stable_samples
    from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
    audit_path = _G/'keyframes_audits'/(_AUDIT_SHA+'.json')
    assert _sha(audit_path) == _AUDIT_SHA
    a = _read(audit_path)
    camera = _read(_D/'gen39_runtime/grasp_scene_r1/camera_configuration.json')
    K = NS(**{key: camera[key] for key in ('width', 'height', 'fx', 'fy', 'cx', 'cy')})
    K.depth_scale = camera['depth_scale']
    qualified = a['remote_diagnostics']['tabletop_geometry']['regular_box_inference']
    identity = TargetTrackIdentity(**qualified['source_target_identity'])
    receipt = _read(_A/'source_images/receipt.json')
    stamps = a['pipeline_funnel']['snapshot_evidence']['source_stamp_ns']
    assert stamps == [1792220758629359245, 1792220759228944301, 1792220759499861717]
    bbox = tuple(a['graspnet_input']['detected_bbox_xywh'])
    q = _read(_C/'accepted_motion_summary.json')['settle_request']['accepted_positions_rad']
    samples = []
    for stamp in stamps:
        raw = {}
        for role, suffix in (('color', '.png'), ('depth', '.npy'), ('mask', '.png')):
            path = _A/'source_images'/(str(stamp)+'_'+role+suffix)
            assert _sha(path) == receipt['output_sha256'][path.name]
            raw[role] = np.load(path, allow_pickle=False) if role == 'depth' else cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        samples.append(RgbdSample(raw['color'], raw['depth'], raw['mask'], bbox,
            None, stamp*1e-9, 'camera_link', np.asarray(q), stamp, identity.epoch, identity))
    snapshot = fuse_stable_samples(samples, True, .85, 5., .01, 2,
        K.depth_scale, .03, 2., 3.5, .002, internal_hole_max_area_px=25)
    assert snapshot.ok, snapshot
    assert _array_sha(snapshot.object_mask) == a['graspnet_input']['object_mask_sha256']
    for role, array in (('color', samples[-1].color_bgr), ('depth', samples[-1].depth_raw), ('mask', samples[-1].object_mask)):
        assert _array_sha(array) == qualified['retained_pixel_sha256'][role+'_sha256']
    old = a['replay_geometry']
    points = np.asarray(old['object_points_base_m'])
    assert len(points) == 2610 and _array_sha(points) == old['object_points_sha256']
    enclosure = a['remote_diagnostics']['tabletop_geometry']['regular_box_planning_enclosure']
    geometry = GeometryEstimate(True, '', '', enclosure['original_center_base'],
        enclosure['original_axes_base'], enclosure['original_size_xyz_m'],
        old['support_normal_base'], old['support_offset_m'], old['support_inlier_ratio'],
        points, old['source_mode'])
    old_planning = GeometryEstimate(True, '', '', old['obb_center_base_m'], old['R_base_obb'],
        old['obb_size_xyz_m'], old['support_normal_base'], old['support_offset_m'],
        old['support_inlier_ratio'], points, old['source_mode'])
    return NS(audit=a, K=K, snapshot=snapshot, identity=identity, geometry=geometry,
        old_planning=old_planning, T=np.asarray(a['snapshot_transform']['T_base_optical']))


def _actual_producer(monkeypatch, inputs):
    """Configure original remote methods with archived data-only providers."""
    import test_remote_grasp6d_streaming as f
    from alicia_flexible_grasp.vision.target_observation import TargetObservation
    from alicia_flexible_grasp.vision.multiview_surface import (
        FusedTargetSurface, SurfaceView, RegistrationResult)
    remote = f.remote_node
    stamp = inputs.snapshot.stamp_ns
    now = stamp*1e-9+.1
    events = _read(_C/'chain_events.json')
    error = [row['data']['json'] for row in events if row['topic'] == '/experiment/runtime_execution_error_snapshot'][-1]
    params = {'/grasp/unknown_require_measured_bilateral_contact': False,
              '/grasp_6d/runtime_execution_error': error}
    monkeypatch.setattr(remote.rospy, 'get_param', lambda name, default=None: deepcopy(params.get(name, default)))
    monkeypatch.setattr(remote.rospy, 'get_param_cached', lambda name, default=None: deepcopy(params.get(name, default)))
    monkeypatch.setattr(remote.rospy.Time, 'now', staticmethod(lambda: remote.rospy.Time.from_sec(now)))
    node = remote.RemoteGrasp6DNode.__new__(remote.RemoteGrasp6DNode)
    node.tabletop_geometry_enabled = True
    node.tabletop_geometry_config = f.TabletopGeometryConfig(max_candidates=16, opening_fit_clearance_each_side_m=.0005)
    node.gripper_geometry = f.tabletop_gripper()
    node.opening_fit_clearance_each_side_m = .0005
    node.gripper_tool_jaw_axis, node.gripper_tool_finger_length_axis = 'y', 'z'
    node.gripper_physical_open_width_m = .05
    node.grasp_config = dict(tool_approach_axis='z', pregrasp_distance_m=.036,
        final_approach_offset_m=.02, lift_height_m=.035)
    node.candidate_min_downward_approach_cos = .65
    node.candidate_max_final_approach_lateral_m = .020
    node.soft_score_weights = f.SoftScoreWeights()
    node.target_absolute_sanity_distance_m = .15
    node.camera_visibility_gate_enabled = False
    node.camera_visibility_diagnostic_enabled = False
    f.configure_identity_handeye(node)
    node.near_field_planning_active = True
    node._stream_generation, node._near_field_phase_id = 7, 4
    node._current_stream_target_identity = lambda: inputs.identity
    node._camera_intrinsics = lambda: inputs.K
    node.latest_joint_state = None  # current-tool audit unavailable; no FK is invented here
    box = inputs.snapshot.bbox
    observation = TargetObservation(inputs.identity, stamp, 'base_link',
        inputs.geometry.object_points_base, inputs.geometry.support_normal_base,
        inputs.geometry.support_offset_m, box, inputs.snapshot.depth_raw.shape,
        58, 'instance_mask', 'unknown_small')
    node._latest_target_observation = observation
    cas_path = _C/'surface_snapshot.json'
    assert _sha(cas_path) == _CURRENT_CAS_SHA
    cas = _read(cas_path)
    surface = FusedTargetSurface(inputs.identity, cas['points_base'], cas['view_indices'],
        cas['view_stamps_ns'], view_sources=cas['view_sources'])
    ref = cas['reference']
    reference = SurfaceView(inputs.identity, ref['stamp_ns'], ref['points_base'],
        ref['support_normal_base'], ref['support_offset_m'], ref['edge_clearance_px'])
    node._near_field_fused_surface, node._near_field_reference_view = surface, reference
    node._near_field_surface_generation, node._near_field_surface_phase_id = 7, 4
    r = inputs.audit['multiview_surface']['latest_registration']
    registration = RegistrationResult(True, 'REGISTERED', np.eye(4),
        r['inlier_count'], r['overlap_fraction'], r['rmse_m'],
        r['support_normal_angle_deg'], r['support_plane_separation_m'])
    node._latest_registration_evidence = remote.RegistrationEvidence(
        inputs.identity, stamp, registration, source_clipped=False)
    planning = node._prepare_regular_box_snapshot(inputs.snapshot, inputs.geometry, inputs.T)
    model = node._current_inferred_box()
    assert model is not None, node._regular_box_snapshot
    assert model.model_sha256 == _MODEL_SHA
    _assert_historical_model_compatibility(model)
    np.testing.assert_array_equal(planning.object_points_base, inputs.geometry.object_points_base)
    prepared = NS(geometry=planning, stamp=remote.rospy.Time(*divmod(stamp, 10**9)),
        near_field=True, snapshot=inputs.snapshot, target_observation=observation)
    return node, prepared


@pytest.mark.skipif(not (_G/'keyframes_audits'/(_AUDIT_SHA+'.json')).is_file(),
    reason='explicit closed GEN39 source fixture is not installed; no historical pixels/points are fabricated')
def test_exact_closed_final_source_traverses_new_enclosure_producer_and_consumers(monkeypatch, tmp_path):
    """New offline generation, with all original points; no original contact path exists."""
    from alicia_flexible_grasp.grasp.contact_evidence import decode_contact_evidence
    from alicia_flexible_grasp.grasp.contact_execution import plan_contact_evaluator, plan_aperture_evaluator
    from alicia_flexible_grasp.grasp.gripper_geometry import _obb_line_interval
    from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
    from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan
    import test_remote_grasp6d_streaming as f
    inputs = _actual_final_inputs()
    node, prepared = _actual_producer(monkeypatch, inputs)
    model = node._current_inferred_box()
    old_interval = _obb_line_interval(model.center_base, model.axes_base[:, 1],
        inputs.old_planning.center_base, inputs.old_planning.axes_base, inputs.old_planning.size_xyz_m)
    old_required = 2.*max(-old_interval[0]+.0005, old_interval[1]+.0005)
    assert old_required == pytest.approx(.0508844308374338, abs=1e-12)
    assert old_required > .05 > .0498
    old_candidates, old_audit = node._generate_tabletop_candidates(inputs.old_planning, inputs.snapshot)
    assert not old_candidates
    assert old_audit['proposal_reach_rejections'][0]['failed_gate'] == 'finger_reach'
    candidates, audit = node._generate_tabletop_candidates(prepared.geometry, prepared.snapshot)
    assert candidates, audit
    assert audit['proposal_count'] >= 1 and audit['materialized_count'] >= 1
    assert node._regular_box_snapshot['geometry'] is inputs.geometry
    assert model.source['stamp_ns'] == inputs.snapshot.stamp_ns
    errors = []
    accepted = None
    for candidate in candidates:
        try:
            normalized = node._normalize_tabletop_candidate(prepared, candidate)
            sequence = normalized.grasp_sequence
            _, profile = node._make_contact_sequence(sequence.grasp, prepared.geometry,
                insertion_axis_base=normalized.insertion_axis_base, snapshot=prepared.snapshot)
            runtime = dict(prepared=prepared, grasp_pose=sequence.grasp, adaptive_stage_profile=profile,
                scored_candidate=NS(payload=normalized, stable_candidate=NS(
                    center_base_xyz=normalized.contact_center_base,
                    required_open_width_m=normalized.required_open_width_m)))
            plan = f.bound_surface_plan(prepared.target_observation)
            plan.valid = True
            plan.header.frame_id = 'base_link'
            plan.object_geometry = f.remote_node.geometry_estimate_to_message(prepared.geometry,
                snapshot=prepared.snapshot, stamp=prepared.stamp, target_identity=inputs.identity)
            plan.candidate_source = 'tabletop_geometry'
            plan.candidate_source_lineage = ['tabletop_geometry']
            plan.model_choice, plan.diagnostic = 'unknown_tabletop', 'CONTACT_EXECUTION_PLAN'
            plan.required_open_width_m = normalized.required_open_width_m
            plan.poses = [deepcopy(getattr(sequence, name).pose) for name in ('pregrasp', 'approach', 'grasp', 'lift')]
            gate = node._registered_sequence_geometry_gate(plan, prepared, runtime, sequence)
            assert gate.ok, gate
            plan.contact_geometry_evidence = runtime['final_contact_geometry_evidence']
            plan.plan_id = compute_plan_id(plan)
            transport = io.BytesIO(); plan.serialize(transport)
            received = Grasp6DPlan().deserialize(transport.getvalue())
            assert received.plan_id == compute_plan_id(received) == plan.plan_id
            evidence = decode_contact_evidence(received, required=True)
            assert evidence['box_model_evidence']['model_sha256'] == _MODEL_SHA
            T = f.remote_node.pose_matrix(sequence.grasp)
            contact = plan_contact_evaluator(received, {}, .0498)(T)
            assert contact['ok'], contact
            aperture = plan_aperture_evaluator(received, {}, .0498, 2)(T)
            assert aperture['ok'], aperture
            necessary = node._contact_goal_inevitable_collision(received, node.gripper_geometry, .05, .001)
            assert necessary['checked'] and not necessary['rejected'], necessary
            accepted = (received, evidence, gate, contact, aperture, necessary)
            break
        except (f.remote_node.CandidateContractError, ValueError, AssertionError) as exc:
            errors.append(str(exc))
    assert accepted is not None, errors
    plan, evidence, gate, contact, aperture, necessary = accepted
    np.testing.assert_array_equal(prepared.geometry.object_points_base, inputs.geometry.object_points_base)
    (tmp_path/'gen40_exact_closed_final_producer.json').write_text(json.dumps(dict(
        diagnostic_only=True, execution_authorized=False, source_stamp_ns=inputs.snapshot.stamp_ns,
        retained_points=2610, model_sha256=_MODEL_SHA,
        historical_model_sha256=_HISTORICAL_MODEL_SHA,
        historical_compatibility_projection_only=True, old_union_required_m=old_required,
        generated_candidates=len(candidates), plan_id=plan.plan_id,
        source_bound_enclosure=node._regular_box_snapshot['enclosure'],
        depth_quality=asdict(inputs.snapshot.quality), original_device_path_available=False,
        future_contact_FK_provider='explicit nominal==actual geometric simulator; not a new hardware measurement',
        endpoint_IK_and_retimed_path='not fabricated; no historical contact path was produced',
        geometry_gate=asdict(gate), typed_contact=contact, nominal_simulated_aperture=aperture,
        necessary_endpoint_collision=necessary,
        evidence_schema=evidence['schema_version'], physical_grasp_verified=False), indent=2)+'\n')
