"""Offline GEN8 exact-source lattice plus first FK, real adapter and scheduling. No RPC/actions."""
import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS

ap = argparse.ArgumentParser()
ap.add_argument('--source-root', type=Path, required=True)
ap.add_argument('--output', type=Path, required=True)
args = ap.parse_args()
root = args.source_root.resolve()
for relative in ('src/alicia_grasp_modes/tests', 'src/alicia_grasp_modes/src',
                 'src/alicia_flexible_grasp_supervisor/src'):
    sys.path.insert(0, str(root / relative))
os.environ['ROS_PACKAGE_PATH'] = str(root / 'src') + ':' + os.environ.get('ROS_PACKAGE_PATH', '')
import numpy as np
import rosbag
import rospy
import tf
from _pytest.monkeypatch import MonkeyPatch
import test_remote_modes as fixture
from alicia_grasp_modes.observation_policy import observation_config

s = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_frozen_no_skin_20261012T021050Z')
assert Path('/media/zhuyupei/SanStand').is_mount() and s in args.output.resolve().parents
inputs = s / 'gen8_analysis/fk_replay_inputs'
meta = json.loads((inputs / 'source.json').read_text())
audit_path = Path(meta['source_audit'])
audit = json.loads(audit_path.read_text())
assert hashlib.sha256(audit_path.read_bytes()).hexdigest() == meta['source_audit_sha256']
ns = audit['snapshot_stamp_ns']
stamp = rospy.Time(ns // 10**9, ns % 10**9)
archive = np.load(str(inputs / 'source.npz'))
source = dict(meta['source'])
source.update({key: archive[key].copy() for key in (
    'points_base', 'corners_base', 'center_base', 'tool_from_camera')})
source['arrival_projection'] = {key: archive[key].copy() for key in (
    'prism_base', 'rgb_valid', 'invalid_integral')}
source['arrival_projection']['audit'] = meta['arrival_audit']
recorded_fk = json.loads((inputs / 'original_fk_replay.json').read_text())['candidates'][0]
lattice_record = json.loads((s / 'gen8_analysis/full_generator_candidate/result.json').read_text())
transformer = tf.Transformer(True, rospy.Duration(200))
static = []
with rosbag.Bag(str(audit_path.parents[1] / 'keyframes.bag')) as bag:
    for topic, message, _ in bag.read_messages(topics=['/tf', '/tf_static']):
        if topic == '/tf_static':
            static.extend(message.transforms)
        else:
            for transform in message.transforms:
                transformer.setTransform(transform, 'recorded')
for transform in static:
    transform.header.stamp = stamp
    transformer.setTransform(transform, 'recorded-static')
translation, quaternion = transformer.lookupTransform('base_link', 'camera_link', stamp)
base_camera = fixture.base.quaternion_matrix([float(v) for v in quaternion])
base_camera[:3, 3] = translation
mount = source['tool_from_camera']
optical_basis = np.eye(4)
optical_basis[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
base_tool = (base_camera @ optical_basis) @ optical_basis.T @ np.linalg.inv(mount)
array_sha = lambda value: hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest()
print('REFERENCE_HASH_DIAGNOSTIC', array_sha(base_tool), source['audit']['frozen_tool_reference_sha256'], 'optical', array_sha(base_camera @ optical_basis), source['audit']['base_optical_sha256'], flush=True)
assert array_sha(base_tool) == source['audit']['frozen_tool_reference_sha256']
source['base_from_tool'] = base_tool
optical = np.eye(4)
optical[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
assert array_sha(base_camera @ optical) == source['audit']['base_optical_sha256']

patch = MonkeyPatch()
try:
    node, params = fixture.configured.__wrapped__(patch)
    params['/robot_description'] = Path(meta['model_path']).read_text()
    assert hashlib.sha256(params['/robot_description'].encode()).hexdigest() == meta['model_sha256']
    node._mode_selection = dict(audit['grasp_mode_selection'])
    node._mode_ready = True
    node._observe_return_phase = 'observation_prefix'
    node._observe_return_paused = False
    node._observe_return_trigger = 'BILATERAL_SURFACE_EVIDENCE_MISSING'
    node._observe_return_diagnostic_observation_only = False
    node._observe_return_view_gaps = tuple(audit['observe_return']['measured_missing_sides'])
    node._observe_return_view_uncertainty_m = .003
    node._observe_return_visited_camera_origins = ()
    node._current_stream_target_identity = lambda: NS(
        epoch=audit['target_epoch'], track_id=audit['observe_return']['target_track_id'])
    node._stream_condition = None
    node.near_field_planning_active = False
    node.grasp_config = observation_config({}, node._mode_selection)
    node.gripper_geometry = fixture.base.GripperGeometry(
        .05, .002, [.0434, .0286, .06], [.1175, .155, .0774], .003)
    node.gripper_tool_jaw_axis = 'y'
    node.gripper_tool_finger_length_axis = 'z'
    node.gripper_physical_open_width_m = .05
    node.observation_envelope_gate_enabled = True
    node.observation_camera_roll_offsets_deg = fixture.base.DEFAULT_OBSERVATION_CAMERA_ROLL_OFFSETS_DEG
    node.observation_camera_tilt_offsets_deg = fixture.base.DEFAULT_OBSERVATION_CAMERA_TILT_OFFSETS_DEG
    node._tool_from_camera_matrix = lambda: mount
    node._camera_intrinsics = lambda: NS(**source['camera'])
    node._observation_view_limit_configuration = lambda: source['view_limits']
    node._require_diagnostic_observation_view_source = lambda _prepared: source
    node._diagnostic_observation_view_source = source
    # This is the actual offline-generated FK endpoint, never an IK-positive hint.
    node._joint_space_observation_candidates = lambda _prepared: ((recorded_fk,), dict(
        execution_authority=False, source='original_fk_replay_first_candidate'))
    g = audit['replay_geometry']
    geometry = NS(center_base=np.array(g['obb_center_base_m']), axes_base=np.array(g['R_base_obb']),
        size_xyz_m=np.array(g['obb_size_xyz_m']), support_normal_base=np.array(g['support_normal_base']),
        support_offset_m=g['support_offset_m'])
    snapshot = NS(stamp_ns=ns, quality=NS(depth_repeatability_m=.001))
    prepared = NS(pose_estimator=NS(T_base_camera_link=base_camera), stamp=stamp,
                  geometry=geometry, snapshot=snapshot, ticket=None)
    reference = node._frozen_observation_reference_pose(prepared)
    uncertainty = audit['observation_failure_diagnostics']['observation_candidates'][0][
        'observation_side_evidence']['observation_side_uncertainty_m']
    # Contact-only placeholder offsets cannot change the overwritten pregrasp.
    # The original recorded side uncertainty is used by the actual side gate.
    profile = NS(depth_uncertainty_m=uncertainty, pregrasp_distance_m=.04,
                 approach_offset_m=.01, lift_height_m=.04)
    node._contact_stage_profile = lambda *a, **kw: profile
    grasp = fixture.base.make_pose_stamped('base_link', geometry.center_base,
        fixture.base.quaternion_from_euler(0., 0., 0.), stamp=stamp)
    binding = source['binding']
    prepared.ticket = NS(request_id=binding['request_id'], generation=binding['generation'],
        target_epoch=binding['target_epoch'], snapshot_stamp_sec=ns/1e9)
    prepared.snapshot.target_identity = NS(epoch=binding['target_epoch'], track_id=binding['target_track_id'])
    def require_prepared(p):
        assert p is prepared and p.snapshot.stamp_ns == binding['snapshot_stamp_ns']
        assert p.ticket.request_id == binding['request_id'] and p.ticket.generation == binding['generation']
        assert p.ticket.target_epoch == binding['target_epoch'] and p.snapshot.target_identity.track_id == binding['target_track_id']
    node._require_unknown_observation_prepared = require_prepared
    began = time.monotonic()
    passing, rejected = node._far_field_observation_variants(grasp, geometry,
        -geometry.support_normal_base, snapshot, reference, prepared, view_cache={})
    adapted_sec = time.monotonic()-began
    lattice = [v for v in passing if v.get('search_family') != 'joint_space_fk']
    fkrows = [v for v in passing if v.get('search_family') == 'joint_space_fk']
    assert len(lattice) == len(lattice_record['candidates']) == 218
    assert len(fkrows) == 1, [r for r in rejected if r.get('search_family') == 'joint_space_fk']
    def matrix_key(value):return tuple(np.asarray(value, dtype=float).reshape(-1))
    exact_lattice = set(matrix_key(fixture.base.pose_matrix(v['sequence'].pregrasp)) for v in lattice)
    recorded_lattice = set(matrix_key(v['base_from_tool']) for v in lattice_record['candidates'])
    assert exact_lattice == recorded_lattice
    assert np.max(np.abs(fixture.base.pose_matrix(fkrows[0]['sequence'].pregrasp)-recorded_fk['T_base_tool0'])) < 1e-12
    runtime = dict(prepared=prepared, grasp_pose=reference, observation_variants=passing)
    node._active_unknown_observation_check_runtime = runtime
    node._observation_information_comparison = dict(ticket=prepared.ticket, deadline=None)
    hint_calls=[]
    def no_hints_before_first(*a, **k):
        hint_calls.append(True)
        raise AssertionError('Advisory IK started before first FK dispatch')
    node._observation_endpoint_search_hints = no_hints_before_first
    schedule = node._order_observation_checks(passing)
    first = next(schedule)
    first_audit = first['observation_view']
    summary = runtime['unknown_observation_scheduling']
    assert first is fkrows[0] and not hint_calls
    assert not first_audit['check_scheduling_metrics']['endpoint_ik_hint']
    assert runtime['_first_fk_strict_target'] is first['sequence'].pregrasp
    result = dict(passed=True, python_executable=sys.executable, numpy_version=np.__version__, numpy_file=np.__file__, numerical_environment={k:os.environ.get(k) for k in ('PYTHONNOUSERSITE','OPENBLAS_NUM_THREADS','OMP_NUM_THREADS')}, source_root=str(root), lattice_count=len(lattice), fk_count=len(fkrows),
        candidate_count=len(passing), all_218_lattice_matrices_exactly_preserved=True,
        first_fk_pose_matches_original_recorded_proposal=True,
        first_dispatch_family=first['search_family'], zero_based_strict_dispatch_index=0,
        advisory_ik_queries_before_first_dispatch=0, injected_positive_ik_hints=0,
        requires_fk_positive_ik_hint=False,
        first_candidate_preference_index=first['preference_index'],
        first_candidate_original_schedule_index=first_audit['first_dispatch_original_check_index'],
        first_candidate_support_schedule=first_audit['check_scheduling_metrics']['measured_support_schedule'],
        first_candidate_information_utility=first_audit['information_utility'],
        first_candidate_observation_view=first_audit,
        scheduling=summary, adapter_and_endpoint_filter_elapsed_sec=adapted_sec,
        strongest_support_stratum=runtime['observation_first_strict']['support_priority'],
        original_strict_start_budget_sec=fixture.base.OBSERVATION_TILT_STRICT_BUDGET_SEC,
        original_strict_maximum_checks=fixture.base.OBSERVATION_TILT_MAX_STRICT_CHECKS,
        note_recorded_four_checks='GEN8 observed four starts under three-second budget; configured maximum is six, unchanged.',
        physical_actions=False, ros_service_calls=0, strict_rpc_calls=0,
        full_path_checked=False, motion_authority=False,
        import_sha256={str(Path(m.__file__).resolve()):hashlib.sha256(Path(m.__file__).read_bytes()).hexdigest()
            for m in (fixture.module, fixture.base)},
        input_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (
            inputs/'source.json', inputs/'source.npz', inputs/'original_fk_replay.json',
            s/'gen8_analysis/full_generator_candidate/result.json')},
        limitations=['Offline recorded binding checks replace live transport/lifetime observations; no live authority claim.',
            'Strict scheduling position is observed by yielding actual production dispatcher; no strict RPC or qualification success was invented.',
            'If first FK strict/path/following/source check fails, original deferred advisory IK/family fallback still follows within unchanged budgets.',
            'First FK may lead only in strongest measured-support stratum and fresh exact-ticket comparison; no source gate relaxed.'])
    schedule.close()
    assert '_first_fk_strict_target' not in runtime
    args.output.mkdir(exist_ok=False)
    result_path=args.output/'result.json'
    result_path.write_text(json.dumps(result,indent=2,default=str)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('first_candidate_observation_view','scheduling','input_sha256','import_sha256')},default=str),flush=True)
    print('RESULT_SHA256',hashlib.sha256(result_path.read_bytes()).hexdigest(),flush=True)
finally:
    patch.undo()
