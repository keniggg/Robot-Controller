"""Offline GEN8 original endpoint gates, with exact recorded source transforms."""
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
base_tool = base_camera @ np.linalg.inv(mount)
array_sha = lambda value: hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest()
assert array_sha(base_tool) == source['audit']['frozen_tool_reference_sha256']
optical = np.eye(4)
optical[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
assert array_sha(base_camera @ optical) == source['audit']['base_optical_sha256']

patch = MonkeyPatch()
try:
    node, params = fixture.configured.__wrapped__(patch)
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
    # Live GEN8 generated no FK proposals. This replay isolates the complete
    # existing camera lattice gates; it does not substitute successful FK rows.
    node._joint_space_observation_candidates = lambda _prepared: ((), {})
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
    began = time.monotonic()
    passing, rejected = node._far_field_observation_variants(grasp, geometry,
        -geometry.support_normal_base, snapshot, reference, prepared, view_cache={})
    elapsed = time.monotonic() - began
    rows = [dict(preference_index=row['preference_index'],
        base_from_tool=fixture.base.pose_matrix(row['sequence'].pregrasp).tolist(),
        camera_target_distance_m=row['camera_target_distance_m'],
        camera_tilt_offset_deg=row['camera_tilt_offset_deg'],
        camera_roll_offset_deg=row['camera_roll_offset_deg'],
        observation_view=row['observation_view'],
        observation_side_evidence=row['observation_side_evidence'],
        gain=row.get('observe_return_missing_side_gain')) for row in passing]
    original = audit['observation_failure_diagnostics']['observation_candidates']
    matched = [all(any(np.allclose(old['base_from_tool'], row['base_from_tool'], atol=1e-10, rtol=0.)
                   for row in rows) for old in original)]
    result = dict(passed=matched[0], recorded_original_count=len(original),
        all_original_passing_poses_preserved=matched[0], passing_count=len(rows),
        rejected_count=len(rejected), elapsed_sec=elapsed,
        passing_tilt_counts=dict(Counter(round(row['camera_tilt_offset_deg'], 6) for row in rows)),
        rejection_counts=dict(Counter(row.get('failure_code', row.get('reason')) for row in rejected)),
        source_root=str(root), source_audit=str(audit_path),
        source_audit_sha256=meta['source_audit_sha256'], frozen_reference_hash_exact=True,
        physical_actions=False, full_path_checked=False,
        limitations=['Offline source lookup and ROS params are local dictionary stubs',
            'Recorded zero FK proposals retained; no planning, path or robot authority',
            'Only contact-only sequence offsets are placeholders; original endpoint gates and recorded side uncertainty are used'],
        imported_source_sha256={str(Path(module.__file__).resolve()):
            hashlib.sha256(Path(module.__file__).read_bytes()).hexdigest()
            for module in (fixture.module, fixture.base)}, candidates=rows, rejected=rejected)
    args.output.mkdir(exist_ok=False)
    (args.output / 'result.json').write_text(json.dumps(result, indent=2, default=str) + '\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('candidates', 'rejected')}, default=str), flush=True)
finally:
    patch.undo()
