"""Bounded offline reconstruction of the sealed GEN29 initial direct snapshot.

No node, ROS master, publisher, service, generator, planner or robot access.
Original request-1 box audit was not retained: never call this exact runtime replay.
"""
import hashlib
import json
import os
import sys
import time
from pathlib import Path
from types import SimpleNamespace as NS

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M / '实验记录/Robot-Controller'
N = E / 'ros_log/full_latest_gen22_restart_20261015T003700Z'
R = E / 'grasp_records/20261015T111226.886093Z_0a4de2be16394344b0fa84a6ad2f8318'
I = N / 'verification/gen29_attempt1_shadow_frames'
O = N / 'verification/gen30_initial_view_box_replay_r1'
IDS = (1792062749691942453, 1792062749893290758, 1792062750092055082)
SELECTION = 1792062749180071115
assert M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY
assert not O.exists()
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:1', PYTHONDONTWRITEBYTECODE='1',
                  OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
for path in ('src/alicia_flexible_grasp_supervisor/src', 'devel/lib/python3/dist-packages'):
    sys.path.insert(0, str(W / path))

import numpy as np
import yaml
import rosbag
import genpy
from alicia_flexible_grasp.vision.rgbd_snapshot import RgbdSample, fuse_stable_samples
from alicia_flexible_grasp.vision.object_geometry import estimate_object_geometry
from alicia_flexible_grasp.vision.regular_box_inference import (
    qualify_regular_box_snapshot, qualify_regular_box_top)
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def arrsha(value):
    return hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def serial(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


inputs = {}


def bound(path):
    inputs[str(path)] = sha(path)
    return read(path)


manifest = bound(R / 'manifest.json')
assert inputs[str(R / 'manifest.json')] == '3a408e2957aeb399b9c8b3761a60dff9cd89bc616c26505e20a742140bdbfa9c'
outcome = bound(R / 'recorder_outcome.json')
assert manifest['state'] == 'complete' and manifest['recorder_closed']
assert manifest['result'] == 'failure' and outcome['bag_closed'] and not outcome['interrupted']
bag = R / 'keyframes.bag'
entry = next(row for row in manifest['files'] if row['path'] == bag.name)
bag_stat = bag.stat()
assert bag_stat.st_size == entry['size_bytes'] and sha(bag) == entry['sha256']
release = bound(N / 'verification/gen29_release_manifest.json')
assert inputs[str(N / 'verification/gen29_release_manifest.json')] == '1551ea57d087b221fed4f6d59c8ce176c7a2e2d5398aa306829af3aa51506666'
source_paths = [
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/rgbd_snapshot.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/object_geometry.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/regular_box_inference.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/target_observation.py',
    'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py',
    'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py',
    'src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml',
    'src/alicia_flexible_grasp_supervisor/config/camera.yaml']
sources = {p: sha(W / p) for p in source_paths}
for path, digest in sources.items():
    assert release['source_sha256'][path] == digest, ('source differs from deployed GEN29', path)

audit_path = R / 'keyframes_audits/a5d23494065e6623395495ae3f128b4eee0a3c7c7ef43adce50e22ee019b2be0.json'
audit = bound(audit_path)
trigger = audit['observe_return']['trigger_evidence']
assert audit['request_id'] == 2 and audit['planning_origin'] == 'observe_return_observation_prefix'
assert trigger['request_id'] == 1 and trigger['snapshot_stamp_ns'] == IDS[-1]
assert trigger['selection']['generation'] == 11 and trigger['selection']['stamp_ns'] == SELECTION
assert trigger['materialized_count'] == 0 and trigger['tabletop_geometry_preflight_completed']
identity = TargetTrackIdentity(trigger['target_epoch'], trigger['target_track_id'])
export = bound(I / 'result.json')
assert inputs[str(I / 'result.json')] == '0fad4e0e41bfe967131b6c09bf9bc33df575d3d045c111a6055fa70d05093c23'
assert export['initial_requested'] == list(IDS) and not export['initial_missing']
assert export['source_bag_sha256'] == entry['sha256']
bound(R / 'perception_execution_configuration.json')
policy = bound(N / 'gen29_attempt_1_scene_r2/startup_policy.json')
assert policy['parameters']['/grasp/unknown_require_measured_bilateral_contact'] is False

# Exact archived pose brackets only. No image or source substitution occurs.
joint_rows, objects = [], {}
topics = ['/alicia_d/accepted_joint_states', '/perception/object']
with rosbag.Bag(str(bag)) as stream:
    for topic, msg, rec in stream.read_messages(topics=topics,
            start_time=genpy.Time((IDS[0]-10**9)//10**9, (IDS[0]-10**9)%10**9),
            end_time=genpy.Time((IDS[-1]+10**9)//10**9, (IDS[-1]+10**9)%10**9)):
        ns = msg.header.stamp.to_nsec()
        if topic == topics[0] and msg.header.frame_id == 'sdk_measured':
            joints = dict(zip(msg.name, msg.position))
            joint_rows.append(dict(source_ns=ns, received_ns=rec.to_nsec(),
                frame_id=msg.header.frame_id, positions_rad=[joints['Joint%d' % i] for i in range(1, 7)]))
        elif topic == topics[1] and ns in IDS:
            assert ns not in objects
            objects[ns] = msg
assert set(objects) == set(IDS) and joint_rows

cfg = yaml.safe_load((W / source_paths[-2]).read_text())['grasp_6d']['remote']
perception = yaml.safe_load((W / source_paths[-1]).read_text())['perception']
old_parameters = bound(N / 'gen28_repositioned_initial_readonly_inputs/parameters.json')['before']['parameters']
rows, samples, arrays = [], [], []
for ns in IDS:
    directory = I / ('source_%d' % ns)
    meta = bound(directory / 'metadata.json')
    assert meta == next(row for row in export['rows'] if row['source_stamp_ns'] == ns)
    status = bound(directory / 'detector_status.json')['payload']
    assert inputs[str(directory / 'detector_status.json')] == meta['files_sha256'][str(directory / 'detector_status.json')]
    assert status['generation'] == 11 and status['selection_stamp_ns'] == SELECTION
    assert status['source_stamp_ns'] == ns and status['state'] == 'ready' and not status['target_lost']
    path = directory / 'raw_arrays.npz'
    assert sha(path) == meta['files_sha256'][str(path)]
    inputs[str(path)] = sha(path)
    with np.load(path, allow_pickle=False) as archive:
        a = {key: archive[key].copy() for key in archive.files}
    assert all(arrsha(a[key]) == digest for key, digest in meta['array_sha256'].items())
    assert int(a['source_stamp_ns']) == ns
    assert all(row['stamp_ns'] == ns and row['frame_id'] == 'camera_link'
               for row in meta['topic_headers'].values())
    obj = objects[ns]
    bbox = (obj.bbox_x, obj.bbox_y, obj.bbox_width, obj.bbox_height)
    assert obj.detected and list(bbox) == meta['object']['bbox_xywh']
    before = max((r for r in joint_rows if r['source_ns'] <= ns), key=lambda r: r['source_ns'])
    after = min((r for r in joint_rows if r['source_ns'] >= ns), key=lambda r: r['source_ns'])
    assert ns-before['source_ns'] < 200000000 and after['source_ns']-ns < 200000000
    np.testing.assert_array_equal(before['positions_rad'], after['positions_rad'])
    q = np.asarray(before['positions_rad'])
    samples.append(RgbdSample(a['color_bgr'], a['depth_raw'], a['object_mask'], bbox,
        obj, ns/1e9, 'camera_link', q, ns, identity.epoch, identity))
    arrays.append(a)
    rows.append(dict(source_stamp_ns=ns, input_npz=str(path), input_sha256=inputs[str(path)],
        array_sha256=meta['array_sha256'], bbox_xywh=bbox, topic_headers=meta['topic_headers'],
        accepted_joint_bracket=dict(before=before, after=after),
        mask_pixels=meta['mask_pixels'], target_mask_depth=meta['target_mask_depth']))
assert np.max(np.ptp(np.array([x.joint_positions for x in samples]), axis=0)) == 0.
for a in arrays[1:]:
    np.testing.assert_array_equal(a['intrinsics'], arrays[0]['intrinsics'])
    np.testing.assert_allclose(a['T_base_camera_link'], arrays[0]['T_base_camera_link'], rtol=0, atol=1e-12)

scale = float(arrays[-1]['depth_scale_m'])
intr = arrays[-1]['intrinsics']
K = NS(fx=intr[0], fy=intr[1], cx=intr[2], cy=intr[3],
       width=arrays[-1]['depth_raw'].shape[1], height=arrays[-1]['depth_raw'].shape[0])
camera_from_optical = np.eye(4)
camera_from_optical[:3, :3] = [[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]]
T = arrays[-1]['T_base_camera_link'] @ camera_from_optical
sk = dict(require_mask=True, min_mask_iou=cfg['planning_mask_min_iou'],
    max_centroid_shift_px=cfg['planning_mask_max_centroid_shift_px'],
    max_joint_delta_rad=cfg['planning_max_joint_delta_rad'], erosion_px=cfg['mask_erosion_px'],
    depth_scale=scale, depth_min_m=perception['depth_min_m'], depth_max_m=perception['depth_max_m'],
    mad_scale=cfg['depth_mad_scale'], mad_absolute_floor_m=cfg['depth_mad_absolute_floor_m'],
    internal_hole_max_area_px=0)
started = time.monotonic()
snapshot = fuse_stable_samples(samples, **sk)
assert snapshot.ok, (snapshot.failure_code, snapshot.failure_reason)
assert list(snapshot.sample_stamp_ns) == list(IDS)
gg = cfg['gripper_geometry']
gk = dict(support_bbox_expand_ratio=cfg.get('support_bbox_expand_ratio', .30),
    support_distance_threshold_m=cfg.get('support_distance_threshold_m', cfg.get('target_cloud_support_plane_inlier_distance_m', .004)),
    voxel_size_m=cfg['target_cloud_voxel_size_m'], min_support_points=cfg.get('geometry_min_support_points', 200),
    min_object_points=cfg.get('geometry_min_object_points', cfg.get('target_cloud_min_points', 120)),
    min_size_m=cfg.get('geometry_min_size_m', .005), max_size_m=cfg.get('geometry_max_size_m', .600),
    max_height_m=cfg.get('geometry_max_height_m', .500), previous_axes_base=None,
    outlier_neighbors=cfg['target_cloud_outlier_neighbors'], outlier_std_ratio=cfg['target_cloud_outlier_std_ratio'],
    support_minimum_span_m=max(gg['finger_box_xyz_m'][0], gg['max_inner_gap_m']+2*gg['finger_box_xyz_m'][1])+2*gg.get('support_clearance_m', .003))
geometry = estimate_object_geometry(snapshot.depth_raw, snapshot.target_depth_raw,
    snapshot.object_mask, snapshot.bbox, K, scale, T, snapshot.source_mode, **gk)
assert geometry.ok, (geometry.failure_code, geometry.failure_reason)
point_hash = arrsha(geometry.object_points_base)


def qualify(call):
    local = {}
    def trace(frame, event, arg):
        if frame.f_code is qualify_regular_box_top.__code__ and event == 'return':
            v = frame.f_locals
            local.update(visual_rectangle_found=v.get('visual') is not None,
                top_point_count=len(v['top']) if 'top' in v else None,
                source_valid_fraction=v.get('valid_fraction'))
        return trace
    previous = sys.gettrace()
    try:
        sys.settrace(trace)
        result = call()
    finally:
        sys.settrace(previous)
    return dict(ok=result.ok, code=result.code, audit=dict(result.audit),
        return_local_diagnostics=local,
        model=result.model.to_evidence_dict() if result.ok else None)


production_entry = qualify(lambda: qualify_regular_box_snapshot(snapshot, K, scale, T,
    geometry, identity, snapshot.stamp_ns))
raw_results = []
for ns, a in zip(IDS, arrays):
    raw_results.append(dict(source_stamp_ns=ns, qualification=qualify(lambda a=a, ns=ns:
        qualify_regular_box_top(a['depth_raw'], a['object_mask'], K, scale,
            a['T_base_camera_link'] @ camera_from_optical, geometry, identity, ns, color_bgr=a['color_bgr']))))
assert arrsha(geometry.object_points_base) == point_hash
assert sources == {p: sha(W/p) for p in source_paths}
for path, digest in inputs.items():
    assert sha(path) == digest
after = bag.stat()
assert (bag_stat.st_ino, bag_stat.st_size, bag_stat.st_mtime_ns) == (after.st_ino, after.st_size, after.st_mtime_ns)
direction = trigger['missing_surface_direction']
comparison = dict(center_max_abs_error_m=float(np.max(np.abs(geometry.center_base-np.asarray(direction['target_center_base_m'])))),
    narrow_size_error_m=float(abs(min(geometry.size_xyz_m[:2])-direction['whole_obb_width_m'])),
    height_error_m=float(abs(geometry.size_xyz_m[2]-direction['object_height_m'])))
O.mkdir(); (O/'tmp').mkdir()
report = dict(classification='offline_initial_three_frame_reconstruction_not_retained_original_qualification',
    record=str(R), bag_sha256=entry['sha256'], bag_hash_verified=True,
    original_initial_trigger_audit=str(audit_path), original_trigger=trigger,
    original_request_id=1, first_observation_contact_request_26_used=False,
    selected_original_sources=list(IDS), frame_bindings=rows,
    snapshot_arguments=sk, geometry_arguments=gk, source_identity=dict(epoch=identity.epoch, track_id=identity.track_id),
    configuration_basis='GEN29-sealed YAML and producer defaults; unknown mode hole fill is zero',
    old_gen28_parameters_corroboration_only={k: dict(yaml=cfg[k], older_capture=old_parameters['/grasp_6d/remote'].get(k))
        for k in ('planning_mask_min_iou', 'planning_mask_max_centroid_shift_px', 'planning_max_joint_delta_rad',
                  'mask_erosion_px', 'depth_mad_scale', 'depth_mad_absolute_floor_m', 'target_cloud_voxel_size_m')},
    geometry=dict(center_base=geometry.center_base, axes_base=geometry.axes_base, size_xyz_m=geometry.size_xyz_m,
        support_normal_base=geometry.support_normal_base, support_offset_m=geometry.support_offset_m,
        support_inlier_ratio=geometry.support_inlier_ratio, object_points_count=len(geometry.object_points_base), object_points_sha256=point_hash),
    comparison_to_original_trigger=comparison, actual_production_fuser_called=True,
    actual_production_box_snapshot_qualifier_called=True, production_entry=production_entry,
    raw_frames_with_same_fused_geometry=raw_results,
    source_sha256=sources, input_sha256=inputs, script_sha256=sha(__file__), source_unchanged=True,
    elapsed_sec=time.monotonic()-started, no_ros_calls=True, no_hardware_access=True,
    no_temperature_reads=True, generator_or_ik_called=False, physical_grasp_verified=False,
    limitations=[
        'Original request-1 regular_box qualification audit was not retained; recorded trigger only proves preflight zero candidates and measured bilateral deficit.',
        'Three exact initial source frames are acquisition through recorded initial source; original request-1 buffer membership was not explicitly retained.',
        'Full contemporaneous remote effective-parameter snapshot is absent. Sealed YAML/defaults and unknown-mode override are explicit; GEN28 parameter capture is only corroboration.',
        'No previous object axes or asynchronous admission schedule is reconstructed; geometry agreement is reported numerically, not assumed.',
        'Source poses come from sealed TF export; real SDK brackets are stationary but do not reproduce the precise online callback pairing.',
        'First two per-frame qualifiers use the same reconstructed three-frame geometry as a controlled pixel comparison, not standalone single-frame geometry.',
        'No generator, full collision trajectory, MoveIt/IK, runtime source authority or robot action is evaluated.'])
path = O/'result.json'
path.write_text(json.dumps(report, ensure_ascii=False, indent=2, default=serial, allow_nan=False)+'\n')
print(json.dumps(dict(path=str(path), sha256=sha(path), result=production_entry,
    comparison=comparison, elapsed_sec=report['elapsed_sec']), ensure_ascii=False, default=serial))
