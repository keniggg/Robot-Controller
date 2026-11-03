"""Bounded sealed-file diagnosis. No ROS node, SDK device, or calibration writes."""
from pathlib import Path
import hashlib
import json
import sys
from collections import Counter, defaultdict

import numpy as np
import rosbag
import pyrealsense2 as rs
from alicia_flexible_grasp.vision.rgbd_rectification import PairedPinholeRectification

WORK = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
REC = EXT/'grasp_records/20261010T050849.339487Z_fa4a10a8358a4fa2b80214f1f626b5c7'
ANALYSIS = EXT/'ros_log/full_latest_restart_20261010T002347Z/gen13_handoff_analysis'
OUT = ANALYSIS/'offline_depth_processing'
assert Path('/media/zhuyupei/SanStand').is_mount()
assert json.loads((REC/'manifest.json').read_text())['recorder_closed']
assert rs.__version__ == '2.56.5'


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def sha_file(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        while True:
            data = stream.read(2**20)
            if not data:
                break
            h.update(data)
    return h.hexdigest()


def plane_fit(points):
    center = points.mean(axis=0)
    centered = points-center
    _, vectors = np.linalg.eigh(centered.T@centered)
    normal = vectors[:, 0]
    return np.r_[normal, -normal@center]


metadata = []
depth_hashes = defaultdict(dict)
image_shapes = Counter()
topics = ['/grasp_record/camera_runtime_metadata', '/supervisor/camera/depth/image_raw',
          '/experiment/observation_rgbd/depth']
with rosbag.Bag(str(REC/'keyframes.bag')) as bag:
    topic_summary = {topic: dict(type=info.msg_type, count=info.message_count)
                     for topic, info in bag.get_type_and_topic_info().topics.items()}
    for topic, msg, receipt in bag.read_messages(topics=topics):
        if topic == topics[0]:
            payload = json.loads(msg.data)
            metadata.append((receipt.to_nsec(), payload))
        else:
            key = (msg.header.stamp.to_nsec(), msg.header.frame_id)
            depth_hashes[topic][key] = hashlib.sha256(msg.data).hexdigest()
            image_shapes[(topic, msg.width, msg.height, msg.encoding, msg.step)] += 1

camera = metadata[0][1]['parameters']
profile = camera['runtime_profile']
immutable_keys = ('alignment', 'color_intrinsics', 'native_depth_intrinsics', 'depth_to_color_extrinsics',
                  'geometry_intrinsics', 'geometry_intrinsics_source', 'geometry_activation_stamp_ns',
                  'depth_scale_m_per_unit', 'depth_scale_source', 'depth_filters', 'mode_depth_preset',
                  'rectification', 'color_projection_correction', 'sdk_version')
fingerprints = Counter(digest({key: data['parameters']['runtime_profile'].get(key)
                              for key in immutable_keys}) for _, data in metadata)
timeline = []
for receipt, data in metadata:
    temporal = data['parameters']['runtime_profile']['stationary_temporal']
    regions = temporal.get('regions', [])
    pair = temporal.get('pair_history', {})
    source = [r.get('visual', {}).get('device_timestamp_ms') for r in regions]
    timeline.append(dict(received_ns=receipt, filtered_fraction=temporal.get('filtered_fraction'),
                         filtered_pixel_count=temporal.get('filtered_pixel_count'),
                         regional_output_active=temporal.get('regional_output_active'),
                         region_count=len(regions), pair_history_epoch=pair.get('history_epoch'),
                         source_device_timestamp_ms=next((s for s in source if s is not None), None),
                         regions=[dict(name=r.get('name'), filter_applied=r.get('filter_applied'),
                                       history_reset=r.get('history_reset'), warming=r.get('warming'),
                                       quiet_span_sec=r.get('visual', {}).get('quiet_span_sec'),
                                       reason=r.get('visual', {}).get('reason')) for r in regions]))

original, copied = depth_hashes[topics[1]], depth_hashes[topics[2]]
common = sorted(set(original)&set(copied))
copy_comparison = dict(original_count=len(original), copied_count=len(copied),
                       shared_exact_header_count=len(common),
                       mismatches=[list(k) for k in common if original[k] != copied[k]])
assert not copy_comparison['mismatches']

# This invokes numerical SDK projection/deprojection only, never an SDK context/pipeline/device.
rect = PairedPinholeRectification(profile['color_intrinsics'], sdk=rs)
assert rect.contract == profile['rectification']
intr = rs.intrinsics()
source = rect.source_intrinsics
intr.width, intr.height = source['width'], source['height']
intr.fx, intr.fy, intr.ppx, intr.ppy = source['fx'], source['fy'], source['cx'], source['cy']
intr.model, intr.coeffs = getattr(rs.distortion, source['model']), source['coeffs']
pixels = np.c_[np.tile(np.arange(intr.width), intr.height), np.repeat(np.arange(intr.height), intr.width)]
exact_rays = np.array([rs.rs2_deproject_pixel_to_point(intr, p.tolist(), 1.) for p in pixels])
geom = rect.output_intrinsics
output_u, output_v = rect.target_indices % rect.width, rect.target_indices // rect.width
rounded_rays = np.c_[(output_u-geom['cx'])/geom['fx'], (output_v-geom['cy'])/geom['fy'],
                     np.ones(len(output_u))]
# The recorded planes and TF use camera_link (forward,left,up), not optical axes.
exact_rays = np.c_[exact_rays[:, 2], -exact_rays[:, 0], -exact_rays[:, 1]]
rounded_rays = np.c_[rounded_rays[:, 2], -rounded_rays[:, 0], -rounded_rays[:, 1]]

plane_report = json.loads((ANALYSIS/'support_plane_replay/report.json').read_text())
frames = []
for frame in plane_report['frames']:
    saved = np.load(frame['source_file'])
    raw = saved['depth_raw']
    z = raw.ravel()[rect.target_indices].astype(float)*float(saved['depth_scale'])
    # Exact inverse gather recovers aligned/filtered Z16 input, NOT native or unfiltered depth.
    recovered = raw.ravel()[rect.target_indices].reshape(intr.height, intr.width)
    canvas = np.zeros_like(raw)
    canvas.ravel()[rect.target_indices] = recovered.ravel()
    assert np.array_equal(canvas, raw)
    center = np.array(frame['ring_center_px'])
    radius = np.hypot(output_u-center[0], output_v-center[1])
    scale = min(rect.width/640., rect.height/480.)
    ring = (radius >= 70.*scale)&(radius <= frame['ring_outer_radius_px'])&(z >= .03)&(z <= 2.)
    rounded = rounded_rays*z[:, None]
    exact = exact_rays*z[:, None]
    original_plane = np.array(frame['plane_camera'])
    support = ring&(abs(rounded@original_plane[:3]+original_plane[3]) <= .002)
    assert support.sum() >= .69*frame['valid_ring_points']
    delta = (rounded-exact)[support]
    plane_rounded, plane_exact = plane_fit(rounded[support]), plane_fit(exact[support])
    if plane_rounded[:3]@plane_exact[:3] < 0:
        plane_exact = -plane_exact
    transform = saved['T_base_camera']
    base_rounded, base_exact = np.linalg.solve(transform.T, plane_rounded), np.linalg.solve(transform.T, plane_exact)
    eval_points = np.vstack([saved['anchor_base'], saved['footprint_base']])
    local = eval_points@(base_rounded-base_exact)[:3]+(base_rounded-base_exact)[3]
    frames.append(dict(source_stamp_ns=frame['source_stamp_ns'], source_npz=frame['source_file'],
                       source_npz_sha256=sha_file(frame['source_file']), depth_roundtrip_byte_exact=True,
                       retained_support_points=int(support.sum()),
                       support_depth_range_m=[float(z[support].min()), float(z[support].max())],
                       point_quantization_error_max_mm=float(np.linalg.norm(delta, axis=1).max()*1000),
                       point_signed_plane_error_max_abs_mm=float(abs(delta@original_plane[:3]).max()*1000),
                       point_signed_plane_error_mean_mm=float(np.mean(delta@original_plane[:3])*1000),
                       same_inlier_refit_plane_normal_delta_deg=float(np.rad2deg(np.arccos(np.clip(plane_rounded[:3]@plane_exact[:3], -1, 1)))),
                       same_inlier_refit_anchor_footprint_delta_max_mm=float(abs(local).max()*1000),
                       observed_reference_disagreement_mm=frame['reference_delta']['maximum_local_shift_mm']))

source_manifest = json.loads((REC/'source_manifest.json').read_text())['source_sha256']
sources = [
 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/realsense_manager.py',
 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/regional_stationary_depth.py',
 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/stationary_depth.py',
 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/rgbd_rectification.py',
 'src/alicia_grasp_modes/src/alicia_grasp_modes/tabletop.py',
]
source_checks = {path: dict(sha256=sha_file(WORK/path), recorded_sha256=source_manifest.get(path)) for path in sources}
for check in source_checks.values():
    check['matches_recorded_source'] = check['sha256'] == check['recorded_sha256']

report = dict(schema=1, disposition='no_verified_depth_processing_software_fault_in_gen13_sealed_evidence',
 record=str(REC), recorder_closed=True, metadata_count=len(metadata),
 profile_variants=dict(fingerprints), profile_consistent=len(fingerprints)==1,
 immutable_profile={k:profile.get(k) for k in immutable_keys if k != 'rectification'},
 temporal_configuration={k:v for k,v in profile['stationary_temporal'].items() if k not in
                         ('regions','pair_history','filtered_fraction','filtered_pixel_count')},
 metadata_time_association=metadata[0][1]['image_time_association'],
 topic_summary=topic_summary, image_shapes=[dict(topic=k[0],width=k[1],height=k[2],encoding=k[3],step=k[4],count=v)
                                          for k,v in image_shapes.items()],
 published_depth_vs_recording_copy=copy_comparison,
 rectification=dict(exact_runtime_contract_reproduced=True, contract_digest=rect.contract['digest'],
                    depth_forward_map_sha256=rect.contract['depth_forward_map_sha256'],
                    sdk_version=rs.__version__,sdk_module=str(rs.__file__),
                    frame_results=frames,
                    interpretation='Exact depth values preserved; measured nearest-cell geometry error and same-inlier plane refit cannot account for the 5.5 mm station disagreement.'),
 temporal_timeline=timeline, source_checks=source_checks,
 regression=dict(log=str(OUT/'offline_regression.log'),sha256=sha_file(OUT/'offline_regression.log'),
                 result='82 passed in 14.34 seconds',
                 files=['test_regional_stationary_depth.py','test_stationary_depth.py','test_rgbd_rectification.py']),
 conclusions=[
  'Native 640x480 SDK profile and published 653x489 pinhole canvas are consistent: +7,+6 principal point translation, unchanged focal lengths and depth scale.',
  'Ordinary full-frame temporal and hole-fill are disabled. Spatial remains enabled. Regional stationary temporal is independently gated, persistence=0.',
  'Recorded snapshots show no regional temporal output during each motion and new warming at the destination. Regional reset destroys histories even if manager pair_history_epoch does not change.',
  'Current support is independently fit from current processed depth before checking the immutable frozen plane. Its fit ROI follows projected frozen target anchor; differing stations sample different physical table regions.',
  'No observed byte mismatch between common-stamp supervisor depth and its observation recorder copy; no separate native/unfiltered pixel stream is in the sealed record.',
  'No justified code/gate/calibration change follows from this diagnosis. Spatial and SDK align effects on the original native measurements cannot be measured without the unrecorded pre-processing input.'
 ],
 limitations=[
  'Metadata are periodic diagnostic parameter snapshots, not source-exact image/filter provenance for every image.',
  'Numerically recovering pre-rectification Z16 does not undo SDK alignment, spatial filtering or regional temporal filtering.',
  'Passing synthetic regression tests does not validate physical depth accuracy or hand-eye/FK calibration.',
  'Same-inlier refit isolates rectification quantization; it is not a new support acceptance decision.',
  'Recorded planar measurements alone cannot distinguish depth sensor bias, surface shape, FK error and physical camera extrinsic error.'
 ],
 restrictions=dict(ros_master_access=False,device_access=False,temperature_read=False,motion=False,
                   production_changes=False,threshold_changes=False,calibration_changes=False))
OUT.mkdir(parents=True,exist_ok=True)
path=OUT/'report.json'
with path.open('x') as stream:
    json.dump(report,stream,ensure_ascii=False,indent=2)
    stream.write('\n')
print(json.dumps(dict(report=str(path),sha256=sha_file(path), profile_consistent=report['profile_consistent'],
                     depth_copy=copy_comparison, rectification_frames=frames),ensure_ascii=False))
