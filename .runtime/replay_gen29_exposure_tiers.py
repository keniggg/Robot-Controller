"""Bounded offline proposal comparison; no ROS operations or production edits.

The counterfactual changes only the optimizer's requested exposure. Actual
endpoint acceptance still calls every production gate with original noise.
"""
import argparse
import hashlib
import inspect
import itertools
import json
from pathlib import Path
import time

import numpy as np
from alicia_grasp_modes import reachable_observation_views as views
from alicia_grasp_modes import observation_arrival_projection as projection
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def ahash(value):
    return hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest()


p = argparse.ArgumentParser()
p.add_argument('--manifest', required=True)
p.add_argument('--output', required=True)
args = p.parse_args()
meta_path = Path(args.manifest)
meta = json.loads(meta_path.read_text())
frame = next(row for row in meta['frames'] if row['source_stamp_ns'] == 1791549916246385812)
out = Path(args.output)
out.mkdir(parents=True, exist_ok=True)
assert not (out/'result.json').exists()
audit_path = Path(meta['record'])/'keyframes_audits/8b168b430bcec6b8fac454723d8f4ff176df9d98032009a3c614dc11137c6bdf.json'
audit = json.loads(audit_path.read_text())
selected = audit['selected']
source_audit = selected['observation_source_audit']
search = selected['observation_view']['proposal_search']
raw_path = Path(frame['npz'])
raw = np.load(raw_path)
camera = frame['camera']
status = frame['ready_status']
if isinstance(status, list):
    assert len(status) == 1
    status = status[0]
if 'message' in status:
    status = status['message']
stamp = audit['snapshot_stamp_ns']
assert status['state'] == 'ready' and status['source_stamp_ns'] == stamp
assert ahash(raw['depth_raw']) == source_audit['raw_depth_sha256']
assert ahash(raw['raw_mask']) == source_audit['raw_mask_sha256']
assert ahash(raw['tool_from_camera']) == source_audit['tool_camera_sha256']
g = audit['replay_geometry']
depth, mask, T = raw['depth_raw'], raw['raw_mask'], raw['T_base_camera']
ys, xs = np.nonzero((depth > 0) & (mask == 0))
z = depth[ys, xs].astype(float)*camera['depth_scale']
optical_points = np.column_stack(((xs-camera['cx'])*z/camera['fx'],
    (ys-camera['cy'])*z/camera['fy'], z))
C = np.eye(4)
C[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
optical = T@C
points = optical_points@optical[:3, :3].T+optical[:3, 3]
normal, offset = np.asarray(g['support_normal_base']), g['support_offset_m']
support = points[np.abs(points@normal+offset) <= .002]
assert len(support) == source_audit['support_point_count']
center, R, size = map(np.asarray, (g['obb_center_base_m'], g['R_base_obb'], g['obb_size_xyz_m']))
corners = center+(np.asarray(list(itertools.product((-.5, .5), repeat=3)))*size)@R.T
assert ahash(corners) == source_audit['conservative_target_corners_sha256']
selection = dict(mode='unknown', strategy='observe_return',
    generation=status['generation'], stamp_ns=status['selection_stamp_ns'])
contract = projection.capture_arrival_projection(status, selection, stamp, camera)
assert contract['audit'] == source_audit['arrival_projection']
source = dict(binding=selected['observation_source_binding'],
    camera=source_audit['camera_parameters'], points_base=support,
    corners_base=corners, center_base=center, tool_from_camera=raw['tool_from_camera'],
    geometry=dict(support_normal_base=normal, support_offset_m=offset,
        R_base_obb=R, obb_size_xyz_m=size), view_limits=source_audit['view_limits'],
    arrival_projection=contract)
model_path = Path('.runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf')
fk = SerialUrdfFk(model_path.read_text(), ['Joint%d' % i for i in range(1, 7)])
assert fk.model_sha256 == search['model_sha256']
common = dict(source=source, current_joint_positions=search['current_joint_positions'], fk=fk,
    gripper=GripperGeometry(.05, .002, np.array([.0434, .0286, .06]), np.array([.1175, .155, .0774]), .003),
    opening_width_m=.05, gaps=audit['observe_return']['measured_missing_sides'],
    depth_uncertainty_m=.003, side_depth_uncertainty_m=search['optical_side_uncertainty_m'],
    joint_error_bounds_rad=search['joint_error_bounds_rad'], max_directions=2,
    max_iterations=96, max_seconds=2., proposal_image_reserve_px=4.)
paths = [Path(views.__file__), Path(projection.__file__), model_path]
before = {str(path): sha(path) for path in paths}
original = inspect.getsource(views.generate_reachable_observation_views)
old = 'required = (previous+depth_uncertainty_m+.0002)/object_height'
assert original.count(old) == 1
replacement = 'required = (previous+2.*depth_uncertainty_m+.0002)/object_height'
modified = original.replace(old, replacement)
namespace = dict(vars(views))
exec(compile(modified, '<offline_two_noise_exposure_proposal>', 'exec'), namespace)
comparison = []
for label, fn in [('original_1x', views.generate_reachable_observation_views),
                  ('proposal_2x_original_acceptance', namespace['generate_reachable_observation_views'])]:
    started = time.perf_counter()
    candidates, report = fn(**common)
    summaries = []
    for row in candidates:
        checks = row['endpoint_checks']
        summaries.append(dict(gap_index=row['direction_gap_index'],
            joint_max_delta_rad=row['joint_max_delta_rad'],
            gain_m=checks['missing_side_view_gain']['predicted_new_side_gain_m'],
            gain_noise_m=checks['missing_side_view_gain']['depth_uncertainty_m'],
            target_side=checks['missing_side_view_gain']['missing_side_sign'],
            optical_projection_m=checks['optical_side_evidence']['observation_projected_side_evidence_m'],
            prism_margin_px=checks['arrival_projection_endpoint']['minimum_image_edge_distance_px']))
    row = dict(label=label, wall_sec=time.perf_counter()-started,
        candidates=candidates, report=report, summaries=summaries)
    comparison.append(row)
    print(json.dumps(dict(label=label, wall_sec=row['wall_sec'], count=len(candidates),
        status=report['status'], summaries=summaries)), flush=True)
after = {str(path): sha(path) for path in paths}
assert after == before
result = dict(scope='OFFLINE_PROPOSAL_ENDPOINT_COMPARISON_NO_MOTION_AUTHORITY',
    audit_path=str(audit_path), audit_sha256=sha(audit_path),
    source_stamp_ns=stamp, source_manifest=str(meta_path), source_manifest_sha256=sha(meta_path),
    source_npz_sha256=sha(raw_path), production_before=before, production_after=after,
    support_point_count=len(support), support_points_sha256=ahash(support),
    original_support_points_sha256=source_audit['support_points_sha256'],
    support_hash_exact=ahash(support) == source_audit['support_points_sha256'],
    arrival_projection_audit_exact=True, counterfactual_edit=dict(before=old, after=replacement),
    script_sha256=sha(__file__), comparison=comparison,
    limitations=['Exact raw source and immutable prism audit; TF reconstruction may change float point hashes.',
        'Each independent policy was given the unchanged 2 s, two-direction, one-seed, 96-iteration budget.',
        'Prediction is exposure, not a guarantee of common contact height or future measured points.',
        'No whole-arm MoveIt/strict path/following, physical arrival or subsequent contact is established.',
        'The two-policy experiment is not a deployed shared-budget tier scheduler.'])
(out/'result.json').write_text(json.dumps(result, indent=2)+'\n')
