"""Read-only finite provenance diagnostic; no ROS, production imports or motion.

Reuses closed recorded geometry and already saved qualifier arrays. Pixel
projection is diagnostic association, not replacement measurement/registration.
"""
from pathlib import Path
import hashlib
import json

import numpy as np
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
C = E / 'ros_log/full_latest_gen29_restart_20261016T000728Z'
R = E / 'grasp_records/20261016T021226.502516Z_0494c8bc84a34a74933e8eae122dd1f4'
I = C / 'verification/gen30_attempt1_closed_evidence_r1'
O = C / 'verification/gen31_first_conflict_provenance_r1'
assert E.parents[1].is_mount()
O.mkdir(exist_ok=False)
inputs = {}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path, numpy=False):
    inputs[str(path)] = sha(path)
    return np.load(path, allow_pickle=False) if numpy else json.loads(path.read_text())


def array_digest(array):
    array = np.ascontiguousarray(array, dtype='<f8')
    return hashlib.sha256(str(array.shape).encode('ascii') + b'\0' +
        array.dtype.str.encode('ascii') + b'\0' + array.tobytes()).hexdigest()


def stats(values):
    return dict(zip(('minimum', 'q25', 'median', 'q75', 'maximum'),
        np.quantile(values, [0., .25, .5, .75, 1.]).tolist()))


audit = read(R / 'keyframes_audits/07a8ac807d6a7f0702ef09c8866949f4ce96f8379dc1421dc6ae203af6086135.json')
initial = read(C / 'verification/gen31_initial_source_geometry_r1/result.json')
current_result = read(C / 'verification/gen31_actual_box_sources_r2/result.json')
arrays = read(C / 'verification/gen31_actual_box_sources_r2/1792116802511774778_conflict_arrays.npz', True)
prior = read(C / 'verification/gen31_first_outer_candidate_counterfactual_r1/complete_outer_rgb_prototype_result.json')
prior_qualifier = read(C / 'verification/gen31_first_outer_candidate_counterfactual_r1/full_qualifier_counterfactual.json')
raw_images = {stamp:read(I / (str(stamp) + '_raw.npz'), True)
              for stamp in (1792116749240888833, 1792116802511774778)}
fusion = audit['multiview_surface']['fused_scene_geometry']
points = np.asarray(audit['replay_geometry']['object_points_base_m'])
assert np.array_equal(points, arrays['points'])
raw_count = fusion['current_raw_point_count']
raw_points = points[:raw_count]
assert array_digest(raw_points) == fusion['current_raw_points_sha256']
assert array_digest(points) == fusion['enclosed_points_sha256']
assert fusion['point_union_policy'] == 'exact_xyz_keep_first_no_downsampling'

# The old and current saved models have identical axes/nominal box. The change
# was the explicitly recorded tangent uncertainty, not an altered threshold.
assert np.array_equal(arrays['axes'], np.asarray(prior['axes']))
assert np.max(np.abs(arrays['center'] @ arrays['axes'][:, :2] -
    (np.asarray(prior['lower']) + prior['upper']) / 2.)) < 1e-14
assert np.max(np.abs(arrays['size'][:2] -
    (np.asarray(prior['upper']) - prior['lower']))) < 1e-14
new_uncertainty = current_result['rows'][1]['audit']['outer_rgb_interval']['empirical_edge_uncertainty_m']
old_excess = arrays['excess'].copy()
old_excess[:, :2] += new_uncertainty - prior['edge_uncertainty']
assert abs(np.max(old_excess) - prior_qualifier['audit']['maximum_observed_model_excess_m']) < 1e-14
old_bad = np.any(old_excess > .001, axis=1)
new_bad = np.any(arrays['excess'] > .001, axis=1)
assert int(np.count_nonzero(old_bad)) == 59
assert int(np.count_nonzero(new_bad)) == 57
bad_ids = np.flatnonzero(old_bad)

T_registration = np.asarray(audit['multiview_surface']['latest_registration']['snapshot_to_reference_transform'])
phase_points = points @ T_registration[:3, :3].T + T_registration[:3, 3]
raw_distance, raw_nearest = cKDTree(raw_points).query(points)
initial_points = np.asarray(initial['replay_geometry']['object_points_base_m'])
initial_distance, initial_nearest = cKDTree(initial_points).query(phase_points)
K = np.array([initial['camera_parameters'][key] for key in ('fx', 'fy', 'cx', 'cy')])
scale = initial['camera_parameters']['depth_scale']
projections = {}
for name, stamp, T, positions in (
    ('initial', 1792116749240888833, np.asarray(initial['snapshot_transform']['T_base_optical']), phase_points),
    ('current', 1792116802511774778, np.asarray(audit['snapshot_transform']['T_base_optical']), points)):
    optical = (positions - T[:3, 3]) @ T[:3, :3]
    uv = optical[:, :2] / optical[:, 2, None] * K[:2] + K[2:]
    yx = np.rint(uv[:, ::-1]).astype(int)
    images = raw_images[stamp]
    yy, xx = yx[bad_ids].T
    assert np.all((yy >= 0) & (yy < images['depth_raw'].shape[0]) &
                  (xx >= 0) & (xx < images['depth_raw'].shape[1]))
    measured = images['depth_raw'][yy, xx] * scale
    in_mask = images['object_mask'][yy, xx] != 0
    valid = measured > 0.
    residual = optical[bad_ids, 2] - measured
    projections[name] = dict(stamp=stamp, uv=uv, yx=yx, measured=measured,
        in_mask=in_mask, valid=valid, residual=residual,
        summary=dict(pixel_lookup='nearest integer only; no filling or interpolation',
            mask_count=int(np.count_nonzero(in_mask)), valid_depth_count=int(np.count_nonzero(valid)),
            invalid_depth_count=int(np.count_nonzero(~valid)),
            depth_residual_definition='point optical Z minus original measured optical Z',
            valid_depth_residual_m=stats(residual[valid]),
            conflict_uv_min=np.min(uv[bad_ids], axis=0).tolist(),
            conflict_uv_max=np.max(uv[bad_ids], axis=0).tolist()))

point_rows = []
for ordinal, index in enumerate(bad_ids):
    row = dict(union_index=int(index), current_raw_member=bool(index < raw_count),
        conflict_in_new_model=bool(new_bad[index]), point_current_base_m=points[index].tolist(),
        point_phase_reference_base_m=phase_points[index].tolist(),
        old_excess_m=old_excess[index].tolist(), new_excess_m=arrays['excess'][index].tolist(),
        nearest_current_raw_m=float(raw_distance[index]), nearest_current_raw_index=int(raw_nearest[index]),
        nearest_initial_new_single_frame_m=float(initial_distance[index]),
        nearest_initial_new_single_frame_index=int(initial_nearest[index]))
    for name, pr in projections.items():
        row[name] = dict(uv=pr['uv'][index].tolist(), nearest_yx=pr['yx'][index].tolist(),
            mask_member=bool(pr['in_mask'][ordinal]), raw_depth_valid=bool(pr['valid'][ordinal]),
            depth_residual_m=float(pr['residual'][ordinal]) if pr['valid'][ordinal] else None)
    point_rows.append(row)

fig, axes = plt.subplots(1, 2, figsize=(13, 6))
for ax, name, bounds in zip(axes, ('initial', 'current'),
                           ((250, 385, 290, 180), (165, 345, 450, 320))):
    pr = projections[name]; im = raw_images[pr['stamp']]
    ax.imshow(im['color_bgr'][:, :, ::-1])
    ax.contour(im['object_mask'] != 0, levels=[.5], colors=['cyan'], linewidths=.8)
    uv = pr['uv'][bad_ids]
    ax.scatter(uv[:, 0], uv[:, 1], s=13, c='#ff6600', edgecolors='black', linewidths=.2,
               label='59 historical-fusion conflict points')
    ax.set_xlim(bounds[:2]); ax.set_ylim(bounds[2:]); ax.set_aspect('equal')
    ax.set_title(name + ' original RGB / measured mask (cyan)\n' + str(pr['stamp']))
    ax.set_xlabel('pixel x'); ax.set_ylabel('pixel y')
axes[0].legend(loc='lower left', fontsize=8)
fig.suptitle('Diagnostic projection using recorded registration; no motion or fresh-scene authority')
fig.tight_layout(); fig.savefig(O / 'conflict_provenance_overlay.png', dpi=160); plt.close(fig)

new_sources = [W / 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/fused_scene_geometry.py',
               W / 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/multiview_surface.py']
report = dict(mode='finite_closed_evidence_provenance_only', production_changed=False,
    no_ros_or_motion=True, source_stamp_ns=1792116802511774778,
    inputs_sha256=inputs, helper_sha256=sha(__file__),
    interpretation_sources_sha256={str(p):sha(p) for p in new_sources},
    exact_current_raw_and_union_hash_verified=True,
    current_raw_point_count=raw_count, union_point_count=len(points),
    old_model=dict(count=59, maximum_excess_m=float(np.max(old_excess)),
        edge_uncertainty_m=prior['edge_uncertainty'], original_conflict_threshold_m=.001),
    new_saved_model=dict(count=57, maximum_excess_m=float(np.max(arrays['excess'])),
        edge_uncertainty_m=new_uncertainty, original_conflict_threshold_m=.001),
    count_difference_indices=np.flatnonzero(old_bad & ~new_bad).tolist(),
    conflict_current_raw_count=int(np.count_nonzero(old_bad[:raw_count])),
    conflict_fused_union_tail_count=int(np.count_nonzero(old_bad[raw_count:])),
    current_raw_maximum_excess_by_axis_m=np.max(old_excess[:raw_count], axis=0).tolist(),
    nearest_current_raw_distance_for_59_m=stats(raw_distance[old_bad]),
    fused_tail_numerically_current_raw_matches=dict(count=int(np.count_nonzero(raw_distance[raw_count:] < 1e-12)),
        numerical_tolerance_m=1e-12, exact_union_duplicates=fusion['exact_duplicates_removed'],
        recorded_current_representative_count=fusion['per_view_representative_point_counts'][-1],
        meaning='Floating-point roundtrip association, not recovered source-index vector.'),
    nearest_initial_new_single_frame_for_59_m=stats(initial_distance[old_bad]),
    initial_new_single_frame_matches_within_1e_12_m=int(np.count_nonzero(initial_distance[old_bad] < 1e-12)),
    initial_new_single_frame_matches_within_1mm=int(np.count_nonzero(initial_distance[old_bad] < .001)),
    projections={name:pr['summary'] for name,pr in projections.items()},
    recorded_registration_current_to_phase_reference=T_registration.tolist(),
    conclusions=[
      'All 59 old-budget conflicts (and all 57 current-budget conflicts) are historical fused-union additions; none are current raw geometry points.',
      'No conflict can be a numerical roundtrip of the current raw point set: its nearest current measured geometry point is several millimetres away. Fusion retains measured representatives rather than averaging them.',
      'All 59 project inside the original initial-view mask after recorded registration; 51 have nearest original depth with residual between about -0.60 and +0.62 mm. This is evidence of historical edge measurement association, not proof of independent exact source provenance.',
      'All 59 project outside the current-view mask; 54 nearest current pixels have no valid depth. Unknown depth does not establish free space. The 5 valid current pixels measure about 7.45–10.46 mm behind the historical point along the optical ray.',
      'The recorded geometry therefore exhibits a cross-view boundary disagreement; these observations alone do not establish a real protrusion, a shadow artefact, or which calibration/registration/noise mechanism caused it.'],
    limitations=[
      'Per-point fused view_indices/view_sources vectors were not recorded; only their hashes/counts were retained. Initial versus first-observation-previous exact membership cannot be recovered from summaries alone.',
      'Initial geometry used for nearest-neighbour comparison is a new 1268-point single-frame reconstruction, not the original temporal request/reference.',
      'Recorded camera metadata was stable across samples but not synchronized to each image, as documented by the initial reconstruction.',
      'Current raw means pre-cross-view geometry from its temporal RGB-D snapshot, not every unprocessed sensor pixel.',
      'Point projection/depth association is a diagnostic only, not a new measurement, model approval, permissible crop or registration correction.',
      'No threshold, measurement, producer source or runtime state was changed.'],
    per_point=point_rows)
(O / 'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
np.savez_compressed(O / 'provenance_arrays.npz', points=points, raw_points=raw_points,
    phase_points=phase_points, old_excess=old_excess, new_excess=arrays['excess'],
    conflict_indices=bad_ids, current_raw_distance=raw_distance, initial_distance=initial_distance,
    initial_uv=projections['initial']['uv'], current_uv=projections['current']['uv'])
receipt = {p.name:sha(p) for p in O.iterdir() if p.is_file()}
(O / 'receipt.json').write_text(json.dumps(receipt, indent=2, sort_keys=True) + '\n')
print(json.dumps(dict(path=str(O / 'result.json'), sha256=sha(O / 'result.json'),
    projection_summary=report['projections'], conflict_raw_count=report['conflict_current_raw_count'],
    conflict_fused_count=report['conflict_fused_union_tail_count']), indent=2))
