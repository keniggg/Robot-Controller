"""Finite read-only geometry diagnosis from saved GEN30 trace and original masks.

No ROS imports, qualification replay, production edits, depth synthesis or authority.
The alternative intersections are diagnostic coordinates, never accepted box models.
"""
import hashlib
import itertools
import json
import math
import os
import time
from pathlib import Path

os.environ.update(OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1",
                  PYTHONDONTWRITEBYTECODE="1", ROS_MASTER_URI="http://127.0.0.1:1")
import cv2
import numpy as np

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
N = M / '实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z'
O = N / 'verification/gen30_initial_visual_axes_r1'
TRACE = N / 'verification/gen30_initial_view_visual_trace_r1/result.json'
SOURCE = W / 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/regular_box_inference.py'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def project(H, points):
    q = np.column_stack((points, np.ones(len(points)))) @ H.T
    return q[:, :2] / q[:, 2:]


def homography(metric, pixels):
    a, b = [], []
    for (x, y), (u, v) in zip(metric, pixels):
        a.extend(([x, y, 1, 0, 0, 0, -u*x, -u*y],
                  [0, 0, 0, x, y, 1, -v*x, -v*y]))
        b.extend((u, v))
    h = np.linalg.lstsq(a, b, rcond=None)[0]
    return np.r_[h, 1.].reshape(3, 3)


def coverage(mask, points):
    out = []
    for xy in points:
        x, y = np.rint(xy).astype(int)
        assert min(x, y, mask.shape[1]-1-x, mask.shape[0]-1-y) >= 4
        count = int(mask[y-2:y+3, x-2:x+3].sum())
        out.append(dict(rounded_xy=[int(x), int(y)], original_mask_pixels_in_5x5=count,
                        original_quadrant_normalized_coverage=min(1., count/25./.25)))
    return out


def measured_corners(selected, theta=0.):
    # The four existing line choices keep their original edge labels and order.
    c, s = math.cos(theta), math.sin(theta)
    R = np.array([[c, -s], [s, c]])
    offsets = [float(np.mean(v['metric'] @ R, axis=0)[across])
               for v, across in zip(selected, [1, 0, 1, 0])]
    x0, y0, x1, y1 = offsets[3], offsets[0], offsets[1], offsets[2]
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]]) @ R.T


def actual_line_intersections(selected):
    lines = [np.cross(np.r_[v['pixels'][0], 1.], np.r_[v['pixels'][1], 1.])
             for v in selected]
    result = []
    for a, b in [(3, 0), (0, 1), (1, 2), (2, 3)]:
        p = np.cross(lines[a], lines[b])
        assert abs(p[2]) > 1e-9
        result.append(p[:2]/p[2])
    return np.array(result)


def canonical_angle(delta, index):
    a = math.atan2(delta[1], delta[0]) - (math.pi/2. if index % 2 else 0.)
    return (a+math.pi/2.) % math.pi - math.pi/2.


assert M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY
assert not O.exists()
assert sha(TRACE) == '7e7012a0303016b0ed5b9e49cf9587bb11bf5be57e677ac5e5301d6a464cb53c'
trace = json.loads(TRACE.read_text())
inputs = {str(TRACE): sha(TRACE), str(SOURCE): sha(SOURCE)}
assert inputs[str(SOURCE)] == trace['source_sha256'][str(SOURCE.relative_to(W))]
rows = []
for saved in trace['raw_frames_with_same_fused_geometry']:
    ns = saved['source_stamp_ns']
    t = saved['qualification']['return_local_diagnostics']['visual_rectangle_trace']
    raw = N / ('verification/gen29_attempt1_shadow_frames/source_%d/raw_arrays.npz' % ns)
    inputs[str(raw)] = sha(raw)
    assert inputs[str(raw)] == trace['input_sha256'][str(raw)]
    with np.load(raw, allow_pickle=False) as data:
        mask = data['object_mask'] > 0
    lines = t['kept_hough_lines']
    metric = np.concatenate([x['metric_points'] for x in lines])
    pixels = np.concatenate([np.asarray(x['pixels']).reshape(2, 2) for x in lines])
    H = homography(metric, pixels)
    herror = float(np.max(np.linalg.norm(project(H, metric)-pixels, axis=1)))
    assert herror < 1e-6
    choices = []
    for edge in t['choices_after_top3']:
        group = []
        for choice in edge:
            same = [x for x in lines if np.array_equal(np.asarray(x['pixels']).reshape(2, 2), choice['pixels'])]
            assert len(same) == 1
            group.append(dict(metric=np.asarray(same[0]['metric_points']),
                              pixels=np.asarray(choice['pixels']), original=choice))
        choices.append(group)
    combos = []
    for number, selected in enumerate(itertools.product(*choices), 1):
        fixed = project(H, measured_corners(selected))
        actual = actual_line_intersections(selected)
        deltas = [v['metric'][1]-v['metric'][0] for v in selected]
        angles = [canonical_angle(v, i) for i, v in enumerate(deltas)]
        weights = [float(np.linalg.norm(v)) for v in deltas]
        theta = .5*math.atan2(float(np.dot(weights, np.sin(2*np.array(angles)))),
                             float(np.dot(weights, np.cos(2*np.array(angles)))))
        fit = project(H, measured_corners(selected, theta))
        examples = [v for v in t['examples'] if v['combination_index'] == number]
        for old in examples:
            np.testing.assert_allclose(fixed, old['projected_corners_px'], rtol=0, atol=1e-6)
            np.testing.assert_allclose([x['original_quadrant_normalized_coverage'] for x in coverage(mask, fixed)],
                                       old['corner_coverage'], rtol=0, atol=1e-12)
        row = dict(combination_index=number,
                   selected_original_lines_pixels=[v['pixels'].tolist() for v in selected],
                   line_angles_relative_to_original_axis_deg=np.degrees(angles).tolist(),
                   length_weighted_common_orthogonal_rotation_deg=math.degrees(theta),
                   fixed_axis_corners_px=fixed.tolist(),
                   actual_line_intersections_px=actual.tolist(),
                   common_orthogonal_corners_px=fit.tolist(),
                   actual_intersection_shift_px=np.linalg.norm(actual-fixed, axis=1).tolist(),
                   common_orthogonal_shift_px=np.linalg.norm(fit-fixed, axis=1).tolist(),
                   fixed_axis_corner_coverage=coverage(mask, fixed),
                   actual_intersection_corner_coverage=coverage(mask, actual),
                   common_orthogonal_corner_coverage=coverage(mask, fit),
                   original_trace_reasons=sorted(set(x['reason'] for x in examples)))
        combos.append(row)
    assert len(combos) == t['combinations_examined']
    rows.append(dict(source_stamp_ns=ns, homography_endpoint_max_error_px=herror,
                     original_axis_source='measured_top_minAreaRect_long_edge_not_PCA',
                     choices_per_edge=[len(x) for x in choices], combinations=combos))
assert all(sha(p) == d for p, d in inputs.items())
report = dict(classification='finite_saved_Hough_geometry_diagnostic_not_box_qualification',
              recorded_ns=time.time_ns(), rows=rows, input_sha256=inputs,
              input_unchanged=True, script_sha256=sha(__file__), production_modified=False,
              no_ros_calls=True, no_hardware_access=True, threshold_changes=False,
              box_model_accepted=False, execution_authority=False,
              limitations=[
                  'Only the original 12/6/12 finite line combinations are read; no new edge search.',
                  'Exact projective mapping reconstructed from saved measured-plane endpoints and verified to <1e-6 pixel.',
                  'Original trace examples and their original-mask coverage are numerically reproduced.',
                  'Line intersections and common orthogonal fits are diagnostic alternatives, not proven physical corners.',
                  'No inward/outward depth, mask-boundary, full uncertainty, collision, IK or runtime identity qualification is rerun.',
                  'Hough segments contain printed inner lines and may be noisy; corner coverage alone cannot select a valid box.',
                  'Three historical frames retain the previously documented reconstruction limitations.'])
O.mkdir()
(O/'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
for r in rows:
    summary = []
    for c in r['combinations']:
        covs = [min(v['original_quadrant_normalized_coverage'] for v in c[k]) for k in
                ['fixed_axis_corner_coverage', 'actual_intersection_corner_coverage', 'common_orthogonal_corner_coverage']]
        if covs[0] < .5:
            summary.append(dict(combination=c['combination_index'], original_actual_common_min=covs,
                                theta_deg=c['length_weighted_common_orthogonal_rotation_deg'],
                                max_actual_shift_px=max(c['actual_intersection_shift_px']),
                                max_common_shift_px=max(c['common_orthogonal_shift_px'])))
    print(json.dumps(dict(source=r['source_stamp_ns'], corner_rejected=summary),ensure_ascii=False))
print(json.dumps(dict(result=str(O/'result.json'), sha256=sha(O/'result.json')),ensure_ascii=False))
