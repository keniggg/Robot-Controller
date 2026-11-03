"""Offline prism projection diagnostic; not a qualifier or hardware entrypoint."""
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import prototype_gen27_rgb_direction_box as prototype

external = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
assert external.parent.parent.is_mount()
root = external / 'ros_log/full_latest_gen22_restart_20261015T003700Z/verification'
result_path = root / 'gen27_rgb_direction_prototype_r1/initial_single_frame.json'
result = json.loads(result_path.read_text())
selected = next(row for row in result['candidates']
                if row['candidate_index'] == result['selected_candidate_index'])
source = Path(result['source_npz'])
with np.load(source, allow_pickle=False) as raw:
    data = {key: raw[key].copy() for key in raw.files}
geometry = prototype.load_geometry(Path(result['source_geometry']))
namespace, captured, qualifier = prototype.make_qualifier()
namespace['_prototype_axes'] = np.asarray(selected['axes_base'])
qualified = qualifier(data['depth_raw'], data['object_mask'], data['intrinsics'],
    float(data['depth_scale_m']), data['T_base_optical'], geometry,
    prototype.TargetTrackIdentity.from_stream(11, 12), int(data['source_stamp_ns']),
    color_bgr=data['color'])
assert qualified.ok
model = qualified.model.to_evidence_dict()
color, mask, depth, scale, T, K, origin, tn, axes, lower, upper, pixel_size = captured['args']
visual = captured['visual']
lo, hi = visual[1:3]
corners2 = np.array([[lo[0], lo[1]], [hi[0], lo[1]], [hi[0], hi[1]], [lo[0], hi[1]]])
n, d = geometry.support_normal_base, geometry.support_offset_m
bottom = corners2 @ axes[:, :2].T - d * n
top = bottom + np.outer(((origin - bottom) @ tn) / (tn @ n), n)
project = prototype.box._project
bottom_image, top_image = project(bottom, T, K), project(top, T, K)
hull = cv2.convexHull(np.vstack((bottom_image, top_image)).astype(np.float32))
prism_mask = np.zeros(mask.shape, np.uint8)
cv2.fillConvexPoly(prism_mask, np.rint(hull).astype(np.int32), 1)
boundary = (mask & ~cv2.erode(mask.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool))
projected_boundary = (prism_mask > 0) & ~(cv2.erode(prism_mask, np.ones((3, 3), np.uint8)) > 0)
to_prism = cv2.distanceTransform((~projected_boundary).astype(np.uint8), cv2.DIST_L2, 5)
to_actual = cv2.distanceTransform((~boundary).astype(np.uint8), cv2.DIST_L2, 5)
edges = cv2.Canny(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY), 50., 100.)
edge_support = cv2.dilate(edges, np.ones((5, 5), np.uint8)) > 0
edge_rows = []
center2 = (lo + hi) / 2.
camera2 = T[:3, 3] @ axes[:, :2]
for i, (across, sign) in enumerate(((1, -1), (0, 1), (1, 1), (0, -1))):
    if sign * (camera2[across] - center2[across]) <= (hi[across] - lo[across]) / 2.:
        continue
    a, b = bottom_image[i], bottom_image[(i + 1) % 4]
    samples = np.rint(a + np.outer(np.linspace(.1, .9, 31), b - a)).astype(int)
    outward = np.array([b[1] - a[1], a[0] - b[0]])
    outward /= np.linalg.norm(outward)
    if outward @ ((a + b) / 2. - np.mean(top_image, axis=0)) < 0:
        outward *= -1
    outer = np.rint(samples + 3. * outward).astype(int)
    z = depth[outer[:, 1], outer[:, 0]].astype(float) * scale
    valid = np.isfinite(z) & (z > 0)
    xyz = np.column_stack(((outer[:, 0] - K[2]) * z / K[0],
                           (outer[:, 1] - K[3]) * z / K[1], z)) @ T[:3, :3].T + T[:3, 3]
    support_residual = xyz @ n + d
    edge_rows.append(dict(edge_index=i, pixels=[a.tolist(), b.tolist()],
        actual_rgb_edge_support_fraction=float(np.mean(edge_support[samples[:, 1], samples[:, 0]])),
        actual_mask_boundary_distance_px=np.quantile(to_actual[samples[:, 1], samples[:, 0]], [0, .5, .9, 1]).tolist(),
        outside_valid_fraction=float(np.mean(valid)),
        outside_support_fraction=float(np.mean(valid & (np.abs(support_residual) <= .004))),
        outside_mask_fraction=float(np.mean(~mask[outer[:, 1], outer[:, 0]]))))
report = dict(classification='offline_projection_diagnostic_no_uncertainty_reduction_authorized',
    source_npz=str(source), source_sha256=prototype.digest(source),
    orientation_result_sha256=prototype.digest(result_path),
    selected_candidate=selected['candidate_index'], model_sha256=model['model_sha256'],
    top_image=top_image.tolist(), bottom_image=bottom_image.tolist(),
    prism_mask_iou=float(np.count_nonzero(mask & (prism_mask > 0)) / np.count_nonzero(mask | (prism_mask > 0))),
    actual_to_prism_boundary_distance_px=np.quantile(to_prism[boundary], [0, .5, .9, 1]).tolist(),
    prism_to_actual_boundary_distance_px=np.quantile(to_actual[projected_boundary], [0, .5, .9, 1]).tolist(),
    visible_bottom_edges=edge_rows, empirical_edge_uncertainty_m=model['empirical_edge_uncertainty_m'],
    caveats=['Projection uses independently fitted top plane and verified support; no old exterior uncertainty expands the nominal projection.',
             'These nominal residuals do not by themselves prove full contour visibility or justify shrinking uncertainty.'])
output = root / 'gen27_prism_projection_diagnostic_r1.json'
assert not output.exists()
output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
print(json.dumps(dict(path=str(output), sha256=prototype.digest(output), **report), ensure_ascii=False))
