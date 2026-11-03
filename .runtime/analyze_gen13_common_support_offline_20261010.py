"""Sealed-record common-support-domain diagnosis; no ROS imports or runtime changes."""
import hashlib
import json
import os
from pathlib import Path
import numpy as np

M = Path('/media/zhuyupei/SanStand')
assert os.path.ismount(str(M)), 'EXPERIMENT_STORAGE_UNAVAILABLE'
A = M / '实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/gen13_handoff_analysis'
O = A / 'offline_support_domains'
O.mkdir(exist_ok=True)
rp = A / 'support_plane_replay/report.json'
r = json.loads(rp.read_text())
ref = np.asarray(r['reference']['plane_base'])
anchor = np.asarray(r['reference']['anchor_base'])
foot = np.asarray(r['reference']['footprint_base'])
eval_points = np.vstack([anchor, foot])
frames = r['frames'][:3]
clouds = []
source_rows = []
for f in frames:
    path = Path(f['source_file'])
    assert hashlib.sha256(path.read_bytes()).hexdigest() == f['source_file_sha256']
    with np.load(path, allow_pickle=False) as z:
        dep = z['depth_raw'].astype(np.float64) * float(z['depth_scale'])
        fx, fy, cx, cy = z['intrinsics']
        T = z['T_base_camera']
    v, u = np.indices(dep.shape)
    center = f['ring_center_px']
    rad = np.hypot(u-center[0], v-center[1])
    scale = min(dep.shape[1]/640., dep.shape[0]/480.)
    mask = ((rad >= 70*scale) & (rad <= f['ring_outer_radius_px']) &
            (dep >= r['config']['depth_min_m']) & (dep <= r['config']['depth_max_m']))
    d = dep[mask]
    pc = np.column_stack([d, -(u[mask]-cx)*d/fx, -(v[mask]-cy)*d/fy])
    pb = pc @ T[:3,:3].T + T[:3,3]
    clouds.append(pb)
    source_rows.append({'source_stamp_ns': f['source_stamp_ns'], 'path': str(path),
                        'sha256': f['source_file_sha256'], 'ring_point_count': len(pb)})

def cells(points, width, band):
    residual = points @ ref[:3] + ref[3]
    selected = points[np.abs(residual) <= band]
    ij = np.floor(selected[:,:2]/width).astype(np.int64)
    unique, inv = np.unique(ij, axis=0, return_inverse=True)
    buckets = {}
    for i, k in enumerate(unique):
        p = selected[inv == i]
        if len(p) >= 5:
            buckets[tuple(k)] = np.median(p, axis=0)
    return buckets

def plane(points):
    # Equal weight per physical XY cell prevents dense pixels in one view
    # from determining a different weighting of an overlapping support area.
    p = np.asarray(points)
    weights = np.ones(len(p))
    for _ in range(12):
        center = np.average(p, axis=0, weights=weights)
        x = p-center
        _, v = np.linalg.eigh((x * weights[:,None]).T @ x)
        n = v[:,0]
        if n @ ref[:3] < 0: n = -n
        b = -n @ center
        e = p @ n + b
        robust_scale = max(.00025, 1.4826*np.median(np.abs(e-np.median(e))))
        weights = np.maximum(0., 1-(e/(4.685*robust_scale))**2)**2
        if np.count_nonzero(weights) < 6:
            raise ValueError('too few robust cells')
    return np.r_[n,b]

def stats(values):
    return {str(q): float(np.percentile(values,q)) for q in [0,10,25,50,75,90,100]}

rows = []
for width in [.005, .010]:
    for band in [.010, .030]:
        maps = [cells(p,width,band) for p in clouds]
        for domain in [(0,1),(0,2),(0,1,2)]:
            common = sorted(set.intersection(*(set(maps[i]) for i in domain)))
            row = {'cell_size_mm': width*1000, 'diagnostic_reference_band_mm': band*1000,
                   'domain_frame_indices': list(domain), 'common_cell_count': len(common),
                   'available_cell_counts': [len(x) for x in maps], 'minimum_points_per_cell': 5}
            if len(common) < 20:
                row['result'] = 'insufficient_overlap'
                rows.append(row)
                continue
            pts = {i: np.array([maps[i][key] for key in common]) for i in domain}
            initial = pts[0] @ ref[:3]+ref[3]
            planes = {i: plane(p) for i,p in pts.items()}
            row['initial_cell_residual_mm_percentiles'] = stats(initial*1000)
            row['comparisons'] = []
            for i in domain[1:]:
                differences = (pts[i] @ ref[:3]+ref[3]-initial)*1000
                delta = planes[i]-planes[0]
                differences_fit = (eval_points @ delta[:3]+delta[3])*1000
                row['comparisons'].append({
                    'frame_index': i, 'source_stamp_ns': frames[i]['source_stamp_ns'],
                    'matched_cell_median_residual_difference_mm_percentiles': stats(differences),
                    'matched_cell_median_xy_distance_mm_percentiles': stats(np.linalg.norm(pts[i][:,:2]-pts[0][:,:2],axis=1)*1000),
                    'same_domain_plane_signed_differences_mm_range': [float(differences_fit.min()),float(differences_fit.max())],
                    'same_domain_plane_max_abs_delta_mm': float(np.max(np.abs(differences_fit))),
                    'same_domain_plane_normal_angle_deg': float(np.degrees(np.arccos(np.clip(planes[i][:3]@planes[0][:3],-1,1)))),
                    'initial_plane': planes[0].tolist(), 'current_plane': planes[i].tolist(),
                })
            rows.append(row)

report = {
    'source_report': str(rp), 'source_report_sha256': hashlib.sha256(rp.read_bytes()).hexdigest(),
    'source_frames': source_rows, 'method': 'Fixed base-XY grid overlap, >=5 points per cell, equal physical cell weighting; independent Tukey plane fit. Diagnostic bands 10/30 mm are not production threshold changes.',
    'rows': rows,
    'limitations': [
        'Physical XY correspondence depends on the recorded TF; this is not an independent camera-pose measurement.',
        'Within-cell sample positions differ. Two cell sizes and two broad bands expose sensitivity; no target identity or execution evidence is generated.',
        'No native prefilter depth was recorded, so this cannot separate sensor bias, local surface shape, extrinsic error, or mechanical compliance.',
    ],
    'restrictions': {'ros_or_hardware_access': False, 'production_code_changes': False, 'calibration_or_threshold_changes': False},
}
p = O / 'common_support_domains.json'
with p.open('x') as file:
    json.dump(report,file,ensure_ascii=False,indent=2)
    file.write('\n')
print(json.dumps({'path':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),
                  'results':[{'cell_mm':v['cell_size_mm'],'band_mm':v['diagnostic_reference_band_mm'],'domain':v['domain_frame_indices'],'n':v['common_cell_count'],
                              'comparisons':[{'frame':c['frame_index'],'median_mm':c['matched_cell_median_residual_difference_mm_percentiles']['50'],'plane_max_mm':c['same_domain_plane_max_abs_delta_mm']} for c in v.get('comparisons',[])]} for v in rows]},ensure_ascii=False))
