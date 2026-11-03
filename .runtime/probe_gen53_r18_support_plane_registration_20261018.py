"""Offline original R18 plane-constrained registration diagnostic; no authority."""
from pathlib import Path
import dataclasses
import gzip
import hashlib
import importlib.abc
import json
import math
import os
import sys
import time

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
G = E/'grasp_records/20261018T143337.794533Z_bfea8721dba04526980035ef69ebe860'
V = E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
O = V/'gen53_r18_support_plane_registration_r1'
B = W/'.runtime/gen53_baseline_source'

class OfflineOnly(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'rospy', 'rosbag', 'rosgraph', 'moveit_commander', 'serial', 'alicia_d_sdk'}:
            raise RuntimeError('Offline diagnostic forbids '+fullname)

sys.meta_path.insert(0, OfflineOnly())
sys.path.insert(0, str(B/'src/alicia_flexible_grasp_supervisor/src'))
os.sched_setaffinity(0, sorted(os.sched_getaffinity(0))[:2])
import numpy as np
from alicia_flexible_grasp.vision.multiview_surface import (
    SurfaceView, RegistrationConfig, evaluate_registration_transform)
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from alicia_flexible_grasp.vision.regular_box_inference import _measured_family_excess

def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def shaped(a):
    a = np.ascontiguousarray(a, dtype='<f8')
    h = hashlib.sha256()
    h.update(str(a.shape).encode('ascii'))
    h.update(b'\0'+a.dtype.str.encode('ascii')+b'\0')
    h.update(a.tobytes())
    return h.hexdigest()

def clean(x):
    if isinstance(x, np.ndarray): return x.tolist()
    if isinstance(x, np.generic): return x.item()
    if dataclasses.is_dataclass(x): return clean(dataclasses.asdict(x))
    if isinstance(x, dict): return {k: clean(v) for k, v in x.items()}
    if isinstance(x, (tuple, list)): return [clean(v) for v in x]
    return x

def main():
    start = time.monotonic()
    assert Path('/media/zhuyupei/SanStand').is_mount() and not O.exists()
    ap = G/'keyframes_audits/404d30752dbb490c83ce54c9d307676d52bb75355b3683a09825d294e293b402.json'
    assert sha(ap) == ap.stem
    a = json.loads(ap.read_text())
    f = a['multiview_surface']['fused_scene_geometry']
    ba = a['remote_diagnostics']['tabletop_geometry']['regular_box_inference']
    cube = ba['cuboid_rgb_interval']
    refs, surfaces = {}, {}
    for k, r in f['surface_replay']['references'].items():
        p = Path(r['store_root'])/r['relative_path']
        assert sha(p) == r['gzip_sha256']
        raw_bytes = gzip.decompress(p.read_bytes())
        assert len(raw_bytes) == r['uncompressed_bytes']
        assert hashlib.sha256(raw_bytes).hexdigest() == r['object_sha256']
        surfaces[k] = json.loads(raw_bytes)
        refs[k] = dict(path=str(p), gzip_sha256=r['gzip_sha256'], object_sha256=r['object_sha256'])
    before = surfaces['before_registration_reference']
    raw = np.asarray(a['replay_geometry']['object_points_base_m'][:f['current_raw_point_count']])
    assert raw.shape == (806, 3) and shaped(raw) == f['current_raw_points_sha256']
    history = np.asarray(before['points_base'])
    assert history.shape == (1563, 3)
    assert shaped(history) == f['registration_choice']['source']['before_points_sha256']
    rd = dict(before['reference'])
    rd['identity'] = TargetTrackIdentity(**rd['identity'])
    reference = SurfaceView(**rd)
    moving = SurfaceView(reference.identity, a['snapshot_stamp_ns'], raw,
        np.asarray(f['support_normal_base']), f['support_offset_m'], ba['edge_clearance_px'])
    cfgp = G/'perception_execution_configuration.json'
    cfg = json.loads(cfgp.read_text())['parameters']['/grasp_6d/remote/multiview']
    config = RegistrationConfig(**cfg)

    # Unique shortest SO(3) rotation mapping the two measured support normals.
    # Translation is strictly parallel to the reference normal and exactly
    # reconciles the measured plane offsets. No box/path score enters this.
    nm, nr = moving.support_normal_base, reference.support_normal_base
    cross = np.cross(nm, nr)
    dot = float(nm @ nr)
    assert dot > 0.0
    K = np.array([[0., -cross[2], cross[1]], [cross[2], 0., -cross[0]], [-cross[1], cross[0], 0.]])
    rotation = np.eye(3)+K+(K@K)/(1.+dot)
    translation = nr*(moving.support_offset_m-reference.support_offset_m)
    plane = np.eye(4)
    plane[:3, :3], plane[:3, 3] = rotation, translation
    assert np.allclose(rotation@nm, nr, rtol=0, atol=1e-12)
    assert abs(moving.support_offset_m-(rotation@nm)@translation-reference.support_offset_m) < 1e-12
    assert np.linalg.norm(np.cross(translation, nr)) < 1e-12
    icp = np.asarray(a['multiview_surface']['latest_registration']['snapshot_to_reference_transform'])
    transforms = dict(original_icp=icp, identity=np.eye(4), support_plane_minimal=plane)

    # Exact existing qualifier family arithmetic. This audit deliberately does
    # not fabricate a qualified model from a failed final qualification.
    axes = np.asarray(cube['nominal_axes_base'])
    lo, hi = np.asarray(cube['nominal_lower']), np.asarray(cube['nominal_upper'])
    height = cube['nominal_height_m']
    center = axes[:, :2]@((lo+hi)/2)+axes[:, 2]*(height/2-cube['support_offset_m'])
    size = np.r_[hi-lo, height]
    edge = np.asarray(cube['empirical_axis_uncertainty_m'])
    hu, yaw = cube['empirical_height_uncertainty_m'], cube['empirical_yaw_uncertainty_rad']
    def family(points):
        return _measured_family_excess(points, center, axes, size, edge, hu, yaw, True)
    raw_excess, raw_conflict = family(raw)
    results = {}
    points_by_name = {}
    for name, transform in transforms.items():
        gate = evaluate_registration_transform(reference, moving, transform, config)
        inverse = np.linalg.inv(transform)
        in_snapshot = history@inverse[:3, :3].T+inverse[:3, 3]
        combined = np.concatenate((raw, in_snapshot), axis=0)
        _, first = np.unique(combined, axis=0, return_index=True)
        union = combined[np.sort(first)]
        excess, conflict = family(union)
        local = (union-center)@axes
        half = size/2.
        for axis in range(2):
            turn = min(yaw, math.atan2(size[1-axis], size[axis]))
            half[axis] = (size[axis]*math.cos(turn)+size[1-axis]*math.sin(turn))/2.
        half += np.r_[edge, hu]
        axis_excess = np.abs(local)-half
        per_view = []
        for vi, stamp in enumerate(before['view_stamps_ns']):
            indices = np.flatnonzero(np.asarray(before['view_indices']) == vi)
            ex, co = family(in_snapshot[indices])
            per_view.append(dict(view_index=vi, stamp_ns=stamp, representative_count=len(indices),
                maximum_excess_m=ex, conflict=co))
        row = dict(registration=clean(gate), transform=transform.tolist(),
            exact_union_count=len(union), exact_duplicate_count=len(combined)-len(union),
            exact_union_sha256=shaped(union), all_history_points_checked=True,
            family_arithmetic_consistent=conflict is None, maximum_family_excess_m=excess,
            conflict=conflict, maximum_axis_excess_m=np.maximum(0., axis_excess.max(axis=0)).tolist(),
            count_over_original_1mm_by_axis=(axis_excess > .001).sum(axis=0).tolist(),
            per_view=per_view)
        if conflict and conflict['point_index'] >= len(raw):
            # There are no duplicates in this original preappend union.
            assert len(union) == len(combined)
            j = conflict['point_index']-len(raw)
            row['conflict_provenance'] = dict(history_index=j,
                view_index=before['view_indices'][j], view_sources=before['view_sources'][j],
                phase_reference_point=history[j].tolist())
        results[name] = row
        points_by_name[name] = in_snapshot
        if name in ('original_icp', 'identity'):
            recorded = f['registration_choice']['hypotheses'][name]
            assert gate.ok == recorded['registration']['ok']
            assert gate.inlier_count == recorded['registration']['inlier_count']
            assert abs(gate.rmse_m-recorded['registration']['rmse_m']) < 1e-14
            assert abs(excess-recorded['consistency']['maximum_observed_model_excess_m']) < 1e-14
            assert shaped(union) == recorded['consistency']['exact_union_sha256']
            row['original_record_exactly_reproduced'] = True
    movement = {}
    for name in ('original_icp', 'support_plane_minimal'):
        delta = points_by_name[name]-points_by_name['identity']
        normal = delta@nm
        tangent = delta-normal[:, None]*nm
        local_delta = delta@axes
        movement[name] = dict(maximum_tangent_displacement_m=float(np.linalg.norm(tangent, axis=1).max()),
            maximum_normal_displacement_m=float(np.abs(normal).max()),
            median_local_delta_m=np.median(local_delta, axis=0).tolist(),
            maximum_absolute_local_delta_m=np.abs(local_delta).max(axis=0).tolist())
    source_hashes = {}
    for module in list(sys.modules.values()):
        p = getattr(module, '__file__', None)
        if p and str(B) in str(p): source_hashes[str(p)] = sha(p)
    result = dict(kind='G52_R18_measured_support_plane_registration_readonly_probe',
        passed=True, diagnostic_only=True, execution_authorized=False, real_grasp_success=False,
        created_ns=time.time_ns(), audit=dict(path=str(ap), sha256=sha(ap)), references=refs,
        configuration=dict(path=str(cfgp), sha256=sha(cfgp), multiview=cfg),
        source_hashes=source_hashes, helper_sha256=sha(__file__),
        request_id=a['request_id'], snapshot_stamp_ns=a['snapshot_stamp_ns'],
        raw_count=len(raw), before_count=len(history), raw_family_excess_m=raw_excess,
        raw_family_conflict=raw_conflict, reference_support_normal=nr.tolist(),
        moving_support_normal=nm.tolist(), reference_support_offset_m=reference.support_offset_m,
        moving_support_offset_m=moving.support_offset_m,
        plane_hypothesis_formula='R = I + skew(n_m x n_r) + skew(n_m x n_r)^2/(1+n_m dot n_r); t=(d_m-d_r)*n_r',
        plane_hypothesis_uses_grasp_score=False, support_rotation_angle_deg=float(np.degrees(np.arccos(dot))),
        hypotheses=results, displacement_decomposition_in_current_snapshot=movement,
        no_history_points_removed=True, gates_modified=False, ros_or_hardware_used=False,
        formal_raw_model_requalified=False, elapsed_sec=time.monotonic()-start,
        affinity=sorted(os.sched_getaffinity(0)),
        limitations=['Same original fixed-transform registration gates and exact complete preappend point union checked.',
            'Family arithmetic uses the retained final qualifier cuboid parameters, not fabricated RegularBoxInference authority.',
            'Minimum plane transform does not constrain unobservable in-plane translation or yaw, and does not establish physical truth.',
            'No fresh pixels, final fused qualification, candidate, path, controller or grasp success claim.'])
    O.mkdir()
    (O/'result.json').write_text(json.dumps(clean(result), ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    (O/'helper_source.py').write_bytes(Path(__file__).read_bytes())
    print(json.dumps(dict(result=str(O/'result.json'), sha256=sha(O/'result.json'),
        hypotheses={k:dict(registration_ok=v['registration']['ok'], code=v['registration']['code'],
            excess_m=v['maximum_family_excess_m'], axes=v['maximum_axis_excess_m']) for k,v in results.items()}), ensure_ascii=False))

if __name__ == '__main__': main()
