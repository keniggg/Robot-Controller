"""Replay conditional joint-pivot diagnostics against independent saved evidence.

No ROS connection, bag playback, camera access, robot command, or deployed model
change. Saved RGB-D observations and already selected independent RGB matches
are the only inputs. Per-pose virtual angles remain diagnostic hypotheses.
"""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs


HERE = Path(__file__).resolve().parent
CANONICAL = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/'
                 'full_ros_latest_start_20261002/gen4_root_cause')
INPUTS = {}


def fingerprint(path):
    path = Path(path).resolve()
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    old = INPUTS.get(str(path))
    if old is not None and old != digest:
        raise RuntimeError('input changed during analysis: ' + str(path))
    INPUTS[str(path)] = digest
    return path


def read_json(path):
    return json.loads(fingerprint(path).read_text())


def read_npz(path):
    with np.load(str(fingerprint(path)), allow_pickle=False) as z:
        return {key: z[key].copy() for key in z.files}


def load_core():
    path = fingerprint(HERE / 'review_error_pivot.py')
    spec = importlib.util.spec_from_file_location('pivot_core', str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def frame_paths(session):
    path = fingerprint(session / 'events.jsonl')
    events = [json.loads(line) for line in path.read_text().splitlines()]
    frames = {}
    for event in events:
        if event.get('event') == 'stationary_snapshot':
            frame = Path(event['file']).resolve()
            if frame.parent != session.resolve():
                raise ValueError('saved frame escapes session directory')
            frames[event['source_stamp_ns']] = frame
    return frames


def correspondence_path(directory, source, destination, model):
    return directory / ('%s_%s_%s_spatialcv_correspondences.npz'
                        % (source, destination, model))


def spatial_folds(uv, width=70, shift=0):
    cells = ((np.asarray(uv) + shift) // width).astype(int)
    unique, counts = np.unique(cells, axis=0, return_counts=True)
    assignments, loads = {}, [0, 0, 0]
    for index in sorted(range(len(unique)),
                        key=lambda i: (-int(counts[i]), tuple(unique[i]))):
        fold = int(np.argmin(loads))
        assignments[tuple(unique[index])] = fold
        loads[fold] += int(counts[index])
    return np.array([assignments[tuple(cell)] for cell in cells])


def prediction(core, F0, F, candidate, angle_deg):
    E = core.axis_transform(candidate['axis_base'],
                            candidate['pivot_base_m'], math.radians(angle_deg))
    return np.linalg.inv(E @ F) @ F0


def check_correspondences(data):
    P, Q, uv = data['P_source'], data['Q_destination'], data['uv0']
    if (P.shape != Q.shape or P.ndim != 2 or P.shape[1] != 3
            or uv.shape != (len(P), 2) or len(P) < 15
            or not np.isfinite(np.r_[P.ravel(), Q.ravel(), uv.ravel()]).all()):
        raise ValueError('invalid saved correspondence arrays')
    return P, Q, uv


def rmse_mm(predicted, measured):
    return float(np.sqrt(np.mean(np.sum((predicted - measured) ** 2,
                                       axis=1))) * 1000)


def alternative_partitions(core, F0, F, candidates, P, Q, uv):
    results = []
    for width, shift in ((50, 0), (70, 17), (70, 35), (90, 0), (90, 45)):
        folds = spatial_folds(uv, width, shift)
        models = {}
        for joint in (2, 3, 5):
            candidate = candidates[joint]
            errors, angles = [], []
            for fold in range(3):
                train = np.flatnonzero(folds != fold)
                test = np.flatnonzero(folds == fold)
                if len(train) < 10 or len(test) < 5:
                    raise ValueError('insufficient alternative spatial fold')
                delta = core.fit_angle(F0, F, candidate['axis_base'],
                                       candidate['pivot_base_m'], P, Q, train)
                T = prediction(core, F0, F, candidate, math.degrees(delta))
                errors.extend(np.linalg.norm(core.mappts(T, P[test]) - Q[test],
                                             axis=1))
                angles.append(math.degrees(delta))
            models[str(joint)] = {
                'pooled_heldout': core.stats(errors),
                'fold_virtual_angles_deg': angles,
            }
        results.append({
            'tile_width_px': width, 'tile_origin_shift_px': shift,
            'models': models,
            'J3_minus_J2_rmse_mm': models['3']['pooled_heldout']['rmse_mm']
                                  - models['2']['pooled_heldout']['rmse_mm'],
            'J5_minus_J2_rmse_mm': models['5']['pooled_heldout']['rmse_mm']
                                  - models['2']['pooled_heldout']['rmse_mm'],
        })
    return results


def optimizer_grid_check(core, F0, F, candidates, P, Q, uv):
    """Independent Rodrigues-free 601-angle scan of each saved fit loss."""
    folds = core.spatial_folds(uv)
    grid = np.radians(np.linspace(-5, 5, 601))
    cos, sin = np.cos(grid)[:, None, None], np.sin(grid)[:, None, None]
    inverse_F = np.linalg.inv(F)
    checks = []
    for candidate in candidates.values():
        axis = np.asarray(candidate['axis_base'], float)
        axis /= np.linalg.norm(axis)
        pivot = np.asarray(candidate['pivot_base_m'], float)
        for fold in candidate['folds']:
            train = folds != fold['fold']
            W = core.mappts(F0, P[train]) - pivot
            points_base = (W * cos - np.cross(axis, W)[None] * sin
                           + axis[None, None] * (W @ axis)[None, :, None]
                           * (1 - cos) + pivot)
            predicted = (points_base @ inverse_F[:3, :3].T
                         + inverse_F[:3, 3])
            error = predicted - Q[train]
            losses = np.mean(np.sqrt(np.sum(error ** 2, axis=2) + .002 ** 2),
                             axis=1)
            exact = (core.mappts(prediction(core, F0, F, candidate,
                                           fold['virtual_angle_deg']), P[train])
                     - Q[train])
            exact_loss = float(np.mean(np.sqrt(np.sum(exact ** 2, axis=1)
                                               + .002 ** 2)))
            excess = exact_loss - float(losses.min())
            if excess > 1e-9:
                raise RuntimeError('saved optimizer lost to exhaustive grid')
            checks.append({
                'joint': candidate['joint'], 'fold': fold['fold'],
                'saved_virtual_angle_deg': fold['virtual_angle_deg'],
                'best_grid_angle_deg': float(np.degrees(grid[losses.argmin()])),
                'saved_loss_excess_over_grid_min_m': excess,
            })
    return checks


def conditional_tile_bootstrap(core, F0, F, candidates, P, Q, uv,
                               repetitions, rng):
    if not repetitions:
        return {'run': False, 'reason': 'Optional repeated bootstrap omitted; '
                'primary independent predictions and alternative folds run.'}
    folds = core.spatial_folds(uv)
    cells = (uv // 70).astype(int)
    unique, ids = np.unique(cells, axis=0, return_inverse=True)
    losses = {}
    for joint in (2, 3, 5):
        candidate = candidates[joint]
        errors = np.zeros(len(P))
        for fold in candidate['folds']:
            test = folds == fold['fold']
            T = prediction(core, F0, F, candidate, fold['virtual_angle_deg'])
            errors[test] = np.linalg.norm(core.mappts(T, P[test]) - Q[test],
                                         axis=1)
        if abs(core.stats(errors)['rmse_mm']
               - candidate['pooled_heldout']['rmse_mm']) > 1e-8:
            raise RuntimeError('replayed heldout errors differ from source')
        losses[joint] = np.bincount(ids, weights=errors ** 2)
    draw = rng.randint(len(unique), size=(repetitions, len(unique)))
    counts = np.bincount(ids)[draw].sum(axis=1)
    base = np.sqrt(losses[2][draw].sum(axis=1) / counts) * 1000
    result = {'run': True, 'repetitions': repetitions,
              'source_tile_count': len(unique), 'angles_frozen': True}
    for joint in (3, 5):
        delta = np.sqrt(losses[joint][draw].sum(axis=1) / counts) * 1000 - base
        result['J%d_minus_J2_conditional_percentiles_mm' % joint] = (
            np.quantile(delta, [.025, .5, .975]).tolist())
    return result


def prepare_session(root, saved):
    session = Path(saved['session']).resolve()
    if session.parent != root:
        raise ValueError('pivot review points outside selected evidence root')
    report_path = Path(saved['source_report']).resolve()
    report = read_json(report_path)
    if INPUTS[str(report_path)] != saved['source_report_sha256']:
        raise RuntimeError('source support report changed after pivot review')
    model = fingerprint(session / 'robot_description.urdf')
    if INPUTS[str(model)] != saved['source_model_urdf_sha256']:
        raise RuntimeError('source URDF digest mismatch')
    frames = frame_paths(session)
    baseline = report['baseline']['publication_header_stamp_ns']
    F0 = read_npz(frames[baseline])['T_base_optical']
    bindings = {(r['label'], r['projection_model']): r
                for r in report['results']}
    rows = []
    for row in saved['results']:
        if (row['model'] == 'sdk_inverse_brown'
                and np.linalg.norm(row['visual_rotation_error_base_deg']) >= 1):
            binding = bindings[(row['label'], row['model'])]
            ns = binding['destination_ns']
            path = correspondence_path(report_path.parent, baseline, ns,
                                       row['model'])
            data = read_npz(path)
            if INPUTS[str(path.resolve())] != row['correspondences_sha256']:
                raise RuntimeError('correspondence digest mismatch')
            P, Q, uv = check_correspondences(data)
            F = read_npz(frames[ns])['T_base_optical']
            rows.append((row, F, P, Q, uv))
    return session, report, frames, F0, rows


def review_partitions(core, root, saved, repetitions, rng):
    session, report, frames, F0, rows = prepare_session(root, saved)
    results = []
    for row, F, P, Q, uv in rows:
        candidates = {c['joint']: c for c in row['candidates']}
        results.append({
            'label': row['label'], 'point_count': len(P),
            'alternative_partitions': alternative_partitions(
                core, F0, F, candidates, P, Q, uv),
            'optimizer_grid_checks': optimizer_grid_check(
                core, F0, F, candidates, P, Q, uv),
            'conditional_tile_bootstrap': conditional_tile_bootstrap(
                core, F0, F, candidates, P, Q, uv, repetitions, rng),
        })
    minimum_gaps = {}
    for joint in (3, 5):
        key = 'J%d_minus_J2_rmse_mm' % joint
        gaps = [p[key] for r in results for p in r['alternative_partitions']]
        minimum_gaps[key] = {'minimum': min(gaps),
                             'J2_lower_error_count': sum(g > 0 for g in gaps),
                             'comparisons': len(gaps)}
    return {'session': str(session), 'large_error_pose_count': len(rows),
            'minimum_alternative_partition_gaps': minimum_gaps,
            'results': results}


def frozen_hold(core, root, saved):
    session, report, frames, F0, _ = prepare_session(root, saved)
    hold = root / 'static_hold_20261003T092104Z'
    frames.update(frame_paths(hold))
    hold_report_path = session / 'support_plane_static_hold/fixed_local_support_report.json'
    hold_report = read_json(hold_report_path)
    baseline = report['baseline']['publication_header_stamp_ns']
    if hold_report['baseline']['publication_header_stamp_ns'] != baseline:
        raise ValueError('passive hold does not share the recorded source baseline')
    frozen = next(r for r in saved['results']
                  if r['label'] == 'return_02_j2'
                  and r['model'] == 'sdk_inverse_brown')
    candidates = {c['joint']: c for c in frozen['candidates']}
    source_binding = next(r for r in report['results']
                          if r['label'] == frozen['label']
                          and r['projection_model'] == frozen['model'])
    source_q = np.array(source_binding['frame_binding']
                        ['bracketing_actual_q_rad'][0][:6])
    results = []
    for row in hold_report['results']:
        if row['projection_model'] != 'sdk_inverse_brown':
            continue
        ns = row['destination_ns']
        data = read_npz(correspondence_path(hold_report_path.parent,
                                           baseline, ns, row['projection_model']))
        P, Q, _ = check_correspondences(data)
        F = read_npz(frames[ns])['T_base_optical']
        models = {}
        for joint in (2, 3, 5):
            candidate = candidates[joint]
            angle = candidate['mean_virtual_angle_deg']
            predicted = core.mappts(prediction(core, F0, F, candidate, angle), P)
            models[str(joint)] = {
                'frozen_virtual_angle_deg': angle,
                'rmse_mm': rmse_mm(predicted, Q),
                'axis_and_pivot_frozen_from_previous_pose': True,
            }
        q = np.array(row['frame_binding']['bracketing_actual_q_rad'][0][:6])
        results.append({
            'label': row['label'], 'source_ns': baseline, 'destination_ns': ns,
            'point_count': len(P),
            'nominal_actual_difference_from_angle_fit_pose_deg':
                np.degrees(q - source_q).tolist(),
            'uncorrected_tf_rmse_mm': rmse_mm(
                core.mappts(np.linalg.inv(F) @ F0, P), Q),
            'models': models,
        })
    if len(results) != 3:
        raise RuntimeError('all three passive hold frames required')
    return {
        'angle_source_session': str(session), 'angle_source_label': frozen['label'],
        'hold_session': str(hold), 'refitted_on_hold': False,
        'scope': 'Temporal replication near the previous nominal pose; not a '
                 'cross-pose predictive calibration. Original axes/pivots and '
                 'mean virtual angles stay frozen. J2/J3/J5 each differ by one '
                 'encoder quantum in the saved hold feedback.',
        'results': results,
    }


def sample_depth(frame, uv):
    """Replay original 5x5 inverse-depth patch estimator without new trimming."""
    depth = frame['depth_raw'] * float(frame['depth_scale'])
    values, quality = [], []
    for x, y in uv:
        ix, iy = int(round(x)), int(round(y))
        x0, y0 = max(ix - 2, 0), max(iy - 2, 0)
        patch = depth[y0:iy + 3, x0:ix + 3]
        py, px = np.indices(patch.shape)
        good = (patch > .15) & (patch < .8)
        if good.sum() < 18:
            values.append(np.nan); quality.append(np.inf); continue
        design = np.column_stack((px[good] + x0 - x,
                                  py[good] + y0 - y, np.ones(good.sum())))
        z = patch[good]
        coeff = np.linalg.lstsq(design, 1. / z, rcond=None)[0]
        error = 1. / (design @ coeff) - z
        if np.percentile(abs(error), 90) > .002 or coeff[2] <= 0:
            values.append(np.nan); quality.append(np.inf); continue
        values.append(1. / coeff[2])
        quality.append(np.sqrt(np.mean(error ** 2)))
    return np.array(values), np.array(quality)


def pixel_rays(intrinsics, uv):
    return np.array([rs.rs2_deproject_pixel_to_point(
        intrinsics, list(map(float, p)), 1.) for p in uv])


def frozen_independent_regions(core, root, saved):
    session, report, frames, _, _ = prepare_session(root, saved)
    unique_path = session / 'visual_geometry/unique_and_off_mat_validation.json'
    unique = read_json(unique_path)
    fingerprint(unique_path.parent / 'validate_unique_high_anchors.py')
    ns0, ns = unique['source'], unique['destination']
    f, g = read_npz(frames[ns0]), read_npz(frames[ns])
    binding = next(r for r in report['results']
                   if r['destination_ns'] == ns
                   and r['projection_model'] == 'sdk_inverse_brown')
    if report['baseline']['publication_header_stamp_ns'] != ns0:
        raise ValueError('independent regions do not share the pivot baseline')
    frozen = next(r for r in saved['results']
                  if r['label'] == binding['label']
                  and r['model'] == 'sdk_inverse_brown')
    candidates = {c['joint']: c for c in frozen['candidates']}
    ii = read_json(session / 'fresh_plan/camera_parameters.json')
    ii = ii['runtime_profile']['color_intrinsics']
    intr = rs.intrinsics()
    intr.width, intr.height = ii['width'], ii['height']
    intr.fx, intr.fy, intr.ppx, intr.ppy = ii['fx'], ii['fy'], ii['cx'], ii['cy']
    intr.coeffs = ii['coeffs']; intr.model = rs.distortion.inverse_brown_conrady
    for frame in (f, g):
        K = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])
        if not np.array_equal(K, frame['K']):
            raise ValueError('independent RGB-D frame/intrinsics binding mismatch')
    F0, F = f['T_base_optical'], g['T_base_optical']
    results = []
    for region in unique['results']:
        if not region['count']:
            results.append({'region': region['region'], 'point_count': 0,
                            'rgb_matches_before_saved_depth_selection':
                                region['rgb_matches']})
            continue
        uv0 = np.array(region['source_uv']); uv1 = np.array(region['destination_uv'])
        z0, quality0 = sample_depth(f, uv0); z1, quality1 = sample_depth(g, uv1)
        if (len(uv0) != region['count'] or len(uv1) != region['count']
                or not np.isfinite(np.r_[z0, z1]).all()
                or np.any(quality0 >= .0015) or np.any(quality1 >= .0015)):
            raise RuntimeError('saved independent points no longer replay patch checks')
        rays0, rays1 = pixel_rays(intr, uv0), pixel_rays(intr, uv1)
        P, Q = rays0 * z0[:, None], rays1 * z1[:, None]
        models = {}
        for joint in (2, 3, 5):
            candidate = candidates[joint]
            angle = candidate['mean_virtual_angle_deg']
            T = prediction(core, F0, F, candidate, angle)
            predicted = core.mappts(T, P)
            delta_px = ((predicted[:, :2] / predicted[:, 2, None]
                         - rays1[:, :2] / rays1[:, 2, None])
                        * [g['K'][0, 0], g['K'][1, 1]])
            errors = np.linalg.norm(predicted - Q, axis=1)
            models[str(joint)] = {
                'frozen_virtual_angle_deg': angle,
                'residual_3d': core.stats(errors),
                'max_3d_mm': float(errors.max() * 1000),
                'per_point_residual_3d_mm': (errors * 1000).tolist(),
                'normalized_projection_rmse_px': float(np.sqrt(
                    np.mean(np.sum(delta_px ** 2, axis=1)))),
            }
        results.append({
            'region': region['region'], 'point_count': len(P),
            'rgb_matches_before_saved_depth_selection': region['rgb_matches'],
            'uncorrected_tf_rmse_mm': rmse_mm(
                core.mappts(np.linalg.inv(F) @ F0, P), Q),
            'points_removed_after_saved_selection': 0,
            'models': models,
        })
    return {
        'session': str(session), 'source_ns': ns0, 'destination_ns': ns,
        'angle_source_label': frozen['label'], 'refitted_on_independent_regions': False,
        'saved_region_selection': unique['scope'],
        'scope': 'RGB matches from previously selected semantic source ROIs, '
                 'not used in the primary table fit; >5px from primary features. '
                 'No new homography/model-inlier trimming. Existing matches and '
                 'the red-logo large outlier remain present. Laptop/off-mat '
                 'regions share the same seven points and are not two independent '
                 'replications. Physical stationarity is assumed from the record.',
        'results': results,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=CANONICAL)
    parser.add_argument('--bootstrap-repetitions', type=int, default=0)
    args = parser.parse_args()
    root = args.root.resolve()
    if (root != CANONICAL.resolve()
            or not Path('/media/zhuyupei/SanStand').is_mount()):
        raise RuntimeError('mounted canonical SanStand evidence root required')
    if args.bootstrap_repetitions < 0 or args.bootstrap_repetitions > 10000:
        raise ValueError('bootstrap repetitions must be in [0,10000]')
    cv2.setNumThreads(1)
    core = load_core()
    pivot = read_json(root / 'joint_pivot_review/result.json')
    if len(pivot['sessions']) != 2:
        raise ValueError('the two specified pivot-review sessions are required')
    names = [Path(s['session']).name for s in pivot['sessions']]
    if names != ['high_posture_20261003T081809Z', 'high_posture_20261003T091023Z']:
        raise ValueError('unexpected pivot-review session order')
    fingerprint(Path(__file__))
    rng = np.random.RandomState(3521)
    report = {
        'method': 'Replay saved source-bound observations. Virtual joint angles '
                  'are per-pose hypotheses; this script produces analysis '
                  'artifacts only and never deploys a correction.',
        'resources': {'opencv_threads': cv2.getNumThreads(),
                      'numpy_version': np.__version__, 'opencv_version': cv2.__version__,
                      'realsense_binding_path': rs.__file__,
                      'bootstrap_repetitions': args.bootstrap_repetitions,
                      'bootstrap_seed': 3521, 'no_ros_connection': True,
                      'no_bag_read_or_device_access': True},
        'limitations': [
            'Homography uses all RGB matches before spatial folds. Heldout '
            'errors are conditional on previously selected correspondences; '
            'angle fitting itself never uses the test residuals.',
            'Alternative folds share data. Poses within each session share '
            'the source baseline. Counts do not represent independent experiments.',
            'Optional tile-bootstrap intervals freeze fitted models and exclude '
            'systematic depth/geometry/extrinsic/exposure errors and model selection.',
            'ROS image headers are host publication times, not independently '
            'calibrated exposure times. Axis construction uses the previous '
            'feedback bracket; the saved camera TF is evaluated at the header.',
            'Frozen-angle static hold validates temporal replication near one '
            'pose, not a predictive cross-pose calibration or unique hardware cause.',
            'Off-mat evidence contains only seven physical points; off-mat and '
            'laptop results overlap. Static off-mat objects are assumed, not proven.',
            'All previously selected points remain, including the red-logo '
            'roughly60mm outlier; no residual-based deletion is permitted.',
            'Parallel axes, robot/base compliance, transmission errors, flexible '
            'camera extrinsics and combinations remain possible. Ranking '
            'single-axis hypotheses cannot identify one failed component.',
        ],
        'sessions': [review_partitions(core, root, saved,
                                      args.bootstrap_repetitions, rng)
                     for saved in pivot['sessions']],
        'frozen_static_hold': frozen_hold(core, root, pivot['sessions'][1]),
        'frozen_independent_regions': frozen_independent_regions(
            core, root, pivot['sessions'][0]),
    }
    # Verify all consumed inputs stayed fixed before publishing the analysis.
    for path in list(INPUTS):
        fingerprint(path)
    report['input_sha256_by_path'] = dict(sorted(INPUTS.items()))
    serialized = json.dumps(report, indent=2, allow_nan=False) + '\n'
    output = root / 'joint_pivot_review/independent_validation'
    output.mkdir(parents=True, exist_ok=True)
    (output / 'result.json').write_text(serialized)
    (output / 'analysis_source.py').write_bytes(Path(__file__).read_bytes())
    (output / 'sha256.json').write_text(json.dumps({
        p.name: hashlib.sha256(p.read_bytes()).hexdigest()
        for p in (output / 'result.json', output / 'analysis_source.py')
    }, indent=2) + '\n')
    print(json.dumps({
        'output': str(output / 'result.json'),
        'input_files_hashed': len(INPUTS),
        'large_error_pose_count': sum(s['large_error_pose_count']
                                     for s in report['sessions']),
        'alternative_partition_comparisons': sum(
            s['minimum_alternative_partition_gaps']['J3_minus_J2_rmse_mm']
            ['comparisons'] for s in report['sessions']),
        'optimizer_checks': sum(len(r['optimizer_grid_checks'])
                                for s in report['sessions'] for r in s['results']),
        'hold_frames': len(report['frozen_static_hold']['results']),
        'independent_region_counts': {
            r['region']: r['point_count']
            for r in report['frozen_independent_regions']['results']},
    }, indent=2), flush=True)


if __name__ == '__main__':
    main()
