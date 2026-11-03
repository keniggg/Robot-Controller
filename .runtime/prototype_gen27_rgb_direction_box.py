"""Offline RGB-direction prototype; never imported by production or ROS.

The sole qualifier change is an in-memory axes override before the original
depth-coordinate calculation. Every original geometry/qualification check is
then rerun. RGB directions come from actual Hough segments, not an angle grid.
"""
import argparse
import hashlib
import inspect
import json
import math
from pathlib import Path
import sys
import time

import cv2
import numpy as np

WORKTREE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORKTREE / 'src/alicia_flexible_grasp_supervisor/src'))
from alicia_flexible_grasp.vision import regular_box_inference as box
from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity

EXTERNAL = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
INPUT = EXTERNAL / 'ros_log/full_latest_gen22_restart_20261015T003700Z/verification/gen27_attempt3_initial_inference_review'
DEFAULT_OUTPUT = INPUT.parent / 'gen27_rgb_direction_prototype_r1'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def wrap_rectangle_angle(angle):
    return (angle + math.pi / 4.) % (math.pi / 2.) - math.pi / 4.


def actual_hough_segments(context):
    """Exactly the production candidate-search domain and line-length cap."""
    color, mask, depth, scale, T, K, origin, tn, axes, lower, upper, pixel_size = context
    edges = cv2.Canny(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY), 50., 100.)
    edges[cv2.dilate(mask.astype(np.uint8), np.ones((5, 5), np.uint8)) == 0] = 0
    ys, xs = np.nonzero(mask)
    min_length = max(12, int(.35 * min(np.ptp(xs), np.ptp(ys))))
    found = cv2.HoughLinesP(edges, 1., np.pi / 180., max(12, min_length // 2),
                          minLineLength=min_length, maxLineGap=4)
    all_lines = [] if found is None else found[:, 0].tolist()
    lines = sorted(all_lines, key=lambda row: (-math.hypot(row[2] - row[0], row[3] - row[1]), tuple(row)))[:48]
    rows = []
    for index, line in enumerate(lines):
        pixels = np.asarray(line, float).reshape(2, 2)
        ray = np.column_stack(((pixels[:, 0] - K[2]) / K[0],
                               (pixels[:, 1] - K[3]) / K[1], np.ones(2))) @ T[:3, :3].T
        distance = float((origin - T[:3, 3]) @ tn) / (ray @ tn)
        if not np.all(np.isfinite(distance)) or np.any(distance <= 0.):
            continue
        points = T[:3, 3] + ray * distance[:, None]
        delta = points[1] - points[0]
        support_delta = delta - axes[:, 2] * float(delta @ axes[:, 2])
        length = float(np.linalg.norm(support_delta))
        if length <= 1e-12:
            continue
        angle = math.atan2(float(support_delta @ axes[:, 1]), float(support_delta @ axes[:, 0]))
        relative = wrap_rectangle_angle(angle)
        rows.append(dict(line_id=index, pixels=pixels.tolist(), metric_endpoints_base=points.tolist(),
                         pixel_length=math.hypot(line[2] - line[0], line[3] - line[1]),
                         support_projected_length_m=length,
                         actual_direction_in_depth_axes_deg=math.degrees(angle),
                         equivalent_rectangle_delta_deg=math.degrees(relative),
                         delta_rad=relative))
    return rows, dict(hough_total_count=len(all_lines), retained_hough_count=len(lines),
                      mask_dilation_size_px=5, minimum_line_length_px=min_length,
                      maximum_hough_segments=48)


def edge_admission_counts(context, lines, axes):
    """Read-only explanation of production's pre-combination edge filters."""
    _color, _mask, _depth, _scale, _T, _K, _origin, _tn, _seed_axes, _lo, _hi, pixel_size = context
    # Caller context contains the freshly recomputed depth extents for this axis.
    lower, upper = context[9], context[10]
    limits = ((1, lower[1]), (0, upper[0]), (1, upper[1]), (0, lower[0]))
    choices = [[] for _ in range(4)]
    rejected = [dict(length=0, angle=0, offset=0, overlap=0) for _ in range(4)]
    for row in lines:
        metric = np.asarray(row['metric_endpoints_base']) @ axes[:, :2]
        delta = metric[1] - metric[0]
        for index, (across, limit) in enumerate(limits):
            along = 1 - across
            if abs(delta[along]) < .35 * (upper[along] - lower[along]):
                rejected[index]['length'] += 1
                continue
            if abs(delta[across]) > math.tan(math.radians(8.)) * abs(delta[along]):
                rejected[index]['angle'] += 1
                continue
            offset = float(np.mean(metric[:, across]))
            if abs(offset - limit) > max(.012, 8. * pixel_size):
                rejected[index]['offset'] += 1
                continue
            overlap = min(float(np.max(metric[:, along])), upper[along]) - max(float(np.min(metric[:, along])), lower[along])
            if overlap < .35 * (upper[along] - lower[along]):
                rejected[index]['overlap'] += 1
                continue
            item = dict(line_id=row['line_id'], offset_m=offset, overlap_m=float(overlap),
                        depth_seed_distance_m=abs(offset - float(limit)))
            old = next((j for j, value in enumerate(choices[index])
                        if abs(value['offset_m'] - offset) < pixel_size), None)
            if old is None:
                choices[index].append(item)
            elif overlap > choices[index][old]['overlap_m']:
                choices[index][old] = item
    for choice in choices:
        choice.sort(key=lambda row: (row['depth_seed_distance_m'], -row['overlap_m']))
    return dict(depth_lower_m=lower.tolist(), depth_upper_m=upper.tolist(),
                edge_order=['lower_y', 'upper_x', 'upper_y', 'lower_x'],
                distinct_counts_before_top3=[len(choice) for choice in choices],
                choices=choices, filter_rejection_counts=rejected,
                empty_edge_indices=[i for i, choice in enumerate(choices) if not choice])


def make_qualifier():
    context_holder = {}
    namespace = dict(vars(box))
    namespace['_prototype_axes'] = None

    def observe_visual(*args):
        context_holder['args'] = args
        started = time.perf_counter()
        result = box._visual_rectangle(*args)
        context_holder['visual_seconds'] = time.perf_counter() - started
        context_holder['visual'] = result
        return result

    namespace['_visual_rectangle'] = observe_visual
    source = inspect.getsource(box.qualify_regular_box_top)
    original = '        coord = top @ axes[:, :2]\n'
    replacement = ('        if _prototype_axes is not None:\n'
                   '            axes = _prototype_axes.copy()\n' + original)
    assert source.count(original) == 1
    source = source.replace(original, replacement)
    exec(compile(source, '<offline_rgb_direction_qualifier>', 'exec'), namespace)
    return namespace, context_holder, namespace['qualify_regular_box_top']


def load_geometry(path):
    g = json.loads(path.read_text())
    assert g.get('available', True) is True
    points = np.asarray(g['object_points_base_m'], dtype=float)
    if 'object_points_sha256' in g:
        assert hashlib.sha256(np.ascontiguousarray(points).tobytes()).hexdigest() == g['object_points_sha256']
    return GeometryEstimate(True, '', '', np.array(g['obb_center_base_m']),
                            np.array(g['R_base_obb']), np.array(g['obb_size_xyz_m']),
                            np.array(g['support_normal_base']), g['support_offset_m'],
                            g['support_inlier_ratio'], points, g['source_mode'])


def qualify_source(npz_path, geometry_path, classification, limitations):
    started = time.perf_counter()
    with np.load(npz_path, allow_pickle=False) as raw:
        data = {key: raw[key].copy() for key in raw.files}
    geometry = load_geometry(geometry_path)
    # The exact recorded target continuity is g11-t12; diagnostic request
    # generation 13/17 is not the target track generation.
    identity = TargetTrackIdentity.from_stream(11, 12)
    arguments = (data['depth_raw'], data['object_mask'], data['intrinsics'],
                 float(data['depth_scale_m']), data['T_base_optical'], geometry,
                 identity, int(data['source_stamp_ns']))
    namespace, captured, qualifier = make_qualifier()

    def evaluate(candidate):
        namespace['_prototype_axes'] = None if candidate['kind'] == 'original_depth_axis' else np.asarray(candidate['axes_base'])
        captured.clear()
        t0 = time.perf_counter()
        result = qualifier(*arguments, color_bgr=data['color'])
        elapsed = time.perf_counter() - t0
        visual = captured.get('visual')
        row = dict(candidate, full_qualification_seconds=elapsed,
                   visual_rectangle_seconds=captured.get('visual_seconds'),
                   visual_rectangle_found=visual is not None,
                   ok=result.ok, code=result.code, audit=dict(result.audit),
                   eligible_rgb_rectangle=result.ok and visual is not None)
        if 'args' in captured:
            row['axes_base'] = captured['args'][8].tolist()
            row['edge_search'] = edge_admission_counts(captured['args'], lines, captured['args'][8]) if lines is not None else None
        if visual is not None:
            row['visual_rectangle'] = dict(area_m2=float(visual[0]), lower_m=visual[1].tolist(),
                upper_m=visual[2].tolist(), corner_coverage=visual[3], edge_coverage=visual[4],
                maximum_line_slope=float(visual[5]), unobserved_boundary_extension_m=float(visual[6]))
        if result.ok:
            row['model'] = result.model.to_evidence_dict()
        return row, captured.get('args')

    lines = None
    baseline, context = evaluate(dict(candidate_index=0, kind='original_depth_axis', relative_angle_deg=0.))
    assert context is not None, ('baseline never reached measured-top rectangle', baseline)
    generation_start = time.perf_counter()
    lines, hough = actual_hough_segments(context)
    seed_axes = context[8]
    baseline['edge_search'] = edge_admission_counts(context, lines, seed_axes)
    selected_angles = []
    candidates = []
    for line in lines:
        angle = line['delta_rad']
        # Zero is represented by the original depth candidate. Other bins are
        # represented by one actual segment, never an averaged/scan direction.
        if abs(angle) < math.radians(.5):
            line['orientation_disposition'] = 'represented_by_original_depth_axis'
            continue
        if any(abs(wrap_rectangle_angle(angle - old)) < math.radians(.5) for old in selected_angles):
            line['orientation_disposition'] = 'duplicate_actual_direction_within_half_degree'
            continue
        if len(candidates) >= 12:
            line['orientation_disposition'] = 'beyond_twelve_actual_directions'
            continue
        selected_angles.append(angle)
        ax0 = math.cos(angle) * seed_axes[:, 0] + math.sin(angle) * seed_axes[:, 1]
        axes = np.column_stack((ax0, np.cross(seed_axes[:, 2], ax0), seed_axes[:, 2]))
        assert np.allclose(axes.T @ axes, np.eye(3), atol=1e-7, rtol=0.)
        line['orientation_disposition'] = 'evaluated_actual_direction'
        candidates.append(dict(candidate_index=len(candidates) + 1, kind='actual_rgb_hough_direction',
                               hough_line_id=line['line_id'], relative_angle_deg=math.degrees(angle),
                               axes_base=axes.tolist()))
    generation_seconds = time.perf_counter() - generation_start
    results = [baseline]
    for candidate in candidates:
        row, _context = evaluate(candidate)
        results.append(row)
    accepted = [row for row in results if row['eligible_rgb_rectangle']]
    winner = max(accepted, key=lambda row: row['visual_rectangle']['area_m2']) if accepted else None
    return dict(classification=classification, source_stamp_ns=int(data['source_stamp_ns']),
                source_npz=str(npz_path), source_geometry=str(geometry_path), limitations=limitations,
                target_identity=dict(epoch=12, track_id='g11-t12'),
                hough=hough, actual_hough_lines=lines, candidate_generation_seconds=generation_seconds,
                candidate_count=len(results), qualified_rgb_candidate_count=len(accepted),
                candidates=results, selected_candidate_index=None if winner is None else winner['candidate_index'],
                selected_model=None if winner is None else winner['model'],
                any_complete_rgb_qualification=winner is not None,
                original_depth_fallback_preserved=winner is None and baseline['ok'],
                total_seconds=time.perf_counter() - started)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    assert Path('/media/zhuyupei/SanStand').is_mount(), 'SanStand mount required; no fallback'
    output = args.output.resolve()
    output.relative_to(EXTERNAL.resolve())
    assert not output.exists(), 'immutable output already exists'
    manifest = json.loads((INPUT / 'source_manifest.json').read_text())
    source_rows = {row['source_stamp_ns']: row for row in manifest['rows']}
    reconstruction = json.loads((INPUT / 'initial_three_frame_replay.json').read_text())
    specs = [
        ('initial_single_frame', INPUT / 'source_1792050907639824151.npz', INPUT / 'geometry_1792050907639824151.json',
         'recorded_raw_frame_with_exact_source_geometry_audit', ['Not the original multi-frame initial planning snapshot.']),
        ('initial_three_frame_reconstruction', INPUT / 'initial_three_frame_reconstruction.npz', INPUT / 'initial_three_frame_geometry.json',
         reconstruction['classification'], reconstruction['limitations']),
        ('nearfield_single_frame', INPUT / 'source_1792050956912783622.npz', INPUT / 'geometry_1792050956912783622.json',
         'recorded_raw_frame_with_exact_source_geometry_audit', ['Raw source, not a reconstruction of all nearfield fusion inputs.']),
    ]
    dependencies = {str(Path(box.__file__).resolve()), str(Path(inspect.getfile(GeometryEstimate)).resolve()),
                    str(Path(inspect.getfile(TargetTrackIdentity)).resolve()), str(Path(__file__).resolve()),
                    str(INPUT / 'source_manifest.json'), str(INPUT / 'initial_three_frame_replay.json')}
    for _name, npz, geometry, _classification, _limitations in specs:
        dependencies.update((str(npz), str(geometry)))
    for stamp in (1792050907639824151, 1792050956912783622):
        row = source_rows[stamp]
        assert digest(row['npz']) == row['sha256']
        dependencies.add(row['audit'])
    before = {path: digest(path) for path in sorted(dependencies)}
    output.mkdir(parents=True)
    summary = []
    for name, npz, geometry, classification, limitations in specs:
        report = qualify_source(npz, geometry, classification, limitations)
        report['no_ros_calls'] = True
        report['no_hardware_access'] = True
        report['production_thresholds_and_uncertainty_unchanged'] = True
        target = output / (name + '.json')
        target.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
        selected = next((r for r in report['candidates'] if r['candidate_index'] == report['selected_candidate_index']), None)
        row = dict(name=name, path=str(target), sha256=digest(target),
                   candidate_count=report['candidate_count'],
                   qualified_rgb_candidate_count=report['qualified_rgb_candidate_count'],
                   baseline_code=report['candidates'][0]['code'],
                   selected_candidate_index=report['selected_candidate_index'],
                   selected_relative_angle_deg=None if selected is None else selected['relative_angle_deg'],
                   selected_dimensions_m=None if selected is None else selected['model']['size_xyz_m'],
                   total_seconds=report['total_seconds'])
        summary.append(row)
        print(json.dumps(row, ensure_ascii=False), flush=True)
    after = {path: digest(path) for path in sorted(dependencies)}
    receipt = dict(classification='offline_prototype_not_deployed_not_motion_authorization',
                   source_and_input_sha256=before, source_and_input_unchanged=before == after,
                   experiments=summary,
                   prototype_change='Only actual RGB candidate axes override in-memory before original coord; original complete qualification rerun per candidate.',
                   selection='Largest area among RGB rectangles passing every original qualification check; original depth fallback retained separately.',
                   limits=dict(maximum_actual_rgb_directions=12, additional_original_depth_direction=1,
                               minimum_direction_separation_deg=.5, arbitrary_angle_scan=False,
                               mask_search_domain_unchanged=True, production_files_edited=False),
                   no_ros_calls=True, no_hardware_access=True)
    assert receipt['source_and_input_unchanged']
    target = output / 'receipt.json'
    target.write_text(json.dumps(receipt, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    print(json.dumps(dict(receipt=str(target), sha256=digest(target)), ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
