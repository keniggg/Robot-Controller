"""Offline nearfield corner localization radius and explicit error budget.

No production edits, ROS or hardware calls. Reuses actual RGB Hough directions,
the separately reviewed 7/9 evidence on each corner branch, and every remaining
qualification check. The edge budget pays radius * metric_pixel_size instead
of the original half-pixel localization term.
"""
import argparse
import ast
from collections import Counter
import inspect
import json
from pathlib import Path
import sys
import time

import numpy as np
import prototype_gen27_rgb_search_domain as search

base, box = search.base, search.box
CORNER_SCRIPT = Path(__file__).with_name('prototype_gen27_rgb_corner_evidence.py')
TRACES = []


def corner_blocks():
    constants = [node.value for node in ast.walk(ast.parse(CORNER_SCRIPT.read_text()))
                 if isinstance(node, ast.Constant) and isinstance(node.value, str)]
    old = [value for value in constants if value.startswith('        for corner in projected:')]
    new = [value for value in constants if value.startswith('        corner_edges=cv2.dilate')]
    assert len(old) == len(new) == 1
    assert 'if min(branch_support)<7./9.:' in new[0]
    return old[0], new[0]


def traced_visual(function, locations, args):
    stats = dict(combinations_examined=0, combinations_accepted=0,
                 rejection_counts=Counter(), examples=[])
    example_counts = Counter()
    seen_edge_checks = set()

    def reject(reason, local, edge=None):
        stats['rejection_counts'][reason] += 1
        if example_counts[reason] >= 3:
            return
        example_counts[reason] += 1
        row = dict(reason=reason, combination_index=stats['combinations_examined'], edge_index=edge)
        for key in ('corner_index', 'branch_support', 'corner_cov', 'visual_support',
                    'valid_inside', 'supported', 'valid_outside', 'continued', 'mask_boundary_count'):
            if key in local:
                value = local[key]
                row[key] = list(value) if isinstance(value, list) else float(value)
        if 'projected' in local:
            row['projected_corners_px'] = local['projected'].tolist()
        stats['examples'].append(row)

    def local_trace(frame, event, _arg):
        if event != 'line':
            return local_trace
        local, line = frame.f_locals, frame.f_lineno
        if line == locations['combination']:
            stats['combinations_examined'] += 1
        elif line == locations['size'] and np.any(local['hi'] - local['lo'] < .005):
            reject('rectangle_extent_below_5mm', local)
        elif line == locations['corner_branches'] and min(local['branch_support']) < 7. / 9.:
            reject('actual_rgb_corner_branch_below_7_of_9', local)
        elif line == locations['corner_margin']:
            if min(local['x'], local['y'], args[2].shape[1] - 1 - local['x'],
                   args[2].shape[0] - 1 - local['y']) < 4:
                reject('corner_image_margin_below_4px', local)
        elif line == locations['edge']:
            # A multiline OR emits repeated trace events for its first line.
            # Count the complete condition only once per combination edge.
            key = (stats['combinations_examined'], local['i'])
            if key in seen_edge_checks:
                return local_trace
            seen_edge_checks.add(key)
            if local['visual_support'] < .70:
                reject('rgb_edge_support_below_70_percent', local, local['i'])
            if local['valid_inside'] >= .60 and local['supported'] / max(local['valid_inside'], 1e-12) < .70:
                reject('positive_inner_depth_not_top_plane', local, local['i'])
            if local['valid_outside'] >= .20 and local['continued'] > .30:
                reject('positive_outside_top_continuation', local, local['i'])
            outside = local['outside']
            if np.mean(outside[0] & (outside[2] > .0025)) > .30:
                reject('higher_outside_occluder', local, local['i'])
        elif line == locations['accepted'] and local['good_candidate']:
            if local['mask_boundary_count'] >= 3:
                stats['combinations_accepted'] += 1
            else:
                reject('fewer_than_three_original_mask_boundaries', local)
        return local_trace

    previous = sys.gettrace()
    try:
        sys.settrace(lambda frame, _event, _arg: local_trace if frame.f_code is function.__code__ else None)
        result = function(*args)
    finally:
        sys.settrace(previous)
    stats['rejection_counts'] = dict(stats['rejection_counts'])
    return result, stats


def install(roi_radius, localization_radius):
    search.install_offline_search(roi_radius)
    source = inspect.getsource(box._visual_rectangle)
    old, new = corner_blocks()
    assert source.count(old) == 1
    new = new.replace('np.ones((3,3),np.uint8)',
                      'np.ones((_corner_diameter,_corner_diameter),np.uint8)')
    source = source.replace(old, new)
    old_domain = 'np.ones((5,5), np.uint8)) == 0'
    assert source.count(old_domain) == 1
    source = source.replace(old_domain, 'np.ones((_roi_diameter,_roi_diameter), np.uint8)) == 0')
    namespace = dict(vars(box), _roi_diameter=2 * roi_radius + 1,
                     _corner_diameter=2 * localization_radius + 1)
    exec(compile(source, '<offline_rgb_corner_localization>', 'exec'), namespace)
    visual_function = namespace['_visual_rectangle']
    lines = source.splitlines()

    def line_of(text):
        found = [i + 1 for i, line in enumerate(lines) if line.strip() == text]
        assert len(found) == 1, text
        return found[0]

    locations = dict(
        combination=line_of('lo=np.array([selected[3][0],selected[0][0]])'),
        size=line_of('if np.any(hi-lo < .005):'),
        corner_branches=line_of('if min(branch_support)<7./9.:'),
        corner_margin=line_of('if min(x,y,depth.shape[1]-1-x,depth.shape[0]-1-y)<4:'),
        edge=line_of('if (visual_support < .70 or'),
        accepted=line_of('if good_candidate and mask_boundary_count>=3:'))

    def make_qualifier():
        captured = {}
        namespace = dict(vars(box), _prototype_axes=None,
                         _corner_localization_radius=float(localization_radius))

        def visual(*args):
            captured['args'] = args
            started = time.perf_counter()
            result, trace = traced_visual(visual_function, locations, args)
            captured['visual'] = result
            captured['visual_seconds'] = time.perf_counter() - started
            TRACES.append(trace)
            return result

        source = inspect.getsource(box.qualify_regular_box_top)
        edits = {
            '        coord = top @ axes[:, :2]\n':
                '        if _prototype_axes is not None:\n            axes = _prototype_axes.copy()\n        coord = top @ axes[:, :2]\n',
            'edge_uncertainty = max(.001, 2.*rms+scale) + .5*pixel_size + unobserved_extension':
                'edge_uncertainty = max(.001, 2.*rms+scale) + (_corner_localization_radius if visual is not None else .5)*pixel_size + unobserved_extension',
        }
        for old, new in edits.items():
            assert source.count(old) == 1, old
            source = source.replace(old, new)
        namespace['_visual_rectangle'] = visual
        exec(compile(source, '<offline_rgb_corner_localization_qualifier>', 'exec'), namespace)
        return namespace, captured, namespace['qualify_regular_box_top']

    base.make_qualifier = make_qualifier


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path,
        default=base.INPUT.parent / 'gen28_rgb_corner_localization_prototype_r1')
    output = parser.parse_args().output.resolve()
    output.relative_to(base.EXTERNAL.resolve())
    assert Path('/media/zhuyupei/SanStand').is_mount()
    assert not output.exists(), 'immutable output exists'
    raw = base.INPUT / 'source_1792050956912783622.npz'
    geometry = base.INPUT / 'geometry_1792050956912783622.json'
    with np.load(raw, allow_pickle=False) as source:
        depth_scale = float(source['depth_scale_m'])
    files = [Path(__file__).resolve(), CORNER_SCRIPT.resolve(),
             Path(search.__file__).resolve(), Path(base.__file__).resolve(),
             Path(box.__file__).resolve(), Path(inspect.getfile(base.GeometryEstimate)).resolve(),
             Path(inspect.getfile(base.TargetTrackIdentity)).resolve(), raw, geometry]
    before = {str(path): base.digest(path) for path in files}
    output.mkdir(parents=True)
    summaries = []
    for roi in (4, 6):
        for radius in (1, 2, 3):
            install(roi, radius)
            TRACES.clear()
            report = base.qualify_source(raw, geometry,
                'offline_rgb_corner_localization_hypothesis_not_deployed',
                ['Nearfield raw source with separately recorded same-source geometry.',
                 'Both actual Canny corner branches require at least seven of nine samples.',
                 'Localization radius is paid as radius * metric_pixel_size in edge uncertainty.',
                 'All other thresholds and unobserved-boundary extension remain unchanged.',
                 'No positive or adversarial regression has been claimed for this hypothesis.'])
            assert len(TRACES) == len(report['candidates'])
            failures = Counter()
            final_codes = Counter()
            for row, trace in zip(report['candidates'], TRACES):
                row['visual_search_trace'] = trace
                failures.update(trace['rejection_counts'])
                final_codes[row['code']] += 1
                if row['ok'] and row['visual_rectangle_found']:
                    q = row['model']['qualification']
                    expected = max(.001, 2. * q['top_plane_rms_m'] + depth_scale)
                    expected += radius * q['metric_pixel_size_m'] + q['unobserved_boundary_extension_m']
                    assert abs(row['model']['empirical_edge_uncertainty_m'] - expected) < 1e-12
            report.update(roi_search_radius_px=roi, corner_localization_radius_px=radius,
                corner_edge_uncertainty_pixel_term=float(radius),
                original_other_thresholds_unchanged=True, unobserved_extension_unchanged=True,
                diagnostic_timing_includes_trace=True, no_ros_calls=True, no_hardware_access=True)
            target = output / ('nearfield_roi%d_corner%d.json' % (roi, radius))
            target.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
            selected = next((row for row in report['candidates'] if row['candidate_index'] == report['selected_candidate_index']), None)
            summary = dict(roi_radius_px=roi, corner_radius_px=radius,
                candidate_count=report['candidate_count'],
                visual_rectangle_count=sum(row['visual_rectangle_found'] for row in report['candidates']),
                qualified_rgb_candidate_count=report['qualified_rgb_candidate_count'],
                selected_relative_angle_deg=None if selected is None else selected['relative_angle_deg'],
                selected_dimensions_m=None if selected is None else selected['model']['size_xyz_m'],
                selected_edge_uncertainty_m=None if selected is None else selected['model']['empirical_edge_uncertainty_m'],
                visual_rejection_counts=dict(failures), final_codes=dict(final_codes),
                total_seconds=report['total_seconds'], path=str(target), sha256=base.digest(target))
            summaries.append(summary)
            print(json.dumps(summary, ensure_ascii=False), flush=True)
    after = {str(path): base.digest(path) for path in files}
    assert before == after
    receipt = dict(classification='offline_nearfield_corner_localization_comparison',
        source_and_input_sha256=before, source_and_input_unchanged=True,
        no_ros_calls=True, no_hardware_access=True, production_files_edited=False,
        results=summaries, negative_regression_pending=True,
        limitations=['No production release or executable grasp authorization.',
            'Model success alone does not establish aperture, collision or physical grasp.',
            'Printed-inner-frame/missing-corner/L-shape/depth-hole regressions are pending a separate decision.',
            'Trace condition counts may overlap and timing is not a production benchmark.'])
    target = output / 'receipt.json'
    target.write_text(json.dumps(receipt, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    print(json.dumps(dict(receipt=str(target), sha256=base.digest(target)), ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
