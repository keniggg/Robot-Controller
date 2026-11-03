"""Offline 2/4/6/8 pixel RGB search-domain comparison; no robot interfaces.

Only the Canny search clipping radius changes. The actual object mask passed
to corner, boundary and occupancy qualification is never expanded. Every
original numerical qualification threshold and uncertainty formula remains.
"""
import argparse
from collections import Counter
import importlib.util
import inspect
import json
from pathlib import Path
import sys
import time

import numpy as np

BASE_PATH = Path(__file__).with_name('prototype_gen27_rgb_direction_box.py')
spec = importlib.util.spec_from_file_location('offline_rgb_direction_base', BASE_PATH)
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)
box = base.box
ALL_TRACES = []


def clone_search(radius):
    source = inspect.getsource(box._visual_rectangle)
    original = 'np.ones((5,5), np.uint8)) == 0'
    replacement = 'np.ones((_prototype_search_diameter,_prototype_search_diameter), np.uint8)) == 0'
    assert source.count(original) == 1
    source = source.replace(original, replacement)
    namespace = dict(vars(box), _prototype_search_diameter=2 * radius + 1)
    exec(compile(source, '<offline_rgb_search_domain>', 'exec'), namespace)
    function = namespace['_visual_rectangle']
    lines = source.splitlines()

    def line_of(text):
        found = [i + 1 for i, line in enumerate(lines) if line.strip() == text]
        assert len(found) == 1, (text, found)
        return found[0]

    locations = {
        'empty': line_of('if any(not values for values in choices):'),
        'combination': line_of('lo=np.array([selected[3][0],selected[0][0]])'),
        'size': line_of('if np.any(hi-lo < .005):'),
        'corner': line_of('if not good_candidate or min(corner_cov)<.5:'),
        'edge': line_of('if (visual_support < .70 or'),
        'accepted': line_of('if good_candidate and mask_boundary_count>=3:'),
    }
    return function, locations


def call_with_trace(function, locations, args):
    stats = dict(combinations_examined=0, combinations_accepted=0,
                 distinct_edge_counts_before_top3=None, empty_edge_indices=[],
                 rejection_reason_counts=Counter(), examples=[])
    examples_per_reason = Counter()

    def reject(reason, local, edge=None):
        stats['rejection_reason_counts'][reason] += 1
        if examples_per_reason[reason] >= 2:
            return
        examples_per_reason[reason] += 1
        row = dict(reason=reason, combination_index=stats['combinations_examined'], edge_index=edge)
        if 'projected' in local:
            row['projected_corners_px'] = local['projected'].tolist()
        if 'corner_cov' in local:
            row['corner_coverage'] = list(local['corner_cov'])
        for key in ('visual_support', 'valid_inside', 'supported', 'valid_outside', 'continued',
                    'mask_border_fraction', 'mask_boundary_count', 'unobserved_extension'):
            if key in local:
                row[key] = float(local[key])
        stats['examples'].append(row)

    def local_trace(frame, event, _arg):
        if event != 'line':
            return local_trace
        local, lineno = frame.f_locals, frame.f_lineno
        if lineno == locations['empty']:
            counts = [len(values) for values in local['choices']]
            stats['distinct_edge_counts_before_top3'] = counts
            stats['empty_edge_indices'] = [i for i, count in enumerate(counts) if count == 0]
        elif lineno == locations['combination']:
            stats['combinations_examined'] += 1
        elif lineno == locations['size']:
            if np.any(local['hi'] - local['lo'] < .005):
                reject('rectangle_extent_below_5mm', local)
        elif lineno == locations['corner']:
            if not local['good_candidate']:
                reject('corner_outside_image_margin', local)
            elif min(local['corner_cov']) < .5:
                reject('original_mask_corner_coverage_below_half', local)
        elif lineno == locations['edge']:
            edge = local['i']
            if local['visual_support'] < .70:
                reject('rgb_edge_support_below_70_percent', local, edge)
            if local['valid_inside'] >= .60 and local['supported'] / max(local['valid_inside'], 1e-12) < .70:
                reject('positive_inner_depth_not_top_plane', local, edge)
            if local['valid_outside'] >= .20 and local['continued'] > .30:
                reject('positive_outside_top_continuation', local, edge)
            outside = local['outside']
            if np.mean(outside[0] & (outside[2] > .0025)) > .30:
                reject('higher_outside_occluder', local, edge)
        elif lineno == locations['accepted']:
            if local['good_candidate']:
                if local['mask_boundary_count'] >= 3:
                    stats['combinations_accepted'] += 1
                else:
                    reject('fewer_than_three_original_mask_boundaries', local)
        return local_trace

    def global_trace(frame, _event, _arg):
        return local_trace if frame.f_code is function.__code__ else None

    previous = sys.gettrace()
    try:
        sys.settrace(global_trace)
        result = function(*args)
    finally:
        sys.settrace(previous)
    stats['rejection_reason_counts'] = dict(stats['rejection_reason_counts'])
    stats['visual_rectangle_found'] = result is not None
    return result, stats


def install_offline_search(radius):
    visual_function, locations = clone_search(radius)

    def make_qualifier():
        context_holder = {}
        namespace = dict(vars(box), _prototype_axes=None)

        def observe_visual(*args):
            context_holder['args'] = args
            started = time.perf_counter()
            result, trace = call_with_trace(visual_function, locations, args)
            context_holder['visual_seconds'] = time.perf_counter() - started
            context_holder['visual'] = result
            ALL_TRACES.append(trace)
            return result

        namespace['_visual_rectangle'] = observe_visual
        source = inspect.getsource(box.qualify_regular_box_top)
        original = '        coord = top @ axes[:, :2]\n'
        replacement = ('        if _prototype_axes is not None:\n'
                       '            axes = _prototype_axes.copy()\n' + original)
        assert source.count(original) == 1
        source = source.replace(original, replacement)
        exec(compile(source, '<offline_rgb_search_qualifier>', 'exec'), namespace)
        return namespace, context_holder, namespace['qualify_regular_box_top']

    source = inspect.getsource(ORIGINAL_HOUGH)
    original = 'np.ones((5, 5), np.uint8)) == 0'
    replacement = 'np.ones((_prototype_search_diameter, _prototype_search_diameter), np.uint8)) == 0'
    assert source.count(original) == 1
    source = source.replace(original, replacement)
    source = source.replace('mask_dilation_size_px=5,', 'mask_dilation_size_px=_prototype_search_diameter,')
    namespace = dict(vars(base), _prototype_search_diameter=2 * radius + 1)
    exec(compile(source, '<offline_rgb_search_actual_hough>', 'exec'), namespace)
    # Only this imported offline helper module is changed in memory. The real
    # box module, production source, and previous prototype file are untouched.
    base.make_qualifier = make_qualifier
    base.actual_hough_segments = namespace['actual_hough_segments']


ORIGINAL_HOUGH = base.actual_hough_segments


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=base.INPUT.parent / 'gen27_rgb_search_domain_prototype_r1')
    args = parser.parse_args()
    assert Path('/media/zhuyupei/SanStand').is_mount(), 'SanStand mount required; no fallback'
    output = args.output.resolve()
    output.relative_to(base.EXTERNAL.resolve())
    assert not output.exists(), 'immutable output already exists'
    reconstruction = json.loads((base.INPUT / 'initial_three_frame_replay.json').read_text())
    specs = [
        ('initial_single_frame', base.INPUT / 'source_1792050907639824151.npz', base.INPUT / 'geometry_1792050907639824151.json',
         'recorded_raw_frame_with_exact_source_geometry_audit', ['Not the original multi-frame initial planning snapshot.']),
        ('initial_three_frame_reconstruction', base.INPUT / 'initial_three_frame_reconstruction.npz', base.INPUT / 'initial_three_frame_geometry.json',
         reconstruction['classification'], reconstruction['limitations']),
        ('nearfield_single_frame', base.INPUT / 'source_1792050956912783622.npz', base.INPUT / 'geometry_1792050956912783622.json',
         'recorded_raw_frame_with_exact_source_geometry_audit', ['Raw source, not reconstruction of all nearfield fusion inputs.']),
    ]
    previous_receipt_path = base.INPUT.parent / 'gen27_rgb_direction_prototype_r1/receipt.json'
    previous_receipt = json.loads(previous_receipt_path.read_text())
    previous_hashes = previous_receipt['source_and_input_sha256']
    for path, expected in previous_hashes.items():
        assert base.digest(path) == expected, ('previous prototype input/source changed', path)
    paths = set(previous_hashes) | {str(Path(__file__).resolve()), str(previous_receipt_path)}
    before = {path: base.digest(path) for path in sorted(paths)}
    output.mkdir(parents=True)
    summaries = []
    for radius in (2, 4, 6, 8):
        install_offline_search(radius)
        for name, npz, geometry, classification, limitations in specs:
            ALL_TRACES.clear()
            report = base.qualify_source(npz, geometry, classification, limitations)
            assert len(ALL_TRACES) == len(report['candidates'])
            for row, trace in zip(report['candidates'], ALL_TRACES):
                row['visual_search_trace'] = trace
                assert trace['distinct_edge_counts_before_top3'] == row['edge_search']['distinct_counts_before_top3']
            reasons = Counter()
            for trace in ALL_TRACES:
                reasons.update(trace['rejection_reason_counts'])
            report.update(search_radius_px=radius, production_thresholds_and_uncertainty_unchanged=True,
                          original_qualification_mask_unchanged=True, no_ros_calls=True,
                          no_hardware_access=True, timing_includes_local_diagnostic_trace=True,
                          aggregate_visual_rejection_reason_counts=dict(reasons))
            target = output / (name + '_radius_%d.json' % radius)
            target.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
            selected = next((r for r in report['candidates'] if r['candidate_index'] == report['selected_candidate_index']), None)
            row = dict(name=name, radius_px=radius, path=str(target), sha256=base.digest(target),
                       hough_count=report['hough']['hough_total_count'], candidate_count=report['candidate_count'],
                       visual_rectangle_candidate_count=sum(r['visual_rectangle_found'] for r in report['candidates']),
                       qualified_rgb_candidate_count=report['qualified_rgb_candidate_count'],
                       selected_candidate_index=report['selected_candidate_index'],
                       selected_relative_angle_deg=None if selected is None else selected['relative_angle_deg'],
                       selected_dimensions_m=None if selected is None else selected['model']['size_xyz_m'],
                       aggregate_visual_rejection_reason_counts=dict(reasons), total_seconds=report['total_seconds'])
            summaries.append(row)
            print(json.dumps(row, ensure_ascii=False), flush=True)
    after = {path: base.digest(path) for path in sorted(paths)}
    assert before == after, 'immutable dependency changed'
    receipt = dict(classification='offline_search_domain_comparison_not_deployed_not_motion_authorization',
                   source_and_input_sha256=before, source_and_input_unchanged=True,
                   experiments=summaries, radii_px=[2, 4, 6, 8],
                   change='Only pre-Hough Canny search clipping dilation; original object mask, numerical checks and uncertainty formulas unchanged.',
                   rgb_edge_support_uses_same_actual_canny_search_map=True,
                   maximum_actual_rgb_directions=12, additional_original_depth_direction=1,
                   arbitrary_angle_scan=False, production_files_edited=False,
                   no_ros_calls=True, no_hardware_access=True,
                   limitations=['Timings include local function trace and are not production latency benchmarks.',
                                'Candidate rejection counts can overlap when an edge violates multiple checks.',
                                'A qualified model alone does not establish aperture, collision safety or executable grasp.'])
    target = output / 'receipt.json'
    target.write_text(json.dumps(receipt, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    print(json.dumps(dict(receipt=str(target), sha256=base.digest(target)), ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
