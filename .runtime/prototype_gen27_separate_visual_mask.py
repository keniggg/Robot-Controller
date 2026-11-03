"""Offline same-source RGB mask versus fused depth mask separation.

No production writes or ROS interfaces. Only _visual_rectangle's mask and the
visual interior-fill mask are changed in memory; all depth extraction remains
bound to the unchanged fused input mask. Canonical model schema is not changed,
so output models are diagnostics, not production-complete two-mask evidence.
"""
import argparse
import importlib.util
import inspect
import json
from pathlib import Path
import time

import numpy as np

DOMAIN_PATH = Path(__file__).with_name('prototype_gen27_rgb_search_domain.py')
spec = importlib.util.spec_from_file_location('offline_rgb_domain_base', DOMAIN_PATH)
domain = importlib.util.module_from_spec(spec)
spec.loader.exec_module(domain)
base, box = domain.base, domain.box


def install_visual_mask(raw_visual_mask, raw_visual_depth=None):
    visual_function, locations = domain.clone_search(2)

    def make_qualifier():
        captured = {}
        namespace = dict(vars(box), _prototype_axes=None, _prototype_visual_mask=raw_visual_mask,
                         _prototype_visual_depth=raw_visual_depth)

        def observe_visual(*args):
            captured['args'] = args
            started = time.perf_counter()
            result, trace = domain.call_with_trace(visual_function, locations, args)
            captured['visual_seconds'] = time.perf_counter() - started
            captured['visual'] = result
            domain.ALL_TRACES.append(trace)
            return result

        namespace['_visual_rectangle'] = observe_visual
        source = inspect.getsource(box.qualify_regular_box_top)
        edits = {
            '        coord = top @ axes[:, :2]\n':
                '        if _prototype_axes is not None:\n            axes = _prototype_axes.copy()\n        coord = top @ axes[:, :2]\n',
            '_visual_rectangle(color_bgr, mask, depth, scale, T, K,':
                '_visual_rectangle(color_bgr, _prototype_visual_mask, '
                + ('depth' if raw_visual_depth is None else '_prototype_visual_depth') + ', scale, T, K,',
            'np.count_nonzero(interior & mask)':
                'np.count_nonzero(interior & _prototype_visual_mask)',
        }
        for old, new in edits.items():
            assert source.count(old) == 1, old
            source = source.replace(old, new)
        exec(compile(source, '<offline_separate_visual_mask_qualifier>', 'exec'), namespace)
        return namespace, captured, namespace['qualify_regular_box_top']

    base.make_qualifier = make_qualifier


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=base.INPUT.parent / 'gen27_separate_visual_mask_prototype_r1')
    args = parser.parse_args()
    assert Path('/media/zhuyupei/SanStand').is_mount(), 'SanStand mount required; no fallback'
    output = args.output.resolve()
    output.relative_to(base.EXTERNAL.resolve())
    assert not output.exists(), 'immutable output already exists'
    raw_path = base.INPUT / 'source_1792050907106353521.npz'
    fused_path = base.INPUT / 'initial_three_frame_reconstruction.npz'
    geometry_path = base.INPUT / 'initial_three_frame_geometry.json'
    with np.load(raw_path, allow_pickle=False) as source:
        raw = {key: source[key].copy() for key in source.files}
    with np.load(fused_path, allow_pickle=False) as source:
        fused = {key: source[key].copy() for key in source.files}
    same_fields = ('color', 'source_stamp_ns', 'intrinsics', 'depth_scale_m', 'T_base_optical')
    for key in same_fields:
        assert np.array_equal(raw[key], fused[key]), ('same-source visual input mismatch', key)
    raw_mask = raw['object_mask'] > 0
    depth_mask = fused['object_mask'] > 0
    assert raw_mask.shape == depth_mask.shape
    manifest = json.loads((base.INPUT / 'source_manifest.json').read_text())
    raw_row = next(row for row in manifest['rows'] if row['source_stamp_ns'] == int(raw['source_stamp_ns']))
    assert base.digest(raw_path) == raw_row['sha256']
    previous_path = base.INPUT.parent / 'gen27_rgb_search_domain_prototype_r1/receipt.json'
    previous = json.loads(previous_path.read_text())
    before = dict(previous['source_and_input_sha256'])
    before.update({str(path): base.digest(path) for path in (Path(__file__).resolve(), DOMAIN_PATH.resolve(), raw_path, previous_path)})
    for path, expected in before.items():
        assert base.digest(path) == expected, ('immutable dependency changed', path)
    reconstruction = json.loads((base.INPUT / 'initial_three_frame_replay.json').read_text())
    output.mkdir(parents=True)
    summaries = []
    modes = [
        ('fused_visual_mask_control', False, False, False),
        ('raw_same_source_visual_mask', True, False, False),
        ('raw_visual_mask_and_local_depth', True, True, False),
        ('raw_full_qualification_with_fused_geometry', False, False, True),
    ]
    for name, separate, local_raw_depth, full_raw in modes:
        domain.install_offline_search(2)
        if separate:
            install_visual_mask(raw_mask, raw['depth_raw'] if local_raw_depth else None)
        domain.ALL_TRACES.clear()
        qualification_path = raw_path if full_raw else fused_path
        report = base.qualify_source(qualification_path, geometry_path, reconstruction['classification'], reconstruction['limitations'])
        assert len(domain.ALL_TRACES) == len(report['candidates'])
        for row, trace in zip(report['candidates'], domain.ALL_TRACES):
            row['visual_search_trace'] = trace
            assert trace['distinct_edge_counts_before_top3'] == row['edge_search']['distinct_counts_before_top3']
        report.update(separate_same_source_rgb_mask=separate, search_radius_px=2,
            separate_same_source_rgb_local_depth=local_raw_depth,
            full_qualification_uses_raw_frame=full_raw,
            fused_top_depth_extraction_preserved=not full_raw,
            production_thresholds_and_uncertainty_unchanged=True,
            same_source_fields_verified=list(same_fields),
            source_roles=dict(color='same recorded final source for all cases',
                top_depth='raw same-source depth' if full_raw else 'three-frame fused depth',
                top_depth_mask='raw same-source mask' if full_raw else 'fused depth mask',
                visual_mask='raw same-source mask' if separate or full_raw else 'fused depth mask',
                visual_local_depth='raw same-source depth' if local_raw_depth or full_raw else 'three-frame fused depth',
                final_outside_depth_validation='raw same-source depth' if full_raw else 'three-frame fused depth',
                geometry='unchanged initial_three_frame_geometry: fused measured points and fitted support; not a raw single-frame geometry estimate'),
            depth_mask_sha256=box._array_hash(raw['object_mask'] if full_raw else fused['object_mask']),
            top_depth_sha256=box._array_hash(raw['depth_raw'] if full_raw else fused['depth_raw']),
            visual_mask_sha256=box._array_hash(raw['object_mask'] if separate or full_raw else fused['object_mask']),
            visual_local_depth_sha256=box._array_hash(raw['depth_raw'] if local_raw_depth or full_raw else fused['depth_raw']),
            raw_visual_mask_source=str(raw_path),
            raw_visual_mask_pixels=int(np.count_nonzero(raw_mask)), fused_depth_mask_pixels=int(np.count_nonzero(depth_mask)),
            raw_only_mask_pixels=int(np.count_nonzero(raw_mask & ~depth_mask)),
            fused_only_mask_pixels=int(np.count_nonzero(depth_mask & ~raw_mask)),
            no_ros_calls=True, no_hardware_access=True,
            canonical_model_binds_all_input_roles=not separate,
            provenance_limitation=('Prototype report binds separate raw visual inputs; unchanged canonical model schema only binds main qualifier input hashes. Not production authorization evidence.'
                if separate else 'Canonical model binds supplied raw/fused qualifier inputs and supplied fused geometry separately; geometry is not claimed to be a raw single-frame estimate.'))
        target = output / (name + '.json')
        target.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
        selected = next((row for row in report['candidates'] if row['candidate_index'] == report['selected_candidate_index']), None)
        summary = dict(name=name, path=str(target), sha256=base.digest(target),
            candidate_count=report['candidate_count'], qualified_rgb_candidate_count=report['qualified_rgb_candidate_count'],
            visual_rectangle_candidate_count=sum(row['visual_rectangle_found'] for row in report['candidates']),
            selected_candidate_index=report['selected_candidate_index'],
            selected_relative_angle_deg=None if selected is None else selected['relative_angle_deg'],
            selected_dimensions_m=None if selected is None else selected['model']['size_xyz_m'],
            total_seconds=report['total_seconds'])
        summaries.append(summary)
        print(json.dumps(summary, ensure_ascii=False), flush=True)
    after = {path: base.digest(path) for path in before}
    assert before == after
    receipt = dict(classification='offline_separate_mask_comparison_not_deployed_not_motion_authorization',
        source_and_input_sha256=before, source_and_input_unchanged=True, experiments=summaries,
        modified_in_memory_only=['candidate axes before original coord computation',
            'mask-only case: raw mask passed to _visual_rectangle and visual interior fill',
            'local-depth case: additionally raw depth used only inside _visual_rectangle',
            'full-raw case: raw frame passed to unchanged qualifier, separately bound unchanged fused geometry'],
        all_numerical_thresholds_and_uncertainty_formulas_unchanged=True,
        same_source_fields_verified=list(same_fields), search_radius_px=2,
        no_ros_calls=True, no_hardware_access=True, production_files_edited=False,
        limitations=reconstruction['limitations'] + [
            'Separate visual input cases bind extra source roles in the report only; canonical model schema remains unchanged.',
            'Full-raw case retains fused geometry explicitly; it does not reconstruct or claim a raw-frame geometry estimate.',
            'A qualified model is not aperture, collision or executable grasp validation.'])
    target = output / 'receipt.json'
    target.write_text(json.dumps(receipt, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    print(json.dumps(dict(receipt=str(target), sha256=base.digest(target)), ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
