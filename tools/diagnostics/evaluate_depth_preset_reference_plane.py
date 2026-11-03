#!/usr/bin/env python3
"""Offline A/B/A residuals to one A1 common-support plane, without refitting B."""
import argparse
from dataclasses import fields
import json
from pathlib import Path
import sys
import time
import zipfile

import numpy as np

import analyze_depth_stage_probe as base


def configured_thresholds(folder, inputs):
    summary = json.loads(base.read_file(folder / 'summary.json', inputs))
    entries = {row['name']: row for row in summary['archive_entries']}
    records = []
    with zipfile.ZipFile(folder / 'diagnostic.zip') as archive:
        for row in summary['parameter_records']:
            name = row['entry']
            raw = archive.read(name)
            base.require(len(raw) == entries[name]['bytes'] and
                         base.sha256(raw) == entries[name]['sha256'], 'parameter archive hash mismatch')
            value = json.loads(raw)
            if value.get('parameter') == '/unknown_perception':
                base.require(value.get('original_rpc_code') == 1, 'missing perception parameter')
                records.append(dict(entry=name, sha256=base.sha256(raw), record=value))
    base.require(bool(records), 'no sealed perception parameters')
    overrides = records[0]['record']['original_parameter_value']
    base.require(all(row['record']['original_parameter_value'] == overrides for row in records),
                 'perception parameters changed')
    source = base.WORKTREE / 'src/alicia_grasp_modes/src'
    sys.path.insert(0, str(source))
    from alicia_grasp_modes.tabletop import Config
    module = source / 'alicia_grasp_modes/tabletop.py'
    base.read_file(module, inputs)
    config = Config(**{field.name: overrides[field.name] for field in fields(Config)
                       if field.name in overrides})
    return dict(support_threshold_m=config.support_threshold_m,
                support_min_ratio=config.support_min_ratio,
                threshold_origin='recorded override' if 'support_threshold_m' in overrides else
                    'production Config default; absent from all sealed parameter snapshots',
                overrides=overrides, parameter_records=records)


def summary_residuals(values, ray_dot_normal, offset, selection, scale, threshold):
    chosen = values[:, selection]
    valid = chosen > 0
    signed = chosen.astype(np.float64) * scale * ray_dot_normal[selection][None, :] + offset
    observed = signed[valid]
    absolute = np.abs(observed)
    count = int(valid.sum())
    inliers = np.abs(signed) <= threshold
    per_frame = []
    for good, residual, inlier in zip(valid, signed, inliers):
        data = np.abs(residual[good])
        per_frame.append(dict(measured_pixels=int(good.sum()),
            absolute_p50_p95_p99_m=np.percentile(data, [50, 95, 99]).tolist() if len(data) else None,
            inlier_fraction_of_measured=float(inlier[good].mean()) if len(data) else None))
    return dict(selected_pixels=int(selection.sum()), possible_observations=int(chosen.size),
        measured_observations=count, zero_observations=int(chosen.size - count),
        absolute_p50_p95_p99_m=np.percentile(absolute, [50, 95, 99]).tolist() if count else None,
        maximum_absolute_m=float(absolute.max()) if count else None,
        signed_p5_p50_p95_m=np.percentile(observed, [5, 50, 95]).tolist() if count else None,
        rms_m=float(np.sqrt(np.mean(observed ** 2))) if count else None,
        inlier_fraction_of_measured=float(inliers[valid].mean()) if count else None,
        inlier_fraction_of_possible=float((inliers & valid).sum() / chosen.size) if chosen.size else None,
        per_frame=per_frame)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--roi-spec', type=Path, required=True)
    parser.add_argument('--support-capture', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    started = time.monotonic()
    inputs = {}
    thresholds = configured_thresholds(args.support_capture, inputs)
    rois, _, reference, contract = base.load_roi_spec(args.roi_spec, inputs)
    rows = [base.verify_capture(args.session / label / 'capture', inputs)[1]
            for label in ('A1', 'B', 'A2')]
    rectifier = base.make_rectifier(sum(rows, []), base.WORKTREE / '.runtime/realsense-2.56.5', inputs)
    base.require(rectifier.contract == contract, 'reference rectification differs')
    source, target = base.source_roi(rectifier, rois['table']['bounds_xyxy'])
    intrinsics = contract['output_intrinsics']
    yy, xx = np.unravel_index(target, (intrinsics['height'], intrinsics['width']))
    rays = np.stack((np.ones(len(target)), -(xx - intrinsics['cx']) / intrinsics['fx'],
                     -(yy - intrinsics['cy']) / intrinsics['fy']), axis=1)
    scale = rows[0][0]['metadata']['context']['depth_scale_m']
    spatial = [np.stack([row['arrays']['after_filter_0_spatial'].ravel()[source] for row in segment])
               for segment in rows]
    always = [np.all(values > 0, axis=0) for values in spatial]
    common = np.logical_and.reduce(always)
    base.require(int(common.sum()) >= 3, 'insufficient common support')
    a1_median = np.median(spatial[0][:, common], axis=0) * scale
    points = a1_median[:, None] * rays[common]
    center = points.mean(axis=0)
    eigenvalues, eigenvectors = np.linalg.eigh((points - center).T @ (points - center))
    base.require(eigenvalues[1] > 0, 'degenerate reference plane support')
    normal = eigenvectors[:, 0]
    if normal[0] < 0:
        normal = -normal
    offset = -float(normal @ center)
    masks = dict(all_measured_locations=np.ones(len(target), dtype=bool),
        valid_in_every_frame_all_three=common,
        B_always_valid_but_each_A_has_a_hole=always[1] & ~always[0] & ~always[2],
        B_always_valid_but_both_A_always_zero=always[1] & np.all(spatial[0] == 0, axis=0)
            & np.all(spatial[2] == 0, axis=0))
    measured = {}
    for stage in ('after_align_depth', 'after_filter_0_spatial', 'after_rectification_depth'):
        indices = target if stage == 'after_rectification_depth' else source
        measured[stage] = {}
        for label, segment in zip(('A1', 'B', 'A2'), rows):
            values = np.stack([row['arrays'][stage].ravel()[indices] for row in segment])
            measured[stage][label] = {name: summary_residuals(values, rays @ normal, offset,
                mask, scale, thresholds['support_threshold_m']) for name, mask in masks.items()}
    result = dict(schema=1, kind='one_frozen_A1_common_support_reference_plane', diagnostic_only=True,
        input_files=inputs, roi_reference=reference, table_roi=rois['table'],
        depth_scale_m_per_unit=scale, thresholds=thresholds,
        reference_plane=dict(coordinates='camera_link [z,-x,-y]', normal=normal.tolist(), offset_m=offset,
            common_pixels=int(common.sum()), fitted_points=len(points), discarded_points=0,
            fit_stage='after_filter_0_spatial', depth_estimator='A1 per-pixel temporal median',
            fit_method='single unweighted total-least-squares plane; no RANSAC, pruning or threshold tuning',
            common_mask_sha256=base.sha256(common.tobytes()),
            initial_points_sha256=base.sha256(points.tobytes()), eigenvalues=eigenvalues.tolist(),
            reference_absolute_p50_p95_p99_m=np.percentile(np.abs(points @ normal + offset), [50,95,99]).tolist()),
        fixed_canvas_padding_excluded=
            (rois['table']['bounds_xyxy'][2]-rois['table']['bounds_xyxy'][0]) *
            (rois['table']['bounds_xyxy'][3]-rois['table']['bounds_xyxy'][1])-len(target),
        residuals=measured, quality_thresholds_changed=False, samples_pruned=False,
        interpretation_limits=['Reference is A1 geometry, not surveyed ground truth or absolute accuracy.',
            'The mat may not be perfectly planar; common support is selected by validity and favors A quality.',
            'All positive measurements, including residual outliers, are reported without clipping or refitting.',
            'Original support_min_ratio is contextual; this rectangular ROI is not the production support-ring acceptance test.',
            'Final output also depends on observed regional temporal eligibility and history.',
            'B-only points lack an A depth reference; distance to this plane does not prove their absolute correctness.'],
        ROS_contacted=False, SDK_device_access=False, motion_commands_sent=False,
        source_sha256=base.sha256(Path(__file__).read_bytes()), elapsed_sec=time.monotonic()-started)
    hashes = base.write_results(args.output, result)
    print(json.dumps(dict(output=str(args.output), files=hashes, elapsed_sec=result['elapsed_sec'])))


if __name__ == '__main__':
    main()
