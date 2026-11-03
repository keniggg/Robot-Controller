#!/usr/bin/env python3
"""One offline high_accuracy-to-saved_baseline deployment observation."""
import argparse
import importlib.util
import json
from pathlib import Path
import time

import cv2
import numpy as np

import analyze_depth_stage_probe as base
import compare_depth_preset_captures as quality
from compare_depth_surface_support import load_capture


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--before', type=Path, required=True)
    parser.add_argument('--deployment', type=Path, required=True)
    parser.add_argument('--roi-spec', type=Path, required=True)
    parser.add_argument('--stage-reference', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(); started = time.monotonic(); inputs = {}
    path = base.WORKTREE / '.runtime/analyze_current_support_rgbd_20261007.py'
    base.read_file(path, inputs)
    spec = importlib.util.spec_from_file_location('sealed_deployment_support_reader', path)
    helper = importlib.util.module_from_spec(spec); spec.loader.exec_module(helper)
    declaration = json.loads(base.read_file(args.roi_spec, inputs))
    base.require(base.sha256(base.read_file(Path(declaration['source_rgb']['path']), inputs)) ==
                 declaration['source_rgb']['sha256'], 'ROI RGB differs')
    deployment = json.loads(base.read_file(args.deployment / 'result.json', inputs))
    datasets = [load_capture(path, inputs, helper) for path in
                (args.before, args.deployment / 'post_capture')]
    cameras = [data[2]['camera'] for data in datasets]
    for camera, preset in zip(cameras, ('high_accuracy', 'saved_baseline')):
        observed = camera['runtime_profile']['mode_depth_preset']
        base.require(observed['preset'] == preset and observed['mode'] == 'unknown',
                     'observed preset/mode differs from this comparison')
    for key in ('width', 'height', 'fx', 'fy', 'cx', 'cy', 'depth_scale'):
        base.require(cameras[0][key] == cameras[1][key], 'camera geometry differs: ' + key)
    for key in ('rectification', 'depth_filters', 'color_projection_correction'):
        base.require(cameras[0]['runtime_profile'][key] == cameras[1]['runtime_profile'][key],
                     'camera filter/geometry differs: ' + key)
    summary = json.loads(base.read_file(args.stage_reference / 'summary.json', inputs))
    expected = next(row for row in summary['files'] if row['name'] == 'pair_01.json')
    raw = base.read_file(args.stage_reference / 'pair_01.json', inputs)
    base.require(len(raw) == expected['bytes'] and base.sha256(raw) == expected['sha256'], 'map source differs')
    rectifier = base.make_rectifier([dict(metadata=json.loads(raw))],
                                  base.WORKTREE / '.runtime/realsense-2.56.5', inputs)
    base.require(all(rectifier.contract == camera['runtime_profile']['rectification'] for camera in cameras),
                 'reference/capture rectification differs')
    scale = cameras[0]['depth_scale']; labels = ('high_accuracy_paper_r2', 'deployed_saved_baseline')
    scopes = {}
    for scope, bounds in declaration['rois'].items():
        _, target = base.source_roi(rectifier, bounds)
        values = [data[0].reshape(10, -1)[:, target] for data in datasets]
        common = np.all(values[0] > 0, axis=0) & np.all(values[1] > 0, axis=0)
        x0, y0, x1, y1 = bounds
        before_center, after_center = [np.median(value[:, common], axis=0) * scale for value in values]
        scopes[scope] = dict(bounds_xyxy=bounds, mapped_pixels=len(target),
            fixed_unmapped_canvas_pixels=(x1-x0)*(y1-y0)-len(target), common_always_valid_pixels=int(common.sum()),
            segments={label:base.sequence(value, scale, list(range(10))) for label,value in zip(labels,values)},
            repeatability_common_always_valid={label:quality.repeatability(value,common,scale)
                for label,value in zip(labels,values)},
            median_depth_change_on_common_support_m=quality.quantiles(after_center-before_center),
            mean_grayscale_per_frame={label:[float(cv2.cvtColor(rgb,cv2.COLOR_BGR2GRAY)[y0:y1,x0:x1].mean())
                for rgb in data[1]] for label,data in zip(labels,datasets)},
            first_last_RGB_mean_absolute_channel_difference={label:float(np.abs(
                data[1][-1,y0:y1,x0:x1].astype(float)-data[1][0,y0:y1,x0:x1].astype(float)).mean())
                for label,data in zip(labels,datasets)})
    result = dict(schema=1, kind='deployed_preset_same_paper_scene_observation', input_files=inputs,
        deployment_receipt=deployment, roi_declaration=declaration,
        captures={label:data[2] for label,data in zip(labels,datasets)}, scopes=scopes,
        source_sha256=base.sha256(Path(__file__).read_bytes()), elapsed_sec=time.monotonic()-started,
        interpretation_limits=[
            'These are two separate sparse 10-frame processed ROS sequences; no new native-stage attribution.',
            'Per-frame exposure truth remains unavailable; RGB review cannot prove all unsampled physical motion absent.',
            'Common-valid repeatability excludes missing locations and is not absolute accuracy.',
            'Paper is curled; no plane truth, fitting, outlier pruning or geometric acceptance was performed.',
            'Actual support subgate evidence and deployment acceptance remain separate from full perception or grasp validation.'],
        SDK_device_access=False, ROS_contacted=False, motion_commands_sent=False, full_perception_or_grasp_passed=False)
    base.write_results(args.output, result)
    np.savez(args.output/'display_arrays.npz', before_rgb=datasets[0][1][0],
             deployed_rgb=datasets[1][1][0], deployed_last_rgb=datasets[1][1][-1],
             before_zero_frequency=(datasets[0][0]==0).mean(axis=0),
             deployed_zero_frequency=(datasets[1][0]==0).mean(axis=0), mapped_target_indices=rectifier.target_indices)
    print(json.dumps(dict(output=str(args.output), elapsed_sec=result['elapsed_sec'],
                         sha256=base.sha256((args.output/'result.json').read_bytes())), ensure_ascii=False))


if __name__ == '__main__':
    main()
