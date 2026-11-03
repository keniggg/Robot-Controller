#!/usr/bin/env python3
"""Describe two sealed ROS RGB-D captures after a declared surface change."""
import argparse
import importlib.util
import json
from pathlib import Path
import time
import zipfile

import cv2
import numpy as np

import analyze_depth_stage_probe as base
import compare_depth_preset_captures as quality


def load_capture(folder, inputs, helper):
    summary_raw = base.read_file(folder / 'summary.json', inputs)
    summary = json.loads(summary_raw)
    sums = {name: digest for digest, name in (line.split(None, 1) for line in
            (folder / 'SHA256SUMS').read_text().splitlines() if line.strip())}
    base.require(sums['summary.json'] == base.sha256(summary_raw) and
        sums['diagnostic.zip'] == summary['archive_sha256'], 'capture checksum mismatch')
    base.require(summary['sealed'] and summary['ready'] and summary['error'] is None and
        summary['rpc_thread_alive'] is False and summary['pair_count'] == 10, 'capture incomplete')
    base.require(base.sha256(base.read_file(folder / 'diagnostic.zip', inputs)) ==
                 summary['archive_sha256'], 'archive hash mismatch')
    entries = {row['name']: row for row in summary['archive_entries']}
    depths, colors, rows, feedback = [], [], [], []
    with zipfile.ZipFile(folder / 'diagnostic.zip') as archive:
        base.require(set(archive.namelist()) == set(entries) and len(archive.namelist()) == len(entries),
                     'archive entry list differs')
        def read(name):
            raw = archive.read(name)
            base.require(len(raw) == entries[name]['bytes'] and base.sha256(raw) == entries[name]['sha256'],
                         'entry hash mismatch: ' + name)
            return raw
        metadata = [json.loads(read(row['entry'])) for row in summary['parameter_records']]
        cameras = [row['original_parameter_value'] for row in metadata if row.get('parameter') == '/camera'
                   and row.get('original_rpc_code') == 1]
        base.require(bool(cameras), 'camera configuration absent')
        camera = cameras[0]
        base.require(all(helper.config_invariant('/camera', v) == helper.config_invariant('/camera', camera)
                         for v in cameras), 'camera configuration changed within capture')
        for saved in summary['pairs']:
            pair = json.loads(read(saved['entry']))
            base.require(pair['source_stamp_ns'] == saved['source_stamp_ns'], 'source stamp conflict')
            decoded = {}
            for image in pair['images']:
                raw = helper.bounded_original_message(read(image['entry']), image['serialized_bytes'])
                base.require(base.sha256(raw) == image['serialized_sha256'], 'original message hash mismatch')
                header, pixels = helper.decode_image(raw)
                base.require(header['source_stamp_ns'] == saved['source_stamp_ns'] and
                    header['frame_id'] == saved['frame_id'] and header['seq'] == image['source_seq'],
                    'original header differs')
                base.require(pixels.shape[:2] == (camera['height'], camera['width']), 'image geometry differs')
                decoded[image['stream']] = pixels
            base.require(set(decoded) == {'color', 'depth'}, 'pair streams differ')
            depths.append(decoded['depth']); colors.append(decoded['color'])
            exact, audit = helper.exact_read_evidence(saved, read, camera)
            regional = exact['evidence'].get('regional_stationary_evidence', {}) if exact else {}
            rows.append(dict(source_stamp_ns=saved['source_stamp_ns'], exact_read_history_audit=audit,
                at_source=pair['at_source'], regional={key: regional.get(key) for key in
                    ('active', 'filtered_fraction', 'current_zero_pixels_preserved', 'global_history_reset_reason')}))
            for edge in ('before', 'after'):
                item = pair['at_source']['feedback'].get(edge)
                if item:
                    feedback.append(item)
    names = feedback[0]['names'] if feedback else []
    base.require(all(item['names'] == names for item in feedback), 'feedback joint names differ')
    positions = np.asarray([item['positions'] for item in feedback])
    stamps = [row['source_stamp_ns'] for row in rows]
    base.require(len(set(stamps)) == 10 and all(b > a for a, b in zip(stamps, stamps[1:])), 'source reuse/order')
    audit = dict(folder=str(folder), frame_count=len(rows), camera=camera, rows=rows,
        source_intervals_sec=[(b-a)/1e9 for a,b in zip(stamps,stamps[1:])],
        exact_read_matches=sum(row['exact_read_history_audit']['available'] for row in rows),
        feedback_bracket_count=sum(row['at_source']['feedback']['source_bracket_available'] for row in rows),
        exact_tf_count=sum((row['at_source'].get('tf') or {}).get('exact_valid', False) for row in rows),
        joint_names=names, recorded_joint_ranges=np.ptp(positions, axis=0).tolist() if feedback else None,
        recorded_joint_min=positions.min(axis=0).tolist() if feedback else None,
        recorded_joint_max=positions.max(axis=0).tolist() if feedback else None)
    return np.stack(depths), np.stack(colors), audit


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--before', type=Path, required=True)
    parser.add_argument('--after', type=Path, required=True)
    parser.add_argument('--roi-spec', type=Path, required=True)
    parser.add_argument('--stage-reference', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(); started = time.monotonic(); inputs = {}
    helper_path = base.WORKTREE / '.runtime/analyze_current_support_rgbd_20261007.py'
    base.read_file(helper_path, inputs)
    spec = importlib.util.spec_from_file_location('sealed_support_reader', helper_path)
    helper = importlib.util.module_from_spec(spec); spec.loader.exec_module(helper)
    declaration = json.loads(base.read_file(args.roi_spec, inputs))
    rgb_bytes = base.read_file(Path(declaration['source_rgb']['path']), inputs)
    base.require(base.sha256(rgb_bytes) == declaration['source_rgb']['sha256'], 'ROI RGB identity differs')
    datasets = [load_capture(path, inputs, helper) for path in (args.before, args.after)]
    camera = datasets[0][2]['camera']; other = datasets[1][2]['camera']
    for key in ('width','height','fx','fy','cx','cy','depth_scale'):
        base.require(camera[key] == other[key], 'cross-capture camera geometry differs: ' + key)
    base.require(camera['runtime_profile']['rectification'] == other['runtime_profile']['rectification'],
                 'cross-capture rectification differs')
    base.require(all(d[2]['camera']['runtime_profile']['mode_depth_preset']['preset'] == 'high_accuracy'
                     for d in datasets), 'surface comparison preset changed')
    stage_summary = json.loads(base.read_file(args.stage_reference / 'summary.json', inputs))
    meta_entry = next(entry for entry in stage_summary['files'] if entry['name'] == 'pair_01.json')
    raw = base.read_file(args.stage_reference / 'pair_01.json', inputs)
    base.require(len(raw) == meta_entry['bytes'] and base.sha256(raw) == meta_entry['sha256'], 'stage reference differs')
    rectifier = base.make_rectifier([dict(metadata=json.loads(raw))],
                                  base.WORKTREE / '.runtime/realsense-2.56.5', inputs)
    base.require(rectifier.contract == camera['runtime_profile']['rectification'], 'fixed map differs')
    scale = camera['depth_scale']; scopes = {}; labels = ('before', 'paper')
    for scope, bounds in declaration['rois'].items():
        _, target = base.source_roi(rectifier, bounds)
        values = [data[0].reshape(10, -1)[:, target] for data in datasets]
        common = np.all(values[0] > 0, axis=0) & np.all(values[1] > 0, axis=0)
        x0,y0,x1,y1 = bounds
        scopes[scope] = dict(bounds_xyxy=bounds, mapped_pixels=len(target),
            fixed_unmapped_canvas_pixels=(x1-x0)*(y1-y0)-len(target), common_always_valid_pixels=int(common.sum()),
            segments={label:base.sequence(value, scale, list(range(10))) for label,value in zip(labels,values)},
            repeatability_common_always_valid={label:quality.repeatability(value,common,scale)
                for label,value in zip(labels,values)},
            mean_grayscale_per_frame={label:[float(cv2.cvtColor(rgb,cv2.COLOR_BGR2GRAY)[y0:y1,x0:x1].mean())
                for rgb in data[1]] for label,data in zip(labels,datasets)})
    result = dict(schema=1, kind='finite_surface_before_after_descriptive_comparison', input_files=inputs,
        roi_declaration=declaration, captures={label:data[2] for label,data in zip(labels,datasets)},
        scopes=scopes, interpretation_limits=[
            'Printed paper is slightly curled and is not a planar or absolute-depth reference.',
            'Material, texture, color, height/normal and auto-exposure may all change together.',
            'The unchanged control ROI was selected from paper-scene RGB before its burst; applied retrospectively to old baseline.',
            'Published processed depth does not provide new same-frame native stage evidence.',
            'Temporal differences describe sampled positive depth only; missing samples remain missing.',
            'The captures have distinct camera geometry activations; equality of geometry does not itself prove physical pose.',
            'No support-plane, target-lock, grasp or automatic-motion acceptance follows from these statistics.'],
        SDK_device_access=False, ROS_contacted=False, motion_commands_sent=False, plane_fitting_performed=False,
        source_sha256=base.sha256(Path(__file__).read_bytes()), elapsed_sec=time.monotonic()-started)
    base.write_results(args.output,result)
    np.savez(args.output/'display_arrays.npz', before_rgb=datasets[0][1][0], paper_rgb=datasets[1][1][0],
             before_last_rgb=datasets[0][1][-1], paper_last_rgb=datasets[1][1][-1],
             before_zero_frequency=(datasets[0][0]==0).mean(axis=0),
             paper_zero_frequency=(datasets[1][0]==0).mean(axis=0), mapped_target_indices=rectifier.target_indices)
    print(json.dumps(dict(output=str(args.output), elapsed_sec=result['elapsed_sec'],
        result_sha256=base.sha256((args.output/'result.json').read_bytes())),ensure_ascii=False))


if __name__ == '__main__':
    main()
