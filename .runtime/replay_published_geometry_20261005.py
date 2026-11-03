#!/usr/bin/env python3
"""Reuse per-frame production geometry on a closed read-only RGB capture."""
import argparse
from collections import Counter
from dataclasses import fields
import json
from pathlib import Path
import shutil

import cv2
import numpy as np
import replay_matched_shadow_geometry_20261005 as replay


def analyze(capture, config_path, output):
    if not replay.support.MOUNT.is_mount():
        raise ValueError('SanStand unavailable; no fallback')
    capture.resolve().relative_to(replay.support.EXT)
    output.resolve().relative_to(capture.resolve())
    original = json.loads((capture/'result.json').read_text())
    if any(original.get(k) is not False for k in ('robot_commands_sent','motion_authorized','contact_geometry_authorized')):
        raise ValueError('capture is not explicitly observation-only')
    parameters = json.loads(config_path.read_text())
    arguments = replay.geometry_arguments(parameters['/grasp_6d']['remote'])
    output.mkdir(exist_ok=False)
    report = dict(schema=1, capture=str(capture), original_result_sha256=replay.support.sha(capture/'result.json'),
        pipeline_config_source=str(config_path), pipeline_config_sha256=replay.support.sha(config_path),
        script_sha256=replay.support.sha(__file__), replay_sha256=replay.support.sha(replay.__file__),
        geometry_arguments=arguments, geometry_authority=False,target_identity_established=False,
        robot_commands_sent=False, coordinate_frame='camera_link_diagnostic',
        limitations=[
            'Per-frame initial selector, complete RGB refinement and pre-snapshot object_geometry only; original multi-frame identity, erosion/MAD and temporal snapshot are not recreated.',
            'Capture requested source-time TF and saved its matrix but not returned-stamp audit. Geometry below therefore uses real camera_link coordinates; saved base matrix is not promoted to strict source proof.',
            'Detector status is the latest observed online record when the RGB-D pair was selected. Stamp delta is reported; it is not fabricated same-frame classification.',
            'Geometry passing here does not authorize planning, contact or motion, or prove physical object dimensions.'], rows=[])
    cv2.setNumThreads(1)
    for entry in original['rows']:
        row=dict(source_stamp_ns=entry['source_stamp_ns'], capture_error=entry.get('error'),
            fresh_for_existing_frame_limit=entry.get('fresh_for_existing_frame_limit'),
            source_context_unchanged=entry.get('source_context_unchanged'))
        report['rows'].append(row)
        detector=entry.get('detector') or {}
        row['online_detector']={k:detector.get(k) for k in ('state','detail','generation','selection_stamp_ns',
            'source_stamp_ns','target_locked','target_lost','first_loss','metric_ambiguity_hold')}
        ns=detector.get('source_stamp_ns')
        row['online_detector_source_delta_sec']=None if ns is None else (entry['source_stamp_ns']-ns)*1e-9
        if 'source_file' not in entry:
            row['excluded']='no retained RGB-D source';continue
        file=Path(entry['source_file'])
        file.resolve().relative_to(capture.resolve())
        if replay.support.sha(file)!=entry['source_file_sha256']:
            raise ValueError('source file hash differs')
        with np.load(file,allow_pickle=False) as data:
            rgb,raw=data['color_bgr'],data['depth_raw']
            if (replay.support.array_sha(rgb)!=entry['source_rgb_sha256']
                    or replay.support.array_sha(raw)!=entry['source_depth_sha256']
                    or int(data['source_stamp_ns'])!=entry['source_stamp_ns']):
                raise ValueError('source array binding differs')
            if raw.dtype!=np.uint16:
                raise ValueError('this replay expects captured integer depth units')
            camera=entry['source_context']['camera']
            config=replay.tabletop.Config(**{f.name:entry['config'][f.name] for f in fields(replay.tabletop.Config)})
            cv2.setRNGSeed(0)
            result,masks=replay.replay_stage(raw,rgb,camera,np.eye(4),config,
                float((parameters.get('/unknown_perception') or {}).get('minimum_geometry_quality',.5)),arguments)
        row['replay']=result
        for kind,mask in masks.items():
            name=str(entry['source_stamp_ns'])+'_'+kind+'.png'
            if not cv2.imwrite(str(output/name),mask):raise OSError('mask save failed')
            result.setdefault('mask_artifacts',{})[kind]=dict(path=name,sha256=replay.support.sha(output/name))
    report['summary']=dict(rows=len(report['rows']),
        detector_states=dict(Counter(str(r['online_detector']['state'])+':'+str(r['online_detector']['detail']) for r in report['rows'])),
        replay_stops=dict(Counter(r['replay']['stop_stage']+':'+r['replay']['reason'] for r in report['rows'] if 'replay' in r)),
        measured_obb_sizes_mm=[r['replay']['geometry']['measured_obb_size_mm'] for r in report['rows']
            if r.get('replay',{}).get('geometry',{}).get('measured_obb_size_mm') is not None])
    shutil.copy2(__file__,output/Path(__file__).name)
    shutil.copy2(replay.__file__,output/Path(replay.__file__).name)
    (output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(report=str(output/'report.json'),sha256=replay.support.sha(output/'report.json'),summary=report['summary'])))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture',type=Path,required=True)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    analyze(args.capture,args.config,args.output)
