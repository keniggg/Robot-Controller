#!/usr/bin/env python3
"""Diagnostic reuse of same-frame GrabCut using CURRENT independent RGB seeds.

The legacy function's depth-oriented labels are not adopted as valid evidence.
No old mask seeds a new segmentation, and no old support plane is supplied.
"""
import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path
import shutil

import cv2
import numpy as np

import check_saved_rgb_hold_overlap_20261005 as base
from alicia_grasp_modes.rgb_instance import instance_mask_from_seed
from alicia_grasp_modes.rgb_observation_region import RgbObservationRegions


def current_outlines(rgb, regions, kwargs, output, prefix):
    rows = []
    refined = []
    for i, region in enumerate(regions.candidates):
        row = dict(candidate_index=i, native_bbox=list(region.bbox_xywh), native_pixels=region.area_px)
        rows.append(row)
        try:
            mask, legacy = instance_mask_from_seed(rgb, region.mask, geometry=None, **kwargs)
            mask = base._mask(mask, rgb.shape[:2])
            ys, xs = np.nonzero(mask)
            bbox = tuple(map(int, cv2.boundingRect(mask)))
            area = int(np.count_nonzero(mask))
            result = replace(region, mask=mask, bbox_xywh=bbox,
                centroid_xy=(float(np.mean(xs)), float(np.mean(ys))), area_px=area)
            name = f'{prefix}_candidate_{i}_current_rgb_outline.png'
            if not cv2.imwrite(str(output/name), mask*255):
                raise OSError('mask write failed')
            row.update(status='refined_rgb_only_diagnostic', refined_bbox=list(bbox), refined_pixels=area,
                mask_artifact=dict(path=name, sha256=base.sha(output/name)),
                evidence_type='independent_current_rgb_seeded_outline_diagnostic',
                input_seed_source='current RGB region; internal eroded core, no depth or previous mask',
                current_core_retained_fraction=legacy['trusted_core_retained_fraction'],
                current_region_retained_fraction=legacy['depth_seed_retained_fraction'],
                current_outline_to_region_area_ratio=legacy['rgb_to_depth_seed_area_ratio'],
                search_attempts=legacy['rgb_search_attempts'], search_margin_px=legacy['rgb_search_margin_px'],
                legacy_depth_evidence_claimed=False, metric_geometry_authority=False)
            refined.append(result)
        except ValueError as exc:
            row.update(status='rejected_by_original_inner_algorithm', reason=str(exc))
    return RgbObservationRegions(tuple(refined), None, 'RGB_only_diagnostic_refinement', rgb.shape[:2], len(refined)), rows


def analyze(capture, geometry, output):
    if not base.MOUNT.is_mount():
        raise RuntimeError('SanStand unavailable; no fallback')
    capture.resolve().relative_to(base.EXT)
    output.resolve().relative_to(capture.resolve())
    record = json.loads((capture/'result.json').read_text())
    previous = json.loads((geometry/'report.json').read_text())
    if previous['original_result_sha256'] != base.sha(capture/'result.json'):
        raise ValueError('geometry capture hash mismatch')
    geom = {r['source_stamp_ns']:r for r in previous['rows']}
    output.mkdir(exist_ok=False)
    cv2.setNumThreads(1)
    frames = []
    for entry in record['rows']:
        file = Path(entry['source_file'])
        file.resolve().relative_to(capture.resolve())
        if base.sha(file) != entry['source_file_sha256']:
            raise ValueError('source file mismatch')
        with np.load(file, allow_pickle=False) as data:
            rgb = data['color_bgr']
            if base.array_sha(rgb) != entry['source_rgb_sha256']:
                raise ValueError('source RGB mismatch')
        valid = base.rectification_rgb_valid_mask(entry['source_context']['camera'], rgb.shape[:2])
        kwargs = {} if valid is None else {'rgb_valid':valid}
        mask = None
        artifact = geom[entry['source_stamp_ns']].get('replay',{}).get('mask_artifacts',{}).get('refined')
        if artifact is not None:
            path = geometry/artifact['path']
            if base.sha(path) != artifact['sha256']:
                raise ValueError('old measurement artifact mismatch')
            mask = base._mask(cv2.imread(str(path),cv2.IMREAD_GRAYSCALE),rgb.shape[:2])
        frames.append(dict(entry=entry, rgb=rgb, kwargs=kwargs, reference_mask=mask))
    rows = []
    for i, ref in enumerate(frames):
        old = ref['reference_mask']
        if old is None:
            continue
        x,y,w,h = cv2.boundingRect(old)
        policy = base.CenterSearchPolicy(((x+.5*w)/ref['rgb'].shape[1], (y+.5*h)/ref['rgb'].shape[0]),
            radius_fraction=ref['entry']['config']['center_radius_fraction'])
        for j in (i, i+1):
            if j == len(frames):
                continue
            current = frames[j]
            delta = (current['entry']['source_stamp_ns']-ref['entry']['source_stamp_ns'])*1e-9
            if not 0 <= delta <= .8:
                continue
            # Previous mask participates ONLY in attention center and final
            # identity comparison. Every segmentation seed below is CURRENT RGB.
            regions = base.propose_rgb_observation_regions(current['rgb'], policy=policy, **current['kwargs'])
            outlines, details = current_outlines(current['rgb'], regions, current['kwargs'], output, f'r{i+1}_c{j+1}')
            matched = base.compare(ref['rgb'], old, current['rgb'], outlines)
            rows.append(dict(reference_index=i+1,current_index=j+1,source_gap_seconds=delta,
                reference_stamp_ns=ref['entry']['source_stamp_ns'],current_stamp_ns=current['entry']['source_stamp_ns'],
                comparison='same_frame' if i==j else 'next_saved_frame',
                search_policy=asdict(policy), original_proposal_reason=regions.reason,
                original_proposal_count=len(regions.candidates), current_candidate_refinements=details,
                original_direct_rgb=base.compare(ref['rgb'],old,current['rgb'],regions),
                current_rgb_seeded_outline=matched,
                all_current_candidates_completed=len(outlines.candidates)==len(regions.candidates),
                identity_gates_and_all_candidates_passed=bool(matched['original_unique_rgb_gate_passed']
                    and len(outlines.candidates)==len(regions.candidates))))
    summary={}
    for kind in ('same_frame','next_saved_frame'):
        selected=[r for r in rows if r['comparison']==kind]
        summary[kind]=dict(comparisons=len(selected),
            unique_identity_gates_passed=sum(r['current_rgb_seeded_outline']['original_unique_rgb_gate_passed'] for r in selected),
            all_candidates_completed_and_unique_identity=sum(r['identity_gates_and_all_candidates_passed'] for r in selected),
            maximum_iou_range=[min(r['current_rgb_seeded_outline']['maximum_iou'] for r in selected),
                               max(r['current_rgb_seeded_outline']['maximum_iou'] for r in selected)],
            total_candidate_count=sum(len(r['current_candidate_refinements']) for r in selected),
            rejected_candidate_count=sum(sum(c['status']=='rejected_by_original_inner_algorithm' for c in r['current_candidate_refinements']) for r in selected))
    report=dict(schema=1,capture=str(capture),capture_sha256=base.sha(capture/'result.json'),
        geometry_report_sha256=base.sha(geometry/'report.json'),script_sha256=base.sha(__file__),
        comparison_script_sha256=base.sha(base.__file__),
        inner_algorithm_source=str(base.WORK/'src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_instance.py'),
        inner_algorithm_sha256=base.sha(base.WORK/'src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_instance.py'),
        authority=dict(robot_commands_sent=False,metric_geometry=False,motion=False,target_identity=False),
        limits=dict(iou_minimum=.85,colour_maximum=.4,shape_maximum=.4,ambiguity_margin=.12,hold_seconds=.8),
        limitations=[
            'Existing geometry=None GrabCut inner algorithm is reused with current independent RGB seeds as a feasibility diagnostic. Its old depth-specific evidence names and measured-foreground assumptions are NOT adopted as a production evidence contract.',
            'Old mask is NEVER segmentation seed or geometry. It supplies attention center and identity reference only; support/depth are not passed to outline extraction.',
            'Exact failed frame and runtime private cache are absent. Same-frame and adjacent saved-frame conditional matches cannot establish the actual failure would have been held.',
            'Candidate extraction failures are retained. Identity passing on successful candidates alone is insufficient unless remaining candidate ownership is separately resolved.',
            'Terminal generation 5 cannot be revived. No current support or metric/contact geometry is granted by an RGB outline.'],rows=rows,summary=summary)
    shutil.copy2(__file__,output/Path(__file__).name)
    shutil.copy2(base.__file__,output/Path(base.__file__).name)
    (output/'report.json').write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    print(json.dumps(dict(report=str(output/'report.json'),sha256=base.sha(output/'report.json'),summary=summary)))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture',type=Path,required=True)
    parser.add_argument('--geometry',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    a=parser.parse_args()
    analyze(a.capture,a.geometry,a.output)
