#!/usr/bin/env python3
"""Offline feasibility only: original independent RGB proposal identity gates.

No ROS, device, tracking authority, threshold changes or missing-frame synthesis.
"""
import argparse
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import shutil
import sys

import cv2
import numpy as np

WORK = Path(__file__).resolve().parents[1]
for rel in ('src/alicia_grasp_modes/src', 'src/alicia_flexible_grasp_supervisor/src'):
    sys.path.insert(0, str(WORK / rel))
from alicia_grasp_modes.rgb_observation_region import CenterSearchPolicy, propose_rgb_observation_regions
from alicia_grasp_modes.rgb_observation_handoff import HandoffPolicy, _mask, _appearance
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask

MOUNT = Path('/media/zhuyupei/SanStand')
EXT = MOUNT / '实验记录/Robot-Controller'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def array_sha(value):
    return hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest()


def compare(reference_rgb, reference_mask, current_rgb, regions):
    hist, contour = _appearance(reference_rgb, reference_mask)
    policy = HandoffPolicy()
    candidates = []
    matches = []
    for index, candidate in enumerate(regions.candidates):
        mask = _mask(candidate.mask, current_rgb.shape[:2])
        other_hist, other_contour = _appearance(current_rgb, mask)
        iou = float(np.count_nonzero(mask & reference_mask) / np.count_nonzero(mask | reference_mask))
        colour = float(np.sqrt(max(0., 1.-np.sum(np.sqrt(hist*other_hist)))))
        shape = float(cv2.matchShapes(contour, other_contour, cv2.CONTOURS_MATCH_I1, 0.))
        score = .65*colour/policy.max_colour_distance + .35*shape/policy.max_contour_distance
        passed = bool(iou >= .85 and colour <= policy.max_colour_distance and shape <= policy.max_contour_distance)
        candidates.append(dict(index=index, bbox=list(candidate.bbox_xywh), pixels=candidate.area_px,
            iou=iou, colour_distance=colour, contour_distance=shape, match_score=score,
            original_outline_gates_passed=passed))
        if passed:
            matches.append((score, index))
    matches.sort()
    unique = bool(matches and (len(matches) == 1 or matches[1][0]-matches[0][0] > policy.ambiguity_margin))
    return dict(proposal_reason=regions.reason, candidates=candidates,
        maximum_iou=max((c['iou'] for c in candidates), default=0.),
        matching_candidates=[index for score, index in matches], original_unique_rgb_gate_passed=unique)


def analyze(capture, geometry, output):
    if not MOUNT.is_mount():
        raise RuntimeError('SanStand unavailable; no fallback')
    capture.resolve().relative_to(EXT)
    output.resolve().relative_to(capture.resolve())
    record = json.loads((capture/'result.json').read_text())
    geometry_record = json.loads((geometry/'report.json').read_text())
    if geometry_record['original_result_sha256'] != sha(capture/'result.json'):
        raise ValueError('replay is not bound to this retained capture')
    geom = {row['source_stamp_ns']: row for row in geometry_record['rows']}
    output.mkdir(exist_ok=False)
    cv2.setNumThreads(1)
    frames = []
    reproduction = []
    for entry in record['rows']:
        path = Path(entry['source_file'])
        path.resolve().relative_to(capture.resolve())
        if sha(path) != entry['source_file_sha256']:
            raise ValueError('source hash mismatch')
        with np.load(path, allow_pickle=False) as data:
            rgb, raw = data['color_bgr'], data['depth_raw']
            if array_sha(rgb) != entry['source_rgb_sha256'] or array_sha(raw) != entry['source_depth_sha256']:
                raise ValueError('source RGB-D arrays differ')
        camera = entry['source_context']['camera']
        valid_rgb = rectification_rgb_valid_mask(camera, raw.shape)
        kwargs = {} if valid_rgb is None else {'rgb_valid': valid_rgb}
        depth = raw.astype(np.float32)*entry['camera']['depth_scale']
        valid = np.isfinite(depth) & (depth >= entry['config']['depth_min_m']) & (depth <= entry['config']['depth_max_m'])
        base_policy = CenterSearchPolicy((.5, .5), radius_fraction=entry['config']['center_radius_fraction'])
        regions = propose_rgb_observation_regions(rgb, policy=base_policy, depth_valid=valid, **kwargs)
        found = [(list(c.bbox_xywh), c.area_px) for c in regions.candidates]
        retained = [(c['bbox_xywh'], c['area_px']) for c in entry['rgb_candidates']]
        reproduced = regions.reason == entry['rgb_reason'] and found == retained
        reproduction.append(dict(source_stamp_ns=entry['source_stamp_ns'], original_proposals_reproduced=reproduced,
            candidate_count=len(regions.candidates), reason=regions.reason))
        if not reproduced:
            raise ValueError('independent RGB proposals do not reproduce retained candidates')
        refined_info = geom[entry['source_stamp_ns']].get('replay', {}).get('mask_artifacts', {}).get('refined')
        mask = None
        if refined_info is not None:
            mask_path = geometry/refined_info['path']
            if sha(mask_path) != refined_info['sha256']:
                raise ValueError('refinement artifact hash mismatch')
            mask = _mask(cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE), raw.shape)
        frames.append(dict(entry=entry, rgb=rgb, kwargs=kwargs, policy=base_policy, regions=regions, mask=mask))
    rows = []
    for i, ref in enumerate(frames):
        mask = ref['mask']
        if mask is None:
            continue
        x, y, w, h = cv2.boundingRect(mask)
        hold_policy = replace(ref['policy'], center_xy=((x+.5*w)/ref['rgb'].shape[1], (y+.5*h)/ref['rgb'].shape[0]))
        for j in (i, i+1):
            if j == len(frames):
                continue
            current = frames[j]
            delta = (current['entry']['source_stamp_ns']-ref['entry']['source_stamp_ns'])*1e-9
            if not 0 <= delta <= .8:
                continue
            hold_regions = propose_rgb_observation_regions(current['rgb'], policy=hold_policy, **current['kwargs'])
            rows.append(dict(reference_index=i+1, current_index=j+1,
                reference_stamp_ns=ref['entry']['source_stamp_ns'], current_stamp_ns=current['entry']['source_stamp_ns'],
                source_gap_seconds=delta, comparison='same_frame' if i == j else 'next_saved_frame',
                reference_mask='original offline per-frame full RGB refinement; NOT the unavailable online private cache',
                hold_center_policy=asdict(hold_policy),
                recorder_center=compare(ref['rgb'], mask, current['rgb'], current['regions']),
                production_hold_center=compare(ref['rgb'], mask, current['rgb'], hold_regions)))
    summary = {}
    for kind in ('same_frame', 'next_saved_frame'):
        selected = [r for r in rows if r['comparison'] == kind]
        summary[kind] = dict(comparisons=len(selected),
            hold_passed=sum(r['production_hold_center']['original_unique_rgb_gate_passed'] for r in selected),
            hold_maximum_iou_range=[min(r['production_hold_center']['maximum_iou'] for r in selected),
                                    max(r['production_hold_center']['maximum_iou'] for r in selected)],
            recorder_passed=sum(r['recorder_center']['original_unique_rgb_gate_passed'] for r in selected))
    report = dict(schema=1, capture=str(capture), capture_sha256=sha(capture/'result.json'),
        geometry_report=str(geometry/'report.json'), geometry_report_sha256=sha(geometry/'report.json'),
        script_sha256=sha(__file__), robot_commands_sent=False, motion_authority=False,
        metric_geometry_authority=False, target_identity_established=False,
        original_iou_minimum=.85, original_colour_maximum=.40, original_contour_maximum=.40,
        original_match_ambiguity_margin=.12, original_hold_maximum_seconds=.8,
        limitations=[
            'Exact first-loss RGB-D source 1791172039717751979 is NOT retained; this is adjacent saved-frame feasibility, not replay of that event.',
            'Online private frozen RGB reference is NOT retained. Offline original per-frame refinement masks substitute only for conditional feasibility; they do not prove the runtime cache had these masks.',
            'Only independent RGB identity gates are tested. No support, accepted feedback, freshness at publication, epoch, source context or terminal-state authority is reconstructed or granted.',
            'Last saved frame was recorded after permanent loss; a passing offline gate cannot resurrect generation 5.',
            'Original recorder center proposals are reproduced exactly by reason, ordered bounding boxes and areas; production hold recenters the same policy on the frozen reference mask.'],
        original_proposal_reproduction=reproduction, rows=rows, summary=summary)
    shutil.copy2(__file__, output/Path(__file__).name)
    (output/'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False)+'\n')
    print(json.dumps(dict(report=str(output/'report.json'), sha256=sha(output/'report.json'), summary=summary)))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--geometry', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    analyze(args.capture, args.geometry, args.output)
