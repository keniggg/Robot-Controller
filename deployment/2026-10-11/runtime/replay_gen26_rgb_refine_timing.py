#!/usr/bin/env python3
"""Offline exact-pixel stage timing; no ROS, runtime mutation, or authority."""
import argparse
import hashlib
import json
import time
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
from alicia_grasp_modes import tabletop, rgb_instance
from alicia_grasp_modes.rgb_observation_handoff import _appearance
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask

parser = argparse.ArgumentParser()
parser.add_argument('--input', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--repeat', type=int, default=5)
args = parser.parse_args()
assert args.repeat >= 1
assert not (args.output / 'result.json').exists()
args.output.mkdir(parents=True, exist_ok=True)
meta = json.loads(args.input.read_text())
cv2.setNumThreads(1)
paths = [Path(tabletop.__file__), Path(rgb_instance.__file__), Path(__file__)]
source_before = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
stage_rows = []

def clocks():
    return time.perf_counter(), time.thread_time(), time.process_time()

def record_stage(name, before):
    after = clocks()
    stage_rows.append(dict(stage=name, wall_sec=after[0]-before[0],
        thread_cpu_sec=after[1]-before[1], process_cpu_sec=after[2]-before[2]))

def timed(name, function):
    def wrapped(*a, **kw):
        started = clocks()
        try:
            return function(*a, **kw)
        finally:
            record_stage(name, started)
    return wrapped

# Diagnostic wrappers invoke the original functions with unchanged arguments.
# Their nested timings overlap and must not be summed.
for name in ('_project', '_observation_spatial_support', 'confidence_labels',
             '_observed_spatial_search_bounds', 'instance_mask_from_seed'):
    setattr(rgb_instance, name, timed(name, getattr(rgb_instance, name)))
cv2.grabCut = timed('opencv_grabcut', cv2.grabCut)
original_lock = rgb_instance._grabcut_lock

class TimedLock:
    def __enter__(self):
        started = clocks()
        original_lock.acquire()
        record_stage('grabcut_lock_acquire', started)
        return self
    def __exit__(self, *unused):
        original_lock.release()

rgb_instance._grabcut_lock = TimedLock()
status = meta['sources'][0]['detector_status']['message']
anchor = SimpleNamespace(position_base=np.array(status['anchor_position_base']),
    extent_m=np.array(status['anchor_extent_m']),
    support_footprint_base=np.array(status['reference_support_footprint_base']),
    metrics={'median_height_m': status['reference_object_height_m']})
plane = np.array(status['reference_support_plane_base'])
spatial = rgb_instance.freeze_observation_spatial_reference(anchor, plane)
config = tabletop.Config()
last = anchor
rows = []
for source in meta['sources']:
    images = np.load(source['npz'])
    camera = source['camera_metadata']['message']['parameters']
    K = [camera[k] for k in ('fx', 'fy', 'cx', 'cy')]
    depth = images['depth_raw'].astype(np.float32) * camera['depth_scale']
    T = images['T_base_camera']
    valid = rectification_rgb_valid_mask(camera, depth.shape)
    stage_rows.clear()
    segment = timed('segment_with_support_fallback', tabletop.segment_with_support_fallback)(
        depth, K, T, config, anchor, last, plane, prefer_current_support=True)
    matches = tabletop._matching_candidates(segment.candidates, anchor, last, config)
    assert len(matches) == 1, len(matches)
    raw = matches[0][1]
    segmentation_timing = list(stage_rows)
    runs = []
    for repeat in range(args.repeat):
        stage_rows.clear()
        refined = timed('refine_candidate_total', rgb_instance.refine_candidate)(
            images['rgb'], depth, K, T, raw, config, rgb_valid=valid,
            observation_arrival_retry=True, observation_spatial_reference=spatial)
        timed('appearance_only_after_refine', _appearance)(images['rgb'], refined.mask)
        runs.append(dict(repeat=repeat, stages=list(stage_rows),
            mask_pixels=int(np.count_nonzero(refined.mask)),
            mask_sha256=hashlib.sha256(refined.mask.tobytes()).hexdigest()))
    assert len({r['mask_sha256'] for r in runs}) == 1
    equals_saved = bool(np.array_equal(refined.mask, images['refined_mask']))
    if source['object_message']['detected']:
        assert equals_saved
    rows.append(dict(source_stamp_ns=source['source_stamp_ns'], role=source['role'],
        native_candidate_count=len(segment.candidates), unique_match_count=len(matches),
        raw_mask_pixels=int(np.count_nonzero(raw.mask)), raw_bbox=raw.bbox,
        published_detected=source['object_message']['detected'],
        published_mask_bitexact=equals_saved if source['object_message']['detected'] else None,
        segmentation_timing=segmentation_timing, runs=runs))
    last = raw

source_after = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
assert source_after == source_before
report = dict(scope='OFFLINE_EXACT_PIXEL_COMPUTATION_TIMING_NO_ROS',
    input_manifest=str(args.input), input_manifest_sha256=hashlib.sha256(args.input.read_bytes()).hexdigest(),
    opencv_threads=cv2.getNumThreads(), repeat=args.repeat, rows=rows,
    source_before=source_before, source_after=source_after,
    limitations=[
        'Original current RGB-D/TF/anchor geometry from sealed recording. Anchor is reconstructed from status fields, not replayed from its acquisition frame.',
        'Both original raw matching and same-frame refinement execute. Last-good output equals the real published mask bit for bit.',
        'Failed-frame published mask was empty after freshness rejection, so its offline refined mask cannot be compared to an unrecorded internal result.',
        'Original default Config matches deployed defaults; this does not replay live ROS callbacks, received timing, TF waits, accepted-feedback waits, or reference locking.',
        'Appearance alone is timed after refinement; the full _prepare_locked_rgb_hold_reference is not invoked.',
        'Scoped wrappers preserve function inputs/results but add timing overhead. Nested rows overlap; summing them double-counts work.',
        'Offline single-process measurement cannot identify the historical GIL, scheduling, TF, or lock state and conveys no execution authority.'])
(args.output/'result.json').write_text(json.dumps(report, indent=2)+'\n')
for row in rows:
    print(json.dumps(dict(source_stamp_ns=row['source_stamp_ns'],
        published_mask_bitexact=row['published_mask_bitexact'],
        stages={name: [round(next(s['wall_sec'] for s in r['stages'] if s['stage']==name),6)
            for r in row['runs']] for name in ('refine_candidate_total','opencv_grabcut',
                'grabcut_lock_acquire','appearance_only_after_refine')})))
print(args.output/'result.json')
