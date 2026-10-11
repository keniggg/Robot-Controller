"""Recorded RGBD -> production segmentation/refinement/arrival/ORB/dense path.

No injected homography or successful appearance result. This is sparse offline
geometry/image replay, not reproduction of ROS timing, ownership or motion.
"""
from copy import deepcopy
from dataclasses import fields
import hashlib
import json
from pathlib import Path
import time
import sys

import cv2
import numpy as np
from alicia_grasp_modes import (
    tabletop, rgb_instance, measured_mask_seeds, observation_arrival_confirmation as arrival,
    observation_appearance, arrival_fragment_group, arrival_dense_identity, arrival_orb_proposal)
from alicia_grasp_modes.rgb_observation_handoff import _appearance

BASE = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z')
OUT = BASE/'gen22_analysis/integrated_appearance_replay'
assert Path('/media/zhuyupei/SanStand').is_mount()
OUT.mkdir(exist_ok=True)
MODULES = [tabletop, rgb_instance, measured_mask_seeds, arrival, observation_appearance,
           arrival_fragment_group, arrival_dense_identity, arrival_orb_proposal]
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
before = {m.__file__: sha(m.__file__) for m in MODULES}


def arrays(frame):
    K = frame['K']
    if K.shape == (3, 3):
        K = [K[0, 0], K[1, 1], K[0, 2], K[1, 2]]
    return frame['depth_raw'].astype(np.float32)*float(frame['depth_scale']), K, frame['T_base_camera']


def reconstruct_gen22_final_raw(frame, status, cfg):
    # Exactly the recorded group-final current-depth calculation, with the
    # measured recorded RGB mask. It is checked against independent READY
    # position/extent/point count; no arbitrary fragment selection is supplied.
    assert status['geometry_metrics']['observation_fragment_group']['final_original_geometry_passed']
    depth, K, T = arrays(frame)
    points = tabletop._project(depth, K)
    plane = np.asarray(status['geometry_metrics']['support_plane_base'])
    camera_plane = T.T @ plane
    height = points @ camera_plane[:3] + camera_plane[3]
    measured = ((frame['recorded_refined_mask'] > 0) & np.isfinite(depth)
                & (depth >= cfg.depth_min_m) & (depth <= cfg.depth_max_m)
                & (height >= cfg.foreground_min_m) & (height <= cfg.foreground_max_m))
    xyz = points[measured].astype(float)
    p_cam = np.median(xyz, axis=0)
    p_base = (T @ np.r_[p_cam, 1.])[:3]
    centered = xyz-np.mean(xyz, axis=0)
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    extent = np.sort(np.percentile(centered @ vh.T, 98, axis=0)-np.percentile(centered @ vh.T, 2, axis=0))[::-1]
    yy, xx = np.nonzero(measured)
    raw = tabletop.Candidate(measured.astype(np.uint8)*255, np.array([xx.mean(), yy.mean()]),
        (int(xx.min()), int(yy.min()), int(xx.max()-xx.min()+1), int(yy.max()-yy.min()+1)),
        p_cam, p_base, extent, float(p_cam[0]), float(status['geometry_quality']), deepcopy(status['geometry_metrics']),
        tabletop._measured_support_footprint(xyz @ T[:3, :3].T+T[:3, 3], plane))
    position_error = float(np.max(abs(p_base-np.asarray(status['position_base']))))
    extent_error = float(np.max(abs(extent-np.asarray(status['geometry_metrics']['observed_surface_extent_m']))))
    assert position_error < 1e-10 and extent_error < 1e-10
    assert len(xyz) == status['geometry_metrics']['observation_fragment_group']['final_measured_points']
    return raw, dict(position_error=position_error, extent_error=extent_error, measured_points=len(xyz))


reports = []
for generation in (21, 22):
    inp = BASE/('gen%d_analysis' % generation)/'replay_inputs'
    manifest = json.loads((inp/'manifest.json').read_text())
    rows = manifest['files']
    params = json.loads((inp/'unknown_perception_configuration.json').read_text())['parameters']
    cfg = tabletop.Config(**{key: value for key, value in params.items() if key in {f.name for f in fields(tabletop.Config)}})
    tracker = tabletop.TargetTracker(cfg)
    ref_rows = []
    preparation = {}
    if generation == 21:
        for role in ('anchor', 'previous_ready_reference_probe', 'last_pre_motion'):
            row = next(row for row in rows if row['role'] == role)
            frame = dict(np.load(row['path']))
            kwargs = {} if tracker.anchor is None else dict(anchor=tracker.anchor, last=tracker.last,
                reference_plane_base=tracker.plane_base, prefer_current_support=True)
            segmented = tabletop.segment_with_support_fallback(*arrays(frame), cfg, **kwargs)
            raw, reason = tracker.choose(segmented, frame['depth_raw'].shape)
            assert raw is not None
            refined = rgb_instance.refine_candidate(frame['color'], *arrays(frame), raw, cfg, rgb_valid=frame['rgb_valid'])
            assert refined.quality >= .5
            ref_rows.append(dict(role=role, source_ns=row['source_stamp_ns'],
                current_refined_mask_sha256=hashlib.sha256(refined.mask.tobytes()).hexdigest(),
                current_refined_mask_pixels=int(np.count_nonzero(refined.mask)), metrics=refined.metrics))
        ref_mask = refined.mask
        ref_frame, ref_source = frame, row['source_stamp_ns']
        preparation['basis'] = 'GEN21 current production seed fix regenerates reference from actual sealed RGBD; not the historical pre-fix mask.'
    else:
        anchor_row = next(row for row in rows if row['role'] == 'anchor')
        anchor_frame = dict(np.load(anchor_row['path']))
        segmented = tabletop.segment_with_support_fallback(*arrays(anchor_frame), cfg)
        raw, reason = tracker.choose(segmented, anchor_frame['depth_raw'].shape)
        status = json.loads((inp/'anchor_status.json').read_text())['message']
        assert np.max(abs(raw.position_base-np.array(status['anchor_position_base']))) < 1e-10
        assert np.max(abs(tracker.plane_base-np.array(status['reference_support_plane_base']))) < 1e-10
        last_row = next(row for row in rows if row['role'] == 'last_pre_motion')
        ref_frame = dict(np.load(last_row['path']))
        last_status = json.loads((inp/'last_pre_motion_status.json').read_text())['message']
        tracker.last, preparation = reconstruct_gen22_final_raw(ref_frame, last_status, cfg)
        preparation['basis'] = 'Last first-view READY before second executing transition; exact recorded refined mask and current measured depth, not initial RGB template.'
        ref_mask, ref_source = ref_frame['recorded_refined_mask'], last_row['source_stamp_ns']
    histogram, contour = _appearance(ref_frame['color'], ref_mask)
    snapshot = observation_appearance.FrozenObservationAppearance(ref_frame['color'], ref_mask,
        ref_frame['rgb_valid'], ref_source, histogram, contour)
    helper = arrival.ObservationArrivalConfirmation(cfg, tracker.anchor, tracker.last, tracker.plane_base,
        histogram, contour, .5, appearance_reference=snapshot)
    observations = []
    frozen_reference_hash = hashlib.sha256(snapshot.bgr.tobytes()+snapshot.mask.tobytes()).hexdigest()
    for row in rows:
        if row['role'] not in ('arrival', 'first_one_of_three', 'recorded_arrival_not_online_evaluated'):
            continue
        frame = dict(np.load(row['path']))
        source = row['source_stamp_ns']
        started = time.monotonic()
        deadline = started+.8
        checks = [0]
        def verify_record_binding():
            checks[0] += 1
            assert source > ref_source
            assert hashlib.sha256(snapshot.bgr.tobytes()+snapshot.mask.tobytes()).hexdigest() == frozen_reference_hash
            assert manifest['selection_stamp_ns'] > 0 and manifest['control_epoch_ns'] > 0
        evidence = {}
        try:
            segmented = tabletop.segment_with_support_fallback(*arrays(frame), cfg, anchor=tracker.anchor,
                last=tracker.last, reference_plane_base=tracker.plane_base, prefer_current_support=True)
            confirmed, candidate, staged, evidence = helper.observe(segmented, frame['color'], *arrays(frame), source,
                rgb_valid=frame['rgb_valid'], appearance_deadline_monotonic=deadline,
                appearance_check_current=verify_record_binding)
            accepted, reason = True, evidence['reason']
        except ValueError as error:
            accepted, confirmed, reason = False, False, str(error)
        observations.append(dict(source_ns=source, online_evaluated=row['role'] != 'recorded_arrival_not_online_evaluated',
            identity_passed=accepted, confirmed=confirmed, reason=reason, evidence=evidence,
            appearance=deepcopy(helper.last_appearance_evidence), record_binding_rechecks=checks[0],
            wall_sec=time.monotonic()-started, original_three_frame_sources=list(helper.source_stamps_ns)))
    reports.append(dict(generation=generation, record=manifest['record'], manifest_sha256=sha(inp/'manifest.json'),
        reference_source_ns=ref_source, reference_mask_sha256=snapshot.mask_sha256, reference_history=ref_rows,
        reference_preparation=preparation, observations=observations,
        identity_passed=sum(row['identity_passed'] for row in observations),
        earliest_three_frame_confirmation=next((row['source_ns'] for row in observations if row['confirmed']), None)))

after = {m.__file__: sha(m.__file__) for m in MODULES}
report = dict(source_sha256=before, source_after_sha256=after, source_unchanged=before == after,
    reports=reports, script_sha256=sha(__file__), no_ros_calls=True, injected_homography=False,
    original_seed_fix_preserved=True, no_production_source_modifications=True,
    limitations=[
        'GEN22 eleven frames are real adjacent saved frames, not the twelve exact online failed frames.',
        'GEN21 uses the current production reference refinement to preserve its seed fix, not the old contaminated recorded template.',
        'Current production arrival/refinement/frozen appearance/ORB/dense code is called without positive-result injection.',
        'Offline callbacks verify sealed input binding only; they do not authenticate a live epoch/owner or exercise ROS transport.',
        'Each frame gets a fixed .8 second processing allowance from offline processing start; historical frame arrival age, scheduling, capture-window deadline and full-load timing are not replayed.',
        'Existing three-consecutive-frame logic is retained; sparse offline confirmations do not claim a live ACK or physical grasp.',
    ])
def default(value):
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, np.generic): return value.item()
    raise TypeError(type(value).__name__)
path = OUT/((sys.argv[1] if len(sys.argv)>1 else 'production_entry_r3')+'.json')
path.write_text(json.dumps(report, indent=2, default=default, allow_nan=False)+'\n')
print(json.dumps(dict(path=str(path), sha256=sha(path), source_unchanged=before == after,
    reports=[dict(generation=r['generation'], passed=r['identity_passed'], count=len(r['observations']),
                  first_confirmation=r['earliest_three_frame_confirmation']) for r in reports])))
