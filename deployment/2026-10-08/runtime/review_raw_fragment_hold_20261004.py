"""Hash-verified archived RGB-D through the actual producer; no live ROS I/O.

Each start offset is an independent hypothetical offline initialization. No
recorded/live lost flag is cleared, no intervening frames are fabricated, and
no diagnostic result grants robot authority.
"""
from collections import Counter
from copy import deepcopy
from dataclasses import replace
import argparse
import hashlib
import json
from pathlib import Path
import sys
from unittest.mock import Mock

import cv2
import numpy as np
from _pytest.monkeypatch import MonkeyPatch

WORK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORK / 'src/alicia_grasp_modes/tests'))
from test_unknown_stream import module, rgb_interest_node


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stamp(ns):
    return module.rospy.Time(int(ns) // 10**9, int(ns) % 10**9)


def load_archive(source):
    rows = json.loads(source.read_text())['rows']
    frames = []
    for row in rows:
        path = Path(row['source_file'])
        assert sha(path) == row['source_file_sha256']
        with np.load(path, allow_pickle=False) as raw:
            rgb, depth, transform = (raw[key].copy() for key in
                                     ('color_bgr', 'depth_raw', 'T_base_camera'))
            assert np.array_equal(raw['K'], [row['camera'][key] for key in ('fx', 'fy', 'cx', 'cy')])
        assert hashlib.sha256(rgb.tobytes()).hexdigest() == row['source_rgb_sha256']
        assert hashlib.sha256(depth.tobytes()).hexdigest() == row['source_depth_sha256']
        assert np.array_equal(transform, row['T_base_camera'])
        assert row['source_context'] == rows[0]['source_context']
        frames.append((rgb, depth, transform))
    return rows, frames


def replay(rows, frames, start, multi=True, stop_at_loss=True):
    patch = MonkeyPatch()
    node, clock, _feed, _joint, parameters = rgb_interest_node(patch)
    context = rows[start]['source_context']
    node.config = module.Config(**rows[start]['config'])
    node.tracker = module.TargetTracker(node.config)
    node.acquisition = module.InitialAcquisition(node.config)
    node.generation, node.selection_ns = context['selection']['generation'], context['selection']['stamp_ns']
    node._rgb_interest_epoch = context['epoch_ns']
    node._rgb_interest_config = deepcopy(context['perception'])
    parameters['/camera'] = deepcopy(context['camera'])
    if 'runtime_profile_geometry' in parameters['/camera']:
        parameters['/camera']['runtime_profile'] = parameters['/camera'].pop('runtime_profile_geometry')
    assert module.camera_geometry_snapshot(parameters['/camera']) == context['camera']
    parameters['/grasp_mode/selection'] = deepcopy(context['selection'])
    parameters['/handeye'] = deepcopy(context['handeye'])
    parameters['/unknown_perception'] = deepcopy(context['perception'])
    node._rgb_interest_source_context = lambda _stamp: deepcopy(context)
    now, transform = [0], [None]
    patch.setattr(module.rospy.Time, 'now', staticmethod(lambda: stamp(now[0])))
    patch.setattr(module.rospy, 'logwarn', Mock())
    patch.setattr(module, 'transform_matrix', lambda args: transform[0].copy()
                  if args[0] == 'base_link' else np.asarray(context['tool_camera']).copy())
    original_hold = node._same_station_zero_candidate_rgb_hold
    if not multi:
        node._same_station_zero_candidate_rgb_hold = lambda source, result, camera: (
            False if len(result.candidates) > 1 else original_hold(source, result, camera))
    current_segmentation = []
    original_segment = module.segment_with_support_fallback

    def observe_segment(*args, **kwargs):
        result = original_segment(*args, **kwargs)
        current_segmentation[:] = [result]
        return result

    patch.setattr(module, 'segment_with_support_fallback', observe_segment)
    replayed = []
    try:
        for index in range(start, len(rows)):
            row = rows[index]
            rgb, raw, transform[0] = frames[index]
            ns = row['source_stamp_ns']
            now[0] = ns + int(round(row['source_age_at_end_sec'] * 10**9))
            clock[0] = 10. + (now[0] - rows[start]['source_stamp_ns']) * 1e-9
            node._rgb_interest_feedback.clear()
            node._rgb_interest_feedback.extend((int(pair[0]), np.asarray(pair[1]))
                for pair in (row['accepted_before'], row['accepted_after']))
            messages = (node.bridge.cv2_to_imgmsg(rgb, encoding='bgr8'),
                        node.bridge.cv2_to_imgmsg(raw, encoding='16UC1' if raw.dtype == np.uint16 else '32FC1'))
            for message in messages:
                message.header.stamp, message.header.frame_id = stamp(ns), node.camera_frame
            anchor, last = node.tracker.anchor, node.tracker.last
            reference = node._locked_rgb_hold_reference
            previous_good = node.last_good_source_stamp_ns
            current_segmentation.clear()
            node.latest = messages
            node.process(None)
            status = json.loads(node.pubs['detector_status'].publish.call_args[0][0].data)
            detected = bool(node.pubs['object'].publish.call_args[0][0].detected)
            candidates = []
            result = current_segmentation[0] if current_segmentation else None
            for candidate in result.candidates if result is not None else ():
                data = dict(bbox=list(candidate.bbox), extent_m=candidate.extent_m.tolist(),
                            measured_points=int(candidate.metrics['valid_points']),
                            mask_sha256=hashlib.sha256(candidate.mask.tobytes()).hexdigest())
                if anchor is not None:
                    data['ratios_xy'] = (candidate.extent_m[:2] / np.maximum(anchor.extent_m[:2], .003)).tolist()
                    data['anchor_distance_m'] = float(np.linalg.norm(candidate.position_base - anchor.position_base))
                    data['last_distance_m'] = float(np.linalg.norm(candidate.position_base - last.position_base))
                if reference is not None:
                    data['pixels_outside_last_measured_rgb'] = int(np.count_nonzero(
                        (candidate.mask != 0) & (reference['mask'] == 0)))
                candidates.append(data)
            rgb_boundary_audit = None
            if start == 0 and index == 9 and reference is not None:
                x, y, width, height = cv2.boundingRect(reference['mask'])
                policy = replace(node._rgb_interest_policy,
                    center_xy=((x + .5*width)/rgb.shape[1], (y + .5*height)/rgb.shape[0]))
                proposals = module.propose_rgb_observation_regions(rgb, policy=policy)
                gates = module.HandoffPolicy()
                matches = []
                for proposal_index, region in enumerate(proposals.candidates):
                    current_mask = module._mask(region.mask, rgb.shape[:2])
                    iou = np.count_nonzero(current_mask & reference['mask']) / np.count_nonzero(current_mask | reference['mask'])
                    histogram, contour = module._appearance(rgb, current_mask)
                    color_distance = float(np.sqrt(max(0., 1. - np.sum(np.sqrt(histogram * reference['histogram'])))))
                    shape_distance = float(cv2.matchShapes(reference['contour'], contour, cv2.CONTOURS_MATCH_I1, 0.))
                    accepted = iou >= .85 and color_distance <= gates.max_colour_distance and shape_distance <= gates.max_contour_distance
                    matches.append(dict(index=proposal_index, bbox=list(region.bbox_xywh),
                        area_px=int(np.count_nonzero(current_mask)), iou=float(iou),
                        color_distance=color_distance, contour_distance=shape_distance,
                        original_match_gates_pass=bool(accepted),
                        candidate_pixels_outside_current_rgb=[int(np.count_nonzero(
                            (candidate.mask != 0) & (current_mask == 0))) for candidate in result.candidates]))
                rgb_boundary_audit = dict(reference_pixels=int(np.count_nonzero(reference['mask'])),
                    reference_bbox=list(cv2.boundingRect(reference['mask'])),
                    reference_source_ns=reference['source_stamp_ns'],
                    current_proposal_reason=proposals.reason,
                    max_colour_distance=gates.max_colour_distance,
                    max_contour_distance=gates.max_contour_distance,
                    ambiguity_margin=gates.ambiguity_margin, proposals=matches)
            replayed.append(dict(index=index, source_stamp_ns=ns,
                source_file_sha256=row['source_file_sha256'], source_rgb_sha256=row['source_rgb_sha256'],
                source_depth_sha256=row['source_depth_sha256'], recorded_source_age_sec=row['source_age_at_end_sec'],
                detail=status['detail'], detected=detected, target_lost=node.tracker.lost,
                target_locked=node.tracker.anchor is not None,
                last_good_before_ns=previous_good, last_good_after_ns=node.last_good_source_stamp_ns,
                rgb_hold=status.get('rgb_identity_hold'), candidates=candidates,
                rgb_boundary_audit=rgb_boundary_audit,
                current_support_metrics=result.metrics if result is not None else None,
                weak_observation_ready=status['observation_interest']['observation_interest_ready']))
            if stop_at_loss and node.tracker.lost:
                break
    finally:
        patch.undo()
    triplets = []
    for before, current, after in zip(replayed, replayed[1:], replayed[2:]):
        hold = current.get('rgb_hold') or {}
        if before['detected'] and hold.get('state') == 'holding_identity_only' and after['detected']:
            triplets.append(dict(indices=[before['index'], current['index'], after['index']],
                fragment_count=hold.get('fragment_count'), kind=hold.get('depth_gap_kind'),
                stamps_ns=[item['source_stamp_ns'] for item in (before, current, after)]))
    return dict(independent_offline_start_index=start, multi_fragment_enabled=multi,
                rows=replayed, adjacent_recorded_good_hold_good=triplets)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--primary-only', action='store_true',
                        help='Replay only the complete archived sequence from its first frame')
    args = parser.parse_args()
    cv2.setNumThreads(1)
    rows, frames = load_archive(args.source)
    runs = [replay(rows, frames, 0)]
    evaluated_starts = [0]
    # Independent offset runs diagnose available recorded evidence; they are
    # not recovery of either the live lost tracker or another replay's loss.
    for start in (() if args.primary_only else range(1, len(rows) - 2)):
        candidate_run = replay(rows, frames, start)
        evaluated_starts.append(start)
        if any((row.get('rgb_hold') or {}).get('fragment_count', 0) >= 2
               and row['rgb_hold'].get('state') == 'holding_identity_only'
               for row in candidate_run['rows']):
            runs.append(candidate_run)
    comparison_starts = {run['independent_offline_start_index'] for run in runs}
    baseline = [replay(rows, frames, start, multi=False) for start in sorted(comparison_starts)]
    summary = dict(archived_frames=len(rows), recorded_generation=rows[0]['selection']['generation'],
        evaluated_offline_start_indices=evaluated_starts,
        primary_details=dict(Counter(row['detail'] for row in runs[0]['rows'])),
        retained_start_indices=sorted(comparison_starts),
        multi_hold_rows=[dict(start=run['independent_offline_start_index'], index=row['index'],
            stamp_ns=row['source_stamp_ns'], fragments=row['rgb_hold']['fragment_count'])
            for run in runs for row in run['rows']
            if (row.get('rgb_hold') or {}).get('fragment_count', 0) >= 2
            and row['rgb_hold'].get('state') == 'holding_identity_only'],
        adjacent_recorded_good_hold_good=[dict(start=run['independent_offline_start_index'], **triplet)
            for run in runs for triplet in run['adjacent_recorded_good_hold_good']])
    report = dict(source=str(args.source.resolve()), source_sha256=sha(args.source),
        script_sha256=sha(__file__), producer_sha256=sha(module.__file__),
        raw_files_and_rgb_depth_hashes_verified=True,
        ros_io=False, live_publications=False, robot_commands_sent=False,
        live_selection_or_lost_modified=False, summary=summary,
        interpretation=['This is a later capture, not generation 9 exact first-loss RGB-D.',
            'Every offset is a fresh OFFLINE hypothetical state; none resumes a lost tracker.',
            'Adjacent recorded samples omit intervening live frames and complete joint feedback history.',
            'Archived end-of-processing source age and clocks are mocked; this is not live latency validation.',
            'All segmentation/refinement/tracking/hold/published detection paths are production code.'],
        multi_runs=runs, single_candidate_baseline_runs=baseline)
    if args.output is not None:
        output = args.output.resolve()
        mount = Path('/media/zhuyupei/SanStand')
        if not mount.is_mount() or mount / '实验记录/Robot-Controller' not in output.parents:
            raise RuntimeError('SanStand output required; no fallback')
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
