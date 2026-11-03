#!/usr/bin/env python3
"""Full pixel rectification counterfactual on archived gen13 sources; no ROS."""
from collections import Counter
from dataclasses import asdict, fields
import hashlib
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np

WORK = Path(__file__).resolve().parent.parent
for directory in (WORK/'.runtime/realsense-2.56.5', WORK/'src/alicia_grasp_modes/src',
                  WORK/'src/alicia_flexible_grasp_supervisor/src'):
    sys.path.insert(0, str(directory))
import pyrealsense2 as rs
from alicia_flexible_grasp.vision import rgbd_rectification as rectification
from alicia_grasp_modes import tabletop, rgb_instance
from alicia_grasp_modes import rgb_observation_handoff as handoff
from alicia_grasp_modes.rgb_observation_region import CenterSearchPolicy, propose_rgb_observation_regions
from alicia_grasp_modes.rgb_observation_projection import projected_rgb_search_policy
from alicia_grasp_modes.rgb_observation_source import digest, measured_support_coverage
from alicia_grasp_modes.rgb_observation_volume import SupportPlaneProvenance, build_rgb_observation_volume

BASE = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log')
ARCHIVE = BASE/'gen13_rgb_support_exact_20261004T122355Z/report.json'
REFERENCE = BASE/'rgb_acquisition_gateway_20261004T114259.948391Z_1541ec9e/result.json'
OUTPUT = ARCHIVE.parent/'rgb_rectified_pixel_counterfactual.json'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def array_sha(array):
    return hashlib.sha256(np.ascontiguousarray(array).tobytes()).hexdigest()


def quantiles(values):
    return dict(zip(('min', 'p10', 'p50', 'p90', 'max'),
        np.percentile(values, [0, 10, 50, 90, 100]).tolist())) if len(values) else None


def apply_pair(mapper, raw):
    rgb, z, provenance = mapper.apply(raw['color_bgr'], raw['depth_raw'], return_source_indices=True)
    positive = z > 0
    source_indices = provenance[positive]
    expected = np.flatnonzero(raw['depth_raw'])
    assert np.array_equal(np.sort(source_indices), expected)
    assert np.array_equal(z[positive], raw['depth_raw'].ravel()[source_indices])
    assert not np.any(provenance[~positive] >= 0)
    return rgb, z, dict(source_positive_pixels=int(len(expected)),
        destination_positive_pixels=int(positive.sum()), every_positive_source_preserved_exactly_once=True,
        source_rgb_sha256=array_sha(raw['color_bgr']), source_z16_sha256=array_sha(raw['depth_raw']),
        rectified_rgb_sha256=array_sha(rgb), rectified_z16_sha256=array_sha(z),
        rectified_shape=list(z.shape), source_depth_values_unchanged=True)


def fit_frame(rgb, z, raw, source, now, feedback, context, source_id, config, K):
    depth = z.astype(np.float32)*float(raw['depth_scale'])
    transform = raw['T_base_camera']
    seg = tabletop.segment(depth, K, transform, config)
    metrics = seg.metrics
    outer = metrics['support_ring_outer_radius_px']
    coverage = measured_support_coverage(depth, K, transform, seg.plane_base, config, outer)
    provenance = SupportPlaneProvenance(source, tuple(seg.plane_base),
        'offline_rectified_pixels:%d'%source,
        int(round(metrics['support_sample_count']*metrics['support_inlier_ratio'])),
        metrics['support_inlier_ratio'], metrics['support_residual_m'], coverage['spatial_quadrants'],
        max_fit_rmse_m=min(.002, config.support_threshold_m),
        min_inlier_ratio=max(.70, config.support_min_ratio), min_inlier_count=max(120, config.support_min_points))
    fx, fy, cx, cy = K
    matrix = np.array([[fx, 0., cx], [0., fy, cy], [0., 0., 1.]])
    frame = handoff.RgbHandoffFrame(rgb, depth, matrix, transform, provenance, source_id,
        context['selection']['generation'], context['epoch_ns'], context['selection']['stamp_ns'],
        source, source, now, tuple((int(ns), tuple(q)) for ns, q in feedback),
        support_ring_outer_radius_px=outer)
    return frame, seg, dict(metrics, **coverage)


def candidate_stats(candidate):
    return dict(bbox_xywh=list(candidate.bbox), quality=candidate.quality,
        valid_points=candidate.metrics.get('valid_points'),
        median_height_m=candidate.metrics.get('median_height_m'),
        mask_pixels=int(np.count_nonzero(candidate.mask)))


def proposal_stats(reference, frame, proposals, projected):
    hull = cv2.convexHull(np.asarray(projected, np.float32)).reshape(-1, 2)
    projected_mask = np.zeros(frame.depth_m.shape, np.uint8)
    cv2.fillConvexPoly(projected_mask, np.rint(hull).astype(np.int32), 1)
    policy = handoff.HandoffPolicy()
    result = []
    for index, candidate in enumerate(proposals.candidates):
        mask = candidate.mask.astype(np.uint8)
        hist, contour = handoff._appearance(frame.bgr, mask)
        colour = float(np.sqrt(max(0., 1.-np.sum(np.sqrt(hist*reference.histogram)))))
        shape = float(cv2.matchShapes(reference.contour, contour, cv2.CONTOURS_MATCH_I1, 0.))
        fraction = float(np.count_nonzero(mask & projected_mask)/np.count_nonzero(mask))
        result.append(dict(index=index, bbox_xywh=list(candidate.bbox_xywh), pixels=int(candidate.area_px),
            mask_sha256=array_sha(mask), colour_distance=colour, contour_distance=shape,
            projected_fraction=fraction,
            appearance_pass=colour <= policy.max_colour_distance and shape <= policy.max_contour_distance,
            projection_pass=fraction >= policy.min_projected_fraction,
            actual_valid_depth_pixels=int(np.count_nonzero(candidate.mask & (frame.depth_m > 0)))))
    return result


def main():
    started = time.monotonic()
    cv2.setNumThreads(1)
    assert rs.__version__ == '2.56.5'
    evidence = json.loads(ARCHIVE.read_text())
    archive = json.loads(REFERENCE.read_text())
    meta = archive['frames']['initial']
    assert sha(meta['source_file']) == meta['source_file_sha256']
    context = meta['source_context']
    config = tabletop.Config(**{f.name:context['perception'][f.name] for f in fields(tabletop.Config)
                               if f.name in context['perception']})
    policy = CenterSearchPolicy(**meta['interest']['rgb_search_policy'])
    mapper = rectification.PairedPinholeRectification(
        context['camera']['runtime_profile_geometry']['geometry_intrinsics'], rs)
    output = mapper.output_intrinsics
    K = [output[key] for key in ('fx', 'fy', 'cx', 'cy')]
    hypothetical_source_id = digest(dict(kind='offline_rectified_pixel_counterfactual',
        recorded_source_id=meta['source_id'], map_contract=mapper.contract))
    report = dict(schema=1, scope=__doc__, ros_io=False, hardware_actions=False,
        motion_authorized=False, geometry_authorized=False, old_reference_reactivated=False,
        source_provenance={str(path):sha(path) for path in (ARCHIVE, REFERENCE)},
        implementation_sha256={str(Path(module.__file__).resolve()):sha(module.__file__)
            for module in (rectification, tabletop, rgb_instance, handoff,
                sys.modules[propose_rgb_observation_regions.__module__])},
        script_sha256=sha(__file__), map_contract=mapper.contract, config=asdict(config),
        base_search_policy=asdict(policy), handoff_policy=asdict(handoff.HandoffPolicy()),
        recorded_original_reference_id=evidence['reference_id'], hypothetical_source_id=hypothetical_source_id,
        limitations=[
            'This constructs a new offline reference from rectified pixels. It neither reuses nor extends the expired original reference or any motion budget.',
            'Only the archived initial source and 88 selected exact frames are available; this is not a replay of every delivered frame or the original three-frame initial RGB admission history.',
            'The saved real camera poses, accepted-feedback brackets and per-source status time are retained. No new TF, joints, camera image, robot motion or timestamped ROS message is generated.',
            'The initial offline evaluation time is derived from its recorded source_age_at_status_sec; it is explicitly not an independently captured clock timestamp.',
            'Sampled-sequence measured handoff is only an offline gate result. It does not demonstrate live planning, execution, successful acquisition or grasp.',
            'Raw files are published aligned/filter-processed Z16. This counterfactual changes only the new central rectification, not upstream matcher/filter settings.',
        ], rows=[])
    with np.load(meta['source_file'], allow_pickle=False) as source:
        raw = {k:source[k] for k in source.files}
    original_K = np.array([context['camera'][key] for key in ('fx', 'fy', 'cx', 'cy')])
    assert np.array_equal(raw['K'], original_K)
    assert int(raw['source_stamp_ns']) == int(meta['source_stamp_ns'])
    rgb, z, preservation = apply_pair(mapper, raw)
    ns = int(meta['source_stamp_ns'])
    recorded_age = meta['interest']['source_age_at_status_sec']
    now = ns+round(recorded_age*1e9)
    initial = dict(source_stamp_ns=ns, source_npz=meta['source_file'], source_sha256=sha(meta['source_file']),
        preservation=preservation, recorded_age_at_status_sec=recorded_age,
        offline_evaluation_ns_derived_from_recorded_age=now,
        new_reference_only_hypothetical=True, original_initial_three_frame_gate_replayed=False)
    report['initial'] = initial
    try:
        frame, seg, support = fit_frame(rgb, z, raw, ns, now,
            (meta['accepted_before'], meta['accepted_after']), context, hypothetical_source_id, config, K)
        proposals = propose_rgb_observation_regions(rgb, policy=policy, rgb_valid=mapper.rgb_valid)
        initial.update(proposal_reason=proposals.reason, support=support,
            measured_candidates=[candidate_stats(c) for c in seg.candidates])
        if proposals.reason != 'observation_region_found' or proposals.selected is None:
            raise ValueError('counterfactual_initial_rgb_not_unique: '+proposals.reason)
        region = proposals.selected
        limits = tuple(meta['volume']['working_depth_range_m'])
        volume = build_rgb_observation_volume(bbox_xywh=region.bbox_xywh, image_shape=z.shape,
            camera_matrix=frame.camera_matrix, T_base_camera=frame.T_base_camera,
            support_plane_base=seg.plane_base, support_provenance=frame.support_provenance,
            foreground_max_m=config.foreground_max_m, source_stamp_ns=ns, working_depth_range_m=limits)
        reference = handoff.freeze_rgb_interest(frame, region, volume,
            support_ring_outer_radius_px=support['support_ring_outer_radius_px'])
        initial.update(new_reference_created=True, region_bbox_xywh=list(region.bbox_xywh),
            region_pixels=int(region.area_px), region_mask_sha256=array_sha(region.mask),
            volume=asdict(volume), reference_mask_fingerprint=reference.mask_fingerprint,
            measured_support_footprint_base=reference.measured_support_footprint_base.tolist(),
            support_consistency_domain_base=reference.support_consistency_domain_base.tolist())
    except (ValueError, AttributeError) as exc:
        initial.update(new_reference_created=False, rejection=str(exc))
        report['elapsed_sec'] = time.monotonic()-started
        OUTPUT.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
        print(json.dumps(dict(output=str(OUTPUT), initial_rejection=str(exc)), ensure_ascii=False), flush=True)
        return
    measured_handoff = handoff.RgbMeasuredHandoff(config)
    for row in sorted(evidence['raw_frames'], key=lambda r:r['source_stamp_ns']):
        source = row['source_stamp_ns']; status = row['status']['message']
        recorded = status['rgb_acquisition_handoff']; observation = recorded['observation_index']
        result = dict(source_stamp_ns=source, source_npz=row['file'], source_sha256=row['sha256'],
            tags=row['tags'], recorded_reason=recorded['reason'], observation_index=observation,
            q_source=row['q_source'], source_age_at_recorded_status_sec=(
                status['frame_timing']['status_build_ros_ns']-source)*1e-9)
        report['rows'].append(result)
        try:
            assert sha(row['file']) == row['sha256']
            with np.load(row['file'], allow_pickle=False) as entry:
                raw = {k:entry[k] for k in entry.files}
            assert int(raw['source_stamp_ns']) == source
            assert np.array_equal(raw['K'], original_K)
            assert np.array_equal(raw['q_source'], row['q_source'])
            rgb, z, preservation = apply_pair(mapper, raw)
            result['preservation'] = preservation
            current, seg, support = fit_frame(rgb, z, raw, source,
                status['frame_timing']['status_build_ros_ns'], (row['accepted_before'], row['accepted_after']),
                context, hypothetical_source_id, config, K)
            result.update(support=support, measured_candidates=[candidate_stats(c) for c in seg.candidates])
            audit = handoff.associate_rgb_interest(reference, current, (), observation_index=observation)
            result['support_audit'] = dict(audit.support_audit)
            if audit.reason != 'no_cross_view_rgb_match':
                result['replayed_reason'] = audit.reason
                measured_handoff.reset()
                continue
            search = projected_rgb_search_policy(policy, np.asarray(audit.projected_volume_pixels), z.shape)
            proposals = propose_rgb_observation_regions(rgb, policy=search, rgb_valid=mapper.rgb_valid,
                depth_valid=np.isfinite(current.depth_m)&(current.depth_m >= config.depth_min_m)&(current.depth_m <= config.depth_max_m))
            association = handoff.associate_rgb_interest(reference, current, proposals, seg.candidates,
                                                         observation_index=observation)
            result.update(replayed_reason=association.reason, associated=association.associated,
                proposal_reason=proposals.reason, search_policy=asdict(search),
                projected_volume_pixels=association.projected_volume_pixels,
                proposals=proposal_stats(reference, current, proposals, association.projected_volume_pixels),
                measured_candidate_index=association.measured_candidate_index,
                associated_region_index=association.rgb_region_index)
            if association.associated:
                mask = association.region.mask.astype(bool)
                valid = mask & np.isfinite(current.depth_m) & (current.depth_m >= config.depth_min_m) & (current.depth_m <= config.depth_max_m)
                points = tabletop._project(current.depth_m, K)
                plane_camera = current.T_base_camera.T @ seg.plane_base
                height = np.einsum('ijk,k->ij', points, plane_camera[:3])+plane_camera[3]
                foreground = valid & (height >= config.foreground_min_m) & (height <= config.foreground_max_m)
                candidate_iou = [float(np.count_nonzero(mask & (c.mask > 0))/np.count_nonzero(mask | (c.mask > 0))) for c in seg.candidates]
                result['associated_rgb_measurements'] = dict(pixels=int(mask.sum()), valid_depth_pixels=int(valid.sum()),
                    valid_depth_fraction=float(valid.sum()/mask.sum()), raw_foreground_4_to_120mm=int(foreground.sum()),
                    measured_optical_z_mm=quantiles(current.depth_m[valid]*1000),
                    height_to_current_support_mm=quantiles(height[valid]*1000),
                    candidate_mask_ious=candidate_iou, maximum_candidate_mask_iou=max(candidate_iou, default=0.))
            raw_candidate, refined = association.measured_candidate, None
            if raw_candidate is not None and raw_candidate.quality >= context['perception'].get('minimum_quality', .5):
                try:
                    refined = rgb_instance.refine_candidate(rgb, current.depth_m, K, current.T_base_camera,
                        raw_candidate, config, rgb_valid=mapper.rgb_valid)
                    result['refined_candidate'] = candidate_stats(refined)
                except ValueError as exc:
                    result['refinement_rejection'] = str(exc)
            verdict = measured_handoff.observe(association, raw_candidate, refined)
            result.update(sampled_sequence_handoff_ready=verdict.ready, measured_handoff_reason=verdict.reason,
                          confirming_stamps_ns=list(verdict.confirming_stamps_ns))
        except (ValueError, KeyError, AttributeError) as exc:
            result['replayed_reason'] = 'source_or_segmentation_rejected: '+str(exc)
            measured_handoff.reset()
        if len(report['rows']) % 10 == 0:
            print(json.dumps(dict(processed=len(report['rows']), elapsed_sec=time.monotonic()-started)), flush=True)
    report['summary'] = dict(frame_count=len(report['rows']),
        recorded_reasons=dict(Counter(r['recorded_reason'] for r in report['rows'])),
        replayed_reasons=dict(Counter(r['replayed_reason'] for r in report['rows'])),
        measured_handoff_reasons=dict(Counter(r.get('measured_handoff_reason', 'not_reached') for r in report['rows'])),
        all_positive_z16_preserved=all(r['preservation']['every_positive_source_preserved_exactly_once'] for r in report['rows']),
        associated_frames=sum(bool(r.get('associated')) for r in report['rows']),
        sampled_sequence_measured_ready=sum(bool(r.get('sampled_sequence_handoff_ready')) for r in report['rows']),
        minimum_associated_projected_fraction=min((r['proposals'][r['associated_region_index']]['projected_fraction']
            for r in report['rows'] if r.get('associated')), default=None),
        associated_valid_depth_fraction=quantiles(np.array([r['associated_rgb_measurements']['valid_depth_fraction']
            for r in report['rows'] if r.get('associated')])),
        associated_raw_foreground_pixel_count=quantiles(np.array([r['associated_rgb_measurements']['raw_foreground_4_to_120mm']
            for r in report['rows'] if r.get('associated')])),
        maximum_actual_candidate_rgb_mask_iou=max((r['associated_rgb_measurements']['maximum_candidate_mask_iou']
            for r in report['rows'] if r.get('associated')), default=None),
        per_recorded_reason={reason:dict(Counter(r['replayed_reason'] for r in report['rows'] if r['recorded_reason']==reason))
            for reason in sorted(set(r['recorded_reason'] for r in report['rows']))})
    report['elapsed_sec'] = time.monotonic()-started
    OUTPUT.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(OUTPUT), sha256=sha(OUTPUT), summary=report['summary']), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
