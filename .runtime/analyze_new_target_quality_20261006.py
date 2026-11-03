#!/usr/bin/env python3
"""Bounded offline RGB-D diagnostics; no ROS, device, motion or parameter I/O.

Use each captured frame's original tabletop Config. Optional pipeline JSON
supplies effective /grasp_6d/remote constructor parameters for single-frame,
pre-snapshot geometry. RGB-only attention is reported separately from RGB-D.
"""
import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import cv2
import numpy as np

WORK = Path(__file__).resolve().parents[1]
for relative in ('src/alicia_grasp_modes/src', 'src/alicia_flexible_grasp_supervisor/src'):
    sys.path.insert(0, str(WORK / relative))
from alicia_grasp_modes import tabletop, rgb_instance, rgb_observation_region
from alicia_grasp_modes.rgb_observation_source import digest, bracket_feedback
from alicia_flexible_grasp.vision.rgbd_rectification import (
    rectification_rgb_valid_mask, require_rectified_source_after_activation)

MOUNT = Path('/media/zhuyupei/SanStand')
EXTERNAL = MOUNT / '实验记录/Robot-Controller'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def array_sha(value):
    return hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest()


def limits(values):
    values = [float(value) for value in values if value is not None]
    return None if not values else dict(min=min(values), median=float(np.median(values)), max=max(values))


def source_binding(row, arrays, path):
    camera = row['camera']; context = row['source_context']; ns = row['source_stamp_ns']
    q, lo, hi = bracket_feedback([row['accepted_before'], row['accepted_after']], ns,
                                max(row['epoch_ns'], row['selection']['stamp_ns']))
    checks = dict(
        archive_sha256=sha(path) == row['source_file_sha256'],
        rgb_sha256=array_sha(arrays['color_bgr']) == row['source_rgb_sha256'],
        depth_sha256=array_sha(arrays['depth_raw']) == row['source_depth_sha256'],
        source_context_digest=digest(context) == row['source_id'],
        camera_context=all(value == context['camera'].get(key) for key, value in camera.items()),
        config_context=row['config'] == asdict(tabletop.Config(**{
            key: value for key, value in context['perception'].items()
            if key in tabletop.Config.__dataclass_fields__})),
        source_stamp=int(arrays['source_stamp_ns']) == ns,
        intrinsics=np.array_equal(arrays['K'], [camera[key] for key in ('fx', 'fy', 'cx', 'cy')]),
        depth_scale=float(arrays['depth_scale']) == camera['depth_scale'],
        array_shape=arrays['depth_raw'].shape == (camera['height'], camera['width']) == arrays['color_bgr'].shape[:2],
        source_joints=np.array_equal(arrays['q_source'], row['q_source']) and np.array_equal(q, arrays['q_source']),
        recorded_transform=np.array_equal(arrays['T_base_camera'], row['T_base_camera']),
        handeye=np.array_equal(arrays['T_tool_camera'], context['tool_camera']) and context['tool_camera'] == context['current_tool_camera'],
        selection=row['selection'] == context['selection'],
        epoch=row['epoch_ns'] == context['epoch_ns'],
        camera_frame=camera['frame_id'] == 'camera_link' and camera['align_depth_to_color'] is True,
        context_unchanged=row.get('source_context_unchanged') is True,
    )
    require_rectified_source_after_activation(context['camera'], ns)
    checks['rectification_activation'] = True
    return dict(checks={k: bool(v) for k, v in checks.items()}, passed=all(checks.values()),
                accepted_feedback_interval_ms=(hi[0] - lo[0]) * 1e-6,
                fresh_at_recorded_adjudication=row.get('fresh_for_existing_frame_limit') is True,
                exact_returned_tf_stamp_auditable=False)


def describe(candidate, raw):
    mask = candidate.mask > 0
    return dict(bbox=list(candidate.bbox), quality=float(candidate.quality),
                visible_pca_extent_mm=(candidate.extent_m * 1000).tolist(),
                rgb_pixels=int(np.count_nonzero(mask)),
                nonzero_depth_fraction=float(np.mean(raw[mask] > 0)), metrics=candidate.metrics)


def analyze_row(row, capture, geometry_helper, parameters):
    output = dict(source_stamp_ns=row['source_stamp_ns'], geometry_authority=False,
                  target_identity_established=False)
    path = Path(row['source_file']).resolve()
    path.relative_to(capture)
    with np.load(path, allow_pickle=False) as archive:
        arrays = {key: archive[key] for key in archive.files}
    output['source_binding'] = source_binding(row, arrays, path)
    if not output['source_binding']['passed']:
        output['error'] = 'source_binding_failed'
        return output
    raw = arrays['depth_raw']; rgb = arrays['color_bgr']
    if raw.dtype != np.uint16 or rgb.dtype != np.uint8:
        raise ValueError('unsupported_source_array_dtype')
    camera = row['source_context']['camera']
    cfg = tabletop.Config(**row['config'])
    # The recorded source context owns the acquisition quality threshold too.
    minimum_quality = float(row['source_context']['perception'].get('minimum_geometry_quality', .5))
    output['config'] = asdict(cfg)
    output['minimum_geometry_quality'] = minimum_quality
    depth = raw.astype(np.float32) * float(arrays['depth_scale'])
    valid = np.isfinite(depth) & (depth >= cfg.depth_min_m) & (depth <= cfg.depth_max_m)
    rgb_valid = rectification_rgb_valid_mask(camera, raw.shape)
    proposals = rgb_observation_region.propose_rgb_observation_regions(
        rgb, policy=rgb_observation_region.CenterSearchPolicy(center_xy=(.5, .5), radius_fraction=.125),
        depth_valid=valid, rgb_valid=rgb_valid)
    attention = dict(reason=proposals.reason, candidate_count=proposals.total_candidate_count,
                     observation_only=True, does_not_override_measured_rgbd=True)
    if len(proposals.candidates) >= 2:
        first, second = proposals.candidates[:2]
        intersection = int(np.count_nonzero(first.mask & second.mask))
        attention['top_two'] = dict(areas=[first.area_px, second.area_px],
            iou=intersection / (first.area_px + second.area_px - intersection),
            intersection_over_smaller=intersection / min(first.area_px, second.area_px),
            centroid_distance_px=float(np.linalg.norm(np.asarray(first.centroid_xy) - second.centroid_xy)),
            score_gap=second.score - first.score, channels=[first.channels, second.channels])
    output['rgb_only_attention'] = attention
    detector = row.get('detector') or {}
    output['nearby_online_status'] = {key: detector.get(key) for key in (
        'source_stamp_ns', 'state', 'detail', 'target_locked', 'target_lost', 'geometry_quality')}
    output['nearby_online_status']['same_frame'] = detector.get('source_stamp_ns') == row['source_stamp_ns']
    try:
        result = tabletop.segment_with_support_fallback(depth, arrays['K'], arrays['T_base_camera'], cfg)
        output['support'] = result.metrics
        output['raw_candidate_count'] = len(result.candidates)
        candidate, reason = tabletop.TargetTracker(cfg).choose(result, raw.shape)
        output['initial_selection'] = reason
        if candidate is None:
            output['stop_stage'] = 'initial_selection'
            return output
        output['selected'] = describe(candidate, raw)
        if candidate.quality < minimum_quality:
            output.update(stop_stage='quality', error='insufficient_geometry_quality')
            return output
        candidate = rgb_instance.refine_candidate(rgb, depth, arrays['K'], arrays['T_base_camera'],
                                                 candidate, cfg, rgb_valid=rgb_valid)
        output['refined'] = describe(candidate, raw)
        selected_valid = (candidate.mask > 0) & valid
        output['refined']['valid_depth_fraction'] = float(np.mean(valid[candidate.mask > 0]))
        output['refined']['median_range_m'] = float(np.median(depth[selected_valid]))
        output['stop_stage'] = 'refine_pass'
    except ValueError as exc:
        output.update(stop_stage='segment_or_refine', error=str(exc))
        return output
    if parameters is not None:
        geometry, _ = geometry_helper.replay_stage(raw, rgb, row['camera'], arrays['T_base_camera'], cfg,
            minimum_quality, geometry_helper.geometry_arguments(parameters['/grasp_6d']['remote']))
        output['pre_snapshot_geometry'] = geometry.get('geometry')
        output['pre_snapshot_geometry_reason'] = geometry['reason']
    else:
        output['pre_snapshot_geometry_reason'] = 'not_evaluated_no_effective_pipeline_config'
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--pipeline-config', type=Path)
    args = parser.parse_args()
    capture = args.capture.resolve(); output = args.output.resolve()
    if not MOUNT.is_mount():
        raise RuntimeError('SanStand unavailable; no fallback')
    capture.relative_to(EXTERNAL.resolve()); output.relative_to(EXTERNAL.resolve())
    if output.exists():
        raise FileExistsError('analysis output already exists; do not overwrite evidence')
    source = capture / 'result.json'
    saved = json.loads(source.read_text())
    if not 1 <= len(saved['rows']) <= 60:
        raise ValueError('bounded capture of 1 to 60 rows required')
    parameters = None; helper = None
    if args.pipeline_config:
        parameters = json.loads(args.pipeline_config.read_text())
        if not isinstance(parameters.get('/grasp_6d', {}).get('remote'), dict):
            raise ValueError('effective /grasp_6d/remote configuration required')
        spec = importlib.util.spec_from_file_location('replay_original_geometry', WORK / '.runtime/replay_matched_shadow_geometry_20261005.py')
        helper = importlib.util.module_from_spec(spec); spec.loader.exec_module(helper)
    cv2.setNumThreads(1)
    report = dict(schema=1, capture=str(capture), capture_sha256=sha(source), script_sha256=sha(__file__),
        robot_commands_sent=False, geometry_authority=False, target_identity_established=False,
        pipeline_config=None if parameters is None else dict(path=str(args.pipeline_config.resolve()), sha256=sha(args.pipeline_config)),
        implementation_sha256={str(Path(module.__file__).resolve()): sha(module.__file__)
            for module in (tabletop, rgb_instance, rgb_observation_region)},
        limitations=[
            'Independent single-frame initial selection/refine only; no locked-target or temporal snapshot replay.',
            'Nearby online status is asynchronous; it is not the saved image frame unless same_frame is true.',
            'RGB-only attention ambiguity is a separate fallback diagnostic and does not invalidate an accepted RGB-D target.',
            'Saved TF matrices were requested at image header time, but returned TF header nanoseconds were not archived; exact returned stamp cannot be independently audited.',
            'No current freshness or execution authority survives in this offline report. Pixel density alone does not certify grasp geometry.',
            'Optional geometry uses the supplied configuration snapshot, whose capture time is separate from each image; multi-frame snapshot processing is absent.'], rows=[])
    if helper is not None:
        report['implementation_sha256'].update({str(Path(module.__file__).resolve()): sha(module.__file__)
            for module in (helper, helper.object_geometry)})
    for row in saved['rows']:
        try:
            report['rows'].append(analyze_row(row, capture, helper, parameters))
        except (ValueError, KeyError, TypeError) as exc:
            report['rows'].append(dict(source_stamp_ns=row.get('source_stamp_ns'), error=str(exc)))
    rows = report['rows']; refined = [row['refined'] for row in rows if 'refined' in row]
    report['summary'] = dict(total=len(rows), source_binding_pass=sum(row.get('source_binding', {}).get('passed', False) for row in rows),
        support_pass=sum('support' in row for row in rows), refine_pass=len(refined),
        support_inlier_ratio=limits(row['support'].get('support_inlier_ratio') for row in rows if 'support' in row),
        support_residual_mm=limits(1000 * row['support']['support_residual_m'] for row in rows if 'support' in row),
        valid_depth_fraction=limits(row['valid_depth_fraction'] for row in refined),
        valid_refined_points=limits(row['metrics']['valid_points'] for row in refined),
        rgb_only_reasons=dict(Counter(row.get('rgb_only_attention', {}).get('reason', 'not_run') for row in rows)),
        pre_snapshot_geometry_reasons=dict(Counter(row.get('pre_snapshot_geometry_reason', 'not_run') for row in rows)))
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    print(json.dumps(dict(output=str(output), sha256=sha(output), summary=report['summary']), ensure_ascii=False))


if __name__ == '__main__':
    main()
