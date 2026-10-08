#!/usr/bin/env python3
"""Offline per-frame initial selection and pre-snapshot geometry diagnostics.

No ROS, prior target, identity, synthetic snapshot or device access.  Does not
claim to replay the missing online multi-frame acquisition/snapshot history.
"""
import argparse
from collections import Counter
from dataclasses import asdict, fields
import importlib.util
import json
from pathlib import Path
import shutil
import sys
from types import SimpleNamespace

import cv2
import numpy as np

WORK = Path(__file__).resolve().parents[1]
for rel in ('src/alicia_grasp_modes/src', 'src/alicia_flexible_grasp_supervisor/src'):
    sys.path.insert(0, str(WORK/rel))
from alicia_grasp_modes import tabletop, rgb_instance
from alicia_flexible_grasp.vision import object_geometry
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask

spec = importlib.util.spec_from_file_location('matched_support', WORK/'.runtime/analyze_matched_shadow_capture_20261005.py')
support = importlib.util.module_from_spec(spec)
spec.loader.exec_module(support)
OPTICAL_TO_LINK = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])


def transform_for(info, qualified):
    """Unqualified sources remain in real camera-link coordinates, never base."""
    result = np.eye(4)
    if not qualified:
        return result, 'camera_link_diagnostic'
    record = info['source_evidence']['exact_tf']
    if record['parent'] != 'base_link' or record['child'] != 'camera_link':
        raise ValueError('unexpected strict source coordinate frames')
    x, y, z, w = np.asarray(record['quaternion_xyzw'], dtype=float)
    if abs(x*x+y*y+z*z+w*w-1.) > 1e-6:
        raise ValueError('nonunit strict source quaternion')
    result[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                      [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                      [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    result[:3, 3] = record['translation']
    return result, 'base_link_strict_host_anchor_diagnostic'


def geometry_arguments(remote):
    """Exact effective scalar defaults/clamps of the production constructor."""
    g = remote.get('gripper_geometry', {}) or {}
    fingers = g.get('finger_box_xyz_m', [.0434, .0286, .0600])
    minimum = max(.0001, float(remote.get('geometry_min_size_m', .005)))
    return dict(support_bbox_expand_ratio=max(0., float(remote.get('support_bbox_expand_ratio', .30))),
        support_minimum_span_m=max(float(fingers[0]), float(g.get('max_inner_gap_m', .05))
            + 2*float(fingers[1]))+2*float(g.get('support_clearance_m', .003)),
        support_distance_threshold_m=max(.0001, float(remote.get('support_distance_threshold_m',
            remote.get('target_cloud_support_plane_inlier_distance_m', .004)))),
        voxel_size_m=max(.0001, float(remote.get('target_cloud_voxel_size_m', .0025))),
        min_support_points=max(3, int(remote.get('geometry_min_support_points', 200))),
        min_object_points=max(3, int(remote.get('geometry_min_object_points', remote.get('target_cloud_min_points', 120)))),
        min_size_m=minimum, max_size_m=max(minimum, float(remote.get('geometry_max_size_m', .600))),
        max_height_m=max(minimum, float(remote.get('geometry_max_height_m', .500))),
        outlier_neighbors=max(1, int(remote.get('target_cloud_outlier_neighbors', 16))),
        outlier_std_ratio=max(0., float(remote.get('target_cloud_outlier_std_ratio', 2.0))),
        previous_axes_base=None)


def describe(c):
    return dict(bbox=list(c.bbox), center_uv=c.center_uv.tolist(), quality=c.quality,
        visible_pca_extents_mm=(c.extent_m*1000).tolist(),
        mask_pixels=int(np.count_nonzero(c.mask)), metrics=c.metrics)


def replay_stage(raw, rgb, camera, transform, config, minimum_quality, arguments):
    """Use original pure functions, retaining their unmodified rejections."""
    scale = camera['depth_scale']
    k = [camera[key] for key in ('fx', 'fy', 'cx', 'cy')]
    depth = raw.astype(np.float32)*scale
    row = dict(geometry_authority=False, target_identity_established=False,
        pipeline_scope='single_frame_initial_proposal_then_pre_snapshot_geometry')
    masks = {}
    try:
        segmentation = tabletop.segment_with_support_fallback(depth, k, transform, config)
    except ValueError as exc:
        row.update(stop_stage='support_segmentation', reason=str(exc))
        return row, masks
    row['segmentation'] = segmentation.metrics
    row['raw_candidates'] = [describe(c) for c in segmentation.candidates]
    candidate, reason = tabletop.TargetTracker(config).choose(segmentation, depth.shape)
    if candidate is None:
        row.update(stop_stage='initial_selection', reason=reason)
        return row, masks
    row['selected'] = describe(candidate)
    masks['depth_seed'] = candidate.mask
    if candidate.quality < minimum_quality:
        row.update(stop_stage='geometry_quality', reason='insufficient_geometry_quality')
        return row, masks
    try:
        candidate = rgb_instance.refine_candidate(rgb, depth, k, transform, candidate, config,
            rgb_valid=rectification_rgb_valid_mask(camera, depth.shape))
    except ValueError as exc:
        row.update(stop_stage='rgb_refinement', reason=str(exc))
        return row, masks
    row['refined'] = describe(candidate)
    masks['refined'] = candidate.mask
    # No invented TargetTrackIdentity/temporal snapshot. These are only the
    # current measured pixels inside the independently selected RGB outline.
    target = np.where(candidate.mask > 0, raw, 0).astype(raw.dtype)
    valid_count = int(np.count_nonzero(target))
    row['pre_snapshot_target_depth_points'] = valid_count
    if valid_count < arguments['min_object_points']:
        row.update(stop_stage='pre_snapshot_minimum_points', reason='DEPTH_INSUFFICIENT')
        return row, masks
    optical_transform = transform.copy()
    optical_transform[:3, :3] = transform[:3, :3] @ OPTICAL_TO_LINK
    estimate = object_geometry.estimate_object_geometry(depth_raw=raw, target_depth_raw=target,
        object_mask=candidate.mask, bbox=candidate.bbox,
        intrinsics=SimpleNamespace(**{key:camera[key] for key in ('width','height','fx','fy','cx','cy')}),
        depth_scale=scale, T_base_camera=optical_transform, source_mode='instance_mask', **arguments)
    evidence = estimate if estimate.ok else estimate.observation_deficit
    row['geometry'] = dict(ok=estimate.ok, failure_code=estimate.failure_code,
        failure_reason=estimate.failure_reason, observation_deficit=estimate.observation_deficit is not None,
        measured_obb_size_mm=None if evidence is None else (evidence.size_xyz_m*1000).tolist(),
        source_mode=estimate.source_mode, min_size_gate_mm=arguments['min_size_m']*1000,
        axis_roles=['support_plane_long','support_plane_short','support_normal_height'],
        cleaned_points=None if evidence is None else len(evidence.object_points_base))
    row.update(stop_stage='pre_snapshot_geometry', reason='accepted_diagnostic_only' if estimate.ok else estimate.failure_code)
    return row, masks


def analyze(capture, config_path, output):
    if not support.MOUNT.is_mount():
        raise ValueError('SanStand unavailable; no fallback')
    output.resolve().relative_to(capture.resolve())
    output.mkdir(exist_ok=False)
    summary, stems, integrity = support.verify_artifacts(capture/'stages')
    params = json.loads(config_path.read_text())
    unknown = params.get('/unknown_perception') or {}
    cfg = tabletop.Config(**{f.name:unknown[f.name] for f in fields(tabletop.Config) if f.name in unknown})
    arguments = geometry_arguments(params['/grasp_6d']['remote'])
    report = dict(schema=1, capture=str(capture), config_source=str(config_path),
        config_sha256=support.sha(config_path), script_sha256=support.sha(__file__), artifact_integrity=integrity,
        initial_selection_config=asdict(cfg), minimum_geometry_quality=float(unknown.get('minimum_geometry_quality', .5)),
        geometry_arguments=arguments, geometry_authority=False, target_identity_established=False,
        implementation_sha256={str(Path(m.__file__).resolve()):support.sha(m.__file__) for m in (tabletop,rgb_instance,object_geometry)},
        limitations=[
            'Independent per-frame proposals only. No initial acquisition or original locked-target RGB/cache is recreated; no continuity/identity authority is established.',
            'Original multi-frame snapshot inputs are absent. Geometry uses current depth within the original full RGB outline before snapshot temporal median, erosion, MAD or hole processing. It cannot certify improvement over the old online 4.278 mm short axis.',
            'Previous geometry axes are unavailable and left None, as on a new initial proposal. No prior target mask or manually selected center is loaded.',
            'Only original complete pairs are replayed. Source-unqualified frames use camera_link coordinates, not fabricated base TF. Qualified TF is at host read completion, not exposure.',
            'Matched shadow is a private recorded branch, not published depth. Its eligibility remains separately reported; no new pixels or thresholds are introduced.',
            'Current runtime configuration was saved after capture, not an exact snapshot at each sample; source hashes and captured camera contracts are preserved.'],
        rows=[])
    sys.path.insert(0, str(WORK/'.runtime/realsense-2.56.5'))
    import pyrealsense2 as rs
    mapper = None
    cv2.setNumThreads(1)
    for stem in stems:
        info, arrays = support.read_sample(capture/'stages', stem)
        source = support.source_classification(info, summary)
        shadow = support.shadow_classification(info, arrays)
        row = dict(index=info['sample_index'], phase=info['phase'], source=source,
            shadow={k:v for k,v in shadow.items() if k != 'recorded'}, stages={})
        report['rows'].append(row)
        if not source['original_pair_complete'] or shadow['contradictions']:
            row['excluded'] = 'incomplete pair or contradictory shadow'; continue
        if mapper is None:
            mapper = support.PairedPinholeRectification(info['runtime_profile']['color_intrinsics'],sdk=rs)
        mapped = support.mapped_stages(info, arrays, mapper)
        transform, frame = transform_for(info, source['source_qualified'])
        row['coordinate_frame'] = frame
        for stage in ('pre_stationary_depth', 'matched_shadow_depth', 'final_depth'):
            cv2.setRNGSeed(0)
            result, masks = replay_stage(mapped[stage], arrays['final_rgb'], info['published_camera_parameters'],
                transform, cfg, report['minimum_geometry_quality'], arguments)
            row['stages'][stage] = result
            # Store diagnostic masks, not RGB edits or authority-bearing snapshots.
            for kind, mask in masks.items():
                name = '%s_%s_%s.png' % (stem, stage, kind)
                if not cv2.imwrite(str(output/name), mask): raise OSError('mask save failed')
                result.setdefault('mask_artifacts', {})[kind] = dict(path=name,sha256=support.sha(output/name))
    report['phase_summary'] = {}
    for phase in ('A1','B','A2'):
        items = [r for r in report['rows'] if r['phase']==phase and r['stages']]
        report['phase_summary'][phase] = {stage:dict(
            rows=len(items), strict_source_rows=sum(r['source']['source_qualified'] for r in items),
            stops=dict(Counter(r['stages'][stage]['stop_stage']+':'+r['stages'][stage]['reason'] for r in items)),
            obb_rows=[dict(index=r['index'], strict_source=r['source']['source_qualified'],
                shadow_history=r['shadow']['history_filtered'], **r['stages'][stage]['geometry'])
                for r in items if 'geometry' in r['stages'][stage]])
            for stage in ('pre_stationary_depth','matched_shadow_depth','final_depth')}
    shutil.copy2(__file__, output/Path(__file__).name)
    (output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(output),report_sha256=support.sha(output/'report.json'),summary=report['phase_summary'])))


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture',type=Path,required=True)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    analyze(args.capture,args.config,args.output)
