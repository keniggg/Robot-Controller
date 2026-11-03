#!/usr/bin/env python3
"""Offline same-read regional depth comparison, with no motion/identity authority."""
import argparse
from collections import Counter
from dataclasses import asdict, fields
import importlib.util
import json
from pathlib import Path
import sys

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


support = load('regional_support_replay', HERE/'analyze_matched_shadow_capture_20261005.py')
geometry = load('regional_geometry_replay', HERE/'replay_matched_shadow_geometry_20261005.py')
STAGES = ('final_depth', 'regional_shadow_final_depth')
OWNER_SCHEMA = 'fixed_center_02_08_01_09_then_four_quadrants_v1'


def source_audit(info):
    """Audit actual saved fields; do not invent the older A/B frozen manifest."""
    evidence = info.get('source_evidence', {})
    feedback = evidence.get('feedback', {})
    guard = info.get('feedback_guard', {})
    anchor = info.get('read_completed_wall_ns')
    tf, audit = evidence.get('exact_tf'), evidence.get('tf_stamp_audit', {})
    original = info.get('production_read_diagnostics', {})
    pair = bool(info.get('complete') is True and original.get('result') == 'complete'
                and original.get('pair_timing', {}).get('accepted') is True
                and not any(info.get(k) for k in ('capture_error', 'metadata_error', 'context_incomplete')))
    reasons = [] if pair else ['original_pair_or_metadata_incomplete']
    if evidence.get('evidence_complete') is not True:
        reasons.append('recorded_source_evidence_incomplete')
    try:
        lo, hi = feedback['accepted_before'], feedback['accepted_after']
        q = np.asarray(feedback['q_at_anchor'], dtype=float)
        if not (type(anchor) is int and anchor > 0 and feedback['qualified'] is True
                and feedback['anchor_ros_stamp_ns'] == anchor and not feedback['faults']
                and guard['epoch'] <= lo['source_stamp_ns'] <= anchor <= hi['source_stamp_ns']
                and hi['source_stamp_ns']-lo['source_stamp_ns'] == feedback['span_ns'] <= 200000000
                and feedback['maximum_span_ns'] == 200000000
                and q.shape == (7,) and np.isfinite(q).all()
                and lo['q'] == hi['q'] == feedback['q_at_anchor'] == guard['accepted']['q']
                and guard['task']['active'] is False):
            raise ValueError('feedback contract')
    except (KeyError, TypeError, ValueError):
        reasons.append('feedback_bracket_not_qualified')
    try:
        if not (audit['requested_stamp_ns'] == audit['returned_stamp_ns'] == anchor
                and audit['delta_ns'] == 0 and audit['exact_match'] is True
                and tf['source_stamp_ns'] == anchor and tf['parent'] == 'base_link'
                and tf['child'] == 'camera_link' and len(tf['translation']) == 3
                and len(tf['quaternion_xyzw']) == 4
                and np.isfinite(tf['translation']+tf['quaternion_xyzw']).all()
                and abs(sum(v*v for v in tf['quaternion_xyzw'])-1.) <= 1e-6):
            raise ValueError('strict TF contract')
    except (KeyError, TypeError, ValueError):
        reasons.append('strict_tf_not_qualified')
    return dict(original_pair_complete=pair, source_qualified=not reasons,
        rejection_reasons=reasons, tf_stamp_audit=audit, recorded_tf_error=evidence.get('tf_error'),
        anchor_ros_stamp_ns=anchor, feedback_snapshot_source='actual per-sample feedback_guard',
        scope='Saved strict host read-completion TF/feedback binding only; not exposure pose, continuous context proof, identity or motion authority.')


def owner_layout(shape):
    h, w = shape
    boxes = [('center', (int(.2*w), int(.1*h), int(.8*w), int(.9*h))),
             ('top_left', (0, 0, w//2, h//2)), ('top_right', (w//2, 0, w, h//2)),
             ('bottom_left', (0, h//2, w//2, h)), ('bottom_right', (w//2, h//2, w, h))]
    owners = np.full(shape, -1, np.int8)
    for index, (_, (x0, y0, x1, y1)) in enumerate(boxes):
        patch = owners[y0:y1, x0:x1]
        patch[patch == -1] = index
    if np.any(owners < 0):
        raise ValueError('owner layout incomplete')
    return boxes, owners


def shadow_audit(info, arrays):
    """Verify output ownership from actual same-read depths and recorded gates."""
    record = info.get('regional_shadow', {})
    pre, shadow = arrays.get('pre_stationary_depth'), arrays.get('regional_shadow_depth')
    reasons = []
    if pre is None or shadow is None or pre.shape != shadow.shape:
        return dict(valid=False, contradictions=['shadow_arrays_missing_or_mismatched'])
    if info.get('production_output_replaced') is not False or info.get('shadow_geometry_authority') is not False:
        reasons.append('shadow_scope_not_recorded')
    if (record.get('alpha'), record.get('delta_depth_units'), record.get('persistence')) != (.4, 100, 0):
        reasons.append('shadow_parameters_differ')
    if record.get('owner_schema') != OWNER_SCHEMA:
        reasons.append('owner_schema_differs')
    if record.get('whole_frame_stationarity_qualified') is not False:
        reasons.append('incorrect_whole_frame_qualification_claim')
    holes = int(np.count_nonzero(shadow[pre == 0]))
    if holes:
        reasons.append('shadow_filled_current_zero')
    boxes, owners = owner_layout(pre.shape)
    rows = record.get('regions', [])
    allowed = np.zeros(pre.shape, bool)
    details = []
    if rows:
        if len(rows) != 5 or [r.get('name') for r in rows] != [name for name, _ in boxes]:
            reasons.append('regional_rows_or_order_differ')
        else:
            original_source = info.get('production_read_diagnostics', {}).get('raw_frames', {}).get('color', {})
            for index, (row, (name, bounds)) in enumerate(zip(rows, boxes)):
                owned = owners == index
                visual = row.get('visual', {})
                qualified, warming = row.get('qualified') is True, row.get('warming') is True
                if (row.get('bounds_xyxy') != list(bounds)
                        or row.get('owned_pixels') != int(owned.sum())
                        or row.get('filter_applied') is not qualified):
                    reasons.append(name+':owner_or_application_mismatch')
                if qualified:
                    if not (visual.get('qualified') is True and visual.get('reason') == 'stationary'
                            and visual.get('window_sec') == .35 and visual.get('max_motion_px') == .25
                            and visual.get('quiet_span_sec', -1) >= .35
                            and visual.get('host_quiet_span_sec', -1) >= .35
                            and visual.get('continuous_frames', 0) >= 4
                            and visual.get('motion_p90_px', 1) <= .25
                            and visual.get('covered_quadrants', 0) >= 3
                            and visual.get('tracked_points', 0) >= 24
                            and visual.get('frame_number') == original_source.get('frame_number')
                            and visual.get('device_timestamp_ms') == original_source.get('device_timestamp_ms')
                            and visual.get('stream_fps') == original_source.get('stream_fps')):
                        reasons.append(name+':recorded_qualification_inconsistent')
                    allowed |= owned
                if warming and (qualified or visual.get('reason') != 'warming_up'):
                    reasons.append(name+':warming_inconsistent')
                changed = int(np.count_nonzero((pre != shadow) & owned))
                if not qualified and changed:
                    reasons.append(name+':unqualified_owner_changed')
                details.append(dict(name=name, qualified=qualified, warming=warming,
                    history_reset=row.get('history_reset'), reason=visual.get('reason'),
                    owned_pixels=int(owned.sum()), changed_pixels=changed,
                    motion_p90_px=visual.get('motion_p90_px'), continuous_frames=visual.get('continuous_frames'),
                    quiet_span_sec=visual.get('quiet_span_sec')))
    changed = int(np.count_nonzero(pre != shadow))
    count = int(allowed.sum())
    if (record.get('active') is not bool(count) or record.get('fully_active') is not (count == pre.size)
            or record.get('filtered_pixel_count') != count
            or not np.isclose(record.get('filtered_fraction', -1), count/float(pre.size), rtol=0, atol=1e-12)
            or record.get('changed_pixel_count') != changed):
        reasons.append('aggregate_owner_counts_differ')
    unauthorized = int(np.count_nonzero((pre != shadow) & ~allowed))
    if unauthorized:
        reasons.append('pixels_changed_outside_qualified_fixed_owner')
    return dict(valid=not reasons, contradictions=reasons, active=bool(count),
        fully_active=count == pre.size, filtered_fraction=count/float(pre.size),
        filtered_pixel_count=count, changed_pixel_count=changed,
        pre_zero_pixels=int(np.count_nonzero(pre == 0)), filled_current_zero_pixels=holes,
        changed_unqualified_pixels=unauthorized, regions=details,
        global_history_reset_reason=record.get('global_history_reset_reason'),
        recorded_reason=record.get('reason'),
        continuous_rgb_history_replayed=False, encoder_source_window_replayed=False,
        limitation='Selected output ownership and stored per-region qualification are checked; unsaved intermediate RGB and exact encoder windows cannot be replayed.')


def map_audit(info, arrays, mapper):
    runtime = info['runtime_profile']
    if (runtime['rectification'] != mapper.contract
            or runtime['geometry_intrinsics'] != mapper.output_intrinsics
            or runtime['color_intrinsics'] != mapper.source_intrinsics):
        raise ValueError('captured rectification contract differs')
    scale = info['depth_scale_m']
    if not np.isfinite(scale) or scale <= 0 or scale != runtime['depth_scale_m_per_unit']:
        raise ValueError('captured depth scale differs')
    interval = runtime['depth_filters']['range_m']
    for before, after in (('post_stationary_depth', 'final_depth'),
                          ('regional_shadow_depth', 'regional_shadow_final_depth')):
        rgb, mapped = mapper.apply(arrays['raw_rgb'], support.clip_raw(arrays[before], scale, interval))
        if not np.array_equal(rgb, arrays['final_rgb']) or not np.array_equal(mapped, arrays[after]):
            raise ValueError(before+' clip/rectify cannot reproduce saved final RGB-D')
    camera = dict(mapper.output_intrinsics, depth_scale=scale, runtime_profile=runtime)
    return camera


def support_comparison(arrays, scale, intrinsics, cfg):
    shape = arrays[STAGES[0]].shape
    if arrays[STAGES[1]].shape != shape:
        raise ValueError('final arrays must share the same raster')
    v, u = np.indices(shape)
    radius = np.hypot(u-shape[1]/2., v-shape[0]/2.)
    factor = min(shape[1]/640., shape[0]/480.)
    ring = (radius >= 70*factor) & (radius <= 140*factor)
    depths = {stage:arrays[stage].astype(np.float32)*scale for stage in STAGES}
    valid = {stage:(depth >= cfg.depth_min_m) & (depth <= cfg.depth_max_m) for stage,depth in depths.items()}
    points = {stage:support.tabletop._project(depth, intrinsics) for stage,depth in depths.items()}
    common = ring & valid[STAGES[0]] & valid[STAGES[1]]
    report = dict(fixed_ring=dict(inner_radius_px=70*factor, outer_radius_px=140*factor,
        pixels=int(ring.sum()), mask_sha256=support.array_sha(ring)),
        common_valid_pixels=int(common.sum()), common_mask_sha256=support.array_sha(common), stages={})
    for stage in STAGES:
        _, own = support.fit_audit(points[stage][ring & valid[stage]], cfg)
        _, matched = support.fit_audit(points[stage][common], cfg)
        report['stages'][stage] = dict(original_initial_support=support.initial_support(
            points[stage], valid[stage], radius, u, v, cfg),
            valid_ring_fraction=float(np.mean(valid[stage][ring])),
            fixed_ring_own_coverage_fit=own, fixed_ring_common_coverage_fit=matched)
    report['same_valid_coverage'] = bool(np.array_equal(valid[STAGES[0]], valid[STAGES[1]]))
    report['common_signed_z_delta_mm'] = support.quantiles(
        (arrays[STAGES[1]][common].astype(float)-arrays[STAGES[0]][common])*scale*1000)
    return report


def summaries(rows):
    result = {}
    for name, accepts in (
        ('all_complete_camera_local', lambda row: True),
        ('strict_tf_feedback_qualified', lambda row: row['source']['source_qualified'])):
        selected = [row for row in rows if row.get('comparison') and accepts(row)]
        result[name] = dict(rows=len(selected), stages={})
        for stage in STAGES:
            measures = [row['comparison']['stages'][stage] for row in selected]
            result[name]['stages'][stage] = dict(
                initial_support_passes=sum(m['original_initial_support']['passed'] for m in measures),
                fixed_ring_fit_passes=sum(m['fixed_ring_own_coverage_fit']['fit_passed'] for m in measures),
                common_ring_residual_p90_mm=support.quantiles([
                    m['fixed_ring_common_coverage_fit']['abs_residual_mm']['p90'] for m in measures
                    if m['fixed_ring_common_coverage_fit'].get('abs_residual_mm') is not None]),
                geometry_stops=dict(Counter(row['geometry'][stage]['stop_stage']+':'+row['geometry'][stage]['reason']
                                           for row in selected if 'geometry' in row)))
    result['source_rejections'] = dict(Counter(reason for row in rows for reason in row['source']['rejection_reasons']))
    result['ownership_violations'] = dict(Counter(reason for row in rows for reason in row['shadow']['contradictions']))
    return result


def analyze(capture, config_path):
    cv2.setNumThreads(1)
    folder = capture if (capture/'summary.json').exists() else capture/'capture'
    summary, stems, integrity = support.verify_artifacts(folder)
    if summary['requested_samples'] != 8 or len(stems) > 8:
        raise ValueError('unexpected regional capture sample contract')
    params = json.loads(config_path.read_text())
    unknown = params.get('/unknown_perception') or {}
    cfg = support.tabletop.Config(**{field.name:unknown[field.name] for field in fields(support.tabletop.Config)
                                   if field.name in unknown})
    if cfg.support_threshold_m != .002 or cfg.support_min_ratio != .70:
        raise ValueError('original 2mm / 70 percent support gates required')
    arguments = geometry.geometry_arguments(params['/grasp_6d']['remote'])
    loaded_path = folder/'loaded_modules.json'
    loaded = json.loads(loaded_path.read_text())
    loaded_checks = {name:dict(recorded=row['sha256'], current=support.sha(row['path']),
        same=support.sha(row['path']) == row['sha256']) for name,row in loaded.items()}
    outer_path = folder.parent/'result.json'
    outer = json.loads(outer_path.read_text()) if outer_path.exists() else None
    report = dict(schema=1, capture=str(folder), artifact_integrity=integrity,
        capture_complete=summary['capture_complete'], capture_counts=summary['counts'], capture_errors=summary['errors'],
        continuous_shadow_counts=summary.get('regional_shadow_counts'),
        loaded_module_comparison=loaded_checks, loaded_manifest_sha256=support.sha(loaded_path),
        runner_completed=outer.get('completed') if outer else None,
        runner_error=outer.get('error') if outer else None,
        config_source=str(config_path), config_sha256=support.sha(config_path), config=asdict(cfg),
        geometry_arguments=arguments, minimum_geometry_quality=float(unknown.get('minimum_geometry_quality', .5)),
        analysis_sources={str(path):support.sha(path) for path in (Path(__file__), HERE/'analyze_matched_shadow_capture_20261005.py',
            HERE/'replay_matched_shadow_geometry_20261005.py', Path(support.tabletop.__file__),
            Path(geometry.rgb_instance.__file__), Path(geometry.object_geometry.__file__))},
        geometry_authority=False, target_identity_established=False, production_output_replaced=False, rows=[],
        limitations=[
            'Only this new target/view and these same-read arrays are compared; no old target reference or manual mask is used.',
            'Original strict TF stamp failures remain failures. Camera-local quality statistics do not grant robot-base geometry or motion permission.',
            'Host read completion includes diagnostic overhead; it is not an exposure timestamp or a published image stamp.',
            'Context is polled every 0.5 seconds; changes and returns between polls were not latched. This does not establish continuous source-context authority.',
            'Saved per-region gates and fixed-owner pixel invariants can be checked; selected eight frames cannot replay intermediate RGB history or exact stationary encoder windows.',
            'Geometry replay uses original single-frame initial proposals and pre-snapshot geometry; no online acquisition, locked reference, temporal snapshot, identity or grasp success is reconstructed.',
            'The geometry transform is the strict host-anchor TF only when its recorded feedback and exact TF audit pass, otherwise camera_link diagnostic coordinates are used.',
            'Configuration is the separately captured pipeline configuration, not a newly read parameter set; its hash is retained. Fixed support rings are not independent tabletop ground truth.'])
    sys.path.insert(0, str(support.WORK/'.runtime/realsense-2.56.5'))
    import pyrealsense2 as rs
    mapper = None
    for stem in stems:
        info, arrays = support.read_sample(folder, stem)
        row = dict(index=info['sample_index'], source=source_audit(info), shadow=shadow_audit(info, arrays),
            production_temporal=info.get('runtime_profile', {}).get('stationary_temporal'),
            shadow_counts=info.get('shadow_counts'), tap_processing_sec=info.get('shadow_tap_processing_sec'))
        report['rows'].append(row)
        if not row['source']['original_pair_complete'] or not row['shadow']['valid']:
            row['excluded'] = 'incomplete original pair or inconsistent regional ownership evidence'
            continue
        if mapper is None:
            mapper = support.PairedPinholeRectification(info['runtime_profile']['color_intrinsics'], sdk=rs)
            report['rectification_contract'] = mapper.contract
        camera = map_audit(info, arrays, mapper)
        row['saved_final_arrays_reproduced_exactly'] = True
        k = [camera[key] for key in ('fx', 'fy', 'cx', 'cy')]
        row['comparison'] = support_comparison(arrays, info['depth_scale_m'], k, cfg)
        transform, row['geometry_coordinate_frame'] = geometry.transform_for(info, row['source']['source_qualified'])
        row['geometry'] = {}
        for stage in STAGES:
            cv2.setRNGSeed(0)
            item, masks = geometry.replay_stage(arrays[stage], arrays['final_rgb'], camera, transform,
                cfg, report['minimum_geometry_quality'], arguments)
            item['diagnostic_mask_hashes'] = {key:support.array_sha(mask) for key,mask in masks.items()}
            row['geometry'][stage] = item
    report['summary'] = summaries(report['rows'])
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not support.MOUNT.is_mount() or support.EXT not in args.output.resolve().parents:
        raise ValueError('analysis output requires mounted SanStand; no fallback')
    if args.output.exists():
        raise ValueError('new output path required; never overwrite evidence')
    report = analyze(args.capture, args.config)
    with args.output.open('x') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(dict(output=str(args.output), sha256=support.sha(args.output), summary=report['summary']), ensure_ascii=False))


if __name__ == '__main__':
    main()
