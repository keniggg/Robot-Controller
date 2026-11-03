"""Offline exact-source replay of the fixed support-domain change; no ROS I/O."""
import argparse
from collections import Counter
from dataclasses import fields
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from alicia_grasp_modes import rgb_observation_handoff as handoff, tabletop
from alicia_grasp_modes.rgb_observation_region import CenterSearchPolicy, propose_rgb_observation_regions
from alicia_grasp_modes.rgb_observation_projection import projected_rgb_search_policy
from alicia_grasp_modes.rgb_observation_source import measured_support_coverage
from alicia_grasp_modes.rgb_observation_volume import RGBObservationVolume, SupportPlaneProvenance


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def matrix(K):
    return np.array([[K[0], 0., K[2]], [0., K[1], K[3]], [0., 0., 1.]])


def construct(cls, values):
    return cls(**{f.name:values[f.name] for f in fields(cls) if f.init and f.name in values})


def main(support_path, output):
    cv2.setNumThreads(1)
    old_report = json.loads(support_path.read_text())
    extract_path = Path(old_report['provenance']['extract'])
    archive = json.loads(extract_path.read_text())
    archived = json.loads(Path(archive['reference_path']).read_text())
    meta = archived['frames']['initial']; context = meta['source_context']; interest = meta['interest']
    assert sha(meta['source_file']) == meta['source_file_sha256']
    raw = np.load(meta['source_file'], allow_pickle=False)
    volume_raw = dict(meta['volume'])
    volume_raw['support_provenance'] = construct(SupportPlaneProvenance, volume_raw['support_provenance'])
    volume = construct(RGBObservationVolume, volume_raw)
    source = int(raw['source_stamp_ns'])
    first = handoff.RgbHandoffFrame(raw['color_bgr'], raw['depth_raw'].astype(np.float32)*float(raw['depth_scale']),
        matrix(raw['K']), raw['T_base_camera'], volume.support_provenance, meta['source_id'],
        context['selection']['generation'], context['epoch_ns'], context['selection']['stamp_ns'],
        source, source, source+200_000_000, tuple(meta[k] for k in ('accepted_before', 'accepted_after')),
        support_ring_outer_radius_px=interest['support_metrics']['support_ring_outer_radius_px'])
    policy = CenterSearchPolicy(**interest['rgb_search_policy'])
    original_regions = propose_rgb_observation_regions(first.bgr, policy=policy)
    region, = [r for r in original_regions.candidates if np.array_equal(r.mask, raw['region_mask'])]
    reference = handoff.freeze_rgb_interest(first, region, volume,
        support_ring_outer_radius_px=first.support_ring_outer_radius_px)
    assert np.array_equal(reference.measured_support_footprint_base, old_report['reference']['footprint_base_m'])
    config = construct(tabletop.Config, context['perception'])
    lookup = {r['source_stamp_ns']:r for r in archive['raw_frames']}
    domain = reference.support_consistency_domain_base.copy()
    rows = []
    for previous in old_report['rows']:
        row = lookup[previous['source_stamp_ns']]
        assert sha(row['file']) == row['sha256']
        data = np.load(row['file'], allow_pickle=False)
        source = row['source_stamp_ns']; status = row['status']['message']
        depth = data['depth_raw'].astype(np.float32)*float(data['depth_scale'])
        T, K, rgb = data['T_base_camera'], data['K'], data['color_bgr']
        seg = tabletop.segment(depth, K, T, config); metrics = seg.metrics
        coverage = measured_support_coverage(depth, K, T, seg.plane_base, config,
            metrics['support_ring_outer_radius_px'])
        provenance = SupportPlaneProvenance(source, tuple(seg.plane_base),
            'rgb_handoff:%s:%d'%(archive['reference_id'], source),
            int(round(metrics['support_sample_count']*metrics['support_inlier_ratio'])),
            metrics['support_inlier_ratio'], metrics['support_residual_m'], coverage['spatial_quadrants'])
        current = handoff.RgbHandoffFrame(rgb, depth, matrix(K), T, provenance,
            reference.source_id, reference.generation, reference.epoch_ns, reference.selection_stamp_ns,
            source, source, status['frame_timing']['status_build_ros_ns'],
            tuple(row[k] for k in ('accepted_before', 'accepted_after')),
            support_ring_outer_radius_px=metrics['support_ring_outer_radius_px'])
        observation_index = status['rgb_acquisition_handoff']['observation_index']
        audit = handoff.associate_rgb_interest(reference, current, (), observation_index=observation_index)
        assert audit.reason == 'no_cross_view_rgb_match', audit.reason
        search = projected_rgb_search_policy(policy, audit.projected_volume_pixels, depth.shape)
        proposals = propose_rgb_observation_regions(rgb, policy=search,
            depth_valid=np.isfinite(depth)&(depth >= config.depth_min_m)&(depth <= config.depth_max_m))
        association = handoff.associate_rgb_interest(reference, current, proposals, seg.candidates,
            observation_index=observation_index)
        measured = handoff.RgbMeasuredHandoff().observe(association)
        assert not measured.ready and association.measured_candidate is None
        assert np.array_equal(reference.support_consistency_domain_base, domain)
        native = data['depth_raw']
        rows.append(dict(source_stamp_ns=source, source_file=row['file'], source_sha256=row['sha256'],
            recorded_reason=previous['recorded_reason'], observation_index=observation_index,
            support_passes=True, support_audit=dict(association.support_audit),
            association_reason=association.reason, rgb_associated=association.associated,
            measured_candidate_index=association.measured_candidate_index, measured_ready=measured.ready,
            measured_handoff_reason=measured.reason, metric_geometry_authority=association.metric_geometry_authority,
            motion_authorized=association.motion_authorized, candidate_count=len(seg.candidates),
            depth_native_sha256=hashlib.sha256(native.tobytes()).hexdigest(),
            depth_zero_count=int(np.count_nonzero(native == 0)),
            synthetic_measured_points_added=association.synthetic_measured_points_added,
            alternatives=association.alternatives))
    failures = [r for r in rows if r['recorded_reason'].startswith('support_inconsistent')]
    assert len(rows) == 22 and len(failures) == 8
    distances = [r['support_audit']['support_reference_max_distance_m']*1000 for r in failures]
    summary = dict(exact_frames=len(rows), originally_support_rejected=len(failures),
        support_qualified=sum(r['support_passes'] for r in rows),
        formerly_rejected_fixed_domain_distance_mm=distances,
        formerly_rejected_distance_range_mm=[min(distances), max(distances)],
        association_reasons=dict(Counter(r['association_reason'] for r in rows)),
        measured_ready_count=sum(r['measured_ready'] for r in rows),
        all_original_and_current_domains_covered=all(r['support_audit']['support_original_domain_covered_vertices'] == 8
            and r['support_audit']['support_current_domain_covered_vertices'] == 8 for r in rows))
    result = dict(scope=__doc__, robot_commands_sent=False, ros_master_io=False,
        production_deployed=False, projection_model='unchanged pinhole; isolated support-domain correction',
        provenance=dict(extract=str(extract_path), extract_sha256=sha(extract_path),
            old_support_report=str(support_path), old_support_report_sha256=sha(support_path),
            script_sha256=sha(__file__), handoff_sha256=sha(handoff.__file__),
            tabletop_sha256=sha(tabletop.__file__), original_reference=str(archive['reference_path']),
            original_reference_sha256=sha(archive['reference_path'])),
        original_reference_id=archive['reference_id'], reference_source_stamp_ns=reference.source_stamp_ns,
        original_measured_support_footprint_base=reference.measured_support_footprint_base.tolist(),
        fixed_consistency_domain_base=domain.tolist(), summary=summary, rows=rows,
        limits=['Offline exact-source replay, not live qualification or deployment.',
            'Coverage is the convex hull of measured inliers; constructed domain vertices are not measured pixels.',
            'This scope change cannot fill target depth holes or authorize a grasp.',
            'Remaining pinhole RGB association failures are retained without changing the 0.80 threshold.'])
    output.mkdir(parents=True, exist_ok=False)
    target = output/'report.json'
    target.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(report=str(target), sha256=sha(target), summary=summary), indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--support-report', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(); main(args.support_report, args.output)
