"""Offline two-ended SDK lens-model counterfactual; no geometry or motion authority.

The original support planes, TF, intrinsics values, timestamps, proposals policy,
RGB appearance and all association gates are retained. Only source bbox ray
deprojection and destination volume projection use the recorded lens model.
"""
from collections import Counter
from dataclasses import asdict, fields
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np

WORK = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(WORK/'.runtime/realsense-2.56.5'))
import pyrealsense2 as rs

import replay_gen13_rgb_misses_20261004 as baseline
from alicia_grasp_modes import rgb_observation_handoff as handoff
from alicia_grasp_modes import tabletop
from alicia_grasp_modes.rgb_observation_projection import projected_rgb_search_policy
from alicia_grasp_modes.rgb_observation_region import propose_rgb_observation_regions
from alicia_grasp_modes.rgb_observation_source import measured_support_coverage
from alicia_grasp_modes.rgb_observation_volume import SupportPlaneProvenance

OUT = baseline.ARCHIVE.parent/'rgb_sdk_two_ended_counterfactual.json'


def intrinsics_from_context(context):
    info = context['camera']['runtime_profile_geometry']['geometry_intrinsics']
    result = rs.intrinsics()
    result.width, result.height = 640, 480
    result.fx, result.fy = info['fx'], info['fy']
    result.ppx, result.ppy = info['cx'], info['cy']
    result.model = getattr(rs.distortion, info['model'])
    result.coeffs = info['coeffs']
    return result, info


def sdk_vertices(volume, intrinsics):
    pixels = np.asarray(volume.pixel_boundary_xy)
    optical = np.array([rs.rs2_deproject_pixel_to_point(intrinsics, p.tolist(), 1.) for p in pixels])
    camera_rays = np.c_[optical[:, 2], -optical[:, 0], -optical[:, 1]]
    transform = np.asarray(volume.T_base_camera)
    plane = np.asarray(volume.support_plane_base)
    origin = transform[:3, 3]
    rays = camera_rays @ transform[:3, :3].T
    camera_height = float(plane[:3] @ origin + plane[3])
    slopes = rays @ plane[:3]
    assert np.all(slopes < 0.)
    depths = np.r_[-camera_height/slopes, (volume.foreground_max_m-camera_height)/slopes]
    assert np.all(depths >= volume.working_depth_range_m[0])
    assert np.all(depths <= volume.working_depth_range_m[1])
    return origin + np.vstack((rays, rays))*depths[:, None], depths


def sdk_pixels(vertices, transform, intrinsics, limits):
    camera = (vertices-transform[:3, 3]) @ transform[:3, :3]
    optical = np.c_[-camera[:, 1], -camera[:, 2], camera[:, 0]]
    assert np.all(optical[:, 2] >= limits[0])
    assert np.all(optical[:, 2] <= limits[1])
    pixels = np.array([rs.rs2_project_point_to_pixel(intrinsics, p.tolist()) for p in optical])
    assert np.isfinite(pixels).all()
    assert np.all(pixels >= -.5)
    assert np.all(pixels[:, 0] <= intrinsics.width-.5)
    assert np.all(pixels[:, 1] <= intrinsics.height-.5)
    return pixels


def candidate_audit(reference, rgb, regions, projected, policy):
    projected_mask = np.zeros(rgb.shape[:2], np.uint8)
    hull = cv2.convexHull(np.asarray(projected, dtype=np.float32)).reshape(-1, 2)
    cv2.fillConvexPoly(projected_mask, np.rint(hull).astype(np.int32), 1)
    candidates = []
    for i, candidate in enumerate(regions.candidates):
        mask = candidate.mask.astype(np.uint8)
        hist, contour = handoff._appearance(rgb, mask)
        colour = float(np.sqrt(max(0., 1.-np.sum(np.sqrt(hist*reference.histogram)))))
        shape = float(cv2.matchShapes(reference.contour, contour, cv2.CONTOURS_MATCH_I1, 0.))
        inside, total = int(np.count_nonzero(mask & projected_mask)), int(np.count_nonzero(mask))
        candidates.append(dict(index=i, bbox_xywh=list(candidate.bbox_xywh), mask_pixels=total,
            mask_sha256=hashlib.sha256(mask.tobytes()).hexdigest(),
            projected_inside_pixels=inside, projected_fraction=inside/total,
            colour_distance=colour, contour_distance=shape,
            appearance_passes=colour <= policy.max_colour_distance and shape <= policy.max_contour_distance,
            projection_passes=inside/total >= policy.min_projected_fraction))
    return candidates


def main():
    cv2.setNumThreads(1)
    assert rs.__version__ == '2.56.5'
    reference, context, base_policy = baseline.original_reference()
    evidence = json.loads(baseline.ARCHIVE.read_text())
    original = json.loads(baseline.OUT.read_text())
    original_rows = {r['source_stamp_ns']: r for r in original['records']}
    intrinsics, info = intrinsics_from_context(context)
    vertices, depths = sdk_vertices(reference.volume, intrinsics)
    roundtrip_pixels = np.array([[u, v] for v in range(480) for u in range(640)], dtype=float)
    roundtrip_pixels = np.vstack((roundtrip_pixels, reference.volume.pixel_boundary_xy))
    roundtrip = np.array([rs.rs2_project_point_to_pixel(intrinsics,
        rs.rs2_deproject_pixel_to_point(intrinsics, p.tolist(), 1.)) for p in roundtrip_pixels])
    errors = abs(roundtrip-roundtrip_pixels)
    assert errors.max() < .001
    original_projection = handoff.project_rgb_observation_volume

    def sdk_projection(volume, *, T_base_camera, camera_matrix, image_shape, working_depth_range_m):
        assert volume is reference.volume
        assert tuple(image_shape) == (480, 640)
        assert np.array_equal(camera_matrix, reference.volume.camera_matrix)
        return tuple(map(tuple, sdk_pixels(vertices, np.asarray(T_base_camera), intrinsics, working_depth_range_m)))

    config = tabletop.Config(**{f.name: context['perception'][f.name]
        for f in fields(tabletop.Config) if f.name in context['perception']})
    policy = handoff.HandoffPolicy()
    records = []
    try:
        # Process-local numerical substitution, never edits production or a live node.
        handoff.project_rgb_observation_volume = sdk_projection
        for row in evidence['raw_frames']:
            source = row['source_stamp_ns']
            old = original_rows[source]
            raw = np.load(row['file'])
            assert baseline.sha(row['file']) == row['sha256']
            status = row['status']['message']
            observation = status['rgb_acquisition_handoff']['observation_index']
            rgb = raw['color_bgr']
            depth = raw['depth_raw'].astype(np.float32)*float(raw['depth_scale'])
            transform, K = raw['T_base_camera'], raw['K']
            segment = tabletop.segment(depth, K, transform, config)
            metrics = segment.metrics
            coverage = measured_support_coverage(depth, K, transform, segment.plane_base, config,
                metrics['support_ring_outer_radius_px'])
            provenance = SupportPlaneProvenance(source, tuple(segment.plane_base),
                'rgb_handoff:%s:%d' % (evidence['reference_id'], source),
                int(round(metrics['support_sample_count']*metrics['support_inlier_ratio'])),
                metrics['support_inlier_ratio'], metrics['support_residual_m'], coverage['spatial_quadrants'])
            frame = handoff.RgbHandoffFrame(rgb, depth, baseline.camera_matrix(K), transform,
                provenance, reference.source_id, reference.generation, reference.epoch_ns,
                reference.selection_stamp_ns, source, source,
                status.get('frame_timing', {}).get('status_build_ros_ns', row['status']['bag_arrival_ns']),
                tuple((int(row[k][0]), tuple(row[k][1])) for k in ('accepted_before', 'accepted_after')))
            audit = handoff.associate_rgb_interest(reference, frame, (), observation_index=observation)
            result = dict(source_stamp_ns=source, tags=row['tags'], observation_index=observation,
                baseline_reason=old['replayed_reason'], sdk_reason=audit.reason,
                support_audit=dict(audit.support_audit), source_sha256=row['sha256'])
            if audit.reason != 'no_cross_view_rgb_match':
                records.append(result)
                continue
            projected = np.asarray(audit.projected_volume_pixels)
            projected_original = np.asarray(old['projected_volume_pixels'])
            search = projected_rgb_search_policy(base_policy, projected, depth.shape)
            regions = propose_rgb_observation_regions(rgb, policy=search,
                depth_valid=np.isfinite(depth) & (depth >= config.depth_min_m) & (depth <= config.depth_max_m))
            association = handoff.associate_rgb_interest(reference, frame, regions, segment.candidates,
                observation_index=observation)
            new_candidates = candidate_audit(reference, rgb, regions, projected, policy)
            old_candidates = {c['mask_sha256']: c for c in old['proposal_candidates']}
            for c in new_candidates:
                previous = old_candidates.get(c['mask_sha256'])
                c['baseline_fraction_same_mask'] = None if previous is None else previous['projected_fraction']
                c['baseline_inside_pixels_same_mask'] = None if previous is None else previous['projected_inside_pixels']
            result.update(sdk_reason=association.reason, projected_volume_pixels=projected.tolist(),
                vertex_shift_xy_px=(projected-projected_original).tolist(),
                max_vertex_shift_px=float(np.max(np.linalg.norm(projected-projected_original, axis=1))),
                search_policy=asdict(search), proposal_reason=regions.reason,
                proposal_count=len(regions.candidates), proposal_candidates=new_candidates,
                associated_index=association.rgb_region_index,
                measured_candidate_index=association.measured_candidate_index,
                alternatives=list(association.alternatives))
            records.append(result)
    finally:
        handoff.project_rgb_observation_volume = original_projection
    failures = [r for r in records if r['baseline_reason'] == 'no_cross_view_rgb_match']
    controls = [r for r in records if r['baseline_reason'] == 'rgb_only_associated']
    summary = dict(rgb_failures=len(failures), sdk_failure_outcomes=dict(Counter(r['sdk_reason'] for r in failures)),
        associated_controls=len(controls), sdk_control_outcomes=dict(Counter(r['sdk_reason'] for r in controls)),
        support_rejects=sum(r['baseline_reason'].startswith('support_inconsistent:') for r in records),
        max_vertex_shift_px=max(r.get('max_vertex_shift_px', 0) for r in records),
        unchanged_mask_set_count=sum({c['mask_sha256'] for c in r.get('proposal_candidates', [])} ==
            {c['mask_sha256'] for c in original_rows[r['source_stamp_ns']].get('proposal_candidates', [])} for r in records))
    result = dict(scope=__doc__, authority='offline model comparison; not a new source-qualified handoff or measured geometry',
        sdk_version=rs.__version__, sdk_module_path=rs.__file__, sdk_module_sha256=baseline.sha(rs.__file__),
        sdk_library_sha256=baseline.sha(WORK/'.runtime/realsense-2.56.5/lib/librealsense2.so.2.56.5'),
        recorded_intrinsics=info, source_report=str(baseline.ARCHIVE), source_report_sha256=baseline.sha(baseline.ARCHIVE),
        baseline_report=str(baseline.OUT), baseline_report_sha256=baseline.sha(baseline.OUT),
        reference_report=str(baseline.REFERENCE), reference_report_sha256=baseline.sha(baseline.REFERENCE),
        script_sha256=baseline.sha(__file__), baseline_script_sha256=baseline.sha(baseline.__file__),
        unchanged_inputs=['native RGB and aligned depth', 'TF', 'K and distortion coefficients',
            'support plane and footprint gates', 'appearance and candidate policies', '.80 coverage gate',
            'source stamps, SDK feedback brackets, generation, epoch and original reference identity'],
        limitations=['Isolates lens model only; support reconstruction retains original pinhole baseline.',
            'Eight constructed slab vertices remain an observation-interest model, not measured object points.',
            'Counterfactual association cannot retroactively authorize the completed episode.'],
        roundtrip=dict(pixel_count=len(roundtrip_pixels), max_abs_xy_px=errors.max(axis=0).tolist(),
            max_l2_px=float(np.max(np.linalg.norm(roundtrip-roundtrip_pixels, axis=1)))),
        source_sdk_vertices_base_m=vertices.tolist(), source_sdk_vertex_optical_depths_m=depths.tolist(),
        source_vertex_shift_mm=(1000*(vertices-np.asarray(reference.volume.vertices_base_m))).tolist(),
        policy=asdict(policy), summary=summary, records=records,
        production_changes=False, ros_io=False, robot_commands_sent=False)
    OUT.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(OUT), sha256=baseline.sha(OUT), summary=summary,
        roundtrip=result['roundtrip'], failure_appearance_candidates=[dict(source=r['source_stamp_ns'],
            reason=r['sdk_reason'], candidates=[c for c in r.get('proposal_candidates', []) if c['appearance_passes']])
            for r in failures]), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
