"""Offline native depth availability; no ROS or geometry authorization."""
from dataclasses import asdict
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from alicia_grasp_modes import tabletop
from alicia_grasp_modes.rgb_observation_handoff import HandoffPolicy, _appearance
from alicia_grasp_modes.rgb_observation_projection import projected_rgb_search_policy
from alicia_grasp_modes.rgb_observation_region import CenterSearchPolicy, propose_rgb_observation_regions
from alicia_grasp_modes.rgb_observation_volume import project_rgb_observation_vertices

BASE = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log')
CURRENT = BASE/'rgb_observation_bootstrap_20261004T114905Z/result.json'
REFERENCE = BASE/'rgb_acquisition_gateway_20261004T114259.948391Z_1541ec9e/result.json'
OUTPUT = Path(__file__).with_name('gen13_native_depth_diagnostic_20261004.json')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def quantiles(values):
    return dict(zip(('min', 'p10', 'p50', 'p90', 'max'),
        np.percentile(values, [0, 10, 50, 90, 100]).tolist())) if len(values) else None


def bbox(mask):
    return list(cv2.boundingRect(mask.astype(np.uint8))) if np.any(mask) else None


def spans(points):
    if len(points) < 3:
        return dict(count=len(points), pca_p98_p02_mm=None, camera_aabb_mm=None)
    centered = points.astype(float)-np.mean(points, axis=0)
    _, _, axes = np.linalg.svd(centered, full_matrices=False)
    oriented = centered@axes.T
    extent = np.sort(np.percentile(oriented, 98, axis=0)-np.percentile(oriented, 2, axis=0))[::-1]
    return dict(count=len(points), pca_p98_p02_mm=(1000.*extent).tolist(),
                camera_aabb_mm=(1000.*np.ptp(points, axis=0)).tolist())


def components(mask):
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=4)
    return [dict(pixels=int(stats[i, cv2.CC_STAT_AREA]), bbox_xywh=stats[i, :4].tolist())
            for i in sorted(range(1, count), key=lambda j:-stats[j, cv2.CC_STAT_AREA])]


def touching_components(binary, interest):
    count, labels, stats, _ = cv2.connectedComponentsWithStats(binary.astype(np.uint8), connectivity=4)
    indices = [int(i) for i in np.unique(labels[interest]) if i > 0]
    return [dict(full_component_pixels=int(stats[i, cv2.CC_STAT_AREA]),
        rgb_intersection_pixels=int(np.count_nonzero((labels == i) & interest)),
        bbox_xywh=stats[i, :4].tolist(), reaches_unchanged_60_point_gate=bool(stats[i, cv2.CC_STAT_AREA] >= 60))
        for i in indices]


def main():
    cv2.setNumThreads(1)
    archived = json.loads(CURRENT.read_text())
    reference = json.loads(REFERENCE.read_text())
    episode = reference['outcome']['episode']
    volume = json.loads(episode['reference_volume_json'])
    first = reference['frames']['initial']
    old = np.load(first['source_file'], allow_pickle=False)
    assert sha(first['source_file']) == first['source_file_sha256']
    old_hist, old_contour = _appearance(old['color_bgr'], old['region_mask'].astype(np.uint8))
    policy = HandoffPolicy()
    report = dict(
        scope='current_archived_native_depth_availability_only',
        ros_io=False, production_changes=False, robot_commands_sent=False,
        metric_geometry_authorized=False, synthetic_measured_points_added=0,
        source_reports=[dict(path=str(p), sha256=sha(p)) for p in (CURRENT, REFERENCE)],
        script_sha256=sha(__file__), tabletop_sha256=sha(tabletop.__file__),
        reference_id=episode['reference_id'], reference_source_stamp_ns=first['source_stamp_ns'],
        original_episode_deadline_ns=episode['deadline_ns'], original_reference_bbox=volume['bbox_xywh'],
        reference_appearance_mask_sha256=hashlib.sha256(old['region_mask'].tobytes()).hexdigest(),
        association_policy=asdict(policy), rows=[],
        limits=[
            'These four later frames are not the exact earlier action-handoff failure frames.',
            'Original RGB reference projection and appearance are used only for offline mask diagnosis; the expired episode is not revived.',
            'Current height is a signed camera-space distance to this frame measured support plane, not independently known physical object thickness.',
            'The frame camera-plane fit is independent of hand-eye; transforming that fitted plane to base and back cancels the transform.',
            'PCA spans describe only available native measurements; neither holes nor complete object geometry are inferred.',
            'No threshold, morphology, min_object_points=60 or measured-mask IoU=0.50 is relaxed.',
        ])
    for row in archived['rows']:
        source = Path(row['source_file'])
        assert sha(source) == row['source_file_sha256']
        data = np.load(source, allow_pickle=False)
        rgb, native = data['color_bgr'], data['depth_raw']
        for array, key in ((rgb, 'source_rgb_sha256'), (native, 'source_depth_sha256')):
            assert hashlib.sha256(np.ascontiguousarray(array).tobytes()).hexdigest() == row[key]
        depth = native.astype(np.float32)*float(data['depth_scale'])
        K, transform = data['K'], data['T_base_camera']
        config = tabletop.Config(**row['config'])
        matrix = np.array([[K[0], 0., K[2]], [0., K[1], K[3]], [0., 0., 1.]])
        projected = np.asarray(project_rgb_observation_vertices(volume['vertices_base_m'],
            T_base_camera=transform, camera_matrix=matrix, image_shape=depth.shape,
            working_depth_range_m=volume['working_depth_range_m']))
        search = projected_rgb_search_policy(CenterSearchPolicy(center_xy=(.5, .5),
            radius_fraction=config.center_radius_fraction), projected, depth.shape)
        proposals = propose_rgb_observation_regions(rgb, policy=search)
        hull = cv2.convexHull(projected.astype(np.float32)).reshape(-1, 2)
        projected_mask = np.zeros(depth.shape, np.uint8)
        cv2.fillConvexPoly(projected_mask, np.rint(hull).astype(np.int32), 1)
        alternatives = []
        for index, region in enumerate(proposals.candidates):
            mask = region.mask.astype(bool)
            hist, contour = _appearance(rgb, mask.astype(np.uint8))
            colour = float(np.sqrt(max(0., 1.-np.sum(np.sqrt(hist*old_hist)))))
            shape = float(cv2.matchShapes(old_contour, contour, cv2.CONTOURS_MATCH_I1, 0.))
            fraction = float(np.count_nonzero(mask & (projected_mask != 0))/mask.sum())
            alternatives.append(dict(index=index, bbox_xywh=list(region.bbox_xywh), pixels=int(mask.sum()),
                colour_distance=colour, contour_distance=shape, projected_fraction=fraction,
                score=.65*colour/policy.max_colour_distance+.35*shape/policy.max_contour_distance,
                eligible=(fraction >= policy.min_projected_fraction and colour <= policy.max_colour_distance
                          and shape <= policy.max_contour_distance)))
        matches = sorted((item for item in alternatives if item['eligible']), key=lambda item:item['score'])
        assert matches and (len(matches) == 1 or matches[1]['score']-matches[0]['score'] > policy.ambiguity_margin)
        mask = proposals.candidates[matches[0]['index']].mask.astype(bool)
        core = cv2.erode(mask.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
        x, y, w, h = bbox(mask)
        rectangle = np.zeros(depth.shape, bool); rectangle[y:y+h, x:x+w] = True
        segmentation = tabletop.segment(depth, K, transform, config)
        points = tabletop._project(depth, K)
        valid = np.isfinite(depth) & (depth >= config.depth_min_m) & (depth <= config.depth_max_m)
        vv, uu = np.indices(depth.shape)
        radius = np.hypot(uu-depth.shape[1]/2., vv-depth.shape[0]/2.)
        scale = min(depth.shape[1]/640., depth.shape[0]/480.)
        ring = valid & (radius >= 70*scale) & (radius <= segmentation.metrics['support_ring_outer_radius_px'])
        direct_plane, _, direct_residual, _ = tabletop._fit_plane(points[ring], config)
        plane_camera = transform.T@segmentation.plane_base
        above = np.einsum('ijk,k->ij', points, plane_camera[:3])+plane_camera[3]
        old_plane_camera = transform.T@np.asarray(volume['support_plane_base'])
        old_above = np.einsum('ijk,k->ij', points, old_plane_camera[:3])+old_plane_camera[3]
        roi = radius <= 155*scale
        raw_foreground = valid & roi & (above >= config.foreground_min_m) & (above <= config.foreground_max_m)
        opened = cv2.morphologyEx(raw_foreground.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3,3),np.uint8))
        opened &= (valid & (above >= config.foreground_min_m)).astype(np.uint8)
        opened = opened.astype(bool)
        scopes = {}
        for name, selected in (('rgb_bbox', rectangle), ('rgb_outline', mask), ('rgb_core_erode3x3', core)):
            seen = selected & valid
            foreground = raw_foreground & selected
            cleaned = opened & selected
            scopes[name] = dict(pixels=int(selected.sum()), bbox_xywh=bbox(selected),
                native_zero_depth=int(np.count_nonzero(selected & (native == 0))),
                valid_native_depth=int(seen.sum()), invalid_native_depth=int(np.count_nonzero(selected & ~valid)),
                missing_fraction=float(np.count_nonzero(selected & ~valid)/selected.sum()),
                within_segmentation_roi=int(np.count_nonzero(selected & roi)),
                depth_mm=quantiles(depth[seen]*1000.), height_to_current_plane_mm=quantiles(above[seen]*1000.),
                height_to_original_plane_mm=quantiles(old_above[seen]*1000.),
                height_bins=dict(below_zero=int(np.count_nonzero(seen & (above < 0))),
                    zero_to_2mm=int(np.count_nonzero(seen & (above >= 0) & (above < .002))),
                    two_to_4mm=int(np.count_nonzero(seen & (above >= .002) & (above < .004))),
                    four_to_120mm=int(np.count_nonzero(seen & (above >= .004) & (above <= .12))),
                    above_120mm=int(np.count_nonzero(seen & (above > .12)))),
                raw_ge_4mm_inside_roi=int(foreground.sum()),
                raw_ge_4mm_outside_roi=int(np.count_nonzero(seen & ~roi & (above >= .004) & (above <= .12))),
                production_opened_pixels=int(cleaned.sum()),
                raw_4_connected_components=components(foreground), opened_4_connected_components=components(cleaned),
                native_point_spans=spans(points[seen]), raw_foreground_spans=spans(points[foreground]),
                opened_foreground_spans=spans(points[cleaned]))
        candidates = []
        for candidate in segmentation.candidates:
            cmask = candidate.mask.astype(bool)
            candidates.append(dict(bbox_xywh=list(candidate.bbox), count=candidate.metrics['valid_points'],
                rgb_mask_intersection=int(np.count_nonzero(cmask & mask)),
                rgb_mask_iou=float(np.count_nonzero(cmask & mask)/np.count_nonzero(cmask | mask)),
                median_height_mm=candidate.metrics['median_height_m']*1000.,
                pca_p98_p02_mm=(candidate.extent_m*1000.).tolist()))
        report['rows'].append(dict(source_stamp_ns=row['source_stamp_ns'], source_npz=str(source), source_sha256=sha(source),
            config=asdict(config), accepted_bracket_span_ns=row['accepted_after'][0]-row['accepted_before'][0],
            after_original_deadline_sec=(row['source_stamp_ns']-int(episode['deadline_ns']))*1e-9,
            original_projection_pixels=projected.tolist(), rgb_search_policy=asdict(search),
            rgb_proposal_reason=proposals.reason, rgb_diagnostic_alternatives=alternatives,
            unique_offline_reference_appearance_match_index=matches[0]['index'],
            rgb_mask_sha256=hashlib.sha256(mask.tobytes()).hexdigest(),
            support_plane_base=segmentation.plane_base.tolist(), support_plane_camera=plane_camera.tolist(),
            direct_camera_fit_max_coefficient_difference=float(np.max(abs(direct_plane-plane_camera))),
            support_metrics=segmentation.metrics, current_plane_fit_residual_mm=direct_residual*1000.,
            original_plane_camera=old_plane_camera.tolist(),
            full_roi_raw_foreground=int(raw_foreground.sum()), full_roi_opened_foreground=int(opened.sum()),
            global_raw_components_touching_rgb=touching_components(raw_foreground, mask),
            global_opened_components_touching_rgb=touching_components(opened, mask),
            scopes=scopes, production_candidates=candidates,
            any_measured_candidate_reaches_required_rgb_iou=any(c['rgb_mask_iou'] >= .50 for c in candidates)))
    report['conclusion'] = dict(current_native_measurements_support_grasp=False,
        reason='Each matched RGB outline has substantial native depth holes; only 0/0/14/16 native points reach the unchanged 4 mm measured height gate, fewer than 60 even before opening. No production measured candidate overlaps the diagnostic RGB outline sufficiently.',
        cause_not_claimed='Neither physical object thickness nor the cause of unavailable/near-plane depth is inferred; no hand-eye error attribution.')
    OUTPUT.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(OUTPUT), sha256=sha(OUTPUT), bytes=OUTPUT.stat().st_size,
        summary=[dict(stamp=r['source_stamp_ns'],
            valid_outline=r['scopes']['rgb_outline']['valid_native_depth'],
            raw_4mm=r['scopes']['rgb_outline']['raw_ge_4mm_inside_roi'],
            opened=r['scopes']['rgb_outline']['production_opened_pixels'],
            global_opened_components=r['global_opened_components_touching_rgb'])
                 for r in report['rows']]), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
