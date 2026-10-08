"""Offline native-depth local-ring diagnostics; no ROS or admission authority."""
from dataclasses import asdict
from collections import Counter
from pathlib import Path
import hashlib
import json
import os
import sys

import cv2
import numpy as np

from alicia_grasp_modes.tabletop import Config, _fit_plane, _project, segment
from alicia_grasp_modes.rgb_observation_region import CenterSearchPolicy, propose_rgb_observation_regions


def sha(value):
    return hashlib.sha256(value).hexdigest()


def stats(values):
    values = np.asarray(values, float)
    if not len(values):
        return None
    median = float(np.median(values))
    return dict(count=len(values), median_mm=median*1000.,
        mad_mm=float(np.median(np.abs(values-median)))*1000.,
        mad_normal_scaled_mm=float(1.4826*np.median(np.abs(values-median)))*1000.,
        quantiles_mm=np.percentile(values*1000., [0, 5, 25, 50, 75, 95, 100]).tolist())


def block_medians(mask, heights, block_ids):
    ids = block_ids[mask]
    values = heights[mask]
    return np.asarray([np.median(values[ids == block]) for block in np.unique(ids)
                       if np.count_nonzero(ids == block) >= 4])


def quadrants(mask, xx, yy, center):
    counts = [int(np.count_nonzero(mask & (xx < center[0] if left else xx >= center[0])
        & (yy < center[1] if top else yy >= center[1])))
        for left in (True, False) for top in (True, False)]
    return dict(counts=counts, at_least_20_pixels=sum(n >= 20 for n in counts))


def main():
    cv2.setNumThreads(1)
    archive = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/rgb_observation_bootstrap_20261004T034110Z')
    source = archive/'result.json'
    document = json.loads(source.read_text())
    rows = []
    for frame_index, record in enumerate(document['rows']):
        rawfile = Path(record['source_file'])
        assert sha(rawfile.read_bytes()) == record['source_file_sha256']
        raw = np.load(rawfile, allow_pickle=False)
        rgb, native = raw['color_bgr'], raw['depth_raw']
        assert sha(rgb.tobytes()) == record['source_rgb_sha256']
        assert sha(native.tobytes()) == record['source_depth_sha256']
        depth = native.astype(np.float32)*float(raw['depth_scale'])
        config = Config(**record['config']); transform = raw['T_base_camera']
        valid = np.isfinite(depth)&(depth >= config.depth_min_m)&(depth <= config.depth_max_m)
        points = _project(depth, tuple(raw['K']))
        segmentation = segment(depth, tuple(raw['K']), transform, config)
        global_camera_plane = transform.T@segmentation.plane_base
        global_height = np.einsum('ijk,k->ij', points, global_camera_plane[:3])+global_camera_plane[3]
        proposals = propose_rgb_observation_regions(rgb,
            policy=CenterSearchPolicy((.5, .5), radius_fraction=config.center_radius_fraction), depth_valid=valid)
        xx, yy = np.meshgrid(np.arange(depth.shape[1]), np.arange(depth.shape[0]))
        blocks = (yy//8)*((depth.shape[1]+7)//8)+xx//8
        split = (xx//8+yy//8)%2
        excluded = np.zeros(depth.shape, bool)
        for region in proposals.candidates:
            excluded |= region.mask
        # Every RGB hypothesis is excluded, including nested masks and nearby
        # competitors. A ring never uses its object's interior to fit its plane.
        excluded_guard = cv2.dilate(excluded.astype(np.uint8), np.ones((7, 7), np.uint8)) > 0
        current = dict(frame_index=frame_index, source_stamp_ns=record['source_stamp_ns'],
            source_id=record['source_id'], source_file_sha256=record['source_file_sha256'],
            source_rgb_sha256=record['source_rgb_sha256'], source_depth_sha256=record['source_depth_sha256'],
            exact_pixels_verified=True, global_support_plane_base=segmentation.plane_base.tolist(),
            global_support_metrics=segmentation.metrics, candidates=[])
        for candidate_index, region in enumerate(proposals.candidates):
            mask = region.mask
            interior = cv2.erode(mask.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
            distance = cv2.distanceTransform((~mask).astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
            candidate = dict(candidate_index=candidate_index, bbox_xywh=region.bbox_xywh,
                centroid_xy=region.centroid_xy, area_px=region.area_px,
                rgb_mask_sha256=sha(mask.tobytes()), rgb_score=region.score,
                valid_pixels=int(np.count_nonzero(mask&valid)),
                global_height=stats(global_height[mask&valid]), rings=[])
            for radius in (8, 16, 24):
                ring = (distance > 3.)&(distance <= radius)&~excluded_guard&valid
                ring_row = dict(outer_distance_px=radius, excluded_guard_px=3,
                    ring_pixels=int(ring.sum()), ring_quadrants=quadrants(ring, xx, yy, region.centroid_xy),
                    ring_uv_span_px=[int(np.ptp(xx[ring])), int(np.ptp(yy[ring]))] if np.any(ring) else None,
                    folds=[])
                for parity in (0, 1):
                    training = ring&(split == parity); held_out = ring&(split != parity)
                    fold = dict(training_parity=parity, training_pixels=int(training.sum()),
                        held_out_pixels=int(held_out.sum()),
                        training_quadrants=quadrants(training, xx, yy, region.centroid_xy),
                        held_out_quadrants=quadrants(held_out, xx, yy, region.centroid_xy))
                    try:
                        local_plane, ratio, residual, sampled = _fit_plane(points[training], config)
                        if local_plane[:3]@global_camera_plane[:3] < 0:
                            local_plane = -local_plane
                        heights = np.einsum('ijk,k->ij', points, local_plane[:3])+local_plane[3]
                        observed = mask&valid
                        target_blocks = block_medians(interior&valid, heights, blocks)
                        support_blocks = block_medians(held_out, heights, blocks)
                        median_delta = float(np.median(heights[observed])-np.median(heights[held_out]))
                        interior_delta = (float(np.median(heights[interior&valid])-np.median(heights[held_out]))
                                          if np.any(interior&valid) else None)
                        bootstrap = None
                        if len(target_blocks) >= 3 and len(support_blocks) >= 3:
                            rng = np.random.RandomState(431)
                            target_boot = np.median(target_blocks[rng.randint(len(target_blocks), size=(512,len(target_blocks)))],axis=1)
                            support_boot = np.median(support_blocks[rng.randint(len(support_blocks), size=(512,len(support_blocks)))],axis=1)
                            bootstrap = np.percentile((target_boot-support_boot)*1000.,[2.5,50,97.5]).tolist()
                        delta = local_plane-global_camera_plane
                        fold.update(fit_succeeded=True, support_source='independent_local_ring_training_pixels',
                            training_mask_sha256=sha(training.tobytes()), held_out_mask_sha256=sha(held_out.tobytes()),
                            local_plane_camera=local_plane.tolist(), fitted_inlier_ratio=ratio,
                            fitted_inlier_rmse_mm=residual*1000., fitted_sample_count=sampled,
                            held_out_residual=stats(heights[held_out]), object_height=stats(heights[observed]),
                            interior_height=stats(heights[interior&valid]), median_delta_from_heldout_mm=median_delta*1000.,
                            interior_median_delta_from_heldout_mm=None if interior_delta is None else interior_delta*1000.,
                            maximum_local_global_plane_distance_on_object_mm=float(np.max(np.abs(
                                np.einsum('ij,j->i',points[observed],delta[:3])+delta[3])))*1000.,
                            support_normal_angle_deg=float(np.rad2deg(np.arccos(np.clip(local_plane[:3]@global_camera_plane[:3],-1.,1.)))),
                            object_8x8_blocks=len(target_blocks), held_out_8x8_blocks=len(support_blocks),
                            empirical_block_bootstrap_difference_mm=bootstrap)
                    except ValueError as exc:
                        fold.update(fit_succeeded=False, reason=str(exc))
                    ring_row['folds'].append(fold)
                candidate['rings'].append(ring_row)
            current['candidates'].append(candidate)
        rows.append(current)
    summary=[]
    # Positions here label the already recorded candidates for diagnostic
    # comparison only. No pixel address enters the fit or a production policy.
    for rank in range(10):
        selected=[c for row in rows for c in row['candidates'] if c['candidate_index']==rank]
        if not selected: continue
        folds=[fold for c in selected for ring in c['rings'] if ring['outer_distance_px']==16
               for fold in ring['folds'] if fold.get('fit_succeeded')]
        summary.append(dict(proposal_rank=rank, frames=len(selected),bbox_first=selected[0]['bbox_xywh'],
            fitted_folds=len(folds),
            contrasts_mm=[min(f['median_delta_from_heldout_mm'] for f in folds),
                float(np.median([f['median_delta_from_heldout_mm'] for f in folds])),
                max(f['median_delta_from_heldout_mm'] for f in folds)] if folds else None,
            held_out_mad_mm=[min(f['held_out_residual']['mad_normal_scaled_mm'] for f in folds),
                float(np.median([f['held_out_residual']['mad_normal_scaled_mm'] for f in folds])),
                max(f['held_out_residual']['mad_normal_scaled_mm'] for f in folds)] if folds else None,
            object_blocks=[min(f['object_8x8_blocks'] for f in folds),max(f['object_8x8_blocks'] for f in folds)] if folds else None,
            held_out_quadrants=[min(f['held_out_quadrants']['at_least_20_pixels'] for f in folds),
                max(f['held_out_quadrants']['at_least_20_pixels'] for f in folds)] if folds else None))
    report=dict(source=str(source), source_sha256=sha(source.read_bytes()),
        source_recording_implementation=document['implementation'], diagnostic_script_sha256=sha(Path(__file__).read_bytes()),
        implementation=dict(python=sys.version, numpy=np.__version__, opencv=cv2.__version__, opencv_threads=cv2.getNumThreads()),
        method=dict(ring_outer_pixel_distances=[8,16,24],guard_pixels=3,spatial_block_width_pixels=8,
            train_validation='two_disjoint_checkerboard_parities_each_fitted_independently',
            excluded='all_current_rgb_proposal_masks_dilated_by_three_pixels',
            bootstrap='512_block_median_resamples_for_diagnostic_repeatability_only_not_sensor_systematic_error_bound'),
        robot_commands_sent=False, thresholds_modified=False, points_synthesized=0,
        geometry_authorized=False, statistical_admission_calibrated=False,
        summary=summary, rows=rows)
    print(json.dumps(summary,ensure_ascii=False))
    if '--write' in sys.argv:
        if not os.path.ismount('/media/zhuyupei/SanStand'):
            raise RuntimeError('SanStand_not_mounted_no_fallback')
        target=archive/'independent_local_support_diagnostic.json'
        if target.exists():raise RuntimeError('diagnostic_report_already_exists')
        target.write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
        print('saved',target,'sha256',sha(target.read_bytes()))


if __name__=='__main__':main()
