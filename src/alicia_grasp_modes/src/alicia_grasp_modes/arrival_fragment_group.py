"""Current measured fragment proposals inside an already bound observation target.

Callers own the frozen arrival/station qualification. No historical pixels or
points enter the proposal; geometry is accepted only after current RGB ownership.
"""
from dataclasses import replace
import hashlib
import cv2
import numpy as np
from .tabletop import Candidate, _matching_candidates, _project, _measured_support_footprint, _depth_components
from .rgb_instance import _FrameRefinement, confidence_labels
from .rgb_observation_handoff import HandoffPolicy, _appearance, _mask
from .observation_appearance import match_observation_appearance


def recover_arrival_fragment_group(result, rgb, depth, intrinsics, transform, config,
        anchor, last, spatial_reference, histogram, contour, minimum_quality, *, rgb_valid=None,
        appearance_context=None):
    """Return same-frame final measured raw/refined pair, or unchanged old path."""
    if spatial_reference is None or _matching_candidates(result.candidates, anchor, last, config):
        return result, None
    candidates = result.candidates
    if isinstance(candidates, (list, tuple)) and len(candidates) < 2:
        return result, None
    if not isinstance(candidates, (list, tuple)) or len(candidates) > HandoffPolicy().maximum_candidates:
        raise ValueError('arrival_fragment_group_candidate_budget')
    if result.metrics.get('support_current_audit_available') is not True:
        raise ValueError('arrival_fragment_group_current_support_required')
    frame = _FrameRefinement(depth, intrinsics, transform, spatial_reference, config)
    points, spatial = frame.points, frame.spatial
    plane = np.asarray(candidates[0].metrics['support_plane_base'], dtype=float)
    plane_camera = np.asarray(transform).T @ plane
    heights = points @ plane_camera[:3] + plane_camera[3]
    valid = (np.isfinite(depth) & (depth >= config.depth_min_m) & (depth <= config.depth_max_m)
             & np.all(np.isfinite(points), axis=-1) & (heights >= config.foreground_min_m)
             & (heights <= config.foreground_max_m))
    members=[]; occupied=np.zeros(depth.shape, bool)
    for candidate in candidates:
        if (not isinstance(candidate, Candidate) or candidate.mask.shape != depth.shape
                or not np.array_equal(np.asarray(candidate.metrics['support_plane_base']), plane)):
            raise ValueError('arrival_fragment_group_source_mismatch')
        native = candidate.mask > 0
        if np.any(native & occupied):
            raise ValueError('arrival_fragment_group_native_overlap')
        occupied |= native
        current = native & valid & spatial['domain_mask']
        if np.any(current):
            members.append(current)
    if len(members) < 2:
        raise ValueError('arrival_fragment_group_insufficient_members')
    measured = np.logical_or.reduce(members)
    if np.count_nonzero(measured) < config.min_object_points:
        raise ValueError('arrival_fragment_group_insufficient_depth')
    xyz = points[measured].astype(float)
    base = xyz @ transform[:3, :3].T + transform[:3, 3]
    position_camera = np.median(xyz, axis=0)
    position_base = (transform @ np.r_[position_camera, 1.])[:3]
    centered = xyz - np.mean(xyz, axis=0)
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    oriented = centered @ vh.T
    extent = np.sort(np.percentile(oriented, 98, axis=0)-np.percentile(oriented, 2, axis=0))[::-1]
    if extent[0] > config.max_surface_extent_m:
        raise ValueError('arrival_fragment_group_extent_too_large')
    yy,xx=np.nonzero(measured)
    residual=float(candidates[0].metrics['support_residual_m'])
    quality=.65*max(0.,1.-residual/(2*config.support_threshold_m))+.35*min(1.,len(xyz)/400.)
    metrics=dict(candidates[0].metrics,valid_points=len(xyz),median_height_m=float(np.median(heights[measured])),
                 observed_surface_extent_m=extent.tolist())
    proposal=Candidate(measured.astype(np.uint8)*255,np.array([xx.mean(),yy.mean()]),
        (int(xx.min()),int(yy.min()),int(xx.max()-xx.min()+1),int(yy.max()-yy.min()+1)),
        position_camera,position_base,extent,float(position_camera[0]),quality,metrics,
        _measured_support_footprint(base,plane))
    if (quality < minimum_quality or not _matching_candidates([proposal],anchor,last,config)):
        raise ValueError('arrival_fragment_group_original_geometry_required')
    refined=frame.refine(rgb,proposal,rgb_valid=rgb_valid)
    # Recreate exactly the original first-small-crop hard core, not all native
    # edge pixels: RGB is allowed to discard unsupported stereo edge noise.
    margin=max(8,int(round(16*min(depth.shape[1]/640.,depth.shape[0]/480.))))
    x0=max(0,int(xx.min())-margin);x1=min(depth.shape[1],int(xx.max())+margin+1)
    y0=max(0,int(yy.min())-margin);y1=min(depth.shape[0],int(yy.max())+margin+1)
    sl=np.s_[y0:y1,x0:x1]
    core,_,_=frame.confidence_labels(measured[sl],points[sl],depth[sl],heights[sl],plane_camera[:3],config,bgr=rgb[sl])
    fullcore=np.zeros(depth.shape,bool);fullcore[sl]=core
    instance=_mask(refined.mask,depth.shape)>0
    member_rows=[]
    for member in members:
        count=int(np.count_nonzero(member & fullcore))
        retained=int(np.count_nonzero(member & fullcore & instance))
        if count and retained < .85*count:
            raise ValueError('arrival_fragment_group_member_not_owned')
        member_rows.append(dict(current_pixels=int(np.count_nonzero(member)),trusted_core_pixels=count,
            retained_trusted_core_pixels=retained,hard_identity_evidence=count>0,
            measured_mask_sha256=hashlib.sha256(np.ascontiguousarray(member,dtype=np.uint8).tobytes()).hexdigest()))
    final_measured=instance & valid
    fy,fx=np.nonzero(final_measured)
    if len(fx) != refined.metrics['valid_points']:
        raise ValueError('arrival_fragment_group_final_depth_mismatch')
    final_quality=.65*max(0.,1.-residual/(2*config.support_threshold_m))+.35*min(1.,len(fx)/400.)
    refined=replace(refined,quality=final_quality,metrics=dict(refined.metrics,
        median_height_m=float(np.median(heights[final_measured]))))
    if (not np.isfinite(refined.quality) or refined.quality < minimum_quality
            or not _matching_candidates([refined],anchor,last,config)):
        raise ValueError('arrival_fragment_group_final_geometry_required')
    appearance = match_observation_appearance(rgb,instance.astype(np.uint8),histogram,contour,
        rgb_valid=rgb_valid,appearance_context=appearance_context)
    colour,shape=appearance['colour_distance'],appearance['contour_distance']
    if not appearance['appearance_match']:
        raise ValueError('arrival_fragment_group_original_rgb_identity_required')
    # Even the tracker raw reference excludes every point discarded by RGB.
    # This is measured depth, not a completed silhouette or a synthetic hull.
    audit=dict(policy='current_fragments_in_frozen_observation_footprint',original_metric_matches=0,
        original_candidate_count=len(candidates),members=member_rows,provisional_measured_points=len(xyz),
        final_measured_points=len(fx),colour_distance=colour,contour_distance=shape,
        final_original_geometry_passed=True,historical_depth_points=0,appearance_identity=appearance)
    refined=replace(refined,metrics=dict(refined.metrics,observation_fragment_group=audit))
    final_raw=replace(refined,mask=final_measured.astype(np.uint8)*255,
        center_uv=np.array([fx.mean(),fy.mean()]),bbox=(int(fx.min()),int(fy.min()),int(fx.max()-fx.min()+1),int(fy.max()-fy.min()+1)))
    return replace(result,candidates=[final_raw]),(final_raw,refined)


def recover_arrival_matching_fragments(result, rgb, depth, intrinsics, transform, config,
        anchor, last, spatial_reference, histogram, contour, minimum_quality, *, rgb_valid=None,
        appearance_context=None):
    """Prove every original match observes the same current RGB instance.

    Unlike the raw-zero proposal above, no union seed is sent to GrabCut.
    Every competing full native seed independently passes the original RGB
    refinement. Mutual full ownership keeps the original intersection. Only
    measured diagonal connectivity plus a unique already-refined complete
    owner can recover a partial outline; no competitor is spatially discarded.
    Callers may keep their original ambiguity hold on a typed ValueError.
    """
    if spatial_reference is None:
        return result, None
    matches = _matching_candidates(result.candidates, anchor, last, config)
    if len(matches) < 2:
        return result, None
    try:
        return _recover_matching_fragments(result, rgb, depth, intrinsics, transform,
            config, anchor, last, spatial_reference, histogram, contour,
            minimum_quality, matches, rgb_valid=rgb_valid, appearance_context=appearance_context)
    except ValueError as exc:
        # Owner retirement, source deadlines and bracket gaps carry typed
        # control flow. Only ordinary geometric rejection is optional here.
        if type(exc) is not ValueError or str(exc).startswith('matching_fragment_'):
            raise
        raise ValueError('matching_fragment_original_gate_rejected: ' + str(exc)) from exc


def _recover_matching_fragments(result, rgb, depth, intrinsics, transform, config,
        anchor, last, spatial_reference, histogram, contour, minimum_quality,
        matches, *, rgb_valid, appearance_context=None):
    policy = HandoffPolicy()
    if not isinstance(result.candidates, (list, tuple)) or len(result.candidates) > policy.maximum_candidates:
        raise ValueError('matching_fragment_candidate_budget')
    if result.metrics.get('support_current_audit_available') is not True:
        raise ValueError('matching_fragment_current_support_required')
    frame = _FrameRefinement(depth, intrinsics, transform, spatial_reference, config)
    points = frame.points
    transform = np.asarray(transform)
    plane = np.asarray(matches[0][1].metrics['support_plane_base'], dtype=float)
    plane_camera = transform.T @ plane
    heights = np.einsum('ijk,k->ij', points, plane_camera[:3]) + plane_camera[3]
    margin = max(8, int(round(16*min(depth.shape[1]/640., depth.shape[0]/480.))))
    occupied = np.zeros(depth.shape, bool)
    cores, instances, independent = [], [], []
    for _, candidate in matches:
        if (not isinstance(candidate, Candidate) or candidate.mask.shape != depth.shape
                or not np.array_equal(np.asarray(candidate.metrics['support_plane_base']), plane)
                or not np.isfinite(candidate.quality) or candidate.quality < minimum_quality):
            raise ValueError('matching_fragment_native_candidate_invalid')
        native = _mask(candidate.mask, depth.shape) > 0
        if np.any(native & occupied):
            raise ValueError('matching_fragment_native_overlap')
        occupied |= native
        # The full native mask reaches the original refiner, without clipping
        # to the frozen footprint or promoting another candidate's hard core.
        refined = frame.refine(rgb, candidate, rgb_valid=rgb_valid)
        if (not np.isfinite(refined.quality) or refined.quality < minimum_quality
                or not _matching_candidates([refined], anchor, last, config)):
            raise ValueError('matching_fragment_independent_geometry_required')
        instance = _mask(refined.mask, depth.shape) > 0
        ys, xs = np.nonzero(native)
        sl = np.s_[max(0, int(ys.min())-margin):min(depth.shape[0], int(ys.max())+margin+1),
                   max(0, int(xs.min())-margin):min(depth.shape[1], int(xs.max())+margin+1)]
        core, _, _ = frame.confidence_labels(native[sl], points[sl], depth[sl],
            heights[sl], plane_camera[:3], config, bgr=rgb[sl])
        fullcore = np.zeros(depth.shape, bool)
        fullcore[sl] = core
        if np.count_nonzero(fullcore) < 25:  # Original first-search core gate.
            raise ValueError('matching_fragment_native_core_unproven')
        appearance = match_observation_appearance(rgb,instance.astype(np.uint8),histogram,contour,
            rgb_valid=rgb_valid,appearance_context=appearance_context)
        colour,shape=appearance['colour_distance'],appearance['contour_distance']
        if not appearance['appearance_match']:
            raise ValueError('matching_fragment_independent_rgb_identity_required')
        index = next(i for i, c in enumerate(result.candidates) if c is candidate)
        independent.append(dict(candidate_index=index, native_pixels=int(native.sum()),
            native_mask_sha256=hashlib.sha256(np.ascontiguousarray(native, dtype=np.uint8).tobytes()).hexdigest(),
            trusted_core_pixels=int(fullcore.sum()),
            trusted_core_outside_domain_pixels=int(np.count_nonzero(fullcore & ~frame.spatial['domain_mask'])),
            rgb_pixels=int(instance.sum()), colour_distance=colour, contour_distance=shape,
            rgb_metrics=dict(refined.metrics), appearance_identity=appearance))
        cores.append(fullcore)
        instances.append(instance)
    pairwise = []
    complete_owners = []
    for i, instance in enumerate(instances):
        retained = [int(np.count_nonzero(instance & core)) for core in cores]
        independent[i]['retained_native_cores'] = retained
        if all(count >= .85*np.count_nonzero(core) for count, core in zip(retained, cores)):
            complete_owners.append(i)
        for j in range(i):
            iou = float(np.count_nonzero(instance & instances[j]) / np.count_nonzero(instance | instances[j]))
            pairwise.append(dict(first=j, second=i, mask_iou=iou))
    all_owned = len(complete_owners) == len(instances)
    diagonal_recovery = None
    if not all_owned:
        # Preserve every original independent RGB/geometry check. A single
        # fully owning result is usable only when the original outlines agree
        # and current measured diagonal neighbors explain the depth split.
        if len(complete_owners) != 1 or any(row['mask_iou'] < .85 for row in pairwise):
            raise ValueError('matching_fragment_competitor_not_owned')
        measured_union = np.logical_or.reduce([c.mask > 0 for _, c in matches])
        groups = _depth_components(measured_union.astype(np.uint8), depth,
            config.min_object_points, config.depth_edge_m, connectivity=8)
        if len(groups) != 1 or not np.array_equal(groups[0], measured_union):
            raise ValueError('matching_fragment_competitor_not_owned')
        # No original nonmatching component, filled pixel or union hard seed
        # is used to bridge the matches. Select by complete measured ownership,
        # never by area/appearance score. This mask already passed refinement.
        selected = complete_owners[0]
        instance = instances[selected].copy()
        diagonal_recovery = dict(
            policy='unique_independent_complete_owner_of_diagonally_connected_measurements',
            selected_independent_index=selected, connectivity=8,
            matched_native_pixels=int(measured_union.sum()),
            connected_current_pixels=int(groups[0].sum()),
            nonmatching_candidates_used_as_bridge=0, added_depth_pixels=0,
            union_hard_seed_used=False, additional_rgb_refinements=0)
    else:
        if any(row['mask_iou'] < .85 for row in pairwise):
            raise ValueError('matching_fragment_independent_outlines_disagree')
        # Existing all-owner success keeps its symmetric intersection exactly.
        instance = _mask(np.logical_and.reduce(instances), depth.shape) > 0
    if any(np.count_nonzero(instance & core) < .85*np.count_nonzero(core) for core in cores):
        raise ValueError('matching_fragment_consensus_core_not_owned')
    if rgb_valid is not None and np.any(instance & ~rgb_valid):
        raise ValueError('matching_fragment_consensus_rgb_unobserved')
    valid = (np.isfinite(depth) & (depth >= config.depth_min_m) & (depth <= config.depth_max_m)
        & np.all(np.isfinite(points), axis=-1) & (heights >= config.foreground_min_m)
        & (heights <= config.foreground_max_m))
    measured = instance & valid
    support_count = int(np.count_nonzero(measured & frame.spatial['domain_mask']))
    if support_count < config.min_object_points or int(instance.sum()) > 4.*support_count:
        raise ValueError('matching_fragment_consensus_current_support_required')
    xyz = points[measured].astype(float)
    position_camera = np.median(xyz, axis=0)
    position_base = (transform @ np.r_[position_camera, 1.])[:3]
    if any(np.linalg.norm(position_base-c.position_base) > config.association_distance_m for _, c in matches):
        raise ValueError('matching_fragment_consensus_shifted_from_native')
    centered = xyz-np.mean(xyz, axis=0)
    _, _, axes = np.linalg.svd(centered, full_matrices=False)
    oriented = centered @ axes.T
    extent = np.sort(np.percentile(oriented, 98, axis=0)-np.percentile(oriented, 2, axis=0))[::-1]
    if extent[0] > config.max_surface_extent_m:
        raise ValueError('matching_fragment_consensus_extent_too_large')
    residual = max(float(c.metrics['support_residual_m']) for _, c in matches)
    quality = .65*max(0., 1.-residual/(2*config.support_threshold_m))+.35*min(1., len(xyz)/400.)
    if not np.isfinite(quality) or quality < minimum_quality:
        raise ValueError('matching_fragment_consensus_rgb_identity_required')
    appearance = match_observation_appearance(rgb,instance.astype(np.uint8),histogram,contour,
        rgb_valid=rgb_valid,appearance_context=appearance_context)
    colour,shape=appearance['colour_distance'],appearance['contour_distance']
    if not appearance['appearance_match']:
        raise ValueError('matching_fragment_consensus_rgb_identity_required')
    audit = dict(policy=('all_native_matches_independently_own_current_rgb_instance'
                 if diagonal_recovery is None else diagonal_recovery['policy']),
        original_metric_matches=len(matches), original_candidate_count=len(result.candidates),
        independent_refinements=independent, pairwise_outlines=pairwise,
        consensus_policy=('intersection_of_all_independent_current_rgb_instances' if diagonal_recovery is None
                          else diagonal_recovery['policy']),
        diagonal_recovery=diagonal_recovery,
        **({'intersection_rgb_pixels': int(instance.sum())} if diagonal_recovery is None
           else {'selected_instance_rgb_pixels': int(instance.sum())}),
        final_measured_points=len(xyz),
        current_measured_support_pixels=support_count,
        colour_distance=colour, contour_distance=shape, historical_depth_points=0,
        historical_rgb_pixels=0,
        all_original_matches_accounted_for=True, final_original_geometry_passed=True,
        appearance_identity=appearance)
    metrics = dict(matches[0][1].metrics, valid_points=len(xyz),
        median_height_m=float(np.median(heights[measured])), support_residual_m=residual,
        observed_surface_extent_m=extent.tolist(), observation_fragment_group=audit)
    yy, xx = np.nonzero(instance)
    base = xyz @ transform[:3, :3].T + transform[:3, 3]
    refined = Candidate(instance.astype(np.uint8)*255, np.array([xx.mean(), yy.mean()]),
        (int(xx.min()), int(yy.min()), int(np.ptp(xx)+1), int(np.ptp(yy)+1)),
        position_camera, position_base, extent, float(position_camera[0]), quality,
        metrics, _measured_support_footprint(base, plane))
    if not _matching_candidates([refined], anchor, last, config):
        raise ValueError('matching_fragment_final_original_geometry_required')
    yy, xx = np.nonzero(measured)
    final_raw = replace(refined, mask=measured.astype(np.uint8)*255,
        center_uv=np.array([xx.mean(), yy.mean()]),
        bbox=(int(xx.min()), int(yy.min()), int(np.ptp(xx)+1), int(np.ptp(yy)+1)))
    return replace(result, candidates=[final_raw]), (final_raw, refined)
