"""Recover a whole current measured source component split by rectification.

Source-map provenance supplies sampling adjacency only. Fixed station identity,
ordinary RGB/depth geometry, positioning and box qualification stay with callers.
No historical image is accepted as geometry and no depth is filled.
"""
from dataclasses import dataclass, replace
import hashlib
import numpy as np
from .rgb_instance import _FrameRefinement
from .rgb_observation_handoff import HandoffPolicy
from .tabletop import Candidate, _depth_components, _matching_candidates, _measured_support_footprint
from .source_sampling_topology import SourceSamplingTopology

@dataclass(frozen=True)
class SourceConnectedOwner:
    owner: Candidate
    audit: dict


def _sha(value):
    return hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest()


def _reject_single_pixel_bridge(mask, depth, cores, edge, check_current, *, connectivity=4):
    """Reject articulation pixels separating two independently trusted cores.

    Graph edges follow the explicitly selected sampling grid. A cut
    must separate at least the original 25-pixel trustworthy core minimum on
    both sides and involve distinct native seeds. No pixel is added or filled.
    """
    yy, xx = np.nonzero(mask)
    ids = np.full(mask.shape, -1, dtype=np.int32)
    ids[yy, xx] = np.arange(len(xx))
    adjacency = [[] for _ in xx]
    if connectivity != 4:
        raise ValueError('current_fragment_graph_connectivity_invalid')
    directions = ((0, 1), (1, 0))
    for dy, dx in directions:
        y2, x2 = yy + dy, xx + dx
        keep = (y2 >= 0) & (y2 < mask.shape[0]) & (x2 >= 0) & (x2 < mask.shape[1])
        first = np.flatnonzero(keep)
        second = ids[y2[keep], x2[keep]]
        selected = second >= 0
        first, second = first[selected], second[selected]
        selected = np.abs(depth[yy[first], xx[first]] - depth[yy[second], xx[second]]) <= edge
        for a, b in zip(first[selected].tolist(), second[selected].tolist()):
            adjacency[a].append(b)
            adjacency[b].append(a)
    count = len(xx)
    if not count:
        raise ValueError('current_fragment_empty_connected_group')
    own = np.stack([core[yy, xx].astype(np.int32) for core in cores], axis=1)
    total = own.sum(axis=0)
    subtree = own.copy()
    # These are scalar DFS bookkeeping, not vector arithmetic. Python integer
    # lists avoid thousands of NumPy scalar conversions in the same graph walk.
    entry = [-1] * count
    low = [0] * count
    parent = [-1] * count
    entry[0] = low[0] = 0
    stack, clock = [(0, 0)], 1
    while stack:
        vertex, next_edge = stack[-1]
        if next_edge < len(adjacency[vertex]):
            neighbor = adjacency[vertex][next_edge]
            stack[-1] = (vertex, next_edge + 1)
            if entry[neighbor] < 0:
                parent[neighbor] = vertex
                entry[neighbor] = low[neighbor] = clock
                clock += 1
                if clock % 512 == 0:
                    check_current()
                stack.append((neighbor, 0))
            elif neighbor != parent[vertex]:
                low[vertex] = min(low[vertex], entry[neighbor])
        else:
            stack.pop()
            p = parent[vertex]
            if p >= 0:
                if low[vertex] >= entry[p]:
                    first = subtree[vertex]
                    rest = total - first - own[p]
                    if (first.sum() >= 25 and rest.sum() >= 25
                            and any(first[i] > 0 and rest[j] > 0
                                    for i in range(len(cores)) for j in range(len(cores)) if i != j)):
                        raise ValueError('current_fragment_single_pixel_measured_bridge')
                subtree[p] += subtree[vertex]
                low[p] = min(low[p], low[vertex])
    if clock != count:
        raise ValueError('current_fragment_measured_group_disconnected')
    check_current()


def recover_source_connected_fragment_owner(result, rgb, depth, intrinsics, transform,
        config, anchor, last, spatial_reference, primary_refined, minimum_quality, *,
        source_topology, camera, source_stamp_ns, rgb_valid=None, check_current,
        record_stage=None):
    """Restore a complete measured proposal split only by canvas resampling.

    The caller restricts this path to the unchanged same-station primary
    outline failure. The only extra adjacency is the exact original source
    grid's four-neighbour measured-depth graph. No template enters this API.
    Per-fragment RGB expansions are not independent object hypotheses here:
    one restored current measurement is passed through the original refiner.
    """
    def stage(name):
        if record_stage is not None:
            record_stage(name)
    stage('source_scope_and_map')
    matches = _matching_candidates(result.candidates, anchor, last, config)
    if spatial_reference is None or len(matches) != 1:
        return None
    if not callable(check_current):
        raise ValueError('current_fragment_current_source_check_required')
    check_current()
    if not isinstance(source_topology, SourceSamplingTopology):
        raise ValueError('current_source_topology_required')
    source_topology.require_source(camera, depth.shape, intrinsics, source_stamp_ns)
    if (not isinstance(result.candidates, (list, tuple))
            or len(result.candidates) > HandoffPolicy().maximum_candidates):
        raise ValueError('current_fragment_candidate_budget')
    if len(result.candidates) < 2:
        return None
    if result.metrics.get('support_current_audit_available') is not True:
        raise ValueError('current_fragment_current_support_required')
    if np.any(depth[source_topology.output_to_source < 0] != 0):
        raise ValueError('current_source_topology_unmapped_depth_present')
    primary = matches[0][1]
    plane = np.asarray(result.plane_base, dtype=float)
    occupied = np.zeros(depth.shape, bool)
    for candidate in result.candidates:
        if (not isinstance(candidate, Candidate) or candidate.mask.shape != depth.shape
                or not np.array_equal(np.asarray(candidate.metrics['support_plane_base']), plane)):
            raise ValueError('current_fragment_native_source_mismatch')
        if np.any(occupied & (candidate.mask > 0)):
            raise ValueError('current_fragment_native_overlap')
        occupied |= candidate.mask > 0
    if np.any(occupied & (source_topology.output_to_source < 0)):
        raise ValueError('current_fragment_native_unmapped_pixel')
    stage('source_four_neighbor_component')
    source_depth = source_topology.source_array(depth)
    source_occupied = source_topology.source_array(occupied)
    components = _depth_components(source_occupied.astype(np.uint8), source_depth,
        config.min_object_points, config.depth_edge_m, connectivity=4)
    source_primary = source_topology.source_array(primary.mask > 0)
    selected = [mask for mask in components if np.any(mask & source_primary)]
    if len(selected) != 1 or np.any(source_primary & ~selected[0]):
        raise ValueError('current_fragment_primary_component_invalid')
    source_group = selected[0]
    group = source_topology.output_mask(source_group)
    members = [(i, c) for i, c in enumerate(result.candidates) if np.any((c.mask > 0) & group)]
    if len(members) < 2:
        return None
    stage('current_projection_and_native_cores')
    frame = _FrameRefinement(depth, intrinsics, transform, spatial_reference, config)
    plane_camera = np.asarray(transform).T @ plane
    heights = np.einsum('ijk,k->ij', frame.points, plane_camera[:3]) + plane_camera[3]
    valid = (np.isfinite(depth) & (depth >= config.depth_min_m) & (depth <= config.depth_max_m)
        & np.all(np.isfinite(frame.points), axis=-1)
        & (heights >= config.foreground_min_m) & (heights <= config.foreground_max_m))
    if np.any(group & ~valid):
        raise ValueError('current_fragment_group_has_unmeasured_or_support_pixels')
    margin = max(8, int(round(16 * min(depth.shape[1]/640., depth.shape[0]/480.))))
    cores, rows = [], []
    for index, candidate in members:
        check_current()
        native = candidate.mask > 0
        if np.any(native & ~group):
            raise ValueError('current_fragment_native_component_split')
        if not np.isfinite(candidate.quality) or candidate.quality < minimum_quality:
            raise ValueError('current_fragment_native_quality_insufficient')
        yy, xx = np.nonzero(native)
        sl = np.s_[max(0, int(yy.min())-margin):min(depth.shape[0], int(yy.max())+margin+1),
                   max(0, int(xx.min())-margin):min(depth.shape[1], int(xx.max())+margin+1)]
        core, _, evidence = frame.confidence_labels(native[sl], frame.points[sl], depth[sl],
            heights[sl], plane_camera[:3], config, bgr=rgb[sl])
        fullcore = np.zeros(depth.shape, bool)
        fullcore[sl] = core
        if np.count_nonzero(fullcore) < 25:
            raise ValueError('current_fragment_native_core_unproven')
        cores.append(fullcore)
        rows.append(dict(candidate_index=index, native_pixels=int(native.sum()),
            native_mask_sha256=_sha(native.astype(np.uint8)), trusted_core_pixels=int(fullcore.sum()),
            trusted_core_sha256=_sha(fullcore.astype(np.uint8)), seed_evidence=evidence))
    stage('raw_source_component_evidence')
    # Native candidates were acceleration-filtered against the anchor AABB
    # and morphologically opened. Neither operation proves absence of another
    # current object. Inspect raw current foreground with the original 60-point
    # component minimum and depth edge, without that candidate prefilter.
    raw_components = _depth_components(source_topology.source_array(valid).astype(np.uint8),
        source_depth, config.min_object_points, config.depth_edge_m, connectivity=4)
    owners = [mask for mask in raw_components if np.any(mask & source_group)]
    if len(owners) != 1 or np.any(source_group & ~owners[0]):
        raise ValueError('current_fragment_raw_source_component_invalid')
    raw_owner = source_topology.output_mask(owners[0])
    raw_only = raw_owner & ~group
    extra_cores = []
    # Apply the same local seed semantics as each original native fragment.
    # A global owner-height percentile can conceal a smaller independent top
    # even when its own current plane supports a trustworthy local interior.
    extra_regions = _depth_components(source_topology.source_array(raw_only).astype(np.uint8),
        source_depth, config.min_object_points, config.depth_edge_m, connectivity=4)
    for region in extra_regions:
        check_current()
        output_region = source_topology.output_mask(region)
        yy, xx = np.nonzero(output_region)
        sl = np.s_[max(0, int(yy.min())-margin):min(depth.shape[0], int(yy.max())+margin+1),
                   max(0, int(xx.min())-margin):min(depth.shape[1], int(xx.max())+margin+1)]
        current_core, _, _ = frame.confidence_labels(output_region[sl], frame.points[sl], depth[sl],
            heights[sl], plane_camera[:3], config, bgr=rgb[sl])
        extra_core = np.zeros(depth.shape, bool)
        extra_core[sl] = current_core & output_region[sl]
        if np.count_nonzero(extra_core) >= 25:
            extra_cores.append(extra_core)
    graph_cores = [source_topology.source_array(core) for core in cores]
    graph_cores.extend(source_topology.source_array(core) for core in extra_cores)
    check_current()
    stage('source_four_neighbor_articulation')
    # Raw-only nodes are graph evidence, never added to the RGB seed/proposal.
    # This catches a neighbor joined through a bridge removed by morphology.
    _reject_single_pixel_bridge(owners[0], source_depth,
        graph_cores, config.depth_edge_m,
        check_current, connectivity=4)
    # Original complete-current proposal arithmetic used by arrival_fragment_group.
    # Only its measurement entity changes: all samples of one original component.
    stage('complete_raw_geometry')
    xyz = frame.points[group].astype(float)
    base = xyz @ np.asarray(transform)[:3, :3].T + np.asarray(transform)[:3, 3]
    position = np.median(xyz, axis=0)
    centered = xyz - xyz.mean(axis=0)
    _, _, axes = np.linalg.svd(centered, full_matrices=False)
    oriented = centered @ axes.T
    extent = np.sort(np.percentile(oriented, 98, axis=0)-np.percentile(oriented, 2, axis=0))[::-1]
    if extent[0] > config.max_surface_extent_m:
        raise ValueError('current_fragment_complete_extent_too_large')
    residual = float(primary.metrics['support_residual_m'])
    quality = .65*max(0., 1.-residual/(2*config.support_threshold_m)) + .35*min(1., len(xyz)/400.)
    yy, xx = np.nonzero(group)
    proposal = Candidate(group.astype(np.uint8)*255, np.array([xx.mean(), yy.mean()]),
        (int(xx.min()), int(yy.min()), int(np.ptp(xx)+1), int(np.ptp(yy)+1)),
        position, (np.asarray(transform) @ np.r_[position, 1.])[:3], extent,
        float(position[0]), quality, dict(primary.metrics, valid_points=len(xyz),
            median_height_m=float(np.median(heights[group])), observed_surface_extent_m=extent.tolist()),
        _measured_support_footprint(base, plane))
    if not np.isfinite(quality) or quality < minimum_quality or not _matching_candidates([proposal], anchor, last, config):
        raise ValueError('current_fragment_complete_original_geometry_required')
    check_current()
    stage('complete_original_rgb_refinement')
    refined = frame.refine(rgb, proposal, rgb_valid=rgb_valid)
    check_current()
    stage('complete_current_ownership_and_geometry')
    # An independent measured source component is not part of this restored
    # proposal. Fixed-template tolerance cannot authorize RGB growth into it,
    # even when that other object is small or has the same surface colour.
    if np.any((refined.mask > 0) & occupied & ~group):
        raise ValueError('current_fragment_other_measured_component_overlap')
    source_output = source_topology.source_array(refined.mask > 0)
    foreign_count = 0
    for component in raw_components:
        if component is owners[0]:
            continue
        foreign_count += 1
        if np.any(component & source_output):
            raise ValueError('current_fragment_foreign_raw_source_component_overlap')
    for core, row in zip(cores, rows):
        fraction = float(np.count_nonzero((refined.mask > 0) & core)/np.count_nonzero(core))
        row['retained_fraction'] = fraction
        if fraction < .85:
            raise ValueError('current_fragment_native_core_not_owned')
    final = (refined.mask > 0) & valid
    quality = .65*max(0., 1.-residual/(2*config.support_threshold_m)) + .35*min(1., int(final.sum())/400.)
    if not np.any(final):
        raise ValueError('current_fragment_final_measurement_empty')
    refined = replace(refined, quality=quality, metrics=dict(refined.metrics,
        median_height_m=float(np.median(heights[final]))))
    if not np.isfinite(quality) or quality < minimum_quality or not _matching_candidates([refined], anchor, last, config):
        raise ValueError('current_fragment_complete_final_geometry_required')
    if rgb_valid is not None and np.any((refined.mask > 0) & ~rgb_valid):
        raise ValueError('current_fragment_boundary_rgb_unobserved')
    check_current()
    # No partial-outline tolerance is used. The unchanged raw-depth/RGB/box
    # and positioning consumers still qualify ordinary object uncertainty.
    stage('complete')
    return SourceConnectedOwner(refined, dict(
        policy='exact_source_four_neighbor_complete_measurement', original_metric_matches=1,
        native_members=rows, source_topology_contract_digest=source_topology.contract_digest,
        source_topology_map_sha256=source_topology.map_sha256, measured_connectivity=4,
        connectivity_raster='pre_rectification_aligned_color', measured_depth_edge_m=float(config.depth_edge_m),
        measured_component_pixels=int(group.sum()), measured_component_sha256=_sha(group.astype(np.uint8)),
        raw_foreground_component_points=int(owners[0].sum()),
        raw_only_regions_checked=len(extra_regions),
        raw_only_trusted_core_points=[int(core.sum()) for core in extra_cores],
        foreign_raw_foreground_components_checked=foreign_count,
        articulation_guard_passed=True, minimum_native_core_retention=.85,
        partial_outline_tolerance_used=False, native_union_used_as_rgb_seed=True,
        original_source_component_restored=True, historical_pixels_used=0, depth_filled=False,
        metric_geometry_authority=False, motion_authorized=False))
