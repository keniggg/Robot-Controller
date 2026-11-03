"""Recover a whole current measured source component split by rectification.

Source-map provenance supplies sampling adjacency only. Fixed station identity,
ordinary RGB/depth geometry, positioning and box qualification stay with callers.
No historical image is accepted as geometry and no depth is filled.
"""
from copy import deepcopy
from dataclasses import asdict, dataclass, field, replace
import hashlib
import json
import numpy as np
from .rgb_instance import _FrameRefinement
from .rgb_observation_handoff import HandoffPolicy
from .tabletop import Candidate, Segmentation, _depth_components, _matching_candidates, _measured_support_footprint
from .source_sampling_topology import SourceSamplingTopology
from .measured_mask_seeds import MeasuredSeedCoreInsufficient


class CurrentMeasurementRejected(ValueError):
    """A measured current-frame rejection; never a claim of target identity loss.

    Only explicit numerical producers below construct supported subclasses.
    Waiting, if any, requires a separately validated existing station owner.
    """


class RawOnlySeedInsufficient(CurrentMeasurementRejected):
    """Unqualified raw-only region; discard this entire current measurement."""
    def __init__(self, evidence):
        super().__init__('station_seed_measurement_insufficient')
        self.evidence = deepcopy(evidence)


class CurrentSourceOwnershipConflict(CurrentMeasurementRejected):
    """The current proposal overlaps independently measured source components."""
    def __init__(self, reason, evidence):
        super().__init__(reason)
        self.evidence = deepcopy(evidence)


def _ownership_conflict(reason, source_ns, topology, proposal, component,
                        output_shape, component_basis):
    proposal = np.ascontiguousarray(proposal, dtype=np.uint8)
    component = np.ascontiguousarray(component, dtype=np.uint8)
    overlap = np.ascontiguousarray((proposal != 0) & (component != 0), dtype=np.uint8)
    return CurrentSourceOwnershipConflict(reason, dict(kind='source_component_overlap',
        source_ns=source_ns, stage='complete_current_ownership_and_geometry',
        component_basis=component_basis, region_shape=list(output_shape),
        source_shape=list(proposal.shape), proposal_pixels=int(np.count_nonzero(proposal)),
        component_pixels=int(np.count_nonzero(component)), overlap_pixels=int(np.count_nonzero(overlap)),
        proposal_mask_sha256=_sha(proposal), component_mask_sha256=_sha(component),
        overlap_mask_sha256=_sha(overlap),
        source_topology_contract_digest=topology.contract_digest,
        source_topology_map_sha256=topology.map_sha256,
        source_qualified=False, identity_verified=False, metric_geometry_authority=False,
        motion_authorized=False, reference_refreshed=False))

@dataclass(frozen=True)
class SourceConnectedOwner:
    owner: Candidate
    audit: dict


@dataclass(frozen=True)
class _RecoveredSourceOwner:
    raw: Candidate
    owner: Candidate
    audit: dict


@dataclass(frozen=True)
class SourceConnectedMeasurement:
    """A proved measurement view; the original segmentation remains intact.

    The caller must still qualify the fixed station template, current owner,
    feedback and original deadlines before committing any staged tracker state.
    """
    original: Segmentation
    normalized_result: Segmentation
    raw: Candidate
    refined: Candidate
    audit: dict
    _inputs: dict = field(repr=False, compare=False)
    _input_digest: str = field(repr=False, compare=False)
    _output_digest: str = field(repr=False, compare=False)
    _original_candidates: tuple = field(repr=False, compare=False)
    _normalized_candidates: tuple = field(repr=False, compare=False)

    def require_unchanged(self):
        """Recheck once at the caller's commit boundary, not at each graph node."""
        for current, frozen in ((self.original.candidates, self._original_candidates),
                                (self.normalized_result.candidates, self._normalized_candidates)):
            if len(current) != len(frozen) or any(a is not b for a, b in zip(current, frozen)):
                raise ValueError('current_measurement_candidate_identity_changed')
        if (_digest(_measurement_inputs(**self._inputs)) != self._input_digest
                or _digest(_measurement_outputs(self.normalized_result, self.raw,
                                               self.refined, self.audit)) != self._output_digest):
            raise ValueError('current_measurement_source_or_proposal_changed')
        return self


def _sha(value):
    return hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest()


def _reject_single_pixel_bridge(mask, depth, cores, edge, check_current, *, connectivity=4,
                                possible_owners=()):
    """Reject articulation pixels separating two independently trusted cores.

    Graph edges follow the explicitly selected sampling grid. A cut
    must separate at least the original 25-pixel trustworthy core minimum on
    both sides and involve distinct native seeds. Unqualified raw regions may
    also supply a negative-only upper bound on a possible independent core:
    every measured region pixel stays in the graph and can cause rejection,
    but none becomes a trusted foreground seed. No pixel is added or filled.
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
    witnesses = list(cores) + list(possible_owners)
    own = np.stack([core[yy, xx].astype(np.int32) for core in witnesses], axis=1)
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
                                    for i in range(len(witnesses)) for j in range(len(witnesses)) if i != j)):
                        raise ValueError('current_fragment_single_pixel_measured_bridge')
                subtree[p] += subtree[vertex]
                low[p] = min(low[p], low[vertex])
    if clock != count:
        raise ValueError('current_fragment_measured_group_disconnected')
    check_current()


def _recover_source_connected_owner(result, rgb, depth, intrinsics, transform,
        config, anchor, last, spatial_reference, primary_refined, minimum_quality, *,
        source_topology, camera, source_stamp_ns, rgb_valid=None, check_current,
        record_stage=None, all_matches=False, independent_measurement=None):
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
    if spatial_reference is None or (len(matches) < 2 if all_matches else len(matches) != 1):
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
    primary = None if all_matches else matches[0][1]
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
    if all_matches:
        selected_indices = []
        for _, candidate in matches:
            source_match = source_topology.source_array(candidate.mask > 0)
            selected = [i for i, mask in enumerate(components) if np.any(mask & source_match)]
            if len(selected) != 1 or np.any(source_match & ~components[selected[0]]):
                return None
            selected_indices.append(selected[0])
        if len(set(selected_indices)) != 1:
            return None
        source_group = components[selected_indices[0]]
    else:
        source_primary = source_topology.source_array(primary.mask > 0)
        selected = [mask for mask in components if np.any(mask & source_primary)]
        if len(selected) != 1 or np.any(source_primary & ~selected[0]):
            raise ValueError('current_fragment_primary_component_invalid')
        source_group = selected[0]
    group = source_topology.output_mask(source_group)
    members = [(i, c) for i, c in enumerate(result.candidates) if np.any((c.mask > 0) & group)]
    if len(members) < 2:
        return None
    if all_matches:
        # Only these original per-fragment fields are replaced by the complete
        # measured proposal below. Every other source/support fact must agree.
        per_fragment = {'valid_points', 'median_height_m', 'observed_surface_extent_m'}
        common = [_proof_value({key: value for key, value in candidate.metrics.items()
                               if key not in per_fragment}) for _, candidate in members]
        if any(value != common[0] for value in common[1:]):
            raise ValueError('current_fragment_common_measurement_metadata_mismatch')
        primary = members[0][1]  # Common metadata only; no member selects pixels.
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
    extra_cores, possible_owners, unqualified_regions = [], [], []
    independent_pair = None
    # Apply the same local seed semantics as each original native fragment.
    # A global owner-height percentile can conceal a smaller independent top
    # even when its own current plane supports a trustworthy local interior.
    extra_regions = _depth_components(source_topology.source_array(raw_only).astype(np.uint8),
        source_depth, config.min_object_points, config.depth_edge_m, connectivity=4)
    for region_index, region in enumerate(extra_regions):
        check_current()
        output_region = source_topology.output_mask(region)
        yy, xx = np.nonzero(output_region)
        sl = np.s_[max(0, int(yy.min())-margin):min(depth.shape[0], int(yy.max())+margin+1),
                   max(0, int(xx.min())-margin):min(depth.shape[1], int(xx.max())+margin+1)]
        try:
            current_core, _, _ = frame.confidence_labels(output_region[sl], frame.points[sl], depth[sl],
                heights[sl], plane_camera[:3], config, bgr=rgb[sl])
        except MeasuredSeedCoreInsufficient as exc:
            # This optional graph probe cannot erase an unproved region or
            # authorize RGB growth into it. Native and union gates still fail
            # normally; only this source-bound raw-only shortage is typed.
            check_current()
            shortage = RawOnlySeedInsufficient(dict(exc.evidence, source_ns=source_stamp_ns,
                stage='raw_only_extra_core', region_index=region_index,
                region_mask_sha256=_sha(output_region.astype(np.uint8)),
                region_shape=list(output_region.shape)))
            if (type(exc) is not MeasuredSeedCoreInsufficient or not all_matches
                    or independent_measurement is None):
                raise shortage from exc
            if independent_pair is None:
                stage('independent_current_native_proof')
                try:
                    independent_result, independent_pair = independent_measurement()
                except ValueError as failure:
                    if type(failure) is not ValueError or not str(failure).startswith('matching_fragment_'):
                        raise
                    raise shortage from failure
                check_current()
                if (not isinstance(independent_pair, tuple) or len(independent_pair) != 2
                        or not all(isinstance(c, Candidate) for c in independent_pair)
                        or not isinstance(independent_result, Segmentation)
                        or len(independent_result.candidates) != 1
                        or independent_result.candidates[0] is not independent_pair[0]):
                    raise ValueError('current_fragment_independent_proof_pair_invalid')
                proof = independent_pair[1].metrics.get('observation_fragment_group', {})
                if (proof.get('all_original_matches_accounted_for') is not True
                        or proof.get('final_original_geometry_passed') is not True
                        or proof.get('original_metric_matches') != len(matches)
                        or proof.get('appearance_identity', {}).get('appearance_match') is not True):
                    raise ValueError('current_fragment_complete_independent_proof_required')
                stage('raw_source_component_evidence')
            # Preserve the entire measured region as an upper bound for a
            # possible separate owner. This can only ADD bridge vetoes; it
            # neither lowers the seed threshold nor labels any RGB pixels.
            possible_owners.append(region.copy())
            unqualified_regions.append(dict(shortage.evidence,
                role='negative_only_possible_owner_bound', foreground_seed_pixels=0))
            continue
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
        check_current, connectivity=4, possible_owners=possible_owners)
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
    refined = (frame.refine(rgb, proposal, rgb_valid=rgb_valid) if independent_pair is None
               else independent_pair[1])
    check_current()
    stage('complete_current_ownership_and_geometry')
    # An independent measured source component is not part of this restored
    # proposal. Fixed-template tolerance cannot authorize RGB growth into it,
    # even when that other object is small or has the same surface colour.
    if np.any((refined.mask > 0) & occupied & ~group):
        raise _ownership_conflict('current_fragment_other_measured_component_overlap',
            source_stamp_ns, source_topology, source_topology.source_array(refined.mask > 0),
            source_topology.source_array(occupied & ~group), depth.shape, 'native_measured')
    source_output = source_topology.source_array(refined.mask > 0)
    foreign_count = 0
    for component in raw_components:
        if component is owners[0]:
            continue
        foreign_count += 1
        if np.any(component & source_output):
            raise _ownership_conflict('current_fragment_foreign_raw_source_component_overlap',
                source_stamp_ns, source_topology, source_output, component,
                depth.shape, 'raw_source')
    for core, row in zip(cores, rows):
        fraction = float(np.count_nonzero((refined.mask > 0) & core)/np.count_nonzero(core))
        row['retained_fraction'] = fraction
        if fraction < .85:
            raise ValueError('current_fragment_native_core_not_owned')
    final = (refined.mask > 0) & valid
    quality = .65*max(0., 1.-residual/(2*config.support_threshold_m)) + .35*min(1., int(final.sum())/400.)
    if not np.any(final):
        raise ValueError('current_fragment_final_measurement_empty')
    if independent_pair is not None and not np.array_equal(independent_pair[0].mask > 0, final):
        raise ValueError('current_fragment_independent_raw_depth_mismatch')
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
    return _RecoveredSourceOwner(proposal if independent_pair is None else independent_pair[0], refined, dict(
        policy='exact_source_four_neighbor_complete_measurement', original_metric_matches=len(matches),
        native_members=rows, source_topology_contract_digest=source_topology.contract_digest,
        source_topology_map_sha256=source_topology.map_sha256, measured_connectivity=4,
        connectivity_raster='pre_rectification_aligned_color', measured_depth_edge_m=float(config.depth_edge_m),
        measured_component_pixels=int(group.sum()), measured_component_sha256=_sha(group.astype(np.uint8)),
        raw_foreground_component_points=int(owners[0].sum()),
        raw_only_regions_checked=len(extra_regions),
        raw_only_trusted_core_points=[int(core.sum()) for core in extra_cores],
        raw_only_unqualified_regions=unqualified_regions,
        independent_current_native_proof_used=independent_pair is not None,
        foreign_raw_foreground_components_checked=foreign_count,
        articulation_guard_passed=True, minimum_native_core_retention=.85,
        partial_outline_tolerance_used=False, native_union_used_as_rgb_seed=independent_pair is None,
        original_source_component_restored=True, historical_pixels_used=0, depth_filled=False,
        metric_geometry_authority=False, motion_authorized=False))


def recover_source_connected_fragment_owner(result, rgb, depth, intrinsics, transform,
        config, anchor, last, spatial_reference, primary_refined, minimum_quality, *,
        source_topology, camera, source_stamp_ns, rgb_valid=None, check_current,
        record_stage=None):
    """Preserve the original single-match recovery entry and return contract."""
    recovered = _recover_source_connected_owner(result, rgb, depth, intrinsics, transform,
        config, anchor, last, spatial_reference, primary_refined, minimum_quality,
        source_topology=source_topology, camera=camera, source_stamp_ns=source_stamp_ns,
        rgb_valid=rgb_valid, check_current=check_current, record_stage=record_stage)
    return None if recovered is None else SourceConnectedOwner(recovered.owner, recovered.audit)


def _proof_value(value):
    if isinstance(value, np.ndarray):
        return dict(array_dtype=value.dtype.str, shape=list(value.shape), sha256=_sha(value))
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        if not all(isinstance(key, str) for key in value):
            raise ValueError('current_measurement_provenance_key_invalid')
        return {key: _proof_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_proof_value(item) for item in value]
    if value is None or isinstance(value, (str, bool, int, float)):
        return value
    raise ValueError('current_measurement_provenance_value_invalid')


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def _candidate_record(candidate):
    return _proof_value({key: getattr(candidate, key) for key in Candidate.__dataclass_fields__})


def _segmentation_record(result):
    return dict(candidates=[_candidate_record(candidate) for candidate in result.candidates],
                plane_base=_proof_value(result.plane_base), metrics=_proof_value(result.metrics))


def _measurement_inputs(result, rgb, depth, intrinsics, transform, config, anchor, last,
                        spatial_reference, minimum_quality, camera, source_stamp_ns, rgb_valid,
                        source_topology):
    return dict(original=_segmentation_record(result), rgb=_proof_value(rgb),
        depth=_proof_value(depth), intrinsics=_proof_value(np.asarray(intrinsics)),
        transform=_proof_value(transform), config=_proof_value(asdict(config)),
        anchor=_proof_value(dict(position_base=anchor.position_base, extent_m=anchor.extent_m)),
        last=_proof_value(dict(position_base=last.position_base)),
        spatial_reference=_proof_value(spatial_reference), minimum_quality=float(minimum_quality),
        camera=_proof_value(camera), source_stamp_ns=_proof_value(source_stamp_ns),
        rgb_valid=_proof_value(rgb_valid),
        source_topology=dict(contract_digest=source_topology.contract_digest,
            map_sha256=source_topology.map_sha256, source_shape=list(source_topology.source_shape),
            output_shape=list(source_topology.output_shape),
            source_to_output=_proof_value(source_topology.source_to_output),
            output_to_source=_proof_value(source_topology.output_to_source)))


def _measurement_outputs(normalized, raw, refined, audit):
    return dict(normalized=_segmentation_record(normalized), raw=_candidate_record(raw),
                refined=_candidate_record(refined), audit=_proof_value(audit))


def recover_source_connected_measurement(result, rgb, depth, intrinsics, transform,
        config, anchor, last, spatial_reference, primary_refined, minimum_quality, *,
        source_topology, camera, source_stamp_ns, rgb_valid=None, check_current,
        record_stage=None, independent_measurement=None):
    """Propose one complete current measurement without dropping competitors.

    A single match remains on the old entry. Multiple matches not wholly in
    one original occupied source component return None with the original
    result untouched. All current-source and strict measurement errors
    propagate; this function does not grant identity or geometric authority.
    """
    if not callable(check_current):
        raise ValueError('current_fragment_current_source_check_required')
    if independent_measurement is not None and not callable(independent_measurement):
        raise ValueError('current_fragment_independent_proof_callback_invalid')
    check_current()
    if (not isinstance(result.candidates, (list, tuple))
            or len(result.candidates) > HandoffPolicy().maximum_candidates
            or any(not isinstance(candidate, Candidate) for candidate in result.candidates)):
        raise ValueError('current_fragment_candidate_budget_or_type_invalid')
    matches = _matching_candidates(result.candidates, anchor, last, config)
    if len(matches) < 2:
        return None
    if spatial_reference is None:
        raise ValueError('current_fragment_spatial_reference_required')
    if not isinstance(source_topology, SourceSamplingTopology):
        raise ValueError('current_source_topology_required')
    original_candidates = tuple(result.candidates)
    inputs = dict(result=result, rgb=rgb, depth=depth, intrinsics=intrinsics, transform=transform,
        config=config, anchor=anchor, last=last, spatial_reference=spatial_reference,
        minimum_quality=minimum_quality, camera=camera, source_stamp_ns=source_stamp_ns,
        rgb_valid=rgb_valid, source_topology=source_topology)
    original_record = _measurement_inputs(**inputs)
    input_digest = _digest(original_record)
    recovered = _recover_source_connected_owner(result, rgb, depth, intrinsics, transform,
        config, anchor, last, spatial_reference, primary_refined, minimum_quality,
        source_topology=source_topology, camera=camera, source_stamp_ns=source_stamp_ns,
        rgb_valid=rgb_valid, check_current=check_current, record_stage=record_stage, all_matches=True,
        independent_measurement=independent_measurement)
    check_current()
    if (_digest(_measurement_inputs(**inputs)) != input_digest
            or len(result.candidates) != len(original_candidates)
            or any(a is not b for a, b in zip(result.candidates, original_candidates))):
        raise ValueError('current_measurement_original_source_changed')
    if recovered is None:
        return None
    members = tuple(row['candidate_index'] for row in recovered.audit['native_members'])
    match_indices = tuple(next(i for i, candidate in enumerate(original_candidates) if candidate is matched)
                          for _, matched in matches)
    if not set(match_indices).issubset(members):
        raise ValueError('current_measurement_original_match_not_covered')
    normalized_candidates = []
    for index, candidate in enumerate(original_candidates):
        if index == members[0]:
            normalized_candidates.append(recovered.raw)
        elif index not in members:
            normalized_candidates.append(candidate)
    normalized = Segmentation(normalized_candidates, np.array(result.plane_base, copy=True),
                              deepcopy(result.metrics))
    audit = dict(recovered.audit, source_stamp_ns=source_stamp_ns,
        original_candidate_count=len(original_candidates),
        original_candidate_records=original_record['original']['candidates'],
        original_metric_match_indices=list(match_indices), member_indices=list(members),
        nonmember_indices=[i for i in range(len(original_candidates)) if i not in members],
        normalized_original_indices=[None if candidate is recovered.raw
            else next(i for i, original in enumerate(original_candidates) if original is candidate)
            for candidate in normalized_candidates],
        original_input_sha256=input_digest, original_input=original_record,
        raw_record=_candidate_record(recovered.raw), refined_record=_candidate_record(recovered.owner),
        original_candidates_modified=False, fixed_station_identity_required=True)
    return SourceConnectedMeasurement(result, normalized, recovered.raw, recovered.owner, audit,
        inputs, input_digest, _digest(_measurement_outputs(normalized, recovered.raw, recovered.owner, audit)),
        original_candidates, tuple(normalized_candidates))
