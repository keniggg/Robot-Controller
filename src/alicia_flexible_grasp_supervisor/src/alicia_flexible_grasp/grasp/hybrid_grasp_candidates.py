"""Source-aware normalized records for hybrid grasp planning candidates."""

from dataclasses import dataclass, replace
import math
from types import MappingProxyType
from typing import Mapping, Optional

import numpy as np

from .gripper_geometry import CandidateGateResult


VALID_CANDIDATE_SOURCES = frozenset(('graspnet', 'tabletop_geometry'))


def _finite_number(value, name, *, non_negative=False, positive=False):
    if isinstance(value, (bool, np.bool_)):
        raise ValueError('{} must be finite'.format(name))
    try:
        converted = float(value)
    except (TypeError, ValueError, OverflowError):
        raise ValueError('{} must be finite'.format(name))
    if not math.isfinite(converted):
        raise ValueError('{} must be finite'.format(name))
    if non_negative and converted < 0.0:
        raise ValueError('{} must be non-negative'.format(name))
    if positive and converted <= 0.0:
        raise ValueError('{} must be positive'.format(name))
    return converted


def _non_negative_integer(value, name):
    if isinstance(value, (bool, np.bool_)):
        raise ValueError('{} must be a non-negative integer'.format(name))
    try:
        converted = int(value)
    except (TypeError, ValueError, OverflowError):
        raise ValueError('{} must be a non-negative integer'.format(name))
    if converted != value or converted < 0:
        raise ValueError('{} must be a non-negative integer'.format(name))
    return converted


def _validated_source_lineage(candidate_source, source_lineage):
    if candidate_source not in VALID_CANDIDATE_SOURCES:
        raise ValueError('candidate_source must be a known canonical source')
    if not isinstance(source_lineage, tuple) or not source_lineage:
        raise ValueError('source_lineage must be a non-empty tuple')
    if any(item not in VALID_CANDIDATE_SOURCES for item in source_lineage):
        raise ValueError('source_lineage contains an unknown source')
    if tuple(sorted(set(source_lineage))) != source_lineage:
        raise ValueError('source_lineage must be sorted and unique')
    if candidate_source not in source_lineage:
        raise ValueError('source_lineage must contain candidate_source')
    return source_lineage


def _frozen_array(value, shape, name):
    try:
        copied = np.array(value, dtype=np.float64, copy=True)
    except (TypeError, ValueError, OverflowError):
        raise ValueError('{} must be a finite array'.format(name))
    if copied.shape != shape or not np.all(np.isfinite(copied)):
        raise ValueError('{} must be a finite array with shape {}'.format(name, shape))
    copied.setflags(write=False)
    return copied


def _frozen_unit_vector(value, name):
    copied = np.array(_frozen_array(value, (3,), name), copy=True)
    norm = float(np.linalg.norm(copied))
    if not math.isfinite(norm) or abs(norm - 1.0) > 1e-6:
        raise ValueError('{} must be a unit vector'.format(name))
    copied /= norm
    copied.setflags(write=False)
    return copied


def _optional_model_value(value, name):
    if value is None:
        return None
    return _finite_number(value, name, non_negative=True)


def bilateral_contact_balance(
    object_points_base,
    jaw_axis_base,
    contact_band_fraction=0.12,
):
    """Return source-neutral balance of opposite jaw projection bands."""
    try:
        points = np.array(object_points_base, dtype=np.float64, copy=True)
    except (TypeError, ValueError, OverflowError):
        raise ValueError('object_points_base must be a finite Nx3 cloud')
    if (
        points.ndim != 2
        or points.shape[1:] != (3,)
        or points.shape[0] < 2
        or not np.all(np.isfinite(points))
    ):
        raise ValueError('object_points_base must be a finite Nx3 cloud')
    jaw_axis = _frozen_unit_vector(jaw_axis_base, 'jaw_axis_base')
    fraction = _finite_number(
        contact_band_fraction,
        'contact_band_fraction',
        positive=True,
    )
    if fraction > 0.5:
        raise ValueError('contact_band_fraction must not exceed 0.5')

    projection = points.dot(jaw_axis)
    lower = float(np.min(projection))
    upper = float(np.max(projection))
    span = upper - lower
    if span <= 1e-12:
        raise ValueError('object_points_base has no extent along jaw_axis_base')
    band_width = fraction * span
    negative_count = int(np.count_nonzero(projection <= lower + band_width))
    positive_count = int(np.count_nonzero(projection >= upper - band_width))
    larger_count = max(negative_count, positive_count)
    if larger_count <= 0:
        raise ValueError('projection bands must contain points')
    balance = min(negative_count, positive_count) / float(larger_count)
    return min(1.0, max(0.0, balance))


@dataclass(frozen=True)
class MergeConfig:
    center_distance_m: float = 0.005
    insertion_angle_deg: float = 10.0
    jaw_angle_deg: float = 10.0

    def __post_init__(self):
        object.__setattr__(
            self,
            'center_distance_m',
            _finite_number(
                self.center_distance_m,
                'center_distance_m',
                positive=True,
            ),
        )
        for name in ('insertion_angle_deg', 'jaw_angle_deg'):
            angle = _finite_number(getattr(self, name), name, positive=True)
            if angle > 180.0:
                raise ValueError('{} must not exceed 180'.format(name))
            object.__setattr__(self, name, angle)


@dataclass(frozen=True)
class NormalizedPlanningCandidate:
    candidate_source: str
    source_index: int
    variant_index: int
    source_lineage: tuple
    contact_center_base: np.ndarray
    T_base_tool0: np.ndarray
    insertion_axis_base: np.ndarray
    jaw_axis_base: np.ndarray
    required_open_width_m: float
    model_width_m: Optional[float]
    model_score: Optional[float]
    source_local_score: float
    common_physical_cost: float
    geometry_gate: object
    grasp_sequence: object
    payload: object
    audit: Mapping

    def __post_init__(self):
        source = str(self.candidate_source)
        lineage = _validated_source_lineage(source, self.source_lineage)
        source_index = _non_negative_integer(self.source_index, 'source_index')
        variant_index = _non_negative_integer(self.variant_index, 'variant_index')
        if variant_index not in (0, 1):
            raise ValueError('variant_index must be 0 or 1')
        center = _frozen_array(
            self.contact_center_base, (3,), 'contact_center_base'
        )
        transform = _frozen_array(self.T_base_tool0, (4, 4), 'T_base_tool0')
        if not np.allclose(
            transform[3], (0.0, 0.0, 0.0, 1.0), atol=1e-9, rtol=0.0
        ):
            raise ValueError('T_base_tool0 must be a homogeneous transform')
        insertion = _frozen_unit_vector(
            self.insertion_axis_base, 'insertion_axis_base'
        )
        jaw = _frozen_unit_vector(self.jaw_axis_base, 'jaw_axis_base')
        required_width = _finite_number(
            self.required_open_width_m,
            'required_open_width_m',
            positive=True,
        )
        model_width = _optional_model_value(self.model_width_m, 'model_width_m')
        model_score = _optional_model_value(self.model_score, 'model_score')
        if source == 'tabletop_geometry' and (
            model_width is not None or model_score is not None
        ):
            raise ValueError('tabletop_geometry must not carry model values')
        if source == 'graspnet' and (
            model_width is None or model_score is None
        ):
            raise ValueError('graspnet must carry model width and score')
        if not isinstance(self.geometry_gate, CandidateGateResult):
            raise TypeError('geometry_gate must be a CandidateGateResult')
        if not self.geometry_gate.ok:
            raise ValueError('geometry_gate must represent a successful gate')
        if not math.isclose(
            required_width,
            self.geometry_gate.required_open_width_m,
            rel_tol=0.0,
            abs_tol=1e-9,
        ):
            raise ValueError('required_open_width_m must match geometry_gate')
        if not isinstance(self.audit, Mapping):
            raise ValueError('audit must be a mapping')

        object.__setattr__(self, 'candidate_source', source)
        object.__setattr__(self, 'source_index', source_index)
        object.__setattr__(self, 'variant_index', variant_index)
        object.__setattr__(self, 'source_lineage', lineage)
        object.__setattr__(self, 'contact_center_base', center)
        object.__setattr__(self, 'T_base_tool0', transform)
        object.__setattr__(self, 'insertion_axis_base', insertion)
        object.__setattr__(self, 'jaw_axis_base', jaw)
        object.__setattr__(self, 'required_open_width_m', required_width)
        object.__setattr__(self, 'model_width_m', model_width)
        object.__setattr__(self, 'model_score', model_score)
        object.__setattr__(
            self,
            'source_local_score',
            _finite_number(self.source_local_score, 'source_local_score'),
        )
        object.__setattr__(
            self,
            'common_physical_cost',
            _finite_number(self.common_physical_cost, 'common_physical_cost'),
        )
        object.__setattr__(self, 'audit', MappingProxyType(dict(self.audit)))


def _vector_angle_deg(first, second, *, undirected):
    dot = float(np.dot(first, second))
    if undirected:
        dot = abs(dot)
    dot = min(1.0, max(-1.0, dot))
    return math.degrees(math.acos(dot))


def _duplicates(first, second, config):
    return (
        float(
            np.linalg.norm(
                first.contact_center_base - second.contact_center_base
            )
        )
        <= config.center_distance_m
        and _vector_angle_deg(
            first.insertion_axis_base,
            second.insertion_axis_base,
            undirected=False,
        )
        <= config.insertion_angle_deg + 1e-12
        and _vector_angle_deg(
            first.jaw_axis_base,
            second.jaw_axis_base,
            undirected=True,
        )
        <= config.jaw_angle_deg + 1e-12
    )


def _canonical_undirected_axis(axis):
    values = tuple(float(value) for value in np.asarray(axis, dtype=float))
    for value in values:
        if abs(value) > 1e-12:
            return tuple(-item for item in values) if value < 0.0 else values
    return values


def _physical_rank(candidate):
    """Rank without source identity, confidence, symmetry, or input order."""

    return (
        float(candidate.common_physical_cost),
        float(candidate.required_open_width_m),
        -float(candidate.geometry_gate.support_clearance_m),
        tuple(float(value) for value in candidate.contact_center_base),
        tuple(float(value) for value in candidate.T_base_tool0.reshape(-1)),
        tuple(float(value) for value in candidate.insertion_axis_base),
        _canonical_undirected_axis(candidate.jaw_axis_base),
        int(candidate.variant_index),
    )


def merge_hybrid_candidates(candidates, config):
    """Merge cross-source duplicates using source-neutral physical facts.

    Physically identical exact ties retain both source-specific candidates.
    With no physical fact available to distinguish them, selecting either one
    would necessarily introduce source name or input order as hidden policy.
    """
    if not isinstance(config, MergeConfig):
        raise TypeError('config must be a MergeConfig')
    candidates = tuple(candidates)
    if any(not isinstance(item, NormalizedPlanningCandidate) for item in candidates):
        raise TypeError('candidates must contain NormalizedPlanningCandidate values')

    ordered = sorted(
        candidates,
        key=_physical_rank,
    )
    merged = []
    for candidate in ordered:
        match = next(
            (
                item
                for item in merged
                if item.candidate_source != candidate.candidate_source
                and item.variant_index == candidate.variant_index
                and _duplicates(item, candidate, config)
            ),
            None,
        )
        if match is None:
            merged.append(candidate)
            continue
        lineage = tuple(
            sorted(set(match.source_lineage + candidate.source_lineage))
        )
        match_index = merged.index(match)
        match_rank = _physical_rank(match)
        candidate_rank = _physical_rank(candidate)
        if candidate_rank == match_rank:
            merged[match_index] = replace(match, source_lineage=lineage)
            merged.append(replace(candidate, source_lineage=lineage))
            continue
        winner = candidate if candidate_rank < match_rank else match
        merged[match_index] = replace(winner, source_lineage=lineage)
    return tuple(merged)
