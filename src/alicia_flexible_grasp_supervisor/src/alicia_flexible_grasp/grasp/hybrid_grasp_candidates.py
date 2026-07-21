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
    copied = _frozen_array(value, (3,), name)
    norm = float(np.linalg.norm(copied))
    if not math.isfinite(norm) or abs(norm - 1.0) > 1e-6:
        raise ValueError('{} must be a unit vector'.format(name))
    return copied


def _optional_model_value(value, name):
    if value is None:
        return None
    return _finite_number(value, name, non_negative=True)


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
        <= config.insertion_angle_deg
        and _vector_angle_deg(
            first.jaw_axis_base,
            second.jaw_axis_base,
            undirected=True,
        )
        <= config.jaw_angle_deg
    )


def merge_hybrid_candidates(candidates, config):
    """Merge only cross-source duplicates of the same wrist variant."""
    if not isinstance(config, MergeConfig):
        raise TypeError('config must be a MergeConfig')
    candidates = tuple(candidates)
    if any(not isinstance(item, NormalizedPlanningCandidate) for item in candidates):
        raise TypeError('candidates must contain NormalizedPlanningCandidate values')

    ordered = sorted(
        candidates,
        key=lambda item: (
            item.common_physical_cost,
            item.required_open_width_m,
            -item.geometry_gate.support_clearance_m,
            item.source_index,
            item.variant_index,
        ),
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
        winner = (
            candidate
            if candidate.common_physical_cost < match.common_physical_cost
            else match
        )
        merged[merged.index(match)] = replace(winner, source_lineage=lineage)
    return tuple(merged)
