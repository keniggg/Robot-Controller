"""Pure geometry-based tabletop grasp proposal generation."""

from dataclasses import dataclass, replace
from types import MappingProxyType
from typing import Mapping

import numpy as np

from .gripper_geometry import (
    ANALYTICAL_FINGER_BOX_PADDING_XYZ_M,
    ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M,
    ANALYTICAL_JAW_CLEARANCE_EACH_SIDE_M,
    ANALYTICAL_MAX_INNER_GAP_M,
    ANALYTICAL_PALM_CENTER_TOOL_XYZ_M,
    BilateralSurfaceEvidence,
    GripperGeometry,
    _obb_line_interval,
    bilateral_surface_contact_bounds_m,
    bilateral_contact_height_bounds_m,
    evaluate_bilateral_surface_evidence,
    gripper_box_centers,
    parse_tool_axis,
)


_ALICIA_MAX_INNER_GAP_M = 0.050
_MAX_TABLETOP_CANDIDATES = 32
# Runtime materialization combines at most four adaptive interior samples with
# at most four continuously solved contact-boundary samples.  The external ROS
# configuration remains capped at four; this larger internal bound exists only
# so those two finite evidence sets can be evaluated together.
_MAX_MATERIALIZED_APPROACH_TILTS = 8


class _InputInvalid(ValueError):
    pass


class _TargetCloudInvalid(ValueError):
    pass


class _SupportPlaneInvalid(ValueError):
    pass


class TabletopCandidateContractError(ValueError):
    def __init__(self, code, reason):
        self.code = str(code)
        self.reason = str(reason)
        super().__init__(self.reason)


@dataclass(frozen=True)
class TabletopGeometryConfig:
    max_inner_gap_m: float = 0.050
    angle_step_deg: float = 15.0
    angle_dedup_deg: float = 2.0
    jaw_clearance_each_side_m: float = 0.002
    opening_fit_clearance_each_side_m: float = 0.002
    width_projection_trim_fraction: float = 0.0
    min_contact_band_points: int = 6
    contact_band_fraction: float = 0.12
    max_candidates: int = 8
    approach_tilt_degrees: tuple = ()


@dataclass(frozen=True)
class TabletopProposal:
    source_index: int
    contact_center_base: np.ndarray
    insertion_axis_base: np.ndarray
    jaw_axis_base: np.ndarray
    required_open_width_m: float
    aperture_margin_m: float
    negative_contact_count: int
    positive_contact_count: int
    contact_symmetry: float
    source_score: float
    angle_deg: float
    audit: Mapping

    def __post_init__(self):
        object.__setattr__(self, 'contact_center_base', _frozen_array(self.contact_center_base))
        object.__setattr__(self, 'insertion_axis_base', _frozen_array(self.insertion_axis_base))
        object.__setattr__(self, 'jaw_axis_base', _frozen_array(self.jaw_axis_base))
        object.__setattr__(self, 'audit', MappingProxyType(dict(self.audit)))


@dataclass(frozen=True)
class TabletopCandidate:
    source_index: int
    variant_index: int
    contact_center_base: np.ndarray
    T_base_tool0: np.ndarray
    insertion_axis_base: np.ndarray
    jaw_axis_base: np.ndarray
    required_open_width_m: float
    minimum_finger_support_clearance_m: float
    source_score: float
    audit: Mapping

    def __post_init__(self):
        object.__setattr__(
            self,
            'contact_center_base',
            _frozen_array(self.contact_center_base),
        )
        object.__setattr__(
            self,
            'T_base_tool0',
            _frozen_array(self.T_base_tool0),
        )
        object.__setattr__(
            self,
            'insertion_axis_base',
            _frozen_array(self.insertion_axis_base),
        )
        object.__setattr__(
            self,
            'jaw_axis_base',
            _frozen_array(self.jaw_axis_base),
        )
        object.__setattr__(self, 'audit', MappingProxyType(dict(self.audit)))


@dataclass(frozen=True)
class TabletopGenerationResult:
    proposals: tuple
    failure_code: str
    failure_reason: str
    sampled_angles_deg: tuple
    surface_evidence_rejections: tuple = ()

    def __post_init__(self):
        object.__setattr__(self, 'proposals', tuple(self.proposals))
        object.__setattr__(
            self, 'sampled_angles_deg', tuple(float(angle) for angle in self.sampled_angles_deg)
        )

    @property
    def ok(self):
        return bool(self.proposals)


def generate_tabletop_proposals(
    object_points_base,
    obb_center_base,
    R_base_obb,
    obb_size_xyz_m,
    support_point_base,
    support_normal_base,
    config,
    fused_surface=None,
    finger_geometry=None,
):
    """Return bounded, category-independent grasp proposals without side effects."""
    try:
        checked_config = _validated_config(config)
        points = _finite_points(object_points_base, checked_config)
        center = _finite_vector(obb_center_base, 'obb_center_base')
        rotation = _validated_rotation(R_base_obb)
        obb_size = _positive_vector(obb_size_xyz_m, 'obb_size_xyz_m')
        support_point = _finite_vector(support_point_base, 'support_point_base')
        normal = _unit_vector(support_normal_base, 'support_normal_base')
        if (fused_surface is None) != (finger_geometry is None):
            raise _InputInvalid(
                'fused_surface and finger_geometry must be supplied together'
            )
        if fused_surface is not None:
            if not isinstance(finger_geometry, GripperGeometry):
                raise _InputInvalid(
                    'finger_geometry must be a GripperGeometry'
                )
            points = _finite_points(
                fused_surface.points_base, checked_config
            )
    except _TargetCloudInvalid as error:
        return _failure('TARGET_CLOUD_INVALID', str(error))
    except _SupportPlaneInvalid as error:
        return _failure('SUPPORT_PLANE_INVALID', str(error))
    except (TypeError, ValueError, _InputInvalid) as error:
        return _failure('TABLETOP_GEOMETRY_INPUT_INVALID', str(error))

    if float(np.dot(rotation[:, 2], normal)) < 1.0 - 1e-5:
        return _failure(
            'SUPPORT_PLANE_INVALID',
            'OBB vertical axis does not match support normal',
        )

    angles = _sample_angles(rotation, normal, checked_config)
    proposals = []
    width_valid_count = 0
    contact_valid_count = 0
    bilateral_evidence_attempted = fused_surface is not None
    surface_evidence_rejections = []
    contact_center = center
    for angle_deg, jaw_axis in angles:
        projection = points.dot(jaw_axis)
        raw_lower = float(np.min(projection))
        raw_upper = float(np.max(projection))
        trim_fraction = checked_config.width_projection_trim_fraction
        if trim_fraction > 0.0:
            lower, upper = (
                float(value)
                for value in np.quantile(
                    projection,
                    (trim_fraction, 1.0 - trim_fraction),
                )
            )
        else:
            lower, upper = raw_lower, raw_upper
        span = upper - lower
        evidence = None
        measured_insertion_bounds = None
        if fused_surface is not None:
            evidence = evaluate_bilateral_surface_evidence(
                fused_surface,
                contact_center,
                jaw_axis,
                -normal,
                finger_geometry,
                minimum_points_per_side=max(
                    12, checked_config.min_contact_band_points
                ),
                support_normal_base=normal,
            )
            if evidence.ok:
                measured_insertion_bounds = (
                    bilateral_surface_contact_bounds_m(
                        fused_surface,
                        contact_center,
                        jaw_axis,
                        -normal,
                        finger_geometry,
                        minimum_points_per_side=max(
                            12, checked_config.min_contact_band_points
                        ),
                        support_normal_base=normal,
                    )
                )
            if not evidence.ok or measured_insertion_bounds is None:
                # Retain observation opportunities even when other measured
                # directions later fail physical materialization. OBB reach
                # only authorizes requesting a view, never a contact proposal.
                interval = _obb_line_interval(
                    contact_center, jaw_axis, center, rotation, obb_size)
                maximum_reach = 0.5 * min(
                    checked_config.max_inner_gap_m,
                    finger_geometry.max_inner_gap_m,
                    ANALYTICAL_MAX_INNER_GAP_M)
                if interval is not None and interval[0] < 0.0 < interval[1]:
                    clearance = checked_config.opening_fit_clearance_each_side_m
                    negative_reach = -float(interval[0]) + clearance
                    positive_reach = float(interval[1]) + clearance
                    if max(negative_reach, positive_reach) <= maximum_reach:
                        surface_evidence_rejections.append({
                            'angle_deg': float(angle_deg),
                            'failure_code': 'BILATERAL_SURFACE_EVIDENCE_MISSING',
                            'negative_reach_m': negative_reach,
                            'positive_reach_m': positive_reach,
                            'maximum_side_reach_m': maximum_reach,
                        })
            required = (
                float(evidence.measured_width_m)
                + 2.0 * checked_config.opening_fit_clearance_each_side_m
            )
        else:
            required = (
                span
                + 2.0 * checked_config.opening_fit_clearance_each_side_m
            )
        if span <= 1e-9 or not 0.0 < required <= checked_config.max_inner_gap_m:
            continue
        width_valid_count += 1
        if evidence is not None:
            negative = int(evidence.negative_jaw_points)
            positive = int(evidence.positive_jaw_points)
            if not evidence.ok or measured_insertion_bounds is None:
                continue
            # The proposal audit and downstream physical gate use support-up
            # coordinates.  The measured evaluator's fixed tool-Z insertion
            # axis is support-down for these tabletop proposals.
            contact_height_bounds = (
                -float(measured_insertion_bounds[1]),
                -float(measured_insertion_bounds[0]),
            )
        else:
            negative, positive = _contact_counts(
                projection, lower, upper, checked_config.contact_band_fraction
            )
            if min(negative, positive) < checked_config.min_contact_band_points:
                continue
            contact_height_bounds = bilateral_contact_height_bounds_m(
                points,
                contact_center,
                jaw_axis,
                normal,
                checked_config.contact_band_fraction,
            )
        if contact_height_bounds is None:
            continue
        contact_valid_count += 1
        symmetry = min(negative, positive) / float(max(negative, positive))
        proposals.append(
            TabletopProposal(
                source_index=len(proposals),
                contact_center_base=contact_center,
                insertion_axis_base=-normal,
                jaw_axis_base=jaw_axis,
                required_open_width_m=required,
                aperture_margin_m=checked_config.max_inner_gap_m - required,
                negative_contact_count=negative,
                positive_contact_count=positive,
                contact_symmetry=symmetry,
                source_score=-(checked_config.max_inner_gap_m - required) - 0.01 * symmetry,
                angle_deg=angle_deg,
                audit={
                    'projection_raw_min_m': raw_lower,
                    'projection_raw_max_m': raw_upper,
                    'projection_min_m': lower,
                    'projection_max_m': upper,
                    'width_projection_trim_fraction': trim_fraction,
                    'bilateral_contact_height_min_m': float(
                        contact_height_bounds[0]
                    ),
                    'bilateral_contact_height_max_m': float(
                        contact_height_bounds[1]
                    ),
                    **(
                        {
                            'bilateral_surface_evidence_code': evidence.code,
                            'negative_jaw_unique_view_count': int(
                                evidence.negative_view_count
                            ),
                            'positive_jaw_unique_view_count': int(
                                evidence.positive_view_count
                            ),
                            'measured_width_m': float(
                                evidence.measured_width_m
                            ),
                            'measured_contact_height_m': float(
                                evidence.contact_height_m
                            ),
                            'contact_height_bounds_source': (
                                'fused_measured_bilateral_surface'
                            ),
                        }
                        if isinstance(evidence, BilateralSurfaceEvidence)
                        else {}
                    ),
                },
            )
        )
    proposals.sort(key=lambda item: (item.source_score, item.angle_deg))
    bounded = tuple(
        replace(item, source_index=index)
        for index, item in enumerate(proposals[:checked_config.max_candidates])
    )
    sampled_angles = tuple(item[0] for item in angles)
    if bounded:
        return TabletopGenerationResult(
            bounded, '', '', sampled_angles, tuple(surface_evidence_rejections))
    if width_valid_count == 0:
        return TabletopGenerationResult(
            (),
            'NO_FIT_DIRECTION',
            'no sampled jaw direction fits the 50 mm gripper',
            sampled_angles,
            tuple(surface_evidence_rejections),
        )
    assert contact_valid_count == 0
    if bilateral_evidence_attempted:
        return TabletopGenerationResult(
            (),
            'BILATERAL_SURFACE_EVIDENCE_MISSING',
            'no aperture-valid jaw direction has measured bilateral surface evidence',
            sampled_angles,
            tuple(surface_evidence_rejections),
        )
    return TabletopGenerationResult(
        (),
        'CONTACT_SUPPORT_INVALID',
        'no aperture-valid jaw direction has bilateral contact support',
        sampled_angles,
    )


def materialize_tabletop_candidates(
    proposal,
    support_point_base,
    support_normal_base,
    gripper,
    tool_jaw_axis='y',
    tool_finger_length_axis='z',
    approach_tilt_degrees=(),
    branch_key=None,
):
    """Materialize one proposal as CAD-grounded approach/jaw variants."""
    if branch_key is not None and (
        not isinstance(branch_key, tuple)
        or len(branch_key) != 2
        or type(branch_key[0]) is not int
        or branch_key[0] not in (0, 1)
        or type(branch_key[1]) not in (int, float)
        or branch_key[1] not in (-1.0, 0.0, 1.0)
    ):
        raise TabletopCandidateContractError(
            'TABLETOP_APPROACH_INVALID',
            'branch_key must contain jaw variant 0/1 and tilt polarity -1/0/1',
        )
    if not isinstance(proposal, TabletopProposal):
        raise TabletopCandidateContractError(
            'TOOL0_GEOMETRY_INVALID',
            'proposal must be a TabletopProposal',
        )
    if not isinstance(gripper, GripperGeometry):
        raise TabletopCandidateContractError(
            'TOOL0_GEOMETRY_INVALID',
            'gripper must be a GripperGeometry',
        )
    if (
        gripper.jaw_clearance_each_side_m
        != ANALYTICAL_JAW_CLEARANCE_EACH_SIDE_M
    ):
        raise TabletopCandidateContractError(
            'TOOL0_GEOMETRY_INVALID',
            'gripper jaw clearance must match the fixed 2 mm contract',
        )
    try:
        support_point = _finite_vector(support_point_base, 'support_point_base')
        support_normal = _unit_vector(
            support_normal_base,
            'support_normal_base',
        )
    except (TypeError, ValueError) as error:
        raise TabletopCandidateContractError(
            'TABLETOP_APPROACH_INVALID',
            str(error),
        ) from error
    try:
        tilt_degrees = _validated_approach_tilt_degrees(approach_tilt_degrees)
    except _InputInvalid as error:
        raise TabletopCandidateContractError(
            'TABLETOP_APPROACH_INVALID',
            str(error),
        ) from error

    insertion = np.asarray(proposal.insertion_axis_base, dtype=float)
    jaw = np.asarray(proposal.jaw_axis_base, dtype=float)
    if (
        insertion.shape != (3,)
        or not np.all(np.isfinite(insertion))
        or abs(float(np.linalg.norm(insertion)) - 1.0) > 1e-6
        or float(np.dot(insertion, support_normal)) > -1.0 + 1e-6
    ):
        raise TabletopCandidateContractError(
            'TABLETOP_APPROACH_INVALID',
            'insertion axis must be antiparallel to the support normal',
        )
    if (
        jaw.shape != (3,)
        or not np.all(np.isfinite(jaw))
        or abs(float(np.linalg.norm(jaw)) - 1.0) > 1e-6
        or abs(float(np.dot(jaw, support_normal))) > 1e-6
    ):
        raise TabletopCandidateContractError(
            'TABLETOP_APPROACH_INVALID',
            'jaw axis must be a unit vector parallel to the support plane',
        )

    variants = []
    approach_variant_count = 1 + 2 * len(tilt_degrees)
    for jaw_variant_index, jaw_axis in enumerate((jaw, -jaw)):
        if branch_key is not None and jaw_variant_index != branch_key[0]:
            continue
        insertion_variants = _approach_insertion_variants(
            insertion,
            jaw_axis,
            support_normal,
            tilt_degrees,
        )
        for approach_variant_index, (
            tilted_insertion,
            tilt_deg,
            tilt_polarity,
        ) in enumerate(insertion_variants):
            if branch_key is not None and tilt_polarity != branch_key[1]:
                continue
            try:
                rotation = semantic_axes_to_tool_rotation(
                    insertion_axis_base=tilted_insertion,
                    jaw_axis_base=jaw_axis,
                    tool_jaw_axis=tool_jaw_axis,
                    tool_finger_length_axis=tool_finger_length_axis,
                )
                translation = solve_tool0_translation_for_support_clearance(
                    rotation=rotation,
                    lateral_target=proposal.contact_center_base,
                    support_point=support_point,
                    support_normal=support_normal,
                    clearance_m=gripper.support_clearance_m,
                    gripper=gripper,
                    tool_jaw_axis=tool_jaw_axis,
                    tool_finger_length_axis=tool_finger_length_axis,
                )
                transform = np.eye(4, dtype=float)
                transform[:3, :3] = rotation
                transform[:3, 3] = translation
                finger_corners = _open_finger_corners(
                    transform,
                    gripper,
                    tool_jaw_axis,
                    tool_finger_length_axis,
                )
                finger_heights = (finger_corners - support_point) @ support_normal
                minimum_clearance = float(np.min(finger_heights))
                if abs(minimum_clearance - gripper.support_clearance_m) > 1e-8:
                    raise ValueError('unable to solve the configured CAD clearance')
                contact_height = float(
                    np.dot(
                        np.asarray(proposal.contact_center_base) - support_point,
                        support_normal,
                    )
                )
                if not (
                    float(np.min(finger_heights)) - 1e-9
                    <= contact_height
                    <= float(np.max(finger_heights)) + 1e-9
                ):
                    raise ValueError(
                        'usable finger side band does not overlap contact height'
                    )
                finger_pair_center = translation + (
                    rotation @ ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M
                )
                center_delta = (
                    finger_pair_center
                    - np.asarray(proposal.contact_center_base, dtype=float)
                )
                center_lateral = center_delta - (
                    float(np.dot(center_delta, support_normal)) * support_normal
                )
                center_lateral_error = float(np.linalg.norm(center_lateral))
                if center_lateral_error > 1e-8:
                    raise ValueError(
                        'finger-pair center is not on the contact-center normal'
                    )
                palm_center = translation + (
                    rotation @ ANALYTICAL_PALM_CENTER_TOOL_XYZ_M
                )
                if (
                    float(np.dot(palm_center - finger_pair_center, tilted_insertion))
                    >= 0.0
                ):
                    raise ValueError('palm lies on the object side of the fingers')
            except TabletopCandidateContractError:
                raise
            except Exception as error:
                raise TabletopCandidateContractError(
                    'TOOL0_GEOMETRY_INVALID',
                    str(error),
                ) from error
            audit = dict(proposal.audit)
            audit.update({
                'proposal_source_index': int(proposal.source_index),
                'approach_variant_index': int(approach_variant_index),
                'approach_tilt_deg': float(tilt_deg),
                'approach_tilt_polarity': float(tilt_polarity),
                'minimum_finger_support_clearance_m': minimum_clearance,
                'tool0_translation_base': tuple(float(item) for item in translation),
                'finger_pair_center_base': tuple(
                    float(item) for item in finger_pair_center
                ),
                'finger_pair_center_lateral_error_m': center_lateral_error,
            })
            variants.append(
                TabletopCandidate(
                    source_index=(
                        int(proposal.source_index) * approach_variant_count
                        + int(approach_variant_index)
                    ),
                    variant_index=int(jaw_variant_index),
                    contact_center_base=proposal.contact_center_base,
                    T_base_tool0=transform,
                    insertion_axis_base=tilted_insertion,
                    jaw_axis_base=jaw_axis,
                    required_open_width_m=proposal.required_open_width_m,
                    minimum_finger_support_clearance_m=minimum_clearance,
                    source_score=proposal.source_score,
                    audit=audit,
                )
            )
    return tuple(variants)


def _approach_insertion_variants(
    insertion,
    jaw_axis,
    support_normal,
    tilt_degrees,
):
    variants = [(np.array(insertion, dtype=float, copy=True), 0.0, 0.0)]
    if not tilt_degrees:
        return tuple(variants)
    finger_tilt_axis = np.cross(jaw_axis, support_normal)
    norm = float(np.linalg.norm(finger_tilt_axis))
    if norm <= 1e-12:
        raise TabletopCandidateContractError(
            'TABLETOP_APPROACH_INVALID',
            'jaw axis cannot define a finger-length tilt direction',
        )
    finger_tilt_axis = finger_tilt_axis / norm
    for tilt_deg in tilt_degrees:
        radians = float(np.deg2rad(tilt_deg))
        for polarity in (-1.0, 1.0):
            tilted = (
                np.cos(radians) * np.asarray(insertion, dtype=float)
                + polarity * np.sin(radians) * finger_tilt_axis
            )
            tilted /= float(np.linalg.norm(tilted))
            variants.append((tilted, float(tilt_deg), float(polarity)))
    return tuple(variants)


def semantic_axes_to_tool_rotation(
    insertion_axis_base,
    jaw_axis_base,
    tool_jaw_axis='y',
    tool_finger_length_axis='z',
):
    insertion = _candidate_unit_axis(
        insertion_axis_base,
        'insertion_axis_base',
    )
    jaw = _candidate_unit_axis(jaw_axis_base, 'jaw_axis_base')
    if abs(float(np.dot(insertion, jaw))) > 1e-6:
        raise TabletopCandidateContractError(
            'TOOL0_GEOMETRY_INVALID',
            'semantic insertion and jaw axes must be orthogonal',
        )
    try:
        tool_jaw, jaw_index = parse_tool_axis(tool_jaw_axis)
        tool_finger, finger_index = parse_tool_axis(tool_finger_length_axis)
    except ValueError as error:
        raise TabletopCandidateContractError(
            'TOOL0_GEOMETRY_INVALID',
            str(error),
        ) from error
    if jaw_index == finger_index:
        raise TabletopCandidateContractError(
            'TOOL0_GEOMETRY_INVALID',
            'jaw and finger length axes must be different',
        )
    tool_cross = np.cross(tool_jaw, tool_finger)
    base_cross = np.cross(jaw, insertion)
    tool_basis = np.column_stack((tool_cross, tool_jaw, tool_finger))
    base_basis = np.column_stack((base_cross, jaw, insertion))
    rotation = base_basis @ tool_basis.T
    if (
        not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8)
        or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-8)
    ):
        raise TabletopCandidateContractError(
            'TOOL0_GEOMETRY_INVALID',
            'semantic axis mapping is not a right-handed orthonormal rotation',
        )
    return rotation


def solve_tool0_translation_for_support_clearance(
    rotation,
    lateral_target,
    support_point,
    support_normal,
    clearance_m,
    gripper,
    tool_jaw_axis='y',
    tool_finger_length_axis='z',
):
    rotation = np.asarray(rotation, dtype=float)
    if (
        rotation.shape != (3, 3)
        or not np.all(np.isfinite(rotation))
        or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8)
        or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-8)
    ):
        raise ValueError('rotation must be right-handed and orthonormal')
    target = _finite_vector(lateral_target, 'lateral_target')
    point = _finite_vector(support_point, 'support_point')
    normal = _unit_vector(support_normal, 'support_normal')
    clearance = float(clearance_m)
    if not np.isfinite(clearance) or clearance < 0.0:
        raise ValueError('clearance_m must be finite and non-negative')
    if not isinstance(gripper, GripperGeometry):
        raise ValueError('gripper must be a GripperGeometry')

    rotated_pair_center = rotation @ ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M
    target_delta = target - rotated_pair_center
    translation = target_delta - normal * float(np.dot(target_delta, normal))
    initial = np.eye(4, dtype=float)
    initial[:3, :3] = rotation
    initial[:3, 3] = translation
    corners = _open_finger_corners(
        initial,
        gripper,
        tool_jaw_axis,
        tool_finger_length_axis,
    )
    minimum = float(np.min((corners - point) @ normal))
    if not np.isfinite(minimum):
        raise ValueError('finger-corner support clearance is non-finite')
    translation = translation + (clearance - minimum) * normal
    return translation


def _candidate_unit_axis(value, name):
    try:
        axis = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as error:
        raise TabletopCandidateContractError(
            'TOOL0_GEOMETRY_INVALID',
            '%s must be a unit three-vector' % name,
        ) from error
    if (
        axis.shape != (3,)
        or not np.all(np.isfinite(axis))
        or abs(float(np.linalg.norm(axis)) - 1.0) > 1e-6
    ):
        raise TabletopCandidateContractError(
            'TOOL0_GEOMETRY_INVALID',
            '%s must be a unit three-vector' % name,
        )
    return np.array(axis, dtype=float, copy=True)


def _open_finger_corners(
    transform,
    gripper,
    tool_jaw_axis,
    tool_finger_length_axis,
):
    physical_gap = min(
        float(gripper.max_inner_gap_m),
        ANALYTICAL_MAX_INNER_GAP_M,
    )
    centers = gripper_box_centers(
        transform[:3, 3],
        transform[:3, :3],
        physical_gap,
        gripper,
        tool_jaw_axis,
        tool_finger_length_axis,
    )
    half = 0.5 * (
        np.asarray(gripper.finger_size_xyz_m, dtype=float)
        + ANALYTICAL_FINGER_BOX_PADDING_XYZ_M
    )
    local_corners = np.asarray(
        [
            [sx * half[0], sy * half[1], sz * half[2]]
            for sx in (-1.0, 1.0)
            for sy in (-1.0, 1.0)
            for sz in (-1.0, 1.0)
        ],
        dtype=float,
    )
    rotation = transform[:3, :3]
    return np.vstack(
        tuple(
            local_corners @ rotation.T + centers[name]
            for name in ('left_finger', 'right_finger')
        )
    )


def _failure(code, reason):
    return TabletopGenerationResult((), code, reason, ())


def _validated_config(config):
    if not isinstance(config, TabletopGeometryConfig):
        raise _InputInvalid('config must be a TabletopGeometryConfig')
    scalar_names = (
        'max_inner_gap_m',
        'angle_step_deg',
        'angle_dedup_deg',
        'jaw_clearance_each_side_m',
        'opening_fit_clearance_each_side_m',
        'width_projection_trim_fraction',
        'contact_band_fraction',
    )
    values = {}
    for name in scalar_names:
        value = getattr(config, name)
        if isinstance(value, bool) or not np.isscalar(value) or not np.isfinite(value):
            raise _InputInvalid('%s must be finite' % name)
        values[name] = float(value)
    if values['max_inner_gap_m'] <= 0.0:
        raise _InputInvalid('max_inner_gap_m must be positive')
    if values['max_inner_gap_m'] > _ALICIA_MAX_INNER_GAP_M:
        raise _InputInvalid('max_inner_gap_m exceeds the fixed 50 mm gripper contract')
    if not 0.0 < values['angle_step_deg'] <= 180.0:
        raise _InputInvalid('angle_step_deg must be in (0, 180]')
    if not 0.0 <= values['angle_dedup_deg'] < 90.0:
        raise _InputInvalid('angle_dedup_deg must be in [0, 90)')
    if values['jaw_clearance_each_side_m'] < 0.0:
        raise _InputInvalid('jaw_clearance_each_side_m must be non-negative')
    if (
        values['jaw_clearance_each_side_m']
        != ANALYTICAL_JAW_CLEARANCE_EACH_SIDE_M
    ):
        raise _InputInvalid(
            'jaw_clearance_each_side_m must match the fixed 2 mm gripper contract'
        )
    if values['opening_fit_clearance_each_side_m'] < 0.0:
        raise _InputInvalid('opening_fit_clearance_each_side_m must be non-negative')
    if (
        values['opening_fit_clearance_each_side_m']
        > ANALYTICAL_JAW_CLEARANCE_EACH_SIDE_M
    ):
        raise _InputInvalid(
            'opening_fit_clearance_each_side_m must not exceed the fixed '
            '2 mm gripper contract'
        )
    if not 0.0 <= values['width_projection_trim_fraction'] <= 0.05:
        raise _InputInvalid(
            'width_projection_trim_fraction must be in [0, 0.05]'
        )
    if not 0.0 < values['contact_band_fraction'] < 0.5:
        raise _InputInvalid('contact_band_fraction must be in (0, 0.5)')
    for name in ('min_contact_band_points', 'max_candidates'):
        value = getattr(config, name)
        if isinstance(value, bool) or not isinstance(value, (int, np.integer)):
            raise _InputInvalid('%s must be an integer' % name)
        values[name] = int(value)
    if values['min_contact_band_points'] <= 0:
        raise _InputInvalid('min_contact_band_points must be positive')
    if not 1 <= values['max_candidates'] <= _MAX_TABLETOP_CANDIDATES:
        raise _InputInvalid(
            'max_candidates must be between 1 and %d'
            % _MAX_TABLETOP_CANDIDATES
        )
    values['approach_tilt_degrees'] = _validated_approach_tilt_degrees(
        getattr(config, 'approach_tilt_degrees', ())
    )
    return TabletopGeometryConfig(**values)


def _validated_approach_tilt_degrees(value):
    if value is None:
        return ()
    if isinstance(value, bool):
        raise _InputInvalid('approach_tilt_degrees must be a sequence')
    if np.isscalar(value):
        items = (value,)
    else:
        try:
            items = tuple(value)
        except TypeError as error:
            raise _InputInvalid(
                'approach_tilt_degrees must be a sequence'
            ) from error
    cleaned = []
    for raw in items:
        if isinstance(raw, bool) or not np.isscalar(raw):
            raise _InputInvalid('approach_tilt_degrees entries must be finite')
        tilt = float(raw)
        if not np.isfinite(tilt):
            raise _InputInvalid('approach_tilt_degrees entries must be finite')
        if not 0.0 < tilt <= 45.0:
            raise _InputInvalid(
                'approach_tilt_degrees entries must be in (0, 45]'
            )
        if not any(abs(tilt - prior) <= 1e-9 for prior in cleaned):
            cleaned.append(tilt)
    if len(cleaned) > _MAX_MATERIALIZED_APPROACH_TILTS:
        raise _InputInvalid(
            'approach_tilt_degrees must contain at most %d values'
            % _MAX_MATERIALIZED_APPROACH_TILTS
        )
    return tuple(cleaned)


def _finite_points(points, config):
    try:
        array = np.asarray(points, dtype=float)
    except (TypeError, ValueError) as error:
        raise _TargetCloudInvalid('object_points_base must be a numeric Nx3 array') from error
    if array.ndim != 2 or array.shape[1:] != (3,):
        raise _TargetCloudInvalid('object_points_base must be an Nx3 array')
    if array.shape[0] < 2 * config.min_contact_band_points:
        raise _TargetCloudInvalid('object_points_base is too sparse for bilateral contacts')
    if not np.all(np.isfinite(array)):
        raise _TargetCloudInvalid('object_points_base contains non-finite values')
    return np.array(array, dtype=float, copy=True)


def _finite_vector(value, name):
    try:
        vector = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as error:
        raise _InputInvalid('%s must be a numeric three-vector' % name) from error
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise _InputInvalid('%s must be a finite three-vector' % name)
    return np.array(vector, dtype=float, copy=True)


def _positive_vector(value, name):
    vector = _finite_vector(value, name)
    if np.any(vector <= 0.0):
        raise _InputInvalid('%s must contain positive values' % name)
    return vector


def _validated_rotation(value):
    try:
        rotation = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as error:
        raise _InputInvalid('R_base_obb must be a numeric 3x3 matrix') from error
    if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
        raise _InputInvalid('R_base_obb must be a finite 3x3 matrix')
    if not np.allclose(rotation.T.dot(rotation), np.eye(3), atol=1e-5, rtol=0.0):
        raise _InputInvalid('R_base_obb must be orthonormal')
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-5, rtol=0.0):
        raise _InputInvalid('R_base_obb must have determinant +1')
    return np.array(rotation, dtype=float, copy=True)


def _unit_vector(value, name):
    try:
        vector = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as error:
        raise _SupportPlaneInvalid('%s must be a numeric unit three-vector' % name) from error
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise _SupportPlaneInvalid('%s must be a finite unit three-vector' % name)
    norm = float(np.linalg.norm(vector))
    if norm <= 1e-12 or not np.isclose(norm, 1.0, atol=1e-5, rtol=0.0):
        raise _SupportPlaneInvalid('%s must be a unit three-vector' % name)
    return np.array(vector, dtype=float, copy=True)


def _sample_angles(rotation, normal, config):
    basis_x = _plane_basis_x(normal)
    basis_y = np.cross(normal, basis_x)
    candidates = [rotation[:, 0], rotation[:, 1]]
    candidates.extend(
        basis_x * np.cos(np.deg2rad(angle_deg))
        + basis_y * np.sin(np.deg2rad(angle_deg))
        for angle_deg in np.arange(0.0, 180.0, config.angle_step_deg)
    )
    sampled = []
    cosine_threshold = float(np.cos(np.deg2rad(config.angle_dedup_deg)))
    for axis in candidates:
        projected = axis - normal * float(np.dot(axis, normal))
        axis_norm = float(np.linalg.norm(projected))
        if axis_norm <= 1e-12:
            continue
        jaw_axis = projected / axis_norm
        if any(abs(float(np.dot(jaw_axis, prior_axis))) > cosine_threshold for _, prior_axis in sampled):
            continue
        angle_deg = _axis_angle_deg(jaw_axis, basis_x, basis_y)
        sampled.append((angle_deg, _frozen_array(jaw_axis)))
    return tuple(sampled)


def _plane_basis_x(normal):
    reference = np.array([1.0, 0.0, 0.0])
    if abs(float(np.dot(reference, normal))) > 0.9:
        reference = np.array([0.0, 1.0, 0.0])
    projected = reference - normal * float(np.dot(reference, normal))
    return projected / float(np.linalg.norm(projected))


def _axis_angle_deg(axis, basis_x, basis_y):
    angle = float(np.rad2deg(np.arctan2(np.dot(axis, basis_y), np.dot(axis, basis_x))))
    return angle % 180.0


def _contact_counts(projection, lower, upper, fraction):
    band_width = (upper - lower) * fraction
    negative = int(np.count_nonzero(projection <= lower + band_width))
    positive = int(np.count_nonzero(projection >= upper - band_width))
    return negative, positive


def _robust_contact_center(points, support_point, normal):
    del support_point, normal
    return np.median(points, axis=0)


def _frozen_array(value):
    array = np.array(value, dtype=float, copy=True)
    array.setflags(write=False)
    return array
