"""Deterministic, geometry-driven grasp-stage profile generation.

The observation standoff belongs to the coarse camera-view policy and is not
handled here.  Contact-stage distances and tabletop tilt samples are derived
from the current object geometry, measured temporal depth repeatability, fixed
gripper CAD, and task-level physical bounds.  No object label or workspace
coordinate is part of the calculation.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class AdaptiveStageLimits:
    tilt_sample_count: int = 4
    max_tilt_deg: float = 45.0
    min_downward_cos: float = 0.65
    max_lateral_sweep_m: float = 0.010
    approach_min_m: float = 0.010
    approach_max_m: float = 0.030
    pregrasp_min_m: float = 0.025
    pregrasp_max_m: float = 0.060
    pregrasp_approach_gap_min_m: float = 0.005
    lift_min_m: float = 0.020
    lift_max_m: float = 0.080
    depth_uncertainty_scale: float = 2.0
    contact_overlap_min_m: float = 0.002
    approach_finger_fraction: float = 0.25
    pregrasp_finger_fraction: float = 0.20
    lift_finger_fraction: float = 0.50

    def __post_init__(self):
        if (
            isinstance(self.tilt_sample_count, bool)
            or not isinstance(self.tilt_sample_count, int)
            or not 1 <= self.tilt_sample_count <= 4
        ):
            raise ValueError('tilt_sample_count must be an integer in [1, 4]')
        scalar_names = (
            'max_tilt_deg',
            'min_downward_cos',
            'max_lateral_sweep_m',
            'approach_min_m',
            'approach_max_m',
            'pregrasp_min_m',
            'pregrasp_max_m',
            'pregrasp_approach_gap_min_m',
            'lift_min_m',
            'lift_max_m',
            'depth_uncertainty_scale',
            'contact_overlap_min_m',
            'approach_finger_fraction',
            'pregrasp_finger_fraction',
            'lift_finger_fraction',
        )
        for name in scalar_names:
            value = float(getattr(self, name))
            if not math.isfinite(value):
                raise ValueError('%s must be finite' % name)
            object.__setattr__(self, name, value)
        if not 0.0 < self.max_tilt_deg <= 45.0:
            raise ValueError('max_tilt_deg must be in (0, 45]')
        if not -1.0 <= self.min_downward_cos <= 1.0:
            raise ValueError('min_downward_cos must be in [-1, 1]')
        if self.max_lateral_sweep_m <= 0.0:
            raise ValueError('max_lateral_sweep_m must be positive')
        if not 0.0 < self.approach_min_m <= self.approach_max_m:
            raise ValueError('approach bounds must be positive and ordered')
        if not 0.0 < self.pregrasp_min_m <= self.pregrasp_max_m:
            raise ValueError('pregrasp bounds must be positive and ordered')
        if self.pregrasp_approach_gap_min_m <= 0.0:
            raise ValueError('pregrasp_approach_gap_min_m must be positive')
        if (
            self.approach_max_m + self.pregrasp_approach_gap_min_m
            > self.pregrasp_max_m
        ):
            raise ValueError(
                'pregrasp_max_m must contain the maximum approach plus gap'
            )
        if not 0.0 < self.lift_min_m <= self.lift_max_m:
            raise ValueError('lift bounds must be positive and ordered')
        for name in (
            'depth_uncertainty_scale',
            'approach_finger_fraction',
            'pregrasp_finger_fraction',
            'lift_finger_fraction',
        ):
            if getattr(self, name) <= 0.0:
                raise ValueError('%s must be positive' % name)
        if self.contact_overlap_min_m < 0.0:
            raise ValueError('contact_overlap_min_m must be non-negative')


@dataclass(frozen=True)
class AdaptiveStageProfile:
    tilt_deg: float
    pregrasp_distance_m: float
    approach_offset_m: float
    lift_height_m: float
    lateral_sweep_m: float
    object_height_m: float
    depth_uncertainty_m: float
    execution_position_error_m: float
    contact_overlap_requirement_m: float

    def __post_init__(self):
        for name in (
            'tilt_deg',
            'pregrasp_distance_m',
            'approach_offset_m',
            'lift_height_m',
            'lateral_sweep_m',
            'object_height_m',
            'depth_uncertainty_m',
            'execution_position_error_m',
            'contact_overlap_requirement_m',
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value):
                raise ValueError('%s must be finite' % name)
            object.__setattr__(self, name, value)
        if self.tilt_deg < 0.0:
            raise ValueError('tilt_deg must be non-negative')
        if self.approach_offset_m <= 0.0:
            raise ValueError('approach_offset_m must be positive')
        if self.pregrasp_distance_m <= self.approach_offset_m:
            raise ValueError('pregrasp must remain outside approach')
        if self.lift_height_m <= 0.0:
            raise ValueError('lift_height_m must be positive')
        if self.lateral_sweep_m < 0.0:
            raise ValueError('lateral_sweep_m must be non-negative')
        if self.object_height_m <= 0.0:
            raise ValueError('object_height_m must be positive')
        if self.depth_uncertainty_m < 0.0:
            raise ValueError('depth_uncertainty_m must be non-negative')
        if self.execution_position_error_m < 0.0:
            raise ValueError(
                'execution_position_error_m must be non-negative'
            )
        if self.contact_overlap_requirement_m < 0.0:
            raise ValueError(
                'contact_overlap_requirement_m must be non-negative'
            )


def derive_adaptive_stage_profiles(
    object_height_m,
    depth_repeatability_m,
    finger_length_m,
    support_clearance_m,
    limits,
    execution_position_error_m=0.0,
):
    """Return vertical plus bounded tilted contact-stage profiles.

    Tilt samples cover the entire physically admissible interval.  Pregrasp
    is a support-normal clearance waypoint; only the final insertion follows
    the tilted approach axis.  Therefore the lateral-sweep bound applies to
    the live approach offset, while pregrasp always retains the independently
    derived clearance and positive approach gap.
    """

    if not isinstance(limits, AdaptiveStageLimits):
        raise TypeError('limits must be AdaptiveStageLimits')
    object_height = _positive(object_height_m, 'object_height_m')
    finger_length = _positive(finger_length_m, 'finger_length_m')
    depth_repeatability = _nonnegative(
        depth_repeatability_m,
        'depth_repeatability_m',
    )
    execution_position_error = _nonnegative(
        execution_position_error_m,
        'execution_position_error_m',
    )
    support_clearance = _nonnegative(
        support_clearance_m,
        'support_clearance_m',
    )
    perception_uncertainty = max(
        support_clearance,
        depth_repeatability * limits.depth_uncertainty_scale,
    )
    # Target-surface repeatability sets the live contact evidence requirement;
    # support-plane clearance remains a separate swept-collision contract.
    contact_overlap_requirement = max(
        limits.contact_overlap_min_m,
        depth_repeatability * limits.depth_uncertainty_scale,
    )
    # Perception and physical endpoint errors arise independently.  Add their
    # measured bounds so neither can consume the other's contact clearance.
    uncertainty = perception_uncertainty + execution_position_error
    approach = _clamp(
        max(
            object_height,
            finger_length * limits.approach_finger_fraction,
        )
        + uncertainty,
        limits.approach_min_m,
        limits.approach_max_m,
    )
    minimum_pregrasp = max(
        limits.pregrasp_min_m,
        (
            approach
            + limits.pregrasp_approach_gap_min_m
            + execution_position_error
        ),
    )
    if minimum_pregrasp > limits.pregrasp_max_m + 1e-12:
        raise ValueError(
            'measured execution uncertainty has no pregrasp inside hard bounds'
        )
    nominal_pregrasp = _clamp(
        approach
        + max(
            finger_length * limits.pregrasp_finger_fraction,
            uncertainty,
        ),
        minimum_pregrasp,
        limits.pregrasp_max_m,
    )
    lift = _clamp(
        max(
            object_height * 0.25,
            finger_length * limits.lift_finger_fraction,
        )
        + uncertainty,
        limits.lift_min_m,
        limits.lift_max_m,
    )

    profiles = [
        _profile(
            tilt_deg=0.0,
            pregrasp_distance_m=nominal_pregrasp,
            approach_offset_m=approach,
            lift_height_m=lift,
            object_height_m=object_height,
            depth_uncertainty_m=uncertainty,
            execution_position_error_m=execution_position_error,
            contact_overlap_requirement_m=contact_overlap_requirement,
        )
    ]
    max_tilt = _maximum_admissible_tilt_deg(approach, limits)
    if max_tilt <= 1e-9:
        return tuple(profiles)
    for index in range(1, limits.tilt_sample_count + 1):
        tilt = max_tilt * float(index) / float(limits.tilt_sample_count)
        profiles.append(
            _profile(
                tilt_deg=tilt,
                pregrasp_distance_m=nominal_pregrasp,
                approach_offset_m=approach,
                lift_height_m=lift,
                object_height_m=object_height,
                depth_uncertainty_m=uncertainty,
                execution_position_error_m=execution_position_error,
                contact_overlap_requirement_m=contact_overlap_requirement,
            )
        )
    return tuple(profiles)


def adaptive_stage_profile_for_tilt(
    tilt_deg,
    object_height_m,
    depth_repeatability_m,
    finger_length_m,
    support_clearance_m,
    limits,
    execution_position_error_m=0.0,
):
    """Derive the admissible stage distances for one runtime insertion tilt."""

    tilt = _nonnegative(tilt_deg, 'tilt_deg')
    base_profiles = derive_adaptive_stage_profiles(
        object_height_m=object_height_m,
        depth_repeatability_m=depth_repeatability_m,
        finger_length_m=finger_length_m,
        support_clearance_m=support_clearance_m,
        limits=limits,
        execution_position_error_m=execution_position_error_m,
    )
    vertical = base_profiles[0]
    maximum_tilt_deg = _maximum_admissible_tilt_deg(
        vertical.approach_offset_m,
        limits,
    )
    if (
        tilt
        > maximum_tilt_deg + 1e-9
    ):
        raise ValueError(
            'insertion tilt %.6f deg exceeds live hard bounds %.6f deg'
            % (tilt, maximum_tilt_deg)
        )
    return _profile(
        tilt_deg=tilt,
        pregrasp_distance_m=vertical.pregrasp_distance_m,
        approach_offset_m=vertical.approach_offset_m,
        lift_height_m=vertical.lift_height_m,
        object_height_m=vertical.object_height_m,
        depth_uncertainty_m=vertical.depth_uncertainty_m,
        execution_position_error_m=(
            vertical.execution_position_error_m
        ),
        contact_overlap_requirement_m=(
            vertical.contact_overlap_requirement_m
        ),
    )


def _maximum_admissible_tilt_deg(minimum_pregrasp_m, limits):
    downward_limit = math.degrees(
        math.acos(max(-1.0, min(1.0, limits.min_downward_cos)))
    )
    ratio = limits.max_lateral_sweep_m / float(minimum_pregrasp_m)
    lateral_limit = 90.0 if ratio >= 1.0 else math.degrees(math.asin(ratio))
    return max(
        0.0,
        min(
            limits.max_tilt_deg,
            downward_limit,
            lateral_limit,
        ),
    )


def _profile(
    *,
    tilt_deg,
    pregrasp_distance_m,
    approach_offset_m,
    lift_height_m,
    object_height_m,
    depth_uncertainty_m,
    execution_position_error_m,
    contact_overlap_requirement_m,
):
    lateral = float(approach_offset_m) * math.sin(
        math.radians(float(tilt_deg))
    )
    return AdaptiveStageProfile(
        tilt_deg=tilt_deg,
        pregrasp_distance_m=pregrasp_distance_m,
        approach_offset_m=approach_offset_m,
        lift_height_m=lift_height_m,
        lateral_sweep_m=lateral,
        object_height_m=object_height_m,
        depth_uncertainty_m=depth_uncertainty_m,
        execution_position_error_m=execution_position_error_m,
        contact_overlap_requirement_m=contact_overlap_requirement_m,
    )


def _clamp(value, minimum, maximum):
    return min(float(maximum), max(float(minimum), float(value)))


def _positive(value, name):
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError('%s must be finite and positive' % name)
    return number


def _nonnegative(value, name):
    number = float(value)
    if not math.isfinite(number) or number < 0.0:
        raise ValueError('%s must be finite and non-negative' % name)
    return number
