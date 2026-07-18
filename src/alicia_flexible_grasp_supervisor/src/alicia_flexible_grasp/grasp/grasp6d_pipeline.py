"""Pure two-stage policy helpers for stable Grasp6D candidates.

This module deliberately has no ROS or MoveIt imports.  Callers map their
latest snapshot facts into the immutable inputs below and provide the strict
reachability checker at the expensive boundary.
"""

from dataclasses import dataclass, fields, replace
import math
from types import MappingProxyType

from alicia_flexible_grasp.grasp.grasp6d_stability import StableCandidate


_PHYSICAL_MAX_OPEN_WIDTH_M = 0.050
_PHYSICAL_CONTRACT_MIN_OPEN_WIDTH_M = 0.0495
_PHYSICAL_CONTRACT_MAX_OPEN_WIDTH_M = 0.0505


def _strict_true(value):
    return isinstance(value, bool) and value is True


def _finite_float(value):
    try:
        converted = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return converted if math.isfinite(converted) else None


def _validated_count(value, name):
    if isinstance(value, bool):
        raise ValueError('{} must be a non-negative integer'.format(name))
    try:
        converted = int(value)
    except (TypeError, ValueError, OverflowError):
        raise ValueError('{} must be a non-negative integer'.format(name))
    if converted != value or converted < 0:
        raise ValueError('{} must be a non-negative integer'.format(name))
    return converted


def _validated_code(value):
    if not isinstance(value, str) or not value.strip():
        raise ValueError('rejection code must be a non-empty string')
    return value.strip()


@dataclass(frozen=True)
class SafetyGateInput:
    """Only mandatory physical and identity facts for one latest pose."""

    depth_valid: bool
    transform_valid: bool
    target_present: bool
    same_target_instance: bool
    target_absolute_distance_m: float
    target_absolute_limit_m: float
    required_open_width_m: float
    physical_open_width_m: float
    geometry_valid: bool
    collision_free: bool


@dataclass(frozen=True)
class GateDecision:
    ok: bool
    code: str
    reason: str


def _gate_failure(code, reason):
    return GateDecision(ok=False, code=code, reason=reason)


def mandatory_safety_gate(gate):
    """Fail closed on mandatory facts without applying preference thresholds."""

    if not isinstance(gate, SafetyGateInput):
        return _gate_failure(
            'SAFETY_INPUT_INVALID',
            'safety gate input is missing or has the wrong type',
        )
    if not _strict_true(gate.depth_valid):
        return _gate_failure(
            'DEPTH_INVALID',
            'candidate depth is missing, invalid, or outside its domain',
        )
    if not _strict_true(gate.transform_valid):
        return _gate_failure(
            'TRANSFORM_INVALID',
            'candidate coordinates or rigid transform are invalid',
        )
    if not _strict_true(gate.target_present):
        return _gate_failure('TARGET_LOST', 'target is not present')
    if not _strict_true(gate.same_target_instance):
        return _gate_failure(
            'TARGET_INSTANCE_MISMATCH',
            'candidate does not belong to the current target instance',
        )

    absolute_distance = _finite_float(gate.target_absolute_distance_m)
    absolute_limit = _finite_float(gate.target_absolute_limit_m)
    if (
        absolute_distance is None
        or absolute_distance < 0.0
        or absolute_limit is None
        or absolute_limit <= 0.0
    ):
        return _gate_failure(
            'TRANSFORM_INVALID',
            'target distance or absolute sanity limit is invalid',
        )
    if absolute_distance > absolute_limit:
        return _gate_failure(
            'TARGET_ABSOLUTE_DISTANCE',
            'candidate exceeds the absolute target-instance sanity distance',
        )

    required_width = _finite_float(gate.required_open_width_m)
    if required_width is None or required_width <= 0.0:
        return _gate_failure(
            'GRIPPER_WIDTH_INVALID',
            'required physical opening must be finite and positive',
        )
    if required_width > _PHYSICAL_MAX_OPEN_WIDTH_M:
        return _gate_failure(
            'GRIPPER_TOO_NARROW',
            'required opening {:.6f}m exceeds {:.6f}m physical limit'.format(
                required_width, _PHYSICAL_MAX_OPEN_WIDTH_M
            ),
        )
    physical_width = _finite_float(gate.physical_open_width_m)
    if physical_width is None or physical_width <= 0.0:
        return _gate_failure(
            'GRIPPER_CONTRACT_INVALID',
            'physical gripper opening contract must be finite and positive',
        )
    if not (
        _PHYSICAL_CONTRACT_MIN_OPEN_WIDTH_M
        <= physical_width
        <= _PHYSICAL_CONTRACT_MAX_OPEN_WIDTH_M
    ):
        return _gate_failure(
            'GRIPPER_CONTRACT_INVALID',
            'physical gripper opening is outside the fixed contract tolerance',
        )
    if required_width > physical_width:
        return _gate_failure(
            'GRIPPER_TOO_NARROW',
            'required opening {:.6f}m exceeds {:.6f}m physical limit'.format(
                required_width, physical_width
            ),
        )

    if not _strict_true(gate.geometry_valid):
        return _gate_failure(
            'GEOMETRY_INVALID',
            'analytical geometry is missing or physically impossible',
        )
    if not _strict_true(gate.collision_free):
        return _gate_failure(
            'COLLISION',
            'analytical geometry reports a definite collision',
        )
    return GateDecision(ok=True, code='OK', reason='')


class CandidateStageFunnel:
    """Deterministic stage and hard-rejection accounting for one request."""

    def __init__(self, input_count):
        self.input_count = _validated_count(input_count, 'input_count')
        self.remaining_by_stage = {}
        self.entered_by_stage = {}
        self.passed_by_stage = {}
        self.rejected_by_stage = {}
        self.rejection_counts = {}
        self.rejection_counts_by_stage = {}

    def record_stage(
        self,
        stage,
        remaining_count=None,
        *,
        entered=None,
        passed=None,
        rejected=None
    ):
        stage = _validated_code(stage)
        if stage in self.remaining_by_stage:
            raise ValueError('stage {!r} has already been recorded'.format(stage))

        if passed is None:
            if remaining_count is None:
                raise ValueError('passed or remaining_count is required')
            passed_count = _validated_count(remaining_count, 'passed')
        else:
            passed_count = _validated_count(passed, 'passed')
            if remaining_count is not None and (
                _validated_count(remaining_count, 'remaining_count')
                != passed_count
            ):
                raise ValueError('remaining_count must equal passed')

        if entered is None:
            if self.remaining_by_stage:
                entered_count = next(reversed(self.remaining_by_stage.values()))
            else:
                entered_count = self.input_count
        else:
            entered_count = _validated_count(entered, 'entered')

        if rejected is None:
            if passed_count > entered_count:
                raise ValueError('passed cannot exceed entered')
            rejected_count = entered_count - passed_count
        else:
            rejected_count = _validated_count(rejected, 'rejected')
            if passed_count + rejected_count != entered_count:
                raise ValueError('passed plus rejected must equal entered')

        self.remaining_by_stage[stage] = passed_count
        self.entered_by_stage[stage] = entered_count
        self.passed_by_stage[stage] = passed_count
        self.rejected_by_stage[stage] = rejected_count

    def record_rejection(self, code, count=1, stage=None):
        code = _validated_code(code)
        count = _validated_count(count, 'count')
        if count == 0:
            return
        self.rejection_counts[code] = self.rejection_counts.get(code, 0) + count
        if stage is not None:
            stage = _validated_code(stage)
            stage_counts = self.rejection_counts_by_stage.setdefault(stage, {})
            stage_counts[code] = stage_counts.get(code, 0) + count

    def to_dict(self):
        ordered_rejections = {
            code: self.rejection_counts[code]
            for code in sorted(self.rejection_counts)
        }
        denominator = float(self.input_count)
        rejection_ratios = {
            code: (count / denominator if self.input_count else 0.0)
            for code, count in ordered_rejections.items()
        }
        primary_failure = None
        if ordered_rejections:
            primary_failure = min(
                ordered_rejections,
                key=lambda code: (-ordered_rejections[code], code),
            )
        stage_counts = {
            stage: {
                'entered': self.entered_by_stage[stage],
                'passed': self.passed_by_stage[stage],
                'rejected': self.rejected_by_stage[stage],
            }
            for stage in self.remaining_by_stage
        }
        rejection_counts_by_stage = {
            stage: {
                code: counts[code]
                for code in sorted(counts)
            }
            for stage, counts in self.rejection_counts_by_stage.items()
        }
        return {
            'input_count': self.input_count,
            'remaining_by_stage': dict(self.remaining_by_stage),
            'stage_counts': stage_counts,
            'rejection_counts': ordered_rejections,
            'rejection_counts_by_stage': rejection_counts_by_stage,
            'rejection_ratios': rejection_ratios,
            'primary_failure': primary_failure,
            'dominant_hard_failure': primary_failure,
        }


def _required_finite(value, name):
    converted = _finite_float(value)
    if converted is None:
        raise ValueError('{} must be finite'.format(name))
    return converted


@dataclass(frozen=True)
class SoftCandidateFeatures:
    """Finite preference features; none of these is a mandatory gate."""

    model_score: float
    cloud_distance_m: float
    center_distance_m: float
    downward_approach_cos: float
    visibility_center_cost: float
    support_margin_m: float
    jaw_tilt_cos: float
    geometry_margin_m: float
    joint_path_cost: float
    joint_max_delta_rad: float
    stability_hit_ratio: float
    position_dispersion_m: float
    orientation_dispersion_rad: float

    def __post_init__(self):
        for item in fields(self):
            object.__setattr__(
                self,
                item.name,
                _required_finite(getattr(self, item.name), item.name),
            )


@dataclass(frozen=True)
class SoftScoreWeights:
    """Configurable non-negative weights and positive normalization knees."""

    model_score_weight: float = 1.0
    cloud_distance_weight: float = 0.4
    center_distance_weight: float = 0.8
    downward_approach_weight: float = 0.8
    visibility_center_weight: float = 0.4
    support_margin_weight: float = 0.3
    jaw_tilt_weight: float = 0.3
    geometry_margin_weight: float = 0.6
    joint_path_weight: float = 0.5
    joint_max_delta_weight: float = 0.5
    stability_hit_ratio_weight: float = 0.8
    position_dispersion_weight: float = 0.4
    orientation_dispersion_weight: float = 0.4
    cloud_distance_knee_m: float = 0.020
    center_distance_knee_m: float = 0.040
    downward_approach_cos_knee: float = 0.75
    visibility_center_cost_knee: float = 0.40
    support_margin_knee_m: float = 0.010
    jaw_tilt_cos_knee: float = 0.90
    geometry_margin_knee_m: float = 0.010
    joint_path_cost_knee: float = 1.0
    joint_max_delta_knee_rad: float = 1.0
    position_dispersion_knee_m: float = 0.010
    orientation_dispersion_knee_rad: float = 0.20

    def __post_init__(self):
        for item in fields(self):
            value = _required_finite(getattr(self, item.name), item.name)
            if item.name.endswith('_weight'):
                if value < 0.0:
                    raise ValueError('{} must be non-negative'.format(item.name))
            elif value <= 0.0:
                raise ValueError('{} must be positive'.format(item.name))
            object.__setattr__(self, item.name, value)


@dataclass(frozen=True)
class SoftScore:
    total: float
    components: object

    def __post_init__(self):
        total = _required_finite(self.total, 'total')
        try:
            copied = dict(self.components)
        except (TypeError, ValueError):
            raise ValueError('components must be a finite mapping')
        normalized = {}
        for name in sorted(copied):
            if not isinstance(name, str) or not name:
                raise ValueError('component names must be non-empty strings')
            normalized[name] = _required_finite(
                copied[name], 'component {!r}'.format(name)
            )
        if not math.isclose(
            total,
            math.fsum(normalized.values()),
            rel_tol=1e-12,
            abs_tol=1e-12,
        ):
            raise ValueError('total must equal the sum of components')
        object.__setattr__(self, 'total', total)
        object.__setattr__(self, 'components', MappingProxyType(normalized))


def _clamp_unit(value):
    if value <= 0.0:
        return 0.0
    if value >= 1.0:
        return 1.0
    return value


def _normalized_positive(value, knee):
    if value <= 0.0:
        return 0.0
    if value >= knee:
        return 1.0
    return value / knee


def _normalized_cosine_reward(value, preferred_knee):
    if value <= -1.0:
        return 0.0
    if value >= preferred_knee:
        return 1.0
    return (value + 1.0) / (preferred_knee + 1.0)


def soft_candidate_cost(features, weights):
    """Return a finite normalized cost; lower is better.

    Margins, confidence, approach alignment, and stability are represented as
    bounded negative costs.  Distance, visibility, motion, and dispersion are
    bounded positive costs.  No component is a hard rejection.
    """

    if not isinstance(features, SoftCandidateFeatures):
        raise TypeError('features must be SoftCandidateFeatures')
    if not isinstance(weights, SoftScoreWeights):
        raise TypeError('weights must be SoftScoreWeights')

    components = {
        'model_score': -weights.model_score_weight
        * _clamp_unit(features.model_score),
        'cloud_distance': weights.cloud_distance_weight
        * _normalized_positive(
            features.cloud_distance_m, weights.cloud_distance_knee_m
        ),
        'center_distance': weights.center_distance_weight
        * _normalized_positive(
            features.center_distance_m, weights.center_distance_knee_m
        ),
        'downward_approach': -weights.downward_approach_weight
        * _normalized_cosine_reward(
            features.downward_approach_cos,
            weights.downward_approach_cos_knee,
        ),
        'visibility_center': weights.visibility_center_weight
        * _normalized_positive(
            features.visibility_center_cost,
            weights.visibility_center_cost_knee,
        ),
        'support_margin': -weights.support_margin_weight
        * _normalized_positive(
            features.support_margin_m, weights.support_margin_knee_m
        ),
        'jaw_tilt': -weights.jaw_tilt_weight
        * _normalized_cosine_reward(
            features.jaw_tilt_cos, weights.jaw_tilt_cos_knee
        ),
        'geometry_margin': -weights.geometry_margin_weight
        * _normalized_positive(
            features.geometry_margin_m, weights.geometry_margin_knee_m
        ),
        'joint_path': weights.joint_path_weight
        * _normalized_positive(
            features.joint_path_cost, weights.joint_path_cost_knee
        ),
        'joint_max_delta': weights.joint_max_delta_weight
        * _normalized_positive(
            features.joint_max_delta_rad,
            weights.joint_max_delta_knee_rad,
        ),
        'stability_hit_ratio': -weights.stability_hit_ratio_weight
        * _clamp_unit(features.stability_hit_ratio),
        'position_dispersion': weights.position_dispersion_weight
        * _normalized_positive(
            features.position_dispersion_m,
            weights.position_dispersion_knee_m,
        ),
        'orientation_dispersion': weights.orientation_dispersion_weight
        * _normalized_positive(
            features.orientation_dispersion_rad,
            weights.orientation_dispersion_knee_rad,
        ),
    }
    return SoftScore(total=math.fsum(components.values()), components=components)


@dataclass(frozen=True)
class MoveItResult:
    """Strict reachability result supplied by the ROS-facing caller."""

    reachable: bool
    joint_path_cost: float
    joint_max_delta_rad: float
    reason: str


@dataclass(frozen=True)
class ScoredStableCandidate:
    """One already-stable physical pose variant awaiting strict MoveIt."""

    stable_candidate: StableCandidate
    variant_index: int
    latest_safety: SafetyGateInput
    soft_features: SoftCandidateFeatures
    score_weights: SoftScoreWeights
    pre_moveit_score: float
    moveit_result: object = None
    final_score: object = None

    def __post_init__(self):
        if not isinstance(self.stable_candidate, StableCandidate):
            raise TypeError('stable_candidate must be StableCandidate')
        variant_index = _validated_count(self.variant_index, 'variant_index')
        if not isinstance(self.latest_safety, SafetyGateInput):
            raise TypeError('latest_safety must be SafetyGateInput')
        if not isinstance(self.soft_features, SoftCandidateFeatures):
            raise TypeError('soft_features must be SoftCandidateFeatures')
        if not isinstance(self.score_weights, SoftScoreWeights):
            raise TypeError('score_weights must be SoftScoreWeights')
        pre_moveit_score = _required_finite(
            self.pre_moveit_score, 'pre_moveit_score'
        )
        if self.moveit_result is not None and not isinstance(
            self.moveit_result, MoveItResult
        ):
            raise TypeError('moveit_result must be MoveItResult or None')
        final_score = self.final_score
        if final_score is not None:
            final_score = _required_finite(final_score, 'final_score')
            if self.moveit_result is None:
                raise ValueError('final_score requires a moveit_result')
        object.__setattr__(self, 'variant_index', variant_index)
        object.__setattr__(self, 'pre_moveit_score', pre_moveit_score)
        object.__setattr__(self, 'final_score', final_score)

    @property
    def track_id(self):
        return self.stable_candidate.track_id

    @property
    def payload(self):
        return self.stable_candidate.payload


@dataclass(frozen=True)
class BoundedMoveItSelection:
    selected: object
    checked: tuple
    reachable: tuple
    funnel: CandidateStageFunnel
    configured_top_n: int


# Concise compatibility name for downstream callers.
MoveItSelection = BoundedMoveItSelection


def _clamped_top_n(value):
    if isinstance(value, bool):
        raise ValueError('top_n must be an integer')
    try:
        converted = int(value)
    except (TypeError, ValueError, OverflowError):
        raise ValueError('top_n must be an integer')
    if converted != value:
        raise ValueError('top_n must be an integer')
    return min(10, max(3, converted))


def _finite_vector(value, size, require_nonzero=False):
    try:
        converted = tuple(float(component) for component in value)
    except (TypeError, ValueError, OverflowError):
        return False
    if len(converted) != size or not all(math.isfinite(x) for x in converted):
        return False
    if require_nonzero and not any(abs(x) > 1e-12 for x in converted):
        return False
    return True


def _stable_coordinate_gate(candidate):
    stable = candidate.stable_candidate
    vectors = (
        (stable.center_base_xyz, 3, False),
        (stable.tool0_position_xyz, 3, False),
        (stable.quaternion_xyzw, 4, True),
        (stable.approach_base_xyz, 3, True),
    )
    if not all(_finite_vector(*specification) for specification in vectors):
        return _gate_failure(
            'TRANSFORM_INVALID',
            'stable candidate contains invalid or non-finite coordinates',
        )
    return GateDecision(ok=True, code='OK', reason='')


def _moveit_failure_code(reason):
    text = reason.strip().lower().replace('_', ' ')
    if 'collision' in text:
        return 'MOVEIT_COLLISION'
    if 'joint' in text and 'limit' in text:
        return 'MOVEIT_JOINT_LIMIT'
    if 'ik' in text or 'inverse kinematic' in text:
        return 'MOVEIT_IK_FAILED'
    if 'plan' in text:
        return 'MOVEIT_PLANNING_FAILED'
    return 'MOVEIT_UNREACHABLE'


def _validated_moveit_result(result):
    if not isinstance(result, MoveItResult):
        return None
    if not isinstance(result.reachable, bool):
        return None
    if not isinstance(result.reason, str):
        return None
    joint_path = _finite_float(result.joint_path_cost)
    joint_max_delta = _finite_float(result.joint_max_delta_rad)
    if (
        joint_path is None
        or joint_path < 0.0
        or joint_max_delta is None
        or joint_max_delta < 0.0
    ):
        return None
    return MoveItResult(
        reachable=result.reachable,
        joint_path_cost=joint_path,
        joint_max_delta_rad=joint_max_delta,
        reason=result.reason,
    )


def _motion_cost_delta(candidate, result):
    baseline = soft_candidate_cost(candidate.soft_features, candidate.score_weights)
    with_motion = soft_candidate_cost(
        replace(
            candidate.soft_features,
            joint_path_cost=result.joint_path_cost,
            joint_max_delta_rad=result.joint_max_delta_rad,
        ),
        candidate.score_weights,
    )
    motion_names = ('joint_path', 'joint_max_delta')
    return math.fsum(
        with_motion.components[name] - baseline.components[name]
        for name in motion_names
    )


def bounded_moveit_select(candidates, checker, top_n=5):
    """Strictly check only the pre-ranked, latest-hard-safe Top N variants."""

    if not callable(checker):
        raise TypeError('checker must be callable')
    configured_top_n = _clamped_top_n(top_n)
    batch = tuple(candidates)
    if not all(isinstance(item, ScoredStableCandidate) for item in batch):
        raise TypeError('candidates must contain ScoredStableCandidate values')

    funnel = CandidateStageFunnel(input_count=len(batch))
    funnel.record_stage(
        'stable', entered=len(batch), passed=len(batch), rejected=0
    )

    hard_safe = []
    for candidate in batch:
        decision = mandatory_safety_gate(candidate.latest_safety)
        if decision.ok:
            decision = _stable_coordinate_gate(candidate)
        if decision.ok:
            hard_safe.append(candidate)
        else:
            funnel.record_rejection(
                decision.code, stage='hard_recheck'
            )
    funnel.record_stage(
        'hard_recheck',
        entered=len(batch),
        passed=len(hard_safe),
        rejected=len(batch) - len(hard_safe),
    )

    ranked = sorted(
        hard_safe,
        key=lambda item: (
            item.pre_moveit_score,
            item.track_id,
            item.variant_index,
        ),
    )
    funnel.record_stage(
        'soft_ranked',
        entered=len(hard_safe),
        passed=len(ranked),
        rejected=0,
    )
    shortlist = tuple(ranked[:configured_top_n])
    funnel.record_stage(
        'moveit_shortlist',
        entered=len(ranked),
        passed=len(shortlist),
        rejected=len(ranked) - len(shortlist),
    )

    checked = []
    reachable = []
    for candidate in shortlist:
        try:
            raw_result = checker(candidate)
        except Exception:
            funnel.record_rejection(
                'MOVEIT_CHECK_ERROR', stage='moveit_reachable'
            )
            checked.append(candidate)
            continue
        result = _validated_moveit_result(raw_result)
        if result is None:
            funnel.record_rejection(
                'MOVEIT_RESULT_INVALID', stage='moveit_reachable'
            )
            checked.append(candidate)
            continue
        evaluated = replace(candidate, moveit_result=result)
        checked.append(evaluated)
        if not result.reachable:
            funnel.record_rejection(
                _moveit_failure_code(result.reason),
                stage='moveit_reachable',
            )
            continue
        final_score = candidate.pre_moveit_score + _motion_cost_delta(
            candidate, result
        )
        evaluated = replace(evaluated, final_score=final_score)
        checked[-1] = evaluated
        reachable.append(evaluated)

    funnel.record_stage(
        'moveit_checked',
        entered=len(shortlist),
        passed=len(checked),
        rejected=0,
    )
    funnel.record_stage(
        'moveit_reachable',
        entered=len(checked),
        passed=len(reachable),
        rejected=len(checked) - len(reachable),
    )
    selected = None
    if reachable:
        selected = min(
            reachable,
            key=lambda item: (
                item.final_score,
                item.track_id,
                item.variant_index,
            ),
        )
    funnel.record_stage(
        'selected',
        entered=len(reachable),
        passed=1 if selected is not None else 0,
        rejected=len(reachable) - (1 if selected is not None else 0),
    )
    return BoundedMoveItSelection(
        selected=selected,
        checked=tuple(checked),
        reachable=tuple(reachable),
        funnel=funnel,
        configured_top_n=configured_top_n,
    )
