from collections import deque
from dataclasses import dataclass, field
import math
from typing import Any, Dict, Optional, Tuple

import numpy as np


_NORM_EPSILON = 1e-12


def _validated_integer(value, name, minimum):
    if isinstance(value, (bool, np.bool_)):
        raise ValueError('{} must be an integer >= {}'.format(name, minimum))
    try:
        converted = int(value)
    except (TypeError, ValueError, OverflowError):
        raise ValueError('{} must be an integer >= {}'.format(name, minimum))
    if converted != value or converted < minimum:
        raise ValueError('{} must be an integer >= {}'.format(name, minimum))
    return converted


def _validated_finite(value, name, positive=False):
    try:
        converted = float(value)
    except (TypeError, ValueError, OverflowError):
        raise ValueError('{} must be finite'.format(name))
    if not math.isfinite(converted):
        raise ValueError('{} must be finite'.format(name))
    if positive and converted <= 0.0:
        raise ValueError('{} must be finite and positive'.format(name))
    return converted


def _validated_target_identity(value):
    if not isinstance(value, tuple) or len(value) != 3:
        raise ValueError(
            'target_identity must be an (epoch, label, model_choice) tuple'
        )
    epoch, label, model_choice = value
    try:
        epoch = _validated_integer(epoch, 'target_identity epoch', 0)
    except ValueError:
        raise ValueError('target_identity epoch must be a non-negative integer')
    if not isinstance(label, str) or not label:
        raise ValueError('target_identity label must be a non-empty string')
    if not isinstance(model_choice, str) or not model_choice:
        raise ValueError(
            'target_identity model_choice must be a non-empty string'
        )
    return (epoch, label, model_choice)


def _frozen_vector(value, size, name, normalize=False):
    try:
        vector = np.array(value, dtype=np.float64, copy=True)
    except (TypeError, ValueError, OverflowError):
        raise ValueError('{} must be a finite {}-vector'.format(name, size))
    if vector.shape != (size,) or not np.all(np.isfinite(vector)):
        raise ValueError('{} must be a finite {}-vector'.format(name, size))
    if normalize:
        scale = float(np.max(np.abs(vector)))
        if not math.isfinite(scale) or scale <= 0.0:
            raise ValueError('{} must have non-zero finite norm'.format(name))
        vector /= scale
        norm = float(np.linalg.norm(vector))
        if not math.isfinite(norm) or norm <= 0.0:
            raise ValueError('{} must have non-zero finite norm'.format(name))
        vector /= norm
    vector.setflags(write=False)
    return vector


def _canonicalized_quaternion(value, name='quaternion'):
    quaternion = np.array(
        _frozen_vector(value, 4, name, normalize=True),
        dtype=np.float64,
        copy=True,
    )
    should_negate = quaternion[3] < 0.0
    if abs(float(quaternion[3])) <= _NORM_EPSILON:
        for component in quaternion[:3]:
            if abs(float(component)) > _NORM_EPSILON:
                should_negate = component < 0.0
                break
    if should_negate:
        quaternion *= -1.0
    quaternion.setflags(write=False)
    return quaternion


def _aligned_quaternion(reference, candidate):
    aligned = np.array(candidate, dtype=np.float64, copy=True)
    if float(np.dot(reference, aligned)) < 0.0:
        aligned *= -1.0
    aligned.setflags(write=False)
    return aligned


def _quaternion_multiply(first, second):
    x1, y1, z1, w1 = first
    x2, y2, z2, w2 = second
    product = np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ],
        dtype=np.float64,
    )
    return _canonicalized_quaternion(product)


def quaternion_angle_rad(first, second):
    """Return the sign-invariant angular distance between two xyzw quaternions."""
    first_q = _canonicalized_quaternion(first)
    second_q = _canonicalized_quaternion(second)
    dot = min(1.0, max(-1.0, abs(float(np.dot(first_q, second_q)))))
    return 2.0 * math.acos(dot)


def parallel_jaw_orientation_distance_rad(first, second):
    """Return distance and second quaternion aligned under local tool-Rz(pi)."""
    reference = _canonicalized_quaternion(first)
    candidate = _canonicalized_quaternion(second)
    local_rz_pi = np.array([0.0, 0.0, 1.0, 0.0], dtype=np.float64)
    symmetric = _quaternion_multiply(candidate, local_rz_pi)

    direct_angle = quaternion_angle_rad(reference, candidate)
    symmetric_angle = quaternion_angle_rad(reference, symmetric)
    if symmetric_angle < direct_angle:
        return symmetric_angle, _aligned_quaternion(reference, symmetric)
    return direct_angle, _aligned_quaternion(reference, candidate)


def weighted_median(values, weights):
    values_array = np.asarray(values, dtype=np.float64)
    weights_array = np.asarray(weights, dtype=np.float64)
    if (
        values_array.ndim != 1
        or weights_array.ndim != 1
        or values_array.size == 0
        or values_array.size != weights_array.size
    ):
        raise ValueError('values and weights must have the same non-zero length')
    if not np.all(np.isfinite(values_array)) or not np.all(
        np.isfinite(weights_array)
    ):
        raise ValueError('values and weights must be finite')
    if np.any(weights_array <= 0.0):
        raise ValueError('weights must be positive')

    order = np.argsort(values_array, kind='mergesort')
    ordered_values = values_array[order]
    ordered_weights = weights_array[order]
    cutoff = 0.5 * float(np.sum(ordered_weights))
    index = int(np.searchsorted(np.cumsum(ordered_weights), cutoff, side='left'))
    return float(ordered_values[index])


@dataclass(frozen=True)
class TrackingConfig:
    window_size: int = 5
    min_hits: int = 3
    position_threshold_m: float = 0.025
    orientation_threshold_deg: float = 25.0
    approach_threshold_deg: float = 20.0
    width_threshold_m: float = 0.008

    def __post_init__(self):
        window_size = _validated_integer(self.window_size, 'window_size', 1)
        min_hits = _validated_integer(self.min_hits, 'min_hits', 1)
        if min_hits > window_size:
            raise ValueError('min_hits must not exceed window_size')
        object.__setattr__(self, 'window_size', window_size)
        object.__setattr__(self, 'min_hits', min_hits)
        for name in (
            'position_threshold_m',
            'orientation_threshold_deg',
            'approach_threshold_deg',
            'width_threshold_m',
        ):
            converted = _validated_finite(getattr(self, name), name, positive=True)
            object.__setattr__(self, name, converted)
        if self.orientation_threshold_deg > 180.0:
            raise ValueError('orientation_threshold_deg must not exceed 180')
        if self.approach_threshold_deg > 180.0:
            raise ValueError('approach_threshold_deg must not exceed 180')


@dataclass(frozen=True)
class CandidateObservation:
    request_id: int
    snapshot_stamp_sec: float
    target_epoch: int
    target_label: str
    model_choice: str
    center_base_xyz: np.ndarray
    tool0_position_xyz: np.ndarray
    quaternion_xyzw: np.ndarray
    approach_base_xyz: np.ndarray
    required_open_width_m: float
    model_width_m: float
    model_score: float
    geometry_margin_m: float
    pre_moveit_score: float
    payload: Any = field(compare=False, repr=False)

    def __post_init__(self):
        object.__setattr__(
            self, 'request_id', _validated_integer(self.request_id, 'request_id', 1)
        )
        object.__setattr__(
            self,
            'snapshot_stamp_sec',
            _validated_finite(
                self.snapshot_stamp_sec, 'snapshot_stamp_sec', positive=True
            ),
        )
        object.__setattr__(
            self,
            'target_epoch',
            _validated_integer(self.target_epoch, 'target_epoch', 0),
        )
        for name in ('target_label', 'model_choice'):
            value = getattr(self, name)
            if not isinstance(value, str) or not value:
                raise ValueError('{} must be a non-empty string'.format(name))
        object.__setattr__(
            self,
            'center_base_xyz',
            _frozen_vector(self.center_base_xyz, 3, 'center_base_xyz'),
        )
        object.__setattr__(
            self,
            'tool0_position_xyz',
            _frozen_vector(self.tool0_position_xyz, 3, 'tool0_position_xyz'),
        )
        object.__setattr__(
            self,
            'quaternion_xyzw',
            _canonicalized_quaternion(self.quaternion_xyzw, 'quaternion_xyzw'),
        )
        object.__setattr__(
            self,
            'approach_base_xyz',
            _frozen_vector(
                self.approach_base_xyz, 3, 'approach_base_xyz', normalize=True
            ),
        )
        for name in ('required_open_width_m', 'model_width_m'):
            object.__setattr__(
                self, name, _validated_finite(getattr(self, name), name, positive=True)
            )
        for name in ('model_score', 'geometry_margin_m', 'pre_moveit_score'):
            object.__setattr__(
                self, name, _validated_finite(getattr(self, name), name)
            )


@dataclass(frozen=True)
class StableCandidate:
    track_id: int
    hit_count: int
    window_count: int
    hit_request_ids: Tuple[int, ...]
    request_id: int
    snapshot_stamp_sec: float
    target_epoch: int
    target_label: str
    model_choice: str
    center_base_xyz: np.ndarray
    tool0_position_xyz: np.ndarray
    quaternion_xyzw: np.ndarray
    approach_base_xyz: np.ndarray
    required_open_width_m: float
    model_width_m: float
    model_score: float
    geometry_margin_m: float
    pre_moveit_score: float
    payload: Any = field(compare=False, repr=False)
    moveit_cost: Optional[float] = None


class _CandidateTrack:
    def __init__(self, track_id, first_observation):
        self.track_id = track_id
        self.observations = {first_observation.request_id: first_observation}
        self._last_fused_quaternion = None

    @property
    def latest(self):
        return self.observations[max(self.observations)]

    def clone(self):
        cloned = object.__new__(_CandidateTrack)
        cloned.track_id = self.track_id
        cloned.observations = dict(self.observations)
        cloned._last_fused_quaternion = self._last_fused_quaternion
        return cloned


class CandidateTracker:
    def __init__(self, config=None):
        self.config = TrackingConfig() if config is None else config
        if not isinstance(self.config, TrackingConfig):
            raise TypeError('config must be a TrackingConfig')
        self._request_ids = deque(maxlen=self.config.window_size)
        self._tracks: Dict[int, _CandidateTrack] = {}
        self._next_track_id = 1
        self._last_request_id = None
        self._active_identity = None

    @property
    def track_count(self):
        return len(self._tracks)

    def update(self, request_id, observations, target_identity=None):
        request_id = _validated_integer(request_id, 'request_id', 1)
        if self._last_request_id is not None and request_id <= self._last_request_id:
            raise ValueError('request_id must be strictly increasing')
        explicit_identity = (
            None
            if target_identity is None
            else _validated_target_identity(target_identity)
        )
        batch = tuple(observations)
        for item in batch:
            if not isinstance(item, CandidateObservation):
                raise TypeError('observations must contain CandidateObservation values')
            if item.request_id != request_id:
                raise ValueError('observation request_id must match update request_id')
        if batch:
            identities = {
                (item.target_epoch, item.target_label, item.model_choice)
                for item in batch
            }
            if len(identities) != 1:
                raise ValueError(
                    'observations in one request must share a single target identity'
                )
            observed_identity = next(iter(identities))
        else:
            observed_identity = None
        if (
            explicit_identity is not None
            and observed_identity is not None
            and explicit_identity != observed_identity
        ):
            raise ValueError(
                'target_identity must exactly match every observation'
            )
        identity = (
            explicit_identity
            if explicit_identity is not None
            else observed_identity
        )

        snapshot = self._state_snapshot()
        try:
            if identity is not None:
                if self._active_identity is None:
                    self._active_identity = identity
                elif identity != self._active_identity:
                    self._request_ids.clear()
                    self._tracks.clear()
                    self._active_identity = identity

            pairs = []
            for track_id in sorted(self._tracks):
                track = self._tracks[track_id]
                for candidate_index, item in enumerate(batch):
                    cost = self._pair_cost(track.latest, item)
                    if cost is not None:
                        pairs.append((cost, track_id, candidate_index))
            pairs.sort(key=lambda pair: (pair[0], pair[1], pair[2]))

            assigned_tracks = set()
            assigned_candidates = set()
            for _cost, track_id, candidate_index in pairs:
                if (
                    track_id in assigned_tracks
                    or candidate_index in assigned_candidates
                ):
                    continue
                self._tracks[track_id].observations[request_id] = batch[
                    candidate_index
                ]
                assigned_tracks.add(track_id)
                assigned_candidates.add(candidate_index)

            for candidate_index, item in enumerate(batch):
                if candidate_index in assigned_candidates:
                    continue
                track_id = self._next_track_id
                self._next_track_id += 1
                self._tracks[track_id] = _CandidateTrack(track_id, item)

            self._last_request_id = request_id
            self._request_ids.append(request_id)
            self._evict_old_observations()
            stable, pending_continuity = self._compute_stable_candidates()
            for track_id, quaternion in pending_continuity.items():
                self._tracks[track_id]._last_fused_quaternion = quaternion
        except Exception:
            self._restore_state(snapshot)
            raise
        return stable

    def _state_snapshot(self):
        return (
            deque(self._request_ids, maxlen=self.config.window_size),
            {
                track_id: track.clone()
                for track_id, track in self._tracks.items()
            },
            self._next_track_id,
            self._last_request_id,
            self._active_identity,
        )

    def _restore_state(self, snapshot):
        (
            self._request_ids,
            self._tracks,
            self._next_track_id,
            self._last_request_id,
            self._active_identity,
        ) = snapshot

    def stable_candidates(self):
        stable, _pending_continuity = self._compute_stable_candidates()
        return stable

    def _compute_stable_candidates(self):
        stable = []
        pending_continuity = {}
        for track_id in sorted(self._tracks):
            track = self._tracks[track_id]
            if len(track.observations) >= self.config.min_hits:
                fused = self._fuse(track)
                stable.append(fused)
                pending_continuity[track_id] = fused.quaternion_xyzw
        return stable, pending_continuity

    def _pair_cost(self, first, second):
        if (
            first.target_epoch != second.target_epoch
            or first.target_label != second.target_label
            or first.model_choice != second.model_choice
        ):
            return None

        position_distance = float(
            np.linalg.norm(first.center_base_xyz - second.center_base_xyz)
        )
        orientation_distance, _aligned = parallel_jaw_orientation_distance_rad(
            first.quaternion_xyzw, second.quaternion_xyzw
        )
        approach_dot = min(
            1.0,
            max(-1.0, float(np.dot(first.approach_base_xyz, second.approach_base_xyz))),
        )
        approach_distance = math.acos(approach_dot)
        width_distance = abs(
            first.required_open_width_m - second.required_open_width_m
        )
        orientation_threshold = math.radians(
            self.config.orientation_threshold_deg
        )
        approach_threshold = math.radians(self.config.approach_threshold_deg)
        if (
            position_distance > self.config.position_threshold_m
            or orientation_distance > orientation_threshold
            or approach_distance > approach_threshold
            or width_distance > self.config.width_threshold_m
        ):
            return None
        return (
            position_distance / self.config.position_threshold_m
            + orientation_distance / orientation_threshold
            + approach_distance / approach_threshold
            + width_distance / self.config.width_threshold_m
        )

    def _evict_old_observations(self):
        retained_request_ids = set(self._request_ids)
        empty_track_ids = []
        for track_id, track in self._tracks.items():
            track.observations = {
                request_id: item
                for request_id, item in track.observations.items()
                if request_id in retained_request_ids
            }
            if not track.observations:
                empty_track_ids.append(track_id)
        for track_id in empty_track_ids:
            del self._tracks[track_id]

    @staticmethod
    def _fusion_weights(observations):
        count = len(observations)
        if count == 1:
            return np.ones(1, dtype=np.float64)
        freshness = np.linspace(1.0, 2.0, num=count, dtype=np.float64)
        confidence = np.maximum(
            np.asarray([item.model_score for item in observations]), 0.0
        )
        maximum_confidence = float(np.max(confidence))
        if maximum_confidence > 0.0:
            confidence = confidence / maximum_confidence
        weights = (confidence + 1e-6) * freshness

        largest_index = int(np.argmax(weights))
        other_total = math.fsum(
            float(weight)
            for index, weight in enumerate(weights)
            if index != largest_index
        )
        if count > 2 and weights[largest_index] >= other_total:
            strict_gap = max(
                other_total * 1e-12,
                4.0 * abs(float(np.spacing(other_total))),
            )
            weights[largest_index] = other_total - strict_gap
        elif weights[largest_index] > other_total:
            weights[largest_index] = other_total
        total = math.fsum(float(weight) for weight in weights)
        weights = weights / total
        if (
            not np.all(np.isfinite(weights))
            or np.any(weights <= 0.0)
            or not math.isclose(
                math.fsum(float(weight) for weight in weights),
                1.0,
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
        ):
            raise ValueError('fusion weights must be positive, finite, and normalized')
        return weights

    @staticmethod
    def _coordinate_weighted_median(vectors, weights, name):
        fused = np.array(
            [
                weighted_median(
                    [float(vector[axis]) for vector in vectors], weights
                )
                for axis in range(3)
            ],
            dtype=np.float64,
        )
        return _frozen_vector(fused, 3, name)

    @staticmethod
    def _fused_approach(observations, weights):
        weighted = np.sum(
            np.asarray(
                [item.approach_base_xyz for item in observations],
                dtype=np.float64,
            )
            * weights[:, np.newaxis],
            axis=0,
        )
        if float(np.linalg.norm(weighted)) <= _NORM_EPSILON:
            weighted = observations[-1].approach_base_xyz
        return _frozen_vector(
            weighted, 3, 'fused_approach_base_xyz', normalize=True
        )

    @staticmethod
    def _fused_orientation(observations, weights):
        quaternions = [item.quaternion_xyzw for item in observations]
        medoid_costs = []
        for index, reference in enumerate(quaternions):
            total_distance = sum(
                parallel_jaw_orientation_distance_rad(reference, candidate)[0]
                for candidate in quaternions
            )
            medoid_costs.append((total_distance, index))
        medoid_index = min(medoid_costs)[1]
        medoid = quaternions[medoid_index]

        aligned = []
        residuals = []
        for candidate in quaternions:
            distance, aligned_candidate = parallel_jaw_orientation_distance_rad(
                medoid, candidate
            )
            residuals.append(distance)
            aligned.append(aligned_candidate)

        residuals_array = np.asarray(residuals, dtype=np.float64)
        residual_median = float(np.median(residuals_array))
        residual_mad = float(
            np.median(np.abs(residuals_array - residual_median))
        )
        robust_scale = max(
            residual_median,
            1.4826 * residual_mad,
            math.radians(0.5),
        )
        tukey_cutoff = 4.685 * robust_scale
        ratios = residuals_array / tukey_cutoff
        robust_multipliers = np.where(
            ratios < 1.0,
            np.square(1.0 - np.square(ratios)),
            0.0,
        )
        robust_weights = weights * robust_multipliers
        if float(np.sum(robust_weights)) <= _NORM_EPSILON:
            return _canonicalized_quaternion(medoid)

        averaged = np.sum(
            np.asarray(aligned, dtype=np.float64)
            * robust_weights[:, np.newaxis],
            axis=0,
        )
        if float(np.linalg.norm(averaged)) <= _NORM_EPSILON:
            return _canonicalized_quaternion(medoid)
        return _canonicalized_quaternion(averaged, 'fused_quaternion_xyzw')

    def _fuse(self, track):
        newest = track.latest
        request_ids = tuple(sorted(track.observations))
        observations = [track.observations[item] for item in request_ids]
        weights = self._fusion_weights(observations)
        center_base_xyz = self._coordinate_weighted_median(
            [item.center_base_xyz for item in observations],
            weights,
            'fused_center_base_xyz',
        )
        tool0_position_xyz = self._coordinate_weighted_median(
            [item.tool0_position_xyz for item in observations],
            weights,
            'fused_tool0_position_xyz',
        )
        required_open_width_m = weighted_median(
            [item.required_open_width_m for item in observations], weights
        )
        model_width_m = weighted_median(
            [item.model_width_m for item in observations], weights
        )
        model_score = float(
            np.median([item.model_score for item in observations])
        )
        pre_moveit_score = float(
            np.median([item.pre_moveit_score for item in observations])
        )
        geometry_margin_m = float(
            np.quantile(
                [item.geometry_margin_m for item in observations], 0.25
            )
        )
        quaternion_xyzw = self._fused_orientation(observations, weights)
        if track._last_fused_quaternion is not None:
            _distance, quaternion_xyzw = parallel_jaw_orientation_distance_rad(
                track._last_fused_quaternion, quaternion_xyzw
            )
        return StableCandidate(
            track_id=track.track_id,
            hit_count=len(request_ids),
            window_count=len(self._request_ids),
            hit_request_ids=request_ids,
            request_id=newest.request_id,
            snapshot_stamp_sec=newest.snapshot_stamp_sec,
            target_epoch=newest.target_epoch,
            target_label=newest.target_label,
            model_choice=newest.model_choice,
            center_base_xyz=center_base_xyz,
            tool0_position_xyz=tool0_position_xyz,
            quaternion_xyzw=quaternion_xyzw,
            approach_base_xyz=self._fused_approach(observations, weights),
            required_open_width_m=required_open_width_m,
            model_width_m=model_width_m,
            model_score=model_score,
            geometry_margin_m=geometry_margin_m,
            pre_moveit_score=pre_moveit_score,
            payload=newest.payload,
            moveit_cost=None,
        )
