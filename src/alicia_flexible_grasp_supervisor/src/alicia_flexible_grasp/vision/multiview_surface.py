"""Deterministic registration and provenance for measured target surfaces.

This boundary intentionally has no semantic label, model, or known-object
input.  Every fused point originates in a validated measured ``SurfaceView``.
"""

from dataclasses import dataclass
from numbers import Integral, Real

import numpy as np

from .target_observation import (
    TargetObservation,
    TargetTrackIdentity,
    validate_target_identity,
)


DEFAULT_FUSION_VOXEL_SIZE_M = 0.0025
_NEAREST_NEIGHBOR_CHUNK = 512
_NEAREST_NEIGHBOR_TARGET_CHUNK = 2048
_TRIM_FRACTION = 0.10
_COARSE_FOOTPRINT_MIN_SPAN_RATIO = 0.75


def _strict_integer(value, name, minimum=0):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Integral):
        raise ValueError('%s must be an integer' % name)
    if int(value) < int(minimum):
        raise ValueError('%s must be >= %d' % (name, minimum))
    return int(value)


def _strict_number(
    value,
    name,
    minimum=None,
    maximum=None,
    minimum_inclusive=True,
):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real):
        raise ValueError('%s must be a real number' % name)
    number = float(value)
    if not np.isfinite(number):
        raise ValueError('%s must be finite' % name)
    if minimum is not None:
        below = number < minimum if minimum_inclusive else number <= minimum
        if below:
            raise ValueError('%s is below its bound' % name)
    if maximum is not None and number > maximum:
        raise ValueError('%s is above its bound' % name)
    return number


def _immutable_float_array(value, shape, name):
    array = np.asarray(value, dtype=np.float64)
    if array.shape != shape or not np.all(np.isfinite(array)):
        raise ValueError('%s has invalid shape or non-finite values' % name)
    return np.frombuffer(array.tobytes(), dtype=np.float64).reshape(shape)


def _immutable_integer_array(value, shape, name):
    array = np.asarray(value)
    if array.shape != shape or array.dtype.kind not in 'iu':
        raise ValueError('%s must contain integers with shape %r' % (name, shape))
    converted = np.asarray(array, dtype=np.int64)
    return np.frombuffer(converted.tobytes(), dtype=np.int64).reshape(shape)


def _immutable_rigid_transform(value):
    transform = _immutable_float_array(value, (4, 4), 'transform_base')
    rotation = transform[:3, :3]
    if (
        not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], rtol=0, atol=1e-9)
        or not np.allclose(rotation.T.dot(rotation), np.eye(3), rtol=0, atol=1e-8)
        or not np.isclose(np.linalg.det(rotation), 1.0, rtol=0, atol=1e-8)
    ):
        raise ValueError('transform_base must be rigid')
    return transform


def maximum_point_displacement_m(points_base, transform_base):
    """Measure the largest rigid correction of any supplied measured point.

    Unlike the raw transform translation, this metric is independent of the
    base origin and includes displacement caused by rotation. Consumers can
    recompute it from their exact source observation before applying a result.
    """

    points = np.asarray(points_base, dtype=np.float64)
    if (
        points.ndim != 2
        or points.shape[1:] != (3,)
        or not len(points)
        or not np.all(np.isfinite(points))
    ):
        raise ValueError('points_base must contain finite Nx3 measured points')
    transform = _immutable_rigid_transform(transform_base)
    # (R - I) p + t avoids subtracting two full base-coordinate positions.
    displacement = points.dot((transform[:3, :3] - np.eye(3)).T) + transform[:3, 3]
    return _strict_number(
        float(np.max(np.linalg.norm(displacement, axis=1))),
        'maximum_point_displacement_m',
        minimum=0.0,
    )


@dataclass(frozen=True)
class SurfaceView:
    identity: TargetTrackIdentity
    stamp_ns: int
    points_base: np.ndarray
    support_normal_base: np.ndarray
    support_offset_m: float
    edge_clearance_px: int

    def __post_init__(self):
        validate_target_identity(self.identity)
        object.__setattr__(
            self, 'stamp_ns', _strict_integer(self.stamp_ns, 'stamp_ns', 1)
        )
        points = np.asarray(self.points_base, dtype=np.float64)
        if points.ndim != 2 or points.shape[1:] != (3,) or not len(points):
            raise ValueError('points_base must contain finite Nx3 measured points')
        object.__setattr__(
            self,
            'points_base',
            _immutable_float_array(points, points.shape, 'points_base'),
        )
        normal = _immutable_float_array(
            self.support_normal_base, (3,), 'support_normal_base'
        )
        if not np.isclose(np.linalg.norm(normal), 1.0, rtol=0.0, atol=1e-6):
            raise ValueError('support_normal_base must be a unit normal')
        object.__setattr__(self, 'support_normal_base', normal)
        object.__setattr__(
            self,
            'support_offset_m',
            _strict_number(self.support_offset_m, 'support_offset_m'),
        )
        object.__setattr__(
            self,
            'edge_clearance_px',
            _strict_integer(self.edge_clearance_px, 'edge_clearance_px'),
        )

    @property
    def is_clipped(self):
        return self.edge_clearance_px == 0


def support_plane_separation_m(reference, moving):
    """Compare measured support planes at a shared target-local anchor."""

    if not isinstance(reference, SurfaceView) or not isinstance(moving, SurfaceView):
        raise ValueError('reference and moving must be SurfaceView instances')
    anchor = 0.5 * (
        np.median(reference.points_base, axis=0)
        + np.median(moving.points_base, axis=0)
    )
    return abs(float(
        np.dot(reference.support_normal_base - moving.support_normal_base, anchor)
        + reference.support_offset_m - moving.support_offset_m
    ))


@dataclass(frozen=True)
class RegistrationConfig:
    correspondence_max_m: float = 0.008
    minimum_inliers: int = 80
    minimum_overlap_fraction: float = 0.30
    maximum_rmse_m: float = 0.004
    # Bound every moving measured point's displacement under the correction.
    maximum_translation_m: float = 0.025
    maximum_yaw_deg: float = 10.0
    maximum_support_normal_angle_deg: float = 4.0
    maximum_support_offset_delta_m: float = 0.004
    maximum_iterations: int = 12

    def __post_init__(self):
        object.__setattr__(self, 'correspondence_max_m', _strict_number(
            self.correspondence_max_m, 'correspondence_max_m',
            minimum=0.0, minimum_inclusive=False))
        object.__setattr__(self, 'minimum_inliers', _strict_integer(
            self.minimum_inliers, 'minimum_inliers', 1))
        object.__setattr__(self, 'minimum_overlap_fraction', _strict_number(
            self.minimum_overlap_fraction, 'minimum_overlap_fraction',
            minimum=0.0, maximum=1.0, minimum_inclusive=False))
        object.__setattr__(self, 'maximum_rmse_m', _strict_number(
            self.maximum_rmse_m, 'maximum_rmse_m',
            minimum=0.0, minimum_inclusive=False))
        object.__setattr__(self, 'maximum_translation_m', _strict_number(
            self.maximum_translation_m, 'maximum_translation_m',
            minimum=0.0, minimum_inclusive=False))
        object.__setattr__(self, 'maximum_yaw_deg', _strict_number(
            self.maximum_yaw_deg, 'maximum_yaw_deg',
            minimum=0.0, maximum=180.0, minimum_inclusive=False))
        object.__setattr__(
            self,
            'maximum_support_normal_angle_deg',
            _strict_number(
                self.maximum_support_normal_angle_deg,
                'maximum_support_normal_angle_deg',
                minimum=0.0,
                maximum=180.0,
            ),
        )
        object.__setattr__(
            self,
            'maximum_support_offset_delta_m',
            _strict_number(
                self.maximum_support_offset_delta_m,
                'maximum_support_offset_delta_m',
                minimum=0.0,
            ),
        )
        object.__setattr__(self, 'maximum_iterations', _strict_integer(
            self.maximum_iterations, 'maximum_iterations', 1))


@dataclass(frozen=True)
class RegistrationResult:
    ok: bool
    code: str
    transform_base: np.ndarray
    inlier_count: int
    overlap_fraction: float
    rmse_m: float
    support_normal_angle_deg: float
    support_plane_separation_m: float
    maximum_point_displacement_m: float = 0.0

    def __post_init__(self):
        if type(self.ok) is not bool:
            raise ValueError('ok must be a boolean')
        if not isinstance(self.code, str) or not self.code:
            raise ValueError('code must be a non-empty string')
        transform = _immutable_rigid_transform(self.transform_base)
        object.__setattr__(self, 'transform_base', transform)
        object.__setattr__(self, 'inlier_count', _strict_integer(
            self.inlier_count, 'inlier_count'))
        object.__setattr__(self, 'overlap_fraction', _strict_number(
            self.overlap_fraction, 'overlap_fraction', minimum=0.0, maximum=1.0))
        object.__setattr__(self, 'rmse_m', _strict_number(
            self.rmse_m, 'rmse_m', minimum=0.0))
        object.__setattr__(
            self,
            'support_normal_angle_deg',
            _strict_number(
                self.support_normal_angle_deg,
                'support_normal_angle_deg',
                minimum=0.0,
                maximum=180.0,
            ),
        )
        object.__setattr__(
            self,
            'support_plane_separation_m',
            _strict_number(
                self.support_plane_separation_m,
                'support_plane_separation_m',
                minimum=0.0,
            ),
        )
        object.__setattr__(
            self,
            'maximum_point_displacement_m',
            _strict_number(
                self.maximum_point_displacement_m,
                'maximum_point_displacement_m',
                minimum=0.0,
            ),
        )


@dataclass(frozen=True)
class FusedTargetSurface:
    identity: TargetTrackIdentity
    points_base: np.ndarray
    view_indices: np.ndarray
    view_stamps_ns: tuple

    def __post_init__(self):
        validate_target_identity(self.identity)
        points = np.asarray(self.points_base, dtype=np.float64)
        if points.ndim != 2 or points.shape[1:] != (3,) or not len(points):
            raise ValueError('points_base must contain finite Nx3 measured points')
        object.__setattr__(
            self, 'points_base',
            _immutable_float_array(points, points.shape, 'points_base'))
        indices = _immutable_integer_array(
            self.view_indices, (len(points),), 'view_indices')
        stamps = tuple(
            _strict_integer(value, 'view stamp', 1)
            for value in tuple(self.view_stamps_ns)
        )
        if not stamps or any(second <= first for first, second in zip(stamps, stamps[1:])):
            raise ValueError('view_stamps_ns must be strictly increasing')
        if np.any(indices < 0) or np.any(indices >= len(stamps)):
            raise ValueError('view_indices must reference view_stamps_ns')
        object.__setattr__(self, 'view_indices', indices)
        object.__setattr__(self, 'view_stamps_ns', stamps)


def surface_view_from_observation(observation):
    if not isinstance(observation, TargetObservation):
        raise ValueError('observation must be a TargetObservation')
    return SurfaceView(
        identity=observation.identity,
        stamp_ns=observation.stamp_ns,
        points_base=observation.points_base,
        support_normal_base=observation.support_normal_base,
        support_offset_m=observation.support_offset_m,
        edge_clearance_px=observation.edge_clearance_px,
    )


def _support_basis(normal):
    normal = np.asarray(normal, dtype=np.float64).reshape(3)
    seed = np.eye(3)[int(np.argmin(np.abs(normal)))]
    first = np.cross(seed, normal)
    first /= np.linalg.norm(first)
    second = np.cross(normal, first)
    second /= np.linalg.norm(second)
    return np.column_stack((first, second, normal))


def _sorted_points(points):
    # Repeated samples are one spatial measurement, never independent votes.
    points = np.unique(np.asarray(points, dtype=np.float64), axis=0)
    order = np.lexsort((points[:, 2], points[:, 1], points[:, 0]))
    return points[order]


def _nearest_neighbors(source, target):
    source = np.asarray(source, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    indices = np.empty(len(source), dtype=np.int64)
    distances_squared = np.empty(len(source), dtype=np.float64)
    for start in range(0, len(source), _NEAREST_NEIGHBOR_CHUNK):
        stop = min(start + _NEAREST_NEIGHBOR_CHUNK, len(source))
        source_chunk = source[start:stop]
        best_indices = np.zeros(stop - start, dtype=np.int64)
        best_distances = np.full(stop - start, np.inf, dtype=np.float64)
        for target_start in range(0, len(target), _NEAREST_NEIGHBOR_TARGET_CHUNK):
            target_stop = min(
                target_start + _NEAREST_NEIGHBOR_TARGET_CHUNK, len(target)
            )
            difference = (
                source_chunk[:, None, :]
                - target[None, target_start:target_stop, :]
            )
            chunk_distances = np.einsum(
                'ijk,ijk->ij', difference, difference
            )
            local_indices = np.argmin(chunk_distances, axis=1)
            local_distances = chunk_distances[
                np.arange(stop - start), local_indices
            ]
            # Strict comparison preserves the lowest target index on ties,
            # independently of chunk size.
            improved = local_distances < best_distances
            best_distances[improved] = local_distances[improved]
            best_indices[improved] = target_start + local_indices[improved]
        indices[start:stop] = best_indices
        distances_squared[start:stop] = best_distances
    return indices, np.sqrt(distances_squared)


def _yaw_degrees(rotation, basis):
    local = basis.T.dot(rotation).dot(basis)
    return float(np.degrees(np.arctan2(local[1, 0], local[0, 0])))


def _principal_yaw(points_local):
    centered = points_local[:, :2] - np.mean(points_local[:, :2], axis=0)
    covariance = centered.T.dot(centered) / float(max(1, len(centered) - 1))
    _values, vectors = np.linalg.eigh(covariance)
    axis = vectors[:, -1]
    return float(np.arctan2(axis[1], axis[0]))


def _initial_support_transform(reference, moving, basis):
    """Align the unoriented measured footprint axes before local ICP."""

    reference_local = reference.dot(basis)
    moving_local = moving.dot(basis)
    yaw = _principal_yaw(reference_local) - _principal_yaw(moving_local)
    yaw = (yaw + 0.5 * np.pi) % np.pi - 0.5 * np.pi
    cosine = np.cos(yaw)
    sine = np.sin(yaw)
    rotation_local = np.asarray(
        [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]]
    )
    rotated_local = moving_local.dot(rotation_local.T)
    reference_midpoint = 0.5 * (
        np.min(reference_local[:, :2], axis=0)
        + np.max(reference_local[:, :2], axis=0)
    )
    moving_midpoint = 0.5 * (
        np.min(rotated_local[:, :2], axis=0)
        + np.max(rotated_local[:, :2], axis=0)
    )
    translation_local = np.asarray(
        [
            reference_midpoint[0] - moving_midpoint[0],
            reference_midpoint[1] - moving_midpoint[1],
            np.median(reference_local[:, 2]) - np.median(rotated_local[:, 2]),
        ]
    )
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = basis.dot(rotation_local).dot(basis.T)
    transform[:3, 3] = basis.dot(translation_local)
    return transform


def _footprint_spans_enough_for_coarse_alignment(reference, moving, basis):
    """Reject PCA yaw from narrow fragments whose dominant axis can flip."""

    reference_xy = np.asarray(reference).dot(basis)[:, :2]
    moving_xy = np.asarray(moving).dot(basis)[:, :2]
    reference_span = np.ptp(reference_xy, axis=0)
    moving_span = np.ptp(moving_xy, axis=0)
    required = _COARSE_FOOTPRINT_MIN_SPAN_RATIO * reference_span
    return bool(np.all(moving_span >= required))


def _rigid_update_about_support(source, target, basis):
    source_local = np.asarray(source).dot(basis)
    target_local = np.asarray(target).dot(basis)
    source_center = np.mean(source_local, axis=0)
    target_center = np.mean(target_local, axis=0)
    source_xy = source_local[:, :2] - source_center[:2]
    target_xy = target_local[:, :2] - target_center[:2]
    left, _singular, right_t = np.linalg.svd(source_xy.T.dot(target_xy))
    rotation_xy = right_t.T.dot(left.T)
    if np.linalg.det(rotation_xy) < 0.0:
        right_t[-1, :] *= -1.0
        rotation_xy = right_t.T.dot(left.T)
    rotation_local = np.eye(3)
    rotation_local[:2, :2] = rotation_xy
    translation_local = target_center - rotation_local.dot(source_center)
    rotation_base = basis.dot(rotation_local).dot(basis.T)
    translation_base = basis.dot(translation_local)
    return rotation_base, translation_base


def _failure(
    code,
    transform=None,
    inlier_count=0,
    overlap_fraction=0.0,
    rmse_m=0.0,
    support_normal_angle_deg=0.0,
    support_plane_separation_m=0.0,
    maximum_point_displacement_m=0.0,
):
    return RegistrationResult(
        ok=False,
        code=str(code),
        transform_base=(np.eye(4) if transform is None else transform),
        inlier_count=int(inlier_count),
        overlap_fraction=float(overlap_fraction),
        rmse_m=float(rmse_m),
        support_normal_angle_deg=float(support_normal_angle_deg),
        support_plane_separation_m=float(support_plane_separation_m),
        maximum_point_displacement_m=float(maximum_point_displacement_m),
    )


def _registration_correspondences(reference, moving, transform, config):
    transformed = moving.dot(transform[:3, :3].T) + transform[:3, 3]
    indices, distances = _nearest_neighbors(transformed, reference)
    inliers = distances <= config.correspondence_max_m
    # One nearest moving sample per reference point. Distance wins first;
    # canonical moving order resolves ties independently of input ordering.
    candidates = np.flatnonzero(inliers)
    order = candidates[np.lexsort((candidates, distances[candidates], indices[candidates]))]
    first = np.ones(len(order), dtype=bool)
    first[1:] = indices[order[1:]] != indices[order[:-1]]
    inliers[:] = False
    inliers[order[first]] = True
    return transformed, indices, distances, inliers


def _connected_measurement_mask(points, seed_mask, config):
    """Return measured radius-graph components containing correspondence seeds."""

    points = np.asarray(points, dtype=np.float64)
    seeds = np.asarray(seed_mask, dtype=bool)
    if points.ndim != 2 or points.shape[1:] != (3,):
        raise ValueError('points must have shape Nx3')
    if seeds.shape != (len(points),):
        raise ValueError('seed_mask must have shape N')
    if not isinstance(config, RegistrationConfig):
        raise ValueError('config must be a RegistrationConfig')

    connected = seeds.copy()
    if not np.any(connected):
        return connected

    radius = config.correspondence_max_m
    frontier = np.flatnonzero(connected)
    remaining = np.flatnonzero(~connected)
    while len(frontier) and len(remaining):
        reached = np.zeros(len(remaining), dtype=bool)
        for start in range(0, len(remaining), _NEAREST_NEIGHBOR_CHUNK):
            stop = min(start + _NEAREST_NEIGHBOR_CHUNK, len(remaining))
            candidate_points = points[remaining[start:stop]]
            for frontier_start in range(
                0, len(frontier), _NEAREST_NEIGHBOR_TARGET_CHUNK
            ):
                frontier_stop = min(
                    frontier_start + _NEAREST_NEIGHBOR_TARGET_CHUNK,
                    len(frontier),
                )
                differences = (
                    candidate_points[:, None, :]
                    - points[frontier[frontier_start:frontier_stop]][None, :, :]
                )
                distances = np.linalg.norm(differences, axis=2)
                reached[start:stop] |= np.any(distances <= radius, axis=1)
                if np.all(reached[start:stop]):
                    break
        frontier = remaining[reached]
        connected[frontier] = True
        remaining = remaining[~reached]
    return connected


def _has_planar_support(points, basis):
    if len(points) < 3:
        return False
    xy = np.asarray(points).dot(basis)[:, :2]
    return np.linalg.matrix_rank(xy - np.mean(xy, axis=0)) == 2


def register_surface_view(reference, moving, config=None):
    """Register ``moving`` to ``reference`` using bounded support-yaw ICP."""

    if not isinstance(reference, SurfaceView) or not isinstance(moving, SurfaceView):
        raise ValueError('reference and moving must be SurfaceView instances')
    config = RegistrationConfig() if config is None else config
    if not isinstance(config, RegistrationConfig):
        raise ValueError('config must be a RegistrationConfig')
    if reference.identity != moving.identity:
        return _failure('IDENTITY_MISMATCH')
    if moving.stamp_ns <= reference.stamp_ns:
        return _failure('STAMP_NOT_MONOTONIC')
    dot = float(np.dot(reference.support_normal_base, moving.support_normal_base))
    normal_angle = float(np.degrees(np.arccos(np.clip(dot, -1.0, 1.0))))
    # Raw plane offsets are relative to the coordinate origin. A small normal
    # fit change can move that coefficient by millimetres for a distant target
    # even when both planes agree locally. Compare signed distances at a robust
    # target-local anchor so this gate measures physical plane separation.
    support_plane_separation = support_plane_separation_m(reference, moving)
    def reject(*args, **kwargs):
        kwargs['support_plane_separation_m'] = support_plane_separation
        rejected_transform = kwargs.get('transform')
        if rejected_transform is None and len(args) > 1:
            rejected_transform = args[1]
        kwargs['maximum_point_displacement_m'] = (
            0.0 if rejected_transform is None else maximum_point_displacement_m(
                moving.points_base, rejected_transform
            )
        )
        return _failure(*args, **kwargs)

    if normal_angle > config.maximum_support_normal_angle_deg:
        return reject(
            'SUPPORT_NORMAL_MISMATCH',
            support_normal_angle_deg=normal_angle,
        )
    if (
        support_plane_separation
        > config.maximum_support_offset_delta_m
    ):
        return reject(
            'SUPPORT_OFFSET_MISMATCH',
            support_normal_angle_deg=normal_angle,
        )

    reference_points = _sorted_points(reference.points_base)
    moving_points = _sorted_points(moving.points_base)
    basis = _support_basis(reference.support_normal_base)
    transform = np.eye(4, dtype=np.float64)

    # A support-plane footprint supplies a deterministic coarse yaw/translation
    # only when the raw measured clouds already overlap enough to justify it.
    # This avoids allowing distant outliers to manufacture an initial pose.
    _initial_points, _initial_indices, initial_distances, initial_inliers = (
        _registration_correspondences(
            reference_points, moving_points, transform, config
        )
    )
    if (
        np.count_nonzero(initial_inliers) >= 3
        and float(np.count_nonzero(initial_inliers)) / len(moving_points)
        >= config.minimum_overlap_fraction
        and _footprint_spans_enough_for_coarse_alignment(
            reference_points, moving_points, basis
        )
    ):
        coarse_transform = _initial_support_transform(
            reference_points, moving_points, basis
        )
        _points, _indices, coarse_distances, coarse_inliers = (
            _registration_correspondences(
                reference_points, moving_points, coarse_transform, config)
        )
        # PCA describes sampling density as well as object orientation. A
        # missing corner or newly visible face can rotate that axis on a
        # stationary object. Compare both initial guesses against the same
        # measured correspondences before allowing PCA to replace identity.
        # Unmatched/duplicate correspondences receive the full distance cost,
        # so collapsing onto a small matching subset cannot improve the score.
        # Selection never depends on the motion bounds; a better-fitting
        # out-of-bounds correction is still rejected below.
        unmatched_cost = config.correspondence_max_m ** 2
        initial_cost = float(np.mean(np.where(
            initial_inliers, initial_distances ** 2, unmatched_cost)))
        coarse_cost = float(np.mean(np.where(
            coarse_inliers, coarse_distances ** 2, unmatched_cost)))
        if coarse_cost <= initial_cost:
            transform = coarse_transform
        yaw_deg = abs(_yaw_degrees(transform[:3, :3], basis))
        translation_m = maximum_point_displacement_m(moving.points_base, transform)
        if yaw_deg > config.maximum_yaw_deg:
            return reject(
                'YAW_BOUND_EXCEEDED', transform=transform,
                support_normal_angle_deg=normal_angle)
        if translation_m > config.maximum_translation_m:
            return reject(
                'TRANSLATION_BOUND_EXCEEDED', transform=transform,
                support_normal_angle_deg=normal_angle)

    for _iteration in range(config.maximum_iterations):
        transformed, indices, distances, inliers = _registration_correspondences(
            reference_points, moving_points, transform, config)
        inlier_positions = np.flatnonzero(inliers)
        if len(inlier_positions) < 3:
            break
        inlier_distances = distances[inlier_positions]
        # Nearest order statistic works with the platform NumPy 1.17 too.
        trim_index = int(round((len(inlier_distances) - 1) * (1.0 - _TRIM_FRACTION)))
        trim_limit = np.sort(inlier_distances)[trim_index]
        # Treat sub-femtometer ties equally only inside the already strictly
        # gated inlier set; this never expands a configured acceptance bound.
        kept = inlier_positions[inlier_distances <= trim_limit + 1e-15]
        if len(kept) < 3:
            break
        if (not _has_planar_support(transformed[kept], basis)
                or not _has_planar_support(reference_points[indices[kept]], basis)):
            return reject('DEGENERATE_SUPPORT', transform=transform,
                            support_normal_angle_deg=normal_angle)
        update_rotation, update_translation = _rigid_update_about_support(
            transformed[kept], reference_points[indices[kept]], basis
        )
        updated = np.eye(4, dtype=np.float64)
        updated[:3, :3] = update_rotation.dot(transform[:3, :3])
        updated[:3, 3] = (
            update_rotation.dot(transform[:3, 3]) + update_translation
        )
        yaw_deg = abs(_yaw_degrees(updated[:3, :3], basis))
        translation_m = maximum_point_displacement_m(moving.points_base, updated)
        transform = updated
        if yaw_deg > config.maximum_yaw_deg:
            return reject(
                'YAW_BOUND_EXCEEDED', transform=transform,
                support_normal_angle_deg=normal_angle)
        if translation_m > config.maximum_translation_m:
            return reject(
                'TRANSLATION_BOUND_EXCEEDED', transform=transform,
                support_normal_angle_deg=normal_angle)
        if (
            abs(_yaw_degrees(update_rotation, basis)) <= 1e-7
            and np.linalg.norm(update_translation) <= 1e-9
        ):
            break

    _transformed, _indices, distances, inliers = _registration_correspondences(
        reference_points, moving_points, transform, config)
    if (not _has_planar_support(_transformed[inliers], basis)
            or not _has_planar_support(reference_points[_indices[inliers]], basis)):
        return reject('DEGENERATE_SUPPORT', transform=transform,
                        support_normal_angle_deg=normal_angle)
    inlier_count = int(np.count_nonzero(inliers))
    overlap = float(inlier_count) / float(len(moving_points))
    rmse = (
        float(np.sqrt(np.mean(np.square(distances[inliers]))))
        if inlier_count
        else 0.0
    )
    yaw_deg = abs(_yaw_degrees(transform[:3, :3], basis))
    translation_m = maximum_point_displacement_m(moving.points_base, transform)
    if yaw_deg > config.maximum_yaw_deg:
        return reject(
            'YAW_BOUND_EXCEEDED', transform, inlier_count, overlap, rmse, normal_angle)
    if translation_m > config.maximum_translation_m:
        return reject(
            'TRANSLATION_BOUND_EXCEEDED', transform, inlier_count, overlap, rmse, normal_angle)
    if inlier_count < config.minimum_inliers:
        return reject(
            'INLIERS_INSUFFICIENT', transform, inlier_count, overlap, rmse, normal_angle)
    if overlap < config.minimum_overlap_fraction:
        return reject(
            'OVERLAP_INSUFFICIENT', transform, inlier_count, overlap, rmse, normal_angle)
    if rmse > config.maximum_rmse_m:
        return reject(
            'RMSE_EXCEEDED', transform, inlier_count, overlap, rmse, normal_angle)
    return RegistrationResult(
        True,
        'REGISTERED',
        transform,
        inlier_count,
        overlap,
        rmse,
        normal_angle,
        support_plane_separation,
        translation_m,
    )


def _voxel_deduplicate(points, view_indices, voxel_size_m):
    voxel_size = _strict_number(
        voxel_size_m, 'voxel_size_m', minimum=0.0, minimum_inclusive=False)
    points = np.asarray(points, dtype=np.float64)
    provenance = np.asarray(view_indices, dtype=np.int64)
    keys = np.floor(points / voxel_size).astype(np.int64)
    # Newer views win a duplicate voxel, retaining the accepted registration's
    # provenance while the older view still owns every distinct measured voxel.
    order = np.lexsort((
        points[:, 2], points[:, 1], points[:, 0], -provenance,
        keys[:, 2], keys[:, 1], keys[:, 0],
    ))
    sorted_keys = keys[order]
    first = np.ones(len(order), dtype=bool)
    first[1:] = np.any(sorted_keys[1:] != sorted_keys[:-1], axis=1)
    selected = order[first]
    selected_points = points[selected]
    selected_provenance = provenance[selected]
    final_order = np.lexsort((
        selected_provenance,
        selected_points[:, 2],
        selected_points[:, 1],
        selected_points[:, 0],
    ))
    return selected_points[final_order], selected_provenance[final_order]


def fused_surface_from_view(view, voxel_size_m=DEFAULT_FUSION_VOXEL_SIZE_M):
    if not isinstance(view, SurfaceView):
        raise ValueError('view must be a SurfaceView')
    points, provenance = _voxel_deduplicate(
        view.points_base,
        np.zeros(len(view.points_base), dtype=np.int64),
        voxel_size_m,
    )
    return FusedTargetSurface(
        identity=view.identity,
        points_base=points,
        view_indices=provenance,
        view_stamps_ns=(view.stamp_ns,),
    )


def append_registered_view(
    surface,
    reference,
    moving,
    config=None,
    voxel_size_m=DEFAULT_FUSION_VOXEL_SIZE_M,
):
    """Register a view and append its seed-connected measured components."""

    if not isinstance(surface, FusedTargetSurface):
        raise ValueError('surface must be a FusedTargetSurface')
    if not isinstance(reference, SurfaceView) or not isinstance(moving, SurfaceView):
        raise ValueError('reference and moving must be SurfaceView instances')
    if surface.identity != reference.identity or surface.view_stamps_ns[0] != reference.stamp_ns:
        return _failure('REFERENCE_SURFACE_MISMATCH'), surface
    if moving.identity != surface.identity:
        return _failure('IDENTITY_MISMATCH'), surface
    if moving.stamp_ns <= surface.view_stamps_ns[-1]:
        return _failure('STAMP_NOT_MONOTONIC'), surface
    config = RegistrationConfig() if config is None else config
    result = register_surface_view(reference, moving, config)
    if not result.ok:
        return result, surface

    transformed, _indices, _distances, inliers = _registration_correspondences(
        _sorted_points(reference.points_base), _sorted_points(moving.points_base),
        result.transform_base, config,
    )
    fusion_members = _connected_measurement_mask(transformed, inliers, config)
    new_index = len(surface.view_stamps_ns)
    combined_points = np.vstack((surface.points_base, transformed[fusion_members]))
    combined_indices = np.concatenate((
        surface.view_indices,
        np.full(np.count_nonzero(fusion_members), new_index, dtype=np.int64),
    ))
    points, provenance = _voxel_deduplicate(
        combined_points, combined_indices, voxel_size_m)
    return result, FusedTargetSurface(
        identity=surface.identity,
        points_base=points,
        view_indices=provenance,
        view_stamps_ns=surface.view_stamps_ns + (moving.stamp_ns,),
    )
