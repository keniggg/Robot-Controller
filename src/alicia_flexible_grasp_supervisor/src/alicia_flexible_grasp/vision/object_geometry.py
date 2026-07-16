from dataclasses import dataclass

import cv2
import numpy as np


_VALID_SOURCE_MODES = frozenset(('instance_mask', 'bbox_depth'))
_MIN_SUPPORT_INLIER_RATIO = 0.35


def _readonly_copy(value, shape=None):
    output = np.asarray(value, dtype=float).copy()
    if shape is not None:
        output = output.reshape(shape)
    output.setflags(write=False)
    return output


@dataclass(frozen=True)
class GeometryEstimate:
    ok: bool
    failure_code: str
    failure_reason: str
    center_base: np.ndarray
    axes_base: np.ndarray
    size_xyz_m: np.ndarray
    support_normal_base: np.ndarray
    support_offset_m: float
    support_inlier_ratio: float
    object_points_base: np.ndarray
    source_mode: str

    def __post_init__(self):
        object.__setattr__(self, 'center_base', _readonly_copy(self.center_base, (3,)))
        object.__setattr__(self, 'axes_base', _readonly_copy(self.axes_base, (3, 3)))
        object.__setattr__(self, 'size_xyz_m', _readonly_copy(self.size_xyz_m, (3,)))
        object.__setattr__(
            self,
            'support_normal_base',
            _readonly_copy(self.support_normal_base, (3,)),
        )
        points = np.asarray(self.object_points_base, dtype=float)
        if points.size == 0:
            points = np.zeros((0, 3), dtype=float)
        object.__setattr__(self, 'object_points_base', _readonly_copy(points, (-1, 3)))


def _failure(code, reason, source_mode):
    return GeometryEstimate(
        ok=False,
        failure_code=str(code),
        failure_reason=str(reason),
        center_base=np.zeros(3, dtype=float),
        axes_base=np.eye(3, dtype=float),
        size_xyz_m=np.zeros(3, dtype=float),
        support_normal_base=np.zeros(3, dtype=float),
        support_offset_m=0.0,
        support_inlier_ratio=0.0,
        object_points_base=np.zeros((0, 3), dtype=float),
        source_mode=str(source_mode or ''),
    )


def _intrinsic_values(intrinsics, image_shape=None):
    try:
        width = int(intrinsics.width)
        height = int(intrinsics.height)
        fx = float(intrinsics.fx)
        fy = float(intrinsics.fy)
        cx = float(intrinsics.cx)
        cy = float(intrinsics.cy)
    except Exception as exc:
        raise ValueError('intrinsics fields are unavailable') from exc
    values = np.asarray([width, height, fx, fy, cx, cy], dtype=float)
    if not np.all(np.isfinite(values)):
        raise ValueError('intrinsics contain non-finite values')
    if width <= 0 or height <= 0 or fx <= 0.0 or fy <= 0.0:
        raise ValueError('intrinsics dimensions and focal lengths must be positive')
    if image_shape is not None and (height, width) != tuple(image_shape):
        raise ValueError(
            'intrinsics image shape %s does not match depth shape %s'
            % ((height, width), tuple(image_shape))
        )
    return width, height, fx, fy, cx, cy


def deproject_depth(depth_raw, pixel_mask, intrinsics, depth_scale):
    """Return Nx3 camera-frame points and matching integer uv pixels."""
    depth = np.asarray(depth_raw)
    mask = np.asarray(pixel_mask)
    if depth.ndim != 2:
        raise ValueError('depth shape must be two-dimensional')
    if mask.shape != depth.shape:
        raise ValueError('pixel mask shape must match depth shape')
    _width, _height, fx, fy, cx, cy = _intrinsic_values(intrinsics, depth.shape)
    scale = float(depth_scale)
    if not np.isfinite(scale) or scale <= 0.0:
        raise ValueError('depth_scale must be finite and positive')
    valid = (mask > 0) & np.isfinite(depth) & (depth > 0)
    v, u = np.nonzero(valid)
    if u.size == 0:
        return np.zeros((0, 3), dtype=float), np.zeros((0, 2), dtype=np.int64)
    z = depth[v, u].astype(np.float64) * scale
    points = np.column_stack(
        [
            (u.astype(np.float64) - cx) * z / fx,
            (v.astype(np.float64) - cy) * z / fy,
            z,
        ]
    )
    uv = np.column_stack([u, v]).astype(np.int64, copy=False)
    return points, uv


def _validated_transform(T_base_camera):
    transform = np.asarray(T_base_camera, dtype=float)
    if transform.shape != (4, 4):
        raise ValueError('transform must have shape (4, 4)')
    if not np.all(np.isfinite(transform)):
        raise ValueError('transform contains non-finite values')
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-7):
        raise ValueError('transform bottom row is not homogeneous')
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5):
        raise ValueError('transform rotation is not orthonormal')
    determinant = float(np.linalg.det(rotation))
    if not np.isfinite(determinant) or abs(determinant - 1.0) > 1e-5:
        raise ValueError('transform rotation determinant is not +1')
    return transform


def _transform_points(points_camera, transform):
    points = np.asarray(points_camera, dtype=float).reshape(-1, 3)
    return points @ transform[:3, :3].T + transform[:3, 3]


def _expanded_bbox_mask(shape, bbox, expand_ratio):
    height, width = shape
    try:
        x, y, box_width, box_height = [int(value) for value in bbox]
    except Exception as exc:
        raise ValueError('bbox must contain four integer-compatible values') from exc
    if box_width <= 0 or box_height <= 0:
        raise ValueError('bbox dimensions must be positive')
    ratio = float(expand_ratio)
    if not np.isfinite(ratio) or ratio < 0.0:
        raise ValueError('support bbox expand ratio must be finite and non-negative')
    pad_x = int(np.ceil(box_width * ratio))
    pad_y = int(np.ceil(box_height * ratio))
    x0 = max(0, x - pad_x)
    y0 = max(0, y - pad_y)
    x1 = min(width, x + box_width + pad_x)
    y1 = min(height, y + box_height + pad_y)
    if x1 <= x0 or y1 <= y0:
        raise ValueError('bbox is outside the depth image')
    expanded = np.zeros(shape, dtype=bool)
    expanded[y0:y1, x0:x1] = True
    bbox_mask = np.zeros(shape, dtype=bool)
    bx0 = min(width, max(0, x))
    by0 = min(height, max(0, y))
    bx1 = min(width, max(bx0, x + box_width))
    by1 = min(height, max(by0, y + box_height))
    bbox_mask[by0:by1, bx0:bx1] = True
    return expanded, bbox_mask


def _fit_support_plane(points, threshold_m, min_support_points):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    if len(points) < min_support_points:
        raise ValueError(
            'support context has too few points %d < %d'
            % (len(points), min_support_points)
        )
    rng = np.random.RandomState(0)
    best_inliers = None
    best_count = 0
    iterations = max(96, min(512, len(points)))
    for _ in range(iterations):
        chosen = rng.choice(len(points), 3, replace=False)
        first, second, third = points[chosen]
        normal = np.cross(second - first, third - first)
        norm = float(np.linalg.norm(normal))
        if norm <= 1e-10 or not np.isfinite(norm):
            continue
        normal /= norm
        offset = -float(np.dot(normal, first))
        distances = np.abs(points @ normal + offset)
        inliers = distances <= threshold_m
        count = int(np.count_nonzero(inliers))
        if count > best_count:
            best_count = count
            best_inliers = inliers

    required = max(
        int(min_support_points),
        int(np.ceil(_MIN_SUPPORT_INLIER_RATIO * len(points))),
    )
    if best_inliers is None or best_count < required:
        raise ValueError(
            'support plane has too few inliers %d < %d'
            % (best_count, required)
        )

    inliers = best_inliers
    for _ in range(2):
        plane_points = points[inliers]
        center = np.mean(plane_points, axis=0)
        _u, _s, vh = np.linalg.svd(plane_points - center, full_matrices=False)
        normal = np.asarray(vh[-1], dtype=float)
        norm = float(np.linalg.norm(normal))
        if norm <= 1e-12 or not np.isfinite(norm):
            raise ValueError('support plane normal is degenerate')
        normal /= norm
        offset = -float(np.dot(normal, center))
        if not np.all(np.isfinite(normal)) or not np.isfinite(offset):
            raise ValueError('support plane coefficients are non-finite')
        inliers = np.abs(points @ normal + offset) <= threshold_m

    count = int(np.count_nonzero(inliers))
    ratio = float(count) / float(len(points))
    if count < required or ratio < _MIN_SUPPORT_INLIER_RATIO:
        raise ValueError(
            'support plane inlier ratio %.3f is below %.3f'
            % (ratio, _MIN_SUPPORT_INLIER_RATIO)
        )
    return normal, offset, ratio


def _voxel_centroids(points, voxel_size_m):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    keys = np.floor(points / float(voxel_size_m)).astype(np.int64)
    _unique, inverse = np.unique(keys, axis=0, return_inverse=True)
    counts = np.bincount(inverse)
    centroids = np.zeros((len(counts), 3), dtype=float)
    for axis in range(3):
        np.add.at(centroids[:, axis], inverse, points[:, axis])
    centroids /= counts[:, None]
    return centroids


def _spatial_neighbor_mean_distances(
    points,
    neighbors,
    cell_size_m,
    max_local_radius=None,
):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    count = len(points)
    neighbor_count = min(max(1, int(neighbors)), max(1, count - 1))
    cell_size = float(cell_size_m)
    if not np.isfinite(cell_size) or cell_size <= 0.0:
        raise ValueError('neighbour bucket cell size must be finite and positive')
    if not np.all(np.isfinite(points)):
        raise ValueError('neighbour points contain non-finite values')
    if count <= 1:
        return np.zeros(count, dtype=float), {
            'bucket_count': count,
            'distance_evaluations': 0,
            'fallback_points': 0,
            'max_radius': 0,
        }

    keys = np.floor(points / cell_size).astype(np.int64)
    if max_local_radius is None:
        radius_limit = max(4, int(np.ceil(np.cbrt(neighbor_count + 1))) * 2)
    else:
        radius_limit = max(1, int(max_local_radius))
    minimum_key = np.min(keys, axis=0)
    maximum_key = np.max(keys, axis=0)
    shape = maximum_key - minimum_key + 1
    max_integer = np.iinfo(np.int64).max
    if (
        np.any(shape <= 0)
        or int(shape[0]) > max_integer // max(1, int(shape[1]))
        or int(shape[0] * shape[1]) > max_integer // max(1, int(shape[2]))
    ):
        linear = None
    else:
        normalized_keys = keys - minimum_key
        linear = (
            (normalized_keys[:, 0] * shape[1] + normalized_keys[:, 1])
            * shape[2]
            + normalized_keys[:, 2]
        )
        if len(np.unique(linear)) != count:
            linear = None

    nearest_squared = np.full((count, neighbor_count), np.inf, dtype=float)
    distance_evaluations = 0
    fallback_points = 0
    maximum_radius = 0
    active = np.arange(count, dtype=np.int64)
    if linear is not None:
        sorted_order = np.argsort(linear)
        sorted_linear = linear[sorted_order]
        maximum_batch_entries = 2_000_000
        for radius in range(1, radius_limit + 1):
            if active.size == 0:
                break
            maximum_radius = radius
            offsets = np.asarray(
                [
                    (dx, dy, dz)
                    for dx in range(-radius, radius + 1)
                    for dy in range(-radius, radius + 1)
                    for dz in range(-radius, radius + 1)
                    if max(abs(dx), abs(dy), abs(dz)) == radius
                ],
                dtype=np.int64,
            )
            chunk_size = max(
                1,
                maximum_batch_entries // max(1, len(offsets)),
            )
            for start in range(0, len(active), chunk_size):
                chunk = active[start:start + chunk_size]
                chunk_distances = np.full(
                    (len(chunk), len(offsets)),
                    np.inf,
                    dtype=float,
                )
                for offset_index, offset in enumerate(offsets):
                    query_keys = keys[chunk] + offset
                    in_bounds = np.all(
                        (query_keys >= minimum_key)
                        & (query_keys <= maximum_key),
                        axis=1,
                    )
                    if not np.any(in_bounds):
                        continue
                    bounded_rows = np.flatnonzero(in_bounds)
                    normalized = query_keys[bounded_rows] - minimum_key
                    query_linear = (
                        (normalized[:, 0] * shape[1] + normalized[:, 1])
                        * shape[2]
                        + normalized[:, 2]
                    )
                    positions = np.searchsorted(sorted_linear, query_linear)
                    valid_positions = positions < len(sorted_linear)
                    clipped = np.minimum(positions, len(sorted_linear) - 1)
                    matched = valid_positions & (
                        sorted_linear[clipped] == query_linear
                    )
                    if not np.any(matched):
                        continue
                    rows = bounded_rows[matched]
                    neighbor_indices = sorted_order[positions[matched]]
                    delta = points[neighbor_indices] - points[chunk[rows]]
                    chunk_distances[rows, offset_index] = np.einsum(
                        'ij,ij->i',
                        delta,
                        delta,
                    )
                    distance_evaluations += int(np.count_nonzero(matched))
                combined = np.concatenate(
                    [nearest_squared[chunk], chunk_distances],
                    axis=1,
                )
                nearest_squared[chunk] = np.partition(
                    combined,
                    neighbor_count - 1,
                    axis=1,
                )[:, :neighbor_count]

            lower_boundary = (keys[active] - radius) * cell_size
            upper_boundary = (keys[active] + radius + 1) * cell_size
            outside_lower_bound = np.min(
                np.concatenate(
                    [
                        points[active] - lower_boundary,
                        upper_boundary - points[active],
                    ],
                    axis=1,
                ),
                axis=1,
            )
            kth_distance = np.sqrt(np.max(nearest_squared[active], axis=1))
            resolved = np.isfinite(kth_distance) & (
                kth_distance <= outside_lower_bound + 1e-12
            )
            active = active[~resolved]

    fallback_points = int(len(active))
    for index in active:
        delta = points - points[index]
        squared = np.einsum('ij,ij->i', delta, delta)
        squared[index] = np.inf
        distance_evaluations += count - 1
        nearest_squared[index] = np.partition(
            squared,
            neighbor_count - 1,
        )[:neighbor_count]

    mean_distances = np.mean(np.sqrt(nearest_squared), axis=1)
    return mean_distances, {
        'bucket_count': int(count if linear is not None else len(np.unique(keys, axis=0))),
        'distance_evaluations': int(distance_evaluations),
        'fallback_points': int(fallback_points),
        'max_radius': int(maximum_radius),
    }


def _neighbor_distance_filter(
    points,
    neighbors,
    std_ratio,
    cell_size_m=None,
    return_stats=False,
):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    count = len(points)
    neighbor_count = min(max(1, int(neighbors)), max(1, count - 1))
    if count <= neighbor_count + 1:
        stats = {
            'bucket_count': count,
            'distance_evaluations': 0,
            'fallback_points': 0,
            'max_radius': 0,
        }
        return (points, stats) if return_stats else points
    if cell_size_m is None:
        spans = np.ptp(points, axis=0)
        positive_spans = spans[spans > 0.0]
        scale = float(np.min(positive_spans)) if positive_spans.size else 1.0
        cell_size_m = max(scale / max(1.0, np.cbrt(count)), 1e-6)
    mean_distances, stats = _spatial_neighbor_mean_distances(
        points,
        neighbor_count,
        cell_size_m,
    )
    mean = float(np.mean(mean_distances))
    deviation = float(np.std(mean_distances))
    cutoff = mean + float(std_ratio) * deviation
    if not np.isfinite(cutoff):
        raise ValueError('outlier neighbour distance threshold is non-finite')
    filtered = points[mean_distances <= cutoff]
    return (filtered, stats) if return_stats else filtered


def _support_basis(normal):
    normal = np.asarray(normal, dtype=float).reshape(3)
    candidates = np.eye(3)
    seed = candidates[int(np.argmin(np.abs(candidates @ normal)))]
    first = seed - np.dot(seed, normal) * normal
    first /= np.linalg.norm(first)
    second = np.cross(normal, first)
    second /= np.linalg.norm(second)
    return np.column_stack([first, second])


def _fit_obb(
    points,
    normal,
    offset,
    min_size_m,
    max_size_m,
    max_height_m,
    previous_axes_base,
):
    signed_height = points @ normal + offset
    basis = _support_basis(normal)
    projected = points - signed_height[:, None] * normal[None, :]
    coordinates = projected @ basis
    low_2d = np.percentile(coordinates, 2.0, axis=0)
    high_2d = np.percentile(coordinates, 98.0, axis=0)
    retained = np.all((coordinates >= low_2d) & (coordinates <= high_2d), axis=1)
    robust_coordinates = coordinates[retained]
    if len(robust_coordinates) < 3:
        raise ValueError('OBB projected object points are degenerate')

    centered = robust_coordinates - np.mean(robust_coordinates, axis=0)
    covariance = centered.T @ centered / float(max(1, len(centered) - 1))
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    if not np.all(np.isfinite(eigenvalues)) or float(eigenvalues[-1]) <= 1e-12:
        raise ValueError('OBB projected PCA is degenerate')
    pca_axis = np.asarray(eigenvectors[:, -1], dtype=float)
    pca_axis /= np.linalg.norm(pca_axis)

    rectangle = cv2.minAreaRect(robust_coordinates.astype(np.float32))
    center_2d = np.asarray(rectangle[0], dtype=float)
    corners = cv2.boxPoints(rectangle).astype(float)
    first_edge = corners[1] - corners[0]
    second_edge = corners[2] - corners[1]
    first_length = float(np.linalg.norm(first_edge))
    second_length = float(np.linalg.norm(second_edge))
    if first_length <= 1e-12 or second_length <= 1e-12:
        raise ValueError('OBB minimum-area rectangle is degenerate')
    first_axis = first_edge / first_length
    second_axis = second_edge / second_length
    edge_options = (
        (first_axis, first_length, second_length),
        (second_axis, second_length, first_length),
    )
    long_axis_2d, long_size, short_size = max(
        edge_options,
        key=lambda item: abs(float(np.dot(item[0], pca_axis))),
    )
    if long_size < short_size:
        long_axis_2d = np.asarray([-long_axis_2d[1], long_axis_2d[0]], dtype=float)
        long_size, short_size = short_size, long_size
    if float(np.dot(long_axis_2d, pca_axis)) < 0.0:
        long_axis_2d = -long_axis_2d

    long_axis = basis @ long_axis_2d
    long_axis /= np.linalg.norm(long_axis)
    short_axis = np.cross(normal, long_axis)
    short_axis /= np.linalg.norm(short_axis)
    axes = np.column_stack([long_axis, short_axis, normal])
    if np.linalg.det(axes) < 0.0:
        short_axis = -short_axis
        axes = np.column_stack([long_axis, short_axis, normal])

    if previous_axes_base is not None:
        previous = np.asarray(previous_axes_base, dtype=float)
        if previous.shape != (3, 3) or not np.all(np.isfinite(previous)):
            raise ValueError('previous axes must be a finite (3, 3) matrix')
        if (
            not np.allclose(previous.T @ previous, np.eye(3), atol=1e-5)
            or abs(float(np.linalg.det(previous)) - 1.0) > 1e-5
        ):
            raise ValueError('previous axes must be a right-handed orthonormal frame')
        unflipped_trace = float(np.trace(previous.T @ axes))
        flipped = axes.copy()
        flipped[:, :2] *= -1.0
        flipped_trace = float(np.trace(previous.T @ flipped))
        if flipped_trace > unflipped_trace:
            axes = flipped

    bottom, top = np.percentile(signed_height, [2.0, 98.0])
    height = float(top - bottom)
    plane_center = basis @ center_2d - float(offset) * normal
    center = plane_center + normal * (0.5 * (float(bottom) + float(top)))
    size = np.asarray([long_size, short_size, height], dtype=float)
    if not np.all(np.isfinite(center)) or not np.all(np.isfinite(axes)) or not np.all(np.isfinite(size)):
        raise ValueError('OBB outputs contain non-finite values')
    if float(np.linalg.det(axes)) < 0.999:
        raise ValueError('OBB axes are not a right-handed orthonormal frame')
    if np.any(size < float(min_size_m)):
        raise ValueError(
            'OBB size %s is below minimum %.6f m'
            % (np.array2string(size, precision=6), float(min_size_m))
        )
    if np.any(size > float(max_size_m)) or height > float(max_height_m):
        raise ValueError(
            'OBB size %s exceeds configured limits'
            % np.array2string(size, precision=6)
        )
    return center, axes, size


def estimate_object_geometry(
    depth_raw,
    target_depth_raw,
    object_mask,
    bbox,
    intrinsics,
    depth_scale,
    T_base_camera,
    source_mode,
    support_bbox_expand_ratio,
    support_distance_threshold_m,
    voxel_size_m,
    min_support_points,
    min_object_points,
    min_size_m,
    max_size_m,
    max_height_m,
    previous_axes_base,
    outlier_neighbors=16,
    outlier_std_ratio=2.0,
):
    """Return GeometryEstimate with one stable base-frame OBB."""
    source = str(source_mode or '')
    if source not in _VALID_SOURCE_MODES:
        return _failure('OBB_INVALID', 'source_mode is invalid: %s' % source, source)
    try:
        depth = np.asarray(depth_raw)
        target_depth = np.asarray(target_depth_raw)
        mask = np.asarray(object_mask)
        if depth.ndim != 2:
            raise ValueError('depth shape must be two-dimensional')
        if target_depth.shape != depth.shape or mask.shape != depth.shape:
            raise ValueError('depth, target depth, and mask shapes must match')
        _intrinsic_values(intrinsics, depth.shape)
        scale = float(depth_scale)
        if not np.isfinite(scale) or scale <= 0.0:
            raise ValueError('depth_scale must be finite and positive')
        transform = _validated_transform(T_base_camera)
        threshold = float(support_distance_threshold_m)
        voxel_size = float(voxel_size_m)
        support_minimum = int(min_support_points)
        object_minimum = int(min_object_points)
        minimum_size = float(min_size_m)
        maximum_size = float(max_size_m)
        maximum_height = float(max_height_m)
        neighbor_count = int(outlier_neighbors)
        neighbor_std_ratio = float(outlier_std_ratio)
        numeric = np.asarray(
            [
                threshold,
                voxel_size,
                minimum_size,
                maximum_size,
                maximum_height,
                neighbor_std_ratio,
            ],
            dtype=float,
        )
        if not np.all(np.isfinite(numeric)):
            raise ValueError('geometry threshold or dimension configuration is non-finite')
        if threshold <= 0.0:
            raise ValueError('support distance threshold must be positive')
        if voxel_size <= 0.0 or support_minimum < 3 or object_minimum < 3:
            raise ValueError('geometry point and voxel thresholds must be positive')
        if minimum_size <= 0.0 or maximum_size < minimum_size or maximum_height <= 0.0:
            raise ValueError('geometry dimension limits are invalid')
        if neighbor_count < 1 or neighbor_std_ratio < 0.0:
            raise ValueError('outlier neighbour configuration is invalid')
        expanded, bbox_mask = _expanded_bbox_mask(
            depth.shape,
            bbox,
            support_bbox_expand_ratio,
        )
        foreground = mask > 0
        if source == 'instance_mask':
            if not np.any(foreground):
                raise ValueError('instance mask is empty')
        else:
            foreground = bbox_mask
        support_mask = expanded & ~foreground
        support_camera, _support_uv = deproject_depth(
            depth,
            support_mask,
            intrinsics,
            scale,
        )
        target_camera, _target_uv = deproject_depth(
            target_depth,
            target_depth > 0,
            intrinsics,
            scale,
        )
    except Exception as exc:
        reason = str(exc)
        code = (
            'SUPPORT_PLANE_INVALID'
            if 'support' in reason or 'threshold' in reason
            else 'OBB_INVALID'
        )
        return _failure(code, reason, source)

    support_base = _transform_points(support_camera, transform)
    target_base = _transform_points(target_camera, transform)
    try:
        normal, offset, inlier_ratio = _fit_support_plane(
            support_base,
            threshold,
            support_minimum,
        )
        if len(target_base) == 0:
            raise ValueError('target point cloud is empty')
        target_median = np.median(target_base, axis=0)
        if float(np.dot(target_median, normal) + offset) < 0.0:
            normal = -normal
            offset = -offset
        if not np.all(np.isfinite(normal)) or not np.isfinite(offset):
            raise ValueError('support plane coefficients are non-finite')
    except Exception as exc:
        return _failure('SUPPORT_PLANE_INVALID', str(exc), source)

    try:
        signed = target_base @ normal + offset
        object_points = target_base[signed > threshold]
        if len(object_points) < object_minimum:
            raise ValueError(
                'OBB target has too few points above support %d < %d'
                % (len(object_points), object_minimum)
            )
        object_points = _voxel_centroids(object_points, voxel_size)
        object_points = _neighbor_distance_filter(
            object_points,
            neighbor_count,
            neighbor_std_ratio,
            cell_size_m=voxel_size,
        )
        if len(object_points) < object_minimum:
            raise ValueError(
                'OBB cleaned target has too few points %d < %d'
                % (len(object_points), object_minimum)
            )
        center, axes, size = _fit_obb(
            object_points,
            normal,
            offset,
            minimum_size,
            maximum_size,
            maximum_height,
            previous_axes_base,
        )
        output_values = np.concatenate(
            [
                center,
                axes.reshape(-1),
                size,
                normal,
                np.asarray([offset, inlier_ratio], dtype=float),
                object_points.reshape(-1),
            ]
        )
        if not np.all(np.isfinite(output_values)):
            raise ValueError('OBB output contains non-finite values')
    except Exception as exc:
        return _failure('OBB_INVALID', str(exc), source)

    return GeometryEstimate(
        ok=True,
        failure_code='',
        failure_reason='',
        center_base=center,
        axes_base=axes,
        size_xyz_m=size,
        support_normal_base=normal,
        support_offset_m=float(offset),
        support_inlier_ratio=float(inlier_ratio),
        object_points_base=object_points,
        source_mode=source,
    )
