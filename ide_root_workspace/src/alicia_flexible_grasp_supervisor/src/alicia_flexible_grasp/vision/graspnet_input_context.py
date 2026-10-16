"""Fail-closed construction of depth context sent to a pure GraspNet backend.

This module deliberately has no ROS dependency.  It only combines arrays from
one already-locked RGB-D planning snapshot; it does not generate or modify
grasp candidates.
"""

from dataclasses import dataclass
import math

import numpy as np


MASKED_TARGET = 'masked_target'
CONTEXT_ROI = 'context_roi'
FULL_SCENE = 'full_scene'
VALID_MODES = frozenset((MASKED_TARGET, CONTEXT_ROI, FULL_SCENE))

DEFAULT_MIN_TARGET_POINTS = 120
DEFAULT_MIN_SUPPORT_POINTS = 200
DEFAULT_MIN_TOTAL_POINTS = 320
DEFAULT_MIN_TARGET_FRACTION = 0.15
DEFAULT_CONTEXT_PLANE_DISTANCE_M = 0.006
DEFAULT_TARGET_GUARD_PX = 2
DEFAULT_CONTEXT_MARGIN_PX = 24.0
DEFAULT_CONTEXT_EXPAND_RATIO = 0.30
DEFAULT_CONTEXT_MAX_MARGIN_PX = 64.0
DEFAULT_DETECTED_BBOX_MIN_IOU = 0.50


class GraspNetInputContextError(RuntimeError):
    """Structured failure raised instead of changing input modes implicitly."""

    def __init__(self, code, reason, audit=None):
        self.code = str(code)
        self.reason = str(reason)
        self.audit = dict(audit or {})
        super().__init__('%s: %s' % (self.code, self.reason))


@dataclass(frozen=True)
class GraspNetInputAudit:
    mode: str
    diagnostic_only: bool
    bbox_xyxy: tuple
    detected_bbox_xywh: tuple
    detected_bbox_mask_iou: object
    roi_xyxy: tuple
    padding_px: int
    context_margin_px: float
    context_expand_ratio: float
    context_max_margin_px: float
    guard_px: int
    plane_distance_threshold_m: float
    target_points: int
    target_hole_filled_points: int
    support_points: int
    total_points: int
    target_fraction: float
    valid_full_scene_points: int
    excluded_guard_points: int
    excluded_off_plane_points: int
    min_target_points: int
    min_support_points: int
    min_total_points: int
    min_target_fraction: float


@dataclass(frozen=True)
class GraspNetInputContext:
    mode: str
    depth_raw: np.ndarray
    color_bgr: np.ndarray
    audit: GraspNetInputAudit
    diagnostic_only: bool

    def __post_init__(self):
        object.__setattr__(self, 'depth_raw', _readonly_copy(self.depth_raw))
        object.__setattr__(
            self,
            'color_bgr',
            _readonly_copy(self.color_bgr, dtype=np.uint8),
        )


def _readonly_copy(value, dtype=None):
    output = np.asarray(value, dtype=dtype).copy()
    output.setflags(write=False)
    return output


def _fail(code, reason, **audit):
    raise GraspNetInputContextError(code, reason, audit)


def _finite_float(value, name, positive=False, nonnegative=False):
    if isinstance(value, (bool, np.bool_)):
        _fail('CONFIG_INVALID', '%s must be numeric, not bool' % name)
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        _fail('CONFIG_INVALID', '%s must be numeric' % name)
    if not np.isfinite(result):
        _fail('CONFIG_INVALID', '%s must be finite' % name)
    if positive and result <= 0.0:
        _fail('CONFIG_INVALID', '%s must be positive' % name)
    if nonnegative and result < 0.0:
        _fail('CONFIG_INVALID', '%s must be non-negative' % name)
    return result


def _count_threshold(value, name, minimum=0):
    if isinstance(value, (bool, np.bool_)):
        _fail('CONFIG_INVALID', '%s must be an integer' % name)
    try:
        result = int(value)
    except (TypeError, ValueError, OverflowError):
        _fail('CONFIG_INVALID', '%s must be an integer' % name)
    try:
        exact = float(value)
    except (TypeError, ValueError, OverflowError):
        exact = float(result)
    if not np.isfinite(exact) or exact != float(result) or result < minimum:
        qualifier = 'positive' if minimum == 1 else 'non-negative'
        _fail('CONFIG_INVALID', '%s must be a %s integer' % (name, qualifier))
    return result


def _unit_interval_threshold(value, name):
    result = _finite_float(value, name, positive=True)
    if result > 1.0:
        _fail('CONFIG_INVALID', '%s must be <= 1.0' % name)
    return result


def _validate_inputs(target_depth_raw, object_mask, full_depth_raw, color_bgr, mode):
    target = np.asarray(target_depth_raw)
    mask = np.asarray(object_mask)
    color = np.asarray(color_bgr)

    if target.ndim != 2 or target.size == 0:
        _fail('SHAPE_INVALID', 'target_depth_raw must be a non-empty 2-D array')
    if target.dtype.kind != 'u':
        _fail('DTYPE_INVALID', 'target_depth_raw must use an unsigned integer dtype')
    if mask.shape != target.shape:
        _fail(
            'SHAPE_INVALID',
            'object_mask shape %s does not match target depth shape %s'
            % (mask.shape, target.shape),
        )
    if mask.dtype != np.dtype(bool) and mask.dtype != np.dtype(np.uint8):
        _fail('DTYPE_INVALID', 'object_mask must have bool or uint8 dtype')
    if color.shape != target.shape + (3,):
        _fail(
            'SHAPE_INVALID',
            'color_bgr shape %s does not match depth shape %s'
            % (color.shape, target.shape),
        )
    if color.dtype != np.dtype(np.uint8):
        _fail('DTYPE_INVALID', 'color_bgr must have uint8 dtype')

    full = None
    if mode in (CONTEXT_ROI, FULL_SCENE):
        if full_depth_raw is None:
            _fail('SHAPE_INVALID', '%s requires full_depth_raw' % mode)
        full = np.asarray(full_depth_raw)
        if full.shape != target.shape:
            _fail(
                'SHAPE_INVALID',
                'full depth shape %s does not match target depth shape %s'
                % (full.shape, target.shape),
            )
        if full.dtype != target.dtype:
            _fail(
                'DTYPE_INVALID',
                'full and target depth dtypes must match exactly (%s != %s)'
                % (full.dtype, target.dtype),
            )

    binary_mask = mask > 0
    if not np.any(binary_mask):
        _fail('MASK_EMPTY', 'object_mask contains no target pixels')
    target_valid = target > 0
    if np.any(target_valid & ~binary_mask):
        _fail(
            'TARGET_DEPTH_INVALID',
            'target_depth_raw contains non-zero pixels outside object_mask',
        )
    target_hole_filled_points = 0
    if full is not None:
        target_hole_filled = target_valid & (full == 0)
        target_hole_filled_points = int(np.count_nonzero(target_hole_filled))
        inconsistent = target_valid & (full > 0) & (full != target)
        if np.any(inconsistent):
            _fail(
                'SNAPSHOT_INCONSISTENT',
                'full depth disagrees with target depth at %d target pixels'
                % int(np.count_nonzero(inconsistent)),
            )
    return target, binary_mask, full, color, target_hole_filled_points


def _intrinsic_values(intrinsics, image_shape, depth_scale):
    try:
        width = int(intrinsics.width)
        height = int(intrinsics.height)
        fx = float(intrinsics.fx)
        fy = float(intrinsics.fy)
        cx = float(intrinsics.cx)
        cy = float(intrinsics.cy)
    except Exception as exc:
        raise GraspNetInputContextError(
            'INTRINSICS_INVALID',
            'intrinsics width/height/fx/fy/cx/cy are unavailable',
        ) from exc
    values = np.asarray([width, height, fx, fy, cx, cy], dtype=float)
    if not np.all(np.isfinite(values)):
        _fail('INTRINSICS_INVALID', 'intrinsics contain non-finite values')
    if width <= 0 or height <= 0 or fx <= 0.0 or fy <= 0.0:
        _fail(
            'INTRINSICS_INVALID',
            'intrinsics dimensions and focal lengths must be positive',
        )
    if (height, width) != tuple(image_shape):
        _fail(
            'INTRINSICS_INVALID',
            'intrinsics image shape %s does not match depth shape %s'
            % ((height, width), tuple(image_shape)),
        )
    if depth_scale is None:
        try:
            depth_scale = intrinsics.depth_scale
        except Exception as exc:
            raise GraspNetInputContextError(
                'INTRINSICS_INVALID',
                'intrinsics depth_scale is unavailable',
            ) from exc
    scale = _finite_float(depth_scale, 'depth_scale', positive=True)
    return fx, fy, cx, cy, scale


def _valid_raw_depth(depth, scale, depth_min_m, depth_max_m, name):
    minimum = _finite_float(depth_min_m, 'depth_min_m', positive=True)
    maximum = _finite_float(depth_max_m, 'depth_max_m', positive=True)
    if maximum < minimum:
        _fail('CONFIG_INVALID', 'depth_max_m must be >= depth_min_m')
    depth_m = depth.astype(np.float64) * scale
    nonzero = depth > 0
    invalid = nonzero & ((depth_m < minimum) | (depth_m > maximum))
    if np.any(invalid):
        values = depth_m[invalid]
        _fail(
            'DEPTH_RANGE_INVALID',
            '%s contains %d non-zero values outside [%.6f, %.6f] m '
            '(observed %.6f..%.6f m)'
            % (
                name,
                int(values.size),
                minimum,
                maximum,
                float(np.min(values)),
                float(np.max(values)),
            ),
        )
    return nonzero


def _mask_bbox(mask):
    ys, xs = np.nonzero(mask)
    if xs.size == 0:
        _fail('MASK_EMPTY', 'object_mask contains no target pixels')
    return (
        int(np.min(xs)),
        int(np.min(ys)),
        int(np.max(xs)) + 1,
        int(np.max(ys)) + 1,
    )


def _context_roi(
    shape,
    bbox,
    context_margin_px,
    context_expand_ratio,
    context_max_margin_px,
):
    x0, y0, x1, y1 = bbox
    max_dimension = max(x1 - x0, y1 - y0)
    padding = int(
        math.ceil(
            min(
                context_max_margin_px,
                max(context_margin_px, context_expand_ratio * max_dimension),
            )
        )
    )
    height, width = shape
    return (
        max(0, x0 - padding),
        max(0, y0 - padding),
        min(width, x1 + padding),
        min(height, y1 + padding),
    ), padding


def _validated_detected_bbox(detected_bbox_xywh, mask_bbox, min_iou):
    if detected_bbox_xywh is None:
        return None, None
    try:
        raw_values = list(detected_bbox_xywh)
    except TypeError as exc:
        raise GraspNetInputContextError(
            'BBOX_INVALID',
            'detected_bbox_xywh must contain four values',
        ) from exc
    if len(raw_values) != 4:
        _fail('BBOX_INVALID', 'detected_bbox_xywh must contain four values')
    values = []
    for index, value in enumerate(raw_values):
        if isinstance(value, (bool, np.bool_)):
            _fail(
                'BBOX_INVALID',
                'detected_bbox_xywh[%d] must be an integer, not bool' % index,
            )
        try:
            numeric = float(value)
            integer = int(value)
        except (TypeError, ValueError, OverflowError):
            _fail(
                'BBOX_INVALID',
                'detected_bbox_xywh[%d] must be a finite non-negative integer'
                % index,
            )
        if (
            not np.isfinite(numeric)
            or numeric != float(integer)
            or integer < 0
        ):
            _fail(
                'BBOX_INVALID',
                'detected_bbox_xywh[%d] must be a finite non-negative integer'
                % index,
            )
        values.append(integer)
    detected = tuple(values)
    x, y, width, height = detected
    detected_xyxy = (x, y, x + width, y + height)
    mx0, my0, mx1, my1 = mask_bbox
    dx0, dy0, dx1, dy1 = detected_xyxy
    intersection_width = max(0, min(mx1, dx1) - max(mx0, dx0))
    intersection_height = max(0, min(my1, dy1) - max(my0, dy0))
    intersection = intersection_width * intersection_height
    mask_area = max(0, mx1 - mx0) * max(0, my1 - my0)
    detected_area = max(0, dx1 - dx0) * max(0, dy1 - dy0)
    union = mask_area + detected_area - intersection
    iou = float(intersection) / float(union) if union > 0 else 0.0
    if iou < min_iou:
        _fail(
            'BBOX_MISMATCH',
            'detected bbox/mask bbox IoU %.6f < %.6f' % (iou, min_iou),
            detected_bbox_xywh=detected,
            mask_bbox_xyxy=tuple(mask_bbox),
            detected_bbox_mask_iou=iou,
            detected_bbox_min_iou=min_iou,
        )
    return detected, iou


def _dilate_square(mask, radius):
    radius = _count_threshold(radius, 'target_guard_px')
    source = np.asarray(mask, dtype=bool)
    if radius == 0:
        return source.copy()
    height, width = source.shape
    padded = np.pad(source, radius, mode='constant', constant_values=False)
    output = np.zeros_like(source, dtype=bool)
    diameter = radius * 2 + 1
    for y_offset in range(diameter):
        for x_offset in range(diameter):
            output |= padded[
                y_offset:y_offset + height,
                x_offset:x_offset + width,
            ]
    return output


def _validated_plane(point_camera, normal_camera):
    try:
        point = np.asarray(point_camera, dtype=float)
        normal = np.asarray(normal_camera, dtype=float)
    except (TypeError, ValueError, OverflowError) as exc:
        raise GraspNetInputContextError(
            'SUPPORT_PLANE_INVALID',
            'support plane point and normal must be numeric vectors',
        ) from exc
    if point.shape != (3,) or normal.shape != (3,):
        _fail(
            'SUPPORT_PLANE_INVALID',
            'support plane point and normal must each contain three values',
        )
    if not np.all(np.isfinite(point)) or not np.all(np.isfinite(normal)):
        _fail('SUPPORT_PLANE_INVALID', 'support plane contains non-finite values')
    norm = float(np.linalg.norm(normal))
    if norm <= 1e-12:
        _fail('SUPPORT_PLANE_INVALID', 'support plane normal is degenerate')
    return point, normal / norm


def _support_mask_camera_link(
    full_depth,
    valid_full,
    roi,
    guard_mask,
    fx,
    fy,
    cx,
    cy,
    scale,
    plane_point,
    plane_normal,
    max_plane_distance_m,
):
    x0, y0, x1, y1 = roi
    candidate = np.zeros(full_depth.shape, dtype=bool)
    candidate[y0:y1, x0:x1] = True
    candidate &= valid_full & ~guard_mask
    v, u = np.nonzero(candidate)
    if u.size == 0:
        return candidate, 0, 0

    z = full_depth[v, u].astype(np.float64) * scale
    optical_x = (u.astype(np.float64) - cx) * z / fx
    optical_y = (v.astype(np.float64) - cy) * z / fy
    # OpenCV optical (right, down, forward) -> ROS camera_link
    # (forward, left, up), matching OPTICAL_TO_ROS_CAMERA in the live node.
    points_camera_link = np.column_stack((z, -optical_x, -optical_y))
    distance = np.abs((points_camera_link - plane_point) @ plane_normal)
    accepted = distance <= max_plane_distance_m
    support = np.zeros(full_depth.shape, dtype=bool)
    support[v[accepted], u[accepted]] = True
    return support, int(np.count_nonzero(candidate)), int(np.count_nonzero(~accepted))


def _audit_and_validate(
    mode,
    diagnostic_only,
    bbox,
    detected_bbox,
    detected_bbox_mask_iou,
    roi,
    padding,
    context_margin_px,
    context_expand_ratio,
    context_max_margin_px,
    guard_px,
    plane_distance_m,
    target_points,
    target_hole_filled_points,
    support_points,
    total_points,
    full_scene_points,
    excluded_guard_points,
    excluded_off_plane_points,
    min_target_points,
    min_support_points,
    min_total_points,
    min_target_fraction,
):
    fraction = (
        float(target_points) / float(total_points)
        if total_points > 0
        else 0.0
    )
    audit_values = {
        'mode': mode,
        'diagnostic_only': bool(diagnostic_only),
        'bbox_xyxy': tuple(bbox),
        'detected_bbox_xywh': (
            None if detected_bbox is None else tuple(detected_bbox)
        ),
        'detected_bbox_mask_iou': (
            None
            if detected_bbox_mask_iou is None
            else float(detected_bbox_mask_iou)
        ),
        'roi_xyxy': tuple(roi),
        'padding_px': int(padding),
        'context_margin_px': float(context_margin_px),
        'context_expand_ratio': float(context_expand_ratio),
        'context_max_margin_px': float(context_max_margin_px),
        'guard_px': int(guard_px),
        'plane_distance_threshold_m': float(plane_distance_m),
        'target_points': int(target_points),
        'target_hole_filled_points': int(target_hole_filled_points),
        'support_points': int(support_points),
        'total_points': int(total_points),
        'target_fraction': float(fraction),
        'valid_full_scene_points': int(full_scene_points),
        'excluded_guard_points': int(excluded_guard_points),
        'excluded_off_plane_points': int(excluded_off_plane_points),
        'min_target_points': int(min_target_points),
        'min_support_points': int(min_support_points),
        'min_total_points': int(min_total_points),
        'min_target_fraction': float(min_target_fraction),
    }
    if target_points < min_target_points:
        _fail(
            'TARGET_POINTS_INSUFFICIENT',
            'target points %d < %d' % (target_points, min_target_points),
            **audit_values
        )
    if mode == CONTEXT_ROI and support_points < min_support_points:
        _fail(
            'SUPPORT_POINTS_INSUFFICIENT',
            'support points %d < %d' % (support_points, min_support_points),
            **audit_values
        )
    if total_points < min_total_points:
        _fail(
            'TOTAL_POINTS_INSUFFICIENT',
            'total points %d < %d' % (total_points, min_total_points),
            **audit_values
        )
    # Target fraction is an execution-quality gate for context_roi.  A masked
    # target is target-only by construction, while full_scene is permanently
    # diagnostic and naturally has a much smaller target fraction.
    if mode == CONTEXT_ROI and fraction < min_target_fraction:
        _fail(
            'TARGET_FRACTION_INSUFFICIENT',
            'target fraction %.6f < %.6f' % (fraction, min_target_fraction),
            **audit_values
        )
    return GraspNetInputAudit(**audit_values)


def build_graspnet_input_context(
    mode,
    target_depth_raw,
    object_mask,
    full_depth_raw,
    color_bgr,
    intrinsics,
    support_plane_point_camera=None,
    support_plane_normal_camera=None,
    depth_scale=None,
    depth_min_m=0.03,
    depth_max_m=2.0,
    context_plane_distance_m=DEFAULT_CONTEXT_PLANE_DISTANCE_M,
    context_margin_px=DEFAULT_CONTEXT_MARGIN_PX,
    context_expand_ratio=DEFAULT_CONTEXT_EXPAND_RATIO,
    context_max_margin_px=DEFAULT_CONTEXT_MAX_MARGIN_PX,
    target_guard_px=DEFAULT_TARGET_GUARD_PX,
    detected_bbox_xywh=None,
    detected_bbox_min_iou=DEFAULT_DETECTED_BBOX_MIN_IOU,
    min_target_points=DEFAULT_MIN_TARGET_POINTS,
    min_support_points=DEFAULT_MIN_SUPPORT_POINTS,
    min_total_points=DEFAULT_MIN_TOTAL_POINTS,
    min_target_fraction=DEFAULT_MIN_TARGET_FRACTION,
):
    """Build immutable RGB-D input for one explicitly selected GraspNet mode.

    ``support_plane_*_camera`` is expressed in ROS ``camera_link`` coordinates,
    while image intrinsics follow OpenCV optical coordinates.  ``full_scene``
    is always marked diagnostic-only; callers must not use it as an implicit
    execution fallback.
    """
    normalized_mode = str(mode or '').strip().lower()
    if normalized_mode not in VALID_MODES:
        _fail(
            'MODE_INVALID',
            'mode must be one of %s, got %r'
            % (', '.join(sorted(VALID_MODES)), mode),
        )

    # These are fail-closed execution-quality gates, not optional feature
    # toggles.  Requiring a strictly positive threshold prevents a runtime
    # ROS parameter value of zero from silently disabling any gate.
    min_target_points = _count_threshold(
        min_target_points,
        'min_target_points',
        minimum=1,
    )
    min_support_points = _count_threshold(
        min_support_points,
        'min_support_points',
        minimum=1,
    )
    min_total_points = _count_threshold(
        min_total_points,
        'min_total_points',
        minimum=1,
    )
    min_target_fraction = _unit_interval_threshold(
        min_target_fraction,
        'min_target_fraction',
    )
    plane_distance = _finite_float(
        context_plane_distance_m,
        'context_plane_distance_m',
        positive=True,
    )
    context_margin = _finite_float(
        context_margin_px,
        'context_margin_px',
        nonnegative=True,
    )
    context_expand = _finite_float(
        context_expand_ratio,
        'context_expand_ratio',
        nonnegative=True,
    )
    context_max_margin = _finite_float(
        context_max_margin_px,
        'context_max_margin_px',
        nonnegative=True,
    )
    if context_margin > context_max_margin:
        _fail(
            'CONFIG_INVALID',
            'context_margin_px must be <= context_max_margin_px',
        )
    detected_bbox_iou_threshold = _unit_interval_threshold(
        detected_bbox_min_iou,
        'detected_bbox_min_iou',
    )
    guard_px = _count_threshold(target_guard_px, 'target_guard_px')

    target, mask, full, color, target_hole_filled_points = _validate_inputs(
        target_depth_raw,
        object_mask,
        full_depth_raw,
        color_bgr,
        normalized_mode,
    )
    fx, fy, cx, cy, scale = _intrinsic_values(
        intrinsics,
        target.shape,
        depth_scale,
    )
    valid_target = _valid_raw_depth(
        target,
        scale,
        depth_min_m,
        depth_max_m,
        'target_depth_raw',
    )
    valid_full = None
    if full is not None:
        valid_full = _valid_raw_depth(
            full,
            scale,
            depth_min_m,
            depth_max_m,
            'full_depth_raw',
        )

    bbox = _mask_bbox(mask)
    detected_bbox, detected_bbox_mask_iou = _validated_detected_bbox(
        detected_bbox_xywh,
        bbox,
        detected_bbox_iou_threshold,
    )
    full_roi = (0, 0, int(target.shape[1]), int(target.shape[0]))
    roi = full_roi
    padding = 0
    support = np.zeros(target.shape, dtype=bool)
    excluded_guard = 0
    excluded_off_plane = 0
    diagnostic_only = normalized_mode == FULL_SCENE

    if normalized_mode == MASKED_TARGET:
        output_depth = target.copy()
        output_target = valid_target
    elif normalized_mode == CONTEXT_ROI:
        plane_point, plane_normal = _validated_plane(
            support_plane_point_camera,
            support_plane_normal_camera,
        )
        roi, padding = _context_roi(
            target.shape,
            bbox,
            context_margin,
            context_expand,
            context_max_margin,
        )
        guard = _dilate_square(mask, guard_px)
        x0, y0, x1, y1 = roi
        roi_mask = np.zeros(target.shape, dtype=bool)
        roi_mask[y0:y1, x0:x1] = True
        excluded_guard = int(
            np.count_nonzero(valid_full & roi_mask & guard & ~valid_target)
        )
        support, _plane_candidates, excluded_off_plane = _support_mask_camera_link(
            full,
            valid_full,
            roi,
            guard,
            fx,
            fy,
            cx,
            cy,
            scale,
            plane_point,
            plane_normal,
            plane_distance,
        )
        output_depth = np.zeros_like(target)
        output_depth[support] = full[support]
        # Target is authoritative and is written last so the guard can never
        # consume it, even where the target lies close to the support plane.
        output_depth[valid_target] = target[valid_target]
        output_target = valid_target
    else:
        output_depth = full.copy()
        output_target = valid_target & valid_full

    output_valid = output_depth > 0
    target_points = int(np.count_nonzero(output_target & output_valid))
    support_points = int(np.count_nonzero(support))
    total_points = int(np.count_nonzero(output_valid))
    full_scene_points = (
        int(np.count_nonzero(valid_full))
        if valid_full is not None
        else total_points
    )
    audit = _audit_and_validate(
        normalized_mode,
        diagnostic_only,
        bbox,
        detected_bbox,
        detected_bbox_mask_iou,
        roi,
        padding,
        context_margin,
        context_expand,
        context_max_margin,
        guard_px if normalized_mode == CONTEXT_ROI else 0,
        plane_distance if normalized_mode == CONTEXT_ROI else 0.0,
        target_points,
        target_hole_filled_points,
        support_points,
        total_points,
        full_scene_points,
        excluded_guard,
        excluded_off_plane,
        min_target_points,
        min_support_points,
        min_total_points,
        min_target_fraction,
    )

    output_color = np.zeros_like(color)
    output_color[output_valid] = color[output_valid]
    return GraspNetInputContext(
        mode=normalized_mode,
        depth_raw=output_depth,
        color_bgr=output_color,
        audit=audit,
        diagnostic_only=diagnostic_only,
    )
