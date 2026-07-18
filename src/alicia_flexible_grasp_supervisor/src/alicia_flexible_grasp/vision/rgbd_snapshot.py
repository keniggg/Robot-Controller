from copy import deepcopy
from dataclasses import dataclass
import threading
import time

import cv2
import numpy as np


@dataclass
class RgbdSample:
    color_bgr: np.ndarray
    depth_raw: np.ndarray
    object_mask: np.ndarray
    bbox: tuple
    object_msg: object
    stamp_sec: float
    frame_id: str
    joint_positions: np.ndarray
    stamp_ns: int = 0


@dataclass(frozen=True)
class DepthQuality:
    fused_frames: int
    mask_area: int
    valid_depth_points: int
    valid_depth_ratio: float
    depth_median_m: float
    depth_mad_m: float


@dataclass(frozen=True)
class SnapshotResult:
    ok: bool
    failure_code: str
    failure_reason: str
    color_bgr: np.ndarray
    depth_raw: np.ndarray
    target_depth_raw: np.ndarray
    object_mask: np.ndarray
    bbox: tuple
    object_msg: object
    stamp_sec: float
    frame_id: str
    quality: DepthQuality
    source_mode: str
    stamp_ns: int = 0

    def __post_init__(self):
        for name in ('color_bgr', 'depth_raw', 'target_depth_raw', 'object_mask'):
            object.__setattr__(self, name, _readonly_copy(getattr(self, name)))
        object.__setattr__(self, 'bbox', tuple(self.bbox or ()))


def mask_iou(first, second):
    first_binary = np.asarray(first) > 0
    second_binary = np.asarray(second) > 0
    if first_binary.shape != second_binary.shape:
        return 0.0
    union = int(np.count_nonzero(first_binary | second_binary))
    if union == 0:
        return 0.0
    intersection = int(np.count_nonzero(first_binary & second_binary))
    return float(intersection) / float(union)


def _mask_centroid(mask):
    ys, xs = np.nonzero(np.asarray(mask) > 0)
    if len(xs) == 0:
        return None
    return np.asarray([float(np.mean(xs)), float(np.mean(ys))], dtype=float)


def _bbox_mask(shape, bbox):
    height, width = shape
    try:
        x, y, box_width, box_height = [int(value) for value in bbox]
    except Exception:
        return np.zeros(shape, dtype=np.uint8)
    x0 = min(width, max(0, x))
    y0 = min(height, max(0, y))
    x1 = min(width, max(x0, x + max(0, box_width)))
    y1 = min(height, max(y0, y + max(0, box_height)))
    output = np.zeros(shape, dtype=np.uint8)
    output[y0:y1, x0:x1] = 255
    return output


def _readonly_copy(array, dtype=None):
    output = np.asarray(array, dtype=dtype).copy()
    output.setflags(write=False)
    return output


def _empty_quality(fused_frames=0):
    return DepthQuality(
        fused_frames=int(fused_frames),
        mask_area=0,
        valid_depth_points=0,
        valid_depth_ratio=0.0,
        depth_median_m=0.0,
        depth_mad_m=0.0,
    )


def _failure(samples, code, reason, source_mode):
    latest = samples[-1] if samples else None
    if latest is None:
        color = np.zeros((0, 0, 3), dtype=np.uint8)
        depth = np.zeros((0, 0), dtype=np.uint16)
        mask = np.zeros((0, 0), dtype=np.uint8)
        bbox = ()
        object_msg = None
        stamp_sec = 0.0
        stamp_ns = 0
        frame_id = ''
    else:
        color = np.asarray(latest.color_bgr).copy()
        depth = np.asarray(latest.depth_raw).copy()
        mask = np.zeros(np.asarray(depth).shape[:2], dtype=np.uint8)
        bbox = tuple(latest.bbox or ())
        object_msg = latest.object_msg
        stamp_sec = float(latest.stamp_sec)
        stamp_ns = int(getattr(latest, 'stamp_ns', round(stamp_sec * 1e9)))
        frame_id = str(latest.frame_id or '')
    return SnapshotResult(
        ok=False,
        failure_code=str(code),
        failure_reason=str(reason),
        color_bgr=_readonly_copy(color),
        depth_raw=_readonly_copy(depth),
        target_depth_raw=_readonly_copy(np.zeros_like(depth)),
        object_mask=_readonly_copy(mask),
        bbox=bbox,
        object_msg=object_msg,
        stamp_sec=stamp_sec,
        frame_id=frame_id,
        quality=_empty_quality(len(samples)),
        source_mode=str(source_mode),
        stamp_ns=stamp_ns,
    )


def _fill_internal_holes(target_depth, target_mask, hole_candidates, max_area):
    max_area = max(0, int(max_area))
    if max_area == 0:
        return target_depth
    output = np.asarray(target_depth).copy()
    inside = np.asarray(target_mask) > 0
    holes = inside & (output == 0) & np.asarray(hole_candidates, dtype=bool)
    if not np.any(holes):
        return output
    component_count, labels = cv2.connectedComponents(holes.astype(np.uint8), connectivity=8)
    kernel = np.ones((3, 3), dtype=np.uint8)
    for label in range(1, component_count):
        component = labels == label
        area = int(np.count_nonzero(component))
        if area == 0 or area > max_area:
            continue
        ys, xs = np.nonzero(component)
        if (
            int(np.min(ys)) == 0
            or int(np.max(ys)) == output.shape[0] - 1
            or int(np.min(xs)) == 0
            or int(np.max(xs)) == output.shape[1] - 1
        ):
            continue
        dilated = cv2.dilate(component.astype(np.uint8), kernel, iterations=1) > 0
        ring = dilated & ~component
        if np.any(ring & ~inside):
            continue
        ring_values = output[ring & inside]
        if ring_values.size == 0 or np.any(ring_values == 0):
            continue
        output[component] = np.asarray(np.median(ring_values), dtype=output.dtype)
    return output


def fuse_stable_samples(
    samples,
    require_mask,
    min_mask_iou,
    max_centroid_shift_px,
    max_joint_delta_rad,
    erosion_px,
    depth_scale,
    depth_min_m,
    depth_max_m,
    mad_scale,
    mad_absolute_floor_m,
    internal_hole_max_area_px=25,
):
    samples = list(samples or [])
    source_mode = 'instance_mask' if require_mask else 'bbox_depth'
    if not samples:
        return _failure(samples, 'DEPTH_UNSTABLE', 'no synchronized RGB-D samples', source_mode)

    first_depth = np.asarray(samples[0].depth_raw)
    first_color = np.asarray(samples[0].color_bgr)
    if first_depth.ndim != 2 or first_color.ndim != 3 or first_color.shape[:2] != first_depth.shape:
        return _failure(samples, 'DEPTH_UNSTABLE', 'RGB-D image shapes are inconsistent', source_mode)
    image_shape = first_depth.shape
    frame_id = str(samples[0].frame_id or '')
    masks = []
    for item in samples:
        color = np.asarray(item.color_bgr)
        depth = np.asarray(item.depth_raw)
        if depth.shape != image_shape or color.shape[:2] != image_shape:
            return _failure(samples, 'DEPTH_UNSTABLE', 'RGB-D image shapes changed within window', source_mode)
        if str(item.frame_id or '') != frame_id:
            return _failure(samples, 'DEPTH_UNSTABLE', 'camera frame changed within window', source_mode)
        if require_mask:
            if item.object_mask is None:
                return _failure(samples, 'MASK_MISSING', 'instance mask is missing', source_mode)
            mask = np.asarray(item.object_mask)
            if mask.shape != image_shape:
                return _failure(
                    samples,
                    'MASK_SIZE_MISMATCH',
                    'mask shape %s does not match depth shape %s' % (mask.shape, image_shape),
                    source_mode,
                )
            mask = np.where(mask > 0, 255, 0).astype(np.uint8)
            if not np.any(mask):
                return _failure(samples, 'MASK_EMPTY', 'instance mask is empty', source_mode)
        else:
            mask = _bbox_mask(image_shape, item.bbox)
            if not np.any(mask):
                return _failure(samples, 'DEPTH_UNSTABLE', 'detected bbox is empty', source_mode)
        masks.append(mask)

    min_iou = float(min_mask_iou)
    max_shift = max(0.0, float(max_centroid_shift_px))
    centroids = [_mask_centroid(mask) for mask in masks]
    for index in range(1, len(masks)):
        overlap = mask_iou(masks[index - 1], masks[index])
        if overlap < min_iou:
            label = 'mask IoU' if require_mask else 'bbox IoU'
            return _failure(
                samples,
                'DEPTH_UNSTABLE',
                '%s %.3f is below %.3f' % (label, overlap, min_iou),
                source_mode,
            )
    centroid_shift = max(
        float(np.linalg.norm(centroids[first] - centroids[second]))
        for first in range(len(centroids))
        for second in range(first + 1, len(centroids))
    ) if len(centroids) > 1 else 0.0
    if centroid_shift > max_shift:
        label = 'mask centroid' if require_mask else 'bbox centroid'
        return _failure(
            samples,
            'DEPTH_UNSTABLE',
            '%s shift %.3f px exceeds %.3f px' % (label, centroid_shift, max_shift),
            source_mode,
        )

    joint_rows = [
        np.asarray(item.joint_positions, dtype=float).reshape(-1)
        for item in samples
    ]
    if any(row.size < 6 for row in joint_rows):
        return _failure(samples, 'DEPTH_UNSTABLE', 'six-joint snapshot is unavailable', source_mode)
    joint_stack = np.stack([row[:6] for row in joint_rows], axis=0)
    if not np.all(np.isfinite(joint_stack)):
        return _failure(samples, 'DEPTH_UNSTABLE', 'joint positions contain non-finite values', source_mode)
    joint_delta = float(np.max(np.ptp(joint_stack, axis=0)))
    if joint_delta > max(0.0, float(max_joint_delta_rad)):
        return _failure(
            samples,
            'DEPTH_UNSTABLE',
            'joint motion %.5f rad exceeds %.5f rad'
            % (joint_delta, max(0.0, float(max_joint_delta_rad))),
            source_mode,
        )

    depth_scale = float(depth_scale)
    min_raw = float(depth_min_m) / depth_scale
    max_raw = float(depth_max_m) / depth_scale
    clipped_frames = []
    for item in samples:
        depth = np.asarray(item.depth_raw)
        clipped = depth.copy()
        valid = np.isfinite(clipped) & (clipped.astype(np.float64) >= min_raw) & (clipped.astype(np.float64) <= max_raw)
        clipped[~valid] = 0
        clipped_frames.append(clipped)
    fused_float = np.median(np.stack(clipped_frames, axis=0), axis=0)
    output_dtype = first_depth.dtype
    if np.issubdtype(output_dtype, np.integer):
        fused_depth = np.rint(fused_float).astype(output_dtype)
    else:
        fused_depth = fused_float.astype(output_dtype)

    combined_mask = (np.count_nonzero(np.stack(masks, axis=0), axis=0) >= int(np.ceil(len(masks) / 2.0))).astype(np.uint8) * 255
    erosion = max(0, int(erosion_px))
    if erosion > 0:
        kernel_size = erosion * 2 + 1
        combined_mask = cv2.erode(
            combined_mask,
            np.ones((kernel_size, kernel_size), dtype=np.uint8),
            iterations=1,
        )
    mask_area = int(np.count_nonzero(combined_mask))
    if mask_area == 0:
        code = 'MASK_EMPTY' if require_mask else 'DEPTH_UNSTABLE'
        return _failure(samples, code, 'foreground is empty after erosion', source_mode)

    target_depth = np.zeros_like(fused_depth)
    foreground = combined_mask > 0
    valid_foreground = foreground & (fused_depth > 0)
    values_raw = fused_depth[valid_foreground].astype(np.float64)
    if values_raw.size:
        median_raw = float(np.median(values_raw))
        mad_raw = float(np.median(np.abs(values_raw - median_raw)))
        threshold_m = max(
            max(0.0, float(mad_scale)) * mad_raw * depth_scale,
            max(0.0, float(mad_absolute_floor_m)),
        )
        kept = valid_foreground & (
            np.abs(fused_depth.astype(np.float64) - median_raw) * depth_scale <= threshold_m
        )
        target_depth[kept] = fused_depth[kept]
    else:
        median_raw = 0.0
        mad_raw = 0.0

    target_depth = _fill_internal_holes(
        target_depth,
        combined_mask,
        foreground & (fused_depth == 0),
        internal_hole_max_area_px,
    )
    final_values = target_depth[target_depth > 0].astype(np.float64)
    if final_values.size:
        final_median_raw = float(np.median(final_values))
        final_mad_raw = float(np.median(np.abs(final_values - final_median_raw)))
    else:
        final_median_raw = median_raw
        final_mad_raw = mad_raw
    valid_points = int(final_values.size)
    quality = DepthQuality(
        fused_frames=len(samples),
        mask_area=mask_area,
        valid_depth_points=valid_points,
        valid_depth_ratio=float(valid_points) / float(mask_area) if mask_area else 0.0,
        depth_median_m=final_median_raw * depth_scale if valid_points else 0.0,
        depth_mad_m=final_mad_raw * depth_scale if valid_points else 0.0,
    )
    latest = samples[-1]
    return SnapshotResult(
        ok=True,
        failure_code='',
        failure_reason='',
        color_bgr=_readonly_copy(latest.color_bgr),
        depth_raw=_readonly_copy(fused_depth),
        target_depth_raw=_readonly_copy(target_depth),
        object_mask=_readonly_copy(combined_mask, dtype=np.uint8),
        bbox=tuple(latest.bbox or ()),
        object_msg=latest.object_msg,
        stamp_sec=float(latest.stamp_sec),
        frame_id=str(latest.frame_id or ''),
        quality=quality,
        source_mode=source_mode,
        stamp_ns=int(getattr(latest, 'stamp_ns', round(float(latest.stamp_sec) * 1e9))),
    )


def _stamp_seconds(stamp):
    if hasattr(stamp, 'to_sec'):
        return float(stamp.to_sec())
    if hasattr(stamp, 'secs'):
        return float(stamp.secs) + float(getattr(stamp, 'nsecs', 0)) * 1e-9
    return float(stamp)


def _timestamp_key(stamp):
    if hasattr(stamp, 'to_nsec'):
        return int(stamp.to_nsec())
    if hasattr(stamp, 'secs'):
        return int(stamp.secs) * 1_000_000_000 + int(getattr(stamp, 'nsecs', 0))
    return int(round(_stamp_seconds(stamp) * 1e9))


def _wall_clock_ns():
    if hasattr(time, 'time_ns'):
        return int(time.time_ns())
    return int(round(time.time() * 1e9))


class SynchronizedRgbdBuffer:
    def __init__(self, source_clock_ns=None, monotonic_clock=None):
        self._condition = threading.Condition()
        self._entries = {}
        self._joint_positions = np.zeros(0, dtype=float)
        self._latest_mask_stamp_ns = None
        self._latest_mask_is_empty = None
        self._source_clock_ns = source_clock_ns or _wall_clock_ns
        self._monotonic_clock = monotonic_clock or time.monotonic
        self._update_sequence = 0

    def update_color(self, color_bgr, stamp_sec, frame_id):
        self._update_entry(
            stamp_sec,
            color_bgr=np.asarray(color_bgr).copy(),
            color_frame_id=str(frame_id or ''),
        )

    def update_depth(self, depth_raw, stamp_sec, frame_id):
        self._update_entry(
            stamp_sec,
            depth_raw=np.asarray(depth_raw).copy(),
            depth_frame_id=str(frame_id or ''),
        )

    def update_mask(self, object_mask, stamp_sec, frame_id):
        self._update_entry(
            stamp_sec,
            object_mask=np.asarray(object_mask).copy(),
            mask_frame_id=str(frame_id or ''),
        )

    def update_object(self, object_msg, stamp_sec):
        self._update_entry(stamp_sec, object_msg=deepcopy(object_msg))

    def update_joints(self, joint_positions):
        with self._condition:
            self._joint_positions = np.asarray(joint_positions, dtype=float).reshape(-1).copy()
            self._condition.notify_all()

    def wait_for_samples(
        self,
        count,
        timeout_sec,
        require_mask,
        max_age_sec,
        collection_span_sec=0.0,
        max_inference_latency_sec=None,
    ):
        count = max(1, int(count))
        collection_span = max(0.0, float(collection_span_sec))
        collection_span_ns = int(round(collection_span * 1e9))
        max_inference_latency = max(
            0.0,
            float(
                max_age_sec
                if max_inference_latency_sec is None
                else max_inference_latency_sec
            ),
        )
        deadline = self._monotonic_clock() + max(0.0, float(timeout_sec))
        collected = {}
        invalidated = set()
        with self._condition:
            while True:
                monotonic_now = self._monotonic_clock()
                self._prune_locked(monotonic_now)
                complete = self._complete_entries_locked(
                    bool(require_mask),
                    monotonic_now,
                    max_age_sec,
                    max_inference_latency,
                )
                if collection_span > 0.0:
                    # A low-rate detector cannot keep several frames inside a
                    # sub-second completion-age window at the same instant.
                    # Admit each exact synchronized sample only soon after its
                    # bounded-latency inference completes, then retain an
                    # independent request-local copy for bounded source-stamp
                    # and monotonic collection spans.  This also makes
                    # collection independent of the shared buffer's short
                    # receive-time retention.
                    structurally_complete = dict(
                        self._complete_entries_locked(
                            bool(require_mask),
                            monotonic_now,
                            0.0,
                            0.0,
                        )
                    )
                    for key in list(collected):
                        if key not in self._entries:
                            continue
                        entry = self._entries[key]
                        if (
                            key not in structurally_complete
                            or int(entry.get('revision', 0)) != collected[key][0]
                        ):
                            del collected[key]
                            invalidated.add(key)
                    for key, entry in complete:
                        if key in invalidated:
                            continue
                        revision = int(entry.get('revision', 0))
                        if key not in collected or revision != collected[key][0]:
                            collected[key] = (
                                revision,
                                self._sample_from_entry(entry, bool(require_mask)),
                                monotonic_now,
                            )
                    if collected:
                        newest_stamp_ns = max(collected)
                        oldest_allowed_ns = newest_stamp_ns - collection_span_ns
                        expired_keys = [
                            key
                            for key, (_revision, _sample, admitted_at) in collected.items()
                            if key < oldest_allowed_ns
                            or monotonic_now - float(admitted_at) > collection_span
                        ]
                        for key in expired_keys:
                            del collected[key]
                            invalidated.add(key)
                    if len(collected) >= count:
                        selected_keys = sorted(collected)[-count:]
                        return [collected[key][1] for key in selected_keys]
                elif len(complete) >= count:
                    selected = complete[-count:]
                    return [
                        self._sample_from_entry(entry, bool(require_mask))
                        for _key, entry in selected
                    ]
                remaining = deadline - monotonic_now
                if remaining <= 0.0:
                    return []
                self._condition.wait(remaining)

    def mask_timeout_failure(
        self,
        count,
        max_age_sec,
        max_inference_latency_sec=None,
    ):
        count = max(1, int(count))
        max_age = max(0.0, float(max_age_sec))
        max_inference_latency = max(
            0.0,
            float(
                max_age_sec
                if max_inference_latency_sec is None
                else max_inference_latency_sec
            ),
        )
        with self._condition:
            mask_was_observed = self._latest_mask_stamp_ns is not None
            latest_mask_is_empty = self._latest_mask_is_empty
        count_text = {1: 'one', 2: 'two', 3: 'three'}.get(count, str(count))
        sample_requirement = (
            '%s fresh exact timestamp-matched RGB-D-mask-object samples'
            % count_text
        )
        if not mask_was_observed:
            return (
                'MASK_MISSING',
                'no instance mask has been observed; cannot collect %s'
                % sample_requirement,
            )
        if latest_mask_is_empty:
            return (
                'MASK_EMPTY',
                'latest instance mask is empty; cannot collect %s'
                % sample_requirement,
            )
        if max_age <= 0.0:
            return (
                'MASK_STALE',
                'instance masks were observed but no %s are available; '
                'post-completion age is unlimited and the '
                'source-to-inference completion limit is %.3f s'
                % (sample_requirement, max_inference_latency),
            )
        return (
            'MASK_STALE',
            'instance masks were observed but no %s are available '
            'within the %.3f s post-completion age limit and %.3f s '
            'source-to-inference completion limit'
            % (sample_requirement, max_age, max_inference_latency),
        )

    def discard_through(self, stamp_sec):
        self.discard_through_ns(_timestamp_key(stamp_sec))

    def discard_through_ns(self, stamp_ns):
        key_limit = int(stamp_ns)
        with self._condition:
            for key in [key for key in self._entries if key <= key_limit]:
                del self._entries[key]
            self._condition.notify_all()

    def _update_entry(self, stamp, **values):
        key = _timestamp_key(stamp)
        stamp_sec = _stamp_seconds(stamp)
        now = self._monotonic_clock()
        source_now_ns = int(self._source_clock_ns())
        with self._condition:
            self._prune_locked(now)
            if 'object_mask' in values:
                if self._latest_mask_stamp_ns is None or key >= self._latest_mask_stamp_ns:
                    self._latest_mask_stamp_ns = key
                    self._latest_mask_is_empty = not bool(
                        np.any(np.asarray(values['object_mask']))
                    )
            entry = self._entries.setdefault(
                key,
                {
                    'stamp_sec': stamp_sec,
                    'stamp_ns': key,
                    'created_at': now,
                    'updated_at': now,
                    'joint_positions': self._joint_positions.copy(),
                    'first_received_at': {},
                    'latest_received_source_ns': {},
                },
            )
            entry.update(values)
            for name in values:
                # Duplicate components may replace their payload but cannot
                # renew the entry's completion age.  Their latest receipt is
                # still used by the inference-latency gate so a late replay
                # can only make the entry less eligible, never fresher.
                entry['first_received_at'].setdefault(name, now)
                entry['latest_received_source_ns'][name] = source_now_ns
            self._update_sequence += 1
            entry['revision'] = self._update_sequence
            entry['updated_at'] = now
            if entry['joint_positions'].size == 0 and self._joint_positions.size:
                entry['joint_positions'] = self._joint_positions.copy()
            self._condition.notify_all()

    def _prune_locked(self, now):
        cutoff = float(now) - 2.0
        for key in [
            key
            for key, entry in self._entries.items()
            if float(entry.get('created_at', now)) < cutoff
        ]:
            del self._entries[key]

    def _complete_entries_locked(
        self,
        require_mask,
        monotonic_now,
        max_age_sec,
        max_inference_latency_sec,
    ):
        max_age = max(0.0, float(max_age_sec))
        max_inference_latency = max(0.0, float(max_inference_latency_sec))
        max_inference_latency_ns = int(round(max_inference_latency * 1e9))
        complete = []
        for key, entry in sorted(self._entries.items()):
            stamp_ns = int(entry['stamp_ns'])
            required = ('color_bgr', 'depth_raw', 'object_msg')
            if require_mask and 'object_mask' not in entry:
                continue
            if any(name not in entry for name in required):
                continue
            if require_mask and not bool(np.any(np.asarray(entry['object_mask']))):
                continue
            timing_names = required + (('object_mask',) if require_mask else ())
            first_received_at = entry.get('first_received_at', {})
            latest_received_source_ns = entry.get(
                'latest_received_source_ns',
                {},
            )
            if any(
                name not in first_received_at
                or name not in latest_received_source_ns
                for name in timing_names
            ):
                continue
            component_source_receipts = [
                int(latest_received_source_ns[name]) for name in timing_names
            ]
            # A source stamp that was in the future when any component arrived
            # is never allowed to become valid merely because a clock later
            # catches up.
            if any(received_ns < stamp_ns for received_ns in component_source_receipts):
                continue
            completion_source_ns = max(component_source_receipts)
            if (
                max_inference_latency > 0.0
                and completion_source_ns - stamp_ns > max_inference_latency_ns
            ):
                continue
            completion_at = max(
                float(first_received_at[name]) for name in timing_names
            )
            completion_age = float(monotonic_now) - completion_at
            if completion_age < 0.0:
                continue
            if max_age > 0.0 and completion_age > max_age:
                continue
            if not bool(getattr(entry['object_msg'], 'detected', False)):
                continue
            frame_ids = [
                str(entry.get('color_frame_id', '')),
                str(entry.get('depth_frame_id', '')),
            ]
            if require_mask:
                frame_ids.append(str(entry.get('mask_frame_id', '')))
            if len(set(frame_ids)) != 1:
                continue
            complete.append((key, entry))
        return complete

    @staticmethod
    def _sample_from_entry(entry, require_mask):
        object_msg = deepcopy(entry['object_msg'])
        bbox = (
            int(getattr(object_msg, 'bbox_x', 0)),
            int(getattr(object_msg, 'bbox_y', 0)),
            int(getattr(object_msg, 'bbox_width', 0)),
            int(getattr(object_msg, 'bbox_height', 0)),
        )
        frame_id = str(entry.get('color_frame_id') or entry.get('depth_frame_id') or entry.get('mask_frame_id') or '')
        return RgbdSample(
            color_bgr=np.asarray(entry['color_bgr']).copy(),
            depth_raw=np.asarray(entry['depth_raw']).copy(),
            object_mask=(
                np.asarray(entry['object_mask']).copy()
                if require_mask and 'object_mask' in entry
                else None
            ),
            bbox=bbox,
            object_msg=object_msg,
            stamp_sec=float(entry['stamp_sec']),
            frame_id=frame_id,
            joint_positions=np.asarray(entry.get('joint_positions', ()), dtype=float).copy(),
            stamp_ns=int(entry['stamp_ns']),
        )
