"""Colour regions that can request observation, never establish a grasp target.

An RGB extremal region may be an object, a marking, or a shadow. Its pixels are
evidence about where to look for additional measurements, not evidence of depth,
height, free space, object identity, or reachability. In particular a depth hole
does not make a region more likely to be selected. This module has no ROS or
motion interface and does not fill, project, or synthesize depth.
"""
from dataclasses import dataclass, field, replace
import hashlib
from typing import Optional, Tuple

import cv2
import numpy as np


@dataclass(frozen=True)
class CenterSearchPolicy:
    """Explicit image-space attention policy, independent of object colour/size.

    The centre is in normalized image coordinates; radius is a fraction of the
    shorter image side. Area bounds reject subpixel fragments and broad image
    backgrounds, and are fractions of image area, not physical size limits.
    """
    center_xy: Tuple[float, float]
    radius_fraction: float = .25
    min_area_fraction: float = .00012
    max_area_fraction: float = .10
    min_solidity: float = .70
    min_compactness: float = .30
    max_aspect_ratio: float = 5.
    min_local_contrast: float = 12.
    center_distance_weight: float = 4.
    ambiguity_score_margin: float = .12
    max_returned_candidates: int = 16
    max_analysis_pixels: int = 1048576
    max_evaluated_regions: int = 512
    max_candidate_mask_pixels: int = 8388608

    def __post_init__(self):
        if (len(self.center_xy) != 2 or not np.all(np.isfinite(self.center_xy))
                or not all(0. <= x <= 1. for x in self.center_xy)):
            raise ValueError('invalid_search_center')
        values = (self.radius_fraction, self.min_area_fraction,
                  self.max_area_fraction, self.min_solidity,
                  self.min_compactness, self.max_aspect_ratio,
                  self.min_local_contrast, self.center_distance_weight,
                  self.ambiguity_score_margin)
        if not np.all(np.isfinite(values)):
            raise ValueError('nonfinite_search_policy')
        if (not 0. < self.radius_fraction <= 1.
                or not 0. < self.min_area_fraction < self.max_area_fraction < .5
                or not 0. < self.min_solidity <= 1.
                or not 0. < self.min_compactness <= 1.
                or self.max_aspect_ratio < 1.
                or self.min_local_contrast <= 0.
                or self.center_distance_weight <= 0.
                or self.ambiguity_score_margin < 0.
                or any(not isinstance(value, int) or isinstance(value, bool)
                       for value in (self.max_returned_candidates,
                                     self.max_analysis_pixels, self.max_evaluated_regions,
                                     self.max_candidate_mask_pixels))
                or self.max_returned_candidates < 2 or self.max_analysis_pixels < 144
                or self.max_evaluated_regions < 2 or self.max_candidate_mask_pixels < 144):
            raise ValueError('invalid_search_policy')


@dataclass(frozen=True)
class RgbObservationRegion:
    # Full-image mask of measured RGB pixels belonging to the region. This is
    # deliberately not named target_mask: geometric target ownership is unknown.
    mask: np.ndarray = field(repr=False, compare=False)
    bbox_xywh: Tuple[int, int, int, int]
    centroid_xy: Tuple[float, float]
    area_px: int
    solidity: float
    compactness: float
    local_contrast: float
    center_distance_fraction: float
    score: float
    channels: Tuple[str, ...]
    valid_depth_fraction: Optional[float]
    observation_only: bool = field(default=True, init=False)
    metric_geometry_authority: bool = field(default=False, init=False)
    object_identity_confirmed: bool = field(default=False, init=False)


@dataclass(frozen=True)
class RgbObservationRegions:
    candidates: Tuple[RgbObservationRegion, ...]
    selected_index: Optional[int]
    reason: str
    image_shape: Tuple[int, int]
    total_candidate_count: int
    evaluated_regions: int = 0
    compact_mask_pixels: int = 0
    returned_mask_pixels: int = 0
    mask_storage_peak_pixels: int = 0
    observation_only: bool = field(default=True, init=False)
    metric_geometry_authority: bool = field(default=False, init=False)

    @property
    def selected(self):
        return (None if self.selected_index is None
                else self.candidates[self.selected_index])


@dataclass(frozen=True)
class _CompactRegion:
    """Private candidate: its exact pixel mask is local to ``bbox_xywh``.

    MSER emits many threshold variants of one outline. These variants must not
    each allocate a full-frame mask before duplicate hypotheses are merged.
    """
    mask: np.ndarray = field(repr=False, compare=False)
    bbox_xywh: Tuple[int, int, int, int]
    centroid_xy: Tuple[float, float]
    area_px: int
    solidity: float
    compactness: float
    local_contrast: float
    center_distance_fraction: float
    score: float
    channels: Tuple[str, ...]
    valid_depth_fraction: Optional[float]


def _local_mask(region):
    if isinstance(region, _CompactRegion):
        return region.mask
    x, y, width, height = region.bbox_xywh
    return region.mask[y:y+height, x:x+width]


def _intersection(region, other):
    ax, ay, aw, ah = region.bbox_xywh
    bx, by, bw, bh = other.bbox_xywh
    x0, y0 = max(ax, bx), max(ay, by)
    x1, y1 = min(ax + aw, bx + bw), min(ay + ah, by + bh)
    if x0 >= x1 or y0 >= y1:
        return 0
    return int(np.count_nonzero(_local_mask(region)[y0-ay:y1-ay, x0-ax:x1-ax]
                               & _local_mask(other)[y0-by:y1-by, x0-bx:x1-bx]))


def _same_region(first, second):
    intersection = _intersection(first, second)
    union = first.area_px + second.area_px - intersection
    # Nested, concentric boundaries are alternative extents of the same place
    # to observe. An off-centre component inside a broad region remains a
    # separate hypothesis. This groups locations, not semantic object parts.
    near_center = np.linalg.norm(np.array(first.centroid_xy) - second.centroid_xy) <= (
        .15 * np.sqrt(max(first.area_px, second.area_px)))
    nested_outline = False
    if near_center:
        for outer, inner in ((first, second), (second, first)):
            ox, oy, ow, oh = outer.bbox_xywh
            ix, iy, iw, ih = inner.bbox_xywh
            if ox < ix and oy < iy and ox + ow > ix + iw and oy + oh > iy + ih:
                outlines, _ = cv2.findContours(
                    _local_mask(outer).astype(np.uint8),
                    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if len(outlines) == 1:
                    nested_outline = all(cv2.pointPolygonTest(outlines[0], point, False) >= 0.
                                         for point in ((float(ix-ox), float(iy-oy)),
                                                       (float(ix+iw-1-ox), float(iy-oy)),
                                                       (float(ix-ox), float(iy+ih-1-oy)),
                                                       (float(ix+iw-1-ox), float(iy+ih-1-oy))))
                if nested_outline:
                    break
    return (intersection / max(1, union) >= .72
            or (intersection / min(first.area_px, second.area_px) >= .92
                and near_center) or nested_outline)


def propose_rgb_observation_regions(bgr, *, policy, depth_valid=None, rgb_valid=None):
    """Find compact regions with a closed boundary and local colour contrast.

    MSER is run on luminance and both Lab chromatic channels; neither dark nor
    bright foreground is preferred. Selection expresses only the explicit
    centre attention policy. Comparable distinct alternatives cause abstention.
    Optional ``depth_valid`` must be a boolean validity mask: it contributes only
    a diagnostic fraction, never the candidate masks, ranking, or acceptance.
    Optional ``rgb_valid`` identifies pixels backed by source RGB. A candidate
    and its contrast ring must both be observed; padding is not image evidence.
    """
    if not isinstance(policy, CenterSearchPolicy):
        raise ValueError('explicit_center_search_policy_required')
    colour = np.asarray(bgr)
    if (colour.dtype != np.uint8 or colour.ndim != 3 or colour.shape[2] != 3
            or min(colour.shape[:2]) < 12):
        raise ValueError('invalid_bgr_image')
    height, width = colour.shape[:2]
    if depth_valid is not None:
        depth_valid = np.asarray(depth_valid)
        if depth_valid.shape != (height, width) or depth_valid.dtype != np.bool_:
            raise ValueError('invalid_depth_validity_mask')
    if rgb_valid is not None:
        rgb_valid = np.asarray(rgb_valid)
        if rgb_valid.shape != (height, width) or rgb_valid.dtype != np.bool_:
            raise ValueError('invalid_rgb_validity_mask')
    shorter = float(min(height, width))
    center = np.array(policy.center_xy) * np.array((width - 1, height - 1))
    radius = policy.radius_fraction * shorter
    # Analyze only the explicit attention window plus shape context. A region
    # touching this crop boundary is rejected: it has no closed observed outline
    # in the bounded analysis. This is not a caller-selected object ROI.
    context_radius = radius + .15 * shorter
    crop_x0, crop_y0 = np.maximum(0, np.floor(center - context_radius)).astype(int)
    crop_x1, crop_y1 = np.minimum((width, height), np.ceil(center + context_radius + 1)).astype(int)
    if (crop_x1 - crop_x0) * (crop_y1 - crop_y0) > policy.max_analysis_pixels:
        return RgbObservationRegions((), None, 'rgb_analysis_resolution_budget_exceeded',
                                     (height, width), 0)
    analysis_colour = np.ascontiguousarray(colour[crop_y0:crop_y1, crop_x0:crop_x1])
    min_area = max(12, int(np.ceil(policy.min_area_fraction * height * width)))
    max_area = int(np.floor(policy.max_area_fraction * height * width))
    # A disconnected fragment smaller than this cannot grow just because it is
    # near a depth hole. All pixel evidence comes from this current image.
    mser = cv2.MSER_create(5, min_area, max_area, .25, .2)
    lab = cv2.cvtColor(analysis_colour, cv2.COLOR_BGR2LAB)
    grey = cv2.cvtColor(analysis_colour, cv2.COLOR_BGR2GRAY)
    channels = (('gray', grey), ('lab_l', lab[:, :, 0]),
                ('lab_a', lab[:, :, 1]), ('lab_b', lab[:, :, 2]))
    proposals = []
    border = max(1, int(round(shorter / 480.)))
    seen_pixels = {}
    evaluated = 0
    budget_exhausted = False
    mask_budget_exhausted = False
    compact_mask_pixels = 0
    for channel_name, channel in channels:
        regions, _ = mser.detectRegions(np.ascontiguousarray(channel))
        for coordinates in regions:
            coordinates = np.asarray(coordinates) + np.array((crop_x0, crop_y0))
            count = len(coordinates)
            if not min_area <= count <= max_area:
                continue
            if rgb_valid is not None and not np.all(rgb_valid[coordinates[:, 1], coordinates[:, 0]]):
                continue
            xmin, ymin = coordinates.min(axis=0)
            xmax, ymax = coordinates.max(axis=0)
            bw, bh = int(xmax - xmin + 1), int(ymax - ymin + 1)
            if (xmin <= crop_x0 + border or ymin <= crop_y0 + border
                    or xmax >= crop_x1 - 1 - border or ymax >= crop_y1 - 1 - border
                    or max(bw, bh) / float(min(bw, bh)) > policy.max_aspect_ratio):
                continue
            centroid = coordinates.mean(axis=0)
            distance = float(np.linalg.norm(centroid - center))
            if distance > radius:
                continue
            patch = np.zeros((bh, bw), np.uint8)
            patch[coordinates[:, 1] - ymin, coordinates[:, 0] - xmin] = 1
            # Identical pixel sets across channels should not change ranking.
            key = (int(xmin), int(ymin), bw, bh, hashlib.sha256(patch).digest())
            if key in seen_pixels:
                previous = seen_pixels[key]
                if previous is not None:
                    proposals[previous] = replace(proposals[previous], channels=tuple(sorted(
                        set(proposals[previous].channels + (channel_name,)))))
                continue
            seen_pixels[key] = None
            evaluated += 1
            if evaluated > policy.max_evaluated_regions:
                budget_exhausted = True
                break
            contours, _ = cv2.findContours(patch, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) != 1:
                continue
            contour = contours[0]
            contour_area = float(cv2.contourArea(contour))
            hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
            perimeter = float(cv2.arcLength(contour, True))
            if min(contour_area, hull_area, perimeter) <= 0.:
                continue
            solidity = contour_area / hull_area
            compactness = min(1., 4. * np.pi * contour_area / perimeter ** 2)
            if (solidity < policy.min_solidity
                    or compactness < policy.min_compactness):
                continue
            ring_radius = max(2, int(round(np.sqrt(count) * .12)))
            x0, y0 = max(crop_x0, int(xmin) - ring_radius), max(crop_y0, int(ymin) - ring_radius)
            x1, y1 = min(crop_x1, int(xmax) + ring_radius + 1), min(crop_y1, int(ymax) + ring_radius + 1)
            local_mask = np.zeros((y1 - y0, x1 - x0), np.uint8)
            local_mask[coordinates[:, 1] - y0, coordinates[:, 0] - x0] = 1
            ring = ((cv2.dilate(local_mask, np.ones((ring_radius * 2 + 1,) * 2,
                                                  np.uint8)) > 0)
                    & (local_mask == 0))
            if rgb_valid is not None and not np.all(rgb_valid[y0:y1, x0:x1][ring]):
                continue
            local_lab = lab[y0-crop_y0:y1-crop_y0, x0-crop_x0:x1-crop_x0].astype(np.float32)
            contrast = float(np.linalg.norm(np.median(local_lab[local_mask > 0], axis=0)
                                           - np.median(local_lab[ring], axis=0)))
            if contrast < policy.min_local_contrast:
                continue
            if compact_mask_pixels + patch.size > policy.max_candidate_mask_pixels:
                mask_budget_exhausted = True
                break
            mask = patch.astype(bool)
            mask.setflags(write=False)
            depth_fraction = (None if depth_valid is None else
                              float(np.count_nonzero(
                                  depth_valid[ymin:ymax+1, xmin:xmax+1] & mask)) / count)
            # Search radius controls eligibility, not confidence. Shrinking the
            # attention window must not silently change the ordering/ambiguity
            # of hypotheses that remain inside it.
            score = (policy.center_distance_weight * distance / shorter
                     + .25 * (1. - solidity)
                     + .15 * (1. - compactness) - .1 * min(1., contrast / 100.))
            seen_pixels[key] = len(proposals)
            proposals.append(_CompactRegion(
                mask=mask, bbox_xywh=(int(xmin), int(ymin), bw, bh),
                centroid_xy=tuple(map(float, centroid)), area_px=count,
                solidity=solidity, compactness=compactness,
                local_contrast=contrast, center_distance_fraction=distance / shorter,
                score=score, channels=(channel_name,), valid_depth_fraction=depth_fraction))
            compact_mask_pixels += mask.size
        if budget_exhausted or mask_budget_exhausted:
            break
    proposals.sort(key=lambda item: (item.score, -item.area_px, item.bbox_xywh))
    unique = []
    for region in proposals:
        duplicates = [index for index, previous in enumerate(unique)
                      if _same_region(region, previous)]
        if not duplicates:
            unique.append(region)
        else:
            index = duplicates[0]
            unique[index] = replace(unique[index], channels=tuple(sorted(
                set(unique[index].channels + region.channels))))
    # Test ambiguity before limiting the returned diagnostic list.
    ambiguous = (len(unique) > 1 and unique[1].score - unique[0].score
                 <= policy.ambiguity_score_margin)
    returned = []
    returned_mask_pixels = 0
    # Only materialize the final diagnostic list. Account for the compact
    # candidates still alive while constructing the public full-frame masks,
    # keeping the same total mask budget rather than raising it to hide copies.
    for region in unique[:policy.max_returned_candidates]:
        if compact_mask_pixels + returned_mask_pixels + height * width > policy.max_candidate_mask_pixels:
            mask_budget_exhausted = True
            break
        mask = np.zeros((height, width), bool)
        x, y, bw, bh = region.bbox_xywh
        mask[y:y+bh, x:x+bw] = region.mask
        mask.setflags(write=False)
        returned.append(RgbObservationRegion(
            mask=mask, bbox_xywh=region.bbox_xywh, centroid_xy=region.centroid_xy,
            area_px=region.area_px, solidity=region.solidity,
            compactness=region.compactness, local_contrast=region.local_contrast,
            center_distance_fraction=region.center_distance_fraction,
            score=region.score, channels=region.channels,
            valid_depth_fraction=region.valid_depth_fraction))
        returned_mask_pixels += mask.size
    return RgbObservationRegions(
        candidates=tuple(returned),
        selected_index=(0 if returned and not ambiguous
                        and not budget_exhausted and not mask_budget_exhausted else None),
        reason=('rgb_candidate_mask_budget_exhausted' if mask_budget_exhausted else
                'rgb_analysis_region_budget_exhausted' if budget_exhausted else
                'no_compact_rgb_region' if not unique else
                'ambiguous_rgb_regions' if ambiguous else 'observation_region_found'),
        image_shape=(height, width), total_candidate_count=len(unique),
        evaluated_regions=min(evaluated, policy.max_evaluated_regions),
        compact_mask_pixels=compact_mask_pixels, returned_mask_pixels=returned_mask_pixels,
        mask_storage_peak_pixels=compact_mask_pixels + returned_mask_pixels)


@dataclass(frozen=True)
class RgbObservationTrack:
    confirmed: bool
    reason: str
    consistent_frames: int
    source_id: str
    generation: int
    source_stamps_ns: Tuple[int, ...]
    region: Optional[RgbObservationRegion]
    observation_only: bool = field(default=True, init=False)
    metric_geometry_authority: bool = field(default=False, init=False)


class RgbObservationTracker:
    """Require three distinct, consistent images from one source/selection/pose.

    This confirms only repeatable RGB evidence. It does not convert the ROI into
    an object or authorize motion. All frames compare against the first pose and
    mask, preventing incremental drift from accumulating into a new identity.
    The caller must validate that image, source identity and measured joints are
    actually synchronized. This pure tracker cannot verify TF, calibration, or
    hardware freshness, and its confirmation is not an execution binding.
    """
    def __init__(self, *, required_frames=3, min_mask_iou=.70,
                 max_joint_delta_rad=.004, max_gap_seconds=1.,
                 max_centroid_shift_fraction=.015):
        values = (min_mask_iou, max_joint_delta_rad, max_gap_seconds,
                  max_centroid_shift_fraction)
        if (not isinstance(required_frames, int) or isinstance(required_frames, bool)
                or required_frames < 3
                or not np.all(np.isfinite(values)) or not 0. < min_mask_iou <= 1.
                or min(max_joint_delta_rad, max_gap_seconds,
                       max_centroid_shift_fraction) <= 0.):
            raise ValueError('invalid_rgb_tracker_policy')
        self.required_frames = required_frames
        self.min_mask_iou = min_mask_iou
        self.max_joint_delta_rad = max_joint_delta_rad
        self.max_gap_ns = int(max_gap_seconds * 1e9)
        self.max_centroid_shift_fraction = max_centroid_shift_fraction
        self.reset()

    def reset(self):
        self._key = None
        self._region = None
        self._joints = None
        self._shape = None
        self._stamps = []
        self._last_stamp = None

    def observe(self, result, *, source_id, generation, stamp_ns, joints_rad):
        if (not isinstance(result, RgbObservationRegions) or not isinstance(source_id, str)
                or not source_id or not isinstance(generation, (int, np.integer))
                or isinstance(generation, bool) or generation < 0
                or not isinstance(stamp_ns, (int, np.integer))
                or isinstance(stamp_ns, bool) or stamp_ns <= 0):
            self.reset()
            raise ValueError('invalid_rgb_source_identity')
        joints = np.asarray(joints_rad, dtype=float)
        if joints.ndim != 1 or joints.size == 0 or not np.all(np.isfinite(joints)):
            self.reset()
            raise ValueError('invalid_measured_joints')
        key = (source_id, int(generation))
        if key != self._key:
            self.reset()
            self._key = key
        if self._last_stamp is not None and stamp_ns <= self._last_stamp:
            self._region, self._joints, self._stamps = None, None, []
            return self._report(False, 'nonincreasing_source_stamp', key, None)
        gap = self._last_stamp is not None and stamp_ns - self._last_stamp > self.max_gap_ns
        self._last_stamp = int(stamp_ns)
        region = result.selected
        if region is None:
            self._region, self._joints, self._stamps = None, None, []
            return self._report(False, result.reason, key, None)
        consistent = self._region is not None and not gap and result.image_shape == self._shape
        if consistent:
            consistent = (joints.shape == self._joints.shape
                          and np.max(np.abs(joints - self._joints)) <= self.max_joint_delta_rad)
        if consistent:
            intersection = _intersection(region, self._region)
            iou = intersection / max(1, region.area_px + self._region.area_px - intersection)
            shift = np.linalg.norm(np.array(region.centroid_xy) - self._region.centroid_xy)
            consistent = (iou >= self.min_mask_iou
                          and shift <= self.max_centroid_shift_fraction * min(result.image_shape))
        if not consistent:
            self._stamps = []
            self._region, self._joints, self._shape = region, joints.copy(), result.image_shape
        self._stamps.append(int(stamp_ns))
        self._stamps = self._stamps[-self.required_frames:]
        confirmed = len(self._stamps) >= self.required_frames
        return self._report(confirmed, 'stable_rgb_observation_region' if confirmed
                            else 'collecting_consistent_rgb_frames', key, region)

    def _report(self, confirmed, reason, key, region):
        return RgbObservationTrack(confirmed, reason, len(self._stamps), key[0], key[1],
                                   tuple(self._stamps), region)
