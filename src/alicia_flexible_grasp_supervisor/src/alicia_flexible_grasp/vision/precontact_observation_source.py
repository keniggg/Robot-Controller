"""Bind failed contact geometry to a request without granting motion authority.

This is an offline/pure adapter. The caller must hold its request lifecycle lock
while supplying the current ticket/identity, and must revalidate that lifecycle
before any later use. No ROS lookup, planning, publication or device operation
occurs here. A returned observation is measured input, not an executable plan.
"""
from dataclasses import dataclass, fields, field
import hashlib
import json
from numbers import Real

import numpy as np

from .latest_only_inference import InferenceTicket
from .object_geometry import (GeometryEstimate, ObservationGeometryDeficit,
                              estimate_object_geometry, _validated_transform)
from .rgbd_snapshot import SnapshotResult, DepthQuality, retained_exact_source
from .target_observation import (TargetObservation, _integer, _immutable_finite,
                                 validate_target_identity)


def _scalar(value, name, positive=False):
    if (isinstance(value, (bool, np.bool_)) or not isinstance(value, Real)
            or not np.isfinite(value) or (positive and value <= 0.)):
        raise ValueError(name + ' must be finite numeric' + (' and positive' if positive else ''))
    return float(value)


def _frame(value, name):
    if (not isinstance(value, str) or not value or value != value.strip()
            or any(character.isspace() for character in value)):
        raise ValueError(name + ' must be a canonical frame')
    return value


def _stamp_ns(stamp):
    secs = _integer(getattr(stamp, 'secs', None), 'stamp seconds')
    nsecs = _integer(getattr(stamp, 'nsecs', None), 'stamp nanoseconds')
    if nsecs >= 10**9:
        raise ValueError('stamp nanoseconds exceed a second')
    return secs * 10**9 + nsecs


@dataclass(frozen=True)
class ObservationGeometryConfig:
    """Original active configuration, supplied independently of failed output."""
    support_bbox_expand_ratio: float
    support_distance_threshold_m: float
    voxel_size_m: float
    min_support_points: int
    min_object_points: int
    min_size_m: float
    max_size_m: float
    max_height_m: float
    outlier_neighbors: int = 16
    outlier_std_ratio: float = 2.
    support_minimum_span_m: float = 0.

    def __post_init__(self):
        for item in fields(self):
            name, value = item.name, getattr(self, item.name)
            if name in ('min_support_points', 'min_object_points', 'outlier_neighbors'):
                value = _integer(value, name, 1 if name == 'outlier_neighbors' else 3)
            else:
                value = _scalar(value, name)
                if value < 0. or (name not in ('support_bbox_expand_ratio',
                        'outlier_std_ratio', 'support_minimum_span_m') and value == 0.):
                    raise ValueError('invalid geometry configuration ' + name)
            object.__setattr__(self, name, value)
        if self.max_size_m < self.min_size_m:
            raise ValueError('geometry maximum is below minimum')


@dataclass(frozen=True)
class PrecontactObservationSource:
    """Source-bound measurements; every executable-authority flag stays false.

    TargetObservation validates its structural measurement contract, but its
    support is the contact estimator's context. It is NOT independent support
    evidence. A fused mask is NOT proof of a complete RGB object outline.
    """
    request_id: int
    request_generation: int
    observation: TargetObservation
    deficit: ObservationGeometryDeficit
    geometry_config: ObservationGeometryConfig
    sample_stamp_ns: tuple
    source_frame_id: str
    transform_stamp_ns: int
    admitted_at_ns: int
    source_expires_at_ns: int
    transform_base_optical: np.ndarray
    source_sha256: str
    kind: str = field(default='source_bound_contact_geometry_deficit', init=False)
    contact_geometry_authorized: bool = field(default=False, init=False)
    observation_motion_authorized: bool = field(default=False, init=False)
    planning_authorized: bool = field(default=False, init=False)
    missing_requirements: tuple = field(default=(
        'current_lifecycle_and_calibration_context', 'complete_rgb_identity',
        'independent_support_quality', 'conservative_occupied_scene',
        'collision_checked_observation_path', 'bounded_information_gain'), init=False)

    def __post_init__(self):
        if (type(self.observation) is not TargetObservation
                or type(self.deficit) is not ObservationGeometryDeficit
                or type(self.geometry_config) is not ObservationGeometryConfig):
            raise ValueError('source requires typed measurement evidence')
        if (not np.array_equal(self.observation.points_base, self.deficit.object_points_base)
                or not np.array_equal(self.observation.support_normal_base, self.deficit.support_normal_base)
                or self.observation.support_offset_m != self.deficit.support_offset_m
                or self.observation.source_kind != self.deficit.source_mode):
            raise ValueError('source observation disagrees with measured deficit')
        for item in fields(self.geometry_config):
            if (hasattr(self.deficit, item.name)
                    and getattr(self.geometry_config, item.name) != getattr(self.deficit, item.name)):
                raise ValueError('source configuration disagrees with measured deficit')
        object.__setattr__(self, 'request_id', _integer(self.request_id, 'request_id', 1))
        object.__setattr__(self, 'request_generation', _integer(self.request_generation, 'generation', 1))
        stamps = tuple(_integer(stamp, 'sample stamp', 1) for stamp in self.sample_stamp_ns)
        if (not stamps or stamps[-1] != self.observation.stamp_ns
                or any(b <= a for a, b in zip(stamps, stamps[1:]))
                or _integer(self.transform_stamp_ns, 'TF stamp', 1) != stamps[-1]):
            raise ValueError('source sample/TF stamps disagree')
        object.__setattr__(self, 'sample_stamp_ns', stamps)
        admitted = _integer(self.admitted_at_ns, 'admission stamp', 1)
        expires = _integer(self.source_expires_at_ns, 'source expiry', 1)
        if not stamps[-1] <= admitted <= expires:
            raise ValueError('source admission is outside its original lifetime')
        _frame(self.source_frame_id, 'source frame')
        transform = _immutable_finite(self.transform_base_optical, (4, 4), 'source TF')
        _validated_transform(transform)
        if not np.array_equal(transform, self.deficit.T_base_camera):
            raise ValueError('source TF disagrees with geometry')
        object.__setattr__(self, 'transform_base_optical', transform)
        if (not isinstance(self.source_sha256, str) or len(self.source_sha256) != 64
                or any(c not in '0123456789abcdef' for c in self.source_sha256)):
            raise ValueError('source digest must be SHA256')

    def __deepcopy__(self, memo):
        # Nested observation arrays are also backed by immutable bytes.
        memo[id(self)] = self
        return self


def _exact_base_optical(exact_tf, stamp_ns, source_frame, base_frame, source_from_optical):
    """Consume the actual lookup result, never a bool claiming TF succeeded."""
    header = getattr(exact_tf, 'header', None)
    if (getattr(header, 'frame_id', None) != base_frame
            or getattr(exact_tf, 'child_frame_id', None) != source_frame
            or _stamp_ns(getattr(header, 'stamp', None)) != stamp_ns):
        raise ValueError('TF must be the exact requested stamp and frame pair')
    parts = getattr(exact_tf, 'transform', None)
    xyz = getattr(parts, 'translation', None)
    quat = getattr(parts, 'rotation', None)
    translation = np.asarray([_scalar(getattr(xyz, axis, None), 'TF translation')
                              for axis in ('x', 'y', 'z')])
    x, y, z, w = [_scalar(getattr(quat, axis, None), 'TF rotation')
                  for axis in ('x', 'y', 'z', 'w')]
    if not np.isclose(x*x+y*y+z*z+w*w, 1., rtol=0., atol=1e-6):
        raise ValueError('TF quaternion must be unit length')
    result = np.eye(4)
    result[:3, :3] = ((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
                      (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
                      (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))
    result[:3, 3] = translation
    optical = _validated_transform(source_from_optical)
    return _validated_transform(result @ optical)


def _detection(snapshot, original, base_frame):
    for item in (snapshot, original):
        obj = item.object_msg
        if getattr(obj, 'detected', None) is not True:
            raise ValueError('snapshot target must be detected')
        header = getattr(obj, 'header', None)
        if (getattr(header, 'frame_id', None) != base_frame
                or _stamp_ns(getattr(header, 'stamp', None)) != snapshot.stamp_ns):
            raise ValueError('detection source header disagrees with snapshot')
        bbox = tuple(_integer(getattr(obj, name, None), 'detection bbox') for name in
                     ('bbox_x', 'bbox_y', 'bbox_width', 'bbox_height'))
        if bbox != snapshot.bbox:
            raise ValueError('detection bbox disagrees with snapshot')
        pose_header = getattr(getattr(obj, 'pose_camera', None), 'header', None)
        if (getattr(pose_header, 'frame_id', None) != snapshot.frame_id
                or _stamp_ns(getattr(pose_header, 'stamp', None)) != snapshot.stamp_ns):
            raise ValueError('detection camera source disagrees with snapshot')


def bind_precontact_observation_source(*, ticket, current_ticket,
        current_target_identity, geometry, geometry_config, intrinsics, depth_scale,
        exact_tf, source_from_optical, now_stamp_ns, max_source_age_ns,
        base_frame='base_link'):
    """Return observation-only provenance or raise ValueError, never a plan.

    ``exact_tf`` is the actual TransformStamped returned for the snapshot's
    integer stamp; even a 1 ns mismatch is rejected. ``source_from_optical``
    explicitly supplies the calibrated source-frame/optical-axis convention.
    For now only instance_mask is supported: bbox_depth preparation changes its
    foreground later, so this adapter cannot assume those are the same inputs.
    Original snapshot admission owns the earlier samples' mask/motion checks;
    only the final raw sample is retained, not invented per-frame evidence.
    """
    if type(ticket) is not InferenceTicket or type(current_ticket) is not InferenceTicket:
        raise ValueError('current request must be an InferenceTicket')
    for name in ('request_id', 'generation', 'target_epoch'):
        value = _integer(getattr(ticket, name), name, 0 if name == 'target_epoch' else 1)
        if value != _integer(getattr(current_ticket, name), 'current ' + name):
            raise ValueError('current request ticket changed')
    for name in ('snapshot_stamp_sec', 'submitted_monotonic_sec'):
        if _scalar(getattr(ticket, name), name) != _scalar(getattr(current_ticket, name), name):
            raise ValueError('current request ticket changed')
    if (not isinstance(ticket.payload, tuple) or len(ticket.payload) != 2
            or not isinstance(current_ticket.payload, tuple) or len(current_ticket.payload) != 2
            or current_ticket.payload[0] is not ticket.payload[0]
            or current_ticket.payload[1] is not ticket.payload[1]):
        raise ValueError('ticket must own the current snapshot/config payload')
    snapshot = ticket.payload[0]
    if (type(snapshot) is not SnapshotResult or snapshot.ok is not True
            or snapshot.failure_code or snapshot.failure_reason):
        raise ValueError('request needs a valid SnapshotResult')
    identity = validate_target_identity(current_target_identity)
    if (snapshot.target_identity != identity
            or _integer(snapshot.target_epoch, 'snapshot epoch') != identity.epoch
            or ticket.target_epoch != identity.epoch):
        raise ValueError('current target identity or epoch changed')
    stamp = _integer(snapshot.stamp_ns, 'snapshot stamp', 1)
    if (ticket.snapshot_stamp_sec != snapshot.stamp_sec
            or abs(_scalar(snapshot.stamp_sec, 'snapshot seconds') - stamp * 1e-9) > 1e-6):
        raise ValueError('snapshot and ticket stamps disagree')
    stamps = tuple(_integer(value, 'sample stamp', 1) for value in snapshot.sample_stamp_ns)
    if not stamps or stamps[-1] != stamp or any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise ValueError('sample stamps must strictly increase to snapshot stamp')
    now = _integer(now_stamp_ns, 'current source stamp', 1)
    age_limit = _integer(max_source_age_ns, 'maximum source age', 1)
    if now < stamp or now - stamp > age_limit:
        raise ValueError('snapshot source is expired or in the future')
    base_frame = _frame(base_frame, 'base frame')
    _frame(snapshot.frame_id, 'camera source frame')
    if snapshot.source_mode != 'instance_mask':
        raise ValueError('only the unchanged instance_mask geometry inputs are supported')
    original = retained_exact_source(snapshot)
    if original is None:
        raise ValueError('original synchronized source sample is required')
    joints = np.asarray(original.joint_positions)
    if (original.bbox != snapshot.bbox or joints.shape != (6,)
            or not np.all(np.isfinite(joints))):
        raise ValueError('original source bbox or six-joint evidence is invalid')
    _detection(snapshot, original, base_frame)
    depth, target = np.asarray(snapshot.depth_raw), np.asarray(snapshot.target_depth_raw)
    mask, color = np.asarray(snapshot.object_mask), np.asarray(snapshot.color_bgr)
    if (depth.ndim != 2 or target.shape != depth.shape or mask.shape != depth.shape
            or color.shape != depth.shape + (3,) or color.dtype != np.uint8
            or np.asarray(original.depth_raw).shape != depth.shape
            or np.asarray(original.object_mask).shape != depth.shape
            or np.asarray(original.object_mask).dtype != np.uint8
            or not np.all(np.isin(original.object_mask, (0, 255)))
            or mask.dtype != np.uint8 or not np.all(np.isin(mask, (0, 255)))
            or not np.any(mask) or not np.any(np.asarray(original.object_mask) > 0)):
        raise ValueError('RGB-D instance mask/source shapes or values are invalid')
    if (not np.all(np.isfinite(depth)) or not np.all(np.isfinite(target))
            or np.any(depth < 0.) or np.any(target < 0.)
            or np.any((target > 0.) & (mask == 0))
            or not np.array_equal(target[target > 0.], depth[target > 0.])):
        raise ValueError('target depth must contain only measured masked snapshot values')
    quality = snapshot.quality
    count, area = int(np.count_nonzero(target)), int(np.count_nonzero(mask))
    if (type(quality) is not DepthQuality
            or _integer(quality.fused_frames, 'fused frame count', 1) != len(stamps)
            or _integer(quality.mask_area, 'mask area', 1) != area
            or _integer(quality.valid_depth_points, 'valid depth count', 1) != count
            or quality.valid_depth_ratio != count / float(area)):
        raise ValueError('snapshot sample/depth quality accounting disagrees')
    for name in ('depth_median_m', 'depth_mad_m', 'depth_repeatability_m'):
        if _scalar(getattr(quality, name), name) < 0.:
            raise ValueError('snapshot depth quality is negative')
    if (type(geometry) is not GeometryEstimate or geometry.ok is not False
            or geometry.failure_code != 'OBB_INVALID' or geometry.source_mode != snapshot.source_mode
            or type(geometry.observation_deficit) is not ObservationGeometryDeficit
            or np.any(geometry.size_xyz_m) or len(geometry.object_points_base)):
        raise ValueError('contact geometry must remain the original failed estimate')
    if type(geometry_config) is not ObservationGeometryConfig:
        raise ValueError('original geometry configuration is required')
    deficit = geometry.observation_deficit
    config = {item.name: getattr(geometry_config, item.name) for item in fields(geometry_config)}
    for name, value in config.items():
        if hasattr(deficit, name) and value != getattr(deficit, name):
            raise ValueError('original geometry configuration changed: ' + name)
    transform = _exact_base_optical(exact_tf, stamp, snapshot.frame_id, base_frame,
                                    source_from_optical)
    if not np.array_equal(transform, deficit.T_base_camera):
        raise ValueError('exact TF differs from failed geometry transform')
    scale = _scalar(depth_scale, 'depth scale', positive=True)
    # Reproduce the original estimator with the original thresholds. No relaxed
    # refit and no fake ok=True geometry are used to borrow the contact adapter.
    checked = estimate_object_geometry(depth_raw=depth, target_depth_raw=target,
        object_mask=mask, bbox=snapshot.bbox, intrinsics=intrinsics, depth_scale=scale,
        T_base_camera=transform, source_mode=snapshot.source_mode,
        previous_axes_base=deficit.axes_base, **config)
    repeated = checked.observation_deficit
    if checked.ok or repeated is None:
        raise ValueError('snapshot does not reproduce the same isolated geometry deficit')
    for item in fields(deficit):
        left, right = getattr(deficit, item.name), getattr(repeated, item.name)
        if not (np.array_equal(left, right) if isinstance(left, np.ndarray) else left == right):
            raise ValueError('snapshot geometry evidence differs: ' + item.name)
    x, y, width, height = snapshot.bbox
    image_height, image_width = depth.shape
    observation = TargetObservation(identity=identity, stamp_ns=stamp, frame_id=base_frame,
        points_base=deficit.object_points_base, support_normal_base=deficit.support_normal_base,
        support_offset_m=deficit.support_offset_m, bbox_xywh=snapshot.bbox,
        image_shape_hw=depth.shape,
        edge_clearance_px=min(x, y, image_width-x-width, image_height-y-height),
        source_kind=snapshot.source_mode, source_label=str(getattr(snapshot.object_msg, 'label', '') or ''))
    digest = hashlib.sha256(json.dumps(dict(request_id=ticket.request_id,
        request_generation=ticket.generation, identity=(identity.epoch, identity.track_id),
        source_expires_at_ns=stamp+age_limit,
        sample_stamp_ns=stamps, frame_id=snapshot.frame_id, base_frame=base_frame,
        config=config, depth_scale=scale, bbox=snapshot.bbox,
        intrinsics={name: float(getattr(intrinsics, name)) for name in
                    ('width', 'height', 'fx', 'fy', 'cx', 'cy')}), sort_keys=True).encode())
    for array in (color, depth, target, mask, original.depth_raw,
                  original.object_mask, original.joint_positions, transform):
        array = np.asarray(array)
        digest.update(str((array.shape, array.dtype.str)).encode())
        digest.update(array.tobytes())
    return PrecontactObservationSource(ticket.request_id, ticket.generation, observation,
        deficit, geometry_config, stamps, snapshot.frame_id, stamp, now, stamp+age_limit,
        transform, digest.hexdigest())
