"""Validated perception boundary for class-agnostic grasp geometry.

Labels and source kinds are audit metadata. Only stream generation and geometric
association epoch create an identity; semantic metadata never participates.
"""

from dataclasses import dataclass
from numbers import Integral

import numpy as np


def _integer(value, name, minimum=0):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Integral):
        raise ValueError('%s must be an integer' % name)
    if value < minimum:
        raise ValueError('%s must be >= %d' % (name, minimum))
    return int(value)


def validate_track_id(value):
    if not isinstance(value, str) or not value or value != value.strip():
        raise ValueError('target_track_id must be a non-empty canonical string')
    if any(character.isspace() or ord(character) < 32 for character in value):
        raise ValueError('target_track_id must not contain whitespace/control characters')
    return value


@dataclass(frozen=True)
class TargetTrackIdentity:
    epoch: int
    track_id: str

    def __post_init__(self):
        object.__setattr__(self, 'epoch', _integer(self.epoch, 'epoch'))
        object.__setattr__(self, 'track_id', validate_track_id(self.track_id))

    @classmethod
    def from_stream(cls, generation, epoch):
        generation = _integer(generation, 'generation')
        epoch = _integer(epoch, 'epoch')
        return cls(epoch, 'g%d-t%d' % (generation, epoch))


def validate_target_identity(value):
    if not isinstance(value, TargetTrackIdentity):
        raise ValueError('target_identity must be a TargetTrackIdentity')
    return value


def _immutable_finite(value, shape, name):
    values = np.asarray(value, dtype=np.float64)
    if values.shape != shape or not np.all(np.isfinite(values)):
        raise ValueError('%s has an invalid shape or non-finite values' % name)
    # bytes-backed arrays cannot be made writable again by a downstream owner.
    return np.frombuffer(values.tobytes(), dtype=np.float64).reshape(shape)


@dataclass(frozen=True)
class TargetObservation:
    identity: TargetTrackIdentity
    stamp_ns: int
    frame_id: str
    points_base: np.ndarray
    support_normal_base: np.ndarray
    support_offset_m: float
    bbox_xywh: tuple
    image_shape_hw: tuple
    edge_clearance_px: int
    source_kind: str
    source_label: str = ''

    def __post_init__(self):
        validate_target_identity(self.identity)
        object.__setattr__(self, 'stamp_ns', _integer(self.stamp_ns, 'stamp_ns', 1))
        if not isinstance(self.frame_id, str) or not self.frame_id or self.frame_id != self.frame_id.strip():
            raise ValueError('frame_id must be a non-empty canonical frame')
        points = np.asarray(self.points_base, dtype=np.float64)
        if points.ndim != 2 or points.shape[1:] != (3,) or not len(points):
            raise ValueError('points_base must contain finite Nx3 measured points')
        object.__setattr__(self, 'points_base', _immutable_finite(points, points.shape, 'points_base'))
        normal = _immutable_finite(self.support_normal_base, (3,), 'support_normal_base')
        if not np.isclose(np.linalg.norm(normal), 1.0, rtol=0.0, atol=1e-6):
            raise ValueError('support_normal_base must be a unit normal')
        object.__setattr__(self, 'support_normal_base', normal)
        offset = float(self.support_offset_m)
        if not np.isfinite(offset):
            raise ValueError('support_offset_m must be finite')
        object.__setattr__(self, 'support_offset_m', offset)
        shape = tuple(_integer(value, 'image dimension', 1) for value in self.image_shape_hw)
        bbox = tuple(_integer(value, 'bbox component') for value in self.bbox_xywh)
        if len(shape) != 2 or len(bbox) != 4:
            raise ValueError('image_shape_hw/bbox_xywh have invalid dimensions')
        height, width = shape
        x, y, box_width, box_height = bbox
        if box_width <= 0 or box_height <= 0 or x + box_width > width or y + box_height > height:
            raise ValueError('bbox_xywh must fit the source image')
        clearance = _integer(self.edge_clearance_px, 'edge_clearance_px')
        if clearance != min(x, y, width - x - box_width, height - y - box_height):
            raise ValueError('edge_clearance_px is inconsistent with bbox/image shape')
        object.__setattr__(self, 'bbox_xywh', bbox)
        object.__setattr__(self, 'image_shape_hw', shape)
        object.__setattr__(self, 'edge_clearance_px', clearance)
        if not isinstance(self.source_kind, str) or not isinstance(self.source_label, str):
            raise ValueError('source diagnostics must be strings')


def observation_from_snapshot(snapshot, geometry, transform_base_optical, base_frame='base_link'):
    """Adapt only an aligned snapshot whose timestamped TF and support succeeded."""
    if not bool(getattr(snapshot, 'ok', False)) or not bool(getattr(geometry, 'ok', False)):
        raise ValueError('snapshot and geometry must be valid')
    identity = validate_target_identity(getattr(snapshot, 'target_identity', None))
    if _integer(getattr(snapshot, 'target_epoch', None), 'target_epoch') != identity.epoch:
        raise ValueError('snapshot target epoch does not match its identity')
    stamp = _integer(getattr(snapshot, 'stamp_ns', None), 'snapshot stamp', 1)
    stamp_sec = float(getattr(snapshot, 'stamp_sec', float('nan')))
    if not np.isfinite(stamp_sec) or abs(stamp_sec - stamp * 1e-9) > 1e-6:
        raise ValueError('snapshot stamps disagree')
    stamps = tuple(_integer(item, 'sample stamp', 1) for item in snapshot.sample_stamp_ns)
    if not stamps or stamps[-1] != stamp or any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise ValueError('sample stamps must increase and end at snapshot stamp')
    if not isinstance(snapshot.frame_id, str) or not snapshot.frame_id.strip():
        raise ValueError('snapshot source frame is missing')
    depth = np.asarray(snapshot.depth_raw)
    color = np.asarray(snapshot.color_bgr)
    if depth.ndim != 2 or color.shape != depth.shape + (3,):
        raise ValueError('RGB-D image shapes disagree')
    if any(np.asarray(getattr(snapshot, name)).shape != depth.shape for name in ('target_depth_raw', 'object_mask')):
        raise ValueError('target depth/mask shape disagrees with RGB-D')
    transform = _immutable_finite(transform_base_optical, (4, 4), 'snapshot TF')
    rotation = transform[:3, :3]
    if (not np.allclose(transform[3], [0, 0, 0, 1], rtol=0, atol=1e-6)
            or not np.allclose(rotation.T @ rotation, np.eye(3), rtol=0, atol=1e-6)
            or not np.isclose(np.linalg.det(rotation), 1., rtol=0, atol=1e-6)):
        raise ValueError('snapshot TF must be a finite rigid transform')
    x, y, width, height = snapshot.bbox
    image_height, image_width = depth.shape
    return TargetObservation(
        identity=identity, stamp_ns=stamp, frame_id=base_frame,
        points_base=geometry.object_points_base,
        support_normal_base=geometry.support_normal_base,
        support_offset_m=geometry.support_offset_m,
        bbox_xywh=snapshot.bbox, image_shape_hw=depth.shape,
        edge_clearance_px=min(x, y, image_width - x - width, image_height - y - height),
        source_kind=str(snapshot.source_mode),
        source_label=str(getattr(snapshot.object_msg, 'label', '') or ''),
    )
