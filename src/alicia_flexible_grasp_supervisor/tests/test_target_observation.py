"""Opaque observation provenance is independent of a segmentation vocabulary."""

from dataclasses import FrozenInstanceError, replace
from types import SimpleNamespace

import numpy as np
import pytest

from alicia_flexible_grasp.vision.target_observation import (
    TargetObservation, TargetTrackIdentity, observation_from_snapshot,
)


def observation(label='carton', **changes):
    values = dict(
        identity=TargetTrackIdentity.from_stream(2, 3), stamp_ns=1_000_000_000,
        frame_id='base_link', points_base=np.array([[0.1, 0.2, 0.3]]),
        support_normal_base=np.array([0., 0., 1.]), support_offset_m=0.1,
        bbox_xywh=(2, 3, 4, 5), image_shape_hw=(12, 12),
        edge_clearance_px=2, source_kind='instance_mask', source_label=label,
    )
    values.update(changes)
    return TargetObservation(**values)


def test_labels_do_not_define_identity_or_observation_geometry():
    rows = [observation(label) for label in ('carton', 'bottle', '')]
    assert len({item.identity for item in rows}) == 1
    for item in rows:
        np.testing.assert_array_equal(item.points_base, rows[0].points_base)
    assert TargetTrackIdentity.from_stream(2, 3) != TargetTrackIdentity.from_stream(3, 3)
    assert TargetTrackIdentity.from_stream(2, 3) != TargetTrackIdentity.from_stream(2, 4)


def test_observation_owns_immutable_arrays_and_identity():
    points = np.array([[0.1, 0.2, 0.3]])
    item = observation(points_base=points)
    points[:] = 9.0
    assert item.points_base[0, 0] == 0.1
    with pytest.raises(ValueError):
        item.points_base.setflags(write=True)
    with pytest.raises(FrozenInstanceError):
        item.identity.epoch = 4


@pytest.mark.parametrize('changes', [
    {'identity': (3, 'carton', 'custom')}, {'stamp_ns': 0}, {'stamp_ns': True},
    {'frame_id': ''}, {'points_base': [[float('nan'), 0, 0]]},
    {'points_base': []}, {'support_normal_base': [0, 0, 0]},
    {'support_normal_base': [0, 0, 2]}, {'support_offset_m': float('inf')},
    {'bbox_xywh': (2, 3, 20, 5)}, {'edge_clearance_px': 3},
])
def test_invalid_observation_authority_fails_closed(changes):
    with pytest.raises(ValueError):
        observation(**changes)


def test_adapter_requires_aligned_snapshot_provenance_before_emitting_points():
    snapshot = SimpleNamespace(
        ok=True, target_identity=TargetTrackIdentity.from_stream(2, 3),
        target_epoch=3, stamp_ns=1_000_000_000, stamp_sec=1.,
        sample_stamp_ns=(900_000_000, 1_000_000_000), frame_id='camera_optical',
        color_bgr=np.zeros((12, 12, 3)), depth_raw=np.ones((12, 12)),
        target_depth_raw=np.ones((12, 12)), object_mask=np.ones((12, 12)),
        bbox=(2, 3, 4, 5), source_mode='instance_mask',
        object_msg=SimpleNamespace(label=''),
    )
    geometry = SimpleNamespace(ok=True, object_points_base=np.array([[.1, .2, .3]]),
        support_normal_base=np.array([0., 0., 1.]), support_offset_m=.1)
    result = observation_from_snapshot(snapshot, geometry, np.eye(4), 'base_link')
    assert result.identity == snapshot.target_identity
    assert result.source_label == ''
    snapshot.object_mask = np.ones((11, 12))
    with pytest.raises(ValueError, match='shape'):
        observation_from_snapshot(snapshot, geometry, np.eye(4), 'base_link')
    snapshot.object_mask = np.ones((12, 12))
    snapshot.sample_stamp_ns = (1_000_000_000, 900_000_000)
    with pytest.raises(ValueError, match='stamp'):
        observation_from_snapshot(snapshot, geometry, np.eye(4), 'base_link')
