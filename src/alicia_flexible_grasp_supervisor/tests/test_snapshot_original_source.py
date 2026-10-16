"""Real-buffer reproduction of queued observation losing its admitted raw frame."""
from dataclasses import replace, FrozenInstanceError
import gc
import weakref
import numpy as np
import pytest
from test_rgbd_snapshot import sample, stable_mask, fuse, add_complete_sample
from alicia_flexible_grasp.vision.rgbd_snapshot import (
    SynchronizedRgbdBuffer, FrozenRgbdSample, retained_exact_source)
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity


def test_original_sample_survives_completion_age_and_buffer_eviction():
    now = [100., 10_000_000_000]
    buffer = SynchronizedRgbdBuffer(source_clock_ns=lambda:now[1], monotonic_clock=lambda:now[0])
    add_complete_sample(buffer, 10.)
    identity = TargetTrackIdentity.from_stream(0, 0)
    raw = buffer.wait_for_exact_sample(10_000_000_000, identity, 0., True, .35, 1.2)
    assert raw is not None
    snapshot = fuse([raw], erosion_px=0)
    assert snapshot.ok
    now[:] = [100.5, 10_500_000_000]
    assert buffer.wait_for_exact_sample(raw.stamp_ns, identity, 0., True, .35, 1.2) is None
    source = retained_exact_source(snapshot)
    assert source.stamp_ns == raw.stamp_ns
    assert np.array_equal(source.depth_raw, raw.depth_raw)
    now[:] = [103., 13_000_000_000]
    add_complete_sample(buffer, 13.)
    assert raw.stamp_ns not in buffer._entries
    assert retained_exact_source(snapshot) is source
    assert source.stamp_ns != 13_000_000_000


def raw_samples():
    rows = [sample(value, stable_mask(), 10.+i*.1) for i,value in enumerate([1000,1010,3000])]
    for row in rows:row.stamp_ns = round(row.stamp_sec*1e9)
    return rows


def test_retains_last_raw_depth_and_mask_not_temporal_median_or_eroded_mask():
    rows = raw_samples()
    snapshot = fuse(rows)
    raw = retained_exact_source(snapshot)
    assert np.all(raw.depth_raw == 3000)
    assert np.all(snapshot.depth_raw == 1010)
    assert np.count_nonzero(raw.object_mask) > np.count_nonzero(snapshot.object_mask)
    for attr in ('color_bgr','depth_raw','object_mask','joint_positions'):
        original = getattr(rows[-1],attr)
        stored = getattr(raw,attr)
        before = stored.copy()
        original[...] = 0
        assert np.array_equal(stored,before)
        with pytest.raises(ValueError): stored.flat[0]=1
    with pytest.raises(FrozenInstanceError):raw.stamp_ns += 1


@pytest.mark.parametrize('fault',['stamp','epoch','identity','frame','rgb','lineage'])
def test_retained_source_binding_rejects_changed_snapshot(fault):
    snapshot = fuse(raw_samples())
    changes = {'stamp':dict(stamp_ns=snapshot.stamp_ns+1),
        'epoch':dict(target_epoch=1,target_identity=TargetTrackIdentity.from_stream(0,1)),
        'identity':dict(target_identity=TargetTrackIdentity.from_stream(1,0)),
        'frame':dict(frame_id='another_camera'),
        'rgb':dict(color_bgr=np.ones_like(snapshot.color_bgr)),
        'lineage':dict(sample_stamp_ns=(snapshot.stamp_ns+1,))}[fault]
    with pytest.raises(ValueError,match='does not match'):retained_exact_source(replace(snapshot,**changes))


def test_retained_source_has_no_global_owner_after_request_release():
    snapshot = fuse(raw_samples())
    source = retained_exact_source(snapshot)
    reference = weakref.ref(source)
    del snapshot, source
    gc.collect()
    assert reference() is None


def test_legacy_snapshot_without_raw_source_returns_none_not_fused_depth():
    snapshot = replace(fuse(raw_samples()),original_sample=None)
    assert retained_exact_source(snapshot) is None
