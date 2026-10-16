"""Sampling provenance and bounded background-cache contracts, without ROS."""
from copy import deepcopy
import threading

import numpy as np
import pytest

from alicia_grasp_modes.source_sampling_topology import (
    SourceSamplingTopology, SourceSamplingTopologyCache)
from alicia_flexible_grasp.vision.rgbd_rectification import PairedPinholeRectification


def identity_camera(shape=(20, 30), intrinsics=(400., 400., 15., 10.)):
    data = dict(width=shape[1], height=shape[0],
        **dict(zip(('fx', 'fy', 'cx', 'cy'), map(float, intrinsics))),
        model='none', coeffs=[0.]*5)
    mapper = PairedPinholeRectification(data)
    return dict(data, intrinsics_source='sdk_color_rectified_pinhole_canvas',
        pair_rectification={'enabled': True}, runtime_profile=dict(
            color_intrinsics=data, geometry_intrinsics=data,
            sdk_version=mapper.contract['sdk_version'],
            geometry_activation_stamp_ns='1', rectification=mapper.contract))


def test_exact_identity_map_preserves_values_and_has_immutable_indices():
    camera = identity_camera()
    topology = SourceSamplingTopology(camera)
    topology.require_source(camera, (20, 30), (400., 400., 15., 10.), 100)
    depth = np.arange(600).reshape(20, 30)
    np.testing.assert_array_equal(topology.source_array(depth), depth)
    mask = depth % 2 == 0
    np.testing.assert_array_equal(topology.output_mask(mask), mask)
    assert not topology.source_to_output.flags.writeable
    assert not topology.output_to_source.flags.writeable


@pytest.mark.parametrize('change', ['intrinsics', 'shape', 'activation', 'profile', 'map_digest'])
def test_current_profile_source_and_activation_are_rechecked(change):
    camera = identity_camera()
    topology = SourceSamplingTopology(camera)
    K, shape, stamp = [400., 400., 15., 10.], (20, 30), 100
    if change == 'intrinsics': K[0] += 1
    elif change == 'shape': shape = (30, 20)
    elif change == 'activation': camera['runtime_profile']['geometry_activation_stamp_ns'] = '100'
    elif change == 'profile': camera['runtime_profile']['color_intrinsics'] = dict(camera['runtime_profile']['color_intrinsics'], fx=401.)
    else: camera['runtime_profile']['rectification']['depth_forward_map_sha256'] = '0'*64
    with pytest.raises(ValueError):
        topology.require_source(camera, shape, K, stamp)


def test_missing_contract_does_not_fall_back_to_canvas_adjacency():
    with pytest.raises(ValueError, match='contract_required'):
        SourceSamplingTopology({})


def test_pending_profile_is_not_built_in_the_frame_thread_and_is_not_rebuilt():
    camera = identity_camera()
    entered, release = threading.Event(), threading.Event()
    calls = []
    caller = threading.get_ident()
    def build(value):
        calls.append(threading.get_ident())
        entered.set()
        assert release.wait(2)
        return SourceSamplingTopology(value)
    cache = SourceSamplingTopologyCache(build)
    cache.offer(camera)
    assert entered.wait(1)
    with pytest.raises(ValueError, match='topology_pending'):
        cache.require(camera, (20, 30), (400., 400., 15., 10.), 100)
    for _ in range(5): cache.offer(deepcopy(camera))
    thread = cache._thread
    release.set(); thread.join(2)
    assert not thread.is_alive() and len(calls) == 1 and calls[0] != caller
    first = cache.require(camera, (20, 30), (400., 400., 15., 10.), 100)
    cache.offer(camera)
    assert cache.require(camera, (20, 30), (400., 400., 15., 10.), 101) is first
    assert len(calls) == 1


def test_profile_replacement_cannot_publish_a_late_old_map():
    first, second = identity_camera(), identity_camera(intrinsics=(401., 400., 15., 10.))
    started = [threading.Event(), threading.Event()]
    release = [threading.Event(), threading.Event()]
    calls = []
    def build(value):
        index = len(calls); calls.append(value)
        started[index].set(); assert release[index].wait(2)
        return SourceSamplingTopology(value)
    cache = SourceSamplingTopologyCache(build)
    cache.offer(first); assert started[0].wait(1)
    worker = cache._thread
    cache.offer(second)
    assert cache._thread is worker
    with pytest.raises(ValueError, match='profile_not_prepared'):
        cache.require(first, (20, 30), (400., 400., 15., 10.), 100)
    release[0].set(); assert started[1].wait(1)
    with pytest.raises(ValueError, match='topology_pending'):
        cache.require(second, (20, 30), (401., 400., 15., 10.), 100)
    release[1].set(); worker.join(2)
    assert not worker.is_alive() and len(calls) == 2
    cache.require(second, (20, 30), (401., 400., 15., 10.), 100)


def test_failed_or_removed_profile_never_reuses_previous_ready_map():
    camera = identity_camera()
    def failed(value): raise ValueError('map_not_verified')
    cache = SourceSamplingTopologyCache(failed)
    cache.offer(camera)
    thread = cache._thread
    if thread is not None: thread.join(2)
    with pytest.raises(ValueError, match='preparation_failed'):
        cache.require(camera, (20, 30), (400., 400., 15., 10.), 100)
    cache.offer({})
    with pytest.raises(ValueError, match='profile_not_prepared'):
        cache.require(camera, (20, 30), (400., 400., 15., 10.), 100)
