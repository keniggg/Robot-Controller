"""Optional topology preparation cannot replace the original source gate."""
import json

import pytest

from alicia_flexible_grasp.vision.rgbd_rectification import PairedPinholeRectification
from alicia_grasp_modes.source_sampling_topology import SourceSamplingTopologyCache
from test_unknown_stream import rgb_interest_node


@pytest.mark.parametrize('previous_ready', [False, True])
def test_ordinary_identity_profile_acquires_but_missing_topology_proof_cannot_recover(
        monkeypatch, previous_ready):
    node, clock, feed, _, params = rgb_interest_node(monkeypatch)
    camera = params['/camera']
    mapper = PairedPinholeRectification(dict(camera, model='none', coeffs=[0.]*5))
    profile = camera['runtime_profile_geometry'] = dict(
        geometry_intrinsics=mapper.output_intrinsics,
        rectification=mapper.contract, geometry_activation_stamp_ns=1)
    cache = node._source_sampling_topology = SourceSamplingTopologyCache()
    if previous_ready:
        profile.update(color_intrinsics=mapper.source_intrinsics,
                       sdk_version=mapper.contract['sdk_version'])
        cache.offer(camera)
        worker = cache._thread
        if worker is not None:
            worker.join(2.)
            assert not worker.is_alive()
        cache.require(camera, (240, 320), [400., 400., 160., 120.], 100)
        del profile['color_intrinsics'], profile['sdk_version']
    for _ in range(3):
        feed(target_depth=.35)
        clock[0] += .2
    assert node.tracker.anchor is not None and not node.tracker.lost
    # A previous ready map must not let optional prewarm failure bypass the
    # strict recovery proof. This invokes the same require used by the node.
    with pytest.raises(ValueError, match='current_source_topology_profile_mismatch'):
        cache.require(camera, (240, 320), [400., 400., 160., 120.],
                      node.last_good_source_stamp_ns)


def test_original_rectification_source_gate_is_not_optional(monkeypatch):
    node, _, feed, _, params = rgb_interest_node(monkeypatch)
    camera = params['/camera']
    mapper = PairedPinholeRectification(dict(camera, model='none', coeffs=[0.]*5))
    camera['runtime_profile_geometry'] = dict(
        geometry_intrinsics=mapper.output_intrinsics,
        rectification=mapper.contract, geometry_activation_stamp_ns=11000000000)
    feed(target_depth=.35)
    assert node.tracker.anchor is None
    report = json.loads(node.pubs['detector_status'].publish.call_args[0][0].data)
    assert report['detail'] == (
        'perception_failure: rectification_source_predates_geometry_activation')
    assert getattr(node, '_source_sampling_topology', None) is None
