"""Central pair geometry, recoverable stream dimensions and conservative beams."""
import copy
from dataclasses import replace
import hashlib
import json
import xmlrpc.client
from types import SimpleNamespace as NS
from unittest.mock import Mock

import numpy as np
import pytest

from test_rgbd_rectification import REAL, sdk, real, sdk_intrinsics
import test_camera_node_recovery as camera_helpers

from alicia_flexible_grasp.vision.realsense_manager import RealSenseManager
from alicia_flexible_grasp.vision.rgbd_rectification import (
    rectification_rgb_valid_mask, require_rectified_source_after_activation)
from alicia_grasp_modes.rgb_observation_source import camera_geometry_snapshot, digest
from alicia_grasp_modes.rgb_observation_volume import RGBObservationVolume, SupportPlaneProvenance
from alicia_grasp_modes.rgb_noncontact_scene import (
    build_rgb_noncontact_scene, retain_observed_obstacles, _validate_new_depth_holes,
    RgbNoncontactError)


def geometry(mapper):
    return dict(runtime_profile=dict(geometry_intrinsics=mapper.output_intrinsics,
        geometry_intrinsics_source='sdk_color_rectified_pinhole_canvas',
        color_intrinsics=REAL, rectification=mapper.contract))


def make_scene(mapper, depth, stamp=1000000000, camera=None):
    output = mapper.output_intrinsics
    matrix = np.array([[output['fx'], 0., output['cx']], [0., output['fy'], output['cy']], [0., 0., 1.]])
    transform = np.array([[0., -1., 0., 0.], [0., 0., 1., 0.], [-1., 0., 0., .6], [0., 0., 0., 1.]])
    plane = (0., 0., 1., 0.)
    volume = RGBObservationVolume(bbox_xywh=(300, 230, 20, 20),
        image_shape=(mapper.height, mapper.width), camera_matrix=matrix, T_base_camera=transform,
        support_plane_base=plane, support_provenance=SupportPlaneProvenance(stamp, plane, 'source', 200, .9, .001, 4),
        foreground_max_m=.12, source_stamp_ns=stamp, working_depth_range_m=(.07, .8))
    return build_rgb_noncontact_scene(interest_volume=volume, depth_image=depth, depth_scale_m=.0001,
        source_stamp_ns=stamp, source_depth_sha256=hashlib.sha256(depth.tobytes()).hexdigest(),
        depth_uncertainty_m=.002, valid_depth_range_m=(.03, 2.), source_record_id='source',
        camera_geometry=geometry(mapper) if camera is None else camera)


def test_disabled_geometry_snapshot_remains_identical_and_ignores_filter_status():
    camera = dict(fx=400., runtime_profile=dict(geometry_intrinsics={'fx': 400.}))
    original = camera_geometry_snapshot(camera)
    camera['runtime_profile']['rectification'] = None
    assert original == camera_geometry_snapshot(camera)
    assert rectification_rgb_valid_mask(camera, (480, 640)) is None


def test_geometry_snapshot_binds_complete_mapping_and_rgb_validity(real):
    camera = geometry(real)
    snapshot = camera_geometry_snapshot(camera)
    mask = rectification_rgb_valid_mask(snapshot, (real.height, real.width))
    assert np.array_equal(mask, real.rgb_valid) and not mask.flags.writeable
    assert snapshot['runtime_profile_geometry']['rectification'] == real.contract
    old = digest(snapshot)
    camera['runtime_profile']['rectification'] = copy.deepcopy(real.contract)
    camera['runtime_profile']['rectification']['source_cell_extra_margin_px'][0] = 0.
    with pytest.raises(ValueError, match='contract_invalid'):
        camera_geometry_snapshot(camera)
    assert digest(snapshot) == old
    with pytest.raises(ValueError, match='shape_mismatch'):
        rectification_rgb_valid_mask(snapshot, (480, 640))


def test_manager_rejects_legacy_projection_composition_and_unaligned_input():
    manager = RealSenseManager(pair_rectification_cfg={'enabled': True})
    manager.color_projection_correction = object()
    with pytest.raises(ValueError, match='legacy color projection'):
        manager._configure_pair_rectification(None)
    manager.color_projection_correction = None
    manager.align_depth_to_color = False
    with pytest.raises(ValueError, match='depth-to-color'):
        manager._configure_pair_rectification(None)
    with pytest.raises(RuntimeError, match='recorded real SDK'):
        RealSenseManager(simulate=True, pair_rectification_cfg={'enabled': True}).start()


def test_camera_recovery_reuses_native_stream_not_published_canvas(real):
    module = camera_helpers.load_camera_node()
    fake = camera_helpers.FakeRospy(dict(width=640, height=480, pair_rectification={'enabled': True}))
    module.rospy = fake
    fake.Time = NS(now=lambda:NS(to_nsec=lambda:1000000000))
    node = module.CameraNode.__new__(module.CameraNode)
    node.cfg = fake.camera_cfg
    node.width, node.height = 640, 480
    node.stream_width, node.stream_height = 640, 480
    node.fps, node.align_depth_to_color = 30, True
    node._async_publication_enabled = False
    profile = geometry(real)['runtime_profile']
    profile.update(source='realsense_active_profile')
    node.cam = NS(simulate=False, depth_scale=.0001, runtime_profile=profile)
    node._publish_runtime_camera_params_locked()
    assert (node.width, node.height) == (real.width, real.height)
    assert (node.cfg['stream_width'], node.cfg['stream_height']) == (640, 480)
    assert node.cfg['model'] == 'none' and node.cfg['coeffs'] == [0.]*5
    module.RealSenseManager = Mock()
    node._make_camera(False)
    call = module.RealSenseManager.call_args.kwargs
    assert (call['width'], call['height']) == (640, 480)
    assert call['pair_rectification_cfg'] == {'enabled': True}
    # A fresh process sees the retained stream fields, not a 653x489 SDK request.
    assert node.cfg['stream_width'] != node.cfg['width']
    before = camera_geometry_snapshot(node.cfg)
    require_rectified_source_after_activation(before, 1000000001)
    with pytest.raises(ValueError, match='predates_geometry_activation'):
        require_rectified_source_after_activation(before, 1000000000)
    # Same image dimensions and same K do not allow a queued pre-recovery frame.
    fake.Time = NS(now=lambda:NS(to_nsec=lambda:2000000000))
    node.cam = NS(simulate=False, depth_scale=.0001, runtime_profile=copy.deepcopy(profile))
    node._publish_runtime_camera_params_locked()
    after = camera_geometry_snapshot(node.cfg)
    assert digest(before) != digest(after)
    with pytest.raises(ValueError, match='predates_geometry_activation'):
        require_rectified_source_after_activation(after, 1500000000)
    # Filter telemetry/parameter republication does not renew the activation.
    fake.Time = NS(now=lambda:NS(to_nsec=lambda:3000000000))
    node._publish_runtime_camera_params_locked()
    assert camera_geometry_snapshot(node.cfg) == after
    # Explicit disable returns to native dimensions, retaining a new cutover.
    fake.camera_cfg['pair_rectification'] = {'enabled': False}
    raw_profile = dict(source='realsense_active_profile', color_intrinsics=REAL,
        geometry_intrinsics=REAL, geometry_intrinsics_source='sdk_active_color_profile')
    node.cam = NS(simulate=False, depth_scale=.0001, runtime_profile=raw_profile)
    node._publish_runtime_camera_params_locked()
    raw = camera_geometry_snapshot(node.cfg)
    assert (node.width, node.height) == (640, 480)
    assert rectification_rgb_valid_mask(raw, (480, 640)) is None
    with pytest.raises(ValueError, match='predates_geometry_activation'):
        require_rectified_source_after_activation(raw, 2999999999)
    require_rectified_source_after_activation(raw, 3000000001)
    assert node.cfg['runtime_profile']['geometry_activation_stamp_ns'] == '3000000000'
    # Exercise the real ROS parameter wire limitation, which fake rospy misses.
    xmlrpc.client.dumps((node.cfg,), allow_none=True)


@pytest.mark.parametrize('stamp', ['01', '+1', '1.0', '', '1'*20, True, 0])
def test_activation_nanoseconds_accept_only_exact_canonical_values(stamp):
    with pytest.raises(ValueError, match='predates_geometry_activation'):
        require_rectified_source_after_activation(dict(runtime_profile=dict(geometry_activation_stamp_ns=stamp)), 10**18)


@pytest.mark.parametrize('kind', ['declared_source', 'enabled_config'])
def test_missing_contract_cannot_silently_remove_rgb_or_beam_proof(kind):
    camera = (dict(runtime_profile=dict(geometry_intrinsics_source='sdk_color_rectified_pinhole_canvas'))
              if kind == 'declared_source' else dict(pair_rectification={'enabled': True}, runtime_profile={}))
    with pytest.raises(ValueError, match='contract_missing'):
        rectification_rgb_valid_mask(camera, (480, 640))
    with pytest.raises(ValueError, match='contract_missing'):
        camera_geometry_snapshot(camera)


@pytest.mark.parametrize('profile', ['missing', None])
@pytest.mark.parametrize('declaration', [dict(pair_rectification={'enabled': True}),
                                      dict(intrinsics_source='sdk_color_rectified_pinhole_canvas')])
def test_snapshot_cannot_erase_missing_profile_active_declaration(profile, declaration):
    camera = copy.deepcopy(declaration)
    if profile != 'missing':
        camera['runtime_profile'] = profile
    with pytest.raises(ValueError):
        camera_geometry_snapshot(camera)


def test_manager_applies_pair_mapping_after_filters_and_native_range_clip():
    from test_realsense_read_diagnostics import fake_manager
    manager, calls, original_color, _, _ = fake_manager()
    manager.color_projection_correction = None
    returned = (np.ones((2, 3, 3), np.uint8), np.ones((2, 3), np.uint16))
    def apply(color, depth):
        assert calls[-1] == 'depth_array_and_clip'
        assert np.array_equal(color, original_color)
        assert np.array_equal(depth, [[0, 0, 300, 1000, 20000, 0]])
        calls.append('paired_rectification')
        return returned
    manager.pair_rectifier = NS(apply=apply)
    result = manager.read()
    assert all(a is b for a, b in zip(result, returned))
    assert calls[-1] == 'paired_rectification'
    assert manager.last_read_diagnostics['result'] == 'complete'
    assert 'pair_rectification' in manager.last_read_diagnostics['stages_sec']


def test_unmodified_legacy_source_needs_no_new_activation_clock():
    require_rectified_source_after_activation({}, 1)


def test_all_border_and_interior_source_cells_keep_depth_occupancy(real, sdk):
    raw = np.zeros((480, 640), np.uint16)
    pixels = [(0, 0), (639, 0), (0, 479), (639, 479), (1, 230), (330, 214), (318, 245)]
    for i, (u, v) in enumerate(pixels):
        raw[v, u] = 3000+100*i
    _, depth = real.apply(np.zeros((480, 640, 3), np.uint8), raw)
    scene = make_scene(real, depth)
    assert scene.valid_depth_pixel_count == len(pixels)
    assert scene.depth_uncertainty_m == .002
    assert scene.rectification_beam_margin_px == tuple(real.contract['source_cell_extra_margin_px'])
    sdk_k = sdk_intrinsics(sdk, REAL)
    columns = int(np.ceil(real.width/scene.pixel_block_size))
    boxes = dict(zip(scene.depth_block_ids, scene.depth_bounds))
    for u, v in pixels:
        dest_v, dest_u = divmod(int(real.target_indices[v*640+u]), real.width)
        identity = (dest_v//scene.pixel_block_size)*columns+dest_u//scene.pixel_block_size
        lo, hi = boxes[identity]
        for du, dv in ((-.5, -.5), (.5, -.5), (-.5, .5), (.5, .5), (.231, -.371)):
            for d in (raw[v, u]*.0001-.002, raw[v, u]*.0001+.002):
                optical = sdk.rs2_deproject_pixel_to_point(sdk_k, [u+du, v+dv], float(d))
                camera = np.array([optical[2], -optical[0], -optical[1]])
                point = scene.T_base_camera[:3, :3]@camera+scene.T_base_camera[:3, 3]
                assert np.all(point >= lo-1e-7) and np.all(point <= hi+1e-7)
    with pytest.raises(RgbNoncontactError, match='RECTIFICATION_BINDING'):
        replace(scene, rectification_beam_margin_px=(0., 0.))


def test_new_depth_hole_beam_uses_same_full_source_cell_margin(real):
    raw = np.zeros((480, 640), np.uint16)
    raw[0, 0], raw[245, 318] = 3000, 4000
    _, original = real.apply(np.zeros((480, 640, 3), np.uint8), raw)
    raw[0, 0] = 0
    _, current = real.apply(np.zeros((480, 640, 3), np.uint8), raw)
    previous, fresh = make_scene(real, original), make_scene(real, current, stamp=1100000000)
    captured = []
    assert _validate_new_depth_holes(previous, fresh, lambda box, points: captured.append(box) or True) == 1
    # With the same raster but unexpanded pixel cells the unknown beam is smaller.
    no_margin = object.__new__(type(fresh))
    no_margin.__dict__.update(fresh.__dict__)
    object.__setattr__(no_margin, 'rectification_beam_margin_px', (0., 0.))
    smaller = []
    _validate_new_depth_holes(previous, no_margin, lambda box, points: smaller.append(box) or True)
    assert np.all(captured[0][0] <= smaller[0][0])
    assert np.all(captured[0][1] >= smaller[0][1])
    assert np.any(captured[0] != smaller[0])


def test_scene_rejects_missing_or_changed_rectifier_binding_during_retention(real):
    _, depth = real.apply(np.zeros((480, 640, 3), np.uint8), np.full((480, 640), 4000, np.uint16))
    first = make_scene(real, depth)
    second = make_scene(real, depth, stamp=1100000000, camera={})
    with pytest.raises(RgbNoncontactError, match='RETAINED_SOURCE_MISMATCH'):
        retain_observed_obstacles(first, second)
