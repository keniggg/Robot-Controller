"""No devices or ROS: actual SDK numerical functions and source pixel evidence."""
import copy
import base64
import importlib
from pathlib import Path
import sys
import zlib

import numpy as np
import pytest

from alicia_flexible_grasp.vision.rgbd_rectification import (
    PairedPinholeRectification, source_cell_ray_intervals,
    validate_rectification_contract)


REAL = dict(width=640, height=480, fx=395.7682800292969, fy=395.6027526855469,
    cx=317.9961242675781, cy=244.833984375, model='inverse_brown_conrady',
    coeffs=[-0.05372992530465126, 0.05834611505270004, 0.002122000325471163,
            0.0007625872385688126, -0.019704923033714294])


def identity_info():
    return dict(width=8, height=6, fx=40., fy=41., cx=3.5, cy=2.5,
                model='none', coeffs=[0.]*5)


@pytest.fixture(scope='module')
def sdk():
    work = Path(__file__).resolve().parents[3]
    isolated = work/'.runtime/realsense-2.56.5'
    if isolated.is_dir():
        sys.path.insert(0, str(isolated))
    try:
        module = importlib.import_module('pyrealsense2')
    except ImportError:
        pytest.skip('SDK numerical library unavailable; no device is required')
    return module


@pytest.fixture(scope='module')
def real(sdk):
    return PairedPinholeRectification(REAL, sdk)


def sdk_intrinsics(sdk, data):
    result = sdk.intrinsics()
    result.width, result.height = data['width'], data['height']
    result.fx, result.fy = data['fx'], data['fy']
    result.ppx, result.ppy = data['cx'], data['cy']
    result.model = getattr(sdk.distortion, data['model'])
    result.coeffs = data['coeffs']
    return result


def test_identity_preserves_pair_exactly_and_marks_only_current_sources():
    mapper = PairedPinholeRectification(identity_info())
    color = np.arange(144, dtype=np.uint8).reshape(6, 8, 3)
    depth = np.arange(48, dtype=np.uint16).reshape(6, 8)
    image, result, origin = mapper.apply(color, depth, return_source_indices=True)
    assert np.array_equal(image, color) and np.array_equal(result, depth)
    assert origin[0, 0] == -1
    assert np.array_equal(origin[depth > 0], np.flatnonzero(depth))
    assert not np.shares_memory(result, depth)
    assert validate_rectification_contract(mapper.contract) == (0., 0.)


@pytest.mark.parametrize('model', ['modified_brown_conrady', 'kannala_brandt4', 'ftheta', 'unknown'])
def test_unproved_models_fail_closed(model):
    with pytest.raises(ValueError, match='model_unproved'):
        PairedPinholeRectification(dict(REAL, model=model))


@pytest.mark.parametrize('field,value', [
    ('width', True), ('height', 0), ('fx', float('nan')), ('fy', -1.),
    ('cx', True), ('fx', 395.123456789), ('coeffs', [0.]*4),
    ('coeffs', [False, 0., 0., 0., 0.]), ('coeffs', [np.inf, 0., 0., 0., 0.])])
def test_invalid_intrinsics_rejected(field, value):
    with pytest.raises(ValueError):
        PairedPinholeRectification(dict(REAL, **{field: value}))


def test_none_cannot_hide_nonzero_distortion():
    with pytest.raises(ValueError, match='none_model_has_distortion'):
        PairedPinholeRectification(dict(REAL, model='none'))


def test_nonzero_requires_sdk_not_pinhole_fallback():
    with pytest.raises(ValueError, match='sdk_numerical_functions_required'):
        PairedPinholeRectification(REAL)


def test_different_sdk_cannot_claim_reviewed_continuous_program():
    from types import SimpleNamespace
    with pytest.raises(ValueError, match='implementation_unverified'):
        PairedPinholeRectification(REAL, SimpleNamespace(__version__='2.55.1'))


@pytest.mark.parametrize('model', ['inverse_brown_conrady', 'brown_conrady'])
def test_continuous_cell_intervals_enclose_actual_sdk_interior_and_edges(sdk, model):
    data = dict(REAL, model=model)
    xl, xh, yl, yh = source_cell_ray_intervals(data)
    intrinsic = sdk_intrinsics(sdk, data)
    rng = np.random.RandomState(3104)
    pixels = np.c_[rng.randint(0, 640, 1800), rng.randint(0, 480, 1800)]
    pixels = np.vstack((pixels, [[0, 0], [639, 0], [0, 479], [639, 479], [318, 245]]))
    for offset in ([0., 0.], [-.5, -.5], [.5, .5], [.5, -.5], [-.5, .5], [.137, -.281]):
        for u, v in pixels:
            x, y, z = sdk.rs2_deproject_pixel_to_point(intrinsic, (np.array([u, v])+offset).tolist(), 1.)
            assert xl[v, u] <= x <= xh[v, u]
            assert yl[v, u] <= y <= yh[v, u]
            assert z == 1.


def test_actual_model_proof_includes_pixel_area_and_quantization(real):
    margin = validate_rectification_contract(real.contract, output_intrinsics=real.output_intrinsics)
    assert .5 < margin[0] < .7 and .5 < margin[1] < .7
    assert real.contract['unique_destination_pixel_count'] == real.contract['mapped_source_pixel_count'] == 640*480
    assert real.width > 640 and real.height > 480
    assert real.output_intrinsics['coeffs'] == [0.]*5
    assert real.source_intrinsics == REAL
    assert real.output_intrinsics['fx'] == REAL['fx']
    assert not real.contract['physical_camera_axes_changed']


def test_depth_output_only_copies_one_current_native_sample_and_keeps_holes(real):
    rng = np.random.RandomState(31)
    depth = rng.randint(0, 5000, (480, 640)).astype(np.uint16)
    depth[rng.rand(480, 640) < .7] = 0
    color = np.zeros((480, 640, 3), np.uint8)
    _, result, provenance = real.apply(color, depth, return_source_indices=True)
    valid = result > 0
    assert np.array_equal(result[valid], depth.ravel()[provenance[valid]])
    assert len(np.unique(provenance[valid])) == int(valid.sum())
    assert np.all(provenance[~valid] == -1)
    assert np.count_nonzero(result) == np.count_nonzero(depth)
    assert not np.count_nonzero(real.apply(color, np.zeros_like(depth))[1])


def test_filled_image_does_not_close_forward_mapping_holes(real):
    _, depth = real.apply(np.zeros((480, 640, 3), np.uint8), np.full((480, 640), 2000, np.uint16))
    assert np.count_nonzero(depth) == real.contract['unique_destination_pixel_count']
    assert set(np.unique(depth)) == {0, 2000}
    assert depth.size > 640*480


def test_forward_nearest_actual_source_cells_fit_expanded_output_beams(real):
    xl, xh, yl, yh = source_cell_ray_intervals(REAL)
    index = real.source_indices
    dest_v, dest_u = np.divmod(real.target_indices, real.width)
    mx, my = real.contract['source_cell_extra_margin_px']
    assert np.all(REAL['fx']*xl.ravel()[index]+real.output_intrinsics['cx'] >= dest_u-.5-mx)
    assert np.all(REAL['fx']*xh.ravel()[index]+real.output_intrinsics['cx'] <= dest_u+.5+mx)
    assert np.all(REAL['fy']*yl.ravel()[index]+real.output_intrinsics['cy'] >= dest_v-.5-my)
    assert np.all(REAL['fy']*yh.ravel()[index]+real.output_intrinsics['cy'] <= dest_v+.5+my)
    assert np.min(REAL['fx']*xl+real.output_intrinsics['cx']) >= -.5
    assert np.max(REAL['fx']*xh+real.output_intrinsics['cx']) <= real.width-.5
    assert np.min(REAL['fy']*yl+real.output_intrinsics['cy']) >= -.5
    assert np.max(REAL['fy']*yh+real.output_intrinsics['cy']) <= real.height-.5


def test_noninjective_model_cannot_drop_far_measured_occupancy(sdk):
    coefficients = [float(np.float32(.15)), 0., 0., 0., 0.]
    with pytest.raises(ValueError, match='noninjective_source_map_unproved_occupancy'):
        PairedPinholeRectification(dict(REAL, model='brown_conrady', coeffs=coefficients), sdk)


def test_canvas_keeps_original_border_samples_and_marks_unobserved_rgb(real):
    color = np.full((480, 640, 3), 123, np.uint8)
    depth = np.arange(480*640, dtype=np.uint32).reshape(480, 640) % 65000 + 1
    image, result, origin = real.apply(color, depth.astype(np.uint16), return_source_indices=True)
    valid = result > 0
    assert set(origin[valid]) == set(range(480*640))
    assert np.all(image[~real.rgb_valid] == 0)
    assert np.all(image[real.rgb_valid] == 123)
    assert np.all(result.ravel()[real.target_indices] == depth.ravel())
    # A genuine border depth sample is retained even if its destination centre
    # has no captured RGB interpolation support. Colour validity stays separate.
    assert np.any((result > 0) & ~real.rgb_valid)


@pytest.mark.parametrize('key,value', [
    ('proof', 'corner_sampling'), ('source_cell_extra_margin_px', [0., 0.]),
    ('depth_forward_map_sha256', '0'*64), ('source_intrinsics', dict(REAL, cx=300.))])
def test_contract_tampering_rejected(real, key, value):
    contract = copy.deepcopy(real.contract)
    contract[key] = value
    with pytest.raises(ValueError, match='contract_invalid'):
        validate_rectification_contract(contract)


def test_wrong_output_geometry_rejected(real):
    with pytest.raises(ValueError, match='output_geometry_mismatch'):
        validate_rectification_contract(real.contract, output_intrinsics=REAL)


@pytest.mark.parametrize('payload', [b'bad-zlib', zlib.compress(b'\x00'*300000), zlib.compress(b'\x00')])
def test_rgb_valid_bitmap_decoding_is_exact_bounded_and_hash_bound(real, payload):
    from alicia_flexible_grasp.vision.rgbd_rectification import _digest
    value = copy.deepcopy(real.contract)
    value['rgb_valid_mask_zlib_base64'] = base64.b64encode(payload).decode('ascii')
    value.pop('digest')
    value['digest'] = _digest(value)
    with pytest.raises(ValueError, match='rgb_valid_encoding_invalid'):
        validate_rectification_contract(value)


@pytest.mark.parametrize('color,depth', [
    (np.zeros((6, 8, 3), float), np.zeros((6, 8), np.uint16)),
    (np.zeros((6, 8, 3), np.uint8), np.zeros((6, 8), np.float32)),
    (np.zeros((6, 8, 3), np.uint8), np.zeros((5, 8), np.uint16))])
def test_invalid_pair_rejected(color, depth):
    with pytest.raises(ValueError):
        PairedPinholeRectification(identity_info()).apply(color, depth)


def test_interval_singularity_does_not_produce_a_finite_fake_margin():
    data = dict(width=2, height=2, fx=1., fy=1., cx=0., cy=0.,
                model='inverse_brown_conrady', coeffs=[-1., 0., 0., 0., 0.])
    with pytest.raises(ValueError, match='interval_divisor_contains_zero'):
        source_cell_ray_intervals(data)
