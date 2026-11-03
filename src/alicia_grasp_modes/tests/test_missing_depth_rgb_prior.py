"""Missing depth is not measured background; current RGB still needs a seed.

Independent finite planar silhouettes vary size, pose, colour and sensor holes.
The actual OpenCV segmentation and measured-depth seed generation run unchanged.
"""
import cv2
import numpy as np
import pytest

from alicia_grasp_modes.rgb_instance import instance_mask_from_seed
from alicia_grasp_modes.tabletop import Config, _project


def scene(size=(80, 56), angle=0., colours=((15, 60, 20), (75, 150, 80)), missing=0.):
    cv2.setNumThreads(1)
    shape = (480, 640)
    yy, xx = np.indices(shape)
    c, s = np.cos(angle), np.sin(angle)
    u, v = c*(xx-320)+s*(yy-240), -s*(xx-320)+c*(yy-240)
    truth = (np.abs(u) < size[0]/2) & (np.abs(v) < size[1]/2)
    holes = truth & (u < -2)
    rgb = np.full(shape+(3,), (70, 100, 185), np.uint8)
    rgb[truth & ~holes] = colours[1]
    rgb[holes] = colours[1]
    rgb[truth & (u < -14)] = colours[0]
    depth = np.full(shape, .4, np.float32)
    depth[truth] = .35
    depth[holes] = missing
    seed = (truth & ~holes).astype(np.uint8)*255
    # This observed background hole is not part of any target or its seed.
    table_hole = (np.abs(u) < 12) & (v > size[1]/2+5) & (v < size[1]/2+16)
    depth[table_hole] = missing
    return dict(rgb=rgb, depth=depth, seed=seed, truth=truth, holes=holes,
                table_hole=table_hole, config=Config(), intrinsics=[600.,600.,320.,240.])


def refine(data, **kwargs):
    points = _project(data['depth'], data['intrinsics'])
    geometry = (points, data['depth'], .4-points[..., 0], np.array([-1.,0.,0.]), data['config'])
    return instance_mask_from_seed(data['rgb'], data['seed'], geometry=geometry,
        observation_arrival_retry=True, **kwargs)


@pytest.mark.parametrize('size,angle,colours', [
    ((80,56), 0., ((15,60,20),(75,150,80))),
    ((66,48), .35, ((14,14,14),(120,130,145))),
    ((72,60), -.45, ((70,20,10),(150,90,70))),
    ((90,48), .75, ((25,25,80),(60,90,190))),
])
@pytest.mark.parametrize('missing', [0., np.nan], ids=['zero','nan'])
def test_missing_coloured_face_preserves_outline_without_filling_depth(size, angle, colours, missing):
    data = scene(size, angle, colours, missing)
    depth, rgb, seed = data['depth'].copy(), data['rgb'].copy(), data['seed'].copy()
    mask, evidence = refine(data)
    assert np.mean(mask[data['holes']] > 0) >= .97
    assert np.mean(mask[data['truth']] > 0) >= .97
    assert np.count_nonzero((mask > 0) & ~data['truth']) <= 3
    assert not np.any(mask[data['table_hole']])
    assert evidence['trusted_core_retained_fraction'] >= .85
    assert evidence['measured_support_background_pixels'] > 0
    assert evidence['rgb_to_depth_seed_area_ratio'] <= 4.
    np.testing.assert_array_equal(data['depth'], depth)
    np.testing.assert_array_equal(data['rgb'], rgb)
    np.testing.assert_array_equal(data['seed'], seed)


@pytest.mark.parametrize('missing', [0., np.nan], ids=['zero','nan'])
def test_disconnected_same_colour_neighbor_is_not_owned_by_target(missing):
    data = scene(missing=missing)
    neighbor = np.zeros(data['truth'].shape, bool)
    neighbor[210:226, 371:382] = True
    assert not np.any(neighbor & data['truth'])
    data['rgb'][neighbor] = (15,60,20)
    data['depth'][neighbor] = missing
    before = data['depth'].copy()
    mask, _ = refine(data)
    assert np.mean(mask[data['truth']] > 0) >= .97
    assert not np.any(mask[neighbor])
    np.testing.assert_array_equal(data['depth'], before)


@pytest.mark.parametrize('missing', [0., np.nan], ids=['zero','nan'])
def test_missing_all_measured_foreground_cannot_create_colour_identity(missing):
    data = scene(missing=missing)
    data['depth'][data['truth']] = missing
    data['seed'][:] = 0
    before = data['depth'].copy()
    with pytest.raises(ValueError, match='insufficient_measured_foreground'):
        refine(data)
    np.testing.assert_array_equal(data['depth'], before)


@pytest.mark.parametrize('region', ['measured_seed','missing_face','table'])
def test_unknown_depth_does_not_authorize_unobserved_rgb(monkeypatch, region):
    data = scene()
    valid = np.ones(data['depth'].shape, bool)
    region_mask = {'measured_seed': data['seed'] > 0, 'missing_face': data['holes'],
                   'table': data['table_hole']}[region]
    row, col = np.argwhere(region_mask)[len(np.argwhere(region_mask))//2]
    valid[row,col] = False
    before = data['depth'].copy()
    monkeypatch.setattr(cv2, 'grabCut', lambda *a, **kw: pytest.fail('unobserved RGB reached GrabCut'))
    with pytest.raises(ValueError, match='contains_unobserved_rgb'):
        refine(data, rgb_valid=valid)
    np.testing.assert_array_equal(data['depth'], before)


@pytest.mark.parametrize('invalid', [-.1, np.inf, -np.inf, .001, 3.])
def test_nonmissing_invalid_depth_is_not_promoted_to_colour_foreground(monkeypatch, invalid):
    data = scene(missing=invalid)
    before = data['depth'].copy()
    labels_seen = []
    original = cv2.grabCut
    def observe(rgb, labels, *args):
        dark = np.all(rgb == (15,60,20), axis=-1)
        labels_seen.extend(labels[dark].tolist())
        return original(rgb, labels, *args)
    monkeypatch.setattr(cv2, 'grabCut', observe)
    _, evidence = refine(data)
    # Invalid ranges and infinities retain their original prior. The actual
    # colour model remains free to classify the observed pixel afterward.
    assert labels_seen and set(labels_seen) == {cv2.GC_PR_BGD}
    assert evidence['rgb_missing_depth_probable_foreground_pixels'] == 0
    np.testing.assert_array_equal(data['depth'], before)


def test_same_colour_measured_table_does_not_become_object():
    data = scene()
    data['rgb'][data['table_hole']] = (15,60,20)
    data['depth'][data['table_hole']] = .4
    mask, _ = refine(data)
    assert np.mean(mask[data['truth']] > 0) >= .97
    assert not np.any(mask[data['table_hole']])
