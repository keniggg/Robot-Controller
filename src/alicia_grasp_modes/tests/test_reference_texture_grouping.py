"""Retain the pre-optimization texture decisions and exact reduction inputs."""
import cv2
import numpy as np
import pytest

from alicia_grasp_modes import arrival_dense_identity as dense


def legacy_samples(image, mask, valid):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY).astype(float)
    x, y, _, _ = cv2.boundingRect(mask)
    inner = (cv2.erode(mask, np.ones((7, 7), np.uint8)) > 0) & valid
    yy, xx = np.nonzero(inner)
    cells = np.c_[(xx - x) // 8, (yy - y) // 8]
    samples = []
    selected = []
    for key in sorted(set(map(tuple, cells))):
        take = np.all(cells == key, axis=1)
        values = gray[yy[take], xx[take]]
        if len(values) >= 25:
            samples.append(values)
            if float(np.std(values)) >= 10.:
                selected.append(tuple(map(int, key)))
    return tuple(selected), samples


@pytest.mark.parametrize('seed', range(8))
def test_grouping_preserves_exact_legacy_std_input_order(monkeypatch, seed):
    rng = np.random.default_rng(seed)
    image = rng.integers(0, 256, (97, 123, 3), dtype=np.uint8)
    mask = np.zeros(image.shape[:2], np.uint8)
    cv2.ellipse(mask, (52, 42), (42, 34), 23, 0, 360, 1, -1)
    mask[24:36, 30:42] = 0
    mask[75:92, 101:116] = 1
    # An independent island and missing pixels exercise partial/small cells.
    mask[rng.random(mask.shape) < .007] = 0
    valid = np.ones(mask.shape, bool)
    expected, old_samples = legacy_samples(image, mask, valid)
    original_std = np.std
    seen = []

    def record(values, *args, **kwargs):
        seen.append(np.array(values, copy=True))
        return original_std(values, *args, **kwargs)

    monkeypatch.setattr(dense.np, 'std', record)
    actual = dense.freeze_reference(image, mask, valid, 123)
    assert actual.texture_cells == expected
    assert len(seen) == len(old_samples)
    for new, old in zip(seen, old_samples):
        assert np.array_equal(new, old)
    assert np.array_equal(actual.bgr, image)
    assert not actual.bgr.flags.writeable


@pytest.mark.parametrize('difference, accepted', [(19, False), (20, True), (21, True)])
def test_original_texture_threshold_boundary(difference, accepted):
    gray = np.tile(np.array([90, 90 + difference], np.uint8), (32, 16))
    image = np.repeat(gray[:, :, None], 3, axis=2)
    mask = np.ones(gray.shape, np.uint8)
    reference = dense.freeze_reference(image, mask, mask.astype(bool), 123)
    assert len(reference.texture_cells) == (16 if accepted else 0)
    expected, _ = legacy_samples(image, mask, mask.astype(bool))
    assert reference.texture_cells == expected


def test_empty_eroded_support_has_no_texture_cells():
    image = np.zeros((20, 20, 3), np.uint8)
    mask = np.zeros((20, 20), np.uint8)
    mask[8:11, 8:11] = 1
    reference = dense.freeze_reference(image, mask, np.ones(mask.shape, bool), 123)
    assert reference.texture_cells == ()
