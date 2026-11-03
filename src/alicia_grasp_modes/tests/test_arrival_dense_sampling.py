"""Pure deterministic sampling tests; no external recordings or ROS required."""
import cv2
import numpy as np

from alicia_grasp_modes import arrival_dense_identity as dense


def test_reduced_h_cannot_count_same_current_pixel_in_multiple_test_cells():
    yy, xx = np.indices((96, 96))
    xy = np.c_[xx.ravel(), yy.ravel()]
    cells = xy // 8
    test = cells.sum(axis=1) % 2 == 1
    mapped = np.rint(xy * .45).astype(int)
    current_flat = mapped[:, 1] * 96 + mapped[:, 0]
    kept, evidence = dense._unique_test_pixels(test, cells, current_flat)
    assert evidence['test_duplicate_pairs_removed'] > 0
    assert evidence['current_pixels_shared_by_test_cells_before_dedup'] > 0
    assert evidence['current_pixels_shared_by_test_cells_after_dedup'] == 0
    assert kept.sum() == len(np.unique(current_flat[kept]))
    assert not np.any(kept & ~test)
    # First source-order choice: pixel values play no part.
    for target in np.unique(current_flat[test]):
        assert np.flatnonzero(kept & (current_flat == target)).tolist() == [np.flatnonzero(test & (current_flat == target))[0]]


def test_heavily_reduced_real_pixel_mapping_cannot_inflate_texture_support():
    shape = (160, 160)
    mask = np.zeros(shape, np.uint8)
    mask[16:144, 16:144] = 1
    yy, xx = np.indices(shape)
    gray = np.where((xx//3 + yy//3) % 4 == 0, 20, 240).astype(np.uint8)
    image = np.repeat(gray[:, :, None], 3, axis=2)
    valid = np.ones(shape, bool)
    reference = dense.freeze_reference(image, mask, valid, 100)
    H = np.array([[.45, 0., 20.], [0., .45, 20.], [0., 0., 1.]])
    current_mask = cv2.warpPerspective(mask, H, shape[::-1], flags=cv2.INTER_NEAREST)
    current = cv2.warpPerspective((image.astype(float)*.6).astype(np.uint8), H, shape[::-1], flags=cv2.INTER_NEAREST)
    proposal = dense.propose_homography(reference, 101, H)
    result = dense.evaluate_appearance(reference, current, current_mask, valid, 101, proposal,
                                       deadline_monotonic=1., clock=lambda: 0.)
    assert result.fallback_attempted
    assert result.metrics['test_duplicate_pairs_removed'] > 0
    assert result.metrics['test_pairs'] == result.metrics['unique_current_test_pixels']
    assert result.metrics['current_pixels_shared_by_test_cells_after_dedup'] == 0
    assert not result.appearance_match
