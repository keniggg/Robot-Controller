#!/usr/bin/env python3
import pathlib
import sys
import unittest

import cv2
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'src'))
from alicia_flexible_grasp.vision.color_projection import ColorProjectionCorrection


def intrinsics(fx=398.4, fy=398.97, cx=317.78, cy=245.36):
    return dict(width=640, height=480, fx=fx, fy=fy, cx=cx, cy=cy,
                model='brown_conrady', coeffs=[-.054, .058, .0021, .00076, -.02])


def camera_matrix(data):
    return np.array([[data['fx'], 0, data['cx']],
                     [0, data['fy'], data['cy']], [0, 0, 1]], dtype=float)


class ColorProjectionTest(unittest.TestCase):
    def test_corrected_rgb_features_match_independently_projected_depth_rays(self):
        measured = intrinsics()
        sdk = intrinsics(438.6, 438.42, 320.1, 241.6)
        correction = ColorProjectionCorrection(measured, sdk)
        yy, xx = np.mgrid[:480, :640]
        # Distinct points, ranges, and image quadrants exercise the physical
        # projection rather than comparing the implementation's map formula.
        points = [(-.045, -.025, .2), (.08, -.035, .35),
                  (-.09, .07, .6), (.025, .055, .3)]
        for point in points:
            xyz = np.array([point], dtype=float)
            raw_pixel, _ = cv2.projectPoints(xyz, np.zeros(3), np.zeros(3),
                                            camera_matrix(measured), np.array(measured['coeffs']))
            depth_pixel, _ = cv2.projectPoints(xyz, np.zeros(3), np.zeros(3),
                                              camera_matrix(sdk), np.array(sdk['coeffs']))
            raw_pixel, depth_pixel = raw_pixel.ravel(), depth_pixel.ravel()
            spot = np.rint(240*np.exp(-((xx-raw_pixel[0])**2+(yy-raw_pixel[1])**2)/8)).astype(np.uint8)
            image = np.repeat(spot[:, :, None], 3, axis=2)
            corrected = correction.apply(image)[:, :, 0].astype(float)
            centroid = np.array([(corrected*xx).sum(), (corrected*yy).sum()])/corrected.sum()
            np.testing.assert_allclose(centroid, depth_pixel, atol=.06)
            self.assertGreater(np.linalg.norm(raw_pixel-depth_pixel), 3)

    def test_identity_preserves_rgb_without_mutating_source(self):
        config = intrinsics()
        image = np.random.RandomState(6).randint(0,256,(480,640,3)).astype(np.uint8)
        original = image.copy()
        result = ColorProjectionCorrection(config,config).apply(image)
        np.testing.assert_array_equal(result, original)
        np.testing.assert_array_equal(image, original)

    def test_rejects_different_lens_models_instead_of_using_affine_approximation(self):
        source, target = intrinsics(), intrinsics(438.6,438.42)
        target['coeffs'][0] += .001
        with self.assertRaisesRegex(ValueError, 'identical distortion'):
            ColorProjectionCorrection(source,target)

    def test_rejects_projection_that_would_invent_image_borders(self):
        with self.assertRaisesRegex(ValueError, 'outside the captured RGB'):
            ColorProjectionCorrection(intrinsics(480,480),intrinsics())

    def test_rejects_wrong_stream_size_at_construction_and_frame_input(self):
        source, target = intrinsics(), intrinsics(438.6,438.42)
        target['height'] = 720
        with self.assertRaisesRegex(ValueError, 'dimensions differ'):
            ColorProjectionCorrection(source,target)
        correction = ColorProjectionCorrection(source,intrinsics(438.6,438.42))
        with self.assertRaisesRegex(ValueError, 'matching uint8 BGR'):
            correction.apply(np.zeros((240,320,3),dtype=np.uint8))


if __name__ == '__main__':
    unittest.main()
