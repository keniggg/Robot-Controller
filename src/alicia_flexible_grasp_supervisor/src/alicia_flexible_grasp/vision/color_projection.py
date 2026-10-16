"""Resample calibrated RGB into the projection used by SDK-aligned depth.

RealSense alignment projects native depth using the SDK's color intrinsics. If
independent calibration finds different focal lengths for the actual RGB image,
replacing the downstream pinhole K alone leaves RGB and aligned depth mismatched.
This correction instead resamples RGB into the SDK color projection, so callers
can keep using the SDK-aligned depth and its matching K.

This affine mapping is valid only when both calibrations use the same distortion
model and coefficients. It is not a replacement for general lens rectification.
"""

import cv2
import numpy as np


def _validate_intrinsics(data):
    size = (int(data['width']), int(data['height']))
    if min(size) <= 0:
        raise ValueError('color projection dimensions must be positive')
    values = np.array([data[key] for key in ('fx', 'fy', 'cx', 'cy')], dtype=float)
    if not np.all(np.isfinite(values)) or np.any(values[:2] <= 0):
        raise ValueError('color projection intrinsics must be finite with positive focal lengths')
    coefficients = np.asarray(data['coeffs'], dtype=float)
    if coefficients.shape != (5,) or not np.all(np.isfinite(coefficients)):
        raise ValueError('color projection requires five finite distortion coefficients')
    model = str(data['model']).split('.')[-1]
    if model not in ('brown_conrady', 'inverse_brown_conrady', 'none'):
        raise ValueError('unsupported color projection distortion model: ' + model)
    return size, values, coefficients, model


class ColorProjectionCorrection:
    """Immutable mapping from calibrated raw RGB pixels to an SDK projection."""

    def __init__(self, measured_intrinsics, sdk_intrinsics):
        source_size, source, source_d, source_model = _validate_intrinsics(measured_intrinsics)
        target_size, target, target_d, target_model = _validate_intrinsics(sdk_intrinsics)
        if source_size != target_size:
            raise ValueError('color projection source and SDK stream dimensions differ')
        if source_model != target_model or not np.allclose(source_d, target_d, rtol=0, atol=1e-9):
            raise ValueError('affine color projection requires identical distortion models and coefficients')
        self.width, self.height = target_size
        u, v = np.meshgrid(np.arange(self.width), np.arange(self.height))
        scale_x, scale_y = source[:2]/target[:2]
        offset_x, offset_y = source[2:]-target[2:]*[scale_x, scale_y]
        self.map_x = (u*scale_x+offset_x).astype(np.float32)
        self.map_y = (v*scale_y+offset_y).astype(np.float32)
        # Every published output pixel must originate from the captured RGB.
        # There is no implicit black padding or extrapolated detector evidence.
        if (self.map_x.min() < 0 or self.map_y.min() < 0 or
                self.map_x.max() > self.width-1 or self.map_y.max() > self.height-1):
            raise ValueError('color projection would require pixels outside the captured RGB')
        self.map_x.setflags(write=False)
        self.map_y.setflags(write=False)

    def apply(self, color):
        image = np.asarray(color)
        if image.shape != (self.height, self.width, 3) or image.dtype != np.uint8:
            raise ValueError('color projection expects a matching uint8 BGR image')
        return cv2.remap(image, self.map_x, self.map_y, cv2.INTER_LINEAR)
