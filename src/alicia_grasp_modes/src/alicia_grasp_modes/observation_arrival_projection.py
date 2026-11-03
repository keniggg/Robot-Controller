"""Endpoint prediction for the same measured prism used at observation arrival.

This is neither observed geometry at a future pose nor a tracking guarantee.
The current OBB and every original path/arrival check remain independent.
"""
from alicia_flexible_grasp.robot.observation_view_limits import MINIMUM_OBSERVATION_IMAGE_MARGIN_PX
import hashlib
import json
import math
from types import SimpleNamespace

import cv2
import numpy as np

from alicia_flexible_grasp.robot.observation_view_guard import _camera, _project, _transform
from alicia_flexible_grasp.vision.rgbd_rectification import (
    rectification_contract, rectification_rgb_valid_mask, require_rectified_source_after_activation)
from .rgb_instance import freeze_observation_spatial_reference
from .tabletop import Config


def _hash(value):
    return hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest()


def arrival_projection_camera_signature(camera):
    profile = camera.get('runtime_profile_geometry', camera.get('runtime_profile', {}))
    value = dict(rectification=rectification_contract(camera),
                 activation=profile.get('geometry_activation_stamp_ns'))
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def capture_arrival_projection(status, selection, source_stamp_ns, camera):
    """Copy one exact ready receipt; historical footprint supplies no new pixels."""
    from .unknown_pending_identity import reference_binding
    binding = reference_binding(status, selection)
    if (selection.get('mode') != 'unknown' or selection.get('strategy') != 'observe_return'
            or status.get('state') != 'ready' or type(source_stamp_ns) is not int
            or status.get('source_stamp_ns') != source_stamp_ns):
        raise ValueError('arrival projection requires the exact ready observation source')
    height = status['reference_object_height_m']
    config = Config()
    reference = freeze_observation_spatial_reference(SimpleNamespace(
        support_footprint_base=status['reference_support_footprint_base'],
        metrics={'median_height_m': height}), status['reference_support_plane_base'])
    if not config.foreground_min_m <= height <= config.foreground_max_m:
        raise ValueError('arrival projection measured height outside original range')
    footprint, plane = reference['footprint_base'], reference['support_plane_base']
    normal = plane[:3] / np.linalg.norm(plane[:3])
    if np.max(np.abs(footprint @ normal + plane[3]/np.linalg.norm(plane[:3]))) > 1e-6:
        raise ValueError('arrival projection footprint is not on its frozen plane')
    axis = np.eye(3)[np.argmin(np.abs(normal))]
    u = np.cross(normal, axis); u /= np.linalg.norm(u)
    xy = footprint @ np.stack((u, np.cross(normal, u)), axis=1)
    hull = cv2.convexHull(np.asarray(xy, np.float32))
    if cv2.contourArea(hull) <= 1e-8 or np.max(np.ptp(xy, axis=0)) > config.max_surface_extent_m:
        raise ValueError('arrival projection measured footprint invalid')
    width, image_height = camera['width'], camera['height']
    _camera([camera[k] for k in ('fx', 'fy', 'cx', 'cy')], width, image_height, MINIMUM_OBSERVATION_IMAGE_MARGIN_PX, .07, .5)
    require_rectified_source_after_activation(camera, source_stamp_ns)
    valid = rectification_rgb_valid_mask(camera, (image_height, width))
    if valid is None:
        valid = np.ones((image_height, width), dtype=bool)
    valid = np.array(valid, dtype=bool, copy=True)
    prism = np.vstack((footprint, footprint+normal*height))
    integral = np.pad(np.cumsum(np.cumsum(~valid, axis=0, dtype=np.int64), axis=1), ((1, 0), (1, 0)))
    for value in (prism, valid, integral):
        value.setflags(write=False)
    audit = dict(policy='frozen_measured_arrival_prism_endpoint_prediction',
        source_stamp_ns=source_stamp_ns, anchor_source_stamp_ns=status['anchor_source_stamp_ns'],
        reference_binding=binding, footprint_sha256=_hash(footprint), support_plane_sha256=_hash(plane),
        measured_height_m=float(height), prism_sha256=_hash(prism), rgb_valid_sha256=_hash(valid),
        invalid_integral_sha256=_hash(integral),
        camera_signature=arrival_projection_camera_signature(camera),
        endpoint_only=True, historical_pixels_counted=0, execution_authority=False,
        certifies_arrival_or_tracking=False)
    return dict(prism_base=prism, rgb_valid=valid, invalid_integral=integral, audit=audit)


def validate_arrival_projection(contract, base_from_camera, intrinsics, image_width, image_height,
        *, roi_margin_px=MINIMUM_OBSERVATION_IMAGE_MARGIN_PX, minimum_depth_m=.07, maximum_depth_m=.5):
    """The original image margin plus observed background for a future endpoint."""
    if (not math.isfinite(float(roi_margin_px)) or roi_margin_px < MINIMUM_OBSERVATION_IMAGE_MARGIN_PX
            or minimum_depth_m < .07 or maximum_depth_m > .5):
        raise ValueError('arrival projection cannot relax original visibility limits')
    k = _camera(intrinsics, image_width, image_height, roi_margin_px, minimum_depth_m, maximum_depth_m)
    transform = _transform(base_from_camera, 'arrival projection camera')
    prism = np.asarray(contract['prism_base'], dtype=float)
    valid = np.asarray(contract['rgb_valid'])
    integral = np.asarray(contract['invalid_integral'])
    if (prism.ndim != 2 or prism.shape[1:] != (3,) or not 6 <= len(prism) <= 2048
            or not np.all(np.isfinite(prism)) or valid.shape != (image_height, image_width)
            or valid.dtype != np.bool_ or integral.shape != (image_height+1, image_width+1)
            or integral.dtype != np.int64):
        raise ValueError('arrival projection contract invalid')
    if any(_hash(value) != contract['audit'][key] for value, key in (
            (prism, 'prism_sha256'), (valid, 'rgb_valid_sha256'), (integral, 'invalid_integral_sha256'))):
        raise ValueError('arrival projection immutable inputs changed')
    uv, depth = _project(prism, transform, k)
    if (not np.all(np.isfinite(uv)) or np.min(depth) < minimum_depth_m
            or np.max(depth) > maximum_depth_m):
        raise ValueError('arrival prism exceeds original camera depth range')
    edge = float(np.min(np.column_stack((uv[:, 0], image_width-1-uv[:, 0],
                                         uv[:, 1], image_height-1-uv[:, 1]))))
    if edge < roi_margin_px:
        raise ValueError('arrival prism leaves required camera image margin')
    # This known prism/background can be predicted. Future first16 seed pixels
    # and actual RGB/identity still require the unchanged arrival-time audit.
    margin = max(8, int(round(16 * min(image_width/640., image_height/480.))))
    low = np.floor(uv.min(axis=0)).astype(int)-margin
    high = np.ceil(uv.max(axis=0)).astype(int)+1+margin
    x0, y0 = map(int, low); x1, y1 = map(int, high)
    if not 0 <= x0 < x1 <= image_width or not 0 <= y0 < y1 <= image_height:
        raise ValueError('arrival prism background exceeds camera canvas')
    invalid = int(integral[y1, x1]-integral[y0, x1]-integral[y1, x0]+integral[y0, x0])
    if invalid:
        raise ValueError('arrival prism required background contains unobserved RGB')
    return dict(ok=True, policy=contract['audit']['policy'], endpoint_only=True,
        minimum_image_edge_distance_px=edge, required_image_margin_px=float(roi_margin_px),
        prism_uv_min_px=uv.min(axis=0).tolist(), prism_uv_max_px=uv.max(axis=0).tolist(),
        required_observed_box_xyxy=[x0, y0, x1, y1], background_margin_px=margin,
        unobserved_rgb_pixels=invalid, source=dict(contract['audit']),
        certifies_arrival_or_tracking=False, certifies_continuous_visibility=False)
