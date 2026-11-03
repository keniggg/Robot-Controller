"""One calibrated RGB-D pair on a shared ideal pinhole raster.

Depth uses a proved injective forward nearest-cell map: every current source
sample is copied once, never interpolated or used to fill an empty destination. Source
pixel cells are enclosed through every float32 operation of the SDK's ten-step
Brown deprojection. The resulting extra beam margin is part of the immutable
geometry contract, not a change to physical clearance or depth uncertainty.
"""
import copy
import base64
import hashlib
import json
import zlib

import cv2
import numpy as np


SCHEMA = 'rgbd_sdk_pair_rectification_v1'
PROOF = 'sdk_brown_10_iterations_float32_outward_intervals_v1'
SUPPORTED = ('none', 'brown_conrady', 'inverse_brown_conrady')


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def _array_digest(*arrays):
    result = hashlib.sha256()
    for array in arrays:
        result.update(np.ascontiguousarray(array).tobytes())
    return result.hexdigest()


def _decode_valid_mask(record, output):
    """A bounded bitmap, never an unbounded decompressor or inferred RGB ROI."""
    encoded = record.get('rgb_valid_mask_zlib_base64')
    if type(encoded) is not str or len(encoded) > 400000:
        raise ValueError('rectification_rgb_valid_encoding_invalid')
    count = output['height']*output['width']
    expected = (count+7)//8
    try:
        compressed = base64.b64decode(encoded, validate=True)
        decoder = zlib.decompressobj()
        packed = decoder.decompress(compressed, expected+1)
        if (len(packed) != expected or not decoder.eof
                or decoder.unconsumed_tail or decoder.unused_data):
            raise ValueError('rectification_rgb_valid_size_invalid')
        bits = np.unpackbits(np.frombuffer(packed, dtype=np.uint8))
        if np.any(bits[count:]):
            raise ValueError('rectification_rgb_valid_padding_invalid')
        mask = bits[:count].reshape(output['height'], output['width']).astype(bool)
    except (ValueError, TypeError, zlib.error) as exc:
        raise ValueError('rectification_rgb_valid_encoding_invalid') from exc
    if _array_digest(mask) != record.get('rgb_valid_mask_sha256'):
        raise ValueError('rectification_rgb_valid_digest_invalid')
    mask.setflags(write=False)
    return mask


def _intrinsics(value, *, sdk_values=True):
    if not isinstance(value, dict):
        raise ValueError('rectification_intrinsics_required')
    result = copy.deepcopy(value)
    for key in ('width', 'height'):
        if type(result.get(key)) is not int or not 1 <= result[key] <= 4096:
            raise ValueError('rectification_image_dimensions_invalid')
    if result['width']*result['height'] > 2097152:
        raise ValueError('rectification_image_budget_exceeded')
    for key in ('fx', 'fy', 'cx', 'cy'):
        raw = result.get(key)
        if (isinstance(raw, (bool, np.bool_)) or not isinstance(raw, (int, float, np.number))
                or not np.isrealobj(raw) or not np.isfinite(raw)
                or (sdk_values and float(np.float32(raw)) != float(raw))):
            raise ValueError('rectification_requires_exact_sdk_float_intrinsics')
        result[key] = float(raw)
    if min(result['fx'], result['fy']) <= 0:
        raise ValueError('rectification_focal_length_invalid')
    model = str(result.get('model', '')).split('.')[-1]
    if model not in SUPPORTED:
        raise ValueError('rectification_distortion_model_unproved: '+model)
    coefficients = np.asarray(result.get('coeffs'), dtype=object)
    if (coefficients.shape != (5,) or any(isinstance(x, (bool, np.bool_))
            or not isinstance(x, (int, float, np.number)) or not np.isrealobj(x)
            or not np.isfinite(x) or float(np.float32(x)) != float(x) for x in coefficients)):
        raise ValueError('rectification_requires_exact_sdk_float_coefficients')
    result['model'], result['coeffs'] = model, [float(x) for x in coefficients]
    if model == 'none' and any(result['coeffs']):
        raise ValueError('rectification_none_model_has_distortion')
    return {key: result[key] for key in ('width', 'height', 'fx', 'fy', 'cx', 'cy', 'model', 'coeffs')}


def _out(lo, hi):
    """Enclose each SDK float operation, including its rounding (not just f64)."""
    with np.errstate(over='ignore', invalid='ignore'):
        lower = np.nextafter(np.asarray(lo, dtype=np.float32), np.float32(-np.inf)).astype(float)
        upper = np.nextafter(np.asarray(hi, dtype=np.float32), np.float32(np.inf)).astype(float)
    if not np.all(np.isfinite(lower)) or not np.all(np.isfinite(upper)):
        raise ValueError('rectification_interval_nonfinite')
    return lower, upper


def _constant(value):
    return np.asarray(value, dtype=float), np.asarray(value, dtype=float)


def _add(a, b):
    return _out(a[0]+b[0], a[1]+b[1])


def _sub(a, b):
    return _out(a[0]-b[1], a[1]-b[0])


def _mul(a, b):
    values = np.broadcast_arrays(a[0]*b[0], a[0]*b[1], a[1]*b[0], a[1]*b[1])
    return _out(np.minimum.reduce(values), np.maximum.reduce(values))


def _div(a, b):
    if np.any((b[0] <= 0) & (b[1] >= 0)):
        raise ValueError('rectification_interval_divisor_contains_zero')
    values = np.broadcast_arrays(a[0]/b[0], a[0]/b[1], a[1]/b[0], a[1]/b[1])
    return _out(np.minimum.reduce(values), np.maximum.reduce(values))


def _square(a):
    low = np.where((a[0] <= 0) & (a[1] >= 0), 0., np.minimum(a[0]**2, a[1]**2))
    result = _out(low, np.maximum(a[0]**2, a[1]**2))
    return np.maximum(result[0], 0.), result[1]


def source_cell_ray_intervals(intrinsics):
    """Continuous enclosure of every source pixel cell, including SDK rounding.

    These are interval evaluations, not sampled corners. Inverse Brown uses
    xq/yq after the inverse radial factor; ordinary Brown uses x/y. This matches
    librealsense rs2_deproject_pixel_to_point, including its ten iterations.
    """
    data = _intrinsics(intrinsics)
    u = np.arange(data['width'], dtype=float)[None, :]
    v = np.arange(data['height'], dtype=float)[:, None]
    xo = _div(_sub((u-.5, u+.5), _constant(data['cx'])), _constant(data['fx']))
    yo = _div(_sub((v-.5, v+.5), _constant(data['cy'])), _constant(data['fy']))
    x, y = xo, yo
    if any(abs(c) >= np.finfo(np.float32).eps for c in data['coeffs']):
        c = tuple(_constant(k) for k in data['coeffs'])
        one, two = _constant(1.), _constant(2.)
        for _ in range(10):
            r2 = _add(_square(x), _square(y))
            radial = _add(one, _mul(_add(_mul(_add(_mul(c[4], r2), c[1]), r2), c[0]), r2))
            inverse = _div(one, radial)
            xq, yq = (_div(x, inverse), _div(y, inverse)) if data['model'] == 'inverse_brown_conrady' else (x, y)
            dx = _add(_mul(_mul(_mul(two, c[2]), xq), yq),
                      _mul(c[3], _add(r2, _mul(two, _square(xq)))))
            dy = _add(_mul(_mul(_mul(two, c[3]), xq), yq),
                      _mul(c[2], _add(r2, _mul(two, _square(yq)))))
            x, y = _mul(_sub(xo, dx), inverse), _mul(_sub(yo, dy), inverse)
    return tuple(np.broadcast_to(a, (data['height'], data['width'])) for a in (x[0], x[1], y[0], y[1]))


def validate_rectification_contract(contract, *, output_intrinsics=None):
    """Validate the source-bound geometry record consumed by beam builders."""
    if not isinstance(contract, dict):
        raise ValueError('rectification_geometry_contract_required')
    record = copy.deepcopy(contract)
    digest = record.pop('digest', None)
    if digest != _digest(record) or record.get('schema') != SCHEMA or record.get('proof') != PROOF:
        raise ValueError('rectification_geometry_contract_invalid')
    source = _intrinsics(record.get('source_intrinsics'))
    output = _intrinsics(record.get('output_intrinsics'), sdk_values=False)
    offset = record.get('canvas_translation_px')
    if (not isinstance(offset, list) or len(offset) != 2
            or any(type(v) is not int or v < 0 for v in offset)):
        raise ValueError('rectification_canvas_translation_invalid')
    expected = dict(source, model='none', coeffs=[0.]*5, width=output['width'], height=output['height'],
                    cx=source['cx']+offset[0], cy=source['cy']+offset[1])
    count = source['width']*source['height']
    if (output != expected or output['width'] < source['width'] or output['height'] < source['height']
            or any(record.get(key) != count for key in ('source_pixel_count', 'mapped_source_pixel_count', 'unique_destination_pixel_count'))
            or (output_intrinsics is not None and output != _intrinsics(output_intrinsics, sdk_values=False))):
        raise ValueError('rectification_output_geometry_mismatch')
    margin = np.asarray(record.get('source_cell_extra_margin_px'), dtype=float)
    if margin.shape != (2,) or not np.isfinite(margin).all() or np.any(margin < 0):
        raise ValueError('rectification_beam_margin_invalid')
    for key in ('depth_forward_map_sha256', 'rgb_inverse_map_sha256', 'rgb_valid_mask_sha256'):
        value = record.get(key)
        if type(value) is not str or len(value) != 64 or any(c not in '0123456789abcdef' for c in value):
            raise ValueError('rectification_geometry_map_digest_invalid')
    _decode_valid_mask(record, output)
    return tuple(map(float, margin))


def rectification_contract(camera):
    """Read the same contract from live camera params or a frozen snapshot."""
    if not isinstance(camera, dict):
        raise ValueError('rectification_camera_geometry_invalid')
    profile = camera.get('runtime_profile_geometry', camera.get('runtime_profile', {}))
    if not isinstance(profile, dict):
        raise ValueError('rectification_camera_geometry_invalid')
    contract = profile.get('rectification')
    if contract is None:
        configured = camera.get('pair_rectification') or {}
        if not isinstance(configured, dict):
            raise ValueError('rectification_camera_geometry_invalid')
        if ('rectified' in str(profile.get('geometry_intrinsics_source', ''))
                or 'rectified' in str(camera.get('intrinsics_source', ''))
                or configured.get('enabled') is True):
            raise ValueError('rectification_declared_but_geometry_contract_missing')
        return None
    validate_rectification_contract(contract, output_intrinsics=profile.get('geometry_intrinsics'))
    return contract


def rectification_rgb_valid_mask(camera, image_shape):
    """Unknown RGB consumers reject regions containing synthetic padding."""
    contract = rectification_contract(camera)
    if contract is None:
        return None
    output = contract['output_intrinsics']
    if tuple(image_shape) != (output['height'], output['width']):
        raise ValueError('rectification_rgb_valid_shape_mismatch')
    return _decode_valid_mask(contract, output)


def require_rectified_source_after_activation(camera, source_stamp_ns):
    """Queued pixels from the preceding geometry can never use a new K/map."""
    contract = rectification_contract(camera)
    profile = camera.get('runtime_profile_geometry', camera.get('runtime_profile', {}))
    activation = profile.get('geometry_activation_stamp_ns')
    if contract is None and activation is None:
        return
    if type(activation) is str:
        if (not 1 <= len(activation) <= 19 or not activation.isascii()
                or not activation.isdigit() or str(int(activation)) != activation):
            raise ValueError('rectification_source_predates_geometry_activation')
        activation = int(activation)
    if (type(activation) is not int or activation <= 0 or type(source_stamp_ns) is not int
            or activation > 2**63-1 or source_stamp_ns <= activation):
        raise ValueError('rectification_source_predates_geometry_activation')


class PairedPinholeRectification:
    """Precomputed maps for one immutable SDK geometry profile; no device I/O."""
    def __init__(self, intrinsics, sdk=None):
        self.source_intrinsics = _intrinsics(intrinsics)
        data = self.source_intrinsics
        self.source_height, self.source_width = data['height'], data['width']
        self.height, self.width = self.source_height, self.source_width
        self.output_intrinsics = dict(data, model='none', coeffs=[0.]*5)
        self.identity = not any(data['coeffs'])
        u, v = np.meshgrid(np.arange(self.width), np.arange(self.height))
        if self.identity:
            target_u, target_v = u.astype(float), v.astype(float)
            map_x, map_y = u.astype(np.float32), v.astype(np.float32)
            margin = np.zeros(2)
            offset = [0, 0]
            rgb_valid = np.ones(u.shape, bool)
        else:
            if sdk is None:
                raise ValueError('rectification_sdk_numerical_functions_required')
            if getattr(sdk, '__version__', None) != '2.56.5':
                raise ValueError('rectification_sdk_interval_implementation_unverified')
            intrinsic = sdk.intrinsics()
            intrinsic.width, intrinsic.height = self.width, self.height
            intrinsic.fx, intrinsic.fy = data['fx'], data['fy']
            intrinsic.ppx, intrinsic.ppy = data['cx'], data['cy']
            intrinsic.model = getattr(sdk.distortion, data['model'])
            intrinsic.coeffs = data['coeffs']
            pixels = np.c_[u.ravel(), v.ravel()].astype(float)
            rays = np.asarray([sdk.rs2_deproject_pixel_to_point(intrinsic, p.tolist(), 1.) for p in pixels])
            if rays.shape != (self.width*self.height, 3) or not np.isfinite(rays).all() or not np.all(rays[:, 2] == 1.):
                raise ValueError('rectification_sdk_ray_invalid')
            target_u = (data['fx']*rays[:, 0]+data['cx']).reshape(u.shape)
            target_v = (data['fy']*rays[:, 1]+data['cy']).reshape(u.shape)
            xl, xh, yl, yh = source_cell_ray_intervals(data)
            # The SDK centre ray must be in the continuous cell proof, too.
            rx, ry = rays[:, 0].reshape(u.shape), rays[:, 1].reshape(v.shape)
            if not (np.all((xl <= rx) & (rx <= xh)) and np.all((yl <= ry) & (ry <= yh))):
                raise ValueError('rectification_sdk_not_enclosed_by_model_proof')
            # float64 outward rounding for the ideal pinhole coordinate change.
            lo_u = np.nextafter(np.nextafter(data['fx']*xl, -np.inf)+data['cx'], -np.inf)
            hi_u = np.nextafter(np.nextafter(data['fx']*xh, np.inf)+data['cx'], np.inf)
            lo_v = np.nextafter(np.nextafter(data['fy']*yl, -np.inf)+data['cy'], -np.inf)
            hi_v = np.nextafter(np.nextafter(data['fy']*yh, np.inf)+data['cy'], np.inf)
            # Grow the canvas; never crop source measurements or change focal scale.
            offset = [max(0, int(np.ceil(-.5-float(lo_u.min())))),
                      max(0, int(np.ceil(-.5-float(lo_v.min()))))]
            self.width = max(self.width, int(np.ceil(np.nextafter(float(hi_u.max())+offset[0]+.5, np.inf))))
            self.height = max(self.height, int(np.ceil(np.nextafter(float(hi_v.max())+offset[1]+.5, np.inf))))
            self.output_intrinsics.update(width=self.width, height=self.height,
                                          cx=data['cx']+offset[0], cy=data['cy']+offset[1])
            _intrinsics(self.output_intrinsics, sdk_values=False)  # fixed image budget
            target_u += offset[0]
            target_v += offset[1]
            dest_u, dest_v = np.rint(target_u), np.rint(target_v)
            lo_u = np.nextafter(lo_u+offset[0], -np.inf)
            hi_u = np.nextafter(hi_u+offset[0], np.inf)
            lo_v = np.nextafter(lo_v+offset[1], -np.inf)
            hi_v = np.nextafter(hi_v+offset[1], np.inf)
            if (lo_u.min() < -.5 or lo_v.min() < -.5
                    or hi_u.max() > self.width-.5 or hi_v.max() > self.height-.5):
                raise ValueError('rectification_complete_source_cell_outside_canvas')
            margin = np.maximum(0., [max(np.max(dest_u-.5-lo_u), np.max(hi_u-dest_u-.5)),
                                      max(np.max(dest_v-.5-lo_v), np.max(hi_v-dest_v-.5))])
            margin = np.nextafter(margin, np.inf)
            output_u, output_v = np.meshgrid(np.arange(self.width), np.arange(self.height))
            ideal = np.c_[(output_u.ravel()-self.output_intrinsics['cx'])/data['fx'],
                          (output_v.ravel()-self.output_intrinsics['cy'])/data['fy'], np.ones(output_u.size)]
            pull = np.asarray([sdk.rs2_project_point_to_pixel(intrinsic, ray.tolist()) for ray in ideal])
            if not np.isfinite(pull).all():
                raise ValueError('rectification_rgb_map_nonfinite')
            map_x = pull[:, 0].reshape(output_u.shape).astype(np.float32)
            map_y = pull[:, 1].reshape(output_u.shape).astype(np.float32)
            rgb_valid = ((map_x >= 0) & (map_y >= 0)
                         & (map_x <= self.source_width-1) & (map_y <= self.source_height-1))
        dest_u, dest_v = np.rint(target_u).astype(np.int64), np.rint(target_v).astype(np.int64)
        valid = (dest_u >= 0) & (dest_u < self.width) & (dest_v >= 0) & (dest_v < self.height)
        if not np.all(valid):
            raise ValueError('rectification_source_center_outside_canvas')
        self.source_indices = np.arange(u.size, dtype=np.int64)
        self.target_indices = (dest_v*self.width+dest_u).ravel()
        if len(np.unique(self.target_indices)) != u.size:
            raise ValueError('rectification_noninjective_source_map_unproved_occupancy')
        self.map_x, self.map_y, self.rgb_valid = map_x, map_y, rgb_valid
        for array in (self.source_indices, self.target_indices, self.map_x, self.map_y, self.rgb_valid):
            array.setflags(write=False)
        record = dict(schema=SCHEMA, proof=PROOF, enabled=not self.identity,
            source_intrinsics=copy.deepcopy(data), output_intrinsics=copy.deepcopy(self.output_intrinsics),
            sdk_version=str(getattr(sdk, '__version__', 'identity_no_sdk_required')),
            depth_mapping='forward_nearest_cell; complete_injective_source_map; no_fill',
            rgb_mapping='ideal_ray_to_sdk_pixel; linear_resampling; unobserved_padding_zero',
            canvas_translation_px=offset,
            depth_forward_map_sha256=_array_digest(self.source_indices, self.target_indices),
            rgb_inverse_map_sha256=_array_digest(map_x, map_y),
            rgb_valid_mask_sha256=_array_digest(rgb_valid),
            rgb_valid_mask_zlib_base64=base64.b64encode(zlib.compress(np.packbits(rgb_valid).tobytes(), 9)).decode('ascii'),
            unobserved_rgb_pixel_count=int(np.count_nonzero(~rgb_valid)),
            source_cell_extra_margin_px=margin.tolist(),
            center_quantization_max_abs_xy_px=[float(np.max(abs(target_u-dest_u))), float(np.max(abs(target_v-dest_v)))],
            source_pixel_count=self.source_width*self.source_height, mapped_source_pixel_count=int(valid.sum()),
            unique_destination_pixel_count=int(len(np.unique(self.target_indices))),
            physical_camera_axes_changed=False, depth_interpolation=False,
            current_depth_holes_filled=False, original_depth_units_preserved=True)
        self.contract = dict(record, digest=_digest(record))
        validate_rectification_contract(self.contract)

    def apply(self, color, depth, *, return_source_indices=False):
        color, depth = np.asarray(color), np.asarray(depth)
        if color.shape != (self.source_height, self.source_width, 3) or color.dtype != np.uint8:
            raise ValueError('rectification_color_pair_invalid')
        if depth.shape != (self.source_height, self.source_width) or depth.dtype != np.uint16:
            raise ValueError('rectification_native_z16_pair_required')
        if self.identity:
            output_color, output_depth = color.copy(), depth.copy()
            if return_source_indices:
                provenance = np.arange(depth.size, dtype=np.int64).reshape(depth.shape)
                provenance[depth == 0] = -1
        else:
            output_color = cv2.remap(color, self.map_x, self.map_y, cv2.INTER_LINEAR)
            output_color[~self.rgb_valid] = 0
            output_depth = np.zeros((self.height, self.width), dtype=np.uint16)
            # Complete injectivity was proved once at construction: no z-buffer
            # conflict, sorting or discarded farther measurement is permitted.
            output_depth.ravel()[self.target_indices] = depth.ravel()
            if return_source_indices:
                provenance = np.full((self.height, self.width), -1, dtype=np.int64)
                valid = depth.ravel() > 0
                provenance.ravel()[self.target_indices[valid]] = self.source_indices[valid]
        if return_source_indices:
            return output_color, output_depth, provenance
        return output_color, output_depth
