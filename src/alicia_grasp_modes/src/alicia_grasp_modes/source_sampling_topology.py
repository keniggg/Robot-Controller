"""Exact pre-rectification sampling adjacency; no interpolation or device I/O.

An injective forward map can turn two adjacent measured source cells into
diagonal canvas cells. Only the verified original source graph restores that
adjacency. Empty destination cells never acquire depth or foreground labels.
"""
from copy import deepcopy
import importlib
import json
import threading
import time

import numpy as np

from alicia_flexible_grasp.vision.rgbd_rectification import (
    PairedPinholeRectification, rectification_contract,
    require_rectified_source_after_activation)


def _record(camera):
    contract = rectification_contract(camera)
    if contract is None:
        raise ValueError('current_source_topology_contract_required')
    profile = camera.get('runtime_profile_geometry', camera.get('runtime_profile', {}))
    if (profile.get('color_intrinsics') != contract['source_intrinsics']
            or profile.get('geometry_intrinsics') != contract['output_intrinsics']
            or profile.get('sdk_version') != contract['sdk_version']):
        raise ValueError('current_source_topology_profile_mismatch')
    return contract


def _key(record):
    # Compare the entire immutable proof, not an unverified digest field alone.
    return json.dumps(record, sort_keys=True, separators=(',', ':'), allow_nan=False)


def _readonly(array):
    array = np.ascontiguousarray(array)
    return np.frombuffer(array.tobytes(), dtype=array.dtype).reshape(array.shape)


class SourceSamplingTopology:
    """Construct once from a proved SDK map; validate each source before use."""
    def __init__(self, camera, *, sdk=None):
        record = deepcopy(_record(camera))
        if sdk is None and record['sdk_version'] != 'identity_no_sdk_required':
            sdk = importlib.import_module('pyrealsense2')
        mapping = PairedPinholeRectification(record['source_intrinsics'], sdk=sdk)
        if mapping.contract != record:
            raise ValueError('current_source_topology_reconstructed_map_mismatch')
        self._key = _key(record)
        self.contract_digest = record['digest']
        self.map_sha256 = record['depth_forward_map_sha256']
        self.source_shape = (mapping.source_height, mapping.source_width)
        self.output_shape = (mapping.height, mapping.width)
        self.intrinsics = tuple(record['output_intrinsics'][k] for k in ('fx', 'fy', 'cx', 'cy'))
        self.source_to_output = _readonly(mapping.target_indices)
        inverse = np.full(self.output_shape, -1, np.int64)
        inverse.ravel()[mapping.target_indices] = mapping.source_indices
        self.output_to_source = _readonly(inverse)

    def require_source(self, camera, shape, intrinsics, source_stamp_ns):
        require_rectified_source_after_activation(camera, source_stamp_ns)
        if (_key(_record(camera)) != self._key or tuple(shape) != self.output_shape
                or not np.array_equal(np.asarray(intrinsics), self.intrinsics)):
            raise ValueError('current_source_topology_source_mismatch')
        return self

    def source_array(self, value):
        value = np.asarray(value)
        if value.shape != self.output_shape:
            raise ValueError('current_source_topology_shape_mismatch')
        return value.ravel()[self.source_to_output].reshape(self.source_shape)

    def output_mask(self, mask):
        if np.asarray(mask).shape != self.source_shape:
            raise ValueError('current_source_topology_shape_mismatch')
        result = np.zeros(self.output_shape, bool)
        result.ravel()[self.source_to_output] = np.asarray(mask, dtype=bool).ravel()
        return result


class SourceSamplingTopologyCache:
    """One ready profile and one worker; construction never runs on a frame.

    A new profile immediately invalidates the old result. Pending or failed
    preparation is an explicit recovery hold, never a canvas-connectivity
    fallback. The worker only consumes copied calibration data and SDK math.
    """
    def __init__(self, builder=SourceSamplingTopology):
        self._builder = builder
        self._lock = threading.Lock()
        self._desired = self._ready = self._error = self._thread = None
        self._timing = None

    def offer(self, camera):
        record = rectification_contract(camera)
        if record is None:
            with self._lock:
                self._desired = self._ready = self._error = None
            return
        key = _key(_record(camera))
        with self._lock:
            if self._desired is not None and self._desired[0] == key:
                return
            self._desired = (key, deepcopy(camera))
            self._ready = self._error = self._timing = None
            if self._thread is None:
                self._thread = threading.Thread(target=self._prepare, daemon=True,
                    name='current-source-topology')
                self._thread.start()

    def _prepare(self):
        while True:
            with self._lock:
                desired = self._desired
                if desired is None:
                    self._thread = None
                    return
            started, cpu = time.perf_counter(), time.process_time()
            try:
                ready, error = self._builder(desired[1]), None
            except Exception as exc:
                ready, error = None, type(exc).__name__ + ': ' + str(exc)
            timing = dict(wall_sec=time.perf_counter()-started, cpu_sec=time.process_time()-cpu)
            with self._lock:
                if self._desired is not desired:
                    continue
                self._ready, self._error, self._timing = ready, error, timing
                self._thread = None
                return

    def require(self, camera, shape, intrinsics, source_stamp_ns):
        key = _key(_record(camera))
        with self._lock:
            if self._desired is None or self._desired[0] != key:
                raise ValueError('current_source_topology_profile_not_prepared')
            ready, error = self._ready, self._error
        if ready is None:
            if error is not None:
                raise ValueError('current_source_topology_preparation_failed: ' + error)
            raise ValueError('current_source_topology_pending')
        return ready.require_source(camera, shape, intrinsics, source_stamp_ns)
