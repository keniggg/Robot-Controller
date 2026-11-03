"""Candidate upstream observation helpers. No ROS, SDK construction or I/O."""
import copy
import hashlib
import json
import math

import numpy as np

SCHEMA = 'd405_upstream_observation_v1'
BASE = '/supervisor/camera/diagnostic_upstream/'
PLANES = {
    'color_native': ('bgr8', 'camera_color_optical_frame', 'SDK color before host correction/rectification'),
    'depth_native': ('16UC1', 'camera_depth_optical_frame', 'SDK Z16 before host alignment/filter/clip/rectification'),
    'depth_aligned_unfiltered': ('16UC1', 'camera_color_optical_frame', 'SDK align(color) only; before host spatial/temporal/hole filling/clip'),
    'depth_rectified_unfiltered': ('16UC1', 'camera_color_optical_frame', 'SDK align(color) then fixed injective rectification only; before all host depth filters and clip'),
}


def own(array):
    array = np.asarray(array)
    return np.frombuffer(array.tobytes(order='C'), dtype=array.dtype).reshape(array.shape)


def native_pair(frames):
    color = np.asanyarray(frames.get_color_frame().get_data())
    depth = np.asanyarray(frames.get_depth_frame().get_data())
    if color.dtype != np.uint8 or color.ndim != 3 or color.shape[2] != 3:
        raise ValueError('native SDK BGR uint8 required')
    if depth.dtype != np.uint16 or depth.ndim != 2:
        raise ValueError('native SDK Z16 required')
    return {'color_native': own(color), 'depth_native': own(depth)}


def aligned_pair(arrays, depth_frame):
    depth = np.asanyarray(depth_frame.get_data())
    if depth.dtype != np.uint16 or depth.shape != arrays['color_native'].shape[:2]:
        raise ValueError('SDK color-aligned Z16 geometry required')
    arrays['depth_aligned_unfiltered'] = own(depth)


def finish(owner, arrays, diagnostic):
    if owner.depth_scale_source != 'sdk_depth_sensor' or not math.isfinite(owner.depth_scale) or owner.depth_scale <= 0:
        raise ValueError('device depth unit not confirmed')
    if not diagnostic.get('sdk_color_alignment_applied'):
        raise ValueError('SDK color alignment not proven')
    if owner.color_projection_correction is not None:
        raise ValueError('unverified additional color correction; native observations need separate geometry review')
    depth = arrays['depth_aligned_unfiltered']; rectifier = owner.pair_rectifier
    if rectifier is None or rectifier.identity:
        rectified = depth
    else:
        indices = rectifier.target_indices
        if len(indices) != depth.size or len(np.unique(indices)) != depth.size:
            raise ValueError('unfiltered rectification is not injective')
        rectified = np.zeros((rectifier.height, rectifier.width), np.uint16)
        rectified.ravel()[indices] = depth.ravel()
    arrays['depth_rectified_unfiltered'] = own(rectified)
    keys = ('source', 'sdk_version', 'device', 'color_intrinsics', 'native_depth_intrinsics',
            'depth_to_color_extrinsics', 'alignment', 'geometry_intrinsics', 'geometry_intrinsics_source',
            'depth_scale_m_per_unit', 'depth_scale_source', 'depth_filters', 'rectification', 'mode_depth_preset')
    metadata = {'schema': SCHEMA, 'profile': {k: copy.deepcopy(owner.runtime_profile[k]) for k in keys if k in owner.runtime_profile},
        'device_frames': copy.deepcopy(diagnostic['raw_frames']), 'pair_timing': copy.deepcopy(diagnostic['pair_timing']),
        'depth_scale_m_per_unit': owner.depth_scale, 'depth_scale_source': owner.depth_scale_source,
        'regional_stationary_evidence_for_processed_comparison_only': copy.deepcopy(diagnostic.get('regional_stationary_evidence')),
        'raw_definition': 'unaltered SDK delivered Z16 before host processing; device firmware, preset, stereo and ISP processing are not disabled',
        'upstream_history_dependency': False, 'motion_authorized': False}
    return {'arrays': arrays, 'metadata': metadata}


def freeze(payload, maximum_bytes):
    if payload is None:
        return None
    if set(payload['arrays']) != set(PLANES):
        raise ValueError('incomplete upstream plane set')
    if sum(a.nbytes for a in payload['arrays'].values()) > maximum_bytes:
        raise ValueError('upstream observation exceeds memory bound')
    meta = copy.deepcopy(payload['metadata'])
    if len(json.dumps(meta, allow_nan=False)) > 1024**2:
        raise ValueError('upstream metadata exceeds bound')
    return {'arrays': {k: own(v) for k, v in payload['arrays'].items()}, 'metadata': meta}


def descriptor(payload, processed_color, processed_depth, stamp_ns, activation):
    result = copy.deepcopy(payload['metadata'])
    result.update(source_stamp_ns=int(stamp_ns), geometry_activation_stamp_ns=str(activation),
                  timestamp_meaning='same host read completion as original processed RGB-D; not exposure calibrated')
    result['planes'] = {}
    for name, array in payload['arrays'].items():
        encoding, frame, stage = PLANES[name]
        result['planes'][name] = {'topic': BASE+name, 'encoding': encoding, 'frame_id': frame,
            'height': array.shape[0], 'width': array.shape[1], 'dtype': array.dtype.str,
            'data_sha256': hashlib.sha256(array.tobytes(order='C')).hexdigest(), 'stage': stage}
    result['processed_comparison_hashes'] = {
        'rgb': hashlib.sha256(processed_color.tobytes(order='C')).hexdigest(),
        'depth_processed': hashlib.sha256(processed_depth.tobytes(order='C')).hexdigest()}
    identity = {'stamp': result['source_stamp_ns'], 'activation': result['geometry_activation_stamp_ns'],
                'frames': {k: v['frame_number'] for k, v in result['device_frames'].items()}}
    result['capture_id'] = hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest()
    return result
