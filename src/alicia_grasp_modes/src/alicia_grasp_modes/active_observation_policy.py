"""Bounded supplemental observation evidence; never grants motion authority."""
from collections import defaultdict
from itertools import product
import hashlib
import json
import math

import numpy as np


MAX_OBSERVATIONS = 3


def evidence_digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def observation_budget(value=MAX_OBSERVATIONS):
    if type(value) is not int or not 1 <= value <= MAX_OBSERVATIONS:
        raise ValueError('OBSERVATION_BUDGET_INVALID: maximum must be an integer in [1, 3]')
    return value


def registered_information_gain(before, after, registration, *, voxel_size_m=.0025,
                                depth_uncertainty_m=0., minimum_novel_voxels=3):
    """Count new occupied cells beyond measured noise, after real registration.

    New timestamps and replacement points in old voxels do not count. Only
    seed-connected points retained by the original fusion and attributed to
    the newest view are considered. This is evidence of new measured coverage,
    not evidence of contact, an unseen shape, or future observation success.
    """
    if (getattr(registration, 'ok', False) is not True
            or before.identity != after.identity
            or tuple(after.view_stamps_ns[:-1]) != tuple(before.view_stamps_ns)
            or after.view_stamps_ns[-1] <= before.view_stamps_ns[-1]):
        raise ValueError('OBSERVATION_GAIN_REGISTRATION_BINDING_INVALID')
    for value in (voxel_size_m, depth_uncertainty_m):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError('OBSERVATION_GAIN_SCALE_INVALID')
    if (voxel_size_m <= 0. or depth_uncertainty_m < 0.
            or type(minimum_novel_voxels) is not int or minimum_novel_voxels < 3):
        raise ValueError('OBSERVATION_GAIN_SCALE_INVALID')
    old = np.asarray(before.points_base, dtype=float)
    points = np.asarray(after.points_base, dtype=float)
    provenance = np.asarray(after.view_indices)
    if (old.ndim != 2 or old.shape[1:] != (3,) or not len(old)
            or points.ndim != 2 or points.shape[1:] != (3,)
            or provenance.shape != (len(points),)
            or not np.all(np.isfinite(old)) or not np.all(np.isfinite(points))):
        raise ValueError('OBSERVATION_GAIN_POINTS_INVALID')
    newest = points[provenance == len(before.view_stamps_ns)]
    threshold = max(voxel_size_m * .5, depth_uncertainty_m)
    bucket_size = max(voxel_size_m, threshold)
    occupied = {tuple(row) for row in np.floor(old / voxel_size_m).astype(np.int64)}
    buckets = defaultdict(list)
    for point, key in zip(old, np.floor(old / bucket_size).astype(np.int64)):
        buckets[tuple(key)].append(point)
    buckets = {key: np.asarray(value) for key, value in buckets.items()}
    new_cells = set()
    for point in newest:
        cell = tuple(np.floor(point / voxel_size_m).astype(np.int64))
        if cell in occupied or cell in new_cells:
            continue
        key = np.floor(point / bucket_size).astype(np.int64)
        novel = True
        for delta in product((-1, 0, 1), repeat=3):
            nearby = buckets.get(tuple(key + delta))
            if nearby is not None and np.any(np.sum((nearby-point)**2, axis=1) <= threshold**2):
                novel = False
                break
        if novel:
            new_cells.add(cell)
    return dict(ok=len(new_cells) >= minimum_novel_voxels,
        code='OBSERVATION_NEW_MEASURED_COVERAGE' if len(new_cells) >= minimum_novel_voxels
             else 'OBSERVATION_NO_INFORMATION_GAIN',
        policy='registered_seed_connected_new_occupied_voxels_beyond_depth_noise',
        target_epoch=int(after.identity.epoch), target_track_id=after.identity.track_id,
        previous_source_stamps_ns=list(before.view_stamps_ns),
        source_stamps_ns=list(after.view_stamps_ns), source_stamp_ns=int(after.view_stamps_ns[-1]),
        newest_retained_point_count=len(newest), new_occupied_voxel_count=len(new_cells),
        minimum_novel_voxels=minimum_novel_voxels, voxel_size_m=voxel_size_m,
        depth_uncertainty_m=depth_uncertainty_m, novelty_distance_m=threshold,
        registration_ok=True, contact_or_motion_authorized=False,
        source_points_sha256=hashlib.sha256(np.ascontiguousarray(newest).tobytes()).hexdigest())


def distinct_view(camera_origin, target_center, visited, uncertainty_m):
    """Reject a repeated viewing ray, including radial-only and roll repeats."""
    camera, center = np.asarray(camera_origin, float), np.asarray(target_center, float)
    if (camera.shape != (3,) or center.shape != (3,) or not np.all(np.isfinite(camera))
            or not np.all(np.isfinite(center)) or not math.isfinite(uncertainty_m)
            or uncertainty_m < 0.):
        raise ValueError('OBSERVATION_VIEW_HISTORY_INVALID')
    ray = camera - center
    radius = float(np.linalg.norm(ray))
    if radius <= 1e-9:
        raise ValueError('OBSERVATION_VIEW_HISTORY_INVALID')
    for origin in visited:
        old = np.asarray(origin, float) - center
        length = float(np.linalg.norm(old))
        if old.shape != (3,) or not np.all(np.isfinite(old)) or length <= 1e-9:
            raise ValueError('OBSERVATION_VIEW_HISTORY_INVALID')
        angle = math.acos(float(np.clip(np.dot(ray / radius, old / length), -1., 1.)))
        if angle <= math.atan2(max(.0025, uncertainty_m), min(radius, length)) + 1e-9:
            return False
    return True


def information_gain_valid(report):
    """Validate the serialized evidence before it can justify another view."""
    try:
        return (report['ok'] is True
            and report['code'] == 'OBSERVATION_NEW_MEASURED_COVERAGE'
            and report['policy'] == 'registered_seed_connected_new_occupied_voxels_beyond_depth_noise'
            and report['registration_ok'] is True and report['contact_or_motion_authorized'] is False
            and type(report['new_occupied_voxel_count']) is int
            and type(report['minimum_novel_voxels']) is int
            and report['new_occupied_voxel_count'] >= report['minimum_novel_voxels'] >= 3
            and type(report['source_stamp_ns']) is int and report['source_stamp_ns'] > 0
            and report['source_stamps_ns'][-1] == report['source_stamp_ns']
            and report['source_stamps_ns'][:-1] == report['previous_source_stamps_ns']
            and all(type(value) is int and value > 0 for value in report['source_stamps_ns'])
            and all(a < b for a, b in zip(report['source_stamps_ns'], report['source_stamps_ns'][1:]))
            and isinstance(report['source_points_sha256'], str) and len(report['source_points_sha256']) == 64)
    except (AttributeError, IndexError, KeyError, TypeError, ValueError):
        return False
