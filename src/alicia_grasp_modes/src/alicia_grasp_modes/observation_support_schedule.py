"""Bounded measured-support witnesses for scheduling, never admission.

The original endpoint gate counts unique pixels and qualifying quadrants; it
has no inlier-ratio condition. A passing subset of actual measured points is
therefore sufficient to prove that the full set passes the same projection.
A failing subset proves nothing. Actual IK endpoints and whole trajectories
still require the original complete checks.
"""
from copy import deepcopy
import time

import numpy as np

from alicia_flexible_grasp.robot.observation_view_guard import (
    ObservationViewError, validate_observation_support_projection)
from .silhouette_observation_order import rigid_transform, transform_fingerprint


POLICY = 'same_source_bounded_measured_support_witness_first'
MAX_WITNESS_POINTS = 2048


def support_schedule_priority(evidence):
    return {'sufficient': 0, 'unknown': 1, 'insufficient': 2}[evidence['status']]


class ObservationSupportSchedule:
    """One immutable, already validated source; full-pose prediction cache."""
    def __init__(self, source):
        self.source = source
        points = np.asarray(source['points_base'])
        if (points.ndim != 2 or points.shape[1:] != (3,) or not 1 <= len(points) <= 2000000
                or not np.all(np.isfinite(points))):
            raise ValueError('invalid frozen measured support points')
        self.input_count = len(points)
        if len(points) > MAX_WITNESS_POINTS:
            # Select original points, without averaging, interpolation or holes.
            indices = np.linspace(0, len(points)-1, MAX_WITNESS_POINTS, dtype=np.int64)
            self.points = points[indices].copy()
        else:
            self.points = points.copy()
        self.points.setflags(write=False)
        self.cache = {}
        self.used_sec = 0.

    def evaluate(self, base_from_tool):
        pose = rigid_transform(base_from_tool)
        key = transform_fingerprint(pose)
        if key not in self.cache:
            started = time.monotonic()
            source = self.source
            camera = source['camera']
            full_set = self.input_count == len(self.points)
            try:
                projection = validate_observation_support_projection(
                    pose @ np.asarray(source['tool_from_camera']), self.points,
                    source['center_base'], [camera[k] for k in ('fx', 'fy', 'cx', 'cy')],
                    camera['width'], camera['height'],
                    target_corners_base=source['corners_base'],
                    **source['view_limits']['effective_visibility_limits'])
                status = 'sufficient'
            except ObservationViewError as exc:
                projection = deepcopy(exc.audit)
                # No full-cloud fallback: keep unproven views eligible without
                # spending another cloud projection per pose before bounded IK.
                status = 'insufficient' if full_set else 'unknown'
            self.used_sec += time.monotonic() - started
            self.cache[key] = dict(policy=POLICY, status=status,
                pose_sha256=key, measured_source_points=self.input_count,
                evaluated_measured_points=len(self.points), full_set_evaluated=full_set,
                projection_counts_are_lower_bounds=not full_set,
                projection=projection, scheduling_only=True,
                proves_actual_ik_endpoint=False, execution_authority=False,
                requires_full_actual_path_and_endpoint_checks=True)
        return deepcopy(self.cache[key])

    def report(self):
        return dict(policy=POLICY, maximum_witness_points=MAX_WITNESS_POINTS,
            unique_pose_count=len(self.cache), used_sec=self.used_sec,
            status_counts={status: sum(row['status'] == status for row in self.cache.values())
                for status in ('sufficient', 'unknown', 'insufficient')},
            candidates_removed=0, execution_authority=False)
