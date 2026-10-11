"""Source-bound endpoint scheduling, never an IK or execution qualification."""
import hashlib
import math

import numpy as np


POLICY = 'bounded_endpoint_ik_hint_then_frozen_tool_so3_support_translation'
COVERAGE_POLICY = 'diagnostic_no_positive_ik_distance_direction_coverage_until_incumbent'
INFORMATION_COVERAGE_POLICY = 'information_first_path_distance_direction_coverage_after_best_ik_hint'


def rigid_transform(value):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.all(np.isfinite(matrix))
            or not np.allclose(matrix[3], [0., 0., 0., 1.], atol=1e-9, rtol=0.)
            or not np.allclose(matrix[:3, :3].T@matrix[:3, :3], np.eye(3), atol=1e-8, rtol=0.)
            or abs(float(np.linalg.det(matrix[:3, :3]))-1.) > 1e-8):
        raise ValueError('finite rigid tool transform required')
    return matrix


def transform_fingerprint(value):
    return hashlib.sha256(rigid_transform(value).astype('<f8').tobytes()).hexdigest()


def endpoint_schedule_metrics(reference, candidate, clearance):
    """Rank actual tool rotations, including optical-axis roll.

    The key preserves all candidates. Equal-angle families only use numerical
    rounding (1e-9 rad), then prioritize proven endpoint support clearance.
    Endpoint rotation cannot establish a small joint turn, a collision-free
    branch, target visibility along the path, or globally minimum duration.
    """
    first, last = rigid_transform(reference), rigid_transform(candidate)
    margin = float(clearance)
    if not math.isfinite(margin):
        raise ValueError('finite endpoint support margin required')
    relative = first[:3, :3].T@last[:3, :3]
    # atan2 avoids acos amplification near identical and 180-degree rotations.
    sine = .5*float(np.linalg.norm([relative[2,1]-relative[1,2],
        relative[0,2]-relative[2,0],relative[1,0]-relative[0,1]]))
    angle = math.atan2(sine,float(np.clip((np.trace(relative)-1.)*.5,-1.,1.)))
    translation = float(np.linalg.norm(last[:3, 3]-first[:3, 3]))
    return dict(tool_so3_delta_rad=angle, tool_translation_delta_m=translation,
        endpoint_support_clearance_m=margin,
        sort_key=(round(angle, 9), -margin, translation))


def diagnostic_coverage_indices(distances, optical_axes, nominal_distance_m):
    """Disperse already qualified endpoints before any full-path incumbent.

    Inputs retain their original preference order. Each round visits every
    available distance before revisiting one; nominal, near and far distances
    lead the first round. Within a distance, try a less visited optical-axis
    direction before another roll of the same direction. Rounding only joins
    numerical representations of the same family; no pose or gate is changed.
    All indices occur exactly once. This is not a reachability claim.
    """
    values = np.asarray(distances, dtype=float)
    axes = np.asarray(optical_axes, dtype=float)
    nominal = float(nominal_distance_m)
    if (values.ndim != 1 or not len(values) or axes.shape != (len(values), 3)
            or not np.all(np.isfinite(values)) or np.any(values <= 0.)
            or not np.all(np.isfinite(axes))
            or not np.allclose(np.linalg.norm(axes, axis=1), 1., atol=1e-8, rtol=0.)
            or not math.isfinite(nominal) or nominal <= 0.):
        raise ValueError('finite endpoint distances and unit optical axes required')
    groups = {}
    directions = []
    for index, (distance, axis) in enumerate(zip(values, axes)):
        groups.setdefault(round(float(distance), 9), []).append(index)
        directions.append(tuple(np.round(axis, 9)))
    distances_in_order = sorted(groups, key=lambda d: (abs(d-nominal), d))
    first = (distances_in_order[0], min(groups), max(groups))
    distance_order = tuple(dict.fromkeys((*first, *distances_in_order)))
    used, result = {}, []
    while any(groups.values()):
        for distance in distance_order:
            choices = groups[distance]
            if not choices:
                continue
            index = min(choices, key=lambda i: (used.get(directions[i], 0), i))
            choices.remove(index)
            used[directions[index]] = used.get(directions[index], 0) + 1
            result.append(index)
    return tuple(result)


class ObservationCheckSchedule:
    """Inspectable first-path order with a single live dispatch iterator.

    Wrappers inspect/reorder ``initial_order`` without advancing checks. Only
    actual dispatch consults the current request and qualified-incumbent signal.
    A first-path reorder never changes the post-incumbent preference order.
    """
    def __init__(self, original_order, indices, evidence, validate, qualified):
        self._original = tuple(original_order)
        self._indices = tuple(indices)
        if sorted(self._indices) != list(range(len(self._original))):
            raise ValueError('first-path schedule must retain all candidates once')
        self._evidence = evidence
        self._validate = validate
        self._qualified = qualified
        self._iterator = None

    @property
    def ranked_order(self):
        """Original score-ranked candidate snapshot; inspection is not dispatch."""
        return self._original

    @property
    def initial_order(self):
        return tuple(self._original[index] for index in self._indices)

    def reorder_before_incumbent(self, variants):
        if self._iterator is not None:
            raise ValueError('cannot rewrite a started observation schedule')
        indices = {id(variant): index for index, variant in enumerate(self._original)}
        order = tuple(indices[id(variant)] for variant in variants)
        if sorted(order) != list(range(len(self._original))):
            raise ValueError('family scheduling must retain every endpoint once')
        self._indices = order
        self._evidence['family_balanced_first_path_order'] = list(order)
        return self

    def __iter__(self):
        return self

    def __next__(self):
        if self._iterator is None:
            self._iterator = self._dispatch()
        return next(self._iterator)

    def _dispatch(self):
        yielded = set()
        for index in self._indices:
            self._validate()
            if self._qualified():
                self._evidence['full_path_incumbent_found'] = True
                break
            yielded.add(index)
            self._evidence['dispatched_original_indices'].append(index)
            yield self._original[index]
        self._evidence['full_path_incumbent_found'] = self._qualified()
        for index, variant in enumerate(self._original):
            if index not in yielded:
                self._validate()
                self._evidence['dispatched_original_indices'].append(index)
                yield variant
