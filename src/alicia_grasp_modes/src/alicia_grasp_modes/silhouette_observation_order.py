"""Source-bound endpoint scheduling, never an IK or execution qualification."""
import hashlib
import math

import numpy as np


POLICY = 'bounded_endpoint_ik_hint_then_frozen_tool_so3_support_translation'


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
