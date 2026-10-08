"""Observation search preferences, never evidence that an unseen side exists.

Use excess projected exposure over measured noise and one pixel of sampling
uncertainty. The bounded score saturates: large turns cannot earn unlimited
credit by exposing more of an already visible side. All geometry, registration
and physical path checks remain independent.
"""
import math
from numbers import Real


POLICY = 'predicted_resolvable_information_per_motion_v1'


def _finite(value, name, positive=False):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError('invalid ' + name)
    result = float(value)
    if not math.isfinite(result) or result < 0. or (positive and result == 0.):
        raise ValueError('invalid ' + name)
    return result


def predicted_information_utility(gain_m, uncertainty_m, depth_m, focal_px):
    gain = _finite(gain_m, 'predicted gain')
    noise = _finite(uncertainty_m, 'depth uncertainty')
    depth = _finite(depth_m, 'optical depth', positive=True)
    focal = _finite(focal_px, 'focal length', positive=True)
    gain_px, noise_px = gain * focal / depth, noise * focal / depth
    denominator = gain_px+noise_px+1.
    if not all(math.isfinite(v) for v in (gain_px, noise_px, denominator)):
        raise ValueError('projected information is nonfinite')
    utility = max(0., gain_px-noise_px) / denominator
    return dict(policy=POLICY, utility=utility, gain_px=gain_px, noise_px=noise_px,
                optical_depth_m=depth, focal_px=focal, sampling_uncertainty_px=1.,
                prediction_only=True, probability_of_success=False,
                measured_registration_required=True, execution_authority=False)


def information_motion_rank(utility, motion_cost, max_delta=0., path_cost=0.):
    """Compare bounded utility per (one unit + nonnegative motion cost).

    Before strict planning motion_cost is an explicitly labelled geometric or
    joint-angle proxy. Afterwards it is the screened duration in seconds.
    Neither the score nor a missing IK hint establishes reachability.
    """
    benefit = _finite(utility, 'information utility')
    if benefit > 1.:
        raise ValueError('information utility exceeds one')
    motion = _finite(motion_cost, 'motion cost')
    delta = _finite(max_delta, 'joint delta')
    path = _finite(path_cost, 'joint path cost')
    cost = (1.+motion)/benefit if benefit > 0. else float('inf')
    return (cost, motion, delta, path, -benefit)
