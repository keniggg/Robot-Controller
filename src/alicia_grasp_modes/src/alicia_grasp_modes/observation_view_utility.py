"""Observation search preferences, never evidence that an unseen side exists.

Use excess projected exposure over measured noise and one pixel of sampling
uncertainty. The bounded score saturates: large turns cannot earn unlimited
credit by exposing more of an already visible side. All geometry, registration
and physical path checks remain independent.
"""
import math
from numbers import Real
from .observation_information_uncertainty import observation_resolvable_gain_target_m


POLICY = 'predicted_resolvable_information_per_motion_v1'
TASK_TARGET_POLICY = 'predicted_contact_band_exposure_per_motion_v1'


def _finite(value, name, positive=False):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError('invalid ' + name)
    result = float(value)
    if not math.isfinite(result) or result < 0. or (positive and result == 0.):
        raise ValueError('invalid ' + name)
    return result


def predicted_information_utility(gain_m, uncertainty_m, depth_m, focal_px, *, predicted_gain_target_m=None):
    """Rank excess exposure over either legacy noise or a typed task target.

    A task target changes a search preference only. The original noise is
    retained in the audit, and the task score does not certify resolvability.
    """
    gain = _finite(gain_m, 'predicted gain')
    noise = _finite(uncertainty_m, 'depth uncertainty')
    depth = _finite(depth_m, 'optical depth', positive=True)
    focal = _finite(focal_px, 'focal length', positive=True)
    gain_px, noise_px = gain * focal / depth, noise * focal / depth
    target = (noise if predicted_gain_target_m is None else
        _finite(predicted_gain_target_m, 'predicted gain target', positive=True))
    target_px = target * focal / depth
    effective_target = observation_resolvable_gain_target_m(noise, predicted_gain_target_m)
    effective_target_px = effective_target * focal / depth
    denominator = gain_px+effective_target_px+1.
    if not all(math.isfinite(v) for v in (gain_px, noise_px, target_px, denominator)):
        raise ValueError('projected information is nonfinite')
    utility = max(0., gain_px-effective_target_px) / denominator
    prediction = ({} if predicted_gain_target_m is None else dict(
        predicted_gain_target_m=target, predicted_gain_target_px=target_px,
        certifies_new_view_perception=False))
    return dict(policy=POLICY if predicted_gain_target_m is None else TASK_TARGET_POLICY,
                utility=utility, gain_px=gain_px, noise_px=noise_px, **prediction,
                effective_gain_target_m=effective_target,
                effective_gain_target_px=effective_target_px,
                resolution_policy='max_current_visual_uncertainty_and_task_target',
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
