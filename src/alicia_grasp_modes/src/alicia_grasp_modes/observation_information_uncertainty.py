"""Separate visual resolution from contact-stage execution allowances.

Adaptive contact profiles contain perception + measured endpoint residual.
The latter remains in all contact distances and physical motion contracts;
it is not additional depth noise after the actual camera pose is measured.
This helper supplies observation information predictions and new measured
coverage after successful registration. It never changes registration gates.
"""
from collections.abc import Mapping
import math
from numbers import Real


def observation_information_uncertainty_m(profiles):
    """Return the largest explicit perception term without modifying profiles.

    Missing execution fields must fail, not silently treat a combined contact
    allowance as optical resolution. Both quantities come from the same
    immutable current-source profile; no parameter or old task sample is read.
    """
    values = []
    for profile in profiles:
        def field(name):
            value = profile[name] if isinstance(profile, Mapping) else getattr(profile, name)
            if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
                raise ValueError('observation information uncertainty requires finite explicit profile fields')
            return float(value)
        try:
            combined = field('depth_uncertainty_m')
            execution = field('execution_position_error_m')
        except (AttributeError, KeyError) as exc:
            raise ValueError('observation information uncertainty profile split is missing') from exc
        perception = combined-execution
        if execution < 0. or perception <= 0.:
            raise ValueError('observation information uncertainty split is invalid')
        values.append(perception)
    if not values:
        raise ValueError('observation information uncertainty requires current-source profiles')
    return max(values)


PREDICTED_GAIN_POLICY = 'source_contact_overlap_prediction_v1'
_MISSING = object()
_BINDING_KEYS = ('request_id', 'generation', 'target_epoch', 'target_track_id', 'snapshot_stamp_ns')


def _field(value, name):
    return value.get(name, _MISSING) if isinstance(value, Mapping) else getattr(value, name, _MISSING)


def _positive_target(value):
    if (isinstance(value, bool) or not isinstance(value, Real)
            or not math.isfinite(value) or value <= 0.):
        raise ValueError('predicted gain target requires an explicit positive finite value')
    return float(value)


def observation_resolvable_gain_target_m(depth_uncertainty_m, predicted_gain_target_m=None):
    """Keep the task's desired contact band above the current visual noise.

    A short contact overlap is a useful task target, but cannot lower measured
    perception uncertainty. Neither quantity includes following error here.
    """
    noise = depth_uncertainty_m
    if (isinstance(noise, bool) or not isinstance(noise, Real)
            or not math.isfinite(noise) or noise < 0.):
        raise ValueError('predicted gain requires finite nonnegative visual uncertainty')
    target = 0. if predicted_gain_target_m is None else _positive_target(predicted_gain_target_m)
    return max(float(noise), target)


def observation_predicted_gain_target_m(profiles, snapshot):
    """Desired exposure at the source contact-band scale, never an error bound.

    The profiles and temporal repeatability must belong to the same frozen
    snapshot. Missing temporal evidence keeps the legacy conservative path. A
    computed zero requires the existing MAD's finite residual support count;
    it does not establish zero measurement error. Explicit malformed fields or
    contradictory stamps are errors.
    This does not assert that a newly exposed band will be measurable or usable;
    optical, registration, actual novelty and all physical gates stay separate.
    """
    rows = tuple(profiles)
    targets, incomplete = [], not rows
    for profile in rows:
        value = _field(profile, 'contact_overlap_requirement_m')
        if value is _MISSING:
            incomplete = True
        else:
            targets.append(_positive_target(value))
    quality = _field(snapshot, 'quality')
    repeatability = _field(quality, 'depth_repeatability_m')
    count = _field(quality, 'fused_frames')
    residual_count = _field(quality, 'depth_repeatability_sample_count')
    stamps = _field(snapshot, 'sample_stamp_ns')
    stamp = _field(snapshot, 'stamp_ns')
    ok = _field(snapshot, 'ok')
    if ok is not _MISSING and ok is not True:
        raise ValueError('predicted gain target requires a valid source snapshot')
    if repeatability is not _MISSING:
        if (isinstance(repeatability, bool) or not isinstance(repeatability, Real)
                or not math.isfinite(repeatability) or repeatability < 0.):
            raise ValueError('invalid source temporal repeatability')
    if count is not _MISSING and (type(count) is not int or count < 0):
        raise ValueError('invalid source fused frame count')
    if residual_count is not _MISSING:
        if type(residual_count) is not int or residual_count < 0 or residual_count == 1:
            raise ValueError('invalid source temporal residual count')
        if residual_count >= 2 and count is not _MISSING and count < 2:
            raise ValueError('temporal residual support requires repeated source frames')
    if stamp is not _MISSING and (type(stamp) is not int or stamp <= 0):
        raise ValueError('invalid source snapshot stamp')
    if stamps is not _MISSING:
        if (not isinstance(stamps, (list, tuple))
                or any(type(ns) is not int or ns <= 0 for ns in stamps)
                or list(stamps) != sorted(set(stamps))):
            raise ValueError('invalid source sample stamps')
        if stamps and ((stamp is not _MISSING and stamps[-1] != stamp)
                or (count is not _MISSING and len(stamps) != count)):
            raise ValueError('source temporal evidence does not match snapshot')
    supported_zero = residual_count is not _MISSING and residual_count >= 2
    if (incomplete or repeatability is _MISSING
            or (repeatability == 0. and not supported_zero)
            or count is _MISSING or count < 2 or stamp is _MISSING
            or stamps is _MISSING or not stamps):
        return None
    return max(targets)


def _source_binding(value):
    if not isinstance(value, Mapping) or any(key not in value for key in _BINDING_KEYS):
        raise ValueError('predicted gain source binding is missing')
    result = {key: value[key] for key in _BINDING_KEYS}
    if (any(type(result[key]) is not int or result[key] < 0
            for key in ('request_id', 'generation', 'target_epoch', 'snapshot_stamp_ns'))
            or result['request_id'] <= 0 or result['snapshot_stamp_ns'] <= 0
            or not isinstance(result['target_track_id'], str) or not result['target_track_id']):
        raise ValueError('predicted gain source binding is invalid')
    return result


def observation_predicted_gain_evidence(profiles, snapshot, source_binding):
    """Freeze the task target inside the existing measured-deficit trigger."""
    rows = tuple(profiles)
    target = observation_predicted_gain_target_m(rows, snapshot)
    if target is None:
        return None
    binding = _source_binding(source_binding)
    if binding['snapshot_stamp_ns'] != _field(snapshot, 'stamp_ns'):
        raise ValueError('predicted gain profile source differs from trigger')
    quality = _field(snapshot, 'quality')
    return dict(policy=PREDICTED_GAIN_POLICY, source_binding=binding,
        predicted_gain_target_m=target,
        sample_stamp_ns=list(_field(snapshot, 'sample_stamp_ns')),
        depth_repeatability_m=float(_field(quality, 'depth_repeatability_m')),
        fused_frames=_field(quality, 'fused_frames'),
        depth_repeatability_sample_count=(0 if _field(quality, 'depth_repeatability_sample_count') is _MISSING
            else _field(quality, 'depth_repeatability_sample_count')),
        contact_overlap_requirements_m=[float(_field(p, 'contact_overlap_requirement_m')) for p in rows],
        prediction_only=True, measured_registration_required=True,
        contact_or_motion_authorized=False)


def bound_observation_predicted_gain_target_m(evidence, source_binding):
    """Validate the typed target against its unchanged original trigger source."""
    binding = _source_binding(source_binding)
    if (not isinstance(evidence, Mapping) or evidence.get('policy') != PREDICTED_GAIN_POLICY
            or evidence.get('source_binding') != binding
            or evidence.get('prediction_only') is not True
            or evidence.get('measured_registration_required') is not True
            or evidence.get('contact_or_motion_authorized') is not False):
        raise ValueError('predicted gain evidence policy or source binding is invalid')
    values = evidence.get('contact_overlap_requirements_m')
    if not isinstance(values, (list, tuple)) or not values:
        raise ValueError('predicted gain contact requirements are missing')
    snapshot = dict(stamp_ns=binding['snapshot_stamp_ns'],
        sample_stamp_ns=evidence.get('sample_stamp_ns'),
        quality=dict(fused_frames=evidence.get('fused_frames'),
            depth_repeatability_m=evidence.get('depth_repeatability_m'),
            depth_repeatability_sample_count=evidence.get('depth_repeatability_sample_count', 0)))
    expected = observation_predicted_gain_target_m(
        [dict(contact_overlap_requirement_m=value) for value in values], snapshot)
    target = _positive_target(evidence.get('predicted_gain_target_m'))
    if expected is None or expected != target:
        raise ValueError('predicted gain target differs from source contact requirements')
    return target
