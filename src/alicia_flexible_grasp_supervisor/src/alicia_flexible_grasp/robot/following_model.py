"""Frozen source-bias arithmetic, with no execution or feedback authority.

Legacy callers retain the raw SDK/encoder gate. A mapped caller must separately
authenticate a live physical-reference lease and both checked trajectories.
This module neither changes encoder feedback nor renews that lease.
"""
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import numbers
import re

import numpy as np


ARM_NAMES = tuple('Joint%d' % i for i in range(1, 7))
SDK_QUANTUM_RAD = 2 * math.pi / 4096
MAX_FOLLOWING_ERROR_RAD = .035
MAX_SOURCE_BIAS_RAD = .09


class FollowingModelError(ValueError):
    pass


def _vector(value):
    try:
        items = tuple(value)
    except TypeError as exc:
        raise FollowingModelError('FOLLOWING_VECTOR_INVALID') from exc
    if (len(items) != 6 or any(isinstance(v, (bool, np.bool_))
            or not isinstance(v, numbers.Real) or not math.isfinite(v) for v in items)):
        raise FollowingModelError('FOLLOWING_VECTOR_INVALID')
    return np.asarray(items, dtype=float)


def _positive_stamp(value):
    return type(value) is int and value > 0


@dataclass(frozen=True)
class SourceBiasModel:
    binding_id: str
    model_sha256: str
    epoch_ns: int
    joint_names: tuple
    source_sdk_stamp_ns: int
    source_accepted_stamp_ns: int
    source_sdk_positions_rad: tuple
    source_accepted_positions_rad: tuple
    bias_rad: tuple
    source_bias_limit_rad: tuple

    @property
    def digest(self):
        validate_source_bias_model(self)
        return hashlib.sha256(json.dumps(asdict(self), sort_keys=True,
            separators=(',', ':'), allow_nan=False).encode('utf-8')).hexdigest()


def validate_source_bias_model(model, sample=None):
    """Validate immutable provenance; freshness/ownership remain live gates.

When supplied, a measured sample must belong to the same model and epoch and
cannot precede the frozen source. Synthetic future poses must not be passed as
measured samples. This validator deliberately grants no execution permission.
"""
    if type(model) is not SourceBiasModel:
        raise FollowingModelError('FOLLOWING_MODEL_TYPE_INVALID')
    if (not isinstance(model.binding_id, str) or not model.binding_id
            or len(model.binding_id) > 256
            or not isinstance(model.model_sha256, str)
            or re.fullmatch(r'[0-9a-f]{64}', model.model_sha256) is None
            or type(model.joint_names) is not tuple or model.joint_names != ARM_NAMES
            or not all(_positive_stamp(v) for v in (model.epoch_ns,
                model.source_sdk_stamp_ns, model.source_accepted_stamp_ns))
            or min(model.source_sdk_stamp_ns, model.source_accepted_stamp_ns) < model.epoch_ns):
        raise FollowingModelError('FOLLOWING_MODEL_SOURCE_INVALID')
    if any(type(v) is not tuple for v in (model.source_sdk_positions_rad,
            model.source_accepted_positions_rad, model.bias_rad, model.source_bias_limit_rad)):
        raise FollowingModelError('FOLLOWING_MODEL_MUTABLE_VECTOR')
    sdk, actual, bias = [_vector(v) for v in (model.source_sdk_positions_rad,
        model.source_accepted_positions_rad, model.bias_rad)]
    cap = _vector(model.source_bias_limit_rad)
    if (np.any(cap <= 0.) or np.any(cap > MAX_SOURCE_BIAS_RAD)
            or np.any(np.abs(bias) > cap)):
        raise FollowingModelError('FOLLOWING_SOURCE_BIAS_OUTSIDE_CONTRACT')
    counts = np.rint(sdk / SDK_QUANTUM_RAD + 2048)
    if (np.any(counts < 0) or np.any(counts > 4095)
            or np.max(np.abs(sdk - (counts-2048)*SDK_QUANTUM_RAD)) > 1e-10
            or np.any(actual < -math.pi) or np.any(actual > math.pi-SDK_QUANTUM_RAD)
            or not np.allclose(bias, sdk-actual, rtol=0., atol=1e-12)):
        raise FollowingModelError('FOLLOWING_MODEL_SOURCE_BIAS_MISMATCH')
    if sample is not None:
        if not isinstance(sample, dict):
            raise FollowingModelError('FOLLOWING_SAMPLE_INVALID')
        if (sample.get('model_sha256') != model.model_sha256
                or sample.get('epoch_ns') != model.epoch_ns
                or tuple(sample.get('joint_names', ())) != ARM_NAMES):
            raise FollowingModelError('FOLLOWING_MODEL_BINDING_MISMATCH')
        for key, source_stamp, source_positions, position_key in (
                ('sdk_stamp_ns', model.source_sdk_stamp_ns,
                    sdk, 'sdk_positions_rad'),
                ('accepted_stamp_ns', model.source_accepted_stamp_ns,
                    actual, 'accepted_positions_rad')):
            stamp = sample.get(key)
            if not _positive_stamp(stamp) or stamp < source_stamp:
                raise FollowingModelError('FOLLOWING_SAMPLE_PRECEDES_SOURCE')
            positions = _vector(sample.get(position_key, ()))
            if stamp == source_stamp and not np.array_equal(positions, source_positions):
                raise FollowingModelError('FOLLOWING_SOURCE_STAMP_REUSED')
    return model


def source_bias_model(sample, *, binding_id, source_bias_limit_rad):
    """Freeze one real source pair; this does not qualify a new episode."""
    if not isinstance(sample, dict):
        raise FollowingModelError('FOLLOWING_SAMPLE_INVALID')
    sdk = _vector(sample.get('sdk_positions_rad', ()))
    actual = _vector(sample.get('accepted_positions_rad', ()))
    model = SourceBiasModel(binding_id=binding_id,
        model_sha256=sample.get('model_sha256'), epoch_ns=sample.get('epoch_ns'),
        joint_names=tuple(sample.get('joint_names', ())),
        source_sdk_stamp_ns=sample.get('sdk_stamp_ns'),
        source_accepted_stamp_ns=sample.get('accepted_stamp_ns'),
        source_sdk_positions_rad=tuple(sdk), source_accepted_positions_rad=tuple(actual),
        bias_rad=tuple(sdk-actual), source_bias_limit_rad=tuple(_vector(source_bias_limit_rad)))
    return validate_source_bias_model(model)


def source_bias_model_record(model):
    """Serializable arithmetic provenance, not a lease or capability."""
    return asdict(validate_source_bias_model(model))


def source_bias_model_from_record(record):
    if not isinstance(record, dict):
        raise FollowingModelError('FOLLOWING_MODEL_RECORD_INVALID')
    values = dict(record)
    try:
        for key in ('joint_names', 'source_sdk_positions_rad',
                'source_accepted_positions_rad', 'bias_rad', 'source_bias_limit_rad'):
            values[key] = tuple(values[key])
        return validate_source_bias_model(SourceBiasModel(**values))
    except (KeyError, TypeError) as exc:
        raise FollowingModelError('FOLLOWING_MODEL_RECORD_INVALID') from exc


def following_evidence(sdk, actual, *, following_model=None):
    sdk, actual = _vector(sdk), _vector(actual)
    if following_model is None:
        bias = sdk-actual
        residual = bias.copy()
        domain, digest, binding = 'legacy_raw_sdk', '', ''
    else:
        model = validate_source_bias_model(following_model)
        bias = np.asarray(model.bias_rad)
        residual = (sdk-bias)-actual
        domain, digest, binding = 'mapped_physical', model.digest, model.binding_id
    return dict(domain=domain, binding_id=binding, source_bias_sha256=digest,
        frozen_bias_rad=bias.tolist(), raw_sdk_minus_encoder_rad=(sdk-actual).tolist(),
        sdk_minus_encoder_rad=(sdk-actual).tolist(),
        physical_reference_minus_encoder_rad=residual.tolist(),
        execution_authorized=False, future_following_verified=False)


def following_offset(sdk, actual, *, following_model=None,
                     max_error_rad=MAX_FOLLOWING_ERROR_RAD):
    """Return the model offset after the unchanged residual bound passes.

The model's bias is never re-estimated from a later error. Under a mapping,
20 counts of physical error remain 20 counts even when raw error is 33.
"""
    if (isinstance(max_error_rad, bool) or not isinstance(max_error_rad, numbers.Real)
            or not math.isfinite(max_error_rad)
            or not 0 < max_error_rad <= MAX_FOLLOWING_ERROR_RAD):
        raise FollowingModelError('FOLLOWING_TOLERANCE_CANNOT_BE_RELAXED')
    evidence = following_evidence(sdk, actual, following_model=following_model)
    if max(abs(v) for v in evidence['physical_reference_minus_encoder_rad']) > max_error_rad:
        raise FollowingModelError('FOLLOWING_OUTSIDE_CONTRACT')
    return np.asarray(evidence['frozen_bias_rad'], dtype=float)
