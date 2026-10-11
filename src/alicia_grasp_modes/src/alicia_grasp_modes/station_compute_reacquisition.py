"""An identity-only wait at a measured observation station, never old geometry.

The producer may offer this contract only with an independently qualified fresh
READY frame. Consumers must join that frame's actual object, mask and status
before its geometry deadline to register it. Failed inputs never renew it.
Station ownership, monotonic expiry, full current measurement validation and
atomic cancellation of executable plans remain responsibilities of callers.
"""
from copy import deepcopy
import math


POLICY = 'qualified_observation_station_compute_wait'
IDENTITY_WAIT_NS = 5_000_000_000
BINDING_KEYS = frozenset((
    'selection', 'lease_id', 'session_id', 'view_plan_id', 'view_index',
    'target_epoch', 'target_track_id', 'control_epoch_ns', 'phase_id',
    'phase_started_ns', 'phase_reference_source_ns', 'reference_binding',
    'source_id', 'camera_configuration_sha256', 'config_sha256', 'station_q_rad'))


def _positive(value):
    if type(value) is not int or value <= 0:
        raise ValueError('station_compute_wait_invalid_integer')
    return value


def _binding(value, source_ns):
    if not isinstance(value, dict) or set(value) != BINDING_KEYS:
        raise ValueError('station_compute_wait_invalid_binding_keys')
    selected = value['selection']
    if (not isinstance(selected, dict)
            or set(selected) != {'mode', 'strategy', 'generation', 'stamp_ns'}
            or selected['mode'] != 'unknown' or selected['strategy'] != 'observe_return'
            or type(selected['generation']) is not int or selected['generation'] < 0
            or not _positive(selected['stamp_ns']) < source_ns):
        raise ValueError('station_compute_wait_invalid_selection')
    for key in ('view_index', 'target_epoch', 'control_epoch_ns', 'phase_id',
                'phase_started_ns', 'phase_reference_source_ns'):
        _positive(value[key])
    if (not selected['stamp_ns'] < value['phase_reference_source_ns'] <= source_ns
            or not value['control_epoch_ns'] < source_ns
            or not value['phase_started_ns'] < source_ns):
        raise ValueError('station_compute_wait_invalid_source_order')
    for key in ('lease_id', 'session_id', 'view_plan_id', 'target_track_id', 'reference_binding'):
        if not isinstance(value[key], str) or not value[key].strip():
            raise ValueError('station_compute_wait_missing_binding')
    for key in ('source_id', 'camera_configuration_sha256', 'config_sha256'):
        digest = value[key]
        if (not isinstance(digest, str) or len(digest) != 64
                or any(char not in '0123456789abcdef' for char in digest)):
            raise ValueError('station_compute_wait_invalid_digest')
    q = value['station_q_rad']
    if (not isinstance(q, list) or len(q) != 6
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in q)):
        raise ValueError('station_compute_wait_invalid_station_joints')
    return deepcopy(value)


def build_contract(*, source_ns, maximum_frame_age_sec, session_deadline_ns,
                   phase_deadline_ns, binding):
    """Derive absolute deadlines from the measured source, never receipt time."""
    _positive(source_ns)
    if (type(maximum_frame_age_sec) not in (int, float)
            or not math.isfinite(maximum_frame_age_sec)
            or not 0. < maximum_frame_age_sec <= .8):
        raise ValueError('station_compute_wait_invalid_frame_age')
    age_ns = round(maximum_frame_age_sec * 1e9)
    if age_ns <= 0:
        raise ValueError('station_compute_wait_invalid_frame_age')
    session_deadline_ns = _positive(session_deadline_ns)
    phase_deadline_ns = _positive(phase_deadline_ns)
    identity_deadline_ns = min(source_ns + IDENTITY_WAIT_NS,
                               session_deadline_ns, phase_deadline_ns)
    if identity_deadline_ns <= source_ns:
        raise ValueError('station_compute_wait_owner_already_expired')
    return dict(version=1, policy=POLICY, source_ns=source_ns,
        maximum_frame_age_sec=maximum_frame_age_sec,
        geometry_deadline_ns=source_ns + age_ns,
        identity_deadline_ns=identity_deadline_ns,
        session_deadline_ns=session_deadline_ns, phase_deadline_ns=phase_deadline_ns,
        binding=_binding(binding, source_ns), metric_geometry_authority=False,
        motion_authorized=False, reference_refreshed=False)


def validate_contract(value, *, binding, now_ns, source_ns=None):
    """Validate a registered identity wait, optionally with a new fresh source.

    ARMED means only that the reference clock has not expired; it grants no
    authority to execute. PENDING explicitly cannot use the old geometry.
    This function never treats an expired input as recovery evidence.
    """
    try:
        expected = build_contract(source_ns=value['source_ns'],
            maximum_frame_age_sec=value['maximum_frame_age_sec'],
            session_deadline_ns=value['session_deadline_ns'],
            phase_deadline_ns=value['phase_deadline_ns'], binding=binding)
        _binding(value['binding'], expected['source_ns'])
    except (KeyError, TypeError, AttributeError) as exc:
        raise ValueError('station_compute_wait_malformed_contract') from exc
    if (not isinstance(value, dict) or set(value) != set(expected) or value != expected
            or type(value.get('version')) is not int
            or any(type(value.get(k)) is not int for k in (
                'source_ns', 'geometry_deadline_ns', 'identity_deadline_ns',
                'session_deadline_ns', 'phase_deadline_ns'))
            or any(value.get(k) is not False for k in (
                'metric_geometry_authority', 'motion_authorized', 'reference_refreshed'))):
        raise ValueError('station_compute_wait_contract_changed')
    now_ns = _positive(now_ns)
    if not expected['source_ns'] <= now_ns < expected['identity_deadline_ns']:
        raise ValueError('station_compute_wait_expired_or_future')
    if source_ns is not None:
        age_ns = expected['geometry_deadline_ns'] - expected['source_ns']
        if (not expected['source_ns'] < _positive(source_ns) <= now_ns
                or now_ns - source_ns >= age_ns):
            raise ValueError('station_compute_wait_recovery_source_not_fresh')
    return 'ARMED' if now_ns < expected['geometry_deadline_ns'] else 'PENDING'


def validate_registration(value, *, binding, now_ns):
    """Late receipts cannot create a pending identity or revive a retired one."""
    if validate_contract(value, binding=binding, now_ns=now_ns) != 'ARMED':
        raise ValueError('station_compute_wait_registration_too_late')
    return deepcopy(value)


def validate_recovery_parent(value, *, parent, chain=None, binding, now_ns, source_ns=None):
    """Bridge a missed READY receipt without adopting its geometry or clock.

    ``value`` must be the caller's already registered, immutable contract.
    Intervening producer contracts are historical identity evidence only. A
    consumer still has its original absolute/monotonic expiry, even if newer
    contracts would permit a later deadline. With no source this validates a
    pending notification; supplying one additionally requires a new fresh input.
    """
    validate_contract(value, binding=binding, now_ns=now_ns, source_ns=source_ns)
    validate_contract(parent, binding=binding, now_ns=now_ns)
    if source_ns is not None and not parent['source_ns'] < source_ns:
        raise ValueError('station_compute_wait_recovery_parent_not_before_source')
    if parent == value and chain is None:
        return
    if (not isinstance(chain, list) or not 1 <= len(chain) <= 32
            or chain[-1] != parent or not any(row == value for row in chain)):
        raise ValueError('station_compute_wait_recovery_chain_unbound')
    previous = None
    for row in chain:
        try:
            # Validate historical contracts at their own source time. They
            # cannot replace the actual consumer's registration or deadline.
            validate_contract(row, binding=binding, now_ns=row['source_ns'])
        except (KeyError, TypeError) as exc:
            raise ValueError('station_compute_wait_recovery_chain_malformed') from exc
        if (row['maximum_frame_age_sec'] != value['maximum_frame_age_sec']
                or row['session_deadline_ns'] != value['session_deadline_ns']
                or row['phase_deadline_ns'] != value['phase_deadline_ns']
                or row['source_ns'] > now_ns
                or (source_ns is not None and row['source_ns'] >= source_ns)
                or (previous is not None and not previous['source_ns'] < row['source_ns']
                    < previous['identity_deadline_ns'])):
            raise ValueError('station_compute_wait_recovery_chain_discontinuous')
        previous = row


SUPPORT_GAP = 'station_support_gap'
_SUPPORT_GAP_REASONS = frozenset((
    'insufficient_valid_depth', 'insufficient_support_depth',
    'support_plane_not_found', 'support_plane_inlier_ratio_low',
    'support_plane_residual_too_large', 'insufficient_support_spatial_coverage',
    'locked_support_plane_no_longer_visible'))


def build_station_support_gap(contract, source_ns, reason, first_source_ns=None):
    """Describe rejected support evidence, bounded by the original 0.8 s clock.

    A caller must already own the complete parent registration. This receipt
    neither qualifies the failed source nor extends the station's identity.
    """
    try:
        validate_contract(contract, binding=contract['binding'], now_ns=contract['source_ns'])
        source_ns = _positive(source_ns)
        first_source_ns = source_ns if first_source_ns is None else _positive(first_source_ns)
        deadline = min(contract[k] for k in ('geometry_deadline_ns',
            'identity_deadline_ns', 'phase_deadline_ns', 'session_deadline_ns'))
        if (reason not in _SUPPORT_GAP_REASONS
                or not contract['source_ns'] < first_source_ns <= source_ns < deadline):
            raise ValueError('station_support_gap_source_or_reason_invalid')
        return dict(version=1, policy=SUPPORT_GAP, parent_source_ns=contract['source_ns'],
            source_ns=source_ns, first_source_ns=first_source_ns,
            source_id=contract['binding']['source_id'], reason=reason, deadline_ns=deadline,
            metric_geometry_authority=False, motion_authorized=False, reference_refreshed=False)
    except (KeyError, TypeError, AttributeError) as exc:
        raise ValueError('station_support_gap_malformed') from exc


def validate_station_support_gap(receipt, contract, now_ns, source_ns=None):
    """Validate the same bounded gap on pending and full recovery fan-in.

    ``source_ns`` names a new, independently measured recovery source; absence
    validates only the non-authorizing pending receipt. No receive-time renewal.
    """
    try:
        expected = build_station_support_gap(contract, receipt['source_ns'],
            receipt['reason'], receipt['first_source_ns'])
        if (not isinstance(receipt, dict) or set(receipt) != set(expected)
                or receipt != expected or type(receipt.get('version')) is not int
                or any(type(receipt.get(k)) is not int for k in
                    ('parent_source_ns', 'source_ns', 'first_source_ns', 'deadline_ns'))
                or any(receipt.get(k) is not False for k in
                    ('metric_geometry_authority', 'motion_authorized', 'reference_refreshed'))):
            raise ValueError('station_support_gap_receipt_changed')
        now_ns = _positive(now_ns)
        validate_contract(contract, binding=contract['binding'], now_ns=now_ns)
        if not receipt['source_ns'] <= now_ns < receipt['deadline_ns']:
            raise ValueError('station_support_gap_expired_or_future')
        if source_ns is not None:
            if not receipt['source_ns'] < _positive(source_ns) <= now_ns:
                raise ValueError('station_support_gap_recovery_source_invalid')
            validate_contract(contract, binding=contract['binding'], now_ns=now_ns,
                source_ns=source_ns)
        return deepcopy(expected)
    except (KeyError, TypeError, AttributeError) as exc:
        raise ValueError('station_support_gap_malformed') from exc
