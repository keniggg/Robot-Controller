"""An identity-only wait at a measured observation station, never old geometry.

The producer may offer this contract only with an independently qualified fresh
READY frame. Consumers must join that frame's actual object, mask and status
before its geometry deadline to register it. Failed inputs never renew it.
Station ownership, monotonic expiry, full current measurement validation and
atomic cancellation of executable plans remain responsibilities of callers.
"""
from alicia_grasp_modes.runtime_gate_policy import (SOURCE_MAX_AGE_SEC, STATION_IDENTITY_WAIT_NS)
from copy import deepcopy
import math


POLICY = 'qualified_observation_station_compute_wait'
IDENTITY_WAIT_NS = STATION_IDENTITY_WAIT_NS
RECOVERY_CHAIN_LIMIT = 64
SEED_PENDING_STATE = 'station_measurement_pending'
SEED_PENDING_REASON = 'station_seed_measurement_insufficient'
FRAME_PENDING_REASON = 'station_current_measurement_rejected'
MEASUREMENT_REJECTION_KINDS = ('seed_measurement_insufficient', 'source_component_overlap')
BINDING_KEYS = frozenset((
    'selection', 'lease_id', 'session_id', 'view_plan_id', 'view_index',
    'target_epoch', 'target_track_id', 'control_epoch_ns', 'phase_id',
    'phase_started_ns', 'phase_reference_source_ns', 'reference_binding',
    'source_id', 'camera_configuration_sha256', 'config_sha256', 'station_q_rad'))


def validate_station_pending_input(value, *, contract, now_ns,
                                   minimum_input_source_ns, recovery_chain=None):
    """An actual unqualified fragment may request bounded nonmetric fan-in.

    The receiver must compare the hash/descriptor to its independently received
    false object or empty mask. This marker cannot explain a bracket gap,
    qualify a source, renew a deadline or authorize publication/movement.
    """
    try:
        binding = contract['binding']
        validate_contract(contract, binding=binding, now_ns=now_ns)
        parent, ns = value['parent_contract'], _positive(value['source_ns'])
        validate_recovery_parent(contract, parent=parent, chain=recovery_chain,
            binding=binding, now_ns=now_ns)
        common = {'version','kind','source_ns','input_kind','parent_contract',
                  'source_qualified','identity_verified','metric_geometry_authority',
                  'motion_authorized','reference_refreshed'}
        if (not isinstance(value, dict) or type(value.get('version')) is not int
                or value['version'] != 1 or value.get('kind') != 'unqualified_station_input'
                or not max(parent['source_ns'], _positive(minimum_input_source_ns)) < ns <= now_ns
                or ns >= min(parent['identity_deadline_ns'], contract['identity_deadline_ns'])
                or any(value.get(key) is not False for key in ('source_qualified',
                    'identity_verified','metric_geometry_authority','motion_authorized','reference_refreshed'))):
            raise ValueError('station_pending_input_owner_or_role_changed')
        if value.get('input_kind') == 'object' and set(value) == common | {'object_sha256'}:
            digests = [value['object_sha256']]
        elif value.get('input_kind') == 'mask' and set(value) == common | {'mask'}:
            mask = value['mask']
            if (not isinstance(mask, dict) or set(mask) != {'sha256','shape','frame_id'}
                    or not isinstance(mask['shape'], list) or len(mask['shape']) != 2
                    or any(type(v) is not int or v <= 0 for v in mask['shape'])
                    or not isinstance(mask['frame_id'], str) or not mask['frame_id']):
                raise ValueError('station_pending_input_mask_malformed')
            digests = [mask['sha256']]
        else:
            raise ValueError('station_pending_input_fragment_changed')
        if any(not isinstance(v, str) or len(v) != 64
               or any(c not in '0123456789abcdef' for c in v) for v in digests):
            raise ValueError('station_pending_input_hash_malformed')
        return deepcopy(value)
    except (KeyError, TypeError, AttributeError) as exc:
        raise ValueError('station_pending_input_malformed') from exc


def _station_pending_envelope(status, contract, now_ns, status_reference_binding):
    """Join an already source-validated status to an existing registration.

    Callers compute status_reference_binding with the original source-envelope
    validator, not a cached/latest status. This helper grants no metric input,
    READY enrollment, computation publication or movement authority.
    """
    try:
        binding = contract['binding']
        validate_contract(contract, binding=binding, now_ns=now_ns)
        ns = _positive(status['source_stamp_ns'])
        if (not isinstance(status, dict) or ns > now_ns
                or status.get('mode') != 'unknown'
                or status.get('generation') != binding['selection']['generation']
                or status.get('selection_stamp_ns') != binding['selection']['stamp_ns']
                or status.get('target_lost') is not False
                or status.get('first_loss') is not None
                or not isinstance(status_reference_binding, str)
                or status_reference_binding != binding['reference_binding']):
            raise ValueError('station_pending_source_owner_changed')
        return binding, ns
    except (KeyError, TypeError, AttributeError) as exc:
        raise ValueError('station_pending_source_malformed') from exc


def validate_seed_insufficiency(value, contract, now_ns, *, recovery_source_ns=None):
    """A current raw-only seed shortage; never a qualified measurement."""
    fields = {'version', 'parent_source_ns', 'source_ns', 'stage', 'region_index',
        'region_mask_sha256', 'region_shape', 'measured_pixels', 'eroded_pixels',
        'reliable_eroded_pixels', 'original_top_pixels', 'trusted_top_pixels',
        'trusted_side_pixels', 'trusted_core_pixels', 'minimum_core_pixels',
        'minimum_measured_points', 'upper_surface_band_used'}
    if type(value) is not dict or set(value) != fields:
        raise ValueError('station_seed_insufficiency_schema_invalid')
    validate_contract(contract, binding=contract['binding'], now_ns=now_ns)
    counts = ('region_index', 'measured_pixels', 'eroded_pixels', 'reliable_eroded_pixels',
        'original_top_pixels', 'trusted_top_pixels', 'trusted_side_pixels', 'trusted_core_pixels',
        'minimum_core_pixels', 'minimum_measured_points')
    if (type(value['version']) is not int or value['version'] != 1
            or type(value['parent_source_ns']) is not int
            or value['parent_source_ns'] != contract['source_ns']
            or type(value['source_ns']) is not int
            or not contract['source_ns'] < value['source_ns'] <= now_ns
            or value['stage'] != 'raw_only_extra_core'
            or any(type(value[k]) is not int or value[k] < 0 for k in counts)
            or value['minimum_core_pixels'] != 25
            or not 0 < value['minimum_measured_points'] <= value['measured_pixels']
            or not 0 <= value['trusted_core_pixels'] < 25
            or not value['reliable_eroded_pixels'] <= value['eroded_pixels'] <= value['measured_pixels']
            or any(value[k] > value['eroded_pixels'] for k in
                   ('original_top_pixels', 'trusted_top_pixels', 'trusted_side_pixels', 'trusted_core_pixels'))
            or type(value['upper_surface_band_used']) is not bool
            or type(value['region_index']) is not int
            or type(value['region_shape']) is not list or len(value['region_shape']) != 2
            or any(type(v) is not int or v <= 0 for v in value['region_shape'])
            or value['measured_pixels'] > value['region_shape'][0]*value['region_shape'][1]
            or type(value['region_mask_sha256']) is not str
            or len(value['region_mask_sha256']) != 64
            or any(c not in '0123456789abcdef' for c in value['region_mask_sha256'])):
        raise ValueError('station_seed_insufficiency_evidence_invalid')
    if recovery_source_ns is not None and (type(recovery_source_ns) is not int
            or not value['source_ns'] < recovery_source_ns <= now_ns):
        raise ValueError('station_seed_insufficiency_recovery_source_invalid')
    return deepcopy(value)


def has_measurement_rejection(status, *, recovery=False):
    suffix = '_recovery' if recovery else ''
    return any(status.get(key+suffix) is not None for key in
               ('station_seed_insufficiency', 'station_frame_rejection'))


def measurement_rejection_receipt(status, *, recovery=False):
    """One causal measurement failure; never coalesce different failed sources."""
    suffix = '_recovery' if recovery else ''
    values = [(key+suffix, status[key+suffix]) for key in ('station_seed_insufficiency', 'station_frame_rejection')
              if status.get(key+suffix) is not None]
    if len(values) > 1:
        raise ValueError('station_measurement_rejection_mixed_receipts')
    if not values:
        return None
    key, receipt = values[0]
    if key != measurement_rejection_field(receipt, recovery=recovery):
        raise ValueError('station_measurement_rejection_cause_field_changed')
    return receipt


def measurement_rejection_kind(receipt):
    return ('source_component_overlap' if isinstance(receipt, dict) and 'kind' in receipt
            else 'seed_measurement_insufficient')


def measurement_rejection_field(receipt, *, recovery=False):
    field = ('station_frame_rejection' if measurement_rejection_kind(receipt) == 'source_component_overlap'
             else 'station_seed_insufficiency')
    return field+('_recovery' if recovery else '')


def validate_measurement_rejection(value, contract, now_ns, *, recovery_source_ns=None):
    """Typed failed measurement evidence, under the unchanged registered parent.

    Seed shortages keep their original wire contract. Source-component overlap
    is a separate truthful cause and never qualifies the conflicting proposal.
    """
    if not isinstance(value, dict) or 'kind' not in value:
        return validate_seed_insufficiency(value, contract, now_ns, recovery_source_ns=recovery_source_ns)
    fields = {'version', 'parent_source_ns', 'source_ns', 'kind', 'stage', 'component_basis',
        'region_shape', 'source_shape', 'proposal_pixels', 'component_pixels', 'overlap_pixels',
        'proposal_mask_sha256', 'component_mask_sha256', 'overlap_mask_sha256',
        'source_topology_contract_digest', 'source_topology_map_sha256',
        'source_qualified', 'identity_verified', 'metric_geometry_authority',
        'motion_authorized', 'reference_refreshed'}
    if type(value) is not dict or set(value) != fields:
        raise ValueError('station_frame_rejection_schema_invalid')
    validate_contract(contract, binding=contract['binding'], now_ns=now_ns)
    if (type(value['version']) is not int or value['version'] != 1
            or type(value['parent_source_ns']) is not int
            or value['parent_source_ns'] != contract['source_ns']
            or type(value['source_ns']) is not int
            or not contract['source_ns'] < value['source_ns'] <= now_ns
            or value['kind'] != 'source_component_overlap'
            or value['stage'] != 'complete_current_ownership_and_geometry'
            or value['component_basis'] not in ('native_measured', 'raw_source')
            or any(type(value[k]) is not list or len(value[k]) != 2
                   or any(type(v) is not int or v <= 0 for v in value[k])
                   for k in ('region_shape', 'source_shape'))
            or any(type(value[k]) is not int or value[k] <= 0
                   for k in ('proposal_pixels', 'component_pixels', 'overlap_pixels'))
            or not value['overlap_pixels'] <= min(value['proposal_pixels'], value['component_pixels'])
            or max(value['proposal_pixels'], value['component_pixels']) > value['source_shape'][0]*value['source_shape'][1]
            or any(value[k] is not False for k in ('source_qualified', 'identity_verified',
                'metric_geometry_authority', 'motion_authorized', 'reference_refreshed'))
            or any(type(value[k]) is not str or len(value[k]) != 64
                   or any(c not in '0123456789abcdef' for c in value[k])
                   for k in ('proposal_mask_sha256', 'component_mask_sha256', 'overlap_mask_sha256',
                       'source_topology_contract_digest', 'source_topology_map_sha256'))):
        raise ValueError('station_frame_rejection_evidence_invalid')
    if recovery_source_ns is not None and (type(recovery_source_ns) is not int
            or not value['source_ns'] < recovery_source_ns <= now_ns):
        raise ValueError('station_frame_rejection_recovery_source_invalid')
    return deepcopy(value)


def validate_station_pending_notice(status, *, contract, now_ns, status_reference_binding):
    """Classify clock age or a proven rejected bracket; never qualify a frame.

    A clock notice is the same registered READY source, without a discarded
    new source. A gap remains explicitly unqualified even when its measured
    endpoints are equal. Topic pairing and local SDK/owner checks remain the
    consumer's responsibility, under its original absolute/monotonic clocks.
    """
    from .feedback_bracket_gap import validate_station_feedback_gap
    binding, ns = _station_pending_envelope(status, contract, now_ns, status_reference_binding)
    if status.get('state') == SEED_PENDING_STATE:
        parent = status.get('station_compute_wait')
        validate_recovery_parent(contract, parent=parent,
            chain=status.get('station_compute_recovery_chain'), binding=binding, now_ns=now_ns)
        receipt = validate_measurement_rejection(measurement_rejection_receipt(status), parent, now_ns)
        kind = measurement_rejection_kind(receipt)
        expected_detail = SEED_PENDING_REASON if kind == 'seed_measurement_insufficient' else FRAME_PENDING_REASON
        if (status.get('detail') != expected_detail or ns != receipt['source_ns']
                or status.get(measurement_rejection_field(receipt)) != receipt
                or status.get('discarded_source_stamp_ns') != ns
                or any(status.get(k) is not None for k in ('station_feedback_gap', 'station_support_gap',
                    'station_feedback_gap_recovery', 'station_support_gap_recovery',
                    'station_seed_insufficiency_recovery', 'station_frame_rejection_recovery'))):
            raise ValueError('station_seed_pending_source_changed')
        return dict(kind=kind, source_ns=ns, parent_contract=deepcopy(parent),
            discarded_source_stamps_ns=[ns], metric_geometry_authority=False, motion_authorized=False)
    if (status.get('state') != 'processing_deadline_pending'
            or has_measurement_rejection(status)
            or has_measurement_rejection(status, recovery=True)):
        raise ValueError('station_pending_state_changed')
    parent = status.get('station_compute_wait')
    validate_recovery_parent(contract, parent=parent,
        chain=status.get('station_compute_recovery_chain'), binding=binding, now_ns=now_ns)
    if (status.get('detail') == 'processing_deadline_pending'
            and parent == contract and ns == contract['source_ns']
            and status.get('discarded_source_stamp_ns') is None
            and status.get('station_feedback_gap') is None
            and status.get('station_support_gap') is None
            and status.get('station_feedback_gap_recovery') is None
            and status.get('station_support_gap_recovery') is None
            and validate_contract(contract, binding=binding, now_ns=now_ns) == 'PENDING'):
        kind, stamps = 'clock_expiry', ()
    elif (status.get('detail') == 'station_accepted_feedback_bracket_pending_revalidation'
            and status.get('station_support_gap') is None
            and status.get('station_feedback_gap_recovery') is None
            and status.get('station_support_gap_recovery') is None):
        stamps = validate_station_feedback_gap(status.get('station_feedback_gap'),
            contract=parent, now_ns=now_ns)
        if ns != stamps[-1] or status.get('discarded_source_stamp_ns') != ns:
            raise ValueError('station_pending_discarded_source_changed')
        kind = 'accepted_feedback_gap'
    elif (status.get('detail') == 'processing_deadline_pending'
            and ns == parent['source_ns']
            and status.get('station_feedback_gap') is not None
            and status.get('station_support_gap') is None
            and status.get('station_feedback_gap_recovery') is None
            and status.get('station_support_gap_recovery') is None):
        # An idle watchdog repeats the registered READY header, while the
        # original rejected frame and its full bracket evidence remain a
        # separate causal fact. This is not a pure clock notification and
        # never qualifies either source or refreshes its registration.
        stamps = validate_station_feedback_gap(status['station_feedback_gap'],
            contract=parent, now_ns=now_ns)
        if status.get('discarded_source_stamp_ns') != stamps[-1]:
            raise ValueError('station_pending_repeat_discarded_source_changed')
        kind = 'accepted_feedback_gap_repeat'
    else:
        raise ValueError('station_pending_kind_unproven')
    return dict(kind=kind, source_ns=ns, parent_contract=deepcopy(parent),
        discarded_source_stamps_ns=list(stamps), metric_geometry_authority=False,
        motion_authorized=False)


def validate_station_feedback_recovery_notice(status, *, contract, now_ns, status_reference_binding):
    """Declare unresolved discarded inputs before a newer READY can join.

    This validates historical recovery obligations only. In particular it is
    not validate_registration: actual READY admission must still run the
    original fresh complete three-topic join and all current robot checks.
    """
    from .feedback_bracket_gap import validate_station_feedback_gap
    binding, ns = _station_pending_envelope(status, contract, now_ns, status_reference_binding)
    if status.get('state') != 'ready':
        raise ValueError('station_pending_recovery_state_changed')
    parent = status.get('station_compute_recovery')
    validate_recovery_parent(contract, parent=parent,
        chain=status.get('station_compute_recovery_chain'), binding=binding, now_ns=now_ns)
    offered = status.get('station_compute_wait')
    try:
        validate_contract(offered, binding=binding, now_ns=ns)
        if offered['source_ns'] != ns or not parent['source_ns'] < ns:
            raise ValueError('station_pending_recovery_source_changed')
    except (KeyError, TypeError, AttributeError) as exc:
        raise ValueError('station_pending_recovery_contract_malformed') from exc
    stamps = validate_station_feedback_gap(status.get('station_feedback_gap_recovery'),
        contract=parent, now_ns=now_ns, recovery_source_ns=ns)
    return dict(kind='accepted_feedback_gap_recovery', source_ns=ns,
        parent_contract=deepcopy(parent), discarded_source_stamps_ns=list(stamps),
        metric_geometry_authority=False, motion_authorized=False)


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
            or not 0. < maximum_frame_age_sec <= SOURCE_MAX_AGE_SEC):
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
    if (not isinstance(chain, list) or not 1 <= len(chain) <= RECOVERY_CHAIN_LIMIT
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
    """Describe absent support within the station's fixed identity lifetime.

    A caller must already own the complete parent registration. This receipt
    neither qualifies the failed source nor extends the station's identity.
    The separate geometry deadline remains unchanged: missing support always
    withdraws geometry, and recovery still requires a complete fresh source.
    """
    try:
        validate_contract(contract, binding=contract['binding'], now_ns=contract['source_ns'])
        source_ns = _positive(source_ns)
        first_source_ns = source_ns if first_source_ns is None else _positive(first_source_ns)
        deadline = min(contract[k] for k in (
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
