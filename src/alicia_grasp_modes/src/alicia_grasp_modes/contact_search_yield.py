"""Typed, non-authorizing evidence for an incomplete contact search yield.

Legacy v1 keeps the exact two-family wire meaning. Version 2 waits for the
30-second scheduling checkpoint and retains every listed family with a bound
failed precheck. Neither version claims all height alternatives were exhausted.
Both require a same-request compensated grasp collision. Independent measured missing sides,
registered gain, live ownership, and complete observation planning remain
separate requirements. Ordinary timeout and completed-search codes never
acquire this meaning.
"""
import re

CODE = 'CONTACT_SEARCH_YIELDED_FOR_MEASURED_VIEW'
_COLLISION_PREFIX = 'predicted_command continuous geometry: target OBB collision at '


def _integer(value, minimum=0):
    return type(value) is int and value >= minimum


def _hex(value, length):
    return isinstance(value, str) and re.fullmatch('[0-9a-f]{%d}' % length, value) is not None


def contact_search_yield_proof_valid(proof):
    """Accept preserved v1 history or explicit v2; unknown versions fail closed."""
    if not isinstance(proof, dict):
        return False
    version = proof.get('proof_version', 1)
    if type(version) is not int:
        return False
    if version == 1:
        return _legacy_contact_search_yield_proof_valid(proof)
    if version == 2:
        return _budget_checkpoint_proof_valid(proof)
    return False


def _legacy_contact_search_yield_proof_valid(proof):
    """Validate wire structure and exact request/time bindings, never motion."""
    try:
        if (proof['contact_search_code'] != CODE
                or proof['selected'] is not False
                or proof['terminated_early'] is not True
                or proof['contact_or_motion_authorized'] is not False
                or not _integer(proof['request_id'], 1)
                or not _integer(proof['generation'])
                or not _integer(proof['target_epoch'])
                or not _integer(proof['snapshot_stamp_ns'], 1)
                or any(not _integer(proof[key]) for key in
                       ('checked_count', 'shortlist_count', 'reachable_count'))
                or not 0 < proof['checked_count'] < proof['shortlist_count']
                or proof['reachable_count'] != 0):
            return False
        local = proof['locally_valid']
        if (any(not _integer(local[key]) for key in ('entered', 'passed', 'rejected'))
                or local['entered'] != local['passed'] + local['rejected']
                or local['passed'] < proof['checked_count']):
            return False
        evidence = proof['yield_evidence']
        if (evidence['policy'] != 'bounded_contact_search_for_measured_view'
                or not _integer(evidence['phase_id'], 1)
                or any(not _integer(evidence[key], 1) for key in
                       ('original_phase_deadline_ns', 'started_ns', 'yielded_ns', 'budget_deadline_ns'))
                or not proof['snapshot_stamp_ns'] <= evidence['started_ns']
                    < evidence['yielded_ns'] < evidence['original_phase_deadline_ns']
                or not evidence['started_ns'] < evidence['budget_deadline_ns']
                    <= min(evidence['original_phase_deadline_ns'] - 2_000_000_000,
                           evidence['started_ns'] + 30_000_000_000)
                or type(evidence['maximum_duration_sec']) not in (int, float)
                or evidence['maximum_duration_sec'] != 30.
                or type(evidence['maximum_original_families']) is not int
                or evidence['maximum_original_families'] != 2
                or evidence['incumbent_found'] is not False
                or evidence['exhausted_by'] not in ('original_family_limit', 'shared_time_budget')):
            return False
        if ('transition_ns' in evidence and (
                not _integer(evidence['transition_ns'], 1)
                or not evidence['yielded_ns'] <= evidence['transition_ns']
                    < evidence['original_phase_deadline_ns'])):
            return False
        families = evidence['original_families']
        if not isinstance(families, (list, tuple)) or len(families) != 2:
            return False
        if any(not _integer(family['track_id']) or not _integer(family['variant_index'])
               or not _hex(family['family_sha256'], 64) for family in families):
            return False
        if (len({family['family_sha256'] for family in families}) != 2
                or len({(family['track_id'], family['variant_index']) for family in families}) != 2
                or proof['checked_count'] < len(families)
                or (evidence['exhausted_by'] == 'shared_time_budget'
                    and evidence['yielded_ns'] < evidence['budget_deadline_ns'])):
            return False
        receipts = evidence['collision_receipts']
        if not isinstance(receipts, (list, tuple)) or not receipts:
            return False
        for receipt in receipts:
            if (any(type(receipt[key]) is not int or receipt[key] != proof[key] for key in
                    ('request_id', 'generation', 'target_epoch', 'snapshot_stamp_ns'))
                    or not _hex(receipt['plan_id'], 24)
                    or receipt['scope'] != 'read_only_candidate_constant_bias_prediction'
                    or receipt['success'] is not False
                    or receipt['execution_authorized'] is not False
                    or receipt['failed_stage'] != 'grasp'
                    or receipt['failure_code'] != 'CONTACT_SEQUENCE_PRECHECK_FAILED'
                    or not isinstance(receipt['reason'], str)
                    or not receipt['reason'].startswith(_COLLISION_PREFIX)
                    or len(receipt['reason']) <= len(_COLLISION_PREFIX)
                    or not _hex(receipt['evidence_sha256'], 64)
                    or any(not _integer(receipt[key], 1)
                           or not proof['snapshot_stamp_ns'] <= receipt[key] <= evidence['yielded_ns']
                           for key in ('sdk_stamp_ns', 'accepted_stamp_ns'))):
                return False
        return True
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
        return False


def _budget_checkpoint_proof_valid(proof):
    """Validate wire structure and exact request/time bindings, never motion."""
    try:
        if (proof['contact_search_code'] != CODE
                or proof['selected'] is not False
                or proof['terminated_early'] is not True
                or proof['contact_or_motion_authorized'] is not False
                or not _integer(proof['request_id'], 1)
                or not _integer(proof['generation'])
                or not _integer(proof['target_epoch'])
                or not _integer(proof['snapshot_stamp_ns'], 1)
                or any(not _integer(proof[key]) for key in
                       ('checked_count', 'shortlist_count', 'reachable_count'))
                or not 0 < proof['checked_count'] < proof['shortlist_count']
                or proof['reachable_count'] != 0):
            return False
        local = proof['locally_valid']
        if (any(not _integer(local[key]) for key in ('entered', 'passed', 'rejected'))
                or local['entered'] != local['passed'] + local['rejected']
                or local['passed'] < proof['checked_count']):
            return False
        evidence = proof['yield_evidence']
        if (evidence['policy'] != 'contact_search_budget_checkpoint_v2'
                or evidence['family_evidence_scope'] != 'at_least_one_bound_failed_precheck_not_exhaustive'
                or evidence['unchecked_count_scope'] != 'original_shortlist_candidates'
                or evidence['height_search_exhaustiveness_claimed'] is not False
                or not _integer(evidence['phase_id'], 1)
                or any(not _integer(evidence[key], 1) for key in
                       ('original_phase_deadline_ns', 'started_ns', 'yielded_ns', 'budget_deadline_ns'))
                or not proof['snapshot_stamp_ns'] <= evidence['started_ns']
                    < evidence['yielded_ns'] < evidence['original_phase_deadline_ns']
                or not evidence['started_ns'] < evidence['budget_deadline_ns']
                    <= min(evidence['original_phase_deadline_ns'] - 2_000_000_000,
                           evidence['started_ns'] + 30_000_000_000)
                or evidence['budget_deadline_ns'] != min(
                    evidence['original_phase_deadline_ns'] - 2_000_000_000,
                    evidence['started_ns'] + 30_000_000_000)
                or type(evidence['maximum_duration_sec']) not in (int, float)
                or evidence['maximum_duration_sec'] != 30.
                or 'maximum_original_families' in evidence
                or type(evidence['minimum_completed_families']) is not int
                or evidence['minimum_completed_families'] != 1
                or evidence['incumbent_found'] is not False
                or evidence['exhausted_by'] != 'budget_checkpoint'):
            return False
        if ('transition_ns' in evidence and (
                not _integer(evidence['transition_ns'], 1)
                or not evidence['yielded_ns'] <= evidence['transition_ns']
                    < evidence['original_phase_deadline_ns'])):
            return False
        families = evidence['original_families']
        if not isinstance(families, (list, tuple)) or len(families) < 1:
            return False
        if any(not _integer(family['track_id']) or not _integer(family['variant_index'])
               or not _hex(family['family_sha256'], 64) for family in families):
            return False
        if (len({family['family_sha256'] for family in families}) != len(families)
                or len({(family['track_id'], family['variant_index']) for family in families}) != len(families)
                or proof['checked_count'] < len(families)
                or evidence['yielded_ns'] < evidence['budget_deadline_ns']):
            return False
        attempted = evidence['attempted_families']
        if (any(not _integer(evidence[key]) for key in
                ('completed_family_count', 'attempted_family_count', 'unchecked_count'))
                or evidence['completed_family_count'] != len(families)
                or not isinstance(attempted, (list, tuple))
                or evidence['attempted_family_count'] != len(attempted)
                or not len(families) <= len(attempted) <= proof['checked_count']
                or evidence['unchecked_count'] != proof['shortlist_count'] - proof['checked_count']):
            return False
        if any(not _integer(f['track_id']) or not _integer(f['variant_index'])
               or not _hex(f['family_sha256'], 64) for f in attempted):
            return False
        if (len({f['family_sha256'] for f in attempted}) != len(attempted)
                or len({(f['track_id'], f['variant_index']) for f in attempted}) != len(attempted)
                or any(f['family_sha256'] not in {a['family_sha256'] for a in attempted}
                       for f in families)):
            return False
        receipts = evidence['collision_receipts']
        if not isinstance(receipts, (list, tuple)) or not receipts:
            return False
        for receipt in receipts:
            if (any(type(receipt[key]) is not int or receipt[key] != proof[key] for key in
                    ('request_id', 'generation', 'target_epoch', 'snapshot_stamp_ns'))
                    or not _hex(receipt['plan_id'], 24)
                    or receipt['scope'] != 'read_only_candidate_constant_bias_prediction'
                    or receipt['success'] is not False
                    or receipt['execution_authorized'] is not False
                    or receipt['failed_stage'] != 'grasp'
                    or receipt['failure_code'] != 'CONTACT_SEQUENCE_PRECHECK_FAILED'
                    or not isinstance(receipt['reason'], str)
                    or not receipt['reason'].startswith(_COLLISION_PREFIX)
                    or len(receipt['reason']) <= len(_COLLISION_PREFIX)
                    or not _hex(receipt['evidence_sha256'], 64)
                    or any(not _integer(receipt[key], 1)
                           or not proof['snapshot_stamp_ns'] <= receipt[key] <= evidence['yielded_ns']
                           for key in ('sdk_stamp_ns', 'accepted_stamp_ns'))):
                return False
        return True
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
        return False
