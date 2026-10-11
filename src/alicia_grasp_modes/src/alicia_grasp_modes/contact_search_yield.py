"""Typed, non-authorizing evidence for an incomplete contact search yield.

This proof says only that two original candidate families were checked and a
same-request compensated grasp collided. Independent measured missing sides,
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
