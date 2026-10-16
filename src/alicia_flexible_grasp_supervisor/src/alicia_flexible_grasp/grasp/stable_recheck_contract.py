"""Completed pre-MoveIt geometry rejection evidence, never motion authority.

Local candidate generation, stable-pose recheck and path planning are distinct
stages. An empty input to MoveIt proves nothing about path reachability. Only
an exhaustive, request-bound ledger of measured height-placement rejections
can support this particular completed-search classification.
"""
import math


CODE = 'NEAR_FIELD_STABLE_RECHECK_REJECTED'
INCOMPLETE_CODE = 'NEAR_FIELD_STABLE_RECHECK_INCOMPLETE'
NO_PATH_CODE = 'NEAR_FIELD_NO_PATH_CANDIDATE'
STAGE = 'stable_contact_height_placement'
REJECTION_CODE = 'CONTACT_HEIGHT_REGISTRATION_MARGIN'
BINDING_FIELDS = ('request_id', 'generation', 'target_epoch', 'snapshot_stamp_ns')


def measured_height_rejection_valid(placement):
    """Require evaluated evidence, not an exception name or missing input."""
    try:
        before = placement['before']
        last = placement.get('after', before)
        limit = before['translation_evidence_limit_m']
        return bool(
            placement['policy'] == 'center_existing_continuous_contact_support_interval'
            and placement['motion_authorized'] is False
            and placement['target_relocated'] is False
            and isinstance(placement['reason'], str) and placement['reason'].strip()
            and before['evaluated'] is True and last['evaluated'] is True
            and type(before['nominal']['ok']) is bool
            and before['ok'] is False and last['ok'] is False
            and type(limit) in (int, float) and math.isfinite(limit) and limit > 0
            and type(last['translation_evidence_limit_m']) in (int, float)
            and last['translation_evidence_limit_m'] == limit)
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
        return False


def completed_stable_recheck_proof_valid(proof):
    """Validate complete geometry rejection while keeping all motion unproved."""
    try:
        if (proof['contact_search_code'] != CODE
                or proof['selected'] is not False
                or proof['terminated_early'] is not False
                or proof['contact_or_motion_authorized'] is not False
                or any(type(proof[key]) is not int or proof[key] != 0
                       for key in ('checked_count', 'shortlist_count', 'reachable_count'))
                or any(type(proof[key]) is not int or proof[key] < 0
                       for key in BINDING_FIELDS)
                or proof['snapshot_stamp_ns'] <= 0):
            return False
        local = proof['locally_valid']
        if (any(type(local[key]) is not int or local[key] < 0
                for key in ('entered', 'passed', 'rejected'))
                or local['passed'] <= 0
                or local['entered'] != local['passed'] + local['rejected']):
            return False
        ledger = proof['stable_recheck']
        if (type(ledger['schema_version']) is not int or ledger['schema_version'] != 1
                or ledger['stage'] != STAGE or ledger['completed'] is not True
                or ledger['contact_or_motion_authorized'] is not False
                or any(type(ledger[key]) is not int or ledger[key] != proof[key]
                       for key in BINDING_FIELDS)):
            return False
        counts = ('stable_candidate_count', 'expected_variant_count',
                  'completed_variant_count', 'generated_count', 'rejected_count',
                  'incomplete_count')
        if any(type(ledger[key]) is not int or ledger[key] < 0 for key in counts):
            return False
        count = ledger['stable_candidate_count']
        expected = 2 * count
        rows = ledger['variants']
        if (count <= 0 or count != local['passed']
                or ledger['expected_variant_count'] != expected
                or ledger['completed_variant_count'] != expected
                or ledger['rejected_count'] != expected
                or ledger['generated_count'] != 0 or ledger['incomplete_count'] != 0
                or not isinstance(rows, list) or len(rows) != expected):
            return False
        seen = set()
        sources = {}
        for row in rows:
            index, variant = row['stable_index'], row['variant_index']
            if (type(index) is not int or not 0 <= index < count
                    or type(variant) is not int or variant not in (0, 1)
                    or (index, variant) in seen
                    or row['status'] != 'rejected' or row['code'] != REJECTION_CODE
                    or row['candidate_source'] != 'tabletop_geometry'
                    or type(row['track_id']) is not int or row['track_id'] < 0
                    or type(row['source_request_id']) is not int
                    or row['source_request_id'] != proof['request_id']
                    or not isinstance(row['reason'], str) or not row['reason'].strip()
                    or not measured_height_rejection_valid(row['contact_height_placement'])):
                return False
            seen.add((index, variant))
            source = (row['track_id'], row['source_request_id'], row['candidate_source'])
            if index in sources and sources[index] != source:
                return False
            sources[index] = source
        return len(seen) == expected and len(set(sources.values())) == count
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
        return False
