"""Request-local search ordering only; no geometry or execution authority."""
from collections import OrderedDict
from collections.abc import Mapping
import math
from numbers import Integral


def contact_direction_priorities(candidates, original_rank, endpoint_priorities, *,
                                 observation_phase=''):
    """Interleave existing tabletop proposal directions inside each IK bucket.

    Return request-local ordinal hints indexed by (track_id, variant_index).
    A mixed/unknown/malformed batch returns no hints, preserving the caller's
    original ordering. No candidate is removed, cloned, merged or authorized.
    The caller must retain all original hard checks and deadline checks.
    """
    if observation_phase not in ('observing_contact', 'returned_contact'):
        return {}
    batch = tuple(candidates)
    if not batch:
        return {}
    records = []
    context = None
    seen = set()
    try:
        for candidate in batch:
            stable, payload = candidate.stable_candidate, candidate.payload
            if (stable.candidate_source != 'tabletop_geometry'
                    or payload.candidate_source != 'tabletop_geometry'
                    or not isinstance(payload.audit, Mapping)):
                return {}
            proposal = payload.audit.get('proposal_source_index')
            if isinstance(proposal, bool) or not isinstance(proposal, Integral) or proposal < 0:
                return {}
            request = candidate.evaluation_request_id
            stamp = candidate.evaluation_snapshot_stamp_sec
            if (isinstance(request, bool) or not isinstance(request, Integral) or request <= 0
                    or isinstance(stamp, bool) or not math.isfinite(float(stamp)) or stamp <= 0):
                return {}
            binding = (request, stamp, candidate.evaluation_context_revision,
                       stable.target_epoch, stable.target_track_id)
            if context is None:
                context = binding
            elif context != binding:
                return {}
            key = (candidate.track_id, candidate.variant_index)
            if key in seen:
                return {}
            seen.add(key)
            bucket = endpoint_priorities.get(key, 1)
            if isinstance(bucket, bool) or not isinstance(bucket, Integral) or bucket not in (0, 1):
                return {}
            records.append((candidate, key, int(bucket), int(proposal)))
    except (AttributeError, TypeError, ValueError, OverflowError):
        return {}
    # Keep the existing endpoint-hint priority absolutely ahead of diversity.
    # Within each bucket, first appearance in the original rank orders groups;
    # each group's original order is retained exactly.
    buckets = OrderedDict()
    for candidate, key, bucket, proposal in sorted(
            records, key=lambda row: (row[2], *original_rank(row[0]))):
        groups = buckets.setdefault(bucket, OrderedDict())
        groups.setdefault(proposal, []).append(key)
    priorities = {}
    for groups in buckets.values():
        ordinal = 0
        for depth in range(max(map(len, groups.values()))):
            for group in groups.values():
                if depth < len(group):
                    priorities[group[depth]] = ordinal
                    ordinal += 1
    return priorities
