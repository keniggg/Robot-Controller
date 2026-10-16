"""Planning-only support-normal placement from frozen measured contact.

No target is relocated. A returned pose needs the existing complete geometry,
path, registration, simulation, plan identity and measured execution checks.
"""
import numpy as np
from .contact_height_margin import evaluate_contact_height_margin


def place_contact_height(*, translation_evidence_limit_m, **geometry):
    before = evaluate_contact_height_margin(
        translation_evidence_limit_m=translation_evidence_limit_m, **geometry)
    audit = dict(motion_authorized=False, target_relocated=False,
                 policy='center_existing_continuous_contact_support_interval',
                 before=before, normal_translation_m=0., reason=before['reason'])
    if not before.get('evaluated') or not before.get('nominal', {}).get('ok'):
        return None, audit
    transform = np.array(geometry['actual_T_base_tool0'], dtype=float, copy=True)
    if before['ok']:
        audit['after'] = before
        return transform, audit
    limit = float(translation_evidence_limit_m)
    # Stay inside the connected feasible component containing the nominal pose;
    # never jump through the concave finger-pad notch into another component.
    intervals = [(lo, hi) for lo, hi in before['allowed_normal_translation_intervals_m']
                 if lo is not None and hi is not None and lo <= 0. <= hi
                 and hi-lo >= 2.*limit]
    if not intervals:
        audit['reason'] = 'No placement in the nominal continuous contact/support interval covers the unchanged translation evidence limit. ' + before['reason']
        return None, audit
    lo, hi = max(intervals, key=lambda ends: (ends[1]-ends[0], -abs(sum(ends))))
    delta = (lo+hi)/2.
    transform[:3, 3] += delta*np.asarray(geometry['support_normal_base'])
    after = evaluate_contact_height_margin(translation_evidence_limit_m=limit,
        **dict(geometry, actual_T_base_tool0=transform, planned_T_base_tool0=transform))
    audit.update(normal_translation_m=float(delta), after=after, reason=after['reason'])
    return (transform if after['ok'] else None), audit


def contact_height_window_proposals(*, translation_evidence_limit_m, **geometry):
    """At most three alternatives inside the original connected contact window.

    The unchanged empirical error interval must fit around every proposal.
    These are planning poses only: the caller must recheck all four stages,
    registration displacement, and both nominal and compensated motion paths.
    """
    before = evaluate_contact_height_margin(
        translation_evidence_limit_m=translation_evidence_limit_m, **geometry)
    if not before.get('ok'):
        return ()
    limit = float(translation_evidence_limit_m)
    intervals = [(lo, hi) for lo, hi in before['allowed_normal_translation_intervals_m']
                 if lo is not None and hi is not None
                 and np.isfinite(lo) and np.isfinite(hi)
                 and lo <= -limit and hi >= limit]
    if len(intervals) != 1:
        return ()
    lo, hi = intervals[0]
    original = np.array(geometry['actual_T_base_tool0'], dtype=float, copy=True)
    normal = np.asarray(geometry['support_normal_base'], dtype=float)
    proposals = []
    # First try a smaller upward displacement to keep more finger-pad depth.
    # Retain both original 75% alternatives: the smaller move does not clear
    # every measured command-path collision. All share the caller's deadline
    # and still need the unchanged path and opposed-contact checks.
    for delta in (.5*(hi-limit), .75*(hi-limit), .75*(lo+limit)):
        if abs(delta) <= 1e-9:
            continue
        transform = original.copy()
        transform[:3, 3] += delta*normal
        after = evaluate_contact_height_margin(
            translation_evidence_limit_m=limit,
            **dict(geometry, actual_T_base_tool0=transform,
                   planned_T_base_tool0=transform))
        if after.get('ok'):
            proposals.append((transform, dict(
                policy='bounded_interior_same_contact_height_window',
                motion_authorized=False, target_relocated=False,
                normal_translation_m=float(delta), before=before, after=after)))
    return tuple(proposals)
