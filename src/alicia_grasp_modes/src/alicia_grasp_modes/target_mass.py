"""Task-scoped operator mass evidence; never a default for unknown objects."""
from copy import deepcopy
import hashlib
import json
import math

from alicia_grasp_modes.selection import parse_selection

MASS_PARAM = '/grasp_mode/operator_mass_estimate'


def bind_mass_evidence(config, plan, selection, evidence, status, now_ns):
    if not evidence or selection.get('mode') != 'unknown':
        return config
    selection = parse_selection(selection)
    if parse_selection(evidence.get('selection')) != selection:
        return config  # A different selection cannot inherit this estimate.
    if getattr(plan, 'model_choice', '') != 'unknown_tabletop':
        raise ValueError('operator mass requires the current unknown target')
    anchor = int(evidence.get('anchor_source_stamp_ns', 0))
    # Mass is attached to identity, not to individual depth samples. The
    # detector must have observed the same anchor at or after this plan's
    # snapshot. Plan/source-age gates already govern physical execution; an
    # extra one-second age test here incorrectly counts normal image compute
    # time and rejects otherwise valid frozen planning snapshots.
    source = int(status.get('source_stamp_ns', 0))
    snapshot = int(plan.header.stamp.to_nsec())
    track = str(getattr(plan, 'target_track_id', ''))
    if (not track or anchor <= selection['stamp_ns'] or snapshot < anchor
            or status.get('generation') != selection['generation']
            or int(status.get('anchor_source_stamp_ns', 0)) != anchor
            or status.get('state') != 'ready' or not status.get('target_locked')
            or status.get('target_lost') or not snapshot <= source <= now_ns):
        raise ValueError('operator mass target identity is no longer current: '
                         + json.dumps(dict(anchor=anchor, observed_anchor=status.get('anchor_source_stamp_ns'),
                             snapshot=snapshot, observed_source=source, state=status.get('state'),
                             target_locked=status.get('target_locked'), target_lost=status.get('target_lost'))))
    reported = int(evidence.get('reported_at_ns', 0))
    expires = int(evidence.get('expires_at_ns', 0))
    if not (selection['stamp_ns'] <= reported <= now_ns <= expires
            and expires-reported <= 1_200_000_000_000):
        raise ValueError('operator mass evidence expired or has invalid time bounds')
    estimate = evidence.get('estimated_mass_kg')
    if (isinstance(estimate, bool) or not isinstance(estimate, (float, int))
            or not math.isfinite(estimate) or estimate <= 0
            or evidence.get('source') != 'operator_estimate'
            or not str(evidence.get('operator_statement', '')).strip()):
        raise ValueError('operator mass needs a finite positive estimate and provenance')
    encoded = json.dumps(evidence, sort_keys=True, allow_nan=False).encode('utf8')
    bound = dict(evidence, evidence_sha256=hashlib.sha256(encoded).hexdigest(),
                 plan_id=str(plan.plan_id), target_track_id=track,
                 snapshot_stamp_ns=str(snapshot))
    result = deepcopy(config)
    result['target_mass_evidence'] = bound
    return result


def mass_config_from_ros(config, plan, selection):
    import rospy
    from std_msgs.msg import String
    evidence = rospy.get_param(MASS_PARAM, None)
    if not evidence or (selection or {}).get('mode') != 'unknown':
        return config
    # Old evidence is ignored before subscribing; carton and new selections
    # keep their normal dynamics without waiting for unknown perception.
    if parse_selection(evidence.get('selection')) != parse_selection(selection):
        return config
    status = json.loads(rospy.wait_for_message(
        '/perception/unknown/detector_status', String, timeout=1.).data)
    if parse_selection(rospy.get_param('/grasp_mode/selection')) != parse_selection(selection):
        raise ValueError('operator mass mode selection changed during lookup')
    return bind_mass_evidence(config, plan, selection, evidence, status, rospy.Time.now().to_nsec())
