"""Retire an expired RGB observation trial without silently selecting a new target.

After a submitted observation, --reason alone is never sufficient. An external
alignment receipt binds the actual user message and measured feedback/context.
An applied receipt may also survive an entirely unreserved generation, provided
its archived RGB intent uniquely matches the current production selection.
One separately attested driver replacement may change the control epoch, with
its actual positive-enable behavior preserved in the maintenance receipt. That
result cannot become a recursive carry-forward ancestor.
Neither path restores a motion owner, geometry, plan or expired budget. This
tool cannot authenticate a chat message or manufacture user confirmation.
Importing this module has no ROS or filesystem effects.
"""
import argparse
from collections import deque
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import threading
import time

import numpy as np
import cv2

from alicia_grasp_modes.selection import parse_selection
from alicia_grasp_modes.rgb_observation_source import (
    JOINT_NAMES, bracket_feedback, camera_geometry_snapshot, digest, require_fresh_source)
from alicia_grasp_modes.rgb_observation_region import (
    CenterSearchPolicy, RgbObservationRegion, RgbObservationRegions,
    propose_rgb_observation_regions)
from alicia_grasp_modes.rgb_observation_handoff import HandoffPolicy, _appearance, _mask
from alicia_grasp_modes.tabletop import Config
from alicia_flexible_grasp.vision.rgbd_rectification import (
    rectification_rgb_valid_mask, require_rectified_source_after_activation)

MOUNT = Path('/media/zhuyupei/SanStand')
ROOT = MOUNT/'实验记录/Robot-Controller/ros_log'
POSE_TOLERANCE_RAD = .004
MOTION_STAMP_FIELDS = ('submission_source_stamp_ns', 'submitted_ns',
    'execution_started_ns', 'completed_ns', 'settled_ns')
MAX_PENDING_RGBD_STAMPS = 32
MAX_UNRESERVED_TRANSITIONS = 4


def _ns(value, name):
    if isinstance(value, str) and value.isdecimal():
        value = int(value)
    if type(value) is not int or value <= 0:
        raise ValueError('invalid_'+name)
    return value


def _q(value):
    result = np.asarray(value, dtype=float)
    if result.shape != (6,) or not np.all(np.isfinite(result)):
        raise ValueError('invalid_measured_joint_feedback')
    return result


def _sample(value):
    if not isinstance(value, dict) or value.get('frame_id') != 'sdk_measured':
        raise ValueError('real_sdk_feedback_required')
    return _ns(value.get('stamp_ns'), 'feedback_stamp'), _q(value.get('positions'))


class AlignmentPoseMonitor:
    """Latch any measured excursion received during this tool's observation window.

    A returned pose cannot erase an intervening SDK measurement. This does not
    assert that a listener has observations from before it was started.
    """
    def __init__(self, receipt):
        self.anchor_ns, q = _sample(receipt.get('bound_feedback'))
        self.anchor_q = q.copy()
        self.invalidated = False

    def observe(self, stamp_ns, positions):
        if (_ns(stamp_ns, 'feedback_stamp') >= self.anchor_ns
                and np.max(np.abs(_q(positions)-self.anchor_q)) > POSE_TOLERANCE_RAD):
            self.invalidated = True

    def require_stationary(self):
        if self.invalidated:
            raise ValueError('measured_pose_excursion_since_alignment')


def cache_rgbd_part(pairs, stamp_ns, key, message):
    """About one second at 30 Hz; the separate .8s source gate remains exact."""
    pairs.setdefault(stamp_ns, {})[key] = message
    while len(pairs) > MAX_PENDING_RGBD_STAMPS:
        del pairs[min(pairs)]


def _sha(value, name):
    if not isinstance(value, str) or len(value) != 64 or any(c not in '0123456789abcdef' for c in value):
        raise ValueError('invalid_'+name)
    return value


def retry_source_context(selection, epoch_ns, camera, handeye, robot_description):
    """Public receipt-binding schema; telemetry counters do not change identity."""
    return dict(selection=parse_selection(selection), epoch_ns=_ns(epoch_ns, 'epoch'),
        camera=camera_geometry_snapshot(camera), handeye=deepcopy(handeye),
        model_sha256=hashlib.sha256(robot_description.encode()).hexdigest())


def previous_motion_possible(ledger):
    """Final-RPC failure cannot prove that the whole episode never moved.

    Existing ledgers do not contain a complete immutable execution history.
    Therefore only zero reservations with no submission marker proves absence;
    a positive/missing attempt count requires alignment, even after plan failure.
    """
    if ledger.get('phase') in ('executing', 'awaiting_measured_target'):
        return True
    for key in MOTION_STAMP_FIELDS:
        if key in ledger and ledger[key] not in (None, '', 0, '0'):
            _ns(ledger[key], key)
            return True
    if ledger.get('motion_submitted') is True:
        return True
    attempts = ledger.get('attempts')
    return type(attempts) is not int or attempts != 0


def validate_retry_preparation(previous_ledger, current_selection, *, now_ns,
        monotonic_now, epoch_ns=None, current_feedback=None, fresh_frame=None,
        confirmation_receipt=None):
    """Validate a selection reset, never a motion/grasp or restored old owner.

    A frame uses the recorder's source_context/source_id, source_stamp_ns,
    q_source, accepted_before/after and raw-array SHA fields. The live adapter
    supplies these directly from bounded sensor subscriptions, not status claims.
    """
    selection = parse_selection(current_selection)
    if selection['mode'] != 'unknown' or selection['strategy'] != 'observe_return':
        raise ValueError('rgb_unknown_selection_required')
    if not isinstance(previous_ledger, dict) or not previous_ledger:
        raise ValueError('previous_episode_ledger_required')
    ledger = deepcopy(previous_ledger)
    if parse_selection(ledger.get('selection')) != selection:
        raise ValueError('previous_episode_selection_changed')
    if ledger.get('phase') in ('reserved', 'executing'):
        raise ValueError('previous_episode_still_owns_motion')
    now_ns = _ns(now_ns, 'now')
    deadline = _ns(ledger.get('deadline_ns'), 'episode_deadline')
    created = _ns(ledger.get('created_ns'), 'episode_created')
    mono_deadline = ledger.get('deadline_monotonic')
    mono_created = ledger.get('created_monotonic')
    numbers = (monotonic_now, mono_created, mono_deadline)
    if (any(isinstance(n, bool) or not isinstance(n, (int, float)) or not math.isfinite(n)
            for n in numbers) or not 0 <= mono_created < mono_deadline or created >= deadline):
        raise ValueError('invalid_previous_episode_lifetime')
    if now_ns < deadline or monotonic_now < mono_deadline:
        raise ValueError('previous_episode_not_expired')
    moved = previous_motion_possible(ledger)
    if confirmation_receipt is None:
        if moved:
            raise ValueError('new_explicit_user_alignment_required_after_observation_motion')
        return dict(version=1, policy='expired_unsubmitted_episode_only',
            previous_selection=selection, previous_ledger_sha256=digest(ledger),
            previous_motion_possible=False, new_generation_authorized=True,
            motion_authorized=False, old_reference_reactivated=False,
            original_budget_modified=False, prior_attempts=ledger.get('attempts'),
            confirmation_id=None)
    receipt = deepcopy(confirmation_receipt)
    if (not isinstance(receipt, dict) or type(receipt.get('version')) is not int or receipt['version'] != 1
            or receipt.get('kind') != 'explicit_user_target_alignment'
            or receipt.get('target_aligned') is not True):
        raise ValueError('explicit_user_alignment_receipt_required')
    origin = receipt.get('confirmation_source')
    if (not isinstance(origin, dict) or origin.get('kind') != 'user_message'
            or not isinstance(origin.get('message_id'), str) or not origin['message_id'].strip()
            or not isinstance(origin.get('text'), str) or not origin['text'].strip()
            or not isinstance(receipt.get('confirmation_id'), str)
            or not receipt['confirmation_id'].strip()):
        raise ValueError('explicit_user_message_provenance_required')
    # These references are an external attestation, not a boolean CLI switch.
    # There is intentionally no helper which fills them on behalf of a user.
    if parse_selection(receipt.get('selection')) != selection:
        raise ValueError('alignment_receipt_selection_changed')
    epoch_ns = _ns(epoch_ns, 'epoch')
    if _ns(receipt.get('epoch_ns'), 'receipt_epoch') != epoch_ns:
        raise ValueError('alignment_receipt_epoch_changed')
    confirmed = _ns(receipt.get('confirmed_ns'), 'confirmation_stamp')
    last_motion = max([created] + [_ns(ledger[k], k) for k in MOTION_STAMP_FIELDS
        if ledger.get(k) not in (None, '', 0, '0')])
    if not max(last_motion, selection['stamp_ns'], epoch_ns) < confirmed <= now_ns:
        raise ValueError('alignment_confirmation_predates_current_view')
    bound_ns, bound_q = _sample(receipt.get('bound_feedback'))
    current_ns, current_q = _sample(current_feedback)
    if not confirmed <= bound_ns <= current_ns <= now_ns:
        raise ValueError('alignment_feedback_temporal_binding_invalid')
    require_fresh_source(current_ns, now_ns)
    if not isinstance(fresh_frame, dict):
        raise ValueError('confirmation_followed_by_fresh_rgbd_required')
    context = fresh_frame.get('source_context')
    if (not isinstance(context, dict) or parse_selection(context.get('selection')) != selection
            or _ns(context.get('epoch_ns'), 'frame_epoch') != epoch_ns):
        raise ValueError('fresh_frame_context_changed')
    if not isinstance(context.get('camera'), dict) or not isinstance(context.get('handeye'), dict):
        raise ValueError('fresh_frame_geometry_context_required')
    _sha(context.get('model_sha256'), 'model_sha256')
    frame_id = _sha(fresh_frame.get('source_id'), 'frame_source_id')
    if digest(context) != frame_id or receipt.get('source_context_sha256') != frame_id:
        raise ValueError('alignment_source_context_changed')
    stamp = _ns(fresh_frame.get('source_stamp_ns'), 'frame_stamp')
    require_rectified_source_after_activation(context['camera'], stamp)
    if not max(confirmed, bound_ns) < stamp <= current_ns:
        raise ValueError('fresh_rgbd_must_follow_alignment_confirmation')
    require_fresh_source(stamp, now_ns)
    try:
        q, before, after = bracket_feedback(
            [fresh_frame.get('accepted_before'), fresh_frame.get('accepted_after')],
            stamp, max(epoch_ns, selection['stamp_ns']))
    except (ValueError, TypeError) as exc:
        raise ValueError('fresh_frame_measured_bracket_invalid') from exc
    if before[0] < confirmed or after[0] > current_ns:
        raise ValueError('fresh_frame_feedback_not_after_confirmation')
    for sample in (q, _q(fresh_frame.get('q_source')), before[1], after[1], current_q):
        if np.max(np.abs(np.asarray(sample)-bound_q)) > POSE_TOLERANCE_RAD:
            raise ValueError('arm_moved_since_user_alignment')
    if not np.array_equal(q, _q(fresh_frame.get('q_source'))):
        raise ValueError('fresh_frame_feedback_binding_changed')
    for key in ('source_rgb_sha256', 'source_depth_sha256'):
        _sha(fresh_frame.get(key), key)
    return dict(version=1, policy='explicit_user_alignment_after_previous_observation',
        previous_selection=selection, previous_ledger_sha256=digest(ledger),
        previous_motion_possible=moved, new_generation_authorized=True,
        confirmation_id=receipt['confirmation_id'], confirmation_receipt_sha256=digest(receipt),
        frame_source_id=frame_id, frame_source_stamp_ns=stamp,
        source_rgb_sha256=fresh_frame['source_rgb_sha256'],
        source_depth_sha256=fresh_frame['source_depth_sha256'],
        epoch_ns=epoch_ns, q_source=q.tolist(), current_feedback_stamp_ns=current_ns,
        motion_authorized=False, old_reference_reactivated=False,
        original_budget_modified=False, prior_attempts=ledger.get('attempts'))


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reason', required=True, help='Audit explanation; never confirms target alignment')
    parser.add_argument('--alignment-receipt', type=Path,
        help='External explicit-user-message receipt with context and measured feedback binding')
    parser.add_argument('--carry-forward-result', type=Path,
        help='Successful original receipt-backed selection result; current generation must be unreserved')
    parser.add_argument('--unreserved-transition', type=Path, action='append', default=[],
        help='Ordered intervening selection-only receipt; never replaces the original RGB reference')
    parser.add_argument('--maintenance-receipt', type=Path,
        help='Completed authorized driver-only reload; one explicit epoch edge, never new user alignment')
    parser.add_argument('--aligned-observation-record', type=Path,
        help='Original recorder result containing the explicitly aligned RGB-D source')
    parser.add_argument('--aligned-source-stamp-ns', type=int,
        help='Exact original archived source timestamp; never picks a convenient different reference')
    parser.add_argument('--apply', action='store_true', help='Call mode selection only after validation')
    parser.add_argument('--wait-sec', type=float, default=5.)
    return parser


def validate_alignment_transition(result, current_selection, current_ledger):
    """Recompute the original attestation, preserving its actual selection.

    The RGB gateway persists every reservation before planning and never resets
    an old ledger at mode selection. Exact equality with the archived old ledger
    is therefore required, not a final `motion_commands_sent=False` claim. This
    only covers that gateway; current SDK pose and visual intent are checked
    separately. A missing/deleted ledger is not evidence of absence.
    """
    if (not isinstance(result, dict) or result.get('preview_only') is not False
            or result.get('failure') is not None
            or result.get('response', {}).get('success') is not True
            or result.get('motion_commands_sent') is not False
            or result.get('torque_commands_sent') is not False
            or result.get('original_budget_modified') is not False):
        raise ValueError('successful_receipt_backed_transition_required')
    previous = parse_selection(result.get('previous_selection'))
    selection = parse_selection(current_selection)
    new = parse_selection(result.get('new_selection'))
    if (selection != new or new['mode'] != 'unknown' or new['strategy'] != 'observe_return'
            or new['generation'] != previous['generation']+1
            or new['stamp_ns'] <= previous['stamp_ns']):
        raise ValueError('carry_forward_selection_changed')
    if not isinstance(current_ledger, dict) or not current_ledger:
        raise ValueError('unreserved_generation_requires_preserved_ledger')
    if digest(current_ledger) != digest(result.get('previous_ledger')):
        raise ValueError('generation_has_reservation_or_changed_ledger')
    evidence = result.get('evidence')
    if not isinstance(evidence, dict) or evidence.get('confirmation_receipt') != result.get('alignment_receipt'):
        raise ValueError('original_confirmation_evidence_changed')
    validation = validate_retry_preparation(current_ledger, previous, **evidence)
    if (validation != result.get('validation')
            or validation['policy'] != 'explicit_user_alignment_after_previous_observation'
            or not evidence['now_ns'] < new['stamp_ns']):
        raise ValueError('original_transition_validation_changed')
    return validation


def _array_sha(array):
    return hashlib.sha256(np.ascontiguousarray(array).tobytes()).hexdigest()


def validate_unreserved_transitions(original, transitions, current_selection, ledger):
    """Account for bounded software reloads without inventing user alignment.

    Intervening records prove only selection bookkeeping. Their images are not
    new anchors: current RGB/SDK must still match the immutable original source.
    Any gateway reservation changes its persisted ledger and blocks this path.
    """
    if not isinstance(transitions, (list, tuple)) or len(transitions) > MAX_UNRESERVED_TRANSITIONS:
        raise ValueError('unreserved_transition_budget_exceeded')
    expected = parse_selection(original.get('new_selection'))
    original_validation = validate_alignment_transition(original, expected, ledger)
    original_context = original['evidence']['fresh_frame']['source_context']
    chain_digests = []
    receipt = original['alignment_receipt']
    anchor_ns, anchor_q = _sample(receipt['bound_feedback'])
    for record in transitions:
        if (not isinstance(record, dict) or record.get('preview_only') is not False
                or record.get('failure') is not None
                or record.get('response', {}).get('success') is not True
                or any(record.get(key) is not False for key in
                       ('motion_commands_sent', 'torque_commands_sent', 'original_budget_modified'))
                or record.get('previous_ledger') != ledger
                or record.get('alignment_receipt') != receipt
                or parse_selection(record.get('previous_selection')) != expected):
            raise ValueError('unreserved_transition_evidence_changed')
        new = parse_selection(record.get('new_selection'))
        validation, evidence = record.get('validation', {}), record.get('evidence', {})
        if (validation.get('policy') != 'confirmed_unreserved_generation_same_rgb_intent'
                or validation.get('transition_result_sha256') != digest(original)
                or validation.get('unreserved_transition_sha256', []) != chain_digests
                or validation.get('previous_selection') != expected
                or validation.get('previous_ledger_sha256') != digest(ledger)
                or validation.get('confirmation_receipt_sha256') != digest(receipt)
                or validation.get('confirmation_id') != receipt['confirmation_id']
                or validation.get('original_confirmation_validation_sha256') != digest(original_validation)
                or validation.get('prior_gateway_generation_unreserved') is not True
                or validation.get('prior_attempts') != 0
                or validation.get('new_generation_authorized') is not True
                or any(validation.get(key) is not False for key in
                       ('motion_authorized', 'old_reference_reactivated', 'original_budget_modified',
                        'historical_continuous_stationarity_claimed'))
                or evidence.get('confirmation_receipt') != receipt
                or new['mode'] != expected['mode'] or new['strategy'] != expected['strategy']
                or new['generation'] != expected['generation']+1
                or not expected['stamp_ns'] < _ns(evidence.get('now_ns'), 'transition_now') < new['stamp_ns']):
            raise ValueError('unreserved_transition_validation_changed')
        frame = evidence.get('fresh_frame', {})
        context = frame.get('source_context', {})
        if (set(context) != set(original_context) or context.get('selection') != expected
                or any(context[key] != original_context[key] for key in original_context if key != 'selection')
                or digest(context) != frame.get('source_id')
                or validation.get('frame_source_id') != frame.get('source_id')
                or evidence.get('epoch_ns') != context.get('epoch_ns')
                or validation.get('epoch_ns') != context.get('epoch_ns')):
            raise ValueError('unreserved_transition_context_changed')
        now = _ns(evidence['now_ns'], 'transition_now')
        stamp = _ns(frame.get('source_stamp_ns'), 'transition_frame')
        current_ns, current_q = _sample(evidence.get('current_feedback'))
        q, lo, hi = bracket_feedback([frame.get('accepted_before'), frame.get('accepted_after')],
                                    stamp, max(int(receipt['epoch_ns']), expected['stamp_ns']))
        require_fresh_source(stamp, now); require_fresh_source(current_ns, now)
        if (not max(anchor_ns, expected['stamp_ns']) < stamp <= current_ns <= now
                or hi[0] > current_ns or not np.array_equal(q, _q(frame.get('q_source')))
                or any(np.max(np.abs(_q(values)-anchor_q)) > POSE_TOLERANCE_RAD
                       for values in (q, lo[1], hi[1], current_q))
                or validation.get('frame_source_stamp_ns') != stamp
                or validation.get('current_feedback_stamp_ns') != current_ns
                or not np.array_equal(_q(validation.get('q_source')), q)
                or any(validation.get(key) != frame.get(key) for key in
                       ('source_rgb_sha256', 'source_depth_sha256'))):
            raise ValueError('unreserved_transition_source_binding_changed')
        chain_digests.append(digest(record))
        expected = new
    if parse_selection(current_selection) != expected:
        raise ValueError('unreserved_transition_chain_incomplete')
    return original_validation, chain_digests


def load_unreserved_transitions(paths, original_path, record_path, source_stamp_ns):
    """Verify the exact archived links, not just their JSON assertions."""
    if len(paths) > MAX_UNRESERVED_TRANSITIONS:
        raise ValueError('unreserved_transition_budget_exceeded')
    if not paths:
        return []
    root_path, rgb_path = Path(original_path).resolve(), Path(record_path).resolve()
    root_sha = hashlib.sha256(root_path.read_bytes()).hexdigest()
    rgb_sha = hashlib.sha256(rgb_path.read_bytes()).hexdigest()
    records, links = [], []
    for path in paths:
        path = Path(path).resolve(); raw = path.read_bytes(); record = json.loads(raw)
        if (Path(record.get('carry_forward_result_file', '')).resolve() != root_path
                or record.get('carry_forward_result_file_sha256') != root_sha
                or Path(record.get('aligned_observation_record', '')).resolve() != rgb_path
                or record.get('aligned_record_file_sha256') != rgb_sha
                or record.get('aligned_source_stamp_ns') != source_stamp_ns
                or record.get('unreserved_transition_files', []) != links):
            raise ValueError('unreserved_transition_archive_link_changed')
        records.append(record)
        links.append(dict(path=str(path), sha256=hashlib.sha256(raw).hexdigest()))
    return records


def _retry_context_from_record(context):
    if not isinstance(context, dict):
        raise ValueError('aligned_record_context_required')
    # Recorder has additional perception/TF fields. Verify its full source hash
    # separately, then compare the exact complete retry identity, without
    # rewriting either record's selection or source ID.
    keys = ('selection', 'epoch_ns', 'camera', 'handeye', 'model_sha256')
    if any(key not in context for key in keys):
        raise ValueError('aligned_record_context_incomplete')
    return {key: deepcopy(context[key]) for key in keys}


def prepare_aligned_rgb_reference(transition, row, bgr, depth_raw):
    """Reconstruct only image-space intent from an original, hash-bound source."""
    validate_alignment_transition(transition, transition['new_selection'], transition['previous_ledger'])
    evidence = transition['evidence']; receipt = transition['alignment_receipt']
    context = row.get('source_context')
    if (digest(context) != row.get('source_id')
            or _retry_context_from_record(context) != evidence['fresh_frame']['source_context']
            or row.get('source_context_unchanged') is not True
            or row.get('fresh_for_existing_frame_limit') is not True):
        raise ValueError('aligned_record_source_context_changed')
    stamp = _ns(row.get('source_stamp_ns'), 'aligned_source_stamp')
    confirmed = _ns(receipt['confirmed_ns'], 'confirmation_stamp')
    anchor_ns, anchor_q = _sample(receipt['bound_feedback'])
    if not max(confirmed, anchor_ns) < stamp <= evidence['fresh_frame']['source_stamp_ns']:
        raise ValueError('aligned_source_not_in_confirmed_transition_interval')
    q, lo, hi = bracket_feedback([row.get('accepted_before'),row.get('accepted_after')],
        stamp, max(int(receipt['epoch_ns']), parse_selection(receipt['selection'])['stamp_ns']))
    if (lo[0] < confirmed or hi[0] > transition['new_selection']['stamp_ns']
            or not np.array_equal(q, _q(row.get('q_source')))
            or any(np.max(np.abs(_q(sample)-anchor_q)) > POSE_TOLERANCE_RAD
                   for sample in (q,lo[1],hi[1]))):
        raise ValueError('aligned_record_feedback_binding_changed')
    _validate_rgb_arrays(bgr, depth_raw, row)
    cfg = Config(**row['config'])
    policy = CenterSearchPolicy(center_xy=(.5,.5), radius_fraction=cfg.center_radius_fraction)
    rgb_valid = rectification_rgb_valid_mask(context['camera'], bgr.shape[:2])
    proposals = propose_rgb_observation_regions(bgr, policy=policy,
        **({} if rgb_valid is None else {'rgb_valid': rgb_valid}))
    if proposals.reason != 'observation_region_found' or proposals.selected is None:
        raise ValueError('aligned_original_rgb_intent_not_unique')
    region = proposals.selected
    histogram, contour = _appearance(bgr, _mask(region.mask, bgr.shape[:2]))
    return dict(row=deepcopy(row), region=region, histogram=histogram, contour=contour,
        search_policy=policy, rgb_sha256=_array_sha(bgr), depth_sha256=_array_sha(depth_raw))


def _validate_rgb_arrays(bgr, depth_raw, frame):
    bgr, depth_raw = np.asarray(bgr), np.asarray(depth_raw)
    if bgr.dtype != np.uint8 or depth_raw.dtype not in (np.dtype('uint16'),np.dtype('float32')):
        raise ValueError('retry_rgbd_source_dtype_invalid')
    camera = frame['source_context']['camera']
    require_rectified_source_after_activation(camera, frame['source_stamp_ns'])
    validate_image_geometry(bgr.shape, depth_raw.shape, camera['frame_id'],camera['frame_id'],camera)
    if (_array_sha(bgr) != frame.get('source_rgb_sha256')
            or _array_sha(depth_raw) != frame.get('source_depth_sha256')):
        raise ValueError('retry_raw_rgbd_hash_changed')


def associate_aligned_rgb(reference, bgr, proposals):
    """Same-station association; reuse the original handoff appearance limits.

    Original IoU .70 and production attention selection remain mandatory.
    Matching an old object elsewhere in the image never authorizes selecting a
    new centre candidate. No depth, object geometry or motion proof is created.
    """
    if (type(proposals) is not RgbObservationRegions
            or proposals.image_shape != bgr.shape[:2]
            or proposals.reason != 'observation_region_found' or proposals.selected is None):
        raise ValueError('current_production_rgb_intent_not_unique')
    policy = HandoffPolicy()
    if len(proposals.candidates) > policy.maximum_candidates:
        raise ValueError('retry_rgb_candidate_budget_exhausted')
    original = np.asarray(reference['region'].mask) != 0
    matches=[]
    for index, region in enumerate(proposals.candidates):
        if type(region) is not RgbObservationRegion:
            raise ValueError('retry_rgb_region_type_invalid')
        mask = _mask(region.mask, bgr.shape[:2])
        iou = np.count_nonzero(original & (mask != 0))/max(1,np.count_nonzero(original | (mask != 0)))
        if iou < .70:
            continue
        hist, contour = _appearance(bgr, mask)
        colour = float(np.sqrt(max(0.,1.-np.sum(np.sqrt(hist*reference['histogram'])))))
        shape = float(cv2.matchShapes(reference['contour'],contour,cv2.CONTOURS_MATCH_I1,0.))
        if colour <= policy.max_colour_distance and shape <= policy.max_contour_distance:
            score=.65*colour/policy.max_colour_distance+.35*shape/policy.max_contour_distance
            matches.append((score,index,iou,colour,shape))
    matches.sort()
    if not matches:
        raise ValueError('no_unique_same_station_rgb_intent')
    if len(matches)>1 and matches[1][0]-matches[0][0] <= policy.ambiguity_margin:
        raise ValueError('same_station_rgb_intent_ambiguous')
    score,index,iou,colour,shape=matches[0]
    if index != proposals.selected_index:
        raise ValueError('production_center_selection_differs_from_aligned_intent')
    region=proposals.candidates[index]
    return dict(kind='same_station_original_rgb_intent', mask_iou=iou,
        colour_distance=colour, contour_distance=shape, score=score,
        current_region_index=index, current_bbox_xywh=list(region.bbox_xywh),
        current_mask_sha256=_array_sha(np.asarray(region.mask,dtype=np.uint8)),
        reference_rgb_sha256=reference['rgb_sha256'],
        reference_mask_sha256=_array_sha(np.asarray(reference['region'].mask,dtype=np.uint8)),
        reference_source_stamp_ns=reference['row']['source_stamp_ns'],
        metric_geometry_authority=False, motion_authorized=False)


def load_maintenance_receipt(path):
    """Verify archived maintenance evidence without changing its assertions.

    Like an alignment receipt, this is an external attestation. The validator
    does not manufacture authorization, infer an unobserved motion history, or
    treat a successful process launch as measured robot health.
    """
    raw = Path(path).read_bytes()
    receipt = json.loads(raw)
    evidence = receipt.get('actuation', {}).get('evidence_files')
    if not isinstance(evidence, list) or not evidence:
        raise ValueError('maintenance_evidence_files_required')
    for item in evidence:
        if not isinstance(item, dict) or not isinstance(item.get('path'), str) or not item['path']:
            raise ValueError('maintenance_evidence_file_invalid')
        expected = _sha(item.get('sha256'), 'maintenance_evidence_sha256')
        if hashlib.sha256(Path(item['path']).read_bytes()).hexdigest() != expected:
            raise ValueError('maintenance_evidence_file_changed')
    return receipt


def validate_maintenance_epoch_edge(receipt, transition, chain, selection, ledger,
        reference, *, now_ns):
    """Qualify one recorded driver replacement, preserving the original intent.

    Only the current edge may change epoch. Existing historical transitions
    retain their strict original-epoch validation, and this new result is not a
    supported ancestor of the ordinary carry-forward policy.
    """
    if (not isinstance(receipt, dict) or type(receipt.get('version')) is not int
            or receipt['version'] != 1 or receipt.get('kind') != 'authorized_driver_only_reload'
            or receipt.get('status') != 'completed'
            or not isinstance(receipt.get('authorization_reference'), str)
            or not receipt['authorization_reference'].strip()):
        raise ValueError('completed_authorized_driver_maintenance_required')
    if receipt.get('driver_node') != '/alicia_d_driver_node':
        raise ValueError('maintenance_driver_scope_changed')
    started = _ns(receipt.get('started_ns'), 'maintenance_started')
    completed = _ns(receipt.get('completed_ns'), 'maintenance_completed')
    if not started < completed <= _ns(now_ns, 'now'):
        raise ValueError('maintenance_lifetime_invalid')
    original_receipt = transition['alignment_receipt']
    anchor_ns, anchor_q = _sample(original_receipt['bound_feedback'])
    if started <= max(anchor_ns, selection['stamp_ns']):
        raise ValueError('maintenance_predates_current_selection')
    expected_binding = dict(original_transition_sha256=digest(transition),
        confirmation_receipt_sha256=digest(original_receipt),
        unreserved_transition_sha256=chain,
        aligned_source_stamp_ns=reference['row']['source_stamp_ns'],
        aligned_rgb_sha256=reference['rgb_sha256'])
    bindings = [receipt['binding']] if 'binding' in receipt else []
    # The first real deployment recorded these same fields at top level.
    # Validate that original layout without rewriting its archived bytes.
    if any(key in receipt for key in expected_binding):
        bindings.append({key: receipt.get(key) for key in expected_binding})
    if not bindings or any(value != expected_binding for value in bindings):
        raise ValueError('maintenance_alignment_binding_changed')
    before, after = receipt.get('before'), receipt.get('after')
    if not isinstance(before, dict) or not isinstance(after, dict):
        raise ValueError('maintenance_before_after_required')
    original_context = transition['evidence']['fresh_frame']['source_context']
    old_epoch = _ns(original_context.get('epoch_ns'), 'original_epoch')
    new_epoch = _ns(after.get('epoch_ns'), 'maintenance_new_epoch')
    if (_ns(before.get('epoch_ns'), 'maintenance_old_epoch') != old_epoch
            or not old_epoch < new_epoch or not started <= new_epoch <= completed):
        raise ValueError('maintenance_epoch_transition_invalid')
    before_ns, before_q = _sample(before.get('feedback'))
    after_ns, after_q = _sample(after.get('feedback'))
    if not old_epoch <= before_ns <= started <= new_epoch <= after_ns <= completed:
        raise ValueError('maintenance_feedback_temporal_binding_invalid')
    require_fresh_source(before_ns, started)
    require_fresh_source(after_ns, completed)
    if any(np.max(np.abs(q-anchor_q)) > POSE_TOLERANCE_RAD for q in (before_q, after_q)):
        raise ValueError('maintenance_measured_pose_changed')
    for snapshot, epoch in ((before, old_epoch), (after, new_epoch)):
        if parse_selection(snapshot.get('selection')) != selection or snapshot.get('ledger') != ledger:
            raise ValueError('maintenance_selection_or_ledger_changed')
        context = snapshot.get('source_context')
        if (not isinstance(context, dict) or set(context) != set(original_context)
                or parse_selection(context.get('selection')) != selection
                or _ns(context.get('epoch_ns'), 'maintenance_context_epoch') != epoch
                or any(context[key] != original_context[key]
                       for key in original_context if key not in ('selection', 'epoch_ns'))):
            raise ValueError('maintenance_geometry_context_changed')
        _sha(snapshot.get('driver_binary_sha256'), 'maintenance_driver_binary_sha256')
    if before['driver_binary_sha256'] == after['driver_binary_sha256']:
        raise ValueError('maintenance_driver_binary_not_replaced')
    old_nodes, new_nodes = before.get('nodes'), after.get('nodes')
    driver = receipt['driver_node']
    if (not isinstance(old_nodes, dict) or not isinstance(new_nodes, dict)
            or set(old_nodes) != set(new_nodes) or driver not in old_nodes or len(old_nodes) < 2):
        raise ValueError('maintenance_node_inventory_changed')
    for name in old_nodes:
        for value in (old_nodes[name], new_nodes[name]):
            if (not isinstance(value, dict) or type(value.get('pid')) is not int or value['pid'] <= 0
                    or not isinstance(value.get('uri'), str) or not value['uri'].strip()):
                raise ValueError('maintenance_node_identity_invalid')
        if name != driver and old_nodes[name] != new_nodes[name]:
            raise ValueError('maintenance_replaced_non_driver_node')
    if old_nodes[driver]['pid'] == new_nodes[driver]['pid']:
        raise ValueError('maintenance_driver_process_not_replaced')
    policy = receipt.get('policy_after', {})
    rate = policy.get('temperature_poll_rate_hz')
    if (policy.get('software_temperature_monitoring_enabled') is not False
            or isinstance(rate, bool) or not isinstance(rate, (int, float)) or rate != 0.):
        raise ValueError('maintenance_temperature_policy_not_disabled')
    actuation = receipt.get('actuation', {})
    if (actuation.get('joint_motion_commands_sent') is not False
            or actuation.get('torque_disable_commands_sent') is not False
            or actuation.get('torque_enable_policy') != 'positive_enable_only'
            or type(actuation.get('positive_enable_observed')) is not bool
            or not isinstance(actuation.get('evidence_files'), list) or not actuation['evidence_files']):
        raise ValueError('maintenance_actuation_scope_changed')
    for item in actuation['evidence_files']:
        if not isinstance(item, dict) or not isinstance(item.get('path'), str) or not item['path']:
            raise ValueError('maintenance_evidence_file_invalid')
        _sha(item.get('sha256'), 'maintenance_evidence_sha256')
    return dict(kind='authorized_driver_only_reload_epoch_edge', maximum_epoch_edges=1,
        before_epoch_ns=old_epoch, after_epoch_ns=new_epoch,
        maintenance_started_ns=started, maintenance_completed_ns=completed,
        maintenance_receipt_sha256=digest(receipt),
        before_driver_binary_sha256=before['driver_binary_sha256'],
        after_driver_binary_sha256=after['driver_binary_sha256'],
        before_source_id=digest(before['source_context']), after_source_id=digest(after['source_context']),
        positive_enable_observed=actuation['positive_enable_observed'],
        positive_enable_evidence_scope=actuation.get('positive_enable_evidence_scope',
            'external maintenance attestation; not hardware acknowledgement or measured actuation confirmation'),
        torque_enable_policy='positive_enable_only', historical_continuous_stationarity_claimed=False,
        recursive_epoch_carry_authorized=False, old_reference_reactivated=False,
        original_budget_modified=False, motion_authorized=False)


def validate_carry_forward_preparation(transition, current_selection, current_ledger, *,
        now_ns, monotonic_now, epoch_ns, current_feedback, fresh_frame, reference,
        current_bgr, current_depth_raw, confirmation_receipt=None, unreserved_transitions=(),
        maintenance_receipt=None):
    original, chain = validate_unreserved_transitions(
        transition, unreserved_transitions, current_selection, current_ledger)
    now_ns=_ns(now_ns,'now'); epoch_ns=_ns(epoch_ns,'epoch')
    selection=parse_selection(current_selection)
    receipt=transition['alignment_receipt']; anchor_ns,anchor_q=_sample(receipt['bound_feedback'])
    context=fresh_frame.get('source_context')
    oldcontext=transition['evidence']['fresh_frame']['source_context']
    maintenance = None
    if maintenance_receipt is None:
        if (not isinstance(context,dict) or parse_selection(context.get('selection')) != selection
                or set(context) != set(oldcontext)
                or any(context[key] != oldcontext[key] for key in oldcontext if key != 'selection')
                or _ns(context.get('epoch_ns'),'frame_epoch') != epoch_ns
                or digest(context) != fresh_frame.get('source_id')):
            raise ValueError('carry_forward_geometry_or_epoch_changed')
    else:
        maintenance = validate_maintenance_epoch_edge(maintenance_receipt, transition, chain,
            selection, current_ledger, reference, now_ns=now_ns)
        if (context != maintenance_receipt['after']['source_context']
                or epoch_ns != maintenance['after_epoch_ns']
                or digest(context) != fresh_frame.get('source_id')):
            raise ValueError('maintenance_current_source_context_changed')
    current_ns,current_q=_sample(current_feedback)
    stamp=_ns(fresh_frame.get('source_stamp_ns'),'frame_stamp')
    if not max(anchor_ns,selection['stamp_ns']) < stamp <= current_ns <= now_ns:
        raise ValueError('carry_forward_fresh_source_temporal_binding_invalid')
    if maintenance is not None and stamp <= maintenance['maintenance_completed_ns']:
        raise ValueError('maintenance_requires_new_post_completion_rgbd')
    require_fresh_source(current_ns,now_ns); require_fresh_source(stamp,now_ns)
    q,lo,hi=bracket_feedback([fresh_frame.get('accepted_before'),fresh_frame.get('accepted_after')],
        stamp,max(epoch_ns,selection['stamp_ns']))
    if (hi[0]>current_ns or not np.array_equal(q,_q(fresh_frame.get('q_source')))
            or any(np.max(np.abs(_q(sample)-anchor_q)) > POSE_TOLERANCE_RAD
                   for sample in (q,lo[1],hi[1],current_q))):
        raise ValueError('carry_forward_measured_pose_changed')
    _validate_rgb_arrays(current_bgr,current_depth_raw,fresh_frame)
    if (reference['row']['source_context']['selection'] != receipt['selection']
            or reference['row']['source_rgb_sha256'] != reference['rgb_sha256']):
        raise ValueError('carry_forward_reference_binding_changed')
    for record in unreserved_transitions:
        association = record['validation'].get('appearance_association', {})
        if (association.get('reference_source_stamp_ns') != reference['row']['source_stamp_ns']
                or association.get('reference_rgb_sha256') != reference['rgb_sha256']
                or association.get('reference_mask_sha256') !=
                   _array_sha(np.asarray(reference['region'].mask, dtype=np.uint8))):
            raise ValueError('unreserved_transition_original_rgb_reference_changed')
    rgb_valid = rectification_rgb_valid_mask(context['camera'], current_bgr.shape[:2])
    proposals=propose_rgb_observation_regions(current_bgr,policy=reference['search_policy'],
        **({} if rgb_valid is None else {'rgb_valid': rgb_valid}))
    association=associate_aligned_rgb(reference,current_bgr,proposals)
    # Account for CPU work using the caller's final recheck too; no stale source
    # is relabelled. Historical timestamps are never reissued as current data.
    result = dict(version=1,policy='confirmed_unreserved_generation_same_rgb_intent',
        previous_selection=selection,previous_ledger_sha256=digest(current_ledger),
        transition_result_sha256=digest(transition),
        unreserved_transition_sha256=chain,
        intervening_validation_scope='selection_bookkeeping_only',
        historical_appearance_revalidated=False,
        confirmation_id=receipt['confirmation_id'],confirmation_receipt_sha256=digest(receipt),
        original_confirmation_validation_sha256=digest(original),
        frame_source_id=fresh_frame['source_id'],frame_source_stamp_ns=stamp,
        source_rgb_sha256=fresh_frame['source_rgb_sha256'],source_depth_sha256=fresh_frame['source_depth_sha256'],
        epoch_ns=epoch_ns,q_source=q.tolist(),current_feedback_stamp_ns=current_ns,
        new_generation_authorized=True,previous_motion_possible=False,prior_attempts=0,
        previous_motion_possible_scope='current_selection_rgb_gateway_reservations_only',
        prior_gateway_generation_unreserved=True,appearance_association=association,
        historical_continuous_stationarity_claimed=False,
        motion_authorized=False,old_reference_reactivated=False,original_budget_modified=False)
    if maintenance is not None:
        result.update(policy='confirmed_unreserved_generation_same_rgb_intent_after_driver_reload',
            maintenance_epoch_transition=maintenance,
            maintenance_receipt_sha256=digest(maintenance_receipt),
            confirmation_continues_from_original_user_message=True)
    return result


def validate_final_carry_snapshot(evidence, *, current_context, current_ledger,
        expected_ledger, current_feedback, now_ns, current_radius_fraction,
        expected_radius_fraction):
    """Last read-only check after image work, before the selection-only call."""
    now_ns=_ns(now_ns,'now')
    frame=evidence['fresh_frame']
    if current_context != frame['source_context'] or digest(current_context) != frame['source_id']:
        raise ValueError('geometry_context_changed_during_rgb_association')
    if current_ledger != expected_ledger:
        raise ValueError('episode_changed_during_rgb_association')
    if (isinstance(current_radius_fraction,bool)
            or current_radius_fraction != expected_radius_fraction):
        raise ValueError('production_rgb_attention_policy_changed_during_association')
    ns,q=_sample(current_feedback)
    if not evidence['current_feedback']['stamp_ns'] <= ns <= now_ns:
        raise ValueError('final_measured_feedback_temporal_binding_invalid')
    require_fresh_source(frame['source_stamp_ns'],now_ns)
    require_fresh_source(ns,now_ns)
    anchor=_sample(evidence['confirmation_receipt']['bound_feedback'])[1]
    if np.max(np.abs(q-anchor)) > POSE_TOLERANCE_RAD:
        raise ValueError('measured_pose_changed_during_rgb_association')


def load_aligned_rgb_reference(transition, record_path, stamp_ns):
    """Read-only archive adapter; every raw image is checked against its record."""
    record=json.loads(Path(record_path).read_text())
    rows=[row for row in record.get('rows',[]) if row.get('source_stamp_ns') == stamp_ns]
    if len(rows)!=1:
        raise ValueError('exact_aligned_source_record_required')
    row=rows[0]; path=Path(row['source_file'])
    if hashlib.sha256(path.read_bytes()).hexdigest() != _sha(row.get('source_file_sha256'),'source_file_sha256'):
        raise ValueError('aligned_source_file_hash_changed')
    with np.load(path,allow_pickle=False) as source:
        if (int(source['source_stamp_ns']) != stamp_ns
                or not np.array_equal(source['q_source'],_q(row.get('q_source')))):
            raise ValueError('aligned_source_array_binding_changed')
        reference=prepare_aligned_rgb_reference(transition,row,source['color_bgr'],source['depth_raw'])
    reference['record_file_sha256']=hashlib.sha256(Path(record_path).read_bytes()).hexdigest()
    return reference


def latest_confirmed_pair(options, records, *, epoch_ns, selection_ns, now_ns):
    """A newer unbracketed image must not hide a usable measured earlier pair."""
    for stamp, pair in sorted(options, key=lambda item:item[0], reverse=True):
        try:
            require_fresh_source(stamp, now_ns)
            bracket = bracket_feedback(records, stamp, max(epoch_ns, selection_ns))
        except ValueError:
            continue
        return stamp, pair, bracket
    raise ValueError('fresh_measured_bracketed_rgbd_unavailable')


def validate_image_geometry(rgb_shape, depth_shape, rgb_frame, depth_frame, camera):
    expected = (camera.get('height'), camera.get('width'))
    if (any(type(value) is not int or value <= 0 for value in expected)
            or tuple(rgb_shape) != expected+(3,) or tuple(depth_shape) != expected
            or not isinstance(camera.get('frame_id'), str) or not camera['frame_id']
            or rgb_frame != camera['frame_id'] or depth_frame != camera['frame_id']):
        raise ValueError('rgbd_pair_geometry_mismatch')


def main(argv=None):
    args = build_parser().parse_args(argv)
    if not args.reason.strip() or not math.isfinite(args.wait_sec) or not 0 < args.wait_sec <= 30:
        raise ValueError('invalid_reason_or_bounded_wait')
    carry_args=(args.carry_forward_result,args.aligned_observation_record,args.aligned_source_stamp_ns)
    if ((any(value is not None for value in carry_args) and not all(value is not None for value in carry_args))
            or (args.carry_forward_result is not None and args.alignment_receipt is not None)
            or (args.unreserved_transition and args.carry_forward_result is None)
            or (args.maintenance_receipt is not None and args.carry_forward_result is None)):
        raise ValueError('carry_forward_requires_original_result_and_exact_aligned_record_only')
    transition = None if args.carry_forward_result is None else json.loads(args.carry_forward_result.read_text())
    intervening = load_unreserved_transitions(args.unreserved_transition,
        args.carry_forward_result, args.aligned_observation_record, args.aligned_source_stamp_ns)
    reference = None if transition is None else load_aligned_rgb_reference(
        transition,args.aligned_observation_record,args.aligned_source_stamp_ns)
    maintenance = None if args.maintenance_receipt is None else load_maintenance_receipt(args.maintenance_receipt)
    receipt = None if args.alignment_receipt is None else json.loads(args.alignment_receipt.read_text())
    if transition is not None:
        receipt=deepcopy(transition['alignment_receipt'])
    if not MOUNT.is_mount():
        raise RuntimeError('SanStand unavailable')
    out = ROOT/('rgb_observation_new_trial_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'))
    out.mkdir()
    (out/'ros').mkdir()
    os.environ['ROS_LOG_DIR'] = str(out/'ros')
    import rospy
    from sensor_msgs.msg import Image, JointState
    from std_msgs.msg import Header, String
    from cv_bridge import CvBridge
    from alicia_grasp_modes.srv import SetGraspMode
    rospy.init_node('prepare_rgb_acquisition_retry', anonymous=True, disable_signals=True)
    old = parse_selection(rospy.get_param('/grasp_mode/selection'))
    ledger = rospy.get_param('/rgb_acquisition/ledger', {})
    if transition is not None:
        validate_unreserved_transitions(transition,intervening,old,ledger)
    cv2.setNumThreads(1)
    state, pairs, feedback = {}, {}, deque(maxlen=256)
    lock = threading.RLock()
    bridge = CvBridge()
    subscribers = []
    pose_monitor = None if receipt is None else AlignmentPoseMonitor(receipt)
    def image_cb(message, key):
        with lock:
            cache_rgbd_part(pairs, message.header.stamp.to_nsec(), key, message)
    def joint_cb(message):
        values = dict(zip(message.name, message.position))
        if (message.header.frame_id != 'sdk_measured' or len(values) != len(message.name)
                or len(message.name) != len(message.position) or any(n not in values for n in JOINT_NAMES)):
            return
        try:
            q = _q([values[n] for n in JOINT_NAMES]).tolist()
        except ValueError:
            return
        with lock:
            ns = message.header.stamp.to_nsec()
            if ns <= 0:
                return
            if pose_monitor is not None:
                pose_monitor.observe(ns, q)
            feedback.append((ns, q))
    def epoch_cb(message):
        with lock:
            state['epoch_ns'] = message.stamp.to_nsec()
    def snapshot():
        selection = parse_selection(rospy.get_param('/grasp_mode/selection'))
        current_ledger = rospy.get_param('/rgb_acquisition/ledger', {})
        if selection != old or current_ledger != ledger:
            raise ValueError('selection_or_episode_changed_during_retry_preparation')
        if reference is not None:
            cfg_raw=rospy.get_param('/unknown_perception',{})
            cfg=Config(**{key:value for key,value in cfg_raw.items() if key in Config.__dataclass_fields__})
            if cfg.center_radius_fraction != reference['search_policy'].radius_fraction:
                raise ValueError('production_rgb_attention_policy_changed')
        if receipt is None:
            return dict(now_ns=rospy.Time.now().to_nsec(), monotonic_now=time.monotonic())
        with lock:
            pose_monitor.require_stationary()
            epoch = state.get('epoch_ns'); records = list(feedback)
            options = [(ns, dict(pair)) for ns, pair in pairs.items() if len(pair) == 2]
        if not records or not options or not epoch:
            raise ValueError('fresh_rgbd_and_measured_feedback_unavailable')
        context = retry_source_context(old, epoch, rospy.get_param('/camera'),
            rospy.get_param('/handeye', {}), rospy.get_param('/robot_description'))
        ns, pair, (q, lo, hi) = latest_confirmed_pair(options, records,
            epoch_ns=epoch, selection_ns=old['stamp_ns'], now_ns=rospy.Time.now().to_nsec())
        require_rectified_source_after_activation(context['camera'], ns)
        rgb = bridge.imgmsg_to_cv2(pair['rgb'], desired_encoding='bgr8')
        depth = bridge.imgmsg_to_cv2(pair['depth'], desired_encoding='passthrough')
        validate_image_geometry(rgb.shape, depth.shape, pair['rgb'].header.frame_id,
            pair['depth'].header.frame_id, context['camera'])
        with lock:
            pose_monitor.require_stationary()
            if state.get('epoch_ns') != epoch:
                raise ValueError('control_epoch_changed_during_capture')
            latest_ns, latest_q = max(feedback, key=lambda item:item[0])
        if np.max(np.abs(np.asarray(latest_q)-q)) > POSE_TOLERANCE_RAD:
            raise ValueError('arm_moved_during_capture')
        frame = dict(source_stamp_ns=ns, source_context=context, source_id=digest(context),
            q_source=q.tolist(), accepted_before=lo, accepted_after=hi,
            source_rgb_sha256=hashlib.sha256(np.ascontiguousarray(rgb).tobytes()).hexdigest(),
            source_depth_sha256=hashlib.sha256(np.ascontiguousarray(depth).tobytes()).hexdigest())
        evidence = dict(now_ns=rospy.Time.now().to_nsec(), monotonic_now=time.monotonic(), epoch_ns=epoch,
            current_feedback=dict(stamp_ns=latest_ns, positions=latest_q, frame_id='sdk_measured'),
            fresh_frame=frame, confirmation_receipt=receipt)
        if transition is not None:
            evidence.update(_current_bgr=rgb,_current_depth_raw=depth)
            if maintenance is not None:
                evidence['maintenance_receipt'] = maintenance
        return evidence
    def validate_snapshot(evidence):
        if transition is None:
            return validate_retry_preparation(ledger,old,**evidence)
        rgb=evidence.pop('_current_bgr'); depth=evidence.pop('_current_depth_raw')
        result=validate_carry_forward_preparation(transition,old,ledger,reference=reference,
            current_bgr=rgb,current_depth_raw=depth,unreserved_transitions=intervening,**evidence)
        # Recheck after image work. The original .8 s limit covers the complete
        # preparation, including appearance computation, without resetting time.
        final_selection=parse_selection(rospy.get_param('/grasp_mode/selection'))
        final_ledger=rospy.get_param('/rgb_acquisition/ledger',{})
        with lock:
            final_epoch=state.get('epoch_ns')
        final_context=retry_source_context(final_selection,final_epoch,rospy.get_param('/camera'),
            rospy.get_param('/handeye',{}),rospy.get_param('/robot_description'))
        final_config_raw=rospy.get_param('/unknown_perception',{})
        final_config=Config(**{key:value for key,value in final_config_raw.items()
            if key in Config.__dataclass_fields__})
        with lock:
            pose_monitor.require_stationary()
            if state.get('epoch_ns') != final_epoch:
                raise ValueError('control_epoch_changed_during_rgb_association')
            final_ns,final_q=max(feedback,key=lambda item:item[0])
        validate_final_carry_snapshot(evidence,current_context=final_context,
            current_ledger=final_ledger,expected_ledger=ledger,
            current_feedback=dict(stamp_ns=final_ns,positions=final_q,frame_id='sdk_measured'),
            current_radius_fraction=final_config.center_radius_fraction,
            expected_radius_fraction=reference['search_policy'].radius_fraction,
            now_ns=rospy.Time.now().to_nsec())
        result['final_feedback_stamp_ns']=final_ns
        return result
    report = dict(previous_selection=old, previous_ledger=ledger, new_selection=None,
        reason=args.reason, alignment_receipt=receipt, preview_only=not args.apply,
        original_budget_modified=False, motion_commands_sent=False, torque_commands_sent=False)
    if transition is not None:
        report.update(carry_forward_result_file=str(args.carry_forward_result),
            unreserved_transition_files=[dict(path=str(path.resolve()),sha256=hashlib.sha256(path.read_bytes()).hexdigest())
                for path in args.unreserved_transition],
            carry_forward_result_file_sha256=hashlib.sha256(args.carry_forward_result.read_bytes()).hexdigest(),
            aligned_observation_record=str(args.aligned_observation_record),
            aligned_record_file_sha256=reference['record_file_sha256'],
            aligned_source_stamp_ns=args.aligned_source_stamp_ns,
            confirmation_continues_from_original_user_message=True,
            historical_continuous_stationarity_claimed=False)
    if maintenance is not None:
        report.update(maintenance_receipt=maintenance,
            maintenance_receipt_file=str(args.maintenance_receipt.resolve()),
            maintenance_receipt_file_sha256=hashlib.sha256(args.maintenance_receipt.read_bytes()).hexdigest(),
            maintenance_actuation_scope='positive_enable_only; no joint motion or torque disable',
            torque_commands_sent_scope='this selection-only preparation tool; maintenance enable is recorded separately')
    try:
        # Missing confirmation after movement fails before subscribing/calling a service.
        if receipt is None:
            validate_retry_preparation(ledger, old, now_ns=rospy.Time.now().to_nsec(),
                monotonic_now=time.monotonic())
        if receipt is not None:
            subscribers = [rospy.Subscriber('/supervisor/camera/'+topic+'/image_raw', Image,
                lambda message, k=key:image_cb(message,k), queue_size=2, buff_size=2**24)
                for topic,key in (('color','rgb'),('depth','depth'))]
            subscribers.extend((rospy.Subscriber('/alicia_d/accepted_joint_states', JointState,
                joint_cb, queue_size=50), rospy.Subscriber('/alicia_d/control_reference_epoch', Header,
                epoch_cb, queue_size=2)))
        if args.apply:
            rospy.wait_for_service('/grasp_mode/select', timeout=args.wait_sec)
        deadline = time.monotonic()+args.wait_sec
        while True:
            try:
                evidence = snapshot()
                validation = validate_snapshot(evidence)
                break
            except ValueError:
                if time.monotonic() >= deadline or rospy.is_shutdown():
                    raise
                time.sleep(.05)
        report.update(validation=validation, evidence=evidence)
        if args.apply:
            # No disk IO between the final live check and selection change.
            evidence = snapshot()
            report['validation'] = validate_snapshot(evidence)
            report['evidence'] = evidence
            response = rospy.ServiceProxy('/grasp_mode/select', SetGraspMode)('unknown','observe_return')
            report['response'] = dict(success=response.success, message=response.message)
            if not response.success:
                raise RuntimeError('new_selection_rejected')
            deadline = time.monotonic()+args.wait_sec
            while time.monotonic() < deadline:
                new = parse_selection(rospy.get_param('/grasp_mode/selection'))
                if (new['mode'] == 'unknown' and new['strategy'] == 'observe_return'
                        and new['generation'] == old['generation']+1 and new['stamp_ns'] > old['stamp_ns']):
                    report['new_selection'] = new
                    break
                time.sleep(.05)
            if report['new_selection'] is None:
                raise RuntimeError('new_selection_not_acknowledged')
    except Exception as exc:
        report['failure'] = str(exc)
        raise
    finally:
        for subscriber in subscribers:
            subscriber.unregister()
        if not MOUNT.is_mount():
            raise RuntimeError('SanStand disappeared; no system disk fallback')
        (out/'result.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        print(json.dumps(dict(path=str(out), new_selection=report['new_selection'],
            preview_only=report['preview_only'], failure=report.get('failure'))))


if __name__ == '__main__':
    main()
