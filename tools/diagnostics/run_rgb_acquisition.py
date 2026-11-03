#!/usr/bin/env python3
"""Run a bounded non-contact observation episode through its gateway.

The gateway owns trajectory admission and execution. This tool only selects a
small informative FK proposal from live RGB interest and records the receipt.
It never publishes joints, a gripper command, or fabricated target geometry.
"""
import argparse
from copy import deepcopy
from dataclasses import fields
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import threading
import time
import uuid

import numpy as np

from alicia_grasp_modes.rgb_observation_source import digest, require_fresh_source
from alicia_grasp_modes.rgb_observation_set import observation_set_metadata_digest
from alicia_grasp_modes.rgb_observation_volume import RGBObservationVolume, SupportPlaneProvenance
from alicia_grasp_modes.rgb_observation_views import rank_rgb_observation_candidates
from alicia_grasp_modes.selection import parse_selection
from alicia_grasp_modes.rgb_prefix_preflight import validate_prefix_preflight_receipt
from alicia_flexible_grasp.robot.observation_view_limits import effective_observation_view_limits
from preview_rgb_observation import local_joint_candidates, NAMES

MOUNT = Path('/media/zhuyupei/SanStand')
ROOT = MOUNT/'实验记录/Robot-Controller/ros_log'
SERVICE = '/supervisor/plan_and_execute_rgb_acquisition_pose_strict'
MAX_TRANSIENT_CAPTURE_FAILURES = 3
TRANSIENT_CAPTURE_MESSAGE = ('RGB_ACQUISITION_REJECTED: RGB_ACQUISITION_INITIAL_CAPTURE_UNAVAILABLE: '
                             'RGB_ACQUISITION_FRESH_READY_RGB_D_SOURCE_REQUIRED')


class StatusDiagnostics:
    """Bounded scalar-only telemetry; never used to admit a motion or a target."""
    MAX_REASON_KEYS = 32
    MAX_TEXT_LENGTH = 256

    def __init__(self):
        self.status_count = 0
        self.ready_status_count = 0
        self.malformed_status_count = 0
        self.nonready_reasons = {}
        self.detector_details = {}
        self.last_status = {}

    @classmethod
    def _scalars(cls, value, names):
        if not isinstance(value, dict):
            return {}
        result = {}
        for name in names:
            item = value.get(name)
            if isinstance(item, str):
                result[name] = item[:cls.MAX_TEXT_LENGTH]
            elif item is None or type(item) in (bool, int):
                result[name] = item
            elif type(item) is float and math.isfinite(item):
                result[name] = item
        return result

    @classmethod
    def _count(cls, counts, reason):
        key = reason[:cls.MAX_TEXT_LENGTH] if isinstance(reason, str) else '<missing>'
        if key not in counts and len(counts) >= cls.MAX_REASON_KEYS:
            key = '<other_reasons>'
        counts[key] = counts.get(key, 0) + 1

    def observe(self, status):
        if not isinstance(status, dict):
            self.malformed_status_count += 1
            return
        self.status_count += 1
        interest = status.get('observation_interest')
        if not isinstance(interest, dict):
            interest = {}
        ready = interest.get('observation_interest_ready') is True
        self.ready_status_count += int(ready)
        if not ready:
            self._count(self.nonready_reasons, interest.get('reason'))
        self._count(self.detector_details, status.get('detail'))
        summary = self._scalars(status, ('state', 'detail', 'generation',
            'source_stamp_ns', 'color_stamp_ns', 'depth_stamp_ns', 'processing_seconds'))
        summary['observation_interest'] = self._scalars(interest, ('reason',
            'observation_interest_ready', 'source_stamp_ns', 'source_id', 'generation',
            'reference_id', 'rgb_candidate_count', 'source_age_at_status_sec',
            'processing_seconds'))
        summary['observation_interest']['rgb_track'] = self._scalars(
            interest.get('rgb_track'), ('confirmed', 'reason', 'consistent_frames'))
        summary['rgb_acquisition_handoff'] = self._scalars(
            status.get('rgb_acquisition_handoff'), ('state', 'reason', 'detail',
                'reference_id', 'source_stamp_ns', 'observation_index'))
        self.last_status = summary

    def snapshot(self):
        return deepcopy(dict(status_count=self.status_count,
            ready_status_count=self.ready_status_count,
            malformed_status_count=self.malformed_status_count,
            nonready_reasons=self.nonready_reasons,
            detector_details=self.detector_details, last_status=self.last_status))


def unreserved_capture_retry(message, before, after, request, *, now_ns, monotonic_now):
    """Only the observed pre-reservation fresh-frame timeout can be retried.

    Exact ledger equality proves this RPC did not charge a reservation or
    advance execution. This is not permission to revive a retired reference.
    """
    if (message != TRANSIENT_CAPTURE_MESSAGE or not isinstance(before,dict)
            or not before or before != after
            or before.get('phase') not in ('awaiting_measured_target','planning_rejected','preflight_rejected')):
        return False
    try:
        if before['phase'] == 'awaiting_measured_target':
            identity_ok = (request.get('request_id') == before['request_id']
                and request.get('reference_id') == before['reference_id'] and bool(before['reference_id'])
                and type(before['observation_index']) is int
                and request.get('observation_index') == before['observation_index']+1)
        elif before['phase'] == 'preflight_rejected':
            validate_prefix_preflight_receipt(before, now_ns=now_ns, monotonic_now=monotonic_now)
            reservation = request.get('reservation_request_id')
            identity_ok = (request.get('request_id') == before['request_id']
                and request.get('reference_id') == before['reference_id']
                and request.get('observation_index') == 1
                and type(reservation) is str and bool(reservation)
                and reservation not in before['request_ids'])
        else:
            # A pre-send rejected plan retired its reference. The pending,
            # unreserved request must freeze a NEW one from fresh evidence.
            identity_ok = (before.get('actuation_prefix_attempted') is not True
                and not request.get('reference_id') and 'observation_index' not in request
                and type(request.get('request_id')) is str and bool(request['request_id'])
                and request['request_id'] not in before['request_ids'])
        return bool(identity_ok and request.get('expected_source_id') == before['source_id']
            and parse_selection(request.get('selection')) == parse_selection(before['selection'])
            and type(before['attempts']) is int and 1 <= before['attempts'] < before['max_observations']
            and request.get('duration_sec') == before['duration_sec']
            and request.get('max_observations') == before['max_observations']
            and int(before['created_ns']) <= now_ns < int(before['deadline_ns'])
            and before['created_monotonic'] <= monotonic_now < before['deadline_monotonic'])
    except (KeyError,ValueError,TypeError,OverflowError):
        return False


def charged_reservations(before, after):
    """Report charged planner reservations separately from RPC transport calls."""
    if not isinstance(after,dict) or type(after.get('attempts')) is not int:
        return 0
    same_episode = isinstance(before,dict) and before.get('created_ns') == after.get('created_ns')
    prior = before.get('attempts',0) if same_episode else 0
    return max(0,after['attempts']-prior) if type(prior) is int else 0


class ObservedViewHistory:
    """At most initial + three fresh, receipt-bound observed camera poses.

    `observe_verified` is called only after rank_live_interest's source/FK
    verification. Successful RPC receipts enable a later frame; planned poses
    and receipts alone never append a camera pose or prove information gain.
    """
    def __init__(self):
        self.clear()

    def clear(self):
        self.source_id = None
        self.selection = None
        self.reference_id = None
        self.receipt = None
        self.preflight_receipt = None
        self.records = []

    def bind_success(self, ledger):
        selection = parse_selection(ledger.get('selection'))
        reference = ledger.get('reference_id')
        index = ledger.get('observation_index')
        if (ledger.get('phase') != 'awaiting_measured_target'
                or type(reference) is not str or not reference
                or type(index) is not int or not 1 <= index <= 3
                or int(ledger.get('settled_ns',0)) <= 0):
            raise ValueError('observed_view_success_receipt_required')
        if (ledger.get('source_id') != self.source_id or selection != self.selection
                or self.reference_id not in (None,reference)):
            self.clear()
            raise ValueError('observed_view_receipt_context_changed')
        self.reference_id = reference
        self.preflight_receipt = None
        self.receipt = dict(source_id=self.source_id,selection=deepcopy(selection),
            reference_id=reference,observation_index=index,
            settled_ns=int(ledger['settled_ns']),request_id=ledger['request_id'])

    def bind_preflight(self, ledger):
        proof = validate_prefix_preflight_receipt(ledger)
        if (proof['source_id'] != self.source_id
                or parse_selection(proof['selection']) != self.selection
                or self.reference_id not in (None,proof['reference_id'])
                or self.receipt is not None):
            raise ValueError('preflight_view_reference_changed')
        self.reference_id = proof['reference_id']
        self.preflight_receipt = proof

    def observe_verified(self, interest):
        selection = parse_selection(interest['source_context']['selection'])
        source_id = interest['source_id']
        reference = interest.get('reference_id') or None
        if source_id != self.source_id or selection != self.selection:
            self.clear()
            self.source_id,self.selection = source_id,deepcopy(selection)
        if reference != self.reference_id:
            self.clear()
            self.source_id,self.selection = source_id,deepcopy(selection)
            if reference is not None:
                raise ValueError('observed_view_unbound_reference')
        index = 0
        if reference is not None:
            receipt = self.preflight_receipt or self.receipt
            if self.preflight_receipt is not None:
                if (interest['source_stamp_ns'] <= int(receipt['last_capture_source_stamp_ns'])
                        or not interest.get('association_sha256')
                        or np.max(np.abs(np.asarray(interest['q_source'])-receipt['q_source'])) > .004):
                    raise ValueError('preflight_view_requires_fresh_original_reference_association')
            elif (receipt is None or receipt['source_id'] != source_id
                    or receipt['selection'] != selection or receipt['reference_id'] != reference
                    or interest['source_stamp_ns'] <= receipt['settled_ns']
                    or not interest.get('association_sha256')):
                raise ValueError('observed_view_requires_fresh_post_settle_association')
            else:
                index = receipt['observation_index']
        if not any(row['observation_index']==index for row in self.records):
            if len(self.records)>=4:
                raise ValueError('observed_view_history_budget_exhausted')
            self.records.append(dict(observation_index=index,
                source_stamp_ns=interest['source_stamp_ns'],
                T_base_camera=deepcopy(interest['T_base_camera']),
                origin='initial_verified_rgb_frame' if index==0 else 'verified_post_settle_rgb_frame'))
        return tuple(deepcopy(row['T_base_camera']) for row in self.records)

    def snapshot(self):
        return deepcopy(dict(source_id=self.source_id,selection=self.selection,
            reference_id=self.reference_id,observations=self.records,
            measured_information_gain=False,motion_authorized=False))


def bound_frozen_reference_vertices(interest, ledger, *, now_ns):
    """Return original vertices from settled or proved-unsubmitted ownership.

    Current-view frusta are deliberately not substitutes for the initial
    reference. This validates ownership/provenance, not motion authority.
    """
    reference = interest.get('reference_id')
    if not reference:
        if (isinstance(ledger, dict) and ledger.get('phase') in ('awaiting_measured_target','preflight_rejected')
                and ledger.get('source_id') == interest['source_id']
                and parse_selection(ledger.get('selection')) == parse_selection(interest['source_context']['selection'])):
            raise ValueError('frozen_reference_missing_from_continuation')
        return ()
    try:
        preflight = isinstance(ledger, dict) and ledger.get('phase') == 'preflight_rejected'
        if preflight:
            proof = validate_prefix_preflight_receipt(ledger, now_ns=now_ns)
            source_floor = int(proof['last_capture_source_stamp_ns'])
            if (interest['epoch_ns'] != int(proof['epoch_ns'])
                    or np.max(np.abs(np.asarray(interest['q_source'])-proof['q_source'])) > .004):
                raise ValueError('frozen_preflight_source_changed')
        else:
            source_floor = int(ledger['settled_ns'])
        if (not isinstance(ledger, dict) or ledger.get('phase') not in ('awaiting_measured_target','preflight_rejected')
                or ledger.get('reference_id') != reference
                or ledger.get('source_id') != interest['source_id']
                or parse_selection(ledger.get('selection')) != parse_selection(interest['source_context']['selection'])
                or type(ledger.get('request_id')) is not str or not ledger['request_id']
                or type(ledger.get('observation_index')) is not int
                or not 1 <= ledger['observation_index'] <= 3
                or not 0 < source_floor < interest['source_stamp_ns'] <= now_ns < int(ledger['deadline_ns'])
                or not interest.get('association_sha256')
                or interest.get('rgb_search_basis') != 'fixed_reference_volume_projection'):
            raise ValueError('frozen_reference_owner_binding_changed')
        raw_envelope = ledger['reference_volume_json']
        raw_members = ledger['reference_volumes_json']
        if any(type(value) is not str or len(value) > 131072 for value in (raw_envelope,raw_members)):
            raise ValueError('frozen_reference_payload_invalid')
        envelope, members = json.loads(raw_envelope), json.loads(raw_members)
        set_sha = ledger['reference_observation_set_sha256']
        if (type(set_sha) is not str or len(set_sha) != 64
                or any(value not in '0123456789abcdef' for value in set_sha)
                or not isinstance(envelope, dict) or not isinstance(members, list)
                or not 1 <= len(members) <= 4 or len(members) != len(interest['interest_volumes'])
                or set_sha != interest.get('association_reference_set_sha256')
                or digest(envelope) != digest(interest.get('association_reference_volume'))
                or digest(members) != digest(interest.get('association_reference_volumes'))):
            raise ValueError('frozen_reference_payload_changed')
        current = interest['interest_volume']
        result = []
        for item in [envelope]+members:
            # Use serialized vertices verbatim. Reconstructing the volume would
            # recompute them and silently substitute different source geometry.
            vertices = np.asarray(item['vertices_base_m'])
            if (vertices.shape != (8,3) or vertices.dtype.kind not in 'fiu'
                    or not np.all(np.isfinite(vertices))
                    or not 0 < item['source_stamp_ns'] <= source_floor
                    or item['source_stamp_ns'] != envelope['source_stamp_ns']
                    or any(digest(item[key]) != digest(current[key]) for key in (
                        'image_shape','camera_matrix','working_depth_range_m','foreground_max_m'))):
                raise ValueError('frozen_reference_geometry_changed')
            result.append(tuple(tuple(float(value) for value in row) for row in vertices))
        return tuple(result)
    except (KeyError, TypeError, OverflowError) as exc:
        raise ValueError('frozen_reference_owner_evidence_missing') from exc


def rank_live_interest(interest, *, now_ns, selection, description, joint_min, joint_max,
                       view_history=None, reference_ledger=None):
    """Verify prediction provenance, not motion authority or collision safety."""
    from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
    if interest.get('observation_interest_ready') is not True:
        raise ValueError('stable_rgb_interest_unavailable')
    stamp = interest['source_stamp_ns']
    require_fresh_source(stamp, now_ns)
    context = interest['source_context']
    selected = parse_selection(selection)
    track = interest['rgb_track']
    if (parse_selection(context['selection']) != selected
            or selected['mode'] != 'unknown' or selected['strategy'] != 'observe_return'
            or interest['source_id'] != digest(context)
            or interest['generation'] != selected['generation']
            or interest['selection_stamp_ns'] != selected['stamp_ns']
            or interest['epoch_ns'] != context['epoch_ns']
            or stamp <= max(selected['stamp_ns'], context['epoch_ns'])
            or track['confirmed'] is not True
            or track['source_id'] != interest['source_id']
            or track['generation'] != selected['generation']
            or len(track['source_stamps_ns']) < 3
            or track['source_stamps_ns'][-1] != stamp
            or any(a >= b for a,b in zip(track['source_stamps_ns'],track['source_stamps_ns'][1:]))):
        raise ValueError('rgb_interest_provenance_changed')
    if hashlib.sha256(description.encode()).hexdigest() != context['model_sha256']:
        raise ValueError('robot_model_changed')
    members = interest.get('rgb_interest_members')
    if observation_set_metadata_digest(members) != interest.get('observation_set_sha256'):
        raise ValueError('rgb_interest_set_digest_changed')
    raw_volumes = interest.get('interest_volumes')
    if not isinstance(raw_volumes,(tuple,list)) or len(raw_volumes) != len(members):
        raise ValueError('rgb_interest_set_volumes_missing')
    if interest.get('region_mask_sha256') != members[0]['mask_sha256']:
        raise ValueError('rgb_interest_set_primary_member_changed')
    raw = interest['interest_volume']
    args = {field.name: raw[field.name] for field in fields(RGBObservationVolume) if field.init}
    args['support_provenance'] = SupportPlaneProvenance(**args['support_provenance'])
    volume = RGBObservationVolume(**args)
    if (volume.source_stamp_ns != stamp
            or not np.array_equal(volume.T_base_camera, interest['T_base_camera'])
            or not np.allclose(volume.vertices_base_m, raw['vertices_base_m'], rtol=0., atol=1e-12)):
        raise ValueError('rgb_interest_volume_changed')
    member_volumes = []
    for member, raw_member in zip(members,raw_volumes):
        parameters = {field.name:raw_member[field.name] for field in fields(RGBObservationVolume) if field.init}
        parameters['support_provenance'] = SupportPlaneProvenance(**parameters['support_provenance'])
        item = RGBObservationVolume(**parameters)
        if (tuple(item.bbox_xywh) != tuple(member['bbox_xywh'])
                or tuple(item.image_shape) != tuple(member['image_shape'])
                or item.source_stamp_ns != stamp
                or item.support_provenance != volume.support_provenance
                or item.foreground_max_m != volume.foreground_max_m
                or item.working_depth_range_m != volume.working_depth_range_m
                or not np.array_equal(item.camera_matrix,volume.camera_matrix)
                or not np.array_equal(item.T_base_camera,volume.T_base_camera)
                or not np.allclose(item.vertices_base_m,raw_member['vertices_base_m'],rtol=0.,atol=1e-12)):
            raise ValueError('rgb_interest_member_volume_changed')
        member_volumes.append(item)
    boxes = np.asarray([member['bbox_xywh'] for member in members])
    minimum = boxes[:,:2].min(axis=0)
    extent = (boxes[:,:2]+boxes[:,2:]).max(axis=0)-minimum
    if tuple(volume.bbox_xywh) != tuple(np.r_[minimum,extent]):
        raise ValueError('rgb_interest_set_envelope_changed')
    fk = SerialUrdfFk(description, NAMES)
    q = np.asarray(interest['q_source'], dtype=float)
    mount = np.asarray(context['tool_camera'], dtype=float)
    if q.shape != (6,) or not np.all(np.isfinite(q)):
        raise ValueError('invalid_source_joints')
    if not np.allclose(fk(q) @ mount, volume.T_base_camera, rtol=0., atol=1e-6):
        raise ValueError('source_fk_tf_mismatch')
    low = np.maximum(np.asarray(joint_min[:6], float), fk.limits[:, 0])
    high = np.minimum(np.asarray(joint_max[:6], float), fk.limits[:, 1])
    candidates = local_joint_candidates(q, low, high, fk, mount)
    limits = effective_observation_view_limits(context['observation_view_parameters'])
    if volume.working_depth_range_m != (limits['minimum_depth_m'], limits['maximum_depth_m']):
        raise ValueError('observation_working_range_changed')
    frozen_vertices = bound_frozen_reference_vertices(interest, reference_ledger, now_ns=now_ns)
    if view_history is not None and type(view_history) is not ObservedViewHistory:
        raise ValueError('observed_view_history_type_invalid')
    if frozen_vertices and view_history is not None and view_history.receipt is not None:
        receipt = view_history.receipt
        if (any(receipt[key] != reference_ledger.get(key) for key in (
                'request_id','reference_id','source_id','observation_index'))
                or receipt['selection'] != parse_selection(reference_ledger.get('selection'))
                or receipt['settled_ns'] != int(reference_ledger['settled_ns'])):
            raise ValueError('frozen_reference_history_owner_changed')
    observed = () if view_history is None else view_history.observe_verified(interest)
    rank_arguments = dict(
        frozen_reference_vertices=frozen_vertices,
        observed_T_base_cameras=observed,
        current_T_base_camera=volume.T_base_camera, camera_matrix=volume.camera_matrix,
        image_shape=volume.image_shape, working_depth_range_m=volume.working_depth_range_m,
        current_joint_positions=q, current_T_base_tool=fk(q),
        roi_margin_px=limits['roi_margin_px'])
    ranking = rank_rgb_observation_candidates(volume,candidates,**rank_arguments)
    # The envelope is only a conservative ranking proxy. Every independent
    # member still passes all eight-corner full-image visibility/depth tests.
    # This runner's gateway freezes the original reference before motion;
    # subsequent perception searches its projection, never the image centre.
    member_rankings = [rank_rgb_observation_candidates(item,candidates,**rank_arguments)
                       for item in member_volumes]
    allowed = set(row['candidate_id'] for row in ranking['ranked'])
    for member_ranking in member_rankings:
        allowed.intersection_update(row['candidate_id'] for row in member_ranking['ranked'])
    ranking['ranked'] = [row for row in ranking['ranked'] if row['candidate_id'] in allowed]
    ranking.update(observation_set_sha256=interest['observation_set_sha256'],
        reference_id=interest.get('reference_id'),
        original_reference_set_sha256=interest.get('association_reference_set_sha256'),
        rgb_interest_members=deepcopy(members),
        member_visibility_checked=True, member_count=len(member_volumes),
        member_visibility_rankings=member_rankings)
    return ranking, {candidate.candidate_id: candidate for candidate in candidates}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--wait-sec', type=float, default=20.)
    parser.add_argument('--max-plans', type=int, default=3)
    parser.add_argument('--preview-only', action='store_true')
    parser.add_argument('--expected-selection-json', help='Exact confirmed selection; never selects a new generation')
    args = parser.parse_args()
    expected_selection = (None if args.expected_selection_json is None
                          else parse_selection(args.expected_selection_json))
    if not 1 <= args.wait_sec <= 60 or not 1 <= args.max_plans <= 3:
        raise ValueError('finite_observation_budget_required')
    if not MOUNT.is_mount():
        raise RuntimeError('SanStand unavailable; no fallback')
    out = ROOT/('rgb_acquisition_run_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
    out.mkdir(); (out/'ros').mkdir()
    os.environ['ROS_LOG_DIR'] = str(out/'ros')
    import rospy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    from tf.transformations import quaternion_from_matrix
    from alicia_flexible_grasp_supervisor.srv import SetTargetPose, SetTargetPoseRequest
    from alicia_flexible_grasp.robot.observation_path_guard import wire_digest
    rospy.init_node('rgb_acquisition_runner', anonymous=True, disable_signals=True)
    lock = threading.Lock(); recent = {}; diagnostics = StatusDiagnostics()
    def status_cb(message):
        try:
            status = json.loads(message.data)
            value = status.get('observation_interest', {})
        except (ValueError,TypeError,AttributeError):
            with lock:
                diagnostics.malformed_status_count += 1
            return
        with lock:
            diagnostics.observe(status)
            recent['status'] = status
            if value.get('observation_interest_ready') is True:
                recent['interest'] = value
    sub = rospy.Subscriber('/perception/unknown/detector_status', String, status_cb, queue_size=2)
    report = dict(started_at_utc=datetime.now(timezone.utc).isoformat(),
        preview_only=args.preview_only, requests=[], observation_completed=False,
        actuation_prefix_completed=False, requested_observation_target_reached=False,
        measured_target_available=False, grasp_started=False, torque_commands_sent=False,
        rpc_count=0, charged_attempts=0, transient_capture_failures=0,
        max_transient_capture_failures=MAX_TRANSIENT_CAPTURE_FAILURES,
        max_rpc_calls=args.max_plans+MAX_TRANSIENT_CAPTURE_FAILURES)
    selection = parse_selection(rospy.get_param('/grasp_mode/selection'))
    description = rospy.get_param('/robot_description')
    joint_min = rospy.get_param('/robot/joint_min_rad')
    joint_max = rospy.get_param('/robot/joint_max_rad')
    print('SESSION '+str(out), flush=True)
    try:
        if expected_selection is not None and selection != expected_selection:
            raise ValueError('active selection differs from confirmed RGB acquisition')
        if not args.preview_only:
            rospy.wait_for_service(SERVICE, timeout=5.)
        deadline = time.monotonic()+args.wait_sec
        attempted = set()
        settled_ns = 0
        reference_id = None
        owner_request_id = None
        completed_observations = 0
        retry_after_source_ns = 0
        last_reserved_ledger = None
        pending_capture_request_id = None
        pending_reservation_request_id = None
        preflight_ledger = None
        view_history = ObservedViewHistory()
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if parse_selection(rospy.get_param('/grasp_mode/selection')) != selection:
                view_history.clear()
                raise ValueError('selection_changed')
            with lock:
                interest = deepcopy(recent.get('interest'))
                detector = deepcopy(recent.get('status', {}))
            handoff = detector.get('rgb_acquisition_handoff') or {}
            if (settled_ns and detector.get('state') == 'ready'
                    and handoff.get('state') == 'measured_ready'
                    and handoff.get('reference_id') == reference_id
                    and int(handoff.get('source_stamp_ns', 0)) > settled_ns):
                report.update(measured_target_available=True, handoff=handoff)
                break
            if interest is None:
                time.sleep(.05); continue
            if report['rpc_count'] >= report['max_rpc_calls']:
                report['failure'] = 'rpc_call_budget_exhausted'
                break
            if interest.get('source_stamp_ns',0) <= retry_after_source_ns:
                time.sleep(.05); continue
            if settled_ns and (interest.get('reference_id') != reference_id
                    or interest.get('source_stamp_ns', 0) <= settled_ns
                    or not interest.get('association_sha256')):
                report['latest_handoff'] = handoff
                time.sleep(.05); continue
            if preflight_ledger is not None and (interest.get('reference_id') != reference_id
                    or not interest.get('association_sha256')):
                report['latest_handoff'] = handoff
                time.sleep(.05); continue
            if report['charged_attempts'] >= args.max_plans:
                report['latest_handoff'] = handoff
                time.sleep(.05); continue
            try:
                ranking, candidates = rank_live_interest(interest, now_ns=rospy.Time.now().to_nsec(),
                    selection=selection, description=description, joint_min=joint_min, joint_max=joint_max,
                    view_history=view_history, reference_ledger=last_reserved_ledger)
            except ValueError as exc:
                report['latest_interest_rejection'] = str(exc)
                time.sleep(.05); continue
            report.update(interest=interest, ranking=ranking, observed_view_history=view_history.snapshot())
            # Identical FK moves at a different origin are distinct. Never
            # retry the same proposal from the same measured pose.
            origin_key = tuple(np.round(interest['q_source'], 5))
            remaining = [row for row in ranking['ranked']
                         if (origin_key, row['candidate_id']) not in attempted]
            if not remaining:
                raise ValueError('no_untried_informative_local_view')
            chosen = remaining[0]['candidate_id']; candidate = candidates[chosen]
            if args.preview_only:
                report['selected_candidate'] = chosen
                break
            target = PoseStamped(); target.header.frame_id = 'base_link'
            target.header.stamp = rospy.Time.now()
            tool = np.asarray(candidate.T_base_tool)
            target.pose.position.x,target.pose.position.y,target.pose.position.z = tool[:3,3]
            quat = quaternion_from_matrix(tool)
            target.pose.orientation.x,target.pose.orientation.y,target.pose.orientation.z,target.pose.orientation.w = quat
            request = dict(version=1, request_id=owner_request_id if reference_id is not None
                else pending_capture_request_id or str(uuid.uuid4()),
                expected_source_id=interest['source_id'],
                selection=selection, target_sha256=wire_digest(target), duration_sec=120., max_observations=3)
            if reference_id is not None:
                request.update(request_id=owner_request_id, reference_id=reference_id,
                    observation_index=completed_observations+1)
                if preflight_ledger is not None:
                    request['reservation_request_id'] = pending_reservation_request_id or str(uuid.uuid4())
            entry = dict(candidate_id=chosen, attempt_id=str(uuid.uuid4()), request=request, target_matrix=tool.tolist())
            report['requests'].append(entry)
            with lock:
                report['status_diagnostics'] = diagnostics.snapshot()
            (out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
            if not MOUNT.is_mount():
                raise RuntimeError('SanStand unavailable before request')
            # XMLRPC parameters only carry signed int32; selection parsers
            # explicitly accept decimal strings for nanosecond timestamps.
            wire_request = deepcopy(request)
            wire_request['selection']['stamp_ns'] = str(selection['stamp_ns'])
            ledger_before = deepcopy(rospy.get_param('/rgb_acquisition/ledger', {}))
            rospy.set_param('/rgb_acquisition/request', wire_request)
            attempted.add((origin_key, chosen))
            report['rpc_count'] += 1
            response = rospy.ServiceProxy(SERVICE, SetTargetPose)(SetTargetPoseRequest(target=target, execute=True))
            entry.update(success=bool(response.success), message=response.message,
                ledger=rospy.get_param('/rgb_acquisition/ledger', {}))
            entry['charged_attempts'] = charged_reservations(ledger_before,entry['ledger'])
            report['charged_attempts'] += entry['charged_attempts']
            if entry['charged_attempts']:
                last_reserved_ledger = deepcopy(entry['ledger'])
                pending_capture_request_id = None
                pending_reservation_request_id = None
                if 'original_episode_deadline' not in report:
                    report['original_episode_deadline'] = {key:deepcopy(entry['ledger'].get(key)) for key in (
                        'created_ns','created_monotonic','deadline_ns','deadline_monotonic',
                        'request_id','reference_id','source_id')}
                deadline = min(deadline,float(entry['ledger']['deadline_monotonic']))
            print(json.dumps(entry), flush=True)
            if response.success:
                preflight_ledger = None
                # A proved actuation prefix is a settled camera movement, not
                # arrival at the requested observation pose. Continue using
                # this exact owner/deadline and fresh post-settle evidence.
                prefix_only = entry['ledger'].get('receipt') == 'actuation_prefix_confirmed_requires_fresh_replan'
                entry['actuation_prefix_only'] = prefix_only
                entry['requested_target_reached'] = (not prefix_only and
                    entry['ledger'].get('receipt') ==
                    'noncontact_observation_completed_requires_new_measured_target_and_rgb_association')
                report['actuation_prefix_completed'] |= prefix_only
                report['requested_observation_target_reached'] |= entry['requested_target_reached']
                view_history.bind_success(entry['ledger'])
                report['observed_view_history'] = view_history.snapshot()
                report['observation_completed'] = True
                settled_ns = int(entry['ledger']['settled_ns'])
                reference_id = entry['ledger']['reference_id']
                owner_request_id = entry['ledger']['request_id']
                completed_observations = int(entry['ledger']['observation_index'])
                last_reserved_ledger = deepcopy(entry['ledger'])
                # The gateway keeps the original episode deadline; waiting
                # here never extends it or resets the per-selection budget.
                deadline = min(time.monotonic()+args.wait_sec,
                    float(entry['ledger']['deadline_monotonic']))
                continue
            if ledger_before == last_reserved_ledger and unreserved_capture_retry(response.message,
                    ledger_before,entry['ledger'],request,now_ns=rospy.Time.now().to_nsec(),
                    monotonic_now=time.monotonic()):
                report['transient_capture_failures'] += 1
                entry['unreserved_capture_failure'] = True
                if report['transient_capture_failures'] >= MAX_TRANSIENT_CAPTURE_FAILURES:
                    report['failure'] = 'transient_capture_failure_budget_exhausted'
                    break
                # No reservation or motion occurred. Preserve the original
                # owner/deadline and wait for an actually newer ready source.
                attempted.discard((origin_key,chosen))
                if reference_id is None:
                    pending_capture_request_id = request['request_id']
                elif preflight_ledger is not None:
                    pending_reservation_request_id = request['reservation_request_id']
                retry_after_source_ns = max(int(interest['source_stamp_ns']),rospy.Time.now().to_nsec())
                entry['retry_requires_source_after_ns'] = str(retry_after_source_ns)
                continue
            # Only a pre-submission rejection permits trying another pose.
            # Any execution/owner change is terminal for this one-move runner.
            ledger = entry['ledger']
            if ledger.get('phase') == 'preflight_rejected' and entry['charged_attempts'] == 1:
                proof = validate_prefix_preflight_receipt(ledger,
                    now_ns=rospy.Time.now().to_nsec(), monotonic_now=time.monotonic())
                if (proof['request_id'] != request['request_id']
                        or proof['failed_reservation_request_id'] !=
                            request.get('reservation_request_id',request['request_id'])
                        or (reference_id is not None and proof['reference_id'] != reference_id)):
                    raise ValueError('preflight_receipt_request_changed')
                view_history.bind_preflight(ledger)
                preflight_ledger = deepcopy(ledger)
                reference_id,owner_request_id = proof['reference_id'],proof['request_id']
                retry_after_source_ns = int(proof['last_capture_source_stamp_ns'])
                entry['preflight_rejected_without_submission'] = True
                continue
            if (reference_id is not None or ledger.get('phase') != 'planning_rejected'
                    or ledger.get('actuation_prefix_attempted') is True
                    or entry['charged_attempts'] == 0):
                # A failure after a completed view retires that reference in
                # perception. Do not reopen it or replay an old association.
                break
        if not report['requests'] and not args.preview_only:
            report['failure'] = 'no_live_observation_interest_within_budget'
    except Exception as exc:
        report['failure'] = str(exc)
    finally:
        if 'view_history' in locals():
            report['observed_view_history'] = view_history.snapshot()
        report['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
        with lock:
            report['status_diagnostics'] = diagnostics.snapshot()
        (out/'result.json').write_text(json.dumps(report, indent=2)+'\n')
        sub.unregister()
    print(json.dumps({k:v for k,v in report.items() if k not in ('interest','ranking')},indent=2),flush=True)
    return 0 if report['observation_completed'] or (args.preview_only and 'selected_candidate' in report) else 2


if __name__ == '__main__':
    raise SystemExit(main())
