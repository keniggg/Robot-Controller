"""Task-owned observe/return handoff; motion remains in the strict gateway."""
from copy import deepcopy
import hashlib
import json
import math
import os
import re
import threading
import time
import uuid

import rospy
from std_msgs.msg import String
from alicia_flexible_grasp_supervisor.srv import CompensatePregrasp
from alicia_flexible_grasp.grasp.grasp_state_machine import GraspStages
from alicia_flexible_grasp.grasp.frozen_contact_handoff import (
    FROZEN_CONTACT_TOPIC, build_contact_freeze_notice)
from alicia_flexible_grasp.robot.observation_path_guard import committed_plan_digest
from alicia_flexible_grasp.grasp.rich_plan_integrity import pose_values
from alicia_flexible_grasp.grasp.stable_recheck_contract import (
    CODE as STABLE_RECHECK_REJECTED, completed_stable_recheck_proof_valid)
from alicia_grasp_modes.selection import parse_selection
from alicia_grasp_modes.contact_search_yield import contact_search_yield_proof_valid
from alicia_grasp_modes.contact_station_task import ContactStationTaskMixin
from alicia_grasp_modes.observe_return_session import exact_ns, parse_session, VIEW_FIELDS
from alicia_grasp_modes.active_observation_policy import observation_budget, evidence_digest, information_gain_valid
from alicia_grasp_modes.observe_return_measurement import (
    REQUEST_TOPIC as MEASUREMENT_REQUEST_TOPIC, ACK_TOPIC as MEASUREMENT_ACK_TOPIC,
    PARTICIPANTS, parse_measurement_request, validate_measurement_ack,
    bootstrap_source_deadline)
from alicia_grasp_modes.runner_contract import (observe_return_audit_error, audit_matches_preview,
                                               observed_contact_audit_error,
                                               unknown_observation_audit_error,
                                               surface_view_required_search_proof_valid)


_EXPERIMENT_MOUNT = '/media/zhuyupei/SanStand'
_EXPERIMENT_ROOT = _EXPERIMENT_MOUNT + '/实验记录/Robot-Controller'
_SILHOUETTE_TRIGGER = 'RGB_DEPTH_SILHOUETTE_AMBIGUOUS'


def positive_ns(value):
    if isinstance(value, str) and value.isdecimal():
        value = int(value)
    if type(value) is not int or value <= 0:
        raise ValueError('positive exact source timestamp required')
    return value


class ObserveReturnTask(ContactStationTaskMixin):
    def _initialize_observe_return_state(self):
        self._observe_return_condition = threading.Condition(threading.RLock())
        self._observe_return_reports = {}
        self._observe_return_report_references = {}
        self._diagnostic_observation_consumed_sources = set()
        self._observe_return_acks = {}
        self._observation_measurement_acks = {}
        self._observation_measurement_request = None
        self._clear_observe_return_attempt()

    def _clear_observe_return_attempt(self):
        """Retire one finished attempt without discarding a fresh plan audit."""
        self._observation_measurement_revoke('task attempt finished')
        self._contact_station_retired_measurement = None
        with self._observe_return_condition:
            self._observe_return_session = None
            self._observe_return_session_clock = None
            self._observe_return_entry_phase = None
            self._observe_return_origin_proof = None
            self._observe_return_origin_plan = None
            self._observe_return_active_view_plan = None
            self._observe_return_submitted_view_requests = set()
            self._observe_return_bound_report = None
            self._observe_return_bound_reference = None
            self._diagnostic_observation_sample_reference = None
            self._observe_return_acks.clear()
            self._observation_measurement_acks.clear()
            self._observation_measurement_request = None
            self._observation_measurement_prepared = None
            self._qualified_station_phase = None
            self._qualified_station_receipt = None
            self._actuation_bootstrap_context = None
            self._actuation_bootstrap_receipt = None
            self._actuation_bootstrap_retired_context = None

    def _setup_observe_return_io(self):
        self._setup_contact_station_io()
        self._frozen_contact_pub = rospy.Publisher(
            FROZEN_CONTACT_TOPIC, String, queue_size=4, latch=False)
        self._observe_return_pub = rospy.Publisher(
            '/grasp_mode/observe_return_session', String, queue_size=4, latch=True)
        self._observe_return_ack_sub = rospy.Subscriber(
            '/grasp_mode/observe_return_ack', String, self._observe_return_ack_cb, queue_size=8)
        self._observe_return_audit_sub = rospy.Subscriber(
            '/grasp_6d/gate_audit', String, self._observe_return_audit_cb, queue_size=8)
        self._observation_measurement_pub = rospy.Publisher(
            MEASUREMENT_REQUEST_TOPIC, String, queue_size=8, latch=True)
        self._observation_measurement_sub = rospy.Subscriber(
            MEASUREMENT_ACK_TOPIC, String, self._observation_measurement_ack_cb, queue_size=16)

    def _observation_measurement_ack_cb(self, msg):
        try:
            ack = json.loads(msg.data)
            if (not isinstance(ack, dict) or ack.get('participant') not in PARTICIPANTS
                    or not isinstance(ack.get('request_id'), str)):
                return
            with self._observe_return_condition:
                self._observation_measurement_acks[(ack['request_id'], ack['participant'])] = ack
                while len(self._observation_measurement_acks) > 32:
                    del self._observation_measurement_acks[next(iter(self._observation_measurement_acks))]
                self._observe_return_condition.notify_all()
        except (ValueError, TypeError, AttributeError):
            return

    def _observation_measurement_scope(self, plan):
        selection = self._mode_bound_selection or {}
        session = self._observe_return_session or {}
        return (selection.get('mode') == 'unknown' and selection.get('strategy') == 'observe_return'
            and session.get('version') == 2 and session.get('phase') in ('observing', 'returning')
            and not self._diagnostic_observation_only()
            and getattr(plan, 'model_choice', '') == 'unknown_tabletop'
            and getattr(plan, 'diagnostic', '') == 'FAR_FIELD_OBSERVATION_PLAN'
            and getattr(plan, 'plan_id', None) == session.get('view_plan_id'))

    def _observation_measurement_live(self, request):
        self._observe_return_live()
        with self._following_evidence_lock:
            epoch = self._following_epoch_ns
        if epoch != request['control_epoch_ns']:
            raise ValueError('OBSERVATION_MEASUREMENT_CONTROL_EPOCH_CHANGED')
        if request.get('motion_kind') == 'actuation_bootstrap':
            self._actuation_bootstrap_live(request)
            return
        session = self._observe_return_session or {}
        if (session.get('session_id') != request['session_id']
                or session.get('view_plan_id') != request['view_plan_id']
                or session.get('target_epoch') != request['target_epoch']
                or session.get('target_track_id') != request['target_track_id']):
            raise ValueError('OBSERVATION_MEASUREMENT_SESSION_CHANGED')

    def _actuation_bootstrap_live(self, request):
        """Check the provisional identity without inventing a captured origin.

        Motion has its original locked action gate. While its RGB-D is being
        reacquired, retained geometry gains no new source timestamp or action
        authority; the fresh target gate runs again after qualification.
        """
        context = getattr(self, '_actuation_bootstrap_context', None)
        if context is None or self._observe_return_session is not None:
            raise ValueError('ACTUATION_BOOTSTRAP_CONTEXT_CHANGED')
        original = context['request']
        keys = ('lease_id', 'selection', 'session_id', 'target_epoch', 'target_track_id',
            'origin_plan_id', 'origin_snapshot_stamp_ns', 'view_plan_id',
            'view_source_stamp_ns', 'view_index', 'plan_sha256', 'control_epoch_ns',
            'deadline_ns', 'motion_kind', 'actuation_bootstrap')
        now = rospy.Time.now().to_nsec()
        if (any(request.get(key) != original[key] for key in keys)
                or self._mode_bound_selection != original['selection']
                or now < context['last_now_ns'] or now >= original['deadline_ns']
                or time.monotonic() >= context['monotonic_deadline']):
            raise ValueError('ACTUATION_BOOTSTRAP_BINDING_CHANGED_OR_EXPIRED')
        context['last_now_ns'] = now
        with self._grasp6d_plan_guard():
            frozen = getattr(self, '_bound_execution_plan', None)
            if (getattr(self, '_execution_authority_revoked', False)
                    or frozen is None or frozen.plan_id != original['view_plan_id']
                    or getattr(self, '_bound_execution_plan_id', None) != original['view_plan_id']
                    or self._bound_execution_plan_digest != context['frozen_digest']
                    or self._execution_plan_digest(frozen) != context['frozen_digest']
                    or committed_plan_digest(frozen) != original['plan_sha256']
                    or self._execution_plan_digest(context['plan']) != context['frozen_digest']):
                raise ValueError('ACTUATION_BOOTSTRAP_AUTHORITY_CHANGED')

    def _bootstrap_bound_actuation(self, plan, gcfg):
        """Join both observers before the existing bounded actuation prefix.

        This provisional receipt is not an observe-return session, a view, an
        origin proof or a contact phase. The ordinary session is captured only
        after confirmed actuation and fresh stationary same-target evidence.
        """
        selected = getattr(self, '_mode_bound_selection', None) or {}
        if (selected.get('mode') != 'unknown' or selected.get('strategy') != 'observe_return'
                or getattr(plan, 'model_choice', '') != 'unknown_tabletop'
                or getattr(plan, 'diagnostic', '') != 'FAR_FIELD_OBSERVATION_PLAN'):
            return super()._bootstrap_bound_actuation(plan, gcfg)
        try:
            if (getattr(self, '_actuation_bootstrap_context', None) is not None
                    or self._observe_return_session is not None
                    or getattr(self, '_initial_preflight_token', None) is not None
                    or self._observation_measurement_request is not None):
                raise ValueError('ACTUATION_BOOTSTRAP_ALREADY_BOUND')
            if not self._actuation_bootstrap_allowed(gcfg):
                raise ValueError('ACTUATION_PREFIX_STATE_CHANGED')
            report = self._observe_return_report(plan)
            metadata = report['observe_return']
            source = positive_ns(plan.header.stamp.to_nsec())
            if (metadata.get('phase') != 'observation_prefix'
                    or metadata.get('diagnostic_observation_only', False) is not False
                    or positive_ns(metadata['origin_snapshot_stamp_ns']) != source
                    or metadata.get('target_track_id') != plan.target_track_id
                    or type(metadata.get('target_epoch')) is not int or metadata['target_epoch'] < 0):
                raise ValueError('ACTUATION_BOOTSTRAP_INITIAL_SOURCE_REQUIRED')
            validity = self._configured_plan_validity(gcfg)
            if type(validity) not in (int, float) or not math.isfinite(validity) or not 0. < validity <= 600.:
                raise ValueError('ACTUATION_BOOTSTRAP_SOURCE_BUDGET_INVALID')
            now = rospy.Time.now().to_nsec()
            deadline = source + round(validity * 1e9)
            with self._following_evidence_lock:
                epoch = positive_ns(self._following_epoch_ns)
            lease = 'bootstrap:' + uuid.uuid4().hex
            request = dict(version=1, state='executing', lease_id=lease,
                request_id=lease+':executing', selection=deepcopy(selected), session_id=lease,
                target_epoch=metadata['target_epoch'], target_track_id=plan.target_track_id,
                origin_plan_id=plan.plan_id, origin_snapshot_stamp_ns=source,
                view_plan_id=plan.plan_id, view_source_stamp_ns=source, view_index=1,
                plan_sha256=committed_plan_digest(plan), control_epoch_ns=epoch,
                deadline_ns=deadline, stamp_ns=now, motion_kind='actuation_bootstrap',
                actuation_bootstrap=dict(version=1, source_deadline_ns=deadline),
                arrival_stamp_ns=0, capture_deadline_ns=0,
                metric_geometry_authority=False, motion_authorized=False)
            request = parse_measurement_request(request, selected, now)
            bootstrap_source_deadline(request, validity)
            self._actuation_bootstrap_context = dict(request=deepcopy(request), plan=deepcopy(plan),
                frozen_digest=self._execution_plan_digest(plan), last_now_ns=now,
                monotonic_deadline=time.monotonic()+(deadline-now)/1e9)
            self.set_state(GraspStages.PLAN_PREGRASP, 'admitting bounded actuation prefix')
            acks = self._observation_measurement_publish(request)
            self._actuation_bootstrap_context['executing_acks'] = deepcopy(acks)
            self._observation_measurement_live(request)
            # Preserve the original action lock, path/collision checks, driver
            # reference synchronization and single-prefix actuation proof.
            if not super()._bootstrap_bound_actuation(plan, gcfg):
                return False
            self._observation_measurement_live(request)
            if self._wait_for_motion_settle('actuation observation prefix') is False:
                raise ValueError('ACTUATION_BOOTSTRAP_POST_MOTION_SETTLE_MISSING')
            self._observation_measurement_live(request)
            self._observation_measurement_finish(plan, gcfg)
            self._observation_measurement_live(self._observation_measurement_request)
            with self._grasp6d_plan_guard():
                validation = self._validate_bound_plan_locked(plan, gcfg)
                if not validation.ok:
                    raise ValueError(validation.code + ': ' + validation.reason)
                # Retire only callbacks that entered before the movement lock
                # drained. Keep their original time/plan/ACK proof until this
                # attempt ends; no measured geometry is copied or refreshed.
                self._actuation_bootstrap_retired_context = self._actuation_bootstrap_context
            return True
        except Exception as exc:
            self.set_state(GraspStages.FAILED, 'ACTUATION_BOOTSTRAP_HANDOFF_FAILED: ' + str(exc))
            return False
        finally:
            self._observation_measurement_revoke('actuation bootstrap ended without qualified measurement')
            self._actuation_bootstrap_context = None

    def _actuation_bootstrap_retired_object(self, msg, received_ns):
        """Discard a timely pre-lease positive delayed by our own action lock.

        The movement ACK retired this input's stationary qualification. It may
        neither replace new arrival geometry nor turn its lock-wait age into
        a false target loss. Every other input keeps the ordinary callback.
        """
        context = (getattr(self, '_actuation_bootstrap_context', None)
                   or getattr(self, '_actuation_bootstrap_retired_context', None))
        if context is None or getattr(msg, 'detected', None) is not True:
            return False
        try:
            original = context['request']
            ns = msg.header.stamp.to_nsec()
            now = rospy.Time.now().to_nsec()
            gcfg = rospy.get_param('/grasp', {})
            validity_ns = round(self._configured_target_observation_validity(gcfg) * 1e9)
            if (not original['view_source_stamp_ns'] <= ns < original['stamp_ns']
                    or not ns <= received_ns < now
                    or received_ns - ns > validity_ns or now - ns <= validity_ns
                    or msg.header.frame_id != context['plan'].header.frame_id
                    or msg.pose_base.header.frame_id != msg.header.frame_id
                    or msg.pose_base.header.stamp.to_nsec() != ns
                    or msg.pose_camera.header.stamp.to_nsec() != ns):
                return False
            confidence = float(msg.confidence)
            xyz = self._object_xyz(msg)
            center = self._plan_geometry_center_xyz(context['plan'])
            maximum = min(self._target_association_threshold(gcfg),
                          self._configured_target_max_drift(gcfg))
            if (not math.isfinite(confidence)
                    or confidence < self._cfg_float(gcfg, 'min_object_confidence', .5)
                    or xyz is None or center is None
                    or not all(math.isfinite(v) for v in xyz)
                    or math.sqrt(sum((a-b)**2 for a, b in zip(xyz, center))) > maximum):
                return False
            self._observe_return_live()
            with self._following_evidence_lock:
                if self._following_epoch_ns != original['control_epoch_ns']:
                    return False
            acks = context.get('executing_acks', ())
            if len(acks) != len(PARTICIPANTS):
                return False
            with self._observe_return_condition:
                current_acks = [deepcopy(self._observation_measurement_acks.get(
                    (original['request_id'], p))) for p in PARTICIPANTS]
            if current_acks != acks:
                return False
            for participant, ack in zip(PARTICIPANTS, acks):
                validate_measurement_ack(ack, original, participant, rospy.Time.now().to_nsec())
            with self._grasp6d_plan_guard():
                frozen = getattr(self, '_bound_execution_plan', None)
                record = getattr(self, '_initial_preflight_attempt', None)
                if (context is not getattr(self, '_actuation_bootstrap_context', None)
                        and context is not getattr(self, '_actuation_bootstrap_retired_context', None)):
                    return False
                if (getattr(self, '_execution_authority_revoked', False)
                        or self._mode_bound_selection != original['selection']
                        or getattr(self, '_mode_selection', None) != original['selection']
                        or frozen is None or frozen.plan_id != original['view_plan_id']
                        or self._bound_execution_plan_id != original['view_plan_id']
                        or self._bound_execution_plan_digest != context['frozen_digest']
                        or self._execution_plan_digest(frozen) != context['frozen_digest']
                        or self._execution_plan_digest(context['plan']) != context['frozen_digest']
                        or committed_plan_digest(frozen) != original['plan_sha256']
                        or record is None or record['ended_ns'] is not None
                        or not record['physical_action_submitted']
                        or any(v != 'measured_pose_changed' for v in record['disqualifications'])
                        or record['selection'] != original['selection']
                        or record['requested_plan_id'] != original['view_plan_id']
                        or record['plan_digest'] != context['frozen_digest']
                        or not any(s['stage'] == 'actuation observation prefix'
                            and original['stamp_ns'] <= s['stamp_ns'] <= now for s in record['submissions'])
                        or record['geometry_invalidations'] or record['plan_invalidations']
                        or record['missing_objects']):
                    return False
                geometry = getattr(self, 'latest_target_geometry', None)
                if geometry is not None:
                    if (not geometry.valid or geometry.target_track_id != original['target_track_id']
                            or geometry.header.frame_id != frozen.header.frame_id
                            or not original['view_source_stamp_ns'] <= geometry.header.stamp.to_nsec() <= now):
                        return False
                    gp = geometry.pose_base.position
                    point = (float(gp.x), float(gp.y), float(gp.z))
                    if (not all(math.isfinite(v) for v in point)
                            or math.sqrt(sum((a-b)**2 for a, b in zip(point, center))) > maximum):
                        return False
                # Recheck both original clocks after all lock waits and audit
                # work. The retired source itself is never made fresh again.
                now = rospy.Time.now().to_nsec()
                cutoff = bootstrap_source_deadline(original, self._configured_plan_validity(gcfg))
                if (now < context['last_now_ns'] or now >= cutoff
                        or time.monotonic() >= context['monotonic_deadline']
                        or self._following_epoch_ns != original['control_epoch_ns']
                        or any(self._observation_measurement_acks.get(
                            (original['request_id'], p)) != ack for p, ack in zip(PARTICIPANTS, acks))):
                    return False
                self._observe_return_live()
                rospy.loginfo('Retired pre-bootstrap object delayed by task lock: %s', json.dumps(dict(
                    source_ns=ns, received_ns=received_ns, retired_ns=now,
                    lease_id=original['lease_id'], metric_geometry_authority=False,
                    motion_authorized=False), sort_keys=True))
                return True
        except (ValueError, TypeError, KeyError, AttributeError, OverflowError):
            return False

    def _observation_measurement_publish(self, request, *, qualified=False):
        original_request = deepcopy(request)
        self._observation_measurement_live(request)
        self._observation_measurement_request = request
        self._observation_measurement_pub.publish(String(data=json.dumps(request, allow_nan=False)))
        rospy.loginfo('Observe-return measurement request: %s', json.dumps(request, allow_nan=False))
        cutoff = original_request['capture_deadline_ns'] if qualified else original_request['deadline_ns']
        remaining = max(0., (cutoff-rospy.Time.now().to_nsec())/1e9)
        deadline = time.monotonic() + (remaining if qualified else min(5., remaining))
        last_validated_acks = None
        while time.monotonic() < deadline:
            self._observation_measurement_live(request)
            with self._observe_return_condition:
                acks = [deepcopy(self._observation_measurement_acks.get((request['request_id'], p)))
                        for p in PARTICIPANTS]
                if any(ack is None for ack in acks):
                    self._observe_return_condition.wait(.02)
                    continue
            # An ACK can arrive while acquiring the condition. Its real
            # source must be compared with a clock sampled after the snapshot,
            # not the earlier loop-entry time. Waiting never renews this lease.
            self._observation_measurement_live(request)
            now = rospy.Time.now().to_nsec()
            waiting = False
            for participant, ack in zip(PARTICIPANTS, acks):
                validate_measurement_ack(ack, request, participant, now)
                if qualified and ack['qualified'] is not True:
                    waiting = True
                elif qualified:
                    validate_measurement_ack(ack, request, participant, now, require_qualified=True)
            last_validated_acks = acks
            # Preserve the validator's specific ROS expiry rejection above;
            # the original wall/monotonic budget still gates waiting/return.
            if now >= cutoff or time.monotonic() >= deadline:
                break
            if waiting:
                with self._observe_return_condition:
                    self._observe_return_condition.wait(.02)
                continue
            if qualified and acks[0]['source_stamps_ns'] != acks[1]['source_stamps_ns']:
                raise ValueError('OBSERVATION_MEASUREMENT_SOURCE_ACK_MISMATCH')
            rospy.loginfo('Observe-return measurement acknowledged: %s', json.dumps(dict(
                request_id=request['request_id'], state=request['state'], acks=acks), allow_nan=False))
            # Owner checks and logging can block too. Commit only the same ACK
            # snapshot while the original wall/monotonic budget is still live.
            self._observation_measurement_live(request)
            with self._observe_return_condition:
                self._observation_measurement_live(request)
                if (self._observation_measurement_request is not request
                        or request != original_request
                        or getattr(self, '_execution_authority_revoked', False)):
                    raise ValueError('OBSERVATION_MEASUREMENT_REQUEST_CHANGED')
                if any(self._observation_measurement_acks.get((request['request_id'], p)) != ack
                       for p, ack in zip(PARTICIPANTS, acks)):
                    continue
                now = rospy.Time.now().to_nsec()
                for participant, ack in zip(PARTICIPANTS, acks):
                    validate_measurement_ack(ack, request, participant, now,
                                             require_qualified=qualified)
                if now >= cutoff or time.monotonic() >= deadline:
                    break
                return acks
        if qualified and last_validated_acks is not None:
            pending = [p for p, ack in zip(PARTICIPANTS, last_validated_acks)
                       if ack['qualified'] is not True]
            raise ValueError('OBSERVATION_MEASUREMENT_QUALIFICATION_TIMEOUT: pending=' + ','.join(pending))
        raise ValueError('OBSERVATION_MEASUREMENT_ACK_TIMEOUT')

    def _observation_measurement_begin(self, plan, motion_kind='observation'):
        old = self._observation_measurement_request
        if old is not None and old['state'] not in ('qualified', 'revoked'):
            raise ValueError('OBSERVATION_MEASUREMENT_ALREADY_PENDING')
        session, origin = self._observe_return_session, self._observe_return_origin_proof
        now = rospy.Time.now().to_nsec()
        lease = '%s:%s:%d' % (session['session_id'], motion_kind, now)
        request = dict(version=1, state='executing', lease_id=lease, request_id=lease+':executing',
            selection=deepcopy(self._mode_bound_selection), session_id=session['session_id'],
            target_epoch=session['target_epoch'], target_track_id=session['target_track_id'],
            origin_plan_id=session['origin_plan_id'], origin_snapshot_stamp_ns=session['origin_snapshot_stamp_ns'],
            view_plan_id=plan.plan_id, view_source_stamp_ns=plan.header.stamp.to_nsec(),
            view_index=session['view_index'], plan_sha256=committed_plan_digest(plan),
            control_epoch_ns=positive_ns(origin['epoch_ns']), deadline_ns=session['deadline_ns'],
            stamp_ns=now, motion_kind=motion_kind, arrival_stamp_ns=0, capture_deadline_ns=0,
            metric_geometry_authority=False, motion_authorized=False)
        request = parse_measurement_request(request, self._mode_bound_selection, now)
        self._observation_measurement_publish(request)
        return request

    def _observation_measurement_revoke(self, reason):
        previous = getattr(self, '_observation_measurement_request', None)
        publisher = getattr(self, '_observation_measurement_pub', None)
        if previous is None or previous['state'] in ('qualified', 'revoked') or publisher is None:
            return
        now = max(rospy.Time.now().to_nsec(), previous['stamp_ns'] + 1)
        request = dict(previous, state='revoked', stamp_ns=now,
            request_id=previous['lease_id']+':revoked:'+str(now), reason=str(reason))
        self._observation_measurement_request = request
        publisher.publish(String(data=json.dumps(request, allow_nan=False)))
        rospy.loginfo('Observe-return measurement revoked: %s', json.dumps(request, allow_nan=False))

    def _observation_measurement_prepare(self, plan):
        """Freeze a supplemental identity before retiring its planning station.

        The existing executing receipt has no motion authority. Its exact
        request and both acknowledgements are consumed once by the original
        strict executor, after its unchanged planning and action checks.
        """
        if getattr(self, '_observation_measurement_prepared', None) is not None:
            raise ValueError('OBSERVATION_MEASUREMENT_PREPARATION_ALREADY_PENDING')
        session = self._observe_return_session or {}
        if (not self._observation_measurement_scope(plan) or session.get('view_index', 1) <= 1
                or getattr(self, 'stage', None) != GraspStages.PLAN_PREGRASP
                or getattr(self, '_last_state_message', '') != 'admitting supplemental observation view'):
            raise ValueError('OBSERVATION_MEASUREMENT_PREPARATION_STAGE_CHANGED')
        request = self._observation_measurement_begin(plan)
        with self._observe_return_condition:
            acks = [deepcopy(self._observation_measurement_acks.get((request['request_id'], p)))
                    for p in PARTICIPANTS]
        prepared = dict(request=deepcopy(request), acks=acks, consumed=False)
        self._observation_measurement_prepared = prepared
        self._observation_measurement_prepared_check(plan, prepared)
        return prepared

    def _observation_measurement_prepared_check(self, plan, prepared, *, consume=False):
        request = prepared['request']
        self._observation_measurement_live(request)
        session = self._observe_return_session or {}
        expected_stage = GraspStages.MOVE_PREGRASP if consume else GraspStages.PLAN_PREGRASP
        if (getattr(self, '_observation_measurement_prepared', None) is not prepared
                or prepared['consumed'] or self._observation_measurement_request != request
                or getattr(self, 'stage', None) != expected_stage
                or (not consume and getattr(self, '_last_state_message', '')
                    != 'admitting supplemental observation view')
                or request['state'] != 'executing' or request['motion_kind'] != 'observation'
                or session.get('version') != 2 or session.get('phase') != 'observing'
                or request['selection'] != self._mode_bound_selection
                or request['selection'] != session.get('selection')
                or request['plan_sha256'] != committed_plan_digest(plan)
                or request['view_plan_id'] != plan.plan_id
                or request['view_source_stamp_ns'] != plan.header.stamp.to_nsec()
                or request['target_track_id'] != plan.target_track_id
                or any(request[k] != session.get(k) for k in (
                    'session_id', 'target_epoch', 'target_track_id', 'origin_plan_id',
                    'view_plan_id', 'view_index'))
                or any(request[k] != positive_ns(session.get(k)) for k in (
                    'origin_snapshot_stamp_ns', 'view_source_stamp_ns', 'deadline_ns'))):
            raise ValueError('OBSERVATION_MEASUREMENT_PREPARED_BINDING_CHANGED')
        now = rospy.Time.now().to_nsec()
        parse_measurement_request(request, self._mode_bound_selection, now)
        with self._observe_return_condition:
            for participant, accepted in zip(PARTICIPANTS, prepared['acks']):
                current = self._observation_measurement_acks.get((request['request_id'], participant))
                if current != accepted:
                    raise ValueError('OBSERVATION_MEASUREMENT_PREPARED_ACK_CHANGED')
                validate_measurement_ack(current, request, participant, now)
        if consume:
            prepared['consumed'] = True

    def _observation_measurement_finish(self, plan, gcfg):
        previous = self._observation_measurement_request
        if previous is None or previous['state'] != 'executing':
            raise ValueError('OBSERVATION_MEASUREMENT_EXECUTION_RECEIPT_MISSING')
        self._observation_measurement_live(previous)
        settled = deepcopy(getattr(self, '_last_motion_settle_sample', None))
        now = rospy.Time.now().to_nsec()
        timeout = gcfg.get('unknown_observe_return_requalification_timeout_sec', 3.)
        if (isinstance(timeout, bool) or not isinstance(timeout, (int, float))
                or not math.isfinite(timeout) or not 0. < timeout <= 10.):
            raise ValueError('OBSERVATION_MEASUREMENT_CAPTURE_BUDGET_INVALID')
        deadline = min(previous['deadline_ns'], now + int(timeout*1e9))
        recovery = previous.get('observation_station_requalification')
        if recovery is not None:
            deadline = min(deadline, recovery['deadline_ns'])
        request = dict(previous, state='requalifying', request_id=previous['lease_id']+':requalifying',
            stamp_ns=now, arrival_stamp_ns=now, capture_deadline_ns=deadline, settle=settled)
        request = parse_measurement_request(request, self._mode_bound_selection, now, previous)
        acks = self._observation_measurement_publish(request, qualified=True)
        sources = acks[0]['source_stamps_ns']
        # Topic callbacks can arrive after the participant ACK. Wait for the
        # actual source-bound object; an ACK never stands in for geometry.
        wall_deadline = time.monotonic() + max(0., (deadline-rospy.Time.now().to_nsec())/1e9)
        while rospy.Time.now().to_nsec() < deadline and time.monotonic() < wall_deadline:
            self._observation_measurement_live(request)
            with self._grasp6d_plan_guard():
                obj = deepcopy(getattr(self, 'latest_obj', None))
            if (obj is not None and getattr(obj, 'detected', False)
                    and self._object_source_stamp(obj).to_nsec() == sources[-1]
                    and self._observed_target_matches_plan(plan, obj, gcfg)):
                break
            with self._observe_return_condition:
                self._observe_return_condition.wait(.02)
        else:
            raise ValueError('OBSERVATION_MEASUREMENT_TASK_SOURCE_MISSING')
        now = rospy.Time.now().to_nsec()
        done = dict(request, state='qualified', request_id=previous['lease_id']+':qualified',
            stamp_ns=now, source_stamps_ns=sources)
        done = parse_measurement_request(done, self._mode_bound_selection, now, request)
        completed_acks = self._observation_measurement_publish(done)
        if done.get('motion_kind') == 'actuation_bootstrap':
            self._observation_measurement_live(done)
            self._actuation_bootstrap_receipt = dict(request=deepcopy(done),
                acks=deepcopy(completed_acks), observed=deepcopy(obj),
                counts_as_observation_view=False, contact_authorized=False)
            return
        self._qualified_station_receipt = dict(request=deepcopy(done), acks=deepcopy(completed_acks),
            observed=deepcopy(obj))
        self._ensure_qualified_station_phase(plan, gcfg)

    def _ensure_qualified_station_phase(self, plan, gcfg):
        """Begin planning at the real qualified arrival, before the next RGB-D.

        The later envelope/range checks and all action gates remain mandatory.
        Reusing this exact lease's phase never renews its original deadline.
        """
        direct = getattr(self, '_direct_near_field_enabled', None)
        if not callable(direct) or not direct(gcfg):
            return None
        selection = getattr(self, '_mode_bound_selection', None) or {}
        session = getattr(self, '_observe_return_session', None) or {}
        if (selection.get('mode') != 'unknown' or selection.get('strategy') != 'observe_return'
                or session.get('version') != 2 or session.get('phase') not in ('observing', 'returning', 'returned')
                or getattr(plan, 'model_choice', '') != 'unknown_tabletop'
                or getattr(plan, 'diagnostic', '') != 'FAR_FIELD_OBSERVATION_PLAN'):
            return None
        try:
            with self._grasp6d_plan_guard(), self._observe_return_condition:
                receipt = getattr(self, '_qualified_station_receipt', None)
                request = getattr(self, '_observation_measurement_request', None)
                now = rospy.Time.now().to_nsec()
                if (receipt is None or request is None or request.get('state') != 'qualified'
                        or receipt['request'] != request or request['view_plan_id'] != plan.plan_id
                        or request['plan_sha256'] != committed_plan_digest(plan)):
                    raise ValueError('QUALIFIED_STATION_PHASE_RECEIPT_REQUIRED')
                self._observation_measurement_live(request)
                selection = getattr(self, '_mode_bound_selection', None) or {}
                session = getattr(self, '_observe_return_session', None) or {}
                if (selection != request['selection'] or session.get('selection') != selection
                        or session.get('version') != 2
                        or session.get('phase') not in ('observing', 'returning', 'returned')
                        or any(session.get(key) != request[key] for key in
                               ('session_id', 'target_epoch', 'target_track_id', 'origin_plan_id',
                                'view_plan_id', 'view_index'))
                        or any(positive_ns(session.get(key)) != positive_ns(request[key]) for key in
                               ('deadline_ns', 'origin_snapshot_stamp_ns', 'view_source_stamp_ns'))):
                    raise ValueError('QUALIFIED_STATION_PHASE_SESSION_BINDING_CHANGED')
                if len(receipt['acks']) != len(PARTICIPANTS):
                    raise ValueError('QUALIFIED_STATION_PHASE_DUAL_ACK_REQUIRED')
                for participant, ack in zip(PARTICIPANTS, receipt['acks']):
                    validate_measurement_ack(ack, request, participant, now)
                    with self._observe_return_condition:
                        if self._observation_measurement_acks.get((request['request_id'], participant)) != ack:
                            raise ValueError('QUALIFIED_STATION_PHASE_ACK_CHANGED')
                existing = getattr(self, '_qualified_station_phase', None)
                if existing is not None and existing['request']['lease_id'] == request['lease_id']:
                    phase = getattr(self, '_published_near_field_phase', None)
                    if (existing['request'] != request or existing['plan_sha256'] != committed_plan_digest(plan)
                            or phase != existing['phase'] or not phase.active
                            or not self._near_field_active or self._near_field_phase_id != phase.phase_id
                            or getattr(self, '_near_field_phase_started_ns', None) != phase.header.stamp.to_nsec()
                            or getattr(self, '_near_field_phase_deadline_ns', None) != phase.deadline.to_nsec()
                            or self._near_field_phase_started_sec != phase.header.stamp.to_sec()
                            or self._near_field_phase_deadline_sec != phase.deadline.to_sec()
                            or not phase.header.stamp.to_nsec() <= now < phase.deadline.to_nsec()
                            or time.monotonic() >= existing['monotonic_deadline']):
                        raise ValueError('QUALIFIED_STATION_PHASE_OWNER_CHANGED_OR_EXPIRED')
                    return deepcopy(phase)
                budget = gcfg.get('near_field_replan_timeout_sec', 60.)
                if (type(budget) not in (int, float) or not math.isfinite(budget) or not 0. < budget <= 300.):
                    raise ValueError('QUALIFIED_STATION_PHASE_BUDGET_INVALID')
                deadline = min(positive_ns(request['deadline_ns']), positive_ns(session['deadline_ns']),
                    now + int(budget*1e9))
                with self._grasp6d_plan_guard(), self._observe_return_condition:
                    self._observation_measurement_live(request)
                    if (self._observation_measurement_request != request
                            or self._qualified_station_receipt != receipt
                            or any(self._observation_measurement_acks.get((request['request_id'], participant)) != ack
                                   for participant, ack in zip(PARTICIPANTS, receipt['acks']))):
                        raise ValueError('QUALIFIED_STATION_PHASE_RECEIPT_CHANGED_BEFORE_PUBLICATION')
                    now = rospy.Time.now().to_nsec()
                    if now >= deadline:
                        raise ValueError('QUALIFIED_STATION_PHASE_EXPIRED_BEFORE_PUBLICATION')
                    observed = deepcopy(receipt['observed'])
                    source_ns = 0 if observed is None else self._object_source_stamp(observed).to_nsec()
                    if (not getattr(observed, 'detected', False)
                            or source_ns != request['source_stamps_ns'][-1]
                            or not 0 <= now-source_ns < 800_000_000
                            or not self._observed_target_matches_plan(plan, observed, gcfg)):
                        raise ValueError('QUALIFIED_STATION_PHASE_FRESH_COMPLETED_SOURCE_REQUIRED')
                    self.set_state(GraspStages.PLAN_PREGRASP,
                        'qualified observation station; checking range before contact planning')
                    self._set_near_field_active(True, force=True, budget_sec=budget,
                        reference_plan=plan, reference_object=observed, absolute_deadline_ns=deadline)
                    phase = deepcopy(self._published_near_field_phase)
                    if (not phase.active or phase.planning_stage != 'contact_replan'
                            or not phase.reference_center_valid
                            or phase.reference_target_track_id != request['target_track_id']
                            or phase.reference_source_stamp.to_nsec() != source_ns
                            or not source_ns <= phase.header.stamp.to_nsec() < phase.deadline.to_nsec() <= deadline):
                        raise ValueError('QUALIFIED_STATION_PHASE_REFERENCE_INVALID')
                    self._observation_measurement_live(request)
                    self._qualified_station_phase = dict(request=deepcopy(request), phase=phase,
                        plan_sha256=request['plan_sha256'],
                        monotonic_deadline=time.monotonic()+(phase.deadline.to_nsec()-rospy.Time.now().to_nsec())*1e-9)
                    return deepcopy(phase)
        except Exception:
            self._qualified_station_phase = None
            self._set_near_field_active(False)
            raise

    def _compensate_observation_before_measurement(self, plan, gcfg):
        """Keep any checked correction inside the existing executing lease.

        The gateway owns command compensation. Only fresh observations after
        its final settle may qualify the station; its receipt is not RGB or
        target-identity evidence and never renews the task deadline.
        """
        enabled = gcfg.get('observation_endpoint_compensation_enabled', False)
        if type(enabled) is not bool:
            raise ValueError('OBSERVATION_COMPENSATION_CONFIG_INVALID')
        if not enabled:
            return
        session = self._observe_return_session or {}
        if session.get('phase') != 'observing':
            return
        previous = deepcopy(self._observation_measurement_request)
        if (not self._observation_measurement_scope(plan)
                or not isinstance(previous, dict)
                or previous.get('state') != 'executing'
                or previous.get('motion_kind') != 'observation'):
            raise ValueError('OBSERVATION_COMPENSATION_EXECUTION_CONTEXT_MISSING')
        self._observation_measurement_live(previous)
        started = rospy.Time.now().to_nsec()
        self.set_state(GraspStages.MOVE_PREGRASP, 'correcting observation endpoint')
        name = '/supervisor/compensate_observation_endpoint'
        rospy.wait_for_service(name, timeout=2.)
        service = rospy.ServiceProxy(name, CompensatePregrasp)
        validation, response = self._invoke_observation_compensation_action(
            plan, gcfg,
            lambda: service(deepcopy(plan), True), expected_request=previous)
        if not validation.ok:
            raise ValueError(validation.code + ': ' + validation.reason)
        if not response.success:
            raise ValueError('OBSERVATION_COMPENSATION_FAILED: ' + response.message)
        report = json.loads(response.message)
        now = rospy.Time.now().to_nsec()
        self._observation_measurement_live(previous)
        if (self._observation_measurement_request != previous
                or report.get('code') != 'OBSERVATION_ENDPOINT_READY'
                or report.get('plan_id') != plan.plan_id
                or report.get('plan_digest') != committed_plan_digest(plan)
                or report.get('fixed_goal_pose') != list(pose_values(plan.poses[0]))
                or any(report.get(key) != previous[key] for key in
                    ('control_epoch_ns', 'session_id', 'lease_id', 'view_index'))
                or type(report.get('physical_motion_submitted')) is not bool
                or report.get('real_grasp_success') is not False
                or not isinstance(report.get('view_audit'), dict)
                or not started <= positive_ns(report['completion_stamp_ns']) <= now
                or not previous['control_epoch_ns'] <= positive_ns(report['accepted_stamp_ns']) <= now
                or now - positive_ns(report['accepted_stamp_ns']) > 500_000_000
                or now >= previous['deadline_ns']):
            raise ValueError('OBSERVATION_COMPENSATION_RECEIPT_INVALID')
        if report['physical_motion_submitted']:
            if (positive_ns(report['accepted_stamp_ns']) <= positive_ns(report['completion_stamp_ns'])
                    or self._wait_for_motion_settle('observation endpoint compensation') is not True):
                raise ValueError('OBSERVATION_COMPENSATION_POST_MOTION_SETTLE_MISSING')
            settled = self._last_motion_settle_sample
            if (not isinstance(settled, dict)
                    or settled.get('epoch_ns') != previous['control_epoch_ns']
                    or positive_ns(settled['stationary_window_start_ns']) <= positive_ns(report['completion_stamp_ns'])):
                raise ValueError('OBSERVATION_COMPENSATION_POST_MOTION_SETTLE_MISSING')
        self._observation_measurement_live(previous)
        self._last_observation_endpoint_compensation_report = deepcopy(report)
        rospy.loginfo('Observation endpoint compensation completed: %s', response.message)

    def _observation_compensation_action_context(self, plan):
        """Bind the already executing observation, never a new observation lease."""
        now = rospy.Time.now().to_nsec()
        request = deepcopy(self._observation_measurement_request)
        parsed = parse_measurement_request(request, self._mode_bound_selection, now)
        self._observation_measurement_live(parsed)
        session = deepcopy(self._observe_return_session)
        origin = deepcopy(self._observe_return_origin_proof)
        origin_plan = self._observe_return_origin_plan
        active_plan = self._observe_return_active_view_plan
        settled = deepcopy(getattr(self, '_last_motion_settle_sample', None))
        if (not self._observation_measurement_scope(plan)
                or session.get('phase') != 'observing'
                or parse_selection(session.get('selection')) != parsed['selection']
                or parsed['state'] != 'executing' or parsed['motion_kind'] != 'observation'
                or self.stage != GraspStages.MOVE_PREGRASP
                or self._last_state_message != 'correcting observation endpoint'
                or parsed['view_plan_id'] != plan.plan_id
                or parsed['view_source_stamp_ns'] != plan.header.stamp.to_nsec()
                or parsed['plan_sha256'] != committed_plan_digest(plan)
                or committed_plan_digest(active_plan) != parsed['plan_sha256']
                or origin_plan.plan_id != parsed['origin_plan_id']
                or origin_plan.header.stamp.to_nsec() != parsed['origin_snapshot_stamp_ns']
                or not self._observe_return_origin_valid(origin, origin_plan, parsed['stamp_ns'])
                or origin['plan_digest'] != committed_plan_digest(origin_plan)
                or origin['plan_id'] != origin_plan.plan_id
                or origin['session_id'] != parsed['session_id']
                or origin['target_track_id'] != parsed['target_track_id']
                or positive_ns(origin['epoch_ns']) != parsed['control_epoch_ns']
                or any(parsed[k] != session.get(k) for k in
                    ('session_id', 'target_epoch', 'target_track_id', 'origin_plan_id', 'view_plan_id'))
                or any(parsed[k] != positive_ns(session.get(k)) for k in
                    ('origin_snapshot_stamp_ns', 'view_source_stamp_ns', 'view_index', 'deadline_ns'))
                or not isinstance(settled, dict)
                or settled.get('epoch_ns') != parsed['control_epoch_ns']
                or not parsed['stamp_ns'] < positive_ns(settled['stationary_window_start_ns'])
                    < positive_ns(settled['stationary_window_end_ns']) <= now):
            raise ValueError('OBSERVATION_COMPENSATION_ACTION_BINDING_INVALID')
        return dict(request=request, session=session, origin=origin,
                    origin_digest=committed_plan_digest(origin_plan),
                    plan_digest=parsed['plan_sha256'], settled=settled)

    def _invoke_observation_compensation_action(self, plan, gcfg, action, *, expected_request):
        # The normal base action gate still owns frozen plan integrity, manual
        # control, cancellation, identity, geometry and drift. Only its age
        # check knows this one in-flight observation has yet to requalify RGB-D.
        with self._grasp6d_plan_guard():
            if self._observation_measurement_request != expected_request:
                raise ValueError('OBSERVATION_COMPENSATION_ACTION_LEASE_CHANGED')
            if getattr(self, '_observation_compensation_action_scope', None) is not None:
                raise ValueError('OBSERVATION_COMPENSATION_ACTION_ALREADY_ACTIVE')
            context = self._observation_compensation_action_context(plan)
            self._observation_compensation_action_scope = context
            try:
                result, response = self._invoke_plan_bound_action(plan, gcfg,
                    'measured observation endpoint compensation', action)
                if result.ok:
                    if self._observation_compensation_action_context(plan) != context:
                        raise ValueError('OBSERVATION_COMPENSATION_ACTION_CONTEXT_CHANGED')
                    result = self._validate_bound_plan_locked(plan, gcfg)
                return result, response
            finally:
                self._observation_compensation_action_scope = None

    def _bound_observation_age_allowed(self, plan, gcfg, observation_age):
        context = getattr(self, '_observation_compensation_action_scope', None)
        if context is None:
            return super()._bound_observation_age_allowed(plan, gcfg, observation_age)
        try:
            if (not math.isfinite(observation_age) or observation_age < 0.
                    or gcfg.get('observation_endpoint_compensation_enabled') is not True
                    or self._observation_compensation_action_context(plan) != context):
                return False
            request = context['request']
            # Retained historical object must have been fresh when this exact
            # view began. No new/future/missing source gains an age exemption.
            source = positive_ns(self.latest_obj_time.to_nsec())
            observed = self.latest_obj
            if (source != positive_ns(self._object_source_stamp(observed).to_nsec())
                    or source != positive_ns(observed.pose_base.header.stamp.to_nsec())
                    or not request['view_source_stamp_ns'] <= source <= request['stamp_ns']
                    or not 0. <= (request['stamp_ns']-source)*1e-9
                        <= self._configured_target_observation_validity(gcfg)):
                return False
            return True
        except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
            return False

    def _wrap_pose_submission_executor(self, stage, plan, gcfg, executor):
        # The base chooses its stage-specific service first. Wrap that final
        # executor, so a later service selection cannot erase this handshake.
        if not self._contact_station_scope(plan, stage):
            return executor
        if not callable(executor):
            raise ValueError('CONTACT_STATION_STRICT_EXECUTOR_MISSING')
        def submit(pose, execute):
            if execute is not True:
                raise ValueError('CONTACT_STATION_SUBMISSION_REQUIRES_EXECUTE')
            self._contact_station_before_submit(plan, gcfg)
            return executor(pose, execute)
        return submit

    def _plan_and_execute_pose(self, stage, label, pose, move_pose, settle_reason,
                              execution_plan_id=None, execution_plan=None, gcfg=None,
                              execute_pose=None, allow_post_failure_observation_validation=False):
        """Wrap only the already validated strict submission of an ordinary view."""
        scoped = self._observation_measurement_scope(execution_plan)
        contact_station = self._contact_station_scope(execution_plan, stage)
        prepared = getattr(self, '_observation_measurement_prepared', None) if scoped else None
        submitted = [False]
        original_execute = execute_pose
        if scoped:
            if not callable(original_execute):
                try:
                    self._observation_measurement_revoke('strict executor missing')
                finally:
                    if getattr(self, '_observation_measurement_prepared', None) is prepared:
                        self._observation_measurement_prepared = None
                self.set_state(GraspStages.FAILED, 'OBSERVATION_MEASUREMENT_STRICT_EXECUTOR_MISSING')
                return False
            def execute_pose(*args, **kwargs):
                if prepared is None:
                    session = getattr(self, '_observe_return_session', None) or {}
                    if session.get('phase') == 'observing' and session.get('view_index', 1) > 1:
                        raise ValueError('OBSERVATION_MEASUREMENT_SUPPLEMENT_NOT_PREPARED')
                    self._observation_measurement_begin(execution_plan)
                else:
                    self._observation_measurement_prepared_check(execution_plan, prepared, consume=True)
                # The dual acknowledgement is asynchronous. Re-run the SAME
                # original gate before the physical call, without age waivers.
                validation = self._validate_bound_plan_locked(execution_plan, gcfg or {})
                if not validation.ok:
                    raise ValueError(validation.code + ': ' + validation.reason)
                submitted[0] = True
                response = original_execute(*args, **kwargs)
                if not bool(getattr(response, 'success', False)):
                    self._observation_measurement_revoke('strict execution failed')
                return response
        try:
            result = super()._plan_and_execute_pose(stage, label, pose, move_pose, settle_reason,
                execution_plan_id=execution_plan_id, execution_plan=execution_plan, gcfg=gcfg,
                execute_pose=execute_pose,
                allow_post_failure_observation_validation=allow_post_failure_observation_validation)
            if scoped:
                if not result:
                    self._observation_measurement_revoke('motion or original endpoint gate failed')
                elif submitted[0]:
                    self._compensate_observation_before_measurement(execution_plan, gcfg or {})
                    self._observation_measurement_finish(execution_plan, gcfg or {})
                elif prepared is not None:
                    raise ValueError('OBSERVATION_MEASUREMENT_PREPARED_NOT_SUBMITTED')
            return result
        except Exception as exc:
            if contact_station:
                self.set_state(GraspStages.FAILED, 'CONTACT_STATION_HANDOFF_FAILED: ' + str(exc))
                return False
            if not scoped:
                raise
            self._observation_measurement_revoke(str(exc))
            self.set_state(GraspStages.FAILED, 'OBSERVATION_MEASUREMENT_FAILED: ' + str(exc))
            return False
        finally:
            if prepared is not None and getattr(self, '_observation_measurement_prepared', None) is prepared:
                self._observation_measurement_prepared = None

    def _observe_return_audit_cb(self, msg):
        try:
            reference = json.loads(msg.data)
            digest = reference['report_sha256']
            if not isinstance(digest, str) or not re.fullmatch('[0-9a-f]{64}', digest):
                return
            with open(reference['report_path'], 'rb') as source:
                payload = source.read()
            if hashlib.sha256(payload).hexdigest() != digest:
                return
            report = json.loads(payload.decode('utf-8'))
            if not isinstance(report, dict) or not report.get('plan_id'):
                return
            with self._observe_return_condition:
                self._observe_return_reports[report['plan_id']] = report
                self._observe_return_report_references[report['plan_id']] = dict(
                    path=os.path.realpath(reference['report_path']), sha256=digest)
                while len(self._observe_return_reports) > 16:
                    oldest = next(iter(self._observe_return_reports))
                    del self._observe_return_reports[oldest]
                    self._observe_return_report_references.pop(oldest, None)
                self._observe_return_condition.notify_all()
        except (KeyError, OSError, TypeError, ValueError, AttributeError):
            return

    def _observe_return_ack_cb(self, msg):
        try:
            ack = json.loads(msg.data)
            key = (str(ack['session_id']), str(ack['phase']), positive_ns(ack['stamp_ns']))
            if type(ack['accepted']) is not bool:
                return
            with self._observe_return_condition:
                self._observe_return_acks[key] = ack
                while len(self._observe_return_acks) > 16:
                    del self._observe_return_acks[next(iter(self._observe_return_acks))]
                self._observe_return_condition.notify_all()
        except (KeyError, TypeError, ValueError, AttributeError):
            return

    def _observe_return_live(self):
        if (not self.active or rospy.is_shutdown()
                or not self._selection_current(self._mode_bound_selection)
                or rospy.get_param('/gui/joint_direct_mode', False) is True):
            raise ValueError('OBSERVE_RETURN_OWNER_CHANGED')
        session = self._observe_return_session
        if session is not None and rospy.Time.now().to_nsec() >= positive_ns(session['deadline_ns']):
            raise ValueError('OBSERVE_RETURN_SESSION_EXPIRED')

    def _observe_return_report(self, plan, allow_returned=False, *, observed_contact=False):
        deadline = time.monotonic() + 3.
        while time.monotonic() < deadline:
            self._observe_return_live()
            with self._observe_return_condition:
                report = deepcopy(self._observe_return_reports.get(plan.plan_id))
                if report is None:
                    self._observe_return_condition.wait(.05)
                    continue
            if parse_selection(report.get('grasp_mode_selection')) != self._mode_bound_selection:
                raise ValueError('OBSERVE_RETURN_AUDIT_SELECTION_CHANGED')
            if observed_contact:
                measurement = getattr(self, '_observation_measurement_request', None)
                self._observation_measurement_live(measurement)
                phase = getattr(self, '_published_near_field_phase', None)
                if (not getattr(self, '_near_field_active', False)
                        or phase is None or phase.phase_id != getattr(self, '_near_field_phase_id', None)
                        or getattr(self, '_near_field_phase_started_ns', None) != phase.header.stamp.to_nsec()
                        or getattr(self, '_near_field_phase_deadline_ns', None) != phase.deadline.to_nsec()
                        or self._near_field_phase_started_sec != phase.header.stamp.to_sec()
                        or self._near_field_phase_deadline_sec != phase.deadline.to_sec()):
                    raise ValueError('OBSERVE_RETURN_CONTACT_PHASE_CHANGED')
                reason = observed_contact_audit_error(report,
                    session=self._observe_return_session,
                    reached_phase=phase,
                    measurement=measurement, now_ns=rospy.Time.now().to_nsec())
            else:
                reason = observe_return_audit_error(report, allow_returned=allow_returned)
            if reason:
                raise ValueError('OBSERVE_RETURN_AUDIT_INVALID: ' + reason)
            if (report['observe_return']['target_track_id'] != plan.target_track_id
                    or not audit_matches_preview(report, plan, plan.diagnostic == 'CONTACT_EXECUTION_PLAN')):
                raise ValueError('OBSERVE_RETURN_AUDIT_PLAN_MISMATCH')
            return report
        raise ValueError('OBSERVE_RETURN_BOUND_AUDIT_UNAVAILABLE')

    def _observe_return_publish(self, message):
        self._observe_return_live()
        # This ACK completes an inference/session handoff, not a robot action.
        # A slow drain may wait within its existing contract; it must not
        # manufacture a new lease or bypass the later source/action checks.
        request = deepcopy(message)
        selection = deepcopy(self._mode_bound_selection)
        owner = self._observe_return_session
        previous = deepcopy(owner)
        started = time.monotonic()
        last_now = rospy.Time.now().to_nsec()
        normalized = parse_session(request, selection, last_now,
            previous if previous != request else None)
        cutoff = normalized['deadline_ns']
        # One wall-clock cap per session, anchored on its first publication.
        # Later view/return handoffs do not renew it if ROS time stalls.
        session_clock = getattr(self, '_observe_return_session_clock', None)
        clock_binding = (normalized['session_id'], normalized['deadline_ns'])
        if session_clock is None:
            session_clock = (*clock_binding, started + (cutoff-last_now)/1e9)
            self._observe_return_session_clock = session_clock
        elif session_clock[:2] != clock_binding:
            raise ValueError('OBSERVE_RETURN_ACK_SESSION_CLOCK_CHANGED')
        if normalized['phase'] == 'returned':
            cutoff = min(cutoff, normalized['contact_deadline_ns'])
        epoch = getattr(self, '_following_epoch_ns', None)
        # Initial observing has no reached contact station. Subsequent view
        # and return transitions retain a current station when one exists;
        # a retired historical handoff is not a new phase requirement.
        station_transition = (normalized['phase'] != 'observing'
            or normalized.get('view_index', 1) > 1)
        phase = (deepcopy(getattr(self, '_published_near_field_phase', None))
            if station_transition and getattr(self, '_near_field_active', False) else None)
        if station_transition and getattr(self, '_near_field_active', False) and phase is None:
            raise ValueError('OBSERVE_RETURN_ACK_CONTACT_PHASE_CHANGED')
        qualified = (getattr(self, '_qualified_station_phase', None)
            if phase is not None else None)
        handoff = getattr(self, '_station_task_handoff', None)
        if not (station_transition and normalized['phase'] == 'observing'
                and isinstance(handoff, dict) and handoff.get('cutoff_ns') is None
                and handoff.get('next_session') == request):
            handoff = None
        deadline = min(session_clock[2], started + max(0., (cutoff-last_now)/1e9))
        if phase is not None:
            cutoff = min(cutoff, positive_ns(phase.deadline.to_nsec()))
            deadline = min(deadline, started + max(0., (cutoff-last_now)/1e9))
            if isinstance(qualified, dict) and qualified.get('phase') == phase:
                deadline = min(deadline, qualified['monotonic_deadline'])
        if handoff is not None:
            cutoff = min(cutoff, handoff['phase_deadline_ns'])
            deadline = min(deadline, handoff['phase_mono'],
                started + max(0., (cutoff-last_now)/1e9))

        def wait_live():
            nonlocal last_now
            self._observe_return_live()
            now = rospy.Time.now().to_nsec()
            wall = time.monotonic()
            if now < last_now or wall < started:
                raise ValueError('OBSERVE_RETURN_ACK_CLOCK_REWOUND')
            last_now = now
            if (self._mode_bound_selection != selection
                    or self._observe_return_session is not owner
                    or self._observe_return_session != previous or message != request
                    or getattr(self, '_following_epoch_ns', None) != epoch
                    or getattr(self, '_execution_authority_revoked', False)):
                raise ValueError('OBSERVE_RETURN_ACK_OWNER_CHANGED_OR_REVOKED')
            if phase is not None and (not getattr(self, '_near_field_active', False)
                    or self._published_near_field_phase != phase or not phase.active):
                raise ValueError('OBSERVE_RETURN_ACK_CONTACT_PHASE_CHANGED')
            if now >= cutoff or wall >= deadline:
                raise ValueError('OBSERVE_RETURN_REMOTE_ACK_TIMEOUT: original contract expired')
            if handoff is not None:
                if self._station_task_handoff is not handoff:
                    raise ValueError('OBSERVE_RETURN_ACK_STATION_HANDOFF_CHANGED')
                self._station_handoff_live(handoff)
            return wall

        wait_live()
        key = (message['session_id'], message['phase'], positive_ns(message['stamp_ns']))
        self._observe_return_pub.publish(String(data=json.dumps(request, allow_nan=False)))
        warned = False
        while True:
            wall = wait_live()
            with self._observe_return_condition:
                ack = deepcopy(self._observe_return_acks.get(key))
                if ack is None:
                    if not warned and wall-started >= 5.:
                        rospy.logwarn('OBSERVE_RETURN_REMOTE_ACK_WAITING: session=%s phase=%s '
                            'stamp_ns=%s; waiting for original handoff, no action submitted; deadline_ns=%d',
                            key[0], key[1], key[2], cutoff)
                        warned = True
                    self._observe_return_condition.wait(min(.05, max(0., deadline-wall)))
                    continue
            if ack['accepted'] is not True:
                raise ValueError('OBSERVE_RETURN_REMOTE_REJECTED: ' + str(ack.get('reason', '')))
            try:
                same = (type(ack.get('version')) is int and ack['version'] == message['version']
                    and parse_selection(ack['selection']) == parse_selection(message['selection'])
                    and type(ack.get('target_epoch')) is int
                    and ack['target_epoch'] == message['target_epoch']
                    and all(ack[name] == message[name] for name in (
                        'session_id', 'phase', 'target_track_id', 'origin_plan_id'))
                    and all(exact_ns(ack[name], name, 0) == exact_ns(message.get(name, 0), name, 0)
                        for name in ('stamp_ns', 'deadline_ns', 'origin_snapshot_stamp_ns',
                                     'return_verified_stamp_ns', 'contact_deadline_ns')))
            except (KeyError, TypeError, ValueError):
                same = False
            if same and message['version'] == 2:
                try:
                    normalized = parse_session(message, self._mode_bound_selection,
                        rospy.Time.now().to_nsec(), self._observe_return_session
                        if self._observe_return_session != message else None)
                    same = all(ack.get(key) == normalized[key] for key in (*VIEW_FIELDS, 'max_observations'))
                except (KeyError, TypeError, ValueError):
                    same = False
            if not same:
                raise ValueError('OBSERVE_RETURN_REMOTE_ACK_BINDING_INVALID')
            wait_live()
            return ack

    def _observe_return_gateway(self, name, plan, *, bound_plan=None, gcfg=None):
        self._observe_return_live()
        rospy.wait_for_service(name, timeout=3.)
        call = rospy.ServiceProxy(name, CompensatePregrasp)
        if bound_plan is None:
            response = call(plan, True)
        else:
            # Waits must remain outside the callback lock. Recheck the active
            # view at the actual return submission, while the gateway retains
            # the immutable origin plan as its return destination.
            def submit_return():
                if (name == '/supervisor/return_observe_return_origin'
                        and self._observation_measurement_scope(bound_plan)):
                    self._observation_measurement_begin(bound_plan, 'return')
                    validation = self._validate_bound_plan_locked(bound_plan, gcfg or {})
                    if not validation.ok:
                        raise ValueError(validation.code + ': ' + validation.reason)
                return call(plan, True)
            validation, response = self._invoke_plan_bound_action(bound_plan, gcfg,
                'observe-return origin submission', submit_return)
            if not validation.ok:
                raise ValueError(validation.code + ': ' + validation.reason)
        if not response.success:
            raise ValueError('OBSERVE_RETURN_GATEWAY_FAILED: ' + response.message)
        proof = json.loads(response.message)
        if (not isinstance(proof, dict) or proof.get('plan_id') != plan.plan_id
                or proof.get('target_track_id') != plan.target_track_id):
            raise ValueError('OBSERVE_RETURN_GATEWAY_BINDING_INVALID')
        self._observe_return_live()
        return proof

    @staticmethod
    def _observe_return_origin_valid(proof, plan, now_ns):
        """Require the measured origin fields supplied by the strict gateway."""
        try:
            def finite_vector(values, size):
                return (isinstance(values, (list, tuple)) and len(values) == size
                    and all(type(value) in (int, float) and math.isfinite(value) for value in values))
            pose = proof['origin_tool_pose']
            return (proof.get('code') in ('OBSERVE_RETURN_ORIGIN_CAPTURED', 'OBSERVE_RETURN_ORIGIN_REUSED')
                and proof.get('physical_motion_submitted') is False
                and isinstance(proof.get('session_id'), str) and bool(proof['session_id'])
                and proof['session_id'] == proof['session_id'].strip()
                and plan.header.stamp.to_nsec() <= positive_ns(proof['origin_accepted_stamp_ns']) <= now_ns
                and positive_ns(proof['epoch_ns']) <= positive_ns(proof['origin_accepted_stamp_ns'])
                and all(isinstance(proof.get(key), str) and re.fullmatch('[0-9a-f]{64}', proof[key])
                    for key in ('plan_digest', 'model_sha256', 'origin_goal_sha256'))
                and finite_vector(proof['origin_joint_positions_rad'], 6)
                and pose['frame_id'] == 'base_link'
                and finite_vector(pose['position_m'], 3)
                and finite_vector(pose['quaternion_xyzw'], 4)
                and math.isclose(sum(value*value for value in pose['quaternion_xyzw']), 1., abs_tol=1e-6))
        except (AttributeError, KeyError, TypeError, ValueError):
            return False

    def _begin_observe_return(self, plan, gcfg):
        report = self._observe_return_report(plan)
        metadata = report['observe_return']
        if metadata.get('phase') != 'observation_prefix':
            raise ValueError('OBSERVE_RETURN_OBSERVATION_PREFIX_REQUIRED')
        with self._observe_return_condition:
            if self._observe_return_reports.get(plan.plan_id) != report:
                raise ValueError('OBSERVE_RETURN_BOUND_AUDIT_CHANGED')
            self._observe_return_bound_report = deepcopy(report)
            self._observe_return_bound_reference = deepcopy(
                self._observe_return_report_references.get(plan.plan_id))
        self.set_state(GraspStages.MOVE_PREGRASP, 'capturing initial observation pose')
        proof = self._observe_return_gateway('/supervisor/capture_observe_return_origin', plan)
        now = rospy.Time.now().to_nsec()
        if not self._observe_return_origin_valid(proof, plan, now):
            raise ValueError('OBSERVE_RETURN_ORIGIN_PROOF_INVALID')
        limit = gcfg.get('unknown_observe_return_session_timeout_sec', 600.)
        if (isinstance(limit, bool) or not isinstance(limit, (float, int))
                or not math.isfinite(limit) or not 30. <= limit <= 600.):
            raise ValueError('OBSERVE_RETURN_SESSION_BUDGET_INVALID')
        epoch = metadata['target_epoch']
        if type(epoch) is not int or epoch < 0 or metadata['target_track_id'] != plan.target_track_id:
            raise ValueError('OBSERVE_RETURN_TARGET_BINDING_INVALID')
        message = dict(version=1, session_id=proof['session_id'],
            selection=deepcopy(self._mode_bound_selection), phase='observing',
            stamp_ns=str(now), deadline_ns=str(now + int(limit * 1e9)),
            target_epoch=epoch, target_track_id=plan.target_track_id,
            origin_plan_id=plan.plan_id,
            origin_snapshot_stamp_ns=str(positive_ns(metadata['origin_snapshot_stamp_ns'])),
            return_verified_stamp_ns='0')
        diagnostic = metadata.get('diagnostic_observation_only', False)
        version = 1 if diagnostic else gcfg.get('unknown_observe_return_session_version', 2)
        if type(version) is not int or version not in (1, 2):
            raise ValueError('OBSERVE_RETURN_SESSION_VERSION_INVALID')
        if version == 2:
            message.update(version=2,
                max_observations=observation_budget(gcfg.get('unknown_observe_return_max_observations', 3)),
                view_index=1, view_plan_id=plan.plan_id,
                view_source_stamp_ns=str(plan.header.stamp.to_nsec()),
                view_request_id='initial:' + plan.plan_id, previous_gain_sha256='0' * 64)
        self._observe_return_session = message
        self._observe_return_origin_proof = proof
        self._observe_return_origin_plan = deepcopy(plan)
        self._observe_return_active_view_plan = deepcopy(plan)
        self._observe_return_publish(message)
        rospy.loginfo('Observe-return origin captured: %s', json.dumps(proof))

    def _coordinated_observation_views(self):
        """A session owns every view: v1 has one; v2 has at most three.

        Neither can borrow an uncounted motion from the legacy recovery path.
        """
        session = self._observe_return_session or {}
        return session.get('version') in (1, 2)

    def _observe_return_view_is_fresh(self, plan, gcfg):
        validity = self._configured_plan_validity(gcfg)
        age = (rospy.Time.now().to_nsec() - positive_ns(plan.header.stamp.to_nsec())) * 1e-9
        if not math.isfinite(validity) or validity <= 0. or not 0. <= age <= validity:
            raise ValueError('OBSERVE_RETURN_SUPPLEMENTAL_PLAN_STALE_OR_FUTURE')

    def _observe_return_supplement_report(self, plan, report):
        session = self._observe_return_session
        metadata = report['observe_return']
        if (parse_selection(report['grasp_mode_selection']) != self._mode_bound_selection
                or report.get('execution_strategy') != 'observe_return'
                or report.get('planning_origin') != 'observe_return_supplemental_observation'
                or report.get('observation_motion_performed') is not True
                or metadata.get('phase') != 'supplemental_observation'
                or metadata.get('version') != 2
                or metadata.get('diagnostic_observation_only', False) is not False
                or metadata.get('trigger_code') != 'BILATERAL_SURFACE_EVIDENCE_MISSING'
                or metadata.get('contact_or_motion_authorized') is not False
                or plan.diagnostic != 'FAR_FIELD_OBSERVATION_PLAN'
                or plan.target_track_id != session['target_track_id']
                or metadata['view_plan_id'] != plan.plan_id
                or metadata['view_source_stamp_ns'] != plan.header.stamp.to_nsec()
                or metadata['previous_view_index'] != session['view_index']
                or metadata['view_index'] != session['view_index'] + 1
                or any(metadata[key] != session[key] for key in (
                    'session_id', 'origin_plan_id', 'target_epoch', 'target_track_id', 'max_observations'))
                or any(positive_ns(metadata[key]) != positive_ns(session[key])
                    for key in ('origin_snapshot_stamp_ns', 'deadline_ns'))
                or not information_gain_valid(metadata['previous_gain'])
                or evidence_digest(metadata['previous_gain']) != metadata['previous_gain_sha256']
                or unknown_observation_audit_error(report)
                or not audit_matches_preview(report, plan, False)):
            raise ValueError('OBSERVE_RETURN_SUPPLEMENTAL_AUDIT_INVALID')
        gain = metadata['previous_gain']
        failure = metadata['failed_search']
        opportunity = metadata['visual_opportunity']
        yielded = (contact_search_yield_proof_valid(failure)
            and opportunity.get('yielded_contact_search') == failure
            and 'transition_ns' in failure['yield_evidence']
            and failure['yield_evidence']['transition_ns'] <= rospy.Time.now().to_nsec()
            and failure['yield_evidence']['original_phase_deadline_ns']
                <= positive_ns(session['deadline_ns']))
        completed = (failure['selected'] is False and failure['terminated_early'] is False
            and failure['reachable_count'] == 0
            and failure['checked_count'] == failure['shortlist_count']
            and failure['contact_search_code'] in ('NEAR_FIELD_NO_HARD_SAFE_CANDIDATE',
                'NEAR_FIELD_NO_REACHABLE_CANDIDATE', 'NEAR_FIELD_SURFACE_VIEW_REQUIRED',
                STABLE_RECHECK_REJECTED))
        if (gain['target_epoch'] != session['target_epoch']
                or gain['target_track_id'] != session['target_track_id']
                or positive_ns(gain['source_stamp_ns']) <= positive_ns(session['view_source_stamp_ns'])
                or positive_ns(failure['snapshot_stamp_ns']) < positive_ns(gain['source_stamp_ns'])
                or failure['target_epoch'] != session['target_epoch']
                or not (completed or yielded)
                or any(opportunity[key] != failure[key] for key in (
                    'request_id', 'generation', 'target_epoch', 'snapshot_stamp_ns', 'contact_search_code'))
                or opportunity.get('contact_or_motion_authorized') is not False
                or opportunity.get('policy') != 'current_request_independent_bilateral_view_gap'):
            raise ValueError('OBSERVE_RETURN_SUPPLEMENTAL_EVIDENCE_INVALID')
        if (failure['contact_search_code'] == 'NEAR_FIELD_SURFACE_VIEW_REQUIRED'
                and (not surface_view_required_search_proof_valid(failure)
                     or opportunity.get('completed_contact_search') != failure)):
            raise ValueError('OBSERVE_RETURN_SUPPLEMENTAL_EVIDENCE_INVALID')
        if (failure['contact_search_code'] == STABLE_RECHECK_REJECTED
                and (not completed_stable_recheck_proof_valid(failure)
                    or opportunity.get('completed_contact_search') != failure
                    or type(opportunity.get('checked_count')) is not int
                    or opportunity['checked_count'] != failure['checked_count'])):
            raise ValueError('OBSERVE_RETURN_SUPPLEMENTAL_EVIDENCE_INVALID')
        request_fields = ('session_id', 'view_index', 'previous_view_index', 'origin_plan_id',
            'origin_snapshot_stamp_ns', 'target_epoch', 'target_track_id', 'deadline_ns',
            'max_observations', 'failed_search', 'visual_opportunity', 'previous_gain',
            'previous_gain_sha256', 'contact_or_motion_authorized')
        if evidence_digest({key: metadata[key] for key in request_fields}) != metadata['view_request_id']:
            raise ValueError('OBSERVE_RETURN_SUPPLEMENTAL_REQUEST_CHANGED')
        return metadata

    def _observe_return_supplement_candidate(self):
        with self._grasp6d_plan_guard():
            return deepcopy(getattr(self, 'latest_grasp6d_preview_plan', None))

    def _before_near_field_terminal_recovery(self, last_result, current_plan, gcfg,
                                             move_pose, strict_execute_pose):
        """Consume a separately typed supplemental preview, never an error alone."""
        del last_result
        session = self._observe_return_session or {}
        if session.get('version') != 2 or session.get('phase') != 'observing':
            return None
        candidate = self._observe_return_supplement_candidate()
        if (candidate is None or not candidate.valid
                or candidate.diagnostic != 'FAR_FIELD_OBSERVATION_PLAN'
                or candidate.plan_id == session['view_plan_id']):
            return None
        with self._observe_return_condition:
            report = deepcopy(self._observe_return_reports.get(candidate.plan_id))
        if report is None:
            return dict(waiting=True)
        if report.get('observe_return', {}).get('phase') != 'supplemental_observation':
            return None
        try:
            self._observe_return_live()
            if (not callable(move_pose) or not callable(strict_execute_pose)
                    or session['view_index'] >= session['max_observations']
                    or current_plan.plan_id != session['view_plan_id']):
                raise ValueError('OBSERVE_RETURN_SUPPLEMENTAL_CONTEXT_INVALID')
            if not self._diagnostic_observation_checkpoint(current_plan, gcfg, 'supplemental observation admission'):
                return dict(failed=True)
            failed_execution = getattr(self, '_last_observation_execution_failure', None)
            if (isinstance(failed_execution, dict)
                    and failed_execution.get('plan_id') == current_plan.plan_id):
                raise ValueError('OBSERVE_RETURN_AFTER_FAILED_EXECUTION_FORBIDDEN')
            self._observe_return_view_is_fresh(candidate, gcfg)
            metadata = self._observe_return_supplement_report(candidate, report)
            request_id = metadata['view_request_id']
            if request_id in self._observe_return_submitted_view_requests:
                raise ValueError('OBSERVE_RETURN_VIEW_ALREADY_ATTEMPTED')
            next_session = dict(session, phase='observing', stamp_ns=str(rospy.Time.now().to_nsec()))
            next_session.update({key: metadata[key] for key in VIEW_FIELDS})
            parse_session(next_session, self._mode_bound_selection,
                          rospy.Time.now().to_nsec(), session)
            # Remote acknowledgement is not execution authority. Validate the
            # old frozen authority after its asynchronous wait, then replace
            # plan and local step under the same callback lock. Revocation
            # while waiting must never be cleared by _freeze_execution_plan.
            self._observe_return_publish(next_session)
            self.set_state(GraspStages.PLAN_PREGRASP, 'admitting supplemental observation view')
            proof = self._observe_return_gateway('/supervisor/admit_observe_return_view', candidate)
            origin = self._observe_return_origin_proof or {}
            if (proof.get('code') != 'OBSERVE_RETURN_VIEW_ADMITTED'
                    or proof.get('physical_motion_submitted') is not False
                    or proof.get('origin_plan_id') != session['origin_plan_id']
                    or proof.get('origin_plan_digest') != origin.get('plan_digest')
                    or proof.get('origin_goal_sha256') != origin.get('origin_goal_sha256')
                    or proof.get('epoch_ns') != origin.get('epoch_ns')
                    or proof.get('previous_view_plan_id') != session['view_plan_id']
                    or proof.get('plan_digest') != committed_plan_digest(candidate)
                    or any(proof.get(key) != next_session[key] for key in
                        ('session_id', 'view_index', 'view_request_id', 'previous_gain_sha256'))
                    or any(positive_ns(proof[key]) != positive_ns(next_session[key]) for key in
                        ('view_source_stamp_ns', 'deadline_ns'))):
                raise ValueError('OBSERVE_RETURN_VIEW_ADMISSION_PROOF_INVALID')
            def bind_next_view():
                self._observe_return_view_is_fresh(candidate, gcfg)
                frozen = self._freeze_execution_plan(candidate)
                self._observe_return_session = next_session
                self._observe_return_active_view_plan = deepcopy(frozen)
                self._observe_return_submitted_view_requests.add(request_id)
                return frozen
            validation, frozen = self._invoke_plan_bound_action(current_plan, gcfg,
                'supplemental observation rebind', bind_next_view)
            if not validation.ok:
                raise ValueError(validation.code + ': ' + validation.reason)
            if not self._diagnostic_observation_checkpoint(frozen, gcfg, 'supplemental observation frozen'):
                return dict(failed=True)
            if self._observation_measurement_scope(frozen):
                self._observation_measurement_prepare(frozen)
            from geometry_msgs.msg import PoseStamped
            observation = PoseStamped(header=deepcopy(frozen.header), pose=deepcopy(frozen.poses[0]))
            self._last_motion_settle_sample = None
            if not self._plan_and_execute_pose(GraspStages.MOVE_PREGRASP,
                    'supplemental observation %d' % next_session['view_index'],
                    observation, move_pose, 'supplemental observation',
                    execution_plan_id=frozen.plan_id, execution_plan=frozen, gcfg=gcfg,
                    execute_pose=strict_execute_pose, allow_post_failure_observation_validation=False):
                return dict(failed=True)
            settled = deepcopy(getattr(self, '_last_motion_settle_sample', None))
            cutoff = rospy.Time.now().to_nsec()
            if (not isinstance(settled, dict)
                    or not positive_ns(settled['stationary_window_start_ns'])
                       < positive_ns(settled['stationary_window_end_ns']) <= cutoff):
                raise ValueError('OBSERVE_RETURN_SUPPLEMENTAL_SETTLE_MISSING')
            if (not self._diagnostic_observation_checkpoint(frozen, gcfg, 'supplemental observation reached')
                    or not self._validate_measured_observation_envelope(frozen, gcfg, 'supplemental observation')):
                return dict(failed=True)
            reached = self._wait_for_fresh_observation_camera_target_range(frozen, gcfg, cutoff)
            if not reached.ok:
                raise ValueError(reached.code + ': ' + reached.reason)
            camera = getattr(self, '_last_observation_camera_range_evidence', None)
            if (not isinstance(camera, dict) or positive_ns(camera['image_source_stamp_ns']) < cutoff
                    or positive_ns(camera['observation_stamp_ns']) < cutoff):
                raise ValueError('OBSERVE_RETURN_SUPPLEMENTAL_FRESH_SOURCE_MISSING')
            if not self._diagnostic_observation_checkpoint(frozen, gcfg, 'supplemental observation sampled'):
                return dict(failed=True)
            now = rospy.Time.now().to_nsec()
            budget = gcfg.get('near_field_replan_timeout_sec', 90.)
            if isinstance(budget, bool) or not isinstance(budget, (int, float)) or not math.isfinite(budget) or not 0. < budget <= 300.:
                raise ValueError('OBSERVE_RETURN_CONTACT_BUDGET_INVALID')
            deadline = min(positive_ns(session['deadline_ns']), now + int(budget * 1e9))
            if deadline <= now:
                raise ValueError('OBSERVE_RETURN_SESSION_EXPIRED')
            station_phase = self._ensure_qualified_station_phase(frozen, gcfg)
            if station_phase is None:
                self._set_near_field_active(True, force=True, budget_sec=(deadline-now)/1e9,
                    reference_plan=frozen, absolute_deadline_sec=deadline/1e9)
            else:
                deadline = min(deadline, station_phase.deadline.to_nsec())
            self.set_state(GraspStages.PLAN_PREGRASP,
                'supplemental observation sampled; requiring measured registration and fresh contact evidence')
            rospy.loginfo('Supplemental observation completed: %s', json.dumps(dict(
                session_id=session['session_id'], view_index=next_session['view_index'],
                view_request_id=request_id, view_plan_id=frozen.plan_id,
                origin_plan_id=session['origin_plan_id'], deadline_ns=session['deadline_ns'],
                sampling_after_ns=cutoff, camera_evidence=camera), allow_nan=False))
            return dict(retry=True, current_plan=frozen,
                minimum_stamp_ns=max(now, cutoff + 1), phase_deadline_sec=deadline/1e9)
        except Exception as exc:
            self.set_state(GraspStages.FAILED, 'OBSERVE_RETURN_SUPPLEMENTAL_FAILED: ' + str(exc))
            return dict(failed=True)
        finally:
            request = getattr(self, '_observation_measurement_request', None)
            try:
                if request is not None and request.get('view_plan_id') == candidate.plan_id:
                    self._observation_measurement_revoke('supplemental handoff ended without qualified measurement')
            finally:
                self._observation_measurement_prepared = None

    def _execution_result_message(self, result, message, slot_released=False):
        reference = self._diagnostic_observation_sample_reference
        if result and reference is not None:
            # The recorder consumes the first inactive terminal publication.
            # Classification must be correct during base start_cb cleanup,
            # before control returns to the mode wrapper.
            return ('DIAGNOSTIC_OBSERVATION_COMPLETE: audit_path=%s audit_sha256=%s; '
                    'no grasp performed' % (reference['path'], reference['sha256']))
        return super()._execution_result_message(result, message, slot_released=slot_released)

    def _diagnostic_observation_only(self):
        report = self._observe_return_bound_report or {}
        metadata = report.get('observe_return', {})
        diagnostic = metadata.get('diagnostic_observation_only', False)
        trigger = metadata.get('trigger_code')
        if trigger == _SILHOUETTE_TRIGGER or diagnostic is not False:
            if (trigger != _SILHOUETTE_TRIGGER or diagnostic is not True
                    or metadata.get('phase') != 'observation_prefix'):
                raise ValueError('DIAGNOSTIC_OBSERVATION_TRIGGER_INVALID')
            return True
        return False

    def _diagnostic_observation_checkpoint(self, plan, gcfg, label):
        self._observe_return_live()
        if not self._execution_checkpoint(plan, gcfg, label):
            return False
        # Preserve the same-generation loss latch, including the original
        # Hard support-plane contradiction; a later good frame cannot erase it.
        failure = self._near_field_wait_failure(plan, gcfg)
        if failure is not None:
            self.set_state(GraspStages.FAILED, '%s: %s' % (failure.code, failure.reason))
            return False
        return True

    def _diagnostic_observation_output_directory(self):
        reference = self._observe_return_bound_reference
        if not isinstance(reference, dict):
            raise ValueError('DIAGNOSTIC_OBSERVATION_AUDIT_REFERENCE_MISSING')
        path = os.path.realpath(reference['path'])
        root = os.path.realpath(_EXPERIMENT_ROOT)
        if (not os.path.ismount(_EXPERIMENT_MOUNT)
                or os.path.commonpath((root, path)) != root):
            raise ValueError('DIAGNOSTIC_OBSERVATION_EXTERNAL_STORAGE_UNAVAILABLE')
        with open(path, 'rb') as source:
            payload = source.read()
        if (hashlib.sha256(payload).hexdigest() != reference['sha256']
                or json.loads(payload) != self._observe_return_bound_report):
            raise ValueError('DIAGNOSTIC_OBSERVATION_BOUND_AUDIT_CHANGED')
        return os.path.dirname(path)

    def _execute_diagnostic_observation(self, plan, gcfg, observation_pose,
                                        move_pose, strict_execute_pose, write_audit):
        """One strict observation, then source-bound evidence only.

        Never enter the grasp executor: no gripper command, range correction,
        contact replan or return. A failed gate is terminal, not a retry.
        """
        try:
            if not self._diagnostic_observation_only():
                raise ValueError('DIAGNOSTIC_OBSERVATION_TRIGGER_REQUIRED')
            report = self._observe_return_bound_report
            session = self._observe_return_session
            metadata = report['observe_return']
            if (self._observe_return_report(plan) != report
                    or session is None or session['phase'] != 'observing'
                    or session['origin_plan_id'] != plan.plan_id
                    or session['target_track_id'] != plan.target_track_id
                    or session['target_epoch'] != metadata['target_epoch']
                    or not callable(strict_execute_pose)):
                raise ValueError('DIAGNOSTIC_OBSERVATION_BOUND_CONTEXT_INVALID')
            if gcfg.get('observation_camera_target_range_check_enabled') is not True:
                raise ValueError('DIAGNOSTIC_OBSERVATION_FRESH_RANGE_REQUIRED')
            directory = self._diagnostic_observation_output_directory()
            source_key = (json.dumps(self._mode_bound_selection, sort_keys=True),
                          metadata['target_epoch'], metadata['origin_snapshot_stamp_ns'])
            if source_key in self._diagnostic_observation_consumed_sources:
                raise ValueError('DIAGNOSTIC_OBSERVATION_ALREADY_ATTEMPTED')
            if not self._diagnostic_observation_checkpoint(plan, gcfg, 'diagnostic observation entry'):
                return False
            self._diagnostic_observation_consumed_sources.add(source_key)
            self._last_motion_settle_sample = None
            self._last_measured_observation_envelope_sample = None
            self._last_observation_camera_range_evidence = None
            if not self._plan_and_execute_pose(
                    GraspStages.MOVE_PREGRASP, 'diagnostic 6D observation',
                    observation_pose, move_pose, 'diagnostic 6D observation',
                    execution_plan_id=plan.plan_id, execution_plan=plan, gcfg=gcfg,
                    execute_pose=strict_execute_pose,
                    allow_post_failure_observation_validation=False):
                return False
            # The shared executor already requires a fresh independent
            # post-action stationary window; sampling adds no physical action.
            settled = deepcopy(getattr(self, '_last_motion_settle_sample', None))
            if not isinstance(settled, dict):
                raise ValueError('DIAGNOSTIC_OBSERVATION_SETTLE_EVIDENCE_MISSING')
            cutoff = rospy.Time.now().to_nsec()
            if (positive_ns(settled['stationary_window_end_ns']) > cutoff
                    or positive_ns(settled['stationary_window_start_ns'])
                       >= positive_ns(settled['stationary_window_end_ns'])):
                raise ValueError('DIAGNOSTIC_OBSERVATION_SETTLE_EVIDENCE_INVALID')
            if not self._diagnostic_observation_checkpoint(plan, gcfg, 'diagnostic observation arrival'):
                return False
            if not self._validate_measured_observation_envelope(
                    plan, gcfg, 'diagnostic 6D observation'):
                return False
            result = self._wait_for_fresh_observation_camera_target_range(plan, gcfg, cutoff)
            if not result.ok:
                self.set_state(GraspStages.FAILED, '%s: %s' % (result.code, result.reason))
                return False
            camera = deepcopy(getattr(self, '_last_observation_camera_range_evidence', None))
            if (not isinstance(camera, dict)
                    or positive_ns(camera['image_source_stamp_ns']) < cutoff
                    or positive_ns(camera['observation_stamp_ns']) < cutoff):
                raise ValueError('DIAGNOSTIC_OBSERVATION_FRESH_SOURCE_MISSING')
            if not self._diagnostic_observation_checkpoint(plan, gcfg, 'diagnostic observation sampling'):
                return False
            self._diagnostic_observation_output_directory()
            marker = dict(version=1, code='DIAGNOSTIC_OBSERVATION_COMPLETE',
                diagnostic_observation_only=True, grasp_completed=False,
                contact_and_return_authorized=False, gripper_command_submitted=False,
                additional_motion_submitted=False, selection=deepcopy(self._mode_bound_selection),
                plan_id=plan.plan_id, target_track_id=plan.target_track_id,
                target_epoch=metadata['target_epoch'], session_id=session['session_id'],
                origin_snapshot_stamp_ns=metadata['origin_snapshot_stamp_ns'],
                source_stamp_ns=camera['image_source_stamp_ns'], sampling_after_ns=cutoff,
                sampling_marker_only=True, physical_contour_certified=False,
                original_audit_reference=deepcopy(self._observe_return_bound_reference),
                stationary_window=settled, camera_range=camera,
                measured_observation_envelope=deepcopy(self._last_measured_observation_envelope_sample))
            session_digest = hashlib.sha256(session['session_id'].encode()).hexdigest()[:24]
            path = os.path.join(directory, 'diagnostic_observation_%s_%s_%d.json' % (
                plan.plan_id, session_digest, camera['image_source_stamp_ns']))
            if os.path.exists(path):
                raise ValueError('DIAGNOSTIC_OBSERVATION_SAMPLE_ALREADY_EXISTS')
            reference = write_audit(path, marker)
            with open(reference['path'], 'rb') as source:
                saved = source.read()
            if hashlib.sha256(saved).hexdigest() != reference['sha256'] or json.loads(saved) != marker:
                raise ValueError('DIAGNOSTIC_OBSERVATION_SAMPLE_WRITE_FAILED')
            if not self._diagnostic_observation_checkpoint(plan, gcfg, 'diagnostic observation saved'):
                return False
            self._diagnostic_observation_sample_reference = reference
            rospy.loginfo('Diagnostic observation source sample: %s', json.dumps(marker, allow_nan=False))
            self.set_state(GraspStages.SUCCESS, 'DIAGNOSTIC_OBSERVATION_COMPLETE: '
                'source_stamp_ns=%d audit_path=%s audit_sha256=%s; no grasp performed' % (
                    camera['image_source_stamp_ns'], reference['path'], reference['sha256']), True)
            return True
        except Exception as exc:
            self.set_state(GraspStages.FAILED, 'DIAGNOSTIC_OBSERVATION_FAILED: ' + str(exc))
            return False

    def _after_near_field_contact_freeze(self, frozen, gcfg):
        """Publish this task's explicit selection after all admission and freeze gates.

        Preview remains inference-only. The gateway still checks its captured
        origin/session and current task before accepting this notification.
        """
        selection = self._mode_bound_selection or {}
        if (selection.get('strategy') != 'observe_return'
                or (self._observe_return_session or {}).get('phase') != 'observing'):
            return
        with self._grasp6d_plan_guard():
            self._observe_return_live()
            validation = self._validate_bound_plan_locked(frozen, gcfg)
            if not validation.ok:
                raise ValueError(validation.code + ': ' + validation.reason)
            with self._following_evidence_lock:
                epoch = self._following_epoch_ns
                origin = self._observe_return_origin_proof or {}
                if epoch != positive_ns(origin.get('epoch_ns')):
                    raise ValueError('CONTACT_FREEZE_CONTROL_EPOCH_CHANGED')
                phase = self._published_near_field_phase
                if (phase is None or not phase.active
                        or phase.phase_id != self._near_field_phase_id
                        or phase.planning_stage != 'contact_replan'
                        or phase.reference_target_track_id != frozen.target_track_id):
                    raise ValueError('CONTACT_FREEZE_PHASE_CHANGED')
                message = build_contact_freeze_notice(frozen,
                    selection=selection, session=self._observe_return_session,
                    epoch_ns=epoch, issued_ns=rospy.Time.now().to_nsec(),
                    phase_id=phase.phase_id, phase_deadline_ns=phase.deadline.to_nsec())
                self._frozen_contact_pub.publish(String(data=json.dumps(
                    message, sort_keys=True, allow_nan=False)))

    def _before_near_field_contact_freeze(self, candidate, current_plan, gcfg):
        """None permits freezing; a retry advances only the new strategy phase."""
        selection = self._mode_bound_selection or {}
        if selection.get('strategy') != 'observe_return':
            return None
        try:
            if self._diagnostic_observation_only():
                raise ValueError('DIAGNOSTIC_OBSERVATION_CONTACT_AND_RETURN_FORBIDDEN')
            self._observe_return_live()
            session = self._observe_return_session
            origin_plan = self._observe_return_origin_plan or current_plan
            active_plan_id = (session.get('view_plan_id', session['origin_plan_id']) if session else None)
            if session is None or current_plan.plan_id != active_plan_id:
                raise ValueError('OBSERVE_RETURN_ORIGIN_CONTEXT_MISSING')
            if session['phase'] == 'returned':
                report = self._observe_return_report(candidate, allow_returned=True)
                metadata = report['observe_return']
                if (metadata.get('phase') != 'returned_contact'
                        or metadata.get('session_id') != session['session_id']
                        or metadata.get('origin_plan_id') != origin_plan.plan_id
                        or type(metadata.get('target_epoch')) is not int
                        or metadata['target_epoch'] != session['target_epoch']
                        or any(positive_ns(metadata[key]) != positive_ns(session[key])
                               for key in ('origin_snapshot_stamp_ns', 'return_verified_stamp_ns',
                                           'deadline_ns', 'contact_deadline_ns'))
                        or candidate.target_track_id != session['target_track_id']
                        or candidate.header.stamp.to_nsec() <= positive_ns(session['return_verified_stamp_ns'])
                        or rospy.Time.now().to_nsec() >= positive_ns(session['contact_deadline_ns'])):
                    raise ValueError('OBSERVE_RETURN_FINAL_CONTACT_BINDING_INVALID')
                return None
            if session['phase'] != 'observing' or candidate.fused_view_count < 2:
                raise ValueError('OBSERVE_RETURN_ADDITIONAL_VIEW_REQUIRED')
            if candidate.target_track_id != session['target_track_id']:
                raise ValueError('OBSERVE_RETURN_TARGET_CHANGED')
            return_to_origin = gcfg.get('unknown_observe_return_return_to_origin', False)
            if type(return_to_origin) is not bool:
                raise ValueError('OBSERVE_RETURN_RETURN_OPTION_MUST_BE_BOOL')
            if not return_to_origin:
                # The full four-stage plan was checked from actual reached
                # state. Continue through the unchanged base freeze/current
                # source/state checks and strict pregrasp execution.
                if getattr(candidate, 'refinement_status', '') != 'VALID_3D':
                    raise ValueError('OBSERVE_RETURN_REGISTERED_CONTACT_REQUIRED')
                self._observe_return_report(candidate, observed_contact=True)
                return None
            returning = dict(session, phase='returning', stamp_ns=str(rospy.Time.now().to_nsec()))
            self._observe_return_publish(returning)
            self._observe_return_session = returning
            self.set_state(GraspStages.MOVE_PREGRASP, 'returning to initial observation pose')
            proof = self._observe_return_gateway('/supervisor/return_observe_return_origin', origin_plan,
                bound_plan=current_plan, gcfg=gcfg)
            origin_proof = self._observe_return_origin_proof or {}
            if (proof.get('code') != 'OBSERVE_RETURN_MEASURED_ARRIVAL'
                    or proof.get('session_id') != session['session_id']
                    or proof.get('post_command_feedback_valid') is not True
                    or proof.get('controller_action_completed') is not True
                    or proof.get('command_submission_attempted') is not True
                    or any(key not in origin_proof or proof.get(key) != origin_proof[key]
                           for key in ('epoch_ns', 'plan_digest', 'model_sha256',
                                       'origin_accepted_stamp_ns', 'origin_joint_positions_rad',
                                       'origin_tool_pose', 'origin_goal_sha256'))):
                raise ValueError('OBSERVE_RETURN_MEASURED_RETURN_REQUIRED')
            verified = positive_ns(proof['accepted_stamp_ns'])
            completed = positive_ns(proof['completion_stamp_ns'])
            now = rospy.Time.now().to_nsec()
            if not positive_ns(returning['stamp_ns']) < completed < verified <= now:
                raise ValueError('OBSERVE_RETURN_FRESH_RETURN_PROOF_REQUIRED')
            if self._observation_measurement_scope(current_plan):
                if self._wait_for_motion_settle('observe-return measured origin') is not True:
                    raise ValueError('OBSERVATION_MEASUREMENT_RETURN_NOT_SETTLED')
                self._observation_measurement_finish(current_plan, gcfg)
                now = rospy.Time.now().to_nsec()
            budget = float(gcfg.get('near_field_replan_timeout_sec', 90.))
            if not math.isfinite(budget) or not 0. < budget <= 300.:
                raise ValueError('OBSERVE_RETURN_CONTACT_BUDGET_INVALID')
            contact_deadline = min(positive_ns(session['deadline_ns']), now + int(budget * 1e9))
            if contact_deadline <= now:
                raise ValueError('OBSERVE_RETURN_SESSION_EXPIRED')
            station_phase = self._ensure_qualified_station_phase(current_plan, gcfg)
            if station_phase is not None:
                contact_deadline = min(contact_deadline, station_phase.deadline.to_nsec())
            returned = dict(returning, phase='returned', stamp_ns=str(now),
                return_verified_stamp_ns=str(verified), contact_deadline_ns=str(contact_deadline))
            self._observe_return_publish(returned)
            self._observe_return_session = returned
            if station_phase is None:
                self._set_near_field_active(True, force=True, budget_sec=(contact_deadline-now)/1e9,
                    reference_plan=current_plan, absolute_deadline_sec=contact_deadline/1e9)
            self.set_state(GraspStages.PLAN_PREGRASP,
                'initial pose measured; replanning complete grasp from returned state')
            rospy.loginfo('Observe-return measured arrival: %s', json.dumps(proof))
            return dict(retry=True, minimum_stamp_ns=max(verified + 1, now),
                        phase_deadline_sec=contact_deadline/1e9)
        except Exception as exc:
            self._observation_measurement_revoke(str(exc))
            self.set_state(GraspStages.FAILED, 'OBSERVE_RETURN_FAILED: ' + str(exc))
            return dict(failed=True)
