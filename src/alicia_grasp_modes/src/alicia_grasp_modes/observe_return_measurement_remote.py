"""Remote participant in a bounded, identity-only observation handoff.

The task owns execution and the producer owns three-frame measurement. This
consumer joins their receipts with the final actual object and mask. No receipt
creates geometry, extends a source timestamp, or unlocks contact planning.
"""
from copy import deepcopy
import json
import math
import time

import numpy as np
import rospy
from std_msgs.msg import Header, String

from alicia_flexible_grasp.robot.observation_path_guard import committed_plan_digest
from .observe_return_measurement import (
    REQUEST_TOPIC, ACK_TOPIC, parse_measurement_request, measurement_ack,
    validate_measurement_ack, qualified_source_stamps, measurement_digest, STATION_REQUALIFICATION,
    ACTUATION_BOOTSTRAP, bootstrap_source_deadline)
from .unknown_pending_identity import reference_binding


class ObserveReturnMeasurementRemoteMixin:
    def _reset_observation_measurement_remote(self):
        self._measurement_remote_request = None
        self._measurement_remote_unknown_ack = None
        self._measurement_remote_reference = None
        self._measurement_remote_objects = {}
        self._measurement_remote_masks = {}
        self._measurement_remote_status = {}
        self._measurement_remote_sources = None
        self._measurement_remote_admission = None
        self._measurement_remote_future_acks = {}
        self._measurement_bootstrap_deadline_mono = None
        if not hasattr(self, '_measurement_bootstrap_seen'):
            self._measurement_bootstrap_seen = set()

    def _setup_observation_measurement_remote_io(self):
        self._measurement_remote_epoch_ns = 0
        self._measurement_remote_ack_pub = rospy.Publisher(ACK_TOPIC, String, queue_size=16)
        self._measurement_remote_request_sub = rospy.Subscriber(
            REQUEST_TOPIC, String, self._measurement_remote_request_cb, queue_size=16)
        self._measurement_remote_ack_sub = rospy.Subscriber(
            ACK_TOPIC, String, self._measurement_remote_ack_cb, queue_size=16)
        self._measurement_remote_epoch_sub = rospy.Subscriber(
            '/alicia_d/control_reference_epoch', Header,
            self._measurement_remote_epoch_cb, queue_size=1)

    def _measurement_remote_active(self):
        request = getattr(self, '_measurement_remote_request', None)
        return request is not None and request['state'] in ('executing', 'requalifying')

    def _measurement_remote_ack(self, request, accepted=True, reason='OK', qualified=False):
        try:
            ack = measurement_ack(request, 'remote', accepted=accepted, reason=reason,
                qualified=qualified, source_stamps_ns=(self._measurement_remote_sources or [])
                if qualified else None)
        except (ValueError, KeyError, TypeError):
            # An unencodable rejected request receives no acceptance receipt.
            return
        self._measurement_remote_ack_pub.publish(String(data=json.dumps(ack, allow_nan=False)))

    def _measurement_remote_context(self, request):
        if rospy.get_param('/gui/joint_direct_mode', False) is not False:
            raise ValueError('OBSERVATION_MEASUREMENT_MANUAL_TAKEOVER')
        if request.get('motion_kind') == ACTUATION_BOOTSTRAP:
            return self._measurement_remote_bootstrap_context(request)
        session = self._observe_return_session or {}
        identity = self._current_stream_target_identity()
        phase = 'observing' if request['motion_kind'] == 'observation' else 'returning'
        restation = request.get(STATION_REQUALIFICATION) is not None
        phase_valid = (self._observe_return_phase == 'observing_contact' if restation
                       else self._observe_return_paused and self._observe_return_phase == phase)
        if (not self._selection_is_current() or request['selection'] != self._mode_selection
                or not self._observe_return_selected() or not phase_valid
                or session.get('phase') != phase
                or session.get('version') != 2
                or self._observe_return_diagnostic_observation_only
                or self._unknown_identity_retired_selection == self._mode_selection
                or self._unknown_identity_pending is not None
                or identity.epoch != request['target_epoch']
                or identity.track_id != request['target_track_id']
                or any(session.get(k) != request[k] for k in (
                    'session_id', 'selection', 'target_epoch', 'target_track_id',
                    'origin_plan_id', 'origin_snapshot_stamp_ns', 'view_plan_id',
                    'view_source_stamp_ns', 'view_index', 'deadline_ns'))):
            raise ValueError('OBSERVATION_MEASUREMENT_REMOTE_SESSION_CHANGED')
        if getattr(self, '_measurement_remote_epoch_ns', 0) != request['control_epoch_ns']:
            raise ValueError('OBSERVATION_MEASUREMENT_CONTROL_EPOCH_CHANGED')
        view = self._observe_return_prefix
        origin = self._observe_return_origin_prefix
        if (view is None or not view.valid or view.plan_id != request['view_plan_id']
                or view.header.stamp.to_nsec() != request['view_source_stamp_ns']
                or view.target_track_id != request['target_track_id']
                or view.model_choice != 'unknown_tabletop'
                or view.diagnostic != 'FAR_FIELD_OBSERVATION_PLAN'
                or committed_plan_digest(view) != request['plan_sha256']
                or origin is None or origin.plan_id != request['origin_plan_id']
                or origin.header.stamp.to_nsec() != request['origin_snapshot_stamp_ns']):
            raise ValueError('OBSERVATION_MEASUREMENT_REMOTE_PLAN_CHANGED')
        now = int(self._ros_source_clock_ns())
        cutoff = (request['capture_deadline_ns'] if request['state'] == 'requalifying'
                  else request['deadline_ns'])
        if restation:
            cutoff = min(cutoff, request[STATION_REQUALIFICATION]['deadline_ns'])
        if now >= cutoff:
            raise ValueError('OBSERVATION_MEASUREMENT_REMOTE_EXPIRED')

    def _measurement_remote_bootstrap_context(self, request):
        """Bind the pre-session prefix without inventing a captured origin."""
        identity = self._current_stream_target_identity()
        view = self._observe_return_prefix
        origin = self._observe_return_origin_prefix
        execution = getattr(self, 'latest_rich_plan', None)
        controller = self._promotion_controller()
        if (not self._selection_is_current() or request['selection'] != self._mode_selection
                or not self._observe_return_selected()
                or self._observe_return_session is not None
                or self._observe_return_phase != 'observation_prefix'
                or self._observe_return_diagnostic_observation_only
                or self._observe_return_pending_supplement is not None
                or self._unknown_identity_retired_selection == self._mode_selection
                or self._unknown_identity_pending is not None
                or identity.epoch != request['target_epoch']
                or identity.track_id != request['target_track_id']
                or self._measurement_remote_epoch_ns != request['control_epoch_ns']):
            raise ValueError('ACTUATION_BOOTSTRAP_REMOTE_OWNER_CHANGED')
        for plan in (view, origin, execution):
            if (plan is None or not plan.valid or plan.plan_id != request['view_plan_id']
                    or plan.header.stamp.to_nsec() != request['view_source_stamp_ns']
                    or plan.target_track_id != request['target_track_id']
                    or plan.model_choice != 'unknown_tabletop'
                    or plan.diagnostic != 'FAR_FIELD_OBSERVATION_PLAN'
                    or committed_plan_digest(plan) != request['plan_sha256']):
                raise ValueError('ACTUATION_BOOTSTRAP_REMOTE_PLAN_CHANGED')
        if not controller.has_execution or controller.execution_plan_id != request['view_plan_id']:
            raise ValueError('ACTUATION_BOOTSTRAP_EXECUTION_NOT_BOUND')
        cutoff = bootstrap_source_deadline(request, self._configured_execution_plan_validity_sec())
        if request['state'] == 'requalifying':
            cutoff = min(cutoff, request['capture_deadline_ns'])
        now = int(self._ros_source_clock_ns())
        mono = self._measurement_bootstrap_deadline_mono
        if now >= cutoff or (mono is not None and time.monotonic() >= mono):
            raise ValueError('ACTUATION_BOOTSTRAP_REMOTE_EXPIRED')
        if self._measurement_remote_active() and (mono is None or not self._observe_return_paused):
            raise ValueError('ACTUATION_BOOTSTRAP_REMOTE_HANDOFF_CHANGED')

    def _measurement_remote_fail(self, reason):
        request = self._measurement_remote_request
        if request is None or request['state'] not in ('executing', 'requalifying'):
            return
        self._measurement_remote_ack(request, accepted=False, reason=reason)
        # Original retirement cancels tickets and clears the old target/surface.
        # It is deliberately irreversible within the same selection.
        self._unknown_identity_retire('observation measurement handoff: ' + str(reason))
        self._reset_observation_measurement_remote()

    def _unknown_identity_retire(self, reason):
        request = getattr(self, '_measurement_remote_request', None) or {}
        if (request.get('motion_kind') == ACTUATION_BOOTSTRAP
                and request.get('state') in ('executing', 'requalifying', 'qualified')
                and request.get('selection') == self._mode_selection):
            controller = getattr(self, 'execution_plan_controller', None)
            if (controller is not None and controller.has_execution
                    and controller.execution_plan_id == request['view_plan_id']):
                # True identity loss must reach the gateway even while the
                # task's synchronous prefix call holds its plan lock. Clearing
                # this exact bootstrap owner lets the original invalidation
                # publish its rich-plan tombstone instead of preserving a
                # frozen plan for ordinary contact occlusion. Expected moving
                # frames never enter identity retirement.
                controller.clear_execution()
                self._execution_authority_ticket = None
        return super()._unknown_identity_retire(reason)

    def _measurement_remote_epoch_cb(self, msg):
        if (getattr(msg, 'frame_id', '') != 'sdk_reference_epoch'
                or msg.stamp.to_nsec() <= 0):
            return
        with self._mode_control_lock, self._stream_condition, self._geometry_state_guard():
            self._measurement_remote_epoch_ns = int(msg.stamp.to_nsec())
            if self._measurement_remote_active():
                try:
                    self._measurement_remote_context(self._measurement_remote_request)
                except (ValueError, KeyError, TypeError, AttributeError) as exc:
                    self._measurement_remote_fail(str(exc))

    def _observation_measurement_poll(self):
        with self._mode_control_lock, self._stream_condition, self._geometry_state_guard():
            if self._measurement_remote_admission is not None:
                self._measurement_remote_try_admit()
                return True
            if not self._measurement_remote_active():
                return False
            try:
                self._measurement_remote_context(self._measurement_remote_request)
            except (ValueError, KeyError, TypeError, AttributeError) as exc:
                self._measurement_remote_fail(str(exc))
            return True

    def _measurement_remote_try_admit(self):
        request = self._measurement_remote_admission
        if request is None:
            return False
        try:
            self._measurement_remote_context(request)
            now = int(self._ros_source_clock_ns())
            if request.get(STATION_REQUALIFICATION) is not None:
                try:
                    reference = self._station_requalification_admit(request)
                except ValueError as exc:
                    if str(exc) == 'STATION_REQUALIFICATION_FEEDBACK_UNPROVEN':
                        return False  # SDK connection can arrive after the task; no ACK.
                    raise
                self._reset_observation_measurement_remote()
                self._measurement_remote_reference = reference
                self._measurement_remote_request = request
                self._measurement_remote_admission = None
                self._measurement_remote_context(request)
                self._measurement_remote_ack(request)
                return True
            obj = self.latest_object
            if obj is None or not obj.detected:
                raise ValueError('OBSERVATION_MEASUREMENT_REMOTE_TARGET_UNAVAILABLE')
            maximum = min(float(rospy.get_param('/unknown_perception', {}).get(
                'max_frame_age_sec', .8)), .8)
            if not math.isfinite(maximum) or maximum <= 0:
                raise ValueError('OBSERVATION_MEASUREMENT_REMOTE_FRAME_AGE_INVALID')
            # Object, mask and producer status use independent connections.
            # Prefer the latest complete fresh join; otherwise wait within the
            # original frame-age bound, without ACK or source/time mutation.
            maximum_ns = round(maximum * 1e9)
            if now >= request['stamp_ns'] + maximum_ns:
                raise ValueError('OBSERVATION_MEASUREMENT_REMOTE_REFERENCE_STALE')
            latest_ns = obj.header.stamp.to_nsec()
            candidates = sorted(set(self._unknown_identity_objects)
                & set(self._unknown_identity_masks) & set(self._unknown_identity_status), reverse=True)
            for ns in candidates:
                status = self._unknown_identity_status[ns]
                mask = self._unknown_identity_masks[ns]
                cached_obj = self._unknown_identity_objects[ns]
                if (not request['view_source_stamp_ns'] <= ns <= latest_ns
                        or not ns <= now < ns + maximum_ns
                        or not cached_obj.detected or status.get('state') != 'ready'
                        or not mask[0] or mask[1] != status.get('color_frame_id')):
                    continue
                binding = reference_binding(status, self._mode_selection)
                if status['anchor_source_stamp_ns'] > request['origin_snapshot_stamp_ns']:
                    raise ValueError('OBSERVATION_MEASUREMENT_REMOTE_REFERENCE_CHANGED')
                self._measurement_remote_reference = dict(binding=binding,
                    maximum_ns=maximum_ns, source_ns=ns)
                if request.get('motion_kind') == ACTUATION_BOOTSTRAP:
                    # Stop new work and make existing workers fail their
                    # ordinary publication guard before either owner ACKs.
                    # Do not advance target/view identity or renew its source.
                    key = (measurement_digest(request['selection']), request['control_epoch_ns'])
                    if key in self._measurement_bootstrap_seen:
                        raise ValueError('ACTUATION_BOOTSTRAP_ALREADY_ADMITTED')
                    self._observe_return_paused = True
                    self._measurement_bootstrap_deadline_mono = time.monotonic() + (
                        request['deadline_ns'] - now) / 1e9
                    self._measurement_bootstrap_seen.add(key)
                self._measurement_remote_request = request
                self._measurement_remote_admission = None
                self._measurement_remote_context(request)
                self._measurement_remote_ack(request)
                return True
            return False
        except (ValueError, KeyError, TypeError, AttributeError, OverflowError) as exc:
            self._measurement_remote_admission = None
            self._measurement_remote_ack(request, accepted=False, reason=str(exc))
            return False

    def _measurement_remote_request_cb(self, msg):
        raw = None
        with self._mode_control_lock, self._stream_condition, self._geometry_state_guard():
            try:
                if len(msg.data) > 32768:
                    raise ValueError('OBSERVATION_MEASUREMENT_REQUEST_TOO_LARGE')
                raw = json.loads(msg.data)
                if self._measurement_remote_admission is not None:
                    if raw == self._measurement_remote_admission:
                        self._measurement_remote_try_admit()
                        return
                    raise ValueError('OBSERVATION_MEASUREMENT_ADMISSION_PENDING')
                previous = self._measurement_remote_request
                if previous is not None and raw == previous:
                    self._measurement_remote_context(previous)
                    if previous['state'] == 'requalifying' and self._measurement_remote_sources is not None:
                        self._measurement_remote_check_source(previous, self._measurement_remote_sources[-1])
                    self._measurement_remote_ack(previous,
                        qualified=previous['state'] == 'requalifying'
                        and self._measurement_remote_sources is not None)
                    return
                prior = previous
                if previous is not None and previous['state'] == 'qualified':
                    if (raw.get('lease_id') == previous['lease_id']
                            or type(raw.get('stamp_ns')) is not int
                            or raw['stamp_ns'] <= previous['stamp_ns']):
                        raise ValueError('OBSERVATION_MEASUREMENT_OLD_LEASE_REPLAY')
                    prior = None
                request = parse_measurement_request(raw, self._mode_selection,
                    int(self._ros_source_clock_ns()), prior)
                measurement_digest(request)  # Reject non-finite extra fields too.
                if request['state'] == 'revoked':
                    self._measurement_remote_fail(request.get('reason', 'task revoked handoff'))
                    self._measurement_remote_ack(request)
                    return
                self._measurement_remote_context(request)
                if request['state'] == 'executing':
                    if request.get(STATION_REQUALIFICATION) is None:
                        self._reset_observation_measurement_remote()
                    else:
                        # A registered parent can withdraw old work before the
                        # independent SDK connection arrives; it cannot admit
                        # the new lease or create replacement geometry.
                        self._station_compute_quarantine()
                    self._measurement_remote_admission = request
                    self._measurement_remote_try_admit()
                    return
                elif request['state'] == 'qualified':
                    if (self._measurement_remote_sources is None
                            or request['source_stamps_ns'] != self._measurement_remote_sources):
                        raise ValueError('OBSERVATION_MEASUREMENT_REMOTE_SOURCE_ACK_MISMATCH')
                    self._measurement_remote_check_source(previous,
                        self._measurement_remote_sources[-1])
                self._measurement_remote_request = request
                self._measurement_remote_ack(request)
                future_ack = self._measurement_remote_future_acks.pop(request['request_id'], None)
                if future_ack is not None:
                    self._measurement_remote_ack_cb(String(data=json.dumps(future_ack)))
            except (ValueError, KeyError, TypeError, AttributeError, OverflowError) as exc:
                if (isinstance(raw, dict) and self._measurement_remote_active()
                        and raw.get('lease_id') == self._measurement_remote_request['lease_id']
                        and type(raw.get('stamp_ns')) is int
                        and raw['stamp_ns'] > self._measurement_remote_request['stamp_ns']):
                    self._measurement_remote_fail(str(exc))
                # An unrelated/stale latched request cannot revoke this lease.
                if isinstance(raw, dict) and all(k in raw for k in ('request_id', 'lease_id', 'state')):
                    self._measurement_remote_ack(raw, accepted=False, reason=str(exc))

    def _measurement_remote_ack_cb(self, msg):
        with self._mode_control_lock, self._stream_condition, self._geometry_state_guard():
            if not self._measurement_remote_active():
                return
            try:
                if len(msg.data) > 32768:
                    raise ValueError('OBSERVATION_MEASUREMENT_ACK_TOO_LARGE')
                ack = json.loads(msg.data)
                request = self._measurement_remote_request
                if (ack.get('participant') != 'unknown' or ack.get('lease_id') != request['lease_id']):
                    return
                if ack.get('request_id') != request['request_id']:
                    if (request['state'] == 'executing' and ack.get('state') == 'requalifying'
                            and isinstance(ack.get('request_id'), str)):
                        self._measurement_remote_future_acks[ack['request_id']] = deepcopy(ack)
                        while len(self._measurement_remote_future_acks) > 8:
                            del self._measurement_remote_future_acks[next(iter(self._measurement_remote_future_acks))]
                    return
                self._measurement_remote_context(request)
                ack = validate_measurement_ack(ack, request, 'unknown',
                    int(self._ros_source_clock_ns()), require_qualified=ack.get('qualified') is True)
                if ack['qualified']:
                    self._measurement_remote_unknown_ack = ack
                    self._measurement_remote_join()
            except (ValueError, KeyError, TypeError, AttributeError) as exc:
                self._measurement_remote_fail(str(exc))

    def _measurement_remote_status_cb(self, status):
        with self._mode_control_lock, self._stream_condition, self._geometry_state_guard():
            if self._measurement_remote_admission is not None:
                self._measurement_remote_try_admit()
            if not self._measurement_remote_active():
                return
            if (status.get('generation') != self._mode_selection['generation']
                    or status.get('selection_stamp_ns') != self._mode_selection['stamp_ns']
                    or status.get('mode') != 'unknown'):
                return
            if status.get('target_lost') is True or status.get('first_loss') is not None:
                self._measurement_remote_fail('producer retired target')
                return
            ns = status.get('source_stamp_ns')
            if type(ns) is int and ns > 0:
                self._unknown_identity_cache(self._measurement_remote_status, ns, deepcopy(status))
                self._measurement_remote_join()

    def _measurement_remote_check_source(self, request, ns):
        status = self._measurement_remote_status[ns]
        obj = self._measurement_remote_objects[ns]
        mask = self._measurement_remote_masks[ns]
        receipt = status.get('observation_measurement')
        now = int(self._ros_source_clock_ns())
        if (not isinstance(receipt, dict) or receipt.get('request') != request
                or receipt.get('state') != 'qualified' or receipt.get('qualified') is not True
                or receipt.get('measurement_evaluated') is not True
                or receipt.get('exact_tf_consumed') is not True
                or receipt.get('metric_geometry_authority') is not False
                or receipt.get('motion_authorized') is not False
                or status.get('state') != 'ready'
                or reference_binding(status, self._mode_selection) != self._measurement_remote_reference['binding']
                or not obj.detected or obj.header.stamp.to_nsec() != ns
                or obj.header.frame_id != 'base_link' or obj.label != 'unknown_small'
                or mask.header.stamp.to_nsec() != ns
                or mask.header.frame_id != status['color_frame_id']
                or not ns <= now < ns + self._measurement_remote_reference['maximum_ns']):
            raise ValueError('OBSERVATION_MEASUREMENT_REMOTE_SOURCE_INVALID')
        sources = qualified_source_stamps(receipt.get('source_stamps_ns'), request, now)
        if sources[-1] != ns:
            raise ValueError('OBSERVATION_MEASUREMENT_REMOTE_SOURCE_MISMATCH')
        # Earlier stamps are the owner's individually audited identity history,
        # bounded by this arrival/capture window. Only ns supplies geometry and
        # must remain fresh at both source joins and the final release.
        pixels = self.bridge.imgmsg_to_cv2(mask, desired_encoding='mono8')
        if pixels.ndim != 2 or not np.any(pixels):
            raise ValueError('OBSERVATION_MEASUREMENT_REMOTE_MASK_EMPTY')
        return sources

    def _measurement_remote_join(self):
        request = self._measurement_remote_request
        if (request is None or request['state'] != 'requalifying'
                or self._measurement_remote_sources is not None
                or self._measurement_remote_unknown_ack is None):
            return
        try:
            self._measurement_remote_context(request)
            ack = validate_measurement_ack(self._measurement_remote_unknown_ack, request,
                'unknown', int(self._ros_source_clock_ns()), require_qualified=True)
            ns = ack['source_stamps_ns'][-1]
            if not all(ns in cache for cache in (self._measurement_remote_status,
                    self._measurement_remote_objects, self._measurement_remote_masks)):
                return
            sources = self._measurement_remote_check_source(request, ns)
            if sources != ack['source_stamps_ns']:
                raise ValueError('OBSERVATION_MEASUREMENT_REMOTE_SOURCE_ACK_MISMATCH')
            # The original confidence/association checks still admit the real
            # third frame. No earlier frame is replayed or invented.
            super().mask_cb(self._measurement_remote_masks[ns])
            super().object_cb(self._measurement_remote_objects[ns])
            self._measurement_remote_context(request)
            self._measurement_remote_check_source(request, ns)
            if (self.latest_object is None or not self.latest_object.detected
                    or self.latest_object.header.stamp.to_nsec() != ns):
                raise ValueError('OBSERVATION_MEASUREMENT_REMOTE_ORIGINAL_GATE_REJECTED')
            self._measurement_remote_sources = sources
            self._measurement_remote_ack(request, qualified=True)
        except (ValueError, KeyError, TypeError, AttributeError, OverflowError) as exc:
            self._measurement_remote_fail(str(exc))

    def object_cb(self, msg):
        if not self._measurement_remote_active():
            result = super().object_cb(msg)
            if self._measurement_remote_admission is not None:
                with self._stream_condition, self._geometry_state_guard():
                    self._measurement_remote_try_admit()
            return result
        with self._stream_condition, self._geometry_state_guard():
            if not getattr(msg, 'detected', False):
                self._measurement_remote_fail('producer published false object')
                return super().object_cb(msg)
            self._unknown_identity_cache(self._measurement_remote_objects,
                msg.header.stamp.to_nsec(), deepcopy(msg))
            self._measurement_remote_join()

    def mask_cb(self, msg):
        if not self._measurement_remote_active():
            result = super().mask_cb(msg)
            if self._measurement_remote_admission is not None:
                with self._stream_condition, self._geometry_state_guard():
                    self._measurement_remote_try_admit()
            return result
        with self._stream_condition, self._geometry_state_guard():
            self._unknown_identity_cache(self._measurement_remote_masks,
                msg.header.stamp.to_nsec(), deepcopy(msg))
            self._measurement_remote_join()
