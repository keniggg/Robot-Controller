"""Identity-only support recovery before the first initial observation action.

A task-owned, irreversible commit handshake separates planning ownership from
physical submission. Existing support receipts and source deadlines are used
unchanged. The historical FAR_FIELD plan never supplies current measurement.
"""
from copy import deepcopy
import json
import math
import time

import numpy as np
import rospy
from std_msgs.msg import Bool, String

from .unknown_pending_identity import CODE, validate_support_receipt
from .rgb_observation_source import bracket_feedback

REQUEST_TOPIC = '/grasp_mode/initial_preflight_request'
ACK_TOPIC = '/grasp_mode/initial_preflight_ack'
TOKEN_FIELDS = ('version', 'token_id', 'selection', 'plan_id', 'target_track_id',
                'target_epoch', 'control_epoch_ns', 'q_station_rad', 'anchor_binding',
                'deadline_ns', 'physical_action_submitted')


def token_from_request(request):
    if not isinstance(request, dict) or set(request) != set(TOKEN_FIELDS) | {
            'action', 'request_id', 'stamp_ns'}:
        raise ValueError('initial_preflight_request_shape')
    if (type(request['version']) is not int or request['version'] != 1
            or request['action'] not in ('arm', 'commit', 'revoke')
            or request['physical_action_submitted'] is not False
            or any(not isinstance(request[k], str) or not request[k]
                   for k in ('token_id', 'plan_id', 'target_track_id', 'request_id', 'anchor_binding'))
            or any(type(request[k]) is not int or request[k] <= 0
                   for k in ('stamp_ns', 'control_epoch_ns', 'deadline_ns'))
            or type(request['target_epoch']) is not int or request['target_epoch'] < 0
            or not isinstance(request['q_station_rad'], list) or len(request['q_station_rad']) != 6
            or any(type(v) not in (int, float) or not math.isfinite(v)
                   for v in request['q_station_rad'])):
        raise ValueError('initial_preflight_request_invalid')
    return {k: deepcopy(request[k]) for k in TOKEN_FIELDS}


class InitialPreflightRemoteMixin:
    def _reset_initial_preflight_remote(self):
        self._initial_preflight_remote = None
        self._initial_preflight_retired_tokens = set()

    def _setup_initial_preflight_remote_io(self):
        self._initial_preflight_ack_pub = rospy.Publisher(ACK_TOPIC, String, queue_size=16)
        self._initial_preflight_request_sub = rospy.Subscriber(
            REQUEST_TOPIC, String, self._initial_preflight_request_cb, queue_size=16)
        self._initial_preflight_manual_sub = rospy.Subscriber(
            '/gui/joint_direct_mode', Bool, self._initial_preflight_manual_cb, queue_size=8)

    def _initial_preflight_context(self, *, source_ns=None, allow_handoff=False, verify_owner=False):
        record = self._initial_preflight_remote
        if record is None or record['state'] != 'armed':
            raise ValueError('initial_preflight_not_armed')
        token = record['token']
        now = int(self._ros_source_clock_ns())
        identity = self._current_stream_target_identity()
        session = getattr(self, '_observe_return_session', None) or {}
        prefix = getattr(self, '_observe_return_origin_prefix', None)
        # Object/mask/status admission already checked authoritative selection.
        # Do not repeat ROS RPCs/full plan serialization at every nested scope
        # check. Registration and the final commit independently repeat both.
        if (not self._mode_ready or (verify_owner and not self._selection_is_current())
                or token['selection'] != self._mode_selection
                or token['selection'].get('mode') != 'unknown'
                or token['selection'].get('strategy') != 'observe_return'
                or not self.streaming_enabled
                or self._unknown_identity_retired_selection == self._mode_selection
                or self._observe_return_phase not in ('observation_prefix', 'observing')
                or getattr(self, '_observe_return_diagnostic_observation_only', True)
                or getattr(self, '_observe_return_pending_supplement', None) is not None
                or prefix is None or not prefix.valid or prefix.plan_id != token['plan_id']
                or prefix.diagnostic != 'FAR_FIELD_OBSERVATION_PLAN'
                or prefix.model_choice != 'unknown_tabletop'
                or prefix.target_track_id != token['target_track_id']
                or (verify_owner and self._execution_prefix_digest(prefix) != record['plan_digest'])
                or identity.track_id != token['target_track_id'] or identity.epoch != token['target_epoch']
                or self._measurement_remote_epoch_ns != token['control_epoch_ns']
                or (rospy.get_param if verify_owner else rospy.get_param_cached)(
                    '/gui/joint_direct_mode', False) is not False
                or not record['armed_ns'] <= now < token['deadline_ns']
                or time.monotonic() >= record['deadline_mono']):
            raise ValueError('initial_preflight_owner_changed')
        if session and (session.get('version') != 2 or session.get('phase') != 'observing'
                or session.get('view_index') != 1 or session.get('view_plan_id') != token['plan_id']
                or session.get('origin_plan_id') != token['plan_id']
                or session.get('selection') != token['selection']
                or session.get('target_track_id') != token['target_track_id']
                or session.get('target_epoch') != token['target_epoch']):
            raise ValueError('initial_preflight_session_changed')
        measurement = getattr(self, '_measurement_remote_request', None) or {}
        if measurement.get('state') in ('executing', 'requalifying', 'qualified'):
            if (not allow_handoff or measurement.get('state') != 'executing'
                    or measurement.get('view_index') != 1
                    or measurement.get('motion_kind') != 'observation'
                    or measurement.get('view_plan_id') != token['plan_id']
                    or measurement.get('origin_plan_id') != token['plan_id']
                    or measurement.get('selection') != token['selection']
                    or measurement.get('target_track_id') != token['target_track_id']
                    or measurement.get('target_epoch') != token['target_epoch']
                    or measurement.get('control_epoch_ns') != token['control_epoch_ns']):
                raise ValueError('initial_preflight_observation_already_submitted')
            # This is the existing dual-participant measurement handoff. The
            # task commits only after its original ACKs; no lease is invented.
            if verify_owner:
                self._measurement_remote_context(measurement)
            elif (not self._observe_return_paused
                    or any(session.get(k) != measurement[k] for k in (
                        'session_id', 'selection', 'target_epoch', 'target_track_id',
                        'origin_plan_id', 'origin_snapshot_stamp_ns', 'view_plan_id',
                        'view_source_stamp_ns', 'view_index', 'deadline_ns'))
                    or prefix.header.stamp.to_nsec() != measurement['origin_snapshot_stamp_ns']
                    or measurement['view_source_stamp_ns'] != measurement['origin_snapshot_stamp_ns']
                    or now >= measurement['deadline_ns']):
                raise ValueError('initial_preflight_handoff_context_changed')
        q0 = np.asarray(token['q_station_rad'], dtype=float)
        since = record['armed_ns']
        owner = ('initial', json.dumps(token, sort_keys=True, separators=(',', ':')))
        rows, diagnostic = self._station_feedback_snapshot(
            owner,
            token['control_epoch_ns'], since, now)
        now = diagnostic['validation_now_ns']
        if not record['armed_ns'] <= now < token['deadline_ns'] or time.monotonic() >= record['deadline_mono']:
            raise ValueError('initial_preflight_owner_changed')
        recent = [q for ns, q in rows if ns >= since]
        reason = ('empty' if not rows else
                  'future' if now < diagnostic['latest_source_stamp_ns'] else
                  'stale' if now-diagnostic['latest_source_stamp_ns'] >= 800000000 else
                  'moved' if (any(np.max(np.abs(q-q0)) > .004 for q in recent)
                              or np.max(np.abs(rows[-1][1]-q0)) > .004) else None)
        if reason:
            self._station_feedback_failure('initial_preflight_feedback_'+reason, diagnostic,
                                           owner=owner if reason == 'moved' else None)
        if source_ns is not None:
            q, before, after = bracket_feedback(rows, source_ns, token['control_epoch_ns'])
            if any(np.max(np.abs(np.asarray(v)-q0)) > .004 for v in (q, before[1], after[1])):
                raise ValueError('initial_preflight_source_station_changed')

    @staticmethod
    def _execution_prefix_digest(plan):
        from alicia_flexible_grasp.robot.observation_path_guard import committed_plan_digest
        return committed_plan_digest(plan)

    def _initial_preflight_identity_scope(self):
        if getattr(self, '_initial_preflight_remote', None) is None:
            return False
        try:
            self._initial_preflight_context()
            return True
        except (ValueError, KeyError, TypeError, AttributeError, OverflowError):
            return False

    def _unknown_identity_scope(self):
        return self._initial_preflight_identity_scope() or super()._unknown_identity_scope()

    def _initial_preflight_ack(self, request, accepted, reason='OK'):
        record = getattr(self, '_initial_preflight_remote', None)
        reference = getattr(self, '_unknown_identity_reference', None)
        state = record['state'] if record is not None else 'unarmed'
        if state == 'armed' and self._unknown_identity_pending is not None:
            state = 'pending'
        reference_ns = reference['source_ns'] if reference else 0
        ack = dict(version=1, request_id=request.get('request_id'), token_id=request.get('token_id'),
            action=request.get('action'), accepted=bool(accepted), reason=str(reason), state=state,
            stamp_ns=int(self._ros_source_clock_ns()), reference_source_ns=reference_ns,
            pending_source_ns=(int(getattr(getattr(getattr(self, 'latest_object', None),
                'header', None), 'stamp', rospy.Time()).to_nsec()) if state == 'pending' else 0),
            reference_status=deepcopy(self._unknown_identity_status.get(reference_ns)))
        self._initial_preflight_ack_pub.publish(String(data=json.dumps(ack, sort_keys=True, allow_nan=False)))
        return ack

    def _initial_preflight_request_cb(self, message):
        request = {}
        try:
            request = json.loads(message.data)
            token = token_from_request(request)
            with self._mode_control_lock, self._stream_condition, self._geometry_state_guard():
                now = int(self._ros_source_clock_ns())
                if not 0 <= now-request['stamp_ns'] < 800000000:
                    raise ValueError('initial_preflight_request_stale_or_future')
                record = self._initial_preflight_remote
                if request['action'] == 'arm':
                    if token['token_id'] in self._initial_preflight_retired_tokens:
                        raise ValueError('initial_preflight_token_already_closed')
                    if record is not None:
                        if record['token'] != token or record['state'] != 'armed':
                            raise ValueError('initial_preflight_token_replaced')
                    else:
                        prefix = getattr(self, '_observe_return_origin_prefix', None)
                        if prefix is None or self._unknown_identity_pending is not None:
                            raise ValueError('initial_preflight_no_fresh_initial_prefix')
                        validity = float(self._configured_execution_plan_validity_sec())
                        # The token can never extend the original plan's validity.
                        if (not math.isfinite(validity) or validity <= 0
                                or not now < token['deadline_ns'] <=
                                prefix.header.stamp.to_nsec()+round(validity*1e9)):
                            raise ValueError('initial_preflight_deadline_invalid')
                        self._initial_preflight_remote = dict(token=token, state='armed',
                            armed_ns=now, deadline_mono=time.monotonic()+(token['deadline_ns']-now)/1e9,
                            plan_digest=self._execution_prefix_digest(prefix), proven_gap_statuses={})
                    try:
                        self._initial_preflight_context(verify_owner=True)
                        # Rejoin the actual cached object/mask/READY at its own
                        # source stamp. Publication never refreshes the old ref.
                        self._unknown_identity_join_locked()
                        reference = self._unknown_identity_reference
                        if (reference is None or reference['binding'] != token['anchor_binding']
                                or reference.get('identity_template') is None
                                or not reference['source_ns'] <= now < reference['deadline_ns']
                                or self._unknown_identity_pending is not None):
                            raise ValueError('initial_preflight_fresh_reference_missing')
                        self._initial_preflight_context(source_ns=reference['source_ns'], verify_owner=True)
                    except Exception:
                        self._initial_preflight_remote = None
                        self._initial_preflight_retired_tokens.add(token['token_id'])
                        raise
                else:
                    if record is None or record['token'] != token:
                        raise ValueError('initial_preflight_token_unknown')
                    if request['action'] == 'revoke':
                        record['state'] = 'revoked'
                        self._initial_preflight_retired_tokens.add(token['token_id'])
                    else:
                        self._initial_preflight_context(allow_handoff=True, verify_owner=True)
                        if self._unknown_identity_pending is not None:
                            return self._initial_preflight_ack(request, False, CODE)
                        measurement = getattr(self, '_measurement_remote_request', None) or {}
                        if measurement.get('state') != 'executing':
                            raise ValueError('initial_preflight_original_measurement_handoff_missing')
                        reference = self._measurement_remote_reference
                        now = int(self._ros_source_clock_ns())
                        deadline = ((reference['source_ns'] + reference['maximum_ns'])
                                    if reference is not None else 0)
                        if (reference is None or reference['binding'] != token['anchor_binding']
                                or not reference['source_ns'] <= now < deadline):
                            return self._initial_preflight_ack(request, False, CODE)
                        self._initial_preflight_context(source_ns=reference['source_ns'], allow_handoff=True,
                                                        verify_owner=True)
                        # Commit is irreversible, including duplicate/replayed
                        # arm requests and a physical service that later fails.
                        record['state'] = 'committed'
                        self._initial_preflight_retired_tokens.add(token['token_id'])
                return self._initial_preflight_ack(request, True)
        except (ValueError, KeyError, TypeError, AttributeError, OverflowError) as exc:
            if isinstance(request, dict):
                return self._initial_preflight_ack(request, False, str(exc))

    def _unknown_identity_join_locked(self):
        previous = getattr(self, '_unknown_identity_pending', None)
        super()._unknown_identity_join_locked()
        record = getattr(self, '_initial_preflight_remote', None)
        if (previous is None or record is None or record['state'] != 'armed'
                or self._unknown_identity_pending is not None
                or self._unknown_identity_retired_selection == self._mode_selection
                or not getattr(self.latest_object, 'detected', False)):
            return
        recovered_ns = self.latest_object.header.stamp.to_nsec()
        # Only preserve facts whose complete original receipt was validated
        # within the fixed source deadline and whose measured recovery joined.
        # This is a replay tombstone, never a future identity or motion lease.
        now = int(self._ros_source_clock_ns())
        for ns, status in self._unknown_identity_status.items():
            if not previous['source_ns'] < ns < recovered_ns:
                continue
            try:
                validate_support_receipt(status, previous, now)
            except (ValueError, KeyError, TypeError, OverflowError):
                continue
            record['proven_gap_statuses'][ns] = dict(status=deepcopy(status),
                recovered_ns=recovered_ns, identity=self._current_stream_target_identity())
        # A token is bounded by its original plan lifetime. Keep only the most
        # recent existing cache-sized set; forgotten old callbacks fail closed.
        while len(record['proven_gap_statuses']) > 64:
            del record['proven_gap_statuses'][min(record['proven_gap_statuses'])]

    def _initial_preflight_is_proven_old_gap(self, msg):
        record = getattr(self, '_initial_preflight_remote', None)
        if (record is None or record['state'] not in ('armed', 'committed')
                or getattr(msg, 'detected', None) is not False):
            return False
        ns = msg.header.stamp.to_nsec()
        proven = record['proven_gap_statuses'].get(ns)
        request = getattr(self, '_measurement_remote_request', None) or {}
        reference = getattr(self, '_measurement_remote_reference', None)
        token = record['token']
        if (proven is None or request.get('state') != 'executing' or reference is None
                or request.get('view_index') != 1 or request.get('motion_kind') != 'observation'
                or request.get('view_plan_id') != token['plan_id']
                or request.get('origin_plan_id') != token['plan_id']
                or request.get('selection') != token['selection']
                or request.get('target_track_id') != token['target_track_id']
                or request.get('target_epoch') != token['target_epoch']
                or request.get('control_epoch_ns') != token['control_epoch_ns']
                or not ns < proven['recovered_ns'] <= reference['source_ns']
                or proven['identity'] != self._current_stream_target_identity()
                or self._unknown_identity_status.get(ns) != proven['status']):
            return False
        try:
            self._measurement_remote_context(request)
        except (ValueError, KeyError, TypeError, AttributeError):
            return False
        return True

    def object_cb(self, msg):
        if not hasattr(self, '_mode_control_lock') or not hasattr(self, '_stream_condition'):
            return super().object_cb(msg)
        with self._mode_control_lock, self._stream_condition, self._geometry_state_guard():
            if self._initial_preflight_is_proven_old_gap(msg):
                return
            return super().object_cb(msg)

    def _invalidate_geometry(self, failure_code, failure_reason, stamp=None, **kwargs):
        if failure_code != CODE or not self._initial_preflight_identity_scope():
            return super()._invalidate_geometry(failure_code, failure_reason, stamp=stamp, **kwargs)
        record = self._initial_preflight_remote
        reference = self._unknown_identity_pending or self._unknown_identity_reference
        source_ns = stamp.to_nsec()
        original = self._observe_return_original
        self._geometry_invalidation_generation = int(getattr(self, '_geometry_invalidation_generation', 0))+1
        self._last_geometry_invalidation_code = CODE
        self._clear_geometry_cache()
        geometry = original.geometry_estimate_to_message(
            self._invalid_geometry_estimate(CODE, failure_reason, ''), stamp=stamp,
            target_identity=self._current_stream_target_identity())
        geometry.failure_reason = CODE + ': ' + json.dumps(dict(initial_preflight=dict(
            token_id=record['token']['token_id'], reference_source_ns=reference['source_ns'],
            pending_source_ns=source_ns)), sort_keys=True, separators=(',', ':'))
        self.latest_object_geometry = deepcopy(geometry)
        self.previous_object_axes_base = None
        self._latest_preview_gate_audit_report = None
        self._publish_invalidation_safely(self.geometry_pub, geometry, 'object geometry')
        preview = original.Grasp6DPlan()
        preview.header = deepcopy(geometry.header)
        preview.valid = False
        preview.candidate_source = 'initial_preflight_pending'
        preview.diagnostic = geometry.failure_reason
        legacy = original.PoseArray(); legacy.header = deepcopy(preview.header)
        # Historical FAR_FIELD authority is held by the task's no-action gate;
        # no invalid executable-plan pair, fresh plan or motion is published.
        original.RemoteGrasp6DNode._publish_preview_plan(self, preview, legacy)
        return geometry

    def _station_compute_accepted_cb(self, msg=None):
        super()._station_compute_accepted_cb(msg)
        record = getattr(self, '_initial_preflight_remote', None)
        if record is None or record['state'] != 'armed':
            return
        with self._stream_condition, self._geometry_state_guard():
            try:
                self._initial_preflight_context(allow_handoff=True)
            except (ValueError, KeyError, TypeError, AttributeError, OverflowError) as exc:
                self._unknown_identity_retire(str(exc))

    def _measurement_remote_epoch_cb(self, msg):
        super()._measurement_remote_epoch_cb(msg)
        record = getattr(self, '_initial_preflight_remote', None)
        if record is not None and record['state'] == 'armed':
            with self._stream_condition, self._geometry_state_guard():
                try:
                    self._initial_preflight_context(allow_handoff=True)
                except (ValueError, KeyError, TypeError, AttributeError, OverflowError) as exc:
                    self._unknown_identity_retire(str(exc))

    def _unknown_identity_retire(self, reason):
        record = getattr(self, '_initial_preflight_remote', None)
        if record is not None and record['state'] == 'armed':
            record['state'] = 'retired'
            self._initial_preflight_retired_tokens.add(record['token']['token_id'])
        return super()._unknown_identity_retire(reason)

    def _initial_preflight_manual_cb(self, msg):
        if msg.data is not True:
            return
        with self._mode_control_lock, self._stream_condition, self._geometry_state_guard():
            record = self._initial_preflight_remote
            if record is not None and record['state'] == 'armed':
                self._unknown_identity_retire('initial_preflight_manual_takeover')
