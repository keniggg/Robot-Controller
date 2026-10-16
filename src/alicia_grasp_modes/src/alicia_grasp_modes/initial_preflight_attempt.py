"""Read-only attempt receipts: an old execution authority is never revived."""
from copy import deepcopy
import json
import math
import threading
import uuid

import rospy
from std_msgs.msg import Bool, String

TOPIC = '/grasp_mode/initial_preflight_attempt'
POLICY = 'initial_observation_preflight_attempt'


class InitialPreflightAttemptMixin:
    def _initialize_initial_preflight(self):
        self._initial_preflight_lock = threading.RLock()
        self._initial_preflight_attempt = None

    def _setup_initial_preflight_io(self):
        self._initial_preflight_pub = rospy.Publisher(TOPIC, String, queue_size=8, latch=True)
        self._initial_preflight_manual_sub = rospy.Subscriber(
            '/gui/joint_direct_mode', Bool, self._initial_preflight_manual, queue_size=16)

    @staticmethod
    def _initial_preflight_q(msg):
        values = dict(zip(msg.name, msg.position))
        q = [float(values['Joint%d' % i]) for i in range(1, 7)]
        if msg.header.frame_id != 'sdk_measured' or not all(math.isfinite(v) for v in q):
            raise ValueError('measured joint state required')
        return q

    def _initial_preflight_begin(self, req):
        selection=self._mode_bound_selection or {}
        if selection.get('mode')!='unknown' or selection.get('strategy')!='observe_return':
            self._initial_preflight_attempt=None
            return
        now = rospy.Time.now().to_nsec()
        record = dict(version=1, policy=POLICY, attempt_id=uuid.uuid4().hex,
            selection=deepcopy(self._mode_bound_selection), requested_plan_id=str(getattr(req, 'plan_id', '')),
            started_ns=now, ended_ns=None, released=False, plan_phase='', model_choice='',
            plan_track_id='', plan_source_ns=0, plan_digest='', freeze_count=0,
            entry_control_epoch_ns=None, entry_q_rad=None,
            physical_action_submitted=False, submissions=[], disqualifications=[],
            geometry_invalidations=[], plan_invalidations=[], missing_objects=[],
            final_revoked=False, final_event='', final_result_success=None,
            final_result_message='')
        try:
            with self._following_evidence_lock:
                msg = deepcopy(self._following_accepted_history[-1])
                record['entry_control_epoch_ns'] = self._following_epoch_ns
            record['entry_q_rad'] = self._initial_preflight_q(msg)
            if (type(record['entry_control_epoch_ns']) is not int
                    or record['entry_control_epoch_ns'] <= 0
                    or not 0 <= now-msg.header.stamp.to_nsec() <= 200000000):
                raise ValueError('fresh measured start/epoch required')
            if rospy.get_param('/gui/joint_direct_mode', False) is not False:
                raise ValueError('manual control at start')
        except Exception:
            record['disqualifications'].append('unqualified_start_feedback')
        with self._initial_preflight_lock:
            previous = self._initial_preflight_attempt
            if previous is not None and previous['ended_ns'] is None:
                raise RuntimeError('initial preflight attempt already active')
            self._initial_preflight_attempt = record

    def _initial_preflight_reject(self, reason):
        with self._initial_preflight_lock:
            record = self._initial_preflight_attempt
            if record is not None and record['ended_ns'] is None:
                if reason not in record['disqualifications']:
                    record['disqualifications'].append(reason)

    def _initial_preflight_manual(self, msg):
        if msg.data is True:
            self._initial_preflight_reject('manual_ownership_seen')

    def _initial_preflight_feedback(self, msg):
        with self._initial_preflight_lock:
            record = self._initial_preflight_attempt
            if record is None or record['ended_ns'] is not None:
                return
            try:
                q = self._initial_preflight_q(msg)
                if (record['entry_q_rad'] is None or any(abs(a-b) > .004
                        for a,b in zip(q, record['entry_q_rad']))):
                    self._initial_preflight_reject('measured_pose_changed')
            except Exception:
                self._initial_preflight_reject('measured_pose_invalid')

    def _initial_preflight_epoch(self):
        with self._initial_preflight_lock:
            record = self._initial_preflight_attempt
            epoch = getattr(self, '_following_epoch_ns', None)
            if (record is not None and record['ended_ns'] is None
                    and (type(epoch) is not int or epoch <= 0
                         or epoch != record['entry_control_epoch_ns'])):
                self._initial_preflight_reject('control_epoch_changed_or_missing')

    def _freeze_execution_plan(self, plan, *args, **kwargs):
        with self._initial_preflight_lock:
            record = self._initial_preflight_attempt
            if record is not None and record['ended_ns'] is None:
                record['freeze_count'] += 1
                if record['freeze_count'] != 1:
                    self._initial_preflight_reject('plan_rebound_within_attempt')
                else:
                    record.update(plan_phase=str(plan.diagnostic), model_choice=str(plan.model_choice),
                        plan_track_id=str(plan.target_track_id), plan_source_ns=plan.header.stamp.to_nsec(),
                        plan_digest=self._execution_plan_digest(plan))
                    if (plan.plan_id != record['requested_plan_id']
                            or plan.diagnostic != 'FAR_FIELD_OBSERVATION_PLAN'
                            or plan.model_choice != 'unknown_tabletop'):
                        self._initial_preflight_reject('not_initial_unknown_observation')
        return super()._freeze_execution_plan(plan, *args, **kwargs)

    def _initial_preflight_check_integrity(self):
        record = self._initial_preflight_attempt
        frozen = getattr(self, '_bound_execution_plan', None)
        if record is None or record['ended_ns'] is not None or frozen is None:
            return
        try:
            if (self._execution_plan_digest(frozen) != record['plan_digest']
                    or self._bound_execution_plan_digest != record['plan_digest']
                    or self._bound_execution_plan_id != record['requested_plan_id']):
                self._initial_preflight_reject('frozen_integrity_changed')
        except Exception:
            self._initial_preflight_reject('frozen_integrity_unreadable')

    def _clear_bound_execution_plan(self):
        # Base finally deliberately clears revoked. Preserve the independent
        # old-attempt receipt before that cleanup; never write authority here.
        lock = getattr(self, '_initial_preflight_lock', None)
        if lock is not None:
            with lock:
                record = self._initial_preflight_attempt
                if record is not None and record['ended_ns'] is None:
                    self._initial_preflight_check_integrity()
                    record['final_revoked'] |= bool(getattr(self, '_execution_authority_revoked', False))
                    record['final_event'] = str(getattr(self, '_last_execution_plan_event', ''))
        return super()._clear_bound_execution_plan()

    def _invoke_plan_bound_action(self, plan, gcfg, stage_label, action):
        def submitted():
            # This wrapper is reached only after the existing locked action
            # validation succeeds. Mark before calling even a failing service.
            with self._initial_preflight_lock:
                record = self._initial_preflight_attempt
                if record is not None and record['ended_ns'] is None:
                    record['physical_action_submitted'] = True
                    if len(record['submissions']) < 16:
                        record['submissions'].append(dict(stage=str(stage_label),
                            stamp_ns=rospy.Time.now().to_nsec()))
            return action()
        return super()._invoke_plan_bound_action(plan, gcfg, stage_label, submitted)

    def target_geometry_cb(self, msg):
        with self._initial_preflight_lock:
            record = self._initial_preflight_attempt
            if record is not None and record['ended_ns'] is None and record['freeze_count']:
                ns = msg.header.stamp.to_nsec()
                if ns >= record['plan_source_ns']:
                    if not msg.valid:
                        if len(record['geometry_invalidations']) >= 32:
                            self._initial_preflight_reject('geometry_event_overflow')
                        else:
                            record['geometry_invalidations'].append(dict(source_ns=ns,
                                track_id=str(msg.target_track_id), reason=str(msg.failure_reason)))
                    elif msg.target_track_id != record['plan_track_id']:
                        self._initial_preflight_reject('valid_geometry_track_changed')
        with self._grasp6d_plan_guard():
            revoked_before = bool(getattr(self, '_execution_authority_revoked', False))
            result = super().target_geometry_cb(msg)
            if (msg.valid and not revoked_before
                    and bool(getattr(self, '_execution_authority_revoked', False))):
                self._initial_preflight_reject('valid_geometry_rejected_by_original_gate')
            return result

    def obj_cb(self, msg):
        with self._initial_preflight_lock:
            record = self._initial_preflight_attempt
            if record is not None and record['ended_ns'] is None and not msg.detected:
                if len(record['missing_objects']) >= 32:
                    self._initial_preflight_reject('object_event_overflow')
                else:
                    record['missing_objects'].append(msg.header.stamp.to_nsec())
        return super().obj_cb(msg)

    def grasp6d_plan_cb(self, msg):
        with self._initial_preflight_lock:
            record = self._initial_preflight_attempt
            if record is not None and record['ended_ns'] is None and record['freeze_count']:
                if msg.header.stamp.to_nsec() >= record['plan_source_ns']:
                    if not msg.valid:
                        if len(record['plan_invalidations']) >= 32:
                            self._initial_preflight_reject('plan_event_overflow')
                        else:
                            record['plan_invalidations'].append(dict(
                                source_ns=msg.header.stamp.to_nsec(), reason=str(msg.diagnostic)))
                    elif msg.target_track_id != record['plan_track_id']:
                        self._initial_preflight_reject('valid_plan_track_changed')
        return super().grasp6d_plan_cb(msg)

    def _validate_bound_plan_locked(self, plan, gcfg):
        result = super()._validate_bound_plan_locked(plan, gcfg)
        if not result.ok and result.code not in ('EXECUTION_AUTHORITY_REVOKED', 'TARGET_LOST'):
            self._initial_preflight_reject('bound_validation:' + result.code)
        return result

    def stop_cb(self, req):
        self._initial_preflight_reject('stop_requested')
        return super().stop_cb(req)

    def _initial_preflight_finish(self, result):
        with self._initial_preflight_lock:
            record = self._initial_preflight_attempt
            if record is None or record['ended_ns'] is not None:
                return
            self._initial_preflight_epoch()
            if rospy.get_param('/gui/joint_direct_mode', False) is not False:
                self._initial_preflight_reject('manual_ownership_at_finish')
            if record['selection'] != self._mode_bound_selection:
                self._initial_preflight_reject('selection_changed')
            if (self._observe_return_session or {}).get('view_index', 1) != 1:
                self._initial_preflight_reject('supplemental_view_attempt')
            record.update(ended_ns=rospy.Time.now().to_nsec(),
                released=not bool(getattr(self, 'active', False) or getattr(self, '_start_inflight', False)),
                final_result_success=getattr(result, 'success', None),
                final_result_message=str(getattr(result, 'message', 'start_exception_or_unknown_result')))
            if result is None:
                record['disqualifications'].append('unknown_result')
            receipt = deepcopy(record)
        publisher = getattr(self, '_initial_preflight_pub', None)
        if publisher is not None:
            publisher.publish(String(data=json.dumps(receipt, sort_keys=True)))
        return receipt
