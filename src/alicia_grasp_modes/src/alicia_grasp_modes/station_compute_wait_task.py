"""Task-side identity wait; cached geometry never acquires execution authority."""
from copy import deepcopy
import hashlib
import json
import time

import numpy as np
import rospy
from sensor_msgs.msg import Image

from .station_compute_reacquisition import (
    validate_contract, validate_registration, validate_recovery_parent, validate_station_support_gap)
from .unknown_pending_identity import reference_binding
from .rgb_observation_source import bracket_feedback, digest
from .feedback_bracket_gap import validate_station_feedback_gap
from .observe_return_measurement import (STATION_REQUALIFICATION, station_requalification_key,
    station_requalification_witness, validate_station_requalification, parse_measurement_request)
from alicia_flexible_grasp.robot.observation_view_gateway import _camera_configuration_contract
from alicia_flexible_grasp.grasp.grasp_state_machine import GraspStages


PENDING = 'TARGET_PROCESSING_PENDING'


def _ns(value):
    return int(value.to_nsec())


class StationComputeWaitTaskMixin:
    def _reset_station_compute_task(self):
        self._station_requalification_task = None
        self._station_requalification_used = set()
        self._station_requalification_tombstones = {}
        self._station_task_record = None
        self._station_task_objects = {}
        self._station_task_masks = {}
        self._station_task_statuses = {}
        self._station_task_proven_contracts = {}
        self._station_task_retired = False
        self._station_task_cutoff = 0
        self._station_task_disarmed = None
        self._station_task_gap_tombstones = {}
        self._station_task_gap_history = {}

    def _station_task_gap_late_status(self, status):
        ns = status.get('source_stamp_ns')
        history = self._station_task_gap_history.get(ns)
        if history is not None and status.get('state') == 'ready':
            if (status.get('station_compute_wait') != history['contract']
                    or status.get('station_compute_mask') != history['mask']):
                self._station_task_retire('historical station READY changed')
                return True
        tombstone = self._station_task_gap_tombstones.get(ns)
        if tombstone is None or self._station_task_record is None:
            return False
        if status.get('target_lost') is True:
            return False
        try:
            if (status.get('state') != 'processing_deadline_pending'
                    or status.get('detail') != 'station_accepted_feedback_bracket_pending_revalidation'
                    or status.get('target_lost') is not False or status.get('first_loss') is not None
                    or status.get('station_compute_wait') != tombstone['contract']
                    or reference_binding(status, self._mode_bound_selection)
                        != tombstone['contract']['binding']['reference_binding']
                    or self._station_task_record['contract']['binding'] != tombstone['contract']['binding']):
                raise ValueError('late station gap receipt changed')
            stamps = validate_station_feedback_gap(status.get('station_feedback_gap'),
                contract=tombstone['contract'], now_ns=rospy.Time.now().to_nsec(),
                recovery_source_ns=tombstone['recovery_source_ns'])
            if ns not in stamps or any(self._station_task_gap_tombstones.get(v, {}).get('contract')
                                      != tombstone['contract'] for v in stamps):
                raise ValueError('late station gap source unproven')
            return True
        except (ValueError, KeyError, TypeError, AttributeError):
            self._station_task_retire('late station gap receipt changed')
            return True

    def _setup_station_compute_task_io(self):
        topic = rospy.get_param('/perception', {}).get('output_mask_topic', '/perception/object_mask')
        self._station_task_mask_sub = rospy.Subscriber(topic, Image,
            self._station_compute_task_mask_cb, queue_size=16)
        self._station_task_timer = rospy.Timer(rospy.Duration(.05), self._station_compute_task_tick)

    @staticmethod
    def _station_task_cache(cache, ns, value):
        cache[ns] = deepcopy(value)
        while len(cache) > 16:
            del cache[min(cache)]

    def _station_task_owner_key(self):
        session = getattr(self, '_observe_return_session', None) or {}
        lease = getattr(self, '_observation_measurement_request', None) or {}
        return (session.get('session_id'), session.get('view_plan_id'),
                lease.get('lease_id'), getattr(self, '_near_field_phase_id', None))

    def _station_task_receiving(self):
        if (getattr(self, '_station_task_record', None) is not None
                or getattr(self, '_station_requalification_task', None) is not None):
            return True
        # Subscriber callbacks must not acquire the action lock during an
        # actual motion, which holds that lock across its synchronous call.
        selection = getattr(self, '_mode_bound_selection', None) or {}
        plan = getattr(self, '_bound_execution_plan', None)
        return (not getattr(self, '_station_task_retired', False)
            and getattr(self, 'active', False)
            and getattr(self, 'stage', None) == GraspStages.PLAN_PREGRASP
            and selection.get('mode') == 'unknown'
            and selection.get('strategy') == 'observe_return'
            and getattr(plan, 'diagnostic', '') == 'FAR_FIELD_OBSERVATION_PLAN'
            and self._station_task_owner_key() != self._station_task_disarmed)

    def _station_task_disarm(self):
        self._station_task_disarmed = self._station_task_owner_key()
        self._station_task_record = None
        self._station_task_objects.clear()
        self._station_task_masks.clear()
        self._station_task_statuses.clear()
        self._station_task_proven_contracts.clear()
        self._station_task_gap_tombstones.clear()
        self._station_task_gap_history.clear()

    def _station_task_context(self, contract, *, admission=False, source_ns=None, pose_only=False):
        b = contract['binding']
        selection = self._mode_bound_selection
        session = getattr(self, '_observe_return_session', None) or {}
        lease = getattr(self, '_observation_measurement_request', None) or {}
        phase = getattr(self, '_published_near_field_phase', None)
        plan = getattr(self, '_bound_execution_plan', None)
        if (self._station_task_retired or not self._selection_current(selection)
                or selection != b['selection'] or not getattr(self, 'active', False)
                or getattr(self, '_execution_authority_revoked', False)
                or rospy.get_param('/gui/joint_direct_mode', False) is not False
                or session.get('version') != 2 or session.get('phase') != 'observing'
                or lease.get('state') != 'qualified' or lease.get('motion_kind') != 'observation'
                or lease.get('lease_id') != b['lease_id']
                or any(session.get(k) != b[k] or lease.get(k) != b[k] for k in (
                    'selection', 'session_id', 'view_plan_id', 'view_index', 'target_epoch', 'target_track_id'))
                or int(session['deadline_ns']) != contract['session_deadline_ns']
                or plan is None or plan.diagnostic != 'FAR_FIELD_OBSERVATION_PLAN'
                or plan.model_choice != 'unknown_tabletop' or plan.plan_id != b['view_plan_id']
                or plan.target_track_id != b['target_track_id']
                or phase is None or phase.active is not True
                or not getattr(self, '_near_field_active', False)
                or phase.planning_stage != 'contact_replan' or phase.phase_id != b['phase_id']
                or self._near_field_phase_id != b['phase_id']
                or _ns(phase.header.stamp) != b['phase_started_ns']
                or _ns(phase.reference_source_stamp) != b['phase_reference_source_ns']
                or phase.reference_target_track_id != b['target_track_id']
                or phase.reference_center_valid is not True
                or _ns(phase.deadline) != contract['phase_deadline_ns']
                or self._near_field_phase_deadline_sec != phase.deadline.to_sec()):
            raise ValueError('station_task_owner_changed')
        # PLAN_PREGRASP is an idle planning wait even while the task's active
        # flag remains true. A frozen CONTACT plan is never in this scope.
        if admission and self.stage != GraspStages.PLAN_PREGRASP:
            raise ValueError('station_task_not_planning')
        if digest(_camera_configuration_contract(rospy.get_param('/camera'))) != b['camera_configuration_sha256']:
            raise ValueError('station_task_camera_changed')
        with self._following_evidence_lock:
            epoch = self._following_epoch_ns
            messages = tuple(self._following_accepted_history)
        if epoch != b['control_epoch_ns'] or lease.get('control_epoch_ns') != epoch:
            raise ValueError('station_task_control_epoch_changed')
        rows = []
        for msg in messages:
            if msg.header.frame_id != 'sdk_measured':
                continue
            names = [name.lower() for name in msg.name]
            if len(names) != len(set(names)) or len(names) != len(msg.position):
                raise ValueError('station_task_invalid_feedback')
            positions = dict(zip(names, msg.position))
            q = np.asarray([positions['joint%d' % i] for i in range(1, 7)], dtype=float)
            if not np.isfinite(q).all():
                raise ValueError('station_task_invalid_feedback')
            rows.append((_ns(msg.header.stamp), q))
        current = rospy.Time.now().to_nsec()
        if not rows or not 0 <= current-max(ns for ns, _ in rows) < 800_000_000:
            raise ValueError('station_task_feedback_stale')
        q_station = np.asarray(b['station_q_rad'])
        since = int(lease['source_stamps_ns'][-1])
        measured = [q for ns, q in rows if ns >= since]
        if not measured or (not pose_only and any(np.max(np.abs(q-q_station)) > .004 for q in measured)):
            raise ValueError('station_task_pose_changed')
        if source_ns is not None:
            q, before, after = bracket_feedback(rows, source_ns, epoch)
            if any(np.max(np.abs(np.asarray(v)-q_station)) > .004 for v in (q, before[1], after[1])):
                raise ValueError('station_task_source_pose_changed')

    def _station_requalification_rows(self):
        with self._following_evidence_lock:
            messages = tuple(self._following_accepted_history)
        rows = []
        for message in messages:
            if message.header.frame_id != 'sdk_measured':
                continue
            names = [str(n).lower() for n in message.name]
            if len(set(names)) != len(names) or len(names) != len(message.position):
                raise ValueError('STATION_REQUALIFICATION_FEEDBACK_INVALID')
            positions = dict(zip(names, message.position))
            q = [float(positions['joint%d' % i]) for i in range(1, 7)]
            if not np.isfinite(q).all():
                raise ValueError('STATION_REQUALIFICATION_FEEDBACK_INVALID')
            rows.append((_ns(message.header.stamp), q))
        return rows

    def _station_requalification_begin(self):
        record = self._station_task_record
        if record is None or self._station_requalification_task is not None:
            return False
        try:
            contract = record['contract']
            self._station_task_context(contract, admission=True, pose_only=True)
            if self._unknown_target_loss is not None:
                return False
            key = station_requalification_key(contract)
            if key in self._station_requalification_used:
                return False
            previous = deepcopy(self._observation_measurement_request)
            # A synthetic partial lease cannot enroll into this exception.
            if any(k not in previous for k in ('request_id','origin_plan_id','origin_snapshot_stamp_ns',
                    'view_source_stamp_ns','plan_sha256','stamp_ns','capture_deadline_ns')):
                return False
            now = rospy.Time.now().to_nsec()
            rows = self._station_requalification_rows()
            changed = [(ns,q) for ns,q in rows if ns > contract['source_ns']
                and max(abs(a-b) for a,b in zip(q,contract['binding']['station_q_rad'])) > .004]
            if not changed:
                return False
            ns,q = changed[0]
            witness = station_requalification_witness(contract, ns, q, now)
            if time.monotonic() >= record['identity_mono']:
                return False
            self._station_requalification_used.add(key)
            self._station_requalification_task = dict(witness=witness, previous=previous,
                monotonic_deadline=record['identity_mono'], started=False, completed=False,
                local_feedback_proof=(ns, tuple(q)), last_now_ns=now, finalizing_phase=None,
                contracts=deepcopy(self._station_task_proven_contracts))
            self._station_task_quarantine()
            self._station_requalification_tombstones.update(deepcopy(self._station_task_proven_contracts))
            self._station_task_record = None
            self._station_task_objects.clear(); self._station_task_masks.clear(); self._station_task_statuses.clear()
            self._station_task_cutoff = max(self._station_task_cutoff, ns)
            return True
        except (ValueError, KeyError, TypeError, AttributeError, OverflowError):
            return False

    def _station_requalification_live(self):
        hold = self._station_requalification_task
        if hold is None:
            raise ValueError('STATION_REQUALIFICATION_NOT_PENDING')
        value = hold['witness']; parent = value['parent_contract']; b = parent['binding']
        now = rospy.Time.now().to_nsec()
        # This sample was read from our accepted SDK history at enrollment.
        # Ring-buffer eviction is not revocation of that already local proof.
        validate_station_requalification(value, parent=hold['contracts'].get(parent['source_ns']),
            feedback=[hold['local_feedback_proof']], now_ns=now)
        rows = self._station_requalification_rows()
        if (not rows or not 0 <= now-max(ns for ns, _ in rows) < 800_000_000
                or now < hold['last_now_ns']):
            raise ValueError('STATION_REQUALIFICATION_FEEDBACK_STALE')
        session = self._observe_return_session or {}; phase = self._published_near_field_phase
        plan = self._bound_execution_plan
        finalizing = hold['finalizing_phase']
        expected_phase_id = b['phase_id'] if finalizing is None else finalizing.phase_id
        expected_start = b['phase_started_ns'] if finalizing is None else _ns(finalizing.header.stamp)
        expected_source = (b['phase_reference_source_ns'] if finalizing is None
                           else _ns(finalizing.reference_source_stamp))
        if (not self.active or self.stage != GraspStages.PLAN_PREGRASP
                or not self._selection_current(b['selection']) or self._mode_bound_selection != b['selection']
                or self._unknown_target_loss is not None or self._execution_authority_revoked
                or rospy.get_param('/gui/joint_direct_mode', False) is not False
                or self._following_epoch_ns != b['control_epoch_ns']
                or session.get('phase') != 'observing'
                or any(session.get(k) != b[k] for k in ('session_id','view_plan_id','view_index','target_epoch','target_track_id'))
                or int(session['deadline_ns']) != parent['session_deadline_ns']
                or plan.diagnostic != 'FAR_FIELD_OBSERVATION_PLAN' or plan.plan_id != b['view_plan_id']
                or not phase.active or phase.phase_id != expected_phase_id
                or self._near_field_phase_id != expected_phase_id
                or _ns(phase.header.stamp) != expected_start
                or _ns(phase.reference_source_stamp) != expected_source
                or phase.reference_target_track_id != b['target_track_id']
                or not phase.reference_center_valid or phase.planning_stage != 'contact_replan'
                or _ns(phase.deadline) != parent['phase_deadline_ns']
                or digest(_camera_configuration_contract(rospy.get_param('/camera'))) != b['camera_configuration_sha256']
                or time.monotonic() >= hold['monotonic_deadline']):
            raise ValueError('STATION_REQUALIFICATION_OWNER_CHANGED')
        if finalizing is not None:
            request = self._observation_measurement_request or {}
            if (request.get('state') != 'qualified'
                    or request.get(STATION_REQUALIFICATION) != value
                    or request.get('lease_id') != hold['new_lease_id']
                    or expected_phase_id != b['phase_id']+1
                    or expected_source < request['source_stamps_ns'][-1]):
                raise ValueError('STATION_REQUALIFICATION_FINALIZATION_CHANGED')
        hold['last_now_ns'] = now
        return hold

    def _observation_measurement_live(self, request):
        result = super()._observation_measurement_live(request)
        if request.get(STATION_REQUALIFICATION) is not None:
            self._station_requalification_live()
            if request[STATION_REQUALIFICATION] != self._station_requalification_task['witness']:
                raise ValueError('STATION_REQUALIFICATION_BINDING_CHANGED')
        return result

    def _station_requalification_settle(self):
        from alicia_flexible_grasp.robot.stationary_following import accepted_stationary_window
        hold = self._station_requalification_live()
        after = max(rospy.Time.now().to_nsec(),
            int(self._observation_measurement_request['stamp_ns'])+1)
        timeout = float(rospy.get_param('/grasp/motion_settle_timeout_sec', 2.))
        duration = float(rospy.get_param('/grasp/motion_settle_min_sec', .8))
        period = max(.02, float(rospy.get_param('/grasp/motion_settle_sample_sec', .05)))
        epsilon = float(rospy.get_param('/grasp/motion_settle_position_epsilon_rad', .0025))
        required = max(2, int(rospy.get_param('/grasp/motion_settle_required_samples', 3)))
        if not all(np.isfinite(v) and v > 0 for v in (timeout, duration, period, epsilon)):
            raise ValueError('STATION_REQUALIFICATION_SETTLE_CONFIG_INVALID')
        cutoff = min(time.monotonic()+timeout, hold['monotonic_deadline'])
        self._last_motion_settle_sample = None
        while time.monotonic() < cutoff and not rospy.is_shutdown():
            self._station_requalification_live()
            with self._following_evidence_lock:
                messages = tuple(self._following_accepted_history)
            try:
                report = accepted_stationary_window(messages, now_sec=rospy.Time.now().to_sec(),
                    epoch_ns=hold['witness']['parent_contract']['binding']['control_epoch_ns'], after_ns=after,
                    duration_sec=max(duration, period*(required-1)), position_epsilon_rad=epsilon,
                    minimum_samples=required)
            except (ValueError, TypeError, OverflowError):
                remaining = min(cutoff-time.monotonic(),
                    (hold['witness']['deadline_ns']-rospy.Time.now().to_nsec())*1e-9)
                if remaining > 0:
                    rospy.sleep(min(period, remaining))
                continue
            self._station_requalification_live()
            self._last_motion_settle_sample = report
            return
        raise ValueError('STATION_REQUALIFICATION_SETTLE_TIMEOUT')

    def _station_requalification_run(self, current_plan, gcfg):
        """Main-thread, measurement-only recovery; never called under plan_guard."""
        hold = self._station_requalification_live()
        if hold['started']:
            return
        hold['started'] = True
        previous = hold['previous']; now = rospy.Time.now().to_nsec()
        lease = previous['session_id']+':restation:'+str(now)
        request = dict(previous, state='executing', lease_id=lease, request_id=lease+':executing',
            stamp_ns=now, arrival_stamp_ns=0, capture_deadline_ns=0,
            **{STATION_REQUALIFICATION:deepcopy(hold['witness'])})
        request.pop('source_stamps_ns', None); request.pop('settle', None)
        request = parse_measurement_request(request, self._mode_bound_selection, now)
        self._observation_measurement_publish(request)
        self._station_requalification_settle()
        self._observation_measurement_finish(current_plan, gcfg)
        # Publish metadata under the short commit lock; all blocking ACK,
        # stationary and frame waits above ran without this lock. Keep the
        # original recovery clock until a real new three-topic registration.
        with self._grasp6d_plan_guard():
            self._station_requalification_live()
            deadline = hold['witness']['parent_contract']['phase_deadline_ns']
            hold['new_lease_id'] = self._observation_measurement_request['lease_id']
            self._station_task_disarm()
            self._set_near_field_active(True, force=True,
                # The exact old absolute cap wins; avoid a floating point
                # subtraction rounding the relative budget below that cap.
                budget_sec=(deadline-rospy.Time.now().to_nsec())*1e-9+1e-6,
                reference_plan=current_plan, absolute_deadline_ns=deadline)
            hold['finalizing_phase'] = deepcopy(self._published_near_field_phase)
            self._station_task_disarmed = None
            self._station_requalification_live()

    def _station_task_support_gap_live(self, record, now):
        hold = record.get('support_gap')
        if hold is not None:
            validate_station_support_gap(hold['receipt'], hold['parent'], now)
            if time.monotonic() >= hold['monotonic_deadline']:
                raise ValueError('station_support_gap_monotonic_expired')

    def _station_task_support_gap_receipt(self, status, recovery=False):
        record = self._station_task_record
        receipt = status.get('station_support_gap_recovery' if recovery else 'station_support_gap')
        hold = None if record is None else record.get('support_gap')
        if receipt is None:
            if (recovery and hold is not None or not recovery
                    and status.get('detail') == 'support_measurement_pending_revalidation'):
                raise ValueError('station_support_gap_receipt_missing')
            return
        if record is None:
            raise ValueError('station_support_gap_requires_registered_parent')
        now = rospy.Time.now().to_nsec()
        parent = status.get('station_compute_recovery' if recovery else 'station_compute_wait')
        ns = status.get('source_stamp_ns')
        if (status.get('target_lost') is not False or status.get('first_loss') is not None
                or (not recovery and (status.get('state') != 'processing_deadline_pending'
                    or status.get('detail') != 'support_measurement_pending_revalidation'
                    or receipt.get('source_ns') != ns))):
            raise ValueError('station_support_gap_status_changed')
        validate_recovery_parent(record['contract'], parent=parent,
            chain=status.get('station_compute_recovery_chain'),
            binding=record['contract']['binding'], now_ns=now,
            source_ns=ns if recovery else None)
        checked = validate_station_support_gap(receipt, parent, now,
            source_ns=ns if recovery else None)
        if hold is not None:
            self._station_task_support_gap_live(record, now)
            if (parent != hold['parent'] or any(checked[k] != hold['receipt'][k]
                    for k in ('first_source_ns', 'deadline_ns', 'parent_source_ns', 'source_id'))
                    or recovery and checked['source_ns'] < hold['receipt']['source_ns']):
                raise ValueError('station_support_gap_parent_or_deadline_changed')
            # Repeated/out-of-order notifications never reset either clock.
            if checked['source_ns'] > hold['receipt']['source_ns']:
                hold['receipt'] = checked
        else:
            record['support_gap'] = dict(receipt=checked, parent=deepcopy(parent),
                monotonic_deadline=min(record['identity_mono'],
                    time.monotonic()+(checked['deadline_ns']-now)/1e9))
        self._station_task_quarantine()

    def _station_task_retire(self, reason):
        self._station_task_retired = True
        self._station_task_record = None
        selected = deepcopy(self._mode_bound_selection)
        with self._unknown_support_lock:
            if self._unknown_target_loss is None:
                self._unknown_target_loss = dict(selection=selected, reason='TARGET_LOST: '+str(reason))
        self._clear_grasp6d_authority()

    def _station_task_quarantine(self):
        record = self._station_task_record
        if record is None:
            return
        record['pending'] = True
        self._station_task_cutoff = max(self._station_task_cutoff, record['contract']['source_ns'])
        self.latest_obj = self.latest_obj_time = None
        self.latest_visual_obj = self.latest_visual_obj_time = None
        self.latest_target_geometry = None
        preview = getattr(self, 'latest_grasp6d_preview_plan', None)
        if preview is not None and getattr(preview, 'diagnostic', '') == 'CONTACT_EXECUTION_PLAN':
            self.latest_grasp6d_preview_plan = None
        # Do not tombstone/revoke the old FAR_FIELD plan: it is still needed
        # for the session's metadata rebind, never for an old physical action.

    def _station_compute_task_geometry(self, msg):
        """Quarantine a bound pending notification without retiring identity.

        Plain reason strings cannot authorize this exception. The task must
        already own a live, three-topic-registered station, and the notification
        must identify that exact contract or a proven same-station successor.
        No incoming geometry, historical plan, or renewed deadline is admitted.
        """
        if (getattr(msg, 'valid', None) is False
                and str(getattr(msg, 'failure_reason', '')).partition(':')[0] == PENDING):
            try:
                value = json.loads(msg.failure_reason.partition(':')[2])
                parent = value['station_compute_wait']; ns = parent['source_ns']
                known = self._station_requalification_tombstones.get(ns)
                if (known == parent and msg.target_track_id == parent['binding']['target_track_id']
                        and self._mode_bound_selection == parent['binding']['selection']
                        and parent['binding']['phase_started_ns'] <= _ns(msg.header.stamp) <= rospy.Time.now().to_nsec()):
                    return True  # Bound old non-executable invalidation only.
            except (ValueError, KeyError, TypeError, AttributeError):
                pass
        if (getattr(msg, 'valid', None) is not False
                or str(getattr(msg, 'failure_reason', '')).partition(':')[0] != PENDING
                or getattr(self, '_station_task_record', None) is None):
            return False
        with self._grasp6d_plan_guard():
            self._station_task_check()
            record = self._station_task_record
            if record is None:
                return False
            try:
                contract = record['contract']
                self._station_task_context(contract, admission=True)
                now = rospy.Time.now().to_nsec()
                ns = _ns(msg.header.stamp)
                if (not contract['binding']['phase_started_ns'] <= ns <= now
                        or msg.header.frame_id != self._published_near_field_phase.header.frame_id
                        or msg.target_track_id != contract['binding']['target_track_id']):
                    return False
                payload = json.loads(msg.failure_reason.partition(':')[2])
                if (not isinstance(payload, dict) or set(payload) != {
                        'station_compute_wait', 'station_compute_recovery_chain'}):
                    return False
                offered = payload['station_compute_wait']
                if offered['source_ns'] < contract['source_ns']:
                    # A recovered source supersedes only a contract that this
                    # task registered or validated as a recovery ancestor; do
                    # not infer ownership from a
                    # matching reason, binding, or arbitrary historical stamp.
                    return (offered == self._station_task_proven_contracts.get(offered['source_ns'])
                        and offered['binding'] == contract['binding'])
                if ns < offered['source_ns']:
                    return False
                validate_recovery_parent(contract, parent=offered,
                    chain=payload['station_compute_recovery_chain'],
                    binding=contract['binding'], now_ns=now)
                self._station_task_quarantine()
                return True
            except (ValueError, KeyError, TypeError, AttributeError, OverflowError):
                return False

    def _station_task_check(self):
        if self._station_requalification_task is not None:
            try:
                self._station_requalification_live()
                return True
            except (ValueError, KeyError, TypeError, AttributeError) as exc:
                self._station_requalification_task = None
                self._station_task_retire(str(exc))
                return False
        record = self._station_task_record
        if record is None:
            return False
        try:
            contract = record['contract']
            self._station_task_context(contract)
            state = validate_contract(contract, binding=contract['binding'], now_ns=rospy.Time.now().to_nsec())
            mono = time.monotonic()
            self._station_task_support_gap_live(record, rospy.Time.now().to_nsec())
            if mono >= record['identity_mono']:
                raise ValueError('station_task_identity_expired')
            if state == 'PENDING' or mono >= record['geometry_mono']:
                self._station_task_quarantine()
            return record['pending']
        except (ValueError, KeyError, TypeError, AttributeError, OverflowError) as exc:
            if str(exc) == 'station_task_pose_changed' and self._station_requalification_begin():
                return True
            self._station_task_retire(str(exc))
            return False

    def _station_compute_task_tick(self, _event=None):
        if not self._station_task_receiving():
            return
        with self._grasp6d_plan_guard():
            self._station_task_check()
            self._station_task_join()

    def _station_task_join(self):
        recovery = self._station_requalification_task
        if recovery is not None and recovery['finalizing_phase'] is None:
            return
        if not self._station_task_receiving():
            return
        now = rospy.Time.now().to_nsec()
        for ns in sorted(self._station_task_statuses):
            record = self._station_task_record
            if record is not None and ns <= record['contract']['source_ns']:
                continue
            status = self._station_task_statuses[ns]
            obj = self._station_task_objects.get(ns)
            mask = self._station_task_masks.get(ns)
            if status.get('state') != 'ready' or obj is None or mask is None or obj.detected is not True:
                continue
            try:
                if recovery is not None:
                    self._station_requalification_live()
                proven_ancestors = ()
                gap_sources = ()
                contract = status['station_compute_wait']
                if contract['source_ns'] != ns or reference_binding(status, self._mode_bound_selection) != contract['binding']['reference_binding']:
                    raise ValueError('station_task_source_binding_changed')
                self._station_task_context(contract, admission=True, source_ns=ns)
                registered = validate_registration(contract, binding=contract['binding'], now_ns=now)
                if obj.label != 'unknown_small' or obj.header.frame_id != self._published_near_field_phase.header.frame_id:
                    raise ValueError('station_task_object_changed')
                if not self._observed_target_matches_plan(self._bound_execution_plan, obj, rospy.get_param('/grasp', {})):
                    raise ValueError('station_task_target_changed')
                if status['station_compute_mask'] != mask or mask['frame_id'] != status['color_frame_id']:
                    raise ValueError('station_task_mask_changed')
                self._station_task_support_gap_receipt(status, recovery=True)
                gap_receipt = status.get('station_feedback_gap_recovery')
                if gap_receipt is not None:
                    if record is None:
                        raise ValueError('station gap recovery requires registered parent')
                    parent = status.get('station_compute_recovery')
                    validate_recovery_parent(record['contract'], parent=parent,
                        chain=status.get('station_compute_recovery_chain'),
                        binding=contract['binding'], now_ns=now, source_ns=ns)
                    gap_sources = validate_station_feedback_gap(gap_receipt,
                        contract=parent, now_ns=now, recovery_source_ns=ns)
                    self._station_task_quarantine()
                if record is not None:
                    self._station_task_support_gap_live(record, now)
                    old = record['contract']
                    if old['binding'] != contract['binding']:
                        raise ValueError('station_task_reference_changed')
                    if record['pending']:
                        validate_recovery_parent(old, parent=status.get('station_compute_recovery'),
                            chain=status.get('station_compute_recovery_chain'),
                            binding=old['binding'], now_ns=now, source_ns=ns)
                        if time.monotonic() >= record['identity_mono']:
                            raise ValueError('station_task_identity_expired')
                        self._station_task_cutoff = max(self._station_task_cutoff, ns-1)
                super().obj_cb(obj)
                if getattr(self, 'latest_obj', None) is None:
                    raise ValueError('station_task_original_object_gate_rejected')
                now = rospy.Time.now().to_nsec()
                registered = validate_registration(contract, binding=contract['binding'], now_ns=now)
                if record is not None and record['pending']:
                    self._station_task_support_gap_live(record, now)
                    validate_recovery_parent(record['contract'],
                        parent=status.get('station_compute_recovery'),
                        chain=status.get('station_compute_recovery_chain'),
                        binding=record['contract']['binding'], now_ns=now, source_ns=ns)
                    if time.monotonic() >= record['identity_mono']:
                        raise ValueError('station_task_identity_expired')
                    proven_ancestors = (status.get('station_compute_recovery_chain')
                        or (status['station_compute_recovery'],))
                if (record is not None and not record['pending']
                        and status.get('station_compute_recovery') is not None):
                    # Optional ancestry is evidence for delayed invalidations,
                    # never an extra condition on an ordinary fresh READY.
                    try:
                        validate_recovery_parent(record['contract'],
                            parent=status['station_compute_recovery'],
                            chain=status.get('station_compute_recovery_chain'),
                            binding=record['contract']['binding'], now_ns=now, source_ns=ns)
                        proven_ancestors = (status.get('station_compute_recovery_chain')
                            or (status['station_compute_recovery'],))
                    except (ValueError, KeyError, TypeError, AttributeError, OverflowError):
                        pass
                mono = time.monotonic()
                if gap_sources:
                    self._station_task_gap_history[record['contract']['source_ns']] = dict(
                        contract=deepcopy(record['contract']), object=deepcopy(record['object']),
                        mask=deepcopy(record['mask']))
                    for discarded in gap_sources:
                        self._station_task_gap_tombstones[discarded] = dict(
                            contract=deepcopy(status['station_compute_recovery']), recovery_source_ns=ns)
                    for cache in (self._station_task_gap_history, self._station_task_gap_tombstones):
                        while len(cache) > 64:
                            del cache[min(cache)]
                # Registration is source-clock based; repeat receipts never
                # change these monotonic deadlines or refresh old geometry.
                if recovery is not None:
                    self._station_requalification_live()
                    request = self._observation_measurement_request
                    if (self._station_requalification_task is not recovery
                            or registered['binding']['lease_id'] != recovery['new_lease_id']
                            or registered['binding']['phase_id'] != recovery['finalizing_phase'].phase_id
                            or ns <= request['source_stamps_ns'][-1]):
                        raise ValueError('STATION_REQUALIFICATION_NEW_REGISTRATION_REQUIRED')
                self._station_task_record = dict(contract=registered, pending=False,
                    object=deepcopy(obj), mask=deepcopy(mask),
                    geometry_mono=mono+(registered['geometry_deadline_ns']-now)/1e9,
                    identity_mono=mono+(registered['identity_deadline_ns']-now)/1e9)
                for ancestor in proven_ancestors:
                    self._station_task_cache(self._station_task_proven_contracts,
                        ancestor['source_ns'], ancestor)
                self._station_task_cache(self._station_task_proven_contracts, ns, registered)
                if recovery is not None:
                    recovery['completed'] = True
                    self._station_requalification_task = None
                    recovery = None
            except (ValueError, KeyError, TypeError, AttributeError, OverflowError):
                if record is not None and record.get('support_gap') is not None:
                    self._station_task_quarantine()
                # Missing/late or unordered inputs cannot manufacture a new
                # wait. Existing pending ownership expires on its fixed clock.
                continue

    def _station_compute_task_status(self, status):
        witness = status.get(STATION_REQUALIFICATION)
        if (witness is not None and status.get('state') == 'pending'
                and status.get('detail') == STATION_REQUALIFICATION
                and status.get('target_lost') is False and status.get('first_loss') is None):
            try:
                parent = witness['parent_contract']
                known = self._station_task_proven_contracts.get(parent['source_ns'])
                if known is None:
                    known = self._station_requalification_tombstones.get(parent['source_ns'])
                if known != parent:
                    return
                # Unknown TCPROS ordering may deliver this notice before our
                # SDK callback. It can quarantine geometry but cannot enroll
                # the recovery: only local measured feedback does that.
                validate_station_requalification(witness, parent=known,
                    feedback=[(witness['feedback_stamp_ns'], witness['feedback_q_rad'])],
                    now_ns=rospy.Time.now().to_nsec())
                self._station_task_check()
                if self._station_task_record is not None:
                    self._station_task_quarantine()
                return
            except (ValueError, KeyError, TypeError, AttributeError):
                return
        if not self._station_task_receiving():
            return
        if self._station_task_record is None and not isinstance(status.get('station_compute_wait'), dict):
            return
        with self._grasp6d_plan_guard():
            if self._station_task_record is None and not isinstance(status.get('station_compute_wait'), dict):
                return
            self._station_task_check()
            record = self._station_task_record
            ns = status.get('source_stamp_ns')
            if type(ns) is not int or not 0 < ns <= rospy.Time.now().to_nsec():
                return
            selected = self._mode_bound_selection or self._mode_selection or {}
            if (status.get('mode') != 'unknown' or status.get('generation') != selected.get('generation')
                    or status.get('selection_stamp_ns') != selected.get('stamp_ns')
                    or ns <= selected.get('stamp_ns', 0)):
                return
            if status.get('target_lost') is True:
                if record is not None:
                    self._station_task_retire('producer reported target lost')
                return
            if self._station_task_gap_late_status(status):
                return
            if (record is not None and ns < record['contract']['source_ns']
                    and ns <= self._station_task_cutoff):
                return
            offered = status.get('station_compute_wait')
            if (record is not None and ns == record['contract']['source_ns']
                    and isinstance(offered, dict) and offered != record['contract']):
                # Same-source metadata cannot rewrite a registered contract.
                # A bridged pending envelope may name a strictly later READY.
                if not (status.get('state') == 'processing_deadline_pending'
                        and offered.get('source_ns', 0) > ns):
                    self._station_task_retire('same-source station contract changed')
                    return
            self._station_task_cache(self._station_task_statuses, ns, status)
            if (record is not None and ns > record['contract']['source_ns']
                    and isinstance(offered, dict) and isinstance(offered.get('binding'), dict)
                    and offered['binding'] != record['contract']['binding']):
                self._station_task_retire('producer station binding changed')
                return
            if record is not None and status.get('state') != 'ready':
                if (status.get('state') == 'processing_deadline_pending'
                        and status.get('target_lost') is False):
                    try:
                        validate_recovery_parent(record['contract'],
                            parent=status.get('station_compute_wait'),
                            chain=status.get('station_compute_recovery_chain'),
                            binding=record['contract']['binding'], now_ns=rospy.Time.now().to_nsec())
                        self._station_task_support_gap_receipt(status)
                        self._station_task_quarantine()
                    except (ValueError, KeyError, TypeError, AttributeError):
                        self._station_task_retire('station pending contract changed')
                elif ns > record['contract']['source_ns']:
                    self._station_task_retire('unqualified producer failure during station wait')
            self._station_task_join()

    def _station_compute_task_object(self, msg):
        if self._station_requalification_task is not None:
            self._station_task_check()
            request = self._observation_measurement_request or {}
            if (self._station_requalification_task is not None and msg.detected is True
                    and request.get(STATION_REQUALIFICATION) is not None
                    and request.get('state') == 'requalifying'
                    and _ns(msg.header.stamp) > request['arrival_stamp_ns']):
                return False  # Finish still requires the exact ACK's final source.
            if (self._station_requalification_task is not None and msg.detected is True
                    and self._station_requalification_task['finalizing_phase'] is None):
                return True  # Old station cannot restore geometry while waiting.
        if not self._station_task_receiving():
            return False
        with self._grasp6d_plan_guard():
            self._station_task_check()
            ns = _ns(msg.header.stamp)
            if ns <= 0 or ns > rospy.Time.now().to_nsec():
                return False
            record = self._station_task_record
            history = self._station_task_gap_history.get(ns)
            if history is not None and msg.detected is True and msg != history['object']:
                self._station_task_retire('historical station object changed')
                return True
            if ns in self._station_task_gap_tombstones:
                if msg.detected is not False:
                    self._station_task_retire('discarded station source became detected')
                return True
            if (record is not None and ns < record['contract']['source_ns']
                    and ns <= self._station_task_cutoff):
                return True
            if (record is not None and ns == record['contract']['source_ns']
                    and msg.detected is True and msg != record['object']):
                self._station_task_retire('same-source station object changed')
                return True
            self._station_task_cache(self._station_task_objects, ns, msg)
            if record is not None and msg.detected is not True:
                self._station_task_quarantine()
                known = self._station_task_statuses.get(ns)
                if known is not None:
                    self._station_compute_task_status(known)
                return True
            pending = bool(record is not None and record['pending'])
            self._station_task_join()
            return pending or self._station_task_retired

    def _station_compute_task_mask_cb(self, msg):
        if not self._station_task_receiving():
            return
        try:
            if msg.encoding != 'mono8' or msg.width <= 0 or msg.height <= 0 or msg.step < msg.width:
                return
            raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            pixels = (raw.reshape(msg.height, msg.step)[:, :msg.width] > 0).astype(np.uint8)
            mask = dict(sha256=hashlib.sha256(pixels.tobytes()).hexdigest(),
                        shape=[msg.height, msg.width], frame_id=msg.header.frame_id)
            ns = _ns(msg.header.stamp)
            with self._grasp6d_plan_guard():
                history = self._station_task_gap_history.get(ns)
                if history is not None and mask != history['mask']:
                    self._station_task_retire('historical station mask changed')
                    return
                if ns in self._station_task_gap_tombstones:
                    if pixels.any():
                        self._station_task_retire('discarded station source became nonempty')
                    return
            if not pixels.any():
                return
            with self._grasp6d_plan_guard():
                self._station_task_check()
                record = self._station_task_record
                if (record is not None and _ns(msg.header.stamp) == record['contract']['source_ns']
                        and mask != record['mask']):
                    self._station_task_retire('same-source station mask changed')
                    return
                self._station_task_cache(self._station_task_masks, _ns(msg.header.stamp), mask)
                self._station_task_join()
        except (ValueError, TypeError, AttributeError):
            return

    def _station_task_plan_failure(self, plan):
        pending = self._station_task_check()
        if self._station_task_retired:
            return self._station_task_result(False, 'TARGET_LOST', 'station wait was permanently retired')
        if pending:
            return self._station_task_result(False, PENDING, 'waiting for a new independently measured frame')
        if (self._station_task_cutoff > 0
                and getattr(plan, 'diagnostic', '') == 'CONTACT_EXECUTION_PLAN'
                and _ns(plan.header.stamp) <= self._station_task_cutoff):
            return self._station_task_result(False, PENDING, 'contact source predates measured recovery')
        return None

    def _copy_near_field_preview_candidate(self, current_plan, *args, **kwargs):
        self._station_task_check()
        if self._station_requalification_task is not None:
            gcfg = args[1] if len(args) > 1 else kwargs.get('gcfg', {})
            try:
                self._station_requalification_run(current_plan, gcfg)
            except (ValueError, KeyError, TypeError, AttributeError) as exc:
                self._station_requalification_task = None
                self._station_task_retire(str(exc))
            return self._station_task_result(False, PENDING, 'station remeasurement requires new registration'), None
        with self._grasp6d_plan_guard():
            fault = self._station_task_plan_failure(current_plan)
            if fault is not None:
                return fault, None
            preview = getattr(self, 'latest_grasp6d_preview_plan', None)
            if (self._station_task_record is not None and preview is not None
                    and not getattr(preview, 'valid', False)
                    and getattr(preview, 'candidate_source', '') == 'near_field_pending'
                    and str(getattr(preview, 'diagnostic', '')).partition(':')[0] == PENDING):
                # A late preview can outlive the old pending interval. It is
                # a waiting hint, never permission to re-pend a fresh record.
                return self._station_task_result(False, PENDING, 'waiting for measured station recovery'), None
            result, candidate = super()._copy_near_field_preview_candidate(current_plan, *args, **kwargs)
            if result.ok:
                fault = self._station_task_plan_failure(candidate)
                if fault is not None:
                    return fault, None
            return result, candidate

    def _before_near_field_terminal_recovery(self, *args, **kwargs):
        with self._grasp6d_plan_guard():
            if self._station_task_check():
                return dict(waiting=True)
        return super()._before_near_field_terminal_recovery(*args, **kwargs)

    def _before_near_field_contact_freeze(self, candidate, current_plan, gcfg):
        with self._grasp6d_plan_guard():
            fault = self._station_task_plan_failure(candidate)
            if fault is not None:
                return dict(retry=True, minimum_stamp_ns=max(1, self._station_task_cutoff+1),
                            phase_deadline_sec=self._near_field_phase_deadline_sec)
        return super()._before_near_field_contact_freeze(candidate, current_plan, gcfg)

    def _freeze_execution_plan(self, plan, *args, **kwargs):
        with self._grasp6d_plan_guard():
            fault = self._station_task_plan_failure(plan)
            if fault is not None:
                raise ValueError(fault.code+': '+fault.reason)
            result = super()._freeze_execution_plan(plan, *args, **kwargs)
            self._station_task_disarm()
            return result

    def _observe_return_publish(self, message):
        session = getattr(self, '_observe_return_session', None) or {}
        if (self._station_task_record is not None and message.get('version') == 2
                and message.get('phase') == 'observing'
                and message.get('session_id') == session.get('session_id')
                and message.get('view_index') == session.get('view_index', 0) + 1):
            # The caller has verified the supplemental report and transition.
            # Stop the old station's clock before the asynchronous ACK/admit
            # wait. This is only metadata handoff; original new-plan action
            # validation and measured movement gates still follow it.
            with self._grasp6d_plan_guard():
                plan = self._bound_execution_plan
                fault = self._station_task_plan_failure(plan)
                if fault is None:
                    fault = self._validate_bound_plan_locked(plan, rospy.get_param('/grasp', {}))
                if not fault.ok:
                    raise ValueError(fault.code + ': ' + fault.reason)
                self._station_task_disarm()
        return super()._observe_return_publish(message)

    def _invoke_plan_bound_action(self, plan, gcfg, stage_label, action):
        with self._grasp6d_plan_guard():
            historical = stage_label == 'supplemental observation rebind'
            fault = self._station_task_plan_failure(plan)
            if fault is not None:
                return fault, None
            if (self._station_task_record is not None and not historical
                    and getattr(plan, 'diagnostic', '') == 'FAR_FIELD_OBSERVATION_PLAN'):
                return self._station_task_result(False, PENDING,
                    'old observation is historical while waiting for contact'), None
            return super()._invoke_plan_bound_action(plan, gcfg, stage_label, action)
