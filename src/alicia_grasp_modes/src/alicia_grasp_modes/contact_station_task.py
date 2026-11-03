"""Task-side station retirement immediately before strict contact submission."""
from alicia_grasp_modes.runtime_gate_policy import (SOURCE_MAX_AGE_NS)
from copy import deepcopy
import json
import math
import time

import rospy
from std_msgs.msg import String

from alicia_flexible_grasp.robot.observation_path_guard import committed_plan_digest
from alicia_flexible_grasp.grasp.grasp_state_machine import GraspStages
from .contact_station_retirement import REQUEST_TOPIC, ACK_TOPIC, PARTICIPANTS, parse_request, validate_ack
from .observe_return_measurement import measurement_digest
from .rgb_observation_source import JOINT_NAMES


class ContactStationTaskMixin:
    def _setup_contact_station_io(self):
        self._contact_station_acks = {}
        self._contact_station_pub = rospy.Publisher(REQUEST_TOPIC, String, queue_size=4)
        self._contact_station_sub = rospy.Subscriber(ACK_TOPIC, String, self._contact_station_ack_cb, queue_size=8)

    def _contact_station_ack_cb(self, message):
        try:
            value = json.loads(message.data)
            key = (value['request_id'], value['state'], value['participant'])
            with self._observe_return_condition:
                self._contact_station_acks[key] = value
                while len(self._contact_station_acks) > 16:
                    del self._contact_station_acks[next(iter(self._contact_station_acks))]
                self._observe_return_condition.notify_all()
        except (ValueError, KeyError, TypeError):
            return

    def _contact_station_scope(self, plan, stage):
        measurement = getattr(self, '_observation_measurement_request', None) or {}
        return bool(stage == GraspStages.MOVE_PREGRASP and getattr(plan, 'diagnostic', '') == 'CONTACT_EXECUTION_PLAN'
            and getattr(plan, 'model_choice', '') == 'unknown_tabletop'
            and measurement.get('state') == 'qualified'
            and measurement.get('motion_kind') == 'observation'
            and getattr(self, '_contact_station_retired_measurement', None) != measurement_digest(measurement))

    def _contact_station_feedback(self, epoch, since=None, reference=None, *, request=None):
        # A recorder's fresh SDK message does not prove this receiver processed
        # it. Keep the original local snapshot/checks, and diagnose failures
        # without querying another source or changing the acceptance threshold.
        marks = []
        def mark(stage):
            try:
                marks.append((stage, time.monotonic_ns(), time.thread_time_ns()))
            except Exception:
                pass  # Diagnostic sampling never changes the feedback gate.
        observed_epoch = raw_count = filtered_count = latest_ns = now = None
        initial_validation_ns = None
        checked_count = 0
        mark('entry')
        try:
            with self._following_evidence_lock:
                mark('lock_acquired')
                observed_epoch = self._following_epoch_ns
                if observed_epoch != epoch:
                    raise ValueError('CONTACT_STATION_CONTROL_EPOCH_CHANGED')
                raw_count = len(self._following_accepted_history)
                rows = deepcopy(list(self._following_accepted_history))
                mark('snapshot_copied')
            mark('lock_released')
            now = rospy.Time.now().to_nsec()
            initial_validation_ns = now
            mark('validation_clock_read')
            rows = [row for row in rows if row.header.frame_id == 'sdk_measured'
                    and row.header.stamp.to_nsec() >= epoch]
            filtered_count = len(rows)
            latest_ns = rows[-1].header.stamp.to_nsec() if rows else None
            mark('filtered')
            if not rows or not 0 <= now - latest_ns <= SOURCE_MAX_AGE_NS:
                raise ValueError('CONTACT_STATION_FRESH_FEEDBACK_REQUIRED')
            values = []
            for row in rows:
                if since is not None and row.header.stamp.to_nsec() < since:
                    continue
                q = dict(zip(row.name, row.position))
                if len(row.name) != len(row.position) or len(q) != len(row.name) or any(k not in q for k in JOINT_NAMES):
                    raise ValueError('CONTACT_STATION_FEEDBACK_INVALID')
                vector = [float(q[k]) for k in JOINT_NAMES]
                if not all(math.isfinite(v) for v in vector):
                    raise ValueError('CONTACT_STATION_FEEDBACK_INVALID')
                if reference is not None and max(abs(a-b) for a, b in zip(vector, reference)) > .004:
                    raise ValueError('CONTACT_STATION_MOVED_BEFORE_SUBMISSION')
                values.append(vector)
                checked_count += 1
            mark('vectors_checked')
            if not values:
                raise ValueError('CONTACT_STATION_FRESH_FEEDBACK_REQUIRED')
            # The epoch callback can clear the live history after the snapshot
            # lock is released. Preserve the same checked rows, and reject a
            # changed epoch instead of treating that old snapshot as current.
            with self._following_evidence_lock:
                now = rospy.Time.now().to_nsec()
                observed_epoch = self._following_epoch_ns
                if observed_epoch != epoch:
                    raise ValueError('CONTACT_STATION_CONTROL_EPOCH_CHANGED')
                if not 0 <= now-latest_ns <= SOURCE_MAX_AGE_NS:
                    raise ValueError('CONTACT_STATION_FRESH_FEEDBACK_REQUIRED')
            mark('return_context_checked')
            return latest_ns, values[-1]
        except Exception as exc:
            # This block runs after releasing the evidence lock. Diagnostic
            # failures must not replace or suppress the original rejection.
            try:
                mark('rejected')
                diagnostic = dict(schema_version=1, diagnostic_only=True,
                    execution_authorized=False, reason=str(exc),
                    expected_epoch_ns=epoch, observed_epoch_ns=observed_epoch,
                    snapshot_count=raw_count, filtered_count=filtered_count,
                    checked_count=checked_count, latest_source_ns=latest_ns,
                    since_ns=since, reference_present=reference is not None,
                    validation_ros_ns=now, initial_validation_ros_ns=initial_validation_ns,
                    latest_source_age_ns=None if now is None or latest_ns is None else now-latest_ns,
                    stages=[dict(stage=stage, monotonic_ns=wall, thread_cpu_ns=cpu,
                        elapsed_ns=wall-marks[0][1], elapsed_thread_cpu_ns=cpu-marks[0][2])
                        for stage, wall, cpu in marks])
                if request is not None:
                    diagnostic['request'] = {key: request[key] for key in
                        ('request_id', 'state', 'stamp_ns', 'deadline_ns')}
                self._contact_station_feedback_diagnostic = diagnostic
                rospy.logwarn('Contact station local feedback rejected: %s',
                              json.dumps(diagnostic, allow_nan=False))
            except Exception:
                pass
            raise

    @staticmethod
    def _contact_station_deadline(request, monotonic_deadline):
        now = rospy.Time.now().to_nsec()
        if (not request['stamp_ns'] <= now < request['deadline_ns']
                or (monotonic_deadline is not None and time.monotonic() >= monotonic_deadline)):
            raise ValueError('CONTACT_STATION_SUBMISSION_CHANGED')

    def _contact_station_context(self, plan, request, monotonic_deadline):
        self._observe_return_live()
        if (measurement_digest(self._observation_measurement_request) != request['measurement_sha256']
                or committed_plan_digest(plan) != request['plan_sha256']):
            raise ValueError('CONTACT_STATION_SUBMISSION_CHANGED')
        self._contact_station_deadline(request, monotonic_deadline)

    def _contact_station_live(self, plan, request, gcfg, start_ns, q, monotonic_deadline=None):
        self._contact_station_context(plan, request, monotonic_deadline)
        self._contact_station_feedback(request['control_epoch_ns'], start_ns, q, request=request)
        validation = self._validate_bound_plan_locked(plan, gcfg)
        if not validation.ok:
            raise ValueError(validation.code + ': ' + validation.reason)
        # The original gate can block. Its return does not renew the handoff or
        # make the pre-gate owner, plan and measured feedback current again.
        self._contact_station_context(plan, request, monotonic_deadline)
        self._contact_station_feedback(request['control_epoch_ns'], start_ns, q, request=request)
        self._contact_station_deadline(request, monotonic_deadline)

    def _contact_station_publish(self, request, plan, gcfg, start_ns, q, monotonic_deadline=None):
        cutoff = monotonic_deadline
        if cutoff is None:
            cutoff = time.monotonic() + max(0., (request['deadline_ns'] - rospy.Time.now().to_nsec())*1e-9)
        self._contact_station_deadline(request, cutoff)
        self._contact_station_pub.publish(String(data=json.dumps(request, allow_nan=False)))
        while time.monotonic() < cutoff:
            self._contact_station_live(plan, request, gcfg, start_ns, q, cutoff)
            with self._observe_return_condition:
                values = {p: deepcopy(self._contact_station_acks.get((request['request_id'], request['state'], p)))
                          for p in PARTICIPANTS}
                for p, value in values.items():
                    if value is not None:
                        validate_ack(value, request, p)
                if all(values.values()):
                    self._contact_station_deadline(request, cutoff)
                    return values
                self._observe_return_condition.wait(.01)
        raise ValueError('CONTACT_STATION_ACK_TIMEOUT')

    def _contact_station_before_submit(self, plan, gcfg):
        measurement = self._observation_measurement_request
        self._observation_measurement_live(measurement)
        phase = getattr(self, '_published_near_field_phase', None)
        if (phase is None or not phase.active or phase.planning_stage != 'contact_replan'
                or phase.reference_target_track_id != plan.target_track_id):
            raise ValueError('CONTACT_STATION_CONTACT_PHASE_REQUIRED')
        started_monotonic = time.monotonic()
        now = rospy.Time.now().to_nsec()
        start_ns, q = self._contact_station_feedback(measurement['control_epoch_ns'])
        request = dict(version=1, state='prepare', request_id='%s:contact:%d' % (measurement['lease_id'], now),
            selection=deepcopy(self._mode_bound_selection), session_id=measurement['session_id'],
            target_track_id=plan.target_track_id, target_epoch=measurement['target_epoch'],
            control_epoch_ns=measurement['control_epoch_ns'], measurement_sha256=measurement_digest(measurement),
            plan_id=plan.plan_id, plan_sha256=committed_plan_digest(plan), plan_source_ns=plan.header.stamp.to_nsec(),
            phase_id=int(phase.phase_id), phase_started_ns=phase.header.stamp.to_nsec(), stage='MOVE_PREGRASP',
            stamp_ns=now, deadline_ns=min(now+SOURCE_MAX_AGE_NS, measurement['deadline_ns'], phase.deadline.to_nsec()),
            metric_geometry_authority=False, motion_authorized=False, controller_motion_confirmed=False)
        request = parse_request(request, self._mode_bound_selection, now)
        # One original budget covers prepare, retire and the final submission
        # check. A ROS clock pause/rollback cannot open a new phase budget.
        cutoff = started_monotonic + (request['deadline_ns']-now)*1e-9
        receipts = self._contact_station_publish(request, plan, gcfg, start_ns, q, cutoff)
        retire = dict(request, state='retire', stamp_ns=rospy.Time.now().to_nsec(), prepared_acks=receipts)
        retire = parse_request(retire, self._mode_bound_selection, rospy.Time.now().to_nsec(), request)
        self._contact_station_publish(retire, plan, gcfg, start_ns, q, cutoff)
        # Both owners have retired stationary privileges. Check again after
        # asynchronous ACKs, immediately before calling the unchanged executor.
        rospy.loginfo('Contact station retired at strict submission boundary: %s', json.dumps(retire, allow_nan=False))
        self._contact_station_live(plan, retire, gcfg, start_ns, q, cutoff)
        self._contact_station_retired_measurement = request['measurement_sha256']
