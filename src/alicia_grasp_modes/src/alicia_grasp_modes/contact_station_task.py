"""Task-side station retirement immediately before strict contact submission."""
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

    def _contact_station_feedback(self, epoch, since=None, reference=None):
        with self._following_evidence_lock:
            if self._following_epoch_ns != epoch:
                raise ValueError('CONTACT_STATION_CONTROL_EPOCH_CHANGED')
            rows = deepcopy(list(self._following_accepted_history))
        now = rospy.Time.now().to_nsec()
        rows = [row for row in rows if row.header.frame_id == 'sdk_measured'
                and row.header.stamp.to_nsec() >= epoch]
        if not rows or not 0 <= now - rows[-1].header.stamp.to_nsec() <= 800_000_000:
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
        if not values:
            raise ValueError('CONTACT_STATION_FRESH_FEEDBACK_REQUIRED')
        return rows[-1].header.stamp.to_nsec(), values[-1]

    def _contact_station_live(self, plan, request, gcfg, start_ns, q):
        self._observe_return_live()
        if (measurement_digest(self._observation_measurement_request) != request['measurement_sha256']
                or committed_plan_digest(plan) != request['plan_sha256']
                or rospy.Time.now().to_nsec() >= request['deadline_ns']):
            raise ValueError('CONTACT_STATION_SUBMISSION_CHANGED')
        self._contact_station_feedback(request['control_epoch_ns'], start_ns, q)
        validation = self._validate_bound_plan_locked(plan, gcfg)
        if not validation.ok:
            raise ValueError(validation.code + ': ' + validation.reason)

    def _contact_station_publish(self, request, plan, gcfg, start_ns, q):
        self._contact_station_pub.publish(String(data=json.dumps(request, allow_nan=False)))
        cutoff = time.monotonic() + max(0., (request['deadline_ns'] - rospy.Time.now().to_nsec())*1e-9)
        while time.monotonic() < cutoff:
            self._contact_station_live(plan, request, gcfg, start_ns, q)
            with self._observe_return_condition:
                values = {p: deepcopy(self._contact_station_acks.get((request['request_id'], request['state'], p)))
                          for p in PARTICIPANTS}
                for p, value in values.items():
                    if value is not None:
                        validate_ack(value, request, p)
                if all(values.values()):
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
        now = rospy.Time.now().to_nsec()
        start_ns, q = self._contact_station_feedback(measurement['control_epoch_ns'])
        request = dict(version=1, state='prepare', request_id='%s:contact:%d' % (measurement['lease_id'], now),
            selection=deepcopy(self._mode_bound_selection), session_id=measurement['session_id'],
            target_track_id=plan.target_track_id, target_epoch=measurement['target_epoch'],
            control_epoch_ns=measurement['control_epoch_ns'], measurement_sha256=measurement_digest(measurement),
            plan_id=plan.plan_id, plan_sha256=committed_plan_digest(plan), plan_source_ns=plan.header.stamp.to_nsec(),
            phase_id=int(phase.phase_id), phase_started_ns=phase.header.stamp.to_nsec(), stage='MOVE_PREGRASP',
            stamp_ns=now, deadline_ns=min(now+800_000_000, measurement['deadline_ns'], phase.deadline.to_nsec()),
            metric_geometry_authority=False, motion_authorized=False, controller_motion_confirmed=False)
        request = parse_request(request, self._mode_bound_selection, now)
        receipts = self._contact_station_publish(request, plan, gcfg, start_ns, q)
        retire = dict(request, state='retire', stamp_ns=rospy.Time.now().to_nsec(), prepared_acks=receipts)
        retire = parse_request(retire, self._mode_bound_selection, rospy.Time.now().to_nsec(), request)
        self._contact_station_publish(retire, plan, gcfg, start_ns, q)
        # Both owners have retired stationary privileges. Check again after
        # asynchronous ACKs, immediately before calling the unchanged executor.
        self._contact_station_live(plan, retire, gcfg, start_ns, q)
        self._contact_station_retired_measurement = request['measurement_sha256']
        rospy.loginfo('Contact station retired at strict submission boundary: %s', json.dumps(retire, allow_nan=False))
