"""Planner attestation for retiring an observation station, never motion authority."""
from copy import deepcopy
import json

import rospy
from std_msgs.msg import String

from alicia_flexible_grasp.robot.observation_path_guard import committed_plan_digest
from .contact_station_retirement import REQUEST_TOPIC, ACK_TOPIC, parse_request, ack, wait_for_pregrasp_state
from .observe_return_measurement import measurement_digest


class ContactStationRemoteMixin:
    def _contact_station_plan_candidates(self, request):
        return [getattr(self, key, None)
                for key in ('latest_rich_plan', 'latest_preview_rich_plan')]

    def _setup_contact_station_io(self):
        self._contact_station_prepared = None
        self._contact_station_retired_measurement = None
        self._contact_station_ack_pub = rospy.Publisher(ACK_TOPIC, String, queue_size=4)
        self._contact_station_sub = rospy.Subscriber(REQUEST_TOPIC, String, self._contact_station_request_cb, queue_size=4)

    def _contact_station_live_context(self, request):
        measurement = getattr(self, '_measurement_remote_request', None) or {}
        session = self._observe_return_session or {}
        phase = getattr(self, '_station_compute_phase', None)
        identity = self._current_stream_target_identity()
        if (not self._selection_is_current() or request['selection'] != self._mode_selection
                or self._observe_return_phase != 'observing_contact'
                or self._observe_return_diagnostic_observation_only
                or session.get('phase') != 'observing' or session.get('session_id') != request['session_id']
                or measurement.get('state') != 'qualified' or measurement.get('motion_kind') != 'observation'
                or measurement_digest(measurement) != request['measurement_sha256']
                or getattr(self, '_contact_station_retired_measurement', None) == request['measurement_sha256']
                or any(measurement.get(k) != request[k] for k in
                       ('selection', 'session_id', 'target_track_id', 'target_epoch', 'control_epoch_ns'))
                or identity.epoch != request['target_epoch'] or identity.track_id != request['target_track_id']
                or self._measurement_remote_epoch_ns != request['control_epoch_ns']
                or self._unknown_identity_pending is not None
                or self._unknown_identity_retired_selection == self._mode_selection
                or self._station_compute_retired
                or self._station_compute_task_state != 'MOVE_PREGRASP'
                or phase is None or not phase.active or phase.planning_stage != 'contact_replan'
                or phase.phase_id != request['phase_id'] or phase.header.stamp.to_nsec() != request['phase_started_ns']
                or phase.reference_target_track_id != request['target_track_id']
                or rospy.get_param('/gui/joint_direct_mode', False) is not False
                or int(self._ros_source_clock_ns()) >= min(request['deadline_ns'], phase.deadline.to_nsec(), session['deadline_ns'])):
            raise ValueError('CONTACT_STATION_REMOTE_CONTEXT_CHANGED')

    def _contact_station_pending_retirement_allowed(self, request):
        check = getattr(self, '_station_contact_pending_retirement_allowed', None)
        return bool(check is not None and check(request))

    def _contact_station_context(self, request):
        self._contact_station_live_context(request)
        if (self._station_compute_pending
                and not self._contact_station_pending_retirement_allowed(request)):
            raise ValueError('CONTACT_STATION_REMOTE_CONTEXT_CHANGED')
        plans = self._contact_station_plan_candidates(request)
        if not any(plan is not None and plan.valid and plan.plan_id == request['plan_id']
                and plan.model_choice == 'unknown_tabletop' and plan.diagnostic == 'CONTACT_EXECUTION_PLAN'
                and plan.target_track_id == request['target_track_id'] and len(plan.poses) == 4
                and plan.header.stamp.to_nsec() == request['plan_source_ns']
                and committed_plan_digest(plan) == request['plan_sha256'] for plan in plans):
            raise ValueError('CONTACT_STATION_PUBLISHED_CONTACT_PLAN_REQUIRED')

    def _contact_station_request_cb(self, message):
        request = None
        try:
            if len(message.data) > 32768:
                raise ValueError('CONTACT_STATION_REQUEST_TOO_LARGE')
            raw = json.loads(message.data)
            if raw.get('state') == 'prepare':
                request = parse_request(raw, self._mode_selection, int(self._ros_source_clock_ns()))
                wait_for_pregrasp_state(request,
                    lambda: (self._station_compute_task_state, bool(getattr(self, 'robot_execution_active', False))),
                    lambda: self._selection_is_current()
                        and self._measurement_remote_epoch_ns == request['control_epoch_ns']
                        and rospy.get_param('/gui/joint_direct_mode', False) is False,
                    lambda: int(self._ros_source_clock_ns()))
        except (ValueError, KeyError, TypeError, AttributeError) as exc:
            if request is not None:
                self._contact_station_ack_pub.publish(String(data=json.dumps(ack(request, 'remote', False, str(exc)))))
            return
        with self._mode_control_lock, self._stream_condition, self._geometry_state_guard():
            try:
                if len(message.data) > 32768:
                    raise ValueError('CONTACT_STATION_REQUEST_TOO_LARGE')
                raw = json.loads(message.data)
                previous = getattr(self, '_contact_station_prepared', None)
                request = parse_request(raw, self._mode_selection, int(self._ros_source_clock_ns()),
                    previous if raw.get('state') == 'retire' else None)
                if request['state'] == 'prepare' and previous is not None:
                    raise ValueError('CONTACT_STATION_REPLAY_OR_ALREADY_PREPARED')
                self._contact_station_context(request)
                if request['state'] == 'prepare':
                    self._contact_station_prepared = deepcopy(request)
                else:
                    self._contact_station_prepared = None
                    self._contact_station_retired_measurement = request['measurement_sha256']
                    # Retiring a computation contract is not retiring identity.
                    # Do not invoke _station_compute_retire(), which means loss.
                    self._station_contact_history = None
                    self._station_compute_reference = None
                    self._station_compute_work.clear()
                    self._station_compute_ready_history.clear()
                    self._station_compute_objects.clear()
                    self._station_compute_masks.clear()
                    self._station_compute_status.clear()
                self._contact_station_ack_pub.publish(String(data=json.dumps(ack(request, 'remote'))))
            except (ValueError, KeyError, TypeError, AttributeError) as exc:
                if request is not None:
                    self._contact_station_ack_pub.publish(String(data=json.dumps(ack(request, 'remote', False, str(exc)))))
                else:
                    rospy.logwarn('Contact station planner receipt rejected: %s', exc)
