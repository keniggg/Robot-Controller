"""Await one already admitted frame before freezing a new observation lease.

This queue grants no identity or motion authority. Only the exact in-flight
RGB-D frame may complete its original checks and publish READY; the ordinary
measurement callback then freezes that new, still-fresh reference.
"""
from collections import deque
from copy import deepcopy
from dataclasses import asdict
import json
import time

import numpy as np
import rospy
import tf2_ros
from std_msgs.msg import String

from .observe_return_measurement import parse_measurement_request, STATION_REQUALIFICATION
from .rgb_observation_source import digest, require_fresh_source
from .rgb_observation_handoff import _candidate_fingerprint
from .selection import parse_selection
from alicia_flexible_grasp.robot.observation_view_gateway import _camera_configuration_contract


class SupplementMeasurementAdmissionMixin:
    def _station_compute_state_cb(self, message):
        with self.input_lock:
            admission = getattr(self, '_supplement_measurement_admission', None)
            if admission is not None and (not message.active or message.state != 'PLAN_PREGRASP'
                    or message.message != 'admitting supplemental observation view'):
                admission['invalidated_reason'] = 'SUPPLEMENT_MEASUREMENT_ADMISSION_TASK_CHANGED'
            return super()._station_compute_state_cb(message)

    def _supplement_admission_used(self, request):
        if not hasattr(self, '_observation_measurement_seen'):
            self._observation_measurement_seen = deque(maxlen=32)
        if request['lease_id'] not in self._observation_measurement_seen:
            self._observation_measurement_seen.append(request['lease_id'])

    def _supplement_admission_cancel(self, reason):
        admission = getattr(self, '_supplement_measurement_admission', None)
        if admission is not None:
            self._supplement_measurement_admission = None
            self._supplement_admission_used(admission['request'])
            self._measurement_ack(admission['request'], False, str(reason))

    def _supplement_admission_receive(self, request):
        admission = getattr(self, '_supplement_measurement_admission', None)
        if admission is None:
            return False
        if request == admission['request']:
            # Exact retries cannot renew the captured source or either clock.
            try:
                self._supplement_admission_check(admission)
            except (ValueError, KeyError, TypeError, AttributeError, tf2_ros.LookupException,
                    tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
                self._supplement_admission_cancel(str(exc))
            return True
        if request['state'] == 'revoked':
            parse_measurement_request(request, request['selection'], rospy.Time.now().to_nsec(),
                admission['request'])
            self._supplement_admission_cancel(request.get('reason', 'task_revoked'))
            self._measurement_ack(request, False, request.get('reason', 'task_revoked'))
            return True
        self._measurement_ack(request, False, 'SUPPLEMENT_MEASUREMENT_ADMISSION_PENDING')
        return True

    def _supplement_admission_check(self, admission, *, completed=False):
        request, marker = admission['request'], admission['marker']
        now = rospy.Time.now().to_nsec()
        if (now >= admission['deadline_ns'] or time.monotonic() >= admission['monotonic_deadline']
                or now < request['stamp_ns']):
            raise ValueError('SUPPLEMENT_MEASUREMENT_ADMISSION_EXPIRED')
        reference = self._locked_rgb_hold_reference
        state = self._station_compute_task_state
        if (admission.get('invalidated_reason')
                or reference is None or reference.get('observation_arrival_retry') is not marker
                or marker.get('invalidated_reason') or reference['pose_invalidated']
                or self.first_loss is not None or self.tracker.lost
                or self.tracker.anchor is not admission['anchor']
                or _candidate_fingerprint(self.tracker.anchor) != marker['anchor_fingerprint']
                or not np.array_equal(self.tracker.plane_base, marker['reference_plane'])
                or (self.mode, self.generation, self.selection_ns) !=
                   ('unknown', request['selection']['generation'], request['selection']['stamp_ns'])
                or parse_selection(rospy.get_param_cached('/grasp_mode/selection')) != request['selection']
                or self._rgb_interest_epoch != request['control_epoch_ns']
                or rospy.get_param_cached('/gui/joint_direct_mode', False) is not False
                or not state.active or state.state != 'PLAN_PREGRASP'
                or state.message != 'admitting supplemental observation view'
                or self._station_compute_phase != admission['phase']
                or getattr(self, '_observation_measurement', None) is not None
                or getattr(self, '_locked_source_pending', None) is not None
                or getattr(self, '_metric_ambiguity_pending', None) is not None
                or digest(asdict(self.config)) != marker['config_sha256']
                or digest(_camera_configuration_contract(rospy.get_param_cached('/camera', {})))
                   != marker['camera_configuration_sha256']):
            raise ValueError('SUPPLEMENT_MEASUREMENT_ADMISSION_CONTEXT_CHANGED')
        registration = self._station_compute_registration
        if registration is None or self._station_compute_scope(registration) is not reference:
            raise ValueError('SUPPLEMENT_MEASUREMENT_ADMISSION_STATION_CHANGED')
        if digest(self._rgb_interest_source_context(admission['source'][0].header.stamp)) != marker['source_id']:
            raise ValueError('SUPPLEMENT_MEASUREMENT_ADMISSION_SOURCE_CONTEXT_CHANGED')
        if self._station_compute_scope(registration) is not reference or admission.get('invalidated_reason'):
            raise ValueError('SUPPLEMENT_MEASUREMENT_ADMISSION_CONTEXT_CHANGED')
        now = rospy.Time.now().to_nsec()
        require_fresh_source(admission['source_ns'], now, min(self.max_age, .8))
        if now >= admission['deadline_ns'] or time.monotonic() >= admission['monotonic_deadline']:
            raise ValueError('SUPPLEMENT_MEASUREMENT_ADMISSION_EXPIRED')
        if completed:
            if (reference is admission['reference']
                    or reference['source_stamp_ns'] != admission['source_ns']
                    or self.last_good_source_stamp_ns != admission['source_ns']
                    or reference['source_id'] != admission['reference']['source_id']):
                raise ValueError('SUPPLEMENT_MEASUREMENT_ADMISSION_NEW_READY_REQUIRED')
            require_fresh_source(reference['source_stamp_ns'], now, min(self.max_age, .8))
        else:
            if (reference is not admission['reference']
                    or registration is not admission['registration']
                    or self._station_compute_frame is not admission['frame']
                    or self._arrival_frame_completion is not admission['completion']):
                raise ValueError('SUPPLEMENT_MEASUREMENT_ADMISSION_FRAME_CHANGED')
            self._require_station_frame_reference_lifetime(reference, marker,
                admission['source_ns'], now, 'observation_arrival_retry')

    def _supplement_admission_queue(self, request):
        """Called only when the ordinary fresh-reference gate has just failed."""
        reference = getattr(self, '_locked_rgb_hold_reference', None)
        token = getattr(self, '_arrival_frame_completion', None)
        frame = getattr(self, '_station_compute_frame', None)
        if (request.get(STATION_REQUALIFICATION) is not None
                or reference is None or token is None or frame is None):
            return False
        marker = reference.get('observation_arrival_retry')
        previous = (getattr(self, '_observation_measurement_status', None) or {}).get('request')
        now = rospy.Time.now().to_nsec()
        if (marker is None or previous is None or previous['state'] != 'qualified'
                or request['state'] != 'executing' or request['motion_kind'] != 'observation'
                or request['view_index'] != previous['view_index'] + 1
                or request['view_plan_id'] == previous['view_plan_id']
                or request['lease_id'] == previous['lease_id']
                or any(request[k] != previous[k] for k in ('selection', 'session_id',
                    'target_epoch', 'target_track_id', 'origin_plan_id', 'origin_snapshot_stamp_ns',
                    'control_epoch_ns', 'deadline_ns'))
                or marker['lease_id'] != previous['lease_id']
                or token['reference'] is not reference or token['marker'] is not marker
                or token['marker_kind'] != 'observation_arrival_retry'
                or not reference['source_stamp_ns'] < frame['source_ns'] <= request['stamp_ns'] <= now
                or frame['source_ns'] != token['source_stamp_ns']
                or now - reference['source_stamp_ns'] <= round(min(self.max_age, .8)*1e9)):
            return False
        if getattr(self, '_supplement_measurement_admission', None) is not None:
            raise ValueError('SUPPLEMENT_MEASUREMENT_ADMISSION_PENDING')
        admission = dict(request=deepcopy(request), reference=reference, marker=marker,
            completion=token, frame=frame, source=frame['source'], source_ns=frame['source_ns'],
            registration=frame['registration'], anchor=self.tracker.anchor,
            phase=deepcopy(self._station_compute_phase),
            deadline_ns=min(token['completion_deadline_ns'], request['deadline_ns'],
                self._station_compute_phase.deadline.to_nsec()),
            monotonic_deadline=min(token['completion_monotonic'],
                frame['registration']['monotonic_deadline']))
        with self.input_lock:
            self._supplement_admission_check(admission)
            self._supplement_measurement_admission = admission
        return True

    def _supplement_admission_commit_check(self, request):
        admission = getattr(self, '_supplement_measurement_admitting', None)
        if admission is not None:
            if request != admission['request']:
                raise ValueError('SUPPLEMENT_MEASUREMENT_ADMISSION_REQUEST_CHANGED')
            self._supplement_admission_check(admission, completed=True)

    def _supplement_admission_ready(self, source):
        admission = getattr(self, '_supplement_measurement_admission', None)
        if admission is None:
            return
        try:
            if source is not admission['source']:
                raise ValueError('SUPPLEMENT_MEASUREMENT_ADMISSION_FRAME_CHANGED')
            self._supplement_admission_check(admission, completed=True)
            self._supplement_measurement_admission = None
            self._supplement_measurement_admitting = admission
            try:
                self._observation_measurement_request_cb(String(data=json.dumps(admission['request'])))
            finally:
                self._supplement_measurement_admitting = None
                self._supplement_admission_used(admission['request'])
        except (ValueError, KeyError, TypeError, AttributeError, tf2_ros.LookupException,
                tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            self._supplement_admission_cancel(str(exc))
