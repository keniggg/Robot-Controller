#!/usr/bin/env python3
"""Independent exact-RGB-D tabletop perception. This node has no arm controls."""
import json
import math
import threading
import time
import hashlib
import gc
from alicia_grasp_modes.initial_station_identity_completion import (
    completion_contract, station_from_marker, validate_completion)
from copy import deepcopy
from contextlib import contextmanager
from dataclasses import asdict, fields, replace
from collections import OrderedDict, deque

import message_filters
import cv2
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, Header, String
from alicia_flexible_grasp_supervisor.msg import ObjectPose, NearFieldPlanningPhase, GraspState
from alicia_grasp_modes.station_compute_wait_producer import (
    StationComputeWaitProducerMixin, StationComputeDeadline, StationComputeRetired, StationFeedbackBracketGap,
    ObservationArrivalStationChanged)
from alicia_grasp_modes.contact_station_producer import ContactStationProducerMixin
from alicia_grasp_modes.supplement_measurement_admission import SupplementMeasurementAdmissionMixin
from alicia_grasp_modes.feedback_bracket_gap import (
    KIND as FEEDBACK_GAP, InitialFeedbackBracketGap, validate_gap_evidence)
from alicia_grasp_modes.feedback_recovery_completion import (
    FIELD as FEEDBACK_COMPLETION, STATE as FEEDBACK_PROCESSING,
    make_completion, validate_feedback_completion)
from alicia_grasp_modes.tabletop import (
    Candidate, Config, TargetTracker, segment_with_support_fallback, _validate_support_plane_update,
    _matching_candidates, _support_offset_diagnostics)
from alicia_grasp_modes.rgb_instance import refine_candidate
from alicia_grasp_modes.arrival_fragment_group import (
    recover_arrival_fragment_group, recover_arrival_matching_fragments)
from alicia_grasp_modes.source_connected_owner import recover_source_connected_fragment_owner
from alicia_grasp_modes.source_sampling_topology import SourceSamplingTopologyCache
from alicia_grasp_modes.metric_ambiguity_identity import metric_seeded_identity_evidence
from alicia_grasp_modes.acquisition import InitialAcquisition
from alicia_grasp_modes.loss_observation import first_loss_observation, candidate_loss_observation
from alicia_grasp_modes.rgb_observation_region import (
    CenterSearchPolicy, RgbObservationRegions, RgbObservationTracker, propose_rgb_observation_regions)
from alicia_grasp_modes.rgb_observation_set import (
    RgbObservationSetTracker, observation_set_digest, observation_set_metadata, _unique_assignment)
from alicia_grasp_modes.rgb_observation_set_handoff import (
    RgbInterestSetReference, freeze_rgb_interest_set, associate_rgb_interest_set)
from alicia_grasp_modes.rgb_observation_volume import (
    SupportPlaneProvenance, build_rgb_observation_volume, project_rgb_observation_volume)
from alicia_grasp_modes.rgb_observation_projection import projected_rgb_search_policy
from alicia_grasp_modes.rgb_observation_handoff import (
    RgbHandoffFrame, RgbMeasuredHandoff, freeze_rgb_interest, associate_rgb_interest,
    HandoffPolicy, _appearance, _mask, _candidate_fingerprint)
from alicia_grasp_modes.rgb_observation_source import (
    JOINT_NAMES, bracket_feedback, camera_geometry_snapshot, digest,
    empty_observation_interest, measured_support_coverage, require_fresh_source)
from alicia_grasp_modes.selection import parse_selection
from alicia_grasp_modes.observe_return_measurement import (
    REQUEST_TOPIC as MEASUREMENT_REQUEST_TOPIC, ACK_TOPIC as MEASUREMENT_ACK_TOPIC,
    parse_measurement_request, measurement_ack, STATION_REQUALIFICATION,
    station_requalification_key, ACTUATION_BOOTSTRAP, bootstrap_source_deadline)
from alicia_grasp_modes.observation_arrival_confirmation import ObservationArrivalConfirmation
from alicia_grasp_modes.observation_appearance import FrozenObservationAppearance
from alicia_grasp_modes.observation_measured_face import (
    FrozenMeasuredFaceSource, MeasuredFaceUnavailable, freeze_measured_face)
from alicia_flexible_grasp.robot.observation_view_gateway import _camera_configuration_contract
from alicia_grasp_modes.rgb_prefix_preflight import validate_prefix_preflight_receipt
from alicia_flexible_grasp.robot.observation_view_limits import (
    DEFAULT_OBSERVATION_VIEW_PARAMETERS, effective_observation_view_limits)
from alicia_flexible_grasp.vision.rgbd_rectification import (
    rectification_rgb_valid_mask, require_rectified_source_after_activation)



# Only failures emitted at the segmentation call may enter an identity-only
# support gap. Malformed input and positive plane disagreement remain failures.
_SUPPORT_MEASUREMENT_GAPS = frozenset((
    'insufficient_valid_depth', 'insufficient_support_depth',
    'support_plane_not_found', 'support_plane_inlier_ratio_low',
    'support_plane_residual_too_large', 'insufficient_support_spatial_coverage',
    'locked_support_plane_no_longer_visible'))


def _binding_copy(value):
    """Owned copy preserving original Python types, including tuples."""
    kind = type(value)
    if value is None or kind in (bool, int, float, str):
        return value
    if kind is dict:
        return {key: _binding_copy(item) for key, item in value.items()}
    if kind is list:
        return [_binding_copy(item) for item in value]
    if kind is tuple:
        return tuple(_binding_copy(item) for item in value)
    if kind is np.ndarray:
        return value.copy()
    return deepcopy(value)


def _binding_cache_supported(value):
    kind = type(value)
    if value is None or kind in (bool, int, str):
        return True
    if kind is float:
        return math.isfinite(value)
    if kind is np.ndarray:
        return not value.dtype.hasobject
    if kind is dict:
        return all(type(key) is str and _binding_cache_supported(item)
                   for key, item in value.items())
    if kind in (list, tuple):
        return all(_binding_cache_supported(item) for item in value)
    return False


def _binding_equal(current, frozen):
    """Exact original values, never object identity or a time-to-live."""
    kind = type(current)
    if kind is not type(frozen):
        return False
    if current is None or kind in (bool, int, str):
        return current == frozen
    if kind is float:
        # JSON distinguishes 1/1.0 and +0.0/-0.0. Never cache NaN/Inf.
        return math.isfinite(current) and current.hex() == frozen.hex()
    if kind is np.ndarray:
        if (current.dtype != frozen.dtype or current.shape != frozen.shape
                or current.dtype.hasobject):
            return False
        return np.array_equal(np.ascontiguousarray(current).reshape(-1).view(np.uint8),
                              np.ascontiguousarray(frozen).reshape(-1).view(np.uint8))
    if kind is dict:
        return (len(current) == len(frozen) and current.keys() == frozen.keys()
                and all(type(key) is str and _binding_equal(value, frozen[key])
                        for key, value in current.items()))
    if kind in (list, tuple):
        return len(current) == len(frozen) and all(
            _binding_equal(a, b) for a, b in zip(current, frozen))
    return False


class _ExactValueMemo:
    """Bounded pure computation reuse after complete value comparison.

    The entry is private and replaced atomically. Mutable results must only be
    exported through _binding_copy; no caller can alter a cached input tree.
    """
    def __init__(self):
        self._entry = None

    def get(self, value, compute):
        try:
            entry = self._entry
            if entry is not None and _binding_equal(value, entry[0]):
                return entry[1]
            frozen = _binding_copy(value)
            if not _binding_cache_supported(frozen):
                # Preserve the original pure validator for non-JSON/custom
                # values. They never qualify for a cache hit.
                return compute(frozen)
            result = compute(frozen)
            if not _binding_equal(value, frozen):
                raise ValueError('observation_binding_changed_during_snapshot')
            self._entry = (frozen, result)
            return result
        except (RuntimeError, KeyError) as exc:
            raise ValueError('observation_binding_changed_during_snapshot') from exc


def _candidate_binding_values(candidate):
    if not isinstance(candidate, Candidate):
        raise ValueError('handoff_measured_candidate_type_invalid')
    return (candidate.mask, candidate.position_camera, candidate.position_base,
        candidate.extent_m, candidate.support_footprint_base, candidate.bbox,
        candidate.center_uv, candidate.depth_m, candidate.quality,
        candidate.metrics.get('support_plane_base'), candidate.metrics.get('valid_points', 0))


def _candidate_binding_fingerprint(values):
    candidate = Candidate(mask=values[0], position_camera=values[1], position_base=values[2],
        extent_m=values[3], support_footprint_base=values[4], bbox=values[5],
        center_uv=values[6], depth_m=values[7], quality=values[8],
        metrics=dict(support_plane_base=values[9], valid_points=values[10]))
    return _candidate_fingerprint(candidate)


def _rgb_source_kwargs(camera, image_shape):
    """Opt-in RGB evidence domain; never mask or rewrite measured depth."""
    valid = rectification_rgb_valid_mask(camera, image_shape)
    return {} if valid is None else {'rgb_valid': valid}


def exact_tf_wait_budget(source_age, max_age, configured_wait):
    """Wait for bracketing measured TF without renewing the image's lifetime."""
    if (not all(math.isfinite(v) for v in (source_age, max_age, configured_wait))
            or max_age <= 0 or configured_wait <= 0):
        raise ValueError('invalid_exact_tf_wait_configuration')
    if not 0 <= source_age <= max_age:
        raise ValueError('source_frame_stale_or_future')
    return min(configured_wait, max_age - source_age)


def transform_matrix(message):
    t, q = message.transform.translation, message.transform.rotation
    quaternion = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = float(quaternion @ quaternion)
    if not np.isfinite(norm) or abs(norm - 1.0) > 0.01:
        raise ValueError('invalid_exact_tf_quaternion')
    x, y, z, w = quaternion / np.sqrt(norm)
    matrix = np.eye(4)
    matrix[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    matrix[:3, 3] = [t.x, t.y, t.z]
    return matrix


class RgbdIngressTiming:
    """Bounded numeric evidence before exact synchronization; never owns pixels.

    Object identity plus the original stamp binds arrival data to the exact
    message chosen by TimeSynchronizer. Sequence is frozen, since downstream
    publishers can mutate a shared Header. This cache never selects frames.
    """
    def __init__(self, capacity=32):
        if isinstance(capacity, bool) or not isinstance(capacity, int) or capacity < 1:
            raise ValueError('invalid RGB-D timing cache capacity')
        self.capacity = capacity
        self.lock = threading.Lock()
        self.records = {name: OrderedDict() for name in ('color', 'depth')}
        self.latest = {name: None for name in self.records}

    def record(self, stream, message, ros_ns, monotonic_sec):
        stamp = message.header.stamp.to_nsec()
        key = (id(message), stamp)
        row = dict(source_stamp_ns=stamp, source_seq=int(getattr(message.header, 'seq', 0)),
                   arrival_ros_ns=ros_ns, arrival_monotonic_sec=monotonic_sec,
                   source_age_at_arrival_sec=(ros_ns-stamp)*1e-9)
        with self.lock:
            records = self.records[stream]
            # Match TimeSynchronizer's latest delivery for a source stamp.
            # We retain no Image references, so an object's id can be reused
            # after release; retaining the first row would misattribute it.
            records[key] = row
            records.move_to_end(key)
            if len(records) > self.capacity:
                records.popitem(last=False)
            self.latest[stream] = row

    def pair(self, color, depth, ros_ns, monotonic_sec):
        stamps = [message.header.stamp.to_nsec() for message in (color, depth)]
        rows = {}
        with self.lock:
            for stream, message, stamp in zip(('color', 'depth'), (color, depth), stamps):
                found = self.records[stream].get((id(message), stamp))
                rows[stream] = None if found is None else dict(found)
        for row in rows.values():
            if row is not None:
                row['until_exact_pair_callback_sec'] = max(0., monotonic_sec-row['arrival_monotonic_sec'])
        both = all(row is not None for row in rows.values())
        return dict(raw_arrivals=rows,
                    exact_pair_callback_ros_ns=ros_ns,
                    exact_pair_callback_monotonic_sec=monotonic_sec,
                    exact_pair_source_age_sec=(ros_ns-stamps[0])*1e-9,
                    raw_arrival_difference_sec=(abs(rows['color']['arrival_monotonic_sec']-
                        rows['depth']['arrival_monotonic_sec']) if both else None),
                    after_latest_raw_arrival_sec=(max(0., monotonic_sec-max(
                        row['arrival_monotonic_sec'] for row in rows.values())) if both else None))

    def snapshot(self, ros_ns, monotonic_sec):
        with self.lock:
            latest = {stream: None if row is None else dict(row) for stream, row in self.latest.items()}
            sizes = {stream: len(rows) for stream, rows in self.records.items()}
        for row in latest.values():
            if row is not None:
                row['source_age_at_status_sec'] = (ros_ns-row['source_stamp_ns'])*1e-9
                row['since_arrival_sec'] = max(0., monotonic_sec-row['arrival_monotonic_sec'])
        return dict(schema_version=1, latest_raw_arrivals=latest,
                    cached_arrivals=sizes, cache_capacity_per_stream=self.capacity,
                    scope='subscriber_filter_arrival_before_exact_synchronizer_not_socket_or_exposure')


class UnknownTabletopPerception(SupplementMeasurementAdmissionMixin,
                               ContactStationProducerMixin, StationComputeWaitProducerMixin):
    def __init__(self):
        cfg = rospy.get_param('/unknown_perception', {})
        camera = rospy.get_param('/camera', {})
        self._source_sampling_topology = SourceSamplingTopologyCache()
        self.config = Config(**{f.name: cfg[f.name] for f in fields(Config) if f.name in cfg})
        self.tracker = TargetTracker(self.config)
        self.acquisition = InitialAcquisition(self.config)
        self.base_frame = str(cfg.get('base_frame', 'base_link'))
        self.camera_frame = str(cfg.get('camera_frame', camera.get('frame_id', 'camera_link')))
        self.max_age = float(cfg.get('max_frame_age_sec', 0.8))
        # Accepted SDK feedback can have a brief gap during command dispatch.
        # Keep waiting for exact bracketing samples inside the original frame age.
        self.tf_wait = float(cfg.get('tf_wait_sec', 0.4))
        self.minimum_quality = float(cfg.get('minimum_geometry_quality', 0.5))
        self.rgb_instance_refinement = bool(cfg.get('rgb_instance_refinement', True))
        self.bridge = CvBridge()
        self.lock = threading.RLock()
        # Image ingress must not wait for GrabCut, geometry or ROS publishing.
        # Keep only the newest exact pair while a previous pair is processed.
        self.input_lock = threading.RLock()
        self._frame_arrivals = RgbdIngressTiming()
        self._initialize_rgb_interest(cfg)
        self.mode, self.generation, self.selection_ns = 'carton', -1, 0
        self.latest = None
        self._latest_frame_timing = None
        self._processing_timing = None
        self.last_consumed_ns = 0
        self.anchor_source_stamp_ns = 0
        self.first_loss = None
        self.last_good_source_stamp_ns = 0
        self.last_receive_wall = 0.0
        self.last_shape = (int(camera.get('height', 480)), int(camera.get('width', 640)))
        self.buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.listener = tf2_ros.TransformListener(self.buffer)
        prefix = '/perception/unknown/'
        self.pubs = {
            'object': rospy.Publisher(prefix+'object', ObjectPose, queue_size=2),
            'object_mask': rospy.Publisher(prefix+'object_mask', Image, queue_size=2),
            'object_pose_camera': rospy.Publisher(prefix+'object_pose_camera', PoseStamped, queue_size=2),
            'object_pose_base': rospy.Publisher(prefix+'object_pose_base', PoseStamped, queue_size=2),
            'object_detected': rospy.Publisher(prefix+'object_detected', Bool, queue_size=2),
            'raw_object_detected': rospy.Publisher(prefix+'raw_object_detected', Bool, queue_size=2),
            'detector_status': rospy.Publisher(prefix+'detector_status', String, queue_size=2, latch=True),
            'rgb_reference_status': rospy.Publisher('/perception/rgb_acquisition/reference_status',
                String, queue_size=2, latch=True),
        }
        self._observation_measurement = None
        self._setup_contact_station_io()
        self._observation_measurement_status = None
        self._observation_measurement_seen = deque(maxlen=32)
        self._observation_measurement_ack_pub = rospy.Publisher(MEASUREMENT_ACK_TOPIC, String,
            queue_size=8, latch=True)
        self._observation_measurement_sub = rospy.Subscriber(MEASUREMENT_REQUEST_TOPIC, String,
            self._observation_measurement_request_cb, queue_size=8)
        self._station_compute_phase_sub = rospy.Subscriber('/grasp/near_field_phase', NearFieldPlanningPhase,
            self._station_compute_phase_cb, queue_size=4)
        self._station_compute_state_sub = rospy.Subscriber('/grasp/state', GraspState,
            self._station_compute_state_cb, queue_size=4)
        self.selection_sub = rospy.Subscriber('/grasp_mode/selection', String, self.selection, queue_size=1)
        self.rgb_interest_joint_sub = rospy.Subscriber('/alicia_d/accepted_joint_states', JointState,
            self._rgb_interest_joint_cb, queue_size=50)
        self.rgb_interest_epoch_sub = rospy.Subscriber('/alicia_d/control_reference_epoch', Header,
            self._rgb_interest_epoch_cb, queue_size=2)
        self.rgb_reference_sub = rospy.Subscriber('/supervisor/rgb_acquisition/reference_request', String,
            self._rgb_reference_request_cb, queue_size=2)
        self.rgb_acquisition_sub = rospy.Subscriber('/supervisor/rgb_acquisition/status', String,
            self._rgb_acquisition_status_cb, queue_size=2)
        self.color_sub = message_filters.Subscriber(camera.get('color_topic', '/supervisor/camera/color/image_raw'), Image, queue_size=2, buff_size=2**24)
        self.depth_sub = message_filters.Subscriber(camera.get('depth_topic', '/supervisor/camera/depth/image_raw'), Image, queue_size=2, buff_size=2**24)
        # SimpleFilter calls callbacks in registration order. Record each raw
        # arrival before TimeSynchronizer.add can queue or match that message.
        self.color_sub.registerCallback(self._raw_frame_arrival, 'color')
        self.depth_sub.registerCallback(self._raw_frame_arrival, 'depth')
        self.sync = message_filters.TimeSynchronizer([self.color_sub, self.depth_sub], 8)
        self.sync.registerCallback(self.frame)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(0.5, float(cfg.get('detect_hz', 5.0)))), self.process)
        self.status('inactive', 'waiting_for_unknown_selection')

    def selection(self, message):
        try:
            selection = json.loads(message.data)
            mode = selection['mode']
            generation, stamp = int(selection['generation']), int(selection['stamp_ns'])
            if mode not in ('carton', 'unknown') or generation < 0 or stamp <= 0:
                raise ValueError('invalid selection')
        except (KeyError, TypeError, ValueError) as exc:
            rospy.logwarn('Unknown perception ignored invalid selection: %s', exc)
            return
        with self.lock:
            if generation < self.generation or (generation == self.generation and mode != self.mode):
                return
            if (generation, mode) == (self.generation, self.mode):
                return
            self._supplement_admission_cancel('SUPPLEMENT_MEASUREMENT_SELECTION_CHANGED')
            with self.input_lock:
                self.mode, self.generation, self.selection_ns = mode, generation, stamp
                self.latest, self.last_consumed_ns = None, 0
                self._latest_frame_timing = None
                self._processing_timing = None
                self.last_receive_wall = time.monotonic()
            self.anchor_source_stamp_ns = 0
            self.first_loss = None
            self._station_compute_registration = None
            self._station_compute_frame = None
            self.last_good_source_stamp_ns = 0
            self._locked_rgb_hold_reference = None
            self._locked_rgb_hold_status = None
            self._locked_source_pending = None
            self._locked_source_status = None
            self._metric_ambiguity_pending = None
            self._metric_ambiguity_status = None
            self._observation_measurement = None
            self._observation_measurement_status = None
            self._contact_station_prepared = None
            self._contact_station_status = None
            self._contact_spatial_context = None
            self.tracker.reset()
            self.acquisition.reset()
            self._clear_rgb_acquisition('selection_changed')
            self._clear_rgb_interest('selection_changed')
            self.unavailable('waiting_for_exact_rgbd' if mode == 'unknown' else 'carton_selected',
                             rospy.Time(stamp // 1000000000, stamp % 1000000000), lose=False)

    def _initialize_rgb_interest(self, cfg):
        self._rgb_interest_tracker = RgbObservationSetTracker()
        self._rgb_interest_policy = CenterSearchPolicy(
            (.5, .5), radius_fraction=self.config.center_radius_fraction)
        self._rgb_interest_feedback = deque(maxlen=128)
        self._rgb_interest_epoch = None
        self._rgb_interest_version = 0
        self._rgb_interest_invalidated = False
        self._rgb_interest_pose_anchor = None
        self._rgb_interest_last_attempt = float('-inf')
        self._rgb_interest_model_cache = None
        self._rgb_interest_config = deepcopy(cfg)
        self._rgb_observation_interest = empty_observation_interest('waiting_for_selection')
        # Keep two ready sources for the asynchronous freeze handshake. This is
        # bounded RAM, not an image subscription queue or a disk experiment log.
        self._rgb_reference_cache = deque(maxlen=2)
        # Actual admitted RGB histories, not repeated copies of the freeze frame.
        # Cache rows share these immutable masks; storage remains bounded.
        self._rgb_initial_tracking_history = deque(maxlen=3)
        self._rgb_reference_request = None
        self._rgb_acquisition_active = None
        self._rgb_acquisition_version = 0
        self._rgb_handoff = RgbMeasuredHandoff(self.config)
        self._rgb_handoff_tracker = RgbObservationTracker()
        self._rgb_handoff_scope = None
        self._rgb_handoff_status = None
        self._rgb_set_handoff_trackers = []
        self._rgb_set_measured_member = None
        self._locked_rgb_hold_reference = None
        self._locked_rgb_hold_status = None
        self._locked_source_pending = None
        self._locked_source_status = None
        self._metric_ambiguity_pending = None
        self._metric_ambiguity_status = None

    def _clear_rgb_acquisition(self, reason):
        if not hasattr(self, '_rgb_reference_cache'):
            return
        with self.input_lock:
            self._rgb_reference_cache.clear()
            self._rgb_initial_tracking_history.clear()
            self._rgb_reference_request = None
            self._rgb_acquisition_active = None
            self._rgb_acquisition_version += 1
            self._rgb_handoff_status = dict(state='cleared', reason=str(reason))
        self._rgb_handoff.reset()
        self._rgb_handoff_tracker.reset()
        self._rgb_handoff_scope = None
        self._rgb_set_handoff_trackers = []
        self._rgb_set_measured_member = None

    def _rgb_reference_request_cb(self, message):
        # Ingress only: decode a bounded control record, never RGB/TF or I/O.
        try:
            if len(message.data) > 8192:
                return
            request = json.loads(message.data)
            if (request.get('version') != 1 or not isinstance(request.get('request_id'), str)
                    or not request['request_id'] or not isinstance(request.get('source_id'), str)):
                return
            with self.input_lock:
                if self._rgb_acquisition_active is not None:
                    return
                self._rgb_reference_request = request
        except (ValueError, TypeError):
            return

    def _rgb_acquisition_status_cb(self, message):
        try:
            if len(message.data) > 16384:
                return
            event = json.loads(message.data)
            with self.input_lock:
                active = self._rgb_acquisition_active
                if (active is None or event.get('request_id') != active['request_id']
                        or active['phase'] == 'revoked'):
                    return
                if event.get('phase') == 'preflight_rejected':
                    # Only an explicit no-submission receipt can retain the
                    # ORIGINAL frozen pixels; a phase string is insufficient.
                    if self._rgb_accept_preflight_rejection(active, event):
                        active['phase'] = 'preflight_rejected'
                    else:
                        active['phase'] = 'revoked'
                elif event.get('phase') in ('revoked', 'planning_rejected', 'cancelled',
                                          'expired', 'completed', 'manual', 'budget_exhausted', 'failed'):
                    # Leave the expensive/stateful reset to process(), not an
                    # image/feedback callback. New selections are never blocked.
                    active['phase'] = 'revoked'
                elif (event.get('reference_id') == active['reference_id']
                        and event.get('source_id') == active['source_id']
                        and event.get('phase') in ('reserved', 'executing', 'awaiting_measured_target')):
                    active['phase'] = event['phase']
                    active['observation_index'] = int(event.get('observation_index', 1))
                    if event['phase'] == 'awaiting_measured_target':
                        active['settled_ns'] = int(event.get('settled_ns', 0))
                        active['completed_observations'] = active['observation_index']
                else:
                    return
                self._rgb_acquisition_version += 1
        except (ValueError, TypeError):
            return

    def _rgb_accept_preflight_rejection(self, active, event):
        try:
            proof = validate_prefix_preflight_receipt(event,
                now_ns=rospy.Time.now().to_nsec(), monotonic_now=time.monotonic())
            if (active['phase'] not in ('frozen', 'reserved', 'preflight_rejected')
                    or active['completed_observations'] != 0 or active['observation_index'] != 1
                    or proof['request_id'] != active['request_id']
                    or proof['reference_id'] != active['reference_id']
                    or proof['source_id'] != active['source_id']
                    or parse_selection(proof['selection']) != active['selection']
                    or int(proof['epoch_ns']) != active['epoch_ns']
                    or int(proof['epoch_ns']) != self._rgb_interest_epoch
                    or int(proof['deadline_ns']) != active['deadline_ns']
                    or proof['max_observations'] != active['max_observations']
                    or int(proof['last_capture_source_stamp_ns']) < active['reference'].source_stamp_ns
                    or np.max(np.abs(np.asarray(proof['q_source'])-active['hold_q'])) > 1e-12):
                return False
            old = active.get('preflight_receipt')
            if old is not None:
                changing = {'attempts', 'failed_reservation_request_id', 'last_capture_source_stamp_ns'}
                if (any(proof.get(key) != value for key,value in old.items() if key not in changing)
                        or proof['attempts'] < old['attempts']
                        or (proof['attempts'] == old['attempts'] and proof != old)
                        or int(proof['last_capture_source_stamp_ns']) < int(old['last_capture_source_stamp_ns'])):
                    return False
            active['preflight_receipt'] = proof
            return True
        except (ValueError, TypeError, KeyError, AttributeError):
            return False

    def _service_rgb_reference_request(self):
        if not hasattr(self, '_rgb_reference_cache'):
            return
        with self.input_lock:
            request, self._rgb_reference_request = self._rgb_reference_request, None
            cache = tuple(self._rgb_reference_cache)
            active = self._rgb_acquisition_active
        now_ns = rospy.Time.now().to_nsec()
        if active is not None and (active['phase'] == 'revoked' or now_ns > active['deadline_ns']
                or time.monotonic() > active['deadline_monotonic']):
            self._clear_rgb_acquisition('rgb_acquisition_revoked_or_expired')
        if request is None:
            return
        response = dict(version=1, request_id=request.get('request_id'),
                        source_id=request.get('source_id'), source_stamp_ns=request.get('source_stamp_ns'))
        try:
            source_ns = int(request['source_stamp_ns']); deadline = int(request['deadline_ns'])
            if not now_ns < deadline <= now_ns+120_000_000_000:
                raise ValueError('reference_request_deadline_invalid')
            if type(request.get('max_observations')) is not int or not 1 <= request['max_observations'] <= 3:
                raise ValueError('reference_request_budget_invalid')
            matches = [row for row in cache if row['frame'].source_stamp_ns == source_ns]
            if len(matches) != 1:
                raise ValueError('reference_source_not_retained')
            row = matches[0]; frame = row['frame']
            require_fresh_source(source_ns, now_ns, min(.8, self.max_age))
            with self.input_lock:
                epoch = self._rgb_interest_epoch
                input_version = self._rgb_interest_version
                newest = max(self._rgb_interest_feedback, key=lambda item:item[0], default=None)
            if (self.mode != 'unknown' or self.tracker.anchor is not None or self.tracker.lost
                    or frame.generation != self.generation or frame.epoch_ns != epoch
                    or frame.selection_stamp_ns != self.selection_ns
                    or request['source_id'] != frame.source_id
                    or request.get('selection') != row['selection']
                    or newest is None or np.max(np.abs(newest[1]-np.asarray(frame.joints_rad))) > .004
                    or frame.source_id != digest(self._rgb_interest_source_context(
                        rospy.Time(source_ns//1000000000, source_ns%1000000000)))):
                raise ValueError('reference_context_changed')
            for key in ('region_mask_sha256', 'source_rgb_sha256', 'source_depth_sha256',
                        'observation_set_sha256'):
                if request.get(key) != row[key]:
                    raise ValueError('reference_exact_pixels_mismatch:'+key)
            if len(row['regions']) > 1:
                reference = freeze_rgb_interest_set(frame, row['regions'], row['volumes'],
                    support_ring_outer_radius_px=row['support_ring_outer_radius_px'])
            else:
                reference = freeze_rgb_interest(frame, row['region'], row['volumes'][0],
                    support_ring_outer_radius_px=row['support_ring_outer_radius_px'])
            reference_id = digest({key:request[key] for key in ('request_id', 'source_id',
                'source_stamp_ns', 'region_mask_sha256', 'source_rgb_sha256', 'source_depth_sha256',
                'observation_set_sha256')})
            inherited, history_receipt = self._inherit_initial_rgb_history(row)
            # Recheck after CPU work before acknowledging a stored reference.
            require_fresh_source(source_ns, rospy.Time.now().to_nsec(), min(.8, self.max_age))
            with self.input_lock:
                if (epoch != self._rgb_interest_epoch or self.generation != frame.generation
                        or input_version != self._rgb_interest_version):
                    raise ValueError('reference_changed_during_freeze')
                self._rgb_acquisition_active = dict(request_id=request['request_id'],
                    source_id=frame.source_id, reference_id=reference_id, reference=reference,
                    selection=parse_selection(row['selection']), epoch_ns=frame.epoch_ns,
                    reference_set_sha256=row['observation_set_sha256'],
                    phase='frozen', deadline_ns=deadline, max_observations=request['max_observations'],
                    deadline_monotonic=time.monotonic()+(deadline-now_ns)*1e-9,
                    observation_index=1, settled_ns=0, completed_observations=0,
                    hold_q=frame.joints_rad, last_deficit_code='associated_region_lacks_measured_3d',
                    initial_rgb_history_receipt=history_receipt)
                self._rgb_acquisition_version += 1
                self._rgb_reference_cache.clear()
                self._rgb_initial_tracking_history.clear()
            self._rgb_handoff.reset(); self._rgb_handoff_tracker.reset()
            self._rgb_set_handoff_trackers = []
            self._rgb_set_measured_member = None
            self._rgb_handoff_scope = None
            if inherited:
                if isinstance(reference, RgbInterestSetReference):
                    self._rgb_set_handoff_trackers = inherited
                else:
                    self._rgb_handoff_tracker = inherited[0]
                # Only the unmoved, first planning view owns this history.
                # A settled movement increments completed_observations, and
                # the normal scope reset then requires three new sources.
                self._rgb_handoff_scope = (reference_id, 0)
            self.acquisition.reset()
            response.update(status='frozen', reference_id=reference_id,
                            observation_set_sha256=row['observation_set_sha256'])
        except (ValueError, KeyError, TypeError, tf2_ros.LookupException,
                tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            response.update(status='rejected', reason=str(exc))
        self.pubs['rgb_reference_status'].publish(String(data=json.dumps(response, sort_keys=True)))

    @staticmethod
    def _rgb_tracking_evidence_digest(evidence):
        return digest(dict(source_id=evidence['source_id'], generation=evidence['generation'],
            epoch_ns=evidence['epoch_ns'], selection_stamp_ns=evidence['selection_stamp_ns'],
            source_stamp_ns=evidence['source_stamp_ns'], q_source=evidence['q_source'],
            source_rgb_sha256=evidence['source_rgb_sha256'], source_depth_sha256=evidence['source_depth_sha256'],
            members=observation_set_metadata(evidence['regions'])))

    def _prepare_initial_rgb_history(self, track, regions, source_id, epoch, q, rgb, raw_depth):
        """Copy/hash bounded real evidence outside the ROS ingress lock."""
        frozen = []
        for region in regions:
            mask = np.array(region.mask, copy=True)
            mask.setflags(write=False)
            frozen.append(replace(region, mask=mask))
        if not frozen:
            return None
        evidence = dict(source_id=source_id, generation=self.generation, epoch_ns=epoch,
            selection_stamp_ns=self.selection_ns, source_stamp_ns=track.source_stamps_ns[-1],
            q_source=tuple(float(v) for v in q), regions=tuple(frozen),
            source_rgb_sha256=hashlib.sha256(np.ascontiguousarray(rgb).tobytes()).hexdigest(),
            source_depth_sha256=hashlib.sha256(np.ascontiguousarray(raw_depth).tobytes()).hexdigest())
        evidence['evidence_sha256'] = self._rgb_tracking_evidence_digest(evidence)
        return evidence

    def _retain_initial_rgb_history(self, track, evidence):
        """Keep only sources still admitted by the initial set tracker."""
        if evidence is None:
            self._rgb_initial_tracking_history.clear()
            return
        previous = [old for old in self._rgb_initial_tracking_history
                    if old['source_stamp_ns'] in track.source_stamps_ns[:-1]]
        self._rgb_initial_tracking_history.clear()
        self._rgb_initial_tracking_history.extend(previous)
        self._rgb_initial_tracking_history.append(evidence)

    def _inherit_initial_rgb_history(self, row):
        """Replay real initial masks through unchanged per-member RGB gates.

        This never seeds measured acquisition. Missing or altered evidence falls
        back to normal fresh confirmation; a new source still must uniquely
        associate to the frozen reference before these trackers are consulted.
        """
        receipt = dict(inherited=False, reason='initial_rgb_history_unavailable',
                       source_stamps_ns=[], evidence_sha256=[])
        try:
            history = tuple(row.get('tracking_history', ()))
            frame, regions = row['frame'], row['regions']
            expected = tuple(row.get('tracking_source_stamps_ns', ()))
            if len(history) != 3 or len(expected) != 3:
                return [], receipt
            if (tuple(item['source_stamp_ns'] for item in history) != expected
                    or expected[-1] != frame.source_stamp_ns
                    or any(type(ns) is not int for ns in expected)
                    or any(not 0 < b-a <= 1_000_000_000 for a,b in zip(expected,expected[1:]))):
                raise ValueError('initial_rgb_history_stamp_mismatch')
            if (history[-1]['source_rgb_sha256'] != row['source_rgb_sha256']
                    or history[-1]['source_depth_sha256'] != row['source_depth_sha256']
                    or observation_set_digest(history[-1]['regions']) != row['observation_set_sha256']):
                raise ValueError('initial_rgb_history_freeze_pixels_mismatch')
            trackers = [RgbObservationTracker() for _ in regions]
            reports = [None for _ in regions]
            for evidence in history:
                if (evidence['evidence_sha256'] != self._rgb_tracking_evidence_digest(evidence)
                        or any(evidence[name] != getattr(frame, name) for name in (
                            'source_id', 'generation', 'epoch_ns', 'selection_stamp_ns'))
                        or evidence['source_stamp_ns'] <= max(frame.epoch_ns, frame.selection_stamp_ns)
                        or len(evidence['regions']) != len(regions)):
                    raise ValueError('initial_rgb_history_binding_mismatch')
                q = np.asarray(evidence['q_source'])
                if q.shape != (6,) or not np.all(np.isfinite(q)) or np.max(np.abs(q-frame.joints_rad)) > .004:
                    raise ValueError('initial_rgb_history_pose_mismatch')
                assignment = _unique_assignment(regions, evidence['regions'], .70,
                    .015*min(frame.bgr.shape[:2]))
                if assignment is None:
                    raise ValueError('initial_rgb_history_member_mismatch')
                for member, (tracker, index) in enumerate(zip(trackers, assignment)):
                    reports[member] = tracker.observe(RgbObservationRegions((evidence['regions'][index],),
                        0, 'original_confirmed_initial_rgb_history', frame.bgr.shape[:2], 1),
                        source_id=frame.source_id, generation=frame.generation,
                        stamp_ns=evidence['source_stamp_ns'], joints_rad=q)
            if not all(report.confirmed for report in reports):
                raise ValueError('initial_rgb_history_reconfirmation_failed')
            receipt.update(inherited=True, reason='original_same_pose_three_source_rgb_history',
                source_stamps_ns=list(expected), evidence_sha256=[item['evidence_sha256'] for item in history])
            return trackers, receipt
        except (KeyError, TypeError, ValueError) as exc:
            receipt['reason'] = str(exc)
            return [], receipt

    def _clear_rgb_interest(self, reason, source_stamp_ns=None):
        if not hasattr(self, '_rgb_interest_tracker'):
            return
        self._rgb_interest_tracker.reset()
        with self.input_lock:
            self._rgb_initial_tracking_history.clear()
            self._rgb_interest_version += 1
            self._rgb_interest_invalidated = False
            self._rgb_interest_pose_anchor = None
            if hasattr(self, '_rgb_reference_cache'):
                self._rgb_reference_cache.clear()
            self._rgb_observation_interest = empty_observation_interest(
                reason, source_stamp_ns=source_stamp_ns)

    def _invalidate_rgb_interest_input(self, reason):
        """Called under input_lock; never run image work in a ROS callback."""
        self._rgb_interest_version += 1
        self._rgb_interest_invalidated = True
        self._rgb_interest_pose_anchor = None
        self._rgb_initial_tracking_history.clear()
        active = getattr(self, '_rgb_acquisition_active', None)
        if (active is not None and active['phase'] in ('frozen', 'reserved', 'preflight_rejected')
                and active['completed_observations'] == 0
                and active.get('initial_rgb_history_receipt', {}).get('inherited')):
            # Even an out-and-back measured pose excursion retires inheritance.
            # The processing worker resets history; the callback does no image
            # work and also invalidates any concurrently computed handoff.
            active['initial_rgb_history_invalidated'] = True
            self._rgb_acquisition_version += 1
        if hasattr(self, '_rgb_reference_cache'):
            self._rgb_reference_cache.clear()
        self._rgb_observation_interest = empty_observation_interest(reason)

    def _rgb_interest_epoch_cb(self, message):
        stamp_ns = message.stamp.to_nsec()
        if message.frame_id != 'sdk_reference_epoch' or stamp_ns <= 0:
            return
        with self.input_lock:
            if stamp_ns != self._rgb_interest_epoch:
                self._rgb_interest_epoch = stamp_ns
                self._contact_spatial_context = None
                measurement = getattr(self, '_observation_measurement', None)
                if measurement is not None:
                    measurement['invalidated_reason'] = 'control_reference_epoch_changed'
                if getattr(self, '_locked_source_pending', None) is not None:
                    self._locked_source_pending['invalidated_reason'] = 'control_reference_epoch_changed'
                if getattr(self, '_metric_ambiguity_pending', None) is not None:
                    self._metric_ambiguity_pending['invalidated_reason'] = 'control_reference_epoch_changed'
                self._locked_rgb_hold_reference = None
                self._rgb_interest_feedback.clear()
                self._invalidate_rgb_interest_input('control_reference_epoch_changed')
                if getattr(self, '_rgb_acquisition_active', None) is not None:
                    self._rgb_acquisition_active['phase'] = 'revoked'
                    self._rgb_acquisition_version += 1
                if hasattr(self, '_rgb_reference_cache'):
                    self._rgb_reference_cache.clear()

    def _rgb_interest_joint_cb(self, message):
        if message.header.frame_id != 'sdk_measured':
            return
        if len(message.name) != len(message.position) or len(message.name) > 32:
            return
        values = dict(zip(message.name, message.position))
        if len(values) != len(message.name) or any(name not in values for name in JOINT_NAMES):
            return
        q = np.array([values[name] for name in JOINT_NAMES], dtype=float)
        stamp_ns = message.header.stamp.to_nsec()
        if stamp_ns <= 0 or not np.all(np.isfinite(q)):
            return
        with self.input_lock:
            epoch = self._rgb_interest_epoch
            if epoch is None or stamp_ns < epoch:
                return
            self._rgb_interest_feedback.append((int(stamp_ns), q))
            measurement = getattr(self, '_observation_measurement', None)
            if (measurement is not None and measurement.get('arrival_q') is not None
                    and stamp_ns >= measurement['request']['arrival_stamp_ns']
                    and np.max(np.abs(q-measurement['arrival_q'])) > .004):
                measurement['invalidated_reason'] = 'movement_after_stationary_arrival'
            hold = getattr(self, '_locked_rgb_hold_reference', None)
            initial_station = (hold or {}).get('initial_station_current_support')
            if (initial_station is not None and stamp_ns >= initial_station['source_stamp_ns']
                    and np.max(np.abs(q-initial_station['q_source'])) > .004):
                # This origin never rolls forward with successful frames.
                initial_station['invalidated_reason'] = 'movement_after_initial_acquisition'
            arrival_retry = (hold or {}).get('observation_arrival_retry')
            if (arrival_retry is not None and stamp_ns >= arrival_retry['qualified_source_ns']
                    and np.max(np.abs(q-arrival_retry['arrival_q'])) > .004):
                self._station_requalification_feedback(stamp_ns, q)
                # Keep the original arrival station across successful frames;
                # small successive changes must not move this qualification.
                arrival_retry['invalidated_reason'] = 'movement_after_qualified_arrival'
                hold.pop('observation_arrival_retry', None)
            if (hold is not None and stamp_ns >= hold['source_stamp_ns']
                    and np.max(np.abs(q-hold['q_source'])) > .004):
                # A later return to the old pose cannot hide measured motion.
                hold['pose_invalidated'] = True
            anchor = self._rgb_interest_pose_anchor
            if (anchor is not None and np.max(np.abs(q - anchor)) >
                    self._rgb_interest_tracker.max_joint_delta_rad):
                self._invalidate_rgb_interest_input('measured_arm_pose_changed')

    def _measurement_pure_value(self, name, value, compute):
        memos = getattr(self, '_measurement_pure_memos', None)
        if memos is None:
            memos = self._measurement_pure_memos = {}
        return memos.setdefault(name, _ExactValueMemo()).get(value, compute)

    def _rgb_interest_source_context(self, stamp):
        # Cached ROS parameters still receive parameter updates; hash the model
        # only when its text changes. No URDF parse or disk I/O occurs here.
        get = rospy.get_param_cached
        with self._rgb_timing_component('context.model_parameter'):
            model = get('/robot_description', '')
            if not isinstance(model, str) or not model.strip():
                raise ValueError('observation_robot_model_unavailable')
            if self._rgb_interest_model_cache is None or self._rgb_interest_model_cache[0] != model:
                self._rgb_interest_model_cache = (model, hashlib.sha256(model.encode()).hexdigest())
        with self._rgb_timed_input_lock('context.input_lock_wait'):
            epoch = self._rgb_interest_epoch
        with self._rgb_timing_component('context.source_tf'):
            tool_camera = transform_matrix(self.buffer.lookup_transform(
                'tool0', self.camera_frame, stamp, rospy.Duration(.1)))
        with self._rgb_timing_component('context.latest_tf'):
            current_tool_camera = transform_matrix(self.buffer.lookup_transform(
                'tool0', self.camera_frame, rospy.Time(0), rospy.Duration(.1)))
        if not np.array_equal(tool_camera, current_tool_camera):
            raise ValueError('observation_camera_mount_changed')
        # rospy's cached parameter values can be mutable shared dictionaries.
        # Freeze every nested value now: an in-place parameter update must not
        # rewrite the context attributed to an already acquired image.
        with self._rgb_timing_component('context.parameter_snapshot'):
            raw = dict(epoch_ns=epoch, selection=get('/grasp_mode/selection'),
                camera=get('/camera'),
                perception=get('/unknown_perception', {}), handeye=get('/handeye', {}),
                model_sha256=self._rgb_interest_model_cache[1], tool_camera=tool_camera.tolist(),
                observation_view_parameters={key: get('/grasp_6d/remote/' + key, value)
                    for key, value in DEFAULT_OBSERVATION_VIEW_PARAMETERS.items()})
            def prepare(snapshot):
                snapshot = _binding_copy(snapshot)
                snapshot['selection'] = parse_selection(snapshot['selection'])
                snapshot['camera'] = camera_geometry_snapshot(snapshot['camera'])
                return snapshot
            context = self._measurement_pure_value('source_context', raw, prepare)
            # Cached results are private. Export an owned tree and preserve
            # the old tuple/list/scalar contract for all callers and hooks.
            return _binding_copy(context)

    def _rgb_bracketed_feedback(self, stamp_ns, epoch, *, source_freshness_check=None):
        """Await the trailing real sample without renewing the image lifetime.

        RGB-D and accepted joints arrive on different callbacks. Retain the
        original 200 ms interpolation limit and source age while briefly waiting
        for feedback newer than the image. No pose is extrapolated.
        """
        deadline = time.monotonic()+.12
        for attempt in range(7):
            if source_freshness_check is None:
                require_fresh_source(stamp_ns, rospy.Time.now().to_nsec(), min(self.max_age, .8))
            else:
                source_freshness_check()
            with self._rgb_timed_input_lock('feedback.input_lock_wait'):
                if epoch != self._rgb_interest_epoch:
                    raise ValueError('observation_control_reference_epoch_changed')
                feedback = tuple(self._rgb_interest_feedback)
            try:
                bracket = bracket_feedback(feedback, stamp_ns, max(epoch, self.selection_ns))
                return feedback, bracket
            except ValueError as exc:
                stamps = [row[0] for row in feedback]
                trailing_pending = (stamps and min(stamps) <= stamp_ns
                    and 0 < stamp_ns-max(stamps) <= 200_000_000)
                if (str(exc) != 'accepted_feedback_does_not_bracket_source'
                        or not trailing_pending or attempt == 6 or time.monotonic() >= deadline):
                    if str(exc) == 'accepted_feedback_does_not_bracket_source':
                        eligible = [(ns, np.asarray(values).tolist()) for ns, values in feedback
                                    if ns >= max(epoch, self.selection_ns)]
                        before = max((row for row in eligible if row[0] <= stamp_ns),
                                     key=lambda row: row[0], default=None)
                        after = min((row for row in eligible if row[0] >= stamp_ns),
                                    key=lambda row: row[0], default=None)
                        exc.accepted_feedback_evidence = dict(accepted_before=before, accepted_after=after,
                            span_ns=None if before is None or after is None else after[0]-before[0],
                            maximum_span_ns=200_000_000, bracket_qualified=False)
                    raise
                with self._rgb_timing_component('feedback.trailing_sample_wait'):
                    time.sleep(min(.02, max(0., deadline-time.monotonic())))

    def _update_initial_rgb_interest(self, source, depth, intrinsics, transform, segmentation, camera):
        """Expose bounded weak evidence only; do not mutate the target tracker."""
        if not hasattr(self, '_rgb_interest_tracker'):
            return
        if self.mode != 'unknown' or self.tracker.anchor is not None or self.tracker.lost:
            self._clear_rgb_interest('rgb_interest_requires_uninitialized_target')
            return
        color, _ = source
        stamp_ns = int(color.header.stamp.to_nsec())
        now = time.monotonic()
        with self.input_lock:
            self._rgb_observation_interest = empty_observation_interest(
                'rgb_observation_rate_limited', source_stamp_ns=stamp_ns)
        if now - self._rgb_interest_last_attempt < .5:
            return
        self._rgb_interest_last_attempt = now
        started = time.monotonic()
        try:
            with self.input_lock:
                if self._rgb_interest_invalidated:
                    self._rgb_interest_tracker.reset()
                    self._rgb_interest_invalidated = False
                version, epoch = self._rgb_interest_version, self._rgb_interest_epoch
            if epoch is None:
                raise ValueError('observation_control_reference_epoch_unavailable')
            require_fresh_source(stamp_ns, rospy.Time.now().to_nsec(), min(self.max_age, .8))
            feedback, (q, before, after) = self._rgb_bracketed_feedback(stamp_ns, epoch)
            context = self._rgb_interest_source_context(color.header.stamp)
            source_id = digest(context)
            selected = context['selection']
            if (selected['mode'] != 'unknown' or selected['strategy'] != 'observe_return'
                    or selected['generation'] != self.generation or selected['stamp_ns'] != self.selection_ns
                    or context['epoch_ns'] != epoch
                    or context['perception'] != self._rgb_interest_config
                    or context['camera'] != camera_geometry_snapshot(camera)):
                raise ValueError('observation_source_context_changed')
            if (camera.get('width') != depth.shape[1] or camera.get('height') != depth.shape[0]
                    or camera.get('frame_id') != self.camera_frame):
                raise ValueError('observation_camera_dimensions_changed')
            limits = effective_observation_view_limits(context['observation_view_parameters'])
            rgb = self.bridge.imgmsg_to_cv2(color, desired_encoding='bgr8')
            valid = (np.isfinite(depth) & (depth >= self.config.depth_min_m)
                     & (depth <= self.config.depth_max_m))
            proposals = propose_rgb_observation_regions(rgb, policy=self._rgb_interest_policy,
                depth_valid=valid, **_rgb_source_kwargs(camera, rgb.shape[:2]))
            track = self._rgb_interest_tracker.observe(proposals, policy=self._rgb_interest_policy, source_id=source_id,
                generation=self.generation, stamp_ns=stamp_ns, joints_rad=q)
            interest = empty_observation_interest(proposals.reason, source_stamp_ns=stamp_ns)
            interest.update(source_id=source_id, source_context=context, generation=self.generation,
                rgb_search_policy=asdict(self._rgb_interest_policy), rgb_search_basis='initial_center_policy',
                selection_stamp_ns=self.selection_ns, epoch_ns=epoch, input_version=version, accepted_before=before,
                accepted_after=after, q_source=q.tolist(), T_base_camera=transform.tolist(),
                rgb_candidate_count=proposals.total_candidate_count,
                rgb_candidates=[{item.name: getattr(candidate, item.name)
                    for item in fields(candidate) if item.name != 'mask'} for candidate in proposals.candidates],
                rgb_track={item.name: getattr(track, item.name)
                    for item in fields(track) if item.name != 'regions'},
                compact_mask_pixels=proposals.compact_mask_pixels,
                returned_mask_pixels=proposals.returned_mask_pixels,
                mask_storage_peak_pixels=proposals.mask_storage_peak_pixels)
            regions = tuple(sorted(track.regions, key=lambda r:(r.bbox_xywh,
                hashlib.sha256(r.mask.tobytes()).hexdigest())))
            if len(regions) > 1:
                interest.update(reason=track.reason, rgb_proposal_reason=proposals.reason)
            if regions:
                interest.update(region_mask_sha256=hashlib.sha256(regions[0].mask.tobytes()).hexdigest(),
                    rgb_interest_members=observation_set_metadata(regions),
                    observation_set_sha256=observation_set_digest(regions),
                    single_rgb_hypothesis=len(regions) == 1)
            if track.confirmed:
                metrics = segmentation.metrics
                if metrics.get('support_source') != 'current_ring_fit':
                    raise ValueError('observation_requires_same_frame_support_fit')
                coverage = measured_support_coverage(depth, intrinsics, transform,
                    segmentation.plane_base, self.config, metrics['support_ring_outer_radius_px'])
                provenance = SupportPlaneProvenance(source_stamp_ns=stamp_ns,
                    support_plane_base=tuple(segmentation.plane_base),
                    source_record_id='unknown_detector:%s:%d' % (source_id, stamp_ns),
                    inlier_count=int(round(metrics['support_sample_count'] * metrics['support_inlier_ratio'])),
                    inlier_ratio=metrics['support_inlier_ratio'], fit_rmse_m=metrics['support_residual_m'],
                    spatial_quadrants=coverage['spatial_quadrants'],
                    max_fit_rmse_m=min(.002, self.config.support_threshold_m),
                    min_inlier_ratio=max(.70, self.config.support_min_ratio),
                    min_inlier_count=max(120, self.config.support_min_points))
                fx, fy, cx, cy = intrinsics
                volume, volumes = self._rgb_interest_volumes(regions, depth.shape,
                    ((fx, 0., cx), (0., fy, cy), (0., 0., 1.)), transform, provenance,
                    (limits['minimum_depth_m'], limits['maximum_depth_m']))
                interest.update(interest_volume=asdict(volume), interest_volumes=[asdict(v) for v in volumes],
                    support_metrics=dict(metrics, **coverage))
                frame = RgbHandoffFrame(rgb, depth, np.asarray(volume.camera_matrix), transform,
                    provenance, source_id, self.generation, epoch, self.selection_ns,
                    stamp_ns, stamp_ns, rospy.Time.now().to_nsec(), tuple(feedback),
                    support_ring_outer_radius_px=metrics['support_ring_outer_radius_px'])
                # Hash native decoded depth, not converted metres. The gateway
                # independently captures these original same-stamp bytes.
                raw_depth = self.bridge.imgmsg_to_cv2(source[1], desired_encoding='passthrough')
                interest.update(source_rgb_sha256=hashlib.sha256(
                    np.ascontiguousarray(rgb).tobytes()).hexdigest(),
                    source_depth_sha256=hashlib.sha256(np.ascontiguousarray(raw_depth).tobytes()).hexdigest())
            if source_id != digest(self._rgb_interest_source_context(color.header.stamp)):
                raise ValueError('observation_source_context_changed')
            raw_depth = self.bridge.imgmsg_to_cv2(source[1], desired_encoding='passthrough')
            history_evidence = self._prepare_initial_rgb_history(
                track, regions, source_id, epoch, q, rgb, raw_depth)
            age = require_fresh_source(stamp_ns, rospy.Time.now().to_nsec(), min(self.max_age, .8))
            with self.input_lock:
                if version != self._rgb_interest_version or epoch != self._rgb_interest_epoch:
                    raise ValueError('observation_input_changed_during_processing')
                newest = max(self._rgb_interest_feedback, key=lambda row: row[0], default=None)
                if (newest is None or np.max(np.abs(newest[1] - q)) >
                        self._rgb_interest_tracker.max_joint_delta_rad):
                    raise ValueError('measured_arm_moved_after_source')
                if self._rgb_interest_pose_anchor is None:
                    self._rgb_interest_pose_anchor = q.copy()
                interest.update(observation_interest_ready=bool(track.confirmed
                    and interest.get('interest_volume')), source_age_at_status_sec=age,
                    processing_seconds=time.monotonic() - started)
                self._rgb_observation_interest = interest
                self._retain_initial_rgb_history(track, history_evidence)
                if interest['observation_interest_ready']:
                    self._rgb_reference_cache.append(dict(frame=frame, region=regions[0], regions=regions,
                        volume=volume, volumes=volumes, selection=deepcopy(selected),
                        tracking_history=tuple(self._rgb_initial_tracking_history),
                        tracking_source_stamps_ns=tuple(track.source_stamps_ns),
                        support_ring_outer_radius_px=metrics['support_ring_outer_radius_px'],
                        **{key:interest[key] for key in ('region_mask_sha256',
                            'source_rgb_sha256', 'source_depth_sha256', 'observation_set_sha256')}))
                else:
                    self._rgb_reference_cache.clear()
        except (ValueError, KeyError, TypeError, tf2_ros.LookupException,
                tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            self._clear_rgb_interest(str(exc), stamp_ns)

    def _rgb_interest_volumes(self, regions, shape, K, transform, provenance, working_range):
        """Keep each hypothesis independent; the outer box is only a view envelope."""
        def build(bbox):
            return build_rgb_observation_volume(bbox_xywh=bbox, image_shape=shape,
                camera_matrix=K, T_base_camera=transform,
                support_plane_base=provenance.support_plane_base, support_provenance=provenance,
                foreground_max_m=self.config.foreground_max_m, source_stamp_ns=provenance.source_stamp_ns,
                working_depth_range_m=working_range)
        volumes = tuple(build(region.bbox_xywh) for region in regions)
        boxes = np.asarray([region.bbox_xywh for region in regions], dtype=int)
        left = boxes[:, :2].min(axis=0)
        right = (boxes[:, :2]+boxes[:, 2:]).max(axis=0)
        envelope = build(tuple(int(v) for v in np.r_[left, right-left]))
        return envelope, volumes

    def _raw_frame_arrival(self, message, stream):
        arrived = time.monotonic()
        received_ns = rospy.Time.now().to_nsec()
        self._frame_arrivals.record(stream, message, received_ns, arrived)

    def _process_rgb_handoff(self, source, depth, intrinsics, transform, segmentation, camera):
        """Return associated (raw, refined), or withhold while RGB task owns lock.

        This runs only in the existing processing worker. It does not command
        the arm; the gateway's settled receipt enables new-frame evaluation.
        """
        with self.input_lock:
            if self._rgb_acquisition_active.pop('initial_rgb_history_invalidated', False):
                self._rgb_handoff.reset(); self._rgb_handoff_tracker.reset()
                self._rgb_set_handoff_trackers = []
                self._rgb_set_measured_member = None
                self._rgb_handoff_scope = None
            active = dict(self._rgb_acquisition_active)
            version = self._rgb_acquisition_version
            epoch = self._rgb_interest_epoch
            feedback = tuple(self._rgb_interest_feedback)
        color = source[0]; stamp_ns = color.header.stamp.to_nsec()
        status = dict(state='waiting', request_id=active['request_id'],
            reference_id=active['reference_id'], source_id=active['source_id'],
            source_stamp_ns=stamp_ns, observation_index=active['observation_index'],
            observation_only=True, motion_authorized=False, metric_geometry_authority=False,
            initial_rgb_history_receipt=active.get('initial_rgb_history_receipt'))
        self._rgb_handoff_status = status
        try:
            if (rospy.Time.now().to_nsec() > active['deadline_ns']
                    or time.monotonic() > active['deadline_monotonic']):
                self._clear_rgb_acquisition('rgb_acquisition_expired')
                raise ValueError('rgb_acquisition_expired')
            planning_refresh = active['phase'] in ('frozen', 'reserved', 'preflight_rejected')
            if (not planning_refresh and (active['phase'] != 'awaiting_measured_target'
                    or active['settled_ns'] <= 0 or stamp_ns <= active['settled_ns'])):
                status['reason'] = 'waiting_for_gateway_settled_receipt_and_new_frame'
                self.unavailable(status['reason'], color.header.stamp, source, lose=False)
                return None
            if not 1 <= active['observation_index'] <= active['max_observations']:
                self._clear_rgb_acquisition('observation_budget_exhausted')
                self.unavailable('observation_budget_exhausted', color.header.stamp, source, lose=False)
                return None
            context = self._rgb_interest_source_context(color.header.stamp)
            if (digest(context) != active['source_id'] or context['epoch_ns'] != epoch
                    or context['perception'] != self._rgb_interest_config
                    or context['camera'] != camera_geometry_snapshot(camera)):
                self._clear_rgb_acquisition('handoff_source_context_changed')
                raise ValueError('handoff_source_context_changed')
            metrics = segmentation.metrics
            if metrics.get('support_source') != 'current_ring_fit':
                raise ValueError('handoff_requires_independent_current_support')
            coverage = measured_support_coverage(depth, intrinsics, transform,
                segmentation.plane_base, self.config, metrics['support_ring_outer_radius_px'])
            provenance = SupportPlaneProvenance(stamp_ns, tuple(segmentation.plane_base),
                'rgb_handoff:%s:%d'%(active['reference_id'], stamp_ns),
                int(round(metrics['support_sample_count']*metrics['support_inlier_ratio'])),
                metrics['support_inlier_ratio'], metrics['support_residual_m'], coverage['spatial_quadrants'],
                max_fit_rmse_m=min(.002, self.config.support_threshold_m),
                min_inlier_ratio=max(.70, self.config.support_min_ratio),
                min_inlier_count=max(120, self.config.support_min_points))
            rgb = self.bridge.imgmsg_to_cv2(color, desired_encoding='bgr8')
            fx, fy, cx, cy = intrinsics
            K = np.array([[fx, 0., cx], [0., fy, cy], [0., 0., 1.]])
            feedback, _ = self._rgb_bracketed_feedback(stamp_ns, epoch)
            frame = RgbHandoffFrame(rgb, depth, K, transform, provenance, active['source_id'],
                self.generation, epoch, self.selection_ns, stamp_ns, stamp_ns,
                rospy.Time.now().to_nsec(), feedback,
                support_ring_outer_radius_px=metrics['support_ring_outer_radius_px'])
            if planning_refresh and np.max(np.abs(np.asarray(frame.joints_rad)-active['hold_q'])) > .004:
                self._clear_rgb_acquisition('reference_pose_changed_before_execution')
                raise ValueError('reference_pose_changed_before_execution')
            reference = active['reference']
            is_set = isinstance(reference, RgbInterestSetReference)
            associate = associate_rgb_interest_set if is_set else associate_rgb_interest
            association_index = max(1, active['completed_observations']) if planning_refresh else active['observation_index']
            audit = associate(reference, frame, (),
                observation_index=association_index)
            status['support_audit'] = dict(audit.support_audit)
            if audit.reason != 'no_cross_view_rgb_match':
                raise ValueError(audit.reason)
            projected = np.asarray(audit.projected_volume_pixels)
            policy = projected_rgb_search_policy(self._rgb_interest_policy, projected, depth.shape)
            proposals = propose_rgb_observation_regions(rgb, policy=policy,
                depth_valid=np.isfinite(depth)&(depth >= self.config.depth_min_m)&(depth <= self.config.depth_max_m),
                **_rgb_source_kwargs(camera, rgb.shape[:2]))
            result_association = associate(reference, frame, proposals,
                segmentation.candidates, observation_index=association_index)
            status.update(reason=result_association.reason, support_audit=dict(result_association.support_audit),
                projected_volume_pixels=result_association.projected_volume_pixels)
            if not result_association.associated:
                self._rgb_handoff.reset(); self._rgb_handoff_tracker.reset()
                self._rgb_set_handoff_trackers = []
                self._rgb_set_measured_member = None
                self.unavailable(result_association.reason, color.header.stamp, source, lose=False)
                return None
            associations = result_association.member_associations if is_set else (result_association,)
            regions = tuple(sorted((item.region for item in associations), key=lambda r:(r.bbox_xywh,
                hashlib.sha256(r.mask.tobytes()).hexdigest())))
            status['rgb_interest_member_associations'] = [dict(rgb_region_index=item.rgb_region_index,
                measured_candidate_index=item.measured_candidate_index, reason=item.reason,
                alternatives=item.alternatives) for item in associations]
            # Three increasing same-pose RGB sources are needed before asking
            # for another view. This is independent of measured acquisition.
            scope = (active['reference_id'], active['completed_observations'])
            if scope != self._rgb_handoff_scope:
                self._rgb_handoff_tracker.reset()
                self._rgb_set_handoff_trackers = []
                self._rgb_set_measured_member = None
                self._rgb_handoff_scope = scope
            if is_set:
                if len(self._rgb_set_handoff_trackers) != len(associations):
                    self._rgb_set_handoff_trackers = [RgbObservationTracker() for _ in associations]
                trackers = self._rgb_set_handoff_trackers
            else:
                trackers = (self._rgb_handoff_tracker,)
            tracks = tuple(tracker.observe(replace(proposals, selected_index=item.rgb_region_index,
                reason='cross_view_rgb_unique'), source_id=active['source_id'], generation=self.generation,
                stamp_ns=stamp_ns, joints_rad=frame.joints_rad)
                for tracker, item in zip(trackers, associations))
            track = tracks[0]
            all_confirmed = all(item.confirmed for item in tracks)
            status.update(rgb_associated_stable=all_confirmed,
                rgb_consistent_frames=min(item.consistent_frames for item in tracks),
                rgb_interest_member_count=len(regions), single_rgb_hypothesis=len(regions) == 1)
            # Retain the original members. More than one measured alternative
            # does not become a unique target because one happens to refine first.
            measured_members = [i for i, item in enumerate(associations) if item.measured_candidate is not None]
            member_index = measured_members[0] if len(measured_members) == 1 else None
            if member_index != self._rgb_set_measured_member:
                self._rgb_handoff.reset()
                self._rgb_set_measured_member = member_index
            association = associations[member_index if member_index is not None else 0]
            status.update(rgb_region_index=association.rgb_region_index,
                measured_candidate_index=association.measured_candidate_index if member_index is not None else None)
            raw = association.measured_candidate
            if len(measured_members) > 1:
                raw = None
            refined = None
            deficit = 'associated_region_lacks_measured_3d'
            deficit_code = deficit
            if not planning_refresh and raw is not None and raw.quality >= self.minimum_quality:
                try:
                    refined = (refine_candidate(rgb, depth, intrinsics, transform, raw, self.config,
                        **_rgb_source_kwargs(camera, rgb.shape[:2]))
                               if self.rgb_instance_refinement else raw)
                except ValueError as exc:
                    deficit = 'associated_rgb_refinement_rejected:'+str(exc)
                    deficit_code = 'associated_rgb_refinement_rejected'
            elif raw is not None:
                deficit = 'associated_geometry_quality_insufficient'
                deficit_code = deficit
            verdict = self._rgb_handoff.observe(association, raw, refined)
            if len(measured_members) > 1:
                deficit = 'multiple_original_rgb_members_have_measured_geometry'
                deficit_code = 'observation_identity_ambiguous'
            if planning_refresh:
                deficit = 'frozen_reference_fresh_scene'
                deficit_code = active['last_deficit_code']
            if refined is not None:
                deficit = verdict.reason
                deficit_code = 'initial_acquisition_pending'
            # No source/config/epoch or gateway transition can authorize an
            # in-flight result, even if it changed while OpenCV held the GIL.
            require_fresh_source(stamp_ns, rospy.Time.now().to_nsec(), min(self.max_age, .8))
            if (rospy.Time.now().to_nsec() > active['deadline_ns']
                    or time.monotonic() > active['deadline_monotonic']):
                self._clear_rgb_acquisition('rgb_acquisition_expired')
                raise ValueError('rgb_acquisition_expired')
            if digest(self._rgb_interest_source_context(color.header.stamp)) != active['source_id']:
                self._clear_rgb_acquisition('handoff_source_context_changed')
                raise ValueError('handoff_source_context_changed')
            with self.input_lock:
                if (version != self._rgb_acquisition_version or epoch != self._rgb_interest_epoch
                        or self._rgb_acquisition_active is None):
                    raise ValueError('handoff_changed_during_processing')
                newest = max(self._rgb_interest_feedback, key=lambda item:item[0], default=None)
                if newest is None or np.max(np.abs(newest[1]-np.asarray(frame.joints_rad))) > .004:
                    raise ValueError('handoff_arm_moved_after_source')
            measured_ready = verdict.ready and all_confirmed
            status.update(state='measured_ready' if measured_ready else 'observing',
                          reason=verdict.reason if measured_ready else deficit,
                          confirming_stamps_ns=verdict.confirming_stamps_ns)
            if measured_ready:
                status['metric_geometry_authority'] = True
                # Ownership is the original RGB association plus the existing
                # real three-frame gate. No centre-based chooser runs again.
                self._rgb_handoff_commit_version = version
                return raw, verdict.candidate
            if all_confirmed:
                volume, volumes = self._rgb_interest_volumes(regions, depth.shape, K, transform,
                    provenance, reference.volume.working_depth_range_m)
                raw_depth = self.bridge.imgmsg_to_cv2(source[1], desired_encoding='passthrough')
                evidence = dict(region_mask_sha256=hashlib.sha256(regions[0].mask.tobytes()).hexdigest(),
                    observation_set_sha256=observation_set_digest(regions),
                    source_rgb_sha256=hashlib.sha256(np.ascontiguousarray(rgb).tobytes()).hexdigest(),
                    source_depth_sha256=hashlib.sha256(np.ascontiguousarray(raw_depth).tobytes()).hexdigest())
                association_sha = digest(dict(reference_id=active['reference_id'],
                    source_stamp_ns=str(stamp_ns), evidence=evidence, support_audit=status['support_audit']))
                with self.input_lock:
                    input_version = self._rgb_interest_version
                interest = empty_observation_interest(deficit, source_stamp_ns=stamp_ns)
                interest.update(observation_interest_ready=True, source_id=active['source_id'],
                    rgb_search_policy=asdict(policy), rgb_search_basis='fixed_reference_volume_projection',
                    association_reference_volume=asdict(reference.volume),
                    association_reference_set_sha256=active['reference_set_sha256'],
                    association_reference_volumes=[asdict(member.volume) for member in reference.members]
                        if is_set else [asdict(reference.volume)],
                    association_projected_volume_pixels=result_association.projected_volume_pixels,
                    source_context=context, generation=self.generation, selection_stamp_ns=self.selection_ns,
                    epoch_ns=epoch, input_version=input_version, q_source=list(frame.joints_rad),
                    accepted_before=list(bracket_feedback(feedback, stamp_ns, max(epoch,self.selection_ns))[1]),
                    accepted_after=list(bracket_feedback(feedback, stamp_ns, max(epoch,self.selection_ns))[2]),
                    T_base_camera=transform.tolist(), interest_volume=asdict(volume),
                    interest_volumes=[asdict(v) for v in volumes],
                    rgb_interest_members=observation_set_metadata(regions),
                    single_rgb_hypothesis=len(regions) == 1,
                    support_metrics=dict(metrics, **coverage), rgb_candidate_count=proposals.total_candidate_count,
                    rgb_candidates=[{item.name:getattr(reg, item.name) for item in fields(reg) if item.name != 'mask'}
                        for reg in proposals.candidates],
                    rgb_track=dict({item.name:getattr(track,item.name) for item in fields(track) if item.name != 'region'},
                        confirmed=all_confirmed, consistent_frames=min(item.consistent_frames for item in tracks)),
                    reference_id=active['reference_id'], association_sha256=association_sha, **evidence)
                require_fresh_source(stamp_ns, rospy.Time.now().to_nsec(), min(self.max_age, .8))
                if digest(self._rgb_interest_source_context(color.header.stamp)) != active['source_id']:
                    self._clear_rgb_acquisition('handoff_source_context_changed')
                    raise ValueError('handoff_source_context_changed')
                with self.input_lock:
                    if (version != self._rgb_acquisition_version or epoch != self._rgb_interest_epoch
                            or self._rgb_acquisition_active is None
                            or rospy.Time.now().to_nsec() > active['deadline_ns']
                            or time.monotonic() > active['deadline_monotonic']):
                        raise ValueError('handoff_changed_before_supplement_status')
                    self._rgb_observation_interest = interest
                    self._rgb_interest_pose_anchor = np.asarray(frame.joints_rad).copy()
                    self._rgb_acquisition_active['hold_q'] = frame.joints_rad
                    self._rgb_acquisition_active['last_deficit_code'] = deficit_code
                if not planning_refresh or active['completed_observations'] > 0:
                    self.pubs['rgb_reference_status'].publish(String(data=json.dumps(dict(version=1,
                        status='supplement_ready', request_id=active['request_id'], reference_id=active['reference_id'],
                        source_id=active['source_id'], source_stamp_ns=str(stamp_ns), association_sha256=association_sha,
                        deficit_code=deficit_code, deficit_detail=deficit,
                        observation_index=active['completed_observations'], **evidence), sort_keys=True)))
            self.unavailable(deficit, color.header.stamp, source, lose=False, preserve_rgb_interest=all_confirmed)
            return None
        except (ValueError, KeyError, TypeError, tf2_ros.LookupException,
                tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            self._rgb_handoff.reset(); self._rgb_handoff_tracker.reset()
            self._rgb_set_handoff_trackers = []
            self._rgb_set_measured_member = None
            status.update(state='rejected', reason=str(exc))
            self.unavailable(str(exc), color.header.stamp, source, lose=False)
            return None

    def frame(self, color, depth):
        arrivals = getattr(self, '_frame_arrivals', None)
        pair_timing = None
        if arrivals is not None:
            pair_entered = time.monotonic()
            pair_ros_ns = rospy.Time.now().to_nsec()
            pair_timing = arrivals.pair(color, depth, pair_ros_ns, pair_entered)
        lock_attempt = time.monotonic()
        with self.input_lock:
            lock_acquired = time.monotonic()
            stamp_ns = color.header.stamp.to_nsec()
            if self.mode != 'unknown' or stamp_ns <= self.selection_ns:
                return
            # TimeSynchronizer is exact; retain a defensive integer check.
            if stamp_ns != depth.header.stamp.to_nsec() or stamp_ns <= self.last_consumed_ns:
                return
            if self.latest is not None and stamp_ns <= self.latest[0].header.stamp.to_nsec():
                return
            self.latest = (color, depth)
            self.last_receive_wall = time.monotonic()
            received_ns = rospy.Time.now().to_nsec()
            self._latest_frame_timing = dict(source_stamp_ns=stamp_ns,
                ingress_monotonic_sec=self.last_receive_wall,
                ingress_ros_ns=received_ns,
                ingress_source_age_sec=(received_ns-stamp_ns)*1e-9,
                color_seq=int(getattr(color.header, 'seq', 0)),
                depth_seq=int(getattr(depth.header, 'seq', 0)))
            if pair_timing is not None:
                pair_timing['input_lock_wait_sec'] = max(0., lock_acquired-lock_attempt)
                pair_timing['exact_pair_to_input_slot_sec'] = max(
                    0., self.last_receive_wall-pair_timing['exact_pair_callback_monotonic_sec'])
                self._latest_frame_timing.update(pair_timing)
                for stream in ('color', 'depth'):
                    arrival = pair_timing['raw_arrivals'][stream]
                    if arrival is not None:
                        self._latest_frame_timing[stream+'_seq'] = arrival['source_seq']

    def _begin_frame_timing(self, source, ingress, started):
        """Small numeric diagnostics only; none of these values admits a frame."""
        source_ns = source[0].header.stamp.to_nsec()
        ingress = dict(ingress or {})
        if ingress.get('source_stamp_ns') != source_ns:
            ingress = {}
        now_ns = rospy.Time.now().to_nsec()
        self._processing_timing = dict(ingress, schema_version=1,
            source_stamp_ns=source_ns,
            color_seq=int(ingress.get('color_seq', getattr(source[0].header, 'seq', 0))),
            depth_seq=int(ingress.get('depth_seq', getattr(source[1].header, 'seq', 0))),
            processing_start_ros_ns=now_ns,
            processing_start_monotonic_sec=started,
            processing_start_thread_cpu_sec=time.thread_time(),
            processing_start_source_age_sec=(now_ns-source_ns)*1e-9,
            ingress_to_processing_sec=(None if not ingress else
                max(0., started-ingress['ingress_monotonic_sec'])),
            max_frame_age_sec=self.max_age, stage='input_age_check', failed_stage=None,
            stage_wall_sec={}, stage_thread_cpu_sec={}, stage_end_elapsed_sec={},
            decode_components={}, rgb_components={}, segmentation_components={})
        self._processing_timing_thread_ident = threading.get_ident()
        self._processing_stage_started = (started, self._processing_timing['processing_start_thread_cpu_sec'])

    def _frame_timing_stage(self, stage):
        timing = self._processing_timing
        if timing is None:
            return  # A newer selection reset this in-flight diagnostic.
        wall, cpu = time.monotonic(), time.thread_time()
        old_wall, old_cpu = self._processing_stage_started
        old_stage = timing['stage']
        timing['stage_wall_sec'][old_stage] = max(0., wall-old_wall)
        timing['stage_thread_cpu_sec'][old_stage] = max(0., cpu-old_cpu)
        timing['stage_end_elapsed_sec'][old_stage] = max(
            0., wall-timing['processing_start_monotonic_sec'])
        timing['stage'] = stage
        self._processing_stage_started = (wall, cpu)

    @contextmanager
    def _decode_timing_component(self, name):
        """Measure existing decode work without changing its errors or authority."""
        timing = getattr(self, '_processing_timing', None)
        if timing is None:
            yield
            return
        wall, cpu = time.monotonic(), time.thread_time()
        completed = False
        try:
            yield
            completed = True
        finally:
            # A selection reset must not attach old work to a newer frame.
            if getattr(self, '_processing_timing', None) is timing:
                timing['decode_components'][name] = dict(
                    wall_sec=max(0., time.monotonic()-wall),
                    thread_cpu_sec=max(0., time.thread_time()-cpu),
                    completed=completed)

    @contextmanager
    def _rgb_timing_component(self, name):
        """Nested numeric evidence only; repeated calls retain separate records.

        Only the processing thread records against its frame. In particular,
        source-context calls made by ROS callbacks cannot pollute this timing.
        The optional finish callback measures lock acquisition without counting
        work performed while holding the lock. Nested durations overlap.
        """
        timing = getattr(self, '_processing_timing', None)
        if (timing is None or getattr(self, '_processing_timing_thread_ident', None)
                != threading.get_ident()):
            yield lambda: None
            return
        wall, cpu, process_cpu = time.monotonic(), time.thread_time(), time.process_time()
        stage = timing['stage']
        finished = False
        def finish(completed=True):
            nonlocal finished
            if finished:
                return
            finished = True
            if getattr(self, '_processing_timing', None) is timing:
                timing.setdefault('rgb_components', {}).setdefault(name, []).append(dict(
                    stage=stage,
                    start_elapsed_sec=max(0., wall-timing['processing_start_monotonic_sec']),
                    wall_sec=max(0., time.monotonic()-wall),
                    thread_cpu_sec=max(0., time.thread_time()-cpu),
                    process_cpu_sec=max(0., time.process_time()-process_cpu),
                    completed=completed))
        completed = False
        try:
            yield finish
            completed = True
        finally:
            finish(completed)

    @contextmanager
    def _rgb_timed_input_lock(self, name):
        # Retain the original context-manager acquire/release and exceptions.
        with self._rgb_timing_component(name) as acquired:
            with self.input_lock:
                acquired()
                yield

    @contextmanager
    def _segmentation_timing_component(self, name):
        """Observe existing segmentation work; GC deltas are process-wide counts.

        Counts neither attribute a collection to this thread nor measure its
        duration. No GC setting, source deadline, or acceptance rule changes.
        """
        timing = getattr(self, '_processing_timing', None)
        if (timing is None or getattr(self, '_processing_timing_thread_ident', None)
                != threading.get_ident()):
            yield lambda: None
            return
        wall, cpu, process_cpu = time.monotonic(), time.thread_time(), time.process_time()
        counts = gc.get_stats()
        stage = timing['stage']
        finished = False
        def finish(completed=True):
            nonlocal finished
            if finished:
                return
            finished = True
            if getattr(self, '_processing_timing', None) is timing:
                wall_end, cpu_end, process_end = time.monotonic(), time.thread_time(), time.process_time()
                current = gc.get_stats()
                timing.setdefault('segmentation_components', {}).setdefault(name, []).append(dict(
                    stage=stage,
                    start_elapsed_sec=max(0., wall-timing['processing_start_monotonic_sec']),
                    wall_sec=max(0., wall_end-wall), thread_cpu_sec=max(0., cpu_end-cpu),
                    process_cpu_sec=max(0., process_end-process_cpu), completed=completed,
                    gc_counts_scope='process_wide_counts_not_duration_or_thread_attribution',
                    gc_generation_deltas=[dict(generation=index, **{
                        key: after[key]-before[key]
                        for key in ('collections', 'collected', 'uncollectable')})
                        for index, (before, after) in enumerate(zip(counts, current))]))
        completed = False
        try:
            yield finish
            completed = True
        finally:
            finish(completed)

    @contextmanager
    def _segmentation_timed_lock(self):
        with self._segmentation_timing_component('station_lock_wait') as acquired:
            with self.lock:
                acquired()
                yield

    def _source_timing_status(self, source):
        timing = getattr(self, '_processing_timing', None)
        if timing is None or timing['source_stamp_ns'] != source[0].header.stamp.to_nsec():
            return None
        result = dict(timing)
        for key in ('stage_wall_sec', 'stage_thread_cpu_sec', 'stage_end_elapsed_sec'):
            result[key] = dict(timing[key])
        result['decode_components'] = {
            name: dict(component) for name, component in timing['decode_components'].items()}
        result['rgb_components'] = {name: [dict(component) for component in components]
            for name, components in timing.get('rgb_components', {}).items()}
        result['segmentation_components'] = deepcopy(timing.get('segmentation_components', {}))
        wall, cpu = time.monotonic(), time.thread_time()
        stage = timing['stage']
        old_wall, old_cpu = self._processing_stage_started
        result['stage_wall_sec'][stage] = max(0., wall-old_wall)
        result['stage_thread_cpu_sec'][stage] = max(0., cpu-old_cpu)
        result['stage_end_elapsed_sec'][stage] = max(
            0., wall-timing['processing_start_monotonic_sec'])
        now_ns = rospy.Time.now().to_nsec()
        result['processing_wall_sec'] = max(0., wall-timing['processing_start_monotonic_sec'])
        result['processing_thread_cpu_sec'] = max(0., cpu-timing['processing_start_thread_cpu_sec'])
        result['status_build_ros_ns'] = now_ns
        result['status_source_age_sec'] = (now_ns-timing['source_stamp_ns'])*1e-9
        with self.input_lock:
            pending = self.latest
            pending_ns = None if pending is None else pending[0].header.stamp.to_nsec()
            pending_ingress = getattr(self, '_latest_frame_timing', None)
            pending_wall = (pending_ingress.get('ingress_monotonic_sec')
                if pending_ingress is not None and pending_ingress.get('source_stamp_ns') == pending_ns
                else None)
        result['latest_pending_source_stamp_ns'] = pending_ns
        result['latest_pending_source_age_sec'] = (
            None if pending_ns is None else (now_ns-pending_ns)*1e-9)
        result['latest_pending_residence_sec'] = (
            None if pending_wall is None else max(0., wall-pending_wall))
        return result

    def status(self, state, detail, source=None, candidate=None, elapsed=None):
        payload = dict(mode='unknown', generation=self.generation,
                       selection_stamp_ns=self.selection_ns, state=state, detail=detail,
                       label='unknown_small', confidence_kind='geometry_quality',
                       semantic_probability=None, target_locked=self.tracker.anchor is not None,
                       target_lost=self.tracker.lost)
        payload['support_plane_consistency_policy'] = dict(
            support_max_local_shift_m=float(self.config.support_max_local_shift_m),
            support_local_shift_warning_m=float(self.config.support_local_shift_warning_m),
            support_max_normal_change_deg=float(self.config.support_max_normal_change_deg))
        with self.input_lock:
            interest = deepcopy(getattr(self, '_rgb_observation_interest', None))
        if interest is not None:
            payload['observation_interest'] = interest
        handoff_status = getattr(self, '_rgb_handoff_status', None)
        if handoff_status is not None:
            payload['rgb_acquisition_handoff'] = deepcopy(handoff_status)
        hold_status = getattr(self, '_locked_rgb_hold_status', None)
        if hold_status is not None:
            payload['rgb_identity_hold'] = deepcopy(hold_status)
        qualification = getattr(self, '_locked_source_status', None)
        if qualification is not None:
            payload['source_qualification'] = deepcopy(qualification)
        ambiguity = getattr(self, '_metric_ambiguity_status', None)
        if ambiguity is not None:
            payload['metric_ambiguity_hold'] = deepcopy(ambiguity)
        measurement = getattr(self, '_observation_measurement_status', None)
        if getattr(self, '_contact_station_status', None) is not None:
            payload['contact_station_retirement'] = deepcopy(self._contact_station_status)
        if measurement is not None:
            payload['observation_measurement'] = deepcopy(measurement)
        reference = getattr(self, '_locked_rgb_hold_reference', None)
        marker = (reference or {}).get('initial_station_current_support')
        if marker is not None and not marker.get('invalidated_reason'):
            template = marker.get('identity_template')
            if template is not None:
                payload['initial_station_identity_template'] = deepcopy(template['provenance'])
            if (state == 'ready' and source is not None
                    and reference['source_stamp_ns'] == source[0].header.stamp.to_nsec()
                    and self._initial_station_selection_matches()
                    and rospy.get_param_cached('/gui/joint_direct_mode', False) is not True
                    and self.first_loss is None and not self.tracker.lost):
                payload['initial_station_measurement'] = dict(
                    version=1, state='qualified', source_stamp_ns=reference['source_stamp_ns'],
                    station_source_stamp_ns=marker['source_stamp_ns'], epoch_ns=marker['epoch_ns'],
                    source_id=marker['source_id'], q_station_rad=marker['q_source'].tolist(),
                    refined_mask_sha256=reference['refined_mask_sha256'],
                    metric_geometry_authority=False, motion_authorized=False)
        if self.first_loss is not None:
            payload['first_loss'] = self.first_loss
        if self.tracker.anchor is not None:
            payload.update(anchor_source_stamp_ns=self.anchor_source_stamp_ns,
                           anchor_position_base=self.tracker.anchor.position_base.tolist(),
                           anchor_extent_m=self.tracker.anchor.extent_m.tolist(),
                           reference_object_height_m=getattr(self.tracker.anchor, 'metrics', {}).get('median_height_m'),
                           reference_support_plane_base=self.tracker.plane_base.tolist(),
                           reference_support_footprint_base=self.tracker.anchor.support_footprint_base.tolist())
        if source is not None:
            color, depth = source
            payload.update(source_stamp_ns=color.header.stamp.to_nsec(),
                           color_stamp_ns=color.header.stamp.to_nsec(),
                           depth_stamp_ns=depth.header.stamp.to_nsec(),
                           color_frame_id=color.header.frame_id,
                           depth_frame_id=depth.header.frame_id,
                           color_seq=int(color.header.seq), depth_seq=int(depth.header.seq),
                           exact_rgbd=True, tf_lookup_stamp_ns=color.header.stamp.to_nsec(),
                           projection_frame_convention='ros_camera_link')
            timing = self._source_timing_status(source)
            if timing is not None:
                payload['frame_timing'] = timing
                # mask.header shares the source Header; rospy publication can
                # change its sequence. Preserve the original ingress counters.
                payload['color_seq'] = timing['color_seq']
                payload['depth_seq'] = timing['depth_seq']
        arrivals = getattr(self, '_frame_arrivals', None)
        if source is None and arrivals is not None:
            # The watchdog has no source pair/frame_timing. Keep the latest two
            # raw arrivals visible in that case without publishing extra topics.
            payload['rgbd_ingress_timing'] = arrivals.snapshot(
                rospy.Time.now().to_nsec(), time.monotonic())
        if candidate is not None:
            payload.update(geometry_quality=candidate.quality,
                           geometry_metrics=candidate.metrics,
                           position_base=candidate.position_base.tolist())
        if elapsed is not None:
            payload['processing_seconds'] = float(elapsed)
        # Publish warning-band evidence on the existing recorded status topic.
        # Pending arrival frames have no publishable candidate yet; their
        # independently measured support is carried by the measurement audit.
        audits = []
        if candidate is not None:
            audits.append(('geometry_metrics', candidate.metrics, payload.get('source_stamp_ns')))
        for name in ('rgb_acquisition_handoff', 'rgb_identity_hold', 'source_qualification',
                     'metric_ambiguity_hold', 'observation_measurement'):
            record = payload.get(name) or {}
            audit = record.get('support_audit')
            if isinstance(audit, dict):
                audits.append((name, audit, audit.get('source_stamp_ns',
                    audit.get('current_support_source_stamp_ns', record.get('source_stamp_ns')))))
                for member in audit.get('support_member_audits', ()):
                    member_audit = member.get('audit')
                    if isinstance(member_audit, dict):
                        audits.append((name+'.member_'+str(member.get('member_index')), member_audit,
                            member_audit.get('current_support_source_stamp_ns', record.get('source_stamp_ns'))))
        warnings = []
        for kind, audit, stamp in audits:
            shift = audit.get('support_reference_max_distance_m')
            if (not isinstance(shift, (int, float)) or isinstance(shift, bool)
                    or not np.isfinite(shift) or shift < 0.):
                continue
            diagnostics = _support_offset_diagnostics(shift, self.config)
            if diagnostics['support_reference_distance_warning']:
                warnings.append(dict(diagnostics, audit_kind=kind, source_stamp_ns=stamp,
                    support_reference_max_distance_m=float(shift),
                    code='SUPPORT_PLANE_LOCAL_SHIFT_WARNING'))
        if warnings:
            payload['support_plane_warnings'] = warnings
            largest = max(warnings, key=lambda item: item['support_reference_max_distance_m'])
            rospy.logwarn_throttle(3., 'Unknown support offset warning generation=%s source=%s '
                'distance=%.6f m warning=%.6f m hard=%.6f m', self.generation,
                largest['source_stamp_ns'], largest['support_reference_max_distance_m'],
                largest['support_reference_warning_distance_m'], largest['support_reference_hard_distance_m'])
        self._station_compute_status(payload, source)
        self.pubs['detector_status'].publish(String(data=json.dumps(payload, sort_keys=True)))

    def _initial_visual_insufficiency(self, reason, source, depth, intrinsics,
                                     transform, segmentation, camera):
        """Partial depth evidence may seek observation without becoming a target."""
        self._frame_timing_stage('rgb_observation_interest')
        self._update_initial_rgb_interest(source, depth, intrinsics, transform, segmentation, camera)
        self.unavailable(reason, source[0].header.stamp, source, lose=False,
                         preserve_rgb_interest=True)

    def _prepare_locked_rgb_hold_reference(self, source, candidate, camera, tracker=None,
                                           *, source_transform=None):
        """Best-effort cache from a genuinely publishable metric RGB-D frame.

        This independent identity cache owns no geometry, plan or observation
        request. Failure to prepare it leaves the existing metric path intact.
        """
        if not hasattr(self, '_rgb_interest_feedback'):
            return None
        tracker = self.tracker if tracker is None else tracker
        try:
            stamp_ns = source[0].header.stamp.to_nsec()
            with self._rgb_timing_component('hold.context_initial'):
                context = self._rgb_interest_source_context(source[0].header.stamp)
            epoch = context['epoch_ns']
            if (context['perception'] != self._rgb_interest_config
                    or context['selection']['generation'] != self.generation
                    or context['selection']['stamp_ns'] != self.selection_ns):
                return None
            with self._rgb_timing_component('hold.feedback'):
                records, (q, before, after) = self._rgb_bracketed_feedback(stamp_ns, epoch)
            with self._rgb_timing_component('hold.bridge_conversion'):
                rgb = self.bridge.imgmsg_to_cv2(source[0], desired_encoding='bgr8')
                raw_depth = self.bridge.imgmsg_to_cv2(source[1], desired_encoding='passthrough')
            if (rgb.shape != (camera['height'], camera['width'], 3)
                    or raw_depth.shape != rgb.shape[:2]
                    or source[1].header.stamp.to_nsec() != stamp_ns):
                return None
            with self._rgb_timing_component('hold.mask_rgb_validity'):
                mask = _mask(candidate.mask, rgb.shape[:2])
                rgb_valid = rectification_rgb_valid_mask(camera, rgb.shape[:2])
                if rgb_valid is not None and np.any((mask > 0) & ~rgb_valid):
                    return None
            with self._rgb_timing_component('hold.appearance'):
                histogram, contour = _appearance(rgb, mask)
            def immutable(array):
                value = np.ascontiguousarray(array)
                return np.frombuffer(value.tobytes(), dtype=value.dtype).reshape(value.shape)
            with self._rgb_timing_component('hold.hash_copy_fingerprints'):
                reference_now = rospy.Time.now().to_nsec()
                reference = dict(source_stamp_ns=stamp_ns, source_id=digest(context),
                    _reference_monotonic=time.monotonic() + (
                        stamp_ns+round(min(self.max_age, .8)*1e9)-reference_now)*1e-9,
                    _reference_last_ros_ns=reference_now,
                    epoch_ns=epoch, generation=self.generation, selection_ns=self.selection_ns,
                    config_sha256=digest(asdict(self.config)), image_shape=rgb.shape[:2],
                    source_rgb_sha256=hashlib.sha256(np.ascontiguousarray(rgb).tobytes()).hexdigest(),
                    source_depth_sha256=hashlib.sha256(np.ascontiguousarray(raw_depth).tobytes()).hexdigest(),
                    refined_mask_sha256=hashlib.sha256(mask.tobytes()).hexdigest(),
                    bgr=immutable(rgb), mask=immutable(mask), histogram=immutable(histogram), contour=immutable(contour),
                    q_source=immutable(q), accepted_before=before, accepted_after=after,
                    anchor_fingerprint=_candidate_fingerprint(tracker.anchor),
                    last_fingerprint=_candidate_fingerprint(tracker.last),
                    reference_plane=immutable(tracker.plane_base),
                    reference_footprint=immutable(tracker.anchor.support_footprint_base),
                    pose_invalidated=False)
            # Keep the exact transform already used by this source's metric
            # process. A new lookup or latest depth cannot supply this cache.
            # Legacy callers without that provenance retain only RGB identity.
            if source_transform is not None:
                with self._rgb_timing_component('hold.measured_face_source'):
                    if (source[0].header.frame_id != self.camera_frame
                            or source[1].header.frame_id != self.camera_frame
                            or camera.get('align_depth_to_color') is not True):
                        return None
                    encoding = source[1].encoding
                    reference['measured_face_source'] = FrozenMeasuredFaceSource(
                        raw_depth, encoding,
                        1. if encoding == '32FC1' else camera['depth_scale'],
                        [float(camera[k]) for k in ('fx', 'fy', 'cx', 'cy')],
                        source_transform, tracker.plane_base, self.config,
                        stamp_ns, reference['source_id'], reference['source_rgb_sha256'],
                        reference['refined_mask_sha256'])
                    reference['measured_face_source_binding_sha256'] = (
                        reference['measured_face_source'].binding_sha256)
            require_fresh_source(stamp_ns, rospy.Time.now().to_nsec(), min(self.max_age, .8))
            with self._rgb_timing_component('hold.context_recheck'):
                if digest(self._rgb_interest_source_context(source[0].header.stamp)) != reference['source_id']:
                    return None
            with self._rgb_timed_input_lock('hold.input_lock_wait'):
                if epoch != self._rgb_interest_epoch:
                    return None
                relevant = [values for ns, values in self._rgb_interest_feedback if ns >= stamp_ns]
                if not relevant or any(np.max(np.abs(values-q)) > .004 for values in relevant):
                    return None
            return reference
        except (ValueError, KeyError, TypeError, AttributeError, cv2.error,
                tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None

    @staticmethod
    def _initial_station_template_provenance(reference):
        return dict(version=1, role='immutable_initial_acquired_rgb_identity',
            source_stamp_ns=reference['source_stamp_ns'], source_id=reference['source_id'],
            generation=reference['generation'], selection_stamp_ns=reference['selection_ns'],
            epoch_ns=reference['epoch_ns'], image_shape=list(reference['image_shape']),
            source_rgb_sha256=reference['source_rgb_sha256'],
            refined_mask_sha256=reference['refined_mask_sha256'])

    def _initial_station_identity_template(self, source, reference):
        """A fixed appearance baseline, never a fresh metric/source reference."""
        self._require_initial_station_support_reference(source, reference)
        marker = reference['initial_station_current_support']
        template = marker['identity_template']
        provenance = template['provenance']
        expected = dict(version=1, role='immutable_initial_acquired_rgb_identity',
            source_stamp_ns=marker['source_stamp_ns'], source_id=marker['source_id'],
            generation=self.generation, selection_stamp_ns=self.selection_ns,
            epoch_ns=marker['epoch_ns'], image_shape=list(reference['image_shape']),
            source_rgb_sha256=hashlib.sha256(template['bgr'].tobytes()).hexdigest(),
            refined_mask_sha256=hashlib.sha256(template['mask'].tobytes()).hexdigest())
        if (provenance != expected or marker['source_stamp_ns'] != self.anchor_source_stamp_ns
                or template['bgr'].shape != tuple(reference['image_shape']) + (3,)
                or template['mask'].shape != tuple(reference['image_shape'])
                or template['bgr'].flags.writeable or template['mask'].flags.writeable):
            raise ValueError('initial_station_identity_template_changed')
        return template

    def _initial_station_selection_matches(self):
        try:
            return self.mode == 'unknown' and parse_selection(
                rospy.get_param_cached('/grasp_mode/selection')) == dict(
                    mode='unknown', strategy='observe_return',
                    generation=self.generation, stamp_ns=self.selection_ns)
        except (ValueError, TypeError, KeyError):
            return False

    def _initial_station_support_reference(self, source, expected=None):
        """Current-table preference at the original measured, stationary pose.

        This supplies neither an RGB retry nor identity/geometry authority.
        Every new frame still needs the original independent support audit.
        """
        reference = getattr(self, '_locked_rgb_hold_reference', None)
        marker = (reference or {}).get('initial_station_current_support')
        if marker is None:
            return None
        if (getattr(self, '_observation_measurement', None) is not None
                or getattr(self, '_metric_ambiguity_pending', None) is not None):
            return None
        try:
            if (expected is not None and reference is not expected
                    or marker.get('invalidated_reason') or not self._initial_station_selection_matches()
                    or self.first_loss is not None
                    or rospy.get_param_cached('/gui/joint_direct_mode', False) is True
                    or reference['source_id'] != marker['source_id']
                    or reference['epoch_ns'] != marker['epoch_ns']
                    or reference['anchor_fingerprint'] != marker['anchor_fingerprint']
                    or not np.array_equal(reference['reference_plane'], marker['reference_plane'])):
                raise ValueError('initial_station_binding_changed')
            self._qualify_support_gap_source(source, reference)
            with self.input_lock:
                values = [q for ns, q in self._rgb_interest_feedback
                          if ns >= reference['source_stamp_ns']]
                if (self._locked_rgb_hold_reference is not reference
                        or reference.get('initial_station_current_support') is not marker
                        or marker.get('invalidated_reason') or not values
                        or np.max(np.abs(reference['q_source']-marker['q_source'])) > .004
                        or any(np.max(np.abs(q-marker['q_source'])) > .004 for q in values)):
                    raise ValueError('initial_station_pose_changed')
            return reference
        except (ValueError, KeyError, TypeError, AttributeError, tf2_ros.LookupException,
                tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            if str(exc) == 'accepted_feedback_does_not_bracket_source':
                evidence = getattr(exc, 'accepted_feedback_evidence', None)
                try:
                    self._validate_initial_feedback_gap(source, reference, evidence)
                except (ValueError, KeyError, TypeError, AttributeError):
                    pass
                else:
                    # Reject this source without permanently revoking its still
                    # fresh station. The outer process publishes false geometry
                    # and a separately typed, non-authorizing pending receipt.
                    raise InitialFeedbackBracketGap(reference, evidence) from exc
            marker['invalidated_reason'] = str(exc)
            timing = getattr(self, '_processing_timing', None)
            if timing is not None:
                timing['initial_station_invalidated_reason'] = str(exc)
            return None

    def _validate_initial_feedback_gap(self, source, reference, evidence):
        marker = reference['initial_station_current_support']
        pending = getattr(self, '_locked_source_pending', None)
        self._require_original_feedback_reference_clock(reference)
        if (self._locked_rgb_hold_reference is not reference or marker.get('invalidated_reason')
                or not self._initial_station_selection_matches() or self.first_loss is not None
                or self.tracker.lost or reference['pose_invalidated']
                or rospy.get_param_cached('/gui/joint_direct_mode', False) is True
                or getattr(self, '_observation_measurement', None) is not None
                or getattr(self, '_metric_ambiguity_pending', None) is not None
                or reference.get('observation_arrival_retry') is not None
                or (pending is not None and pending.get('kind') != FEEDBACK_GAP)
                or (self.generation, self.selection_ns, self._rgb_interest_epoch) !=
                   (reference['generation'], reference['selection_ns'], reference['epoch_ns'])
                or self.last_good_source_stamp_ns != reference['source_stamp_ns']
                or reference['source_id'] != marker['source_id']
                or reference['epoch_ns'] != marker['epoch_ns']
                or digest(asdict(self.config)) != reference['config_sha256']
                or _candidate_fingerprint(self.tracker.anchor) != reference['anchor_fingerprint']
                or _candidate_fingerprint(self.tracker.last) != reference['last_fingerprint']
                or marker['anchor_fingerprint'] != reference['anchor_fingerprint']
                or not np.array_equal(self.tracker.plane_base, reference['reference_plane'])
                or not np.array_equal(marker['reference_plane'], reference['reference_plane'])
                or digest(self._rgb_interest_source_context(source[0].header.stamp)) != reference['source_id']):
            raise ValueError('feedback_gap_station_changed')
        ns = source[0].header.stamp.to_nsec()
        if (source[1].header.stamp.to_nsec() != ns
                or pending is not None and ns <= pending['last_unqualified_source_stamp_ns']):
            raise ValueError('feedback_gap_source_changed')
        with self.input_lock:
            validate_gap_evidence(evidence, ns, reference['source_stamp_ns'], reference['epoch_ns'],
                marker['q_source'], rospy.Time.now().to_nsec(), min(self.max_age, .8))
            seen = [q for stamp, q in self._rgb_interest_feedback if stamp >= reference['source_stamp_ns']]
            if (not seen or self._locked_rgb_hold_reference is not reference
                    or self._rgb_interest_epoch != reference['epoch_ns'] or reference['pose_invalidated']
                    or np.max(np.abs(reference['q_source']-marker['q_source'])) > .004
                    or any(np.max(np.abs(q-marker['q_source'])) > .004 for q in seen)):
                raise ValueError('feedback_gap_measured_pose_changed')

    def _publish_initial_feedback_gap(self, source, error):
        with self.input_lock:
            reference = error.reference
            self._validate_initial_feedback_gap(source, reference, error.evidence)
            ns = source[0].header.stamp.to_nsec()
            pending = getattr(self, '_locked_source_pending', None)
            self._arrival_frame_completion = None  # Never use a 2x completion allowance for this wait.
            self._locked_source_pending = dict(kind=FEEDBACK_GAP,
                reference_source_stamp_ns=reference['source_stamp_ns'], source_id=reference['source_id'],
                first_unqualified_source_stamp_ns=ns if pending is None else pending['first_unqualified_source_stamp_ns'],
                last_unqualified_source_stamp_ns=ns, feedback_gap_source_stamp_ns=ns,
                feedback_gap_evidence=deepcopy(error.evidence))
            self._locked_source_status = dict(self._locked_source_pending,
                state='pending_feedback_bracket', source_stamp_ns=ns, source_qualified=False,
                identity_verified=False, support_measurement_available=False, metric_geometry_authority=False,
                motion_authorized=False, reference_refreshed=False, maximum_hold_seconds=min(self.max_age, .8))
            self.unavailable('accepted_feedback_bracket_pending_revalidation', source[0].header.stamp,
                             source, lose=False)

    def _require_initial_station_support_reference(self, source, reference):
        if (reference is not None and
                self._initial_station_support_reference(source, expected=reference) is not reference):
            raise ValueError('initial_station_current_support_invalidated')

    def _begin_feedback_recovery_completion(self, source, reference):
        """Publish a status-only admission after the complete real source check.

        A receiver must capture this exact frame before the old deadline. No
        false object/mask is emitted for the source that may later recover.
        """
        pending = getattr(self, '_locked_source_pending', None)
        if (reference is None or pending is None or pending.get('kind') != FEEDBACK_GAP
                or pending.get(FEEDBACK_COMPLETION) is not None):
            return
        marker = reference['initial_station_current_support']
        self._require_original_feedback_reference_clock(reference)
        ns = source[0].header.stamp.to_nsec()
        _, (_, before, after) = self._rgb_bracketed_feedback(ns, reference['epoch_ns'])
        accepted = [[int(row[0]), list(row[1])] for row in (before, after)]
        now = rospy.Time.now().to_nsec()
        value = make_completion(reference['source_stamp_ns'], min(self.max_age, .8),
            marker['identity_template']['provenance'], station_from_marker(marker), ns,
            self._processing_timing['processing_start_ros_ns'], now, accepted, pending)
        pending[FEEDBACK_COMPLETION] = value
        pending['_completion_monotonic'] = time.monotonic() + (
            value['identity_completion_deadline_ns']-now)*1e-9
        pending['_completion_last_ros_ns'] = now
        previous = self._locked_source_status
        self._locked_source_status = dict(
            (k, deepcopy(v)) for k, v in pending.items() if not k.startswith('_'))
        self._locked_source_status.update(state=FEEDBACK_PROCESSING, source_stamp_ns=ns,
            source_qualified=False, identity_verified=False, support_measurement_available=False,
            metric_geometry_authority=False, motion_authorized=False, reference_refreshed=False,
            maximum_hold_seconds=min(self.max_age, .8))
        try:
            self.status('unavailable', FEEDBACK_PROCESSING, source)
        finally:
            self._locked_source_status = previous

    def _observation_arrival_retry_reference(self, source, camera, expected=None, check_last=True):
        """Qualify only the third same-frame RGB crop at an acknowledged station.

        This marker supplies no mask, geometry or recovery permission. Its
        session deadline and measured arrival joints never advance with frames.
        """
        reference = getattr(self, '_locked_rgb_hold_reference', None)
        marker = (reference or {}).get('observation_arrival_retry')
        if marker is None:
            return None
        try:
            stamp_ns = source[0].header.stamp.to_nsec()
            now_ns = rospy.Time.now().to_nsec()
            if (expected is not None and reference is not expected
                    or marker.get('invalidated_reason') or reference['pose_invalidated']
                    or self.tracker.lost or self.first_loss is not None
                    or self.tracker.anchor is None or self.mode != 'unknown'
                    or self.last_good_source_stamp_ns != reference['source_stamp_ns']
                    or not reference['source_stamp_ns'] < stamp_ns < marker['session_deadline_ns']
                    or now_ns >= marker['session_deadline_ns']
                    or time.monotonic() >= marker['session_deadline_monotonic']
                    or (self.generation, self.selection_ns, self._rgb_interest_epoch) !=
                       (reference['generation'], reference['selection_ns'], marker['epoch_ns'])
                    or reference['source_id'] != marker['source_id']
                    or digest(asdict(self.config)) != marker['config_sha256']
                    or _candidate_fingerprint(self.tracker.anchor) != marker['anchor_fingerprint']
                    or not np.array_equal(self.tracker.plane_base, marker['reference_plane'])
                    or (check_last and _candidate_fingerprint(self.tracker.last) != reference['last_fingerprint'])
                    or rospy.get_param_cached('/gui/joint_direct_mode', False) is True
                    or digest(_camera_configuration_contract(camera)) != marker['camera_configuration_sha256']):
                raise ValueError('observation_arrival_station_changed')
            def check_source_lifetime():
                self._require_arrival_frame_reference_lifetime(
                    reference, marker, stamp_ns, rospy.Time.now().to_nsec())
            check_source_lifetime()
            if (source[1].header.stamp.to_nsec() != stamp_ns
                    or digest(self._rgb_interest_source_context(source[0].header.stamp)) != marker['source_id']):
                raise ValueError('observation_arrival_source_changed')
            feedback, (q, before, after) = self._rgb_bracketed_feedback(
                stamp_ns, marker['epoch_ns'], source_freshness_check=check_source_lifetime)
            positions = [q, before[1], after[1]] + [values for ns, values in feedback
                if ns >= marker['qualified_source_ns']]
            with self.input_lock:
                positions.extend(values for ns, values in self._rgb_interest_feedback
                    if ns >= marker['qualified_source_ns'])
                if (self._locked_rgb_hold_reference is not reference
                        or reference.get('observation_arrival_retry') is not marker
                        or marker.get('invalidated_reason') or reference['pose_invalidated']
                        or self._rgb_interest_epoch != marker['epoch_ns']
                        or rospy.get_param_cached('/gui/joint_direct_mode', False) is True
                        or any(np.max(np.abs(np.asarray(values)-marker['arrival_q'])) > .004
                               for values in positions)):
                    raise ValueError('observation_arrival_station_changed_during_processing')
            check_source_lifetime()
            return reference
        except (StationComputeDeadline, StationComputeRetired):
            raise
        except (ValueError, KeyError, TypeError, AttributeError, tf2_ros.LookupException,
                tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            if str(exc) == 'accepted_feedback_does_not_bracket_source':
                evidence = getattr(exc, 'accepted_feedback_evidence', None)
                try:
                    registration = self._station_compute_validate_feedback_gap(source, reference, evidence)
                except (ValueError, KeyError, TypeError, AttributeError):
                    pass
                else:
                    raise StationFeedbackBracketGap(registration, evidence) from exc
            marker['invalidated_reason'] = str(exc)
            timing = getattr(self, '_processing_timing', None)
            if timing is not None:
                timing['arrival_station_invalidated_reason'] = str(exc)
            with self.input_lock:
                reference.pop('observation_arrival_retry', None)
            return None

    def _begin_arrival_frame_completion(self, source, reference):
        # Preserve the existing arrival-only diagnostic/testing interface.
        self._begin_station_frame_completion(source, reference, 'observation_arrival_retry')

    def _begin_station_frame_completion(self, source, reference, marker_kind):
        """Admit one normal frame while BOTH reference and input are fresh.

        This is an in-process completion allowance, not a pending identity
        receipt or a renewal of old geometry. The fixed station is rechecked
        by its phase-specific marker gate before and after expensive stages.
        Initial and acknowledged-arrival stations share these clock rules;
        only the latter has a motion lease and a session deadline.
        """
        if (reference is None or getattr(self, '_arrival_frame_completion', None) is not None
                or getattr(self, '_locked_source_pending', None) is not None
                or getattr(self, '_metric_ambiguity_pending', None) is not None
                or getattr(self, '_observation_measurement', None) is not None
                or self.first_loss is not None or self.tracker.lost):
            return
        if marker_kind not in ('observation_arrival_retry', 'initial_station_current_support'):
            raise ValueError('invalid_station_frame_completion_kind')
        marker = (self._require_observation_arrival_marker(reference)
                  if marker_kind == 'observation_arrival_retry' else reference[marker_kind])
        source_ns = source[0].header.stamp.to_nsec()
        now_ns = rospy.Time.now().to_nsec()
        monotonic_now = time.monotonic()
        timing = self._processing_timing
        start_ns = timing['processing_start_ros_ns']
        maximum = min(self.max_age, .8)
        maximum_ns = round(maximum*1e9)
        if not (reference['source_stamp_ns'] < source_ns <= start_ns <= now_ns
                < reference['source_stamp_ns']+maximum_ns):
            # No timely admission: retain the phase's original rejection or
            # non-retry tracking behavior, never manufacture a completion.
            return
        require_fresh_source(reference['source_stamp_ns'], now_ns, maximum)
        require_fresh_source(source_ns, now_ns, maximum)
        if not self._initial_station_selection_matches():
            return
        deadline = min(source_ns+maximum_ns, reference['source_stamp_ns']+2*maximum_ns)
        if marker_kind == 'observation_arrival_retry':
            deadline = min(deadline, marker['session_deadline_ns'])
        completion_monotonic = monotonic_now+(deadline-now_ns)*1e-9
        if marker_kind == 'observation_arrival_retry':
            completion_monotonic = min(completion_monotonic, marker['session_deadline_monotonic'])
        token = dict(reference=reference, marker=marker, marker_kind=marker_kind,
            reference_source_stamp_ns=reference['source_stamp_ns'], source_stamp_ns=source_ns,
            qualified_entry_ns=now_ns, processing_start_ns=start_ns,
            geometry_deadline_ns=reference['source_stamp_ns']+maximum_ns,
            completion_deadline_ns=deadline, maximum_frame_age_ns=maximum_ns,
            completion_monotonic=completion_monotonic)
        if marker_kind == 'observation_arrival_retry':
            token.update(lease_id=marker['lease_id'], session_deadline_ns=marker['session_deadline_ns'])
        self._arrival_frame_completion = token
        diagnostic_key = ('arrival_frame_completion' if marker_kind == 'observation_arrival_retry'
                          else 'station_frame_completion')
        timing[diagnostic_key] = dict(
            (key, value) for key, value in token.items() if key not in ('reference', 'marker'))
        timing[diagnostic_key].update(
            reference_age_at_entry_sec=(now_ns-reference['source_stamp_ns'])*1e-9,
            source_age_at_entry_sec=(now_ns-source_ns)*1e-9,
            used_past_geometry_deadline=False, reference_refreshed=False,
            metric_geometry_authority=False, motion_authorized=False)

    def _require_arrival_frame_reference_lifetime(self, reference, marker, source_ns, now_ns):
        if self._station_compute_reference_lifetime(reference, marker, source_ns, now_ns):
            return
        self._require_station_frame_reference_lifetime(
            reference, marker, source_ns, now_ns, 'observation_arrival_retry')
        require_fresh_source(source_ns, rospy.Time.now().to_nsec(), min(self.max_age, .8))

    def _require_station_frame_reference_lifetime(self, reference, marker, source_ns, now_ns, marker_kind):
        token = getattr(self, '_arrival_frame_completion', None)
        if token is None:
            require_fresh_source(reference['source_stamp_ns'], now_ns, min(self.max_age, .8))
            return
        maximum_ns = round(min(self.max_age, .8)*1e9)
        deadline = min(source_ns+maximum_ns, reference['source_stamp_ns']+2*maximum_ns)
        if marker_kind == 'observation_arrival_retry':
            deadline = min(deadline, marker['session_deadline_ns'])
            if (token.get('lease_id') != marker['lease_id']
                    or token.get('session_deadline_ns') != marker['session_deadline_ns']):
                raise ValueError('arrival_frame_completion_binding_changed_or_expired')
        if (token['reference'] is not reference or token['marker'] is not marker
                or token['marker_kind'] != marker_kind
                or token['reference_source_stamp_ns'] != reference['source_stamp_ns']
                or token['source_stamp_ns'] != source_ns
                or token['maximum_frame_age_ns'] != maximum_ns
                or not reference['source_stamp_ns'] < source_ns <= token['processing_start_ns']
                   <= token['qualified_entry_ns'] < token['geometry_deadline_ns']
                or token['geometry_deadline_ns'] != reference['source_stamp_ns']+maximum_ns
                or token['completion_deadline_ns'] != deadline
                or not token['qualified_entry_ns'] <= now_ns < token['completion_deadline_ns']
                or time.monotonic() >= token['completion_monotonic']
                or getattr(self, '_locked_source_pending', None) is not None
                or getattr(self, '_metric_ambiguity_pending', None) is not None
                or getattr(self, '_observation_measurement', None) is not None):
            raise ValueError('arrival_frame_completion_binding_changed_or_expired')
        require_fresh_source(source_ns, now_ns, min(self.max_age, .8))
        timing = getattr(self, '_processing_timing', None)
        if (timing is not None and timing.get('source_stamp_ns') == source_ns
                and now_ns >= token['geometry_deadline_ns']):
            diagnostic_key = ('arrival_frame_completion' if marker_kind == 'observation_arrival_retry'
                              else 'station_frame_completion')
            timing[diagnostic_key]['used_past_geometry_deadline'] = True

    def _locked_rgb_shrinking_fragment(self, candidate, reference, source, camera):
        """Validate native fragment geometry before the separate RGB audit.

        The existing metric tracking ratios remain unchanged. A valid metric
        match returns None and still takes the original tracking/refinement path.
        """
        if not isinstance(candidate, Candidate):
            raise ValueError('locked_rgb_hold_fragment_type_invalid')
        def numeric(value, shape):
            raw = np.asarray(value)
            if (raw.shape != shape or raw.dtype.kind not in 'iuf'
                    or not np.all(np.isfinite(raw))):
                raise ValueError('locked_rgb_hold_fragment_geometry_invalid')
            return np.asarray(raw, dtype=float)
        for name, shape in (('extent_m', (3,)), ('position_base', (3,)),
                            ('position_camera', (3,)), ('center_uv', (2,))):
            numeric(getattr(candidate, name), shape)
        extent = np.asarray(candidate.extent_m, dtype=float)
        if np.any(extent[:2] <= 0.) or extent[2] < 0.:
            raise ValueError('locked_rgb_hold_fragment_extent_invalid')
        anchor_extent = np.asarray(self.tracker.anchor.extent_m, dtype=float)
        ratios = extent[:2]/np.maximum(anchor_extent[:2], .003)
        if np.all(ratios >= .5):
            return None
        anchor_distance = float(np.linalg.norm(candidate.position_base-self.tracker.anchor.position_base))
        last_distance = float(np.linalg.norm(candidate.position_base-self.tracker.last.position_base))
        if (np.any(extent[:2] > anchor_extent[:2]) or np.any(ratios > 1.)
                or anchor_distance > self.config.association_distance_m
                or last_distance > self.config.association_distance_m):
            raise ValueError('locked_rgb_hold_fragment_not_contained_contraction')
        candidate_depth = float(numeric(candidate.depth_m, ()))
        quality = float(numeric(candidate.quality, ()))
        if (not self.config.depth_min_m <= candidate_depth <= self.config.depth_max_m
                or not 0. <= quality <= 1.):
            raise ValueError('locked_rgb_hold_fragment_geometry_invalid')
        raw_mask = np.asarray(candidate.mask)
        if raw_mask.dtype.kind not in 'bui' or not np.all(np.isin(raw_mask, (0, 1, 255))):
            raise ValueError('locked_rgb_hold_fragment_mask_invalid')
        mask = _mask(raw_mask, reference['image_shape'])
        count = int(np.count_nonzero(mask))
        bbox = numeric(candidate.bbox, (4,))
        footprint = np.asarray(candidate.support_footprint_base)
        if footprint.ndim != 2 or footprint.shape[1:] != (3,) or len(footprint) < 3:
            raise ValueError('locked_rgb_hold_fragment_geometry_invalid')
        numeric(footprint, footprint.shape)
        valid_points = candidate.metrics.get('valid_points') if isinstance(candidate.metrics, dict) else None
        if (not np.array_equal(bbox, cv2.boundingRect(mask))
                or count < self.config.min_object_points
                or isinstance(valid_points, (bool, np.bool_))
                or not isinstance(valid_points, (int, np.integer)) or valid_points != count):
            raise ValueError('locked_rgb_hold_fragment_geometry_invalid')
        depth = np.asarray(self.bridge.imgmsg_to_cv2(source[1], desired_encoding='passthrough'), dtype=float)
        if source[1].encoding in ('16UC1', 'mono16'):
            scale = float(camera['depth_scale'])
            if not np.isfinite(scale) or scale <= 0.:
                raise ValueError('locked_rgb_hold_fragment_depth_scale_invalid')
            depth = depth*scale
        elif source[1].encoding != '32FC1':
            raise ValueError('locked_rgb_hold_fragment_depth_encoding_invalid')
        if depth.shape != reference['image_shape']:
            raise ValueError('locked_rgb_hold_fragment_depth_shape_invalid')
        measured = depth[mask != 0]
        if (not np.all(np.isfinite(measured)) or np.any(measured < self.config.depth_min_m)
                or np.any(measured > self.config.depth_max_m)):
            raise ValueError('locked_rgb_hold_fragment_has_unmeasured_pixels')
        return dict(mask=mask, evidence=dict(kind='shrinking_native_fragment',
            candidate_mask_sha256=hashlib.sha256(mask.tobytes()).hexdigest(), measured_pixel_count=count,
            extent_ratios_xy=ratios.tolist(), anchor_distance_m=anchor_distance,
            last_distance_m=last_distance))

    def _locked_rgb_fragment_ownership(self, mask, reference_mask, current_mask):
        """Audit identity ownership without changing any native fragment pixel.

        Exact containment keeps its original behavior. A boundary discrepancy
        admits only a one-pixel (3x3/Chebyshev) RGB-D outline uncertainty, with
        an independently nonempty measured interior and joint mask retention.
        This is an identity-only contract, not geometric confidence or a depth
        correction. Dilations and erosion are predicates, never output masks.
        """
        pixels = mask != 0
        old, current = reference_mask != 0, current_mask != 0
        joint = old & current
        count = int(np.count_nonzero(pixels))
        retained_old = int(np.count_nonzero(pixels & old))
        retained_current = int(np.count_nonzero(pixels & current))
        retained_joint = int(np.count_nonzero(pixels & joint))
        kernel = np.ones((3, 3), np.uint8)
        core = cv2.erode(pixels.astype(np.uint8), kernel) != 0
        core_count = int(np.count_nonzero(core))
        core_outside = int(np.count_nonzero(core & ~joint))
        boundary_used = retained_joint != count
        if boundary_used:
            if core_count < 25:
                raise ValueError('locked_rgb_hold_fragment_boundary_core_too_small')
            if core_outside:
                raise ValueError('locked_rgb_hold_fragment_boundary_core_conflict')
            if retained_joint < .85*count:
                raise ValueError('locked_rgb_hold_fragment_boundary_retention_insufficient')
            old_band = cv2.dilate(old.astype(np.uint8), kernel) != 0
            current_band = cv2.dilate(current.astype(np.uint8), kernel) != 0
            if np.any(pixels & ~(old_band & current_band)):
                raise ValueError('locked_rgb_hold_fragment_outside_one_pixel_rgb_boundary')
        return dict(last_measured_rgb_containment_fraction=retained_old/count,
            current_rgb_containment_fraction=retained_current/count,
            joint_rgb_containment_fraction=retained_joint/count,
            last_measured_rgb_outside_pixels=count-retained_old,
            current_rgb_outside_pixels=count-retained_current,
            joint_rgb_outside_pixels=count-retained_joint,
            eroded_interior_pixels=core_count, eroded_interior_outside_joint_rgb_pixels=core_outside,
            boundary_tolerance_used=boundary_used,
            ownership_policy='bounded_rgb_depth_boundary_identity_only' if boundary_used else 'exact_containment',
            last_measured_rgb_boundary_margin_px=int(retained_old != count),
            current_rgb_boundary_margin_px=int(retained_current != count),
            maximum_rgb_boundary_margin_px=1, minimum_boundary_interior_pixels=25,
            minimum_boundary_joint_retention_fraction=.85, candidate_mask_modified=False)

    def _require_original_feedback_reference_clock(self, reference):
        now = rospy.Time.now().to_nsec()
        if (time.monotonic() >= reference['_reference_monotonic']
                or now < reference['_reference_last_ros_ns']):
            raise ValueError('feedback_recovery_original_clock_expired_or_reversed')
        reference['_reference_last_ros_ns'] = now

    def _require_support_reference_lifetime(self, reference, source_stamp_ns=None):
        pending = getattr(self, '_locked_source_pending', None)
        contract = (pending or {}).get('identity_completion')
        now_ns = rospy.Time.now().to_nsec()
        feedback_completion = (pending or {}).get(FEEDBACK_COMPLETION)
        if feedback_completion is not None:
            marker = reference.get('initial_station_current_support')
            if (pending.get('kind') != FEEDBACK_GAP or marker is None
                    or marker.get('invalidated_reason') or not self._initial_station_selection_matches()
                    or self.first_loss is not None or self.tracker.lost
                    or getattr(self, '_observation_measurement', None) is not None
                    or getattr(self, '_metric_ambiguity_pending', None) is not None
                    or rospy.get_param_cached('/gui/joint_direct_mode', False) is True
                    or time.monotonic() >= pending['_completion_monotonic']
                    or now_ns < pending['_completion_last_ros_ns']
                    or getattr(self, '_processing_timing_thread_ident', None) != threading.get_ident()
                    or self._processing_timing is None
                    or self._processing_timing.get('source_stamp_ns') != feedback_completion['source_stamp_ns']):
                raise ValueError('feedback_recovery_completion_context_changed')
            validate_feedback_completion(feedback_completion, reference['source_stamp_ns'],
                min(self.max_age, .8), marker['identity_template']['provenance'],
                station_from_marker(marker), now_ns, source_stamp_ns)
            pending['_completion_last_ros_ns'] = now_ns
            return
        if pending is not None and pending.get('kind') == FEEDBACK_GAP:
            self._require_original_feedback_reference_clock(reference)
        marker = reference.get('observation_arrival_retry')
        if (pending is None and source_stamp_ns is not None and marker is not None
                and self._station_compute_reference_lifetime(
                    reference, marker, source_stamp_ns, now_ns)):
            # An already registered station retains identity only, on its
            # original clock. The current exact frame keeps its own .8 s
            # limit and all scope/SDK checks; the old plane is not new support.
            return
        if contract is None:
            token = getattr(self, '_arrival_frame_completion', None)
            marker = reference.get('initial_station_current_support')
            if (pending is None and source_stamp_ns is not None and marker is not None
                    and token is not None and token.get('marker_kind') == 'initial_station_current_support'):
                self._require_station_frame_reference_lifetime(
                    reference, marker, source_stamp_ns, now_ns, 'initial_station_current_support')
                return
            require_fresh_source(reference['source_stamp_ns'], now_ns, min(self.max_age, .8))
            return
        marker = reference.get('initial_station_current_support')
        if (pending.get('kind') != 'support_measurement_gap' or marker is None
                or marker.get('invalidated_reason') or not self._initial_station_selection_matches()
                or self.first_loss is not None or self.tracker.lost
                or getattr(self, '_observation_measurement', None) is not None
                or getattr(self, '_metric_ambiguity_pending', None) is not None
                or rospy.get_param_cached('/gui/joint_direct_mode', False) is True
                or reference['source_id'] != marker['source_id']
                or reference['epoch_ns'] != marker['epoch_ns']
                or reference['anchor_fingerprint'] != marker['anchor_fingerprint']
                or not np.array_equal(reference['reference_plane'], marker['reference_plane'])):
            raise ValueError('identity_completion_initial_station_invalidated')
        with self.input_lock:
            measured = [q for ns, q in self._rgb_interest_feedback
                        if ns >= reference['source_stamp_ns']]
            if (not measured or np.max(np.abs(reference['q_source']-marker['q_source'])) > .004
                    or any(np.max(np.abs(q-marker['q_source'])) > .004 for q in measured)):
                raise ValueError('identity_completion_fixed_pose_changed')
        validate_completion(contract, reference['source_stamp_ns'], min(self.max_age, .8),
            marker['identity_template']['provenance'], station_from_marker(marker), now_ns, source_stamp_ns)

    def _check_pending_source_reference(self):
        """A waiting input cannot extend or replace the last measured lock."""
        pending = getattr(self, '_locked_source_pending', None)
        if pending is None:
            return
        reference = getattr(self, '_locked_rgb_hold_reference', None)
        if (pending.get('invalidated_reason') or reference is None or self.tracker.lost
                or self.tracker.anchor is None
                or reference['source_stamp_ns'] != pending['reference_source_stamp_ns']
                or reference['source_id'] != pending['source_id']
                or self.last_good_source_stamp_ns != reference['source_stamp_ns']
                or (self.generation, self.selection_ns, self._rgb_interest_epoch) !=
                   (reference['generation'], reference['selection_ns'], reference['epoch_ns'])):
            raise ValueError('pending_source_reference_invalidated')
        self._require_support_reference_lifetime(reference)
        with self.input_lock:
            newest = max(self._rgb_interest_feedback, key=lambda row: row[0], default=None)
            if newest is None:
                raise ValueError('pending_source_latest_feedback_unavailable')
            require_fresh_source(int(newest[0]), rospy.Time.now().to_nsec(), min(self.max_age, .8))
        if (reference['pose_invalidated'] or digest(asdict(self.config)) != reference['config_sha256']
                or _candidate_fingerprint(self.tracker.anchor) != reference['anchor_fingerprint']
                or _candidate_fingerprint(self.tracker.last) != reference['last_fingerprint']
                or not np.array_equal(self.tracker.plane_base, reference['reference_plane'])):
            raise ValueError('pending_source_reference_changed')

    def _revalidate_pending_source_commit(self, source, new_reference):
        """Complete qualification after refinement, before live tracker commit."""
        self._check_pending_source_reference()
        reference = self._locked_rgb_hold_reference
        stamp_ns = source[0].header.stamp.to_nsec()
        self._require_support_reference_lifetime(reference, stamp_ns)
        if (new_reference is None or new_reference['source_stamp_ns'] != stamp_ns
                or new_reference['source_id'] != reference['source_id']):
            raise ValueError('pending_source_new_reference_unavailable')
        if digest(self._rgb_interest_source_context(source[0].header.stamp)) != reference['source_id']:
            raise ValueError('pending_source_context_changed_before_commit')
        _, (q, before, after) = self._rgb_bracketed_feedback(stamp_ns, reference['epoch_ns'])
        if any(np.max(np.abs(np.asarray(values)-reference['q_source'])) > .004
               for values in (q, before[1], after[1])):
            raise ValueError('pending_source_pose_changed_before_commit')
        self._check_pending_source_reference()
        self._require_support_reference_lifetime(reference, stamp_ns)
        require_fresh_source(stamp_ns, rospy.Time.now().to_nsec(), min(self.max_age, .8))

    def _check_metric_ambiguity_reference(self, expected=None):
        """Check the unchanged real last-good reference, including timer expiry."""
        reference = getattr(self, '_locked_rgb_hold_reference', None)
        pending = getattr(self, '_metric_ambiguity_pending', None)
        if (reference is None or (expected is not None and reference is not expected)
                or self.tracker.lost or self.tracker.anchor is None
                or getattr(self, '_locked_source_pending', None) is not None
                or self.last_good_source_stamp_ns != reference['source_stamp_ns']
                or (self.generation, self.selection_ns, self._rgb_interest_epoch) !=
                   (reference['generation'], reference['selection_ns'], reference['epoch_ns'])
                or (pending is not None and (pending.get('invalidated_reason')
                    or pending['reference_source_stamp_ns'] != reference['source_stamp_ns']
                    or pending['source_id'] != reference['source_id']))):
            raise ValueError('metric_ambiguity_reference_invalidated')
        now_ns = rospy.Time.now().to_nsec()
        require_fresh_source(reference['source_stamp_ns'], now_ns, min(self.max_age, .8))
        if (reference['pose_invalidated'] or digest(asdict(self.config)) != reference['config_sha256']
                or _candidate_fingerprint(self.tracker.anchor) != reference['anchor_fingerprint']
                or _candidate_fingerprint(self.tracker.last) != reference['last_fingerprint']
                or not np.array_equal(self.tracker.plane_base, reference['reference_plane'])):
            raise ValueError('metric_ambiguity_reference_changed')
        with self.input_lock:
            newest = max(self._rgb_interest_feedback, key=lambda row: row[0], default=None)
            seen = [values for ns, values in self._rgb_interest_feedback
                    if ns >= reference['source_stamp_ns']]
            if newest is None or not seen:
                raise ValueError('metric_ambiguity_feedback_unavailable')
            require_fresh_source(int(newest[0]), rospy.Time.now().to_nsec(), min(self.max_age, .8))
            if any(np.max(np.abs(values-reference['q_source'])) > .004 for values in seen):
                raise ValueError('metric_ambiguity_pose_changed')
        return reference

    def _qualify_metric_ambiguity_frame(self, source, result, reference, *, current_fragment_group=False):
        """No pending qualification: current real brackets and all gates must pass."""
        self._check_metric_ambiguity_reference(reference)
        stamp_ns = source[0].header.stamp.to_nsec()
        if (stamp_ns <= reference['source_stamp_ns']
                or source[1].header.stamp.to_nsec() != stamp_ns):
            raise ValueError('metric_ambiguity_source_sequence_changed')
        require_fresh_source(stamp_ns, rospy.Time.now().to_nsec(), min(self.max_age, .8))
        if digest(self._rgb_interest_source_context(source[0].header.stamp)) != reference['source_id']:
            raise ValueError('metric_ambiguity_source_context_changed')
        _, (q, before, after) = self._rgb_bracketed_feedback(stamp_ns, reference['epoch_ns'])
        if any(np.max(np.abs(np.asarray(values)-reference['q_source'])) > .004
               for values in (q, before[1], after[1])):
            raise ValueError('metric_ambiguity_bracket_pose_changed')
        metrics = result.metrics
        current_fit = current_fragment_group and metrics.get('support_source') == 'current_ring_fit'
        if (metrics.get('support_current_audit_available') is not True
                or (metrics.get('support_source') != 'locked_base_plane' and not current_fit)):
            raise ValueError('metric_ambiguity_current_support_unavailable')
        for key, maximum in (('support_reference_angle_deg', self.config.support_max_normal_change_deg),
                             ('support_reference_max_distance_m', self.config.support_max_local_shift_m)):
            value = metrics[key]
            if not np.isfinite(value) or not 0. <= value <= maximum:
                raise ValueError('metric_ambiguity_support_inconsistent')
        if metrics['support_reference_footprint_vertices'] != len(reference['reference_footprint']):
            raise ValueError('metric_ambiguity_support_footprint_changed')
        quadrants = metrics['support_current_audit_quadrants']
        ratio = metrics['support_current_audit_inlier_ratio']
        residual = metrics['support_current_audit_residual_m']
        if current_fit:
            _validate_support_plane_update(reference['reference_plane'], result.plane_base,
                self.tracker.anchor.position_base, self.config, reference['reference_footprint'])
        if ((not current_fit and not np.array_equal(result.plane_base, reference['reference_plane']))
                or not all(np.isfinite(value) for value in (quadrants, ratio, residual))
                or quadrants not in (3, 4) or not self.config.support_min_ratio <= ratio <= 1.
                or not 0. <= residual <= self.config.support_threshold_m):
            raise ValueError('metric_ambiguity_support_evidence_invalid')
        self._check_metric_ambiguity_reference(reference)
        require_fresh_source(stamp_ns, rospy.Time.now().to_nsec(), min(self.max_age, .8))

    def _arrival_rgb_spatial_kwargs(self, reference, unique=True):
        if reference is None:
            return {}
        marker = self._require_observation_arrival_marker(reference)
        return dict(observation_arrival_retry=True,
                    observation_spatial_reference=marker.get('spatial_reference') if unique else None)

    def _check_recovery_arrival_reference(self, source, camera, reference):
        """The same acknowledged station must cover recovery RGB work too."""
        if (reference is not None and self._observation_arrival_retry_reference(
                source, camera, expected=reference) is not reference):
            raise ValueError('observation_arrival_station_invalidated_during_recovery')

    def _qualified_fragment_pair(self, source, result, camera, arrival_reference, pair):
        if pair is None:
            return False
        if (arrival_reference is None or not isinstance(pair, tuple) or len(pair) != 2
                or len(result.candidates) != 1 or result.candidates[0] is not pair[0]
                or not all(isinstance(candidate, Candidate) for candidate in pair)
                or pair[0].metrics.get('observation_fragment_group') is None
                or pair[0].metrics['observation_fragment_group'] is not pair[1].metrics.get('observation_fragment_group')):
            raise ValueError('observation_fragment_recovery_pair_invalid')
        self._check_recovery_arrival_reference(source, camera, arrival_reference)
        return True

    def _metric_ambiguity_transition(self, source, result, camera, depth, intrinsics, transform,
                                     arrival_retry_reference=None, pre_refined=None):
        """Return a staged UNIQUE recovery, or publish only a bounded ambiguity hold."""
        reference = self._check_metric_ambiguity_reference()
        pending = getattr(self, '_metric_ambiguity_pending', None)
        stamp_ns = source[0].header.stamp.to_nsec()
        if pending is not None and stamp_ns <= pending['last_ambiguous_source_stamp_ns']:
            raise ValueError('metric_ambiguity_requires_new_frame')
        if not self.rgb_instance_refinement:
            raise ValueError('metric_ambiguity_requires_rgb_refinement')
        if not isinstance(result.candidates, (tuple, list)) or len(result.candidates) > HandoffPolicy().maximum_candidates:
            raise ValueError('metric_ambiguity_candidate_budget')
        matches = [row[1] for row in _matching_candidates(
            result.candidates, self.tracker.anchor, self.tracker.last, self.config)]
        if not matches or (pending is None and len(matches) < 2):
            raise ValueError('metric_ambiguity_metric_state_invalid')
        grouped = self._qualified_fragment_pair(source, result, camera, arrival_retry_reference, pre_refined)
        self._qualify_metric_ambiguity_frame(source, result, reference, current_fragment_group=grouped)
        self._check_recovery_arrival_reference(source, camera, arrival_retry_reference)
        rgb = self.bridge.imgmsg_to_cv2(source[0], desired_encoding='bgr8')
        refined, identity = metric_seeded_identity_evidence(rgb, [pre_refined[1]] if grouped else matches,
            reference['bgr'], reference['mask'], minimum_quality=self.minimum_quality,
            rgb_valid=rectification_rgb_valid_mask(camera, depth.shape),
            refine=lambda candidate: (pre_refined[1] if grouped and candidate is pre_refined[1] else refine_candidate(rgb, depth, intrinsics, transform,
                candidate, self.config,
                **self._arrival_rgb_spatial_kwargs(arrival_retry_reference, unique=len(matches) == 1),
                **_rgb_source_kwargs(camera, depth.shape))))
        self._check_recovery_arrival_reference(source, camera, arrival_retry_reference)
        self._qualify_metric_ambiguity_frame(source, result, reference, current_fragment_group=grouped)
        if grouped:
            identity.update(native_role='final_current_rgb_instance',
                observation_fragment_group=pre_refined[1].metrics['observation_fragment_group'])
        status = dict(identity, source_stamp_ns=stamp_ns,
            reference_source_stamp_ns=reference['source_stamp_ns'], source_id=reference['source_id'],
            reference_refreshed=False, maximum_hold_seconds=min(self.max_age, .8),
            elapsed_since_measured_sec=(rospy.Time.now().to_nsec()-reference['source_stamp_ns'])*1e-9,
            support_audit={key: result.metrics[key] for key in (
                'support_reference_angle_deg', 'support_reference_max_distance_m',
                'support_reference_footprint_vertices')})
        if len(matches) > 1:
            with self.input_lock:
                self._check_recovery_arrival_reference(source, camera, arrival_retry_reference)
                self._check_metric_ambiguity_reference(reference)
                require_fresh_source(stamp_ns, rospy.Time.now().to_nsec(), min(self.max_age, .8))
                self._metric_ambiguity_pending = dict(reference_source_stamp_ns=reference['source_stamp_ns'],
                    source_id=reference['source_id'], last_ambiguous_source_stamp_ns=stamp_ns,
                    first_ambiguous_source_stamp_ns=(stamp_ns if pending is None else
                        pending['first_ambiguous_source_stamp_ns']))
                self._metric_ambiguity_status = dict(status, state='holding_identity_only')
                self.unavailable('measured_geometry_temporarily_ambiguous_rgb_identity_held',
                                 source[0].header.stamp, source, lose=False)
            return None
        # The original 3-D tracker is already unique. Stage its ordinary choice;
        # the RGB evidence is an additional recovery gate, never the selector.
        staged = TargetTracker(self.config)
        staged.anchor, staged.last = self.tracker.anchor, self.tracker.last
        staged.plane_base = self.tracker.plane_base.copy()
        candidate, reason = staged.choose(result, depth.shape)
        if candidate is not matches[0] or reason != 'tracked':
            raise ValueError('metric_ambiguity_unique_recovery_changed')
        self._metric_ambiguity_status = dict(status, state='qualified_unique_metric_recovery')
        return staged, refined[0], reference

    def _locked_rgb_outline_match(self, source, camera, reference):
        """Independent complete RGB outline, using the existing identity gates."""
        association_policy = HandoffPolicy()
        rgb = self.bridge.imgmsg_to_cv2(source[0], desired_encoding='bgr8')
        if rgb.shape != reference['image_shape']+(3,):
            raise ValueError('locked_rgb_hold_image_shape_changed')
        x, y, width, height = cv2.boundingRect(reference['mask'])
        policy = replace(self._rgb_interest_policy,
            center_xy=((x+.5*width)/rgb.shape[1], (y+.5*height)/rgb.shape[0]))
        proposals = propose_rgb_observation_regions(rgb, policy=policy,
            **_rgb_source_kwargs(camera, rgb.shape[:2]))
        if 'budget' in proposals.reason:
            raise ValueError('locked_rgb_hold_rgb_candidate_budget')
        matches = []
        for index, region in enumerate(proposals.candidates):
            mask = _mask(region.mask, rgb.shape[:2])
            iou = np.count_nonzero(mask & reference['mask']) / np.count_nonzero(mask | reference['mask'])
            if iou < .85:  # Existing same-station InitialAcquisition outline gate.
                continue
            histogram, contour = _appearance(rgb, mask)
            colour = float(np.sqrt(max(0., 1.-np.sum(np.sqrt(histogram*reference['histogram'])))))
            shape = float(cv2.matchShapes(reference['contour'], contour, cv2.CONTOURS_MATCH_I1, 0.))
            if colour <= association_policy.max_colour_distance and shape <= association_policy.max_contour_distance:
                score = .65*colour/association_policy.max_colour_distance+.35*shape/association_policy.max_contour_distance
                matches.append((score, index, float(iou)))
        matches.sort()
        if not matches or (len(matches) > 1 and matches[1][0]-matches[0][0] <= association_policy.ambiguity_margin):
            raise ValueError('locked_rgb_hold_no_unique_current_rgb_match')
        return _mask(proposals.candidates[matches[0][1]].mask, rgb.shape[:2]), matches[0][2]

    def _qualify_support_gap_source(self, source, reference):
        """A support gap needs real measured brackets; it cannot defer them."""
        self._check_pending_source_reference()
        stamp_ns = source[0].header.stamp.to_nsec()
        pending = getattr(self, '_locked_source_pending', None)
        if (self._locked_rgb_hold_reference is not reference or self.tracker.lost
                or self.tracker.anchor is None or getattr(self, '_metric_ambiguity_pending', None) is not None
                or self.last_good_source_stamp_ns != reference['source_stamp_ns']
                or (self.generation, self.selection_ns, self._rgb_interest_epoch) !=
                   (reference['generation'], reference['selection_ns'], reference['epoch_ns'])
                or stamp_ns <= reference['source_stamp_ns']
                or source[1].header.stamp.to_nsec() != stamp_ns
                or (pending is not None and stamp_ns <= pending['last_unqualified_source_stamp_ns'])):
            raise ValueError('support_gap_source_reference_invalidated')
        now_ns = rospy.Time.now().to_nsec()
        self._require_support_reference_lifetime(reference, stamp_ns)
        require_fresh_source(stamp_ns, now_ns, min(self.max_age, .8))
        if (reference['pose_invalidated'] or digest(asdict(self.config)) != reference['config_sha256']
                or _candidate_fingerprint(self.tracker.anchor) != reference['anchor_fingerprint']
                or _candidate_fingerprint(self.tracker.last) != reference['last_fingerprint']
                or not np.array_equal(self.tracker.plane_base, reference['reference_plane'])
                or digest(self._rgb_interest_source_context(source[0].header.stamp)) != reference['source_id']):
            raise ValueError('support_gap_source_reference_changed')
        _, (q, before, after) = self._rgb_bracketed_feedback(stamp_ns, reference['epoch_ns'])
        if any(np.max(np.abs(np.asarray(values)-reference['q_source'])) > .004
               for values in (q, before[1], after[1])):
            raise ValueError('support_gap_bracket_pose_changed')
        with self.input_lock:
            seen = [values for ns, values in self._rgb_interest_feedback
                    if ns >= reference['source_stamp_ns']]
            newest = max(self._rgb_interest_feedback, key=lambda row: row[0], default=None)
            if (self._locked_rgb_hold_reference is not reference or reference['pose_invalidated']
                    or self._rgb_interest_epoch != reference['epoch_ns'] or not seen or newest is None
                    or any(np.max(np.abs(values-reference['q_source'])) > .004 for values in seen)):
                raise ValueError('support_gap_source_changed_during_processing')
            now_ns = rospy.Time.now().to_nsec()
            require_fresh_source(int(newest[0]), now_ns, min(self.max_age, .8))
            self._require_support_reference_lifetime(reference, stamp_ns)
            require_fresh_source(stamp_ns, now_ns, min(self.max_age, .8))

    def _hold_missing_current_support(self, source, camera, reason, result=None):
        """Withhold all geometry until independent current support recovers.

        Neither the immutable plane nor a candidate filtered with it supplies
        current support. The original last-good stamp bounds every held frame.
        """
        reference = getattr(self, '_locked_rgb_hold_reference', None)
        if reason not in _SUPPORT_MEASUREMENT_GAPS or reference is None or not self.rgb_instance_refinement:
            raise ValueError('support_gap_not_qualified: '+str(reason))
        if result is not None:
            metrics = result.metrics
            if metrics.get('support_current_audit_available') is not False:
                raise ValueError('support_gap_current_audit_state_invalid')
            # Explicit disagreement is never converted into absent evidence,
            # even if an upstream result incorrectly also marks the audit absent.
            for key, maximum in (('support_reference_angle_deg', self.config.support_max_normal_change_deg),
                                 ('support_reference_max_distance_m', self.config.support_max_local_shift_m),
                                 ('support_reference_anchor_distance_m', self.config.support_max_local_shift_m)):
                if key in metrics and (not np.isfinite(metrics[key]) or not 0. <= metrics[key] <= maximum):
                    raise ValueError('support_gap_positive_support_conflict')
            if metrics.get('support_source') == 'locked_base_plane':
                if not np.array_equal(result.plane_base, reference['reference_plane']):
                    raise ValueError('support_gap_reference_plane_changed')
            elif metrics.get('support_source') == 'current_ring_fit':
                _validate_support_plane_update(reference['reference_plane'], result.plane_base,
                    self.tracker.anchor.position_base, self.config, reference['reference_footprint'])
            else:
                raise ValueError('support_gap_support_provenance_invalid')
        if self._station_compute_publish_support_gap(source, reference, reason):
            return
        self._publish_unverified_support_pending(source, reference, reason)

    def _publish_unverified_support_pending(self, source, reference, reason, support_audit=None):
        # A fresh RGB image is required as input, but missing support does not
        # establish either present identity or its loss. Never claim a new RGB
        # match, use an old plane as new support, or renew the original deadline.
        if (getattr(self, '_locked_source_pending', None) or {}).get(FEEDBACK_COMPLETION) is not None:
            raise ValueError('feedback_recovery_requires_complete_current_measurement')
        rgb = self.bridge.imgmsg_to_cv2(source[0], desired_encoding='bgr8')
        if rgb.dtype != np.uint8 or rgb.shape != reference['image_shape']+(3,):
            raise ValueError('support_gap_rgb_input_invalid')
        self._qualify_support_gap_source(source, reference)
        stamp_ns = source[0].header.stamp.to_nsec()
        pending = getattr(self, '_locked_source_pending', None)
        if pending is not None and pending.get('kind') == FEEDBACK_GAP:
            raise ValueError('feedback_gap_requires_full_current_measurement')
        with self.input_lock:
            self._qualify_support_gap_source(source, reference)
            completion = None
            if pending is not None and pending.get('identity_completion') is not None:
                completion = deepcopy(pending['identity_completion'])
            elif reference.get('initial_station_current_support') is not None:
                template = self._initial_station_identity_template(source, reference)
                # Enrollment, unlike later completion, still needs fresh old
                # geometry and cannot be obtained after the original cutoff.
                require_fresh_source(reference['source_stamp_ns'], rospy.Time.now().to_nsec(), min(self.max_age, .8))
                completion = completion_contract(reference['source_stamp_ns'], min(self.max_age, .8),
                    template['provenance'], station_from_marker(reference['initial_station_current_support']))
            self._locked_source_pending = dict(kind='support_measurement_gap',
                reference_source_stamp_ns=reference['source_stamp_ns'], source_id=reference['source_id'],
                first_unqualified_source_stamp_ns=(stamp_ns if pending is None else
                    pending['first_unqualified_source_stamp_ns']), last_unqualified_source_stamp_ns=stamp_ns)
            if completion is not None:
                self._locked_source_pending['identity_completion'] = completion
            self._locked_source_status = dict(self._locked_source_pending,
                state='pending_current_support' if support_audit is None else 'pending_unique_measured_geometry',
                reason=reason, source_stamp_ns=stamp_ns, source_qualified=True,
                identity_verified=False, support_measurement_available=support_audit is not None,
                support_audit=support_audit, metric_geometry_authority=False, motion_authorized=False,
                reference_refreshed=False, maximum_hold_seconds=min(self.max_age, .8),
                elapsed_since_measured_sec=(rospy.Time.now().to_nsec()-reference['source_stamp_ns'])*1e-9)
            self.unavailable('support_measurement_pending_revalidation', source[0].header.stamp, source, lose=False)

    def _support_gap_recovery(self, source, result, camera, depth, intrinsics, transform,
                              arrival_retry_reference=None, pre_refined=None):
        """Stage original unique metric tracking, with like-for-like RGB identity.

        The current independent support fit and native RGB-D refinement must
        recover before publication. No independent MSER mask selects or replaces
        either the old refined outline or this frame's measured geometry.
        """
        reference = self._locked_rgb_hold_reference
        self._qualify_support_gap_source(source, reference)
        metrics = result.metrics
        if metrics.get('support_current_audit_available') is not True:
            raise ValueError('support_gap_recovery_requires_current_support')
        for key, maximum in (('support_reference_angle_deg', self.config.support_max_normal_change_deg),
                             ('support_reference_max_distance_m', self.config.support_max_local_shift_m)):
            value = metrics[key]
            if not np.isfinite(value) or not 0. <= value <= maximum:
                raise ValueError('support_gap_recovery_support_conflict')
        if metrics['support_reference_footprint_vertices'] != len(reference['reference_footprint']):
            raise ValueError('support_gap_recovery_support_footprint_changed')
        quadrants, ratio, residual = (metrics[key] for key in ('support_current_audit_quadrants',
            'support_current_audit_inlier_ratio', 'support_current_audit_residual_m'))
        if (not all(np.isfinite(value) for value in (quadrants, ratio, residual))
                or quadrants not in (3, 4) or not self.config.support_min_ratio <= ratio <= 1.
                or not 0. <= residual <= self.config.support_threshold_m):
            raise ValueError('support_gap_recovery_support_evidence_invalid')
        if metrics.get('support_source') == 'locked_base_plane':
            if not np.array_equal(result.plane_base, reference['reference_plane']):
                raise ValueError('support_gap_recovery_reference_plane_changed')
        elif metrics.get('support_source') == 'current_ring_fit':
            _validate_support_plane_update(reference['reference_plane'], result.plane_base,
                self.tracker.anchor.position_base, self.config, reference['reference_footprint'])
        else:
            raise ValueError('support_gap_recovery_support_provenance_invalid')
        support_audit = {key: metrics[key] for key in ('support_reference_angle_deg',
            'support_reference_max_distance_m', 'support_reference_footprint_vertices',
            'support_current_audit_quadrants', 'support_current_audit_inlier_ratio',
            'support_current_audit_residual_m')}
        if not isinstance(result.candidates, (list, tuple)) or len(result.candidates) > HandoffPolicy().maximum_candidates:
            raise ValueError('support_gap_recovery_candidate_budget')
        if not result.candidates:
            self._publish_unverified_support_pending(source, reference,
                'current_metric_target_unavailable', support_audit)
            return None
        staged = TargetTracker(self.config)
        staged.anchor, staged.last = self.tracker.anchor, self.tracker.last
        staged.plane_base = self.tracker.plane_base.copy()
        candidate, reason = staged.choose(result, depth.shape)
        if candidate is None or reason != 'tracked':
            raise ValueError('support_gap_recovery_'+reason)
        if not self.rgb_instance_refinement:
            raise ValueError('support_gap_recovery_requires_rgb_refinement')
        self._check_recovery_arrival_reference(source, camera, arrival_retry_reference)
        grouped = self._qualified_fragment_pair(source, result, camera, arrival_retry_reference, pre_refined)
        initial_reference = (self._initial_station_support_reference(source, expected=reference)
            if reference.get('initial_station_current_support') is not None else None)
        if reference.get('initial_station_current_support') is not None and initial_reference is None:
            raise ValueError('initial_station_identity_template_unqualified')
        template = (self._initial_station_identity_template(source, initial_reference)
            if initial_reference is not None else None)
        identity_reference = reference if template is None else template
        rgb = self.bridge.imgmsg_to_cv2(source[0], desired_encoding='bgr8')
        refined, identity = metric_seeded_identity_evidence(rgb, [pre_refined[1]] if grouped else [candidate],
            identity_reference['bgr'], identity_reference['mask'], minimum_quality=self.minimum_quality,
            rgb_valid=rectification_rgb_valid_mask(camera, depth.shape),
            refine=lambda measured: (pre_refined[1] if grouped and measured is pre_refined[1] else refine_candidate(rgb, depth, intrinsics, transform,
                measured, self.config,
                **self._arrival_rgb_spatial_kwargs(arrival_retry_reference),
                **_rgb_source_kwargs(camera, depth.shape))))
        if grouped:
            identity.update(native_role='final_current_rgb_instance',
                observation_fragment_group=pre_refined[1].metrics['observation_fragment_group'])
        self._check_recovery_arrival_reference(source, camera, arrival_retry_reference)
        if template is not None:
            if self._initial_station_identity_template(source, initial_reference) is not template:
                raise ValueError('initial_station_identity_template_replaced')
            identity['identity_template_source'] = deepcopy(template['provenance'])
        self._qualify_support_gap_source(source, reference)
        self._locked_source_status = dict(self._locked_source_pending,
            state='qualified_unique_measured_recovery', source_stamp_ns=source[0].header.stamp.to_nsec(),
            source_qualified=True, identity_verified=True, support_measurement_available=True,
            support_audit=support_audit, rgb_identity=identity, metric_geometry_authority=False,
            motion_authorized=False, reference_refreshed=False, maximum_hold_seconds=min(self.max_age, .8))
        return staged, refined[0]

    def _same_station_zero_candidate_rgb_hold(self, source, result, camera):
        """Preserve identity for a brief gap or bounded contained fragments.

        The original last-good stamp fixes the entire window. RGB-only frames
        never refresh it, change the anchor or recover an already lost tracker.
        """
        reference = getattr(self, '_locked_rgb_hold_reference', None)
        pending = getattr(self, '_locked_source_pending', None)
        candidates = getattr(result, 'candidates', (None, None))
        if reference is None or self.tracker.anchor is None or self.tracker.lost:
            return False
        self._locked_rgb_hold_status = dict(state='rejected', metric_geometry_authority=False,
            motion_authorized=False, reference_source_stamp_ns=reference['source_stamp_ns'],
            reference_rgb_sha256=reference['source_rgb_sha256'],
            reference_mask_sha256=reference['refined_mask_sha256'])
        try:
            association_policy = HandoffPolicy()
            if not isinstance(candidates, (list, tuple)):
                raise ValueError('locked_rgb_hold_fragment_candidates_invalid')
            if len(candidates) > association_policy.maximum_candidates:
                raise ValueError('locked_rgb_hold_fragment_candidate_budget')
            fragments = []
            metric_path = False
            seen_pixels = np.zeros(reference['image_shape'], dtype=bool) if candidates else None
            for candidate in candidates:
                fragment = self._locked_rgb_shrinking_fragment(candidate, reference, source, camera)
                if fragment is None:
                    # A potentially usable metric candidate keeps the original
                    # tracker/refinement path, even among shrinking fragments.
                    if pending is None:
                        self._locked_rgb_hold_status = None
                        return False
                    metric_path = True
                    continue
                pixels = fragment['mask'] != 0
                if np.any(seen_pixels & pixels):
                    raise ValueError('locked_rgb_hold_fragment_masks_overlap')
                # This temporary occupancy mask checks disjointness only. It
                # never becomes an object mask or candidate geometry.
                seen_pixels |= pixels
                fragments.append(fragment)
            # This is now identity-only waiting, not a normal measured frame.
            # Its original non-renewable reference window remains independent.
            self._arrival_frame_completion = None
            stamp_ns = source[0].header.stamp.to_nsec()
            self._check_pending_source_reference()
            if pending is not None and stamp_ns <= pending['last_unqualified_source_stamp_ns']:
                raise ValueError('pending_source_requires_new_frame')
            now_ns = rospy.Time.now().to_nsec()
            if (stamp_ns <= reference['source_stamp_ns']
                    or source[1].header.stamp.to_nsec() != stamp_ns
                    or self.last_good_source_stamp_ns != reference['source_stamp_ns']
                    or (self.generation, self.selection_ns) !=
                       (reference['generation'], reference['selection_ns'])):
                raise ValueError('locked_rgb_hold_source_sequence_changed')
            require_fresh_source(reference['source_stamp_ns'], now_ns, min(self.max_age, .8))
            require_fresh_source(stamp_ns, now_ns, min(self.max_age, .8))
            if (reference['pose_invalidated'] or digest(asdict(self.config)) != reference['config_sha256']
                    or _candidate_fingerprint(self.tracker.anchor) != reference['anchor_fingerprint']
                    or _candidate_fingerprint(self.tracker.last) != reference['last_fingerprint']
                    or not np.array_equal(self.tracker.plane_base, reference['reference_plane'])):
                raise ValueError('locked_rgb_hold_reference_changed')
            context = self._rgb_interest_source_context(source[0].header.stamp)
            if digest(context) != reference['source_id']:
                raise ValueError('locked_rgb_hold_source_context_changed')
            feedback_rejection = None
            try:
                records, (q, before, after) = self._rgb_bracketed_feedback(stamp_ns, reference['epoch_ns'])
            except ValueError as exc:
                if str(exc) != 'accepted_feedback_does_not_bracket_source':
                    raise
                feedback_rejection = deepcopy(getattr(exc, 'accepted_feedback_evidence',
                    dict(accepted_before=None, accepted_after=None, span_ns=None,
                         maximum_span_ns=200_000_000, bracket_qualified=False)))
            else:
                if any(np.max(np.abs(np.asarray(values)-reference['q_source'])) > .004
                       for values in (q, before[1], after[1])):
                    raise ValueError('locked_rgb_hold_pose_changed')
            metrics = result.metrics
            if metrics.get('support_current_audit_available') is not True:
                raise ValueError('locked_rgb_hold_requires_current_support_audit')
            if metrics.get('support_source') == 'current_ring_fit':
                _, support_audit = _validate_support_plane_update(reference['reference_plane'],
                    result.plane_base, self.tracker.anchor.position_base, self.config,
                    reference['reference_footprint'])
            elif pending is not None and metric_path and metrics.get('support_source') == 'locked_base_plane':
                # The segmenter already fit and validated independent CURRENT
                # support. Its locked segmentation plane is not that new fit.
                support_audit = {key: metrics[key] for key in ('support_reference_angle_deg',
                    'support_reference_anchor_distance_m', 'support_reference_max_distance_m',
                    'support_reference_footprint_vertices')}
                angle = support_audit['support_reference_angle_deg']
                shift = support_audit['support_reference_max_distance_m']
                if (not np.isfinite(angle) or not np.isfinite(shift) or angle < 0. or shift < 0.
                        or angle > self.config.support_max_normal_change_deg + 1e-9
                        or shift > self.config.support_max_local_shift_m + 1e-12
                        or support_audit['support_reference_footprint_vertices'] != len(reference['reference_footprint'])):
                    raise ValueError('pending_source_current_support_audit_invalid')
                support_audit['source'] = 'independent_current_support_audit'
            else:
                raise ValueError('locked_rgb_hold_requires_current_support_audit')
            current_mask, rgb_iou = self._locked_rgb_outline_match(source, camera, reference)
            if fragments:
                for fragment in fragments:
                    fragment['evidence'].update(self._locked_rgb_fragment_ownership(
                        fragment['mask'], reference['mask'], current_mask))
            # Validate again after image work, before the false/empty publication.
            now_ns = rospy.Time.now().to_nsec()
            require_fresh_source(reference['source_stamp_ns'], now_ns, min(self.max_age, .8))
            require_fresh_source(stamp_ns, now_ns, min(self.max_age, .8))
            if digest(self._rgb_interest_source_context(source[0].header.stamp)) != reference['source_id']:
                raise ValueError('locked_rgb_hold_source_context_changed')
            with self.input_lock:
                # Context validation may wait for exact TF. Its work cannot
                # reuse the earlier age sample to extend either source window.
                now_ns = rospy.Time.now().to_nsec()
                require_fresh_source(reference['source_stamp_ns'], now_ns, min(self.max_age, .8))
                require_fresh_source(stamp_ns, now_ns, min(self.max_age, .8))
                newest = max(self._rgb_interest_feedback, key=lambda item:item[0], default=None)
                measured_since_reference = [values for ns, values in self._rgb_interest_feedback
                                            if ns >= reference['source_stamp_ns']]
                if (self._locked_rgb_hold_reference is not reference or reference['pose_invalidated']
                        or self._rgb_interest_epoch != reference['epoch_ns'] or newest is None
                        or not measured_since_reference
                        or any(np.max(np.abs(values-reference['q_source'])) > .004
                               for values in measured_since_reference)):
                    raise ValueError('locked_rgb_hold_changed_during_processing')
                require_fresh_source(int(newest[0]), now_ns, min(self.max_age, .8))
            if feedback_rejection is not None:
                self._locked_source_pending = dict(
                    reference_source_stamp_ns=reference['source_stamp_ns'], source_id=reference['source_id'],
                    first_unqualified_source_stamp_ns=(stamp_ns if pending is None else
                        pending['first_unqualified_source_stamp_ns']), last_unqualified_source_stamp_ns=stamp_ns)
                self._locked_source_status = dict(self._locked_source_pending,
                    state='pending_source_qualification', source_stamp_ns=stamp_ns,
                    source_qualified=False, identity_verified=False,
                    metric_geometry_authority=False, motion_authorized=False, reference_refreshed=False,
                    reason='accepted_feedback_does_not_bracket_source', feedback_rejection=feedback_rejection,
                    elapsed_since_measured_sec=(now_ns-reference['source_stamp_ns'])*1e-9,
                    maximum_hold_seconds=min(self.max_age, .8), mask_iou=rgb_iou,
                    support_audit=support_audit,
                    fragment_evidence_list=[dict(fragment['evidence']) for fragment in fragments])
                self._locked_rgb_hold_status = None
                return True
            if pending is not None:
                self._locked_source_status = dict(state='qualified_new_frame', source_stamp_ns=stamp_ns,
                    reference_source_stamp_ns=reference['source_stamp_ns'], source_qualified=True,
                    metric_geometry_authority=False, motion_authorized=False, reference_refreshed=False,
                    support_audit=support_audit, mask_iou=rgb_iou)
                if metric_path:
                    self._locked_rgb_hold_status = None
                    return False
                self._locked_source_pending = None
            fragment_evidence = [dict(fragment['evidence']) for fragment in fragments]
            for evidence in fragment_evidence:
                evidence['kind'] = ('boundary_consistent_shrinking_fragment' if evidence['boundary_tolerance_used']
                    else 'unique_contained_shrinking_fragment' if len(fragments) == 1
                    else 'contained_shrinking_fragment')
            self._locked_rgb_hold_status.update(state='holding_identity_only',
                source_stamp_ns=stamp_ns, mask_iou=rgb_iou,
                elapsed_since_measured_sec=(now_ns-reference['source_stamp_ns'])*1e-9,
                maximum_hold_seconds=min(self.max_age, .8), support_audit=support_audit,
                reference_refreshed=False, depth_gap_kind=('zero_candidates' if not fragments else
                    'rgb_boundary_consistent_shrinking_fragments' if any(
                        evidence['boundary_tolerance_used'] for evidence in fragment_evidence) else
                    'unique_contained_shrinking_fragment' if len(fragments) == 1 else
                    'disjoint_contained_shrinking_fragments'),
                fragment_evidence=fragment_evidence[0] if len(fragments) == 1 else None,
                fragment_evidence_list=fragment_evidence, fragment_count=len(fragments),
                maximum_fragment_candidates=association_policy.maximum_candidates,
                fragment_masks_disjoint=True)
            return True
        except (ValueError, KeyError, TypeError, cv2.error,
                tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            self._locked_rgb_hold_status['reason'] = str(exc)
            return False

    def _measurement_ack(self, request, accepted, reason='OK', qualified=False, stamps=None):
        self._observation_measurement_ack_pub.publish(String(data=json.dumps(
            measurement_ack(request, 'unknown', accepted=accepted, reason=reason,
                            qualified=qualified, source_stamps_ns=stamps), sort_keys=True)))

    def _measurement_status(self, pending, state, reason='', qualified=False):
        self._observation_measurement_status = dict(request=deepcopy(pending['request']),
            state=state, reason=str(reason), qualified=bool(qualified),
            source_stamps_ns=list(pending.get('qualified_stamps',
                pending['confirmation'].source_stamps_ns)),
            metric_geometry_authority=False, motion_authorized=False,
            measurement_evaluated=state in ('qualified','completed'),
            exact_tf_consumed=state in ('qualified','completed'))
        if pending.get('support_audit') is not None:
            self._observation_measurement_status['support_audit'] = deepcopy(pending['support_audit'])
        appearance = getattr(pending['confirmation'], 'last_appearance_evidence', None)
        if appearance:
            self._observation_measurement_status['appearance_identity'] = deepcopy(appearance)

    def _observation_appearance_source_deadline(self, source, phase_deadline):
        # Capture the remaining original source lifetime before any lazy
        # reference/ORB work. Completion cannot start a fresh time allowance.
        remaining = min(self.max_age, .8) - (rospy.Time.now()-source[0].header.stamp).to_sec()
        return min(phase_deadline, time.monotonic()+remaining)

    def _observation_marker_appearance(self, marker):
        # The original observation confirmation remains anchored before motion.
        # Only its acknowledged third full measurement supplies this immutable
        # same-station appearance template. READY inheritance never replaces it.
        station = marker.get('station_appearance')
        if station is None:
            return (marker['fragment_histogram'], marker['fragment_contour'],
                    marker.get('fragment_appearance_reference'))
        provenance = marker.get('station_appearance_provenance', {})
        if (not isinstance(station, FrozenObservationAppearance)
                or station.source_ns != marker['qualified_source_ns']
                or station.source_ns != marker['qualified_source_stamps_ns'][-1]
                or station.source_ns != provenance.get('source_stamp_ns')
                or station.rgb_sha256 != provenance.get('source_rgb_sha256')
                or station.mask_sha256 != provenance.get('normalized_mask_sha256')
                or provenance.get('role') != 'qualified_station_identity_template'
                or provenance.get('source_id') != marker['source_id']
                or provenance.get('measurement_binding') != marker['measurement_binding']
                or provenance.get('qualified_source_stamps_ns') != marker['qualified_source_stamps_ns']):
            raise ValueError('observation_station_appearance_binding_changed')
        return station.histogram, station.contour, station

    def _observation_marker_appearance_context(self, source, marker, check_current):
        def record(evidence):
            timing = getattr(self, '_processing_timing', None)
            if timing is not None:
                rows = timing.setdefault('observation_appearance_identity', [])
                rows.append(deepcopy(evidence))
                del rows[:-20]
        _, _, appearance = self._observation_marker_appearance(marker)
        context = dict(reference=appearance,
            source_ns=source[0].header.stamp.to_nsec(), check_current=check_current, record=record,
            deadline_monotonic=self._observation_appearance_source_deadline(
                source, marker['session_deadline_monotonic']))
        if marker.get('station_appearance') is not None:
            context.update(same_station_mask_iou_minimum=.85,
                reference_role='qualified_station_identity_template')
        return context

    def _recover_station_current_fragment_owner(self, source, result, candidate,
            camera, depth, intrinsics, transform):
        """Repair a partial current outline within the existing station owner.

        The exact source topology restores the current measured component.
        The fixed template checks its RGB result afterwards; it never supplies
        pixels. Ordinary downstream geometry and precision gates still apply.
        """
        frame = getattr(self, '_station_compute_frame', None)
        if frame is None:
            return candidate
        registration = frame['registration']
        marker = registration['marker']
        if marker.get('station_appearance') is None:
            return candidate
        if len(_matching_candidates(result.candidates, self.tracker.anchor,
                self.tracker.last, self.config)) != 1:
            return candidate
        if frame['source'] is not source:
            raise ValueError('current_fragment_station_source_changed')
        def check_current():
            self._station_compute_scope(registration, camera)
            if getattr(self, '_station_compute_frame', None) is not frame:
                raise ValueError('current_fragment_station_frame_changed')
            require_fresh_source(source[0].header.stamp.to_nsec(),
                rospy.Time.now().to_nsec(), min(self.max_age, .8))
        check_current()
        from alicia_grasp_modes.observation_appearance import match_observation_appearance
        rgb = self.bridge.imgmsg_to_cv2(source[0], desired_encoding='bgr8')
        valid = rectification_rgb_valid_mask(camera, depth.shape)
        histogram, contour, _ = self._observation_marker_appearance(marker)
        context = self._observation_marker_appearance_context(source, marker, check_current)
        original = match_observation_appearance(rgb, candidate.mask, histogram, contour,
            rgb_valid=valid, appearance_context=context)
        if original['code'] != 'SAME_STATION_COMPLETE_OUTLINE_MISMATCH':
            return candidate
        cache = getattr(self, '_source_sampling_topology', None)
        if cache is None:
            raise ValueError('current_source_topology_profile_not_prepared')
        topology = cache.require(camera, depth.shape, intrinsics, source[0].header.stamp.to_nsec())
        with self._rgb_timing_component('current_fragment_owner'):
            recovered = recover_source_connected_fragment_owner(result, rgb, depth, intrinsics,
                transform, self.config, self.tracker.anchor, self.tracker.last,
                marker['spatial_reference'], candidate, self.minimum_quality,
                source_topology=topology, camera=camera, source_stamp_ns=source[0].header.stamp.to_nsec(),
                rgb_valid=valid, check_current=check_current)
        if recovered is None:
            return candidate
        appearance = match_observation_appearance(rgb, recovered.owner.mask,
            histogram, contour, rgb_valid=valid, appearance_context=context)
        if not appearance['appearance_match']:
            raise ValueError('current_fragment_fixed_station_identity_required')
        check_current()
        return replace(recovered.owner, metrics=dict(recovered.owner.metrics,
            source_connected_owner=dict(recovered.audit, appearance_identity=appearance)))

    def _check_actuation_bootstrap_binding(self, request, reference=None):
        """Keep the initial synchronization handoff on its original source.

        This is only identity custody. Neither this receipt nor the frozen
        station authorizes a command, and a moved station cannot admit it.
        """
        if request['motion_kind'] != ACTUATION_BOOTSTRAP:
            return
        gcfg = rospy.get_param_cached('/grasp', {})
        validity = (gcfg.get('plan_validity_sec') if isinstance(gcfg, dict)
                    and 'plan_validity_sec' in gcfg else
                    rospy.get_param_cached('/grasp_6d/plan_validity_sec', 2.))
        deadline_ns = bootstrap_source_deadline(request, validity)
        if rospy.Time.now().to_nsec() >= deadline_ns:
            raise ValueError('actuation_bootstrap_source_expired')
        if reference is None:
            return
        marker = reference.get('initial_station_current_support')
        if (marker is None or marker.get('invalidated_reason')
                or not self._initial_station_selection_matches()
                or marker['source_stamp_ns'] != self.anchor_source_stamp_ns
                or marker['epoch_ns'] != request['control_epoch_ns']
                or marker['source_id'] != reference['source_id']
                or marker['anchor_fingerprint'] != reference['anchor_fingerprint']
                or not np.array_equal(marker['reference_plane'], reference['reference_plane'])
                or np.max(np.abs(marker['q_source'] - reference['q_source'])) > .004):
            raise ValueError('actuation_bootstrap_initial_station_changed')
        with self.input_lock:
            measured = [(ns, q) for ns, q in self._rgb_interest_feedback
                        if ns >= marker['source_stamp_ns']]
            if (self._locked_rgb_hold_reference is not reference
                    or reference.get('initial_station_current_support') is not marker
                    or marker.get('invalidated_reason') or reference['pose_invalidated']
                    or self._rgb_interest_epoch != request['control_epoch_ns']
                    or not measured
                    or any(np.max(np.abs(q - marker['q_source'])) > .004 for _, q in measured)):
                raise ValueError('actuation_bootstrap_initial_station_changed')
            require_fresh_source(max(ns for ns, _ in measured), rospy.Time.now().to_nsec(),
                                 min(self.max_age, .8))

    def _check_measurement_binding(self, pending):
        request = pending['request']
        self._check_actuation_bootstrap_binding(request)
        if request.get(STATION_REQUALIFICATION) is not None:
            if rospy.Time.now().to_nsec() >= request[STATION_REQUALIFICATION]['deadline_ns']:
                raise ValueError('STATION_REQUALIFICATION_EXPIRED')
        if (pending.get('invalidated_reason') or self.tracker.lost or self.first_loss is not None
                or self.tracker.anchor is not pending['anchor']
                or self._measurement_pure_value('anchor_fingerprint',
                    _candidate_binding_values(self.tracker.anchor), _candidate_binding_fingerprint)
                   != pending['anchor_fingerprint']
                or self._measurement_pure_value('last_fingerprint',
                    _candidate_binding_values(self.tracker.last), _candidate_binding_fingerprint)
                   != pending['last_fingerprint']
                or not np.array_equal(self.tracker.plane_base, pending['plane'])
                or self.generation != request['selection']['generation']
                or self.selection_ns != request['selection']['stamp_ns']
                or self._rgb_interest_epoch != request['control_epoch_ns']
                or self._measurement_pure_value('config_digest',
                    tuple((field.name, getattr(self.config, field.name)) for field in fields(self.config)),
                    lambda snapshot: digest(dict(snapshot))) != pending['config_sha256']):
            raise ValueError(pending.get('invalidated_reason') or 'observation_measurement_identity_changed')
        if rospy.get_param_cached('/gui/joint_direct_mode', False) is True:
            raise ValueError('observation_measurement_manual_control')
        now = rospy.Time.now().to_nsec()
        deadline = (request['capture_deadline_ns'] if request['state']=='requalifying'
                    else request['deadline_ns'])
        if now >= deadline or time.monotonic() >= pending['deadline_monotonic']:
            raise ValueError('observation_measurement_deadline_exceeded')
        stamp = rospy.Time(pending['reference_stamp_ns']//10**9,pending['reference_stamp_ns']%10**9)
        if (self._measurement_pure_value('context_digest', self._rgb_interest_source_context(stamp), digest)
                != pending['source_id']
                or self._measurement_pure_value('camera_contract_digest',
                    rospy.get_param_cached('/camera'), lambda camera: digest(_camera_configuration_contract(camera)))
                   != pending['camera_configuration_sha256']):
            raise ValueError('observation_measurement_configuration_changed')

    def _revoke_observation_measurement(self, pending, reason, source=None):
        self._measurement_status(pending, 'revoked', reason)
        self._measurement_ack(pending['request'], False, reason)
        self._observation_measurement = None
        self.unavailable('observation_measurement_rejected: '+str(reason),
                         None if source is None else source[0].header.stamp, source)

    def _observation_measurement_request_cb(self, message):
        # The task waits for both owners' receipts before submitting motion.
        # This callback freezes only a fresh, already acquired metric identity.
        request = None
        with self.lock:
            pending = getattr(self, '_observation_measurement', None)
            try:
                if len(message.data) > 32768:
                    raise ValueError('observation_measurement_request_too_large')
                offered = json.loads(message.data)
                previous = None if pending is None else pending['request']
                admission = getattr(self, '_station_requalification_admission', None)
                if previous is None and admission is not None and offered.get('state') == 'revoked':
                    previous = admission['request']
                supplement = getattr(self, '_supplement_measurement_admission', None)
                if previous is None and supplement is not None and offered.get('state') == 'revoked':
                    previous = supplement['request']
                request = parse_measurement_request(offered,
                    rospy.get_param_cached('/grasp_mode/selection'), rospy.Time.now().to_nsec(), previous)
                if self._supplement_admission_receive(request):
                    return
                if request['state'] == 'executing':
                    seen = getattr(self, '_observation_measurement_seen', ())
                    reference = getattr(self, '_locked_rgb_hold_reference', None)
                    restation = None
                    if request.get(STATION_REQUALIFICATION) is not None:
                        if getattr(self, '_station_requalification_hold', None) is None:
                            self._station_requalification_queue(request)
                            return
                        restation = self._station_requalification_check(request)
                        reference = restation['reference']
                    if (pending is not None or request['lease_id'] in seen or self.mode!='unknown'
                            or self.tracker.anchor is None or self.tracker.lost or self.first_loss is not None
                            or reference is None or (reference['pose_invalidated'] and restation is None)
                            or getattr(self, '_locked_source_pending', None) is not None
                            or getattr(self, '_metric_ambiguity_pending', None) is not None
                            or getattr(self, '_rgb_acquisition_active', None) is not None
                            or not self.anchor_source_stamp_ns <= request['origin_snapshot_stamp_ns']
                            <= request['view_source_stamp_ns'] <= self.last_good_source_stamp_ns
                            or reference['source_stamp_ns'] != self.last_good_source_stamp_ns):
                        raise ValueError('observation_measurement_fresh_locked_identity_required')
                    if restation is None:
                        try:
                            require_fresh_source(reference['source_stamp_ns'],rospy.Time.now().to_nsec(),min(self.max_age,.8))
                        except ValueError:
                            if self._supplement_admission_queue(request):
                                return
                            raise
                    self._check_actuation_bootstrap_binding(request, reference)
                    pending = dict(request=request, anchor=self.tracker.anchor,
                        anchor_fingerprint=_candidate_fingerprint(self.tracker.anchor),
                        last_fingerprint=_candidate_fingerprint(self.tracker.last),
                        plane=self.tracker.plane_base.copy(), source_id=reference['source_id'],
                        reference_stamp_ns=reference['source_stamp_ns'],config_sha256=digest(asdict(self.config)),
                        camera_configuration_sha256=digest(_camera_configuration_contract(rospy.get_param_cached('/camera'))),
                        deadline_monotonic=time.monotonic()+(request['deadline_ns']-rospy.Time.now().to_nsec())*1e-9,
                        confirmation=ObservationArrivalConfirmation(self.config,self.tracker.anchor,
                            self.tracker.last,self.tracker.plane_base,reference['histogram'],
                            reference['contour'],self.minimum_quality),arrival_q=None)
                    pending['session_deadline_monotonic'] = pending['deadline_monotonic']
                    if restation is not None:
                        pending['deadline_monotonic'] = min(pending['deadline_monotonic'],
                            restation['monotonic_deadline'], time.monotonic()+(
                                request[STATION_REQUALIFICATION]['deadline_ns']-rospy.Time.now().to_nsec())*1e-9)
                    self._check_measurement_binding(pending)
                    # Bind the source before any arrival frame is available.
                    # The optional common face uses only that source's actual
                    # depth and exact metric transform, within this same lease
                    # and source lifetime. Ordinary geometric insufficiency
                    # leaves the original whole-object appearance path intact.
                    measured_source = reference.get('measured_face_source')
                    if measured_source is not None and (
                            not isinstance(measured_source, FrozenMeasuredFaceSource)
                            or measured_source.source_ns != reference['source_stamp_ns']
                            or measured_source.source_id != reference['source_id']
                            or measured_source.rgb_sha256 != reference['source_rgb_sha256']
                            or measured_source.whole_mask_sha256 != reference['refined_mask_sha256']
                            or measured_source.depth_sha256 != reference['source_depth_sha256']
                            or measured_source.binding_sha256 != reference.get('measured_face_source_binding_sha256')
                            or digest(json.loads(measured_source.config_json)) != reference['config_sha256']
                            or not np.array_equal(measured_source.plane, reference['reference_plane'])):
                        raise ValueError('observation_measured_face_source_binding_changed')
                    measured_binding = reference.get('measured_face_source_binding_sha256')
                    freeze_deadline = pending['deadline_monotonic']
                    if restation is None:
                        freeze_deadline = min(freeze_deadline, time.monotonic()+(
                            min(self.max_age, .8) - (
                                rospy.Time.now().to_nsec()-reference['source_stamp_ns'])*1e-9))
                    def check_source_freeze():
                        self._check_measurement_binding(pending)
                        self._check_actuation_bootstrap_binding(request, reference)
                        if (self._locked_rgb_hold_reference is not reference
                                or reference.get('measured_face_source') is not measured_source
                                or reference.get('measured_face_source_binding_sha256') != measured_binding
                                or (reference['pose_invalidated'] and restation is None)
                                or reference['source_stamp_ns'] != self.last_good_source_stamp_ns):
                            raise ValueError('observation_measurement_reference_changed_during_freeze')
                        if restation is None:
                            require_fresh_source(reference['source_stamp_ns'],
                                rospy.Time.now().to_nsec(), min(self.max_age,.8))
                        else:
                            self._station_requalification_check(request)
                        if time.monotonic() >= freeze_deadline:
                            raise ValueError('observation_appearance_deadline_exceeded')
                    check_source_freeze()
                    measured_face = None
                    valid = rectification_rgb_valid_mask(
                        rospy.get_param_cached('/camera'), reference['image_shape'])
                    if measured_source is not None:
                        try:
                            measured_face = freeze_measured_face(measured_source,
                                reference['bgr'], reference['mask'], valid,
                                check_current=check_source_freeze,
                                deadline_monotonic=freeze_deadline, clock=time.monotonic)
                            if measured_face.source is not measured_source:
                                raise ValueError('observation_measured_face_source_binding_changed')
                        except MeasuredFaceUnavailable:
                            check_source_freeze()
                    pending['confirmation'].appearance_reference = FrozenObservationAppearance(
                        reference['bgr'], reference['mask'], valid,
                        reference['source_stamp_ns'], reference['histogram'], reference['contour'],
                        measured_face=measured_face)
                    check_source_freeze()
                    if restation is None:
                        try:
                            require_fresh_source(reference['source_stamp_ns'],rospy.Time.now().to_nsec(),min(self.max_age,.8))
                        except ValueError:
                            if self._supplement_admission_queue(request):
                                return
                            raise
                    else:
                        self._station_requalification_check(request)
                    if (self._locked_rgb_hold_reference is not reference
                            or (reference['pose_invalidated'] and restation is None)
                            or reference['source_stamp_ns'] != self.last_good_source_stamp_ns):
                        raise ValueError('observation_measurement_reference_changed_during_freeze')
                    with self.input_lock:
                        # SDK and epoch callbacks do not acquire self.lock.
                        # Recheck after the final lock wait, before admitting
                        # the immutable copy and publishing its accepted ACK.
                        check_source_freeze()
                        if restation is None:
                            measured = [q for ns, q in self._rgb_interest_feedback
                                if ns >= reference['source_stamp_ns']]
                            if (not measured or any(np.max(np.abs(q-reference['q_source'])) > .004
                                                    for q in measured)):
                                raise ValueError('observation_measurement_reference_changed_during_freeze')
                        self._supplement_admission_commit_check(request)
                        check_source_freeze()
                        self._observation_measurement = pending
                        if restation is not None:
                            restation['new_lease'] = request['lease_id']
                            used = getattr(self, '_station_requalification_used', set())
                            used.add(station_requalification_key(request[STATION_REQUALIFICATION]['parent_contract']))
                            self._station_requalification_used = used
                        if not hasattr(self, '_observation_measurement_seen'):
                            self._observation_measurement_seen=deque(maxlen=32)
                        self._observation_measurement_seen.append(request['lease_id'])
                        # The static cache cannot be transported through motion.
                        self._locked_rgb_hold_reference = None
                        self._arrival_frame_completion = None
                        self._station_compute_registration = None
                        self._measurement_status(pending,'executing')
                        self._measurement_ack(request,True)
                elif request['state'] == 'requalifying':
                    self._check_measurement_binding(pending)
                    pending['request']=request
                    pending['deadline_monotonic']=min(pending['deadline_monotonic'],
                        time.monotonic()+(request['capture_deadline_ns']-rospy.Time.now().to_nsec())*1e-9)
                    pending['confirmation'].clear()
                    self._measurement_status(pending,'requalifying')
                    self._measurement_ack(request,True)
                elif request['state'] == 'qualified':
                    self._check_measurement_binding(pending)
                    if request['source_stamps_ns'] != pending.get('qualified_stamps'):
                        raise ValueError('observation_measurement_completion_sources_changed')
                    require_fresh_source(request['source_stamps_ns'][-1],rospy.Time.now().to_nsec(),min(self.max_age,.8))
                    hold = self._locked_rgb_hold_reference
                    if (hold is None or hold['source_stamp_ns'] != request['source_stamps_ns'][-1]
                            or pending['arrival_q'] is None):
                        raise ValueError('observation_measurement_qualified_station_unavailable')
                    previous_request = deepcopy(pending['request'])
                    # Copy pixels without holding the input callback lock. The
                    # final owner/source check below covers this entire cost.
                    marker = dict(
                        measurement_binding=deepcopy({key: request[key] for key in (
                            'selection', 'lease_id', 'session_id', 'view_plan_id', 'view_index',
                            'target_epoch', 'target_track_id', 'control_epoch_ns')}),
                        lease_id=request['lease_id'], qualified_source_ns=hold['source_stamp_ns'],
                        qualified_source_stamps_ns=list(request['source_stamps_ns']),
                        arrival_q=np.array(pending['arrival_q'], copy=True),
                        epoch_ns=request['control_epoch_ns'], source_id=hold['source_id'],
                        config_sha256=hold['config_sha256'],
                        camera_configuration_sha256=pending['camera_configuration_sha256'],
                        anchor_fingerprint=hold['anchor_fingerprint'],
                        reference_plane=np.array(pending['plane'], copy=True),
                        spatial_reference=pending['confirmation'].spatial_reference,
                        fragment_histogram=pending['confirmation'].histogram,
                        fragment_contour=pending['confirmation'].contour,
                        fragment_appearance_reference=pending['confirmation'].appearance_reference,
                        station_appearance=FrozenObservationAppearance(
                            hold['bgr'], hold['mask'], rectification_rgb_valid_mask(
                                rospy.get_param_cached('/camera'), hold['image_shape']),
                            hold['source_stamp_ns'], hold['histogram'], hold['contour']),
                        session_deadline_ns=request['deadline_ns'],
                        session_deadline_monotonic=pending['session_deadline_monotonic'])
                    template = marker['station_appearance']
                    marker['station_appearance_provenance'] = dict(
                        version=1, role='qualified_station_identity_template',
                        source_stamp_ns=template.source_ns, source_id=hold['source_id'],
                        source_rgb_sha256=hold['source_rgb_sha256'],
                        refined_mask_sha256=hold['refined_mask_sha256'],
                        normalized_mask_sha256=template.mask_sha256,
                        premotion_source_stamp_ns=marker['fragment_appearance_reference'].source_ns,
                        qualified_source_stamps_ns=list(request['source_stamps_ns']),
                        measurement_binding=deepcopy(marker['measurement_binding']))
                    with self.input_lock:
                        # SDK/epoch callbacks only hold input_lock. A completed
                        # immutable copy cannot authorize its earlier snapshot.
                        self._check_measurement_binding(pending)
                        stamp_ns = request['source_stamps_ns'][-1]
                        stamp = rospy.Time(stamp_ns//10**9, stamp_ns%10**9)
                        if (self._observation_measurement is not pending
                                or pending['request'] != previous_request
                                or request['source_stamps_ns'] != pending.get('qualified_stamps')
                                or self._locked_rgb_hold_reference is not hold
                                or hold['source_stamp_ns'] != stamp_ns
                                or self.last_good_source_stamp_ns != stamp_ns
                                or hold['pose_invalidated']
                                or (hold['generation'], hold['selection_ns'], hold['epoch_ns']) !=
                                   (self.generation, self.selection_ns, self._rgb_interest_epoch)
                                or hold['source_id'] != marker['source_id']
                                or digest(self._rgb_interest_source_context(stamp)) != marker['source_id']
                                or hold['config_sha256'] != marker['config_sha256']
                                or hold['anchor_fingerprint'] != marker['anchor_fingerprint']
                                or hold['last_fingerprint'] != pending['last_fingerprint']
                                or not np.array_equal(pending['arrival_q'], marker['arrival_q'])
                                or template.rgb_sha256 != hold['source_rgb_sha256']):
                            raise ValueError('observation_measurement_reference_changed_during_freeze')
                        measured = [q for ns, q in self._rgb_interest_feedback
                            if ns >= previous_request['settle']['stationary_window_start_ns']]
                        measured.extend((hold['q_source'], hold['accepted_before'][1],
                                         hold['accepted_after'][1]))
                        if any(np.max(np.abs(np.asarray(q)-marker['arrival_q'])) > .004
                               for q in measured):
                            pending['invalidated_reason'] = 'movement_after_stationary_arrival'
                            raise ValueError(pending['invalidated_reason'])
                        # Context/TF lookups above also consume the unchanged
                        # third-frame lifetime; no timestamp is renewed.
                        self._check_measurement_binding(pending)
                        require_fresh_source(stamp_ns, rospy.Time.now().to_nsec(), min(self.max_age,.8))
                        hold['observation_arrival_retry'] = marker
                        pending['request']=request
                        self._measurement_status(pending,'completed',qualified=True)
                        self._measurement_ack(request,True,qualified=True,stamps=request['source_stamps_ns'])
                        self._observation_measurement=None
                        if request.get(STATION_REQUALIFICATION) is not None:
                            self._station_requalification_hold['finalizing'] = True
                else:
                    admission = getattr(self, '_station_requalification_admission', None)
                    if pending is None and admission is not None:
                        parse_measurement_request(request, request['selection'], rospy.Time.now().to_nsec(),
                            admission['request'])
                        self._station_requalification_admission = None
                        self._station_requalification_hold = None
                        self._measurement_ack(request,False,request.get('reason','task_revoked'))
                        self.unavailable('observation_measurement_rejected: task_revoked')
                    else:
                        self._revoke_observation_measurement(pending,request.get('reason','task_revoked'))
            except (ValueError, KeyError, TypeError, AttributeError, tf2_ros.LookupException,
                    tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
                if getattr(self, '_observation_measurement', None) is not None and request is not None:
                    self._revoke_observation_measurement(pending,str(exc))
                elif request is not None:
                    self._measurement_ack(request,False,str(exc))
                else:
                    rospy.logwarn('Unknown measurement receipt rejected: %s',exc)

    def _process_observation_measurement(self, source):
        pending = getattr(self, '_observation_measurement', None)
        if pending is None:
            return False
        try:
            self._check_measurement_binding(pending)
        except (ValueError, KeyError, TypeError, AttributeError, tf2_ros.LookupException,
                tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            self._revoke_observation_measurement(pending,str(exc),source)
            return True
        request=pending['request']
        if pending.get('qualified_stamps'):
            # Receipt waiting never renews this measured source, even when the
            # image stream stops and the timer has no new pair to consume.
            try:
                require_fresh_source(pending['qualified_stamps'][-1],rospy.Time.now().to_nsec(),min(self.max_age,.8))
            except ValueError as exc:
                self._revoke_observation_measurement(pending,str(exc),source)
            return True
        if request['state']=='executing' or source is None:
            self._measurement_status(pending,request['state'],'waiting_for_stationary_measurements')
            self.status('pending','observation_measurement_pending',source)
            return True
        color,depth_message=source
        source_ns=color.header.stamp.to_nsec()
        if source_ns<=request['arrival_stamp_ns']:
            self._measurement_status(pending,'requalifying','waiting_for_post_arrival_source')
            self.status('pending','observation_measurement_pending',source)
            return True
        try:
            require_fresh_source(source_ns,rospy.Time.now().to_nsec(),min(self.max_age,.8))
            if (source_ns!=depth_message.header.stamp.to_nsec()
                    or color.header.frame_id!=self.camera_frame or depth_message.header.frame_id!=self.camera_frame
                    or (color.width,color.height)!=(depth_message.width,depth_message.height)):
                raise ValueError('observation_measurement_rgbd_binding_invalid')
            camera=rospy.get_param('/camera',{})
            require_rectified_source_after_activation(camera,source_ns)
            if camera.get('align_depth_to_color') is not True:
                raise ValueError('aligned_depth_required')
            intrinsics=[float(camera[k]) for k in ('fx','fy','cx','cy')]
            array=self.bridge.imgmsg_to_cv2(depth_message,desired_encoding='passthrough')
            if depth_message.encoding in ('16UC1','mono16'):
                scale=float(camera['depth_scale'])
                if not np.isfinite(scale) or scale<=0:raise ValueError('invalid_runtime_depth_scale')
                depth=np.asarray(array,dtype=np.float32)*scale
            elif depth_message.encoding=='32FC1':depth=np.asarray(array,dtype=np.float32)
            else:raise ValueError('unsupported_depth_encoding')
            wait=exact_tf_wait_budget((rospy.Time.now()-color.header.stamp).to_sec(),self.max_age,self.tf_wait)
            transform=transform_matrix(self.buffer.lookup_transform(self.base_frame,self.camera_frame,
                color.header.stamp,rospy.Duration(wait)))
            feedback,(q,before,after)=self._rgb_bracketed_feedback(source_ns,request['control_epoch_ns'])
            settle_start=request['settle']['stationary_window_start_ns']
            stationary=[(ns,values) for ns,values in feedback if settle_start<=ns<=after[0]]
            if (len(stationary)<3 or stationary[-1][0]-stationary[0][0]<250_000_000
                    or stationary[0][0]-settle_start>200_000_000
                    or any(b[0]-a[0]>200_000_000 for a,b in zip(stationary,stationary[1:]))):
                raise ValueError('observation_measurement_stationary_feedback_incomplete')
            if any(np.max(np.abs(values-q))>.004 for _,values in stationary):
                pending['invalidated_reason']='movement_after_stationary_arrival'
                raise ValueError(pending['invalidated_reason'])
            pending['arrival_q']=np.array(q,copy=True)
            result=segment_with_support_fallback(depth,intrinsics,transform,self.config,
                anchor=pending['anchor'],last=pending['confirmation'].last,
                reference_plane_base=pending['plane'],prefer_current_support=True)
            pending['support_audit'] = dict(
                ((key, value) for key, value in result.metrics.items()
                 if key.startswith('support_reference_') or key.startswith('support_current_audit_')),
                source_stamp_ns=source_ns)
            rgb=self.bridge.imgmsg_to_cv2(color,desired_encoding='bgr8')
            def check_appearance_source():
                self._check_measurement_binding(pending)
                require_fresh_source(source_ns,rospy.Time.now().to_nsec(),min(self.max_age,.8))
            confirmed,candidate,staged,evidence=pending['confirmation'].observe(
                result,rgb,depth,intrinsics,transform,source_ns,
                appearance_deadline_monotonic=self._observation_appearance_source_deadline(source,pending['deadline_monotonic']),
                appearance_check_current=check_appearance_source,**_rgb_source_kwargs(camera,depth.shape))
            self._check_measurement_binding(pending)
            require_fresh_source(source_ns,rospy.Time.now().to_nsec(),min(self.max_age,.8))
            if not confirmed:
                self._measurement_status(pending,'requalifying',evidence['reason'])
                self.status('pending','observation_measurement_pending',source)
                return True
            stamps=pending['confirmation'].source_stamps_ns
            # Each historical confirmation passed freshness before and after
            # its own audit. Only this final frame supplies current geometry;
            # the fixed capture window bounds the earlier identity evidence.
            hold=self._prepare_locked_rgb_hold_reference(source,candidate,camera,tracker=staged,
                                                        source_transform=transform)
            if hold is None:raise ValueError('observation_measurement_new_rgb_reference_required')
            with self.input_lock:
                self._check_measurement_binding(pending)
                seen=[values for ns,values in self._rgb_interest_feedback if ns>=settle_start]
                if not seen or any(np.max(np.abs(values-q))>.004 for values in seen):
                    pending['invalidated_reason']='movement_after_stationary_arrival'
                    raise ValueError(pending['invalidated_reason'])
                require_fresh_source(source_ns,rospy.Time.now().to_nsec(),min(self.max_age,.8))
                self.tracker.last=staged.last
                pending['last_fingerprint']=_candidate_fingerprint(self.tracker.last)
                pending['qualified_stamps']=list(stamps)
                self._locked_rgb_hold_reference=hold
                self.last_shape=depth.shape
                self._measurement_status(pending,'qualified',qualified=True)
                self._publish_measurement_candidate(source,candidate)
                self.last_good_source_stamp_ns=source_ns
                self.status('ready','observation_measurement_requalified',source,candidate)
                self._measurement_ack(request,True,qualified=True,stamps=stamps)
        except (ValueError, KeyError, TypeError, AttributeError, cv2.error,tf2_ros.LookupException,
                tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException) as exc:
            pending['confirmation'].clear(preserve_appearance=True)
            if pending.get('invalidated_reason'):
                self._revoke_observation_measurement(pending,str(exc),source)
            else:
                self._measurement_status(pending,'requalifying',str(exc))
                self.status('pending','observation_measurement_pending',source)
        return True

    def _publish_measurement_candidate(self, source, candidate):
        color=source[0]
        obj=ObjectPose();obj.header.stamp=color.header.stamp;obj.header.frame_id=self.base_frame
        obj.header.seq=color.header.seq
        obj.detected,obj.label,obj.confidence=True,'unknown_small',candidate.quality
        obj.u,obj.v=[int(round(v)) for v in candidate.center_uv]
        obj.bbox_x,obj.bbox_y,obj.bbox_width,obj.bbox_height=candidate.bbox
        obj.depth_m=candidate.depth_m
        for name,point,frame in (('pose_camera',candidate.position_camera,self.camera_frame),
                                 ('pose_base',candidate.position_base,self.base_frame)):
            pose=PoseStamped();pose.header.stamp=color.header.stamp;pose.header.frame_id=frame
            pose.header.seq=color.header.seq
            pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=point
            pose.pose.orientation.w=1.
            setattr(obj,name,pose);self.pubs['object_'+name].publish(pose)
        mask=self.bridge.cv2_to_imgmsg(candidate.mask,encoding='mono8');mask.header=color.header
        self.pubs['object_mask'].publish(mask);self.pubs['object'].publish(obj)
        self.pubs['object_detected'].publish(Bool(data=True));self.pubs['raw_object_detected'].publish(Bool(data=True))

    def unavailable(self, reason, stamp=None, source=None, lose=True, candidate_evidence=None,
                    preserve_rgb_interest=False):
        if reason not in ('processing_deadline_pending', 'station_accepted_feedback_bracket_pending_revalidation'):
            self._station_compute_registration = None
        # RGB interest is a separate weak observation channel. Any other
        # perception failure retires it; preservation is an explicit initial
        # acquisition path, never a reason-string whitelist or target recovery.
        with self.input_lock:
            interest = getattr(self, '_rgb_observation_interest', None) or {}
            epoch = getattr(self, '_rgb_interest_epoch', None)
            version = getattr(self, '_rgb_interest_version', None)
        preserve = (preserve_rgb_interest is True and self.mode == 'unknown'
                    and self.tracker.anchor is None and not self.tracker.lost
                    and source is not None
                    and interest.get('source_stamp_ns') == source[0].header.stamp.to_nsec()
                    and source[1].header.stamp.to_nsec() == source[0].header.stamp.to_nsec()
                    and interest.get('generation', self.generation) == self.generation)
        if preserve and 'source_id' in interest:
            try:
                require_fresh_source(interest['source_stamp_ns'], rospy.Time.now().to_nsec(),
                                     min(self.max_age, .8))
                if (interest.get('epoch_ns') != epoch or interest.get('input_version') != version
                        or interest.get('selection_stamp_ns') != self.selection_ns):
                    raise ValueError('observation_binding_changed_before_status')
            except ValueError as exc:
                # Preserve only this tombstone, not the preceding weak record.
                self._clear_rgb_interest(str(exc), interest['source_stamp_ns'])
        if not preserve:
            self._clear_rgb_interest(reason)
        if self.tracker.anchor is not None and (lose or self.tracker.lost) and self.first_loss is None:
            anchor = self.tracker.anchor
            source_ns = source[0].header.stamp.to_nsec() if source is not None else None
            self.first_loss = first_loss_observation(None, reason=reason,
                generation=self.generation, selection_stamp_ns=self.selection_ns,
                observed_stamp_ns=rospy.Time.now().to_nsec(), source_stamp_ns=source_ns,
                anchor_source_stamp_ns=self.anchor_source_stamp_ns,
                anchor_position_base=anchor.position_base.tolist(), anchor_extent_m=anchor.extent_m.tolist(),
                reference_support_plane_base=self.tracker.plane_base.tolist(),
                last_good_source_stamp_ns=self.last_good_source_stamp_ns,
                last_consumed_source_stamp_ns=self.last_consumed_ns,
                candidate_evidence=candidate_evidence)
            rospy.logwarn('Unknown target first loss generation=%s source=%s anchor=%s reason=%s',
                          self.generation, source_ns, self.anchor_source_stamp_ns, reason)
        if lose:
            self._station_compute_registration = None
            self._locked_rgb_hold_reference = None
            self._locked_source_pending = None
            self._metric_ambiguity_pending = None
            self.tracker.unavailable()
        stamp = stamp or rospy.Time.now()
        obj = ObjectPose()
        obj.header.stamp, obj.header.frame_id = stamp, self.base_frame
        obj.label, obj.detected = 'unknown_small', False
        mask = self.bridge.cv2_to_imgmsg(np.zeros(self.last_shape, dtype=np.uint8), encoding='mono8')
        mask.header.stamp, mask.header.frame_id = stamp, self.camera_frame
        self.pubs['object_mask'].publish(mask)
        self.pubs['object'].publish(obj)
        self.pubs['object_detected'].publish(Bool(data=False))
        self.pubs['raw_object_detected'].publish(Bool(data=False))
        self.status('unavailable' if self.mode == 'unknown' else 'inactive', reason, source)

    def process(self, _event):
        with self.lock:
            self._service_rgb_reference_request()
            with self.input_lock:
                if self.mode != 'unknown':
                    return
                source, self.latest = self.latest, None
                ingress = getattr(self, '_latest_frame_timing', None)
                self._latest_frame_timing = None
                self._processing_timing = None
                self._arrival_frame_completion = None
                self._station_compute_frame = None
                self._locked_rgb_hold_status = None
                self._locked_source_status = None
                self._metric_ambiguity_status = None
                generation = self.generation
                contact_station_version = getattr(self, '_contact_station_version', 0)
                self._arrival_phase_begin_frame(source)
                if source is not None:
                    self.last_consumed_ns = source[0].header.stamp.to_nsec()
            if self._station_requalification_admission_tick(source):
                return
            try:
                self._station_requalification_finalizing_check()
            except (ValueError, KeyError, TypeError, AttributeError) as exc:
                self._station_requalification_hold = None
                self.unavailable('station_requalification_rejected: '+str(exc),
                    None if source is None else source[0].header.stamp, source)
                return
            if self._process_observation_measurement(source):
                return
            if self._station_requalification_wait(source):
                return
            if getattr(self, '_metric_ambiguity_pending', None) is not None:
                try:
                    reference = self._check_metric_ambiguity_reference()
                    ref_stamp = rospy.Time(reference['source_stamp_ns']//10**9,
                                           reference['source_stamp_ns']%10**9)
                    if digest(self._rgb_interest_source_context(ref_stamp)) != reference['source_id']:
                        raise ValueError('metric_ambiguity_source_context_changed')
                    self._check_metric_ambiguity_reference(reference)
                except (ValueError, KeyError, TypeError, tf2_ros.LookupException,
                        tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
                    self._metric_ambiguity_status = dict(state='rejected', reason=str(exc),
                        metric_geometry_authority=False, motion_authorized=False)
                    self.unavailable('metric_ambiguity_rejected: '+str(exc),
                        None if source is None else source[0].header.stamp, source)
                    return
            if getattr(self, '_locked_source_pending', None) is not None:
                try:
                    self._check_pending_source_reference()
                    reference = self._locked_rgb_hold_reference
                    ref_stamp = rospy.Time(reference['source_stamp_ns']//10**9,
                                           reference['source_stamp_ns']%10**9)
                    if digest(self._rgb_interest_source_context(ref_stamp)) != reference['source_id']:
                        raise ValueError('pending_source_context_changed')
                    self._check_pending_source_reference()
                except (ValueError, KeyError, TypeError, tf2_ros.LookupException,
                        tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
                    self._locked_source_status = dict(self._locked_source_pending,
                        state='rejected', reason=str(exc), source_qualified=False,
                        identity_verified=False, metric_geometry_authority=False, motion_authorized=False)
                    self.unavailable('source_qualification_rejected: '+str(exc),
                        None if source is None else source[0].header.stamp, source)
                    return
            if source is None:
                if self._station_compute_idle_wait():
                    return
                # The station audit may yield while frame() fills the input
                # slot. Commit a stream-loss verdict against current ingress
                # under its own lock, not the empty snapshot taken above.
                # A queued pair retains its original source stamp and must
                # pass the ordinary age/identity checks on the next process.
                with self.input_lock:
                    if (self.latest is None
                            and time.monotonic() - self.last_receive_wall > self.max_age):
                        if self._station_compute_idle_feedback_updated():
                            return
                        audit = dict(getattr(self, '_station_compute_idle_audit', None) or {})
                        audit.update(diagnostic_only=True, latest_pair_present=False,
                            decision_monotonic_sec=time.monotonic(),
                            decision_ros_ns=rospy.Time.now().to_nsec(),
                            last_receive_monotonic_sec=self.last_receive_wall,
                            maximum_frame_age_sec=self.max_age,
                            latest_feedback_stamp_ns=max((ns for ns, _ in
                                getattr(self, '_rgb_interest_feedback', ())), default=None))
                        self.unavailable('exact_rgbd_stream_stale',
                            candidate_evidence=dict(rgbd_idle_audit=audit))
                return
            color, depth_message = source
            anchor = self.tracker.anchor
            last = self.tracker.last
            plane = None if self.tracker.plane_base is None else self.tracker.plane_base.copy()
        started = time.monotonic()
        self._begin_frame_timing(source, ingress, started)
        try:
            age = (rospy.Time.now() - color.header.stamp).to_sec()
            if not (0 <= age <= self.max_age):
                raise ValueError('source_frame_stale_or_future')
            self._station_compute_begin_frame(source)
            if (color.header.frame_id != self.camera_frame
                    or depth_message.header.frame_id != self.camera_frame):
                raise ValueError('aligned_rgbd_frame_mismatch')
            if (color.width, color.height) != (depth_message.width, depth_message.height):
                raise ValueError('unaligned_rgbd_dimensions')
            # Record timely entry BEFORE decode, dynamic TF or the phase's
            # context/feedback qualification can wait. This token supplies no
            # station, identity or geometry authority: the unchanged complete
            # source and marker checks must still pass before segmentation.
            with self.lock:
                entry_reference = getattr(self, '_locked_rgb_hold_reference', None)
                if (entry_reference or {}).get('observation_arrival_retry') is not None:
                    self._begin_station_frame_completion(
                        source, entry_reference, 'observation_arrival_retry')
                elif (entry_reference or {}).get('initial_station_current_support') is not None:
                    self._begin_station_frame_completion(
                        source, entry_reference, 'initial_station_current_support')
            self._frame_timing_stage('decode')
            with self._decode_timing_component('camera_parameter_read'):
                camera = rospy.get_param('/camera', {})
            with self._decode_timing_component('rectified_source_validation'):
                require_rectified_source_after_activation(camera, color.header.stamp.to_nsec())
                cache = getattr(self, '_source_sampling_topology', None)
                if cache is None:
                    cache = self._source_sampling_topology = SourceSamplingTopologyCache()
                try:
                    cache.offer(camera)
                except ValueError:
                    # Prewarming is optional for the original perception path.
                    # Its source gate above still applies. Guarded recovery
                    # calls cache.require and rechecks the complete profile;
                    # missing or inconsistent topology proof cannot recover.
                    pass
            if not bool(camera.get('align_depth_to_color', False)):
                raise ValueError('aligned_depth_required')
            intrinsics = [float(camera[name]) for name in ('fx', 'fy', 'cx', 'cy')]
            with self._decode_timing_component('cv_bridge_conversion'):
                array = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding='passthrough')
            with self._decode_timing_component('float32_depth_scale'):
                if depth_message.encoding in ('16UC1', 'mono16'):
                    scale = float(camera['depth_scale'])
                    if not np.isfinite(scale) or scale <= 0:
                        raise ValueError('invalid_runtime_depth_scale')
                    depth = np.asarray(array, dtype=np.float32) * scale
                elif depth_message.encoding == '32FC1':
                    depth = np.asarray(array, dtype=np.float32)
                else:
                    raise ValueError('unsupported_depth_encoding')
            self._frame_timing_stage('before_tf')
            wait = exact_tf_wait_budget(
                (rospy.Time.now() - color.header.stamp).to_sec(), self.max_age, self.tf_wait)
            self._frame_timing_stage('tf_lookup')
            transform = self.buffer.lookup_transform(self.base_frame, self.camera_frame,
                                                     color.header.stamp, rospy.Duration(wait))
            self._frame_timing_stage('post_tf')
            with self.lock:
                if self._arrival_phase_checkpoint(source, camera):
                    return
            # TF waiting never permits a stale source, even if a slow callback
            # or a clock jump returns after the bounded lookup deadline.
            exact_tf_wait_budget(
                (rospy.Time.now() - color.header.stamp).to_sec(), self.max_age, self.tf_wait)
            self._frame_timing_stage('segmentation')
            exact_transform = transform_matrix(transform)
            # Qualify the fixed station before choosing this frame's support
            # model. Existing metric ambiguity holds keep their original
            # segmentation until a genuinely unique frame recovers them.
            with self._segmentation_timed_lock(), self._segmentation_timing_component('station_qualification'):
                if self._contact_station_frame_retired(contact_station_version):
                    return
                arrival_retry_reference = self._observation_arrival_retry_reference(source, camera)
                contact_spatial_context = (self._contact_spatial_reference(source, camera)
                    if arrival_retry_reference is None else None)
                initial_station_reference = (self._initial_station_support_reference(source)
                    if arrival_retry_reference is None else None)
                self._begin_feedback_recovery_completion(source, initial_station_reference)
                completion = getattr(self, '_arrival_frame_completion', None)
                if (completion is not None and completion['reference'] is not
                        (arrival_retry_reference if completion['marker_kind'] == 'observation_arrival_retry'
                         else initial_station_reference)):
                    # Failed qualification cannot turn a completion allowance
                    # into a fallback to an unqualified old-plane frame.
                    raise ValueError('station_frame_completion_qualification_failed')
                initial_marker = (getattr(self, '_locked_rgb_hold_reference', None) or {}).get(
                    'initial_station_current_support')
                if (arrival_retry_reference is None and initial_marker is not None
                        and initial_station_reference is None
                        and getattr(self, '_metric_ambiguity_pending', None) is None):
                    # Missing qualification cannot switch this established
                    # station back to old-plane geometry, nor claim the
                    # qualified support-gap receipt without real brackets.
                    raise ValueError('initial_station_current_support_invalidated')
                current_support_kwargs = ({'prefer_current_support': True}
                    if (arrival_retry_reference is not None or initial_station_reference is not None)
                    and getattr(self, '_metric_ambiguity_pending', None) is None else {})
            support_gap_reason = None
            try:
                with self._segmentation_timing_component('segment_with_support_fallback'):
                    result = segment_with_support_fallback(
                        depth, intrinsics, exact_transform, self.config,
                        anchor=anchor, last=last, reference_plane_base=plane,
                        **current_support_kwargs)
            except ValueError as exc:
                if anchor is None or str(exc) not in _SUPPORT_MEASUREMENT_GAPS:
                    raise
                support_gap_reason, result = str(exc), None
            self._frame_timing_stage('post_segment')
            if support_gap_reason is None:
                with self.lock:
                    if self._arrival_phase_checkpoint(source, camera):
                        return
            if support_gap_reason is not None:
                self._arrival_frame_completion = None
            with self.lock:
                if generation != self.generation or self.mode != 'unknown':
                    return
                if self._contact_station_frame_retired(contact_station_version):
                    return
                # A task freeze may have arrived while this frame was fitting.
                # Never publish that in-flight pre-motion measurement afterward.
                if self._process_observation_measurement(source):
                    return
                self._check_recovery_arrival_reference(source, camera, arrival_retry_reference)
                self._require_initial_station_support_reference(source, initial_station_reference)
                exact_tf_wait_budget(
                    (rospy.Time.now() - color.header.stamp).to_sec(), self.max_age, self.tf_wait)
                self._frame_timing_stage('tracking')
                self.last_shape = depth.shape
                if (support_gap_reason is not None or (self.tracker.anchor is not None
                        and result.metrics.get('support_current_audit_available') is False)):
                    self._arrival_frame_completion = None
                    self._hold_missing_current_support(source, camera,
                        support_gap_reason if result is None else result.metrics.get('support_current_audit_reason'),
                        result)
                    return
                initial_refined = None
                # Check the previous publishable frame before choose() changes
                # tracker.last. The post-refinement check uses the same marker.
                if (arrival_retry_reference is None
                        and self._observation_arrival_retry_reference(source, camera) is not None):
                    # A qualification arriving during segmentation belongs to
                    # the next frame, never retroactively to this old-plane fit.
                    self.unavailable('observation_station_changed_during_segmentation',
                                     color.header.stamp, source, lose=False)
                    return
                handoff_pair = None
                if self.tracker.anchor is None and getattr(self, '_rgb_acquisition_active', None) is not None:
                    handoff_pair = self._process_rgb_handoff(source, depth, intrinsics,
                        transform_matrix(transform), result, camera)
                    if handoff_pair is None:
                        return
                if self.tracker.anchor is None and handoff_pair is None:
                    # A one-frame depth fragment must not become an immutable
                    # identity. Confirm it before the real tracker is mutated.
                    provisional = TargetTracker(self.config)
                    initial, reason = provisional.choose(result, depth.shape)
                    if initial is None or initial.quality < self.minimum_quality:
                        self.acquisition.reset()
                        if initial is not None or reason == 'no_center_instance':
                            self._initial_visual_insufficiency(
                                reason if initial is None else 'insufficient_geometry_quality',
                                source, depth, intrinsics, transform_matrix(transform), result, camera)
                        else:
                            self.unavailable(reason, color.header.stamp, source, lose=False)
                        return
                    try:
                        self._frame_timing_stage('initial_rgb_refinement')
                        initial_refined = (refine_candidate(
                            self.bridge.imgmsg_to_cv2(color, desired_encoding='bgr8'),
                            depth, intrinsics, transform_matrix(transform), initial, self.config,
                            **_rgb_source_kwargs(camera, depth.shape))
                            if self.rgb_instance_refinement else initial)
                    except ValueError as exc:
                        self.acquisition.reset()
                        self._initial_visual_insufficiency('initial_instance_' + str(exc),
                            source, depth, intrinsics, transform_matrix(transform), result, camera)
                        return
                    self._frame_timing_stage('initial_acquisition')
                    confirmed, detail = self.acquisition.observe(initial, initial_refined, color.header.stamp.to_nsec())
                    if not confirmed:
                        self._initial_visual_insufficiency(detail, source, depth, intrinsics,
                            transform_matrix(transform), result, camera)
                        return
                fragment_pair = None
                if arrival_retry_reference is not None and not self.tracker.lost:
                    matches = _matching_candidates(result.candidates,
                        self.tracker.anchor, self.tracker.last, self.config)
                    recovery = (recover_arrival_fragment_group if not matches else
                                recover_arrival_matching_fragments if len(matches) > 1 else None)
                    marker = self._require_observation_arrival_marker(arrival_retry_reference)
                    if recovery is not None and marker.get('spatial_reference') is not None:
                        self._check_recovery_arrival_reference(source, camera, arrival_retry_reference)
                        rejection = None
                        appearance_histogram, appearance_contour, _ = self._observation_marker_appearance(marker)
                        try:
                            result, fragment_pair = recovery(result,
                                self.bridge.imgmsg_to_cv2(color, desired_encoding='bgr8'), depth,
                                intrinsics, exact_transform, self.config, self.tracker.anchor,
                                self.tracker.last, marker['spatial_reference'],
                                appearance_histogram, appearance_contour,
                                self.minimum_quality,
                                appearance_context=self._observation_marker_appearance_context(source,marker,
                                    lambda: self._check_recovery_arrival_reference(source,camera,arrival_retry_reference)),
                                **_rgb_source_kwargs(camera, depth.shape))
                        except ValueError as exc:
                            if (type(exc) is not ValueError or len(matches) <= 1
                                    or not str(exc).startswith('matching_fragment')):
                                raise
                            # Failure of this optional same-instance proposal
                            # preserves the original multi-candidate hold/reject.
                            rejection = str(exc)
                        timing = getattr(self, '_processing_timing', None)
                        if len(matches) > 1 and timing is not None:
                            timing['matching_fragment_proposal'] = dict(
                                original_metric_matches=len(matches),
                                accepted=fragment_pair is not None, reason=rejection,
                                metric_geometry_authority=False, motion_authorized=False)
                        self._check_recovery_arrival_reference(source, camera, arrival_retry_reference)
                        exact_tf_wait_budget((rospy.Time.now()-color.header.stamp).to_sec(), self.max_age, self.tf_wait)
                        if fragment_pair is not None:
                            initial_refined = fragment_pair[1]
                was_lost = self.tracker.lost
                support_recovery = None
                if (getattr(self, '_locked_source_pending', None) or {}).get('kind') in ('support_measurement_gap', FEEDBACK_GAP):
                    self._frame_timing_stage('support_gap_measured_revalidation')
                    support_recovery = self._support_gap_recovery(source, result, camera,
                        depth, intrinsics, exact_transform,
                        arrival_retry_reference=arrival_retry_reference, pre_refined=fragment_pair)
                    if support_recovery is None:
                        return
                    initial_refined = support_recovery[1]
                ambiguity_recovery = None
                ambiguity_pending = getattr(self, '_metric_ambiguity_pending', None)
                metric_candidates = getattr(result, 'candidates', ())
                ambiguity_entry = (not was_lost and isinstance(self.tracker.anchor, Candidate)
                    and getattr(self, '_locked_rgb_hold_reference', None) is not None
                    and getattr(self, '_locked_source_pending', None) is None
                    and isinstance(metric_candidates, (tuple, list))
                    and all(isinstance(candidate, Candidate) for candidate in metric_candidates)
                    and len(_matching_candidates(metric_candidates,
                        self.tracker.anchor, self.tracker.last, self.config)) > 1)
                if self._arrival_phase_checkpoint(source, camera):
                    return
                if ambiguity_pending is not None or ambiguity_entry:
                    self._arrival_frame_completion = None
                    self._frame_timing_stage('metric_ambiguity_rgb_identity')
                    ambiguity_recovery = self._metric_ambiguity_transition(source, result, camera,
                        depth, intrinsics, transform_matrix(transform),
                        arrival_retry_reference=arrival_retry_reference, pre_refined=fragment_pair)
                    if ambiguity_recovery is None:
                        return
                    initial_refined = ambiguity_recovery[1]
                if (support_recovery is None and ambiguity_recovery is None
                        and self._same_station_zero_candidate_rgb_hold(source, result, camera)):
                    waiting = getattr(self, '_locked_source_pending', None) is not None
                    self.unavailable('source_qualification_pending' if waiting else
                                     'measured_depth_temporarily_unavailable_rgb_identity_held',
                                     color.header.stamp, source, lose=False)
                    return
                pending_recovery = support_recovery[0] if support_recovery is not None else None
                if support_recovery is None and getattr(self, '_locked_source_pending', None) is not None:
                    if (getattr(self, '_locked_source_status', None) or {}).get('state') == 'qualified_new_frame':
                        # Stage the ORIGINAL metric choice while its required
                        # source qualification is still awaiting final commit.
                        pending_recovery = TargetTracker(self.config)
                        pending_recovery.anchor = self.tracker.anchor
                        pending_recovery.last = self.tracker.last
                        pending_recovery.plane_base = self.tracker.plane_base.copy()
                    else:
                        reason = (self._locked_rgb_hold_status or {}).get('reason', 'pending_source_reference_invalidated')
                        self._locked_source_status = dict(self._locked_source_pending,
                            state='rejected', reason=reason, source_qualified=False,
                            identity_verified=False, metric_geometry_authority=False, motion_authorized=False)
                        self.unavailable('source_qualification_rejected: '+reason, color.header.stamp, source)
                        return
                station_tracker = None
                if ((initial_station_reference is not None or fragment_pair is not None
                        or getattr(self, '_arrival_frame_completion', None) is not None
                        or getattr(self, '_station_compute_frame', None) is not None) and pending_recovery is None
                        and ambiguity_recovery is None and handoff_pair is None):
                    station_tracker = TargetTracker(self.config)
                    station_tracker.anchor, station_tracker.last = self.tracker.anchor, self.tracker.last
                    station_tracker.plane_base = self.tracker.plane_base.copy()
                if handoff_pair is not None:
                    with self.input_lock:
                        if (self._rgb_acquisition_active is None or self.tracker.lost
                                or self._rgb_handoff_commit_version != self._rgb_acquisition_version
                                or rospy.Time.now().to_nsec() > self._rgb_acquisition_active['deadline_ns']
                                or time.monotonic() > self._rgb_acquisition_active['deadline_monotonic']):
                            self.unavailable('handoff_revoked_before_commit', color.header.stamp, source, lose=False)
                            return
                        candidate, initial_refined = handoff_pair
                        self.tracker.anchor, self.tracker.last = candidate, candidate
                        self.tracker.plane_base = result.plane_base.copy()
                        self._rgb_acquisition_active = None
                        self._rgb_acquisition_version += 1
                    reason = 'rgb_associated_measured_acquisition'
                elif ambiguity_recovery is not None:
                    candidate, reason = ambiguity_recovery[0].last, 'tracked'
                elif support_recovery is not None:
                    candidate, reason = support_recovery[0].last, 'tracked'
                else:
                    choice_tracker = (pending_recovery if pending_recovery is not None else
                                      station_tracker if station_tracker is not None else self.tracker)
                    candidate, reason = choice_tracker.choose(result, depth.shape)
                if candidate is not None and self.anchor_source_stamp_ns == 0:
                    self.anchor_source_stamp_ns = color.header.stamp.to_nsec()
                if candidate is None:
                    evidence = (candidate_loss_observation(result, self.tracker.anchor,
                        self.tracker.last, self.config, was_lost) if self.tracker.anchor is not None else None)
                    self.unavailable(reason, color.header.stamp, source, candidate_evidence=evidence)
                    return
                if candidate.quality < self.minimum_quality:
                    self.unavailable('insufficient_geometry_quality', color.header.stamp, source)
                    return
                self._clear_rgb_interest('measured_target_available')
                if initial_refined is not None:
                    candidate = initial_refined
                elif self.rgb_instance_refinement:
                    try:
                        self._frame_timing_stage('rgb_refinement')
                        with self._rgb_timing_component('refine'):
                            candidate = refine_candidate(
                                self.bridge.imgmsg_to_cv2(color, desired_encoding='bgr8'),
                                depth, intrinsics, transform_matrix(transform), candidate, self.config,
                                **self._arrival_rgb_spatial_kwargs(arrival_retry_reference),
                                **({'contact_spatial_reference': contact_spatial_context['spatial_reference']}
                                   if contact_spatial_context is not None else {}),
                                **_rgb_source_kwargs(camera, depth.shape))
                    except ValueError as exc:
                        if isinstance(exc, ObservationArrivalStationChanged):
                            raise
                        # A failed RGB-D ownership estimate must not publish
                        # the contaminated depth proposal as an accepted mask.
                        # Keep identity while withholding this frame; fresh
                        # measurements must still pass the original fusion gates.
                        self.unavailable('rgb_depth_refinement_rejected: ' + str(exc),
                                         color.header.stamp, source, lose=False)
                        return
                candidate = self._recover_station_current_fragment_owner(source, result,
                    candidate, camera, depth, intrinsics, exact_transform)
                self._station_compute_recovery_evidence(source, result, candidate, camera)
                staged_tracker = (ambiguity_recovery[0] if ambiguity_recovery is not None else
                                  pending_recovery if pending_recovery is not None else station_tracker)
                with self._rgb_timing_component('hold.total'):
                    hold_reference = (self._prepare_locked_rgb_hold_reference(source, candidate, camera,
                                          source_transform=exact_transform)
                        if staged_tracker is None else
                        self._prepare_locked_rgb_hold_reference(source, candidate, camera,
                            tracker=staged_tracker, source_transform=exact_transform))
                self._frame_timing_stage('post_refine')
                exact_tf_wait_budget(
                    (rospy.Time.now() - color.header.stamp).to_sec(), self.max_age, self.tf_wait)
                if pending_recovery is not None:
                    self._revalidate_pending_source_commit(source, hold_reference)
                if ambiguity_recovery is not None:
                    reference = ambiguity_recovery[2]
                    if (hold_reference is None or hold_reference['source_stamp_ns'] != color.header.stamp.to_nsec()
                            or hold_reference['source_id'] != reference['source_id']):
                        raise ValueError('metric_ambiguity_new_reference_unavailable')
                    grouped = self._qualified_fragment_pair(source, result, camera,
                        arrival_retry_reference, fragment_pair)
                    self._qualify_metric_ambiguity_frame(source, result, reference, current_fragment_group=grouped)
                if arrival_retry_reference is not None:
                    if (hold_reference is None or self._observation_arrival_retry_reference(
                            source, camera, expected=arrival_retry_reference, check_last=False) is None):
                        raise ValueError('observation_arrival_station_invalidated_before_publish')
                if initial_station_reference is not None:
                    self._require_initial_station_support_reference(source, initial_station_reference)
                    if (hold_reference is None
                            or hold_reference['source_id'] != initial_station_reference['source_id']):
                        raise ValueError('initial_station_new_reference_unavailable')
                with self.input_lock:
                    if self._arrival_phase_checkpoint(source, camera):
                        return
                    if contact_spatial_context is not None:
                        self._contact_spatial_reference(source, camera, expected=contact_spatial_context)
                    self._station_compute_commit_check(source, camera, result, candidate)
                    completion = getattr(self, '_arrival_frame_completion', None)
                    if completion is not None:
                        if completion['marker_kind'] == 'observation_arrival_retry':
                            if self._observation_arrival_retry_reference(source, camera,
                                    expected=arrival_retry_reference) is not arrival_retry_reference:
                                raise ValueError('observation_arrival_station_invalidated_at_commit')
                        elif (initial_station_reference is None or
                                self._initial_station_support_reference(source,
                                    expected=initial_station_reference) is not initial_station_reference):
                            raise ValueError('initial_station_current_support_invalidated')
                        require_fresh_source(color.header.stamp.to_nsec(), rospy.Time.now().to_nsec(), min(self.max_age, .8))
                    self._require_initial_station_support_reference(source, initial_station_reference)
                    if initial_station_reference is not None:
                        hold_reference['initial_station_current_support'] = (
                            initial_station_reference['initial_station_current_support'])
                    elif (reason == 'acquired' and hold_reference is not None
                            and self._initial_station_selection_matches()
                            and rospy.get_param_cached('/gui/joint_direct_mode', False) is not True):
                        # Only the original three-frame acquisition creates
                        # this station. Neither tracking nor a return re-arms it.
                        hold_reference['initial_station_current_support'] = dict(
                            source_stamp_ns=hold_reference['source_stamp_ns'],
                            q_source=hold_reference['q_source'], source_id=hold_reference['source_id'],
                            epoch_ns=hold_reference['epoch_ns'],
                            anchor_fingerprint=hold_reference['anchor_fingerprint'],
                            reference_plane=hold_reference['reference_plane'], invalidated_reason='',
                            identity_template=dict(
                                provenance=self._initial_station_template_provenance(hold_reference),
                                bgr=hold_reference['bgr'], mask=hold_reference['mask']))
                    if arrival_retry_reference is not None:
                        marker = arrival_retry_reference.get('observation_arrival_retry')
                        if (self._locked_rgb_hold_reference is not arrival_retry_reference
                                or marker is None or marker.get('invalidated_reason')
                                or arrival_retry_reference['pose_invalidated']
                                or self._rgb_interest_epoch != marker['epoch_ns']
                                or rospy.get_param_cached('/gui/joint_direct_mode', False) is True
                                or time.monotonic() >= marker['session_deadline_monotonic']
                                or rospy.Time.now().to_nsec() >= marker['session_deadline_ns']):
                            raise ValueError('observation_arrival_station_invalidated_at_commit')
                        hold_reference['observation_arrival_retry'] = marker
                    if ambiguity_recovery is not None:
                        self._check_metric_ambiguity_reference(ambiguity_recovery[2])
                        require_fresh_source(color.header.stamp.to_nsec(), rospy.Time.now().to_nsec(), min(self.max_age, .8))
                        self.tracker.last = ambiguity_recovery[0].last
                        self._metric_ambiguity_pending = None
                        self._metric_ambiguity_status['state'] = 'recovered_unique_measured_frame'
                        self._metric_ambiguity_status['measurement_republished'] = True
                    if pending_recovery is not None:
                        # Callback invalidation and measured publication share
                        # this commit lock. Refinement never mutated live last.
                        self._check_pending_source_reference()
                        self._require_support_reference_lifetime(self._locked_rgb_hold_reference,
                                                                 color.header.stamp.to_nsec())
                        require_fresh_source(color.header.stamp.to_nsec(), rospy.Time.now().to_nsec(), min(self.max_age, .8))
                        self.tracker.last = pending_recovery.last
                        self._locked_source_pending = None
                        if support_recovery is not None:
                            self._locked_source_status.update(state='recovered_unique_measured_frame',
                                measurement_republished=True)
                            if self._locked_source_status.get('kind') == FEEDBACK_GAP:
                                self._locked_source_status['requalified_accepted_feedback'] = deepcopy([
                                    hold_reference['accepted_before'], hold_reference['accepted_after']])
                    if station_tracker is not None:
                        if fragment_pair is not None:
                            require_fresh_source(color.header.stamp.to_nsec(),
                                rospy.Time.now().to_nsec(), min(self.max_age, .8))
                        self.tracker.last = station_tracker.last
                    self._frame_timing_stage('publish')
                    # Frame identity remains that of the RGB-D source, including mask.
                    obj = ObjectPose()
                    obj.header.stamp, obj.header.frame_id = color.header.stamp, self.base_frame
                    obj.header.seq = color.header.seq
                    obj.detected, obj.label, obj.confidence = True, 'unknown_small', candidate.quality
                    obj.u, obj.v = [int(round(value)) for value in candidate.center_uv]
                    obj.bbox_x, obj.bbox_y, obj.bbox_width, obj.bbox_height = candidate.bbox
                    obj.depth_m = candidate.depth_m
                    for name, point, frame_id in (
                            ('pose_camera', candidate.position_camera, self.camera_frame),
                            ('pose_base', candidate.position_base, self.base_frame)):
                        pose = PoseStamped()
                        pose.header.stamp, pose.header.frame_id = color.header.stamp, frame_id
                        pose.header.seq = color.header.seq
                        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = point
                        pose.pose.orientation.w = 1.0
                        setattr(obj, name, pose)
                        self.pubs['object_'+name].publish(pose)
                    mask = self.bridge.cv2_to_imgmsg(candidate.mask, encoding='mono8')
                    mask.header = color.header
                    self.pubs['object_mask'].publish(mask)
                    self.pubs['object'].publish(obj)
                    self.pubs['object_detected'].publish(Bool(data=True))
                    self.pubs['raw_object_detected'].publish(Bool(data=True))
                    self.last_good_source_stamp_ns = color.header.stamp.to_nsec()
                    with self.input_lock:
                        if hold_reference is not None:
                            seen = [values for ns, values in self._rgb_interest_feedback
                                    if ns >= hold_reference['source_stamp_ns']]
                            if (self._rgb_interest_epoch != hold_reference['epoch_ns'] or not seen
                                    or any(np.max(np.abs(values-hold_reference['q_source'])) > .004
                                           for values in seen)):
                                hold_reference = None
                        self._locked_rgb_hold_reference = hold_reference
                    self.status('ready', reason, source, candidate, time.monotonic()-started)
                    self._supplement_admission_ready(source)
        except Exception as exc:
            timing = self._processing_timing
            if timing is not None:
                timing['failed_stage'] = timing['stage']
                timing['failure_ros_ns'] = rospy.Time.now().to_nsec()
                timing['failure_source_age_sec'] = (timing['failure_ros_ns']-timing['source_stamp_ns'])*1e-9
                self._frame_timing_stage('failure_publish')
            with self.lock:
                if generation == self.generation and self.mode == 'unknown':
                    if self._contact_station_frame_retired(contact_station_version):
                        return
                    if self._station_requalification_wait(source, exc):
                        return
                    if self._station_compute_retired_frame(source, exc):
                        return
                    if isinstance(exc, StationFeedbackBracketGap):
                        try:
                            self._station_compute_publish_feedback_gap(source, exc)
                            return
                        except (ValueError, KeyError, TypeError, AttributeError) as rejection:
                            exc = rejection
                    if self._station_compute_timeout(source, exc):
                        return
                    if self._process_observation_measurement(source):
                        return
                    if isinstance(exc, InitialFeedbackBracketGap):
                        try:
                            self._publish_initial_feedback_gap(source, exc)
                            return
                        except (ValueError, KeyError, TypeError, AttributeError) as rejection:
                            exc = rejection
                    pending = getattr(self, '_locked_source_pending', None)
                    if pending is not None:
                        self._locked_source_status = dict(pending, state='rejected', reason=str(exc),
                            source_qualified=False, identity_verified=False,
                            metric_geometry_authority=False, motion_authorized=False)
                    if getattr(self, '_metric_ambiguity_pending', None) is not None:
                        self._metric_ambiguity_status = dict(state='rejected', reason=str(exc),
                            metric_geometry_authority=False, motion_authorized=False)
                    if self.tracker.anchor is None:
                        self.acquisition.reset()
                    reason = ('missing_exact_tf: ' if isinstance(exc, (
                        tf2_ros.LookupException, tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException)) else 'perception_failure: ') + str(exc)
                    self.unavailable(reason, color.header.stamp, source)
                    rospy.logwarn_throttle(3.0, 'Unknown perception unavailable: %s', reason)
        finally:
            with self.lock:
                self._supplement_admission_cancel('SUPPLEMENT_MEASUREMENT_ADMISSION_FRAME_NOT_READY')
            self._arrival_frame_completion = None
            self._station_compute_frame = None
            self._arrival_phase_frame = None
            self._processing_timing_thread_ident = None


if __name__ == '__main__':
    # These small image operations gain no end-to-end speed from the default
    # CPU-sized pool. Bound only this process before any callbacks can start.
    cv2.setNumThreads(1)
    rospy.init_node('unknown_tabletop_perception')
    rospy.loginfo('Unknown perception OpenCV worker threads: %s', cv2.getNumThreads())
    UnknownTabletopPerception()
    rospy.spin()
