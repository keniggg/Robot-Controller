#!/usr/bin/env python3
"""Independent exact-RGB-D tabletop perception. This node has no arm controls."""
import json
import math
import threading
import time
import hashlib
from copy import deepcopy
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
from alicia_flexible_grasp_supervisor.msg import ObjectPose
from alicia_grasp_modes.tabletop import (
    Candidate, Config, TargetTracker, segment_with_support_fallback, _validate_support_plane_update)
from alicia_grasp_modes.rgb_instance import refine_candidate
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
from alicia_flexible_grasp.robot.observation_view_limits import (
    DEFAULT_OBSERVATION_VIEW_PARAMETERS, effective_observation_view_limits)



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


class UnknownTabletopPerception:
    def __init__(self):
        cfg = rospy.get_param('/unknown_perception', {})
        camera = rospy.get_param('/camera', {})
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
            with self.input_lock:
                self.mode, self.generation, self.selection_ns = mode, generation, stamp
                self.latest, self.last_consumed_ns = None, 0
                self._latest_frame_timing = None
                self._processing_timing = None
                self.last_receive_wall = time.monotonic()
            self.anchor_source_stamp_ns = 0
            self.first_loss = None
            self.last_good_source_stamp_ns = 0
            self._locked_rgb_hold_reference = None
            self._locked_rgb_hold_status = None
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
                if active is None or event.get('request_id') != active['request_id']:
                    return
                if event.get('phase') in ('revoked', 'planning_rejected', 'cancelled',
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
        if (active is not None and active['phase'] in ('frozen', 'reserved')
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
            hold = getattr(self, '_locked_rgb_hold_reference', None)
            if (hold is not None and stamp_ns >= hold['source_stamp_ns']
                    and np.max(np.abs(q-hold['q_source'])) > .004):
                # A later return to the old pose cannot hide measured motion.
                hold['pose_invalidated'] = True
            anchor = self._rgb_interest_pose_anchor
            if (anchor is not None and np.max(np.abs(q - anchor)) >
                    self._rgb_interest_tracker.max_joint_delta_rad):
                self._invalidate_rgb_interest_input('measured_arm_pose_changed')

    def _rgb_interest_source_context(self, stamp):
        # Cached ROS parameters still receive parameter updates; hash the model
        # only when its text changes. No URDF parse or disk I/O occurs here.
        get = rospy.get_param_cached
        model = get('/robot_description', '')
        if not isinstance(model, str) or not model.strip():
            raise ValueError('observation_robot_model_unavailable')
        if self._rgb_interest_model_cache is None or self._rgb_interest_model_cache[0] != model:
            self._rgb_interest_model_cache = (model, hashlib.sha256(model.encode()).hexdigest())
        with self.input_lock:
            epoch = self._rgb_interest_epoch
        tool_camera = transform_matrix(self.buffer.lookup_transform(
            'tool0', self.camera_frame, stamp, rospy.Duration(.1)))
        current_tool_camera = transform_matrix(self.buffer.lookup_transform(
            'tool0', self.camera_frame, rospy.Time(0), rospy.Duration(.1)))
        if not np.array_equal(tool_camera, current_tool_camera):
            raise ValueError('observation_camera_mount_changed')
        # rospy's cached parameter values can be mutable shared dictionaries.
        # Freeze every nested value now: an in-place parameter update must not
        # rewrite the context attributed to an already acquired image.
        return deepcopy(dict(epoch_ns=epoch, selection=parse_selection(get('/grasp_mode/selection')),
            camera=camera_geometry_snapshot(get('/camera')),
            perception=get('/unknown_perception', {}), handeye=get('/handeye', {}),
            model_sha256=self._rgb_interest_model_cache[1], tool_camera=tool_camera.tolist(),
            observation_view_parameters={key: get('/grasp_6d/remote/' + key, value)
                for key, value in DEFAULT_OBSERVATION_VIEW_PARAMETERS.items()}))

    def _rgb_bracketed_feedback(self, stamp_ns, epoch):
        """Await the trailing real sample without renewing the image lifetime.

        RGB-D and accepted joints arrive on different callbacks. Retain the
        original 200 ms interpolation limit and source age while briefly waiting
        for feedback newer than the image. No pose is extrapolated.
        """
        deadline = time.monotonic()+.12
        for attempt in range(7):
            require_fresh_source(stamp_ns, rospy.Time.now().to_nsec(), min(self.max_age, .8))
            with self.input_lock:
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
                    raise
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
                                                       depth_valid=valid)
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
                    stamp_ns, stamp_ns, rospy.Time.now().to_nsec(), tuple(feedback))
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
            planning_refresh = active['phase'] in ('frozen', 'reserved')
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
                rospy.Time.now().to_nsec(), feedback)
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
                depth_valid=np.isfinite(depth)&(depth >= self.config.depth_min_m)&(depth <= self.config.depth_max_m))
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
                    refined = (refine_candidate(rgb, depth, intrinsics, transform, raw, self.config)
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
            stage_wall_sec={}, stage_thread_cpu_sec={}, stage_end_elapsed_sec={})
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

    def _source_timing_status(self, source):
        timing = getattr(self, '_processing_timing', None)
        if timing is None or timing['source_stamp_ns'] != source[0].header.stamp.to_nsec():
            return None
        result = dict(timing)
        for key in ('stage_wall_sec', 'stage_thread_cpu_sec', 'stage_end_elapsed_sec'):
            result[key] = dict(timing[key])
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
        if self.first_loss is not None:
            payload['first_loss'] = self.first_loss
        if self.tracker.anchor is not None:
            payload.update(anchor_source_stamp_ns=self.anchor_source_stamp_ns,
                           anchor_position_base=self.tracker.anchor.position_base.tolist(),
                           anchor_extent_m=self.tracker.anchor.extent_m.tolist(),
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
        self.pubs['detector_status'].publish(String(data=json.dumps(payload, sort_keys=True)))

    def _initial_visual_insufficiency(self, reason, source, depth, intrinsics,
                                     transform, segmentation, camera):
        """Partial depth evidence may seek observation without becoming a target."""
        self._frame_timing_stage('rgb_observation_interest')
        self._update_initial_rgb_interest(source, depth, intrinsics, transform, segmentation, camera)
        self.unavailable(reason, source[0].header.stamp, source, lose=False,
                         preserve_rgb_interest=True)

    def _prepare_locked_rgb_hold_reference(self, source, candidate, camera):
        """Best-effort cache from a genuinely publishable metric RGB-D frame.

        This independent identity cache owns no geometry, plan or observation
        request. Failure to prepare it leaves the existing metric path intact.
        """
        if not hasattr(self, '_rgb_interest_feedback'):
            return None
        try:
            stamp_ns = source[0].header.stamp.to_nsec()
            context = self._rgb_interest_source_context(source[0].header.stamp)
            epoch = context['epoch_ns']
            if (context['perception'] != self._rgb_interest_config
                    or context['selection']['generation'] != self.generation
                    or context['selection']['stamp_ns'] != self.selection_ns):
                return None
            records, (q, before, after) = self._rgb_bracketed_feedback(stamp_ns, epoch)
            rgb = self.bridge.imgmsg_to_cv2(source[0], desired_encoding='bgr8')
            raw_depth = self.bridge.imgmsg_to_cv2(source[1], desired_encoding='passthrough')
            if (rgb.shape != (camera['height'], camera['width'], 3)
                    or raw_depth.shape != rgb.shape[:2]
                    or source[1].header.stamp.to_nsec() != stamp_ns):
                return None
            mask = _mask(candidate.mask, rgb.shape[:2])
            histogram, contour = _appearance(rgb, mask)
            def immutable(array):
                value = np.ascontiguousarray(array)
                return np.frombuffer(value.tobytes(), dtype=value.dtype).reshape(value.shape)
            reference = dict(source_stamp_ns=stamp_ns, source_id=digest(context),
                epoch_ns=epoch, generation=self.generation, selection_ns=self.selection_ns,
                config_sha256=digest(asdict(self.config)), image_shape=rgb.shape[:2],
                source_rgb_sha256=hashlib.sha256(np.ascontiguousarray(rgb).tobytes()).hexdigest(),
                source_depth_sha256=hashlib.sha256(np.ascontiguousarray(raw_depth).tobytes()).hexdigest(),
                refined_mask_sha256=hashlib.sha256(mask.tobytes()).hexdigest(),
                bgr=immutable(rgb), mask=immutable(mask), histogram=immutable(histogram), contour=immutable(contour),
                q_source=immutable(q), accepted_before=before, accepted_after=after,
                anchor_fingerprint=_candidate_fingerprint(self.tracker.anchor),
                last_fingerprint=_candidate_fingerprint(self.tracker.last),
                reference_plane=immutable(self.tracker.plane_base),
                reference_footprint=immutable(self.tracker.anchor.support_footprint_base),
                pose_invalidated=False)
            require_fresh_source(stamp_ns, rospy.Time.now().to_nsec(), min(self.max_age, .8))
            if digest(self._rgb_interest_source_context(source[0].header.stamp)) != reference['source_id']:
                return None
            with self.input_lock:
                if epoch != self._rgb_interest_epoch:
                    return None
                relevant = [values for ns, values in self._rgb_interest_feedback if ns >= stamp_ns]
                if not relevant or any(np.max(np.abs(values-q)) > .004 for values in relevant):
                    return None
            return reference
        except (ValueError, KeyError, TypeError, AttributeError, cv2.error,
                tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None

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

    def _same_station_zero_candidate_rgb_hold(self, source, result, camera):
        """Preserve identity for a brief gap or bounded contained fragments.

        The original last-good stamp fixes the entire window. RGB-only frames
        never refresh it, change the anchor or recover an already lost tracker.
        """
        reference = getattr(self, '_locked_rgb_hold_reference', None)
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
            seen_pixels = np.zeros(reference['image_shape'], dtype=bool) if candidates else None
            for candidate in candidates:
                fragment = self._locked_rgb_shrinking_fragment(candidate, reference, source, camera)
                if fragment is None:
                    # A potentially usable metric candidate keeps the original
                    # tracker/refinement path, even among shrinking fragments.
                    self._locked_rgb_hold_status = None
                    return False
                pixels = fragment['mask'] != 0
                if np.any(seen_pixels & pixels):
                    raise ValueError('locked_rgb_hold_fragment_masks_overlap')
                # This temporary occupancy mask checks disjointness only. It
                # never becomes an object mask or candidate geometry.
                seen_pixels |= pixels
                fragments.append(fragment)
            stamp_ns = source[0].header.stamp.to_nsec()
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
                    or _candidate_fingerprint(self.tracker.last) != reference['last_fingerprint']):
                raise ValueError('locked_rgb_hold_reference_changed')
            context = self._rgb_interest_source_context(source[0].header.stamp)
            if digest(context) != reference['source_id']:
                raise ValueError('locked_rgb_hold_source_context_changed')
            records, (q, before, after) = self._rgb_bracketed_feedback(stamp_ns, reference['epoch_ns'])
            if any(np.max(np.abs(np.asarray(values)-reference['q_source'])) > .004
                   for values in (q, before[1], after[1])):
                raise ValueError('locked_rgb_hold_pose_changed')
            metrics = result.metrics
            if (metrics.get('support_current_audit_available') is not True
                    or metrics.get('support_source') != 'current_ring_fit'):
                raise ValueError('locked_rgb_hold_requires_current_support_audit')
            _, support_audit = _validate_support_plane_update(reference['reference_plane'],
                result.plane_base, self.tracker.anchor.position_base, self.config,
                reference['reference_footprint'])
            rgb = self.bridge.imgmsg_to_cv2(source[0], desired_encoding='bgr8')
            if rgb.shape != reference['image_shape']+(3,):
                raise ValueError('locked_rgb_hold_image_shape_changed')
            x, y, width, height = cv2.boundingRect(reference['mask'])
            policy = replace(self._rgb_interest_policy,
                center_xy=((x+.5*width)/rgb.shape[1], (y+.5*height)/rgb.shape[0]))
            proposals = propose_rgb_observation_regions(rgb, policy=policy)
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
            if fragments:
                current_mask = _mask(proposals.candidates[matches[0][1]].mask, rgb.shape[:2])
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
                if (self._locked_rgb_hold_reference is not reference or reference['pose_invalidated']
                        or self._rgb_interest_epoch != reference['epoch_ns'] or newest is None
                        or np.max(np.abs(newest[1]-reference['q_source'])) > .004):
                    raise ValueError('locked_rgb_hold_changed_during_processing')
                require_fresh_source(int(newest[0]), now_ns, min(self.max_age, .8))
            fragment_evidence = [dict(fragment['evidence']) for fragment in fragments]
            for evidence in fragment_evidence:
                evidence['kind'] = ('boundary_consistent_shrinking_fragment' if evidence['boundary_tolerance_used']
                    else 'unique_contained_shrinking_fragment' if len(fragments) == 1
                    else 'contained_shrinking_fragment')
            self._locked_rgb_hold_status.update(state='holding_identity_only',
                source_stamp_ns=stamp_ns, mask_iou=matches[0][2],
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

    def unavailable(self, reason, stamp=None, source=None, lose=True, candidate_evidence=None,
                    preserve_rgb_interest=False):
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
            self._locked_rgb_hold_reference = None
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
                self._locked_rgb_hold_status = None
                generation = self.generation
                receive_wall = self.last_receive_wall
                if source is not None:
                    self.last_consumed_ns = source[0].header.stamp.to_nsec()
            if source is None:
                if time.monotonic() - receive_wall > self.max_age:
                    self.unavailable('exact_rgbd_stream_stale')
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
            if (color.header.frame_id != self.camera_frame
                    or depth_message.header.frame_id != self.camera_frame):
                raise ValueError('aligned_rgbd_frame_mismatch')
            if (color.width, color.height) != (depth_message.width, depth_message.height):
                raise ValueError('unaligned_rgbd_dimensions')
            self._frame_timing_stage('decode')
            camera = rospy.get_param('/camera', {})
            if not bool(camera.get('align_depth_to_color', False)):
                raise ValueError('aligned_depth_required')
            intrinsics = [float(camera[name]) for name in ('fx', 'fy', 'cx', 'cy')]
            array = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding='passthrough')
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
            # TF waiting never permits a stale source, even if a slow callback
            # or a clock jump returns after the bounded lookup deadline.
            exact_tf_wait_budget(
                (rospy.Time.now() - color.header.stamp).to_sec(), self.max_age, self.tf_wait)
            self._frame_timing_stage('segmentation')
            result = segment_with_support_fallback(
                depth, intrinsics, transform_matrix(transform), self.config,
                anchor=anchor, last=last, reference_plane_base=plane)
            self._frame_timing_stage('post_segment')
            with self.lock:
                if generation != self.generation or self.mode != 'unknown':
                    return
                exact_tf_wait_budget(
                    (rospy.Time.now() - color.header.stamp).to_sec(), self.max_age, self.tf_wait)
                self._frame_timing_stage('tracking')
                self.last_shape = depth.shape
                initial_refined = None
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
                            depth, intrinsics, transform_matrix(transform), initial, self.config)
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
                was_lost = self.tracker.lost
                if self._same_station_zero_candidate_rgb_hold(source, result, camera):
                    self.unavailable('measured_depth_temporarily_unavailable_rgb_identity_held',
                                     color.header.stamp, source, lose=False)
                    return
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
                else:
                    candidate, reason = self.tracker.choose(result, depth.shape)
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
                        candidate = refine_candidate(
                            self.bridge.imgmsg_to_cv2(color, desired_encoding='bgr8'),
                            depth, intrinsics, transform_matrix(transform), candidate, self.config)
                    except ValueError as exc:
                        # A failed RGB-D ownership estimate must not publish
                        # the contaminated depth proposal as an accepted mask.
                        # Keep identity while withholding this frame; fresh
                        # measurements must still pass the original fusion gates.
                        self.unavailable('rgb_depth_refinement_rejected: ' + str(exc),
                                         color.header.stamp, source, lose=False)
                        return
                hold_reference = self._prepare_locked_rgb_hold_reference(source, candidate, camera)
                self._frame_timing_stage('post_refine')
                exact_tf_wait_budget(
                    (rospy.Time.now() - color.header.stamp).to_sec(), self.max_age, self.tf_wait)
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
        except Exception as exc:
            timing = self._processing_timing
            if timing is not None:
                timing['failed_stage'] = timing['stage']
                timing['failure_ros_ns'] = rospy.Time.now().to_nsec()
                timing['failure_source_age_sec'] = (timing['failure_ros_ns']-timing['source_stamp_ns'])*1e-9
                self._frame_timing_stage('failure_publish')
            with self.lock:
                if generation == self.generation and self.mode == 'unknown':
                    if self.tracker.anchor is None:
                        self.acquisition.reset()
                    reason = ('missing_exact_tf: ' if isinstance(exc, (
                        tf2_ros.LookupException, tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException)) else 'perception_failure: ') + str(exc)
                    self.unavailable(reason, color.header.stamp, source)
                    rospy.logwarn_throttle(3.0, 'Unknown perception unavailable: %s', reason)


if __name__ == '__main__':
    rospy.init_node('unknown_tabletop_perception')
    UnknownTabletopPerception()
    rospy.spin()
