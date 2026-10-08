"""Independent, bounded non-contact RGB acquisition owner.

The detector's RGB interest is never a Grasp6DPlan or contact geometry. This
adapter pairs it with actual source images, replans through MoveIt, certifies
the final controller curve against an independent RGB non-contact scene, and
uses the ordinary proof-bound action executor. A successful RPC means only
an observation move and measured settling. New 3-D target acceptance and RGB
association remain an explicit subsequent handoff.
"""
from collections import OrderedDict, deque
from copy import deepcopy
from dataclasses import asdict, fields
import hashlib
import itertools
import json
import math
import threading
import time

import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

from alicia_grasp_modes.selection import parse_selection
from alicia_grasp_modes.rgb_observation_source import (
    digest, camera_geometry_snapshot, bracket_feedback, require_fresh_source,
    measured_support_coverage,
)
from alicia_grasp_modes.rgb_observation_region import CenterSearchPolicy, propose_rgb_observation_regions
from alicia_grasp_modes.rgb_observation_set import (
    select_rgb_interest_set, observation_set_digest, observation_set_metadata,
)
from alicia_grasp_modes.rgb_observation_volume import (
    RGBObservationVolume, SupportPlaneProvenance, project_rgb_observation_vertices)
from alicia_grasp_modes.rgb_observation_projection import projected_rgb_search_policy
from alicia_grasp_modes.tabletop import Config
from alicia_flexible_grasp_supervisor.srv import SetTargetPoseResponse
from alicia_flexible_grasp.grasp.gripper_geometry import (
    GripperGeometry, gripper_contract_mismatch_reason, ANALYTICAL_GRIPPER_MODEL_NAME,
)
from .observation_path_guard import (
    ObservationPathError, SerialUrdfFk, wire_digest, segment_coefficients, _polynomial_bounds,
)
from .observation_view_limits import effective_observation_view_limits
from .observation_view_guard import validate_observation_target_trajectory
from .observation_tracking_contract import qualify_contract_path
from .stationary_following import ARM_NAMES
from .actuation_bootstrap import PENDING_BOOTSTRAP_STATES
from alicia_grasp_modes.rgb_prefix_preflight import validate_prefix_preflight_receipt

REQUEST_PARAM = '/rgb_acquisition/request'
LEDGER_PARAM = '/rgb_acquisition/ledger'
STATUS_PARAM = '/rgb_acquisition/status'
SERVICE = '/supervisor/plan_and_execute_rgb_acquisition_pose_strict'
VIEW_DEFAULTS = dict(camera_visibility_margin_px=36., camera_visibility_min_depth_m=.070,
                     camera_visibility_max_depth_m=.500)


def _error(message):
    raise ObservationPathError('RGB_ACQUISITION_' + message)


def _matrix(stamped):
    value = stamped.transform
    q = np.array([getattr(value.rotation, k) for k in 'xyzw'])
    if not np.all(np.isfinite(q)) or abs(np.linalg.norm(q)-1.) > 1e-6:
        _error('TF_ROTATION_INVALID')
    # Match the producer's canonical quaternion conversion byte-for-byte;
    # tf.quaternion_matrix uses a different arithmetic order (~1e-16 drift).
    x, y, z, w = q / np.sqrt(float(q @ q))
    result = np.eye(4)
    result[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                      [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                      [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    result[:3, 3] = [getattr(value.translation, k) for k in 'xyz']
    if not np.all(np.isfinite(result)):
        _error('TF_TRANSLATION_INVALID')
    return result


def ros_parameter_value(value):
    if type(value) is int and not -(2**31) <= value < 2**31:
        return str(value)
    if isinstance(value, dict):
        return {k:ros_parameter_value(v) for k,v in value.items()}
    if isinstance(value, (list,tuple)):
        return [ros_parameter_value(v) for v in value]
    return value


def verify_rgb_interest_members(interest, proposals, policy, *, associated=False):
    """Bind every declared hypothesis to its own real mask; never union masks."""
    if 'budget' in proposals.reason:
        _error('RGB_INTEREST_SET_SOURCE_BUDGET_EXHAUSTED')
    metadata = interest.get('rgb_interest_members')
    if not isinstance(metadata, (list, tuple)) or not 1 <= len(metadata) <= 4:
        _error('EXPLICIT_RGB_INTEREST_SET_REQUIRED')
    hashes = [row.get('mask_sha256') if isinstance(row, dict) else None for row in metadata]
    if any(type(value) is not str or len(value) != 64 for value in hashes) or len(set(hashes)) != len(hashes):
        _error('RGB_INTEREST_SET_MEMBERS_INVALID')
    by_hash = {hashlib.sha256(region.mask.tobytes()).hexdigest(): region for region in proposals.candidates}
    if any(value not in by_hash for value in hashes):
        _error('RGB_INTEREST_SET_PIXEL_BINDING_FAILED')
    regions = tuple(by_hash[value] for value in hashes)
    if (digest(observation_set_metadata(regions)) != digest(metadata)
            or observation_set_digest(regions) != interest.get('observation_set_sha256')
            or interest.get('region_mask_sha256') != hashes[0]):
        _error('RGB_INTEREST_SET_DIGEST_CHANGED')
    if not associated:
        # A stable set may retain an earlier competitor that remains an actual
        # current-frame proposal. It must never omit a current competitor.
        required = select_rgb_interest_set(proposals, policy)
        if not required or any(hashlib.sha256(item.mask.tobytes()).hexdigest() not in hashes
                               for item in required):
            _error('RGB_INTEREST_SET_OMITS_CURRENT_HYPOTHESIS')
    return regions


def decode_rgb_interest_volumes(interest, regions, *, ns, K, T, cfg, limits):
    """Reconstruct all bounded member volumes plus a visibility-only envelope."""
    raw_members = interest.get('interest_volumes')
    if not isinstance(raw_members, (list, tuple)) or len(raw_members) != len(regions):
        _error('RGB_INTEREST_MEMBER_VOLUMES_REQUIRED')
    boxes = np.asarray([region.bbox_xywh for region in regions])
    corner = boxes[:, :2].min(axis=0)
    size = (boxes[:, :2]+boxes[:, 2:]).max(axis=0)-corner
    envelope_box = tuple(np.r_[corner, size])
    volumes = []
    for raw, box in zip(list(raw_members)+[interest['interest_volume']],
                        [tuple(region.bbox_xywh) for region in regions]+[envelope_box]):
        value = deepcopy(raw)
        value['support_provenance'] = SupportPlaneProvenance(**value['support_provenance'])
        volume = RGBObservationVolume(**{f.name:value[f.name] for f in fields(RGBObservationVolume) if f.init})
        if (volume.source_stamp_ns != ns or tuple(volume.bbox_xywh) != box
                or not np.array_equal(volume.camera_matrix, K) or not np.array_equal(volume.T_base_camera, T)
                or volume.foreground_max_m != cfg.foreground_max_m
                or volume.working_depth_range_m != (limits['minimum_depth_m'], limits['maximum_depth_m'])
                or not np.allclose(volume.vertices_base_m, raw.get('vertices_base_m'), atol=1e-12, rtol=0.)):
            _error('VOLUME_PIXEL_SOURCE_BINDING_INVALID')
        if volumes and volume.support_provenance != volumes[0].support_provenance:
            _error('MEMBER_SUPPORT_SOURCE_CHANGED')
        volumes.append(volume)
    return tuple(volumes[:-1]), volumes[-1]


def require_same_rgb_member_masks(current, previous):
    """Require one unique unordered correspondence for every real member."""
    if not 1 <= len(current) == len(previous) <= 4:
        _error('PREPLAN_INTEREST_SET_CHANGED')
    matches = []
    for permutation in itertools.permutations(range(len(previous))):
        if all(a.shape == previous[j].shape
               and np.count_nonzero(a & previous[j])/max(1,np.count_nonzero(a | previous[j])) >= .70
               for a,j in zip(current,permutation)):
            matches.append(permutation)
    if len(matches) != 1:
        _error('PREPLAN_INTEREST_SET_CHANGED_OR_AMBIGUOUS')


def reference_visibility_geometry(reference_id, set_sha256, members, envelope):
    """Bind exact frozen wire vertices, independently of fresh measured scenes.

    Do not reconstruct ray volumes here: these are the original vertices the
    perception handoff must project after every move, including its envelope.
    """
    if (not isinstance(reference_id, str) or not isinstance(set_sha256, str)
            or len(set_sha256) != 64 or not isinstance(members, (list, tuple))
            or not 1 <= len(members) <= 4):
        _error('REFERENCE_VISIBILITY_BINDING_INVALID')
    corners = []
    for value in list(members)+[envelope]:
        points = np.asarray(value.get('vertices_base_m'), dtype=float)
        if (points.shape != (8, 3) or not np.all(np.isfinite(points))
                or np.linalg.matrix_rank(points-points.mean(axis=0)) != 3):
            _error('REFERENCE_VISIBILITY_VERTICES_INVALID')
        corners.append(points.tolist())
    binding = dict(reference_id=reference_id, reference_observation_set_sha256=set_sha256,
        member_vertices_base_m=corners[:-1], envelope_vertices_base_m=corners[-1])
    binding['reference_vertices_sha256'] = digest(binding)
    return binding


def continues_rgb_episode(previous, request):
    """A completed view belongs only to its original selection generation."""
    return (isinstance(previous, dict) and previous.get('phase') == 'awaiting_measured_target'
        and parse_selection(previous.get('selection')) == parse_selection(request.get('selection')))


def reserve_episode(previous, request, source_id, selection, now_ns, mono, *, continuation=None):
    """Persist-before-plan reservation; failed plans never refund/reset budget."""
    if (request.get('version') != 1 or parse_selection(request.get('selection')) != selection
            or request.get('expected_source_id') != source_id):
        _error('REQUEST_CONTEXT_CHANGED')
    ident = request.get('request_id')
    if type(ident) is not str or not 1 <= len(ident) <= 128 or any(c.isspace() for c in ident):
        _error('REQUEST_ID_INVALID')
    reservation = request.get('reservation_request_id', ident)
    if type(reservation) is not str or not 1 <= len(reservation) <= 128 or any(c.isspace() for c in reservation):
        _error('RESERVATION_REQUEST_ID_INVALID')
    duration, maximum = request.get('duration_sec', 120.), request.get('max_observations', 3)
    if (isinstance(duration, bool) or not isinstance(duration, (int, float))
            or not math.isfinite(duration) or not 0. < duration <= 180.
            or type(maximum) is not int or not 1 <= maximum <= 3):
        _error('BUDGET_INVALID')
    prior = deepcopy(previous) if isinstance(previous, dict) else {}
    if prior:
        prior['selection'] = parse_selection(prior['selection'])
    if prior and prior.get('selection') == selection:
        if prior.get('source_id') != source_id:
            _error('EPISODE_SOURCE_CHANGED')
        if prior.get('phase') not in ('planning_rejected', 'preflight_rejected', 'awaiting_measured_target'):
            _error('EPISODE_NOT_REUSABLE_' + str(prior.get('phase')))
        preflight_retry = prior.get('phase') == 'preflight_rejected'
        if preflight_retry:
            validate_prefix_preflight_receipt(prior, now_ns=now_ns, monotonic_now=mono)
        if preflight_retry and (ident != prior.get('request_id')
                or request.get('reference_id') != prior.get('reference_id')
                or not prior.get('reference_id') or request.get('observation_index') != 1
                or prior.get('observation_index') != 1
                or not request.get('reservation_request_id')):
            _error('PREFIX_PREFLIGHT_RETRY_OWNER_CHANGED')
        continuing = prior.get('phase') == 'awaiting_measured_target'
        if continuing:
            if (not isinstance(continuation, dict) or continuation.get('status') != 'supplement_ready'
                    or continuation.get('request_id') != prior.get('request_id') or ident != prior.get('request_id')
                    or continuation.get('reference_id') != prior.get('reference_id')
                    or request.get('reference_id') != prior.get('reference_id')
                    or continuation.get('source_id') != source_id
                    or int(continuation.get('source_stamp_ns', 0)) <= int(prior.get('settled_ns', 0))
                    or continuation.get('observation_index') != prior.get('observation_index', 1)
                    or request.get('observation_index') != prior.get('observation_index', 1)+1
                    or continuation.get('deficit_code') not in ('associated_region_lacks_measured_3d', 'associated_geometry_quality_insufficient',
                        'associated_rgb_refinement_rejected', 'initial_acquisition_pending', 'observation_identity_ambiguous')
                    or type(continuation.get('association_sha256')) is not str
                    or len(continuation['association_sha256']) != 64):
                _error('SUPPLEMENT_REQUIRES_VERIFIED_POST_SETTLE_ASSOCIATION')
            prior['observation_index'] = request['observation_index']
            prior['last_association_sha256'] = continuation['association_sha256']
        if (now_ns < int(prior['created_ns']) or mono < prior['created_monotonic']
                or now_ns >= int(prior['deadline_ns']) or mono >= prior['deadline_monotonic']):
            _error('EPISODE_EXPIRED')
        if duration != prior['duration_sec'] or maximum != prior['max_observations']:
            _error('EPISODE_BUDGET_CHANGE_FORBIDDEN')
        if (reservation in prior['request_ids'] and not continuing) or prior['attempts'] >= maximum:
            _error('EPISODE_REQUEST_REPLAY_OR_BUDGET_EXHAUSTED')
    else:
        if prior:
            previous_selection = prior.get('selection', {})
            if (selection['generation'] <= previous_selection.get('generation', 0)
                    or selection['stamp_ns'] <= previous_selection.get('stamp_ns', 0)):
                _error('NEW_SELECTION_REQUIRED')
        prior = dict(version=1, selection=deepcopy(selection), source_id=source_id,
                     created_ns=str(now_ns), deadline_ns=str(now_ns+int(duration*1e9)),
                     created_monotonic=mono, deadline_monotonic=mono+duration,
                     duration_sec=duration, max_observations=maximum, attempts=0, request_ids=[], observation_index=1)
    prior['attempts'] += 1
    prior['request_ids'].append(reservation)
    prior.update(phase='reserved', request_id=ident, reservation_request_id=reservation, contact_authorized=False)
    return prior


def continuous_excursion(plan, hold, maximum_rad=math.radians(15.), *, origin=None):
    """Reject alternate IK branches including spline excursions between knots."""
    from types import SimpleNamespace
    points = list(plan.joint_trajectory.points)
    if (not 2 <= len(points) <= 4096 or len(hold.positions) != 6
            or getattr(plan.multi_dof_joint_trajectory, 'points', [])):
        _error('TRAJECTORY_STRUCTURE_INVALID')
    reference = np.asarray(hold.positions if origin is None else origin, dtype=float)
    if reference.shape != (6,) or not np.all(np.isfinite(reference)):
        _error('TRAJECTORY_EXCURSION_ORIGIN_INVALID')
    bridge = SimpleNamespace(positions=list(hold.positions), velocities=[0.]*6,
                             accelerations=[0.]*6)
    segments = [(bridge, next((p for p in points if p.time_from_start.to_sec() > 0.), None), 0.)]
    segments += [(a, b, a.time_from_start.to_sec()) for a, b in zip(points, points[1:])]
    excursion = 0.
    for first, second, start in segments:
        if second is None:
            _error('TRAJECTORY_DURATION_INVALID')
        duration = second.time_from_start.to_sec()-start
        if duration <= 0.:
            _error('TRAJECTORY_TIMES_INVALID')
        coeff = segment_coefficients(first, second, duration, 6)
        for a in np.linspace(0., 1., 9)[:-1]:
            lo, hi = _polynomial_bounds(coeff, float(a), float(a+.125))
            excursion = max(excursion, float(np.max(np.abs(lo-reference))),
                            float(np.max(np.abs(hi-reference))))
    if excursion > maximum_rad:
        _error('NONLOCAL_TRAJECTORY_BRANCH')
    return excursion


class RgbAcquisitionGateway:
    def _initialize_rgb_acquisition(self):
        self._rgb_lock = threading.RLock()
        self._rgb_pairs = OrderedDict()
        self._rgb_status = None
        self._rgb_ready_status = None
        self._rgb_reference_ack = None
        self._rgb_status_trace = deque(maxlen=256)
        self._rgb_capture_diagnostics = deque(maxlen=8)
        self._rgb_revocation = 0
        self._rgb_prefix_capability = None
        self._rgb_bridge = CvBridge()
        self._rgb_subscribers = [rospy.Subscriber('/supervisor/camera/'+stream+'/image_raw', Image,
            lambda msg, kind=kind: self._rgb_acquisition_image_cb(msg, kind),
            queue_size=2, buff_size=2**24) for stream, kind in (('color', 'rgb'), ('depth', 'depth'))]
        self._rgb_subscribers.append(rospy.Subscriber('/perception/unknown/detector_status', String,
                                      self._rgb_acquisition_status_cb, queue_size=2))
        self._rgb_subscribers.append(rospy.Subscriber('/perception/rgb_acquisition/reference_status', String,
            self._rgb_reference_ack_cb, queue_size=2))
        self._rgb_reference_pub = rospy.Publisher('/supervisor/rgb_acquisition/reference_request', String,
                                                 queue_size=1, latch=True)
        self._rgb_status_pub = rospy.Publisher('/supervisor/rgb_acquisition/status', String,
                                             queue_size=1, latch=True)
        ledger = rospy.get_param(LEDGER_PARAM, {})
        if ledger and ledger.get('phase') in ('reserved', 'executing'):
            ledger.update(phase='revoked', reason='gateway_restarted_during_episode')
            rospy.set_param(LEDGER_PARAM, ledger)

    def _rgb_acquisition_image_cb(self, message, kind):
        # Only retain bounded immutable ROS messages. No image math or disk I/O.
        stamp = message.header.stamp.to_nsec()
        with self._rgb_lock:
            self._rgb_pairs.setdefault(stamp, {})[kind] = message
            while len(self._rgb_pairs) > 32:
                self._rgb_pairs.popitem(last=False)

    def _rgb_acquisition_status_cb(self, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                return
            with self._rgb_lock:
                self._rgb_status = value
                interest = value.get('observation_interest', {})
                action = 'retained_rate_limit'
                if interest.get('observation_interest_ready') is True:
                    self._rgb_ready_status = value
                    action = 'stored'
                elif (value.get('target_locked') or value.get('target_lost')
                      or interest.get('reason') != 'rgb_observation_rate_limited'):
                    self._rgb_ready_status = None
                    action = 'cleared'
                # Scalar-only bounded ingress history. This does not preserve
                # or admit a rejected frame, and never performs image work/IO.
                if not hasattr(self, '_rgb_status_trace'):
                    self._rgb_status_trace = deque(maxlen=256)
                summary = self._rgb_status_summary(value)
                cached = (self._rgb_ready_status or {}).get('observation_interest', {})
                pair = getattr(self, '_rgb_pairs', {}).get(interest.get('source_stamp_ns'), {})
                summary.update(received_ns=rospy.Time.now().to_nsec(),
                    received_monotonic_sec=time.monotonic(), ready_cache_action=action,
                    cached_ready_stamp_ns=cached.get('source_stamp_ns'),
                    image_pair_rgb='rgb' in pair, image_pair_depth='depth' in pair)
                self._rgb_status_trace.append(summary)
        except (ValueError, TypeError):
            pass

    def _rgb_reference_ack_cb(self, message):
        try:
            value = json.loads(message.data)
            if isinstance(value, dict):
                with self._rgb_lock:
                    self._rgb_reference_ack = value
        except (TypeError, ValueError):
            pass

    @staticmethod
    def _rgb_status_summary(status):
        """Only declared scalar fields and the bounded three source stamps."""
        status = status if isinstance(status, dict) else {}
        interest = status.get('observation_interest') or {}
        handoff = status.get('rgb_acquisition_handoff') or {}
        track = interest.get('rgb_track') or {}
        def scalar(value):
            if isinstance(value, str): return value[:256]
            if value is None or type(value) in (bool, int): return value
            if type(value) is float and math.isfinite(value): return value
            return None
        result = {name:scalar(value) for name,value in (
            ('source_stamp_ns',status.get('source_stamp_ns')),
            ('interest_stamp_ns',interest.get('source_stamp_ns')),
            ('ready',interest.get('observation_interest_ready') is True),
            ('reason',interest.get('reason')), ('detail',status.get('detail')),
            ('target_locked',status.get('target_locked')),('target_lost',status.get('target_lost')),
            ('reference_id',interest.get('reference_id') or handoff.get('reference_id')),
            ('handoff_reason',handoff.get('reason')),('handoff_state',handoff.get('state')),
            ('rgb_consistent_frames',handoff.get('rgb_consistent_frames')),
            ('rgb_associated_stable',handoff.get('rgb_associated_stable')),
            ('member_count',handoff.get('rgb_interest_member_count')),
            ('track_confirmed',track.get('confirmed')),('track_reason',track.get('reason')),
            ('track_consistent_frames',track.get('consistent_frames')))}
        stamps = track.get('source_stamps_ns', ())
        result['track_stamps_ns'] = [value for value in stamps[-3:] if type(value) is int] if isinstance(stamps,(list,tuple)) else []
        return result

    def _rgb_begin_capture_diagnostic(self, stage):
        if not hasattr(self, '_rgb_capture_diagnostics'):
            self._rgb_capture_diagnostics = deque(maxlen=8)
        record = dict(stage=stage, started_ns=rospy.Time.now().to_nsec(),
            started_monotonic_sec=time.monotonic(), attempts=0, accepted=False,
            reasons={}, artifact_stages=list(getattr(self,'_rgb_artifact_frames',{})))
        self._rgb_capture_diagnostics.append(record)
        return record

    def _rgb_capture_diagnostic_sample(self, record, reason):
        record['attempts'] += 1
        key = str(reason)[:256]
        counts = record['reasons']
        if key not in counts and len(counts) >= 32:
            key = '<other_reasons>'
        counts[key] = counts.get(key,0)+1
        with self._rgb_lock:
            latest = getattr(self,'_rgb_status',None)
            cached = getattr(self,'_rgb_ready_status',None)
            interest = (cached or {}).get('observation_interest',{})
            stamp = interest.get('source_stamp_ns')
            pairs = getattr(self,'_rgb_pairs',{})
            pair = pairs.get(stamp,{})
            ack = getattr(self,'_rgb_reference_ack',None) or {}
            record['latest_status'] = self._rgb_status_summary(latest)
            record['cached_ready'] = self._rgb_status_summary(cached)
            record['latest_capture'] = dict(sampled_ns=rospy.Time.now().to_nsec(),
                ready_cache_present=cached is not None, image_pair_rgb='rgb' in pair,
                image_pair_depth='depth' in pair, image_pair_count=len(pairs),
                newest_image_stamp_ns=max(pairs,default=None),
                ready_source_age_sec=None if type(stamp) is not int else
                    (rospy.Time.now().to_nsec()-stamp)*1e-9,
                reference_ack_status=str(ack.get('status',''))[:256],
                reference_ack_stamp_ns=str(ack.get('source_stamp_ns',''))[:256])

    def _rgb_wait_initial(self, source_id, *, reference_id=''):
        deadline = time.monotonic()+3.
        last = ''
        record = self._rgb_begin_capture_diagnostic('initial')
        try:
            while time.monotonic() < deadline:
                if self._manual_control_active():
                    _error('MANUAL_CONTROL_ACTIVE')
                try:
                    result = self._rgb_capture(source_id, reference_id=reference_id)
                    self._rgb_capture_diagnostic_sample(record, 'accepted')
                    record.update(accepted=True, accepted_source_stamp_ns=result['source_stamp_ns'])
                    return result
                except (ValueError, KeyError, TypeError, ObservationPathError) as exc:
                    last = str(exc)
                    self._rgb_capture_diagnostic_sample(record, last)
                    time.sleep(.03)
            _error('INITIAL_CAPTURE_UNAVAILABLE: '+last)
        finally:
            record.update(finished_ns=rospy.Time.now().to_nsec(),
                elapsed_sec=time.monotonic()-record['started_monotonic_sec'])

    def _rgb_freeze_reference(self, episode, captured):
        # A later failed planning attempt does not erase earlier movement.
        # Such an episode must keep its original reference, never fall back to
        # a new centre-selected source through the ordinary retry transaction.
        if (int(episode.get('observation_index', 1)) > 1
                or any(int(episode.get(key, 0) or 0) > 0 for key in
                       ('submission_source_stamp_ns', 'submitted_ns', 'execution_started_ns',
                        'completed_ns', 'settled_ns'))):
            _error('MOVED_EPISODE_REQUIRES_ORIGINAL_REFERENCE')
        request = dict(version=1, request_id=episode['request_id'], source_id=episode['source_id'],
            selection=episode['selection'], source_stamp_ns=str(captured['source_stamp_ns']),
            region_mask_sha256=captured['interest']['region_mask_sha256'],
            observation_set_sha256=captured['interest']['observation_set_sha256'],
            source_rgb_sha256=captured['source_rgb_sha256'], source_depth_sha256=captured['source_depth_sha256'],
            deadline_ns=episode['deadline_ns'], max_observations=episode['max_observations'])
        self._rgb_reference_pub.publish(String(data=json.dumps(request, separators=(',', ':'))))
        deadline = min(episode['deadline_monotonic'], time.monotonic()+1.)
        while time.monotonic() < deadline:
            self._rgb_require_owner(episode)
            with self._rgb_lock:
                ack = deepcopy(self._rgb_reference_ack)
            if ack and ack.get('request_id') == episode['request_id']:
                if (ack.get('version') != 1 or ack.get('status') != 'frozen'
                        or ack.get('source_id') != episode['source_id']
                        or str(ack.get('source_stamp_ns')) != request['source_stamp_ns']
                        or ack.get('observation_set_sha256') != request['observation_set_sha256']
                        or not isinstance(ack.get('reference_id'), str) or not ack['reference_id']):
                    _error('REFERENCE_FREEZE_REJECTED: '+str(ack))
                episode['reference_id'] = ack['reference_id']
                # Preserve the producer's original wire corners exactly; fresh
                # bbox/support reconstructions never replace this FOV contract.
                episode['reference_volume_json'] = json.dumps(captured['interest']['interest_volume'], sort_keys=True)
                episode['reference_volumes_json'] = json.dumps(captured['interest']['interest_volumes'], sort_keys=True)
                episode['reference_observation_set_sha256'] = request['observation_set_sha256']
                # An explicitly ACKed new freeze after a rejected plan replaces
                # that transaction's binding. Ordinary fresh captures and a
                # continuation never enter this branch or renew the budget.
                episode.pop('reference_vertices_sha256', None)
                self._rgb_install_reference_visibility(episode)
                captured['reference_id'] = ack['reference_id']
                self._rgb_update_ledger(episode, 'reserved')
                return
            time.sleep(.02)
        _error('REFERENCE_FREEZE_ACK_TIMEOUT')

    def _rgb_install_reference_visibility(self, episode):
        binding = reference_visibility_geometry(episode['reference_id'],
            episode['reference_observation_set_sha256'],
            json.loads(episode['reference_volumes_json']), json.loads(episode['reference_volume_json']))
        if (episode.get('reference_vertices_sha256') is not None
                and episode['reference_vertices_sha256'] != binding['reference_vertices_sha256']):
            _error('REFERENCE_VISIBILITY_DIGEST_CHANGED')
        episode['reference_vertices_sha256'] = binding['reference_vertices_sha256']
        self._rgb_reference_id = episode['reference_id']
        self._rgb_reference_volume_json = episode['reference_volume_json']
        self._rgb_reference_volumes_json = episode['reference_volumes_json']
        self._rgb_reference_set_sha256 = episode['reference_observation_set_sha256']
        self._rgb_reference_vertices_sha256 = binding['reference_vertices_sha256']

    def _rgb_reference_visibility(self, captured):
        reference_id = captured.get('reference_id', '')
        if not reference_id:
            # Before ACK this is exactly the source that will be frozen.
            if captured['interest'].get('rgb_search_basis') == 'fixed_reference_volume_projection':
                _error('ORIGINAL_REFERENCE_BINDING_REQUIRED')
            return reference_visibility_geometry('', captured['interest']['observation_set_sha256'],
                captured['interest']['interest_volumes'], captured['interest']['interest_volume'])
        if reference_id != getattr(self, '_rgb_reference_id', None):
            _error('REFERENCE_VISIBILITY_ID_CHANGED')
        binding = reference_visibility_geometry(reference_id, self._rgb_reference_set_sha256,
            json.loads(self._rgb_reference_volumes_json), json.loads(self._rgb_reference_volume_json))
        if binding['reference_vertices_sha256'] != self._rgb_reference_vertices_sha256:
            _error('REFERENCE_VISIBILITY_DIGEST_CHANGED')
        return binding

    def _rgb_require_source_fk(self, context, q, transform):
        description = rospy.get_param_cached('/robot_description', '')
        model_sha = hashlib.sha256(description.encode()).hexdigest()
        if not description or model_sha != context.get('model_sha256'):
            _error('SOURCE_ROBOT_MODEL_CHANGED')
        cached = getattr(self, '_rgb_source_fk_cache', None)
        if cached is None or cached[0] != model_sha:
            cached = (model_sha, SerialUrdfFk(description, ARM_NAMES))
            self._rgb_source_fk_cache = cached
        predicted = cached[1](q) @ np.asarray(context['tool_camera'], dtype=float)
        if not np.allclose(predicted, transform, rtol=0., atol=1e-6):
            _error('SOURCE_FK_TF_MISMATCH')

    def _rgb_acquisition_revoke_event(self):
        # Latch transient manual/epoch changes; never command torque or a hold.
        if hasattr(self, '_rgb_lock'):
            with self._rgb_lock:
                self._rgb_revocation += 1
                capability = getattr(self, '_rgb_prefix_capability', None)
                if capability is not None:
                    capability['revoked'] = True

    def _rgb_prefix_actuation_event(self, status):
        """Called under command-reference lock; never acquires planner locks."""
        if not hasattr(self, '_rgb_lock'):
            return  # Ordinary startup before the optional RGB addon exists.
        with self._rgb_lock:
            capability = getattr(self, '_rgb_prefix_capability', None)
            if capability is None:
                return
            if status == 'CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE':
                capability['confirmed_seen'] = True
                return
            allowed = PENDING_BOOTSTRAP_STATES
            if capability['phase'] == 'submitted':
                allowed = allowed | {'PENDING:AWAITING_ENCODER_RESPONSE'}
            if capability['confirmed_seen'] or status not in allowed:
                capability['revoked'] = True

    def _rgb_context(self, stamp):
        get = rospy.get_param_cached
        tool = _matrix(self._observation_view_tf_buffer.lookup_transform(
            'tool0', 'camera_link', stamp, rospy.Duration(.1)))
        latest = _matrix(self._observation_view_tf_buffer.lookup_transform(
            'tool0', 'camera_link', rospy.Time(0), rospy.Duration(.1)))
        if not np.array_equal(tool, latest):
            _error('MOUNT_CHANGED')
        handeye = deepcopy(get('/handeye', {}))
        camera = camera_geometry_snapshot(get('/camera'))
        if (handeye.get('mode') != 'eye_on_hand' or handeye.get('parent_frame') != 'tool0'
                or handeye.get('camera_frame') != 'camera_link'
                or handeye.get('child_frame', 'camera_link') != 'camera_link'
                or camera.get('frame_id') != 'camera_link' or camera.get('align_depth_to_color') is not True):
            _error('RIGID_CAMERA_INSTALLATION_INVALID')
        return dict(epoch_ns=self._control_reference_epoch_ns,
            selection=parse_selection(get('/grasp_mode/selection')),
            camera=camera, perception=deepcopy(get('/unknown_perception', {})), handeye=handeye,
            model_sha256=hashlib.sha256(get('/robot_description', '').encode()).hexdigest(),
            tool_camera=tool.tolist(), observation_view_parameters={key:
                get('/grasp_6d/remote/'+key, value) for key, value in VIEW_DEFAULTS.items()})

    def _rgb_require_owner(self, episode, *, moving=False):
        if self._manual_control_active():
            _error('MANUAL_CONTROL_ACTIVE')
        if self._rgb_revocation != episode['revocation']:
            _error('CONTROL_REVOKED')
        now = rospy.Time.now().to_nsec()
        if (now < int(episode['created_ns']) or time.monotonic() < episode['created_monotonic']
                or now >= int(episode['deadline_ns']) or time.monotonic() >= episode['deadline_monotonic']):
            _error('EPISODE_EXPIRED')
        with self._observation_context_lock:
            if (not self._observation_task_connection_available()
                    or self._observation_task_requires_inactive or self._observation_contexts.active
                    or self._observation_task_state_stamp_ns <= 0):
                _error('INDEPENDENT_IDLE_OWNER_REQUIRED')
        if digest(self._rgb_context(rospy.Time(0))) != episode['source_id']:
            _error('SOURCE_CONFIGURATION_CHANGED')
        if (rospy.get_param_cached('/robot/observation_tracking_contract_enabled', False) is not True
                or rospy.get_param_cached('/alicia_d_driver_node/endpoint_feedback_trim_enabled', False) is not False):
            _error('EXECUTION_CONTRACT_UNAVAILABLE')
        self._rgb_require_actuation(episode)
        if not moving:
            q = np.asarray(self._rgb_measured_arm_snapshot())
            if q.shape != (6,) or np.max(np.abs(q-episode['q_source'])) > .004:
                _error('MEASURED_POSE_CHANGED_BEFORE_SUBMISSION')

    def _rgb_admit_actuation_prefix(self, episode, captured):
        """Grant only this RPC's first, budgeted observation a pending prefix."""
        # Match _actuation_cb and _control_reference_epoch_cb lock order.
        with self._command_reference_lock:
            status = self._fresh_actuation_status()
            with self._rgb_lock:
                return self._rgb_admit_actuation_prefix_locked(episode, captured, status)

    def _rgb_admit_actuation_prefix_locked(self, episode, captured, status):
        capability = getattr(self, '_rgb_prefix_capability', None)
        if episode.get('actuation_prefix_attempted') and episode.get('observation_index') == 1:
            # A real retry is rebound only after checking its private live
            # capability and the exact persisted no-submission receipt.
            # Check this before CONFIRMED: a recovered status cannot erase a
            # fault that was observed between RPCs.
            if (capability is None or capability['episode'] is not episode
                    or capability.get('retry_admitted') is not True):
                _error('ACTUATION_PREFIX_ALREADY_CONSUMED')
            self._rgb_require_actuation_locked(episode, status)
            return True
        if status.startswith('CONFIRMED:'):
            return False
        if status not in PENDING_BOOTSTRAP_STATES:
            _error('ACTUATION_UNCONFIRMED')
        if (episode.get('actuation_prefix_attempted') or episode['attempts'] != 1
                or episode.get('observation_index') != 1):
            _error('ACTUATION_PREFIX_ALREADY_CONSUMED')
        epoch = captured['source_context']['epoch_ns']
        if type(epoch) is not int or epoch <= 0 or epoch != self._control_reference_epoch_ns:
            _error('ACTUATION_PREFIX_EPOCH_CHANGED')
        # This capability is never deserialized from a request or ledger.
        # Object identity binds it to one live owner under the planner lock.
        self._rgb_prefix_capability = dict(episode=episode, epoch_ns=epoch,
            source_id=episode['source_id'], phase='preparing', revoked=False,
            confirmed_seen=False, trajectory_sha256=None, trajectory=None,
            submission_started=False, retry_stage='', original_capture=deepcopy(captured))
        episode['actuation_prefix_attempted'] = True
        episode['actuation_prefix_submission_started'] = False
        return True

    @staticmethod
    def _rgb_prefix_ledger_digest(episode):
        return digest(ros_parameter_value({key:value for key,value in episode.items() if key != 'q_source'}))

    def _rgb_prefix_retry_guard(self, previous):
        capability = getattr(self, '_rgb_prefix_capability', None)
        try:
            validate_prefix_preflight_receipt(previous, now_ns=rospy.Time.now().to_nsec(),
                                              monotonic_now=time.monotonic())
            if (capability is None or capability.get('phase') != 'retry_ready'
                    or capability.get('revoked') or capability.get('submission_started')
                    or capability.get('retry_ledger_sha256') != self._rgb_prefix_ledger_digest(previous)):
                _error('PREFIX_PREFLIGHT_RETRY_NO_LIVE_PROOF')
            self._rgb_require_owner(capability['episode'])
        except Exception:
            if capability is not None:
                capability['revoked'] = True
            raise
        return capability

    def _rgb_prefix_retry_capture(self, capability, captured):
        previous = capability['episode']
        proof = previous['actuation_prefix_preflight']
        original = capability['original_capture']
        interest = captured.get('interest', {})
        if (captured.get('reference_id') != previous.get('reference_id')
                or captured.get('source_id') != capability['source_id']
                or captured.get('source_context', {}).get('epoch_ns') != capability['epoch_ns']
                or interest.get('rgb_search_basis') != 'fixed_reference_volume_projection'
                or interest.get('reference_id') != previous.get('reference_id')
                or not isinstance(interest.get('association_sha256'), str)
                or len(interest['association_sha256']) != 64
                or captured['source_stamp_ns'] <= int(proof['last_capture_source_stamp_ns'])
                or np.max(np.abs(np.asarray(captured['q_source'])-original['q_source'])) > .004):
            _error('PREFIX_PREFLIGHT_RETRY_SOURCE_CHANGED')
        require_same_rgb_member_masks(captured['masks'], original['masks'])
        self._rgb_require_owner(previous)

    def _rgb_finish_preflight_rejection(self, episode, reason):
        capability = getattr(self, '_rgb_prefix_capability', None)
        if (capability is None or capability['episode'] is not episode
                or capability.get('phase') != 'preparing'
                or capability.get('submission_started')
                or capability.get('retry_stage') not in ('planning', 'candidate_preflight', 'path_preflight')
                or not episode.get('reference_id')):
            return False
        self._rgb_require_owner(episode)
        original = capability['original_capture']
        last_capture = self._rgb_artifact_frames.get('postplan',
            self._rgb_artifact_frames.get('initial', original))
        proof = {key:deepcopy(episode[key]) for key in (
            'selection', 'request_id', 'reference_id', 'source_id', 'attempts',
            'created_ns', 'deadline_ns', 'created_monotonic', 'deadline_monotonic',
            'duration_sec', 'max_observations', 'observation_index')}
        proof.update(kind='rgb_prefix_preflight_no_submission_v1', submission_started=False,
            original_reference_preserved=True, epoch_ns=capability['epoch_ns'],
            q_source=list(original['q_source']),
            failed_reservation_request_id=episode['reservation_request_id'],
            last_capture_source_stamp_ns=str(last_capture['source_stamp_ns']))
        episode['actuation_prefix_preflight'] = proof
        capability['phase'] = 'retry_ready'
        try:
            validate_prefix_preflight_receipt(dict(episode, phase='preflight_rejected'),
                now_ns=rospy.Time.now().to_nsec(), monotonic_now=time.monotonic())
            self._rgb_update_ledger(episode, 'preflight_rejected', reason=reason)
            self._rgb_require_owner(episode)
        except Exception:
            capability['revoked'] = True
            raise
        capability['retry_ledger_sha256'] = self._rgb_prefix_ledger_digest(episode)
        return True

    def _rgb_require_actuation(self, episode):
        with self._command_reference_lock:
            status = self._fresh_actuation_status()
            with self._rgb_lock:
                self._rgb_require_actuation_locked(episode, status)

    def _rgb_require_actuation_locked(self, episode, status):
        capability = getattr(self, '_rgb_prefix_capability', None)
        if capability is None or capability['episode'] is not episode:
            if not status.startswith('CONFIRMED:'):
                _error('ACTUATION_UNCONFIRMED')
            return
        valid = (not capability['revoked']
            and capability['phase'] in ('preparing', 'retry_ready', 'submitted')
            and capability['epoch_ns'] == self._control_reference_epoch_ns
            and capability['source_id'] == episode['source_id'])
        if capability['phase'] == 'submitted':
            valid = (valid and capability['trajectory'] is not None
                and wire_digest(capability['trajectory']) == capability['trajectory_sha256']
                and episode.get('actuation_prefix_trajectory_sha256') == capability['trajectory_sha256'])
        if valid and status == 'CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE':
            capability['confirmed_seen'] = True
            return
        allowed = PENDING_BOOTSTRAP_STATES
        if capability['phase'] == 'submitted':
            allowed = allowed | {'PENDING:AWAITING_ENCODER_RESPONSE'}
        if valid and not capability['confirmed_seen'] and status in allowed:
            return
        capability['revoked'] = True
        _error('ACTUATION_PREFIX_REVOKED: '+status)

    def _rgb_prefix_curve(self, episode, plan, desired):
        """Recheck the actual retimed curve and controller bridge, not knots."""
        capability = getattr(self, '_rgb_prefix_capability', None)
        if capability is None or capability['episode'] is not episode:
            return None
        if capability['phase'] != 'preparing':
            _error('ACTUATION_PREFIX_ALREADY_SUBMITTED')
        fingerprint = wire_digest(plan)
        if capability['trajectory_sha256'] not in (None, fingerprint):
            _error('ACTUATION_PREFIX_CURVE_CHANGED')
        # Controller timing may add a bridge or alter derivatives after the
        # waypoint clip. Both the desired baseline and the measured source
        # must bound every point on that final continuous command curve.
        excursion = continuous_excursion(plan, desired, maximum_rad=.025+1e-12)
        measured = dict(zip(ARM_NAMES, episode['q_source']))
        measured_excursion = continuous_excursion(plan, desired, maximum_rad=.025+1e-12,
            origin=[measured[name] for name in plan.joint_trajectory.joint_names])
        terminal = list(plan.joint_trajectory.points[-1].positions)
        if max(abs(a-b) for a,b in zip(terminal, desired.positions)) < .023-1e-9:
            _error('ACTUATION_PREFIX_INSUFFICIENT_COMMAND_DELTA')
        capability.update(trajectory_sha256=fingerprint, trajectory=plan)
        return dict(kind='one_time_rgb_observation_prefix', maximum_joint_delta_rad=.025,
            request_id=episode['request_id'], observation_index=episode['observation_index'],
            source_epoch_ns=capability['epoch_ns'],
            continuous_command_excursion_rad=excursion,
            continuous_measured_source_excursion_rad=measured_excursion,
            terminal_joint_names=list(plan.joint_trajectory.joint_names),
            terminal_joint_positions=terminal, requested_target_reached=False)

    def _rgb_wait_prefix_confirmation(self, episode):
        capability = getattr(self, '_rgb_prefix_capability', None)
        if (capability is None or capability['episode'] is not episode
                or capability['phase'] != 'submitted'):
            _error('ACTUATION_PREFIX_SUBMISSION_REQUIRED')
        deadline = min(episode['deadline_monotonic'], time.monotonic()+1.)
        while time.monotonic() < deadline:
            self._rgb_require_owner(episode, moving=True)
            if self._fresh_actuation_status() == 'CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE':
                return
            time.sleep(.02)
        _error('ACTUATION_PREFIX_NO_MEASURED_RESPONSE')

    def _rgb_capture(self, expected_source_id, *, prior=None, reference_id=''):
        with self._rgb_lock:
            status = deepcopy(self._rgb_ready_status)
            interest = (status or {}).get('observation_interest', {})
            ns = interest.get('source_stamp_ns')
            pair = dict(self._rgb_pairs.get(ns, {}))
            association = deepcopy(self._rgb_reference_ack)
        if (status is None or status.get('target_locked') or status.get('target_lost')
                or interest.get('observation_interest_ready') is not True
                or set(pair) != {'rgb', 'depth'}):
            _error('FRESH_READY_RGB_D_SOURCE_REQUIRED')
        require_fresh_source(ns, rospy.Time.now().to_nsec())
        context = self._rgb_context(pair['rgb'].header.stamp)
        if (digest(context) != expected_source_id or interest.get('source_id') != expected_source_id
                or context != interest.get('source_context')):
            _error('DETECTOR_SOURCE_CONTEXT_CHANGED')
        camera = context['camera']
        from alicia_flexible_grasp.vision.rgbd_rectification import require_rectified_source_after_activation
        require_rectified_source_after_activation(camera, ns)
        if (camera.get('align_depth_to_color') is not True or camera.get('frame_id') != 'camera_link'
                or any(m.header.frame_id != 'camera_link' or m.header.stamp.to_nsec() != ns
                       for m in pair.values())):
            _error('IMAGE_ALIGNMENT_INVALID')
        rgb = np.array(self._rgb_bridge.imgmsg_to_cv2(pair['rgb'], 'bgr8'), copy=True)
        raw = np.array(self._rgb_bridge.imgmsg_to_cv2(pair['depth'], 'passthrough'), copy=True)
        if raw.shape != (camera['height'], camera['width']) or rgb.shape != raw.shape+(3,):
            _error('IMAGE_DIMENSIONS_CHANGED')
        rgb_sha = hashlib.sha256(np.ascontiguousarray(rgb).tobytes()).hexdigest()
        depth_sha = hashlib.sha256(np.ascontiguousarray(raw).tobytes()).hexdigest()
        if (interest.get('source_rgb_sha256') != rgb_sha
                or interest.get('source_depth_sha256') != depth_sha):
            _error('SOURCE_IMAGE_BYTES_CHANGED')
        if raw.dtype == np.uint16:
            depth = raw.astype(float)*float(camera['depth_scale'])
        elif raw.dtype in (np.float32, np.float64):
            depth = raw.astype(float)
        else:
            _error('DEPTH_ENCODING_INVALID')
        with self._endpoint_feedback_lock:
            messages = list(self._endpoint_feedback_history['accepted'])
        feedback = []
        for msg in messages:
            if msg.header.frame_id != 'sdk_measured' or len(msg.name) != len(msg.position):
                continue
            values = dict(zip(msg.name, msg.position))
            if len(values) == len(msg.name) and all(name in values for name in ARM_NAMES):
                feedback.append((msg.header.stamp.to_nsec(), [values[name] for name in ARM_NAMES]))
        q, lo, hi = bracket_feedback(feedback, ns,
            max(context['epoch_ns'], context['selection']['stamp_ns']))
        if (not np.allclose(q, interest.get('q_source'), atol=1e-12, rtol=0.)
                or np.max(np.abs(q-self._rgb_measured_arm_snapshot())) > .004):
            _error('SOURCE_JOINT_BINDING_CHANGED')
        T = _matrix(self._observation_view_tf_buffer.lookup_transform(
            'base_link', 'camera_link', pair['rgb'].header.stamp, rospy.Duration(.1)))
        if not np.array_equal(T, interest.get('T_base_camera')):
            _error('SOURCE_TF_CHANGED')
        self._rgb_require_source_fk(context, q, T)
        cfg = Config(**{k:v for k,v in context['perception'].items() if k in Config.__dataclass_fields__})
        reference_id = reference_id or (prior or {}).get('reference_id', '')
        policy = CenterSearchPolicy(center_xy=(.5, .5), radius_fraction=cfg.center_radius_fraction)
        associated = interest.get('rgb_search_basis') == 'fixed_reference_volume_projection'
        if associated:
            original = json.loads(getattr(self, '_rgb_reference_volume_json', '{}'))
            if (not reference_id or interest.get('reference_id') != reference_id
                    or digest(original) != digest(interest.get('association_reference_volume'))
                    or interest.get('association_reference_set_sha256') != getattr(self, '_rgb_reference_set_sha256', None)
                    or digest(json.loads(getattr(self, '_rgb_reference_volumes_json', '[]')))
                        != digest(interest.get('association_reference_volumes'))
                    or not isinstance(interest.get('association_sha256'), str)
                    or len(interest['association_sha256']) != 64):
                _error('ORIGINAL_REFERENCE_BINDING_REQUIRED')
            # Project the exact frozen vertices with the producer's canonical
            # arithmetic. Rebuilding a volume or using an equivalent scalar
            # pinhole formula can change the policy digest by one float ulp.
            projected = np.asarray(project_rgb_observation_vertices(original['vertices_base_m'],
                T_base_camera=T,
                camera_matrix=[[camera['fx'],0.,camera['cx']], [0.,camera['fy'],camera['cy']], [0.,0.,1.]],
                image_shape=depth.shape, working_depth_range_m=original['working_depth_range_m']))
            policy = projected_rgb_search_policy(policy, projected, depth.shape)
            if (digest(asdict(policy)) != digest(interest.get('rgb_search_policy'))
                    or not np.allclose(projected, interest.get('association_projected_volume_pixels'),
                                       atol=1e-8,rtol=0.)):
                _error('ORIGINAL_REFERENCE_SEARCH_POLICY_CHANGED')
        elif reference_id:
            _error('FROZEN_REFERENCE_CANNOT_RETURN_TO_CENTER_SELECTION')
        elif digest(asdict(policy)) != digest(interest.get('rgb_search_policy')):
            _error('INITIAL_RGB_SEARCH_POLICY_CHANGED')
        from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask
        rgb_valid = rectification_rgb_valid_mask(camera, rgb.shape[:2])
        proposals = (propose_rgb_observation_regions(rgb, policy=policy) if rgb_valid is None else
            propose_rgb_observation_regions(rgb, policy=policy, rgb_valid=rgb_valid))
        regions = verify_rgb_interest_members(interest, proposals, policy, associated=associated)
        region = regions[0]  # Independent first-member archive, never a union mask.
        if associated:
            if (not isinstance(association,dict) or association.get('reference_id') != reference_id
                    or association.get('source_id') != expected_source_id
                    or association.get('status') not in ('frozen','supplement_ready')):
                _error('REFERENCE_ACK_CHANGED')
            if association.get('status') == 'supplement_ready' and (
                    str(association.get('source_stamp_ns')) != str(ns)
                    or association.get('region_mask_sha256') != interest['region_mask_sha256']
                    or association.get('observation_set_sha256') != interest['observation_set_sha256']
                    or association.get('source_rgb_sha256') != rgb_sha
                    or association.get('source_depth_sha256') != depth_sha
                    or association.get('association_sha256') != interest['association_sha256']):
                _error('FRESH_ORIGINAL_REFERENCE_ASSOCIATION_REQUIRED')
        track = interest.get('rgb_track', {})
        stamps = track.get('source_stamps_ns', [])
        if (track.get('confirmed') is not True or len(stamps) != 3 or stamps[-1] != ns
                or track.get('source_id') != expected_source_id
                or track.get('generation') != context['selection']['generation']
                or any(type(t) is not int for t in stamps) or any(a >= b for a,b in zip(stamps, stamps[1:]))
                or stamps[0] <= max(context['epoch_ns'], context['selection']['stamp_ns'])):
            _error('RGB_STABILITY_BINDING_INVALID')
        K = np.array([[camera['fx'],0.,camera['cx']], [0.,camera['fy'],camera['cy']], [0.,0.,1.]])
        limits = effective_observation_view_limits(context['observation_view_parameters'])
        volumes, volume = decode_rgb_interest_volumes(interest, regions, ns=ns, K=K, T=T, cfg=cfg, limits=limits)
        coverage = measured_support_coverage(depth, (camera['fx'],camera['fy'],camera['cx'],camera['cy']),
            T, volume.support_plane_base, cfg, interest['support_metrics']['support_ring_outer_radius_px'])
        if coverage['spatial_quadrants'] < 3 or coverage['measured_support_pixel_count'] < 120:
            _error('SUPPORT_MEASUREMENT_COVERAGE_INVALID')
        if prior is not None:
            require_same_rgb_member_masks(tuple(item.mask for item in regions), prior['masks'])
            if np.max(np.abs(q-prior['q_source'])) > .004 or ns < prior['source_stamp_ns']:
                _error('PREPLAN_INTEREST_CHANGED')
        require_fresh_source(ns, rospy.Time.now().to_nsec())
        return dict(interest=interest, source_context=context, source_id=expected_source_id,
            source_stamp_ns=ns, volume=volume, volumes=volumes, regions=regions,
            masks=tuple(item.mask for item in regions), depth=depth, rgb=rgb, mask=region.mask,
            q_source=q.tolist(), accepted_before=lo, accepted_after=hi,
            depth_raw=raw, depth_scale=float(camera['depth_scale']) if raw.dtype == np.uint16 else 1.,
            source_depth_sha256=depth_sha, source_rgb_sha256=rgb_sha, cfg=cfg, limits=limits,
            reference_id=reference_id, association=association if reference_id else None)

    def _rgb_capture_after_plan(self, episode, initial):
        deadline = min(episode['deadline_monotonic'], time.monotonic()+3.)
        last = ''
        record = self._rgb_begin_capture_diagnostic('post_plan')
        try:
            while time.monotonic() < deadline:
                self._rgb_require_owner(episode)
                try:
                    result = self._rgb_capture(episode['source_id'], prior=initial)
                    self._rgb_capture_diagnostic_sample(record, 'accepted')
                    record.update(accepted=True, accepted_source_stamp_ns=result['source_stamp_ns'])
                    return result
                except (ValueError, KeyError, TypeError, ObservationPathError) as exc:
                    last = str(exc)
                    self._rgb_capture_diagnostic_sample(record, last)
                    time.sleep(.03)
            _error('NO_FRESH_POST_PLAN_SOURCE: '+last)
        finally:
            record.update(finished_ns=rospy.Time.now().to_nsec(),
                elapsed_sec=time.monotonic()-record['started_monotonic_sec'])

    def _rgb_update_ledger(self, episode, phase, **values):
        episode.update(phase=phase, **values)
        persisted = {k:deepcopy(v) for k,v in episode.items() if k not in ('q_source',)}
        rospy.set_param(LEDGER_PARAM, ros_parameter_value(persisted))
        status = dict(persisted, observation_only=True, measured_target_established=False,
                      contact_authorized=False)
        rospy.set_param(STATUS_PARAM, ros_parameter_value(status))
        # Perception already owns the immutable reference pixels/volumes. The
        # ledger retains full geometry for diagnostics/CLI; duplicating those
        # strings on this state topic can exceed its bounded 16 KiB ingress
        # for a valid four-member reference plus its no-submission receipt.
        event = {key:value for key,value in status.items()
                 if key not in ('reference_volume_json', 'reference_volumes_json')}
        self._rgb_status_pub.publish(String(data=json.dumps(event, separators=(',', ':'))))

    def handle_rgb_acquisition_pose_strict(self, req):
        if req.execute is not True:
            return SetTargetPoseResponse(False, 'RGB_ACQUISITION_EXECUTION_RPC_REQUIRES_EXPLICIT_EXECUTE')
        with self._planner_operation_lock():
            episode = None
            planner = None
            original = self.planner
            self._rgb_artifact_frames = {}
            self._rgb_artifact_plan = None
            self._rgb_artifact_model = ''
            self._rgb_artifact_failure = None
            # Keep a no-submission retry capability alive between RPCs so
            # status/manual/epoch callbacks can irreversibly revoke it.
            if (getattr(self, '_rgb_prefix_capability', None) or {}).get('phase') != 'retry_ready':
                self._rgb_prefix_capability = None
            self._rgb_capture_diagnostics = deque(maxlen=8)
            capture_started_monotonic = time.monotonic()
            request = {}
            failure = ''
            preflight_retry = False
            try:
                from .rgb_acquisition_artifacts import require_storage
                require_storage()
                request = deepcopy(rospy.get_param(REQUEST_PARAM, {}))
                if request.get('target_sha256') != wire_digest(req.target) or req.target.header.frame_id != 'base_link':
                    _error('REQUEST_TARGET_BINDING_INVALID')
                if self._manual_control_active():
                    _error('MANUAL_CONTROL_ACTIVE')
                previous = rospy.get_param(LEDGER_PARAM, {})
                same_selection = (previous and parse_selection(previous.get('selection'))
                    == parse_selection(request.get('selection')))
                preflight_retry = bool(same_selection and previous.get('phase') == 'preflight_rejected')
                retry_capability = self._rgb_prefix_retry_guard(previous) if preflight_retry else None
                if (same_selection and previous.get('phase') == 'planning_rejected'
                        and previous.get('actuation_prefix_attempted')):
                    _error('PREFIX_PREFLIGHT_RETRY_NO_LIVE_PROOF')
                if not preflight_retry:
                    self._rgb_prefix_capability = None
                continuation = continues_rgb_episode(previous, request)
                reference = previous['reference_id'] if preflight_retry else request.get('reference_id', '') if continuation else ''
                if continuation or preflight_retry:
                    self._rgb_install_reference_visibility(previous)
                initial = self._rgb_wait_initial(request.get('expected_source_id'), reference_id=reference)
                if preflight_retry:
                    try:
                        self._rgb_prefix_retry_capture(retry_capability, initial)
                    except Exception:
                        retry_capability['revoked'] = True
                        raise
                self._rgb_artifact_frames['initial'] = initial
                episode = reserve_episode(previous, request,
                    initial['source_id'], initial['source_context']['selection'],
                    rospy.Time.now().to_nsec(), time.monotonic(), continuation=initial.get('association'))
                episode.update(revocation=self._rgb_revocation,
                    q_source=list(retry_capability['original_capture']['q_source']) if preflight_retry else initial['q_source'])
                if preflight_retry:
                    episode.pop('actuation_prefix_preflight', None)
                    retry_capability.update(episode=episode, phase='preparing', retry_admitted=True,
                        retry_stage='', trajectory_sha256=None, trajectory=None, retry_ledger_sha256=None)
                prefix = self._rgb_admit_actuation_prefix(episode, initial)
                self._rgb_require_owner(episode)
                self._rgb_update_ledger(episode, 'reserved')
                if not continuation and not preflight_retry:
                    self._rgb_freeze_reference(episode, initial)
                planner = self._rgb_observation_planner()
                self.planner = planner
                planner._last_observation_path_audit = {}
                timing = self._configure_observation_timing(planner)
                self._configure_controller_reference_guard(planner)
                planner.observation_path_guard_required = True
                planner.observation_tracking_contract_required = True
                from .contracted_trajectory import ContractedTrajectoryExecutor
                if getattr(planner, 'observation_trajectory_executor', None) is None:
                    planner.observation_trajectory_executor = ContractedTrajectoryExecutor.connect()
                for operation in (self._ensure_trajectory_controllers_started,
                                  self._synchronize_trajectory_controller_to_feedback,
                                  self._wait_for_stationary_rgb_feedback):
                    self._rgb_require_owner(episode)
                    ok, message = operation()
                    if not ok:
                        _error(message)
                self._rgb_require_owner(episode)
                if prefix:
                    self._rgb_prefix_capability['retry_stage'] = 'planning'
                ok, message = planner.move_to_pose(req.target, execute=False, allow_fallbacks=False)
                if not ok:
                    failure = message
                    if not self._rgb_finish_preflight_rejection(episode, message):
                        self._rgb_update_ledger(episode, 'revoked' if prefix else 'planning_rejected', reason=message)
                    return SetTargetPoseResponse(False, 'RGB_ACQUISITION_PLAN_REJECTED: '+message)
                if prefix:
                    self._rgb_prefix_capability['retry_stage'] = ''
                captured = self._rgb_capture_after_plan(episode, initial)
                self._rgb_artifact_frames['postplan'] = dict(captured)
                planner.observation_path_validator = lambda plan: self._rgb_validate_path(
                    episode, captured, plan, planner, timing)
                planner.observation_execution_authorized = lambda: self._rgb_authorized(episode, planner, timing)
                planner.observation_submission_revalidate = lambda plan, audit: self._rgb_submission(
                    episode, captured, plan, audit, planner, timing)
                execute = (planner.execute_cached_observation_prefix if prefix
                           else planner.execute_cached_strict_pose)
                if prefix:
                    # Clipping, retiming and controller-reference timing are
                    # also pure preflight. The irreversible submission gate
                    # clears this stage before any transport can be invoked.
                    self._rgb_prefix_capability['retry_stage'] = 'candidate_preflight'
                ok, message = execute(req.target)
                if not ok:
                    failure = message
                    if not self._rgb_finish_preflight_rejection(episode, message):
                        phase = ('planning_rejected' if not prefix and episode['phase'] == 'reserved'
                                 and self._rgb_authorized(episode, planner, timing) else 'revoked')
                        self._rgb_update_ledger(episode, phase, reason=message)
                    return SetTargetPoseResponse(False, 'RGB_ACQUISITION_EXECUTION_FAILED: '+message)
                completed = rospy.Time.now().to_nsec()
                self._rgb_require_owner(episode, moving=True)
                if prefix:
                    self._rgb_wait_prefix_confirmation(episode)
                ok, settled = self._wait_for_stationary_rgb_feedback()
                if not ok:
                    _error('POST_EXECUTION_NOT_SETTLED: '+settled)
                self._rgb_require_owner(episode, moving=True)
                if prefix and self._fresh_actuation_status() != 'CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE':
                    _error('ACTUATION_PREFIX_CONFIRMATION_LOST_AFTER_SETTLE')
                outcome = (dict(receipt='actuation_prefix_confirmed_requires_fresh_replan',
                    actuation_prefix=deepcopy(planner._last_observation_path_audit['actuation_prefix']),
                    requested_target_pose_sha256=wire_digest(req.target), requested_target_reached=False)
                    if prefix else dict(target_pose_sha256=wire_digest(req.target),
                        receipt='noncontact_observation_completed_requires_new_measured_target_and_rgb_association'))
                if not prefix:
                    # Earlier prefix evidence remains in its nested audit;
                    # its partial-target result is not this new view's result.
                    episode.pop('requested_target_reached', None)
                    episode.pop('requested_target_pose_sha256', None)
                self._rgb_update_ledger(episode, 'awaiting_measured_target',
                    completed_ns=str(completed), settled_ns=str(rospy.Time.now().to_nsec()),
                    trajectory_sha256=planner._last_observation_path_audit['trajectory_sha256'],
                    source_stamp_ns=str(captured['source_stamp_ns']),
                    source_rgb_sha256=captured['source_rgb_sha256'], source_depth_sha256=captured['source_depth_sha256'],
                    **outcome)
                if prefix:
                    return SetTargetPoseResponse(True,
                        'RGB_ACQUISITION_PREFIX_CONFIRMED_REQUIRES_FRESH_REPLAN; requested observation target not reached')
                return SetTargetPoseResponse(True, 'RGB_ACQUISITION_OBSERVATION_SETTLED; awaiting fresh measured 3D target and RGB association')
            except Exception as exc:
                failure = str(exc)
                capability = getattr(self, '_rgb_prefix_capability', None)
                if episode is None and preflight_retry and capability and capability.get('revoked'):
                    self._rgb_update_ledger(capability['episode'], 'revoked', reason=str(exc))
                if episode is not None:
                    reusable = False
                    if episode['phase'] == 'reserved':
                        try:
                            if self._rgb_finish_preflight_rejection(episode, str(exc)):
                                return SetTargetPoseResponse(False, 'RGB_ACQUISITION_REJECTED: '+str(exc))
                            self._rgb_require_owner(episode)
                            reusable = not episode.get('actuation_prefix_attempted')
                        except Exception:
                            pass
                    self._rgb_update_ledger(episode, 'planning_rejected' if reusable else 'revoked', reason=str(exc))
                return SetTargetPoseResponse(False, 'RGB_ACQUISITION_REJECTED: '+str(exc))
            finally:
                capability = getattr(self, '_rgb_prefix_capability', None)
                if not (capability and capability.get('phase') == 'retry_ready'
                        and not capability.get('revoked')):
                    self._rgb_prefix_capability = None
                if planner is not None:
                    planner._last_pose_plan = None
                    planner.observation_path_validator = None
                    planner.observation_execution_authorized = None
                    planner.observation_submission_revalidate = None
                self.planner = original
                # Disk I/O is intentionally after execution/settling or a
                # pre-send rejection, never in the .8 s proof/send boundary.
                try:
                    from .rgb_acquisition_artifacts import save_artifacts
                    with self._rgb_lock:
                        capture_trace = list(getattr(self,'_rgb_status_trace',()))
                    destination = save_artifacts(self._rgb_artifact_frames,
                        dict(request=request, episode=episode, failure=failure,
                             failed_proof=getattr(self,'_rgb_artifact_failure',None),
                             capture_diagnostics=list(self._rgb_capture_diagnostics),
                             capture_started_monotonic=capture_started_monotonic,
                             status_trace=capture_trace),
                        audit=getattr(planner, '_last_observation_path_audit', None),
                        trajectory=self._rgb_artifact_plan, robot_description=self._rgb_artifact_model)
                    rospy.loginfo('RGB acquisition evidence saved: %s', destination)
                except Exception as artifact_error:
                    rospy.logerr('RGB acquisition evidence save failed (no fallback): %s', artifact_error)
                    capability = getattr(self, '_rgb_prefix_capability', None)
                    if capability is not None and capability.get('phase') == 'retry_ready':
                        with self._rgb_lock:
                            capability['revoked'] = True
                            self._rgb_prefix_capability = None
                        try:
                            self._rgb_update_ledger(capability['episode'], 'revoked',
                                reason='PREFIX_PREFLIGHT_EVIDENCE_SAVE_FAILED: '+str(artifact_error))
                        except Exception as persistence_error:
                            rospy.logerr('RGB retry revocation persistence failed: %s', persistence_error)
                self._rgb_artifact_frames = {}

    def _rgb_observation_planner(self):
        from .moveit_planner import MoveItPlanner
        planner = getattr(self, '_rgb_planner', None)
        if planner is None or not planner.ready:
            planner = MoveItPlanner(self.manipulator_group, self.gripper_group, self.velocity)
            if not planner.ready:
                _error('PLANNER_UNAVAILABLE')
            limit = rospy.get_param('/robot/observation_max_joint_velocity_rad_s', .08)
            if (isinstance(limit,bool) or not isinstance(limit,(int,float)) or not math.isfinite(limit)
                    or not 0. < limit <= planner.strict_execution_max_joint_velocity_rad_s):
                _error('OBSERVATION_VELOCITY_INVALID')
            planner.strict_execution_max_joint_velocity_rad_s = float(limit)
            planner.strict_execution_joint_velocity_limits_rad_s = {}
            planner.observation_branch_hints_enabled = True
            self._rgb_planner = planner
        return planner

    def _rgb_authorized(self, episode, planner, timing):
        try:
            self._rgb_require_owner(episode, moving=True)
            if not self._observation_timing_current(planner, timing):
                _error('TIMING_CHANGED')
            return True
        except Exception:
            return False

    def _rgb_member_visibility(self, captured, plan, model, desired):
        context = captured['source_context']; camera = context['camera']
        members = captured['volumes']
        if not 1 <= len(members) <= 4:
            _error('EXPLICIT_RGB_INTEREST_SET_REQUIRED')
        binding = self._rgb_reference_visibility(captured)
        deadline = time.monotonic()+5.
        def validate(corners):
            remaining = deadline-time.monotonic()
            if remaining <= 0.:
                _error('MEMBER_VISIBILITY_BUDGET_EXHAUSTED')
            return validate_observation_target_trajectory(plan, model,
                np.asarray(context['tool_camera']), np.asarray(corners),
                [camera[k] for k in ('fx','fy','cx','cy')], camera['width'],camera['height'],
                desired, max_seconds=remaining, **captured['limits'])
        reports = [validate(volume.vertices_base_m) for volume in members]
        original = [validate(corners) for corners in binding['member_vertices_base_m']]
        envelope = validate(binding['envelope_vertices_base_m'])
        if self._rgb_reference_visibility(captured) != binding:
            _error('REFERENCE_VISIBILITY_CHANGED_DURING_PROOF')
        return dict(observation_set_sha256=captured['interest']['observation_set_sha256'],
            member_count=len(members), all_members_visible=True, members=reports,
            reference_id=binding['reference_id'],
            reference_observation_set_sha256=binding['reference_observation_set_sha256'],
            reference_vertices_sha256=binding['reference_vertices_sha256'],
            reference_members=original, reference_envelope=envelope,
            all_original_reference_geometry_visible=True)

    def _rgb_validate_path(self, episode, captured, plan, planner, timing):
        from alicia_grasp_modes.rgb_noncontact_scene import build_rgb_noncontact_scene, validate_rgb_noncontact_trajectory
        self._rgb_require_owner(episode)
        capability = getattr(self, '_rgb_prefix_capability', None)
        if capability is not None and capability['episode'] is episode and not capability.get('submission_started'):
            capability['retry_stage'] = 'path_preflight'
        names = tuple(plan.joint_trajectory.joint_names)
        if set(names) != set(ARM_NAMES) or len(names) != 6:
            _error('TRAJECTORY_JOINT_NAMES_INVALID')
        if planner.manipulator.get_planning_frame() != 'base_link' or planner.manipulator.get_end_effector_link() != 'tool0':
            _error('PLANNING_FRAMES_INVALID')
        description = rospy.get_param('/robot_description', '')
        model = SerialUrdfFk(description, names)
        # Retain the exact incoming curve even when an early prefix guard
        # rejects it. Archive I/O remains solely in the RPC's outer finally.
        self._rgb_artifact_plan = deepcopy(plan)
        self._rgb_artifact_model = description
        cfg = deepcopy(rospy.get_param('/grasp_6d/remote/gripper_geometry', {}))
        gripper = GripperGeometry(float(cfg.get('max_inner_gap_m', .05)),
            float(cfg.get('width_safety_margin_per_side_m', .002)),
            np.asarray(cfg.get('finger_box_xyz_m', [.0434,.0286,.0600])),
            np.asarray(cfg.get('palm_box_xyz_m', [.1175,.1550,.0774])),
            float(cfg.get('support_clearance_m', .003)))
        mismatch = gripper_contract_mismatch_reason(gripper, .05, .05, ANALYTICAL_GRIPPER_MODEL_NAME, .05,
            tool_jaw_axis=cfg.get('tool_jaw_axis','y'), tool_finger_length_axis=cfg.get('tool_finger_length_axis','z'))
        if mismatch:
            _error(mismatch)
        opening = self._observation_measured_opening()
        desired = self._observation_controller_hold(names)
        prefix = self._rgb_prefix_curve(episode, plan, desired)
        excursion = continuous_excursion(plan, desired)
        # Reject an unusable original association view before spending the
        # continuous collision/tracking proof budget. Fresh-source proof below
        # repeats this same frozen-reference contract on the final curve.
        self._rgb_member_visibility(captured, plan, model, desired)
        constraints = deepcopy(rospy.get_param('/alicia_controller/constraints', None))
        stop = rospy.get_param('/alicia_controller/stop_trajectory_duration', None)
        source = captured['volumes'][0]
        scene = build_rgb_noncontact_scene(interest_volume=source,
            additional_interest_volumes=captured['volumes'][1:],
            depth_image=captured['depth_raw'], depth_scale_m=captured['depth_scale'],
            source_stamp_ns=captured['source_stamp_ns'], source_depth_sha256=captured['source_depth_sha256'],
            depth_uncertainty_m=captured['cfg'].support_threshold_m,
            valid_depth_range_m=(captured['cfg'].depth_min_m,captured['cfg'].depth_max_m),
            source_record_id='rgb_acquisition:'+episode['request_id'], containment_reserve_m=0.,
            camera_geometry=captured.get('source_context', {}).get('camera'))
        captured['proof_scene'] = scene
        report = qualify_contract_path(plan, desired, scene, model, gripper, opening,
            constraints, stop, path_validator=lambda p, _scene, fk, g, gap, hold, **bounds:
                self._rgb_fresh_path_audit(episode, captured, p, fk, g, gap, hold,
                    bounds['joint_error_bounds_rad'], max_checks=bounds['max_checks'],
                    max_seconds=bounds['max_seconds']))
        if not report['ok']:
            _error('SCENE_PATH_REJECTED: '+str(report))
        audit = report['continuous_path']
        visibility = audit['rgb_interest_visibility']
        captured['proof_plan_sha256'] = wire_digest(plan)
        audit.update(execution_tracking_contract=report['execution_tracking_contract'],
            command_derivative_bounds=report['command_derivative_bounds'],
            tracking_contract_search=report['tracking_contract_search'],
            controller_constraints_snapshot=constraints, gripper_geometry_snapshot=cfg,
            controller_hold_provenance=deepcopy(self._observation_sync_hold_evidence),
            rgb_interest_visibility=visibility, max_joint_excursion_rad=excursion,
            rgb_source_id=episode['source_id'], rgb_request_id=episode['request_id'],
            source_stamp_ns=captured['source_stamp_ns'], noncontact_observation_only=True)
        if prefix is not None:
            prefix['terminal_tool_transform_base'] = model(prefix['terminal_joint_positions']).tolist()
            audit['actuation_prefix'] = prefix
        # Final curve and proof are checked again immediately before send.
        self._rgb_submission(episode, captured, plan, audit, planner, timing, reserve=False)
        return audit

    def _rgb_submission(self, episode, captured, plan, audit, planner, timing, *, reserve=True):
        capability = getattr(self, '_rgb_prefix_capability', None)
        if reserve and capability is not None and capability['episode'] is episode:
            # This boundary may proceed to a transport. Consume before any
            # proof, persistence, or send can fail; never infer non-submission
            # later from a caught exception or a still-reserved ledger.
            if capability.get('submission_started'):
                _error('ACTUATION_PREFIX_ALREADY_SUBMITTED')
            capability['submission_started'] = True
            capability['retry_stage'] = ''
            episode['actuation_prefix_submission_started'] = True
            episode.pop('actuation_prefix_preflight', None)
        self._rgb_require_owner(episode)
        if capability is not None and capability['episode'] is episode:
            if (capability['phase'] != 'preparing'
                    or capability['trajectory_sha256'] != wire_digest(plan)
                    or audit.get('actuation_prefix', {}).get('kind') != 'one_time_rgb_observation_prefix'):
                _error('ACTUATION_PREFIX_FINAL_BINDING_CHANGED')
        binding = self._rgb_reference_visibility(captured)
        visibility = audit.get('rgb_interest_visibility', {})
        if (not binding['reference_id'] or binding['reference_id'] != episode.get('reference_id')
                or binding['reference_vertices_sha256'] != episode.get('reference_vertices_sha256')
                or visibility.get('all_original_reference_geometry_visible') is not True
                or any(visibility.get(key) != binding[key] for key in
                       ('reference_id', 'reference_observation_set_sha256', 'reference_vertices_sha256'))):
            _error('FINAL_REFERENCE_VISIBILITY_BINDING_CHANGED')
        if (not self._observation_timing_current(planner, timing)
                or wire_digest(plan) != audit['trajectory_sha256']
                or rospy.get_param('/alicia_controller/constraints', None) != audit['controller_constraints_snapshot']
                or rospy.get_param('/alicia_controller/stop_trajectory_duration', None) != audit['execution_tracking_contract']['stop_trajectory_duration_sec']
                or rospy.get_param('/grasp_6d/remote/gripper_geometry', {}) != audit['gripper_geometry_snapshot']
                or self._observation_measured_opening() != audit['opening_width_m']
                or hashlib.sha256(rospy.get_param('/robot_description','').encode()).hexdigest() != audit['model_sha256']):
            _error('FINAL_PROOF_INPUT_CHANGED')
        if reserve:
            names = tuple(plan.joint_trajectory.joint_names)
            model = SerialUrdfFk(rospy.get_param('/robot_description', ''), names)
            cfg = audit['gripper_geometry_snapshot']
            gripper = GripperGeometry(float(cfg.get('max_inner_gap_m', .05)),
                float(cfg.get('width_safety_margin_per_side_m', .002)),
                np.asarray(cfg.get('finger_box_xyz_m', [.0434,.0286,.0600])),
                np.asarray(cfg.get('palm_box_xyz_m', [.1175,.1550,.0774])),
                float(cfg.get('support_clearance_m', .003)))
            # Reprove this exact final curve under the SAME action contract on
            # actual latest depth, including its changed obstacle/support bounds.
            fresh_audit = self._rgb_fresh_path_audit(episode, captured, plan, model, gripper,
                audit['opening_width_m'], self._observation_controller_hold(names),
                audit['execution_tracking_contract']['joint_error_bounds_rad'], artifact_label='submission')
            audit.update(fresh_audit)
            self._rgb_artifact_frames['submission'] = dict(captured)
            self._rgb_submission(episode, captured, plan, audit, planner, timing, reserve=False)
            require_fresh_source(captured['source_stamp_ns'], rospy.Time.now().to_nsec())
            prefix_binding = ({'actuation_prefix_trajectory_sha256': capability['trajectory_sha256']}
                if capability is not None and capability['episode'] is episode else {})
            self._rgb_update_ledger(episode, 'executing', trajectory_sha256=audit['trajectory_sha256'],
                submission_source_stamp_ns=str(captured['source_stamp_ns']),
                fresh_scene_revalidation=dict(kind='complete_final_curve_reproof',
                    scene_sha256=audit['scene_sha256'], source_stamp_ns=str(captured['source_stamp_ns']),
                    certified_minimum_clearance_m=audit['certified_minimum_clearance_m'],
                    joint_error_bounds_rad=audit['execution_tracking_contract']['joint_error_bounds_rad']),
                **prefix_binding)
            if capability is not None and capability['episode'] is episode:
                # The sealed one-time binding is now durably in the ledger.
                capability['phase'] = 'submitted'

    def _rgb_fresh_path_audit(self, episode, prior, plan, model, gripper, opening, desired,
                              errors, *, max_checks=4096, max_seconds=2., artifact_label='proof'):
        from alicia_grasp_modes.rgb_noncontact_scene import (build_rgb_noncontact_scene,
            validate_rgb_noncontact_trajectory, retain_observed_obstacles)
        fresh = None
        started = stage_started = time.monotonic()
        stage = 'capture'
        timing = {}
        def finish_stage(next_stage):
            nonlocal stage, stage_started
            now = time.monotonic()
            timing[stage+'_elapsed_sec'] = now-stage_started
            timing['source_age_at_'+stage+'_sec'] = (None if fresh is None else
                (rospy.Time.now().to_nsec()-fresh['source_stamp_ns'])*1e-9)
            timing['total_elapsed_sec'] = now-started
            stage, stage_started = next_stage, now
        self._rgb_artifact_frames.pop(artifact_label,None)
        try:
            fresh = self._rgb_capture_after_plan(episode, prior)
            finish_stage('scene_build')
            # Retain the actual attempted source BEFORE proof, including a
            # rejection. Only memory is touched on the freshness boundary;
            # the RPC's outer finally writes these bounded slots to SanStand.
            self._rgb_artifact_frames[artifact_label] = dict(fresh)
            self._rgb_artifact_plan = deepcopy(plan)
            scene = build_rgb_noncontact_scene(interest_volume=fresh['volumes'][0],
                additional_interest_volumes=fresh['volumes'][1:],
                depth_image=fresh['depth_raw'], depth_scale_m=fresh['depth_scale'],
                source_stamp_ns=fresh['source_stamp_ns'], source_depth_sha256=fresh['source_depth_sha256'],
                depth_uncertainty_m=fresh['cfg'].support_threshold_m,
                valid_depth_range_m=(fresh['cfg'].depth_min_m,fresh['cfg'].depth_max_m),
                source_record_id='rgb_acquisition:'+episode['request_id']+':fresh', containment_reserve_m=0.,
                camera_geometry=fresh.get('source_context', {}).get('camera'))
            scene = retain_observed_obstacles(prior['proof_scene'], scene)
            self._rgb_artifact_frames[artifact_label]['proof_scene'] = scene
            finish_stage('proof')
            remaining = .8-(rospy.Time.now().to_nsec()-fresh['source_stamp_ns'])*1e-9-.03
            if remaining <= .001:
                _error('FRESH_SCENE_PROOF_TIME_BUDGET_EXHAUSTED')
            audit = validate_rgb_noncontact_trajectory(plan, scene, model, gripper, opening, desired,
                joint_error_bounds_rad=errors, max_checks=max_checks, max_seconds=min(max_seconds,remaining))
            finish_stage('member_visibility')
            audit['rgb_interest_visibility'] = self._rgb_member_visibility(fresh, plan, model, desired)
            finish_stage('final_binding')
            require_fresh_source(fresh['source_stamp_ns'], rospy.Time.now().to_nsec())
            self._rgb_require_owner(episode)
            finish_stage('completed')
            audit['fresh_source_timing'] = dict(timing)
            prior.update(fresh)
            prior['proof_scene'] = scene
            self._rgb_artifact_frames[artifact_label] = dict(prior)
            self._rgb_artifact_failure = None
            return audit
        except (ValueError,KeyError,TypeError,ObservationPathError) as exc:
            failed_stage = stage
            finish_stage('failed')
            self._rgb_artifact_failure = dict(stage=artifact_label,
                source_stamp_ns=None if fresh is None else str(fresh['source_stamp_ns']),
                source_depth_sha256=None if fresh is None else fresh['source_depth_sha256'],
                reason=str(exc), audit=deepcopy(getattr(exc,'audit',{})),
                failed_stage=failed_stage, fresh_source_timing=dict(timing),
                joint_error_bounds_rad=np.asarray(errors).tolist(),
                trajectory_sha256=wire_digest(plan), motion_submitted=False)
            if isinstance(exc,ObservationPathError):
                raise
            if isinstance(exc,ValueError):
                raise ObservationPathError(str(exc)) from exc
            raise
