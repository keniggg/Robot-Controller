"""Non-authorizing, source-paired support-gap identity at the remote boundary.

The perception producer owns support/RGB requalification. This consumer only
binds its typed receipt to the already observed target and original deadline;
it never turns a false object or an empty mask into planning geometry.
"""
from copy import deepcopy
import json
import math

import numpy as np
import rospy

from .tabletop import Config
from .rgb_observation_handoff import HandoffPolicy


CODE = 'TARGET_MEASUREMENT_PENDING'
REFERENCE_KEYS = ('anchor_source_stamp_ns', 'anchor_position_base', 'anchor_extent_m',
                  'reference_support_plane_base', 'reference_support_footprint_base')


def _positive_int(value):
    if type(value) is not int or value <= 0:
        raise ValueError('invalid source integer')
    return value


def reference_binding(status, selection):
    """Validate the exact producer/source envelope, not a latest-status match."""
    ns = _positive_int(status['source_stamp_ns'])
    if (status.get('mode') != 'unknown' or status.get('generation') != selection['generation']
            or status.get('selection_stamp_ns') != selection['stamp_ns']
            or ns <= selection['stamp_ns'] or status.get('target_locked') is not True
            or status.get('target_lost') is not False or status.get('first_loss') is not None
            or status.get('exact_rgbd') is not True
            or status.get('projection_frame_convention') != 'ros_camera_link'
            or any(type(status.get(key)) is not int or status[key] != ns
                   for key in ('color_stamp_ns', 'depth_stamp_ns', 'tf_lookup_stamp_ns'))
            or not status.get('color_frame_id') or not status.get('depth_frame_id')):
        raise ValueError('producer source/selection mismatch')
    anchor = _positive_int(status['anchor_source_stamp_ns'])
    if not selection['stamp_ns'] < anchor <= ns:
        raise ValueError('anchor source mismatch')
    for key, shape in (('anchor_position_base', (3,)), ('anchor_extent_m', (3,)),
                       ('reference_support_plane_base', (4,))):
        values = np.asarray(status[key], dtype=float)
        if values.shape != shape or not np.all(np.isfinite(values)):
            raise ValueError('invalid reference geometry')
    footprint = np.asarray(status['reference_support_footprint_base'], dtype=float)
    if footprint.ndim != 2 or footprint.shape[1] != 3 or len(footprint) < 3 or not np.all(np.isfinite(footprint)):
        raise ValueError('invalid reference footprint')
    return json.dumps({key: status[key] for key in REFERENCE_KEYS}, sort_keys=True, allow_nan=False)


def validate_support_receipt(status, reference, now_ns, *, recovery=False):
    """Require the existing bounded support-gap receipt and immutable anchor."""
    if reference_binding(status, reference['selection']) != reference['binding']:
        raise ValueError('reference changed')
    ns = status['source_stamp_ns']
    q = status['source_qualification']
    maximum = q['maximum_hold_seconds']
    if (isinstance(maximum, bool) or not isinstance(maximum, (int, float))
            or not math.isfinite(maximum) or maximum != reference['maximum_sec']
            or not reference['source_ns'] < ns <= now_ns < reference['deadline_ns']
            or q.get('kind') != 'support_measurement_gap'
            or q.get('reference_source_stamp_ns') != reference['source_ns']
            or q.get('source_stamp_ns') != ns or q.get('source_qualified') is not True
            or q.get('metric_geometry_authority') is not False
            or q.get('motion_authorized') is not False or q.get('reference_refreshed') is not False
            or not isinstance(q.get('source_id'), str) or len(q['source_id']) != 64
            or any(c not in '0123456789abcdef' for c in q['source_id'])
            or (reference.get('source_id') is not None and q['source_id'] != reference['source_id'])):
        raise ValueError('invalid support-gap qualification')
    first = _positive_int(q['first_unqualified_source_stamp_ns'])
    last = _positive_int(q['last_unqualified_source_stamp_ns'])
    if not reference['source_ns'] < first <= last <= ns:
        raise ValueError('invalid gap source order')
    if recovery:
        identity = q['rgb_identity']
        rows = identity['candidates']
        if (status.get('state') != 'ready' or q.get('state') != 'recovered_unique_measured_frame'
                or q.get('measurement_republished') is not True or q.get('identity_verified') is not True
                or q.get('support_measurement_available') is not True or last >= ns
                or identity.get('evidence_type') != 'metric_seeded_full_outline_rgb_identity'
                or identity.get('candidate_count') != 1 or identity.get('all_metric_candidates_refined') is not True
                or not isinstance(rows, list) or len(rows) != 1 or rows[0].get('identity_compatible') is not True):
            raise ValueError('missing committed measured recovery')
        audit, policy = q['support_audit'], reference['support_policy']
        for key, maximum in (('support_reference_angle_deg', policy['support_max_normal_change_deg']),
                             ('support_reference_max_distance_m', policy['support_max_local_shift_m']),
                             ('support_current_audit_residual_m', policy['support_threshold_m'])):
            value = audit[key]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0. <= value <= maximum:
                raise ValueError('recovery support audit invalid')
        ratio = audit['support_current_audit_inlier_ratio']
        if (isinstance(ratio, bool) or not isinstance(ratio, (int, float)) or not math.isfinite(ratio)
                or not policy['support_min_ratio'] <= ratio <= 1.
                or type(audit['support_current_audit_quadrants']) is not int
                or audit['support_current_audit_quadrants'] not in (3, 4)
                or audit['support_reference_footprint_vertices'] != reference['footprint_vertices']):
            raise ValueError('recovery support coverage invalid')
        rgb_policy = HandoffPolicy()
        if (identity.get('full_outline_iou_minimum') != .85
                or identity.get('max_colour_distance') != rgb_policy.max_colour_distance
                or identity.get('max_contour_distance') != rgb_policy.max_contour_distance
                or any(identity.get(key) is not False for key in (
                    'metric_geometry_authority', 'motion_authorized', 'candidate_selected', 'geometry_merged'))
                or rows[0].get('candidate_index') != 0):
            raise ValueError('recovery RGB contract invalid')
        for key, lower, upper in (('mask_iou', .85, 1.),
                ('colour_distance', 0., rgb_policy.max_colour_distance),
                ('contour_distance', 0., rgb_policy.max_contour_distance)):
            value = rows[0][key]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not lower <= value <= upper:
                raise ValueError('recovery RGB evidence invalid')
    elif (status.get('state') != 'unavailable'
            or q.get('state') not in ('pending_current_support', 'pending_unique_measured_geometry')
            or q.get('identity_verified') is not False or last != ns
            or q.get('support_measurement_available') is not (q['state'] == 'pending_unique_measured_geometry')):
        raise ValueError('not a typed non-authorizing support gap')
    return q['source_id']


class UnknownPendingIdentityMixin:
    def _initialize_unknown_pending_identity(self):
        self._unknown_identity_reference = None
        self._unknown_identity_pending = None
        self._unknown_identity_retired_selection = None
        self._unknown_identity_objects = {}
        self._unknown_identity_status = {}
        self._unknown_identity_masks = {}
        self._unknown_identity_cancelled_through_ns = 0
        self._unknown_identity_publication_started = False

    def _unknown_identity_scope(self):
        # Caller checks the authoritative selection once at callback admission;
        # repeated ROS RPCs inside the same locked join consume the fixed budget.
        return bool(self._mode_ready
            and (self._mode_selection or {}).get('mode') == 'unknown'
            and self._observe_return_selected() and self.streaming_enabled
            and self._observe_return_phase in ('initial_contact', 'observation_prefix')
            and not self._observe_return_paused
            and not self._unknown_identity_publication_started
            and not any(getattr(self, key, None) is not None for key in (
                '_observe_return_prefix', '_observe_return_origin_prefix', '_observe_return_session',
                '_observe_return_pending_supplement', '_execution_authority_ticket'))
            and not getattr(self, 'robot_execution_active', False)
            and not getattr(getattr(self, 'execution_plan_controller', None), 'has_execution', False)
            and not getattr(getattr(self, 'latest_preview_rich_plan', None), 'valid', False)
            and not getattr(getattr(self, 'latest_rich_plan', None), 'valid', False))

    @staticmethod
    def _unknown_identity_cache(cache, ns, value):
        cache[ns] = value
        while len(cache) > 64:
            del cache[min(cache)]

    def _unknown_identity_retire(self, reason):
        """Called with stream->geometry held; retirement cannot later recover."""
        self._unknown_identity_pending = None
        self._unknown_identity_retired_selection = deepcopy(self._mode_selection)
        self._unknown_identity_reference = None
        self._unknown_identity_objects.clear()
        self._unknown_identity_status.clear()
        self._unknown_identity_masks.clear()
        with self._object_lock:
            absent = deepcopy(self.latest_object)
            if absent is not None:
                absent.detected = False
                self.latest_object = absent
                self.frames.update_object(absent, absent.header.stamp,
                    target_epoch=self.target_instance_epoch,
                    target_identity=self._current_stream_target_identity())
        self._advance_target_instance_epoch('TARGET_LOST')
        self._invalidate_geometry('TARGET_LOST', reason)

    def _unknown_identity_tick_locked(self):
        pending = self._unknown_identity_pending
        if pending is not None and (not self._unknown_identity_scope()
                or pending['identity'] != self._current_stream_target_identity()
                or self._ros_source_clock_ns() >= pending['deadline_ns']):
            self._unknown_identity_retire('support measurement quarantine expired or context changed')
        return self._unknown_identity_pending is not None

    def _unknown_identity_poll(self):
        with self._mode_control_lock, self._stream_condition, self._geometry_state_guard():
            if self._unknown_identity_pending is not None and not self._selection_is_current():
                self._unknown_identity_retire('selection changed during support quarantine')
            return self._unknown_identity_tick_locked()

    def _unknown_identity_ticket_code(self, ticket):
        if self._unknown_identity_retired_selection == self._mode_selection:
            return 'TARGET_LOST'
        if self._unknown_identity_pending is not None:
            return CODE
        cutoff = self._unknown_identity_cancelled_through_ns
        if cutoff:
            try:
                ns = int(ticket.payload[0].stamp_ns)
            except (AttributeError, TypeError, IndexError):
                ns = round(float(getattr(ticket, 'snapshot_stamp_sec', 0.)) * 1e9)
            if ns <= cutoff:
                return CODE
        return ''

    def _unknown_identity_quarantine(self, msg, reference):
        ns = msg.header.stamp.to_nsec()
        if self._unknown_identity_pending is None:
            self._unknown_identity_pending = deepcopy(reference)
            self._unknown_identity_pending['unpaired'] = set()
            self._unknown_identity_pending['source_id'] = None
        pending = self._unknown_identity_pending
        pending['unpaired'].add(ns)
        self._unknown_identity_cancelled_through_ns = max(
            self._unknown_identity_cancelled_through_ns, self.last_submitted_stamp_ns, ns)
        # Clear only queued work; the current worker drains through the source
        # cancellation barrier. Keep identity, phase, original budget and the
        # original non-authorizing view evidence, with their old labels intact.
        request_id = self._pending_request_id
        self.inference_coordinator.reset_target_epoch(self.target_instance_epoch)
        self._pending_request_id = None
        if request_id is not None:
            claimed = self._claim_terminal_request_locked(request_id, CODE)
            self._emit_pending_drop_metrics(request_id, CODE, terminal_claimed=claimed)
        with self._object_lock:
            self.latest_object, self.latest_object_time = msg, rospy.Time.now()
        self.frames.update_object(msg, msg.header.stamp, target_epoch=self.target_instance_epoch,
                                  target_identity=self._current_stream_target_identity())
        self._invalidate_geometry(CODE, 'current measured support/geometry unavailable', stamp=msg.header.stamp)
        self._near_field_request_plan_cache = {}
        self._candidate_plan_metrics = {}
        # The cancelled initial-contact ticket consumed the single-snapshot
        # slot. Fresh admission still waits for recovery and worker-idle, and
        # retains the same absolute contact deadline.
        self._direct_near_field_submission_generation = None
        self.latest_preview_rich_plan = None
        self._stream_condition.notify_all()

    def _unknown_identity_join_locked(self):
        if self._unknown_identity_pending is None and (not self._unknown_identity_scope()
                or self._unknown_identity_retired_selection == self._mode_selection):
            return
        now = int(self._ros_source_clock_ns())
        if self._unknown_identity_tick_locked():
            pending = self._unknown_identity_pending
            try:
                for ns in sorted(tuple(pending['unpaired'])):
                    status = self._unknown_identity_status.get(ns)
                    if status is None:
                        continue
                    pending['source_id'] = validate_support_receipt(status, pending, now)
                    pending['unpaired'].remove(ns)
                if pending['unpaired'] or pending['source_id'] is None:
                    return
                for ns in sorted(self._unknown_identity_objects):
                    obj = self._unknown_identity_objects[ns]
                    status = self._unknown_identity_status.get(ns)
                    mask = self._unknown_identity_masks.get(ns)
                    if ns <= self._unknown_identity_cancelled_through_ns or not obj.detected or status is None or mask is None:
                        continue
                    validate_support_receipt(status, pending, now, recovery=True)
                    if not mask[0] or mask[1] != status['color_frame_id']:
                        raise ValueError('new measured mask missing')
                    if self._ros_source_clock_ns() >= pending['deadline_ns']:
                        raise ValueError('recovery receipt expired before commit')
                    super().object_cb(obj)
                    if (not getattr(self.latest_object, 'detected', False)
                            or self.latest_object.header.stamp.to_nsec() != ns):
                        raise ValueError('recovery object rejected by original gate')
                    if self._ros_source_clock_ns() >= pending['deadline_ns']:
                        raise ValueError('recovery receipt expired during commit')
                    self._unknown_identity_pending = None
                    # All pre-recovery snapshots remain cancelled, including a
                    # collector that waited through the entire gap.
                    self._unknown_identity_cancelled_through_ns = max(
                        self._unknown_identity_cancelled_through_ns, ns - 1)
                    break
            except (ValueError, KeyError, TypeError, OverflowError) as exc:
                self._unknown_identity_retire('support-gap receipt rejected: ' + str(exc))
                return
        if (self._unknown_identity_pending is not None
                or self._unknown_identity_retired_selection == self._mode_selection
                or not self._unknown_identity_scope()):
            return
        obj = self.latest_object
        if obj is None or not obj.detected:
            return
        ns = obj.header.stamp.to_nsec()
        if (self._unknown_identity_reference is not None
                and self._unknown_identity_reference['source_ns'] == ns
                and self._unknown_identity_reference['identity'] == self._current_stream_target_identity()):
            return
        status, mask = self._unknown_identity_status.get(ns), self._unknown_identity_masks.get(ns)
        if status is None or mask is None or not mask[0] or status.get('state') != 'ready':
            return
        try:
            binding = reference_binding(status, self._mode_selection)
            raw = rospy.get_param('/unknown_perception', {})
            maximum = min(float(raw.get('max_frame_age_sec', .8)), .8)
            defaults = Config()
            policy = {key: float(raw.get(key, getattr(defaults, key))) for key in (
                'support_max_normal_change_deg', 'support_max_local_shift_m', 'support_threshold_m', 'support_min_ratio')}
            if not all(math.isfinite(value) and value > 0 for value in policy.values()):
                return
            if not math.isfinite(maximum) or maximum <= 0 or not ns <= now < ns + round(maximum * 1e9):
                return
            if mask[1] != status['color_frame_id']:
                return
            self._unknown_identity_reference = dict(source_ns=ns, binding=binding,
                identity=self._current_stream_target_identity(), selection=deepcopy(self._mode_selection),
                maximum_sec=maximum, deadline_ns=ns+round(maximum*1e9), support_policy=policy,
                footprint_vertices=len(status['reference_support_footprint_base']))
        except (ValueError, KeyError, TypeError, OverflowError):
            return

    def _unknown_identity_status_cb(self, status):
        with self._mode_control_lock, self._stream_condition, self._geometry_state_guard():
            if (status.get('mode') != 'unknown' or status.get('generation') != (self._mode_selection or {}).get('generation')
                    or status.get('selection_stamp_ns') != (self._mode_selection or {}).get('stamp_ns')):
                return
            if not self._selection_is_current():
                if self._unknown_identity_pending is not None:
                    self._unknown_identity_retire('selection changed during support quarantine')
                return
            if self._unknown_identity_retired_selection == self._mode_selection:
                return
            if self._unknown_identity_pending is not None and status.get('target_lost') is True:
                self._unknown_identity_retire('producer retired target')
                return
            ns = status.get('source_stamp_ns')
            if type(ns) is not int or ns <= 0:
                return
            self._unknown_identity_cache(self._unknown_identity_status, ns, deepcopy(status))
            self._unknown_identity_join_locked()

    def object_cb(self, msg):
        if (not hasattr(self, '_stream_condition') or not self.streaming_enabled
                or (self._mode_selection or {}).get('mode') != 'unknown' or not self._observe_return_selected()):
            return super().object_cb(msg)
        with self._stream_condition, self._geometry_state_guard():
            if not self._selection_is_current():
                if self._unknown_identity_pending is not None:
                    self._unknown_identity_retire('selection changed during support quarantine')
                return super().object_cb(msg)
            self._unknown_identity_tick_locked()
            if self._unknown_identity_retired_selection == self._mode_selection:
                # Expiry is irreversible within the producer's selection. A
                # late, formerly valid recovery is not a newly acquired target.
                absent = deepcopy(msg)
                absent.detected = False
                super().object_cb(absent)
                return
            ns = msg.header.stamp.to_nsec()
            self._unknown_identity_cache(self._unknown_identity_objects, ns, deepcopy(msg))
            reference = self._unknown_identity_reference
            if self._unknown_identity_pending is not None:
                if not msg.detected:
                    self._unknown_identity_quarantine(msg, reference)
                self._unknown_identity_join_locked()
                return
            if (not msg.detected and self._unknown_identity_scope() and reference is not None
                    and reference['identity'] == self._current_stream_target_identity()
                    and reference['selection'] == self._mode_selection
                    and reference['source_ns'] < ns <= self._ros_source_clock_ns() < reference['deadline_ns']):
                self._unknown_identity_quarantine(msg, reference)
                self._unknown_identity_join_locked()
                return
            super().object_cb(msg)
            self._unknown_identity_join_locked()

    def mask_cb(self, msg):
        super().mask_cb(msg)
        if (not hasattr(self, '_stream_condition') or not self.streaming_enabled
                or (self._mode_selection or {}).get('mode') != 'unknown' or not self._observe_return_selected()):
            return
        with self._stream_condition, self._geometry_state_guard():
            if not self._selection_is_current():
                if self._unknown_identity_pending is not None:
                    self._unknown_identity_retire('selection changed during support quarantine')
                return
            mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            self._unknown_identity_cache(self._unknown_identity_masks, msg.header.stamp.to_nsec(),
                (bool(mask.ndim == 2 and np.any(mask)), msg.header.frame_id))
            self._unknown_identity_join_locked()
