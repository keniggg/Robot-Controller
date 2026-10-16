#!/usr/bin/env python3
"""Add a mode boundary around the unchanged production Grasp6D node.

The router pauses perception and inference before committing selection. Its
reset service is a completion barrier: no old inference can write caches or
publish a plan after the service returns. This node never commands the robot.
"""

import importlib.util
import hashlib
import json
from dataclasses import asdict, fields, replace
from copy import copy, deepcopy
from alicia_flexible_grasp.grasp.gripper_close_contract import bind_simulated_fixed_close
from alicia_flexible_grasp.vision.rgbd_snapshot import retained_exact_source
import math
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

import rospkg
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from alicia_grasp_modes.selection import (SELECTION_PARAM, parse_selection,
                                        read_authoritative_selection)
from alicia_grasp_modes.target_mass import mass_config_from_ros
from alicia_grasp_modes.observation_policy import (
    observation_config, observation_distance_preferences, WORKING_VOLUME_POLICY)
from alicia_flexible_grasp.grasp.observation_working_volume import (
    observation_obb_corners, validate_planned_observation_working_volume)
from alicia_grasp_modes.observation_view_utility import (
    POLICY as INFORMATION_MOTION_POLICY, predicted_information_utility, information_motion_rank)
from alicia_grasp_modes.observation_support_schedule import support_schedule_priority
from alicia_grasp_modes.silhouette_observation_order import ObservationCheckSchedule
from alicia_grasp_modes.observe_return_remote import ObserveReturnRemoteMixin
from alicia_grasp_modes.unknown_pending_identity import UnknownPendingIdentityMixin
from alicia_grasp_modes.observe_return_measurement_remote import ObserveReturnMeasurementRemoteMixin
from alicia_grasp_modes.contact_station_remote import ContactStationRemoteMixin
from alicia_grasp_modes.station_compute_wait_remote import StationComputeWaitRemoteMixin
from alicia_grasp_modes.station_contact_admission import KEY as STATION_CONTACT_COMPUTATION
from alicia_grasp_modes.initial_preflight_remote import InitialPreflightRemoteMixin
from alicia_grasp_modes.silhouette_observation_planning import SilhouetteObservationPlanningMixin
from alicia_grasp_modes.silhouette_observation import directed_silhouette_rotations
from alicia_grasp_modes.silhouette_snapshot_diagnostic import (
    SilhouetteSnapshotConfig, build_silhouette_snapshot_evidence)
from alicia_grasp_modes.tabletop import Config as UnknownPerceptionConfig
from alicia_grasp_modes.rgb_observation_source import camera_geometry_snapshot
from alicia_flexible_grasp.vision.rgbd_rectification import require_rectified_source_after_activation
from alicia_grasp_modes.silhouette_observation_geometry import conservative_silhouette_observation_geometry
from alicia_grasp_modes.observe_return_visual import (directed_missing_side_rotations,
    distance_bound_missing_side_rotation,
    bounded_existing_camera_rolls, diversified_observation_checks)


def _load_original():
    name = '_alicia_original_remote_grasp6d'
    if name in sys.modules:
        return sys.modules[name]
    root = Path(rospkg.RosPack().get_path('alicia_flexible_grasp_supervisor'))
    spec = importlib.util.spec_from_file_location(
        name, str(root / 'scripts' / 'remote_grasp6d_node.py'))
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


original = _load_original()


class ModeAwareRemoteGrasp6DNode(ContactStationRemoteMixin, InitialPreflightRemoteMixin, StationComputeWaitRemoteMixin, ObserveReturnMeasurementRemoteMixin, UnknownPendingIdentityMixin, SilhouetteObservationPlanningMixin, ObserveReturnRemoteMixin, original.RemoteGrasp6DNode):
    _observe_return_original = original
    def __init__(self):
        self._initialize_mode_boundary()
        super().__init__()
        self._setup_observe_return_io()
        self._setup_observation_measurement_remote_io()
        self._setup_station_compute_io()
        self._setup_contact_station_io()
        self._setup_initial_preflight_remote_io()
        self._silhouette_status_sub = rospy.Subscriber(
            '/perception/unknown/detector_status', String,
            self._silhouette_source_status_cb, queue_size=64)
        self._mode_reset_service = rospy.Service(
            '/grasp_mode/reset_remote', Trigger, self.reset_mode_cb)
        self._mode_subscriber = rospy.Subscriber(
            SELECTION_PARAM, String, self.selection_cb, queue_size=1)

    def _initialize_mode_boundary(self):
        self._mode_control_lock = threading.RLock()
        self._mode_reset_lock = threading.RLock()
        self._mode_work_lock = threading.RLock()
        self._mode_phase_lock = threading.RLock()
        self._silhouette_source_lock = threading.RLock()
        self._silhouette_source_status = {}
        self._mode_selection = None
        self._mode_ready = False
        self._reset_observe_return()
        self._initialize_unknown_pending_identity()
        self._reset_station_compute_wait()
        self._reset_initial_preflight_remote()

    def _read_selection(self):
        return parse_selection(rospy.get_param(SELECTION_PARAM, None))

    def _read_reset_selection(self):
        return read_authoritative_selection(rospy.get_master())

    def _selection_is_current(self, authoritative=False):
        if not self._mode_ready or self._mode_selection is None:
            return False
        try:
            selected = self._read_reset_selection() if authoritative else self._read_selection()
            return selected == self._mode_selection
        except Exception:
            return False

    def selection_cb(self, msg):
        """The atomic parameter is authoritative over delayed latched messages."""
        try:
            selected = parse_selection(msg.data)
            if selected != self._read_reset_selection():
                return
            with self._mode_reset_lock:
                if self._mode_ready and selected == self._mode_selection:
                    return
                self._reset_mode(selected)
        except Exception as exc:
            rospy.logerr('Grasp mode selection unavailable: %s', exc)

    def reset_mode_cb(self, _request):
        try:
            with self._mode_reset_lock:
                selected = self._read_reset_selection()
                self._reset_mode(selected)
                response = dict(selected, ready=True)
            return TriggerResponse(True, json.dumps(response, sort_keys=True))
        except Exception as exc:
            return TriggerResponse(False, str(exc))

    def _reset_mode(self, selected):
        """Caller owns reset mutex; release input lock before draining work."""
        with self._mode_control_lock:
            if bool(getattr(self, 'robot_execution_active', False)):
                raise RuntimeError('cannot change grasp mode during execution')
            if self._read_reset_selection() != selected:
                raise RuntimeError('mode selection changed before remote reset')
            previous = self._mode_selection
            if previous is not None:
                if selected['generation'] < previous['generation']:
                    raise ValueError('mode generation cannot move backwards')
                if selected['generation'] == previous['generation']:
                    if selected != previous:
                        raise ValueError('mode selection changed without a new generation')
                    if self._mode_ready:
                        return
                elif selected['stamp_ns'] <= previous['stamp_ns']:
                    raise ValueError('new mode generation needs a newer source cutoff')
            self._mode_ready = False
            # Inference control only; this method has no robot/torque side effects.
            super().stop_streaming()
            self._advance_target_instance_epoch('GRASP_MODE_CHANGED', preserve_identity=False)
        # Remote inference and candidate acceptance may be in progress. Their
        # ticket checks now cancel, and this lock drains remaining cache writes
        # and ROS publications before a fresh generation becomes admissible.
        with self._mode_work_lock, self._mode_phase_lock:
            with self._execution_publication_lock:
                if bool(getattr(self, 'robot_execution_active', False)):
                    raise RuntimeError('execution became active while resetting mode')
                if self._read_reset_selection() != selected:
                    raise RuntimeError('mode selection changed during remote reset')
                self._mode_selection = dict(selected)
                self._reset_observe_return()
                self._initialize_unknown_pending_identity()
                self._reset_station_compute_wait()
                self._reset_initial_preflight_remote()
                with self._silhouette_source_lock:
                    self._silhouette_source_status.clear()
                if not hasattr(self, '_carton_internal_hole_area_px'):
                    self._carton_internal_hole_area_px = getattr(self, 'mask_internal_hole_max_area_px', 25)
                # A colour silhouette can contain missing-depth pixels. For
                # unknown objects these must remain missing, never become
                # interpolated contact geometry. Restore carton unchanged.
                self.mask_internal_hole_max_area_px = (
                    0 if selected['mode'] == 'unknown' else self._carton_internal_hole_area_px)
                # Two-stage execution needs fresh visual refinement at pregrasp.
                # Direct execution already freezes its contact plan before any
                # motion and disables that refinement in execution_config.
                # Save/restore the carton policy at the mode boundary.
                if not hasattr(self, '_carton_visibility_settings'):
                    defaults = dict(camera_visibility_gate_enabled=False,
                                    camera_visibility_diagnostic_enabled=True,
                                    camera_visibility_min_depth_m=0.070,
                                    camera_visibility_max_depth_m=0.500,
                                    candidate_max_joint_delta_rad=0.0)
                    self._carton_visibility_settings = {
                        key: getattr(self, key, value) for key, value in defaults.items()}
                for key, value in self._carton_visibility_settings.items():
                    setattr(self, key, value)
                if selected['mode'] == 'unknown':
                    self.camera_visibility_gate_enabled = not self._direct_strategy_selected()
                    self.camera_visibility_diagnostic_enabled = True
                    self.camera_visibility_min_depth_m = max(
                        0.070, self.camera_visibility_min_depth_m)
                    self.camera_visibility_max_depth_m = min(
                        0.500, self.camera_visibility_max_depth_m)
                    self._use_unknown_motion_ranking()
                self.frames = original.SynchronizedRgbdBuffer(
                    source_clock_ns=self._ros_source_clock_ns)
                self.frames.configure_retention(
                    self.planning_snapshot_max_age_sec,
                    self.planning_snapshot_max_inference_latency_sec)
                joints = getattr(self, 'latest_joint_state', None)
                if joints is not None:
                    super().joint_cb(joints)
                with self._object_lock:
                    self.latest_object = None
                    self.latest_object_time = None
                self.last_submitted_stamp_ns = selected['stamp_ns']
                self._planning_snapshot_active = False
                self._planning_object_msg = None
                self._planning_object_time = None
                self._current_prepared_prediction = None
                self._target_cloud_request_active = False
                self._latest_target_observation = None
                self._near_field_request_plan_cache = {}
                self._candidate_plan_metrics = {}
                self._active_gate_audit_report = None
                self._latest_gate_audit_reference = {}
                self._latest_gate_audit_summary = {}
                self._active_graspnet_input_audit = {}
                self._latest_streaming_audit = {}
                self._latest_preview_promotion = None
                self.near_field_planning_active = False
                self._near_field_phase_id = 0
                self._near_field_planning_stage = 'contact_replan'
                self._near_field_refinement_binding = None
                self._near_field_phase_started_sec = 0.0
                self._near_field_phase_deadline_sec = 0.0
                self._direct_near_field_submission_generation = None
                self._reset_multiview_surface()
                if selected['mode'] == 'unknown':
                    self._last_model_choice = 'unknown_tabletop'
                else:
                    pcfg = rospy.get_param('/perception', {})
                    self._last_model_choice = str(pcfg.get('yolo_model_choice', 'original'))
                self._invalidate_geometry(
                    'GRASP_MODE_CHANGED',
                    'waiting for fresh %s target, generation %d' % (
                        selected['mode'], selected['generation']))
                invalid = original.Grasp6DPlan()
                invalid.header.frame_id = 'base_link'
                invalid.header.stamp = self._safe_time()
                invalid.valid = False
                invalid.diagnostic = 'GRASP_MODE_CHANGED: previous plans retired'
                legacy = original.PoseArray()
                legacy.header = invalid.header
                # The base invalidation deliberately logs transport errors.
                # Here a failed tombstone must fail the completion barrier.
                self.rich_plan_pub.publish(invalid)
                self.plan_pub.publish(legacy)
                original.RemoteGrasp6DNode._publish_preview_plan(self, invalid, legacy)
                self.latest_preview_rich_plan = None
                self._latest_preview_proposal = None
                self._latest_preview_gate_audit_report = None
                with self._mode_control_lock:
                    if self._read_reset_selection() != selected:
                        raise RuntimeError('mode selection changed during remote reset')
                    self._mode_ready = True

    def _admit_frame(self, msg):
        if not self._mode_ready or self._mode_selection is None:
            return False
        stamp = getattr(getattr(msg, 'header', None), 'stamp', None)
        stamp_ns = original._stamp_to_nsec(stamp)
        return stamp_ns > self._mode_selection['stamp_ns']

    def color_cb(self, msg):
        with self._mode_control_lock:
            if self._admit_frame(msg):
                return super().color_cb(msg)

    def depth_cb(self, msg):
        with self._mode_control_lock:
            if self._admit_frame(msg):
                return super().depth_cb(msg)

    def mask_cb(self, msg):
        with self._mode_control_lock:
            if self._admit_frame(msg):
                return super().mask_cb(msg)

    def object_cb(self, msg):
        with self._mode_control_lock:
            if self._admit_frame(msg):
                if bool(getattr(msg, 'detected', False)):
                    if self._mode_selection['mode'] == 'unknown':
                        expected_label = 'unknown_small'
                    else:
                        expected_label = str(rospy.get_param('/perception', {}).get(
                            'object_label', 'carton'))
                    if str(getattr(msg, 'label', '')) != expected_label:
                        return
                return super().object_cb(msg)

    def near_field_state_cb(self, msg):
        # Serialize state ingress with reached-view commit, then release the
        # short locks before reconstruction waits for camera callbacks.
        with self._stream_condition, self._geometry_state_guard():
            self._station_compute_phase = deepcopy(msg)
        # Reached-view reconstruction may wait for camera callbacks. Do not
        # hold their control lock while waiting for their input.
        if self._observe_return_selected():
            return self._observe_return_near_field_cb(msg)
        with self._mode_phase_lock:
            # Direct planning owns a contact-only phase at the current view.
            # The task's normal entry/exit "far field" event must not convert
            # it into an observation request or manufacture reached evidence.
            if self._direct_strategy_selected():
                return
            if self._admit_frame(msg):
                return super().near_field_state_cb(msg)

    def _direct_strategy_selected(self):
        strategy = (self._mode_selection or {}).get('strategy', 'two_stage')
        return strategy == 'direct' or (strategy == 'observe_return' and
            self._observe_return_phase in ('initial_contact', 'observing_contact', 'returned_contact'))

    def grasp_state_cb(self, msg):
        with self._stream_condition, self._geometry_state_guard():
            self._station_compute_task_state = str(getattr(msg, 'state', ''))
            previous_active = bool(getattr(self, 'robot_execution_active', False))
            super().grasp_state_cb(msg)
        if (previous_active and not bool(getattr(msg, 'active', False))
                and (self._direct_strategy_selected() or self._observe_return_selected())):
            self.stop_streaming()

    def _initial_preflight_manual_cb(self, msg):
        if msg.data is True:
            # Manual takeover cannot retain a pending historical registration.
            # Keep its seen key: another delivery must not renew the attempt.
            with self._stream_condition, self._geometry_state_guard():
                self._observe_return_reached_retry = None
        # The original callback takes mode control; never call it while
        # holding the commit locks in the opposite order.
        return super()._initial_preflight_manual_cb(msg)

    def _configured_near_field_strategy(self):
        if self._direct_strategy_selected():
            return 'single_snapshot_direct'
        return super()._configured_near_field_strategy()

    def _tabletop_placement_gripper(self, contact_execution_phase=None):
        # A boundary goal can fall below the physical clearance after wire/FK
        # roundoff. Reserve 0.5 mm before any candidate is frozen, then still
        # run the original contact, collision, IK and simulation gates.
        contact = (getattr(self, 'near_field_planning_active', False)
                   if contact_execution_phase is None else contact_execution_phase)
        geometry = super()._tabletop_placement_gripper(contact)
        if (self._mode_selection or {}).get('mode') == 'unknown' and contact:
            return replace(geometry, support_clearance_m=geometry.support_clearance_m + .0005)
        return geometry

    def _generate_tabletop_candidates(self, geometry, snapshot=None,
                                     contact_execution_phase=True, target_label=''):
        if ((self._mode_selection or {}).get('mode') != 'unknown'
                or contact_execution_phase):
            # Refine only after a qualified supplementary view. Multiple
            # source stamps alone can also be same-station acquisitions;
            # initial/direct contacts keep their established yaw set.
            selection = self._mode_selection or {}
            receipt = getattr(self, '_measurement_remote_request', None) or {}
            refine_neighbors = bool(
                selection.get('mode') == 'unknown'
                and selection.get('strategy') == 'observe_return'
                and getattr(self, '_observe_return_phase', '') == 'observing_contact'
                and getattr(self, 'near_field_planning_active', False)
                and receipt.get('state') == 'qualified')
            if refine_neighbors:
                return super()._generate_tabletop_candidates(
                    geometry, snapshot, contact_execution_phase, target_label,
                    refine_obb_neighbor_angles=True)
            return super()._generate_tabletop_candidates(
                geometry, snapshot, contact_execution_phase, target_label)
        # Observation poses are independent of object yaw. Thirty-two contact
        # seeds redundantly materialized the same camera-view lattice and
        # often expired the input before inference. Bound only the far-field
        # seeds; preserve every original view/physical check and the complete
        # contact-phase budget. Use a local copy so live config never changes.
        staged = copy(self)
        staged.tabletop_geometry_config = replace(self.tabletop_geometry_config,
            max_candidates=min(8, self.tabletop_geometry_config.max_candidates))
        candidates, audit = original.RemoteGrasp6DNode._generate_tabletop_candidates(
            staged, geometry, snapshot, contact_execution_phase, target_label)
        audit = dict(audit, unknown_observation_seed_limit=staged.tabletop_geometry_config.max_candidates,
                     contact_phase_seed_limit=self.tabletop_geometry_config.max_candidates)
        return candidates, audit

    def _tabletop_candidate_contact_overlap_m(self, proposal, candidate, support_normal,
            contact_height_bounds_m=None, contact_height_axis_base_override=None):
        if ((self._mode_selection or {}).get('mode') != 'unknown'
                or not getattr(self, 'near_field_planning_active', False)):
            return super()._tabletop_candidate_contact_overlap_m(
                proposal, candidate, support_normal, contact_height_bounds_m,
                contact_height_axis_base_override)
        from alicia_grasp_modes.contact_intervals import finger_contact_patch_overlap_m
        lower, upper = (self._proposal_contact_height_bounds(proposal)
                        if contact_height_bounds_m is None else
                        tuple(float(value) for value in contact_height_bounds_m))
        transform = original.np.asarray(candidate.T_base_tool0, dtype=float)
        if contact_height_axis_base_override is None:
            axis = original.contact_height_axis_base(
                transform[:3, :3].dot(original.parse_tool_axis(self.gripper_tool_jaw_axis)[0]),
                support_normal)
        else:
            axis = original.np.asarray(contact_height_axis_base_override, dtype=float).reshape(3)
        return finger_contact_patch_overlap_m(candidate.contact_center_base,
            transform[:3, 3], transform[:3, :3], axis, lower, upper)

    def _contact_boundary_tilts(self, proposal, support_point, support_normal,
                                probe_variants, maximum_tilt_deg, required_overlap_m,
                                contact_height_bounds_m=None, surface_snapshot=None):
        probe = None
        overlap_probe = None
        measured_boundary = ((self._mode_selection or {}).get('mode') == 'unknown'
                             and getattr(self, 'near_field_planning_active', False))
        if measured_boundary:
            from alicia_grasp_modes.contact_probe import contact_tilt_probe
            probe = contact_tilt_probe(proposal, support_point, support_normal,
                self._tabletop_placement_gripper(), self.gripper_tool_jaw_axis,
                self.gripper_tool_finger_length_axis)
            def overlap_probe(item):
                if surface_snapshot is None:
                    return -float('inf')
                _, evidence, bounds, axis = self._current_bilateral_surface_measurement(
                    item.contact_center_base,
                    original.np.asarray(item.T_base_tool0, dtype=float)[:3, :3],
                    surface_snapshot=surface_snapshot)
                if evidence is None or not evidence.ok or bounds is None:
                    return -float('inf')
                return self._tabletop_candidate_contact_overlap_m(
                    proposal, item, support_normal, contact_height_bounds_m=bounds,
                    contact_height_axis_base_override=axis)
        return super()._contact_boundary_tilts(proposal, support_point, support_normal,
            probe_variants, maximum_tilt_deg, required_overlap_m,
            contact_height_bounds_m=contact_height_bounds_m, contact_probe=probe,
            surface_snapshot=surface_snapshot, contact_overlap_probe=overlap_probe,
            include_lower_boundary=measured_boundary)

    def _tabletop_stratum_priority(self, candidate, proposal_index):
        priority = super()._tabletop_stratum_priority(candidate, proposal_index)
        if ((self._mode_selection or {}).get('mode') == 'unknown'
                and self._direct_strategy_selected()
                and getattr(self, 'near_field_planning_active', False)
                and float(candidate.audit.get('approach_tilt_deg', 0.)) > 1e-9):
            # Retain wrist/yaw diversity while exposing the newly measured
            # smaller feasible tilts within the existing 32-candidate budget.
            return priority[:2] + (abs(priority[2]),)
        return priority

    def _use_unknown_motion_ranking(self):
        # Operator preference: compare real motion cost instead of imposing
        # an extra 90-degree turn veto. URDF limits/collision checks remain.
        self.candidate_max_joint_delta_rad = 0.0

    def _visible_unknown_pregrasp(self, sequence, geometry):
        """Bounded table-parallel view offset; all contact poses stay exact.

        This is proposal construction only. The caller still checks the full
        CAD sweep, strict MoveIt sequence and contact simulation before motion.
        """
        if ((self._mode_selection or {}).get('mode') != 'unknown'
                or self._direct_strategy_selected()
                or not getattr(self, 'near_field_planning_active', False)):
            return sequence
        # Capture these once, rather than querying ROS/TF for every angle.
        probe = copy(self)
        transform = self._tool_from_camera_matrix()
        intrinsics = self._camera_intrinsics()
        probe._tool_from_camera_matrix = lambda: transform
        probe._camera_intrinsics = lambda: intrinsics
        target = geometry.center_base
        visible, metrics, reason = probe._candidate_visibility_metrics(
            sequence.grasp, target, sequence=sequence)
        if visible:
            return sequence
        normal = original.np.asarray(geometry.support_normal_base, dtype=float)
        norm = float(original.np.linalg.norm(normal))
        if normal.shape != (3,) or not original.np.all(original.np.isfinite(normal)) or norm <= 1e-9:
            return sequence
        normal = normal / norm
        camera = original.pose_matrix(sequence.pregrasp).dot(transform)
        denominator = float(normal.dot(camera[:3, 0]))
        if abs(denominator) <= 1e-9:
            return sequence
        ray_depth = float(normal.dot(target - camera[:3, 3])) / denominator
        if not math.isfinite(ray_depth) or ray_depth <= 0.0:
            return sequence
        # Moving parallel to the support plane towards the optical-axis
        # intersection improves the view without turning the jaws or changing
        # pregrasp height. Search only up to 40 mm and the original stage bound.
        offset = target - (camera[:3, 3] + ray_depth * camera[:3, 0])
        offset = offset - normal.dot(offset) * normal
        length = float(original.np.linalg.norm(offset))
        if not math.isfinite(length) or length <= 1e-9:
            return sequence
        direction = offset / length
        max_distance = self._effective_adaptive_stage_limits().pregrasp_max_m
        for millimeters in range(5, 41, 5):
            distance = min(millimeters / 1000.0, length)
            proposal = deepcopy(sequence)
            for axis, value in zip('xyz', distance * direction):
                position = proposal.pregrasp.pose.position
                setattr(position, axis, getattr(position, axis) + float(value))
            separation = (original.pose_matrix(proposal.pregrasp)[:3, 3]
                          - original.pose_matrix(proposal.grasp)[:3, 3])
            if float(original.np.linalg.norm(separation)) > max_distance + 1e-12:
                continue
            visible, metrics, reason = probe._candidate_visibility_metrics(
                proposal.grasp, target, sequence=proposal)
            if visible:
                proposal.unknown_pregrasp_view = dict(
                    offset_base_m=(distance * direction).tolist(),
                    offset_m=distance, maximum_offset_m=.040,
                    maximum_pregrasp_distance_m=float(max_distance),
                    support_height_unchanged=True,
                    orientations_and_contact_poses_unchanged=True,
                    metrics=metrics, reason=reason)
                return proposal
        # Keep the original rejection; never relax the view/range criteria.
        return sequence

    @classmethod
    def _execution_sequence_audit(cls, sequence, near_field):
        report = super()._execution_sequence_audit(sequence, near_field)
        adjustment = getattr(sequence, 'unknown_pregrasp_view', None)
        if adjustment is not None:
            report['pregrasp_view_adjustment'] = deepcopy(adjustment)
        return report

    def _make_contact_sequence(self, grasp_pose, geometry,
                               insertion_axis_base=None, snapshot=None):
        sequence, profile = super()._make_contact_sequence(
            grasp_pose, geometry, insertion_axis_base, snapshot)
        return self._visible_unknown_pregrasp(sequence, geometry), profile

    def _dedupe_exact_contact_sequences_for_moveit(self, candidates, prepared,
            *, acceptance_diagnostics=None):
        batch = super()._dedupe_exact_contact_sequences_for_moveit(
            candidates, prepared, acceptance_diagnostics=acceptance_diagnostics)
        if ((self._mode_selection or {}).get('mode') != 'unknown'
                or not getattr(self, 'near_field_planning_active', False)):
            return batch
        # A jaw flip expanded twice can differ only in float64 roundoff. Drop
        # redundant *proposals*, never transfer a cached IK result or authority.
        # Every retained original pose still receives every downstream check.
        tolerance = 8. * original.np.finfo(float).eps
        unique, seen, duplicates = [], [], []
        for candidate in batch:
            if not isinstance(candidate, original.ScoredStableCandidate):
                unique.append(candidate)
                continue
            key, matrices = None, None
            runtime = self._stable_variant_runtime.get(
                (candidate.track_id, candidate.variant_index), {})
            stable = candidate.stable_candidate
            if (stable.candidate_source == 'tabletop_geometry'
                    and runtime.get('prepared') is prepared
                    and original.mandatory_safety_gate(candidate.latest_safety).ok):
                try:
                    sequence = runtime['sequence']
                    audit = self._execution_sequence_audit(sequence, True)
                    rows = audit['stages']
                    if (not audit['available'] or len(rows) != 4 or any(
                            not row['frame_id'] or row['stamp_ns'] <= 0 for row in rows)):
                        raise ValueError('incomplete sequence')
                    matrices = original.np.stack([original.pose_matrix(getattr(sequence, name))
                        for name in ('pregrasp', 'approach', 'grasp', 'lift')])
                    if not original.np.all(original.np.isfinite(matrices)):
                        raise ValueError('nonfinite sequence')
                    for row in rows:
                        orientation = getattr(sequence, row['stage']).pose.orientation
                        q = original.np.asarray([getattr(orientation, axis) for axis in 'xyzw'], dtype=float)
                        if q.shape != (4,) or abs(float(original.np.linalg.norm(q)) - 1.) > 1e-6:
                            raise ValueError('invalid orientation')
                    key = (candidate.evaluation_request_id,
                        candidate.evaluation_snapshot_stamp_sec,
                        candidate.evaluation_context_revision, stable.target_epoch,
                        stable.target_track_id, tuple(stable.center_base_xyz),
                        float(stable.required_open_width_m),
                        tuple((row['stage'], row['frame_id'], row['stamp_ns'],
                               row['linear_from_prior_state']) for row in rows))
                except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
                    key = None
            match = next(((prior, poses) for identity, poses, prior in seen
                          if key is not None and identity == key and
                          float(original.np.max(original.np.abs(poses - matrices))) <= tolerance), None)
            if match is not None:
                prior, poses = match
                duplicates.append(dict(track_id=candidate.track_id,
                    variant_index=candidate.variant_index,
                    representative_track_id=prior.track_id,
                    representative_variant_index=prior.variant_index,
                    maximum_transform_difference=float(original.np.max(original.np.abs(poses - matrices)))))
                continue
            unique.append(candidate)
            if key is not None:
                seen.append((key, matrices, candidate))
        report = dict(policy='same_request_float64_roundoff_four_stage_proposals',
            input_count=len(batch), unique_count=len(unique),
            maximum_transform_difference_allowed=tolerance,
            motion_result_reuse=False, duplicates=duplicates)
        if acceptance_diagnostics is not None:
            acceptance_diagnostics['unknown_roundoff_proposal_deduplication'] = report
        if duplicates:
            rospy.loginfo('UNKNOWN_CONTACT_PROPOSAL_DEDUPLICATION %s', json.dumps(report, sort_keys=True))
        return tuple(unique)

    def _strict_check_final_near_field_plan(self, plan, prepared, runtime):
        sequence, result, metrics = super()._strict_check_final_near_field_plan(
            plan, prepared, runtime)
        if (self._mode_selection or {}).get('mode') == 'unknown':
            delta = float(result.joint_max_delta_rad)
            if not math.isfinite(delta) or delta < 0.0:
                raise original.CandidateContractError('UNKNOWN_CONTACT_MOTION_INVALID',
                    'final near-field joint turn must be finite and non-negative')
        return sequence, result, metrics

    def _resolved_sequence_geometry_gate(self, runtime, sequence):
        gate = super()._resolved_sequence_geometry_gate(runtime, sequence)
        if not gate.ok or (self._mode_selection or {}).get('mode') != 'unknown':
            return gate
        # Recheck the exact final sequence too: registration and strict IK can
        # change a pose after the first candidate visibility calculation.
        geometry = runtime['prepared'].geometry
        center = original.np.asarray(geometry.center_base, dtype=float)
        correction = runtime.get('contact_registration_transform')
        if correction is not None:
            correction = original._readonly_rigid_transform(
                correction, 'final visibility registration')
            center = correction[:3, :3].dot(center) + correction[:3, 3]
        visible, metrics, reason = self._candidate_visibility_metrics(
            sequence.grasp, center, sequence=sequence)
        required = not self._direct_strategy_selected()
        runtime['unknown_pregrasp_visibility'] = dict(
            ok=bool(visible), metrics=metrics, reason=reason, required=required)
        if required and not visible:
            return replace(gate, ok=False,
                failure_code='UNKNOWN_PREGRASP_NOT_VISIBLE',
                failure_reason=reason, failed_gate='unknown_pregrasp_visibility')
        return gate

    @staticmethod
    def _stage_runtime_params(staged):
        result = original.RemoteGrasp6DNode._stage_runtime_params(staged)
        staged.grasp_config = observation_config(
            staged.grasp_config, staged._mode_selection)
        if (staged._mode_selection or {}).get('mode') == 'unknown':
            staged.camera_visibility_gate_enabled = not ModeAwareRemoteGrasp6DNode._direct_strategy_selected(staged)
            staged.camera_visibility_diagnostic_enabled = True
            staged.camera_visibility_min_depth_m = max(
                0.070, getattr(staged, 'camera_visibility_min_depth_m', 0.070))
            staged.camera_visibility_max_depth_m = min(
                0.500, getattr(staged, 'camera_visibility_max_depth_m', 0.500))
            ModeAwareRemoteGrasp6DNode._use_unknown_motion_ranking(staged)
        return result

    def _observation_following_support_evaluation(self, prepared, result, target=None):
        result, report = super()._observation_following_support_evaluation(
            prepared, result, target)
        if (not result.reachable
                or (self._mode_selection or {}).get('mode') != 'unknown'):
            return result, report
        delta = float(result.joint_max_delta_rad)
        report = dict(report, unknown_observation_joint_delta_rad=delta,
                      unknown_observation_motion_policy='rank_checked_duration_and_joint_motion')
        if not math.isfinite(delta) or delta < 0.0:
            code = 'UNKNOWN_OBSERVATION_MOTION_INVALID'
            reason = '%s: checked joint turn is not finite and non-negative' % code
            report.update(ok=False, reason=reason)
            result = replace(result, reachable=False, failure_code=code,
                             reason=reason, evidence_code='')
        if (result.reachable and self._observe_return_selected()
                and self._observe_return_phase == 'observation_prefix'):
            baseline = (self._frozen_observation_translation_delta_m(prepared, target)
                        if target is not None else float('nan'))
            report['observe_return_view_translation_m'] = baseline
            if baseline is None or not math.isfinite(baseline) or baseline < .015:
                code = 'OBSERVE_RETURN_VIEW_BASELINE_INSUFFICIENT'
                reason = code + ': observation needs at least 15 mm actual camera/tool translation'
                report.update(ok=False, reason=reason)
                result = replace(result, reachable=False, failure_code=code,
                                 reason=reason, evidence_code='')
        return result, report

    def _register_near_field_surface(self, observation):
        if (self._observe_return_selected() and self._observe_return_phase != 'initial_contact'):
            return super()._register_near_field_surface(observation)
        if not self._direct_strategy_selected():
            return super()._register_near_field_surface(observation)
        # The original two-stage entry seeds this structure from the reached
        # observation. Direct planning has no reached-view event: seed it from
        # this exact current snapshot, retaining only measured points. This is
        # one view, not cross-view registration or a synthesized hidden side.
        with self._geometry_state_guard():
            if (not isinstance(observation, original.TargetObservation)
                    or not self._selection_is_current()
                    or not self.near_field_planning_active
                    or observation.identity != self._current_stream_target_identity()
                    or observation.stamp_ns <= self._mode_selection['stamp_ns']
                    or observation.stamp_ns < int(round(self._near_field_phase_started_sec * 1e9))):
                raise ValueError('direct surface does not belong to the current phase and target')
            view = original.surface_view_from_observation(observation)
            surface = original.fused_surface_from_view(
                view, voxel_size_m=float(getattr(self, 'geometry_voxel_size_m', .0025)))
            self._near_field_reference_view = view
            self._near_field_fused_surface = surface
            self._near_field_surface_phase_id = self._near_field_phase_id
            self._near_field_surface_generation = self._stream_generation
            self._near_field_reference_center_base = None
            self._latest_registration_evidence = None
        return None

    def _multiview_surface_audit(self):
        audit = super()._multiview_surface_audit()
        if self._direct_strategy_selected() and (not self._observe_return_selected()
                or self._observe_return_phase == 'initial_contact'):
            audit['evidence_kind'] = 'current_view_only'
            audit['cross_view_registration_performed'] = False
        return audit

    def _observation_distance_preferences(self, geometry, snapshot, reference_pose, prepared):
        if self.grasp_config.get('observation_acceptance_policy') != WORKING_VOLUME_POLICY:
            return super()._observation_distance_preferences(geometry, snapshot, reference_pose, prepared)
        camera = self._camera_intrinsics()
        proposed = original.pose_matrix(reference_pose).dot(self._tool_from_camera_matrix())
        current = original._readonly_rigid_transform(
            prepared.pose_estimator.T_base_camera_link, 'source camera transform')
        limits = self._observation_view_limit_configuration()['effective_visibility_limits']
        return observation_distance_preferences(
            observation_obb_corners(geometry.center_base, geometry.axes_base, geometry.size_xyz_m),
            geometry.center_base, proposed[:3, :3], current[:3, 3],
            [camera.fx, camera.fy, camera.cx, camera.cy], camera.width, camera.height, **limits)

    def _observation_reference_tilt_variants(self, reference_tool_pose, support_normal_base):
        standard = super()._observation_reference_tilt_variants(
            reference_tool_pose, support_normal_base)
        if not (self._observe_return_selected() and self._observe_return_phase == 'observation_prefix'):
            return standard
        self._observe_return_directed_views = ()
        try:
            tool_camera = original._readonly_rigid_transform(
                self._tool_from_camera_matrix(), 'observation T_tool0_camera_link')
            reference = original.pose_matrix(reference_tool_pose)
            rotation = reference[:3, :3].dot(tool_camera[:3, :3])
            identity = self._current_stream_target_identity()
            gaps = tuple(gap for gap in self._observe_return_view_gaps
                if gap['target_epoch'] == identity.epoch and gap['target_track_id'] == identity.track_id)
            if self._observe_return_diagnostic_observation_only:
                # This proposal direction belongs to the original ambiguous
                # source. Fresh support is separately verified by the unchanged
                # 7 mm/4 degree union contract and endpoint/path collision gates.
                # Passing a newly fitted normal here would fail the source
                # equality check even for sub-degree accepted fit variation.
                source_normal = self._observe_return_silhouette_evidence['view_geometry']['support_normal_base']
                directed = directed_silhouette_rotations(self._observe_return_silhouette_evidence,
                    rotation, source_normal, self._observe_return_view_uncertainty_m,
                    self.grasp_config.get('observation_camera_target_nominal_distance_m', .260))
            else:
                directed = directed_missing_side_rotations(gaps, rotation, support_normal_base,
                    self._observe_return_view_uncertainty_m,
                    self.grasp_config.get('observation_camera_target_nominal_distance_m', .260),
                    allow_geometry_directed_expansion=True)
            rolls = bounded_existing_camera_rolls(self._parse_observation_camera_roll_offsets_deg(
                getattr(self, 'observation_camera_roll_offsets_deg',
                        original.DEFAULT_OBSERVATION_CAMERA_ROLL_OFFSETS_DEG)))
            added, rolled_views = [], []
            for camera_rotation, metadata in directed:
                for roll in rolls:
                    local_roll = original.quaternion_matrix(original.quaternion_from_euler(
                        math.radians(roll), 0., 0.))[:3, :3]
                    rolled_camera = camera_rotation.dot(local_roll)
                    roll_metadata = dict(metadata, camera_roll_offset_deg=roll,
                        maximum_existing_roll_variants=25,
                        configured_existing_roll_variants=len(rolls),
                        optical_axis_and_centered_camera_origin_unchanged=True)
                    rolled_views.append((rolled_camera, roll_metadata))
                    transform = original.np.eye(4)
                    transform[:3, :3] = rolled_camera.dot(tool_camera[:3, :3].T)
                    added.append(dict(preference_index=len(standard)+len(added),
                        camera_roll_offset_deg=roll,
                        camera_tilt_offset_deg=metadata['camera_rotation_offset_deg'],
                        reference_pose=original.make_pose_stamped(
                            str(reference_tool_pose.header.frame_id or 'base_link'),
                            reference[:3, 3], original.quaternion_from_matrix(transform),
                            stamp=reference_tool_pose.header.stamp)))
            self._observe_return_directed_views = tuple(rolled_views)
            return tuple(standard) + tuple(added)
        except (AttributeError, KeyError, TypeError, ValueError):
            return standard

    def _observation_distance_reference_variant(self, reference_variant, geometry, camera_distance):
        variant = super()._observation_distance_reference_variant(
            reference_variant, geometry, camera_distance)
        if not (self._observe_return_selected() and self._observe_return_phase == 'observation_prefix'
                and not self._observe_return_diagnostic_observation_only):
            return variant
        mount = original._readonly_rigid_transform(
            self._tool_from_camera_matrix(), 'observation T_tool0_camera_link')
        pose = original.pose_matrix(variant['reference_pose'])
        rotation = pose[:3, :3] @ mount[:3, :3]
        identity = self._current_stream_target_identity()
        gaps = tuple(g for g in self._observe_return_view_gaps
            if g.get('target_epoch') == identity.epoch and g.get('target_track_id') == identity.track_id)
        for proposed, metadata in getattr(self, '_observe_return_directed_views', ()):
            if (not original.np.allclose(rotation, proposed, atol=1e-8, rtol=0.)
                    or metadata.get('target_epoch') != identity.epoch
                    or metadata.get('target_track_id') != identity.track_id
                    or abs(float(metadata['camera_rotation_offset_deg'])
                        - float(metadata['minimum_required_rotation_deg'])) > 1e-8):
                continue
            peer_angles = [float(peer['camera_rotation_offset_deg'])
                for _, peer in self._observe_return_directed_views
                if all(peer.get(k) == metadata.get(k) for k in
                    ('target_epoch','target_track_id','angle_deg','missing_side_sign','jaw_axis_base','rotation_axis_base'))]
            fixed, evidence = distance_bound_missing_side_rotation(
                gaps, rotation, metadata, geometry.center_base,
                camera_distance, self._observe_return_view_uncertainty_m,
                geometry.support_normal_base, existing_upper_angle_deg=max(peer_angles))
            if evidence is metadata:
                return variant  # An old passing pose, upper slot or no bounded proposal.
            pose[:3, :3] = fixed @ mount[:3, :3].T
            reference = variant['reference_pose']
            return dict(variant,
                reference_pose=original.make_pose_stamped(str(reference.header.frame_id or 'base_link'),
                    pose[:3, 3], original.quaternion_from_matrix(pose), stamp=reference.header.stamp),
                camera_tilt_offset_deg=float(evidence['camera_rotation_offset_deg']),
                distance_bound_directed_missing_side=deepcopy(evidence))
        return variant

    @staticmethod
    def _observation_rejected_variant_evidence(variant):
        """Preserve passed endpoint evidence when a later view test rejects it."""
        envelope = variant.get('observation_envelope')
        record = {key: variant[key] for key in (
            'camera_target_distance_m', 'camera_roll_offset_deg',
            'camera_tilt_offset_deg', 'search_family') if key in variant}
        if isinstance(envelope, original.ObservationEnvelopeResult):
            record.update(observation_envelope=asdict(envelope),
                minimum_support_clearance_m=envelope.minimum_support_clearance_m)
        return record

    def _joint_space_observation_active(self):
        return bool(self._unknown_observation_planning_active()
            and not self._observe_return_diagnostic_observation_only
            and self._observe_return_trigger in ('BILATERAL_SURFACE_EVIDENCE_MISSING',
                                                'BOX_INFERENCE_PRECISION_VIEW_REQUIRED'))

    def _joint_space_observation_candidates(self, prepared):
        """Bounded proposals; a one-count handoff never authorizes a path."""
        if prepared is None or not self._joint_space_observation_active():
            return (), {}
        from alicia_grasp_modes.reachable_observation_views import generate_reachable_observation_views
        from alicia_grasp_modes.observation_information_uncertainty import observation_information_uncertainty_m
        from alicia_flexible_grasp.robot.observation_tracking_contract import tracking_contract_candidates
        from alicia_flexible_grasp.robot.stationary_following import SDK_QUANTUM_RAD
        names = tuple('Joint%d' % i for i in range(1, 7))
        cache = getattr(self, '_joint_space_observation_cache', None)
        same_request = isinstance(cache, dict) and cache.get('prepared') is prepared
        used = float(cache.get('used_sec', 0.)) if same_request else 0.
        source = None
        invalidated = bool(cache.get('invalidated')) if same_request else False
        report = dict(search_family='joint_space_fk', execution_authority=False, status='unavailable')

        def reject(reason, status='inputs_changed'):
            nonlocal report
            report = dict(report, status=status, reason=str(reason), candidate_count=0, used_sec=used)
            previous = getattr(self, '_joint_space_observation_cache', None)
            previous = previous if isinstance(previous, dict) and previous.get('prepared') is prepared else {}
            self._joint_space_observation_cache = dict(prepared=prepared,
                source=previous.get('source') or source, key=None, candidates=(),
                audit=deepcopy(report), used_sec=used, invalidated=invalidated,
                owner=previous.get('owner'), search_seed=previous.get('search_seed'))
            return (), report

        try:
            if invalidated:
                report = deepcopy(cache['audit'])
                return reject(report.get('reason', 'joint-space observation source or owner invalidated'))

            def require_source():
                nonlocal invalidated
                try:
                    current = self._require_diagnostic_observation_view_source(prepared)
                except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
                    invalidated = True
                    raise
                expected = source if source is not None else (cache.get('source') if same_request else None)
                if expected is not None and current is not expected:
                    invalidated = True
                    raise ValueError('joint-space observation source changed')
                return current

            def measured_seed():
                state = deepcopy(getattr(self, 'latest_joint_state', None))
                stamp = state.header.stamp.to_nsec()
                age = (rospy.Time.now().to_nsec() - stamp) * 1e-9
                if (stamp <= 0 or not 0. <= age <= .5
                        or len(state.name) != len(set(state.name))
                        or len(state.name) != len(state.position)
                        or not set(names).issubset(state.name)
                        or not original.np.all(original.np.isfinite(state.position))):
                    raise ValueError('fresh finite measured joint seed required')
                positions = dict(zip(state.name, state.position))
                return [float(positions[name]) for name in names], stamp

            def owner():
                nonlocal invalidated
                if rospy.get_param('/gui/joint_direct_mode', False) is not False:
                    invalidated = True
                    raise ValueError('joint-space observation manual control active')
                if not self._joint_space_observation_active():
                    invalidated = True
                    raise ValueError('joint-space observation owner phase changed')
                return deepcopy(dict(selection=self._mode_selection,
                    control_epoch_ns=getattr(self, '_measurement_remote_epoch_ns', 0),
                    session=getattr(self, '_observe_return_session', None),
                    supplement=getattr(self, '_observe_return_pending_supplement', None),
                    phase=self._observe_return_phase, paused=self._observe_return_paused))

            def search_inputs():
                geometry = prepared.geometry
                description = rospy.get_param('/robot_description', '')
                inputs = dict(binding=dict(source['binding']),
                    source_audit=original._thaw_request_evidence(source['audit']),
                    model_sha256=hashlib.sha256(str(description).encode()).hexdigest(),
                    controller_constraints=deepcopy(rospy.get_param('/alicia_controller/constraints', None)),
                    tracking_contract_enabled=rospy.get_param('/robot/observation_tracking_contract_enabled', False),
                    stop_duration_sec=rospy.get_param('/alicia_controller/stop_trajectory_duration', None),
                    gripper=repr(self.gripper_geometry), opening_width_m=min(
                        float(self.gripper_geometry.max_inner_gap_m), float(self.gripper_physical_open_width_m)),
                    gaps=deepcopy(self._observe_return_view_gaps),
                    uncertainty=self._observe_return_view_uncertainty_m,
                    optical_side_uncertainty_m=observation_information_uncertainty_m(
                        self._adaptive_stage_profiles(geometry, prepared.snapshot)),
                    view_limits=original._thaw_request_evidence(source['view_limits']),
                    owner=owner(), geometry=dict(
                        support_normal_base=original.np.asarray(geometry.support_normal_base).tolist(),
                        support_offset_m=float(geometry.support_offset_m),
                        R_base_obb=original.np.asarray(geometry.axes_base).tolist(),
                        obb_size_xyz_m=original.np.asarray(geometry.size_xyz_m).tolist()))
                return inputs, description, json.dumps(inputs, sort_keys=True, allow_nan=False)

            def within_quantum(anchor, latest):
                anchor, latest = original.np.asarray(anchor), original.np.asarray(latest)
                # Arithmetic at a nonzero joint angle can add a few ULPs to one
                # exact encoder step. This is not a tracking-error allowance.
                epsilon = 4. * original.np.finfo(float).eps * original.np.maximum(
                    1., original.np.maximum(original.np.abs(anchor), original.np.abs(latest)))
                return bool(original.np.all(original.np.abs(latest-anchor) <= SDK_QUANTUM_RAD+epsilon))

            def handoff(candidates, anchor, latest, latest_stamp):
                nonlocal report
                delta = (original.np.asarray(latest)-original.np.asarray(anchor)).tolist()
                report = dict(report, latest_joint_positions=latest, latest_joint_stamp_ns=latest_stamp,
                    seed_handoff=dict(anchor_joint_positions=anchor, delta_rad=delta,
                        max_delta_rad=max(abs(v) for v in delta), limit_rad=SDK_QUANTUM_RAD,
                        execution_authority=False), used_sec=used)
                if not within_quantum(anchor, latest):
                    return reject('joint seed changed beyond one SDK count')
                # Preserve the original generated endpoint, seed and frozen-view
                # baseline. The later strict path uses the latest actual start;
                # retaining a proposal does not grant execution authority.
                retained = deepcopy(candidates)
                for candidate in retained:
                    candidate['seed_handoff'] = dict(deepcopy(report['seed_handoff']),
                        latest_joint_stamp_ns=latest_stamp, latest_joint_positions=deepcopy(latest))
                report = dict(report, candidate_count=len(retained))
                return tuple(retained), report

            source = require_source()
            q, joint_stamp = measured_seed()
            inputs, description, key = search_inputs()
            if same_request and cache.get('owner') is not None and cache['owner'] != inputs['owner']:
                report = deepcopy(cache['audit'])
                invalidated = True
                return reject('joint-space observation owner changed')
            if (same_request and cache.get('source') is source and cache.get('key') == key
                    and within_quantum(cache['search_seed'], q)):
                report = dict(deepcopy(cache['audit']), cache_hit=True)
                require_source()
                checked_inputs, _, checked_key = search_inputs()
                if checked_inputs['owner'] != inputs['owner']:
                    invalidated = True
                if checked_key != key:
                    report = dict(report, changed_input_fields=sorted(
                        name for name in inputs if checked_inputs[name] != inputs[name]))
                    return reject('joint-space observation configuration or owner changed')
                latest_q, latest_stamp = measured_seed()
                candidates, report = handoff(cache['candidates'], cache['search_seed'], latest_q, latest_stamp)
                if self._joint_space_observation_cache is cache:
                    cache['audit'] = deepcopy(report)
                return candidates, report
            if same_request and cache.get('key') is not None and cache['key'] != key:
                previous_inputs = json.loads(cache['key'])
                report = dict(deepcopy(cache['audit']), changed_input_fields=sorted(
                    name for name in inputs if previous_inputs.get(name) != inputs[name]))
            # The shared budget is never reset by a new feedback packet, cache
            # invalidation, configuration change, or a failed post-search check.
            remaining = max(0., 2. - used)
            progress_context = getattr(self, '_station_observation_generation_context', None)
            if progress_context is not None:
                reserve = self._station_observation_search_checkpoint(progress_context, 'fk_proposals')
                remaining = min(remaining, max(0., min(reserve['remaining_sec'],
                    reserve['source_remaining_sec']) - reserve['estimated_strict_sec']
                    - reserve['publication_reserve_sec']))
            if remaining <= 0.:
                if same_request and cache.get('search_seed') is not None and not within_quantum(cache['search_seed'], q):
                    report = deepcopy(cache['audit'])
                    report['seed_handoff'] = dict(anchor_joint_positions=cache['search_seed'],
                        delta_rad=(original.np.asarray(q)-original.np.asarray(cache['search_seed'])).tolist(),
                        limit_rad=SDK_QUANTUM_RAD, execution_authority=False)
                return reject('shared FK proposal budget exhausted after input invalidation', 'budget_exhausted')
            errors = original.observation_tracking_error_bounds(inputs['controller_constraints'], names)
            contract = None
            if inputs['tracking_contract_enabled'] is True:
                contract = tracking_contract_candidates(inputs['controller_constraints'], names,
                    inputs['stop_duration_sec'])[-1]
                errors = contract['joint_error_bounds_rad']
            fk = original.SerialUrdfFk(description, names)
            frozen = dict(source, geometry=inputs['geometry'])
            candidates, report = generate_reachable_observation_views(source=frozen,
                current_joint_positions=q, fk=fk, gripper=self.gripper_geometry,
                opening_width_m=inputs['opening_width_m'], gaps=inputs['gaps'],
                depth_uncertainty_m=inputs['uncertainty'],
                side_depth_uncertainty_m=inputs['optical_side_uncertainty_m'],
                joint_error_bounds_rad=errors, max_seconds=remaining,
                max_directions=2, proposal_image_reserve_px=4.)
            report = dict(report, search_family='joint_space_fk', cache_hit=False,
                status=report.get('status', 'complete'), joint_seed_stamp_ns=joint_stamp, current_joint_positions=q,
                conditional_tracking_contract=contract, generated_candidate_count=len(candidates),
                execution_authority=False)
            used += float(report.get('elapsed_sec', remaining))
            # Store invalid by default: an exception in any subsequent source,
            # control, model, or feedback check cannot expose these proposals.
            self._joint_space_observation_cache = dict(prepared=prepared, source=source,
                key=None, candidates=(), audit=deepcopy(report), used_sec=used, owner=inputs['owner'],
                search_seed=deepcopy(q))
            require_source()
            checked_inputs, _, checked_key = search_inputs()
            if checked_inputs['owner'] != inputs['owner']:
                invalidated = True
            if checked_key != key:
                report = dict(report, changed_input_fields=sorted(
                    name for name in inputs if checked_inputs[name] != inputs[name]))
                return reject('joint-space observation configuration or owner changed')
            latest_q, latest_stamp = measured_seed()
            retained, report = handoff(candidates, q, latest_q, latest_stamp)
            if report['status'] == 'inputs_changed':
                return retained, report
            self._joint_space_observation_cache.update(key=key, candidates=deepcopy(candidates),
                search_seed=deepcopy(q), owner=inputs['owner'], audit=deepcopy(report))
            return retained, report
        except (AttributeError, KeyError, TypeError, ValueError, OverflowError) as exc:
            return reject(str(exc), 'inputs_changed' if same_request or source is not None else 'unavailable')

    def _joint_space_observation_variants(self, grasp_pose, geometry, insertion_axis,
            snapshot, prepared, existing):
        if prepared is None or not self._joint_space_observation_active():
            return (), ()
        candidates, search = self._joint_space_observation_candidates(prepared)
        if not candidates:
            return (), ()
        reference = self._frozen_observation_reference_pose(prepared)
        mount = self._tool_from_camera_matrix()
        reference_camera = original.pose_matrix(reference).dot(mount)
        output, rejected = [], []
        for index, proposal in enumerate(candidates):
            self._require_diagnostic_observation_view_source(prepared)
            source = self._diagnostic_observation_view_source
            if (proposal.get('execution_authority') is not False
                    or proposal.get('binding') != dict(source['binding'])
                    or proposal.get('model_sha256') != hashlib.sha256(str(
                        rospy.get_param('/robot_description', '')).encode()).hexdigest()):
                raise original.CandidateContractError('UNKNOWN_OBSERVATION_FK_BINDING_INVALID',
                    'joint-space proposal model or source changed')
            tool = original._readonly_rigid_transform(proposal['T_base_tool0'], 'proposed FK tool pose')
            camera = tool.dot(mount)
            if not original.np.allclose(camera, proposal['T_base_camera_link'], atol=1e-10, rtol=0.):
                raise original.CandidateContractError('UNKNOWN_OBSERVATION_FK_BINDING_INVALID',
                    'joint-space proposal handeye changed')
            # Rebuild only this contact family's diagnostic poses/profile;
            # the sole observation endpoint is the unchanged FK proposal.
            if existing:
                sequence = deepcopy(existing[0]['sequence'])
            elif self._independent_observation_planning_active():
                from alicia_grasp_modes.observation_information_uncertainty import observation_information_uncertainty_m
                uncertainty = observation_information_uncertainty_m(
                    self._adaptive_stage_profiles(geometry, snapshot))
                if not math.isfinite(uncertainty) or uncertainty < 0.:
                    raise original.CandidateContractError('UNKNOWN_OBSERVATION_CONTEXT_INVALID',
                        'independent FK view requires finite live depth uncertainty')
                sequence = SimpleNamespace(
                    adaptive_stage_profile=SimpleNamespace(depth_uncertainty_m=uncertainty))
            else:
                profile = self._contact_stage_profile(geometry, insertion_axis, snapshot=snapshot)
                sequence = original.make_grasp_sequence_from_grasp_pose(grasp_pose,
                    pregrasp_distance_m=profile.pregrasp_distance_m,
                    approach_offset_m=profile.approach_offset_m, lift_height_m=profile.lift_height_m,
                    approach_direction_base=insertion_axis)
                sequence.adaptive_stage_profile = profile
            sequence.pregrasp = original.make_pose_stamped('base_link', tool[:3, 3],
                original.quaternion_from_matrix(tool), stamp=prepared.stamp)
            target_camera = camera[:3, :3].T.dot(original.np.asarray(geometry.center_base)-camera[:3, 3])
            radial = float(original.np.linalg.norm(target_camera))
            tilt = math.degrees(math.acos(float(original.np.clip(
                reference_camera[:3, 0].dot(camera[:3, 0]), -1., 1.))))
            audit = dict(search_family='joint_space_fk', construction='measured_joint_space_fk',
                orientation_source='measured_joint_space_fk', contact_candidate_yaw_used=False,
                measured_target_geometry_direction_used=True, object_yaw_used=False,
                actual_camera_target_distance_m=radial, camera_target_distance_m=radial,
                camera_depth_m=float(target_camera[0]),
                center_residual_m=float(original.np.linalg.norm(target_camera[1:])),
                target_base_xyz=original.np.asarray(geometry.center_base).tolist(),
                tool0_position_xyz=tool[:3, 3].tolist(), camera_tilt_offset_deg=tilt,
                camera_roll_offset_deg=0., roll_lattice_coordinate_applicable=False,
                source_stamp_ns=prepared.snapshot.stamp_ns,
                proposal=deepcopy(proposal), proposal_search=deepcopy(search))
            sequence.observation_view_audit = audit
            side = self._observation_side_evidence(geometry, sequence.pregrasp, sequence.adaptive_stage_profile)
            envelope = self._observation_envelope_gate(geometry, sequence.pregrasp)
            variant = dict(search_family='joint_space_fk', sequence=sequence,
                observation_view=deepcopy(audit), observation_side_evidence=side,
                observation_envelope=envelope, camera_target_distance_m=radial,
                camera_tilt_offset_deg=tilt, camera_roll_offset_deg=0.,
                preference_index=max((int(v.get('preference_index', 0)) for v in existing), default=-1)+1+index,
                observation_translation_delta_m=self._frozen_observation_translation_delta_m(
                    prepared, sequence.pregrasp))
            failure = None
            if float(side['observation_side_evidence_deficit_m']) > 0.:
                failure = ('OBSERVATION_TILT_SIDE_EVIDENCE_INSUFFICIENT', 'information_gain')
            elif not isinstance(envelope, original.ObservationEnvelopeResult) or not envelope.ok:
                failure = (getattr(envelope, 'failure_code', '') or 'OBSERVATION_ENVELOPE_INVALID',
                    'endpoint_geometry')
            if failure:
                rejected.append(dict(failure_code=failure[0], rejection_stage=failure[1],
                    **self._observation_rejected_variant_evidence(variant)))
            else:
                output.append(variant)
        return tuple(output), tuple(rejected)

    def _observation_endpoint_cache_context(self, camera, limits, corners, tool_camera, arrival_projection=None):
        """Exact mode-filter inputs, never a path/authority cache key.

        Contact seed yaw and contact-stage poses do not enter these filters.
        The base still rebuilds each family and independently checks its side
        evidence. Only pregrasp, uncertainty and the full base-view evidence
        below can be shared when every request/source/calibration input agrees.
        """
        identity = self._current_stream_target_identity()
        return json.dumps(dict(
            selection=self._mode_selection, phase=self._observe_return_phase,
            paused=self._observe_return_paused,
            diagnostic=self._observe_return_diagnostic_observation_only,
            identity=[identity.epoch, identity.track_id],
            camera={key:getattr(camera, key) for key in
                    ('width', 'height', 'fx', 'fy', 'cx', 'cy', 'depth_scale')},
            limits=limits, corners=original.np.asarray(corners).tolist(),
            tool_camera=original.np.asarray(tool_camera).tolist(),
            arrival_projection=(None if arrival_projection is None else
                original._thaw_request_evidence(arrival_projection['audit'])),
            gaps=self._observe_return_view_gaps,
            uncertainty=self._observe_return_view_uncertainty_m,
            visited=[original.np.asarray(value).tolist()
                     for value in self._observe_return_visited_camera_origins],
            directed=[(original.np.asarray(rotation).tolist(), metadata)
                      for rotation, metadata in self._observe_return_directed_views]),
            sort_keys=True, allow_nan=False)

    def _far_field_observation_variants(self, *args, **kwargs):
        passing, rejected = super()._far_field_observation_variants(*args, **kwargs)
        if self._joint_space_observation_active():
            def argument(name, index):
                return kwargs.get(name, args[index] if len(args) > index else None)
            added, failures = self._joint_space_observation_variants(
                argument('grasp_pose', 0), argument('geometry', 1), argument('insertion_axis_base', 2),
                argument('snapshot', 3), argument('prepared', 5), passing)
            passing, rejected = tuple(passing)+added, tuple(rejected)+failures
        # Read current camera inputs for every invocation. The optional cache
        # below is bound to this request and exact numerical inputs; the original
        # source/calibration binding is still revalidated before path proof and
        # publication. No cached endpoint result grants motion authority.
        camera = None
        camera_transforms = {}
        mode_cache = None
        base_passing = tuple(passing)
        base_rejected_count = len(rejected)
        if self.grasp_config.get('observation_acceptance_policy') == WORKING_VOLUME_POLICY:
            geometry = kwargs.get('geometry', args[1] if len(args) > 1 else None)
            snapshot = kwargs.get('snapshot', args[3] if len(args) > 3 else None)
            camera = self._camera_intrinsics()
            limits = self._observation_view_limit_configuration()['effective_visibility_limits']
            corners = observation_obb_corners(geometry.center_base, geometry.axes_base, geometry.size_xyz_m)
            prepared = kwargs.get('prepared', args[5] if len(args) > 5 else None)
            arrival_projection = None
            if (prepared is not None and self._unknown_observation_planning_active()
                    and not self._observe_return_diagnostic_observation_only):
                arrival_projection = self._require_diagnostic_observation_view_source(prepared).get('arrival_projection')
            view_cache = kwargs.get('view_cache', args[6] if len(args) > 6 else None)
            if (isinstance(view_cache, dict) and self._unknown_observation_planning_active()
                    and not self._observe_return_diagnostic_observation_only):
                prepared = kwargs.get('prepared', args[5] if len(args) > 5 else None)
                reference = kwargs.get('observation_reference_pose', args[4] if len(args) > 4 else None)
                # Base cache identity is necessary as well as full numerical
                # mode inputs. A new request cannot inherit endpoint evidence.
                if (prepared is not None and reference is not None
                        and view_cache.get('prepared') is prepared
                        and view_cache.get('geometry') is geometry
                        and view_cache.get('snapshot') is snapshot
                        and view_cache.get('reference') is reference):
                    tool_camera = self._tool_from_camera_matrix()
                    context = self._observation_endpoint_cache_context(camera, limits, corners, tool_camera, arrival_projection)
                    inputs = []
                    for variant in base_passing:
                        sequence = variant['sequence']
                        pose = original.pose_matrix(sequence.pregrasp)
                        camera_transforms[id(sequence)] = pose.dot(tool_camera)
                        values = dict(variant)
                        values.pop('sequence')
                        envelope = values.get('observation_envelope')
                        if isinstance(envelope, original.ObservationEnvelopeResult):
                            values['observation_envelope'] = asdict(envelope)
                        values['sequence'] = dict(pregrasp=pose.tolist(),
                            depth_uncertainty_m=sequence.adaptive_stage_profile.depth_uncertainty_m,
                            observation_view_audit=sequence.observation_view_audit)
                        inputs.append(values)
                    signature = json.dumps(dict(context=context, source_stamp_ns=int(snapshot.stamp_ns),
                        variants=inputs), sort_keys=True, allow_nan=False)
                    cached = view_cache.get('mode_endpoint_evidence')
                    if (isinstance(cached, dict) and cached.get('signature') == signature
                            and cached.get('prepared') is prepared
                            and cached.get('geometry') is geometry
                            and cached.get('snapshot') is snapshot
                            and cached.get('reference') is reference):
                        admitted = []
                        for index, audit, gain in cached['passing']:
                            variant = dict(base_passing[index], observe_return_missing_side_gain=deepcopy(gain))
                            variant['sequence'].observation_view_audit = deepcopy(audit)
                            variant['observation_view'] = deepcopy(audit)
                            admitted.append(variant)
                        return tuple(admitted), tuple(rejected) + tuple(deepcopy(cached['rejected']))
                    mode_cache = (view_cache, signature, context, tool_camera, prepared, reference)
            admitted, rejected = [], list(rejected)
            for variant in passing:
                sequence = variant['sequence']
                try:
                    camera_transform = camera_transforms.get(id(sequence))
                    if camera_transform is None:
                        camera_transform = original.pose_matrix(sequence.pregrasp).dot(
                            self._tool_from_camera_matrix())
                    prediction = validate_planned_observation_working_volume(corners,
                        camera_transform,
                        [camera.fx, camera.fy, camera.cx, camera.cy], camera.width, camera.height,
                        snapshot_stamp_ns=int(snapshot.stamp_ns), **limits)
                except ValueError as exc:
                    rejected.append(dict(failure_code='OBSERVATION_WORKING_VOLUME_INVALID',
                        failure_reason=str(exc), rejection_stage='working_volume',
                        **self._observation_rejected_variant_evidence(variant)))
                    continue
                camera_transforms[id(sequence)] = camera_transform
                arrival_prediction = None
                if arrival_projection is not None:
                    from alicia_grasp_modes.observation_arrival_projection import validate_arrival_projection
                    try:
                        arrival_prediction = validate_arrival_projection(arrival_projection, camera_transform,
                            [camera.fx, camera.fy, camera.cx, camera.cy], camera.width, camera.height, **limits)
                    except ValueError as exc:
                        rejected.append(dict(failure_code='OBSERVATION_ARRIVAL_PROJECTION_INVALID',
                            failure_reason=str(exc), rejection_stage='arrival_projection_endpoint',
                            **self._observation_rejected_variant_evidence(variant)))
                        continue
                sequence.observation_view_audit = dict(sequence.observation_view_audit,
                    acceptance_policy=WORKING_VOLUME_POLICY, working_volume=prediction,
                    selection_rule=('bounded_measured_joint_space_proposal'
                        if variant.get('search_family') == 'joint_space_fk'
                        else 'target_fov_distance_preferences'), distance_is_soft_preference=True)
                if arrival_prediction is not None:
                    sequence.observation_view_audit['arrival_projection_endpoint'] = arrival_prediction
                variant['observation_view'] = deepcopy(sequence.observation_view_audit)
                admitted.append(variant)
            passing, rejected = tuple(admitted), tuple(rejected)
        if not (self._observe_return_selected() and self._observe_return_phase == 'observation_prefix'):
            return passing, rejected
        admitted, rejected = [], list(rejected)
        for variant in passing:
            baseline = variant.get('observation_translation_delta_m')
            if baseline is not None and math.isfinite(baseline) and baseline >= .015:
                gain = self._observe_return_view_gain(variant.get('sequence'))
                if gain['ok']:
                    gain_key = ('observe_return_silhouette_gain' if
                        self._observe_return_diagnostic_observation_only else 'observe_return_missing_side_gain')
                    variant = dict(variant, **{gain_key: gain})
                    sequence = variant['sequence']
                    sequence.observation_view_audit = dict(
                        getattr(sequence, 'observation_view_audit', {}) or {},
                        **{gain_key: deepcopy(gain)})
                    if self._observe_return_directed_views and variant.get('search_family') != 'joint_space_fk':
                        camera_transform = camera_transforms.get(id(sequence))
                        if camera_transform is None:
                            camera_transform = original.pose_matrix(sequence.pregrasp).dot(
                                self._tool_from_camera_matrix())
                        camera_rotation = camera_transform[:3, :3]
                        for proposed_rotation, metadata in self._observe_return_directed_views:
                            if original.np.allclose(camera_rotation, proposed_rotation, atol=1e-8, rtol=0.):
                                diagnostic = self._observe_return_diagnostic_observation_only
                                sequence.observation_view_audit.update(
                                    orientation_source=('measured_silhouette_ambiguity_geodesic'
                                        if diagnostic else 'measured_missing_side_geodesic'),
                                    object_yaw_used=True, contact_candidate_yaw_used=False,
                                    direction_source=('measured_silhouette_ambiguity'
                                        if diagnostic else 'measured_bilateral_weak_side'),
                                    measured_target_geometry_direction_used=True,
                                    **{('observe_return_directed_silhouette' if diagnostic else
                                        'observe_return_directed_missing_side'): deepcopy(metadata)})
                                break
                    if not self._observe_return_diagnostic_observation_only:
                        # Prediction ranks search only. The most distant OBB
                        # corner and weaker focal axis avoid overstating image
                        # resolution; accepted registration remains mandatory.
                        depths = sequence.observation_view_audit['working_volume']['target_corner_depths_m']
                        if camera is None:
                            camera = self._camera_intrinsics()
                        benefit = predicted_information_utility(
                            gain['predicted_new_side_gain_m'], gain['depth_uncertainty_m'],
                            max(depths), min(camera.fx, camera.fy))
                        sequence.observation_view_audit = dict(sequence.observation_view_audit,
                            information_utility=benefit)
                    variant['observation_view'] = deepcopy(sequence.observation_view_audit)
                    admitted.append(variant)
                else:
                    rejected.append(dict(failure_code=gain['code'],
                        rejection_stage='information_gain',
                        **self._observation_rejected_variant_evidence(variant),
                        failure_reason='existing observation does not add the required measured visual information',
                        observation_translation_delta_m=baseline, **{('observe_return_silhouette_gain'
                            if self._observe_return_diagnostic_observation_only else
                            'observe_return_missing_side_gain'): gain}))
            else:
                rejected.append(dict(reason='OBSERVE_RETURN_VIEW_BASELINE_INSUFFICIENT',
                    failure_code='OBSERVE_RETURN_VIEW_BASELINE_INSUFFICIENT',
                    rejection_stage='view_baseline',
                    **self._observation_rejected_variant_evidence(variant),
                    observation_translation_delta_m=baseline, minimum_translation_m=.015))
        if mode_cache is not None:
            view_cache, signature, context, tool_camera, prepared, reference = mode_cache
            if arrival_projection is not None:
                self._require_diagnostic_observation_view_source(prepared)
            if context == self._observation_endpoint_cache_context(camera, limits, corners, tool_camera, arrival_projection):
                indices = {id(variant['sequence']): index for index, variant in enumerate(base_passing)}
                view_cache['mode_endpoint_evidence'] = dict(signature=signature,
                    prepared=prepared, geometry=geometry, snapshot=snapshot, reference=reference,
                    passing=deepcopy(tuple((indices[id(variant['sequence'])], variant['observation_view'],
                        variant['observe_return_missing_side_gain']) for variant in admitted)),
                    rejected=deepcopy(tuple(rejected[base_rejected_count:])))
        return tuple(admitted), tuple(rejected)

    def _order_observation_checks(self, variants):
        if self._unknown_observation_planning_active():
            runtime = getattr(self, '_active_unknown_observation_check_runtime', None)
            if isinstance(runtime, dict):
                context = self._station_observation_search_context(runtime['prepared'])
                if context is not None:
                    return self._station_progress_observation_checks(variants, context)
            if (self._joint_space_observation_active()
                    and any(v.get('search_family') == 'joint_space_fk' for v in variants)):
                return self._fk_first_observation_checks(variants)
            ordered = self._order_unknown_observation_checks(variants)
            if self._joint_space_observation_active():
                return self._balance_observation_family_checks(ordered)
            return ordered
        original_order = super()._order_observation_checks(variants)
        if not (self._observe_return_selected() and self._observe_return_phase == 'observation_prefix'):
            return original_order
        directed, legacy = [], []
        for variant in original_order:
            audit = getattr(variant.get('sequence'), 'observation_view_audit', {}) or {}
            (directed if audit.get('orientation_source') in ('measured_missing_side_geodesic',
                'measured_silhouette_ambiguity_geodesic') else legacy).append(variant)
        ordered = diversified_observation_checks(directed,
            self.grasp_config.get('observation_camera_target_nominal_distance_m', .26))
        for index, variant in enumerate(ordered):
            sequence = variant['sequence']
            sequence.observation_view_audit = dict(sequence.observation_view_audit,
                check_scheduling_policy='bounded_existing_roll_and_distance_interleave',
                check_scheduling_index=index, original_strict_check_budget_unchanged=True)
            variant['observation_view'] = deepcopy(sequence.observation_view_audit)
        return tuple(ordered) + tuple(legacy)

    def _fk_first_observation_checks(self, variants):
        """Try one same-stratum FK endpoint before spending advisory IK time."""
        clock = getattr(self, '_observation_first_strict_clock', time.monotonic)
        started = clock()
        ordered = self._order_unknown_observation_checks(variants, defer_endpoint_hints=True)
        inspection = ordered.ranked_order if isinstance(ordered, ObservationCheckSchedule) else ordered
        strongest = min(support_schedule_priority(
            v['observation_view']['check_scheduling_metrics']['measured_support_schedule'])
            for v in inspection)
        first = next((v for v in inspection if v.get('search_family') == 'joint_space_fk'
            and support_schedule_priority(v['observation_view']['check_scheduling_metrics'][
                'measured_support_schedule']) == strongest), None)
        runtime = getattr(self, '_active_unknown_observation_check_runtime', None)
        if runtime is None:
            runtime = self._stable_variant_runtime[(0, 0)]
        prepared = runtime['prepared']
        comparison = getattr(self, '_observation_information_comparison', None)
        # This state is set by the original far-field checker for this exact
        # request. An incumbent from another family cannot prove our result.
        fresh_comparison = (isinstance(comparison, dict)
            and comparison.get('ticket') is prepared.ticket
            and comparison.get('deadline') is None)
        if first is None or not fresh_comparison:
            return iter(self._balance_observation_family_checks(
                self._order_unknown_observation_checks(variants)))
        timing = dict(policy='same_support_stratum_fk_before_advisory_ik',
            scheduler_started_monotonic=started, first_strict_delay_sec=None,
            hint_elapsed_sec=0., first_full_path_qualified=False,
            delay_scope='ordering_entry_to_original_strict_call',
            support_priority=strongest, hint_budget_reset=False,
            strict_budget_extended=False, execution_authority=False)
        runtime['observation_first_strict'] = timing
        scheduling = runtime['unknown_observation_scheduling']
        scheduling.update(first_strict_dispatch=timing,
            original_ranking_policy=scheduling['policy'], policy=timing['policy'])
        audit = first['sequence'].observation_view_audit
        audit.update(first_dispatch_original_check_index=audit['check_scheduling_index'],
            check_scheduling_index=0, check_scheduling_policy=timing['policy'])
        first['observation_view'] = deepcopy(audit)

        def dispatch():
            runtime['_first_fk_strict_target'] = first['sequence'].pregrasp
            try:
                yield first
            finally:
                runtime.pop('_first_fk_strict_target', None)
            self._require_unknown_observation_prepared(prepared)
            if (getattr(self, '_observation_information_comparison', None) is comparison
                    and comparison.get('ticket') is prepared.ticket
                    and comparison.get('deadline') is not None
                    and timing['first_strict_delay_sec'] is not None):
                # The base loop sets this deadline only after strict planning,
                # full view/following proof and a current-ticket commit pass.
                timing['first_full_path_qualified'] = True
                timing['scheduler_elapsed_sec'] = max(0., clock() - started)
                return
            hint_started = clock()
            hint_used_before = float((getattr(self,
                '_unknown_observation_endpoint_hint_cache', None) or {}).get('used_sec', 0.))
            remaining = tuple(v for v in variants if v is not first)
            fallback = self._balance_observation_family_checks(
                self._order_unknown_observation_checks(remaining))
            timing['deferred_order_elapsed_sec'] = max(0., clock() - hint_started)
            timing['hint_elapsed_sec'] = max(0., float((getattr(self,
                '_unknown_observation_endpoint_hint_cache', None) or {}).get('used_sec', 0.))
                - hint_used_before)
            timing['scheduler_elapsed_sec'] = max(0., clock() - started)
            # The fallback rewrites its scheduling report; retain actual
            # first-dispatch timing without claiming a failed view was safe.
            runtime['unknown_observation_scheduling']['first_strict_dispatch'] = timing
            runtime['unknown_observation_scheduling'].update(
                candidate_count=len(variants), already_checked_first_fk_count=1,
                deferred_hint_candidate_count=len(remaining))
            yield from fallback
        return dispatch()

    def _strict_moveit_evaluation(self, target, observation=False):
        runtime = getattr(self, '_active_unknown_observation_check_runtime', None)
        context = runtime.get('station_observation_search') if isinstance(runtime, dict) else None
        dispatch = runtime.get('_station_search_dispatch') if isinstance(runtime, dict) else None
        if observation and context is not None and dispatch is not None:
            # Counts are scheduling facts only. A cancelled or changed owner
            # cannot commit a late RPC result into another station's history.
            self._require_unknown_observation_prepared(context['prepared'])
            family = dispatch['family']; clock = context['clock']; started = clock()
            with self._stream_condition:
                if not context['progress'].start(context['token'], family):
                    raise original.StreamResultCancelled('GENERATION_STALE')
            result = super()._strict_moveit_evaluation(target, observation=observation)
            elapsed = max(0., clock()-started)
            current = self._station_observation_search_context(context['prepared'])
            if (current is None or current['token'] != context['token']
                    or current['progress'] is not context['progress']
                    or current['owner'] != context['owner']):
                raise original.StreamResultCancelled('GENERATION_STALE')
            code = str(result[0].failure_code or '')
            with self._stream_condition:
                context['progress'].finish(context['token'], family, elapsed_sec=elapsed, code=code)
            row = dict(family=family, elapsed_sec=elapsed, failure_code=code,
                reachable=bool(result[0].reachable), source_stamp_ns=context['prepared'].snapshot.stamp_ns,
                current_positive_endpoint=self._station_observation_positive_endpoint(dispatch['variant'],context))
            context['dispatched'].append(row)
            original.rospy.loginfo('observation station strict check request=%s evidence=%s',
                context['prepared'].ticket.request_id, json.dumps(row,sort_keys=True,allow_nan=False))
            return result
        if (observation and isinstance(runtime, dict)
                and runtime.get('_first_fk_strict_target') is target):
            timing = runtime['observation_first_strict']
            clock = getattr(self, '_observation_first_strict_clock', time.monotonic)
            timing['first_strict_delay_sec'] = max(
                0., clock() - timing['scheduler_started_monotonic'])
            original.rospy.loginfo(
                'observation first FK strict request=%s scheduling_delay_sec=%.6f advisory_ik=deferred',
                runtime['prepared'].ticket.request_id, timing['first_strict_delay_sec'])
        return super()._strict_moveit_evaluation(target, observation=observation)

    @staticmethod
    def _interleave_observation_families(ordered):
        """Share existing check starts; preserve support strata and family rank."""
        families = {v.get('search_family', 'camera_orientation_lattice') for v in ordered}
        if families != {'joint_space_fk', 'camera_orientation_lattice'}:
            return ordered
        output = []
        # The source-bound scheduler already orders these strata. A family
        # must not gain priority over stronger measured support evidence.
        for priority in range(3):
            group = [v for v in ordered if support_schedule_priority(
                v['observation_view']['check_scheduling_metrics']['measured_support_schedule']) == priority]
            joint = [v for v in group if v.get('search_family') == 'joint_space_fk']
            lattice = [v for v in group if v.get('search_family') != 'joint_space_fk']
            for index in range(max(len(joint), len(lattice))):
                if index < len(joint):
                    output.append(joint[index])
                if index < len(lattice):
                    output.append(lattice[index])
        return tuple(output)

    def _balance_observation_family_checks(self, ordered):
        inspection = ordered.initial_order if isinstance(ordered, ObservationCheckSchedule) else ordered
        balanced = self._interleave_observation_families(inspection)
        if all(first is second for first, second in zip(inspection, balanced)):
            return ordered
        policy = 'measured_support_strata_then_fk_lattice_round_robin'
        original_indices = {id(variant): index for index, variant in enumerate(inspection)}
        runtime = getattr(self, '_active_unknown_observation_check_runtime', None)
        if runtime is None:
            runtime = self._stable_variant_runtime[(0, 0)]
        scheduling = runtime['unknown_observation_scheduling']
        scheduling.update(original_ranking_policy=scheduling['policy'], policy=policy,
            family_check_scheduling=dict(first_family='joint_space_fk',
                within_family_rank_preserved=True, measured_support_strata_preserved=True,
                strict_budget_extended=False, candidates_removed=0, execution_authority=False,
                ordered_original_indices=[original_indices[id(v)] for v in balanced]))
        for index, variant in enumerate(balanced):
            sequence = variant['sequence']
            sequence.observation_view_audit = dict(sequence.observation_view_audit,
                check_scheduling_policy=policy, check_scheduling_index=index,
                family_original_check_index=original_indices[id(variant)],
                family_check_scheduling_only=True)
            variant['observation_view'] = deepcopy(sequence.observation_view_audit)
        if isinstance(ordered, ObservationCheckSchedule):
            return ordered.reorder_before_incumbent(balanced)
        return balanced

    def _information_driven_observation_comparison(self):
        return bool(self._unknown_observation_planning_active()
            and not self._observe_return_diagnostic_observation_only)

    def _checked_observation_variant_rank(self, result, variant):
        if not self._information_driven_observation_comparison():
            return super()._checked_observation_variant_rank(result, variant)
        audit = variant.get('observation_view', {})
        benefit = audit['information_utility']
        metrics = self._parse_plan_metrics(result.reason)
        duration = metrics.get('observation_screened_duration_sec',
            metrics.get('execution_duration_lower_bound_sec'))
        try:
            rank = information_motion_rank(benefit['utility'], duration,
                result.joint_max_delta_rad, result.joint_path_cost)
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            raise original.CandidateContractError('OBSERVATION_INFORMATION_RANK_INVALID', str(exc))
        audit['checked_information_motion_rank'] = dict(
            cost=rank[0] if math.isfinite(rank[0]) else None,
            screened_duration_sec=duration, utility=benefit['utility'],
            prediction_only=True, selection_scope='strictly_qualified_checked_views',
            claims_global_optimum=False)
        return (*rank, int(variant.get('preference_index', 0)))

    def _select_far_field_observation(self, reachable_candidates):
        if (self._mode_selection or {}).get('mode') != 'unknown':
            return super()._select_far_field_observation(reachable_candidates)
        # The base compares hardware duration inside each view family. Compare
        # the already checked winners across families by that same measured
        # motion evidence, after preserving the required side-evidence order.
        def nonnegative(value):
            try:
                value = float(value)
                return value if math.isfinite(value) and value >= 0 else float('inf')
            except (TypeError, ValueError):
                return float('inf')
        def rank(candidate):
            if self._information_driven_observation_comparison():
                runtime = self._stable_variant_runtime[(candidate.track_id, candidate.variant_index)]
                return self._checked_observation_variant_rank(candidate.moveit_result,
                    dict(observation_view=runtime['observation_view']))
            information = super(ModeAwareRemoteGrasp6DNode, self)._far_field_observation_moveit_rank_key(candidate)
            result = candidate.moveit_result
            metrics = self._parse_plan_metrics(result.reason)
            return (information[0], nonnegative(metrics.get('observation_screened_duration_sec',
                        metrics.get('execution_duration_lower_bound_sec'))),
                    nonnegative(result.joint_max_delta_rad), nonnegative(result.joint_path_cost),
                    *information[1:])
        candidates = tuple(reachable_candidates)
        if self._observe_return_selected() and self._observe_return_phase == 'observation_prefix':
            admitted = []
            for candidate in candidates:
                runtime = getattr(self, '_stable_variant_runtime', {}).get(
                    (candidate.track_id, candidate.variant_index), {})
                sequence = runtime.get('observation_sequence')
                baseline = self._frozen_observation_translation_delta_m(
                    runtime.get('prepared'), getattr(sequence, 'pregrasp', None))
                if baseline is not None and math.isfinite(baseline) and baseline >= .015:
                    gain = self._observe_return_view_gain(sequence)
                    if gain['ok']:
                        admitted.append(candidate)
            candidates = tuple(admitted)
        return min(candidates, key=rank) if candidates else None

    def _unknown_projected_body_width(self, candidate):
        """Rank the whole frozen object, not just a narrow local contact band.

        This is a search preference only. The measured aperture, exact pose,
        physical model and every geometry/MoveIt/MuJoCo gate are unchanged.
        """
        runtime = getattr(self, '_stable_variant_runtime', {}).get(
            (candidate.track_id, candidate.variant_index), {})
        try:
            geometry = runtime['prepared'].geometry
            sequence = runtime.get('final_execution_sequence') or runtime['sequence']
            rotation = original.np.asarray(geometry.axes_base, dtype=float).reshape(3, 3)
            size = original.np.asarray(geometry.size_xyz_m, dtype=float).reshape(3)
            tool_rotation = original.pose_matrix(sequence.grasp)[:3, :3]
            jaw = tool_rotation.dot(original.parse_tool_axis(self.gripper_tool_jaw_axis)[0])
            if (not original.np.all(original.np.isfinite(rotation))
                    or not original.np.all(original.np.isfinite(size))
                    or original.np.any(size <= 0.0)
                    or not original.np.allclose(rotation.T.dot(rotation), original.np.eye(3), atol=1e-8)
                    or abs(original.np.linalg.det(rotation)-1.0) > 1e-8
                    or not original.np.all(original.np.isfinite(jaw))
                    or abs(original.np.linalg.norm(jaw)-1.0) > 1e-8):
                return float('inf')
            width = float(original.np.abs(rotation.T.dot(jaw)).dot(size))
            return original.float32_wire_value(width)
        except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
            return float('inf')

    def _unknown_body_ranking_active(self):
        return ((self._mode_selection or {}).get('mode') == 'unknown'
                and bool(getattr(self, 'near_field_planning_active', False)))

    def _direct_near_field_moveit_rank_key(self, candidate):
        rank = super()._direct_near_field_moveit_rank_key(candidate)
        if not self._unknown_body_ranking_active():
            return rank
        width = self._unknown_projected_body_width(candidate)
        # The base search key is a millimetre-band index, not metres. Mixing
        # these units erased the whole-body short-side preference and let a
        # narrow corner contact consume the search before the short face.
        band = (math.floor(width / original.DIRECT_NEAR_FIELD_SEARCH_WIDTH_BIN_M)
                if math.isfinite(width) else float('inf'))
        return (max(rank[0], band), *rank[1:])

    def _direct_near_field_execution_rank_key(self, candidate):
        rank = super()._direct_near_field_execution_rank_key(candidate)
        if not self._unknown_body_ranking_active():
            return rank
        # Final comparison contains complete geometry/MoveIt/simulation successes.
        # Keep short-side geometry as a search preference and a tie-breaker;
        # a sub-millimetre width difference must not buy a full wrist reversal.
        # Physical aperture and all admission gates remain unchanged.
        width = max(rank[0], self._unknown_projected_body_width(candidate))
        return (*rank[1:4], width, *rank[4:])

    def _check_direct_registered_candidate(self, candidate):
        result = super()._check_direct_registered_candidate(candidate)
        if (not result.reachable
                or (self._mode_selection or {}).get('mode') != 'unknown'
                or not (self._direct_strategy_selected()
                        or getattr(self, 'near_field_planning_active', False))):
            return result
        cfg = dict(getattr(self, 'mujoco_config', {}) or {})
        if not cfg.get('enabled', True) or not cfg.get('execution_gate_enabled', True):
            return result  # The unchanged task gate still validates its config.
        runtime = self._stable_variant_runtime[(candidate.track_id, candidate.variant_index)]
        ticket = runtime['prepared'].ticket
        self._require_stream_ticket_current(ticket)
        token = (self._mode_selection['generation'], self._stream_generation, self._near_field_phase_id)
        budget = getattr(self, '_direct_simulation_budget', None)
        if budget is None or budget['token'] != token:
            budget = dict(token=token, count=0, started=time.monotonic())
            self._direct_simulation_budget = budget
        record = dict(attempted=False, passed=False)
        code, reason = 'MUJOCO_SELECTION_BUDGET_EXHAUSTED', 'direct candidate simulation budget exhausted'
        response = None
        if (budget['count'] < int(getattr(self, 'mujoco_selection_max_candidates', 12))
                and time.monotonic()-budget['started'] < float(getattr(self, 'mujoco_selection_time_budget_sec', 80.))):
            budget['count'] += 1
            record.update(attempted=True, attempt_index=budget['count'])
            try:
                score = original.soft_candidate_cost(replace(candidate.soft_features,
                    joint_path_cost=result.joint_path_cost,
                    joint_max_delta_rad=result.joint_max_delta_rad), candidate.score_weights).total
                checked = replace(candidate, moveit_result=result, final_score=score)
                plan = self._build_selected_preview_bundle(checked, ticket=ticket)['rich_plan']
                joints = deepcopy(self.latest_joint_state)
                plan_cfg = mass_config_from_ros(cfg, plan, self._mode_selection)
                plan_cfg = bind_simulated_fixed_close(plan, plan_cfg, rospy.get_param('/gripper', {}))
                payload = original.build_mujoco_payload(plan, list(joints.name), list(joints.position), plan_cfg)
                if 'target_mass_evidence' in plan_cfg:
                    record['operator_mass_evidence'] = deepcopy(plan_cfg['target_mass_evidence'])
                    record['simulation_mass_kg'] = payload['object_model']['mass_kg']
                factory = getattr(self, '_mujoco_client_factory', original.MujocoDigitalTwinClient)
                client = factory(cfg.get('server_url', 'http://172.23.132.97:8000'),
                                 timeout_sec=float(cfg.get('timeout_sec', 20.)))
                policy_cfg = dict(cfg)
                policy_cfg['unknown_lift_gate_enabled'] = rospy.get_param(
                    '/mujoco_digital_twin/unknown_lift_gate_enabled', True)
                record['lift_gate_enabled'] = original.mujoco_lift_gate_enabled(plan, policy_cfg)
                response = client.simulate_grasp(payload)
                self._require_stream_ticket_current(ticket)
                gate = original.validate_mujoco_gate_response(response, str(plan.plan_id), cfg.get('min_score', 80),
                    expected_candidate_source=plan.candidate_source,
                    expected_candidate_source_lineage=plan.candidate_source_lineage,
                    require_lift_success=record['lift_gate_enabled'])
                code, reason = (gate.code or 'OK', gate.reason) if gate.ok else (gate.code, gate.reason)
                record.update(plan_id=plan.plan_id, passed=bool(gate.ok), score=gate.score)
            except original.StreamResultCancelled:
                raise
            except Exception as exc:
                code, reason = 'WSL_UNAVAILABLE', str(exc)
        record.update(code=code, reason=reason, response=self._mujoco_response_audit(response))
        self._record_mujoco_selection_attempt(candidate, record)
        if record['passed']:
            return result
        rospy.loginfo('Unknown contact candidate rejected by MuJoCo: track=%s variant=%s code=%s reason=%s',
                      candidate.track_id, candidate.variant_index, code, reason)
        # Reject this candidate inside the original bounded search. The next
        # candidate must independently pass geometry, strict paths and MuJoCo;
        # execution still repeats its plan-bound MuJoCo gate on fresh joints.
        return original.MoveItResult(reachable=False, joint_path_cost=result.joint_path_cost,
            joint_max_delta_rad=result.joint_max_delta_rad,
            reason='%s: %s' % (code, reason), failure_code='MOVEIT_CHECK_ERROR')

    def _active_profile_requires_mask(self):
        if (self._mode_selection or {}).get('mode') == 'unknown':
            self._last_model_choice = 'unknown_tabletop'
            return True
        return super()._active_profile_requires_mask()

    def _freeze_graspnet_input_config(self):
        config = super()._freeze_graspnet_input_config()
        if ((self._mode_selection or {}).get('mode') != 'unknown'
                or config.mode != original.CONTEXT_ROI):
            return config
        # Small targets need a smaller support-plane context crop than the
        # carton profile. Keep every evidence minimum and physical gate;
        # freeze these two crop fields with the request so existing input
        # audits report the actual values and ROI used by remote inference.
        return replace(
            config,
            context_margin_px=rospy.get_param(
                '/unknown_perception/graspnet_context_margin_px', 12),
            context_expand_ratio=rospy.get_param(
                '/unknown_perception/graspnet_context_expand_ratio', 0.15))

    def start_streaming(self):
        with self._mode_control_lock:
            if not self._selection_is_current(authoritative=True):
                return False
            direct = self._direct_strategy_selected()
            if direct:
                budget = float(getattr(self, 'near_field_direct_timeout_sec', 60.0))
                now_ns = int(self._ros_source_clock_ns())
                if not math.isfinite(budget) or budget <= 0.0 or now_ns <= 0:
                    raise ValueError('direct contact planning needs a valid source clock and budget')
            with self._stream_condition:
                changed = super().start_streaming()
                if changed and direct:
                    # This is a planning phase, not an observation-arrival
                    # claim. Contact geometry comes from the fresh fused
                    # snapshot and keeps NOT_EVALUATED registration evidence.
                    self.near_field_planning_active = True
                    self._near_field_phase_id += 1
                    self._near_field_phase_started_sec = now_ns / 1e9
                    self._near_field_phase_deadline_sec = now_ns / 1e9 + budget
                    self._direct_near_field_submission_generation = None
                    self._direct_snapshot_terminal_generation = None
                    self._direct_snapshot_last_rejection = None
                    self.last_submitted_stamp_ns = max(self.last_submitted_stamp_ns, now_ns)
                    self._reset_multiview_surface()
                return changed

    def stop_streaming(self):
        with self._mode_control_lock:
            changed = super().stop_streaming()
            if self._observe_return_selected():
                self._reset_observe_return()
            if self._direct_strategy_selected():
                with self._stream_condition, self._geometry_state_guard():
                    self.near_field_planning_active = False
                    self._near_field_phase_id = 0
                    self._near_field_phase_started_sec = 0.0
                    self._near_field_phase_deadline_sec = 0.0
                    self._direct_near_field_submission_generation = None
                    self._reset_multiview_surface()
            return changed

    def request_plan_cb(self, req):
        with self._mode_control_lock:
            trigger = bool(getattr(req, 'trigger', False))
            if trigger and not self._selection_is_current(authoritative=True):
                return original.TriggerZeroResponse(False, 'grasp mode reset is not ready')
            response = super().request_plan_cb(req)
            # The base treats start_streaming(False) as already running. A
            # changed owner at the second check is a rejection, including when
            # an earlier stream was running before this control request.
            if (trigger and response.success and (not self.streaming_enabled
                    or not self._selection_is_current(authoritative=True))):
                return original.TriggerZeroResponse(False, 'grasp mode changed before inference start')
            return response

    def replan_execution_cb(self, req):
        with self._mode_control_lock:
            if not self._selection_is_current():
                return original.TriggerZeroResponse(False, 'grasp mode reset is not ready')
            if self._observe_return_selected() and self._observe_return_paused:
                return original.TriggerZeroResponse(False, 'OBSERVE_RETURN_PHASE_PAUSED')
            if self._station_compute_poll():
                return original.TriggerZeroResponse(False, 'TARGET_PROCESSING_PENDING: fresh measurement required')
            binding = (deepcopy(self._mode_selection), self._current_stream_target_identity(),
                self._stream_generation, self._near_field_phase_id,
                self._observe_return_phase, self._observe_return_session)
        # Session admission needs the input/control lock to pause and cancel
        # the current worker. Holding it while waiting for that worker prevents
        # the cancellation from arriving and can consume the task's ACK budget.
        # Retain the worker/publication barrier; repeat all ownership checks
        # after waiting, without donating this request to a newer generation.
        with self._mode_work_lock:
            with self._mode_control_lock:
                if (not self._selection_is_current()
                        or binding[:5] != (self._mode_selection,
                            self._current_stream_target_identity(), self._stream_generation,
                            self._near_field_phase_id, self._observe_return_phase)
                        or binding[5] is not self._observe_return_session):
                    return original.TriggerZeroResponse(False, 'REPLAN_CONTEXT_CHANGED')
                if self._observe_return_selected() and self._observe_return_paused:
                    return original.TriggerZeroResponse(False, 'OBSERVE_RETURN_PHASE_PAUSED')
                if self._station_compute_poll():
                    return original.TriggerZeroResponse(False,
                        'TARGET_PROCESSING_PENDING: fresh measurement required')
                return super().replan_execution_cb(req)

    def _poll_stream_snapshot(self):
        station_pending = self._station_compute_poll()
        if self._observe_return_retry_reached_view():
            return False
        if station_pending:
            return False
        if self._observation_measurement_poll():
            return False
        if self._unknown_identity_poll():
            return False
        if self._observe_return_publish_identity_terminal():
            return False
        if self._unknown_identity_retired_selection == self._mode_selection:
            return False
        with self._mode_control_lock:
            if not self._selection_is_current() or self._observe_return_paused:
                return False
            session = self._observe_return_session
            if session and int(self._ros_source_clock_ns()) >= session['deadline_ns']:
                self._observe_return_paused = True
                return False
            direct = self._direct_strategy_selected()
        # Snapshot collection waits for callbacks, so release control before
        # waiting. Submission rechecks the mode and original target identity.
        if direct:
            return self._poll_direct_snapshot()
        return super()._poll_stream_snapshot()

    def _record_direct_snapshot_rejection(self, generation, phase_id, code, reason):
        with self._stream_condition:
            if (not self._mode_ready or not self.streaming_enabled
                    or generation != self._stream_generation
                    or phase_id != self._near_field_phase_id):
                return
            prior = getattr(self, '_direct_snapshot_last_rejection', None)
            record = {'code': str(code), 'reason': str(reason),
                      'phase_id': phase_id, 'stream_generation': generation,
                      'selection_generation': self._mode_selection['generation']}
            self._direct_snapshot_last_rejection = record
        if prior != record:
            self.status_pub.publish(String('DIRECT_SNAPSHOT_WAITING ' + json.dumps(record, sort_keys=True)))
            rospy.loginfo('direct contact snapshot waiting: %s: %s', code, reason)

    def _direct_buffer_diagnostics(self, identity, require_mask):
        frames = self.frames
        with frames._condition:
            rows = list(frames._entries.values())
            counts = {name: sum(name in row for row in rows)
                      for name in ('color_bgr', 'depth_raw', 'object_mask', 'object_msg')}
            complete = frames._complete_entries_locked(
                require_mask, frames._monotonic_clock(), self.planning_snapshot_max_age_sec,
                self.planning_snapshot_max_inference_latency_sec)
            counts['eligible_current_target'] = sum(
                row.get('target_identity') == identity for _key, row in complete)
        return json.dumps(counts, sort_keys=True)

    def _publish_direct_snapshot_timeout(self, generation, phase_id):
        # Use the same worker publication barrier as reset. No old phase can
        # leave a terminal preview latched after the next reset has returned.
        with self._mode_work_lock:
            with self._stream_condition:
                if (not self._selection_is_current() or not self.streaming_enabled
                        or generation != self._stream_generation
                        or phase_id != self._near_field_phase_id
                        or not self.near_field_planning_active
                        or self._stream_worker_busy
                        or bool(getattr(self, 'robot_execution_active', False))
                        or bool(getattr(getattr(self, 'latest_preview_rich_plan', None), 'valid', False))
                        or getattr(self, '_direct_snapshot_terminal_generation', None) == generation
                        or float(self._stream_source_clock()) < self._near_field_phase_deadline_sec):
                    return False
                self._direct_snapshot_terminal_generation = generation
                rejection = deepcopy(getattr(self, '_direct_snapshot_last_rejection', None))
            reason = 'no executable current-view contact plan before the original phase deadline'
            if rejection:
                reason += '; last rejection: %s: %s' % (rejection['code'], rejection['reason'])
            rich = original.Grasp6DPlan()
            rich.header.frame_id = 'base_link'
            rich.header.stamp = self._safe_time()
            rich.valid = False
            rich.candidate_source = 'near_field_terminal'
            rich.diagnostic = 'NEAR_FIELD_DIRECT_TIMEOUT: ' + reason
            legacy = original.PoseArray()
            legacy.header = deepcopy(rich.header)
            self._publish_preview_plan(rich, legacy)
            self.status_pub.publish(String(rich.diagnostic))
            rospy.logwarn('%s', rich.diagnostic)
            return True

    def _poll_direct_snapshot(self):
        """Use the original buffer/fuser gates, with explicit wait diagnostics.

        The base poller silently returns for an incomplete/rejected snapshot
        and after its phase expires. This current-view entry retains each
        original admission/fusion threshold and emits a bounded terminal.
        """
        with self._stream_condition:
            if not self.enabled or not self.streaming_enabled or not self.near_field_planning_active:
                return False
            generation = int(self._stream_generation)
            phase_id = int(self._near_field_phase_id)
            now = float(self._stream_source_clock())
            expired = now >= self._near_field_phase_deadline_sec
            if self._direct_near_field_submission_generation == generation and not expired:
                return False
            waiting_for_clock = now < self._near_field_phase_started_sec
            worker_busy = self._stream_worker_busy
            identity = self._current_stream_target_identity()
            newest_after_ns = max(self.last_submitted_stamp_ns,
                                  int(round(self._near_field_phase_started_sec * 1e9)))
            count = max(1, int(getattr(self, 'near_field_planning_snapshot_frames',
                                       self.planning_snapshot_frames)))
        if expired:
            self._publish_direct_snapshot_timeout(generation, phase_id)
            return False
        if waiting_for_clock or worker_busy:
            self._record_direct_snapshot_rejection(generation, phase_id,
                'PHASE_CLOCK_BEFORE_START' if waiting_for_clock else 'INFERENCE_WORKER_BUSY',
                'source clock %.9f, phase starts %.9f, worker busy %s' % (
                    now, self._near_field_phase_started_sec, worker_busy))
            return False
        if self._safe_time() < getattr(self, '_backoff_until', rospy.Time(0)):
            self._record_direct_snapshot_rejection(generation, phase_id, 'BACKOFF', 'waiting for original inference backoff')
            return False
        try:
            config = self._freeze_graspnet_input_config()
            require_mask = bool(self._active_profile_requires_mask() or config.requires_instance_mask)
        except Exception as exc:
            self._record_direct_snapshot_rejection(generation, phase_id,
                getattr(exc, 'code', 'CONFIG_INVALID'), str(exc))
            return False
        wait_timeout = min(max(0.0, float(self.planning_snapshot_timeout_sec)),
                           1.0 / max(1e-6, float(self.rate_hz)))
        samples = self.frames.wait_for_samples(
            count, wait_timeout, require_mask=require_mask,
            max_age_sec=self.planning_snapshot_max_age_sec,
            collection_span_sec=self.planning_snapshot_max_span_sec,
            max_inference_latency_sec=self.planning_snapshot_max_inference_latency_sec,
            newest_after_ns=newest_after_ns, target_identity=identity,
            require_all_after_ns=True)
        if len(samples) < count:
            self._record_direct_snapshot_rejection(generation, phase_id, 'SNAPSHOT_INCOMPLETE',
                'need %d exact fresh frames; buffer %s' % (
                    count, self._direct_buffer_diagnostics(identity, require_mask)))
            self._publish_direct_snapshot_timeout(generation, phase_id)
            return False
        depth_scale, depth_min, depth_max = self._snapshot_depth_config()
        snapshot = original.fuse_stable_samples(
            samples, require_mask=require_mask, min_mask_iou=self.planning_mask_min_iou,
            max_centroid_shift_px=self.planning_mask_max_centroid_shift_px,
            max_joint_delta_rad=self.planning_max_joint_delta_rad,
            erosion_px=self.mask_erosion_px, depth_scale=depth_scale,
            depth_min_m=depth_min, depth_max_m=depth_max,
            mad_scale=self.depth_mad_scale, mad_absolute_floor_m=self.depth_mad_absolute_floor_m,
            internal_hole_max_area_px=self.mask_internal_hole_max_area_px)
        if not snapshot.ok:
            self._record_direct_snapshot_rejection(generation, phase_id,
                snapshot.failure_code, snapshot.failure_reason)
            self._publish_direct_snapshot_timeout(generation, phase_id)
            return False
        submitted = self.submit_stream_snapshot(
            snapshot, graspnet_input_config=config, expected_generation=generation,
            require_idle_worker=True)
        if not submitted:
            self._record_direct_snapshot_rejection(generation, phase_id, 'SUBMISSION_REJECTED',
                'snapshot stamp_ns=%d epoch=%d; current stamp_ns=%d epoch=%d generation=%d worker_busy=%s' % (
                    snapshot.stamp_ns, snapshot.target_epoch, self.last_submitted_stamp_ns,
                    self.target_instance_epoch, self._stream_generation, self._stream_worker_busy))
            self._publish_direct_snapshot_timeout(generation, phase_id)
        return submitted

    def submit_stream_snapshot(self, snapshot, **kwargs):
        with self._mode_control_lock:
            if not self._selection_is_current() or self._observe_return_paused:
                return False
            with self._stream_condition:
                self._station_compute_tick_locked()
                if (self._station_compute_pending or self._station_compute_retired
                        or any(int(ns) <= self._station_compute_cutoff_ns for ns in
                               (tuple(getattr(snapshot, 'sample_stamp_ns', ())) or (snapshot.stamp_ns,)))):
                    return False
                if (self._unknown_identity_pending is not None
                        or self._unknown_identity_retired_selection == self._mode_selection
                        or int(snapshot.stamp_ns) <= self._unknown_identity_cancelled_through_ns):
                    return False
                if (any(int(ns) <= self._unknown_identity_cancelled_through_ns
                        for ns in getattr(snapshot, 'sample_stamp_ns', ()))
                        and (not isinstance(snapshot, original.SnapshotResult)
                             or not self._admit_observation_recovery_history(snapshot))):
                    return False
                work_proof = self._station_compute_snapshot_work(snapshot)
                if (any(int(ns) <= self._station_compute_soft_cutoff_ns for ns in
                        (tuple(getattr(snapshot, 'sample_stamp_ns', ())) or (snapshot.stamp_ns,)))
                        and work_proof is None):
                    return False
                submitted = super().submit_stream_snapshot(snapshot, **kwargs)
                if submitted:
                    self._station_compute_register_work(snapshot, work_proof)
                if submitted and self._joint_space_observation_active():
                    self._observation_history_entered = self._observation_history_request_key()
                return submitted

    def _stream_ticket_cancellation_code_locked(self, ticket):
        if not self._mode_ready:
            return 'GRASP_MODE_CHANGED'
        processing_code = self._station_compute_ticket_code(ticket)
        if processing_code:
            return processing_code
        if self._observe_return_selected():
            session = self._observe_return_session
            if self._observe_return_paused or (session and int(self._ros_source_clock_ns()) >= session['deadline_ns']):
                return 'OBSERVE_RETURN_PHASE_PAUSED'
        pending_code = self._unknown_identity_ticket_code(ticket)
        if pending_code:
            return pending_code
        return super()._stream_ticket_cancellation_code_locked(ticket)

    def _prepare_snapshot_geometry(self, snapshot, stamp):
        estimate, depth, transform = super()._prepare_snapshot_geometry(snapshot, stamp)
        if estimate.ok:
            self._observe_return_reacquire_failed_registration(snapshot)
        if not (self._observe_return_selected() and self._observe_return_diagnostic_observation_only
                and self._observe_return_phase == 'observation_prefix'):
            return estimate, depth, transform
        self._silhouette_observation_envelope_audit = None
        if not estimate.ok:
            return estimate, depth, transform
        try:
            source = self._observe_return_diagnostic_collision_source
            frozen = self._observe_return_diagnostic_collision_geometry
            identity = self._current_stream_target_identity()
            if (not self._selection_is_current() or source['selection'] != self._mode_selection
                    or source['target_epoch'] != identity.epoch
                    or source['target_track_id'] != identity.track_id
                    or snapshot.target_identity != identity
                    or snapshot.target_epoch != identity.epoch
                    or snapshot.stamp_ns <= source['source_stamp_ns']):
                raise ValueError('diagnostic observation collision source identity changed')
            with self._silhouette_source_lock:
                status = deepcopy(self._silhouette_source_status.get(snapshot.stamp_ns))
            initial_status = source['perception_status']
            if (not isinstance(status, dict) or not isinstance(initial_status, dict)
                    or status.get('state') != 'ready' or status.get('mode') != 'unknown'
                    or status.get('target_lost') is not False or status.get('exact_rgbd') is not True
                    or status.get('generation') != self._mode_selection['generation']
                    or status.get('selection_stamp_ns') != self._mode_selection['stamp_ns']
                    or any(status.get(key) != snapshot.stamp_ns for key in
                        ('source_stamp_ns','color_stamp_ns','depth_stamp_ns','tf_lookup_stamp_ns'))
                    or any(status.get(key) != snapshot.frame_id for key in ('color_frame_id','depth_frame_id'))
                    or snapshot.frame_id != 'camera_link'
                    or status.get('projection_frame_convention') != 'ros_camera_link'):
                raise ValueError('diagnostic observation requires exact current ready perception status')
            reference_keys = ('anchor_source_stamp_ns', 'anchor_position_base',
                              'reference_support_plane_base', 'reference_support_footprint_base')
            if any(key not in initial_status or status.get(key) != initial_status[key]
                   for key in reference_keys):
                raise ValueError('perception frozen anchor or support reference changed')
            # Perception already enforces its 7 mm default fixed-reference
            # gate. This extra envelope comparison uses separate fitted planes
            # and can only reject more; it does not replace that gate.
            geometry, audit = conservative_silhouette_observation_geometry(frozen, estimate,
                original_source_stamp_ns=source['source_stamp_ns'], fresh_source_stamp_ns=snapshot.stamp_ns,
                max_size_m=float(getattr(self, 'geometry_max_size_m', .600)),
                max_height_m=float(getattr(self, 'geometry_max_height_m', .500)),
                support_config=UnknownPerceptionConfig())
            audit = deepcopy(audit)
            audit.update(selection=deepcopy(self._mode_selection), target_epoch=identity.epoch,
                target_track_id=identity.track_id,
                original_request_id=source['request_id'], original_generation=source['generation'],
                fresh_generation=self._stream_generation,
                perception_reference={key: deepcopy(status[key]) for key in reference_keys},
                perception_source_stamp_ns=snapshot.stamp_ns,
                perception_support_audit=deepcopy(status['geometry_metrics']))
            self._silhouette_observation_envelope_audit = audit
            return geometry, depth, transform
        except (AttributeError, KeyError, TypeError, ValueError, OverflowError) as exc:
            return (self._invalid_geometry_estimate('SILHOUETTE_OBSERVATION_ENVELOPE_INVALID',
                str(exc), snapshot.source_mode), depth, transform)

    def _add_silhouette_envelope_diagnostic(self, prepared):
        if not (self._observe_return_selected() and self._observe_return_diagnostic_observation_only
                and self._observe_return_phase == 'observation_prefix'):
            return prepared
        audit = deepcopy(self._silhouette_observation_envelope_audit)
        if (not isinstance(audit, dict) or audit['source']['fresh_stamp_ns'] != prepared.snapshot.stamp_ns
                or audit['fresh_generation'] != prepared.ticket.generation
                or audit['target_epoch'] != prepared.ticket.target_epoch):
            raise original.CandidateContractError('SILHOUETTE_OBSERVATION_ENVELOPE_INVALID',
                'prepared observation is missing its bound conservative collision envelope')
        audit['fresh_request_id'] = prepared.ticket.request_id
        diagnostics = original._thaw_request_evidence(prepared.remote_diagnostics)
        diagnostics['silhouette_observation_envelope'] = audit
        return replace(prepared, remote_diagnostics=diagnostics)

    def _silhouette_source_status_cb(self, msg):
        """Bounded source-stamped evidence cache, never a current/latest fallback."""
        try:
            status = json.loads(msg.data)
            if not isinstance(status, dict):
                return
            self._station_compute_status_cb(status)
            self._unknown_identity_status_cb(status)
            self._measurement_remote_status_cb(status)
            ns = status['source_stamp_ns']
            if (type(ns) is not int or ns <= 0 or status.get('state') != 'ready'
                    or status.get('mode') != 'unknown' or status.get('target_lost') is not False):
                return
            # Do not take the mode-control lock: the inference worker can be
            # preparing a frame while this frame's status publication arrives.
            with self._silhouette_source_lock:
                self._silhouette_source_status[ns] = status
                while len(self._silhouette_source_status) > 64:
                    del self._silhouette_source_status[min(self._silhouette_source_status)]
        except (KeyError, TypeError, ValueError):
            return

    def _capture_silhouette_source(self, ticket):
        """Capture the exact original final sample before geometry/WSL latency."""
        if ((self._mode_selection or {}).get('mode') != 'unknown'
                or not self._observe_return_selected()
                or self._observe_return_phase != 'initial_contact'):
            return None
        try:
            snapshot, _config = ticket.payload
            sample = retained_exact_source(snapshot)
            if sample is None:
                sample = self.frames.wait_for_exact_sample(
                    snapshot.stamp_ns, snapshot.target_identity, timeout_sec=0., require_mask=True,
                    max_age_sec=self.planning_snapshot_max_age_sec,
                    max_inference_latency_sec=self.planning_snapshot_max_inference_latency_sec)
            raw = rospy.get_param('/unknown_perception', {})
            perception = UnknownPerceptionConfig(**{
                f.name: raw[f.name] for f in fields(UnknownPerceptionConfig) if f.name in raw})
            camera = camera_geometry_snapshot(rospy.get_param('/camera', {}))
            require_rectified_source_after_activation(camera, sample.stamp_ns)
            intrinsics = self._camera_intrinsics()
            if (camera != camera_geometry_snapshot(rospy.get_param('/camera', {}))
                    or any(float(camera[key]) != float(getattr(intrinsics, key))
                           for key in ('fx', 'fy', 'cx', 'cy', 'depth_scale'))
                    or sample.depth_raw.shape != (camera['height'], camera['width'])
                    or sample.frame_id != camera['frame_id']):
                raise ValueError('silhouette_source_camera_geometry_changed')
            return dict(sample=sample, selection=deepcopy(self._mode_selection),
                intrinsics=intrinsics, perception=perception, camera_geometry=camera)
        except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
            return dict(error='SILHOUETTE_ORIGINAL_SOURCE_UNAVAILABLE')

    def _add_silhouette_source_diagnostic(self, prepared, source):
        """Attach immutable diagnosis; leave every candidate and geometry intact."""
        if source is None:
            return prepared
        started = time.perf_counter()
        failure = dict(ok=False, diagnostic_only=True, execution_authority=False,
            observation_authority=False, code='SILHOUETTE_ORIGINAL_SOURCE_UNAVAILABLE')
        try:
            sample = source.get('sample')
            if sample is None:
                diagnostic = failure
            else:
                with self._silhouette_source_lock:
                    status = deepcopy(self._silhouette_source_status.get(sample.stamp_ns))
                if status is None:
                    diagnostic = dict(failure, code='SILHOUETTE_EXACT_SOURCE_STATUS_UNAVAILABLE')
                else:
                    profiles = self._adaptive_stage_profiles(prepared.geometry, prepared.snapshot)
                    uncertainty = max(float(p.depth_uncertainty_m) for p in profiles)
                    intrinsics = source['intrinsics']
                    diagnostic = build_silhouette_snapshot_evidence(sample, prepared.snapshot,
                        prepared.geometry, prepared.pose_estimator._T_base_optical, intrinsics,
                        source['selection'], prepared.ticket,
                        SilhouetteSnapshotConfig(intrinsics.depth_scale, source['perception'], status,
                                                 source['camera_geometry']),
                        uncertainty)
        except (AttributeError, KeyError, TypeError, ValueError, OverflowError) as exc:
            diagnostic = dict(failure, code='SILHOUETTE_SOURCE_DIAGNOSTIC_UNAVAILABLE', reason=str(exc))
        diagnostics = original._thaw_request_evidence(prepared.remote_diagnostics)
        diagnostics['silhouette_ambiguity'] = diagnostic
        elapsed_ms = max(0., (time.perf_counter() - started) * 1000.)
        diagnostics.setdefault('ros_prepare_stages_ms', {})['silhouette_source_diagnostic'] = elapsed_ms
        return replace(prepared, remote_diagnostics=diagnostics,
            ros_prepare_ms=prepared.ros_prepare_ms + elapsed_ms)

    def _maybe_prepare_observation_from_contact_geometry(self, ticket, snapshot,
            geometry, transform, tabletop_candidates, tabletop_diagnostics):
        return self._observe_return_measured_view_preflight(ticket, snapshot,
            geometry, transform, tabletop_candidates, tabletop_diagnostics)

    def _prepare_and_predict(self, ticket):
        with self._mode_work_lock:
            self._require_stream_ticket_current(ticket)
            if self._independent_observation_planning_active():
                return self._prepare_diagnostic_observation_prediction(ticket)
            source = self._capture_silhouette_source(ticket)
            view_source = (self._capture_diagnostic_observation_view_source(ticket)
                if self._unknown_observation_planning_active() else None)
            self._observe_return_registration_ticket = ticket
            try:
                prepared = super()._prepare_and_predict(ticket)
            finally:
                self._observe_return_registration_ticket = None
            self._require_stream_ticket_current(ticket)
            prepared = self._add_silhouette_source_diagnostic(prepared, source)
            prepared = self._add_silhouette_envelope_diagnostic(prepared)
            if view_source is not None:
                self._bind_diagnostic_observation_view_source(prepared, view_source)
            # Diagnosis must not make an expired inference result admissible.
            self._require_stream_ticket_current(ticket)
            return prepared

    def _accept_prediction(self, prepared):
        if self._station_compute_work_certificate(prepared.ticket):
            self._station_compute_wait_for_current(prepared.ticket)
        with self._mode_work_lock:
            self._require_stream_ticket_current(prepared.ticket)
            if self._independent_observation_planning_active():
                return self._accept_diagnostic_observation_prediction(prepared)
            return super()._accept_prediction(prepared)

    def _build_streaming_gate_audit_report(self, prepared, selection, *args, **kwargs):
        if self._independent_observation_planning_active():
            return self._build_diagnostic_observation_gate_audit(prepared, selection, *args, **kwargs)
        report, summary, selected_lineage = super()._build_streaming_gate_audit_report(
            prepared, selection, *args, **kwargs)
        report['grasp_mode_selection'] = dict(self._mode_selection or {})
        report['snapshot_stamp_ns'] = int(getattr(prepared.snapshot, 'stamp_ns', 0))
        proposal = self._station_contact_retained_proposal(prepared.ticket)
        if (proposal is not None and report.get('plan_id') == getattr(proposal, 'plan_id', None)
                and self._station_contact_preview_work(proposal, prepared.ticket)):
            receipt = self._station_contact_computation_receipt(proposal, prepared.ticket)
            if receipt is not None:
                report[STATION_CONTACT_COMPUTATION] = receipt
        if self._unknown_body_ranking_active():
            report.setdefault('moveit_selection', {}).update({
                'aperture_metric': 'max_measured_band_and_frozen_obb_projected_width',
                'physical_aperture_and_gates_unchanged': True,
            })
            if self._direct_near_field_active(prepared):
                report['moveit_selection'].update({
                    'policy': 'CHECKED_HARDWARE_DURATION_THEN_JOINT_MOTION',
                    'ranking_order': ['execution_duration_lower_bound_sec',
                        'joint_max_delta_rad', 'joint_path_cost', 'aperture_width_m'],
                })
        if (self._mode_selection or {}).get('mode') == 'unknown' and not self._direct_strategy_selected():
            report['observation_selection_policy'] = 'SIDE_EVIDENCE_THEN_CHECKED_HARDWARE_DURATION'
            if self._information_driven_observation_comparison():
                report['observation_selection_policy'] = INFORMATION_MOTION_POLICY
                report['observation_ranking_contract'] = dict(
                    primary_cost='(1 + screened_duration_sec) / predicted_information_utility',
                    tie_breakers=['screened_duration_sec', 'joint_max_delta_rad', 'joint_path_cost'],
                    scope='strictly_qualified_checked_views', claims_global_optimum=False,
                    motion_angle_has_independent_weight=False)
        if self._direct_strategy_selected():
            report['execution_strategy'] = 'direct'
            report['planning_origin'] = 'current_view_contact_snapshot'
            report['observation_motion_performed'] = False
            selected = getattr(selection, 'selected', None)
            if selected is not None:
                runtime = getattr(self, '_stable_variant_runtime', {}).get(
                    (selected.track_id, selected.variant_index), {})
                gate = deepcopy(runtime.get('final_registered_geometry_gate'))
                record = report.get('selected')
                if isinstance(record, dict):
                    key = original.audit_lineage_key(record)[:3]
                    # The original execution schema deliberately requires
                    # byte-for-byte equivalent selected/evaluation lineage.
                    # Attach the same actual gate to every bound copy before
                    # hashing; do not alter candidate identity or validation.
                    for collection in ('stable_evaluations', 'lineage', 'rows'):
                        for row in report.get(collection, ()):
                            if (row.get('selected') is True
                                    and original.audit_lineage_key(row)[:3] == key
                                    and row.get('lineage_binding') == record.get('lineage_binding')):
                                row['final_geometry_gate'] = deepcopy(gate)
                    record['final_geometry_gate'] = gate
                if isinstance(selected_lineage, dict):
                    selected_lineage['final_geometry_gate'] = deepcopy(gate)
        self._observe_return_audit(report, prepared)
        return report, summary, selected_lineage

    def _write_gate_audit_report(self, report, ticket=None, commit_callback=None,
            replace_reference_sha256=None, output_path=None,
            reference_attr='_latest_gate_audit_reference'):
        def committed(reference):
            if commit_callback is not None:
                commit_callback(reference)
            if reference_attr == '_latest_gate_audit_reference':
                self._station_contact_commit_audit(report, reference, ticket)
        return super()._write_gate_audit_report(report, ticket=ticket,
            commit_callback=committed, replace_reference_sha256=replace_reference_sha256,
            output_path=output_path, reference_attr=reference_attr)

    def _contact_station_plan_candidates(self, request):
        return (super()._contact_station_plan_candidates(request)
                + self._station_contact_audited_proposals(request))

    def _publish_direct_near_field_terminal(self, prepared, code, reason):
        if self._observe_return_prepare_supplement(prepared, code):
            return
        ambiguity = self._observe_return_silhouette_failure(prepared, code)
        if ambiguity is not None and self._observe_return_silhouette_fallback(prepared, ambiguity):
            return
        if (self._observe_return_precheck_visual_failure(prepared, code)
                and self._observe_return_fallback(prepared)):
            return
        opportunity = self._observe_return_mixed_visual_failure(prepared, code)
        if opportunity is not None and self._observe_return_fallback(
                prepared, visual_opportunity=opportunity):
            return
        return super()._publish_direct_near_field_terminal(prepared, code, reason)

    def _publish_preview_plan(self, *args, **kwargs):
        # A worker failure can publish a terminal preview after acceptance
        # raises. Include that path in the same reset completion barrier.
        rich = args[0] if args else kwargs.get('rich_plan')
        ticket = kwargs.get('ticket', args[2] if len(args) > 2 else None)
        if getattr(rich, 'valid', False) and ticket is not None and self._station_compute_work_certificate(ticket):
            self._station_compute_wait_for_current(ticket)
        with self._mode_work_lock:
            rich = args[0] if args else kwargs.get('rich_plan')
            private_contact = False
            private_supplement = False
            if (rich is not None and not getattr(rich, 'valid', False)
                    and any(p.get('contact_proposal') is not None
                            for p in self._station_compute_work.values())):
                # Clock-only notices bypass this wrapper and carry their exact
                # station owner. An ordinary terminal preview cancels private
                # computation as well as the public proposal.
                with self._stream_condition, self._geometry_state_guard():
                    self._station_compute_quarantine()
            if getattr(rich, 'valid', False):
                with self._stream_condition:
                    self._station_compute_tick_locked()
                    if (self._station_compute_pending or self._station_compute_retired
                            or (self._station_compute_cutoff_ns > 0
                                and rich.header.stamp.to_nsec() <= self._station_compute_cutoff_ns)):
                        raise original.StreamResultCancelled('TARGET_PROCESSING_PENDING')
                    if self._unknown_identity_pending is not None:
                        raise original.StreamResultCancelled('TARGET_MEASUREMENT_PENDING')
                    ticket = kwargs.get('ticket', args[2] if len(args) > 2 else None)
                    if ticket is not None:
                        self._require_stream_ticket_current_locked(ticket)
                    # Publication starts before the first ROS send. A false
                    # callback during that send must use normal retirement,
                    # never the unpublished-planning identity exception.
                    self._unknown_identity_publication_started = True
                    private_contact = self._station_contact_preview_work(rich, ticket)
                    private_supplement = self._station_supplement_preview_work(rich, ticket)
                    if private_contact:
                        proof = self._station_compute_work[int(ticket.request_id)]
                        proof['contact_proposal'] = deepcopy(rich)
                        proof['contact_published'] = False
                        proof.pop('contact_audit', None)
                    if (self._station_compute_reference is not None
                            and not private_contact and not private_supplement):
                        self._station_compute_publication_started = True
            try:
                result = super()._publish_preview_plan(*args, **kwargs)
            except Exception:
                if getattr(rich, 'valid', False) and (self._station_compute_publication_started
                        or private_contact or private_supplement):
                    with self._stream_condition, self._geometry_state_guard():
                        self._station_compute_quarantine()
                raise
            if getattr(rich, 'valid', False) and self._station_compute_publication_started:
                with self._stream_condition:
                    self._station_compute_work_published = True
            if private_contact:
                with self._stream_condition:
                    proof = self._station_compute_work.get(int(ticket.request_id))
                    if proof is not None:
                        proof['contact_published'] = True
            plan = getattr(self, 'latest_preview_rich_plan', None)
            if (self._observe_return_selected() and self._observe_return_phase == 'observation_prefix'
                    and plan is not None and plan.valid
                    and plan.diagnostic == 'FAR_FIELD_OBSERVATION_PLAN'):
                self._observe_return_accept_view_plan(plan)
            return result


if __name__ == '__main__':
    rospy.init_node('remote_grasp6d_node')
    ModeAwareRemoteGrasp6DNode().spin()
