#!/usr/bin/env python3
"""Add a mode boundary around the unchanged production Grasp6D node.

The router pauses perception and inference before committing selection. Its
reset service is a completion barrier: no old inference can write caches or
publish a plan after the service returns. This node never commands the robot.
"""

import importlib.util
import json
from dataclasses import replace
from copy import copy, deepcopy
import math
from pathlib import Path
import sys
import threading
import time

import rospkg
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from alicia_grasp_modes.selection import SELECTION_PARAM, parse_selection
from alicia_grasp_modes.target_mass import mass_config_from_ros
from alicia_grasp_modes.observation_policy import (
    observation_config, UNKNOWN_OBSERVATION_MAX_JOINT_DELTA_RAD)


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


class ModeAwareRemoteGrasp6DNode(original.RemoteGrasp6DNode):
    def __init__(self):
        self._initialize_mode_boundary()
        super().__init__()
        self._mode_reset_service = rospy.Service(
            '/grasp_mode/reset_remote', Trigger, self.reset_mode_cb)
        self._mode_subscriber = rospy.Subscriber(
            SELECTION_PARAM, String, self.selection_cb, queue_size=1)

    def _initialize_mode_boundary(self):
        self._mode_control_lock = threading.RLock()
        self._mode_reset_lock = threading.RLock()
        self._mode_work_lock = threading.RLock()
        self._mode_phase_lock = threading.RLock()
        self._mode_selection = None
        self._mode_ready = False

    def _read_selection(self):
        return parse_selection(rospy.get_param(SELECTION_PARAM, None))

    def _selection_is_current(self):
        if not self._mode_ready or self._mode_selection is None:
            return False
        try:
            return self._read_selection() == self._mode_selection
        except (ValueError, TypeError, KeyError):
            return False

    def selection_cb(self, msg):
        """The atomic parameter is authoritative over delayed latched messages."""
        try:
            selected = parse_selection(msg.data)
            if selected != self._read_selection():
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
                selected = self._read_selection()
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
                if self._read_selection() != selected:
                    raise RuntimeError('mode selection changed during remote reset')
                self._mode_selection = dict(selected)
                if not hasattr(self, '_carton_internal_hole_area_px'):
                    self._carton_internal_hole_area_px = getattr(self, 'mask_internal_hole_max_area_px', 25)
                # A colour silhouette can contain missing-depth pixels. For
                # unknown objects these must remain missing, never become
                # interpolated contact geometry. Restore carton unchanged.
                self.mask_internal_hole_max_area_px = (
                    0 if selected['mode'] == 'unknown' else self._carton_internal_hole_area_px)
                # Unknown-object execution requires a fresh visual refinement
                # after reaching pregrasp. A soft visibility cost alone allowed
                # an otherwise valid endpoint to put the target below the image.
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
                    self.camera_visibility_gate_enabled = True
                    self.camera_visibility_diagnostic_enabled = True
                    self.camera_visibility_min_depth_m = max(
                        0.070, self.camera_visibility_min_depth_m)
                    self.camera_visibility_max_depth_m = min(
                        0.500, self.camera_visibility_max_depth_m)
                    self._bound_unknown_contact_turn()
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
                    if self._read_selection() != selected:
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
        # Reached-view reconstruction may wait for camera callbacks. Do not
        # hold their control lock while waiting for their input.
        with self._mode_phase_lock:
            # Direct planning owns a contact-only phase at the current view.
            # The task's normal entry/exit "far field" event must not convert
            # it into an observation request or manufacture reached evidence.
            if self._direct_strategy_selected():
                return
            if self._admit_frame(msg):
                return super().near_field_state_cb(msg)

    def _direct_strategy_selected(self):
        return (self._mode_selection or {}).get('strategy', 'two_stage') == 'direct'

    def grasp_state_cb(self, msg):
        previous_active = bool(getattr(self, 'robot_execution_active', False))
        super().grasp_state_cb(msg)
        if (previous_active and not bool(getattr(msg, 'active', False))
                and self._direct_strategy_selected()):
            self.stop_streaming()

    def _configured_near_field_strategy(self):
        if self._direct_strategy_selected():
            return 'single_snapshot_direct'
        return super()._configured_near_field_strategy()

    def _generate_tabletop_candidates(self, geometry, snapshot=None,
                                     contact_execution_phase=True, target_label=''):
        if ((self._mode_selection or {}).get('mode') != 'unknown'
                or contact_execution_phase):
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

    def _contact_boundary_tilts(self, proposal, support_point, support_normal,
                                probe_variants, maximum_tilt_deg, required_overlap_m,
                                contact_height_bounds_m=None):
        probe = None
        if ((self._mode_selection or {}).get('mode') == 'unknown'
                and getattr(self, 'near_field_planning_active', False)):
            from alicia_grasp_modes.contact_probe import contact_tilt_probe
            probe = contact_tilt_probe(proposal, support_point, support_normal,
                self.gripper_geometry, self.gripper_tool_jaw_axis,
                self.gripper_tool_finger_length_axis)
        return super()._contact_boundary_tilts(proposal, support_point, support_normal,
            probe_variants, maximum_tilt_deg, required_overlap_m,
            contact_height_bounds_m=contact_height_bounds_m, contact_probe=probe)

    def _bound_unknown_contact_turn(self):
        limit = float(getattr(self, 'candidate_max_joint_delta_rad', 0.0) or 0.0)
        self.candidate_max_joint_delta_rad = (
            min(limit, UNKNOWN_OBSERVATION_MAX_JOINT_DELTA_RAD)
            if math.isfinite(limit) and limit > 0.0
            else UNKNOWN_OBSERVATION_MAX_JOINT_DELTA_RAD)

    def _visible_unknown_pregrasp(self, sequence, geometry):
        """Bounded table-parallel view offset; all contact poses stay exact.

        This is proposal construction only. The caller still checks the full
        CAD sweep, strict MoveIt sequence and contact simulation before motion.
        """
        if ((self._mode_selection or {}).get('mode') != 'unknown'
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

    def _strict_check_final_near_field_plan(self, plan, prepared, runtime):
        sequence, result, metrics = super()._strict_check_final_near_field_plan(
            plan, prepared, runtime)
        if (self._mode_selection or {}).get('mode') == 'unknown':
            self._bound_unknown_contact_turn()
            delta = float(result.joint_max_delta_rad)
            if (not math.isfinite(delta) or delta < 0.0
                    or delta > self.candidate_max_joint_delta_rad):
                # Preserve exact rejected poses and metrics even if the phase
                # deadline expires before its aggregate audit is published.
                # Diagnostic only; the rejection below remains unconditional.
                try:
                    evidence = dict(plan_id=str(plan.plan_id),
                        joint_max_delta_rad=delta,
                        joint_limit_rad=self.candidate_max_joint_delta_rad,
                        input_joint_state=runtime.get('moveit_input_joint_state'),
                        strict_metrics=metrics,
                        stages={name: original.pose_matrix(getattr(sequence, name)).tolist()
                                for name in ('pregrasp', 'approach', 'grasp', 'lift')})
                    rospy.loginfo('UNKNOWN_CONTACT_TURN_REJECTION %s',
                                  json.dumps(evidence, sort_keys=True, allow_nan=False))
                except Exception as exc:
                    rospy.logwarn('Unable to serialize contact-turn diagnostic: %s', exc)
                raise original.CandidateContractError('UNKNOWN_CONTACT_TURN_LIMIT',
                    'final near-field joint turn %.6f rad exceeds %.6f rad' %
                    (delta, self.candidate_max_joint_delta_rad))
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
        runtime['unknown_pregrasp_visibility'] = dict(
            ok=bool(visible), metrics=metrics, reason=reason)
        if not visible:
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
            staged.camera_visibility_gate_enabled = True
            staged.camera_visibility_diagnostic_enabled = True
            staged.camera_visibility_min_depth_m = max(
                0.070, getattr(staged, 'camera_visibility_min_depth_m', 0.070))
            staged.camera_visibility_max_depth_m = min(
                0.500, getattr(staged, 'camera_visibility_max_depth_m', 0.500))
            ModeAwareRemoteGrasp6DNode._bound_unknown_contact_turn(staged)
        return result

    def _observation_following_support_evaluation(self, prepared, result, target=None):
        result, report = super()._observation_following_support_evaluation(
            prepared, result, target)
        if (not result.reachable
                or (self._mode_selection or {}).get('mode') != 'unknown'):
            return result, report
        delta = float(result.joint_max_delta_rad)
        limit = UNKNOWN_OBSERVATION_MAX_JOINT_DELTA_RAD
        report = dict(report, unknown_observation_joint_delta_rad=delta,
                      unknown_observation_joint_delta_limit_rad=limit)
        if not math.isfinite(delta) or delta < 0 or delta > limit:
            code = 'UNKNOWN_OBSERVATION_TURN_LIMIT'
            reason = '%s: checked joint turn %.6f rad exceeds %.6f rad' % (code, delta, limit)
            report.update(ok=False, reason=reason)
            result = replace(result, reachable=False, failure_code=code,
                             reason=reason, evidence_code='')
        return result, report

    def _register_near_field_surface(self, observation):
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
        if self._direct_strategy_selected():
            audit['evidence_kind'] = 'current_view_only'
            audit['cross_view_registration_performed'] = False
        return audit

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
            information = super(ModeAwareRemoteGrasp6DNode, self)._far_field_observation_moveit_rank_key(candidate)
            result = candidate.moveit_result
            metrics = self._parse_plan_metrics(result.reason)
            return (information[0], nonnegative(metrics.get('observation_screened_duration_sec',
                        metrics.get('execution_duration_lower_bound_sec'))),
                    nonnegative(result.joint_max_delta_rad), nonnegative(result.joint_path_cost),
                    *information[1:])
        candidates = tuple(reachable_candidates)
        return min(candidates, key=rank) if candidates else None

    def _check_direct_registered_candidate(self, candidate):
        result = super()._check_direct_registered_candidate(candidate)
        if (not result.reachable or not self._direct_strategy_selected()
                or (self._mode_selection or {}).get('mode') != 'unknown'):
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
                plan = self._build_selected_preview_bundle(checked)['rich_plan']
                joints = deepcopy(self.latest_joint_state)
                plan_cfg = mass_config_from_ros(cfg, plan, self._mode_selection)
                payload = original.build_mujoco_payload(plan, list(joints.name), list(joints.position), plan_cfg)
                if 'target_mass_evidence' in plan_cfg:
                    record['operator_mass_evidence'] = deepcopy(plan_cfg['target_mass_evidence'])
                    record['simulation_mass_kg'] = payload['object_model']['mass_kg']
                factory = getattr(self, '_mujoco_client_factory', original.MujocoDigitalTwinClient)
                client = factory(cfg.get('server_url', 'http://172.23.132.97:8000'),
                                 timeout_sec=float(cfg.get('timeout_sec', 20.)))
                response = client.simulate_grasp(payload)
                self._require_stream_ticket_current(ticket)
                gate = original.validate_mujoco_gate_response(response, str(plan.plan_id), cfg.get('min_score', 80),
                    expected_candidate_source=plan.candidate_source,
                    expected_candidate_source_lineage=plan.candidate_source_lineage)
                code, reason = ('OK', '') if gate.ok else (gate.code, gate.reason)
                record.update(plan_id=plan.plan_id, passed=bool(gate.ok), score=gate.score)
            except original.StreamResultCancelled:
                raise
            except Exception as exc:
                code, reason = 'WSL_UNAVAILABLE', str(exc)
        record.update(code=code, reason=reason, response=self._mujoco_response_audit(response))
        self._record_mujoco_selection_attempt(candidate, record)
        if record['passed']:
            return result
        rospy.loginfo('Unknown direct candidate rejected by MuJoCo: track=%s variant=%s code=%s reason=%s',
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
            if not self._selection_is_current():
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
            if bool(getattr(req, 'trigger', False)) and not self._selection_is_current():
                return original.TriggerZeroResponse(False, 'grasp mode reset is not ready')
            return super().request_plan_cb(req)

    def replan_execution_cb(self, req):
        with self._mode_control_lock:
            if not self._selection_is_current():
                return original.TriggerZeroResponse(False, 'grasp mode reset is not ready')
            with self._mode_work_lock:
                return super().replan_execution_cb(req)

    def _poll_stream_snapshot(self):
        with self._mode_control_lock:
            if not self._selection_is_current():
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
            if not self._selection_is_current():
                return False
            return super().submit_stream_snapshot(snapshot, **kwargs)

    def _stream_ticket_cancellation_code_locked(self, ticket):
        if not self._mode_ready:
            return 'GRASP_MODE_CHANGED'
        return super()._stream_ticket_cancellation_code_locked(ticket)

    def _prepare_and_predict(self, ticket):
        with self._mode_work_lock:
            self._require_stream_ticket_current(ticket)
            return super()._prepare_and_predict(ticket)

    def _accept_prediction(self, prepared):
        with self._mode_work_lock:
            self._require_stream_ticket_current(prepared.ticket)
            return super()._accept_prediction(prepared)

    def _build_streaming_gate_audit_report(self, prepared, selection, *args, **kwargs):
        report, summary, selected_lineage = super()._build_streaming_gate_audit_report(
            prepared, selection, *args, **kwargs)
        report['grasp_mode_selection'] = dict(self._mode_selection or {})
        report['snapshot_stamp_ns'] = int(getattr(prepared.snapshot, 'stamp_ns', 0))
        if (self._mode_selection or {}).get('mode') == 'unknown' and not self._direct_strategy_selected():
            report['observation_selection_policy'] = 'SIDE_EVIDENCE_THEN_CHECKED_HARDWARE_DURATION'
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
        return report, summary, selected_lineage

    def _publish_preview_plan(self, *args, **kwargs):
        # A worker failure can publish a terminal preview after acceptance
        # raises. Include that path in the same reset completion barrier.
        with self._mode_work_lock:
            return super()._publish_preview_plan(*args, **kwargs)


if __name__ == '__main__':
    rospy.init_node('remote_grasp6d_node')
    ModeAwareRemoteGrasp6DNode().spin()
