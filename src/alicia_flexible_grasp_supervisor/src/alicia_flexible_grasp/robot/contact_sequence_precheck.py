"""Read-only contact candidate check with the current measured following bias.

This is a constant-bias prediction at virtual future poses, not future measured
feedback. It cannot authorize execution or replace the fresh contact gateway.
"""
from copy import deepcopy
import json
import math
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest
from trajectory_msgs.msg import JointTrajectoryPoint

from alicia_flexible_grasp_supervisor.srv import CheckContactSequenceResponse
from alicia_flexible_grasp.grasp.gripper_geometry import (
    GripperGeometry, gripper_contract_mismatch_reason, ANALYTICAL_GRIPPER_MODEL_NAME)
from .contact_linear_compensation import compensate_cartesian_path
from .observation_path_guard import (
    FrozenObservationScene, ObservationPathError, SerialUrdfFk,
    validate_observation_trajectory, wire_digest)
from .pregrasp_timing import temporary_attribute
from .stationary_following import stationary_following_error, ARM_NAMES, SDK_QUANTUM_RAD as Q


class ContactPrecheckError(ObservationPathError):
    def __init__(self, message, code='CONTACT_SEQUENCE_PRECHECK_FAILED'):
        super().__init__(message)
        self.code = code


def predict_contact_command_path(nominal, fk, sample, goal):
    """Apply an observed bias to a virtual start, explicitly as a prediction."""
    if not nominal.joint_trajectory.points:
        raise ContactPrecheckError('missing nominal contact trajectory')
    measured = np.asarray(sample['accepted_positions_rad'], dtype=float)
    sdk = np.asarray(sample['sdk_positions_rad'], dtype=float)
    virtual_start = np.asarray(nominal.joint_trajectory.points[0].positions, dtype=float)
    if (measured.shape != (6,) or sdk.shape != (6,)
            or virtual_start.shape != (6,)
            or not np.all(np.isfinite(measured)) or not np.all(np.isfinite(sdk))):
        raise ContactPrecheckError('invalid measured following sample')
    projection = dict(model_sha256=sample['model_sha256'],
        joint_names=sample['joint_names'], accepted_positions_rad=virtual_start.tolist(),
        sdk_positions_rad=(virtual_start+sdk-measured).tolist())
    shifted, derivation = compensate_cartesian_path(nominal, fk, projection, goal)
    # The arithmetic helper names its inputs as measured. Do not mislabel a
    # virtual future state in the audit or pass it to an execution gateway.
    derivation['virtual_nominal_start_positions_rad'] = derivation.pop('measured_start_positions_rad')
    derivation['predicted_command_start_positions_rad'] = derivation.pop('command_start_positions_rad')
    derivation.update(scope='constant_current_bias_prediction',
                      future_following_verified=False, execution_authorized=False)
    return shifted, derivation


def contact_precheck_gripper(config):
    g = GripperGeometry(float(config.get('max_inner_gap_m', .05)),
        float(config.get('width_safety_margin_per_side_m', .002)),
        np.asarray(config.get('finger_box_xyz_m', [.0434,.0286,.06])),
        np.asarray(config.get('palm_box_xyz_m', [.1175,.155,.0774])),
        float(config.get('support_clearance_m', .003)))
    mismatch = gripper_contract_mismatch_reason(g, .05, .05,
        ANALYTICAL_GRIPPER_MODEL_NAME, .05,
        tool_jaw_axis=config.get('tool_jaw_axis', 'y'),
        tool_finger_length_axis=config.get('tool_finger_length_axis', 'z'))
    if mismatch:
        raise ContactPrecheckError('gripper contract: '+mismatch)
    return g


def validate_contact_prediction(nominal, fk, sample, goal, scene, gripper, opening,
                                planner, state, collision_check, remaining):
    """Use the same full-curve validators as execution, without caching motion."""
    shifted, evidence = predict_contact_command_path(nominal, fk, sample, goal)
    for name, trajectory in (('nominal', nominal), ('predicted_command', shifted)):
        before = deepcopy(trajectory)
        remaining()
        retimed = planner._retime_cartesian_plan(deepcopy(trajectory))
        if not planner._same_joint_path_geometry(before, retimed):
            raise ContactPrecheckError('retiming changed '+name+' path')
        hold = JointTrajectoryPoint(positions=list(retimed.joint_trajectory.points[0].positions),
                                    velocities=[0.]*6, accelerations=[0.]*6)
        try:
            report = validate_observation_trajectory(retimed, scene, fk, gripper,
                opening, hold, max_seconds=min(5., remaining()))
        except ObservationPathError as exc:
            raise ContactPrecheckError(name+' continuous geometry: '+str(exc)) from exc
        evidence[name+'_trajectory_sha256'] = wire_digest(retimed)
        evidence[name+'_geometry'] = report
        remaining()
    evidence['collision_state_checks'] = collision_check(shifted, state, deadline_check=remaining)
    remaining()
    return evidence


class ContactSequencePrecheckGateway:
    def _contact_precheck_sample(self, fk, epoch):
        with self._endpoint_feedback_lock:
            sdk = tuple(self._endpoint_feedback_history['sdk'])
            accepted = tuple(self._endpoint_feedback_history['accepted'])
        return stationary_following_error(fk, sdk, accepted,
            now_sec=rospy.get_time(), epoch_ns=epoch)

    def _contact_precheck_scene_digest(self):
        # Bind reuse to collision geometry, attached bodies, transforms, ACM
        # and padding. Robot joint feedback is bound separately below.
        components = PlanningSceneComponents()
        components.components = (components.SCENE_SETTINGS | components.ROBOT_STATE_ATTACHED_OBJECTS
            | components.WORLD_OBJECT_GEOMETRY | components.OCTOMAP | components.TRANSFORMS
            | components.ALLOWED_COLLISION_MATRIX | components.LINK_PADDING_AND_SCALING)
        rospy.wait_for_service('/get_planning_scene', timeout=.5)
        scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)(
            GetPlanningSceneRequest(components=components)).scene
        return wire_digest(scene)

    def handle_contact_sequence_precheck(self, req):
        audit = dict(scope='read_only_candidate_constant_bias_prediction',
                     execution_authorized=False, hardware_response_verified=False,
                     real_grasp_success=False, plan_id=str(req.plan.plan_id))
        metrics = {}
        failed_stage = ''
        with self._planner_operation_lock():
            try:
                plan = deepcopy(req.plan)
                if plan.model_choice != 'unknown_tabletop':
                    raise ContactPrecheckError('unknown contact plan required')
                scene = FrozenObservationScene.from_contact_plan(plan)
                now = rospy.get_time()
                requested = req.deadline.to_sec()
                if not math.isfinite(requested) or requested < 0.:
                    raise ContactPrecheckError('invalid candidate deadline')
                deadline = min(requested, now+15.) if requested > 0. else now+15.
                wall_deadline = time.monotonic() + (deadline-now)
                def remaining():
                    value = min(deadline-rospy.get_time(), wall_deadline-time.monotonic())
                    if not math.isfinite(value) or value <= 0.:
                        raise ContactPrecheckError('candidate precheck deadline consumed', 'MOVEIT_TIMEOUT')
                    return value
                remaining()
                epoch = getattr(self, '_control_reference_epoch_ns', None)
                selection = deepcopy(rospy.get_param('/grasp_mode/selection', {}))
                model = rospy.get_param('/robot_description')
                geometry = deepcopy(rospy.get_param('/grasp_6d/remote/gripper_geometry', {}))
                config = deepcopy(rospy.get_param('/grasp', {}))
                gripper_config = deepcopy(rospy.get_param('/gripper', {}))
                if selection.get('mode') != 'unknown':
                    raise ContactPrecheckError('unknown selection required')
                if not config.get('unknown_contact_cartesian_compensation_enabled', False):
                    raise ContactPrecheckError('contact compensation is not enabled')
                fk = SerialUrdfFk(model, ARM_NAMES)
                sample = self._contact_precheck_sample(fk, epoch)
                audit.update(epoch_ns=epoch, model_sha256=fk.model_sha256,
                             selection=selection, source_sample=sample)
                g = contact_precheck_gripper(geometry)
                opened = float(gripper_config.get('open_position_m', .05))
                tolerance = float(gripper_config.get('open_skip_tolerance_m', .001))
                measured_opening = float(sample['accepted_gripper_opening_m'])
                if (not math.isfinite(opened) or not 0. < opened <= .05
                        or not math.isfinite(tolerance) or not 0. <= tolerance < opened):
                    raise ContactPrecheckError('invalid configured open posture')
                # Match the existing open/skip decision. Any future opening is
                # marked predicted; execution must still use fresh encoders.
                opening = measured_opening if abs(measured_opening-opened) <= tolerance else opened
                audit.update(opening_m=opening, opening_source=(
                    'current_accepted_open_posture' if opening == measured_opening
                    else 'configured_future_open_posture_prediction'))
                planner = self._ensure_planner()
                if planner is None:
                    raise ContactPrecheckError(self._moveit_not_ready_message(), 'MOVEIT_CHECK_ERROR')
                start = deepcopy(planner.robot.get_current_state())
                if not set(ARM_NAMES+('right_finger',)).issubset(start.joint_state.name):
                    raise ContactPrecheckError('incomplete current robot state')
                values = dict(zip(ARM_NAMES, sample['accepted_positions_rad']), right_finger=opening)
                start.joint_state.position = [values.get(n, v) for n, v in
                    zip(start.joint_state.name, start.joint_state.position)]
                start.is_diff = False
                names = ('pregrasp', 'approach', 'grasp', 'lift')
                if config.get('unknown_lift_after_close_enabled', True) is False:
                    names = names[:3]
                audit['checked_stage_names'] = list(names)
                targets = []
                for pose in plan.poses[:len(names)]:
                    target = PoseStamped(); target.header.frame_id = 'base_link'
                    target.pose = deepcopy(pose); targets.append(target)
                def validate(stage, state, path, target):
                    remaining()
                    if stage == 'grasp':
                        audit['contact_prediction'] = validate_contact_prediction(
                            path, fk, sample, target.pose, scene, g, opening, planner, state,
                            self._contact_command_collision_check, remaining)
                scene_digest = self._contact_precheck_scene_digest()
                remaining()
                # Publication rebuilds the same selected candidate. Reuse only
                # an exact success while both independent feedback channels,
                # opening, epoch, frozen target and MoveIt scene are unchanged.
                # This cache stores predictions, never executable trajectories.
                cache_key = json.dumps(dict(plan_digest=scene.digest, model=fk.model_sha256,
                    epoch=epoch, selection=selection, geometry=geometry, config=config,
                    gripper_config=gripper_config, sdk=sample['sdk_positions_rad'],
                    accepted=sample['accepted_positions_rad'], opening=measured_opening,
                    scene_digest=scene_digest), sort_keys=True, allow_nan=False)
                cache = getattr(self, '_contact_precheck_cache', {})
                entry = cache.get(cache_key)
                cached = bool(entry and 0. <= time.monotonic()-entry['time'] <= 15.)
                if cached:
                    metrics = deepcopy(entry['metrics'])
                    message = entry['message']
                    audit['contact_prediction'] = deepcopy(entry['prediction'])
                else:
                    profile = self._unknown_pregrasp_velocity_limits(planner)
                    with temporary_attribute(planner, 'unknown_pregrasp_joint_velocity_limits_rad_s', profile):
                        ok, code, failed_stage, metrics, message = planner.check_pose_sequence(
                            targets, names, [False]+[True]*(len(names)-1), deadline_sec=deadline,
                            stage_validator=validate, start_state_override=start)
                    if not ok:
                        raise ContactPrecheckError(message, code)
                audit.update(prediction_cache_hit=cached, planning_scene_digest=scene_digest)
                if 'contact_prediction' not in audit:
                    raise ContactPrecheckError('contact validator was not executed')
                remaining()
                current = self._contact_precheck_sample(fk, epoch)
                if (epoch != getattr(self, '_control_reference_epoch_ns', None)
                        or selection != rospy.get_param('/grasp_mode/selection', {})
                        or model != rospy.get_param('/robot_description')
                        or geometry != rospy.get_param('/grasp_6d/remote/gripper_geometry', {})
                        or config != rospy.get_param('/grasp', {})
                        or gripper_config != rospy.get_param('/gripper', {})):
                    raise ContactPrecheckError('candidate context changed while planning')
                for key, tolerance in (('sdk_positions_rad', Q/2), ('accepted_positions_rad', Q)):
                    if max(abs(a-b) for a,b in zip(current[key], sample[key])) > tolerance+1e-12:
                        raise ContactPrecheckError('following feedback changed while planning')
                if abs(current['accepted_gripper_opening_m']-measured_opening) > 1e-9:
                    raise ContactPrecheckError('gripper changed while planning')
                remaining()
                if self._contact_precheck_scene_digest() != scene_digest:
                    raise ContactPrecheckError('MoveIt scene changed while planning')
                remaining()
                if not cached:
                    cache[cache_key] = dict(time=time.monotonic(), metrics=deepcopy(metrics),
                        message=message, prediction=deepcopy(audit['contact_prediction']))
                    while len(cache) > 16:
                        del cache[next(iter(cache))]
                    self._contact_precheck_cache = cache
                audit.update(success=True, completion_sample=current)
                return CheckContactSequenceResponse(True, '', '', metrics['path_cost'],
                    metrics['max_delta'], message, json.dumps(audit, sort_keys=True))
            except Exception as exc:
                code = str(getattr(exc, 'code', 'CONTACT_SEQUENCE_PRECHECK_FAILED'))
                audit.update(success=False, failure_code=code, failed_stage=failed_stage, reason=str(exc))
                return CheckContactSequenceResponse(False, code, failed_stage,
                    float(metrics.get('path_cost', 0.)), float(metrics.get('max_delta', 0.)),
                    str(exc), json.dumps(audit, sort_keys=True))
