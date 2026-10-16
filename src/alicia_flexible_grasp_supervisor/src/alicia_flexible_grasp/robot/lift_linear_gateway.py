"""Default-disabled, plan-bound lift route through the contracted executor.

Uses real request-local MoveIt checks and continuous commanded geometry.
Missing optional camera/mount geometry is reported as incomplete coverage,
never invented. Enabling this route never silently falls back to the old path.
"""
from copy import deepcopy
import json
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped

from alicia_flexible_grasp_supervisor.srv import CompensatePregraspResponse
from alicia_flexible_grasp.grasp.grasp_state_machine import GraspStages
from alicia_flexible_grasp.grasp.gripper_geometry import (
    GripperGeometry, gripper_contract_mismatch_reason, ANALYTICAL_GRIPPER_MODEL_NAME)
from .contracted_trajectory import ContractedTrajectoryExecutor
from .lift_moveit_adapter import ClosedLiftMoveItAdapter, bound_lift_action_goal, KIND, POLICY
from .lift_path_continuity import (
    FrozenLiftContext, ToolBodyBox, derive_lift_path, validate_lift_pair)
from .observation_path_guard import (
    FrozenObservationScene, ObservationPathError, SerialUrdfFk, committed_plan_digest, wire_digest)
from .pregrasp_compensation import PregraspCompensation, pose_matrix
from .stationary_following import ARM_NAMES
from alicia_flexible_grasp.grasp.rich_plan_integrity import pose_values

ENABLED_KEY = 'unknown_lift_cartesian_compensation_enabled'
BODY_PARAM = '/robot/closed_lift_tool_bodies'
LABELS = ('planning linear 6D lift', 'moving linear 6D lift')


class LiftLinearCompensationGateway:
    def _validate_lift_context(self, context):
        with self._observation_context_lock:
            self._require_live_observation_task()
            if (self._manual_control_active()
                    or getattr(self, '_pregrasp_task_stage', None) != GraspStages.LIFT_OBJECT
                    or getattr(self, '_compensation_task_label', '') not in LABELS
                    or self._control_reference_epoch_ns != context['epoch']
                    or self._observation_contexts.revocation != context['revocation']):
                raise ObservationPathError('LIFT_AUTHORITY_CHANGED')
        cfg = context['config']
        if (not isinstance(context['selection'],dict)
                or context['selection'].get('mode') != 'unknown'
                or context['selection'].get('strategy') not in ('direct','observe_return')
                or cfg.get(ENABLED_KEY) is not True
                or cfg.get('measured_endpoint_check_enabled') is not True
                or rospy.get_param('/grasp', {}) != cfg
                or rospy.get_param('/grasp_mode/selection', {}) != context['selection']
                or rospy.get_param('/robot/observation_tracking_contract_enabled', False) is not True
                or rospy.get_param('/alicia_d_driver_node/endpoint_feedback_trim_enabled', False) is not False):
            raise ObservationPathError('LIFT_CONFIGURATION_DISABLED_OR_CHANGED')
        if (context['plan'].model_choice != 'unknown_tabletop'
                or type(context['epoch']) is not int or context['epoch'] <= 0
                or not context['epoch'] <= context['scene'].source_ns <= int(rospy.get_time()*1e9)
                or rospy.get_param('/robot_description', '') != context['model']
                or committed_plan_digest(context['plan']) != context['scene'].digest
                or rospy.get_param('/grasp_6d/remote/gripper_geometry', {}) != context['geometry']
                or rospy.get_param(BODY_PARAM, {}) != context['body_config']
                or time.monotonic() >= context['deadline']):
            raise ObservationPathError('LIFT_FROZEN_CONTEXT_CHANGED_OR_EXPIRED')
        if not self._fresh_actuation_status().startswith('CONFIRMED:'):
            raise ObservationPathError('LIFT_ACTUATION_UNCONFIRMED')

    def _lift_context(self, plan):
        # Task message and RPC are different transports: wait only on initial
        # admission, never relax phase/ownership at submission.
        until = time.monotonic()+1.
        while (getattr(self, '_pregrasp_task_stage', None) != GraspStages.LIFT_OBJECT
               or getattr(self, '_compensation_task_label', '') not in LABELS):
            self._require_live_observation_task()
            if self._manual_control_active() or time.monotonic() >= until or rospy.is_shutdown():
                raise ObservationPathError('LIFT_TASK_PHASE_UNAVAILABLE')
            rospy.sleep(.02)
        with self._observation_context_lock:
            context = dict(plan=deepcopy(plan), scene=FrozenObservationScene.from_contact_plan(plan),
                epoch=self._control_reference_epoch_ns, revocation=self._observation_contexts.revocation,
                model=rospy.get_param('/robot_description', ''),
                config=deepcopy(rospy.get_param('/grasp', {})),
                selection=deepcopy(rospy.get_param('/grasp_mode/selection', {})),
                geometry=deepcopy(rospy.get_param('/grasp_6d/remote/gripper_geometry', {})),
                body_config=deepcopy(rospy.get_param(BODY_PARAM, {})),
                admitted_ns=int(rospy.get_time()*1e9),
                deadline=time.monotonic()+PregraspCompensation.MAX_SECONDS, episode_kind='closed_lift')
            self._validate_lift_context(context)
            self._compensation_visual_context = context
            return context

    def _compensation_allows_visual_loss(self, msg):
        context = getattr(self, '_compensation_visual_context', None)
        if not isinstance(context, dict) or context.get('episode_kind') != 'closed_lift':
            return super()._compensation_allows_visual_loss(msg)
        if (msg.valid is not False or str(msg.diagnostic).split(':',1)[0].strip() != 'TARGET_LOST'
                or int(msg.header.stamp.to_nsec()) < context['scene'].source_ns
                or msg.header.frame_id != context['plan'].header.frame_id
                or msg.target_track_id not in ('',context['scene'].track_id)):
            return False
        try:
            self._validate_lift_context(context)
            return True
        except Exception:
            return False

    def _lift_sample(self, fk, context):
        sample = deepcopy(self._endpoint_following_snapshot(fk, context['epoch'], context,
            after_ns=context['admitted_ns'], context_validator=lambda:self._validate_lift_context(context)))
        if (sample.get('measurement_kind') != 'stationary_sdk_following_not_executed_endpoint'
                or sample.get('opening_stamp_ns') != sample.get('accepted_stamp_ns')
                or 'accepted_gripper_opening_m' not in sample
                or sample.get('epoch_ns') != context['epoch']
                or sample.get('model_sha256') != fk.model_sha256):
            raise ObservationPathError('LIFT_ACCEPTED_OPENING_SOURCE_MISMATCH')
        sample['accepted_opening_m'] = sample['accepted_gripper_opening_m']
        return sample

    @staticmethod
    def _lift_gripper(context):
        cfg = context['geometry']
        g = GripperGeometry(float(cfg.get('max_inner_gap_m', .05)),
            float(cfg.get('width_safety_margin_per_side_m', .002)),
            np.asarray(cfg.get('finger_box_xyz_m', [.0434,.0286,.06])),
            np.asarray(cfg.get('palm_box_xyz_m', [.1175,.155,.0774])),
            float(cfg.get('support_clearance_m', .003)))
        mismatch = gripper_contract_mismatch_reason(g,.05,.05,ANALYTICAL_GRIPPER_MODEL_NAME,.05,
            tool_jaw_axis=cfg.get('tool_jaw_axis','y'),tool_finger_length_axis=cfg.get('tool_finger_length_axis','z'))
        if mismatch:
            raise ObservationPathError('LIFT_GRIPPER_CONTRACT: '+mismatch)
        return g

    def _lift_required_capabilities(self, context):
        body_config = context['body_config']
        bodies, missing = [], []
        for name in ('camera','mount'):
            item = body_config.get(name) if isinstance(body_config,dict) else None
            if item is None:
                missing.append(name+'_collision_model_in_runtime_tool0')
                continue
            if not isinstance(item,dict) or item.get('frame_id') != 'tool0':
                raise ObservationPathError('LIFT_RIGID_BODY_FRAME_INVALID: '+name)
            bodies.append(ToolBodyBox(name,item['T_tool_box'],item['size_xyz_m'],item['source_sha256']))
        # Camera/mount CAD was not a historical lift hard gate. Missing
        # models remain explicit diagnostics; provided malformed models fail.
        provider = getattr(self, '_closed_lift_contract_validator', None)
        if provider is None:
            entry = getattr(self, '_closed_lift_moveit_adapter', None)
            if entry is None or entry[0] != context['model']:
                entry = (context['model'], ClosedLiftMoveItAdapter.connect(context['model']))
                self._closed_lift_moveit_adapter = entry
            provider = entry[1]
        if not callable(provider):
            raise ObservationPathError('LIFT_MOVEIT_ADAPTER_UNAVAILABLE')
        return tuple(bodies), provider

    def _lift_prove(self, context, fk, sample, nominal, shifted, start, hold, bodies, provider):
        self._validate_lift_context(context)
        opening = sample['accepted_opening_m']
        frozen = FrozenLiftContext.from_plan(context['plan'],fk,sample,opening)
        derived,_,_ = derive_lift_path(nominal,context['plan'],fk,sample,opening)
        if wire_digest(derived) != wire_digest(shifted):
            raise ObservationPathError('LIFT_COMPLETE_PATH_BIAS_CHANGED')
        pure = validate_lift_pair(nominal,shifted,context['plan'],frozen,fk,sample,opening,
                                  self._lift_gripper(context),bodies)
        constraints = deepcopy(rospy.get_param('/alicia_controller/constraints',None))
        stop = rospy.get_param('/alicia_controller/stop_trajectory_duration',None)
        # The provider has no execute callback and receives immutable copies.
        # A scalar MoveIt endpoint result cannot stand in for this certificate.
        audit = provider(nominal=deepcopy(nominal),shifted=deepcopy(shifted),
            frozen=frozen,start_state=deepcopy(start),sdk_hold=deepcopy(hold),
            bodies=bodies,gripper=self._lift_gripper(context),
            constraints=deepcopy(constraints),stop_duration=stop)
        expected = dict(trajectory_sha256=wire_digest(shifted),
            nominal_trajectory_sha256=wire_digest(nominal),scene_sha256=frozen.scene.digest,
            model_sha256=fk.model_sha256,epoch_ns=frozen.epoch_ns,opening_width_m=opening,
            kind=KIND,whole_arm_collision_method='sampled_exact_Noetic_curve_two_request_local_hypotheses',
            hardware_following_swept_volume_certified=False,hardware_stopping_swept_volume_certified=False,
            includes_sdk_bridge=True,stationary_world_target_removed=False,attachment_confirmed=False,
            controller_constraints_snapshot=constraints)
        if not isinstance(audit,dict) or any(audit.get(k) != v for k,v in expected.items()):
            raise ObservationPathError('LIFT_WHOLE_ARM_CERTIFICATE_MISSING_OR_UNBOUND')
        audit = deepcopy(audit)
        audit.update(pure_lift_geometry=pure,gripper_geometry_snapshot=context['geometry'],
            body_geometry_snapshot=context['body_config'],motion_authorized=False)
        # Preserve configured controller path/goal limits; do not claim the
        # observation following-error-box proof for a contact/liftoff phase.
        bound_lift_action_goal(shifted,audit,constraints,stop,reference=hold)
        self._validate_lift_context(context)
        return audit

    @staticmethod
    def _lift_same_feedback(sample, original):
        return (all(np.array_equal(sample[k],original[k]) for k in
                    ('sdk_positions_rad','accepted_positions_rad'))
                and sample['accepted_opening_m'] == original['accepted_opening_m'])

    def _lift_submission(self, context, cached, fk, trajectory, audit):
        self._validate_lift_context(context)
        if (wire_digest(trajectory) != audit['trajectory_sha256']
                or rospy.get_param('/alicia_controller/constraints',None) != audit['controller_constraints_snapshot']
                or rospy.get_param('/alicia_controller/stop_trajectory_duration',None) != audit['execution_tracking_contract']['stop_trajectory_duration_sec']
                or rospy.get_param(BODY_PARAM,{}) != audit['body_geometry_snapshot']):
            raise ObservationPathError('LIFT_SUBMISSION_CERTIFICATE_CHANGED')
        provider = cached['provider']
        if not callable(getattr(provider,'scene_digest',None)):
            raise ObservationPathError('LIFT_MOVEIT_SCENE_REVALIDATOR_UNAVAILABLE')
        if provider.scene_digest() != audit.get('moveit_scene_sha256'):
            raise ObservationPathError('LIFT_MOVEIT_SCENE_CHANGED_BEFORE_SEND')
        sample = self._lift_sample(fk,context)
        if not self._lift_same_feedback(sample,cached['sample']):
            raise ObservationPathError('LIFT_FEEDBACK_CHANGED_REPLAN_REQUIRED')
        hold = self._observation_controller_hold(ARM_NAMES)
        if not np.array_equal(hold.positions,sample['sdk_positions_rad']):
            raise ObservationPathError('LIFT_SDK_HOLD_CHANGED')
        # Sampling may wait for a new stationary window. Recheck the scene
        # AFTER that wait, immediately before the existing before-send guard.
        if provider.scene_digest() != audit.get('moveit_scene_sha256'):
            raise ObservationPathError('LIFT_MOVEIT_SCENE_CHANGED_DURING_SUBMISSION')
        self._validate_lift_context(context)
        now=float(rospy.get_time())
        if not all(0. <= now-sample[key]*1e-9 <= .5 for key in ('sdk_stamp_ns','accepted_stamp_ns')):
            raise ObservationPathError('LIFT_FEEDBACK_EXPIRED_DURING_SCENE_CHECK')

    def handle_contact_lift(self, req):
        with self._planner_operation_lock():
            planner,context,saved = None,None,{}
            diagnostic = dict(code='LIFT_LINEAR_REJECTED',plan_id=str(getattr(req.plan,'plan_id','')),
                execute=bool(req.execute),motion_authorized=False,attachment_confirmed=False)
            try:
                # No planner, controller startup or publisher access when off.
                if rospy.get_param('/grasp',{}).get(ENABLED_KEY) is not True:
                    raise ObservationPathError('LIFT_COMPENSATION_DISABLED')
                if not req.execute:
                    old = getattr(self,'_lift_linear_cache',None);self._lift_linear_cache=None
                    if old:self._release_contact_visual_context(old['context'])
                    context = self._lift_context(deepcopy(req.plan))
                else:
                    cached = getattr(self,'_lift_linear_cache',None);self._lift_linear_cache=None
                    context = cached['context'] if cached else None
                    if cached is None or committed_plan_digest(req.plan) != context['scene'].digest:
                        raise ObservationPathError('LIFT_CACHE_MISMATCH')
                self._validate_lift_context(context)
                planner = self._ensure_planner()
                if planner is None:raise ObservationPathError(self._moveit_not_ready_message())
                fk = SerialUrdfFk(context['model'],ARM_NAMES)
                sample = self._lift_sample(fk,context)
                goal = PoseStamped();goal.header.frame_id='base_link';goal.pose=deepcopy(req.plan.poses[3])
                if not req.execute:
                    planner._last_pose_plan=None
                    start = deepcopy(planner.robot.get_current_state())
                    values=dict(zip(ARM_NAMES,sample['accepted_positions_rad']))
                    values.update(right_finger=sample['accepted_opening_m'],left_finger=-sample['accepted_opening_m'])
                    if not set(ARM_NAMES+('right_finger',)).issubset(start.joint_state.name):
                        raise ObservationPathError('LIFT_START_STATE_INCOMPLETE')
                    start.joint_state.position=[values.get(n,v) for n,v in zip(start.joint_state.name,start.joint_state.position)]
                    start.is_diff=False
                    if np.linalg.norm(fk(sample['accepted_positions_rad'])[:3,3]-pose_matrix(pose_values(goal.pose))[:3,3]) > .08:
                        raise ObservationPathError('LIFT_CARTESIAN_SEGMENT_TOO_LONG')
                    nominal,message=planner._plan_cartesian_from_start_state(start,goal)
                    if nominal is None:raise ObservationPathError(message)
                    original=deepcopy(nominal);nominal=planner._retime_cartesian_plan(nominal)
                    if not planner._same_joint_path_geometry(original,nominal):
                        raise ObservationPathError('LIFT_RETIMING_CHANGED_NOMINAL_PATH')
                    shifted,frozen,evidence=derive_lift_path(nominal,context['plan'],fk,sample,sample['accepted_opening_m'])
                    diagnostic['derivation']=evidence
                    bodies,provider=self._lift_required_capabilities(context)
                    hold=deepcopy(shifted.joint_trajectory.points[0])
                    hold.positions=sample['sdk_positions_rad'];hold.velocities=[0.]*6;hold.accelerations=[0.]*6
                    audit=self._lift_prove(context,fk,sample,nominal,shifted,start,hold,bodies,provider)
                    self._lift_linear_cache=dict(context=context,sample=sample,nominal=deepcopy(nominal),
                        trajectory=deepcopy(shifted),start=start,bodies=bodies,provider=provider,audit=audit)
                    return CompensatePregraspResponse(True,json.dumps(dict(diagnostic,
                        code='LIFT_LINEAR_PLANNED',proof=audit),sort_keys=True))
                if not self._lift_same_feedback(sample,cached['sample']):
                    raise ObservationPathError('LIFT_FEEDBACK_CHANGED_REPLAN_REQUIRED')
                bodies,provider=self._lift_required_capabilities(context)
                if provider != cached['provider']:
                    raise ObservationPathError('LIFT_CERTIFICATE_PROVIDER_CHANGED')
                names=('observation_path_guard_required','observation_tracking_contract_required',
                       'observation_path_validator','observation_execution_authorized','observation_submission_revalidate',
                       'observation_action_goal_builder')
                saved={n:(hasattr(planner,n),getattr(planner,n,None)) for n in names}
                planner.observation_path_guard_required=True
                planner.observation_tracking_contract_required=True
                planner.observation_action_goal_builder=bound_lift_action_goal
                def guard(trajectory):
                    fresh=self._lift_sample(fk,context)
                    if not self._lift_same_feedback(fresh,cached['sample']):
                        raise ObservationPathError('LIFT_FEEDBACK_CHANGED_REPLAN_REQUIRED')
                    if not planner._same_joint_path_geometry(cached['trajectory'],trajectory):
                        raise ObservationPathError('LIFT_COMMAND_POSITIONS_CHANGED')
                    hold=self._observation_controller_hold(ARM_NAMES)
                    if not np.array_equal(hold.positions,fresh['sdk_positions_rad']):
                        raise ObservationPathError('LIFT_SDK_HOLD_CHANGED')
                    # Controller-reference timing may change derivatives/times.
                    # Reconstruct its complete nominal counterpart, not t=0.
                    nominal=deepcopy(trajectory)
                    for p,original in zip(nominal.joint_trajectory.points,cached['nominal'].joint_trajectory.points):
                        p.positions=list(original.positions)
                    if time.monotonic()+trajectory.joint_trajectory.points[-1].time_from_start.to_sec()+2 >= context['deadline']:
                        raise ObservationPathError('LIFT_EXECUTION_TIME_BUDGET')
                    return self._lift_prove(context,fk,fresh,nominal,trajectory,cached['start'],hold,bodies,provider)
                planner.observation_path_validator=guard
                def authorized():
                    try:self._validate_lift_context(context);return True
                    except Exception:return False
                planner.observation_execution_authorized=authorized
                planner.observation_submission_revalidate=lambda tr,audit:self._lift_submission(context,cached,fk,tr,audit)
                # Only this explicit execute request can reach the existing
                # contracted transport with the final path guard installed.
                if getattr(planner,'observation_trajectory_executor',None) is None:
                    planner.observation_trajectory_executor=ContractedTrajectoryExecutor.connect()
                ok,message=self._ensure_trajectory_controllers_started()
                if not ok:raise ObservationPathError(message)
                self._validate_lift_context(context)
                planner._remember_pose_plan(goal.pose,deepcopy(cached['trajectory']),'cartesian')
                ok,message=planner._execute_cached_pose_plan(goal.pose,planner._pose_xyz_text(goal.pose),
                    required_kind='cartesian',before_execute=lambda:self._validate_lift_context(context))
                return CompensatePregraspResponse(ok,message)
            except Exception as exc:
                self._lift_linear_cache=None;diagnostic['reason']=str(exc)
                rospy.logwarn('Lift continuity rejected: %s',json.dumps(diagnostic,sort_keys=True))
                return CompensatePregraspResponse(False,json.dumps(diagnostic,sort_keys=True))
            finally:
                retained=getattr(self,'_lift_linear_cache',None)
                if req.execute or retained is None or retained['context'] is not context:
                    self._release_contact_visual_context(context)
                if planner is not None:
                    if req.execute:planner._last_pose_plan=None
                    for name,(existed,value) in saved.items():
                        if existed:setattr(planner,name,value)
                        elif hasattr(planner,name):delattr(planner,name)
