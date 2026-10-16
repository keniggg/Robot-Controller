"""One task-bound observation correction before arrival measurement begins."""
from copy import deepcopy
import hashlib
import json
import math
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped

from alicia_flexible_grasp_supervisor.srv import CompensatePregraspResponse
from alicia_flexible_grasp.grasp.grasp_state_machine import GraspStages
from alicia_flexible_grasp.grasp.rich_plan_integrity import (
    pose_values, is_silhouette_observation_plan)
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask
from .observation_path_guard import FrozenObservationScene, ObservationPathError, SerialUrdfFk
from .observation_view_gateway import _camera_configuration_contract
from .observation_endpoint_compensation import (
    ObservationEndpointCompensation, validate_observation_correction_path_bounds)
from .contact_sequence_precheck import contact_precheck_gripper, validate_selected_pregrasp_path
from .endpoint_correction import sdk_counts
from .stationary_following import ARM_NAMES, SDK_QUANTUM_RAD as Q


OBSERVATION_COMPENSATION_LABEL = 'correcting observation endpoint'


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


class ObservationCompensationGateway:
    def _observation_compensation_measurement_cb(self, message):
        # No planner lock: revocation must be visible while planning runs.
        try:
            raw = json.loads(message.data)
            if not isinstance(raw, dict) or type(raw.get('stamp_ns')) is not int:
                raise ValueError('invalid measurement receipt')
        except (TypeError, ValueError):
            with self._observation_context_lock:
                self._observation_compensation_receipt = None
            return
        with self._observation_context_lock:
            previous = getattr(self, '_observation_compensation_receipt', None)
            watermark = getattr(self, '_observation_compensation_receipt_stamp_ns', 0)
            if raw['stamp_ns'] < watermark:
                return
            if raw['stamp_ns'] == watermark:
                if previous != raw:
                    self._observation_compensation_receipt = None
                return
            self._observation_compensation_receipt_stamp_ns = raw['stamp_ns']
            self._observation_compensation_receipt = deepcopy(raw)

    def _observation_compensation_context(self, plan):
        # Optional grasp_modes dependency only for this explicitly opted-in path.
        from alicia_grasp_modes.observe_return_measurement import parse_measurement_request
        scene = FrozenObservationScene.from_plan(plan)
        if (plan.model_choice != 'unknown_tabletop' or is_silhouette_observation_plan(plan)):
            raise ObservationPathError('OBSERVATION_COMPENSATION_ORDINARY_PLAN_REQUIRED')
        origin = getattr(self, '_observe_return_origin', None)
        if origin is None:
            raise ObservationPathError('OBSERVATION_COMPENSATION_ORIGIN_MISSING')
        self._validate_observe_return_context(origin, require_stage=False)
        if origin['config'].get('observation_endpoint_compensation_enabled') is not True:
            raise ObservationPathError('OBSERVATION_COMPENSATION_DISABLED')
        session = deepcopy(origin.get('view_session'))
        if not isinstance(session, dict) or session.get('phase') != 'observing':
            raise ObservationPathError('OBSERVATION_COMPENSATION_SESSION_REQUIRED')
        now = rospy.get_time()
        deadline = min(time.monotonic()+60., origin['deadline'],
                       time.monotonic()+session['deadline_ns']*1e-9-now)
        admission = min(deadline, time.monotonic()+1.)
        while True:
            self._validate_observe_return_context(origin, require_stage=False)
            with self._observation_context_lock:
                raw = deepcopy(getattr(self, '_observation_compensation_receipt', None))
                stage_ready = (getattr(self, '_pregrasp_task_stage', None) == GraspStages.MOVE_PREGRASP
                    and getattr(self, '_compensation_task_label', '') == OBSERVATION_COMPENSATION_LABEL)
            if raw is not None and raw.get('view_plan_id') == scene.plan_id and stage_ready:
                break
            if time.monotonic() >= admission:
                raise ObservationPathError('OBSERVATION_COMPENSATION_HANDOFF_NOT_RECEIVED')
            rospy.sleep(min(.02, max(0., admission-time.monotonic())))
        receipt = parse_measurement_request(raw, origin['selection'], rospy.Time.now().to_nsec())
        active = origin.get('active_view_scene', origin['scene'])
        if (receipt['state'] != 'executing' or receipt['motion_kind'] != 'observation'
                or receipt['plan_sha256'] != scene.digest or active.digest != scene.digest
                or receipt['view_source_stamp_ns'] != scene.source_ns
                or receipt['control_epoch_ns'] != origin['epoch']
                or any(receipt.get(k) != session.get(k) for k in (
                    'session_id', 'origin_plan_id', 'origin_snapshot_stamp_ns', 'view_index',
                    'view_plan_id', 'view_source_stamp_ns', 'target_epoch', 'target_track_id', 'deadline_ns'))
                or scene.track_id != receipt['target_track_id']):
            raise ObservationPathError('OBSERVATION_COMPENSATION_HANDOFF_BINDING_CHANGED')
        configuration = self._unknown_observation_view_configuration(scene)
        context = dict(origin=origin, session=session, receipt=raw,
            receipt_digest=_digest(raw), plan=deepcopy(plan), scene=scene,
            revocation=origin['revocation'], epoch=origin['epoch'], deadline=deadline,
            configuration=configuration, model=origin['model'],
            geometry=deepcopy(rospy.get_param('/grasp_6d/remote/gripper_geometry', {})))
        self._validate_observation_compensation(context)
        return context

    def _validate_observation_compensation(self, context):
        self._validate_observe_return_context(context['origin'], require_stage=False)
        with self._observation_context_lock:
            if (getattr(self, '_observe_return_origin', None) is not context['origin']
                    or context['origin'].get('view_session') != context['session']
                    or context['origin'].get('active_view_scene',context['origin']['scene']).digest != context['scene'].digest
                    or _digest(getattr(self, '_observation_compensation_receipt', None)) != context['receipt_digest']
                    or getattr(self, '_pregrasp_task_stage', None) != GraspStages.MOVE_PREGRASP
                    or getattr(self, '_compensation_task_label', '') != OBSERVATION_COMPENSATION_LABEL):
                raise ObservationPathError('OBSERVATION_COMPENSATION_OWNER_CHANGED')
        if time.monotonic() >= context['deadline'] or rospy.Time.now().to_nsec() >= context['receipt']['deadline_ns']:
            raise ObservationPathError('OBSERVATION_COMPENSATION_EXPIRED')
        current = self._unknown_observation_view_configuration(context['scene'])
        comparable = lambda c: {k:v for k,v in c.items() if k != 'camera_configuration_snapshot'}
        if (comparable(current) != comparable(context['configuration'])
                or _camera_configuration_contract(context['configuration']['camera_configuration_snapshot'])
                   != context['configuration']['camera_configuration_contract']
                or rospy.get_param('/grasp_6d/remote/gripper_geometry', {}) != context['geometry']):
            raise ObservationPathError('OBSERVATION_COMPENSATION_CONFIGURATION_CHANGED')

    @staticmethod
    def _observation_correction_start(state, q, opening):
        state = deepcopy(state)
        if not set(ARM_NAMES+('right_finger',)).issubset(state.joint_state.name):
            raise ObservationPathError('OBSERVATION_COMPENSATION_START_INCOMPLETE')
        values = dict(zip(ARM_NAMES,q)); values.update(right_finger=opening,left_finger=-opening)
        state.joint_state.position = [values.get(n,v) for n,v in zip(state.joint_state.name,state.joint_state.position)]
        state.is_diff = False
        return state

    def handle_compensate_observation(self, req):
        audit = dict(physical_motion_submitted=False, real_grasp_success=False,
                     complete_execution_path_checked=False, arrival_measurement_required=True)
        planner, saved, context = None, {}, None
        with self._planner_operation_lock():
            try:
                if type(req.execute) is not bool:
                    raise ObservationPathError('OBSERVATION_COMPENSATION_EXECUTE_INVALID')
                context = self._observation_compensation_context(deepcopy(req.plan))
                receipt = context['receipt']; scene = context['scene']
                audit.update(plan_id=scene.plan_id, plan_digest=scene.digest,
                    control_epoch_ns=context['epoch'], session_id=receipt['session_id'],
                    lease_id=receipt['lease_id'], view_index=receipt['view_index'],
                    fixed_goal_pose=list(pose_values(context['plan'].poses[0])))
                validate = lambda:self._validate_observation_compensation(context)
                def remaining():
                    validate()
                    return context['deadline']-time.monotonic()
                fk = SerialUrdfFk(context['model'], ARM_NAMES)
                sample = self._endpoint_following_snapshot(fk, context['epoch'], context,
                                                          context_validator=validate)
                camera = context['configuration']['camera']
                valid = rectification_rgb_valid_mask(context['configuration']['camera_configuration_snapshot'],
                                                    (camera['height'],camera['width']))
                if valid is None:
                    valid = np.ones((camera['height'],camera['width']),dtype=bool)
                correction = ObservationEndpointCompensation(fk,sample,context['plan'].poses[0],
                    scene,context['configuration'],valid)
                def sample_fresh(value):
                    validate()
                    now = rospy.get_time()
                    if any(not 0 <= now-value[k]*1e-9 <= .5 for k in ('sdk_stamp_ns','accepted_stamp_ns')):
                        raise ObservationPathError('OBSERVATION_COMPENSATION_FEEDBACK_EXPIRED')
                    if tuple(sdk_counts(self._observe_return_reference())) != tuple(sdk_counts(value['sdk_positions_rad'])):
                        raise ObservationPathError('OBSERVATION_COMPENSATION_REFERENCE_CHANGED')
                sample_fresh(sample)
                view = correction.current_view(sample)
                if view is not None:
                    # No cached false-call evidence and no claim of path execution.
                    sample_fresh(sample)
                    audit.update(code='OBSERVATION_ENDPOINT_READY', no_motion=True,
                        completion_stamp_ns=rospy.Time.now().to_nsec(), accepted_stamp_ns=sample['accepted_stamp_ns'],
                        view_audit=view)
                    return CompensatePregraspResponse(True,json.dumps(audit,allow_nan=False))
                consumed = '/supervisor/observation_compensation_episodes/'+_digest(
                    [receipt['lease_id'],scene.digest,context['epoch']])
                if rospy.get_param(consumed,None) is not None:
                    raise ObservationPathError('OBSERVATION_COMPENSATION_EPISODE_CONSUMED')
                planner = self._ensure_planner()
                if planner is None:
                    raise ObservationPathError(self._moveit_not_ready_message())
                opening = float(sample['accepted_gripper_opening_m'])
                start = self._observation_correction_start(planner.robot.get_current_state(),
                    sample['accepted_positions_rad'],opening)
                goal = PoseStamped(header=deepcopy(context['plan'].header),pose=deepcopy(context['plan'].poses[0]))
                remaining()
                nominal, reason = planner._plan_cartesian_from_start_state(start,goal)
                remaining()
                if nominal is None:
                    raise ObservationPathError('OBSERVATION_COMPENSATION_NOMINAL_PATH_FAILED: '+str(reason))
                step, proposal = correction.propose(nominal)
                audit['proposal'] = proposal
                target = list(step.positions)
                gripper = contact_precheck_gripper(context['geometry'])
                def fresh_start():
                    remaining()
                    fresh = self._endpoint_following_snapshot(fk,context['epoch'],context,
                                                              context_validator=validate)
                    sample_fresh(fresh)
                    if (max(abs(np.asarray(fresh['accepted_positions_rad'])-sample['accepted_positions_rad'])) > Q+1e-12
                            or tuple(sdk_counts(fresh['sdk_positions_rad'])) != step.baseline_counts
                            or fresh['accepted_gripper_opening_m'] != opening):
                        raise ObservationPathError('OBSERVATION_COMPENSATION_START_CHANGED')
                    return fresh
                def path_guard(path):
                    fresh = fresh_start()
                    bounds = validate_observation_correction_path_bounds(path,sample['sdk_positions_rad'],target)
                    state = self._observation_correction_start(planner.robot.get_current_state(),
                        fresh['accepted_positions_rad'],opening)
                    audit['path_prediction'] = validate_selected_pregrasp_path(path,fk,fresh,scene,gripper,
                        opening,planner,state,self._contact_command_collision_check,remaining,
                        command_path_validator=lambda tr:validate_observation_correction_path_bounds(
                            tr,sample['sdk_positions_rad'],target))
                    result = self._validate_frozen_observation_path((scene,context['revocation']),path,planner,
                                                                  context_validator=validate)
                    duration = path.joint_trajectory.points[-1].time_from_start.to_sec()
                    if duration+float(result['controller_constraints_snapshot']['goal_time'])+2. >= remaining():
                        raise ObservationPathError('OBSERVATION_COMPENSATION_EXECUTION_TIME_INSUFFICIENT')
                    audit.update(complete_execution_path_checked=True,command_path_bounds=bounds)
                    return result
                if req.execute is False:
                    command_start = self._observation_correction_start(start,sample['sdk_positions_rad'],opening)
                    path, reason = planner._plan_joint_from_start_state(command_start,target,
                        planning_time_limit_sec=min(remaining(),2.))
                    if path is None: raise ObservationPathError(str(reason))
                    path_guard(path); fresh_start()
                    audit.update(code='OBSERVATION_ENDPOINT_CHECKED_ONLY', execution_authorized=False)
                    return CompensatePregraspResponse(True,json.dumps(audit,allow_nan=False))
                attrs = ('observation_path_guard_required','observation_path_validator',
                    'observation_tracking_contract_required','observation_execution_authorized',
                    'observation_submission_revalidate','observation_execution_monitor',
                    'strict_execution_max_joint_velocity_rad_s','strict_collinear_timing_enabled')
                saved = {k:(hasattr(planner,k),getattr(planner,k,None)) for k in attrs}
                planner.observation_path_guard_required = True
                planner.observation_tracking_contract_required = True
                planner.observation_path_validator = path_guard
                def authorized():
                    try: validate(); return True
                    except Exception: return False
                planner.observation_execution_authorized = authorized
                planner.observation_execution_monitor = None
                planner.observation_submission_revalidate = lambda path,report:self._revalidate_observation_contract_submission(
                    (scene,context['revocation']),path,report,context_validator=validate)
                planner.strict_execution_max_joint_velocity_rad_s = min(
                    planner.strict_execution_max_joint_velocity_rad_s,.008)
                planner.strict_collinear_timing_enabled = True
                self._configure_controller_reference_guard(planner)
                for operation in (self._ensure_trajectory_controllers_started,
                                  self._synchronize_trajectory_controller_to_feedback):
                    ok,reason = operation()
                    if not ok: raise ObservationPathError(reason)
                from .contracted_trajectory import ContractedTrajectoryExecutor
                if getattr(planner,'observation_trajectory_executor',None) is None:
                    planner.observation_trajectory_executor = ContractedTrajectoryExecutor.connect()
                def before_execute():
                    fresh_start()
                    if rospy.get_param(consumed,None) is not None:
                        raise ObservationPathError('OBSERVATION_COMPENSATION_EPISODE_CONSUMED')
                    rospy.set_param(consumed,dict(plan_id=scene.plan_id,plan_digest=scene.digest,
                        lease_id=receipt['lease_id'],epoch_ns=str(context['epoch']),state='consumed_before_submission'))
                    audit['physical_motion_submitted'] = True
                ok,reason = planner.plan_and_execute_joint_probe(target,execute=True,
                    start_joint_positions=sample['sdk_positions_rad'],before_execute=before_execute)
                if not ok: raise ObservationPathError('OBSERVATION_COMPENSATION_EXECUTION_FAILED: '+str(reason))
                completed = rospy.get_time()
                correction.committed(step,completed)
                after = self._endpoint_following_snapshot(fk,context['epoch'],context,
                    after_ns=int(completed*1e9),context_validator=validate)
                result = correction.evaluate(after)
                sample_fresh(after)
                audit.update(code='OBSERVATION_ENDPOINT_READY',no_motion=False,
                    completion_stamp_ns=int(completed*1e9),accepted_stamp_ns=after['accepted_stamp_ns'],
                    view_audit=result['view_audit'],response=result,post_command_feedback_valid=True)
                return CompensatePregraspResponse(True,json.dumps(audit,allow_nan=False))
            except Exception as exc:
                audit.update(code='OBSERVATION_ENDPOINT_REJECTED',reason=str(exc),success=False)
                return CompensatePregraspResponse(False,json.dumps(audit,allow_nan=False))
            finally:
                if planner is not None:
                    planner._last_pose_plan = None
                    for key,(existed,value) in saved.items():
                        if existed: setattr(planner,key,value)
                        elif hasattr(planner,key): delattr(planner,key)
