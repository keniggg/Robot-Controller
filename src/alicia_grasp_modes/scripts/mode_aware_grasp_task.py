#!/usr/bin/env python3
"""Bind mode/strategy to the existing rich-plan executor."""

from copy import deepcopy
from collections import deque
import importlib.util
import json
from pathlib import Path
import sys
import threading

import rospkg
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String

from alicia_grasp_modes.selection import (SELECTION_PARAM, parse_selection,
                                        read_authoritative_selection)
from alicia_grasp_modes.observation_policy import observation_config
from alicia_grasp_modes.runner_contract import unknown_target_loss_reason
from alicia_grasp_modes.observe_return_task import ObserveReturnTask
from alicia_grasp_modes.station_compute_wait_task import StationComputeWaitTaskMixin
from alicia_grasp_modes.initial_preflight_task import InitialPreflightTaskMixin
from alicia_grasp_modes.observe_return_session import exact_ns
from alicia_grasp_modes.target_mass import mass_config_from_ros
from alicia_grasp_modes.support_observation import (
    support_contradiction, contradiction_after_plan, qualify_support_source)
from alicia_flexible_grasp.grasp.endpoint_tolerance import measured_position_tolerance


def _load_original():
    name = '_alicia_original_grasp_task'
    if name in sys.modules:
        return sys.modules[name]
    root = Path(rospkg.RosPack().get_path('alicia_flexible_grasp_supervisor'))
    spec = importlib.util.spec_from_file_location(name, str(root / 'scripts' / 'grasp_task_node.py'))
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


original = _load_original()


def execution_config(config, strategy, entry_phase=None):
    """Only change phase scheduling; retain physical, temporal and IK gates."""
    if strategy == 'two_stage':
        return config
    if strategy == 'observe_return':
        if entry_phase == 'CONTACT_EXECUTION_PLAN':
            return execution_config(config, 'direct')
        return dict(config, near_field_replan_enabled=True, near_field_replan_required=True,
                    near_field_strategy='single_snapshot_direct', final_visual_refine_enabled=False,
                    observation_reuse_position_tolerance_m=0.0)
    if strategy != 'direct':
        raise ValueError('unsupported execution strategy')
    selected = deepcopy(config)
    selected['near_field_replan_enabled'] = False
    selected['final_visual_refine_enabled'] = False
    # The historical single_snapshot_direct branch omits task-side MuJoCo.
    # A current-view contact plan still uses the full original simulation
    # gate before its first physical action, via the normal contact branch.
    selected['near_field_strategy'] = 'legacy_gated'
    return selected


class ModeAwareGraspTaskNode(InitialPreflightTaskMixin, StationComputeWaitTaskMixin, ObserveReturnTask, original.GraspTaskNode):
    _station_task_result = staticmethod(original.PlanValidationResult)

    def __init__(self):
        self._initialize_mode_boundary()
        super().__init__()
        self._setup_observe_return_io()
        self._setup_station_compute_task_io()
        self._setup_initial_preflight_io()
        self._unknown_support_sub = rospy.Subscriber(
            '/perception/unknown/detector_status', String,
            self._unknown_support_status_cb, queue_size=20)
        self._mode_reset_service = rospy.Service(
            '/grasp_mode/reset_task', Trigger, self.reset_mode_cb)

    def _initialize_mode_boundary(self):
        self._initialize_initial_preflight()
        self._initialize_observe_return_state()
        self._reset_station_compute_task()
        self._mode_task_lock = threading.RLock()
        # Independent lock: the detector callback never takes the execution
        # lock held across a synchronous trajectory service. Its positive
        # evidence must remain available at the next physical action boundary.
        self._unknown_support_lock = threading.Lock()
        self._unknown_support_fault = None
        self._unknown_support_pending = {}
        self._unknown_support_overflow = None
        self._unknown_support_diagnostics = deque(maxlen=128)
        self._unknown_support_qualified_sources = set()
        self._unknown_target_loss = None
        self._mode_selection = None
        self._mode_bound_selection = None
        self._mode_ready = False
        self._mode_start_reserved = False

    def _support_source_snapshot(self):
        lock = getattr(self, '_following_evidence_lock', None)
        if lock is None:
            return (), (), None
        with lock:
            return (tuple(self._following_sdk_history),
                    tuple(self._following_accepted_history), self._following_epoch_ns)

    def _refresh_unknown_support_sources(self):
        # Histories are copied under their existing lock, independently of the
        # execution lock. Revisit delayed brackets on feedback arrival, before
        # the 128-entry history can evict evidence from a long trajectory.
        sdk, accepted, epoch = self._support_source_snapshot()
        diagnostics = []
        with self._unknown_support_lock:
            for key, fault in tuple(self._unknown_support_pending.items()):
                qualification = qualify_support_source(sdk, accepted,
                    source_stamp_ns=fault['source_stamp_ns'], epoch_ns=epoch,
                    expected_epoch_ns=fault['observed_epoch_ns'])
                current_epoch = self._support_source_snapshot()[2]
                if epoch != current_epoch:
                    qualification.update(state='unqualified',
                        reason='support_control_epoch_changed_during_qualification')
                previous = fault.get('source_qualification')
                fault['source_qualification'] = qualification
                if previous != qualification:
                    diagnostics.append(deepcopy(fault))
                if qualification['state'] == 'stationary':
                    hard = self._unknown_support_fault
                    if (hard is None or hard['generation'] != fault['generation']
                            or hard['source_stamp_ns'] < fault['source_stamp_ns']):
                        self._unknown_support_fault = deepcopy(fault)
                    del self._unknown_support_pending[key]
                    self._unknown_support_qualified_sources.add(key)
                elif qualification['state'] == 'moving':
                    # Cache this ORIGINAL source verdict; current stillness or
                    # later history eviction must not turn it into a hard fault.
                    del self._unknown_support_pending[key]
                    self._unknown_support_qualified_sources.add(key)
                # Unqualified evidence stays pending and blocks only a plan
                # older than this positive anomaly; ordinary occlusion adds
                # neither a pending observation nor a new observation duty.
            self._unknown_support_diagnostics.extend(diagnostics)
        for diagnostic in diagnostics:
            rospy.loginfo('Support source qualification: %s',
                          json.dumps(diagnostic, sort_keys=True))

    def accepted_joint_cb(self, message):
        super().accepted_joint_cb(message)
        self._initial_preflight_feedback(message)
        self._refresh_unknown_support_sources()
        if self._station_task_record is not None:
            self._station_compute_task_tick()

    def following_epoch_cb(self, message):
        super().following_epoch_cb(message)
        self._initial_preflight_epoch()
        if self._station_task_record is not None:
            self._station_compute_task_tick()

    def following_sdk_cb(self, message):
        super().following_sdk_cb(message)
        self._refresh_unknown_support_sources()

    def _unknown_support_status_cb(self, message):
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError, AttributeError):
            return
        if not isinstance(status, dict):
            return
        self._initial_preflight_status(status)
        self._station_compute_task_status(status)
        self._refresh_unknown_support_sources()
        selected = self._mode_bound_selection or self._mode_selection
        # Independent diagnostic latch: do not change active trajectory or
        # frozen-contact occlusion handling from this callback.
        if status.get('target_lost') is True:
            try:
                reason = unknown_target_loss_reason(status, selected, rospy.Time.now().to_nsec())
            except Exception:
                reason = ''
            if reason:
                with self._unknown_support_lock:
                    previous = self._unknown_target_loss
                    if previous is None or previous['selection'] != selected:
                        self._unknown_target_loss = dict(selection=deepcopy(selected), reason=reason)
        fault = support_contradiction(status, selected)
        if fault is None:
            # A subsequent missing/occluded view cannot erase an observed
            # contradiction of the frozen table model.
            return
        fault['observed_epoch_ns'] = self._support_source_snapshot()[2]
        with self._unknown_support_lock:
            if selected != (self._mode_bound_selection or self._mode_selection):
                return
            key = (fault['generation'], fault['selection_stamp_ns'], fault['source_stamp_ns'])
            if key in self._unknown_support_qualified_sources:
                return
            if key not in self._unknown_support_pending:
                if len(self._unknown_support_pending) >= 128:
                    # Never silently discard unresolved contradictory evidence.
                    fault['source_qualification'] = dict(state='unqualified',
                        reason='support_source_pending_capacity_exceeded')
                    self._unknown_support_overflow = fault
                else:
                    self._unknown_support_pending[key] = fault
        self._refresh_unknown_support_sources()

    def _near_field_wait_failure(self, current_plan, _gcfg):
        """Only an observation waiting for contact evidence consumes this latch."""
        selected = self._mode_bound_selection
        if (not selected or selected.get('mode') != 'unknown'
                or selected.get('strategy') not in ('two_stage', 'observe_return')
                or getattr(current_plan, 'model_choice', '') != 'unknown_tabletop'
                or original._plan_phase(current_plan) != original._FAR_FIELD_OBSERVATION_PLAN):
            return None
        with self._unknown_support_lock:
            fault = self._unknown_target_loss
        if fault is None or fault['selection'] != selected:
            return self._station_observation_search_failure(current_plan)
        return original.PlanValidationResult(False, 'TARGET_LOST', fault['reason'].partition(': ')[2])

    def _final_visual_wait_failure(self, plan):
        # This hook runs only while a fresh post-arrival reference is required;
        # it adds no gate to a frozen approach, closure, or lift action.
        selected = self._mode_bound_selection
        if (not selected or selected.get('mode') != 'unknown'
                or getattr(plan, 'model_choice', '') != 'unknown_tabletop'):
            return None
        with self._unknown_support_lock:
            fault = self._unknown_target_loss
        if fault is None or fault['selection'] != selected:
            return None
        return original.PlanValidationResult(False, 'TARGET_LOST', fault['reason'].partition(': ')[2])

    def _read_selection(self):
        return parse_selection(rospy.get_param(SELECTION_PARAM, None))

    def _read_reset_selection(self):
        return read_authoritative_selection(rospy.get_master())

    def reset_mode_cb(self, _request):
        try:
            with self._mode_task_lock, self._start_guard(), self._grasp6d_plan_guard():
                if (self._mode_start_reserved or bool(getattr(self, 'active', False))
                        or bool(getattr(self, '_start_inflight', False))):
                    raise RuntimeError('cannot reset grasp strategy during execution')
                selected = self._read_reset_selection()
                previous = self._mode_selection
                if previous is not None:
                    if selected['generation'] < previous['generation']:
                        raise ValueError('mode generation cannot move backwards')
                    if selected['generation'] == previous['generation']:
                        if selected != previous:
                            raise ValueError('selection changed without a new generation')
                        if self._mode_ready:
                            if self._read_reset_selection() != selected:
                                raise RuntimeError('mode selection changed during task reset')
                            return TriggerResponse(True, json.dumps(dict(selected, ready=True)))
                    elif selected['stamp_ns'] <= previous['stamp_ns']:
                        raise ValueError('new generation requires a newer source cutoff')
                self._mode_ready = False
                self._clear_grasp6d_authority()
                self._clear_bound_execution_plan()
                self.latest_grasp6d_preview_plan = None
                self.latest_grasp6d_legacy_plan = None
                self.latest_obj = None
                self.latest_obj_time = None
                self.latest_visual_obj = None
                self.latest_visual_obj_time = None
                self.latest_target_geometry = None
                self._grasp6d_watermark_stamp_ns = max(
                    int(getattr(self, '_grasp6d_watermark_stamp_ns', 0)), selected['stamp_ns'])
                self._grasp6d_watermark_plan_id = ''
                self._grasp6d_watermark_tombstoned = True
                self._mode_bound_selection = None
                with self._unknown_support_lock:
                    self._unknown_target_loss = None
                    self._unknown_support_fault = None
                    self._unknown_support_pending.clear()
                    self._unknown_support_overflow = None
                    self._unknown_support_diagnostics.clear()
                    self._unknown_support_qualified_sources.clear()
                self._clear_observe_return_attempt()
                self._reset_station_compute_task()
                self._mode_selection = selected
                self._initial_preflight_selection(selected)
                if self._read_reset_selection() != selected:
                    raise RuntimeError('mode selection changed during task reset')
                self._mode_ready = True
                return TriggerResponse(True, json.dumps(dict(selected, ready=True), sort_keys=True))
        except Exception as exc:
            return TriggerResponse(False, str(exc))

    def _selection_current(self, selected=None):
        expected = self._mode_selection if selected is None else selected
        if not self._mode_ready or expected is None:
            return False
        try:
            return self._read_selection() == expected
        except Exception:
            return False

    def _admit_source(self, msg, plan=False):
        if not self._mode_ready or self._mode_selection is None:
            return False
        stamp_ns = original._stamp_nanoseconds(getattr(getattr(msg, 'header', None), 'stamp', None))
        if stamp_ns <= self._mode_selection['stamp_ns']:
            return False
        if plan and bool(getattr(msg, 'valid', False)):
            pcfg = rospy.get_param('/perception', {})
            expected = ('unknown_tabletop' if self._mode_selection['mode'] == 'unknown'
                        else str(pcfg.get('yolo_model_choice', 'original')))
            if str(getattr(msg, 'model_choice', '')) != expected:
                return False
        return True

    def obj_cb(self, msg):
        received_ns = rospy.Time.now().to_nsec()
        with self._mode_task_lock:
            if self._admit_source(msg):
                if self._actuation_bootstrap_retired_object(msg, received_ns):
                    return
                if self._station_compute_task_object(msg):
                    return
                return super().obj_cb(msg)

    def target_geometry_cb(self, msg):
        with self._mode_task_lock:
            if self._admit_source(msg):
                if self._station_compute_task_geometry(msg):
                    return
                return super().target_geometry_cb(msg)

    def grasp6d_plan_cb(self, msg):
        with self._mode_task_lock:
            if self._admit_source(msg, plan=True):
                return super().grasp6d_plan_cb(msg)

    def grasp6d_preview_plan_cb(self, msg):
        with self._mode_task_lock:
            if self._admit_source(msg, plan=True):
                return super().grasp6d_preview_plan_cb(msg)

    def grasp6d_legacy_plan_cb(self, msg):
        with self._mode_task_lock:
            if self._admit_source(msg):
                return super().grasp6d_legacy_plan_cb(msg)

    def start_cb(self, req):
        with self._mode_task_lock:
            if not self._selection_current():
                return original.StartGraspResponse(False, 'grasp strategy reset is not ready')
            if self._mode_start_reserved or bool(getattr(self, 'active', False)):
                return original.StartGraspResponse(False, 'already active')
            self._clear_observe_return_attempt()
            self._mode_start_reserved = True
            self._mode_bound_selection = dict(self._mode_selection)
        result = None
        try:
            self._initial_preflight_begin(req)
            result = super().start_cb(req)
            return result
        finally:
            try:
                self._initial_preflight_finish(result)
            finally:
                with self._mode_task_lock:
                    self._mode_start_reserved = False
                    self._mode_bound_selection = None

    def _mujoco_config_for_plan(self, plan, config):
        selected = self._mode_bound_selection or self._mode_selection
        if not self._selection_current(selected):
            raise ValueError('operator mass configuration requires a current selection')
        return mass_config_from_ros(config, plan, selected)

    def _strategy_config(self, config):
        selected = self._mode_bound_selection or self._mode_selection or {}
        result = observation_config(
            execution_config(config, selected.get('strategy', 'two_stage'),
                             self._observe_return_entry_phase), selected)
        if selected.get('mode') == 'unknown':
            result = dict(result, measured_endpoint_position_tolerance_m=
                          measured_position_tolerance(config, 'unknown_tabletop'))
        return result

    def _copy_requested_grasp6d_plan(self, plan_id, gcfg=None):
        selected = self._mode_bound_selection or self._mode_selection
        if not self._selection_current(selected):
            return original.PlanValidationResult(False, 'GRASP_STRATEGY_STALE',
                'current selection differs from the bound strategy'), None
        result, plan = super()._copy_requested_grasp6d_plan(plan_id, self._strategy_config(gcfg or {}))
        if (result.ok and selected['strategy'] == 'direct'
                and original._plan_phase(plan) != original._CONTACT_EXECUTION_PLAN):
            return original.PlanValidationResult(False, 'DIRECT_CONTACT_PLAN_REQUIRED',
                'direct strategy requires a fully checked contact execution plan'), None
        return result, plan

    def _validate_bound_plan_locked(self, plan, gcfg):
        if not self._selection_current(self._mode_bound_selection):
            return original.PlanValidationResult(False, 'GRASP_STRATEGY_STALE',
                'mode or strategy changed after execution was bound')
        session = getattr(self, '_observe_return_session', None)
        if isinstance(session, dict) and session.get('version') == 2:
            # This runs again under the action lock after slow preflight. A
            # session budget cannot be renewed by preparing another view.
            try:
                deadline = exact_ns(session.get('deadline_ns'), 'deadline_ns')
                if rospy.Time.now().to_nsec() >= deadline:
                    return original.PlanValidationResult(False, 'OBSERVE_RETURN_SESSION_EXPIRED',
                        'shared observation session expired before action submission')
            except (TypeError, ValueError, OverflowError):
                return original.PlanValidationResult(False, 'OBSERVE_RETURN_SESSION_INVALID',
                    'shared observation deadline is missing or malformed')
            if original._plan_phase(plan) == original._FAR_FIELD_OBSERVATION_PLAN:
                failure = self._near_field_wait_failure(plan, gcfg)
                if failure is not None and not failure.ok:
                    return failure
        if (getattr(plan, 'model_choice', '') == 'unknown_tabletop'
                and original._plan_phase(plan) == original._CONTACT_EXECUTION_PLAN):
            self._refresh_unknown_support_sources()
            with self._unknown_support_lock:
                fault = self._unknown_support_fault
                pending = tuple(self._unknown_support_pending.values())
                if self._unknown_support_overflow is not None:
                    pending += (self._unknown_support_overflow,)
            plan_stamp = original._stamp_nanoseconds(getattr(getattr(plan, 'header', None), 'stamp', None))
            if contradiction_after_plan(fault, self._mode_bound_selection,
                    plan_stamp):
                return original.PlanValidationResult(False,
                    'SUPPORT_GEOMETRY_CONTRADICTION',
                    'source-qualified stationary RGB-D contradicts the frozen support: %s; '
                    'source_stamp_ns=%s; require new registered observations'
                    % (fault['detail'], fault['source_stamp_ns']))
            unresolved = [row for row in pending
                if contradiction_after_plan(row, self._mode_bound_selection, plan_stamp)]
            if unresolved:
                fault = min(unresolved, key=lambda row: row['source_stamp_ns'])
                return original.PlanValidationResult(False,
                    'SUPPORT_OBSERVATION_UNQUALIFIED',
                    'positive support anomaly lacks original-source motion/stillness evidence; '
                    'source_stamp_ns=%s; reason=%s' % (fault['source_stamp_ns'],
                        fault.get('source_qualification', {}).get('reason', 'awaiting_source_evidence')))
        return super()._validate_bound_plan_locked(plan, gcfg)

    def _execute_grasp6d_plan(self, gcfg, gripper_cfg, open_position, move_pose,
                              move_pose_linear, set_gripper, close, plan,
                              strict_execute_pose=None):
        selected = self._mode_bound_selection
        if not self._selection_current(selected) or selected is None:
            self.set_state(original.GraspStages.FAILED, 'GRASP_STRATEGY_STALE')
            return False
        if (selected['strategy'] == 'direct'
                and original._plan_phase(plan) != original._CONTACT_EXECUTION_PLAN):
            self.set_state(original.GraspStages.FAILED, 'DIRECT_CONTACT_PLAN_REQUIRED')
            return False
        observation_source = original.is_silhouette_observation_plan(plan)
        if original.is_independent_observation_plan(plan) and selected['strategy'] != 'observe_return':
            self.set_state(original.GraspStages.FAILED, 'DIAGNOSTIC_OBSERVATION_STRATEGY_REQUIRED')
            return False
        if selected['strategy'] == 'observe_return':
            self._observe_return_entry_phase = original._plan_phase(plan)
            if self._observe_return_entry_phase == original._FAR_FIELD_OBSERVATION_PLAN:
                try:
                    if (not observation_source and self._gripper_position_reached(open_position,
                            self._cfg_float(gripper_cfg, 'open_skip_tolerance_m', 0.001))):
                        if self._initial_preflight_arm(plan, self._strategy_config(gcfg)):
                            move_pose = self._initial_preflight_wrap_move(move_pose)
                            strict_execute_pose = self._initial_preflight_wrap_strict(strict_execute_pose)
                    self._begin_observe_return(plan, self._strategy_config(gcfg))
                    if self._diagnostic_observation_only():
                        observation = original.observation_plan_pose(plan)
                        return self._execute_diagnostic_observation(
                            plan, self._strategy_config(gcfg), observation, move_pose,
                            strict_execute_pose, original.write_mujoco_execution_audit)
                    if observation_source:
                        raise ValueError('DIAGNOSTIC_OBSERVATION_TRIGGER_REQUIRED')
                except Exception as exc:
                    self.set_state(original.GraspStages.FAILED, 'OBSERVE_RETURN_FAILED: ' + str(exc))
                    return False
            elif self._observe_return_entry_phase == original._CONTACT_EXECUTION_PLAN:
                try:
                    report = self._observe_return_report(plan)
                    if report['observe_return']['phase'] != 'initial_contact':
                        raise ValueError('OBSERVE_RETURN_INITIAL_CONTACT_REQUIRED')
                except Exception as exc:
                    self.set_state(original.GraspStages.FAILED, 'OBSERVE_RETURN_FAILED: ' + str(exc))
                    return False
            else:
                self.set_state(original.GraspStages.FAILED, 'OBSERVE_RETURN_ENTRY_PHASE_INVALID')
                return False
        return super()._execute_grasp6d_plan(
            self._strategy_config(gcfg), gripper_cfg, open_position, move_pose,
            move_pose_linear, set_gripper, close, plan,
            strict_execute_pose=strict_execute_pose)


if __name__ == '__main__':
    rospy.init_node('grasp_task_node')
    ModeAwareGraspTaskNode()
    rospy.spin()
