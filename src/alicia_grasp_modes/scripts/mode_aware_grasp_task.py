#!/usr/bin/env python3
"""Bind mode/strategy to the existing rich-plan executor without editing it."""

from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import sys
import threading

import rospkg
import rospy
from std_srvs.srv import Trigger, TriggerResponse

from alicia_grasp_modes.selection import SELECTION_PARAM, parse_selection
from alicia_grasp_modes.observation_policy import observation_config
from alicia_grasp_modes.target_mass import mass_config_from_ros


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


def execution_config(config, strategy):
    """Only change phase scheduling; retain physical, temporal and IK gates."""
    if strategy == 'two_stage':
        return config
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


class ModeAwareGraspTaskNode(original.GraspTaskNode):
    def __init__(self):
        self._initialize_mode_boundary()
        super().__init__()
        self._mode_reset_service = rospy.Service(
            '/grasp_mode/reset_task', Trigger, self.reset_mode_cb)

    def _initialize_mode_boundary(self):
        self._mode_task_lock = threading.RLock()
        self._mode_selection = None
        self._mode_bound_selection = None
        self._mode_ready = False
        self._mode_start_reserved = False

    def _read_selection(self):
        return parse_selection(rospy.get_param(SELECTION_PARAM, None))

    def reset_mode_cb(self, _request):
        try:
            with self._mode_task_lock, self._start_guard(), self._grasp6d_plan_guard():
                if (self._mode_start_reserved or bool(getattr(self, 'active', False))
                        or bool(getattr(self, '_start_inflight', False))):
                    raise RuntimeError('cannot reset grasp strategy during execution')
                selected = self._read_selection()
                previous = self._mode_selection
                if previous is not None:
                    if selected['generation'] < previous['generation']:
                        raise ValueError('mode generation cannot move backwards')
                    if selected['generation'] == previous['generation']:
                        if selected != previous:
                            raise ValueError('selection changed without a new generation')
                        if self._mode_ready:
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
                self._mode_selection = selected
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
        with self._mode_task_lock:
            if self._admit_source(msg):
                return super().obj_cb(msg)

    def target_geometry_cb(self, msg):
        with self._mode_task_lock:
            if self._admit_source(msg):
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
            self._mode_start_reserved = True
            self._mode_bound_selection = dict(self._mode_selection)
        try:
            return super().start_cb(req)
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
        return observation_config(
            execution_config(config, selected.get('strategy', 'two_stage')), selected)

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
        return super()._execute_grasp6d_plan(
            self._strategy_config(gcfg), gripper_cfg, open_position, move_pose,
            move_pose_linear, set_gripper, close, plan,
            strict_execute_pose=strict_execute_pose)


if __name__ == '__main__':
    rospy.init_node('grasp_task_node')
    ModeAwareGraspTaskNode()
    rospy.spin()
