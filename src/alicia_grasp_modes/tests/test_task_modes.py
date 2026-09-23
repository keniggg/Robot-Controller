from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import types

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'mode_aware_grasp_task.py'
spec = importlib.util.spec_from_file_location('mode_task_under_test', str(SCRIPT))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
base = module.original


@pytest.fixture
def configured(monkeypatch):
    params = {module.SELECTION_PARAM: {'mode': 'unknown', 'strategy': 'direct',
                                      'generation': 2, 'stamp_ns': '2000000000'}}
    monkeypatch.setattr(module.rospy, 'get_param', lambda name, default=None: params.get(name, default))
    node = module.ModeAwareGraspTaskNode.__new__(module.ModeAwareGraspTaskNode)
    node._initialize_mode_boundary()
    node.active = False
    node.set_state = lambda *args, **kwargs: None
    assert node.reset_mode_cb(None).success
    return node, params


def test_direct_only_changes_phase_scheduling_and_keeps_physical_config():
    config = {'near_field_replan_enabled': True, 'near_field_replan_required': True,
              'near_field_strategy': 'single_snapshot_direct', 'final_visual_refine_enabled': True,
              'measured_endpoint_check_enabled': True, 'max_gripper_width_m': 0.05,
              'plan_validity_sec': 30, 'contact_endpoint_precision_enabled': True}
    before = deepcopy(config)
    direct = module.execution_config(config, 'direct')
    expected = dict(before, near_field_replan_enabled=False,
                    final_visual_refine_enabled=False, near_field_strategy='legacy_gated')
    assert direct == expected
    assert config == before
    assert module.execution_config(config, 'two_stage') is config


def test_task_reset_ack_binds_strategy_and_clears_cached_authority(configured):
    node, params = configured
    node.latest_grasp6d_preview_plan = object()
    node.latest_visual_obj = object()
    params[module.SELECTION_PARAM] = {'mode': 'carton', 'strategy': 'two_stage',
                                    'generation': 3, 'stamp_ns': '3000000000'}
    result = node.reset_mode_cb(None)
    assert result.success
    assert json.loads(result.message)['strategy'] == 'two_stage'
    assert node.latest_grasp6d_plan is None
    assert node.latest_grasp6d_preview_plan is None
    assert node.latest_visual_obj is None
    assert node._grasp6d_watermark_tombstoned
    assert node._grasp6d_watermark_stamp_ns == 3000000000


def test_task_uses_bound_unknown_observation_distance_then_restores_carton(configured):
    node, _ = configured
    config = {'near_field_replan_enabled':True, 'observation_camera_target_max_distance_m':.22}
    node._mode_bound_selection = dict(mode='unknown', strategy='two_stage')
    assert node._strategy_config(config)['observation_camera_target_max_distance_m'] == .30
    assert node._strategy_config(config)['near_field_replan_enabled'] is True
    node._mode_bound_selection = dict(mode='carton', strategy='two_stage')
    assert node._strategy_config(config) is config


@pytest.mark.parametrize('field', ['active', '_start_inflight', '_mode_start_reserved'])
def test_reset_refuses_active_or_reserved_execution(configured, field):
    node, _ = configured
    setattr(node, field, True)
    assert not node.reset_mode_cb(None).success


def test_strategy_cannot_change_within_generation(configured):
    node, params = configured
    params[module.SELECTION_PARAM]['strategy'] = 'two_stage'
    assert not node.reset_mode_cb(None).success
    result = node.start_cb(types.SimpleNamespace(execute=True, plan_id='p'))
    assert not result.success


def test_direct_rejects_observation_plan_before_original_execution(configured, monkeypatch):
    node, _ = configured
    plan = types.SimpleNamespace(diagnostic=base._FAR_FIELD_OBSERVATION_PLAN)
    monkeypatch.setattr(base.GraspTaskNode, '_copy_requested_grasp6d_plan',
                        lambda self, plan_id, gcfg: (base.PlanValidationResult(True), plan))
    result, copied = node._copy_requested_grasp6d_plan('p', {})
    assert not result.ok
    assert result.code == 'DIRECT_CONTACT_PLAN_REQUIRED'
    assert copied is None
    node._mode_bound_selection = dict(node._mode_selection)
    monkeypatch.setattr(base.GraspTaskNode, '_execute_grasp6d_plan',
                        lambda *args, **kwargs: pytest.fail('observation plan reached executor'))
    assert node._execute_grasp6d_plan({}, {}, .05, None, None, None, None, plan) is False


def test_strategy_change_invalidates_bound_action_before_original_commit(configured, monkeypatch):
    node, params = configured
    node._mode_bound_selection = dict(node._mode_selection)
    params[module.SELECTION_PARAM] = {'mode': 'unknown', 'strategy': 'two_stage',
                                    'generation': 3, 'stamp_ns': '3000000000'}
    monkeypatch.setattr(base.GraspTaskNode, '_validate_bound_plan_locked',
                        lambda *args: pytest.fail('stale strategy passed action boundary'))
    called = []
    validation, response = node._invoke_plan_bound_action(
        object(), {}, 'close', lambda: called.append('close'))
    assert not validation.ok
    assert validation.code == 'GRASP_STRATEGY_STALE'
    assert response is None and called == []


def test_start_reservation_covers_entire_original_blocking_task(configured, monkeypatch):
    node, _ = configured
    def execute(self, req):
        assert self._mode_start_reserved
        assert self._mode_bound_selection['strategy'] == 'direct'
        assert not self.reset_mode_cb(None).success
        assert not self.start_cb(req).success
        return base.StartGraspResponse(True, 'done')
    monkeypatch.setattr(base.GraspTaskNode, 'start_cb', execute)
    assert node.start_cb(types.SimpleNamespace(execute=True)).success
    assert not node._mode_start_reserved
    assert node._mode_bound_selection is None


def test_direct_runs_original_four_stage_contact_flow_and_mujoco_without_observation(configured, monkeypatch):
    node, _ = configured
    node._mode_bound_selection = dict(node._mode_selection)
    plan = types.SimpleNamespace(diagnostic=base._CONTACT_EXECUTION_PLAN, plan_id='a' * 24)
    events = []
    states = []
    node.set_state = lambda *args, **kwargs: states.append(args)
    monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(lambda: module.rospy.Time(10)))
    monkeypatch.setattr(base, 'validate_execution_plan', lambda *args, **kwargs: base.PlanValidationResult(True))
    monkeypatch.setattr(base, 'split_rich_plan_poses', lambda plan: ('pre', 'approach', 'grasp', 'lift'))
    monkeypatch.setattr(base, 'validate_calibration_centering_margin',
                        lambda *args: (events.append('centering') or base.PlanValidationResult(True)))
    node._execution_checkpoint = lambda *args: True
    node._position_only_execute_globally_enabled = lambda: False
    node._simulate_grasp6d_plan_if_required = lambda *args: (events.append('simulation') or True)
    node._command_gripper_position = lambda *args, **kwargs: (events.append('open') or True)
    node._plan_and_execute_pose = lambda stage, label, pose, *args, **kwargs: (events.append(pose) or True)
    node._close_gripper = lambda *args, **kwargs: (events.append('close') or True, 'closed')
    node._post_lift_visual_verification_result = lambda *args: (events.append('verify') or base.PlanValidationResult(True))
    node._set_near_field_active = lambda *args, **kwargs: pytest.fail('direct requested observation replan')
    config = {'near_field_replan_enabled': True, 'near_field_replan_required': True,
              'near_field_strategy': 'single_snapshot_direct', 'final_visual_refine_enabled': True}
    success = node._execute_grasp6d_plan(config, {'use_compliant_close': True}, .05,
                                       lambda *args: None, lambda *args: None,
                                       lambda *args: None, lambda *args: None,
                                       plan, strict_execute_pose=lambda *args: None)
    assert success, (states, events)
    assert events == ['simulation', 'open', 'pre', 'centering', 'approach', 'grasp', 'close', 'lift', 'verify']
    assert config['near_field_replan_enabled'] is True
    assert config['final_visual_refine_enabled'] is True


def test_carton_two_stage_delegates_exact_original_config(configured, monkeypatch):
    node, params = configured
    params[module.SELECTION_PARAM] = dict(params[module.SELECTION_PARAM],
                                         mode='carton', strategy='two_stage', generation=3, stamp_ns='3000000000')
    assert node.reset_mode_cb(None).success
    node._mode_bound_selection = dict(node._mode_selection)
    config = {'near_field_replan_enabled': True, 'final_visual_refine_enabled': True}
    def execute(self, gcfg, *args, **kwargs):
        assert gcfg is config
        return 'original-two-stage'
    monkeypatch.setattr(base.GraspTaskNode, '_execute_grasp6d_plan', execute)
    result = node._execute_grasp6d_plan(config, {}, .05, None, None, None, None, object())
    assert result == 'original-two-stage'


def test_queued_old_plan_and_wrong_mode_cannot_cross_task_reset(configured, monkeypatch):
    node, _ = configured
    calls = []
    monkeypatch.setattr(base.GraspTaskNode, 'grasp6d_plan_cb', lambda self, msg: calls.append(msg))
    msg = types.SimpleNamespace(header=types.SimpleNamespace(stamp=module.rospy.Time(2)),
                                valid=True, model_choice='unknown_tabletop')
    node.grasp6d_plan_cb(msg)
    assert calls == []
    msg.header.stamp = module.rospy.Time(3)
    msg.model_choice = 'carton'
    node.grasp6d_plan_cb(msg)
    assert calls == []
    msg.model_choice = 'unknown_tabletop'
    node.grasp6d_plan_cb(msg)
    assert calls == [msg]
