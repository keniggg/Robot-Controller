from copy import deepcopy
from types import SimpleNamespace as NS
from unittest.mock import Mock
import io
import struct
import pytest

from alicia_flexible_grasp.grasp.gripper_close_contract import (
    fixed_close_contract, bind_simulated_fixed_close)
from test_grasp_task_sequence import grasp_task_node


def settings():
    return dict(use_compliant_close=False,use_plan_bound_close_position=True,
        open_position_m=.05,close_limit_m=0.,simple_close_position_m=0.,
        plan_bound_opening_clearance_each_side_m=.002,plan_bound_close_preload_m=.002,
        simple_close_wait_sec=.8)


@pytest.mark.parametrize('width',[.015661925077438354,.020725126090947336,.044,.005])
def test_payload_target_matches_real_fixed_close_command(width):
    plan=NS(plan_id='frozen',required_open_width_m=width)
    cfg=settings();sim={'gripper_model':{'close_target_inner_gap_m':0.,'name':'fixed-model'}}
    original=deepcopy((cfg,sim))
    bound=bind_simulated_fixed_close(plan,sim,cfg)
    node=grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
    node._command_gripper_position=Mock(return_value=True)
    node._simulated_fixed_close_contract=('frozen',fixed_close_contract(plan,cfg))
    assert node._close_gripper(cfg,object(),None,execution_plan=plan)[0]
    sent=node._command_gripper_position.call_args.args
    assert sent[1]==bound['gripper_model']['close_target_inner_gap_m']
    assert sent[3]==bound['gripper_model']['close_settle_sec']
    assert (cfg,sim)==original
    assert bound['gripper_model']['name']=='fixed-model'
    assert sent[1]==pytest.approx(max(0.,width-.006))


def test_changed_close_target_after_simulation_does_not_publish():
    plan=NS(plan_id='frozen',required_open_width_m=.02);cfg=settings()
    node=grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
    node._simulated_fixed_close_contract=('frozen',fixed_close_contract(plan,cfg))
    node._command_gripper_position=Mock(return_value=True)
    cfg['plan_bound_close_preload_m']+=.001
    ok,reason=node._close_gripper(cfg,object(),None,execution_plan=plan)
    assert not ok and 'PLAN_GRIPPER_COMMAND_CHANGED' in reason
    node._command_gripper_position.assert_not_called()


@pytest.mark.parametrize('key,value',[('plan_bound_opening_clearance_each_side_m',float('nan')),
    ('plan_bound_close_preload_m',float('inf')),('simple_close_wait_sec',float('nan')),
    ('close_limit_m',True),('open_position_m',None)])
def test_invalid_real_close_configuration_cannot_make_simulation_payload(key,value):
    cfg=settings();cfg[key]=value
    with pytest.raises(ValueError):bind_simulated_fixed_close(NS(required_open_width_m=.02),{},cfg)


def test_compliant_mode_keeps_its_separate_simulation_contract():
    cfg=settings();cfg['use_compliant_close']=True;sim={}
    assert bind_simulated_fixed_close(NS(),sim,cfg) is sim


@pytest.mark.parametrize('width', [
    .00974476702670619, .015661925077438354, .020725126090947336,
    .044, .005, .05,
])
def test_preview_and_ros_wire_plan_bind_identical_close_contracts(width):
    wire_width = struct.unpack('<f', struct.pack('<f', width))[0]
    preview = NS(plan_id='frozen', required_open_width_m=width)
    received = NS(plan_id='frozen', required_open_width_m=wire_width)
    cfg = settings()
    simulation = {'gripper_model': {'name': 'fixed-model'}}
    before = deepcopy((vars(preview), vars(received), cfg, simulation))

    bound_preview = bind_simulated_fixed_close(preview, simulation, cfg)
    bound_received = bind_simulated_fixed_close(received, simulation, cfg)
    assert bound_preview == bound_received
    node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
    node._command_gripper_position = Mock(return_value=True)
    node._simulated_fixed_close_contract = ('frozen', fixed_close_contract(preview, cfg))
    assert node._close_gripper(cfg, object(), None, execution_plan=received)[0]
    assert node._command_gripper_position.call_args.args[1] == (
        bound_preview['gripper_model']['close_target_inner_gap_m'])
    assert (vars(preview), vars(received), cfg, simulation) == before


def test_serialized_configured_upper_limit_does_not_expand_physical_opening():
    cfg = settings()
    cfg.update(open_position_m=.035,
               plan_bound_opening_clearance_each_side_m=0.,
               plan_bound_close_preload_m=0.)
    wire_width = struct.unpack('<f', struct.pack('<f', .035))[0]
    assert wire_width > cfg['open_position_m']
    for width in (.035, wire_width):
        assert fixed_close_contract(NS(required_open_width_m=width), cfg)[
            'close_target_inner_gap_m'] == .035
    with pytest.raises(ValueError, match='PLAN_GRIPPER_WIDTH_INVALID'):
        fixed_close_contract(NS(required_open_width_m=.035000004), cfg)


@pytest.mark.parametrize('lower', [.007, .01, .02])
def test_nonzero_close_limit_remains_exclusive_after_ros_serialization(lower):
    cfg = settings()
    cfg['close_limit_m'] = lower
    bits = struct.unpack('<I', struct.pack('<f', lower))[0]
    wire_lower = struct.unpack('<f', struct.pack('<I', bits))[0]
    wire_below = struct.unpack('<f', struct.pack('<I', bits - 1))[0]
    wire_above = struct.unpack('<f', struct.pack('<I', bits + 1))[0]
    for width in (lower, wire_lower, wire_below):
        with pytest.raises(ValueError, match='PLAN_GRIPPER_WIDTH_INVALID'):
            fixed_close_contract(NS(required_open_width_m=width), cfg)
    assert fixed_close_contract(NS(required_open_width_m=wire_above), cfg)[
        'close_target_inner_gap_m'] == lower


@pytest.mark.parametrize('width', [0., -0.001, .051, float('nan'), float('inf'), True, 1e100])
def test_invalid_plan_width_is_not_clamped_into_a_valid_close(width):
    with pytest.raises(ValueError, match='PLAN_GRIPPER_WIDTH_INVALID'):
        fixed_close_contract(NS(required_open_width_m=width), settings())


@pytest.mark.parametrize('width', [.00974476702670619, .0167228875332801, .05])
def test_bound_simulation_payload_survives_actual_ros_plan_serialization(width):
    from test_mujoco_digital_twin_client import _rich_plan
    from alicia_flexible_grasp.vision.mujoco_digital_twin_client import build_mujoco_payload

    plan = _rich_plan()
    plan.required_open_width_m = width
    encoded = io.BytesIO()
    plan.serialize(encoded)
    received = type(plan)()
    received.deserialize(encoded.getvalue())
    payloads = [build_mujoco_payload(
        current, ['Joint1'], [0.],
        bind_simulated_fixed_close(current, {}, settings()),
    ) for current in (plan, received)]
    assert payloads[0] == payloads[1]
