"""Offline candidate-vs-execution parity; never contact a ROS master."""
import base64
from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
from geometry_msgs.msg import PoseStamped

from alicia_flexible_grasp.robot.observation_path_guard import (
    ObservationPathError, SerialUrdfFk, observation_following_support_bound,
)
from alicia_flexible_grasp.robot.observation_preview import (
    encode_path_evidence, decode_path_evidence, qualify_candidate_path, candidate_scene,
)
from test_observation_path_guard import model_xml, gripper, point, trajectory, scene
from test_observation_endpoint_support_bound import minimum

ROOT = Path(__file__).resolve().parents[3]


def example():
    fk = SerialUrdfFk(model_xml(True), ['J1', 'J2'])
    target = PoseStamped(); target.pose.orientation.w = 1.
    plan = trajectory([point([0., 0.], 0., [0., 0.], [0., 0.]),
                       point([.1, .1], 5., [0., 0.], [0., 0.])], fk.names)
    hold = point([0., 0.], v=[0., 0.], a=[0., 0.])
    token = encode_path_evidence(target, plan, hold, .05, 'preserved', 12, fk.model_sha256)
    return fk, target, plan, hold, token


def test_roundtrip_multiaxis_path_and_bridge_is_not_execution_authority():
    fk, target, original, hold, token = example()
    data, plan, reference = decode_path_evidence(token, target, fk)
    assert plan == original
    assert reference.positions == hold.positions
    report = qualify_candidate_path(plan, reference, scene(), fk, gripper(), .05, [.12]*2)
    assert report['ok']
    assert report['continuous_path']['following_support']['includes_controller_bridge']
    assert report['scope'] == 'candidate_path_only_not_execution_authority'
    assert report['certifies_hardware_tracking'] is False


@pytest.mark.parametrize('field,value', [
    ('target_sha256', 'different'), ('trajectory_sha256', 'different'),
    ('model_sha256', 'different'), ('scope', 'execute'), ('hold', [0.]),
    ('opening', .06), ('opening', float('nan')), ('reference_mode', 'manual'),
    ('reference_epoch_ns', 0), ('reference_epoch_ns', True),
])
def test_malformed_or_cross_request_path_evidence_fails_closed(field, value):
    fk, target, _, _, token = example()
    payload = json.loads(base64.b64decode(token)); payload[field] = value
    token = base64.b64encode(json.dumps(payload).encode()).decode()
    with pytest.raises(ObservationPathError): decode_path_evidence(token, target, fk)


def test_goal_safe_but_initial_bridge_unsafe_is_distinguished():
    fk = SerialUrdfFk(model_xml(), ['J1']); n = np.array([0., 0., 1.])
    frozen = replace(scene(), offset=.005-minimum(fk, [1.], n, 0.))
    plan = trajectory([point([.1], 0.), point([.1], 5.)])
    report = qualify_candidate_path(plan, point([1.], v=[0.], a=[0.]),
                                    frozen, fk, gripper(), .05, [.12])
    assert not report['ok'] and report['failure_location'] == 'start'
    assert 'continuous_path' not in report


def test_safe_endpoints_do_not_hide_intermediate_collision():
    fk = SerialUrdfFk(model_xml(), ['J1'])
    plan = trajectory([point([0.], 0.), point([3.], 3.), point([0.], 6.)])
    report = qualify_candidate_path(plan, point([0.], v=[0.], a=[0.]),
                                    scene(), fk, gripper(), .05, [.12])
    assert report['start_support']['ok'] and report['goal_support']['ok']
    assert not report['ok'] and report['failure_location'] == 'path'


def test_september17_recorded_sdk_hold_fails_before_any_new_goal():
    # Exact successful-write hold and frozen plane from plan
    # 2ea71422bc895aaa4d354ea1, audit a765643d..., closed attempt.bag.
    fk = SerialUrdfFk((ROOT/'src/real-arm/alicia_d_descriptions/urdf/alicia_duo_with_gripper.urdf').read_text(),
                      ['Joint%d'%i for i in range(1,7)])
    hold = [-1.8745245227962535, .3896311201229528, -.2745825610315298,
            0., -.035281558121369745, .0046019423636569235]
    geometry = NS(support_normal_base=[-.014978967441305988,.1398866082614099,.9900542244561715],
                  support_offset_m=-.020700994575442493,
                  center_base=[5.,5.,5.], axes_base=np.eye(3), size_xyz_m=[.02]*3)
    frozen = candidate_scene(geometry, 1789698707238812685)
    report = observation_following_support_bound(fk, hold, [.12+np.pi/4096]*6,
        frozen.normal, frozen.offset, gripper(), .04975)
    assert not report['ok']
    witness = np.array(report['counterexample_joint_positions_rad'])
    assert np.all(np.abs(witness-np.array(hold)) <= .12+np.pi/4096+1e-15)
    from alicia_flexible_grasp.grasp.gripper_geometry import _box_corners, _stage_boxes
    transform = fk(witness)
    exact = min(float(np.min(_box_corners(c, transform[:3, :3], size)
                            @ frozen.normal + frozen.offset))
                for _, c, size in _stage_boxes(transform, gripper(), .04975, 'y', 'z'))
    assert report['counterexample_model_clearance_m'] == pytest.approx(exact, abs=1e-12)
    assert exact < .003
    assert report['nominal_minimum_support_clearance_m'] > .05


def test_wire_scene_does_not_round_double_precision_normal_to_float32():
    geometry = NS(support_normal_base=[-.014978967441305988,.1398866082614099,.9900542244561715],
        support_offset_m=-.020700994575442493, center_base=[1.,2.,3.],
        axes_base=np.eye(3), size_xyz_m=[.1]*3)
    frozen = candidate_scene(geometry, 1)
    np.testing.assert_allclose(frozen.normal, geometry.support_normal_base, atol=1e-15, rtol=0.)
    assert frozen.offset == float(np.float32(geometry.support_offset_m))


@pytest.mark.parametrize('mutate', ['', 'epoch', 'reference', 'opening', 'model', 'manual', 'retime_failure'])
def test_gateway_preview_restores_provider_and_never_commands(monkeypatch, mutate):
    from test_motion_gateway_controller_start import motion_gateway_node as module
    from unittest.mock import Mock
    fk, target, plan, hold, _ = example()
    # Use six names for the real gateway contract; this test isolates its
    # transport/reference handling, not FK (covered by other tests above).
    names = ['Joint%d'%i for i in range(1,7)]
    plan.joint_trajectory.joint_names = names
    for p in plan.joint_trajectory.points:
        p.positions=[0.]*6; p.velocities=[0.]*6; p.accelerations=[0.]*6
    gw = module.MotionGateway.__new__(module.MotionGateway)
    gw.joint_names = names+['right_finger']; gw._control_reference_epoch_ns=12
    state = {'reference':[0.]*6,'opening':.05,'signature':'model','manual':False}
    gw._select_controller_sync_reference=lambda: (list(state['reference']),'frozen_sdk_handoff')
    gw._observation_measured_opening=lambda: state['opening']
    gw._manual_control_active=lambda: state['manual']
    gw._synchronize_trajectory_controller_to_feedback=Mock(side_effect=AssertionError('motion'))
    gw._ensure_trajectory_controllers_started=Mock(side_effect=AssertionError('controller start'))
    original_provider=object()
    planner=NS(_last_pose_plan={'kind':'strict pose','plan':plan,
        'observation_branch':{'robot_description_sha256':fk.model_sha256}},
        controller_reference_provider=original_provider,
        _observation_branch_signature=lambda: (state['signature'],tuple(names)),
        _retime_strict_execution_plan=lambda p: (p,''))
    def prepare(p):
        if mutate=='epoch':gw._control_reference_epoch_ns=13
        if mutate=='reference':state['reference'][0]=.001
        if mutate=='opening':state['opening']=.049
        if mutate=='model':state['signature']='new'
        if mutate=='manual':state['manual']=True
        if mutate=='retime_failure':raise ValueError('synthetic timing failure')
        assert planner.controller_reference_provider(tuple(names)).positions == [0.]*6
        return p,{},''
    planner._prepare_controller_reference_timing=prepare
    if mutate:
        with pytest.raises((ObservationPathError,ValueError)):
            gw._observation_preview_path_evidence(planner,target)
    else:
        token=gw._observation_preview_path_evidence(planner,target)
        assert json.loads(base64.b64decode(token))['reference_mode']=='frozen_sdk_handoff'
    assert planner.controller_reference_provider is original_provider
    assert planner._last_pose_plan['plan'] is plan
    gw._synchronize_trajectory_controller_to_feedback.assert_not_called()
    gw._ensure_trajectory_controllers_started.assert_not_called()
