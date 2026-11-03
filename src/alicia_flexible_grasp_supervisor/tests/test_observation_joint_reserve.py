"""Endpoint reserve, real strict-branch rejection and unchanged path checks."""
from pathlib import Path
from types import SimpleNamespace as NS
import json

import numpy as np
import pytest

from alicia_flexible_grasp.robot.observation_path_guard import (
    SerialUrdfFk, ObservationPathError, observation_endpoint_joint_reserve)
from alicia_flexible_grasp.robot.observation_tracking_contract import qualify_contract_path
from test_observation_path_guard import model_xml, point, trajectory, scene, gripper


def bounded_model():
    return SerialUrdfFk(model_xml().replace('<axis xyz="0 1 0"/>',
        '<axis xyz="0 1 0"/><limit lower="-1" upper="1"/>'), ['J1'])


@pytest.mark.parametrize('q,errors,passed', [([0.], [.1], True),
    ([.9], [.1], True), ([-.9], [.1], True), ([.91], [.1], False),
    ([-.91], [.1], False), ([1.], [.01], False)])
def test_both_original_limits_and_per_contract_reserve(q, errors, passed):
    fk = bounded_model(); before = fk.limits.copy()
    report = observation_endpoint_joint_reserve(fk, q, errors)
    assert report['ok'] is passed
    assert report['insufficient_joints'] == ([] if passed else ['J1'])
    assert not report['execution_authority'] and not report['complete_path_checked']
    np.testing.assert_array_equal(fk.limits, before)


@pytest.mark.parametrize('q,errors', [([1.01], [.1]), ([float('nan')], [.1]),
    ([0.], []), ([0.], [0.]), ([0.], [-.1]), ([0.], [.51]),
    ([0.], [float('nan')]), ([0.], [float('inf')])])
def test_invalid_reserve_inputs_do_not_qualify(q, errors):
    with pytest.raises(ObservationPathError):
        observation_endpoint_joint_reserve(bounded_model(), q, errors)


@pytest.mark.parametrize('endpoint,expected', [(.925, True), (.99, False)])
def test_contract_search_reserves_actual_ik_endpoint_and_keeps_full_path(endpoint, expected):
    fk = bounded_model()
    plan = trajectory([point([.8], 0., [0.], [0.]),
                       point([endpoint], 10., [0.], [0.])])
    hold = point([.8], v=[0.], a=[0.])
    args = (plan, hold, scene(), fk, gripper(), .05, {'J1': {'trajectory': .12}}, .5)
    report = qualify_contract_path(*args, reserve_endpoint_joints=True)
    assert report['ok'] is expected, report
    assert len(report['tracking_contract_search']) == 4
    assert report['endpoint_joint_reserve']['ok'] is expected
    if expected:
        assert report['execution_tracking_contract']['path_position_tolerance_rad'] == [.035]
        assert report['continuous_path']['following_support']['minimum_support_clearance_lower_bound_m'] >= .003
        def collision(*a, **k):
            raise ObservationPathError('injected continuous collision')
        denied = qualify_contract_path(*args, reserve_endpoint_joints=True, path_validator=collision)
        assert not denied['ok'] and denied['failure_location'] == 'path'
        assert denied['reason'] == 'injected continuous collision'
    else:
        assert report['reason'] == 'OBSERVATION_ENDPOINT_JOINT_RESERVE_INSUFFICIENT'
        # Other users of this helper retain their original contract; only
        # observation selection opts into the new endpoint condition.
        assert qualify_contract_path(*args)['ok']


@pytest.mark.parametrize('contracted', [True, False])
def test_recorded_strict_ik_branch_is_rejected_before_ranking(contracted, monkeypatch):
    from test_reachable_observation_seed_replay import load_recorded_scene
    from test_remote_modes import base
    from alicia_flexible_grasp.robot.observation_preview import encode_path_evidence
    args = load_recorded_scene('supplement_view_joint_reserve_20261023')
    root = Path(__file__).resolve().parents[3]
    metadata = json.loads((root/'src/alicia_grasp_modes/tests/fixtures/supplement_view_joint_reserve_20261023.json').read_text())
    goal = metadata['provenance']['failed_strict_goal_q']
    fk = args['fk']; names = list(fk.names)
    params = {'/robot_description': (root/metadata['model_relative_path']).read_text(),
        '/alicia_controller/constraints': {n: {'trajectory': .12, 'goal': .035} for n in names},
        '/robot/observation_tracking_contract_enabled': contracted,
        '/alicia_controller/stop_trajectory_duration': .5}
    monkeypatch.setattr(base.rospy, 'get_param', lambda k, default=None: params.get(k, default))
    start = args['current_joint_positions']; target = base.PoseStamped()
    target.header.frame_id = 'base_link'; target.pose.orientation.w = 1.
    plan = trajectory([point(start, 0., [0.]*6, [0.]*6),
        point(goal, 90., [0.]*6, [0.]*6)], names)
    hold = point(start, v=[0.]*6, a=[0.]*6)
    text = ('observation_joint_names=%s observation_goal_joint_positions=%s '
        'observation_robot_description_sha256=%s observation_path_evidence=%s' % (
        json.dumps(names, separators=(',', ':')), json.dumps(goal, separators=(',', ':')),
        fk.model_sha256, encode_path_evidence(target, plan, hold, .05, 'preserved', 123, fk.model_sha256)))
    source = args['source']; g = source['geometry']
    prepared = NS(ticket=NS(snapshot_stamp_sec=source['binding']['snapshot_stamp_ns']*1e-9),
        geometry=NS(support_normal_base=g['support_normal_base'], support_offset_m=g['support_offset_m'],
            center_base=source['center_base'], axes_base=g['R_base_obb'], size_xyz_m=g['obb_size_xyz_m']))
    node = base.RemoteGrasp6DNode.__new__(base.RemoteGrasp6DNode)
    node.gripper_geometry = args['gripper']
    result, report = node._observation_following_support_evaluation(prepared,
        base.MoveItResult(reachable=True, joint_path_cost=1., joint_max_delta_rad=1., reason=text), target)
    assert not result.reachable and not report['ok'], report
    assert report['reason'] == 'OBSERVATION_ENDPOINT_JOINT_RESERVE_INSUFFICIENT'
    assert set(report['endpoint_joint_reserve']['insufficient_joints']) >= {'Joint1', 'Joint5'}

