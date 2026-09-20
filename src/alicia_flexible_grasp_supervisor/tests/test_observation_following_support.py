"""No robot I/O: counterfactual replay and conservative uncertainty proofs."""
import importlib.util
import itertools
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from alicia_flexible_grasp.robot.observation_path_guard import (
    ObservationPathError, ObservationSupportBox, SerialUrdfFk,
    observation_tracking_error_bounds, observation_following_support_bound,
)
from test_observation_path_guard import (
    model_xml, gripper, point, trajectory, check, scene,
)
from test_observation_endpoint_support_bound import minimum

FIXTURE = Path(__file__).parent/'fixtures/observation_following_20260913.json'
ROOT = Path(__file__).resolve().parents[3]
URDF = ROOT/'src/real-arm/alicia_d_descriptions/urdf/alicia_duo_with_gripper.urdf'


def test_controller_path_not_goal_band_and_no_missing_or_disabled_fallback():
    assert observation_tracking_error_bounds({'J1': {'trajectory': .12, 'goal': .035}}, ['J1']) == pytest.approx([.12+np.pi/4096.])
    assert observation_tracking_error_bounds({'J1': {'trajectory': .01}}, ['J1']) == pytest.approx([.12+np.pi/4096.])
    assert observation_tracking_error_bounds({'J1': {'trajectory': .2}}, ['J1']) == pytest.approx([.2+np.pi/4096.])
    for invalid in (None, {}, {'J1': {}}, *({'J1': {'trajectory': value}}
                   for value in [True, 0., -1., float('nan'), float('inf'), .6, '.12'])):
        with pytest.raises(ObservationPathError):
            observation_tracking_error_bounds(invalid, ['J1'])


@pytest.mark.parametrize('names', [('J1', 'J2'), ('J2', 'J1')])
def test_joint_box_taylor_encloses_corners_and_random_interior(names):
    fk = SerialUrdfFk(model_xml(True), names)
    rng = np.random.RandomState(139)
    for _ in range(20):
        q, width = rng.uniform(-1., 1., 2), rng.uniform(.01, .15, 2)
        n = rng.normal(size=3); n /= np.linalg.norm(n)
        box = ObservationSupportBox(fk, gripper(), .05, n, 1.)
        bound, nominal, _ = box.enclosure(q, width)
        assert nominal == pytest.approx(minimum(fk, q, n, 1.), abs=1e-12)
        samples = list(itertools.product([-1., 1.], repeat=2)) + list(rng.uniform(-1., 1., (100, 2)))
        assert all(minimum(fk, q+np.asarray(s)*width, n, 1.) >= bound-1e-12 for s in samples)
        report = box.check(q-width, q+width)
        if report['ok']:
            assert all(minimum(fk, q+np.asarray(s)*width, n, 1.) >=
                       report['minimum_support_clearance_lower_bound_m']-1e-12 for s in samples)


def test_fixed_suffix_composition_does_not_treat_tool_offset_as_another_joint():
    fk = SerialUrdfFk(model_xml(), ['J1'])
    np.testing.assert_allclose(fk.corner_motion_radii([0., 0., -.1]), [.3])
    np.testing.assert_allclose(fk.point_motion_radii(.1), [.5])
    real = SerialUrdfFk(URDF.read_text(), ['Joint%d'%i for i in range(1, 7)])
    rng = np.random.RandomState(1309)
    data = json.loads(FIXTURE.read_text())
    box = ObservationSupportBox(real, gripper(), .05, data['normal'], data['offset'])
    for _ in range(20):
        q = rng.uniform(real.limits[:, 0]+.2, real.limits[:, 1]-.2)
        width = np.full(6, .12)
        bound, _, _ = box.enclosure(q, width)
        for sample in rng.uniform(-1., 1., (50, 6)):
            assert minimum(real, q+sample*width, np.array(data['normal']), data['offset']) >= bound-1e-12


def test_gradient_corner_rejects_with_exact_in_box_fk_counterexample():
    fk = SerialUrdfFk(model_xml(), ['J1'])
    n = np.array([0., 0., 1.])
    q, width = np.array([1.]), np.array([.12])
    offset = .005-minimum(fk, q, n, 0.)
    box = ObservationSupportBox(fk, gripper(), .05, n, offset)
    report = box.check(q-width, q+width, max_checks=2)
    assert report['ok'] is False
    assert report['box_checks'] == 2
    witness = np.array(report['counterexample_joint_positions_rad'])
    assert np.all(witness >= q-width) and np.all(witness <= q+width)
    exact = minimum(fk, witness, n, offset)
    assert report['counterexample_model_clearance_m'] == pytest.approx(exact, abs=1e-12)
    assert exact < .003
    with pytest.raises(ObservationPathError, match='budget exhausted'):
        box.check(q-width, q+width, max_checks=1)


def test_passing_corner_does_not_certify_unresolved_box(monkeypatch):
    box = ObservationSupportBox(SerialUrdfFk(model_xml(), ['J1']),
                                gripper(), .05, [0., 0., 1.], 1.)
    monkeypatch.setattr(box, 'enclosure', lambda q, w: (-.1, .01, np.array([.1])))
    with pytest.raises(ObservationPathError, match='budget exhausted'):
        box.check([0.], [.2], max_checks=2)


@pytest.mark.parametrize('kwargs', [{'max_checks': 0}, {'max_seconds': 0.},
                                  {'max_seconds': float('nan')}])
def test_unresolved_bounds_fail_closed(kwargs):
    fk = SerialUrdfFk(model_xml(), ['J1'])
    with pytest.raises(ObservationPathError):
        observation_following_support_bound(fk, [0.], [.12], [0., 0., 1.], 0.,
                                           gripper(), .05, **kwargs)


def test_multijoint_following_path_includes_bridge_without_changing_nominal_scope():
    plan = trajectory([point([0., 0.], 0., [0., 0.], [0., 0.]),
                       point([.2, .1], 3., [0., 0.], [0., 0.])], ['J1', 'J2'])
    report = check(plan, joint_error_bounds_rad=[.12, .12])
    assert report['following_support']['includes_controller_bridge'] is True
    assert report['following_support']['minimum_support_clearance_lower_bound_m'] >= .003
    assert report['following_support']['certifies_hardware_tracking_stopping_or_calibration'] is False
    assert report['following_support']['target_obb_tracking_uncertainty_checked'] is False


def test_safe_nominal_bridge_is_rejected_when_its_error_box_can_cross_plane():
    from dataclasses import replace
    fk = SerialUrdfFk(model_xml(), ['J1'])
    normal = np.array([0., 0., 1.])
    offset = .005-minimum(fk, [1.], normal, 0.)
    frozen = replace(scene(), offset=offset)
    plan = trajectory([point([.1], 0.), point([.1], 4.)])
    hold = point([1.], v=[0.], a=[0.])
    assert check(plan, frozen=frozen, desired=hold)['certified_minimum_clearance_m'] >= .003
    with pytest.raises(ObservationPathError, match='FOLLOWING_SUPPORT_INVALID'):
        check(plan, frozen=frozen, desired=hold, joint_error_bounds_rad=[.12])


def test_closed_bag_replay_rejects_failed_branch_and_has_a_passing_alternative():
    spec = importlib.util.spec_from_file_location('following_replay', ROOT/'tools/replay_observation_following_readonly.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    fixture = json.loads(FIXTURE.read_text())
    report = module.replay(fixture, URDF.read_text())
    assert report['candidates'][0]['following_support']['ok'] is False
    selected = report['selected_candidate']
    assert selected is not None
    assert selected['camera_target_distance_m'] == .22
    assert selected['camera_roll_offset_deg'] == 180.
    assert report['executed_paths'][0]['following_passed'] is True
    assert report['executed_paths'][1]['following_passed'] is False
    assert all(p['nominal']['certified_minimum_clearance_m'] >= .003 for p in report['executed_paths'])
    assert report['grasp_success_proven'] is False


def test_alternative_has_a_continuous_support_feasible_path_not_only_a_good_endpoint():
    from alicia_flexible_grasp.robot.observation_path_guard import (
        FrozenObservationScene, validate_observation_trajectory,
        FOLLOWING_PATH_MAX_CHECKS, FOLLOWING_PATH_MAX_SECONDS,
    )
    data = json.loads(FIXTURE.read_text())
    fk = SerialUrdfFk(URDF.read_text(), ['Joint%d'%i for i in range(1, 7)])
    frozen = FrozenObservationScene(data['plan_id'], 'synthetic-not-executed', 1,
        np.array(data['normal']), data['offset'], np.array(data['center']),
        np.array(data['rotation']), np.array(data['size']))
    start, goal = data['executed'][0]['hold'], data['candidates'][-1]['q']
    hold = point(start, v=[0.]*6, a=[0.]*6)
    plan = trajectory([point(start, 0., [0.]*6, [0.]*6),
                       point(goal, 90., [0.]*6, [0.]*6)], fk.names)
    # Counterfactual existence test only: not a whole-arm MoveIt check, fresh
    # scene, execution authorization, commanded path or physical success.
    report = validate_observation_trajectory(plan, frozen, fk, gripper(), data['opening'], hold,
        joint_error_bounds_rad=[.12+np.pi/4096.]*6,
        max_checks=FOLLOWING_PATH_MAX_CHECKS, max_seconds=FOLLOWING_PATH_MAX_SECONDS)
    assert report['following_support']['minimum_support_clearance_lower_bound_m'] >= .003


def test_production_selection_qualifies_before_ranking_duration(monkeypatch):
    from alicia_flexible_grasp.robot.observation_preview import encode_path_evidence
    from test_remote_grasp6d_streaming import streaming_node, MutableClock, snapshot, remote_node
    from alicia_flexible_grasp.grasp.grasp6d_pipeline import MoveItResult
    node = streaming_node(clock=MutableClock(50.), start_worker=False)
    data = json.loads(FIXTURE.read_text()); xml = URDF.read_text()
    params = {'/robot_description': xml, '/alicia_controller/constraints': {
        'Joint%d'%i: {'trajectory': .12, 'goal': .035} for i in range(1, 7)}}
    monkeypatch.setattr(remote_node.rospy, 'get_param', lambda key, default=None: params.get(key, default))
    try:
        node.start_streaming(); node.submit_stream_snapshot(snapshot(49.8))
        ticket = node._stream_worker_ticket
        node._configured_execution_plan_validity_sec = lambda: 120.
        node.mujoco_selection_snapshot_reserve_sec = 30.
        node.observation_following_guard_required = True
        node.gripper_geometry = gripper()
        variants, results = [], {}
        names = ['Joint%d'%i for i in range(1, 7)]
        start = data['executed'][0]['hold']
        hold = point(start, v=[0.]*6, a=[0.]*6)
        for index, row in enumerate(data['candidates']):
            pose = remote_node.PoseStamped(); pose.pose.position.x = .1*index
            variants.append(dict(row, sequence=NS(pregrasp=pose), preference_index=index))
            message = ('planned execution_duration_lower_bound_sec=%s observation_goal_joint_positions=%s '
                       'observation_joint_names=%s observation_robot_description_sha256=%s' % (
                row['execution_duration_lower_bound_sec'], json.dumps(row['q'], separators=(',', ':')),
                json.dumps(names, separators=(',', ':')), data['model_sha256']))
            plan = trajectory([point(start, 0., [0.]*6, [0.]*6),
                               point(row['q'], 90., [0.]*6, [0.]*6)], names)
            message += ' observation_path_evidence=' + encode_path_evidence(
                pose, plan, hold, .05, 'preserved', 123, data['model_sha256'])
            results[id(pose)] = MoveItResult(reachable=True, joint_path_cost=1., joint_max_delta_rad=1., reason=message)
        node._strict_moveit_evaluation = lambda pose, observation=False: (results[id(pose)], {}, '')
        runtime = {'prepared': NS(ticket=ticket, geometry=NS(
            support_normal_base=np.array(data['normal']), support_offset_m=data['offset'],
            center_base=data['center'], axes_base=data['rotation'], size_xyz_m=data['size'])),
            'grasp_pose': remote_node.PoseStamped(), 'observation_sequence': variants[0]['sequence'],
            'observation_variants': variants, 'soft_evidence': {}}
        node._stable_variant_runtime = {(3, 1): runtime}
        result = node._check_moveit_stable_candidate(NS(track_id=3, variant_index=1))
        assert result.reachable
        search = runtime['observation_orientation_search']
        assert search['selected_camera_roll_offset_deg'] == 180.
        assert search['selected_execution_duration_lower_bound_sec'] == 38.986
        assert search['selected_screened_duration_sec'] == 90.
        assert search['variant_results'][0]['failure_code'] == 'OBSERVATION_FOLLOWING_SUPPORT_INVALID'
        assert search['variant_results'][0]['following_support']['ok'] is False
        assert search['variant_results'][0]['following_support']['failure_location'] == 'goal'
        # Missing names/hash and stale model metadata must not reuse that pass.
        bad, audit = node._observation_following_support_evaluation(runtime['prepared'],
            MoveItResult(reachable=True, joint_path_cost=0., joint_max_delta_rad=0.,
                         reason='planned', evidence_code='STRICT_SERVICE_SUCCESS'))
        assert not bad.reachable and not audit['ok']
        from alicia_flexible_grasp.grasp.grasp6d_pipeline import _validated_moveit_result
        validated = _validated_moveit_result(bad)
        assert validated is not None
        assert validated.failure_code == 'OBSERVATION_FOLLOWING_SUPPORT_INVALID'
        assert validated.evidence_code == ''
    finally:
        node.shutdown_streaming_worker()


@pytest.mark.parametrize('code', ['OBSERVATION_START_FOLLOWING_SUPPORT_INVALID',
                                  'OBSERVATION_FOLLOWING_SUPPORT_INVALID'])
def test_following_failure_labels_are_rejection_only(code):
    from dataclasses import replace
    from alicia_flexible_grasp.grasp.grasp6d_pipeline import MoveItResult, _validated_moveit_result
    failed = MoveItResult(reachable=False, joint_path_cost=0., joint_max_delta_rad=0.,
                         reason='frozen support proof rejected', failure_code=code)
    assert _validated_moveit_result(failed).failure_code == code
    assert _validated_moveit_result(replace(failed, reachable=True)) is None
    assert _validated_moveit_result(replace(failed, evidence_code='STRICT_SERVICE_SUCCESS')) is None


def test_far_field_start_rejection_stops_only_current_request():
    from test_remote_grasp6d_streaming import streaming_node, MutableClock
    from alicia_flexible_grasp.grasp.grasp6d_pipeline import MoveItResult
    node = streaming_node(clock=MutableClock(50.), start_worker=False)
    try:
        results = iter([
            MoveItResult(False, 0., 0., 'endpoint rejected',
                         failure_code='OBSERVATION_FOLLOWING_SUPPORT_INVALID'),
            MoveItResult(False, 0., 0., 'shared start rejected',
                         failure_code='OBSERVATION_START_FOLLOWING_SUPPORT_INVALID')])
        node._check_moveit_stable_candidate = lambda c: next(results)
        checker, can_continue = node._far_field_observation_search()
        assert can_continue()
        checker(None)
        assert can_continue()  # Another endpoint may still work.
        checker(None)
        assert not can_continue()
        _, next_request = node._far_field_observation_search()
        assert next_request()  # No cached rejection after realignment.
    finally:
        node.shutdown_streaming_worker()
