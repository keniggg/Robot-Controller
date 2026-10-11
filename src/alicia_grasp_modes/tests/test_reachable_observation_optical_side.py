"""Preserve the original optical-side gate inside bounded FK proposals."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_remote_modes import base, configured, module as remote_module
from test_reachable_observation_views import recorded_inputs
from test_reachable_observation_integration import context, recorded_proposal
from alicia_grasp_modes import reachable_observation_views as module


@pytest.mark.parametrize('angles', [(0., .12, -.35), (.23, -.47, 1.02), (-.61, .72, -1.15)])
def test_projection_matches_original_with_tilted_support_and_nonidentity_handeye(angles):
    tool = base.quaternion_matrix(base.quaternion_from_euler(*angles))
    mount = base.quaternion_matrix(base.quaternion_from_euler(.3, -.2, .8))
    mount[:3, 3] = [.018, -.022, .09]
    rotation = base.quaternion_matrix(base.quaternion_from_euler(.04, -.03, .8))[:3, :3]
    normal = np.array([.08, -.06, 1.]); normal *= 1.7
    size = np.array([.061, .036, .022])
    geometry = NS(support_normal_base=normal, axes_base=rotation, size_xyz_m=size)
    stub = NS(_tool_from_camera_matrix=lambda: mount,
        _geometry_object_height_m=base.RemoteGrasp6DNode._geometry_object_height_m)
    pose = base.make_pose_stamped('base_link', tool[:3, 3], base.quaternion_from_matrix(tool))
    expected = base.RemoteGrasp6DNode._observation_side_evidence(stub, geometry, pose,
        NS(depth_uncertainty_m=.003))
    actual = module._optical_side_evidence(base.pose_matrix(pose) @ mount, normal, rotation, size, .003)
    for key, value in expected.items():
        assert actual[key] == (pytest.approx(value, abs=1e-14) if isinstance(value, float) else value)
    assert actual['ok'] is (expected['observation_side_evidence_deficit_m'] == 0.)


@pytest.mark.parametrize('bad', [True, float('nan'), float('inf'), -.001, .051])
def test_invalid_current_side_uncertainty_never_starts_solver(recorded_inputs, monkeypatch, bad):
    monkeypatch.setattr(module, '_backend', lambda: pytest.fail('invalid current side noise reached solver'))
    rows, audit = module.generate_reachable_observation_views(**recorded_inputs, side_depth_uncertainty_m=bad)
    assert not rows and audit['status'] == 'invalid_input'
    assert not audit['execution_authority']


def test_gen28_record_keeps_original_gates_with_new_side_constraint(recorded_inputs):
    rows, audit = module.generate_reachable_observation_views(**recorded_inputs)
    assert rows and audit['status'] == 'complete', audit
    assert audit['optical_side_uncertainty_m'] == recorded_inputs['depth_uncertainty_m']
    assert audit['max_seconds'] == 2. and audit['max_iterations'] == 96
    for row in rows:
        checks = row['endpoint_checks']
        for name in ('missing_side_view_gain', 'working_volume', 'observation_envelope',
                'conditional_endpoint_support', 'measured_support_projection', 'optical_side_evidence'):
            assert checks[name]['ok']
        assert checks['optical_side_evidence']['observation_side_evidence_deficit_m'] == 0.
        assert not row['whole_arm_collision_checked'] and not row['complete_path_checked']


def test_current_profile_noise_is_separate_cache_binding_and_cannot_renew_budget(
        configured, monkeypatch, recorded_inputs, recorded_proposal):
    node, params, _, prepared, _ = context(configured, monkeypatch, recorded_inputs)
    names = ['Joint%d' % i for i in range(1, 7)]
    params['/alicia_controller/constraints'] = {name: {'trajectory': .2, 'goal': .035} for name in names}
    node.latest_joint_state = NS(name=names, position=list(recorded_inputs['current_joint_positions']),
        header=NS(stamp=prepared.stamp))
    monkeypatch.setattr(remote_module.rospy.Time, 'now', lambda: prepared.stamp)
    current_noise = [.004]
    monkeypatch.setattr(node, '_adaptive_stage_profiles', lambda *_: [
        NS(depth_uncertainty_m=current_noise[0]+.01, execution_position_error_m=.01)])
    calls = []
    def generate(**kwargs):
        calls.append(kwargs)
        return (deepcopy(recorded_proposal),), dict(elapsed_sec=min(1.2, kwargs['max_seconds']))
    monkeypatch.setattr(module, 'generate_reachable_observation_views', generate)
    assert node._joint_space_observation_candidates(prepared)[0]
    assert calls[0]['depth_uncertainty_m'] == .003
    assert calls[0]['side_depth_uncertainty_m'] == pytest.approx(.004)
    assert node._joint_space_observation_candidates(prepared)[1]['cache_hit']
    current_noise[0] = .005
    assert node._joint_space_observation_candidates(prepared)[0]
    assert calls[1]['depth_uncertainty_m'] == .003
    assert calls[1]['side_depth_uncertainty_m'] == pytest.approx(.005)
    assert calls[1]['max_seconds'] == pytest.approx(.8)
    current_noise[0] = .006
    rows, audit = node._joint_space_observation_candidates(prepared)
    assert not rows and audit['status'] == 'budget_exhausted' and len(calls) == 2


def test_sealed_gen14_original_proposals_pass_other_endpoints_but_fail_original_optical_gate():
    path = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/'
        '20261009T050350.036425Z_ce203c7b9d4c4ba29e4303eb51b69d62/keyframes_audits/'
        'b8addcd0918bfbffd80a832646845834de2aa73078746db6eb867911e16c8031.json')
    if not path.exists(): pytest.skip('sealed gen14 source is not distributed with code')
    report = json.loads(path.read_text()); geometry = report['replay_geometry']
    candidates = report['observation_failure_diagnostics']['joint_space_fk_search']['candidates']
    assert len(candidates) == 2
    for row in candidates:
        for gate in ('missing_side_view_gain', 'working_volume', 'observation_envelope',
                'conditional_endpoint_support', 'measured_support_projection'):
            assert row['endpoint_checks'][gate]['ok']
        side = module._optical_side_evidence(np.array(row['T_base_camera_link']),
            np.array(geometry['support_normal_base']), np.array(geometry['R_base_obb']),
            np.array(geometry['obb_size_xyz_m']), .003)
        assert not side['ok'] and side['observation_side_evidence_deficit_m'] > .0006


def test_claimed_solver_success_cannot_return_gen14_original_side_deficient_endpoint(monkeypatch):
    from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
    from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
    root = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/'
        'full_latest_ros_20261009T003407Z/gen14_frozen_source')
    if not (root/'source.json').exists(): pytest.skip('sealed local gen14 source unavailable')
    source = json.loads((root/'source.json').read_text())
    with np.load(str(root/'source.npz')) as arrays:
        source.update({key: arrays[key].copy() for key in arrays.files})
    audit = json.loads(Path(source['source_provenance']['audit_path']).read_text())
    source['geometry'] = audit['replay_geometry']
    search = audit['observation_failure_diagnostics']['joint_space_fk_search']
    invalid_q = np.array(search['candidates'][0]['q'])
    fk = SerialUrdfFk(Path(source['source_provenance']['model_path']).read_text(),
        ['Joint%d' % i for i in range(1, 7)])
    probes = []
    def fake_solver(_objective, _seed, **kwargs):
        probes.append(kwargs['constraints'][0]['fun'](invalid_q))
        return NS(x=invalid_q, nit=1, success=True, message='untrusted success despite optical-side deficit')
    monkeypatch.setattr(module, '_backend', lambda: fake_solver)
    rows, report = module.generate_reachable_observation_views(source=source,
        current_joint_positions=search['audit']['current_joint_positions'], fk=fk,
        gripper=GripperGeometry(.05, .002, np.array([.0434, .0286, .06]), np.array([.1175, .155, .0774]), .003),
        opening_width_m=.05, gaps=audit['observe_return']['measured_missing_sides'],
        depth_uncertainty_m=.003, side_depth_uncertainty_m=.003,
        joint_error_bounds_rad=search['audit']['joint_error_bounds_rad'],
        max_seconds=2., max_directions=1)
    assert not rows and probes[0][-1] < 0.
    rejected = report['directions'][0]
    assert rejected['solver_success'] and not rejected['accepted']
    for gate in ('missing_side_view_gain', 'working_volume', 'observation_envelope',
            'conditional_endpoint_support', 'measured_support_projection'):
        assert rejected['endpoint_checks'][gate]['ok']
    assert not rejected['endpoint_checks']['optical_side_evidence']['ok']
