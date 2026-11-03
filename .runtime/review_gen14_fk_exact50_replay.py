"""Independent exact-source 50 mm replay. No ROS master/service use."""
import argparse
from copy import deepcopy
from dataclasses import asdict
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time

import numpy as np
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk

WORK = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
STAGE = WORK / '.runtime/gen14_fk_early_qualified_20261011'
FIXTURE = STAGE / 'src/alicia_grasp_modes/tests/fixtures'
NEW = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_no_skin_20261011T000625Z')
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
arrsha = lambda a: hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()


def load():
    metadata_path = FIXTURE / 'gen14_fk_source.json'
    npz_path = FIXTURE / 'gen14_fk_source.npz'
    metadata = json.loads(metadata_path.read_text())
    assert sha(npz_path) == metadata['arrays_sha256']
    original_path = Path(metadata['source_audit'])
    assert sha(original_path) == metadata['source_audit_sha256']
    original = json.loads(original_path.read_text())
    diagnostics = original['observation_failure_diagnostics']
    captured = diagnostics['source']['captured_audit']
    assert metadata['source']['audit'] == captured
    assert metadata['source']['binding'] == diagnostics['source']['captured_binding']
    assert metadata['source']['geometry'] == original['replay_geometry']
    assert metadata['gaps'] == original['observe_return']['measured_missing_sides']
    assert metadata['current_joint_positions'] == diagnostics['joint_space_fk_search']['audit']['current_joint_positions']
    assert metadata['joint_error_bounds_rad'] == diagnostics['joint_space_fk_search']['audit']['joint_error_bounds_rad']
    assert metadata['arrival_audit'] == captured['arrival_projection']
    assert metadata['source']['camera'] == captured['camera_parameters']
    assert metadata['source']['view_limits'] == captured['view_limits']
    assert metadata['opening_width_m'] == .05
    with np.load(str(npz_path)) as z:
        arrays = {k: z[k].copy() for k in z.files}
    checked_hashes = {}
    for key, hash_key in [('points_base', 'support_points_sha256'),
                          ('corners_base', 'conservative_target_corners_sha256'),
                          ('tool_from_camera', 'tool_camera_sha256')]:
        assert arrsha(arrays[key]) == captured[hash_key]
        checked_hashes[key] = arrsha(arrays[key])
    for key, hash_key in [('prism_base', 'prism_sha256'), ('rgb_valid', 'rgb_valid_sha256'),
                          ('invalid_integral', 'invalid_integral_sha256')]:
        assert arrsha(arrays[key]) == captured['arrival_projection'][hash_key]
        checked_hashes[key] = arrsha(arrays[key])
    assert len(arrays['points_base']) == captured['support_point_count'] == 141094
    assert arrays['center_base'].tolist() == original['replay_geometry']['obb_center_base_m']
    source = deepcopy(metadata['source'])
    source.update({k: arrays[k] for k in ('points_base', 'corners_base', 'center_base', 'tool_from_camera')})
    source['arrival_projection'] = {k: arrays[k] for k in ('prism_base', 'rgb_valid', 'invalid_integral')}
    source['arrival_projection']['audit'] = metadata['arrival_audit']
    model_path = STAGE / metadata['model_relative_path']
    fk = SerialUrdfFk(model_path.read_text(), ['Joint%d' % i for i in range(1, 7)])
    assert sha(model_path) == metadata['model_sha256'] == fk.model_sha256
    gripper = GripperGeometry(.05, .002, np.array([.0434, .0286, .06]), np.array([.1175, .155, .0774]), .003)
    startup_path = NEW / 'core.log'
    startup_lines = startup_path.read_text(errors='replace').splitlines()
    wanted = ('/gripper/open_position_m:', '/grasp_6d/remote/gripper_geometry/',
              '/grasp_6d/remote/candidate_min_finger_support_clearance_m:')
    startup_parameters = [dict(line=i + 1, text=line) for i, line in enumerate(startup_lines)
                          if any(token in line for token in wanted)]
    assert any('/gripper/open_position_m: 0.05' in item['text'] for item in startup_parameters)
    assert any('/gripper_geometry/max_inner_gap_m: 0.05' in item['text'] for item in startup_parameters)
    args = dict(source=source, current_joint_positions=metadata['current_joint_positions'], fk=fk,
        gripper=gripper, opening_width_m=.05, gaps=metadata['gaps'], depth_uncertainty_m=.003,
        side_depth_uncertainty_m=.003, joint_error_bounds_rad=metadata['joint_error_bounds_rad'])
    provenance = dict(source_audit=str(original_path), source_audit_sha256=sha(original_path),
        fixture_metadata_sha256=sha(metadata_path), fixture_arrays_sha256=sha(npz_path),
        checked_array_sha256=checked_hashes, exact_support_point_count=141094,
        opening_width_m=.05, opening_provenance=metadata['opening_provenance'],
        per_GEN14_full_gripper_parameter_snapshot_available=False,
        startup_parameter_evidence=dict(path=str(startup_path), lines=startup_parameters,
            selected_lines_sha256=hashlib.sha256(json.dumps(startup_parameters, sort_keys=True).encode()).hexdigest()),
        opening_review_note='Same-session startup core.log records open_position_m=.05 and max_inner_gap_m=.05; production caller uses their minimum. 49.8 mm was encoder feedback. Full CAD dimensions are corroborated by source YAML; no separate per-GEN14 full parameter snapshot is claimed.',
        gripper=asdict(gripper), current_joint_positions=metadata['current_joint_positions'],
        joint_error_bounds_rad=metadata['joint_error_bounds_rad'], source_binding=source['binding'],
        source_geometry=source['geometry'], model_path=str(model_path), model_sha256=fk.model_sha256,
        view_limits=source['view_limits'], all_endpoint_uncertainties_m=.003,
        original_live_search=dict(candidate_count=diagnostics['joint_space_fk_search']['audit']['candidate_count'],
            elapsed_sec=diagnostics['joint_space_fk_search']['audit']['elapsed_sec'],
            status=diagnostics['joint_space_fk_search']['audit']['status']))
    # Omit the target cloud from this small report; exact source hash already binds it.
    provenance['source_geometry'] = {k: v for k, v in provenance['source_geometry'].items() if k != 'object_points_base_m'}
    return args, provenance


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--module-path', required=True)
    parser.add_argument('--name', required=True)
    parser.add_argument('--constraint-work-sec', type=float, default=0.)
    options = parser.parse_args()
    assert Path('/media/zhuyupei/SanStand').is_mount()
    path = Path(options.module_path)
    args, provenance = load()
    spec = importlib.util.spec_from_file_location('alicia_grasp_modes._independent_gen14_fk_review', path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    code_sha = sha(path)
    work_clock = dict(now=0., constraint_callbacks=0, solver_entries=0)
    if options.constraint_work_sec:
        assert options.constraint_work_sec == .002
        from scipy.optimize import minimize
        def backend(objective, seed, **kwargs):
            work_clock['solver_entries'] += 1
            original_constraint = kwargs['constraints'][0]['fun']
            def metered(q):
                work_clock['constraint_callbacks'] += 1
                work_clock['now'] += options.constraint_work_sec
                return original_constraint(q)
            kwargs['constraints'] = [dict(type='ineq', fun=metered)]
            return minimize(objective, seed, **kwargs)
        module._backend = lambda: backend
        args['clock'] = lambda: work_clock['now']
    began = time.monotonic()
    candidates, audit = module.generate_reachable_observation_views(**args)
    wall = time.monotonic() - began
    assert sha(path) == code_sha
    recalculated = []
    source, g = args['source'], args['source']['geometry']
    camera = source['camera']
    intrinsics = [camera[k] for k in ('fx', 'fy', 'cx', 'cy')]
    limits = source['view_limits']['effective_visibility_limits']
    for row in candidates:
        q = np.asarray(row['q']); tool = args['fk'](q); cam = tool @ source['tool_from_camera']
        assert np.allclose(tool, row['T_base_tool0'], rtol=0., atol=1e-13)
        checks = dict(
            missing_side_view_gain=module.missing_side_view_gain(args['gaps'], cam[:3, 3], .003),
            working_volume=module.validate_planned_observation_working_volume(source['corners_base'], cam,
                intrinsics, camera['width'], camera['height'], snapshot_stamp_ns=source['binding']['snapshot_stamp_ns'], **limits),
            observation_envelope=asdict(module.evaluate_open_gripper_observation_envelope(gripper=args['gripper'],
                T_base_tool0=tool, opening_width_m=.05, support_normal_base=g['support_normal_base'],
                support_offset_m=g['support_offset_m'], obb_center_base=source['center_base'],
                R_base_obb=g['R_base_obb'], obb_size_xyz_m=g['obb_size_xyz_m'])),
            conditional_endpoint_support=module.observation_endpoint_support_bound(args['fk'], q,
                args['joint_error_bounds_rad'], g['support_normal_base'], g['support_offset_m'], args['gripper'], .05),
            measured_support_projection=module.validate_observation_support_projection(cam, source['points_base'],
                source['center_base'], intrinsics, camera['width'], camera['height'], target_corners_base=source['corners_base'], **limits),
            optical_side_evidence=module._optical_side_evidence(cam, np.asarray(g['support_normal_base']),
                np.asarray(g['R_base_obb']), np.asarray(g['obb_size_xyz_m']), .003),
            arrival_projection_endpoint=module.validate_arrival_projection(source['arrival_projection'], cam,
                intrinsics, camera['width'], camera['height'], **limits))
        assert all(check['ok'] for check in checks.values()), checks
        assert all(row[key] is False for key in ('execution_authority', 'whole_arm_collision_checked', 'complete_path_checked'))
        assert row['binding'] == source['binding']
        recalculated.append(dict(q=row['q'], all_original_endpoint_checks_passed=True, checks=checks))
    report = dict(source_module=str(path), source_module_sha256=code_sha, input_verification=provenance,
        clock=('controlled 0.002 s per real inequality callback; actual SLSQP and endpoint gates; not a wall-clock performance benchmark'
            if options.constraint_work_sec else 'real time.monotonic; no instrumentation of solver, constraints or clock'),
        deterministic_work_clock=work_clock if options.constraint_work_sec else None,
        wall_elapsed_sec=wall, max_seconds=2., audit=audit, candidates=candidates,
        independent_candidate_rechecks=recalculated, physical_actions=False,
        limitations=['Endpoint checks only; complete MoveIt path and following proof remain untested.',
            '50 mm opening has same-session startup parameter evidence; complete CAD values are source-config corroborated, not a separately captured per-GEN14 full parameter snapshot.',
            'Wall-clock results depend on runtime load. Historical failure, controlled-work-clock regression, and this independent wall-clock run are separate evidence.',
            'No live ROS service, motion or production source edit.'])
    target = NEW / 'gen14_analysis' / (options.name + '.json')
    assert not target.exists()
    target.write_text(json.dumps(report, indent=2, default=lambda v: v.tolist() if isinstance(v, np.ndarray) else str(v)) + '\n')
    print(json.dumps(dict(path=str(target), sha256=sha(target), candidate_count=len(candidates),
        wall_elapsed_sec=wall, status=audit['status'], all_returned_endpoints_rechecked=True)))


if __name__ == '__main__':
    main()
