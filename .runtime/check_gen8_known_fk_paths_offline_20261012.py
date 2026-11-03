"""Offline original-guard screening of two saved raw MoveIt trajectories.

No node, service, publisher, retiming RPC, or execution. This does not create a
committed scene, controller hold receipt, tracking contract, or motion authority.
"""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import time
from types import SimpleNamespace

ANALYSIS = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_frozen_no_skin_20261012T021050Z/gen8_analysis')
EXPECTED = '7812d6b69e854ed9f37b4e3cbd813ecd53dab3504065b2d04d287abc6fd852e0'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    assert os.environ['ROS_MASTER_URI'] == 'http://127.0.0.1:1'
    out = ANALYSIS / 'known_fk_paths_offline_original_guards'
    out.mkdir(exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(out / 'ros')
    import numpy as np
    from genpy.message import fill_message_args
    from moveit_msgs.msg import RobotTrajectory
    from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
    from alicia_flexible_grasp.robot import observation_path_guard as path_guard
    from alicia_flexible_grasp.robot import observation_view_guard as view_guard
    from alicia_flexible_grasp.robot import observation_preview as preview
    from alicia_flexible_grasp.robot import observation_tracking_contract as tracking
    from alicia_grasp_modes import observation_arrival_projection as arrival
    source_path = ANALYSIS / 'fk_replay_inputs/source.json'
    arrays_path = ANALYSIS / 'fk_replay_inputs/source.npz'
    source = json.loads(source_path.read_text())
    arrays = np.load(arrays_path, allow_pickle=False)
    record = source['source']
    model_path = Path(source['model_path'])
    assert sha(model_path) == source['model_sha256'] == EXPECTED
    assert all(row['match'] for row in source['source_hashes'].values())
    for key, expected_key in [('tool_from_camera', 'tool_camera_sha256'),
                              ('points_base', 'support_points_sha256'),
                              ('corners_base', 'conservative_target_corners_sha256')]:
        assert hashlib.sha256(np.ascontiguousarray(arrays[key]).tobytes()).hexdigest() == record['audit'][expected_key]
    names = ['Joint%d' % i for i in range(1, 7)]
    fk = path_guard.SerialUrdfFk(model_path.read_text(), names)
    geometry = record['geometry']
    scene = preview.candidate_scene(SimpleNamespace(
        support_normal_base=geometry['support_normal_base'], support_offset_m=geometry['support_offset_m'],
        center_base=geometry['obb_center_base_m'], axes_base=geometry['R_base_obb'],
        size_xyz_m=geometry['obb_size_xyz_m']), record['binding']['snapshot_stamp_ns'])
    gripper = GripperGeometry(.05, .002, np.array([.0434, .0286, .0600]),
                              np.array([.1175, .1550, .0774]), .003)
    opening = source['opening_width_m']
    assert opening == .05
    errors = source['joint_error_bounds_rad']
    assert errors == [0.06601699039494283] * 6
    hold = SimpleNamespace(positions=source['current_joint_positions'],
                           velocities=[0.] * 6, accelerations=[0.] * 6)
    camera = record['camera']
    intrinsics = [camera[k] for k in ('fx', 'fy', 'cx', 'cy')]
    limits = record['view_limits']['effective_visibility_limits']
    contract = {key: arrays[key] for key in ('prism_base', 'rgb_valid', 'invalid_integral')}
    contract['audit'] = source['arrival_audit']
    modules = [path_guard, view_guard, preview, tracking, arrival]
    imports = {module.__name__: {'path': module.__file__, 'sha256': sha(Path(module.__file__))} for module in modules}
    result = {
        'created_at': datetime.now(timezone.utc).isoformat(),
        'script': str(Path(__file__).resolve()), 'script_sha256': sha(Path(__file__)),
        'source': {'path': str(source_path), 'sha256': sha(source_path)},
        'arrays': {'path': str(arrays_path), 'sha256': sha(arrays_path)},
        'imports': imports, 'robot_description_sha256': fk.model_sha256,
        'scope': 'Conditional original geometry/view guards on saved raw GetMotionPlan curves; not final retimed command acceptance.',
        'physical_actions': False, 'ros_service_calls': 0, 'motion_authority': False,
        'input_assumptions': {
            'hold': 'Historical first-view measured q with zero derivatives for conditional bridge; no current controller-hold receipt.',
            'opening_width_m': opening,
            'opening_provenance': 'Root exact-source configured proposal opening, separate from current/historical measured finger opening.',
            'joint_error_bounds_rad': errors,
            'tracking_provenance': 'Original recorded minimum proposal contract bound, not a newly issued per-goal contract.',
            'scene': 'Original candidate_scene geometry and float32 support-offset wire semantics; no committed plan ID invented.',
            'timing': 'Saved GetMotionPlan timing, velocities and accelerations unchanged; original strict retime not available.',
        },
        'budgets': {'following_max_checks': path_guard.FOLLOWING_PATH_MAX_CHECKS,
                    'following_max_seconds': path_guard.FOLLOWING_PATH_MAX_SECONDS,
                    'following_max_depth': 20,
                    'view_max_time_step_sec': .05, 'view_max_joint_span_rad': .02,
                    'view_max_samples': 16384, 'view_max_depth': 20, 'view_max_seconds': 5.,
                    'visibility_limits': limits},
        'original_order': [
            'strict retime and same-geometry verification, including smooth unknown timing if eligible',
            'controller-reference bridge timing and derivative checks; encode bound preview evidence',
            'remote path target visibility, endpoint measured-support projection and arrival prism/RGB projection',
            'remote following/stop contract search plus continuous CAD/support/target path',
            'at submission repeat final path, source/hold/model/epoch and authority validations'],
        'missing_for_original_acceptance': [
            'Actual original strict-retimed trajectory and controller-reference timing evidence',
            'Fresh measured opening and proven stationary controller hold/epoch',
            'Full configured contract candidate search, stop-duration and exact issued per-goal tolerance',
            'Live source/ticket/committed plan identity and admission/approval',
            'Actual measured arrival and hardware tracking/stopping/calibration verification'],
        'candidates': [],
    }
    save(out / 'inputs.json', {k:v for k,v in result.items() if k != 'candidates'})
    for index in range(2):
        response_path = ANALYSIS / ('known_fk_joint_plans_current_scene/candidate_%d_response.json' % index)
        raw = json.loads(response_path.read_text())['motion_plan_response']['trajectory']
        trajectory = RobotTrajectory()
        fill_message_args(trajectory, [raw])
        before = path_guard.wire_digest(trajectory)
        row = {'candidate_index': index, 'response_path': str(response_path),
               'response_sha256': sha(response_path), 'trajectory_sha256': before,
               'point_count': len(trajectory.joint_trajectory.points)}
        def check(label, callback):
            started = time.monotonic()
            try:
                value = callback()
                row[label] = {'returned': True, 'elapsed_sec': time.monotonic()-started, 'report': value}
            except (AttributeError, KeyError, TypeError, ValueError, OverflowError) as exc:
                row[label] = {'returned': False, 'elapsed_sec': time.monotonic()-started,
                              'error_type': type(exc).__name__, 'reason': str(exc),
                              'audit': getattr(exc, 'audit', {})}
            save(out / ('candidate_%d.json' % index), row)
            print(json.dumps({'candidate': index, 'stage': label,
                              'returned': row[label]['returned'],
                              'ok': row[label].get('report', {}).get('ok'),
                              'elapsed_sec': row[label]['elapsed_sec'],
                              'reason': row[label].get('reason', row[label].get('report', {}).get('reason', ''))}), flush=True)
        check('view_and_measured_support', lambda: view_guard.validate_observation_view_trajectory(
            trajectory, fk, arrays['tool_from_camera'], arrays['corners_base'], arrays['center_base'],
            arrays['points_base'], intrinsics, camera['width'], camera['height'], hold, **limits))
        check('arrival_projection_endpoint', lambda: arrival.validate_arrival_projection(
            contract, fk(trajectory.joint_trajectory.points[-1].positions) @ arrays['tool_from_camera'],
            intrinsics, camera['width'], camera['height'], **limits))
        check('raw_command_derivative_contract', lambda: tracking.validate_command_derivatives(trajectory, hold))
        check('original_following_cad_support_target_path', lambda: preview.qualify_candidate_path(
            trajectory, hold, scene, fk, gripper, opening, errors))
        row['trajectory_unchanged'] = path_guard.wire_digest(trajectory) == before
        result['candidates'].append(row)
        save(out / ('candidate_%d.json' % index), row)
        save(out / 'partial.json', result)
    result['source_imports_unchanged'] = all(sha(Path(value['path'])) == value['sha256'] for value in imports.values())
    result['input_files_unchanged'] = sha(source_path) == result['source']['sha256'] and sha(arrays_path) == result['arrays']['sha256']
    save(out / 'result.json', result)
    print(json.dumps({'output': str(out / 'result.json'), 'sha256': sha(out / 'result.json')}, indent=2))


if __name__ == '__main__':
    main()
