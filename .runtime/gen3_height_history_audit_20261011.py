"""Offline history/source and identical-cloud checks; no ROS interfaces."""
import ast
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

WORK = Path(__file__).resolve().parents[1]
EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
OUT = EXT / 'ros_log/full_latest_no_skin_20261011T000625Z/gen3_analysis'
if not Path('/media/zhuyupei/SanStand').is_mount():
    raise SystemExit('SanStand is not mounted')
sys.path.insert(0, str(WORK / 'src/alicia_flexible_grasp_supervisor/src'))
from alicia_flexible_grasp.grasp.gripper_geometry import (
    GripperGeometry, evaluate_bilateral_surface_evidence_and_bounds,
)
from alicia_flexible_grasp.vision.multiview_surface import (
    SurfaceView, append_registered_view, fused_surface_from_view,
)
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def functions(text):
    lines = text.splitlines(True)
    return {node.name: ''.join(lines[node.lineno - 1:node.end_lineno])
            for node in ast.parse(text).body if isinstance(node, ast.FunctionDef)}


relative = 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/gripper_geometry.py'
old = subprocess.check_output(['git', 'show', 'fd3f9b1:' + relative], cwd=str(WORK), text=True)
current = (WORK / relative).read_text()
old_functions, current_functions = functions(old), functions(current)
function_history = {}
for name in ('_bilateral_surface_measurement', 'evaluate_bilateral_surface_evidence_and_bounds',
             'finger_contact_patch_overlap_m'):
    function_history[name] = {
        'identical_to_20260923_commit_fd3f9b1': old_functions[name] == current_functions[name],
        'function_sha256': hashlib.sha256(current_functions[name].encode()).hexdigest(),
    }

geometry = GripperGeometry(
    max_inner_gap_m=.050, jaw_clearance_each_side_m=.002,
    finger_size_xyz_m=np.array([.0434, .0286, .060]),
    palm_size_xyz_m=np.array([.1175, .1550, .0774]), support_clearance_m=.003)
physical_span = max(geometry.finger_size_xyz_m[0], geometry.max_inner_gap_m +
                    2 * geometry.finger_size_xyz_m[1]) + 2 * geometry.support_clearance_m
identity = TargetTrackIdentity.from_stream(7, 11)
xs = np.linspace(-.018, .018, 17)
ys = np.linspace(-.0175, .0175, 15)
zs = np.linspace(.004, .022, 17)
points = np.array([(x, y, .022) for x in xs for y in ys] +
                  [(x, side, z) for x in xs for side in (-.0175, .0175) for z in zs])
reference = SurfaceView(identity, 1_000_000_000, points, np.array([0., 0., 1.]), 0., 12)
moving = SurfaceView(identity, 1_100_000_000, points.copy(), np.array([0., 0., 1.]), 0., 12)
single = fused_surface_from_view(reference, voxel_size_m=.0015)
registration, fused = append_registered_view(single, reference, moving, voxel_size_m=.0015)
assert registration.ok, registration.code
cloud_cases = []
for name, surface in [('single_current_view', single), ('same_cloud_registered_second_view', fused)]:
    evidence, bounds = evaluate_bilateral_surface_evidence_and_bounds(
        surface, np.array([0., 0., .013]), np.array([0., 1., 0.]),
        np.array([0., 0., -1.]), geometry, minimum_points_per_side=12,
        support_normal_base=reference.support_normal_base)
    cloud_cases.append(dict(name=name, point_count=len(surface.points_base),
                            evidence=asdict(evidence), bounds_m=bounds))
assert np.allclose(cloud_cases[0]['bounds_m'], cloud_cases[1]['bounds_m'], atol=1e-10, rtol=0.)

old_record = EXT / 'grasp_records/20261006T061242.642572Z_f8b97892d0a84d5599287c74677f32d2'
old_audit = old_record / 'keyframes_audits/1d840d212badd422d1c84c2cfb21f1748a9bc45af6f12e30735dbaf6ad9f3949.json'
success = json.loads(old_audit.read_text())
latest_audit = EXT / 'grasp_records/20261011T010558.427673Z_e0beb4e762d74e7895fa100cec6eaaba/keyframes_audits/9f171e96074e00ed1fa090120f9a00b0fcace08313a8dc744a593439780847d0.json'
latest = json.loads(latest_audit.read_text())
support_dir = EXT / 'ros_log/handeye_auto_20260927T102700Z/deployment_20260927T111819Z'
support_report = support_dir / 'production_support_height_replay.json'
support = json.loads(support_report.read_text())
candidate_flags = [row['feasible'] for row in support['candidates']]
before_sources = {str(WORK / p): sha(WORK / p) for p in (
    relative,
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/object_geometry.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/multiview_surface.py',
    'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py',
    'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py')}
report = dict(
    scope='offline_readonly_history_and_same_cloud_branch_equivalence_not_real_GEN3_fused_replay',
    production_modified=False, ros_api_used=False, motion_authorized=False,
    history_20260927=dict(
        report=str(support_report), sha256=sha(support_report),
        historical_physical_span_m=support['physical_support_span_m'],
        current_hardware_derived_span_m=float(physical_span),
        candidate_count=len(candidate_flags), feasible_count=sum(bool(x) for x in candidate_flags),
        current_entry='remote_grasp6d_node.py:5491 calls estimate_object_geometry before near-field branch',
        current_mask='object_geometry.py:357-390 retains physical span and expanded & ~foreground & ~box',
        historical_success_claim='32-candidate replay is offline; actual successful grabs are separate operator-confirmed records'),
    prior_success=dict(
        audit=str(old_audit), audit_sha256=sha(old_audit), plan_id=success['plan_id'],
        origin=success['planning_origin'], surface=success['multiview_surface'],
        object_size_m=success['replay_geometry']['obb_size_xyz_m'],
        selected_tilt_deg=success['selected']['adaptive_stage_profile']['tilt_deg'],
        planned_overlap_m=success['selected']['final_contact_height_margin']['nominal']['actual_overlap_m'],
        symmetric_margin_m=success['selected']['final_contact_height_margin']['maximum_symmetric_normal_translation_m'],
        confirmation=str(EXT / 'ros_log/grasp_close_hold_20261006T061242Z/operator_confirmed_result.json'),
        result='operator confirmed physical hold of new target, no lift; software HOLD_UNVERIFIED preserved'),
    current_gen3=dict(
        audit=str(latest_audit), audit_sha256=sha(latest_audit),
        origin=latest['planning_origin'], object_size_m=latest['replay_geometry']['obb_size_xyz_m'],
        actual_fused_point_count=latest['multiview_surface']['point_count'],
        available_replay_points=latest['replay_geometry']['object_points_count'],
        full_fused_points_available=False,
        separate_quantities=dict(whole_object_height_m=.022856678630821557,
            exact_minor_support_normal_common_band_m=.000874010747552093,
            best_exact_minor_insertion_pad_overlap_m=.0016064540245),
        limitation='Current-frame points are not the 1567 stored fused points. No exact side-loss or full same-scene pipeline replay claimed.'),
    measurement_history=function_history,
    quantiles=dict(lower=.05, upper=.95,
        note='Shared support-normal common extent followed by insertion-axis extent existed before historical successes; no view-count conditional trimming'),
    same_cloud_check=dict(
        provenance='constructed generic legal measured side cloud, not a historical scene or invented production target',
        registration_ok=registration.ok, registration_code=registration.code,
        registration_transform=registration.transform_base.tolist(), cases=cloud_cases,
        max_bounds_difference_m=float(np.max(np.abs(np.array(cloud_cases[0]['bounds_m']) - np.array(cloud_cases[1]['bounds_m'])))),
        limitation='Exercises actual single-view seeding, registration append and shared measurement, not a complete ROS or current GEN3 scene replay'),
    source_sha256=before_sources,
    conclusion='No reproduced loss of the historical support-height fix or supplementary-only double truncation. Same measured points preserve height bounds in this controlled comparison; real GEN3 surface completeness remains unproven.')
OUT.mkdir(parents=True, exist_ok=True)
target = OUT / 'historical_height_fix_current_branch_audit.json'
target.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
print(json.dumps({'report': str(target), 'sha256': sha(target),
                  'identical_cloud_cases': cloud_cases,
                  'historical_feasible': sum(bool(x) for x in candidate_flags)}, ensure_ascii=False))
