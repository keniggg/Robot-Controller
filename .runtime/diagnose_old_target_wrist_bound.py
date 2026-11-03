#!/usr/bin/env python3
"""Pure offline analytic reach bound for the superseded 2026-09-29 target."""
import hashlib
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log')
URDF = ROOT / 'production_lock_readonly_20260929T045650Z_458657/robot_description.urdf'
AUDIT = ROOT / 'current_grasp_preview_20260929T045714Z/full_gate_audit.json'
OUT = ROOT / 'restart_grasp_20260929T013841Z/old_target_vertical_wrist_bound'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rot(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    K = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + math.sin(angle)*K + (1-math.cos(angle))*(K@K)


def rpy(values):
    r, p, y = values
    return rot([0, 0, 1], y) @ rot([0, 1, 0], p) @ rot([1, 0, 0], r)


def quat(values):
    q = np.asarray(values, dtype=float)
    q /= np.linalg.norm(q)
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def main():
    xml = ET.fromstring(URDF.read_text())
    joints = {j.attrib['name']: j for j in xml.findall('joint')}
    audit = json.loads(AUDIT.read_text())
    transforms, axes, limits = {}, {}, {}
    for name, j in joints.items():
        o = j.find('origin')
        p = np.fromstring(o.get('xyz', '0 0 0'), sep=' ') if o is not None else np.zeros(3)
        R = rpy(np.fromstring(o.get('rpy', '0 0 0'), sep=' ')) if o is not None else np.eye(3)
        T = np.eye(4); T[:3, :3] = R; T[:3, 3] = p
        transforms[name] = T
        a = j.find('axis')
        if a is not None:
            axis = np.fromstring(a.get('xyz'), sep=' ')
            axes[name] = axis / np.linalg.norm(axis) if np.linalg.norm(axis) else axis
        lim = j.find('limit')
        if lim is not None and 'lower' in lim.attrib:
            limits[name] = [float(lim.get('lower')), float(lim.get('upper'))]
    assert sha(URDF) == '7812d6b69e854ed9f37b4e3cbd813ecd53dab3504065b2d04d287abc6fd852e0'
    assert np.array_equal(transforms['Joint6_to_gripper_base'], np.eye(4))
    assert np.array_equal(transforms['Grasp2tool'][:3, :3], np.eye(3))
    p = {k: T[:3, 3] for k, T in transforms.items()}
    R = {k: T[:3, :3] for k, T in transforms.items()}
    assert np.array_equal(R['Joint1'], np.eye(3))
    assert np.array_equal(axes['Joint1'], [0, 0, 1])
    a5parallel = axes['Joint4'] * np.dot(axes['Joint4'], p['Joint5'])
    upper = float(np.linalg.norm(p['Joint3']))
    fore = float(np.linalg.norm(p['Joint4'] + R['Joint4']@a5parallel)
                 + np.linalg.norm(p['Joint5'] - a5parallel))
    reach_upper = upper + fore
    shoulder_center = p['Joint1'] + [0, 0, p['Joint2'][2]]
    shoulder_radius = float(np.linalg.norm(p['Joint2'][:2]))
    v = R['Joint6'].T @ p['Joint6']
    vparallel = axes['Joint6'] * np.dot(axes['Joint6'], v)
    circle_radius = float(np.linalg.norm(v-vparallel))
    wrist_offset_center = p['Grasp2tool'] + vparallel

    def desired_bound(position, rotation):
        center = position - rotation @ wrist_offset_center
        lower = float(np.linalg.norm(center-shoulder_center)-circle_radius-shoulder_radius)
        return dict(tool0_position_m=position.tolist(), tool0_rotation=rotation.tolist(),
                    wrist_circle_center_m=center.tolist(), shoulder_to_j5_lower_bound_m=lower,
                    lower_minus_reach_upper_m=lower-reach_upper,
                    analytically_excluded=bool(lower > reach_upper))

    candidates = []
    for e in audit['stable_evaluations'][:2]:
        stages = {}
        for stage in e['execution_sequence']['stages']:
            stages[stage['stage']] = desired_bound(np.array(stage['position_m']),
                                                   quat(stage['quaternion_xyzw']))
        candidates.append(dict(source_index=e['source_index'], variant_index=e['variant_index'],
                               stages=stages))
    # Formula sanity check only; sampling is NOT the basis of the analytic bound.
    rng = np.random.RandomState(7321)
    largest_circle_identity_error = 0.
    largest_reach_minus_bound = -math.inf
    for _ in range(128):
        q = np.array([rng.uniform(*limits['Joint%d' % i]) for i in range(1, 7)])
        T = np.eye(4); origins = {}
        for i in range(1, 7):
            name = 'Joint%d' % i
            T = T @ transforms[name]
            origins[i] = T[:3, 3].copy()
            T[:3, :3] = T[:3, :3] @ rot(axes[name], q[i-1])
        T = T @ transforms['Joint6_to_gripper_base'] @ transforms['Grasp2tool']
        predicted_j5 = T[:3, 3] - T[:3, :3] @ (
            p['Grasp2tool'] + rot(axes['Joint6'], -q[5]) @ v)
        largest_circle_identity_error = max(largest_circle_identity_error,
                                           float(np.linalg.norm(predicted_j5-origins[5])))
        largest_reach_minus_bound = max(largest_reach_minus_bound,
                                       float(np.linalg.norm(origins[5]-origins[2])-reach_upper))
    assert largest_circle_identity_error < 1e-12
    assert largest_reach_minus_bound <= 1e-12
    # Historic placement illustration only: keep previous TCP z/orientation and radial azimuth.
    stage = audit['stable_evaluations'][0]['execution_sequence']['stages'][2]
    old_p, old_R = np.array(stage['position_m']), quat(stage['quaternion_xyzw'])
    illustrations = []
    for radius in [.42, .44]:
        hypothetical_p = old_p.copy()
        hypothetical_p[:2] *= radius / np.linalg.norm(hypothetical_p[:2])
        entry = desired_bound(hypothetical_p, old_R)
        entry.update(horizontal_tool0_radius_m=radius,
                     note='Hypothetical historic TCP placement; not new object measurement or feasibility certificate.')
        illustrations.append(entry)
    report = dict(schema='old_target_vertical_wrist_bound_v1',
        historical_only=True, target_has_since_moved=True, motion_authorized=False,
        ros_calls=0, ik_search_performed=False,
        source_urdf=str(URDF), urdf_sha256=sha(URDF), source_audit=str(AUDIT), audit_sha256=sha(AUDIT),
        snapshot_stamp_ns=audit['snapshot_stamp_ns'], target_cloud_center_base=audit['target_cloud_center_base'],
        target_horizontal_radius_m=float(np.linalg.norm(audit['target_cloud_center_base'][:2])),
        raw_joint_limits_rad=limits,
        upperarm_length_m=upper, forearm_upper_bound_m=fore, shoulder_to_j5_upper_bound_m=reach_upper,
        shoulder_center_base_m=shoulder_center.tolist(), shoulder_origin_circle_radius_m=shoulder_radius,
        j5_circle_radius_for_fixed_tool_pose_m=circle_radius,
        tool_frame_j5_circle_offset_center_m=wrist_offset_center.tolist(),
        vertical_candidates=candidates, hypothetical_same_tcp_height_and_orientation=illustrations,
        formula_sanity_samples=128, maximum_j5_identity_residual_m=largest_circle_identity_error,
        maximum_sampled_reach_minus_analytic_upper_m=largest_reach_minus_bound,
        conclusion='Both archived near-vertical contact orientations are analytically unreachable. No conclusion about every possible tilted orientation. New target requires new perception and full planning.',
        limitations=['Triangle inequality upper/lower bounds ignore joint limits and collisions, so passing this necessary bound does not establish reachability.',
                     'No tilted IK search or new-location calculation was performed because the user moved the target.',
                     'The bound diagnoses the superseded preview, not the historical approximately 10 mm grasp-height error.'],
        implementation_sha256=sha(Path(__file__)))
    OUT.mkdir(exist_ok=False)
    (OUT/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    shutil.copy2(__file__, OUT/Path(__file__).name)
    print(json.dumps(dict(report=str(OUT/'report.json'), upper_m=reach_upper,
        contact_lower_m=[c['stages']['grasp']['shoulder_to_j5_lower_bound_m'] for c in candidates],
        illustrative_slack_m=[-r['lower_minus_reach_upper_m'] for r in illustrations],
        report_sha256=sha(OUT/'report.json')), ensure_ascii=False, indent=2))

if __name__ == '__main__':
    main()
