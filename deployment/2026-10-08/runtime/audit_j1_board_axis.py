#!/usr/bin/env python3
"""Closed-bag J1 camera rotation axis in the fixed board frame; never motion."""
import argparse
from collections import defaultdict
import hashlib
import json
import math
import os
from pathlib import Path
import numpy as np
import analyze_depth_supported_board_motion as D
G=D.GEOM
MOUNT=Path('/media/zhuyupei/SanStand')


def angle(a,b,unoriented=False):
    x=float(np.dot(a,b)/(np.linalg.norm(a)*np.linalg.norm(b)))
    if unoriented:x=abs(x)
    return math.degrees(math.acos(float(np.clip(x,-1.,1.))))


def rotation_axis(R):
    v=G.log_so3(np.asarray(R));theta=float(np.linalg.norm(v))
    if theta<math.radians(.1):raise ValueError('rotation too small for axis diagnostic')
    return v/theta,math.degrees(theta)


def paired(first,second):
    C0,C1=map(np.asarray,(first['T_camera_board'],second['T_camera_board']))
    A0,A1=map(np.asarray,(first['T_base_tool'],second['T_base_tool']))
    Y0=np.asarray(first['T_base_board'])
    # camera motion expressed in the FIXED BOARD frame, independent of X/FK.
    observed=C1[:3,:3].T@C0[:3,:3]
    expected=Y0[:3,:3].T@A1[:3,:3]@A0[:3,:3].T@Y0[:3,:3]
    axis,theta=rotation_axis(observed);ea,et=rotation_axis(expected)
    base_z=Y0[:3,:3].T@np.array([0.,0.,1.])
    return dict(observed_axis_board=axis.tolist(),observed_rotation_deg=theta,
        observed_axis_to_board_normal_deg=angle(axis,[0,0,1],True),
        actual_fk_axis_board=ea.tolist(),actual_fk_rotation_deg=et,
        actual_fk_axis_to_board_normal_deg=angle(ea,[0,0,1],True),
        observed_to_actual_fk_axis_deg=angle(axis,ea,True),
        baseline_base_Z_board=base_z.tolist(),baseline_base_Z_to_board_normal_deg=angle(base_z,[0,0,1],True),
        observed_to_baseline_base_Z_deg=angle(axis,base_z,True),
        relative_rotation_residual_deg=math.degrees(np.linalg.norm(G.log_so3(expected.T@observed))),
        actual_joint_delta_deg=np.degrees(np.asarray(second['accepted_positions_rad'])[:6]-np.asarray(first['accepted_positions_rad'])[:6]).tolist())


def average(rows):
    return dict(T_camera_board=G.average_transform([r['T_camera_board'] for r in rows]),
        T_base_tool=G.average_transform([r['T_base_tool'] for r in rows]),
        T_base_board=G.average_transform([r['T_base_board'] for r in rows]),
        accepted_positions_rad=np.mean([r['accepted_positions_rad'] for r in rows],axis=0))


def window(rows):
    avg=average(rows); C=np.asarray(avg['T_camera_board']); R=C[:3,:3]
    blocks=defaultdict(list);start=min(r['source_stamp_ns'] for r in rows)
    for row in rows:blocks[(row['source_stamp_ns']-start)//500000000].append(row)
    return dict(frame_count=len(rows),time_span_sec=(max(r['source_stamp_ns'] for r in rows)-start)*1e-9,
        half_second_block_count=len(blocks),source_stamp_ns=[r['source_stamp_ns'] for r in rows],
        orientation_from_mean_deg=D.descriptive([math.degrees(np.linalg.norm(G.log_so3(R.T@np.asarray(r['T_camera_board'])[:3,:3]))) for r in rows]),
        actual_joint_deg=D.descriptive([np.degrees(np.asarray(r['accepted_positions_rad'])[:6]) for r in rows]),
        reprojection_rms_px=D.descriptive([r['quality']['reprojection_rms_px'] for r in rows]),
        charuco_count=D.descriptive([r['quality']['charuco_count'] for r in rows])),[average(v) for v in blocks.values()]


def synthetic_contract_test():
    axis=np.array([.1,.2,1.]);axis/=np.linalg.norm(axis)
    C0=np.eye(4);C0[:3,:3]=G.exp_so3([.4,-.7,.2])
    RB=G.exp_so3(axis*math.radians(3.))
    C1=np.eye(4);C1[:3,:3]=C0[:3,:3]@RB.T
    measured,deg=rotation_axis(C1[:3,:3].T@C0[:3,:3])
    np.testing.assert_allclose(measured,axis,atol=1e-12)
    assert abs(deg-3.)<1e-10
    # A constant change of CAMERA coordinates cancels in board relative motion.
    Q=G.exp_so3([.2,.1,-.3])
    np.testing.assert_allclose((Q@C1[:3,:3]).T@(Q@C0[:3,:3]),RB,atol=1e-12)
    return dict(axis_frame_formula_passed=True,constant_camera_frame_rotation_invariance_passed=True)


def main():
    p=argparse.ArgumentParser();p.add_argument('session',type=Path);p.add_argument('output',type=Path);args=p.parse_args()
    if not os.path.ismount(MOUNT) or MOUNT not in args.output.resolve().parents:raise ValueError('mounted SanStand output required')
    args.output.mkdir(parents=True,exist_ok=True)
    template=json.loads((args.session/'baseline_geometry.json').read_text())
    geos,selection=D.collect_paired_geometries(args.session,template,maximum_per_window=24)
    groups=defaultdict(list)
    for g in geos.values():groups[g['offline_stationary_phase']].append(g)
    report=dict(schema=1,session=str(args.session),selection=selection,mathematical_regression=synthetic_contract_test(),
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        input_sha256={n:hashlib.sha256((args.session/n).read_bytes()).hexdigest() for n in ('events.jsonl','plan.json','baseline_geometry.json','robot_description.urdf')},
        motion_authorized=False,parameters_modified=False,
        limitations=['A single small J1 motion with one-word J4 change is not a pure-axis calibration.',
            'Pair and frame distributions are correlated descriptive variation, not independent confidence intervals.',
            'Axis estimates amplify orientation noise approximately inversely with the 2.6-degree turn.',
            'Observed board-frame relative axis uses RGB board orientation and camera model, but neither hand-eye nor FK.',
            'A discrepancy does not uniquely distinguish camera pose systematics, joint model/zero error, or compliance; no base movement is assumed.'])
    reports={};blocks={}
    for label,rows in groups.items():reports[label],blocks[label]=window(rows)
    report['windows']=reports
    if 'baseline' in groups and 'center_J1' in groups:
        pairs=[paired(a,b) for a in groups['baseline'] for b in groups['center_J1']]
        central=paired(average(groups['baseline']),average(groups['center_J1']))
        bp=[paired(a,b) for a in blocks['baseline'] for b in blocks['center_J1']]
        report.update(status='available',mean_pose_comparison=central,
            paired_descriptive={k:D.descriptive([r[k] for r in pairs]) for k in pairs[0]},
            half_second_block_pair_descriptive={k:D.descriptive([r[k] for r in bp]) for k in bp[0]})
        report['selected_geometry_count']=len(geos)
    else:report.update(status='insufficient_qualified_windows')
    (args.output/'report.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    (args.output/'selected_geometries.json').write_text(json.dumps(geos,indent=2)+'\n')
    (args.output/Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    print(json.dumps({k:report[k] for k in ('status','selection','windows','mean_pose_comparison') if k in report},ensure_ascii=False))

if __name__=='__main__':main()
