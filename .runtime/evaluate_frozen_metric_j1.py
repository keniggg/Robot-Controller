#!/usr/bin/env python3
"""Strict new holdout evaluation of the prior 22-frame fit; no optimization/I/O to ROS."""
import argparse,hashlib,json,os
from pathlib import Path
import numpy as np
import audit_j1_board_axis as J
G=J.G
MOUNT=Path('/media/zhuyupei/SanStand')


def evaluated(rows,xml,X,z):
    XX=X.copy();XX[:3,3]+=np.asarray(z[:3])*.001
    XX[:3,:3]=G.exp_so3(np.radians(z[3:6]))@X[:3,:3]
    offset=np.r_[0.,np.radians(z[6:10]),0.]
    result=[]
    for row in rows:
        q=np.asarray(row['accepted_positions_rad'])[:6]
        A=G.serial_fk_and_axes(xml,q+offset)[0];C=np.asarray(row['T_camera_board'])
        result.append(dict(row,T_base_tool=A.tolist(),T_base_board=(A@XX@C).tolist(),T_tool_camera=XX.tolist()))
    return result


def compare(first,second,X,anchor):
    summary=J.paired(first,second)
    A0,A1=np.asarray(first['T_base_tool']),np.asarray(second['T_base_tool'])
    C0,C1=np.asarray(first['T_camera_board']),np.asarray(second['T_camera_board'])
    Y0,Y1=np.asarray(first['T_base_board']),np.asarray(second['T_base_board'])
    observed=G.inv(C1)[:3,3]-G.inv(C0)[:3,3]
    expected=(G.inv(Y0)@A1@X)[:3,3]-(G.inv(Y0)@A0@X)[:3,3]
    summary.update(observed_camera_travel_board_mm=(1000*observed).tolist(),
        predicted_camera_travel_board_mm=(1000*expected).tolist(),
        camera_travel_vector_error_mm=float(np.linalg.norm(expected-observed)*1000),
        board_origin_drift_mm=float(np.linalg.norm(Y1[:3,3]-Y0[:3,3])*1000))
    for label,p in [('center',np.array([.09,.0675,0.])),('motor_anchor',anchor)]:
        d=Y1[:3,:3]@p+Y1[:3,3]-Y0[:3,:3]@p-Y0[:3,3]
        summary['board_'+label+'_drift_mm']=float(np.linalg.norm(d)*1000)
        summary['board_'+label+'_drift_base_xyz_mm']=(d*1000).tolist()
    return summary


def main():
    p=argparse.ArgumentParser();p.add_argument('session',type=Path);p.add_argument('prior',type=Path);p.add_argument('output',type=Path);args=p.parse_args()
    if not os.path.ismount(MOUNT) or MOUNT not in args.output.resolve().parents:raise ValueError('SanStand output required')
    rows=list(json.loads((args.output/'selected_geometries.json').read_text()).values())
    fits=json.loads(args.prior.read_text())['fits'];fixed=[r for r in fits if r['orientation_unit_deg']==.1]
    if len(fixed)!=1:raise ValueError('unique predeclared prior 0.1deg candidate required')
    z=np.asarray(fixed[0]['z']);X=np.asarray(rows[0]['T_tool_camera']);xml=(args.session/'robot_description.urdf').read_text()
    anchor=np.asarray(json.loads((args.session/'motor_evidence.json').read_text())['manual_motor_seed']['anchor_board'])
    report=dict(diagnostic_only=True,motion_authorized=False,runtime_changed=False,refitted=False,
        candidate_selection='predeclared existing 0.1-degree weight, 22-training-frame fit only; no new weight selection',
        frozen_prior_fit_sha256=hashlib.sha256(args.prior.read_bytes()).hexdigest(),frozen_prior_parameters=fixed[0]['parameters'],
        original_training_count=22,additional_validation_only_frame_count=len(rows),source_session=str(args.session),
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        limitations=['Original task16 maximum pairwise position spread remains 5.449 mm; this new result cannot erase that failed generalization.',
            'Only one 2.6-degree J1 turn, plus a one-word J4 change, was observed.',
            'Frame pairs are correlated descriptive distributions, not independent confidence intervals.',
            'No camera intrinsics, lengths, scale, hand-eye or robot parameters were changed or refitted.',
            'Improvement supports model consistency for this holdout; it does not uniquely attribute physical root cause.'])
    output={}
    for label,zz in [('runtime',np.zeros(10)),('prior_training_only_candidate',z)]:
        rr=evaluated(rows,xml,X,zz);XX=np.asarray(rr[0]['T_tool_camera']);groups={s:[r for r in rr if r['offline_stationary_phase']==s] for s in ('baseline','center_J1')}
        mean=compare(J.average(groups['baseline']),J.average(groups['center_J1']),XX,anchor)
        pairs=[compare(a,b,XX,anchor) for a in groups['baseline'] for b in groups['center_J1']]
        filtered={s:[r for r in v if r['quality']['reprojection_rms_px']<=.15] for s,v in groups.items()}
        output[label]=dict(mean_pose_comparison=mean,paired_descriptive={k:J.D.descriptive([r[k] for r in pairs]) for k in pairs[0]},
            low_reprojection_sensitivity=dict(threshold_px=.15,counts={k:len(v) for k,v in filtered.items()},
                comparison=compare(J.average(filtered['baseline']),J.average(filtered['center_J1']),XX,anchor)))
    report['models']=output
    (args.output/'frozen_metric_holdout.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    (args.output/Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    print(json.dumps({k:v['mean_pose_comparison'] for k,v in output.items()},ensure_ascii=False))

if __name__=='__main__':main()
