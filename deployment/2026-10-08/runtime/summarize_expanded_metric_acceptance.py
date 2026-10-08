#!/usr/bin/env python3
"""Read-only same-metric acceptance summary, no optimization or deployment."""
import json,sys
from pathlib import Path
from collections import defaultdict
import numpy as np
import audit_joint_metric_identifiability as M
import fit_expanded_metric_holdout as E
import evaluate_frozen_metric_failed_grasp as F


def errors(T,Y):
    T=np.asarray(T);dp=np.linalg.norm(T[:,:3,3]-Y[:3,3],axis=1)*1000
    da=np.asarray([np.degrees(np.linalg.norm(M.from_matrix(Y[:3,:3].T@t[:3,:3]).as_rotvec())) for t in T])
    return dict(translation_rms_mm=float(np.sqrt(np.mean(dp**2))),translation_max_mm=float(max(dp)),
        orientation_rms_deg=float(np.sqrt(np.mean(da**2))),orientation_max_deg=float(max(da)),per_frame_translation_mm=dp.tolist(),per_frame_orientation_deg=da.tolist())


def pairs(T):
    T=np.asarray(T);dp=[];dc=[];da=[];p=np.array([.09,.0675,0.])
    for i in range(len(T)):
        for j in range(i):
            dp.append(float(np.linalg.norm(T[i,:3,3]-T[j,:3,3])*1000))
            dc.append(float(np.linalg.norm(T[i,:3,:3]@p+T[i,:3,3]-T[j,:3,:3]@p-T[j,:3,3])*1000))
            da.append(float(np.degrees(np.linalg.norm(M.from_matrix(T[i,:3,:3].T@T[j,:3,:3]).as_rotvec()))))
    return dict(board_origin_maxpair_mm=max(dp),board_center_maxpair_mm=max(dc),rotation_maxpair_deg=max(da))


def main():
    r=Path(sys.argv[1]);out=r/'expanded_metric_training_once'
    if not F.os.path.ismount(F.MOUNT):raise ValueError('SanStand unavailable')
    fit=json.loads((out/'fit_result.json').read_text());z=np.asarray(fit['fit']['z']);X,xsha=M.handeye(Path('/home/zhuyupei/.ros/easy_handeye/d405_cc200_auto_20260927_validated_eye_on_hand.yaml'))
    rows,_=M.load(r/'camera_intrinsics_sensitivity');base=[a for a in rows if a['role']=='training'];ind=[a for a in rows if a['role']=='independent_validation']
    frozen=E.avg(M.predict(base,X,z));indT=M.predict(ind,X,z);inderrors=errors(indT,frozen)
    thresholds={'translation_rms_mm':3.,'translation_max_mm':5.,'orientation_rms_deg':1.,'orientation_max_deg':2.}
    report=dict(refitted=False,runtime_changed=False,fit_sha256=F.sha(out/'fit_result.json'),thresholds=thresholds,
        independent8_fixed_training22_reference=dict(**inderrors,**pairs(indT),reference_base_board=frozen.tolist(),
            threshold_checks={k:dict(value=inderrors[k],limit=v,passed=inderrors[k]<=v) for k,v in thresholds.items()}),
        training_group_metrics=fit['training_fitted'],local_heldouts={})
    original_validation=r.parent/'handeye_auto_20260927T102700Z/final_validation_20260927T110130Z/result.json'
    historical=json.loads(original_validation.read_text());report['historical_validation_threshold_checks']=historical['threshold_checks'];report['historical_validation_path']=str(original_validation)
    assert thresholds=={'translation_rms_mm':historical['threshold_checks']['translation_rms_m']['limit']*1000,
                        'translation_max_mm':historical['threshold_checks']['translation_max_m']['limit']*1000,
                        'orientation_rms_deg':historical['threshold_checks']['orientation_rms_deg']['limit'],
                        'orientation_max_deg':historical['threshold_checks']['orientation_max_deg']['limit']}
    XX=E.modified_X(X,z);off=np.r_[0.,np.radians(z[6:]),0.]
    for label,session,path in [('J1','bounded_motor_centering_20260928T140754Z_327901',r/'j1_board_axis_140754/selected_geometries.json'),
                               ('centering_J2_J3','bounded_motor_centering_20260928T141708Z_351496',r/'frozen_metric_centering_141708/selected_geometries.json')]:
        rr=list(json.loads(path.read_text()).values());fk=M.MetricFk((r.parent/session/'robot_description.urdf').read_text());groups=defaultdict(list);allT=[]
        for row in rr:
            Y=fk(np.asarray(row['accepted_positions_rad'])[:6]+off)@XX@np.asarray(row['T_camera_board'])
            groups[row['offline_stationary_phase']].append(Y);allT.append(Y)
        reference=E.avg(groups['baseline']);report['local_heldouts'][label]=dict(count=len(allT),window_counts={k:len(v) for k,v in groups.items()},
            **pairs(allT),against_same_session_frozen_baseline=errors(allT,reference),within_each_phase={k:errors(v,E.avg(v)) for k,v in groups.items()})
    h=json.loads((out/'heldout_report.json').read_text());g=h['models']['expanded_45_train']['failed_grasp'];table=defaultdict(list)
    for row in g:table[row['stage']].append(row['later_minus_initial_z_mm'])
    report['failed_grasp_support_drift']=dict(stage_z_ranges_mm={k:[min(v),max(v)] for k,v in table.items()},max_abs_all_18_patches_mm=max(abs(x) for v in table.values() for x in v),
        before_close_max_abs_mm=max(abs(x) for x in table['before_close']),interpretation='same original fits; not direct fingertip metrology')
    quat=M.from_matrix(XX[:3,:3]).as_quat();quat=quat if quat[3]>=0 else -quat
    report['candidate_parameters']=dict(T_tool_camera=XX.tolist(),translation_xyz_m=XX[:3,3].tolist(),quaternion_xyzw=quat.tolist(),
        joint_zero_deg=dict(zip(['Joint%d'%i for i in range(1,7)],np.degrees(off).tolist())),joint_zero_rad=dict(zip(['Joint%d'%i for i in range(1,7)],off.tolist())),
        convention='T_base_camera = FK_original_URDF(q_accepted + joint_zero_rad) @ T_tool_camera. This is a candidate kinematic model, not an offset to issue as motor commands.',
        deployed=False,approved_for_motion=False)
    report['assessment']='Numerical evidence supports proceeding to an independent live validation under existing limits; no per-metric strict improvement requirement was introduced. This is not already-deployed or physically validated calibration. Expanded task groups are training; original independent8 against TRAINING reference is reported separately from within-group scatter.'
    (out/'acceptance_summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');(out/Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    print(json.dumps({k:v for k,v in report.items() if k not in ('training_group_metrics','historical_validation_threshold_checks')},ensure_ascii=False))

if __name__=='__main__':main()
