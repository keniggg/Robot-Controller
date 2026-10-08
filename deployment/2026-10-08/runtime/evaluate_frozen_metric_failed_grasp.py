#!/usr/bin/env python3
"""Evaluate one frozen train-only candidate on the archived failed grasp planes.

No fitting, image/ROI reselection, ROS connection, or robot parameter mutation.
A fitted plane is transformed back to its exact archived optical frame, then
forward with the candidate FK/X. Rigid covariance preserves the original fit,
inlier set and RMS; the common anchor follows the INITIAL image's transform.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import numpy as np
import audit_joint_metric_identifiability as M

MOUNT = Path('/media/zhuyupei/SanStand')
O = np.eye(4)
O[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def plane(fit):
    p = np.r_[fit['n'], fit['d']].astype(float)
    if not np.all(np.isfinite(p)) or abs(np.linalg.norm(p[:3])-1.) > 1e-9:
        raise ValueError('archived fit must have unit finite normal')
    return p


def transport(p, old, new):
    camera_plane = old.T @ p
    return np.linalg.solve(new.T, camera_plane), camera_plane


def z_at(p, xy):
    if abs(p[2]) < .8:
        raise ValueError('height comparison has near-horizontal denominator')
    return -(np.dot(p[:2], xy)+p[3])/p[2]


def compare(p0, p1, anchor):
    return dict(later_minus_initial_z_mm=float((z_at(p1, anchor[:2])-z_at(p0, anchor[:2]))*1000.),
                later_signed_plane_distance_at_initial_anchor_mm=float(np.dot(p1, np.r_[anchor, 1.])*1000.),
                support_normal_change_deg=float(np.degrees(np.arccos(np.clip(np.dot(p0[:3], p1[:3]), -1., 1.)))),
                initial_plane_base=p0.tolist(), later_plane_base=p1.tolist(), common_anchor_base_m=anchor.tolist())


def synthetic_covariance():
    rng = np.random.default_rng(52)
    max_error = 0.
    for _ in range(50):
        n=rng.normal(size=3);n/=np.linalg.norm(n);p=np.r_[n,rng.normal()]
        T=np.eye(4);T[:3,:3]=M.matrix(M.Rotation.from_rotvec(rng.normal(size=3)));T[:3,3]=rng.normal(size=3)
        transformed, _ = transport(p,np.eye(4),T)
        xyz=rng.normal(size=(20,3));xyz-=(xyz@n+p[3])[:,None]*n
        y=xyz@T[:3,:3].T+T[:3,3]
        max_error=max(max_error,float(np.max(np.abs(y@transformed[:3]+transformed[3]))))
        np.testing.assert_allclose(transport(transformed,T,np.eye(4))[0],p,atol=2e-14,rtol=0)
    assert max_error < 2e-14
    return dict(trials=50,max_transformed_plane_residual_m=max_error,roundtrip_passed=True)


def main():
    p=argparse.ArgumentParser();p.add_argument('session',type=Path);p.add_argument('prior',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--handeye',type=Path,default=Path('/home/zhuyupei/.ros/easy_handeye/d405_cc200_auto_20260927_validated_eye_on_hand.yaml'))
    a=p.parse_args()
    if not os.path.ismount(MOUNT) or MOUNT not in a.output.resolve().parents:
        raise ValueError('mounted SanStand output required; no fallback')
    gp=a.session/'geometry_forensics/geometry_report.json';up=a.session/'robot_description.urdf';cp=a.session/'initial_context.json'
    fp=a.prior/'fit_report.json';rp=a.prior/'rank_report.json'
    g=json.loads(gp.read_text());fit=json.loads(fp.read_text());rank=json.loads(rp.read_text());context=json.loads(cp.read_text())
    fixed=[v for v in fit['fits'] if v['orientation_unit_deg']==.1]
    if len(fixed)!=1 or rank['training_count']!=22:
        raise ValueError('unique predeclared 22-training 0.1deg candidate required')
    z=np.asarray(fixed[0]['z']);X,xsha=M.handeye(a.handeye)
    if xsha!=rank['handeye_sha256']:raise ValueError('frozen baseline handeye hash mismatch')
    np.testing.assert_allclose(X[:3,3],context['handeye']['translation_xyz'],atol=1e-14,rtol=0)
    np.testing.assert_allclose(X[:3,:3],M.matrix(M.Rotation.from_quat(context['handeye']['rotation_xyzw'])),atol=1e-14,rtol=0)
    if sha(up)!=g['urdf_sha256']:raise ValueError('archived URDF hash mismatch')
    fk=M.MetricFk(up.read_text());XX=X.copy();XX[:3,3]+=z[:3]*.001
    XX[:3,:3]=M.matrix(M.Rotation.from_rotvec(np.radians(z[3:6])))@X[:3,:3]
    offsets=np.r_[0.,np.radians(z[6:]),0.]
    rows={};fk_residual=[]
    for f in g['frames']:
        q=np.asarray(f['q_rad']);old=np.asarray(f['T_base_optical']);recomputed=fk(q)@X@O
        np.testing.assert_allclose(recomputed,old,atol=1e-10,rtol=0)
        fk_residual.append(float(np.max(np.abs(recomputed-old))))
        rows[f['label']]=dict(source_stamp_ns=f['source_stamp_ns'],actual_q_rad=q.tolist(),
            stationary_around_image=f['stationary_around_image'],rgb_depth_exact=f['rgb_depth_exact'],
            accepted_nearest_delta_sec=f['accepted_nearest_delta_sec'],joint_span_in_halfsecond_rad=f['joint_span_in_halfsecond_rad'],
            target_inside_image=f['target_inside_image'],runtime_T_base_optical=old.tolist(),
            candidate_T_base_optical=(fk(q+offsets)@XX@O).tolist())
    def transformed(label,p):
        r=rows[label];return transport(p,np.asarray(r['runtime_T_base_optical']),np.asarray(r['candidate_T_base_optical']))
    D0=np.asarray(rows['initial']['candidate_T_base_optical'])@np.linalg.inv(np.asarray(rows['initial']['runtime_T_base_optical']))
    patches=[];baseline_max=0.
    for item in g['common_patch_comparisons']:
        label=item['stage'];first=item['fits']['initial'];later=item['fits'][label]
        p0,p1=plane(first),plane(later);anchor=np.r_[item['center_xy_base_m'],first['z_at_common_center_mm']*.001]
        old=compare(p0,p1,anchor);baseline_max=max(baseline_max,abs(old['later_minus_initial_z_mm']-item['later_minus_initial_z_mm']))
        np.testing.assert_allclose(old['later_minus_initial_z_mm'],item['later_minus_initial_z_mm'],atol=2e-11,rtol=0)
        pp0,pc0=transformed('initial',p0);pp1,pc1=transformed(label,p1);aa=(D0@np.r_[anchor,1.])[:3]
        new=compare(pp0,pp1,aa)
        patches.append(dict(stage=label,radius_m=item['radius_m'],runtime=old,frozen_candidate=new,
            initial_plane_optical=pc0.tolist(),later_plane_optical=pc1.tolist(),
            same_archived_fit_evidence=item['fits'],both_images_stationary=rows['initial']['stationary_around_image'] and rows[label]['stationary_around_image']))
    close=[r for r in patches if r['stage']=='before_close']
    summary={name:dict(before_close_z_mm=[r[name]['later_minus_initial_z_mm'] for r in close],
        before_close_signed_normal_distance_mm=[r[name]['later_signed_plane_distance_at_initial_anchor_mm'] for r in close],
        all_patch_max_abs_z_mm=max(abs(r[name]['later_minus_initial_z_mm']) for r in patches))
        for name in ('runtime','frozen_candidate')}
    report=dict(schema=1,scope='frozen train-only metric candidate, failed large-motion grasp heldout',
        diagnostic_only=True,refitted=False,runtime_changed=False,motion_authorized=False,
        candidate_selection='predeclared existing 0.1-degree orientation weight; 22 old training images only',
        frozen_parameters=fixed[0]['parameters'],q_offset_rad=offsets.tolist(),X_runtime=X.tolist(),X_candidate=XX.tolist(),
        camera_model_unchanged=g['camera_fingerprint'],plane_pixel_selection_unchanged=True,plane_fit_unchanged=True,
        source_session=str(a.session),source_sha256=sha(__file__),
        input_sha256={str(path):sha(path) for path in (gp,up,cp,fp,rp,a.handeye)},
        verification=dict(plane_covariance=synthetic_covariance(),old_FK_max_abs_matrix_error=max(fk_residual),
                          archived_z_comparison_max_error_mm=baseline_max,original_baseline_X_hash_verified=True),
        coverage=dict(original_frame_count=len(rows),stationary_frame_count=sum(r['stationary_around_image'] for r in rows.values()),
            common_patch_comparisons=len(patches),common_patch_stage_count=len(set(r['stage'] for r in patches)),
            stationary_common_patch_comparisons=sum(r['both_images_stationary'] for r in patches),
            before_close_source_stamp_ns=rows['before_close']['source_stamp_ns'],
            initial_to_before_close_joint_delta_deg=np.degrees(np.asarray(rows['before_close']['actual_q_rad'])-np.asarray(rows['initial']['actual_q_rad'])).tolist()),
        frames=rows,common_patch_comparisons=patches,summary=summary,
        conclusion='Frozen candidate reverses the signed height drift but retains about 7.5mm before close. It fails this independent task-domain consistency test and is not a correction suitable for deployment.',
        limitations=g['limits']+[
            'Planes are exact rigid reexpressions of archived same-pixel fits; no raw point ROI/inlier reselection or optimization under candidate.',
            'Common XY patches were chosen under the original model, not individually identified material correspondences; only their initial-image anchors are transported to the candidate model.',
            'This is a model-consistency residual, not physical fingertip/table metrology or an authorized height correction.',
            'The target was outside or near the image boundary in late stages; common visible table evidence is distinct from extrapolation to target.',
            'No cross-board/epoch absolute coordinate comparisons are used. Earlier mixed heldouts are not superseded by this failure.'])
    a.output.mkdir(parents=True,exist_ok=True)
    (a.output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    (a.output/Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    print(json.dumps(dict(summary=summary,coverage=report['coverage'],verification=report['verification']),ensure_ascii=False))

if __name__=='__main__':main()
