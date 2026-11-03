#!/usr/bin/env python3
"""Offline depth/table and transform consistency; estimates are not calibration."""
import json
from pathlib import Path
import cv2
import numpy as np
from hover_camera_intrinsics_20260927 import Projection, arguments, suffix
from tf.transformations import quaternion_matrix

HERE=Path(__file__).resolve().parents[1]
BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
OUT=BASE/'ros_log/unknown_grasp_20260923/hover_speed_failure_20260927'
if not Path('/media/zhuyupei/SanStand').is_mount():raise SystemExit('SanStand unavailable')
meta=json.loads((OUT/'rgbd_frame_metadata.json').read_text())
data=json.loads((OUT/'bag_forensics.json').read_text())
plan=next(e['msg'] for e in data['events']['/grasp_6d/plan_enriched'] if e['msg']['plan_id']=='ec712883949b08d7cc658d8d')
audit=json.loads((Path(data['record'])/'keyframes_audits/ba37e0297d84de8e212d6062459e48a7e60118ced2df49dd81b8a33281d66fb4.json').read_text())
args=arguments();projection=Projection(args.model);cfg=projection.cfg;K=projection.K;tag=suffix(args.model)
R_camera_optical=np.array([[0,0,1],[-1,0,0],[0,-1,0.]])
normal=np.array([plan['object_geometry']['support_normal_base'][k] for k in ('x','y','z')]);offset=plan['object_geometry']['support_offset_m']
def transform(row):
    T=quaternion_matrix(row['xyzw']);T[:3,3]=row['xyz'];return T
def camera_T(label):
    target=meta[label]['images']['depth']['stamp_ns'];bychild={r['child']:r for r in meta['static']}
    for r in meta[label]['transforms']:
        old=bychild.get(r['child'])
        if old is None or abs(r['stamp_ns']-target)<abs(old['stamp_ns']-target):bychild[r['child']]=r
    T=np.eye(4);child='camera_link';chain=[]
    while child!='base_link':
        row=bychild[child];chain.append(child);T=transform(row)@T;child=row['parent']
    O=np.eye(4);O[:3,:3]=R_camera_optical
    return T@O,chain

def fit_table(points):
    rng=np.random.default_rng(37);subset=points[rng.choice(len(points),min(4000,len(points)),replace=False)]
    best=None
    for _ in range(220):
        a,b,c=subset[rng.choice(len(subset),3,replace=False)];n=np.cross(b-a,c-a)
        length=np.linalg.norm(n)
        if length<1e-7:continue
        n/=length;dd=-n@a;dist=abs(subset@n+dd);inlier=dist<.002
        if best is None or inlier.sum()>best[0]:best=(int(inlier.sum()),n,dd)
    n,dd=best[1:]
    for _ in range(4):
        inlier=abs(points@n+dd)<.002
        centroid=points[inlier].mean(axis=0)
        _,_,vv=np.linalg.svd(points[inlier]-centroid,full_matrices=False)
        n=vv[-1];dd=-n@centroid
    if dd<0:n,dd=-n,-dd
    return n,dd,inlier

report={'scope':'offline_RGBD_table_consistency_not_new_calibration','intrinsics_source':projection.provenance,'ray_model':args.model,'previous_438p6_analysis':'WITHDRAWN; preserved in superseded_438p6_intrinsics',
        'K':K.tolist(),'depth_scale_m_per_unit':cfg['depth_scale'],'frozen_support':{'normal':normal.tolist(),'offset_m':offset},'frames':{}}
for label in ('frozen','preclose','closed'):
    depth=np.load(OUT/(label+'_depth.npy'))*cfg['depth_scale'];color=cv2.imread(str(OUT/(label+'_color.jpg')))
    hsv=cv2.cvtColor(color,cv2.COLOR_BGR2HSV)
    red=((hsv[:,:,0]<38)|(hsv[:,:,0]>165))&(hsv[:,:,1]>80)&(hsv[:,:,2]>60)
    vv,uu=np.indices(depth.shape);selection=red&(depth>.08)&(depth<1.)&(uu%3==0)&(vv%3==0)
    z=depth[selection];u=uu[selection];v=vv[selection]
    rays=projection.rays(u,v);points=rays*z[:,None]
    ncam,dcam,inlier=fit_table(points);T,chain=camera_T(label)
    nbase=T[:3,:3]@ncam;dbase=dcam-nbase@T[:3,3]
    cam_pred=T[:3,:3].T@normal;d_pred=normal@T[:3,3]+offset
    base=points[inlier]@T[:3,:3].T+T[:3,3]
    residual=base@normal+offset
    expected_z=-d_pred/(rays[inlier]@cam_pred)
    z_error=points[inlier,2]-expected_z
    center=np.array([plan['object_geometry']['pose_base']['position'][k] for k in ('x','y','z')]);pointcam=T[:3,:3].T@(center-T[:3,3]);projected=projection.project(pointcam)[0]
    row={'T_base_optical_from_bag':T.tolist(),'tf_chain':chain,'red_depth_point_count':len(points),'table_inlier_count':int(inlier.sum()),
         'table_inlier_rms_m':float(np.sqrt(np.mean((points[inlier]@ncam+dcam)**2))),
         'observed_plane_camera':{'normal':ncam.tolist(),'offset_m':float(dcam)},
         'observed_plane_base_with_recorded_TF':{'normal':nbase.tolist(),'offset_m':float(dbase)},
         'frozen_plane_predicted_camera':{'normal':cam_pred.tolist(),'offset_m':float(d_pred)},
         'normal_angle_deg':float(np.degrees(np.arccos(np.clip(nbase@normal,-1,1)))),
         'observed_table_signed_residual_to_frozen_plane_m':{'median':float(np.median(residual)),'q05':float(np.quantile(residual,.05)),'q95':float(np.quantile(residual,.95))},
         'observed_minus_predicted_depth_m':{'median':float(np.median(z_error)),'q05':float(np.quantile(z_error,.05)),'q95':float(np.quantile(z_error,.95))},
         'predicted_object_center_pixel':projected[:2].tolist(),'predicted_object_center_depth_m':float(pointcam[2])}
    if label=='frozen':row['max_abs_audit_transform_difference']=float(np.max(abs(T-np.array(audit['snapshot_transform']['T_base_optical']))))
    image=color.copy();validu=u[inlier];validv=v[inlier];image[validv,validu]=[0,255,0]
    px,py=np.rint(projected[:2]).astype(int);cv2.drawMarker(image,(px,py),(255,0,255),cv2.MARKER_CROSS,25,2)
    cv2.imwrite(str(OUT/(label+'_table_inliers_and_expected_target'+tag+'.jpg')),image)
    report['frames'][label]=row
(OUT/('rgbd_table_plane_consistency'+tag+'.json')).write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
