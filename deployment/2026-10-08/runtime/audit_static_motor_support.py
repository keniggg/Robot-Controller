#!/usr/bin/env python3
"""Closed-bag SDK local tabletop audit; no board transform or motion authority."""
import argparse, importlib.util, json, sys
from pathlib import Path
import numpy as np
import cv2, rosbag, rospy, tf2_ros
from cv_bridge import CvBridge
HERE=Path(__file__).resolve().parent;ROOT=HERE.parent
sys.path[:0]=[str(ROOT/'src/alicia_grasp_modes/src'),str(HERE)]
from alicia_grasp_modes.tabletop import Config,_fit_plane
spec=importlib.util.spec_from_file_location('camera_model',ROOT/'src/real-arm/alicia_d_calibration/scripts/charuco_tracker.py')
CM=importlib.util.module_from_spec(spec);spec.loader.exec_module(CM)
from audit_centered_production_support import compare_plane

def matrix(t):
    q=t.transform.rotation;v=np.array([q.x,q.y,q.z,q.w]);v=v/np.linalg.norm(v);x,y,z,w=v
    T=np.eye(4);T[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)], [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)], [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    p=t.transform.translation;T[:3,3]=[p.x,p.y,p.z];return T

def main():
    ap=argparse.ArgumentParser();ap.add_argument('session',type=Path);a=ap.parse_args();out=a.session/'independent_local_table_audit';out.mkdir(exist_ok=True)
    camera=json.loads((a.session/'metadata.json').read_text())['camera_parameters']
    model=CM.RuntimeColorModel(CM.runtime_color_model_evidence(camera,'camera_link',camera['color_topic']))
    rgb={};depth={};mask={};status={};b=CvBridge();buffer=tf2_ros.Buffer(cache_time=rospy.Duration(90))
    with rosbag.Bag(str(a.session/'evidence.bag')) as bag:
        for topic,m,t in bag.read_messages(topics=['/tf','/tf_static','/unknown_loss/exact_rgb','/unknown_loss/exact_depth','/perception/unknown/object_mask','/perception/unknown/detector_status']):
            if topic in ('/tf','/tf_static'):
                for tr in m.transforms:
                    (buffer.set_transform_static if topic=='/tf_static' else buffer.set_transform)(tr,'closed_archive')
            elif topic.endswith('detector_status'):
                s=json.loads(m.data)
                if s.get('state')=='ready':status[s['source_stamp_ns']]=s
            else:
                ns=m.header.stamp.to_nsec()
                if topic.endswith('exact_rgb'):rgb[ns]=m
                elif topic.endswith('exact_depth'):depth[ns]=m
                else:mask[ns]=m
    common=sorted(set(rgb)&set(depth)&set(mask)&set(status));chosen=[common[i] for i in np.unique(np.linspace(5,len(common)-6,3).round().astype(int))]
    rows=[]
    for ns in chosen:
        image=b.imgmsg_to_cv2(rgb[ns],'bgr8');z16=b.imgmsg_to_cv2(depth[ns],'passthrough');m=b.imgmsg_to_cv2(mask[ns],'mono8')>0;s=status[ns]
        F=matrix(buffer.lookup_transform('base_link','camera_link',rospy.Time(ns//10**9,ns%10**9)))
        ys,xs=np.nonzero(m);center=np.median(np.c_[xs,ys],axis=0)
        yy,xx=np.indices(m.shape);rr=np.hypot(xx-center[0],yy-center[1]);expanded=cv2.dilate(m.astype(np.uint8),np.ones((9,9),np.uint8))>0
        ring=(rr>=35)&(rr<=70)&~expanded&(z16>0)&((xx%2)==0)&((yy%2)==0)
        uy,ux=np.nonzero(ring);uv=np.c_[ux,uy];z=z16[uy,ux]*camera['depth_scale']
        rays=np.c_[model.normalized_points(uv).reshape(-1,2),np.ones(len(uv))]
        optical=rays*z[:,None];pc=np.c_[optical[:,2],-optical[:,0],-optical[:,1]]
        plane,ratio,rms,count=_fit_plane(pc,Config())
        inliers=abs(pc@plane[:3]+plane[3])<=.002
        quadrants=len(set(((ux[inliers]>center[0]).astype(int)+2*(uy[inliers]>center[1]).astype(int)).tolist()))
        if quadrants<3:raise ValueError('local table lacks three quadrants')
        good=z16[ys,xs]>0;muv=np.c_[xs[good],ys[good]];mz=z16[ys[good],xs[good]]*camera['depth_scale']
        mrays=np.c_[model.normalized_points(muv).reshape(-1,2),np.ones(len(muv))];mo=mrays*mz[:,None];mp=np.c_[mo[:,2],-mo[:,0],-mo[:,1]]
        current=np.array(s['geometry_metrics']['support_plane_base']);prod=np.r_[F[:3,:3].T@current[:3],current[3]+current[:3]@F[:3,3]]
        if plane[:3]@prod[:3]<0:plane=-plane
        foot=mp-(mp@plane[:3]+plane[3])[:,None]*plane[:3]
        row=dict(source_stamp_ns=ns,generation=s['generation'],selection_stamp_ns=s['selection_stamp_ns'],anchor_source_stamp_ns=s['anchor_source_stamp_ns'],model_fingerprint=model.fingerprint,
            exact_tf_stamp_ns=ns,T_base_camera=F.tolist(),motor_pixel_median=center.tolist(),mask_pixels=int(m.sum()),table_ring_px=[35,70],table_mask_exclusion_dilation_px=4,
            table_sample_count=count,table_inlier_ratio=ratio,table_rms_mm=rms*1000,table_quadrants=quadrants,table_plane_camera=plane.tolist(),production_plane_camera=prod.tolist(),
            production_vs_local_sdk_table=compare_plane(prod,plane,foot),motor_surface_height_mm=(np.percentile(mp@plane[:3]+plane[3],[10,50,90])*1000).tolist())
        cv2.imwrite(str(out/(str(ns)+'_raw.png')),image)
        overlay=image.copy();overlay[uy[inliers],ux[inliers]]=[0,255,0];contours,_=cv2.findContours(m.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE);cv2.drawContours(overlay,contours,-1,(255,255,0),1);cv2.imwrite(str(out/(str(ns)+'_audit.png')),overlay)
        np.savez_compressed(out/(str(ns)+'_measured.npz'),rgb=image,z16=z16,mask=m,table_pixels=uv,table_points_camera=pc,table_inliers=inliers)
        rows.append(row)
    report=dict(diagnostic_only=True,motion_authorized=False,board_pose_used=False,session=str(a.session),available_exact_sources=len(common),rows=rows,
        limitations=['Same D405 depth supports both estimates; the local SDK fit is not external ground truth.','This checks the current stationary view; it does not validate cross-pose camera/robot chain or authorize contact.','The production mask selects a local region; image overlays remain necessary to check motor identity.'])
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':main()
