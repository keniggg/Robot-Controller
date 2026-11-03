#!/usr/bin/env python3
"""Offline manual-ROI diagnostic: motor, adjacent mat/table and fixed board.
No ROS node, motion, target selection, replacement calibration or contact permit.
Manual ROI identifies a visual candidate only; no claim of production track identity.
"""
import argparse, hashlib, importlib.util, json, os
from pathlib import Path
import cv2
import numpy as np
import rosbag
from cv_bridge import CvBridge
HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('depth_audit',HERE/'analyze_depth_supported_board_motion.py')
D=importlib.util.module_from_spec(spec);spec.loader.exec_module(D)
MOUNT=Path('/media/zhuyupei/SanStand')

def transform(T,p):
    T=np.asarray(T);return np.asarray(p)@T[:3,:3].T+T[:3,3]

def board_interior(points,margin=.006):
    p=np.asarray(points)
    return (p[:,0]>-margin)&(p[:,0]<.18+margin)&(p[:,1]>-margin)&(p[:,1]<.135+margin)

def fit_table(points,normal_hint):
    n,d,meta=D.fit_plane(points,threshold=.002)
    if np.dot(n,normal_hint)<0:n,d=-n,-d
    if np.dot(n,normal_hint)<np.cos(np.radians(15)):raise ValueError('dominant surface is not approximately parallel to board')
    if meta['inlier_fraction']<.7 or meta['inlier_rms_mm']>2.:raise ValueError('insufficient independent table plane fit')
    return n,d,meta

def frame(g,image,depth,anchor_board,out,name):
    model=D.CM.RuntimeColorModel(g['quality']['camera_model'])
    profile=g['camera_parameters']['runtime_profile']
    if profile['geometry_intrinsics']!=profile['color_intrinsics']:raise ValueError('depth/color geometry mismatch')
    if model.fingerprint!=g['quality']['camera_model_fingerprint']:raise ValueError('camera fingerprint mismatch')
    if depth.dtype!=np.uint16 or depth.shape!=image.shape[:2]:raise ValueError('invalid image/depth pair')
    B=np.asarray(g['T_camera_board']);BC=np.linalg.inv(B)
    anchor_camera=transform(B,anchor_board)
    optical=D.O[:3,:3].T@anchor_camera
    center=np.asarray(model.sdk.rs2_project_point_to_pixel(model.intrinsics,optical.tolist()))
    yy,xx=np.mgrid[max(0,int(center[1])-85):min(480,int(center[1])+86):2,max(0,int(center[0])-85):min(640,int(center[0])+86):2]
    uv=np.c_[xx.ravel(),yy.ravel()]
    z=depth[uv[:,1],uv[:,0]]*g['camera_parameters']['depth_scale']
    pc=(D.rays(model,uv)*z[:,None])@D.O[:3,:3].T
    pb=transform(BC,pc)
    radial=np.linalg.norm(pb[:,:2]-anchor_board[:2],axis=1)
    valid=(z>.1)&(z<.7)&np.isfinite(pc).all(1)
    # Region selection is spatial only. Height is never pulled toward board plane.
    ring=valid&(radial>.022)&(radial<.055)&~board_interior(pb)
    if ring.sum()<100:raise ValueError('less than 100 independent adjacent table pixels')
    hint=B[:3,2].copy()
    if np.dot(hint,B[:3,3])>0:hint=-hint
    n,d,fit=fit_table(pc[ring],hint)
    inlier=ring&(np.abs(pc@n+d)<.002)
    diff=pb[inlier,:2]-anchor_board[:2]
    sectors=np.unique((diff[:,0]>0).astype(int)+2*(diff[:,1]>0).astype(int))
    if len(sectors)<3:raise ValueError('adjacent table lacks three-sector coverage')
    height=pc@n+d
    obj=valid&(radial<.023)&~board_interior(pb)&(height>.003)&(height<.04)
    if obj.sum()<15:raise ValueError('insufficient positive motor candidate surface')
    nb=B[:3,:3].T@n;db=d+float(n@B[:3,3])
    F=np.asarray(g['T_base_tool'])@np.asarray(g['T_tool_camera'])
    nf=F[:3,:3]@n;df=d-float(nf@F[:3,3])
    base_anchor=transform(F,anchor_camera)
    overlay=image.copy();overlay[uv[inlier,1],uv[inlier,0]]=[0,210,0];overlay[uv[obj,1],uv[obj,0]]=[255,0,255]
    cv2.circle(overlay,tuple(np.round(center).astype(int)),4,(0,255,255),1)
    cv2.imwrite(str(out/(name+'.png')),overlay)
    np.savez_compressed(out/(name+'.npz'),source_stamp_ns=g['source_stamp_ns'],pixels=uv,
        points_camera=pc,points_board=pb,table_inliers=inlier,motor_candidate=obj,
        table_plane_camera=np.r_[n,d],table_plane_board=np.r_[nb,db],table_plane_base=np.r_[nf,df])
    return dict(source_stamp_ns=g['source_stamp_ns'],phase=g['offline_stationary_phase'],
        quality_received_age_ms=g['quality_received_age_ms'],tf_received_age_ms=g['tf_received_age_ms'],
        camera_model_fingerprint=model.fingerprint,accepted_stamp_ns=g['accepted_stamp_ns'],
        table_fit=fit,table_sectors=sectors.tolist(),board_excluded=True,
        table_plane_camera=np.r_[n,d].tolist(),table_plane_board=np.r_[nb,db].tolist(),
        table_plane_base=np.r_[nf,df].tolist(),anchor_board=anchor_board.tolist(),anchor_base=base_anchor.tolist(),
        board_local_signed_table_distance_at_anchor_mm=float((nb@anchor_board+db)*1000),
        motor_candidate_points=int(obj.sum()),motor_height_p10_p50_p90_mm=(np.percentile(height[obj],[10,50,90])*1000).tolist(),
        motor_board_surface_median_m=np.median(pb[obj],axis=0).tolist(),
        motor_board_surface_p10_p90_m=np.percentile(pb[obj],[10,90],axis=0).tolist(),
        source_sha256=dict(rgb=hashlib.sha256(image.tobytes()).hexdigest(),z16=hashlib.sha256(depth.tobytes()).hexdigest()),
        accepted_positions_rad=g['accepted_positions_rad'],
        camera_parameters=g['camera_parameters'],
        exact_source_stamps_ns={k:g['source_stamp_ns'] for k in ('rgb','z16','charuco_quality','camera_board_tf')},
        independent_stationary_verifier='collect_paired_geometries: centred +/-0.4s, >=6 accepted, <=0.15deg range, <=0.25s gap',
        T_camera_board=B.tolist(),T_base_camera=F.tolist(),evidence_stem=name)

def analyze(session,out):
    out.mkdir(parents=True,exist_ok=True)
    baseline=json.loads((session/'baseline_geometry.json').read_text())
    geos,audit=D.collect_paired_geometries(session,baseline,maximum_per_window=5)
    wanted={g['source_stamp_ns'] for g in geos.values()}|{baseline['source_stamp_ns']}
    rgb={};depth={};bridge=CvBridge()
    with rosbag.Bag(str(session/'evidence.bag')) as bag:
        for topic,msg,_ in bag.read_messages(topics=['/bounded_response/snapshot_rgb','/bounded_wrist/paired_rgb','/bounded_joint23/paired_rgb','/supervisor/camera/depth/image_raw']):
            ns=msg.header.stamp.to_nsec()
            if ns not in wanted:continue
            if topic.endswith('depth/image_raw'):depth[ns]=bridge.imgmsg_to_cv2(msg,'passthrough')
            else:rgb[ns]=bridge.imgmsg_to_cv2(msg,'bgr8')
    # Same-motor visual seed chosen in reviewed J6 baseline, propagated between
    # these sessions using board coordinates. No cross-session base pose matching.
    seed_session=MOUNT/'实验记录/Robot-Controller/ros_log/bounded_wrist_response_diagnostic_20260928T125421Z_130257'
    seed=json.loads((seed_session/'baseline_geometry.json').read_text())
    # Ray intersects the independent board-local diagnostic reference z=0 to
    # select a neighbourhood only. Table fitting remains independent of this z.
    model=D.CM.RuntimeColorModel(seed['quality']['camera_model']);B=np.asarray(seed['T_camera_board'])
    ray=D.O[:3,:3]@D.rays(model,[[267,155]])[0]
    distance=float(np.dot(B[:3,2],B[:3,3])/np.dot(B[:3,2],ray))
    anchor=transform(np.linalg.inv(B),ray*distance)
    rows=[];rejected={}
    for name,g in geos.items():
        ns=g['source_stamp_ns']
        if ns not in rgb or ns not in depth:rejected[name]='missing exact RGB/Z16';continue
        try:rows.append(frame(g,rgb[ns],depth[ns],anchor,out,name.replace('_geometry.json','')))
        except ValueError as exc:rejected[name]=str(exc)
    summaries={}
    for phase in sorted({r['phase'] for r in rows}):
        rr=[r for r in rows if r['phase']==phase]
        summaries[phase]=dict(frames=len(rr),table_board_distance_mm=D.descriptive([r['board_local_signed_table_distance_at_anchor_mm'] for r in rr]),
            motor_height_quantiles_mm=D.descriptive([r['motor_height_p10_p50_p90_mm'] for r in rr]),
            table_rms_mm=D.descriptive([r['table_fit']['inlier_rms_mm'] for r in rr]))
    comparisons=[]
    ref=[r for r in rows if r['phase']=='baseline']
    for phase in summaries:
        if phase=='baseline':continue
        pairs=[]
        for a in ref:
            for b in [r for r in rows if r['phase']==phase]:
                metrics={}
                for chain,key,ak in [('board','table_plane_board','anchor_board'),('base_fk','table_plane_base','anchor_base')]:
                    p,q=np.asarray(a[key]),np.asarray(b[key]);common=.5*(np.asarray(a[ak])+b[ak])
                    metrics[chain]=float(((q[:3]-p[:3])@common+q[3]-p[3])*1000)
                pairs.append(metrics)
        comparisons.append(dict(phase=phase,pairs=len(pairs),signed_plane_delta_mm={k:D.descriptive([p[k] for p in pairs]) for k in ('board','base_fk')}))
    report=dict(session=str(session),diagnostic_only=True,motion_authorized=False,production_target_identity_verified=False,
        manual_motor_seed=dict(source_session=str(seed_session),source_stamp_ns=seed['source_stamp_ns'],pixel=[267,155],anchor_board=anchor.tolist()),
        model='SDK runtime rays; board and table planes separate; annulus excludes full board +6mm',
        driver_epoch_ns=json.loads((session/'plan.json').read_text())['frozen_epoch_ns'],
        board_reference_id='fixed_board_in_user_confirmed_layout_20260928_current_diagnostic',
        source_code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        paired_source_audit=audit,frames=rows,rejected=rejected,summaries=summaries,comparisons=comparisons,
        limits=['Manual visual candidate association is not verified original production target identity.',
                'Visible motor surface statistics change with view; no centroid registration or correction.',
                'Mouse mat is the observed supporting surface; depth is not external metrology.',
                'Small tested pose spans do not validate a complete approach or 10 mm correction.'])
    (out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(session=str(session),summaries=summaries,comparisons=comparisons,rejected=rejected),ensure_ascii=False),flush=True)

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path);p.add_argument('sessions',nargs='+',type=Path);a=p.parse_args()
    if not os.path.ismount(MOUNT) or MOUNT not in a.output.resolve().parents:raise RuntimeError('mounted SanStand required')
    for session in a.sessions:analyze(session,a.output/session.name)
if __name__=='__main__':main()
