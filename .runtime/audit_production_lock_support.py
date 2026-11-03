#!/usr/bin/env python3
"""Closed production_lock bag: current motor mask, adjacent table and board.

No ROS node, publisher, service, calibration change or motion authority. Only
current selected sources define the motor neighbourhood; no historical seed.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
MOUNT = Path('/media/zhuyupei/SanStand')
REQUIRED = ('rgb', 'depth', 'mask', 'status', 'object', 'quality', 'board')


def validate_source(row, messages, started_ns, after_ns=0):
    ns = row['source_stamp_ns']
    if type(ns) is not int or ns <= max(started_ns + 1_000_000_000,after_ns):
        raise ValueError('selected source does not follow this collector boundary')
    if row.get('board_required') is not True or row.get('current_board_observed') is not True:
        raise ValueError('current exact board evidence required')
    if set(REQUIRED) - set(messages):
        raise ValueError('incomplete selected tuple: ' + ','.join(sorted(set(REQUIRED)-set(messages))))
    if messages.get('_duplicate_selected'):
        raise ValueError('duplicate selected kind/source in bag')
    receipts = row.get('receipts_ns', {})
    if any(messages.get('_bag_receipts_ns',{}).get(k)!=receipts.get(k) for k in REQUIRED):
        raise ValueError('selected bag receipts differ from collector report')
    chosen=row.get('selected_receipt_ns')
    if type(chosen) is not int or not 0<=chosen-ns<=350_000_000 or chosen<max(receipts.values(),default=chosen):
        raise ValueError('selected tuple acquisition receipt is delayed or precedes a member')
    if any(type(receipts.get(k)) is not int or not 0 <= receipts[k]-ns <= 350_000_000 for k in REQUIRED):
        raise ValueError('missing or delayed selected receipt')
    s, q = messages['status'], messages['quality']
    if (s.get('state') != 'ready' or s.get('target_locked') is not True
            or s.get('target_lost') is True or s.get('exact_rgbd') is not True):
        raise ValueError('production source is not locked/ready/exact')
    if any(s.get(k) != ns for k in ('source_stamp_ns','color_stamp_ns','depth_stamp_ns','tf_lookup_stamp_ns')):
        raise ValueError('production RGB/depth/TF source mismatch')
    if (q.get('source_stamp_ns') != ns or q.get('pose_ok') is not True
            or q.get('charuco_count',0) < 35 or (q.get('edge_clearance_px') or 0) < 20
            or q.get('camera_model_change_latched') is True or q.get('camera_model_error')
            or not isinstance(q.get('reprojection_rms_px'),(int,float))
            or not 0 <= q['reprojection_rms_px'] <= 1.5):
        raise ValueError('current board quality does not meet collector contract')
    if (q.get('board_size') != [12,9] or q.get('dictionary_id') != 'DICT_5X5_100'
            or not np.isclose(q.get('square_length_m',0),.015,atol=1e-12,rtol=0)):
        raise ValueError('board layout differs from explicit 180 by 135 mm ROI model')
    for key in ('generation','selection_stamp_ns','anchor_source_stamp_ns'):
        if row.get(key) != s.get(key):
            raise ValueError('selected production identity mismatch: '+key)
    if row.get('camera_model_fingerprint') != q.get('camera_model_fingerprint'):
        raise ValueError('selected camera fingerprint mismatch')
    if (type(s.get('generation')) is not int or s['generation']<1
            or type(s.get('selection_stamp_ns')) is not int or s['selection_stamp_ns']<=0
            or type(s.get('anchor_source_stamp_ns')) is not int
            or not s['selection_stamp_ns'] <= s['anchor_source_stamp_ns'] <= ns):
        raise ValueError('source/anchor does not belong to a valid active selection')
    return ns


def adjacent_region(points_board, center_board, valid, object_excluded):
    """Exclude the complete current board footprint plus 6 mm on every edge."""
    p = np.asarray(points_board)
    radial = np.linalg.norm(p[:,:2]-np.asarray(center_board)[:2],axis=1)
    board = ((p[:,0] >= -.006)&(p[:,0] <= .186)
             &(p[:,1] >= -.006)&(p[:,1] <= .141))
    return np.asarray(valid)&~np.asarray(object_excluded)&~board&(radial>.022)&(radial<.055), board


def support_pass(comparison):
    return (np.isfinite(comparison['normal_deg']) and np.isfinite(comparison['max_footprint_difference_mm'])
            and comparison['normal_deg'] <= 4. and comparison['max_footprint_difference_mm'] <= 4.)



def complete_support_footprint(measured_points, all_mask_rays_camera, table):
    """Plane-comparison query domain, never fabricated object/depth samples.

    Include measured orthogonal projections and every silhouette pixel's ray
    intersection, so missing edge depth cannot silently shrink the queried area.
    """
    plane=np.asarray(table,dtype=float);plane=plane/np.linalg.norm(plane[:3])
    measured=np.asarray(measured_points,dtype=float);rays=np.asarray(all_mask_rays_camera,dtype=float)
    denominators=rays@plane[:3]
    if not len(rays) or not np.isfinite(rays).all() or np.any(abs(denominators)<1e-8):
        raise ValueError('full mask ray support domain unavailable')
    distance=-plane[3]/denominators
    if not np.isfinite(distance).all() or np.any(distance<=0):
        raise ValueError('full mask ray support intersections are not forward/finite')
    silhouette=rays*distance[:,None]
    orthogonal=measured-(measured@plane[:3]+plane[3])[:,None]*plane[:3]
    return np.vstack((orthogonal,silhouette)),orthogonal,silhouette


def stationary_window(accepted, source_ns):
    rows = [(s,q) for s,q in accepted if source_ns-800_000_000 <= s <= source_ns]
    if len(rows)<6:
        raise ValueError('fewer than six accepted samples in preceding 800 ms')
    stamps=np.array([s for s,_ in rows],dtype=np.int64)
    values=np.asarray([q for _,q in rows])
    gaps=np.diff(np.r_[source_ns-800_000_000,stamps,source_ns])/1e9
    if gaps.max()>.25 or np.ptp(values[:,:6],axis=0).max()>np.radians(.15):
        raise ValueError('accepted feedback lacks independent stationary window')
    return dict(count=len(rows),first_ns=int(stamps[0]),last_ns=int(stamps[-1]),
                maximum_gap_sec=float(gaps.max()),joint_range_deg=np.degrees(np.ptp(values[:,:6],axis=0)).tolist())


def transform_matrix(t):
    q=t.transform.rotation;v=np.array([q.x,q.y,q.z,q.w]);v=v/np.linalg.norm(v);x,y,z,w=v
    T=np.eye(4);T[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                         [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                         [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    p=t.transform.translation;T[:3,3]=[p.x,p.y,p.z]
    return T


def audit_frame(messages, camera, T_base_camera, out, ns):
    # Existing independently reviewed SDK rays, robust plane fitter and metrics.
    import cv2
    from cv_bridge import CvBridge
    from audit_motor_adjacent_table import D, transform, fit_table
    from audit_centered_production_support import compare_plane, plane_to_local, normalized
    b=CvBridge();q=messages['quality'];s=messages['status']
    image=b.imgmsg_to_cv2(messages['rgb'],'bgr8')
    depth=b.imgmsg_to_cv2(messages['depth'],'passthrough')
    mask=b.imgmsg_to_cv2(messages['mask'],'mono8')>0
    model=D.CM.RuntimeColorModel(q['camera_model'])
    if model.fingerprint!=q['camera_model_fingerprint']:
        raise ValueError('SDK camera fingerprint differs from selected quality')
    if camera['runtime_profile']['geometry_intrinsics']!=camera['runtime_profile']['color_intrinsics']:
        raise ValueError('depth geometry differs from RGB ray geometry')
    if q['camera_model']['color_intrinsics']!=camera['runtime_profile']['color_intrinsics']:
        raise ValueError('archived camera parameters differ from selected source model')
    if depth.dtype!=np.uint16 or depth.shape!=image.shape[:2] or mask.shape!=depth.shape:
        raise ValueError('RGB/Z16/mask dimensions or encoding mismatch')
    if depth.shape!=(model.intrinsics.height,model.intrinsics.width):
        raise ValueError('image dimensions differ from ray model')
    cv2.imwrite(str(out/(str(ns)+'_raw.png')),image)
    cv2.imwrite(str(out/(str(ns)+'_mask.png')),mask.astype(np.uint8)*255)
    B=transform_matrix(messages['board']);inverse=np.linalg.inv(B)
    ys,xs=np.nonzero(mask);uvmask=np.c_[xs,ys]
    zmask=depth[ys,xs]*camera['depth_scale'];good=(zmask>.1)&(zmask<.7)
    if good.sum()<60:
        raise ValueError('fewer than 60 measured production motor mask pixels')
    motor_camera=(D.rays(model,uvmask[good])*zmask[good,None])@D.O[:3,:3].T
    motor_board=transform(inverse,motor_camera);center=np.median(motor_board,axis=0)
    yy,xx=np.mgrid[0:depth.shape[0]:2,0:depth.shape[1]:2];uv=np.c_[xx.ravel(),yy.ravel()]
    z=depth[uv[:,1],uv[:,0]]*camera['depth_scale']
    pc=(D.rays(model,uv)*z[:,None])@D.O[:3,:3].T;pb=transform(inverse,pc)
    valid=(z>.1)&(z<.7)&np.isfinite(pc).all(1)
    expanded=cv2.dilate(mask.astype(np.uint8),np.ones((9,9),np.uint8))>0
    ring,excluded=adjacent_region(pb,center,valid,expanded[uv[:,1],uv[:,0]])
    if ring.sum()<100:
        raise ValueError('less than 100 adjacent pixels after board/mask exclusion')
    hint=B[:3,2].copy()
    if hint@B[:3,3]>0:hint=-hint
    n,d,fit=fit_table(pc[ring],hint);table=normalized(np.r_[n,d])
    inlier=ring&(abs(pc@n+d)<.002)
    diff=pb[inlier,:2]-center[:2];sectors=np.unique((diff[:,0]>0).astype(int)+2*(diff[:,1]>0).astype(int))
    if len(sectors)<3:
        raise ValueError('adjacent table lacks original three-sector coverage')
    interior=valid&~expanded[uv[:,1],uv[:,0]]&(pb[:,0]>.012)&(pb[:,0]<.168)&(pb[:,1]>.012)&(pb[:,1]<.123)
    bn,bd,boardfit=fit_table(pc[interior],hint);board=normalized(np.r_[bn,bd])
    prod=plane_to_local(T_base_camera,s['geometry_metrics']['support_plane_base'])
    if prod[:3]@table[:3]<0:prod=-prod
    all_mask_rays=D.rays(model,uvmask)@D.O[:3,:3].T
    footprint,measured_footprint,full_mask_projection=complete_support_footprint(motor_camera,all_mask_rays,table)
    comparisons=dict(production_vs_adjacent_table=compare_plane(prod,table,footprint),
        production_vs_board_top=compare_plane(prod,board,footprint),
        board_top_vs_adjacent_table=compare_plane(board,table,footprint))
    height=(motor_camera@table[:3]+table[3])*1000
    overlay=image.copy();overlay[uv[inlier,1],uv[inlier,0]]=[0,210,0]
    contours,_=cv2.findContours(mask.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(overlay,contours,-1,(255,255,0),1)
    cv2.imwrite(str(out/(str(ns)+'_raw.png')),image)
    cv2.imwrite(str(out/(str(ns)+'_audit.png')),overlay)
    np.savez_compressed(out/(str(ns)+'_measured.npz'),source_stamp_ns=ns,rgb=image,z16=depth,
        mask=mask,pixels=uv,points_camera=pc,points_board=pb,board_excluded=excluded,
        adjacent_table_region=ring,table_inliers=inlier,board_interior=interior,
        motor_points_camera=motor_camera,motor_footprint_camera=footprint,
        measured_orthogonal_footprint_camera=measured_footprint,full_mask_ray_table_projection_camera=full_mask_projection)
    return dict(source_stamp_ns=ns,camera_model_fingerprint=model.fingerprint,
        production_status=s,object_label=messages['object'].label,
        motor_pixel_median=np.median(uvmask,axis=0).tolist(),mask_pixels=int(mask.sum()),
        measured_mask_pixels=int(good.sum()),full_mask_support_projection_pixels=len(full_mask_projection),
        support_query_domain='union of measured orthogonal footprint and all mask SDK ray/table intersections; projected queries are not measured object points',
        motor_anchor_board_current_mask_m=center.tolist(),
        table_ring_m=[.022,.055],board_exclusion_margin_m=.006,object_mask_dilation_px=4,
        table_fit=fit,table_sectors=sectors.tolist(),board_depth_fit=boardfit,
        T_camera_board=B.tolist(),T_base_camera=T_base_camera.tolist(),
        production_plane_camera=prod.tolist(),adjacent_table_plane_camera=table.tolist(),board_top_plane_camera=board.tolist(),
        motor_height_p10_p50_p90_mm=np.percentile(height,[10,50,90]).tolist(),
        production_support_consistent=support_pass(comparisons['production_vs_adjacent_table']),
        production_support_matches_board_top=support_pass(comparisons['production_vs_board_top']),
        source_array_sha256={k:hashlib.sha256(v.tobytes()).hexdigest() for k,v in [('rgb',image),('z16',depth),('mask',mask)]},
        **comparisons)


def analyze(session,out,after_ns=0):
    import rosbag,rospy,tf2_ros
    sys.path.insert(0,str(HERE))
    evidence=json.loads((session/'report.json').read_text());camera=json.loads((session/'camera_parameters.json').read_text())
    selected=evidence['selected'];wanted={r['source_stamp_ns'] for r in selected}
    frames={ns:{} for ns in wanted};accepted=[];epochs=[];sdk=[];remote=[]
    buffer=tf2_ros.Buffer(cache_time=rospy.Duration(120),debug=False)
    with rosbag.Bag(str(session/'evidence.bag')) as bag:
        for topic,m,t in bag.read_messages():
            if topic in ('/tf','/tf_static'):
                for tr in m.transforms:
                    (buffer.set_transform_static if topic=='/tf_static' else buffer.set_transform)(tr,'closed_archive')
            elif topic=='/alicia_d/accepted_joint_states':
                named=dict(zip(m.name,m.position));keys=['Joint%d'%i for i in range(1,7)]
                if not all(k in named for k in keys):keys=['joint%d'%i for i in range(1,7)]
                if not all(k in named for k in keys):raise ValueError('unknown accepted joint schema')
                accepted.append((m.header.stamp.to_nsec(),[named[k] for k in keys]))
            elif topic=='/alicia_d/control_reference_epoch':epochs.append((t.to_nsec(),m.stamp.to_nsec()))
            elif topic=='/alicia_d/control_reference':sdk.append((m.header.stamp.to_nsec(),list(m.position)))
            elif topic=='/grasp_6d/object_geometry':
                remote.append(dict(topic=topic,receipt_ns=t.to_nsec(),source_stamp_ns=m.header.stamp.to_nsec(),valid=m.valid,target_track_id=m.target_track_id,failure_reason=m.failure_reason))
            elif topic=='/grasp_6d/pipeline_metrics':
                remote.append(dict(topic=topic,receipt_ns=t.to_nsec(),data=json.loads(m.data)))
            elif topic.startswith('/production_lock/selected_'):
                kind=topic.rsplit('_',1)[-1]
                if kind in ('status','quality'):
                    value=json.loads(m.data);ns=value.get('source_stamp_ns')
                elif kind=='board':
                    if len(m.transforms)!=1:raise ValueError('ambiguous selected board TF')
                    value=m.transforms[0];ns=value.header.stamp.to_nsec()
                else:value=m;ns=m.header.stamp.to_nsec()
                if ns in frames:
                    if kind in frames[ns]:frames[ns].setdefault('_duplicate_selected',[]).append(kind)
                    else:
                        frames[ns][kind]=value
                        frames[ns].setdefault('_bag_receipts_ns',{})[kind]=t.to_nsec()
    accepted.sort();epochs.sort();sdk.sort();rows=[];rejected=[]
    for row in selected:
        ns=row['source_stamp_ns']
        try:
            validate_source(row,frames[ns],evidence['started_ns'],after_ns)
            f=frames[ns]
            for k in ('rgb','depth','mask'):
                if f[k].header.stamp.to_nsec()!=ns or f[k].header.frame_id!='camera_link':raise ValueError('selected image/object frame mismatch')
            if f['object'].header.stamp.to_nsec()!=ns or f['object'].header.frame_id!='base_link':raise ValueError('selected production object must use base_link')
            if f['board'].header.frame_id!='camera_link' or f['board'].child_frame_id!=f['quality']['board_frame']:raise ValueError('selected board frame mismatch')
            if not f['object'].detected:raise ValueError('selected object is not detected')
            stationary=stationary_window(accepted,ns)
            prior_epoch=[e for receipt,e in epochs if receipt<=row['selected_receipt_ns']]
            if not prior_epoch or len(set(prior_epoch))!=1 or not 0<prior_epoch[-1]<=ns:raise ValueError('missing, future or changed driver epoch')
            prior_sdk=[(stamp,q) for stamp,q in sdk if stamp<=ns]
            if not prior_sdk or ns-prior_sdk[-1][0]>350_000_000:raise ValueError('missing fresh successful SDK reference')
            T=transform_matrix(buffer.lookup_transform_core('base_link','camera_link',rospy.Time(ns//10**9,ns%10**9)))
            result=audit_frame(f,camera,T,out,ns)
            result.update(stationary=stationary,driver_epoch_ns=prior_epoch[-1],sdk_source_stamp_ns=prior_sdk[-1][0],sdk_positions_rad=prior_sdk[-1][1],selected_receipts_ns=row['receipts_ns'])
            rows.append(result)
        except Exception as error:rejected.append(dict(source_stamp_ns=ns,reason=str(error)))
    archive_ok=evidence.get('writer',{}).get('archive_complete') is True and not evidence.get('faults')
    identity={(r['production_status']['generation'],r['production_status']['selection_stamp_ns'],r['production_status'].get('anchor_source_stamp_ns'),r['driver_epoch_ns'],r['camera_model_fingerprint']) for r in rows}
    source_spacing_ok=(len(selected)==3 and all(b['source_stamp_ns']-a['source_stamp_ns']>=200_000_000 for a,b in zip(selected,selected[1:])))
    complete=archive_ok and source_spacing_ok and len(wanted)==3 and len(rows)==3 and not rejected and len(identity)==1
    result=dict(schema=1,session=str(session),diagnostic_only=True,motion_authorized=False,
        evidence_complete=complete,collector_archive_complete=archive_ok,selected_count=len(selected),
        original_collector_200ms_source_spacing_preserved=source_spacing_ok,rows=rows,rejected=rejected,
        production_support_consistent=complete and all(r['production_support_consistent'] for r in rows),
        source_after_cutoff_ns=after_ns,robot_description_sha256=hashlib.sha256((session/'robot_description.urdf').read_bytes()).hexdigest(),
        input_sha256={n:hashlib.sha256((session/n).read_bytes()).hexdigest() for n in ('report.json','camera_parameters.json','collector.py')},
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),remote_evidence_recorded=remote,
        limits=['Current production mask selects the motor neighbourhood; overlays require identity review.',
                'Same D405 depth is not independent external metrology or fingertip contact validation.',
                'No historical board anchor, motor coordinate, plan or model permission is reused.',
                'Remote identity messages are preserved, not promoted to a verified motion binding.'])
    (out/'report.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('session',type=Path)
    p.add_argument('--output',type=Path);p.add_argument('--after-ns',type=int,default=0);args=p.parse_args()
    out=args.output or args.session/'current_support_audit'
    if not os.path.ismount(MOUNT) or MOUNT not in out.resolve().parents:raise RuntimeError('mounted SanStand output required')
    out.mkdir(parents=True,exist_ok=False)
    result=analyze(args.session,out,args.after_ns)
    print(json.dumps({k:result[k] for k in ('evidence_complete','selected_count','production_support_consistent','rejected')},ensure_ascii=False))
    return (0 if result['production_support_consistent'] else 3) if result['evidence_complete'] else 2


if __name__=='__main__':sys.exit(main())
