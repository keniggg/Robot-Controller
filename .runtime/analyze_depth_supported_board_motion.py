#!/usr/bin/env python3
"""Offline source-paired depth-supported ChArUco motion, independent of grid scale.

Absolute board square length is never used to create 3D points. ChArUco supplies
pixel/ID correspondences and a dimensionless board layout. SDK rays intersect an
independently depth-fitted board plane. No ROS node, services, motion or writes to
calibration are present. Output must remain on mounted SanStand.
"""
import argparse
import bisect
from collections import defaultdict, Counter
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import numpy as np
import cv2
import rosbag
from cv_bridge import CvBridge

ROOT=Path(__file__).resolve().parents[1]
MOUNT=Path('/media/zhuyupei/SanStand')
O=np.eye(4);O[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]

def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result);return result

CM=module('depth_motion_camera',ROOT/'src/real-arm/alicia_d_calibration/scripts/charuco_tracker.py')
GEOM=module('depth_motion_geometry',ROOT/'.runtime/analyze_fixed_handeye_translation.py')


def rigid(source,target):
    a=np.asarray(source,dtype=float);b=np.asarray(target,dtype=float)
    if a.shape!=b.shape or a.ndim!=2 or a.shape[1]!=3 or len(a)<6:raise ValueError('at least six matched 3D points required')
    ac=a-a.mean(0);bc=b-b.mean(0)
    u,s,vt=np.linalg.svd(ac.T@bc)
    if s[1]<1e-10:raise ValueError('matched points are collinear')
    r=vt.T@np.diag([1.,1.,np.linalg.det(vt.T@u.T)])@u.T
    t=b.mean(0)-r@a.mean(0)
    T=np.eye(4);T[:3,:3]=r;T[:3,3]=t
    residual=(a@r.T+t)-b
    return T,dict(count=len(a),rms_mm=float(np.sqrt(np.mean(np.sum(residual**2,axis=1)))*1000),
        maximum_mm=float(np.max(np.linalg.norm(residual,axis=1))*1000),singular_values_m2=s.tolist(),
        residual_xyz_mm=(residual*1000).tolist())


def fit_plane(points,seed=2809,threshold=.0015):
    points=np.asarray(points,dtype=float)
    if len(points)<100:raise ValueError('fewer than 100 plane depth points')
    rng=np.random.default_rng(seed)
    subset=points[rng.choice(len(points),min(2500,len(points)),replace=False)]
    best=None
    for _ in range(200):
        triplet=subset[rng.choice(len(subset),3,replace=False)]
        n=np.cross(triplet[1]-triplet[0],triplet[2]-triplet[0]);norm=np.linalg.norm(n)
        if norm<1e-8:continue
        n/=norm;d=-float(n@triplet[0]);error=np.abs(subset@n+d)
        count=int(np.count_nonzero(error<threshold))
        cost=(count,-float(np.median(error)))
        if best is None or cost>best[0]:best=(cost,n,d)
    if best is None:raise ValueError('plane is degenerate')
    n,d=best[1:]
    for _ in range(3):
        keep=np.abs(points@n+d)<threshold
        if sum(keep)<100:raise ValueError('insufficient robust plane consensus')
        selected=points[keep];center=selected.mean(0)
        _u,_s,vt=np.linalg.svd(selected-center,full_matrices=False);n=vt[-1]
        if n[2]<0:n=-n
        d=-float(n@center)
    error=points@n+d;keep=np.abs(error)<threshold
    return n,d,dict(points=len(points),inliers=int(sum(keep)),inlier_fraction=float(np.mean(keep)),
        threshold_mm=threshold*1000,inlier_rms_mm=float(np.sqrt(np.mean(error[keep]**2))*1000),
        all_absolute_p50_p95_mm=(np.percentile(np.abs(error),[50,95])*1000).tolist(),
        normal_optical=n.tolist(),offset_m=d)


def make_board(square=1.):
    ar=cv2.aruco;dictionary=ar.getPredefinedDictionary(ar.DICT_5X5_100)
    board=(ar.CharucoBoard((12,9),square,square*.75,dictionary) if hasattr(ar,'CharucoBoard')
           else ar.CharucoBoard_create(12,9,square,square*.75,dictionary))
    if hasattr(board,'setLegacyPattern'):board.setLegacyPattern(True)
    return board,dictionary


def detect(image,scale=1.5,square=1.):
    ar=cv2.aruco;gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    gray=cv2.resize(gray,None,fx=scale,fy=scale,interpolation=cv2.INTER_CUBIC)
    board,dictionary=make_board(square)
    params=ar.DetectorParameters() if hasattr(ar,'DetectorParameters') else ar.DetectorParameters_create()
    params.cornerRefinementMethod=ar.CORNER_REFINE_NONE
    if hasattr(ar,'ArucoDetector'):corners,ids,_=ar.ArucoDetector(dictionary,params).detectMarkers(gray)
    else:corners,ids,_=ar.detectMarkers(gray,dictionary,parameters=params)
    if ids is None:raise ValueError('no markers')
    if hasattr(ar,'CharucoDetector'):
        cp,cids,_,_=ar.CharucoDetector(board,detectorParams=params).detectBoard(gray,None,None,corners,ids)
    else:_count,cp,cids=ar.interpolateCornersCharuco(corners,ids,gray,board)
    if cids is None or len(cids)<20:raise ValueError('fewer than 20 ChArUco corners')
    pixels=cp.reshape(-1,2).astype(float)/scale;indices=cids.reshape(-1).astype(int)
    if len(set(indices))!=len(indices):raise ValueError('ambiguous ChArUco IDs')
    points=board.getChessboardCorners() if hasattr(board,'getChessboardCorners') else board.chessboardCorners
    unit=np.asarray(points)[indices]/square
    return dict(ids=indices,pixels=pixels,unit_board_xyz=unit.astype(float),marker_count=len(ids),scale=scale)


def rays(model,pixels):
    return np.column_stack([model.normalized_points(pixels).reshape(-1,2),np.ones(len(pixels))])


def grid_spacing(points,unit):
    # All scale is supplied by depth-supported points. Unit board coordinates
    # only encode corner identity/layout and are dimensionless.
    uc=unit-unit.mean(0);pc=points-points.mean(0)
    u,s,vt=np.linalg.svd(uc.T@pc)
    det=np.linalg.det(vt.T@u.T);r=vt.T@np.diag([1.,1.,det])@u.T
    scale=float(np.sum(s*np.array([1.,1.,det]))/np.sum(uc**2))
    pred=uc@r.T*scale+points.mean(0)
    lengths=[]
    for i in range(len(points)):
        for j in range(i):
            difference=np.abs(unit[i]-unit[j])
            if np.isclose(difference.sum(),1.,atol=1e-6) and np.count_nonzero(difference>1e-6)==1:
                lengths.append(np.linalg.norm(points[i]-points[j])*1000)
    return dict(similarity_square_mm=scale*1000,delta_from_assumed_15mm=scale*1000-15.,
                similarity_residual_rms_mm=float(np.sqrt(np.mean(np.sum((pred-points)**2,axis=1)))*1000),
                adjacent_corner_pairs=len(lengths),adjacent_spacing_p05_p50_p95_mm=np.percentile(lengths,[5,50,95]).tolist() if lengths else None,
                interpretation='Depth-supported board metric; 15 mm used only for this diagnostic comparison, never for 3D reconstruction.')


def sample(geometry,image,depth,check_units=False):
    params=geometry['camera_parameters'];profile=params['runtime_profile']
    if profile['geometry_intrinsics']!=profile['color_intrinsics']:
        raise ValueError('Depth geometry rays differ from color rays; explicit extrinsic mapping required')
    model=CM.RuntimeColorModel(geometry['quality']['camera_model'])
    if model.fingerprint!=geometry['quality']['camera_model_fingerprint']:raise ValueError('camera model fingerprint mismatch')
    if depth.dtype!=np.uint16 or depth.shape!=image.shape[:2] or depth.shape!=(model.intrinsics.height,model.intrinsics.width):
        raise ValueError('exact Z16/image/model dimensions mismatch')
    detection=detect(image)
    hull=cv2.convexHull(detection['pixels'].astype(np.float32)).reshape(-1,2)
    mask=np.zeros(depth.shape,dtype=np.uint8);cv2.fillConvexPoly(mask,np.rint(hull).astype(np.int32),1)
    mask=cv2.erode(mask,np.ones((9,9),np.uint8),iterations=1)
    yy,xx=np.where(mask>0)
    # Every second source pixel, independent of PnP pose/metric coordinates.
    keep=(xx%2==0)&(yy%2==0);xx=xx[keep];yy=yy[keep]
    pixels=np.column_stack([xx,yy]);z=depth[yy,xx].astype(float)*float(params['depth_scale'])
    valid=(z>.03)&(z<1.)
    if sum(valid)<500:raise ValueError('insufficient independent interior board Z16 pixels')
    pix=pixels[valid];points=rays(model,pix)*z[valid,None]
    train=((pix[:,0]//16+pix[:,1]//16)%2)==0
    if min(sum(train),sum(~train))<100:raise ValueError('insufficient spatial holdout coverage')
    n,d,plane=fit_plane(points[train]);validation=points[~train]@n+d
    plane.update(roi_pixel_count=len(pixels),valid_depth_count=int(sum(valid)),depth_valid_fraction=float(np.mean(valid)),
                 selection='Convex hull of detected ID pixels, 4 px erosion, 2 px sampling; no PnP metric pose or square length used.',
                 validation_points=int(sum(~train)),validation_rms_mm=float(np.sqrt(np.mean(validation**2))*1000),
                 validation_abs_p50_p95_mm=(np.percentile(np.abs(validation),[50,95])*1000).tolist())
    rr=rays(model,detection['pixels']);den=rr@n
    if np.any(abs(den)<.1):raise ValueError('corner ray nearly parallel to fitted depth plane')
    distance=-d/den
    if np.any((distance<.03)|(distance>1.)):raise ValueError('invalid depth plane ray intersection')
    corners=rr*distance[:,None]
    g=grid_spacing(corners,detection['unit_board_xyz'])
    # Archived PnP uses physical square model; keep it independent of our fit.
    pnp=O.T@np.asarray(geometry['T_camera_board'])
    npnp=pnp[:3,2]
    if npnp[2]<0:npnp=-npnp
    angle=math.degrees(math.acos(float(np.clip(n@npnp,-1.,1.))))
    expected=(npnp@pnp[:3,3])/(rr@npnp)
    pnp_corners=detection['unit_board_xyz']*.015@pnp[:3,:3].T+pnp[:3,3]
    pnp_projection=np.asarray([model.sdk.rs2_project_point_to_pixel(model.intrinsics,p.tolist()) for p in pnp_corners])
    reprojection=np.linalg.norm(pnp_projection-detection['pixels'],axis=1)
    unit_test=None
    if check_units:
        other=detect(image,square=.015);om={int(i):p for i,p in zip(other['ids'],other['pixels'])}
        deltas=[p-om[int(i)] for i,p in zip(detection['ids'],detection['pixels']) if int(i) in om]
        unit_test=dict(common_ids=len(deltas),pixel_rms_change_unit_vs_15mm=float(np.sqrt(np.mean(np.sum(np.asarray(deltas)**2,axis=1)))),
                       note='Detection-only scale invariance audit; reconstructed 3D still uses the dimensionless detector and depth plane.')
    return dict(source_stamp_ns=geometry['source_stamp_ns'],ids=detection['ids'].tolist(),pixels=detection['pixels'].tolist(),
        depth_supported_corners_optical_m=corners.tolist(),unit_board_xyz=detection['unit_board_xyz'].tolist(),
        marker_count=detection['marker_count'],charuco_count=len(corners),plane=plane,grid_spacing=g,
        camera_model_fingerprint=model.fingerprint,
        depth_plane_vs_archived_pnp_normal_deg=angle,corner_optical_z_depth_minus_pnp_p05_p50_p95_mm=(np.percentile(distance-expected,[5,50,95])*1000).tolist(),
        archived_pnp_vs_redetected_pixel_rms=float(np.sqrt(np.mean(reprojection**2))),
        archived_pnp_vs_redetected_pixel_maximum=float(np.max(reprojection)),
        detector_unit_invariance=unit_test),model


def compare(first,second,g0,g1,model,xml):
    p0={int(i):p for i,p in zip(first['ids'],first['depth_supported_corners_optical_m'])}
    p1={int(i):p for i,p in zip(second['ids'],second['depth_supported_corners_optical_m'])}
    pixel1={int(i):p for i,p in zip(second['ids'],second['pixels'])}
    ids=sorted(set(p0)&set(p1))
    old=np.asarray([p0[i] for i in ids]);new=np.asarray([p1[i] for i in ids])
    mapping,metrics=rigid(old,new);depth_motion=GEOM.inv(mapping)
    predicted=old@mapping[:3,:3].T+mapping[:3,3]
    uv=np.asarray([model.sdk.rs2_project_point_to_pixel(model.intrinsics,p.tolist()) for p in predicted])
    per_pixel=np.linalg.norm(uv-np.asarray([pixel1[i] for i in ids]),axis=1)
    metrics.update(rigid_reprojection_rms_px=float(np.sqrt(np.mean(per_pixel**2))),rigid_reprojection_maximum_px=float(np.max(per_pixel)))
    ca=O.T@np.asarray(g0['T_camera_board']);cb=O.T@np.asarray(g1['T_camera_board'])
    pnp_motion=ca@GEOM.inv(cb)
    a0=GEOM.serial_fk_and_axes(xml,g0['accepted_positions_rad'][:6])[0]
    a1=GEOM.serial_fk_and_axes(xml,g1['accepted_positions_rad'][:6])[0]
    rel=GEOM.inv(a0)@a1;x=np.asarray(g0['T_tool_camera'])@O
    if not np.allclose(g0['T_tool_camera'],g1['T_tool_camera'],atol=1e-10,rtol=0):raise ValueError('Hand-eye changed across paired images')
    fk_motion=GEOM.inv(x)@rel@x
    motions=dict(depth_supported=depth_motion,archived_pnp=pnp_motion,accepted_fk_current_handeye=fk_motion)
    summary={key:dict(T_camera0_camera1=value.tolist(),rotation_angle_deg=math.degrees(np.linalg.norm(GEOM.log_so3(value[:3,:3]))),
                     translation_optical_xyz_mm=(value[:3,3]*1000).tolist(),translation_norm_mm=float(np.linalg.norm(value[:3,3])*1000)) for key,value in motions.items()}
    differences=[]
    for lhs,rhs in [('depth_supported','archived_pnp'),('depth_supported','accepted_fk_current_handeye'),('archived_pnp','accepted_fk_current_handeye')]:
        left,right=motions[lhs],motions[rhs]
        differences.append(dict(left=lhs,right=rhs,rotation_mismatch_deg=math.degrees(np.linalg.norm(GEOM.log_so3(left[:3,:3].T@right[:3,:3]))),
            translation_difference_xyz_mm=((left[:3,3]-right[:3,3])*1000).tolist(),translation_difference_norm_mm=float(np.linalg.norm(left[:3,3]-right[:3,3])*1000)))
    # Preserve planar spatial coverage rather than counting all corners as 3D rank.
    unit={int(i):p for i,p in zip(first['ids'],first['unit_board_xyz'])}
    coverage=np.ptp(np.asarray([unit[i] for i in ids]),axis=0)
    return dict(first_source_stamp_ns=first['source_stamp_ns'],second_source_stamp_ns=second['source_stamp_ns'],
        matched_ids=ids,matched_unit_grid_span=coverage.tolist(),rigid_fit=metrics,motions=summary,differences=differences,
        accepted_joint_delta_deg=np.degrees(np.asarray(g1['accepted_positions_rad'][:6])-g0['accepted_positions_rad'][:6]).tolist())


def collect_paired_geometries(session, template, maximum_per_window=8):
    """Recover exact archived paired-RGB metadata, with stationary source windows.

    Acquisition age remains <=350 ms for quality/board TF; old RGB archival
    delay never turns into a live acquisition or execution authorization.
    """
    quality={}; board={}; accepted={}; paired=set(); epochs=[]
    with rosbag.Bag(str(session/'evidence.bag')) as bag:
        for topic,msg,t in bag.read_messages(topics=['/charuco/quality','/tf','/alicia_d/accepted_joint_states',
                '/bounded_joint23/pair_provenance','/bounded_wrist/pair_provenance','/alicia_d/control_reference_epoch']):
            received=t.to_nsec()
            if topic=='/charuco/quality':
                q=json.loads(msg.data)
                if isinstance(q.get('source_stamp_ns'),int):quality[q['source_stamp_ns']]=(q,received)
            elif topic=='/tf':
                for m in msg.transforms:
                    if m.header.frame_id=='camera_link':
                        from tf.transformations import quaternion_matrix
                        p=m.transform.translation;q=m.transform.rotation
                        transform=quaternion_matrix([q.x,q.y,q.z,q.w]);transform[:3,3]=[p.x,p.y,p.z]
                        board[(m.header.stamp.to_nsec(),m.child_frame_id)]=(transform,received)
            elif topic in ('/bounded_joint23/pair_provenance','/bounded_wrist/pair_provenance'):
                p=json.loads(msg.data)
                if p.get('exact_rgb_board_quality') is True:paired.add(p['source_stamp_ns'])
            elif topic.endswith('control_reference_epoch'):epochs.append((received,msg.stamp.to_nsec()))
            else:
                values=dict(zip(msg.name,msg.position))
                accepted[msg.header.stamp.to_nsec()]=[values['Joint%d'%i] for i in range(1,7)]+[values['right_finger']]
    if not paired:return {},dict(reason='No paired RGB provenance topic; snapshot geometries still analyzed')
    plan=json.loads((session/'plan.json').read_text())
    events=[json.loads(line) for line in (session/'events.jsonl').read_text().splitlines() if line]
    commands=[e for e in events if e['event']=='command'];settled=[e for e in events if e['event']=='settled']
    if not commands:return {},dict(reason='No executed command phases; only archived snapshot geometries analyzed')
    starts=[dict(label='baseline',start=plan['stationary']['stationary_window_start_ns'],end=min(e['command_stamp_ns'] for e in commands))]
    for event in settled:
        end=min([e['command_stamp_ns'] for e in commands if e['command_stamp_ns']>event['ros_stamp_ns']] or [10**20])
        starts.append(dict(label=event['label'],start=event['stationary']['stationary_window_start_ns'],end=end))
    stamps=sorted(accepted);epochs.sort();etimes=[t for t,_e in epochs]
    xml=(session/'robot_description.urdf').read_text();x=np.asarray(template['T_tool_camera']);good=defaultdict(list);reasons=Counter()
    for ns in sorted(paired):
        window=next((w for w in starts if w['start']<=ns<w['end']),None)
        if window is None:reasons['outside_stationary_phase']+=1;continue
        if ns not in quality:reasons['missing_quality']+=1;continue
        q,received=quality[ns];match=board.get((ns,q.get('board_frame')))
        if match is None:reasons['missing_exact_board_tf']+=1;continue
        b,tfreceived=match
        if not (q.get('pose_ok') is True and q.get('charuco_count',0)>=35 and q.get('reprojection_rms_px',100)<=1.5
                and q.get('camera_model_fingerprint')==template['quality']['camera_model_fingerprint']
                and not q.get('camera_model_change_latched') and not q.get('camera_model_error')):
            reasons['quality_or_model']+=1;continue
        if not all(0<=age<=350_000_000 for age in (received-ns,tfreceived-ns)):
            reasons['quality_or_tf_older_than_350ms']+=1;continue
        index=bisect.bisect_left(stamps,ns);near=min(stamps[max(0,index-1):index+1],key=lambda t:abs(t-ns))
        if abs(near-ns)>100_000_000:reasons['encoder_separation']+=1;continue
        lo=bisect.bisect_right(stamps,ns-400_000_000)-1;hi=bisect.bisect_left(stamps,ns+400_000_000)
        if lo<0 or hi>=len(stamps):reasons['full_stationary_window_missing']+=1;continue
        ts=stamps[lo:hi+1];qq=np.asarray([accepted[t] for t in ts])
        if len(ts)<6 or max(np.diff(ts))>250_000_000 or np.max(np.ptp(qq[:,:6],axis=0))>math.radians(.15):
            reasons['not_stationary']+=1;continue
        ei=bisect.bisect_right(etimes,received)-1
        if ei<0 or epochs[ei][1]!=plan['frozen_epoch_ns']:reasons['epoch_mismatch']+=1;continue
        actual=accepted[near];a=GEOM.serial_fk_and_axes(xml,actual[:6])[0]
        geometry=dict(template,source_stamp_ns=ns,accepted_stamp_ns=near,quality=q,T_camera_board=b.tolist(),
                      T_base_tool=a.tolist(),T_base_board=(a@x@b).tolist(),accepted_positions_rad=actual,
                      offline_stationary_phase=window['label'],quality_received_age_ms=(received-ns)/1e6,
                      tf_received_age_ms=(tfreceived-ns)/1e6)
        good[window['label']].append(geometry)
    selected={};counts={}
    for phase,items in good.items():
        counts[phase]=len(items)
        indices=np.unique(np.linspace(0,len(items)-1,min(maximum_per_window,len(items))).round().astype(int))
        for index in indices:
            g=items[int(index)];selected['paired_%s_%d_geometry.json'%(phase,g['source_stamp_ns'])]=g
    return selected,dict(available_qualified_per_phase=counts,selected_per_phase=dict(Counter(g['offline_stationary_phase'] for g in selected.values())),
        rejected=dict(reasons),maximum_per_window=maximum_per_window,
        note='This derived metadata is offline only, includes original source times, and is never an execution geometry file.')


def descriptive(values):
    a=np.asarray(values,dtype=float)
    if not len(a):return None
    return dict(count=len(a),minimum=np.min(a,axis=0).tolist(),maximum=np.max(a,axis=0).tolist(),
                mean=np.mean(a,axis=0).tolist(),median=np.median(a,axis=0).tolist(),
                p05=np.percentile(a,5,axis=0).tolist(),p95=np.percentile(a,95,axis=0).tolist())


def compare_stationary_windows(rows,geos,models,xml):
    phases=defaultdict(list)
    for name,row in rows.items():
        if row.get('offline_stationary_phase'):phases[row['offline_stationary_phase']].append(name)
    windows={phase:dict(count=len(names),source_stamp_ns=[rows[n]['source_stamp_ns'] for n in names],
        actual_joint_deg=descriptive([np.degrees(geos[n]['accepted_positions_rad'][:6]) for n in names]),
        plane_normal_optical=descriptive([rows[n]['plane']['normal_optical'] for n in names]),
        depth_plane_validation_rms_mm=descriptive([rows[n]['plane']['validation_rms_mm'] for n in names]),
        grid_square_mm=descriptive([rows[n]['grid_spacing']['similarity_square_mm'] for n in names]))
        for phase,names in phases.items()}
    comparisons={}
    for a,b in [('baseline','outbound'),('baseline','return_original_sdk'),('outbound','return_original_sdk')]+[(p,p) for p in phases]:
        pairs=[]
        for i,first in enumerate(phases.get(a,[])):
            for j,second in enumerate(phases.get(b,[])):
                if a==b and j<=i:continue
                pairs.append(compare(rows[first],rows[second],geos[first],geos[second],models[first],xml))
        if not pairs:continue
        comparisons[a+'__'+b]=dict(pair_count=len(pairs),
            matched_corner_count=descriptive([len(p['matched_ids']) for p in pairs]),
            rigid_rms_mm=descriptive([p['rigid_fit']['rms_mm'] for p in pairs]),
            rigid_pixel_rms=descriptive([p['rigid_fit']['rigid_reprojection_rms_px'] for p in pairs]),
            motions={kind:{key:descriptive([p['motions'][kind][key] for p in pairs])
                for key in ('rotation_angle_deg','translation_norm_mm','translation_optical_xyz_mm')}
                for kind in pairs[0]['motions']},
            depth_minus_fk_angle_deg=descriptive([p['motions']['depth_supported']['rotation_angle_deg']-p['motions']['accepted_fk_current_handeye']['rotation_angle_deg'] for p in pairs]),
            pnp_minus_fk_angle_deg=descriptive([p['motions']['archived_pnp']['rotation_angle_deg']-p['motions']['accepted_fk_current_handeye']['rotation_angle_deg'] for p in pairs]),
            differences={pairs[0]['differences'][i]['left']+'__'+pairs[0]['differences'][i]['right']:
                {key:descriptive([p['differences'][i][key] for p in pairs])
                 for key in ('rotation_mismatch_deg','translation_difference_norm_mm','translation_difference_xyz_mm')}
                 for i in range(len(pairs[0]['differences']))})
    return dict(windows=windows,comparisons=comparisons,
        interpretation='Evenly distributed qualified source frames per stationary phase; cross-pairs are correlated descriptive distributions, never independent confidence intervals. Within-phase motion includes depth-plane and image noise plus measured encoder variation.')


def analyze(session,output,check_units=False,paired_stream=False):
    geos={f.name:json.loads(f.read_text()) for f in sorted(session.glob('*_geometry.json'))}
    if not geos:raise ValueError('no snapshot geometry files')
    paired_audit=None
    if paired_stream:
        extra,paired_audit=collect_paired_geometries(session,geos.get('baseline_geometry.json',next(iter(geos.values()))))
        geos.update(extra)
    wanted={g['source_stamp_ns'] for g in geos.values()};rgb={};depth={};counts={}
    bridge=CvBridge()
    with rosbag.Bag(str(session/'evidence.bag')) as bag:
        for topic,msg,_t in bag.read_messages(topics=['/bounded_response/snapshot_rgb','/bounded_joint23/paired_rgb','/bounded_wrist/paired_rgb','/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw']):
            ns=msg.header.stamp.to_nsec()
            if ns not in wanted:continue
            if topic.endswith('depth/image_raw'):depth[ns]=bridge.imgmsg_to_cv2(msg,'passthrough')
            else:rgb[ns]=bridge.imgmsg_to_cv2(msg,'bgr8')
    rows={};models={};failed={}
    for filename,g in geos.items():
        stamp=g['source_stamp_ns']
        if stamp not in rgb or stamp not in depth:
            failed[filename]='exact_source_rgb_or_z16_missing';continue
        try:
            rows[filename],models[filename]=sample(g,rgb[stamp],depth[stamp],check_units)
            rows[filename]['geometry']=filename
            if g.get('offline_stationary_phase'):rows[filename]['offline_stationary_phase']=g['offline_stationary_phase']
        except Exception as exc:failed[filename]=str(exc)
    xml=(session/'robot_description.urdf').read_text();pairs=[]
    ordered=sorted(rows,key=lambda key:rows[key]['source_stamp_ns'])
    if ordered:
        first='baseline_geometry.json' if 'baseline_geometry.json' in rows else ordered[0]
        for second in ordered:
            if rows[second]['source_stamp_ns']<=rows[first]['source_stamp_ns']:continue
            pair=compare(rows[first],rows[second],geos[first],geos[second],models[first],xml)
            pair.update(first_geometry=first,second_geometry=second);pairs.append(pair)
    result=dict(schema=1,kind='offline_source_paired_depth_supported_corner_motion',session=str(session),
        samples=list(rows.values()),failed_samples=failed,pairs=pairs,paired_stream_audit=paired_audit,
        stationary_window_comparisons=compare_stationary_windows(rows,geos,models,xml),
        absolute_board_square_used_for_3d=False,scope='No ROS node, command, parameter update or calibration deployment.',
        limits=['Depth is not external metrology and shares SDK camera rays with PnP.',
                'Plane is fitted using only RGB hull selection and measured depth; no 15 mm grid metric or PnP depth prior.',
                'Planar Kabsch uses two spatial dimensions; its out-of-plane accuracy is limited by depth-plane tilt noise.',
                'A single representative RGB-D frame per window cannot establish between-window uncertainty.',
                'Depth temporal/spatial filtering, camera-to-depth registration, and absolute depth scale may carry systematic bias.',
                'Per-corner rigid residual does not include errors shared by all corners or the whole depth plane.'])
    output.mkdir(parents=True,exist_ok=True)
    (output/'depth_supported_board_motion.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    (output/'geometry_metadata.json').write_text(json.dumps({name:geos[name] for name in rows},ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    (output/'analysis_source.py').write_bytes(Path(__file__).read_bytes())
    summary=dict(output=str(output),samples=[{k:r[k] for k in ('geometry','charuco_count','plane','grid_spacing','archived_pnp_vs_redetected_pixel_rms','detector_unit_invariance')} for r in rows.values()],failed=failed,
                 pairs=[dict(first=p['first_geometry'],second=p['second_geometry'],matched=len(p['matched_ids']),rigid_rms_mm=p['rigid_fit']['rms_mm'],pixel_rms=p['rigid_fit']['rigid_reprojection_rms_px'],motion_summary={key:{k:v[k] for k in ('rotation_angle_deg','translation_norm_mm')} for key,v in p['motions'].items()},differences=p['differences']) for p in pairs])
    print(json.dumps(summary,ensure_ascii=False,allow_nan=False))


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('session',type=Path);ap.add_argument('--output',required=True,type=Path);ap.add_argument('--check-detector-units',action='store_true');ap.add_argument('--paired-stream',action='store_true');args=ap.parse_args()
    if not os.path.ismount(MOUNT) or MOUNT not in args.output.resolve().parents:raise RuntimeError('Mounted SanStand output required; no fallback')
    analyze(args.session,args.output,args.check_detector_units,args.paired_stream)

if __name__=='__main__':main()
