"""Offline fixed-board return: RGB background controls and rotation invariants.

Diagnostic masks below are explicit pixel regions for this 640x480 scene only;
they are not object-specific production logic or robot actions. Never uses depth
or TF to choose image correspondences or RGB camera/board pose.
"""
import argparse
import hashlib
import importlib.util
import itertools
import json
from pathlib import Path

import cv2
import numpy as np

CORE_PATH=Path(__file__).with_name('analyze_rgb_charuco_coordinate_chain.py')
SPEC=importlib.util.spec_from_file_location('rgb_background_core',CORE_PATH)
CORE=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(CORE)


def board_exclusion(image,model,frame,spec):
    w,h=np.asarray(spec['board_size'])*float(spec['square_length_m'])
    points=np.array([[0,0,0],[w,0,0],[w,h,0],[0,h,0]])
    T=np.asarray(frame['rgb']['T_optical_board'])
    pixels=model.project(points,cv2.Rodrigues(T[:3,:3])[0],T[:3,3])
    mask=np.zeros(image.shape[:2],np.uint8)
    cv2.fillConvexPoly(mask,np.rint(pixels).astype(np.int32),255)
    return cv2.dilate(mask,np.ones((23,23),np.uint8))


def diagnostic_masks(image,excluded_board):
    if image.shape!=(480,640,3):raise ValueError('SCENE_DIAGNOSTIC_MASK_REQUIRES_640x480')
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    red=((hsv[:,:,0]<25)|(hsv[:,:,0]>165))&(hsv[:,:,1]>100)&(hsv[:,:,2]>50)
    pad=np.uint8(red)*255;pad[:25]=0;pad[:,:180]=0;pad[excluded_board>0]=0
    regions={'mousemat_texture':pad}
    # Independent non-mat surfaces; left hands/keyboard are excluded. These
    # elevated surfaces are not assumed coplanar with the board/mousemat.
    for name,(x0,x1,y0,y1)in {'laptop_shell':(545,638,15,240),
          'phone_whitebox':(205,340,5,100),'hard_table_strip':(165,180,30,470)}.items():
        mask=np.zeros(image.shape[:2],np.uint8);mask[y0:y1,x0:x1]=255
        if name=='phone_whitebox':mask[red]=0
        mask[excluded_board>0]=0;regions[name]=mask
    return regions


def track_region(first,second,mask):
    A=cv2.cvtColor(first,cv2.COLOR_BGR2GRAY);B=cv2.cvtColor(second,cv2.COLOR_BGR2GRAY)
    source=cv2.goodFeaturesToTrack(A,800,.005,4,mask=mask,blockSize=7)
    if source is None:return np.empty((0,2)),np.empty((0,2)),dict(detected=0,accepted=0)
    target,status,error=cv2.calcOpticalFlowPyrLK(A,B,source,None,winSize=(21,21),maxLevel=3)
    back,status2,_=cv2.calcOpticalFlowPyrLK(B,A,target,None,winSize=(21,21),maxLevel=3)
    fb=np.linalg.norm(back[:,0]-source[:,0],axis=1)
    good=(status.ravel()>0)&(status2.ravel()>0)&(fb<.5)&(error.ravel()<25)
    return source[good,0].astype(float),target[good,0].astype(float),dict(
        detected=len(source),accepted=int(good.sum()),forward_backward_limit_px=.5,
        LK_photometric_limit=25,quality='RGB-only bidirectional LK; all accepted correspondences retained')


def project_normalized(model,points):
    points=np.asarray(points);return model.project(points,np.zeros(3),np.zeros(3))


def pad_homography(model,source,target):
    X=model.normalized_points(source).reshape(-1,2);Y=model.normalized_points(target).reshape(-1,2)
    folds=((source[:,0]//32+source[:,1]//32)%2).astype(int);train=folds==0
    if min(train.sum(),(~train).sum())<15:raise ValueError('BACKGROUND_SPATIAL_HOLDOUT_INSUFFICIENT')
    color=model.evidence['color_intrinsics'];scale=(color['fx']+color['fy'])/2
    H,inliers=cv2.findHomography(X[train],Y[train],cv2.RANSAC,.8/scale)
    if H is None:raise ValueError('BACKGROUND_HOMOGRAPHY_UNAVAILABLE')
    predicted=np.c_[X,np.ones(len(X))]@H.T;predicted/=predicted[:,2:3]
    residual=np.linalg.norm(project_normalized(model,predicted)-target,axis=1)
    return dict(H_source_to_target_normalized=(H/H[2,2]).tolist(),train_count=int(train.sum()),
         train_inlier_count=int(inliers.sum()),heldout_count=int((~train).sum()),
         all_heldout_error_px=CORE.error_stats(residual[~train]),
         all_correspondence_error_px=CORE.error_stats(residual),
         spatial_tile_px=32,RANSAC_train_limit_px=.8,
         limitation='Conditional spatial holdout only; neighboring LK patches/pixels and RGB intrinsics are shared.')


def background_pair(model,first,second,A,B,spec):
    masks=diagnostic_masks(A,board_exclusion(A,model,first,spec));results={}
    for name,mask in masks.items():
        source,target,quality=track_region(A,B,mask);result=dict(quality=quality,source_mask_pixels=int((mask>0).sum()))
        if len(source):
            flow=target-source;result.update(source_pixels=source.tolist(),target_pixels=target.tolist(),
                median_flow_xy_px=np.median(flow,axis=0).tolist(),
                flow_p05_p95_xy_px=np.percentile(flow,[5,95],axis=0).tolist(),
                flow_norm_px=CORE.error_stats(np.linalg.norm(flow,axis=1)),
                fraction_upward_over_2px=float(np.mean(flow[:,1]<-2)))
        if name=='mousemat_texture'and len(source)>=30:
            result['independent_spatial_homography']=pad_homography(model,source,target)
            # Board-only PnP predicts points intersecting its plane. Mousemat
            # thickness/shape remains a possible parallax source; no depth/TF.
            T=np.asarray(first['rgb']['T_optical_board']);motion=np.asarray(second['rgb']['T_optical_board'])@np.linalg.inv(T)
            rays=np.c_[model.normalized_points(source).reshape(-1,2),np.ones(len(source))]
            n=T[:3,2];d=float(n@T[:3,3]);points=rays*(d/(rays@n))[:,None]
            prediction=project_normalized(model,CORE.mappts(motion,points))
            result['board_RGB_plane_prediction_error_px']=CORE.error_stats(np.linalg.norm(prediction-target,axis=1))
            result['plane_limitation']='Board/mousemat may differ in height/shape; low residual is co-motion evidence,not hard-table or unique hardware proof.'
        else:result['plane_limitation']='Only observed RGB pixel motion is compared; these surfaces are not assumed coplanar with the board.'
        if name=='hard_table_strip':result['region_limitation']='Narrow weakly textured strip; LK windows/occlusion/aperture and the nearby keyboard limit interpretation.'
        results[name]=result
    return dict(first_label=first['label'],second_label=second['label'],regions=results)


def fold_angle_range(first,second):
    values=[CORE.rotation_angle_deg((np.asarray(a['T_optical_board'])@np.linalg.inv(np.asarray(b['T_optical_board'])))[:3,:3])
        for a,b in itertools.product(first['rgb']['spatial_folds'],second['rgb']['spatial_folds'])]
    return [min(values),max(values)]


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--session',type=Path,required=True)
    args=parser.parse_args();session=args.session.resolve();mount=Path('/media/zhuyupei/SanStand')
    if not mount.is_mount()or mount not in session.parents:raise ValueError('MOUNTED_EXTERNAL_SESSION_REQUIRED')
    cv2.setNumThreads(1);folder=session/'rgb_charuco_coordinate_chain'
    core_path=folder/'result.json';pairs_path=folder/'all_motion_pairs.json'
    core=json.loads(core_path.read_text());pairs=json.loads(pairs_path.read_text())
    inputs=dict(pairs['input_sha256_by_path']);inputs[str(pairs_path)]=sha(pairs_path);inputs[str(Path(__file__).resolve())]=sha(__file__)
    for path,digest in inputs.items():
        if sha(path)!=digest:raise ValueError('INPUT_HASH_MISMATCH: '+path)
    frames={f['label']:f for f in core['frames']};lookup={(p['first_label'],p['second_label']):p for p in pairs['pairs']}
    events=[json.loads(line)for line in (session/'events.jsonl').read_text().splitlines()]
    complete=[e for e in events if e['event']=='step_complete'];begins={e['label']:e for e in events if e['event']=='step_begin'}
    model=CORE.runtime_model(json.loads((session/'camera_parameters.json').read_text()))
    steps=[];previous='baseline'
    for event in complete:
        label=event['label'];pair=lookup[previous,label];rot=pair['rotation_measurements']
        steps.append(dict(step=event['step'],label=label,previous=previous,
            real_q_change_deg=pair['q_before_bracket_delta_deg'],command_target_sdk_rad=begins[label]['target_sdk'],
            TF_relative_rotation_deg=rot['recorded_TF']['rotation_angle_deg'],
            independent_RGB_relative_rotation_deg=rot['RGB']['rotation_angle_deg'],
            RGB_fold_relative_angle_range_deg=fold_angle_range(frames[previous],frames[label]),
            RGB_minus_TF_angle_deg=rot['RGB']['rotation_angle_deg']-rot['recorded_TF']['rotation_angle_deg'],
            RGB_TF_rotation_disagreement_deg=pair['RGB_TF_rotation_disagreement_deg'],
            board_point_drift_RMS_mm=pair['fixed_board_base_point_drift_mm']['rms'],
            depth_relative_rotation_deg=rot.get('depth_supported',{}).get('rotation_angle_deg')))
        previous=label
    returns=[]
    for label,frame in frames.items():
        if '_return'in label or label=='return_late':
            pair=lookup['baseline',label]
            returns.append(dict(label=label,real_q_change_from_baseline_deg=pair['q_before_bracket_delta_deg'],
              board_point_drift_mm=pair['fixed_board_base_point_drift_mm'],
              board_centroid_shift_xyz_mm=pair['fixed_board_base_centroid_shift_xyz_mm'],
              RGB_TF_rotation_disagreement_deg=pair['RGB_TF_rotation_disagreement_deg']))
    labels=[('factory_board_source','baseline'),('baseline','repeat1_j2_negative3'),
       ('repeat1_j2_negative3','repeat1_j2_negative3_return'),('baseline','repeat1_j2_negative3_return'),
       ('baseline','repeat1_j2_positive3_return'),('baseline','return_late'),
       ('repeat2_j5_negative3_return','return_late')]
    background=[]
    for a,b in labels:
        def image(label):
            ns=frames[label]['binding']['source_publication_header_ns']
            with np.load(session/(str(ns)+'_rgbd_tf.npz'),allow_pickle=False)as z:return z['image_bgr'].copy()
        background.append(background_pair(model,frames[a],frames[b],image(a),image(b),core['board_spec']))
    output=dict(step_rotations=steps,returns_relative_to_baseline=returns,background_regions=background,
      clock_semantics='This session camera source header is HOST_READ_COMPLETION,not calibrated sensor exposure. '
         'This precise session statement supersedes the historical core generic host-publication wording; '
         'q/source association remains conditional on uncalibrated exposure clocks.',
      rotation_invariant='For a fixed X,R_camera_relative=X^-1 R_tool_relative X; '
        'trace and relative rotation angle are unchanged. A fixed X cannot repair an angle-magnitude disagreement '
        'or represent two different persistent scene relations at the same actual q.',
      limitations=['No depth/TF selects LK points,homography,or RGB PnP branch.',
        'Board and mousemat co-motion alone cannot prove motion of the hard table.',
        'Elevated laptop/phone/box regions use pixel motion only; parallax is explicitly retained.',
        'Same feedback q is not an independent external encoder measurement of true rigid mechanical pose.',
        'RGB intrinsics/board/corner systematics,scene changes,camera mounting,base/support compliance and joint '
        'mechanics remain possible; no fitted angle/calibration is deployed.',
        'Fold variation is conditional sensitivity,not independent replicates or confidence intervals.'],
      analysis_runtime=dict(opencv_version=cv2.__version__,numpy_version=np.__version__),input_sha256_by_path=inputs)
    for path,digest in inputs.items():
        if sha(path)!=digest:raise ValueError('INPUT_CHANGED_DURING_ANALYSIS: '+path)
    target=folder/'background_return_review';target.mkdir(exist_ok=True)
    (target/'result.json').write_text(json.dumps(output,indent=2,allow_nan=False)+'\n')
    (target/'analysis_source.py').write_bytes(Path(__file__).read_bytes())
    (target/'sha256.json').write_text(json.dumps({p.name:sha(p)for p in [target/'result.json',target/'analysis_source.py']},indent=2)+'\n')
    print(json.dumps(dict(output=str(target/'result.json'),completed_steps=len(steps),background_pairs=len(background)),indent=2))


if __name__=='__main__':main()
