"""Offline RGB-only ChArUco pose against saved q/TF and independent depth.

No ROS imports/connections,device access,calibration update,or robot commands.
The saved RGB image,physical board specification and SDK RGB rays alone define
the RGB pose. TF/depth never select a PnP solution or calibrate its scale.
"""
import argparse
import ast
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np

WORK=Path(__file__).resolve().parents[2]
TRACKER=WORK/'src/real-arm/alicia_d_calibration/scripts/charuco_tracker.py'
GUARD=WORK/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observation_path_guard.py'
DEPTH_ANALYSIS=WORK/'.runtime/analyze_depth_supported_board_motion.py'
OPTICAL_TO_ROS=np.eye(4)
OPTICAL_TO_ROS[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
INPUT_HASHES={}


def tracked(path):
 path=Path(path).resolve();digest=hashlib.sha256(path.read_bytes()).hexdigest()
 if str(path)in INPUT_HASHES and INPUT_HASHES[str(path)]!=digest:
  raise ValueError('INPUT_CHANGED_DURING_ANALYSIS: '+str(path))
 INPUT_HASHES[str(path)]=digest
 return path


def extract_pure(path,names):
 """Reuse named pure maths without executing the enclosing ROS node module."""
 tree=ast.parse(tracked(path).read_text());selected=[n for n in tree.body
     if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name in names]
 if {n.name for n in selected}!=set(names):raise ValueError('PURE_DEPENDENCY_NOT_FOUND')
 namespace=dict(np=np,cv2=cv2,json=json,hashlib=hashlib,deepcopy=deepcopy,
                math=math,ET=ET,ObservationPathError=ValueError)
 exec(compile(ast.Module(body=selected,type_ignores=[]),str(path),'exec'),namespace)
 return namespace


def runtime_model(parameters,sdk=None):
 pure=extract_pure(TRACKER,['runtime_color_model_evidence','RuntimeColorModel'])
 evidence=pure['runtime_color_model_evidence'](parameters,'camera_link',
                         '/supervisor/camera/color/image_raw')
 return pure['RuntimeColorModel'](evidence,sdk)


def board_from_spec(spec):
 required=('board_size','square_length_m','marker_length_m','dictionary_id','legacy_pattern','detection_scale')
 if any(k not in spec for k in required):raise ValueError('BOARD_SPEC_INCOMPLETE')
 size=spec['board_size'];square=float(spec['square_length_m']);marker=float(spec['marker_length_m']);scale=float(spec['detection_scale'])
 if len(size)!=2 or any(type(x)is not int or x<3 for x in size):raise ValueError('BOARD_SIZE_INVALID')
 if not np.isfinite([square,marker,scale]).all() or not 0<marker<square or not 1<=scale<=4:
  raise ValueError('BOARD_LENGTH_OR_DETECTION_SCALE_INVALID')
 if type(spec['legacy_pattern'])is not bool:raise ValueError('BOARD_PATTERN_MUST_BE_EXPLICIT')
 ar=cv2.aruco
 if not isinstance(spec['dictionary_id'],str) or not hasattr(ar,spec['dictionary_id']):raise ValueError('BOARD_DICTIONARY_INVALID')
 dictionary=ar.getPredefinedDictionary(getattr(ar,spec['dictionary_id']))
 board=(ar.CharucoBoard(tuple(size),square,marker,dictionary) if hasattr(ar,'CharucoBoard')
        else ar.CharucoBoard_create(*size,square,marker,dictionary))
 if hasattr(board,'setLegacyPattern'):board.setLegacyPattern(spec['legacy_pattern'])
 elif not spec['legacy_pattern']:raise ValueError('NONLEGACY_PATTERN_UNSUPPORTED_BY_OLD_OPENCV')
 return board,dictionary


def detect_rgb(image,spec):
 board,dictionary=board_from_spec(spec);ar=cv2.aruco;scale=float(spec['detection_scale'])
 gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
 if scale!=1:gray=cv2.resize(gray,None,fx=scale,fy=scale,interpolation=cv2.INTER_CUBIC)
 params=ar.DetectorParameters() if hasattr(ar,'DetectorParameters') else ar.DetectorParameters_create()
 params.cornerRefinementMethod=ar.CORNER_REFINE_NONE
 if hasattr(ar,'ArucoDetector'):corners,ids,_=ar.ArucoDetector(dictionary,params).detectMarkers(gray)
 else:corners,ids,_=ar.detectMarkers(gray,dictionary,parameters=params)
 if ids is None:raise ValueError('NO_ARUCO_MARKERS')
 if hasattr(ar,'CharucoDetector'):
  pixels,cids,_,_=ar.CharucoDetector(board,detectorParams=params).detectBoard(gray,None,None,corners,ids)
 else:_,pixels,cids=ar.interpolateCornersCharuco(corners,ids,gray,board)
 if cids is None or len(cids)<6:raise ValueError('TOO_FEW_CHARUCO_CORNERS')
 indices=np.asarray(cids).ravel().astype(int)
 all_points=np.asarray(board.getChessboardCorners() if hasattr(board,'getChessboardCorners') else board.chessboardCorners,dtype=float)
 if len(set(indices))!=len(indices) or np.any(indices<0) or np.any(indices>=len(all_points)):
  raise ValueError('CHARUCO_IDS_INVALID')
 return dict(ids=indices,pixels=pixels.reshape(-1,2).astype(float)/scale,
             object_points=all_points[indices],marker_count=len(ids),
             unit_board_xyz=all_points[indices]/float(spec['square_length_m']))


def rotation_angle_deg(R):
 return math.degrees(math.acos(float(np.clip((np.trace(R)-1)/2,-1,1))))


def mappts(T,P):return np.asarray(P)@T[:3,:3].T+T[:3,3]


def error_stats(values,scale=1):
 values=np.asarray(values,float)*scale
 if not len(values):return {'n':0}
 return dict(n=len(values),rms=float(np.sqrt(np.mean(values**2))),
             median=float(np.median(values)),p05=float(np.percentile(values,5)),
             p95=float(np.percentile(values,95)),maximum=float(values.max()))


def solve_rgb(model,points,pixels):
 """PnP selection uses ONLY these RGB observations; no TF/depth arguments."""
 points=np.ascontiguousarray(points,dtype=float).reshape(-1,1,3)
 pixels=np.asarray(pixels,float).reshape(-1,2)
 if len(points)<6 or len(points)!=len(pixels) or not np.isfinite(points).all():raise ValueError('RGB_PNP_POINTS_INVALID')
 if np.linalg.matrix_rank(points[:,0,:2]-points[:,0,:2].mean(0))<2:raise ValueError('RGB_BOARD_CORNERS_COLLINEAR')
 normalized=model.normalized_points(pixels);rvec,tvec,_=model.solve(points,pixels)
 trials=[('runtime_SQPNP_or_EPNP_LM',rvec,tvec)]
 if hasattr(cv2,'SOLVEPNP_IPPE'):
  # OpenCV4.2 requires a floating output buffer for this optional argument.
  answer=cv2.solvePnPGeneric(points,normalized,np.eye(3),None,flags=cv2.SOLVEPNP_IPPE,
                            reprojectionError=np.empty((2,1),dtype=np.float64))
  if answer[0]:
   trials.extend(('IPPE_raw_%d'%i,r,t) for i,(r,t) in enumerate(zip(answer[1],answer[2])))
 candidates=[]
 for label,r,t in trials:
  try:predicted=model.project(points,r,t)
  except ValueError:continue
  R=cv2.Rodrigues(np.asarray(r,float))[0];T=np.eye(4);T[:3,:3]=R;T[:3,3]=np.asarray(t).ravel()
  residual=np.linalg.norm(predicted-pixels,axis=1)
  candidates.append(dict(method=label,T_optical_board=T.tolist(),
                         RGB_reprojection_px=error_stats(residual)))
 if not candidates:raise ValueError('RGB_NO_POSITIVE_DEPTH_PNP_SOLUTION')
 candidates.sort(key=lambda c:c['RGB_reprojection_px']['rms']);best=candidates[0]
 T=np.asarray(best['T_optical_board'])
 for c in candidates:
  other=np.asarray(c['T_optical_board'])
  c['rotation_from_selected_deg']=rotation_angle_deg(other[:3,:3]@T[:3,:3].T)
  c['origin_distance_from_selected_mm']=float(np.linalg.norm(other[:3,3]-T[:3,3])*1000)
 return dict(selected_by='minimum original RGB-pixel reprojection only; no TF/depth input',
             T_optical_board=best['T_optical_board'],RGB_reprojection_px=best['RGB_reprojection_px'],
             candidates=candidates)


def board_spatial_folds(unit_points):
 cells=(np.asarray(unit_points)[:,:2]//2).astype(int);unique,counts=np.unique(cells,axis=0,return_counts=True)
 choices={};loads=[0,0,0]
 for i in sorted(range(len(unique)),key=lambda j:(-int(counts[j]),tuple(unique[j]))):
  k=int(np.argmin(loads));choices[tuple(unique[i])]=k;loads[k]+=int(counts[i])
 return np.array([choices[tuple(c)]for c in cells])


def rgb_review(model,detection):
 P,U=detection['object_points'],detection['pixels'];review=solve_rgb(model,P,U)
 folds=board_spatial_folds(detection['unit_board_xyz']);validation=[]
 for fold in range(3):
  train=np.flatnonzero(folds!=fold);test=np.flatnonzero(folds==fold)
  if len(train)<10 or len(test)<6:raise ValueError('RGB_SPATIAL_HOLDOUT_INSUFFICIENT')
  pose=solve_rgb(model,P[train],U[train]);T=np.array(pose['T_optical_board']);r=cv2.Rodrigues(T[:3,:3])[0]
  error=np.linalg.norm(model.project(P[test],r,T[:3,3])-U[test],axis=1)
  validation.append(dict(fold=fold,train_ids=detection['ids'][train].tolist(),
       test_ids=detection['ids'][test].tolist(),T_optical_board=T.tolist(),
       heldout_RGB_reprojection_px=error_stats(error)))
 review.update(charuco_count=len(P),marker_count=detection['marker_count'],
       ids=detection['ids'].tolist(),pixels=U.tolist(),object_points_m=P.tolist(),
       unit_board_xyz=detection['unit_board_xyz'].tolist(),spatial_folds=validation,
       quality_35corners_1_5px=bool(len(P)>=35 and review['RGB_reprojection_px']['rms']<=1.5),
       nominal_grid_span=np.ptp(detection['unit_board_xyz'],axis=0).tolist(),
       uncertainty='Spatial fold variation and distinct IPPE branches are conditional '
         'diagnostics,not confidence intervals covering RGB intrinsics,board '
         'printing,corner correlations or physical mounting changes.')
 return review


def patch_depth(depth,pixels):
 values=[];quality=[];valid_count=[]
 for x,y in pixels:
  ix,iy=int(round(x)),int(round(y));x0,y0=max(ix-2,0),max(iy-2,0)
  patch=depth[y0:iy+3,x0:ix+3];py,px=np.indices(patch.shape);good=np.isfinite(patch)&(patch>.03)&(patch<1.)
  valid_count.append(int(good.sum()))
  if good.sum()<18:values.append(None);quality.append(None);continue
  A=np.column_stack([px[good]+x0-x,py[good]+y0-y,np.ones(good.sum())]);co=np.linalg.lstsq(A,1/patch[good],rcond=None)[0]
  e=1/(A@co)-patch[good]
  if co[2]<=0 or np.percentile(abs(e),90)>.002:values.append(None);quality.append(float(np.sqrt(np.mean(e*e))*1000));continue
  values.append(float(1/co[2]));quality.append(float(np.sqrt(np.mean(e*e))*1000))
 return dict(optical_z_m=values,within_patch_RMSE_mm=quality,valid_depth_pixel_count=valid_count,
             selection='5x5 same source-pixel inverse-depth fit; >=18 measured valid '
              'pixels and residualP90<=2mm. Missing/invalid depth is never filled.')


def depth_review(model,detection,depth,rgb):
 """Same pixels and interior depth; no change to the independent RGB pose."""
 depth=np.asarray(depth,float);pixels=detection['pixels'];T=np.array(rgb['T_optical_board'])
 rays=np.c_[model.normalized_points(pixels).reshape(-1,2),np.ones(len(pixels))]
 normal=T[:3,2];expected_z=float(normal@T[:3,3])/(rays@normal)
 patch=patch_depth(depth,pixels);measured=patch['optical_z_m'];good=np.array([x is not None for x in measured])
 errors=np.array([measured[i]-expected_z[i]for i in np.flatnonzero(good)])
 result=dict(same_pixel_depth=patch,same_pixel_ids=detection['ids'].tolist(),
             RGB_plane_expected_optical_z_m=expected_z.tolist(),
             depth_minus_RGB_plane_z_mm=error_stats(errors,1000),
             missing_corner_depth_count=int((~good).sum()))
 mask=np.zeros(depth.shape,np.uint8);hull=cv2.convexHull(pixels.astype(np.float32))
 cv2.fillConvexPoly(mask,np.rint(hull).astype(np.int32),1);mask=cv2.erode(mask,np.ones((9,9),np.uint8))
 yy,xx=np.where(mask>0);keep=(xx%2==0)&(yy%2==0);xx,yy=xx[keep],yy[keep];z=depth[yy,xx]
 valid=np.isfinite(z)&(z>.03)&(z<1.);uv=np.c_[xx[valid],yy[valid]];z=z[valid]
 if len(z)<500:raise ValueError('DEPTH_BOARD_INTERIOR_INSUFFICIENT')
 depth_rays=np.c_[model.normalized_points(uv).reshape(-1,2),np.ones(len(z))];P=depth_rays*z[:,None]
 train=((uv[:,0]//16+uv[:,1]//16)%2)==0
 if min(train.sum(),(~train).sum())<100:raise ValueError('DEPTH_SPATIAL_HOLDOUT_INSUFFICIENT')
 functions=extract_pure(DEPTH_ANALYSIS,['fit_plane','rigid','grid_spacing'])
 n,d,plane=functions['fit_plane'](P[train]);validation=P[~train]@n+d
 plane.update(heldout_all_point_residual_mm=error_stats(validation,1000),
              valid_depth_fraction=float(valid.mean()),interior_sampled_pixels=len(valid))
 if np.any(abs(rays@n)<.1):raise ValueError('DEPTH_BOARD_RAYS_NEAR_PARALLEL')
 corner_points=rays*(-d/(rays@n))[:,None]
 metric=functions['grid_spacing'](corner_points,detection['unit_board_xyz'])
 metric.pop('delta_from_assumed_15mm');metric['configured_square_mm']=float(np.linalg.norm(detection['object_points'][0])/np.linalg.norm(detection['unit_board_xyz'][0])*1000)
 metric['interpretation']='Grid spacing derived from depth and unit IDs; never adjusts RGB pose scale.'
 result.update(independent_interior_depth_plane=plane,depth_supported_grid=metric,
               depth_plane_vs_RGB_normal_deg=math.degrees(math.acos(float(np.clip(abs(n@normal),-1,1)))),
               depth_supported_corners_optical_m=corner_points.tolist())
 return result


def validate_binding(event,frame):
 ns=int(frame['source_stamp_ns'])
 if ns<=0 or ns!=event['source_stamp_ns']:raise ValueError('SOURCE_HEADER_BINDING_MISMATCH')
 lo,hi=event['q_source_before'],event['q_source_after']
 if not lo[0]<=ns<=hi[0]:raise ValueError('Q_DOES_NOT_BRACKET_SOURCE_HEADER')
 if len(lo[1])<6 or len(hi[1])<6 or not np.isfinite(np.r_[lo[1],hi[1]]).all():raise ValueError('Q_FEEDBACK_INVALID')
 span=np.asarray(event['span_deg'],float)
 if span.shape!=(6,) or not np.isfinite(span).all():raise ValueError('STATIONARY_SPAN_INVALID')
 return dict(source_publication_header_ns=ns,diagnostic_snapshot_receipt_ros_ns=event.get('time_ns'),
       snapshot_receipt_age_ms=(event['time_ns']-ns)/1e6 if 'time_ns'in event else None,
       q_bracket_before=lo,q_bracket_after=hi,stationary_span_deg=span.tolist(),
       bracket_separations_ms=[(ns-lo[0])/1e6,(hi[0]-ns)/1e6],
       q_nearest_bracket_100ms_pass=bool(min(ns-lo[0],hi[0]-ns)<=100_000_000),
       q_bracket_gap_250ms_pass=bool(hi[0]-lo[0]<=250_000_000),
       recorded_stationary_span_0_15deg_pass=bool(np.max(span)<=.15),
       clock_limitation='Header is host publication time,not calibrated exposure. '
         'Snapshot receipt includes analysis/TF lookup delay,not SDK latency. '
         'Use separately recorded SDK/source metadata to audit true acquisition.')


def compare_frames(first,second):
 B0,B1=np.array(first['rgb']['T_optical_board']),np.array(second['rgb']['T_optical_board'])
 C0,C1=np.array(first['T_base_optical']),np.array(second['T_base_optical'])
 visual=B0@np.linalg.inv(B1);predicted=np.linalg.inv(C0)@C1
 result=dict(first_label=first['label'],second_label=second['label'],
      first_source_ns=first['binding']['source_publication_header_ns'],
      second_source_ns=second['binding']['source_publication_header_ns'],
      T_camera0_camera1_RGB_only=visual.tolist(),T_camera0_camera1_recorded_TF=predicted.tolist(),
      RGB_TF_rotation_disagreement_deg=rotation_angle_deg(visual[:3,:3]@predicted[:3,:3].T),
      camera_motion_translation_difference_mm=((visual[:3,3]-predicted[:3,3])*1000).tolist(),
      q_before_bracket_delta_deg=np.degrees(np.array(second['binding']['q_bracket_before'][1][:6])-first['binding']['q_bracket_before'][1][:6]).tolist())
 points=np.array(first['rgb']['object_points_m']);difference=mappts(C1@B1,points)-mappts(C0@B0,points)
 result['fixed_board_base_point_drift_mm']=error_stats(np.linalg.norm(difference,axis=1),1000)
 result['fixed_board_base_centroid_shift_xyz_mm']=(difference.mean(0)*1000).tolist()
 if 'spatial_folds'in first['rgb'] and 'spatial_folds'in second['rgb']:
  # All 3 x 3 independently fitted tile folds remain visible; these are
  # conditional corner sensitivity checks,not repeated physical experiments.
  variants=[]
  for a in first['rgb']['spatial_folds']:
   A=np.asarray(a['T_optical_board'])
   for b in second['rgb']['spatial_folds']:
    B=np.asarray(b['T_optical_board']);motion=A@np.linalg.inv(B)
    delta=mappts(C1@B,points)-mappts(C0@A,points)
    variants.append(dict(first_fold=a['fold'],second_fold=b['fold'],
       relative_rotation_from_full_RGB_deg=rotation_angle_deg(motion[:3,:3]@visual[:3,:3].T),
       RGB_TF_rotation_disagreement_deg=rotation_angle_deg(motion[:3,:3]@predicted[:3,:3].T),
       camera_motion_translation_from_full_RGB_norm_mm=float(np.linalg.norm(motion[:3,3]-visual[:3,3])*1000),
       fixed_board_base_point_drift_RMS_mm=float(np.sqrt(np.mean(np.sum(delta*delta,axis=1)))*1000)))
  result['RGB_spatial_fold_pair_sensitivity']=dict(variants=variants,
      interpretation='Nine combinations of spatially withheld RGB corner tiles '
       'measure conditional PnP sensitivity only; they share pixels,camera '
       'intrinsics and board scale,are not independent samples or confidence bounds.')
 if ('depth' in first and 'depth' in second
     and 'depth_supported_corners_optical_m'in first['depth']
     and 'depth_supported_corners_optical_m'in second['depth']):
  a={i:p for i,p in zip(first['rgb']['ids'],first['depth']['depth_supported_corners_optical_m'])}
  b={i:p for i,p in zip(second['rgb']['ids'],second['depth']['depth_supported_corners_optical_m'])}
  ids=sorted(set(a)&set(b));pure=extract_pure(DEPTH_ANALYSIS,['rigid'])
  try:
   mapping,fit=pure['rigid']([a[i]for i in ids],[b[i]for i in ids]);motion=np.linalg.inv(mapping)
   result['depth_supported_motion']=dict(common_ids=ids,T_camera0_camera1=motion.tolist(),fit=fit,
         RGB_rotation_difference_deg=rotation_angle_deg(motion[:3,:3]@visual[:3,:3].T),
         RGB_translation_difference_xyz_mm=((motion[:3,3]-visual[:3,3])*1000).tolist())
  except Exception as e:result['depth_supported_motion_error']=repr(e)
 return result


def frame_selection(frame):
 reasons=[]
 if not frame['rgb']['quality_35corners_1_5px']:reasons.append('RGB_35corners_1_5px')
 for key in ('q_nearest_bracket_100ms_pass','q_bracket_gap_250ms_pass',
             'recorded_stationary_span_0_15deg_pass'):
  if not frame['binding'][key]:reasons.append(key)
 return dict(accepted=not reasons,rejected_requirements=reasons,
             interpretation='Depth diagnostics never select or reject independent RGB pose.')


def main():
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--session',type=Path,required=True)
 parser.add_argument('--board-spec',type=Path,required=True);parser.add_argument('--model-snapshot',type=Path)
 args=parser.parse_args();session=args.session.resolve();mount=Path('/media/zhuyupei/SanStand')
 if not mount.is_mount() or mount not in session.parents:raise ValueError('MOUNTED_EXTERNAL_SESSION_REQUIRED')
 cv2.setNumThreads(1);tracked(Path(__file__));spec=json.loads(tracked(args.board_spec).read_text())
 parameters=json.loads(tracked(session/'camera_parameters.json').read_text());model=runtime_model(parameters)
 events=[json.loads(line)for line in tracked(session/'events.jsonl').read_text().splitlines()]
 snapshots=[e for e in events if e.get('event')=='stationary_snapshot']
 if not snapshots:raise ValueError('NO_SAVED_STATIONARY_SNAPSHOTS')
 fk=X=None
 if args.model_snapshot:
  frozen=json.loads(tracked(args.model_snapshot).read_text());X=np.asarray(frozen['T_tool_camera'])
  xml=tracked(session/'robot_description.urdf').read_text()
  if hashlib.sha256(xml.encode()).hexdigest()!=frozen['urdf_sha256']:raise ValueError('FROZEN_URDF_MISMATCH')
  pure=extract_pure(GUARD,['_array','_axis_rotation','SerialUrdfFk']);fk=pure['SerialUrdfFk'](xml,['Joint%d'%i for i in range(1,7)])
 report=dict(method='Independent RGB-only board pose; savedTF/q and measured depth are diagnostic comparisons only.',
     analysis_runtime=dict(opencv_version=cv2.__version__,numpy_version=np.__version__),
     board_spec=spec,camera_model=model.evidence,camera_model_fingerprint=model.fingerprint,
     frames=[],pairs=[],errors=[],limitations=[
      'RGB translation depends on configured physical square scale; RGB rotation does not. Never calibrate this scale from TF or depth.',
      'Fixed hard board and camera/board specifications must remain unchanged across the selected sequence.',
      'Spatial-fold PnP and planar alternative solutions show conditional uncertainty,not all systematic errors.',
      'Host-publication header association is not calibrated exposure association. q/FK comparisons remain conditional on source clocks.',
      'Depth plane fits/intersections are independent comparisons and can retain depth biases; low fit residual is not absolute truth.',
      'No new calibration/TF,robot parameters,thresholds or commands are written.'])
 for event in snapshots:
  row=dict(label=event['label'])
  try:
   path=Path(event['file']).resolve()
   if path.parent!=session:raise ValueError('FRAME_ESCAPES_SESSION')
   with np.load(tracked(path),allow_pickle=False)as z:frame={k:z[k].copy()for k in z.files}
   row['binding']=validate_binding(event,frame);image=frame['image_bgr'];color=model.evidence['color_intrinsics']
   if image.shape!=(color['height'],color['width'],3):raise ValueError('RGB_MODEL_DIMENSIONS_MISMATCH')
   expected_K=np.array([[color['fx'],0,color['cx']],[0,color['fy'],color['cy']],[0,0,1.]])
   if not np.array_equal(frame['K'],expected_K):raise ValueError('RGB_SAVED_INTRINSICS_MISMATCH')
   detection=detect_rgb(image,spec);row['rgb']=rgb_review(model,detection)
   row['T_base_optical']=frame['T_base_optical'].tolist()
   if not np.allclose(frame['T_base_ros_camera']@OPTICAL_TO_ROS,frame['T_base_optical'],atol=1e-10,rtol=0):raise ValueError('ROS_OPTICAL_TF_DISAGREEMENT')
   if fk is not None:
    row['independent_frozen_FK_vs_recorded_ros_camera_max_matrix_errors']=[float(np.max(abs(fk(np.asarray(q[:6]))@X-frame['T_base_ros_camera'])))for _,q in (event['q_source_before'],event['q_source_after'])]
   try:
    profile=parameters['runtime_profile']
    if (profile['geometry_intrinsics']!=profile['color_intrinsics'] or not parameters['align_depth_to_color']):raise ValueError('DEPTH_ALIGNMENT_MODEL_UNSUPPORTED')
    if frame['depth_raw'].dtype!=np.uint16 or frame['depth_raw'].shape!=image.shape[:2]:raise ValueError('SOURCE_Z16_DIMENSIONS_INVALID')
    if float(frame['depth_scale'])!=float(parameters['depth_scale']):raise ValueError('DEPTH_SCALE_BINDING_MISMATCH')
    row['depth']=depth_review(model,detection,frame['depth_raw']*float(frame['depth_scale']),row['rgb'])
   except Exception as e:row['depth_error']=repr(e)
   report['frames'].append(row)
  except Exception as e:report['errors'].append(dict(label=event['label'],error=repr(e)))
 for row in report['frames']:row['independent_RGB_comparison_selection']=frame_selection(row)
 usable=[r for r in report['frames']if r['independent_RGB_comparison_selection']['accepted']]
 if usable:
  for row in usable[1:]:
   try:report['pairs'].append(compare_frames(usable[0],row))
   except Exception as e:report['errors'].append(dict(pair=[usable[0]['label'],row['label']],error=repr(e)))
 report['analysis_complete']=bool(len(usable)>=2 and not report['errors'])
 for path in list(INPUT_HASHES):tracked(path)
 report['input_sha256_by_path']=dict(sorted(INPUT_HASHES.items()))
 output=session/'rgb_charuco_coordinate_chain';output.mkdir(exist_ok=True)
 (output/'result.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
 (output/'analysis_source.py').write_bytes(Path(__file__).read_bytes())
 (output/'sha256.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest()for p in (output/'result.json',output/'analysis_source.py')},indent=2)+'\n')
 print(json.dumps(dict(output=str(output/'result.json'),accepted_RGB_frames=len(usable),pairs=len(report['pairs']),errors=report['errors']),indent=2),flush=True)
 if not report['analysis_complete']:raise RuntimeError('INDEPENDENT_RGB_ANALYSIS_INCOMPLETE')


if __name__=='__main__':main()
