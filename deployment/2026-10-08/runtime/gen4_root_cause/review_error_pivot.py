"""Offline conditional joint-axis error models with spatial held-out pixels.

 A fitted virtual angle is a diagnostic hypothesis, NOT a calibrated encoder
 correction or evidence of a unique failed component. No ROS connection.
 """
import argparse
import hashlib
import json
import math
from pathlib import Path
import cv2
import numpy as np


def axis_transform(axis,pivot,angle):
 axis=np.asarray(axis,float);pivot=np.asarray(pivot,float)
 if axis.shape!=(3,) or pivot.shape!=(3,) or not np.isfinite(np.r_[axis,pivot,angle]).all() or np.linalg.norm(axis)<1e-12:
  raise ValueError('invalid axis/pivot/angle')
 R=cv2.Rodrigues(axis/np.linalg.norm(axis)*angle)[0];E=np.eye(4);E[:3,:3]=R;E[:3,3]=(np.eye(3)-R)@pivot
 return E


def mappts(T,P):
 return np.asarray(P)@T[:3,:3].T+T[:3,3]


def fit_angle(F0,F,axis,pivot,P,Q,train):
 train=np.asarray(train,int)
 if len(train)<10:raise ValueError('insufficient independent training points')
 def loss(delta):
  V=np.linalg.inv(axis_transform(axis,pivot,delta)@F)@F0
  e=mappts(V,P[train])-Q[train]
  # Smooth robust loss fixed at2mm. Held-out points never choose delta.
  return float(np.mean(np.sqrt(np.sum(e*e,axis=1)+.002**2)))
 lo,hi=math.radians(-5),math.radians(5);ratio=(math.sqrt(5)-1)/2
 a=hi-ratio*(hi-lo);b=lo+ratio*(hi-lo);la=loss(a);lb=loss(b)
 for _ in range(48):
  if la<=lb:hi=b;b=a;lb=la;a=hi-ratio*(hi-lo);la=loss(a)
  else:lo=a;a=b;la=lb;b=lo+ratio*(hi-lo);lb=loss(b)
 return (lo+hi)/2


def spatial_folds(uv):
 cells=(np.asarray(uv)//70).astype(int);unique,counts=np.unique(cells,axis=0,return_counts=True)
 assignments={};loads=[0,0,0]
 for i in sorted(range(len(unique)),key=lambda i:(-int(counts[i]),tuple(unique[i]))):
  k=int(np.argmin(loads));assignments[tuple(unique[i])]=k;loads[k]+=int(counts[i])
 return np.array([assignments[tuple(cell)] for cell in cells])


def stats(e):
 e=np.asarray(e,float)
 return dict(n=len(e),rmse_mm=float(np.sqrt(np.mean(e**2))*1000),median_mm=float(np.median(e)*1000),p90_mm=float(np.percentile(e,90)*1000))


def review(session,subdir):
 import sys
 sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
 from gen4_single_axis_plan import SerialUrdfFk,NAMES
 from alicia_flexible_grasp.robot.observation_path_guard import _axis_rotation
 path=session/subdir/'fixed_local_support_report.json';report=json.loads(path.read_text())
 events=[json.loads(s) for s in (session/'events.jsonl').read_text().splitlines()]
 frames={e['source_stamp_ns']:Path(e['file']) for e in events if e['event']=='stationary_snapshot'}
 source=json.loads((session/'fresh_plan/snapshot.json').read_text());xml=(session/'robot_description.urdf').read_text()
 if hashlib.sha256(xml.encode()).hexdigest()!=source['urdf_sha256']:raise RuntimeError('recorded URDF hash differs from frozen source contract')
 fk=SerialUrdfFk(xml,NAMES[:6])
 baseline=report['baseline']['publication_header_stamp_ns'];F0=np.load(frames[baseline])['T_base_optical'];rows=[]
 for row in report['results']:
  if 'error' in row:raise RuntimeError('upstream incomplete correspondence report')
  ns=row['destination_ns'];model=row['projection_model'];binding=row['frame_binding'];q=np.array(binding['bracketing_actual_q_rad'][0][:6])
  F=np.load(frames[ns])['T_base_optical'];V=np.array(row['T_destination_from_source_rgbd'])
  corrpath=session/subdir/('%s_%s_%s_spatialcv_correspondences.npz'%(baseline,ns,model))
  with np.load(corrpath) as z:P=z['P_source'];Q=z['Q_destination'];uv=z['uv0']
  folds=spatial_folds(uv);T=np.eye(4);axes=[]
  for origin,index,axis in fk.chain:
   T=T@origin
   if index is not None:
    axes.append((index,T[:3,:3]@axis,T[:3,3].copy()))
    T[:3,:3]=T[:3,:3]@_axis_rotation(axis,q[index])
  E=F0@np.linalg.inv(V)@np.linalg.inv(F);rot=cv2.Rodrigues(E[:3,:3])[0].ravel();candidates=[]
  for index,axis,pivot in axes:
   estimates=[];errors=[];controls=[]
   for fold in range(3):
    train=np.flatnonzero(folds!=fold);test=np.flatnonzero(folds==fold)
    if len(test)<5 or len(train)<10:raise ValueError('insufficient spatial holdout')
    delta=fit_angle(F0,F,axis,pivot,P,Q,train);Vp=np.linalg.inv(axis_transform(axis,pivot,delta)@F)@F0
    residual=np.linalg.norm(mappts(Vp,P[test])-Q[test],axis=1);unchanged=np.linalg.norm(mappts(np.linalg.inv(F)@F0,P[test])-Q[test],axis=1)
    estimates.append(dict(fold=fold,virtual_angle_deg=math.degrees(delta),heldout=stats(residual),tf_heldout=stats(unchanged)))
    errors.extend(residual);controls.extend(unchanged)
   candidates.append(dict(joint=index+1,pivot_base_m=pivot.tolist(),axis_base=axis.tolist(),
                          folds=estimates,pooled_heldout=stats(errors),unchanged_tf=stats(controls),
                          mean_virtual_angle_deg=float(np.mean([f['virtual_angle_deg'] for f in estimates]))))
  rows.append(dict(label=row['label'],model=model,actual_q_deg=np.degrees(q).tolist(),
                   visual_rotation_error_base_deg=np.degrees(rot).tolist(),
                   visual_camera_origin_shift_mm=(((F0@np.linalg.inv(V))[:3,3]-F[:3,3])*1000).tolist(),
                   candidates=candidates,correspondences_sha256=hashlib.sha256(corrpath.read_bytes()).hexdigest()))
 return dict(session=str(session),source_report=str(path),source_report_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
             source_model_urdf_sha256=source['urdf_sha256'],results=rows)


def main():
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--root',type=Path,required=True);args=parser.parse_args();root=args.root.resolve()
 if not Path('/media/zhuyupei/SanStand').is_mount() or root.parent.name!='full_ros_latest_start_20261002':raise RuntimeError('mounted canonical evidence directory required')
 out=root/'joint_pivot_review';out.mkdir(exist_ok=True)
 result=dict(method='Each known joint axis/pivot is tested with one bounded virtual angle per pose, fitted using2/3 of spatial tiles; remaining tiles only evaluate. No free camera translation/pivot fitted in this comparison.',
             limitations=['Per-pose fitted angles are not a predictive calibration and must never be deployed as encoder/TF corrections.',
                          'Relative camera observations cannot uniquely separate base/robot flexibility, transmission error, extrinsic flexibility, or combinations.',
                          'Nearly parallel joint axes require translation/lever-arm evidence; rotation alone cannot identify J2/J3/J5.',
                          'ROS camera header is host publication time, not independently calibrated exposure time.',
                          'Conditional on unchanged physical support texture and correct independent RGB matches.'],sessions=[])
 for name,sub in [('high_posture_20261003T081809Z','support_plane_review'),('high_posture_20261003T091023Z','support_plane_after_visual_gate')]:
  session=review(root/name,sub);result['sessions'].append(session)
  rows=[r for r in session['results'] if r['model']=='sdk_inverse_brown' and np.linalg.norm(r['visual_rotation_error_base_deg'])>=1]
  medians={i:float(np.median([next(c['pooled_heldout']['rmse_mm'] for c in r['candidates'] if c['joint']==i) for r in rows])) for i in range(1,7)}
  print(json.dumps(dict(session=name,large_error_rows=len(rows),heldout_rmse_median_mm_by_virtual_joint=medians)),flush=True)
 (out/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');(out/'analysis_source.py').write_bytes(Path(__file__).read_bytes())
 (out/'sha256.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in out.iterdir() if p.is_file() and p.name!='sha256.json'},indent=2)+'\n')


if __name__=='__main__':main()
