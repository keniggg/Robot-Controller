"""Offline full metric X/internal-zero identifiability; no ROS/control I/O.

Training uses exactly reserved 22 captures. Validation groups remain held out.
Intrinsics, ray observations, URDF lengths, board scale and q1/q6 gauges fixed.
"""
import argparse
from collections import defaultdict
import hashlib,json,math,os
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
import yaml

ROOT=Path(__file__).resolve().parents[1]
NAMES=['X_tx_mm','X_ty_mm','X_tz_mm','X_rx_deg','X_ry_deg','X_rz_deg','q2_zero_deg','q3_zero_deg','q4_zero_deg','q5_zero_deg']


def matrix(rot):
 return rot.as_matrix() if hasattr(rot,'as_matrix') else rot.as_dcm()


def from_matrix(m):
 return Rotation.from_matrix(m) if hasattr(Rotation,'from_matrix') else Rotation.from_dcm(m)


def rotation_mean(rot):
 # Chordal SO(3) mean: principal eigenvector of quaternion outer products.
 q=rot.as_quat();_,v=np.linalg.eigh(q.T@q)
 return Rotation.from_quat(v[:,-1])


def rotation(axis,a):
 x,y,z=axis;k=np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
 return np.eye(3)+math.sin(a)*k+(1-math.cos(a))*(k@k)


class MetricFk:
 def __init__(self,xml):
  self.sha=hashlib.sha256(xml.encode()).hexdigest();robot=ET.fromstring(xml)
  bychild={j.find('child').get('link'):j for j in robot.findall('joint')}
  chain=[];child='tool0'
  while child!='base_link':j=bychild[child];chain.append(j);child=j.find('parent').get('link')
  self.chain=[]
  for j in reversed(chain):
   if j.get('type') not in ('revolute','fixed'):raise ValueError('unsupported metric chain')
   o=j.find('origin');xyz=np.fromstring(o.get('xyz','0 0 0'),sep=' ');rpy=np.fromstring(o.get('rpy','0 0 0'),sep=' ')
   T=np.eye(4);T[:3,3]=xyz;T[:3,:3]=rotation([0,0,1],rpy[2])@rotation([0,1,0],rpy[1])@rotation([1,0,0],rpy[0])
   index=None;axis=None
   if j.get('type')=='revolute':index=int(j.get('name').replace('Joint',''))-1;axis=np.fromstring(j.find('axis').get('xyz'),sep=' ');axis/=np.linalg.norm(axis)
   self.chain.append((T,index,axis))
 def __call__(self,q):
  T=np.eye(4)
  for origin,index,axis in self.chain:
   T=T@origin
   if index is not None:T[:3,:3]=T[:3,:3]@rotation(axis,q[index])
  return T


def load(directory):
 detected=json.loads((directory/'detected_rays.json').read_text());report=json.loads((directory/'report.json').read_text())
 bycapture={r['capture']:r for r in report['frames']};models={};rows=[];sources=[]
 for row in detected:
  path=Path(row['capture']);data=path.read_bytes()
  if hashlib.sha256(data).hexdigest()!=row['capture_sha256']:raise ValueError('capture hash mismatch')
  c=json.loads(data);sample=next(s for s in c['samples'] if s['source_stamp_ns']==row['source_stamp_ns'])
  qmap=dict(zip(sample['accepted_joint_names'],sample['accepted_joint_positions']));q=np.array([qmap['Joint%d'%i] for i in range(1,7)])
  urdf=Path(row['dataset'])/'robot_description.urdf'
  if not urdf.is_file():urdf=Path(row['dataset']).parent/'robot_description.urdf'
  text=urdf.read_text();sha=hashlib.sha256(text.encode()).hexdigest()
  if sha not in models:models[sha]=MetricFk(text)
  if sample['control_reference_epoch_ns']!=row['epoch']:raise ValueError('epoch binding mismatch')
  frame=bycapture[str(path)]
  rows.append(dict(q=q,C=np.asarray(frame['baseline_T_camera_board']),A=np.asarray(row['T_base_tool']),fk=models[sha],role=row['role'],group=(row['dataset'],row['epoch']),capture=str(path)))
  sources.append(dict(capture=str(path),capture_sha256=row['capture_sha256'],source_stamp_ns=row['source_stamp_ns'],role=row['role'],group=list(rows[-1]['group']),urdf=str(urdf),urdf_sha256=sha))
 if {role:sum(r['role']==role for r in rows) for role in set(r['role'] for r in rows)} != {'training':22,'independent_validation':8,'task_pose_validation':25}:raise ValueError('exact reserved 22 train / 8 independent / 25 task captures required')
 return rows,sources


def handeye(path):
 raw=path.read_bytes();t=yaml.safe_load(raw)['transformation'];X=np.eye(4)
 X[:3,:3]=matrix(Rotation.from_quat([t[k] for k in ('qx','qy','qz','qw')]));X[:3,3]=[t[k] for k in ('x','y','z')]
 return X,hashlib.sha256(raw).hexdigest()


def predict(rows,X0,z):
 X=X0.copy();X[:3,3]+=np.asarray(z[:3])*.001;X[:3,:3]=matrix(Rotation.from_rotvec(np.radians(z[3:6])))@X0[:3,:3]
 offsets=np.zeros(6);offsets[1:5]=np.radians(z[6:10])
 return [r['fk'](r['q']+offsets)@X@r['C'] for r in rows]


def centered_residual(rows,X0,z,orientation_unit_deg=.1):
 predicted=predict(rows,X0,z);groups=defaultdict(list)
 for i,r in enumerate(rows):groups[r['group']].append(i)
 error=np.zeros((len(rows),6))
 for indices in groups.values():
  T=np.asarray([predicted[i] for i in indices]);mean_t=T[:,:3,3].mean(axis=0)
  R=from_matrix(T[:,:3,:3]);mean_R=rotation_mean(R)
  for i,t,r in zip(indices,T,R):
   error[i,:3]=(t[:3,3]-mean_t)/.001
   error[i,3:]=(mean_R.inv()*r).as_rotvec()/math.radians(orientation_unit_deg)
 return error.ravel()


def central_jacobian(fun,z,step):
 eye=np.eye(len(z))*step
 return np.column_stack([(fun(z+e)-fun(z-e))/(2*step) for e in eye])


def audit(rows,X0):
 train=[r for r in rows if r['role']=='training'];out={};z=np.zeros(10)
 for angular_unit in (.1,.25,1.):
  fun=lambda z:centered_residual(train,X0,z,angular_unit)
  entries=[]
  for step in (.001,.003):
   J=central_jacobian(fun,z,step);u,s,vt=np.linalg.svd(J,full_matrices=False)
   threshold=max(float(s[0])*1e-7,1e-6)
   rank=int(np.sum(s>threshold));condition=float(s[0]/s[-1]) if s[-1]>threshold else None
   entries.append(dict(central_step_parameter_units=step,rank_at_relative_tolerance_1e7=rank,absolute_scaled_numerical_floor=1e-6,effective_rank_threshold=threshold,singular_values=s.tolist(),condition=condition,weakest_direction=dict(zip(NAMES,vt[-1].tolist())),scaled_column_norms=dict(zip(NAMES,np.linalg.norm(J,axis=0).tolist())),noise_unit_parameter_sensitivity=np.sqrt(np.diag(np.linalg.pinv(J.T@J,rcond=1e-14))).tolist()))
  out[str(angular_unit)]=dict(position_residual_unit_mm=1.,orientation_residual_unit_deg=angular_unit,jacobian_audits=entries)
 return out



def metrics(rows,X0,z):
 predicted=predict(rows,X0,z);groups=defaultdict(list)
 for i,r in enumerate(rows):groups[r['group']].append(i)
 result=[]
 for group,indices in groups.items():
  T=np.asarray([predicted[i] for i in indices]);tm=T[:,:3,3].mean(axis=0)
  R=from_matrix(T[:,:3,:3]);rm=rotation_mean(R)
  d=np.linalg.norm(T[:,:3,3]-tm,axis=1)*1000
  a=np.array([np.linalg.norm((rm.inv()*r).as_rotvec())*180/math.pi for r in R])
  # Pairwise spread avoids interpreting any fitted group origin as world truth.
  dp=[];ap=[]
  for i in range(len(T)):
   for j in range(i):
    dp.append(np.linalg.norm(T[i,:3,3]-T[j,:3,3])*1000)
    ap.append(np.linalg.norm(from_matrix(T[i,:3,:3].T@T[j,:3,:3]).as_rotvec())*180/math.pi)
  result.append(dict(group=list(group),role=rows[indices[0]]['role'],count=len(indices),informative=len(indices)>1,
   position_centered_rms_mm=float(np.sqrt(np.mean(d*d))),position_centered_max_mm=float(d.max()),
   rotation_centered_rms_deg=float(np.sqrt(np.mean(a*a))),rotation_centered_max_deg=float(a.max()),
   position_pairwise_max_mm=max(dp) if dp else None,rotation_pairwise_max_deg=max(ap) if ap else None,
   captures=[rows[i]['capture'] for i in indices],centered_position_mm=d.tolist(),centered_rotation_deg=a.tolist()))
 return result


def fit_training(train,X0,angular_unit=.1):
 if any(r['role']!='training' for r in train):raise ValueError('validation samples cannot enter estimator')
 # Bounds declared before viewing any validation fit result. No robust loss or
 # validation-informed parameter/weight selection; zero is the sole start.
 bound=np.array([10.,10.,10.,3.,3.,3.,10.,10.,10.,10.])
 fun=lambda z:centered_residual(train,X0,z,angular_unit)
 fit=least_squares(fun,np.zeros(10),bounds=(-bound,bound),jac='3-point',max_nfev=150,xtol=1e-10,ftol=1e-10,gtol=1e-10)
 return dict(orientation_unit_deg=angular_unit,parameter_bounds_abs=dict(zip(NAMES,bound.tolist())),
  parameters=dict(zip(NAMES,fit.x.tolist())),z=fit.x.tolist(),success=bool(fit.success),message=fit.message,
  function_evaluations=int(fit.nfev),train_residual_sum_squares=float(fit.fun@fit.fun),
  baseline_train_residual_sum_squares=float(fun(np.zeros(10))@fun(np.zeros(10))),
  parameters_within_1pct_of_bound=[name for name,value,limit in zip(NAMES,fit.x,bound) if abs(value)>=.99*limit],
  final_jacobian_singular_values=np.linalg.svd(fit.jac,compute_uv=False).tolist())

def main():
 parser=argparse.ArgumentParser();parser.add_argument('directory');parser.add_argument('--fit',action='store_true');parser.add_argument('--handeye',default='/home/zhuyupei/.ros/easy_handeye/d405_cc200_auto_20260927_validated_eye_on_hand.yaml');args=parser.parse_args()
 source=Path(args.directory);mount=Path('/media/zhuyupei/SanStand')
 if not os.path.ismount(mount):raise RuntimeError('SanStand absent; no system-disk fallback')
 out=source.parent/'joint_metric_identifiability';out.mkdir(exist_ok=True)
 rows,sources=load(source);X,sha=handeye(Path(args.handeye))
 match=[(np.linalg.norm(r['fk'](r['q'])[:3,3]-r['A'][:3,3])*1000,np.linalg.norm(from_matrix(r['fk'](r['q'])[:3,:3].T@r['A'][:3,:3]).as_rotvec())*180/math.pi) for r in rows]
 if np.max(match,axis=0)[0]>.01 or np.max(match,axis=0)[1]>.001:raise RuntimeError('metric FK differs from archived measured transform')
 report=dict(diagnostic_only=True,runtime_changed=False,fit_performed=False,parameters=NAMES,training_count=22,validation_counts={'independent':8,'task':25},handeye_sha256=sha,camera_model='unchanged SDK-ray baseline PnP; candidate K unused',length_scale_fitted=False,q1_q6_zero_offsets_fixed=True,FK_archive_max_translation_mm=float(np.max(match,axis=0)[0]),FK_archive_max_rotation_deg=float(np.max(match,axis=0)[1]),training_joint_range_deg=np.degrees(np.ptp(np.array([r['q'] for r in rows if r['role']=='training']),axis=0)).tolist(),rank_audit=audit(rows,X),sources=sources)
 main=report['rank_audit']['0.1']['jacobian_audits'];report['numerically_identifiable']=all(r['rank_at_relative_tolerance_1e7']==10 and r['condition']<1e6 for r in main)
 (out/'rank_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');(out/'audit_source.py').write_bytes(Path(__file__).read_bytes())
 if args.fit:
  if not report['numerically_identifiable']:raise RuntimeError('rank audit rejected fitting')
  train=[r for r in rows if r['role']=='training']
  baseline=metrics(rows,X,np.zeros(10));fits=[]
  for unit in (.1,.25,1.):
   fitted=fit_training(train,X,unit);fitted['evaluation']=metrics(rows,X,np.array(fitted['z']));fits.append(fitted)
  fitting=dict(diagnostic_only=True,runtime_changed=False,intrinsics_changed=False,length_or_board_scale_changed=False,
   note='All estimators see only 22 training samples. Validation group centers are nuisance origins, not refitted robot/camera parameters. Singleton groups are uninformative. Main weight is 0.1deg; others are sensitivity only.',
   rank_report='rank_report.json',source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
   baseline=baseline,fits=fits)
  (out/'fit_report.json').write_text(json.dumps(fitting,ensure_ascii=False,indent=2)+'\n')
  print(json.dumps(dict(fit_report=str(out/'fit_report.json'),fits=[{k:v for k,v in f.items() if k!='evaluation'} for f in fits]),ensure_ascii=False))
 else:
  print(json.dumps({k:v for k,v in report.items() if k not in ('sources',)},ensure_ascii=False))

if __name__=='__main__':main()
