#!/usr/bin/env python3
"""Offline sensitivity audit in SDK ray coordinates; never changes runtime K/X."""
import sys, json, hashlib, importlib.util
from pathlib import Path
import numpy as np
import cv2, rosbag
from cv_bridge import CvBridge
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('depthaudit',ROOT/'.runtime/analyze_depth_supported_board_motion.py')
D=importlib.util.module_from_spec(spec);spec.loader.exec_module(D)
STORE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log')
OUT=Path(json.loads((ROOT/'.runtime/current_grasp_repair_resume.json').read_text())['directory'])/'camera_intrinsics_sensitivity'
assert STORE.is_dir() and Path('/media/zhuyupei/SanStand').is_mount()
OUT.mkdir(exist_ok=True)
K0=np.array([[400.,0,320.],[0,400.,240.],[0,0,1.]])
O=D.O

def detect_sample(path,role):
 a=json.loads(path.read_text())
 if not a['accepted'] or not a['samples']:return None
 s=a['samples'][a['selected_frame_index']];ns=s['source_stamp_ns']
 with rosbag.Bag(str(path.parent/'sources.bag')) as bag:
  image=next(CvBridge().imgmsg_to_cv2(m,'bgr8') for t,m,_ in bag.read_messages(topics=['/supervisor/camera/color/image_raw']) if m.header.stamp.to_nsec()==ns)
 detected=D.detect(image,scale=1.5,square=.015)
 model=D.CM.RuntimeColorModel(s['quality']['camera_model'])
 rays=model.normalized_points(detected['pixels']).reshape(-1,2)
 obj=np.float32(detected['unit_board_xyz']*.015)
 pixels=np.float32(rays*400+np.array([320,240]))
 return dict(capture=str(path),capture_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),role=role,
   source_stamp_ns=ns,epoch=s['control_reference_epoch_ns'],model=model.fingerprint,
   dataset=str(path.parent.parent),object_points=obj.tolist(),virtual_pixels=pixels.tolist(),
   T_base_tool=s['T_base_tool'],original_T_camera_board=s['T_camera_board'],corners=len(obj))

def fit(rows):
 flags=cv2.CALIB_USE_INTRINSIC_GUESS|cv2.CALIB_ZERO_TANGENT_DIST
 for key in ('CALIB_FIX_K1','CALIB_FIX_K2','CALIB_FIX_K3','CALIB_FIX_K4','CALIB_FIX_K5','CALIB_FIX_K6'):flags|=getattr(cv2,key)
 rms,k,d,*_=cv2.calibrateCamera([np.float32(r['object_points']) for r in rows],[np.float32(r['virtual_pixels']) for r in rows],(640,480),K0.copy(),np.zeros(5),flags=flags,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_MAX_ITER,100,1e-10))
 return rms,k

def pose(row,k):
 obj=np.float64(row['object_points']);pix=np.float64(row['virtual_pixels'])
 ok,r,t=cv2.solvePnP(obj,pix,k,np.zeros(5),flags=cv2.SOLVEPNP_SQPNP)
 if not ok:raise ValueError('pnp failed')
 r,t=cv2.solvePnPRefineLM(obj,pix,k,np.zeros(5),r,t)
 pred=cv2.projectPoints(obj,r,t,k,np.zeros(5))[0].reshape(-1,2)
 T=np.eye(4);T[:3,:3]=cv2.Rodrigues(r)[0];T[:3,3]=t.reshape(3)
 return O@T,float(np.sqrt(np.mean(np.sum((pred-pix)**2,axis=1))))

def angle(r):return float(np.degrees(np.arccos(np.clip((np.trace(r)-1)*.5,-1,1))))

def evaluate(rows,k):
 out=[]
 for r in rows:
  base,br=pose(r,K0);new,nr=pose(r,k)
  out.append(dict(capture=r['capture'],role=r['role'],dataset=r['dataset'],epoch=r['epoch'],T_base_tool=r['T_base_tool'],
     original_T_camera_board=r['original_T_camera_board'],baseline_T_camera_board=base.tolist(),candidate_T_camera_board=new.tolist(),
     baseline_virtual_rms_px=br,candidate_virtual_rms_px=nr,
     candidate_translation_change_mm=float(np.linalg.norm(new[:3,3]-base[:3,3])*1000),candidate_rotation_change_deg=angle(base[:3,:3].T@new[:3,:3])))
 return out

def main():
 trainbase=STORE/'handeye_auto_20260927T102700Z'
 groups=[('training',sorted(trainbase.glob('capture_*/capture.json'))),('independent_validation',sorted(trainbase.glob('independent_validation_*/capture_*/capture.json'))),('task_pose_validation',sorted((STORE/'handeye_task_pose_validation_20260928T045735Z').glob('epoch_*/capture_*/capture.json')))]
 cached=OUT/'detected_rays.json'
 if cached.exists():rows=json.loads(cached.read_text())
 else:
  rows=[]
  for role,paths in groups:
   for path in paths:
    row=detect_sample(path,role)
    if row:rows.append(row);print('detected',role,path.parent.name,row['corners'],flush=True)
  cached.write_text(json.dumps(rows,indent=2))
 train=[r for r in rows if r['role']=='training']
 rms,k=fit(train);result=evaluate(rows,k)
 folds=[]
 for fold in range(4):
  tr=[r for i,r in enumerate(train) if i%4!=fold];test=[r for i,r in enumerate(train) if i%4==fold]
  fr,fk=fit(tr);folds.append(dict(fold=fold,train_rms=fr,virtual_K=fk.tolist(),holdout=evaluate(test,fk)))
 pairs=[]
 for i,a in enumerate(result):
  for b in result[:i]:
   if a['dataset']!=b['dataset'] or a['epoch']!=b['epoch']:continue
   A=np.linalg.inv(np.array(b['T_base_tool']))@np.array(a['T_base_tool']);fa=angle(A[:3,:3])
   if fa<2:continue
   dif={}
   for kind in ('baseline','candidate'):
    C=np.array(b[kind+'_T_camera_board'])@np.linalg.inv(np.array(a[kind+'_T_camera_board']))
    dif[kind+'_camera_minus_fk_deg']=angle(C[:3,:3])-fa
   pairs.append(dict(first=b['capture'],second=a['capture'],role=a['role'],fk_angle_deg=fa,**dif))
 report=dict(diagnostic_only=True,runtime_changed=False,method='SDK rays mapped to virtual K400; fit four pinhole parameters with zero additional distortion; object metric 15mm only sets pose scale. Grouped holdout; never deploy from fit residual.',counts={g:sum(r['role']==g for r in rows) for g,_ in groups},virtual_K_baseline=K0.tolist(),virtual_K_candidate=k.tolist(),training_virtual_rms_px=rms,frames=result,folds=folds,angle_pairs=pairs)
 (OUT/'report.json').write_text(json.dumps(report,indent=2));(OUT/Path(__file__).name).write_text(Path(__file__).read_text())
 print(json.dumps({key:report[key] for key in ('counts','virtual_K_candidate','training_virtual_rms_px')},indent=2))
 for g,_ in groups:
  part=[r for r in result if r['role']==g];p=[r for r in pairs if r['role']==g]
  print(g,'RMS median base/candidate',*[np.median([r[k+'_virtual_rms_px'] for r in part]) for k in ('baseline','candidate')],'angle abs median/max',*[np.percentile([abs(r[k+'_camera_minus_fk_deg']) for r in p],[50,100]).tolist() if p else None for k in ('baseline','candidate')])
 print('fold_K', [f['virtual_K'] for f in folds]);print('output',str(OUT),flush=True)
if __name__=='__main__':main()
