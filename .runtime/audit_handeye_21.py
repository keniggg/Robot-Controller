#!/usr/bin/env python3
"""Offline archive audit; never publishes transforms or robot commands."""
import sys,json,hashlib,math,os
from pathlib import Path
from datetime import datetime,timezone
from collections import Counter,defaultdict
import numpy as np
import cv2,rosbag
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/real-arm/alicia_d_calibration/scripts'))
sys.path.insert(0,str(ROOT/'tools'))
import handeye_sample_audit as audit
from handeye_invariance_metrics import mean_transform,quaternion_from_matrix,summarize_transforms
from handeye_exact_sample_server import model_binding,quality_error,window_metrics,transform_matrix,rotation_deg
OUT=Path(sys.argv[1]);PROTOCOL=json.loads((OUT/'protocol.json').read_text());SESSION=Path(PROTOCOL['session'])
assert os.path.ismount('/media/zhuyupei/SanStand')
manifest=json.loads((OUT/'session_snapshot.json').read_text())
entries=[e for e in manifest['captures'] if e.get('active')]
TH=PROTOCOL['thresholds']; failures=[]; groups=[];samples=[];means=[];images=[];all_stamps=[]
def sha(path):
 h=hashlib.sha256()
 with path.open('rb') as f:
  for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
 return h.hexdigest()
def check(ok,reason):
 if not ok:failures.append(reason)
def close(a,b):return bool(np.allclose(a,b,rtol=0,atol=1e-10))
def pass_metrics(m):return all(m[k]<=v for k,v in TH.items())
def summary_cv(ss,method,folds):
 te=[];re=[];by=[];candidates=[]
 for fold,test in enumerate(folds):
  test=[int(i) for i in test];train=[i for i in range(len(ss)) if i not in test]
  x=audit.compute_candidate(ss,method,train);candidates.append(x)
  ref=mean_transform(audit.board_transforms(ss,x,train))
  t,r=audit.errors_against_reference(audit.board_transforms(ss,x,test),ref)
  te.extend(t);re.extend(r)
  by.extend({'gui_index_1based':i+1,'fold':fold+1,'translation_error_mm':a*1000,'orientation_error_deg':b} for i,a,b in zip(test,t,r))
 m=audit.error_summary(te,re)
 return {'summary':m,'passes_thresholds':pass_metrics(m),'per_group':sorted(by,key=lambda r:r['gui_index_1based']),'candidate_spread':summarize_transforms(candidates)}
for n,e in enumerate(entries,1):
 name=e['directory'];d=json.loads((OUT/(name+'.json')).read_text());records=d['samples'];prefix=name+': '
 check(e['gui_index_1based']==n,prefix+'GUI index')
 check(sha(OUT/(name+'.json'))==PROTOCOL['capture_files'][n-1]['capture_json_sha256'],prefix+'snapshot digest')
 check((SESSION/name/'capture.json').read_bytes()==(OUT/(name+'.json')).read_bytes(),prefix+'original changed')
 bag_digest=sha(SESSION/name/'sources.bag');check(bag_digest==e['bag_sha256']==d['bag_sha256'],prefix+'bag digest')
 check(d['accepted'] and not d['rejection'] and e['accepted'],prefix+'acceptance')
 check(len(records)==e['record_count'],prefix+'frame count')
 check(d['selected_frame_index']==len(records)//2,prefix+'middle frame')
 check(records[d['selected_frame_index']]['source_stamp_ns']==e['source_stamp_ns'],prefix+'manifest selected stamp')
 for phase in ['before','after']:
  evidence,fingerprint=model_binding(d['camera_parameters_'+phase],'camera_link','/supervisor/camera/color/image_raw')
  check(fingerprint==d['model_fingerprint']==manifest['camera_model_fingerprint'],prefix+'model '+phase)
 check(d['camera_parameters_before']==d['camera_parameters_after'],prefix+'profile before after')
 try:
  metrics=window_metrics(records)
  check(all(abs(metrics[k]-d['metrics'][k])<1e-9 for k in metrics),prefix+'window metrics')
 except Exception as exc:failures.append(prefix+'window '+str(exc));metrics={}
 bystamp={r['source_stamp_ns']:r for r in records};all_stamps.extend(bystamp)
 for r in records:
  q=r['quality'];ns=r['source_stamp_ns']
  check(quality_error(q,manifest['camera_model_fingerprint'],manifest['board_frame']) is None,prefix+'quality')
  check(q['source_stamp_ns']==ns,prefix+'quality source stamp')
  check(q['camera_model']==evidence,prefix+'model evidence')
  check(q['backend_sdk_version']=='2.56.5',prefix+'SDK')
  check(r['control_reference_epoch_ns']==manifest['initial_epoch'],prefix+'epoch')
  check(r['accepted_joint_stamp_ns']-ns==r['accepted_joint_time_offset_ns'] and abs(r['accepted_joint_time_offset_ns'])<=150000000,prefix+'accepted feedback timing')
  for key in ['T_base_tool','T_camera_board']:
   t=np.asarray(r[key]);check(t.shape==(4,4) and np.isfinite(t).all() and close(t[3],[0,0,0,1]) and close(t[:3,:3].T@t[:3,:3],np.eye(3)) and abs(np.linalg.det(t[:3,:3])-1)<1e-9,prefix+'SE3 '+key)
 counts=Counter();found=defaultdict(Counter)
 with rosbag.Bag(str(SESSION/name/'sources.bag'),'r') as bag:
  for topic,msg,stamp in bag.read_messages():
   counts[topic]+=1
   if topic.startswith('/handeye_capture/camera_'):
    phase=topic.rsplit('_',1)[-1];check(json.loads(msg.data)==d['camera_parameters_'+phase],prefix+'bag profile');continue
   ns=stamp.to_nsec();r=bystamp.get(ns);check(r is not None,prefix+'bag unknown stamp')
   if r is None:continue
   found[ns][topic]+=1
   if topic.endswith('/image_raw'):
    check(msg.header.stamp.to_nsec()==ns and msg.header.frame_id=='camera_link' and (msg.width,msg.height)==(640,480),prefix+'bag image stamp/frame/dimensions')
    check(len(msg.data)==msg.step*msg.height,prefix+'image byte size')
    if topic=='/supervisor/camera/color/image_raw' and ns==e['source_stamp_ns']:
     check(msg.encoding in ('rgb8','bgr8'),prefix+'color encoding')
     im=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3).copy()
     if msg.encoding=='rgb8':im=cv2.cvtColor(im,cv2.COLOR_RGB2BGR)
     images.append((n,name,im));cv2.imwrite(str(OUT/(name+'_selected.jpg')),im)
   elif topic=='/charuco/quality':check(json.loads(msg.data)==r['quality'],prefix+'bag quality')
   elif topic=='/tf':
    check(len(msg.transforms)==1,prefix+'board TF count');t=msg.transforms[0]
    check(t.header.stamp.to_nsec()==ns and t.header.frame_id=='camera_link' and t.child_frame_id==manifest['board_frame'] and close(transform_matrix(t),r['T_camera_board']),prefix+'board TF')
   elif topic=='/handeye_capture/base_tool_at_source':
    check(msg.header.stamp.to_nsec()==ns and msg.header.frame_id=='base_link' and msg.child_frame_id=='tool0' and close(transform_matrix(msg),r['T_base_tool']),prefix+'exact-time robot TF')
   elif topic=='/alicia_d/accepted_joint_states':
    check(msg.header.stamp.to_nsec()==r['accepted_joint_stamp_ns'] and list(msg.name)==r['accepted_joint_names'] and close(msg.position,r['accepted_joint_positions']),prefix+'bag accepted joint')
   elif topic=='/alicia_d/control_reference_epoch':check(msg.stamp.to_nsec()==r['control_reference_epoch_ns'],prefix+'bag epoch')
   else:failures.append(prefix+'unexpected topic '+topic)
 required=['/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw','/charuco/quality','/tf','/handeye_capture/base_tool_at_source','/alicia_d/accepted_joint_states','/alicia_d/control_reference_epoch']
 check(set(found)==set(bystamp),prefix+'bag full source coverage')
 check(all(counts[t]==len(records) and all(v[t]==1 for v in found.values()) for t in required),prefix+'bag topic pairing')
 check(all(counts['/handeye_capture/camera_'+p]==1 for p in ['before','after']),prefix+'bag profiles count')
 selected=records[d['selected_frame_index']]
 samples.append({k:np.asarray(selected[k],float) for k in ['T_base_tool','T_camera_board']})
 means.append({k:mean_transform([np.asarray(r[k]) for r in records]) for k in ['T_base_tool','T_camera_board']})
 groups.append({'gui_index_1based':n,'capture':name,'sha256':bag_digest,'metrics':metrics,'frame_count':len(records),'quality':{k:{'min':min(r['quality'][k] for r in records),'max':max(r['quality'][k] for r in records)} for k in ['charuco_count','edge_clearance_px','reprojection_rms_px']},'max_joint_offset_ms':max(abs(r['accepted_joint_time_offset_ns'])/1e6 for r in records),'selected_joint_deg':np.degrees(selected['accepted_joint_positions'][:6]).tolist()})
 print('archive verified',n,name,'failures',len(failures),flush=True)
check(len(all_stamps)==len(set(all_stamps)), 'source frame reused between pose groups')
# Pose excitation diagnostics; no invented hard acceptance threshold for coverage.
poses=[s['T_base_tool'] for s in samples];p=np.array([t[:3,3] for t in poses]);rotvecs=[];trans_equations=[];pairs=[]
for i in range(len(poses)):
 for j in range(i):
  rel=np.linalg.inv(poses[j])@poses[i];v=cv2.Rodrigues(rel[:3,:3])[0].ravel();rotvecs.append(v);trans_equations.append(rel[:3,:3]-np.eye(3))
  dist=float(np.linalg.norm(p[i]-p[j]));angle=math.degrees(float(np.linalg.norm(v)));pairs.append((dist,angle))
  check(dist>=.005 or angle>=3,'duplicate accepted poses %d/%d'%(j+1,i+1))
sv=np.linalg.svd(np.vstack(trans_equations),compute_uv=False);rv=np.linalg.svd(np.asarray(rotvecs),compute_uv=False)
joints=np.array([g['selected_joint_deg'] for g in groups])
coverage={'base_tool_xyz_min_m':p.min(0).tolist(),'base_tool_xyz_max_m':p.max(0).tolist(),'base_tool_xyz_span_mm':(np.ptp(p,axis=0)*1000).tolist(),'pairwise_translation_max_mm':max(v[0] for v in pairs)*1000,'pairwise_rotation_max_deg':max(v[1] for v in pairs),'translation_system_singular_values':sv.tolist(),'translation_system_condition_number':float(sv[0]/sv[-1]),'rotation_vector_singular_values_rad':rv.tolist(),'joint_min_deg':joints.min(0).tolist(),'joint_max_deg':joints.max(0).tolist(),'joint_span_deg':np.ptp(joints,axis=0).tolist()}
# Use frozen groups only; never save or publish computed calibration through ROS.
results={};n=len(samples)
for name,method in audit.ALGORITHMS.items():
 try:
  r=audit.candidate_record(samples,method,TH)
  r['group_5fold']=summary_cv(samples,method,[list(range(k,n,5)) for k in range(5)])
  r['contiguous_5fold']=summary_cv(samples,method,np.array_split(np.arange(n),5))
  r['leave_one_group_out']=summary_cv(samples,method,[[k] for k in range(n)])
  r['group_mean_5fold']=summary_cv(means,method,[list(range(k,n,5)) for k in range(5)])
  results[name]=r
  print(name,json.dumps(r['cross_validation']['summary']),'PASS',r['cross_validation']['passes_thresholds'],flush=True)
 except Exception as ex:results[name]={'error':str(ex)};print(name,str(ex),flush=True)
old=audit.transform_from_translation_quaternion([-.08327671107922277,.004843742371779117,-.12907560357461276],[.005929097277682129,-.7167581240677906,.018061735450252286,.6970627024169479])
report={'created_at_utc':datetime.now(timezone.utc).isoformat(),'protocol':PROTOCOL,'archive_failures':failures,'archive_passed':not failures,'active_pose_groups':n,'unique_rgbd_source_frames':len(set(all_stamps)),'groups':groups,'coverage':coverage,'algorithms':results,'old_calibration_current_samples':summarize_transforms(audit.board_transforms(samples,old)),'independent_validation_done':False,'deployed':False,'session_manifest_unchanged':sha(SESSION/'session.json')==PROTOCOL['snapshot_sha256'],'audit_script_sha256':sha(Path(__file__))}
for name,r in results.items():
 if 'T_tool_camera' in r:
  t=np.asarray(r['T_tool_camera']);r['delta_from_old_calibration']={'translation_distance_mm':float(np.linalg.norm(t[:3,3]-old[:3,3])*1000),'rotation_deg':rotation_deg(t,old)}
(OUT/'audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
# Contact sheet preserves source pose order for visual inspection.
canvas=np.full((math.ceil(len(images)/4)*205,4*320,3),245,dtype=np.uint8)
for idx,(n,name,im) in enumerate(images):
 thumb=cv2.resize(im,(320,180));y=(idx//4)*205;x=(idx%4)*320;canvas[y:y+180,x:x+320]=thumb
 cv2.putText(canvas,'GUI %02d  %s'%(n,name),(x+7,y+197),cv2.FONT_HERSHEY_SIMPLEX,.45,(10,10,10),1,cv2.LINE_AA)
cv2.imwrite(str(OUT/'selected_frames.jpg'),canvas)
print('ARCHIVE',report['archive_passed'],'unique frames',report['unique_rgbd_source_frames'],'FAILURES',json.dumps(failures),flush=True)
print('COVERAGE',json.dumps(coverage),flush=True)
