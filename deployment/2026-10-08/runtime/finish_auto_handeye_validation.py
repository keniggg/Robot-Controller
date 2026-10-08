#!/usr/bin/env python3
import sys,json,os,hashlib,csv,math
from pathlib import Path
from datetime import datetime,timezone
from collections import Counter,defaultdict
import numpy as np,rosbag
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src/real-arm/alicia_d_calibration/scripts'));sys.path.insert(0,str(ROOT/'tools'))
import handeye_sample_audit as metrics
from handeye_invariance_metrics import summarize_transforms,mean_transform
from handeye_exact_sample_server import model_binding,quality_error,window_metrics,transform_matrix
c=json.loads((ROOT/'.runtime/current_handeye_independent_validation.json').read_text());train=Path(c['training_session']);validation=Path(c['session']);fitdir=Path(c['frozen_candidate_directory']);assert os.path.ismount('/media/zhuyupei/SanStand')
def sha(path):
 h=hashlib.sha256()
 with path.open('rb') as f:
  for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
 return h.hexdigest()
assert sha(fitdir/'results.json')==c['frozen_results_sha256'];fit=json.loads((fitdir/'results.json').read_text());X=np.array(fit['frozen_candidate']['T_tool_camera']);Y=np.array(fit['frozen_candidate']['T_base_board_training_reference']);th=fit['protocol']['thresholds']
failures=[]
def check(ok,reason):
 if not ok:failures.append(reason)
def close(a,b):return np.allclose(a,b,rtol=0,atol=1e-10)
def audit_archive(folder,role):
 m=json.loads((folder/'session.json').read_text());samples=[];sources=[];nsall=[];qstat=[];offsets=Counter();check(m['sample_role']==role,str(folder)+' role')
 for e in [v for v in m['captures'] if v.get('active')]:
  name=e['directory'];prefix=role+'/'+name+': ';path=folder/name;d=json.loads((path/'capture.json').read_text());rr={r['source_stamp_ns']:r for r in d['samples']};nsall.extend(rr)
  check(len(rr)==len(d['samples'])==e['record_count'],prefix+'unique count');check(d['accepted'] and d['sample_role']==role,prefix+'admission');check(sha(path/'sources.bag')==d['bag_sha256']==e['bag_sha256'],prefix+'bag digest')
  for phase in ['before','after']:
   evidence,fp=model_binding(d['camera_parameters_'+phase],'camera_link','/supervisor/camera/color/image_raw');check(fp==fit['model_fingerprint']==d['model_fingerprint'],prefix+'camera binding')
  try:window_metrics(d['samples'])
  except Exception as exc:failures.append(prefix+str(exc))
  for r in rr.values():
   q=r['quality'];check(quality_error(q,fit['model_fingerprint'],c['board_frame']) is None,prefix+'quality');check(q['source_stamp_ns']==r['source_stamp_ns'],prefix+'quality timestamp');check(q['camera_model']==evidence,prefix+'full model');check(r['control_reference_epoch_ns']==m['initial_epoch'],prefix+'epoch');check(abs(r['accepted_joint_time_offset_ns'])<=150000000,prefix+'joint time association');qstat.append(q)
  counts=Counter();frames=defaultdict(Counter)
  with rosbag.Bag(str(path/'sources.bag')) as bag:
   for topic,msg,t in bag.read_messages():
    counts[topic]+=1
    if topic.startswith('/handeye_capture/camera_'):
     phase=topic.rsplit('_',1)[-1];check(json.loads(msg.data)==d['camera_parameters_'+phase],prefix+'bag profile');continue
    ns=t.to_nsec();r=rr.get(ns);check(r is not None,prefix+'source stamp');
    if r is None:continue
    frames[ns][topic]+=1
    if topic.endswith('/image_raw'):check(msg.header.stamp.to_nsec()==ns and msg.header.frame_id=='camera_link' and (msg.width,msg.height)==(640,480) and len(msg.data)==msg.step*msg.height,prefix+'RGBD association')
    elif topic=='/charuco/quality':check(json.loads(msg.data)==r['quality'],prefix+'quality original')
    elif topic=='/tf':
     check(len(msg.transforms)==1,prefix+'board TF count');bt=msg.transforms[0];check(bt.header.stamp.to_nsec()==ns and bt.header.frame_id=='camera_link' and bt.child_frame_id==c['board_frame'] and close(transform_matrix(bt),r['T_camera_board']),prefix+'board TF')
    elif topic=='/handeye_capture/base_tool_at_source':
     delta=msg.header.stamp.to_nsec()-ns;offsets[delta]+=1;check(abs(delta)<=1 and msg.header.frame_id=='base_link' and msg.child_frame_id=='tool0' and close(transform_matrix(msg),r['T_base_tool']),prefix+'robot TF')
    elif topic=='/alicia_d/accepted_joint_states':check(msg.header.stamp.to_nsec()==r['accepted_joint_stamp_ns'] and list(msg.name)==r['accepted_joint_names'] and close(msg.position,r['accepted_joint_positions']),prefix+'accepted joints')
    elif topic=='/alicia_d/control_reference_epoch':check(msg.stamp.to_nsec()==r['control_reference_epoch_ns'],prefix+'epoch original')
  required=['/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw','/charuco/quality','/tf','/handeye_capture/base_tool_at_source','/alicia_d/accepted_joint_states','/alicia_d/control_reference_epoch']
  check(set(frames)==set(rr) and all(counts[t]==len(rr) and all(v[t]==1 for v in frames.values()) for t in required),prefix+'all raw source frames paired')
  check(all(counts['/handeye_capture/camera_'+phase]==1 for phase in ['before','after']),prefix+'profiles count')
  r=d['samples'][d['selected_frame_index']];check(d['selected_frame_index']==len(rr)//2 and r['source_stamp_ns']==e['source_stamp_ns'],prefix+'representative selection')
  samples.append({k:np.array(r[k]) for k in ['T_base_tool','T_camera_board']});sources.append({'gui_index_1based':e['gui_index_1based'],'capture':name,'capture_json_sha256':sha(path/'capture.json'),'bag_sha256':e['bag_sha256'],'source_stamp_ns':r['source_stamp_ns']})
  print('AUDIT',role,len(samples),name,'issues',len(failures),flush=True)
 check(len(nsall)==len(set(nsall)),role+' repeated RGBD sources')
 return {'session':str(folder),'groups':len(samples),'source_frames':len(nsall),'source_stamps':nsall,'sources':sources,'robot_tf_header_offset_ns':dict(offsets),'camera_model_fingerprint':m['camera_model_fingerprint'],'epoch':m['initial_epoch'],'quality':{k:{'min':min(q[k] for q in qstat),'max':max(q[k] for q in qstat)} for k in ['charuco_count','edge_clearance_px','reprojection_rms_px']}},samples
archive_train,training=audit_archive(train,'training');archive_val,heldout=audit_archive(validation,'validation')
check(archive_train['epoch']==archive_val['epoch'],'training/validation epoch changed');check(not(set(archive_train['source_stamps'])&set(archive_val['source_stamps'])),'validation frame reused from training')
# The raw training representatives must still be the frozen inputs.
check(len(training)==len(fit['inputs']),'training count changed')
for r,pin in zip(archive_train['sources'],fit['inputs']):check(r['capture']==pin['capture'] and r['capture_json_sha256']==pin['capture_sha256'],'training freeze changed')
events=[json.loads(line) for line in (validation/'automatic_motion.jsonl').read_text().splitlines()]
accepted={e['count']:e['pose_id'] for e in events if e['event']=='pose_sample_accepted'};plan=json.loads((train/'prospective_independent_validation_plan.json').read_text());roles={p['pose_id']:p['role'] for p in plan['poses']}
check(any(e['event']=='collection_sequence_complete' for e in events),'validation sequence incomplete')
boards=metrics.board_transforms(heldout,X);te,re=metrics.errors_against_reference(boards,Y);per=[]
for i,(t,r) in enumerate(zip(te,re),1):
 pose_id=accepted.get(i);check(pose_id in roles,'validation pose/source mapping');per.append({'index_1based':i,'pose_id':pose_id,'role':roles.get(pose_id,'unknown'),'translation_error_mm':t*1000,'orientation_error_deg':r})
novel=[i for i,p in enumerate(per) if p['role']=='validation'];returns=[i for i,p in enumerate(per) if p['role']=='return'];check(len(novel)>=plan['minimum_novel_validation_groups'],'fewer than 6 independent groups');check(len(returns)==1,'return pose absent')
summary=metrics.error_summary(te,re);novelsummary=metrics.error_summary([te[i] for i in novel],[re[i] for i in novel]) if novel else {};returnsummary=metrics.error_summary([te[i] for i in returns],[re[i] for i in returns]) if returns else {}
checks={k:{'value':summary[k],'limit':lim,'pass':summary[k]<=lim} for k,lim in th.items()};passed=not failures and all(v['pass'] for v in checks.values()) and fit['algorithms'][fit['selected_algorithm']]['cross_validation']['passes_thresholds']
poses=[s['T_base_tool'] for s in training];points=np.array([t[:3,3] for t in poses]);relative=[np.linalg.inv(poses[j])@poses[i] for i in range(len(poses)) for j in range(i)];sv=np.linalg.svd(np.vstack([t[:3,:3]-np.eye(3) for t in relative]),compute_uv=False)
coverage={'training_xyz_span_mm':(np.ptp(points,axis=0)*1000).tolist(),'max_pairwise_orientation_deg':max(float(np.degrees(np.arccos(np.clip((np.trace(t[:3,:3])-1)/2,-1,1)))) for t in relative),'translation_system_singular_values':sv.tolist(),'translation_condition_number':float(sv[0]/sv[-1])}
result={'created_at':datetime.now(timezone.utc).isoformat(),'verdict':'PASS' if passed else 'FAIL','training_algorithm':fit['selected_algorithm'],'training_cross_validation':fit['algorithms'][fit['selected_algorithm']]['cross_validation']['summary'],'independent_validation':novelsummary,'return_repeatability':returnsummary,'all_validation_summary':summary,'threshold_checks':checks,'per_validation_group':per,'archive_training':archive_train,'archive_validation':archive_val,'archive_failures':failures,'coverage':coverage,'frozen_candidate':fit['frozen_candidate'],'candidate_sha256':c['frozen_results_sha256'],'candidate_refitted_with_validation':False,'deployed':False,'scope':'Observed calibration region and orientations only; no external ground-truth or full-workspace absolute accuracy claim.'}
folder=train/('final_validation_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'));folder.mkdir();(folder/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n');(ROOT/'.runtime/current_handeye_auto_final.json').write_text(json.dumps({'directory':str(folder),'verdict':result['verdict']},indent=2)+'\n')
with (folder/'validation_groups.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=list(per[0]));w.writeheader();w.writerows(per)
print('FINAL',folder,json.dumps({k:result[k] for k in ['verdict','training_cross_validation','independent_validation','return_repeatability','all_validation_summary','coverage','archive_failures']},ensure_ascii=False),flush=True)
