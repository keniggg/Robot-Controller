#!/usr/bin/env python3
"""Read-only physical validation of a frozen hand-eye transform; no calibration fit."""
import sys,pathlib,json,hashlib,math,os
from collections import Counter,defaultdict
from datetime import datetime,timezone
import numpy as np,rosbag
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'tools'));sys.path.insert(0,str(ROOT/'src/real-arm/alicia_d_calibration/scripts'))
from handeye_exact_sample_server import model_binding,quality_error,window_metrics,transform_matrix
from handeye_invariance_metrics import mean_transform
from handeye_sample_audit import error_summary,errors_against_reference
c=json.loads((ROOT/'.runtime/current_handeye_task_pose_validation.json').read_text());out=pathlib.Path(c['session']);start=json.loads((out/'starting_geometry.json').read_text());X=np.array(start['T_tool_camera_historical']);Yoriginal=np.array(start['T_base_board_initial_estimate']);prior=json.loads((pathlib.Path(c['training_session'])/'final_validation_20260927T110130Z/result.json').read_text());thresholds={k:v['limit'] for k,v in prior['threshold_checks'].items()};fp=prior['archive_training']['camera_model_fingerprint'];events=[json.loads(v) for v in (out/'automatic_motion.jsonl').read_text().splitlines()];issues=[];raw_frames=0;all_source_stamps=[]
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(4194304),b''):h.update(b)
 return h.hexdigest()
def check(ok,text):
 if not ok:issues.append(text)
def summary(matrices,reference):
 t,r=errors_against_reference(matrices,reference);return error_summary(t,r)
def audit_capture(folder,entry,board,epoch):
 global raw_frames
 path=folder/entry['directory'];d=json.loads((path/'capture.json').read_text());prefix=str(path.relative_to(out))+': ';records=d['samples'];rows={r['source_stamp_ns']:r for r in records};check(len(rows)==len(records)==entry['record_count'],prefix+'frame count');check(sha(path/'sources.bag')==d['bag_sha256']==entry['bag_sha256'],prefix+'bag SHA256')
 check(d['sample_role']=='validation',prefix+'validation role');check(d['model_fingerprint']==fp,prefix+'camera fingerprint')
 try:wm=window_metrics(records)
 except Exception as exc:issues.append(prefix+str(exc));wm={}
 for phase in ['before','after']:
  ev,binding=model_binding(d['camera_parameters_'+phase],'camera_link','/supervisor/camera/color/image_raw');check(binding==fp,prefix+'camera '+phase)
 for r in records:
  check(quality_error(r['quality'],fp,board) is None,prefix+'frame quality');check(r['quality']['source_stamp_ns']==r['source_stamp_ns'],prefix+'quality source stamp');check(r['control_reference_epoch_ns']==epoch,prefix+'epoch');check(abs(r['accepted_joint_time_offset_ns'])<=150000000,prefix+'joint time association')
 frames=defaultdict(Counter);counts=Counter()
 with rosbag.Bag(str(path/'sources.bag')) as bag:
  for topic,m,stamp in bag.read_messages():
   counts[topic]+=1
   if topic.startswith('/handeye_capture/camera_'):
    phase=topic.rsplit('_',1)[-1];check(json.loads(m.data)==d['camera_parameters_'+phase],prefix+'raw camera profile');continue
   ns=stamp.to_nsec();r=rows.get(ns);check(r is not None,prefix+'raw source stamp')
   if r is None:continue
   frames[ns][topic]+=1
   if topic.endswith('/image_raw'):check(m.header.stamp.to_nsec()==ns and m.header.frame_id=='camera_link' and (m.width,m.height)==(640,480) and len(m.data)==m.step*m.height,prefix+'RGBD pairing')
   elif topic=='/charuco/quality':check(json.loads(m.data)==r['quality'],prefix+'raw board quality')
   elif topic=='/tf':
    check(len(m.transforms)==1,prefix+'board TF count');b=m.transforms[0];check(b.header.stamp.to_nsec()==ns and b.header.frame_id=='camera_link' and b.child_frame_id==board and np.allclose(transform_matrix(b),r['T_camera_board'],rtol=0,atol=1e-10),prefix+'same-frame board TF')
   elif topic=='/handeye_capture/base_tool_at_source':check(abs(m.header.stamp.to_nsec()-ns)<=1 and m.header.frame_id=='base_link' and m.child_frame_id=='tool0' and np.allclose(transform_matrix(m),r['T_base_tool'],rtol=0,atol=1e-10),prefix+'same-frame robot TF')
   elif topic=='/alicia_d/accepted_joint_states':check(m.header.stamp.to_nsec()==r['accepted_joint_stamp_ns'] and list(m.name)==r['accepted_joint_names'] and np.allclose(m.position,r['accepted_joint_positions'],rtol=0,atol=1e-10),prefix+'raw measured joints')
   elif topic=='/alicia_d/control_reference_epoch':check(m.stamp.to_nsec()==epoch,prefix+'raw epoch')
 required=['/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw','/charuco/quality','/tf','/handeye_capture/base_tool_at_source','/alicia_d/accepted_joint_states','/alicia_d/control_reference_epoch'];check(set(frames)==set(rows) and all(counts[t]==len(records) and all(v[t]==1 for v in frames.values()) for t in required),prefix+'all raw channels paired');check(all(counts['/handeye_capture/camera_'+p]==1 for p in ['before','after']),prefix+'camera profiles count');check(d['selected_frame_index']==len(records)//2,prefix+'unselected representative')
 raw_frames+=len(records);all_source_stamps.extend(rows);r=records[d['selected_frame_index']];A=np.array(r['T_base_tool']);B=np.array(r['T_camera_board']);board_pose=A@X@B
 return dict(capture=str(path.relative_to(out)),accepted_as_novel=bool(d['accepted']),rejection=d['rejection'],source_stamp_ns=r['source_stamp_ns'],frames=len(records),epoch_ns=epoch,window_metrics=wm,q_deg=np.degrees(r['accepted_joint_positions'][:6]).tolist(),T_base_tool=A.tolist(),T_camera_board=B.tolist(),T_base_board=board_pose.tolist(),bag_sha256=entry['bag_sha256'],quality={k:r['quality'][k] for k in ['charuco_count','edge_clearance_px','reprojection_rms_px']})
segments=[];allrows=[]
for segment in c.get('sampling_segments',[dict(session=str(out))]):
 folder=pathlib.Path(segment['session']);m=json.loads((folder/'session.json').read_text());accepted_events={e['count']:e for e in events if e['event']=='validation_sample_accepted' and e.get('sample_session',str(out))==str(folder)};return_events={e['capture']:e for e in events if e['event']=='validation_return_window_recorded' and e['sample_session']==str(folder)};rows=[]
 for entry in m['captures']:
  if entry.get('active'):event=accepted_events.get(entry['gui_index_1based'],{})
  elif entry['directory'] in return_events and (entry.get('rejection') or '').startswith('与已有姿态差异不足'):event=return_events[entry['directory']]
  else:continue
  row=audit_capture(folder,entry,c['board_frame'],m['initial_epoch']);row['pose_id']=event.get('pose_id');row['role']=event.get('role','initial_reference' if not rows else 'bootstrap_validation');rows.append(row);print('AUDIT',row['capture'],row['frames'],'issues',len(issues),flush=True)
 if not rows:continue
 reference=np.array(rows[0]['T_base_board']);check(rows[0]['source_stamp_ns']==min(r['source_stamp_ns'] for r in rows),'reference must precede later independent samples')
 matrices=[np.array(r['T_base_board']) for r in rows];te,re=errors_against_reference(matrices,reference)
 for row,t,r in zip(rows,te,re):
  A=np.array(row['T_base_board']);row.update(translation_error_mm=float(t*1000),orientation_error_deg=float(r),delta_xyz_mm=((A[:3,3]-reference[:3,3])*1000).tolist(),cross_epoch_original_reference_mm=float(np.linalg.norm(A[:3,3]-Yoriginal[:3,3])*1000))
 novel=[np.array(r['T_base_board']) for r in rows[1:] if 'return' not in r['role']];returns=[np.array(r['T_base_board']) for r in rows if 'return' in r['role']];allstats=summary(matrices[1:],reference) if len(rows)>1 else None;novelstats=summary(novel,reference) if novel else None
 checks={k:dict(value=allstats[k],limit=v,pass_threshold=allstats[k]<=v) for k,v in thresholds.items()} if allstats else {}
 tools=[np.array(r['T_base_tool']) for r in rows];points=np.array([t[:3,3] for t in tools]);rotmax=max(float(np.degrees(np.arccos(np.clip((np.trace(a[:3,:3].T@b[:3,:3])-1)/2,-1,1)))) for a in tools for b in tools)
 result=dict(session=str(folder),epoch_ns=m['initial_epoch'],reference_capture=rows[0]['capture'],reference_frozen_before_future_observations=True,reference_refitted=False,all_nonreference_summary=allstats,novel_pose_summary=novelstats,return_summary_against_epoch_reference=summary(returns,reference) if returns else None,threshold_checks=checks,minimum_novel_groups_for_pass=6,observed_novel_groups=len(novel),within_epoch_verdict=('FAIL' if checks and not all(v['pass_threshold'] for v in checks.values()) else 'PASS' if checks and len(novel)>=6 and len(returns)>=1 else 'INSUFFICIENT'),coverage=dict(tcp_xyz_span_mm=(np.ptp(points,axis=0)*1000).tolist(),joint_span_deg=np.ptp(np.array([r['q_deg'] for r in rows]),axis=0).tolist(),max_pairwise_orientation_deg=rotmax),groups=rows)
 segments.append(result);allrows.extend(rows)
check(len(set(all_source_stamps))==len(all_source_stamps),'raw source frames reused between groups');check(sha(pathlib.Path(c['frozen_handeye']['calibration_file']))==c['frozen_yaml_sha256'],'deployed YAML changed');observed_pose_ids=sorted(set(e['pose_id'] for e in events if e['event'] in ('validation_sample_accepted','validation_return_window_recorded') and e.get('pose_id') is not None));substitutions=[e for e in events if e.get('goal_reached') is False];complete=set(range(1,32)).issubset(set(observed_pose_ids))
verdict='FAIL' if any(s['within_epoch_verdict']=='FAIL' for s in segments) else 'PASS' if complete and segments and not issues else 'INCOMPLETE'
if issues:verdict='EVIDENCE_INVALID'
report=dict(created_at=datetime.now(timezone.utc).isoformat(),verdict=verdict,sequence_complete=complete,sequence_complete_meaning='all 31 scheduled observation slots archived; motion substitution reported separately',nominal_motion_sequence_complete=not substitutions,observed_pose_ids=observed_pose_ids,geometry_limited_substitutions=substitutions,handeye_yaml=c['frozen_handeye']['calibration_file'],handeye_yaml_sha256=c['frozen_yaml_sha256'],handeye_transform_refitted=False,all_groups=len(allrows),raw_frames=raw_frames,thresholds=thresholds,archive_issues=issues,segments=segments,scope='Fixed-board independent invariance under measured task-like poses. Each driver epoch uses its first preexisting capture as fixed reference. Cross-epoch absolute differences are diagnostic only; no full-workspace or physical grip success claim.')
(out/'task_pose_validation_result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
print('RESULT',json.dumps({k:report[k] for k in ['verdict','sequence_complete','all_groups','raw_frames','archive_issues']},ensure_ascii=False),flush=True)
for s in segments:print('SEGMENT',s['session'],s['within_epoch_verdict'],json.dumps(s['all_nonreference_summary']),json.dumps(s['coverage']),flush=True)
