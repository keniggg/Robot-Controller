"""Seal supplemental operator evidence; never alter the original task result."""
from pathlib import Path
from datetime import datetime,timezone
import json,hashlib,shutil
import rosbag
m=Path('/media/zhuyupei/SanStand');assert m.is_mount();e=m/'实验记录/Robot-Controller';w=Path(__file__).resolve().parents[1]
r=e/'grasp_records/20261006T061242.642572Z_f8b97892d0a84d5599287c74677f32d2';out=e/'ros_log/grasp_close_hold_20261006T061242Z'
manifest=json.loads((r/'manifest.json').read_text());assert manifest['state']=='complete' and manifest['recorder_closed'] is True
states=[];actions=[];results=[]
with rosbag.Bag(str(r/'keyframes.bag'),'r') as b:
 topics=list(b.get_type_and_topic_info().topics)
 selected=[t for t in topics if t=='/grasp/state' or ('follow_joint_trajectory' in t and t.endswith(('/goal','/result')))]
 for topic,msg,stamp in b.read_messages(topics=selected):
  if topic=='/grasp/state':
   row=dict(stamp_ns=stamp.to_nsec(),state=msg.state,active=msg.active,success=msg.success,message=msg.message)
   if not states or any(row[k]!=states[-1][k] for k in ['state','active','success','message']): states.append(row)
  elif topic.endswith('/goal'):
   traj=msg.goal.trajectory;actions.append(dict(stamp_ns=stamp.to_nsec(),goal_id=msg.goal_id.id,joint_names=list(traj.joint_names),points=len(traj.points),duration_sec=traj.points[-1].time_from_start.to_sec(),start=list(traj.points[0].positions),end=list(traj.points[-1].positions)))
  else:results.append(dict(stamp_ns=stamp.to_nsec(),goal_id=msg.status.goal_id.id,status=msg.status.status,error_code=msg.result.error_code))
report=dict(generation=8,plan_id='ee718df75606887be44ea8ff',record_dir=str(r),record_manifest_sha256=hashlib.sha256((r/'manifest.json').read_bytes()).hexdigest(),original_record_result=manifest['result'],bag_closed=True,states=states,actions=actions,action_results=results,operator_confirmation=dict(message_id='call_674d41e969604f0a9e9bd8b8850c66e7:0',text='已夹住新目标',source='actual_user_reply',gripped=True),physical_lift_after_close=False,lift_executed=False,updated_at_utc=datetime.now(timezone.utc).isoformat())
assert not any(s['state']=='LIFT_OBJECT' for s in states)
assert states[-1]['state']=='HOLDING' and states[-1]['active'] is False
assert all(x['status']==3 and x['error_code']==0 for x in results)
(out/'operator_confirmed_result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
doc=w/'docs/verification/2026-10-04-thin-target-rgb-observation.md'
with doc.open('a') as f:f.write('\n\n补充gen8终态：新录制 `20261006T061242.642572Z_f8b97892d0a84d5599287c74677f32d2` 已封口，原程序结果 `unknown/HOLDING/GRASP_HOLD_UNVERIFIED` 原样保留；封口bag仅核对状态与action记录，未进入 `LIFT_OBJECT`，已返回的机械臂action全部成功。闭爪前位置误差约1.849 mm，模型接触重合11.391 mm、支撑面间隙15.161 mm，最终无需额外pose2微步。用户真实回复 `call_674d41e969604f0a9e9bd8b8850c66e7:0` 为“已夹住新目标”，因此本次**新目标实际夹取成功、按要求保持不提升**。独立补充报告 `grasp_close_hold_20261006T061242Z/operator_confirmed_result.json` 保存用户来源、完整状态、轨迹及回执，不篡改封口manifest或将触觉未知改写成自动验证成功。后续未知物任务继续使用不提升策略，恢复提升需另有用户指令；此时不启动额外抓取。\n')
shutil.copy2(doc,e/'文档/verification'/doc.name)
p=e/'ros_log/full_ros_resume_20261006T024224Z/current_progress.json';d=json.loads(p.read_text());d['latest_update_utc']=report['updated_at_utc'];d['latest_unknown_grasp'].update(state='closed_and_holding_operator_confirmed_gripped',operator_confirmation=report['operator_confirmation'],record_dir=str(r),report=str(out/'operator_confirmed_result.json'),automatic_next_grasp=False);p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(report=str(out/'operator_confirmed_result.json'),action_count=len(actions),action_results=results,final_state=states[-1],operator_confirmed=True,lift_executed=False),ensure_ascii=False,indent=2))
