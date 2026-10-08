"""Observe current production preview only; no selection or execution request."""
import hashlib,json,os,time
from pathlib import Path
from datetime import datetime,timezone
ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
if not os.path.ismount('/media/zhuyupei/SanStand'):raise RuntimeError('SanStand absent')
out=ROOT/'ros_log'/('current_grasp_preview_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
out.mkdir();(out/'ros').mkdir();os.environ['ROS_LOG_DIR']=str(out/'ros')
import rospy
from std_msgs.msg import String
from alicia_flexible_grasp_supervisor.msg import GraspState,Grasp6DPlan
from alicia_flexible_grasp_supervisor.srv import TriggerZero
def message_dict(value):
 if hasattr(value,'__slots__'):return {k:message_dict(getattr(value,k)) for k in value.__slots__}
 if isinstance(value,(list,tuple)):return [message_dict(x) for x in value]
 return value
rospy.init_node('readonly_current_grasp_preview',anonymous=True,disable_signals=True)
state=rospy.wait_for_message('/grasp/state',GraspState,timeout=3)
if state.active:raise RuntimeError('task is active, do not change planning stream')
selection=rospy.get_param('/grasp_mode/selection');start=rospy.Time.now().to_nsec();seen={};subs=[]
def record(m,kind):
 now=rospy.Time.now().to_nsec()
 if kind=='preview':
  if m.header.stamp.to_nsec()<=start:return
  data=message_dict(m)
  seen[kind]={'source_stamp_ns':m.header.stamp.to_nsec(),'valid':m.valid,'plan_id':m.plan_id,'diagnostic':m.diagnostic,'candidate_source':m.candidate_source}
 else:
  try:data=json.loads(m.data)
  except Exception:data={'message':m.data}
  seen[kind]={'received_ns':now}
 p=out/(kind+'_'+str(now)+'.json');p.write_text(json.dumps(data,ensure_ascii=False,indent=2))
 if kind=='preview':print('PREVIEW',json.dumps(seen[kind],ensure_ascii=False),flush=True)
for topic,kind,typ in [('/grasp_6d/preview_plan_enriched','preview',Grasp6DPlan),('/grasp_6d/gate_audit','gate_audit',String),('/grasp_6d/pipeline_metrics','pipeline_metrics',String),('/grasp_6d/status','planner_status',String)]:
 subs.append(rospy.Subscriber(topic,typ,lambda m,k=kind:record(m,k),queue_size=8))
(out/'context.json').write_text(json.dumps({'selection':selection,'start_ns':start,'motion_requested':False,'source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()},ensure_ascii=False,indent=2))
rospy.wait_for_service('/grasp_6d/request_plan',timeout=3)
r=rospy.ServiceProxy('/grasp_6d/request_plan',TriggerZero)(True)
print('PLAN_STREAM',r.success,r.message,flush=True)
print('SESSION',out,flush=True)
deadline=time.monotonic()+100
while time.monotonic()<deadline and not rospy.is_shutdown():
 if rospy.get_param('/grasp_mode/selection')!=selection:raise RuntimeError('selection changed')
 if 'preview' in seen:break
 time.sleep(.15)
for sub in subs:sub.unregister()
(out/'result.json').write_text(json.dumps({'selection':selection,'motion_requested':False,'seen':seen,'stream_left_in_existing_running_state':True},ensure_ascii=False,indent=2))
print('RESULT',json.dumps(seen,ensure_ascii=False),flush=True)
