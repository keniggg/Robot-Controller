"""Fresh candidate computation only; never requests grasp or motion."""
import json,time
from pathlib import Path
import rospy
from std_msgs.msg import String
from alicia_grasp_modes.srv import SetGraspMode
from alicia_flexible_grasp_supervisor.srv import TriggerZero
from alicia_flexible_grasp_supervisor.msg import GraspState,Grasp6DPlan
rospy.init_node('fixed_target_current_view_preview',anonymous=True,disable_signals=True)
p=Path(json.load(open('.runtime/current_handeye_deployment.json'))['directory'])
assert not rospy.wait_for_message('/grasp/state',GraspState,timeout=5).active
before=rospy.get_param('/grasp_mode/selection');r=rospy.ServiceProxy('/grasp_mode/select',SetGraspMode)('unknown','direct');print('SELECTION',r.success,r.message,flush=True)
assert r.success
for _ in range(90):
 s=json.loads(rospy.wait_for_message('/grasp_mode/status',String,timeout=2).data)
 if s.get('generation',0)>before['generation'] and s.get('state')=='ready':break
 time.sleep(.1)
else:raise RuntimeError('mode not ready')
selection=rospy.get_param('/grasp_mode/selection');start=rospy.Time.now().to_nsec();data={}
def audit(m):
 try:
  a=json.loads(m.data)
  if a.get('snapshot_stamp_ns',0)>start and a.get('grasp_mode_selection',{}).get('generation')==selection['generation']:
   data['audit']=a;(p/'current_observation_direct_audit.json').write_text(json.dumps(a,indent=2)+'\n')
 except Exception as e:print('audit',e,flush=True)
def preview(m):
 if m.header.stamp.to_nsec()>start and (m.valid or m.candidate_source=='near_field_terminal'):
  data['preview']={'valid':m.valid,'plan_id':m.plan_id,'diagnostic':m.diagnostic};print('PREVIEW',data['preview'],flush=True)
sub1=rospy.Subscriber('/grasp_6d/gate_audit',String,audit,queue_size=8);sub2=rospy.Subscriber('/grasp_6d/preview_plan_enriched',Grasp6DPlan,preview,queue_size=8)
request=rospy.ServiceProxy('/grasp_6d/request_plan',TriggerZero)
print('PLAN_ONLY_START',request(True),flush=True)
try:
 deadline=time.monotonic()+125
 while time.monotonic()<deadline and not rospy.is_shutdown():
  if 'preview' in data and ('audit' in data or not data['preview']['valid']):break
  time.sleep(.25)
finally:
 print('PLAN_COMPUTATION_END',request(False),flush=True)
 (p/'current_observation_preview_result.json').write_text(json.dumps({'selection':selection,'motion_requested':False,'preview':data.get('preview'),'audit_available':'audit' in data},indent=2)+'\n')
