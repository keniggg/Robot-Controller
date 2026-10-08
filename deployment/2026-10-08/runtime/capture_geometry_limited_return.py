import json,time,pathlib,rospy
from easy_handeye_msgs.srv import TakeSample
c=json.load(open('.runtime/current_handeye_task_pose_validation.json'));out=pathlib.Path(c['session']);active=pathlib.Path(c.get('active_sample_session',str(out)))
rospy.init_node('capture_bounded_return_observation',anonymous=True,disable_signals=True)
for attempt in range(3):
 try:
  response=rospy.ServiceProxy(c['namespace']+'/take_sample',TakeSample)();e=dict(event='validation_sample_accepted',time=time.time(),pose_id=20,role='geometry_limited_return',goal_reached=False,substitution_reason='full 4.8 degree response envelope could not retain original view/clearance at requested seed; actual stopped pose retained',count=len(response.samples.hand_world_samples),sample_session=str(active));break
 except rospy.ServiceException:
  if attempt==2:raise
  time.sleep(.5)
with (out/'automatic_motion.jsonl').open('a') as f:f.write(json.dumps(e)+'\n')
print(json.dumps(e),flush=True)
