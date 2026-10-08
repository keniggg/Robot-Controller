#!/usr/bin/env python3
"""Fresh geometry-only audit of J2/J5 counterrotation hypotheses; no publishers."""
import sys, pathlib, json, math, time, signal, os
import numpy as np
import gen4_single_axis_plan as base

def main():
 if not pathlib.Path('/media/zhuyupei/SanStand').is_mount():raise RuntimeError('SanStand unavailable')
 out=base.OUT.parent/'coupled_domain_readonly'/time.strftime('%Y%m%dT%H%M%SZ',time.gmtime());out.mkdir(parents=True);(out/'ros').mkdir();os.environ['ROS_LOG_DIR']=str(out/'ros')
 import rospy
 from sensor_msgs.msg import JointState
 from moveit_msgs.msg import RobotState
 from moveit_msgs.srv import GetStateValidity,GetStateValidityRequest
 rospy.init_node('gen4_coupled_geometry_readonly',anonymous=True,disable_signals=True)
 signal.signal(signal.SIGALRM,lambda *_:(_ for _ in ()).throw(RuntimeError('read-only 55s deadline')));signal.alarm(55)
 try:
  s,xml=base.capture(out);rospy.wait_for_service('/check_state_validity',3);proxy=rospy.ServiceProxy('/check_state_validity',GetStateValidity)
  def valid(q,g):
   r=GetStateValidityRequest(group_name='alicia');r.robot_state=RobotState();r.robot_state.joint_state=JointState(name=base.NAMES,position=list(q)+[g]);return proxy(r).valid
  audit=base.GeometryAudit(s,xml,valid);q=np.asarray(s['q_rad'])[:6];rows=[]
  for ratio in [-1.,-.75,-.5,-.25,0]:
   for d in [6.,12.,18.,24.]:
    t=q.copy();t[1]+=math.radians(d);t[4]+=math.radians(ratio*d)
    proof=audit.guard_segment(q,t,False,.3);short={k:v for k,v in proof.items() if k!='rows'}
    # Collision only for candidates whose full source texture/volume stays visible.
    if proof['all_geometry_pass']:
     proof=audit.guard_segment(q,t,True,.3);short={k:v for k,v in proof.items() if k!='rows'}
    rows.append(dict(j2_delta_deg=d,j5_delta_deg=ratio*d,ratio=ratio,target_q_deg=np.degrees(t).tolist(),summary=short))
  report={'source_stamp_ns':s['source_stamp_ns'],'source_q_deg':np.degrees(q).tolist(),'robot_commands_published':0,'candidates':rows,'limits':['Predicted synchronous line only, not execution authorization.','No response envelope or asynchronous scalar segment checked.','Full frozen target and support-feature hull visibility; no geometry overrides.','Sampled MoveIt only if geometric path qualifies.']}
  base.write(out/'report.json',report);print(json.dumps({'directory':str(out),**report},indent=2))
 except BaseException as e:base.write(out/'failure.json',{'error':str(e),'robot_commands_published':0});raise
 finally:signal.alarm(0)
if __name__=='__main__':main()
