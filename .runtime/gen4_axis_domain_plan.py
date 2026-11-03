#!/usr/bin/env python3
"""Fresh read-only, independently varied J2+/J5- domain extension audit.
No command publishers, enable operations, parameter writes, or planner execution.
Imports existing capture/geometry read-only helpers without modifying them.
"""
import pathlib,os,time,json,math,signal,sys,hashlib
import numpy as np
import gen4_single_axis_plan as base
OUT=base.OUT.parent/'axis_domain_plan'
def summary(rows):return {k:v for k,v in base.GeometryAudit.summarize(rows).items() if k!='rows'}
def main():
 if not pathlib.Path('/media/zhuyupei/SanStand').is_mount():raise RuntimeError('external drive unavailable')
 OUT.mkdir(exist_ok=True);directory=OUT/time.strftime('%Y%m%dT%H%M%SZ',time.gmtime());directory.mkdir();(directory/'ros').mkdir();os.environ['ROS_LOG_DIR']=str(directory/'ros')
 import rospy
 from sensor_msgs.msg import JointState
 from moveit_msgs.msg import RobotState
 from moveit_msgs.srv import GetStateValidity,GetStateValidityRequest
 rospy.init_node('gen4_axis_domain_readonly_plan',anonymous=True,disable_signals=True)
 signal.signal(signal.SIGALRM,lambda *_:(_ for _ in ()).throw(RuntimeError('readonly audit 55s deadline')));signal.alarm(55)
 try:
  snapshot,xml=base.capture(directory);rospy.wait_for_service('/check_state_validity',timeout=3);proxy=rospy.ServiceProxy('/check_state_validity',GetStateValidity)
  def valid(q,g):
   req=GetStateValidityRequest(group_name='alicia');req.robot_state=RobotState();req.robot_state.joint_state=JointState(name=base.NAMES,position=list(q)+[g]);return proxy(req).valid
  audit=base.GeometryAudit(snapshot,xml,valid);actual=np.array(snapshot['q_rad']);reference=np.array(snapshot['sdk_q_rad']);analyses=[]
  for joint,direction in [(2,1),(5,-1)]:
   index=joint-1;steps=[];all_samples=[];earliest=None
   step_deg=math.degrees(math.floor(math.radians(3)/base.QUANTUM)*base.QUANTUM)
   for stage_count in [1,2,3,4]:
    requested_cumulative=3.*stage_count;cumulative=step_deg*stage_count
    start=reference[:6].copy();start[index]+=direction*math.radians(cumulative-step_deg)
    end=reference[:6].copy();end[index]+=direction*math.radians(cumulative)
    # Nominal command changes only the selected SDK word; 3deg is conceptual,
    # representable targets are rounded once at each absolute waypoint.
    command=reference.copy();command[index]=((base.words(np.r_[end,reference[6]])[index])*base.QUANTUM-math.pi)
    # Accepted response allowance is +/-0.8deg about each nominal waypoint,
    # clipped by explicit initial-actual bounds. It does NOT compound per step.
    lo=min(actual[index],reference[index]+direction*math.radians(cumulative))-.8*math.pi/180
    hi=max(actual[index],reference[index]+direction*math.radians(cumulative))+.8*math.pi/180
    nominal=audit.guard_segment(actual[:6] if cumulative==3 else start,end)
    # Full response interval accumulated so far, held-axis bridge sampled at
    # actual / SDK midpoint / SDK; current feedback errors retained explicitly.
    reserve=[]
    for bridge in [0,.5,1]:
     held=actual[:6]*(1-bridge)+reference[:6]*bridge
     for a in np.linspace(lo,hi,int(math.ceil(math.degrees(hi-lo)/.15))+1):
      q=held.copy();q[index]=a;r=audit.geometry(q);r['edited_axis_offset_from_initial_actual_deg']=math.degrees(a-actual[index]);r['held_axis_bridge']=bridge;reserve.append(r)
    # Return concept preserves original SDK words. Check each <=3deg nominal
    # reverse segment separately; measured return needs fresh runtime authority.
    ret=[];return_end=end.copy()
    for return_count in reversed(range(stage_count)):
     level=return_count*step_deg
     return_target=reference[:6].copy();return_target[index]+=direction*math.radians(level);ret.append(audit.guard_segment(return_end,return_target));return_end=return_target
    rs=summary(reserve);nomsummary={k:v for k,v in nominal.items() if k!='rows'};returnsummary=[{k:v for k,v in r.items() if k!='rows'} for r in ret]
    failures=[r for r in reserve if not r['geometry_pass'] or r['moveit_collision_valid'] is not True]
    # Earliest outward failing position, on any held-axis bridge.
    if failures:
     f=min(failures,key=lambda r:max(0,direction*r['edited_axis_offset_from_initial_actual_deg']))
     if earliest is None:earliest=f
    passed=nomsummary['all_geometry_pass'] and nomsummary['all_moveit_valid'] and rs['all_geometry_pass'] and rs['all_moveit_valid'] and all(r['all_geometry_pass'] and r['all_moveit_valid'] for r in returnsummary)
    step={'requested_cumulative_sdk_delta_deg':direction*requested_cumulative,'cumulative_sdk_delta_deg':direction*cumulative,'nominal_max_command_step_deg':step_deg,'sdk_command_q_rad':command.tolist(),'sdk_command_words':base.words(command),'edited_sdk_angle_deg':math.degrees(command[index]),'nominal_actual_offset_deg':math.degrees(end[index]-actual[index]),'absolute_response_bounds_deg':list(map(math.degrees,[lo,hi])),'response_bounds_from_initial_actual_deg':list(map(math.degrees,[lo-actual[index],hi-actual[index]])),'response_margin_deg':.8,'reservation_margin_does_not_compound':True,'nominal':nomsummary,'return_segments':returnsummary,'reservation':rs,'all_pass':passed,'first_reservation_failure':None if not failures else f}
    steps.append(step);all_samples.append({'cumulative_sdk_delta_deg':direction*cumulative,'nominal':nominal['rows'],'reservation':reserve,'return':[r['rows'] for r in ret]})
   analyses.append({'joint':joint,'direction':direction,'held_sdk_words':base.words(reference),'initial_actual_angle_deg':math.degrees(actual[index]),'initial_sdk_angle_deg':math.degrees(reference[index]),'initial_sdk_minus_actual_deg':math.degrees(reference[index]-actual[index]),'stages':steps,'first_outward_failure':earliest,'full_samples':all_samples})
  report={'kind':'fresh_independent_single_axis_domain_extension_audit','robot_commands_published':0,'source_stamp_ns':snapshot['source_stamp_ns'],'snapshot':snapshot,'axes':analyses,'collision_unique_states':len(audit.cache),'limits':['No live ownership, enable, commands, calibration, or collector/factory changes.','Fresh at capture only. Each actual motion submission requires freshly validated authority, q, scene and source evidence.','SDK nominal sequence increments by exactly34wire counts=2.98828125deg (always <=3deg); actual feedback-dependent steps may require more segments to stay <=3deg and must not assume the motor reached a command.','Each cumulative endpoint uses +/-0.8deg local response allowance; no 0.8deg-per-stage accumulation is silently authorized. Absolute response bounds are included at every stage.','Selected-axis deviation only; unedited SDK words and gripper preserved. Held actual-vs-SDK bridge sampled at0/.5/1; other physical errors outside these states are not certified.','Return restores initial SDK words; it does not assert return to identical physical pose.','Visibility is target observed volume plus current measured tabletop feature hull. More pose motion is not guaranteed to preserve matching feature quality.','All path/response checks are sampled at<=0.15deg and not a continuous collision proof.']}
  base.write(directory/'report.json',report);base.write(OUT/'latest.json',{'directory':str(directory),'source_stamp_ns':snapshot['source_stamp_ns']});(directory/'source.py').write_bytes(pathlib.Path(__file__).read_bytes())
  compact={'directory':str(directory),'source_stamp_ns':snapshot['source_stamp_ns'],'q_deg':np.degrees(actual[:6]).tolist(),'plane':snapshot['plane_base'],'target_bbox':snapshot['target_bbox_xywh'],'axes':[{**{k:v for k,v in a.items() if k not in ['full_samples','stages']},'stages':[{k:v for k,v in s.items() if k not in ['first_reservation_failure']} for s in a['stages']]} for a in analyses]};print(json.dumps(compact,indent=2))
 except BaseException as e:base.write(directory/'failure.json',{'error':str(e),'robot_commands_published':0});raise
 finally:signal.alarm(0)
if __name__=='__main__':main()
