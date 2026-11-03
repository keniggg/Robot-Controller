#!/usr/bin/env python3
"""Replay closed exact RGB-D against the saved immutable production anchor."""
import sys,json,argparse
from pathlib import Path
import numpy as np
import cv2,rosbag
from cv_bridge import CvBridge
HERE=Path(__file__).resolve().parent;ROOT=HERE.parent
sys.path[:0]=[str(HERE),str(ROOT/'src/alicia_grasp_modes/src'),str(ROOT/'src/alicia_flexible_grasp_supervisor/src')]
from alicia_grasp_modes.tabletop import Config,Candidate,segment_with_support_fallback,_matching_candidates,project_base_to_pixel
from alicia_grasp_modes.rgb_instance import refine_candidate
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk

def main():
 ap=argparse.ArgumentParser();ap.add_argument('session',type=Path);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args();out=a.output;out.mkdir(parents=True,exist_ok=True)
 transition=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/board_removed_scene_transition_20260928T142430Z/transition.json')
 status=json.loads(transition.read_text())['detector_status_after'];camera=json.loads((a.session/'camera_parameters.json').read_text())
 import rospy
 cfgraw=rospy.get_param('/unknown_perception',{});(out/'runtime_unknown_perception.json').write_text(json.dumps(cfgraw,indent=2)+'\n')
 from dataclasses import fields
 cfg=Config(**{f.name:cfgraw[f.name] for f in fields(Config) if f.name in cfgraw});intr=[float(camera[k]) for k in ('fx','fy','cx','cy')]
 rgb={};depth={};q=[];br=CvBridge()
 with rosbag.Bag(str(a.session/'evidence.bag')) as bag:
  for t,m,r in bag.read_messages(topics=['/production_lock/unfiltered_diagnostic_rgb','/production_lock/unfiltered_diagnostic_depth','/alicia_d/accepted_joint_states']):
   if t.endswith('accepted_joint_states'):
    d=dict(zip(m.name,m.position));q.append((m.header.stamp.to_nsec(),[d['Joint%d'%i] for i in range(1,7)]))
   elif t.endswith('rgb'):rgb[m.header.stamp.to_nsec()]=br.imgmsg_to_cv2(m,'bgr8')
   else:depth[m.header.stamp.to_nsec()]=br.imgmsg_to_cv2(m,'passthrough')
 fk=SerialUrdfFk((a.session/'robot_description.urdf').read_text(),['Joint%d'%i for i in range(1,7)])
 closed=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/bounded_motor_centering_20260928T141708Z_351496/center_J3_post_hold_geometry.json')
 X=np.array(json.loads(closed.read_text())['T_tool_camera']);rows=[]
 for ns in sorted(set(rgb)&set(depth)):
  nearest=min(q,key=lambda x:abs(x[0]-ns));F=fk(nearest[1])@X;center=np.array(status['anchor_position_base']);pc=np.linalg.inv(F)@np.r_[center,1]
  anchor=Candidate(np.zeros(depth[ns].shape,np.uint8),project_base_to_pixel(center,F,intr),(0,0,0,0),pc[:3],center,np.array(status['anchor_extent_m']),pc[0],1.,{},np.array(status['reference_support_footprint_base']))
  row=dict(source_stamp_ns=ns,accepted_source_stamp_ns=nearest[0],accepted_rad=nearest[1],starting_anchor=dict(generation=status['generation'],anchor_source_stamp_ns=status['anchor_source_stamp_ns'],position=status['anchor_position_base'],extent=status['anchor_extent_m']),replayed_against_anchor_without_live_reset=True)
  cv2.imwrite(str(out/(str(ns)+'_rgb.png')),rgb[ns]);np.save(out/(str(ns)+'_z16.npy'),depth[ns])
  try:
   dm=np.asarray(depth[ns],np.float32)*float(camera['depth_scale'])
   result=segment_with_support_fallback(dm,intr,F,cfg,anchor=anchor,last=anchor,reference_plane_base=np.array(status['reference_support_plane_base']))
   matching=_matching_candidates(result.candidates,anchor,anchor,cfg)
   row.update(candidate_count=len(result.candidates),matching_count=len(matching),metrics=result.metrics,candidates=[dict(position=c.position_base.tolist(),extent=c.extent_m.tolist(),quality=c.quality,distance_m=float(np.linalg.norm(c.position_base-center)),extent_ratio=(c.extent_m[:2]/anchor.extent_m[:2]).tolist()) for c in result.candidates])
   if len(matching)==1:
    refined=refine_candidate(rgb[ns],dm,intr,F,matching[0][1],cfg);row['refined']=dict(position=refined.position_base.tolist(),extent=refined.extent_m.tolist(),quality=refined.quality,metrics=refined.metrics)
    cv2.imwrite(str(out/(str(ns)+'_mask.png')),refined.mask)
  except Exception as e:row['error']=str(e)
  rows.append(row)
 report=dict(session=str(a.session),diagnostic_only=True,motion_authorized=False,live_tracker_modified=False,used_old_board_pose=False,rows=rows,limits=['The last refined candidate before loss was not archived; replay uses the immutable anchor for the last-position gate too.','These frames follow the original loss, so a passing replay cannot identify the triggering frame.'])
 (out/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(dict(output=str(out),rows=[{k:r[k] for k in ('source_stamp_ns','candidate_count','matching_count','error') if k in r} for r in rows]),indent=2))
if __name__=='__main__':main()
