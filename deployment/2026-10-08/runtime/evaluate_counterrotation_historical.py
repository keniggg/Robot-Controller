#!/usr/bin/env python3
"""Historical closed-bag geometry only. No ROS node, control or parameter I/O."""
import importlib.util
import json
import os
from pathlib import Path
import sys
import numpy as np
import rosbag

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/alicia_flexible_grasp_supervisor/src'))
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry,_stage_boxes,_box_corners
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk

def module(name,path):
 s=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
C=module('coupled',ROOT/'.runtime/coupled_joint23_geometry.py')
CAM=module('camera',ROOT/'src/real-arm/alicia_d_calibration/scripts/charuco_tracker.py')
P=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/bounded_joint3_response_20260928T110035Z_26293')
OUT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/grasp_repair_resume_20260928T120606Z/pose_chain_identifiability')
assert os.path.ismount('/media/zhuyupei/SanStand')
g=json.loads((P/'return_original_sdk_geometry.json').read_text());plan=json.loads((P/'plan.json').read_text())
refs=[]
with rosbag.Bag(str(P/'evidence.bag')) as bag:
 for topic,msg,t in bag.read_messages(topics=['/alicia_d/control_reference']):
  if msg.header.stamp.to_nsec()<=g['source_stamp_ns']:
   refs.append((msg.header.stamp.to_nsec(),dict(zip(msg.name,msg.position))))
ns,ref=max(refs)
names=['Joint%d'%i for i in range(1,7)]+['right_finger'];q=np.asarray(g['accepted_positions_rad']);sdk=np.array([ref[n] for n in names])
binding=dict(now_ns=max(g['source_stamp_ns'],g['accepted_stamp_ns'])+50_000_000,source_stamp_ns=g['source_stamp_ns'],accepted_stamp_ns=g['accepted_stamp_ns'],sdk_stamp_ns=ns,epoch_ns=plan['frozen_epoch_ns'],accepted_epoch_ns=plan['frozen_epoch_ns'],sdk_epoch_ns=plan['frozen_epoch_ns'],camera_fingerprint=g['quality']['camera_model_fingerprint'],handeye_sha256='6ab0e8c902fe87bad1935a69b72044930b79ce1b239728cfce6dda8e94533468')
fk=SerialUrdfFk((P/'robot_description.urdf').read_text(),names[:6])
model=CAM.RuntimeColorModel(CAM.runtime_color_model_evidence(g['camera_parameters'],'camera_link','/supervisor/camera/color/image_raw'),sdk=__import__('pyrealsense2'))
gripper=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003)
def corners(tool,opening):
 return np.concatenate([_box_corners(c,tool[:3,:3],size) for _,c,size in _stage_boxes(tool,gripper,opening,'y','z')])
def project(points):
 optical=np.column_stack((-points[:,1],-points[:,2],points[:,0]))
 return [model.sdk.rs2_project_point_to_pixel(model.intrinsics,p.tolist()) for p in optical]
ci=model.evidence['color_intrinsics']
r=C.evaluate_counterrotation_candidates(q,sdk,g['T_tool_camera'],g['T_base_board'],binding,fk,corners,project,(ci['width'],ci['height']),fk.limits[:,0],fk.limits[:,1])
samples=r.pop('samples')
r['historical_only']=True;r['source_geometry']=str(P/'return_original_sdk_geometry.json');r['source_sdk_stamp_ns']=ns
r['worst_board_sample']=min(samples,key=lambda row:row['full_board_edge_px']);r['worst_plane_sample']=min(samples,key=lambda row:row['plane_clearance_m'])
failed=[row for row in samples if row['kind']=='asynchronous_absolute_response_envelope' and not row['geometric_sample_pass']]
r['failed_envelope_samples']=len(failed)
r['worst_board_sample_delta_from_measured_deg']=np.degrees(np.array(r['worst_board_sample']['q_rad'])-q[:6]).tolist()
(OUT/'historical_j3_counterrotation_geometry.json').write_text(json.dumps(r,indent=2,allow_nan=False)+'\n')
(OUT/'evaluate_counterrotation_historical.py').write_bytes(Path(__file__).read_bytes())
print(json.dumps(r,ensure_ascii=False,allow_nan=False))
