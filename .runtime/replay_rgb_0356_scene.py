"""Offline only: exact saved path and measured scenes; never a ROS client."""
import cProfile
from dataclasses import fields
import hashlib
import io
import json
from pathlib import Path
import pstats
import time
from types import SimpleNamespace

import numpy as np
from moveit_msgs.msg import RobotTrajectory
from alicia_grasp_modes.rgb_observation_volume import RGBObservationVolume,SupportPlaneProvenance
from alicia_grasp_modes.rgb_noncontact_scene import build_rgb_noncontact_scene,retain_observed_obstacles,validate_rgb_noncontact_trajectory
from alicia_grasp_modes.tabletop import Config
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk,wire_digest
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry

P=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/rgb_acquisition_gateway_20261004T035645.336945Z_6fe972e1')
record=json.loads((P/'result.json').read_text());audit=record['audit']
plan=RobotTrajectory().deserialize((P/'trajectory.rosmsg').read_bytes())
fk=SerialUrdfFk((P/'robot_description.urdf').read_text(),plan.joint_trajectory.joint_names)
assert wire_digest(plan)==record['trajectory_sha256']
assert fk.model_sha256==record['robot_description_sha256']
g=audit['gripper_geometry_snapshot']
gripper=GripperGeometry(g['max_inner_gap_m'],g['width_safety_margin_per_side_m'],np.array(g['finger_box_xyz_m']),np.array(g['palm_box_xyz_m']),g.get('support_clearance_m',.003))
hold=SimpleNamespace(positions=audit['bridge_start_joint_positions_rad'],velocities=[0.]*6,accelerations=[0.]*6)

def build(label):
 row=record['frames'][label]
 assert hashlib.sha256(Path(row['source_file']).read_bytes()).hexdigest()==row['source_file_sha256']
 volumes=[]
 for raw in row['interest']['interest_volumes']:
  raw=dict(raw);raw['support_provenance']=SupportPlaneProvenance(**raw['support_provenance'])
  volumes.append(RGBObservationVolume(**{f.name:raw[f.name] for f in fields(RGBObservationVolume) if f.init}))
 config=Config(**row['source_context']['perception'])
 scene_id='rgb_acquisition:'+audit['rgb_request_id']+('' if label=='postplan' else ':fresh')
 with np.load(row['source_file'],allow_pickle=False) as z:
  return build_rgb_noncontact_scene(interest_volume=volumes[0],additional_interest_volumes=volumes[1:],
   depth_image=z['depth_raw'],depth_scale_m=float(z['depth_scale']),source_stamp_ns=int(row['source_stamp_ns']),
   source_depth_sha256=row['source_depth_sha256'],depth_uncertainty_m=config.support_threshold_m,
   valid_depth_range_m=(config.depth_min_m,config.depth_max_m),source_record_id=scene_id)

if __name__=='__main__':
 t=time.monotonic();post=build('postplan');print('BUILD_POST',time.monotonic()-t,flush=True)
 t=time.monotonic();proof=build('proof');print('BUILD_PROOF',time.monotonic()-t,flush=True)
 scene=retain_observed_obstacles(post,proof)
 print('BINDING',json.dumps(dict(scene_matches_saved=scene.digest==audit['scene_sha256'],
  scene=scene.digest,path=wire_digest(plan),source_stamps=[s.source_ns for s in scene.scenes],
  selected_errors=audit['joint_error_bounds_rad'],boxes=len(scene.depth_bounds))),flush=True)
 profile=cProfile.Profile();profile.enable()
 report=validate_rgb_noncontact_trajectory(plan,scene,fk,gripper,audit['opening_width_m'],hold,
  joint_error_bounds_rad=audit['joint_error_bounds_rad'],max_checks=4096,max_seconds=5.)
 profile.disable()
 print('REPLAY',json.dumps({k:report[k] for k in ('ok','elapsed_sec','box_checks','certified_minimum_clearance_m','following_support')}),flush=True)
 stream=io.StringIO();pstats.Stats(profile,stream=stream).sort_stats('cumtime').print_stats(25)
 print(stream.getvalue(),flush=True)
