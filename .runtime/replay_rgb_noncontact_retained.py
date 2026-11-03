import base64,json,numpy as np,time
from pathlib import Path
from dataclasses import fields
from types import SimpleNamespace as NS
from moveit_msgs.msg import RobotTrajectory
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk,observation_tracking_error_bounds
from alicia_flexible_grasp.robot.observation_tracking_contract import tracking_contract_candidates
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
from alicia_grasp_modes.rgb_observation_volume import RGBObservationVolume,SupportPlaneProvenance
from alicia_grasp_modes.rgb_noncontact_scene import build_rgb_noncontact_scene,validate_rgb_noncontact_trajectory,revalidate_rgb_noncontact_scene,retain_observed_obstacles
P=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/rgb_observation_preview_20261004T015755Z')
rows=json.loads(Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/rgb_observation_bootstrap_20261004T011752Z/result.json').read_text())['rows']
def make(r,reserve):
 a={f.name:r['interest_volume'][f.name] for f in fields(RGBObservationVolume) if f.init};a['support_provenance']=SupportPlaneProvenance(**a['support_provenance']);v=RGBObservationVolume(**a)
 with np.load(r['source_file'],allow_pickle=False) as z:
  return build_rgb_noncontact_scene(interest_volume=v,depth_image=z['depth_raw'],depth_scale_m=float(z['depth_scale']),source_stamp_ns=v.source_stamp_ns,source_depth_sha256=r['source_depth_sha256'],depth_uncertainty_m=.002,valid_depth_range_m=(.03,2.),source_record_id=r['source_file'],containment_reserve_m=reserve)
w=json.loads(base64.b64decode((P/'joint2_-5deg_path.txt').read_text()));plan=RobotTrajectory().deserialize(base64.b64decode(w['trajectory']))
fk=SerialUrdfFk((P/'robot_description.urdf').read_text(),plan.joint_trajectory.joint_names)
g=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003)
constraints={name:dict(trajectory=.12) for name in fk.names}
old=make(rows[2],0.);previous=make(rows[3],0.)
start=time.monotonic();fresh=make(rows[4],0.);build=time.monotonic()-start
bundle=retain_observed_obstacles(retain_observed_obstacles(old,previous),fresh)
print(json.dumps(dict(kind='retained_initial_previous_fresh_measured_union',scene=bundle.scope(),fresh_build_seconds=build)),flush=True)
for contract in tracking_contract_candidates(constraints,fk.names,.5):
 try:
  audit=validate_rgb_noncontact_trajectory(plan,bundle,fk,g,w['opening'],NS(positions=w['hold'],velocities=[0.]*6,accelerations=[0.]*6),joint_error_bounds_rad=contract['joint_error_bounds_rad'],max_seconds=5.)
  print(json.dumps(dict(contract=contract,ok=audit['ok'],proof_seconds=audit['elapsed_sec'],total_fresh_build_proof_seconds=build+audit['elapsed_sec'],minimum_clearance_m=audit['certified_minimum_clearance_m'],box_checks=audit['box_checks'],source_stamp_ns=audit['source_stamp_ns'],scene_sha256=audit['scene_sha256'],retained_source_stamp_ns=audit['retained_source_stamp_ns'])),flush=True)
 except Exception as error:
  print(json.dumps(dict(contract=contract,error=str(error),audit=getattr(error,'audit',{}))),flush=True)
