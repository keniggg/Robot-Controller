import base64,json,numpy as np,time
from pathlib import Path
from dataclasses import fields
from types import SimpleNamespace as NS
from moveit_msgs.msg import RobotTrajectory
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk,observation_tracking_error_bounds
from alicia_flexible_grasp.robot.observation_tracking_contract import tracking_contract_candidates
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
from alicia_grasp_modes.rgb_observation_volume import RGBObservationVolume,SupportPlaneProvenance
from alicia_grasp_modes.rgb_noncontact_scene import build_rgb_noncontact_scene,validate_rgb_noncontact_trajectory,revalidate_rgb_noncontact_scene,validate_new_depth_holes
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
start=time.monotonic();old=make(rows[2],.003);oldtime=time.monotonic()-start
start=time.monotonic();fresh=make(rows[3],0.);freshtime=time.monotonic()-start
print(json.dumps(dict(old_stamp=old.source_ns,fresh_stamp=fresh.source_ns,old_build_sec=oldtime,fresh_build_sec=freshtime)),flush=True)
contracts=[dict(policy='original',joint_error_bounds_rad=observation_tracking_error_bounds(constraints,fk.names).tolist())]+list(tracking_contract_candidates(constraints,fk.names,.5))
for contract in contracts:
 try:
  a=validate_rgb_noncontact_trajectory(plan,old,fk,g,w['opening'],NS(positions=w['hold'],velocities=[0.]*6,accelerations=[0.]*6),joint_error_bounds_rad=contract['joint_error_bounds_rad'],max_seconds=15.)
  print(json.dumps(dict(contract=contract,proof_sec=a['elapsed_sec'],clearance_m=a['certified_minimum_clearance_m'],box_checks=a['box_checks'])),flush=True)
  try:
   fresh_receipt=revalidate_rgb_noncontact_scene(old,fresh,a)
   print('REFRESH',json.dumps(fresh_receipt),flush=True)
   break
  except Exception as refresh_error:
   print('REFRESH_FAIL',str(refresh_error),json.dumps(getattr(refresh_error,'audit',{})),flush=True)
   if contract.get('path_position_tolerance_rad',[1])[0] <= .06:
    direct=validate_rgb_noncontact_trajectory(plan,fresh,fk,g,w['opening'],NS(positions=w['hold'],velocities=[0.]*6,accelerations=[0.]*6),joint_error_bounds_rad=contract['joint_error_bounds_rad'],max_seconds=2.)
    hole_receipt=validate_new_depth_holes(old,fresh,direct)
    print('FRESH_HOLES',json.dumps(hole_receipt),flush=True)
    print('FRESH_DIRECT_PROOF',json.dumps(dict(proof_sec=direct['elapsed_sec'],total_fresh_seconds=freshtime+direct['elapsed_sec'],clearance=direct['certified_minimum_clearance_m'],contract=contract,ok=direct['ok'])),flush=True)
    if direct['elapsed_sec']+freshtime<.6: break
 except Exception as e:
  print('FAIL',str(e),json.dumps(getattr(e,'audit',{})),flush=True)
