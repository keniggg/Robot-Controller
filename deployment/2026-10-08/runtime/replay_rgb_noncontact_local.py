import base64,json,numpy as np,time
from pathlib import Path
from dataclasses import fields
from types import SimpleNamespace as NS
from moveit_msgs.msg import RobotTrajectory
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk,observation_tracking_error_bounds
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
from alicia_grasp_modes.rgb_observation_volume import RGBObservationVolume,SupportPlaneProvenance
from alicia_grasp_modes.rgb_noncontact_scene import build_rgb_noncontact_scene,validate_rgb_noncontact_trajectory
P=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/rgb_observation_preview_20261004T015755Z')
r=json.loads(Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/rgb_observation_bootstrap_20261004T011752Z/result.json').read_text())['rows'][-1]
a={f.name:r['interest_volume'][f.name] for f in fields(RGBObservationVolume) if f.init};a['support_provenance']=SupportPlaneProvenance(**a['support_provenance']);v=RGBObservationVolume(**a)
w=json.loads(base64.b64decode((P/'joint2_-5deg_path.txt').read_text()));plan=RobotTrajectory().deserialize(base64.b64decode(w['trajectory']))
fk=SerialUrdfFk((P/'robot_description.urdf').read_text(),plan.joint_trajectory.joint_names)
g=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003)
eps=observation_tracking_error_bounds({name:dict(trajectory=.12) for name in fk.names},fk.names)
with np.load(r['source_file'],allow_pickle=False) as z:
 start=time.monotonic();scene=build_rgb_noncontact_scene(interest_volume=v,depth_image=z['depth_raw'],depth_scale_m=float(z['depth_scale']),source_stamp_ns=v.source_stamp_ns,source_depth_sha256=r['source_depth_sha256'],depth_uncertainty_m=.002,valid_depth_range_m=(.03,2.),source_record_id=r['source_file'])
 print('build_sec',time.monotonic()-start,'scene',json.dumps(scene.scope()),flush=True)
try:
 a=validate_rgb_noncontact_trajectory(plan,scene,fk,g,w['opening'],NS(positions=w['hold'],velocities=[0.]*6,accelerations=[0.]*6),joint_error_bounds_rad=eps,max_seconds=20.)
 print('proof',json.dumps(a),flush=True)
except Exception as e:
 print('ERROR',type(e).__name__,str(e)); print(json.dumps(getattr(e,'audit',{})),flush=True)
