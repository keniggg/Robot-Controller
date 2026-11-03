"""Diagnostic original guard on fresh offline IK curves, not recorded MoveIt paths."""
from pathlib import Path
import importlib.util,json,hashlib
from copy import deepcopy
import numpy as np
import rospy
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from alicia_flexible_grasp.robot.observation_path_guard import FrozenObservationScene,validate_observation_trajectory
from alicia_flexible_grasp.robot.contact_sequence_precheck import predict_contact_command_path
from geometry_msgs.msg import Pose
spec=importlib.util.spec_from_file_location('q30',Path('.runtime/quantify_gen30_actual_contact.py'));m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
scene=FrozenObservationScene(m.data['plan_id'],hashlib.sha256((m.B/'actual_contact_rejections.json').read_bytes()).hexdigest(),m.data['source_stamp_ns'],m.normal,m.offset,m.center,m.axes,m.size)
rows=[]
for goal,placement in m.options:
 delta=placement['normal_translation_m'];first=m.Ts[1].copy();first[:3,3]+=delta*m.normal
 q=m.seed.copy();tr=RobotTrajectory();tr.joint_trajectory.joint_names=list(m.fk.names);ik=[]
 for i,a in enumerate(np.linspace(0,1,13)):
  T=first.copy();T[:3,3]=(1-a)*first[:3,3]+a*goal[:3,3]
  q,e=m.solve(T,q);ik.append(e)
  if e['position_error_m']>1e-8 or e['orientation_error_rad']>1e-8:raise ValueError('IK failed')
  tr.joint_trajectory.points.append(JointTrajectoryPoint(positions=q.tolist(),velocities=[0.]*6,accelerations=[0.]*6,time_from_start=rospy.Duration(float(i))))
 goalmsg=Pose()
 for name,value in zip('xyz',goal[:3,3]):setattr(goalmsg.position,name,float(value))
 for name,value in zip('xyzw',m.data['poses_xyz_xyzw'][2][3:]):setattr(goalmsg.orientation,name,float(value))
 shifted,derivation=predict_contact_command_path(tr,m.fk,m.sample,goalmsg)
 checks={}
 for name,path in [('nominal',tr),('predicted_command',shifted)]:
  try:checks[name]=dict(ok=True,audit=validate_observation_trajectory(path,scene,m.fk,m.g,m.opening,path.joint_trajectory.points[0]))
  except ValueError as exc:checks[name]=dict(ok=False,reason=str(exc))
 rows.append(dict(normal_translation_m=delta,waypoints=len(tr.joint_trajectory.points),nominal_positions_rad=[list(p.positions) for p in tr.joint_trajectory.points],checks=checks,ik=ik,constant_bias_derivation=derivation))
out=dict(scope='ORIGINAL_CONTINUOUS_GEOMETRY_GUARD_ON_INDEPENDENT_IK_CURVES',source_stamp_ns=m.data['source_stamp_ns'],plan_id=m.data['plan_id'],rows=rows,limitations=['Original failed MoveIt trajectories and branch are unavailable. These curves are independently solved with zero velocity/acceleration at 13 waypoints.','No whole-arm MoveIt or updated fused-surface geometry checks have been replaced. A passed gripper curve does not authorize planning or execution.'],input_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),Path('.runtime/quantify_gen30_actual_contact.py'),m.B/'actual_contact_rejections.json']})
p=m.B/'contact_curve_counterfactual.json';p.write_text(json.dumps(m.convert(out),indent=2)+'\n');print(json.dumps(dict(result=str(p),rows=[dict(delta=r['normal_translation_m'],checks=r['checks']) for r in rows]),indent=2))
