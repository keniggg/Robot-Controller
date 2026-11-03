import json,math,time
from pathlib import Path
import numpy as np,cv2,rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from handeye_auto_motion import AutoMotion
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry,_stage_boxes,_box_corners
ROOT=Path(__file__).resolve().parents[1];c=json.loads((ROOT/'.runtime/current_handeye_autocalibration.json').read_text());p=Path(c['session'])
rospy.init_node('handeye_closer_view',anonymous=True,disable_signals=True);motion=AutoMotion(p)
s=json.loads((p/'starting_geometry.json').read_text());names=['Joint%d'%i for i in range(1,7)];q=np.array(motion.q[:6]);fk=SerialUrdfFk(rospy.get_param('/robot_description'),names);A=fk(q);X=np.array(s['T_tool_camera_historical']);Y=np.array(s['T_base_board_initial_estimate']);n=Y[:3,2];n=n if n[2]>0 else -n;d=-n@Y[:3,3];K=s['camera_intrinsics']
g=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003);corners=np.array([[x,y,0,1] for x in [0,.18] for y in [0,.135]])
def guard(target):
 for alpha in np.linspace(0,1,9):
  T=fk(np.array(motion.q[:6])*(1-alpha)+np.array(target[:6])*alpha);clearance=min(float(np.min(_box_corners(center,T[:3,:3],size)@n+d)) for _,center,size in _stage_boxes(T,g,motion.q[6],'y','z'));b=(np.linalg.inv(T@X)@Y@corners.T).T;uv=np.c_[K['cx']-K['fx']*b[:,1]/b[:,0],K['cy']-K['fy']*b[:,2]/b[:,0]];edge=min(uv[:,0].min(),639-uv[:,0].max(),uv[:,1].min(),479-uv[:,1].max())
  if clearance<.04 or edge<20:raise ValueError('view/clearance reservation: %.4fm %.1fpx'%(clearance,edge))
motion.command_guard=guard
target=A.copy();target[:3,3]+=(A@X)[:3,0]*.02
def fun(x):
 T=fk(x);return np.r_[(T[:3,3]-target[:3,3])*10,cv2.Rodrigues(target[:3,:3].T@T[:3,:3])[0].reshape(3)]
x=q.copy();lo=np.maximum(q-np.radians(15),fk.limits[:,0]);hi=np.minimum(q+np.radians(15),fk.limits[:,1])
for iteration in range(120):
 f=fun(x)
 if np.linalg.norm(f)<1e-7:break
 jac=np.column_stack([(fun(x+np.eye(6)[j]*1e-5)-f)/1e-5 for j in range(6)]);step=-np.linalg.solve(jac.T@jac+1e-5*np.eye(6),jac.T@f);step*=min(1,.05/max(np.max(np.abs(step)),1e-9));x=np.clip(x+step,lo,hi)
assert np.linalg.norm(fun(x))<1e-4
(p/('closer_view_execution_'+str(time.time_ns())+'.json')).write_text(json.dumps({'start_q':q.tolist(),'goal_q':x.tolist(),'translation_along_camera_m':.02,'handeye_acceptance_thresholds_unchanged':True,'pose_target_tolerance_deg':1.,'reason':'Sampling uses measured pose; nominal pose tolerance is a motion scheduling rule, not calibration acceptance.'},indent=2)+'\n')
finished=set()
for cycle in range(24):
 remaining=x-np.array(motion.q[:6])
 if all(j in finished or abs(remaining[j])<math.radians(1) for j in range(6)):break
 moved=False
 for j in [2,1,4,3,5,0]:
  if j in finished or abs(remaining[j])<math.radians(1):continue
  delta=float(np.clip(math.degrees(remaining[j]),-4,4));goal=motion.q.copy();goal[j]+=math.radians(delta)
  try:guard(goal)
  except ValueError:continue
  motion.move(j,delta)
  if abs(delta)<4.:finished.add(j)
  moved=True;break
 if not moved:raise RuntimeError('No local segment fits reserved view and clearance')
else:raise RuntimeError('closer view iteration limit')
motion.emit('closer_view_reached',q_deg=np.degrees(motion.q[:6]).tolist())
for i in range(5):
 quality=json.loads(rospy.wait_for_message('/charuco/quality',String,timeout=5).data);print('QUALITY',quality['charuco_count'],quality['reprojection_rms_px'],flush=True);time.sleep(.2)
