#!/usr/bin/env python3
import sys,json,time,math,itertools,os
from pathlib import Path
import numpy as np
import rospy
from std_msgs.msg import String
from easy_handeye_msgs.srv import TakeSample
from handeye_auto_motion import AutoMotion
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.grasp.gripper_geometry import (GripperGeometry,_stage_boxes,_box_corners)
ROOT=Path(__file__).resolve().parents[1]
config_path=Path(os.environ.get('HANDEYE_AUTOMATION_CONFIG',str(ROOT/'.runtime/current_handeye_autocalibration.json')))
config=json.loads(config_path.read_text());OUT=Path(config['session'])
rospy.init_node('automatic_handeye_collection',anonymous=True,disable_signals=True)
motion=AutoMotion(OUT);names=['Joint%d'%i for i in range(1,7)];fk=SerialUrdfFk(rospy.get_param('/robot_description'),names)
start=json.loads((OUT/'starting_geometry.json').read_text());X=np.array(start['T_tool_camera_historical']);Y=np.array(start['T_base_board_initial_estimate']);K=start['camera_intrinsics']
seed=np.array(motion.q[:6]);normal=Y[:3,2].copy()
if normal[2]<0:normal=-normal
planeoffset=-normal@Y[:3,3]
gripper=GripperGeometry(.05,.002,np.array([.0434,.0286,.060]),np.array([.1175,.155,.0774]),.003)
latest={}
def quality(msg):
 try:latest.update(json.loads(msg.data));latest['received']=time.monotonic()
 except Exception:pass
sub=rospy.Subscriber('/charuco/quality',String,quality,queue_size=1)
boardcorners=np.array([[x,y,0,1] for x in [0,.18] for y in [0,.135]])
def geometry(q):
 T=fk(q);clearance=min(float(np.min(_box_corners(center,T[:3,:3],size)@normal+planeoffset)) for _,center,size in _stage_boxes(T,gripper,motion.q[6],'y','z'))
 B=np.linalg.inv(T@X)@Y;points=(B@boardcorners.T).T
 if np.min(points[:,0])<.12:return False,{'reason':'board too close/behind camera'}
 uv=np.column_stack([K['cx']-K['fx']*points[:,1]/points[:,0],K['cy']-K['fy']*points[:,2]/points[:,0]])
 edge=float(min(uv[:,0].min(),639-uv[:,0].max(),uv[:,1].min(),479-uv[:,1].max()))
 return clearance>=.04 and edge>=20,{'minimum_gripper_plane_clearance_m':clearance,'predicted_full_board_edge_px':edge}
def guard_segment(target):
 for alpha in np.linspace(0,1,9):
  ok,details=geometry(np.array(motion.q[:6])*(1-alpha)+np.array(target[:6])*alpha)
  if not ok:raise RuntimeError('observation segment outside reserved clearance/view: '+json.dumps(details))
motion.command_guard=guard_segment
# Fixed prospective pose family; no fits or residuals used to choose these poses.
offsets=[]
for axis,angle in [(3,12),(4,8),(5,16),(0,4),(1,4),(2,4)]:
 for sign in ([1,-1] if axis not in (1,2) else [1]):
  q=np.zeros(6);q[axis]=sign*angle;offsets.append(q)
for a,b,da,db in [(3,4,10,6),(3,5,10,12),(4,5,6,12)]:
 for sa,sb in itertools.product([1,-1],repeat=2):
  q=np.zeros(6);q[a]=sa*da;q[b]=sb*db;offsets.append(q)
plan=[]
for i,delta in enumerate(offsets):
 q=seed+np.radians(delta);ok,detail=geometry(q)
 plan.append({'pose_id':i+1,'offset_deg':delta.tolist(),'q_rad':q.tolist(),'geometry_eligible':bool(ok),'geometry':detail})
if len(sys.argv)>2:
 loaded=json.loads(Path(sys.argv[2]).read_text());plan=loaded['poses'];seed=np.array(loaded['seed_q_rad'])
 for item in plan:
  if 'geometry_eligible' not in item:
   eligible,detail=geometry(item['q_rad']);item['geometry_eligible']=bool(eligible);item['geometry']=detail
(OUT/('prospective_training_plan_'+str(time.time_ns())+'.json')).write_text(json.dumps({'seed_q_rad':seed.tolist(),'source_plan_fixed_before_training_fit':True,'candidate_fit_groups':'none; frozen-candidate validation only' if 'training_session' in config else 'all accepted training groups','reserved_validation':'separate future session after candidate freeze','nominal_clearance_reserve_m':.04,'clearance_model_caveat':'Uses historical hand-eye only for coarse view/clearance reservation; does not certify its accuracy. Robot collision constraints and live image/feedback remain active.','poses':plan},indent=2)+'\n')
print('PLAN',len(plan),'eligible',sum(p['geometry_eligible'] for p in plan),flush=True)
sample=rospy.ServiceProxy(config['namespace']+'/take_sample',TakeSample)
try:
 for item in plan:
  if item['pose_id']<(int(sys.argv[1]) if len(sys.argv)>1 else 1):continue
  if not item['geometry_eligible']:
   motion.emit('pose_not_scheduled',**item);continue
  motion.emit('pose_begin',pose_id=item['pose_id'],offset_deg=item['offset_deg'])
  goal=np.array(item['q_rad'])
  for j in [1,2,0,3,4,5]:
   steps=0
   while abs(motion.q[j]-goal[j])>math.radians(1.):
    if steps>=12:raise RuntimeError('joint steps limit')
    observation_deadline=time.monotonic()+4
    while (not latest or time.monotonic()-latest.get('received',0)>1.2 or not latest.get('pose_ok')) and time.monotonic()<observation_deadline:
     motion.guard();time.sleep(.1)
    if not latest or time.monotonic()-latest.get('received',0)>1.2 or not latest.get('pose_ok'):raise RuntimeError('current board observation unavailable after settling wait')
    remaining_deg=math.degrees(goal[j]-motion.q[j]);final_segment=abs(remaining_deg)<=4.+1e-9
    delta=float(np.clip(remaining_deg,-4,4));motion.move(j,delta);steps+=1
    if final_segment:break
  accepted=False
  for attempt in range(3):
   motion.guard();time.sleep(.5)
   try:
    response=sample();motion.emit('pose_sample_accepted',pose_id=item['pose_id'],count=len(response.samples.hand_world_samples),attempt=attempt+1);accepted=True;break
   except rospy.ServiceException as exc:motion.emit('pose_sample_rejected',pose_id=item['pose_id'],attempt=attempt+1,reason=str(exc))
  if not accepted:motion.emit('pose_no_accepted_sample',pose_id=item['pose_id'])
 motion.emit('collection_sequence_complete')
 (OUT/('collection_terminal_'+str(time.time_ns())+'.json')).write_text(json.dumps({'status':'sequence_complete','at':time.time()},indent=2)+'\n')
except Exception as exc:
 motion.emit('collection_interrupted',reason=str(exc));(OUT/('collection_terminal_'+str(time.time_ns())+'.json')).write_text(json.dumps({'status':'interrupted','at':time.time(),'reason':str(exc)},indent=2)+'\n');raise
