#!/usr/bin/env python3
"""Read-only fresh RGB-D/TF/q single-axis candidate audit. No command publisher.
geometry(q) and guard_segment(q0,q1) are instance methods of GeometryAudit;
they can be reused by a separately authorized executor with a freshly captured
snapshot. Saved reports are never an execution authorization.
"""
import os,sys,json,time,math,threading,hashlib,signal
from pathlib import Path
from collections import deque
import numpy as np, cv2
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src/alicia_grasp_modes/src'),str(ROOT/'src/alicia_flexible_grasp_supervisor/src')]
from alicia_grasp_modes import tabletop as tt
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry,_stage_boxes,_box_corners
NAMES=['Joint%d'%i for i in range(1,7)]+['right_finger'];QUANTUM=2*math.pi/4096
OUT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_ros_latest_start_20261002/gen4_root_cause/single_axis_plan')
def write(path,data):path.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')
def qmsg(m):d=dict(zip(m.name,m.position));return np.array([d[n] for n in NAMES])
def words(q):return [int(math.floor((v+math.pi)/QUANTUM+.5)) for v in q[:6]]
class GeometryAudit:
 def __init__(self,snapshot,xml,validity=None):
  self.snapshot=snapshot;self.fk=SerialUrdfFk(xml,NAMES[:6]);self.X=np.array(snapshot['T_tool_camera']);self.plane=np.array(snapshot['plane_base']);self.K=np.array(snapshot['K']);self.width,self.height=snapshot['width'],snapshot['height'];self.lo=np.array(snapshot['joint_min']);self.hi=np.array(snapshot['joint_max']);self.openings=snapshot['openings'];self.target=np.array(snapshot['target_bbox_volume_corners_base']);self.texture=np.array(snapshot['texture_region_hull_base']);self.validity=validity;self.gripper=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003);self.cache={}
 def geometry(self,q,check_collision=True):
  q=np.asarray(q,dtype=float)[:6]
  if q.shape!=(6,) or not np.isfinite(q).all():raise ValueError('invalid q')
  tool=self.fk(q);T=tool@self.X
  def projection(P):
   cam=(P-T[:3,3])@T[:3,:3];depth=cam[:,0];uv=np.column_stack([self.K[2]-self.K[0]*cam[:,1]/depth,self.K[3]-self.K[1]*cam[:,2]/depth]);margin=float(min(uv[:,0].min(),self.width-1-uv[:,0].max(),uv[:,1].min(),self.height-1-uv[:,1].max()));return {'edge_px':margin,'forward_min_m':float(depth.min()),'forward_max_m':float(depth.max()),'uv_min':uv.min(0).tolist(),'uv_max':uv.max(0).tolist()}
  target=projection(self.target);texture=projection(self.texture);clearance=min(float(np.min(_box_corners(c,tool[:3,:3],size)@self.plane[:3]+self.plane[3])) for g in self.openings for _,c,size in _stage_boxes(tool,self.gripper,float(g),'y','z'));jointok=bool(np.all(q>=self.lo)&np.all(q<=self.hi));collision=None
  if check_collision and self.validity is not None:
   collision=True
   for g in self.openings:
    key=tuple(q)+(g,)
    if key not in self.cache:self.cache[key]=bool(self.validity(q,g))
    collision=collision and self.cache[key]
  passed=jointok and clearance>=.04 and target['edge_px']>=20 and texture['edge_px']>=20 and target['forward_min_m']>=.07 and target['forward_max_m']<=.50 and texture['forward_min_m']>.07 and collision is not False
  return {'q_rad':q.tolist(),'clearance_m':clearance,'target':target,'texture':texture,'joint_limits_pass':jointok,'moveit_collision_valid':collision,'geometry_pass':bool(passed),'T_base_camera':T.tolist()}
 def guard_segment(self,q0,q1,check_collision=True,step_deg=.15):
  q0=np.asarray(q0)[:6];q1=np.asarray(q1)[:6];n=max(2,int(math.ceil(float(np.max(abs(np.degrees(q1-q0))))/step_deg))+1);rows=[self.geometry(q0*(1-u)+q1*u,check_collision) for u in np.linspace(0,1,n)]
  return self.summarize(rows)
 @staticmethod
 def summarize(rows):return {'samples':len(rows),'minimum_clearance_m':min(r['clearance_m'] for r in rows),'minimum_target_margin_px':min(r['target']['edge_px'] for r in rows),'minimum_texture_margin_px':min(r['texture']['edge_px'] for r in rows),'minimum_target_forward_m':min(r['target']['forward_min_m'] for r in rows),'maximum_target_forward_m':max(r['target']['forward_max_m'] for r in rows),'all_geometry_pass':all(r['geometry_pass'] for r in rows),'all_moveit_valid':all(r['moveit_collision_valid'] is True for r in rows),'rows':rows}
def capture(directory):
 import rospy,tf2_ros,rosbag
 from cv_bridge import CvBridge
 from tf.transformations import quaternion_matrix
 from sensor_msgs.msg import Image,JointState
 from std_msgs.msg import Bool,String,Header
 from alicia_flexible_grasp_supervisor.msg import GraspState
 from tf2_msgs.msg import TFMessage
 state={};lock=threading.Lock();accepted=deque(maxlen=100);images={'rgb':{},'depth':{}};record=[]
 buffer=tf2_ros.Buffer(cache_time=rospy.Duration(10));listener=tf2_ros.TransformListener(buffer)
 def receive(topic,m):
  with lock:
   state[topic]=m
   if topic=='accepted':accepted.append(m)
   if topic in images:
    images[topic][m.header.stamp.to_nsec()]=m
    while len(images[topic])>8:images[topic].pop(next(iter(images[topic])))
   if topic not in images:record.append((topic,m,rospy.Time.now()))
 specs=[('rgb','/supervisor/camera/color/image_raw',Image),('depth','/supervisor/camera/depth/image_raw',Image),('accepted','/alicia_d/accepted_joint_states',JointState),('sdk','/alicia_d/sdk_command',JointState),('reference','/alicia_d/control_reference',JointState),('epoch','/alicia_d/control_reference_epoch',Header),('status','/alicia_d/actuation_status',String),('ready','/alicia_d/feedback_ready',Bool),('protected','/alicia_d/protection_latched',Bool),('manual','/gui/joint_direct_mode',Bool),('task','/grasp/state',GraspState)]
 subs=[rospy.Subscriber(topic,kind,lambda m,k=k:receive(k,m),queue_size=5,buff_size=2**24) for k,topic,kind in specs]
 deadline=time.monotonic()+8
 try:
  while time.monotonic()<deadline:
   with lock:
    paired=sorted(set(images['rgb'])&set(images['depth']));now=rospy.Time.now().to_nsec();recent=[m for m in accepted if now-m.header.stamp.to_nsec()<1000000000]
    good=paired and now-paired[-1]<300000000 and len(recent)>=7 and recent[-1].header.stamp.to_nsec()-recent[0].header.stamp.to_nsec()>=700000000 and all(k in state for k in ['sdk','reference','epoch','status','ready','protected','manual','task'])
    if good:rgb=images['rgb'][paired[-1]];depth=images['depth'][paired[-1]];frozen=dict(state);acc=list(recent);break
   time.sleep(.02)
  else:raise RuntimeError('fresh exact image pair / stationary window / status evidence unavailable')
  stamps=[m.header.stamp.to_nsec() for m in acc];qs=np.array([qmsg(m) for m in acc]);span=np.ptp(qs[:,:6],axis=0)
  if np.max(span)>math.radians(.15):raise RuntimeError('current accepted feedback not stationary')
  ns=rgb.header.stamp.to_nsec();near=min(acc,key=lambda m:abs(m.header.stamp.to_nsec()-ns));q=qmsg(near)
  if abs(near.header.stamp.to_nsec()-ns)>150000000:raise RuntimeError('source q age mismatch')
  def trans(parent,child):
   m=buffer.lookup_transform(parent,child,rgb.header.stamp,rospy.Duration(.3));p=m.transform.translation;r=m.transform.rotation;T=quaternion_matrix([r.x,r.y,r.z,r.w]);T[:3,3]=[p.x,p.y,p.z];return T,m
  T,tmsg=trans('base_link','camera_link');X,xmsg=trans('tool0','camera_link');xml=rospy.get_param('/robot_description');params=rospy.get_param('/camera');K=[params[k] for k in ['fx','fy','cx','cy']];bridge=CvBridge();im=bridge.imgmsg_to_cv2(rgb,'bgr8');dm=np.array(bridge.imgmsg_to_cv2(depth,'passthrough'),dtype=np.float32)*params['depth_scale']
  # Preserve failed source geometry as well. A segmentation rejection must not
  # erase the exact image/depth/TF evidence needed to diagnose the rejection.
  cv2.imwrite(str(directory/'source_rgb.png'),im)
  np.savez_compressed(directory/'source_raw.npz',rgb=im,depth_m=dm,T_base_camera=T,T_tool_camera=X,q=q,K=K,source_stamp_ns=np.int64(ns))
  write(directory/'source_binding.json',dict(source_stamp_ns=ns,accepted_source_stamp_ns=near.header.stamp.to_nsec(),q_rad=q.tolist(),camera_parameters=params))
  cfg=tt.Config();seg=tt.segment(dm,K,T,cfg)
  if not seg.candidates:raise RuntimeError('no current measured target candidate')
  candidates=sorted(seg.candidates,key=lambda c:np.linalg.norm(c.center_uv-np.array([im.shape[1]/2,im.shape[0]/2])));target=candidates[0]
  if np.linalg.norm(target.center_uv-np.array([im.shape[1]/2,im.shape[0]/2]))>min(im.shape[:2])*.125:raise RuntimeError('no unambiguous central measured target')
  if len(candidates)>1 and abs(np.linalg.norm(candidates[1].center_uv-np.array([im.shape[1]/2,im.shape[0]/2]))-np.linalg.norm(target.center_uv-np.array([im.shape[1]/2,im.shape[0]/2])))<min(im.shape[:2])*.045:raise RuntimeError('ambiguous central target')
  mask=target.mask>0;depths=dm[mask];x,y,w,h=target.bbox;uv=np.array([[u,v] for u in [x,x+w-1] for v in [y,y+h-1] for _ in range(2)]);dz=np.tile([float(depths.min()),float(depths.max())],4);corners=tt._project_pixels(dz,uv[:,0],uv[:,1],K)@T[:3,:3].T+T[:3,3]
  points=tt._project(dm,K);pbase=points@T[:3,:3].T+T[:3,3];plane=seg.plane_base.copy();plane=plane if plane[2]>0 else -plane;yy,xx=np.indices(dm.shape);rad=np.hypot(xx-target.center_uv[0],yy-target.center_uv[1]);support=(abs(pbase@plane[:3]+plane[3])<=cfg.support_threshold_m)&(rad>=70)&(rad<=140)&(dm>.07)&(dm<.65)&np.isfinite(dm)
  corners_uv=cv2.goodFeaturesToTrack(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),100,.01,8,mask=support.astype(np.uint8)*255)
  if corners_uv is None or len(corners_uv)<20:raise RuntimeError('insufficient fresh tabletop texture features')
  hull=cv2.convexHull(corners_uv).reshape(-1,2);hu=np.rint(hull[:,0]).astype(int);hv=np.rint(hull[:,1]).astype(int);texture=pbase[hv,hu];sdkmap=dict(zip(frozen['sdk'].name,frozen['sdk'].position));ref=qmsg(frozen['reference']);sdk=np.array([sdkmap[n] for n in NAMES[:6]]+[ref[6]])
  if words(sdk)!=words(ref):raise RuntimeError('SDK/reference command mismatch')
  fk=SerialUrdfFk(xml,NAMES[:6]);fkerror=float(np.max(abs(fk(q[:6])@X-T)))
  if fkerror>1e-5:raise RuntimeError('current actual FK and exact source TF disagree')
  snapshot={'kind':'fresh_readonly_single_axis_geometry_snapshot','source_stamp_ns':ns,'accepted_source_stamp_ns':near.header.stamp.to_nsec(),'snapshot_received_ros_ns':rospy.Time.now().to_nsec(),'q_rad':q.tolist(),'sdk_q_rad':sdk.tolist(),'reference_frame_id':frozen['reference'].header.frame_id,'epoch_ns':frozen['epoch'].stamp.to_nsec(),'actuation_status':frozen['status'].data,'feedback_ready':frozen['ready'].data,'protection_latched':frozen['protected'].data,'gui_direct_mode':frozen['manual'].data,'task_active':frozen['task'].active,'task_state':str(frozen['task'].state),'accepted_window_span_deg':np.degrees(span).tolist(),'T_base_camera':T.tolist(),'T_tool_camera':X.tolist(),'plane_base':plane.tolist(),'plane_fit':seg.metrics,'target_bbox_xywh':list(target.bbox),'target_center_uv':target.center_uv.tolist(),'target_extent_m':target.extent_m.tolist(),'target_bbox_volume_corners_base':corners.tolist(),'texture_feature_count':len(corners_uv),'texture_region_hull_uv':hull.tolist(),'texture_region_hull_base':texture.tolist(),'K':K,'width':im.shape[1],'height':im.shape[0],'depth_scale':params['depth_scale'],'joint_min':rospy.get_param('/robot/joint_min_rad'),'joint_max':rospy.get_param('/robot/joint_max_rad'),'openings':sorted(set([float(q[6]),float(ref[6])])),'urdf_sha256':hashlib.sha256(xml.encode()).hexdigest(),'fk_tf_max_abs_error':fkerror,'target_scope':'Fresh central measured geometry for this diagnostic; no reuse of previous generation identity or plane.'}
  cv2.imwrite(str(directory/'source_rgb.png'),im);cv2.imwrite(str(directory/'source_target_mask.png'),target.mask);np.savez_compressed(directory/'source.npz',rgb=im,depth_m=dm,T_base_camera=T,T_tool_camera=X,plane_base=plane,q=q,sdk=sdk,target_corners_base=corners,texture_points_base=texture);write(directory/'snapshot.json',snapshot);(directory/'robot_description.urdf').write_text(xml);write(directory/'camera_parameters.json',params)
  with rosbag.Bag(str(directory/'source_evidence.bag'),'w') as bag:
   for k,m,stamp in record:bag.write('/single_axis_readonly/'+k,m,stamp)
   bag.write('/supervisor/camera/color/image_raw',rgb,rgb.header.stamp);bag.write('/supervisor/camera/depth/image_raw',depth,depth.header.stamp);bag.write('/tf',TFMessage([tmsg]),rgb.header.stamp);bag.write('/tf_static',TFMessage([xmsg]),rgb.header.stamp)
  return snapshot,xml
 finally:
  for sub in subs:sub.unregister()
def plan(snapshot,xml,validity):
 audit=GeometryAudit(snapshot,xml,validity);actual=np.array(snapshot['q_rad']);reference=np.array(snapshot['sdk_q_rad']);results=[];count=int(math.floor(math.radians(3)/QUANTUM))
 for joint in [5,2]:
  index=joint-1;reserve=[]
  for bridge in [0,.5,1]:
   held=actual[:6]*(1-bridge)+reference[:6]*bridge
   for angle in np.linspace(actual[index]-math.radians(4.8),actual[index]+math.radians(4.8),65):
    q=held.copy();q[index]=angle;reserve.append(audit.geometry(q))
  reservation=audit.summarize(reserve)
  for sign in [1,-1]:
   target=reference.copy();target[index]+=sign*count*QUANTUM;expected=np.zeros(6,dtype=int);expected[index]=sign*count
   if not np.array_equal(np.array(words(target))-np.array(words(reference)),expected):raise RuntimeError('command word changed on held axes')
   outbound=audit.guard_segment(actual,target);ret=audit.guard_segment(target,reference)
   results.append({'joint':joint,'sign':sign,'nominal_requested_deg':3*sign,'quantized_step_deg':math.degrees(sign*count*QUANTUM),'sdk_target_rad':target.tolist(),'return_sdk_rad':reference.tolist(),'sdk_target_words':words(target),'return_sdk_words':words(reference),'outbound':outbound,'return':ret,'reservation':reservation,'geometry_and_collision_pass':outbound['all_geometry_pass'] and ret['all_geometry_pass'] and reservation['all_geometry_pass'] and outbound['all_moveit_valid'] and ret['all_moveit_valid'] and reservation['all_moveit_valid']})
 return {'kind':'no_motion_single_axis_audit','source_stamp_ns':snapshot['source_stamp_ns'],'robot_commands_published':0,'candidates':results,'collision_unique_state_checks':len(audit.cache),'limitations':['This sampled geometric audit expires immediately as execution authority; executor must reacquire fresh image/TF/q and authority.','Target volume bounds only its currently observed surface; no unseen geometry fabricated.','Texture hull consists of currently measured RGB feature pixels on current accepted plane; future feature quality is predicted only.','Collision uses current MoveIt scene per sampled state; scene completeness and unsampled tracking errors are not certified.','Reservation is +/-4.8 degrees about initial actual edited axis, with actual-to-SDK held-axis bridge at 0/.5/1.','No GUI ownership, enable or task-state change was made.']}
def main():
 if not Path('/media/zhuyupei/SanStand').is_mount():raise RuntimeError('external experiment drive unavailable')
 OUT.mkdir(parents=True,exist_ok=True);run=OUT/time.strftime('%Y%m%dT%H%M%SZ',time.gmtime());run.mkdir();(run/'ros').mkdir();os.environ['ROS_LOG_DIR']=str(run/'ros')
 import rospy
 from sensor_msgs.msg import JointState
 from moveit_msgs.msg import RobotState
 from moveit_msgs.srv import GetStateValidity,GetStateValidityRequest
 rospy.init_node('gen4_single_axis_readonly_plan',anonymous=True,disable_signals=True);cv2.setNumThreads(1);signal.signal(signal.SIGALRM,lambda *_:(_ for _ in ()).throw(RuntimeError('readonly plan 55 second deadline')));signal.alarm(55)
 try:
  snapshot,xml=capture(run);rospy.wait_for_service('/check_state_validity',timeout=3);proxy=rospy.ServiceProxy('/check_state_validity',GetStateValidity)
  def valid(q,g):
   req=GetStateValidityRequest(group_name='alicia');req.robot_state=RobotState();req.robot_state.joint_state=JointState(name=NAMES,position=list(q)+[g]);return proxy(req).valid
  result=plan(snapshot,xml,valid);write(run/'plan.json',result);write(OUT/'latest.json',{'directory':str(run),'source_stamp_ns':snapshot['source_stamp_ns']});print(json.dumps({'directory':str(run),'snapshot':{k:snapshot[k] for k in ['source_stamp_ns','q_rad','plane_base','plane_fit','target_bbox_xywh','texture_feature_count','actuation_status','gui_direct_mode','task_active']},'candidates':[{**{k:v for k,v in c.items() if k not in ['outbound','return','reservation']},'min_clearance_m':c['reservation']['minimum_clearance_m'],'min_target_margin_px':c['reservation']['minimum_target_margin_px'],'min_texture_margin_px':c['reservation']['minimum_texture_margin_px']} for c in result['candidates']]},indent=2))
 except BaseException as e:write(run/'failure.json',{'error':str(e),'robot_commands_published':0});raise
 finally:signal.alarm(0)

class PreparedExecutorAudit:
 """Freshly constructed review interface. Does not create command publishers."""
 def __init__(self,snapshot,xml,validity):
  self.snapshot=snapshot;self.xml=xml;self.audit=GeometryAudit(snapshot,xml,validity)
  self.source_q=np.asarray(snapshot['q_rad'])[:6].copy()
  self.steps=[(4,3.0),(4,None),(1,3.0),(1,None)]
  self.source_stamp_ns=snapshot['source_stamp_ns']
 def geometry(self,q):return self.audit.geometry(q)
 def guard_segment(self,q0,q1):
  import rospy
  if hashlib.sha256(rospy.get_param('/robot_description').encode()).hexdigest()!=self.snapshot['urdf_sha256']:
   raise RuntimeError('URDF changed since fresh geometry capture')
  camera=rospy.get_param('/camera')
  if not np.allclose([camera[k] for k in ['fx','fy','cx','cy']],self.snapshot['K'],rtol=0,atol=0):
   raise RuntimeError('camera K changed since geometry capture')
  from tf.transformations import quaternion_matrix
  handeye=rospy.get_param('/handeye');x=quaternion_matrix(handeye['rotation_xyzw']);x[:3,3]=handeye['translation_xyz']
  if not np.allclose(x,self.snapshot['T_tool_camera'],rtol=0,atol=1e-9):
   raise RuntimeError('handeye changed since exact source TF capture')
  q0=np.asarray(q0)[:6];q1=np.asarray(q1)[:6]
  if max(float(np.max(abs(q0-self.source_q))),float(np.max(abs(q1-self.source_q))))>math.radians(4.8)+1e-9:
   raise RuntimeError('segment outside fixed initial actual +/-4.8 degree experiment envelope')
  # Collision answers must come from the current scene on every submission.
  self.audit.cache.clear();result=self.audit.guard_segment(q0,q1)
  if not result['all_geometry_pass'] or not result['all_moveit_valid']:
   raise RuntimeError('fresh submission segment violates measured geometry / current MoveIt: '+json.dumps({k:v for k,v in result.items() if k!='rows'}))
  return result

def prepare_executor_audit(directory):
 """Call only after rospy init, before any ownership or motion changes.
 Caller owns fresh authority, epoch, actual-feedback and source freshness checks
 at every submission, including recovery handshake publications. This helper
 deliberately does not implement enable/sync/command/return behavior.
 """
 import rospy
 from sensor_msgs.msg import JointState
 from moveit_msgs.msg import RobotState
 from moveit_msgs.srv import GetStateValidity,GetStateValidityRequest
 directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
 snapshot,xml=capture(directory)
 if snapshot['task_active'] or snapshot['protection_latched'] or not snapshot['feedback_ready']:
  raise RuntimeError('fresh task/feedback/protection evidence not eligible')
 rospy.wait_for_service('/check_state_validity',timeout=3);proxy=rospy.ServiceProxy('/check_state_validity',GetStateValidity)
 def valid(q,g):
  req=GetStateValidityRequest(group_name='alicia');req.robot_state=RobotState();req.robot_state.joint_state=JointState(name=NAMES,position=list(q)+[g]);return proxy(req).valid
 planned=plan(snapshot,xml,valid);write(directory/'plan.json',planned)
 if not all(c['geometry_and_collision_pass'] for c in planned['candidates']):
  raise RuntimeError('one or more requested single-axis reservations failed')
 return PreparedExecutorAudit(snapshot,xml,valid)

if __name__=='__main__':main()
