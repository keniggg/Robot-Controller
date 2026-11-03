from pathlib import Path
import json,hashlib
import numpy as np
import cv2,rosbag,rospy
from alicia_grasp_modes.observation_arrival_projection import capture_arrival_projection
from alicia_flexible_grasp.robot.observation_view_guard import _project
root=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/gen20_analysis/arrival_projection')
data=json.loads((root/'planned_actual_projection.json').read_text())
row=next(x for x in data['frames'] if x['source_stamp_ns']==1791622072921012163);ns=row['source_stamp_ns'];camera=data['camera'];status=data['source_status'];selection={'mode':'unknown','strategy':'observe_return','generation':20,'stamp_ns':1791621970669396162}
contract=capture_arrival_projection(status,selection,status['source_stamp_ns'],camera)
found=[]
with rosbag.Bag(str(Path(data['record'])/'keyframes.bag')) as bag:
 for topic,m,t in bag.read_messages(topics=['/experiment/observation_rgbd/color','/supervisor/camera/color/image_raw'],start_time=rospy.Time.from_sec(ns*1e-9-.3),end_time=rospy.Time.from_sec(ns*1e-9+1.)):
  if m.header.stamp.to_nsec()==ns:
   a=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3).copy()
   if m.encoding=='rgb8':a=cv2.cvtColor(a,cv2.COLOR_RGB2BGR)
   elif m.encoding!='bgr8':raise ValueError(m.encoding)
   found.append((topic,t.to_nsec(),a))
assert found
original=found[0][2];h,w=original.shape[:2];cv2.imwrite(str(root/'arrival_source_original.png'),original)
canvas=cv2.copyMakeBorder(original,55,55,45,80,cv2.BORDER_CONSTANT,value=(35,35,35));shift=np.array([45,55]);valid=contract['rgb_valid'];masked=canvas[55:55+h,45:45+w];masked[~valid]=(masked[~valid]*.3+np.array([255,0,255])*.7).astype(np.uint8)
K=np.asarray([camera[k] for k in ('fx','fy','cx','cy')]);uv,depth=_project(contract['prism_base'],np.asarray(row['T_base_camera']),K);hull=cv2.convexHull(np.rint(uv+shift).astype(np.int32));cv2.polylines(canvas,[hull],True,(0,230,255),2)
n=len(uv)//2
for i in range(n):
 cv2.line(canvas,tuple(np.rint(uv[i]+shift).astype(int)),tuple(np.rint(uv[i+n]+shift).astype(int)),(0,230,255),1)
for bounds,color,label in [(row['projection']['bbox'],(0,190,255),'prism bbox'),(row['projection']['margin_4']['bounds'],(0,0,255),'required +4px')]:
 x0,y0,x1,y1=bounds;cv2.rectangle(canvas,(x0+45,y0+55),(x1+45,y1+55),color,1)
cv2.rectangle(canvas,(45,55),(45+w-1,55+h-1),(255,255,255),1)
for y,text,color in [(18,'Exact RGB + bag TF; yellow=reference prism; red=required ROI',(230,230,230)),(40,'Magenta=invalid RGB; white=image canvas (653x489)',(230,230,230))]:cv2.putText(canvas,text,(8,y),cv2.FONT_HERSHEY_SIMPLEX,.44,color,1,cv2.LINE_AA)
cv2.putText(canvas,'source '+str(ns),(8,canvas.shape[0]-18),cv2.FONT_HERSHEY_SIMPLEX,.43,(230,230,230),1,cv2.LINE_AA)
cv2.imwrite(str(root/'arrival_source_projection_overlay.png'),canvas)
report={'source_stamp_ns':ns,'rgb_matches':[{'topic':x[0],'received_ns':x[1]} for x in found],'identical_rgb_matches':all(np.array_equal(x[2],original) for x in found),'actual_uv':uv.tolist(),'prism_depth_m':depth.tolist(),'frozen_measured_height_m':contract['audit']['measured_height_m'],'reference_footprint':status['reference_support_footprint_base'],'actual_projected_bbox':row['projection']['bbox'],'actual_required_box':row['projection']['margin_4']['bounds'],'files':{},'limitations':['Reference prism and its rectangular search box are not the measured RGB object silhouette.','Bag TF is a kinematic estimate, not independent calibration or physical pose verification.','This image alone does not distinguish reference shape overestimate from depth/kinematic/calibration errors.']}
for name in ('arrival_source_original.png','arrival_source_projection_overlay.png'):
 p=root/name;report['files'][name]={'path':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
p=root/'image_binding.json';p.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
