from pathlib import Path
import hashlib,json,math
from collections import Counter
import numpy as np
import rosbag,rospy,tf2_ros
from tf.transformations import quaternion_matrix
from alicia_grasp_modes.observation_arrival_projection import capture_arrival_projection,validate_arrival_projection
from alicia_flexible_grasp.robot.observation_view_guard import _project
ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller');NEW=ROOT/'ros_log/full_latest_restart_20261010T002347Z';REC=ROOT/'grasp_records/20261010T084609.133186Z_f832fa3e55eb45a59d41fecf2246af73';OUT=NEW/'gen20_analysis/arrival_projection';OUT.mkdir(parents=True,exist_ok=True)
auditpath=REC/'keyframes_audits/f3c7dd20ffb7481a957173d2429e59d47a38bd0b42d0327b320fbdf2de92afed.json';audit=json.loads(auditpath.read_text());selected=audit['selected'];source=audit['snapshot_stamp_ns'];buffer=tf2_ros.Buffer(cache_time=rospy.Duration(300),debug=False);statuses=[];meta=[];source_status=None
with rosbag.Bag(str(REC/'keyframes.bag')) as bag:
 for topic,m,t in bag.read_messages(topics=['/tf','/tf_static','/grasp_record/camera_runtime_metadata','/perception/unknown/detector_status']):
  if topic in ('/tf','/tf_static'):
   for x in m.transforms:(buffer.set_transform_static if topic=='/tf_static' else buffer.set_transform)(x,'sealed_gen20')
  elif topic=='/grasp_record/camera_runtime_metadata':meta.append(json.loads(m.data))
  else:
   d=json.loads(m.data)
   if d.get('source_stamp_ns')==source and d.get('state')=='ready':source_status=d
   if 1791622071990000000<=t.to_nsec()<=1791622075200000000:statuses.append(dict(received_ns=t.to_nsec(),payload=d))
assert source_status and meta
camera=meta[-1]['parameters'];K=np.asarray([camera[k] for k in ['fx','fy','cx','cy']]);w,h=camera['width'],camera['height'];selection=selected['observation_source_binding']['selection'] if 'selection' in selected.get('observation_source_binding',{}) else {'mode':'unknown','strategy':'observe_return','generation':20,'stamp_ns':1791621970669396162}
contract=capture_arrival_projection(source_status,selection,source,camera);assert contract['audit']==selected['observation_source_audit']['arrival_projection']
following=next(x['following_support']['unknown_observation_view'] for x in selected['observation_orientation_search']['variant_results'] if x.get('following_support',{}).get('unknown_observation_view',{}).get('ok'))
planned=np.asarray(following['target_trajectory']['endpoint_transform_base_camera']);valid=contract['rgb_valid'];rows=[]
def project(T):
 uv,depth=_project(contract['prism_base'],T,K);bounds=[*np.floor(uv.min(0)).astype(int),*(np.ceil(uv.max(0)).astype(int)+1)];r=dict(uv_min=uv.min(0).tolist(),uv_max=uv.max(0).tolist(),bbox=list(map(int,bounds)),edge=float(min(uv.min(),w-1-uv[:,0].max(),h-1-uv[:,1].max())),min_depth_m=float(depth.min()),max_depth_m=float(depth.max()))
 for margin in (0,4,16):
  x0,y0,x1,y1=np.array(bounds)+np.array([-margin,-margin,margin,margin]);clipped=(max(x0,0),max(y0,0),min(x1,w),min(y1,h));a,b,c,d=clipped;r['margin_%d'%margin]=dict(bounds=list(map(int,(x0,y0,x1,y1))),outside=bool(x0<0 or y0<0 or x1>w or y1>h),unobserved=int(np.count_nonzero(~valid[b:d,a:c])))
 try:r['guard']=validate_arrival_projection(contract,T,K,w,h)
 except ValueError as e:r['guard']={'ok':False,'reason':str(e)}
 return r
for row in statuses:
 d=row['payload'];obs=d.get('observation_measurement',{});ns=d.get('source_stamp_ns')
 if not ns or not 1791622071999516487<ns<1791622074999516487:continue
 tr=buffer.lookup_transform('base_link','camera_link',rospy.Time(ns//10**9,ns%10**9)).transform;T=quaternion_matrix([tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w]);T[:3,3]=[tr.translation.x,tr.translation.y,tr.translation.z]
 angle=math.degrees(math.acos(float(np.clip((np.trace(planned[:3,:3].T@T[:3,:3])-1)/2,-1,1))))
 row.update(reason=obs.get('reason'),source_stamp_ns=ns,current_age_sec=(row['received_ns']-ns)*1e-9,T_base_camera=T.tolist(),projection=project(T),camera_endpoint_error=dict(position_m=float(np.linalg.norm(T[:3,3]-planned[:3,3])),orientation_deg=angle))
 rows.append(row)
(out:=OUT/'planned_actual_projection.json').write_text(json.dumps(dict(record=str(REC),audit_path=str(auditpath),audit_sha256=hashlib.sha256(auditpath.read_bytes()).hexdigest(),planned_transform=planned.tolist(),planned_projection=project(planned),camera=camera,source_status=source_status,source_contract_audit=contract['audit'],frames=rows,restrictions={'ros_access':False,'hardware_actions':False},limitations=['Exact source bag TF is recorded kinematic geometry; not independent physical calibration verification.','Projected frozen prism only; per-frame first16 seed crop is not yet reconstructed.']),indent=2)+'\n')
print(out,hashlib.sha256(out.read_bytes()).hexdigest());print('planned', {k:v for k,v in project(planned).items() if k!='guard'});print('counts',dict(Counter(x['reason'] for x in rows)))
for x in rows:print(x['source_stamp_ns'],x['reason'],x['projection']['bbox'],x['projection']['margin_4'],x['camera_endpoint_error'])
