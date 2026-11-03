"""Exact closed-record nearfield RGBD qualification and local edge diagnostics."""
import hashlib,json,os,sys
from pathlib import Path
from collections import Counter
W=Path(__file__).resolve().parents[1]
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z')
R=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261015T092659.242714Z_911b220cf1d34113b7c80a7f97dad165')
O=N/'verification/gen28_attempt1_box_edges_r1'
assert Path('/media/zhuyupei/SanStand').is_mount() and not O.exists();O.mkdir();(O/'tmp').mkdir();(O/'ros').mkdir()
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:1',TMPDIR=str(O/'tmp'),ROS_LOG_DIR=str(O/'ros'),OPENBLAS_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1')
sys.path.insert(0,str(W/'src/alicia_flexible_grasp_supervisor/src'))
import numpy as np,cv2,rosbag,rospy
from cv_bridge import CvBridge
from alicia_flexible_grasp.vision import regular_box_inference as box
from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 return h.hexdigest()
def arrsha(a):return hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()
def save(name,v):
 p=O/name;assert not p.exists();p.write_text(json.dumps(v,ensure_ascii=False,indent=2,default=lambda a:a.tolist() if isinstance(a,np.ndarray) else str(a))+'\n');return dict(path=str(p),sha256=sha(p))
manifest=json.loads((R/'manifest.json').read_text());assert manifest['state']=='complete' and manifest['recorder_closed']
ap=R/'keyframes_audits/6c4d904130bcb216658c6ce01f6d6d58186919071df1e53ac3fdd72d442399c8.json'
assert sha(ap)==ap.stem;audit=json.loads(ap.read_text());live=audit['remote_diagnostics']['tabletop_geometry']['regular_box_inference']
ns=live['source_stamp_ns'];assert ns==1792056475938090324
topics=['/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw','/perception/object_mask','/perception/object']
frames={};cameras=[]
def tm(ns):return rospy.Time(*divmod(ns,10**9))
bagsha=sha(R/'keyframes.bag');assert bagsha==next(x['sha256'] for x in manifest['files'] if x['path']=='keyframes.bag')
with rosbag.Bag(str(R/'keyframes.bag')) as bag:
 for topic,msg,rec in bag.read_messages(topics=topics,start_time=tm(ns-100_000_000),end_time=tm(ns+2_000_000_000)):
  if msg.header.stamp.to_nsec()==ns:frames[topic]=(msg,rec.to_nsec())
 for _,msg,rec in bag.read_messages(topics=['/grasp_record/camera_runtime_metadata']):
  value=json.loads(msg.data)
  if value.get('parameters'):cameras.append((rec.to_nsec(),value['parameters']))
assert set(frames)==set(topics)
camera_receipt,camera=max((x for x in cameras if x[0]<=frames[topics[0]][1]),key=lambda x:x[0])
bridge=CvBridge();color=bridge.imgmsg_to_cv2(frames[topics[0]][0],'bgr8');depth=bridge.imgmsg_to_cv2(frames[topics[1]][0],'passthrough');mask=bridge.imgmsg_to_cv2(frames[topics[2]][0],'mono8')
T=np.asarray(audit['snapshot_transform']['T_base_optical']);K=np.array([camera[k] for k in ('fx','fy','cx','cy')]);scale=camera['depth_scale']
assert audit['snapshot_transform']['snapshot_stamp_ns']==ns
assert arrsha(color)==live['retained_pixel_sha256']['color_sha256'] and arrsha(depth)==live['retained_pixel_sha256']['depth_sha256'] and arrsha(mask)==live['retained_pixel_sha256']['mask_sha256']
g=audit['replay_geometry'];assert arrsha(np.asarray(g['object_points_base_m'],float))==g['object_points_sha256']
geometry=GeometryEstimate(True,'','',g['obb_center_base_m'],g['R_base_obb'],g['obb_size_xyz_m'],g['support_normal_base'],g['support_offset_m'],g['support_inlier_ratio'],g['object_points_base_m'],g['source_mode'])
identity=TargetTrackIdentity(**live['source_target_identity']);obj=frames[topics[3]][0]
npz=O/('source_%d.npz'%ns);np.savez_compressed(str(npz),color=color,depth_raw=depth,object_mask=mask,T_base_optical=T,intrinsics=K,depth_scale_m=np.array(scale),source_stamp_ns=np.array(ns),bbox_xywh=np.array([obj.bbox_x,obj.bbox_y,obj.bbox_width,obj.bbox_height]))
cv2.imwrite(str(O/('source_%d_color.png'%ns)),color);cv2.imwrite(str(O/('source_%d_mask.png'%ns)),mask)
save('geometry.json',g)
top={};visual={};rejections=[];edgechecks=[]
def trace(f,event,arg):
 if f.f_code is box.qualify_regular_box_top.__code__ and event=='return':top.update(f.f_locals)
 if f.f_code is box._visual_rectangle.__code__:
  v=f.f_locals
  if event=='line' and f.f_lineno==391:
   rejections.append(dict(reason='corner_mask',corner_coverage=list(v['corner_cov']),projected=np.asarray(v['projected']).tolist()))
  if event=='line' and f.f_lineno==418:
   edgechecks.append(dict(projected=np.asarray(v['projected']).tolist(),edge_index=v['i'],visual_support=v['visual_support'],valid_inside=v['valid_inside'],supported=v['supported'],
    valid_outside=v['valid_outside'],continued=v['continued'],occluder_fraction=float(np.mean(v['outside'][0]&(v['outside'][2]>.0025))),mask_border_fraction=v['mask_border_fraction'],
    inside_signed_residual_m=v['inside'][2][v['inside'][0]].tolist(),outside_signed_residual_m=v['outside'][2][v['outside'][0]].tolist()))
  if event=='line' and f.f_lineno==422:
   row=dict(edgechecks[-1]);row['reason']='edge_depth_or_visual';rejections.append(row)
  if event=='line' and f.f_lineno==431 and v['good_candidate'] and v['mask_boundary_count']<3:
   rejections.append(dict(reason='insufficient_mask_boundaries',count=v['mask_boundary_count'],projected=np.asarray(v['projected']).tolist()))
  if event=='return':visual.update(v)
 return trace
sys.settrace(trace)
try:result=box.qualify_regular_box_top(depth,mask,K,scale,T,geometry,identity,ns,color_bgr=color)
finally:sys.settrace(None)
matches={k:dict(actual=result.audit.get(k),recorded=live.get(k),equal=result.audit.get(k)==live.get(k)) for k in ('top_plane_rms_m','top_support_angle_deg','inferred_size_xyz_m','rectangle_fill_ratio','corner_coverage','edge_coverage')}
depth_edges=[];coord=top['coord'];corners=top['corners2'];tolerance=top['tolerance'];image_corners=box._project(corners@top['axes'][:,:2].T+top['n']*(top['top_height']-top['d']),T,K)
overlay=color.copy();cv2.polylines(overlay,[np.rint(image_corners).astype(np.int32)],True,(0,255,0),1)
for i in range(4):
 a,b=corners[i],corners[(i+1)%4];v=b-a;along=(coord-a)@v/float(v@v);dist=np.abs(np.cross(v,coord-a))/np.linalg.norm(v)
 bins=[]
 for j in range(10):
  choose=(along>=j/10)&(along<=(j+1)/10)
  bins.append(dict(index=j,point_count=int(np.count_nonzero(choose)),nearest_top_distance_m=float(np.min(dist[choose])) if np.any(choose) else None,occupied=bool(np.any(choose&(dist<=tolerance)))))
 depth_edges.append(dict(edge_index=i,projected_endpoints=image_corners[[i,(i+1)%4]].tolist(),tolerance_m=float(tolerance),bins=bins))
 cv2.putText(overlay,str(i),tuple(np.rint((image_corners[i]+image_corners[(i+1)%4])/2).astype(int)),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,0,255),1)
ys,xs=np.nonzero(mask);crop=overlay[max(0,ys.min()-20):min(color.shape[0],ys.max()+21),max(0,xs.min()-20):min(color.shape[1],xs.max()+21)]
cv2.imwrite(str(O/'metric_edges_overlay.png'),overlay);cv2.imwrite(str(O/'metric_edges_crop.png'),cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
np.savez_compressed(str(O/'qualifier_context.npz'),origin=top['origin'],tn=top['tn'],axes=top['axes'],lower=top['lower'],upper=top['upper'],top=top['top'],coord=coord,uv=top['uv'],keep=top['keep'],points=top['points'],pixel_size=np.array(top['pixel_size']))
source_paths=[Path(box.__file__),W/'.runtime/analyze_gen28_attempt1_box_edges.py']
report=dict(record=str(R),record_manifest_sha256=sha(R/'manifest.json'),record_bag_sha256=bagsha,record_closed_verified=True,actual_source_stamp_ns=ns,
 initial_source_stamp_ns=audit['observe_return']['initial_surface_stamp_ns'],stages_not_interchanged=True,source_npz=str(npz),source_npz_sha256=sha(npz),
 raw_hashes_match_live=True,audit_path=str(ap),audit_sha256=sha(ap),actual_live_box=live,replay=dict(ok=result.ok,code=result.code,audit=dict(result.audit)),replay_value_comparison=matches,
 camera_metadata_received_ns=camera_receipt,camera=camera,tf_source='exact audit snapshot_transform.T_base_optical, already optical; no second conversion',
 geometry_source='same actual nearfield replay_geometry audit; source point hash verified',visual_found=top.get('visual') is not None,
 visual_choice_counts=[len(x) for x in visual.get('choices',[])],visual_rejection_counts=dict(Counter(x['reason'] for x in rejections)),visual_rejections=rejections,visual_edge_checks=edgechecks,
 metric_depth_edges=depth_edges,source_sha256={str(p.relative_to(W)):sha(p) for p in source_paths},
 no_ros_calls=True,no_hardware_access=True,no_temperature_reads=True,production_modified=False,physical_grasp_verified=False)
receipt=save('result.json',report);print(json.dumps(receipt,ensure_ascii=False),flush=True)
print(json.dumps(dict(replay_matches={k:v['equal'] for k,v in matches.items()},visual=report['visual_found'],choices=report['visual_choice_counts'],rejections=report['visual_rejection_counts']),ensure_ascii=False),flush=True)
