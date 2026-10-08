"""Closed-bag fixed-footprint support audit, with exact TF and command-tail evidence."""
from pathlib import Path
from types import SimpleNamespace
import hashlib,json,math,shutil,argparse,re
import numpy as np,cv2,rosbag,rospy,tf2_ros
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix
from alicia_grasp_modes.tabletop import (Config,_fit_plane,_project_pixels,project_base_to_pixel,
    _support_quadrants,_normalized_plane,_validate_support_plane_update)
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES
BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
parser=argparse.ArgumentParser()
parser.add_argument('--session',required=True);parser.add_argument('--formal',required=True);parser.add_argument('--generation',type=int,required=True)
args=parser.parse_args();SESSION=Path(args.session);FORMAL=Path(args.formal);GENERATION=args.generation
OUT=SESSION/'offline_support_motion_replay'
URDF=Path('.runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf')

def transform(m):
 q=m.transform.rotation;t=m.transform.translation;F=quaternion_matrix([q.x,q.y,q.z,q.w]);F[:3,3]=[t.x,t.y,t.z];return F

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def codes(q):return np.rint(np.asarray(q)*4096/(2*math.pi)+2048).astype(int).tolist()
def main():
 global FIRST,TASK
 assert Path('/media/zhuyupei/SanStand').is_mount()
 cv2.setNumThreads(1);OUT.mkdir(exist_ok=False);bridge=CvBridge();cfg=Config()
 buffer=tf2_ros.Buffer(cache_time=rospy.Duration(180),debug=False);statuses={};accepted=[];sdk=[];terminal=None;rgb_sources=set();depth_sources=set()
 with rosbag.Bag(str(SESSION/'scene_diagnostic.bag')) as b:
  for topic,m,t in b.read_messages(topics=['/tf','/tf_static','/perception/unknown/detector_status','/grasp/state','/alicia_d/accepted_joint_states','/alicia_d/control_reference']):
   if topic in ['/tf','/tf_static']:
    for tr in m.transforms:(buffer.set_transform_static if topic=='/tf_static' else buffer.set_transform)(tr,'closed_bag')
   elif topic=='/perception/unknown/detector_status':
    s=json.loads(m.data);ns=s.get('source_stamp_ns');
    if ns and s.get('generation')==GENERATION:statuses[ns]=s
   elif topic=='/grasp/state':
    if not m.active and 'SUPPORT_GEOMETRY_CONTRADICTION' in m.message:terminal=dict(source_ns=m.header.stamp.to_nsec(),receipt_ns=t.to_nsec(),message=m.message)
   else:
    q=[m.position[m.name.index(n)] for n in ARM_NAMES];row=dict(source_ns=m.header.stamp.to_nsec(),receipt_ns=t.to_nsec(),q_rad=q,counts=codes(q));(accepted if topic.endswith('accepted_joint_states') else sdk).append(row)
 assert terminal is not None;t0=terminal['source_ns']
 lost=next(s for s in statuses.values() if s.get('first_loss',{}).get('reason','').startswith('perception_failure: support_plane'))
 FIRST=lost['first_loss']['source_stamp_ns'];TASK=int(re.search(r'source_stamp_ns=(\d+)',terminal['message'])[1]);ref_status=lost
 anchor=np.array(ref_status['anchor_position_base']);footprint=np.array(ref_status['reference_support_footprint_base']);reference=np.array(ref_status['reference_support_plane_base'])
 assert all(np.array_equal(s['reference_support_plane_base'],reference) and np.array_equal(s['reference_support_footprint_base'],footprint) for s in statuses.values())
 camera_rows=[]
 with rosbag.Bag(str(FORMAL)) as b:
  for topic,m,t in b.read_messages(topics=['/grasp_record/camera_runtime_metadata']):camera_rows.append(dict(receipt_ns=t.to_nsec(),metadata=json.loads(m.data)))
 camera=camera_rows[0]['metadata']['parameters'];keys=['fx','fy','cx','cy','depth_scale'];assert all(all(r['metadata']['parameters'][k]==camera[k] for k in keys) for r in camera_rows);K=[camera[k] for k in ['fx','fy','cx','cy']]
 (OUT/'camera_metadata.json').write_text(json.dumps(camera_rows,ensure_ascii=False,indent=2)+'\n')
 wanted={FIRST,TASK};depths={};rgbs={};origins={}
 # Scene recorder preserves a bounded post-terminal interval. Formal snapshots
 # additionally preserve the two exact fault sources which scene sampling missed.
 sampled=set()
 for path in [SESSION/'scene_diagnostic.bag',FORMAL]:
  with rosbag.Bag(str(path)) as b:
   for topic,m,t in b.read_messages(topics=['/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw']):
    ns=m.header.stamp.to_nsec()
    selected=(ns in wanted or (t0<=ns<=t0+8000000000) or (t0-38000000000<=ns<t0 and ns//400000000 not in sampled))
    if not selected:continue
    if 'depth' in topic:sampled.add(ns//400000000)
    table=rgbs if 'color' in topic else depths
    if ns not in table:table[ns]=m;origins[(ns,'rgb' if 'color' in topic else 'depth')]=dict(bag=str(path),receipt_ns=t.to_nsec())
 rows=[];fk=SerialUrdfFk(URDF.read_text(),ARM_NAMES);sample_ns=np.array([x['source_ns'] for x in accepted],dtype=np.int64)
 for ns in sorted(set(depths)&set(rgbs)):
  dm=depths[ns];rgb=rgbs[ns];assert dm.header.frame_id==rgb.header.frame_id=='camera_link'
  depth=bridge.imgmsg_to_cv2(dm,'passthrough').astype(np.float32)*camera['depth_scale'];F=transform(buffer.lookup_transform_core('base_link','camera_link',dm.header.stamp))
  center=project_base_to_pixel(anchor,F,K);v,u=np.indices(depth.shape);valid=np.isfinite(depth)&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m);radius=np.hypot(u-center[0],v-center[1]);ring=valid&(radius>=70)&(radius<=170);pu,pv=u[ring],v[ring];points=_project_pixels(depth[ring],pu,pv,K)
  row=dict(source_ns=ns,seconds_after_terminal=(ns-t0)*1e-9,exact_rgbd=True,source_records={k:origins[(ns,k)] for k in ['rgb','depth']},T_base_camera=F.tolist(),anchor_projected_uv=center.tolist(),ring_points=len(points),recorded_detail=statuses.get(ns,{}).get('detail'))
  try:
   plane,ratio,rms,count=_fit_plane(points,cfg);supporting=abs(points@plane[:3]+plane[3])<=cfg.support_threshold_m;quadrants=_support_quadrants(supporting,pu,pv,center);current=_normalized_plane(np.linalg.solve(F.T,plane));current*=1 if current[:3]@reference[:3]>=0 else -1
   delta=current-reference;signed=footprint@delta[:3]+delta[3];anchor_shift=float(delta[:3]@anchor+delta[3]);maximum=max(abs(anchor_shift),float(np.max(abs(signed))));angle=float(np.rad2deg(np.arccos(np.clip(current[:3]@reference[:3],-1,1))))
   available=bool(np.isfinite(rms) and rms<=cfg.support_threshold_m and quadrants>=3)
   row.update(current_plane_base=current.tolist(),fit_ratio=ratio,fit_rms_m=rms,fit_quadrants=quadrants,fit_samples=count,original_fit_evidence_available=available,
      reference_angle_deg=angle,reference_signed_anchor_distance_m=anchor_shift,reference_signed_footprint_distance_m=signed.tolist(),reference_max_distance_m=maximum,worst_footprint_index=int(np.argmax(abs(signed))))
   lag_rows=[]
   if available and ns<t0:
    for lag in np.arange(-.30,.301,.025):
     try:
      LF=transform(buffer.lookup_transform_core('base_link','camera_link',rospy.Time.from_sec(ns*1e-9-float(lag))))
      LP=_normalized_plane(np.linalg.solve(LF.T,plane));LP*=1 if LP[:3]@reference[:3]>=0 else -1
      delta_lag=LP-reference;error=max(abs(float(delta_lag[:3]@anchor+delta_lag[3])),float(np.max(abs(footprint@delta_lag[:3]+delta_lag[3]))))
      lag_rows.append(dict(assumed_image_delay_sec=float(lag),max_footprint_distance_m=error))
     except Exception:pass
   row['timestamp_shift_diagnostic']=lag_rows
   if available:
    try:_validate_support_plane_update(reference,current,anchor,cfg,footprint);row['original_consistency_pass']=True
    except ValueError as e:row.update(original_consistency_pass=False,original_consistency_error=str(e))
   else:row['original_consistency_pass']=None
   # Exact TF chain and bracketing accepted-joint interpolation check. Interpolation
   # is a diagnostic; it is not a fresh feedback sample or a motion permit.
   ti=int(np.searchsorted(sample_ns,ns));
   if 0<ti<len(accepted):
    a,b=accepted[ti-1],accepted[ti];w=(ns-a['source_ns'])/(b['source_ns']-a['source_ns']);q=np.array(a['q_rad'])*(1-w)+np.array(b['q_rad'])*w;published_tool=transform(buffer.lookup_transform_core('base_link','tool0',dm.header.stamp));computed_tool=fk(q)
    row['tf_vs_interpolated_accepted_fk']=dict(bracket_source_ns=[a['source_ns'],b['source_ns']],bracket_gap_sec=(b['source_ns']-a['source_ns'])*1e-9,position_difference_m=float(np.linalg.norm(published_tool[:3,3]-computed_tool[:3,3])),angle_difference_deg=float(np.degrees(np.arccos(np.clip((np.trace(published_tool[:3,:3].T@computed_tool[:3,:3])-1)/2,-1,1)))))
   if ns in wanted or ns==max(set(depths)&set(rgbs)):
    image=bridge.imgmsg_to_cv2(rgb,'bgr8');overlay=image.copy();cv2.circle(overlay,tuple(np.rint(center).astype(int)),70,(0,200,255),1);cv2.circle(overlay,tuple(np.rint(center).astype(int)),170,(0,200,255),1);idx=np.flatnonzero(supporting)[::80];overlay[pv[idx],pu[idx]]=[0,255,0];cv2.imwrite(str(OUT/(str(ns)+'_raw.png')),image);cv2.imwrite(str(OUT/(str(ns)+'_ring.png')),overlay);np.savez_compressed(str(OUT/(str(ns)+'_depth_geometry.npz')),depth_raw=bridge.imgmsg_to_cv2(dm,'passthrough'),T_base_camera=F,current_plane_base=current,reference_plane_base=reference,footprint=footprint)
  except ValueError as e:row['fit_error']=str(e)
  rows.append(row)
 tail=[r for r in rows if r['seconds_after_terminal']>=0];good=[r for r in tail if r.get('original_fit_evidence_available')];at=[r for r in accepted if t0<=r['source_ns']<=t0+8000000000];st=[r for r in sdk if t0<=r['source_ns']<=t0+8000000000]
 summary=dict(post_terminal_frames=len(tail),fit_evidence_available=len(good),consistency_passes=sum(r['original_consistency_pass'] is True for r in good),distance_range_m=[min(r['reference_max_distance_m'] for r in good),max(r['reference_max_distance_m'] for r in good)] if good else None,
   angle_range_deg=[min(r['reference_angle_deg'] for r in good),max(r['reference_angle_deg'] for r in good)] if good else None,
   source_coverage_seconds=[tail[0]['seconds_after_terminal'],tail[-1]['seconds_after_terminal']],
   accepted_samples=len(at),accepted_counts_ptp=np.ptp(np.array([r['counts'] for r in at]),axis=0).tolist(),sdk_samples=len(st),sdk_counts_unique=[list(x) for x in sorted({tuple(r['counts']) for r in st})],
   accepted_first=at[0],accepted_last=at[-1])
 report=dict(scope='same-epoch same-target fixed support-footprint closed-bag replay',plan_id='see frozen formal audit',generation=GENERATION,terminal=terminal,
   first_loss=ref_status['first_loss'],task_fault_source_ns=TASK,reference_plane_base=reference.tolist(),reference_footprint_base=footprint.tolist(),anchor_position_base=anchor.tolist(),
   all_recorded_reference_planes_and_footprints_unchanged=True,model_sha256=fk.model_sha256,camera_parameters_same_attempt=True,original_thresholds=dict(distance_m=.004,angle_deg=4),rows=rows,tail_summary=summary,
   production_changed=False,ros_master_used=False,motion_authorized=False,
   source_script_sha256=sha(Path(__file__)),limitations=['Plane consistency is model/RGB-D evidence, not independent ground-truth TCP metrology.','Timestamp shifts are counterfactual diagnostics, not calibrated exposure timing or motion permission.','No reacquisition, failure reset or physical threshold change.','Bracketing FK comparison is diagnostic interpolation, not a new accepted feedback sample.'])
 (OUT/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');shutil.copy2(__file__,OUT/Path(__file__).name)
 print(json.dumps(dict(faults=[{k:r.get(k) for k in ['source_ns','recorded_detail','original_consistency_error','reference_max_distance_m','reference_angle_deg','fit_ratio','fit_rms_m','tf_vs_interpolated_accepted_fk']} for r in rows if r['source_ns'] in wanted],tail_summary=summary),ensure_ascii=False,indent=2))
if __name__=='__main__':main()
