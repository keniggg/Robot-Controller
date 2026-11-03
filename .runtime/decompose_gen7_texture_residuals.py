from pathlib import Path
import json,hashlib
import numpy as np,cv2,rosbag
from cv_bridge import CvBridge
O=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z/gen7_registration_analysis')
R=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261009T022026.944578Z_cca56e3000b84b5289c1adfa0e09a9bb')
x=json.loads((O/'texture_registration_deduplicated.json').read_text());r=json.loads((O/'reconstruction.json').read_text());f=json.loads((O/'finite_yaw_seed_trace.json').read_text());T=np.array(next(z for z in f['rows'] if z['initial_yaw_deg']==0)['transform']);n=np.array(r['reference_support_normal']);nm=np.array(r['moving_support_normal']);dr=r['reference_support_offset_m'];dm=r['moving_support_offset_m'];rows=[]
for row in x['rows']:
 if not row['thresholds']:continue
 fit=row['thresholds'][0];ind=fit['inlier_indices'];pairs=row['raw_pixel_pairs'];a=np.array([pairs[i]['reference_point_base'] for i in ind]);b=np.array([pairs[i]['moving_point_base'] for i in ind]);F=np.array(fit['transform_base']);results={}
 for name,m in [('translation_only_seed',T),('texture_self_fit',F)]:
  e=b@m[:3,:3].T+m[:3,3]-a;normal=e@n;tangent=e-normal[:,None]*n
  results[name]=dict(normal_signed_mm=(normal*1000).tolist(),tangent_norm_mm=(np.linalg.norm(tangent,axis=1)*1000).tolist(),mean_error_base_mm=(e.mean(axis=0)*1000).tolist(),normal_rms_mm=float(np.sqrt(np.mean(normal**2))*1000),tangent_rms_mm=float(np.sqrt(np.mean(np.sum(tangent*tangent,axis=1)))*1000),per_point_error_base_mm=(e*1000).tolist())
 evidence=[dict(match_index=i,reference_uv=pairs[i]['reference'],moving_uv=pairs[i]['moving'],reference_height_above_own_plane_mm=float((a[k]@n+dr)*1000),moving_height_above_own_plane_mm=float((b[k]@nm+dm)*1000)) for k,i in enumerate(ind)]
 rows.append(dict(detector=row['detector'],ratio=row['ratio'],frozen_inlier_count=len(ind),evidence=evidence,results=results))
strict=next(row for row in x['rows'] if row['detector']=='ORB' and row['ratio']==.7);indices=strict['thresholds'][0]['inlier_indices'];stamps=[r['reference_source_ns'],r['exact_recorded_source_ns']];imgs={};bridge=CvBridge()
with rosbag.Bag(str(R/'keyframes.bag')) as bag:
 for topic,msg,t in bag.read_messages(topics=['/supervisor/camera/color/image_raw']):
  if msg.header.stamp.to_nsec() in stamps:imgs[msg.header.stamp.to_nsec()]=bridge.imgmsg_to_cv2(msg,'bgr8')
sheet=np.full((len(indices)*260,580,3),255,np.uint8)
for row,i in enumerate(indices):
 p=strict['raw_pixel_pairs'][i]
 for col,(ns,key,margin) in enumerate(zip(stamps,['reference','moving'],[8,12])):
  uv=p[key];u,v=np.round(uv).astype(int);crop=imgs[ns][v-margin:v+margin+1,u-margin:u+margin+1];large=cv2.resize(crop,(225,225),interpolation=cv2.INTER_NEAREST)
  x0=20+col*285;y0=row*260+30;sheet[y0:y0+225,x0:x0+225]=large
  center=(x0+112,y0+112);cv2.drawMarker(sheet,center,(0,0,255),cv2.MARKER_CROSS,9,1)
  cv2.putText(sheet,f'{i} {key} ({u},{v})',(x0,row*260+22),cv2.FONT_HERSHEY_SIMPLEX,.47,(0,0,0),1,cv2.LINE_AA)
cv2.imwrite(str(O/'orb_ratio070_same_frozen_matches_patches.png'),sheet)
full=[]
for ns,key in zip(stamps,['reference','moving']):
 crop=imgs[ns][160:292,252:416].copy()
 for i in indices:
  u,v=np.array(strict['raw_pixel_pairs'][i][key])-np.array([252,160]);cv2.circle(crop,(int(round(u)),int(round(v))),2,(0,0,255),1);cv2.putText(crop,str(i),(int(round(u))+3,int(round(v))-3),cv2.FONT_HERSHEY_SIMPLEX,.25,(255,0,0),1,cv2.LINE_AA)
 full.append(cv2.resize(crop,(656,528),interpolation=cv2.INTER_NEAREST))
cv2.imwrite(str(O/'orb_ratio070_frozen_matches_context.png'),np.hstack(full))
report=dict(diagnostic_only=True,matching_reselected=False,plane_decomposition='Reference unit support normal; residual moving transformed to reference minus reference point.',rows=rows,viewed_images=['orb_ratio070_same_frozen_matches_patches.png','orb_ratio070_frozen_matches_context.png'])
(O/'texture_residual_decomposition.json').write_text(json.dumps(report,indent=2)+'\n')
for row in rows:print(row['detector'],row['ratio'],{k:dict(normal_rms_mm=round(v['normal_rms_mm'],4),tangent_rms_mm=round(v['tangent_rms_mm'],4)) for k,v in row['results'].items()})
