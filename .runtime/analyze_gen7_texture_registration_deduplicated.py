#!/usr/bin/env python3
"""Independent feature correspondences; diagnostic only, no ROS actions."""
from pathlib import Path
import json,hashlib,itertools
import numpy as np,cv2,rosbag
from cv_bridge import CvBridge
from alicia_flexible_grasp.vision import multiview_surface as mv
ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
RECORD=ROOT/'grasp_records/20261009T022026.944578Z_cca56e3000b84b5289c1adfa0e09a9bb'
OUT=ROOT/'ros_log/full_latest_ros_20261009T003407Z/gen7_registration_analysis'

def main():
 cv2.setNumThreads(1);r=json.loads((OUT/'reconstruction.json').read_text());stamps=[r['reference_source_ns'],r['exact_recorded_source_ns']]
 frames={ns:{} for ns in stamps};cameras=[]
 topics=['/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw','/perception/object_mask','/grasp_record/camera_runtime_metadata']
 with rosbag.Bag(str(RECORD/'keyframes.bag')) as bag:
  for topic,msg,t in bag.read_messages(topics=topics):
   if topic.endswith('camera_runtime_metadata'):cameras.append((t.to_nsec(),json.loads(msg.data)['parameters']));continue
   ns=msg.header.stamp.to_nsec()
   if ns in frames:frames[ns][topic]=msg
 bridge=CvBridge();data=[]
 for i,ns in enumerate(stamps):
  f=frames[ns];c=min(cameras,key=lambda p:abs(p[0]-ns))[1]
  rgb=bridge.imgmsg_to_cv2(f[topics[0]],'bgr8');depth=bridge.imgmsg_to_cv2(f[topics[1]],'passthrough');mask=bridge.imgmsg_to_cv2(f[topics[2]],'mono8')
  T=np.asarray(r['reference_T_base_optical' if i==0 else 'moving_T_base_optical'])
  data.append(dict(rgb=rgb,gray=cv2.cvtColor(rgb,cv2.COLOR_BGR2GRAY),depth=depth,mask=mask,camera=c,T=T))
 basis=mv._support_basis(np.asarray(r['reference_support_normal']))
 def point(item,uv):
  x,y=np.round(uv).astype(int);d=item['depth'];c=item['camera']
  if not(0<=x<d.shape[1] and 0<=y<d.shape[0]) or not item['mask'][y,x]:return None
  z=float(d[y,x])*c['depth_scale']
  if not np.isfinite(z) or not .1<z<2.:return None
  optical=np.array([(float(uv[0])-c['cx'])*z/c['fx'],(float(uv[1])-c['cy'])*z/c['fy'],z]);return item['T'][:3,:3]@optical+item['T'][:3,3]
 def fit(a,b):
  rot,t=mv._rigid_update_about_support(b,a,basis);T=np.eye(4);T[:3,:3]=rot;T[:3,3]=t;return T
 def residual(a,b,T):return np.linalg.norm(b@T[:3,:3].T+T[:3,3]-a,axis=1)
 outputs=[]
 detectors=[('SIFT',cv2.SIFT_create(nfeatures=1500,contrastThreshold=.02,edgeThreshold=10),cv2.NORM_L2)] if hasattr(cv2,'SIFT_create') else []
 detectors.append(('AKAZE',cv2.AKAZE_create(threshold=.0005),cv2.NORM_HAMMING))
 detectors.append(('ORB',cv2.ORB_create(nfeatures=2500,scaleFactor=1.15,nlevels=8,edgeThreshold=15,patchSize=21,fastThreshold=7),cv2.NORM_HAMMING))
 for name,det,norm in detectors:
  k0,d0=det.detectAndCompute(data[0]['gray'],data[0]['mask']);k1,d1=det.detectAndCompute(data[1]['gray'],data[1]['mask']);matcher=cv2.BFMatcher(norm)
  if d0 is None or d1 is None:outputs.append(dict(detector=name,reason='no_descriptors'));continue
  ab=matcher.knnMatch(d0,d1,k=2);ba=matcher.knnMatch(d1,d0,k=2)
  for ratio in (.70,.75,.80):
   forward={m.queryIdx:m for pair in ab if len(pair)==2 for m,n in [pair] if m.distance<ratio*n.distance};reverse={m.queryIdx:m.trainIdx for pair in ba if len(pair)==2 for m,n in [pair] if m.distance<ratio*n.distance}
   matches=[m for i,m in forward.items() if reverse.get(m.trainIdx)==i]
   valid=[]
   for m in matches:
    p0=point(data[0],k0[m.queryIdx].pt);p1=point(data[1],k1[m.trainIdx].pt)
    if p0 is not None and p1 is not None:valid.append((m,p0,p1))
   raw_valid_count=len(valid)
   dedup=[]
   for item in sorted(valid,key=lambda item:item[0].distance):
    uv0=np.asarray(k0[item[0].queryIdx].pt);uv1=np.asarray(k1[item[0].trainIdx].pt)
    if any(np.linalg.norm(uv0-np.asarray(k0[other[0].queryIdx].pt))<2. or np.linalg.norm(uv1-np.asarray(k1[other[0].trainIdx].pt))<2. for other in dedup):continue
    dedup.append(item)
   valid=dedup
   row=dict(detector=name,ratio=ratio,depth_matches_before_dedup=raw_valid_count,keypoint_counts=[len(k0),len(k1)],mutual_ratio_matches=len(matches),valid_real_depth_matches=len(valid),raw_pixel_pairs=[dict(reference=k0[m.queryIdx].pt,moving=k1[m.trainIdx].pt,distance=float(m.distance),reference_point_base=a.tolist(),moving_point_base=b.tolist()) for m,a,b in valid],thresholds=[])
   if len(valid)>=3:
    a=np.array([p[1] for p in valid]);b=np.array([p[2] for p in valid]);rng=np.random.RandomState(7)
    subsets=list(itertools.combinations(range(len(valid)),3))
    if len(subsets)>1000:subsets=[tuple(rng.choice(len(valid),3,replace=False)) for _ in range(1000)]
    for limit in (.003,.005):
     best=None
     for ix in subsets:
      xy=b[list(ix)]@basis
      if np.linalg.matrix_rank(xy[:,:2]-xy[0,:2],tol=1e-6)<2:continue
      T=fit(a[list(ix)],b[list(ix)]);res=residual(a,b,T);mask=res<=limit
      key=(-int(mask.sum()),float(np.median(res[mask])) if mask.any() else 1e9)
      if best is None or key<best[0]:best=(key,T,mask)
     if best is None:continue
     mask=best[2]
     for _ in range(3):
      if mask.sum()<3:break
      T=fit(a[mask],b[mask]);res=residual(a,b,T);new=res<=limit
      if np.array_equal(new,mask):break
      mask=new
     if mask.sum()<3:continue
     T=fit(a[mask],b[mask]);res=residual(a,b,T)
     evidence=dict(inlier_limit_m=limit,inlier_count=int(mask.sum()),match_count=len(valid),inlier_indices=np.flatnonzero(mask).tolist(),yaw_deg=float(mv._yaw_degrees(T[:3,:3],basis)),transform_base=T.tolist(),inlier_rmse_m=float(np.sqrt(np.mean(res[mask]**2))),all_residuals_m=res.tolist(),uncorrected_inlier_rmse_m=float(np.sqrt(np.mean(np.sum((a[mask]-b[mask])**2,axis=1)))),inlier_reference_span_m=np.ptp(a[mask]@basis,axis=0).tolist(),inlier_moving_span_m=np.ptp(b[mask]@basis,axis=0).tolist(),maximum_point_displacement_m=mv.maximum_point_displacement_m(b[mask],T),reference_xy_covariance_eigenvalues=np.linalg.eigvalsh(np.cov((a[mask]@basis)[:,:2].T)).tolist(),original_icp_transform_texture_residuals_m=residual(a,b,np.asarray(r['registration']['transform_base'])).tolist(),original_icp_transform_texture_inlier_rmse_m=float(np.sqrt(np.mean(residual(a,b,np.asarray(r['registration']['transform_base']))[mask]**2))))
     row['thresholds'].append(evidence)
     if ratio==.75 and limit==.003:
      canvas=cv2.drawMatches(data[0]['rgb'],k0,data[1]['rgb'],k1,[valid[i][0] for i in np.flatnonzero(mask)],None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
      cv2.imwrite(str(OUT/(name.lower()+'_deduplicated_depth_inliers.png')),canvas)
   outputs.append(row)
 report=dict(diagnostic_only=True,execution_authorized=False,no_ros_actions=True,source_stamps_ns=stamps,matching='descriptor mutual nearest with ratio on both directions; original RGB masks; only actual positive nearest-pixel depth, no depth interpolation/invention',fit='independent texture correspondences with support-normal-constrained yaw+translation least squares and 3-pair RANSAC; rank only by 3D residual consensus, never yaw bound',limits=['Repeated printed patterns and small spatial spread can produce ambiguous feature correspondence.','This is independent diagnostic support, not a replacement for production registration or identity gates.','Raw reference final frame is used, not original 3-frame depth median.'],rows=outputs)
 (OUT/'texture_registration_deduplicated.json').write_text(json.dumps(report,indent=2)+'\n')
 for row in outputs:print(json.dumps({k:v for k,v in row.items() if k!='raw_pixel_pairs'}),flush=True)
if __name__=='__main__':main()
