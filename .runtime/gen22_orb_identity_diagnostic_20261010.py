from pathlib import Path
import hashlib,json
import cv2,numpy as np
B=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/gen22_analysis');P=B/'replay_inputs';m=json.loads((P/'manifest.json').read_text());rep=json.loads((B/'arrival_identity_replay_r1.json').read_text());idx={r['source_stamp_ns']:r for r in m['files']};lr=next(r for r in m['files'] if r['role']=='last_pre_motion');last=dict(np.load(lr['path']))
orb=cv2.ORB_create(nfeatures=1000,scaleFactor=1.2,nlevels=8,edgeThreshold=15,firstLevel=0,WTA_K=2,scoreType=cv2.ORB_HARRIS_SCORE,patchSize=31,fastThreshold=20)
matcher=cv2.BFMatcher(cv2.NORM_HAMMING,crossCheck=False)
def detect(image,mask):
 inner=cv2.erode((mask>0).astype(np.uint8)*255,np.ones((7,7),np.uint8));return orb.detectAndCompute(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),inner)
kr,dr=detect(last['color'],last['recorded_refined_mask'])
def coverage(points,mask):
 ys,xs=np.nonzero(mask);center=np.array([xs.mean(),ys.mean()]);quadrants=sorted(set((int(x>=center[0]),int(y>=center[1])) for x,y in points));hull=cv2.convexHull(points.astype(np.float32)) if len(points)>=3 else None
 return dict(quadrants=len(quadrants),quadrant_ids=quadrants,hull_area_fraction_of_mask=float(cv2.contourArea(hull)/np.count_nonzero(mask)) if hull is not None else 0.,bbox_fraction_of_mask_bbox=float(np.prod(np.ptp(points,axis=0))/((xs.max()-xs.min()+1)*(ys.max()-ys.min()+1))) if len(points) else 0.)
def compare(image,mask,ns,variant):
 kc,dc=detect(image,mask);row=dict(source_ns=ns,variant=variant,reference_keypoints=len(kr),current_keypoints=len(kc),mutual_ratio_matches=0,ransac_inliers=0,homography=None)
 if dr is None or dc is None or len(dr)<2 or len(dc)<2:return row,None
 forward={x[0].queryIdx:x[0] for x in matcher.knnMatch(dr,dc,k=2) if len(x)==2 and x[0].distance<.75*x[1].distance}
 reverse={x[0].queryIdx:x[0] for x in matcher.knnMatch(dc,dr,k=2) if len(x)==2 and x[0].distance<.75*x[1].distance}
 matches=[x for i,x in forward.items() if x.trainIdx in reverse and reverse[x.trainIdx].trainIdx==i]
 row.update(forward_ratio_matches=len(forward),reverse_ratio_matches=len(reverse),mutual_ratio_matches=len(matches))
 if len(matches)<4:return row,None
 src=np.asarray([kr[x.queryIdx].pt for x in matches],np.float32);dst=np.asarray([kc[x.trainIdx].pt for x in matches],np.float32)
 cv2.setRNGSeed(1729);H,inliers=cv2.findHomography(src,dst,cv2.RANSAC,3.,maxIters=2000,confidence=.995)
 if H is None:return row,None
 inside=inliers.ravel()!=0;si=src[inside];di=dst[inside];proj=cv2.perspectiveTransform(si[:,None,:],H)[:,0,:];err=np.linalg.norm(proj-di,axis=1);back=cv2.perspectiveTransform(di[:,None,:],np.linalg.inv(H))[:,0,:];berr=np.linalg.norm(back-si,axis=1)
 row.update(ransac_inliers=int(inside.sum()),inlier_fraction=float(inside.mean()),homography=H.tolist(),reprojection_median_px=float(np.median(err)),reprojection_p95_px=float(np.percentile(err,95)),reverse_reprojection_median_px=float(np.median(berr)),reference_coverage=coverage(si,last['recorded_refined_mask']),current_coverage=coverage(di,mask),inlier_points_reference_xy=si.tolist(),inlier_points_current_xy=di.tolist())
 viz=cv2.drawMatches(last['color'],kr,image,kc,[x for x,ok in zip(matches,inside) if ok],None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
 return row,viz
results=[];plots=[]
for i,item in enumerate(rep['results']):
 ns=item['source_ns'];f=dict(np.load(idx[ns]['path']));image=f['color'];mask=cv2.imread(str(B/('crop_ab_production_'+str(ns)+'_mask.png')),0)
 cloth=np.zeros(mask.shape,np.uint8);cloth[100:250,100:250]=255
 # Preserve pixel values but destroy global layout inside current object bbox.
 x,y,w,h=cv2.boundingRect(mask);scramble=image.copy();crop=image[y:y+h,x:x+w];sh=(int(np.ceil(h/16.))*16,int(np.ceil(w/16.))*16);padded=cv2.copyMakeBorder(crop,0,sh[0]-h,0,sh[1]-w,cv2.BORDER_REFLECT_101);tiles=[padded[yy:yy+16,xx:xx+16].copy() for yy in range(0,sh[0],16) for xx in range(0,sh[1],16)];perm=np.random.RandomState(1729).permutation(len(tiles));shuffled=padded.copy();j=0
 for yy in range(0,sh[0],16):
  for xx in range(0,sh[1],16):shuffled[yy:yy+16,xx:xx+16]=tiles[perm[j]];j+=1
 scramble[y:y+h,x:x+w]=shuffled[:h,:w]
 flat=np.empty_like(image);flat[:]=np.median(image[mask>0],axis=0).astype(np.uint8)
 for variant,img,region in [('recorded_object',image,mask),('recorded_wrong_tablecloth_roi',image,cloth),('synthetic_texture_tile_shuffle',scramble,mask),('synthetic_same_colour_textureless',flat,mask)]:
  row,viz=compare(img,region,ns,variant);results.append(row)
  if i==0 and viz is not None:
   path=B/('orb_'+variant+'_first_source_matches.png');cv2.imwrite(str(path),viz);plots.append(dict(variant=variant,path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
report=dict(record=m['record'],reference_source_ns=lr['source_stamp_ns'],replay_source_report_sha256=hashlib.sha256((B/'arrival_identity_replay_r1.json').read_bytes()).hexdigest(),opencv_version=cv2.__version__,algorithm='ORB because installed OpenCV has no SIFT API',parameters=dict(nfeatures=1000,scaleFactor=1.2,nlevels=8,edgeThreshold=15,patchSize=31,fastThreshold=20,mask_erosion_px=3,knn_ratio=.75,mutual_nearest_ratio=True,ransac_threshold_px=3.,ransac_max_iters=2000,ransac_confidence=.995,rng_seed=1729,negative_shuffle_tile_px=16,negative_real_cloth_roi_xyxy=[100,100,250,250]),results=results,plots=plots,limitations=['Diagnostic only. No appearance fallback or robot admission decision is implemented.','Real input second-view sources are adjacent saved images, not the exact twelve online assessed images.','Matching printed texture is evidence about image correspondence, not by itself proof of object identity or grasp safety.','Two negative classes are synthetic diagnostic controls, not observed alternate physical objects.','Parameters were fixed before results and were not tuned on these outputs.'])
out=B/'orb_texture_correspondence_diagnostic.json';out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(dict(path=str(out),sha256=hashlib.sha256(out.read_bytes()).hexdigest(),results=[{k:v for k,v in r.items() if k not in ('homography','inlier_points_reference_xy','inlier_points_current_xy')} for r in results],plots=plots),ensure_ascii=False))
