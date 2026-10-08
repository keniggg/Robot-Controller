import json,numpy as np,cv2
ROOT='/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_ros_resume_20261004T002930Z/aligned_capture_20261004T003847Z'
meta=json.load(open(ROOT+'/capture.json'));out=json.load(open(ROOT+'/offline_segmentation_diagnostic.json'))
v,u=np.indices((480,640));roi=(u>=305)&(u<=325)&(v>=259)&(v<=278);near=(u>=284)&(u<=344)&(v>=239)&(v<=298);local=near&~((u>=300)&(u<=330)&(v>=254)&(v<=283))
rows=[];masks=[]
def stats(a):return {'count':len(a),'min_m':float(np.min(a)),'median_m':float(np.median(a)),'max_m':float(np.max(a))} if len(a) else {'count':0}
for i in range(3):
 d=np.load(ROOT+'/depth_%d.npy'%i).astype(float)*meta['camera']['depth_scale'];c=cv2.imread(ROOT+'/color_%d.png'%i);valid=(d>=.03)&(d<=2)
 # Only to quantify the manually inspected RGB silhouette. Not production detection.
 b=near&(np.max(c,axis=2)<95);count,labels,ss,cents=cv2.connectedComponentsWithStats(b.astype(np.uint8),8);eligible=[k for k in range(1,count) if np.any((labels==k)&roi)]; chosen=max(eligible,key=lambda k:ss[k,cv2.CC_STAT_AREA]);m=labels==chosen;masks.append(m)
 x,y,w,h,n=[int(x) for x in ss[chosen]];z=float(np.median(d[local&valid]));rows.append({'index':i,'diagnostic_manual_roi_valid_depth':stats(d[roi&valid]),'surrounding_table_depth':stats(d[local&valid]),'diagnostic_rgb_silhouette_bbox':[x,y,w,h],'diagnostic_rgb_silhouette_area_px':n,'diagnostic_rgb_silhouette_center_uv':cents[chosen].tolist(),'apparent_projection_at_table_depth_mm':[1000*w*z/meta['camera']['fx'],1000*h*z/meta['camera']['fy']],'projection_warning':'optical-plane projection estimate only; not physical object size or valid grasp geometry'})
out['observation_distance_and_rgb_diagnostic']=rows;out['diagnostic_rgb_silhouette_iou']=[{'from':i,'to':j,'iou':float(np.sum(masks[i]&masks[j])/np.sum(masks[i]|masks[j]))} for i,j in ((0,1),(0,2),(1,2))]
with open(ROOT+'/offline_segmentation_diagnostic.json','w') as f:json.dump(out,f,indent=2)
print(json.dumps({'rows':rows,'rgb_iou':out['diagnostic_rgb_silhouette_iou']},indent=2))
