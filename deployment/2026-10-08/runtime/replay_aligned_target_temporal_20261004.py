import json, numpy as np, cv2
ROOT='/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_ros_resume_20261004T002930Z/aligned_capture_20261004T003847Z'
meta=json.load(open(ROOT+'/capture.json'));out=json.load(open(ROOT+'/offline_segmentation_diagnostic.json'))
v,u=np.indices((480,640));roi=(u>=305)&(u<=325)&(v>=259)&(v<=278);inner=(u>=308)&(u<=322)&(v>=261)&(v<=275); local=(u>=284)&(u<=344)&(v>=239)&(v<=298)&~((u>=300)&(u<=330)&(v>=254)&(v<=283))
ds=[np.load(ROOT+'/depth_%d.npy'%i).astype(float)*meta['camera']['depth_scale'] for i in range(3)]
cs=[cv2.imread(ROOT+'/color_%d.png'%i) for i in range(3)]
rows=[]
for i,j in ((0,1),(0,2),(1,2)):
 row={'from':i,'to':j,'source_dt_s':(meta['pairs'][j]['stamp_ns']-meta['pairs'][i]['stamp_ns'])*1e-9,'rgb_roi_mean_abs_channel_difference':float(np.mean(np.abs(cs[i][roi].astype(float)-cs[j][roi])))}
 for name,m in (('object_inner',inner),('local_table',local)):
  both=m&(ds[i]>.03)&(ds[j]>.03)&(ds[i]<2)&(ds[j]<2);delta=1000*(ds[j][both]-ds[i][both]);row[name]={'same_valid_pixel_count':int(both.sum()),'depth_delta_mm_p5_p50_p95':np.percentile(delta,[5,50,95]).tolist() if len(delta) else []}
 rows.append(row)
out['temporal_same_pixel_audit']=rows
out['interpretation']={'support_fit':'exact-T frames 1 and 2 pass original support fit via initial ring fallback and fail no_center_instance','center_object_depth':'RGB-visible object has inconsistent or missing depth; all original foreground/opening masks exclude center ROI','actual_height':'user reports approximately 5mm; diagnostic apparent height is not a trustworthy physical measurement','rgb_refinement':'not reached until measured foreground instance exists; no center instance means no RGB refinement','frame_0_tf':'No exact TF saved; used only camera-space diagnostic and excluded from production segmentation validation','parameter_mutations':False,'production_source_changes':False}
with open(ROOT+'/offline_segmentation_diagnostic.json','w') as f:json.dump(out,f,indent=2)
print(json.dumps(rows,indent=2))
