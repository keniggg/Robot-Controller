from pathlib import Path
import json,hashlib
import numpy as np,cv2
from alicia_grasp_modes.tabletop import _project
BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/gen21_analysis');p=BASE/'replay_inputs'
m=json.loads((p/'manifest.json').read_text());rows={r['role']:r for r in m['files']};last=dict(np.load(rows['last_pre_motion']['path']));prev=dict(np.load(rows['previous_ready_reference_probe']['path']))
status=json.loads((p/'last_pre_motion_status.json').read_text())['message'];plane=np.asarray(status['reference_support_plane_base']);depth=last['depth_raw'].astype(np.float32)*float(last['depth_scale']);points=_project(depth,last['K']);T=last['T_base_camera'];base=points@T[:3,:3].T+T[:3,3];height=base@plane[:3]+plane[3]
valid=np.isfinite(depth)&(depth>=.03)&(depth<=2.)
a=last['recorded_refined_mask']!=0;b=prev['recorded_refined_mask']!=0
hsv=cv2.cvtColor(last['color'],cv2.COLOR_BGR2HSV);red=((hsv[:,:,0]<10)|(hsv[:,:,0]>170))&(hsv[:,:,1]>80)
result={}
for name,mask in [('last_recorded_mask',a),('same_pixel_intersection',a&b),('last_only_pixels',a&~b),('last_only_red_pixels',a&~b&red)]:
 h=height[mask&valid];v=mask&valid
 result[name]=dict(pixel_count=int(np.count_nonzero(mask)),valid_depth_count=int(np.count_nonzero(v)),invalid_depth_count=int(np.count_nonzero(mask&~valid)),within_2mm_support_count=int(np.count_nonzero(v&(abs(height)<=.002))),between_2_and_4mm_count=int(np.count_nonzero(v&(height>.002)&(height<.004))),above_4mm_count=int(np.count_nonzero(v&(height>=.004))),negative_beyond_2mm_count=int(np.count_nonzero(v&(height<-.002))),height_mm_quantiles=np.percentile(h*1000,[0,10,50,90,100]).tolist() if len(h) else None,red_pixel_count=int(np.count_nonzero(mask&red)))
report=dict(record=m['record'],source_ns=int(last['source_stamp_ns']),previous_source_ns=int(prev['source_stamp_ns']),q_max_change_rad=float(np.max(abs(last['q']-prev['q']))),T_max_change=float(np.max(abs(last['T_base_camera']-prev['T_base_camera']))),frozen_plane_base=plane.tolist(),results=result,limitations=['Same-pixel mask difference between two real frames is a diagnostic; no temporal mask is used as production foreground.','Height uses recorded exact TF and frozen plane, not independent physical metrology.','Depth-invalid pixels cannot be labeled table or object using this measurement.','Red color alone is not a background label.'])
out=BASE/'frozen_reference_mask_depth_diagnostic.json';out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(dict(path=str(out),sha256=hashlib.sha256(out.read_bytes()).hexdigest(),results=result,q_max_change=report['q_max_change_rad']),ensure_ascii=False))
