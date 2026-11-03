from pathlib import Path
import json,hashlib
import numpy as np,cv2
from alicia_grasp_modes.rgb_observation_handoff import _appearance
BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/gen21_analysis')
p=BASE/'replay_inputs';m=json.loads((p/'manifest.json').read_text())
rows=[];refs=[]
for r in m['files']:
 if not r['recorded_refined_mask_available']:continue
 a=dict(np.load(r['path']));h,c=_appearance(a['color'],a['recorded_refined_mask']);refs.append((r,h,c))
for a,ha,ca in refs:
 for b,hb,cb in refs:
  if a['source_stamp_ns']>=b['source_stamp_ns']:continue
  rows.append(dict(a=a['source_stamp_ns'],b=b['source_stamp_ns'],colour_distance=float(np.sqrt(max(0.,1.-np.sum(np.sqrt(ha*hb))))),shape_distance=float(cv2.matchShapes(ca,cb,cv2.CONTOURS_MATCH_I1,0))))
report=dict(record=m['record'],selection_stamp_ns=m['selection_stamp_ns'],replay_inputs=dict(path=str(p/'manifest.json'),sha256=hashlib.sha256((p/'manifest.json').read_bytes()).hexdigest()),frozen_reference_selection=dict(request_publication_ns=m['executing_stamp_ns'],last_ready_source_ns=m['last_pre_motion_source_stamp_ns'],last_ready_status_received_ns=json.loads((p/'last_pre_motion_status.json').read_text())['received_ns'],first_executing_status_received_ns=m['first_producer_executing_status_received_ns'],mechanism='unknown_tabletop_perception._observation_measurement_request_cb acquires self.lock, then freezes existing last_good_source and its appearance; ordinary process commits last_good/reference and publishes ready under the same self.lock. The last READY message from that producer before its first executing status is the reference candidate, not necessarily the READY already delivered before request publication.',limitations='Private reference dictionary was not recorded. This is supported same-publisher transition inference; compare both adjacent recorded READY references for sensitivity if needed.'),recorded_reference_masks=[{k:r[k] for k in ('role','source_stamp_ns','mask_pixels','mask_bbox_xyxy','mask_bytes_sha256','mask_png')} for r,_,_ in refs],pairwise_recorded_reference_appearance=rows,arrival_available_exact_online_sources=[r['source_stamp_ns'] for r in m['files'] if 'detector_status'in r],missing_exact_online_sources=m['missing_exact_sources'],restriction='Offline sealed bag and recorded masks only; no ROS, hardware, source modification, or synthetic current frame.')
out=BASE/'reference_and_recording_provenance.json';out.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(report=str(out),sha256=hashlib.sha256(out.read_bytes()).hexdigest(),reference_distances=rows),ensure_ascii=False))
