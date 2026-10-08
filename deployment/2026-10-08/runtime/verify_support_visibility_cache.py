"""Replay the saved source arrays with final audit code; no ROS calls."""
import json,time,hashlib
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from alicia_grasp_modes import tabletop as t
p=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/grasp_diagnostic_20260928T084245Z/support_visibility_replay')
original=json.loads((p/'report.json').read_text());rows=[];timings=[]
for expected in original['frames']:
 name=expected['label'];f=np.load(p/(name+'.npz'));D=f['depth_raw'].astype(np.float32)*float(f['depth_scale']);K=f['intrinsics'];T=f['transform'];reference=f['reference_plane'];a=SimpleNamespace(position_base=f['anchor_position'],extent_m=f['anchor_extent'],support_footprint_base=f['reference_footprint'])
 started=time.monotonic()
 try:
  r=t.segment_with_support_fallback(D,K,T,t.Config(),a,a,reference);actual=dict(result='segmentation',matching_candidates=len(t._matching_candidates(r.candidates,a,a,t.Config())),metrics=r.metrics)
 except ValueError as e:actual=dict(result='rejected',reason=str(e))
 elapsed=time.monotonic()-started;prior={k:v for k,v in expected['with_independent_audit'].items() if k!='elapsed_sec'}
 assert actual==prior,(name,actual,prior)
 rows.append(dict(frame=name,unchanged_decision_and_metrics=True,result=actual,elapsed_sec=elapsed))
 if name in ['initial','pregrasp_mid','before_close']:
  times=[]
  for i in range(3):
   started=time.monotonic()
   try:t._audit_current_support(D,K,T,t.Config(),a,reference)
   except ValueError:pass
   times.append(time.monotonic()-started)
  timings.append(dict(frame=name,audit_sec=times))
report=dict(source=str(Path(t.__file__)),source_sha256=hashlib.sha256(Path(t.__file__).read_bytes()).hexdigest(),all_saved_frames_equivalent=True,frames=rows,audit_timing=timings)
(p/'final_validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(dict(frames=len(rows),all_saved_frames_equivalent=True,audit_timing=timings),indent=2))
