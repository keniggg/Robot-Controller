#!/usr/bin/env python3
"""Offline recorded RGB-D through actual production arrival, not live authority."""
from pathlib import Path
import argparse,json,hashlib,time
import numpy as np,cv2
from alicia_grasp_modes.observation_arrival_confirmation import ObservationArrivalConfirmation
from alicia_grasp_modes.rgb_observation_handoff import _appearance,_mask
p=Path(__file__).with_name('trace_gen21_arrival_rgb.py');g={};exec(compile(p.read_text().split('rows=[]')[0],str(p),'exec'),g)
parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path)
args=parser.parse_args()
OUT=args.output if args.output is not None else g['ROOT']/'spatial_production_replay';OUT.mkdir(exist_ok=True)
hist,contour=_appearance(g['col'],_mask(g['refined'].mask,g['col'].shape[:2]));helper=ObservationArrivalConfirmation(g['cfg'],g['anchor'],g['last'],g['plane'],hist,contour,.5)
files=[Path(g['rgbmod'].__file__),Path(__import__('alicia_grasp_modes.observation_arrival_confirmation',fromlist=['']).__file__)];before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files};rows=[]
for row in (r for r in g['meta']['files'] if r['role']=='arrival'):
 _,rgb,depth,T,valid=g['frame'](row);ns=row['source_stamp_ns'];r=dict(source_stamp_ns=ns,online_evaluated=row['online_evaluated'],online_reason=row['online_reason']);start=time.perf_counter()
 try:
  seg=g['table'].segment_with_support_fallback(depth,g['K'],T,g['cfg'],g['anchor'],g['last'],g['plane'],prefer_current_support=True)
  confirmed,candidate,tracker,evidence=helper.observe(seg,rgb,depth,g['K'],T,ns,rgb_valid=valid)
  r.update(passed=True,confirmed=confirmed,metrics=candidate.metrics,evidence=evidence)
  cv2.imwrite(str(OUT/(str(ns)+'_mask.png')),candidate.mask)
 except ValueError as exc:r.update(passed=False,reason=str(exc))
 r['offline_wall_sec']=time.perf_counter()-start;rows.append(r);print(json.dumps({k:r.get(k) for k in ('source_stamp_ns','online_evaluated','passed','confirmed','reason','offline_wall_sec')}),flush=True)
after={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files};assert before==after
report=dict(scope='ACTUAL_PRODUCTION_OFFLINE_NO_EXECUTION_AUTHORITY',record=g['meta']['record'],source_sha256=before,last_reference_mask_bitexact=g['reference_equal'],rows=rows,limitations=['13 recorded frames, only four coincide with online evaluated sources; absent sources are not reconstructed.','Uses actual production segmentation/arrival/refinement, original appearance and three-frame qualification; no live ROS age/commit or execution proof.'])
(OUT/'result.json').write_text(json.dumps(report,indent=2)+'\n');print('RESULT',OUT/'result.json')
