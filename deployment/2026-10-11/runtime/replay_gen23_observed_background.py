#!/usr/bin/env python3
"""Frozen gen23 second station through actual deployed-shape production APIs."""
from pathlib import Path
import json,hashlib,time
source=Path(__file__).with_name('trace_gen23_rgb_valid_crop.py');g={}
exec(compile(source.read_text().split('rows=[]')[0],str(source),'exec'),g)
OUT=g['ROOT']/'observed_background_production_replay';OUT.mkdir(exist_ok=True)
modules=['rgb_instance','tabletop','observation_arrival_confirmation','arrival_fragment_group']
files=[Path(__import__('alicia_grasp_modes.'+name,fromlist=['']).__file__) for name in modules]
before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files};rows=[]
for row in [r for r in g['meta']['files'] if 'view2_arrival' in r['roles']]:
 a,rgb,dep,T,valid=g['frame'](row);ns=row['source_stamp_ns'];r=dict(source_stamp_ns=ns,online_evaluated=row['online_evaluated']);start=time.perf_counter()
 try:
  seg=g['table'].segment_with_support_fallback(dep,g['K'],T,g['cfg'],g['anchor'],g['last'],g['plane'],prefer_current_support=True)
  confirmed,candidate,staged,evidence=g['helper'].observe(seg,rgb,dep,g['K'],T,ns,rgb_valid=valid)
  r.update(passed=True,confirmed=confirmed,metrics=candidate.metrics,evidence=evidence)
 except ValueError as exc:r.update(passed=False,reason=str(exc))
 r['offline_wall_sec']=time.perf_counter()-start;rows.append(r)
 print(json.dumps({k:r.get(k) for k in ('source_stamp_ns','online_evaluated','passed','confirmed','reason','offline_wall_sec')}),flush=True)
after={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files};assert before==after
report=dict(scope='ACTUAL_PRODUCTION_OFFLINE_NO_ROS_NO_AUTHORITY',record=g['meta']['record'],source_sha256=before,
 reference_source_stamp_ns=g['role']('view2_last_pre_motion')['source_stamp_ns'],
 reference_mask_bitexact=g['reference_equal'],rows=rows,
 limitations=['12 recorded view2 frames, only two exactly evaluated online; no reconstruction of unavailable sources.',
 'Actual original support, RGB, member ownership, final geometry, appearance and three-frame checks; not live age/commit or hardware execution proof.'])
(OUT/'result.json').write_text(json.dumps(report,indent=2)+'\n');print('RESULT',OUT/'result.json')
