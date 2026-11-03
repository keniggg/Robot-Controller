#!/usr/bin/env python3
"""Offline crop-only counterfactual; no production edit or execution authority."""
from pathlib import Path
import inspect,json,hashlib,time
import numpy as np

def observed_bounds(rgb_valid,desired,first,proposal):
    dx0,dy0,dx1,dy1=map(int,desired)
    px0,py0,px1,py1=map(int,proposal)
    rx0=min(first[0],px0-4);ry0=min(first[1],py0-4)
    rx1=max(first[2],px1+4);ry1=max(first[3],py1+4)
    if (not 0<=dx0<=rx0<rx1<=dx1<=rgb_valid.shape[1]
            or not 0<=dy0<=ry0<ry1<=dy1<=rgb_valid.shape[0]
            or not np.all(rgb_valid[ry0:ry1,rx0:rx1])):
        raise ValueError('rgb_search_contains_unobserved_rgb')
    bad=~rgb_valid[dy0:dy1,dx0:dx1]
    prefix=np.vstack((np.zeros((1,bad.shape[1]),np.int32),np.cumsum(bad,axis=0,dtype=np.int32)))
    best=None;score=None
    for y0 in range(dy0,ry0+1):
        for y1 in range(ry1,dy1+1):
            columns=prefix[y1-dy0]-prefix[y0-dy0]>0
            if np.any(columns[rx0-dx0:rx1-dx0]):continue
            left=np.flatnonzero(columns[:rx0-dx0]);right=np.flatnonzero(columns[rx1-dx0:])
            x0=dx0 if not len(left) else dx0+int(left[-1])+1
            x1=dx1 if not len(right) else rx1+int(right[0])
            key=(-(x1-x0)*(y1-y0),x0,y0,x1,y1)
            if score is None or key<score:score=key;best=(x0,y0,x1,y1)
    if best is None or not np.all(rgb_valid[best[1]:best[3],best[0]:best[2]]):
        raise ValueError('rgb_search_contains_unobserved_rgb')
    return best

src=Path(__file__).with_name('trace_gen23_rgb_valid_crop.py')
g={};exec(compile(src.read_text().split('rows=[]')[0],str(src),'exec'),g)
rgbmod=g['rgbmod'];original=rgbmod.instance_mask_from_seed
fn=inspect.getsource(original)
needle="        # Unobserved black padding must not train a foreground/background\n"
insert="        if spatial is not None and rgb_valid is not None and not np.all(rgb_valid[ay0:ay1, ax0:ax1]):\n            ax0, ay0, ax1, ay1 = observed_bounds(rgb_valid, (ax0, ay0, ax1, ay1), first_bounds, box)\n"
assert fn.count(needle)==1
env=dict(rgbmod.__dict__,observed_bounds=observed_bounds);exec(compile(fn.replace(needle,insert+needle),'<counterfactual_rgb_valid_crop>','exec'),env)
rgbmod.instance_mask_from_seed=env['instance_mask_from_seed']
OUT=g['ROOT']/'valid_background_counterfactual';OUT.mkdir(exist_ok=True);rows=[]
try:
 for row in [r for r in g['meta']['files'] if 'view2_arrival' in r['roles']]:
  a,rgb,dep,T,valid=g['frame'](row);ns=row['source_stamp_ns'];r=dict(source_stamp_ns=ns,online_evaluated=row['online_evaluated']);start=time.perf_counter()
  try:
   seg=g['table'].segment_with_support_fallback(dep,g['K'],T,g['cfg'],g['anchor'],g['last'],g['plane'],prefer_current_support=True)
   confirmed,candidate,staged,evidence=g['helper'].observe(seg,rgb,dep,g['K'],T,ns,rgb_valid=valid)
   r.update(passed=True,confirmed=confirmed,metrics=candidate.metrics,evidence=evidence)
  except ValueError as exc:r.update(passed=False,reason=str(exc))
  r['offline_wall_sec']=time.perf_counter()-start;rows.append(r)
  print(json.dumps({k:r.get(k) for k in ('source_stamp_ns','online_evaluated','passed','confirmed','reason','offline_wall_sec')}),flush=True)
finally:rgbmod.instance_mask_from_seed=original
report=dict(scope='CROP_ONLY_OFFLINE_COUNTERFACTUAL_NO_AUTHORITY',reference_mask_bitexact=g['reference_equal'],
 source_sha256={rgbmod.__file__:hashlib.sha256(Path(rgbmod.__file__).read_bytes()).hexdigest()},rows=rows,
 change='Only trim optional spatial background to maximum-area all-observed rectangle, retaining original first16 and prism plus four-pixel guard. All original RGB, current depth, identity, support and three-frame gates run unchanged.',
 limitations=['12 saved frames, only two exact online failures; not a live timing or execution proof.'])
(OUT/'result.json').write_text(json.dumps(report,indent=2)+'\n');print('RESULT',OUT/'result.json')
