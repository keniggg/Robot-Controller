"""Finite original nine Hough combinations for initial top/side ambiguity."""
from pathlib import Path
from itertools import product
import json,sys,inspect
import numpy as np
W=Path(__file__).resolve().parents[1];sys.path.insert(0,str(W/'src/alicia_flexible_grasp_supervisor/src'))
from alicia_flexible_grasp.vision import regular_box_inference as box,outer_rgb_rectangle as outer
C=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen29_restart_20261016T000728Z');I=C/'verification/gen30_attempt1_closed_evidence_r1';O=C/'verification/gen31_hough_top_candidates_r1';O.mkdir(exist_ok=False)
x=np.load(I/'1792116749240888833_raw.npz');z=np.load(C/'verification/gen31_mask_axis_proposal_r1/1792116749240888833_original_basis.npz');T=z['T'];K=z['K'];mask=x['object_mask']>0;depth=x['depth_raw'];scale=9.999999747378752e-05;yy,xx=np.where(mask&(depth>0));zz=depth[yy,xx]*scale;p=np.column_stack(((xx-K[2])*zz/K[0],(yy-K[3])*zz/K[1],zz))@T[:3,:3].T+T[:3,3];coord=p[abs((p-z['origin'])@z['tn'])<.0015]@z['axes'][:,:2];state={}
def trace(f,e,arg):
 if f.f_code.co_name=='_visual_rectangle' and e=='return':state.update(f.f_locals)
 return trace
sys.settrace(trace)
try:box._visual_rectangle(x['color_bgr'],mask,depth,scale,T,K,z['origin'],z['tn'],z['axes'],coord.min(0),coord.max(0),float(z['pixel_size']))
finally:sys.settrace(None)
text=inspect.getsource(outer.infer_outer_rgb_rectangle)
text=text.replace('    lo, hi = silhouette.min(0), silhouette.max(0)','''    zz = depth[yy,xx].astype(float)*scale
    known = np.isfinite(zz)&(zz>0.)
    measured = np.column_stack(((xx-K[2])*zz/K[0],(yy-K[3])*zz/K[1],zz))@T[:3,:3].T+camera
    possible_top = ~known | (np.abs((measured-origin)@tn)<=.0015)
    silhouette = silhouette[possible_top]
    lo, hi = proposal_lower,proposal_upper''')
space=dict(vars(outer));exec(compile(text,'<isolated_hough_top>','exec'),space);rows=[]
for k,select in enumerate(product(*state['choices'])):
 space['proposal_lower']=np.array([select[3][0],select[0][0]]);space['proposal_upper']=np.array([select[1][0],select[2][0]])
 result=space['infer_outer_rgb_rectangle'](x['color_bgr'],mask,depth,scale,T,K,z['origin'],z['tn'],z['axes'],float(z['pixel_size']),float(z['rms']))
 rows.append(dict(candidate=k,proposal_lines=[v[2].tolist() for v in select],result=result))
(O/'result.json').write_text(json.dumps(dict(mode='isolated_diagnostic_no_production_change',source_stamp_ns=1792116749240888833,candidate_count=len(rows),rows=rows),default=lambda v:v.tolist(),indent=2)+'\n');print(json.dumps([dict(candidate=r['candidate'],ok=r['result']['ok'],reason=r['result'].get('reason'),audit=r['result']['audit']) for r in rows],default=lambda v:v.tolist()))
