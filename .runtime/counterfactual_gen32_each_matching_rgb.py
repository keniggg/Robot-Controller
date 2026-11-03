"""No gate edits: independently refine every original raw match for diagnostics."""
from pathlib import Path
import json,time,hashlib
import numpy as np,cv2
p=Path(__file__).with_name('replay_gen32_arrival_matching.py');g={};exec(compile(p.read_text().split('files=')[0],str(p),'exec'),g)
table=g['table'];rgbmod=g['rgbmod'];meta=g['meta'];frame=g['frame'];cfg=g['cfg'];anchor=g['anchor'];last=g['last'];plane=g['plane'];helper=g['helper']
OUT=g['D']/'each_matching_rgb_counterfactual';OUT.mkdir(exist_ok=True)
rows=[]
for row in meta['sources'][2:]:
    a,rgb,dep,T,K,valid=frame(row);ns=row['source_stamp_ns'];start=time.perf_counter()
    seg=table.segment_with_support_fallback(dep,K,T,cfg,anchor,last,plane,prefer_current_support=True)
    matches=table._matching_candidates(seg.candidates,anchor,last,cfg);computed=rgbmod._FrameRefinement(dep,K,T,helper.spatial_reference,cfg);outputs=[];cores=[];r=dict(source_stamp_ns=ns,raw_matching_count=len(matches),members=[])
    for i,(_,c) in enumerate(matches):
        t=time.perf_counter();item=dict(index=i,raw_points=int(np.count_nonzero(c.mask)),raw_position=c.position_base.tolist())
        try:
            refined=computed.refine(rgb,c,rgb_valid=valid)
            mask=c.mask>0;yy,xx=np.nonzero(mask);margin=max(8,int(round(16*min(dep.shape[1]/640.,dep.shape[0]/480.))))
            x0=max(0,int(xx.min())-margin);x1=min(dep.shape[1],int(xx.max())+margin+1);y0=max(0,int(yy.min())-margin);y1=min(dep.shape[0],int(yy.max())+margin+1);sl=np.s_[y0:y1,x0:x1]
            pc=T.T@np.asarray(c.metrics['support_plane_base']);heights=np.einsum('ijk,k->ij',computed.points,pc[:3])+pc[3]
            core,_,_=computed.confidence_labels(mask[sl],computed.points[sl],dep[sl],heights[sl],pc[:3],cfg,bgr=rgb[sl]);full=np.zeros(dep.shape,bool);full[sl]=core
            cores.append(full);outputs.append(refined)
            item.update(passed=True,rgb_pixels=int(np.count_nonzero(refined.mask)),measured_points=refined.metrics['valid_points'],core_pixels=int(full.sum()),core_outside_domain=int(np.count_nonzero(full&~computed.spatial['domain_mask'])),final_matches_original=bool(table._matching_candidates([refined],anchor,last,cfg)),metrics=refined.metrics)
            cv2.imwrite(str(OUT/(str(ns)+'_member'+str(i)+'_rgbmask.png')),refined.mask)
        except ValueError as exc:item.update(passed=False,reason=str(exc))
        item['wall_sec']=time.perf_counter()-t;r['members'].append(item)
    if len(outputs)==len(matches):
        r['pairwise_mask_iou']=[float(np.count_nonzero((outputs[i].mask>0)&(outputs[j].mask>0))/np.count_nonzero((outputs[i].mask>0)|(outputs[j].mask>0))) for i in range(len(outputs)) for j in range(i)]
        r['each_output_retains_every_native_core']=[[float(np.count_nonzero((c.mask>0)&core)/np.count_nonzero(core)) if np.any(core) else None for core in cores] for c in outputs]
    r['wall_sec']=time.perf_counter()-start;rows.append(r);print(json.dumps({k:r.get(k) for k in ['source_stamp_ns','pairwise_mask_iou','each_output_retains_every_native_core','wall_sec']}),flush=True)
report=dict(scope='OFFLINE_DIAGNOSTIC_NO_GATE_EDITS_NO_AUTHORITY',reference_checks=g['checks'],rows=rows,source_sha256={str(Path(rgbmod.__file__)):hashlib.sha256(Path(rgbmod.__file__).read_bytes()).hexdigest()})
(OUT/'result.json').write_text(json.dumps(report,indent=2)+'\n');print('RESULT',OUT/'result.json')
