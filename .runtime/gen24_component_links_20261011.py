from pathlib import Path
s=Path('.runtime/gen24_diagonal_component_probe_20261011.py').read_text();exec(s[:s.index('start=time.monotonic()')])
points=t._project(depth,K);plane=np.array(seg.candidates[0].metrics['support_plane_base']);pc=T.T@plane;heights=points@pc[:3]+pc[3];valid=np.isfinite(depth)&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m);rawfg=valid&(heights>=cfg.foreground_min_m)&(heights<=cfg.foreground_max_m)
cs=seg.candidates;links=[]
for i,c in enumerate(cs):
 for j,o in enumerate(cs[:i]):
  rows=[]
  for dy,dx in [(-1,-1),(-1,1),(1,-1),(1,1)]:
   yy,xx=np.nonzero(c.mask);y=yy+dy;x=xx+dx;ok=(y>=0)&(y<480)&(x>=0)&(x<640);yy,xx,y,x=yy[ok],xx[ok],y[ok],x[ok];ok=o.mask[y,x]>0;yy,xx,y,x=yy[ok],xx[ok],y[ok],x[ok]
   for y0,x0,y1,x1 in zip(yy,xx,y,x):
    rows.append(dict(a_yx=[int(y0),int(x0)],b_yx=[int(y1),int(x1)],distance_mm=float(np.linalg.norm(points[y0,x0]-points[y1,x1])*1000),depth_delta_mm=float(abs(depth[y0,x0]-depth[y1,x1])*1000),cross_depth_m=[float(depth[y0,x1]),float(depth[y1,x0])],cross_raw_foreground=[bool(rawfg[y0,x1]),bool(rawfg[y1,x0])],cross_height_mm=[float(heights[y0,x1]*1000),float(heights[y1,x0]*1000)]))
  links.append(dict(first=j,second=i,count=len(rows),rows=rows))
csrows=[]
for i,c in enumerate(cs):
 v=c.mask>0;csrows.append(dict(index=i,count=int(v.sum()),bbox=c.bbox,position_base=c.position_base.tolist(),extent_m=c.extent_m.tolist(),height_mm=np.percentile(heights[v]*1000,[0,10,50,90,100]).tolist(),original_match=bool(t._matching_candidates([c],tracker.anchor,tracker.last,cfg))))
origunion=np.logical_or.reduce([c.mask>0 for c in cs]);new=alt.candidates[0].mask>0
report=dict(candidates=csrows,links=links,old_union_count=int(origunion.sum()),new_count=int(new.sum()),added_pixels=int(np.count_nonzero(new&~origunion)),removed_pixels=int(np.count_nonzero(origunion&~new)),source=row['source_stamp_ns'])
(OUT/'native_component_links.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(dict(candidates=csrows,link_summary=[dict(first=r['first'],second=r['second'],count=r['count'],cross_valid=sum(any(v['cross_raw_foreground'])for v in r['rows']))for r in links],old_union_count=int(origunion.sum()),new_count=int(new.sum()),added_pixels=int(np.count_nonzero(new&~origunion)))))
