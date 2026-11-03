"""Exact-source 2D visibility diagnostic; does not infer physical top boundaries."""
import json,pathlib,hashlib,sys
import cv2,numpy as np
D=pathlib.Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen36_restart_20261017T012037Z');O=D/'verification/gen37_box_source_replay_r1';I=D/'verification/gen37_actual_attempt1_source_images_r1'
W=pathlib.Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
assert pathlib.Path('/media/zhuyupei/SanStand').is_mount() and 'sitecustomize' in sys.modules
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
source=read(O/'source_sha256_before.json');assert all(sha(W/p)==v for p,v in source.items())
s='1792207340733134031';c=cv2.imread(str(I/(s+'_color.png')));m=cv2.imread(str(I/(s+'_mask.png')),0)>0;d=np.load(I/(s+'_depth.npy'));e=cv2.Canny(cv2.cvtColor(c,cv2.COLOR_BGR2GRAY),50.,100.)
retained=e.copy();retained[cv2.dilate(m.astype(np.uint8),np.ones((5,5),np.uint8))==0]=0
ey,ex=np.where(e);ep=np.column_stack((ex,ey));ry,rx=np.where(retained);rep=np.column_stack((rx,ry))
a=read(O/'current_depth_axes.json');pix=np.asarray(a['mask_envelope_corners_px']);edge_reports=[]
for i in range(4):
 uv=pix[i]+np.outer(np.linspace(.08,.92,101),pix[(i+1)%4]-pix[i]);row=dict(edge=i)
 for label,points in [('full_current_canny',ep),('mask_dilated_5x5_canny',rep)]:
  distances=np.linalg.norm(uv[:,None,:]-points[None,:,:],axis=2);nearest=np.min(distances,axis=1);idx=int(np.argmax(nearest));near=points[int(np.argmin(distances[idx]))]
  row[label]=dict(maximum_distance_px=float(nearest[idx]),at_pixel=uv[idx].tolist(),nearest_edge_pixel=near.tolist(),passes_original_sqrt8=bool(nearest[idx]<=np.sqrt(8)))
 edge_reports.append(row)
w=[]
for p in ([364,343],[359,337],[244.39289944184281,338.1218250527358]):
 p=np.asarray(p);dist=np.linalg.norm(ep-p,axis=1);n=ep[int(np.argmin(dist))];dist_r=np.linalg.norm(rep-p,axis=1);nr=rep[int(np.argmin(dist_r))];x=int(round(p[0]));local=[dict(x=x0,mask_top_y=int(np.where(m[:,x0])[0].min()),mask_bottom_y=int(np.where(m[:,x0])[0].max())) for x0 in range(max(0,x-2),min(c.shape[1],x+3)) if m[:,x0].any()]
 w.append(dict(pixel=p.tolist(),nearest_full_canny=n.tolist(),full_canny_distance_px=float(dist.min()),nearest_retained_canny=nr.tolist(),retained_canny_distance_px=float(dist_r.min()),depth_raw=int(d[int(round(p[1])),x]),mask=bool(m[int(round(p[1])),x]),mask_column_extents=local))
contours,_=cv2.findContours(m.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE);contour=max(contours,key=cv2.contourArea).reshape(-1,2);rows=[]
for p in contour:
 dist=np.linalg.norm(ep-p,axis=1);n=ep[int(np.argmin(dist))];line=[]
 for j in range(4):
  v=pix[(j+1)%4]-pix[j];line.append(abs(float(np.cross(v,p-pix[j])))/np.linalg.norm(v))
 rows.append(dict(pixel=p.tolist(),edge=int(np.argmin(line)),canny_distance_px=float(dist.min()),nearest_canny=n.tolist(),depth_raw=int(d[p[1],p[0]])))
summary=[]
for j in range(4):
 group=[r for r in rows if r['edge']==j];worst=max(group,key=lambda r:r['canny_distance_px']);summary.append(dict(nearest_projected_edge=j,contour_points=len(group),maximum_actual_mask_contour_canny_distance_px=worst['canny_distance_px'],worst=worst,median_distance_px=float(np.median([r['canny_distance_px'] for r in group]))))
report=dict(kind='actual_mask_contour_rgb_diagnostic',source_stamp_ns=int(s),source_sha256_unchanged=True,source_count=len(source),script_sha256=sha(pathlib.Path(__file__)),current_mask_projected_rectangle_edge_distances=edge_reports,witnesses=w,actual_mask_contour_nearest_edge_summary=summary,contour_points=rows,
 conclusion='The actual 2D mask contour and right outer RGB edge are visible. A rectangular envelope built from all top-plane mask rays is not the observed mask contour: sparse upper-side mask pixels pull its upper boundary beyond most of the mask. Original Canny gating clips the true barcode side upper edge where it is outside the 5x5 dilated mask, so unsupported_edge0 is a mask/projection/top-vs-side issue rather than absent RGB outer-edge contrast.',
 limitations=['Full-image Canny diagnostic cannot be substituted into production gates: it may include background and visible prism-side edges, and does not prove physical top-plane correspondence.','2D silhouette/Canny support with zero boundary depth cannot distinguish top-plane boundary from visible prism side.','Contour side labels here are only nearest projected rectangle-edge labels; actual silhouette need not be a rectangle.'])
(O/'actual_mask_contour_rgb_support.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
vis=c.copy();vis[e>0]=(255,255,0);cv2.drawContours(vis,[contour.reshape(-1,1,2)],-1,(0,255,0),1);cv2.polylines(vis,[np.rint(pix).astype(np.int32)],True,(255,0,255),1)
cv2.imwrite(str(O/'near_outer_contour_crop.png'),cv2.resize(vis[320:450,215:380],(660,520),interpolation=cv2.INTER_NEAREST))
assert all(sha(W/p)==v for p,v in source.items())
print(json.dumps(dict(projected_edges=edge_reports,actual_mask_contour_maxima=[r['maximum_actual_mask_contour_canny_distance_px'] for r in summary]),ensure_ascii=False))
