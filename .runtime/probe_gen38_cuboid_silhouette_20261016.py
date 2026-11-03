"""Offline cuboid silhouette feasibility only; no new qualification contract.

All raw source arrays remain unchanged. Numerical fits prove existence of
examples only, not completeness of the continuous common contact-face family.
"""
import hashlib,json,math,os,sys,traceback
from pathlib import Path
W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
D=E/'ros_log/full_latest_gen36_restart_20261017T012037Z'
G=E/'grasp_records/20261017T032113.476278Z_a84cc89fc53e4c2b82680749d78b9b9f'
I=D/'verification/gen37_actual_attempt1_source_images_r1'
R=D/'verification/gen37_box_source_replay_r1'
O=D/'verification/gen38_cuboid_silhouette_probe_r1'
assert Path('/media/zhuyupei/SanStand').is_mount() and 'sitecustomize' in sys.modules
assert not (O/'result.json').exists()
(O/'tmp').mkdir(exist_ok=True)
os.environ.update(TMPDIR=str(O/'tmp'),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
 return h.hexdigest()
read=lambda p:json.loads(Path(p).read_text())
def serial(x):
 if hasattr(x,'tolist'):return x.tolist()
 if hasattr(x,'item'):return x.item()
 raise TypeError(type(x).__name__)
def save(name,data):(O/name).write_text(json.dumps(data,ensure_ascii=False,indent=2,default=serial,allow_nan=False)+'\n')
report=dict(passed=False,diagnostic_only=True,motion_authorized=False,production_modified=False,
 ros_initialized=False,temperature_values_read=False,guard_scope='Python socket monkeypatch and /dev audit hook; not OS/C isolation')
try:
 import cv2,numpy as np
 # Installed SciPy still imports this removed NumPy alias. This isolated
 # process compatibility alias does not change geometry or production files.
 if not hasattr(np,'typeDict'):np.typeDict=np.sctypeDict
 from scipy.optimize import least_squares
 cv2.setNumThreads(1)
 release=read(D/'verification/gen37_release_manifest.json');before={p:sha(W/p) for p in release['source_sha256']};assert before==release['source_sha256'];save('source_sha256_before.json',before)
 extraction=read(I/'receipt.json');assert sha(G/'manifest.json')==extraction['source_sha256'][str(G/'manifest.json')]
 ap=G/'keyframes_audits/0e5ebbd3c562cfab2617b338045f554ffc0920a23626fbc2a49cc474cc9d798d.json';a=read(ap);manifest=read(G/'manifest.json');assert manifest['state']=='complete' and manifest['recorder_closed']
 assert sha(ap)==next(r['sha256'] for r in manifest['files'] if r['path']==str(ap.relative_to(G)))
 q=a['remote_diagnostics']['tabletop_geometry']['regular_box_inference'];stamp=q['source_stamp_ns'];raw={}
 for role,suffix in (('color','.png'),('depth','.npy'),('mask','.png')):
  p=I/(str(stamp)+'_'+role+suffix);assert sha(p)==extraction['output_sha256'][p.name]
  raw[role]=np.load(p,allow_pickle=False) if role=='depth' else cv2.imread(str(p),cv2.IMREAD_UNCHANGED)
  assert hashlib.sha256(raw[role].tobytes(order='C')).hexdigest()==q['retained_pixel_sha256'][role+'_sha256']
 camera_path=D/'gen37_runtime/gen37_grasp_scene_r2/camera_configuration.json';camera=read(camera_path);K=np.array([camera[k] for k in ('fx','fy','cx','cy')]);scale=camera['depth_scale']
 T=np.asarray(a['snapshot_transform']['T_base_optical']);g=a['replay_geometry'];n=np.asarray(g['support_normal_base']);offset=g['support_offset_m']
 fit=np.load(R/'exact_current_top_fit.npz');top=fit['top'];points=fit['points'];uvraw=fit['uv'];A=fit['axes'];tn=fit['tn'];origin=fit['origin'];top_uv=uvraw[fit['keep']]
 # Only 9912 direct raw depth points / 9862 direct top points are used. No
 # registered historical or fabricated 1832-point cloud enters this probe.
 rawuv=np.column_stack(np.where((raw['mask']>0)&(raw['depth']>0)))[:,::-1]
 assert np.array_equal(uvraw,rawuv)
 zz=raw['depth'][rawuv[:,1],rawuv[:,0]].astype(float)*scale
 pc=np.column_stack(((rawuv[:,0]-K[2])*zz/K[0],(rawuv[:,1]-K[3])*zz/K[1],zz));pcheck=pc@T[:3,:3].T+T[:3,3]
 assert np.allclose(pcheck,points,rtol=0,atol=1e-14)
 mask=raw['mask']>0;color=raw['color'];depth=raw['depth'];height=q['top_height_m'];rms=q['top_plane_rms_m'];cosine=q['camera_top_cosine']
 pixel=float(np.median(zz[fit['keep']]))/min(K[:2])/cosine;plane_slab=max(.001,2*rms+scale)
 coord=top@A[:,:2];lower,upper=coord.min(0),coord.max(0);size0=upper-lower;center0=(upper+lower)/2
 v1_height_floor=max(.001,2*rms+scale,.5*float(size0.max())*math.sin(math.radians(q['top_support_angle_deg'])))
 corners0=np.array([[lower[0],lower[1]],[upper[0],lower[1]],[upper[0],upper[1]],[lower[0],upper[1]]])@A[:,:2].T+n*(height-offset)
 baseline_top_vertical_deviation=float(np.max(np.abs((corners0-origin)@tn)))/float(tn@n)
 height_u=max(v1_height_floor,baseline_top_vertical_deviation+plane_slab/float(tn@n))
 edge_floor=plane_slab+.5*pixel
 full_edges=cv2.Canny(cv2.cvtColor(color,cv2.COLOR_BGR2GRAY),50.,100.);ey,ex=np.where(full_edges);edge_pixels=np.column_stack((ex,ey))
 distance=cv2.distanceTransform((full_edges==0).astype(np.uint8),cv2.DIST_L2,cv2.DIST_MASK_PRECISE)
 contours,_=cv2.findContours(mask.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE);actual_contour=max(contours,key=cv2.contourArea).reshape(-1,2)
 eps=math.sqrt(8.)
 def project(p):
  pc=(p-T[:3,3])@T[:3,:3]
  return np.column_stack((pc[:,0]/pc[:,2]*K[0]+K[2],pc[:,1]/pc[:,2]*K[1]+K[3]))
 def cuboid(params):
  dc0,dc1,dl,dw,yaw,dh=params;rot=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
  basis=A[:,:2]@rot;center=A[:,:2]@(center0+np.array([dc0,dc1]));dims=size0+np.array([dl,dw]);hh=height+dh
  xy=np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*dims/2
  bottom=center+xy@basis.T-offset*n;vertices=np.r_[bottom,bottom+hh*n];uv=project(vertices)
  indices=cv2.convexHull(uv.astype(np.float32),returnPoints=False).reshape(-1);hull=uv[indices]
  return dict(vertices=vertices,pixels=uv,hull=hull,hull_indices=indices,axes=np.column_stack((basis,n)),size=np.r_[dims,hh],center=center+n*(hh/2-offset),params=params)
 def samples(model,full=False,count=101):
  hull=model['hull'];fractions=np.linspace(0.,1.,count) if full else np.linspace(.08,.92,count)
  return np.concatenate([hull[j]+np.outer(fractions,hull[(j+1)%len(hull)]-hull[j]) for j in range(len(hull))])
 def interpolate(pixels):
  x=pixels[:,0];y=pixels[:,1];x0=np.floor(x).astype(int);y0=np.floor(y).astype(int);dx=x-x0;dy=y-y0
  if np.any(x0<0)|np.any(x0>=distance.shape[1]-1)|np.any(y0<0)|np.any(y0>=distance.shape[0]-1):return np.full(len(pixels),100.)
  return ((1-dx)*(1-dy)*distance[y0,x0]+dx*(1-dy)*distance[y0,x0+1]+(1-dx)*dy*distance[y0+1,x0]+dx*dy*distance[y0+1,x0+1])
 def exact_dist(pixels):
  return np.concatenate([np.min(np.linalg.norm(chunk[:,None,:]-edge_pixels[None,:,:],axis=2),axis=1) for chunk in np.array_split(pixels,max(1,len(pixels)//80))])
 def evaluate(params):
  m=cuboid(params);s=samples(m);nearest=exact_dist(s);full=exact_dist(samples(m,True));poly=np.zeros(mask.shape,np.uint8);cv2.fillConvexPoly(poly,np.rint(m['hull']).astype(np.int32),1);interior=cv2.erode(poly,np.ones((3,3),np.uint8))>0
  coverage=float(np.mean(mask[interior]));contour_distance=np.array([abs(cv2.pointPolygonTest(m['hull'].astype(np.float32),(float(p[0]),float(p[1])),True)) for p in actual_contour])
  cornerdist=exact_dist(m['hull']);topcorners=m['vertices'][4:];plane_res=(topcorners-origin)@tn
  candidate_hu=max(height_u,float(np.max(np.abs(plane_res)))/float(tn@n)+plane_slab/float(tn@n))
  local=(points-m['center'])@m['axes'];half=m['size']/2;point_excess=np.maximum(0,np.abs(local)-half-[edge_floor,edge_floor,candidate_hu]);maxex=float(point_excess.max())
  heights_m=top@n+offset;plane_top_u=float(np.max(np.abs(plane_res)))
  return dict(parameters=params,axes=m['axes'],center=m['center'],size_xyz_m=m['size'],vertices_base=m['vertices'],vertices_pixels=m['pixels'],silhouette_hull_pixels=m['hull'],silhouette_vertex_indices=m['hull_indices'],
   original_8_to_92pct_canny_maxima_px=nearest.reshape(-1,101).max(1),full_0_to_100pct_canny_maxima_px=full.reshape(-1,101).max(1),corner_canny_distances_px=cornerdist,
   original_sqrt8_canny_gate=bool(nearest.max()<=eps),full_edge_and_corner_sqrt8_gate=bool(full.max()<=eps and cornerdist.max()<=eps),
   original_mask_coverage=coverage,original_mask_88pct_gate=bool(coverage>=.88),actual_mask_contour_distance_to_silhouette_max_px=float(contour_distance.max()),actual_mask_contour_distance_to_silhouette_p95_px=float(np.quantile(contour_distance,.95)),
   raw_current_point_excess_after_existing_budget_m=maxex,original_1mm_measured_excess_gate=bool(maxex<=.001),
   top_corner_to_measured_plane_max_m=plane_top_u,top_plane_slab_m=plane_slab,height_uncertainty_floor_m=candidate_hu,edge_uncertainty_floor_m=edge_floor,
   diagnostic_projection_feasible=bool(nearest.max()<=eps and coverage>=.88 and maxex<=.001),
   qualified_model=False,limitation='Projection feasibility is not a new model qualification or complete uncertainty family.')
 baseline=evaluate(np.zeros(6));save('baseline_cuboid.json',baseline)
 # Each searched parameter stays inside the existing 12mm side-proposal and
 # +/-8deg orientation windows; height stays inside the unchanged empirical
 # top/support height budget. Existence fits do not certify all feasible fits.
 pos_radius=.012;angle_radius=math.radians(8.);outer_bounds=np.array([pos_radius,pos_radius,2*pos_radius,2*pos_radius,angle_radius,height_u])
 # This inner fit box guarantees each footprint edge offset stays within the
 # original12mm radius (center<=6mm plus half-size change<=6mm).
 bounds=np.array([pos_radius/2,pos_radius/2,pos_radius,pos_radius,angle_radius,height_u])
 def residual(params):
  model=cuboid(params);s=samples(model,full=True,count=31);rgb=interpolate(s)
  local=(top-model['center'])@model['axes'];excess=np.maximum(0,np.abs(local)-model['size']/2-[edge_floor,edge_floor,height_u])
  # Preserve measured geometry in each candidate; this soft term only helps
  # the numerical search. Final gates above always evaluate exact evidence.
  geom=np.array([max(0.,float(excess.max())-.0008)/.00005])
  prior=np.asarray(params)/np.array([.003,.003,.006,.006,math.radians(2.),height_u])*0.02
  return np.r_[rgb,geom,prior]
 starts=[np.zeros(6)]
 for yaw in (-math.radians(.5),math.radians(.5)):
  x=np.zeros(6);x[4]=yaw;starts.append(x)
 for shift in (-.0003,.0003):
  x=np.zeros(6);x[0]=shift;starts.append(x)
 fits=[]
 for j,start in enumerate(starts):
  result=least_squares(residual,start,bounds=(-bounds,bounds),max_nfev=150,diff_step=1e-4,xtol=1e-10,ftol=1e-10,gtol=1e-10)
  row=evaluate(result.x);row.update(start_index=j,nfev=result.nfev,cost=result.cost,optimizer_success=result.success);fits.append(row)
 save('numerical_existence_fits.json',dict(diagnostic_only=True,complete_continuous_family=False,fits=fits))
 candidates=[r for r in fits if r['diagnostic_projection_feasible']];best=min(fits,key=lambda r:max(r['original_8_to_92pct_canny_maxima_px']))
 # A conservative outer parameter domain retains all unsearched competing
 # orientation/position/size explanations in this probe's stated domain.
 # Its width bound is deliberately not replaced by numerical-fit spread.
 maxdims=size0+2*pos_radius;theta=min(angle_radius,math.atan2(maxdims[0],maxdims[1]));narrow_upper=float(maxdims[1]*math.cos(theta)+maxdims[0]*math.sin(theta));required_upper=narrow_upper+.001
 # Necessary-condition witnesses, not an infeasibility proof: raw top
 # footprints project to support. Show sensitivity to exact-vs-budgeted
 # interpretation rather than discarding these measurements.
 bottom_points=top-(top@n+offset)[:,None]*n;bottom_pixels=project(bottom_points);mask_hull=cv2.convexHull(actual_contour.astype(np.float32));signed=np.array([cv2.pointPolygonTest(mask_hull,(float(p[0]),float(p[1])),True) for p in bottom_pixels])
 save('family_outer_bound.json',dict(complete_feasible_family_enumerated=False,outer_parameter_domain_retained=True,parameter_names=['center_axis0_delta_m','center_axis1_delta_m','size0_delta_m','size1_delta_m','yaw_delta_rad','height_delta_m'],parameter_min=-outer_bounds,parameter_max=outer_bounds,numerical_fit_inner_domain_min=-bounds,numerical_fit_inner_domain_max=bounds,domain_semantics='All fits in stated existing side-search windows are retained as possible until excluded; bounding outer domain includes incompatible candidates and is not a qualified uncertainty interval. Independent center/size outer bounds relax their correlation only for conservative bounding, not for numerical proposal search.',conservative_narrow_direction_width_upper_m=narrow_upper,fit_clearance_total_m=.001,conservative_required_opening_upper_m=required_upper,physical_opening_m=.05,opening_certified=False,no_using_fitted_sample_spread_as_u=True,
  direct_top_footprint_bottom_projection=dict(raw_top_count=len(top),outside_mask_convex_hull_sqrt8_count=int(np.count_nonzero(signed < -eps)),worst_signed_distance_px=float(signed.min()),warning='Only six raw points violate by a maximum0.55px beyond tolerance; not robust proof of global infeasibility under empirical depth/plane uncertainty.')))
 vis=color.copy();m=cuboid(best['parameters']);cv2.polylines(vis,[np.rint(m['hull']).astype(np.int32)],True,(255,0,255),1);cv2.polylines(vis,[np.rint(m['pixels'][4:]).astype(np.int32)],True,(0,255,0),1);cv2.drawContours(vis,[actual_contour.reshape(-1,1,2)],-1,(0,255,255),1);cv2.imwrite(str(O/'best_cuboid_overlay.png'),vis);cv2.imwrite(str(O/'best_cuboid_crop.png'),cv2.resize(vis[320:450,215:380],(660,520),interpolation=cv2.INTER_NEAREST))
 after={p:sha(W/p) for p in before};assert after==before;save('source_sha256_after.json',after)
 boundary=Path(os.environ['OFFLINE_BOUNDARY_LOG']);attempts=boundary.read_text().splitlines() if boundary.exists() else [];assert not attempts
 report.update(passed=True,source_count=len(before),source_sha256_unchanged=True,source_stamp_ns=stamp,source_target_identity=q['source_target_identity'],raw_direct_depth_point_count=len(points),raw_direct_top_point_count=len(top),historical_registered_points_used=False,
  raw_pixel_sha256=q['retained_pixel_sha256'],inputs_sha256={str(p):sha(p) for p in [ap,G/'manifest.json',I/'receipt.json',camera_path,R/'exact_current_top_fit.npz']},script_sha256=sha(__file__),external_boundary_attempts=attempts,
  original_gate_constants=dict(canny_distance_px=eps,angle_search_rad=angle_radius,side_search_radius_m=pos_radius,mask_coverage=.88,measured_model_excess_m=.001,plane_slab_m=plane_slab),
  numerical_projection_feasible_count=len(candidates),complete_uncertainty_family=False,best_example=best,conservative_required_opening_upper_m=required_upper,
  qualified_model=False,contact_face_family_certified=False,conclusion='Eight-corner projection is a better geometric explanation of the barcode side than forcing all mask rays onto the top plane. Numerical existence does not establish an exhaustive common contact-face/uncertainty family. No production qualification or motion permission is issued.',
  minimum_remaining_work=['Separate top-face evidence from visible side/silhouette correspondence using existing raw depth; never overwrite unknown pixels or treat printed inner edges as outside boundaries.','Bound the continuous cuboid pose/size/height family and all competing outer-edge correspondences, including calibrated plane/raster uncertainty. Keep this bound in source-signed evidence and validate projected exterior/depth/contact/collision consistency.','If that inverse-projection family is too large a change, keep the existing hard rejection and use the already qualified source through a separately reviewed registration/source-lineage contract; a point-fit cuboid cannot replace those contracts.'],
  limitations=['Candidate fitting is bounded but not exhaustive. A finite collection of good fits is not a common uncertainty envelope.','Existing mask coverage and RGB edge thresholds are diagnostic checks here, not an existing validated cuboid silhouette model kind.','The broad conservative width bound covers the complete stated parameter domain and can include infeasible boxes; it cannot certify a50mm opening.','No fabricated fused cloud, ROS initialization, services, publishers, subscriptions, hardware, or temperature reads.'])
except BaseException as exc:report.update(error_type=type(exc).__name__,error=str(exc),traceback=traceback.format_exc())
finally:
 save('result.json',report)
 print(json.dumps(dict(output=str(O),passed=report['passed'],error=report.get('error'),feasible=report.get('numerical_projection_feasible_count'),best_rgb_max=float(max(report['best_example']['original_8_to_92pct_canny_maxima_px'])) if report.get('best_example') else None,opening_upper=report.get('conservative_required_opening_upper_m')),ensure_ascii=False))
 if not report['passed']:sys.exit(1)
