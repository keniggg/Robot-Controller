"""Directional-budget counterfactual only; no production budget is changed."""
import pathlib,json,math,sys
from copy import deepcopy
import numpy as np
from alicia_flexible_grasp.vision.cuboid_rgb_interval import _plane,_joint
from alicia_flexible_grasp.vision.outer_rgb_rectangle import _trig_range
D=pathlib.Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen36_restart_20261017T012037Z');O=D/'verification/gen38_cuboid_silhouette_probe_r1'
assert pathlib.Path('/media/zhuyupei/SanStand').is_mount() and 'sitecustomize' in sys.modules
r=json.loads((O/'new_interval_latest.json').read_text());assert r['ok'] and r['validator_passed'];a=r['audit']
A=np.array(a['nominal_axes_base']);T=np.array(a['T_base_optical']);K=np.array(a['intrinsics']);n=A[:,2];h=r['nominal_height_m'];hu=r['height_uncertainty'];d=a['support_offset_m'];lo=np.array(r['lower']);hi=np.array(r['upper']);size=hi-lo
groups=deepcopy(a['groups']);breakdown=[]
for i,g in enumerate(groups):
 p=np.array(g['metric_points']);pixels=np.array(g['line_pixels']);directional=0.
 if g['plane']=='top':
  for dh in (-hu,hu):
   displacement=_plane(pixels,h+dh,n,d,T,K)@A[:,:2]-p
   for x,y in displacement:
    lower,upper=_trig_range(x,y,-math.radians(8.),math.radians(8.)) if i%2 else _trig_range(y,-x,-math.radians(8.),math.radians(8.))
    directional=max(directional,abs(lower),abs(upper))
 old=g['height_projection_bound_m'];g['height_projection_bound_m']=directional
 g['strip_radius_m']=g['maximum_line_residual_m']+g['sampling_bound_m']+directional
 breakdown.append(dict(side=i,plane=g['plane'],euclidean_height_projection_m=old,directional_height_projection_m=directional,all_raw_responses_retained=len(p),normal_search_interval_rad=[-math.radians(8.),math.radians(8.)]))
j=_joint(groups,fixed_theta=0.);errors=np.array([max(j['position_intervals'][i]['error_m'] for i in (1,3)),max(j['position_intervals'][i]['error_m'] for i in (0,2))]);yaw=j['yaw_uncertainty'];shift=np.abs((j['lower']+j['upper']-lo-hi)/2);sizedelta=np.abs((j['upper']-j['lower'])-(hi-lo))/2
def positive_rotation_support(x,y,angle):
 turn=min(angle,math.atan2(y,x))
 return x*math.cos(turn)+y*math.sin(turn)
axis_budget=[]
for axis in range(2):
 axis_budget.append(positive_rotation_support(errors[axis],errors[1-axis],yaw)+shift[axis]+positive_rotation_support(sizedelta[axis],sizedelta[1-axis],yaw))
axis_budget=np.array(axis_budget)
nominal_yaw_width=positive_rotation_support(size[1],size[0],yaw)
oldreq=positive_rotation_support(size[1],size[0],r['yaw_uncertainty'])+2*r['edge_uncertainty']+.001
newreq=nominal_yaw_width+2*axis_budget[1]+.001
oldjoint=a['interval'];oldface=np.array([max(oldjoint['position_intervals'][i]['error_m'] for i in (1,3)),max(oldjoint['position_intervals'][i]['error_m'] for i in (0,2))]);oldy=r['yaw_uncertainty'];oldshift=np.abs((np.array(oldjoint['lower'])+np.array(oldjoint['upper'])-lo-hi)/2);olddelta=np.abs((np.array(oldjoint['upper'])-np.array(oldjoint['lower']))-size)/2
oldaxis=np.array([positive_rotation_support(oldface[i],oldface[1-i],oldy)+oldshift[i]+positive_rotation_support(olddelta[i],olddelta[1-i],oldy) for i in (0,1)])
anisotropic_same_norm_req=positive_rotation_support(size[1],size[0],oldy)+2*oldaxis[1]+.001
out=dict(diagnostic_only=True,production_modified=False,common_height_interval_unchanged=a['common_height_interval_m'],nominal_cuboid_unchanged=True,all_raster_groups_and_competing_responses_retained=True,old_scalar_required_opening_upper_m=oldreq,old_4face_position_intervals=oldjoint['position_intervals'],old_scalar_edge_budget_m=r['edge_uncertainty'],old_face_axis_error_m=oldface.tolist(),old_norm_only_anisotropic_axis_budget_m=oldaxis.tolist(),old_norm_anisotropic_required_opening_upper_m=anisotropic_same_norm_req,normal_projection_breakdown=breakdown,directional_joint={k:(v.tolist() if hasattr(v,'tolist') else v) for k,v in j.items()},directional_axis_budget_m=axis_budget.tolist(),directional_required_opening_upper_m=newreq,fit_clearance_total_m=.001,independent_reserve_m=.002,directional_opening_plus_reserve_m=newreq+.002,physical_opening_m=.05,opening_with_reserve_passes=bool(newreq+.002<=.05),explanation='Each side keeps its own strip error; height displacement is maximized analytically over the full unchanged original8deg normal interval instead of using Euclidean length. Both top-plane height endpoints and every raw response are retained. This diagnostic is not a model/evidence schema or motion approval.',limitations=['Anisotropic common-family consumption must be shared by collision and opening, with source/registration projection and independent review.','Legacy scalar may remain for conservative backward compatibility, but a schema3 vector cannot merely reduce scalar consumers; all downstream geometry must consume the same signed family.'])
(O/'directional_budget_diagnostic.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(old_scalar=oldreq,old_norm_anisotropic=anisotropic_same_norm_req,directional_axis_budget=axis_budget.tolist(),directional_opening=newreq,with_reserve=newreq+.002,pass_reserve=bool(newreq+.002<=.05))))
