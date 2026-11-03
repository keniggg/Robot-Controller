"""Read the already computed R30 four-pixel interval; no qualification rerun."""
import hashlib,importlib.util,json,sys,time
from pathlib import Path
from types import SimpleNamespace
W=Path(__file__).resolve().parents[1]
p=W/'.runtime/probe_gen51_r30_cuboid_4px_20261018.py'
spec=importlib.util.spec_from_file_location('r30_probe_guard',p);h=importlib.util.module_from_spec(spec);spec.loader.exec_module(h)
h.setup(SimpleNamespace(mode='interval_budget_r1',attempt=None))
wall=time.perf_counter();cpu=time.thread_time()
import numpy as np,cv2
cv2.setNumThreads(1)
sys.path.insert(0,str(W/'src/alicia_flexible_grasp_supervisor/src'))
import alicia_flexible_grasp.vision
C=W/'.runtime/gen51_cuboid_4px_candidate'
pins={'outer_rgb_rectangle.py':'19693a71aa3283592b0f42413b9ec254b414eb92b89b84c8777f67d32f0162c8','cuboid_rgb_interval.py':'1199f6ad9d0f55a573e16749000a18d5c54240d81e8b92148c46877d1b45e0bb'}
for basename,expected in pins.items():
 path=C/h.REL.replace('cuboid_rgb_interval.py',basename);h.pin(path,expected)
 name='alicia_flexible_grasp.vision.'+path.stem;spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);sys.modules[name]=m;spec.loader.exec_module(m);setattr(alicia_flexible_grasp.vision,path.stem,m)
from alicia_flexible_grasp.vision import cuboid_rgb_interval as c
input_path=h.ROOT/'contour4px_formal_r1/result.json';h.pin(input_path,'97d55677cc517802c6a542cd4ac085de4ab8e8aa82f0ed8156be874469296aff')
result=json.loads(input_path.read_bytes());a=result['output']['audit'];v=a['cuboid_rgb_interval']
A=np.asarray(v['nominal_axes_base']);lo=np.asarray(v['nominal_lower']);hi=np.asarray(v['nominal_upper']);T=np.asarray(v['T_base_optical']);K=np.asarray(v['intrinsics']);height=v['nominal_height_m'];d=v['support_offset_m'];axis=np.asarray(v['empirical_axis_uncertainty_m']);yaw=v['empirical_yaw_uncertainty_rad'];hu=v['empirical_height_uncertainty_m']
model=c._model(A,lo,hi,height,d,T,K);records=v['silhouette_raster_evidence']
pixels=np.unique(np.concatenate([np.asarray(r['corresponding_canny_pixels'],int)for r in records]),axis=0)
compatible=c._unknown_mask_rays(pixels[:,::-1],T,K,A,lo,hi,height,d,axis,yaw,hu)
rows=[]
for record,edge in zip(records,model['edges']):
 start,end=np.asarray(edge['start_pixel']),np.asarray(edge['end_pixel']);t=np.linspace(0.,1.,101);uv=start+np.outer(t,end-start);actual=np.asarray(record['corresponding_canny_pixels'])
 if edge['kind']=='vertical':
  vertices=np.asarray(model['vertices_base']);p0,p1=vertices[edge['start_vertex']],vertices[edge['end_vertex']];plane_heights=(p0+np.outer(t,p1-p0))@A[:,2]+d
 else:plane_heights=height if edge['kind']=='top'else 0.
 delta=(c._plane(actual,plane_heights,A[:,2],d,T,K)-c._plane(uv,plane_heights,A[:,2],d,T,K))@A[:,:2]
 maximum=np.max(np.abs(delta),axis=0)
 rows.append(dict(edge_index=record['edge_index'],plane=edge['kind'],maximum_absolute_tangent_residual_m=maximum.tolist(),existing_axis_budget_m=axis.tolist(),componentwise_covered=bool(np.all(maximum<=axis+1e-12)),projection_assumption='same nominal interpolated edge height; diagnostic only'))
out=dict(kind='existing_interval_contour_residual_coverage_diagnostic_only',input_path=str(input_path),input_sha256=h.sha(input_path),complete_canny_unique_pixel_count=len(pixels),all_five_cell_rays_enclosed_count=int(compatible.sum()),missing_cell_count=int((~compatible).sum()),missing_pixels_yx=pixels[~compatible,::-1].tolist(),all_existing_canny_cells_enclosed=bool(np.all(compatible)),axis_budget_m=axis.tolist(),height_budget_m=hu,yaw_budget_rad=yaw,edge_rows=rows,source_validation_preceded_measured_geometry_failure=True,full_qualification_ok=False,first_failure=result['output']['code'],measured_geometry_conflict=a['measured_geometry_conflict'],new_production_gate_added=False,additional_budget_applied=False,proposal_or_opening_or_motion_authorized=False,limits='Cell ray intersection establishes possible occupancy within the existing empirical enclosure, not measured depth, all physical interpretations, or a certified error bound. Actual Canny correspondences are checked; this does not claim every hypothetical four-pixel perturbation is covered.',helper_sha256=h.sha(__file__),guard_helper_sha256=h.sha(p),wall_sec=time.perf_counter()-wall,thread_cpu_sec=time.thread_time()-cpu,boundary_attempts=h.BOUNDARY)
rr=h.put(h.OUT/'result.json',out);receipt=h.put(h.OUT/'receipt.json',dict(result=rr,helper_sha256=h.sha(__file__),source_pins=pins,boundary_attempts=h.BOUNDARY));print(json.dumps(dict(result=rr,receipt=receipt,all_existing_canny_cells_enclosed=out['all_existing_canny_cells_enclosed'],componentwise_covered=all(r['componentwise_covered']for r in rows),missing_cell_count=out['missing_cell_count'])));assert not h.BOUNDARY
