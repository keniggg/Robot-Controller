"""Original measured-surface side masks, absolute height intervals and source contributions."""
exec(compile(open('.runtime/replay_gen29_contact_angle_window.py').read().split('start=time.monotonic();rows=[]')[0],'.runtime/replay_gen29_contact_angle_window.py','exec'))
from scipy.spatial import cKDTree
from alicia_flexible_grasp.grasp.gripper_geometry import _linear_quantiles,_MEASURED_SURFACE_TRIM_FRACTION
rows=[]
for angle in [40,42.17,42.2,43,44,44.78,45,50,53.843872185121214,55,60,65,65.5,70]:
 jaw=axis(angle);cross=np.cross(jaw,-normal);cross/=np.linalg.norm(cross);delta=surface.points_base-geometry.center_base
 j=delta@jaw;ins=delta@(-normal);cr=delta@cross;support=surface.points_base@normal+geometry.support_offset_m
 footprint=(abs(cr)<=node.gripper_geometry.finger_size_xyz_m[0]*.5)&(abs(ins)<=node.gripper_geometry.finger_size_xyz_m[2]*.5)&(abs(j)<=.025+1e-9)
 lo,hi=_linear_quantiles(j[footprint],(.05,.95));mid=(lo+hi)/2;side=node.gripper_geometry.jaw_clearance_each_side_m
 masks=[footprint&(abs(j-lo)<=side)&(j<mid),footprint&(abs(j-hi)<=side)&(j>mid)]
 side_rows=[]
 for name,mask in zip(('negative','positive'),masks):
  values=support[mask]; side_rows.append(dict(side=name,points=int(mask.sum()),support_height_5_95_m=_linear_quantiles(values,(.05,.95)) if len(values) else None,support_height_min_max_m=[values.min(),values.max()] if len(values) else None,representative_source_counts={str(i):int(np.count_nonzero(mask&(surface.view_indices==i))) for i in range(7)},supporting_source_counts={str(i):sum(bool(flag) and i in src for flag,src in zip(mask,surface.view_sources)) for i in range(7)}))
 bounds=[r['support_height_5_95_m'] for r in side_rows];common=[max(b[0] for b in bounds),min(b[1] for b in bounds)]
 e,b=evaluate_bilateral_surface_evidence_and_bounds(surface,geometry.center_base,jaw,-normal,node.gripper_geometry,support_normal_base=normal)
 rows.append(dict(angle_deg=angle,raw_side_intervals=side_rows,common_support_interval_m=common,common_support_overlap_signed_m=common[1]-common[0],post_common_evidence=asdict(e),post_common_insertion_bounds_m=b))
# Validate the reconstructed point set against audit measured XYZ without altering reconstruction.
online=np.array(audit['replay_geometry']['object_points_base_m']);reconstructed=z['geometry_points'];dist,ix=cKDTree(online).query(reconstructed)
points=surface.points_base;near,idx=cKDTree(online).query(points);matched=online[idx];combined=np.concatenate((np.load(ROOT/'fused_reconstruction_final'/('%d_geometry.npz'%STAMP))['object_points_base'],matched));_,keep=np.unique(combined,axis=0,return_index=True);union=combined[np.sort(keep)];sort=lambda a:a[np.lexsort(a.T[::-1])]
validation=dict(online_union_point_count=len(online),reconstructed_union_point_count=len(reconstructed),bidirectional_hausdorff_m=max(float(dist.max()),float(cKDTree(reconstructed).query(online)[0].max())),audit_xyz_nearest_fused_unique_count=len(np.unique(idx)),audit_xyz_matched_fused_union_set_exact=np.array_equal(sort(union),sort(online)),audit_xyz_matched_union_order_exact=np.array_equal(union,online),scope='Correspondences are diagnostics only; sweep above uses the unchanged reconstructed measured surface, not substituted audit XYZ.')
save('absolute_height_intervals.json',dict(rows=rows,reconstruction_validation=validation,original_trim_fraction=_MEASURED_SURFACE_TRIM_FRACTION,original_required_overlap_m=.002))
print(json.dumps(cv(dict(path=str(OUT/'absolute_height_intervals.json'),rows=[dict(angle=x['angle_deg'],sides=[r['support_height_5_95_m'] for r in x['raw_side_intervals']],common_mm=1000*x['common_support_overlap_signed_m']) for x in rows],validation=validation)),indent=2))
