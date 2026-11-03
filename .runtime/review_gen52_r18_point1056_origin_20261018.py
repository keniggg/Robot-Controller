"""Read only original G52 raw/fused point provenance and coordinate arithmetic."""
from pathlib import Path
import gzip,hashlib,json,math,os,time
import numpy as np
W=Path(__file__).resolve().parents[1]
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
V=E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
G=E/'grasp_records/20261018T143337.794533Z_bfea8721dba04526980035ef69ebe860'
O=V/'gen52_actual_handoff_monitor_r1/point1056_origin_r1'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def shaped(a):
 a=np.ascontiguousarray(a,dtype='<f8');h=hashlib.sha256();h.update(str(a.shape).encode('ascii'));h.update(b'\0'+a.dtype.str.encode('ascii')+b'\0');h.update(a.tobytes());return h.hexdigest()
def save(p,d):
 p.write_text(json.dumps(d,ensure_ascii=False,indent=2,allow_nan=False)+'\n');assert json.loads(p.read_text())==d
def main():
 assert Path('/media/zhuyupei/SanStand').is_mount()and not O.exists();O.mkdir()
 ap=G/'keyframes_audits/404d30752dbb490c83ce54c9d307676d52bb75355b3683a09825d294e293b402.json';assert sha(ap)==ap.stem
 a=json.loads(ap.read_text());f=a['multiview_surface']['fused_scene_geometry'];b=a['remote_diagnostics']['tabletop_geometry']['regular_box_inference'];conflict=b['measured_geometry_conflict'];surfs={};refs={}
 for k,r in f['surface_replay']['references'].items():
  path=Path(r['store_root'])/r['relative_path'];blob=path.read_bytes();assert sha(path)==r['gzip_sha256'];raw=gzip.decompress(blob);assert len(raw)==r['uncompressed_bytes']and hashlib.sha256(raw).hexdigest()==r['object_sha256'];surfs[k]=json.loads(raw);refs[k]=dict(path=str(path),gzip_sha256=r['gzip_sha256'],object_sha256=r['object_sha256'])
 points=np.asarray(a['replay_geometry']['object_points_base_m'],float);raw=points[:f['current_raw_point_count']];s=surfs['snapshot'];ref=surfs['reference'];before=surfs['before_registration_reference'];fused=np.asarray(s['points_base'],float);reference=np.asarray(ref['points_base'],float)
 assert points.shape==(2425,3)and raw.shape==(806,3)and fused.shape==(1652,3)
 assert shaped(points)==f['enclosed_points_sha256']and shaped(raw)==f['current_raw_points_sha256']and shaped(fused)==f['fused_points_sha256']
 target=points[1056];assert target.tolist()==conflict['point_base_m'];raw_matches=np.flatnonzero(np.all(raw==target,axis=1)).tolist();matches=np.flatnonzero(np.all(fused==target,axis=1)).tolist();assert not raw_matches and matches==[257]
 origins=[dict(fused_index=i,view_index=s['view_indices'][i],view_sources=s['view_sources'][i],source_stamps_ns=[s['view_stamps_ns'][j]for j in s['view_sources'][i]])for i in matches]
 assert origins[0]['view_sources']==[0]and origins[0]['source_stamps_ns']==[1792334020688617944]
 combined=np.concatenate((raw,fused),axis=0);_,indices=np.unique(combined,axis=0,return_index=True);union=combined[np.sort(indices)];assert np.array_equal(union,points)
 T=np.asarray(s['snapshot_to_reference'],float);assert np.array_equal(T,np.asarray(a['multiview_surface']['latest_registration']['snapshot_to_reference_transform']))
 inv=np.linalg.inv(T);converted=reference.dot(inv[:3,:3].T)+inv[:3,3];assert np.array_equal(converted,fused)
 assert s['view_sources']==ref['view_sources']and s['view_indices']==ref['view_indices']and s['view_stamps_ns']==ref['view_stamps_ns']
 original_reference=np.asarray(ref['reference']['points_base'],float);snapshot_reference=np.asarray(s['reference']['points_base'],float);assert np.array_equal(original_reference.dot(inv[:3,:3].T)+inv[:3,3],snapshot_reference)
 n=np.asarray(ref['reference']['support_normal_base']);assert np.array_equal(T[:3,:3].T.dot(n),np.asarray(s['reference']['support_normal_base']))
 assert float(ref['reference']['support_offset_m']+n.dot(T[:3,3]))==s['reference']['support_offset_m']
 rpoint=reference[matches[0]];bp=np.asarray(before['points_base'],float);distance=np.linalg.norm(bp-rpoint,axis=1);nearest=int(distance.argmin())
 cube=b['cuboid_rgb_interval'];A=np.asarray(cube['nominal_axes_base']);lo=np.asarray(cube['nominal_lower']);hi=np.asarray(cube['nominal_upper']);h=cube['nominal_height_m'];d=cube['support_offset_m'];center=A[:,:2]@((lo+hi)/2)+A[:,2]*(h/2-d);size=np.r_[hi-lo,h];axis=np.asarray(cube['empirical_axis_uncertainty_m']);hu=cube['empirical_height_uncertainty_m'];yaw=cube['empirical_yaw_uncertainty_rad'];half=size/2
 for j in range(2):
  turn=min(yaw,math.atan2(size[1-j],size[j]));half[j]=(size[j]*math.cos(turn)+size[1-j]*math.sin(turn))/2
 fullhalf=half+np.r_[axis,hu];local=(points-center)@A;excess=np.abs(local)-fullhalf;idx,j=np.unravel_index(np.argmax(excess),excess.shape);assert (idx,j)==(1056,0);assert abs(float(excess[idx,j])-conflict['excess_m'])<1e-15;assert np.allclose(local[1056],conflict['point_local_m'],rtol=0,atol=1e-15)
 # Same rigid transform applied to model and measurements leaves local coordinates unchanged.
 center_reference=T[:3,:3]@center+T[:3,3];axes_reference=T[:3,:3]@A;invariant=(rpoint-center_reference)@axes_reference;assert np.allclose(invariant,local[1056],rtol=0,atol=1e-15)
 summary=dict(kind='GEN52_request18_point1056_exact_origin_and_frame_readonly_audit',passed=True,created_ns=time.time_ns(),request_id=a['request_id'],generation=a['generation'],snapshot_stamp_ns=a['snapshot_stamp_ns'],point_index=1056,point_base_m=target.tolist(),point_local_m=local[1056].tolist(),raw_match_indices=raw_matches,fused_match_indices=matches,origins=origins,source_age_sec=(a['snapshot_stamp_ns']-origins[0]['source_stamps_ns'][0])/1e9,measured_count=len(points),current_raw_count=len(raw),fused_count=len(fused),exact_union_reproduced=True,exact_duplicate_count=len(combined)-len(points),snapshot_points_exact_inverse_registration_of_phase_reference=True,snapshot_reference_points_and_support_plane_exact_inverse_registration=True,view_provenance_preserved=True,nominal_model_local_frame=cube['policy'],model_center_current_snapshot_base=center.tolist(),model_axes_current_snapshot_base=A.tolist(),model_to_reference_then_local_coordinates_invariant=True,raw_vs_registered_coordinate_mix_detected=False,snapshot_to_reference=T.tolist(),point_phase_reference_m=rpoint.tolist(),preappend_nearest_point=dict(index=nearest,distance_m=float(distance[nearest]),view_index=before['view_indices'][nearest],view_sources=before['view_sources'][nearest]),full_family_half_extent_m=fullhalf.tolist(),conflict=conflict,completeRGB_max_residual_px=max(cube['full_silhouette_rgb_distances_px']),unchanged_observed_excess_gate_m=.001,geometry_conflict_reproduced_arithmetically=True,gzip_references=refs,original_audit=dict(path=str(ap),sha256=sha(ap)),limitations=['No omitted inverse transform is detected: reference and snapshot values obey the exact original inverse correction, with model/measurement invariant local coordinates.','This arithmetic does not prove historical ICP is physically correct, does not establish whether depth or RGB contour is biased, and does not justify removing the point or relaxing1mm.','Point provenance is prior view0 only; GeometryEstimate itself has no per-view field, recovered here from the sealed fused surface ledger.','No qualifier/refit/geometry estimation/test/ROS/hardware/motion was run.'],production_or_record_modified=False,temperature_read=False,helper_sha256=sha(__file__))
 save(O/'result.json',summary);(O/'helper_source.py').write_bytes(Path(__file__).read_bytes());print(json.dumps(dict(result=str(O/'result.json'),sha256=sha(O/'result.json'),origins=origins,exact_inverse=True,excess_m=conflict['excess_m']),ensure_ascii=False))
if __name__=='__main__':main()
