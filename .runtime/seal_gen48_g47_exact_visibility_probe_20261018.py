#!/usr/bin/env python3
"""Metadata-only closure of saved original qualification/trace, no new computation."""
from pathlib import Path
import json,hashlib,math,numbers,time
W=Path(__file__).resolve().parents[1]
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller');N=E/'ros_log/full_latest_gen41_restart_20261018T011222Z';V=N/'verification'
O=V/'gen48_g47_exact_box_visibility_probe_r1';A=O/'attempt_r6'
G=E/'grasp_records/20261018T085552.871836Z_7d7d31b2570b433b8ee05398edc47d73'
STAMPS=[1792313803191381216,1792313803454153060,1792313803719039440]
evidence={}
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb')as f:
  for block in iter(lambda:f.read(4*1024*1024),b''):h.update(block)
 return h.hexdigest()
def load(p,expected=None):
 p=Path(p);h=sha(p)
 if expected:assert h==expected
 evidence[str(p)]=h;return json.loads(p.read_text())
def save(p,value):
 raw=(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode()
 with p.open('xb')as f:f.write(raw)
 assert p.read_bytes()==raw
 return dict(path=str(p),sha256=sha(p))
def differences(a,b,path=''):
 if isinstance(a,dict)and isinstance(b,dict):
  assert set(a)==set(b),(path,set(a)^set(b));return sum((differences(a[k],b[k],path+'/'+k)for k in a),[])
 if isinstance(a,list)and isinstance(b,list):
  assert len(a)==len(b);return sum((differences(x,y,path+'/'+str(i))for i,(x,y)in enumerate(zip(a,b))),[])
 if a==b:return []
 assert isinstance(a,(int,float))and isinstance(b,(int,float))and not isinstance(a,bool)and not isinstance(b,bool),(path,a,b)
 assert math.isfinite(a)and math.isfinite(b)
 return [dict(path=path,recomputed=a,recorded=b,absolute_difference=abs(a-b))]
def main():
 assert E.parent.parent.is_mount()and not(O/'result.json').exists()
 release=load(V/'gen47_release_manifest.json');assert release['source_count']==1052
 assert all(sha(W/p)==h for p,h in release['source_sha256'].items())
 audit=load(G/'keyframes_audits/19674cf4fc7b10aa290adff95422e8911eaafd18a8ab55c242dd5ca1f959cda2.json','19674cf4fc7b10aa290adff95422e8911eaafd18a8ab55c242dd5ca1f959cda2')
 original=audit['remote_diagnostics']['tabletop_geometry']['regular_box_inference']
 manifest=load(G/'manifest.json');assert manifest['state']=='complete'and manifest['recorder_closed']and manifest['result']=='failure'
 br=next(x for x in manifest['files']if x['path']=='keyframes.bag');assert sha(G/'keyframes.bag')==br['sha256'];evidence[str(G/'keyframes.bag')]=br['sha256']
 qualification=load(A/'qualification_before_exact_comparison.json');trace=load(A/'assignment_trace_before_exact_comparison.json');partial=load(A/'partial_failure.json')
 assert partial['error_type']=='AssertionError'and partial['boundary_attempts']==[]
 assert qualification['ok']is False and qualification['code']==original['code']=='BOX_TOP_NOT_RECTANGULAR'
 actual=qualification['audit'];assert actual['cuboid_qualification_reason']==original['cuboid_qualification_reason']=='common cuboid family has unresolved top/support silhouette visibility'
 assert actual['retained_pixel_sha256']==original['retained_pixel_sha256']
 diffs=differences(actual['cuboid_rgb_interval'],original['cuboid_rgb_interval'])
 allowed={'full_silhouette_rgb_distances_px','corner_rgb_distances_px','bounded_refit_parameters','original_common_height_interval_m','bounded_proposal_window'}
 assert {r['path'].split('/')[1]for r in diffs}==allowed
 maxdiff=max(r['absolute_difference']for r in diffs)
 # No qualification/gate values are rounded or normalized: the complete exact
 # difference list is retained, including the original equality assertion red.
 exactmeta=[]
 for ns in STAMPS:
  meta=load(A/'inputs'/str(ns)/'metadata.json');assert meta['source_stamp_ns']==ns and meta['exact_source']
  for key in ['color_bgr','depth_raw','mask_uint8']:
   row=meta['files'][key];assert sha(row['file'])==row['file_sha256'];evidence[row['file']]=row['file_sha256']
  exactmeta.append(meta)
 assert exactmeta[-1]['files']['color_bgr']['array_sha256']==original['retained_pixel_sha256']['color_sha256']
 assert exactmeta[-1]['files']['depth_raw']['array_sha256']==original['retained_pixel_sha256']['depth_sha256']
 assert exactmeta[-1]['files']['mask_uint8']['array_sha256']==original['retained_pixel_sha256']['mask_sha256']
 png=load(A/'request16_final_original_color_metadata.json');assert png['source_stamp_ns']==STAMPS[-1]and png['decoded_pixels_exactly_equal'];assert sha(png['png_path'])==png['png_sha256'];evidence[png['png_path']]=png['png_sha256']
 camera=load(A/'camera_metadata.json');assert camera['camera_info']==[]and len(camera['parameter_evidence'])==33 and camera['camera_info_crosscheck_present']is False
 failures=[row for row in trace['events']if row['event']=='exception'];assert len(failures)==1
 f=failures[0];s=f['locals'];parent=f['infer_locals'];assert f['exception_reason']==original['cuboid_qualification_reason']and s['j']==0
 assert all(k in parent for k in ['groups','seed_groups','result','nominal_A','nominal_lo','nominal_hi','top_height','hu','T','K','axis','yaw'])
 half=[x/2 for x in s['size']];j=s['j'];low=s['low'];high=s['high'];u=s['uncertainty']
 margins=dict(outside_positive_m=low-half[j]-u,outside_negative_m=-half[j]-u-high,inside_low_m=low+half[j]-u,inside_high_m=half[j]-u-high)
 assert not(low>half[j]+u)and not(high< -half[j]-u)and not(low> -half[j]+u and high<half[j]-u)
 visibility=dict(failed_axis=j,camera_xy_m=s['camera'],half_size_xy_m=half,axis_uncertainty_xy_m=s['axis'],yaw_uncertainty_rad=s['yaw'],yaw_uncertainty_deg=math.degrees(s['yaw']),common_yaw_interval_rad=parent['result']['common_yaw_interval_rad'],height_uncertainty_m=s['hu'],rotated_camera_interval_m=[low,high],rotated_axis_uncertainty_m=u,margins=margins,nominal_camera_minus_negative_face_m=s['camera'][j]+half[j],nominal_camera_minus_positive_face_m=s['camera'][j]-half[j],gate_semantics='One independently enclosed camera/face assignment is unresolved. This does not prove the exact coupled raster-strip family crosses the visibility boundary.')
 tracecopy=O/'assignment_stability_trace.json';tracecopy.write_bytes((A/'assignment_trace_before_exact_comparison.json').read_bytes());assert sha(tracecopy)==sha(A/'assignment_trace_before_exact_comparison.json');evidence[str(tracecopy)]=sha(tracecopy)
 comparison=save(O/'recorded_vs_recomputed_numeric_difference.json',dict(exact_full_cuboid_audit_equal=False,all_differing_fields=sorted(allowed),differences=diffs,maximum_absolute_numeric_difference=maxdiff,raster_distance_maximum_absolute_difference_px=max(x['absolute_difference']for x in diffs if 'rgb_distances_px'in x['path']),bounded_refit_maximum_absolute_difference=max(x['absolute_difference']for x in diffs if 'bounded_refit_parameters'in x['path']),all_other_cuboid_fields_exactly_equal=True,original_audit_equality_assertion_remains_red=True,original_failure_code_reason_identical=True,production_gate_values_changed=False,conclusion='Same original failure and exact source pixels reproduced. Only the explicitly listed floating-point tails differ; no bit-exact audit claim or normalized passing qualification is made.'))
 attempt_history=[]
 for p in sorted(O.glob('**/partial_failure.json')):
  q=load(p);attempt_history.append(dict(path=str(p),sha256=sha(p),error_type=q['error_type'],error=q['error'],boundary_attempts=q.get('boundary_attempts',[])))
 helper=W/'.runtime/probe_gen48_g47_exact_box_visibility_20261018.py';evidence[str(helper)]=sha(helper);evidence[str(Path(__file__))]=sha(Path(__file__))
 report=dict(kind='gen48_g47_exact_box_visibility_probe',probe_completed=True,reproduced_original_failure=True,original_qualification_ok=False,original_code=qualification['code'],original_cuboid_failure=f['exception_reason'],recorded_cuboid_audit_bit_exact=False,source_pixels_exact=True,retained_pixel_sha256=original['retained_pixel_sha256'],source_stamps_ns=STAMPS,camera_evidence=dict(path=str(A/'camera_metadata.json'),sha256=evidence[str(A/'camera_metadata.json')],intrinsics=camera['intrinsics'],consistent_recorded_parameter_samples=33,camera_info_recorded=False,parameters_are_image_synchronized=False),original_color_png=png,original_trace=dict(path=str(tracecopy),sha256=evidence[str(tracecopy)]),visibility=visibility,raw_qualification=dict(path=str(A/'qualification_before_exact_comparison.json'),sha256=evidence[str(A/'qualification_before_exact_comparison.json')]),numeric_comparison=comparison,geometry_provider=dict(source='exact closed audit replay_geometry and snapshot_transform.T_base_optical',registered_points_count=audit['replay_geometry']['object_points_count'],temporal_median_depth_recreated=False,execution_snapshot_recreated=False,qualification_only_original_sample=True),source_count=1052,source_unchanged=True,sealed_release_path=str(V/'gen47_release_manifest.json'),sealed_release_sha256=evidence[str(V/'gen47_release_manifest.json')],bag_manifest_verified=True,bag_sha256=br['sha256'],qualification_attempt_boundary_attempts=[],partial_attempts_preserved=attempt_history,platform_import_provider='native os.uname and struct.calcsize(P), no subprocess/network/device probes',production_or_model_or_calibration_changed=False,original_gate_patched=False,ROS_init_called=False,hardware_contacted=False,temperature_queried=False,motion_commands_sent=False,physical_grasp_verified=False,pending_question='Determine from original groups/common interval whether a coupled feasible raster family really changes top/support visibility, or independent enclosures alone cause rejection.',input_evidence_sha256=evidence)
 result=save(O/'result.json',report);seal=save(O/'seal.json',dict(result=result,trace=report['original_trace'],comparison=comparison,finalizer_sha256=sha(Path(__file__)),evidence_sha256=evidence))
 print(json.dumps(dict(result=result,seal=seal,visibility=visibility,numeric_maximum_absolute_difference=maxdiff),ensure_ascii=False))
if __name__=='__main__':main()
