#!/usr/bin/env python3
"""Seal already-produced exact R22 evidence; never rerun qualification."""
import hashlib,importlib.util,json,os,sys,time
from pathlib import Path
sys.dont_write_bytecode=True
W=Path(__file__).resolve().parents[1]
V=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
OUT=V/'gen49_g48_exact_box_unknown_probe_r1';A=OUT/'attempt_r1'
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb')as f:
  for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
 return h.hexdigest()
def read(p):return json.loads(Path(p).read_text())
def put(p,a):
 with Path(p).open('x')as f:json.dump(a,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')
 return dict(path=str(p),sha256=sha(p))
assert Path('/media/zhuyupei/SanStand').is_mount() and not(OUT/'result.json').exists()
probe=W/'.runtime/gen49_probe_g48_exact_box_unknown_20261018.py';assert sha(probe)=='886b905003cdc76fa077ef858a72bd39992318b81dfdeb704af1cfd8033b5a49'
spec=importlib.util.spec_from_file_location('gen49_closed_probe_boundary',probe);guard=importlib.util.module_from_spec(spec);spec.loader.exec_module(guard);guard.OUT=OUT;sys.addaudithook(guard.audit_event)
q=read(A/'qualification_replay.json');raw_comparison=read(A/'audit_comparison.json');partial=read(A/'partial_failure.json');detail=read(A/'unknown_pixel_ray_details.json');trace=read(A/'unknown_ray_trace.json');expected=q['expected_audit'];audit=q['audit']
assert q['ok'] is False and q['code']==expected['code'] and expected['ok'] is False
assert audit['cuboid_qualification_reason']==expected['cuboid_qualification_reason']=='unknown mask band lies outside complete cuboid uncertainty family'
assert partial['error_type']=='AssertionError' and not partial['boundary_attempts']
keyrows=[x for x in raw_comparison['differences']if x['kind']=='keys'];numeric=[x for x in raw_comparison['differences']if x['kind']=='numeric']
assert len(keyrows)==1 and keyrows[0]['path']=='' and set(keyrows[0]['expected_keys'])-set(keyrows[0]['actual_keys'])=={'ok','code'} and set(keyrows[0]['actual_keys'])<=set(keyrows[0]['expected_keys'])
assert len(keyrows)+len(numeric)==len(raw_comparison['differences']) and all(x['absolute_difference']<=1e-9 for x in numeric)
assert detail['unknown_count']==3994 and detail['rejected_count']==11 and detail['center_rejected_count']==5 and detail['center_hits_but_some_corner_misses_count']==6 and detail['all_five_miss_count']==1
assert len(trace['events'])==1 and sum(not x for x in trace['events'][0]['returned'])==11
baseline=W/'.runtime/gen49_baseline_source';mp=baseline/'manifest.json';assert sha(mp)=='21f6c6a1b0b8072d78332a081af1a425c570c9e5dd1983b2f982960e3911e935'
releasep=V/'gen48_release_manifest.json';release=read(releasep);sources=release['source_sha256'];assert len(sources)==1053 and all(sha(baseline/p)==h for p,h in sources.items())
g=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261018T095014.238124Z_ea3e1758e151468c9c945b7e94a07b1d');manifest=read(g/'manifest.json');assert manifest['state']=='complete'and manifest['result']=='failure'and manifest['recorder_closed']
bagref=next(x for x in manifest['files']if x['path']=='keyframes.bag');assert bagref['sha256']=='b679486eddc9d41323fc84a6ad0f357433a51e55e5e20c989d98118ecf2d236f' and (g/'keyframes.bag').stat().st_size==bagref['size_bytes'] and sha(g/'keyframes.bag')==bagref['sha256']
arrays=[]
for ns in (1792317082624930858,1792317083490865230,1792317083889718770):
 meta=read(A/'inputs'/str(ns)/'metadata.json');assert meta['source_stamp_ns']==ns and meta['exact_source']
 for name in ['color_bgr','depth_raw','mask_uint8']:
  row=meta['files'][name];assert sha(row['file'])==row['file_sha256']
 arrays.append(dict(source_stamp_ns=ns,metadata=str(A/'inputs'/str(ns)/'metadata.json'),metadata_sha256=sha(A/'inputs'/str(ns)/'metadata.json'),files={key:meta['files'][key] for key in ['color_bgr','depth_raw','mask_uint8']},original_color_png=str(A/'inputs'/str(ns)/'original_color.png'),original_color_png_sha256=sha(A/'inputs'/str(ns)/'original_color.png')))
assert {k:arrays[-1]['files'][name]['array_sha256'] for k,name in [('color_sha256','color_bgr'),('depth_sha256','depth_raw'),('mask_sha256','mask_uint8')]}==expected['retained_pixel_sha256']
assert all(x['original_depth_raw']==0 and x['original_mask_raw']==255 for x in detail['rejected_pixels'])
for x in detail['rejected_pixels']:
 for a in x['original_neighborhoods']:assert sha(a['color_crop_png'])==a['color_crop_sha256']
camera=read(A/'camera_metadata.json');assert len(camera['parameter_evidence'])==143 and camera['all_recorded_numeric_intrinsics_and_scales_consistent'] and not camera['camera_info_crosscheck_present']
paths=[probe,Path(__file__),mp,releasep,g/'manifest.json',g/'keyframes.bag',V/'gen48_actual_handoff_monitor_r1/request22_atomic_audit_e7886527.json']+list(A.glob('*.json'))+[Path(x['metadata']) for x in arrays]
evidence={str(p):sha(p)for p in paths}
assert not guard.BOUNDARY
r=dict(kind='GEN49_exact_G48_R22_original_unknown_pixel_replay',passed=True,passed_meaning='Original same-source pixels and unchanged GEN48 qualifier rejection reproduced; no qualified model, path or motion approval',request_id=22,remote_generation=7,selection_generation=8,target_epoch=4,record=str(g),record_state='complete',record_result='failure',source_root=str(baseline),source_count=1053,source_sha256=sources,source_unchanged=True,bag_before_and_after_SHA_verified=True,bag_sha256=bagref['sha256'],source_stamp_ns=[x['source_stamp_ns']for x in arrays],arrays=arrays,retained_pixel_sha256=expected['retained_pixel_sha256'],qualification_ok=False,qualification_code=q['code'],cuboid_rejection=audit['cuboid_qualification_reason'],unknown_pixel_count=3994,rejected_pixel_count=11,center_rejected_count=5,center_hits_corner_rejected_count=6,any_of_five_hits_count_among_rejected=10,all_five_miss_count=1,rejected_pixels_yx=[x['pixel_yx']for x in detail['rejected_pixels']],pixel_details=str(A/'unknown_pixel_ray_details.json'),pixel_details_sha256=sha(A/'unknown_pixel_ray_details.json'),unknown_cell_gate_policy_unchanged=True,original_color_png=str(A/'request22_final_original_color.png'),original_color_png_sha256=sha(A/'request22_final_original_color.png'),audit_comparison=dict(original_raw_comparison=str(A/'audit_comparison.json'),original_raw_comparison_retained=True,initial_assertion_preserved=str(A/'partial_failure.json'),wrapper_keys_missing_from_raw_qualification_audit=['ok','code'],wrapper_keys_independently_match_original_report=True,numeric_difference_count=len(numeric),maximum_absolute_numeric_difference=max(x['absolute_difference']for x in numeric),original_audit_bit_exact=False,source_hash_reason_counts_and_all_unknown_coordinates_exact=True),camera_runtime_metadata_count=143,camera_info_count=0,camera_parameters_are_not_exposure_synchronized=True,qualification_wall_sec=q['wall_sec'],execution_snapshot_recreated=False,temporal_median_depth_recreated=False,original_gate_patched=False,production_test_or_calibration_modified=False,ROS_initialized=False,hardware_contacted=False,temperature_queried=False,physical_grasp_verified=False,boundary_attempts=guard.BOUNDARY,evidence_sha256=evidence,limitations=['Binary recorded mask cannot distinguish original GrabCut certain/probable labels. Zero raw depth is established, physical sensor/segmentation cause is not.','Other-source neighborhoods refer to the same image coordinates, not registered cross-frame pixel identities.','Signed image distances refer to the exact enlarged OBB projected by the original unknown-ray gate.','This finalizer normalized report-only ok/code wrappers after their typed values were independently checked; it did not rerun qualification or modify original attempt artifacts.'],created_ns=time.time_ns())
ref=put(OUT/'result.json',r);seal=put(OUT/'seal.json',dict(result=ref,finalizer_sha256=sha(__file__),evidence_sha256=evidence));print(json.dumps(dict(result=ref,seal=seal),ensure_ascii=False))
