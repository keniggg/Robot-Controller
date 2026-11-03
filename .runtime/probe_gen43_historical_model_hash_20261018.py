"""Same retained GEN39 source under original/new cuboid qualification, no ROS."""
import ast
from collections.abc import Mapping
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import socket
import sys
import time
import traceback

W=Path(__file__).resolve().parents[1]
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
V=E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
OUT=V/'gen43_historical_model_hash_comparison_r1'
assert E.parent.parent.is_mount() and not OUT.exists()
assert json.loads((E/'grasp_records/archive_config.json').read_text())['upload_enabled'] is False
assert json.loads((E/'grasp_records/archive_config.json').read_text())['delete_enabled'] is False
assert json.loads((E/'maintenance/storage_control.json').read_text())['archive_policy']=='local_only'
OUT.mkdir();os.environ['ROS_LOG_DIR']=str(OUT/'ros')
paths=[W/'src/alicia_flexible_grasp_supervisor/src',W/'src/alicia_grasp_modes/src',
    W/'src/alicia_flexible_grasp_supervisor/tests',W/'src/alicia_grasp_modes/tests',
    W/'devel/lib/python3/dist-packages']
for path in reversed(paths):sys.path.insert(0,str(path))

attempts=[]
class ExternalBoundary(BaseException):pass
def forbidden(*args,**kwargs):
    attempts.append(dict(args=repr(args),kwargs=repr(kwargs)))
    raise ExternalBoundary('offline historical model comparison boundary')
for name in ('connect','connect_ex','sendto','bind','listen'):
    setattr(socket.socket,name,forbidden)
socket.create_connection=forbidden
import rospy
for name in ('init_node','Publisher','Subscriber','Service','ServiceProxy','wait_for_service',
             'set_param','delete_param','get_param','get_param_cached'):
    setattr(rospy,name,forbidden)

sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(path,value):
    Path(path).write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
def canonical(value):
    return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False,ensure_ascii=True)
def digest(value):return hashlib.sha256(canonical(value).encode()).hexdigest()
def plain(value):
    if isinstance(value,Mapping):return {str(k):plain(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [plain(v) for v in value]
    if hasattr(value,'tolist'):return value.tolist()
    return value

r=dict(passed=False,kind='GEN43_exact_GEN39_source_before_after_model_hash',
    ROS_initialized=False,network_access=False,hardware_access=False,temperature_queried=False,
    production_or_tests_changed=False,pytest_suite_run=False)
source_files=[Path(__file__).resolve(),W/'src/alicia_grasp_modes/tests/test_gen40_clearance_enclosure_chain.py',
 W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/cuboid_rgb_interval.py',
 W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/regular_box_inference.py',
 W/'.runtime/gen42_source_before_gen43/src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/cuboid_rgb_interval.py']
source_before={str(p):sha(p) for p in source_files}
try:
    import numpy as np
    import test_gen40_clearance_enclosure_chain as fixture
    from alicia_flexible_grasp.vision import regular_box_inference as box
    from alicia_flexible_grasp.vision import cuboid_rgb_interval as current
    old_path=source_files[-1]
    assert sha(old_path)=='4e127a38faac34b2b981d2f757f22399934b38655e44376f268580a3243dffdf'
    assert sha(current.__file__)=='50e00a5b751ba0470f98c4215922bb6f05471784a364181c297ad5f617751d7f'
    spec=importlib.util.spec_from_file_location('alicia_flexible_grasp.vision._gen42_hash_comparison_baseline',old_path)
    legacy=importlib.util.module_from_spec(spec);spec.loader.exec_module(legacy)
    inputs=fixture._actual_final_inputs()
    geometry=inputs.geometry
    points_before=np.asarray(geometry.object_points_base).tobytes()
    geom_hash=box.regular_box_geometry_sha256(geometry)
    assert len(geometry.object_points_base)==2610
    args=(inputs.snapshot,inputs.K,inputs.K.depth_scale,inputs.T,
          geometry,inputs.identity,inputs.snapshot.stamp_ns)
    names=('infer_cuboid_rgb_interval','validate_cuboid_rgb_interval','validate_cuboid_rgb_interval_source')
    bindings={name:getattr(box,name) for name in names}
    try:
        for name in names:setattr(box,name,getattr(legacy,name))
        old=box.qualify_regular_box_snapshot(*args)
        assert old.ok,plain(old.audit)
        old_model=old.model.to_evidence_dict()
    finally:
        for name,value in bindings.items():setattr(box,name,value)
    new=box.qualify_regular_box_snapshot(*args)
    assert new.ok,plain(new.audit)
    new_model=new.model.to_evidence_dict()
    assert old_model['model_sha256']=='a52f80400af701553a7866f4791b6ff2913b70382e57ebb352bc37bea823e322'
    assert new_model['model_sha256']=='f0e3916e28ce6a0372c953f41ce0b82013c40dc49210f17af8c2d33f5219b336'
    assert np.asarray(geometry.object_points_base).tobytes()==points_before
    assert box.regular_box_geometry_sha256(geometry)==geom_hash
    differences=[]
    def diff(before,after,path=''):
        if isinstance(before,dict) and isinstance(after,dict):
            for key in sorted(set(before)|set(after)):
                child=path+'/'+key
                if key not in before:differences.append(dict(path=child,kind='added',new=after[key]))
                elif key not in after:differences.append(dict(path=child,kind='removed',old=before[key]))
                else:diff(before[key],after[key],child)
        elif before!=after:differences.append(dict(path=path,kind='changed',old=before,new=after))
    diff(old_model,new_model)
    extra=('coverage_policy','silhouette_mask_intersection_pixels',
        'silhouette_mask_eroded_polygon_pixels','top_mask_intersection_pixels',
        'top_mask_eroded_polygon_pixels','top_mask_coverage')
    expected={'/qualification/cuboid_rgb_interval/'+key for key in extra}
    assert {x['path'] for x in differences if x['kind']=='added'}==expected
    assert [x['path'] for x in differences if x['kind']=='changed']==['/model_sha256']
    assert not any(x['kind']=='removed' for x in differences)
    projection=json.loads(canonical(new_model));projection.pop('model_sha256')
    for key in extra:projection['qualification']['cuboid_rgb_interval'].pop(key)
    old_body={key:value for key,value in old_model.items() if key!='model_sha256'}
    new_body={key:value for key,value in new_model.items() if key!='model_sha256'}
    assert canonical(projection)==canonical(old_body)
    assert digest(projection)==digest(old_body)==old_model['model_sha256']
    assert digest(new_body)==new_model['model_sha256']
    interval=new_model['qualification']['cuboid_rgb_interval']
    current.validate_cuboid_rgb_interval(interval,new_model['size_xyz_m'],
        new_model['empirical_edge_uncertainty_m'],new_model['empirical_yaw_uncertainty_rad'],
        new_model['empirical_height_uncertainty_m'])
    raw=inputs.snapshot.original_sample
    current.validate_cuboid_rgb_interval_source(interval,raw.color_bgr,raw.object_mask,
        raw.depth_raw,inputs.K.depth_scale,inputs.T,inputs.K)
    save(OUT/'legacy_model.json',old_model);save(OUT/'current_model.json',new_model)
    save(OUT/'legacy_compatibility_projection_without_model_sha.json',projection)
    audit_path=fixture._G/'keyframes_audits'/(fixture._AUDIT_SHA+'.json')
    r.update(passed=True,source_stamp_ns=inputs.snapshot.stamp_ns,
      historical_audit=str(audit_path),historical_audit_sha256=sha(audit_path),
      legacy_module_path=str(old_path),legacy_module_sha256=sha(old_path),
      qualifier_module_path=str(Path(box.__file__).resolve()),qualifier_module_sha256=sha(box.__file__),
      legacy_model_sha256=old_model['model_sha256'],current_model_sha256=new_model['model_sha256'],
      exact_added_fields=list(extra),differences=differences,
      no_old_fields_or_values_changed=True,legacy_projection_canonical_equal=True,
      legacy_projection_digest=digest(projection),new_source_and_numeric_validator_passed=True,
      original_points_count=2610,original_points_bytes_unchanged=True,geometry_sha256=geom_hash,
      geometry_and_all_old_uncertainty_source_qualification_fields_bit_exact=True,
      numeric_model={key:new_model[key] for key in ('center_base','axes_base','size_xyz_m',
        'empirical_edge_uncertainty_m','empirical_height_uncertainty_m','empirical_yaw_uncertainty_rad')},
      comparison_scope='same exact original test input provider and original qualifier with only three legacy cuboid function bindings swapped in memory; no fixed-hash test assert bypassed or claimed passed',
      downstream_complete_closed_source_test_passed=False,
      limitations='the original test stopped before producer candidates and typed contact/aperture/wire checks; their completion remains to be tested after an explicit fixture compatibility amendment')
except BaseException as exc:r.update(error=repr(exc),traceback=traceback.format_exc())
finally:
    r.update(recorded_ns=time.time_ns(),source_sha256=source_before,
        sources_unchanged=all(sha(p)==h for p,h in source_before.items()),external_boundary_attempts=attempts)
    r['passed']=r['passed'] and r['sources_unchanged'] and not attempts
    r['output_sha256']={str(p):sha(p) for p in OUT.glob('*.json')}
    save(OUT/'result.json',r);save(OUT/'seal.json',dict(result_sha256=sha(OUT/'result.json'),output_sha256=r['output_sha256'],source_sha256=source_before))
    print(json.dumps(dict(passed=r['passed'],output=str(OUT),error=r.get('error')),ensure_ascii=False),flush=True)
