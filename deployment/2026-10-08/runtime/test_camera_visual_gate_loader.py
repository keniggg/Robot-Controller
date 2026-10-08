"""Offline camera-loader and bounded observer failure contracts; no ROS startup."""
import importlib.util
import json
from pathlib import Path
import subprocess
import pytest


def module(name, filename):
    path=Path(__file__).resolve().parent/'gen4_root_cause'/filename
    spec=importlib.util.spec_from_file_location(name,path)
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result)
    return result


loader=module('camera_loader_under_test','reload_camera_visual_gate.py')
observer=module('camera_observer_under_test','camera_existing_read_observer.py')


def test_import_and_unapproved_main_never_start_processes(monkeypatch):
    monkeypatch.setattr(subprocess,'run',lambda *a,**k:pytest.fail('unexpected process'))
    with pytest.raises(SystemExit):loader.main([])


def test_service_preserves_camera_remaps_and_python_environment(tmp_path):
    original=dict(cwd='/work',argv=['/usr/bin/python3',str(loader.CAMERA_ENTRIES[0]),
        '__name:=supervisor_camera','color/image:=/image','__log:=/old.log'])
    args=loader.service_argv('camera-test',original,tmp_path,
        {'PYTHONNOUSERSITE':'1','QT_QPA_PLATFORM':'offscreen','TOKEN':'secret'},45)
    assert '--setenv=PYTHONNOUSERSITE=1' in args
    assert '--setenv=QT_QPA_PLATFORM=offscreen' in args
    assert '--property=Restart=no' in args
    assert 'color/image:=/image' in args
    assert '__name:=supervisor_camera' in args
    assert not any('TOKEN' in arg or 'secret' in arg or '/old.log' in arg for arg in args)
    assert '__log:='+str(tmp_path/'camera.ros.log') in args


def wrapped_argv():
    return ['/usr/bin/python3',str(loader.OBSERVER),
            '--camera-script',str(loader.CAMERA_ENTRIES[0]),
            '--diagnostic-path','/media/zhuyupei/SanStand/old/read_diagnostics.jsonl',
            '--diagnostic-seconds','45.0','--',
            '/perception/object:=/perception/carton/object','__name:=supervisor_camera','__log:=/old.log']


def test_service_unwraps_existing_observer_once_and_preserves_ros_tail(tmp_path):
    original=wrapped_argv()
    args=loader.camera_argv(original,tmp_path,20.)
    assert args.count(str(loader.OBSERVER))==1
    assert args.count('--camera-script')==1
    assert args[args.index('--camera-script')+1]==str(loader.CAMERA_ENTRIES[0])
    assert args[args.index('--diagnostic-seconds')+1]=='20.0'
    assert '/perception/object:=/perception/carton/object' in args
    assert '__log:='+str(tmp_path/'camera.ros.log') in args
    assert '/old.log' not in args and '/media/zhuyupei/SanStand/old/read_diagnostics.jsonl' not in args
    assert original==wrapped_argv()


@pytest.mark.parametrize('case',[
    'unknown_option','repeated_option','missing_option','duplicate_delimiter','no_delimiter',
    'nested_wrapper','foreign_entry','empty_value','nan_duration','zero_duration',
    'large_duration','nonnumeric_duration','relative_diagnostic','extra_before_delimiter',
    'non_ros_tail','duplicate_name','duplicate_remap','empty_ros_value'])
def test_malformed_existing_wrapper_is_rejected(case,tmp_path):
    args=wrapped_argv()
    if case=='unknown_option':args[4]='--unapproved'
    elif case=='repeated_option':args[4]='--camera-script'
    elif case=='missing_option':del args[4:6]
    elif case=='duplicate_delimiter':args.append('--')
    elif case=='no_delimiter':args.remove('--')
    elif case=='nested_wrapper':args[3]=str(loader.OBSERVER)
    elif case=='foreign_entry':args[3]='/different/work/camera_node.py'
    elif case=='empty_value':args[5]=''
    elif case=='nan_duration':args[7]='nan'
    elif case=='zero_duration':args[7]='0'
    elif case=='large_duration':args[7]='121'
    elif case=='nonnumeric_duration':args[7]='bad'
    elif case=='relative_diagnostic':args[5]='relative.jsonl'
    elif case=='extra_before_delimiter':args[8:8]=['--new-option','true']
    elif case=='non_ros_tail':args.append('--camera-script')
    elif case=='duplicate_name':args.append('__name:=other')
    elif case=='duplicate_remap':args.append('/perception/object:=/other')
    elif case=='empty_ros_value':args.append('topic:=')
    with pytest.raises(RuntimeError):loader.camera_argv(args,tmp_path,45.)


@pytest.mark.parametrize('entry',loader.CAMERA_ENTRIES)
def test_only_known_project_entry_points_are_supported(entry,tmp_path):
    args=loader.camera_argv(['/usr/bin/python3',str(entry),'__name:=supervisor_camera'],tmp_path,45.)
    assert args[args.index('--camera-script')+1]==str(entry)


def test_runtime_publication_metadata_is_excluded_only_from_configuration_comparison():
    old=dict(fps=30,depth_scale=.0001,runtime_profile={'stationary_temporal':{'active':False}})
    new=dict(old,runtime_profile={'publication':{'header_stamp_semantics':'host_read_completed'},
                                 'stationary_temporal':{'active':True}})
    assert loader.stable_camera_configuration(old)==loader.stable_camera_configuration(new)
    new['depth_scale']=.001
    assert loader.stable_camera_configuration(old)!=loader.stable_camera_configuration(new)


@pytest.mark.parametrize('args',[
    ['/usr/bin/python3','wrong.py','__name:=supervisor_camera'],
    ['/usr/bin/python3','camera_node.py','__name:=different_camera'],[]])
def test_unexpected_entry_point_rejected(args,tmp_path):
    with pytest.raises(RuntimeError):loader.camera_argv(args,tmp_path,45)


def fixture():
    summary=dict(original_read_restored=True,errors=[],queued_unwritten=0,
        counts=dict(observed=1200,enqueued=1200,written=1200))
    reads=dict(rows=1200,steady_read_fps=30.,visual_qualified_frames=100,temporal_active_frames=100,
               publication_worker_health={'passed':True})
    pubs={k:dict(fps=20.) for k in ('rgb','depth')}
    return summary,reads,pubs,dict(visual_stationarity_required=True),set(range(800)),set(range(800))


def test_static_validation_requires_observer_accounting_and_paired_publication():
    args=fixture();assert loader.static_validation_passes(*args)
    args[-1].remove(1);assert loader.static_validation_passes(*args)
    args[-1].difference_update((2,3));assert not loader.static_validation_passes(*args)


@pytest.mark.parametrize('section,key,value',[
    (0,'original_read_restored',False),(0,'queued_unwritten',1),(0,'errors',['failure']),
    (1,'steady_read_fps',None),(1,'steady_read_fps',22.),
    (1,'visual_qualified_frames',0),(1,'temporal_active_frames',0),
    (1,'publication_worker_health',{'passed':False}),
    (3,'visual_stationarity_required',False)])
def test_static_validation_never_accepts_incomplete_evidence(section,key,value):
    args=fixture();args[section][key]=value
    assert not loader.static_validation_passes(*args)


@pytest.mark.parametrize('key', ['queue_dropped','copy_errors','serialization_errors','write_errors','written'])
def test_static_validation_detects_lost_diagnostic_rows(key):
    args=fixture();args[0]['counts'][key]=1
    assert not loader.static_validation_passes(*args)


def test_loaded_module_validation_rejects_stale_runtime_import(tmp_path):
    source={str(p):'x'*64 for p in loader.SOURCES}
    rows={name:dict(path=str(p.resolve()),sha256=source[str(p)]) for name,p in (
        ('alicia_flexible_grasp.vision.realsense_manager',loader.SOURCES[1]),
        ('alicia_flexible_grasp.vision.visual_stationary',loader.SOURCES[2]),
        ('alicia_flexible_grasp.vision.latest_rgbd_publication',loader.PUBLICATION_SOURCE))}
    path=tmp_path/'modules.json';path.write_text(json.dumps(rows))
    assert loader.validate_loaded_modules(path,source)==rows
    rows['alicia_flexible_grasp.vision.visual_stationary']['sha256']='old'
    path.write_text(json.dumps(rows))
    with pytest.raises(RuntimeError,match='verified source'):loader.validate_loaded_modules(path,source)


def test_missing_publication_module_cannot_pass_import_verification(tmp_path):
    source={str(p):'x'*64 for p in loader.SOURCES}
    rows={name:dict(path=str(p.resolve()),sha256=source[str(p)]) for name,p in (
        ('alicia_flexible_grasp.vision.realsense_manager',loader.SOURCES[1]),
        ('alicia_flexible_grasp.vision.visual_stationary',loader.SOURCES[2]))}
    path=tmp_path/'modules.json';path.write_text(json.dumps(rows))
    with pytest.raises(RuntimeError,match='latest_rgbd_publication'):
        loader.validate_loaded_modules(path,source)


def test_full_read_summary_reports_reused_and_skipped_counters_without_claiming_exposure_rate(tmp_path):
    rows=[]
    for i,(frame,stamp,result,fps) in enumerate(((100,0.,'ok',14996),(100,0.,'rejected_pair',14996),
                                                (102,66.67,'ok',29993),(103,100.,'ok',29995))):
        raw=dict(present=True,frame_number=frame,device_timestamp_ms=stamp,
                 timestamp_domain='global_time',metadata_raw={'frame_timestamp':int(stamp*1000),
                     'actual_fps':fps,'backend_timestamp':int(stamp),'time_of_arrival':int(stamp)+8})
        rows.append(dict(read_started_monotonic=float(i),existing_read_diagnostics={
            'result':result,'raw_frames':{'color':raw,'depth':raw}},
            publication_worker={'worker_alive':True,'pending':1,'failure':None}))
    path=tmp_path/'read.jsonl';path.write_text('\n'.join(json.dumps(row) for row in rows))
    summary=loader.summarize_reads(path);stream=summary['raw_streams']['color']
    assert stream['raw_frame_samples']==4
    assert stream['frame_number_increment_counts']=={'0':1,'2':1,'1':1}
    assert stream['repeated_frame_number_transitions']==1
    assert stream['unobserved_frame_numbers_between_reads']==1
    assert stream['metadata_actual_fps_raw_counts']['14996']==2
    assert stream['device_timestamp_interval_ms']['max']==66.67
    assert 'do not prove exposure' in ' '.join(stream['limitations'])
    assert summary['publication_worker_samples']==4
    assert summary['publication_worker_last']==rows[-1]['publication_worker']
    assert summary['publication_worker_health']['passed'] is False


def healthy_worker_summary():
    return dict(failure=None,worker_alive=True,pending=1,max_pending_pairs=1,closing=False,
                publish_failures=0,rejected_after_failure=0,dropped_on_failure=0,
                published=20,submitted=30,peak_pair_bytes=1000,max_pair_bytes=2000,
                replaced_pending=5,dropped_expired=3)


def test_worker_health_retains_expiry_and_replacement_as_bounded_counters():
    status=healthy_worker_summary()
    assert loader.publication_worker_health([status])['passed']
    assert status['replaced_pending']==5 and status['dropped_expired']==3


@pytest.mark.parametrize('key,value',[
    ('failure','publish error'),('worker_alive',False),('closing',True),
    ('pending',2),('max_pending_pairs',2),('publish_failures',1),
    ('rejected_after_failure',1),('dropped_on_failure',1),
    ('published',0),('published',31),('peak_pair_bytes',2001)])
def test_publication_worker_invalid_or_failed_state_cannot_validate(key,value):
    status=healthy_worker_summary();status[key]=value
    assert not loader.publication_worker_health([status])['passed']


def test_later_worker_status_cannot_erase_an_earlier_failure():
    failed=healthy_worker_summary();failed['failure']='earlier publish error'
    result=loader.publication_worker_health([failed,healthy_worker_summary()])
    assert not result['passed'] and 'sample0' in result['errors'][0]


def test_missing_worker_fields_never_pass_and_missing_diagnostics_never_pass():
    status=healthy_worker_summary();del status['publish_failures']
    assert not loader.publication_worker_health([status])['passed']
    assert not loader.publication_worker_health([])['passed']


def test_read_latency_summary_uses_actual_sdk_stage_and_host_wait_fields(tmp_path):
    rows=[]
    for i,(duration,wait,stage) in enumerate(((.03,.02,.021),(.2,.19,.191),(2.5,2.45,2.47))):
        worker=healthy_worker_summary();worker['maximum_publish_seconds']=(.01,.02,.3)[i]
        rows.append(dict(read_started_monotonic=float(i),read_completed_monotonic=float(i)+duration,
                         existing_read_diagnostics={'result':'complete','host_wait':{'elapsed_sec':wait},
                                                    'stages_sec':{'wait_for_frames':stage}},
                         publication_worker=worker))
    path=tmp_path/'latency.jsonl';path.write_text('\n'.join(json.dumps(row) for row in rows))
    result=loader.summarize_reads(path)
    assert result['read_wall_duration_sec']['min']==pytest.approx(.03)
    assert result['read_wall_duration_sec']['p95']==pytest.approx(2.27)
    assert result['read_wall_duration_sec']['max']==pytest.approx(2.5)
    # Floating-point subtraction at a later monotonic time may round exactly
    # .2 slightly down; the .2 threshold is informational and never admission.
    assert result['read_wall_duration_sec']['greater_than_0_2_sec']==1
    assert result['sdk_host_wait_elapsed_sec']['max']==2.45
    assert result['sdk_wait_for_frames_stage_sec']['max']==2.47
    assert result['sdk_host_wait_elapsed_sec']['greater_than_0_2_sec']==1
    assert result['sdk_wait_for_frames_stage_sec']['greater_than_0_2_sec']==1
    assert result['publication_worker_maximum_publish_seconds']==.3
    assert result['upstream_fault_resolution_verified'] is False
    assert 'cannot establish' in result['validation_scope']
    assert 'may block longer' in result['publication_latency_limitation']


def test_duration_diagnostic_uses_strict_threshold_and_omits_nonfinite_values():
    result=loader.duration_summary([None,float('nan'),float('inf'),.2,.200001])
    assert result['count']==2 and result['greater_than_0_2_sec']==1


def test_publication_summary_preserves_header_gaps_and_same_ros_clock_receipt_age():
    rows=[dict(received_monotonic=stamp,source_stamp_ns=int(stamp*10**9),
               received_ros_time_ns=int((stamp+age)*10**9))
          for stamp,age in ((100.,.06),(100.05,.005),(102.55,2.5))]
    result=loader.summarize_publication(rows)
    assert result['header_interval_sec']['max']==2.5
    assert result['header_interval_sec']['min']==.05
    assert result['header_interval_sec']['greater_than_0_2_sec']==1
    assert result['receipt_source_age_sec']['min']==.005
    assert result['receipt_source_age_sec']['max']==2.5
    assert result['receipt_source_age_sec']['greater_than_0_2_sec']==1
    assert 'not exposure age' in result['receipt_source_age_clock']


def test_publication_summary_reports_duplicate_and_regressing_headers():
    rows=[dict(received_monotonic=float(i),source_stamp_ns=stamp)
          for i,stamp in enumerate((3*10**9,3*10**9,2*10**9))]
    result=loader.summarize_publication(rows)
    assert result['header_duplicates']==1 and result['header_regressions']==1
    assert result['header_interval_sec']['min']==-1.
    assert result['receipt_source_age_sec']['count']==0


@pytest.mark.parametrize('late_error',['SDK target changed','driver command epoch changed'])
def test_final_guard_saves_revoked_success_before_raising_late_error(tmp_path,late_error):
    state={'static_validation_passed':True,'camera_loaded':True}
    def guard():raise RuntimeError(late_error)
    path=tmp_path/'result.json'
    with pytest.raises(RuntimeError,match=late_error):
        loader.save_result_with_final_guard(path,state,guard)
    stored=json.loads(path.read_text())
    assert stored['camera_loaded'] is True
    assert stored['static_validation_passed'] is False
    assert stored['final_guard_passed'] is False
    assert stored['error']==late_error


def test_final_guard_is_checked_immediately_before_success_result_is_written(tmp_path):
    calls=[];path=tmp_path/'result.json'
    def guard():
        assert not path.exists()
        calls.append('guard')
    state={'static_validation_passed':True}
    loader.save_result_with_final_guard(path,state,guard)
    assert calls==['guard']
    assert json.loads(path.read_text())=={'static_validation_passed':True,'final_guard_passed':True}


def test_final_result_preserves_an_already_failed_verification(tmp_path):
    state={'static_validation_passed':False,'error':'earlier validation error'}
    path=tmp_path/'result.json'
    loader.save_result_with_final_guard(path,state,lambda:pytest.fail('cannot turn prior failure into success'))
    assert json.loads(path.read_text())==state


def test_static_average_fps_pass_is_not_an_upstream_stall_resolution_claim():
    args=fixture()
    args[1].update(read_wall_duration_sec={'max':2.5,'greater_than_0_2_sec':1},
                   upstream_fault_resolution_verified=False)
    assert loader.static_validation_passes(*args)
    assert args[1]['upstream_fault_resolution_verified'] is False


def test_observer_restores_read_and_preserves_original_return_and_exception(tmp_path):
    now=[0.];result=object()
    class Camera:
        last_read_diagnostics={'result':'ok','stage_duration_sec':{}}
        runtime_profile={'stationary_temporal':{'active':True}}
        fail=False
        def read(self):
            if self.fail:raise ValueError('original read error')
            return result
    original=Camera.read
    tap=observer.ExistingReadObserver(Camera,tmp_path/'read.jsonl',seconds=1.,clock=lambda:now[0])
    try:
        camera=Camera();assert camera.read() is result
        camera.fail=True;now[0]=.5
        with pytest.raises(ValueError,match='original read error'):camera.read()
        camera.fail=False;now[0]=1.;assert camera.read() is result
        assert Camera.read is original
    finally:tap.shutdown()
    rows=[json.loads(s) for s in tap.output.read_text().splitlines()]
    assert len(rows)==2
    assert all(row['existing_read_diagnostics']['result']=='ok' for row in rows)
    summary=json.loads(tap.summary_path.read_text())
    assert summary['original_read_restored'] and summary['counts']['written']==2


def test_observer_queue_overflow_never_blocks_camera_read(monkeypatch,tmp_path):
    monkeypatch.setattr(observer.threading.Thread,'start',lambda self:None)
    class Camera:
        last_read_diagnostics={'result':'ok'}
        runtime_profile={}
        def read(self):return 123
    original=Camera.read
    tap=observer.ExistingReadObserver(Camera,tmp_path/'bounded.jsonl',queue_size=1)
    camera=Camera()
    assert camera.read()==123 and camera.read()==123
    assert tap.counts['queue_dropped']==1
    tap.finish();tap._writer()
    assert Camera.read is original
    assert json.loads(tap.summary_path.read_text())['counts']['queue_dropped']==1


def test_observer_copies_only_publication_metadata_and_not_camera_pixels(tmp_path):
    status={'worker_alive':True,'pending':1,'failure':None,'ages':{'max':.005}}
    image=object()
    class Camera:
        last_read_diagnostics={'result':'ok'}
        runtime_profile={}
        publication_diagnostics_provider=staticmethod(lambda:status)
        @property
        def image(self):pytest.fail('observer must never access camera pixels')
        def read(self):return image
    tap=observer.ExistingReadObserver(Camera,tmp_path/'meta.jsonl')
    try:
        assert Camera().read() is image
        status['ages']['max']=100.
    finally:tap.shutdown()
    row=json.loads(tap.output.read_text())
    assert row['publication_worker']['ages']['max']==.005
    assert tap.counts['copy_errors']==0


@pytest.mark.parametrize('original_fails',[False,True])
def test_publication_metadata_error_never_changes_camera_read_outcome(tmp_path,original_fails):
    def broken_provider():raise RuntimeError('metadata provider error')
    result=object()
    class Camera:
        last_read_diagnostics={'result':'ok'}
        runtime_profile={}
        publication_diagnostics_provider=staticmethod(broken_provider)
        def read(self):
            if original_fails:raise ValueError('original error')
            return result
    original=Camera.read
    tap=observer.ExistingReadObserver(Camera,tmp_path/'bad_meta.jsonl')
    try:
        if original_fails:
            with pytest.raises(ValueError,match='original error'):Camera().read()
        else:assert Camera().read() is result
    finally:tap.shutdown()
    assert Camera.read is original
    assert tap.counts['copy_errors']==1 and tap.counts['enqueued']==0
    assert 'metadata provider error' in tap.errors[0]['error']
