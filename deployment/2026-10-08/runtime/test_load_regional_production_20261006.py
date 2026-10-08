"""Camera-only opt-in contracts; no ROS, systemd, SDK or device actions."""
from copy import deepcopy
import importlib.util
import json
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

HERE = Path(__file__).parent
spec = importlib.util.spec_from_file_location('regional_production_loader', HERE/'load_regional_production_20261006.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
spec = importlib.util.spec_from_file_location('regional_production_owner_helper', HERE/'gen4_root_cause/reload_camera_visual_gate.py')
h = importlib.util.module_from_spec(spec)
spec.loader.exec_module(h)


def configuration():
    return dict(fps=30, publish_fps=20, simulate=False, fallback_to_simulation=False,
        align_depth_to_color=True, pair_rectification=dict(enabled=True),
        mode_depth_preset=dict(enabled=True, stationary_temporal_enabled=True,
            unknown_preset='high_accuracy', baseline_path='/original'),
        depth_filter=dict(spatial_enabled=True), fx=395., runtime_profile={'old': 'diagnostic'})


def test_source_manifest_requires_tested_pair_history_helper(tmp_path):
    prefix = 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/'
    names = ('realsense_manager.py', 'regional_stationary_depth.py',
             'visual_stationary.py', 'stationary_depth.py', 'pair_history_continuity.py')
    files = {}
    for name in names:
        path = tmp_path/(prefix+name)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('# '+name)
        files[prefix+name] = m.sha(path)
    manifest = dict(tests_passed=True, reviewed=True, files=files)
    m.verify_manifest(manifest, root=tmp_path)
    with pytest.raises(RuntimeError,match='snapshot source manifest incomplete'):
        m.verify_manifest(manifest,root=tmp_path,capture_visual_snapshots=True)
    with pytest.raises(RuntimeError,match='flag must be boolean'):
        m.verify_manifest(manifest,root=tmp_path,capture_visual_snapshots='false')
    for name in (m.VISUAL_TAP,'.runtime/gen4_root_cause/camera_existing_read_observer.py',
                 '.runtime/load_regional_production_20261006.py'):
        path=tmp_path/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_text('# diagnostic')
        files[name]=m.sha(path)
    m.verify_manifest(manifest,root=tmp_path,capture_visual_snapshots=True)
    missing = deepcopy(manifest)
    missing['files'].pop(prefix+'pair_history_continuity.py')
    with pytest.raises(RuntimeError, match='manifest incomplete'):
        m.verify_manifest(missing, root=tmp_path)
    (tmp_path/(prefix+'pair_history_continuity.py')).write_text('# changed')
    with pytest.raises(RuntimeError, match='frozen source changed'):
        m.verify_manifest(manifest, root=tmp_path)


def test_preserve_changes_only_one_flag_and_does_not_mutate_input():
    before = configuration(); frozen = deepcopy(before)
    desired = m.desired_camera_configuration(before, 'preserve')
    assert before == frozen
    desired['mode_depth_preset'].pop('regional_stationary_enabled')
    assert desired == before


def test_node_timing_manifest_requires_tested_tool_and_isolated_capture(tmp_path):
    prefix = 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/'
    names = [prefix+x for x in ('realsense_manager.py', 'regional_stationary_depth.py',
        'visual_stationary.py', 'stationary_depth.py', 'pair_history_continuity.py')]
    files={}
    for name in names:
        path=tmp_path/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_text('# frozen')
        files[name]=m.sha(path)
    manifest=dict(tests_passed=True,reviewed=True,files=files)
    with pytest.raises(RuntimeError,match='node timing source manifest incomplete'):
        m.verify_manifest(manifest,root=tmp_path,capture_node_timings=True)
    for name in (m.NODE_TIMING_TAP,'.runtime/gen4_root_cause/camera_existing_read_observer.py',
                 '.runtime/load_regional_production_20261006.py',
                 'src/alicia_flexible_grasp_supervisor/scripts/camera_node.py'):
        path=tmp_path/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_text('# frozen')
        files[name]=m.sha(path)
    m.verify_manifest(manifest,root=tmp_path,capture_node_timings=True)
    with pytest.raises(RuntimeError,match='isolated'):
        m.verify_manifest(manifest,root=tmp_path,capture_visual_snapshots=True,capture_node_timings=True)
    with pytest.raises(RuntimeError,match='flag must be boolean'):
        m.verify_manifest(manifest,root=tmp_path,capture_node_timings='false')
    (tmp_path/m.NODE_TIMING_TAP).write_text('# changed')
    with pytest.raises(RuntimeError,match='frozen source changed'):
        m.verify_manifest(manifest,root=tmp_path,capture_node_timings=True)


def test_node_timing_option_survives_runtime_constructor(monkeypatch):
    runtime=m.runtime_class(None,None,None,None)
    calls=[]
    monkeypatch.setattr(runtime,'initialize',lambda self,*a:calls.append(a))
    runtime('out','receipt','body','manifest','preserve','manifest_path',False,True)
    assert calls==[('out','receipt','body','manifest','preserve','manifest_path',False,True)]


def node_timing_fixture(tmp_path):
    source=m.W/'src/alicia_flexible_grasp_supervisor/scripts/camera_node.py'
    entry=m.W/'devel/lib/alicia_flexible_grasp_supervisor/camera_node.py'
    files={str(p.resolve()):'frozen-'+p.parent.name for p in (source,entry,m.W/m.NODE_TIMING_TAP)}
    loaded={key:dict(path=str(p.resolve()),sha256=files[str(p.resolve())]) for key,p in (
        ('diagnostic.camera_node_timing_source',source),('diagnostic.camera_node_timing_entry',entry),
        ('diagnostic.node_timing_tap',m.W/m.NODE_TIMING_TAP))}
    events=tmp_path/'events.jsonl';events.write_text('{"event":"start"}\n{"event":"end"}\n')
    summary=dict(schema=1,complete=True,sealed=True,diagnostic_only=True,no_pixel_copies=True,
        class_hooks_restored=True,observed_bound_callbacks_restored=True,source_implementation=loaded,
        errors=[],queued_unwritten=0,unfinished_calls=[],requested_window_sec=30,
        window_complete=True,close_reason='window_elapsed',first_read_start_ns=100,
        closed_monotonic_ns=30000000100,event_bytes=events.stat().st_size,events_sha256=m.sha(events),
        counts=dict(started=1,completed=1,enqueued=2,written=2,queue_dropped=0,
                    diagnostic_errors=0,budget_rejected=0))
    (tmp_path/'summary.json').write_text(json.dumps(summary))
    return summary,loaded,files,entry


def test_node_timing_capture_keeps_catkin_entry_and_source_separate(tmp_path):
    summary,loaded,files,entry=node_timing_fixture(tmp_path)
    assert m.verify_node_timing_capture(summary,tmp_path,loaded,files,entry)==summary
    with pytest.raises(RuntimeError,match='source mismatch'):
        m.verify_node_timing_capture(summary,tmp_path,loaded,files)


@pytest.mark.parametrize('fault',['unsealed','class_hook','bound_hook','unfinished','short_window',
    'shutdown','time_boundary','count','changed_bytes','changed_source','queue_loss','unexpected','symlink'])
def test_node_timing_rejects_incomplete_or_tampered_artifacts(tmp_path,fault):
    summary,loaded,files,entry=node_timing_fixture(tmp_path)
    if fault=='unsealed':summary['sealed']=False
    elif fault=='class_hook':summary['class_hooks_restored']=False
    elif fault=='bound_hook':summary['observed_bound_callbacks_restored']=False
    elif fault=='unfinished':summary['unfinished_calls']=[{'call_id':2}]
    elif fault=='short_window':summary['window_complete']=False
    elif fault=='shutdown':summary['close_reason']='shutdown'
    elif fault=='time_boundary':summary['closed_monotonic_ns']-=1
    elif fault=='count':summary['counts']['written']=1
    elif fault=='changed_bytes':
        p=tmp_path/'events.jsonl';p.write_bytes(p.read_bytes().replace(b'start',b'other'))
    elif fault=='changed_source':files[str(entry.resolve())]='other'
    elif fault=='queue_loss':summary['counts']['queue_dropped']=1
    elif fault=='unexpected':(tmp_path/'extra').write_text('extra')
    elif fault=='symlink':
        p=tmp_path/'summary.json';p.unlink();p.symlink_to(tmp_path/'events.jsonl')
    with pytest.raises(RuntimeError):m.verify_node_timing_capture(summary,tmp_path,loaded,files,entry)


def test_node_timing_unwrap_does_not_reenable_and_waits_for_callbacks(tmp_path,monkeypatch):
    monkeypatch.setattr(m,'EXT',tmp_path)
    diagnostic=tmp_path/'read.jsonl'
    argv=['/usr/bin/python3',str(h.OBSERVER),'--camera-script',str(h.CAMERA_ENTRIES[0]),
        '--diagnostic-path',str(diagnostic),'--diagnostic-seconds','30','--capture-node-timings',
        '--','__name:=supervisor_camera']
    diagnostic.with_suffix('.summary.json').write_text(json.dumps(dict(original_read_restored=True)))
    (tmp_path/'node_timings').mkdir();p=tmp_path/'node_timings/summary.json'
    p.write_text(json.dumps(dict(sealed=True,class_hooks_restored=True,observed_bound_callbacks_restored=False)))
    with pytest.raises(RuntimeError,match='still active'):m.camera_command(argv,h)
    p.write_text(json.dumps(dict(sealed=True,class_hooks_restored=True,observed_bound_callbacks_restored=True)))
    assert m.camera_command(argv,h)==[argv[0],str(h.CAMERA_ENTRIES[0]),'__name:=supervisor_camera']
    doubled=list(argv);doubled.insert(2,'--capture-node-timings')
    with pytest.raises(RuntimeError,match='malformed'):m.camera_command(doubled,h)


def test_explicit_baseline_changes_only_the_two_allowed_fields():
    before = configuration(); desired = m.desired_camera_configuration(before, 'saved_baseline')
    assert desired['mode_depth_preset']['regional_stationary_enabled'] is True
    assert desired['mode_depth_preset']['unknown_preset'] == 'saved_baseline'
    desired['mode_depth_preset'] = before['mode_depth_preset']
    assert desired == before


@pytest.mark.parametrize('fault', ['fps', 'publish_fps', 'simulate', 'align_depth_to_color',
    'fallback_to_simulation', 'pair_rectification', 'mode_enabled', 'stationary', 'invalid_flag', 'preset'])
def test_existing_policy_mismatch_is_rejected(fault):
    c = configuration()
    if fault in ('fps', 'publish_fps'):c[fault] = 10
    elif fault in ('simulate', 'fallback_to_simulation'):c[fault] = True
    elif fault == 'align_depth_to_color':c[fault] = False
    elif fault == 'pair_rectification':c[fault]['enabled'] = False
    elif fault == 'mode_enabled':c['mode_depth_preset']['enabled'] = False
    elif fault == 'stationary':c['mode_depth_preset']['stationary_temporal_enabled'] = False
    elif fault == 'invalid_flag':c['mode_depth_preset']['regional_stationary_enabled'] = 'true'
    else:c['mode_depth_preset']['unknown_preset'] = 'carton'
    with pytest.raises(RuntimeError):m.desired_camera_configuration(c, 'preserve')


def test_no_execute_never_imports_runtime(monkeypatch):
    monkeypatch.setattr(m, 'load', lambda *a: pytest.fail('runtime loaded without execute'))
    with pytest.raises(SystemExit):m.main(['--alignment-receipt', '/not/read.json'])


def receipt_fixture(tmp_path):
    source = dict(kind='user_message', message_id='actual-message', text='已对准')
    feedback = dict(frame_id='sdk_measured', positions=[0.]*6, stamp_ns='30')
    event = dict(confirmation_source=source, confirmed_ns='20', bound_feedback=feedback, target_aligned=True)
    event_path = tmp_path/'event.json'; event_path.write_text(json.dumps(event))
    receipt = dict(kind='explicit_user_target_alignment', target_aligned=True,
        confirmation_id='actual-id:0', confirmation_source=source, confirmed_ns='20',
        bound_feedback=feedback, epoch_ns='10', event_file=str(event_path), event_file_sha256=m.sha(event_path))
    path = tmp_path/'receipt.json';path.write_text(json.dumps(receipt))
    return path, receipt


def test_receipt_preserves_external_user_provenance(tmp_path):
    path, receipt = receipt_fixture(tmp_path)
    assert m.read_receipt(path, tmp_path) == receipt


def stationary_receipt_fixture(tmp_path):
    source=dict(kind='user_message',message_id='actual-stationary-confirmation:0',
                text='可以，现场已保持静止')
    event=dict(kind='explicit_user_stationary_capture',scene_stationary=True,
        camera_diagnostic_only=True,motion_authorized=False,confirmation_id='stationary:0',
        confirmation_source=source,confirmed_ns='20',epoch_ns='10',
        bound_feedback=dict(frame_id='sdk_measured',positions=[0.,0.,0.,.0061359,0.,0.],stamp_ns='30'))
    event_path=tmp_path/'stationary_event.json';event_path.write_text(json.dumps(event))
    receipt=dict(deepcopy(event),event_file=str(event_path),event_file_sha256=m.sha(event_path))
    path=tmp_path/'stationary_receipt.json';path.write_text(json.dumps(receipt))
    return path,receipt,event


def save_stationary_fixture(path,receipt,event):
    event_path=Path(receipt['event_file']);event_path.write_text(json.dumps(event))
    receipt['event_file_sha256']=m.sha(event_path);path.write_text(json.dumps(receipt))


def test_stationary_capture_is_preserved_without_manufacturing_target_alignment(tmp_path):
    path,receipt,event=stationary_receipt_fixture(tmp_path)
    assert m.read_receipt(path,tmp_path)==receipt
    assert 'target_aligned' not in receipt and receipt['kind']=='explicit_user_stationary_capture'
    report=m.receipt_report_fields(receipt,path,m.sha(path))
    assert report['stationary_capture_receipt']==receipt and 'alignment_receipt' not in report
    assert report['target_alignment_confirmation_used'] is False
    assert report['scene_stationary_confirmation_used'] is True
    assert report['confirmation_provenance_scope']=='camera_diagnostic_only_no_target_alignment_no_motion'
    assert report['confirmation_receipt']==dict(event,event_file=receipt['event_file'],
        event_file_sha256=receipt['event_file_sha256'])


def test_legacy_alignment_report_fields_remain_unchanged(tmp_path):
    path,receipt=receipt_fixture(tmp_path)
    report=m.receipt_report_fields(receipt,path,m.sha(path))
    assert report['alignment_receipt']==receipt and report['alignment_receipt_path']==str(path)
    assert report['alignment_receipt_sha256']==m.sha(path) and 'stationary_capture_receipt' not in report
    assert report['target_alignment_confirmation_used'] is True


@pytest.mark.parametrize('field,value',[
    ('scene_stationary',False),('scene_stationary',1),('camera_diagnostic_only',False),
    ('camera_diagnostic_only',1),('motion_authorized',True),('motion_authorized',0),
    ('target_aligned',True),('target_aligned',1),('target_aligned','false')])
def test_stationary_capture_scope_cannot_be_widened_in_either_record(tmp_path,field,value):
    path,receipt,event=stationary_receipt_fixture(tmp_path)
    receipt[field]=value;event[field]=value;save_stationary_fixture(path,receipt,event)
    with pytest.raises(RuntimeError):m.read_receipt(path,tmp_path)


@pytest.mark.parametrize('field',['kind','scene_stationary','camera_diagnostic_only',
    'motion_authorized','confirmation_id','epoch_ns','confirmed_ns','bound_feedback','confirmation_source'])
def test_stationary_capture_requires_exact_event_binding_even_with_valid_new_hash(tmp_path,field):
    path,receipt,event=stationary_receipt_fixture(tmp_path)
    event.pop(field);save_stationary_fixture(path,receipt,event)
    with pytest.raises(RuntimeError):m.read_receipt(path,tmp_path)


@pytest.mark.parametrize('fault',['epoch_after_confirmation','feedback_before_confirmation',
    'zero_epoch','float_stamp','bool_epoch','seven_joints','nan_joint','bool_joint',
    'commanded_feedback','empty_message','missing_message_id','nonstring_message_id','blank_id'])
def test_stationary_capture_keeps_real_feedback_timestamp_and_user_source_contracts(tmp_path,fault):
    path,receipt,event=stationary_receipt_fixture(tmp_path)
    if fault=='epoch_after_confirmation':event['epoch_ns']='21'
    elif fault=='feedback_before_confirmation':event['bound_feedback']['stamp_ns']='19'
    elif fault=='zero_epoch':event['epoch_ns']='0'
    elif fault=='float_stamp':event['confirmed_ns']=20.5
    elif fault=='bool_epoch':event['epoch_ns']=True
    elif fault=='seven_joints':event['bound_feedback']['positions'].append(0.)
    elif fault=='nan_joint':event['bound_feedback']['positions'][0]=float('nan')
    elif fault=='bool_joint':event['bound_feedback']['positions'][0]=False
    elif fault=='commanded_feedback':event['bound_feedback']['frame_id']='commanded'
    elif fault=='empty_message':event['confirmation_source']['text']='  '
    elif fault=='missing_message_id':event['confirmation_source'].pop('message_id')
    elif fault=='nonstring_message_id':event['confirmation_source']['message_id']=1
    else:event['confirmation_id']='  '
    receipt.update(deepcopy(event));save_stationary_fixture(path,receipt,event)
    with pytest.raises(RuntimeError):m.read_receipt(path,tmp_path)


def test_stationary_capture_uses_original_pose_monitor_and_latches_same_excursion(tmp_path,monkeypatch):
    for relative in ('src/alicia_grasp_modes/src','src/alicia_flexible_grasp_supervisor/src'):
        monkeypatch.syspath_prepend(str(m.W/relative))
    alignment=m.load('stationary_receipt_original_pose_monitor','tools/diagnostics/prepare_rgb_acquisition_retry.py')
    path,receipt,_=stationary_receipt_fixture(tmp_path)
    accepted=m.read_receipt(path,tmp_path);monitor=alignment.AlignmentPoseMonitor(accepted)
    monitor.observe(31,accepted['bound_feedback']['positions']);monitor.require_stationary()
    moved=list(accepted['bound_feedback']['positions']);moved[3]+=alignment.POSE_TOLERANCE_RAD+1e-8
    monitor.observe(32,moved)
    with pytest.raises(ValueError,match='measured_pose_excursion'):monitor.require_stationary()
    monitor.observe(33,accepted['bound_feedback']['positions'])
    with pytest.raises(ValueError,match='measured_pose_excursion'):monitor.require_stationary()


@pytest.mark.parametrize('fault', ['hash', 'source', 'feedback', 'epoch', 'frame', 'aligned', 'id'])
def test_receipt_mismatch_is_rejected_without_manufactured_confirmation(tmp_path, fault):
    path, receipt = receipt_fixture(tmp_path)
    if fault == 'hash':receipt['event_file_sha256'] = '0'*64
    elif fault == 'source':receipt['confirmation_source']['text'] = 'invented'
    elif fault == 'feedback':receipt['bound_feedback']['positions'][0] = .1
    elif fault == 'epoch':receipt['epoch_ns'] = '40'
    elif fault == 'frame':receipt['bound_feedback']['frame_id'] = 'commanded'
    elif fault == 'aligned':receipt['target_aligned'] = False
    elif fault == 'id':receipt['confirmation_id'] = ''
    path.write_text(json.dumps(receipt))
    with pytest.raises(RuntimeError):m.read_receipt(path, tmp_path)


def test_normal_entry_retains_ros_remaps():
    argv = ['/usr/bin/python3', str(h.CAMERA_ENTRIES[0]), '/foo:=/bar', '__name:=supervisor_camera']
    assert m.camera_command(argv, h) == argv


def test_active_observer_is_not_interrupted_as_finished(tmp_path):
    diagnostic = tmp_path/'read.jsonl'
    argv = ['/usr/bin/python3', str(h.OBSERVER), '--camera-script', str(h.CAMERA_ENTRIES[0]),
        '--diagnostic-path', str(diagnostic), '--diagnostic-seconds', '30', '--', '__name:=supervisor_camera']
    with pytest.raises(RuntimeError, match='still active'):m.camera_command(argv, h)
    diagnostic.with_suffix('.summary.json').write_text(json.dumps(dict(original_read_restored=True)))
    assert m.camera_command(argv, h) == [argv[0], str(h.CAMERA_ENTRIES[0]), '__name:=supervisor_camera']


def test_unknown_wrapper_is_rejected():
    with pytest.raises(RuntimeError):m.camera_command(['/usr/bin/python3', '/unknown.py', '__name:=supervisor_camera'], h)


def test_visual_snapshot_unwrap_requires_all_hooks_restored_and_does_not_reenable(tmp_path,monkeypatch):
    monkeypatch.setattr(m,'EXT',tmp_path)
    diagnostic=tmp_path/'read.jsonl'
    argv=['/usr/bin/python3',str(h.OBSERVER),'--camera-script',str(h.CAMERA_ENTRIES[0]),
        '--diagnostic-path',str(diagnostic),'--diagnostic-seconds','30','--capture-visual-snapshots',
        '--','__name:=supervisor_camera']
    diagnostic.with_suffix('.summary.json').write_text(json.dumps(dict(original_read_restored=True)))
    (tmp_path/'visual_snapshots').mkdir()
    p=tmp_path/'visual_snapshots/summary.json'
    p.write_text(json.dumps(dict(sealed=True,original_process_restored=True,original_observe_restored=True)))
    with pytest.raises(RuntimeError,match='still active'):m.camera_command(argv,h)
    p.write_text(json.dumps(dict(sealed=True,original_process_restored=True,
        original_observe_restored=True,original_reset_restored=True)))
    assert m.camera_command(argv,h)==[argv[0],str(h.CAMERA_ENTRIES[0]),'__name:=supervisor_camera']
    doubled=list(argv);doubled.insert(2,'--capture-visual-snapshots')
    with pytest.raises(RuntimeError,match='malformed'):m.camera_command(doubled,h)


def test_visual_capture_import_requires_actual_path_and_hash():
    path=str((m.W/m.VISUAL_TAP).resolve());sources={path:'frozen'}
    good={'diagnostic.visual_snapshot_tap':dict(path=path,sha256='frozen')}
    m.verify_visual_snapshot_import(good,sources)
    for bad in ({},{'diagnostic.visual_snapshot_tap':dict(path=path,sha256='different')},
                {'diagnostic.visual_snapshot_tap':dict(path='/elsewhere',sha256='frozen')}):
        with pytest.raises(RuntimeError,match='snapshot helper'):m.verify_visual_snapshot_import(bad,sources)


def snapshot_summary(tmp_path):
    raw=b'bounded diagnostic file';p=tmp_path/'snapshot_00.npz';p.write_bytes(raw)
    summary=dict(sealed=True,original_process_restored=True,original_observe_restored=True,
        original_reset_restored=True,errors=[],counts=dict(written=1,enqueued=1),pixels_changed=False,
        motion_authorized=False,diagnostic_only=True,coordinate_domain='raw_sdk_color_before_pair_rectification',
        queued_unwritten=0,inflight_copies=0,queue_bytes=0,retained_anchor_bytes=0,
        maximum_bytes=64*1024*1024,reserved_bytes=len(raw),anchor_peak_bytes=100,
        requested_window_sec=30.,close_reason='window_expired',first_read_monotonic=1.,finished_monotonic=31.,
        files=[dict(name=p.name,category='center/visual_motion',bytes=len(raw),sha256=m.sha(p))],
        categories={'center/visual_motion':1},directory_bytes_before_summary=len(raw))
    (tmp_path/'summary.json').write_text(json.dumps(summary))
    return summary


def test_complete_bounded_capture_allows_no_stationary_child_samples(tmp_path):
    summary=snapshot_summary(tmp_path)
    assert m.verify_visual_capture(summary,tmp_path)==summary


@pytest.mark.parametrize('fault',['hooks','errors','queue','pixels','budget','duration','file_hash','category','count','extra_file'])
def test_visual_capture_rejects_incomplete_or_inconsistent_evidence(tmp_path,fault):
    s=snapshot_summary(tmp_path)
    if fault=='hooks':s['original_reset_restored']=False
    elif fault=='errors':s['errors']=['writer failed']
    elif fault=='queue':s['queued_unwritten']=1
    elif fault=='pixels':s['pixels_changed']=True
    elif fault=='budget':s['reserved_bytes']=64*1024*1024+1
    elif fault=='duration':s['finished_monotonic']=30.
    elif fault=='file_hash':s['files'][0]['sha256']='changed'
    elif fault=='category':s['categories']['center/visual_motion']=9
    elif fault=='count':s['counts']['enqueued']=2
    else:(tmp_path/'unexpected').write_text('unaccounted')
    with pytest.raises(RuntimeError):m.verify_visual_capture(s,tmp_path)


def row_fixture():
    regions = [dict(name=name, owned_pixels=20, qualified=(name == 'center'),
        filter_applied=(name == 'center')) for name in sorted(m.REGIONS)]
    e = dict(reason='regional_observation', owner_schema=m.OWNER, current_zero_pixels_preserved=True,
        regions=regions, filtered_pixel_count=20, active=True)
    p = dict(regional_enabled=True, active=False, whole_frame_stationarity_qualified=False,
        alignment_proven=True, owner_schema=m.OWNER, alpha=.4, delta_depth_units=100, persistence=0,
        visual_stationarity_required=True, visual_window_sec=.35, visual_max_motion_full_resolution_px=.25)
    return dict(existing_read_diagnostics=dict(result='complete', sdk_color_alignment_applied=True,
        pair_timing={'accepted': True}, regional_stationary_evidence=e), stationary_temporal=p)


def test_regional_active_with_legacy_false_passes():
    assert m.regional_read_summary([row_fixture()], 100)['passed']


@pytest.mark.parametrize('fault', ['global_active', 'alpha', 'persistence', 'alignment', 'zeros',
    'owners', 'count', 'inactive_applied', 'pair', 'no_evidence', 'unqualified_output'])
def test_regional_acceptance_does_not_weaken_ownership_or_policy(fault):
    row = row_fixture();d = row['existing_read_diagnostics'];e = d['regional_stationary_evidence'];p = row['stationary_temporal']
    if fault == 'global_active':p['active'] = True
    elif fault == 'alpha':p['alpha'] = .5
    elif fault == 'persistence':p['persistence'] = 1
    elif fault == 'alignment':d['sdk_color_alignment_applied'] = False
    elif fault == 'zeros':e['current_zero_pixels_preserved'] = False
    elif fault == 'owners':e['regions'][0]['owned_pixels'] += 1
    elif fault == 'count':e['filtered_pixel_count'] += 1
    elif fault == 'inactive_applied':e['regions'][0]['filter_applied'] = not e['regions'][0]['qualified']
    elif fault == 'pair':d['pair_timing']['accepted'] = False
    elif fault == 'no_evidence':d.pop('regional_stationary_evidence')
    else:e['reason'] = 'encoder_or_mode_inactive'
    assert not m.regional_read_summary([row], 100)['passed']


def test_warmup_raw_is_retained_but_cannot_alone_prove_enabled_output():
    row = row_fixture();e = row['existing_read_diagnostics']['regional_stationary_evidence']
    for region in e['regions']:region.update(qualified=False, filter_applied=False)
    e.update(active=False, filtered_pixel_count=0)
    assert not m.regional_read_summary([row], 100)['passed']
    assert m.regional_read_summary([row, row_fixture()], 100)['passed']


class FakeIO:
    def __init__(self, fault=None):self.events=[];self.report={};self.fault=fault
    def event(self, event):
        self.events.append(event)
        if self.fault == event:raise RuntimeError(event)
    def guard(self, force=False):self.event('guard')
    def verify(self):self.event('verify')
    def save(self, event):self.event(event)
    def stop_camera(self):self.event('stop camera')
    def configure(self):self.event('write allowed camera params')
    def start_camera(self):self.event('start camera')
    def validate(self):self.event('validate metadata')
    def close(self):self.event('close subscribers')


def test_transaction_only_stops_camera_before_configuration_and_has_no_actions():
    io = FakeIO();m.transaction(io)
    assert io.report['static_validation_passed'] is True
    assert io.events.index('stop camera') < io.events.index('write allowed camera params') < io.events.index('start camera')
    assert io.events[-1] == 'validated'


@pytest.mark.parametrize('fault', ['guard', 'verify', 'stop camera', 'write allowed camera params', 'start camera', 'validate metadata'])
def test_transaction_failure_does_not_continue_or_retry(fault):
    io = FakeIO(fault)
    with pytest.raises(RuntimeError, match=fault):m.transaction(io)
    assert io.report.get('static_validation_passed') is not True
    assert io.events[-1] == fault
    assert io.events.count('start camera') <= 1


def test_final_guard_failure_revokes_receipt_and_propagates_nonzero():
    io = FakeIO('guard');io.report['static_validation_passed'] = True
    with pytest.raises(RuntimeError, match='guard'):m.finalize(io)
    assert io.report['static_validation_passed'] is False
    assert io.events == ['guard', 'close subscribers', 'finished']


def test_constructor_failure_closes_readonly_subscribers(monkeypatch, tmp_path):
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', NS(Image=object))
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', NS(String=object))
    events = []
    class Evidence:
        def __init__(self, ros):events.append('subscribed')
        def close(self):events.append('closed')
    def node_pid(*a):raise RuntimeError('camera absent')
    ros = NS(get_param=lambda *a: configuration())
    helper = NS(identity=lambda x: x, node_pid=node_pid)
    alignment = NS(AlignmentPoseMonitor=lambda r: object())
    monkeypatch.setattr(m, 'baseline_identity', lambda c: {})
    cls = m.runtime_class(ros, helper, NS(RosEvidence=Evidence), alignment)
    receipt_path, receipt = receipt_fixture(tmp_path)
    with pytest.raises(RuntimeError, match='camera absent'):
        cls(tmp_path, receipt_path, receipt, {}, 'preserve')
    assert events == ['subscribed', 'closed']


def test_static_metadata_requires_real_cadence_and_pairing():
    summary = dict(original_read_restored=True, errors=[], queued_unwritten=0,
        counts=dict(observed=100, enqueued=100, written=100))
    reads = dict(rows=100, steady_read_fps=30., publication_worker_health=dict(passed=True))
    pubs = {k: dict(fps=20., frames=3, unique_source_stamps=3, header_duplicates=0,
        header_regressions=0) for k in ('rgb', 'depth')}
    regional = dict(passed=True)
    assert m.validate_static(summary, reads, pubs, regional, {1,2,3}, {1,2,3})
    reads['steady_read_fps'] = 12.
    assert not m.validate_static(summary, reads, pubs, regional, {1,2,3}, {1,2,3})
    reads['steady_read_fps'] = 30.
    assert not m.validate_static(summary, reads, pubs, regional, {1,2,3}, {7,8,9})


@pytest.mark.parametrize('fault', ['duplicates', 'regressions', 'repeated_pair', 'missing'])
def test_static_cadence_rejects_fast_republication_of_old_or_regressing_frames(fault):
    summary = dict(original_read_restored=True, errors=[], queued_unwritten=0,
        counts=dict(observed=100, enqueued=100, written=100))
    reads = dict(rows=100, steady_read_fps=30., publication_worker_health=dict(passed=True))
    pubs = {k: dict(fps=20., frames=100, unique_source_stamps=100, header_duplicates=0,
        header_regressions=0) for k in ('rgb', 'depth')}
    if fault == 'duplicates':pubs['rgb']['header_duplicates'] = 1
    elif fault == 'regressions':pubs['rgb']['header_regressions'] = 1
    elif fault == 'repeated_pair':pubs['rgb']['unique_source_stamps'] = 2
    else:pubs['rgb'].pop('header_duplicates')
    assert not m.validate_static(summary, reads, pubs, {'passed': True}, {1,2}, {1,2})


def test_baseline_content_is_frozen_beyond_its_filename(tmp_path):
    p = tmp_path/'baseline.json'; settings = dict(parameters=dict(value=1))
    p.write_text(json.dumps(dict(advanced_settings=settings)))
    expected = m.hashlib.sha256(json.dumps(settings).encode()).hexdigest()
    c = dict(mode_depth_preset={'baseline_path': str(p)},
        runtime_profile=dict(mode_depth_preset={'baseline_sha256': expected}))
    identity = m.baseline_identity(c)
    assert identity['advanced_settings_sha256'] == expected and identity['file_sha256'] == m.sha(p)
    p.write_text(json.dumps(dict(advanced_settings={'parameters': {'value': 2}})))
    with pytest.raises(RuntimeError, match='baseline content'):m.baseline_identity(c)


def test_actual_regional_import_is_required_and_bound_to_path():
    path = (m.W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/regional_stationary_depth.py').resolve()
    pair_path = path.with_name('pair_history_continuity.py')
    hashes = {str(p): m.sha(p) for p in (path, pair_path)}
    with pytest.raises(RuntimeError, match='actual regional'):m.verify_regional_import({}, hashes)
    key = 'alicia_flexible_grasp.vision.regional_stationary_depth'
    loaded = {key: dict(path=str(path), sha256=hashes[str(path)])}
    pair_key = 'alicia_flexible_grasp.vision.pair_history_continuity'
    with pytest.raises(RuntimeError, match='pair_history'):m.verify_regional_import(loaded, hashes)
    loaded[pair_key] = dict(path=str(pair_path), sha256=hashes[str(pair_path)])
    m.verify_regional_import(loaded, hashes)
    loaded[pair_key]['sha256'] = 'different'
    with pytest.raises(RuntimeError, match='pair_history'):m.verify_regional_import(loaded, hashes)
    loaded[pair_key]['sha256'] = hashes[str(pair_path)]
    loaded[key]['path'] = '/different/runtime/module.py'
    with pytest.raises(RuntimeError, match='actual regional'):m.verify_regional_import(loaded, hashes)


def test_existing_observer_records_real_regional_import_without_devices(monkeypatch, tmp_path):
    spec = importlib.util.spec_from_file_location('regional_observer_manifest', h.OBSERVER)
    observer = importlib.util.module_from_spec(spec);spec.loader.exec_module(observer)
    imported = []
    def fake_import(name):
        imported.append(name)
        path = tmp_path/(name.rsplit('.', 1)[-1]+'.py');path.write_text('# module '+name)
        return NS(__file__=str(path))
    monkeypatch.setattr(observer.importlib, 'import_module', fake_import)
    loaded = observer.loaded_module_manifest()
    assert len(imported) == 5
    key = 'alicia_flexible_grasp.vision.regional_stationary_depth'
    assert key in loaded and loaded[key]['sha256'] == m.sha(loaded[key]['path'])
    key = 'alicia_flexible_grasp.vision.pair_history_continuity'
    assert key in loaded and loaded[key]['sha256'] == m.sha(loaded[key]['path'])
