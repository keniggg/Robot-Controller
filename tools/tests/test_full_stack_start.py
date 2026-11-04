"""Offline manual-start boundaries; never imports ROS or starts any service."""
from collections import deque
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as NS
import threading
import time
from unittest.mock import patch

import pytest

SOURCE = Path(__file__).resolve().parents[1]/'full_stack_start.py'
spec = importlib.util.spec_from_file_location('full_stack_start_test', SOURCE)
cold = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cold)
from storage_test_support import fake_storage


def stamp(value):
    return NS(to_nsec=lambda:value)


def msg(owner, now, **values):
    return NS(_connection_header={'callerid':owner}, header=NS(stamp=stamp(now), frame_id='sdk_measured'), **values)


def ready_monitor():
    now = 10_000_000_000
    monitor = cold.Monitor.__new__(cold.Monitor)
    monitor.lock = threading.Lock()
    monitor.rospy = NS(Time=NS(now=lambda:stamp(now)))
    joints = [msg('/alicia_d_driver_node', now-195_000_000+i*10_000_000,
                  name=['Joint%d'%i for i in range(1,7)], position=[0.]*6) for i in range(20)]
    values = dict(joint=joints[-1], epoch=msg('/alicia_d_driver_node',now,stamp=stamp(now-1_000_000_000)),
        manual=msg('/alicia_supervisor_gui',now,data=True),task=msg('/grasp_task_node',now,active=False),
        mode=msg('/grasp_mode_router',now,data='{"active":false,"start_reserved":false,"pending":""}'),
        arm=msg('/bessica_d_hw_interface',now,status_list=[]),hand=msg('/bessica_d_hw_interface',now,status_list=[]),
        motion_enabled=msg('/alicia_d_driver_node',now,data=False),
        actuation=msg('/alicia_d_driver_node',now,data='UNCONFIRMED:NEW_CONTROL_REFERENCE'),
        color=msg('/supervisor_camera',now-5_000_000,width=4,height=3,step=12,data=bytes(36),encoding='bgr8'),
        depth=msg('/supervisor_camera',now-5_000_000,width=4,height=3,step=8,data=bytes(24),encoding='16UC1'))
    monitor.values = {k:(time.monotonic(),v) for k,v in values.items()}
    values['task']._connection_header['latching'] = '1'
    values['manual']._connection_header['latching'] = '1'
    monitor.joints = deque(joints)
    monitor.images = {k:deque([values[k]]) for k in ('color','depth')}
    return monitor, now


def test_manual_startup_accepts_unconfirmed_without_motion():
    monitor, now = ready_monitor()
    result = monitor.sample(now-2_000_000_000)
    assert result['motion_enabled'] is False
    assert result['actuation_status'] == 'UNCONFIRMED:NEW_CONTROL_REFERENCE'
    assert result['actuation_confirmation_not_required_for_manual_startup'] is True
    assert result['scene_clearance_granted'] is False


def test_delayed_event_latch_is_not_a_missing_heartbeat():
    monitor, now = ready_monitor()
    _, task = monitor.values['task']
    monitor.values['task'] = (time.monotonic()-120., task)
    result = monitor.sample(now-2_000_000_000)
    assert result['task_active'] is False
    assert result['task_state_received_age_sec'] >= 120.
    assert result['task_state_contract'] == 'event_driven_latched_current_publisher'


@pytest.mark.parametrize('fault', ['stale_feedback','moving','wrong_feedback_owner','active_action','wrong_gui_owner','old_epoch','unpaired_images',
                                  'active_task','wrong_task_owner','unlatched_task','earlier_feedback_owner','unlatched_gui'])
def test_manual_startup_keeps_real_feedback_and_ownership_boundaries(fault):
    monitor, now = ready_monitor()
    values = {k:v[1] for k,v in monitor.values.items()}
    if fault == 'stale_feedback':
        for m in monitor.joints:
            m.header.stamp = stamp(m.header.stamp.to_nsec()-300_000_000)
    elif fault == 'moving': monitor.joints[-1].position[0] = .005
    elif fault == 'wrong_feedback_owner': values['joint']._connection_header['callerid'] = '/fake'
    elif fault == 'active_action': values['arm'].status_list = [NS(status=1)]
    elif fault == 'wrong_gui_owner': values['manual']._connection_header['callerid'] = '/fake'
    elif fault == 'old_epoch': values['epoch'].stamp = stamp(now-3_000_000_000)
    elif fault == 'unpaired_images': values['depth'].header.stamp = stamp(now-4_000_000)
    elif fault == 'active_task': values['task'].active = True
    elif fault == 'wrong_task_owner': values['task']._connection_header['callerid'] = '/fake'
    elif fault == 'unlatched_task': values['task']._connection_header['latching'] = '0'
    elif fault == 'earlier_feedback_owner': monitor.joints[0]._connection_header['callerid'] = '/fake'
    elif fault == 'unlatched_gui': values['manual']._connection_header['latching'] = '0'
    with pytest.raises(RuntimeError): monitor.sample(now-2_000_000_000)


def targeted_fixture():
    baseline = dict(passed=True,full_regression_passed=True,scope_complete=True,test_bodies_executed=True,
        summary=dict(tests=101,failures=0,errors=0,skipped=0),source_sha256={'old.py':'a','changed.py':'b'})
    focus = dict(passed=True,full_regression_passed=False,exact_case_inventory=True,duplicate_cases=0,omitted_cases=0,
        production_sha256={'changed.py':'c'},current_test_sha256={'test_new.py':'d'},total_distinct_case_count=2,
        case_results=[dict(identity=['file','module',str(i)],passed=True) for i in range(2)])
    release = dict(passed=True,reviewed=True,installed=True,source_count=3,kind='targeted_compute_release',full_regression_passed=False,
        source_sha256={'old.py':'a','changed.py':'c','test_new.py':'d'},baseline_full_regression='baseline',actual_targeted_regression='focus',
        targeted_distinct_cases=2,baseline_full_scope='historical only',changed_production_sources=['changed.py'],changed_test_sources=['test_new.py'])
    return baseline,focus,release


def test_targeted_release_never_relabels_historical_full_run():
    baseline,focus,release = targeted_fixture()
    with patch.object(cold,'read_ref',side_effect=lambda ref:{'baseline':baseline,'focus':focus}[ref]):
        result = cold.release_evidence(release)
    assert result['baseline_summary']['tests'] == 101
    assert result['targeted_distinct_cases'] == 2
    assert result['current_full_regression_passed'] is False


@pytest.mark.parametrize('fault',['duplicate','failed_case','untested_delta','false_full_claim','baseline_failed'])
def test_release_requires_exact_passing_evidence(fault):
    baseline,focus,release = targeted_fixture()
    if fault == 'duplicate': focus['case_results'][1] = copy.deepcopy(focus['case_results'][0])
    elif fault == 'failed_case': focus['case_results'][1]['passed'] = False
    elif fault == 'untested_delta': release['source_sha256']['old.py'] = 'changed-without-evidence'
    elif fault == 'false_full_claim': release['full_regression_passed'] = True
    elif fault == 'baseline_failed': baseline['summary']['failures'] = 1
    with patch.object(cold,'read_ref',side_effect=lambda ref:{'baseline':baseline,'focus':focus}[ref]):
        with pytest.raises(RuntimeError): cold.release_evidence(release)


@pytest.mark.parametrize('payload', ['{}', '{"active":false,"pending":""}',
    '{"active":0,"pending":"","start_reserved":false}',
    '{"active":false,"pending":false,"start_reserved":false}',
    '{"active":false,"pending":null,"start_reserved":false}',
    '{"active":false,"pending":"unknown","start_reserved":false}',
    '{"active":true,"active":false,"pending":"","start_reserved":false}'])
def test_startup_rejects_incomplete_malformed_or_busy_router(payload):
    monitor, now = ready_monitor()
    monitor.values['mode'][1].data = payload
    with pytest.raises(ValueError): monitor.sample(now-2_000_000_000)


def current_fixture(tmp_path):
    path = tmp_path/'test_current.py'; path.write_text('current source\n')
    sources = {'test_current.py':hashlib.sha256(path.read_bytes()).hexdigest()}
    run = dict(passed=True,source_unchanged=True,source_sha256=sources,external_boundary_attempts=[],
        full_regression_passed=True,scope_name='current declared offline scope',test_files=['test_current.py'],
        test_roots=['.'], explicit_test_files=[],
        expected_case_ids=['test_current.py::test_one'],excluded_files=[],
        case_results=[dict(nodeid='test_current.py::test_one',setup='passed',call='passed',teardown='passed')])
    release = dict(schema=1,passed=True,reviewed=True,installed=True,kind='current_offline_release',
        source_sha256=sources,source_count=1,full_regression_passed=True,actual_regression='current')
    return run, release


def test_current_release_binds_current_sources_and_each_executed_phase(tmp_path):
    run, release = current_fixture(tmp_path)
    with patch.object(cold,'W',tmp_path), patch.object(cold,'read_ref',return_value=run):
        result = cold.release_evidence(release)
    assert result['current_full_regression_passed'] is True
    assert result['current_summary'] == dict(tests=1,passed=1,skipped=0,failures=0,errors=0)
    assert 'baseline_full' not in result


@pytest.mark.parametrize('phase', ['setup', 'call'])
def test_current_release_keeps_skipped_bodies_explicit_and_never_claims_full_pass(tmp_path, phase):
    run, release = current_fixture(tmp_path)
    run['case_results'][0].update({phase:'skipped', 'skip_reason':'reviewed historical bag unavailable'})
    if phase == 'setup': run['case_results'][0]['call'] = None
    run['full_regression_passed'] = release['full_regression_passed'] = False
    with patch.object(cold,'W',tmp_path), patch.object(cold,'read_ref',return_value=run):
        result = cold.release_evidence(release)
    assert result['current_full_regression_passed'] is False
    assert result['test_bodies_executed'] is False and result['current_summary']['skipped'] == 1


@pytest.mark.parametrize('fault', ['duplicate','omitted','unexpected','setup_failed','call_failed','teardown_failed',
    'missing_phase','missing_source','source_changed','skip_claimed_pass','skip_reason_missing','boundary','collection_duplicate','file_omitted',
    'uncollected_file','omitted_root'])
def test_current_release_rejects_false_or_incomplete_evidence(tmp_path, fault):
    run, release = current_fixture(tmp_path)
    case = run['case_results'][0]
    if fault == 'duplicate': run['case_results'].append(copy.deepcopy(case))
    elif fault == 'omitted': run['case_results'] = []
    elif fault == 'unexpected': case['nodeid'] += '_foreign'
    elif fault.endswith('_failed'): case[fault.split('_')[0]] = 'failed'
    elif fault == 'missing_phase': case.pop('teardown')
    elif fault == 'missing_source': run['source_sha256'] = {}
    elif fault == 'source_changed': (tmp_path/'test_current.py').write_text('changed')
    elif fault == 'skip_claimed_pass': case.update(call='skipped',skip_reason='external bag missing')
    elif fault == 'skip_reason_missing':
        case['call'] = 'skipped'; run['full_regression_passed'] = release['full_regression_passed'] = False
    elif fault == 'boundary': run['external_boundary_attempts'] = ['real socket']
    elif fault == 'collection_duplicate': run['expected_case_ids'] *= 2
    elif fault == 'file_omitted': run['test_files'] = []
    elif fault == 'uncollected_file': (tmp_path/'test_uncollected.py').write_text('def test_omitted(): pass\n')
    elif fault == 'omitted_root': run['test_roots'] = []
    with patch.object(cold,'W',tmp_path), patch.object(cold,'read_ref',return_value=run):
        with pytest.raises(RuntimeError): cold.release_evidence(release)


def storage_fixture(tmp_path, monkeypatch):
    selected = fake_storage(tmp_path, monkeypatch)
    mount, external = selected.mount, selected.base
    monkeypatch.setattr(cold, 'select_storage', lambda *args, **kwargs:selected)
    out = external/'ros_log/session'; out.mkdir(parents=True)
    (external/'grasp_records').mkdir(); (external/'maintenance').mkdir()
    archive = external/'grasp_records/archive_config.json'
    policy = external/'maintenance/storage_control.json'
    minimum = 4831838208
    archive.write_text(json.dumps(dict(upload_enabled=False,delete_enabled=False,min_free_bytes=minimum)))
    policy.write_text(json.dumps(dict(archive_policy='local_only')))
    monkeypatch.setattr(cold,'M',mount); monkeypatch.setattr(cold,'E',external)
    monkeypatch.setattr(Path,'is_mount',lambda path:path==mount)
    monkeypatch.setattr(cold.os,'statvfs',lambda path:NS(f_flag=0))
    monkeypatch.setattr(cold.shutil,'disk_usage',lambda path:NS(free=1024**3))
    return NS(mount=mount,out=out,archive=archive,policy=policy,minimum=minimum,selected=selected)


def contract_fixture(tmp_path, monkeypatch):
    disk = storage_fixture(tmp_path,monkeypatch)
    workspace = tmp_path/'workspace'; workspace.mkdir(); monkeypatch.setattr(cold,'W',workspace)
    run, release = current_fixture(workspace)
    def write_ref(name, value):
        path=tmp_path/name; path.write_text(json.dumps(value)); return cold.ref(path)
    release['actual_regression'] = write_ref('run.json',run)
    configuration = {'parameters':{'unchanged':True}}
    monkeypatch.setattr(cold,'configuration',lambda **kwargs:copy.deepcopy(configuration))
    control = {'state':'active'}
    binary = workspace/'devel/lib/alicia_d_driver/alicia_d_driver_node'
    binary.parent.mkdir(parents=True); binary.write_bytes(b'unchanged binary')
    driver_ref = write_ref('driver_build.json',dict(returncode=0,
        source_sha256=dict(release['source_sha256']),binary_sha256=cold.sha(binary)))
    release['driver_build'] = driver_ref
    contract = dict(schema=1,kind='manual_full_stack_cold_start',workspace_root=str(workspace),
        manual_only=True,automatic_grasp_authorized=False,storage=disk.selected.validate(disk.out),
        output_root=str(disk.out),current_sha256={str(workspace/'test_current.py'):cold.sha(workspace/'test_current.py'),
            driver_ref['path']:driver_ref['sha256']},
        instruction=write_ref('instruction.json',dict(source='actual_user_message',text='start in manual',
            full_restart_authorized=True,new_scene_confirmation_claimed=False)),
        control=write_ref('control.json',control),control_snapshot=control,
        release=write_ref('release.json',release),verification=cold.release_evidence(release),
        configuration=write_ref('configuration.json',configuration),pycache_prefix=str(tmp_path/'unused'),
        driver_build=driver_ref,driver_binary_sha256=cold.sha(binary))
    return NS(disk=disk,contract=contract,workspace=workspace,binary=binary,write_ref=write_ref,
              driver_ref=driver_ref,release=release)


def test_release_selects_new_driver_build_without_reading_historical_receipt(tmp_path,monkeypatch):
    fixture=contract_fixture(tmp_path,monkeypatch)
    monkeypatch.setattr(cold,'DRIVER_BUILD',tmp_path/'absent_historical_receipt.json')
    before=Path(fixture.driver_ref['path']).read_bytes()
    out,_,release=cold.validate_contract(fixture.contract,manual_return=True)
    assert out==fixture.disk.out and release['driver_build']==fixture.driver_ref
    assert Path(fixture.driver_ref['path']).read_bytes()==before


def test_historical_release_preserves_original_driver_receipt(tmp_path,monkeypatch):
    fixture=contract_fixture(tmp_path,monkeypatch)
    fixture.release.pop('driver_build')
    fixture.contract['release']=fixture.write_ref('release.json',fixture.release)
    historical=Path(fixture.driver_ref['path'])
    monkeypatch.setattr(cold,'DRIVER_BUILD',historical)
    before=historical.read_bytes()
    assert cold.validate_contract(fixture.contract,manual_return=True)[0]==fixture.disk.out
    assert historical.read_bytes()==before


@pytest.mark.parametrize('fault',['receipt_changed','failed_build','boolean_returncode','empty_sources',
    'source_changed','source_not_in_release','source_digest_not_in_release','binary_changed'])
def test_driver_build_requires_unchanged_successful_release_bound_sources_and_binary(tmp_path,monkeypatch,fault):
    fixture=contract_fixture(tmp_path,monkeypatch)
    release=copy.deepcopy(fixture.release)
    path=Path(fixture.driver_ref['path']); driver=json.loads(path.read_bytes())
    if fault=='receipt_changed':path.write_text('{}')
    elif fault=='failed_build':driver['returncode']=1
    elif fault=='boolean_returncode':driver['returncode']=False
    elif fault=='empty_sources':driver['source_sha256']={}
    elif fault=='source_changed':(fixture.workspace/'test_current.py').write_text('changed')
    elif fault=='source_not_in_release':
        extra=fixture.workspace/'unreviewed_driver.hpp'; extra.write_text('new driver implementation')
        driver['source_sha256']['unreviewed_driver.hpp']=cold.sha(extra)
    elif fault=='source_digest_not_in_release':release['source_sha256']['test_current.py']='0'*64
    elif fault=='binary_changed':fixture.binary.write_bytes(b'different driver binary')
    if fault not in ('receipt_changed','source_changed','binary_changed'):
        release['driver_build']=fixture.write_ref('rejected_driver_build.json',driver)
    with pytest.raises(RuntimeError):cold.driver_build_evidence(release)


@pytest.mark.parametrize('reference',[None,{}, {'path':'relative.json','sha256':'0'*64},
    {'path':'/unused/receipt.json','sha256':'invalid'}])
def test_explicit_invalid_driver_reference_does_not_fall_back_to_history(tmp_path,monkeypatch,reference):
    fixture=contract_fixture(tmp_path,monkeypatch)
    monkeypatch.setattr(cold,'DRIVER_BUILD',Path(fixture.driver_ref['path']))
    fixture.release['driver_build']=reference
    with pytest.raises(RuntimeError,match='invalid driver build reference'):
        cold.driver_build_evidence(fixture.release)


@pytest.mark.parametrize('fault',['contract_receipt','contract_binary','receipt_bytes_after_plan'])
def test_startup_contract_cannot_substitute_or_modify_release_driver_build(tmp_path,monkeypatch,fault):
    fixture=contract_fixture(tmp_path,monkeypatch)
    if fault=='contract_receipt':
        driver=json.loads(Path(fixture.driver_ref['path']).read_bytes())
        fixture.contract['driver_build']=fixture.write_ref('different_receipt.json',driver)
    elif fault=='contract_binary':fixture.contract['driver_binary_sha256']='0'*64
    else:Path(fixture.driver_ref['path']).write_text('{}')
    with pytest.raises(RuntimeError):cold.validate_contract(fixture.contract,manual_return=True)


def test_recording_reserve_remains_required_except_for_manual_return(tmp_path,monkeypatch):
    fixture=contract_fixture(tmp_path,monkeypatch)
    with pytest.raises(RuntimeError,match='recording minimum'):cold.validate_contract(fixture.contract)
    out,_,_=cold.validate_contract(fixture.contract,manual_return=True)
    assert out==fixture.disk.out
    with pytest.raises(RuntimeError,match='recording minimum'):cold.storage(fixture.disk.out,manual_return=False)
    monkeypatch.setattr(cold.shutil,'disk_usage',lambda path:NS(free=fixture.disk.minimum))
    assert cold.validate_contract(fixture.contract)[0]==fixture.disk.out


@pytest.mark.parametrize('fault',['unmounted','readonly','wrong_root','wrong_device','upload','delete','policy'])
def test_manual_return_storage_keeps_mount_device_and_local_policy_checks(tmp_path,monkeypatch,fault):
    fixture=storage_fixture(tmp_path,monkeypatch); out=fixture.out
    if fault=='unmounted':monkeypatch.setattr(Path,'is_mount',lambda path:False)
    elif fault=='readonly':monkeypatch.setattr(cold.os,'statvfs',lambda path:NS(f_flag=cold.os.ST_RDONLY))
    elif fault=='wrong_root':out=tmp_path
    elif fault=='wrong_device':
        original=Path.stat
        monkeypatch.setattr(Path,'stat',lambda path,*a,**kw:NS(st_dev=original(path).st_dev+1,st_mode=original(path).st_mode)
            if path==out else original(path,*a,**kw))
    elif fault in ('upload','delete'):
        data=json.loads(fixture.archive.read_text());data[fault+'_enabled']=True
        fixture.archive.write_text(json.dumps(data))
    elif fault=='policy':fixture.policy.write_text('{"archive_policy":"remote"}')
    with pytest.raises(RuntimeError):cold.storage(out,manual_return=True)


@pytest.mark.parametrize('fault',['source','configuration','driver','cache','control','instruction','release_evidence'])
def test_low_space_manual_return_still_verifies_the_complete_contract(tmp_path,monkeypatch,fault):
    fixture=contract_fixture(tmp_path,monkeypatch); contract=fixture.contract
    assert cold.validate_contract(contract,manual_return=True)[0]==fixture.disk.out
    if fault=='source':(fixture.workspace/'test_current.py').write_text('changed')
    elif fault=='configuration':monkeypatch.setattr(cold,'configuration',lambda **kwargs:{'parameters':{'unchanged':False}})
    elif fault=='driver':fixture.binary.write_bytes(b'changed')
    elif fault=='cache':Path(contract['pycache_prefix']).mkdir()
    elif fault in ('control','instruction'):Path(contract[fault]['path']).write_text('{}')
    elif fault=='release_evidence':contract['verification']['current_summary']['passed']=99
    with pytest.raises(RuntimeError):cold.validate_contract(contract,manual_return=True)


@pytest.mark.parametrize('value',[None,0,1,'manual_return'])
def test_storage_purpose_cannot_be_a_truthy_untyped_bypass(tmp_path,monkeypatch,value):
    fixture=storage_fixture(tmp_path,monkeypatch)
    with pytest.raises(RuntimeError,match='purpose'):cold.storage(fixture.out,manual_return=value)


@pytest.mark.parametrize('state',['active','paused'])
@pytest.mark.parametrize('manual_return',[False,True])
def test_explicit_manual_restart_preserves_frozen_grasp_authority(tmp_path,monkeypatch,state,manual_return):
    fixture=contract_fixture(tmp_path,monkeypatch)
    control={'state':state,'revision':70}
    fixture.contract['control']=fixture.write_ref('control.json',control)
    fixture.contract['control_snapshot']=control
    before=Path(fixture.contract['control']['path']).read_bytes()
    monkeypatch.setattr(cold.shutil,'disk_usage',lambda path:NS(free=fixture.disk.minimum))
    assert cold.validate_contract(fixture.contract,manual_return=manual_return)[0]==fixture.disk.out
    assert Path(fixture.contract['control']['path']).read_bytes()==before


@pytest.mark.parametrize('state',[None,'','stopped','complete',True,1])
def test_manual_restart_does_not_accept_unknown_control_state(tmp_path,monkeypatch,state):
    fixture=contract_fixture(tmp_path,monkeypatch)
    control={'state':state}
    fixture.contract['control']=fixture.write_ref('control.json',control)
    fixture.contract['control_snapshot']=control
    with pytest.raises(RuntimeError,match='current control'):
        cold.validate_contract(fixture.contract,manual_return=True)


@pytest.mark.parametrize('key,value',[
    ('manual_only',None),('manual_only',False),('manual_only',1),
    ('automatic_grasp_authorized',None),('automatic_grasp_authorized',True),
    ('automatic_grasp_authorized',0)])
def test_manual_restart_requires_strict_manual_scope(tmp_path,monkeypatch,key,value):
    fixture=contract_fixture(tmp_path,monkeypatch)
    fixture.contract[key]=value
    with pytest.raises(RuntimeError,match='manual-only'):
        cold.validate_contract(fixture.contract,manual_return=True)


@pytest.mark.parametrize('fault',['bytes_changed','snapshot_changed','no_actual_restart'])
def test_paused_restart_keeps_control_and_instruction_binding(tmp_path,monkeypatch,fault):
    fixture=contract_fixture(tmp_path,monkeypatch)
    control={'state':'paused','revision':70}
    fixture.contract['control']=fixture.write_ref('control.json',control)
    fixture.contract['control_snapshot']=dict(control)
    assert cold.validate_contract(fixture.contract,manual_return=True)[0]==fixture.disk.out
    if fault=='bytes_changed':
        Path(fixture.contract['control']['path']).write_text('{"state":"paused","revision":71}')
    elif fault=='snapshot_changed':
        fixture.contract['control_snapshot']['revision']=71
    else:
        fixture.contract['instruction']=fixture.write_ref('instruction.json',dict(
            source='actual_user_message',text='pause',full_restart_authorized=False,new_scene_confirmation_claimed=False))
    with pytest.raises(RuntimeError):cold.validate_contract(fixture.contract,manual_return=True)


def inherited_fixture(tmp_path, monkeypatch):
    monkeypatch.setattr(cold, 'W', tmp_path)
    old_test = tmp_path/'test_current.py'; old_test.write_text('def test_old(): pass\n')
    config = tmp_path/'config.yaml'; config.write_text('timeout: 4.0\n')
    before = {p.name:cold.sha(p) for p in (old_test, config)}
    old_cases = [dict(nodeid='test_current.py::test_old', setup='passed', call='passed', teardown='passed'),
        dict(nodeid='test_current.py::test_external', setup='skipped', call=None, teardown='passed', skip_reason='external fixture absent')]
    def run(sources, files, declared, cases, complete):
        skips = [c['nodeid'] for c in cases if c['setup'] == 'skipped' or c['call'] == 'skipped']
        return dict(schema=1, passed=True, source_unchanged=True, source_sha256=sources,
            changed_sources=[], external_boundary_attempts=[], bad_files=[], duplicate_cases=[], collection_skips=[],
            complete_scope_requested=complete, offline_scope_complete=complete,
            test_files=files, declared_test_files=declared, expected_case_ids=[c['nodeid'] for c in cases],
            case_results=cases, skipped_cases=skips, full_regression_passed=complete and not skips,
            summary=dict(tests=len(cases), passed=len(cases)-len(skips), skipped=len(skips), failed=0, collection_skipped_files=0),
            excluded_files=[], test_roots=['.'], explicit_test_files=[], scope_name='declared offline scope')
    baseline = run(before, ['test_current.py'], ['test_current.py'], old_cases, True)
    config.write_text('timeout: 2.0\n')
    new_test = tmp_path/'test_new.py'; new_test.write_text('def test_timing(): pass\n')
    sources = {p.name:cold.sha(p) for p in (old_test, config, new_test)}
    target = run(sources, ['test_new.py'], ['test_current.py', 'test_new.py'],
        [dict(nodeid='test_new.py::test_timing', setup='passed', call='passed', teardown='passed')], False)
    delta = {p:dict(before_sha256=before.get(p), after_sha256=sources[p])
             for p in sorted(sources) if before.get(p) != sources[p]}
    release = dict(schema=1, kind='inherited_full_plus_targeted', passed=True, reviewed=True, installed=True,
        full_regression_passed=False, source_count=len(sources), source_sha256=sources,
        baseline_full_regression='baseline', actual_targeted_regression='target', source_delta=delta,
        targeted_test_files=['test_new.py'], evidence_review='review')
    review = {k:copy.deepcopy(release[k]) for k in ('baseline_full_regression', 'actual_targeted_regression',
                                                   'source_delta', 'targeted_test_files')}
    review.update(passed=True, actual_import_inventory_verified=True,
                  retained_baseline_cases_verified=True, original_skips_preserved=True)
    monkeypatch.setattr(cold, 'read_ref', lambda key:{'baseline':baseline, 'target':target, 'review':review}[key])
    return NS(baseline=baseline, target=target, release=release, review=review)


def test_inherited_release_keeps_historical_skips_and_actual_target_cases_separate(tmp_path, monkeypatch):
    fixture = inherited_fixture(tmp_path, monkeypatch)
    baseline_before = copy.deepcopy(fixture.baseline)
    result = cold.release_evidence(fixture.release)
    assert result['baseline']['summary'] == dict(tests=2, passed=1, skipped=1, failed=0, collection_skipped_files=0)
    assert result['baseline']['skipped_cases'][0]['reason'] == 'external fixture absent'
    assert result['targeted_summary']['tests'] == result['test_bodies_executed_in_current_run'] == 1
    assert result['current_full_regression_passed'] is False
    assert fixture.baseline == baseline_before


def explicit_inheritance_fixture(tmp_path, monkeypatch):
    f = inherited_fixture(tmp_path, monkeypatch)
    names = {p:'tools/tests/'+p for p in ('test_current.py', 'test_new.py')}
    for old, new in names.items():
        path = tmp_path/new; path.parent.mkdir(parents=True, exist_ok=True)
        (tmp_path/old).rename(path)
    for run in (f.baseline, f.target):
        run['source_sha256'] = {names.get(p,p):h for p,h in run['source_sha256'].items()}
        for key in ('test_files', 'declared_test_files'):
            run[key] = [names.get(p,p) for p in run[key]]
        for case in run['case_results']:
            path, identifier = case['nodeid'].split('::',1)
            case['nodeid'] = names.get(path,path)+'::'+identifier
        run['expected_case_ids'] = [case['nodeid'] for case in run['case_results']]
        run['skipped_cases'] = [case['nodeid'] for case in run['case_results']
                                if case['setup'] == 'skipped' or case['call'] == 'skipped']
        run['test_roots'] = ['src','tools']
    extra = [cold.HISTORICAL_PREWARM_TEST, '.runtime/old_binding/tests/test_historical.py']
    for name in extra:
        path = tmp_path/name; path.parent.mkdir(parents=True, exist_ok=True); path.write_text('def test_history(): pass\n')
        for run in (f.baseline, f.target):
            run['source_sha256'][name] = cold.sha(path)
            run['declared_test_files'].append(name)
        f.baseline['test_files'].append(name)
        f.baseline['case_results'].append(dict(nodeid=name+'::test_history', setup='passed', call='passed', teardown='passed'))
    f.baseline['expected_case_ids'] = [case['nodeid'] for case in f.baseline['case_results']]
    f.baseline['summary'].update(tests=4, passed=3)
    f.baseline['explicit_test_files'] = extra
    # The real targeted wrapper keeps its default prewarm declaration but does
    # not list unselected historical runtime files here. Its declared scope is
    # still complete; neither extra file was executed in this targeted run.
    f.target['explicit_test_files'] = [extra[0]]
    f.release.update(source_sha256=f.target['source_sha256'], source_count=len(f.target['source_sha256']),
        source_delta={p:dict(before_sha256=f.baseline['source_sha256'].get(p), after_sha256=h)
            for p,h in f.target['source_sha256'].items() if f.baseline['source_sha256'].get(p) != h},
        targeted_test_files=list(f.target['test_files']))
    for key in ('source_delta','targeted_test_files'): f.review[key] = copy.deepcopy(f.release[key])
    return f, extra


def test_targeted_scope_inherits_unselected_baseline_explicit_declarations_without_rewriting_records(tmp_path, monkeypatch):
    f, extra = explicit_inheritance_fixture(tmp_path, monkeypatch)
    before = copy.deepcopy((f.baseline, f.target))
    evidence = cold.release_evidence(f.release)
    assert (f.baseline, f.target) == before
    assert evidence['targeted_summary']['tests'] == 1
    assert evidence['targeted_test_files'] == ['tools/tests/test_new.py']
    assert evidence['targeted_declaration'] == dict(declared_test_file_count=4,
        explicit_test_files=[extra[0]], inherited_explicit_test_files=extra)
    assert evidence['baseline']['test_file_count'] == 3
    assert evidence['current_full_regression_passed'] is False


@pytest.mark.parametrize('fault', ['omitted_history', 'unknown_explicit', 'invented_declared',
                                  'selected_extra_not_explicit', 'invalid_explicit', 'complete_inherits'])
def test_inherited_explicit_scope_cannot_hide_omissions_or_admit_unknown_extra_files(tmp_path, monkeypatch, fault):
    f, extra = explicit_inheritance_fixture(tmp_path, monkeypatch)
    if fault == 'omitted_history': f.target['declared_test_files'].remove(extra[1])
    elif fault in ('unknown_explicit', 'invented_declared'):
        path = tmp_path/'.runtime/unknown/test_extra.py'; path.parent.mkdir(parents=True); path.write_text('def test_extra(): pass\n')
        name = str(path.relative_to(tmp_path))
        f.target['source_sha256'][name] = cold.sha(path)
        f.release['source_count'] += 1
        f.target['declared_test_files'].append(name)
        if fault == 'unknown_explicit': f.target['explicit_test_files'].append(name)
    elif fault == 'selected_extra_not_explicit':
        f.target['test_files'].append(extra[1])
        f.target['expected_case_ids'].append(extra[1]+'::test_history')
        f.target['case_results'].append(dict(nodeid=extra[1]+'::test_history', setup='passed', call='passed', teardown='passed'))
        f.target['summary'].update(tests=2,passed=2)
    elif fault == 'invalid_explicit': f.target['explicit_test_files'] = None
    elif fault == 'complete_inherits':
        with pytest.raises(RuntimeError, match='inherited explicit'):
            cold.offline_regression_evidence(f.baseline, complete=True, inherited_explicit_files=extra)
        return
    with pytest.raises(RuntimeError): cold.release_evidence(f.release)


@pytest.mark.parametrize('fault', ['baseline_incomplete', 'baseline_skip_relabelled', 'baseline_skip_reason',
    'baseline_file_omitted', 'target_full_claim', 'target_duplicate', 'target_missing_case', 'target_failed_phase',
    'target_skipped', 'target_boundary', 'target_source_changed', 'target_scope_invented', 'target_summary_invented',
    'source_delta_omitted', 'source_delta_before_changed', 'source_removed', 'changed_test_omitted',
    'review_missing_imports', 'review_wrong_target', 'review_missing_history', 'installed_bytes_changed'])
def test_inherited_release_rejects_unproven_delta_or_relabelled_history(tmp_path, monkeypatch, fault):
    f = inherited_fixture(tmp_path, monkeypatch)
    if fault == 'baseline_incomplete': f.baseline['offline_scope_complete'] = False
    elif fault == 'baseline_skip_relabelled': f.baseline['full_regression_passed'] = True
    elif fault == 'baseline_skip_reason': f.baseline['case_results'][1].pop('skip_reason')
    elif fault == 'baseline_file_omitted': f.baseline['declared_test_files'].append('not_executed.py')
    elif fault == 'target_full_claim': f.release['full_regression_passed'] = True
    elif fault == 'target_duplicate': f.target['case_results'] *= 2
    elif fault == 'target_missing_case': f.target['case_results'] = []
    elif fault == 'target_failed_phase': f.target['case_results'][0]['teardown'] = 'failed'
    elif fault == 'target_skipped':
        f.target['case_results'][0].update(call='skipped', skip_reason='not executed')
        f.target['skipped_cases'] = list(f.target['expected_case_ids'])
        f.target['summary'].update(passed=0, skipped=1)
    elif fault == 'target_boundary': f.target['external_boundary_attempts'] = ['external socket']
    elif fault == 'target_source_changed': f.target['source_sha256'] = dict(f.target['source_sha256'], **{'config.yaml':'0'*64})
    elif fault == 'target_scope_invented': f.target['declared_test_files'].append('test_omitted.py')
    elif fault == 'target_summary_invented': f.target['summary']['passed'] += 1
    elif fault == 'source_delta_omitted': f.release['source_delta'].pop('config.yaml')
    elif fault == 'source_delta_before_changed': f.release['source_delta']['config.yaml']['before_sha256'] = '0'*64
    elif fault == 'source_removed':
        f.release['source_sha256'].pop('config.yaml'); f.release['source_count'] -= 1
    elif fault == 'changed_test_omitted':
        p = tmp_path/'test_current.py'; p.write_text('def different(): pass\n')
        f.release['source_sha256'][p.name] = cold.sha(p)
        f.release['source_delta'][p.name] = dict(before_sha256=f.baseline['source_sha256'][p.name], after_sha256=cold.sha(p))
    elif fault == 'review_missing_imports': f.review['actual_import_inventory_verified'] = False
    elif fault == 'review_wrong_target': f.review['actual_targeted_regression'] = 'foreign'
    elif fault == 'review_missing_history': f.review['retained_baseline_cases_verified'] = False
    elif fault == 'installed_bytes_changed': (tmp_path/'config.yaml').write_text('timeout: 9.0\n')
    with pytest.raises(RuntimeError): cold.release_evidence(f.release)


def manual_plan_fixture(tmp_path, monkeypatch):
    f = contract_fixture(tmp_path, monkeypatch)
    monkeypatch.setattr(cold.shutil, 'disk_usage', lambda path:NS(free=f.disk.minimum))
    f.release['generation'] = 'test_manual_restart'
    release_ref = f.write_ref('manual_release.json', f.release)
    for name in ('ENTRY', 'LAUNCH', 'REMOTE', 'SIM', 'DESKTOP', 'CLIENT', 'HAND_EYE'):
        p = f.workspace/(name.lower()+'.input'); p.write_text('frozen input\n')
        monkeypatch.setattr(cold, name, p)
    for name in ('tools/grasp_attempt_boundary.py', 'tools/with_local_realsense_sdk.sh',
                 'tools/experiment_storage.py', '.runtime/candidate_simulation/run_candidate.py'):
        p = f.workspace/name; p.parent.mkdir(parents=True, exist_ok=True); p.write_text('frozen entry\n')
    cfg = dict(nodes={}, parameters={}, launch_source_sha256={})
    monkeypatch.setattr(cold, 'configuration', lambda **kwargs:copy.deepcopy(cfg))
    backend = f.write_ref('backend.json', dict(allowed_backends=['graspnet_baseline']))
    args = NS(session=f.disk.out.parent/'new_manual', storage_backend='ssd',
        release=Path(release_ref['path']), release_sha256=release_ref['sha256'],
        request=Path(f.contract['instruction']['path']), request_sha256=f.contract['instruction']['sha256'],
        backend_binding=Path(backend['path']), control=Path(f.contract['control']['path']))
    return f, args


def test_default_plan_and_start_skip_cold_checks_but_keep_real_contract_validation(tmp_path, monkeypatch):
    f, args = manual_plan_fixture(tmp_path, monkeypatch)
    def forbidden(*args, **kwargs):
        pytest.fail('manual restart must not call cold scan or prestart remote health')
    monkeypatch.setattr(cold, 'cold_scan', forbidden)
    monkeypatch.setattr(cold, 'prestart_health', forbidden)
    commands = []
    monkeypatch.setattr(cold.subprocess, 'run', lambda command, **kwargs:
        (commands.append(command) or NS(returncode=0, stdout='', stderr='')))
    monkeypatch.setattr(cold, 'load', lambda path:NS(wait_ready=lambda **kw:None, service_snapshot=lambda:{'running':True}))
    with patch.dict(cold.os.environ):
        cold.plan(args)
        contract_ref = cold.ref(args.session/'contract.json'); contract = json.loads(Path(contract_ref['path']).read_text())
        assert contract['kind'] == 'manual_full_stack_restart'
        assert contract['cold_check_executed'] is False and 'cold_boot_id' not in contract
        assert contract['remote_health_check_stage'] == 'post_start_acceptance'
        cold.start(contract, contract_ref)
    startup = json.loads((args.session/'startup.json').read_text())
    assert startup['dispatched'] is True and startup['manual_only'] is True
    assert startup['cold_check_executed'] is False and startup['automatic_grasp_started'] is False
    assert len(commands) == 2 and all('--property=Restart=no' in row for row in commands)
    assert '--setenv=ROS_LOG_DIR='+str(args.session/'core_ros') in commands[1]
    assert 'cold' not in startup and 'dispatch_remote_health' not in startup
    f.binary.write_bytes(b'changed driver')
    with pytest.raises(RuntimeError, match='driver binary'):
        cold.validate_contract(contract)


@pytest.mark.parametrize('fault', ['workflow_flag', 'health_stage', 'boot_changed', 'source_changed'])
def test_new_manual_workflow_keeps_binding_checks_before_dispatch(tmp_path, monkeypatch, fault):
    f, args = manual_plan_fixture(tmp_path, monkeypatch)
    with patch.dict(cold.os.environ):
        cold.plan(args)
        ref = cold.ref(args.session/'contract.json'); contract = json.loads(Path(ref['path']).read_text())
        if fault == 'workflow_flag': contract['cold_check_executed'] = True
        elif fault == 'health_stage': contract['remote_health_check_stage'] = 'unverified'
        elif fault == 'boot_changed': contract['boot_id'] = 'different-boot'
        else: cold.ENTRY.write_text('changed entry')
        monkeypatch.setattr(cold.subprocess, 'run', lambda *a, **kw:pytest.fail('invalid contract dispatched'))
        with pytest.raises(RuntimeError): cold.start(contract, ref)


@pytest.mark.parametrize('fault', [None, 'timeout', 'duration', 'manual_type'])
def test_live_policy_records_and_checks_loaded_controller_timing(fault):
    values = {key:False for key in cold.LIVE_STARTUP_POLICY_KEYS}
    values['/robot/strict_execution_controller_sync_timeout_sec'] = 2.0
    values['/robot/strict_execution_controller_sync_duration_sec'] = .30
    config = dict(parameters=copy.deepcopy(values))
    if fault in ('timeout', 'duration'):
        values['/robot/strict_execution_controller_sync_'+fault+'_sec'] = 4.0
    elif fault == 'manual_type': values['/gui/joint_direct_mode'] = 0
    if fault is None:
        observed = cold.live_startup_policy(config, values.__getitem__)
        assert observed == values
        assert observed['/robot/strict_execution_controller_sync_timeout_sec'] == 2.0
    else:
        with pytest.raises(RuntimeError, match='live startup policy differs'):
            cold.live_startup_policy(config, values.__getitem__)
