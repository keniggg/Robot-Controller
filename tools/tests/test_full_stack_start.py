"""Offline cold-start boundaries; never imports ROS or starts any service."""
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
