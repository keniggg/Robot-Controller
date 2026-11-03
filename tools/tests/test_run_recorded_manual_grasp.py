"""Current-instance attempt/ending contracts; all ROS/runner calls are fakes."""
from copy import deepcopy
import importlib.util
import hashlib
import json
import os
from pathlib import Path
import py_compile
import threading
from types import SimpleNamespace as NS

import pytest

from test_finish_returned_grasp import idle_fixture, finish, snapshot, Stamp
from test_manual_return_boundary import boundary

path = Path(__file__).resolve().parents[1]/'run_recorded_manual_grasp.py'
spec = importlib.util.spec_from_file_location('recorded_current_test',path)
entry = importlib.util.module_from_spec(spec); spec.loader.exec_module(entry)


def instruction():
    return dict(source='actual_user_message',text='I have aligned the object; continue once.',
        automatic_grasp_authorized=True,user_alignment_ready=True,
        new_scene_confirmation_required=False,new_scene_confirmation_claimed=False)


def actuation_policy():
    return {'/grasp/require_actuation_confirmation': True,
        '/grasp/auto_confirm_actuation_from_observation': True,
        '/grasp/use_grasp6d_plan': True, '/grasp/actuation_confirmation_freshness_sec': 2.0}


def frozen_startup_policy_projection():
    # Exact selected parameters from manual_full_stack_r1/startup_configuration.json,
    # SHA256 5d70f7383cf6c920e626d6166773191d371cbbf8b823527b4fcdb9901b9e6fe3.
    # The source file and extraction receipt are retained on SanStand. Keeping
    # its projection here avoids a future test dependency on archived logs.
    return {
        '/alicia_d_driver_node/software_temperature_monitoring_enabled': False,
        '/alicia_d_driver_node/temperature_poll_rate_hz': 0.0,
        '/alicia_d_driver_node/self_check_poll_rate_hz': 0.0,
        '/camera/simulate': False,
        '/camera/fallback_to_simulation': False,
        '/grasp/use_grasp6d_plan': True,
        '/grasp/unknown_lift_after_close_enabled': False,
        '/grasp/require_actuation_confirmation': True,
        '/grasp/auto_confirm_actuation_from_observation': True,
        '/grasp/actuation_confirmation_freshness_sec': 2.0,
        '/grasp/unknown_observe_return_max_observations': 1,
        '/camera/mode_depth_preset/enabled': True,
        '/camera/mode_depth_preset/unknown_preset': 'high_accuracy',
        '/camera/mode_depth_preset/stationary_temporal_enabled': True,
        '/camera/mode_depth_preset/regional_stationary_enabled': True,
        '/camera/mode_depth_preset/baseline_path':
            '/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade/.runtime/d405_carton_depth_baseline.json',
    }


def frozen_policy_runtime_view():
    values={k:v for k,v in frozen_startup_policy_projection().items()
            if not k.startswith('/camera/mode_depth_preset/')}
    values['/camera/mode_depth_preset'] = dict(enabled=True, unknown_preset='high_accuracy',
        stationary_temporal_enabled=True, regional_stationary_enabled=True,
        baseline_path='/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade/.runtime/d405_carton_depth_baseline.json')
    return values


@pytest.mark.parametrize('fault', ['startup_only','alignment_missing','fictional_scene','no_text','not_user'])
def test_manual_startup_cannot_supply_attempt_authority(fault):
    value = instruction()
    if fault == 'startup_only': value['automatic_grasp_authorized'] = False
    elif fault == 'alignment_missing': value.pop('user_alignment_ready')
    elif fault == 'fictional_scene': value['new_scene_confirmation_claimed'] = True
    elif fault == 'no_text': value['text'] = ''
    elif fault == 'not_user': value['source'] = 'inferred'
    with pytest.raises(RuntimeError,match='post-alignment'): entry.validate_instruction(value)


def test_actual_aligned_user_instruction_does_not_claim_a_new_scene_capture():
    value = instruction(); prior = deepcopy(value)
    entry.validate_instruction(value)
    assert value == prior


@pytest.mark.parametrize('fault', ['failed','historical_remote','master_missing','unit_missing','source_unbound'])
def test_attempt_requires_current_complete_startup_acceptance(fault):
    value = dict(passed=True,manual_only=True,all_configured_nodes_started=True,
        source_cache_process_binding=True,automatic_grasp_started=False,nodes={'driver':{}},
        master_identity={'pid':100,'start_ticks':'200'},core_unit='new.service',remote_startup_binding={'sha256':'new'})
    if fault == 'failed': value['passed'] = False
    elif fault == 'historical_remote': value.pop('remote_startup_binding')
    elif fault == 'master_missing': value.pop('master_identity')
    elif fault == 'unit_missing': value.pop('core_unit')
    elif fault == 'source_unbound': value['source_cache_process_binding'] = False
    with pytest.raises(RuntimeError): entry.validate_acceptance(value)


class Lifecycle:
    def __init__(self, code=1, fail=None, cleanup_error=False):
        self.report, self.calls = {}, []
        self.code, self.fail, self.cleanup_error = code, fail, cleanup_error
    def step(self, name):
        self.calls.append(name)
        if name == self.fail: raise RuntimeError('original '+name)
    def preflight(self): self.step('preflight')
    def enter_automatic(self): self.step('automatic')
    def select(self): self.step('select'); return {'generation':2}
    def run_selected(self, selection): self.step('run'); return self.code
    def return_manual(self):
        self.step('manual')
        if self.cleanup_error: raise RuntimeError('cleanup failed')
        return dict(automatic_motion_authorized=False,actuation_status='UNCONFIRMED:DISCONTINUOUS_FEEDBACK_RECOVERY')
    def save(self): self.calls.append('save')


def test_failed_task_keeps_failure_while_returning_unconfirmed_to_manual():
    adapter = Lifecycle(code=1)
    assert entry.attempt_lifecycle(adapter) == 1
    assert adapter.report['runner_returncode'] == 1 and adapter.report['manual_return_verified'] is True
    assert adapter.calls == ['preflight','automatic','select','run','manual','save']


def test_failed_cleanup_cannot_report_a_successful_launcher():
    adapter = Lifecycle(code=0,cleanup_error=True)
    assert entry.attempt_lifecycle(adapter) == 1
    assert adapter.report['runner_returncode'] == 0 and adapter.report['manual_return_verified'] is False


@pytest.mark.parametrize('failure', ['preflight','automatic','select','run'])
def test_primary_exception_survives_failed_cleanup_and_receipt_is_written(failure):
    adapter = Lifecycle(fail=failure,cleanup_error=True)
    with pytest.raises(RuntimeError,match='original '+failure): entry.attempt_lifecycle(adapter)
    assert adapter.calls[-2:] == ['manual','save']
    assert 'original '+failure in adapter.report['launcher_error']
    assert 'cleanup failed' in adapter.report['manual_return_error']
    if failure == 'preflight': assert 'automatic' not in adapter.calls


def observed_session():
    validator,samples,states,now,epoch = idle_fixture()
    session = entry.Session.__new__(entry.Session)
    session.ctx = NS(idle=validator,boundary=boundary,recheck=lambda *args,**kwargs:None,
        acceptance=dict(nodes={'driver':{'pid':100}},state={'epoch_ns':epoch}))
    session.lock,session.states,session.samples = threading.RLock(),states,samples
    session.topics = [(key,'/'+key,None) for key in states]
    selection = dict(mode='unknown',strategy='observe_return',generation=1,stamp_ns=1)
    session.rospy = NS(Time=NS(now=lambda:Stamp(now)),is_shutdown=lambda:False,get_name=lambda:'/observer',
        wait_for_message=lambda topic,*args,**kwargs:states[topic[1:]][0],
        get_param=lambda key:states['manual'][0].data if key=='/gui/joint_direct_mode' else selection)
    session.master = NS(getSystemState=lambda:([('/'+key,[owner]) for key,owner in validator.OWNERS.items()],[],[]))
    session.policy = lambda **kwargs:actuation_policy()
    session.publisher = None;session.attempt_epoch=None;session.finish_epoch=None;session.selected=None
    return session,now,epoch


def test_real_observer_uses_distinct_automatic_and_manual_admission():
    session,_,_ = observed_session()
    session.states['actuation'][0].data = 'UNCONFIRMED:DISCONTINUOUS_FEEDBACK_RECOVERY'
    session.states['enabled'][0].data = False
    with pytest.raises(RuntimeError,match='boundary unavailable'): session.observe(timeout_sec=.001)
    result = session.observe(manual_return=True)
    assert result['idle']['motion_enabled'] is False
    assert result['idle']['automatic_motion_authorized'] is False
    assert result['idle']['actuation_status'] == 'UNCONFIRMED:DISCONTINUOUS_FEEDBACK_RECOVERY'


def test_real_observer_rejects_changed_publisher_and_current_epoch():
    session,_,epoch = observed_session()
    session.observe(manual_return=True)
    session.states['epoch'][0].stamp = Stamp(epoch+1)
    with pytest.raises(RuntimeError,match='reference epoch changed'):
        session.observe(manual_return=True,timeout_sec=.001)
    session.states['epoch'][0].stamp = Stamp(epoch)
    session.master = NS(getSystemState=lambda:([('/'+key,[owner,'/foreign']) for key,owner in session.ctx.idle.OWNERS.items()],[],[]))
    with pytest.raises(RuntimeError,match='publisher ownership'):
        session.observe(manual_return=True,timeout_sec=.001)


@pytest.mark.parametrize('fault',['runner_live','response_unknown'])
def test_ambiguous_execution_is_not_overridden_by_manual_cleanup(tmp_path,fault):
    session = entry.Session.__new__(entry.Session)
    session.record=tmp_path;session.attempt_epoch=1;session.finish_epoch=None
    session.prepared=NS(close=lambda:None,released=True,process=NS(poll=lambda:None if fault=='runner_live' else 1))
    session.rospy=NS(get_name=lambda:'/observer');session.master=object()
    session.ctx=NS(finish=finish,recheck=lambda *args,**kwargs:None)
    if fault=='response_unknown':(tmp_path/'runner.log').write_text('RUNNER_EXCEPTION execution_response_unknown=True\n')
    session.observe=lambda **kwargs:pytest.fail('must not observe/restore while execution ownership is ambiguous')
    with pytest.raises(RuntimeError):session.return_manual()


def test_in_process_cleanup_reuses_checked_ending_and_requests_manual_once(tmp_path):
    session = entry.Session.__new__(entry.Session)
    session.record=tmp_path;session.attempt_epoch=1;session.finish_epoch=1
    session.prepared=NS(close=lambda:None,released=True,process=NS(poll=lambda:1))
    session.rospy=NS(get_name=lambda:'/observer');session.master=object()
    session.ctx=NS(finish=finish,recheck=lambda *args,**kwargs:None)
    values=iter([snapshot(False),snapshot(False),snapshot(True)])
    session.observe=lambda **kwargs:next(values)
    calls=[];session.prepare_publisher=lambda:calls.append('prepare')
    session.request_mode=lambda value,observed:calls.append(value) or 100
    session.wait_ack=lambda value,requested:calls.append(('ack',value,requested))
    result=session.return_manual()
    assert result['outcome']=='restored_manual' and result['automatic_motion_authorized'] is False
    assert calls==['prepare',True,('ack',True,100)]
    assert result['recorder_sealed'] is False


@pytest.mark.parametrize('fault',['none','wrong_owner','unlatched','wrong_generation','early_status','reset_error'])
def test_selection_release_requires_router_owned_completed_transaction(fault):
    prior=dict(mode='unknown',strategy='observe_return',generation=7,stamp_ns=100)
    selected=dict(prior,generation=8,stamp_ns=200)
    if fault=='wrong_generation':selected['generation']=9
    header=dict(callerid='/grasp_mode_router',latching='1')
    if fault=='wrong_owner':header['callerid']='/foreign'
    if fault=='unlatched':header['latching']='0'
    status=dict(selected,state='waiting_for_target')
    if fault=='early_status':status['state']='switching'
    if fault=='reset_error':status.update(state='error',detail='reset failed')
    closed=[]
    def subscribe(topic,kind,callback,**kwargs):
        value=status if topic.endswith('/status') else selected
        callback(NS(data=json.dumps(value),_connection_header=header))
        return NS(unregister=lambda:closed.append(topic))
    rospy=NS(Subscriber=subscribe,is_shutdown=lambda:False,get_param=lambda key:dict(selected))
    if fault=='none':
        actual,ack=entry.wait_selection_commit(rospy,prior,timeout_sec=.1)
        assert actual==selected and ack['state']=='waiting_for_target'
    else:
        with pytest.raises(RuntimeError):entry.wait_selection_commit(rospy,prior,timeout_sec=.001)
    assert sorted(closed)==['/grasp_mode/selection','/grasp_mode/status']


def test_actual_non_ros_helpers_load_from_checked_source_without_legacy_sibling_imports():
    sources={name:hashlib.sha256((entry.W/name).read_bytes()).hexdigest() for name in entry.REQUIRED_SOURCES}
    for name in entry.REQUIRED_SOURCES:
        if name.endswith('mode_aware_grasp_runner.py'):continue  # Original runner imports ROS in its isolated prewarm child.
        if name in (entry.BOOTSTRAP_SOURCE, entry.BOOTSTRAP_GUARD_SOURCE):
            continue  # Existing package modules require their actual relative-import context.
        loaded=entry.load_source(name,sources)
        assert Path(loaded.__file__)==entry.W/name
    from alicia_flexible_grasp.robot import actuation_bootstrap
    assert Path(actuation_bootstrap.__file__).resolve() == entry.W/entry.BOOTSTRAP_SOURCE
    assert actuation_bootstrap.PENDING_BOOTSTRAP_STATES == frozenset((
        'PENDING:POSITIVE_ENABLE_REQUESTED', 'PENDING:COMMAND_SYNCHRONIZED'))


def test_current_interpreter_reads_source_instead_of_timestamp_matching_legacy_cache(tmp_path,monkeypatch):
    monkeypatch.setattr(entry.sys,'pycache_prefix',None)
    for key in ('ROS_LOG_DIR','PYTHONPYCACHEPREFIX','PYTHONDONTWRITEBYTECODE'):
        monkeypatch.setenv(key,'test_previous_value')
    source=tmp_path/'cached_attempt_dependency.py'
    source.write_text("VALUE = 'old'\n")
    stamp=source.stat().st_mtime_ns
    py_compile.compile(str(source),doraise=True)
    source.write_text("VALUE = 'new'\n")
    os.utime(source,ns=(stamp,stamp))
    def read_value():
        spec=importlib.util.spec_from_file_location('cached_attempt_dependency',source)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        return module.VALUE
    assert read_value()=='old'  # An existing valid-by-timestamp pyc hides the edit.
    entry.configure_attempt_environment(tmp_path)
    assert read_value()=='new'
    assert entry.sys.pycache_prefix==os.environ['PYTHONPYCACHEPREFIX']
    assert not (tmp_path/'unused_python_cache').exists()


@pytest.mark.parametrize('status', ['CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE',
    'PENDING:POSITIVE_ENABLE_REQUESTED', 'PENDING:COMMAND_SYNCHRONIZED'])
def test_current_entry_distinguishes_confirmation_from_bound_bootstrap_without_mutation(status):
    validator, samples, states, now, epoch = idle_fixture()
    states['actuation'][0].data = status
    states['enabled'][0].data = status.startswith('CONFIRMED:')
    original = json.dumps((samples, states), default=vars, sort_keys=True)
    evidence = entry.validate_automatic_entry_snapshot(samples, states, now, epoch,
        validator.OWNERS, actuation_policy(), boundary)
    confirmed = status.startswith('CONFIRMED:')
    assert evidence['actuation_confirmed'] is confirmed
    assert evidence['automatic_actuation_gate_passed'] is confirmed
    assert evidence['pending_observation_bootstrap_eligible'] is not confirmed
    assert evidence['confirmation_required_before_full_task_motion'] is not confirmed
    assert evidence['automatic_motion_authorized'] is False
    assert evidence['actuation_status'] == status and evidence['motion_enabled'] is confirmed
    assert json.dumps((samples, states), default=vars, sort_keys=True) == original


@pytest.mark.parametrize('status,enabled', [
    ('UNCONFIRMED:DISCONTINUOUS_FEEDBACK_RECOVERY', False),
    ('UNCONFIRMED:ENCODER_RESPONSE_TIMEOUT', False),
    ('DISABLED:MOTION_NOT_REQUESTED', False), ('OVERHEAT_BLOCKED:BLOCKED', False),
    ('PENDING:AWAITING_ENCODER_RESPONSE', False), ('PENDING:', False),
    ('PENDING:POSITIVE_ENABLE_REQUESTED', True),
    ('CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE', False)])
def test_automatic_entry_never_uses_manual_return_state_allowance_as_motion_authority(status, enabled):
    validator, samples, states, now, epoch = idle_fixture()
    states['actuation'][0].data, states['enabled'][0].data = status, enabled
    assert boundary.validate_manual_return_snapshot(samples, states, now, epoch,
        validator.OWNERS)['automatic_motion_authorized'] is False
    with pytest.raises(RuntimeError):
        entry.validate_automatic_entry_snapshot(samples, states, now, epoch,
            validator.OWNERS, actuation_policy(), boundary)


@pytest.mark.parametrize('fault', ['ready_false', 'task_active', 'arm_active', 'gripper_active',
    'router_pending', 'router_missing', 'gui_owner', 'gui_unlatched', 'driver_owner', 'wrong_frame',
    'epoch', 'short_window', 'repeated_stamp', 'moving', 'nan_joint', 'sdk_expired',
    'sdk_receive_late', 'status_expired', 'status_future', 'action_expired',
    'bootstrap_disabled', 'rich_plan_disabled', 'confirmation_disabled', 'string_boolean'])
def test_pending_bootstrap_retains_every_common_idle_gate_and_bound_policy(fault):
    validator, samples, states, now, epoch = idle_fixture()
    states['actuation'][0].data = 'PENDING:POSITIVE_ENABLE_REQUESTED'
    states['enabled'][0].data = False
    policy = actuation_policy()
    if fault == 'ready_false': states['ready'][0].data = False
    elif fault == 'task_active': states['task'][0].active = True
    elif fault == 'arm_active': states['arm'][0].status_list = [NS(status=1)]
    elif fault == 'gripper_active': states['hand'][0].status_list = [NS(status=7)]
    elif fault == 'router_pending': states['mode'][0].data = json.dumps(dict(active=False,pending='unknown',start_reserved=False))
    elif fault == 'router_missing': states['mode'][0].data = '{}'
    elif fault == 'gui_owner': states['manual'][0]._connection_header['callerid'] = '/foreign'
    elif fault == 'gui_unlatched': states['manual'][0]._connection_header['latching'] = '0'
    elif fault == 'driver_owner': samples[-1]['caller'] = '/foreign'
    elif fault == 'wrong_frame': samples[-1]['frame_id'] = 'commanded'
    elif fault == 'epoch': epoch += 1
    elif fault == 'short_window': samples.pop()
    elif fault == 'repeated_stamp': samples[-1]['stamp_ns'] = samples[-2]['stamp_ns']
    elif fault == 'moving': samples[-1]['positions'][0] += .005
    elif fault == 'nan_joint': samples[-1]['positions'][0] = float('nan')
    elif fault == 'sdk_expired': now += 150_000_000  # Exactly 200 ms since the latest SDK sample.
    elif fault == 'sdk_receive_late': samples[-1]['received_ns'] = samples[-1]['stamp_ns']+200_000_000
    elif fault == 'status_expired': states['actuation'] = (states['actuation'][0], now-1_000_000_000)
    elif fault == 'status_future': states['actuation'] = (states['actuation'][0], now+1)
    elif fault == 'action_expired': states['arm'][0].header.stamp = Stamp(now-2_000_000_001)
    elif fault == 'bootstrap_disabled': policy['/grasp/auto_confirm_actuation_from_observation'] = False
    elif fault == 'rich_plan_disabled': policy['/grasp/use_grasp6d_plan'] = False
    elif fault == 'confirmation_disabled': policy['/grasp/require_actuation_confirmation'] = False
    elif fault == 'string_boolean': policy['/grasp/auto_confirm_actuation_from_observation'] = 'true'
    with pytest.raises((RuntimeError, ValueError)):
        entry.validate_automatic_entry_snapshot(samples, states, now, epoch, validator.OWNERS, policy, boundary)


def test_confirmed_entry_honors_bound_freshness_stricter_than_common_receive_gate():
    validator, samples, states, now, epoch = idle_fixture()
    policy = actuation_policy(); policy['/grasp/actuation_confirmation_freshness_sec'] = .005
    with pytest.raises(RuntimeError, match='confirmation expired'):
        entry.validate_automatic_entry_snapshot(samples, states, now, epoch, validator.OWNERS, policy, boundary)


def test_real_pending_observer_keeps_fixed_epoch_and_publisher_ownership():
    session, _, epoch = observed_session()
    session.states['actuation'][0].data = 'PENDING:POSITIVE_ENABLE_REQUESTED'
    session.states['enabled'][0].data = False
    result = session.observe()
    assert result['idle']['pending_observation_bootstrap_eligible'] is True
    assert result['idle']['actuation_confirmed'] is False and session.attempt_epoch == epoch
    session.states['epoch'][0].stamp = Stamp(epoch+1)
    with pytest.raises(RuntimeError, match='reference epoch changed'): session.observe(timeout_sec=.001)
    session.states['epoch'][0].stamp = Stamp(epoch)
    session.master = NS(getSystemState=lambda:([('/'+k,[v,'/foreign']) for k,v in session.ctx.idle.OWNERS.items()],[],[]))
    with pytest.raises(RuntimeError, match='publisher ownership'): session.observe(timeout_sec=.001)


@pytest.mark.parametrize('new_value', [False, 'true', 1])
def test_runtime_bootstrap_policy_must_match_accepted_value_and_type(new_value):
    params = frozen_policy_runtime_view()
    session = entry.Session.__new__(entry.Session)
    session.ctx = NS(config={'parameters':frozen_startup_policy_projection()})
    params['/grasp/auto_confirm_actuation_from_observation'] = new_value
    session.rospy = NS(get_param=params.__getitem__)
    with pytest.raises(RuntimeError, match='runtime policy changed'): session.policy()


def test_health_and_first_full_observation_finish_before_timed_prewarm(tmp_path):
    session = entry.Session.__new__(entry.Session); events=[]
    session.report={};session.record=tmp_path
    session.connect=lambda:events.append('connect')
    session.ctx=NS(remote_ref={'path':'binding.json','sha256':'bound'},
        health=NS(load_binding=lambda *a:events.append('binding') or {},
            evaluate=lambda *a:events.append('health') or {'passed':True}),
        warm=NS(PrewarmedModeRunner=lambda *a,**k:pytest.fail('preflight must not start the release timer')))
    session.observe=lambda:events.append('full_observation') or {'manual_parameter':True}
    session.save=lambda:events.append('save')
    session.preflight()
    assert events==['connect','binding','health','full_observation','save']


@pytest.mark.parametrize('fault', ['none','child_exited','imports_budget','observation_budget','remote_expired','manual_changed'])
def test_prewarm_deadline_and_child_liveness_are_checked_before_automatic_request(monkeypatch, fault):
    clock=[100.]; events=[]
    monkeypatch.setattr(entry.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(entry.time,'time_ns',lambda:int(clock[0]*1e9))
    session=entry.Session.__new__(entry.Session)
    session.automatic_requested=False;session.report={'remote_readiness':{'finished_ns':int(clock[0]*1e9)}}
    session.prepare_publisher=lambda:events.append('publisher')
    def prewarm(path, **kwargs):
        events.append('prewarm'); assert kwargs=={'release_timeout':30.}
        clock[0]+=17. if fault=='imports_budget' else 1.
        return NS(ready={'state':'imports_ready'},released=False,
            process=NS(poll=lambda:75 if fault=='child_exited' else None))
    session.ctx=NS(warm=NS(PrewarmedModeRunner=prewarm))
    def observe():
        events.append('fresh_observation')
        clock[0]+=17. if fault=='observation_budget' else 1.
        return {'manual_parameter':fault!='manual_changed','idle':{'actuation_confirmed':False}}
    session.observe=observe
    session.request_mode=lambda value,current:events.append(('mode',value)) or 1
    session.wait_ack=lambda value,requested:events.append(('ack',value))
    if fault=='remote_expired':session.report['remote_readiness']['finished_ns']-=31_000_000_000
    if fault=='none':
        session.enter_automatic()
        assert events==['publisher','prewarm','fresh_observation',('mode',False),('ack',False)]
        assert session.report['prewarm_remaining_before_mode_sec']==28.
        assert session.report['automatic_entry']['idle']['actuation_confirmed'] is False
    else:
        with pytest.raises(RuntimeError):session.enter_automatic()
        assert not session.automatic_requested and not any(isinstance(e,tuple) for e in events)


def test_expired_budget_after_selection_prevents_any_runner_release(monkeypatch):
    monkeypatch.setattr(entry.time,'monotonic',lambda:30.)
    session=entry.Session.__new__(entry.Session);session.prewarm_deadline=30.;session.report={}
    session.prepared=NS(released=False,process=NS(poll=lambda:None),
        run_once=lambda selected:pytest.fail('expired runner must not receive selection'))
    with pytest.raises(RuntimeError,match='budget exhausted'):session.run_selected({'generation':1})


def revision_fixture(tmp_path, monkeypatch):
    workspace=tmp_path/'workspace'; workspace.mkdir();monkeypatch.setattr(entry,'W',workspace)
    test_path='tools/tests/test_run_recorded_manual_grasp.py'
    names=sorted(entry.LAUNCHER_REVISION_PATHS)+['src/unchanged_node.py']
    def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
    def write_ref(name, value):
        path=tmp_path/name;path.write_text(json.dumps(value,sort_keys=True))
        return {'path':str(path),'sha256':digest(path)}
    before={}
    for name in names:
        path=workspace/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_text('baseline '+name)
        before[name]=digest(path)
    after=dict(before)
    for name in entry.LAUNCHER_REVISION_PATHS:
        (workspace/name).write_text('revised '+name);after[name]=digest(workspace/name)
    def run(sources, suffix):
        nodeid=test_path+'::test_'+suffix
        return dict(passed=True,source_unchanged=True,external_boundary_attempts=[],bad_files=[],duplicate_cases=[],
            collection_skips=[],source_sha256=dict(sources),test_files=[test_path],expected_case_ids=[nodeid],
            case_results=[dict(nodeid=nodeid,setup='passed',call='passed',teardown='passed')],
            summary=dict(tests=1,passed=1,skipped=0,failed=0),offline_scope_complete=suffix=='baseline',
            full_regression_passed=suffix=='baseline',complete_scope_requested=suffix=='baseline',
            test_roots=['src','tools'],explicit_test_files=[],excluded_files=[],scope_name='fixture '+suffix)
    baseline=run(before,'baseline');base_ref=write_ref('baseline.json',baseline)
    release=dict(schema=1,kind='current_offline_release',passed=True,reviewed=True,installed=True,
        source_count=len(before),source_sha256=before,actual_regression=base_ref,full_regression_passed=True)
    release_ref=write_ref('release.json',release)
    verification=dict(current=base_ref,current_summary=dict(tests=1,passed=1,skipped=0,failures=0,errors=0),
        scope=baseline['scope_name'],exact_case_inventory=True,test_file_count=1,test_roots=['src','tools'],
        excluded_files=[],skipped_cases=[],test_bodies_executed=True,current_full_regression_passed=True)
    config={'parameters':{'same':True}}; config_ref=write_ref('configuration.json',config)
    control={'state':'active'};control_ref=write_ref('control.json',control)
    startup_instruction=dict(source='actual_user_message',text='start manually',full_restart_authorized=True,
        new_scene_confirmation_claimed=False)
    instruction_ref=write_ref('restart_instruction.json',startup_instruction)
    binary=workspace/'devel/lib/alicia_d_driver/alicia_d_driver_node';binary.parent.mkdir(parents=True);binary.write_bytes(b'unchanged driver')
    contract=dict(schema=1,kind='manual_full_stack_cold_start',workspace_root=str(workspace),output_root=str(tmp_path),storage={'backend':'ssd'},
        release=release_ref,verification=verification,
        current_sha256={str(workspace/name):sha for name,sha in before.items()},
        instruction=instruction_ref,control=control_ref,control_snapshot=control,configuration=config_ref,
        pycache_prefix=str(tmp_path/'unused'),driver_binary_sha256=digest(binary))
    contract_ref=write_ref('contract.json',contract)
    acceptance=dict(contract=contract_ref,source_manifest=release_ref,verification=verification)
    acceptance_ref=write_ref('acceptance.json',acceptance)
    targeted=run(after,'repaired');target_ref=write_ref('targeted.json',targeted)
    revision=dict(schema=1,kind='launcher_only_revision',reviewed=True,runtime_nodes_changed=False,
        current_full_regression_passed=False,startup_acceptance=acceptance_ref,baseline_contract=contract_ref,
        baseline_release=release_ref,baseline_regression=base_ref,targeted_regression=target_ref,
        changed_sources={name:dict(before_sha256=before[name],after_sha256=after[name]) for name in entry.LAUNCHER_REVISION_PATHS})
    context=entry.Context.__new__(entry.Context)
    context.acceptance_ref,context.acceptance,context.release_ref,context.release=acceptance_ref,acceptance,release_ref,release
    context.contract=contract;context.revision=revision
    def source_check(sources):
        for name,sha in sources.items():entry.require(digest(workspace/name)==sha,'installed source changed: '+name)
    context.stack=NS(storage=lambda _,**kwargs:None,sha=digest,source_check=source_check,configuration=lambda **kwargs:deepcopy(config),
        CURRENT_TEST_ROOTS=('src/alicia_grasp_modes/tests','src/alicia_flexible_grasp_supervisor/tests','tools/tests'),
        HISTORICAL_PREWARM_TEST='.runtime/gen6_runner_prewarm_20261012/tests/test_gen6_initial_template_late_start.py',
        validate_contract=lambda _:pytest.fail('must not pass a rewritten release to original validator'))
    def refresh():
        revision['targeted_regression']=write_ref('targeted.json',targeted)
        context.revision_ref=write_ref('revision.json',revision)
    refresh()
    return NS(ctx=context,revision=revision,targeted=targeted,baseline=baseline,before=before,after=after,
        workspace=workspace,refresh=refresh,write_ref=write_ref,config=config,control=control)


def test_launcher_revision_is_exact_targeted_delta_and_preserves_original_receipts(tmp_path,monkeypatch):
    fixture=revision_fixture(tmp_path,monkeypatch);context=fixture.ctx
    refs=[context.acceptance_ref,context.acceptance['contract'],context.release_ref,context.release['actual_regression']]
    original={r['path']:Path(r['path']).read_bytes() for r in refs}
    sources=entry.launcher_revision_sources(fixture.revision,context.acceptance_ref,context.acceptance,
        context.contract,context.release,context.stack)
    assert sources==fixture.after and context.release['source_sha256']==fixture.before
    out,config,release=context.validate_contract()
    assert config==fixture.config and release is context.release
    assert original=={path:Path(path).read_bytes() for path in original}
    assert fixture.revision['current_full_regression_passed'] is False


@pytest.mark.parametrize('fault', ['extra_source','missing_delta','wrong_before','wrong_after','baseline_ref',
    'full_claim','node_change_claim','target_source','source_changed','boundary','missing_case','duplicate_case',
    'extra_case','setup_failed','teardown_failed','case_skipped','summary_lied','no_entry_tests'])
def test_launcher_revision_rejects_scope_or_evidence_expansion(tmp_path,monkeypatch,fault):
    fixture=revision_fixture(tmp_path,monkeypatch);revision,target=fixture.revision,fixture.targeted
    path='tools/run_recorded_manual_grasp.py'
    if fault=='extra_source':revision['changed_sources']['src/unchanged_node.py']=dict(before_sha256='a'*64,after_sha256='b'*64)
    elif fault=='missing_delta':revision['changed_sources'].pop(path)
    elif fault=='wrong_before':revision['changed_sources'][path]['before_sha256']='a'*64
    elif fault=='wrong_after':revision['changed_sources'][path]['after_sha256']='b'*64
    elif fault=='baseline_ref':revision['baseline_release']=dict(path='different',sha256='a'*64)
    elif fault=='full_claim':revision['current_full_regression_passed']=True
    elif fault=='node_change_claim':revision['runtime_nodes_changed']=True
    elif fault=='target_source':target['source_sha256']['src/unchanged_node.py']='a'*64
    elif fault=='source_changed':target['source_unchanged']=False
    elif fault=='boundary':target['external_boundary_attempts']=['network attempt']
    elif fault=='missing_case':target['case_results']=[]
    elif fault=='duplicate_case':target['case_results']*=2
    elif fault=='extra_case':target['case_results'].append(dict(target['case_results'][0],nodeid='other::test'))
    elif fault=='setup_failed':target['case_results'][0]['setup']='failed'
    elif fault=='teardown_failed':target['case_results'][0]['teardown']='failed'
    elif fault=='case_skipped':target['case_results'][0].update(call='skipped',skip_reason='not run')
    elif fault=='summary_lied':target['summary']['passed']=20
    elif fault=='no_entry_tests':target['test_files']=[]
    fixture.refresh()
    with pytest.raises(RuntimeError):fixture.ctx.validate_contract()


@pytest.mark.parametrize('fault', ['node_source','entry_source','test_source','configuration','driver','cache',
    'control','baseline_bytes','revision_bytes','startup_input'])
def test_launcher_overlay_preserves_all_other_startup_contract_invariants(tmp_path,monkeypatch,fault):
    fixture=revision_fixture(tmp_path,monkeypatch);context=fixture.ctx
    if fault=='node_source':(fixture.workspace/'src/unchanged_node.py').write_text('changed production')
    elif fault=='entry_source':(fixture.workspace/'tools/run_recorded_manual_grasp.py').write_text('unverified entry')
    elif fault=='test_source':(fixture.workspace/'tools/tests/test_run_recorded_manual_grasp.py').write_text('unverified test')
    elif fault=='configuration':context.stack.configuration=lambda **kwargs:{'parameters':{'same':False}}
    elif fault=='driver':(fixture.workspace/'devel/lib/alicia_d_driver/alicia_d_driver_node').write_bytes(b'changed')
    elif fault=='cache':Path(context.contract['pycache_prefix']).mkdir()
    elif fault=='control':Path(context.contract['control']['path']).write_text('{"state":"paused"}')
    elif fault=='baseline_bytes':Path(context.release['actual_regression']['path']).write_text('{}')
    elif fault=='revision_bytes':Path(context.revision_ref['path']).write_text('{}')
    elif fault=='startup_input':Path(context.contract['instruction']['path']).write_text('{}')
    with pytest.raises(RuntimeError):context.validate_contract()


def test_without_launcher_revision_original_contract_validation_remains_unchanged(tmp_path):
    control=tmp_path/'control.json';control.write_text('{"state":"active"}')
    context=entry.Context.__new__(entry.Context);context.revision=None
    context.contract={'original':True,'control':dict(path=str(control),sha256=hashlib.sha256(control.read_bytes()).hexdigest())}
    calls=[];context.stack=NS(validate_contract=lambda contract,**kwargs:calls.append((contract,kwargs)) or 'original result')
    assert context.validate_contract()=='original result'
    assert calls==[(context.contract,{'manual_return':False})]


def test_paused_manual_startup_never_supplies_automatic_attempt_authority(tmp_path,monkeypatch):
    from test_full_stack_start import contract_fixture,cold
    fixture=contract_fixture(tmp_path,monkeypatch)
    control={'state':'paused','revision':70}
    fixture.contract['control']=fixture.write_ref('control.json',control)
    fixture.contract['control_snapshot']=control
    monkeypatch.setattr(cold.shutil,'disk_usage',lambda path:NS(free=fixture.disk.minimum))
    before=Path(fixture.contract['control']['path']).read_bytes()
    assert cold.validate_contract(fixture.contract)[0]==fixture.disk.out
    context=entry.Context.__new__(entry.Context)
    context.revision=None;context.contract=fixture.contract;context.stack=cold
    with pytest.raises(RuntimeError,match='automatic grasp requires active control'):
        context.validate_contract()
    assert context.validate_contract(manual_return=True)[0]==fixture.disk.out
    assert Path(fixture.contract['control']['path']).read_bytes()==before


def test_overlay_rejects_new_test_not_frozen_in_original_or_targeted_inventory(tmp_path,monkeypatch):
    fixture=revision_fixture(tmp_path,monkeypatch)
    (fixture.workspace/'tools/tests/test_unfrozen_new.py').write_text('def test_unfrozen(): pass\n')
    # Every listed source is still byte-for-byte correct; discovery must reject
    # the new unlisted test just as the original current-release verifier does.
    fixture.ctx.stack.source_check(fixture.after)
    with pytest.raises(RuntimeError,match='discovered test files'):
        fixture.ctx.validate_contract()


@pytest.mark.parametrize('fault', ['missing_root','absolute_root','duplicate_root','explicit_missing',
    'exclusion_no_reason','exclusion_duplicate','exclusion_executed','exclusion_invented',
    'historical_prewarm_omitted','blank_scope','release_full_claim','run_full_claim'])
def test_revision_preserves_original_discovery_and_historical_claim_rules(tmp_path,monkeypatch,fault):
    fixture=revision_fixture(tmp_path,monkeypatch)
    release,run=deepcopy(fixture.ctx.release),deepcopy(fixture.baseline)
    if fault=='missing_root':run['test_roots']=['tools']
    elif fault=='absolute_root':run['test_roots']=[str(fixture.workspace/'src'),'tools']
    elif fault=='duplicate_root':run['test_roots'].append('tools')
    elif fault=='explicit_missing':run['explicit_test_files']=['.runtime/missing.py']
    elif fault=='exclusion_no_reason':run['excluded_files']=[dict(path='src/test_absent.py',reason='')]
    elif fault=='exclusion_duplicate':run['excluded_files']=[dict(path='src/test_absent.py',reason='reason')]*2
    elif fault=='exclusion_executed':run['excluded_files']=[dict(path=run['test_files'][0],reason='reason')]
    elif fault=='exclusion_invented':run['excluded_files']=[dict(path='src/test_absent.py',reason='reason')]
    elif fault=='historical_prewarm_omitted':
        path=fixture.workspace/fixture.ctx.stack.HISTORICAL_PREWARM_TEST
        path.parent.mkdir(parents=True);path.write_text('def test_unlisted(): pass\n')
    elif fault=='blank_scope':run['scope_name']=''
    elif fault=='release_full_claim':release['full_regression_passed']=False
    elif fault=='run_full_claim':run['full_regression_passed']=False
    with pytest.raises(RuntimeError):entry.recorded_baseline_verification(release,run,fixture.ctx.stack)


def test_baseline_skips_remain_explicit_and_are_never_changed_into_full_pass(tmp_path,monkeypatch):
    fixture=revision_fixture(tmp_path,monkeypatch)
    release,run=deepcopy(fixture.ctx.release),deepcopy(fixture.baseline)
    release['full_regression_passed']=run['full_regression_passed']=False
    run['case_results'][0].update(setup='skipped',call=None,skip_reason='external recorded fixture absent')
    run['summary'].update(passed=0,skipped=1)
    evidence=entry.recorded_baseline_verification(release,run,fixture.ctx.stack)
    assert evidence['current_summary']==dict(tests=1,passed=0,skipped=1,failures=0,errors=0)
    assert evidence['test_bodies_executed'] is evidence['current_full_regression_passed'] is False
    assert evidence['skipped_cases']==[dict(nodeid=run['expected_case_ids'][0],reason='external recorded fixture absent')]


@pytest.mark.parametrize('field,value', [('scope','invented scope'),('test_file_count',999),
    ('exact_case_inventory',False),('skipped_cases',[{'nodeid':'invented','reason':'invented'}])])
def test_every_accepted_verification_field_is_reconstructed_before_overlay_use(tmp_path,monkeypatch,field,value):
    fixture=revision_fixture(tmp_path,monkeypatch);context=fixture.ctx
    # Even mutually consistent acceptance/contract claims cannot replace the
    # original regression's actual scope, cases or skipped-body explanations.
    context.acceptance['verification'][field]=value
    with pytest.raises(RuntimeError,match='regression verification differs'):
        entry.launcher_revision_sources(fixture.revision,context.acceptance_ref,context.acceptance,
            context.contract,context.release,context.stack)


def test_actual_frozen_roslaunch_policy_projection_matches_all_runtime_namespace_values():
    flat, runtime = frozen_startup_policy_projection(), frozen_policy_runtime_view()
    assert '/camera/mode_depth_preset' not in flat  # Actual failing construction shape.
    assert flat['/grasp/unknown_observe_return_max_observations'] == 1
    session=entry.Session.__new__(entry.Session);reads=[]
    session.ctx=NS(config={'parameters':flat})
    session.rospy=NS(get_param=lambda key:reads.append(key) or deepcopy(runtime[key]))
    assert session.policy()==runtime
    assert set(reads)==set(runtime) and len(reads)==12
    assert session.policy(manual_return=True)=={k:v for k,v in runtime.items() if k.startswith('/alicia_d_driver_node/')}


@pytest.mark.parametrize('representation', ['flat','nested','mixed_nonoverlapping'])
def test_bound_parameter_reader_reconstructs_equivalent_flat_and_nested_trees(representation):
    expected={'enabled':True,'levels':[1,2.0,{'active':False}], 'empty':{}}
    if representation=='flat':
        parameters={'/policy/enabled':True,'/policy/levels':deepcopy(expected['levels']),'/policy/empty':{}}
    elif representation=='nested':parameters={'/policy':deepcopy(expected)}
    else:parameters={'/policy':{'enabled':True},'/policy/levels':deepcopy(expected['levels']),'/policy/empty':{}}
    original=deepcopy(parameters)
    assert entry.read_bound_parameters(parameters,lambda key:deepcopy(expected),['/policy'])=={'/policy':expected}
    assert parameters==original


@pytest.mark.parametrize('parameters', [
    {'/policy':True,'/policy/enabled':True},
    {'/policy/enabled':True,'/policy':False},
    {'/policy':{},'/policy/enabled':True},
    {'/policy':{'enabled':True},'/policy/enabled':True},
    {'/policy/enabled':True,'/policy':{'enabled':False}},
    {'policy/enabled':True}, {'/policy//enabled':True}, {'/policy/../enabled':True},
    {'/policy/':True}, {'/policy':{'bad/key':True}}, {'/policy':{1:True}},
    {'/policy':None}, {'/policy':[float('nan')]}, {'/policy':('tuple',)},
])
def test_bound_parameter_reader_rejects_invalid_or_conflicting_frozen_bindings(parameters):
    with pytest.raises(RuntimeError):
        entry.read_bound_parameters(parameters,lambda key:pytest.fail('invalid frozen binding must fail before runtime read'),['/policy'])


@pytest.mark.parametrize('fault', ['missing_expected','missing_runtime','extra_runtime_leaf','missing_runtime_leaf',
    'nested_bool_as_int','nested_float_as_int','list_bool_as_int','list_extra','dict_key_type','relative_query'])
def test_bound_parameter_reader_requires_exact_recursive_values_types_and_shape(fault):
    expected={'enabled':True,'scale':1.0,'nested':{'active':False},'levels':[True,2.0,{'keep':True}]}
    parameters={'/policy':deepcopy(expected)};actual=deepcopy(expected);keys=['/policy']
    if fault=='missing_expected':parameters={}
    elif fault=='extra_runtime_leaf':actual['new_field']=False
    elif fault=='missing_runtime_leaf':actual.pop('scale')
    elif fault=='nested_bool_as_int':actual['nested']['active']=0
    elif fault=='nested_float_as_int':actual['scale']=1
    elif fault=='list_bool_as_int':actual['levels'][2]['keep']=1
    elif fault=='list_extra':actual['levels'].append(False)
    elif fault=='dict_key_type':actual[1]=actual.pop('enabled')
    elif fault=='relative_query':keys=['policy']
    def read(name):
        if fault=='missing_runtime':raise KeyError(name)
        return actual
    with pytest.raises(RuntimeError):entry.read_bound_parameters(parameters,read,keys)


@pytest.mark.parametrize('fault', ['extra','missing','leaf_type','leaf_value'])
def test_actual_flat_camera_policy_rejects_any_runtime_namespace_drift(fault):
    session=entry.Session.__new__(entry.Session)
    session.ctx=NS(config={'parameters':frozen_startup_policy_projection()})
    runtime=frozen_policy_runtime_view();camera=runtime['/camera/mode_depth_preset']
    if fault=='extra':camera['unbound_flag']=True
    elif fault=='missing':camera.pop('regional_stationary_enabled')
    elif fault=='leaf_type':camera['enabled']=1
    elif fault=='leaf_value':camera['unknown_preset']='default'
    session.rospy=NS(get_param=runtime.__getitem__)
    with pytest.raises(RuntimeError,match='/camera/mode_depth_preset'):session.policy()


def test_manual_context_low_space_exemption_does_not_bypass_instance_check(tmp_path,monkeypatch):
    from test_full_stack_start import cold, contract_fixture
    fixture=contract_fixture(tmp_path,monkeypatch)
    context=entry.Context.__new__(entry.Context)
    context.revision=None;context.contract=fixture.contract;context.stack=cold
    context.acceptance={'core_unit':'accepted.service','core_service':{'MainPID':'accepted'}}
    context.acceptance_ref=fixture.write_ref('acceptance.json',context.acceptance)
    context.instruction_ref=fixture.contract['instruction'];context.instruction=entry.read_ref(context.instruction_ref)
    monkeypatch.setattr(cold,'properties',lambda unit:{'MainPID':'different'})
    with pytest.raises(RuntimeError,match='recording minimum'):context.recheck(None,'/test')
    with pytest.raises(RuntimeError,match='accepted core service changed'):
        context.recheck(None,'/test',manual_return=True)


def test_launcher_revision_keeps_same_manual_storage_purpose_and_other_source_checks(tmp_path,monkeypatch):
    from test_full_stack_start import cold, storage_fixture
    fixture=revision_fixture(tmp_path,monkeypatch);disk=storage_fixture(tmp_path,monkeypatch)
    fixture.ctx.contract['output_root']=str(disk.out)
    fixture.ctx.contract['storage']=disk.selected.validate(disk.out)
    # Contract bytes stay bound to the revised fixture, exactly as a new
    # accepted baseline would be; no production receipt is rewritten.
    contract_ref=fixture.write_ref('contract.json',fixture.ctx.contract)
    fixture.ctx.acceptance['contract']=contract_ref
    fixture.ctx.acceptance_ref=fixture.write_ref('acceptance.json',fixture.ctx.acceptance)
    fixture.revision['baseline_contract']=contract_ref
    fixture.revision['startup_acceptance']=fixture.ctx.acceptance_ref;fixture.refresh()
    fixture.ctx.stack.storage=cold.storage
    with pytest.raises(RuntimeError,match='recording minimum'):fixture.ctx.validate_contract()
    assert fixture.ctx.validate_contract(manual_return=True)[0]==disk.out
    (fixture.workspace/'src/unchanged_node.py').write_text('foreign node source')
    with pytest.raises(RuntimeError):fixture.ctx.validate_contract(manual_return=True)


@pytest.mark.parametrize('fault',['none','active_task','active_action','wrong_gui_owner','epoch','non_sdk','moving'])
def test_low_space_manual_observation_keeps_live_sdk_and_idle_boundaries(tmp_path,monkeypatch,fault):
    from test_full_stack_start import cold, contract_fixture
    fixture=contract_fixture(tmp_path,monkeypatch)
    context=entry.Context.__new__(entry.Context);context.revision=None
    context.stack=cold;context.contract=fixture.contract
    session,_,epoch=observed_session()
    session.ctx.recheck=lambda *args,**kwargs:context.validate_contract(manual_return=kwargs['manual_return'])
    session.states['actuation'][0].data='UNCONFIRMED:DISCONTINUOUS_FEEDBACK_RECOVERY'
    session.states['enabled'][0].data=False
    if fault=='active_task':session.states['task'][0].active=True
    elif fault=='active_action':session.states['arm'][0].status_list=[NS(status=1)]
    elif fault=='wrong_gui_owner':session.states['manual'][0]._connection_header['callerid']='/foreign'
    elif fault=='epoch':session.states['epoch'][0].stamp=Stamp(epoch-1)
    elif fault=='non_sdk':session.samples[-1]['frame_id']='commanded'
    elif fault=='moving':session.samples[-1]['positions']=[.2]*6
    if fault=='none':
        result=session.observe(manual_return=True)
        assert result['idle']['automatic_motion_authorized'] is False
        assert result['idle']['motion_enabled'] is False
        with pytest.raises(RuntimeError,match='recording minimum'):session.observe()
    else:
        with pytest.raises(RuntimeError):session.observe(manual_return=True,timeout_sec=.001)


class TimedObserverTelemetry:
    """A 10 Hz callback stream with advancing ROS and monotonic clocks.

    The seven snapshot reads replay the currently held messages, as rospy's
    shared subscriber does for a latch. They do not pretend to open a new TCP
    connection. Only the scheduled SDK/action callbacks supply new telemetry.
    """
    def __init__(self, monkeypatch, *, recheck_sec=0., read_sec=.005,
                 fault=None, stop_sdk_sec=None, moving_until_sec=None):
        self.session, self.base_ns, self.epoch = observed_session()
        self.templates = {key:deepcopy(value[0]) for key,value in self.session.states.items()}
        self.session.states, self.session.samples = {}, []
        self.elapsed_ns, self.next_tick_ns = 0, 100_000_000
        self.recheck_sec, self.read_sec = recheck_sec, read_sec
        self.fault, self.stop_sdk_sec, self.moving_until_sec = fault, stop_sdk_sec, moving_until_sec
        self.reads, self.guard_calls = [], []
        self.session.rospy.Time = NS(now=lambda:Stamp(self.base_ns+self.elapsed_ns))
        self.session.rospy.wait_for_message = self.read_snapshot
        self.session.ctx.recheck = lambda *args,**kwargs:self.advance(recheck_sec)
        self.session.ctx.boundary = NS(validate_manual_return_snapshot=self.validate_snapshot)
        self.session.master.getSystemState = self.publisher_state
        monkeypatch.setattr(entry, 'time', NS(monotonic=lambda:self.elapsed_ns/1e9,
                                             sleep=self.advance))
        for key,message in self.templates.items():
            if key != 'joint': self.session.receive(key, deepcopy(message))

    def advance(self, seconds):
        assert seconds >= 0
        target = self.elapsed_ns+round(seconds*1e9)
        while self.next_tick_ns <= target:
            self.elapsed_ns = self.next_tick_ns
            self.next_tick_ns += 100_000_000
            self.tick()
        self.elapsed_ns = target

    def tick(self):
        now_ns, elapsed = self.base_ns+self.elapsed_ns, self.elapsed_ns/1e9
        if elapsed >= 1.:
            if self.fault == 'fixed_epoch': self.templates['epoch'].stamp = Stamp(self.epoch+1)
            elif self.fault == 'gui_owner': self.templates['manual']._connection_header['callerid'] = '/foreign'
            elif self.fault == 'task_active': self.templates['task'].active = True
            elif self.fault == 'router_active':
                self.templates['mode'].data = json.dumps(dict(active=True,pending='',start_reserved=False))
        for key in ('arm','hand'):
            message = deepcopy(self.templates[key])
            message.header.stamp = Stamp(now_ns if self.fault != 'action_stamp_stale' else self.epoch)
            if self.fault == key+'_active': message.status_list = [NS(status=1)]
            self.session.receive(key,message)
        if self.stop_sdk_sec is not None and elapsed > self.stop_sdk_sec: return
        tick = self.elapsed_ns//100_000_000
        moving = self.fault == 'moving' or (self.moving_until_sec is not None and elapsed <= self.moving_until_sec)
        positions = [.1+(.01*(tick % 2) if moving else 0.)]*6
        stamp = now_ns-5_000_000
        if self.fault == 'repeated_sdk_stamp': stamp = self.base_ns
        message = NS(_connection_header=dict(callerid=self.session.ctx.idle.OWNERS['joint']),
            header=NS(stamp=Stamp(stamp),frame_id='commanded' if self.fault=='non_sdk' else 'sdk_measured'),
            name=['Joint%d' % i for i in range(1,7)],position=positions)
        self.session.receive('joint',message)

    def read_snapshot(self, topic, kind, timeout):
        key = topic[1:]
        self.reads.append(dict(key=key,started_ns=self.elapsed_ns,timeout=timeout,
                               sample_count=len(self.session.samples)))
        # A timeout consumes the supplied budget; no frozen clock hides it.
        if self.read_sec > timeout:
            self.advance(timeout)
            raise RuntimeError('snapshot timed out: '+key)
        self.advance(self.read_sec)
        return deepcopy(self.templates[key])

    def publisher_state(self):
        rows = [('/'+key,[owner]) for key,owner in self.session.ctx.idle.OWNERS.items()]
        if self.fault == 'publisher_owner' and self.elapsed_ns >= 1_000_000_000:
            rows = [(topic,owners+['/foreign'] if topic=='/joint' else owners) for topic,owners in rows]
        return rows,[],[]

    def validate_snapshot(self, samples, states, now, epoch, owners):
        row = dict(now_ns=now,sample_count=len(samples),
            observed_ns=self.base_ns+self.elapsed_ns,epoch_age_ns=now-states['epoch'][1],
            sdk_stamp_ns=samples[-1]['stamp_ns'] if samples else None)
        self.guard_calls.append(row)
        try:
            result = boundary.validate_manual_return_snapshot(samples,states,now,epoch,owners)
        except (RuntimeError,ValueError,AssertionError,KeyError) as exc:
            row['error'] = repr(exc)
            raise
        row['passed'] = True
        return result


@pytest.mark.parametrize('recheck_sec', [0.,.15,1.5,2.2])
@pytest.mark.parametrize('manual_return', [False,True])
def test_timed_observer_collects_10hz_window_before_latched_snapshot(monkeypatch,recheck_sec,manual_return):
    stream = TimedObserverTelemetry(monkeypatch,recheck_sec=recheck_sec)
    result = stream.session.observe(manual_return=manual_return)
    assert result['epoch_ns'] == stream.epoch
    assert result['idle']['sample_spans_rad'] == {'Joint%d' % i:0. for i in range(1,7)}
    assert stream.reads and all(row['sample_count'] >= 20 for row in stream.reads)
    assert [row['key'] for row in stream.reads] == ['epoch','manual','task','mode','actuation','ready','enabled']
    assert stream.guard_calls[-1]['passed'] is True
    assert 0 <= stream.guard_calls[-1]['epoch_age_ns'] < 1_000_000_000
    assert stream.elapsed_ns/1e9 <= recheck_sec+5.


@pytest.mark.parametrize('manual_return', [False,True])
@pytest.mark.parametrize('fault', ['task_active','router_active','arm_active','hand_active',
    'fixed_epoch','gui_owner','publisher_owner','action_stamp_stale','non_sdk','moving','repeated_sdk_stamp'])
def test_timed_observer_keeps_live_idle_epoch_owner_and_sdk_gates(monkeypatch,manual_return,fault):
    stream = TimedObserverTelemetry(monkeypatch,fault=fault)
    if fault == 'fixed_epoch':
        if manual_return: stream.session.finish_epoch = stream.epoch
        else: stream.session.attempt_epoch = stream.epoch
    with pytest.raises(RuntimeError,match='boundary unavailable'):
        stream.session.observe(manual_return=manual_return)
    assert not any(row.get('passed') for row in stream.guard_calls) or fault == 'publisher_owner'
    assert stream.elapsed_ns <= 5_000_000_000


@pytest.mark.parametrize('manual_return', [False,True])
def test_timed_observer_sdk_stop_before_window_does_not_read_latches(monkeypatch,manual_return):
    stream = TimedObserverTelemetry(monkeypatch,stop_sdk_sec=.8)
    with pytest.raises(RuntimeError,match='sample window incomplete'):
        stream.session.observe(manual_return=manual_return)
    assert len(stream.session.samples) == 8
    assert stream.reads == [] and stream.guard_calls == []
    assert stream.elapsed_ns == 5_000_000_000


@pytest.mark.parametrize('manual_return', [False,True])
def test_timed_observer_stopped_sdk_cannot_be_refreshed_by_latched_replay(monkeypatch,manual_return):
    stream = TimedObserverTelemetry(monkeypatch,recheck_sec=2.1,read_sec=.04,stop_sdk_sec=2.1)
    with pytest.raises(RuntimeError,match='not freshly stationary'):
        stream.session.observe(manual_return=manual_return)
    assert stream.guard_calls and not any(row.get('passed') for row in stream.guard_calls)
    assert stream.elapsed_ns <= 7_100_000_000


@pytest.mark.parametrize('manual_return', [False,True])
def test_timed_observer_retries_complete_snapshot_after_stationarity_settles(monkeypatch,manual_return):
    stream = TimedObserverTelemetry(monkeypatch,moving_until_sec=1.1)
    result = stream.session.observe(manual_return=manual_return)
    assert result['epoch_ns'] == stream.epoch
    assert 'not freshly stationary' in stream.guard_calls[0]['error']
    assert stream.guard_calls[-1]['passed'] is True
    assert len(stream.reads) >= 14
    for index in range(0,len(stream.reads),7):
        assert [row['key'] for row in stream.reads[index:index+7]] == [
            'epoch','manual','task','mode','actuation','ready','enabled']
    assert 3. <= stream.elapsed_ns/1e9 <= 5.


@pytest.mark.parametrize('read_sec', [.6,3.1])
def test_timed_observer_snapshot_reads_share_original_five_second_budget(monkeypatch,read_sec):
    stream = TimedObserverTelemetry(monkeypatch,read_sec=read_sec)
    with pytest.raises(RuntimeError,match='boundary unavailable'):
        stream.session.observe()
    assert stream.reads[0]['started_ns'] >= 2_000_000_000
    assert stream.elapsed_ns <= 5_000_000_000
    assert all(0 < row['timeout'] <= min(3.,5.-row['started_ns']/1e9)+1e-9 for row in stream.reads)
    assert not any(row.get('passed') for row in stream.guard_calls)


def test_timed_observer_preserves_first_guard_rejection_with_final_error(monkeypatch):
    stream = TimedObserverTelemetry(monkeypatch,recheck_sec=2.1,fault='task_active')
    with pytest.raises(RuntimeError,match='boundary unavailable') as caught:
        stream.session.observe()
    assert 'task still active' in stream.guard_calls[0]['error']
    assert 'first_error=' in str(caught.value) and 'last_error=' in str(caught.value)
    assert stream.guard_calls[0]['error'] in str(caught.value)


def test_timed_observer_loads_automatic_dependency_before_fresh_snapshot(monkeypatch):
    import builtins
    stream = TimedObserverTelemetry(monkeypatch,recheck_sec=2.1)
    original_import, imported = builtins.__import__, []
    def delayed_first_import(name, *args, **kwargs):
        result = original_import(name,*args,**kwargs)
        if name == 'alicia_flexible_grasp.robot.actuation_bootstrap' and not imported:
            stream.advance(.35)
            imported.append(stream.elapsed_ns)
        return result
    monkeypatch.setattr(builtins,'__import__',delayed_first_import)
    result = stream.session.observe()
    assert imported and result['epoch_ns'] == stream.epoch
    assert all(row['started_ns'] >= imported[0] for row in stream.reads)
    assert all(row['now_ns'] == row['observed_ns'] for row in stream.guard_calls)
    assert stream.guard_calls[-1]['observed_ns']-result['latest_sdk_stamp_ns'] < 200_000_000
