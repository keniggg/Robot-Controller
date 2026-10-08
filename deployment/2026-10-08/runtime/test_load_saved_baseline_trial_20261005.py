"""Trial preparation and one-change restoration ordering; no ROS/device IO."""
import copy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

HERE=Path(__file__).resolve().parent
def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result);return result
m=module('baseline_trial_under_test',HERE/'load_saved_baseline_trial_20261005.py')
r=module('baseline_trial_runner_helpers',HERE/'run_matched_shadow_current_20261005.py')


def test_default_is_preparation_without_loading_runtime(monkeypatch,capsys):
    monkeypatch.setattr(m,'load',lambda *a:pytest.fail('runtime import during preparation'))
    m.main([]);result=json.loads(capsys.readouterr().out)
    assert result['prepared_only'] and not result['device_actions_performed']
    assert result['unknown_mode_preserved'] and not result['new_motion_authority_created']
    assert result['parameter']==m.PARAM and result['value']=='saved_baseline'
    assert not result['persistent_config_files_changed']


def test_execute_without_reviewed_manifest_never_loads_runtime(monkeypatch):
    monkeypatch.setattr(m,'load',lambda *a:pytest.fail('runtime import without manifest'))
    with pytest.raises(SystemExit):m.main(['--execute'])


def wrapper_fixture():
    argv=['/usr/bin/python3',str(m.WORK/r.WRAPPER),'--enable-preset-ab',
        '--external-consumers-quiesced','--matched-stationary-shadow','--camera-script',
        str(m.WORK/r.RELAY['camera']),'--output',str(m.DEFAULT_RECORD/'stages'),
        '--seconds','75','--interval','.5','--','__name:=supervisor_camera','__log:=/external/camera.log']
    report={'events':[dict(event='start_requested',role='camera',unit='owned-unit',command=['systemd-run']+argv)]}
    recovery=dict(visual_nodes_restored=True,preserved_nodes_unchanged=True)
    summary=dict(restoration=dict(succeeded=True),original_read_restored=True,original_sync_restored=True,experiment_valid=False)
    return argv,report,recovery,summary


def test_only_completed_restored_wrapper_is_unwrapped_to_normal_camera():
    argv,report,recovery,summary=wrapper_fixture()
    normal,unit=m.completed_wrapper_command(argv,report,recovery,summary,r,m.DEFAULT_RECORD)
    assert normal==[argv[0],str(m.WORK/r.RELAY['camera'])]+argv[argv.index('--')+1:]
    assert unit=='owned-unit' and '--enable-preset-ab' not in normal
    # Invalid capture is not relabelled valid merely because device restoration succeeded.
    assert summary['experiment_valid'] is False


@pytest.mark.parametrize('fault',['restoration','read','sync','consumers','core','other_argv',
    'extra_launch','long_duration','wrong_output','nested_wrapper'])
def test_unwrap_rejects_unproved_or_different_wrapper(fault):
    argv,report,recovery,summary=wrapper_fixture()
    report=copy.deepcopy(report)
    if fault=='restoration':summary['restoration']['succeeded']=False
    elif fault=='read':summary['original_read_restored']=False
    elif fault=='sync':summary['original_sync_restored']=False
    elif fault=='consumers':recovery['visual_nodes_restored']=False
    elif fault=='core':recovery['preserved_nodes_unchanged']=False
    elif fault=='other_argv':argv[-1]='__log:=/other.log'
    elif fault=='extra_launch':report['events']*=2
    else:
        if fault=='long_duration':argv[argv.index('--seconds')+1]='999'
        elif fault=='wrong_output':argv[argv.index('--output')+1]='/tmp/elsewhere'
        elif fault=='nested_wrapper':argv[argv.index('--camera-script')+1]=str(m.WORK/r.WRAPPER)
        report['events'][0]['command']=['systemd-run']+argv
    with pytest.raises(RuntimeError):m.completed_wrapper_command(argv,report,recovery,summary,r,m.DEFAULT_RECORD)


class FakeTrial:
    def __init__(self,fault=None,recovery_fault=False):
        self.events=[];self.report={};self.current={role:object() for role in r.MUTABLE}
        self.retirement_completed=False;self.old_preset='high_accuracy';self.fault=fault
        self.recovery_fault=recovery_fault;self.failed=False;self.preset='high_accuracy'
    def verify(self):self.events.append('verify')
    def stop(self,role):
        self.events.append('stop '+role)
        if self.fault=='first_stop' and not self.failed:self.failed=True;raise RuntimeError('stop failed')
        self.current[role]=None
    def retire_selection(self):
        assert self.current['unknown'] is None and self.current['remote'] is not None
        self.events.append('retire once')
        if self.fault=='retire':raise RuntimeError('retirement unproved')
        self.retirement_completed=True
    def configure(self,value):
        assert all(self.current[role] is None for role in r.MUTABLE)
        self.preset=value;self.events.append('preset '+value)
    def start(self,role):
        self.events.append('start '+role)
        if role=='camera':assert self.current['unknown'] is None and self.current['remote'] is None
        if self.fault=='camera' and role=='camera' and not self.failed:
            self.failed=True;raise RuntimeError('camera failed')
        if self.fault==role and role!='camera' and not self.failed:
            self.failed=True;raise RuntimeError('consumer start failed')
        self.current[role]=object()
    def validate_camera(self):
        self.events.append('validate '+self.preset)
        if self.fault=='validate' and not self.failed:self.failed=True;raise RuntimeError('bad camera')
    def restored_warmup(self):
        self.events.append('warmup '+self.preset)
        if self.recovery_fault or (self.fault=='warmup' and not self.failed):
            self.failed=True;raise RuntimeError('no fresh image')
    def clear_camera_validation(self):self.events.append('clear validation')
    def settle_camera_launch(self):self.events.append('settle owned camera')
    def final_validate(self):
        self.events.append('final')
        if self.fault=='final' and not self.failed:self.failed=True;raise RuntimeError('final validation failed')
        self.report['visual_nodes_restored']=True
    def quiesce_consumer(self,role):self.stop(role)
    def record(self,*a,**k):pass


def test_trial_retires_before_parameter_write_and_restores_consumers_after_warmup():
    io=FakeTrial();m.trial_transaction(io);events=io.events
    assert events.index('stop unknown')<events.index('retire once')<events.index('stop remote')
    assert events.index('stop camera')<events.index('preset saved_baseline')<events.index('start camera')
    assert events.index('warmup saved_baseline')<events.index('start unknown')<events.index('start remote')
    assert io.report['trial_loaded'] and io.report['visual_nodes_restored']
    assert events.count('retire once')==1 and 'preset high_accuracy' not in events


@pytest.mark.parametrize('fault',['camera','validate','warmup','unknown','remote','final'])
def test_camera_failures_restore_original_parameter_and_fresh_camera_before_consumers(fault):
    io=FakeTrial(fault)
    with pytest.raises(RuntimeError):m.trial_transaction(io)
    assert io.preset=='high_accuracy' and io.report['old_preset_recovered']
    assert io.events.index('warmup high_accuracy')<max(i for i,event in enumerate(io.events) if event=='start unknown')
    assert io.events.count('retire once')==1 and io.report['visual_nodes_restored']
    assert io.report['trial_loaded'] is False


@pytest.mark.parametrize('fault',['first_stop','retire'])
def test_incomplete_retirement_never_changes_camera_or_revives_old_tracker(fault):
    io=FakeTrial(fault)
    with pytest.raises(RuntimeError):m.trial_transaction(io)
    assert not any(event.startswith(('preset ','start ','stop camera','stop remote')) for event in io.events)


def test_failed_camera_recovery_leaves_consumers_quiesced():
    io=FakeTrial('camera',recovery_fault=True)
    with pytest.raises(RuntimeError):m.trial_transaction(io)
    assert 'start unknown' not in io.events and 'start remote' not in io.events
    assert 'camera_recovery_error' in io.report


def test_source_has_one_parameter_write_and_no_robot_or_persistent_configuration_writes():
    source=(HERE/'load_saved_baseline_trial_20261005.py').read_text()
    assert source.count('.set_param(')==1 and 'self.ros.set_param(PARAM,value)' in source
    assert '.Publisher(' not in source and '.ServiceProxy(' not in source
    assert 'camera.yaml' not in source and 'start_grasp' not in source


def pure_trial_class(helper):
    # Exercise the real two new methods; parent lifecycle helpers have their
    # separate exact PID/URI/start-order regression suite above.
    runner=NS(make_runtime_class=lambda *args:object,
        complete_pair_is_valid=lambda stamp,pair,camera,cutoff,now:cutoff<stamp<=now)
    return m.make_runtime(runner,None,helper)


@pytest.mark.parametrize('observed_active',[True,False])
def test_real_warmup_requires_actual_active_diagnostic_after_fresh_pairs(monkeypatch,observed_active):
    camera=dict(runtime_profile=dict(mode_depth_preset=dict(preset='saved_baseline'),
        geometry_activation_stamp_ns='400',stationary_temporal=dict(enabled=True,active=False,
            alpha=.4,delta_depth_units=100,persistence=0)))
    attempts=[]
    def wait(predicate,*args):
        attempts.append(predicate())
        assert attempts==[False]  # enabled alone and fresh images are insufficient.
        camera['runtime_profile']['stationary_temporal']['active']=observed_active
        answer=predicate()
        if not answer:raise RuntimeError('no active temporal diagnostic')
        return answer
    helper=NS(wait=wait);cls=pure_trial_class(helper);obj=cls.__new__(cls)
    stamps=iter((500,600,601))
    obj.ros=NS(Time=NS(now=lambda:NS(to_nsec=lambda:next(stamps))),get_param=lambda key:camera)
    obj.configuration=dict(mode_depth_preset=dict(unknown_preset='saved_baseline'))
    obj.current=dict(camera=dict(pid=38860));obj.report={};obj.guard=lambda:None
    obj.validate_camera=lambda **kwargs:None;obj.lock=m.threading.RLock()
    obj.pairs={stamp:{} for stamp in range(510,521)}
    ticks=iter((0.,4.,4.,4.))
    monkeypatch.setattr(m,'time',NS(monotonic=lambda:next(ticks)))
    if observed_active:
        obj.restored_warmup()
        proof=obj.report['post_restore_warmup']['stationary_active_diagnostic']
        assert proof['status']['active'] is True and proof['owner_pid']==38860
        assert proof['diagnostic_only'] is True and proof['per_frame_binding'] is False
        assert proof['observed_ros_stamp_ns']==601
        assert proof['geometry_activation_stamp_ns']=='400'
        assert obj.report['post_restore_warmup']['distinct_complete_pairs']==11
    else:
        with pytest.raises(RuntimeError,match='active temporal'):obj.restored_warmup()
        assert 'post_restore_warmup' not in obj.report


def consumer_unit_fixture(monkeypatch,*,foreign=False,mismatch=False,still_running=False):
    owner=dict(pid=40491,start_ticks=101,argv=['/usr/bin/python3','/reviewed/consumer.py'],
        cwd='/reviewed',exe_sha256='reviewed-python')
    alive={'value':True};actions=[]
    helper=NS(identity=lambda pid:copy.deepcopy(owner),alive=lambda identity:alive['value'])
    def wait(predicate,*args):
        if not predicate():raise RuntimeError('owned process did not exit')
    helper.wait=wait;cls=pure_trial_class(helper);obj=cls.__new__(cls)
    obj.verify=lambda:None;obj.guard=lambda:None
    # Lost systemd-run reply: owned process exists although current is still None.
    obj.current={'unknown':None};obj.units={'unknown':'trial-owned-unknown'}
    obj.unit_commands={'unknown':['systemd-run']+owner['argv']}
    obj.before={'unknown':copy.deepcopy(owner)}
    obj.unit_properties=lambda unit:dict(MainPID=str(owner['pid']) if alive['value'] else '0')
    obj.registered=lambda role:90099 if foreign else None
    obj.stop=lambda role:pytest.fail('ambiguous owned unit must be settled explicitly')
    if mismatch:owner['argv']=['/usr/bin/python3','/foreign.py']
    def run(command,**kwargs):
        actions.append(command)
        if not still_running:alive['value']=False
    monkeypatch.setattr(m.subprocess,'run',run)
    return obj,actions,alive


def test_real_quiesce_settles_owned_consumer_after_ambiguous_launch(monkeypatch):
    obj,actions,alive=consumer_unit_fixture(monkeypatch)
    obj.quiesce_consumer('unknown')
    assert actions==[['systemctl','--user','stop','trial-owned-unknown']]
    assert obj.current['unknown'] is None and alive['value'] is False


@pytest.mark.parametrize('fault',['foreign','mismatch','still_running'])
def test_real_quiesce_cannot_hide_foreign_owner_or_unproved_exit(monkeypatch,fault):
    obj,actions,alive=consumer_unit_fixture(monkeypatch,**{fault:True})
    with pytest.raises(RuntimeError):obj.quiesce_consumer('unknown')
    assert alive['value'] is True
    if fault!='still_running':assert actions==[]


@pytest.mark.parametrize('profile',[{}, {'stationary_temporal':None}])
def test_only_unpublished_temporal_diagnostic_is_pending(profile):
    assert m.stationary_policy_ready(profile) is False


@pytest.mark.parametrize('temporal',[{},False,'pending',dict(enabled=False,alpha=.4,delta_depth_units=100,persistence=0),
    dict(enabled=True,alpha=.5,delta_depth_units=100,persistence=0),
    dict(enabled=True,alpha=.4,delta_depth_units=101,persistence=0),
    dict(enabled=True,alpha=.4,delta_depth_units=100,persistence=1),
    dict(enabled=True,alpha=.4,delta_depth_units=100,persistence=False)])
def test_explicit_invalid_temporal_policy_is_not_a_pending_publication(temporal):
    with pytest.raises(RuntimeError,match='policy changed'):
        m.stationary_policy_ready({'stationary_temporal':temporal})


@pytest.mark.parametrize('outcome',['published','missing','invalid','disappeared'])
def test_real_warmup_bounds_wait_for_initial_async_temporal_and_revalidates(monkeypatch,outcome):
    camera=dict(runtime_profile=dict(mode_depth_preset=dict(preset='saved_baseline'),
        geometry_activation_stamp_ns='400'))
    valid=dict(enabled=True,active=True,alpha=.4,delta_depth_units=100,persistence=0)
    checks=[]
    def wait(predicate,seconds,label,guard):
        assert seconds==12.
        checks.append(predicate())
        assert checks==[False]
        if outcome!='missing':
            camera['runtime_profile']['stationary_temporal']=dict(valid,alpha=.5) if outcome=='invalid' else valid
        result=predicate()
        if not result:raise RuntimeError('bounded warmup timeout')
        if outcome=='disappeared':camera['runtime_profile'].pop('stationary_temporal')
        return result
    helper=NS(wait=wait);cls=pure_trial_class(helper);obj=cls.__new__(cls)
    stamps=iter((500,600,601))
    obj.ros=NS(Time=NS(now=lambda:NS(to_nsec=lambda:next(stamps))),get_param=lambda key:camera)
    obj.configuration=dict(mode_depth_preset=dict(unknown_preset='saved_baseline'))
    obj.current=dict(camera=dict(pid=38860));obj.report={};obj.guard=lambda:None
    obj.lock=m.threading.RLock();obj.pairs={stamp:{} for stamp in range(510,521)}
    def final_validate(**kwargs):
        assert kwargs=={'require_temporal':True}
        if not m.stationary_policy_ready(camera['runtime_profile']):
            raise RuntimeError('diagnostic disappeared')
    obj.validate_camera=final_validate
    ticks=iter((0.,4.,4.,4.));monkeypatch.setattr(m,'time',NS(monotonic=lambda:next(ticks)))
    if outcome=='published':
        obj.restored_warmup()
        assert obj.report['post_restore_warmup']['stationary_active_diagnostic']['status']==valid
    else:
        with pytest.raises(RuntimeError):obj.restored_warmup()
        assert 'post_restore_warmup' not in obj.report


@pytest.mark.parametrize('preset',['saved_baseline','high_accuracy'])
def test_real_first_camera_validation_defers_missing_async_status_but_final_requires_it(preset):
    class ValidCameraBase:
        def validate_camera(self):pass  # Existing image/activation checks already succeeded.
    runner=NS(make_runtime_class=lambda *args:ValidCameraBase)
    cls=m.make_runtime(runner,None,None);obj=cls.__new__(cls)
    profile=dict(mode_depth_preset=dict(mode='unknown',preset=preset,
        configured_unknown_preset=preset,baseline_sha256='original-baseline'))
    obj.report=dict(camera_after=dict(runtime_profile=profile),camera_before=dict(
        runtime_profile=dict(mode_depth_preset=dict(baseline_sha256='original-baseline'))))
    obj.configuration=dict(mode_depth_preset=dict(unknown_preset=preset))
    obj.validate_camera()
    assert obj.report['stationary_temporal_initial_diagnostic_pending'] is True
    with pytest.raises(RuntimeError,match='disappeared'):
        obj.validate_camera(require_temporal=True)
    profile['stationary_temporal']=dict(enabled=True,active=True,alpha=.4,delta_depth_units=100,persistence=0)
    obj.validate_camera(require_temporal=True)
    assert obj.report['stationary_temporal_initial_diagnostic_pending'] is False
    profile['stationary_temporal']['enabled']=False
    with pytest.raises(RuntimeError,match='policy changed'):obj.validate_camera()
