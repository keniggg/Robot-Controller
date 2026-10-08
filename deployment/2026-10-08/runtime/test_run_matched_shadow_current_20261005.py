"""Pure preparation/ordering tests; no ROS, SDK, services or process actions."""
import copy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import sys
import pytest

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('current_matched_runner',HERE/'run_matched_shadow_current_20261005.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def test_prepare_default_does_not_load_ros_or_helpers(monkeypatch,capsys):
    monkeypatch.setattr(m,'load',lambda *a:pytest.fail('preparation imported runtime helper'))
    m.main([])
    result=json.loads(capsys.readouterr().out)
    assert result['prepared_only'] and not result['device_actions_performed']
    assert result['lifecycle_roles']==['camera','unknown','remote']
    assert result['maximum_capture_seconds']==75 and 'recovery' in result['time_scope']


def test_execute_requires_current_manifest_before_actions():
    with pytest.raises(SystemExit):m.main(['--execute'])


def fixture_manifest(tmp_path):
    path=tmp_path/'current.py';path.write_text('current source\n')
    return dict(schema=1,scope='matched_shadow_current_owner',reviewed=True,tests_passed=True,
                validation_results=['actual reviewed tests'],files={'current.py':m.sha(path)})


@pytest.mark.parametrize('field,value',[('scope','old_gen13'),('reviewed',False),('tests_passed',False),('validation_results',[])])
def test_manifest_rejects_old_or_unvalidated_claims(tmp_path,field,value):
    doc=fixture_manifest(tmp_path);doc[field]=value
    with pytest.raises(RuntimeError):m.verify_manifest(doc,tmp_path,{'current.py'})


def test_manifest_binds_current_source_and_rejects_escape(tmp_path):
    doc=fixture_manifest(tmp_path);m.verify_manifest(doc,tmp_path,{'current.py'})
    (tmp_path/'current.py').write_text('changed\n')
    with pytest.raises(RuntimeError,match='source changed'):m.verify_manifest(doc,tmp_path,{'current.py'})
    doc['files']={'../else.py':'0'*64}
    with pytest.raises(RuntimeError,match='noncanonical'):m.verify_manifest(doc,tmp_path,set())


def old(role):
    return dict(pid=9001,ppid=4001,start_ticks=7,exe='/usr/bin/python3.8',cwd=str(m.WORK),
        argv=['/usr/bin/python3',str(m.WORK/m.RELAY[role]),'__name:='+m.NODES[role][1:],
              '__log:=/media/zhuyupei/SanStand/old.log'])


@pytest.mark.parametrize('role',m.MUTABLE)
def test_normal_current_entry_accepts_any_bound_pid(role):
    assert m.direct_argv(role,old(role)['argv'])==old(role)['argv']


@pytest.mark.parametrize('change',[
    lambda a:a.__setitem__(0,'/bin/bash'),
    lambda a:a.__setitem__(1,str(m.WORK/m.WRAPPER)),
    lambda a:a.append('--execute'),
    lambda a:a.append('__name:=supervisor_camera'),
    lambda a:a.__setitem__(2,'__name:=other'),
])
def test_unknown_or_nested_current_entries_are_rejected(change):
    argv=old('camera')['argv'];change(argv)
    with pytest.raises(RuntimeError):m.direct_argv('camera',argv)


def test_nonrespawning_launch_proof(tmp_path):
    p=tmp_path/'nodes.launch';p.write_text('<launch><node name="supervisor_camera"/></launch>')
    m.validate_nonrespawning_definition('camera',p)
    p.write_text('<launch><node name="supervisor_camera" respawn="true"/></launch>')
    with pytest.raises(RuntimeError):m.validate_nonrespawning_definition('camera',p)


def test_parent_binds_current_launch_or_exact_nonrestarting_service():
    child=old('camera');parent=dict(pid=4001,exe='/usr/bin/python3.8',argv=['/opt/ros/noetic/bin/roslaunch',str(m.LAUNCH_ROOT)])
    assert 'roslaunch' in m.validate_parent('camera',child,parent)
    parent.update(exe='/lib/systemd/systemd',argv=['/lib/systemd/systemd','--user'])
    assert 'service' in m.validate_parent('camera',child,parent,unit={'MainPID':'9001','Restart':'no'})
    for props in ({'MainPID':'9002','Restart':'no'},{'MainPID':'9001','Restart':'always'},None):
        with pytest.raises(RuntimeError):m.validate_parent('camera',child,parent,unit=props)


@pytest.mark.parametrize('cwd',['/home/zhuyupei/.ros',str(m.WORK)])
def test_current_owner_cwd_uses_reviewed_fact_and_is_preserved(cwd,tmp_path):
    owner=old('camera');owner['cwd']=cwd
    assert m.validate_owner_cwd('camera',owner,{'owner_cwd':{'camera':cwd}})==cwd
    command=m.build_command('camera',owner,{'ROS_HOME':'/home/zhuyupei/.ros'},tmp_path,'unit',('ROS_HOME',))
    assert '--property=WorkingDirectory='+cwd in command
    assert '--setenv=ROS_HOME=/home/zhuyupei/.ros' in command
    for doc in ({},{'owner_cwd':{'camera':'/different'}}):
        with pytest.raises(RuntimeError):m.validate_owner_cwd('camera',owner,doc)


def test_experiment_command_preserves_entry_env_remaps_and_enables_only_shadow(tmp_path):
    command=m.build_command('camera',old('camera'),{'PYTHONPATH':'known','SECRET':'excluded'},tmp_path,'alicia-owned',('PYTHONPATH',),experiment=True)
    assert '--matched-stationary-shadow' in command and '--seconds' in command
    assert command[command.index('--seconds')+1]=='75'
    assert '--property=Restart=no' in command
    assert '--setenv=PYTHONPATH=known' in command and not any('SECRET' in a for a in command)
    assert '__name:=supervisor_camera' in command and '__log:=/media/zhuyupei/SanStand/old.log' not in command
    with pytest.raises(RuntimeError):m.build_command('unknown',old('unknown'),{},tmp_path,'x',(),experiment=True)


def pair_fixture():
    camera=dict(width=653,height=489,frame_id='camera_link',runtime_profile=dict(geometry_activation_stamp_ns='100'))
    pair={k:dict(width=653,height=489,frame_id='camera_link',caller='/supervisor_camera',
        encoding=enc,step=653*bpp,bytes=653*bpp*489) for k,bpp,enc in [('rgb',3,'bgr8'),('depth',2,'16UC1')]}
    return camera,pair


@pytest.mark.parametrize('fault',['old','future','stale','caller','frame','shape','layout','missing'])
def test_warmup_rejects_old_incomplete_or_wrong_pairs(fault):
    camera,pair=pair_fixture();stamp=200;now=300;cutoff=150
    assert m.complete_pair_is_valid(stamp,pair,camera,cutoff,now)
    if fault=='old':stamp=100
    elif fault=='future':stamp=301
    elif fault=='stale':now=800000201
    elif fault=='caller':pair['rgb']['caller']='/other'
    elif fault=='frame':pair['depth']['frame_id']='other'
    elif fault=='shape':pair['depth']['width']=640
    elif fault=='layout':pair['rgb']['bytes']-=1
    else:pair.pop('depth')
    assert not m.complete_pair_is_valid(stamp,pair,camera,cutoff,now)


class FakeIO:
    def __init__(self,fail=None,recovery_fail=False,retire=False):
        self.events=[];self.report={};self.current={r:old(r) for r in m.MUTABLE}
        self.fail=fail;self.failed=False;self.recovery_fail=recovery_fail;self.warmed=False
        self.retire_selection_for_maintenance=retire;self.retirement_completed=False
    def verify(self):self.events.append('verify')
    def stop(self,role):
        self.events.append('stop '+role)
        if self.fail in ('first_stop','remote_stop') and not self.failed and (self.fail=='first_stop' or role=='remote'):
            self.failed=True;raise RuntimeError('consumer stop failed')
        if self.fail=='all_stops':raise RuntimeError('cannot quiesce')
        self.current[role]=None
    def start(self,role,experiment=False):
        self.events.append('start '+role+(' experimental' if experiment else ' normal'))
        if role=='camera':assert self.current['unknown'] is None and self.current['remote'] is None
        else:assert self.warmed
        if self.fail=='camera_ambiguous' and experiment:raise RuntimeError('systemd start reply ambiguous')
        self.current[role]=old(role)
    def validate_camera(self):self.events.append('validate camera')
    def await_experiment(self):
        self.events.append('await experiment')
        if self.fail=='experiment':raise RuntimeError('experiment failed')
        self.report.update(experiment_complete=True,experiment_valid=self.fail!='invalid_capture')
    def clear_camera_validation(self):self.events.append('clear camera');self.warmed=False
    def settle_camera_launch(self):self.events.append('settle camera')
    def restored_warmup(self):
        self.events.append('fresh warmup')
        if self.recovery_fail:raise RuntimeError('camera not fresh')
        self.warmed=True
    def final_validate(self):self.events.append('final');self.report['visual_nodes_restored']=True
    def record(self,event):self.events.append(event)
    def retire_selection(self):
        self.events.append('retire selection')
        assert self.current['unknown'] is None and self.current['remote'] is not None
        if self.fail=='retirement':raise RuntimeError('retirement unconfirmed')
        self.retirement_completed=True


def test_transaction_waits_for_sensor_restore_and_new_warmup_before_consumers():
    io=FakeIO();m.run_transaction(io)
    e=io.events
    assert e.index('stop unknown')<e.index('stop remote')<e.index('stop camera')<e.index('start camera experimental')
    assert e.index('await experiment')<e.index('fresh warmup')<e.index('start unknown normal')<e.index('start remote normal')
    assert io.report['visual_nodes_restored'] and io.report['experiment_valid'] is True


@pytest.mark.parametrize('fault',['experiment','first_stop','remote_stop','camera_ambiguous'])
def test_recovery_quiesces_both_consumers_before_new_camera(fault):
    io=FakeIO(fault)
    with pytest.raises(RuntimeError):m.run_transaction(io)
    assert io.report['normal_camera_recovered_after_error'] and io.report['visual_nodes_restored']
    assert 'start camera normal' in io.events


def test_unprovable_camera_keeps_consumers_quiesced():
    io=FakeIO('experiment',recovery_fail=True)
    with pytest.raises(RuntimeError):m.run_transaction(io)
    assert 'start unknown normal' not in io.events and 'start remote normal' not in io.events
    assert 'consumer_recovery_error' in io.report


def test_invalid_capture_restores_consumers_then_returns_failure_without_second_camera():
    io=FakeIO('invalid_capture')
    with pytest.raises(RuntimeError,match='not validated'):m.run_transaction(io)
    assert io.report['visual_nodes_restored'] and io.report['capture_invalid']
    assert 'start camera normal' not in io.events
    assert io.events.index('fresh warmup')<io.events.index('start unknown normal')


def test_guard_fault_cannot_force_visual_recovery_or_bypass_context():
    io=FakeIO('experiment')
    original=io.verify
    def verify():
        original()
        if 'await experiment' in io.events:raise RuntimeError('guard changed')
    io.verify=verify
    def guarded_stop(role):
        io.verify();FakeIO.stop(io,role)
    io.stop=guarded_stop
    with pytest.raises(RuntimeError):m.run_transaction(io)
    assert 'start camera normal' not in io.events and 'start unknown normal' not in io.events


def test_failed_consumer_quiescence_does_not_restart_camera():
    io=FakeIO('all_stops')
    with pytest.raises(RuntimeError):m.run_transaction(io)
    assert not any(x.startswith('start camera') for x in io.events)


def ambiguous_runtime(monkeypatch,*,foreign=False,mismatch=False,known=False):
    """Exercise real settle method with fake identity/registration/service IO."""
    methods={name:lambda *a:None for name in ('latch','sdk_cb','epoch_cb','task_cb','image_cb')}
    loader=SimpleNamespace(Runtime=SimpleNamespace(**methods))
    camera=old('camera');camera['exe_sha256']='reviewed-python'
    running={'alive':True};actions=[]
    helper=SimpleNamespace(alive=lambda identity:running['alive'],identity=lambda pid:dict(camera))
    def wait(predicate,*args):
        assert predicate();return True
    helper.wait=wait
    runtime_class=m.make_runtime_class(loader,helper,None)
    obj=runtime_class.__new__(runtime_class)
    obj.current={'camera':camera if known else None}
    obj.before={'camera':dict(camera)};obj.last_known={'camera':dict(camera)}
    obj.units={'camera':'alicia-owned'};obj.unit_commands={'camera':['systemd-run']+camera['argv']}
    if mismatch:camera['argv']=['/usr/bin/python3','/unowned.py']
    obj.verify=lambda:actions.append('verify');obj.guard=lambda:None
    obj.registered=lambda role:(9999 if foreign else camera['pid']) if running['alive'] else None
    obj.unit_properties=lambda unit:{'MainPID':str(camera['pid']) if running['alive'] else '0'}
    def run(command,**kwargs):
        assert command==['systemctl','--user','stop','alicia-owned']
        actions.append('stop owned unit');running['alive']=False
    monkeypatch.setattr(m.subprocess,'run',run)
    return obj,actions,running


def test_real_settle_recovers_registered_owned_unit_after_ambiguous_start(monkeypatch):
    obj,actions,running=ambiguous_runtime(monkeypatch)
    obj.settle_camera_launch()
    assert actions==['verify','stop owned unit'] and not running['alive']
    assert obj.current['camera'] is None and obj.last_known['camera']['pid']==9001


@pytest.mark.parametrize('fault',['foreign','mismatch'])
def test_real_settle_refuses_foreign_registration_or_changed_identity(monkeypatch,fault):
    obj,actions,running=ambiguous_runtime(monkeypatch,**{fault:True})
    with pytest.raises(RuntimeError):obj.settle_camera_launch()
    assert actions==['verify'] and running['alive']


def test_real_settle_leaves_known_live_camera_for_normal_verified_stop(monkeypatch):
    obj,actions,running=ambiguous_runtime(monkeypatch,known=True)
    obj.settle_camera_launch()
    assert actions==[] and running['alive']


def registration_runtime(monkeypatch):
    """Exact endpoint/PID/start identities, with no network or process access."""
    methods={name:lambda *a:None for name in ('latch','sdk_cb','epoch_cb','task_cb','image_cb')}
    loader=SimpleNamespace(Runtime=SimpleNamespace(**methods))
    previous=old('camera');previous['exe_sha256']='reviewed-python'
    replacement=dict(previous,pid=38860,start_ticks=88)
    live={(previous['pid'],previous['start_ticks'])}
    helper=SimpleNamespace(alive=lambda owner:(owner['pid'],owner['start_ticks']) in live,
                           identity=lambda pid:copy.deepcopy(replacement))
    cls=m.make_runtime_class(loader,helper,None);obj=cls.__new__(cls)
    endpoint={'uri':'http://host:40001/','pid':previous['pid'],'error':None,'after_uri':None}
    obj.ros=SimpleNamespace()
    obj.current={'camera':copy.deepcopy(previous)};obj.last_known={'camera':copy.deepcopy(previous)}
    obj.before={'camera':copy.deepcopy(previous)};obj.registrations={}
    def lookup(ros,name):
        assert name==m.NODES['camera'];return endpoint['uri']
    def query(ros,uri,identity_helper):
        assert uri==endpoint['uri']
        if endpoint['after_uri'] is not None:endpoint['uri']=endpoint['after_uri']
        if endpoint['error'] is not None:raise endpoint['error']
        return endpoint['pid']
    monkeypatch.setattr(m,'registration_uri',lookup)
    monkeypatch.setattr(m,'registered_pid_at_uri',query)
    assert obj.registered('camera')==previous['pid']
    return obj,helper,previous,replacement,live,endpoint


def test_exact_dead_old_uri_remains_pending_after_expected_new_process_starts(monkeypatch):
    obj,helper,previous,replacement,live,endpoint=registration_runtime(monkeypatch)
    live.remove((previous['pid'],previous['start_ticks']))
    live.add((replacement['pid'],replacement['start_ticks']))
    obj.current['camera']=replacement
    endpoint['error']=ConnectionRefusedError('old ROS XML-RPC socket is closed')
    assert obj.registered('camera') is None
    assert obj.last_known['camera']==previous
    assert obj.registrations['camera']['identity']==previous
    endpoint.update(uri='http://host:40002/',pid=replacement['pid'],error=None)
    assert obj.registered('camera')==replacement['pid']
    assert obj.last_known['camera']==replacement
    assert obj.registrations['camera']['uri']=='http://host:40002/'


def test_dead_bound_current_owner_can_be_retired_before_start(monkeypatch):
    obj,helper,previous,replacement,live,endpoint=registration_runtime(monkeypatch)
    live.clear();endpoint['error']=ConnectionRefusedError('bound owner exited')
    # Still current during stop()'s proof that an already exited owner is gone.
    assert obj.registered('camera') is None
    obj.current['camera']=None
    assert obj.registered('camera') is None


def test_pid_reuse_does_not_make_dead_original_identity_live(monkeypatch):
    obj,helper,previous,replacement,live,endpoint=registration_runtime(monkeypatch)
    live.clear();replacement['pid']=previous['pid']
    live.add((replacement['pid'],replacement['start_ticks']))
    obj.current['camera']=replacement
    endpoint['error']=ConnectionRefusedError('old URI after PID reuse')
    assert obj.registered('camera') is None
    assert obj.registrations['camera']['identity']['start_ticks']==previous['start_ticks']


@pytest.mark.parametrize('fault',['old_alive','foreign_refused','timeout','pending_owner_dead',
    'unbound_uri','changed_during_refusal','changed_during_pid'])
def test_pending_registration_never_swallows_unproved_or_changed_endpoints(monkeypatch,fault):
    obj,helper,previous,replacement,live,endpoint=registration_runtime(monkeypatch)
    live.discard((previous['pid'],previous['start_ticks']))
    live.add((replacement['pid'],replacement['start_ticks']))
    obj.current['camera']=replacement
    endpoint['error']=ConnectionRefusedError('refused')
    if fault=='old_alive':live.add((previous['pid'],previous['start_ticks']))
    elif fault=='foreign_refused':endpoint['uri']='http://foreign:5555/'
    elif fault=='timeout':endpoint['error']=TimeoutError('timeout')
    elif fault=='pending_owner_dead':live.remove((replacement['pid'],replacement['start_ticks']))
    elif fault=='unbound_uri':obj.registrations.clear()
    elif fault=='changed_during_refusal':endpoint['after_uri']='http://foreign:5555/'
    elif fault=='changed_during_pid':
        endpoint.update(error=None,pid=replacement['pid'],after_uri='http://foreign:5555/')
    with pytest.raises((ConnectionRefusedError,TimeoutError,RuntimeError)):
        obj.registered('camera')
    assert obj.last_known['camera']==previous


def test_foreign_live_registration_is_returned_and_never_promoted(monkeypatch):
    obj,helper,previous,replacement,live,endpoint=registration_runtime(monkeypatch)
    obj.current['camera']=replacement
    live.add((replacement['pid'],replacement['start_ticks']))
    endpoint.update(uri='http://foreign:5555/',pid=90099)
    assert obj.registered('camera')==90099
    assert obj.last_known['camera']==previous


def test_start_waits_through_dead_old_registration_without_relabeling_it(monkeypatch,tmp_path):
    obj,helper,previous,replacement,live,endpoint=registration_runtime(monkeypatch)
    live.remove((previous['pid'],previous['start_ticks']))
    obj.current={'camera':None,'unknown':None,'remote':None}
    obj.retire_selection_for_maintenance=False;obj.retirement_completed=False
    obj.units={};obj.unit_commands={};obj.environments={'camera':{}};obj.allowed_env=()
    obj.out=tmp_path;obj.verify=lambda:None;obj.guard=lambda:None;obj.record=lambda *a,**k:None
    obj.ros.Time=SimpleNamespace(now=lambda:SimpleNamespace(to_nsec=lambda:12345))
    obj.unit_properties=lambda unit:{'MainPID':str(replacement['pid'])}
    original_registered=obj.registered
    obj.registered=lambda role:original_registered(role) if role=='camera' else None
    endpoint['error']=ConnectionRefusedError('dead previous endpoint')
    def run(command,**kwargs):
        replacement['argv']=command[command.index('/usr/bin/python3'):]
        live.add((replacement['pid'],replacement['start_ticks']))
    monkeypatch.setattr(m.subprocess,'run',run)
    def wait(predicate,seconds,label,guard):
        assert not predicate()
        assert obj.current['camera']['pid']==38860
        assert obj.last_known['camera']==previous
        endpoint.update(uri='http://host:40002/',pid=replacement['pid'],error=None)
        assert predicate()['pid']==38860
    helper.wait=wait
    obj.start('camera',experiment=True)
    assert obj.last_known['camera']['pid']==38860


def test_start_rejects_foreign_live_owner_instead_of_waiting_to_timeout(monkeypatch,tmp_path):
    obj,helper,previous,replacement,live,endpoint=registration_runtime(monkeypatch)
    live.remove((previous['pid'],previous['start_ticks']))
    obj.current={'camera':None,'unknown':None,'remote':None}
    obj.retire_selection_for_maintenance=False;obj.retirement_completed=False
    obj.units={};obj.unit_commands={};obj.environments={'camera':{}};obj.allowed_env=()
    obj.out=tmp_path;obj.verify=lambda:None;obj.guard=lambda:None;obj.record=lambda *a,**k:None
    obj.ros.Time=SimpleNamespace(now=lambda:SimpleNamespace(to_nsec=lambda:12345))
    obj.unit_properties=lambda unit:{'MainPID':str(replacement['pid'])}
    original_registered=obj.registered
    obj.registered=lambda role:original_registered(role) if role=='camera' else None
    endpoint['error']=ConnectionRefusedError('dead previous endpoint')
    def run(command,**kwargs):
        replacement['argv']=command[command.index('/usr/bin/python3'):]
        live.add((replacement['pid'],replacement['start_ticks']))
        endpoint.update(uri='http://foreign:5555/',pid=99999,error=None)
    monkeypatch.setattr(m.subprocess,'run',run)
    helper.wait=lambda predicate,*a:predicate()
    with pytest.raises(RuntimeError,match='foreign owner'):
        obj.start('camera',experiment=True)
    assert obj.last_known['camera']==previous


@pytest.mark.parametrize('reply',[(0,'master unavailable',''),(1,'ok',None),(1,'ok','file:///tmp/node')])
def test_registration_lookup_does_not_treat_master_errors_or_bad_uri_as_absent(reply):
    ros=SimpleNamespace(get_master=lambda:SimpleNamespace(lookupNode=lambda name:reply))
    with pytest.raises(RuntimeError):m.registration_uri(ros,'/supervisor_camera')


def test_registration_lookup_only_known_unknown_node_is_absent():
    ros=SimpleNamespace(get_master=lambda:SimpleNamespace(
        lookupNode=lambda name:(-1,'unknown node ['+name+']','')))
    assert m.registration_uri(ros,'/supervisor_camera') is None


@pytest.mark.parametrize('reply',[(1,'ok',38860),(1,'ok',True),(1,'ok',0),(-1,'failed',38860)])
def test_pid_probe_uses_exact_captured_uri_and_validates_reply(monkeypatch,reply):
    requests=[];transport=object()
    def proxy(uri,transport):
        requests.append(uri)
        return SimpleNamespace(getPid=lambda caller:reply)
    monkeypatch.setattr(m.xmlrpc.client,'ServerProxy',proxy)
    ros=SimpleNamespace(get_name=lambda:'/read_only_runner')
    helper=SimpleNamespace(Transport=lambda:transport)
    if type(reply[2]) is int and reply[2]>0 and reply[0]==1:
        assert m.registered_pid_at_uri(ros,'http://bound:40001/',helper)==38860
    else:
        with pytest.raises(RuntimeError):m.registered_pid_at_uri(ros,'http://bound:40001/',helper)
    assert requests==['http://bound:40001/']


def test_sources_do_not_publish_robot_commands_or_set_parameters():
    source=(HERE/'run_matched_shadow_current_20261005.py').read_text()
    assert '.set_param(' not in source and '.Publisher(' not in source
    assert source.count('.ServiceProxy(')==1
    assert "ServiceProxy('/grasp_mode/select',SetGraspMode)('unknown','observe_return')" in source
    assert 'EXPECTED_PIDS' not in source and 'PREVIOUS =' not in source
    assert 'Runtime.__init__' not in source


def test_explicit_retirement_occurs_after_unknown_exit_before_remote_and_camera():
    io=FakeIO(retire=True);m.run_transaction(io)
    assert io.events.index('stop unknown')<io.events.index('retire selection')<io.events.index('stop remote')
    assert io.events.index('stop remote')<io.events.index('stop camera')
    assert io.retirement_completed and io.report['visual_nodes_restored']


@pytest.mark.parametrize('fault',['first_stop','retirement'])
def test_unconfirmed_retirement_never_stops_camera_remote_or_revives_old_tracker(fault):
    io=FakeIO(fault,retire=True)
    with pytest.raises(RuntimeError):m.run_transaction(io)
    assert not any(e.startswith('start ') for e in io.events)
    assert 'stop remote' not in io.events and 'stop camera' not in io.events
    assert io.report['maintenance_retirement_incomplete_consumers_not_restarted']


def test_after_retirement_failure_can_restore_visuals_only_with_new_selection():
    io=FakeIO('experiment',retire=True)
    with pytest.raises(RuntimeError):m.run_transaction(io)
    assert io.retirement_completed and io.report['visual_nodes_restored']
    assert io.events.count('retire selection')==1


def selection(generation=3,stamp=100):
    return dict(mode='unknown',strategy='observe_return',generation=generation,stamp_ns=stamp)


def mode(selection_value,state='waiting_for_target',**values):
    return dict(selection_value,state=state,active=False,start_reserved=False,**values)


def test_successor_and_reset_receipts_require_exact_new_generation_and_idle_ack():
    before=selection();after=selection(4,101)
    assert m.retirement_successor(before,dict(after,stamp_ns='101'))==after
    assert m.mode_reset_complete(mode(after),after)
    assert not m.mode_reset_complete(mode(before),after)
    assert not m.mode_reset_complete(mode(after,state='switching'),after)
    for field in ('active','start_reserved'):
        status=mode(after);status[field]=True
        assert not m.mode_reset_complete(status,after)
    for wrong in (before,selection(5,101),selection(4,100),dict(after,strategy='direct'),dict(after,generation=True)):
        with pytest.raises(RuntimeError):m.retirement_successor(before,wrong)


def test_transition_guard_cannot_erase_successor_or_accept_different_successor():
    before=selection();after=selection(4,101)
    assert m.retirement_transition_guard(before,None,before) is None
    assert m.retirement_transition_guard(before,None,after)==after
    assert m.retirement_transition_guard(before,after,dict(after,stamp_ns='101'))==after
    for actual in (before,selection(4,102),selection(5,103),dict(after,strategy='direct')):
        with pytest.raises(RuntimeError):m.retirement_transition_guard(before,after,actual)


def test_prepare_retirement_option_does_not_create_authority_or_call_services(monkeypatch,capsys):
    monkeypatch.setattr(m,'load',lambda *a:pytest.fail('preparation imported runtime helper'))
    m.main(['--retire-selection-for-maintenance'])
    result=json.loads(capsys.readouterr().out)
    assert result['maintenance_retirement_requested'] and result['prepared_only']
    assert result['new_user_alignment_confirmed'] is False
    assert result['new_motion_authority_created'] is False
    assert not result['device_actions_performed']


def closed_record():
    return (dict(runner_finished=True,evidence=dict(runner_returncode=3)),
        dict(selection=selection(),events=[dict(phase='metric_runner_finished',returncode=3)]),
        dict(state='complete',recorder_closed=True),dict(bag_closed=True))


@pytest.mark.parametrize('fault',['runner_live','wrong_generation','not_sealed','recorder_active','no_finish','wrong_result'])
def test_retirement_requires_closed_current_attempt(fault):
    command,dispatch,manifest,closure=closed_record()
    assert m.validate_retirement_record(selection(),command,dispatch,manifest,closure)==selection()
    if fault=='runner_live':command['runner_finished']=False
    elif fault=='wrong_generation':dispatch['selection']=selection(2)
    elif fault=='not_sealed':manifest['state']='recording'
    elif fault=='recorder_active':closure['bag_closed']=False
    elif fault=='no_finish':dispatch['events']=[]
    elif fault=='wrong_result':dispatch['events'][0]['returncode']=0
    with pytest.raises(RuntimeError):m.validate_retirement_record(selection(),command,dispatch,manifest,closure)


def retirement_runtime(monkeypatch,fault=None):
    """Run real retirement method using fake ROS service/parameter and owner IO."""
    methods={name:lambda *a:None for name in ('latch','sdk_cb','epoch_cb','task_cb','image_cb')}
    loader=SimpleNamespace(Runtime=SimpleNamespace(**methods));events=[]
    helper=SimpleNamespace(alive=lambda owner:True)
    def wait(predicate,seconds,label,guard=lambda:None):
        guard();answer=predicate()
        if not answer:raise RuntimeError('timeout '+label)
        return answer
    helper.wait=wait
    cls=m.make_runtime_class(loader,helper,None);obj=cls.__new__(cls)
    obj.retire_selection_for_maintenance=True;obj.retirement_completed=False
    obj.selection_transition=None;obj.retirement_successor_seen=None
    obj.context={'/grasp_mode/selection':selection()}
    obj.current={'unknown':None,'remote':old('remote'),'camera':old('camera')}
    obj.registered=lambda role:None if obj.current[role] is None else obj.current[role]['pid']
    obj.mode_status=mode(selection());obj.report={};actual=[selection()]
    obj.record=lambda event,**kwargs:events.append(event)
    obj.verify=lambda:events.append('verify')
    obj.guard=lambda:events.append('guard')
    service_type=object()
    monkeypatch.setitem(sys.modules,'alicia_grasp_modes.srv',SimpleNamespace(SetGraspMode=service_type))
    def proxy(name,kind):
        assert name=='/grasp_mode/select' and kind is service_type
        def call(mode_name,strategy):
            assert obj.current['unknown'] is None and obj.current['remote'] is not None
            assert (mode_name,strategy)==('unknown','observe_return')
            events.append('service')
            if fault=='transport':raise RuntimeError('response unknown')
            if fault=='reject':return SimpleNamespace(success=False,message='rejected')
            actual[0]=selection(5 if fault=='skipped_generation' else 4,101)
            obj.mode_status=mode(actual[0],state='switching' if fault=='missing_ack' else 'waiting_for_target')
            return SimpleNamespace(success=True,message='SWITCHING')
        return call
    obj.ros=SimpleNamespace(ServiceProxy=proxy,get_param=lambda name:copy.deepcopy(actual[0]))
    return obj,events


def test_real_retirement_rebinds_guard_only_after_exact_successor_and_reset(monkeypatch):
    obj,events=retirement_runtime(monkeypatch)
    obj.retire_selection()
    assert obj.retirement_completed and obj.context['/grasp_mode/selection']==selection(4,101)
    assert obj.selection_transition is None
    assert obj.report['maintenance_retirement']['new_user_alignment_confirmed'] is False
    assert obj.report['maintenance_retirement']['motion_authorized'] is False
    assert events.count('service')==1
    with pytest.raises(RuntimeError):obj.retire_selection()
    assert events.count('service')==1


@pytest.mark.parametrize('fault',['reject','transport','skipped_generation','missing_ack','unknown_alive','remote_gone','reserved'])
def test_real_retirement_failure_is_not_completion_or_old_tracker_restart(monkeypatch,fault):
    obj,events=retirement_runtime(monkeypatch,fault)
    if fault=='unknown_alive':obj.current['unknown']=old('unknown')
    elif fault=='remote_gone':obj.current['remote']=None
    elif fault=='reserved':obj.mode_status['start_reserved']=True
    with pytest.raises(RuntimeError):obj.retire_selection()
    assert not obj.retirement_completed
    assert events.count('service')<=1
