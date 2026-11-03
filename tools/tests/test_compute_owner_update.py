"""Pure contract tests. No ROS, process control, source installation or motion."""
import ast
from copy import deepcopy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest

TOOLS=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(TOOLS))
import compute_owner_update as update


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,ensure_ascii=False,sort_keys=True))
    return dict(path=str(path),sha256=digest(path))


@pytest.fixture
def contract(tmp_path,monkeypatch):
    root=tmp_path/'workspace';candidate=tmp_path/'candidate';history=tmp_path/'history'
    root.mkdir();candidate.mkdir();history.mkdir()
    p='src/alicia_grasp_modes/src/alicia_grasp_modes/measurement.py'
    test='src/alicia_grasp_modes/tests/test_measurement.py'
    def source(base,name,body):
        path=base/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_text(body);return digest(path)
    old={p:source(root,p,'old = 1\n')}
    changed={p:source(candidate,p,'new = 2\n'),test:source(candidate,test,'assert True\n')}
    load={'devel/relay.py':source(root,'devel/relay.py','original_relay = 1\n')}
    primitive=root/'primitive.py';primitive.write_text('ANSWER=42\n')
    module_ref=dict(path=str(primitive),sha256=digest(primitive))
    nodes={n:dict(pid=i+100,parent_pid=50,start_ticks=str(i+1000)) for i,n in enumerate(
        list(update.TARGETS)+['/motion_gateway']+['/retained_'+str(i) for i in range(13)])}
    preserved={n:r for n,r in nodes.items() if n not in update.TARGETS}
    targets={n:nodes[n] for n in update.TARGETS};units={n:'compute-'+n[1:] for n in update.TARGETS}
    previous=dict(mode='unknown',strategy='observe_return',generation=11,stamp_ns=20)
    selected=dict(previous,generation=12,stamp_ns=30)
    control=dict(state='active',revision=73,authorization_id='auth')
    client=root/'client.py';client.write_text('CLIENT=3\n')
    config=dict(nodes={n:dict(respawn=False,required=False) for n in update.TARGETS},
        remote_parameters={'server':'test'},launch_source_sha256={str(primitive):digest(primitive)},
        client_contract_source=dict(path=str(client),sha256=digest(client)))
    values={}
    def put(name,value):
        values[name]=value;return write(history/(name+'.json'),value)
    full=put('baseline_full',dict(passed=True,full_regression_passed=True,source_sha256=old,
        summary=dict(tests=10213,failures=0,errors=0,skipped=0)))
    config_ref=put('startup_configuration',config)
    startup=put('startup',dict(startup_configuration=config_ref['path'],startup_configuration_sha256=config_ref['sha256'],boot_id='boot'))
    core_service={'MainPID':'50'};sim={'MainPID':'55'}
    graph=dict(node_identities=nodes,core_service=core_service,simulation_service=sim)
    core=put('core_reference',dict(core_service=core_service,simulation_service=sim))
    design=put('gateway_service_design',dict(constructor_review=dict(registered_service_names=['/service_'+str(i) for i in range(31)])))
    bundle=put('baseline_bundle',dict(source_sha256=old,preserved=preserved,entries=update.ENTRIES,
        load_resolution_sha256=load,cached_parameter_cleanup_keys=['/old/key']))
    construction=put('baseline_construction',dict(bundle=bundle['path'],bundle_sha256=bundle['sha256'],source_sha256=old,
        processes={n:{} for n in update.TARGETS},remote_parameters=config['remote_parameters']))
    reload=put('baseline_reload',dict(passed=True,bundle=bundle['path'],bundle_sha256=bundle['sha256'],
        construction=construction['path'],construction_sha256=construction['sha256'],targets=targets,units=units,after=graph))
    accepted=put('baseline_acceptance',dict(deployment_passed=True,reload=reload['path'],reload_sha256=reload['sha256'],
        bundle=bundle['path'],bundle_sha256=bundle['sha256'],source_sha256=old,full_runtime_nodes=nodes,
        targets=targets,preserved=preserved,before=graph,after=graph,selection=previous,control_snapshot=control))
    release=put('baseline_release',dict(passed=True,installed=True,acceptance=accepted,source_sha256=old,
        actual_regression=full['path'],actual_regression_sha256=full['sha256'],test_summary=values['baseline_full']['summary']))
    exited=put('exited',dict(selection=selected,runner_returncode=6))
    authorization=put('attempt_authorization',dict(selection=selected,previous_selection=previous,operator_authorized_autonomous_grasp=True))
    record=put('record',dict(state='complete',recorder_closed=True,record_id='record_1'))
    post=put('post_attempt',dict(passed=True,accepted_processes_unchanged=True,source_manifest=release['path'],
        source_manifest_sha256=release['sha256'],deployment=accepted['path'],deployment_sha256=accepted['sha256'],
        full_runtime_nodes_before=nodes,full_runtime_nodes_after=nodes,epoch_ns=40,selection=selected,
        prior_accepted_selection=previous,gui_own_manual_ack=True,task_active=False,actions_active=False,
        sample_count=20,actuation_status='CONFIRMED:current',original_runner_exit=exited,
        selection_authorization=authorization,closed_record_manifest=record,record_dir=str(tmp_path/'record_1')))
    focused=put('targeted',dict(passed=True,full_regression_passed=False,exact_case_inventory=True,
        duplicate_cases=[],omitted_cases=[],production_sha256={p:changed[p]},current_test_sha256={test:changed[test]},
        case_results=[dict(identity=[test,'case','accept'],passed=True)],total_distinct_case_count=1))
    review=put('review',dict(passed=True,blocking_items=[],candidate_source_sha256={str(candidate/p):changed[p]}))
    adoption=put('adoption',dict(source_map=[dict(path=p,baseline_sha256=old[p],candidate_sha256=changed[p])],
        affected_processes={n:{} for n in update.TARGETS},preserved_instance_names=list(preserved),preserved_instance_count=14,
        gateway_compatibility=dict(require_gateway_restart_for_this_patch=False,memory_attestation=False)))
    refs={name:dict(path=str(history/(name+'.json')),sha256=digest(history/(name+'.json'))) for name in update.REFS}
    auth=write(tmp_path/'active_authorization.json',dict(version=1,scope='observe_return_until_physical_grasp_success',
        confirmation_source=dict(source='actual_user_message'),authorization_id='auth'))
    ctl=write(tmp_path/'active_control.json',control)
    archive=write(tmp_path/'archive.json',dict(upload_enabled=False,delete_enabled=False,min_free_bytes=0))
    storage=write(tmp_path/'storage.json',dict(archive_policy='local_only'))
    starter=dict(path=str(TOOLS/'compute_owner_start.py'),sha256=digest(TOOLS/'compute_owner_start.py'))
    current={r['path']:r['sha256'] for r in (auth,ctl,archive,storage,starter,module_ref)}
    current.update({str(TOOLS/n):digest(TOOLS/n) for n in ('compute_owner_update.py','runtime_evidence.py')})
    d=dict(schema=1,kind='three_compute_owner_update',release_label='named-update',workspace_root=str(root),
        candidate_root=str(candidate),storage_root=str(tmp_path),output_root=str(tmp_path/'outputs'),
        reference=dict(epoch_ns=40,selection=selected),refs=refs,authorization=auth,control=ctl,
        archive_config=archive,storage_control=storage,starter=starter,control_events=[],current_sha256=current,
        historical_sha256={str(path):digest(path) for path in history.glob('*.json')},
        modules={role:module_ref for role in update.ROLES},module_dependencies=[],
        source_update=dict(production=[p],tests=[test],candidate_sha256=changed))
    path=tmp_path/'contract.json'
    def save():
        write(path,d);return path,digest(path)
    def replace_artifact(name,change):
        item=values[name];change(item)
        ref=put(name,item)
        if name in refs:d['refs'][name]=ref
        d['historical_sha256'][ref['path']]=ref['sha256']
    monkeypatch.setattr(Path,'is_mount',lambda self:self==tmp_path)
    return dict(data=d,save=save,replace=replace_artifact,values=values,root=root,candidate=candidate,
        production=p,test=test,path=path)


def test_data_reference_and_scoped_test_evidence(contract):
    with update.UpdateTransaction(*contract['save']()) as session:
        session.recheck()
        result=session.summary()
        assert result['current_targeted_cases']==1
        assert result['candidate_full_regression_passed'] is False
        assert session.expected_selection['generation']==12 and session.expected_epoch==40
        assert result['replaced_nodes']==list(update.TARGETS) and len(result['preserved_nodes'])==14


@pytest.mark.parametrize('case',['epoch','selection','source','targeted_full','case_duplicate','case_failed','review','gateway','retained','record','gui'])
def test_cross_artifact_rejection(contract,case):
    d=contract['data'];replace=contract['replace']
    if case=='epoch':d['reference']['epoch_ns']+=1
    elif case=='selection':d['reference']['selection']=dict(d['reference']['selection'],generation=99)
    elif case=='source':d['source_update']['candidate_sha256'][contract['production']]='0'*64
    elif case=='targeted_full':replace('targeted',lambda x:x.update(full_regression_passed=True))
    elif case=='case_duplicate':replace('targeted',lambda x:x['case_results'].append(deepcopy(x['case_results'][0])))
    elif case=='case_failed':replace('targeted',lambda x:x['case_results'][0].update(passed=False))
    elif case=='review':replace('review',lambda x:x.update(blocking_items=['blocked']))
    elif case=='gateway':replace('adoption',lambda x:x['gateway_compatibility'].update(require_gateway_restart_for_this_patch=True))
    elif case=='retained':replace('adoption',lambda x:x.update(preserved_instance_count=13))
    elif case=='record':replace('post_attempt',lambda x:x.update(actions_active=True))
    elif case=='gui':replace('post_attempt',lambda x:x.update(gui_own_manual_ack=False))
    with pytest.raises(RuntimeError):update.UpdateTransaction(*contract['save']())


@pytest.mark.parametrize('case',['control','authorization','source','candidate','module','config'])
def test_live_inputs_rechecked_even_same_timestamp(contract,case):
    with update.UpdateTransaction(*contract['save']()) as session:
        session.recheck()
        paths=dict(control=Path(session.data['control']['path']),authorization=Path(session.data['authorization']['path']),
            source=contract['root']/contract['production'],candidate=contract['candidate']/contract['production'],
            module=contract['root']/'primitive.py',config=contract['root']/'client.py')
        path=paths[case];st=path.stat();raw=path.read_bytes();path.write_bytes(raw[:-1]+(b'!' if raw[-1:]!=b'!' else b'?'))
        os.utime(path,ns=(st.st_atime_ns,st.st_mtime_ns))
        with pytest.raises(RuntimeError,match='current input changed'):session.recheck()


def test_historical_snapshot_once_per_transaction(contract):
    args=contract['save']()
    with update.UpdateTransaction(*args) as session:
        session.recheck();count=session.transaction.stats['read_attempts']
        session.recheck()
        assert session.transaction.stats['read_attempts']==count
        path=Path(session.data['refs']['targeted']['path']);path.write_text('{}')
        session.recheck()
    with pytest.raises(ValueError,match='HISTORICAL_EVIDENCE_HASH_MISMATCH'):update.UpdateTransaction(*args)


def test_current_path_cannot_be_hidden_in_history(contract):
    ref=contract['data']['control'];contract['data']['historical_sha256'][ref['path']]=ref['sha256']
    with pytest.raises(ValueError,match='CURRENT_INPUT_CANNOT_BE_CACHED'):update.UpdateTransaction(*contract['save']())


def test_rebinding_keeps_original_function_code_and_does_not_change_original():
    module=type(sys)('test_primitive');exec('VALUE=1\ndef call():\n return VALUE\n',module.__dict__)
    bound=update.rebind(module,VALUE=7)
    assert bound.call.__code__ is module.call.__code__
    assert bound.call()==7 and module.call()==1


def test_original_sampling_gates_have_only_documented_data_substitutions():
    original=next(parent/'.runtime/gen60_gateway_reload_common_20261022.py'
        for parent in TOOLS.parents
        if (parent/'.runtime/gen60_gateway_reload_common_20261022.py').is_file())
    source=original.read_text();source=source[source.index('def sample_idle('):source.index('\n\nDESIGN=')]
    changes=[("require(not states['task'].active and","require((task_absent or not states['task'].active) and"),
        ("selection['generation']==3","selection['generation']==expected_selection['generation']"),
        ("selection==SELECTION and (expected_selection is None or selection==expected_selection)","selection==expected_selection"),
        ("epoch == (ctx['previous']['epoch_ns'] if expected_epoch is None else expected_epoch)","epoch == expected_epoch")]
    for before,after in changes:
        assert source.count(before)==1;source=source.replace(before,after)
    actual=next(n for n in ast.parse(Path(update.__file__).read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='sample_idle')
    def gates(tree):
        return {ast.dump(node,include_attributes=False) for node in ast.walk(tree)
            if isinstance(node,ast.Call) and isinstance(node.func,ast.Name) and node.func.id=='require'}
    # Every original threshold/ownership/layout/configuration gate remains.
    assert gates(ast.parse(source)) <= gates(actual)


@pytest.fixture
def feedback(contract,monkeypatch):
    now=[10_000_000_000];epoch=1_000_000_000
    driver=dict(pid=200,parent_pid=100,start_ticks='500')
    stamp=lambda n:SimpleNamespace(to_nsec=lambda:n)
    owners=dict(joint='/alicia_d_driver_node',epoch='/alicia_d_driver_node',task='/grasp_task_node',
        mode='/grasp_mode_router',gui='/alicia_supervisor_gui',actuation='/alicia_d_driver_node',
        motion='/alicia_d_driver_node',feedback='/alicia_d_driver_node',arm='/bessica_d_hw_interface',hand='/bessica_d_hw_interface')
    messages={key:SimpleNamespace(_connection_header=dict(callerid=owner,latching='1')) for key,owner in owners.items()}
    messages['epoch'].stamp=stamp(epoch)
    joint=messages['joint'];joint.header=SimpleNamespace(stamp=stamp(now[0]-1_000_000),frame_id='sdk_measured')
    joint.name=['Joint'+str(i) for i in range(1,7)];joint.position=[0.]*6
    messages['task'].active=False;messages['task'].state='IDLE'
    messages['mode'].data=json.dumps(dict(active=False,start_reserved=False,pending=False))
    for key in ('gui','motion','feedback'):messages[key].data=True
    messages['actuation'].data='CONFIRMED:ready'
    for key in ('arm','hand'):
        messages[key].header=SimpleNamespace(stamp=stamp(now[0]-1_000_000));messages[key].status_list=[]
    window=update.IdleWindow(driver,epoch)
    for key,msg in messages.items():window.receive(key,msg,now[0])
    window.history=[]
    for index in range(20):
        row=deepcopy(joint);source=now[0]-1_000_000-(19-index)*10_000_000
        row.header.stamp=stamp(source);window.receive('joint',row,source+500_000)
    window.values['joint']=(joint,now[0])
    selected=dict(mode='unknown',strategy='observe_return',generation=12,stamp_ns=99)
    params={'/gui/joint_direct_mode':True,'/grasp/require_actuation_confirmation':True,
        '/alicia_d_driver_node/software_temperature_monitoring_enabled':False,
        '/alicia_d_driver_node/temperature_poll_rate_hz':0.,'/grasp/unknown_lift_after_close_enabled':False,
        '/grasp/unknown_observe_return_max_observations':1,'/grasp_mode/selection':selected}
    topics=dict(joint='/alicia_d/accepted_joint_states',epoch='/alicia_d/control_reference_epoch',task='/grasp/state',
        mode='/grasp_mode/status',gui='/gui/joint_direct_mode',actuation='/alicia_d/actuation_status',
        motion='/alicia_d/motion_enabled',feedback='/alicia_d/feedback_ready',
        arm='/alicia_controller/follow_joint_trajectory/status',hand='/hand_controller/follow_joint_trajectory/status')
    by_topic={topic:messages[key] for key,topic in topics.items()}
    master=SimpleNamespace(getSystemState=lambda:([ (topics[k],['/bessica_d_hw_interface']) for k in ('arm','hand')],[],
        [('/controller_manager/list_controllers',['/bessica_d_hw_interface'])]),getParam=lambda key:params[key])
    subscribers=[]
    ros=SimpleNamespace(Time=SimpleNamespace(now=lambda:stamp(now[0])),is_shutdown=lambda:False,
        Subscriber=lambda *a,**k:subscribers.append((a,k)),wait_for_service=lambda *a,**k:None,
        wait_for_message=lambda topic,*a,**k:by_topic[topic],
        ServiceProxy=lambda *a,**k:lambda:SimpleNamespace(controller=[SimpleNamespace(name=n,state='running') for n in ('alicia_controller','hand_controller')]))
    for namespace,names in [('sensor_msgs.msg',['JointState']),('std_msgs.msg',['Header','String','Bool']),
        ('actionlib_msgs.msg',['GoalStatusArray']),('alicia_flexible_grasp_supervisor.msg',['GraspState']),
        ('controller_manager_msgs.srv',['ListControllers'])]:
        mod=ModuleType(namespace)
        for name in names:setattr(mod,name,type(name,(),{}))
        monkeypatch.setitem(sys.modules,namespace,mod)
    mod=ModuleType('alicia_grasp_modes.selection');mod.parse_selection=lambda value:value
    monkeypatch.setitem(sys.modules,mod.__name__,mod)
    ctx=dict(preserved={'/alicia_d_driver_node':driver},_idle_window=window,
        recheck=lambda ctx:dict(state='active'),old=SimpleNamespace(CONTROL=contract['data']['control']['path']))
    return dict(now=now,epoch=epoch,driver=driver,window=window,messages=messages,params=params,
        master=master,ros=ros,ctx=ctx,selection=selected,subscribers=subscribers)


def sample(f):
    return update.sample_idle(f['ctx'],f['ros'],f['master'],f['selection'],f['epoch'])


def test_window_reuses_twenty_samples_without_restarting_subscriptions(feedback):
    first=sample(feedback);second=sample(feedback)
    assert first['sample_count']==second['sample_count']==20
    assert first['sample_stamps_ns']==second['sample_stamps_ns']
    assert feedback['subscribers']==[]
    assert feedback['ctx']['_idle_window'] is feedback['window']


def test_slow_current_verification_uses_feedback_arriving_during_the_check(feedback):
    f=feedback;sample(f)
    previous=f['window'].history[-1]['stamp_ns']
    def later(ctx):
        f['now'][0]+=250_000_000
        for index in range(20):
            message=deepcopy(f['messages']['joint'])
            source=f['now'][0]-1_000_000-(19-index)*10_000_000
            message.header.stamp=SimpleNamespace(to_nsec=lambda source=source:source)
            f['window'].receive('joint',message,source+500_000)
        return dict(state='active')
    f['ctx']['recheck']=later
    result=sample(f)
    assert result['sample_stamps_ns'][0]>previous
    assert result['sample_stamps_ns'][-1]==f['now'][0]-1_000_000
    assert result['control_snapshot']==dict(state='active')
    assert f['subscribers']==[]


@pytest.mark.parametrize('case',['stale','epoch','driver','manual','action','drift','source','future','late_verification'])
def test_reused_window_retains_original_refusal_gates(feedback,case):
    f=feedback;sample(f)
    if case=='stale':f['now'][0]+=200_000_000
    elif case=='epoch':
        message=deepcopy(f['messages']['epoch']);message.stamp=SimpleNamespace(to_nsec=lambda:f['epoch']+1)
        f['window'].receive('epoch',message,f['now'][0])
    elif case=='driver':f['ctx']['preserved']['/alicia_d_driver_node']=dict(f['driver'],pid=201)
    elif case=='manual':f['params']['/gui/joint_direct_mode']=False
    elif case=='action':f['messages']['arm'].status_list=[SimpleNamespace(status=1)]
    elif case=='drift':f['window'].history[-1]['positions'][0]=.004001
    elif case=='source':f['window'].history[-1]['frame_id']='synthetic'
    elif case=='future':f['window'].history[-1]['stamp_ns']=f['now'][0]+1
    elif case=='late_verification':
        def later(ctx):f['now'][0]+=200_000_000;return dict(state='active')
        f['ctx']['recheck']=later
    with pytest.raises(RuntimeError):sample(f)


def test_window_is_bounded_and_epoch_change_cannot_self_recover(feedback):
    f=feedback
    for _ in range(100):f['window'].receive('joint',f['messages']['joint'],f['now'][0])
    assert len(f['window'].history)==80
    f['window'].invalidated=True
    f['window'].receive('epoch',f['messages']['epoch'],f['now'][0])
    with pytest.raises(RuntimeError,match='new transaction'):sample(f)


def test_draft_cannot_enter_live_context(contract):
    contract['data']['draft_pending']=['not reviewed']
    with update.UpdateTransaction(*contract['save']()) as session:
        with pytest.raises(RuntimeError,match='draft contract'):session.live_context()


def test_source_phase_change_does_not_refresh_reference(contract):
    with update.UpdateTransaction(*contract['save']()) as session:
        before=deepcopy(session.data['reference'])
        session.phase_sources=session.sources
        # A phase label alone cannot make the still-old installed source pass.
        with pytest.raises(RuntimeError,match='current input changed'):session.recheck()
        assert session.data['reference']==before


def cli_arguments(contract,action='accept'):
    path,digest_value=contract['save']()
    out=Path(contract['data']['output_root'])/'invocation'
    out.parent.mkdir(exist_ok=True)
    args=[action,'--contract',str(path),'--contract-sha256',digest_value,'--output',str(out)]
    if action=='accept':args+=['--reload',str(path),'--reload-sha256',digest_value]
    return args,out


def test_accept_exception_saves_failed_invocation(contract,monkeypatch):
    args,out=cli_arguments(contract)
    def fail(*_):raise RuntimeError('camera ownership changed')
    monkeypatch.setattr(update,'accept',fail)
    with pytest.raises(RuntimeError,match='camera ownership changed'):update.main(args)
    result=json.loads((out/'failure.json').read_text())
    assert result['passed'] is False and result['runtime_acceptance_passed'] is False
    assert 'camera ownership changed' in result['error']
    assert not (out/'result.json').exists()


def test_cli_preserves_existing_deploy_partial_failure(contract,monkeypatch):
    args,out=cli_arguments(contract,'deploy')
    original=b'{"passed":false,"installed_paths":["partial.py"]}\n'
    def fail(*_):
        out.mkdir();(out/'failure.json').write_bytes(original)
        raise RuntimeError('partial installation')
    monkeypatch.setattr(update,'deploy',fail)
    with pytest.raises(RuntimeError,match='partial installation'):update.main(args)
    assert (out/'failure.json').read_bytes()==original


def test_cli_never_modifies_an_existing_output(contract,monkeypatch):
    args,out=cli_arguments(contract)
    out.mkdir();(out/'earlier.txt').write_text('prior invocation')
    def fail(*_):raise RuntimeError('single-use output exists')
    monkeypatch.setattr(update,'accept',fail)
    with pytest.raises(RuntimeError):update.main(args)
    assert [p.name for p in out.iterdir()]==['earlier.txt']


def test_cli_constructor_failure_writes_only_external_receipt(contract,monkeypatch,tmp_path):
    args,out=cli_arguments(contract)
    mount=tmp_path/'external';root=mount/'实验记录'/'Robot-Controller'
    root.mkdir(parents=True);out=root/'failed'
    args[args.index('--output')+1]=str(out)
    monkeypatch.setattr(update,'EXPERIMENT_ROOT',root)
    monkeypatch.setattr(Path,'is_mount',lambda self:self==mount)
    def fail(*_):raise RuntimeError('contract hash changed')
    monkeypatch.setattr(update,'UpdateTransaction',fail)
    with pytest.raises(RuntimeError,match='contract hash changed'):update.main(args)
    assert 'contract hash changed' in json.loads((out/'failure.json').read_text())['error']


def test_cli_receipt_write_failure_preserves_original_error(contract,monkeypatch,capsys):
    args,out=cli_arguments(contract)
    def fail(*_):raise RuntimeError('original gate rejected')
    def failed_receipt(*_):raise OSError('disk unavailable')
    monkeypatch.setattr(update,'accept',fail)
    monkeypatch.setattr(update,'invocation_failure',failed_receipt)
    with pytest.raises(RuntimeError,match='original gate rejected'):update.main(args)
    assert 'disk unavailable' in capsys.readouterr().err


def test_window_cleanup_attempts_every_subscription_and_preserves_primary(contract,monkeypatch):
    args,out=cli_arguments(contract);closed=[]
    class Subscription:
        def __init__(self,key):self.key=key
        def unregister(self):
            closed.append(self.key)
            if self.key==1:raise RuntimeError('unsubscribe failed')
    def fail(session,_):
        window=update.IdleWindow({},40)
        window.subscribers=[Subscription(1),Subscription(2)]
        session._live_ctx={'_idle_window':window}
        raise RuntimeError('primary idle failure')
    monkeypatch.setattr(update,'accept',fail)
    with pytest.raises(RuntimeError,match='primary idle failure'):update.main(args)
    result=json.loads((out/'failure.json').read_text())
    assert closed==[1,2] and 'primary idle failure' in result['error']
    assert 'unsubscribe failed' in result['cleanup_error']


def test_missing_disk_does_not_create_failure_output(contract,monkeypatch,tmp_path,capsys):
    args,out=cli_arguments(contract)
    def fail(*_):raise RuntimeError('contract unavailable')
    monkeypatch.setattr(update,'UpdateTransaction',fail)
    monkeypatch.setattr(update,'EXPERIMENT_ROOT',tmp_path/'missing'/'实验记录'/'Robot-Controller')
    monkeypatch.setattr(Path,'is_mount',lambda self:False)
    with pytest.raises(RuntimeError,match='contract unavailable'):update.main(args)
    assert not out.exists() and 'no fallback' in capsys.readouterr().err
