"""H5 finite successor boundaries, using real sealed history and explicit I/O fixtures.

The archive bridge uses original validators and bytes, without current-tree
projection or an H4 constructor. Reload/child and Session tests reuse only old
test factories for OS/RPC/source-admission isolation and execute H5 methods.
Root's new full plus no-fixture prepare remains current-release admission.
"""
import ast
from copy import deepcopy
from dataclasses import asdict
import importlib.util
import json
from pathlib import Path
import sys
import types

import pytest


def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);sys.modules[name]=module
    spec.loader.exec_module(module);return module


PATH=Path(__file__).resolve().parents[1]/'remote_only_revision_r5.py'
m=load('support_successor_r5_under_test',PATH)
ORIGINAL_WORKSPACE=m.W
NS=types.SimpleNamespace


def write_json(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value),encoding='utf8');return m.ref(path)


def delta_fixture():
    before={m.FK_SOURCE:'1'*64,m.PARENT_SELF:m.PARENT_HELPER_SHA,
        m.POST_SELF:m.POST_SHA,m.READINESS:m.READINESS_SHA,m.PATH_GUARD:m.PATH_GUARD_SHA}
    before.update({p:'2'*64 for p in m.HISTORICAL_TEST_UPDATES})
    after=dict(before,**{p:'3'*64 for p in m.REQUIRED_SUCCESSOR_CHANGES})
    review=dict(kind='r27_support_box_source_review',reviewed=True,
        baseline_release=deepcopy(m.PARENT_RELEASE),parent_acceptance=deepcopy(m.PARENT_ACCEPTANCE),
        parent_continuation=deepcopy(m.PARENT_CONTINUATION),source_sha256=after,
        changed_sources=m.source_delta(before,after),additional_source_files=[m.SELF,m.TEST])
    return before,after,review


def test_only_final_fk_production_and_explicit_history_test_maintenance_are_allowed():
    before,after,review=delta_fixture()
    assert m.validate_source_delta(before,after,review)==review['changed_sources']
    for path in m.HISTORICAL_TEST_UPDATES:after[path]='4'*64
    review['changed_sources']=m.source_delta(before,after)
    assert set(m.validate_source_delta(before,after,review))==m.ALLOWED_SUCCESSOR_CHANGES
    assert {p for p in m.ALLOWED_SUCCESSOR_CHANGES if '/src/' in p}=={m.FK_SOURCE}


@pytest.mark.parametrize('path',sorted(m.REQUIRED_SUCCESSOR_CHANGES))
def test_each_new_source_or_coverage_file_is_required(path):
    before,after,review=delta_fixture()
    if path in before:after[path]=before[path]
    else:after.pop(path)
    review['changed_sources']=m.source_delta(before,after)
    with pytest.raises(RuntimeError):m.validate_source_delta(before,after,review)


@pytest.mark.parametrize('path',[m.PATH_GUARD,m.PRECOMPUTE,m.MODE_ENTRY,m.PLACEMENT,
    m.PARENT_SELF,m.POST_SELF,m.READINESS,m.ENTRY,m.ORIGINAL_RUNNER,
    'src/real-arm/alicia_d_driver/src/alicia_d_driver_node.cpp'])
def test_no_physical_gate_typed_control_or_other_business_source_delta(path):
    before,after,review=delta_fixture();after[path]='9'*64
    review['changed_sources']=m.source_delta(before,after)
    with pytest.raises(RuntimeError):m.validate_source_delta(before,after,review)


@pytest.fixture(scope='module')
def archived():
    data=m.read_ref(m.PARENT_REVISION)
    data.update(parent_acceptance=m.PARENT_ACCEPTANCE,parent_revision=m.PARENT_REVISION,
                parent_continuation=m.PARENT_CONTINUATION)
    sources=m.read_ref(m.PARENT_RELEASE)['source_sha256']
    return NS(data=data,sources=sources,stack=m.load_source(m.STACK,sources),
        entry=m.load_source(m.ENTRY,sources),h4=m.load_source(m.PARENT_SELF,sources))


def parent_chain(f):return m.verify_parent_chain(f.data,f.stack,f.entry,f.h4)


def test_actual_parent_composite_and_two_recordings_without_old_current_tree_or_services(archived,monkeypatch):
    def forbidden(*args,**kwargs):raise AssertionError('historical bridge queried current old release or live process')
    monkeypatch.setattr(archived.stack,'current_release_evidence',forbidden)
    monkeypatch.setattr(archived.h4,'Revision',forbidden)
    monkeypatch.setattr(m.subprocess,'check_output',forbidden)
    chain=parent_chain(archived)
    assert len(chain['source_sha256'])==4286
    assert chain['proof']['historical_verification']['current_summary']==dict(
        tests=20053,passed=20020,skipped=33,failures=0,errors=0)
    assert chain['proof']['continuation_target_summary']==dict(tests=151,passed=151,skipped=0,failures=0,errors=0)
    assert chain['acceptance']['nodes'][m.REMOTE]['pid']==112610
    assert chain['acceptance']['typed_initial_contract_ready'] is False
    assert chain['history'].action_transport_sequence==1
    assert chain['history'].recorded_goal_count==1
    assert chain['history'].connection_callerids_available is False
    assert chain['prior_attempt']['no_new_recorded_action'] is True
    assert chain['prior_attempt']['current_readiness_claimed'] is False
    assert chain['proof']['old_release_claimed_installed_now'] is False
    assert chain['child_argv'][2]==str(m.W/m.PARENT_SELF)
    assert chain['working_directory']['path']!=str(m.W)
    # Same module instance is necessary for its exact frozen dataclass type.
    assert type(chain['history']) is chain['readiness'].TransportHistory


@pytest.mark.parametrize('which',['acceptance','revision','release','continuation','prior_audit',
    'prior_claim','target','history','actions','epoch','prior_authorization','source','launch'])
def test_real_parent_archive_corruption_is_rejected_without_semantic_success_stubs(archived,monkeypatch,which):
    spec=m.read_ref(m.PARENT_CONTINUATION);parent=m.read_ref(m.PARENT_ACCEPTANCE)
    post=m.load_source(m.POST_SELF,m.parent_source_manifest())
    refs=dict(acceptance=m.PARENT_ACCEPTANCE,revision=m.PARENT_REVISION,release=m.PARENT_RELEASE,
        continuation=m.PARENT_CONTINUATION,prior_audit=m.PRIOR_ATTEMPT_AUDIT,prior_claim=m.PRIOR_CLAIM,
        target=spec['targeted_regression'],history=post.ANCHORS['recorded_history'],
        actions=post.ANCHORS['recorded_actions'],epoch=post.ANCHORS['epoch_context'],
        prior_authorization=m.read_ref(m.PRIOR_ATTEMPT_AUDIT)['sources']['authorization.json'],
        source=parent['source_startup_receipt'],launch=dict(path=str(m.PARENT_OUTPUT/'launch_intent.json')))
    target=refs[which]['path'];original=Path.read_bytes
    def read(path):
        raw=original(path);return raw+b'\n' if str(path)==target else raw
    monkeypatch.setattr(Path,'read_bytes',read)
    with pytest.raises(RuntimeError,match='changed'):parent_chain(archived)


@pytest.mark.parametrize('fault',['new_goal','sequence','epoch','manual','unknown_result','old_claim',
    'missing_metadata','nonzero_metadata','bool_metadata','nonzero_actual'])
def test_second_record_semantics_cannot_claim_a_new_execution_or_unverified_manual_return(archived,fault):
    parent=m.read_ref(m.PARENT_ACCEPTANCE)
    readiness=m.load_source(m.READINESS,m.parent_source_manifest())
    post=m.load_source(m.POST_SELF,m.parent_source_manifest())
    history=readiness.validate_transport_history(m.read_ref(post.ANCHORS['recorded_history']),
                                               m.read_ref(post.ANCHORS['recorded_actions']))
    # Mutated document is explicitly a negative semantic fixture, not an archive replacement.
    def bound(reference):
        value=deepcopy(m.read_ref(reference))
        if reference==m.PRIOR_ATTEMPT_AUDIT:
            if fault=='new_goal':value['new_arm_gripper_actions']=[dict(goal='extra')]
            elif fault in ('sequence','epoch'):
                row=value['typed_state_groups']['/alicia_d/physical_reference_state'][0]
                row['action_transport_sequence' if fault=='sequence' else 'epoch_ns']+=1
            elif fault in ('missing_metadata','nonzero_metadata','bool_metadata','nonzero_actual'):
                topic='/alicia_controller/follow_joint_trajectory/goal'
                if fault=='missing_metadata':value['metadata_counts'].pop(topic)
                elif fault=='nonzero_actual':value['actual_read_counts'][topic]=1
                else:value['metadata_counts'][topic]['message_count']=False if fault=='bool_metadata' else 1
        elif reference['path'].endswith('/authorization.json'):
            if fault=='manual':value['manual_return_verified']=False
            elif fault=='unknown_result':value['physical_grasp_verified']=None
        elif reference==m.PRIOR_CLAIM and fault=='old_claim':value['acceptance']['sha256']='0'*64
        return value
    error=('prior attempt includes new execution' if fault=='new_goal' else
           'prior recorded typed transport history differs' if fault in ('sequence','epoch') else
           'prior actual recorder/manual-return chain differs' if fault in ('manual','unknown_result') else
           'consumed prior single-attempt claim differs' if fault=='old_claim' else
           'prior arm/hand action metadata or read count is not exact zero')
    with pytest.raises(RuntimeError,match=error):m.verify_prior_no_action_attempt(parent,history,bound)


def h4_factories(monkeypatch):
    tests=load('r5_h4_io_factories',ORIGINAL_WORKSPACE/m.PARENT_TEST)
    monkeypatch.setattr(tests,'m',m);monkeypatch.setattr(tests,'PATH',PATH)
    return tests


@pytest.fixture
def reload_io(tmp_path,monkeypatch):
    tests=h4_factories(monkeypatch)
    f=tests.successor_reload.__wrapped__(tmp_path,monkeypatch)
    f.revision.parent['state']=dict(epoch_ns=42)
    f.h4tests=tests
    return f


@pytest.mark.parametrize('boundary',['shutdown','launch'])
def test_real_reload_stops_exact_current_h4_and_preserves_cwd_at_os_boundaries(reload_io,boundary):
    f=reload_io;f.stop=boundary
    with pytest.raises(f.legacy.CwdLifecycleBoundary,match=boundary+' boundary'):m.reload_remote(NS())
    construction=m.read_ref(m.ref(f.revision.output/'construction.json'))
    assert construction['parent_acceptance']==m.PARENT_ACCEPTANCE
    assert construction['original_argv']==f.argv and f.argv[2]==str(m.W/m.PARENT_SELF)
    assert construction['parent_relay_argv']==f.relay_argv
    assert construction['working_directory']==m.directory_identity(f.working_path)
    assert f.environment['PWD']==str(m.W)!=str(f.working_path)
    assert f.shutdown_calls==1
    if boundary=='launch':
        assert f.payload['argv'][:3]==f.relay_argv[:3]
        assert f.payload['parent_shutdown']==m.ref(f.revision.output/'parent_stopped.json')
        assert f.payload['environment']['R27_REMOTE_REVISION_SHA256']==f.revision.reference['sha256']
        assert f.launches[0][f.launches[0].index('-B')+1]==str(m.W/m.SELF)
    else:assert not f.launches


@pytest.mark.parametrize('fault',['argv','ticks','unit','cwd','environment'])
def test_reloaded_parent_drift_is_rejected_before_shutdown(reload_io,fault):
    f=reload_io
    def drift():
        if fault=='argv':(f.proc/'cmdline').write_bytes(b'/usr/bin/python3\0old-87941\0')
        elif fault=='ticks':f.processes[f.original['pid']]['start_ticks']='reused'
        elif fault=='unit':f.revision.stack.properties=lambda unit:dict(MainPID='foreign')
        elif fault=='cwd':f.working_path.rename(f.working_path.with_name('detached'));f.working_path.mkdir()
        else:(f.proc/'environ').write_bytes(b'R27_REMOTE_REVISION_SHA256=foreign\0')
    f.on_second_wait=drift
    with pytest.raises(RuntimeError):m.reload_remote(NS())
    assert f.shutdown_calls==0 and not f.launches


def prepare_child(f,monkeypatch,fault):
    args,entered,_=f.h4tests.prepare_successor_child(f,monkeypatch)
    path=m.W/m.PATH_GUARD;path.parent.mkdir(parents=True,exist_ok=True)
    # Real original proof source bytes and actual code object, with only import
    # globals absent. Startup observes identity; it never executes the proof.
    raw=(ORIGINAL_WORKSPACE/m.PATH_GUARD).read_bytes();path.write_bytes(raw)
    assert m.sha(path)==m.PATH_GUARD_SHA
    node=next(n for n in ast.parse(raw).body if isinstance(n,ast.FunctionDef) and n.name==m.SUPPORT_SYMBOL)
    module=types.ModuleType('alicia_flexible_grasp.robot.observation_path_guard');module.__file__=str(path)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),str(path),'exec'),module.__dict__)
    monkeypatch.setitem(sys.modules,module.__name__,module)
    fk=sys.modules['alicia_grasp_modes.reachable_observation_views']
    setattr(fk,m.SUPPORT_SYMBOL,getattr(module,m.SUPPORT_SYMBOL))
    if fault=='function':setattr(fk,m.SUPPORT_SYMBOL,lambda:None)
    elif fault=='bytes':path.write_bytes(raw+b'\n')
    elif fault=='foreign_module':module.__file__=str(path.with_name('foreign.py'))
    return args,entered


@pytest.mark.parametrize('fault',[None,'function','bytes','foreign_module'])
def test_real_child_checks_actual_fk_proof_dependency_before_original_main(reload_io,monkeypatch,fault):
    args,entered=prepare_child(reload_io,monkeypatch,fault)
    if fault:
        with pytest.raises(RuntimeError):m.child(args)
        assert not entered.exists() and not (reload_io.revision.output/'source_start.json').exists()
    else:
        m.child(args)
        receipt=m.read_ref(m.ref(reload_io.revision.output/'source_start.json'))
        assert entered.read_text()=='entered'
        assert receipt['dependency_bindings']['support_proof_dependency']==m.expected_support_binding()
        assert len(receipt['module_sources'])==5 and receipt['memory_attestation'] is False
    assert sys.gettrace() is None


def ticket_fixture(archived):
    chain=parent_chain(archived)
    rev=NS(entry=archived.entry,parent_chain=chain,
        reference=dict(path='/new/revision.json',sha256='r'*64),data=dict(instruction=dict(path='/resume',sha256='s'*64)))
    accepted=dict(path='/new/acceptance/result.json',sha256='a'*64)
    reference=dict(path='/new/ticket.json',sha256='i'*64)
    instruction=dict(source='actual_user_message',text='continued actual grasp instruction',
        automatic_grasp_authorized=True,user_alignment_ready=True,new_scene_confirmation_required=False,
        new_scene_confirmation_claimed=False,maximum_attempts=1,automatic_retry=False,
        runtime_revision=rev.reference,current_acceptance=accepted,latest_resume=rev.data['instruction'],
        parent_continuation=m.PARENT_CONTINUATION)
    return rev,accepted,reference,instruction


@pytest.mark.parametrize('fault',[None,'attempt1_path','attempt2_sha','revision','acceptance','resume','continuation','retry'])
def test_new_ticket_binds_new_actual_acceptance_and_rejects_both_consumed_claims(archived,fault):
    rev,accepted,reference,instruction=ticket_fixture(archived)
    if fault=='attempt1_path':reference['path']=rev.parent_chain['proof']['first_consumed_instruction']['path']
    elif fault=='attempt2_sha':reference['sha256']=rev.parent_chain['prior_attempt']['consumed_instruction']['sha256']
    elif fault=='retry':instruction['automatic_retry']=True
    elif fault:
        instruction[dict(revision='runtime_revision',acceptance='current_acceptance',resume='latest_resume',
            continuation='parent_continuation')[fault]]=dict(path='/other',sha256='o'*64)
    if fault is None:m.validate_new_instruction(rev,accepted,reference,instruction)
    else:
        with pytest.raises(RuntimeError):m.validate_new_instruction(rev,accepted,reference,instruction)


def test_one_runtime_claim_cannot_be_replaced_even_with_a_different_ticket(tmp_path):
    revision=object.__new__(m.Revision)
    revision.storage=NS(validate=lambda *args,**kwargs:None,ros_log_root=tmp_path);revision.binding={}
    path=tmp_path/'attempt_claim.json';value=dict(instruction=dict(path='/ticket1',sha256='1'*64))
    receipt=revision.save(path,value)
    with pytest.raises(FileExistsError):revision.save(path,dict(instruction=dict(path='/ticket2',sha256='2'*64)))
    assert m.read_ref(receipt)==value


@pytest.fixture
def session_io(tmp_path,monkeypatch):
    """Original Session -> H5 recheck/graph/RecorderBinding -> actual pure typed gates.

    Old factory isolates source/health/sensor admission and ROS/process I/O;
    archive tests above cover sealed provenance separately. No ROS is invoked.
    """
    posttests=load('r5_post_test_io_factories',ORIGINAL_WORKSPACE/m.POST_TESTS[0])
    farchive=posttests.archived.__wrapped__()
    tests=h4_factories(monkeypatch);original_load=posttests.load
    def factory(name,path):
        return tests if Path(path)==ORIGINAL_WORKSPACE/m.PARENT_TEST else original_load(name,path)
    monkeypatch.setattr(posttests,'load',factory)
    f=posttests.session_io.__wrapped__(tmp_path,monkeypatch,farchive)
    f.ctx.__class__=m.AttemptContext
    revision=f.f.revision
    revision.readiness=farchive.readiness;revision.history=farchive.history;revision.post=posttests.m
    # The inherited factory creates old-kind evidence. These are clearly new
    # fixture receipts, rewritten to the H5 schema before the actual graph call.
    path=revision.output/'construction.json';construction=json.loads(path.read_bytes())
    construction['kind']='r27_remote_only_construction_r5';construction_ref=write_json(path,construction)
    stopped_path=revision.output/'parent_stopped.json';stopped=json.loads(stopped_path.read_bytes())
    stopped.update(kind='r27_successor_parent_stopped_r5',construction=construction_ref)
    stopped_ref=write_json(stopped_path,stopped)
    write_json(revision.output/'source_start.json',dict(construction=construction_ref,parent_shutdown=stopped_ref))
    return f


def test_original_session_and_recorder_check_history_twice_before_mode_then_allow_auto_selection(session_io):
    f=session_io;f.session.preflight();f.session.enter_automatic();selection=f.session.select()
    assert f.published==[False] and selection['strategy']=='observe_return'
    assert f.observations==[True,True] and len(f.ctx.typed_admission)==2
    assert f.f.recorder.reference is not None
    # Finish a failed fake runner at the original execution-I/O boundary, then
    # exercise the real return_manual/finish_returned_attempt lifecycle.
    def run_once(selected):
        f.prepared.released=True;f.prepared.process.poll=lambda:3;return 3
    f.prepared.run_once=run_once
    assert f.session.run_selected(selection)==3
    f.ctx.finish=m.load_source('tools/finish_returned_grasp.py',
        {'tools/finish_returned_grasp.py':m.sha(ORIGINAL_WORKSPACE/'tools/finish_returned_grasp.py')})
    result=f.session.return_manual()
    assert result['passed'] is True and result['manual_request_sent'] is True
    assert f.published==[False,True] and f.observations==[True,True]
    f.ctx.close();assert f.observations[-1]=='closed'


@pytest.mark.parametrize('fault',['wrong_sequence','held_task','failed_task','foreign_recorder','policy'])
def test_original_preflight_rejects_new_admission_fault_before_any_mode_request(session_io,monkeypatch,fault):
    f=session_io
    if fault=='wrong_sequence':
        f.rows['driver'][-1].action_transport_sequence+=1
        ticks=iter((0.,0.,5.,5.))
        monkeypatch.setattr(m,'time',NS(monotonic=lambda:next(ticks),sleep=lambda _:None,time_ns=lambda:0))
    elif fault in ('held_task','failed_task'):
        f.messages['task'].state,f.messages['task'].stage=('HOLDING',10) if fault=='held_task' else ('FAILED',7)
    elif fault=='foreign_recorder':f.f.state[0].append(['/joint_commands',[f.f.name]])
    else:f.f.actual_simulation_values['/mujoco_digital_twin/min_score']=79
    with pytest.raises(RuntimeError):f.session.preflight()
    assert f.published==[] and f.session.automatic_requested is False


def test_new_goal_sequence_or_unavailable_compute_never_gates_original_manual_cleanup(session_io,monkeypatch):
    f=session_io
    for key in ('driver','hardware'):f.rows[key][-1].action_transport_sequence+=10
    def forbidden(*args,**kwargs):raise AssertionError('compute/recorder/historic sequence entered manual cleanup')
    monkeypatch.setattr(m,'wait_post_ready',forbidden)
    f.ctx.rev.simulation_policy=forbidden;f.ctx.rev.graph=forbidden
    f.ctx.recorder.check=forbidden;f.ctx.recorder.graph_owner=forbidden
    f.ctx.health.observe=forbidden;f.ctx.health.check_binding=forbidden
    assert f.session.observe(manual_return=True)['manual_parameter'] is True
    assert f.observations==[] and f.ctx.typed_admission==[]


@pytest.fixture
def accept_io(tmp_path,monkeypatch,archived):
    """Actual accept orchestrator and fixed-query pure gates; external I/O fixture.

    Binary/graph/health/current-release admission are explicit I/O substitutes
    here. Real archive provenance is tested separately; this does not claim a
    new full run or live acceptance. Raw/typed message values are real snapshot
    values and are passed unchanged to the original post-trajectory validator.
    """
    chain=parent_chain(archived);post=chain['post'];readiness=chain['readiness']
    snapshot=m.read_ref(post.ANCHORS['snapshot']);rows=readiness.recorded_rows(snapshot)
    now=snapshot['finished_ns'];epoch=snapshot['state']['epoch_ns']
    typed=m.load_source(m.TYPED,archived.sources);assets=m.read_ref(archived.data['typed_assets'])
    parent=chain['acceptance'];output=tmp_path/'runtime';output.mkdir()
    reference=dict(path='/new/revision.json',sha256='r'*64);review=dict(path='/new/review.json',sha256='e'*64)
    deployed=deepcopy(m.read_ref(parent['reload']))
    construction_ref=write_json(output/'construction.json',dict(explicit_offline_fixture=True))
    source_ref=write_json(output/'source_start.json',dict(explicit_offline_fixture=True))
    deployed.update(kind='r27_remote_only_reload_r5',revision=reference,execution_review=review,
        parent_acceptance=m.PARENT_ACCEPTANCE,construction=construction_ref,source_startup_receipt=source_ref)
    deployed['parent_shutdown']=write_json(output/'parent_stopped.json',dict(explicit_offline_fixture=True))
    deployed['nodes'][m.REMOTE]=dict(deployed['nodes'][m.REMOTE],pid=300001,start_ticks='300001')
    startup_binding=deepcopy(m.read_ref(parent['remote_startup_binding']))
    startup_binding['node_identity']={k:v for k,v in deployed['nodes'][m.REMOTE].items() if k!='uri'}
    deployed['remote_startup_binding']=write_json(output/'remote_binding_fixture.json',startup_binding)
    reload_ref=write_json(output/'reload_result.json',deployed)
    events=[];queries=[];query_states={key:deepcopy(rows[key][-1]) for key in ('driver','hardware')}
    observer=NS(snapshot=lambda:rows,close=lambda:events.append('observer.close'))
    monitor=NS(close=lambda:events.append('monitor.close'))
    simulation=NS(wait_ready=lambda **kwargs:None,service_snapshot=lambda:parent['simulation'])
    stack=NS(Monitor=lambda config:monitor,gui_windows=lambda pid:['fixture GUI'],SIM='fixture',
        load=lambda path:simulation,current_release_evidence=lambda release:dict(explicit_current_admission_fixture=True))
    revision=object.__new__(m.Revision)
    revision.reference=reference;revision.output=output;revision.session=tmp_path
    revision.binding={};revision.data=dict(release=dict(path='/new/release',sha256='l'*64),
        instruction=dict(path='/new/resume',sha256='s'*64))
    revision.parent=parent;revision.history=chain['history'];revision.post=post;revision.readiness=readiness
    revision.typed=typed;revision.assets=assets;revision.stack=stack
    revision.contract=m.read_ref(parent['contract']);revision.config=m.read_ref(revision.contract['configuration'])
    revision.sources=chain['source_sha256'];revision.parent_chain=chain;revision.entry=archived.entry
    revision.startup=dict(components=dict(simulation=dict(unit='retained-simulation')))
    revision.preserved_typed_evidence=dict(explicit_fixture=True);revision.release=dict(explicit_fixture=True)
    revision.storage=NS(environment=lambda *args,**kwargs:{},validate=lambda *args,**kwargs:None,ros_log_root=tmp_path)
    revision.verify_files=lambda **kwargs:events.append('source.verify')
    revision.live_policy=lambda master:events.append('policy.verify')
    revision.alias_snapshot=lambda reference:deepcopy(deployed['auxiliary_nodes'])
    revision.graph=lambda *args,**kwargs:(deepcopy(deployed['nodes']),deepcopy(deployed['auxiliary_nodes']))
    revision.health=NS(load_binding=lambda *args:{},evaluate=lambda *args:dict(passed=True,finished_ns=m.time.time_ns()))
    monkeypatch.setattr(m,'load_live',lambda args:(revision,review))
    monkeypatch.setattr(m,'remote_graph',lambda master:deployed['remote_graph'])
    monkeypatch.setattr(m,'wait_manual',lambda *args,**kwargs:deepcopy(deployed['state']))
    monkeypatch.setattr(typed,'Observer',lambda rospy:observer)
    monkeypatch.setattr(typed,'runtime_binaries',lambda *args:dict(explicit_binary_io_fixture=True))
    monkeypatch.setattr(typed,'graph_bindings',lambda *args,**kwargs:dict(topics={},services={}))
    monkeypatch.setattr(typed,'bounded_query',lambda service,request:service(request))
    task=NS(_connection_header=dict(callerid='/grasp_task_node',latching='1'),
        state='IDLE',stage=0,active=False,success=False,message='',header=NS(stamp=readiness._Stamp(now)))
    service_keys=dict(zip(typed.SERVICES,('driver','hardware')))
    def proxy(name,kind):
        def call(request=None):
            if name=='/controller_manager/list_controllers':
                return NS(controller=[NS(name=n,state='running') for n in ('alicia_controller','hand_controller')])
            assert request.operation==request.QUERY;queries.append(name)
            return NS(success=True,code='LEGACY',state=query_states[service_keys[name]])
        return call
    ros=NS(init_node=lambda *a,**k:None,get_name=lambda:'/offline_acceptance',is_shutdown=lambda:False,
        Time=NS(now=lambda:readiness._Stamp(now)),get_param=lambda name:assets['required_parameters'][name],
        ServiceProxy=proxy,wait_for_message=lambda *a,**k:task)
    monkeypatch.setitem(sys.modules,'rospy',ros)
    monkeypatch.setitem(sys.modules,'rosgraph',NS(Master=lambda name:NS()))
    monkeypatch.setitem(sys.modules,'controller_manager_msgs.srv',NS(ListControllers=object))
    monkeypatch.setitem(sys.modules,'alicia_flexible_grasp_supervisor.msg',NS(GraspState=object))
    classes={}
    for name in ('PhysicalReferenceState','PhysicalReferenceCommand','SetPhysicalReference'):
        type_name='alicia_d_driver/'+name
        path=next(p for p in assets['current_sha256'] if p.endswith('/_'+name+'.py'))
        module_name='r5_generated_io_'+name
        module=types.ModuleType(module_name);module.__file__=path
        monkeypatch.setitem(sys.modules,module_name,module)
        classes[name]=type(name,(),dict(_type=type_name,_md5sum=assets['type_md5'][type_name],__module__=module_name))
    monkeypatch.setitem(sys.modules,'alicia_d_driver.msg',NS(**{k:classes[k] for k in ('PhysicalReferenceState','PhysicalReferenceCommand')}))
    monkeypatch.setitem(sys.modules,'alicia_d_driver.srv',NS(SetPhysicalReference=classes['SetPhysicalReference'],
        SetPhysicalReferenceRequest=type('Request',(),dict(QUERY=0))))
    return NS(revision=revision,review=review,reload_ref=reload_ref,deployed=deployed,rows=rows,
        query_states=query_states,queries=queries,events=events,task=task,readiness=readiness,
        args=NS(reload=output/'reload_result.json',reload_sha256=reload_ref['sha256']),now=now)


@pytest.mark.parametrize('fault',[None,'fixed_query_sequence','fixed_query_expired','held_task'])
def test_actual_accept_retains_history_and_never_reissues_fixed_query(accept_io,fault):
    f=accept_io
    if fault=='fixed_query_sequence':f.query_states['driver'].action_transport_sequence+=1
    elif fault=='fixed_query_expired':f.query_states['driver'].header.stamp=f.readiness._Stamp(f.now-500000001)
    elif fault=='held_task':f.task.state,f.task.stage='HOLDING',10
    if fault:
        with pytest.raises(RuntimeError):m.accept(f.args)
    else:m.accept(f.args)
    accepted=m.read_ref(m.ref(f.revision.output/'acceptance/result.json'))
    assert len(f.queries)==(0 if fault=='held_task' else 2)
    assert len(set(f.queries))==len(f.queries)
    assert 'observer.close' in f.events and 'monitor.close' in f.events
    assert accepted['typed_initial_contract_ready'] is False
    if fault:
        assert accepted['passed'] is False and accepted['physical_grasp_verified'] is False
    else:
        assert accepted['typed_post_trajectory_contract_ready'] is True
        assert accepted['transport_history']['action_transport_sequence']==1
        assert accepted['query_receipt_source_wait']['query_reissued'] is False
        assert accepted['query_receipt_source_wait']['receipt_refreshed'] is False
        m.validate_acceptance_chain(m.ref(f.revision.output/'acceptance/result.json'),accepted,
            f.revision.reference,f.revision.output,f.revision.data['release'],f.revision.parent['contract'],
            f.review,f.reload_ref,f.deployed,f.revision.history)
        instruction=dict(source='actual_user_message',text='new single attempt',
            automatic_grasp_authorized=True,user_alignment_ready=True,new_scene_confirmation_required=False,
            new_scene_confirmation_claimed=False,maximum_attempts=1,automatic_retry=False,
            runtime_revision=f.revision.reference,current_acceptance=m.ref(f.revision.output/'acceptance/result.json'),
            latest_resume=f.revision.data['instruction'],parent_continuation=m.PARENT_CONTINUATION)
        instruction_ref=write_json(f.revision.output/'new_instruction.json',instruction)
        context=m.AttemptContext(f.revision,m.ref(f.revision.output/'acceptance/result.json'),
            instruction_ref,f.review,NS(explicit_recorder_io_fixture=True))
        assert context.acceptance['transport_history']==asdict(f.revision.history)
        assert context.remote['node_identity']['pid']==300001
        context.validate_contract();context.close()
        # A successful startup-kind receipt or another sequence cannot be used
        # as the new runtime's post-trajectory acceptance.
        for key,value in (('typed_initial_contract_ready',True),('transport_history',dict(accepted['transport_history'],action_transport_sequence=2))):
            bad=deepcopy(accepted);bad[key]=value
            with pytest.raises(RuntimeError):
                m.validate_acceptance_chain(m.ref(f.revision.output/'acceptance/result.json'),bad,
                    f.revision.reference,f.revision.output,f.revision.data['release'],f.revision.parent['contract'],
                    f.review,f.reload_ref,f.deployed,f.revision.history)
