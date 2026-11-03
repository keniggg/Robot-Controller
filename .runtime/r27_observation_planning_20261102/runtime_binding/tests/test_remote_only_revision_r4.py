"""Offline successor contracts; original archive bytes plus isolated OS/RPC fixtures.

The real parent chain is read without native current-tree substitution. Current
full-release admission still requires the root's post-regression prepare smoke.
No fixture below represents a live ROS deployment or a physical grasp.
"""
import ast
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import sys
import types

import pytest


PATH=Path(__file__).resolve().parents[1]/'remote_only_revision_r4.py'
spec=importlib.util.spec_from_file_location('r27_successor_r4_under_test',PATH)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
ORIGINAL_WORKSPACE=m.W


def delta_fixture():
    before={m.PLACEMENT:m.PLACEMENT_BEFORE_SHA,m.PARENT_SELF:m.PARENT_HELPER_SHA,
            m.PARENT_TEST:'1'*64,'src/retained_driver.cpp':'2'*64}
    after=dict(before,**{p:'3'*64 for p in m.REQUIRED_SUCCESSOR_CHANGES})
    review=dict(kind='r27_minimum_placement_source_review',reviewed=True,
        baseline_release=deepcopy(m.PARENT_RELEASE),parent_acceptance=deepcopy(m.PARENT_ACCEPTANCE),
        source_sha256=after,changed_sources=m.source_delta(before,after),additional_source_files=[m.SELF,m.TEST,m.PARENT_TEST])
    return before,after,review


def test_successor_delta_is_relative_to_accepted_r5_not_r26():
    before,after,review=delta_fixture()
    assert m.validate_source_delta(before,after,review)==m.source_delta(before,after)
    assert after[m.PARENT_SELF]==m.PARENT_HELPER_SHA


@pytest.mark.parametrize('path',sorted(m.REQUIRED_SUCCESSOR_CHANGES))
def test_each_successor_source_and_coverage_change_is_required(path):
    before,after,review=delta_fixture()
    if path in before:after[path]=before[path]
    else:after.pop(path)
    review['changed_sources']=m.source_delta(before,after)
    with pytest.raises(RuntimeError):m.validate_source_delta(before,after,review)


@pytest.mark.parametrize('path',[m.PARENT_SELF,m.MODE_ENTRY,m.ENTRY,m.STACK,
    'src/alicia_grasp_modes/src/alicia_grasp_modes/runner_contract.py',
    'src/real-arm/alicia_d_driver/src/alicia_d_driver_node.cpp',m.REGRESSION_WRAPPER])
def test_successor_cannot_expand_the_production_or_launcher_delta(path):
    before,after,review=delta_fixture();after[path]='9'*64;review['changed_sources']=m.source_delta(before,after)
    with pytest.raises(RuntimeError):m.validate_source_delta(before,after,review)


@pytest.mark.parametrize('fault',['baseline','parent','old_module','remove','extra_source'])
def test_source_review_rejects_wrong_parent_or_missing_bytes(fault):
    before,after,review=delta_fixture()
    if fault=='baseline':review['baseline_release']['sha256']=m.BASELINE_RELEASE_SHA
    elif fault=='parent':review['parent_acceptance']['sha256']='0'*64
    elif fault=='old_module':before[m.PLACEMENT]='0'*64
    elif fault=='remove':after.pop('src/retained_driver.cpp')
    else:review['additional_source_files'].append(m.RUNTIME_PREFIX+'unreviewed.py')
    review['changed_sources']=m.source_delta(before,after)
    with pytest.raises(RuntimeError):m.validate_source_delta(before,after,review)


@pytest.fixture(scope='module')
def real_parent():
    data=m.read_ref(m.PARENT_REVISION)
    data.update(parent_acceptance=m.PARENT_ACCEPTANCE,parent_revision=m.PARENT_REVISION)
    sources=m.read_ref(m.PARENT_RELEASE)['source_sha256']
    stack=m.load_source(m.STACK,sources);entry=m.load_source(m.ENTRY,sources)
    original=m.load_source(m.PARENT_SELF,sources)
    return types.SimpleNamespace(data=data,stack=stack,entry=entry,original=original)


def parent_chain(f):return m.verify_parent_chain(f.data,f.stack,f.entry,f.original)


def test_real_parent_archive_and_sealed_full_r4_are_verified_without_old_installed_claim(real_parent,monkeypatch):
    def forbidden(*args,**kwargs):raise AssertionError('historical proof must not query a live unit or old installed tree')
    monkeypatch.setattr(real_parent.stack,'current_release_evidence',forbidden)
    monkeypatch.setattr(m.subprocess,'check_output',forbidden)
    chain=parent_chain(real_parent)
    assert chain['proof']['historical_verification']['current_summary']==dict(
        tests=19933,passed=19900,skipped=33,failures=0,errors=0)
    assert chain['proof']['old_release_claimed_installed_now'] is False
    assert chain['proof']['retained_node_count']==16 and len(chain['references'])==18
    assert chain['child_argv'][2]==str(m.W/m.PARENT_SELF)
    assert chain['relay_argv'][:2]==['/usr/bin/python3',str(m.W/m.RELAY)]
    assert chain['working_directory']['path']!=str(m.W)


@pytest.mark.parametrize('which',['acceptance','revision','release','regression','review','construction',
                                  'launch','source','reload','execution_review','remote_binding'])
def test_real_parent_archive_corruption_is_rejected_at_original_bytes(real_parent,monkeypatch,which):
    chain=parent_chain(real_parent); parent=chain['acceptance']; revision=chain['revision']
    refs=dict(acceptance=m.PARENT_ACCEPTANCE,revision=m.PARENT_REVISION,release=m.PARENT_RELEASE,
        regression=chain['release']['actual_regression'],review=revision['source_review'],
        construction=parent['construction'],source=parent['source_startup_receipt'],reload=parent['reload'],
        execution_review=parent['execution_review'],remote_binding=parent['remote_startup_binding'],
        launch=dict(path=str(m.PARENT_OUTPUT/'launch_intent.json')))
    target=refs[which]['path'];original=Path.read_bytes
    def read(path):
        raw=original(path)
        return raw+b'\n' if str(path)==target else raw
    monkeypatch.setattr(Path,'read_bytes',read)
    with pytest.raises(RuntimeError,match='changed:'):parent_chain(real_parent)


@pytest.mark.parametrize('fault',['missing','duplicate','skip_changed','new_skip'])
def test_original_case_inventory_and_skip_conditions_cannot_disappear(fault):
    old=dict(expected_case_ids=['x.py::a','x.py::b'],case_results=[
        dict(nodeid='x.py::a',setup='passed',call='passed'),
        dict(nodeid='x.py::b',setup='skipped',call=None,skip_reason='original condition')])
    new=deepcopy(old);new['expected_case_ids'].append('y.py::c')
    new['case_results'].append(dict(nodeid='y.py::c',setup='passed',call='passed'))
    assert m.validate_successor_cases(old,new)['added_case_count']==1
    if fault=='missing':new['expected_case_ids'].remove('x.py::a')
    elif fault=='duplicate':new['expected_case_ids'].append('x.py::a')
    elif fault=='skip_changed':new['case_results'][1]['skip_reason']='different'
    else:new['case_results'][2].update(call='skipped',skip_reason='new')
    with pytest.raises(RuntimeError):m.validate_successor_cases(old,new)


def legacy_module(monkeypatch):
    path=PATH.parent/'tests/test_remote_only_revision_r3.py'
    spec=importlib.util.spec_from_file_location('r4_test_io_factories',path)
    legacy=importlib.util.module_from_spec(spec);spec.loader.exec_module(legacy)
    # Reuse only test I/O factories. No production/old-validator monkeypatch.
    monkeypatch.setattr(legacy,'m',m);monkeypatch.setattr(legacy,'PATH',PATH)
    return legacy


@pytest.fixture
def successor_reload(tmp_path,monkeypatch):
    legacy=legacy_module(monkeypatch)
    f=legacy.cwd_reload_runtime.__wrapped__(tmp_path,monkeypatch)
    f.legacy=legacy;revision=f.revision
    parent_output=tmp_path/'accepted_parent';parent_output.mkdir()
    monkeypatch.setattr(m,'PARENT_OUTPUT',parent_output)
    f.relay_argv=list(f.argv)
    f.argv=['/usr/bin/python3','-B',str(m.W/m.PARENT_SELF),'child','--private',
        str(parent_output/'private_start.json'),'--private-sha256','7'*64,'__name:=remote_grasp6d_node']
    (f.proc/'cmdline').write_bytes(b''.join(x.encode()+b'\0' for x in f.argv))
    f.original['cmdline_sha256']=m.sha(f.proc/'cmdline')
    revision.baseline['nodes'][m.REMOTE]=deepcopy(f.original)
    service=dict(unit='accepted-parent.service',properties=dict(MainPID=str(f.original['pid']),
        ActiveState='active',SubState='running',ControlGroup='/accepted-parent',InvocationID='f'*32))
    revision.parent=dict(revision.baseline,remote_service=service)
    revision.parent_chain=dict(child_argv=f.argv,relay_argv=f.relay_argv,
                              working_directory=m.directory_identity(f.working_path))
    revision.stack.properties=lambda unit:deepcopy(service['properties'])
    revision.stack.in_group=lambda pid,group:pid==f.original['pid'] and group=='/accepted-parent'
    f.environment.update(PYTHONPYCACHEPREFIX=str(parent_output/'unused_python_cache'),
        ROS_LOG_DIR=str(parent_output/'remote_ros'),R27_REMOTE_REVISION_SHA256=m.PARENT_REVISION['sha256'])
    (f.proc/'environ').write_bytes(b''.join((k+'='+v).encode()+b'\0' for k,v in f.environment.items()))
    return f


@pytest.mark.parametrize('boundary',['shutdown','launch'])
def test_actual_successor_reload_handles_wrapper_argv_and_original_nonworkspace_cwd(successor_reload,boundary):
    f=successor_reload;f.stop=boundary
    with pytest.raises(f.legacy.CwdLifecycleBoundary,match=boundary+' boundary'):
        m.reload_remote(types.SimpleNamespace())
    construction=m.read_ref(m.ref(f.revision.output/'construction.json'))
    assert construction['original_argv']==f.argv
    assert construction['parent_relay_argv']==f.relay_argv
    assert construction['working_directory']['path']==str(f.working_path)!=str(m.W)
    assert f.shutdown_calls==1
    if boundary=='launch':
        assert f.payload['argv'][:3]==f.relay_argv[:3]
        assert f.payload['parent_shutdown']==m.ref(f.revision.output/'parent_stopped.json')
        assert f.payload['environment']['R27_REMOTE_REVISION_SHA256']==f.revision.reference['sha256']
        # Old --collect service is allowed to vanish after the confirmed exit.
        f.revision.stack.properties=lambda *args:(_ for _ in ()).throw(AssertionError('old unit is gone'))
        assert f.revision.parent_shutdown_snapshot(f.payload['parent_shutdown'],f.payload['construction'])['process_exit_observed']
        assert not (m.PARENT_OUTPUT/'private_start.json').exists()
    else:assert not f.launches
    assert m.read_ref(m.ref(f.revision.output/'reload_result.json'))['passed'] is False


@pytest.mark.parametrize('fault',['argv','ticks','parent','unit','cgroup','cwd','environment','construction'])
def test_successor_parent_drift_is_rejected_before_any_shutdown(successor_reload,fault):
    f=successor_reload
    def drift():
        if fault=='argv':(f.proc/'cmdline').write_bytes(b'/usr/bin/python3\0foreign\0')
        elif fault in ('ticks','parent'):
            f.processes[f.original['pid']]['start_ticks' if fault=='ticks' else 'parent_pid']='reused' if fault=='ticks' else 999
        elif fault=='unit':f.revision.stack.properties=lambda unit:dict(MainPID='foreign')
        elif fault=='cgroup':f.revision.stack.in_group=lambda *args:False
        elif fault=='cwd':
            f.working_path.rename(f.working_path.with_name('detached'));f.working_path.mkdir()
        elif fault=='environment':(f.proc/'environ').write_bytes(b'R27_REMOTE_REVISION_SHA256=foreign\0')
        else:
            p=f.revision.output/'construction.json';p.write_bytes(p.read_bytes()+b'\n')
    f.on_second_wait=drift
    with pytest.raises(RuntimeError):m.reload_remote(types.SimpleNamespace())
    assert f.shutdown_calls==0 and not f.launches


def test_second_remote_may_share_user_systemd_parent_but_not_pid_or_start(monkeypatch):
    legacy=legacy_module(monkeypatch);before,after,service=legacy.nodes()
    after[m.REMOTE]['parent_pid']=before[m.REMOTE]['parent_pid']
    m.validate_transition(before,after,service,True)
    for field in ('pid','start_ticks'):
        bad=deepcopy(after);bad[m.REMOTE][field]=before[m.REMOTE][field]
        with pytest.raises(RuntimeError):m.validate_transition(before,bad,service,True)


def receipt_fixture():
    sources={p:'a'*64 for p in set(m.MODULE_SOURCES.values())|{m.SELF,m.MODE_ENTRY}}
    identity=dict(pid=300,parent_pid=1788,start_ticks='300')
    revision=dict(path='/review/revision.json',sha256='1'*64)
    relay=dict(path=str(m.W/m.RELAY),sha256='2'*64)
    construction=dict(path='/runtime/construction.json',sha256='3'*64)
    directory=dict(path='/original_ros_home',device=1,inode=2)
    receipt=dict(kind='r27_remote_source_start_r4',revision=revision,pid=300,process=identity,
        entry=dict(path=str(m.W/m.SELF),sha256=sources[m.SELF]),relay=relay,
        mode_entry=dict(path=str(m.W/m.MODE_ENTRY),sha256=sources[m.MODE_ENTRY]),
        module_sources=m.expected_module_sources(sources),dependency_bindings=m.expected_entry_bindings(),
        construction=construction,working_directory=directory,parent_acceptance=m.PARENT_ACCEPTANCE,
        parent_shutdown=dict(path='/runtime/parent_stopped.json',sha256='4'*64),
        source_checked_before_original_main=True,original_entry_observed_at_main_guard=True,
        original_compiled_codes_match_frozen_sources=True,temporary_entry_trace_removed_before_main=True,
        memory_attestation=False,actual_compiled_code_sha256=dict(relay='5'*64,mode_entry='6'*64))
    return [receipt,revision,identity,sources,relay,construction,directory]


@pytest.mark.parametrize('fault',[None,'placement_source','placement_binding','parent','shutdown','pid','construction'])
def test_source_receipt_binds_five_modules_and_new_parent_stop(fault):
    args=receipt_fixture();receipt=args[0]
    if fault is None:return m.validate_source_receipt(*args)
    if fault=='placement_source':receipt['module_sources'].pop(m.PLACEMENT_MODULE)
    elif fault=='placement_binding':receipt['dependency_bindings'].pop('placement_dependency')
    elif fault=='parent':receipt['parent_acceptance']={}
    elif fault=='shutdown':receipt['parent_shutdown']={'path':'/foreign/parent_stopped.json'}
    elif fault=='pid':receipt['pid']=42
    else:receipt['construction']={}
    with pytest.raises(RuntimeError):m.validate_source_receipt(*args)


def test_actual_original_remote_ast_imports_resolve_to_preloaded_placement_functions():
    relative='src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py'
    path=m.W/relative;tree=ast.parse(path.read_bytes(),filename=str(path))
    class_node=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='RemoteGrasp6DNode')
    selected=[n for n in class_node.body if isinstance(n,ast.FunctionDef)
              and n.name in ('_contact_joint_window_proposal','_contact_height_recipe_parameters')]
    assert len(selected)==2
    # Compile actual production method AST, not a mirror of deferred imports.
    original=types.ModuleType('actual_original_methods');original.__file__=str(path)
    cls=ast.ClassDef(name='RemoteGrasp6DNode',bases=[],keywords=[],body=selected,decorator_list=[])
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[])),str(path),'exec'),original.__dict__)
    placement=types.ModuleType(m.PLACEMENT_MODULE)
    exec(compile('def contact_joint_placement(): pass\ndef joint_placement_policy_parameters(): pass\n',
                 str(m.W/m.PLACEMENT),'exec'),placement.__dict__)
    node=type('ModeAwareRemoteGrasp6DNode',(original.RemoteGrasp6DNode,),{})
    namespace=dict(original=original,ModeAwareRemoteGrasp6DNode=node)
    assert m.placement_entry_binding(namespace,{m.PLACEMENT_MODULE:placement})==m.expected_placement_binding()
    placement.contact_joint_placement=lambda:None
    with pytest.raises(RuntimeError,match='preloaded source'):
        m.placement_entry_binding(namespace,{m.PLACEMENT_MODULE:placement})


def successor_recorder_graph(legacy,f,monkeypatch):
    ctx,master=legacy._original_graph_factory(f,monkeypatch)
    revision=ctx.rev;revision.parent=dict(revision.baseline,remote_service={'unit':'old','properties':{}})
    revision.entry=m.load_source(m.ENTRY,{m.ENTRY:m.sha(m.W/m.ENTRY)})
    simulation_values=dict(zip(m.SIMULATION_POLICY_KEYS,(False,80,'http://127.0.0.1:8002')))
    revision.config['parameters'].update(simulation_values)
    f.actual_simulation_values=deepcopy(simulation_values)
    master.getParam=lambda name:deepcopy(f.actual_simulation_values[name])
    path=revision.output/'construction.json';construction=json.loads(path.read_bytes())
    construction['kind']='r27_remote_only_construction_r4';path.write_text(json.dumps(construction))
    construction_ref=m.ref(path)
    stop=revision.output/'parent_stopped.json'
    stop.write_text(json.dumps(dict(kind='r27_successor_parent_stopped_r4',revision=revision.reference,
        construction=construction_ref,parent_acceptance=m.PARENT_ACCEPTANCE,
        parent_process=revision.parent['nodes'][m.REMOTE],parent_service=revision.parent['remote_service'],
        process_exit_observed=True,forced_termination=False)))
    (revision.output/'source_start.json').write_text(json.dumps(dict(construction=construction_ref,parent_shutdown=m.ref(stop))))
    return ctx,master


def test_original_session_observe_context_and_successor_graph_keep_recorder_ownership(tmp_path,monkeypatch):
    legacy=legacy_module(monkeypatch)
    legacy._original_graph_factory=legacy.recorder_runtime_graph
    monkeypatch.setattr(legacy,'recorder_runtime_graph',lambda f,mp:successor_recorder_graph(legacy,f,mp))
    # Real Session.preflight/observe -> r4 AttemptContext.recheck -> r4 graph.
    # The reused factory explicitly isolates source admission/sensor/RPC I/O;
    # separate real parent and source-receipt cases above test those contracts.
    legacy.test_original_preflight_observe_rechecks_the_live_ready_recorder_before_mode_transfer(tmp_path,monkeypatch)


@pytest.mark.parametrize('change',['foreign_owner','extra_control_publisher','extra_control_service'])
def test_successor_whole_attempt_graph_rejects_foreign_owners(tmp_path,monkeypatch,change):
    legacy=legacy_module(monkeypatch);legacy._original_graph_factory=legacy.recorder_runtime_graph
    monkeypatch.setattr(legacy,'recorder_runtime_graph',lambda f,mp:successor_recorder_graph(legacy,f,mp))
    legacy.test_whole_attempt_graph_still_rejects_other_owners(tmp_path,monkeypatch,change)


def prepare_successor_child(f,monkeypatch,*,fault=None):
    """Real r4 child lifecycle with minimal entries; no ROS/source-admission claim.

    The OS/release boundaries are the reload I/O fixture. Both inherited remote
    methods and both placement export definitions are compiled from production
    AST. They are observed, never called; the four earlier modules use the
    existing minimal binding fixture. Actual runpy, trace, hashes, parent-stop
    checks, source receipt validation and write-before-main are not replaced.
    """
    args,entered,proc=f.legacy.prepare_cwd_child(f,monkeypatch)
    original_path=ORIGINAL_WORKSPACE/'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py'
    tree=ast.parse(original_path.read_bytes(),filename=str(original_path))
    node=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='RemoteGrasp6DNode')
    names={'_contact_joint_window_proposal','_contact_height_recipe_parameters'}
    methods=[n for n in node.body if isinstance(n,ast.FunctionDef) and n.name in names]
    assert len(methods)==2
    original=types.ModuleType('r4_minimal_original_methods')
    cls=ast.ClassDef(name='RemoteGrasp6DNode',bases=[],keywords=[],body=methods,decorator_list=[])
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[])),
                 str(original_path),'exec'),original.__dict__)
    monkeypatch.setitem(sys.modules,original.__name__,original)
    placement_path=m.W/m.PLACEMENT
    production_source=(ORIGINAL_WORKSPACE/m.PLACEMENT).read_text()
    tree=ast.parse(production_source)
    definitions=[n for n in tree.body if isinstance(n,ast.FunctionDef)
                 and n.name in {'contact_joint_placement','joint_placement_policy_parameters'}]
    assert len(definitions)==2
    source='\n\n'.join(ast.get_source_segment(production_source,n) for n in definitions)+'\n'
    placement_path.write_text(source)
    placement=sys.modules[m.PLACEMENT_MODULE]
    exec(compile(source,str(placement_path),'exec'),placement.__dict__)
    f.revision.sources[m.PLACEMENT]=m.sha(placement_path)
    mode_path=m.W/m.MODE_ENTRY
    mode=mode_path.read_text().replace(
        'class ModeAwareRemoteGrasp6DNode(',
        'import r4_minimal_original_methods as original\nclass ModeAwareRemoteGrasp6DNode(original.RemoteGrasp6DNode, ')
    # Main can only run after the real before-main callback saved its receipt.
    mode=mode.replace("if __name__ == '__main__':",
        "if __name__ == '__main__':\n    assert __import__('os').path.isfile(%r)" % str(f.revision.output/'source_start.json'))
    mode_path.write_text(mode);f.revision.sources[m.MODE_ENTRY]=m.sha(mode_path)
    if fault=='placement_identity':placement.contact_joint_placement=lambda:None
    elif fault=='placement_bytes':placement_path.write_text(source+'# changed after frozen hash\n')
    elif fault=='parent_stop_bytes':
        path=Path(f.payload['parent_shutdown']['path']);path.write_bytes(path.read_bytes()+b'\n')
    elif fault=='parent_stop_claim':
        path=Path(f.payload['parent_shutdown']['path']);value=json.loads(path.read_bytes())
        value['process_exit_observed']=False;path.write_text(json.dumps(value))
        f.payload['parent_shutdown']=m.ref(path);f.private.write_text(json.dumps(f.payload))
        args.private_sha256=m.sha(f.private)
    return args,entered,proc


def test_actual_successor_child_observes_five_modules_and_parent_stop_before_original_main(successor_reload,monkeypatch):
    f=successor_reload;args,entered,proc=prepare_successor_child(f,monkeypatch)
    m.child(args)
    receipt=m.read_ref(m.ref(f.revision.output/'source_start.json'))
    assert entered.read_text()=='entered' and not f.private.exists()
    assert receipt['module_sources']==m.expected_module_sources(f.revision.sources)
    assert len(receipt['module_sources'])==5
    assert receipt['dependency_bindings']['placement_dependency']==m.expected_placement_binding()
    assert receipt['parent_acceptance']==m.PARENT_ACCEPTANCE
    assert receipt['parent_shutdown']==f.payload['parent_shutdown']
    assert receipt['construction']==f.payload['construction']
    assert receipt['working_directory']==m.directory_identity(f.working_path)
    assert receipt['source_checked_before_original_main'] and not receipt['memory_attestation']
    assert sys.gettrace() is None


@pytest.mark.parametrize('fault',['placement_identity','placement_bytes','parent_stop_bytes','parent_stop_claim'])
def test_actual_successor_child_rejects_placement_or_parent_stop_before_main(successor_reload,monkeypatch,fault):
    f=successor_reload;args,entered,_=prepare_successor_child(f,monkeypatch,fault=fault)
    with pytest.raises(RuntimeError):m.child(args)
    assert not entered.exists() and not (f.revision.output/'source_start.json').exists()
    assert sys.gettrace() is None


@pytest.fixture
def regression_review_pair(tmp_path,monkeypatch,real_parent):
    """Metadata binding fixture, not a fabricated regression execution proof."""
    archive=tmp_path/'archive';maintenance=archive/'maintenance/r27'
    monkeypatch.setattr(m,'ARCHIVE',archive);monkeypatch.setattr(m,'MAINTENANCE',maintenance)
    baseline_ref=real_parent.data['baseline_release'];baseline=m.read_ref(baseline_ref)
    sources=dict(m.read_ref(m.PARENT_RELEASE)['source_sha256'])
    for path in m.REQUIRED_SUCCESSOR_CHANGES:sources[path]='a'*64
    extras=[m.REGRESSION_WRAPPER,m.PARENT_SELF,m.PARENT_TEST,m.SELF,m.TEST]
    tests=[m.PARENT_TEST,m.TEST]
    def save(path,value):
        path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(value));return m.ref(path)
    wrapper_review=dict(kind='r27_offline_source_review',reviewed=True,baseline_release=baseline_ref,
        source_sha256=sources,changed_sources=m.source_delta(baseline['source_sha256'],sources),
        additional_source_files=extras,additional_test_files=tests)
    wrapper_path=maintenance/'regression_review.json';wrapper_ref=save(wrapper_path,wrapper_review)
    review=dict(source_sha256=sources,regression_source_review=wrapper_ref)
    name='isolated_metadata_case';result_path=archive/'ros_log/integrated_regression'/name/'result.json'
    run=dict(source_sha256=sources,explicit_test_files=tests,test_files=tests,
        complete_scope_requested=True,offline_scope_complete=True,source_unchanged=True,changed_sources=[])
    result_ref=save(result_path,run);binding=dict(backend='fixture_ssd_identity')
    receipt=dict(source_review=wrapper_ref,
        entry=dict(path=str(m.W/m.REGRESSION_WRAPPER),sha256=sources[m.REGRESSION_WRAPPER]),
        original_runner=dict(path=str(m.W/m.ORIGINAL_RUNNER),sha256=sources[m.ORIGINAL_RUNNER]),
        storage_binding=binding,manifest_extensions=extras,scope_extensions=tests,
        declared_test_file_count=len(tests),source_count=len(sources),
        sandbox_and_reconciliation_unchanged=True,all_original_test_files_preserved=True,
        hardware_or_network_operations_authorized=False,
        argv=['--name',name,'--storage-backend','ssd','--jobs','1'])
    receipt_path=maintenance/'regression_entry_receipts'/(name+'.json')
    receipt_ref=save(receipt_path,receipt)
    return types.SimpleNamespace(review=review,run=run,reference=result_ref,binding=binding,
        wrapper_review=wrapper_review,wrapper_path=wrapper_path,receipt=receipt,
        receipt_path=receipt_path,receipt_ref=receipt_ref,save=save)


def test_two_reviews_retain_distinct_baselines_and_bind_actual_wrapper_entry(regression_review_pair):
    f=regression_review_pair
    proof=m.verify_regression_review(f.review,f.reference,f.run,f.binding,f.receipt_ref)
    assert proof['distinct_review_baselines_verified']
    assert proof['regression_entry_receipt']==f.receipt_ref


@pytest.mark.parametrize('fault',['parent_as_wrapper_baseline','source_map','missing_old_helper',
    'missing_wrapper','missing_old_test','wrong_entry','wrong_storage','scope_count','source_count',
    'pilot','not_complete','source_changed','entry_receipt_replaced','wrong_result_root'])
def test_dual_review_binding_rejects_mixed_history_or_incomplete_runtime_scope(regression_review_pair,fault):
    f=regression_review_pair;expected=None
    if fault=='parent_as_wrapper_baseline':f.wrapper_review['baseline_release']=m.PARENT_RELEASE
    elif fault=='source_map':f.wrapper_review['source_sha256']=dict(f.wrapper_review['source_sha256'],foreign='0'*64)
    elif fault in ('missing_old_helper','missing_wrapper'):
        f.wrapper_review['additional_source_files']=[p for p in f.wrapper_review['additional_source_files']
            if p!=(m.PARENT_SELF if fault=='missing_old_helper' else m.REGRESSION_WRAPPER)]
    elif fault=='missing_old_test':f.wrapper_review['additional_test_files']=[m.TEST]
    elif fault=='wrong_entry':f.receipt['entry']['sha256']='0'*64
    elif fault=='wrong_storage':f.receipt['storage_binding']={}
    elif fault=='scope_count':f.receipt['declared_test_file_count']-=1
    elif fault=='source_count':f.receipt['source_count']-=1
    elif fault=='pilot':f.receipt['argv']+=['--pilot-file',m.TEST]
    elif fault=='not_complete':f.run['complete_scope_requested']=False
    elif fault=='source_changed':f.run['changed_sources']=[m.PLACEMENT]
    elif fault=='entry_receipt_replaced':
        expected=f.receipt_ref;f.receipt['additional_unreviewed_field']=True
    else:f.reference=dict(f.reference,path=str(Path(f.reference['path']).parent.parent/'result.json'))
    f.review['regression_source_review']=f.save(f.wrapper_path,f.wrapper_review)
    f.receipt['source_review']=f.review['regression_source_review'];f.save(f.receipt_path,f.receipt)
    with pytest.raises(RuntimeError):
        m.verify_regression_review(f.review,f.reference,f.run,f.binding,expected)


@pytest.mark.parametrize('fault',[None,'unknown_lift_gate_enabled','min_score','server_url'])
def test_live_policy_binds_original_simulation_gate_separately_from_prediction_service(tmp_path,real_parent,fault):
    parameters={
        '/gui/joint_direct_mode':True,
        '/alicia_d_driver_node/software_temperature_monitoring_enabled':False,
        '/alicia_d_driver_node/temperature_poll_rate_hz':0.,
        '/alicia_d_driver_node/self_check_poll_rate_hz':0.,
        '/camera/simulate':False,'/camera/fallback_to_simulation':False,'/camera/mode_depth_preset':'fixture',
        '/mujoco_digital_twin/unknown_lift_gate_enabled':False,
        '/mujoco_digital_twin/min_score':80,'/mujoco_digital_twin/server_url':'http://127.0.0.1:8002',
        '/grasp/unknown_lift_after_close_enabled':False,'/grasp/unknown_observe_return_max_observations':4}
    config=dict(parameters=parameters);actual=deepcopy(parameters)
    handeye=tmp_path/'handeye.yaml';handeye.write_text('handeye: {fixture: true}\n')
    model='<robot name="policy_fixture"/>'
    actual.update({'/grasp':real_parent.entry.bound_parameter_tree(parameters)['grasp'],
                   '/handeye':{'fixture':True},'/robot_description':model})
    if fault:
        actual['/mujoco_digital_twin/'+fault]={'unknown_lift_gate_enabled':True,'min_score':79,
            'server_url':'http://different.invalid:8002'}[fault]
    revision=object.__new__(m.Revision);revision.session=tmp_path;revision.config=config
    revision.entry=real_parent.entry
    revision.stack=types.SimpleNamespace(configuration=lambda **kwargs:config,HAND_EYE=handeye,SIM='fixture',
        load=lambda source:types.SimpleNamespace(IDENTITY=dict(candidate_model_sha256=m.hashlib.sha256(model.encode()).hexdigest())))
    master=types.SimpleNamespace(getParam=lambda name:deepcopy(actual[name]))
    if fault:
        with pytest.raises(RuntimeError,match='accepted runtime policy changed: /mujoco_digital_twin/'):
            revision.live_policy(master)
    else:
        values=revision.live_policy(master)
        assert values['/mujoco_digital_twin/unknown_lift_gate_enabled'] is False
        assert values['/mujoco_digital_twin/server_url']=='http://127.0.0.1:8002'


def successor_session(tmp_path,monkeypatch,*,manual):
    """Actual Session.observe/Context/graph; only sensor/RPC/source I/O fixture."""
    legacy=legacy_module(monkeypatch);legacy._original_graph_factory=legacy.recorder_runtime_graph
    f=legacy.ready_recording(tmp_path,monkeypatch)
    ctx,master=successor_recorder_graph(legacy,f,monkeypatch);entry=f.revision.entry
    bootstrap=types.ModuleType('alicia_flexible_grasp.robot.actuation_bootstrap')
    bootstrap.PENDING_BOOTSTRAP_STATES=frozenset()
    monkeypatch.setitem(sys.modules,bootstrap.__name__,bootstrap)
    monkeypatch.setitem(sys.modules,'alicia_grasp_modes.selection',legacy.selection)
    monkeypatch.setattr(entry,'validate_automatic_entry_snapshot',lambda *args:dict(sensor_fixture=True))
    ctx.acceptance_ref=dict(path='/accepted.json',sha256='a'*64)
    ctx.release_ref=dict(path='/release.json',sha256='b'*64);ctx.instruction_ref=f.instruction;ctx.revision_ref=None
    ctx.boundary=types.SimpleNamespace(validate_manual_return_snapshot=lambda *args:dict(sensor_fixture=True))
    session=entry.Session(ctx,f.record)
    caller='/current_recorded_grasp_'+str(m.os.getpid())+'_1793674000000'
    now=m.time.time_ns();stamp=types.SimpleNamespace(to_nsec=lambda:100)
    topics=[('joint','/alicia_d/accepted_joint_states',object),('epoch','/alicia_d/control_reference_epoch',object),
        ('manual','/gui/joint_direct_mode',object),('task','/grasp/state',object)]
    messages=dict(joint=types.SimpleNamespace(),epoch=types.SimpleNamespace(stamp=stamp),
        manual=types.SimpleNamespace(data=manual),task=types.SimpleNamespace(active=False))
    selected=dict(mode='unknown',strategy='observe_return',generation=1,stamp_ns=100)
    session.rospy=types.SimpleNamespace(get_name=lambda:caller,is_shutdown=lambda:False,
        Time=types.SimpleNamespace(now=lambda:types.SimpleNamespace(to_nsec=lambda:now)),
        get_param=lambda name:selected if name=='/grasp_mode/selection' else manual,
        wait_for_message=lambda topic,kind,timeout:messages[next(k for k,n,_ in topics if n==topic)])
    session.master=master;session.topics=topics
    session.samples=[dict(stamp_ns=now-i) for i in range(20)]
    session.states={k:(value,now) for k,value in messages.items()}
    session.policy=lambda **kwargs:dict(sensor_fixture=True)
    session.receive=lambda key,message:None
    f.state[1].append(['/attempt_observation',[caller]])
    return f,ctx,session


@pytest.mark.parametrize('fault',[None,'unknown_lift_gate_enabled','min_score','server_url'])
def test_actual_session_rechecks_simulation_policy_with_gui_already_automatic(tmp_path,monkeypatch,fault):
    f,ctx,session=successor_session(tmp_path,monkeypatch,manual=False)
    # A retained simulation configuration passes after the acknowledged mode
    # transfer. Reusing live_policy here would incorrectly require manual=True.
    first=session.observe();assert first['manual_parameter'] is False
    if fault:
        f.actual_simulation_values['/mujoco_digital_twin/'+fault]={
            'unknown_lift_gate_enabled':True,'min_score':79,'server_url':'http://different.invalid:8002'}[fault]
        with pytest.raises(RuntimeError,match='accepted runtime policy changed: /mujoco_digital_twin/'):
            session.observe()
    else:
        assert session.observe()['manual_parameter'] is False
        ctx.recheck(session.master,session.rospy.get_name())


def test_actual_session_manual_cleanup_does_not_require_simulation_or_recorder(tmp_path,monkeypatch):
    f,ctx,session=successor_session(tmp_path,monkeypatch,manual=True)
    def forbidden(*args,**kwargs):raise AssertionError('simulation/compute/recorder must not gate manual cleanup')
    f.revision.simulation_policy=forbidden;f.revision.graph=forbidden
    ctx.recorder.check=forbidden;ctx.recorder.graph_owner=forbidden
    ctx.health.observe=forbidden;ctx.health.check_binding=forbidden
    assert session.observe(manual_return=True)['manual_parameter'] is True
