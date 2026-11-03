"""Offline negative/ownership contracts for the finite R27 runtime adapter."""
import ast
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import runpy
import sys
import types

import pytest


PATH = Path(__file__).resolve().parents[1]/'remote_only_revision_r3.py'
spec = importlib.util.spec_from_file_location('r27_remote_revision_r3_under_test', PATH)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
selection_spec=importlib.util.spec_from_file_location('r27_original_selection',m.W/m.SELECTION)
selection=importlib.util.module_from_spec(selection_spec)
selection_spec.loader.exec_module(selection)


def sources():
    before = {p:h for p,h in m.BASELINE_PRODUCTION.items() if h is not None}
    before['src/retained_driver.cpp']='0'*64
    for p in m.BUSINESS_TESTS:
        if '/test_reachable_' in p:before[p]='a'*64
    after = dict(before, **{p:'1'*64 for p in m.BASELINE_PRODUCTION},
                 **{p:'2'*64 for p in m.BUSINESS_TESTS}, **{m.SELF:'3'*64,m.TEST:'4'*64})
    review = dict(kind='r27_offline_source_review',reviewed=True,
        baseline_release=dict(path='/frozen/r26.json',sha256=m.BASELINE_RELEASE_SHA),
        source_sha256=after,changed_sources=m.source_delta(before,after),
        additional_source_files=[m.SELF,m.TEST],additional_test_files=[m.TEST])
    return before,after,review


def nodes():
    before = {('/retained_%d'%i):dict(pid=100+i,parent_pid=90,start_ticks=str(900+i),
               cmdline_sha256='0'*64,uri='http://retained-%d/'%i) for i in range(16)}
    before[m.REMOTE] = dict(pid=200,parent_pid=90,start_ticks='1000',cmdline_sha256='1'*64,uri='http://old/')
    after = deepcopy(before)
    after[m.REMOTE] = dict(pid=300,parent_pid=80,start_ticks='2000',cmdline_sha256='2'*64,uri='http://new/')
    service = dict(ActiveState='active',SubState='running',MainPID='300',ControlGroup='/new.service')
    return before,after,service


def test_exact_five_production_and_ten_business_test_delta_is_accepted():
    before,after,review=sources()
    assert m.validate_source_delta(before,after,review)==m.source_delta(before,after)


@pytest.mark.parametrize('path',[
    'src/alicia_flexible_grasp_supervisor/scripts/motion_gateway_node.py',
    'src/real-arm/alicia_d_driver/src/alicia_d_driver_node.cpp',
    'src/alicia_grasp_modes/scripts/mode_aware_grasp_task.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/runner_contract.py',
    'tools/full_stack_start.py', 'tools/run_recorded_manual_grasp.py',
    'src/alicia_grasp_modes/tests/test_silhouette_observation_planning.py',
])
def test_a_review_cannot_extend_the_production_or_test_delta(path):
    before,after,review=sources();after[path]='5'*64
    review['source_sha256']=after;review['changed_sources']=m.source_delta(before,after)
    with pytest.raises(RuntimeError,match='exceeds reviewed five-source'):
        m.validate_source_delta(before,after,review)


@pytest.mark.parametrize('path',sorted(set(m.BASELINE_PRODUCTION) | m.BUSINESS_TESTS))
def test_each_business_change_and_its_test_is_required(path):
    before,after,review=sources()
    if path in before:after[path]=before[path]
    else:after.pop(path)
    review['changed_sources']=m.source_delta(before,after)
    with pytest.raises(RuntimeError,match='must change together'):
        m.validate_source_delta(before,after,review)


@pytest.mark.parametrize('path',sorted(m.BASELINE_PRODUCTION))
def test_each_original_production_hash_is_pinned(path):
    before,after,review=sources();before[path]='f'*64
    review['changed_sources']=m.source_delta(before,after)
    with pytest.raises(RuntimeError,match='unexpected original'):
        m.validate_source_delta(before,after,review)


@pytest.mark.parametrize('path',sorted(m.ABANDONED))
def test_abandoned_compute_pair_adapter_cannot_be_added_to_review(path):
    before,after,review=sources();after[path]='5'*64
    review['additional_source_files'].append(path)
    review['changed_sources']=m.source_delta(before,after)
    with pytest.raises(RuntimeError,match='abandoned'):
        m.validate_source_delta(before,after,review)


def test_a_deleted_retained_source_cannot_be_hidden_in_a_finite_review():
    before,after,review=sources();after.pop('src/retained_driver.cpp')
    review['changed_sources']=m.source_delta(before,after)
    with pytest.raises(RuntimeError):m.validate_source_delta(before,after,review)


@pytest.mark.parametrize('path',[
    '/tmp/unreviewed.py', '../outside.py', '.runtime/r27_observation_planning_20261102/../outside.py',
    '.runtime/old_generation/tool.py', 'paper_innovation_validation_20261027/research.py',
])
def test_extra_deployment_source_cannot_escape_this_repair(path):
    before,after,review=sources();review['additional_source_files'].append(path)
    with pytest.raises(RuntimeError,match='escapes R27'):
        m.validate_source_delta(before,after,review)


@pytest.mark.parametrize('kind', ['unreviewed','wrong_baseline','wrong_hash','unfrozen_adapter','duplicate'])
def test_review_identity_and_each_helper_are_pinned(kind):
    before,after,review=sources()
    if kind=='unreviewed':review['reviewed']=False
    elif kind=='wrong_baseline':review['baseline_release']['sha256']='f'*64
    elif kind=='wrong_hash':review['changed_sources'][m.PRODUCTION]['after_sha256']='f'*64
    elif kind=='unfrozen_adapter':review['additional_source_files'].remove(m.SELF)
    else:review['additional_source_files'].append(m.TEST)
    with pytest.raises(RuntimeError):m.validate_source_delta(before,after,review)


def test_new_remote_with_independent_parent_and_unit_preserves_all_other_nodes():
    before,after,service=nodes()
    m.validate_transition(before,after,service,True)


@pytest.mark.parametrize('index',range(16))
@pytest.mark.parametrize('field',['pid','parent_pid','start_ticks','cmdline_sha256','uri'])
def test_any_retained_identity_change_rejects_the_transition(index,field):
    before,after,service=nodes()
    after['/retained_%d'%index][field] = 987 if field in ('pid','parent_pid') else 'changed'
    with pytest.raises(RuntimeError,match='preserved node identity changed'):
        m.validate_transition(before,after,service,True)


@pytest.mark.parametrize('change',['same_pid','same_ticks','same_parent','foreign_unit','inactive','outside_cgroup','missing','extra'])
def test_runtime_transition_rejects_duplicate_stale_or_foreign_ownership(change):
    before,after,service=nodes();membership=True
    if change=='same_pid':after[m.REMOTE]['pid']=before[m.REMOTE]['pid']
    elif change=='same_ticks':after[m.REMOTE]['start_ticks']=before[m.REMOTE]['start_ticks']
    elif change=='same_parent':after[m.REMOTE]['parent_pid']=before[m.REMOTE]['parent_pid']
    elif change=='foreign_unit':service['MainPID']='999'
    elif change=='inactive':service['ActiveState']='inactive'
    elif change=='outside_cgroup':membership=False
    elif change=='missing':after.pop('/retained_0')
    else:after['/uninvited']=deepcopy(after[m.REMOTE])
    with pytest.raises(RuntimeError):m.validate_transition(before,after,service,membership)


def test_evidence_bytes_are_checked_before_json_is_used(tmp_path):
    path=tmp_path/'evidence.json';path.write_text('{"passed": false}')
    frozen=m.ref(path);assert m.read_ref(frozen)==dict(passed=False)
    path.write_text('{"passed": true}')
    with pytest.raises(RuntimeError,match='bound evidence changed'):m.read_ref(frozen)


def test_live_command_line_rejects_unrecognized_actions_and_extra_flags():
    with pytest.raises(SystemExit):m.parser().parse_args(['cold-scan'])
    with pytest.raises(RuntimeError,match='unexpected CLI'):
        m.main(['child','--private','/not-read','--private-sha256','0'*64,'--prepare-lease'])


def test_import_is_offline_and_does_not_import_ros_or_production_modules():
    tree=ast.parse(PATH.read_text())
    top_imports={x.names[0].name.split('.')[0] for x in tree.body if isinstance(x,ast.Import)}
    top_imports|={x.module.split('.')[0] for x in tree.body if isinstance(x,ast.ImportFrom)}
    assert not top_imports.intersection({'rospy','rosgraph','alicia_d_driver','alicia_flexible_grasp','alicia_grasp_modes'})
    # A new adapter must never replace validation functions in the originals.
    assignments=[n for n in ast.walk(tree) if isinstance(n,(ast.Assign,ast.AnnAssign))]
    targets=[]
    for node in assignments:targets.extend(node.targets if isinstance(node,ast.Assign) else [node.target])
    forbidden={'cold_scan','validate_contract','source_check','node_snapshot','validate_state','observed_ready',
               'validate_automatic_entry_snapshot','attempt_lifecycle'}
    assert not any(isinstance(n,ast.Attribute) and n.attr in forbidden for n in targets)
    assert not any(isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='cold_scan'
                   for n in ast.walk(tree))


def test_manual_return_checks_retained_control_owners_without_requiring_remote_health():
    calls=[]
    ctx=object.__new__(m.AttemptContext)
    ctx.validate_contract=lambda manual_return=False:calls.append(('validate',manual_return))
    ctx.acceptance=dict(core_unit='core',core_service={'unit':'core'},master_identity={'pid':1},
        nodes={'/driver':{'pid':2},'/gui':{'pid':3},'/motion_gateway':{'pid':4}})
    ctx.stack=types.SimpleNamespace(properties=lambda _:dict(unit='core'),process=lambda _:dict(pid=1))
    ctx.idle=types.SimpleNamespace(OWNERS={'sdk':'/driver','manual':'/gui'})
    def node(master,name,caller):calls.append(('node',name));return ctx.acceptance['nodes'][name]
    def forbidden(*args):raise AssertionError('compute graph/health must not gate manual cleanup')
    ctx.rev=types.SimpleNamespace(node=node,graph=forbidden)
    ctx.health=types.SimpleNamespace(check_binding=forbidden,observe=forbidden)
    ctx.recorder=types.SimpleNamespace(check=forbidden,graph_owner=forbidden)
    ctx.recheck(types.SimpleNamespace(getPid=lambda:1),'test',manual_return=True)
    assert calls==[('validate',True),('node','/driver'),('node','/gui'),('node','/motion_gateway')]


def test_manual_return_still_rejects_changed_control_owner():
    ctx=object.__new__(m.AttemptContext)
    ctx.validate_contract=lambda **kw:None
    ctx.acceptance=dict(core_unit='core',core_service={},master_identity={'pid':1},
        nodes={'/driver':{'pid':2},'/motion_gateway':{'pid':4}})
    ctx.stack=types.SimpleNamespace(properties=lambda _:dict(),process=lambda _:dict(pid=1))
    ctx.idle=types.SimpleNamespace(OWNERS={'sdk':'/driver'})
    ctx.rev=types.SimpleNamespace(node=lambda *args:dict(pid=999))
    with pytest.raises(RuntimeError,match='manual-return owner changed'):
        ctx.recheck(types.SimpleNamespace(getPid=lambda:1),'test',manual_return=True)


@pytest.mark.parametrize('wire_stamp,reply_stamp',[
    ('1793673973096937656',1793673973096937656),
    (1793673973096937656,'1793673973096937656'),
    ('1793673973096937656','1793673973096937656'),
])
def test_reset_ack_uses_original_exact_nanosecond_selection_contract(wire_stamp,reply_stamp):
    frozen=dict(mode='unknown',strategy='observe_return',generation=2,stamp_ns=wire_stamp)
    ack=dict(frozen,stamp_ns=reply_stamp,ready=True)
    response=types.SimpleNamespace(success=True,message=json.dumps(ack))
    assert m.validate_reset_response(response,frozen,selection.parse_selection)==ack


@pytest.mark.parametrize('field,value',[
    ('stamp_ns',1793673973096937657),('stamp_ns',1793673973096937656.0),
    ('stamp_ns','1793673973096937656.0'),('generation',3),('generation',True),
    ('mode','carton'),('strategy','direct'),('ready',False),
])
def test_reset_ack_normalization_never_relaxes_any_selection_identity(field,value):
    frozen=dict(mode='unknown',strategy='observe_return',generation=2,stamp_ns='1793673973096937656')
    ack=dict(frozen,stamp_ns=1793673973096937656,ready=True);ack[field]=value
    with pytest.raises((RuntimeError,ValueError)):
        m.validate_reset_response(types.SimpleNamespace(success=True,message=json.dumps(ack)),frozen,selection.parse_selection)


def chain_fixture():
    output=Path('/verified/ssd/ros_log/session/remote_r27')
    revision=dict(path='/verified/revision.json',sha256='1'*64)
    release=dict(path='/verified/release.json',sha256='2'*64)
    contract=dict(path='/verified/original_contract.json',sha256='3'*64)
    review=dict(path='/verified/root_review.json',sha256='4'*64)
    reload_ref=dict(path=str(output/'reload_result.json'),sha256='5'*64)
    acceptance_ref=dict(path=str(output/'acceptance/result.json'),sha256='6'*64)
    before,after,service=nodes()
    common=dict(passed=True,revision=revision,execution_review=review,nodes=after,auxiliary_nodes={},
        remote_service=dict(unit='r27.service',properties=service),state=dict(epoch_ns=42),
        construction=dict(path=str(output/'construction.json'),sha256='9'*64),
        source_startup_receipt=dict(path=str(output/'source_start.json'),sha256='8'*64),
        remote_startup_binding=dict(path=str(output/'remote_startup_binding.json'),sha256='7'*64))
    deployed=dict(common,kind='r27_remote_only_reload_r3')
    accepted=deepcopy(dict(common,kind='r27_remote_only_acceptance_r3',typed_initial_contract_ready=True,
        reload=reload_ref,source_manifest=release,contract=contract))
    return [acceptance_ref,accepted,revision,output,release,contract,review,reload_ref,deployed]


def test_actual_typed_acceptance_and_reload_share_one_immutable_revision_chain():
    m.validate_acceptance_chain(*chain_fixture())


@pytest.mark.parametrize('case',[
    'foreign_acceptance_path','foreign_reload_path','old_full_acceptance','untyped','failed_acceptance',
    'failed_reload','foreign_reload_revision','different_reload_ref','different_release','different_contract',
    'different_review','different_remote','different_alias','different_unit','different_binding','different_epoch',
    'different_source_receipt','foreign_source_receipt_path','old_reload_kind','foreign_reload_review',
    'different_construction','foreign_construction_path',
])
def test_each_acceptance_reload_link_rejects_mismatch(case):
    args=chain_fixture();accepted,deployed=args[1],args[8]
    if case=='foreign_acceptance_path':args[0]['path']='/other/result.json'
    elif case=='foreign_reload_path':args[7]['path']='/other/reload_result.json'
    elif case=='old_full_acceptance':accepted['kind']='manual_full_stack_acceptance'
    elif case=='untyped':accepted['typed_initial_contract_ready']=False
    elif case=='failed_acceptance':accepted['passed']=False
    elif case=='failed_reload':deployed['passed']=False
    elif case=='foreign_reload_revision':deployed['revision']=dict(path='/other',sha256='0'*64)
    elif case=='different_reload_ref':accepted['reload']['sha256']='0'*64
    elif case=='different_release':accepted['source_manifest']['sha256']='0'*64
    elif case=='different_contract':accepted['contract']['sha256']='0'*64
    elif case=='different_review':accepted['execution_review']['sha256']='0'*64
    elif case=='different_remote':accepted['nodes'][m.REMOTE]['pid']=999
    elif case=='different_alias':accepted['auxiliary_nodes']['foreign']={}
    elif case=='different_unit':accepted['remote_service']['unit']='another.service'
    elif case=='different_binding':accepted['remote_startup_binding']['sha256']='0'*64
    elif case=='different_epoch':accepted['state']['epoch_ns']=43
    elif case=='different_source_receipt':accepted['source_startup_receipt']['sha256']='0'*64
    elif case=='foreign_source_receipt_path':deployed['source_startup_receipt']['path']='/other/source_start.json'
    elif case=='old_reload_kind':deployed['kind']='r27_remote_only_reload'
    elif case=='foreign_reload_review':deployed['execution_review']['sha256']='0'*64
    elif case=='different_construction':accepted['construction']['sha256']='0'*64
    elif case=='foreign_construction_path':deployed['construction']['path']='/other/construction.json'
    with pytest.raises(RuntimeError):m.validate_acceptance_chain(*args)


def alias_fixture():
    gateway=dict(pid=16013,parent_pid=15932,start_ticks='584538',cmdline_sha256='c'*64,
                 uri='http://gateway:45779/')
    aliases={'/move_group_commander_wrappers_1793674560303727341':
             dict(gateway,uri='http://gateway:40123/')}
    return gateway,aliases


@pytest.mark.parametrize('present',[False,True])
def test_initial_snapshot_can_capture_a_lazy_helper_in_the_unchanged_gateway(present):
    gateway,aliases=alias_fixture()
    if not present:aliases={}
    frozen=deepcopy(aliases)
    assert m.validate_gateway_aliases(aliases,gateway,None,capture=True)==frozen
    assert m.validate_gateway_aliases(aliases,gateway,frozen)==frozen


@pytest.mark.parametrize('field',['pid','parent_pid','start_ticks','cmdline_sha256'])
def test_initial_alias_capture_never_admits_a_foreign_process(field):
    gateway,aliases=alias_fixture()
    aliases[next(iter(aliases))][field]='foreign' if isinstance(gateway[field],str) else 999
    with pytest.raises(RuntimeError,match='foreign gateway alias'):
        m.validate_gateway_aliases(aliases,gateway,None,capture=True)


@pytest.mark.parametrize('fault',['wrong_name','two_aliases','removed','added','renamed','uri_changed',
                                'unfrozen','capture_overwrite'])
def test_frozen_alias_snapshot_rejects_membership_uri_changes_and_unbound_rechecks(fault):
    gateway,aliases=alias_fixture();frozen=deepcopy(aliases);name=next(iter(aliases));capture=False
    if fault=='wrong_name':aliases['/foreign']=aliases.pop(name)
    elif fault=='two_aliases':aliases['/move_group_commander_wrappers_2']=deepcopy(aliases[name])
    elif fault=='removed':aliases={}
    elif fault=='added':frozen={}
    elif fault=='renamed':aliases['/move_group_commander_wrappers_2']=aliases.pop(name)
    elif fault=='uri_changed':aliases[name]['uri']='http://gateway:another/'
    elif fault=='unfrozen':frozen=None
    elif fault=='capture_overwrite':capture=True
    with pytest.raises(RuntimeError):m.validate_gateway_aliases(aliases,gateway,frozen,capture=capture)


def alias_construction_fixture():
    gateway,aliases=alias_fixture();output=Path('/ssd/ros_log/session/r27')
    revision=dict(path='/review/revision.json',sha256='a'*64)
    reference=dict(path=str(output/'construction.json'),sha256='b'*64)
    construction=dict(kind='r27_remote_only_construction_r3',revision=deepcopy(revision),
        gateway_identity=deepcopy(gateway),baseline_auxiliary_nodes={},auxiliary_nodes=deepcopy(aliases))
    return [reference,construction,revision,output,{},gateway]


def test_current_alias_construction_preserves_the_empty_historical_acceptance():
    args=alias_construction_fixture();before=deepcopy(args[4])
    assert m.validate_alias_construction(*args)==args[1]['auxiliary_nodes']
    assert args[4]==before=={}


@pytest.mark.parametrize('change',['path','kind','revision','gateway','baseline','foreign_alias'])
def test_alias_snapshot_cannot_be_rebound_to_a_different_construction(change):
    args=alias_construction_fixture();reference,construction=args[:2]
    if change=='path':reference['path']='/other/construction.json'
    elif change=='kind':construction['kind']='unreviewed'
    elif change=='revision':construction['revision']['sha256']='0'*64
    elif change=='gateway':construction['gateway_identity']['start_ticks']='changed'
    elif change=='baseline':construction['baseline_auxiliary_nodes']={'unexpected':{}}
    elif change=='foreign_alias':construction['auxiliary_nodes'][next(iter(construction['auxiliary_nodes']))]['pid']=99
    with pytest.raises(RuntimeError):m.validate_alias_construction(*args)


@pytest.mark.parametrize('kwargs',[
    dict(remote={},expected_aliases=None,capture_aliases=True),
    dict(service={},expected_aliases=None,capture_aliases=True),
    dict(remote_absent=True,expected_aliases=None,capture_aliases=True),
    dict(expected_aliases={},capture_aliases=True),
])
def test_graph_cannot_capture_new_aliases_after_remote_transition_started(kwargs):
    revision=object.__new__(m.Revision)
    with pytest.raises(RuntimeError,match='only allowed with the original remote'):
        revision.graph(None,'caller',**kwargs)


def test_every_graph_call_requires_an_explicit_alias_snapshot_argument():
    with pytest.raises(TypeError,match='expected_aliases'):
        object.__new__(m.Revision).graph(None,'caller')


def source_receipt_fixture():
    _before,source_map,_review=sources()
    _old,new,_service=nodes();identity=new[m.REMOTE]
    revision=dict(path='/verified/revision.json',sha256='9'*64)
    relay=dict(path=str(m.W/m.RELAY),sha256='e'*64)
    construction=dict(path='/verified/runtime/construction.json',sha256='c'*64)
    working_directory=dict(path='/original/ros_home',device=2053,inode=123)
    receipt=dict(kind='r27_remote_source_start_r3',revision=revision,pid=identity['pid'],process=deepcopy(identity),
        entry=dict(path=str(m.W/m.SELF),sha256=source_map[m.SELF]),relay=relay,
        mode_entry=dict(path=str(m.W/m.MODE_ENTRY),sha256=source_map[m.MODE_ENTRY]),
        module_sources=m.expected_module_sources(source_map),dependency_bindings=m.expected_entry_bindings(),
        source_checked_before_original_main=True,original_entry_observed_at_main_guard=True,
        original_compiled_codes_match_frozen_sources=True,temporary_entry_trace_removed_before_main=True,
        actual_compiled_code_sha256=dict(relay='1'*64,mode_entry='2'*64),
        construction=deepcopy(construction),working_directory=deepcopy(working_directory),
        memory_attestation=False)
    return [receipt,deepcopy(revision),identity,source_map,deepcopy(relay),construction,working_directory]


def test_source_receipt_binds_actual_entry_four_dependencies_and_remote_instance():
    m.validate_source_receipt(*source_receipt_fixture())


@pytest.mark.parametrize('field',[
    'kind','revision','pid','process','entry','relay','mode_entry','module_sources','dependency_bindings',
    'source_checked_before_original_main','original_entry_observed_at_main_guard',
    'original_compiled_codes_match_frozen_sources','temporary_entry_trace_removed_before_main','memory_attestation',
    'actual_compiled_code_sha256','construction','working_directory',
])
def test_any_missing_or_foreign_source_receipt_field_rejects(field):
    args=source_receipt_fixture();receipt=args[0]
    if field in ('pid',):receipt[field]=999
    elif field=='kind':receipt[field]='old_planner_preload'
    elif field=='process':receipt[field]['start_ticks']='9999'
    elif field in ('entry','relay','mode_entry','revision','construction'):receipt[field]['sha256']='0'*64
    elif field in ('module_sources','dependency_bindings','actual_compiled_code_sha256','working_directory'):receipt[field]={}
    else:receipt[field]=not receipt[field]
    with pytest.raises(RuntimeError):m.validate_source_receipt(*args)


def synthetic_bindings():
    modules={name:types.ModuleType(name) for name in m.MODULE_SOURCES}
    planner=modules['alicia_grasp_modes.silhouette_observation_planning']
    remote=modules['alicia_grasp_modes.observe_return_remote']
    refresh=modules['alicia_grasp_modes.observation_source_refresh']
    planner.SilhouetteObservationPlanningMixin=type('SilhouetteObservationPlanningMixin',(),{})
    remote.ObserveReturnRemoteMixin=type('ObserveReturnRemoteMixin',(),{})
    for symbols in m.REFRESH_BINDINGS.values():
        for symbol in symbols:setattr(refresh,symbol,object())
    for module_name,symbols in m.REFRESH_BINDINGS.items():
        for symbol in symbols:setattr(modules[module_name],symbol,getattr(refresh,symbol))
    fk=modules['alicia_grasp_modes.reachable_observation_views']
    fk.FK_PROPOSAL_BUDGET_SEC=5.;fk.generate_reachable_observation_views=lambda **kw:None
    namespace=dict(__name__='__main__',__file__=str(m.W/m.MODE_ENTRY),
        SilhouetteObservationPlanningMixin=planner.SilhouetteObservationPlanningMixin,
        ObserveReturnRemoteMixin=remote.ObserveReturnRemoteMixin)
    source='''class ModeAwareRemoteGrasp6DNode(SilhouetteObservationPlanningMixin, ObserveReturnRemoteMixin):
    def _joint_space_observation_candidates(self, prepared):
        from alicia_grasp_modes.reachable_observation_views import (
            FK_PROPOSAL_BUDGET_SEC, generate_reachable_observation_views)
'''
    exec(compile(source,str(m.W/m.MODE_ENTRY),'exec'),namespace)
    return namespace,modules


def test_actual_entry_class_and_refresh_functions_use_the_preloaded_objects(tmp_path,monkeypatch):
    monkeypatch.setattr(m,'W',tmp_path)
    namespace,modules=synthetic_bindings()
    assert m.validate_entry_bindings(namespace,modules)==m.expected_entry_bindings()
    assert not m.validate_entry_bindings(namespace,modules)['deferred_fk_import']['executed_by_startup_observer']


@pytest.mark.parametrize('change',[
    'foreign_namespace','wrong_entry','unrelated_class','mixin_shadowed','refresh_shadowed','foreign_fk_method',
    'fk_export_absent','wrong_deferred_module',
])
def test_loading_an_unused_module_alone_is_not_actual_entry_binding(change,tmp_path,monkeypatch):
    monkeypatch.setattr(m,'W',tmp_path)
    namespace,modules=synthetic_bindings()
    if change=='foreign_namespace':namespace['__name__']='preload_only'
    elif change=='wrong_entry':namespace['__file__']='/wrong.py'
    elif change=='unrelated_class':namespace['ModeAwareRemoteGrasp6DNode']=type('Unrelated',(),{})
    elif change=='mixin_shadowed':namespace['ObserveReturnRemoteMixin']=object()
    elif change=='refresh_shadowed':modules['alicia_grasp_modes.observe_return_remote'].request_binding=object()
    elif change=='foreign_fk_method':namespace['ModeAwareRemoteGrasp6DNode']._joint_space_observation_candidates=lambda self,p:None
    elif change=='fk_export_absent':del modules['alicia_grasp_modes.reachable_observation_views'].generate_reachable_observation_views
    else:
        exec(compile('def foreign(self, p):\n    from unrelated import FK_PROPOSAL_BUDGET_SEC, generate_reachable_observation_views\n',
                     str(m.W/m.MODE_ENTRY),'exec'),namespace)
        namespace['ModeAwareRemoteGrasp6DNode']._joint_space_observation_candidates=namespace['foreign']
    with pytest.raises(RuntimeError):m.validate_entry_bindings(namespace,modules)


@pytest.mark.parametrize('change',['none','wrong_path','wrong_hash','foreign_sys_module','missing_module'])
def test_four_preloaded_modules_keep_exact_file_hash_and_interpreter_ownership(tmp_path,monkeypatch,change):
    monkeypatch.setattr(m,'W',tmp_path)
    modules={};source_map={}
    for name,path in m.MODULE_SOURCES.items():
        file=tmp_path/path;file.parent.mkdir(parents=True,exist_ok=True);file.write_text('value=1\n')
        module=types.ModuleType(name);module.__file__=str(file)
        modules[name]=module;source_map[path]=m.sha(file);monkeypatch.setitem(sys.modules,name,module)
    name=next(iter(m.MODULE_SOURCES));path=m.MODULE_SOURCES[name]
    if change=='wrong_path':modules[name].__file__=str(tmp_path/'foreign.py')
    elif change=='wrong_hash':(tmp_path/path).write_text('value=2\n')
    elif change=='foreign_sys_module':monkeypatch.setitem(sys.modules,name,types.ModuleType(name))
    elif change=='missing_module':modules.pop(name)
    if change=='none':assert m.validate_loaded_modules(modules,source_map)==m.expected_module_sources(source_map)
    else:
        with pytest.raises(RuntimeError):m.validate_loaded_modules(modules,source_map)


@pytest.mark.parametrize('source',[
    'pass\n',"if __name__ == 'something_else':\n    pass\n", "if __name__ == '__main__':\n    pass\npass\n",
])
def test_no_alternate_main_guard_can_silently_bypass_the_entry_observer(source):
    with pytest.raises(RuntimeError):m.main_guard_line(source,'entry.py')


@pytest.mark.parametrize('change',['mode_before_relay','wrong_mode_code','wrong_relay_code','repeated_relay'])
def test_entry_audit_rejects_foreign_code_at_original_paths_or_wrong_sequence(change):
    relay=compile('pass','/relay.py','exec');mode=compile('pass','/mode.py','exec')
    observed=m.entry_observer(relay,mode,1,lambda *args:None,lambda value:None)
    if change!='mode_before_relay':observed.audit('exec',(relay,))
    with pytest.raises(RuntimeError):
        if change=='wrong_relay_code':observed.audit('exec',(compile('value=1','/relay.py','exec'),))
        elif change=='repeated_relay':observed.audit('exec',(relay,))
        else:observed.audit('exec',(compile('value=1','/mode.py','exec') if change=='wrong_mode_code' else mode,))


def test_entry_audit_does_not_observe_or_block_unrelated_legitimate_loading():
    observed=m.entry_observer(compile('pass','/relay.py','exec'),compile('pass','/mode.py','exec'),
        1,lambda *args:None,lambda value:None)
    observed.audit('compile',(b'arbitrary','/mode.py'))
    observed.audit('exec',(compile('value=1','/unrelated.py','exec'),))
    assert not observed.state['relay_seen'] and not observed.state['mode_seen']


@pytest.mark.parametrize('receipt_fails',[False,True])
def test_real_runpy_relay_compiler_observation_precedes_main_and_removes_trace(tmp_path,receipt_fails):
    relay_path=tmp_path/'relay.py';mode_path=tmp_path/'original_mode.py'
    mode="value=1\nif __name__ == '__main__':\n    entered_main=True\n"
    mode_path.write_text(mode)
    relay="with open(%r, 'r') as f:\n    exec(compile(f.read(), %r, 'exec'), {'__name__': __name__, '__file__': %r})\n" % (
        str(mode_path),str(mode_path),str(mode_path))
    relay_path.write_text(relay)
    observed_namespaces=[];previous_trace=sys.gettrace()
    def complete(namespace,state):
        assert sys.gettrace() is None
        assert 'entered_main' not in namespace
        observed_namespaces.append(namespace)
        if receipt_fails:raise RuntimeError('synthetic source receipt refused')
    observer=m.entry_observer(compile(relay,str(relay_path),'exec'),compile(mode,str(mode_path),'exec'),
        m.main_guard_line(mode,str(mode_path)),complete,sys.settrace)
    sys.addaudithook(observer.audit)
    try:
        if receipt_fails:
            with pytest.raises(RuntimeError,match='synthetic source receipt refused'):
                runpy.run_path(str(relay_path),run_name='__main__')
            assert 'entered_main' not in observed_namespaces[0]
            assert not observer.state['completed']
        else:
            runpy.run_path(str(relay_path),run_name='__main__')
            assert observed_namespaces[0]['entered_main'] is True
            assert observer.state['completed']
        assert sys.gettrace() is None
    finally:
        observer.state['completed']=True
        sys.settrace(previous_trace)


def ready_recording(tmp_path, monkeypatch):
    """The original wrapper's READY receipt and three real-process boundaries."""
    archive=tmp_path/'grasp_records';record=archive/'current_record'
    record.mkdir(parents=True);output=tmp_path/'runtime';output.mkdir()
    caller_pid,parent_pid,writer_pid=m.os.getpid(),m.os.getppid(),900001
    instruction=dict(path='/instructions/current.json',sha256='9'*64)
    attempt_argv=['/usr/bin/python3','-B',str(m.W/m.SELF),'attempt',
                  '--instruction',instruction['path'],'--instruction-sha256',instruction['sha256']]
    wrapper_argv=['/usr/bin/python3','-B',str(m.W/m.RECORD_LAUNCHER),
                  '--storage-backend','ssd','--root',str(archive),'--duration','600','--',*attempt_argv]
    config=dict(min_free_bytes=4831838208,upload_enabled=False,delete_enabled=False,archive_policy='local_only')
    binding=dict(backend='ssd',device='test-device',root=str(tmp_path))
    recorder_argv=['/usr/bin/python3','-u',str(m.W/m.RECORDER),'--record-dir',str(record),
        '--output',str(record/'keyframes.bag'),'--duration','600.0','--defer-finalize',
        '--config',str(archive/'archive_config.json'),'--min-free-bytes',str(config['min_free_bytes'])]
    writer_environment=dict(GRASP_RECORD_DIR=str(record),GRASP_RECORD_ROOT=str(archive),
        ROS_LOG_DIR=str(record/'recorder_ros'),ALICIA_STORAGE_BACKEND='ssd',
        ALICIA_STORAGE_IDENTITY=json.dumps(binding),PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1')
    def evidence(pid,parent,ticks,argv,environment):
        raw=b''.join(arg.encode()+b'\0' for arg in argv)
        return dict(identity=dict(pid=pid,parent_pid=parent,start_ticks=ticks,
                                  cmdline_sha256=m.hashlib.sha256(raw).hexdigest()),
                    argv=argv,environment=environment)
    processes=dict(wrapper=evidence(parent_pid,999,'1000',wrapper_argv,{}),
        recorder=evidence(writer_pid,parent_pid,'1001',recorder_argv,writer_environment),
        attempt=evidence(caller_pid,parent_pid,'1002',attempt_argv,{}))
    ready=dict(pid=writer_pid,record_dir=str(record),bag_path=str(record/'keyframes.bag'),started_at='current')
    manifest=dict(schema=1,record_id=record.name,state='recording',recorder_closed=False,
        completed_at=None,local_data_deleted=False,
        owner=dict(pid=parent_pid,start_ticks='1000',boot_id='current-boot'),
        metadata=dict(owner_pid=parent_pid,launcher='run_recorded_grasp.py',storage=binding,
                      recording_duration_seconds=600.,command=attempt_argv))
    for path,value in ((record/'recorder_ready.json',ready),(record/'manifest.json',manifest),
                       (archive/'archive_config.json',config)):
        path.write_text(json.dumps(value))
    revision=object.__new__(m.Revision)
    revision.storage=types.SimpleNamespace(archive_root=archive,ros_log_root=tmp_path,
                                          validate=lambda *args,**kw:binding)
    revision.binding=binding;revision.contract=dict(cold_boot_id='current-boot')
    revision.sources={path:m.sha(m.W/path) for path in (m.RECORDER,m.RECORD_LAUNCHER,'tools/grasp_archive.py')}
    revision.stack=types.SimpleNamespace();revision.output=output
    revision.reference=dict(path='/revisions/current.json',sha256='8'*64)
    def current_process(_stack,pid):
        return deepcopy(next(row for row in processes.values() if row['identity']['pid']==pid))
    monkeypatch.setattr(m,'recorder_process_evidence',current_process)
    recorder=m.RecorderBinding(revision,record,instruction)
    name='/grasp_keyframe_recorder_readonly_'+str(writer_pid)+'_1793674000000'
    node=dict(processes['recorder']['identity'],uri='http://recorder:30001/')
    state=[[['/rosout',[name]]],
           [['/grasp/state',[name]],['/alicia_d/accepted_joint_states',[name]],['/tf',[name]]],
           [[name+'/get_loggers',[name]],[name+'/set_logger_level',[name]]]]
    revision.node=lambda master,n,caller:deepcopy(node)
    return types.SimpleNamespace(revision=revision,recorder=recorder,record=record,ready=ready,
        manifest=manifest,config=config,processes=processes,archive=archive,binding=binding,
        name=name,node=node,state=state,instruction=instruction)


def test_ready_recorder_captures_one_literal_ros_instance_and_receipt(tmp_path,monkeypatch):
    f=ready_recording(tmp_path,monkeypatch)
    assert f.recorder.graph_owner(None,'/attempt',f.state)==f.name
    receipt=m.read_ref(f.recorder.reference)
    assert receipt['ros_owner']['node']==f.node and receipt['recorder_ready']==m.ref(f.record/'recorder_ready.json')
    assert receipt['processes']['wrapper']['identity']['start_ticks']==f.manifest['owner']['start_ticks']
    assert receipt['memory_attestation'] is False
    assert f.recorder.graph_owner(None,'/attempt',f.state)==f.name


@pytest.mark.parametrize('change',[
    'foreign_parent','reused_writer_pid','reused_wrapper_pid','wrong_boot','other_record','other_bag',
    'short_duration','wrapper_short_duration','other_entry','added_command_flag','other_config',
    'reduced_free_reserve','missing_defer_finalize','other_storage','wrong_recorder_environment',
    'different_attempt_command','already_finalized','wrapper_without_no_bytecode','recorder_starts_after_attempt',
])
def test_ready_chain_rejects_foreign_reused_or_unrecorded_attempts(tmp_path,monkeypatch,change):
    f=ready_recording(tmp_path,monkeypatch)
    if change=='foreign_parent':f.processes['recorder']['identity']['parent_pid']+=1
    elif change=='reused_writer_pid':f.processes['recorder']['identity']['start_ticks']='1003'
    elif change=='reused_wrapper_pid':f.processes['wrapper']['identity']['start_ticks']='999'
    elif change=='wrong_boot':f.manifest['owner']['boot_id']='earlier-boot'
    elif change=='other_record':f.ready['record_dir']=str(f.archive/'older_record')
    elif change=='other_bag':f.ready['bag_path']=str(f.record/'other.bag')
    elif change=='short_duration':f.manifest['metadata']['recording_duration_seconds']=60.
    elif change=='wrapper_short_duration':
        argv=f.processes['wrapper']['argv'];argv[argv.index('--duration')+1]='60'
    elif change=='other_entry':f.processes['recorder']['argv'][2]='/tmp/lookalike_recorder.py'
    elif change=='added_command_flag':f.processes['recorder']['argv'].append('__name:=fake')
    elif change=='other_config':
        argv=f.processes['recorder']['argv'];argv[argv.index('--config')+1]='/tmp/other.json'
    elif change=='reduced_free_reserve':f.processes['recorder']['argv'][-1]='0'
    elif change=='missing_defer_finalize':f.processes['recorder']['argv'].remove('--defer-finalize')
    elif change=='other_storage':f.manifest['metadata']['storage']={}
    elif change=='wrong_recorder_environment':f.processes['recorder']['environment']['GRASP_RECORD_DIR']='/old'
    elif change=='different_attempt_command':f.manifest['metadata']['command']=['/usr/bin/true']
    elif change=='already_finalized':f.manifest['state']='complete'
    elif change=='wrapper_without_no_bytecode':f.processes['wrapper']['argv'].remove('-B')
    else:f.processes['recorder']['identity']['start_ticks']='1003'
    with pytest.raises(RuntimeError):
        m.validate_recorder_chain(f.ready,f.manifest,f.record,f.archive,f.binding,f.config,f.processes,'current-boot')


@pytest.mark.parametrize('field',['pid','parent_pid','start_ticks','cmdline_sha256','uri'])
def test_recorder_ros_process_replacement_after_capture_is_rejected(tmp_path,monkeypatch,field):
    f=ready_recording(tmp_path,monkeypatch);f.recorder.graph_owner(None,'/attempt',f.state)
    f.node[field]=321 if field in ('pid','parent_pid') else 'replaced'
    with pytest.raises(RuntimeError):f.recorder.graph_owner(None,'/attempt',f.state)


@pytest.mark.parametrize('change',['control_publisher','control_service','shared_logger_service',
                                  'additional_logger_name','removed_subscription','missing_node','second_same_pid_name'])
def test_recorder_exception_cannot_hide_control_ownership_or_new_nodes(tmp_path,monkeypatch,change):
    f=ready_recording(tmp_path,monkeypatch)
    if change not in ('second_same_pid_name',):f.recorder.graph_owner(None,'/attempt',f.state)
    if change=='control_publisher':f.state[0].append(['/joint_commands',[f.name]])
    elif change=='control_service':f.state[2].append(['/grasp/start',[f.name]])
    elif change=='shared_logger_service':f.state[2][0][1].append('/foreign')
    elif change=='additional_logger_name':f.state[2].append([f.name+'/tf2_frames',[f.name]])
    elif change=='removed_subscription':f.state[1].pop()
    elif change=='missing_node':f.state[:]=[[],[],[]]
    else:f.state[1].append(['/tf',[f.name[:-1]+'1']])
    with pytest.raises(RuntimeError):f.recorder.graph_owner(None,'/attempt',f.state)


@pytest.mark.parametrize('change',['ready_bytes','manifest_bytes','writer_ticks','wrapper_ticks','recording_ended','config_bytes'])
def test_frozen_ready_recording_is_rechecked_at_each_attempt_boundary(tmp_path,monkeypatch,change):
    f=ready_recording(tmp_path,monkeypatch);f.recorder.graph_owner(None,'/attempt',f.state)
    if change=='ready_bytes':(f.record/'recorder_ready.json').write_text('{}')
    elif change=='manifest_bytes':(f.record/'manifest.json').write_text('{}')
    elif change=='config_bytes':(f.archive/'archive_config.json').write_text('{}')
    elif change=='recording_ended':(f.record/'recorder_outcome.json').write_text('{}')
    else:f.processes['recorder' if change=='writer_ticks' else 'wrapper']['identity']['start_ticks']='99999'
    with pytest.raises(RuntimeError):f.recorder.graph_owner(None,'/attempt',f.state)


@pytest.mark.parametrize('missing',['recorder_ready.json','manifest.json'])
def test_attempt_has_no_recorder_exception_without_current_recording_evidence(tmp_path,monkeypatch,missing):
    f=ready_recording(tmp_path,monkeypatch);(f.record/missing).unlink()
    with pytest.raises((OSError,RuntimeError)):
        m.RecorderBinding(f.revision,f.record,f.instruction)


def recorder_runtime_graph(f,monkeypatch):
    """Use the actual r3 graph; replace only process/ROS/storage I/O boundaries."""
    revision=f.revision
    retained=['/motion_gateway','/alicia_d_driver_node','/grasp_task_node','/supervisor_camera',
              '/alicia_supervisor_gui','/grasp_mode_router']+['/retained_'+str(i) for i in range(10)]
    before={name:dict(pid=100+i,parent_pid=90,start_ticks=str(100+i),cmdline_sha256='1'*64,
                      uri='http://node-'+str(i)+'/') for i,name in enumerate(retained)}
    before[m.REMOTE]=dict(pid=200,parent_pid=90,start_ticks='200',cmdline_sha256='2'*64,uri='http://old/')
    current=deepcopy(before)
    current[m.REMOTE]=dict(pid=300,parent_pid=80,start_ticks='300',cmdline_sha256='3'*64,uri='http://new/')
    alias='/move_group_commander_wrappers_123'
    aliases={alias:dict(current['/motion_gateway'],uri='http://gateway-alias/')}
    core=dict(MainPID='90',InvocationID='c'*32,ControlGroup='/core',ActiveState='active',SubState='running')
    remote_service=dict(MainPID='300',InvocationID='r'*32,ControlGroup='/remote',ActiveState='active',SubState='running')
    simulation=dict(pid=500,invocation_id='s'*32,control_group='/simulation')
    revision.baseline=dict(nodes=before,core_unit='core',core_service=core,master_identity=dict(pid=50),simulation=simulation)
    revision.startup=dict(components=dict(simulation=dict(unit='simulation')))
    revision.contract.update(pycache_prefix='/old/cache',current_sha256={str(m.W/m.RELAY):'4'*64})
    revision.session=Path('/old/session')
    revision.config=dict(parameters={'/camera/color_topic':'/supervisor/camera/color/image_raw',
                                     '/camera/depth_topic':'/supervisor/camera/depth/image_raw'})
    properties={'core':core,'remote':remote_service,'simulation':dict(MainPID='500',InvocationID='s'*32,ControlGroup='/simulation')}
    working_path=revision.output/'original_ros_home';working_path.mkdir()
    working_directory=m.directory_identity(working_path)
    construction=dict(kind='r27_remote_only_construction_r3',revision=revision.reference,
        original_remote=before[m.REMOTE],working_directory=working_directory)
    construction_path=revision.output/'construction.json';construction_path.write_text(json.dumps(construction))
    remote_proc=revision.output/'proc'/str(current[m.REMOTE]['pid']);remote_proc.mkdir(parents=True)
    (remote_proc/'cwd').symlink_to(working_path,target_is_directory=True)
    (remote_proc/'cmdline').write_bytes(b'/usr/bin/python3\0-B\0/r3/child.py\0')
    current[m.REMOTE]['cmdline_sha256']=m.sha(remote_proc/'cmdline')
    f.working_path=working_path;f.remote_proc=remote_proc;f.unit_directory=working_directory['path']
    def process(pid):
        if pid==current[m.REMOTE]['pid']:
            return {key:current[m.REMOTE][key] for key in ('pid','parent_pid','start_ticks')}
        return dict(pid=50)
    revision.stack=types.SimpleNamespace(properties=lambda unit:deepcopy(properties[unit]),
                                        process=process,in_group=lambda *args:True)
    identities=dict(current,**aliases);identities[f.name]=f.node
    revision.node=lambda master,name,caller:deepcopy(identities[name])
    (revision.output/'source_start.json').write_text(json.dumps(dict(construction=m.ref(construction_path))))
    monkeypatch.setattr(m,'validate_source_receipt',lambda *args:None)
    old_environment=dict(PYTHONPYCACHEPREFIX='/old/cache',PYTHONDONTWRITEBYTECODE='1',
                         PYTHONNOUSERSITE='1',ROS_LOG_DIR='/old/session/core_ros')
    new_environment=dict(PYTHONPYCACHEPREFIX=str(revision.output/'unused_python_cache'),
        PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1',ROS_LOG_DIR=str(revision.output/'remote_ros'),
        R27_REMOTE_REVISION_SHA256=revision.reference['sha256'])
    real_path=Path
    class ProcPath:
        def __init__(self,parts=()):self.parts=parts
        def __truediv__(self,part):return ProcPath(self.parts+(str(part),))
        def __fspath__(self):return str(revision.output/'proc'/Path(*self.parts))
        def stat(self):return Path(self.__fspath__()).stat()
        def read_bytes(self):
            if self.parts[-1]=='cmdline':return Path(self.__fspath__()).read_bytes()
            assert self.parts[-1]=='environ'
            environment=new_environment if self.parts[0]=='300' else old_environment
            return b''.join((key+'='+value).encode()+b'\0' for key,value in environment.items())
    monkeypatch.setattr(m,'Path',lambda value:ProcPath() if str(value)=='/proc' else real_path(value))
    def unit_directory(command,**kwargs):
        assert command==['systemctl','--user','show','remote','--property=WorkingDirectory','--value']
        return f.unit_directory+'\n'
    monkeypatch.setattr(m.subprocess,'check_output',unit_directory)
    owners={'/alicia_d/accepted_joint_states':'/alicia_d_driver_node','/grasp/state':'/grasp_task_node',
        '/alicia_d/control_reference_epoch':'/alicia_d_driver_node','/alicia_d/motion_enabled':'/alicia_d_driver_node',
        '/alicia_d/actuation_status':'/alicia_d_driver_node','/supervisor/camera/color/image_raw':'/supervisor_camera',
        '/supervisor/camera/depth/image_raw':'/supervisor_camera','/gui/joint_direct_mode':'/alicia_supervisor_gui'}
    for topic,owner in owners.items():f.state[0].append([topic,[owner]])
    f.state[1].append(['/retained_registry',list(current)+list(aliases)])
    master=types.SimpleNamespace(getSystemState=lambda:deepcopy(f.state),getPid=lambda:50)
    acceptance=dict(nodes=current,auxiliary_nodes=aliases,remote_service=dict(unit='remote',properties=remote_service),
        core_unit='core',core_service=core,master_identity=dict(pid=50),state=dict(epoch_ns=100))
    ctx=object.__new__(m.AttemptContext)
    ctx.rev=revision;ctx.stack=revision.stack;ctx.recorder=f.recorder;ctx.acceptance=acceptance
    ctx.validate_contract=lambda **kw:None
    ctx.health=types.SimpleNamespace(check_binding=lambda *args:None,observe=lambda *args:{},
        load_binding=lambda *args:{},evaluate=lambda *args:dict(passed=True,finished_ns=m.time.time_ns()))
    ctx.remote={};ctx.remote_ref=dict(path='/current/remote.json',sha256='5'*64)
    ctx.idle=types.SimpleNamespace(OWNERS=dict(joint='/alicia_d_driver_node',epoch='/alicia_d_driver_node',
                                             manual='/alicia_supervisor_gui',task='/grasp_task_node'))
    return ctx,master


@pytest.mark.parametrize('stage',['reload','accept'])
def test_recorder_is_not_an_exception_during_reload_or_accept(tmp_path,monkeypatch,stage):
    f=ready_recording(tmp_path,monkeypatch);ctx,master=recorder_runtime_graph(f,monkeypatch)
    kwargs={} if stage=='reload' else dict(remote=ctx.acceptance['nodes'][m.REMOTE],service=ctx.acceptance['remote_service'])
    with pytest.raises(RuntimeError,match='missing or extra owners'):
        f.revision.graph(master,'/caller',expected_aliases=ctx.acceptance['auxiliary_nodes'],**kwargs)


@pytest.mark.parametrize('change',['foreign_owner','extra_control_publisher','extra_control_service'])
def test_whole_attempt_graph_still_rejects_other_owners(tmp_path,monkeypatch,change):
    f=ready_recording(tmp_path,monkeypatch);ctx,master=recorder_runtime_graph(f,monkeypatch)
    if change=='foreign_owner':f.state[1].append(['/tf',['/another_observer']])
    elif change=='extra_control_publisher':f.state[0].append(['/joint_commands',[f.name]])
    else:f.state[2].append(['/grasp/start',[f.name]])
    with pytest.raises(RuntimeError):ctx.recheck(master,'/attempt')


def test_original_preflight_observe_rechecks_the_live_ready_recorder_before_mode_transfer(tmp_path,monkeypatch):
    f=ready_recording(tmp_path,monkeypatch);ctx,master=recorder_runtime_graph(f,monkeypatch)
    entry=m.load_source(m.ENTRY,{m.ENTRY:m.sha(m.W/m.ENTRY)})
    # The test drives the original preflight and observe, the r3 Context and
    # its real whole-graph method. Only sensor/RPC/process boundaries are fake.
    bootstrap=types.ModuleType('alicia_flexible_grasp.robot.actuation_bootstrap')
    bootstrap.PENDING_BOOTSTRAP_STATES=frozenset()
    monkeypatch.setitem(sys.modules,bootstrap.__name__,bootstrap)
    monkeypatch.setitem(sys.modules,'alicia_grasp_modes.selection',selection)
    monkeypatch.setattr(entry,'validate_automatic_entry_snapshot',lambda *args:{'offline_fixture':True})
    ctx.acceptance_ref=dict(path='/accepted.json',sha256='a'*64);ctx.release_ref=dict(path='/release.json',sha256='b'*64)
    ctx.instruction_ref=f.instruction;ctx.revision_ref=None;ctx.boundary=object()
    session=entry.Session(ctx,f.record)
    caller='/current_recorded_grasp_'+str(m.os.getpid())+'_1793674000000'
    now=m.time.time_ns();stamp=types.SimpleNamespace(to_nsec=lambda:100)
    topics=[('joint','/alicia_d/accepted_joint_states',object),('epoch','/alicia_d/control_reference_epoch',object),
            ('manual','/gui/joint_direct_mode',object),('task','/grasp/state',object)]
    messages=dict(joint=types.SimpleNamespace(),epoch=types.SimpleNamespace(stamp=stamp),
                  manual=types.SimpleNamespace(data=True),task=types.SimpleNamespace(active=False))
    selected=dict(mode='unknown',strategy='observe_return',generation=1,stamp_ns=100)
    ros=types.SimpleNamespace(get_name=lambda:caller,is_shutdown=lambda:False,
        Time=types.SimpleNamespace(now=lambda:types.SimpleNamespace(to_nsec=lambda:now)),
        get_param=lambda name:selected if name=='/grasp_mode/selection' else True,
        wait_for_message=lambda topic,kind,timeout:messages[next(key for key,name,_ in topics if name==topic)])
    def connect():
        session.rospy=ros;session.master=master;session.topics=topics
        session.samples=[dict(stamp_ns=now-i) for i in range(20)]
        session.states={key:(message,now) for key,message in messages.items()}
        f.state[1].append(['/attempt_observation',[caller]])
    session.connect=connect;session.policy=lambda **kw:{}
    session.receive=lambda key,message:None
    saves=[];session.save=lambda:saves.append(deepcopy(session.report))
    session.preflight()
    assert saves[-1]['before']['manual_parameter'] is True
    assert f.recorder.reference is not None and session.automatic_requested is False
    # The imports-only prewarm creates no ROS node; the subsequent original
    # observe/select rechecks therefore see the same exact recorded graph.
    session.observe();ctx.recheck(master,caller)
    assert f.recorder.owner['name']==f.name and ctx.acceptance['auxiliary_nodes']


def sealed_case_ledger():
    files=['src/example/tests/test_retained.py','tools/tests/test_legacy.py']
    cases=[dict(nodeid=files[0]+'::test_retained',setup='passed',call='passed',teardown='passed'),
           dict(nodeid=files[1]+'::test_platform',setup='passed',call='skipped',teardown='passed',
                skip_reason='original platform guard')]
    run=dict(passed=True,source_unchanged=True,external_boundary_attempts=[],bad_files=[],duplicate_cases=[],
        collection_skips=[],complete_scope_requested=True,offline_scope_complete=True,changed_sources=[],
        source_sha256={path:'1'*64 for path in files},test_files=files,case_results=cases,
        expected_case_ids=[row['nodeid'] for row in cases],test_roots=['src','tools'],explicit_test_files=[],
        excluded_files=[],scope_name='sealed historical test fixture',full_regression_passed=False,
        summary=dict(tests=2,passed=1,skipped=1,failed=0))
    release=dict(schema=1,kind='current_offline_release',passed=True,reviewed=True,installed=True,
        source_count=2,source_sha256=deepcopy(run['source_sha256']),full_regression_passed=False,
        actual_regression=dict(path='/historical/sealed.json',sha256='2'*64))
    entry=m.load_source(m.ENTRY,{m.ENTRY:m.sha(m.W/m.ENTRY)})
    stack=types.SimpleNamespace(CURRENT_TEST_ROOTS=('src','tools'),HISTORICAL_PREWARM_TEST='.runtime/test_prewarm.py')
    return release,run,entry,stack


def test_historical_scope_uses_sealed_inventory_and_preserves_real_skip_semantics(monkeypatch):
    release,run,entry,stack=sealed_case_ledger()
    def current_tree_is_not_the_historical_tree(*args,**kwargs):
        raise AssertionError('historical validation must not rediscover the installed R27 tree')
    with monkeypatch.context() as context:
        context.setattr(Path,'rglob',current_tree_is_not_the_historical_tree)
        proof=m.sealed_release_evidence(release,run,entry,stack)
    assert proof['current_summary']==dict(tests=2,passed=1,skipped=1,failures=0,errors=0)
    assert proof['current_full_regression_passed'] is False and proof['test_bodies_executed'] is False
    assert proof['skipped_cases']==[dict(nodeid=run['case_results'][1]['nodeid'],reason='original platform guard')]


@pytest.mark.parametrize('fault',[
    'failed_phase','missing_case','unexpected_case','false_full_pass','uncollected_historical_test',
    'missing_historical_source','unknown_explicit','excluded_executed_test','missing_core_root',
    'external_boundary','changed_sources','incomplete_scope',
])
def test_sealed_historical_proof_cannot_relabel_or_omit_execution_evidence(fault):
    release,run,entry,stack=sealed_case_ledger()
    if fault=='failed_phase':run['case_results'][0]['call']='failed'
    elif fault=='missing_case':run['case_results'].pop()
    elif fault=='unexpected_case':run['case_results'].append(dict(nodeid='tools/tests/test_extra.py::test_extra'))
    elif fault=='false_full_pass':run['full_regression_passed']=release['full_regression_passed']=True
    elif fault=='uncollected_historical_test':
        path='src/example/tests/test_uncollected.py'
        run['source_sha256'][path]=release['source_sha256'][path]='3'*64
        release['source_count']+=1
    elif fault=='missing_historical_source':
        path=run['test_files'][0]
        del run['source_sha256'][path];del release['source_sha256'][path];release['source_count']-=1
    elif fault=='unknown_explicit':run['explicit_test_files'].append('.runtime/test_not_recorded.py')
    elif fault=='excluded_executed_test':run['excluded_files'].append(dict(path=run['test_files'][0],reason='hidden'))
    elif fault=='missing_core_root':run['test_roots']=['src']
    elif fault=='external_boundary':run['external_boundary_attempts']=['network call']
    elif fault=='changed_sources':run['changed_sources']=['src/example/changed.py']
    else:run['offline_scope_complete']=False
    with pytest.raises(RuntimeError):m.sealed_release_evidence(release,run,entry,stack)


@pytest.fixture
def r27_real_archive_constructor(monkeypatch):
    """Historical full-r2 filesystem view, retaining actual immutable archive bytes.

    Current successor source is validated by its own full release/prepare. This
    fixture deliberately presents only the source bytes and test discovery of
    the historical release under test; it does not claim that tree is installed.
    """
    maintenance = Path('/home/zhuyupei/ssd_data/实验记录/Robot-Controller/maintenance/r27_observation_planning_20261102')
    revision_ref = dict(path=str(maintenance/'remote_only_prepare_r3_final/revision.json'),
        sha256='6ed88ff049fc2ef38da7804244e5838a997973303d3e78f5db4a04db6982e399')
    preservation_ref = dict(path=str(maintenance/'pre_preserved_typed_revision/receipt.json'),
        sha256='9fd2f21a3131b4a8a5425286a53265e9c7eda79038c2c6af6091e8b3b88c5f06')
    real_bytes, real_text = Path.read_bytes, Path.read_text
    real_rglob = Path.rglob
    real_access, real_load = m.os.access, m.load_source

    def checked(reference):
        raw = real_bytes(Path(reference['path']))
        assert m.hashlib.sha256(raw).hexdigest() == reference['sha256']
        return json.loads(raw)

    data = checked(revision_ref)
    release, assets = checked(data['release']), checked(data['typed_assets'])
    baseline = checked(data['baseline_acceptance'])
    contract = checked(baseline['contract'])
    prior_release = checked(data['baseline_release'])
    prior_run = checked(assets['actual_regression'])
    preservation = checked(preservation_ref)
    assert len(assets['source_sha256']) == 43 and len(assets['current_sha256']) == 18
    assert data['entry']['sha256'] == 'a93dfa4932446ead8f0dfdecef182ec7d84d08f4b551c6ca42ce880e88ddebd3'
    assert release['actual_regression'] != assets['actual_regression']
    assert all(release['source_sha256'][p] == h for p,h in assets['source_sha256'].items())
    assert release['source_sha256'][m.MODE_ENTRY] != prior_run['source_sha256'][m.MODE_ENTRY]

    source_views = {}
    for row in preservation['files']:
        if row['path'] in {str(m.W/m.SELF), str(m.W/m.TEST)}:
            raw = real_bytes(Path(row['backup']))
            assert m.hashlib.sha256(raw).hexdigest() == row['sha256']
            assert row['sha256'] == release['source_sha256'][str(Path(row['path']).relative_to(m.W))]
            source_views[row['path']] = raw
    assert set(source_views) == {str(m.W/m.SELF), str(m.W/m.TEST)}
    successor_preservation_ref = dict(path=str(maintenance/
        'new_near_field_failure_20261102/pre_minimum_placement_revision_r1/receipt.json'),
        sha256='e4aefa7d166a1f93e19979073ba58a5fa178fcbad6899506f67e074eda09799b')
    successor_preservation = checked(successor_preservation_ref)
    historical_placement_paths = {
        'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/contact_joint_placement.py',
        'src/alicia_flexible_grasp_supervisor/tests/test_contact_joint_placement.py',
        'src/alicia_flexible_grasp_supervisor/tests/test_contact_joint_placement_branches.py',
        'src/alicia_flexible_grasp_supervisor/tests/fixtures/contact_joint_terminal_20261027.json',
    }
    restored = set()
    for row in successor_preservation['files']:
        relative = row['relative_path']
        if relative in historical_placement_paths:
            raw = real_bytes(Path(row['backup']))
            assert m.hashlib.sha256(raw).hexdigest() == row['sha256'] == release['source_sha256'][relative]
            assert row['path'] == str(m.W/relative)
            source_views[row['path']] = raw
            restored.add(relative)
    assert restored == historical_placement_paths
    support_preservation = checked(dict(path=str(maintenance/
        'attempt2_observation_failure/pre_support_box_revision_r1/receipt.json'),
        sha256='9c4dc76b9b23fc36ca36c38b374ea06849ee6b2cb11620b6775492fee1f30b29'))
    historical_observation = 'src/alicia_grasp_modes/src/alicia_grasp_modes/reachable_observation_views.py'
    row = next(item for item in support_preservation['files']
               if item['relative_path'] == historical_observation)
    raw = real_bytes(Path(row['copy_path']))
    assert m.hashlib.sha256(raw).hexdigest() == row['sha256'] == release['source_sha256'][historical_observation]
    assert row['source_path'] == str(m.W/historical_observation)
    source_views[row['source_path']] = raw
    gate_test_preservation = checked(dict(path=str(maintenance/
        'attempt2_observation_failure/pre_support_box_test_maintenance_r1/receipt.json'),
        sha256='a0e0178f4c587e404e1b67d5a77ada2243a38bd841aedba25c125d1f0d4d8ab8'))
    gate_test_paths = {
        'src/alicia_grasp_modes/tests/test_gen8_endpoint_precompute_independent.py',
        'src/alicia_grasp_modes/tests/test_reachable_observation_direction_coverage.py',
    }
    assert {row['relative_path'] for row in gate_test_preservation['files']} == gate_test_paths
    for row in gate_test_preservation['files']:
        raw = real_bytes(Path(row['copy_path']))
        assert m.hashlib.sha256(raw).hexdigest() == row['sha256'] == release['source_sha256'][row['relative_path']]
        assert row['source_path'] == str(m.W/row['relative_path'])
        source_views[row['source_path']] = raw
    historical_run = checked(release['actual_regression'])
    assert historical_run['source_sha256'] == release['source_sha256']
    historical_roots = {str(m.W/root) for root in historical_run['test_roots']}

    def historical_rglob(path, pattern):
        if str(path) in historical_roots and pattern == 'test_*.py':
            # Exact sealed discovery, including explicit recorded exclusions.
            # No new candidate test is made part of an old regression result.
            return iter(m.W/relative for relative in sorted(release['source_sha256'])
                if Path(relative).name.startswith('test_') and Path(relative).suffix == '.py'
                and path in (m.W/relative).parents and '__pycache__' not in Path(relative).parts)
        return real_rglob(path, pattern)
    corruptions = {}
    current_calls, source_calls, storage_calls = [], [], []

    def read_bytes(path):
        name = str(path)
        if name in corruptions:
            return corruptions[name]
        if name in source_views:
            return source_views[name]
        return real_bytes(path)

    def read_text(path, *args, **kwargs):
        if str(path) == '/proc/sys/kernel/random/boot_id':
            return contract['cold_boot_id']+'\n'
        return real_text(path, *args, **kwargs)

    def access(path, mode, *args, **kwargs):
        # The sandbox mounts the real archive read-only. Storage.validate and
        # policy/configuration checks still run; only that kernel boundary is
        # represented by the already accepted storage identity.
        root = Path(contract['storage']['experiment_root'])
        path = Path(path)
        if (path == root or root in path.parents) and mode == (m.os.W_OK | m.os.X_OK):
            return True
        return real_access(path, mode, *args, **kwargs)

    def mounted_identity(mount, uuid, kind):
        binding = contract['storage']
        assert str(mount) == binding['mount_point']
        assert uuid == binding['filesystem_uuid'] and kind == binding['filesystem_type']
        storage_calls.append(str(mount))
        return {k:binding[k] for k in ('mount_id','device','filesystem_uuid','filesystem_type','mount_point')}

    def instrument_native(module):
        if getattr(module, '_real_archive_spy_installed', False):
            return module
        module._real_archive_spy_installed = True
        original_current, original_sources = module.current_release_evidence, module.source_check

        def current(value):
            event = dict(regression=deepcopy(value['actual_regression']),
                         source_count=len(value['source_sha256']), completed=False)
            current_calls.append(event)
            answer = original_current(value)
            event['completed'] = True
            return answer

        def sources(value):
            source_calls.append(dict(value))
            return original_sources(value)

        monkeypatch.setattr(module, 'current_release_evidence', current)
        monkeypatch.setattr(module, 'source_check', sources)
        return module

    def load_source(relative, sources):
        module = real_load(relative, sources)
        if relative == m.STACK:
            instrument_native(module)
        if relative == m.TYPED:
            original_native = module.native_module

            def native(value):
                return instrument_native(original_native(value))

            monkeypatch.setattr(module, 'native_module', native)
        if relative == 'tools/experiment_storage.py':
            monkeypatch.setattr(module, 'mounted_identity', mounted_identity)
        # full_stack_start imports this normal module separately from load_source.
        storage_module = sys.modules.get('experiment_storage')
        if storage_module is not None:
            monkeypatch.setattr(storage_module, 'mounted_identity', mounted_identity)
        return module

    def corrupt(path):
        path = Path(path)
        corruptions[str(path)] = read_bytes(path)+b'\n'

    monkeypatch.setattr(Path, 'read_bytes', read_bytes)
    monkeypatch.setattr(Path, 'read_text', read_text)
    monkeypatch.setattr(Path, 'rglob', historical_rglob)
    monkeypatch.setattr(m.os, 'access', access)
    monkeypatch.setattr(m, 'load_source', load_source)
    return types.SimpleNamespace(reference=revision_ref, data=data, release=release,
        assets=assets, contract=contract, prior_release=prior_release, prior_run=prior_run,
        source_views=source_views, current_calls=current_calls, source_calls=source_calls,
        storage_calls=storage_calls, corrupt=corrupt)


def test_real_r26_archive_reproduces_old_failure_and_full_revision_bridge_succeeds(r27_real_archive_constructor):
    f = r27_real_archive_constructor
    typed = m.load_source(m.TYPED, f.release['source_sha256'])
    with pytest.raises(RuntimeError, match='installed source changed:') as refused:
        typed.verify_assets(f.assets)
    assert m.MODE_ENTRY in str(refused.value)
    assert any(call['regression'] == f.assets['actual_regression'] and not call['completed']
               for call in f.current_calls)

    # Actual constructor, actual bridge, real pinned JSON readers and native
    # release/driver validators. No object.__new__, successful stubs or fake run.
    revision = m.Revision(f.reference)
    proof = revision.preserved_typed_evidence
    assert revision.assets == f.assets and revision.sources == f.release['source_sha256']
    assert proof['historical_assets'] == f.data['typed_assets']
    assert proof['historical_regression'] == f.assets['actual_regression']
    assert proof['current_release'] == f.data['release']
    assert proof['current_regression'] == f.release['actual_regression']
    assert proof['current_typed_source_count'] == 43 and proof['current_typed_artifact_count'] == 18
    assert proof['historical_derivation_verified'] is True and proof['current_typed_files_verified'] is True
    assert proof['old_release_claimed_installed_now'] is False and proof['memory_attestation'] is False
    assert proof['physical_grasp_verified'] is False
    assert any(call['regression'] == f.release['actual_regression'] and call['completed']
               for call in f.current_calls)
    assert f.release['source_sha256'] in f.source_calls
    assert f.assets['source_sha256'] in f.source_calls
    assert f.storage_calls

    # This generated header is outside the full source map: its rejection is
    # specifically the preserved typed-artifact check, not the outer full run.
    artifact = m.W/'devel/include/alicia_d_driver/PhysicalReferenceCommand.h'
    assert str(artifact) in f.assets['current_sha256']
    assert str(artifact.relative_to(m.W)) not in f.release['source_sha256']
    f.corrupt(artifact)
    with pytest.raises(RuntimeError, match='preserved typed artifact changed:'):
        revision.verify_preserved_typed_files()


@pytest.mark.parametrize('fault', ['typed_assets','previous_assets','formal_build','old_regression','current_regression'])
def test_real_revision_rejects_corrupted_archive_bytes(r27_real_archive_constructor, fault):
    f = r27_real_archive_constructor
    reference = {
        'typed_assets':f.data['typed_assets'],
        'previous_assets':f.assets['python_source_derivation']['previous_assets'],
        'formal_build':f.assets['formal_build'],
        'old_regression':f.assets['actual_regression'],
        'current_regression':f.release['actual_regression'],
    }[fault]
    f.corrupt(reference['path'])
    with pytest.raises(RuntimeError, match='(bound evidence|reference|pinned evidence) changed:'):
        m.Revision(f.reference)


def test_real_revision_keeps_current_native_source_validation(r27_real_archive_constructor):
    f = r27_real_archive_constructor
    f.corrupt(m.W/m.MODE_ENTRY)
    with pytest.raises(RuntimeError, match='installed source changed:') as refused:
        m.Revision(f.reference)
    assert m.MODE_ENTRY in str(refused.value)
    assert any(call['regression'] == f.release['actual_regression'] and not call['completed']
               for call in f.current_calls)


class CwdLifecycleBoundary(RuntimeError):
    pass


@pytest.fixture
def cwd_reload_runtime(tmp_path,monkeypatch):
    """Actual reload control flow; ROS, telemetry, processes and launch are I/O fixtures.

    The original argv has the observed roslaunch shape. Its real temp cwd is
    outside W while PWD still says W. No ROS import or service call escapes.
    """
    workspace=tmp_path/'workspace';workspace.mkdir()
    helper=workspace/m.SELF;helper.parent.mkdir(parents=True);helper.write_bytes(PATH.read_bytes())
    monkeypatch.setattr(m,'W',workspace);monkeypatch.setattr(m,'__file__',str(helper))
    working_path=tmp_path/'original_ros_home';working_path.mkdir()
    proc_root=tmp_path/'proc';proc=proc_root/'16034';proc.mkdir(parents=True)
    session=tmp_path/'session';session.mkdir()
    argv=['/usr/bin/python3',str(workspace/m.RELAY),'__name:=remote_grasp6d_node',
          '__log:='+str(session/'core_ros'/'remote_grasp6d_node-1.log')]
    (proc/'cmdline').write_bytes(b''.join(value.encode()+b'\0' for value in argv))
    environment=dict(PWD=str(workspace),ROS_LOG_DIR=str(session/'core_ros'),
                     PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1')
    (proc/'environ').write_bytes(b''.join((key+'='+value).encode()+b'\0' for key,value in environment.items()))
    (proc/'cwd').symlink_to(working_path,target_is_directory=True)
    original=dict(pid=16034,parent_pid=15932,start_ticks='584542',cmdline_sha256=m.sha(proc/'cmdline'),uri='http://old/')
    gateway,aliases=alias_fixture()
    baseline=dict(nodes={m.REMOTE:deepcopy(original),'/motion_gateway':gateway},auxiliary_nodes={})
    revision=object.__new__(m.Revision)
    revision.output=session/'r27';revision.session=session;revision.baseline=baseline
    revision.reference=dict(path=str(tmp_path/'revision.json'),sha256='a'*64)
    revision.contract=dict(configuration=dict(path='/frozen/configuration.json',sha256='b'*64))
    revision.config=dict(remote_parameters={});revision.sources={};revision.binding={}
    revision.preserved_typed_evidence=dict(test_view=True)
    f=types.SimpleNamespace(revision=revision,proc_root=proc_root,proc=proc,working_path=working_path,
        argv=argv,environment=environment,original=original,aliases=aliases,events=[],shutdown_calls=0,
        launches=[],payload=None,stop='shutdown',on_second_wait=lambda:None,waits=0)
    f.processes={original['pid']:{key:original[key] for key in ('pid','parent_pid','start_ticks')}}
    def process(pid):return deepcopy(f.processes[pid])
    monitor=types.SimpleNamespace(close=lambda:f.events.append('monitor.close'))
    revision.stack=types.SimpleNamespace(process=process,Monitor=lambda config:monitor)
    revision.health=types.SimpleNamespace(RpcTransport=lambda:None)
    revision.storage=types.SimpleNamespace(ros_log_root=session,
        validate=lambda *args,**kwargs:None,environment=lambda path,**kwargs:dict(ROS_LOG_DIR=str(path)))
    revision.verify_files=lambda **kwargs:f.events.append('verify_files')
    revision.live_policy=lambda master:f.events.append('live_policy')
    def graph(master,caller,**kwargs):
        f.events.append('graph.absent' if kwargs.get('remote_absent') else 'graph.original')
        values=deepcopy(baseline['nodes'])
        if kwargs.get('remote_absent'):values.pop(m.REMOTE)
        return values,deepcopy(aliases)
    revision.graph=graph
    master=types.SimpleNamespace(getParam=lambda name:deepcopy(
        dict(mode='remote',seq=3) if name=='/grasp_mode/selection' else dict(test_configuration=True)))
    ros=types.SimpleNamespace(init_node=lambda *args,**kwargs:None,get_name=lambda:'/test_reload')
    monkeypatch.setitem(sys.modules,'rospy',ros)
    monkeypatch.setitem(sys.modules,'rosgraph',types.SimpleNamespace(Master=lambda name:master,MasterError=RuntimeError))
    monkeypatch.setitem(sys.modules,'std_srvs',types.ModuleType('std_srvs'))
    monkeypatch.setitem(sys.modules,'std_srvs.srv',types.SimpleNamespace(Trigger=object))
    monkeypatch.setattr(m,'load_live',lambda args:(revision,dict(path='/review.json',sha256='c'*64)))
    monkeypatch.setattr(m,'remote_graph',lambda master:{})
    monkeypatch.setenv('ROS_LOG_DIR',str(session/'core_ros'))
    real_path=Path
    monkeypatch.setattr(m,'Path',lambda value:proc_root if str(value)=='/proc' else real_path(value))
    def wait_manual(*args,**kwargs):
        f.waits+=1;f.events.append('manual.'+str(f.waits))
        if f.waits==2:f.on_second_wait()
        return dict(epoch_ns=42,names=['joint'],positions=[0.])
    monkeypatch.setattr(m,'wait_manual',wait_manual)
    def shutdown(*args):
        f.shutdown_calls+=1;f.events.append('shutdown')
        if f.stop=='shutdown':raise CwdLifecycleBoundary('shutdown boundary')
        proc.rename(proc_root/'old_exited')
        return 1,'stopped',0
    monkeypatch.setattr(m.xmlrpc.client,'ServerProxy',lambda *args,**kwargs:types.SimpleNamespace(shutdown=shutdown))
    def launch(command,**kwargs):
        f.launches.append(command);f.events.append('systemd-run')
        f.private=Path(command[command.index('--private')+1])
        f.private_sha=command[command.index('--private-sha256')+1]
        f.payload=m.read_ref(dict(path=str(f.private),sha256=f.private_sha))
        raise CwdLifecycleBoundary('launch boundary')
    monkeypatch.setattr(m.subprocess,'run',launch)
    return f


@pytest.mark.parametrize('boundary',['shutdown','launch'])
def test_actual_reload_preserves_observed_nonworkspace_cwd_despite_pwd(cwd_reload_runtime,boundary):
    f=cwd_reload_runtime;f.stop=boundary
    with pytest.raises(CwdLifecycleBoundary,match=boundary+' boundary'):m.reload_remote(types.SimpleNamespace())
    construction_path=f.revision.output/'construction.json'
    construction=m.read_ref(m.ref(construction_path));expected=m.directory_identity(f.working_path)
    assert construction['working_directory']==expected and construction['original_remote']==f.original
    assert construction['original_argv']==f.argv and f.environment['PWD']==str(m.W)!=expected['path']
    assert f.events[:9]==['verify_files','live_policy','graph.original','manual.1',
                          'verify_files','live_policy','graph.original','manual.2','shutdown']
    assert f.shutdown_calls==1
    if boundary=='launch':
        assert f.payload['construction']==m.ref(construction_path) and f.payload['cwd']==expected['path']
        assert '--property=WorkingDirectory='+expected['path'] in f.launches[0]
        assert f.payload['environment']['PWD']==str(m.W)
        assert f.payload['argv'][:3]==f.argv[:3] and f.payload['argv'][3].endswith('/remote_node.log')
        assert f.launches[0][-1]=='__name:=remote_grasp6d_node'
    else:assert not f.launches
    assert not m.read_ref(m.ref(f.revision.output/'reload_result.json'))['passed']


@pytest.mark.parametrize('fault',[
    'foreign_entry','deleted','cwd_path','replaced_directory','process_ticks','process_parent',
    'cmdline','construction_bytes',
])
def test_reload_rejects_changed_cwd_or_process_before_any_shutdown(cwd_reload_runtime,fault):
    f=cwd_reload_runtime
    def change():
        if fault=='cwd_path':
            other=f.working_path.parent/'other';other.mkdir()
            (f.proc/'cwd').unlink();(f.proc/'cwd').symlink_to(other,target_is_directory=True)
        elif fault=='replaced_directory':
            f.working_path.rename(f.working_path.with_name('detached_original'));f.working_path.mkdir()
        elif fault=='process_ticks':f.processes[16034]['start_ticks']='999999'
        elif fault=='process_parent':f.processes[16034]['parent_pid']=99999
        elif fault=='cmdline':(f.proc/'cmdline').write_bytes(b'foreign\0')
        elif fault=='construction_bytes':(f.revision.output/'construction.json').write_text('{}')
    if fault=='foreign_entry':
        (f.proc/'cmdline').write_bytes(b'/usr/bin/python3\0/foreign/entry.py\0')
        f.revision.baseline['nodes'][m.REMOTE]['cmdline_sha256']=m.sha(f.proc/'cmdline')
    elif fault=='deleted':
        (f.proc/'cwd').unlink();(f.proc/'cwd').symlink_to(str(f.working_path)+' (deleted)')
    else:f.on_second_wait=change
    with pytest.raises((RuntimeError,OSError)):m.reload_remote(types.SimpleNamespace())
    assert f.shutdown_calls==0 and not f.launches
    assert not m.read_ref(m.ref(f.revision.output/'reload_result.json'))['passed']


def prepare_cwd_child(f,monkeypatch,*,fault=None):
    """Pass actual reload payload into actual child with a minimal non-ROS entry."""
    f.stop='launch'
    with pytest.raises(CwdLifecycleBoundary,match='launch boundary'):m.reload_remote(types.SimpleNamespace())
    revision=f.revision
    current_pid=m.os.getpid();proc=f.proc_root/str(current_pid);proc.mkdir()
    f.processes[current_pid]=dict(pid=current_pid,parent_pid=80,start_ticks='999999')
    (proc/'cmdline').write_bytes(b'/usr/bin/python3\0-B\0/r27-helper\0child\0')
    (proc/'cwd').symlink_to(f.working_path,target_is_directory=True)
    namespace,modules=synthetic_bindings()
    package=types.ModuleType('alicia_grasp_modes');package.__path__=[]
    monkeypatch.setitem(sys.modules,'alicia_grasp_modes',package)
    for name,relative in m.MODULE_SOURCES.items():
        path=m.W/relative;path.parent.mkdir(parents=True,exist_ok=True);path.write_text('# fixture module\n')
        modules[name].__file__=str(path);monkeypatch.setitem(sys.modules,name,modules[name])
        revision.sources[relative]=m.sha(path)
    mode_path=m.W/m.MODE_ENTRY;mode_path.parent.mkdir(parents=True,exist_ok=True)
    mode='''from alicia_grasp_modes.silhouette_observation_planning import SilhouetteObservationPlanningMixin
from alicia_grasp_modes.observe_return_remote import ObserveReturnRemoteMixin
class ModeAwareRemoteGrasp6DNode(SilhouetteObservationPlanningMixin, ObserveReturnRemoteMixin):
    def _joint_space_observation_candidates(self, prepared):
        from alicia_grasp_modes.reachable_observation_views import (
            FK_PROPOSAL_BUDGET_SEC, generate_reachable_observation_views)
'''
    entered=m.W/'entered_original_main'
    other=f.working_path.parent/'foreign_ros_home';other.mkdir()
    if fault=='before_main':
        mode+='import os\nos.unlink(%r)\nos.symlink(%r, %r)\nos.chdir(%r)\n' % (
            str(proc/'cwd'),str(other),str(proc/'cwd'),str(other))
    mode+="if __name__ == '__main__':\n    with open(%r, 'w') as f: f.write('entered')\n" % str(entered)
    mode_path.write_text(mode)
    relay=m.W/m.RELAY;relay.parent.mkdir(parents=True,exist_ok=True)
    relay.write_text("with open(%r, 'r') as f:\n    exec(compile(f.read(), %r, 'exec'), "
                     "{'__name__': __name__, '__file__': %r})\n" % (str(mode_path),str(mode_path),str(mode_path)))
    revision.sources[m.MODE_ENTRY]=m.sha(mode_path);revision.sources[m.SELF]=m.sha(m.__file__)
    revision.contract['current_sha256']={str(relay):m.sha(relay)}
    monkeypatch.setattr(m,'Revision',lambda reference:revision)
    monkeypatch.setenv('R27_REMOTE_REVISION_SHA256',revision.reference['sha256'])
    monkeypatch.setattr(sys,'argv',list(sys.argv))
    monkeypatch.setattr(sys,'dont_write_bytecode',sys.dont_write_bytecode)
    monkeypatch.setattr(sys,'pycache_prefix',sys.pycache_prefix)
    monkeypatch.chdir(f.working_path)
    if fault=='initial_cwd':
        (proc/'cwd').unlink();(proc/'cwd').symlink_to(other,target_is_directory=True)
        m.os.chdir(other)
    elif fault=='private_cwd':f.payload['cwd']=str(other)
    elif fault=='construction_ref':f.payload['construction']['sha256']='0'*64
    f.private.write_text(json.dumps(f.payload))
    return types.SimpleNamespace(private=str(f.private),private_sha256=m.sha(f.private)),entered,proc


def test_actual_child_keeps_reload_directory_and_writes_bound_source_receipt(cwd_reload_runtime,monkeypatch):
    f=cwd_reload_runtime;args,entered,proc=prepare_cwd_child(f,monkeypatch)
    m.child(args)
    receipt=m.read_ref(m.ref(f.revision.output/'source_start.json'))
    assert entered.read_text()=='entered' and not f.private.exists()
    assert receipt['construction']==f.payload['construction']
    assert receipt['working_directory']==m.directory_identity(f.working_path)
    assert Path.cwd()==f.working_path
    assert receipt['source_checked_before_original_main'] and not receipt['memory_attestation']
    identity=dict(f.processes[m.os.getpid()],cmdline_sha256=m.sha(proc/'cmdline'))
    assert m.process_working_directory(f.revision.stack,identity)==receipt['working_directory']


@pytest.mark.parametrize('fault',['initial_cwd','private_cwd','construction_ref','before_main'])
def test_child_rejects_directory_or_construction_change_before_original_main(cwd_reload_runtime,monkeypatch,fault):
    f=cwd_reload_runtime;args,entered,_proc=prepare_cwd_child(f,monkeypatch,fault=fault)
    with pytest.raises(RuntimeError):m.child(args)
    assert not entered.exists() and not (f.revision.output/'source_start.json').exists()


@pytest.mark.parametrize('fault',['process_cwd','directory_inode','unit_cwd','construction_bytes'])
def test_attempt_graph_rechecks_actual_frozen_directory_and_unit(tmp_path,monkeypatch,fault):
    f=ready_recording(tmp_path,monkeypatch);ctx,master=recorder_runtime_graph(f,monkeypatch)
    ctx.recheck(master,'/attempt')
    if fault=='process_cwd':
        other=tmp_path/'other';other.mkdir()
        (f.remote_proc/'cwd').unlink();(f.remote_proc/'cwd').symlink_to(other,target_is_directory=True)
    elif fault=='directory_inode':
        f.working_path.rename(f.working_path.with_name('detached'));f.working_path.mkdir()
    elif fault=='unit_cwd':f.unit_directory='/foreign/working_directory'
    else:(f.revision.output/'construction.json').write_text('{}')
    with pytest.raises(RuntimeError):ctx.recheck(master,'/attempt')


def test_cwd_observation_rejects_process_reuse_during_capture(cwd_reload_runtime):
    f=cwd_reload_runtime;calls=[]
    def process(pid):
        calls.append(pid);row=deepcopy(f.processes[pid])
        if len(calls)>1:row['start_ticks']='changed'
        return row
    with pytest.raises(RuntimeError,match='changed during observation'):
        m.process_working_directory(types.SimpleNamespace(process=process),f.original)
