"""Finite continuation integration, without ROS or device operations.

Archive construction reads the actual immutable H4/full/history bytes. The four
subsequently changed files are supplied from checked preimages and discovery
uses the sealed H4 inventory; this is explicitly a historical file view, not
current-tree admission. SSD kernel mount/access identity is represented inside
the read-only sandbox. Composite-proof unit fixtures are labelled and do
not represent an executed target. Session tests reuse the old explicit I/O/source
admission fixture but call the actual new Context, H4 graph/RecorderBinding,
original Session and new typed validator against real archived messages.
"""
from copy import deepcopy
from dataclasses import asdict
import importlib.util
import json
from pathlib import Path
import sys
import types

import pytest


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


PATH = Path(__file__).resolve().parents[1]/'adapter.py'
m = load('post_trajectory_adapter_under_test', PATH)
NS = types.SimpleNamespace


@pytest.fixture
def archived():
    """Real bytes, original pure validators; no synthetic historical success."""
    documents = {key:m.read_ref(value) for key,value in m.ANCHORS.items()}
    release = m.read_ref(documents['loaded_revision']['release'])
    h4 = m.load_module(m.H4_PATH, m.H4_SHA)
    readiness = m.load_module(m.READINESS, m.sha(m.W/m.READINESS))
    typed = h4.load_source(h4.TYPED, release['source_sha256'])
    entry = h4.load_source(h4.ENTRY, release['source_sha256'])
    assets = m.read_ref(documents['loaded_revision']['typed_assets'])
    history = readiness.validate_transport_history(documents['recorded_history'],documents['recorded_actions'])
    return NS(documents=documents, release=release, h4=h4, readiness=readiness,
              typed=typed, entry=entry, assets=assets, history=history)


@pytest.fixture
def real_constructor(archived, monkeypatch):
    """Checked historical file view; original derivation and validators run.

    H5 has a separate current-source/full-regression admission. No new source
    or test is attributed to this older release or its recorded result.
    """
    f = archived
    baseline = m.read_ref(f.documents['loaded_revision']['baseline_acceptance'])
    contract = m.read_ref(baseline['contract']); binding = contract['storage']
    original_load = f.h4.load_source
    original_module = m.load_module
    original_access = m.os.access
    original_bytes, original_rglob = Path.read_bytes, Path.rglob
    f.current_calls, f.mount_calls = [], []

    preservation_ref = dict(path=str(m.M/
        'attempt2_observation_failure/pre_support_box_revision_r1/receipt.json'),
        sha256='9c4dc76b9b23fc36ca36c38b374ea06849ee6b2cb11620b6775492fee1f30b29')
    raw = original_bytes(Path(preservation_ref['path']))
    assert m.hashlib.sha256(raw).hexdigest() == preservation_ref['sha256']
    preservation = json.loads(raw)
    historical_paths = {
        'src/alicia_grasp_modes/src/alicia_grasp_modes/reachable_observation_views.py',
        '.runtime/r27_observation_planning_20261102/runtime_binding/tests/test_remote_only_revision_r3.py',
    }
    source_views = {}
    for row in preservation['files']:
        relative = row['relative_path']
        if relative in historical_paths:
            raw = original_bytes(Path(row['copy_path']))
            assert m.hashlib.sha256(raw).hexdigest() == row['sha256'] == f.release['source_sha256'][relative]
            assert row['source_path'] == str(m.W/relative)
            source_views[row['source_path']] = raw
    gate_test_ref = dict(path=str(m.M/
        'attempt2_observation_failure/pre_support_box_test_maintenance_r1/receipt.json'),
        sha256='a0e0178f4c587e404e1b67d5a77ada2243a38bd841aedba25c125d1f0d4d8ab8')
    raw = original_bytes(Path(gate_test_ref['path']))
    assert m.hashlib.sha256(raw).hexdigest() == gate_test_ref['sha256']
    gate_test_preservation = json.loads(raw)
    gate_test_paths = {
        'src/alicia_grasp_modes/tests/test_gen8_endpoint_precompute_independent.py',
        'src/alicia_grasp_modes/tests/test_reachable_observation_direction_coverage.py',
    }
    assert {row['relative_path'] for row in gate_test_preservation['files']} == gate_test_paths
    historical_paths.update(gate_test_paths)
    for row in gate_test_preservation['files']:
        raw = original_bytes(Path(row['copy_path']))
        assert m.hashlib.sha256(raw).hexdigest() == row['sha256'] == f.release['source_sha256'][row['relative_path']]
        assert row['source_path'] == str(m.W/row['relative_path'])
        source_views[row['source_path']] = raw
    assert set(source_views) == {str(m.W/relative) for relative in historical_paths}
    historical_run = m.read_ref(f.release['actual_regression'])
    assert historical_run['source_sha256'] == f.release['source_sha256']
    historical_roots = {str(m.W/root) for root in historical_run['test_roots']}

    def historical_bytes(path):
        return source_views[str(path)] if str(path) in source_views else original_bytes(path)

    def historical_rglob(path, pattern):
        if str(path) in historical_roots and pattern == 'test_*.py':
            return iter(m.W/relative for relative in sorted(f.release['source_sha256'])
                if Path(relative).name.startswith('test_') and Path(relative).suffix == '.py'
                and path in (m.W/relative).parents and '__pycache__' not in Path(relative).parts)
        return original_rglob(path, pattern)

    def mounted(mount, uuid, kind):
        assert str(mount) == binding['mount_point']
        assert uuid == binding['filesystem_uuid'] and kind == binding['filesystem_type']
        f.mount_calls.append(str(mount))
        return {key:binding[key] for key in ('mount_id','device','filesystem_uuid','filesystem_type','mount_point')}

    def access(path, mode, *args, **kwargs):
        path = Path(path)
        if (path == m.B or m.B in path.parents) and mode == (m.os.W_OK | m.os.X_OK):
            return True
        return original_access(path, mode, *args, **kwargs)

    def sources(relative, hashes):
        module = original_load(relative, hashes)
        if relative == f.h4.STACK:
            current = module.current_release_evidence
            def delegate(release):
                answer = current(release)
                f.current_calls.append(deepcopy(release['actual_regression']))
                return answer
            monkeypatch.setattr(module, 'current_release_evidence', delegate)
        if relative == 'tools/experiment_storage.py':
            monkeypatch.setattr(module, 'mounted_identity', mounted)
        storage = sys.modules.get('experiment_storage')
        if storage is not None:
            monkeypatch.setattr(storage, 'mounted_identity', mounted)
        return module

    monkeypatch.setattr(m.os, 'access', access)
    monkeypatch.setattr(Path, 'read_bytes', historical_bytes)
    monkeypatch.setattr(Path, 'rglob', historical_rglob)
    monkeypatch.setattr(f.h4, 'load_source', sources)
    monkeypatch.setattr(m, 'load_module', lambda relative,digest:
        f.h4 if (relative,digest) == (m.H4_PATH,m.H4_SHA) else original_module(relative,digest))
    return f


def test_actual_loaded_revision_constructor_and_post_trajectory_history(real_constructor,tmp_path):
    f = real_constructor
    loaded = m.LoadedRuntime()
    assert loaded.base.reference == m.ANCHORS['loaded_revision']
    assert len(loaded.base.sources) == 4282
    assert f.current_calls and all(ref == f.release['actual_regression'] for ref in f.current_calls)
    assert f.mount_calls and loaded.base.preserved_typed_evidence['current_typed_source_count'] == 43
    assert loaded.chain['same_retained_control_processes'] is True
    assert loaded.chain['old_epoch_claimed_current'] is False
    assert loaded.chain['recorded_epoch_ns'] < loaded.chain['current_epoch_ns']
    assert f.history.action_transport_sequence == 1
    diagnostic = f.readiness.validate_recorded_snapshot(loaded.documents['snapshot'], f.history,
        startup=loaded.base.typed, assets=loaded.base.assets)
    assert diagnostic['historical_diagnostic_only'] and not diagnostic['current_readiness_claimed']
    rows = f.readiness.recorded_rows(loaded.documents['snapshot'])
    ros = NS(Time=NS(now=lambda:f.readiness._Stamp(loaded.documents['snapshot']['finished_ns'])))
    with pytest.raises(RuntimeError, match='initial typed binding is not empty LEGACY'):
        loaded.base.typed.observed_ready(NS(snapshot=lambda:rows), loaded.chain['current_epoch_ns'], ros, loaded.base.assets)
    # The new acceptance here is explicitly an offline schema fixture, not a
    # live result. Its loaded runtime/health/source references are actual bytes.
    cont,accepted,review,reference = accepted_fixture(f,tmp_path,base=loaded.base)
    _,_,instruction_ref,instruction = ticket_fixture(f)
    instruction.update(runtime_revision=loaded.base.reference,post_trajectory_continuation=cont.reference,
        current_acceptance=reference,latest_resume=loaded.base.data['instruction'])
    instruction_ref = write_json(tmp_path/'new_instruction.json',instruction)
    context = m.AttemptContext(cont,reference,instruction_ref,review,object())
    assert context.acceptance['nodes'] == loaded.documents['loaded_reload']['nodes']
    assert context.remote['node_identity']['pid'] == loaded.documents['loaded_reload']['nodes'][f.h4.REMOTE]['pid']
    assert context.typed_observer is None and context.typed_admission == []


@pytest.mark.parametrize('anchor', ['loaded_reload','recorded_history','recorded_actions','epoch_context'])
def test_actual_loaded_constructor_rejects_changed_sealed_bytes(real_constructor, monkeypatch, anchor):
    original = Path.read_bytes; target = m.ANCHORS[anchor]['path']
    monkeypatch.setattr(Path, 'read_bytes', lambda path:
        original(path)+b'\n' if str(path) == target else original(path))
    with pytest.raises(RuntimeError, match='changed:'):
        m.LoadedRuntime()


@pytest.mark.parametrize('fault', ['unclosed','held_result','manual_unverified','bag_sha','retained_owner',
    'epoch_projected','epoch_source','storage','failed_initial_kind'])
def test_actual_chain_rejects_historical_provenance_mutation(archived, fault):
    f = archived; d = deepcopy(f.documents)
    base = NS(reference=m.ANCHORS['loaded_revision'], output=m.LOADED_OUTPUT,
              sources=f.release['source_sha256'], binding=d['snapshot']['storage_binding'])
    if fault == 'unclosed': d['prior_manifest']['recorder_closed'] = False
    elif fault == 'held_result': d['prior_manifest']['result'] = 'unknown'
    elif fault == 'manual_unverified': d['prior_authorization']['manual_return_verified'] = False
    elif fault == 'bag_sha': d['recorded_actions']['bag_sha256'] = '0'*64
    elif fault == 'retained_owner': d['prior_authorization']['before']['node_identities']['/motion_gateway']['start_ticks'] += '0'
    elif fault == 'epoch_projected': d['snapshot']['state']['epoch_ns'] = d['epoch_context']['observations']['recorded_epoch_ns']
    elif fault == 'epoch_source': d['epoch_context']['source_sha256'][next(iter(d['epoch_context']['source_sha256']))] = '0'*64
    elif fault == 'storage': d['recorded_history']['storage_binding']['device'] = 'foreign'
    else: d['failed_initial_acceptance']['kind'] = 'successful_post_trajectory'
    with pytest.raises(RuntimeError): m.validate_anchor_chain(base,f.h4,d)


def test_no_reload_reset_child_or_typed_mutation_cli():
    for action in ('reload','child','reset','clear','query'):
        with pytest.raises(SystemExit): m.parser().parse_args([action])
    args = m.parser().parse_args(['accept','--continuation','/c','--continuation-sha256','a',
        '--execution-review','/r','--execution-review-sha256','b'])
    assert args.action == 'accept'


def task():
    return NS(_connection_header=dict(callerid='/grasp_task_node',latching='1'),
        state='IDLE',stage=0,active=False,success=False,message='',header=NS(stamp=NS(to_nsec=lambda:123)))


@pytest.mark.parametrize('fault', [None,'failed','holding','success','active','wrong_stage','bool_stage',
    'owner','latch','held_message','unverified_message'])
def test_admission_requires_exact_real_idle_not_unknown_or_held_terminal(fault):
    value = task()
    if fault is None:
        assert m.validate_no_held_task(value)['state'] == 'IDLE'; return
    if fault == 'failed': value.state,value.stage = 'FAILED',7
    elif fault == 'holding': value.state,value.stage = 'HOLDING',10
    elif fault == 'success': value.success = True
    elif fault == 'active': value.active = True
    elif fault == 'wrong_stage': value.stage = 7
    elif fault == 'bool_stage': value.stage = False
    elif fault == 'owner': value._connection_header['callerid'] = '/foreign'
    elif fault == 'latch': value._connection_header['latching'] = '0'
    elif fault == 'held_message': value.message = 'HOLDING'
    else: value.message = 'GRASP_HOLD_UNVERIFIED'
    with pytest.raises(RuntimeError): m.validate_no_held_task(value)


def ticket_fixture(archived):
    base = NS(entry=archived.entry, reference=m.ANCHORS['loaded_revision'],
              data=archived.documents['loaded_revision'])
    continuation = NS(base=base,documents=archived.documents,
                      reference=dict(path='/new/continuation.json',sha256='c'*64))
    accepted = dict(path='/new/result.json',sha256='a'*64)
    reference = dict(path='/new/instruction.json',sha256='i'*64)
    instruction = dict(source='actual_user_message',text='current continued request',
        automatic_grasp_authorized=True,user_alignment_ready=True,new_scene_confirmation_required=False,
        new_scene_confirmation_claimed=False,maximum_attempts=1,automatic_retry=False,
        runtime_revision=base.reference,post_trajectory_continuation=continuation.reference,
        current_acceptance=accepted,latest_resume=base.data['instruction'])
    return continuation,accepted,reference,instruction


@pytest.mark.parametrize('fault', [None,'old_path','old_sha','different_spec','different_acceptance',
    'different_revision','stale_resume','two_attempts','automatic_retry','new_scene_claim'])
def test_single_ticket_binds_actual_successful_continuation_and_cannot_reuse_old_attempt(archived, fault):
    cont,acceptance,reference,instruction = ticket_fixture(archived)
    if fault == 'old_path': reference['path'] = cont.documents['prior_authorization']['user_instruction']['path']
    elif fault == 'old_sha': reference['sha256'] = cont.documents['prior_authorization']['user_instruction']['sha256']
    elif fault == 'different_spec': instruction['post_trajectory_continuation'] = dict(path='/another/c.json',sha256='d'*64)
    elif fault == 'different_acceptance': instruction['current_acceptance'] = dict(path='/another/result.json',sha256='b'*64)
    elif fault == 'different_revision': instruction['runtime_revision'] = dict(path='/parent/revision.json',sha256='e'*64)
    elif fault == 'stale_resume': instruction['latest_resume'] = dict(path='/old/resume.json',sha256='f'*64)
    elif fault == 'two_attempts': instruction['maximum_attempts'] = 2
    elif fault == 'automatic_retry': instruction['automatic_retry'] = True
    elif fault == 'new_scene_claim': instruction['new_scene_confirmation_claimed'] = True
    if fault is None:
        m.validate_new_instruction(cont,acceptance,reference,instruction)
    else:
        with pytest.raises(RuntimeError): m.validate_new_instruction(cont,acceptance,reference,instruction)


def test_exclusive_claim_blocks_same_continuation_even_with_another_ticket(tmp_path):
    cont = object.__new__(m.Continuation)
    cont.base = NS(storage=NS(validate=lambda *a,**k:None),binding={})
    claim = tmp_path/'attempt_claim.json'
    first = dict(instruction=dict(path='/ticket1',sha256='1'*64))
    receipt = cont.save(claim,first)
    with pytest.raises(FileExistsError):
        cont.save(claim,dict(instruction=dict(path='/ticket2',sha256='2'*64)))
    assert m.read_ref(receipt) == first


def accepted_fixture(archived,tmp_path,base=None):
    """New receipt schema fixture derived from the sealed deployed H4 chain."""
    f = archived; deployed = f.documents['loaded_reload']; d = f.documents['loaded_revision']
    if base is None:
        base = NS(reference=m.ANCHORS['loaded_revision'],data=d,entry=f.entry,
                  parent=m.read_ref(f.h4.PARENT_ACCEPTANCE))
    cont = NS(base=base,h4=f.h4,documents=f.documents,history=f.history,
        output=tmp_path/'new_acceptance',reference=dict(path='/offline/continuation_fixture.json',sha256='c'*64),
        target=dict(explicit_offline_fixture=True))
    review = dict(path='/offline/execution_review_fixture.json',sha256='e'*64)
    accepted = deepcopy(deployed)
    accepted.update(kind='r27_post_trajectory_acceptance_r1',passed=True,manual_only=True,
        all_configured_nodes_started=True,source_cache_process_binding=True,automatic_grasp_started=False,
        core_unit=base.parent['core_unit'],core_service=base.parent['core_service'],
        master_identity=base.parent['master_identity'],simulation=base.parent['simulation'],
        typed_post_trajectory_contract_ready=True,typed_initial_contract_ready=False,
        continuation=cont.reference,revision=base.reference,reload=m.ANCHORS['loaded_reload'],
        execution_review=review,reload_execution_review=deployed['execution_review'],
        source_manifest=d['release'],contract=base.parent['contract'],transport_history=asdict(f.history),
        continuation_validation=cont.target)
    reference = write_json(cont.output/'result.json',accepted)
    return cont,accepted,review,reference


@pytest.mark.parametrize('fault',[None,'initial_kind','initial_ready','different_node','different_unit',
    'different_history','different_reload','different_review','different_epoch'])
def test_new_acceptance_cannot_be_confused_with_old_startup_or_other_loaded_instance(archived,tmp_path,fault):
    cont,accepted,review,reference = accepted_fixture(archived,tmp_path)
    if fault == 'initial_kind': accepted['kind'] = 'r27_remote_only_acceptance_r4'
    elif fault == 'initial_ready': accepted['typed_initial_contract_ready'] = True
    elif fault == 'different_node': accepted['nodes'][archived.h4.REMOTE]['start_ticks'] += '0'
    elif fault == 'different_unit': accepted['remote_service']['properties']['InvocationID'] = 'foreign'
    elif fault == 'different_history': accepted['transport_history']['action_transport_sequence'] += 1
    elif fault == 'different_reload': accepted['reload'] = dict(path='/other/reload',sha256='0'*64)
    elif fault == 'different_review': accepted['execution_review'] = dict(path='/other/review',sha256='0'*64)
    elif fault == 'different_epoch': accepted['state']['epoch_ns'] += 1
    if fault is None: m.validate_acceptance(cont,reference,accepted,review)
    else:
        with pytest.raises(RuntimeError): m.validate_acceptance(cont,reference,accepted,review)


def write_json(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value),encoding='utf8')
    return m.ref(path)


@pytest.fixture
def composite(tmp_path,monkeypatch,archived):
    """Synthetic two-case admission ledger; never labelled an actual target."""
    monkeypatch.setattr(m,'B',tmp_path/'disk')
    monkeypatch.setattr(m,'M',m.B/'maintenance')
    h4 = archived.h4; oldtest = h4.TEST
    sources = {path:m.sha(m.W/path) for path in (h4.REGRESSION_WRAPPER,h4.ORIGINAL_RUNNER,oldtest)}
    new = {path:m.sha(m.W/path) for path in m.NEW_SOURCES}
    oldref = write_json(tmp_path/'old_release.json',dict(source_sha256=sources))
    oldreview = dict(kind='r27_offline_source_review',reviewed=True,baseline_release=oldref,
        source_sha256=sources,additional_source_files=[oldtest],additional_test_files=[oldtest])
    oldreview_ref = write_json(tmp_path/'oldreview.json',oldreview)
    production = dict(test_files=[oldtest],excluded_files=[])
    production_ref = write_json(tmp_path/'old_full_fixture.json',production)
    release_ref = dict(path='/synthetic/loaded_release.json',sha256='1'*64)
    base = NS(sources=sources,data=dict(release=release_ref),release=dict(actual_regression=production_ref),
        review=dict(regression_source_review=oldreview_ref),binding=dict(backend='ssd'),entry=archived.entry)
    wanted = dict(sources,**new)
    wrapper = dict(oldreview,source_sha256=wanted,changed_sources=h4.source_delta(sources,wanted),
        additional_source_files=[oldtest]+sorted(m.NEW_SOURCES),additional_test_files=[oldtest]+list(m.TESTS))
    wrapper_path = tmp_path/'new_wrapper_review.json'; wrapper_ref = write_json(wrapper_path,wrapper)
    review = dict(kind='r27_post_trajectory_continuation_source_review',reviewed=True,anchors=deepcopy(m.ANCHORS),
        new_source_sha256=new,loaded_release=release_ref,production_regression=production_ref,
        regression_source_review=wrapper_ref)
    run = dict(passed=True,source_unchanged=True,external_boundary_attempts=[],bad_files=[],duplicate_cases=[],
        collection_skips=[],source_sha256=wanted,test_files=list(m.TESTS),complete_scope_requested=False,
        offline_scope_complete=False,full_regression_passed=False,changed_sources=[],explicit_test_files=list(m.TESTS),
        declared_test_files=[oldtest]+list(m.TESTS),excluded_files=[],
        expected_case_ids=[path+'::test_synthetic_ledger_fixture' for path in m.TESTS],
        case_results=[dict(nodeid=path+'::test_synthetic_ledger_fixture',setup='passed',call='passed',teardown='passed') for path in m.TESTS],
        summary=dict(tests=2,passed=2,skipped=0,failed=0))
    name = 'synthetic_target_fixture'
    run_path = m.B/'ros_log/integrated_regression'/name/'result.json'
    entry_path = m.M/'regression_entry_receipts'/(name+'.json')
    entry = dict(source_review=wrapper_ref,
        entry=dict(path=str(m.W/h4.REGRESSION_WRAPPER),sha256=sources[h4.REGRESSION_WRAPPER]),
        original_runner=dict(path=str(m.W/h4.ORIGINAL_RUNNER),sha256=sources[h4.ORIGINAL_RUNNER]),
        storage_binding=base.binding,manifest_extensions=wrapper['additional_source_files'],
        scope_extensions=wrapper['additional_test_files'],source_count=len(wanted),declared_test_file_count=3,
        all_original_test_files_preserved=True,sandbox_and_reconciliation_unchanged=True,
        hardware_or_network_operations_authorized=False,
        argv=['wrapper','--name',name,'--storage-backend','ssd','--pilot-file',m.TESTS[0],'--pilot-file',m.TESTS[1]])
    def verify():
        wrapper_ref = write_json(wrapper_path,wrapper)
        review['regression_source_review'] = wrapper_ref; entry['source_review'] = wrapper_ref
        write_json(entry_path,entry)
        return m.validate_targeted_extension(base,h4,review,write_json(run_path,run))
    return NS(base=base,review=review,wrapper=wrapper,run=run,entry=entry,verify=verify)


def test_composite_keeps_full_and_two_file_target_distinct(composite):
    value = composite.verify()
    assert value['target_summary'] == dict(tests=2,passed=2,skipped=0,failures=0,errors=0)
    assert value['additive_source_count'] == 4 and value['production_full_repeated'] is False
    assert value['production_full_regression'] == composite.base.release['actual_regression']


@pytest.mark.parametrize('fault', ['extra_source','changed_new_source','wrong_production','wrong_delta',
    'omit_old_discovery','missing_test','duplicate_case','failed_case','new_skip','full_claim',
    'changed_source','boundary_attempt','entry_scope','entry_runner','pilot_subset'])
def test_composite_cannot_replace_current_full_or_weaken_target(composite,fault):
    f = composite
    if fault == 'extra_source': f.review['new_source_sha256']['unreviewed.py'] = '0'*64
    elif fault == 'changed_new_source': f.review['new_source_sha256'][m.SELF] = '0'*64
    elif fault == 'wrong_production': f.review['production_regression'] = dict(path='/another/run',sha256='0'*64)
    elif fault == 'wrong_delta': f.wrapper['changed_sources'] = {}
    elif fault == 'omit_old_discovery': f.run['declared_test_files'] = list(m.TESTS)
    elif fault == 'missing_test': f.run['test_files'] = [m.TESTS[0]]
    elif fault == 'duplicate_case': f.run['case_results'].append(deepcopy(f.run['case_results'][0]))
    elif fault == 'failed_case': f.run['case_results'][0]['call'] = 'failed'
    elif fault == 'new_skip': f.run['case_results'][0].update(call='skipped',skip_reason='new omission')
    elif fault == 'full_claim': f.run['complete_scope_requested'] = True
    elif fault == 'changed_source': f.run['changed_sources'] = ['changed.py']
    elif fault == 'boundary_attempt': f.run['external_boundary_attempts'] = [dict(call='foreign')]
    elif fault == 'entry_scope': f.entry['scope_extensions'] = [m.TESTS[0]]
    elif fault == 'entry_runner': f.entry['original_runner']['sha256'] = '0'*64
    else: f.entry['argv'] = f.entry['argv'][:-2]
    with pytest.raises(RuntimeError): f.verify()


@pytest.fixture
def session_io(tmp_path,monkeypatch,archived):
    """Actual Session/new recheck/H4 graph with the original explicit I/O fixture.

The existing helper fixture isolates source admission/health/snapshot admission
and process/RPC I/O. Real archive construction above covers source admission;
the new typed observer uses actual archived messages and unmodified pure gates.
"""
    h4tests = load('post_trajectory_old_h4_io_factories',m.W/
        '.runtime/r27_observation_planning_20261102/runtime_binding/tests/test_remote_only_revision_r4.py')
    f,oldctx,session = h4tests.successor_session(tmp_path,monkeypatch,manual=True)
    h4 = h4tests.m; entry = f.revision.entry
    ctx = object.__new__(m.AttemptContext); ctx.__dict__.update(oldctx.__dict__)
    # The reused observe-only fixture did not need this original wait_ack owner.
    ctx.idle.GUI = '/alicia_supervisor_gui'
    session.ctx = ctx
    rows = archived.readiness.recorded_rows(archived.documents['snapshot'])
    now = archived.documents['snapshot']['finished_ns']; epoch = archived.documents['snapshot']['state']['epoch_ns']
    ctx.acceptance['state']['epoch_ns'] = epoch
    task_message = task(); task_message.header.stamp = archived.readiness._Stamp(now)
    messages = {key:value[0] for key,value in session.states.items()}
    messages['task'] = task_message; messages['epoch'].stamp = archived.readiness._Stamp(epoch)
    messages['manual']._connection_header = dict(callerid='/alicia_supervisor_gui',latching='1')
    session.states = {key:(value,now) for key,value in messages.items()}
    session.samples = [dict(stamp_ns=now-i) for i in range(20)]
    mode = {'manual':True}; selected = dict(mode='unknown',strategy='observe_return',generation=1,stamp_ns=100)
    ros = session.rospy
    ros.Time = NS(now=lambda:archived.readiness._Stamp(now))
    ros.get_param = lambda name:selected if name=='/grasp_mode/selection' else mode['manual']
    ros.wait_for_message = lambda topic,kind,timeout:messages[next(k for k,n,_ in session.topics if n==topic)]
    def set_param(name,value):
        assert name == '/gui/joint_direct_mode'; mode['manual'] = value
    ros.set_param = set_param
    message_module = types.ModuleType('alicia_flexible_grasp_supervisor.msg'); message_module.GraspState = object
    service_module = types.ModuleType('alicia_grasp_modes.srv'); service_module.SetGraspMode = object
    monkeypatch.setitem(sys.modules,'rospy',ros)
    monkeypatch.setitem(sys.modules,message_module.__name__,message_module)
    monkeypatch.setitem(sys.modules,service_module.__name__,service_module)
    observations = []
    def snapshot():
        observations.append(mode['manual']); return rows
    observer = NS(snapshot=snapshot,close=lambda:observations.append('closed'))
    monkeypatch.setattr(archived.typed,'Observer',lambda rospy:observer)
    f.revision.typed,f.revision.assets = archived.typed,archived.assets
    ctx.continuation = NS(base=f.revision,h4=h4,readiness=archived.readiness,history=archived.history)
    ctx.typed_observer,ctx.typed_admission = None,[]
    session.connect = lambda:None; session.save = lambda:None
    published = []
    class Publisher:
        def get_num_connections(self): return 1
        def publish(self,message):
            published.append(message.data); messages['manual'].data = message.data
            session.states['manual'] = (messages['manual'],now)
        def unregister(self): pass
    def publisher(*args,**kwargs):
        for topic,owners in f.state[0]:
            if topic == '/gui/joint_direct_mode': owners.append(ros.get_name())
        return Publisher()
    ros.Publisher = publisher; session.Bool = lambda **kwargs:NS(**kwargs)
    prepared = NS(released=False,process=NS(poll=lambda:None),ready=dict(offline_imports_only=True),close=lambda:None)
    ctx.warm = NS(PrewarmedModeRunner=lambda *args,**kwargs:prepared)
    ros.ServiceProxy = lambda name,kind:lambda *args:NS(success=True,message='fixture committed')
    f.state[2].extend([[name,['/grasp_mode_router']] for name in ('/grasp_mode/select','/grasp/start')])
    monkeypatch.setattr(entry,'wait_selection_commit',lambda rospy,prior:(deepcopy(selected),dict(fixture=True)))
    return NS(f=f,ctx=ctx,session=session,rows=rows,observations=observations,
              mode=mode,messages=messages,published=published,observer=observer,prepared=prepared)


def test_original_preflight_and_prewarm_recheck_history_before_mode_then_auto_selection(session_io):
    f = session_io; session = f.session
    session.preflight()
    assert len(f.ctx.typed_admission) == 1 and f.observations == [True]
    session.enter_automatic()
    assert f.published == [False] and len(f.ctx.typed_admission) == 2
    assert f.observations == [True,True]
    selected = session.select()
    assert selected['strategy'] == 'observe_return'
    assert f.observations == [True,True]  # no sdk_manual claim after mode transfer
    assert f.f.recorder.reference is not None
    f.ctx.close(); assert f.observations[-1] == 'closed'


@pytest.mark.parametrize('fault',['wrong_sequence','held_task','foreign_recorder','foreign_owner','simulation_policy'])
def test_real_preflight_refuses_bad_admission_before_any_mode_request(session_io,monkeypatch,fault):
    f = session_io
    if fault == 'wrong_sequence':
        f.rows['driver'][-1].action_transport_sequence += 1
        # Advance only the adapter's wait clock after the first real validator
        # rejection; no five-second sleep or positive validator substitution.
        ticks = iter((0.,0.,5.,5.))
        monkeypatch.setattr(m,'time',NS(monotonic=lambda:next(ticks),sleep=lambda _:None,time_ns=lambda:0))
    elif fault == 'held_task': f.messages['task'].state,f.messages['task'].stage = 'HOLDING',10
    elif fault == 'foreign_recorder': f.f.state[0].append(['/joint_commands',[f.f.name]])
    elif fault == 'foreign_owner': f.f.state[1].append(['/foreign_subscriber',['/foreign']])
    else: f.f.actual_simulation_values['/mujoco_digital_twin/min_score'] = 79
    with pytest.raises(RuntimeError): f.session.preflight()
    assert f.published == [] and f.session.automatic_requested is False


def test_new_trajectory_sequence_and_unavailable_compute_do_not_gate_original_manual_return(session_io,monkeypatch):
    f = session_io
    f.rows['driver'][-1].action_transport_sequence += 10
    f.rows['hardware'][-1].action_transport_sequence += 10
    def forbidden(*args,**kwargs): raise AssertionError('new typed/compute/recorder gate entered during cleanup')
    monkeypatch.setattr(m,'wait_ready',forbidden)
    f.ctx.rev.simulation_policy=forbidden; f.ctx.rev.graph=forbidden
    f.ctx.recorder.check=forbidden; f.ctx.recorder.graph_owner=forbidden
    f.ctx.health.observe=forbidden; f.ctx.health.check_binding=forbidden
    assert f.session.observe(manual_return=True)['manual_parameter'] is True
    assert f.ctx.typed_admission == [] and f.observations == []


def test_manual_return_still_rejects_changed_control_owner(session_io):
    f = session_io; original = f.ctx.rev.node
    def node(master,name,caller):
        value = original(master,name,caller)
        if name == '/motion_gateway': value['start_ticks'] += 'reused'
        return value
    f.ctx.rev.node = node
    with pytest.raises(RuntimeError,match='manual-return owner changed'):
        f.session.observe(manual_return=True)
