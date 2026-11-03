#!/usr/bin/python3
"""Pure fake-clock tests; extract only three functions, never import ROS/main."""
from pathlib import Path
from types import SimpleNamespace as NS
import ast
import hashlib
import json
import time
import copy
import importlib.util

import pytest

ROOT = Path(__file__).resolve().parents[1]
ENTRY = ROOT/'diagnostics/verify_physical_reference_startup.py'
SPEC = importlib.util.spec_from_file_location('physical_reference_startup_test', ENTRY)
CHECK = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHECK)
STAMP = 9_900_000_000
EPOCH = 1_000_000_000
NOW = 10_000_000_000
LAUNCH_BEFORE = (b'<launch>\n'
    b'  <include file="unchanged_model.launch"/>\n'
    b'  <param name="/grasp/unknown_physical_reference_mapping_enabled" type="bool" value="true"/>\n'
    b'</launch>\n')
AUDIT_PARAMETER_LINES = (
    b'  <param name="/grasp_6d/remote/gate_audit_output_path" value="$(env ALICIA_SESSION_LOG_DIR)/planning_audits/grasp6d_gate_audit_latest.json"/>\n',
    b'  <param name="/mujoco_digital_twin/audit_output_path" value="$(env ALICIA_SESSION_LOG_DIR)/planning_audits/grasp6d_mujoco_audit_latest.json"/>\n')
LAUNCH_AFTER = LAUNCH_BEFORE.replace(b'</launch>\n', b''.join(AUDIT_PARAMETER_LINES)+b'</launch>\n')


class Clock:
    def __init__(self):
        self.ros_ns = NOW
        self.mono_ns = 0
        self.sleeps = []
        self.mode = 'normal'

    def monotonic(self):
        return self.mono_ns*1e-9

    def sleep(self, seconds):
        assert 0 < seconds <= .01
        ns = max(1, round(seconds*1e9))
        self.sleeps.append(seconds)
        self.mono_ns += ns
        if self.mode == 'normal':
            self.ros_ns += ns
        elif self.mode == 'backward':
            self.ros_ns -= ns
        elif self.mode == 'forward':
            self.ros_ns += 1_000_000_000


def stamp(value):
    return NS(to_nsec=lambda: value)


def message(value):
    return NS(header=NS(stamp=stamp(value)))


def state():
    return NS(header=NS(stamp=stamp(STAMP)), epoch_ns=EPOCH,
              sdk_stamp_ns=STAMP, encoder_stamp_ns=STAMP,
              controller_stamp_ns=STAMP)


def scenario(delay_ns=0, clock_mode='normal', replacement_stamp=None,
             gate_error=None, changed_epoch=False, source_stamp=None,
             mutation=False, long_gate=False, missing_endpoint=False):
    raw = ENTRY.read_bytes()
    tree = ast.parse(raw, str(ENTRY))
    names = {'require', 'fresh', 'wait_for_fixed_query_sources'}
    selected = [node for node in tree.body
                if isinstance(node, ast.FunctionDef) and node.name in names]
    assert {node.name for node in selected} == names
    clock = Clock(); clock.mode = clock_mode
    states = dict(driver=state(), hardware=state())
    if changed_epoch:
        states['hardware'].epoch_ns += 1
    if source_stamp is not None:
        states['driver'].sdk_stamp_ns = source_stamp
    if missing_endpoint:
        states.pop('hardware')
    gate_calls = []
    def observed_ready(observer, epoch, rospy, assets, query_states):
        assert query_states is states and epoch == EPOCH
        gate_calls.append({key: id(value) for key, value in states.items()})
        if gate_error:
            raise RuntimeError(gate_error)
        if long_gate:
            clock.mono_ns += 500_000_000
            clock.ros_ns += 500_000_000
        return {'pure_mock_complete_original_gate': True}
    def snapshot():
        if mutation and clock.mono_ns:
            states['driver'].encoder_stamp_ns += 1
        value = STAMP if replacement_stamp is None else replacement_stamp
        ready = delay_ns is not None and clock.mono_ns >= delay_ns
        return {key: [message(value)] if ready else []
                for key in ('sdk', 'encoder', 'controller')}
    observer = NS(snapshot=snapshot)
    rospy = NS(Time=NS(now=lambda: stamp(clock.ros_ns)))
    namespace = {'time': clock, 'observed_ready': observed_ready}
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(ENTRY), 'exec'), namespace)
    audit = {}; error = None; result = None
    try:
        result = namespace['wait_for_fixed_query_sources'](
            observer, EPOCH, rospy, {}, states, audit)
    except RuntimeError as exc:
        error = str(exc)
    assert all('bounded_query' not in ast.dump(node)
               and 'ServiceProxy' not in ast.dump(node) for node in selected)
    return NS(result=result, error=error, audit=audit, clock=clock,
              states=states, gate_calls=gate_calls)



@pytest.mark.parametrize('delay', [0, 20_000_000])
def test_original_query_receipts_can_arrive_in_original_window(delay):
    r = scenario(delay_ns=delay)
    assert r.error is None and r.result and len(r.gate_calls) == 1
    assert r.audit['expires_ros_ns'] == STAMP+500_000_000
    assert r.audit['original_wait_budget_sec'] == .4
    assert r.audit['query_reissued'] is False and r.audit['receipt_refreshed'] is False
    assert r.gate_calls[0] == {key:id(value) for key,value in r.states.items()}
    assert bool(r.clock.sleeps) == bool(delay)


@pytest.mark.parametrize('reason,kwargs', [
    ('window', dict(delay_ns=None)),
    ('window', dict(replacement_stamp=STAMP+1)),
    ('backward', dict(delay_ns=None, clock_mode='backward')),
    ('window', dict(delay_ns=None, clock_mode='paused')),
    ('expired', dict(delay_ns=None, clock_mode='forward')),
    ('already stale', dict(source_stamp=NOW-500_000_001)),
    ('already stale', dict(source_stamp=NOW+1)),
    ('epoch differs', dict(changed_epoch=True)),
    ('response changed', dict(delay_ns=20_000_000, mutation=True)),
    ('manual source invalid', dict(gate_error='manual source invalid')),
    ('expired during', dict(long_gate=True)),
    ('both fixed', dict(missing_endpoint=True)),
])
def test_fixed_query_rejections_keep_original_deadline(reason, kwargs):
    r = scenario(**kwargs)
    assert r.result is None and r.error and reason in r.error, r.error
    assert not r.audit.get('completed', False)
    assert r.clock.mono_ns <= 900_000_000


@pytest.fixture
def derivation(tmp_path, monkeypatch):
    monkeypatch.setattr(CHECK, 'W', tmp_path)
    native, launch = str(tmp_path/CHECK.NATIVE_SOURCE), str(tmp_path/CHECK.LAUNCH_SOURCE)
    binary = str(tmp_path/'devel/lib/test_driver')
    xml_path=tmp_path/CHECK.LAUNCH_XML_SOURCE
    xml_path.parent.mkdir(parents=True)
    xml_path.write_bytes(LAUNCH_AFTER)
    baseline_path=tmp_path/'original.launch';baseline_path.write_bytes(LAUNCH_BEFORE)
    before_sha=hashlib.sha256(LAUNCH_BEFORE).hexdigest()
    after_sha=hashlib.sha256(LAUNCH_AFTER).hexdigest()
    baseline = dict(source_sha256={CHECK.NATIVE_SOURCE:'old-native', 'src/control.cpp':'same-cpp',
                                  CHECK.LAUNCH_XML_SOURCE:before_sha},
        current_sha256={native:'old-native', launch:'old-launch', binary:'same-binary',str(xml_path):before_sha},
        formal_build=dict(path='/original/build', sha256='same-build'),
        checklist=dict(path='/original/checklist', sha256='same-checklist'),
        expected_nodes={'driver':'/driver'}, topics={'state':{'type':'TypedState'}},
        type_md5={'TypedState':'same-md5'}, required_parameters={'mapping':True}, memory_attestation=False)
    current = copy.deepcopy(baseline)
    current.update(schema=3, kind='immutable_physical_reference_startup_identity',
        baseline_assets=dict(path='/baseline',sha256=CHECK.BASELINE_ASSETS_SHA256),
        baseline_launch=dict(path=str(baseline_path),sha256=before_sha),
        actual_regression=dict(path='/regression',sha256='regression-sha'))
    current['source_sha256'].update({CHECK.NATIVE_SOURCE:'new-native',
        CHECK.ENTRY_SOURCE:'new-entry', CHECK.STORAGE_SOURCE:'new-storage',CHECK.LAUNCH_XML_SOURCE:after_sha})
    current['current_sha256'].update({native:'new-native', launch:'new-launch',str(xml_path):after_sha})
    current['formal_build']['path']='/copied/build'
    current['checklist']['path']='/copied/checklist'
    regression = dict(passed=True,offline_scope_complete=True,source_unchanged=True,
        external_boundary_attempts=[],source_sha256=dict(current['source_sha256'], **{CHECK.LAUNCH_SOURCE:'new-launch'}))
    inputs={'/baseline':baseline,'/regression':regression}
    monkeypatch.setattr(CHECK, 'pinned', lambda ref: inputs[ref['path']])
    return NS(assets=current, baseline=baseline, regression=regression, native=native, launch=launch,
              binary=binary,xml_path=xml_path,baseline_path=baseline_path)


def test_exact_storage_derivation_keeps_typed_contract_and_current_evidence(derivation):
    CHECK.verify_derivation(derivation.assets)


@pytest.mark.parametrize('fault', [
    'schema','kind','baseline_sha','node','topic','md5','parameter','memory','build','checklist',
    'missing_source','extra_source','production_source','missing_artifact','compiled_artifact',
    'native_disagreement','regression_failed','scope_incomplete','source_changed','external_boundary',
    'untested_source','untested_launch','untested_xml_launch',
])
def test_storage_derivation_cannot_change_control_contract_or_evidence(derivation, fault):
    a, r = derivation.assets, derivation.regression
    if fault == 'schema': a['schema']=2
    elif fault == 'kind': a['kind']='different'
    elif fault == 'baseline_sha': a['baseline_assets']['sha256']='other'
    elif fault == 'node': a['expected_nodes']['driver']='/other'
    elif fault == 'topic': a['topics']['state']['type']='String'
    elif fault == 'md5': a['type_md5']['TypedState']='other'
    elif fault == 'parameter': a['required_parameters']['mapping']=False
    elif fault == 'memory': a['memory_attestation']=True
    elif fault == 'build': a['formal_build']['sha256']='other'
    elif fault == 'checklist': a['checklist']['sha256']='other'
    elif fault == 'missing_source': a['source_sha256'].pop(CHECK.STORAGE_SOURCE)
    elif fault == 'extra_source': a['source_sha256']['unknown.py']='unknown'
    elif fault == 'production_source': a['source_sha256']['src/control.cpp']='other'
    elif fault == 'missing_artifact': a['current_sha256'].pop(derivation.binary)
    elif fault == 'compiled_artifact': a['current_sha256'][derivation.binary]='other'
    elif fault == 'native_disagreement': a['current_sha256'][derivation.native]='other'
    elif fault == 'regression_failed': r['passed']=False
    elif fault == 'scope_incomplete': r['offline_scope_complete']=False
    elif fault == 'source_changed': r['source_unchanged']=False
    elif fault == 'external_boundary': r['external_boundary_attempts']=['device']
    elif fault == 'untested_source': r['source_sha256'][CHECK.ENTRY_SOURCE]='other'
    elif fault == 'untested_launch': r['source_sha256'][CHECK.LAUNCH_SOURCE]='other'
    elif fault == 'untested_xml_launch': r['source_sha256'][CHECK.LAUNCH_XML_SOURCE]='other'
    with pytest.raises(RuntimeError): CHECK.verify_derivation(a)


@pytest.mark.parametrize('fault', [
    'remote_path','mujoco_path','different_env','missing_param','duplicate_param','extra_param',
    'conditional_param','extra_type','mapping_disabled','model_changed','comment_added',
    'unrelated_whitespace','reordered_params','no_addition','baseline_bytes','baseline_sha','artifact_sha',
])
def test_launch_delta_is_only_two_exact_storage_parameters(derivation, fault):
    raw=LAUNCH_AFTER
    if fault == 'remote_path': raw=raw.replace(b'grasp6d_gate_audit_latest.json',b'other.json')
    elif fault == 'mujoco_path': raw=raw.replace(b'grasp6d_mujoco_audit_latest.json',b'other.json')
    elif fault == 'different_env': raw=raw.replace(b'ALICIA_SESSION_LOG_DIR',b'ROS_LOG_DIR')
    elif fault == 'missing_param': raw=raw.replace(AUDIT_PARAMETER_LINES[0],b'')
    elif fault == 'duplicate_param': raw=raw.replace(b'</launch>',AUDIT_PARAMETER_LINES[0]+b'</launch>')
    elif fault == 'extra_param': raw=raw.replace(b'</launch>',b'<param name="/other" value="true"/></launch>')
    elif fault == 'conditional_param': raw=raw.replace(b'name="/grasp_6d/remote/',b'if="false" name="/grasp_6d/remote/')
    elif fault == 'extra_type': raw=raw.replace(b'name="/grasp_6d/remote/',b'type="str" name="/grasp_6d/remote/')
    elif fault == 'mapping_disabled': raw=raw.replace(b'value="true"',b'value="false"')
    elif fault == 'model_changed': raw=raw.replace(b'unchanged_model.launch',b'other_model.launch')
    elif fault == 'comment_added': raw=raw.replace(b'</launch>',b'<!-- unrelated edit --></launch>')
    elif fault == 'unrelated_whitespace': raw=raw.replace(b'  <include',b'    <include')
    elif fault == 'reordered_params': raw=raw.replace(b''.join(AUDIT_PARAMETER_LINES),b''.join(reversed(AUDIT_PARAMETER_LINES)))
    elif fault == 'no_addition': raw=LAUNCH_BEFORE
    elif fault == 'baseline_bytes': derivation.baseline_path.write_bytes(LAUNCH_BEFORE+b' ')
    elif fault == 'baseline_sha': derivation.assets['baseline_launch']['sha256']='0'*64
    derivation.xml_path.write_bytes(raw)
    after_sha=hashlib.sha256(raw).hexdigest()
    # Even a purported current regression over the altered file cannot expand
    # the exact reviewed two-parameter derivation scope.
    derivation.assets['source_sha256'][CHECK.LAUNCH_XML_SOURCE]=after_sha
    derivation.assets['current_sha256'][str(derivation.xml_path)]=after_sha
    derivation.regression['source_sha256'][CHECK.LAUNCH_XML_SOURCE]=after_sha
    if fault == 'artifact_sha': derivation.assets['current_sha256'][str(derivation.xml_path)]='0'*64
    with pytest.raises(RuntimeError): CHECK.verify_derivation(derivation.assets)


def test_output_uses_frozen_selected_storage(tmp_path, monkeypatch):
    session=tmp_path/'session'; session.mkdir()
    binding=dict(backend='ssd',device='8:17',mount_id=123)
    calls=[]
    selected=NS(ros_log_root=tmp_path,
        validate=lambda path,**kwargs:calls.append((path,kwargs)))
    monkeypatch.setattr(CHECK,'select_storage',lambda backend:selected if backend=='ssd' else None)
    actual, frozen, root, output=CHECK.admit_output(
        dict(storage=binding,output_root=str(session)),session/'acceptance_typed')
    assert actual is selected and frozen==binding and root==session and output==session/'acceptance_typed'
    assert len(calls)==2 and all(c[1]==dict(subtree=tmp_path,expected=binding) for c in calls)


@pytest.mark.parametrize('fault',['existing','outside','changed_mount'])
def test_output_rejects_reuse_escape_and_changed_media(tmp_path,monkeypatch,fault):
    session=tmp_path/'session';session.mkdir();destination=session/'child'
    if fault=='existing': destination.mkdir()
    if fault=='outside': destination=tmp_path/'other'
    def validate(path,**kwargs):
        if fault=='changed_mount': raise RuntimeError('frozen storage identity changed')
    monkeypatch.setattr(CHECK,'select_storage',lambda backend:NS(ros_log_root=tmp_path,validate=validate))
    with pytest.raises(RuntimeError):
        CHECK.admit_output(dict(storage={'backend':'ssd'},output_root=str(session)),destination)


def test_audit_write_validates_frozen_media_and_never_overwrites(tmp_path):
    calls=[];binding={'backend':'ssd'}
    selected=NS(ros_log_root=tmp_path,validate=lambda path,**kwargs:calls.append((path,kwargs)))
    path=tmp_path/'receipt.json'; value={'passed':False,'reason':'not yet accepted'}
    CHECK.save(path,value,selected,binding)
    assert json.loads(path.read_bytes())==value
    assert calls==[(path,dict(subtree=tmp_path,expected=binding))]
    with pytest.raises(FileExistsError): CHECK.save(path,{'passed':True},selected,binding)
    assert json.loads(path.read_bytes())==value


def test_pinned_ref_rejects_changed_bytes(tmp_path):
    path=tmp_path/'evidence.json';path.write_text('{}')
    with pytest.raises(RuntimeError,match='pinned evidence changed'):
        CHECK.pinned(dict(path=str(path),sha256='0'*64))


def test_native_import_requires_exact_tested_bytes(tmp_path,monkeypatch):
    path=tmp_path/CHECK.NATIVE_SOURCE;path.parent.mkdir();path.write_text('IMPORTED = True\n')
    monkeypatch.setattr(CHECK,'W',tmp_path)
    good=hashlib.sha256(path.read_bytes()).hexdigest()
    module=CHECK.native_module(dict(source_sha256={CHECK.NATIVE_SOURCE:good}))
    assert module.IMPORTED is True
    path.write_text('raise RuntimeError("changed code must not execute")\n')
    with pytest.raises(RuntimeError,match='native verifier changed'):
        CHECK.native_module(dict(source_sha256={CHECK.NATIVE_SOURCE:good}))


@pytest.fixture
def python_derivation(derivation, monkeypatch):
    """Pure receipt fixture; never import/execute a hardware or ROS module."""
    previous = copy.deepcopy(derivation.assets)
    previous['post_build_source_updates'] = []
    for path in CHECK.R26_TYPED_CHANGED_SOURCES-{CHECK.ENTRY_SOURCE}:
        previous['source_sha256'][path] = 'old:'+path
    previous_ref = dict(path='/r24_assets', sha256=CHECK.R26_PREVIOUS_ASSETS_SHA256)
    base_ref = dict(path='/r24_release', sha256=CHECK.R26_BASE_RELEASE_SHA256)
    old_run_ref = dict(path='/r24_full', sha256='old-full')
    run_ref = dict(path='/r26_full', sha256='new-full')
    review_ref = dict(path='/r26_review', sha256='new-review')
    before = dict(previous['source_sha256'], **{CHECK.LAUNCH_SOURCE:'new-launch'})
    for path in CHECK.R26_FULL_CHANGED_SOURCES-CHECK.R26_TYPED_CHANGED_SOURCES:
        before[path] = 'old:'+path
    after = dict(before)
    for path in CHECK.R26_FULL_CHANGED_SOURCES:
        after[path] = 'r26:'+path
    old_id, skip_id, added_id = 'tests/test_existing.py::test_old', 'tests/test_existing.py::test_optional', 'tests/test_new.py::test_new'
    def case(nodeid, skip=False):
        row = dict(nodeid=nodeid, setup='passed', call='skipped' if skip else 'passed', teardown='passed')
        if skip: row['skip_reason'] = 'existing optional dependency'
        return row
    old_run = dict(expected_case_ids=[old_id,skip_id], case_results=[case(old_id),case(skip_id,True)],
                   skipped_cases=[skip_id], excluded_files=[])
    run = dict(passed=True,complete_scope_requested=True,offline_scope_complete=True,
               source_unchanged=True,changed_sources=[],external_boundary_attempts=[],bad_files=[],
               duplicate_cases=[],source_sha256=after,expected_case_ids=[old_id,skip_id,added_id],
               case_results=[case(old_id),case(skip_id,True),case(added_id)],skipped_cases=[skip_id],
               excluded_files=[],full_regression_passed=False)
    base = dict(schema=1,kind='current_offline_release',passed=True,reviewed=True,installed=True,
                source_sha256=before,source_count=len(before),actual_regression=old_run_ref,
                driver_build=previous['formal_build'],full_regression_passed=False)
    review = dict(reviewed=True,source_scope='r26_physical_source_admission_python_only',
                  base_release=base_ref,base_assets=previous_ref,actual_regression=run_ref,
                  formal_build=previous['formal_build'],source_delta=CHECK.r26_source_delta(before,after),
                  typed_runtime_contract_unchanged=True,formal_cpp_build_unchanged=True,
                  typed_artifacts_unchanged=True,reviewed_case_renames=[],added_case_ids=[added_id])
    current = copy.deepcopy(previous)
    for path in CHECK.R26_TYPED_CHANGED_SOURCES:
        current['source_sha256'][path] = after[path]
    current.update(schema=5,revision='r26_physical_source_admission_python_v1',actual_regression=run_ref,
                   runtime_loaded=False,physical_grasp_verified=False,automatic_execution_authorized=False,
                   python_source_derivation=dict(previous_assets=previous_ref,base_release=base_ref,
                       change_review=review_ref,source_delta=copy.deepcopy(review['source_delta']),
                       typed_source_delta=CHECK.r26_source_delta(previous['source_sha256'],current['source_sha256'])))
    build = dict(returncode=0,formal_release_build=True,source_sha256={'src/control.cpp':'same-cpp'})
    inputs = {'/baseline':derivation.baseline,'/r24_assets':previous,'/r24_release':base,
              '/r24_full':old_run,'/r26_full':run,'/r26_review':review,
              previous['formal_build']['path']:build}
    monkeypatch.setattr(CHECK,'pinned',lambda reference:inputs[reference['path']])
    native_calls=[]
    native=NS(release_evidence=lambda value:native_calls.append(('release',value)),
              driver_build_evidence=lambda value:(native_calls.append(('build',value))
                                                   or (previous['formal_build'],build)))
    monkeypatch.setattr(CHECK,'native_module',lambda value:native)
    return NS(assets=current,previous=previous,base=base,run=run,old_run=old_run,review=review,
              build=build,native=native,native_calls=native_calls,old_id=old_id,skip_id=skip_id,
              added_id=added_id,binary=derivation.binary,xml_path=derivation.xml_path)


def test_r26_exact_python_delta_reuses_formal_build_and_all_native_checks(python_derivation):
    v=python_derivation
    CHECK.verify_derivation(v.assets)
    assert set(CHECK.r26_source_delta(v.previous['source_sha256'],v.assets['source_sha256'])) == CHECK.R26_TYPED_CHANGED_SOURCES
    assert v.assets['current_sha256']==v.previous['current_sha256']
    assert [name for name,_ in v.native_calls]==['release','build']
    release=v.native_calls[0][1]
    assert release['source_sha256']==v.run['source_sha256']
    assert release['driver_build']==v.previous['formal_build']
    assert release['full_regression_passed'] is False


@pytest.mark.parametrize('fault',[
    'r25_schema','kind','revision','runtime_claim','physical_claim','automatic_claim',
    'previous_sha','base_sha','node','topic','md5','parameter','memory','build_ref','checklist',
    'baseline_ref','baseline_launch','post_build_update','compiled_artifact','model_artifact',
    'artifact_inventory','typed_source_missing','typed_source_extra','other_typed_source',
    'target_source_unchanged','typed_delta','failed_full','partial_full','incomplete_full',
    'source_changed','changed_sources','external_boundary','bad_file','duplicate_case',
    'extra_full_source','deleted_full_source','missing_full_change','full_typed_disagreement',
    'full_launch_disagreement','unreviewed','scope','review_base','review_assets','review_full',
    'review_build','review_delta','derivation_delta','runtime_contract','cpp_unchanged',
    'artifacts_unchanged','failed_build','boolean_build_code','not_formal_build','build_cpp_source',
    'case_removed','case_rename_unmapped','added_case_unreviewed','new_skip','skip_reason',
    'skip_phase','exclusion',
])
def test_r26_python_delta_cannot_change_artifacts_contract_or_evidence(python_derivation,fault):
    v=python_derivation;a=v.assets;r=v.run;q=v.review;d=a['python_source_derivation']
    target=next(path for path in CHECK.R26_TYPED_CHANGED_SOURCES if path!=CHECK.ENTRY_SOURCE)
    if fault=='r25_schema':a['schema']=4
    elif fault=='kind':a['kind']='other'
    elif fault=='revision':a['revision']='r25'
    elif fault=='runtime_claim':a['runtime_loaded']=True
    elif fault=='physical_claim':a['physical_grasp_verified']=True
    elif fault=='automatic_claim':a['automatic_execution_authorized']=True
    elif fault=='previous_sha':d['previous_assets']=dict(d['previous_assets'],sha256='wrong')
    elif fault=='base_sha':d['base_release']=dict(d['base_release'],sha256='wrong')
    elif fault=='node':a['expected_nodes']['driver']='/other'
    elif fault=='topic':a['topics']['state']['type']='String'
    elif fault=='md5':a['type_md5']['TypedState']='wrong'
    elif fault=='parameter':a['required_parameters']['mapping']=False
    elif fault=='memory':a['memory_attestation']=True
    elif fault=='build_ref':a['formal_build']['sha256']='wrong'
    elif fault=='checklist':a['checklist']['sha256']='wrong'
    elif fault=='baseline_ref':a['baseline_assets']['sha256']='wrong'
    elif fault=='baseline_launch':a['baseline_launch']['sha256']='wrong'
    elif fault=='post_build_update':a['post_build_source_updates'].append({'extra':True})
    elif fault=='compiled_artifact':a['current_sha256'][v.binary]='wrong'
    elif fault=='model_artifact':a['current_sha256'][str(CHECK.W/CHECK.LAUNCH_SOURCE)]='wrong'
    elif fault=='artifact_inventory':a['current_sha256']['/extra']='wrong'
    elif fault=='typed_source_missing':a['source_sha256'].pop(target)
    elif fault=='typed_source_extra':a['source_sha256']['src/extra.py']='wrong'
    elif fault=='other_typed_source':a['source_sha256']['src/control.cpp']='wrong'
    elif fault=='target_source_unchanged':a['source_sha256'][target]=v.previous['source_sha256'][target]
    elif fault=='typed_delta':d['typed_source_delta']={}
    elif fault=='failed_full':r['passed']=False
    elif fault=='partial_full':r['complete_scope_requested']=False
    elif fault=='incomplete_full':r['offline_scope_complete']=False
    elif fault=='source_changed':r['source_unchanged']=False
    elif fault=='changed_sources':r['changed_sources']=['changed']
    elif fault=='external_boundary':r['external_boundary_attempts']=['device']
    elif fault=='bad_file':r['bad_files']=['test']
    elif fault=='duplicate_case':r['duplicate_cases']=['case']
    elif fault=='extra_full_source':r['source_sha256']['src/extra.py']='wrong'
    elif fault=='deleted_full_source':r['source_sha256'].pop(target)
    elif fault=='missing_full_change':r['source_sha256'][target]=v.base['source_sha256'][target]
    elif fault=='full_typed_disagreement':r['source_sha256'][target]='other-r26'
    elif fault=='full_launch_disagreement':r['source_sha256'][CHECK.LAUNCH_SOURCE]='wrong'
    elif fault=='unreviewed':q['reviewed']=False
    elif fault=='scope':q['source_scope']='r25'
    elif fault=='review_base':q['base_release']=dict(q['base_release'],sha256='wrong')
    elif fault=='review_assets':q['base_assets']=dict(q['base_assets'],sha256='wrong')
    elif fault=='review_full':q['actual_regression']=dict(q['actual_regression'],sha256='wrong')
    elif fault=='review_build':q['formal_build']=dict(q['formal_build'],sha256='wrong')
    elif fault=='review_delta':q['source_delta'].pop(target)
    elif fault=='derivation_delta':d['source_delta'].pop(target)
    elif fault=='runtime_contract':q['typed_runtime_contract_unchanged']=False
    elif fault=='cpp_unchanged':q['formal_cpp_build_unchanged']=False
    elif fault=='artifacts_unchanged':q['typed_artifacts_unchanged']=False
    elif fault=='failed_build':v.build['returncode']=1
    elif fault=='boolean_build_code':v.build['returncode']=False
    elif fault=='not_formal_build':v.build['formal_release_build']=False
    elif fault=='build_cpp_source':v.build['source_sha256']['src/control.cpp']='wrong'
    elif fault=='case_removed':r['expected_case_ids'].remove(v.old_id)
    elif fault=='case_rename_unmapped':r['expected_case_ids'][0]='tests/test_existing.py::test_renamed'
    elif fault=='added_case_unreviewed':q['added_case_ids']=[]
    elif fault=='new_skip':r['skipped_cases'].append(v.added_id)
    elif fault=='skip_reason':r['case_results'][1]['skip_reason']='different'
    elif fault=='skip_phase':r['case_results'][1]['setup']='skipped'
    elif fault=='exclusion':r['excluded_files'].append({'path':'hidden.py','reason':'unreviewed'})
    with pytest.raises(RuntimeError):CHECK.verify_derivation(a)


def test_r26_explicit_one_to_one_case_rename_keeps_old_skip_evidence(python_derivation):
    v=python_derivation
    renamed='tests/test_existing.py::test_optional_renamed'
    v.run['expected_case_ids'][1]=renamed
    v.run['case_results'][1]['nodeid']=renamed
    v.run['skipped_cases']=[renamed]
    v.review['reviewed_case_renames']=[dict(old_nodeid=v.skip_id,new_nodeid=renamed,reason='reviewed descriptive rename')]
    CHECK.verify_derivation(v.assets)


@pytest.mark.parametrize('phase',['release','build','selection'])
def test_r26_cannot_bypass_original_native_evidence_failure(python_derivation,phase):
    v=python_derivation
    def reject(value):raise RuntimeError('original native validator refused')
    if phase=='release':v.native.release_evidence=reject
    elif phase=='build':v.native.driver_build_evidence=reject
    else:v.native.driver_build_evidence=lambda value:({'different':True},v.build)
    with pytest.raises(RuntimeError):CHECK.verify_derivation(v.assets)
