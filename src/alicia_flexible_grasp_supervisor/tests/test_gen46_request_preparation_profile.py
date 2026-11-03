"""Execute staged production preparation methods through explicit local providers.

No ROS modules are imported. The exact methods are AST-extracted to avoid
initializing the production module; device/geometry/WSL results are providers.
This verifies diagnostic preservation, not the physical planning algorithms.
"""
import ast
from collections.abc import Mapping
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from dataclasses import dataclass, replace
import importlib.util
import json
from pathlib import Path
import threading
import time
import traceback
from types import MappingProxyType, SimpleNamespace as NS

import pytest

C = Path(__file__).resolve().parents[3]
P = C/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/request_preparation_profile.py'
spec = importlib.util.spec_from_file_location('alicia_flexible_grasp.vision.gen46_profile_test_subject', P)
profile_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(profile_module)
KEY = profile_module.KEY


class Cancel(RuntimeError):
    def __init__(self, code):
        self.code = code
        super().__init__(code)


class Snapshot:
    ok = True
    frame_id = 'camera_link'
    object_msg = NS(detected=True, label='unknown_small')


@dataclass(frozen=True)
class Prepared:
    remote_diagnostics: object
    ticket: object = None
    request_preparation_profile: object = None
    remote_performance: object = None


def prepared_factory(**kwargs):
    return Prepared(profile_module.freeze(kwargs['remote_diagnostics']), kwargs['ticket'],
        kwargs.get(KEY), profile_module.freeze(kwargs['remote_performance']))


def ticket(number):
    return NS(request_id=number, generation=7, target_epoch=4,
        snapshot_stamp_sec=100.+number,
        payload=(Snapshot(), NS(candidate_target_gate_enabled=True, requires_support_plane=False)))


class Providers:
    near_field_planning_active = True
    _near_field_planning_stage = 'contact_replan'
    candidate_frame_convention = 'original'
    _mode_work_lock = threading.RLock()

    def __init__(self, fail=None, barrier=None):
        self.fail = fail
        self.barrier = barrier
        self.checks = 0
        self.error = Cancel('TARGET_LOST')
        self.calls = []

    def step(self, name):
        self.calls.append(name)
        if self.fail == name:
            raise self.error

    def _require_stream_ticket_current(self, unused):
        self.checks += 1
        self.step('lifecycle_'+str(self.checks))

    def _geometry_state_guard(self): return threading.RLock()
    def _capture_geometry_generation(self): return 8
    def _require_graspnet_input_prerequisites(self, *args): self.step('parameters')
    def _refresh_runtime_params(self): pass
    def _snapshot_ros_stamp(self, snapshot): return 1

    def _prepare_snapshot_geometry(self, *args):
        self.step('geometry')
        return NS(ok=True), None, None

    def _prepare_regular_box_snapshot(self, snapshot, estimate, transform):
        self.step('box')
        return estimate

    def _generate_tabletop_candidates(self, *args, **kwargs):
        self.step('tabletop')
        if self.barrier is not None:
            self.barrier.wait(timeout=2.)
        return (), {}

    def _maybe_prepare_observation_from_contact_geometry(self, *args): return False
    def _gripper_contract_mismatch_reason(self): return ''
    def _build_frozen_graspnet_input(self, *args, **kwargs):
        self.step('remote_input')
        return {}, {}

    def _camera_intrinsics(self): return NS()
    def _remote_candidate_limit(self, unused): return 8
    def _predict_remote(self, *args, **kwargs):
        self.step('predict')
        return (), {}, {}, 0., 0., 0.

    def _independent_observation_planning_active(self): return False
    def _unknown_observation_planning_active(self): return False
    def _capture_silhouette_source(self, unused): return None
    def _add_silhouette_source_diagnostic(self, prepared, source):
        self.step('postdiagnostic')
        return prepared

    def _add_silhouette_envelope_diagnostic(self, prepared): return prepared


def method(path, class_name):
    module = ast.parse(path.read_bytes())
    cls = next(n for n in module.body if isinstance(n, ast.ClassDef) and n.name == class_name)
    return next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_prepare_and_predict')


namespace = dict(time=time, profiled_preparation=profile_module.profiled_preparation,
    profiled_prepared=profile_module.profiled_prepared, REQUEST_PROFILE_KEY=KEY,
    inherit_prepared_profile=profile_module.inherit_prepared_profile,
    SnapshotResult=Snapshot, StreamResultCancelled=Cancel, CandidateContractError=Cancel,
    _snapshot_fused_scene_evidence=lambda *args: {},
    FrozenSnapshotCandidatePoseEstimator=lambda *args, **kwargs: NS(),
    PreparedPrediction=prepared_factory, Providers=Providers,
    remote_prediction_failure_code=lambda error: type(error).__name__)
base_path=C/'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py'
mode_path=C/'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py'
tree=ast.Module(body=[ast.ClassDef(name='Base', bases=[ast.Name(id='Providers', ctx=ast.Load())],
    keywords=[], body=[method(base_path, 'RemoteGrasp6DNode')], decorator_list=[]),
    ast.ClassDef(name='Mode', bases=[ast.Name(id='Base', ctx=ast.Load())], keywords=[],
        body=[method(mode_path, 'ModeAwareRemoteGrasp6DNode')], decorator_list=[])], type_ignores=[])
exec(compile(ast.fix_missing_locations(tree), str(base_path), 'exec'), namespace)
Base, Mode = namespace['Base'], namespace['Mode']


def components(profile, scope='base_prepare'):
    return next(row['components'] for row in profile['scopes'] if row['scope'] == scope)


@pytest.mark.parametrize('failure,stage', [
    ('parameters', 'parameters'), ('geometry', 'geometry_registration'),
    ('box', 'geometry_registration'), ('tabletop', 'tabletop_candidates'),
    ('remote_input', 'remote_input'), ('predict', 'predict')])
def test_real_prepare_cancellation_keeps_exception_and_partial_completed_stages(failure, stage):
    node=Base(fail=failure); request=ticket(15)
    with pytest.raises(Cancel) as caught:
        node._prepare_and_predict(request)
    assert caught.value is node.error and caught.value.code == 'TARGET_LOST'
    assert any(frame.name == 'step' for frame in traceback.extract_tb(caught.value.__traceback__))
    profile=getattr(caught.value, KEY)
    rows=components(profile)
    assert profile['ticket']['request_id'] == 15
    assert any(row['name'] == stage and not row['completed'] for row in rows)
    assert 'predict' not in node.calls if failure != 'predict' else True
    with pytest.raises(TypeError): profile['ticket']['request_id']=99
    with pytest.raises(TypeError): rows[0]['completed']=True


def test_post_tabletop_lifecycle_failure_does_not_blame_completed_tabletop():
    node=Base(fail='lifecycle_4')
    with pytest.raises(Cancel) as caught:
        node._prepare_and_predict(ticket(15))
    rows=components(getattr(caught.value, KEY))
    assert [row['completed'] for row in rows if row['name']=='tabletop_candidates'] == [True]
    assert any(row['name']=='lifecycle_check' and not row['completed'] for row in rows)
    assert not any(row['name'] in ('remote_input','predict') for row in rows)


def test_success_retains_original_stage_diagnostics_and_adds_profile_once():
    node=Base(); result=node._prepare_and_predict(ticket(16))
    assert set(result.remote_diagnostics['ros_prepare_stages_ms']) == {
        'parameters','geometry_registration','tabletop_candidates','remote_input'}
    profile=result.request_preparation_profile
    assert [s['scope'] for s in profile['scopes']] == ['base_prepare']
    assert all(row['completed'] for row in components(profile))
    assert not hasattr(node, KEY)


def test_mode_postdiagnostic_cancel_inherits_completed_base_without_duplicate_statistics():
    node=Mode(fail='postdiagnostic')
    with pytest.raises(Cancel) as caught:
        node._prepare_and_predict(ticket(17))
    assert caught.value is node.error and node._observe_return_registration_ticket is None
    profile=getattr(caught.value, KEY)
    assert {row['scope'] for row in profile['scopes']} == {'base_prepare','mode_prepare'}
    assert sum(row['name']=='tabletop_candidates' for row in components(profile))==1
    assert any(row['name']=='postdiagnostic' and not row['completed']
        for row in components(profile, 'mode_prepare'))


def test_mode_success_uses_existing_diagnostics_and_keeps_profiles_request_bound():
    node=Mode();result=node._prepare_and_predict(ticket(18));profile=result.request_preparation_profile
    assert len(profile['scopes'])==2 and profile['ticket']['request_id']==18
    assert 'ros_prepare_stages_ms' in result.remote_diagnostics


@pytest.mark.parametrize('node_class', [Base, Mode])
def test_success_profiles_do_not_change_legacy_diagnostic_or_performance_comparison(node_class):
    first=node_class()._prepare_and_predict(ticket(18))
    second=node_class()._prepare_and_predict(ticket(19))
    first_diagnostics=dict(first.remote_diagnostics)
    second_diagnostics=dict(second.remote_diagnostics)
    # Historical callers remove only this original, deliberately variable key.
    first_diagnostics.pop('ros_prepare_stages_ms')
    second_diagnostics.pop('ros_prepare_stages_ms')
    assert first_diagnostics==second_diagnostics
    assert KEY not in first.remote_diagnostics and KEY not in first.remote_performance
    assert first.remote_performance==second.remote_performance
    assert first.request_preparation_profile['ticket']['request_id']==18
    assert second.request_preparation_profile['ticket']['request_id']==19


def test_actual_prepared_dataclass_keeps_old_positional_fields_and_freezes_optional_profile():
    module=ast.parse(base_path.read_bytes())
    selected=[node for node in module.body
        if isinstance(node,ast.FunctionDef) and node.name=='_freeze_request_evidence'
        or isinstance(node,ast.ClassDef) and node.name=='PreparedPrediction']
    env=dict(dataclass=dataclass,SnapshotResult=Snapshot,Mapping=Mapping,
        MappingProxyType=MappingProxyType,deepcopy=deepcopy,np=NS(ndarray=type('Array',(),{})))
    exec(compile(ast.fix_missing_locations(ast.Module(body=selected,type_ignores=[])),
        str(base_path),'exec'),env)
    actual=env['PreparedPrediction']
    # Frozen GEN45 field order; baseline source SHA256: fa011168dbf945f518934ffd6e0b0705d78a45285c8eabde5e5afafbcdc36818
    # Keep the assertion independent of any neighboring IDE/worktree source.
    old_fields=['ticket', 'snapshot', 'stamp', 'geometry', 'pose_estimator', 'graspnet_input', 'candidates', 'remote_diagnostics', 'remote_performance', 'tabletop_candidates', 'remote_failure_code', 'remote_failure_reason', 'graspnet_input_audit', 'request_invalidation_generation', 'graspnet_input_config', 'model_choice', 'ros_prepare_ms', 'encode_ms', 'transport_ms', 'decode_ms', 'near_field', 'near_field_phase_id', 'near_field_phase_deadline_sec', 'target_observation', 'refinement_binding', 'fused_scene_audit']
    assert list(actual.__dataclass_fields__)==old_fields+[KEY]
    original_args=(ticket(42),Snapshot(),1,None,None,{},(),{'stable':True},{'remote':1})
    legacy=actual(*original_args)
    assert legacy.request_preparation_profile is None
    profile=profile_module.RequestPreparationProfile(ticket(42),'base_prepare').snapshot()
    observed=actual(*original_args,request_preparation_profile=profile)
    assert observed.remote_diagnostics==legacy.remote_diagnostics
    assert observed.remote_performance==legacy.remote_performance
    with pytest.raises(TypeError):observed.request_preparation_profile['ticket']['request_id']=0
    malformed=actual(*original_args,request_preparation_profile=object())
    assert malformed.request_preparation_profile is None
    assert malformed.remote_diagnostics==legacy.remote_diagnostics


@pytest.mark.parametrize('node_class', [Base, Mode])
def test_snapshot_diagnostic_failure_preserves_original_success(node_class,monkeypatch):
    def unavailable(unused):raise ValueError('diagnostic snapshot unavailable')
    monkeypatch.setattr(profile_module.RequestPreparationProfile,'snapshot',unavailable)
    result=node_class()._prepare_and_predict(ticket(43))
    assert result.request_preparation_profile is None
    assert set(result.remote_diagnostics)=={'tabletop_geometry','ros_prepare_stages_ms'}


def test_overlapping_real_preparations_do_not_mix_request_profiles():
    barrier=threading.Barrier(2)
    def run(number):
        node=Base(fail='lifecycle_4',barrier=barrier)
        try:node._prepare_and_predict(ticket(number))
        except Cancel as error:return error, getattr(error,KEY)
        raise AssertionError('expected original cancellation')
    with ThreadPoolExecutor(max_workers=2) as pool:
        first,second=list(pool.map(run,(21,22)))
    assert first[0] is not second[0]
    assert first[1]['ticket']['request_id']==21 and second[1]['ticket']['request_id']==22
    assert first[1] is not second[1]
    assert all(len([c for c in components(p) if c['name']=='tabletop_candidates'])==1 for _,p in (first,second))


def test_nested_wall_accounting_is_not_double_counted():
    clock=iter((0.,0.,1.,1.,2.,2.,4.,4.,5.,5.,6.,6.))
    p=profile_module.RequestPreparationProfile(ticket(23),'base_prepare',
        wall_clock=lambda:next(clock),cpu_clock=lambda:next(clock))
    with p.stage('outer'):
        with p.stage('inner'):pass
    rows=components(p.snapshot())
    assert sum(row['exclusive_wall_sec'] for row in rows)==4.
    assert [row['wall_sec'] for row in rows]==[2.,4.]


def test_worker_error_lookup_rejects_different_request():
    node=Base(fail='tabletop')
    with pytest.raises(Cancel) as caught:node._prepare_and_predict(ticket(24))
    assert profile_module.error_preparation_profile(caught.value,ticket(24)) is not None
    assert profile_module.error_preparation_profile(caught.value,ticket(25)) is None


def worker_profile_code():
    module=ast.parse(base_path.read_bytes())
    cls=next(n for n in module.body if isinstance(n,ast.ClassDef) and n.name=='RemoteGrasp6DNode')
    worker=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_stream_worker_main')
    # Execute the actual staged worker's diagnostic-only block, rather than
    # initializing an inference worker or changing original completion status.
    block=next(n for n in ast.walk(worker) if isinstance(n,ast.If)
        and any(isinstance(child,ast.Constant) and child.value=='partial_request_profile'
                for child in ast.walk(n)) and isinstance(n.test,ast.Compare) and isinstance(n.test.left,ast.Name)
        and n.test.left.id=='metrics')
    return compile(ast.fix_missing_locations(ast.Module(body=[block],type_ignores=[])),str(base_path),'exec')


def test_actual_worker_partial_metrics_fill_only_the_matching_cancelled_request():
    code=worker_profile_code()
    node=Base(fail='lifecycle_4');request=ticket(26)
    with pytest.raises(Cancel) as caught:node._prepare_and_predict(request)
    env=dict(metrics={'ros_prepare_ms':0.,'status':'TARGET_EPOCH_STALE','error':'TARGET_LOST'},
        prepared=None,error=caught.value,ticket=request,REQUEST_PROFILE_KEY=KEY,
        error_preparation_profile=profile_module.error_preparation_profile,
        _thaw_request_evidence=lambda value:value)
    exec(code,env)
    assert env['metrics']['ros_prepare_ms']>0.
    assert env['metrics']['ros_prepare_measurement_status']=='partial_request_profile'
    assert 'tabletop_candidates' in env['metrics']['ros_prepare_stages_ms']
    assert 'remote_input' not in env['metrics']['ros_prepare_stages_ms']
    assert env['metrics']['status']=='TARGET_EPOCH_STALE' and env['metrics']['error']=='TARGET_LOST'
    env.update(metrics={'ros_prepare_ms':0.},ticket=ticket(27))
    exec(code,env)
    assert env['metrics']=={'ros_prepare_ms':0.}


def test_actual_worker_reads_separate_success_profile_without_changing_legacy_metrics():
    request=ticket(28);prepared=Mode()._prepare_and_predict(request)
    original={'ros_prepare_ms':123.,'ros_prepare_stages_ms':{'parameters':1.},'status':'accepted'}
    env=dict(metrics=dict(original),prepared=prepared,error=None,ticket=request,
        REQUEST_PROFILE_KEY=KEY,error_preparation_profile=profile_module.error_preparation_profile,
        _thaw_request_evidence=lambda value:value)
    exec(worker_profile_code(),env)
    assert env['metrics'].pop(KEY) is prepared.request_preparation_profile
    assert env['metrics']==original
    assert KEY not in prepared.remote_diagnostics and KEY not in prepared.remote_performance


def test_unrelated_profile_cannot_turn_original_success_into_a_gate_failure():
    one=profile_module.RequestPreparationProfile(ticket(30),'base_prepare')
    two=profile_module.RequestPreparationProfile(ticket(31),'mode_prepare')
    prepared=Prepared({}, request_preparation_profile=one.snapshot())
    assert profile_module.profiled_prepared(prepared,two) is prepared
    assert two.inherit(one.snapshot()) is False


def test_partial_profile_survives_original_metrics_bound_with_recorded_G45_envelope():
    module=ast.parse(base_path.read_bytes())
    names={'_pipeline_count','strict_metrics_json','_metrics_original_totals','bounded_metrics_json'}
    functions=[n for n in module.body if isinstance(n,ast.FunctionDef) and n.name in names]
    env=dict(json=json,PIPELINE_METRICS_MAX_BYTES=12000,PIPELINE_COUNTER_FIELDS=())
    exec(compile(ast.fix_missing_locations(ast.Module(body=functions,type_ignores=[])),str(base_path),'exec'),env)
    node=Mode(fail='postdiagnostic')
    with pytest.raises(Cancel) as caught:node._prepare_and_predict(ticket(15))
    def plain(value):
        if isinstance(value,profile_module.Mapping):return {k:plain(v) for k,v in value.items()}
        if isinstance(value,(tuple,list)):return [plain(v) for v in value]
        return value
    metrics=json.loads((Path(__file__).parent/'fixtures/gen46_g45_request15_metrics.json').read_bytes())
    metrics[KEY]=plain(getattr(caught.value,KEY))
    encoded=env['bounded_metrics_json'](metrics)
    payload=json.loads(encoded)
    assert len(encoded.encode())<=12000
    assert payload[KEY]['ticket']['request_id']==15
    assert payload['error']=='TARGET_LOST' and payload['status']=='TARGET_EPOCH_STALE'


@pytest.mark.parametrize('node_class', [Base, Mode])
def test_partial_historical_ticket_keeps_original_typed_cancellation(node_class):
    # Exact ticket shape used by the unchanged historical reset/cancel test.
    partial=NS(generation=7,target_epoch=4)
    node=node_class(fail='lifecycle_1')
    with pytest.raises(Cancel) as caught:node._prepare_and_predict(partial)
    assert caught.value is node.error and caught.value.code=='TARGET_LOST'
    assert not hasattr(caught.value,KEY)
    assert node.calls==['lifecycle_1']


@pytest.mark.parametrize('malformed', [None, 'bad', {}, {'ticket':{}},
    {'ticket':{'request_id':15},'schema_version':1,'scopes':None}])
def test_worker_diagnostic_lookup_rejects_malformed_profile_without_raising(malformed):
    error=Cancel('TARGET_LOST');setattr(error,KEY,malformed)
    assert profile_module.error_preparation_profile(error,ticket(15)) is None
    assert profile_module.error_preparation_profile(error,NS(generation=7,target_epoch=4)) is None


def test_noop_profile_preserves_success_without_diagnostic_channel():
    prepared=object();profile=profile_module.NoOpPreparationProfile()
    assert profile.snapshot() is None
    assert profile_module.profiled_prepared(prepared,profile) is prepared
    assert profile_module.inherit_prepared_profile(profile,prepared) is False
    normal=profile_module.RequestPreparationProfile(ticket(40),'mode_prepare')
    assert profile_module.inherit_prepared_profile(normal,prepared) is False
    assert normal.inherited is None


def test_matching_ticket_with_malformed_component_cannot_reach_worker_aggregation():
    error=Cancel('TARGET_LOST')
    identity=profile_module.ticket_identity(ticket(15))
    for component in ({}, {'name':'parameters','completed':True,'wall_sec':float('nan')}, None):
        setattr(error,KEY,{'schema_version':1,'ticket':identity,
            'scopes':({'scope':'base_prepare','components':(component,)},)})
        assert profile_module.error_preparation_profile(error,ticket(15)) is None
