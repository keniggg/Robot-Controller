"""Independent read-only GEN62 adapter contract probe; never invokes live APIs."""
import ast
from copy import deepcopy
import hashlib
import importlib.util
import inspect
import json
import os
from pathlib import Path
import platform
import sys
from types import SimpleNamespace

sys.dont_write_bytecode = True
W = Path(__file__).resolve().parents[1]
G = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1')
OUT = G/'contact_feedback_fix_r1/runtime_independent_review_r1'
RUNTIME = W/'.runtime/gen62_compute_runtime_20261022.py'
RUNTIME_SHA = '871b1b6b2ae048940beedd9d1a7c214fe2c9ff4201a981e1fd1f71efab6829de'
STARTER = W/'.runtime/start_gen62_compute_20261022.py'
STARTER_SHA = 'e4ad349339594bb0ffeaa57511b70c5c8462ab8df3d6aec866a588ab2e920460'
RESUME = W/'.runtime/gen62_resume_runtime_20261022.py'
RESUME_SHA = 'd9f1ac014f09b5743a7bddedaa8219b386161d41bd5c3eacebf30afcd0ee3127'
DESIGN = G/'contact_feedback_fix_r1/runtime_design_r1/bundle.json'

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def read(path):
    return json.loads(Path(path).read_bytes())

def save(path, value):
    raw = (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n').encode()
    with path.open('xb') as stream:
        stream.write(raw); stream.flush(); os.fsync(stream.fileno())
    assert path.read_bytes() == raw

def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result

def dump(tree):
    return ast.dump(tree, include_attributes=False)

def function(path, name):
    return next(n for n in ast.parse(path.read_bytes()).body
                if isinstance(n, ast.FunctionDef) and n.name == name)

def main():
    pins = {str(RUNTIME):RUNTIME_SHA, str(STARTER):STARTER_SHA, str(RESUME):RESUME_SHA}
    assert all(sha(p) == h for p,h in pins.items())
    assert OUT.parent.is_dir() and not OUT.exists()
    OUT.mkdir()
    # Cache standard-library platform metadata before the guard. This does
    # not run platform's optional `uname -p` subprocess.
    u = os.uname()
    platform._uname_cache = platform.uname_result(u.sysname,u.nodename,u.release,u.version,u.machine,'')
    guard_attempts = []
    def guard(event, args):
        forbidden = (event in {'socket.connect','socket.connect_ex','socket.bind','socket.listen',
                              'subprocess.Popen','os.system','os.fork','os.forkpty','os.posix_spawn'}
                     or event.startswith('os.exec'))
        if event == 'open' and isinstance(args[0], (str, bytes)):
            path = os.fsdecode(args[0])
            if path.startswith(('/dev/','/sys/')) or (path.startswith('/proc/') and
                    path != '/proc/sys/kernel/random/boot_id'):
                forbidden = True
        if forbidden:
            guard_attempts.append(dict(event=event, arg=repr(args[:1])[:200]))
            raise RuntimeError('independent offline guard: '+event)
    sys.addaudithook(guard)
    adapter = load(RUNTIME, 'independent_gen62_runtime')
    assert adapter.RESUME_SHA == RESUME_SHA
    captured = []
    builtin_compile = compile
    def capture(tree, *args, **kwargs):
        if isinstance(tree, ast.AST):
            captured.append(deepcopy(tree))
        return builtin_compile(tree, *args, **kwargs)
    adapter.compile = capture
    baseline = read(adapter.BASE)['source_sha256']
    resume, actual = adapter.resumed_context(baseline)
    assert len(actual['pins']) == 422 and len(actual['nodes']) == 17 and len(actual['preserved']) == 14
    assert str(resume.CONTROL) in {str(p) for p in actual['pins']}
    assert actual['pins'][resume.CONTROL] == resume.CONTROL_SHA
    expected = ast.Module(body=[function(RESUME, 'context')], type_ignores=[])
    call = expected.body[0].body[-2].value
    assert dump(call) == dump(ast.parse("recheck(ctx,release['source_sha256'])", mode='eval').body)
    call.args[1] = ast.Name(id='expected_sources',ctx=ast.Load())
    assert dump(captured[-1]) == dump(expected)
    assert actual['binding']['node_identity']['pid'] == actual['nodes']['/remote_grasp6d_node']['pid']
    assert actual['binding']['parameters'] == actual['config']['remote_parameters']
    assert not any(str(W/p) in {str(k) for k in actual['pins']} for p in adapter.PRODUCTION)

    # The real design receipt remains unchanged and non-deployable. Context
    # selection is tested against an IN-MEMORY fixture only. The accept-phase
    # source check records the expected map; no candidate source is installed.
    real_read = adapter.read
    original_design_sha = sha(DESIGN)
    fixture_base = read(DESIGN)
    assert fixture_base['deployment_ready'] is False
    fixture_base['deployment_ready'] = True
    fixture_base['pins'].update(pins)
    phase_calls = []
    active_fixture = [None]
    def fixture_read(path):
        return active_fixture[0] if Path(path) == DESIGN else real_read(path)
    adapter.read = fixture_read
    def fixture_context(sources):
        phase_calls.append(deepcopy(sources))
        ctx = dict(actual, pins=dict(actual['pins']), nodes=deepcopy(actual['nodes']),
                   preserved=deepcopy(actual['preserved']))
        verifier = SimpleNamespace(**actual['v'].__dict__)
        def expected_sources(current):
            assert current == sources
        verifier.source_check = expected_sources
        ctx['v'] = verifier
        return resume, ctx
    adapter.resumed_context = fixture_context
    records = []
    mutations = {
        'design_not_deployable': lambda b: b.update(deployment_ready=False),
        'integer_truth_not_bool': lambda b: b.update(deployment_ready=1),
        'kind': lambda b: b.update(kind='GEN61_three_compute_bundle'),
        'old_helper_pin': lambda b: b['pins'].update({str(RUNTIME):'0'*64}),
        'old_current_control_pin': lambda b: b['pins'].update({str(resume.CONTROL):resume.OLD_CONTROL_SHA}),
        'changed_historical_control_pin': lambda b: b['pins'].update({str(G/'user_pause_r2/control_before.json'):'0'*64}),
        'wrong_candidate_source': lambda b: b['source_sha256'].update({adapter.PRODUCTION[0]:'0'*64}),
        'missing_new_fixture': lambda b: b['source_sha256'].pop(adapter.ADDITIONS[1]),
        'different_baseline': lambda b: b['baseline_source_sha256'].update({adapter.PRODUCTION[0]:'0'*64}),
        'changed_relay': lambda b: b['load_resolution_sha256'].update({next(iter(b['load_resolution_sha256'])):'0'*64}),
        'different_prior_pid': lambda b: b['nodes']['/grasp_task_node'].update(pid=-1),
        'lost_preserved_node': lambda b: b['preserved'].pop('/motion_gateway'),
    }
    for action in ('deploy','accept'):
        for label,mutation in [('valid_in_memory_phase',None), *mutations.items()]:
            value = deepcopy(fixture_base)
            if mutation: mutation(value)
            active_fixture[0] = value
            before_count = len(phase_calls)
            try:
                facade, ctx, bound = adapter.context(SimpleNamespace(bundle=DESIGN,
                    bundle_sha256=original_design_sha,action=action))
            except (RuntimeError,AssertionError,KeyError) as error:
                assert mutation is not None, str(error)
                records.append(dict(action=action,case=label,rejected=True,reason=str(error)))
            else:
                assert mutation is None, 'bad fixture admitted: '+label
                wanted = value['baseline_source_sha256'] if action=='deploy' else value['source_sha256']
                assert len(phase_calls) == before_count+1 and phase_calls[-1] == wanted
                assert facade.recheck is resume.recheck
                assert facade.fresh_remote_health is adapter.current_remote_health
                assert ctx['prior_transition']['units'] == actual['transition']['units']
                assert ctx['prior_construction'] == read(actual['transition']['construction'])
                records.append(dict(action=action,case=label,rejected=False,
                    phase_source_count=len(wanted),new_source_installed=False))
    assert sha(DESIGN) == original_design_sha and read(DESIGN)['deployment_ready'] is False

    # Independent exact AST expectation for both current and explicitly absent
    # task sampling. Preserve every other original SDK/owner/manual/idle gate.
    from types import ModuleType
    facade=ModuleType('independent_sampling_facade')
    facade.__dict__.update(actual['base'].__dict__)
    facade.recheck=resume.recheck
    original_sample=function(adapter.COMMON,'sample_idle')
    gate_count=sum(isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='require'
                   for n in ast.walk(original_sample))
    class ExpectedSample(ast.NodeTransformer):
        def __init__(self, absent): self.absent=absent
        def visit_Compare(self,node):
            if dump(node)==dump(ast.parse("selection['generation']==3",mode='eval').body):
                node.comparators[0]=ast.Constant(5)
            return self.generic_visit(node)
        def visit_Assign(self,node):
            if self.absent and any(isinstance(t,ast.Name) and t.id=='topics' for t in node.targets):
                node.value.elts=[e for e in node.value.elts if e.elts[0].value!='task']
            return self.generic_visit(node)
        def visit_UnaryOp(self,node):
            if self.absent and dump(node)==dump(ast.parse("not states['task'].active",mode='eval').body):
                return ast.Constant(True)
            return self.generic_visit(node)
        def visit_Attribute(self,node):
            if self.absent and dump(node)==dump(ast.parse("states['task'].state",mode='eval').body):
                return ast.Constant('PROCESS_ABSENT_VERIFIED')
            return self.generic_visit(node)
    sample_checks=[]
    for absent in (False,True):
        f,counts=adapter.idle_function(facade,absent)
        expected=ExpectedSample(absent).visit(ast.Module(body=[deepcopy(original_sample)],type_ignores=[]))
        assert dump(captured[-1])==dump(expected)
        assert f.__globals__['recheck'] is resume.recheck
        assert f.__globals__['SELECTION']==adapter.SELECTION
        assert sum(isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='require'
                   for n in ast.walk(captured[-1])) == gate_count == 20
        sample_checks.append(dict(task_absent=absent,exact_ast=True,require_calls=gate_count,changes=counts))
    previous_start=(W/'.runtime/start_gen61_compute_20261022.py').read_bytes()
    assert previous_start.replace(b'gen61',b'gen62').replace(b'GEN61',b'GEN62')==STARTER.read_bytes()
    assert not guard_attempts
    save(OUT/'offline_contract.json',dict(passed=True,helper_sha256=pins,
        actual_baseline_context_pin_count=len(actual['pins']),actual_baseline_source_count=len(baseline),
        actual_old_nodes=len(actual['nodes']),actual_preserved_nodes=len(actual['preserved']),
        actual_resume_control_revision=69,actual_resume_epoch=adapter.EPOCH,actual_selection=adapter.SELECTION,
        resumed_context_exact_single_ast_substitution=True,phase_context_cases=records,
        phase_accept_source_check_is_in_memory_model_not_installation=True,
        source_map_recheck_bound_to_original_resume69=True,sampling_ast_checks=sample_checks,
        starter_exact_identifier_mapping=True,design_bundle_unchanged=True,
        design_sha256=original_design_sha,guard_attempts=guard_attempts,
        live_ROS_contacted=False,live_process_inspection=False,motion_or_mode_actions=False,
        script_sha256=sha(__file__)))
    print(json.dumps(dict(passed=True,path=str(OUT/'offline_contract.json'),
        sha256=sha(OUT/'offline_contract.json'),cases=len(records),pins=len(actual['pins']))))

if __name__=='__main__':
    main()
