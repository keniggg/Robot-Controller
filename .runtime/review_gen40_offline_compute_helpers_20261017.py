"""Independent inert AST checks only; never execute deployment/pytest main."""
import ast
import difflib
import hashlib
import json
from pathlib import Path
import time
import types

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
D = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen36_restart_20261017T012037Z')
O = D/'verification/gen40_offline_compute_helpers_review_r1'
assert Path('/media/zhuyupei/SanStand').is_mount()
assert not O.exists()
O.mkdir()
inputs = {}


def read(path):
    path = Path(path)
    raw = path.read_bytes()
    h = hashlib.sha256(raw).hexdigest()
    assert inputs.setdefault(str(path), h) == h
    return raw


def load(path):
    return json.loads(read(path))


def dump(name, value):
    path = O/name
    raw = (json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True,
                      allow_nan=False)+'\n').encode()
    with path.open('xb') as f:
        f.write(raw)
    assert path.read_bytes() == raw
    return hashlib.sha256(raw).hexdigest()


runner = W/'.runtime/verify_gen40_offline_tests_20261017.py'
loader = W/'.runtime/load_gen40_idle_compute_20261017.py'
old_runner = W/'.runtime/verify_gen39_offline_tests_20261016.py'
old_loader = W/'.runtime/resume_gen39_compute_reload_20261016.py'
runner_text, loader_text = read(runner).decode(), read(loader).decode()
old_runner_text, old_loader_text = read(old_runner).decode(), read(old_loader).decode()
assert inputs[str(runner)] == 'd5d839bccd13d8bfe267b8f745eb8ccd5cd3f99a0a1cde7c47c6055ed802e498'
assert inputs[str(loader)] == '2066db2da96c33d77d73e4fa4b21f0ec1aa21e653c5eb96e3dbcee3fcd8e520e'
assert inputs[str(old_loader)] == 'e016504577fa9673cc7ab578ffce15ea83e049a339c2874cafdf4036f2c46a75'
expected = old_runner_text
for a, b in [
    ('gen38_release_manifest.json','gen39_release_manifest.json'),
    ('3f1f510df0e46508b33fe4d1c5c0dcbecd64744dd745feb7798d9fe07b060b6c',
     '47a2f5148d585dfa0d938868453e8293fffdefc35290f1b788bc013703257030'),
    ('gen38_baseline_source','gen39_baseline_source'),
    ("baseline['source_count'] == 1019 and baseline['test_summary']['tests'] == 7834",
     "baseline['source_count'] == 1025 and baseline['test_summary']['tests'] == 7962"),
    ("test_gen39*.py", "test_gen40*.py"),
    ('changed_from_gen38','changed_from_gen39'),
    ('changes_from_gen38.diff','changes_from_gen39.diff'),
    ("fromfile='GEN38/'+name, tofile='GEN39/'+name", "fromfile='GEN39/'+name, tofile='GEN40/'+name"),
]:
    assert a in expected
    expected = expected.replace(a,b)
assert expected == runner_text, 'runner has an unreviewed behavioral change'

release_path = D/'verification/gen39_release_manifest.json'
release = load(release_path)
assert inputs[str(release_path)] == '47a2f5148d585dfa0d938868453e8293fffdefc35290f1b788bc013703257030'
backup = W/'.runtime/gen39_baseline_source'
bm = load(backup/'manifest.json')
assert bm['source_count'] == release['source_count'] == 1025
assert bm['source_sha256'] == release['source_sha256']
assert release['test_summary'] == dict(tests=7962,failures=0,errors=0,skipped=0)
for name,digest in bm['source_sha256'].items():
    assert hashlib.sha256((backup/name).read_bytes()).hexdigest() == digest

tree, old_tree = ast.parse(loader_text), ast.parse(old_loader_text)
main = next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
old_main = next(n for n in old_tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
functions = {n.name:n for n in main.body if isinstance(n,ast.FunctionDef)}
old_functions = {n.name:n for n in old_main.body if isinstance(n,ast.FunctionDef)}
for name in ('sources','rpc','identity','old_process_alive','manual_idle','save'):
    assert ast.dump(functions[name], include_attributes=False) == ast.dump(old_functions[name], include_attributes=False), name
pin_old = old_functions['pinned_inputs']
pin_old.body = [n for n in pin_old.body if not (isinstance(n,ast.Assert)
    and 'failed' in ast.dump(n,include_attributes=False))]
assert ast.dump(functions['pinned_inputs'],include_attributes=False) == ast.dump(pin_old,include_attributes=False)

calls = [n for n in ast.walk(main) if isinstance(n,ast.Call)]
subprocess_calls = [n for n in calls if isinstance(n.func,ast.Attribute)
    and isinstance(n.func.value,ast.Name) and n.func.value.id=='subprocess' and n.func.attr=='run']
assert len(subprocess_calls) == 1
shutdowns = [n for n in calls if isinstance(n.func,ast.Attribute) and n.func.attr=='shutdown']
assert len(shutdowns) == 1 and isinstance(shutdowns[0].func.value,ast.Name) and shutdowns[0].func.value.id=='endpoint'
initial_idle = next(n for n in main.body if isinstance(n,ast.Try)).body[1]
assert isinstance(initial_idle,ast.Expr) and isinstance(initial_idle.value,ast.Call)
assert ast.dump(initial_idle.value,include_attributes=False) == ast.dump(ast.parse(
    "manual_idle(require_task=old_process_alive(expected['/grasp_task_node']))").body[0].value,include_attributes=False)
assert "expected = preflight['node_identities']" in loader_text
assert 'assert set(expected) == set(targets) | set(preserve)' in loader_text
assert "result.update(before={name: expected[name] for name in targets}" in loader_text
assert "assert result['preserved_before'] == {name: expected[name] for name in preserve}" in loader_text
assert "parser.add_argument('--failed'" not in loader_text
assert 'recovery_from=' not in loader_text
assert loader_text.index("while time.monotonic() < deadline") < loader_text.index('subprocess.run(command')
assert loader_text.index("manual_idle(require_task=False)",loader_text.index('deadline =')) < loader_text.index('subprocess.run(command')
assert loader_text.index("for service in ('/grasp_mode/reset_remote', '/grasp_mode/reset_task')") < loader_text.index("result['deployment_passed'] = True")
assert loader_text.index("assert all(result['after'][name] != result['before'][name]") < loader_text.index("result['deployment_passed'] = True")

checks = []


def compile_function(name, namespace):
    module = ast.Module(body=[functions[name]], type_ignores=[])
    exec(compile(ast.fix_missing_locations(module),str(loader),'exec'),namespace)
    return namespace[name]


def case(name, fn, should_raise=False):
    raised = False
    try:
        fn()
    except AssertionError:
        raised = True
    assert raised == should_raise, name
    checks.append(dict(name=name,passed=True,expected_assertion=should_raise))


class FakePath:
    content = ''
    missing = False
    def __init__(self, path):
        self.path = str(path)
    def __truediv__(self, item):
        return FakePath(self.path+'/'+str(item))
    def read_text(self):
        if self.missing:
            raise FileNotFoundError(self.path)
        return self.content


def proc(state='R',tick='123',ppid=888):
    fields = [state,str(ppid)]+['0']*17+[tick]+['0']*5
    return '345 (inert worker) '+' '.join(fields)


ns = {'Path':FakePath}
alive = compile_function('old_process_alive',ns)
previous = dict(pid=345,start_ticks='123',parent_pid=888)
FakePath.content = proc()
case('same_pid_start_tick_alive',lambda: (_ for _ in ()).throw(AssertionError()) if not alive(previous) else None)
FakePath.content = proc(state='Z')
case('zombie_not_alive',lambda: (_ for _ in ()).throw(AssertionError()) if alive(previous) else None)
FakePath.content = proc(tick='124')
case('pid_reuse_not_same_writer',lambda: (_ for _ in ()).throw(AssertionError()) if alive(previous) else None)
FakePath.missing = True
case('removed_proc_stat_not_alive',lambda: (_ for _ in ()).throw(AssertionError()) if alive(previous) else None)
FakePath.missing = False
FakePath.content = proc()
endpoint_calls = []
class Endpoint:
    code=1
    def getPid(self,caller):
        endpoint_calls.append(caller)
        return self.code,'ok',345
def forbidden_lookup(name):
    raise RuntimeError('provided endpoint must avoid name lookup')
ns = {'Path':FakePath,'rospy':types.SimpleNamespace(get_name=lambda:'/inert_review'), 'rpc':forbidden_lookup}
identity = compile_function('identity',ns)
endpoint = Endpoint()
case('resolved_endpoint_used_without_second_lookup',lambda: (_ for _ in ()).throw(AssertionError())
    if identity('/task',endpoint)!=previous else None)
endpoint.code = 0
case('getpid_failure_rejected',lambda:identity('/task',endpoint),True)
endpoint.code = 1
FakePath.content = proc(state='Z')
case('identity_rejects_zombie',lambda:identity('/task',endpoint),True)

state = dict(gui=True,active=False,start_reserved=False,pending=False,task_active=False)
waited = []
def get_param(name):
    assert name == '/gui/joint_direct_mode'
    return state['gui']
def wait_for_message(topic,kind,timeout):
    waited.append(topic)
    if topic=='/grasp_mode/status':
        return types.SimpleNamespace(data=json.dumps({k:state[k] for k in ('active','start_reserved','pending')}))
    assert topic=='/grasp/state'
    return types.SimpleNamespace(active=state['task_active'])
manual = compile_function('manual_idle',dict(rospy=types.SimpleNamespace(
    get_param=get_param,wait_for_message=wait_for_message),json=json,String=object,GraspState=object))
case('live_task_inactive_accepted',manual)
state['task_active']=True
case('live_task_active_rejected',manual,True)
waited.clear()
case('absent_old_task_does_not_require_absent_topic',lambda:manual(require_task=False))
assert '/grasp/state' not in waited
state['task_active']=False
for key in ('active','start_reserved','pending'):
    state[key]=True
    case('router_'+key+'_rejected',manual,True)
    state[key]=False
state['gui']=False
case('manual_control_not_owned_rejected',manual,True)
assert len(checks)==14

draft = D/'verification/gen40_compute_loader_draft_r1'
candidate = load(draft/'candidate.json')
read(draft/'helper.diff')
assert candidate['helper_sha256'] == inputs[str(loader)] and candidate['deployed'] is False
assert candidate['base_reviewed_helper_sha256'] == inputs[str(old_loader)]
old_review_path = D/'verification/gen39_compute_reload_recovery_independent_r4/result.json'
old_review = load(old_review_path)
assert old_review['reviewed'] and not old_review['blocking_findings']
assert old_review['reviewed_source_sha256']['.runtime/resume_gen39_compute_reload_20261016.py'] == inputs[str(old_loader)]
prior_deployment_path = D/'gen39_runtime/compute_deployment_r2/result.json'
prior_deployment = load(prior_deployment_path)
assert prior_deployment['deployment_passed'] is True
assert prior_deployment['recovery_helper_sha256'] == inputs[str(old_loader)]
startup = W/'.runtime/start_observation_measurement_consumers_20261008.sh'
launch = W/'.runtime/observation_measurement_consumers_20261008.launch'
read(startup);read(launch)
reviewed = {str(p.relative_to(W)):inputs[str(p)] for p in (runner,loader,old_runner,old_loader,startup,launch)}
for path,digest in inputs.items():
    assert hashlib.sha256(Path(path).read_bytes()).hexdigest()==digest, path
review = dict(schema_version=1,kind='gen40_offline_runner_normal_compute_loader_independent_review',
    recorded_ns=time.time_ns(),reviewed=True,helper_review_complete=True,release_review_complete=False,
    blocking_findings=[],reviewed_source_sha256=reviewed,evidence_sha256=inputs,
    helper_sha256={str(runner.relative_to(W)):inputs[str(runner)],str(loader.relative_to(W)):inputs[str(loader)]},
    runner_review=dict(exact_normalized_diff_only=True,baseline_release_sha256=inputs[str(release_path)],
        baseline_source_count=1025,baseline_case_count=7962,baseline_test_files=len(release['test_scope']),
        backup_all_1025_files_match=True,new_glob='test_gen40*.py',
        original_guard_scope_and_source_after_checks_unchanged=True,
        complete_scope_xml_assignment_original_file_case_counts_and_zero_FEskip_required=True,
        no_actual_pytest_run_performed=True),
    loader_review=dict(normal_preflight_entry=True,failed_receipt_fabricated=False,
        preflight_11_identity_partition='3 compute targets + 8 preserved hardware/controller/gui/router/gateway identities',
        live_old_task_requires_inactive=True,same_endpoint_identity_and_shutdown=True,
        old_process_exit_proof='bounded PID/start_ticks check for all old compute writers before one launch',
        shutdown_transport_exception_does_not_imply_exit=True,
        responsive_unexpected_replacement_rejected=True,preserved_before_prelaunch_after_identity_checks=True,
        reset_ack_services=['/grasp_mode/reset_remote','/grasp_mode/reset_task'],
        before_sources_are_preflight_original_identities=True,
        manifest_preflight_bytes_sources_evidence_selection_pinned=True,
        motion_or_temperature_interfaces_added=False,
        prior_reviewed_successful_helper_sha256=inputs[str(old_loader)],
        prior_live_success_sha256=inputs[str(prior_deployment_path)]),
    pure_ast_checks=checks,pure_ast_case_count=len(checks),
    source_inputs_unchanged=True,helper_main_executed=False,ros_contacted=False,
    hardware_contacted=False,process_mutated=False,motion_commands_sent=False,
    production_or_new_helper_modified=False,
    nonblocking_findings=[
        'Static/inert checks do not establish actual GEN40 stop/reset/launch success; eventual deployment receipts must prove those runtime facts.',
        'Runner inherits the existing Python socket/device guard; it does not claim a universal native C SDK boundary.',
        'Normal loader pins selection and preflight/source bytes; current control-file/epoch final acceptance remains the surrounding preflight/finalizer responsibility, as in the reviewed recovery entry.',
        'The start script launches only the existing three compute consumers; the separate gateway reload/finalizer still needs its own reviewed GEN40 entry and evidence.',
    ])
review_sha=dump('result.json',review)
diff=''.join(difflib.unified_diff(old_loader_text.splitlines(True),loader_text.splitlines(True),
    fromfile=str(old_loader.relative_to(W)),tofile=str(loader.relative_to(W))))
q=O/'loader_reviewed.diff';q.write_text(diff)
seal_sha=dump('seal.json',dict(schema_version=1,review_completed=True,release_review_complete=False,
    artifacts_sha256={'result.json':review_sha,'loader_reviewed.diff':hashlib.sha256(q.read_bytes()).hexdigest()},
    helper_sha256=review['helper_sha256'],script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    offline_only=True,helper_main_executed=False,blocking_findings=[]))
print(json.dumps(dict(result=str(O/'result.json'),result_sha256=review_sha,seal_sha256=seal_sha,
    pure_ast_cases=len(checks),blocking_findings=[]),ensure_ascii=False))
