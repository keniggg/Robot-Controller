"""Independent, synthetic admission checks only; never invokes helper main/ROS."""
import ast
from collections import Counter
from copy import deepcopy
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import platform
import sys
from types import SimpleNamespace as NS
import xml.etree.ElementTree as ET

W = Path(__file__).resolve().parents[1]
R = W/'.runtime'
V = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
OUT = V/('gen55_runtime_helpers_independent_review_'+sys.argv[1])
assert os.path.ismount('/media/zhuyupei/SanStand')
OUT.mkdir()
sys.dont_write_bytecode = True
_uname = os.uname()
platform.uname = lambda: platform.uname_result(*(tuple(_uname)+(_uname.machine,)))
sys.path[:0] = [str(R), str(W/'src/alicia_grasp_modes/src')]
denied = []
def boundary(event, args):
    bad = event.startswith(('socket.', 'subprocess.')) or event in ('os.system','os.exec','os.posix_spawn','os.fork')
    if event == 'open' and args and isinstance(args[0], (str, bytes, os.PathLike)):
        path = Path(os.fsdecode(args[0])).resolve()
        flags = args[2] if len(args)>2 and isinstance(args[2], int) else 0
        mode = args[1] if len(args)>1 and isinstance(args[1], str) else ''
        if flags & (os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC|os.O_APPEND) or any(c in mode for c in 'wax+'):
            bad = OUT != path and OUT not in path.parents
        if str(path).startswith('/dev/') and str(path) not in ('/dev/null','/dev/urandom','/dev/random'):
            bad = True
    if bad:
        denied.append(dict(event=event,args=repr(args)[:200]))
        raise RuntimeError('review boundary denied '+event)
sys.addaudithook(boundary)

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
def load(name):
    spec = importlib.util.spec_from_file_location(name, R/(name+'.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

FILES = ('prepare_gen55_runtime_helpers_20261019.py', 'gen55_release_validation_body.py',
    'gen55_idle_reload_common_20261019.py', 'read_gen55_deployment_preflight_20261019.py',
    'load_gen55_idle_compute_gateway_20261019.py', 'finalize_gen55_idle_deployment_20261019.py',
    'gen55_observation_measurement_consumers_20261019.launch',
    'start_gen55_observation_measurement_consumers_20261019.sh',
    'install_and_seal_gen55_20261019.py')
if (R/'read_gen55_preinstall_readiness_20261019.py').exists():
    FILES += ('read_gen55_preinstall_readiness_20261019.py',)
dependencies = {str(R/n):sha(R/n) for n in FILES}
common = load('gen55_idle_reload_common_20261019')
finalizer = load('finalize_gen55_idle_deployment_20261019')
installer = load('install_and_seal_gen55_20261019')
runner = load('verify_gen55_parallel_offline_20261019')
checks = []
def check(name, function, accept):
    try:
        function()
    except (RuntimeError, AssertionError, ValueError, KeyError, TypeError) as exc:
        observed = False; reason = repr(exc)
    else:
        observed = True; reason = ''
    checks.append(dict(name=name,expected_accept=accept,accepted=observed,
                       passed=observed==accept,reason=reason))

for n in FILES:
    if n.endswith('.py'):
        ast.parse((R/n).read_text())
old_tree=ast.parse((R/'gen54_idle_reload_common_20261018.py').read_text())
new_tree=ast.parse((R/'gen55_idle_reload_common_20261019.py').read_text())
inherited={'sha','read','require','inside','storage','write','Transport','process','exited','feedback','validate_sources'}
old_nodes={n.name:ast.dump(n,include_attributes=False)for n in old_tree.body if isinstance(n,(ast.FunctionDef,ast.ClassDef))and n.name in inherited}
new_nodes={n.name:ast.dump(n,include_attributes=False)for n in new_tree.body if isinstance(n,(ast.FunctionDef,ast.ClassDef))and n.name in inherited}
assert old_nodes==new_nodes and set(new_nodes)==inherited
assert common.TARGETS == ('/grasp_task_node','/remote_grasp6d_node','/unknown_tabletop_perception','/motion_gateway')
assert len(common.PRESERVE) == 7 and not set(common.TARGETS)&set(common.PRESERVE)
assert all(sha(p)==h for p,h in finalizer.HELPER_SHA.items())
xml = ET.parse(R/'gen55_observation_measurement_consumers_20261019.launch').getroot()
assert {n.get('name') for n in xml.findall('node')} == {n.lstrip('/') for n in common.TARGETS}
assert xml.find("arg[@name='include_gateway']").get('default') == 'false'
assert xml.find("node[@name='motion_gateway']").get('if') == '$(arg include_gateway)'
shell = (R/'start_gen55_observation_measurement_consumers_20261019.sh').read_text()
assert 'include_gateway:=false' in shell and 'OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1' in shell
check('actual_control_active58_hash_bound', lambda: common.validate_control(common.CONTROL), True)
check('actual_previous_GEN54_bound', lambda: common.validate_previous(common.PREVIOUS), True)
candidate_path = (Path(sys.argv[2]).resolve(strict=True) if len(sys.argv)>2 else
                  V/'gen55_final_collected_r1/candidate_manifest.json')
candidate = json.loads(candidate_path.read_bytes())
actual_manifest, actual_context, _ = common.candidate_context(candidate_path,sha(candidate_path),
    R/'verify_gen55_parallel_offline_20261019.py',sha(R/'verify_gen55_parallel_offline_20261019.py'))
assert len(actual_context['sources'])==1082 and len(actual_context['scope'])>=335 and len(actual_context['case_identities'])>=9561
dependencies[str(candidate_path)]=sha(candidate_path)
dependencies[str(R/'verify_gen55_parallel_offline_20261019.py')]=sha(R/'verify_gen55_parallel_offline_20261019.py')
check('actual_candidate_disk_delta_reviewed', lambda: installer.check_delta(candidate), True)
for mutation in ('extra_change','missing_baseline','missing_new_module'):
    trial=deepcopy(candidate)
    if mutation=='extra_change':
        old=next(p for p in json.loads(installer.BASE_RELEASE.read_bytes())['source_sha256']
                 if p not in installer.PRODUCTION|{installer.OLD_TEST})
        trial['source_sha256'][old]='0'*64
    if mutation=='missing_baseline':trial['source_sha256'].pop(next(iter(installer.PRODUCTION)))
    if mutation=='missing_new_module':trial['source_sha256'].pop(installer.NEW_MODULE)
    check('disk_delta_rejects_'+mutation,lambda:installer.check_delta(trial),False)
previous, baseline = common.validate_previous(common.PREVIOUS)
selection = dict(mode='unknown',strategy='observe_return',generation=3,stamp_ns='1792379927063770294')
preflight = dict(node_identities=baseline,gateway_original_parent=dict(pid=90000,parent_pid=90001,start_ticks='1'))
after = {n:dict(pid=91000+i,parent_pid=92000,start_ticks='2') for i,n in enumerate(common.TARGETS)}
after['/motion_gateway']['parent_pid']=90000
preserved = {n:baseline[n] for n in common.PRESERVE}
compute = dict(before={n:baseline[n] for n in common.TARGETS},after=after,
    preserved_before=preserved,preserved_after=deepcopy(preserved),replaced_count=4,preserved_count=7,
    gateway_respawn_parent=preflight['gateway_original_parent'],gateway_after_respawn=after['/motion_gateway'],
    gateway_started_by_compute_launch=False,gateway_original_parent_preserved=True,
    gateway_service_owners={n:['/motion_gateway'] for n in common.GATEWAY_SERVICES})
check('synthetic_four_replaced_seven_preserved_chain', lambda: finalizer.deployment_chain(previous,preflight,compute), True)
for mutation in ('preserved_pid','same_target','wrong_gateway_parent','second_gateway','service_owner'):
    trial=deepcopy(compute)
    if mutation=='preserved_pid':trial['preserved_after'][common.PRESERVE[0]]['pid']+=1
    if mutation=='same_target':trial['after'][common.TARGETS[0]]=deepcopy(baseline[common.TARGETS[0]])
    if mutation=='wrong_gateway_parent':trial['after']['/motion_gateway']['parent_pid']+=1
    if mutation=='second_gateway':trial['gateway_started_by_compute_launch']=True
    if mutation=='service_owner':trial['gateway_service_owners'][common.GATEWAY_SERVICES[0]]=['/other']
    check('chain_rejects_'+mutation,lambda:finalizer.deployment_chain(previous,preflight,trial),False)
ack=dict(selection,ready=True)
ack_row=dict(success=True,ready=True,raw_selection=ack,selection=finalizer.parse_selection(ack),message=json.dumps(ack))
ack_compute=dict(reset_ack={s:deepcopy(ack_row)for s in ('/grasp_mode/reset_remote','/grasp_mode/reset_task')})
check('synthetic_bound_reset_ack',lambda:finalizer.validate_ack(ack_compute,selection),True)
for key,value in (('generation',2),('stamp_ns','1792374537080906391')):
    trial=deepcopy(ack_compute);wrong=dict(ack,**{key:value})
    trial['reset_ack']['/grasp_mode/reset_task'].update(message=json.dumps(wrong),raw_selection=wrong,selection=finalizer.parse_selection(wrong))
    check('reset_rejects_'+key,lambda:finalizer.validate_ack(trial,selection),False)

# Unit-test the actual result gate with a minimal, explicit synthetic context.
# No real candidate is admitted, installed, sealed or published by this test.
source='src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observation_path_guard.py'
source_sha=sha(W/source)
scope=['synthetic_test.py']; identities=[[scope[0],'synthetic','test_one']]
context=dict(scope=scope,sources={source:source_sha},case_identities=identities,
    required_changed_sources=[source],required_imports_by_test_file={scope[0]:[source]},
    generated_imports={},compatibility_dependencies={})
guard_source,audit_source=runner.bootstrap_sources(runner.load_serial())
context.update(guard_sha256=hashlib.sha256(guard_source.encode()).hexdigest(),
               audit_sha256=hashlib.sha256(audit_source.encode()).hexdigest())
manifest=dict(focus_scope=scope,workspace_root=str(W))
synthetic_manifest=OUT/'synthetic_manifest.json';save(synthetic_manifest,manifest)
runner_path=R/'verify_gen55_parallel_offline_20261019.py'
original_context=common.candidate_context
common.candidate_context=lambda *args:(manifest,context,runner)
old_text=(V/'gen55_runtime_helpers_independent_review_r2/original_common_before_provenance_fix.py').read_text()
assert hashlib.sha256(old_text.encode()).hexdigest()=='b3415b1eb176f573370f36d26bdecea104388e5e9de79be3b632c783bab35d5b'
(OUT/'original_common_before_provenance_fix.py').write_text(old_text)
old_function=next(n for n in ast.parse(old_text).body if isinstance(n,ast.FunctionDef)and n.name=='validate_result')
old_env=dict(vars(common))
exec(compile(ast.Module(body=[old_function],type_ignores=[]),str(OUT/'original_common_before_provenance_fix.py'),'exec'),old_env)
counterexamples=[]
def evidence_case(name, mutation=None):
    folder=OUT/name;folder.mkdir();worker=folder/'worker_0';worker.mkdir()
    summary=dict(tests=1,failures=0,errors=0,skipped=0)
    xml=b'<testsuites><testsuite tests="1" failures="0" errors="0" skipped="0"><testcase file="synthetic_test.py" classname="synthetic" name="test_one" time="0.1"/></testsuite></testsuites>'
    for p in (folder/'pytest.xml',worker/'pytest.xml'):p.write_bytes(xml)
    (worker/'pytest.log').write_text('Explicit synthetic check. No pytest body ran.\n')
    (worker/'bootstrap').mkdir()
    (worker/'bootstrap/sitecustomize.py').write_text(guard_source)
    (worker/'bootstrap/gen42_import_audit.py').write_text(audit_source)
    if mutation=='bootstrap_identity_mismatch':
        (worker/'bootstrap/sitecustomize.py').write_text('# deliberately missing guard\n')
    provenance={'alicia_flexible_grasp.robot.observation_path_guard':dict(path=str(W/source),sha256=source_sha)}
    if mutation=='missing_provenance':provenance={}
    if mutation=='foreign_provenance':provenance={'unexpected':dict(path='/foreign/module.py',sha256='0'*64)}
    save(worker/'module_provenance.json',provenance)
    shard=dict(scope=scope,case_identities=identities,required_changed_sources=[source])
    plan=dict(context=context,expected_case_identities=identities,test_scope=scope,mode='full',runner_sha256=sha(runner_path),shards=[shard],required_changed_sources=[source],worker_count=1)
    save(folder/'plan.json',plan)
    row=dict(worker_index=0,passed=True,scope_complete=True,source_unchanged=True,direct_child_reaped=True,test_bodies_executed=True,
        returncode=0,interrupted=False,direct_child_cleanup_signals=[],external_boundary_attempts=[],foreign_or_inconsistent_imports={},
        plan_sha256=sha(folder/'plan.json'),test_scope=scope,mode='full',loaded_changed_sources={source:True},
        required_changed_sources=[source],generated_bootstrap_sha256={str(p.relative_to(worker)):sha(p)for p in(worker/'bootstrap').iterdir()},
        summary=summary,case_identities=identities,xml_sha256=sha(worker/'pytest.xml'),log_sha256=sha(worker/'pytest.log'),
        module_provenance_sha256=sha(worker/'module_provenance.json'))
    if mutation=='row_interrupted':row['interrupted']=True
    if mutation=='bootstrap_empty':row['generated_bootstrap_sha256']={}
    save(worker/'result.json',row)
    result=dict(mode='full',passed=True,scope_complete=True,source_unchanged=True,test_bodies_executed=True,combined_xml_matches_worker_results=True,
        candidate_manifest_sha256=sha(synthetic_manifest),candidate_manifest_path=str(synthetic_manifest),runner_sha256=sha(runner_path),source_sha256=context['sources'],
        interrupted=False,launcher_error=None,offline_child_cleanup_required=False,residual_offline_process_groups=[],worker_count=1,worker_returncodes=[0],
        external_boundary_attempts=[],foreign_or_inconsistent_imports={},duplicate_case_identities=[],missing_case_identities=[],unexpected_case_identities=[],missing_worker_results=[],
        xml_sha256=sha(folder/'pytest.xml'),summary=summary,test_scope_files=1,source_count=1,plan_sha256=sha(folder/'plan.json'),
        worker_result_sha256={str(worker/'result.json'):sha(worker/'result.json')},full_regression_scope=True,full_regression_passed=True,loaded_changed_sources_global={source:True})
    if mutation=='empty_global_loaded':result['loaded_changed_sources_global']={}
    if mutation=='interrupted':result['interrupted']=True
    if mutation=='nonzero_worker':result['worker_returncodes']=[1]
    if mutation=='missing_worker':result['worker_result_sha256']={}
    save(folder/'result.json',result)
    if mutation=='artifact_drift':(worker/'pytest.log').write_text('changed after bound digest\n')
    if mutation=='boundary_append':(worker/'boundary_attempts.jsonl').write_text('{"event":"forbidden"}\n')
    return lambda:common.validate_result(folder/'result.json',synthetic_manifest,sha(synthetic_manifest),runner_path,sha(runner_path),'full')
check('synthetic_complete_result_consistency',evidence_case('positive'),True)
for mutation in ('interrupted','row_interrupted','nonzero_worker','missing_worker','artifact_drift','boundary_append','missing_provenance','foreign_provenance','empty_global_loaded','bootstrap_empty','bootstrap_identity_mismatch'):
    function=evidence_case(mutation,mutation)
    check('result_rejects_'+mutation,function,False)
    if mutation in ('missing_provenance','foreign_provenance','empty_global_loaded'):
        path=OUT/mutation/'result.json'
        old_env['validate_result'](path,synthetic_manifest,sha(synthetic_manifest),runner_path,sha(runner_path),'full')
        counterexamples.append(dict(mutation=mutation,original_gate_incorrectly_accepted=True,
            corrected_gate_rejected=checks[-1]['passed'],synthetic_evidence=str(path)))
common.candidate_context=original_context
actual_focus=None
if len(sys.argv)>3:
    focus_path=Path(sys.argv[3]).resolve(strict=True)
    _,_,focus,focus_evidence=common.validate_result(focus_path,candidate_path,sha(candidate_path),runner_path,sha(runner_path),'focus')
    actual_focus=dict(path=str(focus_path),sha256=sha(focus_path),summary=focus['summary'],
        worker_count=focus['worker_count'],passed=focus['passed'],full_regression=False,
        bodies_reexecuted_by_review=False)
    dependencies.update(focus_evidence)
assert all(sha(p)==h for p,h in dependencies.items()),'helper changed during review'
result=dict(kind='GEN55_runtime_helpers_independent_offline_review',passed=all(c['passed']for c in checks),
    dependency_sha256=dependencies,checks=checks,failed_checks=[c for c in checks if not c['passed']],boundary_attempts=denied,
    original_helper_sha256=sha(OUT/'original_common_before_provenance_fix.py'),historical_counterexamples=counterexamples,
    original_common_inherited_functions_AST_unchanged=sorted(inherited),
    actual_frozen_manifest_validated=dict(path=str(candidate_path),sha256=sha(candidate_path),
        sources=len(actual_context['sources']),test_files=len(actual_context['scope']),
        case_identities=len(actual_context['case_identities']),test_bodies_executed=False),
    actual_focus_validated=actual_focus,
    synthetic_result_gate_context=True,synthetic_cases_are_not_real_regression_evidence=True,
    live_ROS_contacted=False,hardware_contacted=False,motion_commands_sent=False,temperature_queried=False,
    runtime_deployed=False,physical_grasp_verified=False)
save(OUT/'result.json',result)
print(json.dumps(dict(passed=result['passed'],checks=len(checks),failures=result['failed_checks'],result=str(OUT/'result.json'),sha256=sha(OUT/'result.json')),ensure_ascii=False,indent=2))
