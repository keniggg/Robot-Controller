"""Inert AST receipt checks; never import ROS or execute deployment main."""
import ast
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import time

W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
D=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen36_restart_20261017T012037Z')
O=D/'verification/gen40_preflight_finalizer_helpers_review_r1'
assert Path('/media/zhuyupei/SanStand').is_mount() and not O.exists()
O.mkdir()
inputs={}


def read(path):
    path=Path(path);raw=path.read_bytes();h=hashlib.sha256(raw).hexdigest()
    assert inputs.setdefault(str(path),h)==h
    return raw


def write(name,value):
    b=(json.dumps(value,ensure_ascii=False,indent=2,sort_keys=True,allow_nan=False)+'\n').encode()
    p=O/name
    with p.open('xb') as f:f.write(b)
    assert p.read_bytes()==b
    return hashlib.sha256(b).hexdigest()


pre=W/'.runtime/read_gen40_deployment_preflight_20261017.py'
fin=W/'.runtime/finalize_gen40_idle_deployment_20261017.py'
old_pre=W/'.runtime/read_gen39_deployment_preflight_20261016.py'
old_fin=W/'.runtime/finalize_gen39_idle_deployment_20261016.py'
pt,ft=read(pre).decode(),read(fin).decode()
opt,oft=read(old_pre).decode(),read(old_fin).decode()
assert inputs[str(pre)]=='7fe2afdd5da45467e9e1b6298a12c04fd73d815033454f1d180b00131082ebb9'
assert inputs[str(fin)]=='c1bd131aea5678755d76552e4e3f61caf3c7aca068b65e64d248d76458ad9376'
def normalize(old):
    return (old.replace('GEN39','__UPPER_NEW__').replace('gen39','__lower_new__')
        .replace('GEN38','GEN39').replace('gen38','gen39')
        .replace('__UPPER_NEW__','GEN40').replace('__lower_new__','gen40')
        .replace('verify_gen40_offline_tests_20261016','verify_gen40_offline_tests_20261017'))
assert pt==normalize(opt)
assert ft==normalize(oft)
prior_review_path=D/'verification/gen39_release_helpers_review_r1/result.json'
prior=json.loads(read(prior_review_path))
assert prior['reviewed'] and not prior['blocking_findings']
assert prior['reviewed_source_sha256'][str(old_pre.relative_to(W))]==inputs[str(old_pre)]
assert prior['reviewed_source_sha256'][str(old_fin.relative_to(W))]==inputs[str(old_fin)]
verify=W/'.runtime/verify_gen40_offline_tests_20261017.py'
vt=read(verify).decode()
assert "BASE = C/'verification/gen39_release_manifest.json'" in vt
assert "BASE_SHA = '47a2f5148d585dfa0d938868453e8293fffdefc35290f1b788bc013703257030'" in vt
assert "output.resolve().relative_to((C/'verification').resolve())" in vt

tree=ast.parse(ft)
names={'COMPUTE','TARGETS','PRESERVED','ALL_NODES'}
functions={'require','merge','identities_valid','deployment_chain','joint_map'}
pure=[n for n in tree.body if (isinstance(n,ast.Assign)
    and any(isinstance(t,ast.Name) and t.id in names for t in n.targets))
    or (isinstance(n,ast.FunctionDef) and n.name in functions)]
ns={'math':math}
exec(compile(ast.fix_missing_locations(ast.Module(body=pure,type_ignores=[])),str(fin),'exec'),ns)
all_nodes=ns['ALL_NODES'];targets=ns['TARGETS'];compute=ns['COMPUTE'];preserved=ns['PRESERVED']
baseline={n:dict(pid=100+i,parent_pid=20,start_ticks=str(10000+i))
    for i,n in enumerate(sorted(all_nodes))}


def fixture():
    previous=dict(after={n:deepcopy(baseline[n]) for n in targets},
        preserved_after={n:deepcopy(baseline[n]) for n in preserved})
    preflight=dict(node_identities=deepcopy(baseline))
    changed={n:dict(pid=baseline[n]['pid']+1000,parent_pid=30,
        start_ticks=str(int(baseline[n]['start_ticks'])+1000)) for n in compute}
    comp=dict(before={n:deepcopy(baseline[n]) for n in compute},after=changed,
        preserved_before={n:deepcopy(baseline[n]) for n in all_nodes-compute})
    comp['preserved_after']=deepcopy(comp['preserved_before'])
    intermediate={**comp['after'],**comp['preserved_after']}
    gateway=dict(before=deepcopy(baseline['/motion_gateway']),
        after=dict(pid=5000,parent_pid=20,start_ticks='20000'),
        preserved_before={n:deepcopy(v) for n,v in intermediate.items() if n!='/motion_gateway'})
    gateway['preserved_after']=deepcopy(gateway['preserved_before'])
    return previous,preflight,comp,gateway


checks=[]
def case(name,mutation=None,reject=False):
    rows=fixture()
    if mutation:mutation(*rows)
    failed=False
    try:
        output=ns['deployment_chain'](*rows)
        assert len(output[1])==4 and len(output[2])==7 and len(output[3])==11
    except ValueError:failed=True
    assert failed is reject,name
    checks.append(dict(name=name,passed=True,expected_rejection=reject))

case('exact_current_11_process_chain')
case('stale_preflight_identity',lambda p,f,c,g:f['node_identities']['/grasp_task_node'].update(pid=9999),True)
case('compute_before_not_preflight',lambda p,f,c,g:c['before']['/grasp_task_node'].update(pid=9999),True)
case('preserved_compute_process_changed',lambda p,f,c,g:c['preserved_after']['/alicia_d_driver_node'].update(pid=9999),True)
case('gateway_before_not_intermediate',lambda p,f,c,g:g['before'].update(pid=9999),True)
case('gateway_preserved_process_changed',lambda p,f,c,g:g['preserved_after']['/remote_grasp6d_node'].update(pid=9999),True)
def no_task_reload(p,f,c,g):
    c['after']['/grasp_task_node']=deepcopy(baseline['/grasp_task_node'])
    for k in ('preserved_before','preserved_after'):
        g[k]['/grasp_task_node']=deepcopy(baseline['/grasp_task_node'])
case('one_compute_consumer_not_replaced',no_task_reload,True)
case('gateway_not_replaced',lambda p,f,c,g:g.update(after=deepcopy(baseline['/motion_gateway'])),True)
case('invalid_new_start_ticks',lambda p,f,c,g:c['after']['/remote_grasp6d_node'].update(start_ticks='not_ticks'),True)
assert ns['joint_map'](['a','b'],[1.,2.])=={'a':1.,'b':2.}
checks.append(dict(name='finite_named_joint_map',passed=True))
for name,names_in,positions in [('duplicate_names',['a','a'],[1.,2.]),
        ('nonfinite_joint',['a'],[float('nan')]),('boolean_joint',['a'],[True])]:
    try:ns['joint_map'](names_in,positions)
    except ValueError:checks.append(dict(name=name,passed=True,expected_rejection=True))
    else:raise AssertionError(name)
assert len(checks)==13

# Publication order and pinning are unchanged from the reviewed prior helper.
main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
atomic_calls=[n for n in ast.walk(main) if isinstance(n,ast.Call)
    and isinstance(n.func,ast.Name) and n.func.id=='atomic_json']
atomic_calls.sort(key=lambda n:n.lineno)
assert len(atomic_calls)==6
last=atomic_calls[-1]
assert isinstance(last.args[0],ast.Name) and last.args[0].id=='pointer_path'
assert "control_state=control['state'], control_revision=control['revision']" in ft
assert "require(preflight['control_sha256'] == input_sha[str(control_path)]" in ft
assert "preflight['node_identities'] == baseline" in ft
assert "new deployment must not overwrite the GEN39 session" in ft
assert "'gen40_runtime_deployment.json'" in ft
assert "snapshot_semantics='immutable_deployment_time_snapshot; authoritative dynamic control file unchanged'" in ft
assert "fcntl.LOCK_SH | fcntl.LOCK_NB" in ft
for path,digest in inputs.items():assert hashlib.sha256(Path(path).read_bytes()).hexdigest()==digest,path
review=dict(schema_version=1,kind='gen40_preflight_finalizer_independent_review',
    recorded_ns=time.time_ns(),reviewed=True,helpers_review_complete=True,release_review_complete=False,
    blocking_findings=[],reviewed_source_sha256={str(p.relative_to(W)):inputs[str(p)] for p in (pre,fin,verify)},
    evidence_sha256=inputs,exact_generation_only_replacement=True,
    baseline_gen39_release_sha256='47a2f5148d585dfa0d938868453e8293fffdefc35290f1b788bc013703257030',
    preflight_output_guard='SanStand session verification directory only',
    previous_deployment_identity_baseline='The previous accepted GEN39 11-process identity map; current epoch/selection/control are independently sampled and pinned, not copied from old deployment posture.',
    control_policy='Preserve actual preflight state/revision and bytes, do not write or hardcode a pause/revision.',
    finalizer_chain='3 compute consumers followed by original-core gateway; four replaced plus seven preserved, all eleven live identities rechecked.',
    final_publication='Original GEN39 deployment/release immutable; new GEN40 runtime/status/progress snapshots written atomically; terminal marker precedes last current-pointer publication under shared nonblocking control metadata lock.',
    pure_ast_case_count=len(checks),pure_ast_checks=checks,source_inputs_unchanged=True,
    helper_main_executed=False,ros_contacted=False,hardware_contacted=False,process_mutated=False,
    motion_commands_sent=False,temperature_queried=False,
    limitations=['Inert receipt tests and static review do not prove later live deployment success.',
        'This review does not cover an as-yet-unwritten GEN40 release sealer or any complete-regression result.',
        'The GEN40 source/evidence release must be actually complete and approved before running these helpers.'])
h=write('result.json',review)
s=write('seal.json',dict(schema_version=1,review_completed=True,release_review_complete=False,
    artifacts_sha256={'result.json':h},helper_sha256=review['reviewed_source_sha256'],
    script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),blocking_findings=[]))
print(json.dumps(dict(result=str(O/'result.json'),result_sha256=h,seal_sha256=s,pure_ast_cases=len(checks))))
