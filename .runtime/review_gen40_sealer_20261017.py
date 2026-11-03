"""Read-only sealer source review with inert extracted admission predicates."""
import ast
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import time

W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
D=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen36_restart_20261017T012037Z')
O=D/'verification/gen40_sealer_helper_review_r1'
assert Path('/media/zhuyupei/SanStand').is_mount()
if O.exists():
    assert not list(O.iterdir()), 'refuse to overwrite existing review artifacts'
    O.rmdir()
assert not O.exists()
O.mkdir();inputs={}
def read(path):
    path=Path(path);raw=path.read_bytes();digest=hashlib.sha256(raw).hexdigest()
    assert inputs.setdefault(str(path),digest)==digest
    return raw
def load(path):return json.loads(read(path))
def write(name,value):
    raw=(json.dumps(value,ensure_ascii=False,indent=2,sort_keys=True,allow_nan=False)+'\n').encode()
    path=O/name
    with path.open('xb') as f:f.write(raw)
    assert path.read_bytes()==raw
    return hashlib.sha256(raw).hexdigest()

path=W/'.runtime/seal_gen40_release_20261017.py'
text=read(path).decode();tree=ast.parse(text)
definitions=[n for n in tree.body if isinstance(n,ast.Assign)]
ns={};exec(compile(ast.fix_missing_locations(ast.Module(body=definitions,type_ignores=[])),str(path),'exec'),ns)
P=ns['P']
assert len(ns['CHANGED'])==5 and len(ns['MODIFIED_TESTS'])==6 and len(ns['NEW_TESTS'])==4 and not ns['MODULES']
checklist=load(D/'verification/gen40_release_sealer_review_checklist_r1/checklist.json')
assert ns['CHANGED']|ns['MODIFIED_TESTS']==set(checklist['expected_changed_original_sources'])
assert ns['NEW_TESTS']==set(checklist['expected_new_test_files'])
baseline=load(D/'verification/gen39_release_manifest.json')
assert inputs[str(D/'verification/gen39_release_manifest.json')]=='47a2f5148d585dfa0d938868453e8293fffdefc35290f1b788bc013703257030'
assert len(baseline['source_sha256'])==1025 and len(baseline['test_scope'])==275

main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
assertions=[n for n in main.body if isinstance(n,ast.Assert)]
def predicate(fragment):
    matches=[n for n in assertions if fragment in ast.get_source_segment(text,n)]
    assert len(matches)==1,(fragment,len(matches))
    return compile(ast.Expression(matches[0].test),str(path),'eval')

fragments=[
    "all(combined[k] for k in ('passed'",
    "combined['returncode'] == 0",
    "not combined['external_boundary_attempts']",
    "combined['live_ros_contacted'] is False",
    'new_tests == NEW_TESTS',
    'set(scope) == set(baseline',
    'set(sources) == set(baseline',
    'len(sources) == combined',
    'len(scope) == combined',
    'changed_baseline == CHANGED',
    "summary['tests'] > baseline",
    "all(summary[k] == 0",
    'set(counts) == set(scope)',
    'all(counts[n] >= c',
    "review['reviewed'] is True",
    "not review['blocking_findings']",
    'CHANGED | MODIFIED_TESTS | NEW_TESTS | MODULES <= set(reviewed)',
]
compiled={f:predicate(f) for f in fragments}
def context():
    source=dict(baseline['source_sha256'])
    for n in ns['CHANGED']|ns['MODIFIED_TESTS']:source[n]='modified_inert_case'
    source.update({n:'new_inert_case' for n in ns['NEW_TESTS']})
    scope=list(baseline['test_scope'])+sorted(ns['NEW_TESTS'])
    counts=dict(baseline['test_case_counts_by_file']);counts.update({n:1 for n in ns['NEW_TESTS']})
    summary=dict(tests=sum(counts.values()),failures=0,errors=0,skipped=0)
    combined=dict(passed=True,source_unchanged=True,scope_complete=True,
        baseline_file_test_counts_retained=True,returncode=0,regression_scope='full_combined',
        external_boundary_attempts=[],live_ros_contacted=False,motion_commands_sent=False,
        source_count=1029,test_scope_files=279)
    reviewed={n:'inert' for n in ns['CHANGED']|ns['MODIFIED_TESTS']|ns['NEW_TESTS']}
    return dict(ns,baseline=baseline,new_tests=set(ns['NEW_TESTS']),sources=source,scope=scope,
        changed_baseline=ns['CHANGED']|ns['MODIFIED_TESTS'],counts=counts,summary=summary,
        combined=combined,review=dict(reviewed=True,release_review_complete=True,blocking_findings=[]),reviewed=reviewed)

cases=[]
def case(name,fragment,mutate=None,reject=False):
    ctx=context()
    if mutate:mutate(ctx)
    result=bool(eval(compiled[fragment],ctx))
    assert result is (not reject),name
    cases.append(dict(name=name,passed=True,expected_rejection=reject))
case('exact_expected_source_test_admission',fragments[9])
case('failed_full_result',fragments[0],lambda c:c['combined'].update(passed=False),True)
case('source_changed_during_full',fragments[0],lambda c:c['combined'].update(source_unchanged=False),True)
case('focused_result_cannot_seal',fragments[1],lambda c:c['combined'].update(regression_scope='focused'),True)
case('nonzero_full_exit',fragments[1],lambda c:c['combined'].update(returncode=1),True)
case('external_boundary_attempt',fragments[2],lambda c:c['combined'].update(external_boundary_attempts=['inert attempted boundary']),True)
case('live_ros_flag',fragments[3],lambda c:c['combined'].update(live_ros_contacted=True),True)
case('unexpected_new_test',fragments[4],lambda c:c['new_tests'].add('src/unapproved.py'),True)
case('old_test_removed_from_scope',fragments[5],lambda c:c['scope'].pop(0),True)
case('missing_new_source',fragments[6],lambda c:c['sources'].pop(next(iter(ns['NEW_TESTS']))),True)
case('wrong_source_total',fragments[7],lambda c:c['combined'].update(source_count=1028),True)
case('wrong_scope_total',fragments[8],lambda c:c['combined'].update(test_scope_files=278),True)
case('unauthorized_original_source_change',fragments[9],lambda c:c['changed_baseline'].add('src/unapproved.py'),True)
case('declared_old_change_missing',fragments[9],lambda c:c['changed_baseline'].remove(next(iter(ns['MODIFIED_TESTS']))),True)
case('nonzero_failure_count',fragments[11],lambda c:c['summary'].update(failures=1),True)
case('old_case_count_reduction',fragments[13],lambda c:c['counts'].update({baseline['test_scope'][0]:0}),True)
case('incomplete_independent_review',fragments[14],lambda c:c['review'].update(release_review_complete=False),True)
case('blocking_review_finding',fragments[15],lambda c:c['review'].update(blocking_findings=['inert']),True)
case('unreviewed_modified_policy_test',fragments[16],lambda c:c['reviewed'].pop(next(iter(ns['MODIFIED_TESTS']))),True)
case('unreviewed_new_test',fragments[16],lambda c:c['reviewed'].pop(next(iter(ns['NEW_TESTS']))),True)
assert len(cases)==20

for fragment in [
    "assert actual == summary", "assert xml_counts == counts and all(xml_counts.values())",
    "assert not combined['unassigned_testcases']", "assert declared['full'] is True and declared['tests'] == scope",
    "assert declared['source_sha256'] == sources", "assert all(sha(W/n) == d for n, d in sources.items())",
    "assert review['evidence_sha256'].get(str(a.combined.resolve())) == sha(a.combined)",
    "runtime_deployment_pending=True", "consumer verifies frozen plan and complete inferred family, not independent all-points verification",
]:assert fragment in text,fragment
helpers={}
for name in ['verify_gen40_offline_tests_20261017.py','load_gen40_idle_compute_20261017.py',
        'read_gen40_deployment_preflight_20261017.py','finalize_gen40_idle_deployment_20261017.py']:
    p=W/'.runtime'/name;read(p);helpers[str(p.relative_to(W))]=inputs[str(p)]
helpers[str(path.relative_to(W))]=inputs[str(path)]
for p in [D/'verification/gen40_offline_compute_helpers_review_r1/result.json',
        D/'verification/gen40_preflight_finalizer_helpers_review_r1/result.json']:
    r=load(p);assert r['reviewed'] and not r['blocking_findings']
for p,h in inputs.items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==h,p
r=dict(schema_version=1,kind='gen40_sealer_independent_static_review',recorded_ns=time.time_ns(),reviewed=True,
    helpers_review_complete=True,release_review_complete=False,blocking_findings=[],
    reviewed_source_sha256=helpers,evidence_sha256=inputs,pure_ast_predicate_cases=cases,
    pure_ast_case_count=len(cases),sealer_main_executed=False,full_regression_claimed=False,
    ros_contacted=False,hardware_contacted=False,source_or_helper_modified=False,
    approved_expected_sets=dict(changed_production=sorted(ns['CHANGED']),modified_original_tests=sorted(ns['MODIFIED_TESTS']),new_tests=sorted(ns['NEW_TESTS']),new_modules=[]),
    expected_counts=dict(baseline_cases=7962,baseline_sources=1025,baseline_test_files=275,full_sources=1029,full_test_files=279),
    static_checks=['GEN39 exact baseline/backup admission; authorized old changes and exactly four new test files.',
        'Actual full result, XML totals and every-file counts, all old counts, zero F/E/skip and boundary attempts.',
        'Full declared scope/source map and all current workspace bytes bound; source drift refuses seal.',
        'Independent review must be actual complete/no blocking and cover every changed/new source; immutable combined result is explicitly bound.',
        'Sealed release states runtime deployment pending and physical grasp false; producer-vs-consumer enclosure proof boundary is explicit.'],
    operational_requirements=['Final independent whole-source review must include all five opshelper hashes; sealer itself explicitly binds runner/self and binds the remaining helpers through reviewed_source/evidence provided by that review.',
        'Actual complete GEN40 full regression and final source SHA review are still required; these inert predicates are not a substitute.',
        'Keep r1 fixture failures immutable and only admit final corrected-source results; no dynamic progress/doc/live-log evidence inputs.'])
h=write('result.json',r)
s=write('seal.json',dict(schema_version=1,review_completed=True,release_review_complete=False,
    artifacts_sha256={'result.json':h},helper_sha256=helpers,script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),blocking_findings=[]))
print(json.dumps(dict(result=str(O/'result.json'),result_sha256=h,seal_sha256=s,helper_sha256=inputs[str(path)],pure_ast_cases=len(cases))))
