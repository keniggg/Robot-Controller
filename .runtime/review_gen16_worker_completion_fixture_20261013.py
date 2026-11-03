import ast,hashlib,json,time
from pathlib import Path
import xml.etree.ElementTree as E
W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade');C=W/'.runtime/gen16_worker_completion_fixture_20261013'
V=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z/verification');sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
p=C/'delivery_manifest.json';expected='b48e6fa952abcdb269eb69e01831816f982851095a94dac9580f39ebfdb185c6';assert sha(p)==expected;m=json.loads(p.read_text());assert m['passed'] and len(m['files'])==1 and m['production_files_changed']==0
f=m['files'][0];path=f['path'];assert sha(W/path)==f['base_sha256'] and sha(C/path)==f['candidate_sha256']
evidence={str(p):expected}
for k in ('prior_actual_result','proof'):
 q=m[k];assert sha(q['path'])==q['sha256'];evidence[q['path']]=q['sha256']
patch=C/'changes.patch'
if not patch.exists():patch=C/'delivery.patch'
assert sha(patch)==m['patch_sha256'];evidence[str(patch)]=sha(patch)
rp=Path(m['proof']['path']);r=json.loads(rp.read_text());assert r['passed'] and r['baseline_expected_race_reproduced'] and r['source_unchanged'] and r['actual_unchanged'] and not r['production_changed']
assert r['baseline']['summary']==dict(tests=1,errors=0,failures=1,skipped=0) and r['baseline']['returncode']==1
assert r['candidate']['summary']==dict(tests=21,errors=0,failures=0,skipped=0) and r['candidate']['returncode']==0
for name,h in r['candidate_sha256'].items():assert sha(name)==h;evidence[name]=h
runner=W/'.runtime/run_gen16_worker_completion_fixture_r2_20261013.py';assert sha(runner)==r['runner_sha256'];evidence[str(runner)]=sha(runner)
for key,dirname in [('baseline','baseline_deterministic_race'),('candidate','candidate')]:
 d=r[key];folder=rp.parent/dirname
 assert sha(d['helper_path'])==d['helper_sha256']
 for fn,hkey in [('pytest.log','log_sha256'),('pytest.xml','xml_sha256')]:assert sha(folder/fn)==d[hkey];evidence[str(folder/fn)]=d[hkey]
 suites=list(E.parse(folder/'pytest.xml').getroot().iter('testsuite'));assert {k:sum(int(s.get(k,0)) for s in suites) for k in ('tests','failures','errors','skipped')}==d['summary']
 if key=='baseline':assert 'helper reported completion before locked slot retirement' in (folder/'pytest.log').read_text()
a=ast.parse((W/path).read_text());b=ast.parse((C/path).read_text())
origtests={x.name:x for x in a.body if isinstance(x,ast.FunctionDef) and x.name.startswith('test_')};newtests={x.name:x for x in b.body if isinstance(x,ast.FunctionDef) and x.name.startswith('test_')}
assert len(newtests)==len(origtests)+2
assert all(ast.dump(x,include_attributes=False)==ast.dump(newtests[k],include_attributes=False) for k,x in origtests.items())
a.body=[x for x in a.body if not(isinstance(x,ast.FunctionDef) and x.name=='complete_worker')]
b.body=[x for x in b.body if not(isinstance(x,ast.FunctionDef) and (x.name=='complete_worker' or x.name in set(newtests)-set(origtests)))]
assert ast.dump(a,include_attributes=False)==ast.dump(b,include_attributes=False)
prior=V/'gen16_success_first_integrated_install/result.json';install=json.loads(prior.read_text());assert len(install['source_sha256'])==915
assert all(sha(W/q)==h for q,h in install['source_sha256'].items());assert m['test_scope']==install['test_scope'] and len(m['test_scope'])==176;evidence[str(prior)]=sha(prior)
result=dict(passed=True,kind='GEN16_WORKER_COMPLETION_FIXTURE_INDEPENDENT_REVIEW',recorded_ns=time.time_ns(),delivery_manifest=str(p),delivery_sha256=expected,test_scope=m['test_scope'],evidence_sha256=evidence,test_only=True,production_modified=False,actual_source_count=915,actual_sources_unchanged=True,original_test_functions_AST_unchanged=len(origtests),original_parametrized_case_count=19,focused_test_summary=r['candidate']['summary'],baseline_expected_failure_summary=r['baseline']['summary'],blocking_findings=[],findings=[
 'Production worker clears busy then runs slot retirement inside one _stream_condition critical section, after metrics publication. The old helper reads busy and metrics without that lock and can return midway; production _poll_stream_snapshot reads both busy and slot generation under the same lock.',
 'The new helper observes busy/metrics while holding the production condition. It does not poll for marker=None or modify production state, so it observes transaction completion without manufacturing cleanup.',
 'The controlled Event test pauses the actual worker inside its release hook while holding the real lock. Original helper fails deterministically for returning before retirement; corrected helper waits, then observes real cleanup and unchanged source watermark.',
 'The missing-cleanup negative control runs the original successful-path test with only the release hook deliberately suppressed. It still fails at the exact unchanged marker-is-None assertion; a genuine slot leak is not hidden.',
 'All three original test functions and their assertions are AST-identical, covering 19 parameterized cases: original successful fresh-source retry, four non-reopen cases, and fourteen original owner/scope restrictions.',
 'No production code is delivered. Original full-run five-failure receipt is retained; this package addresses only test synchronization and does not claim the historical thread schedule was recorded.'
 ],limitations=['Read-only independent review of existing one-failure baseline and 21-pass candidate evidence; no repeated tests or ROS.', 'A deterministic reachable interleaving explains the observed state, but raw historical thread interleaving was not recorded.', 'Fixture worker transport is offline; no physical grasp success or full combined regression is claimed.'],live_ros_contacted=False,motion_commands_sent=False,temperature_queried=False,physical_grasp_verified=False)
out=W/'.runtime/gen16_worker_completion_fixture_independent_review.pending.json';out.write_text(json.dumps(result,ensure_ascii=False,sort_keys=True,indent=2)+'\n');print(json.dumps(dict(path=str(out),sha256=sha(out),passed=True)))
