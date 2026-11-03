import ast,hashlib,json,time
from pathlib import Path
import xml.etree.ElementTree as E
W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade');C=W/'.runtime/gen16_height_recipe_policy_test_20261013'
V=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z/verification')
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
p=C/'delivery_manifest.json';assert sha(p)=='e9da9d2fa1107a6dc2279b1728f560fd2cb977c86368ac1ea08045836eaf170d';m=json.loads(p.read_text())
assert m['test_only'] and not m['production_modified'] and len(m['files'])==1
row=m['files'][0];path=row['relative_path'];assert '/tests/test_' in path
assert sha(C/path)==row['sha256'] and sha(W/path)==row['baseline_sha256']
evidence={str(p):sha(p)}
for k in ('verification','patch','failure'):
 x=m[k];assert sha(x['path'])==x['sha256'];evidence[x['path']]=x['sha256']
rp=Path(m['verification']['path']);r=json.loads(rp.read_text());assert r['passed'] and r['returncode']==0
assert r['summary']==dict(tests=60,failures=0,errors=0,skipped=0)
assert r['actual_sources_before']==r['actual_sources_after'];assert len(r['actual_sources_after'])==915
assert all(sha(W/q)==h for q,h in r['actual_sources_after'].items())
assert r['candidate']['before_sha256']==r['candidate']['after_sha256']==row['sha256']
assert r['loaded']['test']==str(C/path)
assert r['loaded']['base_remote']==str(W/'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py')
assert r['loaded']['mode_mixin']==str(W/'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_remote.py')
for f,k in [('pytest.log','log_sha256'),('pytest.xml','xml_sha256')]:
 assert sha(rp.parent/f)==r[k];evidence[str(rp.parent/f)]=r[k]
suites=list(E.parse(rp.parent/'pytest.xml').getroot().iter('testsuite'))
assert {k:sum(int(s.get(k,0)) for s in suites) for k in ('tests','failures','errors','skipped')}==r['summary']
old='test_observe_return_override_forwards_options_only_after_original_yield_policy'
new={'test_observe_return_override_keeps_height_search_and_forwards_options','test_observe_return_override_runs_real_height_search_and_fresh_publication_check'}
a=ast.parse((W/path).read_text());b=ast.parse((C/path).read_text());a.body=[x for x in a.body if not(isinstance(x,ast.FunctionDef) and x.name==old)];b.body=[x for x in b.body if not(isinstance(x,ast.FunctionDef) and x.name in new)]
assert ast.dump(a,include_attributes=False)==ast.dump(b,include_attributes=False)
prior=V/'gen16_success_first_integrated_install/result.json';installed=json.loads(prior.read_text());assert installed['passed'] and len(installed['test_scope'])==176
assert installed['source_sha256']==r['actual_sources_after'];evidence[str(prior)]=sha(prior)
assert path in installed['test_scope']
full=Path(m['failure']['path']);failed=json.loads(full.read_text());assert failed['summary']==dict(tests=5636,failures=5,errors=0,skipped=0)
cases=[x.get('name') for s in E.parse(full.parent/'pytest.xml').getroot().iter('testsuite') for x in s.findall('testcase') if x.find('failure') is not None]
assert set(m['failure']['cases']).issubset(cases)
evidence[str(full.parent/'pytest.xml')]=sha(full.parent/'pytest.xml')
review=dict(passed=True,kind='GEN16_HEIGHT_RECIPE_POLICY_TEST_INDEPENDENT_REVIEW',recorded_ns=time.time_ns(),delivery_manifest=str(p),delivery_sha256=sha(p),test_scope=installed['test_scope'],focused_test_summary=r['summary'],evidence_sha256=evidence,test_only=True,production_modified=False,actual_source_count=915,actual_sources_unchanged=True,unchanged_other_tests_AST=True,blocking_findings=[],findings=[
 'The four original failures require a measured-view opportunity to defer height alternatives, contradicting the explicitly reviewed GEN16 success-first scheduling policy.',
 'All eight option/opportunity combinations now require exactly one parent call, unchanged 123-second deadline and original option objects; no deferred audit or family entry can be synthesized.',
 'Two new cases run the actual mode/base height method chain through initial failure, real height proposal selection and another fresh strict invocation for publication rebuild.',
 'All other module AST is unchanged. Binding mutation, geometry/strict rejection, pre/post deadline, service adapter and cancellation assertions remain.',
 'Only one test file is delivered; the frozen 915 actual-source hashes remain unchanged. The separate worker-slot observation issue is not covered or declared fixed by this package.'
 ],limitations=[
 'This verifies software scheduling intent and test orchestration. Fixture registered-geometry gate, necessary-goal rejection and stream-current check are mocked boundaries, in addition to synthetic strict replies.',
 'The height proposal/margin and mode/base orchestration run real code, but passing these fixtures does not prove real geometry clearance, collision freedom or physical grasp success.',
 'Existing 60-case evidence was reviewed, not rerun. Actual integrated regression must follow the separately reviewed test changes.'
 ],motion_commands_sent=False,live_ros_contacted=False,temperature_queried=False,physical_grasp_verified=False)
out=W/'.runtime/gen16_height_recipe_policy_test_independent_review.pending.json';out.write_text(json.dumps(review,ensure_ascii=False,sort_keys=True,indent=2)+'\n');print(json.dumps(dict(path=str(out),sha256=sha(out),passed=True)))
