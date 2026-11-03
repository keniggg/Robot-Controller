"""Review exact GEN41 offline sources and evidence; approve only a closed full pass."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
D=E/'ros_log/full_latest_gen36_restart_20261017T012037Z'
V=D/'verification'
BASE=V/'gen40_release_manifest.json'
BASE_SHA='7d5e8042e8dc4c5735109b6e8d73dc0ab78436f720d51eed29f886f97a2f1c63'
P='src/alicia_grasp_modes/'
TASK=P+'src/alicia_grasp_modes/station_compute_wait_task.py'
PROD={
 P+'src/alicia_grasp_modes/station_compute_wait_remote.py':'7159dff1880305264585e14000e8a395f0f4cef23dc4ee5dcd4bb5c3158358d7',
 P+'src/alicia_grasp_modes/station_compute_reacquisition.py':'078ca934c3db77adbc608ccf785e617e6118b2c2e749d8f139d370488cf5107c',
 P+'src/alicia_grasp_modes/station_contact_admission.py':'eccc235deb14ab7a2c6e49b5a24435d1ab17af05c419c03e36a5dfa9afd6cdbc',
 TASK:'3db5266e2c4c35d05adc3906baceb2cae4830947ed26432b702a3a6001c27749'}
OLD_TASK_SHA='4ceb578b1ed20166ead19682e6875ced196d96255cf3c6f1c9a0905b7f23d904'
NEW_TESTS={P+'tests/test_gen41_station_private_pending.py':62,
 P+'tests/test_gen41_task_availability_wait.py':37,
 P+'tests/test_gen41_pending_handoff_chain.py':11}
KNOWN_TEST_SHA={P+'tests/test_gen41_station_private_pending.py':'4629f842c058d1dcf395880daec523cd4bfeb2ac266c94a5a4da0b28980c077e',
 P+'tests/test_gen41_task_availability_wait.py':'a73d4873b4713a2338b9ea063877208aa2b3714d8a106c6aef52a196d999da07'}
HELPERS=['.runtime/'+n for n in ('verify_gen41_offline_tests_20261017.py',
 'seal_gen41_release_20261017.py','load_gen41_idle_compute_20261017.py',
 'read_gen41_deployment_preflight_20261017.py','finalize_gen41_idle_deployment_20261017.py')]
EXPECTED=dict(tests=8153,failures=0,errors=0,skipped=0)
PROOFS={
 'gen40_actual_attempt1_pending_handoff_r1':'d22f7d1cd42d132e827f642427854e7273ce50346c812dc3a2b70c3d681b3029',
 'gen40_actual_attempt1_source_replay_r1':'5d6547ef7f8496ac0d5d9c0dbc34172927dc92f8745b6a9166f80434553fa45d',
 'gen41_before_edit_r1':'fd44510e13d49731845f720bfba5d64daab454b6c308a467f9386ccc105c76a9',
 'gen41_remote_candidate_independent_review_r1':'050f656ead35fdfd7ed336dc20d8434aeb7bd37ecc2ff8e31ae03f76fe4e4420',
 'gen41_task_candidate_independent_review_r2':'ff76398768c65a788b0e17f098b44b8b579d9713e9f862e8fcfba330486d4e28',
 'gen41_four_production_independent_review_r1':'d12b278a3cf7641ffea5c0a568c2dbb74cb816ad6fd1daf4defa39b6a89f336d',
 'gen41_four_production_initialization_review_r2':'797c74175d64668921a9c9d81587a5a945b402399b6b5e042659353214fbe57c',
 'gen41_before_recovery_fixture_addendum_r1':'b1688458fba7864ef5a501bd92045cd300c5269edadba7e66ad1a43b04ce79c0',
 'gen41_recovery_proof_contract_addendum_r1':'a2bf3c4b440208547ee80a854e56229d4f5a9421d199849a229efd6ec2591e98',
 'gen41_operations_helpers_independent_review_r1':'d728f0075844a1839325cbf769fb9cbde84aa6109abf9e397aed29c346a19120',
 'gen41_sealer_independent_review_r1':'7558f824fb858df985e9e6c818ef34c4190d0a97476e554bdb09d9a499529144'}


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
 ap=argparse.ArgumentParser(description=__doc__)
 ap.add_argument('--final',action='store_true')
 ap.add_argument('--source-copy-manifest',type=Path,required=True)
 ap.add_argument('--combined',type=Path,default=V/'gen41_combined_r1')
 ap.add_argument('--output',type=Path,default=V/'gen41_combined_review_r1')
 ap.add_argument('--chain-focus',type=Path,default=V/'gen41_pending_handoff_chain_focus_r3')
 ap.add_argument('--chain-fixture-fix',type=Path,default=V/'gen41_sdk_transport_fixture_patch_r3')
 ap.add_argument('--chain-fixture-review',type=Path,default=V/'gen41_sdk_transport_fixture_independent_review_r3')
 args=ap.parse_args()
 assert Path('/media/zhuyupei/SanStand').is_mount()
 for p in (args.output,args.combined,args.chain_focus,args.chain_fixture_fix,args.chain_fixture_review):
  p.resolve().relative_to(V.resolve())
 args.source_copy_manifest.resolve().relative_to((W/'.runtime').resolve())
 args.output.mkdir(exist_ok=True)
 inputs={};pending=[]
 def read(path,expected=None):
  path=Path(path).resolve(strict=True);raw=path.read_bytes();h=hashlib.sha256(raw).hexdigest()
  assert inputs.setdefault(str(path),h)==h,path
  assert expected is None or expected==h,path
  return raw
 def load(path,expected=None):return json.loads(read(path,expected))
 def write(name,value):
  path=args.output/name;raw=(json.dumps(value,ensure_ascii=False,indent=2,sort_keys=True,allow_nan=False)+'\n').encode()
  with path.open('xb') as f:f.write(raw)
  assert path.read_bytes()==raw
  return hashlib.sha256(raw).hexdigest()
 def relative(n):
  p=Path(n)
  return str(p.relative_to(W)) if p.is_absolute() else n
 def match_review(value,allow_old_task=False):
  for n,h in value['reviewed_source_sha256'].items():
   n=relative(n)
   expected=OLD_TASK_SHA if allow_old_task and n==TASK else sources.get(n)
   assert h==expected,(n,h,expected)

 baseline=load(BASE,BASE_SHA)
 assert baseline['source_count']==1029 and baseline['test_scope_files']==279
 assert baseline['test_summary']==dict(tests=8043,failures=0,errors=0,skipped=0)
 backup=W/'.runtime/gen40_baseline_source';backup_meta=load(backup/'manifest.json')
 assert backup_meta['source_sha256']==baseline['source_sha256']
 assert all(sha(backup/n)==h for n,h in baseline['source_sha256'].items())
 copy_meta=load(args.source_copy_manifest);copy=args.source_copy_manifest.resolve().parent
 assert copy_meta['baseline_release_sha256']==BASE_SHA and copy_meta['source_count']==1032
 assert copy_meta['regression_pending'] is True and copy_meta.get('runtime_deployed',False) is False
 assert copy_meta.get('physical_grasp_verified',False) is False
 sources=copy_meta['source_sha256']
 assert len(sources)==1032 and set(sources)==set(baseline['source_sha256'])|set(NEW_TESTS)
 assert {n for n,h in baseline['source_sha256'].items() if sources[n]!=h}==set(PROD)
 assert all(sources[n]==h for n,h in PROD.items())
 assert all(sources[n]==h for n,h in KNOWN_TEST_SHA.items())
 assert all(sources[n]==baseline['source_sha256'][n] for n in baseline['test_scope'])
 def source_check():
  for n,h in sources.items():assert sha(W/n)==sha(copy/n)==h,n
 source_check()
 found={str(p.relative_to(W)) for root in (W/'src/alicia_grasp_modes/tests',W/'src/alicia_flexible_grasp_supervisor/tests') for p in root.glob('test_gen41*.py')}
 assert found==set(NEW_TESTS)
 reviewed={n:sources[n] for n in sorted(set(PROD)|set(NEW_TESTS))}
 for n in HELPERS:
  read(W/n);reviewed[n]=inputs[str((W/n).resolve())]
 expected_scope=set(baseline['test_scope'])|set(NEW_TESTS)
 scope=load(args.combined/'scope.json')
 assert scope['full'] is True and len(scope['tests'])==282 and len(set(scope['tests']))==282
 assert set(scope['tests'])==expected_scope and scope['source_sha256']==sources
 assert scope['baseline']==str(BASE) and scope['baseline_sha256']==BASE_SHA

 def xml_check(directory,r):
  root=ET.fromstring(read(directory/'pytest.xml',r['xml_sha256']))
  summary={k:sum(int(s.get(k,0)) for s in root.iter('testsuite')) for k in ('tests','failures','errors','skipped')}
  assert summary==r['summary']
  counts=dict.fromkeys(r['test_scope'],0)
  assert len(counts)==r['test_scope_files']
  actual_status=dict(failures=0,errors=0,skipped=0)
  for case in root.iter('testcase'):
   assert case.get('file') in counts,case.get('file')
   counts[case.get('file')]+=1
   for tag,key in (('failure','failures'),('error','errors'),('skipped','skipped')):
    actual_status[key]+=len(case.findall(tag))
  assert counts==r['test_case_counts_by_file'] and sum(counts.values())==summary['tests']
  assert all(counts.values()) and actual_status=={k:summary[k] for k in actual_status}
  read(directory/'pytest.log',r['log_sha256']);declared=load(directory/'scope.json')
  assert declared['tests']==r['test_scope'] and declared['source_sha256']==r['source_sha256']
  assert declared['baseline_sha256']==BASE_SHA
  read(directory/'bootstrap/sitecustomize.py',r['bootstrap_sha256'])
  assert r['source_count']==len(r['source_sha256'])
  assert not r['external_boundary_attempts'] and not r['changed_during_run']
  assert r['source_unchanged'] and r['scope_complete'] and not r['unassigned_testcases']
  assert r['baseline_file_test_counts_retained']
  assert r['live_ros_contacted'] is False and r['motion_commands_sent'] is False
  assert r['physical_grasp_verified'] is False and r['temperature_queried'] is False
  assert r['baseline']==str(BASE) and r['baseline_sha256']==BASE_SHA
  assert '-k' not in r['command'] and not any(a.startswith('--deselect') for a in r['command'])
  if (directory/'seal.json').exists():load(directory/'seal.json')
  return counts
 def focus(directory,summary,*,pass_expected,old_task=False):
  r=load(directory/'result.json');assert r['summary']==summary and r['passed'] is pass_expected
  assert r['returncode']==(0 if pass_expected else 1)
  xml_check(directory,r)
  if pass_expected:
   for n,h in PROD.items():assert r['source_sha256'][n]==(OLD_TASK_SHA if old_task and n==TASK else h),n
   for n in set(r['test_scope'])&set(NEW_TESTS):
    assert r['source_sha256'][n]==sources[n] and r['test_case_counts_by_file'][n]==NEW_TESTS[n]
  return r
 focus_results={}
 for name,total,old_task in [('gen41_remote_private_wait_focus_r1',148,True),('gen41_task_availability_focus_r2',124,False)]:
  focus_results[name]=focus(V/name,dict(tests=total,failures=0,errors=0,skipped=0),pass_expected=True,old_task=old_task)['summary']
 if (args.chain_focus/'result.json').exists():
  focus_results[args.chain_focus.name]=focus(args.chain_focus,dict(tests=11,failures=0,errors=0,skipped=0),pass_expected=True)['summary']
 else:pending.append('closed passing current 11-case cross-consumer chain focus')

 proofs={}
 for name,h in PROOFS.items():
  proofs[name]=load(V/name/'result.json',h)
  if (V/name/'seal.json').exists():load(V/name/'seal.json')
 a=proofs['gen41_remote_candidate_independent_review_r1'];assert a['complete'] and not a['blocking_findings'];match_review(a)
 b=proofs['gen41_task_candidate_independent_review_r2'];assert b['review_complete_for_bound_candidate_hashes'] and not b['blocking_findings'];match_review(b)
 c=proofs['gen41_four_production_independent_review_r1'];assert c['review_complete_for_bound_hashes'] and not c['blocking_findings'];match_review(c,True)
 c2=proofs['gen41_four_production_initialization_review_r2'];assert c2['review_complete_for_bound_hashes'] and not c2['blocking_findings'] and c2['no_gate_or_threshold_changes'];match_review(c2)
 # Independently repeat the C protected-original-gate AST comparison.
 def functions(path):
  out={}
  def walk(rows,prefix=''):
   for n in rows:
    if isinstance(n,(ast.FunctionDef,ast.AsyncFunctionDef)):out[prefix+n.name]=ast.dump(n,include_attributes=False)
    elif isinstance(n,ast.ClassDef):walk(n.body,prefix+n.name+'.')
  walk(ast.parse(Path(path).read_text()).body);return out
 for n,names in c['protected_original_gate_AST_identical'].items():
  old,new=functions(backup/n),functions(W/n)
  assert all(old[k]==new[k] for k in names),(n,names)
 # The only post-review production fix is exact early-initialization cleanup.
 initdir=V/'gen41_task_initialization_patch_r2';init=load(initdir/'result.json','3730fe191b4705ef7cfbcc4535a61ae2807b0854c37d0c9b85fe0816da067094')
 load(initdir/'seal.json');read(initdir/'r2_changes.diff')
 old=read(initdir/'station_compute_wait_task_r1.py',OLD_TASK_SHA).decode();new=read(initdir/'station_compute_wait_task_r2.py',PROD[TASK]).decode()
 anchor='    def _clear_observe_return_attempt(self):\n        self._station_supplement_proposal = None\n        self._station_task_availability_pending.clear()\n        self._station_task_availability_verified.clear()\n'
 replacement=anchor.replace('self._station_task_availability_pending.clear()','self._station_task_availability_pending = {}').replace('self._station_task_availability_verified.clear()','self._station_task_availability_verified = {}')
 assert old.count(anchor)==1 and old.replace(anchor,replacement)==new
 assert init['source_unchanged_except_authorized_two_lines'] and init['new_test_files_unchanged'] and not init['modified_old_test_files']
 ops=proofs['gen41_operations_helpers_independent_review_r1'];assert not ops['blocking_findings']
 for row in ops['helpers']:assert sha(W/row['path'])==row['sha256']
 sealer=proofs['gen41_sealer_independent_review_r1'];assert sealer['reviewed'] and not sealer['blocking_findings']
 assert set(sealer['reviewed_source_sha256'])==set(HELPERS)
 for n,h in sealer['reviewed_source_sha256'].items():assert reviewed[n]==h
 # Final C test is allowed to evolve only through explicitly preserved new SDK-fixture repairs.
 if (args.chain_fixture_fix/'result.json').exists() and (args.chain_fixture_review/'result.json').exists():
  fix=load(args.chain_fixture_fix/'result.json');load(args.chain_fixture_fix/'seal.json');read(args.chain_fixture_fix/'exact.diff')
  assert fix['test_path']==P+'tests/test_gen41_pending_handoff_chain.py' and fix['after_sha256']==sources[fix['test_path']]
  assert fix['assertions_case_count_unchanged']==11 and fix['original_gate_and_old_helpers_unchanged']
  assert fix['unchanged_gap_span_ns']==280072042 and fix['unchanged_original_max_span_ns']==200000000
  fr=load(args.chain_fixture_review/'result.json');load(args.chain_fixture_review/'seal.json')
  assert fr['complete'] and not fr['blocking_findings'];match_review(fr)
  assert fr['case_count_preserved']==11 and fr['all_original_279_test_files_byte_unchanged']
  assert fr['all_original_new_test_parametrization_unchanged']
  read(args.chain_fixture_review/'reviewed_test.py',sources[fix['test_path']])
  read(args.chain_fixture_review/'reviewed.diff',fr['reviewed_diff_sha256'])
  prior=load(fr['prior_failure_cause_report'],fr['prior_failure_cause_report_sha256'])
  assert prior['complete'] is True and prior['production_or_tests_edited'] is False
  for n,h in prior['unchanged_gen40_helper_and_gate_sha256'].items():assert sha(W/n)==h,n
  if Path(fr['prior_failure_cause_report']).with_name('seal.json').exists():load(Path(fr['prior_failure_cause_report']).with_name('seal.json'))
 else:pending.append('closed SDK-fixture repair and independent bound-new-test review')

 red={}
 for name,summary in [('gen41_task_availability_focus_r1',dict(tests=124,failures=0,errors=108,skipped=0)),
  ('gen41_pending_handoff_chain_focus_r1',dict(tests=11,failures=11,errors=0,skipped=0)),
  ('gen41_pending_handoff_chain_focus_r2',dict(tests=11,failures=4,errors=0,skipped=0))]:
  red[name]=focus(V/name,summary,pass_expected=False)['summary']
 # Healthy old-GEN40 repro: actual geometry relay and full recovery SDK still fail at pending.
 before=focus(V/'gen41_pending_handoff_public_relay_before_r2',dict(tests=1,failures=1,errors=0,skipped=0),pass_expected=False)
 assert all(before['source_sha256'][n]==h for n,h in baseline['source_sha256'].items())
 read(V/'gen41_pending_handoff_public_relay_before_r2/interpretation.json')
 assert proofs['gen41_before_recovery_fixture_addendum_r1']['original_files_unchanged']
 assert proofs['gen41_recovery_proof_contract_addendum_r1']['case_count']==11
 # Bind immutable original explanations. Do not pin live progress/control, unclosed logs or docs.
 for p in (V/'gen41_pending_handoff_chain_focus_r2/interpretation.json',V/'gen41_pending_handoff_before_focus_r1/result.json'):
  read(p)
 read(__file__);reviewed[str(Path(__file__).resolve().relative_to(W))]=sha(__file__)
 source_check()
 for path,h in inputs.items():assert sha(path)==h,path
 prepared=dict(schema_version=1,kind='gen41_combined_review_preparation',recorded_ns=time.time_ns(),
  reviewed=False,release_review_complete=False,full_regression_approved=False,
  reviewed_source_sha256=reviewed,evidence_sha256=dict(inputs),pending=pending,
  source_count=1032,full_test_files=282,expected_summary=EXPECTED,new_test_counts=NEW_TESTS,
  exact_four_authorized_production_sources=PROD,all_old_279_test_files_byte_unchanged=True,
  focused_evidence=focus_results,retained_first_round_failures=red)
 if not args.final:
  h=write('prepared.json',prepared);print(json.dumps(dict(prepared=True,release_review_complete=False,pending=pending,sha256=h)));return
 assert not pending,pending
 # No approval exists unless the full runner has actually closed with every test passing.
 result_path=args.combined/'result.json';assert result_path.exists(),'full result not closed; review remains pending'
 r=load(result_path);assert r['passed'] is True and r['returncode']==0 and r['regression_scope']=='full_combined'
 assert r['summary']==EXPECTED and r['source_count']==1032 and r['source_sha256']==sources
 assert r['test_scope_files']==282 and r['test_scope']==scope['tests'] and set(r['test_scope'])==expected_scope
 assert r['baseline_file_test_counts_retained'];counts=xml_check(args.combined,r)
 assert set(counts)==expected_scope and all(counts[n]==v for n,v in baseline['test_case_counts_by_file'].items())
 assert all(counts[n]==v for n,v in NEW_TESTS.items()) and sum(counts.values())==8153
 assert r['runner_sha256']==reviewed[HELPERS[0]]
 source_check()
 for path,h in inputs.items():assert sha(path)==h,path
 review=dict(schema_version=1,kind='gen41_final_independent_combined_source_review',recorded_ns=time.time_ns(),
  reviewed=True,release_review_complete=True,full_regression_approved=True,blocking_findings=[],
  reviewed_source_sha256=reviewed,evidence_sha256=dict(inputs),source_count=1032,full_summary=r['summary'],
  full_test_files=282,full_regression=str(result_path.resolve()),source_and_candidate_copy_match_full=True,
  all_original_279_file_counts_exact=True,all_original_279_tests_byte_unchanged=True,
  authorized_changed_production=sorted(PROD),modified_existing_tests=[],new_test_counts=NEW_TESTS,
  focused_evidence=focus_results,retained_first_round_failures=red,
  fixed_findings=[
   'Station availability and irrevocable metric/input changes use separate causal paths across Remote and Task; complete owner-bound evidence retains private unpublished work only.',
   'Actual feedback gap remains clock_only=false and unqualified. Partial raw-input markers only quarantine, never explain a bracket, register READY, renew timestamps/deadlines, undo a hard cutoff, or authorize motion.',
   'A complete rejected-bracket pending or READY recovery proof is sufficient without obsolete failed-topic arrival. The current READY still passes original real three-topic, source/owner, SDK, support, geometry and deadline gates.',
   'Actual identity, pose/support, original input contradiction, control revocation and expiration still retire work. Original 200 ms and 90 second limits and source timestamps remain unchanged.',
   'Task first construction exposed 108 real initialization errors and 11 chain failures missed by static review; only two early clear calls became empty dict assignments. Failed sources and results remain immutable.',
   'C chain r2 7P/4F arose after admission/first contact in the new synthetic SDK transport. Only the new provider ordering/duplicate transport fixture changed; old helpers, original stationary gates and negative assertions remain unchanged.',
   'Early new-hook swallowed-exception and recovery-fixture missing consumer bracket runs are retained with their stated limits; phase0 from the latter is not asserted as G40 historical cause.'],
  actual_record_boundary='Closed G40 truly accepted measured-schema2 contact precheck but did not publish/admit/execute it. The 280.072 ms new frame is unqualified; the following recovery and pure clock case justify nonmetric wait repair, not a new physical permission.',
  limitations=['Full offline method replay uses explicit simulated SDK/FK/IK/services and never proves current physical target, live path or actual grip.',
   'Earlier 18/54 memory components were not cross-consumer or physical success proofs; the obsolete unconditional wait for old failed topics is superseded by proof sufficiency.',
   'Reviewer owns Remote/shared/schema; independent A and C reviews cover those exact production hashes. First-construction static-review miss is explicitly retained.',
   'Python socket/device audit covers Python boundaries; native SDK factories still require explicit inert fixtures.'],
  runtime_deployment_pending=True,deployed=False,physical_grasp_verified=False,motion_authorized=False,
  live_ros_contacted=False,hardware_contacted=False,production_or_tests_edited_during_final_review=False)
 h=write('review.json',review)
 s=write('seal.json',dict(schema_version=1,review_completed=True,release_review_complete=True,
  artifacts_sha256={'review.json':h},reviewed_source_sha256=reviewed,full_summary=r['summary'],
  source_count=1032,blocking_findings=[],runtime_deployment_pending=True,motion_authorized=False))
 print(json.dumps(dict(review=str(args.output/'review.json'),review_sha256=h,seal_sha256=s,blocking_findings=[])))


if __name__=='__main__':main()
