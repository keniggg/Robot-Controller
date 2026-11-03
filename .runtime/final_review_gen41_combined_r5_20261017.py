"""Review the final GEN41 causal handoff sources and closed offline results.

No ROS, hardware, deployment or motion. --final is allowed only after current
Remote150/Task288/chain14 and the full8168 have actually closed successfully.
"""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
V = E/'ros_log/full_latest_gen36_restart_20261017T012037Z/verification'
BASE = V/'gen40_release_manifest.json'
BASE_SHA = '7d5e8042e8dc4c5735109b6e8d73dc0ab78436f720d51eed29f886f97a2f1c63'
P = 'src/alicia_grasp_modes/'
REMOTE = P+'src/alicia_grasp_modes/station_compute_wait_remote.py'
SHARED = P+'src/alicia_grasp_modes/station_compute_reacquisition.py'
SCHEMA = P+'src/alicia_grasp_modes/station_contact_admission.py'
TASK = P+'src/alicia_grasp_modes/station_compute_wait_task.py'
RTEST = P+'tests/test_gen41_station_private_pending.py'
TTEST = P+'tests/test_gen41_task_availability_wait.py'
CTEST = P+'tests/test_gen41_pending_handoff_chain.py'
PROD = {
 REMOTE: 'eb0c6abc8eb3e0c1852a775989028f065131f9b7c46ba7c87061586a91118bab',
 SHARED: 'd1ff4e79bcc56ad3486f88c34e8647f61ef3caf44d807b67569830975ddaa60f',
 SCHEMA: 'eccc235deb14ab7a2c6e49b5a24435d1ab17af05c419c03e36a5dfa9afd6cdbc',
 TASK: '2766b6cba7ed56f7505934db27a5148f226a76b0aaf2655ace8cef3e39f9692d'}
NEW_TESTS = {RTEST: 64, TTEST: 47, CTEST: 14}
TEST_SHA = {
 RTEST: '78828cad86e6b29879473ebb95df827f16634f654c3727f22f6d558d8391e862',
 TTEST: '8e62cb974ed087ce6a6b204d1c21a2d99ef2b7e7021ca259dfec15f948addc93',
 CTEST: '3d1c91558ffa6333cd7e1417fc5f84225209b698748b4d8c2b92f8c4b95dc2ab'}
OLD_C13_SHA = 'e792fd3fdaa04c0f66f6cfc8bb6c52aa4b6942207476665d54c6bb72d93156e6'
EXPECTED = dict(tests=8168, failures=0, errors=0, skipped=0)
OLD_TASK = '4ceb578b1ed20166ead19682e6875ced196d96255cf3c6f1c9a0905b7f23d904'
INIT_TASK = '3db5266e2c4c35d05adc3906baceb2cae4830947ed26432b702a3a6001c27749'
COMPAT_TASK = 'f0f66371cfd8410af4a5bf870a83173e41d845ca077db3b159c953657a3a660a'
SUPPORT_TASK = '9ef5adf26263f5a301c83954a076517a33f1e7882046a9c27bc94350db4001e5'
TEST45 = 'b3bcd65da38effbd586799060ad0184f01f3ba8e928ddc0fd61603998e734fdf'
OLD_COPY = W/'.runtime/gen41_r2_source'
OLD_COPY_SHA = 'd8af0c7a2205cea55115b7b0c295c54365accc485a37ba5dfa348caa09a56031'
FINAL_COPY_SHA = 'd46f690d5188af5c639be8daab3c87d31cf43e95dc17769d3d6b7d43c5b9cde2'
HELPERS = ['.runtime/'+n for n in (
 'verify_gen41_offline_tests_20261017.py', 'seal_gen41_release_20261017.py',
 'load_gen41_idle_compute_20261017.py', 'read_gen41_deployment_preflight_20261017.py',
 'finalize_gen41_idle_deployment_20261017.py')]
PROOFS = {
 'gen40_actual_attempt1_pending_handoff_r1': 'd22f7d1cd42d132e827f642427854e7273ce50346c812dc3a2b70c3d681b3029',
 'gen40_actual_attempt1_source_replay_r1': '5d6547ef7f8496ac0d5d9c0dbc34172927dc92f8745b6a9166f80434553fa45d',
 'gen41_before_edit_r1': 'fd44510e13d49731845f720bfba5d64daab454b6c308a467f9386ccc105c76a9',
 'gen41_remote_candidate_independent_review_r1': '050f656ead35fdfd7ed336dc20d8434aeb7bd37ecc2ff8e31ae03f76fe4e4420',
 'gen41_task_candidate_independent_review_r2': 'ff76398768c65a788b0e17f098b44b8b579d9713e9f862e8fcfba330486d4e28',
 'gen41_four_production_independent_review_r1': 'd12b278a3cf7641ffea5c0a568c2dbb74cb816ad6fd1daf4defa39b6a89f336d',
 'gen41_four_production_initialization_review_r2': '797c74175d64668921a9c9d81587a5a945b402399b6b5e042659353214fbe57c',
 'gen41_before_recovery_fixture_addendum_r1': 'b1688458fba7864ef5a501bd92045cd300c5269edadba7e66ad1a43b04ce79c0',
 'gen41_recovery_proof_contract_addendum_r1': 'a2bf3c4b440208547ee80a854e56229d4f5a9421d199849a229efd6ec2591e98',
 'gen41_operations_helpers_independent_review_r1': 'd728f0075844a1839325cbf769fb9cbde84aa6109abf9e397aed29c346a19120',
 'gen41_sealer_independent_review_r1': '7558f824fb858df985e9e6c818ef34c4190d0a97476e554bdb09d9a499529144',
 'gen41_feedback_late_remote_shared_independent_review_r2': '2e2eb3a4d6f9166a05894f4e2b8fc69ba66590b6b63f302def4101093031e8a3',
 'gen41_feedback_idle_task_independent_review_r2': '5d100be958ebe3b3b419dcf5c63762148c8bfd06f201aab131ac0fc0d3f70613',
 'gen41_feedback_idle_chain_increment_review_r3': 'ad3ac6292349161737b08d033703900730f3ba9576fa7d017214156b2484545a',
 'gen41_feedback_idle_remote_candidate_r2': 'fa1e6550f687d5990652a76274739327e4f7ca742959bbef090c28d88406dd97',
 'gen41_support_idle_delta_independent_review_r1': 'ab542a8e155d76a12ca60632c06887c7344d65cd1e2c3febc505c6ec5c9189fc',
 'gen41_plan_freshness_chain_fixture_independent_review_r5': '398078671eebda3eb1d1408833a0b8d65178b1c8699cbb5bc1de5cf50003a640',
 'gen41_plan_freshness_chain_independent_review_r5': '96791adc15d7fef95230e4937c974b4d58c76b1502b0f3f795660b0451198932',
 'gen41_plan_freshness_chain_fixture_r5': 'fc43e1e33a71f92f3af20b81f955fa105c9767e6bbdc6fd70c08b089c9f69619'}


def sha(path):
 return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def functions(path):
 out = {}
 def walk(rows, prefix=''):
  for n in rows:
   if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)):
    out[prefix+n.name] = ast.dump(n, include_attributes=False)
   elif isinstance(n, ast.ClassDef):
    walk(n.body, prefix+n.name+'.')
 walk(ast.parse(Path(path).read_text()).body)
 return out


def main():
 ap = argparse.ArgumentParser(description=__doc__)
 ap.add_argument('--final', action='store_true')
 ap.add_argument('--source-copy-manifest', required=True, type=Path)
 ap.add_argument('--combined', type=Path, default=V/'gen41_combined_r2')
 ap.add_argument('--output', type=Path, default=V/'gen41_combined_review_r5')
 ap.add_argument('--remote-focus', type=Path, default=V/'gen41_remote_private_wait_focus_r2')
 ap.add_argument('--compat-focus', type=Path, default=V/'gen41_legacy_pending_compat_focus_r3')
 ap.add_argument('--chain-focus', type=Path, default=V/'gen41_pending_handoff_chain_focus_r6')
 ap.add_argument('--echo-before', type=Path, default=V/'gen41_feedback_idle_echo_chain_before_r3')
 ap.add_argument('--bound-review', type=Path, action='append', default=[],
  help='Additional immutable review receipts; matching hashes supplement, never replace fixed reviews.')
 args = ap.parse_args()
 assert Path('/media/zhuyupei/SanStand').is_mount()
 for path in (args.output, args.combined, args.remote_focus, args.compat_focus, args.chain_focus,
              args.echo_before, *args.bound_review):
  path.resolve().relative_to(V.resolve())
 args.source_copy_manifest.resolve().relative_to((W/'.runtime').resolve())
 args.output.mkdir(exist_ok=True)
 inputs = {}; pending = []; focused = {}; history = {}; failures = {}; findings = []
 bound_helper_reviews = []
 def read(path, expected=None):
  path = Path(path).resolve(strict=True); raw = path.read_bytes(); h = hashlib.sha256(raw).hexdigest()
  assert inputs.setdefault(str(path), h) == h, path
  assert expected is None or h == expected, path
  return raw
 def load(path, expected=None):
  return json.loads(read(path, expected))
 def write(name, value):
  path = args.output/name
  raw = (json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False)+'\n').encode()
  with path.open('xb') as f:
   f.write(raw)
  assert path.read_bytes() == raw
  return hashlib.sha256(raw).hexdigest()
 def sealed(directory, expected=None):
  directory = Path(directory); result = load(directory/'result.json', expected)
  if (directory/'seal.json').exists():
   seal = load(directory/'seal.json')
   # Verify local named artifacts where the review schema provides them.
   for key in ('artifacts_sha256', 'files_sha256'):
    for n, h in seal.get(key, {}).items():
     p = Path(n) if Path(n).is_absolute() else directory/n
     p.resolve().relative_to(directory.resolve())
     if isinstance(h, str):
      read(p, h)
  return result
 def rel(n):
  if n.startswith(P):
   return n
  marker = '/'+P
  if marker in n:
   return P+n.split(marker, 1)[1]
  p = Path(n)
  return str(p.relative_to(W)) if p.is_absolute() else n
 def match_map(value, mapping):
  for n, h in value['reviewed_source_sha256'].items():
   n = rel(n)
   assert mapping.get(n) == h, (n, h, mapping.get(n))
 def original_sources(r):
  assert all(r['source_sha256'].get(n) == h for n, h in baseline['source_sha256'].items()
             if n not in PROD)
 def xml_check(directory, r, *, incomplete=False):
  directory = Path(directory)
  root = ET.fromstring(read(directory/'pytest.xml', r['xml_sha256']))
  summary = {k:sum(int(s.get(k, 0)) for s in root.iter('testsuite'))
             for k in ('tests', 'failures', 'errors', 'skipped')}
  assert summary == r['summary']
  counts = dict.fromkeys(r['test_scope'], 0); statuses = dict(failures=0, errors=0, skipped=0)
  bad = {}; blanks = []
  for case in root.iter('testcase'):
   n = case.get('file')
   if n not in counts:
    assert incomplete and not n and not case.get('name')
    assert not any(case.findall(t) for t in ('failure', 'error', 'skipped'))
    blanks.append(dict(case.attrib)); continue
   counts[n] += 1
   for tag, key in (('failure', 'failures'), ('error', 'errors'), ('skipped', 'skipped')):
    statuses[key] += len(case.findall(tag))
   if case.find('failure') is not None:
    bad.setdefault(n, []).append(case.get('name'))
  assert counts == r['test_case_counts_by_file'] and sum(counts.values()) == summary['tests']
  assert statuses == {k:summary[k] for k in statuses}
  assert len(counts) == r['test_scope_files']
  declared = load(directory/'scope.json')
  assert declared['tests'] == r['test_scope'] and declared['source_sha256'] == r['source_sha256']
  assert declared['baseline_sha256'] == BASE_SHA
  read(directory/'pytest.log', r['log_sha256'])
  read(directory/'bootstrap/sitecustomize.py', r['bootstrap_sha256'])
  assert r['source_count'] == len(r['source_sha256'])
  assert r['source_unchanged'] and not r['changed_during_run'] and not r['external_boundary_attempts']
  assert r['baseline_file_test_counts_retained']
  assert r['live_ros_contacted'] is False and r['motion_commands_sent'] is False
  assert r['physical_grasp_verified'] is False and r['temperature_queried'] is False
  assert r['baseline'] == str(BASE) and r['baseline_sha256'] == BASE_SHA
  assert '-k' not in r['command'] and not any(a.startswith('--deselect') for a in r['command'])
  if incomplete:
   assert r['scope_complete'] is False and r['returncode'] == 2
   assert blanks and r['unassigned_testcases']
  else:
   assert r['scope_complete'] and not r['unassigned_testcases'] and all(counts.values())
  if (directory/'seal.json').exists():
   load(directory/'seal.json')
  return counts, bad
 def run(directory, total, *, pass_expected, task_sha=None, new_counts=None, expected_sources=None,
         failed_summary=None):
  r = load(Path(directory)/'result.json')
  assert r['passed'] is pass_expected and r['returncode'] == (0 if pass_expected else 1)
  assert r['summary']['tests'] == total
  if pass_expected:
   assert r['summary'] == dict(tests=total, failures=0, errors=0, skipped=0)
  if failed_summary is not None:
   assert r['summary'] == failed_summary
  counts, bad = xml_check(directory, r)
  original_sources(r)
  if expected_sources is not None:
   for n, h in expected_sources.items():
    assert r['source_sha256'][n] == h, (str(directory), n)
  if task_sha is not None:
   assert r['source_sha256'][TASK] == task_sha
  for n, count in (new_counts or {}).items():
   assert counts[n] == count
  return r, counts, bad

 baseline = load(BASE, BASE_SHA)
 assert baseline['source_count'] == 1029 and baseline['test_scope_files'] == 279
 assert baseline['test_summary'] == dict(tests=8043, failures=0, errors=0, skipped=0)
 backup = W/'.runtime/gen40_baseline_source'; bm = load(backup/'manifest.json')
 assert bm['source_sha256'] == baseline['source_sha256']
 assert all(sha(backup/n) == h for n, h in baseline['source_sha256'].items())
 old_meta = load(OLD_COPY/'manifest.json', OLD_COPY_SHA); old_sources = old_meta['source_sha256']
 assert len(old_sources) == 1032 and old_sources[TASK] == INIT_TASK
 assert all(sha(OLD_COPY/n) == h for n, h in old_sources.items())
 cm = load(args.source_copy_manifest, FINAL_COPY_SHA); copy = args.source_copy_manifest.resolve().parent
 assert cm['baseline_release_sha256'] == BASE_SHA and cm['source_count'] == 1032
 assert cm['regression_pending'] is True and cm.get('runtime_deployed', False) is False
 assert cm.get('physical_grasp_verified', False) is False
 sources = cm['source_sha256']
 assert set(sources) == set(baseline['source_sha256']) | set(NEW_TESTS) and len(sources) == 1032
 assert {n for n, h in baseline['source_sha256'].items() if sources[n] != h} == set(PROD)
 assert all(sources[n] == h for n, h in PROD.items())
 assert all(sources[n] == h for n, h in TEST_SHA.items())
 assert all(sources[n] == baseline['source_sha256'][n] for n in baseline['test_scope'])
 def source_check():
  for n, h in sources.items():
   assert sha(W/n) == sha(copy/n) == h, n
 source_check()
 found = {str(p.relative_to(W)) for root in (W/'src/alicia_grasp_modes/tests', W/'src/alicia_flexible_grasp_supervisor/tests')
          for p in root.glob('test_gen41*.py')}
 assert found == set(NEW_TESTS)
 reviewed = {n:sources[n] for n in sorted(set(PROD) | set(NEW_TESTS))}
 for n in HELPERS:
  read(W/n); reviewed[n] = sha(W/n)
 scope = load(args.combined/'scope.json')
 expected_scope = set(baseline['test_scope']) | set(NEW_TESTS)
 assert scope['full'] is True and len(scope['tests']) == len(set(scope['tests'])) == 282
 assert set(scope['tests']) == expected_scope and scope['source_sha256'] == sources
 assert scope['baseline'] == str(BASE) and scope['baseline_sha256'] == BASE_SHA

 # Historical review SHA maps remain attached to their original source phase.
 proofs = {name:sealed(V/name, h) for name, h in PROOFS.items()}
 a = proofs['gen41_remote_candidate_independent_review_r1']
 assert a['complete'] and not a['blocking_findings']; match_map(a, old_sources)
 b = proofs['gen41_task_candidate_independent_review_r2']
 assert b['review_complete_for_bound_candidate_hashes'] and not b['blocking_findings']; match_map(b, old_sources)
 c = proofs['gen41_four_production_independent_review_r1']
 assert c['review_complete_for_bound_hashes'] and not c['blocking_findings']
 old_init = dict(old_sources); old_init[TASK] = OLD_TASK; match_map(c, old_init)
 c2 = proofs['gen41_four_production_initialization_review_r2']
 assert c2['review_complete_for_bound_hashes'] and not c2['blocking_findings']; match_map(c2, old_sources)
 for n, names in c['protected_original_gate_AST_identical'].items():
  old, current = functions(backup/n), functions(W/n)
  assert all(old[k] == current[k] for k in names), (n, names)
 # Exact two-line constructor cleanup stays distinct from later causal fixes.
 initdir = V/'gen41_task_initialization_patch_r2'
 init = sealed(initdir, '3730fe191b4705ef7cfbcc4535a61ae2807b0854c37d0c9b85fe0816da067094')
 before = read(initdir/'station_compute_wait_task_r1.py', OLD_TASK).decode()
 after = read(initdir/'station_compute_wait_task_r2.py', INIT_TASK).decode()
 anchor = '    def _clear_observe_return_attempt(self):\n        self._station_supplement_proposal = None\n        self._station_task_availability_pending.clear()\n        self._station_task_availability_verified.clear()\n'
 replacement = anchor.replace('self._station_task_availability_pending.clear()', 'self._station_task_availability_pending = {}').replace('self._station_task_availability_verified.clear()', 'self._station_task_availability_verified = {}')
 assert before.count(anchor) == 1 and before.replace(anchor, replacement) == after
 assert init['source_unchanged_except_authorized_two_lines'] and init['new_test_files_unchanged']
 assert not init['modified_old_test_files']
 # Final hashes must be covered by independent receipts, not by a digest alone.
 final_review_names = ('gen41_feedback_late_remote_shared_independent_review_r2',
   'gen41_feedback_idle_task_independent_review_r2',
   'gen41_plan_freshness_chain_fixture_independent_review_r5',
   'gen41_plan_freshness_chain_independent_review_r5')
 coverage = {SCHEMA: sources[SCHEMA]}
 for name in final_review_names:
  proof = proofs[name]; assert proof['complete'] and not proof['blocking_findings']
  match_map(proof, sources)
  coverage.update({rel(n):h for n, h in proof['reviewed_source_sha256'].items()})
 for path in args.bound_review:
  proof = sealed(path.parent) if path.name == 'result.json' else load(path)
  assert proof.get('complete', proof.get('reviewed', False)) and not proof.get('blocking_findings', [])
  if 'reviewed_source_sha256' in proof:
   match_map(proof, sources)
   coverage.update({rel(n):h for n, h in proof['reviewed_source_sha256'].items()})
  else:
   # A typed script review binds an actual current helper, or the exact
   # preserved r4 predecessor, plus all seven current source bindings. It
   # cannot replace any independently required production-source review.
   assert proof['review_kind'] == 'independent read-only final checker incremental review'
   assert proof['complete'] is True and proof['blocking_findings'] == []
   assert proof['checker_main_executed'] is False and proof['pytest_executed'] is False
   assert proof['release_approved'] is False and proof['full_regression_approved'] is False
   script_path = Path(proof['reviewed_script_path']).resolve()
   script_path.relative_to((W/'.runtime').resolve())
   is_current_checker = script_path == Path(__file__).resolve()
   if is_current_checker:
    expected_script_sha = sha(__file__)
   else:
    assert script_path == (W/'.runtime/final_review_gen41_combined_r4_20261017.py').resolve(), 'unsupported script-review helper'
    assert path.resolve() == (V/'gen41_combined_checker_independent_review_r4/result.json').resolve()
    read(path, '1d450a77b06254a28ba4d670f4ef18d75d21a8ff3fed5e586294a9402b4836fb')
    expected_script_sha = '5a32ee99bafecaadf9ff8e78ecd0837e7210ea2146f7ab4ed0265d45faf5b6d6'
   assert proof['reviewed_script_sha256'] == expected_script_sha
   read(script_path, expected_script_sha)
   assert proof['R6_source_count'] == 1032 and proof['actual_R6_all_sources_exact'] is True
   assert proof['R6_source_copy_manifest_sha256'] == FINAL_COPY_SHA
   assert proof['current_source_bindings'] == {n:sources[n] for n in set(PROD) | set(NEW_TESTS)}
   bound_helper_reviews.append(dict(review_receipt=str(path.resolve()),
    review_sha256=sha(path), reviewed_script=str(script_path),
    reviewed_script_sha256=expected_script_sha, current_source_bindings_exact=True,
    counted_as_production_source_coverage=False, reviewed_current_checker=is_current_checker,
    reviewed_predecessor_checker=not is_current_checker))
 assert all(coverage.get(n) == sources[n] for n in set(PROD) | set(NEW_TESTS))
 # Preserve the actual preparation-only schema failure. No regression
 # result or source changed while this adapter was corrected.
 preparation_failure = load(V/'gen41_checker_r4_prepare_failure/result.json', '5ef63227fdc284e06d18cab760685938646fd5fd4f8913153de5f65682a620d2')
 assert preparation_failure['helper_sha256'] == '5a32ee99bafecaadf9ff8e78ecd0837e7210ea2146f7ab4ed0265d45faf5b6d6'
 assert preparation_failure['phase'] == 'preparation_not_final' and preparation_failure['exit_code'] == 1
 assert preparation_failure['exception'] == 'KeyError: reviewed_source_sha256'
 assert preparation_failure['production_and_tests_edited'] is False
 assert preparation_failure['release_review_complete'] is False
 history['gen41_checker_r4_prepare_failure'] = dict(phase='preparation_not_final', exit_code=1,
  exception=preparation_failure['exception'], interpretation='additional known script-review schema was not the production source-review schema; no test failure or release approval')
 # Preserve all old test bytes, and the original62/45 prefix of extended tests.
 assert (W/RTEST).read_bytes().startswith((OLD_COPY/RTEST).read_bytes())
 compat_receipt = sealed(V/'gen41_legacy_pending_compat_candidate_r2')
 assert compat_receipt['complete'] and compat_receipt['candidate_source_sha256'][TASK] == COMPAT_TASK
 assert compat_receipt['candidate_source_sha256'][TTEST] == TEST45
 support_receipt = sealed(V/'gen41_legacy_pending_support_candidate_r1', '5b0796e33176ae95a64a5a3a15c61bb232274a0ef60b130ea6e7dcb1894a1d36')
 support_review = proofs['gen41_support_idle_delta_independent_review_r1']
 assert support_review['review_complete_for_bound_candidate_hashes'] and not support_review['blocking_findings']
 assert list(support_review['reviewed_source_sha256'].values()) == [SUPPORT_TASK]
 # SDK fixture correction applies only to new test11, then explicit new echo13.
 sdkfix = sealed(V/'gen41_sdk_transport_fixture_patch_r3')
 sdkreview = sealed(V/'gen41_sdk_transport_fixture_independent_review_r3', '438ab3ed4e052fcf81fb29b1e8ca418c162f1617574df0744c90b89cd6fa7452')
 assert sdkfix['assertions_case_count_unchanged'] == 11 and sdkfix['original_gate_and_old_helpers_unchanged']
 assert sdkfix['unchanged_gap_span_ns'] == 280072042 and sdkfix['unchanged_original_max_span_ns'] == 200000000
 assert sdkreview['complete'] and not sdkreview['blocking_findings'] and sdkreview['case_count_preserved'] == 11
 match_map(sdkreview, old_sources)
 old_test = functions(OLD_COPY/CTEST); new_test = functions(W/CTEST)
 assert all(new_test[n] == body for n, body in old_test.items() if n.startswith('test_'))
 ops = proofs['gen41_operations_helpers_independent_review_r1']; assert not ops['blocking_findings']
 for row in ops['helpers']:
  assert sha(W/row['path']) == row['sha256']
 sealer = proofs['gen41_sealer_independent_review_r1']; assert sealer['reviewed'] and not sealer['blocking_findings']
 assert set(sealer['reviewed_source_sha256']) == set(HELPERS)
 assert all(reviewed[n] == h for n, h in sealer['reviewed_source_sha256'].items())

 # Historical green focus does not stand in for final changed source evidence.
 for name, total, task in (
   ('gen41_remote_private_wait_focus_r1', 148, OLD_TASK),
   ('gen41_task_availability_focus_r2', 124, INIT_TASK),
   ('gen41_pending_handoff_chain_focus_r3', 11, INIT_TASK)):
  r, _, _ = run(V/name, total, pass_expected=True, task_sha=task)
  history[name] = dict(summary=r['summary'], source_task_sha256=task, current_source_evidence=False)
 # Preserve actual earlier red outcomes; no fixture failure is called a grasp failure.
 for name, summary in (
   ('gen41_task_availability_focus_r1', dict(tests=124, failures=0, errors=108, skipped=0)),
   ('gen41_pending_handoff_chain_focus_r1', dict(tests=11, failures=11, errors=0, skipped=0)),
   ('gen41_pending_handoff_chain_focus_r2', dict(tests=11, failures=4, errors=0, skipped=0))):
  r, _, _ = run(V/name, summary['tests'], pass_expected=False, failed_summary=summary)
  history[name] = dict(summary=r['summary'], source_task_sha256=r['source_sha256'][TASK], interpretation='initialization_or_new_fixture_failure_retained')
 legacy_files = {P+'tests/'+n for n in ('test_station_compute_wait_integration.py',
   'test_station_support_gap_consumers.py', 'test_station_support_gap_three_owners.py',
   'test_station_contact_admission.py', 'test_gen24_station_support_identity_wait.py',
   'test_gen24_support_gap_identity_independent.py')}
 oldfull, counts, bad = run(V/'gen41_combined_r1', 8153, pass_expected=False,
   failed_summary=dict(tests=8153, failures=16, errors=0, skipped=0), expected_sources=old_sources)
 assert oldfull['regression_scope'] == 'full_combined' and set(bad) == legacy_files
 assert all(counts[n] == v for n, v in baseline['test_case_counts_by_file'].items())
 assert sum(map(len, bad.values())) == 16
 failures['gen41_combined_r1'] = dict(summary=oldfull['summary'], failing_cases=bad, source_task_sha256=INIT_TASK)
 # Compatibility first285/286 and support-only delta final286/286 are historical.
 oldcompat, _, compat_bad = run(V/'gen41_legacy_pending_compat_focus_r1', 286,
   pass_expected=False, task_sha=COMPAT_TASK,
   failed_summary=dict(tests=286, failures=1, errors=0, skipped=0), new_counts={TTEST:45})
 assert set(compat_bad) == {P+'tests/test_gen24_support_gap_identity_independent.py'}
 assert len(next(iter(compat_bad.values()))) == 1 and 'empty' in next(iter(compat_bad.values()))[0]
 failures['gen41_legacy_pending_compat_focus_r1'] = dict(summary=oldcompat['summary'], failing_cases=compat_bad, source_task_sha256=COMPAT_TASK)
 oldcompat2, _, _ = run(V/'gen41_legacy_pending_compat_focus_r2', 286,
   pass_expected=True, task_sha=SUPPORT_TASK, new_counts={TTEST:45})
 history['gen41_legacy_pending_compat_focus_r2'] = dict(summary=oldcompat2['summary'], source_task_sha256=SUPPORT_TASK, current_source_evidence=False)
 # Root's old complete public-relay reproducer had all consumer SDK brackets.
 before, _, _ = run(V/'gen41_pending_handoff_public_relay_before_r2', 1,
   pass_expected=False, failed_summary=dict(tests=1, failures=1, errors=0, skipped=0))
 assert all(before['source_sha256'][n] == h for n, h in baseline['source_sha256'].items())
 read(V/'gen41_pending_handoff_public_relay_before_r2/interpretation.json')
 # The first13-case echo attempt stopped at missing optional first_loss; not a causal witness.
 echo1, _, echo1_bad = run(V/'gen41_feedback_idle_echo_chain_before_r1', 13,
   pass_expected=False, task_sha=SUPPORT_TASK,
   failed_summary=dict(tests=13, failures=2, errors=0, skipped=0))
 xroot = ET.fromstring(read(V/'gen41_feedback_idle_echo_chain_before_r1/pytest.xml', echo1['xml_sha256']))
 for case in xroot.iter('testcase'):
  failure = case.find('failure')
  if failure is not None:
   assert 'first_loss' in ''.join(failure.itertext())
 failures['gen41_feedback_idle_echo_chain_before_r1'] = dict(summary=echo1['summary'], failing_cases=echo1_bad, interpretation='new_fixture_optional_field_KeyError_before_echo_delivery_not_causal')
 # The second attempt was explicitly interrupted by user pause; three passes are not failure.
 idir = V/'gen41_feedback_idle_echo_chain_before_r2'
 interrupted = load(idir/'result.json', 'c46d5ccc820b52793243501a98a47fddafc3f1ef280d3caa6293a41d1f566194')
 assert interrupted['passed'] is False and interrupted['returncode'] == 2
 assert interrupted['summary'] == dict(tests=3, failures=0, errors=0, skipped=0)
 xml_check(idir, interrupted, incomplete=True)
 history[idir.name] = dict(summary=interrupted['summary'], returncode=2,
   scope_complete=False, interpretation='user_interrupted_no_failure_or_complete_pass_claim')
 # The C13after reached every causal check, then its original fixture5s
 # freshness correctly rejected a5.291143274-secondplan. Preserve the red
 # result; a new C14fixture must explicitly read existing YAML120 for only
 # the long positive and retain an original5-second stale negative.
 stale_dir = V/'gen41_pending_handoff_chain_focus_r4'
 stale, _, stale_bad = run(stale_dir, 13, pass_expected=False,
  task_sha=PROD[TASK], failed_summary=dict(tests=13, failures=1, errors=0, skipped=0),
  expected_sources=PROD, new_counts={CTEST:13})
 assert stale['source_sha256'][CTEST] == OLD_C13_SHA
 assert {n for n in sources if stale['source_sha256'][n] != sources[n]} == {CTEST}
 assert set(stale_bad) == {CTEST}
 assert set(stale_bad[CTEST]) == {'test_feedback_gap_idle_echo_before_fresh_recovery_keeps_original_handoff'}
 failures[stale_dir.name] = dict(summary=stale['summary'], failing_cases=stale_bad,
  interpretation='causal_private_and_current_READY_checks_passed_then_original_fixture5s_staleplan_rejected')
 stale_review = sealed(V/'gen41_chain_stale_plan_boundary_review_r1',
  'b4e7f8529bf119095358e4e953ce4e8f3cd434e66c8be0cb7ef340a20d528400')
 assert stale_review['complete'] and stale_review['component_causal_fixes_passed_in_failed_chain']
 addendum = load(V/'gen41_chain_stale_plan_boundary_review_r1/source_config_addendum.json',
  '20e1718863d9b44c80b5d1c814bdcccf5440964952ff98e0629986b7eedf8855')
 assert addendum['prior_result_sha256'] == sha(V/'gen41_chain_stale_plan_boundary_review_r1/result.json')
 cfg = addendum['source_configuration']
 assert cfg['grasp_6d_plan_validity_sec'] == 120.0
 assert sources[cfg['path']] == cfg['sha256'] == baseline['source_sha256'][cfg['path']]
 fixture = proofs['gen41_plan_freshness_chain_fixture_r5']
 assert fixture['candidate_sha256'] == TEST_SHA[CTEST] and fixture['previous_test_sha256'] == OLD_C13_SHA
 assert fixture['expected_new_case_count'] == 14 and fixture['pytest_run'] is False
 assert fixture['config_source'] == cfg['path'] and fixture['config_sha256'] == cfg['sha256']
 assert fixture['config_existing_plan_validity_sec'] == 120.0
 assert fixture['original_five_second_negative_retained'] and fixture['no_plan_retimestamps'] and fixture['no_freshness_override']
 c14review = proofs['gen41_plan_freshness_chain_fixture_independent_review_r5']
 assert c14review['complete'] and not c14review['blocking_findings']
 r5_copy = W/'.runtime/gen41_r5_source'
 previous_meta = load(r5_copy/'manifest.json', '7069f2ebd3eee05c5efa8d4f7c40e3f98e0b508a66af54a27c55e26465b7a77f')
 assert previous_meta['source_sha256'][CTEST] == OLD_C13_SHA
 assert {n for n in sources if previous_meta['source_sha256'][n] != sources[n]} == {CTEST}
 old_c13 = read(r5_copy/CTEST, OLD_C13_SHA)
 c13_functions, c14_functions = functions(r5_copy/CTEST), functions(W/CTEST)
 unchanged_names = ('test_prechecked_candidate_crosses_feedback_gap_and_idle_expiry_once',
   'test_true_changes_still_cancel_prechecked_private_candidate',
   'test_recovery_without_consumer_sdk_bracket_never_publishes_or_moves',
   'test_idle_echo_with_inconsistent_gap_span_never_authorizes_original_plan')
 assert all(c13_functions[n] == c14_functions[n] for n in unchanged_names)
 assert 'test_same_long_idle_echo_scenario_keeps_five_second_plan_expiry_hard' in c14_functions
 # Optional read-only live configuration diagnostic is contextual evidence,
 # never a substitute for the original current-source/freshness/motion gates.
 live_policy_path = V/'gen41_readonly_plan_validity_policy_r1/result.json'
 if live_policy_path.exists():
  live_policy = load(live_policy_path)
  assert live_policy['live_parameter_plan_validity_sec'] == 120.0
  assert live_policy['parameters_changed'] is False and live_policy['motion_commands_sent'] is False
  assert live_policy['temperature_queried'] is False and live_policy['diagnostic_only'] is True
  assert live_policy['execution_clearance'] is False
 # The first C14after was explicitly interrupted after one completed pass.
 # It is neither an experimental failure nor a complete C14pass.
 c14_interrupted_dir = V/'gen41_pending_handoff_chain_focus_r5'
 c14_interrupted = load(c14_interrupted_dir/'result.json',
  '3d9010e12a213f4bce4778df20f3ae19bd1701108c9eeb2da7d9a994cbe721c8')
 assert c14_interrupted['passed'] is False and c14_interrupted['returncode'] == 2
 assert c14_interrupted['summary'] == dict(tests=1, failures=0, errors=0, skipped=0)
 assert c14_interrupted['source_sha256'] == sources
 xml_check(c14_interrupted_dir, c14_interrupted, incomplete=True)
 history[c14_interrupted_dir.name] = dict(summary=c14_interrupted['summary'], returncode=2,
  scope_complete=False, interpretation='user_interrupted_one_pass_not_failure_or_complete_C14pass')
 # Actual producer idle->gap echo must independently reproduce before final sources are adopted.
 if (args.echo_before/'result.json').exists():
  eb, _, ebbad = run(args.echo_before, 13, pass_expected=False, task_sha=SUPPORT_TASK)
  assert eb['summary']['failures'] > 0 and eb['summary']['errors'] == eb['summary']['skipped'] == 0
  assert eb['source_sha256'][REMOTE] == old_sources[REMOTE] and eb['source_sha256'][SHARED] == old_sources[SHARED]
  assert eb['source_sha256'][CTEST] == OLD_C13_SHA
  assert eb['source_sha256'][RTEST] == old_sources[RTEST] and eb['source_sha256'][TTEST] == TEST45
  assert set(ebbad) == {CTEST}
  assert set(ebbad[CTEST]) == {'test_feedback_gap_idle_echo_before_fresh_recovery_keeps_original_handoff'}
  explanation = load(args.echo_before/'interpretation.json')
  assert explanation['result_sha256'] == sha(args.echo_before/'result.json')
  assert explanation['summary'] == eb['summary']
  assert explanation['qualified_as_echo_handoff_reproduction'] is True
  assert explanation['physical_grasp_verified'] is False
  failures[args.echo_before.name] = dict(summary=eb['summary'], failing_cases=ebbad, interpretation='closed_original_producer_idle_causal_handoff_reproducer')
 else:
  pending.append('closed genuine13-case producer echo before_r3 (before_r2 was user interrupted)')
 for p in (V/'gen41_before_recovery_fixture_addendum_r1/result.json',
           V/'gen41_pending_handoff_chain_focus_r2/interpretation.json',
           V/'gen41_pending_handoff_before_focus_r1/result.json'):
  read(p)

 late = P+'tests/test_arrival_feedback_gap_late_consumers.py'
 task_original = P+'tests/test_station_compute_wait_task.py'
 old_task_focus = load(V/'gen41_task_availability_focus_r2/result.json')
 assert late not in baseline['test_case_counts_by_file']
 assert old_task_focus['test_case_counts_by_file'][late] == 26
 assert old_task_focus['source_sha256'][late] == baseline['source_sha256'][late] == sources[late]
 compat_scope = legacy_files | {task_original, late, TTEST}
 compat_counts = {n:baseline['test_case_counts_by_file'][n] for n in compat_scope if n not in (late, TTEST)}
 compat_counts.update({late:26, TTEST:47}); assert sum(compat_counts.values()) == 288
 remote_scope = {RTEST, P+'tests/test_station_compute_wait_remote.py', P+'tests/test_station_compute_work_handoff.py'}
 for directory, total, exact_scope, required_counts in (
   (args.remote_focus, 150, remote_scope, {RTEST:64}),
   (args.compat_focus, 288, compat_scope, compat_counts),
   (args.chain_focus, 14, {CTEST}, {CTEST:14})):
  if (directory/'result.json').exists():
   r, counts, _ = run(directory, total, pass_expected=True, expected_sources=PROD,
                      new_counts=required_counts)
   assert set(r['test_scope']) == exact_scope and r['source_count'] == 1032
   assert set(r['source_sha256']) == set(sources)
   differences = {n for n, h in r['source_sha256'].items() if h != sources[n]}
   allowed_old_capture = directory in (args.remote_focus, args.compat_focus)
   if differences:
    assert allowed_old_capture and differences == {CTEST} and CTEST not in exact_scope
    assert r['source_sha256'][CTEST] == OLD_C13_SHA
   for n in exact_scope & set(NEW_TESTS):
    assert r['source_sha256'][n] == sources[n]
   focused[directory.name] = dict(summary=r['summary'], current_production_sha256=PROD,
    captured_source_differences={n:dict(captured_sha256=r['source_sha256'][n],
     final_sha256=sources[n], executed_in_this_focus=False) for n in sorted(differences)},
    all_other_captured_sources_exact=True,
    all_1032_captured_sources_equal_final=not differences)
  else:
   pending.append('closed current source focus '+directory.name)

 read(__file__); reviewed[str(Path(__file__).resolve().relative_to(W))] = sha(__file__)
 source_check()
 for path, h in inputs.items():
  assert sha(path) == h, path
 prepared = dict(schema_version=1, kind='gen41_causal_handoff_review_preparation', recorded_ns=time.time_ns(),
  reviewed=False, release_review_complete=False, full_regression_approved=False,
  reviewed_source_sha256=reviewed, evidence_sha256=dict(inputs), pending=pending,
  bound_helper_review_lineage=bound_helper_reviews,
  source_count=1032, full_test_files=282, expected_summary=EXPECTED, new_test_counts=NEW_TESTS,
  exact_four_authorized_production_sources=PROD, all_old_279_test_files_byte_unchanged=True,
  focused_evidence=focused, historical_focused_evidence=history, retained_failures=failures,
  supplemental_original_outside_full_scope={late:26}, runtime_deployment_pending=True,
  physical_grasp_verified=False, motion_authorized=False)
 if not args.final:
  h = write('prepared.json', prepared)
  print(json.dumps(dict(prepared=True, release_review_complete=False, pending=pending, sha256=h)))
  return
 assert not pending, pending
 result = args.combined/'result.json'; assert result.exists(), 'full result not closed'
 r, counts, _ = run(args.combined, EXPECTED['tests'], pass_expected=True,
                    expected_sources=sources, new_counts=NEW_TESTS)
 assert r['summary'] == EXPECTED and r['source_count'] == 1032 and r['source_sha256'] == sources
 assert r['regression_scope'] == 'full_combined' and r['test_scope_files'] == 282
 assert r['test_scope'] == scope['tests'] and set(counts) == expected_scope
 assert all(counts[n] == v for n, v in baseline['test_case_counts_by_file'].items())
 assert r['runner_sha256'] == reviewed[HELPERS[0]]
 source_check()
 for path, h in inputs.items():
  assert sha(path) == h, path
 review = dict(prepared)
 review.update(kind='gen41_final_causal_handoff_combined_review', recorded_ns=time.time_ns(),
  reviewed=True, release_review_complete=True, full_regression_approved=True, blocking_findings=[],
  evidence_sha256=dict(inputs), pending=[], full_summary=r['summary'], full_regression=str(result.resolve()),
  source_and_candidate_copy_match_full=True, all_original_279_file_counts_exact=True,
  authorized_changed_production=sorted(PROD), modified_existing_tests=[], deployed=False,
  live_ros_contacted=False, hardware_contacted=False, production_or_tests_edited_during_final_review=False,
  fixed_findings=[
   'Full8153 was honestly red on16 original cases; original sourcecopy and six failing-file case names are retained. Legacy original-overrun/support owners remain original hard handlers, rather than being relabeled as feedback availability.',
   'Compatibility286 first had one support-idle empty failure. Final9ef restored original support/discarded routing and independently passed286; those sources are historical, not final causal proof.',
   'A real producer can repeat its parent READY header after geometry expiry while preserving the separate full rejected-feedback cause/discarded stamp. The shared classifier now recognizes that cause as nonmetric, with clock_only=false and no timestamp rewrite.',
   'Exact local rejected-row and parent history may classify late original gap or idle repeats at their original measured endpoints. Current owner/SDK is still checked; an old callback cannot quarantine a newer READY. Unknown or changed cause remains hard.',
   'Current pending work is private and nonexecutable. Partial markers cannot qualify a gap, enroll READY, renew a source or deadline, undo a hard cutoff, or authorize motion. Full current source/SDK/support/geometry/path checks remain.',
   'Complete rejected-bracket pending or full READY recovery proof is sufficient; obsolete empty topics are not a new mandatory gate. Original200ms SDK bracket and original90-second phase limit remain unchanged.',
   'Initial108 construction errors and11 chain failures required exactly two early clear-to-dict assignments. C11 synthetic SDK transport4F were fixture-only and retained. C13 first_loss2F occurred before echo delivery; its interrupted3P retry is neither failure nor complete success.',
   'C13after12P1F reached every fresh currentREADY/private/late-callback check, then the original five-second fixture freshness correctly rejected the old plan at5.291143274seconds. That failure remains red. Only the new long positive now reads immutablepre-existing YAML120 policy; a same-long-scenario originalfive-secondnegative still requires PLAN_STALE and zero contact/close.',
   'A288/B150 executed their final unchanged targets while capturing the old C13test outside those scopes. Only that exact olde792 versus final3d1c file difference is allowed and disclosed; all other1031captured sources match final. CurrentC14after and full require all1032final sources.',
   'The first C14after was user-interrupted after onepass. Its return2/incomplete scope is never classified as failure or complete success.'],
  actual_record_boundary='Actual G40 accepted a measured-schema2 precheck without approach/close. The original rejected280.072ms frame is unqualified; offline new analytic scene checks repair source handoff, not historical physical success.',
  limitations=[
   'All focused/full replays use explicit simulated SDK/FK/IK/services and synthetic scene data; they do not prove a current physical target/path/grip. HOLD_UNVERIFIED is not physical success.',
   'Earlier18/54 memory components were isolated state checks. Obsolete unconditional waiting for old failed topics was superseded by evidence sufficiency.',
   'Root remains responsible for separately reviewed idle deployment and fresh live motion admission. This checker never contacts ROS/hardware or authorizes motion.'])
 h = write('review.json', review)
 seal = write('seal.json', dict(schema_version=1, review_completed=True, release_review_complete=True,
  artifacts_sha256={'review.json':h}, reviewed_source_sha256=reviewed, full_summary=r['summary'],
  source_count=1032, blocking_findings=[], runtime_deployment_pending=True, motion_authorized=False))
 print(json.dumps(dict(review=str(args.output/'review.json'), review_sha256=h, seal_sha256=seal, blocking_findings=[])))


if __name__ == '__main__':
 main()
