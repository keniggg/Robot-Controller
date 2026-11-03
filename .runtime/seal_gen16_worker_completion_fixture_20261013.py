#!/usr/bin/env python3
import difflib,hashlib,json,time
from pathlib import Path
W=Path(__file__).resolve().parents[1];C=W/'.runtime/gen16_worker_completion_fixture_20261013'
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
O=N/'verification/gen16_worker_completion_fixture_delivery'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
proof=N/'verification/gen16_worker_completion_fixture_r2/result.json';p=json.loads(proof.read_text());assert p['passed']
prior=N/'verification/gen16_success_first_actual_integrated/result.json';d=json.loads(prior.read_text())
assert all(sha(W/r)==h for r,h in d['source_sha256'].items())
r='src/alicia_grasp_modes/tests/test_station_cancelled_direct_retry.py'
patch=''.join(difflib.unified_diff((W/r).read_text().splitlines(True),(C/r).read_text().splitlines(True),fromfile='a/'+r,tofile='b/'+r))
(C/'changes.patch').write_text(patch)
scope=[x for x in d['command'] if isinstance(x,str) and x.startswith(('src/','.runtime/')) and x.endswith('.py')]
assert len(scope)==len(set(scope))==176 and r in scope
m=dict(schema=1,kind='GEN16_WORKER_COMPLETION_FIXTURE_DELIVERY',passed=True,candidate_root=str(C),
 files=[dict(path=r,base_sha256=sha(W/r),candidate_sha256=sha(C/r))],test_scope=scope,
 prior_actual_result=dict(path=str(prior),sha256=sha(prior),passed=False),
 proof=dict(path=str(proof),sha256=sha(proof)),
 verification=dict(path=str(proof),sha256=sha(proof)),patch_sha256=sha(C/'changes.patch'),
 focused_test_scope=[r],focused_tests=21,baseline_expected_failed_cases=1,
 current_baseline_sources=len(d['source_sha256']),production_files_changed=0,
 explanation='Old helper reads busy/metrics without the stream lock between real worker busy=False and direct-slot release within that lock. Metrics publish earlier. GEN16 full run exposed this scheduling window; actual poller reads marker while holding the same lock. Candidate helper observes the completed lock transaction.',
 change_scope='Only complete_worker synchronization and two deterministic regression tests appended; all original 19 test functions and assertions AST unchanged.',
 negative_control='Deliberately withhold real slot cleanup: helper completes, original successful-path test still fails exactly at its unchanged marker is None assertion. Helper does not poll or wait for marker=None.',
 tests=['Controlled Event pauses the actual worker inside release hook under its real lock; baseline helper returns early and fails deterministically. Candidate waits until transaction ends.',
 'Original real callback collection/submission/worker/recovery/poll success path retained.',
 'Original acceptance/geometry failure non-reopen and 14 owner/scope refusal cases retained.',
 'Deliberately missing cleanup remains caught by original None assertion.'],
 limits='No claim all historical intermittent failures were reproduced from original thread scheduling; this establishes an existing deterministic read race using unchanged production and explains the observed state.',
 actual_unchanged=True,motion_commands_sent=False,temperature_queried=False,live_ros_contacted=False,physical_grasp_verified=False,recorded_ns=time.time_ns())
(C/'delivery_manifest.json').write_text(json.dumps(m,indent=2)+'\n')
assert Path('/media/zhuyupei/SanStand').is_mount();O.mkdir()
for name in ('delivery_manifest.json','changes.patch'):(O/name).write_bytes((C/name).read_bytes())
print(str(C/'delivery_manifest.json'));print(sha(C/'delivery_manifest.json'));print('candidate',sha(C/r),'base',sha(W/r),'scope',len(scope))
