"""Independent immutable delivery/evidence audit. No ROS imports or test reruns."""
from pathlib import Path
import hashlib, json, time

W=Path(__file__).resolve().parent.parent
C=W/'.runtime/gen15_qualified_phase_timestamp_20261013'
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
V=N/'verification'; F=V/'gen14_arrival_independent'
D=C/'delivery_manifest.json'
O=V/'gen15_qualified_phase_delivery_review'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
assert Path('/media/zhuyupei/SanStand').is_mount()
d=read(D)
assert d['passed'] is True and d['provisional'] is False
assert d['installed'] is False and d['deployed'] is False and d['physical_grasp_verified'] is False
assert len(d['files'])==7
for row in d['files']:
    p=W/row['path']
    assert (sha(p) if p.exists() else None)==row['base_sha256'],row['path']
    assert sha(C/row['path'])==row['candidate_sha256'],row['path']
assert sha(C/'delivery.patch')==d['patch_sha256']
assert sha(d['precision_audit']['path'])==d['precision_audit']['sha256']
prior_path=V/'gen14_common_face_integrated_install/result.json'
prior=read(prior_path)
assert len(prior['source_sha256'])==904
assert all(sha(W/p)==h for p,h in prior['source_sha256'].items())
assert d['prior_release']['sha256']==sha(prior_path)
for p,h in d['evidence_sha256'].items():assert sha(p)==h,p
focused_path=V/'gen15_owner_focused_r1/result.json'
focused=read(focused_path)
assert focused['passed'] and focused['summary']==dict(tests=370,failures=0,errors=0,skipped=0)
assert focused['source_unchanged'] and focused['actual_unchanged'] and focused['actual_source_count']==904
assert focused['loaded_sources']==dict(base=str(C/'src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py'),task=str(C/'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_task.py'))
assert all(sha(p)==h for p,h in focused['source_sha256'].items())
independent_path=V/'gen15_owner_independent_r5/result.json'
independent=read(independent_path)
assert independent['passed'] and independent['summary']==dict(tests=121,failures=0,errors=0,skipped=0)
scope=sorted(set(prior['test_scope'])|set(focused['test_scope']))
assert len(scope)==165 and scope==sorted(d['test_scope'])
assert len(d['focused_scope'])==9 and set(d['focused_scope'])==set(focused['test_scope'])
baseline_path=V/'gen15_task_continuation_baseline_r2/result.json'
candidate_path=V/'gen15_task_continuation_candidate_r1/result.json'
baseline=read(baseline_path); candidate=read(candidate_path)
assert baseline['error']=='ValueError: QUALIFIED_STATION_PHASE_OWNER_CHANGED_OR_EXPIRED'
assert baseline['first_phase_created'] and not baseline['continuation_completed']
assert candidate['error'] is None and candidate['continuation_completed']
assert candidate['first_phase']==candidate['final_phase']
assert candidate['first_phase']['header']['stamp']==1791880763644008874
assert candidate['first_phase']['deadline']==1791880853639396667
assert candidate['remaining_original_budget_sec']>89
assert not candidate['motion_commands_sent'] and not baseline['motion_commands_sent']
matrix_path=F/'three_owner_v7_r1_matrix_result.json'
matrix=read(matrix_path)
assert matrix['passed'] and matrix['all_frozen_files_unchanged'] and not matrix['source_changes']
assert matrix['predeclared_order']==[0,1,2] and matrix['attempts_per_phase']==1
assert len(matrix['phases'])==3
freeze_path=Path(matrix['source_freeze']); freeze=read(freeze_path)
assert sha(freeze_path)==matrix['source_freeze_sha256']
assert freeze['predeclared_phases']==[0,1,2] and freeze['attempts_per_phase']==1
for p,info in freeze['sources'].items():
    assert Path(p).stat().st_size==info['size'] and sha(p)==info['sha256'],p
phase_audit=[]
for index,entry in enumerate(matrix['phases']):
    assert entry['phase']==index
    p=Path(entry['result']); r=read(p)
    assert sha(p)==entry['sha256']
    assert r['passed'] and r['source_unchanged'] and r['error'] is None
    assert r['qualified_handoff_only_passed'] and r['task_finish_returned']
    assert r['timer_phase']['selected']==index and r['timer_phase']['predeclared_values']==[0,1,2]
    assert r['timer_phase']['selection_independent_of_frame_results'] and not r['motion_commands_sent'] and r['no_ros_master']
    assert r['script_sha256']==freeze['script_sha256']
    assert r['final_first_loss'] is None and not r['shutdown_pending_transport_events']
    cc=r['concurrency']; assert not cc['worker_errors'] and cc['all_threads_joined']
    assert cc['full_original_57_pair_time_window'] and cc['task_station_admission_actual']
    coverage=r['input_coverage']
    assert coverage['original_rgbd_pair_count']==coverage['producer_pairs_enqueued']==57
    assert coverage['remote_raw_callbacks_completed']['color']==coverage['remote_raw_callbacks_completed']['depth']==57
    assert all(row['unfinished_callbacks']==row['queued_transport_events']==0 for row in r['shutdown_drain'])
    fusion=r['fusion']; contracts=[fusion[prefix+'_registration_contract'] for prefix in ('producer','remote','task')]
    assert contracts[0] and contracts[0]==contracts[1]==contracts[2]
    assert not fusion['task_station_retired'] and fusion['task_target_loss'] is None
    assert fusion['task_support_fault'] is None and not fusion['task_support_overflow']
    assert fusion['registration']['result']['ok'] and fusion['information_gain']['ok']
    assert fusion['phase']=='observing_contact' and fusion['paused'] is False
    continuation=r['task_reached_continuation']
    assert continuation['completed'] and continuation['successful_ensure_count']>=2
    assert continuation['all_ensure_calls_same_owner'] and continuation['final_owner_unchanged']
    calls=continuation['calls']; assert not any('error' in call for call in calls)
    ensure=[call for call in calls if call['method']=='_ensure_qualified_station_phase']
    owner=ensure[0]['phase_owner']
    assert all(call['phase_owner']==owner for call in ensure)
    assert all(call['cache']['started_ns']==owner['header_ns'] and call['cache']['deadline_ns']==owner['deadline_ns'] for call in ensure)
    assert 0<owner['deadline_ns']-owner['header_ns']<=90_000_000_000
    assert 0<=owner['header_ns']-owner['source_ns']<800_000_000
    assert continuation['envelope']['ok'] and continuation['range']['ok']
    assert [call['method'] for call in calls]==['_ensure_qualified_station_phase','_validate_measured_observation_envelope','_wait_for_fresh_observation_camera_target_range','_ensure_qualified_station_phase']
    assert all(0<=row['source_age_sec']<.8 for row in r['rows'])
    phase_audit.append(dict(phase=index,result=str(p),sha256=sha(p),qualified_sources=r['task_receipt']['request']['source_stamps_ns'],exact_owner=owner,successful_ensure_count=len(ensure),full_pairs=57,maximum_processed_source_age_sec=max(row['source_age_sec'] for row in r['rows']),all_three_contracts_equal=True,no_first_loss_or_worker_error=True))

evidence=dict(d['evidence_sha256'])
evidence[str(matrix_path)]=sha(matrix_path); evidence[str(freeze_path)]=sha(freeze_path)
for entry in phase_audit:evidence[entry['result']]=entry['sha256']
for i in range(1,5):
    p=V/('gen15_owner_independent_r%d'%i)/'result.json'
    evidence[str(p)]=sha(p)
evidence[str(Path(__file__).resolve())]=sha(Path(__file__))
result=dict(schema=1,kind='gen15_exact_phase_ownership_independent_delivery_review',passed=True,
    delivery=str(D),delivery_sha256=sha(D),candidate_root=str(C),recorded_ns=time.time_ns(),
    test_scope=scope,focused_scope=d['focused_scope'],files=d['files'],evidence_sha256=evidence,
    actual_baseline_sources_verified=904,candidate_files_verified=7,source_freeze_count=len(freeze['sources']),
    phase_reviews=phase_audit,blocking_findings=[],
    qualified_scope='Ready for hash-bound install and required 165-file actual regression. Candidate software only; no deployment or physical grasp success assertion.',
    findings=[
        'Recorded R3 start timestamp has a real one-ULP writer/reader mismatch. Original failure preserved; exact R3 baseline reproduces, candidate passes actual CAD/range and second ensure without phase renewal.',
        'Both second caches derive from final wire; integer start/deadline caches retain exact identity. All previous qualified owner, request, receipt, ACK, session, plan, manual-stop, ROS and monotonic checks remain.',
        'Observed-contact float-to-ns roundtrip is a distinct potential defect. The exact R3 deadline happens to roundtrip; its valid adjacent +1 ns does not. Real production absolute_deadline_ns setter acceptance is independently tested.',
        'Remote/producer contracts bind integer wire fields. Task station cache comparison already uses wire.to_sec. Seconds fallbacks after ensure returns None are outside current qualified path; remaining seconds uses are budget/age computations, not this equality defect.',
        'Recorder change only waits after owned wrapper exit for actual final file with no active file; timeout preserves failure state and return code. No recording-period or motion logic changed.',
        'Focused 370 includes the 121 independent tests and six recorder tests; counts must not be added as disjoint evidence.',
        'v7 retains original concurrent per-channel clock/ingress/drain and adds unchanged reached-branch AST with real CAD/range/second ensure. Motion/retreat/RPC entries throw; no success stubs create authority.'
    ],
    prior_independent_attempts=[
        dict(run='r1',outcome='collection error: candidate missing unchanged tools dependency; no tests executed'),
        dict(run='r2',outcome='97 passed, 8 test-only Python 3.8 math API errors; replaced with numpy equivalents'),
        dict(run='r3',outcome='104 passed, 1 overly strong test assertion about exact R3 deadline roundtrip; corrected to adjacent legal ns'),
        dict(run='r4',outcome='105 passed but concurrent author-test edit changed source hash; not accepted as immutable verification'),
        dict(run='r5',outcome='121 passed, zero failures/errors/skips; source unchanged; adds 16 real setter ns-reachability cases')
    ],
    invariants=dict(no_epsilon=True,no_deadline_extension=True,no_motion_commands=True,no_temperature_query=True,original_r3_failure_preserved=True),
    limitations=['Actual installed 165-file regression and deployment remain root work.','Offline shared-process replay is not live network scheduling proof.','Full task motion preamble, remote candidate planner/RPC, contact execution and physical grasp remain outside replay success.'],
    installed=False,deployed=False,physical_grasp_verified=False)
O.mkdir(exist_ok=False)
(O/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(path=str(O/'result.json'),sha256=sha(O/'result.json'),passed=True,test_scope_count=len(scope),source_freeze_count=len(freeze['sources'])),ensure_ascii=False),flush=True)
