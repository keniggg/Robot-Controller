"""Independent evidence verification only; no test execution, ROS or mutation."""
from pathlib import Path
import hashlib
import json
import time
import xml.etree.ElementTree as ET

W = Path(__file__).resolve().parent.parent
C = W / '.runtime/gen14_arrival_repair_20261013'
N = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
F = N / 'verification/gen14_arrival_independent'
OUTPUT = N / 'verification/gen14_common_face_delivery_review'
DELIVERY = C / 'delivery_manifest.json'
EXPECTED_DELIVERY_SHA = 'c27b49bd78fd75287c1ff4620d32fbf98edc488f2d4e24937810c9e9fba7bcc0'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


assert Path('/media/zhuyupei/SanStand').is_mount()
assert not OUTPUT.exists()
assert sha(DELIVERY) == EXPECTED_DELIVERY_SHA
delivery = read(DELIVERY)
assert delivery['passed'] and delivery['provisional'] is False
assert not any(delivery[key] for key in ('installed', 'deployed', 'physical_grasp_verified'))
assert Path(delivery['candidate_root']) == C
assert len(delivery['files']) == 16
assert len(delivery['focused_scope']) == 35
assert len(delivery['test_scope']) == 161
assert delivery['limits'] == dict(frame_source_seconds=.8, capture_seconds=3,
                                 actual_task_phase_seconds=90, three_consecutive_observations=3)

evidence = dict(delivery['evidence_sha256'])
for path, expected in evidence.items():
    assert sha(path) == expected, path
prior_path = Path(delivery['prior_release']['path'])
assert sha(prior_path) == delivery['prior_release']['sha256']
prior = read(prior_path)
assert prior['passed'] and len(prior['source_sha256']) == 892
for relative, expected in prior['source_sha256'].items():
    assert sha(W / relative) == expected, relative
assert sorted(set(prior['test_scope']) | set(delivery['focused_scope'])) == sorted(delivery['test_scope'])

candidate = {}
base = {}
for row in delivery['files']:
    relative = row['path']
    assert not Path(relative).is_absolute() and '..' not in Path(relative).parts
    assert relative not in candidate
    candidate[relative] = sha(C / relative)
    assert candidate[relative] == row['candidate_sha256']
    base[relative] = sha(W / relative) if (W / relative).exists() else None
    assert base[relative] == row['base_sha256']
changed = {str(path.relative_to(C)) for path in C.rglob('*.py')
           if not (W / path.relative_to(C)).is_file()
           or sha(path) != sha(W / path.relative_to(C))}
assert changed == set(candidate)
production = {str(C / path): value for path, value in candidate.items() if '/tests/' not in path}
assert len(production) == 6

combo_path = Path(delivery['combined_regression']['path'])
combo = read(combo_path)
assert sha(combo_path) == delivery['combined_regression']['sha256']
assert combo['passed'] and combo['source_unchanged'] and combo['actual_unchanged']
assert combo['summary'] == dict(tests=1069, failures=0, errors=0, skipped=0)
assert sorted(combo['test_scope']) == sorted(delivery['focused_scope'])
assert combo['source_sha256'] == combo['source_sha256_after']
assert len(combo['source_sha256']) == 325
for path, expected in combo['source_sha256'].items():
    assert sha(W / path) == expected, path
for relative, expected in candidate.items():
    assert combo['source_sha256'][str(C / relative)] == expected
xml_path = combo_path.with_name('pytest.xml')
assert sha(xml_path) == combo['pytest_xml_sha256']
xml = ET.parse(xml_path).getroot()
assert len(list(xml.iter('testcase'))) == 1069
assert not list(xml.iter('failure')) and not list(xml.iter('error')) and not list(xml.iter('skipped'))

matrix_path = Path(delivery['three_owner_v6']['path'])
assert sha(matrix_path) == delivery['three_owner_v6']['sha256']
matrix = read(matrix_path)
assert matrix['passed'] and matrix['predeclared_order'] == [0, 1, 2]
assert matrix['each_phase_attempt_count'] == 1
assert matrix['all_sources_unchanged'] and matrix['production_six_sha256'] == production
assert matrix['unchanged_limits'] == dict(source_age_sec=.8, capture_sec=3., recorded_contact_phase_sec=90.)
freeze_path = Path(matrix['source_freeze'])
assert sha(freeze_path) == matrix['source_freeze_sha256']
freeze = read(freeze_path)
assert len(freeze['sources']) == 204
assert freeze['predeclared_phases'] == [0, 1, 2]
assert freeze['production_six_sha256'] == production
for path, record in freeze['sources'].items():
    assert str(Path(path).resolve()) == record['resolved_path']
    assert sha(path) == record['sha256'], path

raw_message_count = 0
for path, record in freeze['new_original_message_fixtures'].items():
    assert sha(path) == record['sha256']
    manifest = read(path)
    assert manifest['passed'] and manifest['message_count'] == record['count']
    for row in manifest['rows']:
        assert sha(row['file']) == row['sha256']
        raw_message_count += 1
assert raw_message_count == 5149

raw_fixture_path = F / 'full_arrival_frames/fixture.json'
raw_fixture = read(raw_fixture_path)
raw_frames = {row['source_ns']: row for row in raw_fixture['frames']}
assert len(raw_frames) == 57
last_pair = raw_fixture['frames'][-1]
phase_findings = []
for expected_phase, record in enumerate(matrix['phases']):
    assert record['phase'] == expected_phase
    path = Path(record['result'])
    assert sha(path) == record['sha256']
    result = read(path)
    assert result['passed'] and result['error'] is None
    assert result['source_unchanged'] and result['task_finish_returned']
    assert result['qualified_handoff_only_passed']
    assert result['script_sha256'] == '46dbfc3c9b704a8c9f2f016956a02a67cd3003948fbab24f998f6b7bd397ffa1'
    assert result['raw_fixture_sha256'] == sha(raw_fixture_path)
    assert result['no_ros_master'] and result['motion_commands_sent'] is False
    for source, expected in result['source_sha256'].items():
        assert sha(source) == expected
    for module, record2 in result['loaded_dependency_sha256'].items():
        assert sha(record2['path']) == record2['sha256'], module
    for source, expected in result['helper_sha256'].items():
        assert sha(source) == expected
    timer = result['timer_phase']
    assert timer['predeclared_values'] == [0, 1, 2] and timer['selected'] == expected_phase
    assert timer['period_ns'] == round(1e9 / 6.)
    assert timer['offset_ns'] == round(expected_phase * timer['period_ns'] / 3)
    assert timer['selection_independent_of_frame_results']
    concurrent = result['concurrency']
    assert not concurrent['worker_errors'] and concurrent['all_threads_joined']
    assert concurrent['task_station_admission_actual'] and concurrent['full_original_57_pair_time_window']
    assert result['final_first_loss'] is None and not result['shutdown_pending_transport_events']
    coverage = result['input_coverage']
    assert coverage['producer_pairs_enqueued'] == coverage['original_rgbd_pair_count'] == 57
    assert coverage['last_producer_pair_source_ns'] == last_pair['source_ns']
    assert coverage['remote_raw_callbacks_completed']['color'] == coverage['remote_raw_callbacks_completed']['depth'] == 57
    assert coverage['producer_accepted_enqueued'] == coverage['remote_accepted_enqueued'] == coverage['task_accepted_enqueued']
    assert coverage['task_sdk_commands_enqueued'] > 0
    for item in result['remote_input_journal']:
        assert item['original_received_ns'] <= item['replay_callback_ns'] <= item['completion_ns']
    callbacks = [row for row in result['event_journal'] if 'channel' in row]
    for item in callbacks:
        assert item['due_ns'] <= item['enqueued_ns'] <= item['start_ns'] <= item['completion_ns']
        assert item['error'] is None
    channels = {row['channel'] for row in callbacks}
    assert {'producer_sdk', 'remote_accepted_sdk', 'task_accepted_sdk', 'task_sdk_command',
            'remote_feedback_timer', 'task_station_timer', 'task_status', 'task_mask'} <= channels
    assert result['shutdown_drain']
    assert all(row['unfinished_callbacks'] == row['queued_transport_events'] == 0
               for row in result['shutdown_drain'])
    assert result['clock_monotonicity']['every_clock_return_asserted_nondecreasing']
    end_ns = result['clock_monotonicity']['last_return_ns']
    assert end_ns >= last_pair['pair_available_ns']
    assert all(row['real_condition_released_lock'] and 0 < row['requested_sec'] <= .02
               and row['start_ns'] <= row['completion_ns'] for row in result['task_waits'])
    receipt = result['task_receipt']
    request = receipt['request']
    sources = request['source_stamps_ns']
    assert len(sources) == len(set(sources)) == 3 and sources == sorted(sources)
    assert request['state'] == 'qualified'
    assert request['arrival_stamp_ns'] == raw_fixture['arrival_ns']
    assert request['capture_deadline_ns'] == raw_fixture['capture_deadline_ns']
    assert request['capture_deadline_ns'] - request['arrival_stamp_ns'] == 3_000_000_000
    assert request['stamp_ns'] < request['capture_deadline_ns']
    qualifying = [ack for ack in result['acks'] if ack['state'] == 'requalifying' and ack['qualified']]
    assert {ack['participant'] for ack in qualifying} == {'unknown', 'remote'}
    assert len(qualifying) == 2
    assert all(ack['accepted'] and ack['source_stamps_ns'] == sources for ack in qualifying)
    assert {ack['participant'] for ack in receipt['acks']} == {'unknown', 'remote'}
    assert all(ack['accepted'] and ack['request_sha256'] == digest(request) for ack in receipt['acks'])
    assert len(result['actual_task_phases']) == 1
    phase = result['actual_task_phases'][0]
    assert phase['active'] and phase['planning_stage'] == 'contact_replan'
    assert phase['reference_source_stamp'] == sources[-1]
    assert phase['header']['stamp'] < request['capture_deadline_ns']
    assert phase['header']['stamp'] < phase['deadline'] <= request['deadline_ns']
    ready = []
    for row in result['rows']:
        assert row['source_ns'] in raw_frames
        age = (row['completion_ns'] - row['source_ns']) * 1e-9
        assert abs(age - row['source_age_sec']) < 1e-12
        assert row['processing_start_ns'] >= raw_frames[row['source_ns']]['pair_available_ns']
        if row['status']:
            assert row['status']['target_lost'] is False and row['status'].get('first_loss') is None
        if row['state'] == 'ready':
            assert 0 <= age < .8
            if row['detail'] == 'tracked':
                ready.append(row)
    assert len(ready) == 5
    assert any(row['source_ns'] > sources[-1] for row in ready)
    fusion = result['fusion']
    assert fusion['phase'] == 'observing_contact' and fusion['paused'] is False
    assert not any(fusion[key] for key in ('task_station_retired', 'task_target_loss',
                                         'task_support_fault', 'task_support_overflow'))
    contract = fusion['producer_registration_contract']
    assert contract == fusion['remote_registration_contract'] == fusion['task_registration_contract']
    assert contract['source_ns'] == ready[-1]['source_ns']
    assert contract['maximum_frame_age_sec'] == .8
    assert contract['geometry_deadline_ns'] - contract['source_ns'] == 800_000_000
    assert end_ns < contract['geometry_deadline_ns']
    assert contract['binding']['phase_reference_source_ns'] == sources[-1]
    assert contract['binding']['lease_id'] == request['lease_id']
    assert contract['metric_geometry_authority'] is False and contract['motion_authorized'] is False
    registration = fusion['registration']
    assert registration['result']['ok'] and registration['result']['code'] == 'REGISTERED'
    assert registration['stamp_ns'] == sources[-1]
    gain = fusion['information_gain']
    assert gain['ok'] and gain['registration_ok']
    assert gain['new_occupied_voxel_count'] >= gain['minimum_novel_voxels'] == 3
    assert gain['contact_or_motion_authorized'] is False
    assert fusion['surface_source_stamps_ns'] == [1791860167684837341, sources[-1]]
    fusion_callbacks = [row for row in callbacks if row['kind'] == 'remote_task_phase']
    assert any(row['processing_start_ns'] < other['completion_ns']
               and row['completion_ns'] > other['start_ns']
               for row in ready for other in fusion_callbacks)
    phase_findings.append(dict(phase=expected_phase, result=str(path), sha256=sha(path),
        qualified_sources=sources, qualification_completed_after_arrival_sec=(request['stamp_ns'] - request['arrival_stamp_ns'])*1e-9,
        max_processed_age_sec=max(row['source_age_sec'] for row in result['rows']),
        tracked_ready_count=len(ready), max_ready_age_sec=max(row['source_age_sec'] for row in ready),
        final_contract_source_ns=contract['source_ns'], final_contract_age_sec=(end_ns-contract['source_ns'])*1e-9,
        new_registered_voxels=gain['new_occupied_voxel_count'], raw_pairs_completed=57,
        three_actual_contracts_identical=True, no_loss_or_worker_fault=True,
        real_fusion_and_producer_processing_overlap=True))

assert len(phase_findings) == 3
for relative, expected in prior['source_sha256'].items():
    assert sha(W / relative) == expected, relative
assert sha(DELIVERY) == EXPECTED_DELIVERY_SHA
evidence[str(prior_path)] = sha(prior_path)
evidence[str(xml_path)] = sha(xml_path)
evidence[str(DELIVERY)] = sha(DELIVERY)
evidence[str(Path(__file__).resolve())] = sha(__file__)
evidence[str(freeze_path)] = sha(freeze_path)
for path, value in freeze['new_original_message_fixtures'].items():
    evidence[path] = value['sha256']
evidence[str(F / 'v6_scheduler_infrastructure_r3/result.json')] = sha(F / 'v6_scheduler_infrastructure_r3/result.json')

result = dict(schema=1, kind='gen14_common_face_independent_delivery_review',
    passed=True, delivery=str(DELIVERY), delivery_sha256=EXPECTED_DELIVERY_SHA,
    recorded_ns=time.time_ns(), candidate_root=str(C), test_scope=delivery['test_scope'],
    focused_scope=delivery['focused_scope'], source_sha256=candidate,
    base_sha256=base, actual_baseline_source_count=892,
    actual_baseline_unchanged_before_and_after=True,
    candidate_files=16, production_files=6, focused_test_files=35, integrated_test_files=161,
    combined_regression_summary=combo['summary'], combined_bound_source_files=325,
    frozen_replay_source_count=204, exact_original_message_files_verified=raw_message_count,
    replay_phases=phase_findings, evidence_sha256=evidence,
    review_findings=[
        'All sixteen delivered files match the current candidate and the 1069-test combined result; all actual 892 baseline source hashes match before and after this read-only review.',
        'The actual worktree integration scope is the exact deduplicated union of the 139-file prior scope and the 35-file candidate scope: 161 files.',
        'All three predeclared v6 timer phases pass on unchanged frozen sources. Each completes three actual source qualifications, real dual qualified requalifying ACKs, task completion/phase, registered new measured coverage and five later tracked READY publications.',
        'Remote completion-state ACK intentionally reports accepted=true, qualified=false, empty sources under the unchanged protocol. Independent checks separately verified both qualified requalifying ACKs and the completion ACK request digests; these meanings are not conflated.',
        'All three final producer/remote/task contracts are identical, source-bound and still within the original 0.8-second geometry lifetime at the final clock. All 57 original raw RGBD pairs reach producer and remote callbacks in each run.',
        'Callback timestamps obey original due lower bounds, producer and remote fusion actually overlap, task conditions release their locks, all threads join, and no target loss, positive support fault, overflow or worker error occurs.',
        'Static memo and source-cache review preserved mutable-value comparisons, owner/SDK checks, original checkpoints, both TF lookups and original source/capture/phase gates. No hardware, temperature, actuation or collision settings are changed.'
    ],
    approval_scope='Hash-bound installation of this candidate followed by the full actual 161-file offline regression. Runtime deployment and physical execution still require their existing release/readiness checks.',
    installed=False, deployed=False, actual_integrated_regression_passed=False,
    physical_grasp_verified=False, no_tests_reexecuted=True, no_ros_master_contact=True,
    motion_commands_sent=False, temperature_queried=False,
    limitations=['Offline traffic replay and synthetic/regression tests do not establish live ROS scheduling, candidate planning feasibility or physical grasp success.',
                 'The historical task contact phase is configured at 90 seconds. The capture gate remains 3 seconds, every original source gate remains 0.8 seconds, and three consecutive observations remain required.',
                 'No model RPC or physical action was run. Actual installation, full integrated regression, release seal, deployment and fresh scene/recording readiness are outstanding.'])
OUTPUT.mkdir(exist_ok=False)
destination = OUTPUT / 'result.json'
destination.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
print(json.dumps(dict(path=str(destination), sha256=sha(destination), passed=True,
                     files=16, focused_tests=1069, integrated_test_files=161,
                     replay_phases_passed=len(phase_findings)), ensure_ascii=False))
