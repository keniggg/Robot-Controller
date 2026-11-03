#!/usr/bin/env python3
"""Verify all three predeclared once-only replay artifacts and input hashes."""
import hashlib
import json
from pathlib import Path

W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
F=N/'verification/gen14_arrival_independent'
C=W/'.runtime/gen15_qualified_phase_timestamp_20261013'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
freeze=F/'three_owner_v7_r1_source_freeze.json'
f=json.loads(freeze.read_text());changed=[]
for filename,record in f['sources'].items():
    path=Path(filename)
    if not path.exists() or sha(path)!=record['sha256']:changed.append(filename)
assert not changed,changed
rows=[]
for phase in f['predeclared_phases']:
    path=F/('three_owner_v7_phase%d_r1/result.json'%phase)
    d=json.loads(path.read_text());c=d['task_reached_continuation'];fusion=d['fusion']
    assert d['passed'] and d['error'] is None and d['source_unchanged']
    assert d['timer_phase']['selected']==phase and d['timer_phase']['predeclared_values']==[0,1,2]
    assert d['script_sha256']==f['script_sha256']
    assert d['helper_sha256'][str(W/'.runtime/gen15_actual_reached_continuation_20261013.py')]==f['helper_sha256']
    for filename,digest in d['source_sha256'].items():
        assert f['sources'][str(Path(filename).resolve())]['sha256']==digest
    task_base=C/'src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py'
    task_mixin=C/'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_task.py'
    assert d['source_sha256'][str(task_base)]==f['sources'][str(task_base)]['sha256']
    actual=d['loaded_dependency_sha256']['alicia_grasp_modes.observe_return_task']
    assert Path(actual['path']).resolve()==task_mixin.resolve()
    assert actual['sha256']==f['sources'][str(task_mixin)]['sha256']
    assert c['successful_ensure_count']==2 and c['all_ensure_calls_same_owner'] and c['final_owner_unchanged']
    assert [x['method']for x in c['calls']]==['_ensure_qualified_station_phase','_validate_measured_observation_envelope','_wait_for_fresh_observation_camera_target_range','_ensure_qualified_station_phase']
    assert all(x.get('ok',True) and 'error'not in x for x in c['calls'])
    assert fusion['all_three_exact_contract_match'] and not fusion['task_station_retired']
    assert not any(fusion[k] for k in ('task_target_loss','task_support_fault','task_support_overflow'))
    assert d['final_first_loss'] is None and not d['concurrency']['worker_errors'] and d['concurrency']['all_threads_joined']
    assert d['input_coverage']['producer_pairs_enqueued']==57
    assert all(d['input_coverage']['remote_raw_callbacks_completed'][k]==57 for k in ('color','depth'))
    assert d['clock_monotonicity']['every_clock_return_asserted_nondecreasing']
    request=d['task_receipt']['request'];gain=fusion['information_gain']
    assert gain['ok'] and gain['registration_ok'] and not gain['contact_or_motion_authorized']
    assert not d['motion_commands_sent']
    ready=[r for r in d['rows']if r['state']=='ready']
    assert max(r['source_age_sec']for r in ready)<.8
    rows.append(dict(phase=phase,passed=True,result=str(path),sha256=sha(path),
        qualified_sources_ns=request['source_stamps_ns'],qualification_elapsed_sec=(request['stamp_ns']-request['arrival_stamp_ns'])*1e-9,
        ensure_count=c['successful_ensure_count'],exact_phase_owner=c['final_phase_owner'],
        original_phase_deadline_unchanged=c['final_owner_unchanged'],
        range_wait_sec=(c['calls'][2]['exit_ns']-c['calls'][2]['entry_ns'])*1e-9,
        measured_envelope_code=c['envelope']['code'],working_volume_range=c['range'],
        three_contracts_exact=True,full_rgbd_pairs=57,
        later_tracked_ready_count=sum(r['detail']=='tracked'for r in ready),
        max_ready_source_age_sec=max(r['source_age_sec']for r in ready),
        gain_new_occupied_voxels=gain['new_occupied_voxel_count'],registration_ok=True,
        first_loss=None,all_threads_joined=True,actual_task_base=str(task_base),actual_task_mixin=actual))
output=F/'three_owner_v7_r1_matrix_result.json';assert not output.exists()
result=dict(passed=True,phases=rows,predeclared_order=[0,1,2],attempts_per_phase=1,
    source_freeze=str(freeze),source_freeze_sha256=sha(freeze),all_frozen_files_unchanged=True,
    frozen_file_count=len(f['sources']),source_changes=changed,
    script_sha256=sha(Path(__file__)),
    scope='Three-owner source/ACK/fusion/registration/READY and actual task CAD/RGBD range/original reached-branch second ensure across the complete original 57-pair window. No remote candidate planning RPC, contact plan audit/execution, or physical grasp claim.',
    all_cpu_on_single_advancing_wall_clock=True,source_max_age_sec=.8,arrival_capture_budget_sec=3.,
    original_phase_budget_sec=90.,no_deadline_renewal=True,no_live_ros=True,motion_commands_sent=False)
output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(passed=True,matrix=str(output),sha256=sha(output),
    phases=[{k:r[k]for k in ('phase','ensure_count','later_tracked_ready_count','max_ready_source_age_sec','gain_new_occupied_voxels','sha256')}for r in rows]),ensure_ascii=False))
