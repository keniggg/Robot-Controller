"""Prepare/apply evidence-bound GEN15 metadata reconciliation only.

Default is validation plus a printed preview. --apply is required for writes.
No ROS import, service, process signal, source edit, or test execution occurs.
Two files on different filesystems cannot be atomically renamed together;
both originals are backed up first, replacements share a transaction id, and
partial commit is rolled back only if the written file is still ours.
"""
import argparse
from copy import deepcopy
import fcntl
import hashlib
import json
import os
from pathlib import Path
import time

W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
V=N/'verification'
C=W/'.runtime/gen15_qualified_phase_timestamp_20261013'
TARGETS=(N/'current_progress.json', W/'.runtime/latest_full_ros_start.json')
COMPUTE={'/grasp_task_node','/remote_grasp6d_node','/unknown_tabletop_perception'}

def require(ok, message):
    if not ok: raise RuntimeError(message)

def sha_bytes(data): return hashlib.sha256(data).hexdigest()
def sha(path): return sha_bytes(Path(path).read_bytes())
def read(path): return json.loads(Path(path).read_text())
def resolve(path):
    path=Path(path)
    return path if path.is_absolute() else W/path

def bound(evidence, path):
    path=Path(path).resolve()
    matches=[expected for name,expected in evidence.items() if resolve(name).resolve()==path]
    require(matches and all(expected==sha(path) for expected in matches), 'Missing/mismatched evidence SHA: '+str(path))

def check_sources(sources):
    require(len(sources)==908,'Expected complete 908-source installed release')
    for name,expected in sources.items():
        path=resolve(name).resolve()
        require(path.is_relative_to(W) if hasattr(path,'is_relative_to') else str(path).startswith(str(W)+'/'), 'Source outside worktree')
        require(sha(path)==expected,'Installed source changed: '+name)

def process_identities(identities):
    checked={}
    require(COMPUTE <= set(identities),'Deployment missing compute process identity')
    for node,record in identities.items():
        pid=record['pid']; require(type(pid) is int and pid>0,'Invalid deployment PID')
        stat=Path('/proc/%d/stat'%pid).read_text().rsplit(')',1)[1].split()
        require(stat[0] != 'Z','Zombie runtime process: '+node)
        require(stat[19]==str(record['start_ticks']),'Runtime PID identity changed: '+node)
        require(int(stat[1])==record['parent_pid'],'Runtime parent identity changed: '+node)
        cmd=Path('/proc/%d/cmdline'%pid).read_bytes()
        require(bool(cmd),'Runtime command missing: '+node)
        checked[node]=dict(pid=pid,start_ticks=stat[19],parent_pid=int(stat[1]),cmdline_sha256=sha_bytes(cmd))
    return checked

def durable_replace(path, payload, transaction):
    temp=path.with_name(path.name+'.'+transaction+'.tmp')
    with temp.open('xb') as stream:
        stream.write(payload); stream.flush(); os.fsync(stream.fileno())
    os.replace(temp,path)
    fd=os.open(str(path.parent),os.O_RDONLY)
    try: os.fsync(fd)
    finally: os.close(fd)

def encode(value): return (json.dumps(value,ensure_ascii=False,indent=2)+'\n').encode()

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply',action='store_true')
    parser.add_argument('--install',type=Path,default=V/'gen15_phase_timestamp_integrated_install_r2/result.json')
    parser.add_argument('--regression',type=Path,default=V/'gen15_phase_timestamp_actual_integrated_r2/result.json')
    parser.add_argument('--release',type=Path,default=V/'gen15_phase_timestamp_release_manifest.json')
    parser.add_argument('--deployment',type=Path,default=N/'gen15_deployment.json')
    parser.add_argument('--expected-deployment-sha256',required=True)
    args=parser.parse_args()
    require(Path('/media/zhuyupei/SanStand').is_mount(),'SanStand unavailable; no fallback')
    require(sha(args.deployment)==args.expected_deployment_sha256,'Deployment SHA differs from explicitly selected evidence')
    dep=read(args.deployment); release=read(args.release)
    install=read(args.install); regression=read(args.regression)
    for label,data in [('install',install),('regression',regression),('release',release)]:
        require(data.get('passed') is True,label+' is not passed')
    require(release.get('reviewed') is True and release.get('installed') is True,'Release not reviewed/installed')
    require(str(release.get('kind','')).lower().startswith('gen15'),'Release label is not GEN15')
    sealed_path=resolve(release['sealed_source_manifest']); sealed=read(sealed_path)
    require(sha(sealed_path)==release['sealed_source_manifest_sha256'],'Original sealed source manifest SHA differs')
    for key in ('passed','reviewed','installed','source_sha256','test_summary','evidence_sha256'):
        require(sealed[key]==release[key],'Final release altered sealed evidence: '+key)
    require(release['legacy_sealer_kind']==sealed['kind'],'Legacy sealer label not preserved')
    summary=regression['summary']
    require(summary['tests']>0 and all(summary[k]==0 for k in ('failures','errors','skipped')),'Actual regression contains failures/errors/skips')
    require(regression.get('production_worktree_tested') is True and regression.get('source_unchanged') is True,'Regression does not prove unchanged actual worktree')
    sources=install['source_sha256']
    require(sources==regression['source_sha256']==release['source_sha256']==dep['source_sha256'],'Install/regression/release/deployment source identities differ')
    check_sources(sources)
    scope=sorted(set(install['test_scope']))
    require(len(scope)==165 and len(scope)==len(install['test_scope']),'Actual installed regression scope must contain exactly 165 unique files')
    # Actual runner stores selected paths in its argv, not a second test_scope.
    selected=sorted(str(resolve(p).resolve()) for p in scope)
    command_paths=sorted(str(resolve(arg).resolve()) for arg in regression['command']
        if isinstance(arg,str) and arg.endswith('.py') and not arg.startswith('--'))
    require(command_paths==selected,'Actual regression argv does not match all 165 installed tests')
    require(release.get('test_summary')==summary,'Release test summary differs from actual result')
    bound(release['evidence_sha256'],args.regression)
    bound(release['evidence_sha256'],args.install)
    for name,h in install['evidence_sha256'].items(): require(sha(resolve(name))==h,'Install evidence changed: '+name)
    require(dep.get('deployment_passed') is True and dep.get('whole_patch_deployed') is True
        and dep.get('compute_reloaded') is True and dep.get('hardware_processes_preserved') is True,
        'GEN15 deployment is incomplete')
    require(resolve(dep['source_manifest']).resolve()==args.release.resolve() and dep['source_manifest_sha256']==sha(args.release),'Deployment release binding differs')
    reload_path=resolve(dep['reload_evidence']); reload=read(reload_path)
    require(sha(reload_path)==dep['reload_sha256'],'Reload evidence SHA differs')
    require(reload.get('deployment_passed') is True and reload['source_sha256']==sources,'Reload did not verify the installed sources')
    require(dep['after']==reload['after'] and dep['preserved_after']==reload['preserved_after'],'Reload process identities differ from deployment')
    require(dep['preserved_before']==dep['preserved_after'],'Hardware identities changed during compute reload')
    identities=dict(dep['preserved_after'],**dep['after'])
    runtime=process_identities(identities)
    acknowledgements=[json.loads(value) if isinstance(value,str) else value for value in dep['reset_ack'].values()]
    require(len(acknowledgements)>=2 and all(row.get('ready') is True for row in acknowledgements),'Missing ready reset ACKs')
    selections=[{key:row[key] for key in ('mode','strategy','generation','stamp_ns')} for row in acknowledgements]
    require(all(row==selections[0] for row in selections),'Deployment selection ACKs disagree')
    selection=selections[0]
    delivery=C/'delivery_manifest.json'; review=V/'gen15_qualified_phase_delivery_review/result.json'
    # A supplemental fixture-only install may chain to the original GEN15
    # install. Its complete 908 source map, not an old candidate claim, is final.
    original=read(delivery); independent=read(review)
    require(original['passed'] is True and original['provisional'] is False,'Original GEN15 candidate was not final/passed')
    require(independent['passed'] is True and independent['delivery_sha256']==sha(delivery),'GEN15 candidate delivery review binding changed')
    require(sorted(original['test_scope'])==scope and sorted(independent['test_scope'])==scope,'Original candidate/review scope differs')
    for row in original['files']:
        require(sha(C/row['path'])==row['candidate_sha256']==sha(W/row['path']),
            'Original seven-file GEN15 content is no longer installed: '+row['path'])
    original_install_path=V/'gen15_phase_timestamp_integrated_install/result.json'
    ancestor=install;ancestor_path=args.install.resolve();seen=set()
    while ancestor_path!=original_install_path.resolve():
        require(ancestor_path not in seen,'Install evidence chain cycle');seen.add(ancestor_path)
        parent_path=resolve(ancestor['prior']).resolve()
        bound(ancestor['evidence_sha256'],parent_path)
        ancestor=read(parent_path);ancestor_path=parent_path
        require(ancestor.get('passed') is True,'Install ancestor not passed')
    bound(ancestor['evidence_sha256'],delivery);bound(ancestor['evidence_sha256'],review)
    focused=V/'gen15_owner_focused_r1/result.json'; focused_data=read(focused)
    replay=V/'gen14_arrival_independent/three_owner_v7_r1_matrix_result.json'; replay_data=read(replay)
    diagnosis=V/'gen15_qualified_owner_diagnosis/result.json'
    bound(original['evidence_sha256'],focused)
    require(original['focused_regression']['path']==str(focused)
        and original['focused_regression']['sha256']==sha(focused),'Focused regression binding differs')
    require(focused_data.get('passed') is True and focused_data.get('source_unchanged') is True
        and all(focused_data['summary'][k]==0 for k in ('failures','errors','skipped')),'Original focused candidate evidence is not passed')
    require(replay_data['passed'] is True and replay_data['all_frozen_files_unchanged'] is True,'Three-owner replay is not passed')
    require(replay_data['predeclared_order']==[0,1,2] and replay_data['attempts_per_phase']==1,'Replay fixed phase coverage differs')
    require(sha(replay)==original['evidence_sha256'][str(replay)],'Candidate replay binding changed')
    require(sha(diagnosis)==original['evidence_sha256'][str(diagnosis)],'Recorded diagnosis binding changed')
    failed_actual_path=V/'gen15_phase_timestamp_actual_integrated/result.json'
    failed_actual=read(failed_actual_path)
    require(failed_actual.get('passed') is False and failed_actual['summary']['failures']>0,
        'Original failed actual regression fact changed')
    prior_failed_actual=dict(path=str(failed_actual_path),sha256=sha(failed_actual_path),
        summary=failed_actual['summary'],scope='Historical first GEN15 actual regression; fixture defect, retained unchanged')
    # Preserve the physical failure; reconciliation may only precede a new run.
    before={str(p):p.read_bytes() for p in TARGETS}
    states={name:json.loads(payload) for name,payload in before.items()}
    for name,state in states.items():
        require(state.get('directory')==str(N),'Metadata belongs to another session: '+name)
        require(not state.get('automatic_grasp_running') and not state.get('start_reserved')
            and state.get('active_record') is None,'A grasp started; do not replace its progress')
        require(state.get('last_result')=='failure' and state.get('physical_grasp_verified') is False,'Expected retained R3 failure changed')
    require(len({s['last_record'] for s in states.values()})==1,'Metadata last-record histories disagree')
    transaction='gen15_runtime_progress_'+str(time.time_ns())
    audit_dir=V/transaction
    verified_ns=time.time_ns()
    common=dict(active_work='gen15_deployed_fresh_grasp_preflight_pending',
        current_regression=str(args.regression.parent), current_test_result=str(args.regression),
        current_test_result_sha256=sha(args.regression), current_regression_summary=summary,
        current_test_summary=summary,current_test_passed=True,current_test_returncode=0,current_regression_passed=True,
        current_regression_elapsed_sec=regression['elapsed_sec'],current_regression_state='passed',
        current_test_scope_files=len(scope),pending_combined_test_scope_files=0,
        installed_repair=str(args.install),installed_repair_sha256=sha(args.install),
        source_manifest=str(args.release),source_manifest_sha256=sha(args.release),
        runtime_source_manifest=str(args.release),runtime_source_manifest_sha256=sha(args.release),
        verified_disk_release_manifest=str(args.release),verified_disk_release_manifest_sha256=sha(args.release),
        sealed_source_manifest=str(sealed_path),sealed_source_manifest_sha256=sha(sealed_path),
        disk_source_evidence=str(args.install),disk_source_evidence_sha256=sha(args.install),
        source_count=908,current_source_count=908,disk_source_count=908,runtime_source_count=908,
        operational_source_sha256=sources,disk_source_sha256=sources,
        runtime_matches_disk=True,runtime_reload_pending=False,runtime_status='gen15_compute_loaded_hardware_preserved',
        installed_but_not_loaded_patch=None,
        installed_but_not_loaded_patch_sha256=None,whole_patch_deployed=True,compute_reloaded=True,
        hardware_processes_preserved=True,deployment_passed=True, startup_runtime_verified=True,
        deployment=str(args.deployment),current_deployment=str(args.deployment),latest_deployment=str(args.deployment),
        deployment_sha256=sha(args.deployment),runtime_receipt=str(args.deployment),runtime_receipt_sha256=sha(args.deployment),
        compute_unit=dep['unit'],reload_evidence=str(reload_path),reload_evidence_sha256=sha(reload_path),
        deployed_node_identities=identities,runtime_identity_verified_ns=verified_ns,
        runtime_identity_verification='Read-only /proc PID/start_ticks/parent plus bound successful reload receipt; snapshot, not continuous health monitoring.',
        selection=selection,current_selection=selection,current_generation=selection['generation'],
        expected_generation=selection['generation'],next_generation=selection['generation']+1,
        selection_evidence=str(args.deployment),selection_evidence_sha256=sha(args.deployment),
        selection_is_live_snapshot=False,selection_scope='Selection agreed in deployment reset ACKs; fresh runtime preflight still required.',
        staged_algorithm_candidate=str(C),candidate_delivery=str(delivery),candidate_delivery_sha256=sha(delivery),
        candidate_installed=True,staged_candidate_installed=True,
        staged_candidate_test_result=str(focused),staged_candidate_test_result_sha256=sha(focused),
        staged_candidate_test_count=focused_data['summary']['tests'],
        staged_candidate_test_scope_note='Original seven-file candidate focused evidence. Final actual installed sources and any supplemental fixture-only install are bound by current install/release/regression.',
        three_owner_replay_passed=True,three_owner_replay_result=str(replay),three_owner_replay_result_sha256=sha(replay),
        latest_three_owner_replay=str(replay),latest_three_owner_replay_sha256=sha(replay),
        latest_three_owner_budget_receipt=str(replay),latest_three_owner_budget_receipt_sha256=sha(replay),
        recorded_phase_budget_sec=90.0,recorded_capture_budget_sec=3.0,
        root_cause_confirmed=True,deterministic_phase_start_comparison_failure_confirmed=True,
        root_cause_evidence=str(diagnosis),root_cause_evidence_sha256=sha(diagnosis),
        failure_stage_confirmed='Historical R3: real three-frame qualification and dual ACK succeeded; unchanged phase was falsely rejected during second ensure because timestamp second representations differed by one ULP.',
        failure_stage_evidence=str(diagnosis),failure_stage_evidence_sha256=sha(diagnosis),
        algorithm_root_cause_status='Recorded R3 start-cache mismatch repaired and software-verified; adjacent legal deadline roundtrip defect also covered. Physical grasp success remains unverified.',
        metadata_scope='GEN15 installed 908-source bundle passed the selected actual 165-file regression and bound compute reload. R3 remains a sealed physical failure; software validation is not grasp success.',
        source_manifest_scope='Current reviewed GEN15 source bundle; disk SHA and runtime reload receipt agree.',
        source_count_scope='All four counts describe the same complete 908-file release bundle, including tests/resources; they do not claim every file is loaded into a process.',
        deployment_scope='Bound GEN15 compute-only reload; hardware identities preserved. Runtime identity is a reconciliation-time snapshot.',
        next_action='Complete fresh runtime/recording/scene preflight, then continue the authorized recorded grasp task.',
        pending_work=['Fresh runtime, recording and scene preflight','Recorded physical grasp; preserve each real result'],
        last_completed_regression=dict(path=str(args.regression),sha256=sha(args.regression),summary=summary,test_scope_files=len(scope)),
        current_record=None,active_record=None,automatic_grasp_running=False,start_reserved=False,
        manual_state_is_live_snapshot=False,manual_mode_scope='Last recorded manual evidence only; metadata reconciliation does not query or change operator control.',
        physical_grasp_success=False,physical_grasp_verified=False,
        updated_ns=verified_ns,metadata_reconciliation_id=transaction,metadata_update_audit=str(audit_dir/'result.json'))
    historical_only={'proposal_diagnostic','proposal_diagnostic_sha256','train_only_diagnostic_result',
        'train_only_diagnostic_all_frames','train_only_diagnostic_dense_passes','train_only_diagnostic_is_qualification',
        'wire_diagnosis_evidence','wire_diagnosis_evidence_sha256','work_before_user_pause','attempt_scene',
        'resume_scene','resume_observed_epoch_ns','manual_restore_receipt','gui_manual_restored',
        'fresh_probe_pose_scope','failed_reload_evidence','failed_reload_evidence_sha256'}
    after={}; changes={}
    for name,state in states.items():
        value=deepcopy(state)
        moved={k:value.pop(k) for k in historical_only if k in value}
        prior_fields={k:deepcopy(state[k]) for k in common if k in state and state[k]!=common[k]
            and k not in ('operational_source_sha256','disk_source_sha256')}
        history=value.setdefault('metadata_history',[])
        require(isinstance(history,list),'Metadata history schema changed')
        history.append(dict(recorded_ns=verified_ns,reason='Replace mixed GEN13/GEN14 current claims after bound GEN15 deployment',
            backup=str(audit_dir/(Path(name).name+'.before')),backup_sha256=sha_bytes(before[name]),
            prior_current_fields=prior_fields,historical_fields=moved,
            original_failed_gen15_actual_regression=prior_failed_actual))
        value.update(common)
        # Preserve actual last-result, authorization and user pause/resume facts.
        require(all(value.get(k)==state.get(k) for k in ('last_record','last_result','last_failure_reason',
            'continuation_authorization','paused','paused_by_user','continuation_control_revision','continuation_state')),
            'Reconciliation changed historical result or authorization')
        after[name]=encode(value)
        changes[name]=sorted(k for k in set(state)|set(value) if state.get(k)!=value.get(k))
    preview=dict(apply=args.apply,transaction=transaction,changed_fields=changes,source_count=908,
        actual_summary=summary,test_scope_files=165,deployment_sha256=sha(args.deployment),runtime_identity_count=len(runtime),
        metadata_targets=list(before),backup_directory=str(audit_dir),physical_grasp_verified=False)
    if not args.apply:
        print(json.dumps(preview,ensure_ascii=False,indent=2));return
    # This lock coordinates this reconciler; CAS also detects unrelated writers.
    lock_path=N/'.gen15_metadata_reconciliation.lock'
    with lock_path.open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        require(all(Path(p).read_bytes()==payload for p,payload in before.items()),'Metadata changed during verification; retry from fresh evidence')
        check_sources(sources); process_identities(identities)
        require(sha(args.deployment)==args.expected_deployment_sha256,'Deployment changed before metadata commit')
        audit_dir.mkdir(exist_ok=False)
        for name,payload in before.items():
            backup=audit_dir/(Path(name).name+'.before')
            with backup.open('xb') as stream:stream.write(payload);stream.flush();os.fsync(stream.fileno())
        record=dict(preview,status='prepared',passed=False,script_sha256=sha(Path(__file__)),
            evidence_sha256={str(p):sha(p) for p in (args.install,args.regression,args.release,sealed_path,args.deployment,reload_path,delivery,review,focused,replay,diagnosis,failed_actual_path)},
            runtime_identities=runtime,before_sha256={p:sha_bytes(b) for p,b in before.items()},
            after_sha256={p:sha_bytes(b) for p,b in after.items()},motion_commands_sent=False,temperature_queried=False)
        receipt=audit_dir/'result.json';durable_replace(receipt,encode(record),transaction)
        committed=[]
        try:
            for name,payload in after.items():
                require(Path(name).read_bytes()==before[name],'Metadata changed before replacement: '+name)
                durable_replace(Path(name),payload,transaction);committed.append(name)
            require(all(Path(p).read_bytes()==payload for p,payload in after.items()),'Post-write metadata verification failed')
            record.update(status='committed',passed=True,committed_ns=time.time_ns())
        except BaseException as exc:
            restored=[];conflicts=[];rollback_errors={}
            # A replace may succeed before its directory fsync fails. Inspect
            # both targets, rather than relying on the completed-call list.
            for name in reversed(list(after)):
                try:
                    current=Path(name).read_bytes()
                    if current==after[name]:
                        durable_replace(Path(name),before[name],transaction+'-rollback');restored.append(name)
                    elif current!=before[name]:conflicts.append(name)
                except BaseException as rollback_exc:
                    rollback_errors[name]=repr(rollback_exc)
            incomplete=bool(conflicts or rollback_errors)
            record.update(status='partial_commit_conflict' if incomplete else 'rolled_back',
                error=repr(exc),restored=restored,conflicts=conflicts,rollback_errors=rollback_errors,
                manual_reconciliation_required=incomplete)
            durable_replace(receipt,encode(record),transaction+'-failed');raise
        durable_replace(receipt,encode(record),transaction+'-committed')
    print(json.dumps(dict(passed=True,receipt=str(receipt),sha256=sha(receipt),transaction=transaction),ensure_ascii=False))

if __name__=='__main__':main()
