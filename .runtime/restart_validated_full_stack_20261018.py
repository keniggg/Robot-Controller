"""Cold start an installed, sealed release after an actual restart request."""
import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
import time

W = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('restart_primitives', W/'.runtime/start_gen18_full_restart_20261013.py')
legacy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(legacy)
CONTROL = legacy.LOG_ROOT/'full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/control.json'


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session', type=Path, required=True)
    p.add_argument('--release', type=Path, required=True)
    p.add_argument('--release-sha256', required=True)
    p.add_argument('--expected-control-revision', type=int, required=True)
    p.add_argument('--start', action='store_true')
    a = p.parse_args()
    session, release_path = a.session.resolve(strict=True), a.release.resolve(strict=True)
    legacy.storage_check(session)
    request = legacy.read(session/'operator_restart_instruction.json')
    legacy.require(request.get('source') == 'actual_user_message' and
        request.get('user_message') == '你先重启各节点和上位机，继续处理任务' and
        request.get('full_restart_authorized') is True, 'actual restart instruction required')
    control = legacy.read(CONTROL)
    legacy.require(control['state'] == 'active' and control['revision'] == a.expected_control_revision,
        'continuation control changed')
    legacy.require(legacy.sha(release_path) == a.release_sha256, 'release digest changed')
    release = legacy.verify_release(release_path)
    legacy.parameter_source_check()
    serial = legacy.serial_check()
    legacy.idle_process_check()
    planned, template = legacy.commands(session)
    for component, row in planned.items():
        old = row['unit']; row['unit'] = old.replace('alicia-gen18-', 'alicia-validated-restart-', 1)
        row['command'] = [part.replace(old, row['unit']) for part in row['command']]
        if component == 'core':
            row['command'][-1] = str(W/'.runtime/start_full_latest_gen36_20261016.sh')
    sources = {str(W/name):legacy.sha(W/name) for name in (
        '.runtime/restart_validated_full_stack_20261018.py',
        '.runtime/verify_validated_full_restart_20261018.py',
        '.runtime/start_full_latest_gen36_20261016.sh',
        '.runtime/full_latest_gen36_single_observation_20261016.launch')}
    plan = dict(storage=legacy.storage_check(session), serial=serial, control=control,
        source_manifest=str(release_path), source_manifest_sha256=a.release_sha256,
        test_summary=release['test_summary'], command_template=template,
        planned_components=planned, operational_source_sha256=sources,
        user_instruction_sha256=legacy.sha(session/'operator_restart_instruction.json'))
    plan_path = session/'startup_plan.json'
    if not a.start:
        with plan_path.open('x') as stream: json.dump(dict(plan, prepared_ns=time.time_ns()),stream,ensure_ascii=False,indent=2)
        print(json.dumps(dict(preflight_passed=True, plan=str(plan_path))))
        return
    saved = legacy.read(plan_path)
    for key in ('control','source_manifest_sha256','planned_components','operational_source_sha256','user_instruction_sha256'):
        legacy.require(plan[key] == saved[key], 'prepared startup changed: '+key)
    with (session/'cold_start_claim.json').open('x') as stream:
        json.dump(dict(claimed_ns=time.time_ns(), helper_sha256=legacy.sha(__file__)),stream)
    current = dict(plan, directory=str(session), full_restart=True, launch_attempted=True,
        continuation_control_state=control['state'], continuation_control_revision=control['revision'],
        automatic_grasp_started=False, startup_positive_enable_configured=True,
        temperature_queries_sent=False, torque_disable_commands_sent=False,
        active_work='starting_installed_validated_release_in_manual_mode')
    legacy.save(session,current)
    for component in ('simulation','core'):
        legacy.verify_release(release_path)
        legacy.require(legacy.sha(release_path) == a.release_sha256 and legacy.read(CONTROL) == control,'release/control changed')
        legacy.require(all(legacy.sha(path)==digest for path,digest in sources.items()),'startup source changed')
        row = dict(planned[component],requested_ns=time.time_ns(),dispatch_attempted=True)
        current[component]=row;legacy.save(session,current)
        try:
            run=subprocess.run(row['command'],capture_output=True,text=True,timeout=30.)
            row.update(returncode=run.returncode,stdout=run.stdout,stderr=run.stderr)
        except BaseException as exc:
            row.update(error=repr(exc),launch_state_uncertain=True);legacy.save(session,current);raise
        legacy.save(session,current)
        legacy.require(run.returncode==0,'startup failed: '+component)
        print(json.dumps(dict(component=component,unit=row['unit'],returncode=run.returncode)),flush=True)
    current['active_work']='waiting_for_full_restart_acceptance';legacy.save(session,current)

if __name__=='__main__':
    main()
