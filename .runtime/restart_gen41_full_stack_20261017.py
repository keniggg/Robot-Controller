"""One explicitly requested cold start of the sealed GEN41 stack, in manual mode.

Prepare is read-only except for its SanStand receipt. Start rechecks that receipt,
claims the session once and uses the existing frozen startup entry. No grasp is
started here. The existing driver startup includes its positive enable request.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import time
import subprocess

W = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('gen41_restart_primitives',
    W / '.runtime/start_gen18_full_restart_20261013.py')
legacy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(legacy)
CONTROL = legacy.LOG_ROOT / 'full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/control.json'
TEXT = '你重新启动各节点和上位机，继续处理没完成的任务'
SOURCES = ('.runtime/restart_gen41_full_stack_20261017.py',
    '.runtime/verify_gen41_full_restart_20261017.py',
    '.runtime/start_full_latest_gen36_20261016.sh',
    '.runtime/full_latest_gen36_single_observation_20261016.launch',
    '.runtime/start_gen18_full_restart_20261013.py',
    'src/alicia_grasp_modes/config/minimal_observation_chain.yaml')


def preflight(session, release_path):
    storage = legacy.storage_check(session)
    request = legacy.read(session / 'operator_restart_instruction.json')
    legacy.require(request.get('source') == 'actual_user_message'
        and request.get('user_message') == TEXT and request.get('full_restart_authorized') is True,
        'actual restart instruction missing')
    control = legacy.read(CONTROL)
    legacy.require(control['state'] == 'paused' and control['revision'] == 47,
        'saved hardware pause changed before manual startup')
    release = legacy.verify_release(release_path)
    legacy.require(legacy.sha(release_path) == 'b98c25da01b77582bff4567a791afed8b8c908fcfe391b3b84601276c9ed706d'
        and release['source_count'] == 1032 and release['test_summary']['tests'] == 8168,
        'unexpected GEN41 release')
    legacy.parameter_source_check()
    serial = legacy.serial_check()
    legacy.idle_process_check()
    planned, template = legacy.commands(session)
    for component, row in planned.items():
        old = row['unit']
        row['unit'] = old.replace('alicia-gen18-', 'alicia-gen41-restart-', 1)
        row['command'] = [part.replace(old, row['unit']) for part in row['command']]
        if component == 'core':
            legacy.require(row['command'][-1] == str(W / '.runtime/start_full_latest_frozen_two_observations_20261011.sh'),
                'unexpected core entry')
            row['command'][-1] = str(W / '.runtime/start_full_latest_gen36_20261016.sh')
    return dict(storage=storage, serial=serial, control=control,
        source_manifest=str(release_path), source_manifest_sha256=legacy.sha(release_path),
        test_summary=release['test_summary'], command_template=template,
        planned_components=planned, operational_source_sha256={p:legacy.sha(W/p) for p in SOURCES},
        user_instruction_sha256=legacy.sha(session/'operator_restart_instruction.json'))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session', type=Path, required=True)
    p.add_argument('--release', type=Path, required=True)
    p.add_argument('--start', action='store_true')
    a = p.parse_args()
    session, release_path = a.session.resolve(strict=True), a.release.resolve(strict=True)
    plan = preflight(session, release_path)
    plan_path = session/'startup_plan.json'
    if not a.start:
        with plan_path.open('x') as f:
            json.dump(dict(plan, prepared_ns=time.time_ns()), f, ensure_ascii=False, indent=2)
        print(json.dumps(dict(preflight_passed=True, plan=str(plan_path))))
        return
    saved = legacy.read(plan_path)
    for key in ('control', 'source_manifest_sha256', 'planned_components',
                'operational_source_sha256', 'user_instruction_sha256'):
        legacy.require(plan[key] == saved[key], 'prepared startup changed: '+key)
    with (session/'cold_start_claim.json').open('x') as f:
        json.dump(dict(claimed_ns=time.time_ns(), helper_sha256=legacy.sha(__file__)), f)
    current = dict(plan, directory=str(session), launch_attempted=True, full_restart=True,
        continuation_control_state=plan['control']['state'],
        continuation_control_revision=plan['control']['revision'],
        automatic_grasp_started=False, automatic_grasp_running=False,
        startup_positive_enable_configured=True, temperature_queries_sent=False,
        torque_disable_commands_sent=False, active_work='starting_manual_gen41_stack')
    legacy.save(session, current)
    for component in ('simulation', 'core'):
        legacy.verify_release(release_path)
        legacy.source_check(plan['operational_source_sha256'])
        legacy.require(legacy.read(CONTROL) == plan['control'], 'control changed before dispatch')
        row = dict(plan['planned_components'][component], requested_ns=time.time_ns(), dispatch_attempted=True)
        current[component] = row
        legacy.save(session, current)
        try:
            run = subprocess.run(row['command'], capture_output=True, text=True, timeout=30.)
            row.update(returncode=run.returncode, stdout=run.stdout, stderr=run.stderr)
        except Exception as exc:
            row.update(error=repr(exc), launch_state_uncertain=True)
            current['active_work'] = 'startup_dispatch_failed_no_automatic_retry'
            legacy.save(session, current)
            raise
        legacy.save(session, current)
        legacy.require(run.returncode == 0, 'startup failed: '+component)
        print(json.dumps(dict(component=component, unit=row['unit'], returncode=run.returncode)), flush=True)
    current['active_work'] = 'waiting_for_full_restart_acceptance'
    legacy.save(session, current)


if __name__ == '__main__':
    main()
