"""Prepare or explicitly dispatch the installed GEN58 full cold stack once.

Run from the sourced ROS/worktree environment. The default only writes the
reviewable plan and GET-only dependency diagnostic. --start dispatches the two
reviewed systemd services, in manual mode. It never starts a grasp or sends a
temperature query, negative enable, home or trajectory command.
"""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time

sys.dont_write_bytecode = True
from gen58_full_cold_restart_common_20261021 import (
    W, CONTROL_SHA, REQUEST, REQUEST_SHA, ENTRY, LAUNCH, CHECKER, sha, read,
    require, new_json, control_and_request, configuration, standalone_health,
    CONTINUE_REQUEST, CONTINUE_REQUEST_SHA, SESSION, release_scope, stopped_reference)
import gen58_cold_install_guard_20261021 as guard

spec = importlib.util.spec_from_file_location('gen57_full_start_primitives',
    W/'.runtime/start_gen18_full_restart_20261013.py')
legacy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(legacy)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', required=True, type=Path)
    parser.add_argument('--release', required=True, type=Path)
    parser.add_argument('--release-sha256', required=True)
    parser.add_argument('--stop-receipt', required=True, type=Path)
    parser.add_argument('--stop-receipt-sha256', required=True)
    parser.add_argument('--start', action='store_true')
    args = parser.parse_args()
    session, release_path = args.session.resolve(strict=True), args.release.resolve(strict=True)
    require(session == SESSION, 'GEN58 must use its new independently sealed startup session')
    legacy.storage_check(session)
    os.environ['ROS_LOG_DIR'] = str(session/'configuration_ros')
    control, request = control_and_request()
    require(sha(release_path) == args.release_sha256, 'release digest changed')
    release = legacy.verify_release(release_path)
    release_scope(release_path,release)
    transition_path = args.stop_receipt.resolve(strict=True)
    transition = stopped_reference(transition_path,args.stop_receipt_sha256)
    require(release['generation'] == 'GEN58' and
        release['source_count'] == len(release['source_sha256']), 'wrong generation/source scope')
    legacy.parameter_source_check()
    serial = legacy.serial_check()
    cold = guard.cold_pair()
    config = configuration()
    config_path = session/'startup_configuration.json'
    request_path = session/'operator_restart_instruction.json'
    mapped_request = dict(source=request['source'],user_message=request['text'],
        full_restart_authorized=True,source_request=str(REQUEST),source_request_sha256=REQUEST_SHA,
        scope=request['scope'],derived_format_only=True,
        continuation_request=str(CONTINUE_REQUEST),continuation_request_sha256=CONTINUE_REQUEST_SHA,
        new_scene_confirmation_claimed=False)
    if not args.start:
        require(not (session/'startup_plan.json').exists() and not (session/'cold_start_claim.json').exists(),
            'startup plan/claim already exists; no implicit retry')
        new_json(request_path,mapped_request)
        new_json(config_path,config)
    else:
        require(read(request_path) == mapped_request and read(config_path) == config,
            'prepared startup request/configuration changed')
    planned, template = legacy.commands(session)
    for component,row in planned.items():
        old = row['unit']; row['unit'] = old.replace('alicia-gen18-','alicia-gen58-cold-',1)
        row['command'] = [part.replace(old,row['unit']) for part in row['command']]
        if component == 'core':
            row['command'][-1] = str(ENTRY)
    operational = (
        Path(__file__).resolve(), W/'.runtime/gen58_full_cold_restart_common_20261021.py',
        W/'.runtime/verify_gen58_full_cold_restart_20261021.py',
        W/'.runtime/gen58_cold_install_guard_20261021.py',
        W/'.runtime/start_gen18_full_restart_20261013.py',
        W/'.runtime/verify_validated_full_restart_20261019.py', ENTRY,LAUNCH,CHECKER)
    sources = {str(p):sha(p) for p in operational}
    sources.update(config['launch_source_sha256'])
    plan = dict(storage=legacy.storage_check(session),serial=serial,control=control,
        control_sha256=CONTROL_SHA,generation='GEN58',full_restart=True,
        startup_positive_enable_configured=True,automatic_grasp_started=False,
        source_manifest=str(release_path),source_manifest_sha256=args.release_sha256,
        test_summary=release['test_summary'],command_template=template,
        planned_components=planned,operational_source_sha256=sources,
        user_instruction_sha256=sha(request_path),actual_user_instruction=str(REQUEST),
        actual_user_instruction_sha256=REQUEST_SHA,startup_configuration=str(config_path),
        continuation_request=str(CONTINUE_REQUEST),continuation_request_sha256=CONTINUE_REQUEST_SHA,
        stop_receipt=str(transition_path),stop_receipt_sha256=args.stop_receipt_sha256,
        before_restart_epoch_ns=transition['before']['epoch_ns'],
        before_restart_nodes=transition['before']['node_identities'],
        startup_configuration_sha256=sha(config_path),cold_software_absence=cold,
        boot_id=cold['after']['boot_id'],expected_persistent_nodes=config['persistent_nodes'],
        one_shot_nodes=config['one_shot_nodes'],physical_grasp_verified=False,
        scene_permission='not_granted_by_cold_start')
    if not args.start:
        diagnostic = standalone_health(config,session/'prestart_remote_health')
        plan['prestart_remote_health'] = dict(path=str(session/'prestart_remote_health/result.json'),
            sha256=sha(session/'prestart_remote_health/result.json'),passed=diagnostic['passed'],
            current_node_dependency_ready=False)
        new_json(session/'startup_plan.json',dict(plan,prepared_ns=time.time_ns()))
        require(diagnostic['passed'], 'configured remote dependency is not ready; plan retained, no dispatch')
        print(json.dumps(dict(preflight_passed=True,plan=str(session/'startup_plan.json'),
            configuration=str(config_path),request_sha256=sha(request_path),
            persistent_node_count=len(config['persistent_nodes']),automatic_grasp_started=False)))
        return
    saved = read(session/'startup_plan.json')
    for key in ('control','control_sha256','generation','source_manifest_sha256','planned_components',
                'operational_source_sha256','user_instruction_sha256','actual_user_instruction_sha256',
                'startup_configuration_sha256','boot_id','expected_persistent_nodes','one_shot_nodes',
                'continuation_request','continuation_request_sha256','stop_receipt','stop_receipt_sha256',
                'before_restart_epoch_ns','before_restart_nodes'):
        require(plan[key] == saved[key], 'prepared startup changed: '+key)
    diagnostic = standalone_health(config,session/'dispatch_remote_health')
    require(diagnostic['passed'], 'configured remote dependency is not ready; no dispatch')
    plan['dispatch_remote_health'] = dict(path=str(session/'dispatch_remote_health/result.json'),
        sha256=sha(session/'dispatch_remote_health/result.json'),current_node_dependency_ready=False)
    require(guard.cold_pair()['after']['boot_id'] == plan['boot_id'], 'cold host changed before claim')
    new_json(session/'cold_start_claim.json',dict(claimed_ns=time.time_ns(),helper_sha256=sha(__file__),
        boot_id=plan['boot_id'],pid=os.getpid()))
    current = dict(plan,directory=str(session),launch_attempted=True,
        continuation_control_state=control['state'],continuation_control_revision=control['revision'],
        temperature_queries_sent=False,torque_disable_commands_sent=False,
        active_work='starting_installed_GEN58_in_manual_mode')
    legacy.save(session,current)
    for component in ('simulation','core'):
        legacy.verify_release(release_path)
        release_scope(release_path,read(release_path))
        require(sha(release_path) == args.release_sha256 and control_and_request()[0] == control,
            'release/actual control changed')
        require(stopped_reference(transition_path,args.stop_receipt_sha256) == transition,
            'pre-stop measured reference changed')
        require(all(sha(p) == h for p,h in sources.items()) and
            sha(config_path) == plan['startup_configuration_sha256'] and
            configuration() == config and sha(request_path) == plan['user_instruction_sha256'],
            'startup configuration/source changed')
        row = dict(planned[component],requested_ns=time.time_ns(),dispatch_attempted=True)
        current[component] = row; legacy.save(session,current)
        try:
            run = subprocess.run(row['command'],capture_output=True,text=True,timeout=30.)
            row.update(returncode=run.returncode,stdout=run.stdout,stderr=run.stderr)
        except BaseException as exc:
            row.update(error=repr(exc),launch_state_uncertain=True)
            current['active_work'] = 'dispatch_failed_no_automatic_retry'
            legacy.save(session,current); raise
        legacy.save(session,current)
        require(run.returncode == 0,'startup failed: '+component)
        print(json.dumps(dict(component=component,unit=row['unit'],returncode=run.returncode)),flush=True)
    current['active_work'] = 'waiting_for_full_restart_and_remote_acceptance'
    legacy.save(session,current)


if __name__ == '__main__':
    main()
