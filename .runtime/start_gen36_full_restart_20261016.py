"""Start the reviewed GEN36 full stack once, with one view and manual mode."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

W = Path(__file__).resolve().parents[1]
legacy_path = W/'.runtime/start_gen18_full_restart_20261013.py'
spec = importlib.util.spec_from_file_location('reviewed_cold_start_primitives', legacy_path)
legacy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(legacy)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', required=True, type=Path)
    parser.add_argument('--release', required=True, type=Path)
    args = parser.parse_args()
    session, release_path = args.session.resolve(), args.release.resolve(strict=True)
    storage = legacy.storage_check(session)
    request = legacy.read(session/'operator_restart_instruction.json')
    assert request['source'] == 'actual_user_message' and request['full_restart_authorized'] is True
    assert request['user_message'] == '各节点和上位机你都要重新启动'
    assert request['automatic_grasp_authorized_by_this_request'] is False
    control_path = legacy.LOG_ROOT/'full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/control.json'
    control = legacy.read(control_path)
    assert control['state'] == 'paused' and control['revision'] == 41
    resumed = legacy.read(session/'resume_after_pause41.json')
    assert resumed['source'] == 'actual_user_message' and resumed['user_message'] == '继续处理'
    assert resumed['control_state'] == 'paused' and resumed['control_revision'] == 41
    assert resumed['motion_authorization_resumed'] is False
    current = legacy.read(session/'startup.json') if (session/'startup.json').exists() else {}
    assert not any(current.get(k) for k in ('core', 'simulation', 'launch_attempted'))
    release = legacy.verify_release(release_path)
    legacy.parameter_source_check()
    own_sources = {name: legacy.sha(W/name) for name in (
        '.runtime/start_gen36_full_restart_20261016.py',
        '.runtime/start_full_latest_gen36_20261016.sh',
        '.runtime/full_latest_gen36_single_observation_20261016.launch',
        '.runtime/start_gen18_full_restart_20261013.py',
        'src/alicia_grasp_modes/config/minimal_observation_chain.yaml')}
    # Helper source hashes must be part of the independently reviewed seal.
    for name, digest in own_sources.items():
        assert release['evidence_sha256'].get(str(W/name)) == digest
    root = ET.parse(W/'.runtime/full_latest_gen36_single_observation_20261016.launch').getroot()
    assert root.find('include').get('file') == str(W/'.runtime/full_latest_no_skin_20261010.launch')
    load = root.find('rosparam')
    assert load.attrib == dict(command='load', file=str(W/'src/alicia_grasp_modes/config/minimal_observation_chain.yaml'), ns='/grasp')
    assert root.find('param').attrib == dict(name='/gui/joint_direct_mode', type='bool', value='true')
    serial = legacy.serial_check()
    legacy.idle_process_check()
    planned, template = legacy.commands(session)
    for component, row in planned.items():
        old_unit = row['unit']
        row['unit'] = old_unit.replace('alicia-gen18-', 'alicia-gen36-', 1)
        row['command'] = [part.replace(old_unit, row['unit']) for part in row['command']]
        if component == 'core':
            assert row['command'][-2:] == ['/bin/bash', str(W/'.runtime/start_full_latest_frozen_two_observations_20261011.sh')]
            row['command'][-1] = str(W/'.runtime/start_full_latest_gen36_20261016.sh')
    legacy.storage_check(session)
    with (session/'cold_start_claim.json').open('x') as f:
        json.dump(dict(claimed_ns=time.time_ns(), script_sha256=legacy.sha(__file__)), f)
    current.update(directory=str(session), source_manifest=str(release_path),
        source_manifest_sha256=legacy.sha(release_path), test_summary=release['test_summary'],
        startup_dependencies_sha256=dict(legacy.DEPENDENCIES, **own_sources),
        startup_script_sha256=legacy.sha(__file__), command_template=template, storage=storage,
        serial=serial, full_restart=True, pre_start_existing_nodes=[], pre_start_listener_ports=[],
        pre_start_checked_ns=time.time_ns(), automatic_grasp_running=False,
        automatic_grasp_started=False, start_tactile=False,
        software_temperature_monitoring_enabled=False, temperature_poll_rate_hz=0.0,
        temperature_queries_sent=False, torque_disable_commands_sent=False,
        startup_positive_enable_configured=True, launch_attempted=True,
        continuation_control_state=control['state'], continuation_control_revision=control['revision'],
        active_work='starting_reviewed_gen36_manual_single_view_stack', planned_components=planned)
    legacy.save(session, current)
    for component in ('simulation', 'core'):
        # Revalidate all bytes and the paused control before each launch.
        legacy.verify_release(release_path)
        legacy.source_check(own_sources)
        assert legacy.read(control_path) == control
        row = dict(planned[component], requested_ns=time.time_ns(), dispatch_attempted=True)
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
        assert run.returncode == 0, component
        print(json.dumps(dict(component=component, **row), ensure_ascii=False), flush=True)
    current['active_work'] = 'waiting_for_gen36_full_stack_acceptance'
    legacy.save(session, current)


if __name__ == '__main__':
    main()
