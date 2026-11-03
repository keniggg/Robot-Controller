"""One camera configuration revision; preserve the running manual control stack."""
import argparse
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
F = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1')
BASE = F / 'depth_stage_online_aba_r1'
OUT = F / 'depth_baseline_deployment_r2'
PROFILE = W / '.runtime/full_latest_no_skin_20261010.launch'
PARAM = '/camera/mode_depth_preset/unknown_preset'
HELPER = W / '.runtime/depth_stage_camera_session_20261022.py'


def ref(path):
    return dict(path=str(path), sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest())


def replace_profile(expected_sha, content):
    temporary = PROFILE.with_name(PROFILE.name + '.revision-' + str(time.time_ns()))
    try:
        with temporary.open('xb') as stream:
            stream.write(content); stream.flush(); os.fsync(stream.fileno())
        os.chmod(temporary, PROFILE.stat().st_mode & 0o777)
        assert ref(PROFILE)['sha256'] == expected_sha, 'concurrent profile edit; no overwrite'
        os.replace(temporary, PROFILE)
    finally:
        if temporary.exists(): temporary.unlink()


def save_recovery_evidence(s, path, value):
    try: s.save(path, value)
    except Exception as exc:
        print('Recovery evidence write failed: ' + repr(exc) + '\n' + json.dumps(value),
              file=sys.stderr, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=('prepare', 'apply'))
    parser.add_argument('--sha256')
    args = parser.parse_args()
    assert Path('/media/zhuyupei/SanStand').is_mount(), 'SanStand unavailable'
    spec = importlib.util.spec_from_file_location('camera_revision_helpers', HELPER)
    s = importlib.util.module_from_spec(spec); spec.loader.exec_module(s)
    s.OUT = OUT
    observer = F / ('depth_baseline_observer_' + args.operation + '_' + str(time.time_ns()))
    observer.mkdir(); os.environ['ROS_LOG_DIR'] = str(observer / 'ros')
    monitor = s.Monitor()
    try:
        if args.operation == 'prepare':
            old = json.loads((BASE / 'plan.json').read_text())
            assert s.digest(BASE / 'plan.json') == '064a4379a797983a54965caf83c63750426090e58f60769bf9dc37ecc5e3ff60'
            restored = s.read_ref(dict(path=str(BASE / 'restored/registered.json'),
                sha256='847b7923659e834fbda6d18226d1748b43da21dde7062a394fc0ce6ce14fcd43'))
            for path, digest in old['sources'].items():
                s.require(s.digest(path) == digest, 'camera source changed: ' + path)
            s.preserved(old, monitor)
            current = monitor.node('/supervisor_camera')
            s.require(current == restored['new_camera'], 'current camera identity changed')
            intent = json.loads((BASE / 'restored/launch_intent.json').read_text())
            s.require(s.unit_pid(intent['unit']) == current, 'restored camera unit changed')
            s.require_device_owner(old, current['pid'])
            cfg = monitor.master.getParam('/camera')
            s.require(s.configuration_contract(cfg) == old['configuration_contract'], 'camera configuration changed')
            before = monitor.sample()
            s.require(before['selection']['mode'] == 'unknown', 'unknown task mode required')
            source = PROFILE.read_bytes()
            needle = b'unknown_preset: high_accuracy'
            s.require(source.count(needle) == 1, 'persistent preset field changed')
            desired = source.replace(needle, b'unknown_preset: saved_baseline')
            OUT.mkdir()
            (OUT / 'profile_before.launch').write_bytes(source)
            (OUT / 'profile_desired.launch').write_bytes(desired)
            plan = copy.deepcopy(old)
            plan.update(schema=3, kind='production_camera_preset_revision', output=str(OUT),
                old_camera=current, old_camera_uri=monitor.master.lookupNode('/supervisor_camera'),
                old_camera_unit=intent['unit'], before=before, diagnostic_trials=[],
                configuration_before=s.configuration_contract(cfg), camera_configuration=copy.deepcopy(cfg),
                persistent_profile=dict(path=str(PROFILE), before=ref(OUT / 'profile_before.launch'),
                    desired=ref(OUT / 'profile_desired.launch')),
                revision_inputs={name:ref(BASE / name) for name in ('plan.json', 'result.json',
                    'restored/registered.json', 'production_ring_fit_r1/result.json',
                    'production_support_replay_r1/result.json')},
                previous_configuration='high_accuracy', desired_configuration='saved_baseline',
                depth_quality_or_grasp_success_claimed=False)
            plan['sources'][str(Path(__file__).resolve())] = s.digest(__file__)
            cfg = plan['camera_configuration']
            cfg['mode_depth_preset']['unknown_preset'] = 'saved_baseline'
            cfg['runtime_profile']['mode_depth_preset'].update(
                configured_unknown_preset='saved_baseline', preset='saved_baseline')
            plan['configuration_contract'] = s.configuration_contract(cfg)
            digest = s.save(OUT / 'plan.json', plan)
            check = subprocess.run([plan['interpreter'], '-B', str(s.ENTRY), '--production',
                '--preflight-only', '--plan', str(OUT / 'plan.json'), '--sha256', digest],
                capture_output=True, text=True, timeout=20.)
            s.save(OUT / 'production_preflight.json', dict(returncode=check.returncode,
                stdout=check.stdout, stderr=check.stderr))
            s.require(check.returncode == 0, 'production import preflight failed; camera untouched')
            print(json.dumps(dict(prepared=True, plan=ref(OUT / 'plan.json'), current_camera=current)), flush=True)
            return

        s.require(args.sha256 and s.digest(OUT / 'plan.json') == args.sha256, 'revision plan changed')
        plan = json.loads((OUT / 'plan.json').read_text())
        for path, digest in plan['sources'].items():
            s.require(s.digest(path) == digest, 'camera source changed: ' + path)
        for reference in plan['revision_inputs'].values(): s.read_ref(reference)
        profile = plan['persistent_profile']
        for name in ('before', 'desired'):
            s.require(s.digest(profile[name]['path']) == profile[name]['sha256'], 'profile snapshot changed')
        s.require(s.digest(PROFILE) == profile['before']['sha256'], 'persistent profile changed')
        s.require(json.loads((OUT / 'production_preflight.json').read_text())['returncode'] == 0,
                  'production preflight incomplete')
        state = s.validate_control(plan, monitor)
        s.require(monitor.node('/supervisor_camera') == plan['old_camera'], 'camera identity changed')
        s.require(s.unit_pid(plan['old_camera_unit']) == plan['old_camera'], 'camera service changed')
        s.require(s.configuration_contract(monitor.master.getParam('/camera')) == plan['configuration_before'],
                  'camera configuration changed')
        s.require_device_owner(plan, plan['old_camera']['pid'])
        s.save(OUT / 'before_stop.json', dict(state=state, camera=plan['old_camera']))
        started = time.monotonic(); stopped = False; parameter_changed = False; profile_changed = False
        known = {plan['old_camera_uri']}
        try:
            stopped = True
            s.stop_camera(plan['old_camera'])
            s.require_camera_absent(monitor, known); s.require_device_owner(plan)
            s.validate_control(plan, monitor)
            parameter_changed = True
            monitor.master.setParam(PARAM, 'saved_baseline')
            s.require(monitor.master.getParam(PARAM) == 'saved_baseline', 'preset parameter write failed')
            s.launch(plan, True)
            ready = s.wait_ready(plan, monitor, 'restored', known)
            s.require(ready['state']['diagnostic_reference_preserved'], 'control reference changed during reload')
            ready['strict_control_recheck'] = s.validate_control(plan, monitor)
            s.require(s.digest(PROFILE) == profile['before']['sha256'], 'persistent profile concurrently changed')
            profile_changed = True
            replace_profile(profile['before']['sha256'], Path(profile['desired']['path']).read_bytes())
            s.require(s.digest(PROFILE) == profile['desired']['sha256'], 'persistent profile readback failed')
            s.save(OUT / 'result.json', dict(passed=True, configuration_revision=True,
                production=ready, plan=ref(OUT / 'plan.json'), persistent_profile=ref(PROFILE),
                non_camera_instances_preserved=len(plan['preserved']),
                control_reference_preserved=ready['state']['diagnostic_reference_preserved'],
                wall_elapsed_sec=time.monotonic()-started, motion_commands_sent=False,
                mode_changes_sent=False, temperature_queried=False, full_stack_restart=False,
                original_geometric_and_execution_gates_preserved=True,
                full_perception_or_grasp_passed=None))
            print(json.dumps(dict(passed=True, result=ref(OUT / 'result.json'),
                camera=ready['new_camera'], seconds=time.monotonic()-started)), flush=True)
        except BaseException as exc:
            save_recovery_evidence(s, OUT / 'failure.json', dict(error=repr(exc), recorded_ns=time.time_ns()))
            try:
                if stopped and not s.alive(plan['old_camera']):
                    if (OUT / 'restored/launch_intent.json').exists():
                        active = s.owned_unit_camera('restored')
                        if active is not None:
                            try: known.add(monitor.master.lookupNode('/supervisor_camera'))
                            except Exception: pass
                        s.stop_probe_unit('restored')
                    s.require_camera_absent(monitor, known); s.require_device_owner(plan)
                    if parameter_changed:
                        value = monitor.master.getParam(PARAM)
                        s.require(value in ('high_accuracy', 'saved_baseline'), 'concurrent preset edit; no overwrite')
                        if value == 'saved_baseline': monitor.master.setParam(PARAM, 'high_accuracy')
                    if profile_changed:
                        actual = s.digest(PROFILE)
                        s.require(actual in (profile['before']['sha256'], profile['desired']['sha256']),
                                  'concurrent profile edit; no overwrite')
                        if actual == profile['desired']['sha256']:
                            replace_profile(actual, Path(profile['before']['path']).read_bytes())
                    rollback = copy.deepcopy(plan)
                    rollback.update(output=str(OUT / 'rollback'), configuration_contract=plan['configuration_before'])
                    s.OUT = OUT / 'rollback'; s.OUT.mkdir()
                    s.save(s.OUT / 'plan.json', rollback)
                    s.preserved(rollback, monitor)
                    s.launch(rollback, True)
                    restored = s.wait_ready(rollback, monitor, 'restored', known)
                    save_recovery_evidence(s, OUT / 'rollback_result.json', restored)
                else:
                    save_recovery_evidence(s, OUT / 'rollback_not_started.json',
                        dict(original_camera_alive=s.alive(plan['old_camera']), stopped=stopped))
            except BaseException as rollback_error:
                save_recovery_evidence(s, OUT / 'rollback_failure.json', dict(
                    deployment_error=repr(exc), rollback_error=repr(rollback_error),
                    original_camera_alive=s.alive(plan['old_camera']),
                    current_camera=s.current_camera(monitor), root_takeover_required=True))
            raise
    finally:
        monitor.close()


if __name__ == '__main__':
    def interrupted(_signal, _frame):
        raise KeyboardInterrupt('camera preset revision interrupted')
    signal.signal(signal.SIGTERM, interrupted)
    main()
