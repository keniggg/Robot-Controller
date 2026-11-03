"""Restore the original depth preset using the existing idle camera transaction."""
import copy
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import time

W = Path(__file__).resolve().parents[1]
F = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1')
R = F / 'actual_gen63_baseline_validation_20261023T041149Z'
BASE = R / 'current_depth_preset_aba_r1'
OUT = R / 'high_accuracy_restore_r1'
PROFILE = W / '.runtime/full_latest_no_skin_20261010.launch'
PARAM = '/camera/mode_depth_preset/unknown_preset'


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    OUT.mkdir()
    os.environ['ROS_LOG_DIR'] = str(OUT / 'controller_ros')
    s = module('camera_session', W / '.runtime/depth_stage_camera_session_20261022.py')
    atomic = module('existing_atomic_camera_profile', W / '.runtime/deploy_depth_baseline_20261022.py')
    s.OUT = OUT
    old = s.read_ref(dict(path=str(BASE / 'plan.json'),
        sha256='a9629d2d09721a72a4fbeff522edbe4dd9608c085238ac5ce1ac902ba312665a'))
    finished = s.read_ref(dict(path=str(BASE / 'result.json'),
        sha256='b7a12227cef0a7f96c160e0ea51539608cd460e016d9a864ca36002e28945589'))
    s.require(finished['passed'] and finished['production_configuration_restored'], 'A/B/A not restored')
    for path, sha in old['sources'].items():
        s.require(s.digest(path) == sha, 'camera source changed: ' + path)
    monitor = s.Monitor()
    try:
        camera = finished['restoration']['new_camera']
        s.require(monitor.node('/supervisor_camera') == camera, 'camera instance changed')
        intent = json.loads((BASE / 'restored/launch_intent.json').read_text())
        s.require(s.unit_pid(intent['unit']) == camera, 'camera service changed')
        s.preserved(old, monitor)
        s.require_device_owner(old, camera['pid'])
        cfg = monitor.master.getParam('/camera')
        s.require(s.configuration_contract(cfg) == old['configuration_contract'], 'camera configuration changed')
        before = monitor.sample()
        source = PROFILE.read_bytes()
        needle = b'unknown_preset: saved_baseline'
        s.require(source.count(needle) == 1, 'persistent preset not uniquely identified')
        desired = source.replace(needle, b'unknown_preset: high_accuracy')
        (OUT / 'profile_before.launch').write_bytes(source)
        (OUT / 'profile_desired.launch').write_bytes(desired)
        plan = copy.deepcopy(old)
        plan.update(kind='restore_original_depth_preset_after_controlled_comparison', output=str(OUT),
            old_camera=camera, old_camera_uri=monitor.master.lookupNode('/supervisor_camera'),
            old_camera_unit=intent['unit'], before=before, diagnostic_trials=[],
            configuration_before=s.configuration_contract(cfg), camera_configuration=copy.deepcopy(cfg),
            configuration_revision=True, previous_configuration='saved_baseline', desired_configuration='high_accuracy',
            persistent_profile=dict(path=str(PROFILE), before_sha256=s.digest(OUT / 'profile_before.launch'),
                desired_sha256=s.digest(OUT / 'profile_desired.launch')),
            comparison_source=dict(path=str(R / 'repaired_grasp_r2/support_stage_result.json'),
                sha256='e491b451c531241e42d74bee421b41aed3769bb6543700c83b74513d33d2ffc3'))
        s.read_ref(plan['comparison_source'])
        plan['camera_configuration']['mode_depth_preset']['unknown_preset'] = 'high_accuracy'
        plan['camera_configuration']['runtime_profile']['mode_depth_preset'].update(
            preset='high_accuracy', configured_unknown_preset='high_accuracy')
        plan['configuration_contract'] = s.configuration_contract(plan['camera_configuration'])
        plan['sources'][str(Path(__file__).resolve())] = s.digest(__file__)
        digest = s.save(OUT / 'plan.json', plan)
        check = subprocess.run([plan['interpreter'], '-B', str(s.ENTRY), '--production',
            '--preflight-only', '--plan', str(OUT / 'plan.json'), '--sha256', digest],
            capture_output=True, text=True, timeout=20)
        s.save(OUT / 'production_preflight.json', dict(returncode=check.returncode, stdout=check.stdout, stderr=check.stderr))
        s.require(check.returncode == 0, 'production import failed; original camera remains')
        state = s.validate_control(plan, monitor)
        s.require(s.digest(PROFILE) == plan['persistent_profile']['before_sha256'], 'profile changed')
        s.require(monitor.node('/supervisor_camera') == camera, 'camera changed before stop')
        s.save(OUT / 'before_stop.json', dict(state=state, camera=camera))
        started = time.monotonic()
        stopped = False
        known = {plan['old_camera_uri']}
        try:
            stopped = True
            s.stop_camera(camera)
            s.require_camera_absent(monitor, known)
            s.require_device_owner(plan)
            s.validate_control(plan, monitor)
            s.require(monitor.master.getParam(PARAM) == 'saved_baseline', 'concurrent preset change')
            monitor.master.setParam(PARAM, 'high_accuracy')
            s.launch(plan, True)
            ready = s.wait_ready(plan, monitor, 'restored', known)
            s.require(ready['state']['diagnostic_reference_preserved'], 'control reference changed')
            atomic.replace_profile(plan['persistent_profile']['before_sha256'], desired)
            s.require(s.digest(PROFILE) == plan['persistent_profile']['desired_sha256'], 'profile readback mismatch')
            s.save(OUT / 'result.json', dict(passed=True, production=ready,
                wall_elapsed_sec=time.monotonic()-started, non_camera_instances_preserved=len(plan['preserved']),
                control_reference_preserved=True, motion_commands_sent=False, mode_changes_sent=False,
                temperature_queried=False, full_stack_restart=False, physical_grasp_verified=False,
                original_geometric_and_execution_gates_preserved=True,
                current_preset='high_accuracy', persistent_profile_sha256=s.digest(PROFILE)))
            print(json.dumps(dict(passed=True, camera=ready['new_camera'], seconds=time.monotonic()-started)), flush=True)
        except BaseException as exc:
            s.save(OUT / 'failure.json', dict(error=repr(exc), recorded_ns=time.time_ns()))
            if stopped and not s.alive(camera):
                if (OUT / 'restored/launch_intent.json').exists():
                    row = s.owned_unit_camera('restored')
                    if row is not None:
                        known.add(monitor.master.lookupNode('/supervisor_camera'))
                    s.stop_probe_unit('restored')
                s.require_camera_absent(monitor, known)
                s.require_device_owner(plan)
                value = monitor.master.getParam(PARAM)
                s.require(value in ('saved_baseline', 'high_accuracy'), 'concurrent preset edit; no overwrite')
                monitor.master.setParam(PARAM, 'saved_baseline')
                actual = s.digest(PROFILE)
                s.require(actual in plan['persistent_profile'].values(), 'concurrent profile edit; no overwrite')
                if actual == plan['persistent_profile']['desired_sha256']:
                    atomic.replace_profile(actual, source)
                recovery = copy.deepcopy(plan)
                recovery.update(output=str(OUT / 'rollback'), configuration_contract=plan['configuration_before'])
                s.OUT = OUT / 'rollback'
                s.OUT.mkdir()
                s.save(s.OUT / 'plan.json', recovery)
                s.preserved(recovery, monitor)
                s.launch(recovery, True)
                restored = s.wait_ready(recovery, monitor, 'restored', known)
                s.save(OUT / 'rollback_result.json', restored)
            raise
    finally:
        monitor.close()


if __name__ == '__main__':
    def interrupted(_signal, _frame):
        raise KeyboardInterrupt('camera preset restoration interrupted')
    signal.signal(signal.SIGTERM, interrupted)
    main()
