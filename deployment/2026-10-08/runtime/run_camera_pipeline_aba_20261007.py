#!/usr/bin/env python3
"""One camera-only diagnostic followed by restoration; never sends robot commands."""
import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import time

W = Path(__file__).resolve().parents[1]
EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
CAMERA = '/supervisor_camera'
HARNESS = W/'.runtime/camera_pipeline_aba_20261007.py'
MAX_PROBE_CONSOLE = 128*1024


def load(name, relative):
    spec = importlib.util.spec_from_file_location(name, W/relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def camera_absent(rospy, h):
    try:
        pid = h.node_pid(rospy, CAMERA)
    except (h.CameraRegistrationPending, ConnectionRefusedError, ConnectionResetError):
        return True
    raise RuntimeError('another live camera owns ROS registration: '+str(pid))


def owned_probe_exit(process, identity, h):
    """Never signal any PID except the still-owned diagnostic child."""
    if process.poll() is not None:
        return
    for sig, wait in ((signal.SIGINT, 8.), (signal.SIGTERM, 5.), (signal.SIGKILL, 3.)):
        if process.poll() is not None:
            return
        if not h.alive(identity):
            raise RuntimeError('diagnostic process identity changed')
        os.kill(identity['pid'], sig)
        try:
            process.wait(timeout=wait)
        except subprocess.TimeoutExpired:
            pass
    if process.poll() is None:
        raise RuntimeError('diagnostic camera owner did not exit')


def drain_console(source, destination, state):
    """Nonblocking diagnostic output, with a separate explicit finite budget."""
    while True:
        try:
            chunk = source.read(16384)
        except BlockingIOError:
            return
        if not chunk:
            return
        remaining = MAX_PROBE_CONSOLE-state['bytes']
        destination.write(chunk[:remaining])
        state['bytes'] += min(len(chunk), remaining)
        if len(chunk) > remaining:
            raise RuntimeError('diagnostic console exceeded 128 KiB budget')


def stopped_camera_exited(owner, stop_issued, alive, *, clock=time.monotonic,
                          sleep=time.sleep, wait_seconds=12.):
    # After a sent SIGINT, a lost pose guard cannot make the old owner stop
    # disappearing. Wait for that already-requested exit without pose checks.
    if not alive(owner):
        return True
    if not stop_issued:
        return False
    deadline = clock()+wait_seconds
    while clock() < deadline:
        if not alive(owner):
            return True
        sleep(.05)
    return not alive(owner)


def run_with_restore(io):
    """Restoration is attempted after every probe outcome, once old owner exits."""
    primary = None
    try:
        io.prepare()
        io.stop_camera()
        io.probe()
    except BaseException as exc:
        primary = exc
        io.record_error('diagnostic', exc)
    finally:
        if io.old_camera_exited():
            try:
                io.close_probe()
                io.restore()
            except BaseException as exc:
                io.record_error('restoration', exc)
                if primary is None:
                    primary = exc
        io.finish()
    if primary is not None:
        raise primary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--stationary-receipt', required=True, type=Path)
    parser.add_argument('--validation-manifest', required=True, type=Path)
    parser.add_argument('--harness-sha256', required=True)
    args = parser.parse_args()
    if not args.execute:
        parser.error('explicit --execute required')
    assert EXT.parent.parent.is_mount(), 'SanStand unavailable; no fallback'
    assert sha(HARNESS) == args.harness_sha256, 'diagnostic source changed'
    m = load('aba_existing_loader', '.runtime/load_regional_production_20261006.py')
    h = m.load('aba_camera_owner', '.runtime/gen4_root_cause/reload_camera_visual_gate.py')
    ab = m.load('aba_feedback', '.runtime/camera_preset_ab_observer.py')
    alignment = m.load('aba_alignment_monitor', 'tools/diagnostics/prepare_rgb_acquisition_retry.py')
    manifest = json.loads(args.validation_manifest.read_text())
    m.verify_manifest(manifest)
    receipt = m.read_receipt(args.stationary_receipt)
    assert receipt['kind'] == 'explicit_user_stationary_capture'
    out = EXT/'ros_log'/('camera_pipeline_aba_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
    out.mkdir(); (out/'ros').mkdir()
    os.environ['ROS_LOG_DIR'] = str(out/'ros')
    import rospy
    rospy.init_node('camera_pipeline_aba_owner', anonymous=True, disable_signals=True)
    io = m.runtime_class(rospy, h, ab, alignment)(out, args.stationary_receipt.resolve(),
        receipt, manifest, 'preserve', args.validation_manifest.resolve(), False)
    environment = dict(os.environ, **h.camera_environment(io.old['pid']))
    environment['ROS_LOG_DIR'] = str(out/'probe_ros')
    environment['PYTHONDONTWRITEBYTECODE'] = '1'
    process = None
    probe_identity = None
    log = None
    console_state = dict(bytes=0)
    interruption = dict(requested=False, cleanup=False, signals=[])
    io.report.update(diagnostic_complete=False, camera_restored=False,
        harness_sha256=args.harness_sha256, supervisor_sha256=sha(__file__),
        stationary_user_receipt_scope='camera-only; no target or motion authority')

    class Transaction:
        def prepare(self):
            io.guard(force=True); io.verify()
            assert sha(HARNESS) == args.harness_sha256
            snapshot = io.ev.guard.snapshot()
            paths = list(h.SOURCES)+[HARNESS, Path(__file__).resolve(),
                W/'.runtime/camera_preset_ab_observer.py',
                W/'.runtime/camera_stage_observer.py',
                W/'.runtime/realsense-2.56.5/pyrealsense2.cpython-38-x86_64-linux-gnu.so',
                W/'.runtime/realsense-2.56.5/lib/librealsense2.so.2.56.5',
                Path(io.baseline['path'])]
            paths += sorted((W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision').glob('*.py'))
            contract = dict(schema=1, kind='camera_pipeline_aba_inner',
                camera_config=io.before, perception_config=rospy.get_param('/perception', {}),
                selection=io.ev.initial_context['selection'], epoch_ns=snapshot['epoch'],
                expected_feedback=snapshot, source_sha256={str(p):sha(p) for p in set(paths)},
                camera_diagnostic_only=True, motion_authorized=False,
                stationary_receipt_path=str(args.stationary_receipt),
                stationary_receipt_sha256=sha(args.stationary_receipt))
            (out/'contract.json').write_text(json.dumps(contract, indent=2)+'\n')
            harness = load('aba_contract_validation', '.runtime/camera_pipeline_aba_20261007.py')
            harness.validate_contract(contract); harness.verify_sources(contract)
            io.report['probe_contract_sha256'] = sha(out/'contract.json')
            io.save('aba_prepared')

        def stop_camera(self):
            io.stop_camera()

        def old_camera_exited(self):
            interruption['cleanup'] = True
            issued = io.report.get('old_camera_sigint_pid') == io.old['pid']
            exited = stopped_camera_exited(io.old, issued, h.alive)
            io.report['owned_camera_exit_after_cleanup_wait'] = exited
            io.report['camera_stop_exit_unconfirmed'] = issued and not exited
            return exited

        def probe(self):
            nonlocal process, probe_identity, log
            io.guard(force=True); camera_absent(rospy, h)
            command = [io.normal[0], str(HARNESS), '--enable-pipeline-aba',
                       '--contract', str(out/'contract.json'), '--output', str(out/'probe')]
            log = (out/'probe.stdout.log').open('xb')
            process = subprocess.Popen(command, cwd=str(W), env=environment,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True)
            os.set_blocking(process.stdout.fileno(), False)
            probe_identity = h.identity(process.pid)
            io.report['diagnostic_owner'] = probe_identity
            io.save('aba_probe_started')
            deadline = time.monotonic()+120
            while process.poll() is None:
                drain_console(process.stdout, log, console_state)
                io.guard()
                if time.monotonic() > deadline:
                    raise RuntimeError('diagnostic process exceeded 120 second supervision bound')
                time.sleep(.1)
            drain_console(process.stdout, log, console_state)
            io.report['diagnostic_returncode'] = process.returncode
            summary = m.completed_json(out/'probe/summary.json')
            io.report['diagnostic_summary'] = summary
            assert process.returncode == 0 and summary, 'diagnostic did not complete'
            assert summary.get('pipeline_closed') is True, 'diagnostic pipeline close not confirmed'
            assert summary.get('writer', {}).get('drained') is True, 'diagnostic queue not sealed'
            assert summary.get('complete') is True, 'diagnostic final completion not confirmed'
            assert sha(HARNESS) == args.harness_sha256
            io.report['diagnostic_complete'] = True
            io.save('aba_probe_completed')

        def close_probe(self):
            if process is not None:
                owned_probe_exit(process, probe_identity, h)
                io.report['diagnostic_final_returncode'] = process.returncode
            if log is not None:
                log.close()
            if process is not None and process.stdout is not None:
                process.stdout.close()
            io.report['diagnostic_console_bytes'] = console_state['bytes']
            io.report['diagnostic_console_max_bytes'] = MAX_PROBE_CONSOLE

        def restore(self):
            # Camera restoration is not a motion: changed manual pose does not
            # justify leaving the user without video. It never restores a target.
            assert not h.alive(io.old)
            assert process is None or process.poll() is not None
            camera_absent(rospy, h)
            io.verify()
            assert h.stable_camera_configuration(rospy.get_param('/camera')) == h.stable_camera_configuration(io.before)
            for name, owner in io.core.items():
                assert h.alive(owner) and h.node_pid(rospy, name) == owner['pid'], 'core owner changed: '+name
            result = subprocess.run(io.command, capture_output=True, text=True, timeout=10)
            (out/'systemd_restore.txt').write_text(result.stdout+result.stderr)
            assert result.returncode == 0, 'camera restore start failed'
            expected = io.command[io.command.index(io.normal[0]):]

            def registered():
                pid = int(subprocess.check_output(['systemctl', '--user', 'show', io.unit,
                    '--property=MainPID', '--value'], text=True, timeout=3).strip() or 0)
                if pid <= 0:
                    return False
                try:
                    if h.node_pid(rospy, CAMERA) != pid:
                        return False
                except (h.CameraRegistrationPending, ConnectionRefusedError):
                    return False
                identity = h.identity(pid)
                assert identity['argv'] == expected and identity['exe_sha256'] == io.old['exe_sha256']
                return identity

            io.new = h.wait(registered, 20., 'restored camera registration')
            io.report.update(camera_loaded=True, camera_restored=True, new_camera=io.new,
                             systemd_user_unit=io.unit)
            io.save('aba_camera_restored')
            io.validate()
            io.report['static_validation_passed'] = True
            io.save('aba_restored_video_validated')

        def record_error(self, stage, exc):
            io.report[stage+'_error'] = repr(exc)
            io.report['static_validation_passed'] = False
            io.save('aba_'+stage+'_failed')

        def finish(self):
            io.report['supervisor_interrupt_signals'] = list(interruption['signals'])
            io.close(); io.save('aba_finished')
            print(json.dumps(dict(out=str(out), diagnostic_complete=io.report['diagnostic_complete'],
                camera_restored=io.report['camera_restored'],
                video_validated=io.report['static_validation_passed'])), flush=True)

    def interrupted(sig, frame):
        interruption['signals'].append(int(sig))
        if interruption['cleanup'] or interruption['requested']:
            return
        interruption['requested'] = True
        raise RuntimeError('camera diagnostic interrupted by signal '+str(sig))
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, interrupted)
    run_with_restore(Transaction())


if __name__ == '__main__':
    main()
