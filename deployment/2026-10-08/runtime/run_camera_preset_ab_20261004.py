#!/usr/bin/env python3
"""One finite sensor comparison using the last verified visual owner identities."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import time
from datetime import datetime, timezone

WORK = Path(__file__).resolve().parents[1]
PREVIOUS = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/camera_preset_ab_20261004T144329.072192Z/result.json')
WRAPPER = WORK / '.runtime/camera_preset_ab_observer.py'
WRAPPER_SHA = '29e65066399fedf120462ebe9f9e43dea049c6f44d19a165ee17bf0b72ad6742'


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec); spec.loader.exec_module(result)
    return result


def complete_pair_is_valid(stamp, pair, camera, cutoff, now):
    if set(pair) != {'rgb', 'depth'} or not cutoff < stamp <= now or now-stamp > 800000000:
        return False
    activation = camera['runtime_profile']['geometry_activation_stamp_ns']
    if stamp <= int(activation): return False
    for kind, pixel_bytes, encodings in (('rgb', 3, ('bgr8', 'rgb8')), ('depth', 2, ('16UC1',))):
        row = pair[kind]
        if (row['caller'] != '/supervisor_camera' or row['frame_id'] != camera['frame_id']
                or row['encoding'] not in encodings
                or (row['width'], row['height']) != (camera['width'], camera['height'])
                or row['step'] < row['width']*pixel_bytes
                or row['bytes'] != row['step']*row['height']):
            return False
    return True


def main():
    d = module('visual_loader', WORK / '.runtime/load_pair_rectification_20261004.py')
    stale = module('dead_uri', WORK / '.runtime/visual_registration_20261004.py')
    manifest = json.loads(d.MANIFEST.read_text()); d.verify_manifest(manifest)
    assert d.sha(WRAPPER) == WRAPPER_SHA
    assert d.sha(WORK / '.runtime/visual_registration_20261004.py') == '25aae37c88a5af895ad1d57ade35c4e8736b3ab706b0f7a82f263fb8b9cb1b5f'
    previous = json.loads(PREVIOUS.read_text())
    assert previous.get('loaded') or previous.get('visual_nodes_restored')
    assert d.MOUNT.is_mount()
    # This is a new experiment baseline from the verified successful reload,
    # not a fabricated resumption of its earlier failed initial snapshot.
    d.EXPECTED_PIDS = {role: row['pid'] for role, row in previous['after'].items()}
    def retain_rectified(configuration):
        from alicia_flexible_grasp.vision.rgbd_rectification import rectification_contract
        contract = rectification_contract(configuration)
        assert contract is not None and configuration['pair_rectification']['enabled'] is True
        assert (configuration['stream_width'], configuration['stream_height']) == (640, 480)
        assert configuration['mode_depth_preset']['unknown_preset'] == 'high_accuracy'
        return d.deepcopy(configuration)
    d.enabled_configuration = retain_rectified  # Constructor validation only; no parameter write.
    out = d.EXT / 'ros_log' / ('camera_preset_ab_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'))
    out.mkdir(); (out / 'ros').mkdir()
    os.environ['ROS_LOG_DIR'] = str(out / 'ros')

    class Experiment(d.Runtime):
        def registered(self, role):
            return stale.registered_or_pending(self.ros, d.NODES[role], self.before[role], self.helper)

        def verify(self):
            super().verify()
            assert d.sha(WRAPPER) == WRAPPER_SHA
            assert d.sha(WORK / '.runtime/visual_registration_20261004.py') == '25aae37c88a5af895ad1d57ade35c4e8736b3ab706b0f7a82f263fb8b9cb1b5f'
            assert d.sha(WORK / '.runtime/camera_stage_observer.py') == '7e4d9e2c8f00b9f948d93a27528e6ac9e5fb068565b51f894770457c6c173676'

    runtime = Experiment(manifest, out)
    runtime.report.update(experiment_complete=False, experiment_valid=False,
                          previous_deployment=str(PREVIOUS), wrapper_sha256=WRAPPER_SHA)
    for role, expected in previous['after'].items():
        assert runtime.before[role]['start_ticks'] == expected['start_ticks'], 'visual owner reused'
    original_service_argv = d.service_argv
    def experimental_service(role, old, environment, directory, unit, allowed):
        cmd = original_service_argv(role, old, environment, directory, unit, allowed)
        if role == 'camera':
            offset = cmd.index('/usr/bin/python3')
            entry, remaps = cmd[offset+1], cmd[offset+2:]
            return cmd[:offset] + ['/usr/bin/python3', str(WRAPPER), '--enable-preset-ab',
                '--external-consumers-quiesced', '--camera-script', entry, '--output', str(out / 'stages'),
                '--seconds', '75', '--interval', '.5', '--'] + remaps
        return cmd

    summary_path = out / 'stages/summary.json'
    failure = None
    consumers_stopped = False
    camera_ready_for_consumers = False

    def restored_warmup():
        cutoff = runtime.ros.Time.now().to_nsec(); started = time.monotonic(); observed = set()
        def warm():
            camera = runtime.ros.get_param('/camera')
            if camera['runtime_profile']['mode_depth_preset']['preset'] != 'high_accuracy':
                raise RuntimeError('high_accuracy restoration disappeared')
            now = runtime.ros.Time.now().to_nsec()
            with runtime.lock:
                observed.update(ns for ns, pair in runtime.pairs.items()
                                if complete_pair_is_valid(ns, pair, camera, cutoff, now))
            return time.monotonic()-started >= 3. and len(observed) >= 10
        runtime.helper.wait(warm, 12., 'post-restore high_accuracy warmup', runtime.guard)
        runtime.validate_camera(True)
        runtime.report['post_restore_warmup'] = dict(duration_sec=time.monotonic()-started,
            distinct_complete_published_pairs=len(observed), after_stamp_ns=cutoff)

    def settle_ambiguous_camera_launch():
        current = runtime.current.get('camera')
        if current is not None:
            if runtime.helper.alive(current):
                return  # The ordinary stop path still verifies this live owner.
            runtime.verify()
            if runtime.registered('camera') is not None:
                raise RuntimeError('replacement camera registered after known owner exited')
            runtime.record('known_camera_owner_exited', pid=current['pid'],
                           start_ticks=current['start_ticks'])
            runtime.verify()
            if runtime.helper.alive(current) or runtime.registered('camera') is not None:
                raise RuntimeError('camera owner changed while confirming exit')
            # This exact PID/start was already obtained from our unit (or the
            # initial baseline). Restart=no means its confirmed exit is final.
            # Runtime.stop intentionally leaves dead identities untouched.
            runtime.current['camera'] = None
            return
        if 'camera' not in runtime.units:
            return
        runtime.verify()
        unit = runtime.units['camera']
        pid = int(subprocess.check_output(['systemctl', '--user', 'show', unit,
                  '--property=MainPID', '--value'], text=True, timeout=3).strip() or 0)
        if pid:
            identity = runtime.helper.identity(pid)
            event = next(e for e in reversed(runtime.report['events'])
                         if e['event'] == 'start_requested' and e['role'] == 'camera')
            expected = event['command'][event['command'].index('/usr/bin/python3'):]
            assert identity['argv'] == expected and identity['exe_sha256'] == runtime.before['camera']['exe_sha256']
            assert identity['cwd'] == runtime.before['camera']['cwd']
        # Stop exactly our recorded user unit, including a queued start with
        # MainPID=0. No second camera may start before its exit is proved.
        subprocess.run(['systemctl', '--user', 'stop', unit], check=True, timeout=15)
        if pid: runtime.helper.wait(lambda:not runtime.helper.alive(identity), 8., 'owned camera unit exit', runtime.guard)
        assert int(subprocess.check_output(['systemctl', '--user', 'show', unit,
            '--property=MainPID', '--value'], text=True, timeout=3).strip() or 0) == 0
        runtime.record('ambiguous_camera_unit_stopped', unit=unit, pid=pid)
    try:
        runtime.verify()
        consumers_stopped = True
        runtime.stop('unknown'); runtime.stop('remote'); runtime.stop('camera')
        d.service_argv = experimental_service
        runtime.start('camera')
        runtime.validate_camera(True)
        print(json.dumps(dict(phase='sensor_comparison_started', output=str(out))), flush=True)
        def complete():
            if not summary_path.exists(): return False
            try: return json.loads(summary_path.read_text())
            except json.JSONDecodeError: return False
        summary = runtime.helper.wait(complete, 85., 'finite A1/B/A2 summary', runtime.guard)
        runtime.report['sensor_summary'] = summary
        assert summary.get('restoration', {}).get('succeeded'), 'sensor restoration not proved'
        assert summary.get('original_read_restored') and summary.get('original_sync_restored')
        # Do not relabel an in-flight B read as A. Independently wait for new
        # complete published pairs after restoration, through a fresh warmup.
        restored_warmup()
        camera_ready_for_consumers = True
        for item in summary['files']:
            path = summary_path.parent / item['name']
            assert path.stat().st_size == item['bytes'] and d.sha(path) == item['sha256']
        size = sum(p.stat().st_size for p in summary_path.parent.iterdir() if p.is_file())
        assert size <= 159000000
        runtime.report.update(experiment_complete=True, experiment_valid=bool(summary.get('experiment_valid')),
            actual_capture_bytes=size)
    except BaseException as exc:
        failure = exc
        runtime.report['experiment_error'] = repr(exc)
        # Restore normal high_accuracy camera ownership if the diagnostic did
        # not prove completion/restoration. Driver/gateway remain untouched.
        if consumers_stopped:
            try:
                camera_ready_for_consumers = False
                runtime.verified_camera = runtime.verified_camera_settings = None
                settle_ambiguous_camera_launch()
                runtime.stop('camera')
                d.service_argv = original_service_argv
                runtime.start('camera'); runtime.validate_camera(True)
                restored_warmup()
                camera_ready_for_consumers = True
                runtime.report['normal_camera_recovered_after_error'] = True
            except BaseException as recovery:
                runtime.report['normal_camera_recovery_error'] = repr(recovery)
    finally:
        d.service_argv = original_service_argv
        try:
            runtime.verify()
            if consumers_stopped:
                if not camera_ready_for_consumers:
                    raise RuntimeError('normal camera not verified; consumers remain quiesced')
                for role in ('unknown', 'remote'):
                    if runtime.current.get(role) is None:
                        runtime.start(role)
            runtime.verify()
            for role in d.NODES:
                current = runtime.current[role]
                assert current and runtime.helper.alive(current) and runtime.registered(role) == current['pid']
            assert runtime.current['gateway'] == runtime.before['gateway']
            runtime.report.update(visual_nodes_restored=True, after=d.deepcopy(runtime.current))
        except BaseException as recovery:
            runtime.report['consumer_recovery_error'] = repr(recovery)
            failure = failure or recovery
        runtime.record('camera_comparison_finished')
        for sub in runtime.subscribers: sub.unregister()
        for path in (Path(__file__), WRAPPER, WORK / '.runtime/camera_preset_ab_plan_20261004.json'):
            (out / path.name).write_bytes(path.read_bytes())
    print(json.dumps(dict(result=str(out / 'result.json'), experiment_valid=runtime.report['experiment_valid'],
        visual_nodes_restored=runtime.report.get('visual_nodes_restored'), error=repr(failure) if failure else None)), flush=True)
    if failure: raise failure


if __name__ == '__main__':
    main()
