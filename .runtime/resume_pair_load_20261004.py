#!/usr/bin/env python3
"""Resume exactly the saved visual deployment after dead ROS URI retention."""
from collections import OrderedDict
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import threading

WORK = Path(__file__).resolve().parents[1]
FAILED = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/pair_rectification_load_20261004T140052.707632Z/result.json')


def module(name, relative):
    spec = importlib.util.spec_from_file_location(name, WORK / relative)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def main():
    deployment = module('reviewed_deployment', '.runtime/load_pair_rectification_20261004.py')
    pending = module('dead_uri_helper', '.runtime/visual_registration_20261004.py')
    assert deployment.sha(WORK / '.runtime/visual_registration_20261004.py') == '25aae37c88a5af895ad1d57ade35c4e8736b3ab706b0f7a82f263fb8b9cb1b5f'
    manifest = json.loads(deployment.MANIFEST.read_text())
    deployment.verify_manifest(manifest)
    failed = json.loads(FAILED.read_text())
    assert failed['loaded'] is False and failed['error'] == "ConnectionRefusedError(111, 'Connection refused')"
    assert failed['manifest_sha256'] == deployment.sha(deployment.MANIFEST)
    assert [e['event'] for e in failed['events']] == [
        'preflight_snapshot', 'stop_requested', 'stop_requested', 'stop_requested',
        'camera_configuration', 'rollback_finished']
    assert not failed['rollback']['restored']
    assert deployment.MOUNT.is_mount()
    out = FAILED.parent.parent / ('pair_rectification_resume_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'))
    out.mkdir(); (out / 'ros').mkdir()
    os.environ['ROS_LOG_DIR'] = str(out / 'ros')
    import rospy
    from sensor_msgs.msg import Image, JointState
    from std_msgs.msg import Header
    from alicia_flexible_grasp_supervisor.msg import GraspState
    rospy.init_node('resume_pair_rectification', anonymous=True, disable_signals=True)

    class Resume(deployment.Runtime):
        def registered(self, role):
            return pending.registered_or_pending(self.ros, deployment.NODES[role], self.before[role], self.helper)

    runtime = Resume.__new__(Resume)
    runtime.ros = rospy
    runtime.helper = module('reviewed_identity', deployment.HELPER)
    runtime.manifest, runtime.out = manifest, out
    runtime.allowed_env = runtime.helper.ENV_KEYS + deployment.EXTRA_ENV
    runtime.before = deepcopy(failed['before'])
    runtime.launch = deepcopy(failed['launch'])
    runtime.configuration = deepcopy(failed['camera_before'])
    runtime.new_configuration = deployment.enabled_configuration(runtime.configuration)
    runtime.selection, runtime.handeye, runtime.ledger = [deepcopy(failed[k]) for k in ('selection', 'handeye', 'ledger')]
    runtime.environments = deepcopy(failed['environments'])
    runtime.current = dict(camera=None, unknown=None, remote=None, gateway=runtime.before['gateway'])
    runtime.units = {}
    runtime.camera_started_ns = runtime.verified_camera = runtime.verified_camera_settings = None
    runtime.lock = threading.RLock()
    runtime.faults, runtime.latest, runtime.pairs = [], {}, OrderedDict()
    runtime.frozen = dict(sdk=deepcopy(failed['sdk_hold']), epoch=failed['epoch_ns'], task=False)
    runtime.report = dict(loaded=False, resumed_from=str(FAILED), resumed_from_sha256=deployment.sha(FAILED),
        script_sha256=deployment.sha(__file__), pending_helper_sha256=deployment.sha(WORK / '.runtime/visual_registration_20261004.py'),
        started_at_utc=datetime.now(timezone.utc).isoformat(), manifest_sha256=deployment.sha(deployment.MANIFEST),
        motion_commands_sent=False, torque_commands_sent=False, selection_commands_sent=False,
        temperature_queries_sent=False, events=[], before=runtime.before, launch=runtime.launch,
        camera_before=runtime.configuration, selection=runtime.selection, handeye=runtime.handeye,
        ledger=runtime.ledger, sdk_hold=runtime.frozen['sdk'], epoch_ns=runtime.frozen['epoch'])
    runtime.subscribers = [
        rospy.Subscriber('/alicia_d/sdk_command', JointState, runtime.sdk_cb, queue_size=100),
        rospy.Subscriber('/alicia_d/control_reference_epoch', Header, runtime.epoch_cb, queue_size=100),
        rospy.Subscriber('/grasp/state', GraspState, runtime.task_cb, queue_size=100)]
    for stream, key in (('rgb', 'color_topic'), ('depth', 'depth_topic')):
        runtime.subscribers.append(rospy.Subscriber(runtime.configuration[key], Image,
            lambda msg, kind=stream:runtime.image_cb(msg, kind), queue_size=3, buff_size=2**24))
    try:
        runtime.helper.wait(lambda: all(k in runtime.latest for k in ('sdk', 'epoch', 'task')), 5., 'original held state')
        runtime.verify()
        assert rospy.get_param('/camera') == runtime.new_configuration, 'interrupted camera configuration changed'
        for role in ('camera', 'unknown', 'remote'):
            assert not runtime.helper.alive(runtime.before[role]), 'old visual process still alive'
            assert runtime.registered(role) is None, 'replacement visual owner exists'
        # A newly started but unregistered process must not be duplicated.
        for process in Path('/proc').iterdir():
            if not process.name.isdigit(): continue
            try:
                argv = (process / 'cmdline').read_bytes().decode().split('\0')
            except (OSError, UnicodeError):
                continue
            assert not any('__name:=' + deployment.NODES[role][1:] in argv
                           for role in ('camera', 'unknown', 'remote')), 'unregistered visual process exists'
        runtime.record('explicit_failed_deployment_resume')
        deployment.perform_reload(runtime)
    except BaseException as exc:
        runtime.report.update(loaded=False, error=repr(exc))
        runtime.record('resume_failed')
        print(json.dumps(dict(loaded=False, result=str(out / 'result.json'), error=repr(exc))), flush=True)
        raise
    finally:
        for sub in runtime.subscribers: sub.unregister()
        (out / Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    print(json.dumps(dict(loaded=True, result=str(out / 'result.json'))), flush=True)


if __name__ == '__main__':
    main()
