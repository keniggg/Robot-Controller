"""One isolated offline ownership check, with evidence only on SanStand."""
import datetime
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import sys
import time
import unittest
from unittest import mock


W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
F = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1')
assert os.path.ismount('/media/zhuyupei/SanStand')
stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
snapshot = W / '.runtime' / ('depth_stage_session_ownership_' + stamp)
output = F / 'depth_stage_probe_validation_r1' / ('session_ownership_' + stamp)
snapshot.mkdir()
output.mkdir(mode=0o700)
(snapshot / 'depth_stage_probe_candidate/tests').mkdir(parents=True)
files = []
for relative in ('depth_stage_camera_session_20261022.py',
                 'depth_stage_probe_candidate/tests/test_depth_stage_camera_session_ownership.py'):
    source = W / '.runtime' / relative
    frozen = snapshot / relative
    data = source.read_bytes()
    frozen.write_bytes(data)
    files.append(dict(source=str(source), frozen=str(frozen), sha256=hashlib.sha256(data).hexdigest()))
(output / 'frozen_inputs.json').write_text(json.dumps(dict(files=files), indent=2) + '\n')
denied = []
def guard(event, args):
    reason = None
    if event.startswith(('socket.connect', 'socket.bind', 'subprocess.Popen',
                         'os.system', 'os.fork', 'os.exec', 'os.kill')):
        reason = event
    elif event == 'import' and args[0].split('.')[0] in ('rospy', 'rosgraph', 'pyrealsense2'):
        reason = 'real ROS/SDK import'
    elif event == 'open':
        name, mode, flags = args
        if isinstance(name, str):
            if name.startswith('/dev/') and name not in ('/dev/null', '/dev/urandom', '/dev/random'):
                reason = 'device open'
            if name.startswith('/proc/'):
                reason = 'process access outside fakes'
            if (flags or 0) & (os.O_WRONLY | os.O_RDWR | os.O_CREAT | os.O_TRUNC | os.O_APPEND):
                target = Path(name).absolute()
                if target != output and output not in target.parents and name != '/dev/null':
                    reason = 'write outside isolated external output'
    if reason:
        denied.append(dict(event=event, reason=reason))
        raise RuntimeError('offline guard blocked ' + reason)

sys.addaudithook(guard)
test = snapshot / 'depth_stage_probe_candidate/tests/test_depth_stage_camera_session_ownership.py'
spec = importlib.util.spec_from_file_location('_ownership_tests', test)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
suite = unittest.defaultTestLoader.loadTestsFromModule(module)
def cases(suite):
    for item in suite:
        if isinstance(item, unittest.TestSuite):
            yield from cases(item)
        else:
            yield item.id()
collection = list(cases(suite))
assert len(collection) == len(set(collection)) == 9
started = time.monotonic()
cpu = time.process_time()
stream = io.StringIO()
result = unittest.TextTestRunner(stream=stream, verbosity=2).run(suite)
elapsed = time.monotonic() - started
cpu = time.process_time() - cpu
(output / 'unittest.log').write_text(stream.getvalue())
report = dict(schema=1, passed=result.wasSuccessful() and not denied,
              cases=collection, tests_run=result.testsRun, failures=len(result.failures),
              errors=len(result.errors), denied=denied, wall_sec=elapsed, cpu_sec=cpu,
              guard_scope='No device, process, network, subprocess, ROS, or writes outside external output',
              source_changed=[row['source'] for row in files if hashlib.sha256(Path(row['source']).read_bytes()).hexdigest() != row['sha256']],
              sources={row['source']:row['sha256'] for row in files},
              original_probe_tests_repeated=False)
assert not report['source_changed']
data = (json.dumps(report, indent=2) + '\n').encode()
with (output / 'result.json').open('xb') as target:
    target.write(data)
    target.flush()
    os.fsync(target.fileno())
assert (output / 'result.json').read_bytes() == data
print(json.dumps(dict(path=str(output / 'result.json'), sha256=hashlib.sha256(data).hexdigest(), **report)))
sys.exit(0 if report['passed'] else 1)
