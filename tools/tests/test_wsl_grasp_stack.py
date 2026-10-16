"""Exercise process ownership and real HTTP health without ROS or robot commands."""
import contextlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import start_wsl_grasp_stack as stack


@contextlib.contextmanager
def endpoint(payload):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            body = json.dumps(payload).encode()
            self.send_response(200)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass
    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield server.server_port
    finally:
        server.shutdown()
        server.server_close()
        thread.join()


def test_reuse_dedicated_simulation_and_never_stop_external_service():
    payload = {'ok': False, 'protocol_version': 3, 'digital_twin': stack.SIM_CONTRACT}
    with endpoint(payload) as port:
        service = stack.ServiceStack(startup_timeout=1)
        service.ensure('MuJoCo', port, stack.validate_simulation,
                       ['/nonexistent-command-must-not-run'], Path.cwd(), os.environ.copy())
        service.close()
        assert not service.owned
        assert stack.read_health(port) == payload


@pytest.mark.parametrize('change', [
    {'deployment_id': 'old-deployment'},
    {'initial_unknown_ik_position_tolerance_m': 0.005},
    {'strict_dynamic_object_lift_required': False},
])
def test_mismatched_service_not_replaced_or_killed(change):
    payload = {'protocol_version': 3, 'digital_twin': dict(stack.SIM_CONTRACT, **change)}
    with endpoint(payload) as port:
        service = stack.ServiceStack(startup_timeout=1)
        with pytest.raises(RuntimeError, match='health mismatch'):
            service.ensure('MuJoCo', port, stack.validate_simulation,
                           ['/nonexistent-command-must-not-run'], Path.cwd(), os.environ.copy())
        service.close()
        assert not service.owned
        assert stack.read_health(port) == payload


def test_unwarmed_or_wrong_checkpoint_not_ready():
    health = {'protocol_version': 3, 'grasp_backend': {
        'ok': True, 'backend': 'graspnet_baseline', 'loaded': True,
        'baseline_root': '/baseline', 'checkpoint': '/checkpoint',
        'torch_cuda_available': True}}
    stack.validate_graspnet(health, Path('/baseline'), Path('/checkpoint'), 'cuda:0')
    health['grasp_backend']['loaded'] = False
    with pytest.raises(RuntimeError, match='loaded'):
        stack.validate_graspnet(health, Path('/baseline'), Path('/checkpoint'), 'cuda:0')
    health['grasp_backend']['loaded'] = True
    with pytest.raises(RuntimeError, match='checkpoint'):
        stack.validate_graspnet(health, Path('/baseline'), Path('/other'), 'cuda:0')


def test_failed_child_cleanup_keeps_unrelated_process_alive(tmp_path):
    # Include a shell grandchild: both must be signalled as one owned group.
    script = tmp_path / 'child.py'
    marker = tmp_path / 'terminated'
    script.write_text('import signal,time,sys\n'
                      'def stop(*args):\n'
                      ' open(sys.argv[1], "w").write("terminated")\n'
                      ' raise SystemExit(0)\n'
                      'signal.signal(signal.SIGTERM, stop)\n'
                      'while True: time.sleep(.05)\n')
    unrelated = subprocess.Popen([sys.executable, '-c', 'import time;time.sleep(30)'])
    service = stack.ServiceStack(startup_timeout=0.5)
    # Port 0 cannot host the fake child; exercise timeout and group cleanup.
    try:
        with pytest.raises(RuntimeError, match='startup timed out'):
            service.ensure('fake', 0, lambda value: None,
                           ['bash', '-c', '"$1" "$2" "$3" & wait',
                            'fake-service', sys.executable, str(script), str(marker)],
                           tmp_path, os.environ.copy())
        owned = service.owned[0][1]
        service.close()
        deadline = time.monotonic() + 2
        while not marker.exists() and time.monotonic() < deadline:
            time.sleep(.02)
        assert marker.read_text() == 'terminated'
        assert owned.poll() is not None
        assert unrelated.poll() is None
    finally:
        service.close()
        unrelated.terminate()
        unrelated.wait(timeout=3)
