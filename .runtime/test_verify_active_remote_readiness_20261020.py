#!/usr/bin/env python3
"""Isolated contract tests. All evidence/temp files live on SanStand; no ROS/network."""
import argparse
import base64
from copy import deepcopy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import time
import unittest
from unittest import mock


HERE = Path(__file__).resolve().parent
HELPER = HERE / 'verify_active_remote_readiness_20261020.py'
spec = importlib.util.spec_from_file_location('active_remote_readiness', HELPER)
h = importlib.util.module_from_spec(spec)
spec.loader.exec_module(h)
CLIENT = HERE.parent / 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/remote_grasp6d_client.py'
CLIENT_REF = {'path': str(CLIENT), 'sha256': h.digest(CLIENT.read_bytes())}
CONTRACT = h.client_contract(CLIENT_REF)
TEMP_ROOT = None


def binding():
    return {'schema': 1, 'construction_evidence_review_required': True,
            'master_uri': 'http://localhost:11311', 'node_name': '/remote_grasp6d_node',
            'parameters': {h.URL: 'http://backend.example:8000', h.DEPTH: True, h.TIMEOUT: 30.0},
            'node_identity': {'pid': 2345, 'parent_pid': 1234, 'start_ticks': '4567', 'cmdline_sha256': 'a' * 64},
            'allowed_backends': ['graspnet_baseline'], 'evidence': [CLIENT_REF],
            'client_contract_source': CLIENT_REF, '_client_contract': deepcopy(CONTRACT)}


def healthy():
    return {'ok': True, 'backend': 'graspnet_baseline', 'protocol_version': CONTRACT['protocol_version'],
            'candidate_fields': list(CONTRACT['candidate_fields']), 'loaded': True,
            'torch_cuda_available': True}


def observation(b):
    return {'parameters': deepcopy(b['parameters']), 'selection': {'generation': 2, 'stamp_ns': '123456789',
            'mode': 'unknown', 'strategy': 'observe_return'}, 'node_identity': deepcopy(b['node_identity']),
            'node_uri': 'http://localhost:45000/', 'proxy_environment': {}}


def response(payload):
    return {'url': 'http://backend.example:8000/health', 'response_url': 'http://backend.example:8000/health',
            'http_status': 200, 'body_complete': True,
            'body_base64': base64.b64encode(json.dumps(payload).encode()).decode()}


class Checks(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(dir=TEMP_ROOT)
        self.addCleanup(self.tmp.cleanup)
        self.out = Path(self.tmp.name)
        self.b = binding()
        self.obs = observation(self.b)
        self.network = mock.patch.object(socket, 'socket', side_effect=AssertionError('test attempted real network'))
        self.network.start()
        self.addCleanup(self.network.stop)

    def evaluate(self, payload=None, probe=None, observations=None):
        getter = mock.Mock(return_value=response(payload or healthy()) if probe is None else probe)
        observer = mock.Mock(side_effect=observations) if observations else mock.Mock(return_value=self.obs)
        result = h.evaluate(self.b, self.out, observer=observer, fetcher=getter)
        self.assertEqual(result['ros_write_operations'], [])
        self.assertEqual(result['prediction_requests'], 0)
        self.assertFalse(result['motion_commands_sent'])
        return result, getter

    def test_direct_legal(self):
        result, _ = self.evaluate()
        self.assertTrue(result['passed'], result)

    def test_unified_legal_lazy_cpu(self):
        child = healthy(); child.update(loaded=False, torch_cuda_available=False)
        payload = dict(healthy(), grasp_backend=child)
        result, _ = self.evaluate(payload)
        self.assertTrue(result['passed'], result)
        self.assertIs(result['capabilities']['loaded'], False)
        self.assertIs(result['capabilities']['torch_cuda_available'], False)

    def test_capabilities_optional(self):
        payload = healthy(); del payload['loaded']; del payload['torch_cuda_available']
        result, _ = self.evaluate(payload)
        self.assertTrue(result['passed'], result)

    def test_offline_is_fail_fast(self):
        result, getter = self.evaluate(probe={'error_type': 'ConnectionResetError', 'error': 'arbitrary transport error'})
        self.assertFalse(result['passed'])
        self.assertEqual(result['failure_code'], 'REMOTE_UNREACHABLE')
        self.assertEqual(getter.call_count, 1)

    def test_url_drift_rejected_before_network(self):
        self.obs['parameters'][h.URL] = 'http://changed.example:8000'
        result, getter = self.evaluate()
        self.assertEqual(result['failure_code'], 'CLIENT_CONFIG_DRIFT')
        getter.assert_not_called()

    def test_url_changes_during_network(self):
        after = deepcopy(self.obs); after['parameters'][h.URL] = 'http://changed.example:8000'
        result, _ = self.evaluate(observations=[self.obs, after])
        self.assertEqual(result['failure_code'], 'CLIENT_CONFIG_DRIFT')

    def test_node_changes_during_network(self):
        after = deepcopy(self.obs); after['node_identity']['start_ticks'] = '9999'
        result, _ = self.evaluate(observations=[self.obs, after])
        self.assertEqual(result['failure_code'], 'NODE_IDENTITY_CHANGED')

    def test_selection_changes_during_network(self):
        after = deepcopy(self.obs); after['selection']['generation'] += 1
        result, _ = self.evaluate(observations=[self.obs, after])
        self.assertEqual(result['failure_code'], 'RUNTIME_CHANGED_DURING_PROBE')

    def test_proxy_environment_changes_during_network(self):
        after = deepcopy(self.obs); after['proxy_environment'] = {'http_proxy': 'http://proxy.example:3128'}
        result, _ = self.evaluate(observations=[self.obs, after])
        self.assertEqual(result['failure_code'], 'RUNTIME_CHANGED_DURING_PROBE')

    def test_wrong_config_boolean_type(self):
        self.obs['parameters'][h.DEPTH] = 'true'
        result, getter = self.evaluate()
        self.assertEqual(result['failure_code'], 'CONFIG_INVALID')
        getter.assert_not_called()

    def test_raw_body_saved_exactly_on_protocol_failure(self):
        payload = healthy(); payload['protocol_version'] = 2
        result, _ = self.evaluate(payload)
        raw = (self.out / 'health_response.body').read_bytes()
        self.assertEqual(json.loads(raw), payload)
        self.assertEqual(result['probe']['body_sha256'], h.digest(raw))

    def test_binding_evidence_and_contract_load(self):
        payload = deepcopy(self.b); del payload['_client_contract']
        path = self.out / 'binding.json'; path.write_text(json.dumps(payload))
        actual = h.load_binding(path, h.digest(path.read_bytes()))
        self.assertEqual(actual['_client_contract'], CONTRACT)
        payload['evidence'] = [dict(CLIENT_REF, sha256='0' * 64)]
        path.write_text(json.dumps(payload))
        with self.assertRaises(h.NotReady) as caught:
            h.load_binding(path, h.digest(path.read_bytes()))
        self.assertEqual(caught.exception.code, 'STARTUP_EVIDENCE_CHANGED')

    def test_log_prefix_allows_append_but_detects_rewrite(self):
        path = self.out / 'startup.log'; path.write_bytes(b'bound\n')
        ref = {'path': str(path), 'prefix_bytes': 6, 'sha256': h.digest(b'bound\n')}
        path.write_bytes(b'bound\nnew\n')
        self.assertEqual(h.reference_bytes(ref), b'bound\n')
        path.write_bytes(b'other\nnew\n')
        with self.assertRaises(h.NotReady): h.reference_bytes(ref)

    def test_total_deadline_and_transport_environment(self):
        proxy = {'http_proxy': 'http://proxy.example:3128', 'no_proxy': 'backend.example'}
        with mock.patch.object(subprocess, 'run', side_effect=subprocess.TimeoutExpired('own health child', 3)) as run:
            result = h.fetch('http://backend.example:8000/health', proxy)
        self.assertEqual(result['error_type'], 'HealthDeadlineExceeded')
        self.assertEqual(run.call_args.kwargs['timeout'], 3.0)
        self.assertEqual(run.call_args.kwargs['env']['no_proxy'], 'backend.example')
        self.assertEqual(run.call_args.args[0][-2:], ['--health-worker', 'http://backend.example:8000/health'])

    def test_worker_only_get_health(self):
        reply = mock.MagicMock(); reply.code = 200; reply.geturl.return_value = 'http://backend.example:8000/health'
        reply.headers.items.return_value = [('Content-Type', 'application/json')]
        reply.read.return_value = json.dumps(healthy()).encode()
        opener = mock.Mock(); opener.open.return_value = reply
        with mock.patch.object(h.urllib.request, 'build_opener', return_value=opener):
            result = h.health_worker('http://backend.example:8000/health')
        request = opener.open.call_args.args[0]
        self.assertEqual(request.get_method(), 'GET'); self.assertIsNone(request.data)
        self.assertEqual(opener.open.call_args.kwargs['timeout'], 3.0)
        self.assertEqual(result['http_status'], 200)

    def test_main_failure_nonzero_and_no_downstream(self):
        payload = deepcopy(self.b); del payload['_client_contract']
        bp = self.out / 'binding.json'; bp.write_text(json.dumps(payload))
        target = self.out / 'run'
        with mock.patch.object(h, 'evaluate', return_value={'passed': False, 'dependency_ready': False,
                'failure_code': 'REMOTE_UNREACHABLE'}), mock.patch.object(h.subprocess, 'run') as downstream:
            rc = h.main(['--startup-binding', str(bp), '--startup-binding-sha256', h.digest(bp.read_bytes()),
                         '--output', str(target)])
        self.assertEqual(rc, 2); downstream.assert_not_called()
        self.assertFalse(json.loads((target / 'result.json').read_text())['passed'])


def rejected_case(mutate, expected='REMOTE_PROTOCOL_INVALID'):
    def test(self):
        payload = healthy()
        payload = mutate(payload)
        result, _ = self.evaluate(payload)
        self.assertFalse(result['passed'], result)
        self.assertEqual(result['failure_code'], expected, result)
    return test


for name, mutate, code in [
    ('protocol_missing', lambda p: {k: v for k, v in p.items() if k != 'protocol_version'}, 'REMOTE_PROTOCOL_INVALID'),
    ('protocol_string', lambda p: dict(p, protocol_version='3'), 'REMOTE_PROTOCOL_INVALID'),
    ('protocol_boolean', lambda p: dict(p, protocol_version=True), 'REMOTE_PROTOCOL_INVALID'),
    ('protocol_wrong', lambda p: dict(p, protocol_version=2), 'REMOTE_PROTOCOL_INVALID'),
    ('ok_string', lambda p: dict(p, ok='true'), 'REMOTE_PROTOCOL_INVALID'),
    ('ok_number', lambda p: dict(p, ok=1), 'REMOTE_PROTOCOL_INVALID'),
    ('unhealthy', lambda p: dict(p, ok=False), 'REMOTE_UNHEALTHY'),
    ('fields_missing_depth', lambda p: dict(p, candidate_fields=[f for f in p['candidate_fields'] if f != 'depth_m']), 'REMOTE_PROTOCOL_INVALID'),
    ('fields_reverse', lambda p: dict(p, candidate_fields=list(reversed(p['candidate_fields']))), 'REMOTE_PROTOCOL_INVALID'),
    ('fields_extra', lambda p: dict(p, candidate_fields=p['candidate_fields'] + ['extra']), 'REMOTE_PROTOCOL_INVALID'),
    ('fields_wrong_type', lambda p: dict(p, candidate_fields='score,width_m'), 'REMOTE_PROTOCOL_INVALID'),
    ('unified_backend_wrong_type', lambda p: dict(p, grasp_backend=[]), 'REMOTE_PROTOCOL_INVALID'),
    ('unified_nested_protocol', lambda p: dict(p, grasp_backend=dict(p, protocol_version=1)), 'REMOTE_PROTOCOL_INVALID'),
    ('mock_direct', lambda p: dict(p, backend='mock'), 'MOCK_BACKEND_FORBIDDEN'),
    ('mock_nested', lambda p: dict(p, grasp_backend=dict(p, backend='mock')), 'MOCK_BACKEND_FORBIDDEN'),
    ('mock_marker', lambda p: dict(p, mock=True), 'MOCK_BACKEND_FORBIDDEN'),
]:
    setattr(Checks, 'test_reject_' + name, rejected_case(mutate, code))


def main():
    global TEMP_ROOT
    parser = argparse.ArgumentParser(); parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(); output = args.output.resolve()
    assert h.STORE.is_mount() and h.STORE in output.parents and not output.exists()
    output.mkdir(parents=True); TEMP_ROOT = output
    available = sorted(os.sched_getaffinity(0)); os.sched_setaffinity(0, available[:2])
    started = time.perf_counter()
    with (output / 'unittest.log').open('w') as stream:
        suite = unittest.defaultTestLoader.loadTestsFromTestCase(Checks)
        result = unittest.TextTestRunner(stream=stream, verbosity=2).run(suite)
    report = {'passed': result.wasSuccessful(), 'tests': result.testsRun,
              'failures': len(result.failures), 'errors': len(result.errors), 'skipped': len(result.skipped),
              'elapsed_sec': time.perf_counter() - started, 'cpu_affinity': sorted(os.sched_getaffinity(0)),
              'real_network_calls': 0, 'ros_operations': [], 'helper_sha256': h.digest(HELPER.read_bytes()),
              'test_sha256': h.digest(Path(__file__).read_bytes()), 'production_contract': CLIENT_REF,
              'log_sha256': h.digest((output / 'unittest.log').read_bytes())}
    sha = h.write_json(output / 'result.json', report)
    print(json.dumps(dict(report, result=str(output / 'result.json'), result_sha256=sha)))
    return 0 if result.wasSuccessful() else 1


if __name__ == '__main__':
    raise SystemExit(main())
