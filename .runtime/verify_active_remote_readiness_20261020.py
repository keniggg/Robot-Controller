#!/usr/bin/env python3
"""Read-only remote dependency preflight; never selects a mode or runs prediction.

A reviewed, SHA-pinned startup binding is mandatory. Its construction parameters
come from frozen launch evidence, not from this probe's current parameter read.
This detects parameter drift from the client constructed in the bound process;
it does not claim to introspect Python object memory. Recreate/review the binding
after a node replacement or a deliberate construction-configuration change.
"""
import argparse
import ast
import base64
import hashlib
import http.client
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
import xmlrpc.client


STORE = Path('/media/zhuyupei/SanStand')
URL = '/grasp_6d/remote/server_url'
DEPTH = '/grasp_6d/remote/require_candidate_depth'
TIMEOUT = '/grasp_6d/remote/timeout_sec'
PARAMETERS = (URL, DEPTH, TIMEOUT)
SELECTION = '/grasp_mode/selection'
PROXY_KEYS = ('http_proxy', 'https_proxy', 'all_proxy', 'no_proxy',
              'HTTP_PROXY', 'HTTPS_PROXY', 'ALL_PROXY', 'NO_PROXY', 'REQUEST_METHOD')
CALLER = '/active_remote_readiness_readonly'
MAX_BODY = 1024 * 1024
HEALTH_DEADLINE = 3.0


class NotReady(RuntimeError):
    def __init__(self, code, reason):
        super().__init__(reason)
        self.code = code


def require(condition, code, reason):
    if not condition:
        raise NotReady(code, reason)


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def strict_json(raw):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError('duplicate JSON key: ' + key)
            result[key] = value
        return result
    def nonfinite(value):
        raise ValueError('nonfinite JSON number: ' + value)
    return json.loads(raw, object_pairs_hook=pairs, parse_constant=nonfinite)


def write_json(path, value):
    raw = (json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True,
                      allow_nan=False) + '\n').encode()
    path.write_bytes(raw)
    require(path.read_bytes() == raw, 'EVIDENCE_WRITE_FAILED', str(path))
    return digest(raw)


def valid_url(value):
    require(type(value) is str and value == value.strip(), 'CONFIG_INVALID', 'URL must be a string')
    parts = urllib.parse.urlsplit(value)
    require(parts.scheme in ('http', 'https') and bool(parts.hostname)
            and parts.username is None and parts.password is None
            and not parts.query and not parts.fragment,
            'CONFIG_INVALID', 'expected credential-free HTTP(S) service base URL')
    try:
        parts.port
    except ValueError as exc:
        raise NotReady('CONFIG_INVALID', str(exc)) from exc
    return value.rstrip('/')


def validate_parameters(parameters):
    require(set(parameters) == set(PARAMETERS), 'CONFIG_INVALID', 'construction parameter allowlist differs')
    valid_url(parameters[URL])
    require(type(parameters[DEPTH]) is bool, 'CONFIG_INVALID', 'require_candidate_depth must be boolean')
    timeout = parameters[TIMEOUT]
    require(type(timeout) in (int, float) and math.isfinite(timeout) and timeout > 0,
            'CONFIG_INVALID', 'client timeout must be positive and finite')


def reference_bytes(reference):
    path = Path(reference['path']).resolve()
    if 'prefix_bytes' in reference:
        size = reference['prefix_bytes']
        require(type(size) is int and 0 < size <= 16 * 1024 * 1024,
                'STARTUP_BINDING_INVALID', 'invalid startup log prefix size')
        with path.open('rb') as stream:
            raw = stream.read(size)
        require(len(raw) == size, 'STARTUP_EVIDENCE_CHANGED', str(path))
    else:
        raw = path.read_bytes()
    require(digest(raw) == reference['sha256'], 'STARTUP_EVIDENCE_CHANGED', str(path))
    return raw


def client_contract(reference):
    # Read constants without importing ROS, numpy, or executing production code.
    values = {}
    for statement in ast.parse(reference_bytes(reference)).body:
        if isinstance(statement, ast.Assign):
            for target in statement.targets:
                if isinstance(target, ast.Name) and target.id in (
                        'GRASP6D_PROTOCOL_VERSION', 'GRASP6D_CANDIDATE_FIELDS'):
                    values[target.id] = ast.literal_eval(statement.value)
    version = values.get('GRASP6D_PROTOCOL_VERSION')
    fields = values.get('GRASP6D_CANDIDATE_FIELDS')
    require(type(version) is int and version == 3 and type(fields) in (list, tuple)
            and bool(fields) and all(type(field) is str for field in fields)
            and len(fields) == len(set(fields)),
            'STARTUP_BINDING_INVALID', 'production protocol constants are invalid')
    return {'protocol_version': version, 'candidate_fields': list(fields)}


def load_binding(path, expected_sha):
    raw = path.read_bytes()
    require(digest(raw) == expected_sha, 'STARTUP_BINDING_CHANGED', str(path))
    value = strict_json(raw)
    require(type(value.get('schema')) is int and value['schema'] == 1
            and value.get('construction_evidence_review_required') is True,
            'STARTUP_BINDING_INVALID', 'reviewed construction evidence is required')
    validate_parameters(value['parameters'])
    require(type(value['node_name']) is str and value['node_name'].startswith('/'),
            'STARTUP_BINDING_INVALID', 'absolute remote node name required')
    valid_url(value['master_uri'])
    require(type(value['allowed_backends']) is list and value['allowed_backends']
            and all(type(item) is str and item for item in value['allowed_backends']),
            'STARTUP_BINDING_INVALID', 'explicit real backend allowlist required')
    identity = value['node_identity']
    require(set(identity) == {'pid', 'parent_pid', 'start_ticks', 'cmdline_sha256'}
            and type(identity['pid']) is int and identity['pid'] > 1
            and type(identity['parent_pid']) is int and identity['parent_pid'] > 1
            and type(identity['start_ticks']) is str and identity['start_ticks'].isdigit()
            and re.fullmatch('[0-9a-f]{64}', identity['cmdline_sha256']) is not None,
            'STARTUP_BINDING_INVALID', 'exact local process identity required')
    require(type(value['evidence']) is list and bool(value['evidence']),
            'STARTUP_BINDING_INVALID', 'frozen construction references required')
    for reference in value['evidence']:
        reference_bytes(reference)
    value['_client_contract'] = client_contract(value['client_contract_source'])
    return value


class RpcTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = HEALTH_DEADLINE
        return connection


def rpc_value(reply, operation):
    require(type(reply) in (list, tuple) and len(reply) == 3 and type(reply[0]) is int and reply[0] == 1,
            'RUNTIME_UNAVAILABLE', operation + ' was not accepted')
    return reply[2]


def observe(binding):
    master = xmlrpc.client.ServerProxy(binding['master_uri'], transport=RpcTransport())
    parameters = {name: rpc_value(master.getParam(CALLER, name), 'getParam ' + name)
                  for name in PARAMETERS}
    validate_parameters(parameters)
    selection = rpc_value(master.getParam(CALLER, SELECTION), 'getParam selection')
    require(type(selection) is dict and type(selection.get('generation')) is int
            and selection['generation'] >= 0 and type(selection.get('mode')) is str
            and type(selection.get('strategy')) is str
            and type(selection.get('stamp_ns')) in (int, str)
            and str(selection['stamp_ns']).isdigit() and int(selection['stamp_ns']) > 0,
            'RUNTIME_UNAVAILABLE', 'selection identity is invalid')
    uri = rpc_value(master.lookupNode(CALLER, binding['node_name']), 'lookupNode')
    node = xmlrpc.client.ServerProxy(uri, transport=RpcTransport())
    pid = rpc_value(node.getPid(CALLER), 'getPid')
    require(type(pid) is int and pid > 1, 'RUNTIME_UNAVAILABLE', 'remote node PID is invalid')
    proc = Path('/proc') / str(pid)
    stat = (proc / 'stat').read_text().rsplit(')', 1)[1].split()
    require(stat[0] != 'Z', 'RUNTIME_UNAVAILABLE', 'remote node is a zombie')
    command = (proc / 'cmdline').read_bytes()
    expected_name = ('__name:=' + binding['node_name'].lstrip('/')).encode()
    require(expected_name in command.split(b'\0') or ('__name:=' + binding['node_name']).encode() in command.split(b'\0'),
            'NODE_IDENTITY_CHANGED', 'process command does not identify the bound node')
    environment = dict(item.split('=', 1) for item in (proc / 'environ').read_bytes().decode().split('\0') if '=' in item)
    proxy = {name: environment[name] for name in PROXY_KEYS if name in environment}
    return {'parameters': parameters, 'selection': selection, 'node_uri': uri,
            'node_identity': {'pid': pid, 'parent_pid': int(stat[1]), 'start_ticks': stat[19],
                              'cmdline_sha256': digest(command)},
            'proxy_environment': proxy}


def check_binding(binding, observation):
    validate_parameters(observation['parameters'])
    require(observation['parameters'] == binding['parameters'], 'CLIENT_CONFIG_DRIFT',
            'live parameters differ from reviewed construction values; old client URL must not be assumed updated')
    require(observation['node_identity'] == binding['node_identity'], 'NODE_IDENTITY_CHANGED',
            'remote process differs from reviewed startup instance')


def public_observation(observation):
    result = {key: value for key, value in observation.items() if key != 'proxy_environment'}
    proxy = observation['proxy_environment']
    result['proxy_environment_keys'] = sorted(proxy)
    result['proxy_environment_sha256'] = digest(json.dumps(proxy, sort_keys=True).encode())
    return result


def health_worker(url):
    class NoRedirect(urllib.request.HTTPRedirectHandler):
        def redirect_request(self, *args, **kwargs):
            return None
    result = {'url': url, 'method': 'GET', 'timeout_sec': HEALTH_DEADLINE}
    started = time.monotonic()
    try:
        opener = urllib.request.build_opener(NoRedirect())
        request = urllib.request.Request(url, method='GET', headers={'Accept': 'application/json'})
        try:
            response = opener.open(request, timeout=HEALTH_DEADLINE)
        except urllib.error.HTTPError as exc:
            response = exc
        with response:
            result['http_status'] = response.code
            result['response_url'] = response.geturl()
            result['headers'] = {key: value for key, value in response.headers.items()
                                 if key.lower() in ('server', 'date', 'content-type', 'content-length')}
            body = response.read(MAX_BODY + 1)
        result['body_complete'] = len(body) <= MAX_BODY
        result['body_base64'] = base64.b64encode(body).decode('ascii')
    except Exception as exc:
        result.update(error_type=type(exc).__name__, error=str(exc))
    result['elapsed_sec'] = time.monotonic() - started
    return result


def fetch(url, proxy):
    # Isolate DNS/connect/read under one wall deadline. Timeout cleanup affects
    # only this helper's own read-only HTTP child, never a ROS/service process.
    env = {'PATH': os.defpath, 'PYTHONDONTWRITEBYTECODE': '1', 'PYTHONNOUSERSITE': '1', **proxy}
    started = time.monotonic()
    try:
        completed = subprocess.run([sys.executable, '-B', str(Path(__file__).resolve()), '--health-worker', url],
                                   env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                   timeout=HEALTH_DEADLINE, check=False)
    except subprocess.TimeoutExpired:
        return {'url': url, 'error_type': 'HealthDeadlineExceeded',
                'error': 'GET /health exceeded 3-second total deadline',
                'elapsed_sec': time.monotonic() - started}
    require(completed.returncode == 0, 'REMOTE_PROBE_FAILED', 'health worker failed')
    result = strict_json(completed.stdout)
    result['total_elapsed_sec'] = time.monotonic() - started
    return result


def validate_health(payload, binding):
    require(type(payload) is dict, 'REMOTE_PROTOCOL_INVALID', 'health must be a JSON object')
    backend = payload.get('grasp_backend', payload)
    require(type(backend) is dict, 'REMOTE_PROTOCOL_INVALID', 'grasp_backend must be an object')
    contract = binding['_client_contract']
    for label, item in [('health', payload), ('grasp_backend', backend)]:
        require(type(item.get('ok')) is bool, 'REMOTE_PROTOCOL_INVALID', label + '.ok must be boolean')
        require(item['ok'], 'REMOTE_UNHEALTHY', label + ' reports unhealthy')
        require(type(item.get('protocol_version')) is int and item['protocol_version'] == contract['protocol_version'],
                'REMOTE_PROTOCOL_INVALID', label + ' must advertise integer protocol_version=3')
        fields = item.get('candidate_fields')
        require(type(fields) is list and all(type(field) is str for field in fields)
                and fields == contract['candidate_fields'],
                'REMOTE_PROTOCOL_INVALID', label + ' candidate fields differ from the exact production protocol')
        name = item.get('backend')
        require(type(name) is str and name.strip() == name and bool(name),
                'REMOTE_PROTOCOL_INVALID', label + '.backend must be a nonempty string')
        require(not ({'mock', 'fake', 'dummy', 'synthetic'} & set(re.split(r'[^a-z0-9]+', name.lower()))),
                'MOCK_BACKEND_FORBIDDEN', 'real attempts cannot use a mock backend')
        require(name in binding['allowed_backends'], 'REMOTE_BACKEND_MISMATCH', 'backend differs from reviewed configuration')
        for flag in ('mock', 'is_mock'):
            if flag in item:
                require(item[flag] is False, 'MOCK_BACKEND_FORBIDDEN', 'mock marker is not boolean false')
    return {'backend': backend['backend'], 'protocol_version': 3, 'candidate_fields': backend['candidate_fields'],
            'require_candidate_depth': binding['parameters'][DEPTH],
            'loaded': backend.get('loaded'), 'torch_cuda_available': backend.get('torch_cuda_available'),
            'capability_policy': 'loaded/CUDA reported only; legal lazy-loading and CPU are allowed',
            'health_scope': 'dependency availability and advertised protocol; no prediction or motion proof'}


def evaluate(binding, output, observer=observe, fetcher=fetch):
    result = {'schema': 1, 'passed': False, 'dependency_ready': False, 'started_ns': time.time_ns(),
              'ros_write_operations': [], 'prediction_requests': 0, 'motion_commands_sent': False,
              'temperature_queried': False, 'torque_disable_commands_sent': False}
    try:
        before = observer(binding)
        result['before'] = public_observation(before)
        check_binding(binding, before)
        url = valid_url(before['parameters'][URL]) + '/health'
        result['health_request_started_ns'] = time.time_ns()
        probe = fetcher(url, before['proxy_environment'])
        result['health_request_finished_ns'] = time.time_ns()
        encoded = probe.pop('body_base64', None)
        raw = base64.b64decode(encoded, validate=True) if encoded is not None else None
        if raw is not None:
            path = output / 'health_response.body'
            path.write_bytes(raw)
            require(path.read_bytes() == raw, 'EVIDENCE_WRITE_FAILED', str(path))
            probe.update(body_path=str(path), body_bytes=len(raw), body_sha256=digest(raw))
        result['probe'] = probe
        after = observer(binding)
        result['after'] = public_observation(after)
        check_binding(binding, after)
        require(before == after, 'RUNTIME_CHANGED_DURING_PROBE', 'node, selection, configuration or transport environment changed')
        require('error_type' not in probe, 'REMOTE_UNREACHABLE', probe.get('error', 'health request failed'))
        require(type(probe.get('http_status')) is int and probe['http_status'] == 200,
                'REMOTE_HTTP_FAILURE', 'health did not return HTTP 200')
        require(probe.get('response_url') == url and probe.get('body_complete') is True and raw is not None,
                'REMOTE_PROTOCOL_INVALID', 'redirected, oversized or missing health response')
        try:
            payload = strict_json(raw)
        except (ValueError, UnicodeError) as exc:
            raise NotReady('REMOTE_PROTOCOL_INVALID', 'invalid health JSON: ' + str(exc)) from exc
        result['capabilities'] = validate_health(payload, binding)
        result.update(passed=True, dependency_ready=True, failure_code=None)
    except Exception as exc:
        result.update(failure_code=getattr(exc, 'code', 'READINESS_CHECK_FAILED'), error_type=type(exc).__name__, error=str(exc))
    result['finished_ns'] = time.time_ns()
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--startup-binding', type=Path, required=True)
    parser.add_argument('--startup-binding-sha256', required=True)
    args = parser.parse_args(argv)
    output = args.output.resolve()
    require(STORE.is_mount() and STORE in output.parents and not output.exists(),
            'EVIDENCE_STORAGE_INVALID', 'new output directory on mounted SanStand required')
    output.mkdir(parents=True)
    result = {'schema': 1, 'passed': False, 'dependency_ready': False}
    helper_raw = Path(__file__).read_bytes()
    try:
        binding = load_binding(args.startup_binding.resolve(), args.startup_binding_sha256)
        result = evaluate(binding, output)
        # Keep construction proof and the checker itself fixed across the probe.
        require(load_binding(args.startup_binding.resolve(), args.startup_binding_sha256) == binding
                and Path(__file__).read_bytes() == helper_raw,
                'STARTUP_EVIDENCE_CHANGED', 'binding/evidence/helper changed during probe')
    except Exception as exc:
        result.update(passed=False, dependency_ready=False,
                      failure_code=getattr(exc, 'code', 'READINESS_CHECK_FAILED'), error_type=type(exc).__name__, error=str(exc))
    result.update(startup_binding=str(args.startup_binding.resolve()), startup_binding_sha256=args.startup_binding_sha256,
                  helper_sha256=digest(helper_raw), automatic_grasp_started=False)
    sha = write_json(output / 'result.json', result)
    print(json.dumps({'passed': result['passed'], 'dependency_ready': result['dependency_ready'],
                      'failure_code': result.get('failure_code'), 'result': str(output / 'result.json'), 'sha256': sha}))
    return 0 if result['passed'] else 2


if __name__ == '__main__':
    if len(sys.argv) == 3 and sys.argv[1] == '--health-worker':
        # Internal subprocess mode is GET-only and exposes no ROS operation.
        url = valid_url(sys.argv[2])
        require(urllib.parse.urlsplit(url).path.endswith('/health'), 'CONFIG_INVALID', 'health path required')
        print(json.dumps(health_worker(url), allow_nan=False))
    else:
        sys.exit(main())
