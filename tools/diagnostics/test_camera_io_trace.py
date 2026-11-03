#!/usr/bin/env python3
"""Offline-only interposer tests. Temporary files contain synthetic events, never camera data."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT / '.runtime/camera_io_trace'
SRC = ROOT / 'tools/diagnostics'
BUILD.mkdir(parents=True, exist_ok=True)

def compile_(source, output, extra=()):
    subprocess.run(['cc', '-O2', '-std=gnu11', '-Wall', '-Wextra', '-Werror', *extra,
                    str(SRC / source), '-o', str(BUILD / output), '-ldl', '-lpthread'], check=True)

compile_('camera_io_trace.c', 'libcamera_io_trace.so', ['-fPIC', '-shared', '-Wl,-z,nodelete'])
compile_('camera_io_trace.c', 'libcamera_io_trace_test.so',
         ['-fPIC', '-shared', '-Wl,-z,nodelete', '-DTRACE_TEST_BUILD', '-DTRACE_CAPACITY=64'])
compile_('test_camera_io_trace.c', 'fixture')
compile_('test_camera_io_trace_mock.c', 'mock.so', ['-fPIC', '-shared'])
compile_('test_camera_io_trace_success.c', 'success')
checks = []
metrics = {}

def run(name, mode='', preload=True, extra=None, success=False, production=False):
    env = os.environ.copy()
    for key in list(env):
        if key.startswith('CAMERA_IO_TRACE_') or key == 'LD_PRELOAD':
            env.pop(key)
    if preload:
        env['LD_PRELOAD'] = str(BUILD / ('libcamera_io_trace.so' if production else 'libcamera_io_trace_test.so'))
    if success:
        env['LD_PRELOAD'] += ':' + str(BUILD / 'mock.so')
    env.update(extra or {})
    start = time.monotonic()
    result = subprocess.run([str(BUILD / ('success' if success else 'fixture'))] + ([mode] if mode else []),
                            env=env, capture_output=True, text=True, timeout=8)
    assert result.returncode == 0, (name, result.returncode, result.stdout, result.stderr)
    checks.append(name)
    return result, time.monotonic() - start

def load_events(path):
    events = [json.loads(line) for line in path.read_text().splitlines()]
    assert events[0]['kind'] == 'header'
    assert events[-1]['kind'] == 'summary'
    body = events[1:-1]
    last = {}
    seen = set()
    for event in body:
        assert event['mono_ns'] > 0 and event['wall_ns'] > 10**18
        assert event['mono_ns'] >= last.get(event['tid'], 0)
        last[event['tid']] = event['mono_ns']
        identity = (event['id'], event['kind'])
        assert identity not in seen
        seen.add(identity)
        if event['kind'].endswith('_exit'):
            assert event['duration_ns'] >= 0
    summary = events[-1]
    assert summary['recorded'] == summary['flushed'] == len(body), summary
    assert summary['write_failed'] == 0
    return events

run('baseline_functional', preload=False)
run('disabled_transparent')
with tempfile.TemporaryDirectory(prefix='camera_io_trace_test_', dir='/tmp') as tmp:
    tmp = Path(tmp)
    base = {'CAMERA_IO_TRACE_TEST_TMP': '1', 'CAMERA_IO_TRACE_SECONDS': '3'}
    path = tmp / 'functional.jsonl'
    run('select_pselect_ioctl_errno_and_invalid_pointer', extra={**base, 'CAMERA_IO_TRACE_PATH': str(path)})
    events = load_events(path)
    assert any(e['kind'] == 'select_exit' and e['ret'] == 1 and int(e['sets_hex'][0], 16) != 0 for e in events)
    assert any(e['kind'] == 'pselect_exit' and e['ret'] == 0 and e['duration_ns'] >= 15_000_000 for e in events)
    assert sum(e['kind'] == 'ioctl_exit' for e in events) == 3
    assert all(e['buffer_valid'] == 0 for e in events if e['kind'] == 'ioctl_exit')
    checks.append('event_clock_ready_set_duration_and_failed_buffer_semantics')
    path = tmp / 'mock.jsonl'
    run('success_buffer_fields_and_unknown_scalar_forwarding', success=True,
        extra={**base, 'CAMERA_IO_TRACE_PATH': str(path)})
    events = load_events(path)
    b = [e for e in events if e['kind'] == 'ioctl_exit']
    assert len(b) == 2 and all(e['buffer_valid'] == 1 and e['sequence'] == 77 and e['index'] == 3 and
                             e['bytesused'] == 512 and e['buffer_sec'] == 123 and e['buffer_usec'] == 456 for e in b)
    checks.append('successful_v4l2_fields_no_pixel_data')
    path = tmp / 'full.jsonl'
    result, elapsed = run('eight_producer_full_queue_slow_writer_bounded_exit', mode='stress',
        extra={**base, 'CAMERA_IO_TRACE_PATH': str(path), 'CAMERA_IO_TRACE_TEST_WRITER_DELAY_US': '100000'})
    assert elapsed < 5
    events = load_events(path)
    assert events[-1]['dropped'] > 0
    metrics['overflow'] = {'elapsed_sec': elapsed, 'summary': events[-1], 'producer': json.loads(result.stdout)}
    path = tmp / 'deadline.jsonl'
    run('bounded_recording_deadline', mode='deadline', extra={**base, 'CAMERA_IO_TRACE_SECONDS': '1', 'CAMERA_IO_TRACE_PATH': str(path)})
    events = load_events(path)
    assert all(e['mono_ns'] < events[0]['deadline_mono_ns'] for e in events[1:-1])
    assert len(events[1:-1]) == 20
    path = tmp / 'never_create.jsonl'
    result, _ = run('production_rejects_system_disk_output', production=True,
                    extra={'CAMERA_IO_TRACE_PATH': str(path)})
    assert not path.exists() and 'no fallback' in result.stderr
    path = tmp / 'preserve.jsonl'
    path.write_text('preserve')
    result, _ = run('existing_log_not_overwritten', extra={**base, 'CAMERA_IO_TRACE_PATH': str(path)})
    assert path.read_text() == 'preserve' and 'no fallback' in result.stderr
    run('disabled_overlong_path_transparent', extra={**base, 'CAMERA_IO_TRACE_PATH': '/tmp/' + 'x' * 5000})
    for name, preload, recording in [('baseline',False,False), ('disabled',True,False), ('enabled',True,True)]:
        path = tmp / (name + '.jsonl')
        extra = {**base, 'CAMERA_IO_TRACE_PATH': str(path)} if recording else None
        result, _ = run(name+'_benchmark', mode='benchmark', preload=preload, extra=extra)
        value = json.loads(result.stdout)
        value['ns_per_call'] = value['elapsed_ns'] / value['benchmark_calls']
        metrics[name] = value
        if recording: load_events(path)
print(json.dumps({'status': 'passed', 'check_count': len(checks), 'checks': checks, 'metrics': metrics}, indent=2))
