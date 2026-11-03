#!/usr/bin/env python3
"""Offline header-only diagnostic queue replay; no ROS initialization or pixels."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace


class SizedData:
    def __init__(self, size): self.size = size
    def __len__(self): return self.size


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()


def replay(module, outcome, observed_writer):
    start = outcome['capture_start_monotonic']
    cache = module.ExactPairs(start, outcome['capture_deadline_monotonic'])
    events = [(h['receipt_monotonic'], 0, stream, h)
              for stream, rows in outcome['receive']['arrivals'].items() for h in rows]
    if observed_writer:
        events += [(p['write_completed_monotonic'], 1, '', p) for p in outcome['pairs']]
    first_limit = None
    max_bytes = max_ready = 0
    emitted, mismatches = [], []
    for now, kind, stream, h in sorted(events):
        if kind:
            pair = cache.pop()
            actual = None if pair is None else pair[0]
            if actual != h['source_ns']:
                mismatches.append([now, actual, h['source_ns']])
            if pair is not None: emitted.append(pair[0])
        else:
            ns = h['source_ns']
            msg = SimpleNamespace(header=SimpleNamespace(
                stamp=SimpleNamespace(to_nsec=lambda ns=ns: ns), seq=h['sequence'], frame_id=h['frame_id']),
                height=h['height'], width=h['width'], encoding=h['encoding'], step=h['step'],
                data=SizedData(h['data_bytes']))
            before = cache.counts['memory_limit']
            cache.add(stream, msg, now, h['receipt_ros_ns'])
            if cache.counts['memory_limit'] > before and first_limit is None:
                first_limit = dict(elapsed_sec=now-start, source_ns=ns, retained_bytes=cache.bytes,
                                   pending_sources=len(cache.pending), ready_pairs=len(cache.ready))
            max_bytes, max_ready = max(max_bytes, cache.bytes), max(max_ready, len(cache.ready))
            if not observed_writer:
                while True:
                    pair = cache.pop()
                    if pair is None: break
                    emitted.append(pair[0])
    pending = dict(sources=len(cache.pending), ready_pairs=len(cache.ready),
        streams={stream: sum(stream in row for row in cache.pending.values()) for stream in module.TOPICS})
    cache.stop()
    return dict(emitted_source_ns=emitted, emitted_pairs=len(emitted), counts=dict(cache.counts),
        first_memory_limit=first_limit, maximum_retained_bytes=max_bytes,
        maximum_ready_pairs=max_ready, before_stop=pending, writer_source_mismatches=mismatches)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--experiment', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    mount = Path('/media/zhuyupei/SanStand')
    assert mount.is_mount() and mount.resolve() in args.output.resolve().parents
    args.output.mkdir(exist_ok=False)
    old = args.experiment/'sources/compare_raw_rgbd_recording_load_20261007.py'
    current = root/'.runtime/compare_raw_rgbd_recording_load_20261007.py'
    tests = root/'.runtime/test_compare_raw_rgbd_recording_load_20261007.py'
    original = args.experiment/'raw_load/outcome.json'
    data = json.loads(original.read_text())
    assert data['bag_closed'] is True
    before, after = load(old, 'captured_original_raw'), load(current, 'fixed_raw')
    original_timing = replay(before, data, True)
    assert not original_timing['writer_source_mismatches']
    assert original_timing['emitted_source_ns'] == [p['source_ns'] for p in data['pairs']]
    for key in ('exact_pairs', 'memory_limit', 'unmatched_sources_at_end', 'unmatched_messages_at_end'):
        assert original_timing['counts'][key] == data['receive']['counts'][key]
    eager_original, eager_fixed = replay(before, data, False), replay(after, data, False)
    shared = set(h['source_ns'] for h in data['receive']['arrivals']['color']) & set(
        h['source_ns'] for h in data['receive']['arrivals']['depth'])
    assert set(eager_fixed['emitted_source_ns']) == shared
    assert eager_fixed['maximum_retained_bytes'] <= 64*1024**2
    candidate_report = dict(data, receive={'counts': eager_fixed['counts'], 'queue_pairs_remaining': 0})
    assert not after.validate_capture(candidate_report)[0]
    command = [sys.executable, '-m', 'pytest', '-q', '-p', 'no:cacheprovider', str(tests),
               '--junitxml='+str(args.output/'pytest.xml')]
    result = subprocess.run(command, cwd=root, env=dict(os.environ, PYTHONDONTWRITEBYTECODE='1'),
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    (args.output/'pytest.log').write_text(result.stdout)
    for row in (original_timing, eager_original, eager_fixed):
        row['emitted_source_list_sha256'] = hashlib.sha256(json.dumps(row.pop('emitted_source_ns')).encode()).hexdigest()
    report = dict(schema=1, diagnostic_only=True, live_operations=False,
        source_sha256={str(p): sha(p) for p in (old, current, tests, original, Path(__file__))},
        original_capture_valid=data['capture_valid'], real_exact_sources_in_both_arrival_lists=len(shared),
        old_with_observed_writer_completion=original_timing,
        old_with_eager_writer=eager_original, fixed_with_eager_writer=eager_fixed,
        fixed_replay_capture_valid=False, tests=dict(command=command, returncode=result.returncode, output=result.stdout),
        limits=['Observed writer completion is a conservative release time, not the actual pre-write pop time; all original emitted sources and relevant counters nevertheless exactly reproduce.',
                'Eager-writer replay isolates queue starvation, and is not a prediction of real I/O throughput, allocation or future source delivery.',
                'Any pending capacity eviction permanently invalidates completeness. No original capture, image, timestamp or production node was changed.'])
    path = args.output/'report.json'
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(dict(report=str(path), sha256=sha(path), tests_returncode=result.returncode,
                          old_pairs=eager_original['emitted_pairs'], fixed_pairs=eager_fixed['emitted_pairs'])))
    return result.returncode


if __name__ == '__main__': raise SystemExit(main())
