#!/usr/bin/env python3
"""Single-source-read adapter for the sealed GEN55 resource analyzer.

Only adapts its output guard and bag I/O. Resource mathematics, timing topic
allowlist and task-derived counting remain in the SHA-pinned original helper.
The sealed bag is streamed once into RAM while hashing; rosbag decodes that
buffer once. Later provenance checks reuse the verified digest and recheck the
file identity rather than reading the large source file again.
"""
import argparse
import contextlib
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import resource
import sys
import tempfile
import time


sys.dont_write_bytecode = True
W = Path(__file__).resolve().parents[1]
ORIGINAL = W / '.runtime/analyze_gen55_actual_resources_20261019.py'
ORIGINAL_SHA = '2b8436bc85b1cc717688247089ac0da0c10cc2847e92dfa0a66a69929a1c2082'
M = Path('/media/zhuyupei/SanStand')
V = M / '实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification'


def identity(path):
    stat = path.stat()
    return (stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns)


class BagBuffer(io.BytesIO):
    mode = 'rb'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('record', 'samples', 'output', 'release'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--release-sha256', required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    assert output == V / 'gen55_actual_resources_attempt2_r1' and not output.exists()
    assert M.is_mount() and output.parent.stat().st_dev == M.stat().st_dev
    assert not os.statvfs(M).f_flag & os.ST_RDONLY
    for name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
        os.environ[name] = '1'
    os.sched_setaffinity(0, {min(os.sched_getaffinity(0))})
    original_raw = ORIGINAL.read_bytes()
    assert hashlib.sha256(original_raw).hexdigest() == ORIGINAL_SHA
    spec = importlib.util.spec_from_file_location('gen55_attempt2_sealed_resource_analyzer', ORIGINAL)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    bag_path = (args.record / 'keyframes.bag').resolve()
    manifest_path = args.record / 'manifest.json'
    manifest = json.loads(manifest_path.read_bytes())
    assert manifest['state'] == 'complete' and manifest['recorder_closed'] is True
    entry = next(row for row in manifest['files'] if row['path'] == 'keyframes.bag')
    initial_identity = identity(bag_path)
    assert initial_identity[2] == entry['size_bytes']
    cache = {'source_read_passes': 0, 'bag_decode_passes': 0}
    normal_sha = module.sha
    def cached_sha(path):
        if Path(path).resolve() != bag_path:
            return normal_sha(path)
        assert identity(bag_path) == initial_identity, 'sealed bag identity changed'
        if 'sha256' not in cache:
            buffer = BagBuffer()
            digest = hashlib.sha256()
            with bag_path.open('rb') as source:
                cache['source_read_passes'] += 1
                for block in iter(lambda: source.read(1024 * 1024), b''):
                    digest.update(block)
                    buffer.write(block)
            cache['sha256'] = digest.hexdigest()
            assert cache['sha256'] == entry['sha256'], 'sealed bag digest differs'
            assert identity(bag_path) == initial_identity, 'sealed bag changed during single read'
            buffer.seek(0)
            cache['buffer'] = buffer
        return cache['sha256']
    module.sha = cached_sha
    original_checked = module.checked_file
    def checked(path, references, expected=None):
        references[str(Path(__file__).resolve())] = normal_sha(__file__)
        return original_checked(path, references, expected)
    module.checked_file = checked
    def storage(path):
        assert path == output and not path.exists()
        assert M.is_mount() and path.parent.stat().st_dev == M.stat().st_dev
        assert not os.statvfs(M).f_flag & os.ST_RDONLY
        return M.stat().st_dev
    module.storage = storage
    original_base = module.reviewed_base
    def reviewed_base(record, references):
        base = original_base(record, references)
        normal_reader = base.read_bag
        normal_bag = base.rosbag.Bag
        def memory_bag(filename, mode='r', *positional, **keywords):
            assert Path(filename).resolve() == bag_path and mode == 'r'
            assert cache['bag_decode_passes'] == 0
            cached_sha(bag_path)
            cache['bag_decode_passes'] += 1
            return normal_bag(cache['buffer'], 'r', *positional, **keywords)
        def read_once():
            base.rosbag.Bag = memory_bag
            try:
                data = normal_reader()
                cache['decoded'] = data
                return data
            finally:
                base.rosbag.Bag = normal_bag
        base.read_bag = read_once
        return base
    module.reviewed_base = reviewed_base
    began, cpu_began = time.perf_counter(), time.process_time()
    captured = io.StringIO()
    status = {'passed': False}
    with tempfile.TemporaryDirectory(prefix='gen55_resources_attempt2_tmp_', dir=V) as temporary:
        os.environ['TMPDIR'] = temporary
        try:
            with contextlib.redirect_stdout(captured):
                module.main()
            assert cache['source_read_passes'] == cache['bag_decode_passes'] == 1
            assert identity(bag_path) == initial_identity
            status['passed'] = True
        except Exception as exc:
            status.update(error_type=type(exc).__name__, error=str(exc))
            output.mkdir(exist_ok=True)
            if cache.get('decoded') is not None and not (output / 'bag_timing_evidence.json').exists():
                module.write(output, 'bag_timing_evidence.json', cache['decoded'])
            raise
        finally:
            if output.exists():
                status.update(adapter=str(Path(__file__).resolve()), adapter_sha256=normal_sha(__file__),
                    original_analyzer=str(ORIGINAL), original_analyzer_sha256=ORIGINAL_SHA,
                    bag=str(bag_path), bag_sha256=cache.get('sha256'), bag_source_identity=initial_identity,
                    source_read_passes=cache['source_read_passes'], bag_decode_passes=cache['bag_decode_passes'],
                    source_unchanged=identity(bag_path) == initial_identity,
                    elapsed_sec=time.perf_counter() - began, cpu_sec=time.process_time() - cpu_began,
                    maximum_rss_bytes=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024,
                    affinity_cpus=sorted(os.sched_getaffinity(0)), numerical_threads=1,
                    original_stdout=captured.getvalue(), output=str(output),
                    source_or_sealed_helper_changed=False, ROS_initialized=False, network_contacted=False,
                    temperature_queried=False, hardware_contacted=False)
                module.write(output, 'adapter_run.json', status)
    print(captured.getvalue(), end='')
    print(json.dumps({'adapter_run': str(output / 'adapter_run.json'), 'sha256': normal_sha(output / 'adapter_run.json'),
                      'elapsed_sec': status['elapsed_sec'], 'maximum_rss_bytes': status['maximum_rss_bytes']}))


if __name__ == '__main__':
    main()
