#!/usr/bin/env python3
"""Read-only ROS parameter latency audit; never construct or start a ROS node."""
import argparse
from copy import copy
from dataclasses import asdict, is_dataclass
import hashlib
import json
from pathlib import Path
import statistics
import sys
import time

import numpy as np

WORK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORK / 'src/alicia_flexible_grasp_supervisor/tests'))
import test_remote_grasp6d_node as fixture


def canonical(value):
    if isinstance(value, np.ndarray):
        return {'array': value.tolist(), 'dtype': str(value.dtype)}
    if is_dataclass(value):
        return canonical(asdict(value))
    if isinstance(value, dict):
        return {str(k): canonical(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [canonical(v) for v in value]
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return repr(value)


def summary(rows, key):
    vals = [row[key] for row in rows]
    return {'count': len(vals), 'min': min(vals), 'median': statistics.median(vals),
            'max': max(vals), 'sum': sum(vals)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--rounds', type=int, default=12)
    args = parser.parse_args()
    mount = Path('/media/zhuyupei/SanStand')
    if not mount.is_mount() or mount not in args.output.resolve().parents:
        raise RuntimeError('external log mount required')
    args.output.mkdir(parents=True, exist_ok=True)
    remote = fixture.remote_node
    original_get = remote.rospy.get_param
    node = fixture.make_processing_node()
    node.candidate_max_target_distance_m = 0.04
    node.candidate_min_relative_z_m = -0.015
    node.candidate_max_relative_z_m = 0.08
    # Each call stages only an unstarted local object; fixture publishers are in-memory.
    roots = ('/grasp_6d/remote', '/mujoco_digital_twin', '/grasp', '/gripper')
    results = {'started_wall_ns': time.time_ns(), 'read_only': True,
               'node_started': False, 'rounds': [], 'method': 'real _stage_runtime_params on unstarted fixture'}
    source = WORK / 'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py'
    results['source_sha256'] = hashlib.sha256(source.read_bytes()).hexdigest()
    try:
        for number in range(args.rounds):
            calls = []
            captured = {}
            def measured(name, *default):
                if not any(name == r or name.startswith(r + '/') for r in roots):
                    raise RuntimeError('unexpected parameter root: ' + name)
                started = time.perf_counter()
                value = original_get(name, *default)
                calls.append({'name': name, 'elapsed_ms': (time.perf_counter() - started) * 1000.0})
                captured[name] = value
                return value
            remote.rospy.get_param = measured
            staged = copy(node)
            started = time.perf_counter()
            config = remote.RemoteGrasp6DNode._stage_runtime_params(staged)
            elapsed = (time.perf_counter() - started) * 1000.0
            results['rounds'].append({'round': number, 'elapsed_ms': elapsed,
                'get_param_ms': sum(row['elapsed_ms'] for row in calls), 'calls': calls})
            # Replay the same captured parameters through the real method, no network.
            def cached_exact(name, *default):
                if name in captured:
                    return captured[name]
                if default:
                    return default[0]
                raise KeyError(name)
            remote.rospy.get_param = cached_exact
            local = copy(node)
            started = time.perf_counter()
            local_config = remote.RemoteGrasp6DNode._stage_runtime_params(local)
            replay_elapsed = (time.perf_counter() - started) * 1000.0
            comparable = lambda v: canonical({k: val for k, val in v.__dict__.items()
                if node.__dict__.get(k) is not val})
            equal = comparable(staged) == comparable(local) and canonical(config) == canonical(local_config)
            results['rounds'][-1].update(replay_elapsed_ms=replay_elapsed, exact_replay_equal=equal)
            # Proposed request-local namespace snapshot: fresh server reads every round.
            root_calls = []
            root_values = {}
            for name in roots:
                started = time.perf_counter()
                root_values[name] = original_get(name, {})
                root_calls.append({'name': name, 'elapsed_ms': (time.perf_counter() - started) * 1000.0})
            def namespace_snapshot(name, *default):
                for root in roots:
                    if name == root or name.startswith(root + '/'):
                        value = root_values[root]
                        try:
                            for part in name[len(root):].strip('/').split('/') if name != root else ():
                                value = value[part]
                            return value
                        except (KeyError, TypeError):
                            if default:
                                return default[0]
                            raise KeyError(name)
                raise KeyError(name)
            remote.rospy.get_param = namespace_snapshot
            bundled = copy(node)
            started = time.perf_counter()
            bundled_config = remote.RemoteGrasp6DNode._stage_runtime_params(bundled)
            bundled_ms = (time.perf_counter() - started) * 1000.0
            bundled_equal = comparable(staged) == comparable(bundled) and canonical(config) == canonical(bundled_config)
            results['rounds'][-1].update(namespace_calls=root_calls,
                namespace_elapsed_ms=sum(x['elapsed_ms'] for x in root_calls) + bundled_ms,
                namespace_equal=bundled_equal)
        results['summaries'] = {key: summary(results['rounds'], key) for key in
            ('elapsed_ms', 'get_param_ms', 'replay_elapsed_ms', 'namespace_elapsed_ms')}
        results['all_outputs_equal'] = all(r['exact_replay_equal'] and r['namespace_equal'] for r in results['rounds'])
    finally:
        remote.rospy.get_param = original_get
        results['finished_wall_ns'] = time.time_ns()
        (args.output / 'result.json').write_text(json.dumps(results, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps({'summaries': results['summaries'], 'all_outputs_equal': results['all_outputs_equal'],
                      'parameter_calls_per_round': len(results['rounds'][0]['calls'])}))


if __name__ == '__main__':
    main()
