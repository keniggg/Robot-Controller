"""Read-only closure of exact direct-exec GEN62 verification processes."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time

ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_verification_r1/formal')


def read(path):
    return json.loads(Path(path).read_bytes())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def process(pid):
    try:
        value = (Path('/proc')/str(pid)/'stat').read_text()
    except FileNotFoundError:
        return None
    fields = value[value.rfind(')')+2:].split()
    return dict(pid=int(pid), state=fields[0], ppid=int(fields[1]),
                pgid=int(fields[2]), session=int(fields[3]), start_ticks=int(fields[19]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result', type=Path, required=True)
    parser.add_argument('--launcher-receipt', type=Path, required=True)
    args = parser.parse_args()
    assert Path('/media/zhuyupei/SanStand').is_mount()
    result = args.result.resolve(strict=True)
    assert result == ROOT/'full_r1/result.json'
    launcher_path = args.launcher_receipt.resolve(strict=True)
    assert launcher_path.parent == result.parent
    d = read(result)
    owned_path = result.parent/'owned_workers.json'
    coordinator_path = result.parent/'coordinator_identity.json'
    owned, coordinator, launcher = map(read, (owned_path, coordinator_path, launcher_path))
    inputs = {str(p): sha(p) for p in (result, owned_path, coordinator_path, launcher_path,
                                      result.parent/'plan.json', Path(__file__).resolve())}
    assert d['mode'] == 'full'
    assert owned['plan_sha256'] == coordinator['plan_sha256'] == d['plan_sha256'] == sha(result.parent/'plan.json')
    assert launcher['result_sha256'] == sha(result) and launcher['plan_sha256'] == d['plan_sha256']
    assert type(launcher['session_id']) is int and launcher['session_id'] > 0
    assert type(launcher['exit_code']) is int and launcher['exit_code'] == (0 if d['passed'] else 1)
    assert isinstance(launcher['tool_chunk_id'], str) and launcher['tool_chunk_id']
    assert coordinator['durable_unit'] is None
    assert owned['coordinator_pid'] == coordinator['pid']
    assert len(owned['workers']) == d['worker_count']
    assert sorted(row['index'] for row in owned['workers']) == list(range(d['worker_count']))
    rows, groups, sessions = [], set(), set()
    for role, obj in [('coordinator', coordinator)]+[('worker', x) for x in owned['workers']]:
        pid, expected = obj['pid'], obj['identity']['start_ticks']
        assert obj['identity']['pid'] == pid and expected > 0
        groups.add(obj['identity']['pgid'])
        sessions.add(obj['identity']['session'])
        current = process(pid)
        observed = None if current is None else current['start_ticks']
        rows.append(dict(role=role, pid=pid, expected_start_ticks=expected,
                         current_start_ticks=observed, exact_process_residual=observed == expected))
    residual = []
    for entry in Path('/proc').iterdir():
        if entry.name.isdecimal():
            current = process(int(entry.name))
            if current and (current['pgid'] in groups or current['session'] in sessions):
                residual.append(current)
    assert not any(row['exact_process_residual'] for row in rows)
    assert not residual and not d['residual_offline_process_groups']
    worker_children = []
    for relative, digest in d['worker_result_sha256'].items():
        path = Path(relative)
        if not path.is_absolute():
            path = result.parent/path
        assert result.parent in path.parents and sha(path) == digest
        worker = read(path)
        assert worker['plan_sha256'] == d['plan_sha256'] and worker['direct_child_reaped']
        assert not worker['direct_child_cleanup_signals']
        worker_children.append(dict(path=str(path), sha256=digest,
            worker_index=worker['worker_index'], direct_child_reaped=True, cleanup_signals=[]))
    assert len(worker_children) == d['worker_count']
    assert all(sha(path) == digest for path, digest in inputs.items())
    report = dict(passed=True, kind='GEN62_exact_owned_offline_process_exit_receipt',
        recorded_ns=time.time_ns(), helper_sha256=sha(__file__), result_sha256=sha(result),
        verification_passed=d['passed'], verification_summary=d['summary'],
        launcher_receipt=str(launcher_path), launcher_receipt_sha256=sha(launcher_path),
        exec_session_id=launcher['session_id'], exec_exit_code=launcher['exit_code'],
        tool_chunk_id=launcher['tool_chunk_id'], unit_name=None,
        unit_completion_evidence='direct_exec_tool_completion_and_exact_proc_identity_scan',
        shared_systemd_invocation_used_as_completion_evidence=False,
        owned_processes=rows, owned_pgids=sorted(groups), owned_sessions=sorted(sessions),
        observed_residual_processes=residual, reported_residual_groups=d['residual_offline_process_groups'],
        worker_children=worker_children, input_sha256=inputs,
        ROS_or_hardware_touched=False, motion_commands_sent=False, temperature_queried=False)
    path = result.parent/'final_process_receipt.json'
    with path.open('x') as stream:
        json.dump(report, stream, indent=2)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    print(json.dumps(dict(passed=True, output=str(path), sha256=sha(path))))


if __name__ == '__main__':
    main()
