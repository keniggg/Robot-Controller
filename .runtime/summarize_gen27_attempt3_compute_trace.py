"""Bounded offline trace: contact rejection, yielded view, and failed IK search."""
import hashlib
import json
import os
from pathlib import Path
import re
import time

W = Path(__file__).resolve().parent.parent
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N = E / 'ros_log/full_latest_gen22_restart_20261015T003700Z'
OUT = N / 'verification/gen27_attempt3_compute_wait_trace'
START, END = 1792050955., 1792051048.


def sha(data):
    return hashlib.sha256(data).hexdigest()


def bounded_lines(path, watch=False):
    # Consumers can continue writing. Bind the exact read prefix, never claim
    # its digest describes a later complete live file.
    with path.open('rb') as stream:
        size = os.fstat(stream.fileno()).st_size
        raw = stream.read(size)
    assert len(raw) == size
    bound = dict(path=str(path), read_prefix_bytes=size, read_prefix_sha256=sha(raw),
                 whole_file_identity_claimed=False)
    rows = []
    for number, line in enumerate(raw.decode('utf-8', errors='strict').splitlines(), 1):
        if watch:
            try:
                stamp = float(line.split()[0])
            except (ValueError, IndexError):
                continue
        else:
            match = re.search(r'\[(179\d+\.\d+)\]', line)
            if match is None:
                continue
            stamp = float(match.group(1))
        if START <= stamp <= END:
            rows.append((stamp, number, line))
    return bound, rows


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    assert not os.statvfs(E).f_flag & os.ST_RDONLY
    watch_source, watch = bounded_lines(N / 'gen27_attempt_3_watch.log', True)
    compute_source, compute = bounded_lines(N / 'gen27_deployment/consumers.log')
    raw_evidence, failures, tasks, completions, contacts = [], [], [], [], []
    for stamp, number, line in watch:
        if 'GRASP_STATE' in line:
            tasks.append(dict(stamp_sec=stamp, line=line))
        if stamp >= 1792050998. and '/motion_gateway check_pose_strict result' in line:
            failures.append(dict(stamp_sec=stamp, line=line))
        if ('GRASP_STATE' in line or
                stamp >= 1792050998. and any(text in line for text in
                    ('/motion_gateway check_pose_strict result',
                     'Unable to sample any valid states for goal tree'))):
            raw_evidence.append(dict(source='watch', line_number=number, line=line))
    for stamp, number, line in compute:
        if 'remote 6D final candidate rejected:' in line:
            contacts.append(dict(stamp_sec=stamp, line=line))
            raw_evidence.append(dict(source='consumers', line_number=number, line=line))
        if 'remote 6D pipeline metrics:' in line:
            row = json.loads(line.split('remote 6D pipeline metrics:', 1)[1])
            if row.get('event') == 'request_completed' and row.get('request_id') in (74, 75, 76):
                completions.append(dict(stamp_sec=stamp, metrics=row))
                raw_evidence.append(dict(source='consumers', line_number=number, line=line))
    audit_path = N / 'verification/gen27_attempt3_initial_reason/1792050998644975185_gate_audit.json'
    audit_raw = audit_path.read_bytes()
    audit = json.loads(audit_raw)
    assert len(contacts) == 3, len(contacts)
    assert failures and all('OBSERVATION_PLANNING_TIMEOUT' in row['line'] for row in failures)
    assert len([row for row in tasks if 'station_compute_wait_expired_or_future' in row['line']]) == 2
    metrics = {row['metrics']['request_id']: row['metrics'] for row in completions}
    assert metrics[74]['stage_counts']['preview']['passed'] == 0
    assert metrics[75]['transport_ms'] == 0.0
    assert metrics[75]['status'] == 'TARGET_LOST'
    report = dict(schema=1, created_ns=time.time_ns(),
        conclusion='Contact search yielded a supplementary view; its independent local observation planning made repeated unsuccessful MoveIt calls until the unchanged station deadline. No deadlock or image-wait cause is established.',
        record=str(E / 'grasp_records/20261015T075503.791411Z_24849e8304a44341adebca3867d29fdd'),
        interval_sec=[START, END], source_prefixes=[watch_source, compute_source],
        yield_audit_source=dict(path=str(audit_path), sha256=sha(audit_raw)),
        yield_audit=audit, task_transitions=tasks, contact_rejections=contacts,
        supplementary_moveit=dict(count=len(failures),
            first_result_stamp_sec=failures[0]['stamp_sec'],
            last_result_stamp_sec=failures[-1]['stamp_sec'],
            all_results='OBSERVATION_PLANNING_TIMEOUT / moveit_error_code=-6',
            result_spacing_sec=[round(b['stamp_sec']-a['stamp_sec'], 6)
                for a,b in zip(failures, failures[1:])]),
        request_completions=completions,
        limitations=['No successful contact plan, physical grasp, or globally unreachable goal is claimed.',
                    'Individual timeout results do not establish geometric unreachability.',
                    'No timeout, tolerance, production source, ROS state or hardware setting was changed.'],
        production_modified=False, ros_master_contact=False, hardware_commands_sent=False,
        temperature_queried=False, script_sha256=sha(Path(__file__).read_bytes()))
    OUT.mkdir(exist_ok=False)
    outputs = {}
    for name, value in (('findings.json', report), ('raw_selected_lines.json', raw_evidence)):
        raw = (json.dumps(value, ensure_ascii=False, indent=2)+'\n').encode()
        path = OUT / name
        path.write_bytes(raw)
        assert path.read_bytes() == raw
        outputs[name] = dict(bytes=len(raw), sha256=sha(raw))
    receipt = dict(schema=1, verified_readback=True, output_files=outputs,
                   no_hardware_access=True, source_prefixes=report['source_prefixes'])
    (OUT / 'receipt.json').write_text(json.dumps(receipt, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(dict(output=str(OUT), moveit_failures=len(failures),
        request_75_end_to_end_ms=metrics[75]['end_to_end_ms'], receipt=receipt['output_files']),
        ensure_ascii=False))


if __name__ == '__main__':
    main()
