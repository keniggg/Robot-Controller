"""Read-only source-bound preparation and station failure diagnostics snapshot."""
import argparse
from collections import Counter
import json
from pathlib import Path
import re
import time

from monitor_gen45_actual_handoff_20261018 import (
    MOUNT, E, N, V, inside, digest, read_rows, numeric_time, json_after)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record', required=True, type=Path)
    parser.add_argument('--consumers', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    assert MOUNT.is_mount() and inside(args.record, E/'grasp_records')
    assert inside(args.consumers, N) and inside(args.output, V)
    assert args.output.resolve() != V.resolve()
    runner, runner_range = read_rows(args.record/'runner.log', 2*1024*1024)
    consumers, consumer_range = read_rows(args.consumers, 8*1024*1024)
    selection = None
    mode_evidence = None
    for text, reference in runner:
        match = re.search(r'^MODE_ATTEMPT selection=(\{.*\}) minimum_stamp_ns=(\d+)', text)
        if match:
            selection = json.loads(match.group(1))
            mode_evidence = reference
    assert selection and type(selection.get('stamp_ns')) is int
    start = selection['stamp_ns']/1e9
    requests = []
    stations = []
    phases = []
    for text, reference in consumers:
        stamp = numeric_time(text)
        if stamp is None or stamp < start:
            continue
        phase = re.search(r'\[Grasp\] (\S+) (.*)', text)
        if phase:
            phases.append(dict(stamp_sec=stamp, state=phase.group(1),
                message=phase.group(2), evidence=reference))
        station = json_after(text, 'Station handoff diagnostic: ')
        if station is not None:
            stations.append(dict(stamp_sec=stamp, diagnostic=station, evidence=reference))
        metrics = json_after(text, 'remote 6D pipeline metrics: ')
        if metrics is not None and metrics.get('event') == 'request_completed':
            keys = ('request_id', 'generation', 'target_epoch', 'status', 'event',
                'primary_failure', 'drop_reason', 'error', 'error_type', 'error_traceback',
                'end_to_end_ms', 'result_age_ms', 'ros_prepare_ms', 'ros_prepare_stages_ms',
                'wsl_total_ms', 'transport_ms', 'snapshot_evidence',
                'request_preparation_profile')
            requests.append(dict(stamp_sec=stamp, evidence=reference,
                                 metrics={key:metrics[key] for key in keys if key in metrics}))
    errors = [request for request in requests if request['metrics'].get('error')]
    value = dict(kind='read_only_actual_preparation_and_station_diagnostic_snapshot',
        captured_ns=time.time_ns(), record=str(args.record), selection=selection,
        mode_attempt_evidence=mode_evidence, selection_start_stamp_ns=selection['stamp_ns'],
        completed_request_count=len(requests), request_error_count=len(errors),
        request_error_counts=dict(Counter(request['metrics']['error'] for request in errors)),
        first_completed_request_error=errors[0] if errors else None,
        request_preparation_events=requests, station_failure_diagnostics=stations,
        task_phases=phases,
        first_Task_failure=next((phase for phase in phases if phase['state'] == 'FAILED'), None),
        source_ranges=[runner_range, consumer_range],
        helper_sha256=digest(Path(__file__).read_bytes()),
        ROS_or_hardware_access=False, temperature_read=False,
        source_or_record_modified=False, execution_authorized=False,
        interpretation='Candidate/request failures are not Task terminal. Subscriber diagnostic timestamps describe actual callbacks; bag timing is not inferred.')
    args.output.mkdir(exist_ok=True)
    path = args.output/('diagnostics_'+str(value['captured_ns'])+'.json')
    with path.open('x') as handle:
        json.dump(value, handle, ensure_ascii=False, indent=2, allow_nan=False);handle.write('\n')
    assert json.loads(path.read_text()) == value
    print(json.dumps(dict(path=str(path), sha256=digest(path.read_bytes()),
        selection=selection, completed_requests=len(requests), errors=len(errors),
        first_error=errors[0]['metrics'].get('error') if errors else None,
        station_failure_diagnostic_count=len(stations), task_phases=len(phases)), ensure_ascii=False))


if __name__ == '__main__':
    main()
