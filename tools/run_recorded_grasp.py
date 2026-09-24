#!/usr/bin/env python3
"""Record one authorized grasp command and queue only closed recording files.

This wrapper does not choose a target, change modes, enable/disable joints, or
call any robot service. The command following ``--`` is supplied by the caller.
Signals sent to this wrapper stop only its recorder; an already running grasp
command is allowed to finish before its log is sealed and queued.
"""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def atomic_json(path, payload):
    path = Path(path)
    temporary = path.with_name(path.name + '.tmp')
    with temporary.open('w', encoding='utf-8') as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def observe_task_state(outcome, message, minimum_stamp_ns):
    """Require a fresh active -> inactive transition, ignoring old latched state."""
    stamp = int(message.get('stamp_ns', 0))
    if stamp < minimum_stamp_ns or stamp <= 0:
        return
    if message.get('active'):
        outcome['active_seen'] = True
        outcome['last_active_stamp_ns'] = max(stamp, outcome.get('last_active_stamp_ns', 0))
        return
    if (not outcome.get('active_seen') or outcome.get('task_result')
            or stamp < outcome.get('last_active_stamp_ns', 0)):
        return
    state = str(message.get('state', ''))
    reason = str(message.get('message', ''))
    if state == 'SUCCESS' and message.get('success') is True:
        result = 'success'
    elif state == 'HOLDING' and message.get('success') is False:
        result = 'unknown'
    elif state == 'FAILED' and message.get('success') is False:
        result = 'failure'
    elif state == 'EMERGENCY_STOP' or (state == 'IDLE' and reason == 'stop requested'):
        result = 'interrupted'
    else:
        return
    outcome['task_result'] = {
        'result': result, 'reason': reason,
        'evidence': {'source': '/grasp/state', 'message': dict(message)},
    }


_SERVICE_RESULT = re.compile(r'^GRASP_RESULT success=(True|False) message=(.*)$')
_PLANNING_FAILURES = (
    'DIRECT_PLANNING_TERMINAL ', 'NO_FRESH_MODE_PREVIEW ',
    'MODE_EXECUTION_AUTHORITY_UNAVAILABLE ',
    'MODE_BINDING_VERIFY_BEFORE_START_FAILED ',
    'CANDIDATE_COMPUTATION_START success=False ',
)


def classify_result(recorder_outcome, runner_log, interrupted=False, returncode=None):
    """Use explicit task evidence. Exit status, recording end and names prove nothing."""
    observations = []
    state_result = recorder_outcome.get('task_result')
    if state_result:
        observations.append(state_result)
    planning_failures = []
    response_unknown = False
    for line in runner_log.splitlines():
        match = _SERVICE_RESULT.fullmatch(line)
        if match:
            observations.append({
                'result': ('unknown' if match.group(2).startswith('GRASP_HOLD_UNVERIFIED:')
                           else ('success' if match.group(1) == 'True' else 'failure')),
                'reason': match.group(2),
                'evidence': {'source': 'StartGrasp service response in runner.log', 'line': line},
            })
        elif line.startswith(_PLANNING_FAILURES):
            planning_failures.append(line)
        elif line.startswith('RUNNER_EXCEPTION ') and 'execution_response_unknown=True' in line:
            response_unknown = True
    evidence = {'observations': observations, 'runner_returncode': returncode,
                'recorder_stop_reason': recorder_outcome.get('stop_reason', '')}
    results = {entry['result'] for entry in observations}
    if len(results) > 1:
        return 'unknown', 'conflicting actual task results; manual confirmation required', evidence
    if observations:
        result = observations[-1]['result']
        reason = '' if result == 'success' else observations[-1]['reason']
        return result, reason, evidence
    if response_unknown:
        return 'unknown', 'grasp service response unavailable; actual outcome unknown', evidence
    if planning_failures:
        evidence['planning_failure'] = planning_failures[-1]
        return 'failure', planning_failures[-1], evidence
    if interrupted or recorder_outcome.get('interrupted'):
        return 'interrupted', 'recording interrupted without a confirmed task result', evidence
    return 'unknown', 'no actual task result observed', evidence


def _read_json(path):
    try:
        return json.loads(Path(path).read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return {}


def _await_recorder_ready(process, directory, timeout, interrupted):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if interrupted['signal'] or process.poll() is not None:
            return False
        ready = _read_json(directory / 'recorder_ready.json')
        if ready.get('pid') == process.pid and ready.get('record_dir') == str(directory):
            return True
        time.sleep(.1)
    return False


def _closed_recording_files(directory, outcome):
    """No recursive directory sweep: only exact files produced by this invocation."""
    files = []
    if outcome.get('bag_closed'):
        files.extend(outcome.get('closed_files', []))
    for name in ('runner.log', 'recorder.log', 'recorder_ready.json',
                 'recorder_outcome.json', 'command_result.json', 'authorization.json'):
        if (directory / name).is_file():
            files.append(name)
    return sorted(set(files))


def main(argv=None):
    from grasp_archive import begin_record, finalize_record, load_config

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', default=os.environ.get(
        'GRASP_RECORD_ROOT', str(Path(__file__).resolve().parents[1] / '.grasp_records')))
    parser.add_argument('--config', help='archive configuration (default: ROOT/archive_config.json)')
    parser.add_argument('--duration', type=float, default=600.,
                        help='unchanged keyframe recorder duration, seconds (default: 600)')
    parser.add_argument('--ready-timeout', type=float, default=30.)
    parser.add_argument('--min-free-bytes', type=int,
                        help='override recording startup free-space reserve')
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command:
        parser.error('an explicitly authorized command is required after --')
    if not 0 < args.duration <= 1800:
        parser.error('duration must be in (0, 1800] seconds')
    if args.ready_timeout <= 0 or (args.min_free_bytes is not None and args.min_free_bytes < 0):
        parser.error('ready timeout must be positive and free-space reserve nonnegative')
    root = Path(args.root).expanduser().resolve()
    config = load_config(args.config or root / 'archive_config.json')
    if args.min_free_bytes is not None:
        config['min_free_bytes'] = args.min_free_bytes
    directory = begin_record(root, metadata={
        'command': command, 'recording_duration_seconds': args.duration,
        'owner_pid': os.getpid(), 'launcher': Path(__file__).name,
    }, config=config)
    print('GRASP_RECORD_DIR=%s' % directory, flush=True)
    interrupted = {'signal': None}
    previous_handlers = {}

    def request_recorder_stop(signum, _frame):
        interrupted['signal'] = signum

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.signal(signum, request_recorder_stop)
    recorder = None
    runner = None
    runner_returncode = None
    launcher_error = ''
    try:
        with (directory / 'recorder.log').open('w') as recorder_log, \
                (directory / 'runner.log').open('w') as runner_log:
            recorder = subprocess.Popen([
                sys.executable, '-u', str(Path(__file__).with_name('record_grasp_keyframes_readonly.py')),
                '--record-dir', str(directory), '--output', str(directory / 'keyframes.bag'),
                '--duration', str(args.duration), '--defer-finalize',
                '--config', str(Path(args.config).expanduser().resolve() if args.config
                                else root / 'archive_config.json'),
                '--min-free-bytes', str(config['min_free_bytes']),
            ], stdout=recorder_log, stderr=subprocess.STDOUT, start_new_session=True)
            if not _await_recorder_ready(recorder, directory, args.ready_timeout, interrupted):
                launcher_error = 'recorder did not become ready; grasp command was not started'
                print(launcher_error, file=sys.stderr, flush=True)
            else:
                runner_environment = dict(os.environ, GRASP_RECORD_DIR=str(directory))
                runner = subprocess.Popen(command, stdout=runner_log, stderr=subprocess.STDOUT,
                                          start_new_session=True, env=runner_environment)
                stop_sent = False
                while runner.poll() is None:
                    if interrupted['signal'] and not stop_sent:
                        if recorder.poll() is None:
                            recorder.send_signal(signal.SIGINT)
                        stop_sent = True
                        print('Recording interruption requested; waiting for the existing grasp '
                              'command without sending it a signal.', flush=True)
                    time.sleep(.2)
                runner_returncode = runner.returncode
            # Preserve the existing duration and 3-second post-task tail. A
            # normally exiting runner does not shorten the recording window.
            # Only an explicit interruption/startup error stops this recorder.
            recorder_stop_sent = False
            while recorder.poll() is None:
                if (interrupted['signal'] or launcher_error) and not recorder_stop_sent:
                    recorder.send_signal(signal.SIGINT)
                    recorder_stop_sent = True
                time.sleep(.2)
            recorder.wait()
    except Exception as exc:
        launcher_error = '%s: %s' % (type(exc).__name__, exc)
        # No runner terminate/kill: keep an already authorized control command alive.
        if runner is not None and runner.poll() is None:
            runner_returncode = runner.wait()
        if recorder is not None and recorder.poll() is None:
            recorder.send_signal(signal.SIGINT)
            recorder.wait()
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
    outcome = _read_json(directory / 'recorder_outcome.json')
    log = (directory / 'runner.log').read_text(encoding='utf-8', errors='replace')
    # Recorder interruption and robot-task outcome are separate facts.
    # A startup failure never started a grasp command and remains unknown.
    outcome_for_result = dict(outcome)
    outcome_for_result['interrupted'] = bool(interrupted['signal'])
    result, reason, evidence = classify_result(
        outcome_for_result, log, bool(interrupted['signal']), runner_returncode)
    evidence['launcher_error'] = launcher_error
    atomic_json(directory / 'command_result.json', {
        'result': result, 'failure_reason': reason, 'evidence': evidence,
        'finished_at': utc_now(), 'recorder_returncode': recorder.returncode if recorder else None,
        'closed_logs': ['runner.log', 'recorder.log'],
        'runner_finished': runner is not None and runner.poll() is not None,
    })
    if not outcome.get('bag_closed'):
        # A killed/crashed writer may leave a recoverable bag. Never queue or
        # prune it until an explicit recovery confirms that the file is closed.
        print('RECORDING_NOT_FINALIZED: bag close was not confirmed; preserve %s' % directory,
              file=sys.stderr, flush=True)
        return 2
    finalize_record(directory, result=result, failure_reason=reason, evidence=evidence,
                    closed_files=_closed_recording_files(directory, outcome))
    print('GRASP_RECORD_QUEUED result=%s path=%s' % (result, directory), flush=True)
    return 2 if launcher_error else (runner_returncode if runner_returncode is not None else 2)


if __name__ == '__main__':
    sys.exit(main())
