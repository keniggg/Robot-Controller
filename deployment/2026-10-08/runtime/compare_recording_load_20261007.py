"""Bounded passive A/B/A experiment; no selection, arm command or device query.

Observe existing SDK diagnostics and ROS feedback, retaining summaries in RAM
until all phases finish. Only B adds the existing keyframe recorder on SanStand.
Existing operational logs remain active in every phase.
"""
from datetime import datetime, timezone
import hashlib
import json
import os
import signal
import xmlrpc.client
from pathlib import Path
import subprocess
import sys
import threading
import time

WORK = Path(__file__).resolve().parents[1]
EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
sys.path.insert(0, str(WORK / 'tools'))
from grasp_archive import begin_record, load_config, finalize_record


def validate_recording_load(returncode, outcome, bag_bytes):
    """An empty/failed recorder is never a recording-on load observation."""
    reasons = []
    if returncode != 0:
        reasons.append('recorder_nonzero_or_running')
    if outcome.get('bag_closed') is not True:
        reasons.append('bag_not_closed')
    if outcome.get('stop_reason') != 'duration_elapsed' or outcome.get('interrupted'):
        reasons.append('recording_interrupted')
    duration = None
    try:
        duration = (datetime.fromisoformat(outcome['finished_at'].replace('Z', '+00:00'))
                    - datetime.fromisoformat(outcome['started_at'].replace('Z', '+00:00'))).total_seconds()
    except (KeyError, ValueError, TypeError, AttributeError):
        reasons.append('missing_or_invalid_recording_duration')
    if duration is not None and duration < 45.:
        reasons.append('short_recording')
    count = outcome.get('keyframe_count', 0)
    if not isinstance(count, int) or isinstance(count, bool) or count <= 0 or bag_bytes <= 0:
        reasons.append('no_actual_keyframe_image_load')
    return dict(valid=not reasons, reasons=reasons, keyframe_count=count,
                duration_seconds=duration, bag_bytes=bag_bytes,
                scope='Measured exact detected keyframes only; nonempty is not proof of full-rate raw RGB-D load.')


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    out = EXT / 'ros_log' / ('recording_load_aba_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
    out.mkdir()
    os.environ['ROS_LOG_DIR'] = str(out / 'ros')
    (WORK / '.runtime/current_recording_load_aba_20261007.json').write_text(json.dumps(dict(output=str(out)))+'\n')
    import rospy
    from sensor_msgs.msg import JointState
    rospy.init_node('passive_recording_load_comparison', anonymous=True, disable_signals=True)
    rows, streams, sdk, phases = [], {'accepted': [], 'heartbeat': []}, {}, []
    lock = threading.Lock()

    def feedback(message, key):
        with lock:
            if len(streams[key]) < 30000:
                streams[key].append(dict(source_ns=message.header.stamp.to_nsec(),
                    receipt_ns=rospy.Time.now().to_nsec(), monotonic=time.monotonic(),
                    caller=message._connection_header.get('callerid')))

    subscribers = [rospy.Subscriber(topic, JointState, feedback, callback_args=key, queue_size=200)
        for key, topic in [('accepted', '/alicia_d/accepted_joint_states'), ('heartbeat', '/joint_states')]]

    class BoundedTransport(xmlrpc.client.Transport):
        def make_connection(self, host):
            connection = super().make_connection(host)
            connection.timeout = 1.0
            return connection

    master = xmlrpc.client.ServerProxy(os.environ.get('ROS_MASTER_URI', 'http://localhost:11311'), transport=BoundedTransport())

    def observe():
        start = time.monotonic()
        row = dict(monotonic=start, wall_ns=time.time_ns())
        try:
            code, message, value = master.getParam(rospy.get_name(), '/camera_read_diagnostics')
            if code != 1:
                raise RuntimeError(message)
            payload = json.loads(value['payload_json'])
            for entry in payload.get('entries', []):
                ns = int(entry['source_stamp_ns'])
                if ns not in sdk and len(sdk) < 12000:
                    e = entry.get('evidence', {})
                    sdk[ns] = dict(source_ns=ns, monotonic=entry['read_completed_monotonic'],
                        host_wait=e.get('host_wait'), stages_sec=e.get('stages_sec'),
                        result=e.get('result'), raw_frames=e.get('raw_frames'),
                        regional_reset_reason=e.get('regional_stationary_evidence', {}).get('global_history_reset_reason'))
        except Exception as exc:
            row['camera_metadata_error'] = repr(exc)
        row['metadata_read_seconds'] = time.monotonic() - start
        for name in ('cpu', 'memory', 'io'):
            row['psi_'+name] = Path('/proc/pressure/'+name).read_text()
        vm = dict(line.split() for line in Path('/proc/vmstat').read_text().splitlines())
        row['vmstat'] = {k:int(v) for k,v in vm.items() if k.startswith('allocstall') or k in
            ('pswpin','pswpout','pgmajfault','nr_dirty','nr_writeback','pgscan_direct','pgsteal_direct')}
        row['meminfo'] = {line.split(':')[0]:line.split(':',1)[1].strip()
            for line in Path('/proc/meminfo').read_text().splitlines()
            if line.split(':')[0] in ('MemAvailable','MemFree','Cached','Dirty','Writeback','SwapFree')}
        row['diskstats'] = [line.split() for line in Path('/proc/diskstats').read_text().splitlines()
            if line.split()[2].startswith(('sd', 'nvme'))]
        rows.append(row)

    def phase(name, seconds):
        start = time.monotonic()
        print('PHASE '+name, flush=True)
        while time.monotonic() - start < seconds:
            observe()
            time.sleep(.5)
        phases.append(dict(name=name, start=start, end=time.monotonic()))

    record, recorder, log, error, load_assessment = None, None, None, None, None
    try:
        phase('A1_without_extra_keyframe_recording', 45)
        root = EXT / 'grasp_records'
        config_path = root / 'archive_config.json'
        config = load_config(config_path)
        record = begin_record(root, metadata=dict(diagnostic_only=True, motion_authorized=False,
            experiment='stationary_recording_load_ABA', owner_pid=os.getpid()), config=config)
        log = (record / 'recorder.log').open('w')
        recorder = subprocess.Popen(['/usr/bin/python3', '-u', str(WORK / 'tools/record_grasp_keyframes_readonly.py'),
            '--record-dir', str(record), '--output', str(record / 'keyframes.bag'),
            '--duration', '45', '--defer-finalize', '--config', str(config_path)], stdout=log, stderr=subprocess.STDOUT)
        start = time.monotonic()
        while not (record / 'recorder_ready.json').exists():
            if recorder.poll() is not None or time.monotonic() - start > 15:
                raise RuntimeError('Diagnostic recorder startup failed; no motion was issued')
            observe()
            time.sleep(.1)
        phases.append(dict(name='B_startup', start=start, end=time.monotonic()))
        start = time.monotonic()
        print('PHASE B_keyframe_recording', flush=True)
        while recorder.poll() is None:
            if time.monotonic() - start > 70:
                raise RuntimeError('Owned diagnostic recorder exceeded its 45-second recording plus close reserve')
            observe()
            time.sleep(.5)
        phases.append(dict(name='B_keyframe_recording', start=start, end=time.monotonic()))
        log.close()
        outcome_file = record / 'recorder_outcome.json'
        outcome = json.loads(outcome_file.read_text()) if outcome_file.exists() else {}
        bag_path = record / 'keyframes.bag'
        load_assessment = validate_recording_load(recorder.returncode, outcome,
                                                  bag_path.stat().st_size if bag_path.is_file() else 0)
        if not load_assessment['valid']:
            raise RuntimeError('Recording-on contrast invalid: ' + repr(load_assessment['reasons']))
        phase('writeback_cooldown_not_a_baseline', 10)
        phase('A2_without_extra_keyframe_recording', 45)
    except Exception as exc:
        error = repr(exc)
    finally:
        if recorder is not None and recorder.poll() is None:
            recorder.send_signal(signal.SIGINT)
            try:
                recorder.wait(timeout=10)
            except subprocess.TimeoutExpired:
                error = (error or '') + '; owned diagnostic recorder still finishing, PID=' + str(recorder.pid)
        if log is not None and not log.closed:
            log.close()
    for sub in subscribers:
        sub.unregister()
    result = dict(diagnostic_only=True, hardware_commands_sent=False, configuration_changed=False,
        existing_operational_logs_continued=True, phases=phases, samples=rows,
        sdk_reads=list(sdk.values()), feedback=streams, record_dir=str(record), recorder_returncode=None if recorder is None else recorder.returncode, error=error,
        load_assessment=load_assessment,
        limits='45 second phases; no counterfactual proof from a short clean window; cache writeback may continue after B')
    (out / 'observations.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    outcome_path = None if record is None else record / 'recorder_outcome.json'
    outcome = json.loads(outcome_path.read_text()) if outcome_path is not None and outcome_path.exists() else {}
    if outcome.get('bag_closed'):
        finalize_record(record, result='unknown', failure_reason='diagnostic_only_recording_load_comparison_no_grasp',
            evidence=dict(diagnostic_only=True, motion_authorized=False, observations=str(out/'observations.json')),
            closed_files=sorted(set(outcome.get('closed_files', []) + ['recorder.log','recorder_ready.json','recorder_outcome.json'])))
    (out / 'SHA256SUMS').write_text(hashlib.sha256((out/'observations.json').read_bytes()).hexdigest()+'  observations.json\n')
    print('FINISHED '+str(out), flush=True)
    if error:
        raise RuntimeError(error)


if __name__ == '__main__':
    main()
