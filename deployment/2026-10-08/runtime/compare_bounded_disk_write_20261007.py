"""Bounded A/B/A disk-write diagnostic; no selection, arm command or device query.

Thirty-second phases compare existing operations with an extra 5.5MiB/s buffered
file writer. Retain at least 2GiB and cap writes at165MiB. No full grasp recorder
is started, no acquisition gates or existing recording reserves are modified.
"""
from datetime import datetime, timezone
import hashlib
import json
import os
import signal
import shutil
import xmlrpc.client
from pathlib import Path
import subprocess
import sys
import threading
import time

WORK = Path(__file__).resolve().parents[1]
EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
sys.path.insert(0, str(WORK / 'tools'))


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    out = EXT / 'ros_log' / ('bounded_disk_write_aba_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
    out.mkdir()
    os.environ['ROS_LOG_DIR'] = str(out / 'ros')
    (WORK / '.runtime/current_bounded_disk_write_aba_20261007.json').write_text(json.dumps(dict(output=str(out)))+'\n')
    import rospy
    from sensor_msgs.msg import JointState
    rospy.init_node('passive_bounded_disk_write_comparison', anonymous=True, disable_signals=True)
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

    error, write_timings, bytes_written = None, [], 0
    block = hashlib.shake_256(b'bounded disk IO diagnostic 20261007').digest(512 * 1024)
    limit_bytes, reserve_bytes = 165 * 1024**2, 2 * 1024**3
    assert shutil.disk_usage(out).free >= limit_bytes + reserve_bytes
    try:
        phase('A1_without_test_writes', 30)
        start = time.monotonic()
        print('PHASE B_bounded_buffered_disk_write_5p5_MiB_per_sec', flush=True)
        with (out / 'bounded_write_load.bin').open('xb') as stream:
            for index in range(330):
                desired = start + index / 11.
                while time.monotonic() < desired:
                    time.sleep(max(0., min(.02, desired-time.monotonic())))
                if time.monotonic() - start > 45:
                    raise RuntimeError('Write phase exceeded 30 seconds plus 15-second reserve')
                if not Path('/media/zhuyupei/SanStand').is_mount() or shutil.disk_usage(out).free < reserve_bytes + len(block):
                    raise RuntimeError('External disk reserve unavailable; no fallback')
                if index % 6 == 0:
                    observe()
                began = time.monotonic()
                count = stream.write(block)
                bytes_written += count
                write_timings.append(dict(monotonic=began, seconds=time.monotonic()-began, bytes=count))
                if count != len(block):
                    raise RuntimeError('Short write')
            stream.flush()
            flush_start = time.monotonic()
            os.fsync(stream.fileno())
            write_timings.append(dict(monotonic=flush_start, seconds=time.monotonic()-flush_start, phase='final_fsync'))
        phases.append(dict(name='B_bounded_buffered_disk_write', start=start, end=time.monotonic()))
        phase('writeback_cooldown_not_a_baseline', 10)
        phase('A2_without_test_writes', 30)
    except Exception as exc:
        error = repr(exc)
    for sub in subscribers:
        sub.unregister()
    result = dict(diagnostic_only=True, hardware_commands_sent=False, configuration_changed=False,
        existing_operational_logs_continued=True, phases=phases, samples=rows,
        sdk_reads=list(sdk.values()), feedback=streams, write_timings=write_timings, bytes_written=bytes_written, error=error,
        limits='30-second phases; 165MiB hard total write limit; at least 2GiB retained for this diagnostic. Full grasp recording reserve unchanged. B isolates buffered file writing, not compression or extra image subscriptions. Brief clean windows cannot rule out intermittent faults.')
    (out / 'observations.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    (out / 'SHA256SUMS').write_text(hashlib.sha256((out/'observations.json').read_bytes()).hexdigest()+'  observations.json\n')
    print('FINISHED '+str(out), flush=True)
    if error:
        raise RuntimeError(error)


if __name__ == '__main__':
    main()
