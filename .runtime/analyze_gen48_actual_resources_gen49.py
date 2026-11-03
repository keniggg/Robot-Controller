"""Segment completed /proc samples by exact recorded task events; no ROS."""
import hashlib
import json
from pathlib import Path

E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N = E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
V = N/'verification'
S = V/'gen48_actual_task_resources_r1'
O = V/'gen49_g48_actual_resources_r2'

def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    assert E.parent.parent.is_mount()
    meta = json.loads((S/'metadata.json').read_text())
    result = json.loads((S/'result.json').read_text())
    assert result['passed'] and result['samples_sha256'] == sha(S/'samples.jsonl')
    rows = [json.loads(s) for s in (S/'samples.jsonl').read_text().splitlines()]
    assert len(rows) == result['sample_count']
    log = N/'gen48_runtime/compute_gateway_deployment_r1/consumers.log'
    lines = log.read_text(errors='strict').splitlines()
    events = [
        ('task_start', '1792317049.079111', '[Grasp] MOVE_PREGRASP capturing initial observation pose'),
        ('observation_motion', '1792317049.986922', '[Grasp] MOVE_PREGRASP moving 6D pregrasp'),
        ('qualified_station', '1792317081.330730', '[Grasp] PLAN_PREGRASP qualified observation station'),
        ('contact_wait', '1792317082.394007', '[Grasp] PLAN_PREGRASP waiting for near-field 6D preview'),
        ('request22_completed', '1792317107.436249', 'remote 6D pipeline metrics:'),
        ('task_failed', '1792317171.410957', '[Grasp] FAILED TARGET_LOST: station_compute_wait_expired_or_future'),
    ]
    evidence = []
    for name, stamp, needle in events:
        matches = [line for line in lines if '['+stamp+']' in line and needle in line]
        assert len(matches) == 1, (name, len(matches))
        sec, frac = stamp.split('.')
        evidence.append(dict(name=name, stamp_ns=int(sec)*10**9+int(frac.ljust(9,'0')),
                             exact_log_line=matches[0]))
    times = {e['name']: e['stamp_ns'] for e in evidence}
    windows = [
        ('task_initial_planning', 'task_start', 'observation_motion'),
        ('observation_motion_and_settle', 'observation_motion', 'qualified_station'),
        ('station_to_contact_request_completion', 'qualified_station', 'request22_completed'),
        ('after_request_completion_to_task_failure', 'request22_completed', 'task_failed'),
    ]
    summaries = []
    for name, start, end in windows:
        selected = [r for r in rows if times[start] <= r['recorded_ns'] <= times[end]]
        processes, system = {}, dict(total_ticks=0, busy_ticks=0, iowait_ticks=0, covered_sec=0.)
        for a, b in zip(selected, selected[1:]):
            dt = (b['monotonic_ns']-a['monotonic_ns'])/1e9
            assert dt > 0
            delta = [y-x for x,y in zip(a['cpu_ticks'][:8],b['cpu_ticks'][:8])]
            assert min(delta) >= 0
            total = sum(delta)
            system['total_ticks'] += total
            system['busy_ticks'] += total-delta[3]-delta[4]
            system['iowait_ticks'] += delta[4]
            system['covered_sec'] += dt
            prior = {(p['pid'],p['start_ticks']):p for p in a['processes']}
            for p in b['processes']:
                key = (p['pid'],p['start_ticks']); q = prior.get(key)
                if q is None: continue
                acc = processes.setdefault(key, dict(name=p['name'],pid=p['pid'],start_ticks=p['start_ticks'],
                    cpu_sec=0.,covered_sec=0.,read_bytes=0,write_bytes=0,peak_observed_rss_bytes=0,intervals=0))
                cpu = p['cpu_ticks']-q['cpu_ticks']; assert cpu >= 0
                acc['cpu_sec'] += cpu/meta['clock_ticks_per_second']
                acc['covered_sec'] += dt; acc['intervals'] += 1
                acc['peak_observed_rss_bytes'] = max(acc['peak_observed_rss_bytes'],p['rss_bytes'],q['rss_bytes'])
                for metric in ('read_bytes','write_bytes'):
                    acc[metric] += max(0,p.get('io',{}).get(metric,0)-q.get('io',{}).get(metric,0))
        for acc in processes.values():
            acc['cpu_percent_one_core'] = 100*acc['cpu_sec']/acc['covered_sec']
        for out, counter in [('busy_percent_all_cpus','busy_ticks'),('iowait_percent_all_cpus','iowait_ticks')]:
            system[out] = (100*system[counter]/system['total_ticks']
                           if system['total_ticks'] else None)
        summaries.append(dict(name=name,begin_event=start,end_event=end,
            event_span_sec=(times[end]-times[start])/1e9, sample_count=len(selected),
            interval_measurement_available=len(selected)>1,
            first_sample_ns=selected[0]['recorded_ns'] if selected else None,
            last_sample_ns=selected[-1]['recorded_ns'] if selected else None,
            system=system,processes=sorted(processes.values(),key=lambda p:-p['cpu_sec'])))
    value=dict(kind='source_event_bound_actual_G48_resource_analysis',passed=True,
        source_samples=dict(path=str(S/'samples.jsonl'),sha256=sha(S/'samples.jsonl')),
        metadata=dict(path=str(S/'metadata.json'),sha256=sha(S/'metadata.json')),
        helper_sha256=sha(Path(__file__)),source_event_log_path=str(log),events=evidence,windows=summaries,
        idle_prefix_excluded=True,source_cpu_count=meta['os_cpu_count'],
        supersedes=str(V/'gen49_g48_actual_resources_r1/result.json'),
        supersedes_reason='A single-sample window has no interval measurement; report null instead of misleading zero CPU/iowait.',
        limits=['Each window uses only complete adjacent sample intervals inside its event bounds.',
                'Process deltas require the same PID and start time in both adjacent samples.',
                'CPU uses 100 percent per core; system busy/iowait use all CPU ticks.',
                'Proc counters do not measure ROS queues, callback lag, device latency, or prove causality.',
                'Request completion is the metrics log event, not the earlier terminal publication.',
                'Later manual restoration occurred after sampler end and is not covered here.'],
        motion_sent=False,temperature_read=False)
    O.mkdir(exist_ok=False)
    raw=(json.dumps(value,ensure_ascii=False,indent=2)+'\n').encode()
    (O/'result.json').write_bytes(raw); assert (O/'result.json').read_bytes()==raw
    print(json.dumps(dict(result=str(O/'result.json'),sha256=sha(O/'result.json'),windows=[dict(name=w['name'],samples=w['sample_count'],covered_sec=w['system']['covered_sec'],busy_percent=w['system']['busy_percent_all_cpus'],iowait_percent=w['system']['iowait_percent_all_cpus'])for w in summaries]),ensure_ascii=False))

if __name__ == '__main__': main()
