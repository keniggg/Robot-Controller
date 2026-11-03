"""Read-only bounded /proc and log-offset observation. No ROS or UI calls."""
import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import sys
import time

OUT = Path(sys.argv[1])
if not OUT.is_dir():
    raise RuntimeError('External evidence directory is unavailable')
ROLES = {
    28041: ('codex_main_extension_host', 252324),
    28068: ('usage_monitor_extension_host', 252423),
    28073: ('ros_python_cpp_extension_host', 252424),
    28313: ('codex_app_server', 252529),
    21326: ('vscode_renderer', 183195),
    21679: ('vscode_renderer', 183453),
    29666: ('vscode_renderer', 254088),
}
LOGROOT = Path('/home/zhuyupei/.config/Code/logs/20261009T174503')
LOGS = [LOGROOT / f for f in (
    'main.log', 'window1/renderer.log', 'window1/exthost/exthost.log',
    'window1/exthost/openai.chatgpt/Codex.log',
)]
HZ = os.sysconf('SC_CLK_TCK')
PAGE = os.sysconf('SC_PAGE_SIZE')

def utc():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()

def process(pid):
    try:
        raw = Path('/proc/%d/stat' % pid).read_text()
        vals = raw[raw.rfind(')') + 2:].split()
        return dict(start_ticks=int(vals[19]), cpu_ticks=int(vals[11]) + int(vals[12]),
                    state=vals[0], rss_mib=int(vals[21]) * PAGE / 1048576)
    except (OSError, ValueError, IndexError) as exc:
        return dict(unavailable=type(exc).__name__)

def logsnapshot(path):
    try:
        s = path.stat()
        return dict(inode=s.st_ino, size=s.st_size, mtime_ns=s.st_mtime_ns)
    except OSError as exc:
        return dict(unavailable=type(exc).__name__)

def logfds(pid):
    found = set()
    try:
        for p in Path('/proc/%d/fd' % pid).iterdir():
            try:
                value = os.readlink(p)
                if value.startswith(str(LOGROOT)) and value.endswith('.log'):
                    found.add(value)
            except OSError:
                pass
    except OSError:
        pass
    return sorted(found)

started = utc()
before = {str(p): logsnapshot(p) for p in LOGS}
initial = {str(pid): process(pid) for pid in ROLES}
fds = {str(pid): logfds(pid) for pid in ROLES if 'extension_host' in ROLES[pid][0]}
previous = initial
previous_mono = time.monotonic()
samples = []
for _ in range(5):
    time.sleep(5)
    now_mono = time.monotonic()
    current = {str(pid): process(pid) for pid in ROLES}
    rows = {}
    for pid, (role, expected_start) in ROLES.items():
        key = str(pid)
        prior, cur = previous[key], current[key]
        same = (prior.get('start_ticks') == cur.get('start_ticks') == expected_start)
        rows[key] = dict(role=role, same_saved_process=same, **cur)
        rows[key]['cpu_percent_one_core'] = (
            100 * (cur['cpu_ticks'] - prior['cpu_ticks']) / HZ / (now_mono - previous_mono)
            if same else None)
    samples.append(dict(ended_utc=utc(), duration_seconds=now_mono-previous_mono, processes=rows))
    previous, previous_mono = current, now_mono

log_results = {}
for path in LOGS:
    key = str(path)
    a, b = before[key], logsnapshot(path)
    valid = 'size' in a and 'size' in b and a['inode'] == b['inode'] and b['size'] >= a['size']
    entry = dict(before=a, after=b, offset_comparable=valid)
    if valid:
        size = b['size'] - a['size']
        cap = 1048576
        with path.open('rb') as stream:
            stream.seek(a['size'])
            new = stream.read(min(size, cap))
        events = []
        for line in new.decode('utf-8', 'replace').splitlines():
            if re.search(r'\bunresponsive\b|\bresponsive\b', line, re.I):
                timestamp = re.match(r'\d{4}-\d\d-\d\d[ T]\d\d:\d\d:\d\d(?:\.\d+)?', line)
                events.append(dict(timestamp=timestamp.group() if timestamp else None,
                                   kind='unresponsive' if re.search(r'\bunresponsive\b', line, re.I) else 'responsive',
                                   line_sha256=hashlib.sha256(line.encode()).hexdigest()))
        entry.update(appended_bytes=size, scanned_bytes=len(new), scan_complete=size<=cap,
                     new_responsiveness_events=events)
    log_results[key] = entry

summary = {}
for pid, (role, expected_start) in ROLES.items():
    key = str(pid)
    cpus = [s['processes'][key]['cpu_percent_one_core'] for s in samples]
    valid = [x for x in cpus if x is not None]
    summary[key] = dict(role=role, expected_start_ticks=expected_start,
                        same_saved_process_all_samples=all(s['processes'][key]['same_saved_process'] for s in samples),
                        mean_cpu_percent=sum(valid)/len(valid) if valid else None,
                        max_cpu_percent=max(valid) if valid else None,
                        last_rss_mib=samples[-1]['processes'][key].get('rss_mib'))
report = dict(started_utc=started, completed_utc=utc(),
    task_context='Parent reports GEN6 has moved; phase not independently subscribed during this measurement.',
    method='Five 5-second /proc CPU deltas, percent of one logical CPU; exact saved PID start-time identity; log start/end offsets.',
    script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    prior_identity_evidence=str(OUT/'post_affinity_isolation_60s.json'),
    initial_processes=initial, open_log_fds=fds, samples=samples, cpu_summary=summary,
    log_observation=log_results,
    limitations=['25-second sampled interval only; no proof of permanent absence of freezes.',
                'Low CPU and absent responsiveness logs do not prove webview messages were rendered.',
                'No UI interaction, profiler, environment/credential reads, ROS calls, node restarts, or control commands.'])
name='generation6_editor_load_25s_'+datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'.json'
dest=OUT/name
payload=(json.dumps(report,ensure_ascii=False,indent=2)+'\n').encode()
with dest.open('xb') as stream:
    stream.write(payload)
print(json.dumps(dict(path=str(dest), sha256=hashlib.sha256(payload).hexdigest(), cpu_summary=summary,
                     logs=[dict(path=k, appended_bytes=v.get('appended_bytes'), events=v.get('new_responsiveness_events'), complete=v.get('scan_complete')) for k,v in log_results.items()]),ensure_ascii=False))
