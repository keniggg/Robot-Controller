"""Frozen G47 strip-family arithmetic; no ROS, device or network access."""
import hashlib
import json
import os
from pathlib import Path
import sys
import time

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
V = E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
OUT = V/'gen48_visibility_enclosure_numeric_r1'
TRACE = V/'gen48_g47_exact_box_visibility_probe_r1/attempt_r5/assignment_trace_before_exact_comparison.json'
BOUNDARY = []


def guard(event, args):
    denied = event.startswith('socket.') or event in ('subprocess.Popen', 'os.system', 'os.fork', 'os.posix_spawn', 'os.exec')
    if event == 'open' and isinstance(args[0], (str, bytes, os.PathLike)):
        p = Path(os.fsdecode(args[0]))
        flags = args[2] if len(args) > 2 and isinstance(args[2], int) else 0
        if str(p).startswith('/dev/') and str(p) not in ('/dev/null', '/dev/urandom', '/dev/random'):
            denied = True
        if flags & (os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC|os.O_APPEND):
            denied = denied or OUT not in p.resolve().parents
    if denied:
        BOUNDARY.append(event)
        raise RuntimeError('offline boundary: '+event)


def main():
    assert E.parent.parent.is_mount() and not OUT.exists()
    OUT.mkdir()
    os.environ.update(PYTHONDONTWRITEBYTECODE='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
    sys.dont_write_bytecode = True
    sys.addaudithook(guard)
    import numpy as np
    sys.path.insert(0, str(W/'.runtime/gen48_visibility_candidate/src/alicia_flexible_grasp_supervisor/src'))
    from alicia_flexible_grasp.vision import cuboid_rgb_interval as c
    raw = TRACE.read_bytes()
    assert hashlib.sha256(raw).hexdigest() == '49b319760b13b9ccd98817b73cab3f30be64abdf3277a53858a7f77765240245'
    event = next(row for row in json.loads(raw)['events'] if row['event'] == 'call')
    local, parent = event['locals'], event['infer_locals']
    for key in ('A', 'lo', 'hi', 'T', 'axis'):
        local[key] = np.asarray(local[key])
    k = np.asarray(parent['K'])
    start = time.perf_counter()
    result = c._visibility_family(parent['groups'], local['A'], local['lo'], local['hi'],
        local['h'], local['d'], local['T'], k, local['hu'], local['axis'], local['yaw'])
    elapsed = time.perf_counter()-start
    value = dict(result=c._plain(result), elapsed_sec=elapsed,
        original_trace_sha256=hashlib.sha256(raw).hexdigest(),
        candidate_source_sha256=hashlib.sha256(Path(c.__file__).read_bytes()).hexdigest(),
        helper_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        boundary_violations=BOUNDARY, qualification_or_motion_authorized=False)
    target = OUT/'result.json'
    with target.open('x') as f:
        json.dump(value, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write('\n')
    print(json.dumps(dict(path=str(target), retained=result['retained_branch_count'],
        axis_mm=(result['axis_uncertainty']*1000.).tolist(), yaw_rad=result['yaw_uncertainty'],
        elapsed_sec=elapsed, branches=[dict(states=x['states'], retained=x['retained'],
        rejection=x.get('rejection')) for x in result['branches']]), ensure_ascii=False))


if __name__ == '__main__':
    main()
