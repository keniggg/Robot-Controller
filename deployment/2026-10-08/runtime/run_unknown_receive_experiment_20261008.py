#!/usr/bin/env python3
"""Bounded isolated receive trace with an independent raw-recording load.

No ROS service, publication, SDK or robot control in this orchestrator. Only
its own diagnostic children receive SIGINT. Production selection is untouched.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

W = Path(__file__).resolve().parents[1]
MOUNT = Path('/media/zhuyupei/SanStand')
EXT = MOUNT/'实验记录/Robot-Controller/ros_log'


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def identity(pid):
    p = Path('/proc')/str(pid)
    stat = p.joinpath('stat').read_text().rsplit(')', 1)[1].split()
    return dict(pid=pid, start_ticks=int(stat[19]), state=stat[0],
                argv=p.joinpath('cmdline').read_bytes().decode().split('\0')[:-1])


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--production-unknown', required=True, help='PID:start_ticks')
    ap.add_argument('--deserializer', choices=('original', 'skip_pruned_copies'), default='original')
    ap.add_argument('--candidate-sha256', help='Required frozen helper digest for private candidate only')
    a = ap.parse_args()
    out = a.output.resolve()
    if not MOUNT.is_mount() or EXT not in out.parents or a.output.is_symlink():
        raise RuntimeError('Mounted canonical external output required; no fallback')
    if shutil.disk_usage(MOUNT).free < 6*1024**3:
        raise RuntimeError('6 GiB free required for bounded diagnostics')
    pid, ticks = map(int, a.production_unknown.split(':'))
    old = identity(pid)
    if old['start_ticks'] != ticks or old['state'] == 'Z':
        raise RuntimeError('Production unknown identity differs')
    out.mkdir(exist_ok=False)
    names = ['run_unknown_receive_shadow_20261008.py', 'unknown_transport_trace_20261008.py',
             'observe_sdk_memory_pressure_20261007.py', 'compare_raw_rgbd_recording_load_20261007.py']
    if a.deserializer == 'skip_pruned_copies':
        candidate = W/'.runtime/unknown_pruned_copy_candidate_20261008.py'
        if not a.candidate_sha256 or sha(candidate) != a.candidate_sha256:
            raise RuntimeError('Private deserializer candidate digest differs')
        names.append(candidate.name)
    elif a.candidate_sha256 is not None:
        raise RuntimeError('Candidate digest supplied without private candidate variant')
    source_sha = {str(W/'.runtime'/n): sha(W/'.runtime'/n) for n in names}
    source_sha[str(Path(__file__).resolve())] = sha(__file__)
    (out/'sources').mkdir()
    for path, digest in source_sha.items():
        target = out/'sources'/Path(path).name
        shutil.copyfile(path, target)
        assert sha(target) == digest
    report = dict(schema=1, started_monotonic=time.monotonic(),
        started_wall_ns=time.time_ns(), motion_authorized=False,
        hardware_commands_sent=False, temperature_queried=False,
        deserializer_variant=a.deserializer, candidate_sha256=a.candidate_sha256,
        production_unknown=old, source_sha256=source_sha, events=[], children={},
        limits=['Shadow adds a separate image subscriber and algorithm process; it is not the original process.',
                'B adds reception, serialization, LZ4, memory and external I/O; it is not a disk-only intervention.',
                'Use actual ready/close boundaries; timings are not assumed to be exact 20/45/25 seconds.'])
    children = {}; logs = []; stopped = []
    hard_deadline = time.monotonic()+140
    env = os.environ.copy()
    env.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
               PYTHONDONTWRITEBYTECODE='1', PYTHONUNBUFFERED='1')

    def save(event):
        report['events'].append(dict(event=event, monotonic=time.monotonic(), wall_ns=time.time_ns()))
        tmp=out/'experiment.tmp'
        tmp.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        tmp.replace(out/'experiment.json')

    def launch(name, command):
        for p, digest in source_sha.items():
            if sha(p) != digest:
                raise RuntimeError('Frozen diagnostic source changed: '+p)
        log=(out/(name+'.log')).open('xb');logs.append(log)
        child=subprocess.Popen(command, cwd=str(W), env=env, stdout=log, stderr=subprocess.STDOUT)
        children[name]=child
        report['children'][name]=dict(command=command, identity=identity(child.pid))
        save(name+'_started')
        return child

    def await_ready(name, path, seconds):
        deadline=min(hard_deadline,time.monotonic()+seconds)
        while time.monotonic()<deadline:
            if path.is_file():
                value=json.loads(path.read_text());report['children'][name]['ready']=value
                save(name+'_ready');return value
            if children[name].poll() is not None:
                raise RuntimeError(name+' exited before ready')
            time.sleep(.05)
        raise RuntimeError(name+' readiness timeout')

    def interrupted(sig, _frame):
        stopped.append(sig)
        raise InterruptedError('Diagnostic interrupted: '+str(sig))

    for sig in (signal.SIGINT, signal.SIGTERM): signal.signal(sig, interrupted)
    failure=None
    try:
        shadow_command = ['/usr/bin/python3','-u',str(W/'.runtime'/names[0]),
            '--output',str(out/'shadow'),'--seconds','90',
            '--trace-sha256',source_sha[str(W/'.runtime'/names[1])],
            '--observe-production-status','--deserializer',a.deserializer]
        if a.deserializer == 'skip_pruned_copies':
            shadow_command += ['--candidate-sha256',a.candidate_sha256]
        launch('shadow', shadow_command)
        ready=await_ready('shadow', out/'shadow/ready.json', 15)
        ready_seen=time.monotonic()
        shadow_identity = identity(children['shadow'].pid)
        if (ready.get('pid') != shadow_identity['pid']
                or ready.get('start_ticks') != shadow_identity['start_ticks']):
            raise RuntimeError('Shadow ready process identity differs')
        launch('resources', ['/usr/bin/python3','-u',str(W/'.runtime'/names[2]),
            '--run','--output',str(out/'resources'),'--seconds','80',
            '--watch-pid','%d:%d' % (shadow_identity['pid'],shadow_identity['start_ticks'])])
        await_ready('resources',out/'resources/ready.json',8)
        next_heartbeat=time.monotonic()
        while time.monotonic()<hard_deadline:
            now=time.monotonic()
            if 'raw_load' not in children and now>=ready_seen+20:
                if children['shadow'].poll() is not None:
                    raise RuntimeError('Shadow ended before recording contrast')
                (out/'raw_load').mkdir()
                launch('raw_load', ['/usr/bin/python3','-u',str(W/'.runtime'/names[3]),
                    '--record-child','--output',str(out/'raw_load')])
                await_ready('raw_load',out/'raw_load/ready.json',8)
            for name, child in children.items():
                rc=child.poll()
                if rc is not None and 'returncode' not in report['children'][name]:
                    report['children'][name].update(returncode=rc, exit_observed_monotonic=now)
                    save(name+'_exited')
                    if rc and name == 'raw_load':
                        # A closed but incomplete bag is still invalid. Keep
                        # the independent observers running to their original
                        # deadlines so the post-recording phase is preserved.
                        report['raw_recording_nonzero'] = rc
                    elif rc:
                        raise RuntimeError(name+' returned '+str(rc))
            if len(children)==3 and all(c.poll() is not None for c in children.values()):
                break
            if now>=next_heartbeat:
                print(json.dumps(dict(elapsed_sec=now-report['started_monotonic'],
                    children={k:v.poll() for k,v in children.items()})),flush=True)
                next_heartbeat=now+15
            time.sleep(.1)
        if any(c.poll() is None for c in children.values()):
            raise RuntimeError('Bounded diagnostic deadline exceeded')
        if report.get('raw_recording_nonzero'):
            failure = 'Raw recording invalid; full independent observation retained'
    except BaseException as exc:
        failure=repr(exc)
    finally:
        for sig in (signal.SIGINT, signal.SIGTERM): signal.signal(sig, signal.SIG_IGN)
        for name, child in children.items():
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                report['children'][name]['owned_diagnostic_sigint_sent']=True
        close_deadline=time.monotonic()+15
        for name,child in children.items():
            try:child.wait(timeout=max(.01,close_deadline-time.monotonic()))
            except subprocess.TimeoutExpired:pass
            report['children'][name]['returncode']=child.poll()
        for log in logs:log.close()
        report.update(failure=failure, signals=stopped, finished_monotonic=time.monotonic(),
            diagnostic_children_remaining=[n for n,c in children.items() if c.poll() is None])
        try:
            after=identity(pid);report['production_unknown_after']=after
            report['production_unknown_identity_preserved']=after['start_ticks']==ticks and after['state']!='Z'
        except OSError as exc:
            report['production_unknown_after_error']=repr(exc)
        report['orchestration_completed']=failure is None and not report['diagnostic_children_remaining']
        save('finished')
        (out/'experiment.sha256').write_text(sha(out/'experiment.json')+'  experiment.json\n')
        print(json.dumps(dict(output=str(out),completed=report['orchestration_completed'],
                             failure=failure,remaining=report['diagnostic_children_remaining'])),flush=True)
    return 0 if report['orchestration_completed'] else 2


if __name__=='__main__':raise SystemExit(main())
