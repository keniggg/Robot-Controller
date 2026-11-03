"""Record one frozen offline regression directly on the requested external disk."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess

WORK = Path(__file__).resolve().parents[1]
MOUNT = Path('/media/zhuyupei/SanStand')
RUN = MOUNT/'实验记录/Robot-Controller/ros_log/full_ros_resume_20261005T012259Z'


def hashes(names):
    result={}
    for name in names:
        p=Path(name)
        assert not p.is_absolute() and '..' not in p.parts
        result[name]=hashlib.sha256((WORK/p).read_bytes()).hexdigest()
    return result


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--source',action='append',default=[])
    parser.add_argument('--test',action='append',default=[])
    args=parser.parse_args()
    assert MOUNT.is_mount() and RUN.is_dir(), 'SanStand unavailable; no fallback'
    previous=WORK/'.runtime/rgbd_rectification_validation_20261004.json'
    old=json.loads(previous.read_text())
    sources=sorted(set(old['source_sha256'])|set(args.source))
    tests=sorted(set(old['test_sha256'])|set(args.test))
    source_before,test_before=hashes(sources),hashes(tests)
    tag=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    log=RUN/('metric_ambiguity_regression_'+tag+'.log')
    out=RUN/('metric_ambiguity_validation_'+tag+'.json')
    command=['/usr/bin/python3','-m','pytest','-q','-ra']+tests
    with log.open('w') as stream:
        result=subprocess.run(command,cwd=str(WORK),stdout=stream,stderr=subprocess.STDOUT)
    after={**hashes(sources),**hashes(tests)}
    changed=[n for n,v in {**source_before,**test_before}.items() if after[n]!=v]
    text=log.read_text(errors='replace')
    report=dict(recorded_at_utc=datetime.now(timezone.utc).isoformat(),command=command,
        test_exit_code=result.returncode,test_result=text.strip().splitlines()[-1],
        test_log=str(log),test_log_sha256=hashlib.sha256(log.read_bytes()).hexdigest(),
        source_sha256=source_before,test_sha256=test_before,
        changed_during_test=changed,source_unchanged_during_test=not changed,
        previous_manifest=str(previous),previous_manifest_sha256=hashlib.sha256(previous.read_bytes()).hexdigest(),
        ros_or_robot_commands_sent=False,production_deployed=False)
    out.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(report=str(out),result=report['test_result'],changed=changed)),flush=True)
    if result.returncode or changed:raise SystemExit(1)
    (WORK/'.runtime/metric_ambiguity_validation_20261005.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
