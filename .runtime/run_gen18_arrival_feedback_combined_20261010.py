"""Offline pytest with exact source hashes and external-only artifacts."""
import contextlib
import hashlib
import json
import os
from pathlib import Path
import sys
import pytest

WORK = Path(__file__).resolve().parents[1]
OUT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/verification/gen18_arrival_feedback_gap_combined')
PRODUCTION = [
    'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/feedback_bracket_gap.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_producer.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_remote.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_task.py',
]
FIXTURE = 'src/alicia_grasp_modes/tests/fixtures/gen18_arrival_feedback_gap.json'


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    assert Path.cwd() == WORK
    OUT.mkdir(parents=True, exist_ok=True)
    stem, *scope = sys.argv[1:]
    assert scope and '/' not in stem
    for package in ('alicia_grasp_modes', 'alicia_flexible_grasp_supervisor'):
        sys.path.insert(0, str(WORK/'src'/package/'tests'))
    files = sorted(set(PRODUCTION + [FIXTURE] + [item.split('::')[0] for item in scope]))
    def hashes():
        return {f: hashlib.sha256((WORK/f).read_bytes()).hexdigest() for f in files}
    before = hashes()
    with (OUT/(stem+'.log')).open('w') as log, contextlib.redirect_stdout(log), contextlib.redirect_stderr(log):
        code = pytest.main(['-q', '-p', 'no:cacheprovider',
            '--basetemp='+str(OUT/(stem+'_pytest_tmp')),
            '--junitxml='+str(OUT/(stem+'.xml'))] + scope)
    after = hashes()
    (OUT/(stem+'_manifest.json')).write_text(json.dumps({
        'test_scope': scope, 'source_sha256': before,
        'source_unchanged': before == after, 'source_sha256_after': after,
        'exit_code': code, 'offline_only': True,
        'environment': {'python': sys.executable, 'PYTHONNOUSERSITE': os.getenv('PYTHONNOUSERSITE')},
    }, indent=2)+'\n')
    print(json.dumps({'stem': stem, 'exit_code': code, 'source_unchanged': before == after}))
    return code


if __name__ == '__main__':
    sys.exit(main())
