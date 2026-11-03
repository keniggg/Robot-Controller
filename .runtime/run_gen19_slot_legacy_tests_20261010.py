"""Run only old regressions not already covered by the new worker-chain suite."""
import contextlib
import hashlib
import json
from pathlib import Path
import sys
import pytest

WORK=Path(__file__).resolve().parents[1]
OUT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/verification/gen19_direct_slot_retry')
PROD=['src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py',
      'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_remote.py']
STATION=['src/alicia_grasp_modes/tests/'+f for f in (
    'test_station_compute_work_handoff.py','test_station_compute_wait_remote.py','test_station_compute_wait_integration.py')]
STREAM='src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py'


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    assert Path.cwd()==WORK
    OUT.mkdir(parents=True,exist_ok=True)
    for package in ('alicia_grasp_modes','alicia_flexible_grasp_supervisor'):
        sys.path.insert(0,str(WORK/'src'/package/'tests'))
    files=PROD+STATION+[STREAM]
    hashes=lambda:{p:hashlib.sha256((WORK/p).read_bytes()).hexdigest() for p in files}
    before=hashes()
    selected=[]
    class Capture:
        def pytest_collection_finish(self,session):
            selected.extend(item.nodeid for item in session.items)
    with (OUT/'legacy_collect.log').open('w') as log,contextlib.redirect_stdout(log),contextlib.redirect_stderr(log):
        code=pytest.main(['-q','-p','no:cacheprovider','--collect-only',STREAM,
            '-k','direct or stream_worker or prediction_failure'],plugins=[Capture()])
    assert code==0 and selected
    # Pytest may choose a package-local root; preserve exact full source paths.
    selected=[STREAM+'::'+node.split('::',1)[1] for node in selected]
    scope=STATION+selected
    stem='legacy_r1'
    with (OUT/(stem+'.log')).open('w') as log,contextlib.redirect_stdout(log),contextlib.redirect_stderr(log):
        code=pytest.main(['-q','-p','no:cacheprovider','--basetemp='+str(OUT/(stem+'_pytest_tmp')),
            '--junitxml='+str(OUT/(stem+'.xml'))]+scope)
    after=hashes()
    (OUT/(stem+'_manifest.json')).write_text(json.dumps(dict(test_scope=scope,source_sha256=before,
        source_unchanged=before==after,exit_code=code,streaming_selection_expression='direct or stream_worker or prediction_failure',
        new_targeted_tests_repeated=False),indent=2)+'\n')
    print(json.dumps(dict(exit_code=code,source_unchanged=before==after,selected_streaming_nodeids=len(selected))))
    return code


if __name__=='__main__':
    sys.exit(main())
