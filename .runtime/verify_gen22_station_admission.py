"""Offline callback tests, logs on user-designated SanStand only."""
import sys,json,hashlib,contextlib
from pathlib import Path
W=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(W/'src/alicia_flexible_grasp_supervisor/tests'),str(W/'src/alicia_grasp_modes/tests'),str(W/'src/alicia_flexible_grasp_supervisor/src'),str(W/'src/alicia_grasp_modes/src')]
import pytest
OUT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/gen22_analysis/consumer_geometry')
assert Path('/media/zhuyupei/SanStand').is_mount();OUT.mkdir(parents=True,exist_ok=True)
stem=sys.argv[1];scope=sys.argv[2:] or ['src/alicia_grasp_modes/tests/test_station_ready_admission_order.py']
sources=['src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_remote.py']+[s.split('::')[0] for s in scope]
def hashes():return {p:hashlib.sha256((W/p).read_bytes()).hexdigest() for p in sources}
before=hashes()
with (OUT/(stem+'.log')).open('w') as stream,contextlib.redirect_stdout(stream),contextlib.redirect_stderr(stream):
    code=pytest.main(['-q','-p','no:cacheprovider','--junitxml='+str(OUT/(stem+'.xml'))]+scope)
after=hashes()
(OUT/(stem+'_manifest.json')).write_text(json.dumps(dict(test_scope=scope,source_sha256=before,source_after_sha256=after,source_unchanged=before==after,exit_code=int(code)),indent=2)+'\n')
print(stem,'exit',code);sys.exit(int(code))
