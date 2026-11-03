"""Offline focused verification; logs only on the designated mounted disk."""
import sys,json,hashlib,contextlib
from pathlib import Path
W=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(W/'src/alicia_flexible_grasp_supervisor/tests'),str(W/'src/alicia_grasp_modes/tests'),str(W/'src/alicia_flexible_grasp_supervisor/src'),str(W/'src/alicia_grasp_modes/src')]
import pytest
OUT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/gen20_analysis/verification')
assert Path('/media/zhuyupei/SanStand').is_mount();OUT.mkdir(parents=True,exist_ok=True)
stem=sys.argv[1]
scope=sys.argv[2:] or ['src/alicia_flexible_grasp_supervisor/tests/test_held_pregrasp_selection.py']
sources=['src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/'+n for n in ['held_pregrasp_selection.py','contact_sequence_precheck.py','moveit_planner.py']]+[s.split('::')[0] for s in scope]+['src/alicia_flexible_grasp_supervisor/tests/fixtures/held_pregrasp_gen20.json']
def hashes():return {n:hashlib.sha256((W/n).read_bytes()).hexdigest() for n in sources}
before=hashes()
with (OUT/(stem+'.log')).open('w') as stream,contextlib.redirect_stdout(stream),contextlib.redirect_stderr(stream):
 result=pytest.main(['-q','-p','no:cacheprovider','--junitxml='+str(OUT/(stem+'.xml'))]+scope)
after=hashes()
(OUT/(stem+'_manifest.json')).write_text(json.dumps(dict(test_scope=scope,source_sha256=before,source_after_sha256=after,source_unchanged=before==after,exit_code=int(result)),indent=2)+'\n')
print(stem,'exit',result)
sys.exit(int(result))
