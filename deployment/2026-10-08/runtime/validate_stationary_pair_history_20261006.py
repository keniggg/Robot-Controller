"""Freeze and validate the camera history fix and its existing consumers."""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
assert Path('/media/zhuyupei/SanStand').is_mount()
OUT = E/'ros_log'/('stationary_pair_history_validation_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
OUT.mkdir()
vision = W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision'
production = sorted(str(p.relative_to(W)) for p in vision.glob('*.py'))
production += ['src/alicia_flexible_grasp_supervisor/scripts/camera_node.py',
    'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py',
    '.runtime/load_regional_production_20261006.py',
    '.runtime/gen4_root_cause/reload_camera_visual_gate.py',
    '.runtime/gen4_root_cause/camera_existing_read_observer.py']
production += ['.runtime/record_support_gap_15s.py',
    '.runtime/observe_support_metadata_20261006.py',
    '.runtime/gen4_root_cause/visual_snapshot_tap.py',
    '.runtime/gen4_root_cause/node_timing_tap.py',
    '.runtime/perception_diagnostic_chain_20261006.py',
    '.runtime/run_gen21_support_diagnostic_20261006.py',
    '.runtime/replay_raw_visual_snapshots_20261006.py']
names = ('test_camera', 'test_realsense', 'test_stationary', 'test_visual_stationary',
    'test_visual_snapshot',
    'test_regional_stationary', 'test_regional_nested', 'test_rgbd_rectification', 'test_pair_rectification',
    'test_mode_depth_preset', 'test_color_projection', 'test_depth', 'test_pair_history')
tests = sorted(str(p.relative_to(W)) for p in
    (W/'src/alicia_flexible_grasp_supervisor/tests').glob('test_*.py') if p.name.startswith(names))
tests += ['.runtime/test_load_regional_production_20261006.py',
    '.runtime/test_camera_preset_ab_observer.py',
    '.runtime/test_record_support_gap_15s.py',
    '.runtime/test_observe_support_metadata_20261006.py',
    '.runtime/test_camera_visual_gate_loader.py',
    '.runtime/test_visual_snapshot_tap.py',
    '.runtime/test_node_timing_tap.py',
    '.runtime/test_gen21_support_diagnostic_20261006.py',
    '.runtime/test_replay_raw_visual_snapshots_20261006.py',
    'src/alicia_grasp_modes/tests/test_unknown_tf_alignment.py']
assert all((W/f).is_file() for f in production+tests)
hashes = {f:hashlib.sha256((W/f).read_bytes()).hexdigest() for f in production+tests}
env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
    PYTHONDONTWRITEBYTECODE='1', PYTHONPATH=os.pathsep.join([
    str(W/'src/alicia_flexible_grasp_supervisor/tests'),
    str(W/'src/alicia_grasp_modes/tests'), os.environ.get('PYTHONPATH','')]))
with (OUT/'pytest.log').open('w') as log:
    result = subprocess.run([sys.executable, '-m', 'pytest', '-q', '-p', 'no:cacheprovider',
        '--junitxml='+str(OUT/'pytest.xml'), *tests], cwd=W, env=env,
        stdout=log, stderr=subprocess.STDOUT)
unchanged = all(hashlib.sha256((W/f).read_bytes()).hexdigest()==h for f,h in hashes.items())
report = dict(tests_passed=result.returncode==0 and unchanged, reviewed=False,
    files=hashes, production_files=production, tests=tests, returncode=result.returncode,
    source_unchanged=unchanged, diagnostic_only=True, robot_commands_sent=False,
    offline_sdk_copy_tests_enabled=env.get('ALICIA_OFFLINE_SDK_COPY_TEST') == '1')
(OUT/'validation.json').write_text(json.dumps(report, indent=2)+'\n')
(W/'.runtime/current_stationary_pair_history_validation_20261006.json').write_text(
    json.dumps(dict(manifest=str(OUT/'validation.json')), indent=2)+'\n')
print(json.dumps(dict(directory=str(OUT), returncode=result.returncode, unchanged=unchanged)), flush=True)
print((OUT/'pytest.log').read_text()[-2400:], flush=True)
sys.exit(result.returncode or (0 if unchanged else 2))
