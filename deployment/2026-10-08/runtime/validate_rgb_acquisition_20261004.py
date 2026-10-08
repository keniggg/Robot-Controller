"""Record final joint regression and immutable loaded-source hashes."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess

ROOT=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
OUT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_ros_resume_20261004T002930Z')
if not Path('/media/zhuyupei/SanStand').is_mount():
    raise RuntimeError('SanStand unavailable; no fallback')
old=json.loads((ROOT/'.runtime/rgb_observation_validation_20261004.json').read_text())
files=list(old['source_sha256'])+[
 '.runtime/load_rgb_acquisition_20261004.py',
 '.runtime/run_confirmed_unknown_grasp_20261004.py',
 '.runtime/recorded_reload_then_grasp_20261004.py',
 'tools/diagnostics/confirmed_unknown_dispatch.py',
 'src/alicia_grasp_modes/scripts/mode_aware_grasp_runner.py',
 '.runtime/collect_unknown_first_loss.py',
 '.runtime/bounded_async_bag_writer.py',
 'src/alicia_flexible_grasp_supervisor/tests/fixtures/rgb_frozen_reference_visibility_20261004.json',
 'src/alicia_grasp_modes/src/alicia_grasp_modes/measured_mask_seeds.py',
 'src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_observation_projection.py',
 'tools/diagnostics/prepare_rgb_acquisition_retry.py',
 '.runtime/prepare_rgb_retry_20261004.py',
 'src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_noncontact_scene.py',
 'src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_observation_handoff.py',
 'src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_observation_set.py',
 'src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_observation_set_handoff.py',
 'src/alicia_flexible_grasp_supervisor/scripts/motion_gateway_node.py',
 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/rgb_acquisition_gateway.py',
 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/rgb_acquisition_artifacts.py',
 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observation_tracking_contract.py',
 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observation_preview.py',
 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/contracted_trajectory.py',
 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/actuation_bootstrap.py',
 'tools/diagnostics/run_rgb_acquisition.py']
tests=list(old['command'][5:])+[
 '.runtime/test_recorded_reload_then_grasp.py',
 'tools/tests/test_confirmed_unknown_dispatch.py',
 'tools/tests/test_confirmed_unknown_dispatch_wrapper.py',
 'src/alicia_grasp_modes/tests/test_runner_expected_selection.py',
 '.runtime/test_fragment_hold_consumer_20261004.py',
 '.runtime/test_collect_unknown_first_loss.py',
 'src/alicia_grasp_modes/tests/test_locked_rgb_hold.py',
 'src/alicia_grasp_modes/tests/test_measured_mask_seeds.py',
 'src/alicia_grasp_modes/tests/test_rgb_observation_projection.py',
 'tools/tests/test_prepare_rgb_acquisition_retry.py',
 'src/alicia_grasp_modes/tests/test_rgb_observation_handoff.py',
 'src/alicia_grasp_modes/tests/test_rgb_handoff_stream.py',
 'src/alicia_grasp_modes/tests/test_rgb_observation_set.py',
 'src/alicia_grasp_modes/tests/test_rgb_observation_set_handoff.py',
 'src/alicia_grasp_modes/tests/test_rgb_set_stream.py',
 'src/alicia_grasp_modes/tests/test_rgb_feedback_wait.py',
 'src/alicia_grasp_modes/tests/test_rgb_planning_history.py',
 'src/alicia_grasp_modes/tests/test_rgb_noncontact_scene.py',
 'src/alicia_flexible_grasp_supervisor/tests/test_rgb_acquisition_gateway.py',
 'src/alicia_flexible_grasp_supervisor/tests/test_contracted_trajectory.py',
 'src/alicia_flexible_grasp_supervisor/tests/test_actuation_bootstrap.py',
 'tools/tests/test_run_rgb_acquisition.py']
for pattern in ('test_observation_*.py','test_gateway_*.py'):
    tests.extend(str(path.relative_to(ROOT)) for path in
        sorted((ROOT/'src/alicia_flexible_grasp_supervisor/tests').glob(pattern)))
tests=list(dict.fromkeys(tests))
for name in files+tests:
    if not (ROOT/name).is_file():
        raise RuntimeError('missing final implementation/test: '+name)
before={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in files+tests}
stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
log=OUT/('rgb_acquisition_regression_'+stamp+'.txt')
command=['/usr/bin/python3','-m','pytest','-q','-ra']+tests
with log.open('w') as stream:
    result=subprocess.run(command,cwd=str(ROOT),stdout=stream,stderr=subprocess.STDOUT)
after={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in files+tests}
changed=[name for name in before if before[name]!=after[name]]
summary=log.read_text().strip().splitlines()[-1]
manifest=dict(recorded_at_utc=datetime.now(timezone.utc).isoformat(),command=command,
    test_exit_code=result.returncode if not changed else 99,test_result=summary,
    source_sha256={name:after[name] for name in files},
    test_sha256={name:after[name] for name in tests},changed_during_test=changed,test_log=str(log))
(OUT/('rgb_acquisition_validation_'+stamp+'.json')).write_text(json.dumps(manifest,indent=2)+'\n')
(ROOT/'.runtime/rgb_acquisition_validation_20261004.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps(dict(test_exit_code=manifest['test_exit_code'],test_result=summary,
    changed_during_test=changed,test_log=str(log)),indent=2),flush=True)
raise SystemExit(manifest['test_exit_code'])
