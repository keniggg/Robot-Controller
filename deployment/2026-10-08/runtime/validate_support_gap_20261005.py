"""Run the existing regression set plus support-gap tests; record to SanStand."""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
R = M/'实验记录/Robot-Controller/ros_log/full_ros_resume_20261005T012259Z'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    assert M.is_mount(), 'SanStand unavailable; no fallback'
    prior = json.loads((R/'depth_followup_validation_20261005T022427.129512Z.json').read_text())
    tests = prior['validation_results'][0]['command'][5:]
    tests += ['src/alicia_grasp_modes/tests/test_support_measurement_hold.py',
        'src/alicia_grasp_modes/tests/test_support_pending_consumers.py',
        'src/alicia_flexible_grasp_supervisor/tests/test_precontact_observation_source.py',
        'src/alicia_flexible_grasp_supervisor/tests/test_mode_depth_preset.py',
        'src/alicia_flexible_grasp_supervisor/tests/test_stationary_depth.py']
    tests = sorted(set(tests))
    paths = set(tests) | {'.runtime/load_support_gap_20261005.py',
        '.runtime/validate_support_gap_20261005.py', '.runtime/load_pair_rectification_20261004.py',
        '.runtime/gen4_root_cause/reload_camera_visual_gate.py',
        'tools/diagnostics/prepare_rgb_acquisition_retry.py',
        'devel/lib/alicia_grasp_modes/unknown_tabletop_perception.py'}
    for root in ('src/alicia_grasp_modes', 'src/alicia_flexible_grasp_supervisor/src'):
        paths.update(str(p.relative_to(W)) for p in (W/root).rglob('*.py') if '__pycache__' not in p.parts)
    frozen = {p: sha(W/p) for p in sorted(paths)}
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    log = R/('support_gap_validation_'+stamp+'.log')
    receipt = R/('support_gap_validation_'+stamp+'.json')
    command = ['/usr/bin/python3','-m','pytest','-q','-ra','-p','no:cacheprovider']+tests
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
    began = time.monotonic()
    with log.open('w') as stream:
        result = subprocess.run(command, cwd=W, env=env, stdout=stream, stderr=subprocess.STDOUT)
    unchanged = all(sha(W/p) == value for p,value in frozen.items())
    report = dict(tests_passed=result.returncode == 0 and unchanged, reviewed=False,
        command=command, exit_code=result.returncode, elapsed_sec=time.monotonic()-began,
        files=frozen, unchanged_during_tests=unchanged, log=str(log), log_sha256=sha(log),
        receipt=str(receipt), summary=log.read_text()[-2500:])
    receipt.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    if report['tests_passed']:
        (W/'.runtime/support_gap_validation_20261005.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('files','command')},ensure_ascii=False),flush=True)
    if not report['tests_passed']: raise SystemExit(1)


if __name__ == '__main__':
    main()
