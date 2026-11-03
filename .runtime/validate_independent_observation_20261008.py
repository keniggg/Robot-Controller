"""Freeze and run the combined offline observation/contact/support regressions."""
import hashlib
import json
import argparse
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

WORK = Path(__file__).resolve().parents[1]
SESSION = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/supplement_grasp_20261008T103117Z')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-name', default='independent_observation_combined')
    args = parser.parse_args()
    assert args.output_name.replace('_', '').isalnum()
    assert Path('/media/zhuyupei/SanStand').is_mount()
    out = SESSION/'verification'/args.output_name
    out.mkdir(exist_ok=False)
    producer = json.loads((SESSION/'verification/independent_observation/source_manifest.json').read_text())
    consumer = json.loads((SESSION/'verification/measured_observation_consumers/source_sha256.json').read_text())
    support = json.loads((SESSION/'verification/support_visibility_recovery/result.json').read_text())
    assert support['passed'] is True
    sources = {item['path']: item['sha256'] for item in producer['files']}
    sources.update(consumer['source_sha256'])
    sources.update(support['source_sha256'])
    for name in ('src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py',
                 'src/alicia_grasp_modes/src/alicia_grasp_modes/reachable_observation_views.py'):
        sources[name] = hashlib.sha256((WORK/name).read_bytes()).hexdigest()
    def check():
        for name, digest in sources.items():
            assert hashlib.sha256((WORK/name).read_bytes()).hexdigest() == digest, name
    check()
    modes = WORK/'src/alicia_grasp_modes/tests'
    base = WORK/'src/alicia_flexible_grasp_supervisor/tests'
    patterns = ('test_*observe_return*.py', 'test_*observation*.py', 'test_support*.py',
                'test_tabletop*.py', 'test_locked_support*.py', 'test_runner*.py',
                'test_mode_runner.py', 'test_task_modes.py', 'test_unknown_frame_timing.py')
    tests = set(path for pattern in patterns for path in modes.glob(pattern))
    for pattern in ('test_*observation*.py', 'test_*observe_return*.py',
                    'test_rich_plan_candidate_source.py', 'test_grasp_task_sequence.py',
                    'test_task_contact_admission.py', 'test_contact_execution.py'):
        tests.update(base.glob(pattern))
    for path in tests:
        sources[str(path.relative_to(WORK))] = hashlib.sha256(path.read_bytes()).hexdigest()
    command = ['/usr/bin/python3', '-m', 'pytest', '-q', '-p', 'no:cacheprovider',
               '--junitxml='+str(out/'result.xml')]+[str(p.relative_to(WORK)) for p in sorted(tests)]
    env = dict(os.environ)
    env['PYTHONPATH'] = os.pathsep.join((str(modes), str(base), env.get('PYTHONPATH', '')))
    with (out/'result.log').open('w') as log:
        run = subprocess.run(command, cwd=str(WORK), env=env, stdout=log, stderr=subprocess.STDOUT)
    check()
    stats = [dict(s.attrib) for s in ET.parse(str(out/'result.xml')).getroot().iter('testsuite')]
    result = dict(passed=run.returncode == 0, returncode=run.returncode,
                  source_sha256=sources, command=command, suites=stats,
                  hardware_executed=False, deployment_performed=False)
    (out/'validated_sources.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print((out/'result.log').read_text()[-12000:])
    raise SystemExit(run.returncode)


if __name__ == '__main__':
    main()
