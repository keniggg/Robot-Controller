"""Run exactly one frozen affected-scope suite and the final real entry replay."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

WORK = Path(__file__).resolve().parents[1]
OUT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/gen22_analysis/verification')
assert Path('/media/zhuyupei/SanStand').is_mount()
OUT.mkdir(exist_ok=True)
STEM = sys.argv[1]
deliveries = [Path(value) for value in sys.argv[2:]]
assert len(deliveries) >= 2
tests = [
    'test_arrival_dense_identity_recorded.py', 'test_arrival_orb_proposal_recorded.py',
    'test_arrival_dense_sampling.py', 'test_observation_appearance_integration.py',
    'test_observation_measurement_unknown.py', 'test_observation_arrival_station.py',
    'test_observation_arrival_frame_completion.py', 'test_observation_arrival_frame_completion_guards.py',
    'test_arrival_fragment_group.py', 'test_arrival_same_instance_confirmation.py',
    'test_arrival_matching_fragments.py', 'test_arrival_matching_fragment_tracking.py',
    'test_arrival_fragment_recovery_guards.py', 'test_station_compute_wait_producer.py',
    'test_arrival_feedback_bracket_gap.py', 'test_arrival_feedback_gap_integration.py',
    'test_arrival_feedback_gap_late_consumers.py',
]
scope = ['src/alicia_grasp_modes/tests/'+name for name in tests]
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
mapping = {}
for path in deliveries:
    delivery = json.loads(path.read_text())
    assert (delivery.get('delivery_ready') is True
            or (delivery.get('source_frozen') is True and delivery.get('ready_for_combined') is True))
    for key in ('files', 'test_files'):
        for name, digest in delivery[key].items():
            if isinstance(digest, dict): digest = digest['sha256']
            if name in mapping: assert mapping[name] == digest
            mapping[name] = digest
    for name in delivery.get('test_files', {}):
        if name.endswith('.py') and '/tests/test_' in name and name not in scope:
            scope.append(name)
for name, digest in mapping.items():
    assert sha(WORK/name) == digest, ('delivery_source_changed', name)
for name in scope:
    assert (WORK/name).exists(), name
replay = '.runtime/replay_gen21_gen22_integrated_appearance_20261010.py'
bound_sources = sorted(set(mapping) | set(scope) | {replay, str(Path(__file__).resolve().relative_to(WORK))})
before = {name: sha(WORK/name) for name in bound_sources}
xml = OUT/(STEM+'.xml')
log = OUT/(STEM+'.log')
command = [sys.executable, '-m', 'pytest', '-q', '-p', 'no:cacheprovider', *scope,
           '--basetemp', str(OUT/(STEM+'_tmp')), '--junitxml', str(xml)]
with log.open('w') as output:
    result = subprocess.run(command, cwd=str(WORK), stdout=output, stderr=subprocess.STDOUT, check=False)
cases = list(ET.parse(str(xml)).getroot().iter('testcase')) if xml.exists() else []
ids = [(case.attrib.get('classname'), case.attrib.get('name')) for case in cases]
assert len(ids) == len(set(ids)), 'Duplicate collected node identity'
failed = [case for case in cases if case.find('failure') is not None]
errors = [case for case in cases if case.find('error') is not None]
skipped = [case for case in cases if case.find('skipped') is not None]
replay_result = None
if result.returncode == 0:
    replay_stem = STEM+'_production_entry'
    replay_log = OUT/(STEM+'_replay.log')
    with replay_log.open('w') as output:
        replay_process = subprocess.run([sys.executable, replay, replay_stem], cwd=str(WORK),
                                        stdout=output, stderr=subprocess.STDOUT, check=False)
    replay_path = OUT.parent/'integrated_appearance_replay'/(replay_stem+'.json')
    replay_result = dict(exit_code=replay_process.returncode, log=dict(path=str(replay_log), sha256=sha(replay_log)))
    if replay_path.exists():
        replay_result.update(path=str(replay_path), sha256=sha(replay_path))
        replay_json = json.loads(replay_path.read_text())
        replay_result['source_unchanged'] = replay_json['source_unchanged']
        for name, digest in replay_json['source_sha256'].items():
            # Include imported unchanged dependencies too, using their own
            # actual before/after measurement from the replay process.
            name = str(Path(name).relative_to(WORK))
            assert name not in before or before[name] == digest
            before[name] = digest
after = {name: sha(WORK/name) for name in before}
manifest = dict(test_scope=scope, command=command, source_sha256=before,
    source_after_sha256=after, source_unchanged=before == after,
    source_binding_note='Changed sources and all test files hashed before pytest; imported unchanged replay dependencies additionally checked by replay pre/post hashes.',
    deliveries=[dict(path=str(path), sha256=sha(path)) for path in deliveries],
    exit_code=result.returncode, test_count_unique=len(cases), passed=len(cases)-len(failed)-len(errors)-len(skipped),
    failed=len(failed), errors=len(errors), skipped=len(skipped),
    failing_nodes=[dict(classname=case.attrib.get('classname'), name=case.attrib.get('name')) for case in failed+errors],
    xml=dict(path=str(xml), sha256=sha(xml)) if xml.exists() else None,
    log=dict(path=str(log), sha256=sha(log)), replay=replay_result,
    environment=dict(PYTHONNOUSERSITE=os.environ.get('PYTHONNOUSERSITE'),
                     PYTHONPATH=os.environ.get('PYTHONPATH'),
                     OPENBLAS_NUM_THREADS=os.environ.get('OPENBLAS_NUM_THREADS'),
                     OMP_NUM_THREADS=os.environ.get('OMP_NUM_THREADS')),
    hardware_access=False, ros_calls=False)
path = OUT/(STEM+'_manifest.json')
path.write_text(json.dumps(manifest, indent=2)+'\n')
print(json.dumps(dict(path=str(path), sha256=sha(path), source_unchanged=manifest['source_unchanged'],
                     passed=manifest['passed'], failed=manifest['failed'], errors=manifest['errors'], skipped=manifest['skipped'],
                     replay=replay_result)))
if result.returncode or before != after or (replay_result and replay_result['exit_code']):
    sys.exit(1)
