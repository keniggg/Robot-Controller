#!/usr/bin/env python3
"""Seal existing offline results, deduplicating test identities; no test execution."""
from pathlib import Path
import collections
import difflib
import hashlib
import json
import tarfile
import time
import xml.etree.ElementTree as ET

WORK = Path(__file__).resolve().parents[1]
OUT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/verification/support_offset_warning_4mm_hard_7mm')
POLICY = WORK / '.runtime/support_offset_policy_20261010'

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def load(path):
    return json.loads(Path(path).read_text())

def save(name, content):
    path = OUT / name
    path.write_text(json.dumps(content, indent=2, ensure_ascii=False) + '\n')
    return {'path': str(path), 'sha256': sha(path)}

def xml_cases(name):
    result = {}
    for case in ET.parse(OUT / name).getroot().findall('.//testcase'):
        identity = case.attrib['classname'] + '::' + case.attrib['name']
        status = ('failed' if case.find('failure') is not None else
                  'error' if case.find('error') is not None else
                  'skipped' if case.find('skipped') is not None else 'passed')
        assert identity not in result
        result[identity] = status
    return result

before = load(POLICY / 'source_before_manifest.json')
review = load(OUT / 'remote_task_independent_review.json')
supervisor = load(OUT / 'supervisor_result.json')
perception = load(POLICY / 'perception_delivery.json')
assert sha(before['archive']) == before['archive_sha256']
for entry in review['files']:
    assert sha(WORK / entry['path']) == entry['sha256'], entry['path']
for path, digest in supervisor['source_sha256'].items():
    assert sha(WORK / path) == digest, path
for entry in perception['files']:
    assert sha(WORK / entry['path']) == entry['sha256'], entry['path']

own_files = [entry['path'] for entry in review['files']]
selected = sorted(set(own_files + list(supervisor['source_sha256']) + [entry['path'] for entry in perception['files']]))
source = {path: {'before_sha256': before['source_sha256'].get(path), 'sha256': sha(WORK / path)} for path in selected}
previous_deployment_path = OUT.parents[1] / 'shared_reference_deployment/result.json'
previous_deployment = load(previous_deployment_path)
deployment_source = dict(previous_deployment['source_sha256'])
deployment_source.update({path: entry['sha256'] for path, entry in source.items()})
runtime_launch = '.runtime/observation_measurement_consumers_20261008.launch'
launch_text = (WORK / runtime_launch).read_text()
assert 'support_max_local_shift_m' in launch_text and 'support_local_shift_warning_m' in launch_text
deployment_source[runtime_launch] = sha(WORK / runtime_launch)
assert not any('/gui/' in path.lower() or '_gui.' in path.lower() or 'widget' in path.lower() for path in deployment_source)
for path, digest in deployment_source.items():
    assert sha(WORK / path) == digest, ('unaccounted_source_drift', path)
deployment_manifest = save('deployment_sources.json', {
    'passed': True,
    'reviewed': True,
    'verification_scope': 'Offline targeted regression passed: 1088 current unique tests passed, 1 skipped; independent remote/task integration review found no blocker. Deployment is still pending.',
    'review_evidence': {
        'path': str(OUT / 'remote_task_independent_review.json'),
        'sha256': sha(OUT / 'remote_task_independent_review.json'),
    },
    'source_sha256': deployment_source,
    'previous_deployment': str(previous_deployment_path),
    'previous_deployment_sha256': sha(previous_deployment_path),
    'source_count': len(deployment_source),
    'new_policy_source_paths': selected,
    'runtime_launch_updated_by_root': runtime_launch,
    'deployment_performed': False,
    'gui_loaded_by_this_change': False,
    'gui_scope_note': 'GUI changes are excluded; they require their own future process restart and verification. This manifest prepares only the compute-node policy deployment.',
})
diff = []
with tarfile.open(before['archive']) as archive:
    names = set(archive.getnames())
    for path in own_files:
        old = archive.extractfile(path).read().decode().splitlines(keepends=True) if path in names else []
        new = (WORK / path).read_text().splitlines(keepends=True)
        diff.extend(difflib.unified_diff(old, new, fromfile='a/' + path, tofile='b/' + path))
(OUT / 'remote_task_changes.diff').write_text(''.join(diff))

suites = {
    'perception_r3': xml_cases('perception_targeted_r3/tests.xml'),
    'remote_task_r2': xml_cases('remote_task_r2.junit.xml'),
    'boundary_r2': xml_cases('boundary_integration_r2.junit.xml'),
    'boundary_fixture_r3': xml_cases('boundary_fixture_r3.junit.xml'),
}
assert collections.Counter(suites['remote_task_r2'].values()) == {'passed': 51}
assert collections.Counter(suites['boundary_r2'].values()) == {'passed': 186, 'failed': 1}
assert collections.Counter(suites['boundary_fixture_r3'].values()) == {'passed': 1}
assert set(suites['boundary_fixture_r3']) == {k for k, v in suites['boundary_r2'].items() if v == 'failed'}
collected = {}
for line in (OUT / 'supervisor_collected_nodeids.log').read_text().splitlines():
    if not line.startswith('tests/') or '::' not in line:
        continue
    components = line.split('::')
    classname = components[0][:-3].replace('/', '.')
    if len(components) > 2:
        classname += '.' + '.'.join(components[1:-1])
    identity = classname + '::' + components[-1]
    assert identity not in collected
    collected[identity] = 'passed'
assert len(collected) == 160 and supervisor['passed']
suites['supervisor_final'] = collected

merged = {}
provenance = collections.defaultdict(list)
for suite, cases in suites.items():
    for identity, status in cases.items():
        merged[identity] = status
        provenance[identity].append({'suite': suite, 'status': status})
historical_counts = dict(collections.Counter(merged.values()))
assert historical_counts == {'passed': 1090, 'skipped': 1}, historical_counts
fixture_replacements = {
    'tests.test_rgb_observation_set_handoff::test_even_empty_proposals_cannot_hide_raw_support_height_failure[-0.0041]':
    'tests.test_rgb_observation_set_handoff::test_even_empty_proposals_cannot_hide_raw_support_height_failure[-0.0071]',
    'tests.test_rgb_observation_set_handoff::test_even_empty_proposals_cannot_hide_raw_support_height_failure[0.0041]':
    'tests.test_rgb_observation_set_handoff::test_even_empty_proposals_cannot_hide_raw_support_height_failure[0.0071]',
}
for old, new in fixture_replacements.items():
    assert merged[old] == merged[new] == 'passed'
current = {key: value for key, value in merged.items() if key not in fixture_replacements}
counts = dict(collections.Counter(current.values()))
assert counts == {'passed': 1088, 'skipped': 1}, counts
intersections = {}
names = list(suites)
for i, left in enumerate(names):
    for right in names[i + 1:]:
        shared = sorted(set(suites[left]) & set(suites[right]))
        if shared:
            intersections[left + ' / ' + right] = shared
nodeids = save('integration_unique_test_ids.json', {
    'unique_status_counts': counts,
    'historical_executed_unique_status_counts': historical_counts,
    'superseded_fixture_nodeids': fixture_replacements,
    'identities': {identity: {'final_status': merged[identity], 'runs': provenance[identity]} for identity in sorted(merged)},
    'intersections': intersections,
    'supervisor_identity_evidence': '160 identities from collect-only at unchanged source hashes; pass evidence is supervisor_result.json and supervisor_targeted_final.log. No tests were repeated for collection.',
})

artifacts = ['remote_task_r2.log', 'remote_task_r2.junit.xml', 'boundary_integration_r2.log',
             'boundary_integration_r2.junit.xml', 'boundary_fixture_r3.log', 'boundary_fixture_r3.junit.xml',
             'supervisor_result.json', 'supervisor_targeted_final.log', 'supervisor_collected_nodeids.log',
             'perception_targeted_r3/tests.xml', 'remote_task_independent_review.json', 'remote_task_changes.diff']
evidence = {name: {'path': str(OUT / name), 'sha256': sha(OUT / name)} for name in artifacts}
own = save('remote_task_result.json', {
    'passed': True, 'offline_only': True, 'source_sha256': {p: source[p] for p in own_files},
    'test_result': '51 passed, 628 deselected', 'test_evidence': evidence['remote_task_r2.junit.xml'],
    'independent_review': evidence['remote_task_independent_review.json'],
    'changes': evidence['remote_task_changes.diff'],
    'preserved_prior_failure': {'log': str(OUT / 'remote_task_regression.log'), 'cause': 'Old offline fixture lacked rospy.logwarn_throttle stub after the new real warning became visible; fixture mock added, production logging retained.'},
    'warning_contract': '4 mm diagnostic only; 7 mm hard default; warning success remains ok=True; all three actual task consumers log after rejection checks.',
})
report = save('integration_result.json', {
    'passed': True, 'recorded_ns': time.time_ns(), 'offline_only': True,
    'policy': {'support_offset_warning_m': .004, 'support_offset_hard_default_m': .007},
    'source_before_manifest': str(POLICY / 'source_before_manifest.json'),
    'selected_source_sha256': source,
    'prepared_deployment_manifest': deployment_manifest,
    'suite_counts': {name: dict(collections.Counter(cases.values())) for name, cases in suites.items()},
    'unique_status_counts': counts, 'historical_executed_unique_status_counts': historical_counts,
    'superseded_fixture_nodeids': fixture_replacements, 'unique_test_id_manifest': nodeids,
    'resolved_failure': {'identity': next(iter(suites['boundary_fixture_r3'])), 'cause': 'The old loose_support fixture treated a 5 mm configured hard limit as illegal. The newly authorized upper bound is 7 mm. Only this fixture changed to 7.01 mm, preserving its rejection assertion; the failed nodeid was rerun and passed.'},
    'preserved_environment_failure': {'log': str(OUT / 'boundary_integration.log'), 'cause': 'Initial collection lacked supervisor test helper directory on PYTHONPATH; corrected environment, no production change.'},
    'evidence': evidence, 'remote_task_result': own,
    'unchanged_checks': ['fit residual bands', '4 mm registration RMS', 'foreground height', 'normal angle', 'identity', 'freshness', 'registration motion limits', 'contact evidence', 'collision and path gates'],
    'limitations': ['No new ROS, device access, deployment, or physical grasp performed by this integration run.', 'The suite is targeted and deduplicated across existing completed runs; it is not a claim that every repository test passed.', 'The one skip remains a skip.'],
})
print(json.dumps({'remote_task_result': own, 'integration_result': report, 'counts': counts}, ensure_ascii=False))
