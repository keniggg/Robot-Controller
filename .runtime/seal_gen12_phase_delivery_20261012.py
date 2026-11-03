"""Seal reviewed GEN12 staging and evidence; never install or contact ROS."""
import argparse
import difflib
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

W = Path(__file__).resolve().parents[1]
STAGE = W / '.runtime/gen12_phase_entry_frame_owner_20261012'
N = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
VERIFY = N / 'verification'
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
read = lambda p: json.loads(Path(p).read_text())
ap = argparse.ArgumentParser()
ap.add_argument('--author', type=Path, required=True)
ap.add_argument('--negative', type=Path, required=True)
ap.add_argument('--output', type=Path, required=True)
a = ap.parse_args()
assert Path('/media/zhuyupei/SanStand').is_mount()
assert VERIFY in a.output.resolve().parents and not a.output.exists()
assert not (STAGE / 'delivery_manifest.json').exists()

def record(path):
    return dict(path=str(path), sha256=sha(path))

def tested(folder, count):
    data = read(folder / 'result.json')
    assert data['passed'] and data['source_unchanged']
    counts = {k:sum(int(s.attrib.get(k, 0)) for s in ET.parse(folder/'pytest.xml').iter('testsuite'))
              for k in ('tests', 'failures', 'errors', 'skipped')}
    assert counts == dict(tests=count, failures=0, errors=0, skipped=0)
    assert data['counts'] == counts and data['pytest_xml_sha256'] == sha(folder/'pytest.xml')
    for p, h in data['source_sha256'].items():
        resolved = Path(p) if Path(p).is_absolute() else W/p if p.startswith('.runtime/') else STAGE/p
        assert sha(resolved) == h, p
    return data, dict(counts=counts, files={p.name:record(p) for p in folder.iterdir() if p.is_file()})

prior_path = VERIFY / 'gen11_contact_release_manifest.json'
prior = read(prior_path)
assert prior['passed'] and all(sha(W/p) == h for p,h in prior['source_sha256'].items())
author, author_evidence = tested(a.author, 214)
assert author['actual_unchanged'] and author['tests_unchanged'] and author['no_ros_actions']
assert all(sha(STAGE/p) == h for p,h in author['test_sha256'].items())
independent, independent_evidence = tested(VERIFY/'gen12_qualified_phase_independent_r7', 52)
original_test = W/'.runtime/gen12_phase_entry_independent_20261012/test_gen12_qualified_phase_independent.py'
assert sha(original_test) == sha(STAGE/'src/alicia_grasp_modes/tests/test_qualified_station_phase.py')

files = []
for p in sorted((STAGE/'src').rglob('*')):
    if not p.is_file() or p.is_symlink() or '__pycache__' in p.parts or p.suffix == '.pyc':
        continue
    relative = str(p.relative_to(STAGE))
    old = sha(W/relative) if (W/relative).exists() else None
    if old != sha(p):
        files.append(dict(path=relative, base_sha256=old, candidate_sha256=sha(p)))
assert len(files) == 7 and all(r['path'].endswith('.py') for r in files)
assert sum('/tests/' not in r['path'] for r in files) == 4
assert {r['path'] for r in files if '/tests/' not in r['path']} == set(author['source_sha256'])
assert all(r['candidate_sha256'] == author['test_sha256'][r['path']] for r in files if '/tests/' in r['path'])
scope = list(author['test_sha256'])
assert len(scope) == 7

# The independently recorded BAD-frame replay is a rejection check, not a
# proposed successful grasp. The caller supplies its final hash-bound receipt.
negative = read(a.negative)
assert negative.get('passed') is True
assert negative.get('source_unchanged') is True
assert negative['expected_rejects'] == 2
assert all(sha(p) == h for p,h in negative['dependencies_sha256'].items())
assert sha(W/'.runtime/replay_attempt1_full_process_negative_20261013.py') == negative['harness_sha256']
assert sha(W/'.runtime/replay_gen12_registered_bad_negative_20261013.py') == negative['wrapper_sha256']
for row, expected in zip(negative['rows'], ('source_exceeds_existing_frame_limit',
        'metric_ambiguity_no_unique_complete_rgb_identity')):
    assert row['registered_good'] and not row['detected'] and row['state'] == 'unavailable'
    assert row['detail'] == 'perception_failure: '+expected
    following = row['following_actual_saved_frame']
    assert following['detail'] == 'target_lost_requires_new_generation' and not following['detected']
    assert following['first_loss_preserved'] and following['last_good_ns'] == row['last_good_ns']
fixture_path = VERIFY/'attempt1_first_phase_independent_v2/fixture.json'
fixture = read(fixture_path)
assert sha(Path(fixture['record'])/'keyframes.bag') == fixture['bag_sha256']
negative_sources = negative['source_sha256']
for relative in ('src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py',
                 'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_producer.py'):
    path = STAGE/relative
    assert negative_sources.get(str(path), negative_sources.get(relative)) == sha(path), relative

patch = ''.join(''.join(difflib.unified_diff(
    (W/r['path']).read_text().splitlines(True) if (W/r['path']).exists() else [],
    (STAGE/r['path']).read_text().splitlines(True), fromfile='a/'+r['path'], tofile='b/'+r['path'])) for r in files)
(STAGE/'delivery.patch').write_text(patch)
limits = [
    'GEN12 repairs qualified-arrival planning phase timing and timely predecessor-frame ownership only; physical grasp and the recorded BAD identity/fragment problem remain unverified and unresolved by this delivery.',
    'The recorded BAD frame retains the original source-age rejection with the original schedule and the original no-unique-complete-RGB rejection under the timely-identity diagnostic schedule. Neither schedule is a physical result.',
    'Original 0.8-second frame/reference limits, planning/session deadlines, collision sampling, solver iterations, maximum observations and no-lift policy remain unchanged.',
    'Independent task tests execute real task MRO, contracts, ACK gates, target association and phase creation; origin capture/transport are inert and bound-plan collision validation is isolated by the existing fixture.',
    'Caller tests execute original first/supplement reached gates with an observation motion sentinel. Original start_cb finally remains responsible for phase deactivation on terminal task exit.',
    'The 52 independent tests are included in the 214-case combined focused suite; counts must not be added.',
    'Actual installed union regression is required before deployment; this operation does not install, reload nodes, query temperature or send commands.'
]
manifest = dict(schema=1, kind='gen12_qualified_arrival_phase_and_frame_ownership', passed=True,
    recorded_ns=time.time_ns(), candidate_root=str(STAGE), stage=str(STAGE), files=files,
    source_sha256={r['path']:r['candidate_sha256'] for r in files}, test_scope=scope,
    validation=dict(author_final=author_evidence, independent_phase=independent_evidence,
                    recorded_bad_negative=record(a.negative)),
    prior_release=record(prior_path), current_baseline_sources=len(prior['source_sha256']),
    patch=record(STAGE/'delivery.patch'),
    independent_test_provenance=record(original_test),
    installer=record(W/'.runtime/reviewed_grasp_patch_20261012.py'), limits=limits,
    production_installed=False, physical_grasp_verified=False, motion_commands_sent=False, temperature_queried=False)
payload=json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True)+'\n'
a.output.mkdir()
(STAGE/'delivery_manifest.json').write_text(payload)
(a.output/'delivery_manifest.json').write_text(payload)
review=dict(schema=1, passed=True, recorded_ns=time.time_ns(),
    delivery=str(STAGE/'delivery_manifest.json'), delivery_sha256=sha(STAGE/'delivery_manifest.json'),
    test_scope=scope, source_sha256=manifest['source_sha256'], validation=manifest['validation'],
    source_review=[
        'Actual qualified dual ACK and source-bound geometry precede PLAN and planning phase; motion and contact gates are unchanged.',
        'Receipt, current ACKs and immutable selection/session/lease/plan bindings are checked under the plan and condition locks; reuse checks both published phase and internal phase state without renewing ROS or monotonic deadlines.',
        'Owner loss at actual phase publication causes immediate inactive publication; original start_cb finally still retires phase after caller failures.',
        'Producer cancellation occurs only at explicit checkpoints for a timely first-phase predecessor with an unchanged qualified station. It neither catches known processing exceptions nor changes the old source stamp/reference.',
        'Cancellation accepts only the actual original qualified source or the actual current reference source; arbitrary timestamps between those sources do not authorize cancellation.',
        'Final input-lock ordering is tested by real callback threads. A callback before commit retires predecessor output; a callback after commit does not retroactively invalidate its earlier measured publication.'
    ], limits=limits,
    sealing_script=record(Path(__file__).resolve()), production_sources_unchanged=True,
    motion_commands_sent=False, temperature_queried=False, production_installed=False)
assert all(sha(W/p) == h for p,h in prior['source_sha256'].items())
(a.output/'independent_review.json').write_text(json.dumps(review, ensure_ascii=False, indent=2, sort_keys=True)+'\n')
receipt=dict(passed=True, delivery=record(STAGE/'delivery_manifest.json'),
    external_mirror=record(a.output/'delivery_manifest.json'), review=record(a.output/'independent_review.json'),
    changed_files=len(files), test_scope_files=len(scope), tests=author['counts'],
    production_sources_unchanged=True, production_installed=False,
    physical_grasp_verified=False, motion_commands_sent=False, temperature_queried=False)
(a.output/'result.json').write_text(json.dumps(receipt, ensure_ascii=False, indent=2, sort_keys=True)+'\n')
print(json.dumps(receipt, ensure_ascii=False))
