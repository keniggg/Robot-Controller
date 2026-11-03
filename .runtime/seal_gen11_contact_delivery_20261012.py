"""Seal already-tested staged sources without installing or running ROS."""
import base64
import difflib
import gzip
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

W = Path(__file__).resolve().parents[1]
STAGE = W / '.runtime/gen11_contact_compute_admission_20261012'
EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
SESSION = EXT / 'ros_log/full_latest_frozen_no_skin_20261012T021050Z'
VERIFY = SESSION / 'verification'
OUT = VERIFY / 'gen11_contact_delivery_seal'
sha = lambda path: hashlib.sha256(Path(path).read_bytes()).hexdigest()
read = lambda path: json.loads(Path(path).read_text())

assert Path('/media/zhuyupei/SanStand').is_mount()
assert not OUT.exists()
assert not (STAGE / 'delivery_manifest.json').exists()
prior_path = VERIFY / 'gen10_handoff_fixture_integrated_install/result.json'
prior = read(prior_path)
assert prior['passed']
assert all(sha(W / p) == h for p, h in prior['source_sha256'].items())

changed = []
for candidate in sorted((STAGE / 'src').rglob('*')):
    if not candidate.is_file() or candidate.is_symlink():
        continue
    relative = str(candidate.relative_to(STAGE))
    base = W / relative
    base_sha = sha(base) if base.exists() else None
    candidate_sha = sha(candidate)
    if base_sha != candidate_sha:
        changed.append(dict(path=relative, base_sha256=base_sha,
                            candidate_sha256=candidate_sha))
assert len(changed) == 9
source_sha = {r['path']: r['candidate_sha256'] for r in changed}
assert sum('/tests/' not in p for p in source_sha) == 5
assert sum(p.endswith('.py') and '/tests/' in p for p in source_sha) == 3
assert sum(p.endswith('.gz') for p in source_sha) == 1

patch = (STAGE / 'delivery.patch').read_text()
patch_paths = [line[6:] for line in patch.splitlines() if line.startswith('+++ b/')]
assert set(patch_paths) == {p for p in source_sha if p.endswith('.py')}
expected_patch = ''.join(''.join(difflib.unified_diff(
    (W / p).read_text().splitlines(True) if (W / p).exists() else [],
    (STAGE / p).read_text().splitlines(True), fromfile='a/' + p, tofile='b/' + p))
    for p in patch_paths)
assert expected_patch == patch

validation = {}
scope = []
expected_counts = {'task_final': ('gen11_contact_task_final_r3', 393),
                   'remote_final': ('gen11_contact_remote_stage/final', 144),
                   'independent_final': ('gen11_contact_independent_final_r2', 40)}
for name, (directory, expected_count) in expected_counts.items():
    folder = VERIFY / directory
    result = read(folder / 'result.json')
    assert result['passed'] and result['exit_code'] == 0
    suites = list(ET.parse(folder / 'pytest.xml').getroot().iter('testsuite'))
    counts = {k: sum(int(s.attrib.get(k, 0)) for s in suites)
              for k in ('tests', 'failures', 'errors', 'skipped')}
    assert counts == dict(tests=expected_count, failures=0, errors=0, skipped=0)
    if 'counts' in result:
        assert result['counts'] == counts and result['source_unchanged']
        assert result['pytest_xml_sha256'] == sha(folder / 'pytest.xml')
    for relative, h in result.get('source_sha256', {}).items():
        assert sha(STAGE / relative) == h
    if 'candidate_sources_before' in result:
        assert result['candidate_sources_before'] == result['candidate_sources_after']
        assert all(sha(p) == h for p, h in result['candidate_sources_after'].items())
    for info in result.get('imports', {}).values():
        assert sha(info['path']) == info['sha256']
    scope.extend(result.get('test_scope', []))
    validation[name] = dict(counts=counts, files={p.name: dict(path=str(p), sha256=sha(p))
        for p in sorted(folder.iterdir()) if p.is_file()})

independent_relative = 'src/alicia_grasp_modes/tests/test_gen11_contact_compute_admission_independent.py'
fixture_relative = 'src/alicia_grasp_modes/tests/fixtures/gen11_contact_transport.json.gz'
original_review = W / '.runtime/gen11_task_admission_review_20261012'
assert sha(STAGE / independent_relative) == sha(original_review / Path(independent_relative).name)
assert sha(STAGE / fixture_relative) == sha(original_review / 'fixtures' / Path(fixture_relative).name)
scope.append(independent_relative)
scope = list(dict.fromkeys(scope))
assert all((STAGE / p).is_file() for p in scope)

fixture = json.loads(gzip.decompress((STAGE / fixture_relative).read_bytes()))
wire_path = SESSION / 'gen11_analysis/contact_promotion_evidence/wire.jsonl.gz'
assert sha(wire_path) == fixture['wire_sha256']
audit_path = EXT / ('grasp_records/20261012T140905.069509Z_022d4230fcb4456d8808a52b6c1db9ca/'
                    'keyframes_audits/' + fixture['original_report_sha256'] + '.json')
assert sha(audit_path) == fixture['original_report_sha256']
audit = read(audit_path)
assert all(audit.get(k) == v for k, v in fixture['report'].items())
rows = list(fixture['wire'].values())
for row in rows:
    assert hashlib.sha256(base64.b64decode(row['serialized_base64'])).hexdigest() == row['serialized_sha256']
found = set()
with gzip.open(wire_path, 'rt') as stream:
    for line in stream:
        event = json.loads(line)
        for index, row in enumerate(rows):
            if event == row:
                found.add(index)
assert found == set(range(len(rows)))

provenance = dict(
    fixture=dict(path=str(STAGE / fixture_relative), sha256=sha(STAGE / fixture_relative)),
    original_wire=dict(path=str(wire_path), sha256=sha(wire_path)),
    original_audit=dict(path=str(audit_path), sha256=sha(audit_path)),
    wire_rows_identical=True, audit_selected_fields_identical=True,
    original_independent_test=dict(path=str(original_review / Path(independent_relative).name),
                                   sha256=sha(STAGE / independent_relative)),
    original_independent_fixture=dict(path=str(original_review / 'fixtures' / Path(fixture_relative).name),
                                      sha256=sha(STAGE / fixture_relative)))

manifest = dict(schema=1, passed=True, recorded_ns=time.time_ns(),
    kind='gen11_registered_station_contact_compute_admission',
    stage=str(STAGE), candidate_root=str(STAGE), files=changed,
    source_sha256=source_sha, test_scope=scope,
    test_summary={k: v['counts'] for k, v in validation.items()},
    validation=validation, fixture_provenance=provenance,
    prior_installation=dict(path=str(prior_path), sha256=sha(prior_path),
                            current_sources_verified=len(prior['source_sha256'])),
    patch=dict(path=str(STAGE / 'delivery.patch'), sha256=sha(STAGE / 'delivery.patch'),
               text_file_count=len(patch_paths), binary_fixture_supplied_by_manifest=True),
    design=dict(path=str(STAGE / 'DESIGN.json'), sha256=sha(STAGE / 'DESIGN.json')),
    installer=dict(path=str(W / '.runtime/reviewed_grasp_patch_20261012.py'),
                   sha256=sha(W / '.runtime/reviewed_grasp_patch_20261012.py')),
    limits=[
        'Staged delivery only; not installed, deployed, or a physical grasp result.',
        'Task/remote/independent test suites overlap; 393, 144, and 40 must not be added as unique tests.',
        'Historical independent final result bound five production sources, but did not record test/fixture hashes. Current staged test and fixture exactly match original review copies and are pinned here; full installed regression remains required.',
        'The 40-case replay uses original validators with translated recorded geometry, synthetic RGB-D and inert transport; no physical collision/path or grasp outcome validated.',
        'DESIGN.json retains its historical pause instruction as provenance; current operator authorization is controlled separately by root.',
        'Original deadlines, collision sampling, iteration limits, no-lift setting, maximum two observations and temperature policy unchanged.'
    ], motion_commands_sent=False, temperature_queried=False, production_installed=False)
OUT.mkdir()
payload = json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True) + '\n'
(STAGE / 'delivery_manifest.json').write_text(payload)
(OUT / 'delivery_manifest.json').write_text(payload)
receipt = dict(passed=True, recorded_ns=time.time_ns(),
    delivery=str(STAGE / 'delivery_manifest.json'), delivery_sha256=sha(STAGE / 'delivery_manifest.json'),
    external_mirror=str(OUT / 'delivery_manifest.json'),
    source_count=len(changed), test_scope_count=len(scope),
    source_sha256=source_sha, fixture_provenance=provenance,
    sealing_script=dict(path=str(Path(__file__).resolve()), sha256=sha(__file__)),
    production_sources_unchanged=all(sha(W / p) == h for p, h in prior['source_sha256'].items()),
    motion_commands_sent=False, temperature_queried=False, production_installed=False)
assert receipt['production_sources_unchanged']
(OUT / 'result.json').write_text(json.dumps(receipt, ensure_ascii=False, indent=2, sort_keys=True) + '\n')
print(json.dumps({k: receipt[k] for k in ('passed', 'delivery', 'delivery_sha256', 'source_count', 'test_scope_count')}, ensure_ascii=False))
