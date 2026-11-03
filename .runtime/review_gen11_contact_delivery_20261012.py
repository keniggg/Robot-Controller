"""Read-only artifact verification; writes only the independent review receipt."""
from pathlib import Path
import hashlib
import json
import time
import xml.etree.ElementTree as ET

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
STAGE = W / '.runtime/gen11_contact_compute_admission_20261012'
S = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_frozen_no_skin_20261012T021050Z')
V = S / 'verification'
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
read = lambda p: json.loads(Path(p).read_text())

manifest = STAGE / 'delivery_manifest.json'
delivery = read(manifest)
assert delivery['passed']
assert len(delivery['files']) == 9
files = {}
for row in delivery['files']:
    rel = row['path']
    wanted = row.get('candidate_sha256') or row['sha256']
    assert sha(STAGE / rel) == wanted, rel
    base = sha(W / rel) if (W / rel).exists() else None
    assert base == row['base_sha256'], ('actual base changed', rel)
    files[rel] = dict(candidate_sha256=wanted, base_sha256=base)

evidence = []
scope = []
for rel, expected in [('gen11_contact_task_final_r3', 393),
                      ('gen11_contact_remote_stage/final', 144),
                      ('gen11_contact_independent_final_r2', 40)]:
    result_path = V / rel / 'result.json'
    result = read(result_path)
    assert result['passed'] and result['exit_code'] == 0
    xml = V / rel / 'pytest.xml'
    tree = ET.parse(xml).getroot()
    counts = {k: sum(int(x.get(k, 0)) for x in tree.findall('testsuite'))
              for k in ('tests', 'errors', 'failures', 'skipped')}
    assert counts == dict(tests=expected, errors=0, failures=0, skipped=0)
    assert len(list(tree.iter('testcase'))) == expected
    if 'pytest_xml_sha256' in result:
        assert result['pytest_xml_sha256'] == sha(xml)
    if 'source_sha256' in result:
        assert result['source_unchanged']
        for path, digest in result['source_sha256'].items():
            assert sha(STAGE / path) == digest, path
    else:
        assert result['candidate_sources_before'] == result['candidate_sources_after']
        for path, digest in result['candidate_sources_before'].items():
            assert sha(path) == digest, path
    scope.extend(result.get('test_scope', []))
    evidence.append(dict(path=str(result_path), sha256=sha(result_path),
                         pytest_xml=str(xml), pytest_xml_sha256=sha(xml), counts=counts))

independent = 'src/alicia_grasp_modes/tests/test_gen11_contact_compute_admission_independent.py'
fixture = 'src/alicia_grasp_modes/tests/fixtures/gen11_contact_transport.json.gz'
comparison = []
for rel in (independent, fixture):
    source = W / '.runtime/gen11_task_admission_review_20261012' / rel.split('/tests/', 1)[1]
    assert (STAGE / rel).read_bytes() == source.read_bytes()
    comparison.append(dict(path=rel, sha256=sha(STAGE / rel), independent_original_path=str(source)))
scope.append(independent)
scope = sorted(set(scope))
assert all(p.startswith('src/') and p.endswith('.py') and (STAGE / p).is_file() for p in scope)

wire_path = S / 'gen11_analysis/task_admission_independent/candidate_original_wire_no_receipt.json'
wire = read(wire_path)
assert wire['original_validators_replaced'] is False
assert wire['admit_source'] is True
assert wire['cached_preview_id'] == '2b1ab71f0919c17e5d6ce5bb'
assert len(wire['checks']) == 18
assert all(not r['task']['ok'] and r['base']['ok']
           and r['task']['reason'] == 'waiting for exact contact computation audit receipt'
           and r['candidate_plan_id'] is None for r in wire['checks'])
assert wire['audit_probe']['original_contact_rebind_entry']['sentinel_calls'] == 0
for path, digest in wire['source_sha256'].items():
    assert sha(W / path) == digest, path
assert sha(S / 'gen11_analysis/contact_promotion_evidence/wire.jsonl.gz') == wire['wire_sha256']

review = dict(
    schema=1, passed=True, reviewed_ns=time.time_ns(),
    reviewer='independent review_contact_repair',
    delivery=str(manifest), delivery_sha256=sha(manifest),
    candidate_root=str(STAGE), files=files, test_scope=scope,
    verification_evidence=evidence, independent_fixture_comparison=comparison,
    original_wire_counterexample=dict(path=str(wire_path), sha256=sha(wire_path),
        wire_sha256=wire['wire_sha256'], base_valid_checks=18,
        no_receipt_task_rejections=18, action_sentinel_calls=0),
    scope=[
        'Read actual staged source, independent three-actor tests and final evidence; no changes to production or candidate source.',
        'Original producer RGBD joins and qualified handshake, remote ticket and preview publication, original atomic audit finalizer and task receipt callback.',
        'Original task complete candidate validation and maybe-rebind loop, before-freeze hook, atomic action guard/freeze, original bound-authority checks.',
        'Both real producer/remote prepare and retire contexts and ACK callbacks, then original action-entry wrapper with inert physical executor.',
        'Late audit, six input-topic permutations, receipt/source/digest/epoch tampering, invalid preview, stop/manual/epoch/deadline changes, callback lock gaps, original-source contradictions and audit-commit failure negatives.'
    ],
    source_review=[
        'Receipt preserves original three source stamps and exact canonical plan; all inputs must match task independently admitted witnesses and current measured registration.',
        'Only typed clock expiry keeps private computation; real invalidity, source contradictions, unresolved false ownership and changed owner remain blocked.',
        'Same provenance check at candidate copy, before-freeze and atomic metadata commit; a stage label alone does not authorize an old observation action.',
        'After exact freeze the immutable plan/digest/epoch is bound; original downstream authority, SDK, strict movement and dual retirement ACK gates remain.',
        'Remote retirement lookup requires exact published proposal and successful original atomic audit commit; it does not repurpose expired station geometry as a new motion gate.',
        'Original phase/session deadlines, strict solver/collision settings, source validity, temperature policy and physical execution code remain unchanged.'
    ],
    limitations=[
        'These are offline protocol and regression artifacts, not hardware execution, a new collision/path calculation or physical grasp evidence.',
        'Independent positive cases translate recorded GEN11 geometry/report into a synthetic actual RGBD producer scene; geometric report builder input and transport/physical execution are fixtures.',
        'Independent final_r2 originally bound five production source hashes before/after but did not bind test/fixture hashes. This review verifies exact byte equality with the original review copies and includes the installable test in required actual combined regression.',
        '393, 144 and 40 are separate test runs with overlapping existing tests; their sum is not a unique-test count.',
        'Installation, complete actual-worktree regression and live restart/readiness remain root responsibilities; this receipt authorizes reviewable installation, not a claim of physical readiness.'
    ],
    fresh_test_execution=False, hardware_or_ros_io_performed=False,
    required_next_check='Run the complete installer-unioned actual-worktree test_scope and bind release to unchanged installed source hashes before deployment.')
output = V / 'gen11_contact_admission_independent_review.json'
assert not output.exists(), 'Do not overwrite an existing review receipt'
output.write_text(json.dumps(review, indent=2, ensure_ascii=False) + '\n')
print(json.dumps(dict(passed=True, path=str(output), sha256=sha(output),
                     delivery_sha256=sha(manifest), test_files=len(scope)), ensure_ascii=False))
