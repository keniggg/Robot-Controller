"""Seal independent GEN13 review and exact source/evidence hashes, no install."""
import difflib
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

W = Path(__file__).resolve().parents[1]
STAGE = W/'.runtime/gen13_station_fragment_identity_20261012'
N = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
V = N/'verification'
OUT = V/'gen13_station_identity_delivery_seal'
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
read = lambda p: json.loads(Path(p).read_text())
record = lambda p: dict(path=str(p), sha256=sha(p))
assert Path('/media/zhuyupei/SanStand').is_mount() and not OUT.exists()

def resolve(p):
    p = Path(p)
    return p if p.is_absolute() else W/p if str(p).startswith('.runtime/') else STAGE/p

def tested(folder, count):
    data = read(folder/'result.json')
    counts = {k:sum(int(s.attrib.get(k, 0)) for s in ET.parse(folder/'pytest.xml').iter('testsuite'))
              for k in ('tests', 'failures', 'errors', 'skipped')}
    assert data['passed'] and data['source_unchanged']
    assert counts == dict(tests=count, failures=0, errors=0, skipped=0)
    if 'counts' in data: assert data['counts'] == counts
    if 'pytest_xml_sha256' in data: assert data['pytest_xml_sha256'] == sha(folder/'pytest.xml')
    for p,h in data['source_sha256'].items(): assert sha(resolve(p)) == h, p
    return data, dict(counts=counts, files={p.name:record(p) for p in folder.iterdir() if p.is_file()})

prior_path = V/'gen12_phase_integrated_install/result.json'
prior = read(prior_path)
assert prior['passed'] and len(prior['source_sha256']) == 885
assert all(sha(W/p) == h for p,h in prior['source_sha256'].items())
author, author_evidence = tested(V/'gen13_station_appearance_focused_final_r5', 487)
assert author['actual_unchanged'] and author['tests_unchanged'] and author['no_ros_actions']
assert all(sha(STAGE/p) == h for p,h in author['test_sha256'].items())
independent, independent_evidence = tested(V/'gen13_station_identity_and_typed_independent_r9', 98)
control, control_evidence = tested(V/'gen13_control_flow_candidate', 6)
historical_reproductions = {}
for name,total,failures in [('gen13_support_guard_baseline',5,1),
                            ('gen13_support_guard_expectation',5,0),
                            ('gen13_control_flow_baseline',6,4)]:
    folder = V/name; data = read(folder/'result.json')
    counts = {k:sum(int(s.attrib.get(k,0)) for s in ET.parse(folder/'pytest.xml').iter('testsuite'))
              for k in ('tests','failures','errors','skipped')}
    assert counts == dict(tests=total,failures=failures,errors=0,skipped=0)
    if 'source_sha256' in data:
        for p,h in data['source_sha256'].items():
            path = Path(p) if Path(p).is_absolute() else W/p
            assert sha(path) == h
    historical_reproductions[name] = dict(counts=counts,
        files={p.name:record(p) for p in folder.iterdir() if p.is_file()})
test_originals = {}
for old,new in [('test_gen13_station_identity_independent.py','test_qualified_station_identity_independent.py'),
                ('test_gen13_typed_fragment_propagation.py','test_station_fragment_typed_propagation.py')]:
    original = W/'.runtime/gen13_station_identity_independent_20261012'/old
    installed = STAGE/'src/alicia_grasp_modes/tests'/new
    assert sha(original) == sha(installed)
    test_originals[str(installed.relative_to(STAGE))] = record(original)

files = []
for p in sorted((STAGE/'src').rglob('*')):
    if not p.is_file() or p.is_symlink() or '__pycache__' in p.parts or p.suffix == '.pyc': continue
    relative = str(p.relative_to(STAGE)); old = sha(W/relative) if (W/relative).exists() else None
    if old != sha(p): files.append(dict(path=relative, base_sha256=old, candidate_sha256=sha(p)))
assert len(files) == 11 and all(r['path'].endswith('.py') for r in files)
assert {r['path'] for r in files if '/tests/' not in r['path']} == set(author['source_sha256'])
assert len(author['source_sha256']) == 4
assert all(r['candidate_sha256'] == author['test_sha256'][r['path']] for r in files if '/tests/' in r['path'])
scope = list(author['test_sha256']); assert len(scope) == 22

replay_path = V/'gen13_full_qualified_station_process_v5/validation_receipt.json'
replay = read(replay_path)
assert replay['passed'] and replay['source_unchanged']
assert replay['source_sha256'] == {str(STAGE/p):h for p,h in author['source_sha256'].items()}
assert all(sha(resolve(p)) == h for p,h in replay['dependencies_sha256'].items())
assert (replay['fresh_current_geometry_count'], replay['natural_fifo_expired_pending_count'],
        replay['deterministic_post_sdk_deadline_expected_pending_count'],
        replay['full_process_no_qualified_callback_expected_rejects'],
        replay['selector_lineage_tamper_expected_rejects']) == (4, 1, 1, 2, 9)
assert replay['phase_schedule_explicitly_counterfactual'] and replay['all_new_computation_elapsed_charged']
assert not replay['source_restamped'] and not replay['old_metric_geometry_reused']
positive = read(replay['positive_result'])
assert positive['passed'] and positive['full_saved_sequence_replayed'] and len(positive['rows']) == 5
assert [r['state'] for r in positive['rows']] == ['ready']*4+['processing_deadline_pending']
assert all(r['first_loss'] is None and r['template_object_unchanged'] for r in positive['rows'])
assert positive['rows'][-1]['last_good_ns'] == positive['rows'][-2]['source_ns']
fixture_path = V/'attempt1_first_phase_with_qualified_source/fixture.json'
fixture = read(fixture_path)
bag = Path(fixture['record'])/'keyframes.bag'
assert sha(bag) == fixture['bag_sha256']

# This stage was cloned from GEN12, including its old metadata. Preserve those
# exact inherited bytes before replacing only the new stage's delivery files.
inherited = {}
for name in ('delivery_manifest.json', 'delivery.patch'):
    p = STAGE/name; original = W/'.runtime/gen12_phase_entry_frame_owner_20261012'/name
    assert p.read_bytes() == original.read_bytes()
    inherited[name] = dict(data=p.read_bytes(), original=record(original))
patch = ''.join(''.join(difflib.unified_diff(
    (W/r['path']).read_text().splitlines(True) if (W/r['path']).exists() else [],
    (STAGE/r['path']).read_text().splitlines(True), fromfile='a/'+r['path'], tofile='b/'+r['path'])) for r in files)
limits = [
    'GEN13 fixes same-station fragment appearance provenance and typed current-source expiry propagation; no physical grasp success is established by this delivery.',
    'Original premotion three-frame confirmation, geometric association, per-native core ownership, current depth/support and colour/shape thresholds remain required; fixed third-qualified full-outline IoU >= 0.85 is additional.',
    'Only the real final qualified callback freezes the third accepted frame; final input-lock owner/SDK/context and unchanged 0.8-second source lifetime are checked before marker and ACK commit. READY never rolls the station template.',
    'Expiry retains only an already registered, independently valid station under its original fixed identity deadline. Old geometry is never republished, timestamps are never renewed, and true station/identity failures cannot be relabelled by ordinary fragment fallback.',
    'The 98 independent and 6 root control-flow tests are included in 487 focused tests; their counts must not be added. Actual installed combined regression is still required before runtime deployment.',
    'Independent pixel replay uses all five saved post-qualification frames with original timestamps/TF/SDK and elapsed new computation charged: four current READY, then one expired frame pending. Forced FIFO is not the live latest-only queue; absent camera frames cannot be reconstructed.',
    'The first two qualified images were not preserved; their successful original three-frame receipt is explicitly seeded, while third-frame pixels, qualified callback and later full process calculations execute real candidate code. Early phase scheduling is a stated counterfactual.',
    'The post-SDK +0.801-second test is a deterministic fault injection and not a performance claim. All prior failed, misclassified and fixture-error reports remain preserved.',
    'No hardware motion, temperature query, source installation or node reload occurs in this sealing operation.'
]
source_review = [
    'The production diff has exactly four files: immutable station appearance with final copy-window revalidation; selected template in normal fragment and station recovery; added fixed complete-outline IoU; existing typed deadline/retirement/bracket propagation.',
    'Provenance binds exact third source, full accepted three-source list, pixel/mask hashes, source context and full measurement ownership. Plain READY, missing callback, new lease, earlier/current source and tampered owner cannot create station identity.',
    'Actual callback tests cover concurrent SDK out-and-back, manual, epoch, model, source expiry and pending/hold replacement both during immutable copy and while waiting for final input lock. A process thread cannot use the template before the callback finishes.',
    'Arrival-only optional bracket freshness callback preserves existing default behavior for other callers. Original registered scope is checked before typed current expiry and again by the original timeout router; a simultaneous true manual/pose/epoch/model fault is not accepted as pending.',
    'Four typed outcomes retain the same exception instance through two actual native refinements, two native-core checks, both member appearances and final consensus appearance, including real process optional proposal catch to outer router. Plain original geometry/identity ValueError keeps its prior wrapper/fallback and real separate-object counterexamples retain rejection.',
    'The deterministic real-pixel post-SDK deadline reproduction was terminal before the fix and pending after it with unchanged last-good and no first loss. No-qualified-callback BAD followed by NEXT retains original source-age/no-unique-RGB rejection.',
    'Two existing tests were updated to assert new qualified-template cache provenance and an already-existing multi-match geometry rejection reason. No rejection or numerical acceptance threshold is weakened.'
]
validation = dict(author_final=author_evidence, independent_template_and_typed=independent_evidence,
                  root_control_flow=control_evidence, real_pixel_process=record(replay_path),
                  historical_reproductions=historical_reproductions,
                  original_keyframes_bag=dict(path=str(bag),sha256=fixture['bag_sha256']))
OUT.mkdir()
for name, item in inherited.items(): (OUT/('inherited_gen12_'+name)).write_bytes(item['data'])
(STAGE/'delivery.patch').write_text(patch)
manifest = dict(schema=1, kind='gen13_fixed_qualified_station_identity_and_typed_source_deadline', passed=True,
    recorded_ns=time.time_ns(), candidate_root=str(STAGE), stage=str(STAGE), files=files,
    source_sha256={r['path']:r['candidate_sha256'] for r in files}, test_scope=scope,
    validation=validation, prior_release=record(prior_path), current_baseline_sources=885,
    patch=record(STAGE/'delivery.patch'), independent_test_provenance=test_originals,
    installer=record(W/'.runtime/reviewed_grasp_patch_20261012.py'), limits=limits,
    production_installed=False, physical_grasp_verified=False, motion_commands_sent=False, temperature_queried=False)
payload = json.dumps(manifest,ensure_ascii=False,indent=2,sort_keys=True)+'\n'
(STAGE/'delivery_manifest.json').write_text(payload); (OUT/'delivery_manifest.json').write_text(payload)
review = dict(schema=1, passed=True, recorded_ns=time.time_ns(),
    delivery=str(STAGE/'delivery_manifest.json'), delivery_sha256=sha(STAGE/'delivery_manifest.json'),
    test_scope=scope, source_sha256=manifest['source_sha256'], validation=validation,
    source_review=source_review, limits=limits, sealing_script=record(Path(__file__).resolve()),
    production_sources_unchanged=True, motion_commands_sent=False, temperature_queried=False,
    production_installed=False, physical_grasp_verified=False)
assert all(sha(W/p) == h for p,h in prior['source_sha256'].items())
(OUT/'independent_review.json').write_text(json.dumps(review,ensure_ascii=False,indent=2,sort_keys=True)+'\n')
receipt = dict(passed=True, delivery=record(STAGE/'delivery_manifest.json'),
    external_mirror=record(OUT/'delivery_manifest.json'), review=record(OUT/'independent_review.json'),
    changed_files=len(files), test_scope_files=len(scope), tests=author['counts'],
    production_sources_unchanged=True, production_installed=False, physical_grasp_verified=False,
    motion_commands_sent=False, temperature_queried=False)
(OUT/'result.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2,sort_keys=True)+'\n')
print(json.dumps(receipt,ensure_ascii=False))
