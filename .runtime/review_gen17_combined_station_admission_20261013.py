from pathlib import Path
import hashlib,json,time
W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
V=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z/verification')
C=W/'.runtime/gen17_combined_station_admission_20261013';M=C/'delivery_manifest.json'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
assert sha(M)=='7eff1fa245260cf8e782e2c34848aa74b1b7877cc0f238b1627d46d888174946'
m=read(M);evidence={}
def bind(p,h=None):
 p=Path(p);p=p if p.is_absolute() else W/p;h=h or sha(p)
 assert sha(p)==h,(p,'changed');assert str(p) not in evidence or evidence[str(p)]==h;evidence[str(p)]=h
reviews=[
 ('gen17_station_contact_current_wait_independent_review','0f230f94598cd5c9d4ad9bbb0d76756d80a7d9ba3a3fd1fd685a59acf3eab1aa'),
 ('gen17_station_join_clock_pair_independent_review','67760ade4618b3b37a97dc481f106d4b9b3039a10f478dcb0605b7fb8f40cfe4'),
 ('gen17_station_status_witness_atomic_independent_review','3b4fcf1f4f37a05c90d588bc11336451234775969888d965724c674bff88196b'),
 ('gen17_remote_receipt_current_wait_r2_independent_review','37e0634e9d15e4b9e2c40794b8ecc709dded32092c0cfc254209f99fb6f0e8e7'),
 ('gen17_frozen_station_retirement_independent_review','8d8c6aa86ed9822e8d2111aa87baae29fc01883cdf1da7452777de804b2781fe')]
scopes=set();review_map={};parents={};latest={};all_paths=set()
for name,h in reviews:
 p=V/name/'result.json';bind(p,h);r=read(p);assert r['passed'];review_map[str(p)]=h
 for path,digest in r['evidence_sha256'].items():bind(path,digest)
 scopes.update(r['test_scope'])
 path=r['delivery_manifest'];digest=r['delivery_sha256'];bind(path,digest)
 assert m['parent_sha256'][path]==digest
 parent=read(path);parents[path]=parent
 for f in parent['files']:
  src=Path(parent['candidate_root'])/f['path'];bind(src,f['candidate_sha256'])
  assert (sha(W/f['path']) if (W/f['path']).exists() else None)==f['base_sha256']
  latest[f['path']]=f;all_paths.add(f['path'])
assert len(parents)==5 and len(scopes)==181 and set(m['test_scope'])==scopes and len(m['test_scope'])==181
assert {r['path'] for r in m['files']}==all_paths and len(m['files'])==8
for row in m['files']:
 bind(C/row['path'],row['candidate_sha256'])
 assert row['candidate_sha256']==latest[row['path']]['candidate_sha256']
 assert row['base_sha256']==latest[row['path']]['base_sha256']
 expected={p for p,d in parents.items() if any(f['path']==row['path'] for f in d['files'])}
 assert set(row['parent_deliveries'])==expected
 assert (sha(W/row['path']) if (W/row['path']).exists() else None)==row['base_sha256']
for p in scopes:assert (C/p).is_file() or (W/p).is_file(),p
release=V/'gen16_success_first_release_manifest.json';bind(release,'97f9805ab90d1e6e6524583c26e52b7de9d1f7ae2defe4141d1195f6b9f13e3d')
rel=read(release);assert len(rel['source_sha256'])==915
assert all(sha(W/p)==h for p,h in rel['source_sha256'].items())
for p,h in m['parent_sha256'].items():bind(p,h)
prior_install=V/'gen16_success_first_integrated_install_r3/result.json'
assert read(prior_install)['source_sha256']==rel['source_sha256']
expected_installed=dict(rel['source_sha256'])
for row in m['files']:expected_installed[row['path']]=row['candidate_sha256']
assert len(expected_installed)==920
assert all(x['returncode']==0 and not x['stderr'] for x in m['merges']) and len(m['merges'])==3
bind(C/'delivery.patch',m['patch_sha256']);bind(M)
merge=V/'gen17_combined_merge_r1/result.json';bind(merge,'28b5f7bf40e882a2f225c7c21eca0efaaa07b9e49556eaf91a45b5fac7a4b521')
mr=read(merge);assert mr['passed'] and mr['delivery_sha256']==sha(M) and mr['files']==m['files'] and mr['test_scope']==m['test_scope']
helpers={
 str(W/'.runtime/seal_gen17_station_admission_release_20261013.py'):'37fbd0d5723bbb13eec1bec5d705332621dd3ddccca69fdc5ba6888ba0bf775d',
 str(W/'.runtime/finalize_gen17_deployment_20261013.py'):'7efa2dee103015230975d7a22c19d025015840a6304b2f0796cc88d2677e8cdc'}
for p,h in helpers.items():bind(p,h)
supp=V/'gen17_frozen_station_retirement_current_witness_r1'
bind(supp/'result.json','461bc1e92cd05d015bf367c4e52c53529cd6ad4a0c7a33068901c115ef9d5e74')
bind(supp/'supplemental_receipt.json','f6ccc36737c2070a7cc603378a0f7875ae7afa7d4704e32d6b58035df7bd48d8')
sp=read(supp/'result.json');sr=read(supp/'supplemental_receipt.json')
assert sp['passed'] and sr['passed'] and sr['production_unchanged'] and sp['actual_unchanged']
assert sp['summary']==dict(tests=4,failures=0,errors=0,skipped=0)
for typ in ('log','xml'):bind(supp/('pytest.'+typ),sp[typ+'_sha256'])
diag=W/'.runtime/gen17_frozen_station_retirement_r2_20261013'
for p,h in sr['production_sha256'].items():assert sha(diag/p)==sha(C/p)==h
bind(diag/'src/alicia_grasp_modes/tests/test_frozen_station_retirement_history.py',sr['supplemental_test_sha256'])
bind(W/'.runtime/run_gen17_frozen_station_retirement_current_witness_20261013.py',sp['runner_sha256'])
loaded=list((supp/'tmp').glob('test_held_latest_current*/loaded_sources.json'))
assert len(loaded)==4
for p in loaded:
 bind(p)
 for src,h in read(p).items():bind(src,h)
bind(W/'.runtime/merge_gen17_reviewed_candidates_20261013.py');bind(Path(__file__))
out=dict(schema=1,kind='GEN17_COMBINED_STATION_ADMISSION_INDEPENDENT_SOURCE_REVIEW',passed=True,
 delivery_manifest=str(M),delivery_sha256=sha(M),test_scope=sorted(scopes),test_scope_count=181,
 evidence_sha256=dict(sorted(evidence.items())),parent_reviews_sha256=review_map,metadata_helpers_sha256=helpers,
 actual_source_count=915,actual_sources_unchanged=True,expected_installed_source_count=920,
 combined_file_count=8,production_file_count=3,new_test_file_count=5,blocking_findings=[],
 review_method='Exact final byte equality to independently reviewed parent files, five-manifest/review recursive evidence checks and full actual-source baseline hash verification; no test rerun',
 findings=[
 'The combined task source exactly equals the reviewed status-atomic candidate, retaining original task PENDING recovery, narrow metadata pre-freeze retry, paired clocks and both atomic task entry points.',
 'The combined station remote and contact remote sources exactly equal the reviewed final retirement-history package, retaining both earlier remote wait-signature repairs and last-boundary freshness retry.',
 'All five new test files are byte-identical to their sealed parents. Eight delivered paths are exactly the five-parent union; 181 test paths are exactly the five independent-review scope union.',
 'All three recorded git merge-file operations completed without conflict, and the delivered patch hash, merge receipt, source bases and candidate hashes match. No additional implementation change is hidden in the combination.',
 'The actual 915-source GEN16 baseline remains unchanged; installing these three replacements and five additions should produce the expected 920-source set.',
 'The suspected history object/mask KeyError was ruled out by reassignment to the live reference before dereference, and four extra real callback cases passed on unchanged sealed production: identical latest-current messages preserve one retirement, while object/nonempty-mask contradictions revoke it. This diagnostic-only test copy is not a delivery replacement.'
 ],limitations=[
 'Passed means independent source/evidence review only. The combined 181-file installed regression has not been run by this review and remains required before deployment.',
 'Parent focused counts overlap and are not added into a fabricated independent total. Historical failures and conditional counterexamples remain in recursively bound receipts.',
 'No live ROS, recording, hardware action, new collision solution or successful physical grasp is claimed.',
 'Metadata helpers were statically reviewed for evidence identity/control preservation; deployment-time identity, double ACK and hardware-preservation checks must still run.'
 ],integrated_regression_performed=False,installed=False,deployed=False,live_ros_contacted=False,motion_commands_sent=False,temperature_queried=False,physical_grasp_verified=False,recorded_ns=time.time_ns())
dest=W/'.runtime/gen17_combined_station_admission_review_draft.json';dest.write_text(json.dumps(out,ensure_ascii=False,sort_keys=True,indent=2)+'\n');print(dest,sha(dest),'evidence',len(evidence))
