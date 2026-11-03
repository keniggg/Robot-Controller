#!/usr/bin/env python3
"""Offline combination verification; only emits local review drafts by default."""
from pathlib import Path
import hashlib,json,time
W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
V=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z/verification')
C=W/'.runtime/gen16_combined_success_first_r3_20261013'
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p): return json.loads(Path(p).read_text())
def absolute(p):
 p=Path(p);return p.resolve() if p.is_absolute() else (W/p).resolve()
D=C/'delta_manifest.json';F=C/'delivery_manifest_r3.json'
assert sha(D)=='c715001d8f4414d822cbbce8855279bd5e38cde52f98b9c29d2908666ce5d2c9'
assert sha(F)=='d048d3a86f475e3bc83ccb74ae5462efb8ecf69a7cf4a1c35eebb7f093cecec5'
d,f=read(D),read(F)
reviews=[V/x/'result.json' for x in ['gen16_combined_independent_review','gen16_height_recipe_policy_test_independent_review','gen16_worker_completion_fixture_independent_review']]
original=W/'.runtime/gen16_combined_success_first_20261013/delivery_manifest_r2.json';o=read(original)
installp=V/'gen16_success_first_integrated_install/result.json';inst=read(installp)
assert inst['passed'] and len(inst['source_sha256'])==915
assert all(sha(W/p)==h for p,h in inst['source_sha256'].items()),'actual pre-install sources changed'
assert d['passed'] and f['passed'] and len(d['files'])==2 and len(f['files'])==21
assert d['production_modified'] is False and f['production_delta_from_r2'] is False
assert f['files']==o['files']+d['files']
assert len({x['path'] for x in f['files']})==21
scope=set()
for p in reviews:
 r=read(p);assert r['passed'];assert len(r['test_scope'])==len(set(r['test_scope']));scope.update(r['test_scope'])
assert len(scope)==176 and scope==set(d['test_scope'])==set(f['test_scope'])==set(inst['test_scope'])
assert len(d['test_scope'])==len(f['test_scope'])==176
for row in o['files']:
 p=row['path'];h=row['candidate_sha256'];assert sha(C/p)==h==sha(Path(o['candidate_root'])/p)==sha(W/p)
for rp,row in zip(reviews[1:],d['files']):
 r=read(rp);mp=Path(r['delivery_manifest']);m=read(mp);assert sha(mp)==r['delivery_sha256'];assert len(m['files'])==1
 old=m['files'][0];p=row['path'];assert p==(old.get('path') or old.get('relative_path'))
 h=old.get('candidate_sha256') or old['sha256'];base=old.get('base_sha256') or old['baseline_sha256']
 assert row['candidate_sha256']==h and row['base_sha256']==base and '/tests/' in p
 author=Path(m.get('candidate_root') or mp.parent)
 assert sha(C/p)==sha(author/p)==h and sha(W/p)==base
# Recursively merge review evidence only. Referenced receipts are retained, and
# their evidence maps recursively checked; no source_sha256 history is flattened.
evidence={};visited=set()
def add(p,h=None):
 p=absolute(p);key=str(p);h=h or sha(p)
 assert key not in evidence or evidence[key]==h,('conflicting evidence',key)
 assert sha(p)==h,('evidence changed',key)
 evidence[key]=h
 if key in visited:return
 visited.add(key)
 if p.suffix=='.json':
  j=read(p)
  if isinstance(j,dict):
   for q,expected in j.get('evidence_sha256',{}).items(): add(q,expected)
for manifest in [d,f]:
 for p,h in manifest['parent_sha256'].items():add(p,h)
for p in reviews+[D,F,original,installp,W/'.runtime/assemble_gen16_test_corrections_20261013.py',Path(__file__)]:add(p)
# Bind final candidate bytes as well as immutable parent candidates.
for row in f['files']:add(C/row['path'],row['candidate_sha256'])
firstp=V/'gen16_success_first_actual_integrated/result.json';first=read(firstp)
assert not first['passed']
common=dict(schema=1,passed=True,recorded_ns=time.time_ns(),test_scope=sorted(scope),test_scope_count=176,evidence_sha256=dict(sorted(evidence.items())),review_method='Independent exact-byte composition and recursively verified immutable evidence; no tests or ROS rerun',actual_source_count=915,actual_sources_unchanged=True,blocking_findings=[],integrated_regression_performed=False,integrated_regression_passed=False,retry_regression_required=True,first_integrated_failure_preserved={'path':str(firstp),'sha256':sha(firstp),'passed':False,'tests':5636,'failures':5,'errors':0,'skipped':0},live_ros_contacted=False,motion_commands_sent=False,temperature_queried=False,physical_grasp_verified=False,limitations=['This is source/evidence review, not a passing full 176-file regression or physical grasp result.','Prior full regression had five failures; original failed result remains bound and unchanged.','Focused test evidence and its fixture limitations are inherited unchanged from both independent reviews.'],findings=['Original nineteen-file GEN16 bundle is preserved byte-for-byte; no production delta.','The two added test-file replacements exactly equal their separately sealed and reviewed author deliveries.','Test scope is the unique union of the original combined review and both focused reviews: 176 files.','All 915 actual pre-install source hashes match the current installation receipt.','All parent review evidence maps are recursively merged and hash-verified without changing their historical statuses.'])
for name,manifest,kind in [('gen16_test_delta_independent_review',D,'GEN16_TEST_ONLY_DELTA_INDEPENDENT_REVIEW'),('gen16_complete_r3_independent_review',F,'GEN16_COMPLETE_R3_INDEPENDENT_REVIEW')]:
 result=dict(common,kind=kind,delivery_manifest=str(manifest),delivery_sha256=sha(manifest),files=read(manifest)['files'],production_modified=False,production_delta_from_r2=False,test_only_delta=True)
 out=W/'.runtime'/(name+'_draft.json');out.write_text(json.dumps(result,ensure_ascii=False,indent=2,sort_keys=True)+'\n')
 print(json.dumps({'draft':str(out),'sha256':sha(out),'external_target':str(V/name/'result.json'),'evidence_count':len(evidence),'scope_count':len(scope)},ensure_ascii=False))
