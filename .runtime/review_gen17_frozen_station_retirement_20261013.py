from pathlib import Path
import ast,copy,hashlib,json,time,xml.etree.ElementTree as ET
W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
V=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z/verification')
C=W/'.runtime/gen17_frozen_station_retirement_20261013';P=W/'.runtime/gen17_remote_receipt_current_wait_r2_20261013'
M=C/'delivery_manifest.json'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
assert sha(M)=='689f81ca233770ef9c4ded62a59225219b9dea1cbf4698a89f599d55445103aa'
m=read(M);evidence={}
def bind(p,h=None):
 p=Path(p);p=p if p.is_absolute() else W/p;h=h or sha(p)
 assert sha(p)==h,(p,'changed');assert str(p) not in evidence or evidence[str(p)]==h;evidence[str(p)]=h
rp=V/'gen17_remote_receipt_current_wait_r2_independent_review/result.json'
assert sha(rp)=='37e0634e9d15e4b9e2c40794b8ecc709dded32092c0cfc254209f99fb6f0e8e7'
prior=read(rp)
for p,h in prior['evidence_sha256'].items():bind(p,h)
bind(M);bind(rp)
for row in m['inherits']:bind(row['manifest'],row['sha256'])
release=V/'gen16_success_first_release_manifest.json';bind(release,'97f9805ab90d1e6e6524583c26e52b7de9d1f7ae2defe4141d1195f6b9f13e3d')
r=read(release);assert len(r['source_sha256'])==915 and r['passed'] and m['passed']
assert all(sha(W/p)==h for p,h in r['source_sha256'].items())
for row in m['files']:
 p=row['path'];bind(C/p,row['candidate_sha256']);assert (sha(W/p) if (W/p).exists() else None)==row['base_sha256']
assert len(m['files'])==3 and len(m['test_scope'])==len(set(m['test_scope']))==178
assert set(m['test_scope'])==set(prior['test_scope'])|{'src/alicia_grasp_modes/tests/test_frozen_station_retirement_history.py'}
def methods(p):return {n.name:n for n in ast.walk(ast.parse(Path(p).read_text())) if isinstance(n,ast.FunctionDef)}
base='src/alicia_grasp_modes/src/alicia_grasp_modes/contact_station_remote.py'
old=methods(W/base);new=methods(C/base)
body=copy.deepcopy(old['_contact_station_context'].body[:-2])
cond=body[-1].test;count=len(cond.values)
cond.values=[v for v in cond.values if not(isinstance(v,ast.Attribute) and v.attr=='_station_compute_pending')]
assert len(cond.values)==count-1
assert [ast.dump(n) for n in body]==[ast.dump(n) for n in new['_contact_station_live_context'].body]
assert [ast.dump(n) for n in old['_contact_station_context'].body[-2:]]==[ast.dump(n) for n in new['_contact_station_context'].body[-2:]]
assert set(new)-set(old)=={'_contact_station_live_context','_contact_station_pending_retirement_allowed'}
assert ast.dump(new['_contact_station_pending_retirement_allowed'].body[0].value)==ast.dump(ast.parse("getattr(self, '_station_contact_pending_retirement_allowed', None)",mode='eval').body)
for k in set(old)-{'_contact_station_context','_contact_station_request_cb'}:assert ast.dump(old[k])==ast.dump(new[k]),k
request=copy.deepcopy(new['_contact_station_request_cb']);removed=[0]
class RemoveHistory(ast.NodeTransformer):
 def visit_Assign(self,n):
  if len(n.targets)==1 and isinstance(n.targets[0],ast.Attribute) and n.targets[0].attr=='_station_contact_history':
   assert isinstance(n.value,ast.Constant) and n.value.value is None;removed[0]+=1;return None
  return self.generic_visit(n)
RemoveHistory().visit(request)
assert removed==[1] and ast.dump(request)==ast.dump(old['_contact_station_request_cb'])
remote='src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_remote.py'
oldr=methods(P/remote);newr=methods(C/remote)
allowed={'_reset_station_compute_wait','_station_gap_late_status','_station_compute_retire','_station_compute_tick_geometry_locked','_station_compute_status_cb','object_cb','mask_cb'}
assert set(newr)-set(oldr)=={'_station_contact_preserve_history','_station_contact_pending_retirement_allowed'}
assert {k for k in oldr if ast.dump(oldr[k])!=ast.dump(newr[k])}==allowed
# All earlier receipt/wait changes, audit verification and cancellation rules remain exactly inherited.
mro=ast.parse((W/'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py').read_text())
klass=next(n for n in mro.body if isinstance(n,ast.ClassDef) and n.name=='ModeAwareRemoteGrasp6DNode')
bases=[ast.dump(n) for n in klass.bases]
assert bases.index("Name(id='ContactStationRemoteMixin', ctx=Load())")<bases.index("Name(id='StationComputeWaitRemoteMixin', ctx=Load())")
proofp=Path(m['validation']['result']);bind(proofp,m['validation']['sha256']);proof=read(proofp)
assert proof['passed'] and proof['source_unchanged'] and proof['actual_unchanged'] and proof['actual_sources']==915
assert proof['summary']==dict(tests=309,failures=0,errors=0,skipped=0)
for p,h in proof['source_sha256'].items():assert sha(C/p)==h,(p,'candidate source changed')
for typ in ('log','xml'):bind(proofp.parent/('pytest.'+typ),proof[typ+'_sha256'])
cases=list(ET.parse(proofp.parent/'pytest.xml').getroot().iter('testcase'))
assert len(cases)==309 and all(x.find('failure') is None and x.find('error') is None for x in cases)
boundary=[x for x in cases if 'test_frozen_station_retirement_history' in x.attrib.get('classname','')]
assert len(boundary)==22
ep=Path(m['validation']['review_evidence']);bind(ep,m['validation']['review_evidence_sha256']);e=read(ep)
assert len(e['loaded_sources'])==22 and e['base_live_context_ast_identical_except_pending']
for row in e['loaded_sources']:
 bind(row['path'],row['sha256']);assert read(row['path'])==row['source_sha256']
 assert set(row['source_sha256'])=={str(C/base),str(C/remote)}
 for p,h in row['source_sha256'].items():bind(p,h)
bp=Path(m['baseline_defect']['result']);bind(bp,m['baseline_defect']['sha256']);b=read(bp)
assert b['reproduced'] and len(b['diagnoses'])==2
assert {x['tick'] for x in b['diagnoses']}=={'before_prepare','deferred'}
assert all(not x['remote_ack']['accepted'] and x['producer_ack']['accepted'] and x['physical_actions']==0 and x['before']['committed_audit'] and x['before']['task_admitted'] for x in b['diagnoses'])
for typ in ('log','xml'):bind(bp.parent/('pytest.'+typ),b[typ+'_sha256'])
for f in m['preserved_failures']:
 bind(f['path'],f['sha256']);d=read(f['path'])
 for typ in ('log','xml'):
  if typ+'_sha256' in d:bind(Path(f['path']).parent/('pytest.'+typ),d[typ+'_sha256'])
assert read(V/'gen17_frozen_station_retirement_r1/result.json')['summary']==dict(tests=22,failures=9,errors=0,skipped=0)
bind(V/'gen17_frozen_station_retirement_delivery/result.json','9159503cca37e9e5ba8ad8635ab693fac1d319b3a535bef384c9010647b1f409')
bind(W/'.runtime/run_gen17_frozen_station_retirement_20261013.py',proof['runner_sha256']);bind(Path(__file__))
out=dict(schema=1,kind='GEN17_FROZEN_STATION_RETIREMENT_HISTORY_INDEPENDENT_REVIEW',passed=True,
 delivery_manifest=str(M),delivery_sha256=sha(M),test_scope=m['test_scope'],test_scope_count=178,
 evidence_sha256=dict(sorted(evidence.items())),inherits_review=str(rp),inherits_review_sha256=sha(rp),
 blocking_findings=[],actual_source_count=915,actual_sources_unchanged=True,focused_test_summary=proof['summary'],
 review_method='Independent static semantic/MRO/AST review and exact saved evidence/source verification; no rerun',
 findings=[
 'Both original scheduling counterexamples genuinely freeze a plan and atomically commit its audit before a pure clock pending state; the producer accepts prepare while the remote rejects. They are offline conditional proofs, not a claim that R5 reached this later boundary.',
 'Only exact published and atomically committed audit facts are retained after MOVE exits planning scope. The marker has a copied historical contract and creates no request, ACK, frozen authority or geometry registration. Reference is cleared, pending remains true and the work certificate remains false.',
 'Actual parsed prepare and retire still pass every original live owner/phase/session/manual/deadline condition, exact audited plan/receipt matching, original plan validation and both participant ACKs. The earlier private-work receipt/wait and cancellation code remains AST-identical to the reviewed remote parent.',
 'Base ContactStationRemote precedes Station in the real MRO. The corrected optional hook uses a distinct name so absence still rejects pending and the actual Station implementation is reached; all 22 new cases bind actual loaded source paths.',
 'Reset, real retire and successful handshake clear the historical marker. Real target loss/first loss, false object, empty mask and bound historical contradictions revoke history even after reference is disarmed; existing old-source/tombstone filters remain ahead of invalidation.',
 'Two positive schedules reach the original inert action sentinel exactly once with one frozen publication. Sixteen faults and four prepare-to-retire changes reject with zero action. Replayed handshakes cannot execute again.'
 ],limitations=[
 'Tests use synthetic RGBD, supplied recorded geometry/audit fixture, inert transport and an action sentinel. They do not recompute a new collision solution or establish physical success.',
 'No installed combined regression, live ROS or physical action was performed by this review.',
 'Initial 13/22 test result with MRO dispatch error and fixture assertions, and r3 missing-test collection failure, remain preserved. Final r4 is 309/309; earlier failures are not relabeled successful.',
 'Geometry/identity cache freshness is not renewed or repurposed as an execution gate after planning is disarmed; the original owner, phase/session, exact request and downstream execution checks remain mandatory.'
 ],integrated_regression_performed=False,installed=False,deployed=False,live_ros_contacted=False,motion_commands_sent=False,temperature_queried=False,physical_grasp_verified=False,recorded_ns=time.time_ns())
dest=W/'.runtime/gen17_frozen_station_retirement_review_draft.json';dest.write_text(json.dumps(out,ensure_ascii=False,sort_keys=True,indent=2)+'\n');print(dest,sha(dest),'evidence',len(evidence))
