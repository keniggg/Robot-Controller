from pathlib import Path
import ast,copy,hashlib,json,time,xml.etree.ElementTree as ET
W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade');V=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z/verification');C=W/'.runtime/gen17_remote_receipt_current_wait_r2_20261013';M=C/'delivery_manifest.json';OLD=W/'.runtime/gen17_remote_receipt_current_wait_20261013'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
assert sha(M)=='f241068ad731d81c7d0b00c9306cec706f72b0c7b3934f2f7813b43c69408482'
m=read(M);priorp=V/'gen17_remote_receipt_current_wait_independent_review/result.json';assert sha(priorp)=='22dd39a68fc1264c55341c957f519c5999aa5a5142bd56a02a46baecf91b2f4f';prior=read(priorp);evidence={}
def bind(p,h=None):
 p=Path(p);p=p if p.is_absolute() else W/p;h=h or sha(p);assert sha(p)==h,(p,'changed');assert str(p) not in evidence or evidence[str(p)]==h;evidence[str(p)]=h
for p,h in prior['evidence_sha256'].items():bind(p,h)
for p,h in m['proof_sha256'].items():bind(p,h)
bind(M);bind(priorp);r=read(m['baseline_release']);assert m['passed'] and r['passed']
assert m['baseline_source_sha256']==r['source_sha256'] and len(r['source_sha256'])==915
assert all(sha(W/p)==h for p,h in r['source_sha256'].items())
for row in m['files']:
 p=row['path'];bind(C/p,row['candidate_sha256'])
 assert (sha(W/p) if (W/p).exists() else None)==row['base_sha256']
assert m['test_scope']==prior['test_scope'] and len(m['test_scope'])==177
p='src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_remote.py'
old=(OLD/p).read_text();new=(C/p).read_text();needle='                    self._station_compute_soft_wait,\n';assert old.count(needle)==1
assert new==old.replace(needle,needle+'                    self._stream_generation,\n')
testpath='src/alicia_grasp_modes/tests/test_station_contact_remote_current_wait.py'
a=ast.parse((OLD/testpath).read_text());b=ast.parse((C/testpath).read_text());assert len(b.body)==len(a.body)+1 and ast.dump(ast.Module(body=b.body[:-1],type_ignores=[]))==ast.dump(a)
assert b.body[-1].name=='test_original_preserve_identity_generation_change_cancels_receipt_wait'
proofp=V/'gen17_remote_receipt_current_wait_r5/result.json';proof=read(proofp)
assert proof['passed'] and proof['source_unchanged'] and proof['actual_unchanged']
assert proof['candidate']['summary']==dict(tests=142,failures=0,errors=0,skipped=0)
for name,root in [('baseline',W),('prior_remote_generation',OLD),('candidate',C)]:
 q=proof[name];assert q['loaded']['remote']==str(root/p)
 for kind in ('log','xml'):bind(proofp.parent/name/('pytest.'+kind),q[kind+'_sha256'])
 cases=list(ET.parse(proofp.parent/name/'pytest.xml').getroot().iter('testcase'));assert len(cases)==q['summary']['tests']
 if name=='candidate':assert all(x.find('failure') is None and x.find('error') is None for x in cases)
 else:
  assert q['summary']==dict(tests=1,failures=1,errors=0,skipped=0)
  text=cases[0].find('failure').text
  assert ('station_compute_wait_registration_too_late' if name=='baseline' else 'assert done.wait(3)') in text
for rel,h in proof['candidate_sha256'].items():assert sha(C/rel)==h
bind(W/'.runtime/run_gen17_remote_receipt_current_wait_r2_20261013.py',m['runner_sha256']);bind(W/'.runtime/seal_gen17_remote_receipt_current_wait_r2_20261013.py',m['sealer_sha256']);bind(Path(__file__))
result=copy.deepcopy(prior);result.update(kind='GEN17_REMOTE_RECEIPT_CURRENT_WAIT_FINAL_INDEPENDENT_REVIEW',delivery_manifest=str(M),delivery_sha256=sha(M),evidence_sha256=dict(sorted(evidence.items())),focused_test_summary=proof['candidate']['summary'],previous_candidate_expected_failure_summary=proof['prior_remote_generation']['summary'],recorded_ns=time.time_ns(),previous_review=str(priorp),previous_review_sha256=sha(priorp))
result['findings']=[s.replace('Eleven new cases plus 130 related cases pass.','Twelve new cases plus 130 related cases pass.') for s in result['findings']]
result['findings'].append('Relative to the reviewed previous candidate, production adds only existing _stream_generation to wait signature; other production behavior is not otherwise changed. A real MODEL_RELOADED preserve_identity transition changes generation while keeping scope, session, reference and target identity; old candidate waits without cancelling, final candidate promptly raises the existing ModeAware StreamResultCancelled with processing-pending precedence. No receipt is issued.')
result['limitations']=[s for s in result['limitations'] if not s.startswith('Potential preserve-identity')]
result['limitations'].append('Other cancellation fields were not added speculatively: actual stop/shutdown already change streaming scope and ordinary epoch changes reset observe-return state. This fix addresses the reproduced preserved-identity generation transition.')
out=W/'.runtime/gen17_remote_receipt_current_wait_r2_review_draft.json';out.write_text(json.dumps(result,ensure_ascii=False,sort_keys=True,indent=2)+'\n');print(str(out),sha(out),'evidence',len(evidence))
