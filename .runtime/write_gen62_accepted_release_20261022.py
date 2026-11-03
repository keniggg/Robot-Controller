"""Index an already accepted GEN62 deployment; never deploys or starts motion."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time

W=Path(__file__).resolve().parents[1]
M=Path('/media/zhuyupei/SanStand')
N=M/'实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z'
E=N/'gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1'

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_bytes())
def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--acceptance',type=Path,required=True)
    parser.add_argument('--acceptance-sha256',required=True)
    args=parser.parse_args()
    a=args.acceptance.resolve(strict=True)
    assert M.is_mount() and a==E/'runtime_reload_r2/acceptance/result.json'
    assert sha(a)==args.acceptance_sha256
    dep=read(a)
    assert dep['generation']=='GEN62' and dep['deployment_passed'] is True
    assert dep['replaced_count']==3 and dep['preserved_count']==14
    assert dep['automatic_attempt_authorized'] is False and dep['fresh_scene_and_dependency_gate_required_before_automatic_attempt'] is True
    bundle_path=Path(dep['bundle']);assert sha(bundle_path)==dep['bundle_sha256']
    bundle=read(bundle_path);assert bundle['deployment_ready'] is True
    evidence=dict(bundle['pins'])
    evidence.update({str(a):sha(a),str(bundle_path):sha(bundle_path),str(Path(__file__).resolve()):sha(__file__)})
    artifacts={key:read(ref['path']) for key,ref in bundle['frozen_evidence'].items()}
    assert all(sha(ref['path'])==ref['sha256'] for ref in bundle['frozen_evidence'].values())
    manifest,full,review=(artifacts[k] for k in ('manifest','full','review'))
    assert manifest['full_scope_ready'] and full['full_regression_passed'] and full['passed']
    assert full['source_unchanged'] and full['scope_complete'] and review['passed']
    assert full['summary']==dict(tests=manifest['total_case_count'],failures=0,errors=0,skipped=0)
    assert manifest['source_sha256']==full['source_sha256']==bundle['source_sha256']==dep['source_sha256']
    assert full['candidate_manifest_sha256']==bundle['frozen_evidence']['manifest']['sha256']
    assert review['production_source_sha256']=={p:bundle['source_sha256'][p] for p in bundle['production']}
    for p,h in bundle['source_sha256'].items():assert sha(W/p)==h,p
    full_dir=Path(bundle['frozen_evidence']['full']['path']).parent
    for name in ('pytest.xml','plan.json','final_process_receipt.json'):
        p=full_dir/name;evidence[str(p)]=sha(p)
    closed=read(full_dir/'final_process_receipt.json')
    assert closed['passed'] and closed['verification_passed'] and not closed['reported_residual_groups']
    assert closed['result_sha256']==bundle['frozen_evidence']['full']['sha256']
    transition_path=Path(dep['reload']);assert sha(transition_path)==dep['reload_sha256']
    transition=read(transition_path);assert transition['passed'] and transition['targets']==dep['targets']
    for p in (transition_path,Path(transition['construction'])):evidence[str(p)]=sha(p)
    assert evidence[transition['construction']]==transition['construction_sha256']
    for ref in (transition['remote_binding'],dep['remote_dependency']):
        p=Path(ref['path']);assert sha(p)==ref['sha256'];evidence[str(p)]=sha(p)
    health=read(dep['remote_dependency']['path']);assert health['passed'] and health['dependency_ready']
    for row in dep['source_process_proof'].values():
        p=Path(row['receipt']);assert sha(p)==row['sha256'];evidence[str(p)]=sha(p)
    for name in ('common','capture','receipt','entry','gate'):
        p=W/('.runtime/gen62_compute_attempt_'+name+'_20261022.py');evidence[str(p)]=sha(p)
    assert all(sha(p)==h for p,h in evidence.items())
    result=dict(generation='GEN62',passed=True,reviewed=True,installed=True,full_regression_passed=True,
        recorded_ns=time.time_ns(),source_count=len(bundle['source_sha256']),source_sha256=bundle['source_sha256'],
        test_summary=full['summary'],test_scope_files=full['test_scope_files'],
        candidate_manifest=bundle['frozen_evidence']['manifest']['path'],
        candidate_manifest_sha256=bundle['frozen_evidence']['manifest']['sha256'],
        actual_regression=bundle['frozen_evidence']['full']['path'],
        actual_regression_sha256=bundle['frozen_evidence']['full']['sha256'],
        acceptance=dict(path=str(a),sha256=sha(a)),evidence_sha256=evidence,
        changed_production_sources=bundle['production'],replaced_nodes=dep['targets'],preserved_nodes=dep['preserved'],
        gateway_compatibility=dep['gateway_compatibility'],source_process_proof=dep['source_process_proof'],
        provenance_scope='Three computation owners reloaded from verified source/cache/process inputs; fourteen instances retained. No blanket memory attestation.',
        physical_grasp_verified=False,scene_clearance_granted=False,automatic_attempt_authorized=False)
    out=N/'verification/gen62_release_manifest.json'
    assert out.parent.stat().st_dev==M.stat().st_dev
    raw=(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode()
    with out.open('xb') as f:f.write(raw);f.flush();os.fsync(f.fileno())
    assert out.read_bytes()==raw
    print(json.dumps(dict(path=str(out),sha256=sha(out),generation='GEN62')))

if __name__=='__main__':main()
