"""Seal completed production-worktree regression and independent GEN21 review."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET
from verify_gen21_combined_20261014 import PRODUCTION

W=Path(__file__).resolve().parents[1]
M=Path('/media/zhuyupei/SanStand')
N=M/'实验记录/Robot-Controller/ros_log/full_latest_gen18_restart_20261014T012216Z'
def read(p): return json.loads(Path(p).read_text())
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--combined',type=Path,required=True)
    ap.add_argument('--review',type=Path,required=True)
    ap.add_argument('--review-sha256',required=True)
    ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args()
    assert M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY
    assert N/'verification' in a.output.resolve().parents
    evidence={}
    def bind(p,expected=None):
        p=Path(p).resolve(strict=True);digest=sha(p)
        assert expected is None or digest==expected,str(p)
        assert str(p) not in evidence or evidence[str(p)]==digest
        evidence[str(p)]=digest;return p
    combined_path=bind(a.combined);c=read(combined_path)
    assert c['passed'] and c['source_unchanged'] and c['returncode']==0
    assert c['production_worktree_tested'] and not c['live_ros_contacted']
    s=c['summary']
    assert s['tests']>=6024 and all(s[k]==0 for k in ('failures','errors','skipped'))
    for name,key in [('pytest.log','log_sha256'),('pytest.xml','xml_sha256')]:
        bind(combined_path.parent/name,c[key])
    suites=ET.parse(combined_path.parent/'pytest.xml').getroot()
    assert {k:sum(int(t.get(k,0)) for t in suites.iter('testsuite')) for k in s}==s
    assert c['source_count']==len(c['source_sha256'])
    assert c['test_scope_files']==len(c['test_scope'])
    for name,digest in c['source_sha256'].items():
        assert not Path(name).is_absolute() and '..' not in Path(name).parts
        assert sha(W/name)==digest,name
    for key in ('baseline','patch'): bind(c[key],c[key+'_sha256'])
    bind(W/'.runtime/verify_gen21_combined_20261014.py',c['runner_sha256'])
    review_path=bind(a.review,a.review_sha256);review=read(review_path)
    assert review['reviewed'] and not review['blocking_findings']
    assert set(PRODUCTION)<=set(review['reviewed_source_sha256'])
    for name,digest in review['reviewed_source_sha256'].items():
        assert sha(W/name)==digest,name
        if name in c['source_sha256']: assert c['source_sha256'][name]==digest
    for name,digest in review['evidence_sha256'].items(): bind(name,digest)
    reload=review['reload_review'];assert not reload['blockers']
    bind(W/reload['path'],reload['sha256'])
    for name in ('prepare_gen20_continuation_receipt_20261014.py','continuous_grasp_authorization_20261012.py',
                 'run_reboot_realigned_grasp_20261010.py','capture_continuation_scene_20261012.py',
                 'read_gen7_yield_readiness_20261012.py'):
        bind(W/'.runtime'/name)
    bind(__file__)
    release=dict(schema_version=1,kind='gen21_pre_session_actuation_measurement_handoff',
        sealed_ns=time.time_ns(),passed=True,reviewed=True,installed=True,
        installed_semantics='Actual production worktree tested; three compute consumer reload pending; gateway and hardware preserved.',
        source_sha256=c['source_sha256'],source_count=c['source_count'],
        test_scope=c['test_scope'],test_scope_files=c['test_scope_files'],test_summary=s,
        actual_regression=str(combined_path),combined_review=str(review_path),evidence_sha256=evidence,
        runtime_deployment_pending=True,physical_grasp_verified=False,motion_commands_sent=False,temperature_queried=False)
    assert a.output.parent.stat().st_dev==M.stat().st_dev
    with a.output.open('x') as stream:
        json.dump(release,stream,ensure_ascii=False,indent=2);stream.write('\n');stream.flush();os.fsync(stream.fileno())
    print(json.dumps(dict(release=str(a.output),sha256=sha(a.output),summary=s),ensure_ascii=False))
if __name__=='__main__': main()
