"""Review only the two-file old-test allowlist change and four affected gates."""
import argparse
from copy import deepcopy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys

W=Path(__file__).resolve().parents[1]
R=W/'.runtime'
V=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
INSTALLER=R/'install_and_seal_gen55_20261019.py'
OLD_SHA='bd6da64698f2bf56bf31df40ec0c60937bbc18a21dff25ba16ebde36d5693dc2'
NEW_SHA='c0a7c29793488c7af55f94a9c7ec11fedd790e8a45b3937960b1f678322c2aff'
COMMON=R/'gen55_idle_reload_common_20261019.py'
COMMON_SHA='1cefafce628d26d6775ba41e39b633551e890fbc41514042dcc9c803309724b0'
PRIOR=V/'gen55_runtime_helpers_independent_review_r3/result.json'
PRIOR_SHA='da47d37c8dd3f1575519e7e176983c1e8112def70b68f8c97aab60b3c8414d13'
OLD_MANIFEST=V/'gen55_final_collected_r2/candidate_manifest.json'
OLD_MANIFEST_SHA='5c781a1a47775007f2a10cbf5f088802c331588d79ec1e36fcb065777538bd91'
GEN52_TEST='src/alicia_flexible_grasp_supervisor/tests/test_gen52_contact_terminal_collision_screen.py'

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--manifest',type=Path,required=True)
    ap.add_argument('--manifest-sha256',required=True)
    ap.add_argument('--attempt',default='r1')
    args=ap.parse_args()
    assert args.attempt.isalnum() and os.path.ismount('/media/zhuyupei/SanStand')
    output=V/('gen55_installer_delta_review_'+args.attempt);output.mkdir()
    sys.dont_write_bytecode=True
    sys.path.insert(0,str(R))
    denied=[]
    def boundary(event,values):
        bad=event.startswith(('socket.','subprocess.')) or event in ('os.system','os.exec','os.posix_spawn','os.fork')
        if event=='open' and values and isinstance(values[0],(str,bytes,os.PathLike)):
            p=Path(os.fsdecode(values[0])).resolve()
            flags=values[2] if len(values)>2 and isinstance(values[2],int) else 0
            mode=values[1] if len(values)>1 and isinstance(values[1],str) else ''
            if flags&(os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC|os.O_APPEND) or any(c in mode for c in 'wax+'):
                bad=p!=output and output not in p.parents
            if str(p).startswith('/dev/') and str(p) not in ('/dev/null','/dev/random','/dev/urandom'):
                bad=True
        if bad:
            denied.append(dict(event=event,args=repr(values)[:200]))
            raise RuntimeError('review-only boundary denied '+event)
    sys.addaudithook(boundary)
    manifest=args.manifest.resolve(strict=True)
    dependencies={str(INSTALLER):NEW_SHA,str(COMMON):COMMON_SHA,str(PRIOR):PRIOR_SHA,
                  str(OLD_MANIFEST):OLD_MANIFEST_SHA,str(manifest):args.manifest_sha256}
    assert all(sha(p)==digest for p,digest in dependencies.items())
    prior=json.loads(PRIOR.read_bytes())
    assert prior['passed'] is True and prior['dependency_sha256'][str(INSTALLER)]==OLD_SHA
    text=INSTALLER.read_text()
    reverse=text.replace("OLD_TESTS = frozenset('src/alicia_flexible_grasp_supervisor/tests/'+name for name in\n    ('test_gen54_collision_contract.py', 'test_gen52_contact_terminal_collision_screen.py'))",
                         "OLD_TEST = 'src/alicia_flexible_grasp_supervisor/tests/test_gen54_collision_contract.py'")
    reverse=reverse.replace('changed == PRODUCTION|OLD_TESTS and added ==','changed == PRODUCTION|{OLD_TEST} and added ==')
    assert hashlib.sha256(reverse.encode()).hexdigest()==OLD_SHA, 'change exceeds reviewed allowlist adaptation'
    old_copy=output/'original_installer_before_second_test_allowlist.py'
    old_copy.write_text(reverse)
    assert sha(old_copy)==OLD_SHA
    old_manifest=json.loads(OLD_MANIFEST.read_bytes());candidate=json.loads(manifest.read_bytes())
    old_sources=old_manifest['source_sha256'];sources=candidate['source_sha256']
    assert set(old_sources)==set(sources)
    assert {p for p in old_sources if old_sources[p]!=sources[p]}=={GEN52_TEST}
    spec=importlib.util.spec_from_file_location('gen55_exact_installer_delta_review',INSTALLER)
    installer=importlib.util.module_from_spec(spec);spec.loader.exec_module(installer)
    checks=[]
    def check(name,value,accept):
        try:
            installer.check_delta(value)
        except RuntimeError as exc:
            accepted=False;reason=str(exc)
        else:
            accepted=True;reason=''
        checks.append(dict(name=name,accepted=accepted,expected_accept=accept,passed=accepted==accept,reason=reason))
    check('actual_candidate_disk_delta_reviewed',candidate,True)
    for mutation in ('extra_change','missing_baseline','missing_new_module'):
        trial=deepcopy(candidate)
        if mutation=='extra_change':
            baseline=json.loads(installer.BASE_RELEASE.read_bytes())['source_sha256']
            name=next(p for p in baseline if p not in installer.PRODUCTION|installer.OLD_TESTS)
            trial['source_sha256'][name]='0'*64
        if mutation=='missing_baseline':trial['source_sha256'].pop(next(iter(installer.PRODUCTION)))
        if mutation=='missing_new_module':trial['source_sha256'].pop(installer.NEW_MODULE)
        check('disk_delta_rejects_'+mutation,trial,False)
    assert all(c['passed'] for c in checks) and not denied
    assert all(sha(p)==digest for p,digest in dependencies.items())
    dependencies.update({str(old_copy):OLD_SHA,str(Path(__file__).resolve()):sha(__file__)})
    result=dict(kind='GEN55_installer_reviewed_old_test_delta',passed=True,reviewed=True,
        dependency_sha256=dependencies,source_sha256=sources,
        installer_old_sha256=OLD_SHA,installer_new_sha256=NEW_SHA,
        exact_source_delta_from_previous_manifest={GEN52_TEST:dict(before=old_sources[GEN52_TEST],after=sources[GEN52_TEST])},
        exact_installer_delta='Replace one old-test literal with two-file frozen set and require exact equality; other bytes unchanged',
        checks=checks,affected_prior_checks_revalidated=4,unaffected_prior_checks_reused=23,
        full_checks_retained=True,boundary_attempts=denied,test_bodies_reexecuted=False,
        full_regression_passed=False,installed=False,deployed=False,live_ROS_contacted=False,
        hardware_contacted=False,motion_commands_sent=False,temperature_queried=False,physical_grasp_verified=False)
    path=output/'result.json'
    with path.open('x') as f:
        json.dump(result,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n');f.flush();os.fsync(f.fileno())
    assert json.loads(path.read_bytes())==result
    print(json.dumps(dict(passed=True,result=str(path),sha256=sha(path)),ensure_ascii=False))

if __name__=='__main__':
    main()
