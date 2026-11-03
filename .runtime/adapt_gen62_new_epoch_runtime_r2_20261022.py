"""Exact offline new-epoch runtime copy and four dependent helper bindings."""
import argparse
import difflib
import hashlib
import json
import os
from pathlib import Path

W=Path(__file__).resolve().parents[1]
G=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1')
F=G/'contact_feedback_fix_r1'
OUT=F/'runtime_epoch_r2_implementation_r1'
BACKUP=W/'.runtime/gen62_epoch_r2_helpers_backup_20261022'
BASE_RUNTIME='gen62_compute_runtime_20261022.py'
NEW_RUNTIME='gen62_compute_runtime_r2_20261022.py'
NAMES=(BASE_RUNTIME,'gen62_compute_attempt_common_20261022.py','sample_gen62_actual_task_resources_20261022.py',
       'read_gen62_post_attempt_20261022.py','restore_gen62_after_handoff_20261022.py')
BEFORE=(
    '871b1b6b2ae048940beedd9d1a7c214fe2c9ff4201a981e1fd1f71efab6829de',
    '9fc98aec9bd8b76daf78b342b2f986a3f75a7c703d3ae2a99a6cfe333e6bf279',
    'd702ce022a87d5f7ef5458e80654791af1f59432ac36cb3c3f02c27e86ddf10c',
    '258cc8470e3c268ea7b96b59aded9196cd835bc126af47166d019833beb4be5b',
    '9863db46a49f21da911bd451069e8ba37c875d11628e7ef8ddadc7aff6dc7e4b')
DESIGN=G/'user_resume_r1/readiness_r2/result.json'
DESIGN_SHA='dcff1906efceebfd8f49a5104b1a093356e6b4e9f4d9e287dbd5464f143aa580'
EPOCH=1792668378941525117
SELECTION=dict(mode='unknown',strategy='observe_return',generation=5,stamp_ns=1792661288399203300)


def digest(raw):return hashlib.sha256(raw).hexdigest()
def sha(path):return digest(Path(path).read_bytes())
def read(path):return json.loads(Path(path).read_bytes())


def replace_once(source,old,new):
    assert source.count(old)==1,(old,source.count(old))
    return source.replace(old,new)


def transformed(before):
    new={}
    s=before[BASE_RUNTIME].decode()
    s=replace_once(s,"DESIGN = G/'user_resume_r1/readiness_r1/result.json'","DESIGN = G/'user_resume_r1/readiness_r2/result.json'")
    s=replace_once(s,"DESIGN_SHA = '876214ba8befe29b1083fcaee62876127f30e1a0e5fcbf179b728fd602cd2af8'","DESIGN_SHA = '"+DESIGN_SHA+"'")
    s=replace_once(s,'EPOCH = 1792661888995982113','EPOCH = '+str(EPOCH))
    s=replace_once(s,
        "        str(RESUME):RESUME_SHA,str(Path(__file__).resolve()):sha(__file__),str(STARTER):sha(STARTER)})",
        "        str(RESUME):RESUME_SHA,str(Path(__file__).resolve()):sha(__file__),str(STARTER):sha(STARTER),\n"
        "        str(W/'.runtime/gen62_compute_runtime_20261022.py'):'"+BEFORE[0]+"'})")
    new[NEW_RUNTIME]=s.encode()
    s=replace_once(before[NAMES[1]].decode(),"RUNTIME = W / '.runtime/gen62_compute_runtime_20261022.py'",
        "RUNTIME = W / '.runtime/gen62_compute_runtime_r2_20261022.py'")
    new[NAMES[1]]=s.encode()
    s=replace_once(before[NAMES[2]].decode(),'runtime_prepare_r1/bundle.json','runtime_prepare_r2/bundle.json')
    new[NAMES[2]]=s.encode()
    s=replace_once(before[NAMES[3]].decode(),"RUNTIME = W / '.runtime/gen62_compute_runtime_20261022.py'",
        "RUNTIME = W / '.runtime/gen62_compute_runtime_r2_20261022.py'")
    s=replace_once(s,BEFORE[0],digest(new[NEW_RUNTIME]))
    s=replace_once(s,BEFORE[1],digest(new[NAMES[1]]))
    new[NAMES[3]]=s.encode()
    s=replace_once(before[NAMES[4]].decode(),BEFORE[3],digest(new[NAMES[3]]))
    new[NAMES[4]]=s.encode()
    for name,raw in new.items():compile(raw,str(W/'.runtime'/name),'exec')
    return new


def protected_paths():
    return [W/'.runtime'/name for name in (BASE_RUNTIME,'gen62_resume_runtime_20261022.py',
        'start_gen62_compute_20261022.py','write_gen62_accepted_release_20261022.py',
        'gen62_compute_attempt_entry_20261022.py','gen62_compute_attempt_capture_20261022.py',
        'gen62_compute_attempt_receipt_20261022.py','gen62_compute_attempt_gate_20261022.py')] + [
        W/'src/alicia_grasp_modes/src/alicia_grasp_modes/contact_station_task.py',
        W/'.runtime/gen62_contact_feedback_candidate_r1/src/alicia_grasp_modes/src/alicia_grasp_modes/contact_station_task.py',
        F/'runtime_prepare_r1/bundle.json',F/'runtime_reload_r1/failure.json',DESIGN]


def save(path,value):
    raw=(json.dumps(value,ensure_ascii=False,indent=2)+'\n').encode()
    with path.open('xb') as stream:
        stream.write(raw);stream.flush();os.fsync(stream.fileno())
    assert path.read_bytes()==raw


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('prepare','apply'));args=parser.parse_args()
    mount=Path('/media/zhuyupei/SanStand')
    assert mount.is_mount() and F.stat().st_dev==mount.stat().st_dev and not os.statvfs(mount).f_flag & os.ST_RDONLY
    assert sha(DESIGN)==DESIGN_SHA
    readiness=read(DESIGN)
    assert readiness['passed'] is True and readiness['epoch_ns']==EPOCH and readiness['selection']==SELECTION
    assert readiness['manual_mode'] is True and readiness['task_active'] is False and readiness['actions_active'] is False
    assert readiness['actuation_status'].startswith('CONFIRMED:') and readiness['sample_count']==20
    assert readiness['before']['node_identities']==readiness['after']['node_identities'] and len(readiness['before']['node_identities'])==17
    if args.action=='prepare':
        assert not OUT.exists() and not BACKUP.exists() and not (W/'.runtime'/NEW_RUNTIME).exists()
        before={name:(W/'.runtime'/name).read_bytes() for name in NAMES}
        assert [digest(before[name]) for name in NAMES]==list(BEFORE)
        new=transformed(before)
        # Construct all checks and hashes before creating any output.
        protected={str(p):sha(p) for p in protected_paths()}
        assert protected[str(W/'.runtime/gen62_resume_runtime_20261022.py')]=='d9f1ac014f09b5743a7bddedaa8219b386161d41bd5c3eacebf30afcd0ee3127'
        assert protected[str(W/'.runtime/start_gen62_compute_20261022.py')]=='e4ad349339594bb0ffeaa57511b70c5c8462ab8df3d6aec866a588ab2e920460'
        OUT.mkdir();BACKUP.mkdir();rows=[]
        for name in NAMES:
            backup=BACKUP/name
            with backup.open('xb') as stream:
                stream.write(before[name]);stream.flush();os.fsync(stream.fileno())
            assert backup.read_bytes()==before[name];backup.chmod(0o444)
            target=NEW_RUNTIME if name==BASE_RUNTIME else name
            patch=''.join(difflib.unified_diff(before[name].decode().splitlines(True),new[target].decode().splitlines(True),
                fromfile=str(backup),tofile=str(W/'.runtime'/target)))
            patchpath=OUT/(target+'.patch');patchpath.write_text(patch)
            rows.append(dict(path=str(W/'.runtime'/target),baseline=str(W/'.runtime'/name),backup=str(backup),
                new_file=name==BASE_RUNTIME,old_sha256=digest(before[name]),new_sha256=digest(new[target]),
                diff=dict(path=str(patchpath),sha256=sha(patchpath))))
        value=dict(kind='GEN62_new_epoch_r2_exact_helper_plan',readiness=dict(path=str(DESIGN),sha256=DESIGN_SHA),
            current_epoch_ns=EPOCH,selection=SELECTION,control_revision=69,rows=rows,protected_sha256=protected,
            old_runtime_pin_reason='Unchanged starter e4ad imports old runtime871b for cache/storage/source-loader/save primitives.',
            prepare_output=str(F/'runtime_prepare_r2'),reload_output=str(F/'runtime_reload_r2'),
            ROS_called=False,prepare_run=False,deployment_run=False)
        save(OUT/'plan.json',value)
        print(json.dumps(dict(plan=str(OUT/'plan.json'),sha256=sha(OUT/'plan.json'),rows=rows),ensure_ascii=False))
    else:
        plan=read(OUT/'plan.json');assert not (OUT/'applied.json').exists()
        assert all(sha(p)==h for p,h in plan['protected_sha256'].items())
        before={name:(BACKUP/name).read_bytes() for name in NAMES}
        assert [digest(before[name]) for name in NAMES]==list(BEFORE)
        new=transformed(before)
        for row in plan['rows']:
            assert sha(row['baseline'])==row['old_sha256']
            assert digest(new[Path(row['path']).name])==row['new_sha256']
            if row['new_file']:assert not Path(row['path']).exists()
        for row in plan['rows']:
            path=Path(row['path'])
            with path.open('xb' if row['new_file'] else 'wb') as stream:
                stream.write(new[path.name]);stream.flush();os.fsync(stream.fileno())
            assert sha(path)==row['new_sha256']
        assert all(sha(p)==h for p,h in plan['protected_sha256'].items())
        value=dict(kind='GEN62_new_epoch_r2_exact_helper_applied',passed=True,rows=plan['rows'],
            plan=dict(path=str(OUT/'plan.json'),sha256=sha(OUT/'plan.json')),protected_inputs_unchanged=True,
            original_runtime_871b_unchanged=True,old_resume_and_starter_unchanged=True,
            ROS_called=False,prepare_run=False,deployment_run=False,production_modified=False,
            helper=dict(path=str(Path(__file__).resolve()),sha256=sha(__file__)))
        save(OUT/'applied.json',value)
        print(json.dumps(dict(result=str(OUT/'applied.json'),sha256=sha(OUT/'applied.json'),passed=True),ensure_ascii=False))


if __name__=='__main__':main()
