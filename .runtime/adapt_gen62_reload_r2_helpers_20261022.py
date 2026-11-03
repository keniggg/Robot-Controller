"""Finite r1-to-r2 artifact-path adaptation, never a runtime operation."""
import argparse
import difflib
import hashlib
import json
import os
from pathlib import Path

W = Path(__file__).resolve().parents[1]
F = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1')
BACKUP = W/'.runtime/gen62_reload_r1_helpers_backup_20261022'
OUT = F/'reload_r2_helpers_adaptation_r1'
NAMES = (
    'gen62_compute_attempt_common_20261022.py',
    'write_gen62_accepted_release_20261022.py',
    'sample_gen62_actual_task_resources_20261022.py',
    'read_gen62_post_attempt_20261022.py',
    'restore_gen62_after_handoff_20261022.py')
OLD = (
    '7dc87bf19147ebfa24d242203d79ff95542e2f809a0eae9f788c2ddd698146d7',
    '3d4f0e94b4ca81875c69374fc860e114665bfef8d1266a4f41d9a6cb4a5c4e6a',
    'd2c6d289a4e214fa061943b399a1d8a8790c7d02c0f1e51747ac99122c01a0eb',
    'fb65dde02e80636774ec677a21cbc595c82b41d396a27fb9548f36743456f7c0',
    'd5bff3a973d5f7e9d3efff6a3253540c04b778fdad67a2ccdf82806294b6bc0b')


def digest(raw): return hashlib.sha256(raw).hexdigest()
def sha(path): return digest(Path(path).read_bytes())


def json_new(path, value):
    raw = (json.dumps(value,ensure_ascii=False,indent=2)+'\n').encode()
    with path.open('xb') as stream:
        stream.write(raw);stream.flush();os.fsync(stream.fileno())
    assert path.read_bytes()==raw


def translated(baselines):
    new = {}
    for index,name in enumerate(NAMES[:4]):
        source = baselines[name].decode()
        expected = 2 if index==2 else 1
        assert source.count('runtime_reload_r1')==expected
        source = source.replace('runtime_reload_r1','runtime_reload_r2')
        if index==3:
            assert source.count(OLD[0])==1
            source = source.replace(OLD[0],digest(new[NAMES[0]]))
        new[name] = source.encode()
    source = baselines[NAMES[4]].decode()
    assert source.count(OLD[3])==1 and 'runtime_reload_r1' not in source
    new[NAMES[4]] = source.replace(OLD[3],digest(new[NAMES[3]])).encode()
    for name,raw in new.items(): compile(raw,str(W/'.runtime'/name),'exec')
    return new


def protected_paths():
    return [W/'.runtime'/name for name in (
        'gen62_compute_runtime_20261022.py','start_gen62_compute_20261022.py',
        'gen62_compute_attempt_capture_20261022.py','gen62_compute_attempt_receipt_20261022.py',
        'gen62_compute_attempt_entry_20261022.py','gen62_compute_attempt_gate_20261022.py')] + [
        W/'src/alicia_grasp_modes/src/alicia_grasp_modes/contact_station_task.py',
        W/'.runtime/gen62_contact_feedback_candidate_r1/src/alicia_grasp_modes/src/alicia_grasp_modes/contact_station_task.py',
        F/'runtime_prepare_r1/bundle.json',F/'runtime_reload_r1/failure.json']


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('prepare','apply'))
    args=parser.parse_args()
    mount=Path('/media/zhuyupei/SanStand')
    assert mount.is_mount() and F.stat().st_dev==mount.stat().st_dev
    assert not os.statvfs(mount).f_flag & os.ST_RDONLY
    if args.action=='prepare':
        assert not BACKUP.exists() and not OUT.exists()
        baselines={name:(W/'.runtime'/name).read_bytes() for name in NAMES}
        assert [digest(baselines[name]) for name in NAMES]==list(OLD)
        new=translated(baselines)
        BACKUP.mkdir();OUT.mkdir()
        rows=[]
        for name in NAMES:
            backup=BACKUP/name
            with backup.open('xb') as stream:
                stream.write(baselines[name]);stream.flush();os.fsync(stream.fileno())
            assert backup.read_bytes()==baselines[name]
            backup.chmod(0o444)
            patch=''.join(difflib.unified_diff(baselines[name].decode().splitlines(True),
                new[name].decode().splitlines(True),fromfile=str(backup),tofile=str(W/'.runtime'/name)))
            diffpath=OUT/(name+'.patch');diffpath.write_text(patch)
            rows.append(dict(path=str(W/'.runtime'/name),backup=str(backup),old_sha256=digest(baselines[name]),
                new_sha256=digest(new[name]),diff=dict(path=str(diffpath),sha256=sha(diffpath))))
        pins={str(p):sha(p) for p in protected_paths()}
        assert pins[str(F/'runtime_prepare_r1/bundle.json')]=='8fd950281af703d9c8de2021f6d2b7dfdfc51cabdc406144ffbf3ab9c2795ae3'
        assert pins[str(W/'.runtime/gen62_compute_runtime_20261022.py')]=='871b1b6b2ae048940beedd9d1a7c214fe2c9ff4201a981e1fd1f71efab6829de'
        plan=dict(kind='GEN62_exact_reload_r2_helper_path_plan',backup_verified=True,rows=rows,protected_sha256=pins,
            changes='Only exact GEN62 future runtime_reload_r1 artifact paths become runtime_reload_r2, then post COMMON_SHA and manual POST_SHA follow those bytes.',
            runtime_or_production_modified=False,ROS_called=False,deployment_run=False,
            r1_failure_evidence_preserved=True,control_revision=69,prior_generation=5,returned_generation=6,source_count=1103)
        json_new(OUT/'plan.json',plan)
        print(json.dumps(dict(plan=str(OUT/'plan.json'),sha256=sha(OUT/'plan.json'),rows=rows),ensure_ascii=False))
    else:
        plan=json.loads((OUT/'plan.json').read_bytes())
        assert not (OUT/'applied.json').exists()
        assert all(sha(p)==h for p,h in plan['protected_sha256'].items())
        baselines={name:(BACKUP/name).read_bytes() for name in NAMES}
        assert [digest(baselines[name]) for name in NAMES]==list(OLD)
        new=translated(baselines)
        assert all(sha(row['path'])==row['old_sha256'] and digest(new[Path(row['path']).name])==row['new_sha256'] for row in plan['rows'])
        for name in NAMES:
            path=W/'.runtime'/name
            with path.open('wb') as stream:
                stream.write(new[name]);stream.flush();os.fsync(stream.fileno())
            assert path.read_bytes()==new[name]
        assert all(sha(p)==h for p,h in plan['protected_sha256'].items())
        result=dict(kind='GEN62_exact_reload_r2_helper_path_applied',passed=True,
            plan=dict(path=str(OUT/'plan.json'),sha256=sha(OUT/'plan.json')),rows=plan['rows'],
            protected_inputs_unchanged=True,backup_verified=True,ROS_called=False,
            deployment_run=False,r1_failure_evidence_preserved=True,
            helper=dict(path=str(Path(__file__).resolve()),sha256=sha(__file__)))
        json_new(OUT/'applied.json',result)
        print(json.dumps(dict(result=str(OUT/'applied.json'),sha256=sha(OUT/'applied.json'),passed=True),ensure_ascii=False))


if __name__=='__main__':main()
