#!/usr/bin/env python3
"""Execute this one frozen 15-run scope with two independent existing Cleanup stores."""
from pathlib import Path
import concurrent.futures, datetime, errno, fcntl, hashlib, importlib.util, json, os
import resource, shutil, sqlite3, stat, sys, time

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
R = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/maintenance/minimal_trace_cleanup_20261023T024945Z_pytest15_r1')
PLAN_SHA = '0207acc2e11f288bcb303ee31971a96921268bdf8cb61b02560c96d6d7cf22c7'
SCOPE_SHA = '266f0e905b21405013d7ad1d2c25635f661571353714426977e9adbd3f7d8d4d'
spec = importlib.util.spec_from_file_location('minimal_trace', W/'tools/minimal_experiment_trace.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)

def read(p):
    return json.loads(p.read_text())

def free():
    s = os.statvfs(m.ROOT)
    return s.f_bavail*s.f_frsize

def verify_plan():
    assert m.sha_file(R/'plan.json') == PLAN_SHA
    p = read(R/'plan.json')
    assert p['source_scope_sha256'] == SCOPE_SHA and m.sha_file(Path(p['source_scope_path'])) == SCOPE_SHA
    assert len(p['reviewed_runs']) == 15 and len(p['reviewed_pytest_roots']) == 106
    assert len(p['text_files']) == 28509 and len(p['directories']) == 23831 and not p['raw_files']
    assert not any('gen54_full_parallel_r1' in s for s in p['reviewed_runs'])
    assert len({r['path'] for r in p['text_files']}) == 28509
    for r in p['text_files']:
        assert r['remove_original'] is True
        assert any(r['path'].startswith(s+'/') for s in p['reviewed_pytest_roots'])
    return p

def parent_evidence_check(plan):
    for row in plan['retained_parent_evidence']:
        p = m.ROOT/row['path']
        assert m.identity(p.stat()) == row['identity'] and m.sha_file(p) == row['sha256'], str(p)

def prepare():
    plan = verify_plan()
    owned = [[], []]
    weights = [0, 0]
    counts = {r:sum(v['path'].startswith(r+'/') for v in plan['text_files']) for r in plan['reviewed_runs']}
    for run in sorted(plan['reviewed_runs'], key=lambda r:(-counts[r], r)):
        worker = min(range(2), key=lambda i:weights[i])
        owned[worker].append(run)
        weights[worker] += counts[run]
    pilot_paths = {r['path'] for r in plan['text_files'][:100]}
    workers = []
    for i, runs in enumerate(owned):
        child = R/('owned_store_'+str(i))
        child.mkdir()
        inside = lambda r:any(r['path'].startswith(run+'/') for run in runs)
        part = dict(plan)
        part.update(text_files=[r for r in plan['text_files'] if inside(r)],
                    directories=[r for r in plan['directories'] if inside(r)],
                    reviewed_runs=runs,
                    reviewed_pytest_roots=[p for p in plan['reviewed_pytest_roots'] if any(p.startswith(r+'/') for r in runs)],
                    closure_evidence=[r for r in plan['closure_evidence'] if r['run'] in runs],
                    parent_plan=str(R/'plan.json'), parent_plan_sha256=PLAN_SHA,
                    ownership_note='Disjoint complete runs; each original file and directory has exactly one owner.')
        m.atomic(child/'plan.json', part)
        paths = {r['path'] for r in part['text_files']}
        if pilot_paths <= paths:
            assert not (child/'trace.sqlite3').exists()
            shutil.copyfile(R/'trace.sqlite3', child/'trace.sqlite3')
            with (child/'trace.sqlite3').open('rb') as f:os.fsync(f.fileno())
            m.fsync_dir(child)
            assert m.sha_file(R/'trace.sqlite3') == m.sha_file(child/'trace.sqlite3')
        workers.append(dict(run=str(child), plan_sha256=m.sha_file(child/'plan.json'),
                            owned_runs=runs, files=len(paths), directories=len(part['directories']),
                            logical_bytes=sum(v['identity'][2] for v in part['text_files'])))
    sets = [{r['path'] for r in read(Path(w['run'])/'plan.json')['text_files']} for w in workers]
    dirsets = [{r['path'] for r in read(Path(w['run'])/'plan.json')['directories']} for w in workers]
    assert not sets[0]&sets[1] and sets[0]|sets[1] == {r['path'] for r in plan['text_files']}
    assert not dirsets[0]&dirsets[1] and dirsets[0]|dirsets[1] == {r['path'] for r in plan['directories']}
    approval = dict(schema=1, approved=True, approval_source='Explicit root independent finite-scope approval; no further reply required',
                    approved_plan_sha256=PLAN_SHA, approved_source_scope_sha256=SCOPE_SHA,
                    executor_sha256=m.sha_file(Path(__file__)), source_tool_sha256=m.sha_file(W/'tools/minimal_experiment_trace.py'),
                    workers=workers, parallel_workers=2, all_paths_once=True,
                    excluded_interrupted_run='gen54_full_parallel_r1', created_ns=time.time_ns())
    m.atomic(R/'root_execution_approval.json', approval)
    print(json.dumps(approval, ensure_ascii=False), flush=True)

def directory_cleanup(C, plan, child):
    deleted = []
    skipped = []
    with (child/'directory_receipts.jsonl').open('x', encoding='utf-8') as journal:
        def append(value):
            journal.write(json.dumps(value, ensure_ascii=False)+'\n')
            journal.flush()
            os.fsync(journal.fileno())
        for i, row in enumerate(plan['directories']):
            C.mounted()
            p = C.path(row['path'])
            try:
                s = p.lstat()
                assert stat.S_ISDIR(s.st_mode) and not p.is_symlink()
                assert [s.st_dev,s.st_ino] == row['identity'][:2]
                assert not any(p.iterdir()), 'not empty; new or protected content remains'
                receipt = dict(path=row['path'], original_identity=row['identity'],
                               allocated_bytes=s.st_blocks*512, kind='empty_directory',
                               metadata_recoverable_from=str(child/'plan.json'), intent_ns=time.time_ns())
                append(dict(state='intent', **receipt))
                p.rmdir()
                m.fsync_dir(p.parent)
                receipt['deleted_ns'] = time.time_ns()
                append(dict(state='deleted', **receipt))
                deleted.append(receipt)
            except (OSError, AssertionError, ValueError) as error:
                receipt = dict(path=row['path'], state='kept', reason=repr(error), recorded_ns=time.time_ns())
                append(receipt)
                skipped.append(receipt)
            if (i+1)%1000 == 0:
                m.atomic(child/'directory_progress.json', dict(checked=i+1, total=len(plan['directories']),
                         deleted=len(deleted), skipped=len(skipped), updated_ns=time.time_ns()))
    return deleted, skipped

def run_worker(owner):
    child = Path(owner['run'])
    assert m.sha_file(child/'plan.json') == owner['plan_sha256']
    plan = read(child/'plan.json')
    assert plan['parent_plan_sha256'] == PLAN_SHA
    start = time.monotonic()
    m.atomic(child/'worker_started.json', dict(started_ns=time.time_ns(), pid=os.getpid(),
             source_tool_sha256=m.sha_file(W/'tools/minimal_experiment_trace.py')))
    with (child/'execution.log').open('x', buffering=1) as log:
        os.dup2(log.fileno(),1)
        os.dup2(log.fileno(),2)
        C = m.Cleanup(m.ROOT, child)
        before = free()
        file_receipt = C.run_cleanup()
        m.atomic(child/'file_cleanup_receipt.json', file_receipt)
        # Use the existing mounted/plan/path guards for empty-directory removal.
        dirs, skips = directory_cleanup(C, plan, child)
        con = sqlite3.connect('file:'+str(child/'trace.sqlite3')+'?mode=ro', uri=True)
        states = dict(con.execute('select state,count(*) from deletions group by state'))
        assert not states.get('intent',0)
        recovery = []
        for path, state, receipt in con.execute('select path,state,receipt from deletions order by path'):
            assert state == 'deleted'
            receipt = json.loads(receipt)
            stored = con.execute('select sha,size from files where path=?',(path,)).fetchone()
            assert stored and stored[0] == receipt['original_sha256'] and stored[1] == receipt['original_identity'][2]
            assert receipt['full_raw_recoverable'] is True
            recovery.append(dict(path=path, sha256=stored[0], size_bytes=stored[1], store=str(child/'trace.sqlite3')))
        con.close()
        m.atomic(child/'recovery_index.json', dict(files=recovery, directory_metadata=str(child/'plan.json'),
                 fully_lossless=True, archived_readback_before_and_after_file_deletions=True))
        usage = resource.getrusage(resource.RUSAGE_SELF)
        final = dict(state='complete' if not skips and not file_receipt['skipped'] else 'complete_with_protected_skips',
                     owned_runs=owner['owned_runs'], files_deleted=len(recovery), directories_deleted=len(dirs),
                     file_allocated_deleted_bytes=file_receipt['deleted_allocated_bytes'],
                     directory_allocated_deleted_bytes=sum(d['allocated_bytes'] for d in dirs),
                     source_deleted_bytes=sum(d['size_bytes'] for d in recovery),
                     directories_skipped=skips, files_skipped=file_receipt['skipped'],
                     trace_bytes=(child/'trace.sqlite3').stat().st_size,
                     all_deleted_files_fully_recoverable=True, no_unresolved_file_deletion_intents=True,
                     recovery_index_sha256=m.sha_file(child/'recovery_index.json'),
                     plan_sha256=owner['plan_sha256'], file_cleanup_receipt_sha256=m.sha_file(child/'file_cleanup_receipt.json'),
                     free_bytes_before=before, free_bytes_after=free(), elapsed_seconds=time.monotonic()-start,
                     user_cpu_seconds=usage.ru_utime, system_cpu_seconds=usage.ru_stime,
                     max_rss_kib=usage.ru_maxrss, github_contacted=False, robot_nodes_restarted=False,
                     temperature_queried=False, motion_commands_sent=False, completed_ns=time.time_ns())
        m.atomic(child/'receipt.json', final)
        return final

def apply():
    plan = verify_plan()
    approval = read(R/'root_execution_approval.json')
    assert approval['approved'] and approval['approved_plan_sha256'] == PLAN_SHA
    assert approval['executor_sha256'] == m.sha_file(Path(__file__))
    assert approval['source_tool_sha256'] == m.sha_file(W/'tools/minimal_experiment_trace.py')
    parent_evidence_check(plan)
    config_paths = [m.ROOT/'maintenance/storage_control.json',m.ROOT/'maintenance/failed_record_space_guard_policy.json',
                    m.ROOT/'maintenance/failed_record_space_guard_catalog.json',m.ROOT/'maintenance/failed_record_space_guard_protected.json',
                    m.ROOT/'grasp_records/archive_config.json']
    configs = {str(p):m.sha_file(p) for p in config_paths}
    before = free()
    started = time.monotonic()
    lock = (m.ROOT/'maintenance/failed_record_space_guard_lock').open('a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    try:
        with concurrent.futures.ProcessPoolExecutor(max_workers=2) as pool:
            futures = [pool.submit(run_worker, owner) for owner in approval['workers']]
            results = [future.result() for future in futures]
        parent_evidence_check(plan)
        assert all(m.sha_file(Path(p)) == s for p,s in configs.items())
        result = dict(schema=1, state='complete' if all(r['state']=='complete' for r in results) else 'complete_with_protected_skips',
                      scope='Only the 15 approved complete software regression runs; gen54 excluded; only exact pytest_tmp contents/empty directories',
                      stop_reason='finite_approved_scope_exhausted', plan_sha256=PLAN_SHA,
                      root_approval_sha256=m.sha_file(R/'root_execution_approval.json'),
                      executor_sha256=m.sha_file(Path(__file__)), workers=results,
                      files_deleted=sum(r['files_deleted'] for r in results), directories_deleted=sum(r['directories_deleted'] for r in results),
                      source_deleted_bytes=sum(r['source_deleted_bytes'] for r in results),
                      allocated_deleted_bytes=sum(r['file_allocated_deleted_bytes']+r['directory_allocated_deleted_bytes'] for r in results),
                      trace_database_bytes=sum(r['trace_bytes'] for r in results),
                      free_bytes_before=before, free_bytes_after=free(), net_free_bytes_change=free()-before,
                      retained_parent_files=len(plan['retained_parent_evidence']), all_parent_evidence_sha256_unchanged=True,
                      all_deleted_files_fully_recoverable=True, directory_metadata_preserved=True,
                      storage_config_sha256=configs, storage_configs_unchanged=True,
                      parallel_workers=2, elapsed_seconds=time.monotonic()-started,
                      github_contacted=False, robot_nodes_restarted=False, motion_commands_sent=False,
                      temperature_queried=False, completed_ns=time.time_ns())
        m.atomic(R/'receipt.json', result)
        print(json.dumps(result, ensure_ascii=False), flush=True)
    except BaseException as error:
        m.atomic(R/'execution_stopped.json',dict(error=repr(error),recorded_ns=time.time_ns()))
        raise
    finally:
        lock.close()

if __name__ == '__main__':
    assert sys.argv[1:] in [['prepare'],['apply']]
    prepare() if sys.argv[1]=='prepare' else apply()
