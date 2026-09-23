#!/usr/bin/env python3
"""Durable, explicit-file grasp archive queue; never controls ROS or the arm."""
import argparse
import contextlib
import datetime as dt
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import stat
import sys
import time
import uuid

GIB = 1 << 30
RETAIN = 5
RESULTS = {'success', 'failure', 'unknown', 'interrupted'}
ALLOWED = {'.bag', '.log', '.jsonl', '.json', '.jpg', '.jpeg', '.png', '.txt'}
PERSISTENT = {'manifest.json', 'result_summary.json', 'archive_manifest.json', '.lock'}
DEFAULTS = dict(repo='', tag_prefix='grasp-record-', chunk_size=GIB,
                min_free_bytes=2*GIB, retain_count=RETAIN, delete_enabled=False,
                deletion_validation=None, retry_base_seconds=30, retry_max_seconds=3600)

class ArchiveError(RuntimeError): pass
class InsufficientSpace(ArchiveError): pass


def utc_now():
    return dt.datetime.now(dt.timezone.utc).isoformat()


def root_path(root=None):
    return Path(root or os.environ.get('GRASP_RECORD_ROOT', Path(__file__).resolve().parents[1] / '.grasp_records')).expanduser().resolve()


def atomic_json(path, value):
    path = Path(path)
    tmp = path.with_name('.' + path.name + '.' + uuid.uuid4().hex + '.tmp')
    try:
        with tmp.open('x', encoding='utf-8') as f:
            json.dump(value, f, ensure_ascii=False, indent=2, allow_nan=False)
            f.write('\n'); f.flush(); os.fsync(f.fileno())
        os.replace(str(tmp), str(path))
        fd = os.open(str(path.parent), os.O_RDONLY | os.O_DIRECTORY)
        try: os.fsync(fd)
        finally: os.close(fd)
    finally:
        if tmp.exists(): tmp.unlink()


def load_config(config_path=None):
    p = Path(config_path) if config_path else root_path() / 'archive_config.json'
    value = dict(DEFAULTS)
    if p.exists(): value.update(json.loads(p.read_text()))
    if value['retain_count'] != RETAIN: raise ArchiveError('retain_count must be exactly 5 total records')
    for key in ('chunk_size', 'min_free_bytes'):
        if type(value[key]) is not int or value[key] < 0: raise ArchiveError('invalid ' + key)
    if not 0 < value['chunk_size'] < 2*GIB: raise ArchiveError('invalid chunk size')
    return value


def config_for(root, config=None):
    return {**load_config(Path(root) / 'archive_config.json'), **(config or {}), 'retain_count': RETAIN}


@contextlib.contextmanager
def locked(path, blocking=True):
    path = Path(path)
    flags = os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW
    fd = os.open(str(path), flags, 0o600)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | (0 if blocking else fcntl.LOCK_NB))
        yield
    finally:
        os.close(fd)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(4 << 20), b''): digest.update(chunk)
    return digest.hexdigest()


def managed_dir(directory):
    p = Path(directory).absolute()
    if p.is_symlink() or not p.is_dir(): raise ArchiveError('record directory is not a real directory')
    # All parents must be real directories: do not follow aliases into source/worktrees.
    if any(x.is_symlink() for x in (p, *p.parents)): raise ArchiveError('symlink in record path')
    return p


def data_path(directory, relative):
    directory = managed_dir(directory)
    rel = Path(relative)
    if rel.is_absolute() or not rel.parts or any(p in ('..', '.', '.git', '.worktrees', 'src') for p in rel.parts):
        raise ArchiveError('unsafe registered path: ' + str(relative))
    if (any(p.startswith('.') for p in rel.parts) or rel.name in PERSISTENT
            or any(word in rel.name.lower() for word in ('calibration', 'handeye', 'launch', 'startup'))
            or rel.suffix.lower() not in ALLOWED):
        raise ArchiveError('not recording data: ' + str(relative))
    p = directory / rel
    if any(x.is_symlink() for x in (p, *p.parents)): raise ArchiveError('symlink in registered path')
    if p.exists():
        s = p.stat()
        if not stat.S_ISREG(s.st_mode) or s.st_nlink != 1: raise ArchiveError('registered file must be a private regular file')
    return p


def load_manifest(directory):
    p = managed_dir(directory)
    path = p / 'manifest.json'
    if path.is_symlink(): raise ArchiveError('symlink manifest')
    m = json.loads(path.read_text())
    if m.get('schema') != 1 or m.get('record_id') != p.name: raise ArchiveError('unmanaged record directory')
    if m.get('result') not in RESULTS: raise ArchiveError('invalid result')
    return m


def summary(m):
    return {k:m.get(k) for k in ('record_id','started_at','completed_at','result','failure_reason','result_evidence','state','local_data_deleted','upload','remote_manifest')}


def save_manifest(directory, m):
    atomic_json(Path(directory)/'manifest.json', m)
    atomic_json(Path(directory)/'result_summary.json', summary(m))


def records(root):
    root = root_path(root)
    if not root.exists(): return []
    out=[]
    for p in root.iterdir():
        if p.is_dir() and not p.is_symlink() and (p/'manifest.json').is_file():
            try: out.append((p,load_manifest(p)))
            except (ArchiveError,ValueError,OSError): continue
    return out


def process_alive(m):
    owner=m.get('owner',{})
    try:
        if owner.get('boot_id') != Path('/proc/sys/kernel/random/boot_id').read_text().strip(): return False
        raw=Path('/proc/%d/stat'%owner['pid']).read_text()
        return raw.rsplit(')',1)[1].split()[19] == owner['start_ticks']
    except (OSError,KeyError,ValueError): return False


def preflight_space(root, config=None):
    root=root_path(root); root.mkdir(parents=True,exist_ok=True)
    cfg=config_for(root,config)
    free=shutil.disk_usage(root).free
    if free < cfg['min_free_bytes']:
        # Reclaim already verified older records before a potentially long
        # upload. All retention and remote verification gates still apply.
        if cfg.get('delete_enabled'): cleanup(root,cfg,dry_run=False)
        free=shutil.disk_usage(root).free
        if free < cfg['min_free_bytes']:
            process_queue(root,cfg)
            if cfg.get('delete_enabled'): cleanup(root,cfg,dry_run=False)
            free=shutil.disk_usage(root).free
    result={'free_bytes':free,'required_free_bytes':cfg['min_free_bytes'],'recording_allowed':free>=cfg['min_free_bytes'],'at':utc_now()}
    atomic_json(root/'disk_status.json',result)
    if not result['recording_allowed']:
        raise InsufficientSpace('RECORDING_BLOCKED_LOW_DISK: free=%d required=%d; arm control unchanged' % (free,cfg['min_free_bytes']))
    return result


def begin_record(root=None, metadata=None, config=None):
    root=root_path(root); preflight_space(root,config)
    stamp=dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    p=root/(stamp+'_'+uuid.uuid4().hex)
    p.mkdir(mode=0o700)
    owner={'pid':os.getpid(),'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
           'start_ticks':Path('/proc/self/stat').read_text().rsplit(')',1)[1].split()[19]}
    m={'schema':1,'record_id':p.name,'started_at':utc_now(),'completed_at':None,'state':'recording',
       'result':'unknown','failure_reason':'','result_evidence':None,'owner':owner,'metadata':metadata or {},
       'recorder_closed':False,'files':[],'local_data_deleted':False,
       'upload':{'status':'recording','attempts':0,'next_retry_at':0,'error':''}}
    save_manifest(p,m)
    return p


def validate_result(result, evidence):
    if result not in RESULTS: raise ArchiveError('invalid result')
    if result in ('success','failure') and not evidence:
        raise ArchiveError('success/failure requires actual task or manual evidence')


def finalize_record(record_dir, result='unknown', failure_reason='', evidence=None, closed_files=None):
    validate_result(result,evidence)
    if closed_files is None: raise ArchiveError('explicit closed_files required')
    p=managed_dir(record_dir)
    with locked(p/'.lock'):
        m=load_manifest(p)
        if m['state'] != 'recording': raise ArchiveError('record already finalized')
        entries=[]
        for rel in dict.fromkeys(str(x) for x in closed_files):
            f=data_path(p,rel)
            if not f.is_file(): raise ArchiveError('closed file missing: '+rel)
            before=f.stat(); digest=sha256(f); after=f.stat()
            if (before.st_size,before.st_mtime_ns,before.st_ino)!=(after.st_size,after.st_mtime_ns,after.st_ino):
                raise ArchiveError('closed file changed during hash')
            entries.append({'path':rel,'size_bytes':after.st_size,'sha256':digest,
                            'upload':{'status':'pending'}})
        if not any(x['path'].endswith('.bag') for x in entries): raise ArchiveError('a normally closed original .bag is required')
        m.update(state='complete',completed_at=utc_now(),recorder_closed=True,result=result,
                 failure_reason=failure_reason,result_evidence=evidence,files=entries)
        m['upload']={'status':'pending','attempts':0,'next_retry_at':0,'error':''}
        save_manifest(p,m)
        return m


def mark_result(record_dir, result, reason='', evidence=None):
    validate_result(result,evidence)
    p=managed_dir(record_dir)
    with locked(p/'.lock'):
        m=load_manifest(p)
        m.update(result=result,failure_reason=reason,result_evidence=evidence)
        if m['state']=='complete':
            m['upload'].update(status='pending',next_retry_at=0)
            m.pop('remote_manifest',None)
        save_manifest(p,m)
        return m


def protected_ids(items):
    complete=[m for p,m in items if m.get('state')=='complete' and m.get('recorder_closed')]
    complete.sort(key=lambda m:(m['completed_at'],m['record_id']),reverse=True)
    return {m['record_id'] for m in complete[:RETAIN]}


def eligible(m,protected):
    return (m['record_id'] not in protected and m['state']=='complete' and m.get('recorder_closed')
            and not m.get('local_data_deleted') and m.get('upload',{}).get('status')=='verified'
            and m.get('remote_manifest',{}).get('verified') is True
            and all(f.get('upload',{}).get('status')=='verified' for f in m['files']))


def preview(root,config=None):
    root=root_path(root); cfg=config_for(root,config); items=records(root); keep=protected_ids(items)
    return {'dry_run':True,'repo':cfg['repo'],'retain_count':RETAIN,'delete_enabled':cfg['delete_enabled'],
            'protected':sorted(keep),
            'upload':[{'record_id':m['record_id'],'result':m['result'],'files':[f['path'] for f in m['files'] if f.get('upload',{}).get('status')!='verified']} for p,m in items if m['state']=='complete' and m['upload']['status']!='verified' ],
            'cleanup':[{'record_id':m['record_id'],'files':[f['path'] for f in m['files']],'bytes':sum(f['size_bytes'] for f in m['files'])} for p,m in items if eligible(m,keep)],
            'records':[{'record_id':m['record_id'],'state':m['state'],'result':m['result'],'upload':m['upload']['status'],'protected':m['record_id'] in keep,'local_data_deleted':m['local_data_deleted']} for p,m in items]}


def make_store(config,m):
    from grasp_archive_transport import GithubReleaseStore
    owner,repo=config['repo'].split('/',1)
    return GithubReleaseStore(owner,repo,config['tag_prefix']+m['record_id'],chunk_size=config['chunk_size'])


def unchanged(p,f):
    path=data_path(p,f['path'])
    return path.is_file() and path.stat().st_size==f['size_bytes'] and sha256(path)==f['sha256']


def process_queue(root,config=None,store_factory=None,max_records=None):
    root=root_path(root); root.mkdir(parents=True,exist_ok=True); cfg=config_for(root,config)
    if max_records is not None and (type(max_records) is not int or max_records < 1):
        raise ValueError('max_records must be a positive integer')
    report={'uploaded':[],'failed':[],'pending':[]}
    if not cfg['repo']: return dict(report,configuration_required='GitHub owner/repo')
    factory=store_factory or make_store
    try:
        with locked(root/'.queue.lock',blocking=False):
            # Completion order, not filesystem enumeration or directory names.
            # Old records reclaim capacity only after verified backup.
            ordered=sorted(records(root), key=lambda item: (
                item[1].get('completed_at') or '', item[1]['record_id']))
            attempted=0
            for p,old in ordered:
                if old['state']!='complete' or old['upload']['status']=='verified': continue
                if old['upload'].get('next_retry_at',0)>time.time(): continue
                if max_records is not None and attempted >= max_records: break
                attempted += 1
                with locked(p/'.lock'):
                    m=load_manifest(p)
                    try:
                        if not m.get('recorder_closed'): raise ArchiveError('recorder not closed')
                        # Validate local immutability before any network write.
                        for f in m['files']:
                            if not m['local_data_deleted'] and not unchanged(p,f): raise ArchiveError('local recording changed: '+f['path'])
                        store=factory(cfg,m); store.prepare_release(create=True)
                        m['upload']['status']='uploading'; save_manifest(p,m)
                        for f in m['files']:
                            if f['upload']['status']=='verified' and store.verify_file(f['upload']['entry']): continue
                            if m['local_data_deleted']: raise ArchiveError('deleted recording lost remote verification')
                            entry=store.upload_file(m['record_id'],p/f['path'],f['path'])
                            if not store.verify_file(entry): raise ArchiveError('remote verification failed: '+f['path'])
                            if not unchanged(p,f): raise ArchiveError('local recording changed while uploading')
                            f['upload']={'status':'verified','entry':entry,'verified_at':utc_now()}
                            save_manifest(p,m)
                        archived={**m,'upload':{**m['upload'],'status':'verified','error':''}}
                        archived.pop('remote_manifest',None)
                        atomic_json(p/'archive_manifest.json',archived)
                        entry=store.upload_file(m['record_id'],p/'archive_manifest.json','archive_manifest.json')
                        if not store.verify_file(entry): raise ArchiveError('remote manifest verification failed')
                        m['remote_manifest']={'verified':True,'entry':entry,'verified_at':utc_now()}
                        m['upload'].update(status='verified',error='',next_retry_at=0,verified_at=utc_now())
                        save_manifest(p,m)
                        report['uploaded'].append(m['record_id'])
                    except Exception as exc:
                        attempt=int(m['upload'].get('attempts',0))+1
                        m['upload'].update(status='retry',attempts=attempt,error=str(exc),
                            next_retry_at=time.time()+min(cfg['retry_max_seconds'],cfg['retry_base_seconds']*2**min(attempt-1,10)))
                        save_manifest(p,m); report['failed'].append({'record_id':m['record_id'],'error':str(exc)})
    except BlockingIOError: report['busy']=True
    return report


def deletion_authorized(root,cfg):
    proof=cfg.get('deletion_validation')
    if not cfg.get('delete_enabled') or not isinstance(proof,dict): return False
    return (proof.get('repo')==cfg['repo'] and proof.get('retain_count')==RETAIN
            and all(proof.get(k) is True for k in ('upload_verified','restore_verified','retention_verified')))


def cleanup(root,config=None,store_factory=None,dry_run=True):
    root=root_path(root); cfg=config_for(root,config)
    plan=preview(root,cfg)
    if dry_run: return plan
    if not deletion_authorized(root,cfg): return {'deleted':[],'blocked':'deletion disabled or validation incomplete'}
    result={'deleted':[],'failed':[]}; factory=store_factory or make_store
    with locked(root/'.queue.lock'):
        items=records(root); keep=protected_ids(items)
        for p,old in items:
            if not eligible(old,keep): continue
            with locked(p/'.lock'):
                m=load_manifest(p)
                if not eligible(m,protected_ids(records(root))): continue
                try:
                    store=factory(cfg,m)
                    if not store.verify_file(m['remote_manifest']['entry']): raise ArchiveError('remote manifest no longer verified')
                    # Verify every registered file before deleting any file.
                    for f in m['files']:
                        path=data_path(p,f['path'])
                        if path.exists() and not unchanged(p,f): raise ArchiveError('local data changed; cleanup refused')
                        if not path.exists() and not m.get('deletion_started_at'): raise ArchiveError('registered data missing')
                        if not store.verify_file(f['upload']['entry']): raise ArchiveError('remote data verification failed')
                    m['deletion_started_at']=m.get('deletion_started_at') or utc_now(); save_manifest(p,m)
                    for f in m['files']:
                        path=data_path(p,f['path'])
                        if path.exists():
                            # O_NOFOLLOW also protects the last component at open time.
                            fd=os.open(str(path),os.O_RDONLY|os.O_NOFOLLOW)
                            try:
                                st=os.fstat(fd)
                                if st.st_nlink!=1 or st.st_size!=f['size_bytes']: raise ArchiveError('file changed before deletion')
                                digest=hashlib.sha256()
                                while True:
                                    block=os.read(fd,4<<20)
                                    if not block: break
                                    digest.update(block)
                                after=os.fstat(fd)
                                identity=lambda x:(x.st_dev,x.st_ino,x.st_size,x.st_mtime_ns,x.st_ctime_ns)
                                if digest.hexdigest()!=f['sha256'] or identity(st)!=identity(after):
                                    raise ArchiveError('file changed during remote verification or deletion hash')
                                if path.is_symlink() or identity(path.stat())!=identity(after):
                                    raise ArchiveError('file replaced before deletion')
                                path.unlink()
                            finally: os.close(fd)
                        f['local_deleted']=True
                        save_manifest(p,m)
                    m.update(local_data_deleted=True,local_deleted_at=utc_now()); save_manifest(p,m)
                    result['deleted'].append(m['record_id'])
                except Exception as exc:
                    m['cleanup_error']=str(exc); save_manifest(p,m)
                    result['failed'].append({'record_id':m['record_id'],'error':str(exc)})
    return result


def recover_abandoned(root):
    found=[]
    for p,m in records(root):
        if m['state']!='recording' or process_alive(m): continue
        # Crash after bag.close / log.close but before manifest commit: resume
        # from explicit close receipts. Never guess closure from file extension.
        try:
            outcome=json.loads((p/'recorder_outcome.json').read_text())
            command_path=p/'command_result.json'
            if not outcome.get('bag_closed') or not outcome.get('finished_at'):
                raise ArchiveError('normal close was not confirmed')
            files=list(outcome['closed_files'])+['recorder_ready.json','recorder_outcome.json']
            if m.get('metadata',{}).get('launcher'):
                command=json.loads(command_path.read_text())
                if command.get('closed_logs')!=['runner.log','recorder.log']:
                    raise ArchiveError('runner logs may still be open')
                files+=command['closed_logs']+['command_result.json']
                if (p/'authorization.json').is_file():files.append('authorization.json')
                result=command['result'];reason=command.get('failure_reason','');evidence=command.get('evidence')
            else:
                from run_recorded_grasp import classify_result
                result,reason,evidence=classify_result(outcome,'')
            finalize_record(p,result,reason,evidence,files)
            found.append({'record_id':m['record_id'],'recovered_closed':True})
            continue
        except (OSError,ValueError,KeyError,ArchiveError):
            pass
        with locked(p/'.lock'):
            m=load_manifest(p)
            if m['state']!='recording' or process_alive(m): continue
            m.update(state='incomplete',result='interrupted',completed_at=utc_now(),
                failure_reason='Recorder owner exited without a confirmed close; preserve all local data')
            m['upload']['status']='blocked_unclosed';save_manifest(p,m);found.append(m['record_id'])
    return found


def restore_manifest(manifest,destination,config=None,store_factory=None):
    m=json.loads(Path(manifest).read_text()); dest=Path(destination)
    if dest.exists(): raise ArchiveError('restore destination must not exist')
    cfg={**DEFAULTS,**(config or {})}; store=(store_factory or make_store)(cfg,m)
    dest.mkdir(parents=True)
    for f in m['files']:
        target=data_path(dest,f['path']); target.parent.mkdir(parents=True,exist_ok=True)
        store.restore_file(f['upload']['entry'],target)
        if target.stat().st_size!=f['size_bytes'] or sha256(target)!=f['sha256']: raise ArchiveError('restored original hash mismatch')
    atomic_json(dest/'manifest.json',m)
    return {'restored':len(m['files']),'destination':str(dest),'verified':True}


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--root',default=str(root_path()))
    sub=ap.add_subparsers(dest='command',required=True)
    sub.add_parser('preview'); sub.add_parser('preflight'); sub.add_parser('recover')
    worker=sub.add_parser('worker');worker.add_argument('--once',action='store_true');worker.add_argument('--interval',type=float,default=30)
    clean=sub.add_parser('cleanup'); clean.add_argument('--apply',action='store_true')
    mark=sub.add_parser('mark');mark.add_argument('record_id');mark.add_argument('result',choices=sorted(RESULTS));mark.add_argument('--reason',required=True)
    restore=sub.add_parser('restore');restore.add_argument('manifest');restore.add_argument('destination')
    enable=sub.add_parser('enable-deletion');enable.add_argument('--validation-report',required=True)
    args=ap.parse_args(); root=root_path(args.root);cfg=config_for(root)
    if args.command=='preview':result=preview(root,cfg)
    elif args.command=='preflight':result=preflight_space(root,cfg)
    elif args.command=='recover':result={'interrupted':recover_abandoned(root)}
    elif args.command=='cleanup':result=cleanup(root,cfg,dry_run=not args.apply)
    elif args.command=='mark':result=mark_result(root/args.record_id,args.result,args.reason,{'source':'manual_confirmation','reason':args.reason,'at':utc_now()})
    elif args.command=='restore':result=restore_manifest(args.manifest,args.destination,cfg)
    elif args.command=='enable-deletion':
        proof=json.loads(Path(args.validation_report).read_text())
        candidate=dict(cfg,delete_enabled=True,deletion_validation=proof)
        if not deletion_authorized(root,candidate): raise ArchiveError('validation proof is incomplete or for another repository')
        if not proof.get('validation_record_manifest'): raise ArchiveError('remote roundtrip evidence required')
        validated=json.loads(Path(proof['validation_record_manifest']).read_text())
        store=make_store(candidate,validated)
        if not store.verify_file(validated['remote_manifest']['entry'],readback=True): raise ArchiveError('validation remote manifest failed')
        for f in validated['files']:
            if not store.verify_file(f['upload']['entry'],readback=True): raise ArchiveError('validation data failed')
        atomic_json(root/'archive_config.json',candidate)
        result={'enabled':True,'preview':preview(root,candidate),'validation_report':args.validation_report}
    elif args.command=='worker':
        while True:
            cfg=config_for(root)
            recover_abandoned(root)
            prior_cleanup=cleanup(root,cfg,dry_run=not cfg['delete_enabled'])
            # Release the queue lock and run retention after each record; a
            # later multi-GiB upload must not postpone reclaiming an old one.
            result=process_queue(root,cfg,max_records=1)
            result['cleanup_before_upload']=prior_cleanup
            result['cleanup']=cleanup(root,cfg,dry_run=not cfg['delete_enabled'])
            result['disk']={'free_bytes':shutil.disk_usage(root).free,'required_free_bytes':cfg['min_free_bytes']}
            result['disk']['recording_allowed']=result['disk']['free_bytes']>=cfg['min_free_bytes']
            atomic_json(root/'worker_status.json',dict(result,at=utc_now()))
            print(json.dumps(result,ensure_ascii=False),flush=True)
            if args.once:return
            time.sleep(max(1,args.interval))
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':
    try:main()
    except (ArchiveError,OSError,ValueError) as exc:
        print(str(exc),file=sys.stderr);sys.exit(2)
