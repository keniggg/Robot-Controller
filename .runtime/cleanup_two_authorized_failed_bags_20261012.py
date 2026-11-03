"""One explicitly authorized two-bag deletion; never changes archive policy."""
from pathlib import Path
import hashlib,json,os,stat,time,shutil
root=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
folder=root/'maintenance/user_failed_cleanup_proposed_20261012T0310Z'
proposal_path=folder/'proposal.json';proposal=json.loads(proposal_path.read_text())
allowed=('20261006T085709.051090Z_bc8e72c457f542a18a60e9e94fb6b3e1','20261006T093000.069892Z_2b67317c7fb54de5a1fd94d6b4f7bb49')
assert Path('/media/zhuyupei/SanStand').is_mount()
assert tuple(x['record_id'] for x in proposal['candidates'])==allowed
assert not (folder/'execution_receipt.json').exists()
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
auth=dict(message_id='call_aaa53eb3ec4c42c0b6425dd35736be4b:0',user_answer='允许，仅清理这两份失败录制后继续抓取',authorized_record_ids=list(allowed),only_file_name='keyframes.bag',authorization_ns=time.time_ns(),proposal_sha256=sha(proposal_path),automatic_deletion_enabled=False,upload_enabled=False,archive_policy='local_only')
(folder/'new_user_authorization.json').write_text(json.dumps(auth,ensure_ascii=False,indent=2)+'\n')
receipt=dict(status='validating',authorization=auth,free_before=shutil.disk_usage(root).free,validated=[],deleted=[],policy_files_unchanged=True,motion_commands_sent=False)
control=root/'maintenance/storage_control.json';control_sha=sha(control)
def save(): (folder/'execution_receipt.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n')
def identity(p):
 s=p.stat();return (s.st_dev,s.st_ino,s.st_size,s.st_mtime_ns)
def scan_open(paths):
 matches=[];denied=0
 for p in Path('/proc').iterdir():
  if not p.name.isdigit():continue
  try:fds=list((p/'fd').iterdir())
  except PermissionError:denied+=1;continue
  except OSError:continue
  for f in fds:
   try:target=os.readlink(f)
   except OSError:continue
   if target in paths:matches.append(dict(pid=int(p.name),fd=f.name,path=target))
 assert not matches,matches
 return dict(matches=matches,other_user_processes_unreadable=denied,scope='accessible host FDs plus independently closed recorder metadata')
save()
for row in proposal['candidates']:
 p=root/'grasp_records'/row['record_id']/'keyframes.bag';assert str(p)==row['delete_candidate_exact_path'] and p.resolve()==p and not p.is_symlink() and stat.S_ISREG(p.stat().st_mode)
 expected=(row['device_at_review'],row['inode_at_review'],row['size_bytes'],row['mtime_ns']);assert identity(p)==expected
 m=json.loads((p.parent/'manifest.json').read_text());assert m['state']=='complete' and m['result']=='failure' and m['recorder_closed'] is True
 for evidence in row['evidence_files']:assert sha(Path(evidence['path']))==evidence['sha256'],evidence['path']
 owner=m.get('owner',{});pid=owner.get('pid');proc=Path('/proc')/str(pid)/'stat'
 if owner.get('boot_id')==Path('/proc/sys/kernel/random/boot_id').read_text().strip() and proc.exists():
  assert proc.read_text().rsplit(')',1)[1].split()[19]!=str(owner.get('start_ticks')),'original recorder still alive'
 digest=hashlib.sha256()
 with p.open('rb') as f:
  while True:
   b=f.read(8*1024*1024)
   if not b:break
   digest.update(b)
 assert digest.hexdigest()==row['bag_sha256_from_existing_manifest']
 assert identity(p)==expected
 summary=Path(row['prepared_summary_path']);assert sha(summary)==row['prepared_summary_sha256']
 summary_copy=p.parent/'问题方法与验证摘要_20261012.md';assert not summary_copy.exists();summary_copy.write_bytes(summary.read_bytes())
 receipt['validated'].append(dict(path=str(p),identity=list(expected),sha256=digest.hexdigest(),summary=str(summary_copy)))
 save();print('Validated closed failed bag '+row['record_id'],flush=True)
receipt['open_file_check']=scan_open([x['path'] for x in receipt['validated']]);receipt['status']='deleting_exact_authorized_list';save()
for row in receipt['validated']:
 p=Path(row['path']);assert identity(p)==tuple(row['identity'])
 p.unlink();assert not p.exists()
 deleted=dict(path=str(p),size_bytes=row['identity'][2],sha256=row['sha256'],deleted_ns=time.time_ns(),authorization_message_id=auth['message_id'])
 receipt['deleted'].append(deleted);save()
 (p.parent/'raw_recording_removal_receipt_20261012.json').write_text(json.dumps(deleted,indent=2)+'\n')
for row in proposal['candidates']:
 for evidence in row['evidence_files']:assert sha(Path(evidence['path']))==evidence['sha256']
assert sha(control)==control_sha
receipt.update(status='complete',free_after=shutil.disk_usage(root).free,deleted_bytes=sum(x['size_bytes'] for x in receipt['deleted']),all_retained_evidence_hashes_unchanged=True,storage_control_sha256=control_sha,finished_ns=time.time_ns());save()
print(json.dumps(dict(status=receipt['status'],deleted_files=len(receipt['deleted']),deleted_bytes=receipt['deleted_bytes'],free_after=receipt['free_after'],receipt=str(folder/'execution_receipt.json'))),flush=True)
