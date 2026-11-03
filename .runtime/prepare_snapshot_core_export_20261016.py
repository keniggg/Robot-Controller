"""Export only verified historical Release bytes into a new isolated git tree."""
from pathlib import Path, PurePosixPath
import collections, hashlib, json, os, re, subprocess, tarfile
W=Path(__file__).resolve().parents[1]
SNAP=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/maintenance/source_snapshot_20261016_unfinished_supplemental_view')
OUT=SNAP/'browseable_core_branch';OUT.mkdir(exist_ok=True)
DEST=W/'.runtime/github_core_export_20261016_unfinished'
ARCHIVE=SNAP/'github_readback/2026.10.16-unfinished-supplemental-view-code.tar.gz'
MANIFEST=SNAP/'github_readback/source-manifest.json'
BRANCH='snapshot/2026-10-16-unfinished-supplemental-view-code'
EXPECTED_A='d7f60fda328a34b60355a65b53e0dea1377c657d7fa278647f5dab13c7ac24ad'
EXPECTED_M='55ec5e794bfc2776297cc394d412e9e9db2ff94583f568e21a672ac6d072b97c'
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 return h.hexdigest()
def original_git_state(root):
 def git(*args):return subprocess.check_output(['git','-C',str(root),*args],env=dict(os.environ,GIT_OPTIONAL_LOCKS='0'),text=True).strip()
 index=Path(git('rev-parse','--git-path','index'))
 if not index.is_absolute():index=root/index
 return dict(head=git('rev-parse','HEAD'),branch=git('symbolic-ref','HEAD'),index_sha256=sha(index))
originals=[W,W.parents[1]];before={str(p):original_git_state(p) for p in originals}
assert sha(ARCHIVE)==EXPECTED_A and sha(MANIFEST)==EXPECTED_M
metadata=json.loads(MANIFEST.read_text());source={r['archive_path']:r for r in metadata['files']}
assert len(source)==len(metadata['files'])==52420
allowed={'.py','.c','.cpp','.h','.hpp','.md','.rst','.txt','.yaml','.yml','.json','.xml','.launch','.sh','.bash','.js','.ts','.toml','.ini','.cfg','.config','.conf','.service','.timer','.msg','.srv','.urdf','.srdf','.xacro','.rviz','.ui','.perspective','.scad','.cmake','.code-workspace'}
allowed_names={'.catkin_workspace','.gitignore','.clang-format','.setup_assistant','LICENSE','LICENSE.md','LICENSE.txt','COPYING','Makefile','Dockerfile','CMakeLists.txt','setup.cfg','rqt_calibration_evaluator','rqt_calibrationmovements','rqt_easy_handeye'}
forbidden_parts={'.git','.runtime','build','devel','install','logs','log','ros_log','.ros_log','grasp_records','experiments','recordings','__pycache__','.pytest_cache','.mypy_cache','node_modules','.cache','.venv','venv','data'}
core={'src','docs','tools','srv','config','configs','launch','scripts','tests'}
def selection(path):
 p=PurePosixPath(path);parts=p.parts
 if len(parts)<2 or any(q in {'..',''} for q in parts) or p.is_absolute():raise ValueError(path)
 prefix,rel=parts[0],PurePosixPath(*parts[1:])
 if prefix=='external_dependencies':return path,None
 if prefix not in {'current_runtime_workspace','ide_root_workspace'}:return None,'non-core historical root'
 if rel.parts[0] not in core and not (len(rel.parts)==1 and rel.name in allowed_names):return None,'outside core text roots'
 if any(x in forbidden_parts for x in rel.parts):return None,'runtime/data/generated tree'
 if rel.parts[:3]==('docs','verification','evidence'):return None,'historical experimental evidence'
 if rel.name=='.env' or rel.name.startswith('.env.') or rel.suffix.lower() in {'.key','.pem','.p12','.pfx'} or rel.name in {'credentials.json','.netrc','id_rsa','id_ed25519'}:return None,'credential path'
 if '.bak' in rel.name or rel.name.endswith('~'):return None,'historical backup'
 if rel.suffix.lower() not in allowed and rel.name not in allowed_names:return None,'non-text asset or unsupported artifact'
 return (str(rel) if prefix=='current_runtime_workspace' else str(PurePosixPath('ide_root_workspace')/rel)),None
chosen={};excluded=[]
for p,r in source.items():
 dest,reason=selection(p)
 if dest:chosen[p]=(dest,r)
 else:excluded.append(dict(archive_path=p,bytes=r['bytes'],reason=reason))
assert len({v[0] for v in chosen.values()})==len(chosen)
assert not DEST.exists();DEST.mkdir()
written=[];seen=set();high=[];assignments=[]
patterns={
 'private_key':re.compile(rb'-----BEGIN (?:RSA |EC |OPENSSH |DSA )?PRIVATE KEY-----'),
 'github_token':re.compile(rb'\b(?:gh[pousr]_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{40,})\b'),
 'openai_key':re.compile(rb'\bsk-(?:proj-|svcacct-)?[A-Za-z0-9_-]{35,}\b'),
 'aws_access_key':re.compile(rb'\b(?:AKIA|ASIA)[A-Z0-9]{16}\b'),
 'slack_token':re.compile(rb'\bxox[baprs]-[A-Za-z0-9-]{20,}\b')}
assignment=re.compile(rb'''(?i)(?:password|passwd|api[_-]?key|client[_-]?secret|access[_-]?token)\s*[:=]\s*["']([^"'\r\n]{4,})["']''')
with tarfile.open(ARCHIVE,'r|gz') as tar:
 for member in tar:
  if member.name not in chosen:continue
  if member.name in seen:raise ValueError('duplicate archive member')
  seen.add(member.name);relative,row=chosen[member.name];target=DEST/relative;target.parent.mkdir(parents=True,exist_ok=True)
  if member.issym():
   data=member.linkname.encode();assert row.get('symlink_target')==member.linkname
   assert member.name.endswith('/src/CMakeLists.txt') and member.linkname=='/opt/ros/noetic/share/catkin/cmake/toplevel.cmake'
   target.symlink_to(member.linkname);kind='symlink'
  else:
   assert member.isfile() and 'symlink_target' not in row
   data=tar.extractfile(member).read();assert b'\0' not in data,'binary data in selected text'
   data.decode('utf-8')
   target.write_bytes(data);os.chmod(target,0o755 if row['mode']&0o111 else 0o644);kind='file'
  digest=hashlib.sha256(data).hexdigest();assert len(data)==row['bytes'] and digest==row['sha256']
  actual=os.readlink(target).encode() if target.is_symlink() else target.read_bytes();assert actual==data
  written.append(dict(archive_path=member.name,branch_path=relative,bytes=len(data),sha256=digest,kind=kind,mode=oct(0o120000 if kind=='symlink' else (0o100755 if row['mode']&0o111 else 0o100644)),**({'symlink_target':member.linkname} if kind=='symlink' else {})))
  for name,pattern in patterns.items():
   for m in pattern.finditer(data):high.append(dict(path=relative,line=data[:m.start()].count(b'\n')+1,pattern=name,value_redacted=True))
  for m in assignment.finditer(data):assignments.append(dict(path=relative,line=data[:m.start()].count(b'\n')+1,pattern='literal_secret_named_assignment_review',matched_value_sha256=hashlib.sha256(m.group(1)).hexdigest(),value_redacted=True))
assert seen==set(chosen)
assert not high,high
# Historical restoration instructions are unchanged, with the new branch scope
# stated separately. The 26.5 MB original manifest stays a Release attachment.
restore=SNAP/'github_readback/RESTORE.md';expected_restore='6e2b03045edafb218a65a4c73a8b3cc423645dda0f6de32e08eb498a45b0fa7b';assert sha(restore)==expected_restore
meta=DEST/'snapshot_metadata';meta.mkdir();(meta/'ORIGINAL_RELEASE_RESTORE.md').write_bytes(restore.read_bytes())
release='https://github.com/keniggg/Robot-Controller/releases/tag/snapshot-20261016-unfinished-supplemental-view-code'
readme='''# 2026.10.16 未完成补充视角观察代码：可浏览核心快照

此分支逐字节导出自此前已上传并读回核验的 Release 附件，**不是之后继续修复的 GEN31 工作区版本**。当时状态为未完成 GEN31、尚未部署、尚未通过完整回归；未声称补充视角抓取或实物夹持成功。

- 分支根 `src/`、`docs/`、`tools/`、`srv/` 等来自归档 `current_runtime_workspace/`，用于直接浏览当时实际运行工作区的核心代码和操作文档。
- `ide_root_workspace/` 保存同一归档里的旧 IDE 根工作区核心文本，仅供对照；它与主工作区不同，不能混用。
- `external_dependencies/` 保留归档内的三份手眼配置、服务和来源合同文本；未从当前机器补入任何新内容。
- `snapshot-export-manifest.json` 记录每个原归档路径到分支路径的映射、字节数和 SHA256；符号链接以原链接目标校验。原文件内容未改写。

本分支供 GitHub 直接读取核心源码。权重、大型网格、图片、NPZ/实验录制、构建/缓存目录、`.runtime` 历史备份和旧 deployment 重复副本不在此导出范围，因此**不是包含所有依赖和资产的可直接运行安装包**。原脚本内的绝对路径、外部 SDK/模型/标定引用保持历史原文。`src/CMakeLists.txt` 的原 catkin 符号链接仍指向 `/opt/ros/noetic/share/catkin/cmake/toplevel.cmake`。

完整原始快照及 `source-manifest.json`、SHA256SUMS、历史恢复说明仍在 [原 Release]('''+release+''')。`snapshot_metadata/ORIGINAL_RELEASE_RESTORE.md` 原文描述的是该完整附件，不是此精简分支。它不代表当前软件已修复、已经部署或可自动启动机械臂。

## 原附件身份

- Archive SHA256: `'''+EXPECTED_A+'''`
- Source manifest SHA256: `'''+EXPECTED_M+'''`
- Archive: `2026.10.16-unfinished-supplemental-view-code.tar.gz`
- Release title: `2026.10.16日未完成补充视角观察代码`

本分支创建不会修改现有分支或原 Release tag。所有软件/硬件验证结论以相应时期记录为准。
'''
(DEST/'README_SNAPSHOT.md').write_text(readme)
manifest=dict(schema=1,branch=BRANCH,release=release,archive=dict(name=ARCHIVE.name,sha256=EXPECTED_A),source_manifest=dict(name=MANIFEST.name,sha256=EXPECTED_M),snapshot_status=metadata['snapshot_status'],physical_success_claimed=False,selection='Core readable source/config/docs/tests text; original runtime root primary, original IDE root nested.',excluded_by_reason=dict(collections.Counter(r['reason'] for r in excluded)),selected_count=len(written),selected_bytes=sum(r['bytes'] for r in written),files=sorted(written,key=lambda r:r['branch_path']),extra_files=[dict(path='snapshot_metadata/ORIGINAL_RELEASE_RESTORE.md',source_release_asset='RESTORE.md',sha256=expected_restore),dict(path='README_SNAPSHOT.md',origin='new export scope explanation',sha256=sha(DEST/'README_SNAPSHOT.md'))])
(DEST/'snapshot-export-manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
(OUT/'excluded-files.json').write_text(json.dumps(excluded,ensure_ascii=False,indent=2)+'\n')
(OUT/'sensitive-scan.json').write_text(json.dumps(dict(high_confidence_findings=high,literal_assignment_candidates=assignments,patterns=list(patterns),content_values_never_printed=True,scope='All exported original UTF-8 text and original symlink target bytes; heuristic scan, not a proof of absent secrets.'),indent=2)+'\n')
subprocess.run(['git','init',str(DEST)],check=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
subprocess.run(['git','-C',str(DEST),'symbolic-ref','HEAD','refs/heads/'+BRANCH],check=True)
subprocess.run(['git','-C',str(DEST),'add','--force','--all'],check=True)
after={str(p):original_git_state(p) for p in originals};assert before==after
receipt=dict(status='prepared_staged_for_independent_review_not_pushed',export_dir=str(DEST),branch=BRANCH,original_workspaces_before=before,original_workspaces_after=after,original_workspaces_unchanged=True,archive_sha256=EXPECTED_A,source_manifest_sha256=EXPECTED_M,selected_count=len(written),selected_bytes=sum(r['bytes'] for r in written),export_manifest_sha256=sha(DEST/'snapshot-export-manifest.json'),scope_readme_sha256=sha(DEST/'README_SNAPSHOT.md'),exclusions_sha256=sha(OUT/'excluded-files.json'),sensitive_scan_sha256=sha(OUT/'sensitive-scan.json'),high_confidence_secret_findings=0,literal_assignment_candidates=len(assignments),git_commit_created=False,pushed=False)
(OUT/'export-preparation.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(receipt,ensure_ascii=False));print('literal_scan_metadata',json.dumps(assignments))
