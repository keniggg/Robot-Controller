"""Publish a reviewed, immutable source snapshot without changing either checkout."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

ROOT = Path('/home/zhuyupei/alicia_wa_full')
WORK = ROOT / '.worktrees/protocol-v3-upgrade'
NEW = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_no_skin_20261011T000625Z')
URL = 'https://github.com/keniggg/Robot-Controller.git'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--prepared', type=Path, required=True)
    ap.add_argument('--resume-commit')
    ap.add_argument('--resume-tag-object')
    args = ap.parse_args()
    assert bool(args.resume_commit) == bool(args.resume_tag_object)
    prepared = args.prepared.resolve()
    assert Path('/media/zhuyupei/SanStand').is_mount()
    assert str(prepared).startswith(str(NEW) + '/')
    data = json.loads(prepared.read_text())
    assert data['prepared'] and data['source_upload_authorized']
    out = prepared.parent
    receipt_path = out / 'publication_receipt.json'
    assert not receipt_path.exists(), 'Do not create duplicate commits after a partial publish'
    base_env = dict(os.environ, GIT_OPTIONAL_LOCKS='0')
    for key in ('GIT_DIR', 'GIT_INDEX_FILE', 'GIT_WORK_TREE', 'GIT_OBJECT_DIRECTORY', 'GIT_ALTERNATE_OBJECT_DIRECTORIES'):
        base_env.pop(key, None)

    def git(*argv, path=WORK, timeout=60, **kwargs):
        return subprocess.check_output(['git', '-C', str(path), *argv], env=base_env, timeout=timeout, **kwargs)

    def sha(value):
        return hashlib.sha256(value).hexdigest()

    def state(path):
        idx = Path(git('rev-parse', '--git-path', 'index', path=path).decode().strip())
        if not idx.is_absolute():
            idx = path / idx
        return dict(head=git('rev-parse', 'HEAD', path=path).decode().strip(),
            branch=git('branch', '--show-current', path=path).decode().strip(), index_sha256=sha(idx.read_bytes()))

    assert git('remote', 'get-url', 'origin').decode().strip() == URL
    assert git('remote', 'get-url', '--push', '--all', 'origin').decode().splitlines() == [URL]
    before = dict(work=state(WORK), root=state(ROOT))
    manifest_path = Path(data['tree_directory']) / 'deployment/2026-10-11/source_tree_manifest.json'
    assert sha(manifest_path.read_bytes()) == data['manifest_sha256']
    manifest = json.loads(manifest_path.read_text())
    assert before == manifest['workspace_states'], 'Original checkouts changed since preparation'
    for entry in manifest['source_files']:
        assert sha((Path(data['tree_directory']) / entry['path']).read_bytes()) == entry['sha256']
    deployment = Path(data['deployment']['path'])
    assert sha(deployment.read_bytes()) == data['deployment']['sha256']
    for rel, digest in json.loads(deployment.read_text())['source_sha256'].items():
        assert sha((WORK / rel).read_bytes()) == digest, ('deployed_source_changed', rel)
    branch, tag = data['snapshot_branch'], data['snapshot_tag']
    refs = ('refs/heads/' + branch, 'refs/tags/' + tag)
    assert not git('ls-remote', '--refs', 'origin', *refs, timeout=60).strip(), 'Remote snapshot already exists'
    local_refs = git('for-each-ref', '--format=%(refname)', *refs).decode().splitlines()
    assert not local_refs, 'Local snapshot ref already exists'
    parent = data['parent_commit']
    assert git('rev-parse', '--verify', parent + '^{commit}').decode().strip() == parent
    prior_backup = git('ls-remote', '--refs', 'origin',
        'refs/heads/backup/20261008-initial-view-grasp-code').decode().splitlines()
    assert prior_backup == [parent + '\trefs/heads/backup/20261008-initial-view-grasp-code']

    # Import a standalone pack into the normal object store. Published refs
    # must never depend on temporary/external alternate object directories.
    isolated_env = dict(base_env, GIT_DIR=data['common_git_dir'],
        GIT_OBJECT_DIRECTORY=data['private_objects'],
        GIT_ALTERNATE_OBJECT_DIRECTORIES=str(Path(data['common_git_dir']) / 'objects'))
    pack_path = out / 'snapshot-import.pack'
    if not args.resume_commit:
        with pack_path.open('xb') as handle:
            subprocess.run(['git', 'pack-objects', '--revs', '--stdout'],
                input=(data['tree'] + '\n').encode(), stdout=handle, env=isolated_env, check=True, timeout=240)
        with pack_path.open('rb') as handle:
            git('index-pack', '--stdin', stdin=handle, timeout=120)
    assert git('cat-file', '-t', data['tree']).strip() == b'tree'
    # Verify imported Git bytes, including preserved symlink/execute modes.
    tree_rows = git('ls-tree', '-r', '-z', data['tree']).split(b'\0')
    expected = {entry['path']: entry for entry in manifest['source_files']}
    assert len([row for row in tree_rows if row]) == len(expected) + 1
    for row in tree_rows:
        if not row:
            continue
        meta, name = row.split(b'\t', 1)
        mode, kind, oid = meta.decode().split()
        path = os.fsdecode(name)
        if path == 'deployment/2026-10-11/source_tree_manifest.json':
            assert mode == '100644' and sha(git('cat-file', 'blob', oid)) == data['manifest_sha256']
        else:
            entry = expected[path]
            assert mode == entry['mode'] and kind == 'blob' and oid == entry['git_blob']

    message = ('10.11日，当前完整源码与补视抓取问题修复记录\n\n'
        '保存当前运行工作树的完整源码、配置、启动依赖及问题/已解决问题文档。\n'
        '修复 GEN19 支撑暂缺的三端等待交接，376 项实际工作树回归通过，已部署。\n'
        '补视实物抓取仍待验证；不声明任务已成功。原始实验继续 SanStand local_only。\n'
        '保留 10.8 备份和两个现有工作区，ROOT 独立编辑另存 overlay。\n')
    body = out / 'commit-message.txt'
    if args.resume_commit:
        commit, tag_object = args.resume_commit, args.resume_tag_object
        assert git('show', '-s', '--format=%T%n%P', commit).decode().splitlines() == [data['tree'], parent]
        assert git('cat-file', '-t', tag_object).strip() == b'tag'
        tag_body = git('cat-file', 'tag', tag_object).decode()
        assert tag_body.startswith(f'object {commit}\ntype commit\ntag {tag}\ntagger ')
        assert tag_body.split('\n\n', 1)[1] == message
    else:
        body.write_text(message)
        commit = git('commit-tree', data['tree'], '-p', parent, '-F', str(body)).decode().strip()
        identity = git('var', 'GIT_COMMITTER_IDENT').decode().strip()
        tag_text = f'object {commit}\ntype commit\ntag {tag}\ntagger {identity}\n\n' + message
        tag_object = git('mktag', input=tag_text.encode()).decode().strip()
    # Git 2.25 atomically commits its --stdin batch at EOF; explicit
    # start/prepare/commit commands were added in newer Git versions.
    transaction = f'create {refs[0]} {commit}\ncreate {refs[1]} {tag_object}\n'
    git('update-ref', '--stdin', input=transaction.encode())
    result = dict(prepared_receipt=str(prepared), prepared_sha256=sha(prepared.read_bytes()),
        created_ns=time.time_ns(), remote_url=URL, branch=branch, tag=tag, commit=commit,
        tree=data['tree'], annotated_tag_object=tag_object, parent_commit=parent,
        source_files=data['source_files'], source_manifest_sha256=data['manifest_sha256'],
        source_uploaded=False, original_checkout_before=before,
        original_experiments_uploaded=False, archive_policy='local_only')

    def save():
        receipt_path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')

    save()
    try:
        output = git('push', '--atomic', '--porcelain', 'origin',
            '--force-with-lease=' + refs[0] + ':', '--force-with-lease=' + refs[1] + ':',
            refs[0] + ':' + refs[0], refs[1] + ':' + refs[1], timeout=300)
        (out / 'git-push-result.txt').write_bytes(output)
        remote_lines = git('ls-remote', 'origin', *refs, refs[1] + '^{}',
            'refs/heads/backup/20261008-initial-view-grasp-code').decode().splitlines()
        remote = {line.split('\t')[1]: line.split('\t')[0] for line in remote_lines}
        assert remote[refs[0]] == commit and remote[refs[1]] == tag_object
        assert remote[refs[1] + '^{}'] == commit
        assert remote['refs/heads/backup/20261008-initial-view-grasp-code'] == parent
        after = dict(work=state(WORK), root=state(ROOT))
        assert after == before
        result.update(source_uploaded=True, remote_verified=remote,
            original_checkout_after=after, original_indices_unchanged=True,
            verified_ns=time.time_ns(), commit_url=URL[:-4]+'/commit/'+commit,
            download_url='https://github.com/keniggg/Robot-Controller/archive/refs/tags/'+tag+'.zip')
    except Exception as exc:
        result['publication_error'] = str(exc)
        save()
        raise
    save()
    print(json.dumps({k:result[k] for k in ('source_uploaded','commit','branch','tag','source_files','commit_url','download_url')}, ensure_ascii=False))


if __name__ == '__main__':
    main()
