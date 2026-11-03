#!/usr/bin/env python3
"""Create an immutable source backup using a separate index; no ROS operations."""
import hashlib
import json
import os
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/maintenance/source_snapshot_20261008')
BRANCH = 'backup/20261008-initial-view-grasp-code'
TAG = 'snapshot-20261008-initial-view-grasp-code'
TITLE = '10.8日，一次性抓取通过的完整代码'


def main():
    os.chdir(str(ROOT))
    if not Path('/media/zhuyupei/SanStand').is_mount():
        raise SystemExit('SanStand mount unavailable')
    required = ROOT / 'docs/verification/2026-10-08-initial-view-grasp-regression.md'
    if not required.is_file():
        raise SystemExit('Independent regression report not ready')
    OUT.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, GIT_INDEX_FILE=str(ROOT / '.runtime/github_snapshot_20261008.index'))
    def git(*args, **kwargs):
        return subprocess.check_output(['git', *args], env=env, **kwargs)
    for ref in ('refs/heads/' + BRANCH, 'refs/tags/' + TAG):
        if subprocess.run(['git', 'show-ref', '--verify', '--quiet', ref], env=env).returncode == 0:
            raise SystemExit('Ref already exists: ' + ref)
    original_branch = subprocess.check_output(['git', 'symbolic-ref', 'HEAD']).decode().strip()
    original_index = Path(subprocess.check_output(['git', 'rev-parse', '--git-path', 'index']).decode().strip())
    original_index_sha = hashlib.sha256(original_index.read_bytes()).hexdigest()
    # Refresh auxiliary copies so their hashes describe exactly this snapshot.
    subprocess.run(['/usr/bin/python3', str(ROOT / '.runtime/prepare_source_snapshot_20261008.py')], check=True)
    git('add', '-A', '--', '.')
    manifest_path = ROOT / 'deployment/2026-10-08/source_tree_manifest.json'
    entries = []
    for entry in git('ls-files', '--stage', '-z').split(b'\0'):
        if not entry:
            continue
        attrs, raw_path = entry.split(b'\t', 1)
        mode, blob, stage = attrs.decode().split()
        rel = os.fsdecode(raw_path)
        if rel == str(manifest_path.relative_to(ROOT)):
            continue
        path = ROOT / rel
        data = os.readlink(str(path)).encode() if path.is_symlink() else path.read_bytes()
        object_sha = hashlib.sha1(('blob %d\0' % len(data)).encode() + data).hexdigest()
        if object_sha != blob:
            raise SystemExit('File changed during snapshot: ' + rel)
        entries.append(dict(path=rel, mode=mode, bytes=len(data),
                            git_blob=blob, sha256=hashlib.sha256(data).hexdigest()))
    manifest_path.write_text(json.dumps(dict(schema=1, excludes_self=True,
        symlink_hash_scope='link target string; git mode 120000', files=entries),
        ensure_ascii=False, indent=2) + '\n')
    git('add', '--', str(manifest_path.relative_to(ROOT)))
    parent = git('rev-parse', 'HEAD').decode().strip()
    tree = git('write-tree').decode().strip()
    body = (TITLE + '\n\n2026-10-08 current complete source snapshot with historical '
            '2026-10-06 initial-view physical grasp success evidence. Includes all current '
            'source/configuration, ignored startup sources, model/calibration dependencies, '
            'and restore instructions.\n\n65 historical file hashes compared: 64 unchanged; '
            'the sole change is a startup test. All 10 recorded production sources unchanged. '
            '652 focused direct/contact regression tests pass. No new physical grasp claimed; '
            'supplemental-view motion planning remains unresolved.\n\n'
            'Experiment recordings/logs remain local_only on SanStand.\n')
    commit = git('commit-tree', tree, '-p', parent, input=body.encode()).decode().strip()
    git('update-ref', 'refs/heads/' + BRANCH, commit, '0' * 40)
    git('tag', '-a', TAG, commit, '-m', body)
    assert subprocess.check_output(['git', 'symbolic-ref', 'HEAD']).decode().strip() == original_branch
    assert hashlib.sha256(original_index.read_bytes()).hexdigest() == original_index_sha
    archive = OUT / (TAG + '.tar.gz')
    if archive.exists():
        raise SystemExit('Archive already exists; refusing overwrite')
    git('archive', '--format=tar.gz', '--prefix=Robot-Controller-20261008/',
        '-o', str(archive), commit)
    digest = hashlib.sha256(archive.read_bytes()).hexdigest()
    (OUT / 'SHA256SUMS').write_text(digest + '  ' + archive.name + '\n')
    receipt = dict(label=TITLE, branch=BRANCH, tag=TAG, commit=commit, tree=tree,
        parent=parent, source_files=len(entries) + 1, source_bytes=sum(x['bytes'] for x in entries),
        archive=str(archive), archive_bytes=archive.stat().st_size, archive_sha256=digest,
        runtime_branch_preserved=original_branch, original_index_sha256=original_index_sha,
        runtime_index_unchanged=True, physical_motion_requested=False, uploaded=False)
    (OUT / 'snapshot_receipt.json').write_text(json.dumps(receipt, ensure_ascii=False, indent=2) + '\n')
    (OUT / 'release_notes.md').write_text(
        '# ' + TITLE + '\n\n' +
        '按要求保存的 2026-10-08 当前完整源码、配置、模型、测试和启动依赖。'
        '10.8 为保存日期；最近实物成功为 10 月 6 日 gen8，用户确认“已夹住新目标”，闭爪保持、不提升。\n\n'
        '历史 65 项文件指纹中 64 项相同，唯一变化为测试文件；10 个关键生产源码全部一致。'
        '直接抓取/公共执行链 652 项回归通过；本轮支撑面修复另有 425 通过、1 跳过及真实数据回放。'
        '未重新进行实物成功验收，补充视角仍卡在观察移动路径规划。\n\n'
        '完整恢复说明：[docs/operations/2026-10-08-source-snapshot.md]('
        'https://github.com/keniggg/Robot-Controller/blob/' + TAG + '/docs/operations/2026-10-08-source-snapshot.md)。'
        '包含原被忽略的自有运行脚本、模型、标定和 SHA256 清单；需安装 ROS/SDK/MuJoCo/GraspNet 依赖并构建。'
        '原始实验及日志继续留在 SanStand，不启动上传/删除归档。\n\n'
        '固定提交：`' + commit + '`。可下载本页 Source code (zip/tar.gz)，或克隆固定标签。\n')
    print(json.dumps(receipt, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
