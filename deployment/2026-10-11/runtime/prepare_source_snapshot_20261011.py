#!/usr/bin/env python3
"""Prepare a source-only tree and isolated Git index/objects, never a commit/ref/push.

All output belongs to an explicit new SanStand directory. Actual working trees,
indices, refs, ROS and devices are untouched. Invoke only after final deployment.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path('/home/zhuyupei/alicia_wa_full')
WORK = ROOT / '.worktrees/protocol-v3-upgrade'
NEW = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_no_skin_20261011T000625Z')
MOUNT = Path('/media/zhuyupei/SanStand')
PLAN = WORK / '.runtime/plan_source_snapshot_20261011.py'


def sha(data):
    return hashlib.sha256(data).hexdigest()


def raw(path):
    return os.fsencode(os.readlink(str(path))) if path.is_symlink() else path.read_bytes()


def write_json(path, data):
    with path.open('x', encoding='utf8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
        f.write('\n')


def git(base, *args):
    return subprocess.check_output(['git', '-C', str(base), *args], env=dict(os.environ, GIT_OPTIONAL_LOCKS='0'))


def state(base):
    index = Path(git(base, 'rev-parse', '--git-path', 'index').decode().strip())
    if not index.is_absolute():
        index = base / index
    return dict(head=git(base, 'rev-parse', 'HEAD').decode().strip(),
        branch=git(base, 'branch', '--show-current').decode().strip(),
        index_sha256=sha(index.read_bytes()))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--deployment', type=Path, required=True,
        help='Final successful deployment result JSON; no GEN18 fallback')
    ap.add_argument('--output', type=Path, required=True, help='New external directory, must not exist')
    ap.add_argument('--runtime-source', action='append', required=True,
        help='Explicit final repair sealer/startup source relative to WORK; repeatable')
    ap.add_argument('--metadata', type=Path, action='append', required=True,
        help='Compact integration/review/test manifest JSON or Markdown, never raw logs; repeatable')
    args = ap.parse_args()
    assert MOUNT.is_mount(), 'SanStand unavailable; no system-disk fallback'
    out, deployment = args.output.resolve(), args.deployment.resolve()
    assert str(out).startswith(str(NEW) + '/') and not out.exists()
    assert str(deployment).startswith(str(NEW) + '/')
    deployed = json.loads(deployment.read_text())
    assert deployed.get('deployment_passed') is True and deployed.get('source_sha256')
    before = {'work': state(WORK), 'root': state(ROOT)}
    refs_before = sha(git(WORK, 'show-ref'))
    out.mkdir(parents=True, exist_ok=False)
    command = [sys.executable, str(PLAN), '--output', str(out / 'inventory'), '--deployment', str(deployment)]
    runtime_sources = list(dict.fromkeys(args.runtime_source + [
        '.runtime/plan_source_snapshot_20261011.py', '.runtime/prepare_source_snapshot_20261011.py']))
    for rel in runtime_sources:
        command.extend(['--runtime-source', rel])
    subprocess.run(command, check=True)
    plan = json.loads((out / 'inventory/plan.json').read_text())
    inventory = json.loads((out / 'inventory/candidate_source_inventory.json').read_text())
    assert not inventory['candidate_secret_findings'], 'Resolve candidate secret findings before preparing publication'
    assert not inventory['missing_dependencies']
    assert not plan['deployed_baseline']['observed_current_drift'], 'Working sources differ from final deployment'
    tree_dir = out / 'tree'
    tree_dir.mkdir()
    entries, entries_by_path = [], {}
    def put(destination, data, mode='100644', source=None, expected_sha=None):
        rel = Path(destination)
        assert not rel.is_absolute() and '..' not in rel.parts and '.git' not in rel.parts
        actual = sha(data)
        assert expected_sha is None or expected_sha == actual, ('source_changed', source)
        if destination in entries_by_path:
            assert entries_by_path[destination]['sha256'] == actual, ('destination_collision', destination)
            return
        target = tree_dir / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        # ExFAT cannot represent symlinks or Unix execute bits. Store link text
        # and preserve the authoritative Git mode in the isolated index.
        with target.open('xb') as f:
            f.write(data)
        item = dict(path=destination, mode=mode, bytes=len(data), sha256=actual,
            git_blob=hashlib.sha1(('blob %d\0' % len(data)).encode() + data).hexdigest())
        if source:
            item['source'] = source
        if mode == '120000':
            item['offline_tree_representation'] = 'literal symlink target bytes; checkout from Git restores link'
        entries.append(item); entries_by_path[destination] = item
    for item in inventory['files'] + inventory['dependencies'] + inventory['root_workspace_overlay']:
        if item.get('deleted'):
            continue
        put(item['snapshot_path'], raw(Path(item['source'])), item['mode'], item['source'], item['sha256'])

    metadata_paths = list(dict.fromkeys([deployment] + [p.resolve() for p in args.metadata]))
    metadata_entries = []
    for index, path in enumerate(metadata_paths):
        assert str(path).startswith((str(NEW) + '/', str(WORK / '.runtime') + '/'))
        assert path.suffix in ('.json', '.md') and path.stat().st_size <= 2_000_000, ('not_compact_metadata', str(path))
        data = path.read_bytes()
        if path.suffix == '.json':
            json.loads(data)
        dest = 'deployment/2026-10-11/verification_metadata/%02d_%s' % (index, path.name)
        put(dest, data, source=str(path))
        metadata_entries.append(dict(source=str(path), snapshot_path=dest, sha256=sha(data)))

    dependencies = dict(schema=1, source_worktree=str(WORK),
        files=inventory['dependencies'], metadata=metadata_entries,
        excluded_binary_models=inventory['external_binary_models'],
        external_packages=['ROS Noetic / MoveIt and Python 3.8 environment',
            'RealSense SDK 2.56.5 with Python bindings: runtime/realsense-2.56.5/install_manifest.json',
            'MuJoCo 3.2.3 wheel/native runtime: runtime/candidate_simulation/pins.json',
            'Existing GraspNet server/CUDA/WSL runtime; project server source is included, external environment/weights are not'],
        automatic_restore_or_ros_start=False)
    put('deployment/2026-10-11/source_dependencies.json', (json.dumps(dependencies, ensure_ascii=False, indent=2)+'\n').encode())
    overlay = dict(schema=1, base=before['root'], files=inventory['root_workspace_overlay'],
        policy='Apply only in a separate ROOT-base checkout; never overlay old root versions onto current WORK sources.',
        unsaved_editor_buffers_captured=False)
    put('deployment/2026-10-11/root_workspace_overlay_manifest.json', (json.dumps(overlay, ensure_ascii=False, indent=2)+'\n').encode())

    readme = '''# 2026-10-11 源码快照恢复说明

本快照的主源码来自 `.worktrees/protocol-v3-upgrade`，包含保存时的修改和未跟踪源码、配置、模型网格、工具、测试以及全部已有操作/问题文档。最终实验与修复结论见 [问题与修复入口](../../docs/operations/2026-10-11-code-snapshot-and-grasp-issues.md)；源码归档不表示补视抓取已成功。

`source_dependencies.json` 为原路径到本目录复制件的精确映射；每项包含大小、文件模式和 SHA256。请先在新目录检出固定提交/标签并核验 `source_tree_manifest.json`。主树可以重新构建，`build/devel` 不是源文件，未收录。

恢复顺序（本脚本没有执行这些步骤）：

1. 安装已有版本的 ROS Noetic、MoveIt、Python/相机依赖及固定 MuJoCo 3.2.3 环境；核对 dependency 清单。未进行空白机器安装测试。
2. 按精确映射将 `runtime/` 恢复至工作区 `.runtime/`，将必需的外置手眼与固定仿真契约恢复至原路径。另一机器改变路径时，launch、模型 mesh、相机标定、仿真 pins 与相关代码里的绝对路径应一致更新并重新审核。不要通过跳过 pins 检查来运行。
3. 外部模型权重没有加入本次新快照树。清单给出 SHA256、原路径和已有历史提交；可使用现有本地文件，或从已有仓库历史恢复相同路径后核验 SHA256。未在本次上传或下载 CUDA/WSL、外部 GraspNet 权重及第三方 native SDK 二进制。
4. 重新构建 catkin 工作区。当前完整启动入口为 `runtime/start_full_latest_no_skin_20261010.sh` 与同名 launch，电子皮肤为关闭。`runtime/observation_measurement_consumers_20261008.launch` 是专属三计算节点的配置，恢复时应核对当前配置入口及 ROS 节点唯一性。旧 `historical_root_entries/` 仅作历史保存，不能作为当前默认启动入口。
5. ROOT 的独立磁盘编辑仅在 `root_workspace_overlay/` 保存，其基线提交与删改清单见 `root_workspace_overlay_manifest.json`。需要恢复旧 ROOT 时请另建基线 checkout 后应用；绝不能覆盖当前主源码。未保存的 VS Code 编辑缓冲区不属于磁盘快照，不能声称已经保存。

该准备目录的 `tree/` 在 SanStand 文件系统上用字节保存软链接目标，Unix 文件模式由私有 Git index 精确保留。请从生成的 Git 树/提交检出；不要把准备目录当作已经恢复好的工作区直接构建。

原始实验、rosbag、RGB-D 流及运行日志继续保存在 SanStand。这里只包含必要的固定配置、紧凑测试 fixture 和修复/验证元数据，不改变 `local_only`、`upload_enabled=false`、`delete_enabled=false`，不启用归档自动上传/删除。本次来源清单生成与快照准备没有调用 ROS、温度或机械臂控制，也没有创建 commit、branch、tag 或 push。
'''
    put('deployment/2026-10-11/README.md', readme.encode())
    all_sources = inventory['files'] + inventory['dependencies'] + inventory['root_workspace_overlay']
    for item in all_sources:
        if not item.get('deleted'):
            assert sha(raw(Path(item['source']))) == item['sha256'], ('changed_while_copying', item['source'])
    for rel, expected in deployed['source_sha256'].items():
        assert sha(raw(WORK / rel)) == expected, ('deployment_changed', rel)
        matched = [x for x in entries if x.get('source') == str(WORK / rel)]
        assert matched and all(x['sha256'] == expected for x in matched), ('deployed_file_missing', rel)

    manifest = dict(schema=1, excludes_self=True, source_worktree=str(WORK), workspace_states=before,
        deployment=dict(path=str(deployment), sha256=sha(deployment.read_bytes())),
        source_files=sorted(entries, key=lambda x: x['path']),
        explicit_exclusions=inventory['exclusions'], preserved_original_files=True,
        largest_files=sorted(entries, key=lambda x:x['bytes'], reverse=True)[:20],
        model_weights_in_new_tree=False, experiment_raw_data_in_new_tree=False,
        raw_symlink_storage='Git mode 120000; tree directory contains target bytes',
        clean_machine_build_verified=False, physical_success_claimed=False)
    put('deployment/2026-10-11/source_tree_manifest.json', (json.dumps(manifest, ensure_ascii=False, indent=2)+'\n').encode())
    assert all(x['bytes'] < 100 * 1024**2 for x in entries), 'Review oversized source blob before publication'
    assert all(Path(x['path']).suffix.lower() not in {'.bag','.active','.log','.pyc','.pt','.pth','.onnx','.ckpt','.safetensors'} for x in entries)

    common = Path(git(WORK, 'rev-parse', '--git-common-dir').decode().strip())
    if not common.is_absolute():
        common = (WORK / common).resolve()
    objects = out / 'objects'; objects.mkdir()
    index = out / 'snapshot.index'
    env = dict(os.environ, GIT_DIR=str(common), GIT_WORK_TREE=str(tree_dir),
        GIT_INDEX_FILE=str(index), GIT_OBJECT_DIRECTORY=str(objects),
        GIT_ALTERNATE_OBJECT_DIRECTORIES=str(common / 'objects'), GIT_OPTIONAL_LOCKS='0')
    def isolated(*argv, **kwargs):
        return subprocess.check_output(['git', '-c', 'core.autocrlf=false', *argv], env=env, cwd=str(tree_dir), **kwargs)
    isolated('read-tree', '--empty')
    rows = []
    for item in sorted(entries, key=lambda x: x['path']):
        data = (tree_dir / item['path']).read_bytes()
        blob = isolated('hash-object', '-w', '--stdin', input=data).decode().strip()
        assert blob == item['git_blob']
        rows.append(('%s %s\t%s\0' % (item['mode'], blob, item['path'])).encode())
    isolated('update-index', '-z', '--index-info', input=b''.join(rows))
    tree = isolated('write-tree').decode().strip()
    indexed = isolated('ls-files', '--stage', '-z').split(b'\0')
    assert len([x for x in indexed if x]) == len(entries)
    for row in indexed:
        if not row:
            continue
        left, filename = row.split(b'\t', 1); mode, blob, stage = left.decode().split()
        entry = entries_by_path[os.fsdecode(filename)]
        assert stage == '0' and mode == entry['mode'] and blob == entry['git_blob']
    assert before == {'work': state(WORK), 'root': state(ROOT)}
    assert refs_before == sha(git(WORK, 'show-ref')), 'Original Git refs changed during preparation'
    receipt = dict(schema=1, prepared=True, created_ns=time.time_ns(), tree=tree,
        tree_directory=str(tree_dir), private_index=str(index), private_objects=str(objects),
        common_git_dir=str(common), parent_commit='b2fdac56a62fbb1b4f34af2125f9d362f80891b0',
        commit_created=False, refs_changed=False, uploaded=False, source_upload_authorized=True,
        publication_state='final_source_tree_prepared_root_may_commit_and_upload',
        source_files=len(entries), source_bytes=sum(x['bytes'] for x in entries),
        largest_files=sorted(entries,key=lambda x:x['bytes'],reverse=True)[:10],
        working_indices_unchanged=True, original_heads_unchanged=True, original_refs_unchanged=True,
        deployment=dict(path=str(deployment),sha256=sha(deployment.read_bytes())),
        manifest_sha256=entries_by_path['deployment/2026-10-11/source_tree_manifest.json']['sha256'],
        snapshot_branch='backup/20261011-current-grasp-source', snapshot_tag='snapshot-20261011-current-grasp-source',
        next_step='Root verifies receipt/tree and imports private Git objects before creating a new dated commit/ref/tag and pushing only those refs. Never remove private objects while published refs still depend on them.',
        import_hint='With GIT_OBJECT_DIRECTORY set to private objects and actual objects as alternates, pack the prepared tree via git pack-objects --revs --stdout; import that pack with git index-pack --stdin into the ordinary repository before commit/ref creation. No import/commit/ref/push executed by this script.',
        restore_document='deployment/2026-10-11/README.md')
    write_json(out / 'prepared_receipt.json', receipt)
    print(json.dumps(dict(receipt=str(out / 'prepared_receipt.json'),sha256=sha((out/'prepared_receipt.json').read_bytes()),
        tree=tree,files=len(entries),max_file_bytes=max(x['bytes'] for x in entries),commit_created=False,uploaded=False),ensure_ascii=False))


if __name__ == '__main__':
    main()
