#!/usr/bin/env python3
"""Inventory a source backup only. Never stage, commit, change refs, or upload."""
import argparse
import collections
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import time

ROOT = Path('/home/zhuyupei/alicia_wa_full')
WORK = ROOT / '.worktrees/protocol-v3-upgrade'
EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
NEW = EXT / 'ros_log/full_latest_no_skin_20261011T000625Z'
OUT = NEW / 'github_source_snapshot_20261011'
DEPLOYMENT = NEW / 'gen18_station_registration_reacquire_compute_deployment/result.json'
SOURCE_EXTS = {'.py', '.sh', '.bash', '.launch', '.yaml', '.yml', '.json', '.xml', '.urdf', '.md', '.service', '.ini', '.cpp', '.hpp', '.h', '.js'}
DENY_PARTS = {'.git', '.worktrees', 'build', 'devel', 'install', '__pycache__', '.pytest_cache', '.catkin_tools', 'CMakeFiles', '.ros_log', '.grasp_records'}
DENY_EXTS = {'.bag', '.active', '.log', '.pyc', '.pyo', '.so', '.o', '.a', '.deb', '.whl', '.tar', '.gz', '.zip', '.mp4', '.avi'}
WEIGHT_EXTS = {'.pt', '.pth', '.onnx', '.ckpt', '.safetensors'}
SECRET_NAMES = {'.env', 'credentials.json', 'token.json', 'auth.json', 'id_rsa', 'id_ed25519'}


def git(base, *args):
    return subprocess.check_output(['git', '-C', str(base), *args], env=dict(os.environ, GIT_OPTIONAL_LOCKS='0'))


def digest(data):
    return hashlib.sha256(data).hexdigest()


def describe(path, destination=None):
    path = Path(path)
    data = os.fsencode(os.readlink(str(path))) if path.is_symlink() else path.read_bytes()
    mode = '120000' if path.is_symlink() else ('100755' if path.stat().st_mode & 0o111 else '100644')
    result = dict(source=str(path), bytes=len(data), sha256=digest(data), mode=mode,
                  git_blob=hashlib.sha1(('blob %d\0' % len(data)).encode() + data).hexdigest())
    if destination is not None:
        result['snapshot_path'] = destination
    if path.is_symlink():
        result['symlink_target'] = os.readlink(str(path))
        result['hash_scope'] = 'link text, not target file'
    return result


def state(base):
    idx = Path(git(base, 'rev-parse', '--git-path', 'index').decode().strip())
    if not idx.is_absolute():
        idx = base / idx
    return dict(path=str(base), branch=git(base, 'branch', '--show-current').decode().strip(),
                head=git(base, 'rev-parse', 'HEAD').decode().strip(),
                index_path=str(idx), index_sha256=digest(idx.read_bytes()))


def excluded(rel):
    p = Path(rel)
    if p.name.lower() in SECRET_NAMES or p.suffix.lower() in {'.key', '.pem', '.p12', '.pfx'}:
        return 'credential-like filename; do not publish'
    if p.parts[0] in DENY_PARTS or p.parts[0].startswith(('build.bak_', 'devel.bak_')) or any(part in {'__pycache__', '.pytest_cache', 'CMakeFiles', '.git'} for part in p.parts):
        return 'generated cache/build/runtime recording tree'
    if p.parts[0] == '.runtime':
        return 'runtime is allowlisted separately; never recurse staging trees'
    if p.suffix.lower() in WEIGHT_EXTS:
        return 'binary model dependency; preserve external acquisition/placement reference'
    if p.suffix.lower() in DENY_EXTS:
        return 'raw recording, log, build, binary dependency or archive'
    if p.parts[0] == 'calibration_results' and p.parts[1] != 'fixtures':
        return 'historical experiment artifact; SanStand policy is separate'
    return None


def main():
    global OUT, DEPLOYMENT
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=OUT)
    parser.add_argument('--deployment', type=Path, default=DEPLOYMENT)
    parser.add_argument('--runtime-source', action='append', default=[])
    args = parser.parse_args()
    OUT = args.output.resolve()
    DEPLOYMENT = args.deployment.resolve()
    assert str(OUT).startswith(str(NEW) + '/'), 'Plan output must remain in the external session'
    assert str(DEPLOYMENT).startswith(str(NEW) + '/'), 'Deployment must be from this external session'
    assert Path('/media/zhuyupei/SanStand').is_mount(), 'No fallback from SanStand'
    assert not (OUT / 'plan.json').exists(), 'Refuse to overwrite an earlier plan'
    before = {'work': state(WORK), 'root': state(ROOT)}
    tracked = {os.fsdecode(p) for p in git(WORK, 'ls-files', '-z').split(b'\0') if p}
    untracked = {os.fsdecode(p) for p in git(WORK, 'ls-files', '--others', '--exclude-standard', '-z').split(b'\0') if p}
    # Global/info excludes must not hide actual project source or compact test fixtures.
    ignored = {os.fsdecode(p) for p in git(WORK, 'ls-files', '--others', '--ignored', '--exclude-standard', '-z',
        '--', 'src', 'docs', 'tools', 'srv', '.vscode', 'calibration_results/fixtures').split(b'\0') if p}
    ignored = {p for p in ignored if excluded(p) is None and
        (Path(p).suffix in SOURCE_EXTS | {'.msg', '.srv', '.cfg', '.cmake', '.scad', '.stl', '.STL', '.dae', '.npz', '.npy'}
         or Path(p).name in {'CMakeLists.txt', 'package.xml', 'README', 'LICENSE'})}
    source, omit, weights, deleted = [], [], [], []
    for rel in sorted(tracked | untracked | ignored):
        p = WORK / rel
        if not p.exists() and not p.is_symlink():
            deleted.append(rel)
            continue
        why = excluded(rel)
        if why:
            entry = dict(path=rel, reason=why, tracked=rel in tracked)
            if p.is_file():
                entry['bytes'] = p.stat().st_size
            omit.append(entry)
            if p.suffix.lower() in WEIGHT_EXTS:
                weights.append(dict(describe(p), restore_to=rel,
                    acquisition='Preserve existing local file or recover the same path from historical repository commit after verifying this SHA256; no model download performed.',
                    historical_commit=before['work']['head'], included_in_new_source_tree=False))
            continue
        item = describe(p, rel)
        item['git_state'] = 'tracked' if rel in tracked else ('ignored_project_source' if rel in ignored else 'untracked')
        source.append(item)

    # Explicit runtime roots only: sealed deployment sources and actual startup dependencies.
    deployed = json.loads(DEPLOYMENT.read_text())
    assert deployed.get('deployment_passed') is True
    runtime = {p for p in deployed['source_sha256'] if p.startswith('.runtime/')}
    runtime.update({
        '.runtime/full_latest_no_skin_20261010.launch',
        '.runtime/start_full_latest_no_skin_20261010.sh',
        '.runtime/d405_carton_depth_baseline.json', '.runtime/experiment_storage.json',
        '.runtime/realsense-2.56.5/install_manifest.json',
        '.runtime/run_reboot_realigned_grasp_20261010.py',
        '.runtime/seal_gen18_station_registration_reacquire_20261011.py',
        '.runtime/seal_gen16_feedback_pending_integration_20261010.py',
        '.runtime/plan_source_snapshot_20261011.py',
    })
    for rel in args.runtime_source:
        assert rel.startswith('.runtime/') and '..' not in Path(rel).parts and Path(rel).suffix in SOURCE_EXTS
        runtime.add(rel)
    for directory in ('full_startup_candidate', 'candidate_simulation', 'candidate_models/expanded_metric_20260929'):
        for p in (WORK / '.runtime' / directory).iterdir():
            if p.is_file() and p.suffix in SOURCE_EXTS and '.before_' not in p.name and not p.name.startswith('original_'):
                runtime.add(str(p.relative_to(WORK)))
    pins = json.loads((WORK / '.runtime/candidate_simulation/pins.json').read_text())
    for path in pins['files']:
        if path.startswith(str(WORK / '.runtime') + '/') and '/mujoco-3.2.3/' not in path:
            runtime.add(str(Path(path).relative_to(WORK)))
    # Resolve explicit references within these reviewed entry files, not arbitrary directories.
    for _ in range(3):
        additions = set()
        for rel in runtime:
            p = WORK / rel
            if not p.is_file() or p.suffix not in SOURCE_EXTS:
                continue
            for match in re.findall(r'\.runtime/[A-Za-z0-9_./-]+', p.read_text(errors='replace')):
                q = WORK / match
                if q.is_file() and q.suffix in SOURCE_EXTS - {'.json'} and '/mujoco-3.2.3/' not in match and '/realsense-2.56.5/' not in match:
                    additions.add(match)
        if additions.issubset(runtime):
            break
        runtime.update(additions)
    dependencies, missing = [], []
    for rel in sorted(runtime):
        p = WORK / rel
        if not p.is_file():
            missing.append(str(p))
            continue
        dependencies.append(dict(describe(p, 'deployment/2026-10-11/runtime/' + rel[len('.runtime/'):]),
                                 restore_to=str(p), category='runtime source/configuration'))
    externals = {
        ROOT / 'AGENTS.md': 'AGENTS.md',
        Path('/home/zhuyupei/.ros/easy_handeye/d405_cc200_auto_20260927_validated_eye_on_hand.yaml'): 'deployment/2026-10-11/external_config/d405_cc200_auto_20260927_validated_eye_on_hand.yaml',
        Path(pins['old_contract_source']): 'deployment/2026-10-11/external_config/server_source_contract.json',
        Path('/home/zhuyupei/.config/systemd/user/alicia-candidate-simulation.service'): 'deployment/2026-10-11/external_config/alicia-candidate-simulation.service',
    }
    for p, dest in externals.items():
        if p.is_file():
            dependencies.append(dict(describe(p, dest), restore_to=str(p), category='fixed external source/configuration'))
        else:
            missing.append(str(p))
    for rel in ('.ros_log/ros_live_20260922_session/start_ros.sh', '.ros_log/ros_live_20260922_session/start_camera_sdk.sh',
                '.ros_log/ros_live_20260922_session/resume_modes.launch', '.ros_log/unknown_grasp_20260923/sanstand_storage.sh'):
        p = ROOT / rel
        if p.is_file():
            dependencies.append(dict(describe(p, 'deployment/2026-10-11/historical_root_entries/' + rel),
                restore_to=str(p), category='historical entry only; current no-skin startup supersedes this; do not auto-run'))

    root_overlay = []
    root_status = git(ROOT, 'status', '--porcelain=v1', '-z', '--untracked-files=all').split(b'\0')
    i = 0
    while i < len(root_status):
        row = root_status[i]; i += 1
        if not row:
            continue
        status, rel = row[:2].decode(), os.fsdecode(row[3:])
        old = None
        if 'R' in status or 'C' in status:
            old = os.fsdecode(root_status[i]); i += 1
        if excluded(rel):
            continue
        p = ROOT / rel
        item = dict(path=rel, status=status, former_path=old, root_head=before['root']['head'],
                    restore_policy='Only overlay a separate checkout of the root base commit; never replace WORK current code')
        if p.exists() or p.is_symlink():
            item.update(describe(p, 'deployment/2026-10-11/root_workspace_overlay/' + rel))
            other = WORK / rel
            item['worktree_file_sha256'] = describe(other)['sha256'] if other.exists() or other.is_symlink() else None
        else:
            item['deleted'] = True
        root_overlay.append(item)

    # Local scan emits only path and detector label, never matching secret bytes.
    secret_patterns = {
        'private-key-header': re.compile(rb'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----'),
        'github-token': re.compile(rb'\b(?:gh[pousr]_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{40,})\b'),
        'openai-project-token': re.compile(rb'\bsk-proj-[A-Za-z0-9_-]{30,}\b'),
    }
    findings = []
    for item in source + dependencies + root_overlay:
        p = Path(item.get('source', ''))
        if not p.is_file() or p.is_symlink() or p.stat().st_size > 2_000_000:
            continue
        data = p.read_bytes()
        for label, pattern in secret_patterns.items():
            if pattern.search(data):
                findings.append(dict(source=str(p), detector=label))

    drift = []
    for rel, expected in deployed['source_sha256'].items():
        p = WORK / rel
        actual = describe(p)['sha256'] if p.exists() else None
        if actual != expected:
            drift.append(dict(path=rel, deployment_sha256=expected, current_sha256=actual))
    after = {'work': state(WORK), 'root': state(ROOT)}
    assert before == after, 'HEAD/branch/index changed during read-only inventory'
    OUT.mkdir(parents=True, exist_ok=True)
    def save(name, value):
        path = OUT / name
        with path.open('x') as f:
            json.dump(value, f, ensure_ascii=False, indent=2); f.write('\n')
        return dict(path=str(path), sha256=digest(path.read_bytes()), bytes=path.stat().st_size)
    inventory = save('candidate_source_inventory.json', dict(schema=1, snapshot_final=False,
        files=source, dependencies=dependencies, root_workspace_overlay=root_overlay,
        deleted_source_paths=deleted, exclusions=omit, external_binary_models=weights,
        missing_dependencies=missing, candidate_secret_findings=findings))
    plan = dict(schema=1, status='read_only_plan_not_snapshot', created_ns=time.time_ns(),
        authorized_scope='After the current repair: complete current project source, configuration, required startup tools and issue/resolution documents. Not raw experiment upload.',
        source_upload_authorized=True, publication_state='awaiting_final_fix_not_yet_published',
        commit_created=False, refs_changed=False, uploaded=False,
        source_worktree=str(WORK), remote='origin', remote_url=git(WORK, 'remote', 'get-url', 'origin').decode().strip(),
        proposed_branch='backup/20261011-current-grasp-source', proposed_tag='snapshot-20261011-current-grasp-source',
        proposed_title='10.11日，当前完整源码与补视抓取问题修复记录',
        proposed_parent='b2fdac56a62fbb1b4f34af2125f9d362f80891b0',
        parent_reason='Existing 10.8 backup history; rebuild the new tree from an empty separate index so inherited bags/build output are excluded from this snapshot only.',
        branch_collision_policy='Refuse existing branch/tag; choose a distinct timestamp suffix. Never force-push or replace prior snapshots.',
        workspaces=before, inventory=inventory,
        counts=dict(main_source_files=len(source), main_source_bytes=sum(x['bytes'] for x in source),
            untracked_main_source_files=sum(x['git_state']=='untracked' for x in source),
            explicit_dependency_files=len(dependencies), dependency_bytes=sum(x['bytes'] for x in dependencies),
            root_overlay_files=len(root_overlay), excluded_paths=len(omit), binary_models_excluded=len(weights)),
        deployed_baseline=dict(describe(DEPLOYMENT), source_count=len(deployed['source_sha256']), observed_current_drift=drift),
        issue_document='docs/operations/2026-10-11-code-snapshot-and-grasp-issues.md',
        operation_document='docs/operations/2026-10-08-full-latest-ros-startup.md',
        required_pre_snapshot_refresh=[
            'Wait for root to finish the current repair/integration and update the 10.11 issue document accurately; do not label supplemental grasp successful without physical evidence.',
            'Rerun this inventory under a new output filename/directory and bind the final deployment and reviewed repair files, including newly untracked source/tests.',
            'Add final GEN19 source/review deployment metadata and any new runtime launcher source dependencies explicitly. Copy only compact verification summaries, not raw experiment logs.',
            'Re-hash every included source after reading; retry inventory on change. Preserve both HEADs, branches and actual index hashes.',
            'Scan the final allowlisted publish tree for secrets; resolve findings without printing secret values. This limited plan scan is not a complete secret audit.',
        ],
        source_tree_assembly=[
            'Use a new temporary GIT_INDEX_FILE outside either actual index; git read-tree --empty, never git add -A on the real index.',
            'Populate only reviewed inventory entries using git hash-object -w --stdin and git update-index -z --index-info, preserving 100644/100755/120000 modes and symlink text. Sources remain untouched.',
            'Map explicit runtime/config dependencies to deployment/2026-10-11 and create source_dependencies.json with exact original paths, destinations, mode, size and SHA256. Restore instructions must copy them back before building/launching.',
            'Add root_workspace_overlay bytes and tombstones plus ROOT HEAD in a separate directory, with a README that forbids applying old ROOT code over the current WORK source.',
            'Generate an included-file SHA256 manifest and restore README. Exclude the manifest itself from its recursive content hash.',
            'Inspect the temporary index: no bag/active/rawlog/build/devel/cache/credential/binary-weight entries and no source file omitted merely because it was untracked or ignored.',
            'Create tree/commit with commit-tree only after root closes the fix; create a new dated backup ref and annotated tag with compare-and-create semantics. Do not checkout/reset/stash either worktree.',
            'Push only the new branch/tag to the verified origin and verify remote IDs before reporting upload complete. Do not push master or the dirty live working branch.',
        ],
        restore_outline=[
            'Clone the fixed dated tag into a fresh directory, verify the source and dependency manifests.',
            'Recover local/existing historical binary models to listed paths and install pinned external ROS/RealSense/MuJoCo/GraspNet dependencies; no claim that model binaries or CUDA/WSL environment were uploaded.',
            'Restore explicit ignored runtime and external calibration/config files to reviewed paths; absolute model/mesh/source-contract paths and pin hashes need coordinated relocation on another machine.',
            'Build the ROS catkin workspace with the recorded Noetic/Python environment. Use current full_latest_no_skin startup, never historical tactile=true entry by default.',
            'Historical compact pytest fixtures under tests are retained; test records do not prove future robot motion or successful grasp.',
        ],
        preservation=dict(no_ros_calls=True, no_robot_commands=True, no_temperature_queries=True,
            original_indices_unchanged=True, original_refs_unchanged=True,
            experiment_archive_policy='local_only', upload_enabled=False, delete_enabled=False,
            external_originals_not_deleted=True,
            root_driver_disk_status='No tracked or untracked source change in active driver file relative to ROOT HEAD at inventory time.',
            editor_limit='Unsaved VS Code buffers are not filesystem contents; cannot claim they are backed up.'),
        limitations=['This is a candidate inventory while GEN19 repair is in progress, not a final immutable source snapshot.',
            'Local origin-tracking refs are not a fresh remote verification; no network operation was performed.',
            'Existing historical repository objects may already contain bags; this plan excludes them from the new snapshot tree and does not rewrite old history.',
            'No clean-machine rebuild or launch was performed.',
            'Existing deployment/2026-10-08 contents are historical recovery material, not the current runtime authority.'],
        candidate_secret_findings=findings, missing_dependencies=missing,
        historical_references=['docs/operations/2026-10-08-source-snapshot.md', '.runtime/prepare_source_snapshot_20261008.py', '.runtime/finalize_source_snapshot_20261008.py'])
    result = save('plan.json', plan)
    print(json.dumps(dict(plan=result, counts=plan['counts'], missing_dependencies=missing,
                         candidate_secret_findings=findings, deployment_drift_count=len(drift)), ensure_ascii=False))


if __name__ == '__main__':
    main()
