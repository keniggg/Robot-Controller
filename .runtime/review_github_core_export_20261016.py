"""Independent, offline review of the exported immutable Release text tree."""
from pathlib import Path, PurePosixPath
from collections import Counter
import hashlib
import json
import os
import re
import stat

W = Path(__file__).resolve().parents[1]
S = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/maintenance/source_snapshot_20261016_unfinished_supplemental_view')
C = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen29_restart_20261016T000728Z')
EXPORT = W / '.runtime/github_core_export_20261016_unfinished'


def sha(data):
    return hashlib.sha256(data).hexdigest()


assert Path('/media/zhuyupei/SanStand').is_mount()
original_bytes = (S / 'github_readback/source-manifest.json').read_bytes()
assert sha(original_bytes) == '55ec5e794bfc2776297cc394d412e9e9db2ff94583f568e21a672ac6d072b97c'
original = json.loads(original_bytes)
by_archive = {v['archive_path']: v for v in original['files']}
manifest_path = EXPORT / 'snapshot-export-manifest.json'
manifest_bytes = manifest_path.read_bytes()
manifest = json.loads(manifest_bytes)
findings = []
verified = {}
patterns = {
    'private_key': rb'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----',
    'github_token': rb'\b(?:gh[pousr]_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{40,})\b',
    'openai_token': rb'\bsk-(?:proj-)?[A-Za-z0-9_-]{30,}\b',
    'aws_access_key': rb'\b(?:AKIA|ASIA)[A-Z0-9]{16}\b',
}
credential_names = {'.env', 'credentials', 'credentials.json', 'token.json', 'auth.json',
                    'id_rsa', 'id_ed25519', '.netrc', '.git-credentials'}
bad_assets = {'.pt', '.pth', '.onnx', '.engine', '.stl', '.obj', '.npz', '.npy', '.bag',
              '.png', '.jpg', '.jpeg', '.so', '.dll', '.pkl', '.zip', '.gz', '.mp4'}
mapped = set()
for row in manifest['files']:
    rel = row['branch_path']
    path = PurePosixPath(rel)
    if path.is_absolute() or '..' in path.parts or rel in verified:
        findings.append(dict(kind='unsafe_or_duplicate_path', path=rel))
        continue
    expected_archive = ('ide_root_workspace/' + rel[len('ide_root_workspace/'):]
        if rel.startswith('ide_root_workspace/') else rel
        if rel.startswith('external_dependencies/') else 'current_runtime_workspace/' + rel)
    if row['archive_path'] != expected_archive:
        findings.append(dict(kind='workspace_mapping_changed', path=rel))
    source = by_archive.get(row['archive_path'])
    if source is None or any(row[k] != source[k] for k in ('bytes', 'sha256')):
        findings.append(dict(kind='original_manifest_mismatch', path=rel))
        continue
    target = EXPORT / rel
    if target.is_symlink():
        data = os.readlink(target).encode()
        if os.readlink(target) != source.get('symlink_target'):
            findings.append(dict(kind='symlink_target_changed', path=rel))
    else:
        data = target.read_bytes()
        if 'symlink_target' in source:
            findings.append(dict(kind='symlink_materialized', path=rel))
    actual = sha(data)
    if actual != row['sha256'] or len(data) != row['bytes']:
        findings.append(dict(kind='payload_mismatch', path=rel))
    if bool(target.lstat().st_mode & 0o111) != bool(source['mode'] & 0o111):
        findings.append(dict(kind='executable_bit_changed', path=rel))
    if path.name.lower() in credential_names or path.suffix.lower() in bad_assets:
        findings.append(dict(kind='excluded_path_or_asset', path=rel))
    if any(part in {'.runtime', 'build', 'devel', '__pycache__', 'logs', '.git'} for part in path.parts):
        findings.append(dict(kind='excluded_generated_tree', path=rel))
    try:
        data.decode('utf-8')
        if b'\x00' in data:
            raise ValueError('NUL byte')
    except (UnicodeError, ValueError):
        findings.append(dict(kind='non_text_payload', path=rel))
    for kind, pattern in patterns.items():
        if re.search(pattern, data):
            findings.append(dict(kind=kind, path=rel))  # Never emit candidate secret bytes.
    verified[rel] = actual
    mapped.add(row['archive_path'])

extra = {'snapshot-export-manifest.json'}
for row in manifest['extra_files']:
    rel = row['path']; extra.add(rel)
    if sha((EXPORT / rel).read_bytes()) != row['sha256']:
        findings.append(dict(kind='generated_metadata_changed', path=rel))
actual_files = {str(p.relative_to(EXPORT)) for p in EXPORT.rglob('*')
                if (p.is_file() or p.is_symlink()) and '.git' not in p.relative_to(EXPORT).parts}
if actual_files != set(verified) | extra:
    findings.append(dict(kind='unmanifested_or_missing_files',
        extra=sorted(actual_files-set(verified)-extra), missing=sorted(set(verified)|extra-actual_files)))

code_suffixes = {'.py', '.cpp', '.hpp', '.h', '.c', '.cmake', '.in', '.sh', '.bash',
                '.launch', '.xml', '.yaml', '.yml', '.msg', '.srv', '.urdf', '.xacro'}
unexplained_core = []
excluded_core = []
for row in original['files']:
    path = PurePosixPath(row['archive_path']); parts = path.parts
    if len(parts) < 3 or parts[0] not in {'current_runtime_workspace', 'ide_root_workspace'}:
        continue
    if parts[1] not in {'src', 'docs', 'tools', 'srv'} or str(path) in mapped:
        continue
    excluded_core.append(str(path))
    if ('logs' in parts or 'data' in parts or 'evidence' in parts
            or any('.bak' in v for v in parts)):
        continue
    if path.suffix.lower() in code_suffixes or path.name in {'CMakeLists.txt', 'package.xml', 'LICENSE', 'COPYING'}:
        unexplained_core.append(str(path))
if unexplained_core:
    findings.append(dict(kind='unexplained_core_code_omissions', paths=unexplained_core))

checks = []
for rel in ('src/alicia_grasp_modes/src/alicia_grasp_modes/silhouette_observation_planning.py',
            'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_remote.py',
            'src/alicia_grasp_modes/tests/test_gen31_observation_search_progress.py',
            'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/regular_box_inference.py'):
    checks.append(dict(path=rel, exported_snapshot_sha256=verified[rel],
        current_worktree_sha256=sha((W / rel).read_bytes()),
        differs_from_current_repair=verified[rel] != sha((W / rel).read_bytes())))
assert manifest_path.read_bytes() == manifest_bytes
for rel, expected in verified.items():
    target = EXPORT / rel
    data = os.readlink(target).encode() if target.is_symlink() else target.read_bytes()
    assert sha(data) == expected
out = C / 'verification/github_core_export_independent_review_r1'
out.mkdir()
report = dict(kind='independent_immutable_release_core_export_review', reviewed=True,
    export=str(EXPORT), source_archive_sha256_independently_rehashed='d7f60fda328a34b60355a65b53e0dea1377c657d7fa278647f5dab13c7ac24ad',
    original_manifest_sha256=sha(original_bytes), export_manifest_sha256=sha(manifest_bytes),
    verified_original_files=len(verified), verified_bytes=manifest['selected_bytes'],
    all_payload_hashes_read_twice_stable=True, blocking_findings=findings,
    unexplained_core_code_omissions=unexplained_core,
    excluded_core_extension_counts=dict(Counter(PurePosixPath(p).suffix.lower() for p in excluded_core)),
    current_repair_distinction=checks,
    scope='Offline filesystem read only; no production edit, test, ROS, Git mutation or remote publication.',
    limitations=['Credential scan is limited to high-confidence patterns and path rules, not a universal absence proof.',
        'Notebook/native dependency/mesh/image/NPZ and historical deployment/runtime/evidence content is omitted; this is a text browsing tree, not a runnable complete release.',
        'Historical original RESTORE describes the complete Release attachment; new README scopes this branch.',
        'Remote branch publication, existing-ref equality and remote blob readback remain root-owned checks.'])
result = out / 'result.json'
result.write_text(json.dumps(report, indent=2, ensure_ascii=False, sort_keys=True)+'\n')
print(json.dumps(dict(path=str(result), sha256=sha(result.read_bytes()),
    verified_original_files=len(verified), findings=findings), ensure_ascii=False))
