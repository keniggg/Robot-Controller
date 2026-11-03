from pathlib import Path
import datetime
import gzip
import hashlib
import json
import os
import shutil

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
S = E / 'ros_log/full_latest_gen41_restart_20261018T011222Z'
G = S / 'gen57_resume_r3/actual_attempt_r1_geometry'
O = G / 'missing_depth_prior_fix_r1/final_documentation_r1'
assert os.path.ismount('/media/zhuyupei/SanStand'), 'SanStand unavailable; no fallback'
assert not (O / 'receipt.json').exists(), 'Already finalized'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, data):
    temp = path.with_name(path.name + '.missing_depth_doc_tmp')
    with temp.open('xb') as f:
        f.write(data)
        f.flush()
        os.fsync(f.fileno())
    assert temp.read_bytes() == data
    os.replace(temp, path)
    assert path.read_bytes() == data


def save(name, value):
    write(O / name, (json.dumps(value, ensure_ascii=False, indent=2) + '\n').encode())


before = json.loads((O / 'before.json').read_text())
resume = json.loads((O / 'partial_attempt_1.json').read_text())['verified_resume_targets'] if (O / 'partial_attempt_1.json').exists() else {}
for row in before['rows']:
    if row['existed']:
        raw = gzip.decompress(Path(row['backup']).read_bytes())
        assert hashlib.sha256(raw).hexdigest() == row['sha256']
    if row['path'].startswith(str(E)):
        p = Path(row['path'])
        if str(p) in resume:
            assert p.exists() and sha(p) == resume[str(p)], 'Partial write changed before resuming'
        elif p.exists():
            assert row['existed']
            assert sha(p) == row['sha256'], 'Document changed since backup'
        else:
            assert not row['existed']

assert sha(W / 'src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_instance.py') == '3d400dd2e17f3b1ac6a2078e85c9a4b9a855d4c7402b0bc4aca291312b467b0a'
expected = {
    S / 'verification/missing_depth_prior_full_r2/result.json': '8a332657019ad0790a53e9a71bd5459cafd51d956b759dcfe00a89fdd7a477a9',
    S / 'verification/missing_depth_prior_full_r2/independent_verification.json': '1800bacf740ba45208966296de430ba663764e8ccf439c5bd11b04b8c996451e',
    G / 'owner_handoff_r8/result.json': '650c95a8a025fcf927bc05911c661b48329166fcd312d0cd906cf47d734f2f3a',
    G / 'owner_handoff_r8/syscall_audit.json': 'e8b03290a9f625e1c7f0129e46bc08f495818686d6979a0d1b3ec3bf42bde372',
}
for path, digest in expected.items():
    assert sha(path) == digest

entry = '''

### 2026-10-20 GEN57 到位掩码缺口：源码修复与原配置离线交接通过

问题原因：目标的零深度区域超过测量种子膨胀范围后被初始化为 RGB probable background，污染背景颜色模型并遗漏可见轮廓；单改空间裁剪或可靠种子不足以解决。修复只在当前有界且已观测的 RGB 域将零/NaN 深度的软背景改为软前景，原硬核、实测背景、几何/身份/时效门保持。两原拒绝帧形状距离降至 0.180281/0.180572，通过原 0.4 门。

重点 280/280、完整 9,893/9,893 通过（六 worker，374.495 秒，精确身份与来源核验通过）。还原全部 909 项原参数并精确重现原 source_id 后，真实三方回调完成三帧资格确认、同源双回执、任务交接与第四帧跟踪；contact_replan 消息仅收集，未执行后续远端预测/注册、接触规划或实物动作。虚拟时序不等于在线期限验证。本轮未重启节点加载源码，回归及回放进程已退出；历史实际记录仍为失败，补偿碰撞、接触实物交接与最终夹持仍未闭环。

详见 [GEN57 缺深度 RGB 初始化验证](2026-10-20-gen57-missing-depth-rgb-prior.md) 及 [操作记录](2026-10-16-minimal-observation-chain.md)。证据基目录为 `../ros_log/full_latest_gen41_restart_20261018T011222Z/`，含 `verification/missing_depth_prior_*` 和 `gen57_resume_r3/actual_attempt_r1_geometry/owner_handoff_r8/`；精确 SHA、旧文档无损备份及镜像回执位于后者同级 `missing_depth_prior_fix_r1/final_documentation_r1/`。所有本轮记录本地保存在 SanStand，未上传或清理。
'''
index = E / '文档/问题回溯索引.md'
index_before = gzip.decompress((O / 'before_4.gz').read_bytes())
assert index.read_bytes() in (index_before, index_before + entry.encode())
write(index, index_before + entry.encode())
mirrors = []
for relative in ['docs/verification/2026-10-20-gen57-missing-depth-rgb-prior.md', 'docs/operations/2026-10-16-minimal-observation-chain.md']:
    source = W / relative
    target = E / '文档' / source.name
    write(target, source.read_bytes())
    mirrors.append({'source': str(source), 'target': str(target), 'sha256': sha(source), 'readback_equal': sha(source) == sha(target)})

needles = ('replay_gen57_', 'probe_gen57_', 'inspect_gen57_', 'extract_rgbd_inputs', 'run_recorded_grasp', 'gen57_cold_attempt_entry', 'gen57_attempt_qualification_replay', 'verify_missing_depth_prior_', 'replay_masks')
remaining = []
for p in Path('/proc').iterdir():
    if not p.name.isdigit() or int(p.name) == os.getpid():
        continue
    try:
        args = [v.decode(errors='replace') for v in (p / 'cmdline').read_bytes().split(b'\0') if v]
        if not args:
            continue
        exe = Path(args[0]).name
        scripts = [Path(x).name for x in args[1:] if x.endswith('.py')]
        matches = (exe.startswith('python') and any(any(n in s for n in needles) for s in scripts)) or exe in ('pytest', 'py.test', 'rosbag', 'record')
        if matches:
            stat = (p / 'stat').read_text().rsplit(')', 1)[1].split()
            remaining.append({'pid': int(p.name), 'start_ticks': int(stat[19]), 'executable': exe, 'scripts': scripts})
    except (OSError, ValueError):
        continue
save('process_scan.json', {'recorded_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(), 'scope': 'Read-only proc scan for named offline/attempt scripts and pytest/rosbag/record executables; not a live ROS task-state query.', 'remaining': remaining, 'signals_sent': [], 'services_stopped': []})

files = list(expected) + [
    W / 'src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_instance.py',
    W / 'src/alicia_grasp_modes/tests/test_missing_depth_rgb_prior.py',
    W / 'src/alicia_grasp_modes/tests/test_observation_arrival_probable_foreground.py',
    W / '.runtime/replay_gen57_arrival_chain_20261020.py',
    W / '.runtime/replay_gen57_owner_handoff_20261020.py',
    Path(__file__).resolve(), index, O / 'before.json', O / 'process_scan.json',
    S / 'verification/missing_depth_prior_freeze_r2/manifest_closed.json',
    S / 'verification/missing_depth_prior_full_r2/final_process_receipt.json',
    S / 'verification/missing_depth_prior_focus_r2/result.json',
    G / 'arrival_chain_baseline_r3/result.json',
    G / 'arrival_chain_candidate_r1/result.json',
    G / 'arrival_chain_candidate_reconstructed_r1/result.json',
    G / 'historical_parameter_reconstruction_r1/attempt_2/all_resolved_parameters.json',
    G / 'missing_depth_prior_fix_r1/diagnosis_evidence_r1.json',
    G / 'missing_depth_prior_fix_r1/root_full_image_comparison_r2.json',
    G / 'missing_depth_prior_fix_r1/test_contract_review_r1.json',
    G / 'missing_depth_prior_fix_r1/review_r1.json',
]
files += [Path(row[k]) for row in mirrors for k in ('source', 'target')]
save('evidence_map.json', {'files': [{'path': str(p), 'size': p.stat().st_size, 'sha256': sha(p)} for p in files]})
usage = shutil.disk_usage(E)
receipt = {'recorded_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(), 'status': 'offline_fix_and_documentation_complete', 'mirrors': mirrors, 'index_sha256': sha(index), 'evidence_map_sha256': sha(O / 'evidence_map.json'), 'sanstand_mounted': True, 'sanstand_free_gib': usage.free / 2**30, 'no_matching_offline_or_attempt_processes': not remaining, 'production_source_sha256': sha(files[4]), 'full_regression_cases_passed': 9893, 'exact_historical_context_handoff_passed': True, 'node_restart_performed': False, 'new_automatic_attempt_started': False, 'temperature_queried': False, 'disable_command_sent': False, 'uploaded': False, 'experimental_data_deleted': False, 'physical_grasp_verified': False, 'outstanding': ['New-source runtime acceptance with current instance/config-bound remote health and real feedback', 'Remote prediction/registration and contact planning beyond replay message collector', 'Historical compensated collision diagnosis and contact execution/grasp closure']}
save('receipt.json', receipt)
print(json.dumps({'status': receipt['status'], 'mirrors_verified': len(mirrors), 'evidence_files': len(files), 'remaining_processes': remaining, 'sanstand_free_gib': receipt['sanstand_free_gib'], 'receipt': str(O / 'receipt.json')}, ensure_ascii=False))
