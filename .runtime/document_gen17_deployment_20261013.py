"""Mirror verified deployment notes; no ROS or experiment outcome changes."""
from pathlib import Path
import hashlib
import json
import time

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N = E / 'ros_log/full_latest_contact_admission_20261013T011103Z'
O = N / 'verification/gen17_deployed_document_update'
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def save(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write('\n')


assert Path('/media/zhuyupei/SanStand').is_mount() and not O.exists()
dep_path = N / 'gen17_deployment.json'
dep = read(dep_path)
assert dep['deployment_passed'] and dep['whole_patch_deployed']
release_path = Path(dep['source_manifest'])
assert sha(release_path) == dep['source_manifest_sha256']
release = read(release_path)
assert release['passed'] and release['installed'] and release['reviewed']
tests_path = Path(release['actual_regression'])
tests = read(tests_path)
assert tests['passed'] and tests['production_worktree_tested'] and tests['source_unchanged']
assert tests['source_sha256'] == release['source_sha256'] == dep['source_sha256']
assert all(sha(W / name) == digest for name, digest in release['source_sha256'].items())
summary = tests['summary']
assert summary['tests'] > 0 and all(summary[k] == 0 for k in ('failures', 'errors', 'skipped'))
count = summary['tests']
scope = len(release['test_scope'])
sources = len(release['source_sha256'])
paragraph = (f'\n实际工作树组合回归为 {count} 项全部通过，{scope} 个测试文件，'
             f'{sources} 份来源在测试和部署核验中一致。三个计算节点已重新载入，'
             '八个硬件／相机／上位机相关进程保持原身份。'
             '回执分别见 `verification/gen17_station_admission_actual_integrated/result.json`、'
             '`verification/gen17_station_admission_release_manifest.json` 和 `gen17_deployment.json`。'
             '此处确认的是软件验证与部署；第5轮仍为已封口失败，尚未新增实物成功结论。\n')
gate = W / 'docs/operations/2026-10-13-grasp-gate-rationality.md'
acceptance = W / 'docs/operations/2026-10-11-frozen-grasp-acceptance.md'
heading = '## GEN17：将已登记测量过期恢复为等待，并检查后续交接入口'
assert heading not in gate.read_text()
assert '## GEN17 组合修复已部署，等待新的实物验收' not in acceptance.read_text()
updates = {
    gate: gate.read_text().replace(
        '当前任务侧针对性165项通过，组合验证、运行部署及新实物验收仍待完成。',
        '该记录时任务侧针对性165项通过；后续组合验证与部署见下节，实物验收另行记录。')
        + '\n' + (W / '.runtime/gen17_gate_followup_draft_20261013.md').read_text() + paragraph,
    acceptance: acceptance.read_text() + '\n## GEN17 组合修复已部署，等待新的实物验收\n\n'
        + '已登记测量复验、最终冻结前提交、远端回执出具的超窗改为在原合同内等待新测量；'
        + '补齐登记时钟配对、copy／状态通知原子检查，以及冻结后精确站位退役的历史审计交接。'
        + '两个退役消息顺序的真实三参与方反例均保留；历史证明不恢复几何或动作权限。'
        + '详见同目录 `2026-10-13-grasp-gate-rationality.md`。\n' + paragraph,
}
O.mkdir()
rows = []
for index, (source, content) in enumerate(updates.items()):
    targets = [source, E / '文档' / source.name, N / source.name]
    for target_index, target in enumerate(targets):
        assert target.exists()
        before = target.read_bytes()
        (O / f'before_{index}_{target_index}.md').write_bytes(before)
        assert target.read_bytes() == before
        target.write_text(content)
    assert len({sha(target) for target in targets}) == 1
    rows.append(dict(source=str(source), sha256=sha(source), mirrors=[str(p) for p in targets[1:]]))
save(O / 'result.json', dict(passed=True, documents=rows,
    evidence_sha256={str(p): sha(p) for p in (dep_path, release_path, tests_path, Path(__file__))},
    recorded_ns=time.time_ns(), physical_grasp_verified=False, motion_commands_sent=False))
print(json.dumps(dict(passed=True, documents=len(rows), tests=count, sources=sources)))
