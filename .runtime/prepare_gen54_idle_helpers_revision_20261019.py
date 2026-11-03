#!/usr/bin/env python3
"""Seal the revised GEN54 drafts without importing or executing them."""
import ast
import difflib
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import time
import xml.etree.ElementTree as ET

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
M = Path('/media/zhuyupei/SanStand')
V = M/'实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
PREP = V/'gen54_idle_helpers_preparation_r1/result.json'
OUT = V/'gen54_idle_helpers_preparation_r2'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def assignment(path, name):
    text = path.read_text()
    nodes = [node for node in ast.parse(text).body if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == name for target in node.targets)]
    assert len(nodes) == 1
    return text, nodes[0]


def main():
    assert M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY
    assert OUT.parent.stat().st_dev == M.stat().st_dev and not OUT.exists()
    assert sha(PREP) == '8d2f48aa181524e8fc20d64d00c7a350ee66dd71d9f6de322f5a11e92b7bd5da'
    before = json.loads(PREP.read_text())
    for row in before['files']:
        assert sha(row['template']) == row['template_sha256']
    pins = {name: sha(name) for name in before['helper_sha256']}
    finalizer = W/'.runtime/finalize_gen54_idle_deployment_20261018.py'
    assert sha(finalizer) == next(row['sha256'] for row in before['files'] if row['path'] == str(finalizer))
    text, node = assignment(finalizer, 'HELPER_SHA')
    old_assignment = ast.get_source_segment(text, node)
    assert text.count(old_assignment) == 1
    finalizer.write_text(text.replace(old_assignment, 'HELPER_SHA = '+repr(pins)))
    texts = {row['path']: Path(row['path']).read_text() for row in before['files']}
    for name, text in texts.items():
        if name.endswith('.py'):
            compile(text, name, 'exec')
        elif name.endswith('.launch'):
            ET.fromstring(text)
    subprocess.run(['bash', '-n', str(W/'.runtime/start_gen54_observation_measurement_consumers_20261018.sh')], check=True)
    _, helper_node = assignment(finalizer, 'HELPER_SHA')
    assert ast.literal_eval(helper_node.value) == pins
    for name in ('install_gen54_validated_candidate_20261018.py', 'seal_gen54_release_20261018.py'):
        _, node = assignment(W/'.runtime'/name, 'EXPECTED_NEW_TEST_FILES')
        assert ast.literal_eval(node.value) is None
    _, node = assignment(W/'.runtime/seal_gen54_release_20261018.py', 'SCOPE_EVIDENCE')
    assert ast.literal_eval(node.value) is None
    supplemental = None
    for name in ('gen54_idle_reload_common_20261018.py', 'install_gen54_validated_candidate_20261018.py'):
        _, node = assignment(W/'.runtime'/name, 'SUPPLEMENTAL_TESTS')
        assert isinstance(node.value, ast.Call) and node.value.func.id == 'frozenset'
        value = set(ast.literal_eval(node.value.args[0]))
        assert len(value) == 4 and (supplemental is None or value == supplemental)
        supplemental = value
    baseline_path = W/'.runtime/gen54_baseline_source/runner_manifest.json'
    baseline = json.loads(baseline_path.read_text())
    assert sha(baseline_path) == 'e0ea9529ff6afdff8a6794578f1e64d996cdb124ee1adf3e6732e38e44984748'
    assert supplemental <= set(baseline['source_sha256']) and not supplemental & set(baseline['test_scope'])
    assert all(sha(W/path) == baseline['source_sha256'][path]
        and sha(baseline_path.parent/path) == baseline['source_sha256'][path] for path in supplemental)
    assert all(sha(row['template']) == row['template_sha256'] for row in before['files'])
    assert all(sha(path) == digest for path, digest in before['dependency_sha256'].items())
    rows = [dict(row, sha256=sha(row['path']), previous_draft_sha256=row['sha256']) for row in before['files']]
    changed = [row['path'] for row in rows if row['sha256'] != row['previous_draft_sha256']]
    assert {Path(name).name for name in changed} == {
        'gen54_idle_reload_common_20261018.py', 'install_gen54_validated_candidate_20261018.py',
        'seal_gen54_release_20261018.py', 'finalize_gen54_idle_deployment_20261018.py'}
    known_sha = set(before['dependency_sha256'].values()) | set(pins.values())
    known_sha.add(before['expected_control_sha256'])
    for row in before['files']:
        known_sha.update(re.findall(r'\b[0-9a-f]{64}\b', Path(row['template']).read_text()))
    new_sha = {digest for text in texts.values() for digest in re.findall(r'\b[0-9a-f]{64}\b', text)}
    assert new_sha <= known_sha, sorted(new_sha-known_sha)
    OUT.mkdir()
    for row in rows:
        path, template = Path(row['path']), Path(row['template'])
        diff = ''.join(difflib.unified_diff(template.read_text().splitlines(True), texts[str(path)].splitlines(True),
            fromfile=str(template), tofile=str(path)))
        (OUT/(path.name+'.diff')).write_text(diff)
        (OUT/path.name).write_bytes(path.read_bytes())
    result = dict(before, kind='GEN54_idle_helpers_supplemental_scope_preparation',
        files=rows, helper_sha256=pins,
        previous_preparation=str(PREP), previous_preparation_sha256=sha(PREP),
        supplemental_existing_tests=sorted(supplemental),
        supplemental_existing_tests_source_unchanged=True,
        test_scope_formula='322 historical files + 4 supplemental existing files + collected new files',
        test_count_formula='9010 historical cases + collected supplemental cases + collected new cases',
        supplemental_case_count_guessed=False,
        added_fixture_suffixes=['.json', '.json.gz'],
        changed_since_first_preparation=changed,
        preparation_helper=str(Path(__file__).resolve()), preparation_helper_sha256=sha(__file__),
        created_ns=time.time_ns())
    with (OUT/'result.json').open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    (OUT/'summary.md').write_text(
        'GEN54 运维草稿已补入四份原有几何测试的强制精确范围。它们属于冻结的 1,069 份源文件，但未包含于历史 322 文件 / 9,010 用例；当前项目与基线字节一致。\n\n'
        '安装前要求最终清单 full_scope_ready、四份补充测试及所有新增测试的受隔离收集证据与精确用例身份；封口文件数为 322 + 4 + 新增文件数，用例数为 9,010 + 补充收集数 + 新增收集数。新 fixture 允许命名限定的 .json / .json.gz，仍不能替换既有 fixture 或引入代码。\n\n'
        'common 的补充测试检查已重新绑定到 finalizer。原 GEN53 九份模板和第一次准备记录保持不变。当前九份草稿快照及相对 GEN53 的完整差异保存在本目录；只完成语法/AST/文件身份核对，未导入或运行运维入口、未联系 ROS、未修改生产源文件。\n\n'
        'EXPECTED_NEW_TEST_FILES 和 SCOPE_EVIDENCE 仍为 None，阻止安装和封口。最终收集清单及失败复核证据固定后须绑定并由 root 独立审查。\n')
    print(json.dumps(dict(output=str(OUT), result_sha256=sha(OUT/'result.json'),
        prepared=True, ready_for_execution=False, helper_count=len(rows)), ensure_ascii=False))


if __name__ == '__main__':
    main()
