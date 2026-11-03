#!/usr/bin/env python3
"""Finish documentation and local-only storage maintenance after verification."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

WORK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORK / 'tools'))
from minimal_experiment_trace import TraceStore, atomic, sha_file

ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
RUN = ROOT / 'maintenance/minimal_trace_cleanup_1791956657642651189'
GIB = 1024 ** 3


def main():
    verified = json.loads((RUN / 'final_verification.json').read_text())
    assert verified['state'] == 'verified'
    if (RUN / 'final_result.json').exists():
        print((RUN / 'final_result.json').read_text())
        return
    plan = json.loads((RUN / 'plan.json').read_text())
    archive = json.loads((ROOT / 'grasp_records/archive_config.json').read_text())
    assert archive['upload_enabled'] is False and archive['delete_enabled'] is False
    assert archive['archive_policy'] == 'local_only'
    policy_path = ROOT / 'maintenance/failed_record_space_guard_policy.json'
    policy = json.loads(policy_path.read_text())
    assert policy['schema'] == 3
    if policy['enabled']:
        assert policy['latest_minimal_trace_run'] == str(RUN)
        assert policy['auto_catalog_pre_motion_failures'] is False
        assert sha_file(policy_path) == sha_file(RUN / 'policy_after_resume.json')
    else:
        if not (RUN / 'policy_before_resume.json').exists():
            atomic(RUN / 'policy_before_resume.json', policy)
        policy.update(enabled=True, auto_catalog_pre_motion_failures=False,
                      retention_profile='minimal_trace_local',
                      authorization=plan['authorization_text'],
                      latest_minimal_trace_run=str(RUN),
                      problem_index=str(ROOT / '文档/问题回溯索引.md'),
                      automatic_catalog_scope='Disabled; only separately reviewed finite records are eligible.',
                      updated_ns=time.time_ns())
        policy.pop('user_hold', None)
        policy['authorization_source'] = dict(source='actual_user_message', platform_message_id=None,
                                              text=plan['authorization_text'], clarification='已经迁移的就不管了')
        policy['metadata_github']['scope'] = 'No further GitHub contact; existing migrated material unchanged. Separate reviewed local minimal-trace workflow only.'
        policy['minimal_trace_validation'] = dict(result=str(RUN / 'validation_r3/result.json'),
                                                   sha256=sha_file(RUN / 'validation_r3/result.json'))
        atomic(RUN / 'policy_after_resume.json', policy)
        atomic(policy_path, policy)
    # Inspect the old finite catalog without invoking its deletion branch.
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONNOUSERSITE='1')
    guard = subprocess.run([sys.executable, str(WORK / 'tools/maintain_failed_record_space.py')],
                           cwd=str(WORK), env=env, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    atomic(RUN / 'guard_resume_check.json', dict(returncode=guard.returncode, stdout=guard.stdout, stderr=guard.stderr))
    assert guard.returncode == 0
    guard_result = json.loads(guard.stdout)
    assert (guard_result['state'] == 'above_trigger' or
            (guard_result['state'] == 'reviewed_plan' and guard_result['candidate_records'] == 0))
    units = ['alicia-failed-record-space.timer', 'alicia-failed-record-space.service',
             'alicia-grasp-archive.service', 'alicia-grasp-archive-unknown.service',
             'alicia-archive-finish-20260927.service']
    states = {}
    for unit in units:
        text = subprocess.check_output(['systemctl', '--user', 'show', unit, '-p', 'ActiveState',
                                        '-p', 'SubState', '-p', 'UnitFileState'], text=True)
        states[unit] = dict(line.split('=', 1) for line in text.splitlines() if '=' in line)
    assert all(states[u]['ActiveState'] == 'inactive' for u in units[2:])
    total = verified['totals']
    synthetic = json.loads((RUN / 'synthetic_fixture_cleanup.json').read_text())
    assert synthetic['state'] == 'complete' and synthetic['validation_logs_and_results_unchanged']
    assert all(item['state'] == 'deleted' and not (RUN / item['path']).exists() for item in synthetic['files'])
    synthetic_summary = {k: v for k, v in synthetic.items() if k != 'files'}
    text = ('\n## 本次最小化清理结果（2026-10-14 UTC）\n\n'
            '执行完成并核验：无损压缩后移除 **{text_deleted:,}** 份分散文本原件；'
            '移除 **{raw_deleted}** 份已封口大录制，完整非图像ROS消息及首尾视觉样本留在回溯库中。'
            '原件分配空间合计 **{freed:.3f} GiB**，其中录制 **{raw:.3f} GiB**、文本 **{text:.3f} GiB**；'
            '压缩库占用约 **{db:.3f} GiB**。核验时外盘可用 **{free:.3f} GiB**，停止原因为 `{reason}`。\n\n'
            '**{protected}** 份受保护文件的设备、inode、大小与修改时间全部一致；'
            '所有实际删除均有原路径、原SHA256及留存位置的持久回执，无未决删除意图。'
            '18项专项验证及独立复核通过，原结果保持真实，软件修复不能据此记为实物抓取成功。\n\n'
            '精确依据：`maintenance/{run}/receipt.json`、`final_verification.json`、'
            '`final_result.json`；逐文件删除记录在同目录 `trace.sqlite3` 的 `deletions` 表。'
            '已迁移材料保持现状，没有新增GitHub联系。低空间定时器恢复，自动扩充名单关闭，'
            '低于10 GiB触发、80 GiB目标不变；通用上传及删除开关仍关闭。\n').format(
                **total, freed=total['deleted_allocated_bytes']/GIB,
                raw=verified['raw_allocated_bytes']/GIB, text=verified['text_allocated_bytes']/GIB,
                db=verified['retained_trace']['database_bytes']/GIB,
                free=verified['actual_free_bytes']/GIB, reason=verified['stop_reason'],
                protected=verified['protected_checked'], run=RUN.name)
    text += ('\n另清理本轮三次已结束验证的115个合成临时文件，分配空间约14.375 MiB；'
             '测试源码、日志、XML和结果均保留，核验未变。逐文件回执为同目录 '
             '`synthetic_fixture_cleanup.json`，不计入上面的实验原件删除统计。\n')
    docs = [WORK / 'docs/operations/2026-10-13-minimal-experiment-history.md',
            WORK / 'docs/operations/experiment-storage-sanstand.md']
    for path in docs:
        if '## 本次最小化清理结果（2026-10-14 UTC）' not in path.read_text():
            with path.open('a') as f:
                f.write(text)
    mirrors = [(docs[0], ROOT / '文档/问题回溯索引.md'),
               (docs[1], ROOT / '文档/experiment-storage-sanstand.md'),
               (docs[1], ROOT / '文档/operations/experiment-storage-sanstand.md'),
               (Path('/home/zhuyupei/alicia_wa_full/AGENTS.md'), ROOT / '文档/AGENTS.md')]
    for source, target in mirrors:
        shutil.copyfile(str(source), str(target))
        assert sha_file(source) == sha_file(target)
    store = TraceStore(RUN / 'trace.sqlite3')
    try:
        completion_sources = {}
        for name in ['.runtime/verify_minimal_trace_cleanup_20261014.py',
                     '.runtime/finalize_minimal_trace_cleanup_20261014.py']:
            path = WORK / name
            completion_sources[name] = sha_file(path)
            store.add_stream('@project/' + name, [path.read_bytes()],
                             dict(kind='completion_audit_source', sha256=completion_sources[name]))
        for path in docs:
            key = '@project/' + str(path.relative_to(WORK))
            try:
                old = store.info(key)
            except KeyError:
                old = None
            history = '@history/before_final_receipt/' + key
            try:
                store.info(history)
            except KeyError:
                if old is not None:
                    store.add_stream(history, store.iter_bytes(key), old['metadata'])
            store.add_stream(key, [path.read_bytes()], dict(kind='final_cleanup_document', sha256=sha_file(path)))
        knowledge_path = RUN / 'knowledge_index.json'
        knowledge = json.loads(knowledge_path.read_text())
        if not (RUN / 'knowledge_index_initial.json').exists():
            atomic(RUN / 'knowledge_index_initial.json', knowledge)
        by_path = {item['path']: item for item in knowledge['source_documents']}
        for path in docs:
            key = '@project/' + str(path.relative_to(WORK))
            by_path[key] = dict(path=key, title=path.read_text().splitlines()[0], sha256=sha_file(path))
        knowledge['source_documents'] = list(by_path.values())
        index_key = '@index/source_documents.json'
        history_key = '@history/before_final_receipt/' + index_key
        try:
            store.info(history_key)
        except KeyError:
            old = store.info(index_key)
            store.add_stream(history_key, store.iter_bytes(index_key), old['metadata'])
        store.add_stream(index_key, [json.dumps(knowledge['source_documents'], ensure_ascii=False).encode()],
                         dict(kind='updated_document_index'))
        knowledge['store_stats'] = store.stats()
        knowledge['updated_ns'] = time.time_ns()
        knowledge['validated_cleanup_source'] = json.loads((RUN / 'validation_r3/result.json').read_text())['source_sha256']
        atomic(knowledge_path, knowledge)
        stats = store.stats()
    finally:
        store.close()
    subprocess.run(['systemctl', '--user', 'start', 'alicia-failed-record-space.timer'], check=True)
    text = subprocess.check_output(['systemctl', '--user', 'show', units[0], '-p', 'ActiveState',
                                    '-p', 'SubState', '-p', 'UnitFileState'], text=True)
    states[units[0]] = dict(line.split('=', 1) for line in text.splitlines() if '=' in line)
    assert states[units[0]]['ActiveState'] == 'active'
    values = os.statvfs(ROOT)
    result = dict(state='complete', verification=str(RUN / 'final_verification.json'),
                  totals=total, trace=stats, free_bytes=values.f_bavail * values.f_frsize,
                  synthetic_fixture_cleanup=synthetic_summary,
                  completion_source_sha256=completion_sources,
                  stop_reason=verified['stop_reason'], service_states=states,
                  policy_sha256=sha_file(policy_path), auto_catalog_pre_motion_failures=False,
                  documentation=[dict(source=str(s), mirror=str(t), sha256=sha_file(s)) for s,t in mirrors],
                  github_contacted=False, robot_nodes_restarted=False,
                  motion_commands_sent=False, temperature_queried=False, completed_ns=time.time_ns())
    atomic(RUN / 'final_result.json', result)
    print(json.dumps(result, ensure_ascii=False))


if __name__ == '__main__':
    main()
