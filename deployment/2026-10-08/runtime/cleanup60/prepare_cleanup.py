#!/usr/bin/env python3
"""Freeze the current, independently classified old failed bags; delete nothing."""
import datetime as dt
import json
from pathlib import Path
import cleanup_failed_sanstand as c

LOCAL = Path(__file__).resolve().parent
WORKTREE = LOCAL.parents[1]


def main():
    classification = json.loads((LOCAL / 'classification_review.json').read_text())
    review = json.loads((LOCAL / 'success_conflicts_review.json').read_text())
    protected_ids = set(review['protected_record_ids'])
    docs = {p: p.read_text(errors='replace').splitlines()
            for folder in ('operations', 'verification')
            for p in (WORKTREE / 'docs' / folder).glob('*.md')}
    policies = [c.ROOT / 'maintenance/storage_control.json',
                c.ROOT / 'grasp_records/archive_config.json',
                c.ROOT / 'ros_log/unknown_grasp_20260923/archive_config.json']
    inputs = {str(LOCAL / name): c.digest(LOCAL / name)
              for name in ('classification_review.json', 'success_conflicts_review.json',
                           'cleanup_failed_sanstand.py', 'prepare_cleanup.py')}
    policy_before = {str(p): {'sha256': c.digest(p), 'data': json.loads(p.read_text())}
                     for p in policies}
    (LOCAL / 'policy_before.json').write_text(json.dumps(policy_before, ensure_ascii=False, indent=2)+'\n')
    inputs.update({str(p): info['sha256'] for p, info in policy_before.items()})
    opened = c.open_paths()
    initial = c.free_bytes()
    estimate = initial
    records = []
    eligible = sorted((r for r in classification['records'] if r['eligible_for_cleanup']),
                      key=lambda r: (r['started_utc'], r['path']))
    for row in eligible:
        if estimate >= 60*c.GIB:
            break
        if row['record_id'] in protected_ids or row['protection_reasons']:
            raise RuntimeError('conflicting protection: '+row['record_id'])
        d = c.checked_path(row['path'])
        bags = []
        for bag in row['candidate_bags']:
            if estimate >= 60*c.GIB:
                break
            p = c.checked_path(bag['path'])
            fresh = c.identity(p)
            expected = bag['identity']
            if fresh != dict(size=expected['size_bytes'], allocated=expected['allocated_bytes'],
                             mtime_ns=expected['mtime_ns'], device=expected['device'], inode=expected['inode']):
                raise RuntimeError('classified bag changed: '+str(p))
            if estimate + fresh['allocated'] > 63*c.GIB:
                raise RuntimeError('whole bag would exceed authorized bound')
            bags.append({'path': str(p), 'identity': fresh})
            estimate += fresh['allocated']
        if not bags:
            continue
        deleting = {i['path'] for i in bags}
        keep = [{'path': str(p), 'identity': c.identity(c.checked_path(p))}
                for p in sorted(d.rglob('*')) if p.is_file() and str(p) not in deleting]
        evidence = []
        for item in row['evidence']:
            p = c.checked_path(item['path'])
            if c.digest(p) != item['sha256']:
                raise RuntimeError('failure evidence changed: '+str(p))
            evidence.append(item)
        related = []
        for p, lines in docs.items():
            matches = [i for i, line in enumerate(lines) if row['record_id'] in line]
            if matches:
                inputs[str(p)] = c.digest(p)
                i = matches[0]
                related.append('来源：%s:%d\n%s' % (p, i+1, '\n'.join(lines[max(0,i-2):i+5])[:1500]))
        independent = row.get('independent_success_review', {})
        direct_findings = []
        for item in independent.get('evidence', []) + (independent.get('overlap_review') or {}).get('evidence', []):
            if item.get('path') and item.get('sha256'):
                p = Path(item['path'])
                if c.digest(p) != item['sha256']:
                    raise RuntimeError('operator evidence changed: '+str(p))
                inputs[str(p)] = item['sha256']
                direct_findings.append(item.get('finding', ''))
                # Bind the external mirror too when its exact contents match.
                try:
                    mirror = c.ROOT / '文档' / p.relative_to(WORKTREE / 'docs')
                    if mirror.is_file() and c.digest(mirror) == item['sha256']:
                        evidence.append({'path': str(mirror), 'sha256': item['sha256']})
                except ValueError:
                    pass
        reason = row['failure_reason'] or row['result_evidence'].get('planning_failure', '')
        summary = ('# 实验简要记录\n\n实验：%s\n时间：%s\n\n'
                   '## 发现的问题\n\n%s\n\n'
                   '## 解决思路和方法\n\n'
                   '保留原命令、失败阶段和审计；按该失败原因重放，核对图像/深度来源时间、'
                   '支撑面和坐标绑定、候选路径及真实关节反馈。修复后分别做回归和实机验收；'
                   '未找到直接关联验收时，此段仅为后续排查方法，不表示问题已修复。\n\n'
                   '运行方法：%s\n\n%s\n\n'
                   '## 验证记录\n\n'
                   '原自动结果：failure；记录已 complete 且 recorder_closed=true。\n'
                   '原结果证据：%s\n\n%s\n\n'
                   '本次实验保持失败分类；关联文档中的后续修复或子阶段通过不改变本次结果。\n\n'
                   '仅删除回执列出的旧 bag，原图、分析输入、问题/验证资料、日志、授权和历史清单全部保留。'
                   '历史 manifest 的文件登记需结合 user_cleanup_receipt.json 查询现有可用性。\n') % (
                       row['record_id'], row['started_utc'], reason,
                       json.dumps(row['metadata'].get('command', []), ensure_ascii=False),
                       '\n\n'.join(related[:3]) or '未找到直接关联的后续验收文档。',
                       json.dumps(row['result_evidence'], ensure_ascii=False),
                       '\n'.join(direct_findings))
        record = dict(path=str(d), started_at=row['started_utc'], classification='failed',
                      formal_record=True, evidence=evidence, bound_evidence=evidence,
                      delete_files=bags, keep_files=keep, summary_markdown=summary)
        c.validate_record(record, opened)
        records.append(record)
    protected = [r['path'] for r in classification['records'] if not r['eligible_for_cleanup']]
    protected_bags = [{'path': b['path'], 'identity': c.identity(b['path'])}
                      for r in classification['records'] if r['path'] in protected
                      for b in r['raw_present'] if Path(b['path']).is_file()]
    (LOCAL / 'protected_before.json').write_text(json.dumps(protected_bags, ensure_ascii=False, indent=2)+'\n')
    stamp = dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    plan = dict(schema=1, authorization='用户本次要求按前次规则清理最早失败实验，保留问题、方法和验证；可用空间达到约60GiB停止。清理完成后继续未完成问题。',
                created_at=c.now(), initial_available_bytes=initial, target_available_bytes=60*c.GIB,
                max_available_bytes=63*c.GIB, mount_device=c.MOUNT.stat().st_dev,
                external_run_path=str(c.ROOT/'maintenance'/('user_failed_cleanup_'+stamp)),
                records=records, protect_records=protected, input_sha256=inputs,
                estimated_available_bytes=estimate)
    path = LOCAL/'plan.json'
    path.write_text(json.dumps(plan, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(dict(plan=str(path), sha256=c.digest(path), records=len(records),
                         bags=sum(len(r['delete_files']) for r in records),
                         expected_delete_gib=(estimate-initial)/c.GIB,
                         expected_available_gib=estimate/c.GIB,
                         first=records[0]['path'], last=records[-1]['path']), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
