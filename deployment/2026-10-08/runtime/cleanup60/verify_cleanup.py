#!/usr/bin/env python3
"""Verify the exact receipt, retained contents and untouched protected records."""
import json
from pathlib import Path
import cleanup_failed_sanstand as c

LOCAL = Path(__file__).resolve().parent


def main():
    plan = json.loads((LOCAL/'plan.json').read_text())
    run = Path(plan['external_run_path'])
    receipt = json.loads((run/'receipt.json').read_text())
    errors, retained_count, summary_count, deleted_count = [], 0, 0, 0
    if receipt['plan_sha256'] != c.digest(LOCAL/'plan.json') or c.digest(run/'plan.json') != c.digest(LOCAL/'plan.json'):
        errors.append('frozen plan mismatch')
    if receipt['state'] != 'complete' or not 60*c.GIB <= receipt['final_available_bytes'] <= 63*c.GIB:
        errors.append('recorded stopping criterion not met')
    records = {r['path']: r for r in plan['records']}
    allocation = 0
    for r in receipt['records']:
        original = records[r['path']]
        d = c.checked_path(r['path'])
        if r['state'] != 'complete':
            errors.append('incomplete record '+str(d))
        summary = d/'实验简要记录.md'
        if c.digest(summary) != r['summary_sha256'] or summary.read_text() != original['summary_markdown']:
            errors.append('summary changed '+str(summary))
        summary_count += 1
        if json.loads((d/'user_cleanup_receipt.json').read_text()) != r:
            errors.append('per-record receipt differs '+str(d))
        allowed = {i['path']: i['identity'] for i in original['delete_files']}
        for item in r['deleted']:
            if item['identity'] != allowed.get(item['path']) or Path(item['path']).exists():
                errors.append('deleted-file mismatch '+item['path'])
            allocation += item['identity']['allocated']
            deleted_count += 1
        for item in r['retained']:
            if c.digest(item['path']) != item['sha256']:
                errors.append('retained contents changed '+item['path'])
            retained_count += 1
    if deleted_count != receipt['deleted_file_count'] or allocation != receipt['deleted_allocated_bytes']:
        errors.append('deleted totals differ')
    protected = json.loads((LOCAL/'protected_before.json').read_text())
    for item in protected:
        if c.identity(item['path']) != item['identity']:
            errors.append('protected recording changed '+item['path'])
    policies = json.loads((LOCAL/'policy_before.json').read_text())
    for p, info in policies.items():
        if c.digest(p) != info['sha256']:
            errors.append('automatic storage policy changed '+p)
    result = dict(schema=1, verified_at=c.now(), ok=not errors, errors=errors,
                  plan_sha256=c.digest(LOCAL/'plan.json'), summaries_verified=summary_count,
                  retained_files_sha256_verified=retained_count, deleted_bags_verified=deleted_count,
                  protected_files_identity_verified=len(protected), policy_sha256_verified=len(policies),
                  deleted_allocated_gib=allocation/c.GIB,
                  initial_available_gib=receipt['initial_available_bytes']/c.GIB,
                  recorded_stop_available_gib=receipt['final_available_bytes']/c.GIB,
                  available_gib_now=c.free_bytes()/c.GIB,
                  limits='Allocated deleted bytes and net free-space gain differ when existing logs keep writing.')
    c.atomic(run/'verification.json', result)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if errors:
        raise RuntimeError('cleanup verification failed')


if __name__ == '__main__':
    main()
