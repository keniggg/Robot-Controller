#!/usr/bin/env python3
"""Measure bounded lossless staging with the existing validated Cleanup methods."""
from pathlib import Path
import importlib.util,json,os,resource,signal,time
W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
R=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/maintenance/minimal_trace_cleanup_20261023T024945Z_pytest15_r1')
spec=importlib.util.spec_from_file_location('minimal_trace',W/'tools/minimal_experiment_trace.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
C=m.Cleanup(m.ROOT,R);start=time.monotonic();ru0=resource.getrusage(resource.RUSAGE_SELF);old=signal.signal(signal.SIGIO,lambda *_:setattr(C,'broken',True))
try:
 C.mounted();assert len(C.plan['reviewed_runs'])==15 and len(C.plan['reviewed_pytest_roots'])==106 and len(C.plan['text_files'])==28509 and not C.plan['raw_files']
 rows=C.plan['text_files'][:100]
 for row in rows:C.snapshot_text(row)
 usage=resource.getrusage(resource.RUSAGE_SELF);elapsed=time.monotonic()-start
 result=dict(state='pilot_staged_verified_no_deletion',files=len(rows),logical_bytes=sum(v['identity'][2] for v in rows),elapsed_seconds=elapsed,user_cpu_seconds=usage.ru_utime-ru0.ru_utime,system_cpu_seconds=usage.ru_stime-ru0.ru_stime,max_rss_kib=usage.ru_maxrss,store=C.store.stats(),source_files_deleted=0,plan_sha256=C.plan_sha,source_tool_sha256=m.sha_file(W/'tools/minimal_experiment_trace.py'),pilot_source_sha256=m.sha_file(Path(__file__)),note='Each snapshot uses existing write lease, source identity checks, per-file lossless compressed chunk verification and SQLite FULL durable commit; this sample is not the full batch.')
 m.atomic(R/'pilot_result.json',result);print(json.dumps(result,ensure_ascii=False),flush=True)
finally:signal.signal(signal.SIGIO,old);C.store.close()
