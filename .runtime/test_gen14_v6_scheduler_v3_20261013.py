#!/usr/bin/env python3
"""Infrastructure checks only; never qualify a production observation."""
import ast, hashlib, json, queue, threading, time, traceback
from pathlib import Path
P=Path(__file__).with_name('replay_gen14_three_owner_qualification_v6_20261013.py')
F=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z/verification/gen14_arrival_independent')
source=P.read_text();tree=ast.parse(source);main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
names={'raw_clock','clock','channel','Worker','submit','schedule','dispatch','stop_workers'}
parts=[n for n in main.body if isinstance(n,(ast.FunctionDef,ast.ClassDef))and n.name in names]
assert {n.name for n in parts}==names
import heapq
ns=dict(threading=threading,queue=queue,time=time,heapq=heapq,traceback=traceback,
 concurrency_active=[True],clock_lock=threading.Lock(),ingress_busy=[False],
 start=[1791860212162192583],perf=[time.perf_counter_ns()],running=[True],now=[0],
 last_clock_return=[0],clock_return_count=[0],heap_lock=threading.Lock(),workers={},workers_lock=threading.Lock(),worker_errors=[],
 scheduler_stop=threading.Event(),producer_tick_queued=threading.Event(),timer_queued=threading.Event(),task_timer_queued=threading.Event(),scheduler_thread=[None],capture_running=[False],timers_enabled=[False],drain_audit=[],
 callback_journal=[],journal=[],events=[],event_id=[0])
exec(compile(ast.Module(body=parts,type_ignores=[]),str(P),'exec'),ns)
records=[];entered=threading.Event();release=threading.Event();producer_done=threading.Event();sdk_done=threading.Event();fifo=[]
def blocking_phase():
 entered.set();assert release.wait(1.)
def producer():
 assert entered.is_set() and not release.is_set();records.append(('producer',ns['clock']()));producer_done.set()
def sdk():
 assert entered.is_set() and not release.is_set();records.append(('sdk',ns['clock']()));sdk_done.set()
ns['submit']('remote_task_phase',blocking_phase,ns['clock']());assert entered.wait(1.)
ns['submit']('producer_tick',producer,ns['clock']());ns['submit']('remote_accepted_sdk',sdk,ns['clock']())
assert producer_done.wait(1.) and sdk_done.wait(1.)
for i in range(4):ns['submit']('remote_status',lambda i=i:fifo.append(i),ns['clock']())
future_ran=threading.Event();ns['schedule']('remote_mask',future_ran.set,20_000_000)
ns['dispatch']();assert not future_ran.is_set()
start_clock=ns['clock']();time.sleep(.025);end_clock=ns['clock']();assert end_clock-start_clock>=20_000_000
ns['dispatch']();assert future_ran.wait(1.);release.set()
# Many simultaneous clock consumers must not publish an older pre-lock read.
for i in range(5):ns['submit']('clock_reader_'+str(i),lambda:[ns['clock']()for _ in range(400)],ns['clock']())
ns['stop_workers']();assert fifo==[0,1,2,3] and not ns['worker_errors']
assert all(not worker.thread.is_alive()for worker in ns['workers'].values())
for row in ns['callback_journal']:assert row['start_ns']>=row['due_ns']
phase=next(r for r in ns['callback_journal']if r['kind']=='remote_task_phase')
for label in ('producer_tick','remote_accepted_sdk'):
 row=next(r for r in ns['callback_journal']if r['kind']==label)
 assert phase['start_ns']<=row['start_ns']<=row['completion_ns']<=phase['completion_ns']
assert source.count('not worker_errors')>=3
positive_workers_joined=all(not worker.thread.is_alive() for worker in ns['workers'].values())
# An exception in the task main thread can occur before the recorded window
# ends. Cleanup must join promptly without demanding future source arrivals.
ns.update(workers={},scheduler_stop=threading.Event(),worker_errors=[],events=[],
 frames=[dict(pair_available_ns=ns['clock']()+10_000_000_000)],last_ingress_advanced_ns=[0])
def transport_wait():ns['scheduler_stop'].wait(1.)
ns['scheduler_thread']=[threading.Thread(target=transport_wait)];ns['scheduler_thread'][0].start()
started=time.perf_counter();ns['stop_workers'](require_full_window=False);early_cleanup_elapsed=time.perf_counter()-started
assert early_cleanup_elapsed<.5 and not ns['worker_errors'] and not ns['scheduler_thread'][0].is_alive()
# A real callback exception remains in the failure journal after shutdown.
ns.update(workers={},scheduler_stop=threading.Event(),worker_errors=[],events=[],scheduler_thread=[None])
def intentional_failure():raise ValueError('intentional infrastructure negative control')
ns['submit']('negative_callback',intentional_failure,ns['clock']())
assert ns['scheduler_stop'].wait(1.)
ns['stop_workers'](require_full_window=False)
assert len(ns['worker_errors'])==1 and ns['worker_errors'][0]['label']=='negative_callback'
assert 'intentional infrastructure negative control' in ns['worker_errors'][0]['error']
assert all(not worker.thread.is_alive()for worker in ns['workers'].values())
expected_worker_failure=list(ns['worker_errors'])

report=dict(passed=True,harness=str(P),harness_sha256=hashlib.sha256(P.read_bytes()).hexdigest(),
 expected_worker_failure=expected_worker_failure,early_failure_cleanup_elapsed_sec=early_cleanup_elapsed,positive_workers_joined=positive_workers_joined,
 checks=['early task failure cleanup does not wait for future window','callback exceptions are retained after shutdown','remote fusion cannot block independent producer and accepted SDK workers','same-connection FIFO','future delivery waits until original due lower bound','shared wall clock continues through waits','concurrent clock returns checked under one lock','all workers joined','worker errors remain in all success gates'],
 callback_journal=ns['callback_journal'],clock_calls=ns['clock_return_count'][0],wall_clock_wait_delta_ns=end_clock-start_clock,
 scope='Infrastructure only, no production qualification, model RPC, ROS master, motion or altered deadlines',script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
O=F/'v6_scheduler_infrastructure_r3';O.mkdir(exist_ok=False);(O/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(O/'result.json')
