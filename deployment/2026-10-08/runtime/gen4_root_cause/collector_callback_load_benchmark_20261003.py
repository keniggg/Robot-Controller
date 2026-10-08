"""Offline scheduling benchmark using saved RGB-D and exact failed event.
No node, ROS master call, device context, command interface or source edit.
Heartbeat probes scheduling in this independent process; it is not a recording
of the failed collector and cannot uniquely attribute that historical delay.
"""
import hashlib,io,json,math,sys,threading,time
from pathlib import Path
import numpy as np
import roslz4
from sensor_msgs.msg import Image
import rospy

WORK=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
sys.path[:0]=[str(WORK/'.runtime'),str(WORK/'.runtime/realsense-2.56.5')]
import gen4_board_probe_plan as board
ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_ros_latest_start_20261002/gen4_root_cause')
SOURCE=ROOT/'orthogonal_probe_20261003T134743Z'
OUT=ROOT/'collector_callback_load_benchmark_20261003'
HASHES={}


def use(p):
 p=Path(p).resolve();h=hashlib.sha256(p.read_bytes()).hexdigest()
 if str(p) in HASHES and HASHES[str(p)]!=h:raise RuntimeError('input changed: '+str(p))
 HASHES[str(p)]=h;return p


def benchmark(name,work,duration=1.5,iterations=None):
 done=threading.Event();started=threading.Event();ticks=[]
 def pulse():
  ticks.append(time.monotonic());started.set()
  while not done.wait(.005):ticks.append(time.monotonic())
 thread=threading.Thread(target=pulse);thread.start();started.wait()
 durations=[];cpu0=time.thread_time();start=time.monotonic()
 try:
  while ((len(durations)<iterations) if iterations is not None else time.monotonic()-start<duration):
   before=time.monotonic();work();durations.append(time.monotonic()-before)
 finally:
  done.set();thread.join(1.)
 elapsed=time.monotonic()-start;cpu=time.thread_time()-cpu0
 gaps=np.diff(ticks)*1000
 return dict(case=name,work_iterations=len(durations),wall_seconds=elapsed,main_thread_CPU_seconds=cpu,
  heartbeat_samples=len(ticks),heartbeat_interval_ms=5.,
  heartbeat_gap_p50_p95_max_ms=np.percentile(gaps,[50,95,100]).tolist() if len(gaps) else [],
  operation_p50_p95_max_ms=np.percentile(np.array(durations)*1000,[50,95,100]).tolist(),
  heartbeat_gap_above_50ms=int(np.sum(gaps>50)),heartbeat_gap_above_500ms=int(np.sum(gaps>500)))


def main():
 if not Path('/media/zhuyupei/SanStand').is_mount():raise RuntimeError('SanStand missing; no fallback')
 event_rows=[json.loads(line) for line in use(SOURCE/'events.jsonl').read_text().splitlines()]
 row=next(r for r in event_rows if r['event']=='actual_submission_geometry')
 event_bytes=len(json.dumps(row).encode())
 baseline=json.loads(use(SOURCE/'fresh_plan/snapshot.json').read_text())
 endpoint=json.loads(use(SOURCE/'1791035294623654842_board_binding.json').read_text())
 xml=use(SOURCE/'robot_description.urdf').read_text()
 with np.load(use(SOURCE/'1791035294623654842_rgbd_tf.npz'),allow_pickle=False) as data:
  rgb=data['image_bgr'].copy();depth=data['depth_raw'].copy()
 messages=[]
 for image,encoding,step in ((rgb,'bgr8',rgb.shape[1]*3),(depth,'16UC1',depth.shape[1]*2)):
  m=Image();m.height,m.width=image.shape[:2];m.encoding=encoding;m.step=step;m.data=image.tobytes()
  m.header.stamp=rospy.Time(*divmod(endpoint['source_stamp_ns'],10**9));m.header.frame_id='camera_link';messages.append(m)
 def serialize_pair():
  for m in messages:
   stream=io.BytesIO();m.serialize(stream);stream.getvalue()
 payload=rgb.tobytes()+depth.tobytes()
 def compress_pair():
  c=roslz4.LZ4Compressor();c.compress(payload);c.flush()
 larger=payload*16
 def compress_large():
  c=roslz4.LZ4Compressor();c.compress(larger);c.flush()
 def json_twice():json.dumps(row);json.dumps(row)
 model=board.make_runtime_model(baseline['camera_model_evidence'])
 current={k:baseline[k] for k in ('urdf_sha256','T_tool_camera','camera_model_evidence',
   'camera_model_fingerprint','epoch_ns','depth_scale')}
 plan=board.BoardOrthogonalProbeAudit(baseline,xml,lambda q,g:True,model,lambda:current)
 # Logical setup for a computation-only return audit, never hardware authority.
 plan.prepared=True
 plan.begin_step(0,baseline['q_rad'],baseline['sdk_q_rad'])
 plan.complete_step(endpoint['q_rad'],endpoint['sdk_q_rad'])
 plan.begin_step(1,endpoint['q_rad'],endpoint['sdk_q_rad'])
 def return_math():plan.guard_segment(endpoint['q_rad'],plan.sequence[1]['target_sdk'])
 results=[benchmark('sleep_control',lambda:time.sleep(.005)),
   benchmark('serialize_original_RGB_depth_pair',serialize_pair),
   benchmark('JSON_encode_exact_205KB_event_twice',json_twice),
   benchmark('native_ROSLZ4_real_pair_1_536MB_compress_flush',compress_pair),
   benchmark('native_ROSLZ4_16pairs_stress_not_actual_chunk',compress_large,iterations=3),
   benchmark('frozen_factory_actual_return_geometry_no_XMLRPC_MoveIt',return_math,iterations=2)]
 for p in (Path(__file__),Path(board.__file__),Path(roslz4.__file__),Path(roslz4._roslz4.__file__)):
  use(p)
 for p in list(HASHES):use(p)
 report=dict(method='Saved data only; heartbeat period5ms in a separate offline process. Native compression has no filesystem I/O. Geometry model has explicit stub validity and static model reads; no ROS service, command or device opened.',
  payload_bytes=len(payload),stress_payload_bytes=len(larger),original_event_bytes=event_bytes,
  results=results,input_sha256_by_path=dict(sorted(HASHES.items())),
  historical_timing=dict(first_205KB_event_ns=row['time_ns'],
   return_step_begin_ns=event_rows[-2]['time_ns'],failure_ns=event_rows[-1]['time_ns'],
   return_submission_event_present=False),
  limitations=['These scheduling measurements are not the failing process or the same host load and do not establish a unique historical cause.',
   'No filesystem latency benchmark; observed archive maximum write788ms belongs to the earlier record.',
   '16-pair compressed stress input is deliberately larger than ordinary image chunks; do not claim it occurred in the failure.',
   'Native GIL release/retention is inferred only from scheduling sensitivity here,not conclusively from unavailable extension source.'])
 OUT.mkdir(exist_ok=False)
 (OUT/'result.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
 (OUT/'analysis_source.py').write_bytes(Path(__file__).read_bytes())
 (OUT/'sha256.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in OUT.iterdir() if p.is_file()},indent=2)+'\n')
 print(json.dumps(dict(output=str(OUT/'result.json'),results=results),indent=2),flush=True)


if __name__=='__main__':main()
