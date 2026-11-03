"""Process-local glibc allocation A/B on exact sealed frames; no ROS/control."""
import argparse,json,hashlib,os,subprocess,sys,time,resource
from pathlib import Path
P=argparse.ArgumentParser();P.add_argument('--child',choices=['baseline','retained_pages']);P.add_argument('--output',type=Path,required=True);A=P.parse_args()
ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z/gen33_handoff_analysis')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def memory():
 status={k:v.strip() for k,v in (s.split(':',1) for s in Path('/proc/self/status').read_text().splitlines() if ':' in s)}
 return {k:status[k] for k in ('VmRSS','VmHWM','VmSize','Threads')}
if not A.child:
 assert Path('/media/zhuyupei/SanStand').is_mount();A.output.mkdir(exist_ok=False)
 reports=[]
 for mode in ['baseline','retained_pages']:
  env=os.environ.copy()
  for name in ('MALLOC_TRIM_THRESHOLD_','MALLOC_MMAP_THRESHOLD_','MALLOC_ARENA_MAX'):env.pop(name,None)
  if mode=='retained_pages':env.update(MALLOC_TRIM_THRESHOLD_='-1',MALLOC_MMAP_THRESHOLD_='33554432')
  out=A.output/(mode+'.json');subprocess.run([sys.executable,__file__,'--child',mode,'--output',str(out)],env=env,check=True)
  reports.append(json.loads(out.read_text()))
 expected={}
 for report in reports:
  for row in report['runs']:
   key=row['role'];expected.setdefault(key,row['output_sha256']);assert expected[key]==row['output_sha256'],(report['mode'],key)
 summary=[]
 for x in reports:
  rows=x['runs'];steady=rows[2:]
  summary.append(dict(mode=x['mode'],all_wall_sec=[round(r['wall_sec'],6) for r in rows],steady_max_wall_sec=max(r['wall_sec'] for r in steady),steady_mean_wall_sec=sum(r['wall_sec'] for r in steady)/len(steady),steady_minor_faults=sum(r['resource_delta']['ru_minflt'] for r in steady),steady_system_cpu_sec=sum(r['resource_delta']['ru_stime'] for r in steady),final_memory=x['final_memory']))
 result=dict(scope='OFFLINE_PROCESS_LOCAL_ALLOCATOR_AB_NO_ROS_NO_CONTROL',all_output_bitexact=True,config_changes='glibc process environment only; all geometry/support/identity code and inputs unchanged',summaries=summary,artifacts=[dict(path=str(A.output/(x['mode']+'.json')),sha256=sha(A.output/(x['mode']+'.json'))) for x in reports],script_sha256=sha(__file__),limitations=['Sequential offline allocation experiment under current host load; not live frame processing guarantee.','Retained allocator pages raise RSS; does not cache target geometry or relax source timestamps.'])
 (A.output/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));sys.exit(0)
import cv2,numpy as np
from types import SimpleNamespace
from alicia_grasp_modes import tabletop as table
cv2.setNumThreads(1);cfg=table.Config();meta_path=ROOT/'exact_source_images/manifest.json';meta=json.loads(meta_path.read_text());rows={r['role']:r for r in meta['sources']}
s=rows['original_anchor']['detector_status']['message'];anchor=SimpleNamespace(position_base=np.asarray(s['anchor_position_base']),extent_m=np.asarray(s['anchor_extent_m']),support_footprint_base=np.asarray(s['reference_support_footprint_base']));plane=np.asarray(s['reference_support_plane_base']);last=SimpleNamespace(position_base=np.asarray(rows['last_good']['detector_status']['message']['position_base']))
frames={}
for role in ('last_good','first_loss'):
 row=rows[role];data=np.load(row['npz']);cam=row['camera_metadata']['message']['parameters'];frames[role]=(data['depth_raw'].astype(np.float32)*cam['depth_scale'],[cam[k] for k in ('fx','fy','cx','cy')],data['T_base_camera'])
def digest(result):
 h=hashlib.sha256()
 for array in [result.plane_base]+[a for c in result.candidates for a in [c.mask,c.center_uv,c.position_camera,c.position_base,c.extent_m,c.support_footprint_base]]:h.update(np.ascontiguousarray(array).tobytes())
 h.update(json.dumps([result.metrics]+[dict(bbox=c.bbox,depth_m=c.depth_m,quality=c.quality,metrics=c.metrics) for c in result.candidates],sort_keys=True).encode());return h.hexdigest()
before=sha(table.__file__);runs=[];initial_memory=memory()
for i in range(12):
 role=('last_good','first_loss')[i%2];dep,K,T=frames[role];u=resource.getrusage(resource.RUSAGE_THREAD);t=time.perf_counter();result=table.segment_with_support_fallback(dep,K,T,cfg,anchor,last,plane,prefer_current_support=True);wall=time.perf_counter()-t;v=resource.getrusage(resource.RUSAGE_THREAD)
 runs.append(dict(iteration=i,role=role,wall_sec=wall,resource_delta={k:getattr(v,k)-getattr(u,k) for k in ('ru_utime','ru_stime','ru_minflt','ru_majflt','ru_nvcsw','ru_nivcsw')},output_sha256=digest(result),candidate_count=len(result.candidates),memory=memory()))
assert sha(table.__file__)==before
report=dict(mode=A.child,environment={k:os.environ.get(k) for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MALLOC_TRIM_THRESHOLD_','MALLOC_MMAP_THRESHOLD_','MALLOC_ARENA_MAX')},opencv_threads=cv2.getNumThreads(),source_sha256={table.__file__:before},input_manifest=str(meta_path),input_manifest_sha256=sha(meta_path),initial_memory=initial_memory,final_memory=memory(),runs=runs)
assert not A.output.exists();A.output.write_text(json.dumps(report,indent=2)+'\n')
