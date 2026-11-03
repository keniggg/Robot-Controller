"""Single pure tabletop pass from frozen measured geometry; no bag/SDK/ROS.

The recorded descriptor explicitly declares counterfactual preappend geometry.
This restores only pure generation inputs, runs one original generation pass,
and retains every candidate/audit/diagnostic in a canonical compressed result.
"""
import argparse, dataclasses, gzip, hashlib, importlib, importlib.util, json
import os, platform, socket, struct, sys, threading, time, traceback
from pathlib import Path
from types import SimpleNamespace as NS

W=Path(__file__).resolve().parents[1];B=W/'.runtime/gen46_baseline_source'
V=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
F=V/'gen46_g45_tabletop_profile_r1';attempts=[]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def forbidden(*a,**k):attempts.append(repr(a)[:300]);raise RuntimeError('pure frozen tabletop forbids external boundary')
def baseline_pins():
 manifest=B/'manifest.json';assert sha(manifest)=='ba859bc97179662f214db2d92fdb74716468dfefb2cbe81bb3975163b90adc7b'
 pins=json.loads(manifest.read_text())['source_sha256'];assert len(pins)==1044
 assert all(sha(B/path)==digest for path,digest in pins.items())
 return {'manifest_sha256':sha(manifest),'source_count':len(pins),'all_source_hashes_match':True}
def plain(v):
 if dataclasses.is_dataclass(v):return {f.name:plain(getattr(v,f.name))for f in dataclasses.fields(v)}
 if hasattr(v,'items'):return {str(k):plain(x)for k,x in v.items()}
 if isinstance(v,(tuple,list)):return [plain(x)for x in v]
 if hasattr(v,'tolist'):return v.tolist()
 return v

def typed(v):
 """Keep class/container/array semantics alongside exact plain values."""
 kind=type(v).__module__+'.'+type(v).__qualname__
 if dataclasses.is_dataclass(v):return {'type':kind,'fields':{f.name:typed(getattr(v,f.name))for f in dataclasses.fields(v)}}
 if hasattr(v,'items'):return {'type':kind,'items':{str(k):typed(x)for k,x in v.items()}}
 if isinstance(v,(tuple,list)):return {'type':kind,'items':[typed(x)for x in v]}
 if hasattr(v,'dtype')and hasattr(v,'shape'):
  return {'type':kind,'dtype':str(v.dtype),'shape':list(v.shape),'writeable':bool(v.flags.writeable)if hasattr(v,'flags')else None}
 return {'type':kind}

def restore_inputs(model_source,expected_sha):
 """Return original ModeAware node, measured GeometryEstimate and quality.

 Original class constructors validate every dataclass/model. Box observation
 aliases the identical restored current observation, preserving cache identity.
 No synthetic pixels, geometry, messages or new measurement are introduced.
 """
 assert sha(model_source)==expected_sha
 sys.dont_write_bytecode=True;platform.architecture=lambda *a,**k:(str(struct.calcsize('P')*8)+'bit','ELF');platform._syscmd_uname=lambda *a,**k:''
 socket.has_ipv6=False
 for key in ('connect','connect_ex','sendto','bind','listen'):setattr(socket.socket,key,forbidden)
 socket.create_connection=socket.getaddrinfo=forbidden
 def audit(ev,args):
  if ev in ('subprocess.Popen','os.system','os.posix_spawn','os.exec'):forbidden(ev)
  if ev=='open'and args and isinstance(args[0],(str,bytes,os.PathLike)):
   p=os.path.realpath(os.fsdecode(args[0]))
   if p.startswith('/dev/')and p not in ('/dev/null','/dev/zero','/dev/random','/dev/urandom')and not p.startswith(('/dev/shm/','/dev/fd/','/dev/pts/')):forbidden('device',p)
 sys.addaudithook(audit)
 sys.path[:0]=[str(B/'src/alicia_grasp_modes/src'),str(B/'src/alicia_flexible_grasp_supervisor/src'),str(W/'devel/lib/python3/dist-packages'),'/opt/ros/noetic/lib/python3/dist-packages']
 import numpy as np,rospy
 for key in ('init_node','Publisher','Subscriber','Service','ServiceProxy','wait_for_service','set_param','delete_param','get_master'):setattr(rospy,key,forbidden)
 # Only this original-qualified module may vary between the two passes.
 name='alicia_flexible_grasp.vision.regular_box_inference'
 importlib.import_module('alicia_flexible_grasp.vision')
 spec=importlib.util.spec_from_file_location(name,str(model_source));model=importlib.util.module_from_spec(spec);sys.modules[name]=model;spec.loader.exec_module(model)
 mode_path=B/'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py'
 spec=importlib.util.spec_from_file_location('gen46_frozen_mode_remote',str(mode_path));mode=importlib.util.module_from_spec(spec);sys.modules[spec.name]=mode;spec.loader.exec_module(mode)
 frozen=json.loads((F/'frozen_geometry_inputs.json').read_text())
 assert sha(F/'frozen_geometry_arrays.npz')=='02e5aab4d7a78e9dba90b6ff4b5706312ec2ac8051c69562cb3db4a04391ecaa'
 assert sha(F/'frozen_geometry_inputs.json')=='4d9e8190fe4b797800f5093dcfde046e20c92c68dec687795cb269469ef92f4c'
 arrays=np.load(F/'frozen_geometry_arrays.npz',allow_pickle=False)
 def restore(v):
  if isinstance(v,list):return [restore(x)for x in v]
  if not isinstance(v,dict):return v
  if 'npz_key'in v:
   a=arrays[v['npz_key']].copy();assert list(a.shape)==v['shape']and str(a.dtype)==v['dtype'];a.setflags(write=False);return a
  if 'fields'in v and 'type'in v:
   mod,cls=v['type'].rsplit('.',1);typ=getattr(importlib.import_module(mod),cls)
   fields={k:restore(x)for k,x in v['fields'].items()};allowed={f.name for f in dataclasses.fields(typ)if f.init}
   return typ(**{k:x for k,x in fields.items()if k in allowed})
  return {k:restore(x)for k,x in v.items()}
 n=mode.ModeAwareRemoteGrasp6DNode.__new__(mode.ModeAwareRemoteGrasp6DNode)
 for key,value in frozen['node_inputs'].items():setattr(n,key,restore(value))
 geometry=restore(frozen['geometry']);quality=restore(frozen['snapshot_quality'])
 snapshot=NS(quality=quality,stamp_ns=frozen['snapshot_source_stamps_ns'][-1],sample_stamp_ns=tuple(frozen['snapshot_source_stamps_ns']))
 life=tuple(n._regular_box_snapshot['lifecycle']);n._regular_box_snapshot['lifecycle']=life
 n._regular_box_snapshot['observation']=n._latest_target_observation
 n._stream_generation,n._near_field_phase_id,n.near_field_planning_active=life[:3]
 assert life[3]==n._latest_target_observation.identity
 n._near_field_surface_generation=n._stream_generation;n._near_field_surface_phase_id=n._near_field_phase_id
 n._target_identity_generation=1;n.target_instance_epoch=n._latest_target_observation.identity.epoch
 n._geometry_state_lock=threading.RLock();n._stream_condition=threading.Condition(threading.RLock())
 params=frozen['parameters'];n._mode_selection=params['/grasp_mode/selection'];n._observe_return_phase='observing_contact';n._measurement_remote_request={'state':'qualified'}
 n.grasp_config=params['/grasp'];n.runtime_execution_error_max_age_sec=180.
 def get_param(name,default=None):
  if name in params:return params[name]
  for root in sorted(params,key=len,reverse=True):
   if name.startswith(root+'/'):
    item=params[root]
    try:
     for key in name[len(root)+1:].split('/'):item=item[key]
     return item
    except(KeyError,TypeError):return default
  return default
 rospy.get_param=rospy.get_param_cached=get_param
 # Same explicitly frozen profile clock, only the pure runtime-error age uses
 # this value. There is no owner ticket, publication or freshness authorization.
 clock_ns=1792306912204162279;rospy.Time.now=staticmethod(lambda:rospy.Time(*divmod(clock_ns,10**9)))
 return n,geometry,snapshot,frozen

def run(model_source,expected_sha,output):
 assert Path('/media/zhuyupei/SanStand').is_mount()and V in output.parents and not output.exists();output.mkdir()
 os.environ['ROS_LOG_DIR']=str(output/'unused_ros');os.environ['TMPDIR']=str(output/'tmp');(output/'tmp').mkdir()
 result={'pipeline_success':False,'physical_grasp_verified':False,'historical_G45_geometry_exact':False,'ROS_or_hardware_access':False,'temperature_read':False,'bag_or_RGBD_read':False,'errors':[]}
 try:
  result['source_pins_before']=baseline_pins()
  n,geometry,snapshot,frozen=restore_inputs(model_source,expected_sha)
  old_loads=json.loads;count={'calls':0,'UTF8_bytes':0}
  def counted(s,*a,**k):
   count['calls']+=1;count['UTF8_bytes']+=len(s)if isinstance(s,(bytes,bytearray))or isinstance(s,str)and s.isascii()else len(s.encode('utf8'))
   return old_loads(s,*a,**k)
  json.loads=counted;start=time.perf_counter();cpu=time.thread_time()
  try:out=n._generate_tabletop_candidates(geometry,snapshot=snapshot,contact_execution_phase=True,target_label='unknown_small')
  finally:
   wall=time.perf_counter()-start;thread_cpu=time.thread_time()-cpu;json.loads=old_loads
  assert sha(model_source)==expected_sha
  assert sha(F/'frozen_geometry_arrays.npz')=='02e5aab4d7a78e9dba90b6ff4b5706312ec2ac8051c69562cb3db4a04391ecaa'
  assert sha(F/'frozen_geometry_inputs.json')=='4d9e8190fe4b797800f5093dcfde046e20c92c68dec687795cb269469ef92f4c'
  result['source_pins_after']=baseline_pins()
  payload=json.dumps({'candidates':plain(out[0]),'diagnostics':plain(out[1]),'typed_outputs':typed(out)},sort_keys=True,separators=(',',':'),allow_nan=False).encode()
  compressed=gzip.compress(payload,mtime=0);(output/'candidates_and_diagnostics.json.gz').write_bytes(compressed)
  result.update(completed=True,candidate_count=len(out[0]),wall_sec=wall,thread_cpu_sec=thread_cpu,json_loads=count,canonical_output_bytes=len(payload),canonical_output_sha256=hashlib.sha256(payload).hexdigest(),compressed_output_sha256=sha(output/'candidates_and_diagnostics.json.gz'),input_descriptor_sha256=sha(F/'frozen_geometry_inputs.json'),input_arrays_sha256=sha(F/'frozen_geometry_arrays.npz'),model_source=str(model_source),model_source_sha256=expected_sha,counterfactual_preappend=frozen['counterfactual_preappend'],snapshot_provider='Original recorded immutable DepthQuality and source stamps only; pure generation reads no image arrays.',observer_overhead='Wall/thread CPU include Python JSON-call and UTF8 byte counters; no cProfile.',source_provenance={name:{'path':str(Path(m.__file__).resolve()),'sha256':sha(m.__file__)}for name,m in sys.modules.copy().items()if getattr(m,'__file__',None)and(str(B)in str(m.__file__)or Path(m.__file__).resolve()==model_source.resolve())and Path(m.__file__).is_file()})
 except BaseException as e:result['errors'].append({'type':type(e).__name__,'message':str(e),'traceback':traceback.format_exc()})
 result['boundary_attempts']=attempts;result['helper_sha256']=sha(__file__)
 (output/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
 print(json.dumps({k:result.get(k)for k in ['completed','candidate_count','wall_sec','thread_cpu_sec','json_loads','canonical_output_sha256','errors']},ensure_ascii=False))
 print(json.dumps({'result':str(output/'result.json'),'sha256':sha(output/'result.json')}))

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--model-source',type=Path,required=True);p.add_argument('--expected-model-sha256',required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.model_source,a.expected_model_sha256,a.output)
