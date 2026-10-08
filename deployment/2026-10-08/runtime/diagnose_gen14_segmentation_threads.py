"""Offline sealed-frame A/B and passive proc sampling; no ROS node or hardware calls."""
import argparse, hashlib, json, os, time
from pathlib import Path
from dataclasses import asdict
from types import SimpleNamespace

ROOT=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
RECORD=EXT/'grasp_records/20261006T085709.051090Z_bc8e72c457f542a18a60e9e94fb6b3e1'
SOURCE=1791277093240559816
LAST=1791277093104944467

def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,x): Path(p).write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')
def proc(pid):
    p=Path('/proc')/str(pid)
    result={'pid':pid,'threads':{}}
    try:
        stat=(p/'stat').read_text().rsplit(')',1)[1].split()
        result['start_ticks']=int(stat[19])
        result['status']={l.split(':')[0]:l.split(':',1)[1].strip() for l in (p/'status').read_text().splitlines() if l.startswith(('State:', 'Threads:', 'VmRSS:', 'voluntary_ctxt_switches:', 'nonvoluntary_ctxt_switches:'))}
        for t in (p/'task').iterdir():
            try: result['threads'][t.name]={'name':(t/'comm').read_text().strip(),'schedstat':[int(x) for x in (t/'schedstat').read_text().split()]}
            except FileNotFoundError: pass
        env=(p/'environ').read_bytes().split(b'\0')
        result['thread_environment']={e.split(b'=',1)[0].decode():e.split(b'=',1)[1].decode() for e in env if e.startswith((b'OPENBLAS_NUM_THREADS=',b'OMP_NUM_THREADS=',b'MKL_NUM_THREADS=',b'OPENCV_FOR_THREADS_NUM='))}
    except (FileNotFoundError, PermissionError) as e: result['error']=str(e)
    return result

def passive(out):
    rows=[]
    for i in range(21):
        row={'wall_sec':time.time(),'monotonic_sec':time.monotonic(),'processes':[proc(98801),proc(13484)], 'psi':{k:Path('/proc/pressure',k).read_text() for k in ('cpu','memory','io')},'loadavg':Path('/proc/loadavg').read_text().strip()}
        rows.append(row)
        if i<20:time.sleep(1)
    save(out/'passive.json',{'scope':'20-second later observation; not historical failure scheduling evidence','rows':rows,'script_sha256':sha(__file__)})
    a,b=rows[0],rows[-1];summary={'elapsed_sec':b['monotonic_sec']-a['monotonic_sec'],'psi_first':a['psi'],'psi_last':b['psi'],'processes':[]}
    for before,after in zip(a['processes'],b['processes']):
        deltas=[]
        for tid,v in after['threads'].items():
            if tid in before['threads']:
                d=[x-y for x,y in zip(v['schedstat'],before['threads'][tid]['schedstat'])]
                deltas.append(dict(tid=tid,name=v['name'],runtime_sec=d[0]/1e9,runqueue_wait_sec=d[1]/1e9,timeslices=d[2]))
        summary['processes'].append({'pid':after['pid'],'start_unchanged':before.get('start_ticks')==after.get('start_ticks'),'threads_first':len(before['threads']),'threads_last':len(after['threads']),'runtime_sum_sec':sum(x['runtime_sec'] for x in deltas),'runqueue_wait_sum_sec':sum(x['runqueue_wait_sec'] for x in deltas),'top_threads':sorted(deltas,key=lambda x:x['runqueue_wait_sec'],reverse=True)[:8]})
    save(out/'passive_summary.json',summary);print(json.dumps(summary),flush=True)

def extract(out):
    import cv2,numpy as np,rosbag,rospy,tf2_ros
    from cv_bridge import CvBridge
    from tf.transformations import quaternion_matrix
    m=json.loads((RECORD/'manifest.json').read_text());assert m['state']=='complete' and m['recorder_closed'] is True
    buffer=tf2_ros.Buffer(cache_time=rospy.Duration(30),debug=False)
    statuses={};depths={};camera=None;statics={}
    with rosbag.Bag(str(RECORD/'keyframes.bag')) as bag:
        for topic,msg,t in bag.read_messages(topics=['/tf_static','/grasp_record/camera_runtime_metadata']):
            if topic=='/tf_static':
                for tr in msg.transforms:statics[(tr.header.frame_id,tr.child_frame_id)]=tr;buffer.set_transform_static(tr,'sealed_record')
            elif camera is None:camera=json.loads(msg.data)
            if camera is not None and buffer.can_transform_core('tool0','camera_link',rospy.Time(0))[0]:break
        for topic,msg,t in bag.read_messages(topics=['/tf','/perception/unknown/detector_status','/supervisor/camera/depth/image_raw','/grasp_record/camera_runtime_metadata'],start_time=rospy.Time.from_sec(1791277091.0),end_time=rospy.Time.from_sec(1791277095.2)):
            if topic=='/tf':
                for tr in msg.transforms:buffer.set_transform(tr,'sealed_record')
            elif topic.endswith('detector_status'):
                s=json.loads(msg.data)
                if s.get('source_stamp_ns',0)<=SOURCE and s.get('source_stamp_ns') not in statuses:statuses[s['source_stamp_ns']]=s
            elif topic.endswith('camera_runtime_metadata'):
                if t.to_nsec()<SOURCE:camera=json.loads(msg.data)
            elif msg.header.stamp.to_nsec()<=SOURCE:depths[msg.header.stamp.to_nsec()]=msg
    selected=max(ns for ns in depths if ns in statuses and statuses[ns].get('state')=='ready')
    depth=depths[selected]
    last_ns=max(ns for ns in statuses if ns<selected and statuses[ns].get('state')=='ready')
    assert SOURCE in statuses and LAST in statuses
    tr=buffer.lookup_transform_core('base_link','camera_link',depth.header.stamp)
    q=tr.transform.rotation;t=tr.transform.translation;F=quaternion_matrix([q.x,q.y,q.z,q.w]);F[:3,3]=[t.x,t.y,t.z]
    params=camera['parameters'];array=CvBridge().imgmsg_to_cv2(depth,'passthrough')
    d=np.asarray(array,dtype=np.float32)*(params['depth_scale'] if depth.encoding in ('16UC1','mono16') else 1.)
    np.savez_compressed(out/'failure_frame.npz',depth_m=d,T_base_camera=F)
    meta={'record':str(RECORD),'manifest_sha256':sha(RECORD/'manifest.json'),'requested_failure_source_ns':SOURCE,'source_ns':selected,'source_delta_from_failure_sec':(SOURCE-selected)/1e9,'requested_failure_depth_recorded':SOURCE in depths,'shape':list(d.shape),'camera_metadata':camera,'failure_status':statuses[SOURCE],'selected_status':statuses[selected],'last_ready_status':statuses[last_ns], 'exact_tf':{'stamp_ns':depth.header.stamp.to_nsec(),'T_base_camera':F.tolist()},'scope':'Nearest recorded pre-failure measured frame only; failure depth absent from 5Hz keyframes. Exact selected-frame status and TF, no image reconstruction or simulated freshness.'}
    save(out/'fixture.json',meta);print(json.dumps({'extracted_source':selected,'requested_failure_source':SOURCE,'failure_depth_recorded':SOURCE in depths,'shape':list(d.shape),'cv2':cv2.__version__}),flush=True)

def benchmark(out):
    import cv2,numpy as np
    import alicia_grasp_modes.tabletop as tt
    meta=json.loads((out/'fixture.json').read_text());z=np.load(out/'failure_frame.npz');d=z['depth_m'];F=z['T_base_camera'];s=meta['selected_status'];prev=meta['last_ready_status'];cam=meta['camera_metadata']['parameters'];K=[cam[k] for k in ('fx','fy','cx','cy')]
    anchor=SimpleNamespace(position_base=np.array(s['anchor_position_base']),extent_m=np.array(s['anchor_extent_m']),support_footprint_base=np.array(s['reference_support_footprint_base']))
    last=SimpleNamespace(position_base=np.array(prev['position_base']))
    cfg=tt.Config();ref=np.array(s['reference_support_plane_base'])
    result={'script_sha256':sha(__file__),'tabletop_sha256':sha(ROOT/'src/alicia_grasp_modes/src/alicia_grasp_modes/tabletop.py'),'fixture_sha256':sha(out/'fixture.json'),'depth_npz_sha256':sha(out/'failure_frame.npz'),'cv2':{'version':cv2.__version__,'module':cv2.__file__,'default_threads':cv2.getNumThreads(),'cpu_count':cv2.getNumberOfCPUs(),'build_information':cv2.getBuildInformation(),'opencl_enabled':cv2.ocl.useOpenCL()},'config':asdict(cfg),'config_provenance':'source Config defaults; earlier post-attempt /unknown_perception={} snapshot; no new parameters fetched; complete live tracker serialized state unavailable, reproduced fields used by pure segmentation frozen from selected-frame status and preceding ready status. Original failure depth absent.','scope':'Offline segmentation only. Not deadline/transport or full perception replay. Current host workload differs from failure. CPU may include native helper threads. No motion authorization.','rows':[]}
    default=cv2.getNumThreads();stage_rows=[]
    def wrapped(fn,name):
        def call(*a,**kw):
            w=time.perf_counter();c=time.thread_time();p=time.process_time()
            try:return fn(*a,**kw)
            finally:stage_rows.append({'name':name,'wall':time.perf_counter()-w,'thread_cpu':time.thread_time()-c,'process_cpu':time.process_time()-p})
        return call
    for name in ('_audit_current_support','_depth_components','_fit_plane'):
        setattr(tt,name,wrapped(getattr(tt,name),name))
    for name in ('connectedComponentsWithStats','morphologyEx','floodFill'):
        setattr(cv2,name,wrapped(getattr(cv2,name),'cv2.'+name))
    for i in range(8):
        for setting in ((default,1) if i%2==0 else (1,default)):
            cv2.setNumThreads(setting);stage_rows.clear();before=len(list(Path('/proc/self/task').iterdir()));w=time.perf_counter();c=time.thread_time();p=time.process_time()
            try:
                got=tt.segment_with_support_fallback(d,K,F,cfg,anchor=anchor,last=last,reference_plane_base=ref)
                payload={'plane':got.plane_base.tolist(),'metrics':got.metrics,'candidates':[{'mask_sha256':hashlib.sha256(v.mask.tobytes()).hexdigest(),'position_base':v.position_base.tolist(),'extent_m':v.extent_m.tolist(),'metrics':v.metrics} for v in got.candidates]}
                error=None
            except ValueError as e:payload={'error':str(e)};error=str(e)
            row={'iteration':i,'opencv_threads':setting,'wall_sec':time.perf_counter()-w,'thread_cpu_sec':time.thread_time()-c,'process_cpu_sec':time.process_time()-p,'threads_before':before,'threads_after':len(list(Path('/proc/self/task').iterdir())),'result_sha256':hashlib.sha256(json.dumps(payload,sort_keys=True,allow_nan=False).encode()).hexdigest(),'stages':list(stage_rows),'error':error}
            result['rows'].append(row)
            if 'segmentation_result' not in result:result['segmentation_result']=payload
            print(json.dumps({k:v for k,v in row.items() if k!='stages'}),flush=True)
    result['all_results_identical']=len({r['result_sha256'] for r in result['rows']})==1
    result['summary']={str(n):{k:{'median':float(np.median([r[k] for r in result['rows'] if r['opencv_threads']==n])),'p95':float(np.percentile([r[k] for r in result['rows'] if r['opencv_threads']==n],95)),'max':max(r[k] for r in result['rows'] if r['opencv_threads']==n)} for k in ('wall_sec','thread_cpu_sec','process_cpu_sec')} for n in (default,1)}
    save(out/'benchmark.json',result);print(json.dumps({'summary':result['summary'],'all_results_identical':result['all_results_identical']}),flush=True)

def main():
    p=argparse.ArgumentParser();p.add_argument('--out',required=True);p.add_argument('--mode',choices=['passive','extract','benchmark'],required=True);a=p.parse_args();out=Path(a.out)
    assert Path('/media/zhuyupei/SanStand').is_mount() and str(out).startswith(str(EXT)+'/')
    out.mkdir(parents=True,exist_ok=True)
    {'passive':passive,'extract':extract,'benchmark':benchmark}[a.mode](out)
if __name__=='__main__':main()
