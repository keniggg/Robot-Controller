#!/usr/bin/env python3
"""Explicit camera-only reload with a finite tap of existing read diagnostics.

Never publishes a robot, enable, selection or task command. The replacement
camera is an independent non-restarting systemd user service. All remaps and
the existing camera's ROS/SDK environment are preserved; only its log path
changes. Image stamps denote host read completion, not calibrated exposure.
"""
import argparse
from collections import Counter
import datetime
import hashlib
import http.client
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xmlrpc.client

ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
NODE='/supervisor_camera'
MOUNT=Path('/media/zhuyupei/SanStand')
OUTROOT=MOUNT/'实验记录/Robot-Controller/ros_log/full_ros_latest_start_20261002/gen4_root_cause'
OBSERVER=HERE/'camera_existing_read_observer.py'
CAMERA_ENTRIES=(ROOT/'devel/lib/alicia_flexible_grasp_supervisor/camera_node.py',
                ROOT/'src/alicia_flexible_grasp_supervisor/scripts/camera_node.py')
PUBLICATION_SOURCE=ROOT/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/latest_rgbd_publication.py'
SOURCES=[ROOT/'src/alicia_flexible_grasp_supervisor/scripts/camera_node.py',
         ROOT/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/realsense_manager.py',
         ROOT/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/visual_stationary.py',
         ROOT/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/stationary_depth.py',
         PUBLICATION_SOURCE,CAMERA_ENTRIES[0],OBSERVER,Path(__file__).resolve()]
ENV_KEYS=('ROS_MASTER_URI','ROS_IP','ROS_HOSTNAME','ROS_DISTRO','ROS_VERSION','ROS_PYTHON_VERSION',
          'ROS_PACKAGE_PATH','CMAKE_PREFIX_PATH','LD_LIBRARY_PATH','PYTHONPATH','PATH',
          'OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS',
          'PYTHONNOUSERSITE','QT_QPA_PLATFORM')


def identity(pid):
    p=Path('/proc')/str(int(pid));s=(p/'stat').read_text();fields=s[s.rfind(')')+2:].split()
    return dict(pid=int(pid),ppid=int(fields[1]),start_ticks=int(fields[19]),exe=os.readlink(str(p/'exe')),
                exe_sha256=hashlib.sha256((p/'exe').read_bytes()).hexdigest(),
                cwd=os.readlink(str(p/'cwd')),argv=(p/'cmdline').read_bytes().decode().strip('\0').split('\0'))


def alive(original):
    try:
        s=(Path('/proc')/str(original['pid'])/'stat').read_text();fields=s[s.rfind(')')+2:].split()
        return int(fields[19])==original['start_ticks'] and fields[0]!='Z'
    except FileNotFoundError:return False


def camera_environment(pid):
    rows=(Path('/proc')/str(pid)/'environ').read_bytes().split(b'\0')
    parsed={row.split(b'=',1)[0].decode():row.split(b'=',1)[1].decode() for row in rows if b'=' in row}
    return {key:parsed[key] for key in ENV_KEYS if key in parsed}


def camera_entry_and_ros_tail(old_argv):
    """Unwrap exactly one known diagnostic wrapper; never replay its options."""
    if len(old_argv)<3 or any(not isinstance(a,str) or not a for a in old_argv):
        raise RuntimeError('expected existing camera entry point and node name')
    entry=old_argv[1]
    if Path(entry).resolve()==OBSERVER.resolve():
        rest=old_argv[2:]
        if rest.count('--')!=1:
            raise RuntimeError('camera observer must have exactly one ROS argument delimiter')
        split=rest.index('--');options=rest[:split];tail=rest[split+1:]
        allowed={'--camera-script','--diagnostic-path','--diagnostic-seconds'}
        parsed={}
        if len(options)!=6:
            raise RuntimeError('unexpected camera observer option count')
        for key,value in zip(options[::2],options[1::2]):
            if key not in allowed or key in parsed or value.startswith('--'):
                raise RuntimeError('unknown, repeated or malformed camera observer option')
            parsed[key]=value
        if set(parsed)!=allowed:
            raise RuntimeError('camera observer options incomplete')
        try:old_seconds=float(parsed['--diagnostic-seconds'])
        except ValueError:raise RuntimeError('invalid existing observer duration')
        if not math.isfinite(old_seconds) or not 0<old_seconds<=120:
            raise RuntimeError('invalid existing observer duration')
        if not Path(parsed['--diagnostic-path']).is_absolute():
            raise RuntimeError('existing observer diagnostic path must be absolute')
        entry=parsed['--camera-script']
    else:
        tail=old_argv[2:]
    if not Path(entry).is_absolute() or Path(entry).resolve() not in {p.resolve() for p in CAMERA_ENTRIES}:
        raise RuntimeError('expected verified project camera entry point')
    seen=set()
    for arg in tail:
        if arg.startswith('-') or ':=' not in arg:
            raise RuntimeError('unexpected non-ROS camera argument')
        key,value=arg.split(':=',1)
        if not key or not value or key in seen:
            raise RuntimeError('empty or repeated camera ROS argument')
        seen.add(key)
    if '__name:=supervisor_camera' not in tail:
        raise RuntimeError('expected existing camera node name')
    return entry,list(tail)


def camera_argv(old_argv,out,seconds):
    entry,previous_tail=camera_entry_and_ros_tail(old_argv)
    tail=[arg for arg in previous_tail if not arg.startswith('__log:=')]
    tail.append('__log:='+str(out/'camera.ros.log'))
    return [old_argv[0],str(OBSERVER),'--camera-script',entry,
            '--diagnostic-path',str(out/'read_diagnostics.jsonl'),'--diagnostic-seconds',str(seconds),'--']+tail


def service_argv(unit,old,out,environment,seconds):
    command=['systemd-run','--user','--collect','--unit='+unit,'--property=Type=exec','--property=Restart=no',
             '--property=WorkingDirectory='+old['cwd'],
             '--property=StandardOutput=append:'+str(out/'camera.stdout.log'),
             '--property=StandardError=append:'+str(out/'camera.stderr.log')]
    for key in ENV_KEYS:
        if key in environment:command.append('--setenv='+key+'='+environment[key])
    command+=['--setenv=ROS_LOG_DIR='+str(out/'ros'),'--setenv=PYTHONUNBUFFERED=1']
    return command+camera_argv(old['argv'],out,seconds)


class CameraRegistrationPending(RuntimeError):pass
class Transport(xmlrpc.client.Transport):
    def make_connection(self,host):return http.client.HTTPConnection(host,timeout=2.)


def node_pid(rospy,name):
    code,detail,uri=rospy.get_master().lookupNode(name)
    if code!=1:
        if detail=='unknown node ['+name+']':raise CameraRegistrationPending(detail)
        raise RuntimeError('ROS lookup failed: '+detail)
    code,detail,pid=xmlrpc.client.ServerProxy(uri,transport=Transport()).getPid(rospy.get_name())
    if code!=1:raise RuntimeError('ROS PID failed: '+detail)
    return int(pid)


def wait(check,seconds,label,guard=lambda:None):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        guard();result=check()
        if result:return result
        time.sleep(.05)
    raise RuntimeError('timeout: '+label)


def rate(rows):
    if len(rows)<2 or rows[-1]<=rows[0]:return None
    return (len(rows)-1)/(rows[-1]-rows[0])


def stable_camera_configuration(configuration):
    # Runtime geometry/publication evidence is intentionally refreshed by the
    # camera; configuration checks still include all other camera parameters.
    return {key:value for key,value in configuration.items() if key!='runtime_profile'}


def numeric_summary(values):
    values=sorted(float(v) for v in values if isinstance(v,(int,float)) and math.isfinite(v))
    if not values:return dict(count=0)
    def quantile(p):
        index=(len(values)-1)*p;lo=int(index);hi=min(lo+1,len(values)-1)
        return values[lo]+(values[hi]-values[lo])*(index-lo)
    return dict(count=len(values),min=values[0],median=quantile(.5),p95=quantile(.95),max=values[-1])


def duration_summary(values):
    finite=[float(value) for value in values if isinstance(value,(int,float)) and math.isfinite(value)]
    return dict(numeric_summary(finite),unit='seconds',
                diagnostic_threshold_sec=.2,greater_than_0_2_sec=sum(value>.2 for value in finite))


def summarize_publication(rows):
    times=[row['received_monotonic'] for row in rows]
    intervals=[(current['source_stamp_ns']-previous['source_stamp_ns'])/1e9
               for previous,current in zip(rows,rows[1:])]
    ages=[(row['received_ros_time_ns']-row['source_stamp_ns'])/1e9
          for row in rows if isinstance(row.get('received_ros_time_ns'),int)]
    return dict(frames=len(rows),fps=rate(times),
                unique_source_stamps=len(set(row['source_stamp_ns'] for row in rows)),
                header_interval_sec=duration_summary(intervals),
                receipt_interval_sec=duration_summary([b-a for a,b in zip(times,times[1:])]),
                receipt_source_age_sec=duration_summary(ages),
                receipt_source_age_clock='receipt ROS time minus header host read-completion ROS time; not exposure age',
                header_duplicates=sum(value==0 for value in intervals),
                header_regressions=sum(value<0 for value in intervals))


def save_result_with_final_guard(path,state,guard):
    """Persist a revoked result if a late SDK/epoch error invalidates success."""
    final_error=None
    if state.get('static_validation_passed') is True:
        try:
            guard()
            state['final_guard_passed']=True
        except BaseException as exc:
            state.update(static_validation_passed=False,final_guard_passed=False,error=str(exc))
            final_error=exc
    Path(path).write_text(json.dumps(state,indent=2,allow_nan=False)+'\n')
    if final_error is not None:raise final_error


def summarize_raw_stream(rows,stream):
    frames=[]
    for row in rows:
        raw=(row.get('existing_read_diagnostics') or {}).get('raw_frames',{}).get(stream,{})
        if raw.get('present') is True:frames.append(raw)
    increments=[];source_dt=[];raw_timestamp_dt=[];arrival_lag=[]
    for previous,current in zip(frames,frames[1:]):
        if all(isinstance(f.get('frame_number'),int) for f in (previous,current)):
            increments.append(current['frame_number']-previous['frame_number'])
        if all(isinstance(f.get('device_timestamp_ms'),(int,float)) for f in (previous,current)):
            source_dt.append(current['device_timestamp_ms']-previous['device_timestamp_ms'])
        pm=previous.get('metadata_raw',{});cm=current.get('metadata_raw',{})
        if all(isinstance(m.get('frame_timestamp'),int) for m in (pm,cm)):
            raw_timestamp_dt.append(cm['frame_timestamp']-pm['frame_timestamp'])
    for frame in frames:
        metadata=frame.get('metadata_raw',{})
        if all(isinstance(metadata.get(k),(int,float)) for k in ('time_of_arrival','backend_timestamp')):
            arrival_lag.append(metadata['time_of_arrival']-metadata['backend_timestamp'])
    reused=sum(value==0 for value in increments)
    skipped=sum(max(0,value-1) for value in increments)
    warnings=[]
    if reused:warnings.append('Read results contained repeated SDK frame numbers; raw RGB-D pairing must remain authoritative.')
    if skipped:warnings.append('Some SDK frame numbers were not observed between reads; this does not identify where they were lost or skipped.')
    if any(value<0 for value in increments):warnings.append('SDK frame number regression observed; continuity is not established.')
    return dict(raw_frame_samples=len(frames),
                timestamp_domain_counts=dict(Counter(str(f.get('timestamp_domain','missing')) for f in frames)),
                frame_number_increment_counts=dict(Counter(str(value) for value in increments)),
                repeated_frame_number_transitions=reused,unobserved_frame_numbers_between_reads=skipped,
                counter_regressions=sum(value<0 for value in increments),
                device_timestamp_interval_ms=numeric_summary(source_dt),
                metadata_frame_timestamp_interval_raw=numeric_summary(raw_timestamp_dt),
                backend_to_arrival_interval_raw=numeric_summary(arrival_lag),
                metadata_actual_fps_raw_counts=dict(Counter(str(f.get('metadata_raw',{}).get('actual_fps','missing')) for f in frames)),
                warnings=warnings,
                limitations=['Full read-window samples include accepted and rejected pairs; warning rows alone cannot establish a stream rate.',
                             'SDK actual_fps metadata and delivered-frame rates do not prove exposure cadence.',
                             'Unobserved counters may include unread SDK frames; they do not uniquely prove USB loss.',
                             'Raw metadata values are preserved without assuming an exposure-to-ROS clock calibration.'])


def publication_worker_health(samples):
    """Validate the actual worker summary API, including all failure latches.

    A later summary cannot erase a failure recorded earlier in this finite
    window. Replacement/expiry counters are retained, not called failures.
    """
    errors=[]
    if not samples:return dict(passed=False,samples=0,errors=['publication worker diagnostics unavailable'])
    required=('failure','worker_alive','pending','max_pending_pairs','closing',
              'publish_failures','rejected_after_failure','dropped_on_failure',
              'published','submitted','peak_pair_bytes','max_pair_bytes')
    for index,snapshot in enumerate(samples):
        missing=[key for key in required if key not in snapshot]
        if missing:
            errors.append('sample%d missing worker summary fields: %s'%(index,','.join(missing)))
            continue
        bounded=(snapshot['pending'] in (0,1) and snapshot['max_pending_pairs']==1 and
                 isinstance(snapshot['peak_pair_bytes'],(int,float)) and
                 isinstance(snapshot['max_pair_bytes'],(int,float)) and
                 0<=snapshot['peak_pair_bytes']<=snapshot['max_pair_bytes'])
        no_failure=(snapshot['failure'] is None and
                    all(snapshot[key]==0 for key in ('publish_failures','rejected_after_failure','dropped_on_failure')))
        counts_valid=(isinstance(snapshot['published'],int) and isinstance(snapshot['submitted'],int) and
                      0<=snapshot['published']<=snapshot['submitted'])
        if not bounded:errors.append('sample%d exceeded or lacked publication bounds'%index)
        if not no_failure:errors.append('sample%d recorded a publication worker failure'%index)
        if not counts_valid:errors.append('sample%d has invalid publication counters'%index)
        if snapshot['worker_alive'] is not True or snapshot['closing'] is not False:
            errors.append('sample%d publication worker is not running'%index)
    latest=samples[-1]
    if not isinstance(latest.get('published'),int) or latest['published']<=0:
        errors.append('no successful worker publication recorded')
    return dict(passed=not errors,samples=len(samples),errors=errors[:16],error_count=len(errors))


def summarize_reads(path):
    rows=[json.loads(line) for line in Path(path).read_text().splitlines()]
    starts=[row['read_started_monotonic'] for row in rows]
    # Exclude the first second of startup from the measured steady read rate.
    steady=[r for r in rows if starts and r['read_started_monotonic']>=starts[0]+1.]
    times=[r['read_started_monotonic'] for r in steady]
    evidence=[(r.get('existing_read_diagnostics') or {}).get('stationary_rgb_evidence',{}) for r in rows]
    publication=[r['publication_worker'] for r in rows if isinstance(r.get('publication_worker'),dict)]
    diagnostics=[r.get('existing_read_diagnostics') or {} for r in rows]
    read_durations=[r['read_completed_monotonic']-r['read_started_monotonic'] for r in rows
                    if isinstance(r.get('read_completed_monotonic'),(int,float))]
    sdk_waits=[d.get('host_wait',{}).get('elapsed_sec') for d in diagnostics]
    sdk_wait_stages=[d.get('stages_sec',{}).get('wait_for_frames') for d in diagnostics]
    maxima=[p.get('maximum_publish_seconds') for p in publication]
    maxima=[float(v) for v in maxima if isinstance(v,(int,float)) and math.isfinite(v)]
    return dict(rows=len(rows),steady_read_fps=rate(times),first_read_monotonic=starts[0] if starts else None,
                last_read_monotonic=starts[-1] if starts else None,
                read_wall_duration_sec=duration_summary(read_durations),
                read_start_interval_sec=duration_summary([b-a for a,b in zip(starts,starts[1:])]),
                sdk_host_wait_elapsed_sec=duration_summary(sdk_waits),
                sdk_wait_for_frames_stage_sec=duration_summary(sdk_wait_stages),
                result_counts=dict(Counter((r.get('existing_read_diagnostics') or {}).get('result','missing') for r in rows)),
                visual_qualified_frames=sum(bool(e.get('qualified')) for e in evidence),
                visual_reason_counts=dict(Counter(str(e.get('reason','missing')) for e in evidence)),
                temporal_active_frames=sum(bool((r.get('stationary_temporal') or {}).get('active')) for r in rows),
                raw_streams={name:summarize_raw_stream(rows,name) for name in ('color','depth')},
                publication_worker_last=publication[-1] if publication else None,
                publication_worker_samples=len(publication),
                publication_worker_health=publication_worker_health(publication),
                publication_worker_maximum_publish_seconds=max(maxima) if maxima else None,
                publication_latency_limitation='Worker age admission limit applies before publisher entry; an admitted publisher may block longer and cannot be cancelled.',
                full_raw_source_and_read_stage_metadata_file=str(path),
                upstream_fault_resolution_verified=False,
                validation_scope='Finite-window average read/publication cadence, metadata accounting and worker health only; this cannot establish that upstream USB/device/host stalls are resolved.',
                source_stamp_semantics='host read completion ROS stamp; SDK global_time is not calibrated to ROS exposure time')


def validate_loaded_modules(path, source_manifest):
    loaded=json.loads(Path(path).read_text())
    expected={
        'alicia_flexible_grasp.vision.realsense_manager':SOURCES[1],
        'alicia_flexible_grasp.vision.visual_stationary':SOURCES[2],
        'alicia_flexible_grasp.vision.latest_rgbd_publication':PUBLICATION_SOURCE}
    for name,source in expected.items():
        actual=loaded.get(name,{})
        if actual.get('path')!=str(source.resolve()) or actual.get('sha256')!=source_manifest[str(source)]:
            raise RuntimeError('loaded camera module does not match verified source: '+name)
    return loaded


def static_validation_passes(summary, reads, pubs, profile, rgb, depth):
    counts=summary.get('counts',{})
    accounted=(counts.get('observed',0)>0 and
               counts.get('observed')==counts.get('enqueued')==counts.get('written')==reads.get('rows') and
               summary.get('queued_unwritten')==0)
    # Subscriber receipt-window boundaries can differ by one pair. Large
    # unmatched sets indicate a publication/pairing problem and must fail.
    paired=(len(rgb&depth)>=2 and len(rgb-depth)<=2 and len(depth-rgb)<=2)
    return bool(summary.get('original_read_restored') is True and not summary.get('errors') and
                accounted and paired and
                all(counts.get(k,0)==0 for k in ('queue_dropped','copy_errors','serialization_errors','write_errors')) and
                reads.get('steady_read_fps') is not None and 27<=reads['steady_read_fps']<=33 and
                all(pubs.get(k,{}).get('fps') is not None and 18<=pubs[k]['fps']<=22 for k in ('rgb','depth')) and
                reads.get('visual_qualified_frames',0)>0 and reads.get('temporal_active_frames',0)>0 and
                reads.get('publication_worker_health',{}).get('passed') is True and
                profile.get('visual_stationarity_required') is True)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--old-pid',type=int,default=9896)
    parser.add_argument('--old-parent-pid',type=int,default=9685)
    parser.add_argument('--diagnostic-seconds',type=float,default=45.)
    args=parser.parse_args(argv)
    if not args.execute:parser.error('--execute is required; importing/compiling this file never changes ROS')
    if not 10.<=args.diagnostic_seconds<=120.:raise RuntimeError('diagnostic window must be10..120 seconds')
    if not MOUNT.is_mount():raise RuntimeError('SanStand unavailable; no fallback')
    subprocess.run(['systemctl','--user','show-environment'],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,timeout=5)
    stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    out=OUTROOT/('camera_visual_gate_reload_'+stamp);out.mkdir();(out/'ros').mkdir()
    os.environ['ROS_LOG_DIR']=str(out/'ros')
    import rospy
    from sensor_msgs.msg import Image,JointState
    from std_msgs.msg import Header,String
    rospy.init_node('camera_visual_gate_reload_observer',anonymous=True,disable_signals=True)
    state=dict(out=str(out),camera_loaded=False,static_validation_passed=False,no_robot_commands_published=True)
    observations={'rgb':[],'depth':[],'sdk':None,'epoch':None};fatal=[];subs=[]
    freeze={'sdk':None,'epoch':None}
    def image_callback(kind,msg):
        observations[kind].append(dict(received_monotonic=time.monotonic(),received_ros_time_ns=rospy.Time.now().to_nsec(),
                                        source_stamp_ns=msg.header.stamp.to_nsec(),
                                        frame_id=msg.header.frame_id,width=msg.width,height=msg.height,encoding=msg.encoding))
    def sdk_callback(msg):
        values=dict(zip(msg.name,msg.position))
        if set(values)!=set('Joint%d'%i for i in range(1,7)):fatal.append('invalid SDK message during camera reload');return
        words=[int(round((values['Joint%d'%i]+3.141592653589793)/(2*3.141592653589793)*4096)) for i in range(1,7)]
        if freeze['sdk'] is not None and words!=freeze['sdk']:fatal.append('SDK target changed during camera-only reload')
        observations['sdk']=words
    def epoch_callback(msg):
        ns=msg.stamp.to_nsec()
        if freeze['epoch'] is not None and ns!=freeze['epoch']:fatal.append('driver command epoch changed during camera-only reload')
        observations['epoch']=ns
    def guard():
        if fatal:raise RuntimeError(fatal[0])
        if rospy.is_shutdown():raise RuntimeError('observer ROS shutdown')
        if not MOUNT.is_mount():raise RuntimeError('SanStand disappeared')
    def interrupted(sig,_frame):raise RuntimeError('camera loader interrupted by signal%d'%sig)
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGALRM):signal.signal(sig,interrupted)
    signal.alarm(int(args.diagnostic_seconds+70))
    try:
        cfg=rospy.get_param('/camera');before_params=json.loads(json.dumps(cfg))
        if cfg.get('fps')!=30 or cfg.get('publish_fps')!=20 or cfg.get('simulate') or cfg.get('fallback_to_simulation'):
            raise RuntimeError('expected existing real30Hz read/20Hz publish configuration without simulation fallback')
        old=identity(args.old_pid)
        if old['ppid']!=args.old_parent_pid or node_pid(rospy,NODE)!=args.old_pid:
            raise RuntimeError('old camera PID/parent/master identity mismatch')
        environment=camera_environment(args.old_pid)
        source_manifest={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in SOURCES}
        unit='alicia-camera-visual-gate-'+stamp.lower()
        command=service_argv(unit,old,out,environment,args.diagnostic_seconds)
        driver_before=identity(node_pid(rospy,'/alicia_d_driver_node'))
        (out/'load_inputs.json').write_text(json.dumps(dict(old_camera=old,driver=driver_before,camera_parameters=cfg,
            retained_environment=environment,source_sha256=source_manifest,systemd_argv=command),indent=2)+'\n')
        for kind,key,default in [('rgb','color_topic','/supervisor/camera/color/image_raw'),('depth','depth_topic','/supervisor/camera/depth/image_raw')]:
            subs.append(rospy.Subscriber(cfg.get(key,default),Image,lambda msg,k=kind:image_callback(k,msg),queue_size=5,buff_size=2**24))
        subs.append(rospy.Subscriber('/alicia_d/sdk_command',JointState,sdk_callback,queue_size=20))
        subs.append(rospy.Subscriber('/alicia_d/control_reference_epoch',Header,epoch_callback,queue_size=1))
        wait(lambda:observations['sdk'] is not None and observations['epoch'] is not None,3,'current unchanged driver hold',guard)
        freeze.update(sdk=list(observations['sdk']),epoch=observations['epoch'])
        if not alive(old) or node_pid(rospy,NODE)!=old['pid']:raise RuntimeError('old camera identity changed before signal')
        guard();os.kill(old['pid'],signal.SIGINT);state['old_camera_sigint_pid']=old['pid']
        wait(lambda:not alive(old),12,'specified old camera exit',guard)
        if not Path('/proc/%d'%args.old_parent_pid).exists():raise RuntimeError('specified camera parent process unexpectedly exited')
        print(json.dumps(dict(event='starting_camera_only',unit=unit,argv=command),ensure_ascii=False),flush=True)
        result=subprocess.run(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=10)
        (out/'systemd_start.txt').write_text(result.stdout)
        if result.returncode:raise RuntimeError('camera systemd start failed: '+result.stdout)
        def new_camera():
            try:
                pid=int(subprocess.check_output(['systemctl','--user','show',unit,'--property=MainPID','--value'],text=True,timeout=2).strip())
                if pid<=0 or pid==old['pid'] or node_pid(rospy,NODE)!=pid:return False
                new=identity(pid)
                if new['exe_sha256']!=old['exe_sha256']:raise RuntimeError('camera Python interpreter changed')
                if str(OBSERVER) not in new['argv']:raise RuntimeError('unexpected new camera entry point')
                return new
            except (CameraRegistrationPending,subprocess.SubprocessError,OSError,xmlrpc.client.Error):return False
        new=wait(new_camera,15,'new camera service/ROS PID binding',guard)
        state.update(camera_loaded=True,new_camera=new,systemd_user_unit=unit)
        summary_path=out/'read_diagnostics.summary.json'
        def completed_diagnostic():
            if not summary_path.exists():return False
            try:return json.loads(summary_path.read_text())
            except json.JSONDecodeError:return False
        summary=wait(completed_diagnostic,args.diagnostic_seconds+25,'bounded first-read diagnostic completion',guard)
        reads=summarize_reads(out/'read_diagnostics.jsonl')
        loaded_modules=validate_loaded_modules(out/'loaded_observer_modules.json',source_manifest)
        t0=reads['first_read_monotonic'];t1=reads['last_read_monotonic']
        pubs={}
        for kind in ('rgb','depth'):
            rows=[r for r in observations[kind] if t0 is not None and t0+1<=r['received_monotonic']<=t1]
            pubs[kind]=summarize_publication(rows)
        rgb={r['source_stamp_ns'] for r in observations['rgb'] if t0 is not None and t0+1<=r['received_monotonic']<=t1}
        depth={r['source_stamp_ns'] for r in observations['depth'] if t0 is not None and t0+1<=r['received_monotonic']<=t1}
        after=rospy.get_param('/camera');profile=after.get('runtime_profile',{}).get('stationary_temporal',{})
        if stable_camera_configuration(before_params)!=stable_camera_configuration(after):raise RuntimeError('camera geometry/configuration changed during reload')
        driver_after=identity(node_pid(rospy,'/alicia_d_driver_node'))
        if driver_after['pid']!=driver_before['pid'] or driver_after['start_ticks']!=driver_before['start_ticks'] or driver_after['exe_sha256']!=driver_before['exe_sha256']:
            raise RuntimeError('driver process changed during camera-only reload')
        current_hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in SOURCES}
        if current_hashes!=source_manifest:raise RuntimeError('camera source changed during verification')
        guard()
        passed=static_validation_passes(summary,reads,pubs,profile,rgb,depth)
        state.update(static_validation_passed=bool(passed),read_observer=summary,reads=reads,publication=pubs,
                     rgb_depth_common_source_stamps=len(rgb&depth),rgb_only_stamps=len(rgb-depth),depth_only_stamps=len(depth-rgb),
                     driver_after=driver_after,unchanged_sdk_words=observations['sdk'],unchanged_driver_epoch=observations['epoch'],
                     loaded_stationary_profile=profile,source_sha256=current_hashes,loaded_modules=loaded_modules,
                     upstream_fault_resolution_verified=False,
                     validation_scope=reads['validation_scope'],
                     timestamp_limitation='headers record host read completion, not publication/exposure time; SDK global_time/exposure-to-ROS correspondence is not calibrated by this repair')
        (out/'camera_parameters_after.json').write_text(json.dumps(after,indent=2)+'\n')
        if not passed:raise RuntimeError('camera loaded, but recorded static fps/visual-gate validation did not fully pass; inspect preserved diagnostics')
    except BaseException as exc:
        state.update(static_validation_passed=False,error=str(exc));raise
    finally:
        signal.alarm(0)
        for sub in subs:sub.unregister()
        (out/'publication_observations.json').write_text(json.dumps(observations,indent=2)+'\n')
        try:save_result_with_final_guard(out/'result.json',state,guard)
        finally:print(json.dumps(state,ensure_ascii=False,allow_nan=False),flush=True)


if __name__=='__main__':main()
