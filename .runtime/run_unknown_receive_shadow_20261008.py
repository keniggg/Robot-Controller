#!/usr/bin/env python3
"""Bounded isolated original perception consumer, not a production target restart.

Import is pure. The explicit live entrypoint creates only a private diagnostic
selection and remapped outputs. It adds one RGB-D/TF/joint consumer and CPU load.
No images are stored; transport and detector-status evidence are metadata only.
"""
import argparse
from collections import Counter
import hashlib
import importlib.util
import inspect
import json
import math
import os
from pathlib import Path
import queue
import re
import runpy
import signal
import sys
import threading
import time
import xmlrpc.client

W=Path(__file__).resolve().parents[1]
EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
SOURCE=W/'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py'
SOURCE_SHA='7e82145c91c33edf86566c270da150b85da7dcedc36aa97ea420bb4de66e2a77'
TRACE=W/'.runtime/unknown_transport_trace_20261008.py'
CANDIDATE=W/'.runtime/unknown_pruned_copy_candidate_20261008.py'
TOTAL_BYTES=64*1024*1024
TRACE_BYTES=50*1024*1024
STATUS_BYTES=12*1024*1024
OUTPUTS=tuple('/perception/unknown/'+name for name in (
    'object','object_mask','object_pose_camera','object_pose_base','object_detected',
    'raw_object_detected','detector_status'))+('/perception/rgb_acquisition/reference_status',)
PRIVATE_INPUTS=('/grasp_mode/selection','/supervisor/rgb_acquisition/reference_request',
                '/supervisor/rgb_acquisition/status')
PRODUCTION_STATUS='/perception/unknown/detector_status'


def require(value,reason):
    if not value: raise ValueError(reason)


def digest(raw): return hashlib.sha256(raw).hexdigest()
def encoded(value): return (json.dumps(value,ensure_ascii=False,sort_keys=True,allow_nan=False,separators=(',',':'))+'\n').encode()
def write_new(path,value):
    with path.open('xb') as handle: handle.write(encoded(value))


def own_start_ticks():
    text=Path('/proc/self/stat').read_text()
    # Fields after the final comm parenthesis start at field 3; starttime is 22.
    return int(text[text.rfind(')')+2:].split()[19])


def validate_deserializer_option(mode,candidate_sha256):
    require(mode in ('original','skip_pruned_copies'),'invalid_deserializer_variant')
    if mode=='original':
        require(candidate_sha256 is None,'candidate_sha_requires_candidate_variant')
    else:
        require(isinstance(candidate_sha256,str) and re.fullmatch('[0-9a-f]{64}',candidate_sha256) is not None,
                'candidate_variant_requires_frozen_sha256')


class CandidateAlias:
    """Private process alias only. The outer transport trace must stop first."""
    def __init__(self,tcp,helper,image_class,original):
        require(tcp.deserialize_messages is original,'original_deserializer_alias_changed')
        self.tcp=tcp;self.original=original;self.finished=False;self.restored=False;self.deactivated=False;self.conflicts=[]
        candidate=helper.make_candidate(original,image_class)
        require(tuple(inspect.signature(candidate).parameters)==('b','msg_queue','data_class','queue_size','max_msgs','start'),
                'candidate_deserializer_ABI_mismatch')
        require(callable(getattr(candidate,'snapshot_statistics',None)),'candidate_statistics_missing')
        require(callable(getattr(candidate,'deactivate',None)),'candidate_deactivation_missing')
        self.candidate=candidate
        tcp.deserialize_messages=candidate

    def restore(self):
        if not self.finished:
            self.finished=True
            try:
                self.candidate.deactivate()
                self.deactivated=self.statistics().get('active') is False
                if not self.deactivated:self.conflicts.append('candidate_deactivation_not_confirmed')
            except Exception as exc:
                self.conflicts.append('candidate_deactivation_failed:'+repr(exc))
            if self.tcp.deserialize_messages is self.candidate:
                self.tcp.deserialize_messages=self.original;self.restored=True
            else:
                self.conflicts.append('tcpros_base.deserialize_messages_owner_changed')
        return dict(original_alias_restored=self.restored,candidate_deactivated=self.deactivated,
                    restore_conflicts=list(self.conflicts))

    def statistics(self):
        # Values are provided by the implementation actually used, never
        # estimated from a second scan of the socket buffer.
        value=self.candidate.snapshot_statistics()
        require(isinstance(value,dict),'candidate_statistics_not_object')
        return json.loads(json.dumps(value,allow_nan=False))


def candidate_statistics_validation(statistics,restoration):
    """A selected variant is not evidence that its optimized branch ran."""
    reasons=[]
    required=('calls_started','calls_completed','calls_failed','inflight','optimized_calls','fallback_calls',
              'scanned_payload_bytes','retained_payload_bytes','skipped_payload_bytes',
              'materialized_payload_bytes','complete_messages_scanned','retained_messages')
    well_formed=isinstance(statistics,dict) and all(type(statistics.get(k)) is int and statistics[k]>=0 for k in required)
    if not well_formed:reasons.append('invalid_candidate_statistics')
    if restoration.get('original_alias_restored') is not True or restoration.get('restore_conflicts'):
        reasons.append('candidate_alias_not_restored')
    if restoration.get('candidate_deactivated') is not True or not isinstance(statistics,dict) or statistics.get('active') is not False:
        reasons.append('candidate_not_deactivated')
    if well_formed:
        if statistics['calls_started']!=statistics['calls_completed']+statistics['calls_failed']+statistics['inflight']:
            reasons.append('candidate_call_accounting_mismatch')
        if statistics['inflight']:reasons.append('candidate_calls_still_inflight')
        if statistics['calls_failed']:reasons.append('candidate_calls_failed')
        if not statistics['optimized_calls']:reasons.append('candidate_not_exercised')
        if not statistics['complete_messages_scanned']:reasons.append('candidate_no_complete_image_scanned')
        if not statistics['retained_messages']:reasons.append('candidate_no_complete_image_retained')
    used=bool(well_formed and statistics['optimized_calls']>0
              and statistics['complete_messages_scanned']>0 and statistics['retained_messages']>0)
    return dict(passed=not reasons,reasons=reasons,
        candidate_used=used,
        copy_pruning_observed=bool(used and statistics['skipped_payload_bytes']>0),
        no_skip_interpretation='An exercised candidate with zero skipped bytes does not demonstrate avoided payload copies.')


def private_namespace(pid,created_ns):
    require(type(pid) is int and pid>0 and type(created_ns) is int and created_ns>0,'invalid_private_identity')
    return '/diagnostics/unknown_receive_shadow_%d_%d'%(pid,created_ns)


def remappings(namespace,observe_production=False):
    require(re.fullmatch(r'/diagnostics/unknown_receive_shadow_[1-9][0-9]*_[1-9][0-9]*',namespace) is not None,
            'invalid_private_namespace')
    require(type(observe_production) is bool,'invalid_production_observer_flag')
    # Keep original output names unmapped in the ROS client. Topic AND
    # _TopicImpl resolve names; an alias -> production -> shadow is unsafe.
    # Original publishers are instead remapped before ROS object construction.
    return {topic:namespace+topic for topic in PRIVATE_INPUTS}


class PrivatePublisherRos:
    """Only original-node construction uses this proxy; other APIs are identical."""
    def __init__(self,rospy,namespace):
        remappings(namespace)
        self.original=rospy;self.namespace=namespace

    def __getattr__(self,name):return getattr(self.original,name)

    def Publisher(self,name,*args,**kwargs):
        require(name in OUTPUTS,'unexpected_original_publisher:'+str(name))
        return self.original.Publisher(self.namespace+name,*args,**kwargs)


def construct_isolated_node(node_class,rospy,namespace):
    scope=node_class.__init__.__globals__
    require(scope.get('rospy') is rospy,'original_constructor_rospy_binding_changed')
    original=scope['rospy'];scope['rospy']=PrivatePublisherRos(rospy,namespace)
    try:return node_class()
    finally:scope['rospy']=original


def private_selection(stamp_ns):
    require(type(stamp_ns) is int and stamp_ns>0,'invalid_private_selection_clock')
    return dict(mode='unknown',strategy='observe_return',generation=1,stamp_ns=str(stamp_ns))


def verify_resolutions(resolve,namespace,mappings,camera_topics):
    for topic in PRIVATE_INPUTS:
        require(resolve(topic)==namespace+topic,'unsafe_topic_or_parameter_resolution:'+topic)
    for topic in OUTPUTS:
        require(resolve(topic)==topic and resolve(namespace+topic)==namespace+topic,
                'unsafe_output_or_observer_remapping:'+topic)
    require(len(camera_topics)==2 and len(set(camera_topics))==2,'invalid_RGBD_topics')
    for topic in tuple(camera_topics)+('/tf','/tf_static','/alicia_d/accepted_joint_states','/alicia_d/control_reference_epoch',
                                       '/camera','/unknown_perception','/handeye','/robot_description'):
        require(isinstance(topic,str) and topic.startswith('/') and resolve(topic)==topic,'changed_readonly_input:'+str(topic))


def verify_node_topics(node,namespace,camera_topics):
    require({p.resolved_name for p in node.pubs.values()}=={namespace+t for t in OUTPUTS},'unsafe_original_output')
    require(all(p.impl.resolved_name==p.resolved_name for p in node.pubs.values()),'publisher_implementation_name_mismatch')
    for attribute,topic in (('selection_sub',PRIVATE_INPUTS[0]),('rgb_reference_sub',PRIVATE_INPUTS[1]),
                            ('rgb_acquisition_sub',PRIVATE_INPUTS[2])):
        sub=getattr(node,attribute)
        require(sub.resolved_name==sub.impl.resolved_name==namespace+topic,'unsafe_private_subscriber:'+attribute)
    require(node.color_sub.sub.resolved_name==camera_topics[0] and node.depth_sub.sub.resolved_name==camera_topics[1],
            'unexpected_original_camera_inputs')


def cleanup_private_selection(rospy,key,expected,namespace):
    """Never delete a namespace or a value not created by this invocation."""
    require(key==namespace+'/grasp_mode/selection','unsafe_private_parameter_cleanup')
    sentinel=object();current=rospy.get_param(key,sentinel)
    if current is sentinel: return 'already_absent'
    if current!=expected: return 'changed_private_value_preserved'
    rospy.delete_param(key)
    return 'deleted_own_exact_private_selection'


def trace_window_complete(summary,started,deadline):
    stopped=summary.get('stopped_monotonic')
    return bool(summary.get('sealed') is True and summary.get('observer_integrity_passed') is True
        and summary.get('original_method_bindings_restored') is True and not summary.get('restore_conflicts')
        and summary.get('stop_reason') in ('deadline','duration_limit')
        and summary.get('started_monotonic')==started and summary.get('deadline_monotonic')==deadline
        and type(stopped) in (int,float) and math.isfinite(stopped) and stopped>=deadline)


class StatusRecorder:
    def __init__(self,output,maximum_bytes=STATUS_BYTES,maximum_rows=3000):
        self.path=output/'detector_status.jsonl';self.partial=output/'detector_status.jsonl.partial'
        self.maximum_bytes,self.maximum_rows=maximum_bytes,maximum_rows
        self.lock=threading.Lock();self.queue=queue.Queue(maxsize=256);self.closed=False
        self.reserved_bytes=0;self.queued_bytes=0;self.written_bytes=0;self.written_rows=0
        self.counts=Counter();self.error=None;self.limit_reached=False
        self.handle=self.partial.open('xb');self.checksum=hashlib.sha256()
        self.worker=threading.Thread(target=self._write,name='shadow_status_writer',daemon=True);self.worker.start()

    def offer(self,data,kind,topic,receipt):
        if not isinstance(data,str) or kind not in ('shadow','production'):
            with self.lock: self.counts['invalid_input']+=1
            return
        original=data.encode('utf8')
        if len(original)>128*1024:
            with self.lock: self.counts['oversize_status']+=1;self.limit_reached=True
            return
        raw=encoded(dict(kind=kind,topic=topic,receipt=receipt,original_status_data=data,
            original_status_utf8_sha256=digest(original),authority='none_diagnostic_delivery_only'))
        with self.lock:
            if self.closed: self.counts['after_close']+=1;return
            if self.reserved_bytes+len(raw)>self.maximum_bytes or sum(self.counts[k] for k in ('accepted_shadow','accepted_production'))>=self.maximum_rows:
                self.counts['budget_rejected']+=1;self.limit_reached=True;return
            if self.queue.full() or self.queued_bytes+len(raw)>1024*1024:
                self.counts['queue_rejected']+=1;self.limit_reached=True;return
            self.reserved_bytes+=len(raw);self.queued_bytes+=len(raw);self.counts['accepted_'+kind]+=1
            self.queue.put_nowait(raw)

    def _write(self):
        try:
            while True:
                try: raw=self.queue.get(timeout=.05)
                except queue.Empty:
                    with self.lock:
                        if self.closed: break
                    continue
                self.handle.write(raw);self.checksum.update(raw)
                with self.lock:
                    self.queued_bytes-=len(raw);self.written_bytes+=len(raw);self.written_rows+=1
                self.queue.task_done()
            self.handle.close();os.replace(self.partial,self.path)
        except BaseException as exc:
            self.error=type(exc).__name__+': '+str(exc)
            try:self.handle.close()
            except Exception:pass

    def finish(self,timeout=3.):
        with self.lock:self.closed=True
        self.worker.join(timeout)
        with self.lock:
            return dict(sealed=not self.worker.is_alive() and self.error is None and self.path.is_file(),
                bytes=self.written_bytes,rows=self.written_rows,sha256=self.checksum.hexdigest(),
                reserved_bytes=self.reserved_bytes,queued_bytes=self.queued_bytes,counts=dict(self.counts),
                error=self.error,writer_alive=self.worker.is_alive(),limit_reached=self.limit_reached,
                raw_json_preserved=True,contains_pixels=False)


class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self,host):
        conn=super().make_connection(host);conn.timeout=1.;return conn


def original_selection(master_uri,caller):
    # Direct read avoids this process's intentional private parameter remapping.
    proxy=xmlrpc.client.ServerProxy(master_uri,transport=TimeoutTransport(),allow_none=True)
    try:
        code,message,value=proxy.getParam(caller,'/grasp_mode/selection')
        require(code==1,'original_selection_unavailable:'+str(message))
        return value
    finally:proxy('close')()


def configure_logs(output):
    os.environ['ROS_LOG_DIR']=str(output/'ros')
    os.environ['PYTHONDONTWRITEBYTECODE']='1'
    os.environ['OPENBLAS_NUM_THREADS']='1'
    os.environ['OMP_NUM_THREADS']='1'
    config=output/'logging.conf'
    config.write_text('''[loggers]
keys=root
[handlers]
keys=fileHandler
[formatters]
keys=defaultFormatter
[logger_root]
level=WARNING
handlers=fileHandler
[handler_fileHandler]
class=handlers.RotatingFileHandler
level=WARNING
formatter=defaultFormatter
args=(os.environ['ROS_LOG_FILENAME'],'a',131072,1)
[formatter_defaultFormatter]
format=[%(name)s][%(levelname)s] %(asctime)s: %(message)s
''')
    os.environ['ROS_PYTHON_LOG_CONFIG_FILE']=str(config)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--seconds',type=float,default=90.)
    parser.add_argument('--trace-sha256',required=True)
    parser.add_argument('--observe-production-status',action='store_true')
    parser.add_argument('--deserializer',choices=('original','skip_pruned_copies'),default='original')
    parser.add_argument('--candidate-sha256')
    args=parser.parse_args(argv)
    require(math.isfinite(args.seconds) and 0<args.seconds<=90,'duration_out_of_bounds')
    require(re.fullmatch('[0-9a-f]{64}',args.trace_sha256) is not None,'invalid_trace_sha256')
    validate_deserializer_option(args.deserializer,args.candidate_sha256)
    output=args.output.resolve()
    require(Path('/media/zhuyupei/SanStand').is_mount() and EXT.resolve() in output.parents
            and output.parent.is_dir() and not output.exists(),'new_external_output_required_no_fallback')
    source_bytes=SOURCE.read_bytes();trace_bytes=TRACE.read_bytes()
    require(digest(source_bytes)==SOURCE_SHA,'original_perception_source_changed')
    require(digest(trace_bytes)==args.trace_sha256,'trace_source_changed')
    namespace=private_namespace(os.getpid(),time.time_ns());mapping=remappings(namespace,args.observe_production_status)
    frozen={str(SOURCE):SOURCE_SHA,str(TRACE):args.trace_sha256,str(Path(__file__).resolve()):digest(Path(__file__).read_bytes())}
    if args.deserializer=='skip_pruned_copies':
        require(digest(CANDIDATE.read_bytes())==args.candidate_sha256,'candidate_source_changed')
        frozen[str(CANDIDATE)]=args.candidate_sha256
    output.mkdir();configure_logs(output)
    stop=threading.Event();signals=[];handles=[];node=None;session=None;statuses=None;rospy=None;candidate=None
    private_key=namespace+'/grasp_mode/selection';selection=None;private_created=False;error=None;reason='startup'
    report=dict(schema=1,diagnostic_only=True,motion_authorized=False,production_target_created=False,
        production_selection_changed_by_wrapper=False,production_node_restart_performed=False,
        robot_commands_sent=False,temperature_queried=False,pixels_recorded=False,namespace=namespace,
        remappings=mapping,publisher_remappings={topic:namespace+topic for topic in OUTPUTS},
        publisher_remapping_scope='Original class constructor Publisher calls only; ROS global output names are not remapped.',
        deserializer_variant=args.deserializer,candidate_source_sha256=args.candidate_sha256,
        maximum_duration_sec=args.seconds,maximum_directory_bytes=TOTAL_BYTES,
        source_sha256=frozen,extra_load='One original perception process plus RGB-D/TF/joint subscriptions, trace hooks, status observer and bounded writer threads.',
        private_generation_resets_performed=0,ready=False,complete=False,errors=[],cleanup_errors=[])
    for signum in (signal.SIGINT,signal.SIGTERM):
        signal.signal(signum,lambda value,_frame:(signals.append(value),stop.set()))
    try:
        import rospy as ros
        rospy=ros
        from std_msgs.msg import String
        ros_argv=[str(Path(__file__).resolve()),'__ns:='+namespace]+[key+':='+value for key,value in mapping.items()]
        rospy.init_node('unknown_receive_shadow',argv=ros_argv,anonymous=False,disable_signals=True,disable_rosout=True)
        caller=rospy.get_name();master_uri=os.environ.get('ROS_MASTER_URI','http://127.0.0.1:11311/')
        before=original_selection(master_uri,caller);report['production_selection_before']=before
        camera=rospy.get_param('/camera',{})
        topics=(camera.get('color_topic','/supervisor/camera/color/image_raw'),camera.get('depth_topic','/supervisor/camera/depth/image_raw'))
        verify_resolutions(rospy.resolve_name,namespace,mapping,topics)
        report['camera_topics']=list(topics)
        report['camera_parameters_at_start']=camera
        report['unknown_parameters_at_start']=rospy.get_param('/unknown_perception',{})
        report['selection_parameter_resolution']=rospy.resolve_name('/grasp_mode/selection')
        require(not rospy.has_param(private_key),'private_parameter_already_exists')
        # Do not execute original __main__: it would create the production name.
        definitions=runpy.run_path(str(SOURCE),run_name='unknown_receive_shadow_definition')
        cls=definitions['UnknownTabletopPerception']
        require(Path(cls.__init__.__code__.co_filename).resolve()==SOURCE.resolve(),'unexpected_original_class_source')
        definitions['cv2'].setNumThreads(1)
        require(definitions['cv2'].getNumThreads()==1,'unexpected_opencv_threads')
        spec=importlib.util.spec_from_file_location('unknown_shadow_transport_trace',TRACE)
        trace=importlib.util.module_from_spec(spec);sys.modules[spec.name]=trace;spec.loader.exec_module(trace)
        if args.deserializer=='skip_pruned_copies':
            # Install before the standard trace so its deserialize_messages
            # event includes the actual candidate, with unchanged stage names.
            tcp=importlib.import_module('rospy.impl.tcpros_base')
            original_msg=importlib.import_module('rospy.msg')
            from sensor_msgs.msg import Image
            require(tcp.deserialize_messages is original_msg.deserialize_messages,'unexpected_original_deserializer_alias')
            for source in (Path(inspect.getfile(original_msg.deserialize_messages)).resolve(),Path(tcp.__file__).resolve()):
                frozen[str(source)]=digest(source.read_bytes())
            candidate_spec=importlib.util.spec_from_file_location('unknown_shadow_pruned_copy_candidate',CANDIDATE)
            candidate_module=importlib.util.module_from_spec(candidate_spec)
            sys.modules[candidate_spec.name]=candidate_module;candidate_spec.loader.exec_module(candidate_module)
            candidate=CandidateAlias(tcp,candidate_module,Image,original_msg.deserialize_messages)
            report['candidate_binding']=dict(scope='Private process tcpros_base alias; exact sensor_msgs/Image only.',
                original_function_source=str(Path(inspect.getfile(original_msg.deserialize_messages)).resolve()),
                original_rospy_msg_function_unchanged=original_msg.deserialize_messages is candidate.original,
                queue_size_changed=False,buff_size_changed=False,synchronizer_changed=False,
                installed_before_transport_trace=True,
                statistics_scope='Actual calls admitted from candidate installation until cleanup deactivation; includes startup/exit boundaries, not exactly the trace window.')
        statuses=StatusRecorder(output)
        def on_status(message,scope_topic):
            kind,topic=scope_topic
            statuses.offer(message.data,kind,topic,dict(monotonic=time.monotonic(),wall_ns=time.time_ns(),
                ros_ns=rospy.Time.now().to_nsec(),publisher_callerid=(getattr(message,'_connection_header',None) or {}).get('callerid'),
                semantics='separate status subscriber delivery; not image socket arrival'))
        handles.append(rospy.Subscriber(namespace+PRODUCTION_STATUS,String,on_status,
            callback_args=('shadow',namespace+PRODUCTION_STATUS),queue_size=4,buff_size=262144))
        if args.observe_production_status:
            handles.append(rospy.Subscriber(PRODUCTION_STATUS,String,on_status,
                callback_args=('production',PRODUCTION_STATUS),queue_size=4,buff_size=262144))
        require(all(handle.resolved_name==handle.impl.resolved_name for handle in handles),
                'status_observer_implementation_name_mismatch')
        require(len({id(handle.impl) for handle in handles})==len(handles),'status_observers_share_implementation')
        session=trace.install_transport_trace(output/'transport',topics=topics,node_class=cls,
            duration_sec=args.seconds,maximum_bytes=TRACE_BYTES)
        require(math.isfinite(session.started_monotonic) and math.isfinite(session.deadline_monotonic)
                and 0<session.deadline_monotonic-session.started_monotonic<=args.seconds+1e-6,'invalid_trace_window')
        node=construct_isolated_node(cls,rospy,namespace);verify_node_topics(node,namespace,topics)
        require(not stop.is_set() and time.monotonic()<session.deadline_monotonic,'startup_used_capture_window')
        # Both the subscriber and cached parameter read resolve to this key.
        selection=private_selection(rospy.Time.now().to_nsec())
        private_created=True  # Cleanup also covers an RPC that writes then times out.
        rospy.set_param(private_key,selection)
        publisher=rospy.Publisher(private_key,String,queue_size=1,latch=True);handles.append(publisher)
        require(publisher.resolved_name==private_key,'unsafe_private_selection_publisher')
        publisher.publish(String(data=json.dumps(selection,separators=(',',':'))))
        acknowledgement_deadline=min(session.deadline_monotonic,time.monotonic()+3.)
        while (node.generation,node.selection_ns)!=(1,int(selection['stamp_ns'])) and not stop.is_set() and time.monotonic()<acknowledgement_deadline:
            stop.wait(.01)
        require((node.generation,node.selection_ns)==(1,int(selection['stamp_ns'])),'private_selection_not_received')
        report.update(private_selection=selection,private_selection_scope='isolated diagnostic only; not an actual mode selection or user alignment',
            capture_start_monotonic=session.started_monotonic,capture_deadline_monotonic=session.deadline_monotonic,
            started_monotonic=session.started_monotonic,deadline_monotonic=session.deadline_monotonic,
            ready_monotonic=time.monotonic(),ready=True,node_name=caller,pid=os.getpid(),start_ticks=own_start_ticks(),opencv_threads=1)
        write_new(output/'ready.json',report)
        print('READY '+str(output),flush=True);reason='duration_limit'
        while not stop.is_set() and time.monotonic()<session.deadline_monotonic:
            if rospy.is_shutdown():reason='ros_shutdown';break
            if statuses.error or statuses.limit_reached:reason='status_writer_error_or_budget';break
            if session.errors or (session.stopped and (session.stop_reason not in ('deadline','duration_limit')
                                                       or time.monotonic()<session.deadline_monotonic)):
                report['trace_early_stop_reason']=session.stop_reason
                reason='trace_error_or_early_stop';break
            print(json.dumps(dict(kind='heartbeat',monotonic=time.monotonic(),remaining_sec=max(0.,session.deadline_monotonic-time.monotonic()),
                private_generation=getattr(node,'generation',None),private_target_lost=bool(node.tracker.lost))),flush=True)
            stop.wait(min(1.,max(0.,session.deadline_monotonic-time.monotonic())))
        report['capture_finished_monotonic']=time.monotonic()
        if stop.is_set():reason='signal_or_requested_stop'
    except BaseException as exc:
        error=type(exc).__name__+': '+str(exc);report['errors'].append(error);reason='exception'
    finally:
        if session is not None:
            try:session.stop(reason=reason)
            except Exception as exc:report['cleanup_errors'].append('trace_stop:'+repr(exc))
        if candidate is not None:
            try:report['candidate_restore']=candidate.restore()
            except Exception as exc:report['cleanup_errors'].append('candidate_restore:'+repr(exc))
        if node is not None:
            try:node.timer.shutdown()
            except Exception as exc:report['cleanup_errors'].append('timer:'+repr(exc))
            for obj in [node.color_sub.sub,node.depth_sub.sub,node.selection_sub,node.rgb_interest_joint_sub,
                        node.rgb_interest_epoch_sub,node.rgb_reference_sub,node.rgb_acquisition_sub,node.listener,*node.pubs.values()]:
                try:obj.unregister()
                except Exception as exc:report['cleanup_errors'].append('own_unregister:'+repr(exc))
        for obj in handles:
            try:obj.unregister()
            except Exception as exc:report['cleanup_errors'].append('own_wrapper_unregister:'+repr(exc))
        if session is not None:
            try:report['trace']=session.finalize(timeout=3.)
            except Exception as exc:report['cleanup_errors'].append('trace_finalize:'+repr(exc))
        if statuses is not None:report['statuses']=statuses.finish(timeout=3.)
        if candidate is not None:
            try:
                # After deactivation, late trace closures call the unchanged
                # original. Only already admitted calls can update counters.
                drain_deadline=time.monotonic()+.5
                statistics=candidate.statistics()
                while statistics.get('inflight',0) and time.monotonic()<drain_deadline:
                    time.sleep(.005);statistics=candidate.statistics()
                report['candidate_statistics']=statistics
                report['candidate_statistics_snapshot_monotonic']=time.monotonic()
                report['candidate_validation']=candidate_statistics_validation(
                    report['candidate_statistics'],report.get('candidate_restore',{}))
            except Exception as exc:report['cleanup_errors'].append('candidate_statistics:'+repr(exc))
        else:
            report['candidate_validation']=dict(passed=args.deserializer=='original',candidate_used=False,
                copy_pruning_observed=False,applicable=False)
        if rospy is not None and private_created:
            try:report['private_parameter_cleanup']=cleanup_private_selection(rospy,private_key,selection,namespace)
            except Exception as exc:report['cleanup_errors'].append('private_parameter_cleanup:'+repr(exc))
        if rospy is not None and report.get('production_selection_before') is not None:
            try:
                report['production_selection_after']=original_selection(master_uri,caller)
                report['production_selection_unchanged']=report['production_selection_after']==report['production_selection_before']
            except Exception as exc:report['cleanup_errors'].append('original_selection_read:'+repr(exc))
        report.update(stop_reason=reason,signals=signals,sources_unchanged=all(Path(path).is_file() and digest(Path(path).read_bytes())==value for path,value in frozen.items()))
        report['directory_bytes_before_summary']=sum(path.stat().st_size for path in output.rglob('*') if path.is_file())
        report['hooks_restored']=bool(report.get('trace',{}).get('original_method_bindings_restored') is True
            and not report.get('trace',{}).get('restore_conflicts'))
        report['trace_full_window_passed']=bool(session is not None and trace_window_complete(
            report.get('trace',{}),session.started_monotonic,session.deadline_monotonic))
        report['complete']=bool(report['ready'] and reason=='duration_limit' and not report['errors'] and not report['cleanup_errors']
            and report['sources_unchanged'] and report.get('production_selection_unchanged') is True
            and report['trace_full_window_passed'] and report['hooks_restored']
            and report.get('candidate_validation',{}).get('passed') is True
            and report.get('statuses',{}).get('sealed') is True
            and not report.get('statuses',{}).get('limit_reached'))
        report['limits']=['Capture deadline is monotonic and cooperative; construction, RPC and cleanup may extend process lifetime. Parent must supervise only this diagnostic process.',
            'It adds an independent subscriber/processing load, so it is not identical to tracing the original production connection.',
            'Private outputs have no production router/planner consumers; serialization and downstream pressure may differ. No recurrence in this mirror does not prove the original consumer fixed.',
            'Only this private selection is created; no private result can authorize production motion.',
            'Status JSON may describe old latched production state; it is preserved without claiming freshness.',
            'Inactive cached bound wrappers can persist until this process exits; they must call original methods directly after trace stop.']
        raw=encoded(report)
        report['directory_budget_passed']=report['directory_bytes_before_summary']+len(raw)+4096<=TOTAL_BYTES
        if not report['directory_budget_passed']:report['complete']=False
        write_new(output/'summary.json',report)
        sealed_names=['summary.json']+(['ready.json'] if (output/'ready.json').is_file() else [])
        if report.get('statuses',{}).get('sealed') is True:sealed_names.append('detector_status.jsonl')
        (output/'SHA256SUMS').write_text(''.join(digest((output/name).read_bytes())+'  '+name+'\n' for name in sealed_names))
        print(json.dumps(dict(summary=str(output/'summary.json'),complete=report['complete'],error=error)),flush=True)
        if rospy is not None:
            try:rospy.signal_shutdown('owned diagnostic consumer ended')
            except Exception:pass
    return 0 if report['complete'] else 2


if __name__=='__main__':raise SystemExit(main())
