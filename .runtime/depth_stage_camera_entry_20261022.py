"""Verified diagnostic overlay for the existing camera entry; no SDK proxies."""
import argparse
import copy
import hashlib
import importlib
import importlib.machinery
import json
import os
from pathlib import Path
import runpy
import sys
import time
import types


SDK_BINDINGS = ('pipeline', 'context', 'config', 'align', 'spatial_filter',
                'temporal_filter', 'hole_filling_filter')


def diagnostic_trial(plan, label):
    rows=plan.get('diagnostic_trials',[dict(label='probe',preset=None)])
    matches=[row for row in rows if row['label']==label]
    if label not in ('probe','A1','B','A2') or len(matches)!=1 or matches[0]['preset'] not in (None,'high_accuracy','saved_baseline'):
        raise RuntimeError('diagnostic trial is not in the frozen plan')
    return matches[0]


def preset_snapshot(owner):
    preset=owner.mode_depth_preset
    result=dict(evidence=preset.evidence(),
        advanced_settings=json.loads(preset.advanced.serialize_json()),sensor_options={})
    for name in ('visual_preset','enable_auto_exposure','exposure','gain','power_line_frequency'):
        try:
            option=getattr(owner.rs.option,name)
            result['sensor_options'][name]=(preset.sensor.get_option(option)
                if preset.sensor.supports(option) else dict(unavailable='not_supported'))
        except Exception as exc:
            result['sensor_options'][name]=dict(unavailable=type(exc).__name__)
    return result


def install_instrumentation(manager_module, probe_module, plan, args, out, instances):
    trial=diagnostic_trial(plan,args.trial)
    manager=manager_module.RealSenseManager
    original_init=manager.__init__
    if trial['preset'] is not None:
        def init_for_trial(owner,*positional,**keywords):
            config=copy.deepcopy(keywords.get('mode_depth_preset_cfg') or {})
            if config.get('enabled') is not True:
                raise RuntimeError('diagnostic preset requires existing enabled preset support')
            config['unknown_preset']=trial['preset']
            keywords['mode_depth_preset_cfg']=config
            return original_init(owner,*positional,**keywords)
        manager.__init__=init_for_trial
    original_metadata=manager._raw_frame_diagnostics
    def with_exposure(owner,frames):
        data=original_metadata(owner,frames)
        for stream in ('color','depth'):
            for name in ('actual_exposure','gain_level','auto_exposure'):
                try:
                    frame=getattr(frames,'get_'+stream+'_frame')()
                    field=getattr(owner.rs.frame_metadata_value,name)
                    if frame.supports_frame_metadata(field):
                        data[stream]['metadata_raw'][name]=int(frame.get_frame_metadata(field))
                    else:data[stream]['unavailable'][name]='not_supported'
                except Exception as exc:data[stream]['unavailable'][name]=type(exc).__name__
        return data
    manager._raw_frame_diagnostics=with_exposure
    original_read=manager.read
    def read_with_finite_probe(owner):
        if not getattr(owner,'_finite_stage_probe_considered',False):
            owner._finite_stage_probe_considered=True
            if not instances:
                try:
                    if not callable(getattr(owner,'stationary_depth_provider',None)):
                        raise RuntimeError('original camera encoder provider is missing')
                    initial=preset_snapshot(owner)
                    if initial['evidence']['mode']!='unknown' or (trial['preset'] is not None
                            and initial['evidence']['preset']!=trial['preset']):
                        raise RuntimeError('actual diagnostic preset differs')
                    probe=probe_module.DepthStageProbe(out/'capture',
                        **plan.get('probe_limits',dict(frame_limit=10,seconds=30.,warmup_sec=5.)),
                        owner_metadata=dict(plan_sha256=args.sha256,pid=os.getpid(),sources=plan['sources'],
                            trial=args.trial,preset_snapshot=initial,stationary_provider_present=True))
                    instances.append(probe)
                    owner._finite_stage_probe=probe
                    owner._finite_stage_initial=initial
                    owner._finite_stage_skipped=0
                    owner._finite_stage_completion_written=False
                except Exception as exc:
                    print('Finite depth diagnostic setup failed: '+repr(exc),file=sys.stderr,flush=True)
        probe=getattr(owner,'_finite_stage_probe',None)
        if probe is not None:
            # The read loop is the only producer. Admission before this read
            # leaves a queue slot for finish_pair; skipped reads use no quota.
            skip=probe.queue.full()
            owner.depth_stage_probe=None if skip else probe
            owner._finite_stage_skipped+=int(skip)
        result=original_read(owner)
        if probe is not None and probe.summary()['closed'] and not owner._finite_stage_completion_written:
            closed=probe.close(timeout=0.)
            if not closed['writer_alive']:
                owner._finite_stage_completion_written=True
                receipt=dict(trial=args.trial,initial_preset=owner._finite_stage_initial,
                    unselected_backpressure_reads=owner._finite_stage_skipped,summary=closed,error=None)
                try:
                    receipt['final_preset']=preset_snapshot(owner)
                except Exception as exc:receipt['error']=repr(exc)
                try:
                    if not os.path.ismount('/media/zhuyupei/SanStand'):
                        raise RuntimeError('SanStand unavailable')
                    with (out/'probe_completion.json').open('x') as stream:
                        json.dump(receipt,stream,ensure_ascii=False,allow_nan=False)
                        stream.write('\n');stream.flush();os.fsync(stream.fileno())
                except Exception as exc:
                    print('Finite depth completion receipt failed: '+repr(exc),file=sys.stderr,flush=True)
        return result
    manager.read=read_with_finite_probe


def read_verified(path, digest):
    data = Path(path).read_bytes()
    if hashlib.sha256(data).hexdigest() != digest:
        raise RuntimeError('startup source changed: ' + str(path))
    return data


def _module_record(module):
    return dict(file=getattr(module, '__file__', None),
                version=getattr(module, '__version__', None))


def _native_sdk_bindings(sdk):
    """Inspect the loaded extension classes without constructing SDK objects."""
    records = {}
    for name in SDK_BINDINGS:
        value = getattr(sdk, name)
        origin = sys.modules.get(getattr(value, '__module__', ''))
        spec = getattr(origin, '__spec__', None)
        filename = getattr(origin, '__file__', '') or ''
        if (not isinstance(value, type)
                or not isinstance(getattr(spec, 'loader', None),
                                  importlib.machinery.ExtensionFileLoader)
                or not any(filename.endswith(suffix)
                           for suffix in importlib.machinery.EXTENSION_SUFFIXES)
                or getattr(origin, name, None) is not value):
            raise RuntimeError('SDK binding is not an original native class: '+name)
        records[name] = dict(module=value.__module__, file=filename)
    return records


def _install_preflight_guard():
    """Reject Python-level I/O side effects; keep every SDK object unmodified."""
    denied = []
    def guard(event, args):
        reason = None
        if event.startswith(('socket.connect', 'socket.bind', 'subprocess.Popen',
                             'os.system', 'os.fork', 'os.exec')):
            reason = event
        if event == 'open':
            name, mode, flags = args
            if (isinstance(name, str) and name.startswith('/dev/')
                    and name not in ('/dev/null', '/dev/urandom', '/dev/random')):
                reason = 'device open'
            if ((flags or 0) & (os.O_WRONLY | os.O_RDWR | os.O_CREAT |
                               os.O_TRUNC | os.O_APPEND)) and name != '/dev/null':
                reason = 'file write'
        if reason is not None:
            denied.append(dict(event=event, reason=reason))
            raise RuntimeError('import-only preflight blocked '+reason)
    sys.addaudithook(guard)
    return denied


def _prime_preflight_platform():
    # Python 3.8 eagerly runs `uname -p` even for platform.system(), which
    # rosgraph calls during import. Supply the same kernel fields by syscall;
    # leave processor unknown instead of allowing any child process.
    import platform
    primed = getattr(platform, '_uname_cache', None) is None
    if primed:
        values = tuple(os.uname())
        if 'processor' in platform.uname_result._fields:
            values += ('',)
        platform._uname_cache = platform.uname_result(*values)
    return dict(source='os.uname' if primed else 'existing_platform_cache',
                cache_primed=primed, processor_queried=False)


def _preflight_report(plan, args, probe_instances, denied, platform_cache=None):
    # The generated catkin relay propagates this non-main name to the actual
    # source. It imports the same camera module without invoking main().
    namespace = runpy.run_path(plan['entry'], run_name='_depth_stage_import_preflight')
    camera = namespace.get('context', namespace)
    modules = {name: importlib.import_module(name) for name in
               ('rospy', 'cv_bridge', 'numpy', 'cv2', 'pyrealsense2',
                'alicia_flexible_grasp.vision.realsense_manager')}
    manager = modules['alicia_flexible_grasp.vision.realsense_manager']
    if camera.get('RealSenseManager') is not manager.RealSenseManager:
        raise RuntimeError('camera entry imported a different manager class')
    if camera.get('CvBridge') is not modules['cv_bridge'].CvBridge:
        raise RuntimeError('camera entry did not import the active CvBridge')
    if modules['rospy'].core.is_initialized() or probe_instances:
        raise RuntimeError('preflight unexpectedly initialized ROS or a probe')
    bindings = _native_sdk_bindings(modules['pyrealsense2'])
    if denied:
        raise RuntimeError('an import attempted a blocked side effect')
    return dict(schema=1, ok=True, preflight_only=True,
        recorded_ns=time.time_ns(), pid=os.getpid(),
        phase='restored' if args.production else getattr(args,'trial','probe'),
        plan_sha256=args.sha256, interpreter=sys.executable,
        cwd=os.getcwd(), sys_path=list(sys.path),
        exact_environment_restored=True,
        modules={name:_module_record(module) for name,module in modules.items()},
        sdk_bindings=bindings, SDK_objects_replaced=False,
        platform_cache=platform_cache,
        ROS_initialized=False, probe_created=False, denied=denied,
        scope='Imports and native binding identity only; no SDK device/context/pipeline constructed')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--plan', required=True)
    parser.add_argument('--sha256', required=True)
    parser.add_argument('--restored', action='store_true')
    parser.add_argument('--production', action='store_true')
    parser.add_argument('--preflight-only', action='store_true')
    parser.add_argument('--trial', default='probe')
    args = parser.parse_args()
    plan = json.loads(read_verified(args.plan, args.sha256))
    for path, digest in plan['sources'].items():
        read_verified(path, digest)
    phase = 'restored' if args.production else args.trial
    if not args.production:diagnostic_trial(plan,phase)
    env = dict(plan['environment'])
    out = Path(plan['output']) / phase
    env['ROS_LOG_DIR'] = str(out / 'ros')
    env['PYTHONDONTWRITEBYTECODE'] = '1'
    if not args.restored:
        os.chdir(plan['cwd'])
        os.execve(plan['interpreter'], [plan['interpreter'], '-B', __file__,
            '--plan', args.plan, '--sha256', args.sha256, '--restored',
            *(['--production'] if args.production else []),
            '--trial',args.trial,
            *(['--preflight-only'] if args.preflight_only else [])], env)
    if dict(os.environ) != env or os.getcwd() != plan['cwd']:
        raise RuntimeError('camera environment/cwd restoration differs')
    sys.path[0] = str(Path(plan['entry']).parent)
    platform_cache = _prime_preflight_platform() if args.preflight_only else None
    denied = _install_preflight_guard() if args.preflight_only else None
    probe_instances = []
    if not args.production:
        import alicia_flexible_grasp.vision
        def preload(name, path):
            source = read_verified(path, plan['sources'][path])
            module = types.ModuleType(name)
            module.__file__, module.__package__ = path, name.rpartition('.')[0]
            sys.modules[name] = module
            exec(compile(source, path, 'exec'), module.__dict__)
            return module
        manager = preload('alicia_flexible_grasp.vision.realsense_manager', plan['manager'])
        probe_module = preload('alicia_flexible_grasp.vision.depth_stage_probe', plan['probe_module'])
        install_instrumentation(manager,probe_module,plan,args,out,probe_instances)
    if args.preflight_only:
        try:
            report = _preflight_report(plan, args, probe_instances, denied, platform_cache)
        except Exception as exc:
            report = dict(schema=1, ok=False, preflight_only=True,
                phase=phase, plan_sha256=args.sha256,
                error=type(exc).__name__+': '+str(exc), denied=denied)
        print(json.dumps(report, ensure_ascii=False, sort_keys=True), flush=True)
        return 0 if report['ok'] else 1
    receipt = dict(recorded_ns=time.time_ns(), pid=os.getpid(), phase=phase,
        plan_sha256=args.sha256, sources=plan['sources'], exact_environment_restored=True,
        SDK_objects_replaced=False, memory_attestation=False,
        preset_changed=(not args.production and diagnostic_trial(plan,args.trial)['preset'] is not None))
    (out / 'source_start.json').write_text(json.dumps(receipt, ensure_ascii=False, indent=2)+'\n')
    tail = [a for a in plan['ros_args'] if not a.startswith('__log:=')]
    tail.append('__log:='+str(out/'camera.ros.log'))
    sys.argv = [plan['entry'], *tail]
    try:
        runpy.run_path(plan['entry'], run_name='__main__')
    finally:
        for probe in probe_instances:
            probe.close(timeout=2.)


if __name__ == '__main__':
    sys.exit(main())
