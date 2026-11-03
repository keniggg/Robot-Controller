#!/usr/bin/env python3
"""Finite camera-only A/B/A diagnosis while the ROS stack is stopped.

Uses the reviewed manager with detached stage taps. No ROS import, robot
command, filled depth or synthetic encoder evidence. Stationary filtering has
no encoder provider in all three trials and is explicitly unqualified.
"""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import signal
import sys
import time
import types

W = Path(__file__).resolve().parents[2]
STORE = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
VISION = 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision'
CANDIDATE = W/'.runtime/depth_stage_probe_candidate'/VISION
EXPECTED = {'realsense_manager.py':'4e8b843540b62c0f76c2ac6dbcaf88f075811228877bb4b1e749ed5b52b014d3',
            'depth_stage_probe.py':'24de1f28cb3366eb87c02ec669c3d751114c9a3b167d36f9378bbbee6e633ae8'}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def require(value, reason):
    if not value:
        raise RuntimeError(reason)


def save(path, value):
    require(os.path.ismount(STORE.parents[1]), 'SanStand unavailable')
    require(STORE.resolve() in Path(path).resolve().parents, 'external output required')
    with Path(path).open('x') as stream:
        json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False)
        stream.write('\n');stream.flush();os.fsync(stream.fileno())


def camera_owners():
    devices={str(p) for p in Path('/dev').glob('video*')}
    for node in Path('/sys/bus/usb/devices').glob('*'):
        try:
            if (node/'idVendor').read_text().strip() == '8086':
                devices.add('/dev/bus/usb/%03d/%03d' % (int((node/'busnum').read_text()),int((node/'devnum').read_text())))
        except OSError:
            pass
    found=[]
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name)==os.getpid():
            continue
        try:
            used={os.readlink(fd) for fd in (proc/'fd').iterdir()} & devices
            if used: found.append(dict(pid=int(proc.name),devices=sorted(used)))
        except OSError:
            pass
    return found


def load(name,path):
    raw=path.read_bytes()
    module=types.ModuleType(name);module.__file__=str(path);module.__package__=name.rpartition('.')[0]
    sys.modules[name]=module;exec(compile(raw,str(path),'exec'),module.__dict__)
    return module


def sensor_snapshot(manager):
    preset=manager.mode_depth_preset
    result=dict(evidence=preset.evidence(),advanced_settings=json.loads(preset.advanced.serialize_json()),sensor_options={})
    for name in ('visual_preset','enable_auto_exposure','exposure','gain','power_line_frequency'):
        option=getattr(manager.rs.option,name)
        result['sensor_options'][name]=(preset.sensor.get_option(option) if preset.sensor.supports(option) else None)
    return result


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--reference-plan',type=Path,required=True)
    parser.add_argument('--reference-sha256',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    require(sha(args.reference_plan)==args.reference_sha256,'reference plan differs')
    reference=json.loads(args.reference_plan.read_text())
    cfg=reference['camera_configuration']
    require(os.path.ismount(STORE.parents[1]) and STORE.resolve() in args.output.resolve().parents,'mounted external output required')
    require(not camera_owners(),'another process owns a camera device')
    for name,digest in EXPECTED.items():require(sha(CANDIDATE/name)==digest,'reviewed probe source differs')
    args.output.mkdir()
    import alicia_flexible_grasp.vision
    manager_module=load('alicia_flexible_grasp.vision.realsense_manager',CANDIDATE/'realsense_manager.py')
    probe_module=load('alicia_flexible_grasp.vision.depth_stage_probe',CANDIDATE/'depth_stage_probe.py')
    import numpy as np
    import cv2
    import pyrealsense2 as rs
    cv2.setNumThreads(1)
    require(np.__version__=='1.17.4' and cv2.__version__=='4.2.0' and rs.__version__=='2.56.5','owner numerical dependencies differ')
    source_paths=[Path(__file__),CANDIDATE/'realsense_manager.py',CANDIDATE/'depth_stage_probe.py',*list((W/VISION).glob('*.py'))]
    sources={str(p):sha(p) for p in source_paths}
    original_metadata=manager_module.RealSenseManager._raw_frame_diagnostics
    def metadata_with_exposure(owner,frames):
        data=original_metadata(owner,frames)
        for stream in ('color','depth'):
            frame=getattr(frames,'get_'+stream+'_frame')()
            for name in ('actual_exposure','gain_level','auto_exposure'):
                try:
                    field=getattr(owner.rs.frame_metadata_value,name)
                    if frame.supports_frame_metadata(field):data[stream]['metadata_raw'][name]=int(frame.get_frame_metadata(field))
                    else:data[stream]['unavailable'][name]='not_supported'
                except Exception as exc:data[stream]['unavailable'][name]=type(exc).__name__
        return data
    manager_module.RealSenseManager._raw_frame_diagnostics=metadata_with_exposure
    result=dict(kind='camera_only_native_preset_ABA',reference_plan=str(args.reference_plan),reference_sha256=args.reference_sha256,
        recorded_ns=time.time_ns(),sources=sources,encoder_provider_present=False,
        regional_stationary_qualified=False,task_mode='unknown',motion_commands_sent=False,ROS_contacted=False,
        temperature_queried=False,physical_grasp_claimed=False,trials=[],completed=False)
    save(args.output/'plan.json',result)
    manager=probe=None
    try:
        for label,preset in [('A1','high_accuracy'),('B','saved_baseline'),('A2','high_accuracy')]:
            trial=args.output/label;trial.mkdir()
            preset_cfg=copy.deepcopy(cfg['mode_depth_preset']);preset_cfg['unknown_preset']=preset
            filters=copy.deepcopy(cfg['depth_filter'])
            filters.update(depth_min_m=reference['perception_depth_range']['depth_min_m'],depth_max_m=reference['perception_depth_range']['depth_max_m'])
            manager=manager_module.RealSenseManager(width=cfg['stream_width'],height=cfg['stream_height'],fps=cfg['fps'],
                align_depth_to_color=True,simulate=False,depth_filter_cfg=filters,
                color_projection_cfg=cfg.get('color_projection_correction',{}),
                pair_rectification_cfg=cfg.get('pair_rectification',{}),mode_depth_preset_cfg=preset_cfg)
            started=time.monotonic();manager.start();manager.set_depth_preset_mode('unknown')
            require(manager.stationary_depth_provider is None,'unexpected encoder provider')
            initial=sensor_snapshot(manager)
            save(trial/'initial_configuration.json',initial)
            probe=probe_module.DepthStageProbe(trial/'capture',frame_limit=10,seconds=30.,warmup_sec=5.,
                owner_metadata=dict(pid=os.getpid(),sources=sources,preset_snapshot=initial,
                    standalone_diagnostic=True,encoder_provider_present=False,trial=label))
            manager.depth_stage_probe=probe
            sensor_samples=[]
            backpressure_skipped=0
            while time.monotonic()-started<40.:
                # This sole producer cannot fill the queue between admission
                # and finish_pair. Keep acquisition running when the disk
                # writer is behind; an unselected read spends no sample slot.
                skip=probe.queue.full()
                manager.depth_stage_probe=None if skip else probe
                backpressure_skipped+=int(skip)
                manager.read()
                diagnostic=manager.last_read_diagnostics
                if diagnostic and diagnostic.get('result')=='complete':
                    sensor_samples.append(dict(source=diagnostic.get('raw_frames'),snapshot=sensor_snapshot(manager)))
                if probe.summary()['closed']:break
            summary=probe.close(timeout=3.)
            require(not summary['writer_alive'],'probe writer did not close')
            save(trial/'sensor_samples.json',sensor_samples)
            final=sensor_snapshot(manager);save(trial/'final_configuration.json',final)
            manager.stop();manager=None
            require(summary['error'] is None and summary['counts']['written']==10
                    and summary['counts']['completed']==summary['counts']['selected']==10
                    and summary['counts']['interrupted_pairs']==0
                    and summary['counts']['queue_dropped']==0,'diagnostic sample count incomplete')
            result['trials'].append(dict(label=label,preset=preset,summary=summary,wall_sec=time.monotonic()-started,
                unselected_backpressure_reads=backpressure_skipped,
                geometry_digest=final['evidence'].get('baseline_sha256'),camera_stopped=True))
            print(json.dumps(dict(trial=label,written=summary['counts']['written'],wall_sec=result['trials'][-1]['wall_sec'])),flush=True)
        result['completed']=True
    except BaseException as exc:
        result['error']=type(exc).__name__+': '+str(exc)
        raise
    finally:
        if probe is not None:probe.close(timeout=3.)
        if manager is not None:manager.stop()
        result['camera_owners_after']=camera_owners()
        result['finished_ns']=time.time_ns()
        save(args.output/'result.json',result)


if __name__=='__main__':
    def interrupted(_signal,_frame):raise KeyboardInterrupt('diagnostic interrupted')
    signal.signal(signal.SIGTERM,interrupted)
    main()
