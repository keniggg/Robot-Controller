#!/usr/bin/env python3
"""Read a sealed record once into shared handoff evidence; never connects to ROS."""
import argparse, collections, hashlib, json, os, time
from pathlib import Path
import rosbag
import numpy as np
import cv2

def plain(value):
    if hasattr(value, 'to_nsec'):
        return {'ns': value.to_nsec()}
    if hasattr(value, '__slots__'):
        return {s: plain(getattr(value, s)) for s in value.__slots__}
    if isinstance(value, (tuple, list)):
        return [plain(x) for x in value]
    if isinstance(value, bytes):
        return {'bytes': len(value), 'sha256': hashlib.sha256(value).hexdigest()}
    return value

def save(path, value):
    with path.open('x') as f:
        json.dump(value, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write('\n')

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--record', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--image-times', type=float, nargs='+', required=True)
    a=ap.parse_args()
    mount=Path('/media/zhuyupei/SanStand')
    if not mount.is_mount() or a.output.parent.stat().st_dev != mount.stat().st_dev:
        raise RuntimeError('SanStand unavailable')
    manifest_path=a.record/'manifest.json'
    manifest_bytes=manifest_path.read_bytes()
    manifest=json.loads(manifest_bytes)
    if manifest.get('state')!='complete' or not manifest.get('recorder_closed'):
        raise RuntimeError('record is not sealed')
    bagpath=a.record/'keyframes.bag'
    identity=(bagpath.stat().st_dev,bagpath.stat().st_ino,bagpath.stat().st_size,bagpath.stat().st_mtime_ns)
    declared=next(x for x in manifest['files'] if x['path']=='keyframes.bag')
    if identity[2]!=declared['size_bytes']:
        raise RuntimeError('sealed bag size changed')
    a.output.mkdir(exist_ok=False)
    (a.output/'frames').mkdir()
    started=time.monotonic()
    selected={}
    counts=collections.Counter()
    transitions=collections.defaultdict(list)
    laststate={}
    lastcontroller=None
    excluded={'/alicia_d/sdk_diagnostics','/alicia_d/protection_latched','/alicia_d/run_status',
              '/alicia_d/control_reference','/alicia_d/sdk_command','/joint_commands','/joint_states'}
    image_topics={'/experiment/observation_rgbd/color','/supervisor/camera/color/image_raw'}
    with rosbag.Bag(str(bagpath),'r') as bag, (a.output/'events.jsonl').open('x') as out:
        topics=bag.get_type_and_topic_info().topics
        save(a.output/'topics.json',{k:dict(type=v.msg_type,count=v.message_count) for k,v in topics.items()})
        wanted=[t for t in topics if t not in excluded and 'depth' not in t.lower()]
        for topic,msg,ts,connection in bag.read_messages(topics=wanted,return_connection_header=True):
            ns=ts.to_nsec()
            row=dict(topic=topic,bag_ns=ns,caller=connection.get('callerid'),type=msg._type)
            counts[topic]+=1
            if topic in image_topics:
                for target in a.image_times:
                    key=(topic,target)
                    gap=abs(ns/1e9-target)
                    if key not in selected or gap<selected[key][0]:
                        selected[key]=(gap,msg,ns)
                row['data']={k:plain(getattr(msg,k)) for k in ('header','height','width','encoding','step')}
            elif topic=='/perception/object_mask':
                pixels=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width]
                yy,xx=np.nonzero(pixels)
                row['data']=dict(header=plain(msg.header),encoding=msg.encoding,
                    pixels=int(xx.size),bbox=[int(xx.min()),int(yy.min()),int(xx.max()),int(yy.max())] if xx.size else None)
            elif topic=='/alicia_controller/state':
                if lastcontroller is not None and ns-lastcontroller<250_000_000:
                    continue
                lastcontroller=ns
                row['data']=plain(msg)
            elif topic=='/rosout_agg':
                if not (min(a.image_times)-30)*1e9 <= ns <= (max(a.image_times)+3)*1e9:
                    continue
                if 'temperature' in msg.msg.lower() or '温度' in msg.msg:
                    continue
                row['data']=plain(msg)
            else:
                row['data']=plain(msg)
            if msg._type=='std_msgs/String':
                try: row['parsed']=json.loads(msg.data)
                except (ValueError,TypeError): pass
            if topic in ('/grasp/state','/grasp_6d/status','/perception/unknown/detector_status',
                         '/gui/joint_direct_mode','/alicia_d/actuation_status','/alicia_d/motion_enabled'):
                data=row.get('parsed',row['data'])
                # Ignore transport stamps for a compact status transition index.
                state={k:v for k,v in data.items() if k not in ('header','stamp','stamp_ns')} if isinstance(data,dict) else data
                key=json.dumps(state,sort_keys=True)
                if laststate.get(topic)!=key:
                    transitions[topic].append(row)
                    laststate[topic]=key
            out.write(json.dumps(row,ensure_ascii=False,allow_nan=False,separators=(',',':'))+'\n')
    frames=[]
    for i,((topic,target),(gap,msg,ns)) in enumerate(sorted(selected.items())):
        if msg.encoding not in ('rgb8','bgr8'):
            raise RuntimeError('unsupported source colour encoding: '+msg.encoding)
        arr=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
        if msg.encoding=='rgb8':
            arr=cv2.cvtColor(arr,cv2.COLOR_RGB2BGR)
        path=a.output/'frames'/('%02d_%d.png'%(i,msg.header.stamp.to_nsec()))
        if not cv2.imwrite(str(path),arr):
            raise RuntimeError('image export failed')
        frames.append(dict(path=str(path),topic=topic,target_sec=target,offset_sec=gap,
                           bag_ns=ns,source_ns=msg.header.stamp.to_nsec(),sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    if identity!=(bagpath.stat().st_dev,bagpath.stat().st_ino,bagpath.stat().st_size,bagpath.stat().st_mtime_ns) or manifest_path.read_bytes()!=manifest_bytes:
        raise RuntimeError('source changed while reading')
    save(a.output/'transitions.json',transitions)
    save(a.output/'frames.json',frames)
    result=dict(record=str(a.record),manifest_sha256=hashlib.sha256(manifest_bytes).hexdigest(),
        bag_declared_sha256=declared['sha256'],bag_digest_recomputed=False,bag_identity=identity,
        source_unchanged=True,elapsed_sec=time.monotonic()-started,selected_topic_counts=dict(counts),
        frame_count=len(frames),events_bytes=(a.output/'events.jsonl').stat().st_size,
        ros_connected=False,temperature_topics_excluded=True)
    save(a.output/'result.json',result)
    print(json.dumps(result))
if __name__=='__main__':
    main()

