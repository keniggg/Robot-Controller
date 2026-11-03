#!/usr/bin/env python3
"""Lossless recorded GEN3 callback inputs, with explicit bag-time replay order.

This file imports rosbag/rospy types, but never initializes or contacts ROS.
read_fixture_events() is an offline iterator for downstream protocol tests.
"""
import argparse
import base64
from collections import Counter,defaultdict,deque
import hashlib
import io
import json
from pathlib import Path
import sys
import rosbag

WORK=Path(__file__).resolve().parents[1]
EXTERNAL=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
SOURCES=dict(acquired=1791594097223730325,reference=1791594201510251998,
             gap=1791594201710199594,recovery=1791594201808995485)
TRIPLETS=('/perception/unknown/detector_status','/perception/object','/perception/object_mask')
IMAGES=('/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw')
WHOLE=('/grasp_6d/object_geometry','/grasp_6d/plan_enriched','/grasp_6d/preview_plan_enriched',
       '/grasp/state','/gui/joint_direct_mode','/alicia_d/control_reference_epoch')
MEASURED=('/alicia_d/accepted_joint_states',)


def sha_bytes(data):return hashlib.sha256(data).hexdigest()
def sha_file(path):
    h=hashlib.sha256()
    with path.open('rb') as handle:
        for chunk in iter(lambda:handle.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()
def encoded(data):return (json.dumps(data,ensure_ascii=False,sort_keys=True,indent=2,allow_nan=False)+'\n').encode()
def serialized(msg):
    stream=io.BytesIO();msg.serialize(stream);return stream.getvalue()
def raw_source(topic,msg):
    if topic.endswith('detector_status'):return json.loads(msg.data).get('source_stamp_ns')
    header=getattr(msg,'header',None)
    if header is not None:return header.stamp.to_nsec()
    if topic=='/alicia_d/control_reference_epoch':return msg.stamp.to_nsec()
    return None

def event_key(row):return (row['topic'],row['bag_received_ns'],row['serialized_sha256'])

def read_fixture_events(folder):
    """Yield (original-order metadata, exact decoded message), never publish.

    Metadata order breaks equal bag timestamps only as an offline deterministic
    enumeration convention. It makes no claim about real subscriber callbacks.
    """
    folder=Path(folder);meta_raw=(folder/'fixture.json').read_bytes();meta=json.loads(meta_raw)
    checks=json.loads((folder/'checksums.json').read_text())
    if sha_bytes(meta_raw)!=checks['fixture.json'] or sha_file(folder/'messages.bag')!=checks['messages.bag']:
        raise ValueError('fixture_checksum_mismatch')
    cache=defaultdict(deque)
    with rosbag.Bag(str(folder/'messages.bag')) as bag:
        for topic,raw,received in bag.read_messages(raw=True):
            cache[(topic,received.to_nsec(),sha_bytes(raw[1]))].append(raw)
    for event in meta['events']:
        key=event_key(event)
        if not cache[key]:raise ValueError('original_event_missing')
        raw=cache[key].popleft();msg=raw[4]();msg.deserialize(raw[1])
        if serialized(msg)!=raw[1]:raise ValueError('decoded_message_not_lossless')
        yield event,msg
    if any(cache.values()):raise ValueError('unindexed_fixture_message')


def build(record,output):
    if not EXTERNAL.parent.parent.is_mount():raise RuntimeError('SanStand missing; no fallback')
    if EXTERNAL.resolve() not in output.resolve().parents:raise ValueError('external output required')
    manifest_path=record/'manifest.json';manifest=json.loads(manifest_path.read_text())
    if not(manifest['state']=='complete' and manifest['recorder_closed'] is True):raise ValueError('sealed_record_required')
    bag_path=record/'keyframes.bag';expected=next(row['sha256'] for row in manifest['files'] if row['path']=='keyframes.bag')
    if sha_file(bag_path)!=expected:raise ValueError('original_bag_checksum_mismatch')
    originals=[];max_received=0;min_received=None;valid_plans=[]
    with rosbag.Bag(str(bag_path)) as bag:
        for query_index,entry in enumerate(bag.read_messages(topics=list(TRIPLETS+IMAGES+WHOLE+MEASURED),raw=True,return_connection_header=True)):
            topic,raw,received,connection=entry;msg=raw[4]();msg.deserialize(raw[1]);source=raw_source(topic,msg)
            selected=(topic in TRIPLETS+IMAGES and source in SOURCES.values()) or topic in WHOLE+MEASURED
            if not selected:continue
            if topic in TRIPLETS and source in SOURCES.values():
                max_received=max(max_received,received.to_nsec())
                min_received=received.to_nsec() if min_received is None else min(min_received,received.to_nsec())
            if topic.endswith('plan_enriched') and msg.valid:valid_plans.append((received.to_nsec(),msg.plan_id))
            originals.append((query_index,topic,raw,received,connection,msg,source))
    assert max_received and min_received
    # Retain measured continuity and all recorded plan/geometry transitions
    # through receipt of the selected recovered triplet, not later experiments.
    originals=[r for r in originals if r[3].to_nsec()<=max_received]
    output.mkdir(parents=True,exist_ok=False)
    source_copies={}
    for filename in ('manifest.json','alignment_receipt.json','authorization.json','result_summary.json'):
        data=(record/filename).read_bytes();target=output/('original_'+filename);target.write_bytes(data);source_copies[target.name]=sha_bytes(data)
    originals_by_source={label:[] for label in SOURCES};events=[];connections={};outbag=output/'messages.bag'
    with rosbag.Bag(str(outbag),'w',compression='bz2') as bag:
        for ordinal,(query_index,topic,raw,received,connection,msg,source) in enumerate(originals):
            # Raw bytes and recorded receive timestamps are copied unchanged.
            bag.write(topic,raw,received,raw=True,connection_header=connection)
            connection_json={key:({'type':'bytes','base64':base64.b64encode(value).decode('ascii')} if isinstance(value,bytes) else {'type':'str','value':value}) for key,value in connection.items()}
            connection_bytes=encoded(connection_json);connection_id=sha_bytes(connection_bytes);connections[connection_id]=connection_json
            role='recorded_output_only' if topic in ('/grasp_6d/object_geometry','/grasp_6d/plan_enriched','/grasp_6d/preview_plan_enriched','/grasp/state') else 'recorded_input'
            if topic in IMAGES:role='exact_source_pixels'
            event=dict(ordinal=ordinal,original_selected_topic_query_index=query_index,original_bag_position=list(raw[3]),
                topic=topic,message_type=raw[0],message_md5=raw[2],serialized_size_bytes=len(raw[1]),serialized_sha256=sha_bytes(raw[1]),
                bag_received_ns=received.to_nsec(),source_stamp_ns=source,connection_header_sha256=connection_id,role=role)
            if topic.endswith('detector_status'):
                parsed=json.loads(msg.data);event['detector_summary']={k:parsed.get(k) for k in ('generation','selection_stamp_ns','state','detail','target_lost','anchor_source_stamp_ns')}
            elif topic.endswith('plan_enriched'):
                event.update(valid=msg.valid,plan_id=msg.plan_id,target_track_id=msg.target_track_id)
            elif topic.endswith('object_geometry'):
                event.update(valid=msg.valid,target_track_id=msg.target_track_id,failure_reason=msg.failure_reason)
            elif topic=='/perception/object':event['detected']=msg.detected
            elif topic=='/perception/object_mask':
                import numpy as np
                event['nonzero_bytes']=int(np.count_nonzero(np.frombuffer(msg.data,dtype=np.uint8)))
                event['encoding']=msg.encoding;event['shape']=[msg.height,msg.width]
            elif topic=='/grasp/state':event.update(state=msg.state,active=msg.active,message=msg.message)
            elif topic=='/alicia_d/control_reference_epoch':event['epoch_ns']=msg.stamp.to_nsec()
            if source in SOURCES.values():
                label=next(k for k,v in SOURCES.items() if v==source);event['source_role']=label;originals_by_source[label].append(ordinal)
            events.append(event)
    source_checks={}
    for label,ns in SOURCES.items():
        bytopic=Counter(events[i]['topic'] for i in originals_by_source[label])
        if any(bytopic[t]!=1 for t in TRIPLETS+IMAGES):raise ValueError('source_triplet_or_rgbd_not_unique:'+label)
        source_checks[label]=dict(source_stamp_ns=ns,event_ordinals=originals_by_source[label],topic_counts=dict(bytopic))
    ties=defaultdict(list)
    for event in events:ties[event['bag_received_ns']].append(event['ordinal'])
    bag_ties=[dict(bag_received_ns=ns,ordinals=indices) for ns,indices in ties.items() if len(indices)>1]
    gap_geometry=[r for r in events if r['topic']=='/grasp_6d/object_geometry' and r['source_stamp_ns']==SOURCES['gap']]
    recovery_geometry=[r for r in events if r['topic']=='/grasp_6d/object_geometry' and r['source_stamp_ns']==SOURCES['recovery']]
    original_failure=next(r for r in events if r['topic']=='/grasp/state' and r['state']=='FAILED' and r['active'])
    recovery_receipts=[events[i]['bag_received_ns'] for i in originals_by_source['recovery'] if events[i]['topic'] in TRIPLETS]
    metadata=dict(schema_version=1,scope='lossless GEN3 initial acquired + reference/gap/recovery consumer triplets and original context',
        source_record=str(record),source_bag=str(bag_path),source_bag_sha256=expected,source_manifest_sha256=sha_file(manifest_path),
        builder=str(Path(__file__).resolve()),builder_sha256=sha_file(Path(__file__)),
        old_replay_source=str(WORK/'.runtime/replay_gen3_initial_pending_20261009.py'),old_replay_sha256=sha_file(WORK/'.runtime/replay_gen3_initial_pending_20261009.py'),
        source_stamps_ns=SOURCES,selection=dict(mode='unknown',strategy='observe_return',generation=3,stamp_ns=1791594095215439558),
        stop_recorded_received_ns=max_received,source_copies=source_copies,connections=connections,source_checks=source_checks,
        topic_counts=dict(Counter(r['topic'] for r in events)),events=events,equal_bag_timestamp_groups=bag_ties,
        key_facts=dict(valid_recorded_plans=valid_plans,gap_geometry_ordinals=[r['ordinal'] for r in gap_geometry],
            recorded_gap_geometry_reason=[r['failure_reason'] for r in gap_geometry],
            original_task_failure_received_ns=original_failure['bag_received_ns'],recovery_triplet_received_ns=recovery_receipts,
            original_recovery_received_after_original_task_failure=all(ns>original_failure['bag_received_ns'] for ns in recovery_receipts),
            recovery_geometry_present=bool(recovery_geometry),original_remote_target_track_advanced_on_gap=True),
        replay_usage=dict(iterator='Import builder module and call read_fixture_events(output_directory). It yields (event_metadata, exact_decoded_message) in recorded enumeration order without ROS I/O.',
            inputs='Dispatch original /perception/object, object_mask and detector_status plus measured SDK accepted joint states, control epoch and GUI ownership when the test node supports those callbacks.',
            outputs='plan/preview/geometry/state are ORIGINAL outputs from the failing implementation: expected baseline evidence or explicit task-bound context, not a demand that a corrected remote produce the old TARGET_LOST event.',
            source_pixels='The four exact RGB-D pairs are retained unchanged for producer/RGB identity checks; no image synthesized or restamped.'),
        limitations=[
            'Bag receive order is recorder observation order, not a trace of remote/task callback execution, lock acquisition, scheduling or publication calls.',
            'Object/mask pairs share identical recorded timestamps. Their metadata ordinal is a deterministic iteration convention, not proof of consumer ordering; test both delivery orders.',
            'Only four selected perception sources are retained. Intervening perception statuses/objects/masks are omitted, while historical remote outputs are retained. Do not infer that replaying only these four inputs recreates g1-t11 without separately supplying recorded state or replaying the omitted history.',
            'The selected recovery triplet arrives at the recorder after the original task failure. A new implementation can wait, but this fixture does not show that the old task consumed recovery before it failed.',
            'No actual ObjectGeometry with the selected recovery source was recorded; do not manufacture one and label it recorded evidence.',
            'A repacked rosbag uses one connection header per topic. Every original per-message connection header is preserved separately by digest in this metadata.',
            'Original published plan is based on an older planning snapshot; its source time is not replaced by the final reference/gap/recovery time.',
            'This is an input fixture, not proof of new protocol correctness, real hardware behavior or physical grasp success.'
        ],production_changes=False,robot_commands_sent=False,ros_master_io=False)
    (output/'fixture.json').write_bytes(encoded(metadata))
    checks=dict(source_copies);checks['messages.bag']=sha_file(outbag);checks['fixture.json']=sha_file(output/'fixture.json')
    (output/'checksums.json').write_bytes(encoded(checks))
    verified=list(read_fixture_events(output))
    assert len(verified)==len(events)
    summary=dict(output=str(output),fixture_sha256=checks['fixture.json'],bag_sha256=checks['messages.bag'],bag_size_bytes=outbag.stat().st_size,
        lossless_message_roundtrips=len(verified),topic_counts=metadata['topic_counts'],source_checks=source_checks,key_facts=metadata['key_facts'])
    (output/'verification.json').write_bytes(encoded(summary));print(json.dumps(summary,ensure_ascii=False))

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--record',type=Path);parser.add_argument('--output',type=Path,required=True);parser.add_argument('--verify-only',action='store_true');a=parser.parse_args()
    if a.verify_only:
        rows=list(read_fixture_events(a.output));print(json.dumps(dict(verified_messages=len(rows),fixture=str(a.output)),ensure_ascii=False))
    else:
        if a.record is None:parser.error('--record is required without --verify-only')
        build(a.record,a.output)
