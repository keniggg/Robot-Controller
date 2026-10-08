#!/usr/bin/env python3
"""Offline transport comparison, never initialize ROS or a camera."""
import argparse
from collections import deque
import hashlib
import importlib.util
import json
from pathlib import Path
import statistics
import threading
import time
from types import SimpleNamespace
import xmlrpc.client

from alicia_flexible_grasp.vision.realsense_manager import _parameter_pair_history

W = Path(__file__).resolve().parents[1]
SOURCE = W/'src/alicia_flexible_grasp_supervisor/scripts/camera_node.py'


def timed(call, n=100):
    times=[]
    value=None
    for _ in range(n):
        start=time.perf_counter(); value=call()
        times.append((time.perf_counter()-start)*1000.)
    return value,dict(iterations=n,p50_ms=statistics.median(times),
                      p95_ms=sorted(times)[int(.95*(n-1))],max_ms=max(times))


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--input',required=True)
    parser.add_argument('--output',required=True);args=parser.parse_args()
    source=Path(args.input)
    fixture=None
    with source.open() as stream:
        for number,line in enumerate(stream,1):
            row=json.loads(line)
            d=row.get('existing_read_diagnostics',{})
            if d.get('result')=='complete' and d.get('regional_stationary_evidence',{}).get('filtered_pixel_count',0)>0:
                fixture=d;break
    if fixture is None:raise RuntimeError('no complete filtered regional diagnostic')
    spec=importlib.util.spec_from_file_location('offline_camera_diagnostics',SOURCE)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    compact,copy_timing=timed(lambda:module._compact_read_evidence(fixture))
    node=module.CameraNode.__new__(module.CameraNode)
    node._recent_read_lock=threading.Lock()
    node._recent_successful_reads=deque(maxlen=32)
    node._recent_read_generation=1;node._recent_read_copy_errors=0
    node.frame_id='camera_color_optical_frame'
    node.cfg={'runtime_profile':{'geometry_activation_stamp_ns':1791284010000000000}}
    node.cam=SimpleNamespace(runtime_profile={'stationary_temporal':{}},last_read_diagnostics=fixture)
    for i in range(32):node._remember_successful_read(1791284750000000000+i,100.+i/30.)
    envelope,encode_timing=timed(node._recent_successful_read_parameters)
    payload=json.loads(envelope['payload_json'])
    old,old_conversion=timed(lambda:_parameter_pair_history(payload))
    old_xml,old_xml_timing=timed(lambda:xmlrpc.client.dumps((old,)))
    new_xml,new_xml_timing=timed(lambda:xmlrpc.client.dumps((envelope,)))
    _,new_xml_decode=timed(lambda:xmlrpc.client.loads(new_xml))
    _,json_decode=timed(lambda:json.loads(envelope['payload_json']))
    roundtrip=json.loads(xmlrpc.client.loads(new_xml)[0][0]['payload_json'])
    assert roundtrip==payload
    assert payload['entries']==list(node._recent_successful_reads)
    assert payload['entries'][0]['source_stamp_ns']==1791284750000000000
    report=dict(schema=1,scope='offline metadata-only; one real complete regional diagnostic repeated into a synthetic full 32-row history; no live ROS or camera calls and no real-time guarantee',
        input=str(source),input_line=number,input_row_sha256=hashlib.sha256(line.encode()).hexdigest(),
        source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        evidence_bytes=len(json.dumps(compact,separators=(',',':')).encode()),
        json_payload_bytes=len(envelope['payload_json'].encode()),
        old_xmlrpc_bytes=len(old_xml.encode()),new_xmlrpc_bytes=len(new_xml.encode()),
        xmlrpc_size_ratio=len(new_xml.encode())/len(old_xml.encode()),
        metadata_copy=copy_timing,json_envelope_encoding=encode_timing,
        old_recursive_conversion=old_conversion,old_xmlrpc_encoding=old_xml_timing,
        new_xmlrpc_encoding=new_xml_timing,new_xmlrpc_decoding=new_xml_decode,
        json_decoding=json_decode,typed_roundtrip_equal=True,
        full_history_entries=len(payload['entries']))
    out=Path(args.output);out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
