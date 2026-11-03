"""Sealed G49 feedback timing audit. Reuses the exact offline extraction guard."""
import importlib.util
import hashlib
import io
import json
import math
import statistics
import time
from pathlib import Path

W = Path(__file__).resolve().parents[1]
GUARD = W / '.runtime/extract_gen49_arrival_visibility_20261018.py'
spec = importlib.util.spec_from_file_location('g49_arrival_offline_guard', GUARD)
guard = importlib.util.module_from_spec(spec)
spec.loader.exec_module(guard)
O = guard.E / 'ros_log/full_latest_gen41_restart_20261018T011222Z/verification/gen49_arrival_motion_audit_r1'
guard.O = O
TOPICS = ('/alicia_d/accepted_joint_states','/alicia_d/sdk_diagnostics','/joint_states',
          '/alicia_controller/state','/alicia_controller/follow_joint_trajectory/goal',
          '/alicia_controller/follow_joint_trajectory/result','/alicia_controller/follow_joint_trajectory/status',
          '/alicia_controller/command','/alicia_d/sdk_command', guard.COLOR)
LO, HI = 1792321744500000000, 1792321807000000000
ARRIVAL = 1792321795873908758
SOURCES = (1792321796018874883,1792321796208590507,1792321796341869592,1792321796686522722)
def sha(p): return guard.sha(p)
def save(p,x): guard.save(p,x); return dict(path=str(p),sha256=sha(p))
def basic(m, recv):
    return dict(received_ns=recv,source_stamp_ns=m.header.stamp.to_nsec(),seq=m.header.seq,frame_id=m.header.frame_id)
def source_summary(rows,lo,hi):
    r=[x for x in rows if lo<=x['source_stamp_ns']<=hi]
    gaps=[(b['source_stamp_ns']-a['source_stamp_ns'])/1e9 for a,b in zip(r,r[1:])]
    if not r:return dict(count=0)
    n=len(r[0]['position']); ranges=[max(x['position'][j]for x in r)-min(x['position'][j]for x in r)for j in range(n)]
    vel=[max((abs((b['position'][j]-a['position'][j])/((b['source_stamp_ns']-a['source_stamp_ns'])/1e9)) for a,b in zip(r,r[1:]) if b['source_stamp_ns']>a['source_stamp_ns']),default=0.)for j in range(n)]
    return dict(count=len(r),first_ns=r[0]['source_stamp_ns'],last_ns=r[-1]['source_stamp_ns'],position_peak_to_peak_rad=ranges,unique_positions_per_joint=[len({x['position'][j]for x in r})for j in range(n)],maximum_finite_difference_abs_velocity_rad_s=vel,median_source_interval_sec=statistics.median(gaps)if gaps else None,maximum_source_interval_sec=max(gaps)if gaps else None,unique_header_stamps=len({x['source_stamp_ns']for x in r}),unique_header_sequences=len({x['seq']for x in r}),seq_nonincrements=[dict(before=a['seq'],after=b['seq'])for a,b in zip(r,r[1:])if b['seq']<=a['seq']])
def main():
    assert Path('/media/zhuyupei/SanStand').is_mount() and not O.exists()
    m=json.loads((guard.G/'manifest.json').read_text());assert m['state']=='complete'and m['recorder_closed']is True
    assert sha(guard.G/'manifest.json')=='747a7e1f2240a6e612763e4320a34a428975fb4fe022b278c479f695b8ad7ed0'
    bref=next(x for x in m['files']if x['path']=='keyframes.bag');assert sha(guard.G/'keyframes.bag')==bref['sha256']
    O.mkdir();(O/'tmp').mkdir()
    import os,tempfile
    os.environ['TMPDIR']=str(O/'tmp');tempfile.tempdir=str(O/'tmp')
    rows={x:[]for x in TOPICS}; metadata={}
    with guard.rosbag.Bag(str(guard.G/'keyframes.bag'),'r')as bag:
        info=bag.get_type_and_topic_info().topics
        metadata={t:dict(type=info[t].msg_type,count=info[t].message_count)for t in TOPICS if t in info}
        for topic,msg,received in bag.read_messages(topics=list(TOPICS)):
            recv=received.to_nsec();ns=msg.header.stamp.to_nsec()if hasattr(msg,'header')else recv
            if not LO<=ns<=HI:continue
            r=basic(msg,recv)
            if topic==guard.COLOR:
                if not 1792321794500000000<=ns<=1792321797200000000:continue
                assert msg.encoding in ('bgr8','rgb8')
                arr=guard.np.frombuffer(msg.data,dtype=guard.np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
                bgr=arr if msg.encoding=='bgr8'else arr[:,:,::-1]
                png=O/('original_color_%s.png'%ns)
                assert guard.cv2.imwrite(str(png),bgr)and guard.np.array_equal(guard.cv2.imread(str(png)),bgr)
                r.update(width=msg.width,height=msg.height,encoding=msg.encoding,step=msg.step,
                         data_sha256=hashlib.sha256(msg.data).hexdigest(),png_path=str(png),png_sha256=sha(png),
                         decoded_BGR_sha256=hashlib.sha256(guard.np.ascontiguousarray(bgr).tobytes()).hexdigest(),
                         resampled=False,overlay_added=False)
                rows[topic].append(r)
            elif topic=='/alicia_d/sdk_diagnostics':
                for status in msg.status:
                    fn=next((v.value for v in status.values if v.key=='command_function_hex'),None)
                    if fn!='06 00':continue
                    allowed=('command_function_hex','payload_hex','packet_id','frame_counter','sequence','crc_valid')
                    values={v.key:v.value for v in status.values if v.key in allowed}
                    payload=bytes.fromhex(values.get('payload_hex','')); assert len(payload)>=12
                    words=[payload[j]+256*payload[j+1]for j in range(0,12,2)]
                    raw=dict(r,diagnostic_name=status.name,values=values,joint_words=words,
                             decoded_joint_positions_rad=[(-180+min(4095,w)/4096*360)*math.pi/180 for w in words])
                    rows[topic].append(raw)
            else:
                stream=io.BytesIO();msg.serialize(stream);raw=stream.getvalue()
                r.update(wire_sha256=hashlib.sha256(raw).hexdigest(),payload=guard.payload(msg))
                if topic in ('/alicia_d/accepted_joint_states','/joint_states','/alicia_d/sdk_command'):
                    r.update(name=list(msg.name),position=list(msg.position),velocity=list(msg.velocity))
                rows[topic].append(r)
    for t in ('/alicia_d/accepted_joint_states','/joint_states','/alicia_d/sdk_diagnostics'):
        rows[t].sort(key=lambda x:(x['source_stamp_ns'],x['received_ns']))
    refs={t:save(O/(t.strip('/').replace('/','__')+'.json'),dict(topic=t,rows=r))for t,r in rows.items()}
    accepted=rows['/alicia_d/accepted_joint_states'];raw=rows['/alicia_d/sdk_diagnostics']
    pairs=[]
    for x in accepted:
        prior=[y for y in raw if 0<=x['source_stamp_ns']-y['source_stamp_ns']<20_000_000]
        if prior:
            y=prior[-1];pairs.append(dict(accepted_source_ns=x['source_stamp_ns'],raw_source_ns=y['source_stamp_ns'],host_parse_to_accepted_ns=x['source_stamp_ns']-y['source_stamp_ns'],positions_equal_to_six_raw_words=all(abs(a-b)<1e-12for a,b in zip(x['position'][:6],y['decoded_joint_positions_rad']))))
    brackets=[]
    for ns in SOURCES:
        before=[x for x in accepted if x['source_stamp_ns']<=ns];after=[x for x in accepted if x['source_stamp_ns']>=ns]
        b=before[-1]if before else None;a=after[0]if after else None
        brackets.append(dict(image_source_ns=ns,before=b,after=a,joint_delta_rad=[v-u for u,v in zip(b['position'],a['position'])]if b and a else None))
    windows={name:source_summary(accepted,lo,hi)for name,lo,hi in [
        ('entire_selected_motion_and_arrival',LO,HI),('two_seconds_before_arrival',ARRIVAL-2_000_000_000,ARRIVAL),
        ('two_seconds_after_arrival',ARRIVAL,ARRIVAL+2_000_000_000),
        ('four_seconds_around_arrival',ARRIVAL-2_000_000_000,ARRIVAL+2_000_000_000),
        ('original_settle_receipt_window',1792321794914409423,1792321795714435137),
        ('three_qualified_and_first_failure',SOURCES[0]-200_000_000,SOURCES[-1]+200_000_000)]}
    assert sha(guard.G/'keyframes.bag')==bref['sha256']and not guard.ATTEMPTS
    result=dict(kind='sealed_G49_motion_arrival_feedback_audit',passed=True,record=str(guard.G),manifest_sha256=sha(guard.G/'manifest.json'),bag_sha256=bref['sha256'],bag_before_after_same=True,window_ns=[LO,HI],recorded_topics=metadata,extracted_refs=refs,accepted_count=len(accepted),joint_packet_raw_count=len(raw),raw_accepted_pair_count=len(pairs),raw_accepted_all_paired_positions_equal=all(x['positions_equal_to_six_raw_words']for x in pairs),raw_accepted_pairs=save(O/'raw_accepted_pairs.json',pairs),image_feedback_brackets=save(O/'image_feedback_brackets.json',brackets),accepted_windows=windows,helper_sha256=sha(__file__),guard_helper_sha256=sha(GUARD),external_boundary_attempts=guard.ATTEMPTS,ROS_initialized=False,hardware_contacted=False,temperature_read=False,source_control_or_record_modified=False,limitations=['Source stamps are host packet parser/acceptance times, not synchronized device sensor timestamps.','Only 06 00 joint-response diagnostic payloads are decoded; no other diagnostics or temperature values are extracted.','Heartbeat /joint_states is not counted as independent accepted feedback.','Finite differences of quantized joint positions are not a direct velocity sensor.','Stable encoder feedback cannot exclude elastic end-effector or camera vibration below encoder resolution.'])
    ref=save(O/'result.json',result);print(json.dumps(dict(result=ref,counts={t:len(r)for t,r in rows.items()},windows=windows),ensure_ascii=False))
if __name__=='__main__':main()
