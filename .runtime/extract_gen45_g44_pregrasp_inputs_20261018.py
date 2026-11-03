"""Preserve exact closed G44 plan and bounded non-image joint-source windows."""
import base64
import hashlib
import io
import json
from pathlib import Path
import socket
import sys
W=Path(__file__).resolve().parents[1]
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
V=E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
G=E/'grasp_records/20261018T062015.472638Z_df36d0775e4d4fb4a8b3baa949267488'
M=V/'gen44_actual_handoff_monitor_r1'
O=V/'gen45_g44_input_preparation_r2'
F=W/'src/alicia_flexible_grasp_supervisor/tests/fixtures/gen45_g44_pregrasp.json'
sys.path[:0]=[str(W/'src/alicia_flexible_grasp_supervisor/src'),str(W/'devel/lib/python3/dist-packages')]
attempts=[]
class OfflineSocket(socket.socket):
    def __new__(cls,*a,**k):
        attempts.append('socket');raise RuntimeError('closed nonimage bag only')
socket.socket=OfflineSocket
import rosbag
from alicia_flexible_grasp.robot.stationary_following import stationary_following_error,SerialUrdfFk,ARM_NAMES
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb')as f:
        for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
    return h.hexdigest()
def wire(msg):
    s=io.BytesIO();msg.serialize(s);b=s.getvalue()
    return dict(message_type=msg._type,md5sum=msg._md5sum,wire_sha256=hashlib.sha256(b).hexdigest(),wire_base64=base64.b64encode(b).decode())
def save(p,d):
    p.write_text(json.dumps(d,ensure_ascii=False,indent=2,allow_nan=False)+'\n');assert json.loads(p.read_text())==d
def main():
    assert Path('/media/zhuyupei/SanStand').is_mount() and not O.exists() and not F.exists()
    m=json.loads((G/'manifest.json').read_text());assert m['state']=='complete' and m['recorder_closed'] and m['result']=='failure'
    b=next(x for x in m['files']if x['path']=='keyframes.bag');assert sha(G/'keyframes.bag')==b['sha256']
    O.mkdir();before=json.loads((M/'pregrasp_proposal.json').read_text());after=json.loads((M/'pregrasp_measured_result.json').read_text())
    topics=['/grasp_6d/plan_enriched','/grasp_6d/preview_plan_enriched','/alicia_d/sdk_command','/alicia_d/accepted_joint_states']
    rows={'sdk':[],'accepted':[]};objects={'sdk':[],'accepted':[]};plans=[]
    with rosbag.Bag(str(G/'keyframes.bag'),'r')as bag:
        recorded_topics=sorted(bag.get_type_and_topic_info().topics)
        for topic,msg,t in bag.read_messages(topics=topics):
            if topic.startswith('/grasp_6d/'):
                if msg.plan_id==before['plan_id']:plans.append(dict(topic=topic,received_ns=t.to_nsec(),**wire(msg)))
                continue
            stamp=msg.header.stamp.to_nsec()
            if not 1792304679400000000<=stamp<=1792304685300000000:continue
            key='sdk'if topic.endswith('sdk_command')else'accepted'
            rows[key].append(dict(topic=topic,received_ns=t.to_nsec(),source_stamp_ns=stamp,**wire(msg)))
            objects[key].append(msg)
    assert plans and len({x['wire_sha256']for x in plans})==1
    modelrel='deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf'
    model=W/modelrel;assert sha(model)==before['model_sha256'];fk=SerialUrdfFk(model.read_text(),ARM_NAMES)
    def snapshot(sdkstamp,acceptedstamp):
        a=[q for q in objects['sdk']if q.header.stamp.to_nsec()<=sdkstamp]
        c=[q for q in objects['accepted']if q.header.stamp.to_nsec()<=acceptedstamp]
        now=max(sdkstamp,acceptedstamp)*1e-9+.005
        return stationary_following_error(fk,a,c,now_sec=now,epoch_ns=before['epoch_ns'])
    first=snapshot(before['sdk_stamp_ns'],before['accepted_stamp_ns'])
    assert first['sdk_positions_rad']==before['command_start_positions_rad']
    assert first['accepted_positions_rad']==before['measured_start_positions_rad']
    sdkfresh=[q for q in objects['sdk']if q.header.stamp.to_nsec()>before['sdk_stamp_ns'] and list(q.position)==first['sdk_positions_rad']]
    accfresh=[q for q in objects['accepted']if q.header.stamp.to_nsec()>before['accepted_stamp_ns'] and list(q.position)[:6]==first['accepted_positions_rad']]
    assert sdkfresh and accfresh
    second=snapshot(sdkfresh[0].header.stamp.to_nsec(),accfresh[0].header.stamp.to_nsec())
    assert second['sdk_positions_rad']==first['sdk_positions_rad'] and second['accepted_positions_rad']==first['accepted_positions_rad']
    final=snapshot(after['sdk_stamp_ns'],after['accepted_stamp_ns'])
    cfgpath=G/'perception_execution_configuration.json';cfg=json.loads(cfgpath.read_text())['parameters']['/grasp']
    fixture=dict(kind='exact_closed_G44_plan_and_recorded_pregrasp_joint_windows',plan_id=before['plan_id'],plan_wire=plans[0],
        original_plan_deliveries=plans,source_record=str(G),record_manifest_sha256=sha(G/'manifest.json'),bag_sha256=b['sha256'],
        robot_model_path=modelrel,model_sha256=sha(model),config=cfg,
        original_proposal=before,original_measured_result=after,
        original_joint_source_wires=rows,recomputed_original_snapshots=[first,second,final],
        replay_clock_note='stamp_sec uses source-end+5ms as explicit replay capture time; all source stamps, wires, positions and epochs remain original. Original runtime capture now was not separately logged.',
        original_MoveIt_trajectory_available=False,future_response_or_path_provider='must be explicitly simulated; no future physical-success claim',
        recorded_topics=recorded_topics,read_topics=topics,
        original_inputs={str(p):sha(p)for p in (M/'result.json',M/'pregrasp_proposal.json',M/'pregrasp_measured_result.json',cfgpath,model)})
    save(F,fixture);(O/'fixture.json').write_bytes(F.read_bytes());(O/'helper_source.py').write_bytes(Path(__file__).read_bytes())
    d=dict(passed=True,fixture=str(F),fixture_sha256=sha(F),fixture_bytes=F.stat().st_size,
        bag_source_identity_verified=True,plan_id=before['plan_id'],recorded_sdk_messages=len(rows['sdk']),recorded_accepted_messages=len(rows['accepted']),
        exact_before_and_next_fresh_snapshot_sources=True,external_boundary_attempts=attempts,ROS_initialized=False,
        hardware_contacted=False,temperature_read=False,production_or_existing_tests_modified=False)
    assert not attempts;save(O/'result.json',d);print(json.dumps(d,ensure_ascii=False))
if __name__=='__main__':main()
