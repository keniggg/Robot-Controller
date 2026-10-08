"""Pure tests; never initialize a ROS node or access a device."""
import importlib.util
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest

S = importlib.util.spec_from_file_location('raw_load', Path(__file__).with_name('compare_raw_rgbd_recording_load_20261007.py'))
m = importlib.util.module_from_spec(S)
S.loader.exec_module(m)


class Stamp:
    def __init__(self, sec, nsec=0): self.ns=sec*10**9+nsec
    def to_nsec(self): return self.ns


def image(ns, data=b'original', frame='camera_link'):
    return SimpleNamespace(header=SimpleNamespace(stamp=Stamp(0,ns),seq=7,frame_id=frame),
                           height=1,width=len(data),encoding='mono8',step=len(data),data=data)


def valid_report():
    return dict(bag_closed=True,errors=[],stop_reason='duration_elapsed',pair_count=2,
                capture_start_monotonic=10.,capture_end_monotonic=55.,
                capture_deadline_monotonic=55.,
                receive=dict(queue_pairs_remaining=0,counts={}))


def test_exact_join_does_not_nearest_match():
    cache=m.ExactPairs(0,45)
    cache.add('color',image(100),1,10)
    cache.add('depth',image(101),1.1,11)
    assert cache.pop() is None
    cache.add('depth',image(100),1.2,12)
    ns,row=cache.pop()
    assert ns==100 and set(row)==set(m.TOPICS)
    assert row['color'][0].data==b'original'
    cache.stop()
    assert cache.snapshot()['counts']['unmatched_messages_at_end']==1
    assert cache.bytes==0


def test_same_pair_duplicate_not_written_twice():
    c=m.ExactPairs(0,45)
    for _ in range(2):
        c.add('color',image(100),1,10);c.add('depth',image(100),1,10)
    assert c.pop()[0]==100 and c.pop() is None
    assert c.snapshot()['counts']['duplicate']==2


def test_repeated_stop_preserves_unmatched_counts_and_ready_pair_drain():
    c=m.ExactPairs(0,45)
    c.add('color',image(100,b'color-only'),1,10)
    c.add('depth',image(101,b'depth-only'),1,10)
    c.add('color',image(200,b'rgb'),1,10)
    c.add('depth',image(200,b'depth'),1,10)
    c.stop()
    first=c.snapshot()
    assert first['counts']['unmatched_sources_at_end']==2
    assert first['counts']['unmatched_messages_at_end']==2
    assert first['queue_pairs_remaining']==1
    assert first['retained_pixel_bytes']==8
    assert not c.pending
    c.stop()
    assert c.snapshot()==first
    assert c.pop()[0]==200
    c.stop()
    final=c.snapshot()
    assert final['counts']==first['counts']
    assert final['queue_pairs_remaining']==0
    assert final['retained_pixel_bytes']==0


def test_window_is_receipt_bounded_and_late_callback_does_not_rearm():
    c=m.ExactPairs(10,55)
    for t in (9.9,55,80):c.add('color',image(100),t,1)
    assert not c.pending
    c.stop();c.add('depth',image(100),20,2)
    assert not c.pending and not c.ready


@pytest.mark.parametrize('ns,data',[(0,b'x'),(-1,b'x'),(1,b'')])
def test_invalid_source_or_empty_data_recorded_not_fabricated(ns,data):
    c=m.ExactPairs(0,45);c.add('color',image(ns,data),1,1)
    assert c.snapshot()['counts']['invalid_source_or_empty_image']==1
    assert c.pop() is None


def test_memory_bound_includes_ready_pairs():
    c=m.ExactPairs(0,45,max_bytes=4)
    c.add('color',image(1,b'ab'),1,1);c.add('depth',image(1,b'cd'),1,1)
    c.add('color',image(2,b'ab'),1,1)
    assert c.bytes==4 and c.snapshot()['counts']['memory_limit']==1
    assert c.pop()[0]==1 and c.bytes==0


def test_pending_source_count_bound_recycles_oldest_unmatched_source():
    c=m.ExactPairs(0,45,max_entries=2)
    for ns in (1,2,3):c.add('color',image(ns),1,1)
    assert list(c.pending)==[2,3]
    assert c.snapshot()['counts']['pending_capacity_evicted_sources']==1
    assert c.snapshot()['counts']['pending_capacity_evicted_messages']==1
    assert c.snapshot()['counts']['pending_capacity_evicted_bytes']==len(b'original')
    assert c.counts['memory_limit']==0
    c.add('depth',image(3),2,2)
    assert c.pop()[0]==3


def test_byte_pressure_preserves_incoming_exact_mate_and_ready_pair():
    c=m.ExactPairs(0,45,max_bytes=10,max_entries=64)
    c.add('color',image(10,b'aa'),1,1)
    c.add('depth',image(10,b'bb'),1,1)  # Complete writer-owned four bytes.
    c.add('color',image(20,b'ccc'),2,2)  # Oldest incomplete is the incoming mate.
    c.add('depth',image(30,b'ddd'),3,3)
    c.add('depth',image(20,b'eee'),4,4)
    assert c.bytes==10 and not c.pending
    assert c.counts['pending_capacity_evicted_bytes']==3
    assert [c.pop()[0],c.pop()[0]]==[10,20]
    assert c.bytes==0


def test_impossible_admit_does_not_drop_ready_or_other_pending_data():
    c=m.ExactPairs(0,45,max_bytes=6)
    c.add('color',image(10,b'ab'),1,1);c.add('depth',image(10,b'cd'),1,1)
    c.add('color',image(20,b'ef'),2,2)
    c.add('depth',image(20,b'gh'),3,3)
    assert c.counts['memory_limit']==1 and c.bytes==6
    assert list(c.pending)==[20] and c.counts['pending_capacity_evicted_sources']==0
    assert c.pop()[0]==10
    c.add('depth',image(20,b'gh'),4,4)
    assert c.pop()[0]==20 and c.bytes==0


def test_oversize_message_does_not_evict_useful_incomplete_data():
    c=m.ExactPairs(0,45,max_bytes=4)
    c.add('color',image(10,b'ab'),1,1)
    c.add('depth',image(20,b'oversize'),2,2)
    assert list(c.pending)==[10] and c.bytes==2
    assert c.counts['memory_limit']==1 and c.counts['pending_capacity_evicted_sources']==0


def test_capacity_recovery_keeps_exact_stamps_and_loss_after_repeated_stop():
    c=m.ExactPairs(0,45,max_entries=2)
    c.add('color',image(100),1,1);c.add('depth',image(101),1,1)
    c.add('color',image(200),2,2);c.add('depth',image(200),2,2)
    assert c.pop()[0]==200
    # The late mate of evicted 100 must not be joined to the nearby 101.
    c.add('depth',image(100),3,3)
    assert c.pop() is None and set(c.pending)=={100,101}
    c.stop();first=c.snapshot();c.stop()
    assert c.snapshot()==first and first['retained_pixel_bytes']==0
    assert first['counts']['pending_capacity_evicted_sources']==1
    report=valid_report();report['receive']=first
    assert m.validate_capture(report)==(False,['diagnostic_input_loss_or_limit'])


def test_write_pair_preserves_same_objects_source_and_frame():
    c=m.ExactPairs(0,45)
    color,depth=image(1000000001,b'rgb'),image(1000000001,b'depth')
    c.add('depth',depth,1,20);c.add('color',color,2,30)
    calls=[]
    bag=SimpleNamespace(write=lambda *args:calls.append(args))
    pairs=[];m.write_pair(bag,c.pop(),pairs,Stamp)
    assert calls[0][1] is color and calls[1][1] is depth
    assert all(t.to_nsec()==1000000001 for _,_,t in calls)
    assert pairs[0]['images']['color']['frame_id']=='camera_link'
    assert pairs[0]['images']['depth']['data_bytes']==5


def test_mutated_source_rejected_before_write():
    c=m.ExactPairs(0,45);a,b=image(1),image(1)
    c.add('color',a,1,1);c.add('depth',b,1,1);a.header.stamp=Stamp(0,2)
    with pytest.raises(RuntimeError,match='mutated'):
        m.write_pair(SimpleNamespace(write=lambda *_:None),c.pop(),[],Stamp)


def test_exact_file_limit_enforced_even_on_seek_and_close(tmp_path):
    p=tmp_path/'test.bag.active';f=m.LimitedBagFile(p,8)
    f.write(b'12345678');f.seek(0);f.write(b'AB')
    f.seek(8)
    with pytest.raises(m.BudgetError):f.write(b'X')
    with pytest.raises(m.BudgetError):f.truncate(9)
    f.close();f.close()
    assert p.read_bytes()==b'AB345678' and f.closed


def test_file_creation_is_exclusive(tmp_path):
    p=tmp_path/'bag';p.write_bytes(b'old')
    with pytest.raises(FileExistsError):m.LimitedBagFile(p)
    assert p.read_bytes()==b'old'


def test_atomic_json_does_not_replace_old_on_size_error(tmp_path,monkeypatch):
    p=tmp_path/'r.json';m.atomic_json(p,{'old':True});monkeypatch.setattr(m,'MAX_JSON_BYTES',4)
    with pytest.raises(m.BudgetError):m.atomic_json(p,{'new':True})
    assert json.loads(p.read_text())=={'old':True}
    assert not p.with_name('r.json.tmp').exists()


@pytest.mark.parametrize('change,reason',[
    ({'bag_closed':False},'bag_not_closed'),({'errors':['disk']},'recorder_error'),
    ({'stop_reason':'interrupted'},'recording_did_not_run_full_window'),
    ({'capture_end_monotonic':54.9},'short_capture'),({'pair_count':0},'no_exact_raw_rgbd_pairs'),
    ({'receive':{'queue_pairs_remaining':1}},'writer_not_drained'),
    ({'receive':{'counts':{'memory_limit':1}}},'diagnostic_input_loss_or_limit')])
def test_capture_validity_fails_closed(change,reason):
    report=valid_report();report.update(change)
    valid,reasons=m.validate_capture(report)
    assert not valid and reason in reasons


def test_complete_sparse_data_report_not_claimed_full_rate():
    assert m.validate_capture(valid_report())==(True,[])
    # The report records two pairs, not invented 30 Hz output or target geometry.
    assert valid_report()['pair_count']==2


@pytest.mark.parametrize('returncode',[None,1,-9])
def test_parent_rejects_failed_or_running_child_even_if_bag_closed(returncode):
    ready=dict(pid=42,capture_start_monotonic=10.,capture_deadline_monotonic=55.)
    okay,reasons=m.validate_child_result(ready,valid_report(),42,returncode)
    assert not okay and 'owned_recorder_nonzero_or_still_running' in reasons


def test_parent_ready_owner_and_actual_capture_window_are_bound():
    ready=dict(pid=42,capture_start_monotonic=10.,capture_deadline_monotonic=55.)
    assert m.validate_child_result(ready,valid_report(),42,0)==(True,[])
    assert not m.validate_child_result(ready,valid_report(),43,0)[0]
    assert not m.validate_child_result(None,valid_report(),42,0)[0]
    ready['capture_start_monotonic']=11.
    assert not m.validate_child_result(ready,valid_report(),42,0)[0]


def test_only_owned_child_shutdown_escalation():
    events=[]
    class Child:
        def poll(self):return None
        def send_signal(self,sig):events.append(('signal',sig))
        def terminate(self):events.append(('terminate',))
        def kill(self):events.append(('kill',))
        def wait(self,timeout):
            events.append(('wait',timeout))
            if timeout!=1 or sum(x==('wait',1) for x in events)==1:
                raise subprocess.TimeoutExpired('owned',timeout)
    m.stop_owned(Child())
    assert [x[0] for x in events]==['signal','wait','terminate','wait','kill','wait']
    m.stop_owned(None)


def test_finished_child_not_signalled():
    m.stop_owned(SimpleNamespace(poll=lambda:0))


def test_invalid_mount_or_outside_path_rejected(tmp_path,monkeypatch):
    with pytest.raises(RuntimeError,match='SanStand'):m.external_directory(tmp_path/'new',create=True)
    assert not (tmp_path/'new').exists()


def test_metadata_uses_actual_reset_key(monkeypatch):
    entry={'source_stamp_ns':'123','read_completed_monotonic':1.,'evidence':{
        'regional_stationary_evidence':{'global_history_reset_reason':'sdk_source_discontinuity'},
        'raw_frames':{'color':{'frame_number':5}}}}
    value={'payload_json':json.dumps({'entries':[entry]})}
    monitor=object.__new__(m.Monitor)
    monitor.master=SimpleNamespace(getParam=lambda *_:(1,'',value))
    monitor.name='offline';monitor.rows=[];monitor.sdk={}
    monitor.observe();monitor.observe()
    assert len(monitor.sdk)==1
    assert monitor.sdk[123]['regional_reset_reason']=='sdk_source_discontinuity'
    assert monitor.sdk[123]['raw_frames']['color']['frame_number']==5


def test_ros_logs_have_external_bounded_configuration(tmp_path,monkeypatch):
    m.setup_logging(tmp_path)
    assert m.os.environ['ROS_LOG_DIR']==str(tmp_path/'ros')
    text=(tmp_path/'python_logging.conf').read_text()
    assert 'RotatingFileHandler' in text and '262144, 2' in text


def test_offline_real_rosbag_lz4_roundtrip(tmp_path):
    rosbag=pytest.importorskip('rosbag')
    Image=pytest.importorskip('sensor_msgs.msg').Image
    Time=pytest.importorskip('genpy').Time
    active=tmp_path/'raw.bag.active';closed=tmp_path/'raw.bag'
    stream=m.LimitedBagFile(active,1024**2)
    bag=rosbag.Bag(stream,'w',compression=rosbag.Compression.LZ4)
    msg=Image();msg.header.stamp=Time(10,7);msg.header.frame_id='camera_link'
    msg.height=2;msg.width=3;msg.encoding='mono8';msg.step=3;msg.data=b'abcdef'
    c=m.ExactPairs(0,45);c.add('color',msg,1,1);c.add('depth',msg,1,1)
    pairs=[];m.write_pair(bag,c.pop(),pairs,Time);bag.close();m.os.replace(active,closed)
    with rosbag.Bag(closed,'r') as b:
        rows=list(b.read_messages())
    assert len(rows)==2 and all(x.message.data==b'abcdef' for x in rows)
    assert all(x.timestamp.to_nsec()==10000000007 for x in rows)
    assert all(x.message.header.stamp.to_nsec()==10000000007 for x in rows)


@pytest.mark.parametrize('emit,close_error,expected',[(True,False,0),(False,False,2),(True,True,2)])
def test_child_lifecycle_with_fake_ros_only(tmp_path,monkeypatch,emit,close_error,expected):
    import sys
    subs=[]
    now=[0.0]
    serial=[0]
    def sleep(dt):
        now[0]+=dt
        if emit:
            serial[0]+=1
            for sub in subs:
                if not sub.closed:sub.callback(image(serial[0]),sub.key)
    class Subscriber:
        def __init__(self,topic,kind,callback,callback_args,**kw):
            self.callback=callback;self.key=callback_args;self.closed=False;subs.append(self)
        def unregister(self):self.closed=True
    class FakeTime(Stamp):
        @staticmethod
        def now():return Stamp(0,int(now[0]*10**9)+1)
    fake_ros=SimpleNamespace(init_node=lambda *a,**k:None,Subscriber=Subscriber,
                             Time=FakeTime,is_shutdown=lambda:False)
    class FakeBag:
        def __init__(self,stream,*a,**k):self.stream=stream
        def write(self,topic,message,stamp):self.stream.write(message.data)
        def close(self):
            if close_error:raise OSError('injected close failure')
            self.stream.close()
    monkeypatch.setitem(sys.modules,'rospy',fake_ros)
    monkeypatch.setitem(sys.modules,'rosbag',SimpleNamespace(Bag=FakeBag,Compression=SimpleNamespace(LZ4='lz4')))
    monkeypatch.setitem(sys.modules,'sensor_msgs',SimpleNamespace())
    monkeypatch.setitem(sys.modules,'sensor_msgs.msg',SimpleNamespace(Image=object))
    monkeypatch.setattr(m,'external_directory',lambda path,create:Path(path))
    monkeypatch.setattr(m,'DURATION',.01)
    monkeypatch.setattr(m,'time',SimpleNamespace(monotonic=lambda:now[0],sleep=sleep))
    rc=m.record_child(tmp_path)
    assert rc==expected and all(sub.closed for sub in subs)
    out=json.loads((tmp_path/'outcome.json').read_text())
    assert out['capture_valid']==(expected==0)
    if close_error:
        assert not out['bag_closed'] and (tmp_path/'raw_rgbd.bag.active').exists()
        assert not (tmp_path/'raw_rgbd.bag').exists()
    else:
        assert out['bag_closed'] and not (tmp_path/'raw_rgbd.bag.active').exists()
    if not emit:assert out['pair_count']==0
