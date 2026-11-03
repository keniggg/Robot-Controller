#!/usr/bin/env python3
"""Extract only immutable R3 task continuation inputs; never connect to ROS."""
import hashlib
import io
import json
from pathlib import Path
import rosbag

ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N = ROOT / 'ros_log/full_latest_contact_admission_20261013T011103Z'
R = ROOT / 'grasp_records/20261013T083836.150489Z_f4adfdd7ede44edeb725b4bfd7ac4f32'
O = N / 'verification/gen15_continuation_inputs_r1'
B = R / 'keyframes.bag'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
prior = json.loads((N / 'verification/gen15_qualified_owner_diagnosis/result.json').read_text())
before = B.stat()
assert sha(B) == prior['source_bag_sha256']
O.mkdir(exist_ok=False)
(O / 'raw').mkdir()
topics = ['/tf', '/tf_static', '/alicia_d/accepted_joint_states', '/alicia_d/sdk_command', '/joint_states']
rows = []
with rosbag.Bag(str(B)) as bag:
    available = bag.get_type_and_topic_info().topics
    accepted = [name for name in available if 'accepted' in name and available[name].msg_type == 'sensor_msgs/JointState']
    topics = sorted(set(topics + accepted))
    for topic, msg, received in bag.read_messages(topics=topics):
        ns = received.to_nsec()
        if topic != '/tf_static' and not 1791880761500000000 <= ns <= 1791880764200000000:
            continue
        raw = io.BytesIO(); msg.serialize(raw)
        path = O / ('raw/%05d.rosmsg' % len(rows)); path.write_bytes(raw.getvalue())
        rows.append(dict(index=len(rows), topic=topic, received_ns=ns,
                         datatype=msg._type, md5sum=msg._md5sum,
                         file=str(path), sha256=sha(path)))
after = B.stat()
assert (before.st_size, before.st_mtime_ns) == (after.st_size, after.st_mtime_ns)
configuration = R / 'perception_execution_configuration.json'
out = dict(passed=True, source_bag=str(B), source_bag_sha256=prior['source_bag_sha256'],
           source_bag_unchanged=True, rows=rows, topic_counts={t:sum(r['topic']==t for r in rows) for t in topics},
           configuration=str(configuration), configuration_sha256=sha(configuration),
           script_sha256=sha(Path(__file__)), no_live_ros=True, no_motion=True,
           window_received_ns=[1791880761500000000,1791880764200000000])
(O / 'manifest.json').write_text(json.dumps(out, ensure_ascii=False, indent=2)+'\n')
print(json.dumps(dict(path=str(O/'manifest.json'), sha256=sha(O/'manifest.json'),
                     counts=out['topic_counts']), ensure_ascii=False))
