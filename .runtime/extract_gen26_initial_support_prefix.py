"""Copy bounded open-bag prefix and index only an independent in-memory copy."""
import hashlib
import io
import json
import os
from pathlib import Path
import rosbag
import rospy

root = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
source = root/'grasp_records/20261015T051744.120111Z_0ffc956cd2e54398a35ebff62282e999/keyframes.bag'
out = root/'ros_log/full_latest_gen22_restart_20261015T003700Z/verification/gen26_attempt1_initial_support_prefix'
assert Path('/media/zhuyupei/SanStand').is_mount()
assert not os.statvfs(root).f_flag & os.ST_RDONLY
before = source.stat()
limit = 64*1024*1024
if out.exists():
    assert sorted(p.name for p in out.iterdir()) == ['prefix_64MiB.bag']
    prefix = (out/'prefix_64MiB.bag').read_bytes()
else:
    with source.open('rb') as stream:
        prefix = stream.read(limit)
    out.mkdir()
    (out/'prefix_64MiB.bag').write_bytes(prefix)
after = source.stat()
assert (before.st_dev, before.st_ino) == (after.st_dev, after.st_ino)
buffer = io.BytesIO(prefix)
buffer.mode = 'rb'
rows = []
with rosbag.Bag(buffer, mode='r', allow_unindexed=True) as bag:
    chunks = list(bag.reindex())
    bag._connection_indexes_read = True
    topics = sorted({connection.topic for connection in bag._connections.values()})
    print(json.dumps(dict(topics=topics, indexed_chunks=len(chunks))))
    for topic, message, stamp in bag.read_messages(topics=['/perception/unknown/detector_status'],
            start_time=rospy.Time.from_sec(1792041464.), end_time=rospy.Time.from_sec(1792041480.)):
        data = json.loads(message.data)
        rows.append(dict(topic=topic, record_stamp_ns=stamp.to_nsec(), message=data))
receipt = dict(source_path=str(source), source_initial_size=before.st_size,
    prefix_size=len(prefix), prefix_sha256=hashlib.sha256(prefix).hexdigest(),
    source_inode=before.st_ino, original_reindexed=False, original_modified=False,
    reindex_scope='independent_in_memory_prefix_only', message_count=len(rows),
    ros_master_contacted=False, motion_commands_sent=False)
(out/'detector_status.json').write_text(json.dumps(rows, ensure_ascii=False, indent=2)+'\n')
(out/'receipt.json').write_text(json.dumps(receipt, ensure_ascii=False, indent=2)+'\n')
for row in rows:
    m = row['message']; t = m.get('frame_timing', {})
    print(json.dumps(dict(record_stamp_ns=row['record_stamp_ns'], source=m.get('source_stamp_ns'),
        state=m.get('state'), detail=m.get('detail'), initial_reason=t.get('initial_station_invalidated_reason'),
        failed_stage=t.get('failed_stage'), pending=m.get('locked_source_status'),
        source_age=t.get('status_source_age_sec')), ensure_ascii=False))
print(json.dumps(receipt, ensure_ascii=False))
