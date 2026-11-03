"""Minimal original mask data plus exact recorded three-topic fan-in; offline only."""
import base64, hashlib, io, json, os, platform, struct, sys, time
from pathlib import Path
import extract_gen52_g51_station_wire_20261018 as prior
import extract_gen50_retry_r30_rgb_20261018 as boundary
W, E, G = prior.W, prior.E, prior.G
OUT = prior.OUT / 'mask_fanin_r1'

def main():
    assert E.parent.parent.is_mount() and not OUT.exists()
    OUT.mkdir(); (OUT / 'original_masks').mkdir()
    os.environ.update(PYTHONDONTWRITEBYTECODE='1', ROS_MASTER_URI='http://127.0.0.1:1',
                      ROS_LOG_DIR=str(OUT), ROS_HOME=str(OUT), TMPDIR=str(OUT),
                      OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
    sys.dont_write_bytecode = True
    sys.path[:0] = [str(W / 'devel/lib/python3/dist-packages'), '/opt/ros/noetic/lib/python3/dist-packages']
    native = os.uname()
    platform.uname = lambda: platform.uname_result(*(tuple(native) + (native.machine,)))
    platform.architecture = lambda *a, **k: (str(8 * struct.calcsize('P')) + 'bit', 'ELF')
    boundary.OUT = OUT
    sys.addaudithook(boundary.audit)
    import numpy as np
    import rosbag
    extracted = json.loads((prior.OUT / 'extraction.json').read_text())
    def rows(topic):
        info = extracted['files'][topic]
        assert boundary.sha(info['path']) == info['sha256']
        return json.loads(Path(info['path']).read_text())['messages']
    statuses = rows('/perception/unknown/detector_status')
    ready = [(i, r) for i, r in enumerate(statuses)
             if (r['parsed_json'] or {}).get('state') == 'ready'
             and isinstance((r['parsed_json'] or {}).get('station_compute_wait'), dict)]
    wanted = {r['parsed_json']['station_compute_wait']['source_ns'] for _, r in ready}
    assert wanted
    saved = {s for s in wanted if s >= 1792330048000000000} | {min(wanted)}
    m = json.loads((G / 'manifest.json').read_text())
    assert m['state'] == 'complete' and m['recorder_closed'] is True and m['result'] == 'failure'
    bref = next(x for x in m['files'] if x['path'] == 'keyframes.bag')
    bp = G / 'keyframes.bag'
    assert boundary.sha(bp) == bref['sha256']
    masks = []
    with rosbag.Bag(str(bp), 'r') as bag:
        for topic, msg, recv in bag.read_messages(topics=['/perception/object_mask']):
            assert msg.encoding == 'mono8' and msg.height > 0 and msg.width > 0 and msg.step >= msg.width
            raw = bytes(msg.data)
            assert len(raw) == msg.height * msg.step
            raster = np.frombuffer(raw, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width]
            binary = np.ascontiguousarray(raster != 0, dtype=np.uint8)
            ns = msg.header.stamp.to_nsec()
            stream = io.BytesIO(); msg.serialize(stream); wire = stream.getvalue()
            row = dict(source_stamp_ns=ns, bag_timestamp_ns=recv.to_nsec(), frame_id=msg.header.frame_id,
                       shape=[msg.height, msg.width], step=msg.step, encoding=msg.encoding,
                       raw_data_sha256=hashlib.sha256(raw).hexdigest(),
                       normalized_binary_sha256=hashlib.sha256(binary.tobytes()).hexdigest(),
                       nonzero_pixels=int(np.count_nonzero(binary)), wire_sha256=hashlib.sha256(wire).hexdigest())
            if ns in saved:
                p = OUT / 'original_masks' / ('%d_%d.json' % (ns, len(masks)))
                prior.put(p, dict(row, message_type=msg._type, md5sum=msg._md5sum,
                                  wire_base64=base64.b64encode(wire).decode()))
                row['original_message'] = dict(path=str(p), sha256=boundary.sha(p))
            masks.append(row)
    objby = {}; maskby = {}
    for i, r in enumerate(rows('/perception/object')):
        stamp = r['payload']['header']['stamp']; ns = stamp['secs'] * 10**9 + stamp['nsecs']
        objby.setdefault(ns, []).append((i, r))
    for i, r in enumerate(masks): maskby.setdefault(r['source_stamp_ns'], []).append((i, r))
    joins = []
    for i, r in ready:
        p = r['parsed_json']; c = p['station_compute_wait']; ns = c['source_ns']
        oo, mm = objby.get(ns, []), maskby.get(ns, [])
        expected = p['station_compute_mask']
        exact = bool(mm) and all(x['normalized_binary_sha256'] == expected['sha256']
             and x['shape'] == expected['shape'] and x['frame_id'] == expected['frame_id']
             and x['nonzero_pixels'] > 0 for _, x in mm)
        latest = max([r['received_ns']] + [x['received_ns'] for _, x in oo] + [x['bag_timestamp_ns'] for _, x in mm])
        joins.append(dict(source_ns=ns, status_index=i, status_bag_timestamp_ns=r['received_ns'],
            status_wire_sha256=r['wire_sha256'], object_indices=[j for j, _ in oo],
            object_bag_timestamps_ns=[x['received_ns'] for _, x in oo], mask_indices=[j for j, _ in mm],
            mask_bag_timestamps_ns=[x['bag_timestamp_ns'] for _, x in mm],
            object_detected_true=bool(oo) and all(x['payload']['detected'] is True for _, x in oo),
            producer_mask_descriptor=expected, mask_exact_descriptor_match=exact,
            recorder_three_topic_present=bool(oo and mm), recorder_latest_ns=latest,
            geometry_deadline_ns=c['geometry_deadline_ns'], identity_deadline_ns=c['identity_deadline_ns'],
            recorder_all_three_before_geometry_expiry=bool(oo and mm) and latest < c['geometry_deadline_ns'],
            recovery_parent_source_ns=(p.get('station_compute_recovery') or {}).get('source_ns'),
            recovery_chain_sources=[x.get('source_ns') for x in p.get('station_compute_recovery_chain', [])],
            feedback_gap_recovery=p.get('station_feedback_gap_recovery')))
    assert not boundary.BOUNDARY and boundary.sha(bp) == bref['sha256']
    prior.put(OUT / 'mask_descriptors.json', masks)
    prior.put(OUT / 'station_READY_three_topic_fanin.json', joins)
    result = dict(passed=True, kind='GEN51_original_masks_and_recorded_three_topic_fanin',
        original_extraction_sha256=boundary.sha(prior.OUT / 'extraction.json'),
        bag_sha256=bref['sha256'], bag_before_after_same=True, mask_topic_whitelist=['/perception/object_mask'],
        mask_count=len(masks), ready_count=len(joins), full_triplet_count=sum(x['recorder_three_topic_present'] for x in joins),
        matched_count=sum(x['mask_exact_descriptor_match'] and x['object_detected_true'] for x in joins),
        original_mask_wires_saved_for_sources=sorted(saved),
        final_READY=joins[-1], helper_sha256=boundary.sha(__file__),
        mask_descriptors=dict(path=str(OUT/'mask_descriptors.json'),sha256=boundary.sha(OUT/'mask_descriptors.json')),
        fanin=dict(path=str(OUT/'station_READY_three_topic_fanin.json'),sha256=boundary.sha(OUT/'station_READY_three_topic_fanin.json')),
        recorder_timestamp_semantics='Status metadata bag timestamp is recorder callback write. Same-source object/mask/RGB/depth bag timestamp is recorder bundle completion, not original individual callbacks. Neither proves Task processing order or admission.',
        external_boundary_attempts=boundary.BOUNDARY, ROS_initialized=False, hardware_contacted=False,
        temperature_read=False, RGB_or_depth_read=False, source_record_modified=False, created_ns=time.time_ns())
    prior.put(OUT/'result.json', result)
    print(json.dumps(dict(result=str(OUT/'result.json'), sha256=boundary.sha(OUT/'result.json'),
       ready=len(joins), triplets=result['full_triplet_count'], matched=result['matched_count'],
       late_sources=[x['source_ns'] for x in joins if not x['recorder_all_three_before_geometry_expiry']]),ensure_ascii=False))

if __name__ == '__main__': main()
