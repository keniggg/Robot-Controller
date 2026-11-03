"""Bounded exact four-source availability; preserve original masks and pixels."""
import hashlib, io, json, os, platform, struct, sys
from pathlib import Path
import extract_gen50_retry_r30_rgb_20261018 as base

E, N, G = base.E, base.N, base.G
OUT = N / 'verification/gen50_retry_actual_handoff_r1/station_source_availability_r1'
STAMPS = (1792327026858420848, 1792327027058121204,
          1792327027257877588, 1792327029323935508)

def main():
    assert E.parent.parent.is_mount() and not OUT.exists()
    OUT.mkdir(); (OUT / 'arrays').mkdir()
    os.environ.update(PYTHONDONTWRITEBYTECODE='1', ROS_MASTER_URI='http://127.0.0.1:1',
                      ROS_LOG_DIR=str(OUT), ROS_HOME=str(OUT), TMPDIR=str(OUT),
                      OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
    sys.dont_write_bytecode = True
    native = os.uname()
    platform.uname = lambda: platform.uname_result(*(tuple(native) + (native.machine,)))
    platform.architecture = lambda *a, **k: (str(8 * struct.calcsize('P')) + 'bit', 'ELF')
    base.OUT = OUT
    sys.addaudithook(base.audit)
    import numpy as np
    import rosbag
    import genpy
    manifest = json.loads((G / 'manifest.json').read_text())
    assert manifest['state'] == 'complete' and manifest['result'] == 'failure' and manifest['recorder_closed'] is True
    bagref = next(x for x in manifest['files'] if x['path'] == 'keyframes.bag')
    bagpath = G / 'keyframes.bag'
    assert bagpath.stat().st_size == bagref['size_bytes'] and base.sha(bagpath) == bagref['sha256']
    requested_topics = {
        '/supervisor/camera/color/image_raw': 'color_bgr',
        '/supervisor/camera/depth/image_raw': 'depth_raw',
        '/perception/object_mask': 'mask_original',
        '/experiment/observation_rgbd/color': 'color_bgr',
        '/experiment/observation_rgbd/depth': 'depth_raw',
        '/experiment/observation_rgbd/mask': 'mask_original',
    }
    records = {ns: [] for ns in STAMPS}; objects = {}
    with rosbag.Bag(str(bagpath), 'r') as bag:
        info = bag.get_type_and_topic_info().topics
        topics = [t for t in requested_topics if t in info]
        other_image_topics = sorted(t for t, v in info.items() if v.msg_type == 'sensor_msgs/Image' and t not in requested_topics)
        lo = genpy.Time(min(STAMPS) // 10**9 - 1, 0)
        hi = genpy.Time(max(STAMPS) // 10**9 + 4, 0)
        for topic, msg, received in bag.read_messages(topics=topics, start_time=lo, end_time=hi):
            ns = msg.header.stamp.to_nsec()
            if ns not in records:
                continue
            kind = requested_topics[topic]; enc = msg.encoding.lower(); raw = bytes(msg.data)
            if enc in ('bgr8', 'rgb8'):
                dtype, channels = np.dtype('u1'), 3
            elif enc in ('mono8', '8uc1'):
                dtype, channels = np.dtype('u1'), 1
            elif enc in ('16uc1', 'mono16'):
                dtype, channels = np.dtype('>u2' if msg.is_bigendian else '<u2'), 1
            elif enc == '32fc1':
                dtype, channels = np.dtype('>f4' if msg.is_bigendian else '<f4'), 1
            else:
                raise ValueError('unsupported original encoding ' + msg.encoding)
            assert len(raw) == msg.step * msg.height
            row = np.frombuffer(raw, dtype=dtype).reshape(msg.height, msg.step // dtype.itemsize)
            shape = (msg.height, msg.width, channels) if channels > 1 else (msg.height, msg.width)
            a = row[:, :msg.width * channels].reshape(shape).copy()
            if enc == 'rgb8':
                a = a[:, :, ::-1].copy()
            a = a.astype(a.dtype.newbyteorder('='), copy=False)
            key = hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()
            if key not in objects:
                path = OUT / 'arrays' / (key + '.npy')
                with path.open('xb') as f:
                    np.save(f, a, allow_pickle=False)
                restored = np.load(path, allow_pickle=False)
                assert restored.shape == a.shape and restored.dtype == a.dtype and restored.tobytes() == a.tobytes()
                objects[key] = dict(path=str(path), file_sha256=base.sha(path), array_sha256=key,
                                    shape=list(a.shape), dtype=str(a.dtype), bytes=int(a.nbytes))
            wire = io.BytesIO(); msg.serialize(wire)
            item = dict(kind=kind, topic=topic, exact_source_stamp_ns=ns, received_ns=received.to_nsec(),
                        frame_id=msg.header.frame_id, encoding=msg.encoding, step=msg.step,
                        is_bigendian=msg.is_bigendian, wire_data_sha256=hashlib.sha256(raw).hexdigest(),
                        serialized_message_sha256=hashlib.sha256(wire.getvalue()).hexdigest(),
                        array=objects[key], raw_mask_not_replaced_or_thresholded=(kind == 'mask_original'))
            if kind == 'mask_original':
                item.update(nonzero_count=int(np.count_nonzero(a)), unique_values=[int(x) for x in np.unique(a)])
            records[ns].append(item)
    by_source = []
    for ns in STAMPS:
        present = sorted({x['kind'] for x in records[ns]})
        by_source.append(dict(source_stamp_ns=ns, exact_messages=records[ns], available_kinds=present,
                              missing_kinds=sorted(set(requested_topics.values()) - set(present)),
                              source_role='R30 final computation sample' if ns == STAMPS[-1] else 'arrival qualification source'))
    expected = json.loads((G / 'keyframes_audits' / (base.AUDIT_SHA + '.json')).read_text())['remote_diagnostics']['tabletop_geometry']['regular_box_inference']['retained_pixel_sha256']
    final = records[STAMPS[-1]]
    matches = {k: any(x['kind'] == kind and x['array']['array_sha256'] == expected[k] for x in final)
               for k, kind in [('color_sha256', 'color_bgr'), ('depth_sha256', 'depth_raw'), ('mask_sha256', 'mask_original')]}
    result = dict(kind='bounded_exact_recorded_input_availability_not_qualification_replay',
                  record=str(G), manifest_sha256=base.sha(G / 'manifest.json'), bag_sha256=bagref['sha256'],
                  requested_source_stamps_ns=list(STAMPS), checked_topics=topics,
                  absent_requested_topics=sorted(set(requested_topics) - set(topics)),
                  other_image_topics_not_claimed_checked=other_image_topics, by_source=by_source,
                  unique_array_count=len(objects), r30_final_retained_hash_matches=matches,
                  missing_inputs_synthesized=False, mask_or_fused_geometry_invented=False,
                  own_source_TF_geometry_not_reconstructed=True, full_request_reproduction_claimed=False,
                  original_qualification_or_execution_not_run=True, network_ROS_hardware_access=False,
                  temperature_read=False, boundary_attempts=base.BOUNDARY,
                  helper_sha256=base.sha(Path(__file__)), reusable_guard_sha256=base.sha(Path(base.__file__)))
    p = OUT / 'index.json'
    p.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    assert json.loads(p.read_text()) == result
    print(json.dumps(dict(index=str(p), sha256=base.sha(p), sources=[dict(source_stamp_ns=x['source_stamp_ns'],
          available=x['available_kinds'], missing=x['missing_kinds']) for x in by_source],
          final_retained_hash_matches=matches), ensure_ascii=False))

if __name__ == '__main__':
    main()
