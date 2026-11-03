"""Read exact source RGB/depth/mask frames from a closed local grasp record."""
import argparse
import hashlib
import json
import os
from pathlib import Path


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(4*1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--record', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--stamp', type=int, action='append', required=True)
    a = ap.parse_args()
    mount = Path('/media/zhuyupei/SanStand')
    assert mount.is_mount()
    root = mount/'实验记录/Robot-Controller'
    a.record.resolve().relative_to(root)
    a.output.resolve().relative_to(root)
    assert not a.output.exists()
    manifest_path = a.record/'manifest.json'
    manifest_hash = sha(manifest_path)
    manifest = json.loads(manifest_path.read_text())
    assert manifest['state'] == 'complete' and manifest['recorder_closed'] is True
    bagpath = a.record/'keyframes.bag'
    item = next(x for x in manifest['files'] if x['path'] == 'keyframes.bag')
    assert bagpath.stat().st_size == item['size_bytes'] and sha(bagpath) == item['sha256']
    a.output.mkdir()
    import cv2
    import numpy as np
    import rosbag
    from cv_bridge import CvBridge
    bridge = CvBridge()
    roles = {
        '/supervisor/camera/color/image_raw':'color',
        '/supervisor/camera/depth/image_raw':'depth',
        '/perception/object_mask':'mask',
        '/experiment/observation_rgbd/color':'color',
        '/experiment/observation_rgbd/depth':'depth',
    }
    found, inventory = {}, {}
    with rosbag.Bag(str(bagpath), 'r') as bag:
        for topic, message, stamp in bag.read_messages(topics=list(roles)):
            source = message.header.stamp.to_nsec()
            if source not in a.stamp:
                continue
            role = roles[topic]
            identity = (source, role)
            payload = hashlib.sha256(bytes(message.data)).hexdigest()
            if identity in found:
                assert found[identity]['message_data_sha256'] == payload
                continue
            pixels = bridge.imgmsg_to_cv2(message, 'bgr8' if role == 'color' else 'passthrough')
            name = str(source)+'_'+role+('.npy' if role == 'depth' else '.png')
            path = a.output/name
            if role == 'depth':
                np.save(str(path), pixels, allow_pickle=False)
            else:
                assert cv2.imwrite(str(path), pixels)
            found[identity] = dict(source_stamp_ns=source, role=role, topic=topic,
                recorded_ns=stamp.to_nsec(), encoding=message.encoding,
                width=message.width, height=message.height,
                message_data_sha256=payload, output=str(path), output_sha256=sha(path))
            inventory[name] = sha(path)
    assert sha(manifest_path) == manifest_hash and sha(bagpath) == item['sha256']
    boundary = Path(os.environ['OFFLINE_BOUNDARY_LOG'])
    attempts = boundary.read_text().splitlines() if boundary.exists() else []
    assert not attempts
    receipt = dict(record=str(a.record), requested_stamps_ns=a.stamp,
        found=list(found.values()), missing=[dict(source_stamp_ns=s, role=r)
            for s in a.stamp for r in ('color','depth','mask') if (s,r) not in found],
        source_sha256={str(manifest_path):manifest_hash,str(bagpath):item['sha256']},
        output_sha256=inventory, extractor_sha256=sha(__file__),
        source_unchanged=True, exact_sources_only=True,
        external_boundary_attempts=attempts, ros_initialized=False,
        temperature_topics_read=False, motion_commands_sent=False)
    (a.output/'receipt.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(found=len(found),missing=receipt['missing'],output=str(a.output)),ensure_ascii=False))


if __name__ == '__main__':
    main()
