"""Read only the three exact sealed RGB-D pair headers and code provenance."""
from pathlib import Path
import hashlib,json
import rosbag,rospy
WORK=Path(__file__).resolve().parents[1]
ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
OUT=ROOT/'ros_log/full_latest_restart_20261010T002347Z/gen13_handoff_analysis/offline_camera_chain'
audit=json.loads((OUT/'camera_geometry_audit.json').read_text())
rec=Path(audit['record']);wanted=[f['source_stamp_ns'] for f in audit['frames']]
topic_names=['/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw','/experiment/observation_rgbd/color','/experiment/observation_rgbd/depth']
pairs={ns:{} for ns in wanted}
with rosbag.Bag(str(rec/'keyframes.bag')) as bag:
    for ns in wanted:
        for topic,msg,received in bag.read_messages(topics=topic_names,
                start_time=rospy.Time.from_sec(ns*1e-9-.1),end_time=rospy.Time.from_sec(ns*1e-9+1.)):
            if msg.header.stamp.to_nsec()!=ns:continue
            key='rgb' if '/color' in topic else 'depth'
            row=dict(topic=topic,received_ns=received.to_nsec(),source_stamp_ns=ns,seq=msg.header.seq,
                frame_id=msg.header.frame_id,width=msg.width,height=msg.height,encoding=msg.encoding,step=msg.step,
                is_bigendian=msg.is_bigendian,bytes_sha256=hashlib.sha256(msg.data).hexdigest())
            if key in pairs[ns]:assert pairs[ns][key]['bytes_sha256']==row['bytes_sha256']
            pairs[ns][key]=row
for ns,pair in pairs.items():
    assert set(pair)=={'rgb','depth'}
    a,b=pair['rgb'],pair['depth']
    assert a['source_stamp_ns']==b['source_stamp_ns'] and a['frame_id']==b['frame_id']=='camera_link'
    assert (a['width'],a['height'])==(b['width'],b['height'])==(653,489)
    assert a['encoding']=='bgr8' and b['encoding']=='16UC1'
manifest=json.loads((rec/'source_manifest.json').read_text())['source_sha256']
sources={}
for rel in ['src/alicia_flexible_grasp_supervisor/scripts/camera_node.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/realsense_manager.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/rgbd_rectification.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/latest_rgbd_publication.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/tabletop.py']:
    actual=hashlib.sha256((WORK/rel).read_bytes()).hexdigest();saved=manifest.get(rel)
    sources[rel]=dict(inspected_sha256=actual,recorded_source_manifest_sha256=saved,
        exact_manifest_match=(actual==saved) if saved else None)
report=dict(exact_pairs=pairs,geometry_audit=str(OUT/'camera_geometry_audit.json'),
    geometry_audit_sha256=hashlib.sha256((OUT/'camera_geometry_audit.json').read_bytes()).hexdigest(),
    sources=sources,source_binding_limit='Camera files are absent from the sealed grasp source manifest. Current source inspection is not by itself proof of running source identity. The recorded geometry and both map hashes independently match the numerical replay.',
    code_audit=[
        'realsense_manager aligns to color before depth processing; paired rectification runs after processing and preserves each aligned uint16 value.',
        'camera_node publishes the same source stamp and camera_link frame on BGR8 and 16UC1 arrays. RGBDSnapshot owns immutable byte copies of the complete pair.',
        'Published 653x489 geometry has the same fx/fy as original 640x480 source and cx/cy shifted by +7/+6; there is no focal rescale or image crop in this geometry.',
        'Published pixels are already rectified and carry distortion model none. Applying original color inverse_brown distortion a second time would be incorrect.',
        'The only resized depth path is the preview topic; no selected measurement uses the preview.',
        'Depth meters are uint16 counts times 9.999999747378752e-05. The proper optical-to-camera_link rotation maps optical [X,Y,Z] to [Z,-X,-Y].',
    ],
    conclusion='No reproducible intrinsics, dimensions, units, duplicate-distortion, axis-convention, or pairing bug was found in the available GEN13 evidence. The approximately 5.8 mm inter-view support disagreement survives independent plane fitting and removal of rectification pixel quantization.',
    exclusions=['No CameraInfo messages were recorded; no CameraInfo-derived guarantee is claimed.',
        'Unaligned native frames are absent, so original SDK alignment/filter output cannot be independently reconstructed byte-for-byte.',
        'No regression or production fix was fabricated when this scoped audit found no confirmed bug.'],
    no_ros_master_access=True,no_device_access=True,no_live_edits=True)
p=OUT/'pair_binding_and_code_review.json'
with p.open('x') as f:json.dump(report,f,indent=2,ensure_ascii=False);f.write('\n')
print(json.dumps(dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),pairs=pairs)))
