#!/usr/bin/env python3
"""Export sealed original color/mask messages only; no segmentation or ROS calls."""
from pathlib import Path
import hashlib,json
import cv2
import rosbag
from cv_bridge import CvBridge
from PIL import Image,ImageDraw

ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
RECORD=ROOT/'grasp_records/20261009T042814.711798Z_02d7a2117e114e0ea476744c19fa64c4'
SESSION=ROOT/'ros_log/full_latest_ros_20261009T003407Z'
OUT=SESSION/'gen12_arrival_analysis/reference_images'
analysis=json.loads((SESSION/'gen12_arrival_analysis/analysis.json').read_text())
assert json.loads((RECORD/'manifest.json').read_text())['recorder_closed']
request=next(r['message'] for r in analysis['requests'] if r['message']['state']=='executing')
reference_ns=1791520119101696014
reference_set={reference_ns,1791520118901880264,1791520096896853685}
arrival=analysis['arrival_ns'];deadline=analysis['capture_deadline_ns']
bridge=CvBridge();rows=[];statuses=[];reference_mask=None;reference_rgb=None
OUT.mkdir(parents=True,exist_ok=True)
online={r['message'].get('source_stamp_ns'):r['message'].get('observation_measurement',{}).get('reason')
        for r in analysis['arrival_statuses'] if arrival<=r['received_ns']<=deadline}
with rosbag.Bag(str(RECORD/'keyframes.bag')) as bag:
    for topic,msg,received in bag.read_messages(topics=[
        '/supervisor/camera/color/image_raw','/perception/object_mask',
        '/experiment/observation_rgbd/color','/perception/unknown/detector_status']):
        if topic.endswith('detector_status'):
            d=json.loads(msg.data)
            if d.get('generation')==12 and request['stamp_ns']-1_000_000_000<received.to_nsec()<request['stamp_ns']+200_000_000:
                statuses.append(dict(received_ns=received.to_nsec(),message=d))
            continue
        ns=msg.header.stamp.to_nsec()
        is_mask=topic.endswith('object_mask')
        is_reference=ns in reference_set
        is_arrival=arrival<ns<=deadline and topic.endswith('/color')
        if not is_reference and not is_arrival:continue
        if is_mask and not is_reference:continue
        encoding='mono8' if is_mask else 'bgr8'
        data=bridge.imgmsg_to_cv2(msg,encoding)
        kind='mask' if is_mask else 'rgb'
        role='reference' if is_reference else 'arrival'
        name='%s_%d_%s.png'%(role,ns,kind)
        path=OUT/name
        assert not path.exists(),path
        assert cv2.imwrite(str(path),data)
        row=dict(role=role,kind=kind,source_stamp_ns=ns,original_topic=topic,
                 original_frame_id=msg.header.frame_id,original_sequence=msg.header.seq,
                 bag_received_ns=received.to_nsec(),file=name,shape=list(data.shape),
                 png_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                 online_evaluation_reason=online.get(ns),online_evaluated_source=ns in online)
        if is_mask:
            ys,xs=(data>0).nonzero()
            row['original_mask_nonzero_pixels']=int(len(xs))
            row['original_mask_bbox_xyxy']=None if not len(xs) else [int(xs.min()),int(ys.min()),int(xs.max()+1),int(ys.max()+1)]
        if ns==reference_ns:
            if is_mask:reference_mask=data
            else:reference_rgb=data
        rows.append(row)
assert reference_mask is not None and reference_rgb is not None
display=[r for r in rows if r['kind']=='rgb' and (r['source_stamp_ns']==reference_ns or r['role']=='arrival')]
display=sorted(display,key=lambda r:(0 if r['role']=='reference' else 1,r['source_stamp_ns']))
tile_w=326;tile_h=268;sheet=Image.new('RGB',(3*tile_w,((len(display)+2)//3)*tile_h),(30,30,30));draw=ImageDraw.Draw(sheet)
for i,r in enumerate(display):
    im=Image.open(OUT/r['file']).convert('RGB');im.thumbnail((tile_w,244))
    x=(i%3)*tile_w;y=(i//3)*tile_h;sheet.paste(im,(x,y+24))
    draw.text((x+3,y+3),r['role']+' '+str(r['source_stamp_ns']),fill=(255,255,255))
sheet_path=OUT/'recorded_rgb_contact_sheet.png';sheet.save(sheet_path)
manifest=dict(record=str(RECORD),executing_request=request,last_pre_motion_reference_source_ns=reference_ns,
    interpretation='Original saved last ready RGB and refined mask before executing request, with original post-arrival RGB frames. No segmentation rerun and no generated mask.',
    files=rows,reference_near_execution_statuses=statuses,contact_sheet=sheet_path.name,
    limitations=['Contact sheet is resized for human comparison; use individual PNGs for exact pixels.',
        'The 5 Hz saved arrival frames may differ from online evaluated source frames; per-file provenance marks exact matches.',
        'No arrival mask was published during pending requalification, so none is fabricated.'],
    no_ros_actions=True,production_edited=False)
path=OUT/'manifest.json'
with path.open('x') as f:json.dump(manifest,f,indent=2,ensure_ascii=False)
print(json.dumps(dict(path=str(path),reference_rgb=str(OUT/('reference_%d_rgb.png'%reference_ns)),
    reference_mask=str(OUT/('reference_%d_mask.png'%reference_ns)),arrival_rgb_count=sum(r['role']=='arrival' for r in rows),
    reference_mask_bbox=next(r['original_mask_bbox_xyxy'] for r in rows if r['kind']=='mask' and r['source_stamp_ns']==reference_ns)),ensure_ascii=False))
