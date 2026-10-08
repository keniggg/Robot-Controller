"""Offline-only original source/TF/transport checks of a sealed diagnostic ZIP."""
import collections,hashlib,json,zlib,zipfile
from pathlib import Path
import rospy
import tf2_py
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage

BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/gen17_support_measurement_20261006')
CAP=BASE/'raw_support_v2'

def sha(b):return hashlib.sha256(b).hexdigest()
def stats(a):
 a=sorted(a)
 return dict(count=len(a),median=a[len(a)//2]if a else None,p95=a[min(len(a)-1,int(.95*len(a)))]if a else None,maximum=a[-1]if a else None)
def main():
 summary=json.loads((CAP/'summary.json').read_text());run=json.loads((BASE/'run_v2.json').read_text())
 assert summary['archive_closed']is True and summary['complete_without_writer_error']is True
 assert run['capture_returncode']==0
 path=CAP/'diagnostic.zip'
 errors=[]; pairs={};matrix_rows=[];raw_edges=collections.defaultdict(list)
 with zipfile.ZipFile(path)as z:
  names=z.namelist();assert len(names)==len(set(names)),'duplicate ZIP names'
  manifest=json.loads(z.read('entry_manifest.json'))
  for entry in manifest:
   b=z.read(entry['name']);assert len(b)==entry['bytes']and sha(b)==entry['sha256'],entry['name']
  statuses=[json.loads(z.read(n))for n in names if n.startswith('status/')and n.endswith('.json')]
  for row in statuses:
   assert sha(row['original_status_data'].encode())==row['original_status_utf8_sha256']
   row['status']=json.loads(row['original_status_data'])
  metadata=[json.loads(z.read(n))for n in names if n.startswith('metadata/')and n.endswith('.json')]
  metadata.sort(key=lambda d:d['sequence'])
  frames={'base':metadata[0]['parameters']['/unknown_perception'].get('base_frame','base_link'),
          'camera':metadata[0]['parameters']['/unknown_perception'].get('camera_frame',metadata[0]['parameters']['/camera'].get('frame_id','camera_link'))}
  tf_buffer=tf2_py.BufferCore(rospy.Duration(30))
  tf_records=[]
  for name in names:
   if not(name.startswith('tf/')and name.endswith('.json')):continue
   info=json.loads(z.read(name));raw=zlib.decompress(z.read(name[:-5]+'.rosmsg.zlib'))
   assert sha(raw)==info['serialized_sha256']and len(raw)==info['serialized_bytes']
   msg=TFMessage().deserialize(raw)
   assert msg._md5sum==info['md5sum']and msg._type==info['message_type']
   authority=info.get('original_connection_header',{}).get('callerid','recorded_unknown_authority')
   for t in msg.transforms:
    tf_records.append((t.header.stamp.to_nsec(),info['topic'],t,authority))
    if info['topic']=='/tf':raw_edges[(t.header.frame_id,t.child_frame_id)].append(t.header.stamp.to_nsec())
  for _,topic,t,authority in sorted(tf_records,key=lambda x:(x[1]!='/tf_static',x[0])):
   if topic=='/tf_static':tf_buffer.set_transform_static(t,authority)
   else:tf_buffer.set_transform(t,authority)
  for name in names:
   if not(name.startswith('pairs/')and name.endswith('.json')):continue
   info=json.loads(z.read(name));stamp=info['source_stamp_ns'];headers={}
   for stream in ('color','depth'):
    raw=zlib.decompress(z.read('pairs/%d_%s.rosmsg.zlib'%(stamp,stream)))
    proof=info['streams'][stream];assert sha(raw)==proof['serialized_sha256']and len(raw)==proof['serialized_bytes']
    msg=Image().deserialize(raw)
    assert msg._md5sum==proof['md5sum']and msg._type==proof['message_type']
    assert msg.header.stamp.to_nsec()==stamp==proof['source_stamp_ns']
    assert msg.header.seq==proof['source_seq']and msg.header.frame_id==proof['frame_id']
    headers[stream]=dict(seq=msg.header.seq,frame_id=msg.header.frame_id,width=msg.width,height=msg.height,encoding=msg.encoding,step=msg.step,pixel_bytes=len(msg.data),receipt_ros_ns=proof['receipt_ros_ns'])
   assert all(headers['color'][k]==headers['depth'][k]for k in ('width','height','frame_id'))
   row=dict(source_stamp_ns=stamp,headers=headers)
   try:
    t=tf_buffer.lookup_transform_core(frames['base'],frames['camera'],rospy.Time(stamp//10**9,stamp%10**9))
    row['exact_tf']=dict(source_frame=t.child_frame_id,target_frame=t.header.frame_id,lookup_stamp_ns=t.header.stamp.to_nsec(),
      translation=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z],
      quaternion_xyzw=[t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w])
    row['exact_tf']['requested_integer_source_stamp_ns']=stamp
    row['exact_tf']['returned_header_stamp_delta_ns']=t.header.stamp.to_nsec()-stamp
    assert t.header.frame_id==frames['base']and t.child_frame_id==frames['camera']
   except Exception as exc:
    row['tf_error']=repr(exc);errors.append(dict(source_stamp_ns=stamp,reason=repr(exc)))
   pairs[stamp]=row;matrix_rows.append(row)
  arrivals=json.loads(z.read('raw_image_header_arrivals.json'))
  by={stream:{r['source_stamp_ns']:r for r in arrivals if r['stream']==stream}for stream in ('color','depth')}
  transport={}
  for stream in ('color','depth'):
   rows=sorted(by[stream].values(),key=lambda r:r['arrival_monotonic'])
   proc={}
   for row in statuses:
    a=row['status'].get('frame_timing',{}).get('raw_arrivals',{}).get(stream)
    if a is not None:proc[a['source_stamp_ns']]=a
   matches=[a for stamp,a in proc.items()if stamp in by[stream]]
   transport[stream]=dict(independent_age_sec=stats([(r['receipt_ros_ns']-r['source_stamp_ns'])*1e-9 for r in rows]),
       independent_receive_gap_sec=stats([b['arrival_monotonic']-a['arrival_monotonic']for a,b in zip(rows,rows[1:])]),
       unknown_age_sec=stats([r['source_age_at_arrival_sec']for r in matches]),
       unknown_minus_independent_receive_sec=stats([(r['arrival_ros_ns']-by[stream][r['source_stamp_ns']]['receipt_ros_ns'])*1e-9 for r in matches]))
  g17=[row for row in statuses if row['status'].get('generation')==17]
  first=next(row['status']['first_loss']for row in g17 if row['status'].get('first_loss'))
  key_sources={first['last_good_source_stamp_ns'],first['source_stamp_ns'],first['anchor_source_stamp_ns']}
  key_sources.update(row['status']['source_stamp_ns']for row in g17 if row['status'].get('detail')=='support_measurement_pending_revalidation')
  pending=[]
  for row in g17:
   s=row['status'];stamp=s.get('source_stamp_ns')
   if stamp in key_sources and not any(p['source_stamp_ns']==stamp for p in pending):
    f=s.get('frame_timing',{})
    pending.append(dict(source_stamp_ns=stamp,detail=s['detail'],exact_pair_saved=stamp in pairs,tf_integer_source_lookup_succeeded=stamp in pairs and 'exact_tf'in pairs[stamp]and 'tf_error'not in pairs[stamp],
      processing_wall_sec=f.get('processing_wall_sec'),processing_thread_cpu_sec=f.get('processing_thread_cpu_sec'),source_age_at_status_sec=f.get('status_source_age_sec'),
      raw_arrivals=f.get('raw_arrivals'),stage_wall_sec=f.get('stage_wall_sec'),status_archive_name='status/%06d.json'%row['status_index']))
  param_variants={}
  for key in ('/unknown_perception','/handeye','/robot_description'):
   param_variants[key]=len({json.dumps(row['parameters'].get(key),sort_keys=True)for row in metadata})
  # The camera runtime profile contains live statistics; freeze only projection/scale here.
  camera_keys=('fx','fy','cx','cy','width','height','depth_scale','frame_id','align_depth_to_color','intrinsics_source')
  param_variants['camera_projection']=len({json.dumps({k:row['parameters']['/camera'].get(k)for k in camera_keys},sort_keys=True)for row in metadata})
  report=dict(schema=1,archive=str(path),archive_sha256=sha(path.read_bytes()),
    source_sha256=sha(Path(__file__).read_bytes()),entry_checks_passed=len(manifest),status_original_payload_checks_passed=len(statuses),
    image_pairs_verified=len(pairs),tf_integer_source_lookup_succeeded=len(pairs)-len(errors),tf_lookup_failures=errors,
    tf_returned_header_delta_ns_counts=dict(collections.Counter(str(r['exact_tf']['returned_header_stamp_delta_ns'])for r in matrix_rows if 'tf_error'not in r)),
    frames=frames,raw_tf_transform_count=len(tf_records),dynamic_edge_coverage=[dict(parent=k[0],child=k[1],first_stamp_ns=min(v),last_stamp_ns=max(v),samples=len(v))for k,v in raw_edges.items()],
    metadata_variant_counts=param_variants,source_pairs=matrix_rows,transport=transport,critical_source_records=pending,
    first_loss=first,first_loss_source_delta_sec=(first['source_stamp_ns']-first['last_good_source_stamp_ns'])*1e-9,
    first_loss_wall_age_since_good_sec=(first['observed_stamp_ns']-first['last_good_source_stamp_ns'])*1e-9,
    no_recorded_candidate_load=run['scope'],
    supersedes_diagnostic_assertion_only=str(BASE/'source_transport_validation.json'),
    header_stamp_note='The installed tf2 Python binding returns the requested transform with a header 0 or +1 ns from the original integer query. Original input stamps remain unchanged. The previous diagnostic incorrectly counted nonidentical returned headers as failed lookups despite successful transforms; both raw reports are retained. No time gate or pairing threshold changed.',
    limitations=['This is offline reconstruction from raw TF; interpolation is normal TF behavior, not nearest-source substitution.',
      'The captured TF chain and source SHA are evidence of software coordinates, not independent physical calibration accuracy.',
      'Images were not edited/decoded to arrays or fitted in this validation; plane fitting is a separate analysis.',
      'The recorder adds RGB-D transport and serialization/compression load; cannot call its timing overhead zero.',
      'The pre-existing gen15 intermittent RGB backlog remains unlocated; this experiment cannot prove its resolution.'])
  out=BASE/'source_transport_validation_v2.json'
  if out.exists():raise RuntimeError('preserve existing validation')
  out.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
  print(json.dumps({k:report[k]for k in ('image_pairs_verified','tf_integer_source_lookup_succeeded','tf_lookup_failures','tf_returned_header_delta_ns_counts','metadata_variant_counts','transport','critical_source_records')},ensure_ascii=False))
if __name__=='__main__':main()
