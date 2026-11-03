"""Exact message/clock comparison; no callback-delivery inference or ROS I/O."""
import hashlib
import importlib.util
import json
from pathlib import Path
import time
W=Path(__file__).resolve().parents[1]
V=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
I=V/'gen45_station_owner_messages_r1'
O=V/'gen45_station_owner_timeline_r1'

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def ns(stamp):return stamp['secs']*1000000000+stamp['nsecs']
def save(path,value):
 path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
 assert json.loads(path.read_text())==value

def main():
 assert not O.exists();r=json.loads((I/'result.json').read_text());assert r['passed']
 def rows(topic):
  info=r['files'][topic];assert sha(info['path'])==info['sha256']
  return json.loads(Path(info['path']).read_text())['messages']
 def ref(topic,index):
  row=rows(topic)[index]
  return dict(topic=topic,index=index,file=r['files'][topic]['path'],file_sha256=r['files'][topic]['sha256'],
   received_ns=row['received_ns'],wire_sha256=row['wire_sha256'])
 status=rows('/perception/unknown/detector_status');geometry=rows('/grasp_6d/object_geometry')
 pending=[];hard=[]
 for i,row in enumerate(geometry):
  p=row['payload'];reason=p['failure_reason'];code=reason.partition(':')[0]
  if row['received_ns']<1792306910000000000:continue
  if code=='TARGET_PROCESSING_PENDING':
   value=json.loads(reason.partition(':')[2]);pending.append((i,row,value))
  if code=='TARGET_LOST':hard.append((i,row))
 assert len(pending)==3 and hard
 oi,orow,opending=pending[-1];old=opending['station_compute_wait']
 hi,hrow=hard[0];first_hard_ns=ns(hrow['payload']['header']['stamp'])
 ready=[(i,row)for i,row in enumerate(status)if row['received_ns']<first_hard_ns and(row['parsed_json']or{}).get('state')=='ready'and isinstance((row['parsed_json']or{}).get('station_compute_wait'),dict)]
 assert ready
 li,lrow=ready[-1];latest=lrow['parsed_json'];new=latest['station_compute_wait']
 assert old['source_ns']==1792306915498465299 and old['identity_deadline_ns']==1792306920498465299
 assert first_hard_ns==1792306920504162788
 assert new['source_ns']==1792306920164707660 and new['identity_deadline_ns']==1792306925164707660
 assert old['binding']==new['binding']
 assert old in latest['station_compute_recovery_chain']
 modulepath=W/'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_reacquisition.py'
 release=V/'gen45_release_manifest.json';release_data=json.loads(release.read_text())
 assert sha(modulepath)==release_data['source_sha256'][str(modulepath.relative_to(W))]
 spec=importlib.util.spec_from_file_location('_gen45_exact_contract_clock',modulepath)
 module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
 checks=[]
 def check(label,fn,expected):
  try:actual=fn()
  except ValueError as exc:actual='ValueError:'+str(exc)
  assert actual==expected,(label,actual,expected);checks.append(dict(label=label,result=actual))
 check('old_registered_parent_at_first_hard_invalidation',lambda:module.validate_contract(old,binding=old['binding'],now_ns=first_hard_ns),'ValueError:station_compute_wait_expired_or_future')
 check('latest_public_READY_at_first_hard_invalidation',lambda:module.validate_contract(new,binding=new['binding'],now_ns=first_hard_ns),'ARMED')
 check('latest_public_READY_recovery_ancestry_at_original_receive',lambda:module.validate_recovery_parent(old,parent=latest['station_compute_recovery'],chain=latest['station_compute_recovery_chain'],binding=old['binding'],now_ns=lrow['received_ns'],source_ns=new['source_ns']),None)
 station_rows=[]
 for i,row in ready:
  j=row['parsed_json'];c=j['station_compute_wait'];b=c['binding']
  assert b==old['binding']and j['target_lost']is False and j.get('first_loss')is None
  station_rows.append(dict(**ref('/perception/unknown/detector_status',i),source_ns=c['source_ns'],geometry_deadline_ns=c['geometry_deadline_ns'],identity_deadline_ns=c['identity_deadline_ns'],recovery_parent_ns=(j.get('station_compute_recovery')or{}).get('source_ns'),recovery_chain_count=len(j.get('station_compute_recovery_chain')or[])))
 manual=[dict(**ref('/gui/joint_direct_mode',i),manual=row['payload']['data'])for i,row in enumerate(rows('/gui/joint_direct_mode'))]
 phases=[]
 for i,row in enumerate(rows('/grasp/near_field_phase')):
  p=row['payload'];phases.append(dict(**ref('/grasp/near_field_phase',i),source_ns=ns(p['header']['stamp']),active=p['active'],phase_id=p['phase_id'],reference_target_track_id=p['reference_target_track_id']))
 afterold=[row for row in station_rows if row['source_ns']>old['source_ns']]
 O.mkdir();save(O/'station_READY_timeline.json',station_rows)
 save(O/'exact_old_and_latest_contracts.json',dict(old_pending=opending,latest_READY=latest))
 result=dict(kind='GEN45_station_owner_exact_message_clock_review',passed=True,created_ns=time.time_ns(),
  extraction_receipt=str(I/'result.json'),extraction_sha256=sha(I/'result.json'),release_sha256=sha(release),
  pure_clock_source=dict(path=str(modulepath),sha256=sha(modulepath)),clock_checks=checks,
  first_hard_geometry=dict(**ref('/grasp_6d/object_geometry',hi),source_ns=first_hard_ns,valid=hrow['payload']['valid'],target_track_id=hrow['payload']['target_track_id'],failure_reason=hrow['payload']['failure_reason']),
  last_pending_geometry=dict(**ref('/grasp_6d/object_geometry',oi),source_ns=ns(orow['payload']['header']['stamp']),parent_source_ns=old['source_ns'],identity_deadline_ns=old['identity_deadline_ns']),
  latest_READY=dict(**ref('/perception/unknown/detector_status',li),source_ns=new['source_ns'],geometry_deadline_ns=new['geometry_deadline_ns'],identity_deadline_ns=new['identity_deadline_ns'],target_lost=latest['target_lost'],first_loss=latest.get('first_loss')),
  at_first_hard_geometry=dict(old_parent_expired_ns=first_hard_ns-old['identity_deadline_ns'],latest_geometry_remaining_ns=new['geometry_deadline_ns']-first_hard_ns,latest_identity_remaining_ns=new['identity_deadline_ns']-first_hard_ns),
  same_binding_exact=True,new_READY_original_ancestry_contains_exact_old_parent=True,
  latest_READY_delivered_before_old_expiry_ns=old['identity_deadline_ns']-lrow['received_ns'],
  newer_READY_deliveries_after_old_parent=len(afterold),manual_messages=manual,phase_messages=phases,
  message_based_finding='Old private-pending parent expired, while newer same-owner public READY measurement and its unmodified ancestry were still valid under the original pure validators. No published manual/session/lease/phase change precedes the first hard geometry invalidation.',
  causal_limit='Bag receipt proves transport availability only. It does not prove which subscriber callback ran, what private-work ledger contained, or that the newer full object/mask/status join was accepted. Remote source/call-path review is still required to identify the first incorrect state transition.',
  suggested_replay='Use exact old parent, every original READY successor/ancestry and original receive order; hold request15 private work across its original parent deadline. Verify old work loses publication authority without globally revoking a separately registered live station; real owner/target/control conflicts must remain hard failures.',
  production_or_tests_modified=False,ROS_initialized=False,hardware_contacted=False,temperature_read=False,helper_sha256=sha(__file__))
 save(O/'result.json',result);(O/'helper_source.py').write_bytes(Path(__file__).read_bytes())
 print(json.dumps(dict(result=str(O/'result.json'),sha256=sha(O/'result.json'),clock_checks=checks,newer_READY_deliveries_after_old_parent=len(afterold),same_binding=True),ensure_ascii=False))
if __name__=='__main__':main()
