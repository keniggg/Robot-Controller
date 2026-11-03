#!/usr/bin/env python3
"""Prepare versioned operational helpers only; never import/execute them."""
import ast,difflib,hashlib,json,time,xml.etree.ElementTree as ET
from pathlib import Path
W=Path(__file__).resolve().parents[1];E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller');N=E/'ros_log/full_latest_gen41_restart_20261018T011222Z';V=N/'verification';OUT=V/'gen48_idle_helper_preparation_r1'
BASE='69e63cc71c4c63f1bbeaa2693dd88829363f06c878117f5f6129f38eaaf43e5a'
PREVIOUS=N/'gen47_runtime/gen47_runtime_deployment.json';PREVIOUS_SHA='6e41bb2528b44adbbe6409fcae6c1a8f61099ffc0df78b454b37d9ac16b2a637'
names=['gen47_idle_reload_common_20261018.py','read_gen47_deployment_preflight_20261018.py','load_gen47_idle_compute_gateway_20261018.py','gen47_observation_measurement_consumers_20261018.launch','start_gen47_observation_measurement_consumers_20261018.sh','finalize_gen47_idle_deployment_20261018.py']
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def replace_verified(s,old,new):
 assert old in s,old
 return s.replace(old,new)
def main():
 assert E.parent.parent.is_mount()and not OUT.exists();assert sha(V/'gen47_release_manifest.json')==BASE and sha(PREVIOUS)==PREVIOUS_SHA
 OUT.mkdir();old_hashes={name:sha(W/'.runtime'/name)for name in names};entries=[];diffs=[]
 for oldname in names:
  original=(W/'.runtime'/oldname).read_text();newname=oldname.replace('gen47','gen48');dest=W/'.runtime'/newname;assert not dest.exists()
  text=original.replace('gen47','gen48').replace('GEN47','GEN48').replace('gen46','gen47').replace('GEN46','GEN47')
  if oldname.startswith('gen47_idle_reload_common'):
   text=replace_verified(text,"BASE_SHA = '97eda3daf5a958245911e59d73a45b141089b5839f2c9cccc908ec6da3d152c2'","BASE_SHA = '"+BASE+"'")
   text=replace_verified(text,"PREVIOUS_SHA = '2dd851d9eec6dda6c6982a83ce63900350df812c20ae24519c846d2a652b5b0a'","PREVIOUS_SHA = '"+PREVIOUS_SHA+"'")
   text=replace_verified(text,"release['source_count'] > 1050","release['source_count'] > 1052")
   text=replace_verified(text,"full['summary']['tests'] > 8545 and full['test_scope_files'] == 306","full['summary']['tests'] > 8646 and full['test_scope_files'] == 306 + release['new_test_files']")
   text=replace_verified(text,"release.get('historical_test_files') == 301 and release.get('historical_tests') == 8545","release.get('historical_test_files') == 306 and release.get('historical_tests') == 8646")
   text=replace_verified(text,"release.get('supplemental_existing_test_files') == 4","release.get('supplemental_existing_test_files') == 0")
   text=replace_verified(text,"summary['tests'] == 8545 + release['supplemental_existing_test_cases']","summary['tests'] == 8646 + release['supplemental_existing_test_cases']")
   text=replace_verified(text,"value['revision'] == 50","value['revision'] == 52")
   text=replace_verified(text,'expected unchanged active control revision 50','expected unchanged active control revision 52')
   text=replace_verified(text,'GEN48 new diagnostic and execution tests are missing','GEN48 complete visibility-family test scope is missing')
  if oldname.startswith('load_gen47'):
   text=replace_verified(text,'load reviewed GEN48 top-mask contract while manual/idle','load reviewed GEN48 cuboid visibility-family enclosure while manual/idle')
  if oldname.startswith('finalize_gen47'):
   text=replace_verified(text,"release['source_count'] > 1050","release['source_count'] > 1052")
   tree=ast.parse(text);node=next(n for n in tree.body if isinstance(n,ast.Assign)and any(isinstance(t,ast.Name)and t.id=='HELPER_SHA'for t in n.targets))
   helpers={str(W/'.runtime'/name.replace('gen47','gen48')):sha(W/'.runtime'/name.replace('gen47','gen48'))for name in names[:5]}
   lines=text.splitlines(True);lines[node.lineno-1:node.end_lineno]=['HELPER_SHA = '+repr(helpers)+'\n'];text=''.join(lines)
  if dest.suffix=='.py':ast.parse(text)
  if dest.suffix=='.launch':ET.fromstring(text)
  dest.write_text(text);assert dest.read_text()==text
  if dest.suffix=='.sh':dest.chmod((W/'.runtime'/oldname).stat().st_mode&0o777)
  entries.append(dict(old_path=str(W/'.runtime'/oldname),old_sha256=old_hashes[oldname],new_path=str(dest),new_sha256=sha(dest)))
  diffs.extend(difflib.unified_diff(original.splitlines(True),text.splitlines(True),fromfile=oldname,tofile=newname))
 assert all(sha(W/'.runtime'/name)==h for name,h in old_hashes.items())
 (OUT/'changes.diff').write_text(''.join(diffs))
 common=ast.parse((W/'.runtime/gen48_idle_reload_common_20261018.py').read_text());literal={}
 for n in common.body:
  if isinstance(n,ast.Assign)and len(n.targets)==1and isinstance(n.targets[0],ast.Name):
   try:literal[n.targets[0].id]=ast.literal_eval(n.value)
   except(ValueError,TypeError):pass
 assert literal['BASE_SHA']==BASE and literal['PREVIOUS_SHA']==PREVIOUS_SHA
 assert literal['COMPUTE']==('/grasp_task_node','/remote_grasp6d_node','/unknown_tabletop_perception')and len(literal['PRESERVE'])==7
 launch=ET.parse(W/'.runtime/gen48_observation_measurement_consumers_20261018.launch').getroot();gateway=[n for n in launch.findall('node')if n.get('name')=='motion_gateway'];assert len(gateway)==1and gateway[0].get('if')=='$(arg include_gateway)'
 assert any(n.get('name')=='include_gateway'and n.get('default')=='false'for n in launch.findall('arg'))
 script=(W/'.runtime/start_gen48_observation_measurement_consumers_20261018.sh').read_text();assert 'include_gateway:=false'in script
 control=json.loads((E/'ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/control.json').read_text());assert control['state']=='active'and control['revision']==52
 r=dict(kind='gen48_idle_helper_source_preparation',prepared=True,helpers=entries,helper_count=6,old_helpers_unchanged=True,baseline_release_sha256=BASE,previous_deployment=str(PREVIOUS),previous_deployment_sha256=PREVIOUS_SHA,expected_control=dict(state='active',revision=52),historical_scope=dict(files=306,tests=8646),new_scope_semantics='One new file; case count and complete source identities must come from the sealed actual manifest. Anticipated1053/307/8662 is not validation.',preserved_safety=dict(replaced_compute=3,replaced_gateway=1,preserved_hardware_and_GUI=7,gateway_only_original_parent_respawn=True,compute_launch_gateway_disabled=True,manual_idle_required=True,SDK_fresh_samples_minimum=20,task_and_actions_inactive_required=True,selection_epoch_parameters_control_source_and_evidence_pinned=True,reset_ACK_canonical_parse_selection=True,temperature_queries=False,disable_commands=False,motion_commands=False),AST_and_XML_parse_passed=True,bash_syntax_check_pending=True,helpers_executed=False,ROS_imported=False,source_or_test_installed=False,release_sealer_prepared=False,sealer_status='Parent may prepare specific GEN48 focus/exact-qualification/math/integration evidence sealer; GEN47 paired-identity acceptance must not carry forward as a new GEN48 proof.',diff_path=str(OUT/'changes.diff'),diff_sha256=sha(OUT/'changes.diff'),preparation_helper_sha256=sha(__file__),physical_grasp_verified=False,timestamp_ns=time.time_ns())
 raw=(json.dumps(r,ensure_ascii=False,indent=2)+'\n').encode();(OUT/'result.json').write_bytes(raw);assert(OUT/'result.json').read_bytes()==raw
 print(json.dumps(dict(result=str(OUT/'result.json'),result_sha256=sha(OUT/'result.json'),helpers=[{'path':x['new_path'],'sha256':x['new_sha256']}for x in entries]),ensure_ascii=False))
if __name__=='__main__':main()
