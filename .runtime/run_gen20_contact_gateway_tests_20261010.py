from pathlib import Path
import json,hashlib,subprocess,os,sys,xml.etree.ElementTree as E
out=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/verification/gen20_contact_pregrasp_gateway')
out.mkdir(exist_ok=True,parents=True)
name=sys.argv[1];files=sys.argv[2:]
sources=['src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py','src/alicia_flexible_grasp_supervisor/scripts/motion_gateway_node.py','src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_task.py']+['src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/'+f for f in ['contact_pregrasp_gateway.py','held_pregrasp_selection.py','contact_sequence_precheck.py','moveit_planner.py']]
def sha(f):return hashlib.sha256(Path(f).read_bytes()).hexdigest()
before={f:sha(f) for f in sources+files}
cmd=['/usr/bin/python3','-m','pytest','-q','-p','no:cacheprovider','--basetemp='+str(out/(name+'_tmp')),'--junitxml='+str(out/(name+'.xml'))]+files
manifest=dict(command=cmd,source_sha256=before,pythonpath=os.environ.get('PYTHONPATH'),hardware_access=False)
(out/(name+'_manifest.json')).write_text(json.dumps(manifest,indent=2))
with (out/(name+'.log')).open('w') as log:r=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT)
print('exit',r.returncode)
if (out/(name+'.xml')).exists():
 root=E.parse(str(out/(name+'.xml'))).getroot()
 print([(v.tag,v.attrib) for v in root])
 for x in root.iter('testcase'):
  for t in ('failure','error'):
   v=x.find(t)
   if v is not None:print(x.attrib.get('name'),t,(v.text or '')[-1800:])
after={f:sha(f) for f in before}
manifest.update(source_after_sha256=after,source_unchanged=before==after,exit_code=r.returncode)
(out/(name+'_manifest.json')).write_text(json.dumps(manifest,indent=2))
print('changed_sources',[f for f,h in before.items() if after[f]!=h])
