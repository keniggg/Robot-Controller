"""Compare sealed GEN13 plane metrics against original and requested policies."""
from pathlib import Path
import hashlib, importlib.util, json, os, sys, tarfile
import numpy as np

W=Path(__file__).resolve().parents[1]
M=Path('/media/zhuyupei/SanStand')
assert M.is_mount()
N=M/'实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z'
O=N/'verification/support_offset_warning_4mm_hard_7mm';O.mkdir(exist_ok=True)
path='src/alicia_grasp_modes/src/alicia_grasp_modes/tabletop.py'
old=W/'.runtime/support_offset_policy_20261010/baseline_tabletop.py'
with tarfile.open(str(W/'.runtime/support_offset_policy_20261010/source_before.tar')) as archive:
    old.write_bytes(archive.extractfile(path).read())
def module(name,p):
    spec=importlib.util.spec_from_file_location(name,str(p));m=importlib.util.module_from_spec(spec)
    sys.modules[name]=m;spec.loader.exec_module(m);return m
before=module('support_before',old)
after=module('support_after',W/path)
p=N/'gen13_handoff_analysis/support_plane_replay/report.json';r=json.loads(p.read_text())
ref=r['reference'];rows=[]
for f in r['frames']:
    row={'source_stamp_ns':f['source_stamp_ns'],'raw_plane_base':f['plane_base']}
    for label,m in [('before',before),('after',after)]:
        try:
            _,metrics=m._validate_support_plane_update(ref['plane_base'],f['plane_base'],
                ref['anchor_base'],m.Config(),ref['footprint_base'])
            row[label]={'accepted':True,'metrics':metrics}
        except ValueError as e:row[label]={'accepted':False,'reason':str(e)}
    rows.append(row)
assert all(x['after']['accepted'] for x in rows)
assert [x['before']['accepted'] for x in rows]==[True,True,False,False,False]
assert [x['after']['metrics']['support_reference_distance_warning'] for x in rows]==[False,False,True,True,True]
hard_rejection=False
plane=np.array(ref['plane_base']);plane[3]+=.0071
try:after._validate_support_plane_update(ref['plane_base'],plane,ref['anchor_base'],after.Config(),ref['footprint_base'])
except ValueError as e:
    hard_rejection='support_plane_local_distance_inconsistent' in str(e)
assert hard_rejection
old_config=before.Config().__dict__;new_config=after.Config().__dict__
assert {k:v for k,v in new_config.items() if k!='support_local_shift_warning_m' and k!='support_max_local_shift_m'}=={k:v for k,v in old_config.items() if k!='support_max_local_shift_m'}
out={'passed':True,'scope':'plane continuity only; not target segmentation, registration, path or grasp success',
     'source_report':str(p),'source_report_sha256':hashlib.sha256(p.read_bytes()).hexdigest(),
     'old_source_sha256':hashlib.sha256(old.read_bytes()).hexdigest(),'new_source_sha256':hashlib.sha256((W/path).read_bytes()).hexdigest(),
     'rows':rows,'synthetic_7p1mm_rejected':hard_rejection,'other_config_fields_unchanged':True,
     'device_commands':False,'ros_access':False}
p=O/'gen13_recorded_plane_replay.json';p.write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({'report':str(p),'passed':True,'old_accepted':[x['before']['accepted'] for x in rows],
                  'new_warning':[x['after']['metrics']['support_reference_distance_warning'] for x in rows]},ensure_ascii=False))
