import sys,json,time,hashlib
from pathlib import Path
from dataclasses import fields,asdict
import numpy as np,cv2
W=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(W/'.runtime/gen14_arrival_repair_20261013/src/alicia_grasp_modes/src'))
from alicia_grasp_modes.observation_measured_face import *
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask
F=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z/verification/gen14_arrival_independent')
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=False)
cv2.setNumThreads(1)
f=json.loads((F/'fixture.json').read_text());cam=f['camera'];config=Config(**{x.name:f['unknown_parameters'][x.name] for x in fields(Config) if x.name in f['unknown_parameters']});k=[cam[z]for z in ('fx','fy','cx','cy')];plane=np.array(json.loads((F/'original_geometry_reconstruction_v2.json').read_text())['seed']['reference_plane_base']);ns=1791860176945733070;pre=np.load(F/f'{ns}_source.npz');valid=rectification_rgb_valid_mask(cam,pre['depth'].shape)
sha=lambda a:hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()
start=time.monotonic();source=FrozenMeasuredFaceSource(pre['raw_depth'],'16UC1',cam['depth_scale'],k,pre['T_base_camera'],plane,config,ns,'diagnostic-original-pixels',sha(pre['rgb']),sha(pre['recorded_mask']));frozen=freeze_measured_face(source,pre['rgb'],pre['recorded_mask'],valid,check_current=lambda:None,deadline_monotonic=time.monotonic()+2);freeze_elapsed=time.monotonic()-start
rows=[]
for row in f['frames']:
 ns=row['source_stamp_ns']
 if ns<1791860212000000000:continue
 cur=np.load(row['file']);mask=cv2.imread(str(F/f'{ns}_rejected_proposal_0_mask.png'),0)
 start=time.monotonic()
 try:
  evidence,audit=corroborate_measured_face(frozen,cur['rgb'],mask,valid,dict(depth=cur['depth'],intrinsics=k,transform=cur['T_base_camera'],plane=plane,config=config),source_ns=ns,deadline_monotonic=time.monotonic()+2,check=lambda:None)
  item=dict(source_ns=ns,evidence=asdict(evidence),audit=audit)
 except Exception as exc:item=dict(source_ns=ns,error=str(exc))
 item['elapsed_sec']=time.monotonic()-start;rows.append(item);print(ns,item.get('error')or item['evidence']['code'],item['elapsed_sec'],flush=True)
prod=W/'.runtime/gen14_arrival_repair_20261013/src/alicia_grasp_modes/src/alicia_grasp_modes'
report=dict(scope='Pure actual RGBD fixed source-face diagnostic, not source-clock qualification or geometry authority',freeze_elapsed_sec=freeze_elapsed,reference=json.loads(frozen.audit_json),rows=rows,source_sha256={p.name:sha(np.frombuffer(p.read_bytes(),np.uint8)) for p in [prod/'observation_measured_face.py',prod/'arrival_dense_identity.py']})
(out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
