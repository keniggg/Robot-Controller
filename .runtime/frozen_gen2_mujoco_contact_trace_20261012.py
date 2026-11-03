"""Independent offline replay of two recorded simulation requests. No HTTP/ROS."""
from pathlib import Path
import sys,os,json,hashlib,time,dataclasses
os.environ['MUJOCO_GL']='disable';os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1';sys.dont_write_bytecode=True
WORK=Path(__file__).resolve().parents[1];sys.path[:0]=[str(WORK/'.runtime/mujoco-3.2.3'),str(WORK/'tools')]
import numpy as np
import mujoco_digital_twin_server as engine
import mujoco
BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log')
SERVICE=BASE/'candidate_simulation_full_start_20261012T021050Z_jc7jctxv/service';OUT=BASE/'full_latest_frozen_no_skin_20261012T021050Z/gen2_analysis/mujoco_contact_trace';OUT.mkdir(parents=True,exist_ok=True)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
health=json.loads((SERVICE/'startup_verification.json').read_text())['health']
assert sha(engine.__file__)==health['source_sha256'];assert sha(SERVICE/'loaded_candidate.xml')==health['model_xml_sha256'];assert mujoco.__version__==health['mujoco']
class Trace(engine.MujocoDigitalTwinBackend):
 def __init__(self):
  super().__init__(model_xml=SERVICE/'loaded_candidate.xml',max_snapshot_age_sec=120.,ros_sync_joint_states=False);self.trace=[];self.phase='setup'
 def _simulate_close_contact(self,*a,**kw):
  self.phase='closure_scan';return super()._simulate_close_contact(*a,**kw)
 def _simulate_lift(self,*a,**kw):
  self.phase='preload_or_lift';return super()._simulate_lift(*a,**kw)
 def _classify_current_contacts(self,model,data,meta):
  result=super()._classify_current_contacts(model,data,meta)
  self.trace.append(dict(index=len(self.trace),phase=self.phase,time=float(data.time),classification=dataclasses.asdict(result),gripper=self._gripper_dynamic_state(model,data,meta),object_position_base_m=np.asarray(data.xpos[meta['object_body']]).tolist(),object_quaternion_wxyz=np.asarray(data.xquat[meta['object_body']]).tolist(),arm_qpos_rad=[float(data.qpos[model.jnt_qposadr[j]])for j in meta['arm_joints']]))
  return result
reports=[]
for directory in sorted((SERVICE/'requests').iterdir()):
 payload=json.loads((directory/'request.raw.json').read_text());recorded=json.loads((directory/'response.json').read_text())
 backend=Trace();backend._import_mujoco()
 # Validate immutable protocol against its ORIGINAL request age; no live authority.
 checked=engine._validate_v3_payload(payload,now_sec=float(payload['snapshot_stamp_sec'])+1.,max_snapshot_age_sec=120.)
 before=time.monotonic();response=backend._simulate_validated_grasp(checked,payload['plan_id'])
 compare={k:dict(recorded=recorded.get(k),replayed=response.get(k))for k in recorded if k not in ('ok','used_joint_state_source') and recorded.get(k)!=response.get(k)}
 steps=backend.trace;both=[s['index']for s in steps if s['classification']['two_sided']];left=[s['index']for s in steps if s['classification']['left_contact']];right=[s['index']for s in steps if s['classification']['right_contact']]
 record=dict(scope='independent_offline_same_server_same_request_same_model_simulation',plan_id=payload['plan_id'],source_request=str(directory/'request.raw.json'),source_request_sha256=sha(directory/'request.raw.json'),source_response_sha256=sha(directory/'response.json'),engine_sha256=sha(engine.__file__),model_sha256=sha(SERVICE/'loaded_candidate.xml'),mujoco_version=mujoco.__version__,motion_authorized=False,ros_called=False,http_called=False,production_changed=False,physics_changed=False,protocol_time_validation='recorded request timestamp plus one second; historical simulation replay only, not live freshness authorization',elapsed_sec=time.monotonic()-before,recorded_response=recorded,replayed_response=response,response_differences=compare,steps=steps,summary=dict(step_count=len(steps),two_sided_indices=both,left_indices=left,right_indices=right,first=steps[0],last=steps[-1]))
 out=OUT/(payload['plan_id']+'.json');out.write_text(json.dumps(record,indent=2,ensure_ascii=False)+'\n')
 reports.append(dict(path=str(out),sha256=sha(out),plan_id=payload['plan_id'],steps=len(steps),same_response=not compare,failure=response.get('failure_reason'),two_sided_indices=both,first_object=steps[0]['object_position_base_m'],last_object=steps[-1]['object_position_base_m'],last_gripper=steps[-1]['gripper']))
print(json.dumps(reports,indent=2,ensure_ascii=False))
