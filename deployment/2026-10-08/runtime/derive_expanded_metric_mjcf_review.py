#!/usr/bin/env python3
"""Read-only offline MJCF derivation audit. No ROS node/network/control/deployment.

Outputs a four-body rotation-only artifact, deliberately NOT an equivalence
certificate for the current URDF. Measures the pre-existing TCP/model mismatch.
"""
import argparse, copy, hashlib, itertools, json, os, sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
if not hasattr(np, "typeDict"): np.typeDict = np.sctypeDict
from scipy.spatial.transform import Rotation
import yaml

ROOT=Path(__file__).resolve().parents[1]
os.environ['MUJOCO_GL']='disable'
sys.path[:0]=[str(ROOT/'.runtime/mujoco-3.2.3'), str(ROOT/'tools')]
import mujoco
import mujoco_digital_twin_server as S
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
NAMES=tuple('Joint%d'%i for i in range(1,7))
MOUNT=Path('/media/zhuyupei/SanStand')
SOURCE=ROOT/'src/arm-mujoco/synriard/mjcf/Alicia_D_v5_6/Alicia_D_v5_6_gripper_50mm.xml'
SOURCE_SHA='b349fae85b95e507a5441588f44bbeeb0af37e360a351ed66d3c28b40c0e1026'

def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def rotmat(r): return r.as_matrix() if hasattr(r,'as_matrix') else r.as_dcm()
def frommat(m): return Rotation.from_matrix(m) if hasattr(Rotation,'from_matrix') else Rotation.from_dcm(m)
def transform(pos,mat):
 t=np.eye(4);t[:3,3]=pos;t[:3,:3]=mat;return t

def handeye(p):
 t=yaml.safe_load(p.read_text())['transformation']
 return transform([t[k] for k in ('x','y','z')],rotmat(Rotation.from_quat([t[k] for k in ('qx','qy','qz','qw')])))

def compile_xml(root):
 r=copy.deepcopy(root)
 r.find('compiler').set('meshdir',str((SOURCE.parent/r.find('compiler').get('meshdir')).resolve()))
 return mujoco.MjModel.from_xml_string(ET.tostring(r,encoding='unicode'))

def fk(model,data,q,gap=.05):
 for name,v in zip(NAMES,q):
  j=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,name);data.qpos[model.jnt_qposadr[j]]=v
 for name,v in zip(('left_finger','right_finger'),S._finger_qpos_for_inner_gap(gap)):
  j=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,name);data.qpos[model.jnt_qposadr[j]]=v
 mujoco.mj_kinematics(model,data)
 ids=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,n) for n in ('Link6','Link7','Link8')]
 palm=transform(data.xpos[ids[0]].copy(),data.xmat[ids[0]].reshape(3,3).copy())
 tcp=transform((data.xpos[ids[1]]+data.xpos[ids[2]])/2,palm[:3,:3])
 return tcp,palm

def err(a,b):
 return dict(translation_m=float(np.linalg.norm(a[:3,3]-b[:3,3])),rotation_rad=float(np.linalg.norm(frommat(a[:3,:3].T@b[:3,:3]).as_rotvec())))
def summarize(rows):
 return {k:dict(maximum=max(r[k] for r in rows),median=float(np.median([r[k] for r in rows]))) for k in rows[0]}

def derive(source, changes):
 root=ET.fromstring(source);records=[]
 for change in changes:
  name=change['joint'];matches=[b for b in root.findall('.//body') if b.find("joint[@name='%s']"%name) is not None]
  if len(matches)!=1: raise ValueError('unique explicit body joint required')
  b=matches[0];j=b.find("joint[@name='%s']"%name)
  if any(k in b.attrib for k in ('euler','axisangle','xyaxes','zaxis')) or j.get('ref','0')!='0':raise ValueError('unsupported rotation/reference')
  old=np.fromstring(b.get('quat','1 0 0 0'),sep=' ');axis=np.fromstring(j.get('axis'),sep=' ');axis/=np.linalg.norm(axis)
  oldR=rotmat(Rotation.from_quat(old[[1,2,3,0]]));delta=change['offset_rad']
  newq=frommat(oldR@rotmat(Rotation.from_rotvec(axis*delta))).as_quat()[[3,0,1,2]]
  b.set('quat',' '.join(format(float(x),'.17g') for x in newq))
  records.append(dict(joint=name,body=b.get('name'),old_quat_wxyz=old.tolist(),new_quat_wxyz=newq.tolist(),local_axis=axis.tolist(),delta_rad=delta))
 return root,records

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--candidate',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
 if not os.path.ismount(str(MOUNT)) or MOUNT not in a.output.resolve().parents:raise ValueError('external mounted output required; no fallback')
 if a.output.exists():raise ValueError('refuse overwrite experiment')
 artifact=ROOT/'.runtime/expanded_metric_mjcf_review'
 if artifact.exists():raise ValueError('refuse overwrite artifact')
 manifest=json.loads((a.candidate/'manifest.json').read_text());oldU=a.candidate/'rollback_source/original_robot_description.urdf';newU=a.candidate/'alicia_expanded_metric_candidate.urdf';Xp=a.candidate/'d405_expanded_metric_candidate_eye_on_hand.yaml'
 assert sha(oldU)==manifest['original_urdf_sha256'] and sha(newU)==manifest['candidate_model_sha256'] and sha(Xp)==manifest['candidate_handeye_sha256'] and sha(SOURCE)==SOURCE_SHA
 archive=MOUNT/'实验记录/Robot-Controller/ros_log/unknown_grasp_20260923/wsl_transfer/alicia-mujoco-20260923/src/arm-mujoco/synriard/mjcf/Alicia_D_v5_6/Alicia_D_v5_6_gripper_50mm.xml'
 assert sha(archive)==SOURCE_SHA
 oldfk=SerialUrdfFk(oldU.read_text(),NAMES);newfk=SerialUrdfFk(newU.read_text(),NAMES);X=handeye(Xp)
 source=SOURCE.read_text();original=ET.fromstring(source);derived,changes=derive(source,manifest['origin_changes'])
 # Structural proof: only the four body quaternions change, no physics/geometry/joint edit.
 reverse=copy.deepcopy(derived)
 for change in changes:
  b=reverse.find(".//body[@name='%s']"%change['body']);b.set('quat',original.find(".//body[@name='%s']"%change['body']).get('quat'))
 assert ET.tostring(reverse)==ET.tostring(original)
 m0=compile_xml(original);m1=compile_xml(derived);d0=mujoco.MjData(m0);d1=mujoco.MjData(m1)
 zero=np.zeros(6);tcp0,palm0=fk(m0,d0,zero);C=np.linalg.inv(tcp0)@oldfk(zero);H=np.linalg.inv(palm0)@oldfk(zero)
 lo,hi=newfk.limits.T;middle=(lo+hi)/2;rng=np.random.default_rng(20260929)
 samples=[*rng.uniform(lo,hi,size=(1024,6)),*np.asarray(list(itertools.product(*zip(lo,hi)))),middle,np.clip(zero,lo,hi)]
 for i in range(6):
  for v in (lo[i],hi[i],np.nextafter(lo[i],hi[i]),np.nextafter(hi[i],lo[i])):
   q=middle.copy();q[i]=v;samples.append(q)
 delta=np.zeros(6)
 for c in manifest['origin_changes']:delta[NAMES.index(c['joint'])]=c['offset_rad']
 rows={k:[] for k in ('mjcf_offset_algebra','candidate_raw_server_tcp_vs_urdf','candidate_zero_derived_ros_tool_vs_urdf','candidate_zero_derived_camera_vs_urdf','baseline_raw_server_tcp_vs_urdf')}
 lower=np.array([m0.jnt_range[mujoco.mj_name2id(m0,mujoco.mjtObj.mjOBJ_JOINT,n)][0] for n in NAMES]);upper=np.array([m0.jnt_range[mujoco.mj_name2id(m0,mujoco.mjtObj.mjOBJ_JOINT,n)][1] for n in NAMES])
 mjcf_outside=0
 for q in samples:
  actual,palm=fk(m1,d1,q);reference,_=fk(m0,d0,q+delta);expected=newfk(q);baseline,_=fk(m0,d0,q)
  rows['mjcf_offset_algebra'].append(err(actual,reference))
  rows['candidate_raw_server_tcp_vs_urdf'].append(err(actual,expected))
  rows['candidate_zero_derived_ros_tool_vs_urdf'].append(err(palm@H,expected))
  rows['candidate_zero_derived_camera_vs_urdf'].append(err(palm@H@X,expected@X))
  rows['baseline_raw_server_tcp_vs_urdf'].append(err(baseline,oldfk(q)))
  mjcf_outside+=int(np.any(q<lower) or np.any(q>upper))
 algebra=summarize(rows['mjcf_offset_algebra'])
 assert algebra['translation_m']['maximum']<1e-12 and algebra['rotation_rad']['maximum']<1e-12
 # Unchanged server injectors must still validate the complete real collision-mesh jaw contract.
 compiled=S._inject_actuators(S._inject_options(ET.tostring(derived,encoding='unicode')))
 cr=ET.fromstring(compiled);cr.find('compiler').set('meshdir',str((SOURCE.parent/original.find('compiler').get('meshdir')).resolve()))
 gm=mujoco.MjModel.from_xml_string(ET.tostring(cr,encoding='unicode'));gd=mujoco.MjData(gm)
 backend=object.__new__(S.MujocoDigitalTwinBackend);backend._mujoco=mujoco
 meta={k:mujoco.mj_name2id(gm,typ,n) for k,typ,n in [('left_finger_joint',mujoco.mjtObj.mjOBJ_JOINT,'left_finger'),('right_finger_joint',mujoco.mjtObj.mjOBJ_JOINT,'right_finger'),('left_finger_actuator',mujoco.mjtObj.mjOBJ_ACTUATOR,'left_finger_act'),('right_finger_actuator',mujoco.mjtObj.mjOBJ_ACTUATOR,'right_finger_act'),('left_body',mujoco.mjtObj.mjOBJ_BODY,'Link7'),('right_body',mujoco.mjtObj.mjOBJ_BODY,'Link8'),('orientation_body',mujoco.mjtObj.mjOBJ_BODY,'Link6')]}
 backend._validate_runtime_gripper_contract(gm,gd,meta)
 artifact.mkdir();a.output.mkdir(parents=True)
 target=artifact/'four_body_offset_candidate_NOT_FOR_DEPLOYMENT.xml';target.write_text('<!-- REVIEW ONLY: retains legacy TCP/frame mismatch; not deployable equivalence -->\n'+ET.tostring(derived,encoding='unicode')+'\n')
 result=dict(schema=1,execution_authority=False,deployed=False,network_writes=False,mujoco_version=mujoco.__version__,generator_sha256=sha(__file__),source_xml=str(SOURCE),source_xml_sha256=SOURCE_SHA,archived_deployment_xml=str(archive),archived_deployment_xml_equal=True,candidate_urdf_sha256=sha(newU),candidate_X_sha256=sha(Xp),candidate_pair_sha256=manifest['candidate_pair_sha256'],artifact=str(target),artifact_sha256=sha(target),sample_count=len(samples),random_seed=20260929,same_raw_q_mapping=True,extra_q_offset_applied=False,changes=changes,structurally_only_four_body_quats_changed=True,physics_meshes_finger_ranges_actuator_injection_unchanged=True,actual_server_gripper_contract_passed=True,statistics={k:summarize(v) for k,v in rows.items()},zero_pose_derived_legacy_tcp_to_ros_tool=C.tolist(),zero_pose_derived_Link6_to_ros_tool=H.tolist(),T_tool_camera=X.tolist(),urdf_limits_rad=newfk.limits.tolist(),mjcf_limits_rad=np.column_stack((lower,upper)).tolist(),urdf_legal_samples_outside_unchanged_mjcf_limits=mjcf_outside,strict_URDF_MJCF_identity_passed=False,reason='Four-axis offset algebra is exact inside legacy MJCF, but legacy TCP/frame/model already differs from frozen URDF. Do not deploy this artifact or treat fixed zero-pose mapping as full exact equivalence.',current_server_tcp_contract='arm payload raw radians to qpos directly; TCP translation=(Link7.xpos+Link8.xpos)/2; TCP rotation=Link6.xmat; IK translation Jacobian is mean of both finger body Jacobians; no camera frame or handeye in model',camera_mapping_scope='palm(q) @ H_zero_derived @ candidate_X; virtual camera only; inherits finite residual against URDF and no camera physical body/mesh claim')
 (a.output/'report.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
 (artifact/'README.md').write_text('# Review-only four-body offset MJCF\n\nNot deployed, not an exact replacement for the candidate URDF. See '+str(a.output/'report.json')+' for 1114-q comparison and the pre-existing TCP/model/limit mismatch. Only Joint2–5 body quaternions change; raw q, physics, meshes, gripper ranges, friction and force limits remain original.\n\nXML meshdir intentionally retains original deployment layout. The generator resolves the existing source asset directory only in memory for local compilation.\n')
 print(json.dumps(dict(report=str(a.output/'report.json'),artifact=str(target),statistics=result['statistics'],outside_mjcf_limits=mjcf_outside,gripper_contract=True),ensure_ascii=False))
if __name__=='__main__':main()
