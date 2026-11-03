from pathlib import Path
import json,hashlib,itertools,math
import numpy as np
from tf.transformations import quaternion_matrix
from alicia_flexible_grasp.grasp.gripper_geometry import ANALYTICAL_FINGER_CONTACT_PATCH_TOOL_XZ_M as PATCH,ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M as PAIR
ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_frozen_no_skin_20261012T021050Z')
def area(poly):
 p=np.asarray(poly);return abs(float(np.sum(p[:,0]*np.roll(p[:,1],-1)-p[:,1]*np.roll(p[:,0],-1))*.5))if len(poly)>=3 else 0.
def clipped(subject,clip):
 c=np.asarray(clip);sgn=np.sign(np.sum(c[:,0]*np.roll(c[:,1],-1)-c[:,1]*np.roll(c[:,0],-1)))
 result=list(subject)
 for a,b in zip(c,np.roll(c,-1,axis=0)):
  incoming=result;result=[]
  if not incoming:break
  for p,q in zip(incoming,incoming[1:]+incoming[:1]):
   dp=sgn*np.cross(b-a,p-a);dq=sgn*np.cross(b-a,q-a)
   if dp>=-1e-12:result.append(p)
   if (dp>=0)!=(dq>=0):result.append(p+(q-p)*dp/(dp-dq))
 return result
rows=[]
for n in (2936,3001,3089,3154):
 d=json.loads((ROOT/'gen2_analysis/compensation_collision'/str(n)/'offline_reconstruction.json').read_text());g=d['scene'];T=quaternion_matrix(d['registered_goal_pose'][3:]);T[:3,3]=d['registered_goal_pose'][:3];O=quaternion_matrix(g['pose_xyz_xyzw'][3:]);O[:3,3]=g['pose_xyz_xyzw'][:3];size=np.array(g['size_xyz_m']);normal=np.array(g['support_normal_base']);offset=g['support_offset_m'];half=size/2;bottom=float(O[:3,3]@normal+offset-np.abs(O[:3,:3].T@normal)@half)
 row=dict(log_line=n,plan_id=d['plan_id'],object_bottom_to_support_m=bottom,short_face_jaw_normal_angle_deg=math.degrees(math.acos(np.clip(abs(T[:3,1]@O[:3,1]),0,1))),states=[])
 for drop in (0,max(0,bottom)):
  c=O[:3,3]-drop*normal;faces=[]
  for side in (-1,1):
   corners=np.array([[x,side*half[1],z]for x,z in ((-half[0],-half[2]),(half[0],-half[2]),(half[0],half[2]),(-half[0],half[2]))]);tool=(corners@O[:3,:3].T+c-T[:3,3])@T[:3,:3]
   poly=clipped(PATCH+PAIR[[0,2]],tool[:,[0,2]]);arr=np.asarray(poly)
   faces.append(dict(side=side,projected_patch_overlap_area_m2=area(poly),patch_intersection_tool_xz_m=arr.tolist(),tool_z_span_m=(float(np.ptp(arr[:,1]))if len(arr)>1 else 0.)))
  row['states'].append(dict(assumed_support_normal_drop_m=drop,faces=faces))
 rows.append(row)
record=json.loads((ROOT/'gen2_analysis/compensation_collision/3154/recorded_candidates.json').read_text())[0]
auditfile=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261012T021428.519576Z_6e47045b48754a3da85f16cd53ae0cce/keyframes_audits/0b0ba319ce42b8bf982cd3fe66ba7792d070544f62762e597d2df17247ed4797.json')
audit=json.loads(auditfile.read_text());simrow=next(r for r in audit['rows']if r.get('mujoco_selection'))
report=dict(scope='static_contact_patch_vs_obb_projection_only',motion_authorized=False,rows=rows,recorded_second_simulation=simrow['mujoco_selection'],recorded_point_counts=dict(positive=simrow['positive_contact_count'],negative=simrow['negative_contact_count']),evidence_source=str(auditfile),evidence_sha256=hashlib.sha256(auditfile.read_bytes()).hexdigest(),limitations=['Projected finite pad/OBB side-face intersection is a necessary geometric diagnostic, not a dynamic contact or holding certificate.','At the 2.4-degree jaw misalignment projected area cannot be treated as coplanar loaded-contact area.','No MuJoCo was installed in this interpreter; no dynamics were run. First simulation response was not retained in this audit, only core failure text exists.','Original and raised proposals still intersect the finite pad; raising reduces area but does not geometrically erase all possible contact.','The OBB bottom is about 0.75 mm above the measured support plane; static projection after that support-normal settling still has intersection. Actual settling/tilting cannot be inferred without stepwise simulation telemetry.','Current server failure branches return before detailed closure/preload loss-state capture; absent contact forces/normal/position traces prevent attributing failures specifically to height, slipping, tipping, or mesh mismatch.'])
out=ROOT/'gen2_analysis/static_contact_patch_summary.json';out.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n');print(json.dumps(dict(path=str(out),sha256=hashlib.sha256(out.read_bytes()).hexdigest(),rows=[dict(line=r['log_line'],angle=r['short_face_jaw_normal_angle_deg'],faces=[dict(drop=s['assumed_support_normal_drop_m'],areas_mm2=[f['projected_patch_overlap_area_m2']*1e6 for f in s['faces']])for s in r['states']])for r in rows]),indent=2))
