"""Independent potential-energy and payload sensitivity audit; no ROS/hardware."""
from pathlib import Path
import hashlib,json,math,xml.etree.ElementTree as ET
import numpy as np
ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_ros_latest_start_20261002/gen4_root_cause')
SESSION=ROOT/'high_posture_20261003T081809Z';OUT=ROOT/'joint_identifiability'
source_path=Path('.runtime/gen4_gravity_direction_readonly.py')
ns={};exec(source_path.read_text().split('\nrows=[]')[0],ns)
xml=(SESSION/'robot_description.urdf').read_text();tree=ET.fromstring(xml);joints=tree.findall('joint');links={e.get('name'):e for e in tree.findall('link')}
source=json.loads((SESSION/'fresh_plan/snapshot.json').read_text());X=np.array(source['T_tool_camera'])
events=[json.loads(x) for x in (SESSION/'events.jsonl').read_text().splitlines()];rows=[x for x in events if x.get('event')=='step_complete']
plane_summary=json.loads((SESSION/'support_plane_review/summary.json').read_text());plane_rows={x['label']:x for x in plane_summary['results']}

def xyz(e,key='xyz',default='0 0 0'):
 return np.fromstring(e.get(key,default),sep=' ') if e is not None else np.fromstring(default,sep=' ')

def rot(axis,angle):
 axis=np.asarray(axis,float);axis=axis/np.linalg.norm(axis);x,y,z=axis;K=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
 return np.eye(3)+math.sin(angle)*K+(1-math.cos(angle))*(K@K)

def frames(q):
 values=dict(zip(['Joint%d'%i for i in range(1,7)]+['right_finger'],q));T={'base_link':np.eye(4)};axes={};todo=list(joints)
 while todo:
  before=len(todo)
  for j in todo[:]:
   parent=j.find('parent').get('link');child=j.find('child').get('link')
   if parent not in T:continue
   o=j.find('origin');r,p,y=xyz(o,'rpy');M=np.eye(4);M[:3,:3]=rot([0,0,1],y)@rot([0,1,0],p)@rot([1,0,0],r);M[:3,3]=xyz(o)
   A=T[parent]@M;axis=xyz(j.find('axis'),default='1 0 0');axis=axis/np.linalg.norm(axis);name=j.get('name');v=values.get(name,0);mimic=j.find('mimic')
   if mimic is not None:v=values.get(mimic.get('joint'),0)*float(mimic.get('multiplier',1))+float(mimic.get('offset',0))
   J=np.eye(4)
   if j.get('type') in ('revolute','continuous'):axes[name]=(A[:3,3],A[:3,:3]@axis,child);J[:3,:3]=rot(axis,v)
   elif j.get('type')=='prismatic':J[:3,3]=axis*v
   T[child]=A@J;todo.remove(j)
  assert len(todo)<before
 return T,axes

def masses(T):
 output=[]
 for name,l in links.items():
  inert=l.find('inertial')
  if inert is not None:
   mass=float(inert.find('mass').get('value'));point=(T[name]@np.r_[xyz(inert.find('origin')),1])[:3];output.append((name,mass,point))
 return output

def potential(q,g=np.array([0.,0.,-9.80665]),camera_mass=0):
 T,_=frames(q);return -sum(m*p@g for _,m,p in masses(T))-camera_mass*((T['tool0']@X)[:3,3]@g)

records=[];max_frame_diff=0.;max_tau_diff=0.;max_payload_diff=0.;eps=1e-6
for row in rows:
 q=np.array(row['q']);T,axes=frames(q);Tr,ar=ns['frames'](q);max_frame_diff=max(max_frame_diff,max(np.max(abs(T[k]-Tr[k])) for k in T))
 record=dict(label=row['label'],q_deg=np.degrees(q[:6]).tolist(),axes={})
 for index in (1,4):
  name='Joint%d'%(index+1);o,axis,child=axes[name];desc=ns['descendants'](child);g=np.array([0,0,-9.80665]);torque=sum(float(np.cross(p-o,m*g)@axis) for link,m,p in masses(T) if link in desc)
  plus=q.copy();minus=q.copy();plus[index]+=eps;minus[index]-=eps
  fd=-(potential(plus)-potential(minus))/(2*eps);max_tau_diff=max(max_tau_diff,abs(torque-fd))
  cam=(T['tool0']@X)[:3,3];perkg=float(np.cross(cam-o,g)@axis);fdk=(-(potential(plus,camera_mass=1)-potential(minus,camera_mass=1))/(2*eps))-fd;max_payload_diff=max(max_payload_diff,abs(perkg-fdk))
  # Location sensitivity is deterministic per kilogram; these are NOT claims
  # about the camera COM or its actual mass/cable forces.
  offsets=[np.array([x,y,z]) for x in (-.03,.03) for y in (-.03,.03) for z in (-.03,.03)]
  com_sensitivity=[float(np.cross(cam+(T['tool0']@X)[:3,:3]@d-o,g)@axis) for d in offsets]
  gravity_scenarios={}
  for global_axis,axis_name in (([1,0,0],'x'),([0,1,0],'y')):
   for degrees in (-3,3):
    gt=rot(global_axis,math.radians(degrees))@g
    gravity_scenarios[f'gravity_tilt_{axis_name}_{degrees}_deg']=sum(float(np.cross(p-o,m*gt)@axis) for link,m,p in masses(T) if link in desc)
  record['axes'][name]=dict(urdf_gravity_Nm=torque,negative_dpotential_dq_Nm=fd,camera_origin_point_mass_Nm_per_kg=perkg,camera_origin_plusminus30mm_com_Nm_per_kg_range=[min(com_sensitivity),max(com_sensitivity)],camera_origin_critical_positive_mass_for_zero_kg=(-torque/perkg if abs(perkg)>1e-12 and -torque/perkg>0 else None),example_added_mass_kg_to_total_torque_Nm={str(m):torque+m*perkg for m in (0,.05,.1,.2)},urdf_only_gravity_direction_scenarios_Nm=gravity_scenarios)
 if row['label'] in plane_rows:record['independent_support_anchor_shift_mm']=plane_rows[row['label']]['anchor_shift_mm']
 records.append(record)

def crossings(m):
 out=[r for r in records if r['label'].startswith('domain_')];found=[]
 for r0,r1 in zip(out,out[1:]):
  t0=r0['axes']['Joint2'];t1=r1['axes']['Joint2'];a=t0['urdf_gravity_Nm']+m*t0['camera_origin_point_mass_Nm_per_kg'];b=t1['urdf_gravity_Nm']+m*t1['camera_origin_point_mass_Nm_per_kg']
  if a*b<=0:found.append([r0['label'],r1['label']])
 return found
report=dict(scope='Independent URDF parent chain/formula review and diagnostic sensitivity only; no hardware or calibration changes.',
formula='tau_gravity_j = sum_downstream axis_base dot ((COM_base-origin_j_base) cross mass*g_base). Equals -dU/dq. Static motor holding torque has opposite sign.',
validation=dict(max_independent_forward_frame_difference=float(max_frame_diff),max_negative_potential_gradient_vs_axis_moment_Nm=float(max_tau_diff),max_camera_unit_mass_gradient_vs_axis_moment_Nm_per_kg=float(max_payload_diff),all_urdf_axis_norms={j.get('name'):float(np.linalg.norm(xyz(j.find('axis'),default='1 0 0'))) for j in joints if j.get('type') in ('revolute','continuous')},inertial_origin_rotation_irrelevant_for_COM_position=True),
source_urdf_sha256=hashlib.sha256(xml.encode()).hexdigest(),reviewed_root_script_sha256=hashlib.sha256(source_path.read_bytes()).hexdigest(),
limits=['Camera optical/link origin is a hypothetical lumped point mass location, not a measured camera COM. Listed masses are sensitivity examples, not measured payloads.','No cable force, contact force, friction, backlash, compliance, motor current or real gravity/base orientation is measured.','A sign coincidence in incomplete URDF model is not evidence of physical joint torque sign reversal.','Support shift uses independent same-texture baseline comparison and header-bound TF, not calibrated image exposure time.'],
outbound_J2_sign_crossings_by_hypothetical_camera_origin_mass_kg={str(m):crossings(m) for m in (0,.025,.05,.075,.1,.2)},rows=records)
(OUT/'gravity_independent_review_and_payload_sensitivity.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
(OUT/'gravity_independent_review_source.py').write_bytes(Path(__file__).read_bytes())
print(json.dumps(report['validation'],indent=2));print('crossings',report['outbound_J2_sign_crossings_by_hypothetical_camera_origin_mass_kg'])
for r in records:
 if r['label'] in ('domain_06_j5','domain_07_j2','domain_07_j5','high_j2_positive3','return_02_j2','return_04_j2'):
  print(r['label'],r['axes']['Joint2'], 'J5/kg',r['axes']['Joint5']['camera_origin_point_mass_Nm_per_kg'])
