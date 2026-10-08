"""Read-only URDF gravity-direction check; no fitted model or hardware writes."""
from pathlib import Path
import json,math,xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_ros_latest_start_20261002/gen4_root_cause')
session=ROOT/'high_posture_20261003T081809Z'
xml=ET.fromstring((session/'robot_description.urdf').read_text());joints=xml.findall('joint')
links={l.attrib['name']:l for l in xml.findall('link')}
def numbers(node,key,default):return np.fromstring(node.attrib.get(key,default),sep=' ')
def frames(q):
 values=dict(zip(['Joint%d'%i for i in range(1,7)]+['right_finger'],q));T={'base_link':np.eye(4)};axes={};pending=list(joints)
 while pending:
  progress=False
  for j in list(pending):
   parent=j.find('parent').attrib['link'];child=j.find('child').attrib['link']
   if parent not in T:continue
   name=j.attrib['name'];o=j.find('origin');M=np.eye(4)
   if o is not None:M[:3,3]=numbers(o,'xyz','0 0 0');M[:3,:3]=Rotation.from_euler('xyz',numbers(o,'rpy','0 0 0')).as_dcm()
   F=T[parent]@M;axis=numbers(j.find('axis'),'xyz','1 0 0') if j.find('axis') is not None else np.array([1.,0.,0.]);v=values.get(name,0.)
   mimic=j.find('mimic')
   if mimic is not None:v=values.get(mimic.attrib['joint'],0.)*float(mimic.attrib.get('multiplier',1))+float(mimic.attrib.get('offset',0))
   move=np.eye(4)
   if j.attrib['type'] in ('revolute','continuous'):
    axes[name]=(F[:3,3],F[:3,:3]@axis,child);move[:3,:3]=Rotation.from_rotvec(v*axis).as_dcm()
   elif j.attrib['type']=='prismatic':move[:3,3]=axis*v
   T[child]=F@move;pending.remove(j);progress=True
  if not progress:raise RuntimeError('unreachable URDF tree')
 return T,axes
children={}
for j in joints:children.setdefault(j.find('parent').attrib['link'],[]).append(j.find('child').attrib['link'])
def descendants(link):
 out={link}
 for child in children.get(link,[]):out|=descendants(child)
 return out
rows=[]
for line in (session/'events.jsonl').read_text().splitlines():
 r=json.loads(line)
 if r['event']!='step_complete':continue
 T,axes=frames(r['q']);data={}
 for name in ['Joint2','Joint5']:
  o,axis,child=axes[name];torque=0.;masses=[]
  for link in descendants(child):
   inertial=links[link].find('inertial')
   if inertial is None:continue
   mass=float(inertial.find('mass').attrib['value']);position=T[link][:3,:3]@numbers(inertial.find('origin'),'xyz','0 0 0')+T[link][:3,3]
   torque+=float(np.cross(position-o,[0,0,-9.80665*mass])@axis);masses.append(mass)
  data[name]={'urdf_only_gravity_torque_Nm':torque,'included_urdf_mass_kg':sum(masses)}
 rows.append({'label':r['label'],'q_deg':np.degrees(r['q'][:6]).tolist(),'model':data})
report={'kind':'unfitted_urdf_gravity_direction_sensitivity','robot_commands_published':0,'limits':['URDF mass/inertia is model data, not force measurement.','Camera/cables/mount mass and compliance not represented; result cannot identify backlash or base motion.','No control parameter or calibration change; this only tests a model-level sign-reversal hypothesis.'],'rows':rows}
out=ROOT/'joint_identifiability'/'urdf_gravity_direction.json';out.write_text(json.dumps(report,indent=2)+'\n')
for r in rows:
 if r['label'] in ['domain_01_j5','domain_05_j5','domain_06_j5','domain_07_j2','domain_07_j5','high_j5_negative3','high_j2_positive3']:
  print(r['label'],{k:round(v['urdf_only_gravity_torque_Nm'],4) for k,v in r['model'].items()})
print('saved',out)
