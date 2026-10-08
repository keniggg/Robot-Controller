import sys,json,itertools,time
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
import bounded_board_centering as c
b=c.motor.load_module('pure_base',HERE/'bounded_joint_response_diagnostic.py')
A=c.wrist.ARCHIVE;S=A/'bounded_wrist_response_diagnostic_20260929T020336Z_45784'
g=json.loads((S/'baseline_geometry.json').read_text());q=np.array(g['accepted_positions_rad']);X=np.array(g['T_tool_camera']);Y=np.array(g['T_base_board']);m=b.CAMERA.RuntimeColorModel(g['quality']['camera_model'])
fk=b.SerialUrdfFk((S/'robot_description.urdf').read_text(),b.NAMES[:6]);gr=b.GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003)
r=np.array([-1.833107041523341,.3742913122440964,-.12118448224296564,0.,-.20862138715244719,-.0046019423636569235,.04975])
geom=dict(q=q,X=X,Y=Y,model=m);inc,proposal=c.solve_board_center(fk,geom);contract=c.board_contract(q,r,1790645893464309304,inc)
corners=np.array([[x,y,0,1] for x in [0,.18] for y in [0,.135]])
out=[]
for j in [1,2]:
 target=r.copy();target[j]+=inc[j]*c.motor.QUANTUM;lo,hi,_=contract.rectangle(q,target,r,j)
 worst=None
 for bits in itertools.product([0,1],repeat=6):
  a=np.where(bits,hi,lo);C=np.linalg.inv(fk(a)@X);pc=(C@Y@corners.T).T
  uv=np.asarray([m.sdk.rs2_project_point_to_pixel(m.intrinsics,[-p[1],-p[2],p[0]]) for p in pc]);edge=float(min(uv[:,0].min(),639-uv[:,0].max(),uv[:,1].min(),479-uv[:,1].max()))
  if worst is None or edge<worst['edge']:worst=dict(edge=edge,delta_deg=np.degrees(a-q[:6]).tolist(),pixels=uv.tolist())
 print('EXACT_VERTEX',j+1,json.dumps(worst),flush=True)
 out.append(dict(joint=j+1,vertex_worst=worst,lower_rad=lo.tolist(),upper_rad=hi.tolist()))
 if worst['edge']<20:continue
 # Fixed-depth bisection only on failing cells; exact complete union preserved.
 queue=[(a,h,0) for a,h in c.motor.joint_cells(lo,hi)];counts=[0]*7;minimum=1e9;failed=None;start=time.monotonic()
 while queue:
  a,h,d=queue.pop();counts[d]+=1
  try:p=c.certify_joint_box_geometry(fk,gr,np.r_[a,q[6]],geom,fk.limits[:,0],fk.limits[:,1],h);minimum=min(minimum,p['minimum_full_board_edge_lower_bound_px'])
  except RuntimeError as error:
   if d>=6:failed=str(error);break
   axis=np.argmax(h);child=h.copy();child[axis]*=.5;delta=np.eye(6)[axis]*child[axis]
   queue.extend([(a-delta,child,d+1),(a+delta,child,d+1)])
 out[-1].update(bisection_proof_passed=failed is None,failure=failed,counts=counts,minimum_certified_edge=minimum,elapsed=time.monotonic()-start)
 print('BISECTION',j+1,json.dumps(out[-1]),flush=True)
R=Path(json.loads((HERE/'latest_restart_grasp.json').read_text())['directory'])/'board_centering_offline_proof';R.mkdir(exist_ok=True)
(R/'firstbox_diagnosis.json').write_text(json.dumps(out,indent=2)+'\n')
