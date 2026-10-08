"""Read-only exact-STL surface/OBB clipping diagnostic; never a motion certificate."""
import ast, hashlib, json, math, struct
from pathlib import Path
import numpy as np

ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/candidate_real_grasp_20260929T052500Z/offline_initial_pad_audit')
G=Path('src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/gripper_geometry.py')
M=Path('src/arm-mujoco/synriard/meshes/Alicia_D_v5_6/gripper_50mm')

def constant(name):
    tree=ast.parse(G.read_text())
    for a in tree.body:
        if isinstance(a,ast.Assign) and any(isinstance(t,ast.Name) and t.id==name for t in a.targets):
            return np.array(ast.literal_eval(a.value.args[0]),dtype=float)
    raise KeyError(name)

def margin(point, poly):
    inside=False; dist=float('inf');x,y=point
    for a,b in zip(poly,np.roll(poly,-1,axis=0)):
        d=b-a;t=float(np.clip(np.dot(point-a,d)/np.dot(d,d),0,1));dist=min(dist,float(np.linalg.norm(point-a-t*d)))
        if (a[1]>y)!=(b[1]>y) and x<(b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0]:inside=not inside
    return dist if inside else -dist

def clip(poly,A,b):
    for n,limit in zip(A,b):
        if not len(poly):return []
        out=[]
        for a,c in zip(poly,np.roll(poly,-1,axis=0)):
            da=float(n@a-limit);dc=float(n@c-limit)
            if da<=0:out.append(a)
            if (da<=0)!=(dc<=0):out.append(a+da/(da-dc)*(c-a))
        poly=np.array(out)
    return poly

def main():
    source=ROOT/'initial_pad_audit.json';j=json.loads(source.read_text());center=np.array(j['target_tool_center_m']);R=np.array(j['target_tool_rotation']);size=np.array(j['target_size_m']);A=np.concatenate([R.T,-R.T]);b=np.concatenate([R.T@center+size/2,-R.T@center+size/2]);opening=j['accepted_opening_m'];q=(.05-opening)/2
    pair=constant('ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M');patch=constant('ANALYTICAL_FINGER_CONTACT_PATCH_TOOL_XZ_M');reports=[]
    for mesh,physical_side,angle,oy,q_sign,axis_z in [('Link7','right_finger',1.5708,-.025469,-1,1),('Link8','left_finger',-1.5708,.025469,1,-1)]:
        p=M/(mesh+'.STL');raw=p.read_bytes();count=struct.unpack('<I',raw[80:84])[0];data=np.frombuffer(raw,dtype=np.dtype([('normal','<f4',(3,)),('vertex','<f4',(3,3)),('attribute','<u2')]),offset=84,count=count);tri=data['vertex'].astype(float)
        rot=np.array([[1.,0,0],[0,math.cos(angle),-math.sin(angle)],[0,math.sin(angle),math.cos(angle)]])
        translation=np.array([.0002,oy,.13098])+rot@np.array([0,0,axis_z*q_sign*q])-np.array([-.0002,-.0003,.13118])
        transformed=tri@rot.T+translation;vertices=[];tri_ids=[]
        for idx,face in enumerate(transformed):
            clipped=clip(face,A,b)
            if len(clipped):vertices.extend(clipped);tri_ids.extend([idx]*len(clipped))
        if not vertices:
            reports.append(dict(mesh=mesh,name=physical_side,surface_intersection_found=False));continue
        vertices=np.array(vertices);margins=np.array([margin(x[[0,2]]-pair[[0,2]],patch) for x in vertices]);idx=int(np.argmin(margins));pstar=vertices[idx];side=1 if physical_side=='left_finger' else -1
        reports.append(dict(mesh=mesh,name=physical_side,mesh_sha256=hashlib.sha256(raw).hexdigest(),
            surface_intersection_found=True,clipped_surface_vertex_count=len(vertices),distinct_surface_triangles=len(set(tri_ids)),
            source50mm_q_native_m=q_sign*q,tool_from_mesh_rotation=rot.tolist(),tool_from_mesh_translation_m=translation.tolist(),
            minimum_xz_pad_margin_m=float(margins[idx]),worst_surface_point_tool_m=pstar.tolist(),worst_triangle_index=int(tri_ids[idx]),
            worst_point_target_halfspace_max_residual_m=float(np.max(A@pstar-b)),
            worst_point_penetration_m=float(side*(pstar[1]-pair[1])-.5*opening),
            true_mesh_tool_aabb_m=[transformed.reshape(-1,3).min(axis=0).tolist(),transformed.reshape(-1,3).max(axis=0).tolist()],
            negative_pad_margin_vertices=int(np.count_nonzero(margins<-1e-8))))
    out=dict(scope='closed_bag_STL_surface_OBB_diagnostic_not_solid_certificate',input_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        input=str(source),frozen_opening_m=opening,physical_side_mapping='analytical +Y left_finger = 50mm Link8; analytical -Y right_finger = 50mm Link7',
        results=reports,contains_volume_test=False,convex_decomposition_validated=False,roundoff_certificate=False,
        motion_authorized=False,production_changed=False,implementation_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        limitations=['Surface clipping identifies material-boundary/OBB overlap only; it is not a full solid collision or separation proof.',
                     'No padding is added in this diagnostic; it tests raw frozen 50mm CAD before any conservative uncertainty dilation.',
                     'Target OBB remains a conservative object model, not observed target material.',
                     'An unchanged finite pad may still reject bevel/edge overlap even when the real target was reported held.'])
    target=ROOT/'mesh_boundary_diagnostic.json';target.write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n');print(json.dumps(out,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
