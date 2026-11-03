"""Quantify finite CAD collisions in the unchanged older RGBD fixture."""
import ast
import json
import os
from pathlib import Path
import platform
import struct
import sys
import extract_gen50_retry_r30_rgb_20261018 as boundary

W = Path(__file__).resolve().parents[1]
C = W / '.runtime/gen54_integrated_candidate'
OUT = boundary.N / 'verification/gen54_geometry_pipeline_r1/recorded_compat_sat_r1'


def main():
    assert boundary.E.parent.parent.is_mount() and not OUT.exists()
    OUT.mkdir()
    os.environ.update(PYTHONDONTWRITEBYTECODE='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', TMPDIR=str(OUT))
    sys.dont_write_bytecode=True
    native=os.uname()
    platform.uname=lambda:platform.uname_result(*(tuple(native)+(native.machine,)))
    platform.architecture=lambda *a,**k:(str(8*struct.calcsize('P'))+'bit','ELF')
    boundary.OUT=OUT
    sys.addaudithook(boundary.audit)
    import numpy as np
    sys.path.insert(0,str(C/'src/alicia_flexible_grasp_supervisor/src'))
    from alicia_flexible_grasp.grasp import gripper_geometry as gg
    from alicia_flexible_grasp.vision.cuboid_rgb_interval import _plain
    source=W/'.runtime/gen54_collision_geometry_r1/analyze_original.py'
    tree=ast.parse(source.read_text())
    satfn=next(x for x in tree.body if isinstance(x,ast.FunctionDef) and x.name=='sat')
    env={'np':np}
    exec(compile(ast.Module(body=[satfn],type_ignores=[]),str(source),'exec'),env)
    sat=env['sat']
    path=OUT.parent/'recorded_compat_r3/result.json'
    old=json.loads(path.read_text())
    rows=[]
    g=gg.GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003)
    for row in old['outputs'][0]['rows']:
        r=row['failing_checks'][0]
        T=np.array(r['transform']);R=T[:3,:3]
        center=np.array(r['obb_center']);rotation=np.array(r['obb_rotation']);size=np.array(r['obb_size'])
        bodies=[]
        for name,c,s in gg._stage_boxes(T,g,r['physical_gap_m'],'y','z'):
            padded=sat(c,R,s,center,rotation,size)
            assert padded['overlap']==gg._obb_overlap(c,R,s,center,rotation,size)
            rawsize=s-gg.ANALYTICAL_FINGER_BOX_PADDING_XYZ_M if 'finger' in name else s
            bodies.append(dict(name=name,padded=padded,without_padding_diagnostic=sat(c,R,rawsize,center,rotation,size)))
        rows.append(dict(source_index=row['source_index'],variant_index=row['variant_index'],
                         angle_deg=row['audit']['sampled_angle_deg'],tilt_deg=row['audit']['approach_tilt_deg'],
                         required_open_width_m=row['required_open_width_m'],physical_opening_m=r['physical_gap_m'],
                         full_obb_jaw_projection_m=float(np.abs(rotation.T@R[:,1])@size),
                         bodies=bodies))
    result=dict(diagnostic_only=True,motion_authorized=False,real_grasp_success=False,
                source_result=str(path),source_result_sha256=boundary.sha(path),
                independently_reviewed_sat_source=str(source),sat_source_sha256=boundary.sha(source),
                boundary_attempts=boundary.BOUNDARY,rows=rows)
    (OUT/'result.json').write_text(json.dumps(_plain(result),indent=2,allow_nan=False)+'\n')
    print(json.dumps([dict(angle=x['angle_deg'],variant=x['variant_index'],projection=x['full_obb_jaw_projection_m'],
                          bodies=[dict(name=b['name'],overlap=b['padded']['overlap'],depth=b['padded']['minimum_axis']['overlap_m'],
                                       bare_depth=b['without_padding_diagnostic']['minimum_axis']['overlap_m']) for b in x['bodies']]) for x in rows]))


if __name__=='__main__':main()
