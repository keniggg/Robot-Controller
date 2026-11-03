"""Original R18 pixels -> measured plane registration -> full fused qualifier."""
from pathlib import Path
from dataclasses import replace
from types import SimpleNamespace
import hashlib, importlib.util, json, os, platform, signal, struct, sys, time
W=Path(__file__).resolve().parents[1]
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
V=E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
B=W/'.runtime/gen53_baseline_source'
C=W/'.runtime/gen53_registration_candidate'
INPUT=V/'gen52_actual_handoff_monitor_r1/exact_box_inputs_r1'
OUT=V/'gen53_r18_support_plane_final_qualification_r1'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    assert Path('/media/zhuyupei/SanStand').is_mount() and not OUT.exists()
    OUT.mkdir()
    os.sched_setaffinity(0, sorted(os.sched_getaffinity(0))[:2])
    os.environ.update(PYTHONDONTWRITEBYTECODE='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', TMPDIR=str(OUT))
    sys.dont_write_bytecode=True
    native=os.uname()
    platform.uname=lambda:platform.uname_result(*(tuple(native)+(native.machine,)))
    platform.architecture=lambda *a,**k:(str(8*struct.calcsize('P'))+'bit','ELF')
    denied=[]
    def guard(event,args):
        bad=event.startswith('socket.') or event in ('subprocess.Popen','os.system','os.exec','os.posix_spawn','os.fork','os.forkpty')
        if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)):
            p=os.fsdecode(args[0]); flags=args[2] if len(args)>2 and isinstance(args[2],int) else 0
            mode=args[1] if len(args)>1 and isinstance(args[1],str) else ''
            bad|=(p.startswith('/dev/') and p not in ('/dev/null','/dev/urandom','/dev/random')) or p.startswith('/sys/class/thermal') or '/hwmon/' in p
            if p!='/dev/null' and (flags&(os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC|os.O_APPEND) or any(c in mode for c in 'wax+')):
                bad|=not (Path(p).resolve()==OUT or OUT in Path(p).resolve().parents)
        if bad:denied.append(event);raise RuntimeError('offline boundary '+event)
    sys.addaudithook(guard)
    signal.signal(signal.SIGALRM,lambda *a:(_ for _ in ()).throw(TimeoutError('45 second final qualifier budget')))
    signal.alarm(45)
    import numpy as np,cv2
    cv2.setNumThreads(1)
    sys.path.insert(0,str(B/'src/alicia_flexible_grasp_supervisor/src'))
    import alicia_flexible_grasp.vision
    candidate=C/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/box_surface_registration.py'
    name='alicia_flexible_grasp.vision.box_surface_registration'
    spec=importlib.util.spec_from_file_location(name,candidate)
    choice=importlib.util.module_from_spec(spec);sys.modules[name]=choice;spec.loader.exec_module(choice)
    alicia_flexible_grasp.vision.box_surface_registration=choice
    from alicia_flexible_grasp.vision import regular_box_inference as box
    from alicia_flexible_grasp.vision import multiview_surface as surface
    from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
    from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
    from alicia_flexible_grasp.vision.fused_scene_geometry import enclose_registered_surface
    from alicia_flexible_grasp.vision.cuboid_rgb_interval import _plain
    fixture=C/'src/alicia_flexible_grasp_supervisor/tests/fixtures/gen53_g52_r18_support_registration.json'
    assert sha(fixture)=='d2bde5957ff5fddbfd2c24d83d59ebe45b8f24d5ceedafc4df1d251a38441618'
    data=json.loads(fixture.read_text());gd=dict(data['raw_geometry'])
    for k in ('center_base','axes_base','size_xyz_m','support_normal_base','object_points_base'):gd[k]=np.asarray(gd[k])
    raw=GeometryEstimate(**gd);model=box.RegularBoxInference.from_evidence_dict(data['raw_model'])
    assert box.regular_box_geometry_sha256(raw)==model.source['geometry_sha256']
    rd=dict(data['reference']);rd['identity']=TargetTrackIdentity(**rd['identity']);reference=surface.SurfaceView(**rd)
    bd=dict(data['before']);bd['identity']=TargetTrackIdentity(**bd['identity']);before=surface.FusedTargetSurface(**bd)
    moving=surface.SurfaceView(reference.identity,data['moving_stamp_ns'],raw.object_points_base,raw.support_normal_base,raw.support_offset_m,data['edge_clearance_px'])
    config=surface.RegistrationConfig(**data['registration_config'])
    start=time.monotonic()
    registered,fused,audit=choice.append_box_consistent_registered_view(before,reference,moving,raw,model,config)
    registration_elapsed=time.monotonic()-start
    assert registered.ok and audit['selected_hypothesis']=='support_plane_minimal'
    inverse=np.linalg.inv(registered.transform_base)
    snapshot=replace(fused,points_base=fused.points_base@inverse[:3,:3].T+inverse[:3,3])
    geometry,enclosure=enclose_registered_surface(raw,snapshot,expected_identity=moving.identity)
    assert not enclosure['contact_or_motion_authorized']
    receipt=json.loads((INPUT/'result.json').read_text())
    assert sha(INPUT/'result.json')=='65bb78ad0ee7c6fa9b110fbc82cd1776e42db800eba579d4aea2102714458cf9'
    ap=Path(receipt['audit']['path']);assert sha(ap)==data['source_audit_sha256'];a=json.loads(ap.read_text())
    context=json.loads((INPUT/'original_nonimage_context.json').read_text())
    cams=[r['parsed_json']['parameters']for r in context if r['topic']=='/grasp_record/camera_runtime_metadata']
    assert cams and all(c==cams[0]for c in cams)
    cam=SimpleNamespace(**{k:cams[0][k]for k in ('width','height','fx','fy','cx','cy','depth_scale')})
    arrays={k:np.load(INPUT/(str(moving.stamp_ns)+'_'+k+'.npy'),allow_pickle=False)for k in ('color','depth','mask')}
    for k,x in arrays.items():assert hashlib.sha256(x.tobytes()).hexdigest()==receipt['final_retained_pixels'][k]['expected_retained_sha256']
    start=time.monotonic()
    q=box.qualify_regular_box_top(arrays['depth'],arrays['mask'],cam,cam.depth_scale,
        np.asarray(a['snapshot_transform']['T_base_optical']),geometry,moving.identity,moving.stamp_ns,
        'instance_mask',color_bgr=arrays['color'])
    qualification_elapsed=time.monotonic()-start
    qualified=dict(ok=q.ok,code=q.code,audit=_plain(dict(q.audit)),model=q.model.to_evidence_dict()if q.model else None)
    (OUT/'final_qualification.json').write_text(json.dumps(qualified,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    provenance={}
    for name,module in list(sys.modules.items()):
        if name.startswith('alicia_flexible_grasp')and getattr(module,'__file__',None):
            p=Path(module.__file__).resolve();assert p==candidate or B in p.parents
            provenance[name]=dict(path=str(p),sha256=sha(p))
    result=dict(passed=bool(q.ok)and not denied,diagnostic_only=True,execution_authorized=False,
        real_grasp_success=False,registration_audit=audit,registration_elapsed_sec=registration_elapsed,
        original_icp_and_identity_both_conflict=True,all_preappend_points_checked=1563,
        raw_count=806,new_fused_count=len(fused.points_base),new_exact_enclosure_count=len(geometry.object_points_base),
        enclosure=enclosure,final_qualification=dict(path=str(OUT/'final_qualification.json'),sha256=sha(OUT/'final_qualification.json'),
            ok=q.ok,code=q.code,model_sha256=q.model.model_sha256 if q.model else None,elapsed_sec=qualification_elapsed),
        source_provenance=provenance,fixture_sha256=sha(fixture),helper_sha256=sha(__file__),
        original_pixels_sha256=receipt['final_retained_pixels'],source_stamp_ns=moving.stamp_ns,
        original_source_bound_raw_model_sha256=model.model_sha256,external_boundary_attempts=denied,
        ROS_initialized=False,hardware_contacted=False,production_modified=False,
        temporal_snapshot_or_task_replayed=False,full_path_or_execution_verified=False)
    (OUT/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    signal.alarm(0)
    print(json.dumps(dict(result=str(OUT/'result.json'),sha256=sha(OUT/'result.json'),passed=result['passed'],
        final_qualification=result['final_qualification']),ensure_ascii=False))
if __name__=='__main__':main()
