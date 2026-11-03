"""Exact recorded G52 raw/fused public box qualification; no live interfaces."""
from pathlib import Path
import hashlib, json, os, platform, signal, struct, sys, time
from dataclasses import replace
from types import SimpleNamespace

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
V = E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
INPUT = V/'gen52_actual_handoff_monitor_r1/exact_box_inputs_r1'
OUT = V/'gen53_g52_original_box_qualification_r1'

def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
    assert Path('/media/zhuyupei/SanStand').is_mount() and not OUT.exists()
    OUT.mkdir()
    os.environ.update(PYTHONDONTWRITEBYTECODE='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', TMPDIR=str(OUT))
    sys.dont_write_bytecode = True
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
        if bad:
            denied.append(event);raise RuntimeError('offline boundary: '+event)
    sys.addaudithook(guard)
    signal.signal(signal.SIGALRM,lambda *a:(_ for _ in ()).throw(TimeoutError('bounded original qualification')))
    signal.alarm(45)
    import numpy as np,cv2
    cv2.setNumThreads(1)
    sys.path.insert(0,str(W/'src/alicia_flexible_grasp_supervisor/src'))
    from alicia_flexible_grasp.vision.regular_box_inference import qualify_regular_box_top,regular_box_geometry_sha256
    from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
    from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
    from alicia_flexible_grasp.vision.cuboid_rgb_interval import _plain
    release=V/'gen52_release_manifest.json'
    assert sha(release)=='dfa045bdf7051cf177e59722ed5ad5b81e856a59e5492d49bc78c2d7a9a367c1'
    sources=json.loads(release.read_text())['source_sha256']
    assert all(sha(W/p)==h for p,h in sources.items())
    receipt=json.loads((INPUT/'result.json').read_text())
    assert sha(INPUT/'result.json')=='65bb78ad0ee7c6fa9b110fbc82cd1776e42db800eba579d4aea2102714458cf9'
    ap=Path(receipt['audit']['path']);assert sha(ap)==receipt['audit']['sha256'];a=json.loads(ap.read_text())
    g=a['replay_geometry'];f=a['multiview_surface']['fused_scene_geometry']
    geometry=GeometryEstimate(True,'','',np.array(g['obb_center_base_m']),np.array(g['R_base_obb']),np.array(g['obb_size_xyz_m']),np.array(g['support_normal_base']),g['support_offset_m'],g['support_inlier_ratio'],np.array(g['object_points_base_m']),g['source_mode'])
    raw=replace(geometry,center_base=np.array(f['old_center_base_m']),size_xyz_m=np.array(f['old_size_xyz_m']),object_points_base=geometry.object_points_base[:f['current_raw_point_count']])
    assert regular_box_geometry_sha256(raw)==f['registration_choice']['source']['raw_geometry_sha256']
    context=json.loads((INPUT/'original_nonimage_context.json').read_text())
    cams=[r['parsed_json']['parameters'] for r in context if r['topic']=='/grasp_record/camera_runtime_metadata']
    assert cams and all(c==cams[0] for c in cams)
    cam=SimpleNamespace(**{k:cams[0][k] for k in ('width','height','fx','fy','cx','cy','depth_scale')})
    stamp=a['snapshot_stamp_ns'];arrays={k:np.load(INPUT/(str(stamp)+'_'+k+'.npy'),allow_pickle=False) for k in ('color','depth','mask')}
    for k,x in arrays.items():assert hashlib.sha256(x.tobytes()).hexdigest()==receipt['final_retained_pixels'][k]['expected_retained_sha256']
    results={}
    for kind,geo in [('raw',raw),('fused',geometry)]:
        began=time.monotonic();q=qualify_regular_box_top(arrays['depth'],arrays['mask'],cam,cam.depth_scale,np.array(a['snapshot_transform']['T_base_optical']),geo,TargetTrackIdentity(a['target_epoch'],a['multiview_surface']['target_track_id']),stamp,'instance_mask',color_bgr=arrays['color'])
        row=dict(ok=q.ok,code=q.code,audit=_plain(dict(q.audit)),model=q.model.to_evidence_dict() if q.model else None,elapsed_sec=time.monotonic()-began)
        p=OUT/(kind+'_qualification.json');p.write_text(json.dumps(row,ensure_ascii=False,indent=2,allow_nan=False)+'\n');results[kind]=dict(path=str(p),sha256=sha(p),ok=q.ok,code=q.code,model_sha256=q.model.model_sha256 if q.model else None)
    assert results['raw']['ok'] and results['raw']['model_sha256']==f['registration_choice']['raw_model_sha256']
    assert not results['fused']['ok'] and results['fused']['code']=='BOX_MEASURED_GEOMETRY_CONFLICT'
    provenance={}
    for name,module in list(sys.modules.items()):
        if name.startswith('alicia_flexible_grasp') and getattr(module,'__file__',None):
            p=Path(module.__file__).resolve();rel=str(p.relative_to(W));assert sha(p)==sources[rel];provenance[name]=dict(path=str(p),sha256=sources[rel])
    assert all(sha(W/p)==h for p,h in sources.items()) and not denied
    result=dict(passed=True,results=results,source_provenance=provenance,inputs_sha256={str(ap):sha(ap),str(INPUT/'result.json'):sha(INPUT/'result.json'),str(INPUT/'original_nonimage_context.json'):sha(INPUT/'original_nonimage_context.json')},recorded_ns=time.time_ns(),public_qualification_replayed=True,temporal_snapshot_reconstructed=False,ROS_initialized=False,hardware_contacted=False,physical_grasp_verified=False,external_boundary_attempts=denied)
    (OUT/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(result=str(OUT/'result.json'),sha256=sha(OUT/'result.json'),results=results),ensure_ascii=False))

if __name__=='__main__':main()
