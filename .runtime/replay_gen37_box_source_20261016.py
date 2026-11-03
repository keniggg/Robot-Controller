"""Exact retained-source box diagnostic; no ROS imports, network or hardware.

Existing production functions run unchanged. Prior-axis/trimmed-axis probes are
diagnostic proposals only and never authorize contact or motion.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import traceback
from types import SimpleNamespace

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
D = E/'ros_log/full_latest_gen36_restart_20261017T012037Z'
G = E/'grasp_records/20261017T032113.476278Z_a84cc89fc53e4c2b82680749d78b9b9f'
I = D/'verification/gen37_actual_attempt1_source_images_r1'
O = D/'verification/gen37_box_source_replay_r1'
assert Path('/media/zhuyupei/SanStand').is_mount()
assert O.exists() and not (O/'result.json').exists()
assert os.environ['OFFLINE_BOUNDARY_LOG'] == str(O/'boundary.jsonl')
assert 'sitecustomize' in sys.modules
(O/'tmp').mkdir(exist_ok=True)
os.environ.update(TMPDIR=str(O/'tmp'), PYTHONDONTWRITEBYTECODE='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
sys.path.insert(0, str(W/'src/alicia_flexible_grasp_supervisor/src'))

def sha(p):
    h = hashlib.sha256()
    with Path(p).open('rb') as stream:
        for block in iter(lambda:stream.read(4*1024*1024), b''): h.update(block)
    return h.hexdigest()
def read(p): return json.loads(Path(p).read_text())
def arrsha(a): return hashlib.sha256(a.tobytes(order='C')).hexdigest()
def serial(x):
    if hasattr(x, 'tolist'): return x.tolist()
    if hasattr(x, 'item'): return x.item()
    raise TypeError(type(x).__name__)
def save(name, data):
    (O/name).write_text(json.dumps(data, ensure_ascii=False, indent=2, default=serial, allow_nan=False)+'\n')

report = dict(passed=False, kind='gen37_exact_source_box_diagnostic', production_modified=False,
    ros_initialized=False, temperature_values_read=False, motion_commands_sent=False,
    guard_scope='Python socket monkeypatch and Python audit hook for /dev; not OS/C isolation')
try:
    release = read(D/'verification/gen37_release_manifest.json')
    assert sha(D/'verification/gen37_release_manifest.json') == '1ef29fa3c200b139fb3b0fabaa3d6d34b12df76606047e24f61efcfaf6074487'
    before = {p:sha(W/p) for p in release['source_sha256']}
    assert before == release['source_sha256']
    save('source_sha256_before.json', before)
    import cv2
    import numpy as np
    from alicia_flexible_grasp.vision import regular_box_inference as box
    from alicia_flexible_grasp.vision import outer_rgb_rectangle as outer
    from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
    from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
    cv2.setNumThreads(1)
    manifest = read(G/'manifest.json'); extraction = read(I/'receipt.json')
    assert manifest['state'] == 'complete' and manifest['recorder_closed'] and manifest['result'] == 'failure'
    assert sha(G/'manifest.json') == extraction['source_sha256'][str(G/'manifest.json')]
    audits = {p.name:read(p) for p in (G/'keyframes_audits').glob('*.json')}
    for name in audits:
        row = next(r for r in manifest['files'] if r['path'] == 'keyframes_audits/'+name)
        assert sha(G/'keyframes_audits'/name) == row['sha256']
    old = audits['cd24be7b05a0d9141658dd05b8c032df5b443399da5c3622f158ade48c0bba11.json']
    new = audits['0e5ebbd3c562cfab2617b338045f554ffc0920a23626fbc2a49cc474cc9d798d.json']
    oldtrigger = old['observe_return']['trigger_evidence']
    oldq = oldtrigger['regular_box_inference']
    newq = new['remote_diagnostics']['tabletop_geometry']['regular_box_inference']
    camera_path = D/'gen37_runtime/gen37_grasp_scene_r2/camera_configuration.json'
    camera = read(camera_path)
    recorded_k = old['selected']['observation_source_audit']['camera_parameters']
    assert all(camera[k] == recorded_k[k] for k in ('width','height','fx','fy','cx','cy','depth_scale'))
    K = SimpleNamespace(**{k:camera[k] for k in ('width','height','fx','fy','cx','cy')})
    k = np.array([K.fx,K.fy,K.cx,K.cy]); scale = camera['depth_scale']
    stamp = newq['source_stamp_ns']
    assert stamp == new['snapshot_stamp_ns'] == 1792207340733134031
    raw = {}
    for role, suffix in (('color','.png'),('mask','.png'),('depth','.npy')):
        p=I/(str(stamp)+'_'+role+suffix); assert sha(p) == extraction['output_sha256'][p.name]
        raw[role] = np.load(p, allow_pickle=False) if role == 'depth' else cv2.imread(str(p), cv2.IMREAD_UNCHANGED)
        assert arrsha(raw[role]) == newq['retained_pixel_sha256'][role+'_sha256']
    g = new['replay_geometry']; points=np.asarray(g['object_points_base_m'],float)
    assert arrsha(points) == g['object_points_sha256']
    geometry = GeometryEstimate(True,'','',np.asarray(g['obb_center_base_m']),np.asarray(g['R_base_obb']),
        np.asarray(g['obb_size_xyz_m']),np.asarray(g['support_normal_base']),g['support_offset_m'],
        g['support_inlier_ratio'],points,g['source_mode'])
    T=np.asarray(new['snapshot_transform']['T_base_optical']); identity=TargetTrackIdentity(**newq['source_target_identity'])
    captures=[]
    def profile(frame,event,arg):
        if event == 'return' and frame.f_code.co_filename == box.__file__ and frame.f_code.co_name == '_qualify_regular_box_top':
            captures.append(dict(frame.f_locals))
    sys.setprofile(profile)
    current=box.qualify_regular_box_top(raw['depth'],raw['mask'],K,scale,T,geometry,identity,stamp,geometry.source_mode,color_bgr=raw['color'])
    sys.setprofile(None)
    assert not current.ok and current.code == newq['code']
    save('current_qualification_actual.json',dict(ok=current.ok,code=current.code,audit=dict(current.audit),historical=newq))
    actual_outer=current.audit['outer_rgb_interval']; historical_outer=newq['outer_rgb_interval']
    assert actual_outer['unsupported_edge'] == historical_outer['unsupported_edge']
    assert abs(actual_outer['maximum_edge_distance_px']-historical_outer['maximum_edge_distance_px']) < 1e-4
    assert actual_outer['top_side_search']['choices_per_edge'] == historical_outer['top_side_search']['choices_per_edge']
    state=captures[-1]; origin=state['origin']; tn=state['tn']; axes=state['axes']; pixel=state['pixel_size']; rms=state['rms']
    color=raw['color']; mask=raw['mask']>0; depth=raw['depth']; top=state['top']
    camera_center=T[:3,3]
    def on_plane(uv):
        ray=np.column_stack(((uv[:,0]-k[2])/k[0],(uv[:,1]-k[3])/k[1],np.ones(len(uv))))@T[:3,:3].T
        return camera_center+ray*(float((origin-camera_center)@tn)/(ray@tn))[:,None]
    def project_metric(xy, A):
        p=xy@A[:,:2].T; p+=np.outer(((origin-p)@tn)/(tn@A[:,2]),A[:,2])
        return box._project(p,T,k)
    yy,xx=np.where(mask)
    edges=cv2.Canny(cv2.cvtColor(color,cv2.COLOR_BGR2GRAY),50.,100.)
    edges[cv2.dilate(mask.astype(np.uint8),np.ones((5,5),np.uint8))==0]=0
    ey,ex=np.where(edges); edgepixels=np.column_stack((ex,ey))
    min_length=max(12,int(.35*min(np.ptp(xx),np.ptp(yy))))
    found=cv2.HoughLinesP(edges,1.,np.pi/180.,max(12,min_length//2),minLineLength=min_length,maxLineGap=4)
    lines=sorted(found[:,0].tolist(), key=lambda r:(-math.hypot(r[2]-r[0],r[3]-r[1]),tuple(r)))[:48]
    def analyze_axes(A, label):
        coord=top@A[:,:2]; lower,upper=coord.min(0),coord.max(0)
        silhouette=on_plane(np.column_stack((xx,yy)))@A[:,:2]; lo,hi=silhouette.min(0),silhouette.max(0)
        corners=np.array([[lo[0],lo[1]],[hi[0],lo[1]],[hi[0],hi[1]],[lo[0],hi[1]]]); pixels=project_metric(corners,A)
        distances=[]
        for i in range(4):
            a,b=pixels[i],pixels[(i+1)%4]; uv=a+np.outer(np.linspace(.08,.92,101),b-a)
            nearest=np.min(np.linalg.norm(uv[:,None,:]-edgepixels[None,:,:],axis=2),axis=1)
            index=int(np.argmax(nearest)); distances.append(dict(edge=i,maximum_distance_px=float(nearest[index]),at_pixel=uv[index],pass_original_sqrt8_gate=bool(nearest[index]<=math.sqrt(8.))))
        limits=((1,lower[1]),(0,upper[0]),(1,upper[1]),(0,lower[0])); rows=[]
        for line in lines:
            pix=np.asarray(line,float).reshape(2,2); metric=on_plane(pix)@A[:,:2]; delta=metric[1]-metric[0]; candidates=[]
            for i,(across,limit) in enumerate(limits):
                along=1-across; angle=math.degrees(math.atan2(abs(delta[across]),abs(delta[along])))
                length=abs(delta[along]); offset=float(np.mean(metric[:,across])); overlap=min(float(metric[:,along].max()),upper[along])-max(float(metric[:,along].min()),lower[along])
                gates=dict(length=bool(length>=.35*(upper[along]-lower[along])),angle_8deg=bool(angle<=8.),offset=bool(abs(offset-limit)<=max(.012,8.*pixel)),overlap=bool(overlap>=.35*(upper[along]-lower[along])))
                candidates.append(dict(edge=i,angle_from_axis_deg=angle,offset_from_depth_limit_m=offset-limit,overlap_m=overlap,gates=gates,pre_depth_bands_pass=all(gates.values())))
            rows.append(dict(line_pixels_xy=pix,length_px=float(np.linalg.norm(pix[1]-pix[0])),edges=candidates))
        props,search=outer._top_side_proposals(color,mask,depth,scale,T,k,origin,tn,A,pixel)
        out=outer.infer_outer_rgb_rectangle(color,mask,depth,scale,T,k,origin,tn,A,pixel,rms)
        visual=box._visual_rectangle(color,mask,depth,scale,T,k,origin,tn,A,lower,upper,pixel)
        result=dict(label=label,diagnostic_only=True,axes_base=A,top_extent_m=upper-lower,
            mask_envelope_corners_px=pixels,mask_envelope_distances=distances,hough_lines=rows,
            top_side_search=search,proposal_count=len(props),outer_result=out,
            visual_found=visual is not None,visual_result=visual)
        image=color.copy();cv2.polylines(image,[np.rint(pixels).astype(np.int32)],True,(255,0,255),1)
        for j,p in enumerate(pixels):cv2.putText(image,str(j),tuple(np.rint(p).astype(int)),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,0,255),1)
        for j,row in enumerate(rows):
            a,b=np.rint(row['line_pixels_xy']).astype(int);cv2.line(image,tuple(a),tuple(b),(255,255,0),1)
            cv2.putText(image,str(j),tuple((a+b)//2),cv2.FONT_HERSHEY_SIMPLEX,.35,(0,0,255),1)
        cv2.imwrite(str(O/(label+'_lines.png')),image)
        save(label+'.json',result)
        return result
    actual=analyze_axes(axes,'current_depth_axes')
    old_axes=np.asarray(oldtrigger['regular_box_planning_enclosure']['planning_axes_base'])
    reference=np.asarray(new['multiview_surface']['latest_registration']['snapshot_to_reference_transform'])
    old_current=np.linalg.inv(reference)[:3,:3]@old_axes
    ax=old_current[:,0]-axes[:,2]*float(old_current[:,0]@axes[:,2]);ax/=np.linalg.norm(ax)
    prior_axes=np.column_stack((ax,np.cross(axes[:,2],ax),axes[:,2]))
    prior=analyze_axes(prior_axes,'registered_old_axis_proposal')
    yaw=math.degrees(math.acos(np.clip(abs(axes[:,0]@prior_axes[:,0]),-1.,1.)))
    # Sensitivity only: trimming is not a production proposal or evidence edit.
    centered=top@axes[:,:2]; lower=np.quantile(centered,.01,axis=0);upper=np.quantile(centered,.99,axis=0)
    take=np.all((centered>=lower)&(centered<=upper),axis=1); B=box._basis(axes[:,2]); fit=cv2.minAreaRect((top[take]@B).astype(np.float32)); vertices=cv2.boxPoints(fit).astype(float)
    ea,eb=vertices[1]-vertices[0],vertices[2]-vertices[1];ea=eb if np.linalg.norm(eb)>np.linalg.norm(ea) else ea
    ax=B@(ea/np.linalg.norm(ea));ax*=1 if ax@axes[:,0]>=0 else -1
    trimmed_axes=np.column_stack((ax,np.cross(axes[:,2],ax),axes[:,2]))
    trimmed=analyze_axes(trimmed_axes,'diagnostic_trim1pct_axis_proposal')
    # Record exactly which raw valid top samples determine the hull and fit.
    xy=top@B; hull_idx=cv2.convexHull(xy.astype(np.float32),returnPoints=False).reshape(-1)
    top_uv=state['uv'][state['keep']]
    hullrows=[dict(index=int(j),pixel=top_uv[j],base_point=top[j],depth_raw=int(depth[top_uv[j,1],top_uv[j,0]]),coord=xy[j]) for j in hull_idx]
    overlay=color.copy()
    for uv in top_uv: overlay[int(uv[1]),int(uv[0])]=(0,255,0)
    for row in hullrows:cv2.circle(overlay,tuple(row['pixel'].astype(int)),2,(255,0,255),1)
    cv2.imwrite(str(O/'measured_top_hull.png'),overlay)
    np.savez_compressed(O/'exact_current_top_fit.npz',points=state['points'],uv=state['uv'],keep=state['keep'],top=top,origin=origin,tn=tn,axes=axes,prior_axes=prior_axes,trimmed_axes=trimmed_axes)
    save('current_fit_support.json',dict(top_count=len(top),raw_valid_count=len(state['points']),mask_pixels=int(mask.sum()),valid_fraction=float(state['valid_fraction']),origin=origin,tn=tn,axes=axes,min_area_rect=state['rect'],hull=hullrows,
         prior_axis_separation_deg=yaw,trimmed_axis_separation_deg=math.degrees(math.acos(np.clip(abs(axes[:,0]@trimmed_axes[:,0]),-1.,1.))),trimmed_count=int(take.sum()),trimmed_warning='1% trim is a diagnostic sensitivity probe only; all actual depth is untouched and must remain evidence.'))
    after={p:sha(W/p) for p in before};save('source_sha256_after.json',after)
    assert before==after
    boundary=O/'boundary.jsonl';attempts=boundary.read_text().splitlines() if boundary.exists() else [];assert not attempts
    report.update(passed=True,source_count=len(before),source_unchanged=True,inputs_sha256={str(p):sha(p) for p in (G/'manifest.json',I/'receipt.json',camera_path,G/'keyframes_audits'/old['observe_return'].get('audit_sha256','cd24be7b05a0d9141658dd05b8c032df5b443399da5c3622f158ade48c0bba11.json')) if p.exists()},
        source_stamp_ns=stamp,source_binding_verified=True,historical_outer_rejection_reproduced=True,historical_edge_distance_delta_px=actual_outer['maximum_edge_distance_px']-historical_outer['maximum_edge_distance_px'],current_code=current.code,current_audit=dict(current.audit),
        current_top_axes=axes,registered_old_axes=prior_axes,prior_axis_separation_deg=yaw,prior_search=prior['top_side_search'],prior_outer_ok=prior['outer_result']['ok'],prior_outer_reason=prior['outer_result'].get('reason'),prior_visual_found=prior['visual_found'],
        old_model_sha256=oldq['model_sha256'],old_inferred_size_xyz_m=oldq['inferred_size_xyz_m'],old_model_preserved_uncertainty=oldtrigger['missing_surface_direction']['targeted_view'],
        latest_registration=new['multiview_surface']['latest_registration'],external_boundary_attempts=attempts,script_sha256=sha(__file__),
        limitations=['Only exact retained current source and audit geometry qualification replayed; no temporal admission replay, candidate materialization or collision/motion.',
        'Old model axis is transformed by archived current-to-phase-reference inverse, projected onto current verified support solely for diagnostic proposal; this is not a validated multi-view fused model.',
        'Registered old-axis and 1% trimmed-axis probes do not reduce uncertainty and do not authorize contact or motion.'])
except BaseException as exc:
    sys.setprofile(None)
    report.update(error_type=type(exc).__name__,error=str(exc),traceback=traceback.format_exc())
finally:
    save('result.json',report)
    print(json.dumps(dict(output=str(O),passed=report['passed'],error=report.get('error'),current_code=report.get('current_code'),prior_angle_deg=report.get('prior_axis_separation_deg'),prior_search=report.get('prior_search'),prior_outer_ok=report.get('prior_outer_ok')),ensure_ascii=False))
    if not report['passed']:sys.exit(1)
